// ---------------------------------------------------------------------------
//! @title pcie_config_handler
//! @author Idris Somoye
//! Completes CfgRd0 and CfgWr0 requests from the configuration registers.
//
// Purpose
//   Takes one request header at a time from pcie_config_decode. A CfgRd0
//   becomes one AXI4-Lite read of pcie_config_reg, answered with a CplD
//   carrying the Dword read. A CfgWr0 becomes one AXI4-Lite write, answered
//   with a Cpl, and updates the captured Bus, Device and Function Numbers.
//
// Interfaces
//   Request       rx_tlp_*: from pcie_config_decode, taken in ST_IDLE.
//                 rx_tlp_strb and rx_tlp_error are not read.
//   Registers     s_axil_*: AXI4-Lite manager to pcie_config_reg, AW before
//                 W. s_axil_bresp and s_axil_rresp are not read.
//   Captured ID   cfg_bus_number_o, cfg_device_number_o,
//                 cfg_function_number_o: header bytes 8 and 9 of the last
//                 CfgWr0; 0 after reset.
//   Completion    cpl_axis_*: through a skid buffer, one Dword per beat,
//                 header byte 0 in bits 7:0 of the first beat.
//
// Clock and reset
//   clk_i only. rst_i is synchronous and active high.
//
// Limitations
//   DATA_WIDTH must be 32. The Completion Status is always Successful
//   Completion. s_axil_wstrb is 1111b, so the request's First DW Byte
//   Enables are not applied, and s_axil_wdata is 0 (see pcie_config_decode).
//   The Cpl carries Byte Count 0 (gen_cpl), where PCIe Base Spec r2.1,
//   §2.2.9 requires 4. The Completer ID is the request's Bus, Device and
//   Function Numbers, where §2.2.9 requires the captured Bus and Device
//   Numbers, and 0s before the first CfgWr0. ST_WAIT_WR is never entered.
//
// References
//   PCIe Base Spec r2.1, §2.2.6.2
//   PCIe Base Spec r2.1, §2.2.7
//   PCIe Base Spec r2.1, §2.2.9
//   PCIe Base Spec r2.1, §7.2.2
// ---------------------------------------------------------------------------
module pcie_config_handler
  import pcie_datalink_pkg::*;
  import pcie_tlp_pkg::*;
#(
    parameter int DATA_WIDTH     = 32,
    parameter int STRB_WIDTH     = DATA_WIDTH / 8,
    parameter int KEEP_WIDTH     = STRB_WIDTH,
    parameter int USER_WIDTH     = 1,
    parameter int TLP_SEG_COUNT  = 1,
    parameter int TLP_DATA_WIDTH = 128,
    parameter int TLP_STRB_WIDTH = 5,
    parameter int TLP_HDR_WIDTH  = 128

) (
    input  logic                                   clk_i,
    input  logic                                   rst_i,

    // ---- request header, from pcie_config_decode ---------------------------
    input  logic [             TLP_DATA_WIDTH-1:0] rx_tlp_data,
    input  logic [             TLP_STRB_WIDTH-1:0] rx_tlp_strb,
    input  logic [TLP_SEG_COUNT*TLP_HDR_WIDTH-1:0] rx_tlp_hdr,
    input  logic [            TLP_SEG_COUNT*4-1:0] rx_tlp_error,
    input  logic [              TLP_SEG_COUNT-1:0] rx_tlp_valid,
    input  logic [              TLP_SEG_COUNT-1:0] rx_tlp_sop,
    input  logic [              TLP_SEG_COUNT-1:0] rx_tlp_eop,
    output logic                                   rx_tlp_ready,

    // ---- AXI4-Lite manager, to pcie_config_reg -----------------------------
    output logic        s_axil_awvalid,
    input  logic        s_axil_awready,
    output logic [31:0] s_axil_awaddr,
    output logic        s_axil_wvalid,
    input  logic        s_axil_wready,
    output logic [31:0] s_axil_wdata,
    output logic [ 3:0] s_axil_wstrb,
    output logic        s_axil_arvalid,
    input  logic        s_axil_arready,
    output       [31:0] s_axil_araddr,
    input  logic        s_axil_rvalid,
    output logic        s_axil_rready,
    input  logic [31:0] s_axil_rdata,
    input  logic [ 1:0] s_axil_rresp,
    input  logic        s_axil_bvalid,
    output logic        s_axil_bready,
    input  logic [ 1:0] s_axil_bresp,

    // ---- captured from each CfgWr0 -----------------------------------------

    output logic [7:0] cfg_bus_number_o,
    output logic [4:0] cfg_device_number_o,
    output logic [2:0] cfg_function_number_o,

    // ---- Cpl and CplD, to the transmit path --------------------------------
    output logic [(DATA_WIDTH)-1:0] cpl_axis_tdata,
    output logic [(KEEP_WIDTH)-1:0] cpl_axis_tkeep,
    output logic                    cpl_axis_tvalid,
    output logic                    cpl_axis_tlast,
    output logic [  USER_WIDTH-1:0] cpl_axis_tuser,
    input  logic                    cpl_axis_tready
);



  typedef enum logic [4:0] {
    ST_IDLE,          // waits for a request header
    ST_CFG_RD,        // read address (AR)
    ST_CFG_WR,        // write address (AW)
    ST_CFG_WR_DATA,   // write data (W)
    ST_CFG_WR_ACK,    // write response (B); builds the Cpl
    ST_WAIT_RD,       // read data (R); builds the CplD
    ST_WAIT_WR,       // never entered
    ST_SEND_CPL_TLP   // sends the completion, one Dword per beat
  } axis_pcie_conv_t;

  typedef struct {

    axis_pcie_conv_t           state;
    tlp_hdr_union_t            tlp_hdr;
    logic [31:0]               word_count;
    cpl_tlp_hdr_t              cpl_tlp;
    logic [7:0]                cfg_bus_number;
    logic [4:0]                cfg_device_number;
    logic [2:0]                cfg_function_number;
    // tlp_dw0 and the four tlp_is_* flags are not read.
    pcie_tlp_header_dw0_t      tlp_dw0;
    logic                      tlp_is_3dw;
    logic                      tlp_is_sop;
    logic                      tlp_is_pd;
    logic                      tlp_is_eop;
    logic [TLP_DATA_WIDTH-1:0] tlp_data;
    logic [31:0]               length;
  } fsm_struct_t;

  fsm_struct_t D, Q;


  // s_axis_*: the input of the completion skid buffer.
  logic [DATA_WIDTH-1:0] s_axis_tdata;
  logic [KEEP_WIDTH-1:0] s_axis_tkeep;
  logic                  s_axis_tvalid;
  logic                  s_axis_tlast;
  logic [USER_WIDTH-1:0] s_axis_tuser;
  logic                  s_axis_tready;
  logic [          31:0] address;

  assign cfg_bus_number_o      = Q.cfg_bus_number;
  assign cfg_device_number_o   = Q.cfg_device_number;
  assign cfg_function_number_o = Q.cfg_function_number;



  always_ff @(posedge clk_i) begin : main_seq
    if (rst_i) begin
      Q <= '{state: ST_IDLE, default: 'd0};
    end else begin
      Q <= D;
    end
  end

  // The 40-bit concatenation truncates to 32 bits, dropping the leading 1'b1
  // and 7'h0. From header bytes 10 and 11, address[11:8] is the Extended
  // Register Number and address[7:2] the Register Number: the Dword's byte
  // address in configuration space (PCIe Base Spec r2.1, §7.2.2).
  assign address = {
    1'b1,
    7'h0,
    Q.tlp_hdr.struct_.word_2.byte_0,
    Q.tlp_hdr.struct_.word_2.byte_1,
    Q.tlp_hdr.struct_.word_2.byte_2,
    Q.tlp_hdr.struct_.word_2.byte_3[7:0]
  };
  assign s_axil_awaddr = address;
  assign s_axil_wdata = Q.tlp_data[31:0];
  assign s_axil_wstrb = 4'b1111;
  assign s_axil_araddr = address;


  always_comb begin : main_combo
    D             = Q;
    D.tlp_dw0     = '0;
    s_axis_tdata  = '0;
    s_axis_tkeep  = '0;
    s_axis_tvalid = '0;
    s_axis_tlast  = '0;
    s_axis_tuser  = '0;
    rx_tlp_ready  = '0;
    s_axil_awvalid = 1'b0;
    s_axil_wvalid = 1'b0;
    s_axil_arvalid = 1'b0;
    s_axil_rready = 1'b0;
    s_axil_bready = 1'b0;
    case (Q.state)
      // pcie_config_mux sends only CfgRd0 and CfgWr0 toward this module; any
      // other header is accepted here and dropped without a completion.
      ST_IDLE: begin
        rx_tlp_ready = '1;
        if (rx_tlp_valid && rx_tlp_sop) begin
          D.tlp_hdr.whole_ = rx_tlp_hdr;
          D.tlp_data       = rx_tlp_data;
          D.tlp_is_eop     = rx_tlp_eop;
          D.word_count     = '0;
          if (D.tlp_hdr.struct_.word_0.byte0 == CfgRd0) begin
            s_axis_tlast = '1;
            D.state = ST_CFG_RD;
          end else if (D.tlp_hdr.struct_.word_0.byte0 == CfgWr0) begin
            D.state = ST_CFG_WR;
          end
        end
      end
      ST_CFG_WR: begin
        s_axil_awvalid = 1'b1;
        s_axil_wvalid = 1'b0;
        s_axil_arvalid = 1'b0;
        s_axil_rready = 1'b0;
        s_axil_bready = 1'b0;
        // A Function captures the Bus and Device Numbers from each CfgWr0 it
        // completes (PCIe Base Spec r2.1, §2.2.6.2). pcie_endpoint_top gives
        // the captured numbers to tlp_layer as Requester ID and Completer ID.
        {D.cfg_bus_number, D.cfg_device_number, D.cfg_function_number} = {
          Q.tlp_hdr.struct_.word_2.byte_0, Q.tlp_hdr.struct_.word_2.byte_1
        };
        if (s_axil_awready == 1'b1) begin
          D.state = ST_CFG_WR_DATA;
        end else begin
          D.state = ST_CFG_WR;
        end
      end 

      ST_CFG_WR_DATA: begin
        s_axil_awvalid = 1'b0;
        s_axil_wvalid = 1'b1;
        s_axil_arvalid = 1'b0;
        s_axil_rready = 1'b0;
        s_axil_bready = 1'b0;
        {D.cfg_bus_number, D.cfg_device_number, D.cfg_function_number} = {
          Q.tlp_hdr.struct_.word_2.byte_0, Q.tlp_hdr.struct_.word_2.byte_1
        };
        if (s_axil_wready == 1'b1) begin
          D.state = ST_CFG_WR_ACK;
        end else begin
          D.state = ST_CFG_WR_DATA;
        end
      end
      ST_CFG_WR_ACK: begin
        s_axil_awvalid = 1'b0;
        s_axil_wvalid  = 1'b0;
        s_axil_arvalid = 1'b0;
        s_axil_rready  = 1'b0;
        s_axil_bready  = 1'b1;
        if (s_axil_bvalid == 1'b1) begin
          D.cpl_tlp    = gen_cpl(Q.tlp_hdr, s_axil_rdata);
          D.word_count = '0;
          D.length     = 32'd2;
          D.state      = ST_SEND_CPL_TLP;
        end else begin
          D.state = ST_CFG_WR_ACK;
        end
      end

      ST_CFG_RD: begin
        s_axil_awvalid = 1'b0;
        s_axil_wvalid  = 1'b0;
        s_axil_arvalid = 1'b1;
        s_axil_rready  = 1'b0;
        s_axil_bready  = 1'b0;
        if (s_axil_arready == 1'b1) begin
          D.state = ST_WAIT_RD; 
        end else begin
          D.state = ST_CFG_RD;
        end
      end  

      // cpl_tlp is rebuilt every cycle here and keeps the value from the
      // cycle s_axil_rvalid is high.
      ST_WAIT_RD: begin
        D.cpl_tlp      = gen_cpld(Q.tlp_hdr, s_axil_rdata);
        s_axil_awvalid = 1'b0;
        s_axil_wvalid  = 1'b0;
        s_axil_arvalid = 1'b0;
        s_axil_rready  = 1'b1;
        s_axil_bready  = 1'b0;
        if (s_axil_rvalid == 1'b1) begin
          D.word_count = '0;
          D.length     = 32'd3;
          D.state      = ST_SEND_CPL_TLP;
        end  
      end

      ST_SEND_CPL_TLP: begin
        s_axis_tdata  = Q.cpl_tlp[(32*Q.word_count)+:32];
        s_axis_tkeep  = '1;
        s_axis_tvalid = '1;
        s_axis_tlast  = '0;
        s_axis_tuser  = 8'h2;
        if (s_axis_tready) begin
          D.word_count = Q.word_count + 1'b1;
          // Q.length is the index of the last Dword: 2 for a Cpl (three
          // header Dwords), 3 for a CplD (header and one data Dword).
          if ((Q.word_count >= Q.length)) begin
            s_axis_tlast = '1;
            D.state      = ST_IDLE;
          end
        end

      end
      default: begin

      end
    endcase
  end


  //axis skid buffer
  axis_register #(
      .DATA_WIDTH (DATA_WIDTH),
      .KEEP_ENABLE('1),
      .KEEP_WIDTH (KEEP_WIDTH),
      .LAST_ENABLE('1),
      .ID_ENABLE  ('0),
      .ID_WIDTH   (1),
      .DEST_ENABLE('0),
      .DEST_WIDTH (1),
      .USER_ENABLE('1),
      .USER_WIDTH (USER_WIDTH),
      .REG_TYPE   (SkidBuffer)
  ) axis_register_pipeline_inst (
      .clk          (clk_i),
      .rst          (rst_i),
      .s_axis_tdata (s_axis_tdata),
      .s_axis_tkeep (s_axis_tkeep),
      .s_axis_tvalid(s_axis_tvalid),
      .s_axis_tready(s_axis_tready),
      .s_axis_tlast (s_axis_tlast),
      .s_axis_tuser (s_axis_tuser),
      .s_axis_tid   ('0),
      .s_axis_tdest ('0),
      .m_axis_tdata (cpl_axis_tdata),
      .m_axis_tkeep (cpl_axis_tkeep),
      .m_axis_tvalid(cpl_axis_tvalid),
      .m_axis_tready(cpl_axis_tready),
      .m_axis_tlast (cpl_axis_tlast),
      .m_axis_tuser (cpl_axis_tuser),
      .m_axis_tid   (),
      .m_axis_tdest ()
  );

endmodule
