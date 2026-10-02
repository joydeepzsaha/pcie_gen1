// ---------------------------------------------------------------------------
// pcie_cfg_wrapper -- configuration request path and configuration registers
//
// Purpose
//   Sits on the received TLP stream inside dllp_receive and completes Type 0
//   Configuration Requests from the configuration registers. pcie_config_mux
//   passes every TLP to m_tlp_axis_* and also copies each CfgRd0 and CfgWr0
//   to pcie_config_decode, which hands the header to pcie_config_handler.
//   The handler reads or writes one Dword of pcie_config_reg over AXI4-Lite,
//   sends the Cpl or CplD on cpl_axis_*, and captures the Bus, Device and
//   Function Numbers of each CfgWr0.
//
// Interfaces
//   TLP input     s_axis_*: received TLPs from dllp2tlp.
//   TLP output    m_tlp_axis_*: every received TLP, configuration requests
//                 included.
//   Completion    cpl_axis_*: one Cpl per CfgWr0, one CplD per CfgRd0.
//   Captured ID   cfg_bus_number_o, cfg_device_number_o,
//                 cfg_function_number_o: from the last CfgWr0.
//   Registers     hwif_in, hwif_out: pcie_config_reg's hardware interface.
//
// Clock and reset
//   clk_i only. rst_i is synchronous and active high. pcie_datalink_layer
//   also asserts it while the link is down, which returns the registers and
//   the captured numbers to their reset values.
//
// Limitations
//   DATA_WIDTH must be 32: pcie_config_decode and pcie_config_handler move
//   one header Dword per beat. Every CfgWr0 writes 0 (see
//   pcie_config_decode).
//
// References
//   PCIe Base Spec r2.1, §2.2.6.2
//   PCIe Base Spec r2.1, §7.2
// ---------------------------------------------------------------------------
module pcie_cfg_wrapper
  import pcie_datalink_pkg::*;
  import pcie_tlp_pkg::*;
  import pcie_config_reg_pkg::*;
#(
    parameter int DATA_WIDTH     = 32,
    parameter int STRB_WIDTH     = DATA_WIDTH / 8,
    parameter int KEEP_WIDTH     = STRB_WIDTH,
    parameter int USER_WIDTH     = 1,
    // Widths of the rx_tlp_* interface between pcie_config_decode and
    // pcie_config_handler.
    parameter int TLP_SEG_COUNT  = 1,
    parameter int TLP_DATA_WIDTH = 128,
    parameter int TLP_STRB_WIDTH = 5,
    parameter int TLP_HDR_WIDTH  = 128

) (
    input  logic                    clk_i,
    input  logic                    rst_i,

    // ---- received TLPs, from dllp2tlp --------------------------------------
    input  logic [  DATA_WIDTH-1:0] s_axis_tdata,
    input  logic [  KEEP_WIDTH-1:0] s_axis_tkeep,
    (* mark_debug = "true", keep = "true" *) input  logic                    s_axis_tvalid,
    input  logic                    s_axis_tlast,
    input  logic [  USER_WIDTH-1:0] s_axis_tuser,
    output logic                    s_axis_tready,

    // ---- completions for CfgRd0 and CfgWr0 ---------------------------------
    output logic [(DATA_WIDTH)-1:0] cpl_axis_tdata,
    output logic [(KEEP_WIDTH)-1:0] cpl_axis_tkeep,
    output logic                    cpl_axis_tvalid,
    output logic                    cpl_axis_tlast,
    output logic [  USER_WIDTH-1:0] cpl_axis_tuser,
    input  logic                    cpl_axis_tready,

    // ---- every received TLP, on toward the Transaction Layer ---------------
    output logic [  DATA_WIDTH-1:0] m_tlp_axis_tdata,
    output logic [  KEEP_WIDTH-1:0] m_tlp_axis_tkeep,
    output logic                    m_tlp_axis_tvalid,
    output logic                    m_tlp_axis_tlast,
    output logic [  USER_WIDTH-1:0] m_tlp_axis_tuser,
    input  logic                    m_tlp_axis_tready,

    // ---- Bus, Device and Function Numbers of the last CfgWr0 ---------------
    output logic [7:0] cfg_bus_number_o,
    output logic [4:0] cfg_device_number_o,
    output logic [2:0] cfg_function_number_o,


    // ---- pcie_config_reg hardware interface --------------------------------
    input  pcie_config_reg__in_t  hwif_in,
    output pcie_config_reg__out_t hwif_out
);

  // pcie_config_decode to pcie_config_handler: one request header per
  // transfer.
  logic [             TLP_DATA_WIDTH-1:0] rx_tlp_data;
  logic [             TLP_STRB_WIDTH-1:0] rx_tlp_strb;
  logic [TLP_SEG_COUNT*TLP_HDR_WIDTH-1:0] rx_tlp_hdr;
  logic [            TLP_SEG_COUNT*4-1:0] rx_tlp_error;
  logic [              TLP_SEG_COUNT-1:0] rx_tlp_valid;
  logic [              TLP_SEG_COUNT-1:0] rx_tlp_sop;
  logic [              TLP_SEG_COUNT-1:0] rx_tlp_eop;
  logic                                   rx_tlp_ready;


  // pcie_config_handler to pcie_config_reg (AXI4-Lite). s_axil_awprot and
  // s_axil_arprot have no driver; pcie_config_reg does not read them.
  logic                                   s_axil_awready;
  logic                                   s_axil_awvalid;
  logic [                           31:0] s_axil_awaddr;
  logic [                            2:0] s_axil_awprot;
  logic                                   s_axil_wready;
  logic                                   s_axil_wvalid;
  logic [                           31:0] s_axil_wdata;
  logic [                            3:0] s_axil_wstrb;
  logic                                   s_axil_bready;
  logic                                   s_axil_bvalid;
  logic [                            1:0] s_axil_bresp;
  logic                                   s_axil_arready;
  logic                                   s_axil_arvalid;
  logic [                           31:0] s_axil_araddr;
  logic [                            2:0] s_axil_arprot;
  logic                                   s_axil_rready;
  logic                                   s_axil_rvalid;
  logic [                           31:0] s_axil_rdata;
  logic [                            1:0] s_axil_rresp;


  // pcie_config_mux to pcie_config_decode: CfgRd0 and CfgWr0 only.
  logic [                 DATA_WIDTH-1:0] m_cfg_axis_tdata;
  logic [                 KEEP_WIDTH-1:0] m_cfg_axis_tkeep;
  logic                                   m_cfg_axis_tvalid;
  logic                                   m_cfg_axis_tlast;
  logic [                 USER_WIDTH-1:0] m_cfg_axis_tuser;
  logic                                   m_cfg_axis_tready;


  pcie_config_decode #(
      .DATA_WIDTH    (DATA_WIDTH),
      .STRB_WIDTH    (STRB_WIDTH),
      .KEEP_WIDTH    (KEEP_WIDTH),
      .USER_WIDTH    (USER_WIDTH),
      .TLP_SEG_COUNT (TLP_SEG_COUNT),
      .TLP_DATA_WIDTH(TLP_DATA_WIDTH),
      .TLP_STRB_WIDTH(TLP_STRB_WIDTH),
      .TLP_HDR_WIDTH (TLP_HDR_WIDTH)
  ) pcie_config_decode_inst (
      .clk_i        (clk_i),
      .rst_i        (rst_i),
      .s_axis_tdata (m_cfg_axis_tdata),
      .s_axis_tkeep (m_cfg_axis_tkeep),
      .s_axis_tvalid(m_cfg_axis_tvalid),
      .s_axis_tlast (m_cfg_axis_tlast),
      .s_axis_tuser (m_cfg_axis_tuser),
      .s_axis_tready(m_cfg_axis_tready),
      .rx_tlp_data  (rx_tlp_data),
      .rx_tlp_strb  (rx_tlp_strb),
      .rx_tlp_hdr   (rx_tlp_hdr),
      .rx_tlp_error (rx_tlp_error),
      .rx_tlp_valid (rx_tlp_valid),
      .rx_tlp_sop   (rx_tlp_sop),
      .rx_tlp_eop   (rx_tlp_eop),
      .rx_tlp_ready (rx_tlp_ready)
  );

  pcie_config_mux #(
      .DATA_WIDTH    (DATA_WIDTH),
      .STRB_WIDTH    (STRB_WIDTH),
      .KEEP_WIDTH    (KEEP_WIDTH),
      .USER_WIDTH    (USER_WIDTH),
      .TLP_SEG_COUNT (TLP_SEG_COUNT),
      .TLP_DATA_WIDTH(TLP_DATA_WIDTH),
      .TLP_STRB_WIDTH(TLP_STRB_WIDTH),
      .TLP_HDR_WIDTH (TLP_HDR_WIDTH)
  ) pcie_config_mux_inst (
      .clk_i            (clk_i),
      .rst_i            (rst_i),
      .s_axis_tdata     (s_axis_tdata),
      .s_axis_tkeep     (s_axis_tkeep),
      .s_axis_tvalid    (s_axis_tvalid),
      .s_axis_tlast     (s_axis_tlast),
      .s_axis_tuser     (s_axis_tuser),
      .s_axis_tready    (s_axis_tready),
      .m_cfg_axis_tdata (m_cfg_axis_tdata),
      .m_cfg_axis_tkeep (m_cfg_axis_tkeep),
      .m_cfg_axis_tvalid(m_cfg_axis_tvalid),
      .m_cfg_axis_tlast (m_cfg_axis_tlast),
      .m_cfg_axis_tuser (m_cfg_axis_tuser),
      .m_cfg_axis_tready(m_cfg_axis_tready),
      .m_tlp_axis_tdata (m_tlp_axis_tdata),
      .m_tlp_axis_tkeep (m_tlp_axis_tkeep),
      .m_tlp_axis_tvalid(m_tlp_axis_tvalid),
      .m_tlp_axis_tlast (m_tlp_axis_tlast),
      .m_tlp_axis_tuser (m_tlp_axis_tuser),
      .m_tlp_axis_tready(m_tlp_axis_tready)
  );

  pcie_config_handler #(
      .DATA_WIDTH    (DATA_WIDTH),
      .STRB_WIDTH    (STRB_WIDTH),
      .KEEP_WIDTH    (KEEP_WIDTH),
      .USER_WIDTH    (USER_WIDTH),
      .TLP_SEG_COUNT (TLP_SEG_COUNT),
      .TLP_DATA_WIDTH(TLP_DATA_WIDTH),
      .TLP_STRB_WIDTH(TLP_STRB_WIDTH),
      .TLP_HDR_WIDTH (TLP_HDR_WIDTH)
  ) pcie_config_handler_inst (
      .clk_i                (clk_i),
      .rst_i                (rst_i),
      .rx_tlp_data          (rx_tlp_data),
      .rx_tlp_strb          (rx_tlp_strb),
      .rx_tlp_hdr           (rx_tlp_hdr),
      .rx_tlp_error         (rx_tlp_error),
      .rx_tlp_valid         (rx_tlp_valid),
      .rx_tlp_sop           (rx_tlp_sop),
      .rx_tlp_eop           (rx_tlp_eop),
      .rx_tlp_ready         (rx_tlp_ready),
      .s_axil_awvalid       (s_axil_awvalid),
      .s_axil_awready       (s_axil_awready),
      .s_axil_awaddr        (s_axil_awaddr),
      .s_axil_wvalid        (s_axil_wvalid),
      .s_axil_wready        (s_axil_wready),
      .s_axil_wdata         (s_axil_wdata),
      .s_axil_wstrb         (s_axil_wstrb),
      .s_axil_arvalid       (s_axil_arvalid),
      .s_axil_arready       (s_axil_arready),
      .s_axil_araddr        (s_axil_araddr),
      .s_axil_rvalid        (s_axil_rvalid),
      .s_axil_rready        (s_axil_rready),
      .s_axil_rdata         (s_axil_rdata),
      .s_axil_rresp         (s_axil_rresp),
      .s_axil_bvalid        (s_axil_bvalid),
      .s_axil_bready        (s_axil_bready),
      .s_axil_bresp         (s_axil_bresp),
      .cfg_bus_number_o     (cfg_bus_number_o),
      .cfg_device_number_o  (cfg_device_number_o),
      .cfg_function_number_o(cfg_function_number_o),

      .cpl_axis_tdata (cpl_axis_tdata),
      .cpl_axis_tkeep (cpl_axis_tkeep),
      .cpl_axis_tvalid(cpl_axis_tvalid),
      .cpl_axis_tlast (cpl_axis_tlast),
      .cpl_axis_tuser (cpl_axis_tuser),
      .cpl_axis_tready(cpl_axis_tready)
  );


  // Twelve address bits: a Function's configuration space is 4096 bytes
  // (PCIe Base Spec r2.1, §7.2). pcie_config_handler places the request's
  // Extended Register Number and Register Number in address bits 11:2.
  pcie_config_reg pcie_config_reg_inst (
      .clk           (clk_i),
      .rst           (rst_i),
      .s_axil_awready(s_axil_awready),
      .s_axil_awvalid(s_axil_awvalid),
      .s_axil_awaddr (s_axil_awaddr[11:0]),
      .s_axil_awprot (s_axil_awprot),
      .s_axil_wready (s_axil_wready),
      .s_axil_wvalid (s_axil_wvalid),
      .s_axil_wdata  (s_axil_wdata),
      .s_axil_wstrb  (s_axil_wstrb),
      .s_axil_bready (s_axil_bready),
      .s_axil_bvalid (s_axil_bvalid),
      .s_axil_bresp  (s_axil_bresp),
      .s_axil_arready(s_axil_arready),
      .s_axil_arvalid(s_axil_arvalid),
      .s_axil_araddr (s_axil_araddr[11:0]),
      .s_axil_arprot (s_axil_arprot),
      .s_axil_rready (s_axil_rready),
      .s_axil_rvalid (s_axil_rvalid),
      .s_axil_rdata  (s_axil_rdata),
      .s_axil_rresp  (s_axil_rresp),
      .hwif_in       (hwif_in),
      .hwif_out      (hwif_out)
  );


endmodule
