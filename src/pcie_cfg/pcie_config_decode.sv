// ---------------------------------------------------------------------------
//! @title pcie_config_decode
//! @author Idris Somoye
//! Collects the header of each configuration request for
//! pcie_config_handler.
//
// Purpose
//   Takes CfgRd0 and CfgWr0 TLPs from pcie_config_mux, one Dword per beat,
//   and assembles the header in Q.tlp_hdr. It then offers the header to
//   pcie_config_handler on rx_tlp_* as one transfer, with rx_tlp_sop and
//   rx_tlp_eop both set. A configuration request has a 3DW header (PCIe Base
//   Spec r2.1, §2.2.7), so only ST_IDLE, ST_TLP_HEADER_WORD_1,
//   ST_TLP_HEADER_WORD_2 and ST_TLP_SEND are entered. ST_TLP_HEADER_WORD_3
//   and ST_TLP_STREAM are reached only after a header that is not 3DW,
//   which pcie_config_mux never sends.
//
// Interfaces
//   Input         s_axis_*: CfgRd0 and CfgWr0 from pcie_config_mux, through a
//                 skid buffer; header byte 0 is in bits 7:0 of the first beat.
//   Request       rx_tlp_hdr: Q.tlp_hdr. rx_tlp_valid: high in ST_TLP_SEND.
//                 rx_tlp_sop, rx_tlp_eop: driven only while rx_tlp_ready is
//                 high. rx_tlp_data, rx_tlp_strb, rx_tlp_error: always 0.
//
// Clock and reset
//   clk_i only. rst_i is synchronous and active high.
//
// Limitations
//   DATA_WIDTH must be 32. A CfgWr0's payload never reaches rx_tlp_data:
//   ST_TLP_HEADER_WORD_2 goes straight to ST_TLP_SEND, ST_IDLE then accepts
//   and discards the payload beat, and tlp_tdata is never assigned from
//   Q.tlp_data. pcie_config_handler therefore writes 0 for every CfgWr0. A
//   first beat that carries tlast is accepted and discarded.
//   ST_TLP_HEADER_WORD_0 is declared and never entered.
//
// References
//   PCIe Base Spec r2.1, §2.2.7
// ---------------------------------------------------------------------------
module pcie_config_decode
  import pcie_datalink_pkg::*;
  import pcie_tlp_pkg::*;
#(
    parameter int DATA_WIDTH = 32,
    parameter int STRB_WIDTH = DATA_WIDTH / 8,
    parameter int KEEP_WIDTH = STRB_WIDTH,
    parameter int USER_WIDTH = 1,
    parameter int TLP_SEG_COUNT = 1,
    parameter int TLP_DATA_WIDTH = 128,
    parameter int TLP_STRB_WIDTH = 5,
    parameter int TLP_HDR_WIDTH = 128

) (
    input  logic                  clk_i,
    input  logic                  rst_i,

    // ---- CfgRd0 and CfgWr0, from pcie_config_mux ---------------------------
    input  logic [DATA_WIDTH-1:0] s_axis_tdata,
    input  logic [KEEP_WIDTH-1:0] s_axis_tkeep,
    input  logic                  s_axis_tvalid,
    input  logic                  s_axis_tlast,
    input  logic [USER_WIDTH-1:0] s_axis_tuser,
    output logic                  s_axis_tready,


    // ---- request header, to pcie_config_handler ----------------------------
    output wire [             TLP_DATA_WIDTH-1:0] rx_tlp_data,
    output wire [             TLP_STRB_WIDTH-1:0] rx_tlp_strb,
    output wire [TLP_SEG_COUNT*TLP_HDR_WIDTH-1:0] rx_tlp_hdr,
    output wire [            TLP_SEG_COUNT*4-1:0] rx_tlp_error,
    output wire [              TLP_SEG_COUNT-1:0] rx_tlp_valid,
    output wire [              TLP_SEG_COUNT-1:0] rx_tlp_sop,
    output wire [              TLP_SEG_COUNT-1:0] rx_tlp_eop,
    input  wire                                   rx_tlp_ready
);
  /* verilator lint_off WIDTHEXPAND */
  /* verilator lint_off WIDTHTRUNC */

  typedef enum logic [4:0] {
    ST_IDLE,               // takes header Dword 0 from a first beat
    ST_TLP_HEADER_WORD_0,  // never entered
    ST_TLP_HEADER_WORD_1,
    ST_TLP_HEADER_WORD_2,  // last header Dword of a 3DW request
    ST_TLP_HEADER_WORD_3,  // header Dword 3, for a header that is not 3DW
    ST_TLP_STREAM,         // collects payload into tlp_data once tlp_is_pd is set
    ST_TLP_SEND            // offers the header on rx_tlp_*
  } cfg_decode_state_t;


  // Only reset clears tlp_is_pd and word_count.
  typedef struct packed {
    cfg_decode_state_t         state;
    tlp_hdr_union_t            tlp_hdr;
    logic [31:0]               word_count;
    logic                      tlp_is_3dw;
    logic                      tlp_is_pd;
    logic                      tlp_is_sop;
    logic                      tlp_is_eop;
    logic [TLP_DATA_WIDTH-1:0] tlp_data;
  } cfg_decode_t;

  cfg_decode_t D, Q;


  // The first beat as received, for its Fmt field; assigned only when
  // ST_IDLE takes a first beat.
  pcie_tlp_header_dw0_t                       tlp_dw0;
  //skid buffer axis signals
  logic                 [     DATA_WIDTH-1:0] skid_axis_tdata;
  logic                 [     KEEP_WIDTH-1:0] skid_axis_tkeep;
  logic                                       skid_axis_tvalid;
  logic                                       skid_axis_tlast;
  logic                 [     USER_WIDTH-1:0] skid_axis_tuser;
  logic                                       skid_axis_tready;
  logic                 [               31:0] tlp_byte_swapped;
  // Connected to rx_tlp_*. tlp_tdata, tlp_strb and tlp_error are only ever 0.
  logic                 [     DATA_WIDTH-1:0] tlp_tdata;
  logic                 [     KEEP_WIDTH-1:0] tlp_strb;
  logic                                       tlp_valid;
  logic                                       tlp_eop;
  logic                 [     USER_WIDTH-1:0] tlp_sop;
  logic                 [TLP_SEG_COUNT*4-1:0] tlp_error;
  logic                                       tlp_ready;



  always_ff @(posedge clk_i) begin : main_seq
    if (rst_i) begin
      Q <= '{state: ST_IDLE, default: 'd0};
    end else begin
      Q <= D;
    end
  end


  // The stream carries the first byte of each Dword in bits 7:0; tlp_hdr_t
  // holds it in bits 31:24. Reversing the four bytes of a beat converts one
  // to the other.
  always_comb begin : byte_swap_tlp
    for (int i = 0; i < 4; i++) begin
      tlp_byte_swapped[(8*i)+:8] = skid_axis_tdata[8*(3-i)+:8];
    end
  end


  always_comb begin : main_combo
    D                = Q;
    skid_axis_tready = '0;
    tlp_tdata        = '0;
    tlp_strb         = '0;
    tlp_valid        = '0;
    tlp_eop          = '0;
    tlp_sop          = '0;
    tlp_error        = '0;

    case (Q.state)
      ST_IDLE: begin
        skid_axis_tready = '1;
        D.tlp_hdr.whole_ = '0;
        if (skid_axis_tvalid && !skid_axis_tlast) begin
          D.tlp_is_3dw             = '0;
          D.tlp_data               = '0;
          D.tlp_is_sop             = '1;
          tlp_dw0                  = skid_axis_tdata;
          D.tlp_hdr.struct_.word_0 = tlp_byte_swapped;
          if (tlp_dw0.byte0.Fmt inside {TLP_3DW_WD, TLP_3DW_ND}) begin
            D.tlp_is_3dw = '1;
          end
          // tlp_is_pd selects payload collection but is set by the 4DW
          // formats, so a 3DW request with data, such as CfgWr0, leaves it
          // clear and its payload is not collected.
          if (tlp_dw0.byte0.Fmt inside {TLP_4DW_ND, TLP_4DW_WD}) begin
            D.tlp_is_pd = '1;
          end
          D.state = ST_TLP_HEADER_WORD_1;
        end
      end
      ST_TLP_HEADER_WORD_1: begin
        skid_axis_tready = '1;
        if (skid_axis_tvalid) begin
          D.tlp_hdr.struct_.word_1 = tlp_byte_swapped;
          D.state = ST_TLP_HEADER_WORD_2;
        end
      end
      ST_TLP_HEADER_WORD_2: begin
        skid_axis_tready = '1;
        if (skid_axis_tvalid) begin
          D.tlp_hdr.struct_.word_2 = tlp_byte_swapped;
          D.tlp_is_3dw = '0;
          // A configuration request takes the ST_TLP_SEND arm. A CfgWr0's
          // payload beat is not accepted until ST_IDLE, which takes it as a
          // beat with tlast and discards it.
          if (Q.tlp_is_3dw) begin
            if (Q.tlp_is_pd) begin
              D.state = ST_TLP_STREAM;
            end else begin
              D.tlp_is_eop = '1;
              D.state = ST_TLP_SEND;
            end
          end else begin
            D.state = ST_TLP_HEADER_WORD_3;
          end
        end

      end
      ST_TLP_HEADER_WORD_3: begin
        skid_axis_tready = '1;
        if (skid_axis_tvalid) begin
          D.tlp_hdr.struct_.word_3 = tlp_byte_swapped;
          if (Q.tlp_is_pd) begin
            D.state = ST_TLP_STREAM;
          end else begin
            D.tlp_is_eop = '1;
            D.state = ST_TLP_SEND;
          end
        end
      end
      // Reads s_axis_tvalid and s_axis_tlast, the skid buffer's input, while
      // the data and handshake come from its output.
      ST_TLP_STREAM: begin
        skid_axis_tready = '1;
        if (s_axis_tvalid) begin
          D.tlp_data[(3-Q.word_count)*32+:32] = tlp_byte_swapped;
          D.word_count = Q.word_count + 1'b1;
          if (Q.word_count >= 8'd3) begin
            D.state = ST_TLP_SEND;
          end
          if (s_axis_tlast) begin
            D.tlp_is_eop = '1;
            D.state = ST_TLP_SEND;
          end
        end
      end
      // rx_tlp_sop and rx_tlp_eop are driven only in the cycle rx_tlp_ready
      // is high. pcie_config_handler's ready is high in its ST_IDLE, where it
      // samples them.
      ST_TLP_SEND: begin
        tlp_valid = '1;
        if (tlp_ready) begin
          tlp_tdata    = '0;
          tlp_strb     = '0;
          D.tlp_is_sop = '0;
          D.tlp_is_eop = '0;
          tlp_eop      = Q.tlp_is_eop;
          tlp_sop      = Q.tlp_is_sop;
          D.state      = ST_TLP_STREAM;
          if (Q.tlp_is_eop) begin
            D.tlp_is_eop = '0;
            D.state = ST_IDLE;
          end
        end
      end
      default: begin
      end
    endcase
  end


  // tlp_tdata is never assigned from Q.tlp_data, so rx_tlp_data is always 0.
  assign rx_tlp_data  = tlp_tdata;
  assign rx_tlp_strb  = tlp_strb;
  assign rx_tlp_hdr   = Q.tlp_hdr;
  assign rx_tlp_error = tlp_error;
  assign rx_tlp_valid = tlp_valid;
  assign rx_tlp_sop   = tlp_sop;
  assign rx_tlp_eop   = tlp_eop;
  assign tlp_ready    = rx_tlp_ready;


  //axis input skid buffer
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
      .m_axis_tdata (skid_axis_tdata),
      .m_axis_tkeep (skid_axis_tkeep),
      .m_axis_tvalid(skid_axis_tvalid),
      .m_axis_tready(skid_axis_tready),
      .m_axis_tlast (skid_axis_tlast),
      .m_axis_tuser (skid_axis_tuser),
      .m_axis_tid   (),
      .m_axis_tdest ()
  );

  /* verilator lint_on WIDTHEXPAND */
  /* verilator lint_on WIDTHTRUNC */
endmodule
