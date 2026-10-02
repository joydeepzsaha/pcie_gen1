// ---------------------------------------------------------------------------
// pcie_to_axis_converter -- verilog-pcie TLP interface to AXI4-Stream TLP
//
//! @title pcie_to_axis_converter
//! @author Idris Somoye
//! Converts the TLP interface of verilog-pcie to AXI4-Stream TLPs.
//
// Purpose
//   Takes TLPs on tx_tlp_* through a pcie_tlp_fifo and sends each on
//   m_axis_*, one DW per beat, through an axis_fifo. main_combo reads the
//   header as a tlp_hdr_t and sends its DWs as stored, then sends the
//   payload from each data segment, highest DW first, with the bytes of
//   each DW reversed. Fmt in DW0 selects a 3 or 4 DW header and whether a
//   payload follows.
//
// Interfaces
//   TLP input    tx_tlp_*: pcie_tlp_fifo's input. tx_tlp_hdr is read as a
//                tlp_hdr_t; tx_tlp_strb and tx_tlp_error pass through the
//                FIFO unread.
//   Output       m_axis_*: DW0 first. tkeep is all ones and tuser is 0.
//
// Clock and reset
//   clk_i only. rst_i is synchronous and active high. It sets Q.state to
//   ST_IDLE and the other fields of Q to 0, and resets pcie_tlp_fifo and
//   axis_fifo.
//
// Limitations
//   No module in the repository instantiates it. With the default
//   TLP_DATA_WIDTH (128) and TLP_STRB_WIDTH (5), pcie_tlp_fifo's check that
//   TLP_STRB_WIDTH * 32 equals TLP_DATA_WIDTH fails and ends the simulation.
//   ST_TLP_HEADER_WORD_2 and ST_TLP_HEADER_WORD_3 name fields of word_0
//   that common_tlp_hdr_t does not declare. At the default DATA_WIDTH the
//   payload DWs sent are 0, and their count does not follow Length;
//   main_combo's case arms give the detail. tx_tlp_hdr is read with DW0 in
//   bits 31:0, where the verilog-pcie modules that build a header (pcie_msix,
//   for one) put DW0 in bits 127:96.
//
// References
//   PCIe Base Spec r2.1, §2.2.1
// ---------------------------------------------------------------------------
module pcie_to_axis_converter
  import pcie_datalink_pkg::*;
  import pcie_tlp_pkg::*;
#(
    // Width of m_axis_tdata; main_combo drives one DW in bits 31:0 of each
    // beat.
    parameter int DATA_WIDTH       = 32,
    // Sets the KEEP_WIDTH default only.
    parameter int STRB_WIDTH       = DATA_WIDTH / 8,
    parameter int KEEP_WIDTH       = STRB_WIDTH,
    parameter int USER_WIDTH       = 1,
    // MAX_PAYLOAD_SIZE and RX_FIFO_SIZE are not used.
    parameter int MAX_PAYLOAD_SIZE = 256,
    parameter int RX_FIFO_SIZE     = 2,
    // Widths of tx_tlp_*. pcie_tlp_fifo takes TLP_DATA_WIDTH, TLP_STRB_WIDTH
    // and TLP_HDR_WIDTH; its segment counts are fixed at 1, not TLP_SEG_COUNT.
    parameter int TLP_SEG_COUNT    = 1,
    parameter int TLP_DATA_WIDTH   = 128,
    parameter int TLP_STRB_WIDTH   = 5,
    parameter int TLP_HDR_WIDTH    = 128

) (
    // ---- clock and reset ----------------------------------------------------
    input  logic                                   clk_i,
    input  logic                                   rst_i,
    // ---- TLP input, through pcie_tlp_fifo -----------------------------------
    input  logic [             TLP_DATA_WIDTH-1:0] tx_tlp_data,
    input  logic [             TLP_STRB_WIDTH-1:0] tx_tlp_strb,
    input  logic [TLP_SEG_COUNT*TLP_HDR_WIDTH-1:0] tx_tlp_hdr,
    input  logic [            TLP_SEG_COUNT*4-1:0] tx_tlp_error,
    input  logic [              TLP_SEG_COUNT-1:0] tx_tlp_valid,
    input  logic [              TLP_SEG_COUNT-1:0] tx_tlp_sop,
    input  logic [              TLP_SEG_COUNT-1:0] tx_tlp_eop,
    output logic                                   tx_tlp_ready,

    // ---- TLP output, through axis_fifo --------------------------------------
    output logic [DATA_WIDTH-1:0] m_axis_tdata,
    output logic [KEEP_WIDTH-1:0] m_axis_tkeep,
    output logic                  m_axis_tvalid,
    output logic                  m_axis_tlast,
    output logic [USER_WIDTH-1:0] m_axis_tuser,
    input  logic                  m_axis_tready
);
  /* verilator lint_off WIDTHEXPAND */
  /* verilator lint_off WIDTHTRUNC */

  // main_combo's states.
  typedef enum logic [4:0] {
    ST_IDLE,
    ST_TLP_HEADER_WORD_0,
    ST_TLP_HEADER_WORD_1,
    ST_TLP_HEADER_WORD_2,
    ST_TLP_HEADER_WORD_3,
    ST_TLP_STREAM,
    ST_TLP_SEND
  } axis_pcie_conv_t;

  // axis_fifo input
  logic [                 DATA_WIDTH-1:0] s_axis_tdata;
  logic [                 KEEP_WIDTH-1:0] s_axis_tkeep;
  logic                                   s_axis_tvalid;
  logic                                   s_axis_tlast;
  logic [                 USER_WIDTH-1:0] s_axis_tuser;
  logic                                   s_axis_tready;

  logic [                           31:0] tlp_data_word;
  logic [                           31:0] tlp_byte_swapped;
  // pcie_tlp_fifo output; main_combo does not read tlp_strb or tlp_error.
  logic [                 DATA_WIDTH-1:0] tlp_tdata;
  logic [TLP_SEG_COUNT*TLP_HDR_WIDTH-1:0] tlp_hdr;
  logic [                 KEEP_WIDTH-1:0] tlp_strb;
  logic                                   tlp_valid;
  logic                                   tlp_eop;
  logic [                 USER_WIDTH-1:0] tlp_sop;
  logic [            TLP_SEG_COUNT*4-1:0] tlp_error;
  logic                                   tlp_ready;

  typedef struct {

    axis_pcie_conv_t           state;
    tlp_hdr_union_t            tlp_hdr;
    logic [31:0]               word_count;
    // tlp_dw0, tlp_is_3dw, tlp_is_sop and tlp_is_pd are never read.
    pcie_tlp_header_dw0_t      tlp_dw0;
    logic                      tlp_is_3dw;
    logic                      tlp_is_sop;
    logic                      tlp_is_pd;
    logic                      tlp_is_eop;
    logic [TLP_DATA_WIDTH-1:0] tlp_data;
    logic [31:0]               length;

  } fsm_struct_t;

  fsm_struct_t D, Q;

  always_ff @(posedge clk_i) begin : main_seq
    if (rst_i) begin
      Q <= '{state: ST_IDLE, default: 'd0};
    end else begin
      Q <= D;
    end
  end



  always_comb begin : main_combo
    D                = Q;
    tlp_ready        = '0;
    D.tlp_dw0          = '0;
    s_axis_tdata     = '0;
    s_axis_tkeep     = '0;
    s_axis_tvalid    = '0;
    s_axis_tlast     = '0;
    s_axis_tuser     = '0;
    tlp_data_word    = '0;
    tlp_byte_swapped = '0;

    case (Q.state)
      // Takes the first segment of a TLP. A segment without SOP is taken and
      // dropped.
      ST_IDLE: begin
        tlp_ready        = '1;
        D.tlp_hdr.whole_ = '0;
        if (tlp_valid && tlp_sop) begin
          D.tlp_hdr.whole_ = tlp_hdr;
          D.tlp_data       = tlp_tdata;
          D.tlp_is_eop     = tlp_eop;
          D.word_count     = '0;
          D.state          = ST_TLP_HEADER_WORD_0;
        end
      end
      // The header DWs go out as tlp_hdr_t holds them, first byte in bits
      // 31:24, with no byte reversal.
      ST_TLP_HEADER_WORD_0: begin
        s_axis_tdata  = Q.tlp_hdr.struct_.word_0;
        s_axis_tkeep  = '1;
        s_axis_tvalid = '1;
        s_axis_tlast  = '0;
        s_axis_tuser  = '0;
        if (s_axis_tready) begin
          D.state = ST_TLP_HEADER_WORD_1;
        end
      end
      ST_TLP_HEADER_WORD_1: begin
        s_axis_tdata  = Q.tlp_hdr.struct_.word_1;
        s_axis_tkeep  = '1;
        s_axis_tvalid = '1;
        s_axis_tlast  = '0;
        s_axis_tuser  = '0;
        if (s_axis_tready) begin
          D.state = ST_TLP_HEADER_WORD_2;
        end
      end
      // Fmt and Length are read from word_0.byte_0, byte_2 and byte_3;
      // common_tlp_hdr_t names its fields byte0 to byte3. Any other Fmt, 100b
      // (a TLP Prefix) or a reserved value, returns to ST_IDLE without tlast.
      ST_TLP_HEADER_WORD_2: begin
        s_axis_tdata  = Q.tlp_hdr.struct_.word_2;
        s_axis_tkeep  = '1;
        s_axis_tvalid = '1;
        s_axis_tlast  = '0;
        s_axis_tuser  = '0;
        if (s_axis_tready) begin
          case (Q.tlp_hdr.struct_.word_0.byte_0.Fmt)
            TLP_3DW_ND: begin
              s_axis_tlast = '1;
              D.state = ST_IDLE;
            end
            TLP_4DW_ND: begin
              D.state = ST_TLP_HEADER_WORD_3;
            end
            TLP_3DW_WD: begin
              D.length = {
                Q.tlp_hdr.struct_.word_0.byte_2.Length, Q.tlp_hdr.struct_.word_0.byte_3.Length
              };
              D.state = ST_TLP_STREAM;
            end
            TLP_4DW_WD: begin
              D.length = {
                Q.tlp_hdr.struct_.word_0.byte_2.Length, Q.tlp_hdr.struct_.word_0.byte_3.Length
              };
              D.state = ST_TLP_HEADER_WORD_3;

            end
            default: begin
              D.state = ST_IDLE;
            end
          endcase
        end
      end
      // Reads Fmt from word_0.byte_0, as ST_TLP_HEADER_WORD_2 does.
      ST_TLP_HEADER_WORD_3: begin
        s_axis_tdata  = Q.tlp_hdr.struct_.word_3;
        s_axis_tkeep  = '1;
        s_axis_tvalid = '1;
        s_axis_tlast  = '0;
        s_axis_tuser  = '0;
        if (s_axis_tready) begin
          if (Q.tlp_hdr.struct_.word_0.byte_0.Fmt == TLP_4DW_ND) begin
            s_axis_tlast = '1;
            D.state = ST_IDLE;
          end else begin
            D.state = ST_TLP_STREAM;
          end
        end
      end
      // Sends the current segment's payload from bits 127:96 down. tlp_tdata
      // is DATA_WIDTH bits wide, so at DATA_WIDTH = 32 only bits 31:0 of
      // tlp_data come from pcie_tlp_fifo, and the DWs sent here are 0.
      ST_TLP_STREAM: begin
        tlp_data_word = Q.tlp_data[(32'd3-Q.word_count[1:0])*32+:32];
        for (int i = 0; i < 4; i++) begin
          tlp_byte_swapped[(8*i)+:8] = tlp_data_word[8*(3-i)+:8];
        end
        s_axis_tdata  = tlp_byte_swapped;
        s_axis_tkeep  = '1;
        s_axis_tvalid = '1;
        s_axis_tlast  = '0;
        s_axis_tuser  = '0;
        if (s_axis_tready) begin
          D.word_count = Q.word_count + 1'b1;
          // After the DW in bits 63:32 it fetches the next segment, so the DW
          // in bits 31:0 is never sent.
          if ((Q.word_count[1:0] == 2'b10)) begin
            D.state = ST_TLP_SEND;
          end
          // word_count restarts at 0 with each segment and is at most 2 in this
          // state, yet is compared with the TLP's Length field: tlast is sent
          // only when that field is 0, 1 or 2, after field + 1 DWs of a segment
          // with EOP.
          if ((Q.word_count >= Q.length) && Q.tlp_is_eop) begin
            s_axis_tlast = '1;
            D.state      = ST_IDLE;
          end
        end
      end
      // Takes the next pcie_tlp_fifo segment as the rest of the payload
      // without checking tlp_sop, so it can be the first segment of the next
      // TLP.
      ST_TLP_SEND: begin
        tlp_ready        = '1;
        D.tlp_hdr.whole_ = '0;
        if (tlp_valid) begin
          D.tlp_hdr.whole_ = tlp_hdr;
          D.tlp_data       = tlp_tdata;
          D.tlp_is_eop     = tlp_eop;
          D.word_count     = '0;
          D.state          = ST_TLP_STREAM;
        end
      end
      default: begin
      end
    endcase
  end

  pcie_tlp_fifo #(
      .DEPTH            (1),
      .TLP_DATA_WIDTH   (TLP_DATA_WIDTH),
      .TLP_STRB_WIDTH   (TLP_STRB_WIDTH),
      .TLP_HDR_WIDTH    (TLP_HDR_WIDTH),
      .SEQ_NUM_WIDTH    (8),
      .IN_TLP_SEG_COUNT (1),
      .OUT_TLP_SEG_COUNT(1),
      .WATERMARK        ('0)
  ) pcie_tlp_fifo_inst (
      .clk             (clk_i),
      .rst             (rst_i),
      //tlp in
      .in_tlp_data     (tx_tlp_data),
      .in_tlp_strb     (tx_tlp_strb),
      .in_tlp_hdr      (tx_tlp_hdr),
      .in_tlp_seq      ('0),
      .in_tlp_bar_id   ('0),
      .in_tlp_func_num ('0),
      .in_tlp_error    (tx_tlp_error),
      .in_tlp_valid    (tx_tlp_valid),
      .in_tlp_sop      (tx_tlp_sop),
      .in_tlp_eop      (tx_tlp_eop),
      .in_tlp_ready    (tx_tlp_ready),
      //tlp out
      .out_tlp_data    (tlp_tdata),
      .out_tlp_strb    (tlp_strb),
      .out_tlp_hdr     (tlp_hdr),
      .out_tlp_seq     (),
      .out_tlp_bar_id  (),
      .out_tlp_func_num(),
      .out_tlp_error   (tlp_error),
      .out_tlp_valid   (tlp_valid),
      .out_tlp_sop     (tlp_sop),
      .out_tlp_eop     (tlp_eop),
      .out_tlp_ready   (tlp_ready),
      .half_full       (),
      .watermark       ()
  );


  // The output FIFO. With KEEP_ENABLE set, DEPTH counts bytes: 16 is four
  // beats at DATA_WIDTH = 32.
  axis_fifo #(
      .DEPTH(16),
      .DATA_WIDTH (DATA_WIDTH),
      .KEEP_ENABLE('1),
      .KEEP_WIDTH (KEEP_WIDTH),
      .LAST_ENABLE('1),
      .ID_ENABLE  ('0),
      .ID_WIDTH   (1),
      .DEST_ENABLE('0),
      .DEST_WIDTH (1),
      .USER_ENABLE('1),
      .USER_WIDTH (USER_WIDTH)
  ) axis_register_pipeline_inst (
      .clk                (clk_i),
      .rst                (rst_i),
      .s_axis_tdata       (s_axis_tdata),
      .s_axis_tkeep       (s_axis_tkeep),
      .s_axis_tvalid      (s_axis_tvalid),
      .s_axis_tready      (s_axis_tready),
      .s_axis_tlast       (s_axis_tlast),
      .s_axis_tuser       (s_axis_tuser),
      .s_axis_tid         ('0),
      .s_axis_tdest       ('0),
      .m_axis_tdata       (m_axis_tdata),
      .m_axis_tkeep       (m_axis_tkeep),
      .m_axis_tvalid      (m_axis_tvalid),
      .m_axis_tready      (m_axis_tready),
      .m_axis_tlast       (m_axis_tlast),
      .m_axis_tuser       (m_axis_tuser),
      .m_axis_tid         (),
      .m_axis_tdest       (),
      .pause_req          (),
      .pause_ack          (),
      .status_depth       (),
      .status_depth_commit(),
      .status_overflow    (),
      .status_bad_frame   (),
      .status_good_frame  ()



  );

  /* verilator lint_on WIDTHEXPAND */
  /* verilator lint_on WIDTHTRUNC */
endmodule
