// ---------------------------------------------------------------------------
//! @title pcie_config_mux
//! @author Idris Somoye
//! Copies each CfgRd0 and CfgWr0 to the configuration path and passes every
//! received TLP on.
//
// Purpose
//   Chooses a route for each received TLP from the Fmt and Type byte of its
//   first beat, and holds it until the beat with tlast. Every TLP goes to
//   m_tlp_axis_*. A CfgRd0 or CfgWr0 also goes to m_cfg_axis_*, for
//   pcie_config_decode; an input beat of such a TLP is accepted only once
//   both outputs have taken it. The input and both outputs each have a skid
//   buffer.
//
// Interfaces
//   Input         s_axis_*: received TLPs from dllp2tlp; header byte 0 is in
//                 bits 7:0 of the first beat.
//   Config        m_cfg_axis_*: CfgRd0 and CfgWr0 only.
//   TLP output    m_tlp_axis_*: every TLP, configuration requests included.
//
// Clock and reset
//   clk_i only. rst_i is synchronous and active high.
//
// Limitations
//   The parameters TLP_SEG_COUNT, TLP_DATA_WIDTH, TLP_STRB_WIDTH and
//   TLP_HDR_WIDTH are not used. Q.tlp_hdr, Q.word_count and
//   tlp_byte_swapped are not read.
//
// References
//   PCIe Base Spec r2.1, §2.2.1
// ---------------------------------------------------------------------------
module pcie_config_mux
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

    // ---- received TLPs -----------------------------------------------------
    input  logic [DATA_WIDTH-1:0] s_axis_tdata,
    input  logic [KEEP_WIDTH-1:0] s_axis_tkeep,
    input  logic                  s_axis_tvalid,
    input  logic                  s_axis_tlast,
    input  logic [USER_WIDTH-1:0] s_axis_tuser,
    output logic                  s_axis_tready,


    // ---- CfgRd0 and CfgWr0, to pcie_config_decode --------------------------
    output logic [DATA_WIDTH-1:0] m_cfg_axis_tdata,
    output logic [KEEP_WIDTH-1:0] m_cfg_axis_tkeep,
    output logic                  m_cfg_axis_tvalid,
    output logic                  m_cfg_axis_tlast,
    output logic [USER_WIDTH-1:0] m_cfg_axis_tuser,
    input  logic                  m_cfg_axis_tready,


    // ---- every TLP, on toward the Transaction Layer ------------------------
    output logic [DATA_WIDTH-1:0] m_tlp_axis_tdata,
    output logic [KEEP_WIDTH-1:0] m_tlp_axis_tkeep,
    output logic                  m_tlp_axis_tvalid,
    output logic                  m_tlp_axis_tlast,
    output logic [USER_WIDTH-1:0] m_tlp_axis_tuser,
    input  logic                  m_tlp_axis_tready
);
  /* verilator lint_off WIDTHEXPAND */
  /* verilator lint_off WIDTHTRUNC */

  typedef enum logic [4:0] {
    ST_IDLE,     // routes each first beat
    ST_CFG_TLP,  // rest of a CfgRd0 or CfgWr0, to both outputs
    ST_MEM_TLP   // rest of any other TLP, to m_tlp_axis_* only
  } cfg_decode_state_t;

  typedef struct packed {

    cfg_decode_state_t state;
    tlp_hdr_union_t    tlp_hdr;
    logic [31:0]       word_count;
  } cfg_decode_t;

  pcie_tlp_header_dw0_t tlp_dw0;

  cfg_decode_t D, Q;

  logic [DATA_WIDTH-1:0] tlp_axis_tdata;
  logic [KEEP_WIDTH-1:0] tlp_axis_tkeep;
  logic                  tlp_axis_tvalid;
  logic                  tlp_axis_tlast;
  logic [USER_WIDTH-1:0] tlp_axis_tuser;
  logic                  tlp_axis_tready;


  logic [DATA_WIDTH-1:0] cfg_axis_tdata;
  logic [KEEP_WIDTH-1:0] cfg_axis_tkeep;
  logic                  cfg_axis_tvalid;
  logic                  cfg_axis_tlast;
  logic [USER_WIDTH-1:0] cfg_axis_tuser;
  logic                  cfg_axis_tready;
  logic                  cfg_beat_sent_r;
  logic                  tlp_beat_sent_r;
  logic                  route_cfg;


  // skid_axis_*: the input skid buffer's output.
    logic                 [     DATA_WIDTH-1:0] skid_axis_tdata;
    logic                 [     KEEP_WIDTH-1:0] skid_axis_tkeep;
    logic                                       skid_axis_tvalid;
    logic                                       skid_axis_tlast;
    logic                 [     USER_WIDTH-1:0] skid_axis_tuser;
    logic                                       skid_axis_tready;
    logic                 [               31:0] tlp_byte_swapped;

  always_ff @(posedge clk_i) begin : main_seq
    if (rst_i) begin
      Q <= '{state: ST_IDLE, default: 'd0};
      cfg_beat_sent_r <= 1'b0;
      tlp_beat_sent_r <= 1'b0;
    end else begin
      Q <= D;
      // Each flag records that its output has taken the current beat of a
      // configuration request while the other output has not. Both clear
      // when the input accepts the beat, and outside configuration requests.
      if (!route_cfg || (skid_axis_tvalid && skid_axis_tready)) begin
        cfg_beat_sent_r <= 1'b0;
        tlp_beat_sent_r <= 1'b0;
      end else begin
        if (cfg_axis_tvalid && cfg_axis_tready)
          cfg_beat_sent_r <= 1'b1;
        if (tlp_axis_tvalid && tlp_axis_tready)
          tlp_beat_sent_r <= 1'b1;
      end
    end
  end


  // tlp_byte_swapped is not read in this module.
  always_comb begin : byte_swap_tlp
    for (int i = 0; i < 4; i++) begin
      tlp_byte_swapped[(8*i)+:8] = skid_axis_tdata[8*(3-i)+:8];
    end
  end


  always_comb begin : main_combo
    D               = Q;
    tlp_dw0         = '0;

    tlp_axis_tvalid = '0;
    cfg_axis_tvalid = '0;
    skid_axis_tready = '0;
    route_cfg = 1'b0;
    tlp_axis_tdata  = skid_axis_tdata;
    tlp_axis_tkeep  = skid_axis_tkeep;
    tlp_axis_tlast  = skid_axis_tlast;
    tlp_axis_tuser  = skid_axis_tuser;
    cfg_axis_tdata  = skid_axis_tdata;
    cfg_axis_tkeep  = skid_axis_tkeep;
    cfg_axis_tlast  = skid_axis_tlast;
    cfg_axis_tuser  = skid_axis_tuser;

    case (Q.state)
      ST_IDLE: begin
        if (skid_axis_tvalid) begin
          tlp_dw0 = skid_axis_tdata;
          if (tlp_dw0.byte0 inside {CfgRd0, CfgWr0}) begin
            // Both outputs: pcie_config_handler answers the request, and the
            // Transaction Layer still receives it. Each output's handshake is
            // tracked on its own, so neither takes a beat twice while the
            // other applies back-pressure.
            route_cfg = 1'b1;
            cfg_axis_tvalid = skid_axis_tvalid && !cfg_beat_sent_r;
            tlp_axis_tvalid = skid_axis_tvalid && !tlp_beat_sent_r;
            skid_axis_tready =
                (cfg_beat_sent_r || cfg_axis_tready) &&
                (tlp_beat_sent_r || tlp_axis_tready);
            if (skid_axis_tready && !skid_axis_tlast)
              D.state = ST_CFG_TLP;
          end else begin
            tlp_axis_tvalid = skid_axis_tvalid;
            skid_axis_tready = tlp_axis_tready;
            if (skid_axis_tready && !skid_axis_tlast)
              D.state = ST_MEM_TLP;
          end
        end
      end
      ST_CFG_TLP: begin
        route_cfg = 1'b1;
        cfg_axis_tvalid = skid_axis_tvalid && !cfg_beat_sent_r;
        tlp_axis_tvalid = skid_axis_tvalid && !tlp_beat_sent_r;
        skid_axis_tready =
            (cfg_beat_sent_r || cfg_axis_tready) &&
            (tlp_beat_sent_r || tlp_axis_tready);
        if (skid_axis_tvalid && skid_axis_tready && skid_axis_tlast)
          D.state = ST_IDLE;
      end
      ST_MEM_TLP: begin
        skid_axis_tready = tlp_axis_tready;
        tlp_axis_tvalid = skid_axis_tvalid;
        if (skid_axis_tvalid && skid_axis_tready && skid_axis_tlast)
          D.state = ST_IDLE;
      end
      default: begin
      end
    endcase
  end

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
  ) tlp_fifo_inst (
      .clk          (clk_i),
      .rst          (rst_i),
      // AXI input
      .s_axis_tdata (tlp_axis_tdata),
      .s_axis_tkeep (tlp_axis_tkeep),
      .s_axis_tvalid(tlp_axis_tvalid),
      .s_axis_tready(tlp_axis_tready),
      .s_axis_tlast (tlp_axis_tlast),
      .s_axis_tuser (tlp_axis_tuser),
      .s_axis_tid   (),
      .s_axis_tdest (),
      // AXI output
      .m_axis_tdata (m_tlp_axis_tdata),
      .m_axis_tkeep (m_tlp_axis_tkeep),
      .m_axis_tvalid(m_tlp_axis_tvalid),
      .m_axis_tready(m_tlp_axis_tready),
      .m_axis_tlast (m_tlp_axis_tlast),
      .m_axis_tuser (m_tlp_axis_tuser),
      .m_axis_tid   (),
      .m_axis_tdest ()
  );


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
  ) cfg_fifo_inst (
      .clk          (clk_i),
      .rst          (rst_i),
      // AXI input
      .s_axis_tdata (cfg_axis_tdata),
      .s_axis_tkeep (cfg_axis_tkeep),
      .s_axis_tvalid(cfg_axis_tvalid),
      .s_axis_tready(cfg_axis_tready),
      .s_axis_tlast (cfg_axis_tlast),
      .s_axis_tuser (cfg_axis_tuser),
      .s_axis_tid   (),
      .s_axis_tdest (),
      // AXI output
      .m_axis_tdata (m_cfg_axis_tdata),
      .m_axis_tkeep (m_cfg_axis_tkeep),
      .m_axis_tvalid(m_cfg_axis_tvalid),
      .m_axis_tready(m_cfg_axis_tready),
      .m_axis_tlast (m_cfg_axis_tlast),
      .m_axis_tuser (m_cfg_axis_tuser),
      .m_axis_tid   (),
      .m_axis_tdest ()
  );


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
