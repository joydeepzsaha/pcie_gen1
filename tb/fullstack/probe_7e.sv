// =============================================================================
// §63 #7e Phase 1 -- measure the CfgRd0 at five stages. BENCH-ONLY, `bind`.
//
// !! NOTHING IN src/ CHANGES FOR THIS FILE, and verilate_fullstack is untouched:
// own fileset, own target (§63 #7d's rule). A probe that perturbs the row it
// measures is not a measurement of that row.
//
// !! WRITTEN WITHOUT READING ANY MODULE BODY (D-7E.1). Every name below came
// from a PORT DECLARATION or a SIGNAL DECLARATION, extracted mechanically. No
// behavioural line of src/ has been read at the time this file was authored, and
// predictions G1-G7 were committed first (PREDICTIONS_7E.md, pcie_docs b45150e).
// The distinction that matters is #7d's: knowing a wire's NAME is not a causal
// story about what drives it.
//
// == THE FIVE STAGES, AND WHERE EACH ONE LIVES ==============================
//
// BRIEF_7E_CHAT §1 Phase 1 asks for five measurements. Four of the five land on
// ONE module -- pcie_datalink_layer -- which exists in BOTH stacks, so a single
// `bind` and %m answer them for the RC and the EP at once:
//
//   (1) RC transmit    s_tlp_axis_* (TL->DLL)  and  m_phy_axis_* (DLL->PHY)
//   (2) the bridge     phy_transmit.pipe_data_o / phy_receive.pipe_data_i  [K-codes]
//   (3) EP receive     s_phy_axis_* (PHY->DLL)  and  phy_tlp_axis_* (the TLP arm)
//   (4) EP TL          m_tlp_axis_* (DLL->TL) up, s_tlp_axis_* (CplD) back down
//   (5) RC receive     s_phy_axis_* / phy_tlp_axis_* / m_tlp_axis_* on the RC
//
// ⚠️ phy_receive HAS ONLY ONE AXIS OUTPUT PORT -- m_dllp_axis_*. TLPs and DLLPs
// share it. The name says DLLP and the stream is both. So "did the TLP arrive"
// CANNOT be answered at the PHY's port by counting beats; it is answered one
// level up, at pcie_datalink_layer's INTERNAL phy_tlp_axis_*, which is the arm
// the DLL splits out of that shared stream. That is why stage (3) probes an
// internal signal rather than a port -- not for convenience.
//
// ⚠️ A DLLP is 8 bytes and a CfgRd0 is 3 DW header + seq + LCRC. Beat counts at
// a shared seam are therefore MIXED and mean nothing on their own. Every counter
// below is split TLP-arm vs DLLP-arm, and first-beat tdata is captured at each
// seam so the packet can be IDENTIFIED rather than assumed.
// =============================================================================
`timescale 1ns / 1ps

// -----------------------------------------------------------------------------
// One AXIS seam, measured. Beats, tlast, tkeep-on-last histogram, first-beat
// tdata, first/last cycle stamps. Instanced once per seam inside pr7e_dll.
// -----------------------------------------------------------------------------
module pr7e_axis #(
    parameter string NAME = "?"
) (
    input logic        clk,
    input logic        rst,
    input logic [63:0] tdata,
    input logic [7:0]  tkeep,
    input logic        tvalid,
    input logic        tlast,
    input logic        tready
);
  longint unsigned cyc = 0;
  longint unsigned beats = 0, pkts = 0, stalled = 0;
  longint unsigned keep_on_last[256];
  longint unsigned first_beat_cyc = 0, last_beat_cyc = 0;
  logic [63:0] first_tdata = 64'hDEAD_DEAD_DEAD_DEAD;
  logic [63:0] last_tdata  = 64'hDEAD_DEAD_DEAD_DEAD;
  logic [7:0]  last_keep   = 8'hFF;
  // beats of the FIRST packet only -- the CfgRd0 is the first TLP either stack
  // ever sends, so its own beat count must not be averaged with later traffic.
  longint unsigned first_pkt_beats = 0;
  logic            first_pkt_done  = 1'b0;

  initial for (int i = 0; i < 256; i++) keep_on_last[i] = 0;

  always @(posedge clk) begin
    if (!rst) begin
      cyc <= cyc + 1;
      // a beat is valid AND ready; valid-without-ready is a stall, counted apart
      if (tvalid && !tready) stalled <= stalled + 1;
      if (tvalid && tready) begin
        beats <= beats + 1;
        if (beats == 0) begin
          first_beat_cyc <= cyc;
          first_tdata    <= tdata;
        end
        last_beat_cyc <= cyc;
        last_tdata    <= tdata;
        if (!first_pkt_done) first_pkt_beats <= first_pkt_beats + 1;
        if (tlast) begin
          pkts <= pkts + 1;
          keep_on_last[tkeep] <= keep_on_last[tkeep] + 1;
          last_keep <= tkeep;
          if (!first_pkt_done) first_pkt_done <= 1'b1;
        end
      end
    end
  end

  task automatic report(input string scope);
    $display("PR7E_AXIS %s %s beats=%0d pkts=%0d stall_cycles=%0d first_beat_cyc=%0d last_beat_cyc=%0d first_pkt_beats=%0d first_tdata=0x%016h last_tdata=0x%016h last_keep=0x%0h",
             scope, NAME, beats, pkts, stalled, first_beat_cyc, last_beat_cyc,
             first_pkt_beats, first_tdata, last_tdata, last_keep);
    for (int i = 0; i < 256; i++)
      if (keep_on_last[i] != 0)
        $display("PR7E_AXIS %s %s tkeep_on_tlast[0x%0h]=%0d", scope, NAME, i,
                 keep_on_last[i]);
  endtask
endmodule

// -----------------------------------------------------------------------------
// The Data Link Layer, both instances. Five seams + the sequence-number and
// first-TLP flags.
// -----------------------------------------------------------------------------
module pr7e_dll #(
    parameter int P_DATA_WIDTH = 0,
    parameter int P_KEEP_WIDTH = 0,
    parameter int P_USER_WIDTH = 0
) (
    input logic clk,
    input logic rst,

    input logic fc_init,
    input logic link_up_ish,

    // (1)/(4) TL -> DLL : the CfgRd0 on the RC, the CplD on the EP
    input logic [63:0] s_tlp_tdata,  input logic [7:0] s_tlp_tkeep,
    input logic s_tlp_tvalid, input logic s_tlp_tlast, input logic s_tlp_tready,
    // (1) DLL -> PHY : what phy_transmit is handed
    input logic [63:0] m_phy_tdata,  input logic [7:0] m_phy_tkeep,
    input logic m_phy_tvalid, input logic m_phy_tlast, input logic m_phy_tready,
    // (3)/(5) PHY -> DLL : the shared stream, TLP and DLLP together
    input logic [63:0] s_phy_tdata,  input logic [7:0] s_phy_tkeep,
    input logic s_phy_tvalid, input logic s_phy_tlast, input logic s_phy_tready,
    // (3)/(5) the TLP ARM split out of that stream -- the discriminating seam
    input logic [63:0] tlparm_tdata, input logic [7:0] tlparm_tkeep,
    input logic tlparm_tvalid, input logic tlparm_tlast, input logic tlparm_tready,
    // (4)/(5) DLL -> TL : the received TLP delivered upward
    input logic [63:0] m_tlp_tdata,  input logic [7:0] m_tlp_tkeep,
    input logic m_tlp_tvalid, input logic m_tlp_tlast, input logic m_tlp_tready,

    input logic [11:0] seq_num,
    input logic        seq_num_vld,
    input logic        seq_num_acknack,
    input logic        first_tlp_valid
);
  pr7e_axis #(.NAME("TX_TL2DLL")) u_s_tlp (.clk(clk), .rst(rst), .tdata(s_tlp_tdata),
      .tkeep(s_tlp_tkeep), .tvalid(s_tlp_tvalid), .tlast(s_tlp_tlast), .tready(s_tlp_tready));
  pr7e_axis #(.NAME("TX_DLL2PHY")) u_m_phy (.clk(clk), .rst(rst), .tdata(m_phy_tdata),
      .tkeep(m_phy_tkeep), .tvalid(m_phy_tvalid), .tlast(m_phy_tlast), .tready(m_phy_tready));
  pr7e_axis #(.NAME("RX_PHY2DLL")) u_s_phy (.clk(clk), .rst(rst), .tdata(s_phy_tdata),
      .tkeep(s_phy_tkeep), .tvalid(s_phy_tvalid), .tlast(s_phy_tlast), .tready(s_phy_tready));
  // ⚠️ NAME CORRECTED AFTER RUN 1. This seam was first labelled "RX_TLP_ARM" on
  // the assumption that phy_tlp_axis_* was the TLP arm split out of the RECEIVE
  // stream. It is not: pcie_datalink_layer.sv:243-248 drives it from tlp2dllp's
  // m_axis_*, and :343 feeds it into arbiter_mux_inst whose output is
  // m_phy_axis_* -- it is the TRANSMIT TLP after seq+LCRC framing, pre-mux. The
  // run-1 numbers are unaffected (a counter does not care what it is called) but
  // the label was, and a wrong name in an evidence file outlives the run.
  pr7e_axis #(.NAME("TX_TLP_FRAMED")) u_tlparm (.clk(clk), .rst(rst), .tdata(tlparm_tdata),
      .tkeep(tlparm_tkeep), .tvalid(tlparm_tvalid), .tlast(tlparm_tlast), .tready(tlparm_tready));
  pr7e_axis #(.NAME("RX_DLL2TL")) u_m_tlp (.clk(clk), .rst(rst), .tdata(m_tlp_tdata),
      .tkeep(m_tlp_tkeep), .tvalid(m_tlp_tvalid), .tlast(m_tlp_tlast), .tready(m_tlp_tready));

  longint unsigned cyc = 0;
  longint unsigned fc_first = 0, seqvld = 0, acknack = 0, firsttlp = 0;
  longint unsigned firsttlp_first = 0, seqvld_first = 0;
  logic [11:0] max_seq = 0;

  initial
    $display("PR7E_PARAM %m DATA_WIDTH=%0d KEEP_WIDTH=%0d USER_WIDTH=%0d",
             P_DATA_WIDTH, P_KEEP_WIDTH, P_USER_WIDTH);

  always @(posedge clk) begin
    if (!rst) begin
      cyc <= cyc + 1;
      if (fc_init && fc_first == 0) fc_first <= cyc;
      if (seq_num_vld) begin
        seqvld <= seqvld + 1;
        if (seqvld_first == 0) seqvld_first <= cyc;
        if (seq_num > max_seq) max_seq <= seq_num;
      end
      if (seq_num_acknack) acknack <= acknack + 1;
      if (first_tlp_valid) begin
        firsttlp <= firsttlp + 1;
        if (firsttlp_first == 0) firsttlp_first <= cyc;
      end
    end
  end

  final begin
    $display("PR7E_DLL %m cycles=%0d fc_init_first=%0d seq_num_vld=%0d seq_vld_first=%0d max_seq=%0d acknack=%0d first_tlp_valid_cycles=%0d first_tlp_first=%0d",
             cyc, fc_first, seqvld, seqvld_first, max_seq, acknack, firsttlp,
             firsttlp_first);
    u_s_tlp.report($sformatf("%m"));
    u_m_phy.report($sformatf("%m"));
    u_s_phy.report($sformatf("%m"));
    u_tlparm.report($sformatf("%m"));
    u_m_tlp.report($sformatf("%m"));
  end
endmodule

// -----------------------------------------------------------------------------
// Stage (3)'s LCRC question, at the module that answers it: dllp2tlp compares
// crc_from_tlp_r against crc_calculated_r and raises tlp_nullified_o.
// -----------------------------------------------------------------------------
module pr7e_lcrc (
    input logic        clk,
    input logic        rst,
    input logic        s_tvalid,
    input logic        s_tlast,
    input logic        s_tready,
    input logic        m_tvalid,
    input logic        m_tlast,
    input logic        nullified,
    input logic [31:0] crc_rx,
    input logic [31:0] crc_calc,
    input logic [15:0] next_seq
);
  longint unsigned cyc = 0;
  longint unsigned in_beats = 0, in_pkts = 0, out_beats = 0, out_pkts = 0;
  longint unsigned null_cycles = 0, null_first = 0;
  // the LCRC verdict, sampled on the cycle the inbound packet ENDS
  longint unsigned lcrc_match = 0, lcrc_mismatch = 0;
  logic [31:0] first_crc_rx = 32'hDEAD_BEEF, first_crc_calc = 32'hDEAD_BEEF;
  logic        sampled = 1'b0;
  longint unsigned max_next_seq = 0;

  always @(posedge clk) begin
    if (!rst) begin
      cyc <= cyc + 1;
      if (s_tvalid && s_tready) begin
        in_beats <= in_beats + 1;
        if (s_tlast) begin
          in_pkts <= in_pkts + 1;
          if (crc_rx == crc_calc) lcrc_match <= lcrc_match + 1;
          else lcrc_mismatch <= lcrc_mismatch + 1;
          if (!sampled) begin
            first_crc_rx   <= crc_rx;
            first_crc_calc <= crc_calc;
            sampled        <= 1'b1;
          end
        end
      end
      if (m_tvalid) begin
        out_beats <= out_beats + 1;
        if (m_tlast) out_pkts <= out_pkts + 1;
      end
      if (nullified) begin
        null_cycles <= null_cycles + 1;
        if (null_first == 0) null_first <= cyc;
      end
      if (next_seq > max_next_seq) max_next_seq <= next_seq;
    end
  end

  final
    $display("PR7E_LCRC %m cycles=%0d in_beats=%0d in_pkts=%0d out_beats=%0d out_pkts=%0d lcrc_match=%0d lcrc_mismatch=%0d nullified_cycles=%0d nullified_first=%0d first_crc_rx=0x%08h first_crc_calc=0x%08h max_next_transmit_seq=%0d",
             cyc, in_beats, in_pkts, out_beats, out_pkts, lcrc_match,
             lcrc_mismatch, null_cycles, null_first, first_crc_rx,
             first_crc_calc, max_next_seq);
endmodule

// -----------------------------------------------------------------------------
// Stage (2) -- STP/END across the bridge. Counted on the PIPE stream at BOTH
// ends: phy_transmit's output (RC side, before the encoder) and phy_receive's
// input (EP side, after the decoder). The DLLP census did SDP/END; this does
// STP/END, and keeps SDP so the two framings can be told apart.
//
// Base 2.1 §4.2.2: STP = K27.7 = 0xFB, END = K29.7 = 0xFD, SDP = K28.2 = 0x5C,
// EDB = K30.7 = 0xFE, COM = K28.5 = 0xBC.
// -----------------------------------------------------------------------------
module pr7e_kcode #(
    parameter int P_LANES      = 1,
    parameter int P_DATA_WIDTH = 32
) (
    input logic                             clk,
    input logic                             rst,
    input logic                             link_up,
    input logic [(P_LANES*P_DATA_WIDTH)-1:0] data,
    input logic [P_LANES-1:0]               valid,
    input logic [(4*P_LANES)-1:0]           kflag
);
  localparam int NBYTE = 4 * P_LANES;

  longint unsigned cyc = 0, valid_beats = 0;
  longint unsigned n_stp = 0, n_end = 0, n_sdp = 0, n_edb = 0, n_com = 0;
  longint unsigned n_k_total = 0, n_k_other = 0;
  longint unsigned stp_first = 0, end_first = 0;
  // STP..END pairing: a TLP is delimited by one of each. A count that matches
  // but PAIRS wrong is still a loss, so track the open/close balance too.
  longint unsigned stp_open = 0, max_stp_open = 0, unbalanced_end = 0;

  always @(posedge clk) begin
    if (!rst) begin
      cyc <= cyc + 1;
      for (int lane = 0; lane < P_LANES; lane++) begin
        if (valid[lane]) begin
          if (lane == 0) valid_beats <= valid_beats + 1;
          for (int b = 0; b < 4; b++) begin
            if (kflag[lane*4+b]) begin
              automatic logic [7:0] sym = data[(lane*P_DATA_WIDTH)+(b*8) +: 8];
              n_k_total <= n_k_total + 1;
              case (sym)
                8'hFB: begin
                  n_stp <= n_stp + 1;
                  if (stp_first == 0) stp_first <= cyc;
                  stp_open <= stp_open + 1;
                  if (stp_open + 1 > max_stp_open) max_stp_open <= stp_open + 1;
                end
                8'hFD: begin
                  n_end <= n_end + 1;
                  if (end_first == 0) end_first <= cyc;
                  if (stp_open == 0) unbalanced_end <= unbalanced_end + 1;
                  else stp_open <= stp_open - 1;
                end
                8'h5C: n_sdp <= n_sdp + 1;
                8'hFE: n_edb <= n_edb + 1;
                8'hBC: n_com <= n_com + 1;
                default: n_k_other <= n_k_other + 1;
              endcase
            end
          end
        end
      end
    end
  end

  final
    $display("PR7E_KCODE %m cycles=%0d valid_beats=%0d STP=%0d END=%0d SDP=%0d EDB=%0d COM=%0d k_total=%0d k_other=%0d stp_first=%0d end_first=%0d stp_unclosed=%0d end_unopened=%0d",
             cyc, valid_beats, n_stp, n_end, n_sdp, n_edb, n_com, n_k_total,
             n_k_other, stp_first, end_first, stp_open, unbalanced_end);
endmodule

// =============================================================================
// bind statements -- file scope. Each module below exists in BOTH stacks, so
// every bind fires twice and %m says which.
// =============================================================================

bind pcie_datalink_layer pr7e_dll #(
    .P_DATA_WIDTH(DATA_WIDTH),
    .P_KEEP_WIDTH(KEEP_WIDTH),
    .P_USER_WIDTH(USER_WIDTH)
) u_pr7e_dll (
    .clk        (clk_i),
    .rst        (rst_i),
    .fc_init    (fc_initialized_o),
    .link_up_ish(1'b1),

    .s_tlp_tdata (64'(s_tlp_axis_tdata)),  .s_tlp_tkeep (8'(s_tlp_axis_tkeep)),
    .s_tlp_tvalid(s_tlp_axis_tvalid), .s_tlp_tlast(s_tlp_axis_tlast),
    .s_tlp_tready(s_tlp_axis_tready),

    .m_phy_tdata (64'(m_phy_axis_tdata)),  .m_phy_tkeep (8'(m_phy_axis_tkeep)),
    .m_phy_tvalid(m_phy_axis_tvalid), .m_phy_tlast(m_phy_axis_tlast),
    .m_phy_tready(m_phy_axis_tready),

    .s_phy_tdata (64'(s_phy_axis_tdata)),  .s_phy_tkeep (8'(s_phy_axis_tkeep)),
    .s_phy_tvalid(s_phy_axis_tvalid), .s_phy_tlast(s_phy_axis_tlast),
    .s_phy_tready(s_phy_axis_tready),

    .tlparm_tdata (64'(phy_tlp_axis_tdata)), .tlparm_tkeep(8'(phy_tlp_axis_tkeep)),
    .tlparm_tvalid(phy_tlp_axis_tvalid), .tlparm_tlast(phy_tlp_axis_tlast),
    .tlparm_tready(phy_tlp_axis_tready),

    .m_tlp_tdata (64'(m_tlp_axis_tdata)),  .m_tlp_tkeep (8'(m_tlp_axis_tkeep)),
    .m_tlp_tvalid(m_tlp_axis_tvalid), .m_tlp_tlast(m_tlp_axis_tlast),
    .m_tlp_tready(m_tlp_axis_tready),

    .seq_num        (seq_num),
    .seq_num_vld    (seq_num_vld),
    .seq_num_acknack(seq_num_acknack),
    .first_tlp_valid(first_tlp_valid)
);

// -----------------------------------------------------------------------------
// ⚠️ ADDED AFTER RUN 1 -- the seam run 1 proved was missing.
//
// Run 1 measured 250 packets / 516 beats arriving at the RC's
// pcie_datalink_layer.s_phy_axis_*, and 16 beats / ZERO packets arriving at that
// same stack's dllp2tlp.s_axis_*. Something between those two points drops the
// Completion, and run 1 had no instrument between them. dllp_receive IS that
// span: its s_axis_* is the DLL's inbound stream and it fans out to tlp_axis_*
// (-> dllp2tlp), tlp_to_mac_* (-> pcie_cfg_wrapper) and m_axis_dllp2tlp_*.
// -----------------------------------------------------------------------------
module pr7e_rxdemux (
    input logic clk,
    input logic rst,
    input logic [63:0] s_tdata,   input logic [7:0] s_tkeep,
    input logic s_tvalid, input logic s_tlast, input logic s_tready,
    input logic [63:0] tlp_tdata, input logic [7:0] tlp_tkeep,
    input logic tlp_tvalid, input logic tlp_tlast, input logic tlp_tready,
    input logic [63:0] mac_tdata, input logic [7:0] mac_tkeep,
    input logic mac_tvalid, input logic mac_tlast, input logic mac_tready,
    input logic [63:0] cpl_tdata, input logic [7:0] cpl_tkeep,
    input logic cpl_tvalid, input logic cpl_tlast, input logic cpl_tready,
    input logic [63:0] up_tdata,  input logic [7:0] up_tkeep,
    input logic up_tvalid, input logic up_tlast, input logic up_tready
);
  pr7e_axis #(.NAME("RXD_IN")) u_in (.clk(clk), .rst(rst), .tdata(s_tdata),
      .tkeep(s_tkeep), .tvalid(s_tvalid), .tlast(s_tlast), .tready(s_tready));
  pr7e_axis #(.NAME("RXD_TO_DLLP2TLP")) u_tlp (.clk(clk), .rst(rst), .tdata(tlp_tdata),
      .tkeep(tlp_tkeep), .tvalid(tlp_tvalid), .tlast(tlp_tlast), .tready(tlp_tready));
  pr7e_axis #(.NAME("RXD_TO_CFGWRAP")) u_mac (.clk(clk), .rst(rst), .tdata(mac_tdata),
      .tkeep(mac_tkeep), .tvalid(mac_tvalid), .tlast(mac_tlast), .tready(mac_tready));
  pr7e_axis #(.NAME("RXD_CPL_FROM_CFG")) u_cpl (.clk(clk), .rst(rst), .tdata(cpl_tdata),
      .tkeep(cpl_tkeep), .tvalid(cpl_tvalid), .tlast(cpl_tlast), .tready(cpl_tready));
  pr7e_axis #(.NAME("RXD_UP_TO_TL")) u_up (.clk(clk), .rst(rst), .tdata(up_tdata),
      .tkeep(up_tkeep), .tvalid(up_tvalid), .tlast(up_tlast), .tready(up_tready));

  final begin
    u_in.report($sformatf("%m"));
    u_tlp.report($sformatf("%m"));
    u_mac.report($sformatf("%m"));
    u_cpl.report($sformatf("%m"));
    u_up.report($sformatf("%m"));
  end
endmodule

bind dllp_receive pr7e_rxdemux u_pr7e_rxdemux (
    .clk(clk_i), .rst(rst_i),
    .s_tdata (64'(s_axis_tdata)),  .s_tkeep (8'(s_axis_tkeep)),
    .s_tvalid(s_axis_tvalid), .s_tlast(s_axis_tlast), .s_tready(s_axis_tready),
    .tlp_tdata (64'(tlp_axis_tdata)), .tlp_tkeep(8'(tlp_axis_tkeep)),
    .tlp_tvalid(tlp_axis_tvalid), .tlp_tlast(tlp_axis_tlast), .tlp_tready(tlp_axis_tready),
    .mac_tdata (64'(tlp_to_mac_tdata)), .mac_tkeep(8'(tlp_to_mac_tkeep)),
    .mac_tvalid(tlp_to_mac_tvalid), .mac_tlast(tlp_to_mac_tlast), .mac_tready(tlp_to_mac_tready),
    .cpl_tdata (64'(m_cpl_from_cfg_tdata)), .cpl_tkeep(8'(m_cpl_from_cfg_tkeep)),
    .cpl_tvalid(m_cpl_from_cfg_tvalid), .cpl_tlast(m_cpl_from_cfg_tlast),
    .cpl_tready(m_cpl_from_cfg_tready),
    .up_tdata (64'(m_axis_dllp2tlp_tdata)), .up_tkeep(8'(m_axis_dllp2tlp_tkeep)),
    .up_tvalid(m_axis_dllp2tlp_tvalid), .up_tlast(m_axis_dllp2tlp_tlast),
    .up_tready(m_axis_dllp2tlp_tready)
);

// -----------------------------------------------------------------------------
// ⚠️ ADDED AFTER RUN 2 -- inside the module runs 1 and 2 named.
//
// Run 2: the RC's axis_user_demux takes 516 beats / 250 packets in and emits 16
// beats / ZERO packets on its TLP arm, where 24 beats / 4 packets are due. The
// EP's identical instance emits 20 / 4 correctly.
//
// Two candidate mechanisms are visible in the module's interface, and READING
// CANNOT CHOOSE BETWEEN THEM (§63 #7c: five causal stories from RTL, five wrong).
// So this probe measures both, plus the null hypothesis:
//   (a) ROUTING -- s_axis_tuser[1]/[0] mis-selects, or changes mid-packet
//   (b) READY MISMATCH -- ST_IDLE drives s_axis_tready from m_tlp_axis_tready
//       (the far side of the skid buffer) while the skid buffer's own slave
//       ready is tlp_ready. When those two disagree, a beat is either handshaked
//       upstream and not captured (LOST) or captured twice (DUPLICATED).
//   (c) neither -- the loss is downstream in the skid buffer itself.
// The counters below are differential: each mechanism has a signature no other
// one produces.
// -----------------------------------------------------------------------------
module pr7e_demux (
    input logic clk,
    input logic rst,
    input logic [2:0] state,
    input logic [7:0] s_tuser,
    input logic s_tvalid, input logic s_tready, input logic s_tlast,
    input logic tlp_valid, input logic tlp_ready, input logic m_tlp_ready,
    input logic dllp_valid, input logic dllp_ready, input logic m_dllp_ready,
    input logic m_tlp_valid, input logic m_tlp_last
);
  localparam int NST = 8;
  longint unsigned cyc = 0;
  longint unsigned in_beats = 0, in_last = 0;
  longint unsigned tlp_accepted = 0, dllp_accepted = 0;
  longint unsigned m_tlp_beats = 0, m_tlp_last_n = 0;
  // (b)'s signature, split by direction of the disagreement
  longint unsigned rdy_mismatch = 0;      // tlp_ready != m_tlp_axis_tready, any cycle
  longint unsigned lost_beats = 0;        // handshaked upstream, NOT captured by skid
  longint unsigned dup_beats = 0;         // captured by skid, NOT handshaked upstream
  longint unsigned lost_last = 0;         // and the lost beat carried tlast
  longint unsigned occ[NST], ent[NST];
  longint unsigned tuser_first[256];      // tuser on the beat that leaves ST_IDLE
  longint unsigned tuser_all[256];
  logic [2:0] prev = 3'h7;

  initial for (int i = 0; i < NST; i++) begin occ[i] = 0; ent[i] = 0; end
  initial for (int i = 0; i < 256; i++) begin tuser_first[i] = 0; tuser_all[i] = 0; end

  always @(posedge clk) begin
    if (!rst) begin
      cyc <= cyc + 1;
      occ[state] <= occ[state] + 1;
      if (state != prev) ent[state] <= ent[state] + 1;
      prev <= state;

      if (s_tvalid) tuser_all[s_tuser] <= tuser_all[s_tuser] + 1;
      if (s_tvalid && state == 3'd0) tuser_first[s_tuser] <= tuser_first[s_tuser] + 1;

      if (s_tvalid && s_tready) begin
        in_beats <= in_beats + 1;
        if (s_tlast) in_last <= in_last + 1;
      end
      if (tlp_valid && tlp_ready) tlp_accepted <= tlp_accepted + 1;
      if (dllp_valid && dllp_ready) dllp_accepted <= dllp_accepted + 1;
      if (m_tlp_valid) begin
        m_tlp_beats <= m_tlp_beats + 1;
        if (m_tlp_last) m_tlp_last_n <= m_tlp_last_n + 1;
      end

      if (tlp_ready != m_tlp_ready) rdy_mismatch <= rdy_mismatch + 1;
      // the two differential signatures of mechanism (b), on the TLP arm
      if (s_tvalid && s_tready && tlp_valid && !tlp_ready) begin
        lost_beats <= lost_beats + 1;
        if (s_tlast) lost_last <= lost_last + 1;
      end
      if (s_tvalid && !s_tready && tlp_valid && tlp_ready)
        dup_beats <= dup_beats + 1;
    end
  end

  final begin
    $display("PR7E_DEMUX %m cycles=%0d in_beats=%0d in_tlast=%0d tlp_accepted=%0d dllp_accepted=%0d m_tlp_beats=%0d m_tlp_tlast=%0d",
             cyc, in_beats, in_last, tlp_accepted, dllp_accepted, m_tlp_beats,
             m_tlp_last_n);
    $display("PR7E_DEMUX %m READY_MISMATCH_cycles=%0d LOST_beats=%0d LOST_carrying_tlast=%0d DUP_beats=%0d",
             rdy_mismatch, lost_beats, lost_last, dup_beats);
    for (int i = 0; i < NST; i++)
      if (ent[i] != 0)
        $display("PR7E_DEMUX %m state[%0d] entries=%0d occupancy=%0d", i, ent[i], occ[i]);
    for (int i = 0; i < 256; i++)
      if (tuser_all[i] != 0)
        $display("PR7E_DEMUX %m tuser_all[0x%0h]=%0d", i, tuser_all[i]);
    for (int i = 0; i < 256; i++)
      if (tuser_first[i] != 0)
        $display("PR7E_DEMUX %m tuser_in_ST_IDLE[0x%0h]=%0d", i, tuser_first[i]);
  end
endmodule

// -----------------------------------------------------------------------------
// ⚠️ ADDED AFTER RUN 3 -- the demux was EXONERATED, so this goes upstream.
//
// Run 3: the RC's axis_user_demux receives only 16 beats tagged TLP
// (tuser_all[0x2]=16) and NONE of them carries tlast; it forwards all 16 and
// asserts tlast zero times. tlp_accepted == tuser_all[0x2] == m_tlp_beats on
// BOTH stacks, so the demux passes exactly what it is handed. The EP's instance
// receives 20 TLP beats with 4 tlasts. The truncation happens BEFORE the DLL.
//
// ⚠️ IT ALSO REFUTED MY OWN PHASE-1 ARITHMETIC. Phase 1 inferred "4 CplDs of 6
// beats arrive complete" from 250 pkts / 516 beats by assuming 246 two-beat
// DLLPs. The true split is 250 DLLPs (500 beats) + 16 orphan TLP beats. Beat
// arithmetic over a MIXED stream has more than one solution and I published the
// wrong one; only a tuser-classified count distinguishes them.
//
// data_handler is the last module before the DLL (phy_receive.sv:263-289 ->
// axis_async_fifo -> m_dllp_axis_*). ST_TX has TWO END-detection loops:
//   loop 1 (:241) END in THIS word, guarded by (BytesPerTransfer - word_count_r) > byte_idx
//   loop 2 (:268) END in the REGISTERED word, guarded by !data_start_r
// This probe re-evaluates both guards in the bench and counts, per arm and per
// packet class, which one fires -- and crucially how often an END is present
// and NEITHER fires. That count is the defect if it is nonzero on the RC.
// -----------------------------------------------------------------------------
module pr7e_dh #(
    parameter int P_DATA_WIDTH = 32,
    parameter int P_BPT        = 4
) (
    input logic clk,
    input logic rst,
    input logic tready,
    input logic dvalid,
    input logic [31:0] data_i,
    input logic [3:0]  k_i,
    input logic [31:0] data_r,
    input logic [3:0]  k_r,
    input logic [5:0]  word_count,
    input logic        data_start,
    input logic        is_tlp,
    input logic        is_dllp,
    input logic        ax_tvalid,
    input logic        ax_tlast,
    input logic [3:0]  ax_tkeep,
    // ⚠️ ADDED AFTER RUN 4. Run 4 found tlp_fire=0 AND tlp_MISS=0 on the RC --
    // the END is not merely rejected by loop 1's guard, it is never PRESENT
    // while is_tlp_r holds. But run 4's loops sit inside `tready && dvalid`,
    // which is ST_TX's condition, so they are BLIND to an END arriving in
    // ST_CHECK_END (:335) or ST_TX_TLP (:265). "Never seen" was therefore a
    // claim my instrument could not support. These two count END occurrences
    // UNCONDITIONALLY, tagged by state, which can.
    input logic [4:0]  state
);
  longint unsigned cyc = 0, active = 0;
  // END seen in the CURRENT word, split by whether loop 1's guard admits it
  longint unsigned e1_tlp_fire = 0, e1_tlp_miss = 0;
  longint unsigned e1_dllp_fire = 0, e1_dllp_miss = 0;
  // END seen in the REGISTERED word, split by loop 2's guard
  longint unsigned e2_tlp_fire = 0, e2_tlp_block = 0;
  longint unsigned e2_dllp_fire = 0, e2_dllp_block = 0;
  // the joint miss: an END exists and NEITHER loop can act on it
  longint unsigned orphan_tlp = 0, orphan_dllp = 0;
  // where the misses land, so the guard can be read against real operands
  longint unsigned miss_wc[64], miss_bi[8];
  longint unsigned tlp_beats = 0, tlp_last = 0, dllp_beats = 0, dllp_last = 0;
  // run-5 additions: unconditional END/STP census by state
  localparam int NST = 32;
  longint unsigned st_occ[NST], st_ent[NST];
  longint unsigned end_in_state_tlp[NST], end_in_state_dllp[NST];
  // §63 #7g-1: these two are UNGATED -- they count CYCLES A SYMBOL SITS ON THE
  // BUS, not symbols, because the loop below is inside `!rst` and not inside
  // `dvalid`.  §63 #7e registered that as a bench defect.  The fix is NOT to
  // replace them: §63 #7h established that the two windows answer different
  // questions -- *ungated = what the wire carries, valid-gated = what was
  // sent* -- and #7e's published END/STP numbers were taken with this window.
  // So the ungated pair keeps its meaning under a name that states it, and a
  // valid-gated pair is added beside it.  Comparing the two IS the measurement:
  // equal means every symbol was presented for exactly one cycle.
  longint unsigned end_seen_total = 0, stp_seen_total = 0;      // cycles, ungated
  longint unsigned end_symbols_gated = 0, stp_symbols_gated = 0; // symbols, dvalid-gated
  longint unsigned istlp_cycles = 0, isdllp_cycles = 0;
  logic [4:0] prev_st = 5'h1F;

  initial begin
    for (int i = 0; i < 64; i++) miss_wc[i] = 0;
    for (int i = 0; i < 8; i++) miss_bi[i] = 0;
    for (int i = 0; i < NST; i++) begin
      st_occ[i] = 0; st_ent[i] = 0;
      end_in_state_tlp[i] = 0; end_in_state_dllp[i] = 0;
    end
  end

  always @(posedge clk) begin
    if (!rst) begin
      cyc <= cyc + 1;
      st_occ[state] <= st_occ[state] + 1;
      if (state != prev_st) st_ent[state] <= st_ent[state] + 1;
      prev_st <= state;
      if (is_tlp) istlp_cycles <= istlp_cycles + 1;
      if (is_dllp) isdllp_cycles <= isdllp_cycles + 1;
      // unconditional: where is the END when it goes past?  (cycles, not symbols)
      for (int b = 0; b < P_BPT; b++) begin
        if (k_i[b] && (data_i[8*b+:8] == 8'hFD || data_i[8*b+:8] == 8'hFE)) begin
          end_seen_total <= end_seen_total + 1;
          if (is_tlp) end_in_state_tlp[state] <= end_in_state_tlp[state] + 1;
          else end_in_state_dllp[state] <= end_in_state_dllp[state] + 1;
        end
        if (k_i[b] && data_i[8*b+:8] == 8'hFB) stp_seen_total <= stp_seen_total + 1;
      end
      // §63 #7g-1: the same census, gated by dvalid -- SYMBOLS.  One cycle of a
      // held symbol counts once here and N times above.
      if (dvalid) begin
        for (int b = 0; b < P_BPT; b++) begin
          if (k_i[b] && (data_i[8*b+:8] == 8'hFD || data_i[8*b+:8] == 8'hFE))
            end_symbols_gated <= end_symbols_gated + 1;
          if (k_i[b] && data_i[8*b+:8] == 8'hFB)
            stp_symbols_gated <= stp_symbols_gated + 1;
        end
      end
      if (ax_tvalid) begin
        if (is_tlp) begin
          tlp_beats <= tlp_beats + 1;
          if (ax_tlast) tlp_last <= tlp_last + 1;
        end else begin
          dllp_beats <= dllp_beats + 1;
          if (ax_tlast) dllp_last <= dllp_last + 1;
        end
      end
      if (tready && dvalid) begin
        automatic bit any_e1 = 0, any_e1_ok = 0, any_e2 = 0, any_e2_ok = 0;
        active <= active + 1;
        for (int b = 0; b < P_BPT; b++) begin
          if (k_i[b] && (data_i[8*b+:8] == 8'hFD || data_i[8*b+:8] == 8'hFE)) begin
            any_e1 = 1;
            if ((P_BPT - word_count) > b) begin
              any_e1_ok = 1;
              if (is_tlp) e1_tlp_fire <= e1_tlp_fire + 1;
              else e1_dllp_fire <= e1_dllp_fire + 1;
            end else begin
              if (is_tlp) e1_tlp_miss <= e1_tlp_miss + 1;
              else e1_dllp_miss <= e1_dllp_miss + 1;
              miss_wc[word_count] <= miss_wc[word_count] + 1;
              miss_bi[b] <= miss_bi[b] + 1;
            end
          end
          if (k_r[b] && (data_r[8*b+:8] == 8'hFD || data_r[8*b+:8] == 8'hFE)) begin
            any_e2 = 1;
            if (!data_start) begin
              any_e2_ok = 1;
              if (is_tlp) e2_tlp_fire <= e2_tlp_fire + 1;
              else e2_dllp_fire <= e2_dllp_fire + 1;
            end else begin
              if (is_tlp) e2_tlp_block <= e2_tlp_block + 1;
              else e2_dllp_block <= e2_dllp_block + 1;
            end
          end
        end
        if ((any_e1 || any_e2) && !any_e1_ok && !any_e2_ok) begin
          if (is_tlp) orphan_tlp <= orphan_tlp + 1;
          else orphan_dllp <= orphan_dllp + 1;
        end
      end
    end
  end

  final begin
    $display("PR7E_DH %m cycles=%0d active=%0d tlp_beats=%0d tlp_tlast=%0d dllp_beats=%0d dllp_tlast=%0d",
             cyc, active, tlp_beats, tlp_last, dllp_beats, dllp_last);
    $display("PR7E_DH %m LOOP1(cur word) tlp_fire=%0d tlp_MISS=%0d dllp_fire=%0d dllp_MISS=%0d",
             e1_tlp_fire, e1_tlp_miss, e1_dllp_fire, e1_dllp_miss);
    $display("PR7E_DH %m LOOP2(reg word) tlp_fire=%0d tlp_BLOCKED=%0d dllp_fire=%0d dllp_BLOCKED=%0d",
             e2_tlp_fire, e2_tlp_block, e2_dllp_fire, e2_dllp_block);
    $display("PR7E_DH %m ORPHAN_END_tlp=%0d ORPHAN_END_dllp=%0d  <-- an END no loop can act on",
             orphan_tlp, orphan_dllp);
    for (int i = 0; i < 64; i++)
      if (miss_wc[i] != 0) $display("PR7E_DH %m miss_word_count_r[%0d]=%0d", i, miss_wc[i]);
    for (int i = 0; i < 8; i++)
      if (miss_bi[i] != 0) $display("PR7E_DH %m miss_byte_idx[%0d]=%0d", i, miss_bi[i]);
    $display("PR7E_DHST %m END_seen_total=%0d STP_seen_total=%0d is_tlp_cycles=%0d is_dllp_cycles=%0d",
             end_seen_total, stp_seen_total, istlp_cycles, isdllp_cycles);
    // §63 #7g-1: the dvalid-gated pair.  Printed as its own line so the ungated
    // line above keeps the exact form #7e's analyses parse.
    $display("PR7E_DHSYM %m END_symbols=%0d STP_symbols=%0d (dvalid-gated; compare against END/STP_seen_total above)",
             end_symbols_gated, stp_symbols_gated);
    for (int i = 0; i < NST; i++)
      if (st_ent[i] != 0)
        $display("PR7E_DHST %m state[%0d] entries=%0d occupancy=%0d END_while_tlp=%0d END_while_dllp=%0d",
                 i, st_ent[i], st_occ[i], end_in_state_tlp[i], end_in_state_dllp[i]);
  end
endmodule

bind data_handler pr7e_dh #(
    .P_DATA_WIDTH(DATA_WIDTH),
    .P_BPT       (DATA_WIDTH / 8)
) u_pr7e_dh (
    .clk(clk_i), .rst(rst_i),
    .tready(data_handler_axis_tready),
    .dvalid(|data_valid_i),
    .data_i(32'(data_i)), .k_i(4'(data_k_i)),
    .data_r(32'(data_r)), .k_r(4'(data_k_r)),
    .word_count(word_count_r),
    .data_start(data_start_r),
    .is_tlp(is_tlp_r), .is_dllp(is_dllp_r),
    .ax_tvalid(data_handler_axis_tvalid),
    .ax_tlast (data_handler_axis_tlast),
    .ax_tkeep (4'(data_handler_axis_tkeep)),
    .state    (5'(curr_state))
);

bind axis_user_demux pr7e_demux u_pr7e_demux (
    .clk(clk_i), .rst(rst_i),
    .state(3'(curr_state)),
    .s_tuser(8'(s_axis_tuser)),
    .s_tvalid(s_axis_tvalid), .s_tready(s_axis_tready), .s_tlast(s_axis_tlast),
    .tlp_valid(tlp_valid), .tlp_ready(tlp_ready), .m_tlp_ready(m_tlp_axis_tready),
    .dllp_valid(dllp_valid), .dllp_ready(dllp_ready), .m_dllp_ready(m_dllp_axis_tready),
    .m_tlp_valid(m_tlp_axis_tvalid), .m_tlp_last(m_tlp_axis_tlast)
);

bind dllp2tlp pr7e_lcrc u_pr7e_lcrc (
    .clk      (clk_i),
    .rst      (rst_i),
    .s_tvalid (s_axis_tvalid),
    .s_tlast  (s_axis_tlast),
    .s_tready (s_axis_tready),
    .m_tvalid (m_tlp_axis_tvalid),
    .m_tlast  (m_tlp_axis_tlast),
    .nullified(tlp_nullified_o),
    .crc_rx   (crc_from_tlp_r),
    .crc_calc (crc_calculated_r),
    .next_seq (next_transmit_seq_o)
);

// The RC's transmitted PIPE stream, before the bridge encodes it.
bind phy_transmit pr7e_kcode #(
    .P_LANES     (MAX_NUM_LANES),
    .P_DATA_WIDTH(DATA_WIDTH)
) u_pr7e_ktx (
    .clk    (pipe_tx_usr_clk_i),
    .rst    (rst_i),
    .link_up(link_up_i),
    .data   (pipe_data_o),
    .valid  (pipe_data_valid_o),
    .kflag  (pipe_data_k_o)
);

// The EP's received PIPE stream, after the bridge decodes it.
bind phy_receive pr7e_kcode #(
    .P_LANES     (MAX_NUM_LANES),
    .P_DATA_WIDTH(DATA_WIDTH)
) u_pr7e_krx (
    .clk    (pipe_rx_usr_clk_i),
    .rst    (rst_i),
    .link_up(link_up_i),
    .data   (pipe_data_i),
    .valid  (pipe_data_valid_i),
    .kflag  (pipe_data_k_i)
);
