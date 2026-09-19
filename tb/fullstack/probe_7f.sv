// =============================================================================
// §63 #7f Phase 1 -- instrument 1-L: the LATENCY LADDER. BENCH-ONLY, `bind`.
//
// !! NOTHING IN src/ CHANGES FOR THIS FILE, and verilate_fullstack is untouched:
// own fileset, own target (§63 #7d's rule, kept since). A probe that perturbs
// the row it measures is not a measurement of that row.
//
// !! WRITTEN WITHOUT READING ANY MODULE BODY (D-7F.1). Every name below came
// from a PORT DECLARATION extracted mechanically, or from a constant in
// pcie_phy_pkg. Predictions F18-a/b, H1-H5 and C1-C5 were committed first
// (pcie_docs 3af3881, PREDICTIONS_7F.md). Knowing a wire's NAME is not a causal
// story about what drives it.
//
// == WHAT THIS MEASURES =====================================================
//
// #7e measured the round trip at FIVE stages and got 2,359 cycles one way. That
// number is ~20x the spec's entire Ack Latency budget (Base 2.1 Table 3-6: 237
// Symbol Times = 948 ns for x1/MPS128), which is why the link replays every TLP
// while running a replay timer already 3.83x ABOVE the spec ceiling
// (FINDINGS_7F_PHASE0 §4). This probe splits that 2,359 into per-stage deltas so
// the dominant stage can be NAMED rather than inferred.
//
// == THREE RULES THIS FILE OBEYS, ALL EARNED IN #7e ==========================
//
// (1) NO CUMULATIVE `final` COUNTERS. #7e's `final`-block counters summed tests
//     whose windows differ 6x and produced a published claim that had to be
//     withdrawn. Every measurement here is an EVENT LINE emitted at the instant
//     of the event, carrying $time. Windowing is done offline, per test, from
//     the timestamps. The only `final` output is a census of what was SUPPRESSED
//     (see rule 3) and an explicitly-windowed occupancy statistic.
//
// (2) CLASSIFY BY FRAMING, NEVER BY BEAT ARITHMETIC. "Beat arithmetic over a
//     mixed TLP/DLLP stream is a hypothesis with a free parameter." At AXIS
//     seams that means tuser. At SYMBOL seams -- where there is no tuser -- it
//     means the START framing symbol, which is exactly as discriminating:
//     STP (0xFB, K) opens a TLP, SDP (0x5C, K) opens a DLLP, END (0xFD, K)
//     closes either. K-codes are NOT scrambled, so this identity survives every
//     stage of the ladder including the ciphertext ones. That is what makes a
//     per-stage ladder possible at all.
//
// (3) NO SILENT CAPS. Each stamper emits at most MAX_EV event lines PER EVENT
//     CLASS. It then keeps COUNTING and reports the suppressed total per class
//     in `final`. A bounded log that does not say it is bounded reads as "this
//     is everything".
//
// !! ⚠️ RULE 3 IS PER-CLASS, AND RUN 1 IS WHY. The first version of this file
// had ONE budget of 12 shared by every framing symbol. The result: 120 SDP,
// 120 END, and **zero STP** across 28 stampers -- every captured event was a
// DLLP, because FC init emits InitFC DLLPs continuously while the CfgRd0 emits
// exactly one STP, so the noisy class exhausted the budget before the
// interesting one arrived. Read literally, that log says *no TLP ever reaches
// the PIPE seam*, which is false and is exactly the confident-false-negative
// HANDSHAKE §5 warns about ("AN UNDER-BUDGETED CAPTURE PRODUCES A CONFIDENT
// FALSE NEGATIVE POINTING AT THE WRONG MODULE").
//
// ⭐ The general form, which is new and belongs in the record: **a budget shared
// across event classes of different natural rates is not a budget, it is a
// filter selecting the most frequent class.** Budget per class, or measure
// nothing about the rare one.
//
// !! K-CODE DETECTION IS GATED BY data_valid. #7e's probe counted "cycles a
// symbol sits on the bus", not symbols, and the resulting K-code numbers meant
// nothing. Every symbol stamper below gates on its own valid.
//
// !! x1 LINK: lane 0 only. Every symbol seam is sliced [31:0]/[3:0]. Stated
// rather than assumed -- on a wider link these binds would need a lane loop.
// =============================================================================
`timescale 1ns / 1ps

// -----------------------------------------------------------------------------
// A SYMBOL seam: 32 bits of data, 4 K flags, one valid. Emits one line per
// framing event (STP / SDP / END) seen in any of the four byte lanes.
//
// Also accumulates a *windowed* occupancy statistic: valid cycles vs total
// cycles. This is the measurement that settles how many Symbol Times a cycle is
// worth -- PREDICTIONS_7F §D. It is reported WITH its window, because an
// occupancy without its window is the #7e counter mistake wearing a ratio.
// -----------------------------------------------------------------------------
module pr7f_sym #(
    parameter string NAME     = "?",
    parameter int    MAX_TLP  = 24,  // STP + its closing END. Rare: ~1 per TLP.
    parameter int    MAX_DLLP = 6    // SDP + its closing END. Continuous.
) (
    input logic        clk,
    input logic        rst,
    input logic [31:0] data,
    input logic [3:0]  k,
    input logic        valid
);
  localparam logic [7:0] SYM_STP = 8'hFB;
  localparam logic [7:0] SYM_SDP = 8'h5C;
  localparam logic [7:0] SYM_END = 8'hFD;
  localparam logic [7:0] SYM_EDB = 8'hFE;

  longint unsigned cyc = 0;
  longint unsigned tlp_emit = 0, tlp_supp = 0;
  longint unsigned dllp_emit = 0, dllp_supp = 0;
  longint unsigned valid_cyc = 0, win_start = 0;
  logic            win_open = 1'b0;
  // An END closes whichever packet is open. Without this flag an END cannot be
  // attributed, and an unattributable END is what forces beat arithmetic.
  logic            in_tlp = 1'b0;

  always @(posedge clk) begin
    if (!rst) begin
      cyc <= cyc + 1;
      if (valid) begin
        valid_cyc <= valid_cyc + 1;
        if (!win_open) begin
          win_open  <= 1'b1;
          win_start <= cyc;
        end
        for (int b = 0; b < 4; b++) begin
          if (k[b]) begin
            case (data[b*8+:8])
              SYM_STP: begin
                in_tlp <= 1'b1;
                if (tlp_emit < MAX_TLP) begin
                  tlp_emit <= tlp_emit + 1;
                  $display("PR7F_L %s scope=%m cls=TLP sym=STP lane_byte=%0d t=%0t cyc=%0d data=0x%08h k=0x%01h",
                           NAME, b, $time, cyc, data, k);
                end else tlp_supp <= tlp_supp + 1;
              end
              SYM_SDP: begin
                in_tlp <= 1'b0;
                if (dllp_emit < MAX_DLLP) begin
                  dllp_emit <= dllp_emit + 1;
                  $display("PR7F_L %s scope=%m cls=DLLP sym=SDP lane_byte=%0d t=%0t cyc=%0d data=0x%08h k=0x%01h",
                           NAME, b, $time, cyc, data, k);
                end else dllp_supp <= dllp_supp + 1;
              end
              SYM_END, SYM_EDB: begin
                if (in_tlp) begin
                  in_tlp <= 1'b0;
                  if (tlp_emit < MAX_TLP) begin
                    tlp_emit <= tlp_emit + 1;
                    $display("PR7F_L %s scope=%m cls=TLP sym=%s lane_byte=%0d t=%0t cyc=%0d data=0x%08h k=0x%01h",
                             NAME, (data[b*8+:8] == SYM_EDB) ? "EDB" : "END", b,
                             $time, cyc, data, k);
                  end else tlp_supp <= tlp_supp + 1;
                end else begin
                  if (dllp_emit < MAX_DLLP) begin
                    dllp_emit <= dllp_emit + 1;
                    $display("PR7F_L %s scope=%m cls=DLLP sym=%s lane_byte=%0d t=%0t cyc=%0d data=0x%08h k=0x%01h",
                             NAME, (data[b*8+:8] == SYM_EDB) ? "EDB" : "END", b,
                             $time, cyc, data, k);
                  end else dllp_supp <= dllp_supp + 1;
                end
              end
              default: ;  // COM/SKP/PAD and every other K -- not a framing event
            endcase
          end
        end
      end
    end
  end

  final begin
    // Rule 3, per class. Rule 1: the occupancy carries its window, because an
    // occupancy without its window is the #7e counter mistake wearing a ratio.
    $display("PR7F_LSUM %s scope=%m tlp_emit=%0d tlp_supp=%0d dllp_emit=%0d dllp_supp=%0d valid_cyc=%0d total_cyc=%0d win_start=%0d",
             NAME, tlp_emit, tlp_supp, dllp_emit, dllp_supp, valid_cyc, cyc,
             win_start);
  end
endmodule

// -----------------------------------------------------------------------------
// An AXIS seam. Emits one line on the FIRST BEAT of each packet, carrying tuser
// (the mandated classifier) and the first data word (the identity anchor), and
// one on tlast. Beats are valid AND ready; valid-without-ready is a stall and is
// counted separately rather than folded in.
// -----------------------------------------------------------------------------
// !! tuser IS THE MANDATED CLASSIFIER and it is ALSO the budget key. Rather than
// assume what the encoding means -- which would be reading a module body, and
// D-7F.1 forbids it -- each distinct tuser value gets its OWN budget. Whatever
// the classes turn out to be, no class can starve another. Eight buckets keyed
// on tuser[2:0]; collisions above 3 bits are disclosed by the per-bucket counts.
module pr7f_axis #(
    parameter string NAME   = "?",
    parameter int    MAX_EV = 16   // PER tuser BUCKET, not per stamper
) (
    input logic        clk,
    input logic        rst,
    input logic [63:0] tdata,
    input logic [7:0]  tkeep,
    input logic        tvalid,
    input logic        tlast,
    input logic        tready,
    input logic [7:0]  tuser
);
  longint unsigned cyc = 0, stalled = 0, pkts = 0;
  longint unsigned emitted[8], suppressed[8];
  logic            in_pkt = 1'b0;
  logic [2:0]      bkt;

  initial
    for (int i = 0; i < 8; i++) begin
      emitted[i]    = 0;
      suppressed[i] = 0;
    end

  always @(posedge clk) begin
    if (!rst) begin
      cyc <= cyc + 1;
      if (tvalid && !tready) stalled <= stalled + 1;
      if (tvalid && tready) begin
        bkt = tuser[2:0];
        if (!in_pkt) begin
          in_pkt <= 1'b1;
          if (emitted[bkt] < MAX_EV) begin
            emitted[bkt] <= emitted[bkt] + 1;
            $display("PR7F_L %s scope=%m cls=U%0d sym=SOP t=%0t cyc=%0d tdata=0x%016h tuser=0x%02h tkeep=0x%02h",
                     NAME, bkt, $time, cyc, tdata, tuser, tkeep);
          end else suppressed[bkt] <= suppressed[bkt] + 1;
        end
        if (tlast) begin
          in_pkt <= 1'b0;
          pkts   <= pkts + 1;
          if (emitted[bkt] < MAX_EV) begin
            emitted[bkt] <= emitted[bkt] + 1;
            $display("PR7F_L %s scope=%m cls=U%0d sym=EOP t=%0t cyc=%0d tdata=0x%016h tuser=0x%02h tkeep=0x%02h",
                     NAME, bkt, $time, cyc, tdata, tuser, tkeep);
          end else suppressed[bkt] <= suppressed[bkt] + 1;
        end
      end
    end
  end

  final begin
    for (int i = 0; i < 8; i++)
      if (emitted[i] != 0 || suppressed[i] != 0)
        $display("PR7F_LSUM %s scope=%m tuser_bucket=%0d emitted=%0d suppressed=%0d",
                 NAME, i, emitted[i], suppressed[i]);
    $display("PR7F_LSUM %s scope=%m pkts=%0d stall_cyc=%0d total_cyc=%0d", NAME,
             pkts, stalled, cyc);
  end
endmodule

// -----------------------------------------------------------------------------
// A CYCLE-ACCURATE WINDOW, opened by a signal and bounded in extent.
//
// !! WHY THIS EXISTS. The ladder above samples only when `valid` is high, and
// that makes two very different worlds indistinguishable:
//
//   (a) the END symbol is DELAYED -- it genuinely arrives ~965 cycles late; or
//   (b) the END symbol is presented ON TIME with `valid` LOW, the bus then holds
//       the stale value, and the next valid-high cycle re-presents it.
//
// Both produce "first END seen at cycle 7750". They are opposite claims about
// the DUT, and (b) is the F16 family -- #7c's `block_alignment` defect was
// exactly a valid/data disagreement across idle. A valid-gated probe cannot
// settle its own gating question, so this window logs EVERY cycle, valid or
// not, for a bounded span after the first STP.
//
// The window OPENS ON A SIGNAL (the first STP at this stage) rather than at a
// hardcoded cycle -- §22.89's second limb -- and its extent is bounded so the
// log stays finite. It arms ONCE.
// -----------------------------------------------------------------------------
module pr7f_trace #(
    parameter string NAME = "?",
    parameter int    SPAN = 28
) (
    input logic        clk,
    input logic        rst,
    input logic [31:0] data,
    input logic [3:0]  k,
    input logic        valid
);
  localparam logic [7:0] SYM_STP = 8'hFB;
  longint unsigned cyc = 0;
  int              left = -1;   // -1 = not armed yet, 0 = finished
  logic            armed = 1'b0;

  always @(posedge clk) begin
    if (!rst) begin
      cyc <= cyc + 1;
      if (!armed && valid) begin
        for (int b = 0; b < 4; b++)
          if (k[b] && data[b*8+:8] == SYM_STP) begin
            armed <= 1'b1;
            left  <= SPAN;
          end
      end else if (armed && left > 0) begin
        left <= left - 1;
        // EVERY cycle, valid or not. That is the whole point.
        $display("PR7F_T %s scope=%m cyc=%0d valid=%0b data=0x%08h k=0x%01h",
                 NAME, cyc, valid, data, k);
      end
    end
  end
endmodule

// The two stages that bracket the first stretch: the scrambler's own faces.
bind scrambler pr7f_trace #(.NAME("T_SCRAM_IN")) u_pr7f_tr_in (
    .clk(clk_i), .rst(rst_i), .data(data_in_i), .k(data_k_in_i),
    .valid(data_valid_i)
);
bind scrambler pr7f_trace #(.NAME("T_SCRAM_OUT")) u_pr7f_tr_out (
    .clk(clk_i), .rst(rst_i), .data(data_out_o), .k(data_k_out_o),
    .valid(data_valid_o)
);

// =============================================================================
// THE LADDER, STAGE BY STAGE.
//
// Each bind is by MODULE TYPE, so it fires at BOTH stacks (and, for scrambler,
// at both the TX and RX instance of each). %m says which -- the #7e idiom. The
// RC/EP split is therefore a property of the log, not of the instrument, and no
// hierarchical path is hardcoded anywhere in this file.
// =============================================================================

// -- TX: the DLL hands the PHY a framed TLP -----------------------------------
bind frame_symbols pr7f_axis #(.NAME("TX1_FRAME_IN")) u_pr7f_fs_in (
    .clk(clk_i), .rst(rst_i),
    .tdata({32'h0, s_axis_tdata}), .tkeep({4'h0, s_axis_tkeep}),
    .tvalid(s_axis_tvalid), .tlast(s_axis_tlast), .tready(s_axis_tready),
    .tuser({{(8-USER_WIDTH){1'b0}}, s_axis_tuser})
);
bind frame_symbols pr7f_axis #(.NAME("TX2_FRAME_OUT")) u_pr7f_fs_out (
    .clk(clk_i), .rst(rst_i),
    .tdata({32'h0, m_axis_tdata}), .tkeep({4'h0, m_axis_tkeep}),
    .tvalid(m_axis_tvalid), .tlast(m_axis_tlast), .tready(m_axis_tready),
    .tuser({{(8-USER_WIDTH){1'b0}}, m_axis_tuser})
);

// -- TX/RX: the scrambler pair. Four instances across the two stacks. ---------
// !! ROW 4a WARNING (HANDSHAKE §6): the scrambler publishes 3 fabricated beats
// after reset. Those will appear in this log as early events with no framing
// symbol; they are expected and are NOT ladder stages. Gated by data_valid,
// which is what keeps them from polluting the occupancy statistic.
bind scrambler pr7f_sym #(.NAME("TX3_SCRAM_IN")) u_pr7f_sc_in (
    .clk(clk_i), .rst(rst_i), .data(data_in_i), .k(data_k_in_i), .valid(data_valid_i)
);
bind scrambler pr7f_sym #(.NAME("TX4_SCRAM_OUT")) u_pr7f_sc_out (
    .clk(clk_i), .rst(rst_i), .data(data_out_o), .k(data_k_out_o), .valid(data_valid_o)
);

// -- TX: the PIPE seam, our side ---------------------------------------------
bind phy_transmit pr7f_sym #(.NAME("TX5_PIPE_OUT")) u_pr7f_pipe_tx (
    .clk(clk_i), .rst(rst_i),
    .data(pipe_data_o[31:0]), .k(pipe_data_k_o[3:0]), .valid(pipe_data_valid_o[0])
);

// -- THE BRIDGE: H3's number, measured directly at both its faces -------------
// H3 predicts this holds < 10 % of the one-way latency. It is a codec with no
// gearbox, so that is the expectation -- but two of #7e's faults were bench
// faults, so it is measured FIRST and not assumed.
bind pipe_codec_bridge pr7f_sym #(.NAME("BR1_BRIDGE_IN")) u_pr7f_br_in (
    .clk(clk_i), .rst(rst_i),
    .data(a_txdata_i[31:0]), .k(a_txdatak_i[3:0]), .valid(a_txdata_valid_i[0])
);
bind pipe_codec_bridge pr7f_sym #(.NAME("BR2_BRIDGE_OUT")) u_pr7f_br_out (
    .clk(clk_i), .rst(rst_i),
    .data(a_rxdata_o[31:0]), .k(a_rxdatak_o[3:0]), .valid(a_rxdata_valid_o[0])
);

// -- RX: the far PIPE seam ----------------------------------------------------
bind phy_receive pr7f_sym #(.NAME("RX6_PIPE_IN")) u_pr7f_pipe_rx (
    .clk(clk_i), .rst(rst_i),
    .data(pipe_data_i[31:0]), .k(pipe_data_k_i[3:0]), .valid(pipe_data_valid_i[0])
);

// -- RX: block_alignment in/out ----------------------------------------------
// !! #7c PROVED THIS MODULE PERFORMS NO ALIGNMENT and that rst_i does not reset
// it. Both faces are stamped precisely because its pass-through behaviour is
// already characterised -- if a delta appears here it is new information.
bind block_alignment pr7f_sym #(.NAME("RX7_ALIGN_IN")) u_pr7f_ba_in (
    .clk(clk_i), .rst(rst_i),
    .data(data_i[31:0]), .k(data_k_i[3:0]), .valid(data_valid_i[0])
);
bind block_alignment pr7f_sym #(.NAME("RX8_ALIGN_OUT")) u_pr7f_ba_out (
    .clk(clk_i), .rst(rst_i),
    .data(data_o[31:0]), .k(data_k_o[3:0]), .valid(data_valid_o[0])
);

// -- RX: data_handler in, and the AXIS stream it produces ---------------------
bind data_handler pr7f_sym #(.NAME("RX9_DH_IN")) u_pr7f_dh_in (
    .clk(clk_i), .rst(rst_i),
    .data(data_i[31:0]), .k(data_k_i[3:0]), .valid(data_valid_i[0])
);
bind data_handler pr7f_axis #(.NAME("RX10_DH_OUT")) u_pr7f_dh_out (
    .clk(clk_i), .rst(rst_i),
    .tdata({32'h0, m_dllp_axis_tdata}), .tkeep({4'h0, m_dllp_axis_tkeep}),
    .tvalid(m_dllp_axis_tvalid), .tlast(m_dllp_axis_tlast),
    .tready(m_dllp_axis_tready),
    .tuser({{(8-USER_WIDTH){1'b0}}, m_dllp_axis_tuser})
);

// -- RX: the PHY hands the DLL the shared TLP+DLLP stream ---------------------
bind phy_receive pr7f_axis #(.NAME("RX11_PHY2DLL")) u_pr7f_phy2dll (
    .clk(clk_i), .rst(rst_i),
    .tdata({32'h0, m_dllp_axis_tdata}), .tkeep({4'h0, m_dllp_axis_tkeep}),
    .tvalid(m_dllp_axis_tvalid), .tlast(m_dllp_axis_tlast),
    .tready(m_dllp_axis_tready),
    .tuser({{(8-USER_WIDTH){1'b0}}, m_dllp_axis_tuser})
);

// =============================================================================
// §63 #7f Phase 1 -- (a) POSITIVE CONTROL, (b) TERMINATION+LCRC, 1-R, 1-B.
//
// Added after Kourosh's challenge retired the first root cause. Same rules:
// event lines at the event, per-class budgets, windows opened by a signal.
// =============================================================================

// -- (a) POSITIVE CONTROL -----------------------------------------------------
// The SAME every-cycle window, armed on a DLLP's SDP instead of a TLP's STP.
// A DLLP survives the link (FC init completes, so DLLPs demonstrably arrive),
// so this is the control that says whether the TLP's two-burst delivery is
// SPECIFIC to TLPs or is just what this PHY does to everything.
// !! WITHOUT THIS CONTROL the TLP trace has no baseline and any claim from it
// is 22.80's error -- a measurement with no comparison is not evidence.
module pr7f_ctrl #(
    parameter string NAME = "?",
    parameter int    SPAN = 36
) (
    input logic        clk,
    input logic        rst,
    input logic [31:0] data,
    input logic [3:0]  k,
    input logic        valid
);
  localparam logic [7:0] SYM_SDP = 8'h5C;
  longint unsigned cyc = 0;
  int              left = -1;
  logic            armed = 1'b0;
  always @(posedge clk) begin
    if (!rst) begin
      cyc <= cyc + 1;
      if (!armed && valid) begin
        for (int b = 0; b < 4; b++)
          if (k[b] && data[b*8+:8] == SYM_SDP) begin armed <= 1'b1; left <= SPAN; end
      end else if (armed && left > 0) begin
        left <= left - 1;
        $display("PR7F_C %s scope=%m cyc=%0d valid=%0b data=0x%08h k=0x%01h",
                 NAME, cyc, valid, data, k);
      end
    end
  end
endmodule

bind scrambler pr7f_ctrl #(.NAME("C_SCRAM_IN")) u_pr7f_c_in (
    .clk(clk_i), .rst(rst_i), .data(data_in_i), .k(data_k_in_i), .valid(data_valid_i));
bind scrambler pr7f_ctrl #(.NAME("C_SCRAM_OUT")) u_pr7f_c_out (
    .clk(clk_i), .rst(rst_i), .data(data_out_o), .k(data_k_out_o), .valid(data_valid_o));

// -- (b) WHAT TERMINATES THE TLP, AND WHETHER ITS LCRC PASSES ------------------
// dllp2tlp is where the receive LCRC verdict is formed. Logged as an EVENT per
// completed packet -- never a cumulative count -- so the CfgRd0's own verdict
// can be read rather than an average over the run.
module pr7f_lcrc (
    input logic clk, rst,
    input logic s_tvalid, s_tlast, s_tready,
    input logic m_tvalid, m_tlast,
    input logic nullified,
    input logic [31:0] crc_rx, crc_calc
);
  longint unsigned cyc = 0, n_in = 0, n_up = 0;
  always @(posedge clk) begin
    if (!rst) begin
      cyc <= cyc + 1;
      if (s_tvalid && s_tready && s_tlast) begin
        n_in <= n_in + 1;
        $display("PR7F_B scope=%m ev=RX_PKT_END n=%0d cyc=%0d nullified=%0b crc_rx=0x%08h crc_calc=0x%08h match=%0b",
                 n_in + 1, cyc, nullified, crc_rx, crc_calc, (crc_rx == crc_calc));
      end
      if (m_tvalid && m_tlast) begin
        n_up <= n_up + 1;
        $display("PR7F_B scope=%m ev=UP_TO_TL n=%0d cyc=%0d", n_up + 1, cyc);
      end
    end
  end
endmodule

bind dllp2tlp pr7f_lcrc u_pr7f_lcrc (
    .clk(clk_i), .rst(rst_i),
    .s_tvalid(s_axis_tvalid), .s_tlast(s_axis_tlast), .s_tready(s_axis_tready),
    .m_tvalid(m_tlp_axis_tvalid), .m_tlast(m_tlp_axis_tlast),
    .nullified(tlp_nullified_o), .crc_rx(crc_from_tlp_r), .crc_calc(crc_calculated_r));

// -- 1-R: THE ACK LEDGER -------------------------------------------------------
// Per accepted TLP and per Ack: seq, cycle, direction (via %m). The replay
// machine's own view -- what it sent, what it was acked for, what it replayed.
module pr7f_ack (
    input logic clk, rst,
    input logic [11:0] tx_seq, input logic tx_vld,
    input logic ack_nack, input logic ack_vld, input logic [11:0] ack_seq,
    input logic retry_avail, input logic retry_err
);
  longint unsigned cyc = 0, n_tx = 0, n_ack = 0, n_nak = 0;
  always @(posedge clk) begin
    if (!rst) begin
      cyc <= cyc + 1;
      if (tx_vld) begin
        n_tx <= n_tx + 1;
        $display("PR7F_R scope=%m ev=TX_TLP n=%0d cyc=%0d seq=%0d", n_tx+1, cyc, tx_seq);
      end
      if (ack_vld) begin
        if (ack_nack) n_nak <= n_nak + 1; else n_ack <= n_ack + 1;
        $display("PR7F_R scope=%m ev=%s cyc=%0d ack_seq=%0d retry_avail=%0b retry_err=%0b",
                 ack_nack ? "NAK_RX" : "ACK_RX", cyc, ack_seq, retry_avail, retry_err);
      end
    end
  end
  final $display("PR7F_RSUM scope=%m tx=%0d ack=%0d nak=%0d cyc=%0d", n_tx, n_ack, n_nak, cyc);
endmodule

bind retry_management pr7f_ack u_pr7f_ack (
    .clk(clk_i), .rst(rst_i),
    .tx_seq(tx_seq_num_i), .tx_vld(tx_valid_i),
    .ack_nack(ack_nack_i), .ack_vld(ack_nack_vld_i), .ack_seq(ack_seq_num_i),
    .retry_avail(retry_available_o), .retry_err(retry_err_o));

// -- 1-B: THE STALL TRACE ------------------------------------------------------
// !! THE ONE OBSERVATION THAT DECIDES C5 vs F18-a. pcie_enum_bar carries a
// dedicated err_credit_blocked_o, so "is this credit starvation?" is a wire,
// not an inference.
module pr7f_bar (
    input logic clk, rst,
    input logic busy, done, err, input logic [3:0] ecode,
    input logic credit_blocked, input logic fc_blocked,
    input logic [3:0] bar_count, input logic [5:0] bar_valid,
    input logic cmd_vld, cmd_rdy, cmd_wr, input logic [5:0] cmd_reg,
    input logic [31:0] cmd_wdata,
    input logic rsp_vld, input logic [1:0] rsp_out, input logic [31:0] rsp_data
);
  longint unsigned cyc = 0, n_cmd = 0, n_rsp = 0;
  logic [3:0] last_cnt = 4'hF;
  always @(posedge clk) begin
    if (!rst) begin
      cyc <= cyc + 1;
      if (cmd_vld && cmd_rdy) begin
        n_cmd <= n_cmd + 1;
        $display("PR7F_BAR scope=%m ev=CMD n=%0d cyc=%0d wr=%0b reg=0x%02h wdata=0x%08h bar_count=%0d fc_blocked=%0b credit_blocked=%0b",
                 n_cmd+1, cyc, cmd_wr, cmd_reg, cmd_wdata, bar_count, fc_blocked, credit_blocked);
      end
      if (rsp_vld) begin
        n_rsp <= n_rsp + 1;
        $display("PR7F_BAR scope=%m ev=RSP n=%0d cyc=%0d outcome=%0d rdata=0x%08h",
                 n_rsp+1, cyc, rsp_out, rsp_data);
      end
      if (bar_count != last_cnt) begin
        last_cnt <= bar_count;
        $display("PR7F_BAR scope=%m ev=COUNT cyc=%0d bar_count=%0d bar_valid=0x%02h", cyc, bar_count, bar_valid);
      end
      if (err)
        $display("PR7F_BAR scope=%m ev=ERR cyc=%0d code=%0d credit_blocked=%0b fc_blocked=%0b bar_count=%0d",
                 cyc, ecode, credit_blocked, fc_blocked, bar_count);
    end
  end
  final $display("PR7F_BARSUM scope=%m cmds=%0d rsps=%0d busy=%0b done=%0b err=%0b code=%0d credit_blocked=%0b bar_count=%0d bar_valid=0x%02h",
                 n_cmd, n_rsp, busy, done, err, ecode, credit_blocked, bar_count, bar_valid);
endmodule

bind pcie_enum_bar pr7f_bar u_pr7f_bar (
    .clk(clk_i), .rst(rst_i),
    .busy(bar_busy_o), .done(enum_done_o), .err(bar_error_o), .ecode(bar_error_code_o),
    .credit_blocked(err_credit_blocked_o), .fc_blocked(tx_fc_blocked_i),
    .bar_count(bar_count_o), .bar_valid(bar_valid_o),
    .cmd_vld(cmd_valid_o), .cmd_rdy(cmd_ready_i), .cmd_wr(cmd_write_o),
    .cmd_reg(cmd_reg_num_o), .cmd_wdata(cmd_wdata_o),
    .rsp_vld(rsp_valid_i), .rsp_out(rsp_outcome_i), .rsp_data(rsp_rdata_i));

// The credit side of the same question: what was ADVERTISED (Phase 0 deferred
// this deliberately rather than tracing it) and what is AVAILABLE at the stall.
module pr7f_credit (
    input logic clk, rst, input logic fc_init, fc_upd,
    input logic [7:0] ph, input logic [11:0] pd,
    input logic [7:0] nph, input logic [11:0] npd,
    input logic [7:0] cplh, input logic [11:0] cpld,
    input logic req_vld, req_rdy, input logic [1:0] req_cls,
    input logic blocked,
    input logic [7:0] nph_avail, input logic [7:0] ph_avail
);
  longint unsigned cyc = 0, n_req = 0, n_blk = 0;
  logic seen_init = 1'b0;
  always @(posedge clk) begin
    if (!rst) begin
      cyc <= cyc + 1;
      if (fc_init && !seen_init) begin
        seen_init <= 1'b1;
        $display("PR7F_CR scope=%m ev=FC_INIT cyc=%0d ADVERTISED ph=%0d pd=%0d nph=%0d npd=%0d cplh=%0d cpld=%0d",
                 cyc, ph, pd, nph, npd, cplh, cpld);
      end
      if (fc_upd)
        $display("PR7F_CR scope=%m ev=FC_UPDATE cyc=%0d ph=%0d pd=%0d nph=%0d npd=%0d cplh=%0d cpld=%0d",
                 cyc, ph, pd, nph, npd, cplh, cpld);
      if (req_vld && req_rdy) begin
        n_req <= n_req + 1;
        $display("PR7F_CR scope=%m ev=REQ n=%0d cyc=%0d class=%0d nph_avail=%0d ph_avail=%0d",
                 n_req+1, cyc, req_cls, nph_avail, ph_avail);
      end
      if (req_vld && !req_rdy) n_blk <= n_blk + 1;
    end
  end
  final $display("PR7F_CRSUM scope=%m reqs=%0d blocked_cyc=%0d blocked_now=%0b nph_avail=%0d ph_avail=%0d cyc=%0d",
                 n_req, n_blk, blocked, nph_avail, ph_avail, cyc);
endmodule

bind tlp_credit_manager pr7f_credit u_pr7f_cr (
    .clk(clk_i), .rst(rst_i), .fc_init(fc_initialized_i), .fc_upd(fc_update_valid_i),
    .ph(fc_ph_i), .pd(fc_pd_i), .nph(fc_nph_i), .npd(fc_npd_i),
    .cplh(fc_cplh_i), .cpld(fc_cpld_i),
    .req_vld(request_valid_i), .req_rdy(request_ready_o), .req_cls(request_class_i),
    .blocked(blocked_o),
    .nph_avail(nonposted_header_available_o), .ph_avail(posted_header_available_o));

// =============================================================================
// §63 #7f Phase 2b -- LCRC oracle capture, Nak decision point, DLLP type
// census, and the tready holder. MEASUREMENT ONLY. Predictions committed first
// (pcie_docs a6d80b7).
// =============================================================================

// -- (1) BYTE CAPTURE for the Python LCRC oracle ------------------------------
// Every beat of a TLP, tdata AND tkeep, at two AXIS points. The oracle
// recomputes the LCRC offline and compares against BOTH DUT registers, which is
// what separates class B (field mis-captured) from class C (computation wrong).
// !! tkeep ON EVERY BEAT, not just the last: without it the byte count of the
// final beat is a guess, and a guessed byte count makes the oracle a
// free-parameter hypothesis rather than a check.
// !! ONLY_TLP is a PARAMETER, not a hardcoded filter. At the DLL's TX seam the
// stream is mixed and tuser discriminates; at dllp2tlp's input the stream is
// ALREADY the TLP arm and tuser is one bit wide, so filtering on tuser==2 there
// would match nothing and silently capture zero packets. Same probe, two
// routes, two settings -- §22.85.
module pr7f_bytes #(
    parameter string NAME     = "?",
    parameter int    MAX_PKT  = 4,
    parameter bit    ONLY_TLP = 1'b1
) (
    input logic        clk,
    input logic        rst,
    input logic [31:0] tdata,
    input logic [3:0]  tkeep,
    input logic        tvalid,
    input logic        tlast,
    input logic        tready,
    input logic [7:0]  tuser
);
  longint unsigned cyc = 0, pkt = 0, beat = 0;
  logic            in_pkt = 1'b0;
  always @(posedge clk) begin
    if (!rst) begin
      cyc <= cyc + 1;
      if (tvalid && tready) begin
        if (!in_pkt) begin in_pkt <= 1'b1; beat <= 0; end
        if (pkt < MAX_PKT && (!ONLY_TLP || tuser[2:0] == 3'd2))
          $display("PR7F_BY %s scope=%m pkt=%0d beat=%0d cyc=%0d tdata=0x%08h tkeep=0x%01h tlast=%0b tuser=0x%02h",
                   NAME, pkt, beat, cyc, tdata, tkeep, tlast, tuser);
        beat <= beat + 1;
        if (tlast) begin in_pkt <= 1'b0; pkt <= pkt + 1; end
      end
    end
  end
endmodule

// -- (2) THE NAK DECISION POINT, and (3) the SENT side ------------------------
// Per received packet: the sequence state, the verdict, and both CRC registers
// in ONE line, so LCRC-vs-sequence is read rather than inferred.
module pr7f_verdict (
    input logic clk, rst,
    input logic s_tvalid, s_tlast, s_tready,
    input logic [11:0] next_exp, resp_seq,
    input logic resp_is_nak, nak_sched, adv_exp, nullified,
    input logic [31:0] crc_rx, crc_calc
);
  longint unsigned cyc = 0, n = 0, n_adv = 0, n_nak = 0;
  always @(posedge clk) begin
    if (!rst) begin
      cyc <= cyc + 1;
      if (s_tvalid && s_tready && s_tlast) begin
        n <= n + 1;
        if (adv_exp) n_adv <= n_adv + 1;
        if (resp_is_nak) n_nak <= n_nak + 1;
        $display("PR7F_V scope=%m n=%0d cyc=%0d next_exp=%0d resp_seq=%0d is_nak=%0b nak_sched=%0b adv_exp=%0b nullified=%0b crc_rx=0x%08h crc_calc=0x%08h crcmatch=%0b",
                 n+1, cyc, next_exp, resp_seq, resp_is_nak, nak_sched, adv_exp,
                 nullified, crc_rx, crc_calc, (crc_rx == crc_calc));
      end
    end
  end
  final $display("PR7F_VSUM scope=%m pkts=%0d advanced=%0d nak_verdicts=%0d final_next_exp=%0d",
                 n, n_adv, n_nak, next_exp);
endmodule

bind dllp2tlp pr7f_verdict u_pr7f_v (
    .clk(clk_i), .rst(rst_i),
    .s_tvalid(s_axis_tvalid), .s_tlast(s_axis_tlast), .s_tready(s_axis_tready),
    .next_exp(next_expected_seq_num_r), .resp_seq(response_seq_r),
    .resp_is_nak(response_is_nak_r), .nak_sched(nak_scheduled_r),
    .adv_exp(advance_expected_seq_r), .nullified(tlp_nullified_r),
    .crc_rx(crc_from_tlp_r), .crc_calc(crc_calculated_r));

// -- (4) DLLP TYPE CENSUS at both DLL TX, whole run ---------------------------
// The DLLP's first beat carries its type. Logged raw and classified OFFLINE --
// naming the encoding here would be asserting a mapping I have not measured.
// Answers C9 (any UpdateFC at all?) and the SENT half of C11 in one capture.
module pr7f_dllptype #(parameter string NAME = "?") (
    input logic clk, rst,
    input logic [31:0] tdata, input logic tvalid, tlast, tready,
    input logic [7:0] tuser
);
  longint unsigned cyc = 0, n = 0;
  logic in_pkt = 1'b0;
  always @(posedge clk) begin
    if (!rst) begin
      cyc <= cyc + 1;
      if (tvalid && tready) begin
        if (!in_pkt) begin
          in_pkt <= 1'b1;
          if (tuser[2:0] == 3'd1) begin      // DLLP arm
            n <= n + 1;
            $display("PR7F_D %s scope=%m n=%0d cyc=%0d first=0x%08h tuser=0x%02h",
                     NAME, n+1, cyc, tdata, tuser);
          end
        end
        if (tlast) in_pkt <= 1'b0;
      end
    end
  end
  final $display("PR7F_DSUM %s scope=%m dllps_sent=%0d cyc=%0d", NAME, n, cyc);
endmodule

// -- (5) WHO HOLDS tready LOW (third priority, weak) --------------------------
// frame_symbols is the last AXIS seam before the scrambler. If its m_axis_tready
// is low during the stall, the holder is downstream of it; if high, the stall is
// upstream and C10 loses.
module pr7f_ready #(parameter string NAME = "?") (
    input logic clk, rst, input logic tvalid, tready
);
  longint unsigned cyc = 0, stall = 0, longest = 0, run = 0, at = 0;
  always @(posedge clk) begin
    if (!rst) begin
      cyc <= cyc + 1;
      if (tvalid && !tready) begin
        stall <= stall + 1;
        run   <= run + 1;
        if (run + 1 > longest) begin longest <= run + 1; at <= cyc; end
      end else run <= 0;
    end
  end
  final $display("PR7F_RDY %s scope=%m stall_cyc=%0d longest_run=%0d longest_end_cyc=%0d total=%0d",
                 NAME, stall, longest, at, cyc);
endmodule

bind frame_symbols pr7f_ready #(.NAME("FS_MOUT")) u_pr7f_rdy (
    .clk(clk_i), .rst(rst_i), .tvalid(m_axis_tvalid), .tready(m_axis_tready));

// Capture point 1: the TX DLL-out seam (mixed stream, tuser discriminates).
bind pcie_datalink_layer pr7f_bytes #(.NAME("BY1_TXDLL"), .ONLY_TLP(1'b1)) u_pr7f_by_tx (
    .clk(clk_i), .rst(rst_i),
    .tdata(32'(m_phy_axis_tdata)), .tkeep(4'(m_phy_axis_tkeep)),
    .tvalid(m_phy_axis_tvalid), .tlast(m_phy_axis_tlast),
    .tready(m_phy_axis_tready),
    .tuser({{(8-USER_WIDTH){1'b0}}, m_phy_axis_tuser}));

// Capture point 3: dllp2tlp's input -- ALREADY the TLP arm, so no tuser filter.
bind dllp2tlp pr7f_bytes #(.NAME("BY3_D2TIN"), .ONLY_TLP(1'b0)) u_pr7f_by_rx (
    .clk(clk_i), .rst(rst_i),
    .tdata(32'(s_axis_tdata)), .tkeep(4'(s_axis_tkeep)),
    .tvalid(s_axis_tvalid), .tlast(s_axis_tlast), .tready(s_axis_tready),
    .tuser({{(8-USER_WIDTH){1'b0}}, s_axis_tuser}));

// (4) DLLP type census at the DLL TX, both stacks, whole run.
bind pcie_datalink_layer pr7f_dllptype #(.NAME("TXDLL")) u_pr7f_d (
    .clk(clk_i), .rst(rst_i),
    .tdata(32'(m_phy_axis_tdata)), .tvalid(m_phy_axis_tvalid),
    .tlast(m_phy_axis_tlast), .tready(m_phy_axis_tready),
    .tuser({{(8-USER_WIDTH){1'b0}}, m_phy_axis_tuser}));

// =============================================================================
// §63 #7f Phase 2c -- replay trigger, H9's tail/arrival coincidence, and the
// UpdateFC payloads for #18. MEASUREMENT ONLY. Predictions at pcie_docs 37f871b.
// =============================================================================

// -- (3) REPLAY TRIGGER: timer expiry vs Nak, per replay ----------------------
// 2b measured ZERO Nak DLLPs in the whole run, so any replay observed here is
// timer-driven by elimination. Logged per event anyway, with the Nak input
// beside it on the same line, so the elimination is visible rather than argued.
module pr7f_replay #(parameter int W = 3) (
    input logic clk, rst,
    input logic [W-1:0] retry_valid, input logic retry_err,
    input logic ack_vld, ack_nack, input logic [11:0] ack_seq
);
  longint unsigned cyc = 0, n_rep = 0, n_ack = 0, n_nak = 0;
  logic [W-1:0] prev = '0;
  always @(posedge clk) begin
    if (!rst) begin
      cyc  <= cyc + 1;
      prev <= retry_valid;
      if ((retry_valid & ~prev) != '0) begin
        n_rep <= n_rep + 1;
        $display("PR7F_RP scope=%m ev=REPLAY n=%0d cyc=%0d mask=0x%0h nak_this_cycle=%0b retry_err=%0b",
                 n_rep + 1, cyc, retry_valid & ~prev, (ack_vld && !ack_nack), retry_err);
      end
      if (ack_vld) begin
        if (ack_nack) n_ack <= n_ack + 1; else n_nak <= n_nak + 1;
      end
    end
  end
  // !! POLARITY FIXED HERE: ack_nack==1 is ACK (dllp_handler's Ack arm sets it).
  // Phase 2 had this inverted and published the inverse conclusion.
  final $display("PR7F_RPSUM scope=%m replays=%0d acks_rx=%0d naks_rx=%0d cyc=%0d",
                 n_rep, n_ack, n_nak, cyc);
endmodule

bind retry_management pr7f_replay #(.W(RETRY_TLP_SIZE)) u_pr7f_rp (
    .clk(clk_i), .rst(rst_i),
    .retry_valid(retry_valid_o), .retry_err(retry_err_o),
    .ack_vld(ack_nack_vld_i), .ack_nack(ack_nack_i), .ack_seq(ack_seq_num_i));

// -- (4) H9: does the tail exit on the cycle the next burst enters? -----------
// Logs every RISING edge of valid at a face, plus every END. If H9 holds, an
// output END cycle coincides with an input valid-rise cycle.
// !! The isolated-DLLP control is ALREADY IN HAND: §1's positive control showed
// a lone DLLP on a quiet link crossing with ZERO added stall, which is what H9
// predicts (nothing behind it to push it out). Re-logged here for the same run.
module pr7f_vedge #(parameter string NAME = "?", parameter int MAX_EV = 40) (
    input logic clk, rst, input logic [31:0] data, input logic [3:0] k,
    input logic valid
);
  longint unsigned cyc = 0, n = 0, supp = 0;
  logic prev = 1'b0;
  always @(posedge clk) begin
    if (!rst) begin
      cyc  <= cyc + 1;
      prev <= valid;
      if (valid && !prev) begin
        if (n < MAX_EV) begin
          n <= n + 1;
          $display("PR7F_E %s scope=%m ev=VRISE cyc=%0d data=0x%08h k=0x%01h", NAME, cyc, data, k);
        end else supp <= supp + 1;
      end
      if (!valid && prev) begin
        if (n < MAX_EV) begin
          n <= n + 1;
          $display("PR7F_E %s scope=%m ev=VFALL cyc=%0d", NAME, cyc);
        end else supp <= supp + 1;
      end
    end
  end
  final $display("PR7F_ESUM %s scope=%m edges=%0d suppressed=%0d", NAME, n, supp);
endmodule

bind scrambler pr7f_vedge #(.NAME("E_SCRAM_IN")) u_pr7f_e_in (
    .clk(clk_i), .rst(rst_i), .data(data_in_i), .k(data_k_in_i), .valid(data_valid_i));
bind scrambler pr7f_vedge #(.NAME("E_SCRAM_OUT")) u_pr7f_e_out (
    .clk(clk_i), .rst(rst_i), .data(data_out_o), .k(data_k_out_o), .valid(data_valid_o));

// -- (5) #18: the UpdateFC payloads, all beats -------------------------------
// The first word alone gives the TYPE; HdrFC/DataFC live in the following
// bytes. Captured raw and decoded OFFLINE against the spec's DLLP layout.
module pr7f_fcpay #(parameter string NAME = "?", parameter int MAX_PKT = 24) (
    input logic clk, rst, input logic [31:0] tdata,
    input logic tvalid, tlast, tready, input logic [7:0] tuser
);
  longint unsigned cyc = 0, pkt = 0, beat = 0;
  logic in_pkt = 1'b0, cap = 1'b0;
  always @(posedge clk) begin
    if (!rst) begin
      cyc <= cyc + 1;
      if (tvalid && tready) begin
        if (!in_pkt) begin
          in_pkt <= 1'b1; beat <= 0;
          cap <= (tuser[2:0] == 3'd1) && (pkt < MAX_PKT);
        end
        if ((!in_pkt && (tuser[2:0] == 3'd1) && (pkt < MAX_PKT)) || (in_pkt && cap))
          $display("PR7F_FC %s scope=%m pkt=%0d beat=%0d cyc=%0d w=0x%08h tlast=%0b",
                   NAME, pkt, beat, cyc, tdata, tlast);
        beat <= beat + 1;
        if (tlast) begin in_pkt <= 1'b0; cap <= 1'b0; pkt <= pkt + 1; end
      end
    end
  end
endmodule

bind pcie_datalink_layer pr7f_fcpay #(.NAME("FCPAY")) u_pr7f_fc (
    .clk(clk_i), .rst(rst_i), .tdata(32'(m_phy_axis_tdata)),
    .tvalid(m_phy_axis_tvalid), .tlast(m_phy_axis_tlast), .tready(m_phy_axis_tready),
    .tuser({{(8-USER_WIDTH){1'b0}}, m_phy_axis_tuser}));

// =============================================================================
// §63 #7f Phase 2d -- H9, ARMED. Every budgeted probe from here on prints its
// arming cycle, its remaining budget, and BUDGET_EXHAUSTED at the cycle it runs
// out. An exhausted budget BEFORE the window of interest makes the run VOID,
// not merely incomplete -- Kourosh's rule, earned three times in this rung.
//
// H9: "the tail of a TLP exits each stage on the cycle the next valid burst
// enters it". Arms on the packet's OWN start symbol rather than counting from
// cycle 0, which is what the previous two attempts got wrong.
// =============================================================================
module pr7f_h9 #(
    parameter string NAME    = "?",
    parameter logic [7:0] ARM_SYM = 8'hFB,   // STP for TLPs, SDP for the control
    parameter int    MAX_PKT = 6
) (
    input logic clk, rst,
    input logic [31:0] din,  input logic [3:0] kin,  input logic vin,
    input logic [31:0] dout, input logic [3:0] kout, input logic vout
);
  localparam logic [7:0] SYM_END = 8'hFD;
  longint unsigned cyc = 0, pkt = 0;
  longint unsigned in_end = 0, out_end = 0, next_rise = 0;
  logic armed = 1'b0, seen_in_end = 1'b0, seen_out_end = 1'b0, want_rise = 1'b0;
  logic vin_p = 1'b0;
  logic exhausted = 1'b0;

  function automatic logic has(input logic [31:0] d, input logic [3:0] k,
                               input logic [7:0] s);
    has = 1'b0;
    for (int b = 0; b < 4; b++) if (k[b] && d[b*8+:8] == s) has = 1'b1;
  endfunction

  always @(posedge clk) begin
    if (!rst) begin
      cyc   <= cyc + 1;
      vin_p <= vin;

      if (!armed && pkt < MAX_PKT && vin && has(din, kin, ARM_SYM)) begin
        armed <= 1'b1; seen_in_end <= 1'b0; seen_out_end <= 1'b0;
        want_rise <= 1'b0;
        $display("PR7F_H9 %s scope=%m ev=ARMED pkt=%0d cyc=%0d budget_remaining=%0d",
                 NAME, pkt, cyc, MAX_PKT - pkt - 1);
      end

      if (armed) begin
        // tail ENTERS the stage
        if (!seen_in_end && vin && has(din, kin, SYM_END)) begin
          seen_in_end <= 1'b1; in_end <= cyc; want_rise <= 1'b1;
        end
        // the NEXT valid burst to enter, after the tail
        if (want_rise && vin && !vin_p) begin
          want_rise <= 1'b0; next_rise <= cyc;
        end
        // tail EXITS the stage
        if (!seen_out_end && vout && has(dout, kout, SYM_END)) begin
          seen_out_end <= 1'b1; out_end <= cyc;
        end
        if (seen_in_end && seen_out_end && !want_rise) begin
          armed <= 1'b0; pkt <= pkt + 1;
          $display("PR7F_H9 %s scope=%m ev=RESULT pkt=%0d tail_in=%0d tail_out=%0d next_input_vrise=%0d out_minus_rise=%0d",
                   NAME, pkt, in_end, out_end, next_rise,
                   longint'(out_end) - longint'(next_rise));
        end
      end

      if (pkt >= MAX_PKT && !exhausted) begin
        exhausted <= 1'b1;
        $display("PR7F_H9 %s scope=%m ev=BUDGET_EXHAUSTED cyc=%0d", NAME, cyc);
      end
    end
  end
endmodule

bind scrambler pr7f_h9 #(.NAME("H9_TLP"), .ARM_SYM(8'hFB)) u_pr7f_h9t (
    .clk(clk_i), .rst(rst_i),
    .din(data_in_i),  .kin(data_k_in_i),  .vin(data_valid_i),
    .dout(data_out_o), .kout(data_k_out_o), .vout(data_valid_o));
// The control: an isolated DLLP. §1's positive control already showed zero
// added stall; re-measured here in the same run so the comparison is in-run.
bind scrambler pr7f_h9 #(.NAME("H9_DLLP_CTRL"), .ARM_SYM(8'h5C)) u_pr7f_h9d (
    .clk(clk_i), .rst(rst_i),
    .din(data_in_i),  .kin(data_k_in_i),  .vin(data_valid_i),
    .dout(data_out_o), .kout(data_k_out_o), .vout(data_valid_o));

// =============================================================================
// §63 #7f Phase 2e -- two probes. MEASUREMENT ONLY.
//
// (A) #18's discriminator. H8 was re-scored NOT TESTED because both UpdateFC
//     samples PRECEDE any NP processing, so HdrFC=16 is correct at those
//     instants and proves nothing about whether the allocated register ever
//     advances. What IS proven is the absence of any UpdateFC after credit
//     consumption. This probe separates the two: it watches the advertised
//     value itself, the consumed counters, and the emission trigger.
//
// (B) H9, paired BY FRAMING CLASS. The 2d probe paired by "first END after
//     arming" and mis-attributed a preceding DLLP's END to a TLP. Pairing by
//     tail_out > tail_in would only have hidden that; the correct pairing is
//     STP -> END with NO intervening SDP (a TLP), and SDP -> END (a DLLP).
// =============================================================================

// -- (A) advertised credit vs credit actually consumed ------------------------
module pr7f_alloc (
    input logic clk, rst,
    input logic [7:0]  nph_adv, input logic [11:0] npd_adv,
    input logic [7:0]  ph_adv,
    input logic [7:0]  nph_cons, input logic [11:0] npd_cons,
    input logic [7:0]  ph_cons,
    input logic update_fc, fc1_stored, fc2_stored
);
  longint unsigned cyc = 0, n_upd = 0;
  logic [7:0]  nph_adv_p = 8'hFF, ph_adv_p = 8'hFF, nph_cons_p = 8'hFF;
  always @(posedge clk) begin
    if (!rst) begin
      cyc <= cyc + 1;
      // EVERY change of the advertised register -- the HdrFC source
      if (nph_adv != nph_adv_p) begin
        nph_adv_p <= nph_adv;
        $display("PR7F_AL scope=%m ev=ADV_NPH_CHANGE cyc=%0d nph_adv=%0d npd_adv=%0d",
                 cyc, nph_adv, npd_adv);
      end
      if (ph_adv != ph_adv_p) begin
        ph_adv_p <= ph_adv;
        $display("PR7F_AL scope=%m ev=ADV_PH_CHANGE cyc=%0d ph_adv=%0d", cyc, ph_adv);
      end
      // EVERY change of the consumed counter -- proves NP traffic was processed
      if (nph_cons != nph_cons_p) begin
        nph_cons_p <= nph_cons;
        $display("PR7F_AL scope=%m ev=NPH_CONSUMED cyc=%0d nph_cons=%0d npd_cons=%0d nph_adv_now=%0d",
                 cyc, nph_cons, npd_cons, nph_adv);
      end
      // the emission trigger itself
      if (update_fc) begin
        n_upd <= n_upd + 1;
        $display("PR7F_AL scope=%m ev=UPDATE_FC_TRIGGER n=%0d cyc=%0d nph_adv=%0d nph_cons=%0d fc1=%0b fc2=%0b",
                 n_upd + 1, cyc, nph_adv, nph_cons, fc1_stored, fc2_stored);
      end
    end
  end
  final $display("PR7F_ALSUM scope=%m update_fc_pulses=%0d final_nph_adv=%0d final_nph_cons=%0d cyc=%0d",
                 n_upd, nph_adv, nph_cons, cyc);
endmodule

bind dllp_receive pr7f_alloc u_pr7f_al (
    .clk(clk_i), .rst(rst_i),
    .nph_adv(tx_fc_nph_o), .npd_adv(tx_fc_npd_o), .ph_adv(tx_fc_ph_o),
    // sec 63 #7f commit A renamed dllp_receive's wires: these ARE the
    // receive side's CREDITS_ALLOCATED (Phase 2e called them "consumed" and
    // the peer's limit "advertised" -- both labels were the misnomer).
    .nph_cons(nph_credits_allocated), .npd_cons(npd_credits_allocated),
    .ph_cons(ph_credits_allocated),
    .update_fc(update_fc_o), .fc1_stored(fc1_values_stored_o),
    .fc2_stored(fc2_values_stored_o));

// -- (B) H9, paired by framing class -----------------------------------------
module pr7f_h9c #(parameter string NAME = "?", parameter int MAX_PKT = 8) (
    input logic clk, rst,
    input logic [31:0] din,  input logic [3:0] kin,  input logic vin,
    input logic [31:0] dout, input logic [3:0] kout, input logic vout
);
  localparam logic [7:0] S_STP = 8'hFB, S_SDP = 8'h5C, S_END = 8'hFD;
  longint unsigned cyc = 0, pkt = 0;
  // input-side class tracker
  logic in_tlp_i = 1'b0; longint unsigned in_start = 0, in_end = 0;
  logic have_in = 1'b0; longint unsigned in_len = 0;
  // output-side class tracker -- INDEPENDENT, so an END is attributed to the
  // class that opened on THIS face, never to whatever armed the probe
  logic in_tlp_o = 1'b0; longint unsigned out_start = 0;
  logic vin_p = 1'b0;
  logic exhausted = 1'b0;

  function automatic logic has(input logic [31:0] d, input logic [3:0] k,
                               input logic [7:0] s);
    has = 1'b0;
    for (int b = 0; b < 4; b++) if (k[b] && d[b*8+:8] == s) has = 1'b1;
  endfunction

  always @(posedge clk) begin
    if (!rst) begin
      cyc <= cyc + 1;
      vin_p <= vin;
      if (vin) begin
        if (has(din, kin, S_STP)) begin in_tlp_i <= 1'b1; in_start <= cyc; end
        else if (has(din, kin, S_SDP)) in_tlp_i <= 1'b0;
        if (has(din, kin, S_END) && in_tlp_i) begin
          in_end <= cyc; have_in <= 1'b1; in_len <= cyc - in_start + 1;
          in_tlp_i <= 1'b0;
        end
      end
      if (vout) begin
        if (has(dout, kout, S_STP)) begin in_tlp_o <= 1'b1; out_start <= cyc; end
        else if (has(dout, kout, S_SDP)) in_tlp_o <= 1'b0;
        // a TLP's END on the OUTPUT face, paired to the STP that opened it here
        if (has(dout, kout, S_END) && in_tlp_o) begin
          in_tlp_o <= 1'b0;
          if (pkt < MAX_PKT && have_in) begin
            pkt <= pkt + 1;
            $display("PR7F_H9C %s scope=%m ev=TLP_TAIL pkt=%0d in_start=%0d in_end=%0d in_len=%0d out_start=%0d out_end=%0d residue=%0d coincident_input_burst=%0b coincident_class=%s budget_remaining=%0d",
                     NAME, pkt, in_start, in_end, in_len, out_start, cyc,
                     cyc - out_start + 1, (vin && !vin_p),
                     (vin && has(din, kin, S_STP)) ? "STP" :
                     (vin && has(din, kin, S_SDP)) ? "SDP" :
                     (vin ? "DATA" : "none"), MAX_PKT - pkt - 1);
          end
        end
      end
      if (pkt >= MAX_PKT && !exhausted) begin
        exhausted <= 1'b1;
        $display("PR7F_H9C %s scope=%m ev=BUDGET_EXHAUSTED cyc=%0d", NAME, cyc);
      end
    end
  end
endmodule

bind scrambler pr7f_h9c #(.NAME("H9C")) u_pr7f_h9c (
    .clk(clk_i), .rst(rst_i),
    .din(data_in_i),  .kin(data_k_in_i),  .vin(data_valid_i),
    .dout(data_out_o), .kout(data_k_out_o), .vout(data_valid_o));

// =============================================================================
// §63 #7f Phase 3.1 -- 21-c: DLLP tails, with the quiet-link condition MEASURED
// rather than assumed. 2d called a back-to-back InitFC stream a "control"; this
// records idle-before-arm so "isolated" is a datum, not a label.
// =============================================================================
module pr7f_dtail #(parameter string NAME = "?", parameter int MAX_PKT = 12,
                    parameter int QUIET = 16) (
    input logic clk, rst,
    input logic [31:0] din,  input logic [3:0] kin,  input logic vin,
    input logic [31:0] dout, input logic [3:0] kout, input logic vout
);
  localparam logic [7:0] S_STP = 8'hFB, S_SDP = 8'h5C, S_END = 8'hFD;
  longint unsigned cyc = 0, pkt = 0, idle = 0;
  logic in_d_i = 1'b0, in_d_o = 1'b0;
  longint unsigned d_start = 0, d_end = 0, o_start = 0, idle_at_arm = 0;
  logic have = 1'b0, exhausted = 1'b0;
  logic [31:0] first_w = '0;

  function automatic logic has(input logic [31:0] d, input logic [3:0] k,
                               input logic [7:0] s);
    has = 1'b0;
    for (int b = 0; b < 4; b++) if (k[b] && d[b*8+:8] == s) has = 1'b1;
  endfunction

  always @(posedge clk) begin
    if (!rst) begin
      cyc <= cyc + 1;
      idle <= vin ? 0 : idle + 1;
      if (vin) begin
        if (has(din, kin, S_SDP)) begin
          in_d_i <= 1'b1; d_start <= cyc; idle_at_arm <= idle; first_w <= din;
        end else if (has(din, kin, S_STP)) in_d_i <= 1'b0;
        if (has(din, kin, S_END) && in_d_i) begin
          in_d_i <= 1'b0; d_end <= cyc; have <= 1'b1;
        end
      end
      if (vout) begin
        if (has(dout, kout, S_SDP)) begin in_d_o <= 1'b1; o_start <= cyc; end
        else if (has(dout, kout, S_STP)) in_d_o <= 1'b0;
        if (has(dout, kout, S_END) && in_d_o) begin
          in_d_o <= 1'b0;
          if (pkt < MAX_PKT && have) begin
            pkt <= pkt + 1;
            $display("PR7F_DT %s scope=%m ev=DLLP_TAIL pkt=%0d in_start=%0d in_end=%0d in_len=%0d out_start=%0d out_end=%0d residue=%0d idle_before=%0d isolated=%0b first_w=0x%08h budget_remaining=%0d",
                     NAME, pkt, d_start, d_end, d_end - d_start + 1, o_start, cyc,
                     cyc - o_start + 1, idle_at_arm, (idle_at_arm >= QUIET),
                     first_w, MAX_PKT - pkt - 1);
          end
        end
      end
      if (pkt >= MAX_PKT && !exhausted) begin
        exhausted <= 1'b1;
        $display("PR7F_DT %s scope=%m ev=BUDGET_EXHAUSTED cyc=%0d", NAME, cyc);
      end
    end
  end
endmodule

bind scrambler pr7f_dtail #(.NAME("DT")) u_pr7f_dt (
    .clk(clk_i), .rst(rst_i),
    .din(data_in_i),  .kin(data_k_in_i),  .vin(data_valid_i),
    .dout(data_out_o), .kout(data_k_out_o), .vout(data_valid_o));

// =============================================================================
// §63 #7f Phase 3.1 (revised) -- 21-b coincidence census, 21-d 2-beat Ack.
// =============================================================================

// -- 21-b: which 1-bit signal toggles at the TLP tail release? ----------------
// Bound to phy_transmit so the tail-release event (at its own scrambler child)
// and the candidate signals share one clock and one cycle counter. Mechanical:
// no signal is privileged, every one is counted the same way, and each one's
// TOGGLE PERIOD is reported so the tiebreak against 21-a's fitted period is
// possible (C16 predicts more than one signal will sit at N/N).
module pr7f_cens #(parameter string NAME = "?", parameter int NSIG = 22,
                   parameter int WIN = 2, parameter int MAXREL = 12) (
    input logic clk, rst,
    input logic [NSIG-1:0] sig,
    input logic [31:0] sdata, input logic [3:0] skk, input logic svalid
);
  localparam logic [7:0] S_STP = 8'hFB, S_SDP = 8'h5C, S_END = 8'hFD;
  longint unsigned cyc = 0, rel = 0;
  logic [NSIG-1:0] prev = '0;
  longint unsigned last_tog[64];
  longint unsigned sum_per[64];
  longint unsigned n_per[64];
  longint unsigned hit[64];
  longint unsigned tog_cyc[64];
  logic in_tlp_o = 1'b0;

  function automatic logic has(input logic [31:0] d, input logic [3:0] k,
                               input logic [7:0] s);
    has = 1'b0;
    for (int b = 0; b < 4; b++) if (k[b] && d[b*8+:8] == s) has = 1'b1;
  endfunction

  initial for (int i = 0; i < 64; i++) begin
    last_tog[i]=0; sum_per[i]=0; n_per[i]=0; hit[i]=0; tog_cyc[i]=0;
  end

  always @(posedge clk) begin
    if (!rst) begin
      cyc <= cyc + 1;
      for (int i = 0; i < NSIG; i++) begin
        if (sig[i] !== prev[i]) begin
          if (last_tog[i] != 0) begin
            sum_per[i] <= sum_per[i] + (cyc - last_tog[i]);
            n_per[i]   <= n_per[i] + 1;
          end
          last_tog[i] <= cyc;
          tog_cyc[i]  <= cyc;
        end
      end
      prev <= sig;
      if (svalid) begin
        if (has(sdata, skk, S_STP)) in_tlp_o <= 1'b1;
        else if (has(sdata, skk, S_SDP)) in_tlp_o <= 1'b0;
        if (has(sdata, skk, S_END) && in_tlp_o) begin
          in_tlp_o <= 1'b0;
          if (rel < MAXREL) begin
            rel <= rel + 1;
            for (int i = 0; i < NSIG; i++)
              if (tog_cyc[i] != 0 && (cyc - tog_cyc[i]) <= WIN)
                hit[i] <= hit[i] + 1;
            $display("PR7F_CB %s scope=%m ev=TAIL_RELEASE n=%0d cyc=%0d", NAME, rel+1, cyc);
          end
        end
      end
    end
  end

  final begin
    $display("PR7F_CB %s scope=%m ev=CENSUS releases=%0d", NAME, rel);
    for (int i = 0; i < NSIG; i++)
      $display("PR7F_CB %s scope=%m ev=SIG idx=%0d hits=%0d of=%0d toggles=%0d mean_period=%0d",
               NAME, i, hit[i], rel, n_per[i],
               (n_per[i] == 0) ? 0 : sum_per[i] / n_per[i]);
  end
endmodule

bind phy_transmit pr7f_cens #(.NAME("CB"), .NSIG(22)) u_pr7f_cb (
    .clk(clk_i), .rst(rst_i),
    .sig({fifo_framed_axis_tlast, fifo_framed_axis_tready, fifo_framed_axis_tvalid,
          fifo_phy_axis_tlast, fifo_phy_axis_tready, fifo_phy_axis_tvalid,
          framed_axis_tlast, framed_axis_tready, framed_axis_tvalid,
          ordered_set_tranmitted, phy_axis_tlast, phy_axis_tready, phy_axis_tvalid,
          rx_fifo_empty, rx_fifo_full, rx_rd_en, rx_wr_en, send_ordered_set,
          tx_fifo_empty, tx_fifo_full, tx_rd_en, tx_wr_en}),
    .sdata(gen_lane_scramble[0].scrambler_inst.data_out_o),
    .skk  (gen_lane_scramble[0].scrambler_inst.data_k_out_o),
    .svalid(gen_lane_scramble[0].scrambler_inst.data_valid_o));

// -- 21-d: the Ack's SECOND beat, where AckNak_Seq_Num actually lives ---------
module pr7f_ack2 #(parameter string NAME = "?", parameter int MAX_PKT = 40) (
    input logic clk, rst, input logic [31:0] tdata,
    input logic tvalid, tlast, tready, input logic [7:0] tuser
);
  longint unsigned cyc = 0, pkt = 0, beat = 0;
  logic in_pkt = 1'b0, cap = 1'b0;
  logic [31:0] w0 = '0;
  always @(posedge clk) begin
    if (!rst) begin
      cyc <= cyc + 1;
      if (tvalid && tready) begin
        if (!in_pkt) begin
          in_pkt <= 1'b1; beat <= 1; w0 <= tdata;
          cap <= (tuser[2:0] == 3'd1) && ((tdata & 32'hFF) == 32'h00) && (pkt < MAX_PKT);
        end else begin
          beat <= beat + 1;
          if (cap && beat == 1)
            $display("PR7F_A2 %s scope=%m ev=ACK pkt=%0d cyc=%0d w0=0x%08h w1=0x%08h",
                     NAME, pkt, cyc, w0, tdata);
        end
        if (tlast) begin in_pkt <= 1'b0; cap <= 1'b0; pkt <= pkt + 1; end
      end
    end
  end
endmodule

bind pcie_datalink_layer pr7f_ack2 #(.NAME("A2")) u_pr7f_a2 (
    .clk(clk_i), .rst(rst_i), .tdata(32'(m_phy_axis_tdata)),
    .tvalid(m_phy_axis_tvalid), .tlast(m_phy_axis_tlast), .tready(m_phy_axis_tready),
    .tuser({{(8-USER_WIDTH){1'b0}}, m_phy_axis_tuser}));

// =============================================================================
// §63 #7f — the last #21 measurement in this rung. RAW EVENTS ONLY.
//
// !! NEW STANDING RULE (Kourosh, 2026-09-18, after seven instrument faults):
// an SV probe emits raw timestamped events and NOTHING ELSE. All pairing,
// correlation and arithmetic happens in Python over the log, with a
// known-answer self-test before any number is interpreted. Six of this rung's
// seven faults were in SV-side correlation logic that a Python pass would have
// made visible and cheap to fix.
//
// This probe therefore does not pair, classify, window or decide. It logs the
// cycle of every K28.5 (COM, 0xBC) at the RC PIPE TX seam. Whether those
// cycles fit 7750 + 679n is a Python question.
//
// ⚠️ §22.85: `send_ordered_set` being static and H9's scrambler-INPUT check are
// both SINGLE-ROUTE negatives. Neither excludes an ordered set appearing on the
// WIRE — a different route, which is exactly what this probe reads.
// =============================================================================
module pr7f_com #(parameter string NAME = "?", parameter int MAX_EV = 4000) (
    input logic clk, rst,
    input logic [31:0] data, input logic [3:0] k, input logic valid
);
  localparam logic [7:0] SYM_COM = 8'hBC;
  longint unsigned cyc = 0, n = 0, supp = 0;
  logic exhausted = 1'b0;
  always @(posedge clk) begin
    if (!rst) begin
      cyc <= cyc + 1;
      if (valid) begin
        for (int b = 0; b < 4; b++)
          if (k[b] && data[b*8+:8] == SYM_COM) begin
            if (n < MAX_EV) begin
              n <= n + 1;
              $display("PR7F_COM %s scope=%m cyc=%0d lane_byte=%0d data=0x%08h k=0x%01h",
                       NAME, cyc, b, data, k);
            end else begin
              supp <= supp + 1;
              if (!exhausted) begin
                exhausted <= 1'b1;
                $display("PR7F_COM %s scope=%m ev=BUDGET_EXHAUSTED cyc=%0d", NAME, cyc);
              end
            end
          end
      end
    end
  end
  final $display("PR7F_COM %s scope=%m ev=SUM emitted=%0d suppressed=%0d cyc=%0d",
                 NAME, n, supp, cyc);
endmodule

bind phy_transmit pr7f_com #(.NAME("COM_PIPE_TX")) u_pr7f_com (
    .clk(clk_i), .rst(rst_i),
    .data(pipe_data_o[31:0]), .k(pipe_data_k_o[3:0]), .valid(pipe_data_valid_o[0]));
