// =============================================================================
// §63 #7h Phase 1, instrument 1-e -- THE RAW PIPE TX WINDOW. BENCH-ONLY, `bind`.
//
// !! NOTHING IN src/ CHANGES FOR THIS FILE, and verilate_fullstack is untouched:
// own fileset, own target (§63 #7d's rule, kept since).  A probe that perturbs
// the row it measures is not a measurement of that row.
//
// == WHAT THIS MEASURES =====================================================
//
// H7H-5: "between a CfgRd0's STP and END on the PIPE TX there are hundreds of
// non-packet cycles.  What those cycles CARRY (valid low?  idle symbols?) is
// unmeasured, and it decides what a real link partner would see."
//
// So this logs EVERY CYCLE, valid or not, for a long bounded span after the
// first STP, at the seam D-7H.4(a) is stated about: `phy_transmit`'s PIPE TX
// output.  Nothing is classified here; the log is raw and every question is
// answered offline in Python (§22.92).
//
// == WHY IT IS A COPY OF pr7f_trace, NOT A REUSE ============================
//
// `pr7f_trace` already has this exact shape and its SPAN is 28 -- sized for
// #7f's latency ladder, where the question was "which stage adds the delay" and
// 28 cycles bracketed a stage.  #21's span is the whole STP-to-END stretch,
// which #7f measured at ~679 cycles on the COM grid and up to 2,743 across the
// link.  Re-parameterising the shared probe would move every #7f row that
// depends on it (§22.85: a deletion or a widening is a route change too), so
// this rung gets its own file and its own target, and `probe_7f.sv` is left
// byte-identical.
//
// == THE THREE #7f RULES, ALL KEPT ==========================================
//
// (1) NO CUMULATIVE `final` COUNTERS -- every line carries $time and cyc, and
//     all windowing is offline.
// (2) CLASSIFY BY FRAMING, NEVER BY BEAT ARITHMETIC -- the window ARMS on STP
//     (0xFB, K) and the closing END (0xFD, K) is found offline in the log.
//     K codes are not scrambled (Base 2.1 §4.2.3 p.199), so this identity
//     survives the scrambler.  No beat counting anywhere.
// (3) NO SILENT CAPS -- the span is fixed, it is printed in the header line
//     this module emits when it arms, and the offline reader is told the bound
//     so a truncated window can never read as "this is everything".
//
// ⚠️ THE WINDOW OPENS ON A SIGNAL, NOT A CYCLE NUMBER (§22.89 second limb), and
// it arms ONCE.  A probe that armed on every STP would interleave the two
// stacks' packets in one log and the offline reader would have to guess which
// STP a given cycle belonged to.
// =============================================================================

module pr7h_window #(
    parameter string NAME = "?",
    // The whole STP-to-END stretch plus margin.  #7f measured release on a
    // free-running 679-cycle COM grid and an EP->RC stretch of 2,743 cycles;
    // 1400 covers a full grid period either side of the packet at one stage.
    parameter int    SPAN = 1400
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
            // Rule 3: the bound is IN the log, at the moment of arming.
            $display("PR7H_ARM %s scope=%m cyc=%0d t=%0t span=%0d",
                     NAME, cyc, $time, SPAN);
          end
      end else if (armed && left > 0) begin
        left <= left - 1;
        // EVERY cycle, valid or not.  That is the whole point: a valid-gated
        // probe cannot settle its own gating question.
        $display("PR7H_W %s cyc=%0d t=%0t valid=%0b data=0x%08h k=0x%01h",
                 NAME, cyc, $time, valid, data, k);
        if (left == 1)
          $display("PR7H_END %s scope=%m cyc=%0d t=%0t span_exhausted=1",
                   NAME, cyc, $time);
      end
    end
  end
endmodule

// -----------------------------------------------------------------------------
// THE SEAM D-7H.4(a) IS STATED ABOUT: phy_transmit's PIPE TX output.
//
// Bound by MODULE TYPE, so it fires at BOTH stacks; `scope=%m` in the arm line
// is what separates them offline.  Lane 0 only -- #21 is per-lane-identical and
// the link is x1 in this bench.
// -----------------------------------------------------------------------------
bind phy_transmit pr7h_window #(.NAME("E1_PIPE_TX")) u_pr7h_pipe_tx (
    .clk(pipe_tx_usr_clk_i), .rst(rst_i),
    .data(pipe_data_o[31:0]), .k(pipe_data_k_o[3:0]), .valid(pipe_data_valid_o[0])
);

// The scrambler's own two faces at the SAME long span, so the stretch measured
// at the PIPE can be attributed to this stage rather than inferred.  #7f's
// T_SCRAM_IN / T_SCRAM_OUT are the same two points at SPAN=28.
bind scrambler pr7h_window #(.NAME("E2_SCRAM_IN")) u_pr7h_sc_in (
    .clk(clk_i), .rst(rst_i), .data(data_in_i), .k(data_k_in_i),
    .valid(data_valid_i)
);
bind scrambler pr7h_window #(.NAME("E3_SCRAM_OUT")) u_pr7h_sc_out (
    .clk(clk_i), .rst(rst_i), .data(data_out_o), .k(data_k_out_o),
    .valid(data_valid_o)
);
