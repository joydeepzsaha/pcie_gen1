// =============================================================================
// §63 #7d PROBE PHASE -- the FC-init blocker. BENCH-ONLY, attached by `bind`.
//
// !! NO src/ CHANGE. The modules measured are byte-identical to the ones the
// gate builds.
//
// Answers Kourosh's four probes in one elaboration:
//   1. FSM state sequence with cycle stamps, both sides
//   2. elaborated timer constants (BY ELABORATION, not by reading the source --
//      there is a documented NAME COLLISION on `FcWaitPeriod` between
//      pcie_flow_ctrl_init.sv:43 and dllp_fc_update.sv:45, five orders of
//      magnitude apart, so reading one file cannot answer which governs)
//   3. window length vs pending timer
//   4. fc_initialized_o sampled EVERY CYCLE, with its first rise stamped --
//      E3 says the flag rises and the monitor misses it
// =============================================================================
`timescale 1ns / 1ps

// -----------------------------------------------------------------------------
// Probe 1 + 2a -- the FC-init FSM itself.
// -----------------------------------------------------------------------------
module pr7de_fcinit #(
    parameter int P_FC_WAIT      = -1,
    parameter int P_FC_INIT_WAIT = -1
) (
    input logic       clk,
    input logic       rst,
    input logic [4:0] state,
    input logic [15:0] seq_count,
    input logic       fc2_sent,
    input logic       fc2_stored,
    // §63 #7d probe phase: CHECK_FC2 (:401) exits only on
    //   fc2_values_stored_i && (update_fc_r == '1 || idle_count_r >= 16'h60)
    // fc2_values_stored_i is measured TRUE from cycle 4,439, so the dead term is
    // the parenthesised pair. These two say WHICH.
    input logic        update_fc,
    input logic [15:0] idle_count,
    // §63 #7d item (3): the DRIVERS. idle_count_c increments on idle_valid_i and
    // update_fc_c sets on update_fc_i (:168-176), both gated by curr_state >=
    // ST_FC2. A dead term means a dead input; these say which input is dead.
    input logic        idle_valid_in,
    input logic        update_fc_in
);
  localparam int NSTATE = 32;

  longint unsigned cyc = 0;
  longint unsigned first_cyc[NSTATE];     // first cycle each state was entered
  longint unsigned last_cyc[NSTATE];      // last cycle each state was occupied
  longint unsigned occupancy[NSTATE];     // cycles spent in each state
  longint unsigned entries[NSTATE];       // number of distinct entries
  logic [4:0] prev_state = 5'h1F;
  logic       seen = 1'b0;

  longint unsigned max_seq = 0;
  longint unsigned fc2_sent_first = 0, fc2_stored_first = 0;
  longint unsigned upd_high = 0, upd_first = 0, max_idle = 0, idle_ge_60 = 0;
  // §63 #7d E4: WHICH TERM FIRED FIRST. Counts alone cannot say -- in #33's
  // bench both terms are live, so only the first-true cycle of each separates them.
  longint unsigned idle_ge60_first = 0, exit_term_first = 0;
  longint unsigned idle_valid_high = 0, idle_valid_first = 0;
  longint unsigned update_fc_in_high = 0, update_fc_in_first = 0;

  initial begin
    for (int i = 0; i < NSTATE; i++) begin
      first_cyc[i] = 0;
      last_cyc[i]  = 0;
      occupancy[i] = 0;
      entries[i]   = 0;
    end
    $display("PR7DE_CONST %m FcWaitPeriod=%0d FcInitWaitPeriod=%0d",
             P_FC_WAIT, P_FC_INIT_WAIT);
  end

  always @(posedge clk) begin
    if (!rst) begin
      cyc <= cyc + 1;
      occupancy[state] <= occupancy[state] + 1;
      last_cyc[state]  <= cyc;
      if (state != prev_state) begin
        entries[state] <= entries[state] + 1;
        if (entries[state] == 0) first_cyc[state] <= cyc;
      end
      prev_state <= state;
      if (seq_count > max_seq) max_seq <= seq_count;
      if (fc2_sent && fc2_sent_first == 0) fc2_sent_first <= cyc;
      if (fc2_stored && fc2_stored_first == 0) fc2_stored_first <= cyc;
      if (update_fc) begin
        upd_high <= upd_high + 1;
        if (upd_first == 0) upd_first <= cyc;
      end
      if (idle_count > max_idle) max_idle <= idle_count;
      if (idle_count >= 16'h60) begin
        idle_ge_60 <= idle_ge_60 + 1;
        if (idle_ge60_first == 0) idle_ge60_first <= cyc;
      end
      if ((update_fc || idle_count >= 16'h60) && exit_term_first == 0)
        exit_term_first <= cyc;
      if (idle_valid_in) begin
        idle_valid_high <= idle_valid_high + 1;
        if (idle_valid_first == 0) idle_valid_first <= cyc;
      end
      if (update_fc_in) begin
        update_fc_in_high <= update_fc_in_high + 1;
        if (update_fc_in_first == 0) update_fc_in_first <= cyc;
      end
    end
  end

  final begin
    $display("PR7DE_FSM %m cycles=%0d max_seq_count=%0d fc2_sent_first=%0d fc2_stored_first=%0d",
             cyc, max_seq, fc2_sent_first, fc2_stored_first);
    for (int i = 0; i < NSTATE; i++)
      if (entries[i] != 0)
        $display("PR7DE_FSM %m state[%0d] entries=%0d first_cyc=%0d last_cyc=%0d occupancy=%0d",
                 i, entries[i], first_cyc[i], last_cyc[i], occupancy[i]);
    $display("PR7DE_FSM %m TERMINAL_STATE=%0d occupancy=%0d", prev_state,
             occupancy[prev_state]);
    $display("PR7DE_DRIVER %m idle_valid_i_high=%0d idle_valid_i_first=%0d update_fc_i_high=%0d update_fc_i_first=%0d",
             idle_valid_high, idle_valid_first, update_fc_in_high, update_fc_in_first);
    $display("PR7DE_GATE %m update_fc_high=%0d update_fc_first=%0d max_idle_count=%0d idle_ge_0x60_cycles=%0d idle_ge60_first=%0d exit_term_first=%0d WHICH_FIRED_FIRST=%s EXIT_TERM_EVER_TRUE=%s",
             upd_high, upd_first, max_idle, idle_ge_60, idle_ge60_first, exit_term_first,
             (upd_high == 0 && idle_ge_60 == 0) ? "NEITHER" :
               (idle_ge_60 != 0 && (upd_high == 0 || idle_ge60_first < upd_first)) ? "idle_count_r" :
               (upd_high != 0 && (idle_ge_60 == 0 || upd_first < idle_ge60_first)) ? "update_fc_r" : "SAME_CYCLE",
             (upd_high != 0 || idle_ge_60 != 0) ? "YES" : "NO -- BOTH DEAD");
  end
endmodule

// -----------------------------------------------------------------------------
// Probe 2b + 3 -- the OTHER FcWaitPeriod, in dllp_fc_update, and its live timer
// against the length of the run. This is the collision's other half.
// -----------------------------------------------------------------------------
module pr7de_fcupd #(
    parameter int P_CLK_PERIOD_NS = -1,
    parameter int P_TWO_MS        = -1,
    parameter int P_FC_WAIT       = -1
) (
    input logic        clk,
    input logic        rst,
    input logic [31:0] timer
);
  longint unsigned cyc = 0, max_timer = 0, cyc_timer_saturated = 0;

  initial
    $display("PR7DE_CONST %m ClockPeriodNs=%0d TwoMsTimeOut=%0d FcWaitPeriod=%0d",
             P_CLK_PERIOD_NS, P_TWO_MS, P_FC_WAIT);

  always @(posedge clk) begin
    if (!rst) begin
      cyc <= cyc + 1;
      if (timer > max_timer) max_timer <= timer;
      if (P_FC_WAIT > 0 && timer >= P_FC_WAIT && cyc_timer_saturated == 0)
        cyc_timer_saturated <= cyc;
    end
  end

  final begin
    // THE PROBE-3 COMPARISON, stated as a verdict rather than left to the reader.
    $display("PR7DE_TIMER %m run_cycles=%0d FcWaitPeriod=%0d max_timer_reached=%0d saturated_at_cyc=%0d REACHABLE=%s",
             cyc, P_FC_WAIT, max_timer, cyc_timer_saturated,
             (P_FC_WAIT > 0 && cyc >= P_FC_WAIT) ? "YES" : "NO -- RUN IS SHORTER THAN THE TIMER");
  end
endmodule

// -----------------------------------------------------------------------------
// Probe 4 -- fc_initialized_o sampled EVERY CYCLE. E3: the flag rises and the
// monitor misses it. A per-cycle rise counter cannot miss a one-cycle pulse the
// way an end-of-window read can.
// -----------------------------------------------------------------------------
module pr7de_fcflag (
    input logic clk,
    input logic rst,
    input logic fc_initialized,
    input logic fc2_sent,
    input logic fc2_stored
);
  longint unsigned cyc = 0, high_cycles = 0, rises = 0, falls = 0;
  longint unsigned first_rise = 0, last_rise = 0, first_fall = 0;
  longint unsigned longest_high = 0, run = 0;
  logic q = 1'b0;

  always @(posedge clk) begin
    if (!rst) begin
      cyc <= cyc + 1;
      if (fc_initialized) begin
        high_cycles <= high_cycles + 1;
        run <= run + 1;
        if (run + 1 > longest_high) longest_high <= run + 1;
      end else begin
        run <= 0;
      end
      if (fc_initialized && !q) begin
        rises <= rises + 1;
        if (first_rise == 0) first_rise <= cyc;
        last_rise <= cyc;
      end
      if (!fc_initialized && q) begin
        falls <= falls + 1;
        if (first_fall == 0) first_fall <= cyc;
      end
      q <= fc_initialized;
    end
  end

  final begin
    $display("PR7DE_FLAG %m cycles=%0d high_cycles=%0d rises=%0d falls=%0d first_rise=%0d last_rise=%0d first_fall=%0d longest_high_run=%0d",
             cyc, high_cycles, rises, falls, first_rise, last_rise, first_fall,
             longest_high);
  end
endmodule

// =============================================================================
// bind statements -- file scope. Both sides are hit: pcie_flow_ctrl_init,
// dllp_fc_update and pcie_datalink_layer each exist in the RC and the EP.
// =============================================================================

bind pcie_flow_ctrl_init pr7de_fcinit #(
    .P_FC_WAIT     (FcWaitPeriod),
    .P_FC_INIT_WAIT(FcInitWaitPeriod)
) u_pr7de_fcinit (
    .clk       (clk_i),
    .rst       (rst_i),
    .state     (5'(curr_state)),
    .seq_count (seq_count_r),
    .fc2_sent  (fc2_values_sent_o),
    .fc2_stored(fc2_values_stored_i),
    .update_fc (update_fc_r),
    .idle_count(idle_count_r),
    .idle_valid_in(idle_valid_i),
    .update_fc_in (update_fc_i)
);

bind dllp_fc_update pr7de_fcupd #(
    .P_CLK_PERIOD_NS(ClockPeriodNs),
    .P_TWO_MS       (TwoMsTimeOut),
    .P_FC_WAIT      (FcWaitPeriod)
) u_pr7de_fcupd (
    .clk  (clk_i),
    .rst  (rst_i),
    .timer(32'(timer_r))
);

bind pcie_datalink_layer pr7de_fcflag u_pr7de_fcflag (
    .clk           (clk_i),
    .rst           (rst_i),
    .fc_initialized(fc_initialized_o),
    .fc2_sent      (fc2_values_sent),
    .fc2_stored    (fc2_values_stored)
);
