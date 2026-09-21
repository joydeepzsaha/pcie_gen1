// =============================================================================
// §63 #7j-2 Phase 1 -- where does a packet starve when Logical Idle is
// requested continuously?  BENCH-ONLY, `bind`, own fileset, own target.
//
// §22.92: this probe emits RAW timestamped events and nothing else.  It has no
// assertions, no counters that encode a hypothesis, and no classification.  All
// pairing and arithmetic happen offline in analyse_7j2_arb.py.
//
// ⚠️ It exists because the port-level measurement is AMBIGUOUS about location.
// `phy_transmit.s_dllp_axis_tready` asserts immediately under continuous idle
// -- so the packet IS accepted -- and yet no STP and no END ever reach
// pipe_data_o.  Those two facts are compatible with at least two stories:
//   (1) the packet sits in phy_transmit's framing FIFO and lane_management
//       never services the DLLP arm, or
//   (2) the framing stage never presents it.
// Inferring between them from the datapath's shape is exactly the mistake this
// arc has made five times (§22.85).  This probe reads lane_management's own
// arbitration state and both of its input valids, so the answer is measured.
// =============================================================================
`timescale 1ns / 1ps

module pr7j2_lm (
    input logic clk_i,
    input logic rst_i,
    input logic [4:0] state,
    input logic phy_tvalid,          // s_phy_axis_tvalid  (OS arm, far side)
    input logic fifo_phy_tvalid,     // fifo_phy_axis_tvalid (OS arm, near side)
    input logic phy_tready,
    input logic dllp_tvalid,         // s_dllp_axis_tvalid (packet arm)
    input logic dllp_tready,
    input logic is_phy,
    input logic is_dllp
);
  int unsigned c;
  always_ff @(posedge clk_i) begin
    if (rst_i) c <= 0;
    else begin
      c <= c + 1;
      // Raw event, one line per cycle.  %m names the instance so a bind that
      // fires more than once stays attributable.
      $display("LM|%m|%0d|%0t|%0d|%0d|%0d|%0d|%0d|%0d|%0d|%0d",
               c, $time, state, phy_tvalid, fifo_phy_tvalid, phy_tready,
               dllp_tvalid, dllp_tready, is_phy, is_dllp);
    end
  end
endmodule

bind lane_management pr7j2_lm u_pr7j2_lm (
    .clk_i          (clk_i),
    .rst_i          (rst_i),
    .state          (curr_state),
    .phy_tvalid     (s_phy_axis_tvalid),
    .fifo_phy_tvalid(fifo_phy_axis_tvalid),
    .phy_tready     (s_phy_axis_tready),
    .dllp_tvalid    (s_dllp_axis_tvalid),
    .dllp_tready    (s_dllp_axis_tready),
    .is_phy         (is_phy_r),
    .is_dllp        (is_dllp_r)
);
