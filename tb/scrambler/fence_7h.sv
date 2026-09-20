// =============================================================================
// §63 #7h Phase 3 -- THE FENCE DUMP. BENCH-ONLY, `bind`.
//
// !! NOTHING IN src/ CHANGES FOR THIS FILE, and no existing target changes:
// own fileset, own targets that MIRROR the four fenced ones.  The four gate
// rows (verilate_tx_golden, verilate_phy_transmit_stall, verilate_tx_framing,
// verilate_scrambler_align) stay byte-identical -- a probe that perturbs the
// row it measures is not a measurement of that row.
//
// == WHAT THE FENCE IS ======================================================
//
// Chat's Phase-3 fence: "published-valid word sequence (data + K) byte-identical
// old tree vs new tree for every existing stimulus in tx_golden,
// phy_transmit_stall, tx_framing, scrambler_align; only timing may move.  Raw
// dump, Python diff, known-answer first."
//
// So this dumps, for every clock on which the DUT says "here is a Symbol", the
// (data, k) pair and a monotonically increasing SEQUENCE INDEX.  The sequence
// index is what makes "only timing may move" checkable: two runs agree iff the
// n-th published pair is equal for every n, regardless of which CLOCK carried
// it.  The cycle number is dumped too, so the timing delta can be reported
// rather than merely tolerated.
//
// ⚠️ THE DUMP IS GATED ON VALID, AND THAT IS DELIBERATE -- it is the opposite
// choice from probe_7h.sv's window, on purpose.  Phase 1 instrument fault #4:
// a window must be UNGATED to see what the wire carries and GATED ON VALID to
// count what was SENT.  The fence is a claim about what was sent, so it is
// gated.  (An ungated fence would compare freeze lengths and would "move" for
// every timing change, which the fence explicitly permits.)
//
// ⚠️ `scope=%m` IS ON EVERY LINE, not only on a header -- Phase 1 instrument
// fault #2.  These benches instantiate `scrambler` once (scrambler_align) or
// per lane inside phy_transmit (tx_golden, phy_transmit_stall), and a multi-lane
// run would otherwise interleave two lanes into one unattributable sequence.
// =============================================================================

module fence_7h_sym #(
    parameter string NAME = "?"
) (
    input logic        clk,
    input logic        rst,
    input logic [31:0] data,
    input logic [3:0]  k,
    input logic        valid
);
  longint unsigned cyc = 0;
  longint unsigned seq = 0;

  always @(posedge clk) begin
    if (!rst) begin
      cyc <= cyc + 1;
      if (valid) begin
        $display("FENCE7H %s scope=%m seq=%0d cyc=%0d data=0x%08h k=0x%01h",
                 NAME, seq, cyc, data, k);
        seq <= seq + 1;
      end
    end
  end
endmodule

// The AXIS form, for frame_symbols -- tx_framing's toplevel.  It sits UPSTREAM
// of the scrambler, so this rung cannot change it; the dump is therefore a
// CONTROL, and it is the strongest kind: a stream that must be bit-identical
// for a reason independent of the fix.
module fence_7h_axis #(
    parameter string NAME = "?"
) (
    input logic        clk,
    input logic        rst,
    input logic [31:0] data,
    input logic [3:0]  keep,
    input logic        valid,
    input logic        ready,
    input logic        last
);
  longint unsigned cyc = 0;
  longint unsigned seq = 0;

  always @(posedge clk) begin
    if (!rst) begin
      cyc <= cyc + 1;
      if (valid && ready) begin
        $display("FENCE7H %s scope=%m seq=%0d cyc=%0d data=0x%08h k=0x%01h last=%0b",
                 NAME, seq, cyc, data, keep, last);
        seq <= seq + 1;
      end
    end
  end
endmodule
