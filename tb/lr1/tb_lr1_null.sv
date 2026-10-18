// ---------------------------------------------------------------------------
// tb_lr1_null -- an empty toplevel for the far end's known-answer rows
//
// Author: Kourosh Ghahramani
// Silicon Systems Research Lab, University of Washington
//
// Purpose
//   verilate_lr1_kat checks the LR1 far end's protocol parts (CRCs,
//   scrambler, ordered sets, framing, TLP and DLLP layouts) against outside
//   known answers in Python. Those rows read no RTL; cocotb needs a toplevel,
//   and this one has a single unused clock input so the target builds in
//   seconds.
// ---------------------------------------------------------------------------

module tb_lr1_null (
    input logic clk_i
);
endmodule
