// §63 #7h fence -- the `scrambler` binds, split into their own file so a target
// whose toplevel does not carry `scrambler` (tx_framing, toplevel frame_symbols)
// can take the module definitions without binding to an undeclared module.
//
// The fenced seam: `scrambler`'s published face.  This is the signal the fix
// changes (scrambler.sv:76's ungated 1-stage copy becomes the chain's own
// last-stage valid bit), so it is exactly where the fence belongs.  Bound by
// MODULE TYPE, so it fires wherever `scrambler` is instantiated -- phy_transmit's
// per-lane TX instances and phy_receive's descrambler alike.
bind scrambler fence_7h_sym #(.NAME("F_SCRAM_OUT")) u_fence7h_out (
    .clk(clk_i), .rst(rst_i),
    .data(data_out_o), .k(data_k_out_o), .valid(data_valid_o)
);

// The scrambler's INPUT face, so a fence diff can say whether a moved output
// came from a moved input (it must not: the fix is downstream of this point).
bind scrambler fence_7h_sym #(.NAME("F_SCRAM_IN")) u_fence7h_in (
    .clk(clk_i), .rst(rst_i),
    .data(data_in_i), .k(data_k_in_i), .valid(data_valid_i)
);
