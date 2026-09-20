// §63 #7h fence -- the `frame_symbols` bind, in its own file for the same reason.
//
// frame_symbols sits UPSTREAM of the scrambler, so this rung cannot change it.
// The dump is therefore a CONTROL, and it is the strongest kind: a stream that
// must be bit-identical for a reason independent of the fix.
bind frame_symbols fence_7h_axis #(.NAME("F_FRAME_OUT")) u_fence7h_fs (
    .clk(clk_i), .rst(rst_i),
    .data(m_axis_tdata), .keep(m_axis_tkeep), .valid(m_axis_tvalid),
    .ready(m_axis_tready), .last(m_axis_tlast)
);
