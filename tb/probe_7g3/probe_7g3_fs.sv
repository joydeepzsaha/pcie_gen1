// sec 63 #7g-3 Phase 1 -- NOT A GATE ROW, NOT IN ANY CORE.  The frame_symbols
// fence for G7-5 / C-7G3-5: every beat frame_symbols PUBLISHES (m_axis handshake)
// and every change of fifo_axis_tready, the signal the latch fix defaults.
//
// Raw events only (sec 22.92): one line per event, $time first, no pairing, no
// counters.  One file per instance, named from %m, so the two stacks of the full
// stack never interleave (7g-2 handoff item 8).  Appended to a staged .vc by
// run_probe_7g3.sh; the gate's own invocation is otherwise unchanged.
//
// Written width-clean on purpose: six cores are warnings-fatal and this file is
// outside every waiver.
module pr7g3_fs (
    input logic        clk,
    input logic [31:0] tdata,
    input logic [3:0]  tkeep,
    input logic [4:0]  tuser,
    input logic        tvalid,
    input logic        tready,
    input logic        tlast,
    input logic        fifo_rdy,
    input logic [7:0]  rate
);
  integer fd;
  logic   fifo_rdy_q;
  logic   started;
  initial begin
    fd = $fopen($sformatf("pr7g3_fs.%m.log"), "w");
    started = 1'b0;
  end
  always @(posedge clk) begin
    if (!started) begin
      started    <= 1'b1;
      fifo_rdy_q <= fifo_rdy;
      $fwrite(fd, "I %0t fifo_rdy=%0d rate=%0d\n", $time, fifo_rdy, rate);
    end else if (fifo_rdy !== fifo_rdy_q) begin
      fifo_rdy_q <= fifo_rdy;
      $fwrite(fd, "T %0t fifo_rdy=%0d\n", $time, fifo_rdy);
    end
    if (tvalid && tready)
      $fwrite(fd, "B %0t %h %h %h %0d\n", $time, tdata, tkeep, tuser, tlast);
  end
  final $fclose(fd);
endmodule

bind frame_symbols pr7g3_fs u_pr7g3_fs (
    .clk     (clk_i),
    .tdata   (32'(m_axis_tdata)),
    .tkeep   (4'(m_axis_tkeep)),
    .tuser   (5'(m_axis_tuser)),
    .tvalid  (m_axis_tvalid),
    .tready  (m_axis_tready),
    .tlast   (m_axis_tlast),
    .fifo_rdy(fifo_axis_tready),
    .rate    (8'(curr_data_rate_i))
);
