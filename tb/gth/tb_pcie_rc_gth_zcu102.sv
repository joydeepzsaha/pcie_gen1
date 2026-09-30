// ===========================================================================
// tb_pcie_rc_gth_zcu102 -- xsim bench for the ZCU102 board top: the RC on
// PG239 with its debug cores.  sec 63 #5, 8-3.
//
// Like tb_pcie_rc_gth, this bench decides nothing.  It prints RAW TIMESTAMPED
// EVENTS, and evidence/gth-8/8-3/scripts/analyse_83_xsim.py pairs, times and
// judges them (sec 22.92).  The loop is the serial pins, rxp = txp.
//
// == WHAT "THE ILA AND VIO, SIMULATED" MEANS HERE ===========================
//
// AMD's simulation models of the debug cores are shells.  The ILA model is an
// empty module with the ILA's ports, and the VIO model drives each probe_out
// at its INIT value.  So the bench observes the debug cores at their PORTS:
//   * every ila_pclk / ila_free probe is printed as the ILA would sample it,
//     i.e. what a capture on the board would contain;
//   * the VIO's INIT values are what start the link (en, transmit_enable,
//     perst_n);
//   * +PULSE does what a user at the VIO console would do: it writes
//     vio_free.perst_n 0, then 1.  It is a force on the model's output.  A
//     reg holds its forced value after release, so the 1 is forced too.
//
// == THE PCLK-GAP WITNESS, CHECKED AGAINST PCLK ITSELF ======================
//
// Inside the FR windows the bench prints every clk125 edge (FR, ila_free's
// samples) and every pclk edge (PK).  The analysis finds each PCLK gap
// directly from the PK edges, and then again from the witness alone (FR).
// The two must agree, gap for gap.
//
// CLK_125 is a different oscillator from the PCIe refclk on the board.  Here
// it runs at 124.97 MHz (-250 ppm), so the synchronisers' sampling phase
// drifts against PCLK instead of sitting at one alignment.
//
// == EVENT LINES (every time in ps; sec 22.89 phase: FR and PK are sampled in
// the always block at the edge, i.e. PRE-edge values, what the flops capture) ==
//
//   CFG|0|<key>|<value>
//   EV|t|<name>|<hex>       value change of a debug-core port (ila_*.probeN,
//                           vio_*.probe_*) or of the reset chain
//   FR|t|<gap_cnt>|<tick>|<free_status>   each clk125 edge, inside a window
//   PK|t                    each pclk edge, inside a window
//   PULSE|t|<0|1>           the bench's VIO write
//   END|t|<reason>
//
// +MAX_US=<n>   (default 400) end at n us
// +FR_US=<n>    (default 40)  length of each FR/PK window: [0, n) us from time
//                             0, and [pulse - 1, pulse + n) us around the pulse
// +PULSE=<0|1>  (default 1)   5 us after the first fc_initialized, hold
//                             perst_n low for +PULSE_US (default 10), then end
//                             10 us after the second fc_initialized
// ===========================================================================
`timescale 1ps / 1ps

module tb_pcie_rc_gth_zcu102;

  localparam int  REFCLK_HALF_PS = 5000;         // 100 MHz, as tb_pcie_rc_gth
  localparam int  CLK125_HALF_PS = 4001;         // 124.97 MHz: -250 ppm against PCLK's source
  localparam int  POR_CYCLES     = 625;          // 5 us of clk125 = tb_pcie_rc_gth's 500 refclk cycles
  localparam time AFTER_FCINIT   = 10_000_000;

  reg  sys_clk_p = 1'b0;
  wire sys_clk_n = ~sys_clk_p;
  reg  clk125_p  = 1'b0;
  wire clk125_n  = ~clk125_p;
  always #(REFCLK_HALF_PS) sys_clk_p = ~sys_clk_p;
  always #(CLK125_HALF_PS) clk125_p  = ~clk125_p;

  wire [0:0] txp, txn;

  pcie_rc_gth_zcu102 #(.POR_CYCLES(POR_CYCLES), .SIM_FAST_LINK(1)) dut (
      .sys_clk_p(sys_clk_p), .sys_clk_n(sys_clk_n),
      .clk125_p(clk125_p),   .clk125_n(clk125_n),
      .pci_exp_txp(txp), .pci_exp_txn(txn),
      .pci_exp_rxp(txp), .pci_exp_rxn(txn));     // the serial loopback

  // ---- plusargs ---------------------------------------------------------------
  time max_time = 400_000_000;
  time fr_len   = 40_000_000;
  time pulse_len = 10_000_000;
  int  max_us, fr_us, pulse, pulse_us;
  initial begin
    $timeformat(-12, 0, "", 0);
    if ($value$plusargs("MAX_US=%d", max_us))     max_time  = max_us   * 64'd1_000_000;
    if ($value$plusargs("FR_US=%d", fr_us))       fr_len    = fr_us    * 64'd1_000_000;
    if (!$value$plusargs("PULSE=%d", pulse))      pulse     = 1;
    if ($value$plusargs("PULSE_US=%d", pulse_us)) pulse_len = pulse_us * 64'd1_000_000;
    $display("CFG|0|MAX_TIME_PS|%0d", max_time);
    $display("CFG|0|FR_LEN_PS|%0d", fr_len);
    $display("CFG|0|PULSE|%0d", pulse);
    $display("CFG|0|PULSE_LEN_PS|%0d", pulse_len);
    $display("CFG|0|REFCLK_HALF_PS|%0d", REFCLK_HALF_PS);
    $display("CFG|0|CLK125_HALF_PS|%0d", CLK125_HALF_PS);
    $display("CFG|0|POR_CYCLES|%0d", POR_CYCLES);
  end

  // ---- the windows ---------------------------------------------------------------
  // !! a function, not a continuous assign: an assign reading $time re-evaluates
  // only when its other operands change, so it would freeze at time 0.
  time win2_lo = 0, win2_hi = 0;
  function automatic bit in_win();
    return ($time < fr_len) || ($time >= win2_lo && $time < win2_hi);
  endfunction

  always @(posedge clk125_p)
    if (in_win()) $display("FR|%0t|%0d|%0d|%h", $time, dut.u_ila_free.probe0, dut.u_ila_free.probe1,
                         dut.u_ila_free.probe2);
  always @(posedge dut.pclk)
    if (in_win()) $display("PK|%0t", $time);

  // ---- the run: first training, the VIO pulse, the second training -------------------
  initial begin : run
    wait (dut.fc_initialized === 1'b1);
    if (pulse == 0) begin
      #(AFTER_FCINIT);
      $display("END|%0t|fc_initialized+%0d", $time, AFTER_FCINIT);
      $finish;
    end
    #(5_000_000);
    win2_lo = $time - 1_000_000;
    win2_hi = $time + fr_len;
    force dut.u_vio_free.probe_out0 = 1'b0;
    $display("PULSE|%0t|0", $time);
    #(pulse_len);
    force dut.u_vio_free.probe_out0 = 1'b1;
    $display("PULSE|%0t|1", $time);
    wait (dut.fc_initialized === 1'b0);
    wait (dut.fc_initialized === 1'b1);
    #(AFTER_FCINIT);
    $display("END|%0t|second fc_initialized+%0d", $time, AFTER_FCINIT);
    $finish;
  end

  initial begin
    #(max_time);
    $display("END|%0t|MAX_US", $time);
    $finish;
  end

  // ---- raw events: the debug cores at their ports, and the reset chain ----------------
  `define EV(NAME, SIG) always @(SIG) $display("EV|%0t|%s|%h", $time, NAME, SIG); \
                        initial #1 $display("EV|%0t|%s|%h", $time, NAME, SIG);
  `EV("por_done",            dut.por_done)
  `EV("sys_rst_n",           dut.sys_rst_n_r)
  `EV("rc_rst_i",            dut.u_rc_gth.u_rc.rst_i)
  `EV("ila_pclk.ltssm",      dut.u_ila_pclk.probe0)
  `EV("ila_pclk.link_up",    dut.u_ila_pclk.probe1)
  `EV("ila_pclk.fc_init",    dut.u_ila_pclk.probe2)
  `EV("ila_pclk.phystatus",  dut.u_ila_pclk.probe3)
  `EV("ila_pclk.phystatus_rst", dut.u_ila_pclk.probe4)
  `EV("ila_pclk.rxstatus",   dut.u_ila_pclk.probe5)
  `EV("ila_pclk.rxvalid",    dut.u_ila_pclk.probe6)
  `EV("ila_pclk.rxelecidle", dut.u_ila_pclk.probe7)
  `EV("ila_pclk.as_mac_in_detect", dut.u_ila_pclk.probe8)
  `EV("ila_pclk.txdetectrx", dut.u_ila_pclk.probe9)
  `EV("ila_pclk.txelecidle", dut.u_ila_pclk.probe10)
  `EV("ila_pclk.powerdown",  dut.u_ila_pclk.probe11)
  `EV("ila_free.free_status", dut.u_ila_free.probe2)
  `EV("vio_free.perst_n",    dut.u_vio_free.probe_out0)
  `EV("vio_free.gap_clear",  dut.u_vio_free.probe_out1)
  `EV("vio_free.gap_max",    dut.u_vio_free.probe_in0)
  `EV("vio_free.gap_events", dut.u_vio_free.probe_in1)
  `EV("vio_pclk.en",         dut.u_vio_pclk.probe_out0)
  `EV("vio_pclk.transmit_enable", dut.u_vio_pclk.probe_out1)
  `EV("vio_pclk.scan_start", dut.u_vio_pclk.probe_out2)
  `EV("vio_pclk.scan_bus",   dut.u_vio_pclk.probe_out3)
  `EV("vio_pclk.bar_enable", dut.u_vio_pclk.probe_out4)
  `EV("vio_pclk.bridge_enable", dut.u_vio_pclk.probe_out5)
  `EV("vio_pclk.link_status", dut.u_vio_pclk.probe_in0)
  `EV("vio_pclk.err_flags",  dut.u_vio_pclk.probe_in28)
  `undef EV

endmodule
