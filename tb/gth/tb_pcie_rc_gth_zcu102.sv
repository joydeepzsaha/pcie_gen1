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
//   * the VIO's INIT values are what start the link (en, transmit_enable).
//     perst_n's INIT is 0 (G0 A-dbg), so the bench releases it at
//     +PERST_REL_US as the VIO console would: a force to 1 on the model's
//     output.  The default, 0, releases it at time 0, which is what INIT 1 did;
//   * ila_pclk stores only the samples its storage qualifier marks (probe13,
//     the board top's ILA_PCLK CAPTURE CONTROL).  The bench prints one IS line
//     per sample the ILA would store, with its own count of pclk edges;
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
//   VREL|t|perst_n|1        (G0 A-dbg) the bench's release of perst_n, +PERST_REL_US
//   IS|t|<edges>|<ts>|<ltssm>|<phystatus>|<rxstatus>|<link_up>
//                           (G0 A-dbg) a pclk edge at which ila_pclk's qualifier
//                           (probe13) is 1: the sample the ILA stores.  <edges> =
//                           the bench's count of pclk edges before this one; <ts> =
//                           probe12, the board's pclk_ts; the rest pre-edge, hex
//   R2|t|<what>|<value>     (+R2 runs) the PCLK stop and the VIO write, below
//   PINCHK|t|samples=<n>|mismatch=<m>
//                           (sec 63 #23) once, at the end: the board top's PERST#
//                           pin slot_perst_assert against ~sys_rst_n_r at every
//                           clk125 edge, rising and falling; m = the edges at which
//                           the pin was not the inverse. The pin's changes are EV
//                           lines (slot_perst_assert)
//   XCHK|t|<tag>|checked=<n>|unknown=<k>|ctl_unknown=<m>
//   XU|t|<tag>|<name>       (G0 R4) $isunknown over u_rc's outputs, 16 PCLK edges
//                           after every release of its reset (a 1 -> 0 of rc_rst),
//                           at every link_up and fc_initialized rise, and at END;
//                           one XU line per checked output that reads unknown.
//                           The six outputs u_rc never drives (phy_txswing and the
//                           five equalisation outputs, HANDSHAKE sec 7) are the
//                           positive control: they must read unknown, ctl = 6
//   END|t|<reason>
//
// +MAX_US=<n>   (default 400) end at n us
// +FR_US=<n>    (default 40)  length of each FR/PK window: [0, n) us from time
//                             0, and [pulse - 1, pulse + n) us around the pulse
// +PULSE=<0|1>  (default 1)   5 us after the first fc_initialized, hold
//                             perst_n low for +PULSE_US (default 10), then end
//                             10 us after the second fc_initialized
// +R2=<0|1>     (default 0)   G0 R2, in place of the pulse: 5 us after the first
//                             fc_initialized, STOP PCLK by forcing PG239's own
//                             stop, bufg_gt_pclk.CE = 0 (window 2 opens 1 us
//                             earlier); 1 us later write perst_n 0, and
//                             +R2_PERST_NS (default 200) later write it 1; 1 us
//                             later release CE; end 10 us after fc_initialized
//                             falls and rises again
// +PERST_REL_US=<n> (default 0) G0 A-dbg: release vio_free.perst_n (INIT 0) at n us
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
  wire       slot_perst_assert;                  // 1 = the slot's PERST# asserted

  pcie_rc_gth_zcu102 #(.POR_CYCLES(POR_CYCLES), .SIM_FAST_LINK(1)) dut (
      .sys_clk_p(sys_clk_p), .sys_clk_n(sys_clk_n),
      .clk125_p(clk125_p),   .clk125_n(clk125_n),
      .pci_exp_txp(txp), .pci_exp_txn(txn),
      .pci_exp_rxp(txp), .pci_exp_rxn(txn),      // the serial loopback
      .slot_perst_assert(slot_perst_assert));

  // ---- plusargs ---------------------------------------------------------------
  time max_time = 400_000_000;
  time fr_len   = 40_000_000;
  time pulse_len = 10_000_000;
  int  max_us, fr_us, pulse, pulse_us;
  int  r2, r2_perst_ns;
  int  perst_rel_us;
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
    if (!$value$plusargs("R2=%d", r2))                   r2          = 0;
    if (!$value$plusargs("R2_PERST_NS=%d", r2_perst_ns)) r2_perst_ns = 200;
    $display("CFG|0|R2|%0d", r2);
    $display("CFG|0|R2_PERST_NS|%0d", r2_perst_ns);
    // G0 A-dbg: vio_free.perst_n's INIT is 0; release it as the VIO console would
    if (!$value$plusargs("PERST_REL_US=%d", perst_rel_us)) perst_rel_us = 0;
    $display("CFG|0|PERST_REL_US|%0d", perst_rel_us);
    if (perst_rel_us > 0) #(perst_rel_us * 64'd1_000_000);
    force dut.u_vio_free.probe_out0 = 1'b1;
    $display("VREL|%0t|perst_n|1", $time);
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

  // ---- G0 A-dbg: ila_pclk's storage qualification, at the ILA's ports -------------------
  // The ILA model is a shell, so the bench prints what the ILA would store with its
  // capture condition set to probe13 == 1: the pre-edge probes at every pclk edge where
  // probe13 is 1.  pclk_edges is the bench's own count, independent of the board's pclk_ts.
  longint unsigned pclk_edges = 0;
  always @(posedge dut.pclk) begin
    if (dut.u_ila_pclk.probe13 === 1'b1)
      $display("IS|%0t|%0d|%0d|%h|%h|%h|%h", $time, pclk_edges, dut.u_ila_pclk.probe12,
               dut.u_ila_pclk.probe0, dut.u_ila_pclk.probe3, dut.u_ila_pclk.probe5,
               dut.u_ila_pclk.probe1);
    pclk_edges++;
  end

  // ---- the run: first training, the VIO pulse, the second training -------------------
  `define PCLK_CE dut.u_rc_gth.u_pg239.inst.diablo_gt.diablo_gt_phy_wrapper.phy_clk_i.bufg_gt_pclk.CE
  initial begin : run
    wait (dut.fc_initialized === 1'b1);
    if (r2 != 0) begin
      #(5_000_000);
      win2_lo = $time - 1_000_000;
      win2_hi = $time + fr_len;
      force `PCLK_CE = 1'b0;
      $display("R2|%0t|pclk_ce|0", $time);
      #(1_000_000);
      force dut.u_vio_free.probe_out0 = 1'b0;
      $display("R2|%0t|perst_n|0", $time);
      #(r2_perst_ns * 1000);
      force dut.u_vio_free.probe_out0 = 1'b1;
      $display("R2|%0t|perst_n|1", $time);
      #(1_000_000);
      release `PCLK_CE;
      $display("R2|%0t|pclk_ce|released", $time);
      wait (dut.fc_initialized === 1'b0);
      wait (dut.fc_initialized === 1'b1);
      #(AFTER_FCINIT);
      xchk("END");
      $display("END|%0t|R2 fc_initialized+%0d", $time, AFTER_FCINIT);
      $finish;
    end
    if (pulse == 0) begin
      #(AFTER_FCINIT);
      xchk("END");
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
    xchk("END");
    $display("END|%0t|second fc_initialized+%0d", $time, AFTER_FCINIT);
    $finish;
  end

  initial begin
    #(max_time);
    xchk("END");
    $display("END|%0t|MAX_US", $time);
    $finish;
  end

  // ---- raw events: the debug cores at their ports, and the reset chain ----------------
  `define EV(NAME, SIG) always @(SIG) $display("EV|%0t|%s|%h", $time, NAME, SIG); \
                        initial #1 $display("EV|%0t|%s|%h", $time, NAME, SIG);
  `EV("por_done",            dut.por_done)
  `EV("sys_rst_n",           dut.sys_rst_n_r)
  `EV("slot_perst_assert",   slot_perst_assert)
  `EV("rc_rst_i",            dut.u_rc_gth.u_rc.rst_i)
  `EV("rc_rst_req",          dut.u_rc_gth.rc_rst_req)
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

  // ---- sec 63 #23: the slot's PERST# pin is the inverse of sys_rst_n_r ----------------
  // Sampled at every clk125 edge, both directions, in the always block at the edge: the
  // pre-edge values of both, which sys_rst_n_r's own nonblocking update does not change.
  int unsigned pin_samples = 0, pin_mismatch = 0;
  always @(clk125_p) begin
    pin_samples++;
    if (slot_perst_assert !== ~dut.sys_rst_n_r) pin_mismatch++;
  end
  final $display("PINCHK|%0t|samples=%0d|mismatch=%0d", $time, pin_samples, pin_mismatch);

  // ---- G0 R4: no X on any RC output after its reset releases ---------------------------
  // u_rc's 115 outputs: the 109 it drives are checked; the six it never drives are the
  // positive control (they must read unknown).  Generated from pcie_rc_top.sv's port list.
  // !! Integers and one $display per unknown output, and NO string built in the task:
  // T1 appended to a string variable here and xsim 2023.2 died in the task with
  // "FATAL_ERROR: Vivado Simulator kernel has discovered an exceptional condition"
  // (and exited 0).
  `define XC(NAME, SIG) n++; if ($isunknown(SIG)) begin unk++; $display("XU|%0t|%s|%s", $time, tag, NAME); end
  `define XCTL(SIG) if ($isunknown(SIG)) ctl++;
  task automatic xchk(input string tag);
    int n, unk, ctl;
    n = 0; unk = 0; ctl = 0;
    `XC("phy_txdata", dut.u_rc_gth.u_rc.phy_txdata)
    `XC("phy_txdata_valid", dut.u_rc_gth.u_rc.phy_txdata_valid)
    `XC("phy_txdatak", dut.u_rc_gth.u_rc.phy_txdatak)
    `XC("phy_txstart_block", dut.u_rc_gth.u_rc.phy_txstart_block)
    `XC("phy_txsync_header", dut.u_rc_gth.u_rc.phy_txsync_header)
    `XC("phy_txdetectrx", dut.u_rc_gth.u_rc.phy_txdetectrx)
    `XC("phy_txelecidle", dut.u_rc_gth.u_rc.phy_txelecidle)
    `XC("phy_txcompliance", dut.u_rc_gth.u_rc.phy_txcompliance)
    `XC("phy_rxpolarity", dut.u_rc_gth.u_rc.phy_rxpolarity)
    `XC("phy_powerdown", dut.u_rc_gth.u_rc.phy_powerdown)
    `XC("phy_rate", dut.u_rc_gth.u_rc.phy_rate)
    `XC("phy_txmargin", dut.u_rc_gth.u_rc.phy_txmargin)
    `XC("phy_txdeemph", dut.u_rc_gth.u_rc.phy_txdeemph)
    `XC("pipe_width_o", dut.u_rc_gth.u_rc.pipe_width_o)
    `XC("as_mac_in_detect", dut.u_rc_gth.u_rc.as_mac_in_detect)
    `XC("as_cdr_hold_req", dut.u_rc_gth.u_rc.as_cdr_hold_req)
    `XC("ltssm_debug_state", dut.u_rc_gth.u_rc.ltssm_debug_state)
    `XC("link_up_o", dut.u_rc_gth.u_rc.link_up_o)
    `XC("fc_initialized_o", dut.u_rc_gth.u_rc.fc_initialized_o)
    `XC("fc_init_done_o", dut.u_rc_gth.u_rc.fc_init_done_o)
    `XC("ok_to_issue_o", dut.u_rc_gth.u_rc.ok_to_issue_o)
    `XC("cfg_bus_number_o", dut.u_rc_gth.u_rc.cfg_bus_number_o)
    `XC("cfg_device_number_o", dut.u_rc_gth.u_rc.cfg_device_number_o)
    `XC("cfg_function_number_o", dut.u_rc_gth.u_rc.cfg_function_number_o)
    `XC("scan_busy_o", dut.u_rc_gth.u_rc.scan_busy_o)
    `XC("scan_done_o", dut.u_rc_gth.u_rc.scan_done_o)
    `XC("scan_error_o", dut.u_rc_gth.u_rc.scan_error_o)
    `XC("scan_error_code_o", dut.u_rc_gth.u_rc.scan_error_code_o)
    `XC("err_credit_blocked_o", dut.u_rc_gth.u_rc.err_credit_blocked_o)
    `XC("device_present_o", dut.u_rc_gth.u_rc.device_present_o)
    `XC("unsupported_device_o", dut.u_rc_gth.u_rc.unsupported_device_o)
    `XC("device_bdf_o", dut.u_rc_gth.u_rc.device_bdf_o)
    `XC("vendor_id_o", dut.u_rc_gth.u_rc.vendor_id_o)
    `XC("device_id_o", dut.u_rc_gth.u_rc.device_id_o)
    `XC("header_type_o", dut.u_rc_gth.u_rc.header_type_o)
    `XC("multifunction_o", dut.u_rc_gth.u_rc.multifunction_o)
    `XC("bar_busy_o", dut.u_rc_gth.u_rc.bar_busy_o)
    `XC("enum_done_o", dut.u_rc_gth.u_rc.enum_done_o)
    `XC("enum_error_o", dut.u_rc_gth.u_rc.enum_error_o)
    `XC("enum_error_code_o", dut.u_rc_gth.u_rc.enum_error_code_o)
    `XC("bar_count_o", dut.u_rc_gth.u_rc.bar_count_o)
    `XC("bar_valid_o", dut.u_rc_gth.u_rc.bar_valid_o)
    `XC("bar_is_64_o", dut.u_rc_gth.u_rc.bar_is_64_o)
    `XC("bar_prefetch_o", dut.u_rc_gth.u_rc.bar_prefetch_o)
    `XC("bar_size_o", dut.u_rc_gth.u_rc.bar_size_o)
    `XC("bar_addr_o", dut.u_rc_gth.u_rc.bar_addr_o)
    `XC("io_bar_mask_o", dut.u_rc_gth.u_rc.io_bar_mask_o)
    `XC("bus_done_o", dut.u_rc_gth.u_rc.bus_done_o)
    `XC("bus_bypassed_o", dut.u_rc_gth.u_rc.bus_bypassed_o)
    `XC("sec_scan_done_o", dut.u_rc_gth.u_rc.sec_scan_done_o)
    `XC("sec_device_present_o", dut.u_rc_gth.u_rc.sec_device_present_o)
    `XC("sec_unsupported_device_o", dut.u_rc_gth.u_rc.sec_unsupported_device_o)
    `XC("sec_device_bdf_o", dut.u_rc_gth.u_rc.sec_device_bdf_o)
    `XC("sec_vendor_id_o", dut.u_rc_gth.u_rc.sec_vendor_id_o)
    `XC("sec_device_id_o", dut.u_rc_gth.u_rc.sec_device_id_o)
    `XC("sec_header_type_o", dut.u_rc_gth.u_rc.sec_header_type_o)
    `XC("sec_multifunction_o", dut.u_rc_gth.u_rc.sec_multifunction_o)
    `XC("sec_enum_done_o", dut.u_rc_gth.u_rc.sec_enum_done_o)
    `XC("sec_bar_count_o", dut.u_rc_gth.u_rc.sec_bar_count_o)
    `XC("sec_bar_valid_o", dut.u_rc_gth.u_rc.sec_bar_valid_o)
    `XC("sec_bar_is_64_o", dut.u_rc_gth.u_rc.sec_bar_is_64_o)
    `XC("sec_bar_prefetch_o", dut.u_rc_gth.u_rc.sec_bar_prefetch_o)
    `XC("sec_bar_size_o", dut.u_rc_gth.u_rc.sec_bar_size_o)
    `XC("sec_bar_addr_o", dut.u_rc_gth.u_rc.sec_bar_addr_o)
    `XC("sec_io_bar_mask_o", dut.u_rc_gth.u_rc.sec_io_bar_mask_o)
    `XC("s_axis_rq_tready", dut.u_rc_gth.u_rc.s_axis_rq_tready)
    `XC("m_axis_rc_tdata", dut.u_rc_gth.u_rc.m_axis_rc_tdata)
    `XC("m_axis_rc_tkeep", dut.u_rc_gth.u_rc.m_axis_rc_tkeep)
    `XC("m_axis_rc_tvalid", dut.u_rc_gth.u_rc.m_axis_rc_tvalid)
    `XC("m_axis_rc_tlast", dut.u_rc_gth.u_rc.m_axis_rc_tlast)
    `XC("pcie_rq_tag_o", dut.u_rc_gth.u_rc.pcie_rq_tag_o)
    `XC("pcie_rq_tag_vld_o", dut.u_rc_gth.u_rc.pcie_rq_tag_vld_o)
    `XC("rq_engine_owns_o", dut.u_rc_gth.u_rc.rq_engine_owns_o)
    `XC("m_axis_cq_tdata", dut.u_rc_gth.u_rc.m_axis_cq_tdata)
    `XC("m_axis_cq_tkeep", dut.u_rc_gth.u_rc.m_axis_cq_tkeep)
    `XC("m_axis_cq_tvalid", dut.u_rc_gth.u_rc.m_axis_cq_tvalid)
    `XC("m_axis_cq_tlast", dut.u_rc_gth.u_rc.m_axis_cq_tlast)
    `XC("m_axis_cq_tuser", dut.u_rc_gth.u_rc.m_axis_cq_tuser)
    `XC("s_axis_cc_tready", dut.u_rc_gth.u_rc.s_axis_cc_tready)
    `XC("cq_dropped_o", dut.u_rc_gth.u_rc.cq_dropped_o)
    `XC("cq_error_code_o", dut.u_rc_gth.u_rc.cq_error_code_o)
    `XC("cq_gearbox_error_o", dut.u_rc_gth.u_rc.cq_gearbox_error_o)
    `XC("cc_protocol_error_o", dut.u_rc_gth.u_rc.cc_protocol_error_o)
    `XC("cc_error_code_o", dut.u_rc_gth.u_rc.cc_error_code_o)
    `XC("cc_gearbox_error_o", dut.u_rc_gth.u_rc.cc_gearbox_error_o)
    `XC("rq_protocol_error_o", dut.u_rc_gth.u_rc.rq_protocol_error_o)
    `XC("rq_error_code_o", dut.u_rc_gth.u_rc.rq_error_code_o)
    `XC("rq_gearbox_error_o", dut.u_rc_gth.u_rc.rq_gearbox_error_o)
    `XC("rc_unexpected_completion_o", dut.u_rc_gth.u_rc.rc_unexpected_completion_o)
    `XC("rc_completion_error_code_o", dut.u_rc_gth.u_rc.rc_completion_error_code_o)
    `XC("rc_protocol_error_o", dut.u_rc_gth.u_rc.rc_protocol_error_o)
    `XC("rc_error_code_o", dut.u_rc_gth.u_rc.rc_error_code_o)
    `XC("rc_gearbox_error_o", dut.u_rc_gth.u_rc.rc_gearbox_error_o)
    `XC("command_error_valid_o", dut.u_rc_gth.u_rc.command_error_valid_o)
    `XC("command_error_code_o", dut.u_rc_gth.u_rc.command_error_code_o)
    `XC("malformed_o", dut.u_rc_gth.u_rc.malformed_o)
    `XC("rx_error_valid_o", dut.u_rc_gth.u_rc.rx_error_valid_o)
    `XC("rx_error_code_o", dut.u_rc_gth.u_rc.rx_error_code_o)
    `XC("rx_ecrc_error_o", dut.u_rc_gth.u_rc.rx_ecrc_error_o)
    `XC("tx_error_valid_o", dut.u_rc_gth.u_rc.tx_error_valid_o)
    `XC("tx_error_code_o", dut.u_rc_gth.u_rc.tx_error_code_o)
    `XC("tx_fc_blocked_o", dut.u_rc_gth.u_rc.tx_fc_blocked_o)
    `XC("credit_error_o", dut.u_rc_gth.u_rc.credit_error_o)
    `XC("vc_overflow_o", dut.u_rc_gth.u_rc.vc_overflow_o)
    `XC("cpl_timeout_valid_o", dut.u_rc_gth.u_rc.cpl_timeout_valid_o)
    `XC("cpl_timeout_tag_o", dut.u_rc_gth.u_rc.cpl_timeout_tag_o)
    `XC("late_cpl_valid_o", dut.u_rc_gth.u_rc.late_cpl_valid_o)
    `XC("late_cpl_tag_o", dut.u_rc_gth.u_rc.late_cpl_tag_o)
    `XC("outstanding_o", dut.u_rc_gth.u_rc.outstanding_o)
    `XCTL(dut.u_rc_gth.u_rc.phy_txswing)
    `XCTL(dut.u_rc_gth.u_rc.phy_txeq_ctrl)
    `XCTL(dut.u_rc_gth.u_rc.phy_txeq_preset)
    `XCTL(dut.u_rc_gth.u_rc.phy_txeq_coeff)
    `XCTL(dut.u_rc_gth.u_rc.phy_rxeq_ctrl)
    `XCTL(dut.u_rc_gth.u_rc.phy_rxeq_txpreset)
    $display("XCHK|%0t|%s|checked=%0d|unknown=%0d|ctl_unknown=%0d", $time, tag, n, unk, ctl);
  endtask
  `undef XC
  `undef XCTL

  // A RELEASE is 1 -> 0 only.  xpm_cdc_async_rst's stages start at 0 and are preset
  // at time 0, so rc_rst's first transition is x -> 0, which @(negedge) takes for one.
  logic rst_prev = 1'bx;
  event rst_released;
  always @(dut.u_rc_gth.rc_rst) begin
    if (rst_prev === 1'b1 && dut.u_rc_gth.rc_rst === 1'b0) -> rst_released;
    rst_prev = dut.u_rc_gth.rc_rst;
  end
  always @(rst_released) begin : xchk_release
    repeat (16) @(posedge dut.pclk);
    xchk("rst_release+16");
  end
  always @(posedge dut.link_up)        xchk("link_up");
  always @(posedge dut.fc_initialized) xchk("fc_init");

endmodule
