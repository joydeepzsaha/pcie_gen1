// ===========================================================================
// tb_pcie_rc_gth -- xsim bench for pcie_rc_gth_top: the RC through AMD's PCIe
// PHY IP (PG239) and back to itself over a serial loopback.  sec 63 #5, 8-2.
//
// xsim, not Verilator: the GT model is encrypted (D-8.7).  This bench decides
// nothing.  It prints RAW TIMESTAMPED EVENTS and the Python analysis beside it
// (evidence/gth-8/8-2/) pairs, classifies and times them (sec 22.92).
//
// == THE LOOPBACK ===========================================================
//
// PG239 exposes no loopback control (PHASE0_8-2.md sec 3), so the loop is
// the serial pins: rxp = txp, rxn = txn.  That runs the whole GT -- TX PMA,
// serialiser, CDR, comma alignment, 8b/10b and the RX elastic buffer.  With
// SIM_TX_EIDLE_DRIVE_LEVEL = "Z" (the generated wrapper), TX electrical idle
// arrives at RX as Z.  A self-loop cannot complete FC init (sec 63 #7a: one
// scrambler against its own output, LCRC fails), so nothing here waits for it.
//
// == +ASSIST=rtl | zero =====================================================
//
// rtl  (default): the assist ports are whatever u_rc drives.  At 8-2 Phase 1
//      that is nothing -- an undriven output reg, X in 4-state simulation.
// zero: the bench forces the wire at the IP's input to 0, which is what
//      synthesis makes of an undriven output.  The pair separates X
//      pessimism from design.
//
// == +RXDV=ip | rxvalid  (a PROBE of a candidate fix, not the fix) ===========
//
// ip      (default): u_rc's phy_rxdata_valid is the IP's.  PG239 Table 7 p.13
//         defines that port for Gen3 and above only, and the IP holds it 0
//         at Gen1 (measured, 8-2 Phase 1).
// rxvalid: the bench forces u_rc's phy_rxdata_valid input to the IP's
//         phy_rxvalid ("symbol lock and valid data", p.16).  It measures what
//         the RC does once its RX data path sees valid beats; it changes no
//         source file.
//
// == EVENT LINES (every time in ps) =========================================
//
//   EV|t|<name>|<hex>   a value change of a named signal, printed in the time
//                       step it happens (the new value)
//   CK|t|pclk|<n>       the first 16 rising edges of phy_pclk, then every 4096th
//   PT|t|<txdata>|<txk>|<txelecidle>|<txcompliance>
//   PR|t|<rxdata>|<rxk>|<rxvalid>|<rxstatus>|<rxelecidle>|<phystatus>
//                       one pair per phy_pclk rising edge, from the first edge.
//                       !! Read in the always block at the edge: the PRE-edge
//                       values, i.e. what every flop clocked by phy_pclk
//                       captures at that edge (sec 22.89: the phase stated).
//   END|t|<reason>
// ===========================================================================
`timescale 1ps / 1ps

module tb_pcie_rc_gth;

  localparam int  REFCLK_HALF_PS = 5000;       // 100 MHz
  localparam int  PERST_CYCLES   = 500;        // PG239's own board.v
  localparam time MAX_TIME       = 400_000_000; // 400 us
  localparam time AFTER_LINKUP   = 30_000_000;  // run 30 us past the first link_up

  reg        sys_clk_p = 1'b0;
  wire       sys_clk_n = ~sys_clk_p;
  reg        sys_rst_n = 1'b0;
  wire [0:0] txp, txn;
  wire       pclk;

  always #(REFCLK_HALF_PS) sys_clk_p = ~sys_clk_p;

  pcie_rc_gth_top #(.SIM_FAST_LINK(1)) dut (
      .sys_clk_p(sys_clk_p), .sys_clk_n(sys_clk_n), .sys_rst_n(sys_rst_n),
      .pci_exp_txp(txp), .pci_exp_txn(txn),
      .pci_exp_rxp(txp), .pci_exp_rxn(txn),          // the serial loopback
      .pclk_o(pclk),
      .en_i(1'b1), .transmit_enable_i(1'b1),
      .ltssm_debug_state(),
      .link_up_o(), .fc_initialized_o(), .fc_init_done_o(), .ok_to_issue_o(),
      .requester_id_i(16'h0000), .completer_id_i(16'h0000), .bus_number_i(8'h00),
      .device_number_i(5'h00), .function_number_i(3'h0), .memory_enable_i(1'b1),
      .extended_tag_enable_i(1'b0), .max_payload_bytes_i(13'd128),
      .max_read_bytes_i(13'd512), .rcb_128b_i(1'b0),
      .cfg_bus_number_o(), .cfg_device_number_o(), .cfg_function_number_o(),
      .scan_start_i(1'b0), .scan_bus_i(8'h00), .bar_enable_i(1'b0), .bridge_enable_i(1'b0),
      .scan_busy_o(), .scan_done_o(), .scan_error_o(), .scan_error_code_o(),
      .err_credit_blocked_o(), .device_present_o(), .unsupported_device_o(),
      .device_bdf_o(), .vendor_id_o(), .device_id_o(), .header_type_o(),
      .multifunction_o(), .bar_busy_o(), .enum_done_o(), .enum_error_o(),
      .enum_error_code_o(), .bar_count_o(), .bar_valid_o(), .bar_is_64_o(),
      .bar_prefetch_o(), .bar_size_o(), .bar_addr_o(), .io_bar_mask_o(),
      .bus_done_o(), .bus_bypassed_o(), .sec_scan_done_o(), .sec_device_present_o(),
      .sec_unsupported_device_o(), .sec_device_bdf_o(), .sec_vendor_id_o(),
      .sec_device_id_o(), .sec_header_type_o(), .sec_multifunction_o(),
      .sec_enum_done_o(), .sec_bar_count_o(), .sec_bar_valid_o(), .sec_bar_is_64_o(),
      .sec_bar_prefetch_o(), .sec_bar_size_o(), .sec_bar_addr_o(), .sec_io_bar_mask_o(),
      .s_axis_rq_tdata('0), .s_axis_rq_tkeep('0), .s_axis_rq_tvalid(1'b0),
      .s_axis_rq_tlast(1'b0), .s_axis_rq_tuser('0), .s_axis_rq_tready(),
      .m_axis_rc_tdata(), .m_axis_rc_tkeep(), .m_axis_rc_tvalid(), .m_axis_rc_tlast(),
      .m_axis_rc_tready(1'b1), .pcie_rq_tag_o(), .pcie_rq_tag_vld_o(), .rq_engine_owns_o(),
      .m_axis_cq_tdata(), .m_axis_cq_tkeep(), .m_axis_cq_tvalid(), .m_axis_cq_tlast(),
      .m_axis_cq_tuser(), .m_axis_cq_tready(1'b1),
      .s_axis_cc_tdata('0), .s_axis_cc_tkeep('0), .s_axis_cc_tvalid(1'b0),
      .s_axis_cc_tlast(1'b0), .s_axis_cc_tuser('0), .s_axis_cc_tready(),
      .cq_dropped_o(), .cq_error_code_o(), .cq_gearbox_error_o(),
      .cc_protocol_error_o(), .cc_error_code_o(), .cc_gearbox_error_o(),
      .rq_protocol_error_o(), .rq_error_code_o(), .rq_gearbox_error_o(),
      .rc_unexpected_completion_o(), .rc_completion_error_code_o(),
      .rc_protocol_error_o(), .rc_error_code_o(), .rc_gearbox_error_o(),
      .command_error_valid_o(), .command_error_code_o(), .malformed_o(),
      .rx_error_valid_o(), .rx_error_code_o(), .rx_ecrc_error_o(),
      .tx_error_valid_o(), .tx_error_code_o(), .tx_fc_blocked_o(),
      .credit_error_o(), .vc_overflow_o(), .cpl_timeout_valid_o(),
      .cpl_timeout_tag_o(), .late_cpl_valid_o(), .late_cpl_tag_o(), .outstanding_o()
  );

  // ---- +ASSIST, +RXDV ------------------------------------------------------
  string assist, rxdv;
  initial begin
    if (!$value$plusargs("ASSIST=%s", assist)) assist = "rtl";
    $display("CFG|0|ASSIST|%s", assist);
    $display("CFG|0|REFCLK_HALF_PS|%0d", REFCLK_HALF_PS);
    $display("CFG|0|PERST_CYCLES|%0d", PERST_CYCLES);
    if (!$value$plusargs("RXDV=%s", rxdv)) rxdv = "ip";
    $display("CFG|0|RXDV|%s", rxdv);
    if (rxdv == "rxvalid") force dut.phy_rxdata_valid = dut.phy_rxvalid;
    else if (rxdv != "ip") begin
      $display("END|%0t|bad RXDV=%s", $time, rxdv);
      $finish;
    end
    if (assist == "zero") begin
      force dut.as_mac_in_detect = 1'b0;
      force dut.as_cdr_hold_req  = 1'b0;
    end else if (assist != "rtl") begin
      $display("END|%0t|bad ASSIST=%s", $time, assist);
      $finish;
    end
  end

  // ---- PERST# ---------------------------------------------------------------
  initial begin
    repeat (PERST_CYCLES) @(posedge sys_clk_p);
    sys_rst_n = 1'b1;
  end

  // ---- raw events -------------------------------------------------------------
  `define EV(NAME, SIG) always @(SIG) $display("EV|%0t|%s|%h", $time, NAME, SIG);
  `EV("sys_rst_n",         sys_rst_n)
  `EV("gt_gtpowergood",    dut.gt_gtpowergood)
  `EV("gt_cplllock",       dut.u_pg239.inst.diablo_gt.diablo_gt_phy_wrapper.gt_cplllock)
  `EV("gt_txresetdone",    dut.u_pg239.inst.diablo_gt.diablo_gt_phy_wrapper.gt_txresetdone)
  `EV("gt_rxresetdone",    dut.u_pg239.inst.diablo_gt.diablo_gt_phy_wrapper.gt_rxresetdone)
  `EV("phy_phystatus_rst", dut.phy_phystatus_rst)
  `EV("phy_phystatus",     dut.phy_phystatus)
  `EV("phy_rxstatus",      dut.phy_rxstatus)
  `EV("phy_rxelecidle",    dut.phy_rxelecidle)
  `EV("phy_rxvalid",       dut.phy_rxvalid)
  `EV("phy_rxdata_valid",  dut.phy_rxdata_valid)
  `EV("phy_txdata_valid",  dut.phy_txdata_valid)
  `EV("phy_rxstart_block", dut.pg_rxstart_block)
  `EV("phy_rxsync_header", dut.phy_rxsync_header)
  `EV("phy_txdetectrx",    dut.phy_txdetectrx)
  `EV("phy_txelecidle",    dut.phy_txelecidle)
  `EV("phy_txcompliance",  dut.phy_txcompliance)
  `EV("phy_powerdown",     dut.phy_powerdown)
  `EV("phy_rate",          dut.phy_rate)
  `EV("phy_rxpolarity",    dut.phy_rxpolarity)
  `EV("as_mac_in_detect",  dut.as_mac_in_detect)
  `EV("as_cdr_hold_req",   dut.as_cdr_hold_req)
  `EV("ltssm_state",       dut.ltssm_debug_state)
  `EV("link_up",           dut.link_up_o)
  `EV("fc_initialized",    dut.fc_initialized_o)
  `EV("rc_rst_i",          dut.u_rc.rst_i)
  `EV("idle_valid",        dut.u_rc.u_phy.idle_valid)
  `EV("ts1_valid",         dut.u_rc.u_phy.ts1_valid)
  `EV("ts2_valid",         dut.u_rc.u_phy.ts2_valid)
  `undef EV

  // ---- phy_pclk and the PIPE, per edge -------------------------------------
  longint unsigned n_pclk = 0;
  always @(posedge pclk) begin
    n_pclk <= n_pclk + 1;
    if (n_pclk < 16 || (n_pclk % 4096) == 0) $display("CK|%0t|pclk|%0d", $time, n_pclk);
    $display("PT|%0t|%h|%b|%b|%b", $time, dut.phy_txdata, dut.phy_txdatak,
             dut.phy_txelecidle, dut.phy_txcompliance);
    $display("PR|%0t|%h|%b|%b|%h|%b|%b", $time, dut.pg_rxdata[15:0], dut.phy_rxdatak,
             dut.phy_rxvalid, dut.phy_rxstatus, dut.phy_rxelecidle, dut.phy_phystatus);
  end

  // ---- the end ----------------------------------------------------------------
  time t_linkup = 0;
  always @(posedge dut.link_up_o) if (t_linkup == 0) t_linkup = $time;
  initial begin
    forever begin
      #1_000_000;  // 1 us
      if (t_linkup != 0 && $time >= t_linkup + AFTER_LINKUP) begin
        $display("END|%0t|link_up+%0d", $time, AFTER_LINKUP); $finish;
      end
      if ($time >= MAX_TIME) begin
        $display("END|%0t|max_time", $time); $finish;
      end
    end
  end

endmodule
