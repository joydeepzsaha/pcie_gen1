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
// == +FAREND=loop | commafree ===============================================
//
// loop      (default): rxp = txp, rxn = txn for the whole run.  Ends 10 us after
//           fc_initialized rises (FC init completes on the RC's own echoed
//           InitFC2, sec 63 #7d; through the IP at 78.7 us, 8-2 Phase 1 sec 11),
//           or at +MAX_US.
// commafree: the loop until the LTSSM first enters Polling; from then on the RX
//           pins carry a 1.25 GHz square wave (1010... at 2.5 Gb/s).  The line
//           leaves Electrical Idle but carries no comma, so the RC never locks
//           a Symbol, Polling.Active times out after 24 ms and the LTSSM
//           re-enters Detect -- the second Detect entry, with the IP's receiver
//           termination FSM already in IDLE, that W2' needs to see it re-arm.
//           Run it with +MAX_US.
//
// !! Phase 1's +ASSIST and +RXDV probes are gone: C2 made the RC read
// phy_rxvalid, and "driver removed" is a source mutant (MR-8.2a), not a force.
//
// == EVENT LINES (every time in ps) =========================================
//
// !! %t prints in $timeformat's unit, which defaults to the GLOBAL precision
// -- 1 fs here, because of the secureip GT model -- not in this module's
// 1 ps.  8-2 Phase 1's first runs (at 9746194) printed fs under this header;
// the $timeformat call below makes the header true.
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
//
// +MAX_US=<n>  (default 400) end the run at n us.
// +PIPE_US=<n> (default: no limit) stop the per-edge PT/PR lines after n us, so a
//              long run keeps its EV lines without a per-cycle log.
// ===========================================================================
`timescale 1ps / 1ps

module tb_pcie_rc_gth;

  localparam int  REFCLK_HALF_PS = 5000;       // 100 MHz
  localparam int  PERST_CYCLES   = 500;        // PG239's own board.v
  localparam time AFTER_FCINIT   = 10_000_000;  // loop mode: run 10 us past fc_initialized

  reg        sys_clk_p = 1'b0;
  wire       sys_clk_n = ~sys_clk_p;
  reg        sys_rst_n = 1'b0;
  wire [0:0] txp, txn;
  reg        commafree = 1'b0;                  // +FAREND=commafree, after Polling entry
  reg        sq = 1'b0;                         // 1.25 GHz: 1010... at 2.5 Gb/s
  always #400 sq = ~sq;
  wire [0:0] rxp = commafree ? sq  : txp;
  wire [0:0] rxn = commafree ? ~sq : txn;
  wire       pclk;

  always #(REFCLK_HALF_PS) sys_clk_p = ~sys_clk_p;

  pcie_rc_gth_top #(.SIM_FAST_LINK(1)) dut (
      .sys_clk_p(sys_clk_p), .sys_clk_n(sys_clk_n), .sys_rst_n(sys_rst_n),
      .pci_exp_txp(txp), .pci_exp_txn(txn),
      .pci_exp_rxp(rxp), .pci_exp_rxn(rxn),          // the far end, below
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

  // ---- plusargs --------------------------------------------------------------
  string farend;
  time   max_time  = 400_000_000;                 // 400 us
  time   pipe_time = 0;                           // 0 = no limit
  int    max_us, pipe_us;
  initial begin
    $timeformat(-12, 0, "", 0);                   // %t in ps (see the header)
    if ($value$plusargs("MAX_US=%d", max_us))  max_time  = max_us  * 64'd1_000_000;
    if ($value$plusargs("PIPE_US=%d", pipe_us)) pipe_time = pipe_us * 64'd1_000_000;
    $display("CFG|0|MAX_TIME_PS|%0d", max_time);
    $display("CFG|0|PIPE_TIME_PS|%0d", pipe_time);
    if (!$value$plusargs("FAREND=%s", farend)) farend = "loop";
    $display("CFG|0|FAREND|%s", farend);
    $display("CFG|0|REFCLK_HALF_PS|%0d", REFCLK_HALF_PS);
    $display("CFG|0|PERST_CYCLES|%0d", PERST_CYCLES);
    if (farend != "loop" && farend != "commafree") begin
      $display("END|%0t|bad FAREND=%s", $time, farend);
      $finish;
    end
  end

  // ---- +FAREND=commafree: the loop until the first Polling entry ------------
  initial begin
    wait (farend == "commafree");
    wait (dut.ltssm_debug_state[4:0] == 5'b00010);        // the Polling family
    commafree = 1'b1;
    $display("EV|%0t|farend_commafree|1", $time);
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
  // 8-2 Phase 2: the reset inputs W5 measures, the IP's receiver termination
  // (W2'), and the sec 63 #7a two-signal DLL probe (the FC-init correction).
  `EV("dll_rst_i",         dut.u_rc.u_phy.pcie_datalink_layer_inst.rst_i)
  `EV("tl_rst_i",          dut.u_rc.u_tl.rst_i)
  `EV("enum_rst_i",        dut.u_rc.u_enum.rst_i)
  `EV("ip_mac_in_detect",  dut.u_pg239.inst.diablo_gt.diablo_gt_phy_wrapper.PHY_PCIE_MAC_IN_DETECT_REG)
  `EV("rxterm_fsm",        dut.u_pg239.inst.diablo_gt.diablo_gt_phy_wrapper.phy_lane[0].receiver_detect_termination_i.ctrl_fsm)
  `EV("rxterm_term",       dut.u_pg239.inst.diablo_gt.diablo_gt_phy_wrapper.phy_lane[0].receiver_detect_termination_i.rxtermination)
  `EV("fci_start_fc",      dut.u_rc.u_phy.pcie_datalink_layer_inst.pcie_flow_ctrl_init_inst.start_flow_control_i)
  `EV("fci_fc1_stored",    dut.u_rc.u_phy.pcie_datalink_layer_inst.pcie_flow_ctrl_init_inst.fc1_values_stored_i)
  `EV("fci_fc2_stored",    dut.u_rc.u_phy.pcie_datalink_layer_inst.pcie_flow_ctrl_init_inst.fc2_values_stored_i)
  `undef EV

  // ---- phy_pclk and the PIPE, per edge -------------------------------------
  longint unsigned n_pclk = 0;
  always @(posedge pclk) begin
    n_pclk <= n_pclk + 1;
    if (n_pclk < 16 || (n_pclk % 4096) == 0) $display("CK|%0t|pclk|%0d", $time, n_pclk);
    if (pipe_time == 0 || $time < pipe_time) begin
      $display("PT|%0t|%h|%b|%b|%b", $time, dut.phy_txdata, dut.phy_txdatak,
               dut.phy_txelecidle, dut.phy_txcompliance);
      $display("PR|%0t|%h|%b|%b|%h|%b|%b", $time, dut.pg_rxdata[15:0], dut.phy_rxdatak,
               dut.phy_rxvalid, dut.phy_rxstatus, dut.phy_rxelecidle, dut.phy_phystatus);
    end
  end

  // ---- the end ----------------------------------------------------------------
  time t_fcinit = 0;
  always @(posedge dut.fc_initialized_o) if (t_fcinit == 0) t_fcinit = $time;
  initial begin
    forever begin
      #1_000_000;  // 1 us
      if (farend == "loop" && t_fcinit != 0 && $time >= t_fcinit + AFTER_FCINIT) begin
        $display("END|%0t|fc_initialized+%0d", $time, AFTER_FCINIT); $finish;
      end
      if ($time >= max_time) begin
        $display("END|%0t|max_time", $time); $finish;
      end
    end
  end

endmodule
