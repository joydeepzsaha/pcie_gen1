// ===========================================================================
// tb_pcie_rc_gth_pg213 -- xsim bench for pcie_rc_gth_top: the RC through
// AMD's PCIe PHY IP (PG239) against AMD's PCIe Endpoint (PG213), cross-wired
// on the serial pins.  sec 63 #27.
//
// xsim, not Verilator: both GT models and the Endpoint's hard block are
// encrypted.  This bench decides nothing.  It prints raw timestamped lines and
// xsim_gate/xsim_gate_rows.py turns them into the pg213 and pg213_enum rows.
//
// == WHY ======================================================================
//
// A conformance row against foreign silicon.  The far end in tb/lr1 shares no
// code with the RC, but it shares its authors' reading of the spec; a vendor's
// Endpoint does not.  Chat's ruling B (link robustness part 1): PG213 becomes
// a conformance row of the xsim gate once enumeration completes against it.
//
// == THE FAR END ================================================================
//
// pcie4_uscale_plus v1.3 (PG213), a Gen1 x1 Endpoint, generated on
// xczu7ev-ffvc1156-2-e by xsim_gate/ip_sim_export_pg213.tcl, inside the IP's
// own example design top (xilinx_pcie4_uscale_ep, with its PIO application),
// unmodified.  Its configuration space is the hard block's: Vendor ID 10EEh,
// Device ID 9011h, BAR0 32-bit 128 KB, no other BAR (the IP's defaults; the
// export prints them).  The part differs from the RC's ZU9EG, which has no
// PCIe hard block; in simulation both GTs are the same GTHE4 model.
//
// One 100 MHz reference clock drives both sides, as one clock generator does
// on the HTG card (sec 63 #23), and both see the same PERST# (sys_rst_n).
//
// == SETTINGS ===================================================================
//
// RC: SIM_FAST_LINK = 0, the board's value (link robustness ruling D), and
// the post-reset hold and CRS window at the loop bench's short bench values
// (sec 63 #22).  PG213: no PL_SIM_FAST_LINK_TRAINING defparam, so it trains
// as the silicon does.  scan_start pulses 1 us after fc_initialized rises;
// pcie_rc_top latches it and waits its own hold before the first request.
//
// !! Elaborate this bench with xelab --debug all.  It delivers TLPs to the RC,
// and xsim 2023.2 drops the value-change events of the second instance of
// tlp_validator in tlp_layer (sec 63 #25, a tool trap; HANDSHAKE sec 5).
// Without the flag the RC classifies PG213's first Completion as unsupported
// and enumeration never completes.
//
// == LINES (every time in ps) ===================================================
//
//   CFG|0|<key>|<value>                      the run's settings
//   EV|t|<name>|<hex>                        a value change, in its time step
//   SCAN|t|<present>|<error>|<code>|<vid>|<did>|<header type>
//                                            when scan_done first rises
//   ENUM|t|<done>|<error>|<code>|<bar count> when enum_done or enum_error
//                                            first rises
//   BAR|t|<slot>|<valid>|<is64>|<prefetch>|<io>|<size>|<address>
//                                            each BAR slot, with ENUM
//   FS|t|<cfg_function_status[3:0]>          PG213's Function 0 Command bits
//                                            (I/O, memory, bus master, INTx
//                                            disable), with ENUM and at the end
//   END|t|<reason>                           enum_done+<ps>, enum_error+<ps>
//                                            or max_time
//
// +MAX_US=<n>  (default 600) end the run at n us.
// ===========================================================================
`timescale 1ps / 1ps

module tb_pcie_rc_gth_pg213;

  import pcie_enum_pkg::*;

  localparam int  REFCLK_HALF_PS = 5000;          // 100 MHz
  localparam int  PERST_CYCLES   = 500;           // PG239's own board.v
  localparam time SCAN_DELAY     = 1_000_000;     // 1 us after fc_initialized
  localparam time AFTER_ENUM     = 5_000_000;     // run 5 us past the enumeration's end

  reg        sys_clk_p = 1'b0;
  wire       sys_clk_n = ~sys_clk_p;
  reg        sys_rst_n = 1'b0;
  wire [0:0] rc_txp, rc_txn, ep_txp, ep_txn;
  wire       pclk;
  reg        scan_start = 1'b0;

  always #(REFCLK_HALF_PS) sys_clk_p = ~sys_clk_p;

  wire                   scan_done, scan_err, dev_present, enum_done, enum_err;
  enum_error_e           scan_code, enum_code;
  wire [15:0]            vid, did;
  wire [7:0]             hdr_type;
  wire [3:0]             bar_count;
  wire [BAR_SLOTS-1:0]   bar_valid, bar_is_64, bar_prefetch, io_bar_mask;
  wire [BAR_SLOTS*64-1:0] bar_size, bar_addr;

  pcie_rc_gth_top #(.SIM_FAST_LINK(0), .CFG_HOLD_CYCLES(2000), .CRS_WINDOW_CYCLES(20000), .CRS_BACKOFF_CYCLES(200)) dut (
      .sys_clk_p(sys_clk_p), .sys_clk_n(sys_clk_n), .sys_rst_n(sys_rst_n),
      .pci_exp_txp(rc_txp), .pci_exp_txn(rc_txn),
      .pci_exp_rxp(ep_txp), .pci_exp_rxn(ep_txn),
      .pclk_o(pclk),
      .en_i(1'b1), .transmit_enable_i(1'b1),
      .ltssm_debug_state(),
      .link_up_o(), .fc_initialized_o(), .fc_init_done_o(), .ok_to_issue_o(),
      .requester_id_i(16'h0000), .completer_id_i(16'h0000), .bus_number_i(8'h00),
      .device_number_i(5'h00), .function_number_i(3'h0), .memory_enable_i(1'b1),
      .extended_tag_enable_i(1'b0), .max_payload_bytes_i(13'd128),
      .max_read_bytes_i(13'd512), .rcb_128b_i(1'b0),
      .cfg_bus_number_o(), .cfg_device_number_o(), .cfg_function_number_o(),
      .scan_start_i(scan_start), .scan_bus_i(8'h00), .bar_enable_i(1'b1), .bridge_enable_i(1'b0),
      .scan_busy_o(), .scan_done_o(scan_done), .scan_error_o(scan_err), .scan_error_code_o(scan_code),
      .err_credit_blocked_o(), .device_present_o(dev_present), .unsupported_device_o(),
      .device_bdf_o(), .vendor_id_o(vid), .device_id_o(did), .header_type_o(hdr_type),
      .multifunction_o(), .bar_busy_o(), .enum_done_o(enum_done), .enum_error_o(enum_err),
      .enum_error_code_o(enum_code), .bar_count_o(bar_count), .bar_valid_o(bar_valid), .bar_is_64_o(bar_is_64),
      .bar_prefetch_o(bar_prefetch), .bar_size_o(bar_size), .bar_addr_o(bar_addr), .io_bar_mask_o(io_bar_mask),
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

  // ---- the far end: PG213's example design, unmodified --------------------------
  xilinx_pcie4_uscale_ep ep (
      .sys_clk_p(sys_clk_p), .sys_clk_n(sys_clk_n), .sys_rst_n(sys_rst_n),
      .pci_exp_txp(ep_txp), .pci_exp_txn(ep_txn),
      .pci_exp_rxp(rc_txp), .pci_exp_rxn(rc_txn));

  // ---- plusargs --------------------------------------------------------------
  time max_time = 600_000_000;                    // 600 us
  int  max_us;
  initial begin
    $timeformat(-12, 0, "", 0);                   // %t in ps
    if ($value$plusargs("MAX_US=%d", max_us)) max_time = max_us * 64'd1_000_000;
    $display("CFG|0|MAX_TIME_PS|%0d", max_time);
    $display("CFG|0|REFCLK_HALF_PS|%0d", REFCLK_HALF_PS);
    $display("CFG|0|PERST_CYCLES|%0d", PERST_CYCLES);
    $display("CFG|0|RC_SIM_FAST_LINK|%0d", dut.SIM_FAST_LINK);
    $display("CFG|0|RC_CFG_HOLD_CYCLES|%0d", dut.CFG_HOLD_CYCLES);
  end

  // ---- PERST# ---------------------------------------------------------------
  initial begin
    repeat (PERST_CYCLES) @(posedge sys_clk_p);
    sys_rst_n = 1'b1;
  end

  // ---- scan_start: one pulse, 1 us after fc_initialized first rises ----------
  initial begin
    wait (dut.fc_initialized_o === 1'b1);
    #(SCAN_DELAY);
    scan_start = 1'b1;
    $display("EV|%0t|scan_start|1", $time);
    @(posedge pclk); @(posedge pclk);
    scan_start = 1'b0;
    $display("EV|%0t|scan_start|0", $time);
  end

  // ---- raw events -------------------------------------------------------------
  `define EV(NAME, SIG) always @(SIG) $display("EV|%0t|%s|%h", $time, NAME, SIG);
  `EV("sys_rst_n",          sys_rst_n)
  `EV("rc_rst_i",           dut.u_rc.rst_i)
  `EV("ltssm_state",        dut.ltssm_debug_state)
  `EV("link_up",            dut.link_up_o)
  `EV("fc_initialized",     dut.fc_initialized_o)
  `EV("phy_rxpolarity",     dut.phy_rxpolarity)
  `EV("scan_done",          scan_done)
  `EV("enum_done",          enum_done)
  `EV("enum_error",         enum_err)
  `EV("ep_ltssm",           ep.cfg_ltssm_state)
  `EV("ep_user_lnk_up",     ep.user_lnk_up)
  `EV("ep_phy_link_status", ep.pg213_ep_i.cfg_phy_link_status)   // unconnected in the example top
  `EV("ep_negotiated_width", ep.cfg_negotiated_width)
  `EV("ep_current_speed",   ep.cfg_current_speed)
  `EV("ep_function_status", ep.cfg_function_status[3:0])
  `undef EV

  // ---- the enumeration's results ---------------------------------------------
  bit scan_seen = 0, enum_seen = 0;
  always @(posedge scan_done) if (!scan_seen) begin
    scan_seen = 1;
    $display("SCAN|%0t|%b|%b|%0d|%h|%h|%h", $time, dev_present, scan_err, scan_code, vid, did, hdr_type);
  end
  task automatic report_enum(input string reason);
    enum_seen = 1;
    $display("ENUM|%0t|%b|%b|%0d|%0d", $time, enum_done, enum_err, enum_code, bar_count);
    for (int s = 0; s < BAR_SLOTS; s++)
      $display("BAR|%0t|%0d|%b|%b|%b|%b|%h|%h", $time, s, bar_valid[s], bar_is_64[s], bar_prefetch[s],
               io_bar_mask[s], bar_size[s*64 +: 64], bar_addr[s*64 +: 64]);
    $display("FS|%0t|%h", $time, ep.cfg_function_status[3:0]);
    #(AFTER_ENUM);
    $display("FS|%0t|%h", $time, ep.cfg_function_status[3:0]);
    $display("END|%0t|%s+%0d", $time, reason, AFTER_ENUM);
    $finish;
  endtask
  always @(posedge enum_done) if (!enum_seen) report_enum("enum_done");
  always @(posedge enum_err)  if (!enum_seen) report_enum("enum_error");

  // ---- the end ----------------------------------------------------------------
  initial begin
    #1;                                           // after the plusargs are read
    #(max_time - 1);
    $display("END|%0t|max_time", $time);
    $finish;
  end

endmodule
