// ===========================================================================
// pcie_rc_gth_zcu102 -- pcie_rc_gth_top on the ZCU102 (xczu9eg-ffvb1156-2-e),
// with its debug cores.  sec 63 #5 (the GTH rung), sub-rung 8-3.
//
// 8-3 synthesises, places and routes this file and simulates it in xsim.  It
// builds no bitstream (D-8.9).  Pin citations are in pcie_rc_gth_zcu102.xdc.
//
//   sys_clk_p/n   100 MHz PCIe refclk: FMC HPC1 GBTCLK0_M2C -> MGTREFCLK0_130
//   pci_exp_*     FMC HPC1 DP0 <-> GTH Quad 130 channel 0 (x1 is the identity)
//   clk125_p/n    CLK_125, the SI5341B's fixed 125 MHz: the debug clock
//
// == WHY THERE IS A BOARD TOP =============================================
//
// pcie_rc_gth_top has about 2,650 port bits and the package has 328 user I/O,
// so its fabric surface cannot be pins.  It is driven and observed on chip:
//
//   * the runtime controls come from vio_pclk.  Their INIT values are
//     tb_pcie_rc_gth.sv's, so the board powers up doing what 8-2 simulated.
//   * the RC identity inputs are constants.  The RC is BDF 00:00.0, and MPS /
//     MRRS / RCB are fixed.  These are the xsim bench's values too.
//   * every non-AXIS status output goes to vio_pclk's probe_in.
//   * the four AXIS surfaces are idle: RQ and CC tvalid 0, RC and CQ tready 1,
//     and only their handshake bits are observed.  No user exists on the board
//     yet.  PAR_8-3.md measures what that trims.
//
// Tying the runtime controls instead would let synthesis remove the
// enumeration engine: scan_start_i = 0 makes every state past S_IDLE
// unreachable.
//
// == CLOCKS =================================================================
//
// pclk (PG239 phy_pclk) is the ONE design clock (D-7B.1).  u_rc, vio_pclk and
// ila_pclk run on it.
//
// clk125 is instrumentation only: the debug hub, vio_free, ila_free, the POR
// and the PCLK-gap witness.  It must be free-running because PCLK is not
// (HANDSHAKE sec 7, 8-2 Phase 1 sec 6).  A debug hub on PCLK would go deaf
// exactly when there is something to see.  A reset VIO on PCLK would assert
// PERST#, stop PCLK, and then have no clock to release it.
//
// Every crossing is one of two kinds:
//   pclk -> clk125  a 2-flop ASYNC_REG synchroniser, bounded in the XDC by
//                   set_max_delay -datapath_only
//   clk125 -> pclk  sys_rst_n only, false-pathed as PG239's own example
//                   design does (the RESET section, below)
//
// == RESET ==================================================================
//
// sys_rst_n (PERST#, active low) is vio_free.perst_n AND por_done, registered
// on clk125.  The POR holds it for POR_CYCLES of clk125 after configuration.
// The default is 16384, i.e. 131 us, against the PCIe CEM's 100 us of refclk
// stability before PERST# is released.  vio_free's INIT is 1, so the link
// trains unattended after configuration.  Pulsing it re-runs the reset, and
// the PCLK gap with it, while ila_free is armed.
//
// Why the false path is safe: u_rc.rst_i is ~sys_rst_n | phy_phystatus_rst.
// When sys_rst_n rises, phy_phystatus_rst is already high: PG239 raises it
// "immediately upon reset" and drops it only once the PHY and GT resets
// complete (p.16).  So the asynchronous edge that could violate recovery is
// always masked.  Assertion is asynchronous by nature, as for any PERST#.
//
// == THE PCLK-GAP WITNESS (HANDSHAKE sec 7) =================================
//
// pclk_div[1] is a 31.25 MHz square wave while PCLK runs.  Synchronised into
// clk125, it produces an edge every 2 cycles.  gap_cnt counts clk125 cycles
// since the last edge, saturating at 0xFFFF (524 us).  A gap is a count past
// GAP_THRESH = 16 cycles (128 ns).  The threshold separates a stopped PCLK from
// the slow one: 8-2 measured 40 ns during reset, whose edges arrive every 10
// cycles.  gap_max and gap_events are sticky, and vio_free.gap_clear clears
// them.
// ===========================================================================

module pcie_rc_gth_zcu102
  import tlp_pkg::*;
  import pcie_rq_rc_pkg::*;
  import pcie_enum_pkg::*;
#(
    parameter int unsigned POR_CYCLES    = 16384,
    parameter int          SIM_FAST_LINK = 0
) (
    input  wire       sys_clk_p,
    input  wire       sys_clk_n,
    input  wire       clk125_p,
    input  wire       clk125_n,
    output wire [0:0] pci_exp_txp,
    output wire [0:0] pci_exp_txn,
    input  wire [0:0] pci_exp_rxp,
    input  wire [0:0] pci_exp_rxn
);

  localparam int          TAG_COUNT  = 32;
  localparam int unsigned GAP_THRESH = 16;

  // ---- clk125 --------------------------------------------------------------
  wire clk125_ibuf, clk125;
  IBUFDS clk125_ibufds (.I(clk125_p), .IB(clk125_n), .O(clk125_ibuf));
  BUFG   clk125_bufg   (.I(clk125_ibuf), .O(clk125));

  // ---- POR and PERST# (clk125) --------------------------------------------
  logic [$clog2(POR_CYCLES+1)-1:0] por_cnt  = '0;
  logic                            por_done = 1'b0;
  logic                            sys_rst_n_r = 1'b0;
  wire                             vio_perst_n;
  wire                             vio_gap_clear;

  always_ff @(posedge clk125) begin
    if (!por_done) begin
      por_cnt  <= por_cnt + 1'b1;
      por_done <= (por_cnt == POR_CYCLES - 1);
    end
    sys_rst_n_r <= por_done & vio_perst_n;
  end

  // ---- the RC --------------------------------------------------------------
  wire                            pclk;

  // runtime controls (vio_pclk)
  wire                            vio_en, vio_transmit_enable, vio_scan_start;
  wire [7:0]                      vio_scan_bus;
  wire                            vio_bar_enable, vio_bridge_enable;
  logic                           scan_start_q = 1'b0;
  always_ff @(posedge pclk) scan_start_q <= vio_scan_start;
  wire                            scan_start_pulse = vio_scan_start & ~scan_start_q;

  // status
  wire [20:0]                     ltssm_debug_state;
  wire                            link_up, fc_initialized, fc_init_done, ok_to_issue;
  wire [7:0]                      cfg_bus;
  wire [4:0]                      cfg_dev;
  wire [2:0]                      cfg_fn;
  wire                            scan_busy, scan_done, scan_error, err_credit_blocked;
  enum_error_e                    scan_error_code, enum_error_code;
  wire                            device_present, unsupported_device, multifunction;
  wire [15:0]                     device_bdf, vendor_id, device_id;
  wire [7:0]                      header_type;
  wire                            bar_busy, enum_done, enum_error;
  wire [3:0]                      bar_count;
  wire [BAR_SLOTS-1:0]            bar_valid, bar_is_64, bar_prefetch, io_bar_mask;
  wire [BAR_SLOTS*64-1:0]         bar_size, bar_addr;
  wire                            bus_done, bus_bypassed, sec_scan_done, sec_device_present;
  wire                            sec_unsupported_device, sec_multifunction, sec_enum_done;
  wire [15:0]                     sec_device_bdf, sec_vendor_id, sec_device_id;
  wire [7:0]                      sec_header_type;
  wire [3:0]                      sec_bar_count;
  wire [BAR_SLOTS-1:0]            sec_bar_valid, sec_bar_is_64, sec_bar_prefetch, sec_io_bar_mask;
  wire [BAR_SLOTS*64-1:0]         sec_bar_size, sec_bar_addr;
  wire                            s_axis_rq_tready, m_axis_rc_tvalid, m_axis_cq_tvalid, s_axis_cc_tready;
  wire [7:0]                      pcie_rq_tag;
  wire                            pcie_rq_tag_vld, rq_engine_owns;
  wire                            cq_dropped, cq_gearbox_error, cc_protocol_error, cc_gearbox_error;
  wire [3:0]                      cq_error_code, cc_error_code;
  wire                            rq_protocol_error, rq_gearbox_error;
  rq_error_e                      rq_error_code;
  wire                            rc_unexpected_completion, rc_protocol_error, rc_gearbox_error;
  tlp_error_e                     rc_completion_error_code;
  rc_error_e                      rc_error_code;
  wire                            command_error_valid, malformed, rx_error_valid, rx_ecrc_error;
  tlp_error_e                     command_error_code, rx_error_code, tx_error_code;
  wire                            tx_error_valid, tx_fc_blocked, credit_error, vc_overflow;
  wire                            cpl_timeout_valid, late_cpl_valid;
  wire [7:0]                      cpl_timeout_tag, late_cpl_tag;
  wire [$clog2(TAG_COUNT+1)-1:0]  outstanding;

  // the PIPE taps (pcie_rc_gth_top's dbg_* ports)
  wire                            dbg_phystatus, dbg_phystatus_rst, dbg_rxvalid, dbg_rxelecidle;
  wire [2:0]                      dbg_rxstatus;
  wire                            dbg_as_mac_in_detect, dbg_txdetectrx, dbg_txelecidle;
  wire [1:0]                      dbg_powerdown;
  wire                            dbg_gtpowergood;

  pcie_rc_gth_top #(
      .TAG_COUNT    (TAG_COUNT),
      .SIM_FAST_LINK(SIM_FAST_LINK)
  ) u_rc_gth (
      .sys_clk_p(sys_clk_p), .sys_clk_n(sys_clk_n), .sys_rst_n(sys_rst_n_r),
      .pci_exp_txp(pci_exp_txp), .pci_exp_txn(pci_exp_txn),
      .pci_exp_rxp(pci_exp_rxp), .pci_exp_rxn(pci_exp_rxn),
      .pclk_o(pclk),
      .en_i(vio_en), .transmit_enable_i(vio_transmit_enable),
      .ltssm_debug_state(ltssm_debug_state),
      .dbg_phy_phystatus_o(dbg_phystatus), .dbg_phy_phystatus_rst_o(dbg_phystatus_rst),
      .dbg_phy_rxstatus_o(dbg_rxstatus), .dbg_phy_rxvalid_o(dbg_rxvalid),
      .dbg_phy_rxelecidle_o(dbg_rxelecidle), .dbg_as_mac_in_detect_o(dbg_as_mac_in_detect),
      .dbg_phy_txdetectrx_o(dbg_txdetectrx), .dbg_phy_txelecidle_o(dbg_txelecidle),
      .dbg_phy_powerdown_o(dbg_powerdown), .dbg_gt_gtpowergood_o(dbg_gtpowergood),
      .link_up_o(link_up), .fc_initialized_o(fc_initialized), .fc_init_done_o(fc_init_done),
      .ok_to_issue_o(ok_to_issue),
      // the RC's identity: tb_pcie_rc_gth.sv's values
      .requester_id_i(16'h0000), .completer_id_i(16'h0000), .bus_number_i(8'h00),
      .device_number_i(5'h00), .function_number_i(3'h0), .memory_enable_i(1'b1),
      .extended_tag_enable_i(1'b0), .max_payload_bytes_i(13'd128),
      .max_read_bytes_i(13'd512), .rcb_128b_i(1'b0),
      .cfg_bus_number_o(cfg_bus), .cfg_device_number_o(cfg_dev), .cfg_function_number_o(cfg_fn),
      .scan_start_i(scan_start_pulse), .scan_bus_i(vio_scan_bus),
      .bar_enable_i(vio_bar_enable), .bridge_enable_i(vio_bridge_enable),
      .scan_busy_o(scan_busy), .scan_done_o(scan_done), .scan_error_o(scan_error),
      .scan_error_code_o(scan_error_code), .err_credit_blocked_o(err_credit_blocked),
      .device_present_o(device_present), .unsupported_device_o(unsupported_device),
      .device_bdf_o(device_bdf), .vendor_id_o(vendor_id), .device_id_o(device_id),
      .header_type_o(header_type), .multifunction_o(multifunction), .bar_busy_o(bar_busy),
      .enum_done_o(enum_done), .enum_error_o(enum_error), .enum_error_code_o(enum_error_code),
      .bar_count_o(bar_count), .bar_valid_o(bar_valid), .bar_is_64_o(bar_is_64),
      .bar_prefetch_o(bar_prefetch), .bar_size_o(bar_size), .bar_addr_o(bar_addr),
      .io_bar_mask_o(io_bar_mask),
      .bus_done_o(bus_done), .bus_bypassed_o(bus_bypassed), .sec_scan_done_o(sec_scan_done),
      .sec_device_present_o(sec_device_present), .sec_unsupported_device_o(sec_unsupported_device),
      .sec_device_bdf_o(sec_device_bdf), .sec_vendor_id_o(sec_vendor_id),
      .sec_device_id_o(sec_device_id), .sec_header_type_o(sec_header_type),
      .sec_multifunction_o(sec_multifunction), .sec_enum_done_o(sec_enum_done),
      .sec_bar_count_o(sec_bar_count), .sec_bar_valid_o(sec_bar_valid),
      .sec_bar_is_64_o(sec_bar_is_64), .sec_bar_prefetch_o(sec_bar_prefetch),
      .sec_bar_size_o(sec_bar_size), .sec_bar_addr_o(sec_bar_addr),
      .sec_io_bar_mask_o(sec_io_bar_mask),
      // the four AXIS surfaces, idle (header: no user on the board yet)
      .s_axis_rq_tdata('0), .s_axis_rq_tkeep('0), .s_axis_rq_tvalid(1'b0),
      .s_axis_rq_tlast(1'b0), .s_axis_rq_tuser('0), .s_axis_rq_tready(s_axis_rq_tready),
      .m_axis_rc_tdata(), .m_axis_rc_tkeep(), .m_axis_rc_tvalid(m_axis_rc_tvalid),
      .m_axis_rc_tlast(), .m_axis_rc_tready(1'b1),
      .pcie_rq_tag_o(pcie_rq_tag), .pcie_rq_tag_vld_o(pcie_rq_tag_vld),
      .rq_engine_owns_o(rq_engine_owns),
      .m_axis_cq_tdata(), .m_axis_cq_tkeep(), .m_axis_cq_tvalid(m_axis_cq_tvalid),
      .m_axis_cq_tlast(), .m_axis_cq_tuser(), .m_axis_cq_tready(1'b1),
      .s_axis_cc_tdata('0), .s_axis_cc_tkeep('0), .s_axis_cc_tvalid(1'b0),
      .s_axis_cc_tlast(1'b0), .s_axis_cc_tuser('0), .s_axis_cc_tready(s_axis_cc_tready),
      .cq_dropped_o(cq_dropped), .cq_error_code_o(cq_error_code),
      .cq_gearbox_error_o(cq_gearbox_error), .cc_protocol_error_o(cc_protocol_error),
      .cc_error_code_o(cc_error_code), .cc_gearbox_error_o(cc_gearbox_error),
      .rq_protocol_error_o(rq_protocol_error), .rq_error_code_o(rq_error_code),
      .rq_gearbox_error_o(rq_gearbox_error),
      .rc_unexpected_completion_o(rc_unexpected_completion),
      .rc_completion_error_code_o(rc_completion_error_code),
      .rc_protocol_error_o(rc_protocol_error), .rc_error_code_o(rc_error_code),
      .rc_gearbox_error_o(rc_gearbox_error),
      .command_error_valid_o(command_error_valid), .command_error_code_o(command_error_code),
      .malformed_o(malformed), .rx_error_valid_o(rx_error_valid), .rx_error_code_o(rx_error_code),
      .rx_ecrc_error_o(rx_ecrc_error), .tx_error_valid_o(tx_error_valid),
      .tx_error_code_o(tx_error_code), .tx_fc_blocked_o(tx_fc_blocked),
      .credit_error_o(credit_error), .vc_overflow_o(vc_overflow),
      .cpl_timeout_valid_o(cpl_timeout_valid), .cpl_timeout_tag_o(cpl_timeout_tag),
      .late_cpl_valid_o(late_cpl_valid), .late_cpl_tag_o(late_cpl_tag),
      .outstanding_o(outstanding)
  );

  // ---- the PCLK-gap witness -------------------------------------------------
  logic [1:0]  pclk_div = 2'b00;                                   // pclk; no reset
  always_ff @(posedge pclk) pclk_div <= pclk_div + 1'b1;

  (* ASYNC_REG = "TRUE" *) logic tick_meta = 1'b0, tick_sync = 1'b0;
  logic        tick_q      = 1'b0;
  logic [15:0] gap_cnt     = '0;
  logic [15:0] gap_max     = '0;
  logic [15:0] gap_events  = '0;
  wire         tick_edge   = tick_sync ^ tick_q;

  always_ff @(posedge clk125) begin
    tick_meta <= pclk_div[1];
    tick_sync <= tick_meta;
    tick_q    <= tick_sync;
    if (tick_edge)            gap_cnt <= '0;
    else if (~&gap_cnt)       gap_cnt <= gap_cnt + 1'b1;
    if (vio_gap_clear) begin
      gap_max    <= '0;
      gap_events <= '0;
    end else begin
      if (gap_cnt > gap_max)                         gap_max    <= gap_cnt;
      if (gap_cnt == GAP_THRESH && ~&gap_events)     gap_events <= gap_events + 1'b1;
    end
  end

  // status into clk125: 2-flop synchronisers (gt_gtpowergood is an async GT output)
  (* ASYNC_REG = "TRUE" *) logic [2:0] fs_meta = '0, fs_sync = '0;
  always_ff @(posedge clk125) begin
    fs_meta <= {link_up, dbg_phystatus_rst, dbg_gtpowergood};
    fs_sync <= fs_meta;
  end
  // free_status = {link_up, phy_phystatus_rst, gt_gtpowergood, sys_rst_n, por_done}
  wire [4:0] free_status = {fs_sync, sys_rst_n_r, por_done};

  // ---- debug cores ------------------------------------------------------------
  ila_free u_ila_free (
      .clk(clk125), .probe0(gap_cnt), .probe1(tick_sync), .probe2(free_status));

  vio_free u_vio_free (
      .clk(clk125), .probe_in0(gap_max), .probe_in1(gap_events), .probe_in2(free_status),
      .probe_out0(vio_perst_n), .probe_out1(vio_gap_clear));

  ila_pclk u_ila_pclk (
      .clk(pclk),
      .probe0(ltssm_debug_state), .probe1(link_up), .probe2(fc_initialized),
      .probe3(dbg_phystatus), .probe4(dbg_phystatus_rst), .probe5(dbg_rxstatus),
      .probe6(dbg_rxvalid), .probe7(dbg_rxelecidle), .probe8(dbg_as_mac_in_detect),
      .probe9(dbg_txdetectrx), .probe10(dbg_txelecidle), .probe11(dbg_powerdown));

  // vio_pclk probe map (widths are ip_debug.tcl's; every probe is full width)
  vio_pclk u_vio_pclk (
      .clk       (pclk),
      .probe_in0 ({ok_to_issue, fc_init_done, fc_initialized, link_up}),
      .probe_in1 ({cfg_bus, cfg_dev, cfg_fn}),
      .probe_in2 ({enum_error, enum_done, bar_busy, multifunction, unsupported_device,
                   device_present, err_credit_blocked, scan_error, scan_done, scan_busy}),
      .probe_in3 (scan_error_code),
      .probe_in4 (enum_error_code),
      .probe_in5 (device_bdf),
      .probe_in6 (vendor_id),
      .probe_in7 (device_id),
      .probe_in8 (header_type),
      .probe_in9 (bar_count),
      .probe_in10({io_bar_mask, bar_prefetch, bar_is_64, bar_valid}),
      .probe_in11(bar_size[255:0]),
      .probe_in12(bar_size[BAR_SLOTS*64-1:256]),
      .probe_in13(bar_addr[255:0]),
      .probe_in14(bar_addr[BAR_SLOTS*64-1:256]),
      .probe_in15({sec_enum_done, sec_multifunction, sec_unsupported_device,
                   sec_device_present, sec_scan_done, bus_bypassed, bus_done}),
      .probe_in16(sec_device_bdf),
      .probe_in17(sec_vendor_id),
      .probe_in18(sec_device_id),
      .probe_in19(sec_header_type),
      .probe_in20(sec_bar_count),
      .probe_in21({sec_io_bar_mask, sec_bar_prefetch, sec_bar_is_64, sec_bar_valid}),
      .probe_in22(sec_bar_size[255:0]),
      .probe_in23(sec_bar_size[BAR_SLOTS*64-1:256]),
      .probe_in24(sec_bar_addr[255:0]),
      .probe_in25(sec_bar_addr[BAR_SLOTS*64-1:256]),
      .probe_in26({rq_engine_owns, pcie_rq_tag_vld, pcie_rq_tag}),
      .probe_in27({s_axis_cc_tready, m_axis_cq_tvalid, m_axis_rc_tvalid, s_axis_rq_tready}),
      .probe_in28({late_cpl_valid, cpl_timeout_valid, vc_overflow, credit_error, tx_fc_blocked,
                   tx_error_valid, rx_ecrc_error, rx_error_valid, malformed, command_error_valid,
                   rc_gearbox_error, rc_protocol_error, rc_unexpected_completion,
                   rq_gearbox_error, rq_protocol_error, cc_gearbox_error, cc_protocol_error,
                   cq_gearbox_error, cq_dropped}),
      .probe_in29({tx_error_code, rx_error_code, command_error_code, rc_error_code,
                   rc_completion_error_code, rq_error_code, cc_error_code, cq_error_code}),
      .probe_in30({outstanding, late_cpl_tag, cpl_timeout_tag}),
      .probe_out0(vio_en),
      .probe_out1(vio_transmit_enable),
      .probe_out2(vio_scan_start),
      .probe_out3(vio_scan_bus),
      .probe_out4(vio_bar_enable),
      .probe_out5(vio_bridge_enable));

endmodule
