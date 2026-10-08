// ---------------------------------------------------------------------------
// pcie_rc_gth_zcu102 -- pcie_rc_gth_top and its debug cores on the ZCU102
//
// Author: Kourosh Ghahramani
// Silicon Systems Research Lab, University of Washington
//
// Purpose
//   Board top for the ZCU102 (xczu9eg-ffvb1156-2-e). Its only pins are the
//   GTH lane, its reference clock, a debug clock and the slot's PERST#:
//   pcie_rc_gth_top has 2,689 port bits, the package 328 PL user I/O (Vivado
//   2023.2 package model). The RC is driven and observed on chip: controls and status
//   through vio_pclk, LTSSM and PIPE activity through ila_pclk, PERST# and a
//   PCLK-gap witness through vio_free and ila_free.
//
// Interfaces
//   Reference clock  sys_clk_p, sys_clk_n: 100 MHz, on MGTREFCLK0 of GTH
//                    bank 130, wired to FMC HPC1 GBTCLK0_M2C; routed to the
//                    lane in bank 129 (pcie_rc_gth_zcu102.xdc).
//   Lane             pci_exp_*: lane 0 on FMC HPC1 DP5, bank 129 channel 1,
//                    the slot's lane 0 on the HTG-FMC-PCIE-RC.
//   Debug clock      clk125_p, clk125_n: CLK_125, fixed at 125 MHz.
//   PERST#           slot_perst_assert: FMC HPC1 LA00_P_CC. 1 asserts the
//                    slot's PERST#: the HTG-FMC-PCIE-RC drives its PERST#
//                    through an NMOS from this pin. It is ~sys_rst_n_r, so
//                    the slot and PG239 leave reset on the same clk125 edge.
//
// Clock and reset
//   pclk, PG239's phy_pclk (125 MHz at Gen1), clocks the RC in u_rc_gth,
//   vio_pclk and ila_pclk. clk125 clocks the power-on reset, PERST#,
//   the PCLK-gap witness, the status synchronisers, vio_free, ila_free and
//   the debug hub. No reset input: registers start from their initial values
//   at configuration, and sys_rst_n_r (PERST#) resets PG239 and the RC and,
//   through slot_perst_assert, the device in the slot.
//
// Limitations
//   The four AXIS interfaces are idle. The power-on reset counts from
//   configuration and does not observe the reference clock.
//
// References
//   PG239, Table 4: Clock and Reset Signals
//   UG1182, Table 3-12: ZCU102 Board Clock Sources
//   UG1182, Table 3-36: ZCU102 GTH Bank 129 Interface Connections
//   UG1182, Table 3-37: ZCU102 GTH Bank 130 Interface Connections
//   UG576, Table 3-31: TX Fabric Clock Output Control Ports
//   UG576, TX Programmable Divider
//   PCIe Base Spec r2.1, §4.2.6.2.1
//   PCIe CEM Spec r3.0, Table 2-4: Power Sequencing and Reset Signal Timings
// ---------------------------------------------------------------------------

module pcie_rc_gth_zcu102
  import tlp_pkg::*;
  import pcie_rq_rc_pkg::*;
  import pcie_enum_pkg::*;
#(
    // Minimum clk125 cycles of PERST# after configuration. 16384 is 131 us,
    // more than the 100 us that PERST# must stay asserted after the reference
    // clock is stable (PCIe CEM Spec r3.0, Table 2-4).
    parameter int unsigned POR_CYCLES    = 16384,
    // Passed to pcie_rc_gth_top; 1 shortens the LTSSM's 12 ms and 1 ms
    // timeouts and its Polling TS1 count, for simulation only.
    parameter int          SIM_FAST_LINK = 0
) (
    input  wire       sys_clk_p,
    input  wire       sys_clk_n,
    input  wire       clk125_p,
    input  wire       clk125_n,
    output wire [0:0] pci_exp_txp,
    output wire [0:0] pci_exp_txn,
    input  wire [0:0] pci_exp_rxp,
    input  wire [0:0] pci_exp_rxn,
    output wire       slot_perst_assert
);

  localparam int          TAG_COUNT  = 32;
  // A gap is GAP_THRESH clk125 cycles (128 ns) without a pclk_div[1] edge.
  // A 125 MHz PCLK gives an edge every 2 cycles, so a gap means PCLK stopped
  // or ran below about 16 MHz.
  localparam int unsigned GAP_THRESH = 16;

  // -------------------------------------------------------------------------
  // Debug clock
  // -------------------------------------------------------------------------
  // clk125 keeps running while PERST# holds PG239 in reset, and PCLK does
  // not. PCLK comes from the GT channel's TXPROGDIVCLK (TXOUTCLKSEL = 101b,
  // TX_PROGCLK_SEL = POSTPI in the PG239 IP that Vivado 2023.2 generates),
  // which is interrupted while the channel or its PLL is reset (UG576, TX
  // Programmable Divider). So the logic that must work through that reset is
  // on clk125: the power-on reset and PERST#, vio_free, which releases
  // PERST#, the PCLK-gap witness, ila_free and the debug hub.
  wire clk125_ibuf, clk125;
  IBUFDS clk125_ibufds (.I(clk125_p), .IB(clk125_n), .O(clk125_ibuf));
  BUFG   clk125_bufg   (.I(clk125_ibuf), .O(clk125));

  // -------------------------------------------------------------------------
  // Power-on reset and PERST#
  // -------------------------------------------------------------------------
  // sys_rst_n_r is PERST# for u_rc_gth: low for at least POR_CYCLES clk125
  // cycles after configuration, and low whenever vio_free's perst_n is 0.
  // perst_n starts at 0 (ip_debug.tcl), so PERST# stays asserted, and the
  // link does not train, until the VIO console writes 1. Until then PCLK,
  // the clock of ila_pclk and vio_pclk, does not run; Vivado 2023.2 warns
  // that a debug core whose clock is not free running may not respond once
  // the design is loaded (create_debug_port). Writing 0 and then 1 repeats
  // the reset, and the PCLK stop with it, for ila_free to record.
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
    // pcie_rc_gth_zcu102.xdc cuts every path from sys_rst_n_r, as Vivado
    // 2023.2's PG239 example design does from its sys_rst_n port. Both loads
    // outside clk125 re-time it: PG239 synchronises phy_rst_n to phy_refclk
    // (rst_n_internal_i in its reset module), and pcie_rc_gth_top passes it to
    // u_rc only through u_rc_rst_sync, an xpm_cdc_async_rst on PCLK.
    sys_rst_n_r <= por_done & vio_perst_n;
  end

  // The slot's PERST#, through the adapter's NMOS: 1 here asserts it. The
  // inverse of sys_rst_n_r, the same register, so the slot is held exactly
  // while PG239 and the RC are. pcie_rc_gth_zcu102.xdc cuts the path to the
  // pin: PERST# is asynchronous at the slot (PCIe CEM Spec r3.0, Table 2-4,
  // note 2).
  assign slot_perst_assert = ~sys_rst_n_r;

  // -------------------------------------------------------------------------
  // Root Complex
  // -------------------------------------------------------------------------
  // The runtime controls come from vio_pclk, whose initial values are the
  // constants tb_pcie_rc_gth drives: en and transmit_enable 1; scan_start,
  // bar_enable and bridge_enable 0; scan_bus 00h (ip_debug.tcl). After PERST#
  // is released the RC is driven as that bench drives it, and enumeration
  // waits for the console to write scan_start. A constant 0 on scan_start_i
  // would hold pcie_enum_scan in S_IDLE and leave the enumeration engine as
  // constant logic for synthesis to remove.
  wire                            pclk;

  // runtime controls (vio_pclk)
  wire                            vio_en, vio_transmit_enable, vio_scan_start;
  wire [7:0]                      vio_scan_bus;
  wire                            vio_bar_enable, vio_bridge_enable;
  // One start per 0-to-1 write of scan_start: a scan_start left at 1 does
  // not start the scan again after the RC is reset.
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
      // The RC's identity and limits, the constants tb_pcie_rc_gth uses: BDF
      // 00:00.0, Max_Payload_Size 128 bytes, Max_Read_Request_Size 512 bytes,
      // RCB 64 bytes.
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
      // Idle: RQ and CC carry no requests or completions, and RC and CQ beats
      // are accepted and dropped. vio_pclk observes the four handshake outputs.
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

  // -------------------------------------------------------------------------
  // PCLK-gap witness
  // -------------------------------------------------------------------------
  // Measures on clk125 each interval in which PCLK stops. pclk_div[1]
  // toggles every two PCLK cycles, a 31.25 MHz square wave at 125 MHz;
  // tick_meta and tick_sync bring it into clk125, and tick_edge marks each
  // toggle, about every 2 clk125 cycles. gap_cnt counts clk125 cycles since
  // the last edge and saturates at FFFFh (524 us). gap_events counts gaps and
  // gap_max holds the longest count; both hold until vio_free's gap_clear.
  // pcie_rc_gth_zcu102.xdc bounds pclk_div[1] into tick_meta with
  // set_max_delay -datapath_only.
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

  // link_up and phy_phystatus_rst cross from pclk, and gt_gtpowergood from
  // PG239's intclk; pcie_rc_gth_zcu102.xdc bounds each with set_max_delay
  // -datapath_only. In the PG239 IP that Vivado 2023.2 generates,
  // gt_gtpowergood is the inverse of txpisopd_r, a register of the reset
  // module's power-on FSM on intclk, not the GT's GTPOWERGOOD output.
  (* ASYNC_REG = "TRUE" *) logic [2:0] fs_meta = '0, fs_sync = '0;
  always_ff @(posedge clk125) begin
    fs_meta <= {link_up, dbg_phystatus_rst, dbg_gtpowergood};
    fs_sync <= fs_meta;
  end
  // free_status = {link_up, phy_phystatus_rst, gt_gtpowergood, sys_rst_n, por_done}
  wire [4:0] free_status = {fs_sync, sys_rst_n_r, por_done};

  // -------------------------------------------------------------------------
  // ila_pclk capture control
  // -------------------------------------------------------------------------
  // ila_pclk's 8192 samples cover 65.5 us at one per PCLK edge. Polling.Active
  // alone lasts at least as long: it sends at least 1024 TS1 Ordered Sets of
  // 16 Symbols, 65.5 us at 2.5 GT/s (PCIe Base Spec r2.1, §4.2.6.2.1). So
  // ila_pclk stores only the samples in which a watched signal changed: in
  // the Hardware Manager, set its capture mode to BASIC and its capture
  // condition to probe13 == 1 (Vivado 2023.2, run_hw_ila). ila_store
  // (probe13) is 1 when the LTSSM state, phystatus, rxstatus or link_up
  // differs from its value one PCLK edge earlier. pclk_ts (probe12) counts
  // PCLK edges, 8 ns each while PCLK runs, and times the stored samples; it
  // stops while PCLK stops, which ila_free's gap witness measures. pclk_ts
  // and ila_watch_q have no reset: pclk_ts starts at configuration and wraps
  // after 2.4 hours of running PCLK.
  localparam int TS_W = 40;
  logic [TS_W-1:0] pclk_ts = '0;                                   // pclk; no reset
  always_ff @(posedge pclk) pclk_ts <= pclk_ts + 1'b1;

  // the watched signals, and their values one PCLK edge earlier
  wire  [25:0] ila_watch   = {ltssm_debug_state, dbg_phystatus, dbg_rxstatus, link_up};
  logic [25:0] ila_watch_q = '0;                                   // pclk; no reset
  always_ff @(posedge pclk) ila_watch_q <= ila_watch;
  wire         ila_store   = (ila_watch != ila_watch_q);

  // -------------------------------------------------------------------------
  // Debug cores
  // -------------------------------------------------------------------------
  // ip_debug.tcl sets every probe width, and each probe here is connected at
  // its full width.
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
      .probe9(dbg_txdetectrx), .probe10(dbg_txelecidle), .probe11(dbg_powerdown),
      .probe12(pclk_ts), .probe13(ila_store));

  // vio_pclk probe map. A VIO input probe is at most 256 bits wide (VIO v3.0
  // in Vivado 2023.2), so each 384-bit BAR size and address bus takes two.
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
