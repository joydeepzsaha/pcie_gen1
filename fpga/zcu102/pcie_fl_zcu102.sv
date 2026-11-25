// ---------------------------------------------------------------------------
// pcie_fl_zcu102 -- pcie_rc_top and pcie_endpoint_top linked in fabric on the
//                   ZCU102, with their debug cores
//
// Author: Kourosh Ghahramani
// Silicon Systems Research Lab, University of Washington
//
// Purpose
//   Board top for the ZCU102 (xczu9eg-ffvb1156-2-e) that needs no card and
//   no transceiver. The Root Complex (u_rc) and the Endpoint (u_ep,
//   INTEGRATED_GEN1_PHY = 1) face each other through pipe_fabric_link
//   (u_link) on one clock, so link training, Flow Control initialization and
//   enumeration run on the device as they do in the full-stack bench
//   (tb/fullstack). Its only pins are the 125 MHz clock and four LEDs. The
//   stacks are driven and observed on chip:
//     vio_fl   reset release, counter clear, automatic enumeration and
//              the link's fault injection out; link status, the Vendor and
//              Device ID read, the sticky counters and the per-release
//              event times in
//     vio_rc   the Root Complex's runtime controls out and its status in,
//              with pcie_rc_gth_zcu102's vio_pclk probe map
//     ila_fl   both LTSSM states, both Data Link Layer states, the PIPE data
//              and K flags in both directions, the Symbols at the
//              Endpoint, the link's sideband, a timestamp and a store strobe
//
// Interfaces
//   Clock         clk125_p, clk125_n: CLK_125, fixed at 125 MHz.
//   LEDs          led[0] Root Complex link up, led[1] Endpoint link up,
//                 led[2] Flow Control initialized at both ends, led[3]
//                 enumeration done. 1 lights the LED.
//
// Clock and reset
//   clk125 clocks everything: both stacks on all of their clock inputs, the
//   link, the debug cores and the debug hub. No reset input: registers start
//   from their initial values at configuration, with the stacks in reset.
//   vio_fl's run output releases the reset when written 1 and asserts it
//   when written 0; each assertion lasts at least RST_MIN_CYCLES cycles.
//   run starts at 0, so nothing trains until the console writes it. The
//   reset reaches u_rc, u_ep and u_link from three registers that change on
//   the same clock edge. The counters and their clear are not reset by it.
//
// Limitations
//   - The link is pipe_fabric_link's: no elastic buffer, no Electrical Idle
//     from the other end, receiver detection always succeeds.
//   - The fault injection is pipe_codec_bridge's, with its limits: the
//     injector alters one packet from the Root Complex per arming, counted
//     from the rise of inj_en or from the reset release, and the blackouts
//     corrupt every DLLP of a direction, Flow Control DLLPs included.
//   - The four AXIS interfaces of the Root Complex and the Endpoint's
//     command, target and Completion interfaces are idle. The Endpoint
//     originates nothing; every TLP it sends is a Completion from its
//     configuration space.
//   - The Data Link Layer states and the replay, Nak and retrain events are
//     read by hierarchical reference from inside u_rc and u_ep; neither
//     stack has ports for them.
//   - The Root Complex has no 8b/10b decoder. The receive-error counters of
//     its direction count u_link's decoder, which stands for its PHY.
//   - The event times count clk125 cycles from the reset release and
//     saturate at FFFF_FFFFh (34 s).
//
// References
//   UG1182, Table 3-12: ZCU102 Board Clock Sources
//   PCIe Base Spec r2.1, §3.5.2.1
//   PCIe Base Spec r2.1, §4.2.6
//   PCIe CEM Spec r3.0, §2.6.2
// ---------------------------------------------------------------------------
`timescale 1ns/1ps

module pcie_fl_zcu102
  import tlp_pkg::*;
  import pcie_rq_rc_pkg::*;
  import pcie_enum_pkg::*;
#(
    // Passed to both stacks; 1 shortens each LTSSM's 12 ms and 1 ms timeouts
    // and its Polling TS1 count, for simulation only.
    parameter int          SIM_FAST_LINK      = 0,
    // The Root Complex's waits, in clk125 cycles, counted from Flow Control
    // initialization: no Configuration Request for 100 ms, every CRS reissued
    // until 1.0 s, reissues 1 ms apart (PCIe CEM Spec r3.0, §2.6.2). The
    // values of pcie_rc_gth_top.
    parameter int unsigned CFG_HOLD_CYCLES    = 12_500_000,
    parameter int unsigned CRS_WINDOW_CYCLES  = 125_000_000,
    parameter int unsigned CRS_BACKOFF_CYCLES = 125_000,
    // Minimum clk125 cycles of reset for each assertion. The stacks'
    // clock-crossing FIFOs synchronise the reset through three registers.
    parameter int unsigned RST_MIN_CYCLES     = 16,
    // clk125 cycles from the reset release to the automatic scan start.
    parameter int unsigned AUTO_SCAN_CYCLES   = 16
) (
    input  wire       clk125_p,
    input  wire       clk125_n,
    output wire [3:0] led
);

  // The timer values this build uses, stated at elaboration: the top's own
  // parameters appear in the synthesis log nowhere else. Vivado 2023.2
  // reports each as "Synth Info: ..." (Synth 8-6059).
  if (1) begin : g_report
    $info("pcie_fl_zcu102: SIM_FAST_LINK = %0d", SIM_FAST_LINK);
    $info("pcie_fl_zcu102: CFG_HOLD_CYCLES = %0d", CFG_HOLD_CYCLES);
    $info("pcie_fl_zcu102: CRS_WINDOW_CYCLES = %0d", CRS_WINDOW_CYCLES);
    $info("pcie_fl_zcu102: CRS_BACKOFF_CYCLES = %0d", CRS_BACKOFF_CYCLES);
  end

  localparam int TAG_COUNT = 32;
  localparam int CNT_W     = 16;
  localparam int T_W       = 32;
  localparam int TS_W      = 40;
  // The Recovery family of pcie_ltssm_downstream's state code: bits [4:0].
  localparam logic [4:0] LTSSM_RECOVERY = 5'h04;

  // -------------------------------------------------------------------------
  // Clock
  // -------------------------------------------------------------------------
  wire clk125_ibuf, clk125;
  IBUFDS clk125_ibufds (.I(clk125_p), .IB(clk125_n), .O(clk125_ibuf));
  BUFG   clk125_bufg   (.I(clk125_ibuf), .O(clk125));

  // -------------------------------------------------------------------------
  // Reset
  // -------------------------------------------------------------------------
  // vio_run 1 releases the reset. rst_cnt holds it for RST_MIN_CYCLES cycles
  // after run_q rises, so an assertion is never shorter than that, however
  // short the console's 0. rst_rc, rst_ep and rst_link are one value in three
  // registers, one per load; rst is the copy this module's own logic reads.
  wire                                 vio_run, vio_clear, vio_auto_scan;
  logic                                run_q   = 1'b0;
  localparam int                       RST_W   = $clog2(RST_MIN_CYCLES + 1);
  localparam logic [RST_W-1:0]         RST_MIN = RST_W'(RST_MIN_CYCLES);
  logic [RST_W-1:0]                    rst_cnt = '0;
  logic                                rst     = 1'b1;
  (* DONT_TOUCH = "TRUE" *) logic      rst_rc   = 1'b1;
  (* DONT_TOUCH = "TRUE" *) logic      rst_ep   = 1'b1;
  (* DONT_TOUCH = "TRUE" *) logic      rst_link = 1'b1;
  wire                                 rst_release = run_q && (rst_cnt == RST_MIN);

  always_ff @(posedge clk125) begin
    run_q <= vio_run;
    if (!run_q)                         rst_cnt <= '0;
    else if (rst_cnt != RST_MIN)        rst_cnt <= rst_cnt + 1'b1;
    rst      <= ~rst_release;
    rst_rc   <= ~rst_release;
    rst_ep   <= ~rst_release;
    rst_link <= ~rst_release;
  end

  // -------------------------------------------------------------------------
  // The link's wires
  // -------------------------------------------------------------------------
  wire [15:0] rc_txdata, rc_rxdata;
  wire [1:0]  rc_txdatak, rc_rxdatak;
  wire        rc_txdata_valid, rc_rxvalid;
  wire        rc_txdetectrx, rc_phystatus, rc_rxelecidle, rc_phystatus_rst;
  wire [2:0]  rc_rxstatus;
  wire [19:0] ep_rx_symbol, ep_tx_symbol;
  wire        ep_rx_symbol_valid, ep_tx_symbol_valid;
  wire        ep_txdetectrx, ep_phystatus, ep_rxelecidle, ep_phystatus_rst;
  wire [2:0]  ep_rxstatus;
  wire        inj_fired, enc_illegal_k, dec_code_err, dec_disp_err;
  wire [7:0]  inj_end_byte;
  wire [15:0] starve_cnt, starve_ep_cnt;

  // -------------------------------------------------------------------------
  // Fault injection controls
  // -------------------------------------------------------------------------
  // vio_fl's outputs for pipe_codec_bridge's injector and DLLP blackouts,
  // registered once. All start at 0, which leaves the link's data unchanged.
  //   inj_en with inj_mode 0   flips bit inj_bit of the PIPE word inj_off
  //                            beats after the STP of the inj_pkt-th packet
  //                            the Root Complex sends, before encoding: a
  //                            legal Symbol carrying wrong data, which the
  //                            Endpoint's LCRC check rejects
  //   starve_en                corrupts every DLLP from the Endpoint, so the
  //                            Root Complex hears no Ack
  //   starve_ep_en             the same in the other direction
  wire        vio_inj_en, vio_starve_en, vio_starve_ep_en;
  wire [7:0]  vio_inj_pkt, vio_inj_off;
  wire [1:0]  vio_inj_mode;
  wire [3:0]  vio_inj_bit;
  logic       inj_en = 1'b0, starve_en = 1'b0, starve_ep_en = 1'b0;
  logic [7:0] inj_pkt = '0, inj_off = '0;
  logic [1:0] inj_mode = '0;
  logic [3:0] inj_bit = '0;

  always_ff @(posedge clk125) begin
    inj_en       <= vio_inj_en;
    inj_pkt      <= vio_inj_pkt;
    inj_mode     <= vio_inj_mode;
    inj_off      <= vio_inj_off;
    inj_bit      <= vio_inj_bit;
    starve_en    <= vio_starve_en;
    starve_ep_en <= vio_starve_ep_en;
  end

  // -------------------------------------------------------------------------
  // Root Complex
  // -------------------------------------------------------------------------
  // The runtime controls come from vio_rc: en and transmit_enable 1;
  // bar_enable 1, so that enum_done_o follows the scan; scan_bus 00h and
  // bridge_enable 0 (ip_debug_fl.tcl). The scan starts by itself
  // AUTO_SCAN_CYCLES cycles after each reset release while vio_fl's
  // auto_scan is 1, and on each 0-to-1 write of vio_rc's scan_start.
  // pcie_rc_top holds a start until Flow Control is initialized.
  wire                            vio_en, vio_transmit_enable, vio_scan_start;
  wire [7:0]                      vio_scan_bus;
  wire                            vio_bar_enable, vio_bridge_enable;
  logic                           scan_start_q = 1'b0;
  logic                           auto_scan_q  = 1'b0;
  logic                           auto_start   = 1'b0;
  logic [T_W-1:0]                 t_cnt        = '0;
  wire                            scan_start_pulse = (vio_scan_start & ~scan_start_q) | auto_start;

  always_ff @(posedge clk125) begin
    scan_start_q <= vio_scan_start;
    auto_scan_q  <= vio_auto_scan;
    auto_start   <= auto_scan_q && !rst && (t_cnt == T_W'(AUTO_SCAN_CYCLES - 1));
  end

  // status
  wire [20:0]                     rc_ltssm;
  wire                            rc_link_up, rc_fc_initialized, rc_fc_init_done, ok_to_issue;
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

  // The identity and limits are the full-stack bench's: BDF 00:00.0,
  // Max_Payload_Size 128 bytes, Max_Read_Request_Size 512 bytes, RCB 64
  // bytes. The Gen3 equalization inputs and the two inputs pcie_phy_top does
  // not read are tied off.
  pcie_rc_top #(
      .TAG_COUNT         (TAG_COUNT),
      .CRS_BACKOFF_CYCLES(CRS_BACKOFF_CYCLES),
      .CFG_HOLD_CYCLES   (CFG_HOLD_CYCLES),
      .CRS_WINDOW_CYCLES (CRS_WINDOW_CYCLES),
      .CLK_PERIOD_NS     (8),
      .MAX_NUM_LANES     (1),
      .PHY_DATA_WIDTH    (16),
      .IS_ROOT_PORT      (1),
      .LINK_NUM          (0),
      .SIM_FAST_LINK     (SIM_FAST_LINK)
  ) u_rc (
      .clk_i            (clk125),
      .rst_i            (rst_rc),
      .en_i             (vio_en),
      .pipe_rx_usr_clk_i(clk125),
      .pipe_tx_usr_clk_i(clk125),
      .transmit_enable_i(vio_transmit_enable),

      .phy_txdata       (rc_txdata),
      .phy_txdata_valid (rc_txdata_valid),
      .phy_txdatak      (rc_txdatak),
      .phy_txstart_block(),
      .phy_txsync_header(),
      .phy_rxdata       (rc_rxdata),
      .phy_rxdata_valid (1'b0),
      .phy_rxdatak      (rc_rxdatak),
      .phy_rxstart_block(1'b0),
      .phy_rxsync_header(2'b00),

      .phy_txdetectrx  (rc_txdetectrx),
      .phy_txelecidle  (),
      .phy_txcompliance(),
      .phy_rxpolarity  (),
      .phy_powerdown   (),
      .phy_rate        (),

      .phy_rxvalid      (rc_rxvalid),
      .phy_phystatus    (rc_phystatus),
      .phy_phystatus_rst(rc_phystatus_rst),
      .phy_rxelecidle   (rc_rxelecidle),
      .phy_rxstatus     (rc_rxstatus),

      .phy_txmargin(),
      .phy_txswing (),
      .phy_txdeemph(),
      .pipe_width_o(),

      .phy_txeq_ctrl       (),
      .phy_txeq_preset     (),
      .phy_txeq_coeff      (),
      .phy_txeq_fs         (6'd0),
      .phy_txeq_lf         (6'd0),
      .phy_txeq_new_coeff  (18'd0),
      .phy_txeq_done       (1'b0),
      .phy_rxeq_ctrl       (),
      .phy_rxeq_txpreset   (),
      .phy_rxeq_preset_sel (1'b0),
      .phy_rxeq_new_txcoeff(18'd0),
      .phy_rxeq_adapt_done (1'b0),
      .phy_rxeq_done       (1'b0),

      .tx_elec_idle     (1'b0),
      .phy_ready_en     (1'b0),
      .as_mac_in_detect (),
      .as_cdr_hold_req  (),
      .ltssm_debug_state(rc_ltssm),

      .link_up_o       (rc_link_up),
      .fc_initialized_o(rc_fc_initialized),
      .fc_init_done_o  (rc_fc_init_done),
      .ok_to_issue_o   (ok_to_issue),

      .requester_id_i       (16'h0000),
      .completer_id_i       (16'h0000),
      .bus_number_i         (8'h00),
      .device_number_i      (5'h00),
      .function_number_i    (3'h0),
      .memory_enable_i      (1'b1),
      .extended_tag_enable_i(1'b0),
      .max_payload_bytes_i  (13'd128),
      .max_read_bytes_i     (13'd512),
      .rcb_128b_i           (1'b0),
      .cfg_bus_number_o     (cfg_bus),
      .cfg_device_number_o  (cfg_dev),
      .cfg_function_number_o(cfg_fn),

      .scan_start_i   (scan_start_pulse),
      .scan_bus_i     (vio_scan_bus),
      .bar_enable_i   (vio_bar_enable),
      .bridge_enable_i(vio_bridge_enable),

      .scan_busy_o         (scan_busy),
      .scan_done_o         (scan_done),
      .scan_error_o        (scan_error),
      .scan_error_code_o   (scan_error_code),
      .err_credit_blocked_o(err_credit_blocked),
      .device_present_o    (device_present),
      .unsupported_device_o(unsupported_device),
      .device_bdf_o        (device_bdf),
      .vendor_id_o         (vendor_id),
      .device_id_o         (device_id),
      .header_type_o       (header_type),
      .multifunction_o     (multifunction),
      .bar_busy_o          (bar_busy),
      .enum_done_o         (enum_done),
      .enum_error_o        (enum_error),
      .enum_error_code_o   (enum_error_code),
      .bar_count_o         (bar_count),
      .bar_valid_o         (bar_valid),
      .bar_is_64_o         (bar_is_64),
      .bar_prefetch_o      (bar_prefetch),
      .bar_size_o          (bar_size),
      .bar_addr_o          (bar_addr),
      .io_bar_mask_o       (io_bar_mask),

      .bus_done_o              (bus_done),
      .bus_bypassed_o          (bus_bypassed),
      .sec_scan_done_o         (sec_scan_done),
      .sec_device_present_o    (sec_device_present),
      .sec_unsupported_device_o(sec_unsupported_device),
      .sec_device_bdf_o        (sec_device_bdf),
      .sec_vendor_id_o         (sec_vendor_id),
      .sec_device_id_o         (sec_device_id),
      .sec_header_type_o       (sec_header_type),
      .sec_multifunction_o     (sec_multifunction),
      .sec_enum_done_o         (sec_enum_done),
      .sec_bar_count_o         (sec_bar_count),
      .sec_bar_valid_o         (sec_bar_valid),
      .sec_bar_is_64_o         (sec_bar_is_64),
      .sec_bar_prefetch_o      (sec_bar_prefetch),
      .sec_bar_size_o          (sec_bar_size),
      .sec_bar_addr_o          (sec_bar_addr),
      .sec_io_bar_mask_o       (sec_io_bar_mask),

      // Idle: RQ and CC carry no requests or completions, and RC and CQ beats
      // are accepted and dropped. vio_rc observes the four handshake outputs.
      .s_axis_rq_tdata  ('0),
      .s_axis_rq_tkeep  ('0),
      .s_axis_rq_tvalid (1'b0),
      .s_axis_rq_tlast  (1'b0),
      .s_axis_rq_tuser  ('0),
      .s_axis_rq_tready (s_axis_rq_tready),
      .m_axis_rc_tdata  (),
      .m_axis_rc_tkeep  (),
      .m_axis_rc_tvalid (m_axis_rc_tvalid),
      .m_axis_rc_tlast  (),
      .m_axis_rc_tready (1'b1),
      .pcie_rq_tag_o    (pcie_rq_tag),
      .pcie_rq_tag_vld_o(pcie_rq_tag_vld),
      .rq_engine_owns_o (rq_engine_owns),

      .m_axis_cq_tdata    (),
      .m_axis_cq_tkeep    (),
      .m_axis_cq_tvalid   (m_axis_cq_tvalid),
      .m_axis_cq_tlast    (),
      .m_axis_cq_tuser    (),
      .m_axis_cq_tready   (1'b1),
      .s_axis_cc_tdata    ('0),
      .s_axis_cc_tkeep    ('0),
      .s_axis_cc_tvalid   (1'b0),
      .s_axis_cc_tlast    (1'b0),
      .s_axis_cc_tuser    ('0),
      .s_axis_cc_tready   (s_axis_cc_tready),
      .cq_dropped_o       (cq_dropped),
      .cq_error_code_o    (cq_error_code),
      .cq_gearbox_error_o (cq_gearbox_error),
      .cc_protocol_error_o(cc_protocol_error),
      .cc_error_code_o    (cc_error_code),
      .cc_gearbox_error_o (cc_gearbox_error),

      .rq_protocol_error_o       (rq_protocol_error),
      .rq_error_code_o           (rq_error_code),
      .rq_gearbox_error_o        (rq_gearbox_error),
      .rc_unexpected_completion_o(rc_unexpected_completion),
      .rc_completion_error_code_o(rc_completion_error_code),
      .rc_protocol_error_o       (rc_protocol_error),
      .rc_error_code_o           (rc_error_code),
      .rc_gearbox_error_o        (rc_gearbox_error),
      .command_error_valid_o     (command_error_valid),
      .command_error_code_o      (command_error_code),
      .malformed_o               (malformed),
      .rx_error_valid_o          (rx_error_valid),
      .rx_error_code_o           (rx_error_code),
      .rx_ecrc_error_o           (rx_ecrc_error),
      .tx_error_valid_o          (tx_error_valid),
      .tx_error_code_o           (tx_error_code),
      .tx_fc_blocked_o           (tx_fc_blocked),
      .credit_error_o            (credit_error),
      .vc_overflow_o             (vc_overflow),
      .cpl_timeout_valid_o       (cpl_timeout_valid),
      .cpl_timeout_tag_o         (cpl_timeout_tag),
      .late_cpl_valid_o          (late_cpl_valid),
      .late_cpl_tag_o            (late_cpl_tag),
      .outstanding_o             (outstanding)
  );

  // -------------------------------------------------------------------------
  // Link
  // -------------------------------------------------------------------------
  pipe_fabric_link u_link (
      .clk_i(clk125),
      .rst_i(rst_link),

      .a_txdata_i       (rc_txdata),
      .a_txdatak_i      (rc_txdatak),
      .a_txdata_valid_i (rc_txdata_valid),
      .a_rxdata_o       (rc_rxdata),
      .a_rxdatak_o      (rc_rxdatak),
      .a_rxvalid_o      (rc_rxvalid),
      .a_txdetectrx_i   (rc_txdetectrx),
      .a_phystatus_o    (rc_phystatus),
      .a_rxstatus_o     (rc_rxstatus),
      .a_rxelecidle_o   (rc_rxelecidle),
      .a_phystatus_rst_o(rc_phystatus_rst),

      .b_rx_symbol_o      (ep_rx_symbol),
      .b_rx_symbol_valid_o(ep_rx_symbol_valid),
      .b_tx_symbol_i      (ep_tx_symbol),
      .b_tx_symbol_valid_i(ep_tx_symbol_valid),
      .b_txdetectrx_i     (ep_txdetectrx),
      .b_phystatus_o      (ep_phystatus),
      .b_rxstatus_o       (ep_rxstatus),
      .b_rxelecidle_o     (ep_rxelecidle),
      .b_phystatus_rst_o  (ep_phystatus_rst),

      .inj_en_i       (inj_en),
      .inj_pkt_i      (inj_pkt),
      .inj_mode_i     (inj_mode),
      .inj_off_i      (inj_off),
      .inj_bit_i      (inj_bit),
      .inj_fired_o    (inj_fired),
      .inj_end_byte_o (inj_end_byte),
      .starve_en_i    (starve_en),
      .starve_cnt_o   (starve_cnt),
      .starve_ep_en_i (starve_ep_en),
      .starve_ep_cnt_o(starve_ep_cnt),

      .enc_illegal_k_o(enc_illegal_k),
      .dec_code_err_o (dec_code_err),
      .dec_disp_err_o (dec_disp_err)
  );

  // -------------------------------------------------------------------------
  // Endpoint
  // -------------------------------------------------------------------------
  // The full-stack bench's instance: one DW per beat, the integrated Gen1
  // logical PHY, memory enabled, Max_Payload_Size 128 bytes. Its packet PHY
  // ports are idle in this configuration and every interface above the
  // Transaction Layer is tied off, ready where it is a sink.
  wire        ep_link_up, ep_fc_initialized;
  wire [19:0] ep_ltssm;
  wire        ep_rx_code_error, ep_rx_disparity_error, ep_tx_illegal_k;

  pcie_endpoint_top #(
      .DATA_WIDTH         (32),
      .KEEP_WIDTH         (4),
      .USER_WIDTH         (5),
      .CONTEXT_WIDTH      (16),
      .MAX_NUM_LANES      (1),
      .INTEGRATED_GEN1_PHY(1'b1),
      .PHY_CLK_RATE       (125),
      .SIM_FAST_LINK      (SIM_FAST_LINK)
  ) u_ep (
      .clk_i(clk125),
      .rst_i(rst_ep),

      .phy_link_up_i    (1'b0),
      .idle_valid_i     (1'b0),
      .transmit_enable_i(vio_transmit_enable),

      .s_phy_axis_tdata ('0),
      .s_phy_axis_tkeep ('0),
      .s_phy_axis_tvalid(1'b0),
      .s_phy_axis_tlast (1'b0),
      .s_phy_axis_tuser ('0),
      .s_phy_axis_tready(),
      .m_phy_axis_tdata (),
      .m_phy_axis_tkeep (),
      .m_phy_axis_tvalid(),
      .m_phy_axis_tlast (),
      .m_phy_axis_tuser (),
      .m_phy_axis_tready(1'b1),

      .pipe_rx_usr_clk_i    (clk125),
      .pipe_tx_usr_clk_i    (clk125),
      .phy_rx_symbol_i      (ep_rx_symbol),
      .phy_rx_symbol_valid_i(ep_rx_symbol_valid),
      .phy_tx_symbol_o      (ep_tx_symbol),
      .phy_tx_symbol_valid_o(ep_tx_symbol_valid),

      .phy_phystatus_i    (ep_phystatus),
      .phy_phystatus_rst_i(ep_phystatus_rst),
      .phy_rxelecidle_i   (ep_rxelecidle),
      .phy_rxstatus_i     (ep_rxstatus),
      .phy_txdetectrx_o   (ep_txdetectrx),
      .phy_txelecidle_o   (),
      .phy_txcompliance_o (),
      .phy_rxpolarity_o   (),
      .phy_powerdown_o    (),
      .phy_rate_o         (),
      .phy_txmargin_o     (),
      .phy_txswing_o      (),
      .phy_txdeemph_o     (),
      .phy_pipe_width_o   (),
      .phy_link_up_o      (ep_link_up),
      .ltssm_state_o      (ep_ltssm),
      .phy_rx_code_error_o     (ep_rx_code_error),
      .phy_rx_disparity_error_o(ep_rx_disparity_error),
      .phy_tx_illegal_k_o      (ep_tx_illegal_k),

      .memory_enable_i      (1'b1),
      .extended_tag_enable_i(1'b0),
      .max_payload_bytes_i  (13'd128),
      .max_read_bytes_i     (13'd512),
      .rcb_128b_i           (1'b0),

      .command_valid_i       (1'b0),
      .command_ready_o       (),
      .command_i             (tlp_pkg::TLP_CMD_MEM_READ),
      .command_address_i     ('0),
      .command_byte_count_i  ('0),
      .command_tc_i          ('0),
      .command_attr_i        ('0),
      .command_context_i     ('0),
      .command_prefix_valid_i(1'b0),
      .command_prefix_i      ('0),
      .command_ecrc_enable_i (1'b0),
      .command_data_i        ('0),
      .command_keep_i        ('0),
      .command_data_valid_i  (1'b0),
      .command_data_last_i   (1'b0),
      .command_data_ready_o  (),
      .command_error_valid_o (),
      .command_error_code_o  (),

      .target_request_valid_o  (),
      .target_request_ready_i  (1'b1),
      .target_request_header_o (),
      .target_request_class_o  (),
      .target_memory_o         (),
      .target_config_o         (),
      .target_config_hit_o     (),
      .target_config_type_one_o(),
      .target_config_offset_o  (),
      .target_read_o           (),
      .target_write_o          (),
      .target_unsupported_o    (),
      .target_bar_hit_o        (),
      .target_bar_overlap_o    (),
      .target_bar_o            (),
      .target_offset_o         (),
      .target_data_o           (),
      .target_keep_o           (),
      .target_data_valid_o     (),
      .target_data_last_o      (),
      .target_data_ready_i     (1'b1),

      .completion_request_valid_i        (1'b0),
      .completion_request_ready_o        (),
      .completion_request_header_i       ('0),
      .completion_request_status_i       ('0),
      .completion_request_byte_count_i   ('0),
      .completion_request_lower_address_i('0),
      .completion_request_ecrc_enable_i  (1'b0),
      .completion_request_data_i         ('0),
      .completion_request_keep_i         ('0),
      .completion_request_data_valid_i   (1'b0),
      .completion_request_data_last_i    (1'b0),
      .completion_request_data_ready_o   (),

      .received_completion_valid_o     (),
      .received_completion_ready_i     (1'b1),
      .received_completion_header_o    (),
      .received_completion_data_o      (),
      .received_completion_keep_o      (),
      .received_completion_data_valid_o(),
      .received_completion_data_last_o (),
      .received_completion_data_ready_i(1'b1),

      .result_valid_o  (),
      .result_ready_i  (1'b1),
      .result_context_o(),
      .result_status_o (),
      .result_last_o   (),

      .cfg_bus_number_o       (),
      .cfg_device_number_o    (),
      .cfg_function_number_o  (),
      .fc_initialized_o       (ep_fc_initialized),
      .fc_update_valid_o      (),
      .fc_ph_o                (),
      .fc_pd_o                (),
      .fc_nph_o               (),
      .fc_npd_o               (),
      .fc_cplh_o              (),
      .fc_cpld_o              (),
      .malformed_o            (),
      .rx_error_valid_o       (),
      .rx_error_code_o        (),
      .rx_ecrc_error_o        (),
      .tx_error_valid_o       (),
      .tx_error_code_o        (),
      .tx_fc_blocked_o        (),
      .credit_error_o         (),
      .vc_overflow_o          (),
      .unexpected_completion_o(),
      .completion_error_code_o(),
      .cpl_timeout_valid_o    (),
      .cpl_timeout_tag_o      (),
      .late_cpl_valid_o       (),
      .late_cpl_tag_o         (),
      .outstanding_o          ()
  );

  // -------------------------------------------------------------------------
  // Taps inside the stacks
  // -------------------------------------------------------------------------
  // Read by hierarchical reference; nothing is driven. Per stack:
  //   dl_state    pcie_datalink_init's state: 0 DL_Inactive, 1 and 2 before
  //               and during InitFC1, 3 InitFC2, 4 DL_Active
  //   retry_valid one bit per retry slot, set when the slot starts a replay,
  //               on a Nak or on its REPLAY_TIMER, and held until the replay
  //               is taken (PCIe Base Spec r2.1, §3.5.2.1)
  //   retrain_req the Data Link Layer's retrain request, raised when a slot's
  //               REPLAY_NUM rolls over
  //   nak_rx      one cycle for each Nak DLLP received with a good CRC
  //   nak_sched   NAK_SCHEDULED in the receive path: set by a bad TLP,
  //               cleared by the next good one
  wire [2:0] rc_dl_state    = u_rc.u_phy.pcie_datalink_layer_inst.pcie_datalink_init_inst.curr_state;
  wire [2:0] rc_retry_valid = u_rc.u_phy.pcie_datalink_layer_inst.dllp_transmit_inst.retry_valid;
  wire       rc_retrain_req = u_rc.u_phy.pcie_datalink_layer_inst.dllp_transmit_inst.retry_err;
  wire       rc_nak_rx      = u_rc.u_phy.pcie_datalink_layer_inst.seq_num_vld &
                              ~u_rc.u_phy.pcie_datalink_layer_inst.seq_num_acknack;
  wire       rc_nak_sched   = u_rc.u_phy.pcie_datalink_layer_inst.dllp_receive_inst.dllp2tlp_inst.nak_scheduled_r;
  wire [2:0] ep_dl_state    = u_ep.datalink_layer_inst.pcie_datalink_init_inst.curr_state;
  wire [2:0] ep_retry_valid = u_ep.datalink_layer_inst.dllp_transmit_inst.retry_valid;
  wire       ep_retrain_req = u_ep.datalink_layer_inst.dllp_transmit_inst.retry_err;
  wire       ep_nak_rx      = u_ep.datalink_layer_inst.seq_num_vld &
                              ~u_ep.datalink_layer_inst.seq_num_acknack;
  wire       ep_nak_sched   = u_ep.datalink_layer_inst.dllp_receive_inst.dllp2tlp_inst.nak_scheduled_r;
  // u_link's decoder, on the Endpoint-to-Root-Complex direction: one bit per
  // Symbol, meaningful while ep_tx_symbol_valid is high.
  wire       rc_rx_code_error      = ep_tx_symbol_valid & (|u_link.u_codec.gen_lane[0].dec_code_err);
  wire       rc_rx_disparity_error = ep_tx_symbol_valid & (|u_link.u_codec.gen_lane[0].dec_disp_err);

  // -------------------------------------------------------------------------
  // Sticky counters
  // -------------------------------------------------------------------------
  // Each counts events on clk125, saturates at FFFFh and holds through a
  // reset of the stacks; vio_fl's clear zeroes them all. A link-down is a
  // fall of link up while the stacks are out of reset, so a reset the
  // console asks for is not one. The index names are the order of vio_fl's
  // counter probes (ip_debug_fl.tcl).
  typedef enum int {
    C_RELEASE,          // reset releases
    C_RC_LINKUP,        // rises of the Root Complex's link up
    C_EP_LINKUP,        // rises of the Endpoint's link up
    C_RC_LINKDOWN,      // falls of it outside reset
    C_EP_LINKDOWN,
    C_RC_RECOVERY,      // entries to Recovery from another LTSSM state
    C_EP_RECOVERY,
    C_RC_REPLAY,        // replays started, summed over the retry slots
    C_EP_REPLAY,
    C_RC_RETRAIN,       // retrain requests after a REPLAY_NUM rollover
    C_EP_RETRAIN,
    C_RC_NAK_RX,        // Nak DLLPs received
    C_EP_NAK_RX,
    C_RC_NAK_SCHED,     // Naks scheduled for a bad received TLP
    C_EP_NAK_SCHED,
    C_RC_RX_CODE,       // cycles with an invalid code group received
    C_RC_RX_DISP,       // cycles with a disparity error received
    C_EP_RX_CODE,
    C_EP_RX_DISP,
    C_COUNT
  } cnt_e;

  logic [1:0] ev [C_COUNT];
  logic [CNT_W-1:0] cnt [C_COUNT];

  logic       rst_q = 1'b1, clear_q = 1'b0;
  logic       rc_link_up_q = 1'b0, ep_link_up_q = 1'b0;
  logic [4:0] rc_fam_q = '0, ep_fam_q = '0;
  logic [2:0] rc_retry_valid_q = '0, ep_retry_valid_q = '0;
  logic       rc_retrain_req_q = 1'b0, ep_retrain_req_q = 1'b0;
  logic       rc_nak_sched_q = 1'b0, ep_nak_sched_q = 1'b0;

  function automatic logic [1:0] ones3(input logic [2:0] v);
    return {1'b0, v[0]} + {1'b0, v[1]} + {1'b0, v[2]};
  endfunction

  always_comb begin
    ev[C_RELEASE]      = {1'b0, rst_q & ~rst};
    ev[C_RC_LINKUP]    = {1'b0, rc_link_up & ~rc_link_up_q};
    ev[C_EP_LINKUP]    = {1'b0, ep_link_up & ~ep_link_up_q};
    ev[C_RC_LINKDOWN]  = {1'b0, ~rc_link_up & rc_link_up_q & ~rst};
    ev[C_EP_LINKDOWN]  = {1'b0, ~ep_link_up & ep_link_up_q & ~rst};
    ev[C_RC_RECOVERY]  = {1'b0, (rc_ltssm[4:0] == LTSSM_RECOVERY) & (rc_fam_q != LTSSM_RECOVERY)};
    ev[C_EP_RECOVERY]  = {1'b0, (ep_ltssm[4:0] == LTSSM_RECOVERY) & (ep_fam_q != LTSSM_RECOVERY)};
    ev[C_RC_REPLAY]    = ones3(rc_retry_valid & ~rc_retry_valid_q);
    ev[C_EP_REPLAY]    = ones3(ep_retry_valid & ~ep_retry_valid_q);
    ev[C_RC_RETRAIN]   = {1'b0, rc_retrain_req & ~rc_retrain_req_q};
    ev[C_EP_RETRAIN]   = {1'b0, ep_retrain_req & ~ep_retrain_req_q};
    ev[C_RC_NAK_RX]    = {1'b0, rc_nak_rx};
    ev[C_EP_NAK_RX]    = {1'b0, ep_nak_rx};
    ev[C_RC_NAK_SCHED] = {1'b0, rc_nak_sched & ~rc_nak_sched_q};
    ev[C_EP_NAK_SCHED] = {1'b0, ep_nak_sched & ~ep_nak_sched_q};
    ev[C_RC_RX_CODE]   = {1'b0, rc_rx_code_error};
    ev[C_RC_RX_DISP]   = {1'b0, rc_rx_disparity_error};
    ev[C_EP_RX_CODE]   = {1'b0, ep_rx_code_error};
    ev[C_EP_RX_DISP]   = {1'b0, ep_rx_disparity_error};
  end

  always_ff @(posedge clk125) begin
    rst_q            <= rst;
    clear_q          <= vio_clear;
    rc_link_up_q     <= rc_link_up;
    ep_link_up_q     <= ep_link_up;
    rc_fam_q         <= rc_ltssm[4:0];
    ep_fam_q         <= ep_ltssm[4:0];
    rc_retry_valid_q <= rc_retry_valid;
    ep_retry_valid_q <= ep_retry_valid;
    rc_retrain_req_q <= rc_retrain_req;
    ep_retrain_req_q <= ep_retrain_req;
    rc_nak_sched_q   <= rc_nak_sched;
    ep_nak_sched_q   <= ep_nak_sched;
    for (int i = 0; i < C_COUNT; i++) begin
      if (clear_q) begin
        cnt[i] <= '0;
      end else if (cnt[i] > {CNT_W{1'b1}} - CNT_W'(ev[i])) begin
        cnt[i] <= '1;
      end else begin
        cnt[i] <= cnt[i] + CNT_W'(ev[i]);
      end
    end
  end

  initial begin
    for (int i = 0; i < C_COUNT; i++) cnt[i] = '0;
  end

  // -------------------------------------------------------------------------
  // Event times
  // -------------------------------------------------------------------------
  // t_cnt counts clk125 cycles from the reset release. Each t_* holds t_cnt
  // at the first cycle its signal was seen high since that release, and 0
  // until then; all are zeroed while the stacks are in reset.
  typedef enum int {
    T_RC_LINKUP, T_EP_LINKUP, T_RC_FC, T_EP_FC, T_SCAN_DONE, T_ENUM_DONE, T_COUNT
  } t_e;

  logic [T_COUNT-1:0] t_sig;
  logic [T_COUNT-1:0] t_seen = '0;
  logic [T_W-1:0]     t_at [T_COUNT];

  assign t_sig[T_RC_LINKUP] = rc_link_up;
  assign t_sig[T_EP_LINKUP] = ep_link_up;
  assign t_sig[T_RC_FC]     = rc_fc_initialized;
  assign t_sig[T_EP_FC]     = ep_fc_initialized;
  assign t_sig[T_SCAN_DONE] = scan_done;
  assign t_sig[T_ENUM_DONE] = enum_done;

  always_ff @(posedge clk125) begin
    if (rst) begin
      t_cnt  <= '0;
      t_seen <= '0;
      for (int i = 0; i < T_COUNT; i++) t_at[i] <= '0;
    end else begin
      if (~&t_cnt) t_cnt <= t_cnt + 1'b1;
      for (int i = 0; i < T_COUNT; i++) begin
        if (t_sig[i] && !t_seen[i]) begin
          t_seen[i] <= 1'b1;
          t_at[i]   <= t_cnt;
        end
      end
    end
  end

  initial begin
    for (int i = 0; i < T_COUNT; i++) t_at[i] = '0;
  end

  // -------------------------------------------------------------------------
  // LEDs
  // -------------------------------------------------------------------------
  logic [3:0] led_r = '0;
  always_ff @(posedge clk125) begin
    led_r <= {enum_done, rc_fc_initialized & ep_fc_initialized, ep_link_up, rc_link_up};
  end
  assign led = led_r;

  // -------------------------------------------------------------------------
  // ila_fl capture control
  // -------------------------------------------------------------------------
  // ila_fl's 8192 samples cover 65.5 us at one per clk125 edge, and
  // Polling.Active alone lasts at least that long (PCIe Base Spec r2.1,
  // §4.2.6.2.1). To see a whole bring-up, store only the samples in which a
  // watched signal changed: in the Hardware Manager, set the capture mode to
  // BASIC and the capture condition to the ila_store probe == 1. ila_store
  // is 1 when either LTSSM state, either Data Link Layer state, a link up or
  // Flow Control level, a PhyStatus, a scan or enumeration flag or the reset
  // differs from its value one clk125 edge earlier. ila_ts counts clk125
  // edges, 8 ns each, from configuration and times the stored samples; it
  // wraps after 2.4 hours. To see the PIPE data around an event, set the
  // capture mode to ALWAYS and a trigger on the event: the samples are then
  // consecutive clk125 edges around it. ila_ts and ila_watch_q have no
  // reset.
  logic [TS_W-1:0] ila_ts = '0;
  always_ff @(posedge clk125) ila_ts <= ila_ts + 1'b1;

  wire  [3:0] link_levels = {ep_fc_initialized, rc_fc_initialized, ep_link_up, rc_link_up};
  // sideband = {EP RxElecIdle, EP PhyStatus, EP TxDetectRx,
  //             RC RxElecIdle, RC PhyStatus, RC TxDetectRx, rst, run_q}
  wire  [7:0] sideband    = {ep_rxelecidle, ep_phystatus, ep_txdetectrx,
                             rc_rxelecidle, rc_phystatus, rc_txdetectrx, rst, run_q};
  // events = {enum_done, scan_done, scan_start_pulse, an Endpoint replay
  //           start, a Root Complex replay start, EP Nak received, RC Nak
  //           received, inj_fired}
  wire  [7:0] events      = {enum_done, scan_done, scan_start_pulse,
                             |(ep_retry_valid & ~ep_retry_valid_q),
                             |(rc_retry_valid & ~rc_retry_valid_q),
                             ep_nak_rx, rc_nak_rx, inj_fired};

  wire  [55:0] ila_watch   = {rc_ltssm, ep_ltssm, rc_dl_state, ep_dl_state, link_levels,
                              ep_phystatus, rc_phystatus, enum_done, scan_done, rst};
  logic [55:0] ila_watch_q = '0;
  always_ff @(posedge clk125) ila_watch_q <= ila_watch;
  wire         ila_store   = (ila_watch != ila_watch_q);

  // -------------------------------------------------------------------------
  // Debug cores
  // -------------------------------------------------------------------------
  // ip_debug_fl.tcl sets every probe width, and each probe here is connected
  // at its full width.
  ila_fl u_ila_fl (
      .clk    (clk125),
      .probe0 (rc_ltssm),
      .probe1 (ep_ltssm),
      .probe2 (rc_dl_state),
      .probe3 (ep_dl_state),
      .probe4 (link_levels),
      .probe5 (rc_txdata),
      .probe6 (rc_txdatak),
      .probe7 (rc_txdata_valid),
      .probe8 (rc_rxdata),
      .probe9 (rc_rxdatak),
      .probe10(rc_rxvalid),
      .probe11(ep_rx_symbol),
      .probe12(ep_rx_symbol_valid),
      .probe13(ep_tx_symbol),
      .probe14(ep_tx_symbol_valid),
      .probe15(sideband),
      .probe16(events),
      .probe17(ila_ts),
      .probe18(ila_store));

  // fl_status = {dec_disp_err, dec_code_err, enc_illegal_k, inj_fired,
  //              device_present, enum_error, scan_error, enum_done,
  //              scan_done, ep_fc_initialized, rc_fc_initialized,
  //              ep_link_up, rc_link_up, rst, run_q}
  wire [14:0] fl_status = {dec_disp_err, dec_code_err, enc_illegal_k, inj_fired,
                           device_present, enum_error, scan_error, enum_done,
                           scan_done, ep_fc_initialized, rc_fc_initialized,
                           ep_link_up, rc_link_up, rst, run_q};

  vio_fl u_vio_fl (
      .clk       (clk125),
      .probe_in0 (fl_status),
      .probe_in1 (vendor_id),
      .probe_in2 (device_id),
      .probe_in3 (rc_ltssm),
      .probe_in4 (ep_ltssm),
      .probe_in5 ({ep_dl_state, rc_dl_state}),
      .probe_in6 (cnt[C_RELEASE]),
      .probe_in7 (cnt[C_RC_LINKUP]),
      .probe_in8 (cnt[C_EP_LINKUP]),
      .probe_in9 (cnt[C_RC_LINKDOWN]),
      .probe_in10(cnt[C_EP_LINKDOWN]),
      .probe_in11(cnt[C_RC_RECOVERY]),
      .probe_in12(cnt[C_EP_RECOVERY]),
      .probe_in13(cnt[C_RC_REPLAY]),
      .probe_in14(cnt[C_EP_REPLAY]),
      .probe_in15(cnt[C_RC_RETRAIN]),
      .probe_in16(cnt[C_EP_RETRAIN]),
      .probe_in17(cnt[C_RC_NAK_RX]),
      .probe_in18(cnt[C_EP_NAK_RX]),
      .probe_in19(cnt[C_RC_NAK_SCHED]),
      .probe_in20(cnt[C_EP_NAK_SCHED]),
      .probe_in21(cnt[C_RC_RX_CODE]),
      .probe_in22(cnt[C_RC_RX_DISP]),
      .probe_in23(cnt[C_EP_RX_CODE]),
      .probe_in24(cnt[C_EP_RX_DISP]),
      .probe_in25(t_cnt),
      .probe_in26(t_at[T_RC_LINKUP]),
      .probe_in27(t_at[T_EP_LINKUP]),
      .probe_in28(t_at[T_RC_FC]),
      .probe_in29(t_at[T_EP_FC]),
      .probe_in30(t_at[T_SCAN_DONE]),
      .probe_in31(t_at[T_ENUM_DONE]),
      .probe_in32({starve_ep_cnt, starve_cnt}),
      .probe_in33(inj_end_byte),
      .probe_out0(vio_run),
      .probe_out1(vio_clear),
      .probe_out2(vio_auto_scan),
      .probe_out3(vio_inj_en),
      .probe_out4(vio_inj_pkt),
      .probe_out5(vio_inj_mode),
      .probe_out6(vio_inj_off),
      .probe_out7(vio_inj_bit),
      .probe_out8(vio_starve_en),
      .probe_out9(vio_starve_ep_en));

  // vio_rc probe map: pcie_rc_gth_zcu102's vio_pclk map. A VIO input probe is
  // at most 256 bits wide (VIO v3.0 in Vivado 2023.2), so each 384-bit BAR
  // size and address bus takes two.
  vio_rc u_vio_rc (
      .clk       (clk125),
      .probe_in0 ({ok_to_issue, rc_fc_init_done, rc_fc_initialized, rc_link_up}),
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
