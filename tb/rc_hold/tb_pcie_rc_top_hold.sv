// ---------------------------------------------------------------------------
// tb_pcie_rc_top_hold -- pcie_rc_top with a post-reset hold, over a PIPE
// loopback
//
// Author: Kourosh Ghahramani
// Silicon Systems Research Lab, University of Washington
//
// Purpose
//   tb_pcie_rc_top with one difference: the hold (§63 #22) is set to
//   CFG_HOLD_CYCLES. Row (e) times the engine's first request into the
//   Transaction Layer against fc_init_done_o, which proves which signal
//   pcie_rc_top connects to the engine's link_active_i: with link_up_o the
//   hold would run out during FC initialization, and with the pin open
//   nothing would ever be issued.
//
// Interfaces
//   As tb_pcie_rc_top, plus bench_cfg_hold_cycles. Everything else the row
//   reads is reached hierarchically through u_rc.
//
// Clock and reset
//   clk_i drives every clock of u_rc, as in tb_pcie_rc_top.
//
// Limitations
//   The loopback far end cannot answer a Configuration Request, so the row
//   ends at the first request.
//
// References
//   PCIe Base Spec r2.1, §6.6.1
//   PCIe Base Spec r3.0, §6.7.3.3
// ---------------------------------------------------------------------------

module tb_pcie_rc_top_hold #(
    parameter int MAX_NUM_LANES  = 1,
    parameter int PHY_DATA_WIDTH = 16,   // §63 #5 8-1: per-lane PIPE width
    parameter int SIM_FAST_LINK  = 1     // scaled LTSSM timers -- see below
) (
    input  logic clk_i,
    input  logic rst_i,
    input  logic en_i,

    // ---- link control (group B2) ------------------------------------------
    input  logic transmit_enable_i,
    input  logic tx_elec_idle,
    input  logic phy_ready_en,

    // ---- the PIPE seam: TX observed, RX driven (group A) -------------------
    output logic [(MAX_NUM_LANES*PHY_DATA_WIDTH)-1:0] phy_txdata,
    output logic [MAX_NUM_LANES-1:0]                  phy_txdata_valid,
    output logic [(MAX_NUM_LANES*PHY_DATA_WIDTH/8)-1:0] phy_txdatak,
    output logic [MAX_NUM_LANES-1:0]                  phy_txstart_block,
    output logic [(2*MAX_NUM_LANES)-1:0]              phy_txsync_header,
    input  logic [(MAX_NUM_LANES*PHY_DATA_WIDTH)-1:0] phy_rxdata,
    input  logic [MAX_NUM_LANES-1:0]                  phy_rxdata_valid,
    input  logic [(MAX_NUM_LANES*PHY_DATA_WIDTH/8)-1:0] phy_rxdatak,
    input  logic [MAX_NUM_LANES-1:0]                  phy_rxstart_block,
    input  logic [(2*MAX_NUM_LANES)-1:0]              phy_rxsync_header,

    // ---- PIPE command the far end must ANSWER (group A) --------------------
    // phy_txdetectrx is exposed because receiver detection is a HANDSHAKE, not
    // a level: pcie_phy_top.sv:235-240 clears its detect latch on this signal's
    // rising edge and re-arms it only when phy_phystatus is seen with
    // phy_rxstatus == 3'b011. A far end that drives status without watching
    // this pin can never complete Detect.
    output logic       phy_txdetectrx,
    output logic [1:0] phy_powerdown,

    // ---- PHY status the far end must present (group A) ---------------------
    input  logic [MAX_NUM_LANES-1:0]     phy_rxvalid,
    input  logic [MAX_NUM_LANES-1:0]     phy_phystatus,
    input  logic                         phy_phystatus_rst,
    input  logic [MAX_NUM_LANES-1:0]     phy_rxelecidle,
    input  logic [(MAX_NUM_LANES*3)-1:0] phy_rxstatus,

    // ---- enumeration control (group B6) ------------------------------------
    input  logic       scan_start_i,
    input  logic [7:0] scan_bus_i,
    input  logic       bar_enable_i,
    input  logic       bridge_enable_i,

    // ---- the three rows care about most ------------------------------------
    output logic link_up_o,
    output logic fc_initialized_o,      // UNFILTERED -- row 1's subject
    output logic ok_to_issue_o,
    output logic enum_done_o,
    output logic rq_engine_owns_o
);

  // The bench's hold, 1,000 cycles; row (e) reads it from
  // bench_cfg_hold_cycles.
  localparam int unsigned CFG_HOLD_CYCLES = 1000;
  logic [31:0] bench_cfg_hold_cycles;
  assign bench_cfg_hold_cycles = CFG_HOLD_CYCLES;

  // =========================================================================
  // Group C -- the 8 undriven Gen3/4 equalization inputs, tied here.
  // =========================================================================
  localparam logic [5:0]                      EQ_FS_TIE      = '0;
  localparam logic [5:0]                      EQ_LF_TIE      = '0;
  localparam logic [(MAX_NUM_LANES*18)-1:0]   EQ_COEFF_TIE   = '0;
  localparam logic [MAX_NUM_LANES-1:0]        EQ_LANE_TIE    = '0;

  // =========================================================================
  // The RC identity is fixed at 00:00.0 for the whole run (Base 2.1: a Root
  // Complex's Requester ID is its own BDF). These are constants rather than
  // wrapper ports because no row varies them.
  // =========================================================================
  pcie_rc_top #(
      .MAX_NUM_LANES (MAX_NUM_LANES),
      .PHY_DATA_WIDTH(PHY_DATA_WIDTH),
      .CLK_PERIOD_NS (8),
      .IS_ROOT_PORT  (1),
      .LINK_NUM      (0),
      .CFG_HOLD_CYCLES(CFG_HOLD_CYCLES),
      // !! SIM_FAST_LINK=1 scales TwelveMsTimeOut / OneMsTimeOut and drops
      // MinTS1sPolling 1024 -> 24 (pcie_ltssm_downstream.sv:111-121). Without
      // it Detect.Quiet alone is 12 ms = 1.5 M cycles at 8 ns. Note the
      // asymmetry recorded in FINDINGS_PHASE0.md F3: pcie_endpoint_top
      // HARDCODES this to 0, which is why the RC<->EP full-stack bench is a
      // later rung and this one uses a Python far end.
      .SIM_FAST_LINK (SIM_FAST_LINK)
  ) u_rc (
      .clk_i            (clk_i),
      .rst_i            (rst_i),
      .en_i             (en_i),
      .pipe_rx_usr_clk_i(clk_i),
      .pipe_tx_usr_clk_i(clk_i),
      .transmit_enable_i(transmit_enable_i),

      .phy_txdata       (phy_txdata),
      .phy_txdata_valid (phy_txdata_valid),
      .phy_txdatak      (phy_txdatak),
      .phy_txstart_block(phy_txstart_block),
      .phy_txsync_header(phy_txsync_header),
      .phy_rxdata       (phy_rxdata),
      .phy_rxdata_valid (phy_rxdata_valid),
      .phy_rxdatak      (phy_rxdatak),
      .phy_rxstart_block(phy_rxstart_block),
      .phy_rxsync_header(phy_rxsync_header),

      .phy_txdetectrx  (phy_txdetectrx),
      .phy_txelecidle  (),
      .phy_txcompliance(),
      .phy_rxpolarity  (),
      .phy_powerdown   (phy_powerdown),
      .phy_rate        (),

      .phy_rxvalid      (phy_rxvalid),
      .phy_phystatus    (phy_phystatus),
      .phy_phystatus_rst(phy_phystatus_rst),
      .phy_rxelecidle   (phy_rxelecidle),
      .phy_rxstatus     (phy_rxstatus),

      .phy_txmargin(),
      .phy_txswing (),
      .phy_txdeemph(),
      .pipe_width_o(),

      // ---- group C: the five outputs float, the eight inputs are TIED ------
      .phy_txeq_ctrl       (),
      .phy_txeq_preset     (),
      .phy_txeq_coeff      (),
      .phy_txeq_fs         (EQ_FS_TIE),
      .phy_txeq_lf         (EQ_LF_TIE),
      .phy_txeq_new_coeff  (EQ_COEFF_TIE),
      .phy_txeq_done       (EQ_LANE_TIE),
      .phy_rxeq_ctrl       (),
      .phy_rxeq_txpreset   (),
      .phy_rxeq_preset_sel (EQ_LANE_TIE),
      .phy_rxeq_new_txcoeff(EQ_COEFF_TIE),
      .phy_rxeq_adapt_done (EQ_LANE_TIE),
      .phy_rxeq_done       (EQ_LANE_TIE),

      .tx_elec_idle     (tx_elec_idle),
      .phy_ready_en     (phy_ready_en),
      .as_mac_in_detect (),
      .as_cdr_hold_req  (),
      .ltssm_debug_state(),

      .link_up_o       (link_up_o),
      .fc_initialized_o(fc_initialized_o),
      .fc_init_done_o  (),
      .ok_to_issue_o   (ok_to_issue_o),

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
      .cfg_bus_number_o     (),
      .cfg_device_number_o  (),
      .cfg_function_number_o(),

      .scan_start_i   (scan_start_i),
      .scan_bus_i     (scan_bus_i),
      .bar_enable_i   (bar_enable_i),
      .bridge_enable_i(bridge_enable_i),

      .scan_busy_o         (), .scan_done_o      (),
      .scan_error_o        (), .scan_error_code_o(),
      .err_credit_blocked_o(), .device_present_o (),
      .unsupported_device_o(), .device_bdf_o     (),
      .vendor_id_o         (), .device_id_o      (),
      .header_type_o       (), .multifunction_o  (),
      .bar_busy_o          (), .enum_done_o      (enum_done_o),
      .enum_error_o        (), .enum_error_code_o(),
      .bar_count_o         (), .bar_valid_o      (),
      .bar_is_64_o         (), .bar_prefetch_o   (),
      .bar_size_o          (), .bar_addr_o       (),
      .io_bar_mask_o       (),

      .bus_done_o              (), .bus_bypassed_o          (),
      .sec_scan_done_o         (), .sec_device_present_o    (),
      .sec_unsupported_device_o(), .sec_device_bdf_o        (),
      .sec_vendor_id_o         (), .sec_device_id_o         (),
      .sec_header_type_o       (), .sec_multifunction_o     (),
      .sec_enum_done_o         (), .sec_bar_count_o         (),
      .sec_bar_valid_o         (), .sec_bar_is_64_o         (),
      .sec_bar_prefetch_o      (), .sec_bar_size_o          (),
      .sec_bar_addr_o          (), .sec_io_bar_mask_o       (),

      // ---- the requester surface: idle until a row drives it ---------------
      .s_axis_rq_tdata  ('0),
      .s_axis_rq_tkeep  ('0),
      .s_axis_rq_tvalid (1'b0),
      .s_axis_rq_tlast  (1'b0),
      .s_axis_rq_tuser  ('0),
      .s_axis_rq_tready (),
      .m_axis_rc_tdata  (),
      .m_axis_rc_tkeep  (),
      .m_axis_rc_tvalid (),
      .m_axis_rc_tlast  (),
      .m_axis_rc_tready (1'b1),
      .pcie_rq_tag_o    (),
      .pcie_rq_tag_vld_o(),
      .rq_engine_owns_o (rq_engine_owns_o),

      .m_axis_cq_tdata (), .m_axis_cq_tkeep (),
      .m_axis_cq_tvalid(), .m_axis_cq_tlast (),
      .m_axis_cq_tuser (), .m_axis_cq_tready(1'b1),
      .s_axis_cc_tdata ('0), .s_axis_cc_tkeep ('0),
      .s_axis_cc_tvalid(1'b0), .s_axis_cc_tlast (1'b0),
      .s_axis_cc_tuser ('0), .s_axis_cc_tready(),
      .cq_dropped_o    (), .cq_error_code_o (),
      .cq_gearbox_error_o(),
      .cc_protocol_error_o(), .cc_error_code_o(),
      .cc_gearbox_error_o(),

      .rq_protocol_error_o(), .rq_error_code_o    (),
      .rq_gearbox_error_o (), .rc_unexpected_completion_o(),
      .rc_completion_error_code_o(), .rc_protocol_error_o(),
      .rc_error_code_o    (), .rc_gearbox_error_o (),
      .command_error_valid_o(), .command_error_code_o(),
      .malformed_o     (), .rx_error_valid_o(),
      .rx_error_code_o (), .rx_ecrc_error_o (),
      .tx_error_valid_o(), .tx_error_code_o (),
      .tx_fc_blocked_o (), .credit_error_o  (),
      .vc_overflow_o   (), .cpl_timeout_valid_o(),
      .cpl_timeout_tag_o  (), .late_cpl_valid_o   (),
      .late_cpl_tag_o     (), .outstanding_o      ()
  );

endmodule
