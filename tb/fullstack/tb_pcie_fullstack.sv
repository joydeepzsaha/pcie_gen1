// ===========================================================================
// tb_pcie_fullstack -- our Root Complex against Joy's Endpoint, at the PIPE
// symbol seam, with the 8b/10b codec in the path.
//
// This is the composition SS63 #7 has been working towards. Both sides are real
// RTL from the enumeration engine / Transaction Layer down to a logical PHY;
// the only Python in the datapath is the PIPE sideband both MACs need a PHY to
// answer.
//
//     pcie_rc_top                pipe_codec_bridge           pcie_endpoint_top
//   engine + TL + DLL     32+4 plaintext <-> 20-bit    TL + DLL + LTSSM + PHY
//   + LTSSM + PHY   <---------------------------------->  INTEGRATED_GEN1_PHY=1
//
// == ONE CLOCK (D-7B.1) =====================================================
//
// clk_i drives clk_i, pipe_rx_usr_clk_i and pipe_tx_usr_clk_i on BOTH stacks
// and the bridge between them. That is a decision, not a convenience, and it
// overrules BRIEF_7B.md SS3.1's "the bench must drive the two PIPE clocks
// deliberately".
//
// The reason: PG239 supplies a single PCLK (p.11-12) and the board will drive
// all three pins from it, so a crossing modelled here would be a crossing THE
// HARDWARE WILL NEVER HAVE -- and the alternative is synchroniser RTL written
// for that non-existent topology. The cost is stated rather than hidden: the
// TL<->DLL seam paths join the clk_i setup check, where they previously sat at
// >= +2.766 ns on their own clock. A new population under the check, not a new
// risk. pcie_rc_dl_top already had both layers on one clock; the crossing at
// pcie_phy_top.sv:337 is an artifact of instantiation.
//
// == BOTH ENDS ARE MACs, SO BOTH NEED A PHY TO ANSWER =======================
//
// !! RECEIVER DETECT IS A HANDSHAKE, NOT A LEVEL, AT BOTH ENDS. Each stack
// clears its detect latch on the RISING EDGE of its own phy_txdetectrx and sets
// it only on a SUBSEQUENT phy_phystatus with rxstatus == 3'b011
// (pcie_phy_top.sv:235-240 for the RC, pcie_endpoint_top.sv:428-451 for the
// EP). Holding the status pins high is self-defeating: the MAC's own request
// edge wipes the latch.
//
// BOTH phy_txdetectrx pins are therefore wrapper OUTPUTS. A wrapper that left
// either as () could not train and the failure would give no hint why. The
// in-tree brief warned that "the same mistake is available at both ends of this
// bench" -- this is the line that makes it unavailable.
//
// == WHY NOT REUSE tb_pcie_endpoint_top.sv (SS48.1) =========================
//
// !! THAT BENCH CANNOT BE FLIPPED TO INTEGRATED_GEN1_PHY=1. Its lines 182-192
// tie pipe_rx_usr_clk_i, pipe_tx_usr_clk_i, phy_rx_symbol_i and
// phy_rx_symbol_valid_i to '0. Setting the parameter there would elaborate the
// PHY, NEVER CLOCK IT, and pass vacuously -- a green row proving nothing. Hence
// a new wrapper.
//
// == THE SURFACE IS DELIBERATELY NARROW ======================================
//
// Neither top's full port list is re-exported. cocotb reaches anything else
// hierarchically through dut.u_rc.* / dut.u_ep.*, so only signals a row DRIVES
// or ASSERTS ON are ports here. That keeps the boundary honest about what is
// stimulus and what is observation.
// ===========================================================================

module tb_pcie_fullstack #(
    parameter int MAX_NUM_LANES  = 1,
    parameter int PHY_DATA_WIDTH = 32,
    parameter int AXIS_DATA_WIDTH = 128,
    parameter int AXIS_KEEP_WIDTH = AXIS_DATA_WIDTH / 32,
    parameter int AXIS_USER_WIDTH = 60,
    // Both stacks scaled. The EP half is only settable because SS63 #7b Phase 1
    // promoted it from a hardcoded 1'b0; before that a cross-wired bench trained
    // the Endpoint against real 12 ms timers -- 1.5 M cycles at 8 ns for Detect
    // alone.
    parameter int SIM_FAST_LINK  = 1
) (
    input  logic clk_i,
    input  logic rst_i,
    input  logic en_i,

    // ---- link control ------------------------------------------------------
    input  logic transmit_enable_i,
    input  logic tx_elec_idle,
    input  logic phy_ready_en,

    // ---- the RC's PIPE sideband: its PHY is the bench ----------------------
    output logic                         rc_phy_txdetectrx,
    input  logic [MAX_NUM_LANES-1:0]     rc_phy_rxvalid,
    input  logic [MAX_NUM_LANES-1:0]     rc_phy_phystatus,
    input  logic                         rc_phy_phystatus_rst,
    input  logic [MAX_NUM_LANES-1:0]     rc_phy_rxelecidle,
    input  logic [(MAX_NUM_LANES*3)-1:0] rc_phy_rxstatus,

    // ---- the EP's PIPE sideband: likewise ----------------------------------
    output logic                         ep_phy_txdetectrx,
    input  logic [MAX_NUM_LANES-1:0]     ep_phy_phystatus,
    input  logic                         ep_phy_phystatus_rst,
    input  logic [MAX_NUM_LANES-1:0]     ep_phy_rxelecidle,
    input  logic [(MAX_NUM_LANES*3)-1:0] ep_phy_rxstatus,

    // ---- the seam, exposed for row 3 ---------------------------------------
    // The RC's plaintext characters and the encoded symbols crossing to the EP.
    // Row 3 asserts at BOTH: the 32+4 side witnesses framing and scrambling,
    // the 20-bit side witnesses the codec, which no seam in SS63 #7a could.
    output logic [(MAX_NUM_LANES*PHY_DATA_WIDTH)-1:0] rc_phy_txdata,
    output logic [(4*MAX_NUM_LANES)-1:0]              rc_phy_txdatak,
    output logic [MAX_NUM_LANES-1:0]                  rc_phy_txdata_valid,
    output logic [(MAX_NUM_LANES*20)-1:0]             seam_symbol,
    output logic [MAX_NUM_LANES-1:0]                  seam_symbol_valid,

    // ---- codec health, both directions -------------------------------------
    output logic [MAX_NUM_LANES-1:0] br_enc_illegal_k,
    output logic [MAX_NUM_LANES-1:0] br_dec_code_err,
    output logic [MAX_NUM_LANES-1:0] br_dec_disp_err,
    output logic [MAX_NUM_LANES-1:0] ep_phy_rx_code_error,
    output logic [MAX_NUM_LANES-1:0] ep_phy_rx_disparity_error,
    output logic [MAX_NUM_LANES-1:0] ep_phy_tx_illegal_k,

    // ---- enumeration control and results (rows 2, 6) -----------------------
    input  logic       scan_start_i,
    input  logic [7:0] scan_bus_i,
    input  logic       bar_enable_i,
    input  logic       bridge_enable_i,

    output logic        scan_busy_o,
    output logic        scan_done_o,
    output logic        scan_error_o,
    output logic [3:0]  scan_error_code_o,
    output logic        device_present_o,
    output logic        unsupported_device_o,
    output logic [15:0] vendor_id_o,
    output logic [15:0] device_id_o,
    output logic [7:0]  header_type_o,
    output logic        multifunction_o,
    output logic        enum_done_o,
    output logic        enum_error_o,
    output logic [3:0]  enum_error_code_o,
    output logic [2:0]  bar_count_o,
    output logic [5:0]  bar_valid_o,
    output logic [5:0]  bar_is_64_o,
    output logic [383:0] bar_size_o,
    output logic [383:0] bar_addr_o,

    // ---- the RC's requester surface (row 4) --------------------------------
    input  logic [AXIS_DATA_WIDTH-1:0] s_axis_rq_tdata,
    input  logic [AXIS_KEEP_WIDTH-1:0] s_axis_rq_tkeep,
    input  logic                       s_axis_rq_tvalid,
    input  logic                       s_axis_rq_tlast,
    input  logic [AXIS_USER_WIDTH-1:0] s_axis_rq_tuser,
    output logic                       s_axis_rq_tready,
    output logic                       rq_engine_owns_o,

    // ---- what rows 1, 2 and 5 assert on ------------------------------------
    output logic        rc_link_up_o,
    output logic        rc_fc_initialized_o,
    output logic        rc_ok_to_issue_o,
    output logic        ep_fc_initialized_o,
    output logic        ep_phy_link_up_o,
    output logic [19:0] ep_ltssm_state_o,

    // ---- the EP's far-end view of a received request (row 4) ---------------
    output logic        ep_target_request_valid_o,
    output logic        ep_target_memory_o,
    output logic        ep_target_write_o,
    output logic        ep_target_bar_hit_o,
    output logic [63:0] ep_target_offset_o,
    output logic        ep_target_data_valid_o
);

  // =========================================================================
  // Group C -- the RC's 8 undriven Gen3/4 equalization inputs, tied here for
  // the same reason tb_pcie_rc_top.sv ties them: an undriven boundary input is
  // SS45.2's unconstrained-endpoint class, not a cosmetic gap.
  // =========================================================================
  localparam logic [5:0]                    EQ_FS_TIE    = '0;
  localparam logic [5:0]                    EQ_LF_TIE    = '0;
  localparam logic [(MAX_NUM_LANES*18)-1:0] EQ_COEFF_TIE = '0;
  localparam logic [MAX_NUM_LANES-1:0]      EQ_LANE_TIE  = '0;

  // ---- the seam wires ------------------------------------------------------
  logic [(MAX_NUM_LANES*PHY_DATA_WIDTH)-1:0] rc_rxdata;
  logic [(4*MAX_NUM_LANES)-1:0]              rc_rxdatak;
  logic [MAX_NUM_LANES-1:0]                  rc_rxdata_valid;

  logic [(MAX_NUM_LANES*20)-1:0] ep_rx_symbol;
  logic [MAX_NUM_LANES-1:0]      ep_rx_symbol_valid;
  logic [(MAX_NUM_LANES*20)-1:0] ep_tx_symbol;
  logic [MAX_NUM_LANES-1:0]      ep_tx_symbol_valid;

  assign seam_symbol       = ep_rx_symbol;
  assign seam_symbol_valid = ep_rx_symbol_valid;

  // =========================================================================
  // The bridge. A -> B is the RC's transmit path; B -> A is the EP's.
  // =========================================================================
  pipe_codec_bridge #(
      .MAX_NUM_LANES(MAX_NUM_LANES)
  ) u_bridge (
      .clk_i(clk_i),
      .rst_i(rst_i),

      .a_txdata_i      (rc_phy_txdata),
      .a_txdatak_i     (rc_phy_txdatak),
      .a_txdata_valid_i(rc_phy_txdata_valid),
      .a_rxdata_o      (rc_rxdata),
      .a_rxdatak_o     (rc_rxdatak),
      .a_rxdata_valid_o(rc_rxdata_valid),

      .b_rx_symbol_o      (ep_rx_symbol),
      .b_rx_symbol_valid_o(ep_rx_symbol_valid),
      .b_tx_symbol_i      (ep_tx_symbol),
      .b_tx_symbol_valid_i(ep_tx_symbol_valid),

      .enc_illegal_k_o(br_enc_illegal_k),
      .dec_code_err_o (br_dec_code_err),
      .dec_disp_err_o (br_dec_disp_err)
  );

  // =========================================================================
  // The Root Complex. Identity fixed at 00:00.0 -- a Root Complex's Requester
  // ID is its own BDF (Base 2.1).
  // =========================================================================
  pcie_rc_top #(
      .AXIS_DATA_WIDTH(AXIS_DATA_WIDTH),
      .AXIS_KEEP_WIDTH(AXIS_KEEP_WIDTH),
      .AXIS_USER_WIDTH(AXIS_USER_WIDTH),
      .MAX_NUM_LANES  (MAX_NUM_LANES),
      .PHY_DATA_WIDTH (PHY_DATA_WIDTH),
      .CLK_RATE       (125),
      .IS_ROOT_PORT   (1),
      .LINK_NUM       (0),
      .SIM_FAST_LINK  (SIM_FAST_LINK)
  ) u_rc (
      .clk_i            (clk_i),
      .rst_i            (rst_i),
      .en_i             (en_i),
      .pipe_rx_usr_clk_i(clk_i),
      .pipe_tx_usr_clk_i(clk_i),
      .transmit_enable_i(transmit_enable_i),

      .phy_txdata       (rc_phy_txdata),
      .phy_txdata_valid (rc_phy_txdata_valid),
      .phy_txdatak      (rc_phy_txdatak),
      .phy_txstart_block(),
      .phy_txsync_header(),
      .phy_rxdata       (rc_rxdata),
      .phy_rxdata_valid (rc_rxdata_valid),
      .phy_rxdatak      (rc_rxdatak),
      // Gen3+ only; the EP ties its own equivalents to '0 at :405-406, so
      // carrying them across the bridge would be carrying zeros.
      .phy_rxstart_block('0),
      .phy_rxsync_header('0),

      .phy_txdetectrx  (rc_phy_txdetectrx),
      .phy_txelecidle  (),
      .phy_txcompliance(),
      .phy_rxpolarity  (),
      .phy_powerdown   (),
      .phy_rate        (),

      .phy_rxvalid      (rc_phy_rxvalid),
      .phy_phystatus    (rc_phy_phystatus),
      .phy_phystatus_rst(rc_phy_phystatus_rst),
      .phy_rxelecidle   (rc_phy_rxelecidle),
      .phy_rxstatus     (rc_phy_rxstatus),

      .phy_txmargin(),
      .phy_txswing (),
      .phy_txdeemph(),
      .pipe_width_o(),

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

      .link_up_o       (rc_link_up_o),
      .fc_initialized_o(rc_fc_initialized_o),
      .fc_init_done_o  (),
      .ok_to_issue_o   (rc_ok_to_issue_o),

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

      .scan_busy_o         (scan_busy_o),
      .scan_done_o         (scan_done_o),
      .scan_error_o        (scan_error_o),
      .scan_error_code_o   (scan_error_code_o),
      .err_credit_blocked_o(),
      .device_present_o    (device_present_o),
      .unsupported_device_o(unsupported_device_o),
      .device_bdf_o        (),
      .vendor_id_o         (vendor_id_o),
      .device_id_o         (device_id_o),
      .header_type_o       (header_type_o),
      .multifunction_o     (multifunction_o),
      .bar_busy_o          (),
      .enum_done_o         (enum_done_o),
      .enum_error_o        (enum_error_o),
      .enum_error_code_o   (enum_error_code_o),
      .bar_count_o         (bar_count_o),
      .bar_valid_o         (bar_valid_o),
      .bar_is_64_o         (bar_is_64_o),
      .bar_prefetch_o      (),
      .bar_size_o          (bar_size_o),
      .bar_addr_o          (bar_addr_o),
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

      .s_axis_rq_tdata  (s_axis_rq_tdata),
      .s_axis_rq_tkeep  (s_axis_rq_tkeep),
      .s_axis_rq_tvalid (s_axis_rq_tvalid),
      .s_axis_rq_tlast  (s_axis_rq_tlast),
      .s_axis_rq_tuser  (s_axis_rq_tuser),
      .s_axis_rq_tready (s_axis_rq_tready),
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

  // =========================================================================
  // The Endpoint -- Joy's, unmodified apart from SS63 #7b Phase 1's parameter
  // promotion. INTEGRATED_GEN1_PHY=1 is what puts its LTSSM, logical PHY,
  // scrambler and 8b/10b codec below its Data Link Layer, which is what makes
  // the 20-bit symbol seam exist at all.
  //
  // !! IT IS THE FAR END, NOT THE DUT. If it does not train or does not answer,
  // that is a STOP and a report, not a patch (SS63 #7b's scope fence). The
  // probe for "did not train" is the two-signal one: start_flow_control_i and
  // fc1_values_stored_i inside its own pcie_flow_ctrl_init.
  // =========================================================================
  pcie_endpoint_top #(
      .DATA_WIDTH         (PHY_DATA_WIDTH),
      .KEEP_WIDTH         (PHY_DATA_WIDTH / 8),
      .USER_WIDTH         (3),
      .CONTEXT_WIDTH      (16),
      .MAX_NUM_LANES      (MAX_NUM_LANES),
      .INTEGRATED_GEN1_PHY(1'b1),
      .PHY_CLK_RATE       (125),
      .SIM_FAST_LINK      (SIM_FAST_LINK)
  ) u_ep (
      .clk_i(clk_i),
      .rst_i(rst_i),

      // With INTEGRATED_GEN1_PHY=1 the packet-PHY arm does not elaborate and
      // protocol_link_up comes from the integrated LTSSM (:413), so these two
      // are read by no elaborated logic. Tied to a determinate idle rather than
      // left floating.
      .phy_link_up_i    (1'b0),
      .idle_valid_i     (1'b0),
      .transmit_enable_i(transmit_enable_i),

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

      // ---- the 20-bit symbol seam, to the bridge --------------------------
      .pipe_rx_usr_clk_i    (clk_i),
      .pipe_tx_usr_clk_i    (clk_i),
      .phy_rx_symbol_i      (ep_rx_symbol),
      .phy_rx_symbol_valid_i(ep_rx_symbol_valid),
      .phy_tx_symbol_o      (ep_tx_symbol),
      .phy_tx_symbol_valid_o(ep_tx_symbol_valid),

      .phy_phystatus_i    (ep_phy_phystatus),
      .phy_phystatus_rst_i(ep_phy_phystatus_rst),
      .phy_rxelecidle_i   (ep_phy_rxelecidle),
      .phy_rxstatus_i     (ep_phy_rxstatus),
      .phy_txdetectrx_o   (ep_phy_txdetectrx),
      .phy_txelecidle_o   (),
      .phy_txcompliance_o (),
      .phy_rxpolarity_o   (),
      .phy_powerdown_o    (),
      .phy_rate_o         (),
      .phy_txmargin_o     (),
      .phy_txswing_o      (),
      .phy_txdeemph_o     (),
      .phy_pipe_width_o   (),
      .phy_link_up_o      (ep_phy_link_up_o),
      .ltssm_state_o      (ep_ltssm_state_o),
      .phy_rx_code_error_o     (ep_phy_rx_code_error),
      .phy_rx_disparity_error_o(ep_phy_rx_disparity_error),
      .phy_tx_illegal_k_o      (ep_phy_tx_illegal_k),

      .memory_enable_i      (1'b1),
      .extended_tag_enable_i(1'b0),
      .max_payload_bytes_i  (13'd128),
      .max_read_bytes_i     (13'd512),
      .rcb_128b_i           (1'b0),

      // The Endpoint originates nothing in this bench; every TLP it emits is a
      // Completion its own config space generated.
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

      // Always ready: a row that asserts a request ARRIVED must not also be
      // the thing that stalls it.
      .target_request_valid_o  (ep_target_request_valid_o),
      .target_request_ready_i  (1'b1),
      .target_request_header_o (),
      .target_request_class_o  (),
      .target_memory_o         (ep_target_memory_o),
      .target_config_o         (),
      .target_config_hit_o     (),
      .target_config_type_one_o(),
      .target_config_offset_o  (),
      .target_read_o           (),
      .target_write_o          (ep_target_write_o),
      .target_unsupported_o    (),
      .target_bar_hit_o        (ep_target_bar_hit_o),
      .target_bar_overlap_o    (),
      .target_bar_o            (),
      .target_offset_o         (ep_target_offset_o),
      .target_data_o           (),
      .target_keep_o           (),
      .target_data_valid_o     (ep_target_data_valid_o),
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

      .cfg_bus_number_o     (),
      .cfg_device_number_o  (),
      .cfg_function_number_o(),
      .fc_initialized_o     (ep_fc_initialized_o),
      .fc_update_valid_o    (),
      .fc_ph_o              (), .fc_pd_o  (),
      .fc_nph_o             (), .fc_npd_o (),
      .fc_cplh_o            (), .fc_cpld_o(),
      .malformed_o          (),
      .rx_error_valid_o     (), .rx_error_code_o(),
      .rx_ecrc_error_o      (),
      .tx_error_valid_o     (), .tx_error_code_o(),
      .tx_fc_blocked_o      (), .credit_error_o (),
      .vc_overflow_o        (),
      .unexpected_completion_o(), .completion_error_code_o(),
      .cpl_timeout_valid_o  (), .cpl_timeout_tag_o(),
      .late_cpl_valid_o     (), .late_cpl_tag_o   (),
      .outstanding_o        ()
  );

endmodule
