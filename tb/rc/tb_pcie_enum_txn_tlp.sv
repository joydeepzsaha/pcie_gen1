// ---------------------------------------------------------------------------
// tb_pcie_enum_txn_tlp -- pcie_cfg_txn in front of a REAL pcie_rq_rc_top.
// Commit 2b-1, integration half.
//
//   cmd_* -> pcie_cfg_txn -> s_axis_rq -> pcie_rq_if -> tlp_layer -> TX DLLP
//   RX DLLP -> tlp_layer -> pcie_rc_if -> m_axis_rc -> pcie_cfg_txn -> rsp_*
//
// The standalone target (verilate_enum_txn) owns the primitive's behaviour
// against a socket the bench invents. This one owns what only the real stack
// can answer: that the descriptor the primitive builds becomes the TLP the spec
// says it should, that the tag it latches is the tag the tracker allocated and
// put in the header, that the completion timeout it reacts to is the tracker's
// own, and that the whole thing still works when credit is scarce instead of
// saturated.
//
// Neither target subsumes the other and both are required. The blind spots run
// in BOTH directions -- 2a-ii mutation A survived every integration test, and
// 2a-iii M4 survived every standalone one.
//
// ! FLOW CONTROL. tlp_layer emits ZERO TLPs and reports NO error until
// link_up_i, transmit_enable_i, fc_initialized_i and one fc_update_valid_i
// pulse with non-zero credits are ALL present (tlp_layer.sv:249,
// tlp_credit_manager.sv:53-54, 66-83). Every "N packets" assertion in this
// bench would otherwise be vacuously satisfied by silence -- regression RC1.
// All four are driven from Python so the RC1 negative control can drop one.
//
// !! AN ADVERTISEMENT OF 00h/000h AT FC INIT MEANS INFINITE, NOT ZERO
// (Base 2.1 SS2.6.1 p.138, footnote 33 p.137; tlp_credit_manager.sv:106-120).
// Credit STARVATION therefore requires a small FINITE advertisement with no
// replenishment, never a zero one. See test_pcie_enum_txn_tlp.py, P-NPD-INF
// and P-NPD1-STALL.
//
// TAG_COUNT is 8 rather than the module's 32: the primitive is single
// outstanding by construction, so nothing here needs a large tag space, and a
// small one makes tag reuse visible.
//
// CRS parameters are 3 retries / 8 cycles rather than the shipped 16 / 64, so
// the CRS tests cost tens of cycles instead of a thousand. P-CRS-BUDGET still
// holds with room to spare: 3*8 = 24, against CPL_TIMEOUT_CYCLES = 4096, which
// is left at the SHIPPED default deliberately -- the timeout test must exercise
// the value Commit 2b will actually run with.
// ---------------------------------------------------------------------------
`timescale 1ns/1ps
module tb_pcie_enum_txn_tlp
  import tlp_pkg::*;
  import pcie_rq_rc_pkg::*;
  import pcie_enum_pkg::*;
;

  localparam int AXIS_DATA_WIDTH = 128;
  localparam int AXIS_KEEP_WIDTH = AXIS_DATA_WIDTH / 32;
  localparam int AXIS_USER_WIDTH = 60;
  localparam int TL_DATA_WIDTH   = 32;
  localparam int TL_KEEP_WIDTH   = TL_DATA_WIDTH / 8;
  localparam int TL_USER_WIDTH   = 3;
  localparam int CONTEXT_WIDTH   = 16;
  localparam int TAG_COUNT       = 8;

  localparam int unsigned CRS_RETRY_MAX       = 3;
  localparam int unsigned CRS_BACKOFF_CYCLES  = 8;
  localparam int unsigned CPL_TIMEOUT_CYCLES  = 32'd4096;

  localparam int CRS_CNT_W = $clog2(CRS_RETRY_MAX + 1);

  logic clk_i = 0;
  logic rst_i;

  // ---- link state and flow control ----------------------------------------
  logic        link_up_i;
  logic        transmit_enable_i;
  logic        fc_initialized_i;
  logic        fc_update_valid_i;
  logic [7:0]  fc_ph_i,  fc_nph_i,  fc_cplh_i;
  logic [11:0] fc_pd_i,  fc_npd_i,  fc_cpld_i;

  // ---- identity and negotiated limits -------------------------------------
  logic [15:0] requester_id_i;
  logic [15:0] completer_id_i;
  logic [7:0]  bus_number_i;
  logic [4:0]  device_number_i;
  logic [2:0]  function_number_i;
  logic        memory_enable_i;
  logic        extended_tag_enable_i;
  logic [12:0] max_payload_bytes_i;
  logic [12:0] max_read_bytes_i;
  logic        rcb_128b_i;

  // ---- transaction primitive command/response port ------------------------
  logic        cmd_valid_i;
  logic        cmd_ready_o;
  logic        cmd_write_i;
  logic        cmd_type1_i;
  logic [15:0] cmd_bdf_i;
  logic [5:0]  cmd_reg_num_i;
  logic [3:0]  cmd_ext_reg_i;
  logic [3:0]  cmd_first_be_i;
  logic [31:0] cmd_wdata_i;

  logic         rsp_valid_o;
  logic         rsp_ready_i;
  txn_outcome_e rsp_outcome;
  logic [2:0]   rsp_outcome_o;
  assign rsp_outcome_o = 3'(rsp_outcome);
  logic [31:0]  rsp_rdata_o;
  logic [2:0]   rsp_status_raw_o;
  logic [CRS_CNT_W-1:0] crs_retries_o;

  // ---- the socket, internal: primitive <-> pcie_rq_rc_top -----------------
  logic [AXIS_DATA_WIDTH-1:0] s_axis_rq_tdata;
  logic [AXIS_KEEP_WIDTH-1:0] s_axis_rq_tkeep;
  logic                       s_axis_rq_tvalid;
  logic                       s_axis_rq_tlast;
  logic [AXIS_USER_WIDTH-1:0] s_axis_rq_tuser;
  logic                       s_axis_rq_tready;

  logic [7:0] pcie_rq_tag_o;
  logic       pcie_rq_tag_vld_o;

  logic [AXIS_DATA_WIDTH-1:0] m_axis_rc_tdata;
  logic [AXIS_KEEP_WIDTH-1:0] m_axis_rc_tkeep;
  logic                       m_axis_rc_tvalid;
  logic                       m_axis_rc_tlast;
  logic                       m_axis_rc_tready;

  // ---- DLL-facing streams: the bench plays the Data Link Layer ------------
  logic [TL_DATA_WIDTH-1:0] s_dllp_axis_tdata;
  logic [TL_KEEP_WIDTH-1:0] s_dllp_axis_tkeep;
  logic                     s_dllp_axis_tvalid;
  logic                     s_dllp_axis_tlast;
  logic [TL_USER_WIDTH-1:0] s_dllp_axis_tuser;
  logic                     s_dllp_axis_tready;

  logic [TL_DATA_WIDTH-1:0] m_dllp_axis_tdata;
  logic [TL_KEEP_WIDTH-1:0] m_dllp_axis_tkeep;
  logic                     m_dllp_axis_tvalid;
  logic                     m_dllp_axis_tlast;
  logic [TL_USER_WIDTH-1:0] m_dllp_axis_tuser;
  logic                     m_dllp_axis_tready;

  // ---- error surface, enums flattened for cocotb --------------------------
  logic       rq_protocol_error_o;
  rq_error_e  rq_error_code;
  logic [3:0] rq_error_code_o;
  assign rq_error_code_o = 4'(rq_error_code);
  logic       rq_gearbox_error_o;

  logic       rc_unexpected_completion_o;
  tlp_error_e rc_completion_error_code;
  logic [4:0] rc_completion_error_code_o;
  assign rc_completion_error_code_o = 5'(rc_completion_error_code);

  logic       rc_protocol_error_o;
  rc_error_e  rc_error_code;
  logic [3:0] rc_error_code_o;
  assign rc_error_code_o = 4'(rc_error_code);
  logic       rc_gearbox_error_o;

  logic       command_error_valid_o;
  tlp_error_e command_error_code;
  logic [4:0] command_error_code_o;
  assign command_error_code_o = 5'(command_error_code);

  logic       malformed_o;
  logic       rx_error_valid_o;
  tlp_error_e rx_error_code;
  logic [4:0] rx_error_code_o;
  assign rx_error_code_o = 5'(rx_error_code);
  logic       rx_ecrc_error_o;

  logic       tx_error_valid_o;
  tlp_error_e tx_error_code;
  logic [4:0] tx_error_code_o;
  assign tx_error_code_o = 5'(tx_error_code);

  logic tx_fc_blocked_o;
  logic credit_error_o;
  logic vc_overflow_o;

  logic       cpl_timeout_valid_o;
  logic [7:0] cpl_timeout_tag_o;
  logic       late_cpl_valid_o;
  logic [7:0] late_cpl_tag_o;
  logic [$clog2(TAG_COUNT+1)-1:0] outstanding_o;

  // -------------------------------------------------------------------------
  // The DUT under test: the transaction primitive.
  // -------------------------------------------------------------------------
  pcie_cfg_txn #(
      .AXIS_DATA_WIDTH   (AXIS_DATA_WIDTH),
      .AXIS_KEEP_WIDTH   (AXIS_KEEP_WIDTH),
      .AXIS_USER_WIDTH   (AXIS_USER_WIDTH),
      .CRS_RETRY_MAX     (CRS_RETRY_MAX),
      .CRS_BACKOFF_CYCLES(CRS_BACKOFF_CYCLES),
      .CPL_TIMEOUT_CYCLES(CPL_TIMEOUT_CYCLES)
  ) u_txn (
      .clk_i(clk_i),
      .rst_i(rst_i),

      .cmd_valid_i   (cmd_valid_i),
      .cmd_ready_o   (cmd_ready_o),
      .cmd_write_i   (cmd_write_i),
      .cmd_type1_i   (cmd_type1_i),
      .cmd_bdf_i     (cmd_bdf_i),
      .cmd_reg_num_i (cmd_reg_num_i),
      .cmd_ext_reg_i (cmd_ext_reg_i),
      .cmd_first_be_i(cmd_first_be_i),
      .cmd_wdata_i   (cmd_wdata_i),

      .rsp_valid_o     (rsp_valid_o),
      .rsp_ready_i     (rsp_ready_i),
      .rsp_outcome_o   (rsp_outcome),
      .rsp_rdata_o     (rsp_rdata_o),
      .rsp_status_raw_o(rsp_status_raw_o),
      .crs_retries_o   (crs_retries_o),

      .s_axis_rq_tdata_o (s_axis_rq_tdata),
      .s_axis_rq_tkeep_o (s_axis_rq_tkeep),
      .s_axis_rq_tvalid_o(s_axis_rq_tvalid),
      .s_axis_rq_tlast_o (s_axis_rq_tlast),
      .s_axis_rq_tuser_o (s_axis_rq_tuser),
      .s_axis_rq_tready_i(s_axis_rq_tready),

      .pcie_rq_tag_i    (pcie_rq_tag_o),
      .pcie_rq_tag_vld_i(pcie_rq_tag_vld_o),

      .m_axis_rc_tdata_i (m_axis_rc_tdata),
      .m_axis_rc_tkeep_i (m_axis_rc_tkeep),
      .m_axis_rc_tvalid_i(m_axis_rc_tvalid),
      .m_axis_rc_tlast_i (m_axis_rc_tlast),
      .m_axis_rc_tready_o(m_axis_rc_tready),

      .cpl_timeout_valid_i(cpl_timeout_valid_o),
      .cpl_timeout_tag_i  (cpl_timeout_tag_o)
  );

  // -------------------------------------------------------------------------
  // The real Root Complex requester surface, unmodified.
  // -------------------------------------------------------------------------
  pcie_rq_rc_top #(
      .AXIS_DATA_WIDTH   (AXIS_DATA_WIDTH),
      .AXIS_KEEP_WIDTH   (AXIS_KEEP_WIDTH),
      .AXIS_USER_WIDTH   (AXIS_USER_WIDTH),
      .TL_DATA_WIDTH     (TL_DATA_WIDTH),
      .TL_KEEP_WIDTH     (TL_KEEP_WIDTH),
      .TL_USER_WIDTH     (TL_USER_WIDTH),
      .CONTEXT_WIDTH     (CONTEXT_WIDTH),
      .TAG_COUNT         (TAG_COUNT),
      .CPL_TIMEOUT_CYCLES(CPL_TIMEOUT_CYCLES)
  ) u_rq_rc_top (
      .clk_i(clk_i),
      .rst_i(rst_i),

      .link_up_i        (link_up_i),
      .transmit_enable_i(transmit_enable_i),
      .fc_initialized_i (fc_initialized_i),
      .fc_update_valid_i(fc_update_valid_i),
      .fc_ph_i  (fc_ph_i),   .fc_pd_i  (fc_pd_i),
      .fc_nph_i (fc_nph_i),  .fc_npd_i (fc_npd_i),
      .fc_cplh_i(fc_cplh_i), .fc_cpld_i(fc_cpld_i),

      .requester_id_i       (requester_id_i),
      .completer_id_i       (completer_id_i),
      .bus_number_i         (bus_number_i),
      .device_number_i      (device_number_i),
      .function_number_i    (function_number_i),
      .memory_enable_i      (memory_enable_i),
      .extended_tag_enable_i(extended_tag_enable_i),
      .max_payload_bytes_i  (max_payload_bytes_i),
      .max_read_bytes_i     (max_read_bytes_i),
      .rcb_128b_i           (rcb_128b_i),

      .s_axis_rq_tdata (s_axis_rq_tdata),
      .s_axis_rq_tkeep (s_axis_rq_tkeep),
      .s_axis_rq_tvalid(s_axis_rq_tvalid),
      .s_axis_rq_tlast (s_axis_rq_tlast),
      .s_axis_rq_tuser (s_axis_rq_tuser),
      .s_axis_rq_tready(s_axis_rq_tready),

      .pcie_rq_tag_o    (pcie_rq_tag_o),
      .pcie_rq_tag_vld_o(pcie_rq_tag_vld_o),

      .m_axis_rc_tdata (m_axis_rc_tdata),
      .m_axis_rc_tkeep (m_axis_rc_tkeep),
      .m_axis_rc_tvalid(m_axis_rc_tvalid),
      .m_axis_rc_tlast (m_axis_rc_tlast),
      .m_axis_rc_tready(m_axis_rc_tready),

      .s_dllp_axis_tdata (s_dllp_axis_tdata),
      .s_dllp_axis_tkeep (s_dllp_axis_tkeep),
      .s_dllp_axis_tvalid(s_dllp_axis_tvalid),
      .s_dllp_axis_tlast (s_dllp_axis_tlast),
      .s_dllp_axis_tuser (s_dllp_axis_tuser),
      .s_dllp_axis_tready(s_dllp_axis_tready),

      .m_dllp_axis_tdata (m_dllp_axis_tdata),
      .m_dllp_axis_tkeep (m_dllp_axis_tkeep),
      .m_dllp_axis_tvalid(m_dllp_axis_tvalid),
      .m_dllp_axis_tlast (m_dllp_axis_tlast),
      .m_dllp_axis_tuser (m_dllp_axis_tuser),
      .m_dllp_axis_tready(m_dllp_axis_tready),

      // ---- Stage F-1 completer surface -----------------------------------
      // Tied off: this top does not present a completer interface. The CQ/CC
      // ports live on pcie_rq_rc_top only until a later rung carries them
      // through the DL-stacked and enumeration tops. m_axis_cq_tready is 1'b0
      // and s_axis_cc_tvalid is 1'b0, so the completer is idle and
      // back-pressured rather than accepting-and-dropping.
      .m_axis_cq_tdata (),     .m_axis_cq_tkeep (),     .m_axis_cq_tvalid(),
      .m_axis_cq_tlast (),     .m_axis_cq_tuser (),     .m_axis_cq_tready(1'b0),
      .s_axis_cc_tdata ('0),   .s_axis_cc_tkeep ('0),   .s_axis_cc_tvalid(1'b0),
      .s_axis_cc_tlast (1'b0), .s_axis_cc_tuser ('0),   .s_axis_cc_tready(),
      .cq_dropped_o(),         .cq_error_code_o(),      .cq_gearbox_error_o(),
      .cc_protocol_error_o(),  .cc_error_code_o(),      .cc_gearbox_error_o(),

      .rq_protocol_error_o(rq_protocol_error_o),
      .rq_error_code_o    (rq_error_code),
      .rq_gearbox_error_o (rq_gearbox_error_o),

      .rc_unexpected_completion_o(rc_unexpected_completion_o),
      .rc_completion_error_code_o(rc_completion_error_code),
      .rc_protocol_error_o       (rc_protocol_error_o),
      .rc_error_code_o           (rc_error_code),
      .rc_gearbox_error_o        (rc_gearbox_error_o),

      .command_error_valid_o(command_error_valid_o),
      .command_error_code_o (command_error_code),
      .malformed_o          (malformed_o),
      .rx_error_valid_o     (rx_error_valid_o),
      .rx_error_code_o      (rx_error_code),
      .rx_ecrc_error_o      (rx_ecrc_error_o),
      .tx_error_valid_o     (tx_error_valid_o),
      .tx_error_code_o      (tx_error_code),
      .tx_fc_blocked_o      (tx_fc_blocked_o),
      .credit_error_o       (credit_error_o),
      .vc_overflow_o        (vc_overflow_o),
      .cpl_timeout_valid_o  (cpl_timeout_valid_o),
      .cpl_timeout_tag_o    (cpl_timeout_tag_o),
      .late_cpl_valid_o     (late_cpl_valid_o),
      .late_cpl_tag_o       (late_cpl_tag_o),
      .outstanding_o        (outstanding_o)
  );

endmodule
