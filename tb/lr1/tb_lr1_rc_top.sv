// ---------------------------------------------------------------------------
// tb_lr1_rc_top -- pcie_rc_top with every pin exported, for the LR1 far end
//
// Author: Kourosh Ghahramani
// Silicon Systems Research Lab, University of Washington
//
// Purpose
//   The LR1 far end (lr1_*.py) is an upstream port written from Base 2.1. It
//   connects at pcie_rc_top's PIPE seam and observes and drives PINS ONLY
//   (BRIEF_LR1 D3): no row reaches into u_rc hierarchically. So this wrapper
//   exports every PIPE pin -- including phy_txelecidle, phy_rxpolarity,
//   phy_txcompliance, phy_rate and ltssm_debug_state, which tb_pcie_rc_top
//   leaves open -- together with the link and flow-control levels, the
//   enumeration results, the requester sockets and the error outputs.
//
// Interfaces
//   Ports named as on pcie_rc_top. The Root Complex identity is fixed at
//   00:00.0, as in tb_pcie_rc_top. The completer socket is tied off
//   (m_axis_cq_tready = 1, s_axis_cc_* idle): no LR1-a row sends the RC a
//   request.
//
// Clock and reset
//   clk_i drives every clock of u_rc (one clock, as in every Verilator bench
//   of pcie_rc_top). rst_i is active high.
//
// Parameters
//   SIM_FAST_LINK is 0 for every LR1 row (RULINGS_LR1 D; R and R-dbg are
//   built with 0). MAX_PAYLOAD_BYTES drives max_payload_bytes_i.
//
// References
//   PCIe Base Spec r2.1, §4.2.6
//   PG239, Table 5 to Table 14
// ---------------------------------------------------------------------------

module tb_lr1_rc_top
  import tlp_pkg::*;
  import pcie_rq_rc_pkg::*;
  import pcie_enum_pkg::*;
#(
    parameter int MAX_NUM_LANES     = 1,
    parameter int PHY_DATA_WIDTH    = 16,
    parameter int SIM_FAST_LINK     = 0,
    parameter int MAX_PAYLOAD_BYTES = 128,
    parameter int AXIS_DATA_WIDTH   = 128,
    parameter int AXIS_KEEP_WIDTH   = AXIS_DATA_WIDTH / 32,
    parameter int AXIS_USER_WIDTH   = 60,
    parameter int TAG_COUNT         = 32
) (
    input  logic clk_i,
    input  logic rst_i,
    input  logic en_i,
    input  logic transmit_enable_i,
    input  logic tx_elec_idle,
    input  logic phy_ready_en,

    // ---- PIPE data ---------------------------------------------------------
    output logic [(MAX_NUM_LANES*PHY_DATA_WIDTH)-1:0]   phy_txdata,
    output logic [MAX_NUM_LANES-1:0]                    phy_txdata_valid,
    output logic [(MAX_NUM_LANES*PHY_DATA_WIDTH/8)-1:0] phy_txdatak,
    output logic [MAX_NUM_LANES-1:0]                    phy_txstart_block,
    output logic [(2*MAX_NUM_LANES)-1:0]                phy_txsync_header,
    input  logic [(MAX_NUM_LANES*PHY_DATA_WIDTH)-1:0]   phy_rxdata,
    input  logic [MAX_NUM_LANES-1:0]                    phy_rxdata_valid,
    input  logic [(MAX_NUM_LANES*PHY_DATA_WIDTH/8)-1:0] phy_rxdatak,
    input  logic [MAX_NUM_LANES-1:0]                    phy_rxstart_block,
    input  logic [(2*MAX_NUM_LANES)-1:0]                phy_rxsync_header,

    // ---- PIPE command and status -------------------------------------------
    output logic                         phy_txdetectrx,
    output logic [MAX_NUM_LANES-1:0]     phy_txelecidle,
    output logic [MAX_NUM_LANES-1:0]     phy_txcompliance,
    output logic [MAX_NUM_LANES-1:0]     phy_rxpolarity,
    output logic [1:0]                   phy_powerdown,
    output logic [2:0]                   phy_rate,
    input  logic [MAX_NUM_LANES-1:0]     phy_rxvalid,
    input  logic [MAX_NUM_LANES-1:0]     phy_phystatus,
    input  logic                         phy_phystatus_rst,
    input  logic [MAX_NUM_LANES-1:0]     phy_rxelecidle,
    input  logic [(MAX_NUM_LANES*3)-1:0] phy_rxstatus,
    output logic [2:0]                   phy_txmargin,
    output logic                         phy_txswing,
    output logic                         phy_txdeemph,
    output logic [7:0]                   pipe_width_o,
    output logic                         as_mac_in_detect,
    output logic                         as_cdr_hold_req,
    output logic [20:0]                  ltssm_debug_state,

    // ---- link and flow-control levels --------------------------------------
    output logic link_up_o,
    output logic fc_initialized_o,
    output logic fc_init_done_o,
    output logic ok_to_issue_o,
    output logic [7:0] cfg_bus_number_o,
    output logic [4:0] cfg_device_number_o,
    output logic [2:0] cfg_function_number_o,

    // ---- enumeration -------------------------------------------------------
    input  logic       scan_start_i,
    input  logic [7:0] scan_bus_i,
    input  logic       bar_enable_i,
    input  logic       bridge_enable_i,
    output logic                    scan_busy_o,
    output logic                    scan_done_o,
    output logic                    scan_error_o,
    output enum_error_e             scan_error_code_o,
    output logic                    err_credit_blocked_o,
    output logic                    device_present_o,
    output logic                    unsupported_device_o,
    output logic [15:0]             device_bdf_o,
    output logic [15:0]             vendor_id_o,
    output logic [15:0]             device_id_o,
    output logic [7:0]              header_type_o,
    output logic                    multifunction_o,
    output logic                    bar_busy_o,
    output logic                    enum_done_o,
    output logic                    enum_error_o,
    output enum_error_e             enum_error_code_o,
    output logic [3:0]              bar_count_o,
    output logic [BAR_SLOTS-1:0]    bar_valid_o,
    output logic [BAR_SLOTS-1:0]    bar_is_64_o,
    output logic [BAR_SLOTS-1:0]    bar_prefetch_o,
    output logic [BAR_SLOTS*64-1:0] bar_size_o,
    output logic [BAR_SLOTS*64-1:0] bar_addr_o,
    output logic [BAR_SLOTS-1:0]    io_bar_mask_o,

    // ---- requester sockets -------------------------------------------------
    input  logic [AXIS_DATA_WIDTH-1:0] s_axis_rq_tdata,
    input  logic [AXIS_KEEP_WIDTH-1:0] s_axis_rq_tkeep,
    input  logic                       s_axis_rq_tvalid,
    input  logic                       s_axis_rq_tlast,
    input  logic [AXIS_USER_WIDTH-1:0] s_axis_rq_tuser,
    output logic                       s_axis_rq_tready,
    output logic [AXIS_DATA_WIDTH-1:0] m_axis_rc_tdata,
    output logic [AXIS_KEEP_WIDTH-1:0] m_axis_rc_tkeep,
    output logic                       m_axis_rc_tvalid,
    output logic                       m_axis_rc_tlast,
    input  logic                       m_axis_rc_tready,
    output logic [7:0]                 pcie_rq_tag_o,
    output logic                       pcie_rq_tag_vld_o,
    output logic                       rq_engine_owns_o,

    // ---- error surface -----------------------------------------------------
    output logic       cq_dropped_o,
    output logic [3:0] cq_error_code_o,
    output logic       rq_protocol_error_o,
    output rq_error_e  rq_error_code_o,
    output logic       rc_unexpected_completion_o,
    output tlp_error_e rc_completion_error_code_o,
    output logic       rc_protocol_error_o,
    output rc_error_e  rc_error_code_o,
    output logic       command_error_valid_o,
    output tlp_error_e command_error_code_o,
    output logic       malformed_o,
    output logic       rx_error_valid_o,
    output tlp_error_e rx_error_code_o,
    output logic       rx_ecrc_error_o,
    output logic       tx_error_valid_o,
    output tlp_error_e tx_error_code_o,
    output logic       tx_fc_blocked_o,
    output logic       credit_error_o,
    output logic       vc_overflow_o,
    output logic       cpl_timeout_valid_o,
    output logic [7:0] cpl_timeout_tag_o,
    output logic       late_cpl_valid_o,
    output logic [7:0] late_cpl_tag_o
);

  // The 8 Gen3/4 equalization inputs, tied as in tb_pcie_rc_top.
  localparam logic [5:0]                    EQ_FS_TIE    = '0;
  localparam logic [5:0]                    EQ_LF_TIE    = '0;
  localparam logic [(MAX_NUM_LANES*18)-1:0] EQ_COEFF_TIE = '0;
  localparam logic [MAX_NUM_LANES-1:0]      EQ_LANE_TIE  = '0;

  pcie_rc_top #(
      .AXIS_DATA_WIDTH(AXIS_DATA_WIDTH),
      .AXIS_USER_WIDTH(AXIS_USER_WIDTH),
      .TAG_COUNT      (TAG_COUNT),
      .MAX_NUM_LANES  (MAX_NUM_LANES),
      .PHY_DATA_WIDTH (PHY_DATA_WIDTH),
      .CLK_PERIOD_NS  (8),
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
      .phy_txelecidle  (phy_txelecidle),
      .phy_txcompliance(phy_txcompliance),
      .phy_rxpolarity  (phy_rxpolarity),
      .phy_powerdown   (phy_powerdown),
      .phy_rate        (phy_rate),

      .phy_rxvalid      (phy_rxvalid),
      .phy_phystatus    (phy_phystatus),
      .phy_phystatus_rst(phy_phystatus_rst),
      .phy_rxelecidle   (phy_rxelecidle),
      .phy_rxstatus     (phy_rxstatus),

      .phy_txmargin(phy_txmargin),
      .phy_txswing (phy_txswing),
      .phy_txdeemph(phy_txdeemph),
      .pipe_width_o(pipe_width_o),

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
      .as_mac_in_detect (as_mac_in_detect),
      .as_cdr_hold_req  (as_cdr_hold_req),
      .ltssm_debug_state(ltssm_debug_state),

      .link_up_o       (link_up_o),
      .fc_initialized_o(fc_initialized_o),
      .fc_init_done_o  (fc_init_done_o),
      .ok_to_issue_o   (ok_to_issue_o),

      .requester_id_i       (16'h0000),
      .completer_id_i       (16'h0000),
      .bus_number_i         (8'h00),
      .device_number_i      (5'h00),
      .function_number_i    (3'h0),
      .memory_enable_i      (1'b1),
      .extended_tag_enable_i(1'b0),
      .max_payload_bytes_i  (13'(MAX_PAYLOAD_BYTES)),
      .max_read_bytes_i     (13'd512),
      .rcb_128b_i           (1'b0),
      .cfg_bus_number_o     (cfg_bus_number_o),
      .cfg_device_number_o  (cfg_device_number_o),
      .cfg_function_number_o(cfg_function_number_o),

      .scan_start_i   (scan_start_i),
      .scan_bus_i     (scan_bus_i),
      .bar_enable_i   (bar_enable_i),
      .bridge_enable_i(bridge_enable_i),

      .scan_busy_o         (scan_busy_o),
      .scan_done_o         (scan_done_o),
      .scan_error_o        (scan_error_o),
      .scan_error_code_o   (scan_error_code_o),
      .err_credit_blocked_o(err_credit_blocked_o),
      .device_present_o    (device_present_o),
      .unsupported_device_o(unsupported_device_o),
      .device_bdf_o        (device_bdf_o),
      .vendor_id_o         (vendor_id_o),
      .device_id_o         (device_id_o),
      .header_type_o       (header_type_o),
      .multifunction_o     (multifunction_o),
      .bar_busy_o          (bar_busy_o),
      .enum_done_o         (enum_done_o),
      .enum_error_o        (enum_error_o),
      .enum_error_code_o   (enum_error_code_o),
      .bar_count_o         (bar_count_o),
      .bar_valid_o         (bar_valid_o),
      .bar_is_64_o         (bar_is_64_o),
      .bar_prefetch_o      (bar_prefetch_o),
      .bar_size_o          (bar_size_o),
      .bar_addr_o          (bar_addr_o),
      .io_bar_mask_o       (io_bar_mask_o),

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
      .m_axis_rc_tdata  (m_axis_rc_tdata),
      .m_axis_rc_tkeep  (m_axis_rc_tkeep),
      .m_axis_rc_tvalid (m_axis_rc_tvalid),
      .m_axis_rc_tlast  (m_axis_rc_tlast),
      .m_axis_rc_tready (m_axis_rc_tready),
      .pcie_rq_tag_o    (pcie_rq_tag_o),
      .pcie_rq_tag_vld_o(pcie_rq_tag_vld_o),
      .rq_engine_owns_o (rq_engine_owns_o),

      .m_axis_cq_tdata (), .m_axis_cq_tkeep (),
      .m_axis_cq_tvalid(), .m_axis_cq_tlast (),
      .m_axis_cq_tuser (), .m_axis_cq_tready(1'b1),
      .s_axis_cc_tdata ('0), .s_axis_cc_tkeep ('0),
      .s_axis_cc_tvalid(1'b0), .s_axis_cc_tlast (1'b0),
      .s_axis_cc_tuser ('0), .s_axis_cc_tready(),
      .cq_dropped_o      (cq_dropped_o),
      .cq_error_code_o   (cq_error_code_o),
      .cq_gearbox_error_o(),
      .cc_protocol_error_o(), .cc_error_code_o(),
      .cc_gearbox_error_o(),

      .rq_protocol_error_o       (rq_protocol_error_o),
      .rq_error_code_o           (rq_error_code_o),
      .rq_gearbox_error_o        (),
      .rc_unexpected_completion_o(rc_unexpected_completion_o),
      .rc_completion_error_code_o(rc_completion_error_code_o),
      .rc_protocol_error_o       (rc_protocol_error_o),
      .rc_error_code_o           (rc_error_code_o),
      .rc_gearbox_error_o        (),
      .command_error_valid_o     (command_error_valid_o),
      .command_error_code_o      (command_error_code_o),
      .malformed_o     (malformed_o),
      .rx_error_valid_o(rx_error_valid_o),
      .rx_error_code_o (rx_error_code_o),
      .rx_ecrc_error_o (rx_ecrc_error_o),
      .tx_error_valid_o(tx_error_valid_o),
      .tx_error_code_o (tx_error_code_o),
      .tx_fc_blocked_o (tx_fc_blocked_o),
      .credit_error_o  (credit_error_o),
      .vc_overflow_o   (vc_overflow_o),
      .cpl_timeout_valid_o(cpl_timeout_valid_o),
      .cpl_timeout_tag_o  (cpl_timeout_tag_o),
      .late_cpl_valid_o   (late_cpl_valid_o),
      .late_cpl_tag_o     (late_cpl_tag_o),
      .outstanding_o      ()
  );

endmodule
