// ---------------------------------------------------------------------------
// tb_pcie_cfg_hold_tlp -- the post-reset hold and the CRS window, through a
// real pcie_rq_rc_top
//
// Author: Kourosh Ghahramani
// Silicon Systems Research Lab, University of Washington
//
// Purpose
//   pcie_enum_top in front of the real pcie_rq_rc_top, the scan_tlp shape,
//   with the Python test playing the Data Link Layer and a far end whose
//   answers depend on time: CRS until a given cycle, silence, or a Successful
//   Completion. The rows (§63 #22) time every Configuration Request against
//   the rise of fc_initialized_i, which stands for DL_Active: pcie_rc_top
//   connects fc_init_done_o to the engine's link_active_i, and C2 connects
//   fc_initialized_i here the same way.
//
// Interfaces
//   As tb_pcie_enum_scan_tlp, plus bench_*: this bench's timing values.
//
// Clock and reset
//   clk_i only, from the test. rst_i is synchronous and active high.
//
// Limitations
//   The BAR and bridge stages are off. Credits are advertised once, at the
//   field maxima, unless a row drives them.
//
// References
//   PCIe Base Spec r2.1, §2.3.1
//   PCIe Base Spec r2.1, §2.3.2
//   PCIe Base Spec r2.1, §6.6.1
//   PCIe Base Spec r3.0, §6.7.3.3
// ---------------------------------------------------------------------------
`timescale 1ns/1ps
module tb_pcie_cfg_hold_tlp
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

  // Bench-scale timing. The hold is a tenth of the window, as on the board
  // (100 ms and 1.0 s); the backoff is coarser than the board's 1 ms so a
  // row stays short. CRS_WINDOW_CYCLES is not yet a parameter of
  // pcie_enum_top (§63 #22 C2): the rows read it from bench_crs_window_cycles
  // below, and C3 passes it in.
  localparam int unsigned CRS_RETRY_MAX       = 3;
  localparam int unsigned CRS_BACKOFF_CYCLES  = 200;
  localparam int unsigned CPL_TIMEOUT_CYCLES  = 32'd4096;
  localparam int unsigned CFG_HOLD_CYCLES     = 2000;
  localparam int unsigned CRS_WINDOW_CYCLES   = 20000;

  // The values above, for the test module, so they have one source.
  logic [31:0] bench_crs_retry_max;
  logic [31:0] bench_crs_backoff_cycles;
  logic [31:0] bench_cpl_timeout_cycles;
  logic [31:0] bench_cfg_hold_cycles;
  logic [31:0] bench_crs_window_cycles;
  assign bench_crs_retry_max      = CRS_RETRY_MAX;
  assign bench_crs_backoff_cycles = CRS_BACKOFF_CYCLES;
  assign bench_cpl_timeout_cycles = CPL_TIMEOUT_CYCLES;
  assign bench_cfg_hold_cycles    = CFG_HOLD_CYCLES;
  assign bench_crs_window_cycles  = CRS_WINDOW_CYCLES;


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

  // ---- scan control and status surface ------------------------------------
  logic        scan_start_i;
  logic [7:0]  scan_bus_i;

  logic        scan_busy_o;
  logic        scan_done_o;
  logic        scan_error_o;
  enum_error_e scan_error_code;
  logic [3:0]  scan_error_code_o;
  assign scan_error_code_o = 4'(scan_error_code);
  logic        err_credit_blocked_o;

  logic        device_present_o;
  logic        unsupported_device_o;
  logic [15:0] device_bdf_o;
  logic [15:0] vendor_id_o;
  logic [15:0] device_id_o;
  logic [7:0]  header_type_o;
  logic        multifunction_o;

  // ---- the socket, internal: scan/primitive <-> pcie_rq_rc_top ------------
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
  // The enumeration engine. Its one pcie_cfg_txn holds the timer (§63 #22).
  // -------------------------------------------------------------------------
  pcie_enum_top #(
      .AXIS_DATA_WIDTH   (AXIS_DATA_WIDTH),
      .AXIS_KEEP_WIDTH   (AXIS_KEEP_WIDTH),
      .AXIS_USER_WIDTH   (AXIS_USER_WIDTH),
      .CRS_RETRY_MAX     (CRS_RETRY_MAX),
      .CRS_BACKOFF_CYCLES(CRS_BACKOFF_CYCLES),
      .CPL_TIMEOUT_CYCLES(CPL_TIMEOUT_CYCLES),
      .CFG_HOLD_CYCLES   (CFG_HOLD_CYCLES)
  ) u_enum (
      .clk_i(clk_i),
      .rst_i(rst_i),
      // DL_Active, as pcie_rc_top connects fc_init_done_o.
      .link_active_i(fc_initialized_i),

      .scan_start_i(scan_start_i),
      .scan_bus_i  (scan_bus_i),
      // Presence scan only: the rows judge the probe and header reads.
      .bar_enable_i(1'b0),
      .bridge_enable_i(1'b0),
      .bus_done_o(), .bus_bypassed_o(),
      .sec_scan_done_o(), .sec_device_present_o(), .sec_unsupported_device_o(),
      .sec_device_bdf_o(), .sec_vendor_id_o(), .sec_device_id_o(),
      .sec_header_type_o(), .sec_multifunction_o(),
      .sec_enum_done_o(), .sec_bar_count_o(), .sec_bar_valid_o(),
      .sec_bar_is_64_o(), .sec_bar_prefetch_o(), .sec_bar_size_o(),
      .sec_bar_addr_o(), .sec_io_bar_mask_o(),

      .scan_busy_o         (scan_busy_o),
      .scan_done_o         (scan_done_o),
      .scan_error_o        (scan_error_o),
      .scan_error_code_o   (scan_error_code),
      .err_credit_blocked_o(err_credit_blocked_o),

      .device_present_o    (device_present_o),
      .unsupported_device_o(unsupported_device_o),
      .device_bdf_o        (device_bdf_o),
      .vendor_id_o         (vendor_id_o),
      .device_id_o         (device_id_o),
      .header_type_o       (header_type_o),
      .multifunction_o     (multifunction_o),

      // BAR-phase surface: unused here, and provably inert with bar_enable_i
      // tied low -- pcie_enum_bar never leaves S_IDLE.
      .bar_busy_o       (),
      .enum_done_o      (),
      .enum_error_o     (),
      .enum_error_code_o(),
      .bar_count_o      (),
      .bar_valid_o      (),
      .bar_is_64_o      (),
      .bar_prefetch_o   (),
      .bar_size_o       (),
      .bar_addr_o       (),
      .io_bar_mask_o    (),

      .tx_fc_blocked_i(tx_fc_blocked_o),

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
