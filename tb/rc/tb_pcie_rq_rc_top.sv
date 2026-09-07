// ---------------------------------------------------------------------------
// tb_pcie_rq_rc_top -- shim for pcie_rq_rc_top (Commit 2a-iii).
//
// The DUT is already the whole loop, so this shim does almost nothing: it
// exists to flatten the enum-typed error outputs into plain vectors that
// cocotb can read, and to give the Python side the DLL-facing streams it needs
// to play the completer.
//
// The bench sits where the Data Link Layer would: it observes the TX stream
// (m_dllp_axis_*) to see what the Root Complex emitted, and drives the RX
// stream (s_dllp_axis_*) to answer it. The tiny config completer that does
// that lives entirely in Python -- see test_pcie_rq_rc_top.py, SS THE COMPLETER
// -- so replacing it with Joy's protocol-checking endpoint model is a matter of
// swapping one Python class, with no RTL change here.
//
// ! FLOW CONTROL. The DUT emits nothing and reports nothing until link_up_i,
// transmit_enable_i, fc_initialized_i and one fc_update_valid_i pulse with
// non-zero credits are all present. Every one of those is driven from Python.
// See pcie_rq_rc_top.sv's header.
//
// TAG_COUNT is 8 here, not the module's default 32. V4 drives the design to
// tag exhaustion on purpose, and 8 makes that a short test instead of a slow
// one; nothing in either wrapper is sensitive to the count.
// ---------------------------------------------------------------------------
`timescale 1ns/1ps
module tb_pcie_rq_rc_top
  import tlp_pkg::*;
  import pcie_rq_rc_pkg::*;
;

  localparam int AXIS_DATA_WIDTH = 128;
  localparam int AXIS_KEEP_WIDTH = AXIS_DATA_WIDTH / 32;
  localparam int AXIS_USER_WIDTH = 60;
  localparam int TL_DATA_WIDTH   = 32;
  localparam int TL_KEEP_WIDTH   = TL_DATA_WIDTH / 8;
  localparam int TL_USER_WIDTH   = 3;
  localparam int CONTEXT_WIDTH   = 16;
  localparam int TAG_COUNT       = 8;

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

  // ---- host RQ AXIS -------------------------------------------------------
  logic [AXIS_DATA_WIDTH-1:0] s_axis_rq_tdata;
  logic [AXIS_KEEP_WIDTH-1:0] s_axis_rq_tkeep;
  logic                       s_axis_rq_tvalid;
  logic                       s_axis_rq_tlast;
  logic [AXIS_USER_WIDTH-1:0] s_axis_rq_tuser;
  logic                       s_axis_rq_tready;

  logic [7:0] pcie_rq_tag_o;
  logic       pcie_rq_tag_vld_o;

  // ---- host RC AXIS -------------------------------------------------------
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
  // ---- Stage F-1 completer surface, flattened for cocotb ------------------
  // The CQ/CC ports are declared here from the boundary commit onward so the
  // bench's interface stops changing once behaviour starts landing. Until
  // pcie_cq_if exists the DUT drives these to constants; the A4 rows in
  // test_pcie_rq_rc_top.py observe the DLL wire, not these, so they are
  // unaffected either way.
  localparam int CQ_USER_WIDTH = 88;   // PG213 Table 10, 128/256-bit interface
  localparam int CC_USER_WIDTH = 33;   // PG213 Table 62

  logic [AXIS_DATA_WIDTH-1:0] m_axis_cq_tdata;
  logic [AXIS_KEEP_WIDTH-1:0] m_axis_cq_tkeep;
  logic                       m_axis_cq_tvalid;
  logic                       m_axis_cq_tlast;
  logic [CQ_USER_WIDTH-1:0]   m_axis_cq_tuser;
  logic                       m_axis_cq_tready;

  logic [AXIS_DATA_WIDTH-1:0] s_axis_cc_tdata;
  logic [AXIS_KEEP_WIDTH-1:0] s_axis_cc_tkeep;
  logic                       s_axis_cc_tvalid;
  logic                       s_axis_cc_tlast;
  logic [CC_USER_WIDTH-1:0]   s_axis_cc_tuser;
  logic                       s_axis_cc_tready;

  logic       cq_dropped_o;
  logic [3:0] cq_error_code_o;
  logic       cq_gearbox_error_o;
  logic       cc_protocol_error_o;
  logic [3:0] cc_error_code_o;
  logic       cc_gearbox_error_o;

  logic       rq_protocol_error_o;
  rq_error_e  rq_error_code;
  logic [3:0] rq_error_code_o;
  logic       rq_gearbox_error_o;
  assign rq_error_code_o = 4'(rq_error_code);

  logic       rc_unexpected_completion_o;
  tlp_error_e rc_completion_error_code;
  logic [4:0] rc_completion_error_code_o;
  assign rc_completion_error_code_o = 5'(rc_completion_error_code);

  logic       rc_protocol_error_o;
  rc_error_e  rc_error_code;
  logic [3:0] rc_error_code_o;
  logic       rc_gearbox_error_o;
  assign rc_error_code_o = 4'(rc_error_code);

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

  pcie_rq_rc_top #(
      .AXIS_DATA_WIDTH(AXIS_DATA_WIDTH),
      .AXIS_KEEP_WIDTH(AXIS_KEEP_WIDTH),
      .AXIS_USER_WIDTH(AXIS_USER_WIDTH),
      .TL_DATA_WIDTH  (TL_DATA_WIDTH),
      .TL_KEEP_WIDTH  (TL_KEEP_WIDTH),
      .TL_USER_WIDTH  (TL_USER_WIDTH),
      .CONTEXT_WIDTH  (CONTEXT_WIDTH),
      .TAG_COUNT      (TAG_COUNT)
  ) dut (
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

      .m_axis_cq_tdata (m_axis_cq_tdata),
      .m_axis_cq_tkeep (m_axis_cq_tkeep),
      .m_axis_cq_tvalid(m_axis_cq_tvalid),
      .m_axis_cq_tlast (m_axis_cq_tlast),
      .m_axis_cq_tuser (m_axis_cq_tuser),
      .m_axis_cq_tready(m_axis_cq_tready),

      .s_axis_cc_tdata (s_axis_cc_tdata),
      .s_axis_cc_tkeep (s_axis_cc_tkeep),
      .s_axis_cc_tvalid(s_axis_cc_tvalid),
      .s_axis_cc_tlast (s_axis_cc_tlast),
      .s_axis_cc_tuser (s_axis_cc_tuser),
      .s_axis_cc_tready(s_axis_cc_tready),

      .cq_dropped_o       (cq_dropped_o),
      .cq_error_code_o    (cq_error_code_o),
      .cq_gearbox_error_o (cq_gearbox_error_o),
      .cc_protocol_error_o(cc_protocol_error_o),
      .cc_error_code_o    (cc_error_code_o),
      .cc_gearbox_error_o (cc_gearbox_error_o),

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
