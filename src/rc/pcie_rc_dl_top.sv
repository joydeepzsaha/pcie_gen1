// ---------------------------------------------------------------------------
// pcie_rc_dl_top -- Root Complex Transaction Layer on the Data Link Layer
//
// Author: Kourosh Ghahramani
// Silicon Systems Research Lab, University of Washington
// Based on: src/pcie_endpoint/pcie_endpoint_top.sv, src/tlp/tlp_layer.sv (Joydeep Saha); src/pcie_phy_core/pcie_phy_top.sv (Idris Somoye)
//
// Purpose
//   Stacks pcie_rq_rc_top (u_rc) on pcie_datalink_layer (u_dl): the
//   Transaction Layer's DLL streams connect to u_dl, and its flow-control
//   credit comes from u_dl's InitFC and UpdateFC exchange. The PHY side is a
//   32-bit AXI4-Stream, so the LTSSM and logical PHY, or a bench, sit outside.
//   fc_init_done_o and ok_to_issue_o export the standing part of u_rc's
//   transmit gate.
//
// Interfaces
//   Link          phy_link_up_i: low also resets the Transaction Layer, u_dl's
//                 link state and fc_init_sticky_r. transmit_enable_i: a term
//                 of u_rc's transmit gate. idle_valid_i: passed to u_dl.
//   Start gate    fc_init_done_o, ok_to_issue_o: FC-init state and the
//                 standing transmit conditions.
//   PHY streams   s_phy_axis_*, m_phy_axis_*: u_dl's 32-bit PHY side.
//   Identity      requester_id_i to rcb_128b_i: inputs to u_rc, not from u_dl.
//                 cfg_*_number_o: what a received CfgWr0 stored in u_dl.
//   Host          s_axis_rq_*, pcie_rq_tag_*, m_axis_rc_*, m_axis_cq_*,
//                 s_axis_cc_*: pcie_rq_rc_top's PG213-style interfaces.
//   Status        rq_*, rc_*, cq_*, cc_* and the Transaction Layer error and
//                 Completion Timeout outputs: from u_rc, unchanged.
//
// Clock and reset
//   clk_i only. rst_i is active high and synchronous, except in u_dl's
//   pcie_datalink_init, which resets asynchronously. u_dl keeps its default
//   CLK_PERIOD_NS of 8 for its timers.
//
// References
//   PCIe Base Spec r2.1, §2.2.6.2
//   PCIe Base Spec r2.1, §2.3.1
//   PCIe Base Spec r2.1, §3.3.1
//   PG213, Table 9: Completer Request Interface Port Descriptions
//   PG213, Table 11: Completer Completion Interface Port Descriptions
// ---------------------------------------------------------------------------
`timescale 1ns/1ps
module pcie_rc_dl_top
  import tlp_pkg::*;
  import pcie_rq_rc_pkg::*;
#(
    parameter int AXIS_DATA_WIDTH = 128,
    parameter int AXIS_KEEP_WIDTH = AXIS_DATA_WIDTH / 32,
    parameter int AXIS_USER_WIDTH = 60,
    parameter int CONTEXT_WIDTH   = 16,
    parameter int TAG_COUNT       = 32,
    // Completion Timeout; 0 disables. See tlp_request_tracker.sv header.
    parameter int unsigned CPL_TIMEOUT_CYCLES = tlp_pkg::CPL_TIMEOUT_DEFAULT_CYCLES,  // 10 ms at 8 ns
    // PG213 tuser widths, passed to pcie_rq_rc_top: m_axis_cq_tuser is 88 bits
    // (PG213, Table 9), s_axis_cc_tuser 33 (PG213, Table 11). pcie_cc_if does
    // not read s_axis_cc_tuser.
    parameter int CQ_USER_WIDTH   = 88,
    parameter int CC_USER_WIDTH   = 33
) (
    input  logic                        clk_i,
    input  logic                        rst_i,

    // ---- link state ---------------------------------------------------------
    // phy_link_up_i is also the link-scoped reset. Low, it resets tlp_layer
    // and its credit pools through u_rc.link_up_i, raises u_dl's soft_reset
    // through pcie_datalink_init, and clears fc_init_sticky_r below, so a
    // link-down clears all three.
    input  logic                        phy_link_up_i,
    input  logic                        idle_valid_i,
    input  logic                        transmit_enable_i,

    // ---- start-gate status --------------------------------------------------
    // fc_init_done_o is fc_init_sticky_r, the FC-init state u_rc uses, not
    // u_dl's raw fc_initialized_o. It does not fall within a link-up.
    //
    // ok_to_issue_o is the standing part of tlp_layer's transmit gate
    // (vc_packet_ready): FC init done, transmit_enable_i and phy_link_up_i.
    // The rest of that gate is header and data credit availability in
    // tlp_credit_manager, which depends on the class and size of the packet
    // waiting, so there is no class-independent bit to export.
    // tx_fc_blocked_o is the credit view, and it is asserted only while a
    // packet waits.
    //
    // phy_link_up_i is a term of its own because fc_init_sticky_r is
    // registered and still reads 1 in the cycle after a link drop, while
    // tlp_layer's gate is combinational.
    output logic                        fc_init_done_o,
    output logic                        ok_to_issue_o,

    // ---- u_dl's PHY-side streams --------------------------------------------
    input  logic [31:0]                 s_phy_axis_tdata,
    input  logic [3:0]                  s_phy_axis_tkeep,
    input  logic                        s_phy_axis_tvalid,
    input  logic                        s_phy_axis_tlast,
    input  logic [2:0]                  s_phy_axis_tuser,
    output logic                        s_phy_axis_tready,
    output logic [31:0]                 m_phy_axis_tdata,
    output logic [3:0]                  m_phy_axis_tkeep,
    output logic                        m_phy_axis_tvalid,
    output logic                        m_phy_axis_tlast,
    output logic [2:0]                  m_phy_axis_tuser,
    input  logic                        m_phy_axis_tready,

    // ---- identity and negotiated limits -------------------------------------
    // Inputs to u_rc rather than u_dl's cfg_*_number_o. Those hold the numbers
    // a received Type 0 Configuration Write supplied, which is how a Function
    // learns its Bus and Device Number; a Root Complex assigns its own in an
    // implementation-specific way (PCIe Base Spec r2.1, §2.2.6.2). u_dl's
    // ext_tag_enable_o to max_payload_size_o are constant 0, so the limits
    // are inputs too.
    input  logic [15:0]                 requester_id_i,
    input  logic [15:0]                 completer_id_i,
    input  logic [7:0]                  bus_number_i,
    input  logic [4:0]                  device_number_i,
    input  logic [2:0]                  function_number_i,
    input  logic                        memory_enable_i,
    input  logic                        extended_tag_enable_i,
    input  logic [12:0]                 max_payload_bytes_i,
    input  logic [12:0]                 max_read_bytes_i,
    input  logic                        rcb_128b_i,

    // ---- PG213 Requester Request AXI4-Stream slave --------------------------
    input  logic [AXIS_DATA_WIDTH-1:0]  s_axis_rq_tdata,
    input  logic [AXIS_KEEP_WIDTH-1:0]  s_axis_rq_tkeep,
    input  logic                        s_axis_rq_tvalid,
    input  logic                        s_axis_rq_tlast,
    input  logic [AXIS_USER_WIDTH-1:0]  s_axis_rq_tuser,
    output logic                        s_axis_rq_tready,

    // ---- core-managed tag presentation --------------------------------------
    output logic [7:0]                  pcie_rq_tag_o,
    output logic                        pcie_rq_tag_vld_o,

    // ---- PG213 Requester Completion AXI4-Stream master ----------------------
    output logic [AXIS_DATA_WIDTH-1:0]  m_axis_rc_tdata,
    output logic [AXIS_KEEP_WIDTH-1:0]  m_axis_rc_tkeep,
    output logic                        m_axis_rc_tvalid,
    output logic                        m_axis_rc_tlast,
    input  logic                        m_axis_rc_tready,

    // ---- numbers stored by u_dl, observation only ---------------------------
    output logic [7:0]                  cfg_bus_number_o,
    output logic [4:0]                  cfg_device_number_o,
    output logic [2:0]                  cfg_function_number_o,

    // ---- RQ / RC / Transaction Layer error and status, from u_rc ------------
    output logic                        rq_protocol_error_o,
    output rq_error_e                   rq_error_code_o,
    output logic                        rq_gearbox_error_o,
    output logic                        rc_unexpected_completion_o,
    output tlp_error_e                  rc_completion_error_code_o,
    output logic                        rc_protocol_error_o,
    output rc_error_e                   rc_error_code_o,
    output logic                        rc_gearbox_error_o,
    output logic                        command_error_valid_o,
    output tlp_error_e                  command_error_code_o,
    output logic                        malformed_o,
    output logic                        rx_error_valid_o,
    output tlp_error_e                  rx_error_code_o,
    output logic                        rx_ecrc_error_o,
    output logic                        tx_error_valid_o,
    output tlp_error_e                  tx_error_code_o,
    output logic                        tx_fc_blocked_o,
    output logic                        credit_error_o,
    output logic                        vc_overflow_o,
    output logic                        cpl_timeout_valid_o,
    output logic [7:0]                  cpl_timeout_tag_o,
    output logic                        late_cpl_valid_o,
    output logic [7:0]                  late_cpl_tag_o,
    output logic [$clog2(TAG_COUNT+1)-1:0] outstanding_o,

    // ---- PG213 Completer reQuest / Completer Completion ---------------------
    // pcie_rq_rc_top's completer interfaces, passed through unchanged; the
    // descriptor rules are in pcie_cq_if and pcie_cc_if. An inbound request
    // leaves on m_axis_cq_* or raises cq_dropped_o, and a dropped non-posted
    // request is answered with an Unsupported Request Completion (PCIe Base
    // Spec r2.1, §2.3.1).
    output logic [AXIS_DATA_WIDTH-1:0]  m_axis_cq_tdata,
    output logic [AXIS_KEEP_WIDTH-1:0]  m_axis_cq_tkeep,
    output logic                        m_axis_cq_tvalid,
    output logic                        m_axis_cq_tlast,
    output logic [CQ_USER_WIDTH-1:0]    m_axis_cq_tuser,
    input  logic                        m_axis_cq_tready,

    input  logic [AXIS_DATA_WIDTH-1:0]  s_axis_cc_tdata,
    input  logic [AXIS_KEEP_WIDTH-1:0]  s_axis_cc_tkeep,
    input  logic                        s_axis_cc_tvalid,
    input  logic                        s_axis_cc_tlast,
    input  logic [CC_USER_WIDTH-1:0]    s_axis_cc_tuser,
    output logic                        s_axis_cc_tready,

    output logic                        cq_dropped_o,
    output logic [3:0]                  cq_error_code_o,
    output logic                        cq_gearbox_error_o,
    output logic                        cc_protocol_error_o,
    output logic [3:0]                  cc_error_code_o,
    output logic                        cc_gearbox_error_o
);

  // The TL<->DLL seam is fixed at the DLL's native 32-bit Dword-serial shape.
  localparam int TL_DATA_WIDTH = 32;
  localparam int TL_KEEP_WIDTH = 4;
  localparam int TL_USER_WIDTH = 3;

  // -------------------------------------------------------------------------
  // Transaction Layer to Data Link Layer seam
  // -------------------------------------------------------------------------
  // tuser carries nothing in either direction: tlp2dllp overwrites it on the
  // way out and tlp_parser does not read it on the way in. Both sides declare
  // it, so it is wired for AXI4-Stream shape.
  logic [TL_DATA_WIDTH-1:0] tl_to_dl_tdata;
  logic [TL_KEEP_WIDTH-1:0] tl_to_dl_tkeep;
  logic                     tl_to_dl_tvalid;
  logic                     tl_to_dl_tlast;
  logic [TL_USER_WIDTH-1:0] tl_to_dl_tuser;
  logic                     tl_to_dl_tready;

  logic [TL_DATA_WIDTH-1:0] dl_to_tl_tdata;
  logic [TL_KEEP_WIDTH-1:0] dl_to_tl_tkeep;
  logic                     dl_to_tl_tvalid;
  logic                     dl_to_tl_tlast;
  logic [TL_USER_WIDTH-1:0] dl_to_tl_tuser;
  logic                     dl_to_tl_tready;

  logic                     dl_fc_initialized;
  logic                     dl_fc_update_valid;
  logic [7:0]               dl_fc_ph;
  logic [11:0]              dl_fc_pd;
  logic [7:0]               dl_fc_nph;
  logic [11:0]              dl_fc_npd;
  logic [7:0]               dl_fc_cplh;
  logic [11:0]              dl_fc_cpld;

  // FC-init state for u_rc and the start-gate ports: set when u_dl reports FC
  // initialization complete, cleared on reset or link-down. FC_INIT1 for VC0
  // is entered only on entry to DL_Init (PCIe Base Spec r2.1, §3.3.1), and
  // pcie_datalink_init raises soft_reset only on rst_i or a low
  // phy_link_up_i, so the clear cannot mask a re-initialization. dl_fc_initialized does not fall
  // within a link-up either: pcie_flow_ctrl_init holds fc2_values_sent_o high
  // from CHECK_FC2's exit, and dllp_handler's InitFC2 flags stay set until
  // reset.
  logic fc_init_sticky_r;
  always_ff @(posedge clk_i) begin
    if (rst_i || !phy_link_up_i) fc_init_sticky_r <= 1'b0;
    else if (dl_fc_initialized)  fc_init_sticky_r <= 1'b1;
  end

  // The ports read fc_init_sticky_r rather than recompute FC-init state, so
  // u_rc and a client above see the same bit.
  assign fc_init_done_o = fc_init_sticky_r;
  assign ok_to_issue_o  = fc_init_sticky_r && transmit_enable_i && phy_link_up_i;

  // PCIE_WIRE_ORDER = 1 puts the first wire byte of every header Dword on
  // byte lane 0, as DW0 always is: the order u_dl carries TLPs in.
  // pcie_endpoint_top sets it the same way. The default, 0, keeps the header
  // Dwords after DW0 in host Dword order, for benches that drive
  // pcie_rq_rc_top directly.
  pcie_rq_rc_top #(
      .AXIS_DATA_WIDTH(AXIS_DATA_WIDTH),
      .AXIS_KEEP_WIDTH(AXIS_KEEP_WIDTH),
      .AXIS_USER_WIDTH(AXIS_USER_WIDTH),
      .TL_DATA_WIDTH  (TL_DATA_WIDTH),
      .TL_KEEP_WIDTH  (TL_KEEP_WIDTH),
      .TL_USER_WIDTH  (TL_USER_WIDTH),
      .CONTEXT_WIDTH  (CONTEXT_WIDTH),
      .TAG_COUNT      (TAG_COUNT),
      .CPL_TIMEOUT_CYCLES(CPL_TIMEOUT_CYCLES),
      .PCIE_WIRE_ORDER(1'b1),
      .CQ_USER_WIDTH  (CQ_USER_WIDTH),
      .CC_USER_WIDTH  (CC_USER_WIDTH)
  ) u_rc (
      .clk_i(clk_i),
      .rst_i(rst_i),

      .link_up_i        (phy_link_up_i),
      .transmit_enable_i(transmit_enable_i),
      .fc_initialized_i (fc_init_sticky_r),
      .fc_update_valid_i(dl_fc_update_valid),
      .fc_ph_i  (dl_fc_ph),   .fc_pd_i  (dl_fc_pd),
      .fc_nph_i (dl_fc_nph),  .fc_npd_i (dl_fc_npd),
      .fc_cplh_i(dl_fc_cplh), .fc_cpld_i(dl_fc_cpld),

      .requester_id_i   (requester_id_i),
      .completer_id_i   (completer_id_i),
      .bus_number_i     (bus_number_i),
      .device_number_i  (device_number_i),
      .function_number_i(function_number_i),
      .memory_enable_i  (memory_enable_i),
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

      .s_dllp_axis_tdata (dl_to_tl_tdata),
      .s_dllp_axis_tkeep (dl_to_tl_tkeep),
      .s_dllp_axis_tvalid(dl_to_tl_tvalid),
      .s_dllp_axis_tlast (dl_to_tl_tlast),
      .s_dllp_axis_tuser (dl_to_tl_tuser),
      .s_dllp_axis_tready(dl_to_tl_tready),

      .m_dllp_axis_tdata (tl_to_dl_tdata),
      .m_dllp_axis_tkeep (tl_to_dl_tkeep),
      .m_dllp_axis_tvalid(tl_to_dl_tvalid),
      .m_dllp_axis_tlast (tl_to_dl_tlast),
      .m_dllp_axis_tuser (tl_to_dl_tuser),
      .m_dllp_axis_tready(tl_to_dl_tready),

      // ---- completer interfaces, passed through ---------------------------
      .m_axis_cq_tdata (m_axis_cq_tdata),  .m_axis_cq_tkeep (m_axis_cq_tkeep),
      .m_axis_cq_tvalid(m_axis_cq_tvalid), .m_axis_cq_tlast (m_axis_cq_tlast),
      .m_axis_cq_tuser (m_axis_cq_tuser),  .m_axis_cq_tready(m_axis_cq_tready),
      .s_axis_cc_tdata (s_axis_cc_tdata),  .s_axis_cc_tkeep (s_axis_cc_tkeep),
      .s_axis_cc_tvalid(s_axis_cc_tvalid), .s_axis_cc_tlast (s_axis_cc_tlast),
      .s_axis_cc_tuser (s_axis_cc_tuser),  .s_axis_cc_tready(s_axis_cc_tready),
      .cq_dropped_o    (cq_dropped_o),     .cq_error_code_o (cq_error_code_o),
      .cq_gearbox_error_o(cq_gearbox_error_o),
      .cc_protocol_error_o(cc_protocol_error_o),
      .cc_error_code_o (cc_error_code_o),
      .cc_gearbox_error_o(cc_gearbox_error_o),

      .rq_protocol_error_o(rq_protocol_error_o),
      .rq_error_code_o    (rq_error_code_o),
      .rq_gearbox_error_o (rq_gearbox_error_o),
      .rc_unexpected_completion_o(rc_unexpected_completion_o),
      .rc_completion_error_code_o(rc_completion_error_code_o),
      .rc_protocol_error_o(rc_protocol_error_o),
      .rc_error_code_o    (rc_error_code_o),
      .rc_gearbox_error_o (rc_gearbox_error_o),
      .command_error_valid_o(command_error_valid_o),
      .command_error_code_o (command_error_code_o),
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
      .outstanding_o      (outstanding_o)
  );

  pcie_datalink_layer #(
      .DATA_WIDTH(TL_DATA_WIDTH),
      .STRB_WIDTH(TL_KEEP_WIDTH),
      .KEEP_WIDTH(TL_KEEP_WIDTH),
      .USER_WIDTH(TL_USER_WIDTH)
  ) u_dl (
      .clk_i(clk_i),
      .rst_i(rst_i),

      .s_tlp_axis_tdata (tl_to_dl_tdata),
      .s_tlp_axis_tkeep (tl_to_dl_tkeep),
      .s_tlp_axis_tvalid(tl_to_dl_tvalid),
      .s_tlp_axis_tlast (tl_to_dl_tlast),
      .s_tlp_axis_tuser (tl_to_dl_tuser),
      .s_tlp_axis_tready(tl_to_dl_tready),

      .m_tlp_axis_tdata (dl_to_tl_tdata),
      .m_tlp_axis_tkeep (dl_to_tl_tkeep),
      .m_tlp_axis_tvalid(dl_to_tl_tvalid),
      .m_tlp_axis_tlast (dl_to_tl_tlast),
      .m_tlp_axis_tuser (dl_to_tl_tuser),
      .m_tlp_axis_tready(dl_to_tl_tready),

      .s_phy_axis_tdata (s_phy_axis_tdata),
      .s_phy_axis_tkeep (s_phy_axis_tkeep),
      .s_phy_axis_tvalid(s_phy_axis_tvalid),
      .s_phy_axis_tlast (s_phy_axis_tlast),
      .s_phy_axis_tuser (s_phy_axis_tuser),
      .s_phy_axis_tready(s_phy_axis_tready),
      .m_phy_axis_tdata (m_phy_axis_tdata),
      .m_phy_axis_tkeep (m_phy_axis_tkeep),
      .m_phy_axis_tvalid(m_phy_axis_tvalid),
      .m_phy_axis_tlast (m_phy_axis_tlast),
      .m_phy_axis_tuser (m_phy_axis_tuser),
      .m_phy_axis_tready(m_phy_axis_tready),

      .phy_link_up_i(phy_link_up_i),
      .idle_valid_i (idle_valid_i),

      .fc_initialized_o (dl_fc_initialized),
      .fc_update_valid_o(dl_fc_update_valid),
      .fc_ph_o  (dl_fc_ph),   .fc_pd_o  (dl_fc_pd),
      .fc_nph_o (dl_fc_nph),  .fc_npd_o (dl_fc_npd),
      .fc_cplh_o(dl_fc_cplh), .fc_cpld_o(dl_fc_cpld),

      .cfg_bus_number_o     (cfg_bus_number_o),
      .cfg_device_number_o  (cfg_device_number_o),
      .cfg_function_number_o(cfg_function_number_o),

      // Constant 0 inside pcie_datalink_layer. Connected empty rather than
      // omitted, as pcie_endpoint_top does, so that a missing-pin lint warning
      // still means a real omission.
      .ext_tag_enable_o(),
      .rcb_128b_o(),
      .max_read_request_size_o(),
      .max_payload_size_o(),
      .msix_enable_o(),
      .msix_mask_o(),

      // pcie_datalink_layer does not read these three inputs.
      .status_error_cor_i  (rx_error_valid_o || rx_ecrc_error_o),
      .status_error_uncor_i(tx_error_valid_o || malformed_o),
      .rx_cpl_stall_i      (1'b0),
      // No LTSSM here, so nothing can retrain the link: the request is left
      // open and link_retraining_i keeps its default of 0.
      .link_retrain_req_o  ()
  );

endmodule
