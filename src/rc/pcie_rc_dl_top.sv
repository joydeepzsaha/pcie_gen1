// ---------------------------------------------------------------------------
// pcie_rc_dl_top -- the RC-side Transaction Layer stacked on the Data Link
// Layer: pcie_rq_rc_top (u_rc) above pcie_datalink_layer (u_dl).
//
// This is the first netlist in which the RC's s_dllp_axis_*/m_dllp_axis_* seam
// is closed by real RTL instead of a Python bench, and the first in which the
// RC's flow-control credit is produced by a real InitFC exchange.  Design
// record: ~/pcie_docs/evidence/rc-integration-top/DESIGN_RC_INTEGRATION_TOP.md.
//
// u_rc is instantiated with PCIE_WIRE_ORDER = 1'b1: the DLL carries TLPs in
// PCIe wire byte order (pcie_endpoint_top.sv:180 is the precedent), while the
// parameter's default 1'b0 serves the Dword-speaking RC benches
// (tlp_layer.sv:13; swap sites tlp_parser.sv:58-60, tlp_generator.sv:97-101).
//
// Identity is NOT taken from the DLL's cfg_*_number_o the way
// pcie_endpoint_top:166-170 does: those registers hold what a RECEIVED
// configuration write assigned -- an endpoint concept.  A Root Complex's
// Requester ID is its own BDF, so requester_id_i/completer_id_i and
// bus/device/function_number_i stay top-level inputs and the three
// cfg_*_number_o are exposed for observation only.
//
// The DLL's six config-space outputs (ext_tag_enable_o .. msix_mask_o) are
// hardwired '0 inside the DLL (pcie_datalink_layer.sv:407-412); they are left
// named-empty here exactly as pcie_endpoint_top:355-360 does, and the
// negotiated limits enter as top-level inputs instead.
//
// rx_cpl_stall_i is tied 0: pcie_rq_rc_top consumes the received completion
// internally into pcie_rc_if and exposes no equivalent of
// received_completion_ready_i.  A known under-report, recorded in the design
// record SS3.4, not a signal to invent here.
//
// Completer path (CQ/CC): tied off inside pcie_rq_rc_top -- Stage F.  A MemRd
// arriving from the far end is accepted and discarded (pcie_rq_rc_top.sv:143-150);
// this top must not sit opposite a DMA-capable endpoint.
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
    parameter int unsigned CPL_TIMEOUT_CYCLES = 32'd4096
) (
    input  logic                        clk_i,
    input  logic                        rst_i,

    // ---- link state ---------------------------------------------------------
    // phy_link_up_i doubles as the link-scoped reset: it drives u_rc.link_up_i
    // (so layer_reset clears the credit pools, tlp_layer.sv:225), the DLL's
    // soft_reset (pcie_datalink_init.sv:76,86,97,108) and the FC-init filter
    // below -- all three fall together on a link-down.
    input  logic                        phy_link_up_i,
    input  logic                        idle_valid_i,
    input  logic                        transmit_enable_i,

    // ---- start-gate status, outward (shape (iii)) ---------------------------
    // Exposing these is the whole point of this rung: an integrator above this
    // module could previously learn when it was safe to issue only by reaching
    // hierarchically into fc_init_sticky_r, which pcie_enum_dl_top.sv:22-25
    // recorded as the blocker that made an RTL start gate impossible.
    //
    // fc_init_done_o -- the FILTERED FC-init state, i.e. the sticky bit below,
    // not the DLL's raw fc_initialized_o.  Monotonic within a link-up.
    //
    // ok_to_issue_o -- the standing preconditions for transmission.  The actual
    // parking decision is tlp_layer.sv:280,
    //     vc_packet_ready = credit_request_ready && transmit_enable_i && link_up_i
    // which is FOUR terms; this port carries the three that are state.  The
    // fourth, credit_request_ready, is deliberately excluded: it resolves to
    // fc_initialized_i && selected_header_available && selected_data_available
    // (tlp_credit_manager.sv:184), and those two availability terms are assigned
    // inside a unique case (request_class_i) and compared against
    // request_data_credits_i (:155-172).  They are a function of the request
    // CURRENTLY PRESENTED, not of module state, so there is no class-independent
    // "credit is ready" bit to export -- publishing one would mean picking a
    // request class here, i.e. a second copy of a decision that already has an
    // owner.  A consumer needing the credit view should use tx_fc_blocked_o,
    // which is NOT this port's inverse: it is request-qualified
    // (tlp_credit_manager.sv:186) and therefore reads 0 while idle.
    //
    // phy_link_up_i is not redundant with the sticky bit.  fc_init_sticky_r is
    // registered, so it still reads 1 for the cycle after a link drop, while the
    // :280 gate it mirrors is combinational.
    output logic                        fc_init_done_o,
    output logic                        ok_to_issue_o,

    // ---- PHY-facing streams (pcie_datalink_layer.sv:48-63,72) ---------------
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

    // ---- identity and negotiated limits (see header) ------------------------
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

    // ---- DLL-assigned identity, observation only (see header) ---------------
    output logic [7:0]                  cfg_bus_number_o,
    output logic [4:0]                  cfg_device_number_o,
    output logic [2:0]                  cfg_function_number_o,

    // ---- RQ / RC / TL error and status surfaces (pcie_rq_rc_top.sv:317-361) -
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
    output logic [$clog2(TAG_COUNT+1)-1:0] outstanding_o
);

  // The TL<->DLL seam is fixed at the DLL's native 32-bit Dword-serial shape.
  localparam int TL_DATA_WIDTH = 32;
  localparam int TL_KEEP_WIDTH = 4;
  localparam int TL_USER_WIDTH = 3;

  // -------------------------------------------------------------------------
  // Seam wires.  tuser is inert in both directions -- outbound the DLL
  // overwrites it (tlp2dllp.sv:263), inbound tlp_parser never reads it -- but
  // both sides declare it, so it is carried for AXIS shape.
  // -------------------------------------------------------------------------
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

  // fc_initialized_o glitches 1->0->1 while pcie_flow_ctrl_init walks ST_UPDATE_P ..
  // ST_UPDATE_NP_CRC (pcie_flow_ctrl_init.sv:355-400), where fc2_values_sent_o falls back
  // to its combinational default (:144).  The window is four STATES, each gated on
  // fc_axis_tready (:363,374,386,397) -- it is not bounded at four cycles.
  //
  // Base 2.1 SS3.3.1 p.160: for VC0, FC_INIT1 is entered only on "Entrance to DL_Init
  // state".  FC initialisation completes once per link-up and is not re-entered without
  // a link-down, so a sticky bit cleared by phy_link_up_i cannot mask a legitimate
  // re-initialisation -- verified against pcie_datalink_init.sv:76,86,97,108, where every
  // soft_reset assertion is guarded by !phy_link_up_i.
  logic fc_init_sticky_r;
  always_ff @(posedge clk_i) begin
    if (rst_i || !phy_link_up_i) fc_init_sticky_r <= 1'b0;
    else if (dl_fc_initialized)  fc_init_sticky_r <= 1'b1;
  end

  // Outward view of the two states above.  Continuous assigns from signals that
  // already exist and already drive u_rc -- fc_init_sticky_r remains the single
  // source of truth for FC-init, and these ports read it rather than recompute
  // it.  See the port declarations for why ok_to_issue_o carries three of the
  // four conjuncts of tlp_layer.sv:280 and not the fourth.
  assign fc_init_done_o = fc_init_sticky_r;
  assign ok_to_issue_o  = fc_init_sticky_r && transmit_enable_i && phy_link_up_i;

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
      .PCIE_WIRE_ORDER(1'b1)
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

      // Hardwired '0 inside the DLL (pcie_datalink_layer.sv:407-412); named-
      // empty rather than omitted so PINMISSING stays enabled for real
      // omissions, per pcie_endpoint_top:355-360.
      .ext_tag_enable_o(),
      .rcb_128b_o(),
      .max_read_request_size_o(),
      .max_payload_size_o(),
      .msix_enable_o(),
      .msix_mask_o(),

      .status_error_cor_i  (rx_error_valid_o || rx_ecrc_error_o),
      .status_error_uncor_i(tx_error_valid_o || malformed_o),
      .rx_cpl_stall_i      (1'b0)
  );

endmodule
