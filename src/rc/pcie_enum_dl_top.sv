// ---------------------------------------------------------------------------
// pcie_enum_dl_top -- the enumeration engine on the RC's TL and DLL stack
//
// Author: Kourosh Ghahramani
// Silicon Systems Research Lab, University of Washington
//
// Purpose
//   Stacks pcie_enum_top on pcie_rc_dl_top. The engine's configuration
//   requests pass through the RC's Transaction Layer and Data Link Layer,
//   which send them only on credit from a real InitFC exchange; the boundary
//   is the PHY-facing stream, where the device being enumerated sits.
//   The only logic of its own is the start gate, which holds a scan start
//   until flow control initialisation is complete.
//
// Interfaces
//   Link          phy_link_up_i, idle_valid_i, transmit_enable_i: to
//                 pcie_rc_dl_top; phy_link_up_i also feeds the start gate.
//   PHY streams   s_phy_axis_*, m_phy_axis_*: to and from the far end.
//   Identity      requester_id_i, completer_id_i, bus_number_i,
//                 device_number_i, function_number_i and the negotiated
//                 limits: inputs. cfg_*_number_o: observation only.
//   Start gate    fc_init_done_o, ok_to_issue_o: from pcie_rc_dl_top.
//   Enumeration   scan_start_i, scan_bus_i, bar_enable_i, bridge_enable_i,
//                 and pcie_enum_top's status and result outputs.
//   TL status     rq_*, rc_*, command_*, rx_*, tx_*, malformed_o,
//                 credit_error_o, vc_overflow_o, cpl_timeout_*, late_cpl_*,
//                 outstanding_o: pcie_rc_dl_top's, forwarded.
//   Completer     m_axis_cq_*, s_axis_cc_*, cq_*, cc_*: forwarded; the engine
//                 does not use them.
//
// Clock and reset
//   clk_i only. rst_i is synchronous and active high, except in
//   pcie_datalink_init inside pcie_rc_dl_top, which resets asynchronously.
//   phy_link_up_i low also clears the start gate, as it clears
//   pcie_rc_dl_top's flow-control state.
//
// Limitations
//   pcie_enum_top is the only master on the RQ socket. MEM_BAR_BASE and
//   MEM_BAR_WINDOW are not parameters here, so pcie_enum_top's defaults
//   apply.
//
// Structure
//   Ports
//   Seam wires
//   Start gate
//   Enumeration engine
//   Transaction and Data Link Layers
//
// References
//   PCIe Base Spec r2.1, §2.2.6.2
//   PCIe Base Spec r2.1, §3.3.1
// ---------------------------------------------------------------------------
module pcie_enum_dl_top
  import tlp_pkg::*;
  import pcie_rq_rc_pkg::*;
  import pcie_enum_pkg::*;
#(
    // ---- passed to both children ------------------------------------------
    parameter int AXIS_DATA_WIDTH = 128,
    parameter int AXIS_USER_WIDTH = 60,
    // One parameter for both children. tlp_request_tracker, inside
    // pcie_rc_dl_top, runs the completion timeout with it; pcie_cfg_txn uses
    // its copy only in an elaboration check against the CRS settings, which
    // therefore sees the value the timer uses.
    parameter int unsigned CPL_TIMEOUT_CYCLES = tlp_pkg::CPL_TIMEOUT_DEFAULT_CYCLES,

    // ---- pcie_rc_dl_top only ----------------------------------------------
    parameter int TAG_COUNT = 32,

    // ---- pcie_enum_top only ------------------------------------------------
    // 3 * 8 = 24 is far below the default CPL_TIMEOUT_CYCLES, so
    // pcie_cfg_txn's elaboration check stays silent. The tb/rc benches that
    // put the engine on pcie_rq_rc_top use the same pair.
    parameter int unsigned CRS_RETRY_MAX      = 3,
    parameter int unsigned CRS_BACKOFF_CYCLES = 8,

    // ---- pcie_rc_dl_top only: PG213 completer tuser widths -----------------
    parameter int CQ_USER_WIDTH   = 88,
    parameter int CC_USER_WIDTH   = 33
) (
    input  logic                        clk_i,
    input  logic                        rst_i,

    // ---- link state, to pcie_rc_dl_top --------------------------------------
    input  logic                        phy_link_up_i,
    input  logic                        idle_valid_i,
    input  logic                        transmit_enable_i,

    // ---- PHY-facing streams: the far end ------------------------------------
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
    // A Root Complex assigns its own Bus and Device Numbers instead of
    // capturing them from a received Configuration Write (PCIe Base Spec
    // r2.1, §2.2.6.2), so the identity ports are inputs.
    // Enumeration does not feed them: scan_bus_i is the bus to probe and
    // SEC_BUS_NUMBER is a bridge's secondary bus, and neither is this port's
    // own identity.
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

    // ---- DLL-assigned identity, observation only ----------------------------
    output logic [7:0]                  cfg_bus_number_o,
    output logic [4:0]                  cfg_device_number_o,
    output logic [2:0]                  cfg_function_number_o,

    // ---- start-gate status, forwarded from pcie_rc_dl_top -------------------
    // fc_init_done_o is also this module's start-gate input. ok_to_issue_o
    // carries the standing terms of the Transaction Layer's transmit gate
    // (FC init done, transmit_enable_i, phy_link_up_i) but not credit
    // availability; pcie_rc_dl_top's port comment says why.
    output logic                        fc_init_done_o,
    output logic                        ok_to_issue_o,

    // ---- enumeration control ------------------------------------------------
    // scan_start_i is a command, not a pulse train: a start requested before
    // flow control has initialised is held and taken when it has. A level or
    // a one-cycle pulse both work; see the start gate below.
    input  logic                        scan_start_i,
    input  logic [7:0]                  scan_bus_i,
    input  logic                        bar_enable_i,
    input  logic                        bridge_enable_i,

    // ---- enumeration status: presence phase ---------------------------------
    output logic                        scan_busy_o,
    output logic                        scan_done_o,
    output logic                        scan_error_o,
    output enum_error_e                 scan_error_code_o,
    output logic                        err_credit_blocked_o,
    output logic                        device_present_o,
    output logic                        unsupported_device_o,
    output logic [15:0]                 device_bdf_o,
    output logic [15:0]                 vendor_id_o,
    output logic [15:0]                 device_id_o,
    output logic [7:0]                  header_type_o,
    output logic                        multifunction_o,

    // ---- enumeration status: BAR phase --------------------------------------
    output logic                        bar_busy_o,
    output logic                        enum_done_o,
    output logic                        enum_error_o,
    output enum_error_e                 enum_error_code_o,
    output logic [3:0]                  bar_count_o,
    output logic [BAR_SLOTS-1:0]        bar_valid_o,
    output logic [BAR_SLOTS-1:0]        bar_is_64_o,
    output logic [BAR_SLOTS-1:0]        bar_prefetch_o,
    output logic [BAR_SLOTS*64-1:0]     bar_size_o,
    output logic [BAR_SLOTS*64-1:0]     bar_addr_o,
    output logic [BAR_SLOTS-1:0]        io_bar_mask_o,

    // ---- enumeration status: bridge path, second bus level ------------------
    output logic                        bus_done_o,
    output logic                        bus_bypassed_o,
    output logic                        sec_scan_done_o,
    output logic                        sec_device_present_o,
    output logic                        sec_unsupported_device_o,
    output logic [15:0]                 sec_device_bdf_o,
    output logic [15:0]                 sec_vendor_id_o,
    output logic [15:0]                 sec_device_id_o,
    output logic [7:0]                  sec_header_type_o,
    output logic                        sec_multifunction_o,
    output logic                        sec_enum_done_o,
    output logic [3:0]                  sec_bar_count_o,
    output logic [BAR_SLOTS-1:0]        sec_bar_valid_o,
    output logic [BAR_SLOTS-1:0]        sec_bar_is_64_o,
    output logic [BAR_SLOTS-1:0]        sec_bar_prefetch_o,
    output logic [BAR_SLOTS*64-1:0]     sec_bar_size_o,
    output logic [BAR_SLOTS*64-1:0]     sec_bar_addr_o,
    output logic [BAR_SLOTS-1:0]        sec_io_bar_mask_o,

    // ---- RQ / RC / TL error and status surface, forwarded verbatim ----------
    // tx_fc_blocked_o, cpl_timeout_valid_o and cpl_timeout_tag_o also feed
    // pcie_enum_top inside this module.
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
    // Carried straight through from pcie_rc_dl_top. Enumeration does not use
    // the completer side: pcie_enum_top issues Configuration Requests and
    // consumes their Completions.
    output logic [AXIS_DATA_WIDTH-1:0]  m_axis_cq_tdata,
    output logic [AXIS_DATA_WIDTH/32-1:0] m_axis_cq_tkeep,
    output logic                        m_axis_cq_tvalid,
    output logic                        m_axis_cq_tlast,
    output logic [CQ_USER_WIDTH-1:0]    m_axis_cq_tuser,
    input  logic                        m_axis_cq_tready,

    input  logic [AXIS_DATA_WIDTH-1:0]  s_axis_cc_tdata,
    input  logic [AXIS_DATA_WIDTH/32-1:0] s_axis_cc_tkeep,
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

  // Derived, not a parameter. Both children default their AXIS_KEEP_WIDTH to
  // AXIS_DATA_WIDTH / 32, and a localparam leaves no way to give them
  // different keep widths.
  localparam int AXIS_KEEP_WIDTH = AXIS_DATA_WIDTH / 32;

  // -------------------------------------------------------------------------
  // Seam wires
  // -------------------------------------------------------------------------
  // The RQ, tag and RC connections between pcie_enum_top and pcie_rc_dl_top,
  // port to port with no adaptor. tx_fc_blocked_o and the cpl_timeout_*
  // outputs also cross the seam, through this module's output ports.
  logic [AXIS_DATA_WIDTH-1:0] rq_tdata;
  logic [AXIS_KEEP_WIDTH-1:0] rq_tkeep;
  logic                       rq_tvalid;
  logic                       rq_tlast;
  logic [AXIS_USER_WIDTH-1:0] rq_tuser;
  logic                       rq_tready;

  logic [7:0]                 rq_tag;
  logic                       rq_tag_vld;

  logic [AXIS_DATA_WIDTH-1:0] rc_tdata;
  logic [AXIS_KEEP_WIDTH-1:0] rc_tkeep;
  logic                       rc_tvalid;
  logic                       rc_tlast;
  logic                       rc_tready;

  // -------------------------------------------------------------------------
  // Start gate
  // -------------------------------------------------------------------------
  // The scan starts only once fc_init_done_o is high: until flow control
  // initialisation completes, the Transaction Layer may not transmit TLPs
  // (PCIe Base Spec r2.1, §3.3.1). An earlier request would still get a tag,
  // because tlp_request_tracker allocates before the credit gate, and its
  // completion timer runs from that allocation until the request is handed
  // to the Data Link Layer, so a request held at the credit gate for the
  // whole timeout interval would time out without being transmitted. Only
  // this start is gated. The second bus level starts from bus_done_o, which
  // follows a completed transaction, so flow control is initialised by then.

  // start_pending_r holds a start requested while the gate is shut, so a
  // one-cycle scan_start_i is delayed rather than lost: pcie_enum_scan reads
  // scan_start_i only in S_IDLE and keeps no record of a start it missed.
  // Reset or link-down clears it, as they clear the FC-init state it waits
  // for, so a request does not carry over into the next link-up.
  logic start_pending_r;
  always_ff @(posedge clk_i) begin
    if (rst_i || !phy_link_up_i) start_pending_r <= 1'b0;
    // Release before set: a request in the cycle the gate opens passes
    // straight through scan_start_gated and is not latched. The other order
    // would hold scan_start_gated high one cycle longer, after pcie_enum_scan
    // has left S_IDLE; it does not return there before reset, so the two
    // orders behave the same.
    else if (fc_init_done_o)     start_pending_r <= 1'b0;
    else if (scan_start_i)       start_pending_r <= 1'b1;
  end

  logic scan_start_gated;
  assign scan_start_gated = fc_init_done_o && (scan_start_i || start_pending_r);

  // -------------------------------------------------------------------------
  // Enumeration engine
  // -------------------------------------------------------------------------
  // pcie_enum_top, the only master on the RQ socket here. pcie_rc_top, which
  // instantiates pcie_enum_top directly rather than this module, hands the
  // socket to an external requester after enum_done_o.
  pcie_enum_top #(
      .AXIS_DATA_WIDTH   (AXIS_DATA_WIDTH),
      .AXIS_KEEP_WIDTH   (AXIS_KEEP_WIDTH),
      .AXIS_USER_WIDTH   (AXIS_USER_WIDTH),
      .CRS_RETRY_MAX     (CRS_RETRY_MAX),
      .CRS_BACKOFF_CYCLES(CRS_BACKOFF_CYCLES),
      .CPL_TIMEOUT_CYCLES(CPL_TIMEOUT_CYCLES)
  ) u_enum (
      .clk_i(clk_i),
      .rst_i(rst_i),

      .scan_start_i   (scan_start_gated),
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

      .bar_busy_o       (bar_busy_o),
      .enum_done_o      (enum_done_o),
      .enum_error_o     (enum_error_o),
      .enum_error_code_o(enum_error_code_o),
      .bar_count_o      (bar_count_o),
      .bar_valid_o      (bar_valid_o),
      .bar_is_64_o      (bar_is_64_o),
      .bar_prefetch_o   (bar_prefetch_o),
      .bar_size_o       (bar_size_o),
      .bar_addr_o       (bar_addr_o),
      .io_bar_mask_o    (io_bar_mask_o),

      .bus_done_o              (bus_done_o),
      .bus_bypassed_o          (bus_bypassed_o),
      .sec_scan_done_o         (sec_scan_done_o),
      .sec_device_present_o    (sec_device_present_o),
      .sec_unsupported_device_o(sec_unsupported_device_o),
      .sec_device_bdf_o        (sec_device_bdf_o),
      .sec_vendor_id_o         (sec_vendor_id_o),
      .sec_device_id_o         (sec_device_id_o),
      .sec_header_type_o       (sec_header_type_o),
      .sec_multifunction_o     (sec_multifunction_o),
      .sec_enum_done_o         (sec_enum_done_o),
      .sec_bar_count_o         (sec_bar_count_o),
      .sec_bar_valid_o         (sec_bar_valid_o),
      .sec_bar_is_64_o         (sec_bar_is_64_o),
      .sec_bar_prefetch_o      (sec_bar_prefetch_o),
      .sec_bar_size_o          (sec_bar_size_o),
      .sec_bar_addr_o          (sec_bar_addr_o),
      .sec_io_bar_mask_o       (sec_io_bar_mask_o),

      // An annotation only, not control flow (see pcie_enum_top).
      .tx_fc_blocked_i(tx_fc_blocked_o),

      .s_axis_rq_tdata_o (rq_tdata),
      .s_axis_rq_tkeep_o (rq_tkeep),
      .s_axis_rq_tvalid_o(rq_tvalid),
      .s_axis_rq_tlast_o (rq_tlast),
      .s_axis_rq_tuser_o (rq_tuser),
      .s_axis_rq_tready_i(rq_tready),

      .pcie_rq_tag_i    (rq_tag),
      .pcie_rq_tag_vld_i(rq_tag_vld),

      .m_axis_rc_tdata_i (rc_tdata),
      .m_axis_rc_tkeep_i (rc_tkeep),
      .m_axis_rc_tvalid_i(rc_tvalid),
      .m_axis_rc_tlast_i (rc_tlast),
      .m_axis_rc_tready_o(rc_tready),

      .cpl_timeout_valid_i(cpl_timeout_valid_o),
      .cpl_timeout_tag_i  (cpl_timeout_tag_o)
  );

  // -------------------------------------------------------------------------
  // Transaction and Data Link Layers
  // -------------------------------------------------------------------------
  // pcie_rc_dl_top: pcie_rq_rc_top above pcie_datalink_layer. Its RQ, tag and
  // RC ports face pcie_enum_top across the seam; everything else goes to this
  // module's ports.
  pcie_rc_dl_top #(
      .AXIS_DATA_WIDTH   (AXIS_DATA_WIDTH),
      .AXIS_KEEP_WIDTH   (AXIS_KEEP_WIDTH),
      .AXIS_USER_WIDTH   (AXIS_USER_WIDTH),
      .TAG_COUNT         (TAG_COUNT),
      .CPL_TIMEOUT_CYCLES(CPL_TIMEOUT_CYCLES),
      .CQ_USER_WIDTH     (CQ_USER_WIDTH),
      .CC_USER_WIDTH     (CC_USER_WIDTH)
  ) u_rcdl (
      .clk_i(clk_i),
      .rst_i(rst_i),

      // ---- completer surface, carried straight out -----------------------
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

      .phy_link_up_i    (phy_link_up_i),
      .idle_valid_i     (idle_valid_i),
      .transmit_enable_i(transmit_enable_i),

      .fc_init_done_o(fc_init_done_o),
      .ok_to_issue_o (ok_to_issue_o),

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

      .s_axis_rq_tdata (rq_tdata),
      .s_axis_rq_tkeep (rq_tkeep),
      .s_axis_rq_tvalid(rq_tvalid),
      .s_axis_rq_tlast (rq_tlast),
      .s_axis_rq_tuser (rq_tuser),
      .s_axis_rq_tready(rq_tready),

      .pcie_rq_tag_o    (rq_tag),
      .pcie_rq_tag_vld_o(rq_tag_vld),

      .m_axis_rc_tdata (rc_tdata),
      .m_axis_rc_tkeep (rc_tkeep),
      .m_axis_rc_tvalid(rc_tvalid),
      .m_axis_rc_tlast (rc_tlast),
      .m_axis_rc_tready(rc_tready),

      .cfg_bus_number_o     (cfg_bus_number_o),
      .cfg_device_number_o  (cfg_device_number_o),
      .cfg_function_number_o(cfg_function_number_o),

      .rq_protocol_error_o       (rq_protocol_error_o),
      .rq_error_code_o           (rq_error_code_o),
      .rq_gearbox_error_o        (rq_gearbox_error_o),
      .rc_unexpected_completion_o(rc_unexpected_completion_o),
      .rc_completion_error_code_o(rc_completion_error_code_o),
      .rc_protocol_error_o       (rc_protocol_error_o),
      .rc_error_code_o           (rc_error_code_o),
      .rc_gearbox_error_o        (rc_gearbox_error_o),
      .command_error_valid_o     (command_error_valid_o),
      .command_error_code_o      (command_error_code_o),
      .malformed_o               (malformed_o),
      .rx_error_valid_o          (rx_error_valid_o),
      .rx_error_code_o           (rx_error_code_o),
      .rx_ecrc_error_o           (rx_ecrc_error_o),
      .tx_error_valid_o          (tx_error_valid_o),
      .tx_error_code_o           (tx_error_code_o),
      .tx_fc_blocked_o           (tx_fc_blocked_o),
      .credit_error_o            (credit_error_o),
      .vc_overflow_o             (vc_overflow_o),
      .cpl_timeout_valid_o       (cpl_timeout_valid_o),
      .cpl_timeout_tag_o         (cpl_timeout_tag_o),
      .late_cpl_valid_o          (late_cpl_valid_o),
      .late_cpl_tag_o            (late_cpl_tag_o),
      .outstanding_o             (outstanding_o)
  );

endmodule
