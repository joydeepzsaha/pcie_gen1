// ===========================================================================
// pcie_enum_dl_top -- the enumeration engine stacked on the RC's TL+DLL stack.
//
// pcie_enum_top issues configuration requests; pcie_rc_dl_top frames them,
// numbers them, LCRCs them and gates them on credit that came from a real
// InitFC exchange.  This is the first netlist in which NO PYTHON SITS BETWEEN
// THE ENUMERATOR AND THE WIRE -- the Python that remains is the far end, the
// device being enumerated, which is where a model belongs.
//
// SS SHAPE: WIRING, PLUS ONE BIT OF STATE.  Two instantiations, sixteen
// internal wires, and exactly one register -- start_pending_r, the start gate
// below.  Neither child changes.  This module was PURE wiring until the start
// gate landed; the design record justifies the original choice rather than
// defaulting into it (~/pcie_docs/evidence/enum-stack/DESIGN_ENUM_STACK_TOP.md
// SS2), and ~/pcie_docs/evidence/start-gate/ records why the one register had
// to be added here rather than in either child.
//
// SS THE START GATE.  scan_start_i IS GATED HERE, AND THIS IS THE WHOLE RUNG.
// The correct start condition is NOT phy_link_up_i.  It is the Transaction
// Layer's filtered view of flow-control initialisation -- Base 2.1 SS3.3.1
// p.160, quoted in pcie_rc_dl_top.sv:176-177: for VC0 the FC_INIT sequence is
// entered on entrance to DL_Init and completes once per link-up, and a
// transmitter holds no credit until it does.
//
// This block previously said that signal "IS NOT ON ITS PORT LIST", and
// deferred the gate to a later rung on that ground.  It is on the port list
// now: pcie_rc_dl_top exposes fc_init_done_o, and this module consumes it.
//
// !! THE HAZARD THE GATE CLOSES.  Tag allocation sits UPSTREAM of the credit
// gate (pcie_enum_scan.sv:137-144) and the completion timer measures from
// ALLOCATION (tlp_request_tracker.sv:39).  A scan_start_i that fires on
// link-up alone produces a request that is tagged, parked in the VC buffer,
// and TIMES OUT HAVING NEVER BEEN TRANSMITTED.  The enumerator cannot see it
// coming: it reads neither outstanding_o nor any tag, and that is structural,
// not an oversight (pcie_enum_scan.sv:145-150).  Measured before the gate
// existed: no frame in 4000 cycles, ENUM_ERR_TIMEOUT at 33116 ns.
//
// !! WHY A LATCH AND NOT JUST AN AND GATE.  A bare
// scan_start_i && fc_init_done_o would ANNIHILATE a start request that arrives
// early rather than delay it, because a requester is entitled to PULSE the
// start -- and the bench does exactly that (test_pcie_enum_dl_top.py:296-302,
// one cycle high).  The engine's start is a command, not a pulse train: a
// request made before the gate opens must take effect WHEN it opens, not be
// lost.  pcie_enum_scan re-samples scan_start_i every cycle it sits in S_IDLE
// (pcie_enum_scan.sv:346), so it will accept the release whenever it comes --
// but nothing in the engine REMEMBERS a request that was masked away, which is
// why the memory has to live here.
//
// ONLY THE OUTER START IS GATED.  pcie_enum_top chains its second bus level
// from bus_done_o (pcie_enum_top.sv:490); that path is downstream of a scan
// that has already run, so ANDing FC-init into it would gate a condition
// already implied and could only stall multi-bus traversal.
//
// SS IDENTITY.  A Root Complex's Requester ID is its own BDF, fixed at 00:00.0
// for the whole run, so requester_id_i / completer_id_i / bus_number_i /
// device_number_i / function_number_i stay top-level inputs and the DLL's
// cfg_*_number_o stay observation-only -- pcie_rc_dl_top.sv:17-20.  ENUM'S BUS
// ASSIGNMENT DOES NOT FEED BACK: scan_bus_i names the bus to PROBE, and the bus
// number enum writes (register 18h) is a bridge's SECONDARY bus.  Neither
// describes this port's own BDF, and routing either into bus_number_i would
// corrupt the Requester ID of every subsequent request header.
//
// SS SCOPE.  Direct-attach (Type 0) enumeration.  bridge_enable_i is a real
// port and is forwarded, but the bridged flow needs a second bus level behind
// the DLL and is a later rung (DESIGN SS9 D2).  The completer path (CQ/CC) is
// tied off inside pcie_rq_rc_top (pcie_rc_dl_top.sv:32-34), so ECRC follows it
// out of scope.  The PG213 RQ/RC AXIS socket DISAPPEARS from the surface --
// that is the point: pcie_enum_top is the only master.
// ===========================================================================
module pcie_enum_dl_top
  import tlp_pkg::*;
  import pcie_rq_rc_pkg::*;
  import pcie_enum_pkg::*;
#(
    // ---- shared by BOTH children: one name each, passed to both -----------
    parameter int AXIS_DATA_WIDTH = 128,
    parameter int AXIS_USER_WIDTH = 60,
    // !! CPL_TIMEOUT_CYCLES IS THE ONE THAT MATTERS.  pcie_cfg_txn's copy arms
    // NO COUNTER -- its only use is the elaboration-time P-CRS-BUDGET guard at
    // pcie_cfg_txn.sv:222-225 -- while the timer that actually fires is
    // tlp_request_tracker's, configured from pcie_rc_dl_top's copy.  Passing
    // different values to the two children would leave the guard silently
    // checking a number no timer uses, and a slow device would be misreported
    // as dead with no warning.  ONE NAME FEEDS BOTH.  (DESIGN SS3.4, SSC5.)
    parameter int unsigned CPL_TIMEOUT_CYCLES = 32'd4096,

    // ---- pcie_rc_dl_top only ----------------------------------------------
    parameter int TAG_COUNT = 32,

    // ---- pcie_enum_top only ------------------------------------------------
    // 3 * 8 = 24 < CPL_TIMEOUT_CYCLES, so the P-CRS-BUDGET guard is satisfied.
    // Matches the three seam benches (tb_pcie_enum_bridge_tlp.sv:37-38).
    parameter int unsigned CRS_RETRY_MAX      = 3,
    parameter int unsigned CRS_BACKOFF_CYCLES = 8,

    // ---- pcie_rc_dl_top only: PG213 tuser widths (Stage F-3) ---------------
    parameter int CQ_USER_WIDTH   = 88,
    parameter int CC_USER_WIDTH   = 33
) (
    input  logic                        clk_i,
    input  logic                        rst_i,

    // ---- link state (pcie_rc_dl_top.sv:57-59) -------------------------------
    input  logic                        phy_link_up_i,
    input  logic                        idle_valid_i,
    input  logic                        transmit_enable_i,

    // ---- PHY-facing streams -- the far end sits here ------------------------
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

    // ---- identity and negotiated limits (see SS IDENTITY above) -------------
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
    // Both are pass-throughs, exposed because hardware wants them: a status LED
    // or an ILA probe answering "why is nothing happening?" without a
    // hierarchical reach.  fc_init_done_o is also what this module's own start
    // gate runs on, so probing it shows the gate's input, not a copy of it.
    // ok_to_issue_o carries three of the four conjuncts of the real parking
    // decision; pcie_rc_dl_top's port declaration explains which one is left
    // out and why.
    output logic                        fc_init_done_o,
    output logic                        ok_to_issue_o,

    // ---- enumeration control ------------------------------------------------
    // scan_start_i: a COMMAND, not a pulse train.  Assert it whenever you want
    // the scan to run; if flow control has not initialised yet the request is
    // held and honoured when it does.  Level or single-cycle pulse both work.
    // See the start-gate block in the header.
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

    // ---- enumeration status: bridge path, second bus level (Stage D) --------
    // Exposed even though this rung drives bridge_enable_i low from the bench:
    // leaving a real output unconnected costs a PINMISSING waiver, and this
    // codebase keeps PINMISSING enabled for genuine omissions
    // (pcie_rc_dl_top.sv:326-328).
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
    // Twenty-four outputs.  tx_fc_blocked_o and cpl_timeout_valid_o/_tag_o are
    // ALSO consumed internally across the seam; they are exposed because the
    // bench asserts on them.
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

    // ---- PG213 Completer reQuest / Completer Completion (Stage F-3) ---------
    // Carried straight through from u_rcdl. Enumeration does not use the
    // completer surface itself -- pcie_enum_top issues Configuration requests
    // and consumes Completions -- but a top that hides a child's interface
    // makes the stack unusable for anything else, and this is the netlist a
    // full-stack top will instantiate.
    //
    // !! DECLARED, NOT YET DRIVEN, because u_rcdl's copies are not either: the
    // pass-through is real from this commit, the SOURCE is constant until the
    // wiring commit. That ordering is deliberate -- when u_rcdl starts driving
    // them this module needs no further change.
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

  // AXIS_KEEP_WIDTH is DERIVED, not a parameter.  Both children default it to
  // AXIS_DATA_WIDTH/32; making it a localparam here removes the only way the
  // two sides could be given inconsistent keep widths -- an override that set
  // it on one child and not the other.
  localparam int AXIS_KEEP_WIDTH = AXIS_DATA_WIDTH / 32;

  // =========================================================================
  // The seam: sixteen wires, all ports on both sides, no adaptor.
  // Every row matched in width and opposite in direction before this file was
  // written -- RECON_REFRESH_588f634.md SS2.1.
  // =========================================================================
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

  // =========================================================================
  // THE START GATE.  See the header block for the spec argument and for why
  // this is a latch rather than a bare AND.
  //
  // start_pending_r remembers a start requested while the gate was shut, so a
  // single-cycle scan_start_i is DELAYED rather than lost.  Cleared by the same
  // condition that clears the FC-init state it waits on -- reset or link-down
  // (pcie_rc_dl_top.sv:183) -- so a link that drops mid-wait does not leave a
  // stale request armed for the next link-up.
  //
  // ARM ORDER: the release arm precedes the set arm, so a request arriving in
  // the very cycle the gate opens is passed through by the scan_start_i term of
  // the assign below and is never latched.
  //
  // !! THIS IS DEFENSIVE, NOT LOAD-BEARING, AND THE CENSUS PROVED IT.  This
  // comment previously claimed the reverse order "would fire a SECOND, spurious
  // scan one cycle later".  That is false here: reversing the arms leaves
  // scan_start_gated high for one extra cycle, but pcie_enum_scan's terminal
  // states hold until reset and the FSM never re-enters S_IDLE
  // (pcie_enum_scan.sv:413-419), so nothing can consume the extra cycle.
  // Mutation M5 swapped the arms and all 7 tests still passed -- an EQUIVALENT
  // mutant, not a test gap, and no test was added because there is no
  // behaviour left to observe.  The order is kept because it is the correct one
  // if the engine ever gains a re-arm path; the claim that it mattered TODAY
  // was wrong.
  // =========================================================================
  logic start_pending_r;
  always_ff @(posedge clk_i) begin
    if (rst_i || !phy_link_up_i) start_pending_r <= 1'b0;
    else if (fc_init_done_o)     start_pending_r <= 1'b0;
    else if (scan_start_i)       start_pending_r <= 1'b1;
  end

  logic scan_start_gated;
  assign scan_start_gated = fc_init_done_o && (scan_start_i || start_pending_r);

  // =========================================================================
  // The enumeration engine.  Only master on the RQ socket.
  // =========================================================================
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

      // Annotation, NOT control flow -- pcie_enum_top.sv port comment.
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

  // =========================================================================
  // The Root Complex TL + DLL stack, unmodified.
  // =========================================================================
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

      // ---- completer surface, carried straight out (Stage F-3) -----------
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
