// ---------------------------------------------------------------------------
// pcie_enum_top -- the enumeration assembly. Commit 2b-3.
//
// Wiring only. No policy, no mechanism, no state of its own.
//
// SPEC ANCHORS: none of its own, by construction. Every spec rule this assembly
// obeys is enforced in a module it instantiates -- presence in pcie_enum_scan,
// BAR layout in pcie_enum_bar, bus numbering in pcie_enum_bus, transaction
// mechanics in pcie_cfg_txn. Two constraints do surface here because they are
// properties of the ASSEMBLY rather than of any one stage:
//   PCIe Base 2.1 Table 2-37 ..... only ONE pcie_cfg_txn exists, so
//                                  single-outstanding is a property of the
//                                  netlist and not of an FSM behaving well.
//   [BASE] Figure 7-5 p.491 ...... the Command register's write-1-to-clear
//                                  bits, which a whole-Dword write destroys.
//
//   pcie_enum_top
//   |- pcie_cfg_txn    (ONE instance -- the only tag holder)
//   |- pcie_enum_scan  (presence policy, level 1)
//   |- pcie_enum_bar   (BAR policy, level 1) + the static handoff mux
//   `- [Stage D] pcie_enum_bus + a SECOND scan/BAR pair for the bus behind
//      a discovered bridge, behind the same mux, widened
//
// ===========================================================================
// SS WHY THIS MODULE EXISTS
// ===========================================================================
//
// Before 2b-3, pcie_enum_scan instantiated the primitive itself. The BAR phase
// (pcie_enum_bar, Commit D) needs the same primitive, and the alternatives to
// hoisting were both worse -- extending pcie_enum_scan in place breaks the 16
// scan tests that assert no traffic after scan_done_o, and giving pcie_enum_bar
// its own pcie_cfg_txn duplicates the primitive and destroys
// single-outstanding-by-construction. See pcie_enum_scan's header, SS THE HOIST.
//
// So this module exists to own the primitive on behalf of every stage that
// needs it, and to prove that exactly one stage drives it at a time.
//
// ===========================================================================
// SS SINGLE OUTSTANDING, STRUCTURALLY
// ===========================================================================
//
// There is ONE pcie_cfg_txn here and there will never be a second. Table 2-37
// p.137 permits a peer to advertise a single NPH credit, so a second config
// request could not be transmitted anyway (pcie_cfg_txn's header). Holding one
// instance makes that a property of the netlist rather than of an FSM's good
// behaviour.
//
// ===========================================================================
// SS THE HANDOFF -- STATIC, NOT ARBITRATED
// ===========================================================================
//
//     scan owns the command port until scan_done_o, then bar owns it
//
// One combinational select, no arbiter, no round robin, no request/grant. The
// select is scan_done_o, which is a TERMINAL condition: pcie_enum_scan reaches
// S_DONE or S_UNSUPPORTED and stays there until reset. So ownership transfers
// exactly once per reset and never transfers back.
//
// STAGE D WIDENED THE SAME MUX, IT DID NOT CHANGE ITS NATURE. Five arms now:
//
//     scan1 -> (Type 0 verdict)  bar1                              -- Stage C
//           -> (Type 1 verdict)  enum_bus -> scan2 -> bar2         -- Stage D
//
// Every select input is a TERMINAL level -- scan_done_o, the scan's
// unsupported verdict, bus_done_o, scan2's scan_done_o -- each rises at most
// once per reset and holds, so ownership still transfers exactly once per
// boundary, monotonically, with no arbiter and no registered state of its
// own. (The recon sized this as "4-way"; it is five arms because pcie_enum_bus
// owns the port for its one write between bar1 and scan2 -- same shape,
// one more stop.) !! Per docs/recon/RECON_stageD.md SS11.2 this widened mux is NOT
// load-bearing architecture: it does not iterate, and the sequencing layer is
// expected to be redesigned for the Stage E tree walk.
//
// !! BOTH DIRECTIONS OF EVERY HANDSHAKE ARE GATED, NOT JUST THE FORWARD PATH.
// The loser sees cmd_ready = 0 and rsp_valid = 0. Selecting only the forward
// signals would leave the loser able to complete a handshake against a response
// meant for the winner, which is the shape of bug a "static mux" is supposed to
// make impossible.
//
// ===========================================================================
// SS ⭐ PROVING EXACTLY ONE STAGE IS LIVE -- AND THE FIELD THAT PROVES IT
// ===========================================================================
//
// The Commit-D mutation to kill is "the mux allows both stages to drive the
// command port", i.e. MERGING the two command ports instead of SELECTING one.
//
// Most of the port cannot show the difference, and it is worth writing down
// which parts cannot and why, because it explains why the surviving field is
// the one the bench must assert on:
//
//   cmd_valid   scan drives 0 in every terminal state          -> merge is a no-op
//   cmd_write   scan drives a hard 0 (pcie_enum_scan.sv:282)   -> merge is a no-op
//   cmd_wdata   scan drives a hard 0 (:272)                    -> merge is a no-op
//   cmd_reg_num scan drives 0 in every terminal state          -> merge is a no-op
//   cmd_ext_reg both stages drive the same constant            -> merge is a no-op
//
//   cmd_first_be  scan drives a HARD CONSTANT 1111 (:271), ALWAYS, terminal or
//                 not -- it is not qualified by state at all.
//
// So a merged mux forces first_be to 1111 on every transaction the BAR phase
// issues. Fourteen of the fifteen are 1111 anyway and stay invisible. THE
// FIFTEENTH IS THE COMMAND WRITE, which must be 0011 -- the Command register is
// the lower half of the Dword at offset 04h and the upper half is Status, whose
// write-1-to-clear bits a whole-Dword write would destroy ([BASE] Figure 7-5
// p.491, SSE.6).
//
// !! THEREFORE: THE ON-WIRE ASSERTION THAT THE COMMAND WRITE CARRIES
// !! first_be = 0011 IS THE TEST THAT PROVES EXACTLY ONE STAGE IS LIVE.
// It is not a byte-enable check that happens to be nearby; it is the only
// observable consequence the two constructions do not share.
//
// The Stage D widening preserves the mutation and its kill unchanged: scan2
// is another pcie_enum_scan, so it too drives a HARD 1111 unqualified by
// state, and a merged mux forces 1111 onto every transaction bar2 issues --
// caught by the same fifteenth transaction, the DEVICE's Command write,
// which must carry 0011 on the wire. cmd_type1_i joining the muxed set adds
// a second single-field observable: a merge that ORs the scan2/bar2 arms'
// Type select into a level-1 transaction emits a well-formed CFG1 where the
// golden demands CFG0, one bit of DW0 (Trap A cuts both ways).
//
// ===========================================================================
// SS WHY THE BAR PHASE NEEDS AN ENABLE
// ===========================================================================
//
// bar_enable_i gates the BAR phase, and it exists for a MEASURED reason rather
// than a speculative one.
//
// Both scan targets instantiate THIS module, not pcie_enum_scan -- they have
// since the Commit-B hoist. Three of their tests assert the exact number of
// configuration requests a presence scan emits (test_pcie_enum_scan.py:199,
// :438, :533: "exactly 2", and "no further config requests after scan_done_o").
// Those assertions are correct and they are the scan's own subject. A BAR phase
// that started automatically at scan_done_o would falsify all three, so the 24
// scan tests would have to be rewritten to accommodate a stage they do not test.
//
// Tying bar_enable_i low in the two scan shims keeps them measuring the presence
// scan and nothing else, and keeps the Commit-D gate honest: the scan targets
// must come back byte-identical in count, verdict AND sim end time.
//
// It is not a test hook. An integrator that wants presence detection without
// configuration -- "what is attached?" -- ties it low too, and that is a real
// use. An integrator that wants the whole sequence ties it high and gets
// scan -> bar -> enum_done_o with no further intervention.
//
// ===========================================================================
// SS THE BDF IS NOT MUXED BY PHASE -- IT IS SELECTED BY LEVEL (Stage D form)
// ===========================================================================
//
// cmd_bdf_i is driven from a scan's device_bdf_o, never chosen per phase.
// The target BDF is a property of the DEVICE BEING ENUMERATED, not of the
// phase enumerating it -- the BAR phase configures the same Function its
// scan found, and the bus-number write targets the same Function the first
// scan found (the bridge IS that device).
//
// Stage D restates this per level rather than repealing it (recon SS6.5):
// each scan instance owns its own bus level, so there are now two devices
// being enumerated and two device_bdf_o sources, selected by the same
// terminal ownership levels as the command mux -- scan1's BDF for
// scan1/bar1/enum_bus, scan2's for scan2/bar2. Within a level the BDF is
// still not muxed: no PHASE ever chooses a BDF, and the two stages of one
// level cannot disagree about which device they are talking to.
//
// This also preserves the pre-hoist wiring exactly: pcie_enum_scan used to
// connect cmd_bdf_i to its own device_bdf_o at its instantiation of the
// primitive.
//
// ===========================================================================
// SS WHAT THIS MODULE DELIBERATELY DOES NOT DO
// ===========================================================================
//
// It does not observe pcie_rq_tag_o, decode a completion status, run a timer or
// gate anything on credit. Those belong to the primitive, the tracker and the
// stages respectively. It forwards the pcie_rq_rc_top socket verbatim.
// ---------------------------------------------------------------------------
`timescale 1ns/1ps
module pcie_enum_top
  import pcie_rq_rc_pkg::*;
  import pcie_enum_pkg::*;
#(
    parameter int AXIS_DATA_WIDTH = 128,
    parameter int AXIS_KEEP_WIDTH = AXIS_DATA_WIDTH / 32,
    parameter int AXIS_USER_WIDTH = 60,
    parameter int unsigned CRS_RETRY_MAX      = CRS_RETRY_MAX_DEFAULT,
    parameter int unsigned CRS_BACKOFF_CYCLES = CRS_BACKOFF_CYCLES_DEFAULT,
    parameter int unsigned CPL_TIMEOUT_CYCLES = 32'd6250,
    // Forwarded verbatim to pcie_enum_bar; documented there.
    parameter logic [63:0] MEM_BAR_BASE       = 64'h0000_0000_8000_0000,
    parameter logic [63:0] MEM_BAR_WINDOW     = 64'h0000_0000_1000_0000
) (
    input  logic                        clk_i,
    input  logic                        rst_i,

    // ---- control -----------------------------------------------------------
    input  logic                        scan_start_i,
    input  logic [7:0]                  scan_bus_i,
    // Level. Low means "stop after presence detection" -- see SS WHY THE BAR
    // PHASE NEEDS AN ENABLE. With it low, enum_done_o never asserts and not one
    // BAR-phase transaction is emitted.
    input  logic                        bar_enable_i,
    // Stage D. Level, same rationale as bar_enable_i: with it low the bridge
    // path (pcie_enum_bus + the second scan/BAR pair) never leaves IDLE and
    // not one of its transactions is emitted, which is what keeps every
    // Stage C target byte-identical -- including the scan tests that drive a
    // Type 1 header and assert no traffic after scan_done_o. High, a Type 1
    // discovery continues into bus-number assignment and full configuration
    // of the device on the secondary bus.
    input  logic                        bridge_enable_i,

    // ---- status surface: presence phase ------------------------------------
    output logic                        scan_busy_o,
    output logic                        scan_done_o,
    output logic                        scan_error_o,
    output enum_error_e                 scan_error_code_o,
    // Diagnostic, from WHICHEVER stage reported a timeout. The two stages cannot
    // both error -- the BAR phase starts only from scan_done_o, and a scan that
    // errored never asserts it -- so the OR below has at most one live input.
    output logic                        err_credit_blocked_o,

    output logic                        device_present_o,
    output logic                        unsupported_device_o,
    output logic [15:0]                 device_bdf_o,
    output logic [15:0]                 vendor_id_o,
    output logic [15:0]                 device_id_o,
    output logic [7:0]                  header_type_o,
    output logic                        multifunction_o,

    // ---- status surface: BAR phase -----------------------------------------
    output logic                        bar_busy_o,
    output logic                        enum_done_o,
    // The single "enumeration failed" surface, from either phase. scan_error_o
    // above is retained because it names WHICH phase, and because the 24 scan
    // tests are written against it.
    output logic                        enum_error_o,
    output enum_error_e                 enum_error_code_o,

    output logic [3:0]                  bar_count_o,
    output logic [BAR_SLOTS-1:0]        bar_valid_o,
    output logic [BAR_SLOTS-1:0]        bar_is_64_o,
    output logic [BAR_SLOTS-1:0]        bar_prefetch_o,
    output logic [BAR_SLOTS*64-1:0]     bar_size_o,
    output logic [BAR_SLOTS*64-1:0]     bar_addr_o,
    output logic [BAR_SLOTS-1:0]        io_bar_mask_o,

    // ---- status surface: bridge path, second bus level (Stage D) -----------
    // The first level's outputs above are UNCHANGED; everything the bridge
    // path learns is exposed alongside under a sec_ prefix. Errors from any
    // bridge-path stage fold into enum_error_o / enum_error_code_o below --
    // the pipeline gating still guarantees at most one stage can error.
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

    // ---- annotation input (NOT control flow) -------------------------------
    input  logic                        tx_fc_blocked_i,

    // ---- pcie_rq_rc_top socket: Requester Request --------------------------
    output logic [AXIS_DATA_WIDTH-1:0]  s_axis_rq_tdata_o,
    output logic [AXIS_KEEP_WIDTH-1:0]  s_axis_rq_tkeep_o,
    output logic                        s_axis_rq_tvalid_o,
    output logic                        s_axis_rq_tlast_o,
    output logic [AXIS_USER_WIDTH-1:0]  s_axis_rq_tuser_o,
    input  logic                        s_axis_rq_tready_i,

    // ---- pcie_rq_rc_top socket: core-managed tag ---------------------------
    input  logic [7:0]                  pcie_rq_tag_i,
    input  logic                        pcie_rq_tag_vld_i,

    // ---- pcie_rq_rc_top socket: Requester Completion -----------------------
    input  logic [AXIS_DATA_WIDTH-1:0]  m_axis_rc_tdata_i,
    input  logic [AXIS_KEEP_WIDTH-1:0]  m_axis_rc_tkeep_i,
    input  logic                        m_axis_rc_tvalid_i,
    input  logic                        m_axis_rc_tlast_i,
    output logic                        m_axis_rc_tready_o,

    // ---- pcie_rq_rc_top socket: completion timeout sideband ----------------
    input  logic                        cpl_timeout_valid_i,
    input  logic [7:0]                  cpl_timeout_tag_i
);

  // -------------------------------------------------------------------------
  // The command/response bus that reaches the primitive -- the mux OUTPUT.
  // -------------------------------------------------------------------------
  logic         cmd_valid;
  logic         cmd_ready;
  logic         cmd_write;
  logic [5:0]   cmd_reg_num;
  logic [3:0]   cmd_ext_reg;
  logic [3:0]   cmd_first_be;
  logic [31:0]  cmd_wdata;

  logic         rsp_valid;
  logic         rsp_ready;
  txn_outcome_e rsp_outcome;
  logic [31:0]  rsp_rdata;

  // Per-stage command/response ports -- the mux INPUTS.
  logic         scan_cmd_valid,  bar_cmd_valid;
  logic         scan_cmd_ready,  bar_cmd_ready;
  logic         scan_cmd_write,  bar_cmd_write;
  logic [5:0]   scan_cmd_reg_num, bar_cmd_reg_num;
  logic [3:0]   scan_cmd_ext_reg, bar_cmd_ext_reg;
  logic [3:0]   scan_cmd_first_be, bar_cmd_first_be;
  logic [31:0]  scan_cmd_wdata,  bar_cmd_wdata;

  logic         scan_rsp_valid,  bar_rsp_valid;
  logic         scan_rsp_ready,  bar_rsp_ready;

  logic         scan_credit_blocked, bar_credit_blocked;
  logic         bar_error;
  enum_error_e  scan_error_code, bar_error_code;

  // Stage D: the bridge-path stages' command/response ports.
  logic         bus_cmd_valid,   scan2_cmd_valid,   bar2_cmd_valid;
  logic         bus_cmd_ready,   scan2_cmd_ready,   bar2_cmd_ready;
  logic         bus_cmd_write,   scan2_cmd_write,   bar2_cmd_write;
  logic         bus_cmd_type1;
  logic [5:0]   bus_cmd_reg_num, scan2_cmd_reg_num, bar2_cmd_reg_num;
  logic [3:0]   bus_cmd_ext_reg, scan2_cmd_ext_reg, bar2_cmd_ext_reg;
  logic [3:0]   bus_cmd_first_be, scan2_cmd_first_be, bar2_cmd_first_be;
  logic [31:0]  bus_cmd_wdata,   scan2_cmd_wdata,   bar2_cmd_wdata;

  logic         bus_rsp_valid,   scan2_rsp_valid,   bar2_rsp_valid;
  logic         bus_rsp_ready,   scan2_rsp_ready,   bar2_rsp_ready;

  logic         bus_error,       scan2_error,       bar2_error;
  enum_error_e  bus_error_code,  scan2_error_code,  bar2_error_code;
  logic         bus_credit_blocked, scan2_credit_blocked, bar2_credit_blocked;
  logic [7:0]   bus_sec_bus;
  logic         bus_type1;

  // -------------------------------------------------------------------------
  // Presence phase.
  // -------------------------------------------------------------------------
  pcie_enum_scan u_scan (
      .clk_i(clk_i),
      .rst_i(rst_i),

      .scan_start_i(scan_start_i),
      .scan_bus_i  (scan_bus_i),

      .scan_busy_o         (scan_busy_o),
      .scan_done_o         (scan_done_o),
      .scan_error_o        (scan_error_o),
      .scan_error_code_o   (scan_error_code),
      .err_credit_blocked_o(scan_credit_blocked),

      .device_present_o    (device_present_o),
      .unsupported_device_o(unsupported_device_o),
      .device_bdf_o        (device_bdf_o),
      .vendor_id_o         (vendor_id_o),
      .device_id_o         (device_id_o),
      .header_type_o       (header_type_o),
      .multifunction_o     (multifunction_o),

      .tx_fc_blocked_i(tx_fc_blocked_i),

      .cmd_valid_o   (scan_cmd_valid),
      .cmd_ready_i   (scan_cmd_ready),
      .cmd_write_o   (scan_cmd_write),
      .cmd_reg_num_o (scan_cmd_reg_num),
      .cmd_ext_reg_o (scan_cmd_ext_reg),
      .cmd_first_be_o(scan_cmd_first_be),
      .cmd_wdata_o   (scan_cmd_wdata),

      .rsp_valid_i  (scan_rsp_valid),
      .rsp_ready_o  (scan_rsp_ready),
      .rsp_outcome_i(rsp_outcome),
      .rsp_rdata_i  (rsp_rdata)
  );

  assign scan_error_code_o = scan_error_code;

  // -------------------------------------------------------------------------
  // BAR sizing, assignment and enable phase.
  //
  // device_present_i / unsupported_device_i are the scan's verdict, sampled by
  // pcie_enum_bar in its S_CHECK state. They are stable by then: scan_done_o is
  // terminal, and bar_start_i cannot rise before it.
  // -------------------------------------------------------------------------
  pcie_enum_bar #(
      .MEM_BAR_BASE  (MEM_BAR_BASE),
      .MEM_BAR_WINDOW(MEM_BAR_WINDOW)
  ) u_bar (
      .clk_i(clk_i),
      .rst_i(rst_i),

      .bar_start_i         (bar_enable_i && scan_done_o),
      .device_present_i    (device_present_o),
      .unsupported_device_i(unsupported_device_o),

      .tx_fc_blocked_i(tx_fc_blocked_i),

      .bar_busy_o          (bar_busy_o),
      .enum_done_o         (enum_done_o),
      .bar_error_o         (bar_error),
      .bar_error_code_o    (bar_error_code),
      .err_credit_blocked_o(bar_credit_blocked),

      .bar_count_o   (bar_count_o),
      .bar_valid_o   (bar_valid_o),
      .bar_is_64_o   (bar_is_64_o),
      .bar_prefetch_o(bar_prefetch_o),
      .bar_size_o    (bar_size_o),
      .bar_addr_o    (bar_addr_o),
      .io_bar_mask_o (io_bar_mask_o),

      .cmd_valid_o   (bar_cmd_valid),
      .cmd_ready_i   (bar_cmd_ready),
      .cmd_write_o   (bar_cmd_write),
      .cmd_reg_num_o (bar_cmd_reg_num),
      .cmd_ext_reg_o (bar_cmd_ext_reg),
      .cmd_first_be_o(bar_cmd_first_be),
      .cmd_wdata_o   (bar_cmd_wdata),

      .rsp_valid_i  (bar_rsp_valid),
      .rsp_ready_o  (bar_rsp_ready),
      .rsp_outcome_i(rsp_outcome),
      .rsp_rdata_i  (rsp_rdata)
  );

  // -------------------------------------------------------------------------
  // Stage D: the bridge path -- sequencer, then a SECOND scan/BAR pair for
  // the secondary bus. The existing stages are never re-armed (decision B,
  // docs/recon/RECON_stageD.md SS11.2): each instance below is single-shot exactly like
  // its level-1 twin, and "one bridge level, no recursion" is structural.
  // Both library modules instantiate AS-IS -- no parameter, no port differs
  // from the level-1 instances except the wiring.
  // -------------------------------------------------------------------------
  pcie_enum_bus u_bus (
      .clk_i(clk_i),
      .rst_i(rst_i),

      // Started at the scan's terminal state, gated by the level. Whether
      // there is a bridge to configure is decided inside, from the verdict.
      .bus_start_i         (bridge_enable_i && scan_done_o),
      .device_present_i    (device_present_o),
      .unsupported_device_i(unsupported_device_o),
      .header_type_i       (header_type_o),
      .bridge_bus_i        (scan_bus_i),

      .bus_busy_o          (),
      .bus_done_o          (bus_done_o),
      .bus_bypassed_o      (bus_bypassed_o),
      .bus_error_o         (bus_error),
      .bus_error_code_o    (bus_error_code),
      .err_credit_blocked_o(bus_credit_blocked),
      .sec_bus_o           (bus_sec_bus),
      .bus_type1_o         (bus_type1),

      .tx_fc_blocked_i(tx_fc_blocked_i),

      .cmd_valid_o   (bus_cmd_valid),
      .cmd_ready_i   (bus_cmd_ready),
      .cmd_write_o   (bus_cmd_write),
      .cmd_type1_o   (bus_cmd_type1),
      .cmd_reg_num_o (bus_cmd_reg_num),
      .cmd_ext_reg_o (bus_cmd_ext_reg),
      .cmd_first_be_o(bus_cmd_first_be),
      .cmd_wdata_o   (bus_cmd_wdata),

      .rsp_valid_i  (bus_rsp_valid),
      .rsp_ready_o  (bus_rsp_ready),
      .rsp_outcome_i(rsp_outcome),
      .rsp_rdata_i  (rsp_rdata)
  );

  // The second presence scan: same module, its own bus level. scan_bus_i is
  // the sequencer's sec_bus_o -- zero until bus_done_o, the secondary bus
  // number after -- and the start is bus_done_o itself, so the probe cannot
  // launch into a bridge that does not route yet.
  pcie_enum_scan u_scan2 (
      .clk_i(clk_i),
      .rst_i(rst_i),

      .scan_start_i(bus_done_o),
      .scan_bus_i  (bus_sec_bus),

      .scan_busy_o         (),
      .scan_done_o         (sec_scan_done_o),
      .scan_error_o        (scan2_error),
      .scan_error_code_o   (scan2_error_code),
      .err_credit_blocked_o(scan2_credit_blocked),

      .device_present_o    (sec_device_present_o),
      .unsupported_device_o(sec_unsupported_device_o),
      .device_bdf_o        (sec_device_bdf_o),
      .vendor_id_o         (sec_vendor_id_o),
      .device_id_o         (sec_device_id_o),
      .header_type_o       (sec_header_type_o),
      .multifunction_o     (sec_multifunction_o),

      .tx_fc_blocked_i(tx_fc_blocked_i),

      .cmd_valid_o   (scan2_cmd_valid),
      .cmd_ready_i   (scan2_cmd_ready),
      .cmd_write_o   (scan2_cmd_write),
      .cmd_reg_num_o (scan2_cmd_reg_num),
      .cmd_ext_reg_o (scan2_cmd_ext_reg),
      .cmd_first_be_o(scan2_cmd_first_be),
      .cmd_wdata_o   (scan2_cmd_wdata),

      .rsp_valid_i  (scan2_rsp_valid),
      .rsp_ready_o  (scan2_rsp_ready),
      .rsp_outcome_i(rsp_outcome),
      .rsp_rdata_i  (rsp_rdata)
  );

  // The second BAR phase, for the device the second scan found. Same
  // allocator geometry: on the bridge path the level-1 BAR phase configures
  // nothing (its S_CHECK sees the unsupported verdict and terminates with no
  // transaction), so the window cannot be double-allocated.
  //
  // !! P4.7 lives here structurally: this instance's verdict inputs are the
  // SECOND scan's, so it sizes the six-BAR Type 0 header at 05:00.0 and can
  // never be pointed at the bridge's Type 1 header -- whose register 6 is
  // the bus-number Dword an all-ones sweep would destroy.
  pcie_enum_bar #(
      .MEM_BAR_BASE  (MEM_BAR_BASE),
      .MEM_BAR_WINDOW(MEM_BAR_WINDOW)
  ) u_bar2 (
      .clk_i(clk_i),
      .rst_i(rst_i),

      .bar_start_i         (bridge_enable_i && sec_scan_done_o),
      .device_present_i    (sec_device_present_o),
      .unsupported_device_i(sec_unsupported_device_o),

      .tx_fc_blocked_i(tx_fc_blocked_i),

      .bar_busy_o          (),
      .enum_done_o         (sec_enum_done_o),
      .bar_error_o         (bar2_error),
      .bar_error_code_o    (bar2_error_code),
      .err_credit_blocked_o(bar2_credit_blocked),

      .bar_count_o   (sec_bar_count_o),
      .bar_valid_o   (sec_bar_valid_o),
      .bar_is_64_o   (sec_bar_is_64_o),
      .bar_prefetch_o(sec_bar_prefetch_o),
      .bar_size_o    (sec_bar_size_o),
      .bar_addr_o    (sec_bar_addr_o),
      .io_bar_mask_o (sec_io_bar_mask_o),

      .cmd_valid_o   (bar2_cmd_valid),
      .cmd_ready_i   (bar2_cmd_ready),
      .cmd_write_o   (bar2_cmd_write),
      .cmd_reg_num_o (bar2_cmd_reg_num),
      .cmd_ext_reg_o (bar2_cmd_ext_reg),
      .cmd_first_be_o(bar2_cmd_first_be),
      .cmd_wdata_o   (bar2_cmd_wdata),

      .rsp_valid_i  (bar2_rsp_valid),
      .rsp_ready_o  (bar2_rsp_ready),
      .rsp_outcome_i(rsp_outcome),
      .rsp_rdata_i  (rsp_rdata)
  );

  // -------------------------------------------------------------------------
  // ⭐ THE STATIC HANDOFF MUX, WIDENED. See SS THE HANDOFF and SS PROVING
  // EXACTLY ONE STAGE IS LIVE in the header.
  //
  // SELECT, never merge -- the named mutation to kill is still "two stages
  // drive simultaneously", and its kill is still the Command write's on-wire
  // first_be = 0011 (now the DEVICE's, transaction last).
  //
  // Every own_* term is a function of TERMINAL levels only, so each is
  // monotone within a reset epoch and exactly one is true at any time:
  //   scan1: until its terminal state;
  //   bar1:  after it, on the Type 0 path (bridge path not taken);
  //   bus:   after it, on the bridge path, until the 18h write completes --
  //          and PERMANENTLY if the sequencer bypasses or errors, holding
  //          the port idle;
  //   scan2/bar2: the second level, split by scan2's terminal state.
  // -------------------------------------------------------------------------
  wire bridge_path = bridge_enable_i && unsupported_device_o;

  wire own_scan1 = !scan_done_o;
  wire own_bar1  = scan_done_o && !bridge_path;
  wire own_bus   = scan_done_o && bridge_path && !bus_done_o;
  wire own_scan2 = bridge_path && bus_done_o && !sec_scan_done_o;
  wire own_bar2  = bridge_path && bus_done_o && sec_scan_done_o;

  assign cmd_valid    = own_scan1 ? scan_cmd_valid
                      : own_bar1  ? bar_cmd_valid
                      : own_bus   ? bus_cmd_valid
                      : own_scan2 ? scan2_cmd_valid
                                  : bar2_cmd_valid;
  assign cmd_write    = own_scan1 ? scan_cmd_write
                      : own_bar1  ? bar_cmd_write
                      : own_bus   ? bus_cmd_write
                      : own_scan2 ? scan2_cmd_write
                                  : bar2_cmd_write;
  assign cmd_reg_num  = own_scan1 ? scan_cmd_reg_num
                      : own_bar1  ? bar_cmd_reg_num
                      : own_bus   ? bus_cmd_reg_num
                      : own_scan2 ? scan2_cmd_reg_num
                                  : bar2_cmd_reg_num;
  assign cmd_ext_reg  = own_scan1 ? scan_cmd_ext_reg
                      : own_bar1  ? bar_cmd_ext_reg
                      : own_bus   ? bus_cmd_ext_reg
                      : own_scan2 ? scan2_cmd_ext_reg
                                  : bar2_cmd_ext_reg;
  assign cmd_first_be = own_scan1 ? scan_cmd_first_be
                      : own_bar1  ? bar_cmd_first_be
                      : own_bus   ? bus_cmd_first_be
                      : own_scan2 ? scan2_cmd_first_be
                                  : bar2_cmd_first_be;
  assign cmd_wdata    = own_scan1 ? scan_cmd_wdata
                      : own_bar1  ? bar_cmd_wdata
                      : own_bus   ? bus_cmd_wdata
                      : own_scan2 ? scan2_cmd_wdata
                                  : bar2_cmd_wdata;
  assign rsp_ready    = own_scan1 ? scan_rsp_ready
                      : own_bar1  ? bar_rsp_ready
                      : own_bus   ? bus_rsp_ready
                      : own_scan2 ? scan2_rsp_ready
                                  : bar2_rsp_ready;

  // cmd_type1 joins the muxed set (different stages need different values):
  // hard 0 for every level-1 stage -- scan1 and bar1 predate the port and
  // talk Type 0 by construction, and the sequencer's own cmd_type1_o is the
  // Trap C hard 0 -- and hard 1 for the second level, whose bus is beyond
  // the port's own (P2.2). Everything CFG1 in this design flows through
  // exactly these two arms.
  wire cmd_type1 = own_bus ? bus_cmd_type1
                 : (own_scan2 || own_bar2);

  // The per-LEVEL BDF select -- see SS THE BDF in the header. scan1's
  // device_bdf_o for its own level and for the sequencer (the bridge IS the
  // device the first scan found); scan2's for the second level.
  wire [15:0] cmd_bdf = (own_scan2 || own_bar2) ? sec_device_bdf_o
                                                : device_bdf_o;

  // The back channels, gated the same way. A stage that does not own the port
  // is told the primitive is permanently busy and permanently silent, so it
  // cannot complete a handshake against traffic that is not its own.
  assign scan_cmd_ready  = own_scan1 ? cmd_ready : 1'b0;
  assign bar_cmd_ready   = own_bar1  ? cmd_ready : 1'b0;
  assign bus_cmd_ready   = own_bus   ? cmd_ready : 1'b0;
  assign scan2_cmd_ready = own_scan2 ? cmd_ready : 1'b0;
  assign bar2_cmd_ready  = own_bar2  ? cmd_ready : 1'b0;
  assign scan_rsp_valid  = own_scan1 ? rsp_valid : 1'b0;
  assign bar_rsp_valid   = own_bar1  ? rsp_valid : 1'b0;
  assign bus_rsp_valid   = own_bus   ? rsp_valid : 1'b0;
  assign scan2_rsp_valid = own_scan2 ? rsp_valid : 1'b0;
  assign bar2_rsp_valid  = own_bar2  ? rsp_valid : 1'b0;

  // -------------------------------------------------------------------------
  // Combined status. At most one stage can be in error -- each stage starts
  // only from the previous stage's non-error terminal state, so the chain
  // property that held for two stages holds for five. ORs and selects, not a
  // priority scheme; the select order is just the pipeline order.
  // -------------------------------------------------------------------------
  assign enum_error_o         = scan_error_o || bar_error || bus_error ||
                                scan2_error || bar2_error;
  assign enum_error_code_o    = scan_error_o ? scan_error_code
                              : bar_error    ? bar_error_code
                              : bus_error    ? bus_error_code
                              : scan2_error  ? scan2_error_code
                                             : bar2_error_code;
  assign err_credit_blocked_o = scan_credit_blocked || bar_credit_blocked ||
                                bus_credit_blocked || scan2_credit_blocked ||
                                bar2_credit_blocked;

  // -------------------------------------------------------------------------
  // The one transaction primitive.
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

      .cmd_valid_i   (cmd_valid),
      .cmd_ready_o   (cmd_ready),
      .cmd_write_i   (cmd_write),
      // Part of the muxed set since Stage D -- 0 for every level-1 stage,
      // 1 for the second level. See the mux above.
      .cmd_type1_i   (cmd_type1),
      // Selected per LEVEL, never per phase -- see SS THE BDF in the header.
      .cmd_bdf_i     (cmd_bdf),
      .cmd_reg_num_i (cmd_reg_num),
      .cmd_ext_reg_i (cmd_ext_reg),
      .cmd_first_be_i(cmd_first_be),
      .cmd_wdata_i   (cmd_wdata),

      .rsp_valid_o     (rsp_valid),
      .rsp_ready_i     (rsp_ready),
      .rsp_outcome_o   (rsp_outcome),
      .rsp_rdata_o     (rsp_rdata),
      // Deliberately unused: the raw completion status and the CRS retry count
      // are the primitive's business. The stages act on the classified outcome
      // only -- re-decoding status up here would duplicate policy.
      .rsp_status_raw_o(),
      .crs_retries_o   (),

      .s_axis_rq_tdata_o (s_axis_rq_tdata_o),
      .s_axis_rq_tkeep_o (s_axis_rq_tkeep_o),
      .s_axis_rq_tvalid_o(s_axis_rq_tvalid_o),
      .s_axis_rq_tlast_o (s_axis_rq_tlast_o),
      .s_axis_rq_tuser_o (s_axis_rq_tuser_o),
      .s_axis_rq_tready_i(s_axis_rq_tready_i),

      .pcie_rq_tag_i    (pcie_rq_tag_i),
      .pcie_rq_tag_vld_i(pcie_rq_tag_vld_i),

      .m_axis_rc_tdata_i (m_axis_rc_tdata_i),
      .m_axis_rc_tkeep_i (m_axis_rc_tkeep_i),
      .m_axis_rc_tvalid_i(m_axis_rc_tvalid_i),
      .m_axis_rc_tlast_i (m_axis_rc_tlast_i),
      .m_axis_rc_tready_o(m_axis_rc_tready_o),

      .cpl_timeout_valid_i(cpl_timeout_valid_i),
      .cpl_timeout_tag_i  (cpl_timeout_tag_i)
  );

endmodule
