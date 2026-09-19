// ---------------------------------------------------------------------------
// pcie_enum_scan -- the presence phase of Root Complex enumeration.
// Commit 2b-2.
//
// SPEC ANCHORS
//   [BASE] SS7.3.1 p.479 ......... Downstream Ports associate with Device 0
//                                  only -- the reason this module probes ONE
//                                  device and not a 0..31 sweep.
//   [BASE] SS7.3.3 p.480 ......... the general Endpoint rule for an
//                                  unimplemented Function.
//   [BASE] SS2.3.2 p.122 ......... Implementation Note: a Root Complex
//                                  synthesises all-ones for a failed probe,
//                                  which is what "absent" decodes from.
//   [BASE] Figure 7-5 p.491 ...... Type 0 header layout (Vendor/Device ID,
//                                  Header Type).
//   Tag conventions are defined in pcie_enum_pkg.sv:29.
//
//   scan_start_i -> Vendor/Device ID probe -> [absent? done]
//                -> Header Type read       -> [Type 1? unsupported]
//                -> scan_done_o
//
// This module drives exactly one pcie_cfg_txn. It contains all the POLICY; the
// primitive contains all the MECHANISM. Nothing here decodes a completion
// status, builds a descriptor or touches a tag, and nothing in pcie_cfg_txn
// knows what phase of enumeration it is serving.
//
// ===========================================================================
// SS THE HOIST -- WHERE THE PRIMITIVE WENT (Commit 2b-3)
// ===========================================================================
//
// Until 2b-3 this module INSTANTIATED the primitive. It no longer does: the one
// pcie_cfg_txn now lives one level up in pcie_enum_top, and this module reaches
// it through the command/response ports below.
//
// The reason is pcie_enum_bar, the BAR phase added in the same increment. Three
// options were weighed:
//
//   (a) extend this module in place    -- breaks the 16 scan tests, which assert
//                                         no further traffic after scan_done_o;
//                                         a BAR phase produces exactly that
//   (b) give pcie_enum_bar its own     -- duplicates the primitive and abandons
//       pcie_cfg_txn                      single-outstanding-BY-CONSTRUCTION as
//                                         a structural property
//   (c) hoist the primitive up one     -- one behaviour-neutral refactor, every
//       level                             module stays single-purpose
//
// (c) was taken. The sequencer body below is UNCHANGED by the hoist -- the
// internal signal names cmd_valid/cmd_ready/rsp_* are kept exactly as they were
// when the instance was here, and only the port bindings are new. That is what
// makes "behaviour-neutral" checkable by diff rather than by assertion.
//
// Single-outstanding is still structural, and is now enforced somewhere
// STRONGER: pcie_enum_top holds one primitive and a static handoff mux that
// gives the command port to exactly one stage at a time. Neither this module
// nor pcie_enum_bar can issue while the other owns it.
//
// ===========================================================================
// SS WHY THE SPLIT IS REAL, AND NOT BOOKKEEPING
// ===========================================================================
//
// One row of the outcome table justifies the whole two-module structure:
//
//   TXN_UR during the probe      -> "nothing here to enumerate", normal exit
//   TXN_UR after the probe       -> fault
//
// Identical wire event, opposite meaning, and the difference is *which
// transaction we are on* -- context pcie_cfg_txn deliberately does not have.
// Folding the two together would put a phase bit inside the transaction engine,
// which is the shape bugs live in.
//
// ===========================================================================
// SS ⭐ DEVICE 0 ONLY -- THERE IS NO DEVICE-NUMBER LOOP
// ===========================================================================
//
// The PCI-era 0-31 sweep is inherited convention and is WRONG here, not merely
// wasteful. PCIe Base 2.1 SS7.3.1 p.479 says two things:
//
//   1. "Configuration Requests specifying all other Device Numbers (1-31) must
//      be terminated by the Switch Downstream Port or the Root Port with an
//      Unsupported Request Completion Status."
//
//      In a conventional Root Complex the enumerating SOFTWARE does sweep 0-31
//      and sees UR for 1-31 -- but that UR is synthesised by the Root Port's own
//      downstream-port logic and the request never reaches the wire. THIS DESIGN
//      HAS NO SUCH LOGIC: the completer surface is tied off
//      (pcie_rq_rc_top.sv:96-99) and pcie_rq_rc_top originates Type 0 straight
//      onto the link. A request naming device 5 would actually be transmitted.
//
//   2. "Non-ARI Devices must respond to all Type 0 Configuration Read Requests,
//      REGARDLESS of the Device Number specified in the Request."
//
//      So if such a request were transmitted, the attached device would answer
//      it. A 0-31 sweep on a direct-attach link would not find one device and
//      31 absences -- it would find the SAME DEVICE 32 TIMES, each with
//      identical Vendor/Device ID.
//
// Probing device 0 only satisfies the rule BY CONSTRUCTION: no request naming
// device 1-31 is ever formed, so there is nothing to terminate. That is a
// structural equivalence, not a deviation -- a compliant RC's software would see
// UR for those numbers; this hardware enumerator simply never asks.
//
// Building the Root-Port termination path is Commits 3/4 work, where a switch
// can first appear below the port and Type 1 arrives with it.
//
// ===========================================================================
// SS WHAT "ABSENT" MEANS ON A POINT-TO-POINT LINK
// ===========================================================================
//
// link_up_i is a precondition of the whole stack, and a PCIe link is
// point-to-point, so a device IS attached whenever this scan runs. Absence
// therefore cannot mean "no device on the link". It means "nothing here to
// enumerate", and the spec gives exactly one signal for it: an Unsupported
// Request to the Function 0 probe -- SS7.3.1 p.479 for an unimplemented Function
// in an ARI Device, SS7.3.3 p.480 for the general Endpoint rule.
//
// That is why TXN_UR on the probe is a NORMAL exit with device_present_o low,
// and why there is no ENUM_ERR code for it.
//
// !! AND WHY THE ALL-1s CONVENTION IS NOT USED HERE.
//
// SS2.3.2 Implementation Note p.122 has a Root Complex synthesise a read value of
// all 1s "when UR Completion Status is returned for a Configuration Read
// Request", for software that depends on it. That synthesis is performed BY the
// Root Complex FOR software above it. This module sits where it would be
// PERFORMED, not consumed: it sees the UR itself, as TXN_UR.
//
// So absence is signalled by TXN_UR and by nothing else, and vendor_id_o ==
// FFFFh is NOT treated as absence. Re-deriving absence from a sentinel would
// discard information the spec took care to keep distinguishable. A Successful
// Completion carrying FFFFFFFF is reported PRESENT with that Vendor ID; it is
// not this module's business to reinterpret an SC.
//
// ===========================================================================
// SS A TAG STROBE IS NOT EVIDENCE OF TRANSMISSION
// ===========================================================================
//
// This module does not observe pcie_rq_tag_o at all -- it watches only the
// primitive's response port. That is deliberate. Tag allocation sits UPSTREAM
// of the credit gate: tlp_requester raises tag_request_valid_o on entering
// REQ_TAG (tlp_requester.sv:138, 211) with no reference to fc_initialized_i or
// credit, while the gate is at the VC-buffer-to-transmit boundary
// (tlp_layer.sv:280). A tag can therefore be handed out for a request that is
// still parked in the VC buffer and may never leave (measured: Commit 2b-1 test
// i8).
//
// Since the 2b-3 hoist this is STRUCTURAL rather than a matter of discipline:
// pcie_rq_tag_i is not on this module's port list at all. It terminates at
// pcie_enum_top and goes only to the primitive, so there is no longer a wire on
// which this module could observe a tag even by mistake. pcie_enum_bar inherits
// the same guarantee for free.
//
// ===========================================================================
// SS NO TIMERS HERE, AND ONE THING THAT CANNOT BE FIXED HERE
// ===========================================================================
//
// The CRS backoff lives in pcie_cfg_txn; the completion timeout lives in
// tlp_request_tracker. This module waits indefinitely on cmd_ready_i and
// rsp_valid_i and has no counter of its own.
//
// tx_fc_blocked_i is sampled ONLY when a TXN_TIMEOUT is reported: it sets
// err_credit_blocked_o and (sec 63 #7f #19) selects ENUM_ERR_CREDIT_STARVED
// over ENUM_ERR_TIMEOUT as the code. IT APPEARS IN NO NEXT-STATE EXPRESSION --
// both codes land in S_ERROR. A credit signal gating control flow is exactly
// the watchdog mistake the design forbids, and the mutation set proves the
// state sequence is identical with tx_fc_blocked_i forced either way.
//
// The annotation exists because of a bound this module cannot remove:
// tlp_request_tracker measures per-tag age from ALLOCATION (that module's
// header, :39), and allocation precedes the credit gate. A request starved of
// credit for longer than CPL_TIMEOUT_CYCLES therefore times out WITHOUT EVER
// HAVING BEEN TRANSMITTED, and is indistinguishable from a dead device
// (measured: Commit 2b-1 test i9). No FSM above this stack can ride that out.
// err_credit_blocked_o is the most a client can do -- say "this timeout smells
// like credit". Fixing it means raising CPL_TIMEOUT_CYCLES toward the ~10 ms the
// spec recommends, which is Stage-H work.
//
// Guards use $warning, never $error: a procedural $error maps to $stop under
// the simulator, which would abort the shared multi-test process.
// ---------------------------------------------------------------------------
`timescale 1ns/1ps
// After the Commit 2b-3 hoist this module has NO PARAMETERS. All six it used to
// carry (the three AXIS widths, the two CRS constants, CPL_TIMEOUT_CYCLES) were
// forwarded verbatim to the pcie_cfg_txn it instantiated and were referenced
// nowhere else in its body. They now live on pcie_enum_top, which is where the
// primitive lives. A parameter that reaches nothing is a trap, so none is kept
// for symmetry.
module pcie_enum_scan
  import pcie_rq_rc_pkg::*;
  import pcie_enum_pkg::*;
(
    input  logic                        clk_i,
    input  logic                        rst_i,

    // ---- control -----------------------------------------------------------
    // Sampled in IDLE only. The integrator raises it once the four link/flow
    // control preconditions hold; enumeration is single-shot after link-up and
    // does not support re-entry after a retrain (tracker SS20.4).
    input  logic                        scan_start_i,
    // Bus number of the Logical Bus representing the Link. Device and Function
    // are 0 by SS7.3.1 -- see the header.
    input  logic [7:0]                  scan_bus_i,

    // ---- status surface ----------------------------------------------------
    // scan_done_o marks a terminal NON-ERROR outcome: the scan finished and its
    // results are stable. It covers both "device found" and "nothing to
    // enumerate", and also the unsupported-device exit -- a consumer that wants
    // a configurable Type 0 device asks for
    //     scan_done_o && device_present_o && !unsupported_device_o
    output logic                        scan_busy_o,
    output logic                        scan_done_o,
    output logic                        scan_error_o,
    output enum_error_e                 scan_error_code_o,
    // Diagnostic ONLY, valid with scan_error_o on a timeout: tx_fc_blocked_o was
    // asserted when the timeout was reported, so the request was probably never
    // transmitted. See the header. Never an input to control flow. Since
    // sec 63 #7f #19 the same sample also selects ENUM_ERR_CREDIT_STARVED as
    // the code, so this bit is redundant with the code and kept for its
    // existing consumers.
    output logic                        err_credit_blocked_o,

    output logic                        device_present_o,
    output logic                        unsupported_device_o,
    output logic [15:0]                 device_bdf_o,
    output logic [15:0]                 vendor_id_o,
    output logic [15:0]                 device_id_o,
    output logic [7:0]                  header_type_o,
    output logic                        multifunction_o,

    // ---- annotation input (NOT control flow) -------------------------------
    input  logic                        tx_fc_blocked_i,

    // ---- pcie_cfg_txn command port -----------------------------------------
    // The primitive now lives one level up, in pcie_enum_top. See the HOIST
    // note in the header. cmd_bdf_i is NOT driven from here: the target BDF is
    // a property of the device, not of the phase, so pcie_enum_top wires it
    // from device_bdf_o once for every stage.
    output logic                        cmd_valid_o,
    input  logic                        cmd_ready_i,
    output logic                        cmd_write_o,
    output logic [5:0]                  cmd_reg_num_o,
    output logic [3:0]                  cmd_ext_reg_o,
    output logic [3:0]                  cmd_first_be_o,
    output logic [31:0]                 cmd_wdata_o,

    // ---- pcie_cfg_txn response port ----------------------------------------
    input  logic                        rsp_valid_i,
    output logic                        rsp_ready_o,
    input  txn_outcome_e                rsp_outcome_i,
    input  logic [31:0]                 rsp_rdata_i
);

  // -------------------------------------------------------------------------
  // Command/response wiring to the one transaction primitive, which lives in
  // pcie_enum_top. One in flight, always -- Table 2-37 p.137 permits a peer to
  // advertise a single NPH credit, so a second config request could not be
  // transmitted anyway (see pcie_cfg_txn's header).
  //
  // The internal names below are kept EXACTLY as they were when the primitive
  // was instantiated here, so the sequencer body underneath is byte-identical
  // across the hoist. Only these bindings are new.
  // -------------------------------------------------------------------------
  logic         cmd_valid;
  logic         cmd_ready;
  logic         rsp_valid;
  logic         rsp_ready;
  txn_outcome_e rsp_outcome;
  logic [31:0]  rsp_rdata;

  logic [5:0]   cmd_reg_num;

  assign cmd_valid_o   = cmd_valid;
  assign cmd_ready     = cmd_ready_i;
  assign cmd_reg_num_o = cmd_reg_num;

  assign rsp_valid     = rsp_valid_i;
  assign rsp_ready_o   = rsp_ready;
  assign rsp_outcome   = rsp_outcome_i;
  assign rsp_rdata     = rsp_rdata_i;

  // Constants formerly written at the u_txn instantiation site. Driven from
  // here rather than tied off in the parent so that the Commit-D handoff mux
  // sees a COMPLETE command port from each stage and needs no per-stage
  // constants of its own.
  //
  // Both scan transactions are READS. Nothing here writes.
  assign cmd_write_o    = 1'b0;
  assign cmd_ext_reg_o  = CFG_EXT_REG_NONE;
  assign cmd_first_be_o = CFG_BE_DWORD;
  assign cmd_wdata_o    = 32'd0;

  // -------------------------------------------------------------------------
  // Sequencer.
  //
  // The probe and header-type phases get their OWN response states rather than
  // sharing one with a phase flag, so that the phase-dependent policy of
  // docs/predictions/SPEC_PREDICTIONS_ENUM.md SSD.5 maps one-to-one onto the RTL and a reader can
  // check the TXN_UR rows against the table by eye.
  // -------------------------------------------------------------------------
  typedef enum logic [2:0] {
    S_IDLE,         // waiting for scan_start_i
    S_PROBE_CMD,    // offering the Vendor/Device ID read
    S_PROBE_RSP,    // classifying it -- PROBE policy: UR means absent
    S_HDR_CMD,      // offering the Header Type read
    S_HDR_RSP,      // classifying it -- POST-PROBE policy: UR is a fault
    S_DONE,         // terminal, no error
    S_UNSUPPORTED,  // terminal, no error: a device we cannot enumerate yet
    S_ERROR         // terminal, sticky, reset-only
  } scan_state_e;

  scan_state_e state_r;

  logic [15:0] vendor_id_r, device_id_r;
  logic [7:0]  header_type_r;
  logic        present_r;
  enum_error_e error_code_r;
  logic        credit_blocked_r;

  // Header Type sits in byte 2 of register 3 -- [BASE] Figure 7-5 p.491.
  wire [7:0]  hdr_byte    = rsp_rdata[HDR_TYPE_LSB +: 8];
  wire [6:0]  hdr_layout  = hdr_byte[6:0];
  wire        hdr_is_type0 = (hdr_layout == HDR_LAYOUT_TYPE0);

  // Register 0 is {Device ID[31:16], Vendor ID[15:0]} -- [BASE] Figure 7-5 p.491.
  wire [15:0] probe_vendor = rsp_rdata[15:0];
  wire [15:0] probe_device = rsp_rdata[31:16];

  // A fault that is not UR-specific classifies the same way in both phases.
  // Written once so the two response states cannot drift apart.
  function automatic enum_error_e fault_code(input txn_outcome_e outcome,
                                             input logic         credit_blocked);
    case (outcome)
      TXN_CA:            fault_code = ENUM_ERR_CA;
      TXN_CRS_EXHAUSTED: fault_code = ENUM_ERR_CRS_EXHAUSTED;
      // sec 63 #7f #19: a timeout the credit gate caused gets its own code.
      // credit_blocked is tx_fc_blocked_i at the moment the timeout is
      // reported -- the same sample err_credit_blocked_o records.
      TXN_TIMEOUT:       fault_code = credit_blocked ? ENUM_ERR_CREDIT_STARVED
                                                    : ENUM_ERR_TIMEOUT;
      default:           fault_code = ENUM_ERR_UR_POST_PROBE;
    endcase
  endfunction

  always_ff @(posedge clk_i) begin
    if (rst_i) begin
      state_r          <= S_IDLE;
      vendor_id_r      <= '0;
      device_id_r      <= '0;
      header_type_r    <= '0;
      present_r        <= 1'b0;
      error_code_r     <= ENUM_ERR_NONE;
      credit_blocked_r <= 1'b0;
    end else begin
      unique case (state_r)
        S_IDLE: begin
          if (scan_start_i) begin
            vendor_id_r      <= '0;
            device_id_r      <= '0;
            header_type_r    <= '0;
            present_r        <= 1'b0;
            error_code_r     <= ENUM_ERR_NONE;
            credit_blocked_r <= 1'b0;
            state_r          <= S_PROBE_CMD;
          end
        end

        S_PROBE_CMD: if (cmd_ready) state_r <= S_PROBE_RSP;

        // ---- PROBE policy. docs/predictions/SPEC_PREDICTIONS_ENUM.md SSD.5, left column. -----
        S_PROBE_RSP: begin
          if (rsp_valid) begin
            case (rsp_outcome)
              TXN_OK: begin
                vendor_id_r <= probe_vendor;
                device_id_r <= probe_device;
                present_r   <= 1'b1;
                state_r     <= S_HDR_CMD;
              end
              // ABSENT, not an error: on a point-to-point link with link_up_i
              // asserted a device is attached, so a UR to the Function 0 probe
              // means "nothing here to enumerate" (SS7.3.1 p.479, SS7.3.3 p.480).
              TXN_UR: begin
                present_r <= 1'b0;
                state_r   <= S_DONE;
              end
              default: begin
                error_code_r     <= fault_code(rsp_outcome, tx_fc_blocked_i);
                credit_blocked_r <= (rsp_outcome == TXN_TIMEOUT) && tx_fc_blocked_i;
                state_r          <= S_ERROR;
              end
            endcase
          end
        end

        S_HDR_CMD: if (cmd_ready) state_r <= S_HDR_RSP;

        // ---- POST-PROBE policy. Same table, right column. -------------------
        S_HDR_RSP: begin
          if (rsp_valid) begin
            case (rsp_outcome)
              TXN_OK: begin
                header_type_r <= hdr_byte;
                // A Type 1 header is a bridge or switch: a valid device that
                // answered correctly and that Commit 2b cannot enumerate.
                // Terminal, but NOT an error.
                state_r       <= hdr_is_type0 ? S_DONE : S_UNSUPPORTED;
              end
              // A device that answered register 0 has no business rejecting a
              // legal configuration read of register 3.
              TXN_UR: begin
                error_code_r <= ENUM_ERR_UR_POST_PROBE;
                state_r      <= S_ERROR;
              end
              default: begin
                error_code_r     <= fault_code(rsp_outcome, tx_fc_blocked_i);
                credit_blocked_r <= (rsp_outcome == TXN_TIMEOUT) && tx_fc_blocked_i;
                state_r          <= S_ERROR;
              end
            endcase
          end
        end

        // Terminal states. S_DONE and S_UNSUPPORTED hold until reset just as
        // S_ERROR does: enumeration is single-shot after link-up, and a status
        // surface that could be re-entered would let a consumer sample it
        // mid-rescan.
        S_DONE:        state_r <= S_DONE;
        S_UNSUPPORTED: state_r <= S_UNSUPPORTED;
        S_ERROR:       state_r <= S_ERROR;
      endcase
    end
  end

  assign cmd_valid   = (state_r == S_PROBE_CMD) || (state_r == S_HDR_CMD);
  assign cmd_reg_num = (state_r == S_HDR_CMD) ? CFG_REG_CACHE_HEADER
                                              : CFG_REG_VENDOR_DEVICE;
  // rsp_ready is a real handshake, not a strobe: the primitive holds rsp_valid_o
  // until it is consumed, so an unready consumer cannot miss an outcome.
  assign rsp_ready   = (state_r == S_PROBE_RSP) || (state_r == S_HDR_RSP);

  // {Bus[15:8], Device[7:3], Function[2:0]}, device and function fixed at 0 --
  // see the header for why there is no device loop.
  assign device_bdf_o = {scan_bus_i, 5'd0, 3'd0};

  assign scan_busy_o          = (state_r != S_IDLE) && (state_r != S_DONE) &&
                                (state_r != S_UNSUPPORTED) && (state_r != S_ERROR);
  assign scan_done_o          = (state_r == S_DONE) || (state_r == S_UNSUPPORTED);
  assign scan_error_o         = (state_r == S_ERROR);
  assign scan_error_code_o    = error_code_r;
  assign err_credit_blocked_o = credit_blocked_r;

  assign device_present_o     = present_r;
  assign unsupported_device_o = (state_r == S_UNSUPPORTED);
  assign vendor_id_o          = vendor_id_r;
  assign device_id_o          = device_id_r;
  assign header_type_o        = header_type_r;
  assign multifunction_o      = header_type_r[HDR_MULTIFUNCTION_BIT];

endmodule
