// ---------------------------------------------------------------------------
// pcie_enum_bus -- the bridge bus-number assignment phase.  Stage D.
//
// SPEC ANCHORS
//   [BASE] SS7.3.3 p.481 ..... the three Configuration Request routing arms;
//                              why a bridge answers UR until the 18h write
//                              lands, and why bus-number policy is left to the
//                              enumerator rather than mandated.
//   [BASE] SS7.3.1 p.479 ..... Device 0 association on a point-to-point link.
//   [PCI3] SS3.2.2.3.x p.49 .. Type 1 is compelled only for a target on ANOTHER
//                              bus -- which is why cmd_type1_o is a hard 0
//                              here. See the note below.
//   Tag conventions are defined in pcie_enum_pkg.sv:29.
//
//   bus_start_i + the scan's Type 1 verdict
//       -> ONE CfgWr0 to the bridge's register 6 (offset 18h)
//       -> sec_bus_o / bus_type1_o asserted, handoff to the second scan
//
// Policy only.  The one pcie_cfg_txn (owned by pcie_enum_top) does the
// transaction; nothing here decodes a status word, builds a descriptor or
// touches a tag.  This module also deliberately duplicates NO scan policy:
// probing the secondary bus is the second pcie_enum_scan instance's job, and
// this module's whole output is "the bridge now routes bus SEC_BUS_NUMBER --
// go" (docs/recon/RECON_stageD.md SS6.4, decision (c) rejected).
//
// ===========================================================================
// SS THE WRITE IS TYPE 0.  THAT IS THE POINT, NOT A DETAIL.
// ===========================================================================
//
// The name "bridge sequencer" invites "the bridge phase uses Type 1".  It
// does not: the bus-number write targets the BRIDGE ITSELF, which sits on
// the bus directly behind the port, and [PCI3] SS3.2.2.3.x p.49 compels
// Type 1 only for a target on ANOTHER bus.  So cmd_type1_o is a hard 0 here
// (Trap C, docs/predictions/SPEC_PREDICTIONS_STAGE_D.md SS8.3).  Everything from the first
// secondary-bus probe onward is Type 1 -- and that traffic is the second
// scan/BAR pair's, selected by the widened handoff mux, never this module's.
//
// A wrongly-typed CfgWr1 at 18h would be answered UR by a spec-faithful
// bridge automatically: at that instant Secondary is still 00h, so no bus
// matches and [BASE] SS7.3.3 p.481 case 3 applies.  The bench bridge model
// implements that arm literally, which is what makes the mistake
// self-detecting rather than something a test had to anticipate.
//
// ===========================================================================
// SS ORDERING: THE WRITE COMPLETES BEFORE THE HANDOFF.  SCOPED CLAIM.
// ===========================================================================
//
// A bridge at reset has Secondary = Subordinate = 00h, so every Type 1
// request is answered UR until the 18h write lands (SS7.3.3 p.481).  The FSM
// therefore asserts bus_done_o / sec_bus_o / bus_type1_o only AFTER the
// write's completion is classified TXN_OK -- structurally, from the state.
//
// !! This ordering is the acceptance criterion for THIS sequencer, NOT a
// spec check (P5.1 as scoped by P5.7): [BASE] SS7.3.3 p.481 leaves bus-number
// assignment implementation-specific, and a legal depth-first enumerator
// writes a provisional Subordinate, descends, then rewrites.  Stage D's
// single write is viable only because Subordinate is fixed a priori
// (SUB_BUS_NUMBER), not discovered.
//
// ===========================================================================
// SS ONE BRIDGE LEVEL, BY CONSTRUCTION -- AND THE STAGE E CAVEAT
// ===========================================================================
//
//  * ONE level only.  There is one write, one secondary bus constant, one
//    downstream scan/BAR pair.  DEVICES_TO_SCAN = 1 applies on the secondary
//    link unchanged: it too is point-to-point, so the SS7.3.1 p.479 Device-0
//    association holds below the bridge exactly as it does above
//    (docs/predictions/SPEC_PREDICTIONS_STAGE_D.md P5.3).
//
//  * !! THIS SHAPE DOES NOT ITERATE (docs/recon/RECON_stageD.md SS11.2).  A tree walk
//    needs iteration and the two-phase provisional-Subordinate protocol
//    (P5.7); per-level instances and this single-shot FSM provide neither.
//    The sequencing layer above pcie_cfg_txn is EXPECTED to be redesigned at
//    Stage E -- do not treat this module or the 4-way mux as load-bearing
//    architecture.  pcie_cfg_txn itself, being phase-blind, should survive.
//
// ===========================================================================
// SS OUTCOME POLICY
// ===========================================================================
//
//   TXN_OK            -> S_DONE, hand off
//   CRS (inside the primitive) -> invisible here: pcie_cfg_txn retries a
//                        CfgWr0-to-bridge exactly as any other request (P6.1
//                        -- a bridge is as entitled to a self-initialisation
//                        period as any device; P6.3 -- the retry is
//                        phase-blind)
//   TXN_UR            -> ENUM_ERR_UR_POST_PROBE.  Post-discovery, the bridge
//                        is KNOWN present -- it answered two probe reads --
//                        so UR here is a fault, exactly the scan's own
//                        post-probe policy
//   TXN_CA / TXN_CRS_EXHAUSTED / TXN_TIMEOUT -> the shared fault codes; a
//                        timeout is annotated with err_credit_blocked_o and
//                        (sec 63 #7f #19) named ENUM_ERR_CREDIT_STARVED when
//                        the credit gate held it -- reporting, never control
//                        flow
//
// Terminal states self-loop until reset, same invariant and same rationale
// as the scan (pcie_enum_scan.sv:413-416): enumeration is single-shot after
// link-up, and a status surface that could be re-entered would let a
// consumer sample it mid-rescan.  bus_start_i is sampled in S_IDLE only.
//
// No timers here: the CRS backoff lives in pcie_cfg_txn, the completion
// timeout in tlp_request_tracker.  tx_fc_blocked_i appears in no next-state
// expression.
//
// Guards use $warning, never $error: a procedural $error maps to $stop under
// the simulator, which would abort the shared multi-test process.
// ---------------------------------------------------------------------------
`timescale 1ns/1ps
// No parameters, same reasoning as post-hoist pcie_enum_scan: everything this
// module needs is fixed policy from pcie_enum_pkg, and a parameter that
// reaches nothing is a trap.
module pcie_enum_bus
  import pcie_rq_rc_pkg::*;
  import pcie_enum_pkg::*;
(
    input  logic                        clk_i,
    input  logic                        rst_i,

    // ---- control -----------------------------------------------------------
    // Sampled in IDLE only.  The integrator raises it at the scan's terminal
    // state; whether there is anything to do is decided HERE from the verdict
    // inputs below, so a plain Type 0 device bypasses this stage without it
    // ever emitting a transaction or asserting the handoff.
    input  logic                        bus_start_i,
    // The scan's verdict surface, stable by construction when bus_start_i can
    // rise (scan_done_o is terminal).  The eligible case is a PRESENT device
    // with a Type 1 layout: header_type_i[6:0] == HDR_LAYOUT_TYPE1, bit 7
    // masked off exactly as the scan masks it -- a multi-function bridge
    // CLASSIFIES as a bridge (P4.5; enumerating its functions 1-7 is out of
    // scope, and functions are 0 throughout by P5.3).
    input  logic                        device_present_i,
    input  logic                        unsupported_device_i,
    input  logic [7:0]                  header_type_i,
    // The bridge's own bus -- the bus the first scan probed.  Becomes the
    // Primary Bus Number byte, which is read-write but functionally inert on
    // PCI Express (SS7.5.3.2 p.493): the RC still writes the correct value;
    // no routing decision anywhere reads it (P4.3).
    input  logic [7:0]                  bridge_bus_i,

    // ---- status surface ----------------------------------------------------
    output logic                        bus_busy_o,
    // Terminal, non-error, bridge CONFIGURED: the handoff.  The second scan's
    // start is gated on this level, so "no CFG1 before the 18h write
    // completes" is a property of the state machine, not of test timing.
    output logic                        bus_done_o,
    // Terminal, non-error, nothing to do (no device / not a bridge).  The
    // direct-attach path must see this stage take no ownership at all.
    output logic                        bus_bypassed_o,
    output logic                        bus_error_o,
    output enum_error_e                 bus_error_code_o,
    // Diagnostic only, valid with bus_error_o on a timeout.  Never control.
    output logic                        err_credit_blocked_o,

    // The secondary bus number and Type select for the second-level pair.
    // Zero until S_DONE -- presenting them earlier would let a mis-wired
    // consumer start probing a bridge that does not route yet.
    output logic [7:0]                  sec_bus_o,
    output logic                        bus_type1_o,

    // ---- annotation input (NOT control flow) -------------------------------
    input  logic                        tx_fc_blocked_i,

    // ---- pcie_cfg_txn command port -----------------------------------------
    // The primitive lives in pcie_enum_top; cmd_bdf_i is not driven from here
    // (the BDF mux in pcie_enum_top selects the first scan's device_bdf_o
    // while this stage owns the port -- the bridge IS that device).
    output logic                        cmd_valid_o,
    input  logic                        cmd_ready_i,
    output logic                        cmd_write_o,
    // Trap C, structurally: the bus-number write addresses the bridge on the
    // LOCAL bus, so it is Type 0.  A hard 0, not a register.
    output logic                        cmd_type1_o,
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

  typedef enum logic [2:0] {
    S_IDLE,     // waiting for bus_start_i
    S_WR_CMD,   // offering the one CfgWr0 to register 6 (18h)
    S_WR_RSP,   // classifying its outcome
    S_DONE,     // terminal: bridge configured, handoff asserted
    S_BYPASS,   // terminal: nothing here for this stage
    S_ERROR     // terminal, sticky, reset-only
  } bus_state_e;

  bus_state_e  state_r;
  enum_error_e error_code_r;
  logic        credit_blocked_r;

  // The Type 1 verdict, masked exactly as the scan masks it (bit 7 is the
  // multi-function bit, not part of the layout code).
  wire hdr_is_type1 = (header_type_i[6:0] == HDR_LAYOUT_TYPE1);
  wire eligible     = device_present_i && unsupported_device_i && hdr_is_type1;

  // Same shape as pcie_enum_scan's fault_code, written once here so the
  // response state cannot drift from the scan's post-probe policy.
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
      error_code_r     <= ENUM_ERR_NONE;
      credit_blocked_r <= 1'b0;
    end else begin
      unique case (state_r)
        S_IDLE: begin
          if (bus_start_i) begin
            error_code_r     <= ENUM_ERR_NONE;
            credit_blocked_r <= 1'b0;
            state_r          <= eligible ? S_WR_CMD : S_BYPASS;
          end
        end

        S_WR_CMD: if (cmd_ready_i) state_r <= S_WR_RSP;

        S_WR_RSP: begin
          if (rsp_valid_i) begin
            case (rsp_outcome_i)
              TXN_OK: state_r <= S_DONE;
              // Post-discovery policy: the bridge answered two probe reads,
              // so it is KNOWN present and a UR to a legal write of its own
              // register 6 is a fault -- never "absent".
              TXN_UR: begin
                error_code_r <= ENUM_ERR_UR_POST_PROBE;
                state_r      <= S_ERROR;
              end
              default: begin
                error_code_r     <= fault_code(rsp_outcome_i, tx_fc_blocked_i);
                credit_blocked_r <= (rsp_outcome_i == TXN_TIMEOUT) && tx_fc_blocked_i;
                state_r          <= S_ERROR;
              end
            endcase
          end
        end

        // Terminal states self-loop until reset -- single-shot, same
        // rationale as the scan's (see the header).
        S_DONE:   state_r <= S_DONE;
        S_BYPASS: state_r <= S_BYPASS;
        S_ERROR:  state_r <= S_ERROR;
      endcase
    end
  end

  // -------------------------------------------------------------------------
  // The one command.  Whole-Dword (first_be = 1111): all four fields live in
  // one Dword so the Stage C whole-DW precedent applies exactly, and there
  // are no reserved bits to read-modify-write around (P4.1).  The latency
  // byte is written 00h -- the register is read-only 00h (SS7.5.3.3 p.493),
  // so any other value would be asserting a spec violation on read-back.
  // -------------------------------------------------------------------------
  assign cmd_valid_o    = (state_r == S_WR_CMD);
  assign cmd_write_o    = 1'b1;
  assign cmd_type1_o    = 1'b0;                    // Trap C -- see the header
  assign cmd_reg_num_o  = CFG_REG_BUS_NUMBER;
  assign cmd_ext_reg_o  = CFG_EXT_REG_NONE;
  assign cmd_first_be_o = CFG_BE_DWORD;
  assign cmd_wdata_o    = {SEC_LATENCY_TIMER_WDATA, SUB_BUS_NUMBER,
                           SEC_BUS_NUMBER, bridge_bus_i};

  assign rsp_ready_o    = (state_r == S_WR_RSP);

  // -------------------------------------------------------------------------
  // Status.  sec_bus_o / bus_type1_o are qualified by the terminal state, so
  // the downstream pair cannot be offered a bus number the bridge does not
  // route yet -- the ordering claim is combinational off S_DONE, not timed.
  // -------------------------------------------------------------------------
  assign bus_busy_o           = (state_r == S_WR_CMD) || (state_r == S_WR_RSP);
  assign bus_done_o           = (state_r == S_DONE);
  assign bus_bypassed_o       = (state_r == S_BYPASS);
  assign bus_error_o          = (state_r == S_ERROR);
  assign bus_error_code_o     = error_code_r;
  assign err_credit_blocked_o = credit_blocked_r;

  assign sec_bus_o            = bus_done_o ? SEC_BUS_NUMBER : 8'h00;
  assign bus_type1_o          = bus_done_o;

endmodule
