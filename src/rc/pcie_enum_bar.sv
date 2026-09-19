// ---------------------------------------------------------------------------
// pcie_enum_bar -- the BAR sizing, assignment and enable phase of Root Complex
// enumeration. Commit 2b-3.
//
// SPEC ANCHORS
//   [PCI3] SS6.2.5.1 p.225-226 ... BAR bit layout and the write-all-ones sizing
//                                  algorithm; the I/O bit, the unimplemented
//                                  encoding, and the reserved [2:1] faults.
//   [PCI3] Table 6-4 p.226 ....... memory BAR type field encodings.
//   [PCI3] SS6.2.2 Table 6-1 p.218 reset state of the Command register bits.
//   [BASE] Figure 7-5 p.491 ...... Type 0 header: which registers are BARs
//                                  (4..9) and which are not -- register 11 is
//                                  the Expansion ROM pointer, not a BAR.
//   Tag conventions are defined in pcie_enum_pkg.sv:29.
//
//   bar_start_i -> [for each candidate register 4..9]
//                     all-ones write -> readback -> decode
//                        unimplemented -> skip,          N += 1
//                        I/O BAR       -> skip and log,  N += 1
//                        32-bit memory -> size, allocate, assign,      N += 1
//                        64-bit memory -> probe N+1, combine, assign,  N += 2
//               -> Command register write (Memory Space + Bus Master)
//               -> enum_done_o
//
// Like pcie_enum_scan this module is ALL POLICY. It drives one pcie_cfg_txn
// through the command/response port below; the primitive, which lives in
// pcie_enum_top, is ALL MECHANISM. Nothing here decodes a completion status,
// builds a descriptor, runs a timer or touches a tag.
//
// ===========================================================================
// SS ⭐ THE ORDERING INVARIANT, AND WHY IT IS THE WHOLE DESIGN
// ===========================================================================
//
// [PCI3] SS6.2.2 Table 6-1 p.218 gives the reset state of the three Command bits
// this module cares about, and says the same thing three times -- "State after
// RST# is 0" for I/O Space Enable (:10761), Memory Space Enable (:10764) and
// Bus Master Enable (:10767).
//
// So at enumeration time THE DEVICE'S DECODERS ARE OFF. That single fact
// licenses three things that would otherwise be hazards:
//
//   1. Writing FFFFFFFF into a BAR to size it. The BAR momentarily names an
//      enormous address range; a device with Memory Space Enable set would
//      briefly decode it.
//   2. Assigning BAR N while BAR N+2 is still unprobed (SSE.5.2) -- a BAR holding
//      a real address decodes nothing yet.
//   3. Writing the two halves of a 64-bit pair NON-ATOMICALLY (SSE.3.3). Between
//      the two writes the pair holds {old_upper, new_lower}, a garbage address.
//
// ONE invariant covers all three:
//
//   !! ALL SIZING AND ALL ASSIGNMENT COMPLETE BEFORE THE COMMAND WRITE, AND THE
//   !! COMMAND WRITE IS THE LAST TRANSACTION OF ENUMERATION.
//
// It is STRUCTURAL here: S_CMD_WR is reachable only from the candidate-advance
// path once cand_r has passed CFG_REG_BAR_LAST, and from nowhere else. But the
// bench asserts it ON THE WIRE anyway (P-CMD-LAST, SSE.5.1) rather than trusting
// the structure -- a structural argument is exactly what the mutation "Command
// write moved before sizing completes" invalidates.
//
// !! AND IT IS WHY THE 64-BIT WRITE ORDER IS FREE. Lower-then-upper is chosen
// for legibility (it matches the probe order), not for safety. The thing that
// makes it safe is the invariant. Stated because it becomes a real bug the
// moment enumeration re-entry arrives (Stage H): re-enumerating an ALREADY
// ENABLED device would have to clear Command bit 1 first, and only then does
// the write order start to matter.
//
// ===========================================================================
// SS ⭐ WHY THE ALL-ONES WRITE CANNOT BE SKIPPED
// ===========================================================================
//
// The tempting simplification is to read a BAR without writing first and call a
// zero readback "unimplemented". That is wrong, and the difference is
// observable.
//
// After reset an IMPLEMENTED 32-bit non-prefetchable memory BAR reads
// 00000000 -- its base address is unassigned, every size bit is a don't-care
// returning 0, and bits [3:0] are all legitimately zero (memory, 32-bit, not
// prefetchable). It is BIT-FOR-BIT IDENTICAL to an unimplemented BAR's hardwired
// zero ([PCI3] p.226 :11224).
//
// Only after the all-ones write do the two diverge. So the write is mandatory
// FOR CORRECTNESS, not merely conventional.
//
// !! AND THE TEST THAT PROVES IT MUST USE A 32-BIT NON-PREFETCHABLE BAR. With a
// 64-bit or prefetchable BAR the reset readback is 0000000C, not 00000000, so a
// "removed the all-ones write" mutation still classifies it as implemented and
// SURVIVES. That is reaching the mutated LINE without reaching the mutated
// CONDITION. SSE.9 EF7.
//
// ===========================================================================
// SS THE DECODE ORDER, WHICH IS A DECISION
// ===========================================================================
//
// On the readback the tests are applied in this order, and the order is load
// bearing for malformed devices even though it is immaterial for well-formed
// ones:
//
//   1. bit 0 == 1                -> I/O BAR      [PCI3] p.225 :11187
//   2. the whole register == 0   -> unimplemented[PCI3] p.226 :11224
//   3. [2:1] in {01, 11}         -> fault        [PCI3] Table 6-4 p.226 :11207
//   4. [2:1] == 00               -> 32-bit memory
//   5. [2:1] == 10               -> 64-bit pair
//
// I/O FIRST, because an I/O BAR small enough that its masked value is zero would
// otherwise be misread as unimplemented. An unimplemented register reads zero,
// so its bit 0 is 0 and it falls through to test 2 correctly.
//
// !! TEST 2 IS "THE REGISTER READS ZERO", NOT SSE.2.1's "encoded == 0", AND THE
// DIFFERENCE IS REAL. SSE.2.1 scopes its rule to a 32-BIT memory BAR, where the
// two are equivalent: encoded == 0 with a non-zero register would need bits
// [3:0] only, which is either an I/O BAR (caught at test 1) or a 32-bit BAR
// claiming 2^32 bytes, which cannot be encoded. They DIVERGE for a 64-bit BAR of
// 4 GB or more, whose lower half legitimately reads 0000000C with every size bit
// zero -- the size lives entirely in the upper register. Masking first would
// report that BAR unimplemented. The hardwired-zero rule of :11224 is the
// general one and is used here; it agrees with SSE.2.1 everywhere SSE.2.1 speaks.
//
// ===========================================================================
// SS THE PROBE ORDER FOR A 64-BIT PAIR IS LAZY, NOT SPECULATIVE
// ===========================================================================
//
//   speculative:  W(N), W(N+1), R(N), R(N+1)      rejected
//   lazy:         W(N), R(N) -> decode -> W(N+1), R(N+1)     CHOSEN
//
// The FSM cannot know a BAR is 64-bit until it has read register N. Speculating
// writes a register it may have no business writing -- harmless when N+1 is the
// next candidate anyway, which is a coincidence rather than an argument, and it
// evaporates when N is the LAST candidate. There is nothing to gain in exchange:
// the stack is single-outstanding by construction (pcie_enum_top holds exactly
// one pcie_cfg_txn), so both orders cost four serialized transactions. SSE.3.1.
//
// !! A 64-BIT TYPE DECLARED IN THE LAST CANDIDATE REGISTER IS A FAULT. Register
// 9 (BAR5) has no register 10 to pair with -- offset 28h is the Cardbus CIS
// Pointer, not a BAR ([BASE] Figure 7-5 p.491) -- so a device declaring one
// there is malformed, and honouring it would drive an all-ones write clean out
// of the candidate window. Reported as ENUM_ERR_BAR_TYPE: it is the Type field
// that is wrong.
//
// ===========================================================================
// SS OUTCOME POLICY -- FLAT, UNLIKE THE SCAN'S
// ===========================================================================
//
// pcie_enum_scan needs two policies because TXN_UR means "nothing here to
// enumerate" during the probe and "fault" one transaction later. THAT SPLIT DOES
// NOT EXIST HERE. Every transaction this module issues goes to a device that has
// already answered its Vendor-ID probe and its Header Type read, so any
// TXN_UR / TXN_CA / TXN_CRS_EXHAUSTED / TXN_TIMEOUT is a fault -- a device that
// answered the probe and then stopped is broken, not absent.
//
// One classifier, used from all seven response states, so they cannot drift.
//
// ===========================================================================
// SS NO TIMERS, AND THE ANNOTATION THAT IS NOT CONTROL FLOW
// ===========================================================================
//
// The CRS backoff lives in pcie_cfg_txn; the completion timeout lives in
// tlp_request_tracker. This module waits indefinitely on cmd_ready_i and
// rsp_valid_i and has no counter of its own.
//
// tx_fc_blocked_i is sampled ONLY when a TXN_TIMEOUT is reported: it sets
// err_credit_blocked_o and (sec 63 #7f #19) selects ENUM_ERR_CREDIT_STARVED
// over ENUM_ERR_TIMEOUT. IT APPEARS IN NO NEXT-STATE EXPRESSION -- inherited
// verbatim from pcie_enum_scan, for the same reason: tlp_request_tracker measures
// per-tag age from ALLOCATION (that module's header, :39) and allocation precedes
// the credit gate (tlp_layer.sv:280), so a request starved of credit for longer
// than CPL_TIMEOUT_CYCLES times out WITHOUT EVER HAVING BEEN TRANSMITTED and is
// indistinguishable from a dead device. err_credit_blocked_o is the most a client
// can be told: "this timeout smells like credit". A credit signal steering
// control flow is the watchdog mistake this design forbids.
//
// ===========================================================================
// SS ⭐ THIS MODULE HAS NO pcie_rq_tag_i PORT, AND THAT IS THE POINT
// ===========================================================================
//
// Commit 2b-1's Finding 1 -- a tag strobe is not evidence of transmission -- is
// STRUCTURAL here rather than a matter of discipline. The tag terminates at
// pcie_enum_top and goes only to the primitive. There is no wire on which this
// module could observe a tag even by mistake, exactly as pcie_enum_scan has had
// since the hoist. No assertion in either BAR bench may reference a tag value
// either (the tracker recycles a tag the moment a completion carrying Request
// Completed retires it -- PG213 :4257).
//
// ===========================================================================
// SS ONE FAULT SSE DOES NOT NAME
// ===========================================================================
//
// ENUM_ERR_BAR_ADDR32 is beyond docs/predictions/SPEC_PREDICTIONS_ENUM.md SSE's named fault set,
// and is recorded here rather than left implicit. MEM_BAR_BASE is 64-bit, and
// setting it above 4 GB is a legitimate parameterization for a device with only
// 64-bit BARs. A 32-BIT BAR CANNOT HOLD SUCH AN ADDRESS. The alternative to
// faulting is truncating the upper half, which produces a BAR overlapping
// whatever sits low in memory -- the same silent-overlap failure SSE.7.1 forbids
// for allocator wraparound, arriving by a different route. It is a DISTINCT code
// from ENUM_ERR_BAR_WINDOW: there the window ran out, here the window is fine
// and the register is too narrow to name the address.
//
// Guards use $warning, never $error: a procedural $error maps to $stop under the
// simulator, which would abort the shared multi-test process.
// ---------------------------------------------------------------------------
`timescale 1ns/1ps
module pcie_enum_bar
  import pcie_rq_rc_pkg::*;
  import pcie_enum_pkg::*;
#(
    // Where memory BARs are allocated from, ascending.
    //
    // ⚠️ MEM_BAR_BASE, NEVER "BAR_BASE". That name is taken by
    // tlp_bar_decoder.sv:4 / tlp_layer.sv:15 for the ENDPOINT-SIDE decode
    // aperture -- inbound CQ, tied off in this design. Different concept,
    // opposite direction, and an easy and expensive thing to confuse.
    parameter logic [63:0] MEM_BAR_BASE   = 64'h0000_0000_8000_0000,
    // Allocating past MEM_BAR_BASE + MEM_BAR_WINDOW is a sticky ERROR and never
    // a wraparound. A wrapped allocation produces overlapping BARs that appear
    // to enumerate successfully and then fail much later, in Stage F, as data
    // corruption.
    parameter logic [63:0] MEM_BAR_WINDOW = 64'h0000_0000_1000_0000
) (
    input  logic                        clk_i,
    input  logic                        rst_i,

    // ---- control -------------------------------------------------------------
    // LEVEL, not a pulse: pcie_enum_top drives it from bar_enable_i && scan_done_o.
    // Sampled in S_IDLE only.
    input  logic                        bar_start_i,
    // The scan's verdict on the device. This module configures a device only if
    // it is present AND its header is Type 0; an absent device has nothing to
    // configure and a Type 1 bridge is Stage D's problem. Both are NORMAL exits
    // with enum_done_o high and bar_count_o zero -- neither is an error, exactly
    // as pcie_enum_scan reports them.
    input  logic                        device_present_i,
    input  logic                        unsupported_device_i,

    // ---- annotation input (NOT control flow) -------------------------------
    input  logic                        tx_fc_blocked_i,

    // ---- status surface ------------------------------------------------------
    // Every output below is stable from the cycle enum_done_o asserts until
    // reset. There is no re-entry: enumeration is single-shot after link-up.
    output logic                        bar_busy_o,
    output logic                        enum_done_o,
    output logic                        bar_error_o,
    output enum_error_e                 bar_error_code_o,
    output logic                        err_credit_blocked_o,

    // BARs FOUND, not registers consumed. A 64-bit pair occupies registers N and
    // N+1 but is ONE BAR and counts once. "count" is ambiguous in English and
    // unambiguous here; SSE.7.4 pins it, and the mutation "64-bit pair treated as
    // two 32-bit BARs" is caught by it reporting 2 where 1 is expected.
    output logic [3:0]                  bar_count_o,
    // Indexed by BAR slot in discovery order, NOT by register number.
    output logic [BAR_SLOTS-1:0]        bar_valid_o,
    output logic [BAR_SLOTS-1:0]        bar_is_64_o,
    output logic [BAR_SLOTS-1:0]        bar_prefetch_o,
    output logic [BAR_SLOTS*64-1:0]     bar_size_o,
    output logic [BAR_SLOTS*64-1:0]     bar_addr_o,
    // The "log" half of "skip and log" for I/O BARs. Indexed by CANDIDATE
    // REGISTER, not by BAR slot: bit k means register CFG_REG_BAR_FIRST + k came
    // back with bit 0 set. An I/O BAR consumes no slot, so a slot index could not
    // name it.
    output logic [BAR_SLOTS-1:0]        io_bar_mask_o,

    // ---- pcie_cfg_txn command port -------------------------------------------
    // cmd_bdf_i is NOT driven from here. The target BDF is a property of the
    // device, not of the phase configuring it, so pcie_enum_top wires it from
    // the scan's device_bdf_o for every stage.
    output logic                        cmd_valid_o,
    input  logic                        cmd_ready_i,
    output logic                        cmd_write_o,
    output logic [5:0]                  cmd_reg_num_o,
    output logic [3:0]                  cmd_ext_reg_o,
    output logic [3:0]                  cmd_first_be_o,
    output logic [31:0]                 cmd_wdata_o,

    // ---- pcie_cfg_txn response port ------------------------------------------
    input  logic                        rsp_valid_i,
    output logic                        rsp_ready_o,
    input  txn_outcome_e                rsp_outcome_i,
    input  logic [31:0]                 rsp_rdata_i
);

  // -------------------------------------------------------------------------
  // The allocator's upper bound, computed in 65 bits so the parameter sum
  // cannot itself wrap and quietly widen the window to everything.
  // -------------------------------------------------------------------------
  localparam logic [64:0] MEM_BAR_LIMIT =
      {1'b0, MEM_BAR_BASE} + {1'b0, MEM_BAR_WINDOW};
  // A 32-bit Base Address register can name addresses strictly below 4 GB.
  localparam logic [64:0] ADDR32_LIMIT = 65'h0_0000_0001_0000_0000;

  // -------------------------------------------------------------------------
  // Sequencer.
  //
  // Two states per transaction throughout -- one offering the command, one
  // classifying the response -- rather than one state with a "what am I waiting
  // for" register. Seven transaction pairs is more states than a microcoded form
  // would need, but every arrow in the header's diagram is then a literal
  // transition a reader can check by eye, and the mutation set targets specific
  // arrows.
  // -------------------------------------------------------------------------
  typedef enum logic [4:0] {
    S_IDLE,            // waiting for bar_start_i
    S_CHECK,           // is this device configurable at all?
    S_SIZE_WR,         // all-ones -> register cand_r
    S_SIZE_WR_RSP,
    S_SIZE_RD,         // read back register cand_r
    S_SIZE_RD_RSP,     // ⭐ the decode: I/O / unimplemented / type / size
    S_UP_WR,           // 64-bit only: all-ones -> register cand_r + 1
    S_UP_WR_RSP,
    S_UP_RD,           // 64-bit only: read back register cand_r + 1
    S_UP_RD_RSP,       // combine the halves, size the pair, allocate
    S_ASSIGN_LO,       // write the assigned address, low half
    S_ASSIGN_LO_RSP,
    S_ASSIGN_UP,       // 64-bit only: write the assigned address, high half
    S_ASSIGN_UP_RSP,
    S_CMD_WR,          // ⭐ the last transaction of enumeration
    S_CMD_WR_RSP,
    S_DONE,            // terminal, no error
    S_ERROR            // terminal, sticky, reset-only
  } bar_state_e;

  bar_state_e  state_r;

  logic [5:0]  cand_r;              // candidate register, CFG_REG_BAR_FIRST..LAST
  logic [2:0]  slot_r;              // next BAR slot to fill
  logic [64:0] cursor_r;            // allocator cursor, 65-bit to catch overflow
  logic [31:0] enc_lo_r;            // masked lower half of a 64-bit pair
  logic        is64_r;              // the BAR being assigned right now is 64-bit
  logic        prefetch_r;
  logic [63:0] size_r;
  logic [63:0] addr_r;
  enum_error_e error_code_r;
  logic        credit_blocked_r;

  logic [BAR_SLOTS-1:0] io_bar_mask_r;

  logic        slot_valid_r    [BAR_SLOTS];
  logic        slot_is64_r     [BAR_SLOTS];
  logic        slot_prefetch_r [BAR_SLOTS];
  logic [63:0] slot_size_r     [BAR_SLOTS];
  logic [63:0] slot_addr_r     [BAR_SLOTS];

  // -------------------------------------------------------------------------
  // Readback decode, combinational off the response port.
  // [PCI3] SS6.2.5.1 p.225-226; masks at :11205 (memory) and :11187 (I/O).
  // -------------------------------------------------------------------------
  wire        rb_is_io     = rsp_rdata_i[BAR_BIT_IO];
  wire [1:0]  rb_type      = rsp_rdata_i[BAR_TYPE_LSB +: 2];
  wire        rb_prefetch  = rsp_rdata_i[BAR_BIT_PREFETCH];
  wire [31:0] rb_enc_mem   = rsp_rdata_i & BAR_MEM_MASK;
  wire        rb_zero      = (rsp_rdata_i == 32'h0);
  wire        rb_type_bad  = (rb_type == BAR_TYPE_RESERVED1) ||
                             (rb_type == BAR_TYPE_RESERVED3);

  // A 32-bit memory BAR's size, from the masked readback.  [PCI3] p.226 :11222
  wire [63:0] size32 = {32'h0, (~rb_enc_mem) + 32'd1};
  // A 64-bit pair's size. The mask applies to the LOWER half only: bits 3:0 of
  // the pair are the lower register's read-only field, and the upper register is
  // 32 ordinary address bits with no reserved field at all.  SSE.3.2
  wire [63:0] enc64  = {rsp_rdata_i, enc_lo_r};
  wire [63:0] size64 = (~enc64) + 64'd1;

  // -------------------------------------------------------------------------
  // Allocation, combinational from cursor_r and a candidate size.
  //
  // Everything is 65-bit so that a round-up or an end-of-range that overflows 64
  // bits is VISIBLE to the checks below rather than wrapping into a plausible
  // low address. SSE.7.1's "never silent wraparound", enforced in the arithmetic
  // rather than only in the policy.
  // -------------------------------------------------------------------------
  // The BAR being placed is a pair exactly when the upper half has just been
  // read back; every other allocation is of a 32-bit BAR.
  wire        pair_now     = (state_r == S_UP_RD_RSP);
  wire [63:0] alloc_size   = pair_now ? size64 : size32;

  wire [64:0] alloc_size65 = {1'b0, alloc_size};
  // "power of two in size and are naturally aligned" -- [PCI3] p.226 :11226.
  wire [64:0] align_mask65 = alloc_size65 - 65'd1;
  wire [64:0] alloc_addr65 = (cursor_r + align_mask65) & ~align_mask65;
  wire [64:0] alloc_end65  = alloc_addr65 + alloc_size65;

  // A decoded size is legal only if it clears the PCIe floor AND is a power of
  // two. Both mean "the decode itself is wrong" -- see ENUM_ERR_BAR_SIZE.
  wire        size_legal   = (alloc_size >= BAR_MEM_MIN_BYTES) &&
                             ((alloc_size & (alloc_size - 64'd1)) == 64'd0);
  wire        window_bad   = (alloc_end65 > MEM_BAR_LIMIT);
  // Keyed on pair_now rather than on is64_r, because on the 32-bit path is64_r
  // has not been written yet when the check is evaluated.
  wire        addr32_bad   = !pair_now && (alloc_end65 > ADDR32_LIMIT);

  // -------------------------------------------------------------------------
  // Every non-OK outcome is a fault in this phase -- see SS OUTCOME POLICY.
  // Written once so the seven response states cannot drift apart.
  // -------------------------------------------------------------------------
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
      // TXN_UR and anything unforeseen. A device that answered its probe has no
      // business rejecting a legal configuration access.
      default:           fault_code = ENUM_ERR_UR_POST_PROBE;
    endcase
  endfunction

  // The next candidate, and whether that lands past the end of the window.
  // A 64-bit pair consumes TWO registers -- SSE.3, [PCI3] Table 6-4 p.226 :11207.
  wire [6:0] cand_after_one  = {1'b0, cand_r} + 7'd1;
  wire [6:0] cand_after_pair = {1'b0, cand_r} + 7'd2;

  always_ff @(posedge clk_i) begin
    if (rst_i) begin
      state_r          <= S_IDLE;
      cand_r           <= CFG_REG_BAR_FIRST;
      slot_r           <= 3'd0;
      cursor_r         <= {1'b0, MEM_BAR_BASE};
      enc_lo_r         <= 32'h0;
      is64_r           <= 1'b0;
      prefetch_r       <= 1'b0;
      size_r           <= 64'h0;
      addr_r           <= 64'h0;
      error_code_r     <= ENUM_ERR_NONE;
      credit_blocked_r <= 1'b0;
      io_bar_mask_r    <= '0;
      for (int i = 0; i < BAR_SLOTS; i++) begin
        slot_valid_r[i]    <= 1'b0;
        slot_is64_r[i]     <= 1'b0;
        slot_prefetch_r[i] <= 1'b0;
        slot_size_r[i]     <= 64'h0;
        slot_addr_r[i]     <= 64'h0;
      end
    end else begin
      unique case (state_r)

        S_IDLE: begin
          if (bar_start_i) begin
            cand_r           <= CFG_REG_BAR_FIRST;
            slot_r           <= 3'd0;
            cursor_r         <= {1'b0, MEM_BAR_BASE};
            error_code_r     <= ENUM_ERR_NONE;
            credit_blocked_r <= 1'b0;
            io_bar_mask_r    <= '0;
            state_r          <= S_CHECK;
          end
        end

        // A device that is absent, or whose header is Type 1, has nothing this
        // module can configure. Terminal and NOT an error -- and, critically,
        // NO TRANSACTION IS EMITTED, including no Command write: there is no
        // configured device to enable.
        S_CHECK: begin
          state_r <= (device_present_i && !unsupported_device_i) ? S_SIZE_WR
                                                                 : S_DONE;
        end

        // ---- size: all-ones write ------------------------------------------
        S_SIZE_WR: if (cmd_ready_i) state_r <= S_SIZE_WR_RSP;

        S_SIZE_WR_RSP: begin
          if (rsp_valid_i) begin
            if (rsp_outcome_i == TXN_OK) begin
              state_r <= S_SIZE_RD;
            end else begin
              error_code_r     <= fault_code(rsp_outcome_i, tx_fc_blocked_i);
              credit_blocked_r <= (rsp_outcome_i == TXN_TIMEOUT) && tx_fc_blocked_i;
              state_r          <= S_ERROR;
            end
          end
        end

        // ---- size: readback and decode --------------------------------------
        S_SIZE_RD: if (cmd_ready_i) state_r <= S_SIZE_RD_RSP;

        S_SIZE_RD_RSP: begin
          if (rsp_valid_i) begin
            if (rsp_outcome_i != TXN_OK) begin
              error_code_r     <= fault_code(rsp_outcome_i, tx_fc_blocked_i);
              credit_blocked_r <= (rsp_outcome_i == TXN_TIMEOUT) && tx_fc_blocked_i;
              state_r          <= S_ERROR;
            end else if (rb_is_io) begin
              // SKIP AND LOG. No assignment: this design programs no I/O
              // allocator, and skipping is inert because Command bit 0 stays 0
              // so the device is never told to decode I/O space. SSE.7.2.
              io_bar_mask_r[cand_r - CFG_REG_BAR_FIRST] <= 1'b1;
              if (cand_after_one > {1'b0, CFG_REG_BAR_LAST}) begin
                state_r <= S_CMD_WR;
              end else begin
                cand_r  <= cand_after_one[5:0];
                state_r <= S_SIZE_WR;
              end
            end else if (rb_zero) begin
              // Unimplemented: hardwired to zero, consumes no address space.
              if (cand_after_one > {1'b0, CFG_REG_BAR_LAST}) begin
                state_r <= S_CMD_WR;
              end else begin
                cand_r  <= cand_after_one[5:0];
                state_r <= S_SIZE_WR;
              end
            end else if (rb_type_bad) begin
              error_code_r <= ENUM_ERR_BAR_TYPE;
              state_r      <= S_ERROR;
            end else if (rb_type == BAR_TYPE_64BIT) begin
              // A pair needs a register to pair WITH -- see the header.
              if (cand_r == CFG_REG_BAR_LAST) begin
                error_code_r <= ENUM_ERR_BAR_TYPE;
                state_r      <= S_ERROR;
              end else begin
                enc_lo_r   <= rb_enc_mem;
                prefetch_r <= rb_prefetch;
                is64_r     <= 1'b1;
                state_r    <= S_UP_WR;
              end
            end else begin
              // 32-bit memory BAR: size it and place it now.
              is64_r     <= 1'b0;
              prefetch_r <= rb_prefetch;
              if (!size_legal) begin
                error_code_r <= ENUM_ERR_BAR_SIZE;
                state_r      <= S_ERROR;
              end else if (window_bad) begin
                error_code_r <= ENUM_ERR_BAR_WINDOW;
                state_r      <= S_ERROR;
              end else if (addr32_bad) begin
                error_code_r <= ENUM_ERR_BAR_ADDR32;
                state_r      <= S_ERROR;
              end else begin
                size_r  <= alloc_size;
                addr_r  <= alloc_addr65[63:0];
                state_r <= S_ASSIGN_LO;
              end
            end
          end
        end

        // ---- size: the upper half of a 64-bit pair ---------------------------
        S_UP_WR: if (cmd_ready_i) state_r <= S_UP_WR_RSP;

        S_UP_WR_RSP: begin
          if (rsp_valid_i) begin
            if (rsp_outcome_i == TXN_OK) begin
              state_r <= S_UP_RD;
            end else begin
              error_code_r     <= fault_code(rsp_outcome_i, tx_fc_blocked_i);
              credit_blocked_r <= (rsp_outcome_i == TXN_TIMEOUT) && tx_fc_blocked_i;
              state_r          <= S_ERROR;
            end
          end
        end

        S_UP_RD: if (cmd_ready_i) state_r <= S_UP_RD_RSP;

        S_UP_RD_RSP: begin
          if (rsp_valid_i) begin
            if (rsp_outcome_i != TXN_OK) begin
              error_code_r     <= fault_code(rsp_outcome_i, tx_fc_blocked_i);
              credit_blocked_r <= (rsp_outcome_i == TXN_TIMEOUT) && tx_fc_blocked_i;
              state_r          <= S_ERROR;
            end else if (!size_legal) begin
              error_code_r <= ENUM_ERR_BAR_SIZE;
              state_r      <= S_ERROR;
            end else if (window_bad) begin
              error_code_r <= ENUM_ERR_BAR_WINDOW;
              state_r      <= S_ERROR;
            end else begin
              // addr32_bad cannot apply: this BAR is 64 bits wide by decode.
              size_r  <= alloc_size;
              addr_r  <= alloc_addr65[63:0];
              state_r <= S_ASSIGN_LO;
            end
          end
        end

        // ---- assign ----------------------------------------------------------
        S_ASSIGN_LO: if (cmd_ready_i) state_r <= S_ASSIGN_LO_RSP;

        S_ASSIGN_LO_RSP: begin
          if (rsp_valid_i) begin
            if (rsp_outcome_i != TXN_OK) begin
              error_code_r     <= fault_code(rsp_outcome_i, tx_fc_blocked_i);
              credit_blocked_r <= (rsp_outcome_i == TXN_TIMEOUT) && tx_fc_blocked_i;
              state_r          <= S_ERROR;
            end else if (is64_r) begin
              state_r <= S_ASSIGN_UP;
            end else begin
              // Commit the slot only once the device has actually taken the
              // address. bar_valid_o means PROGRAMMED, not merely decoded.
              slot_valid_r[slot_r]    <= 1'b1;
              slot_is64_r[slot_r]     <= 1'b0;
              slot_prefetch_r[slot_r] <= prefetch_r;
              slot_size_r[slot_r]     <= size_r;
              slot_addr_r[slot_r]     <= addr_r;
              slot_r                  <= slot_r + 3'd1;
              cursor_r                <= {1'b0, addr_r} + {1'b0, size_r};
              if (cand_after_one > {1'b0, CFG_REG_BAR_LAST}) begin
                state_r <= S_CMD_WR;
              end else begin
                cand_r  <= cand_after_one[5:0];
                state_r <= S_SIZE_WR;
              end
            end
          end
        end

        S_ASSIGN_UP: if (cmd_ready_i) state_r <= S_ASSIGN_UP_RSP;

        S_ASSIGN_UP_RSP: begin
          if (rsp_valid_i) begin
            if (rsp_outcome_i != TXN_OK) begin
              error_code_r     <= fault_code(rsp_outcome_i, tx_fc_blocked_i);
              credit_blocked_r <= (rsp_outcome_i == TXN_TIMEOUT) && tx_fc_blocked_i;
              state_r          <= S_ERROR;
            end else begin
              slot_valid_r[slot_r]    <= 1'b1;
              slot_is64_r[slot_r]     <= 1'b1;
              slot_prefetch_r[slot_r] <= prefetch_r;
              slot_size_r[slot_r]     <= size_r;
              slot_addr_r[slot_r]     <= addr_r;
              slot_r                  <= slot_r + 3'd1;
              cursor_r                <= {1'b0, addr_r} + {1'b0, size_r};
              // TWO registers, hence N += 2.
              if (cand_after_pair > {1'b0, CFG_REG_BAR_LAST}) begin
                state_r <= S_CMD_WR;
              end else begin
                cand_r  <= cand_after_pair[5:0];
                state_r <= S_SIZE_WR;
              end
            end
          end
        end

        // ---- enable ----------------------------------------------------------
        // Reachable ONLY from the three candidate-advance paths above, once
        // cand_r has passed CFG_REG_BAR_LAST. That is the ordering invariant,
        // structurally.
        S_CMD_WR: if (cmd_ready_i) state_r <= S_CMD_WR_RSP;

        S_CMD_WR_RSP: begin
          if (rsp_valid_i) begin
            if (rsp_outcome_i == TXN_OK) begin
              state_r <= S_DONE;
            end else begin
              error_code_r     <= fault_code(rsp_outcome_i, tx_fc_blocked_i);
              credit_blocked_r <= (rsp_outcome_i == TXN_TIMEOUT) && tx_fc_blocked_i;
              state_r          <= S_ERROR;
            end
          end
        end

        // Terminal states hold until reset. Enumeration is single-shot after
        // link-up; a status surface that could be re-entered would let a
        // consumer sample it mid-re-enumeration.
        S_DONE:  state_r <= S_DONE;
        S_ERROR: state_r <= S_ERROR;
      endcase
    end
  end

  // -------------------------------------------------------------------------
  // Command port.
  //
  // Constants are driven from HERE rather than tied off in the parent, so that
  // pcie_enum_top's handoff mux sees a COMPLETE command port from each stage and
  // needs no per-stage constants of its own -- the same rule pcie_enum_scan
  // follows since the hoist.
  // -------------------------------------------------------------------------
  assign cmd_valid_o = (state_r == S_SIZE_WR)   || (state_r == S_SIZE_RD)   ||
                       (state_r == S_UP_WR)     || (state_r == S_UP_RD)     ||
                       (state_r == S_ASSIGN_LO) || (state_r == S_ASSIGN_UP) ||
                       (state_r == S_CMD_WR);

  assign cmd_write_o = (state_r == S_SIZE_WR)   || (state_r == S_UP_WR)     ||
                       (state_r == S_ASSIGN_LO) || (state_r == S_ASSIGN_UP) ||
                       (state_r == S_CMD_WR);

  // The upper half of a pair is always cand_r + 1; everything else addresses
  // cand_r, except the Command register which is fixed at 1.
  assign cmd_reg_num_o =
      (state_r == S_CMD_WR)                          ? CFG_REG_COMMAND_STATUS :
      ((state_r == S_UP_WR) || (state_r == S_UP_RD) ||
       (state_r == S_ASSIGN_UP))                     ? cand_after_one[5:0]    :
                                                       cand_r;

  assign cmd_ext_reg_o = CFG_EXT_REG_NONE;

  // ⭐ 0011 FOR THE COMMAND WRITE, 1111 EVERYWHERE ELSE.
  //
  // The Command register is the LOWER half of the Dword at offset 04h; the upper
  // half is Status ([BASE] Figure 7-5 p.491), several of whose bits are
  // write-1-to-clear. A whole-Dword write would clear sticky error state the
  // Root Complex has not read yet.
  //
  // !! THIS IS ALSO THE ONE FIELD THAT MAKES THE HANDOFF MUX OBSERVABLE. See
  // pcie_enum_top, SS PROVING EXACTLY ONE STAGE IS LIVE.
  assign cmd_first_be_o = (state_r == S_CMD_WR) ? CFG_BE_LOWER_HALF : CFG_BE_DWORD;

  assign cmd_wdata_o =
      (state_r == S_CMD_WR)    ? CMD_ENABLE_VALUE   :
      (state_r == S_ASSIGN_LO) ? addr_r[31:0]       :
      (state_r == S_ASSIGN_UP) ? addr_r[63:32]      :
                                 BAR_PROBE_ALL_ONES;

  // A real handshake, not a strobe: the primitive holds rsp_valid_o until it is
  // consumed, so an unready consumer cannot miss an outcome.
  assign rsp_ready_o = (state_r == S_SIZE_WR_RSP)   || (state_r == S_SIZE_RD_RSP)   ||
                       (state_r == S_UP_WR_RSP)     || (state_r == S_UP_RD_RSP)     ||
                       (state_r == S_ASSIGN_LO_RSP) || (state_r == S_ASSIGN_UP_RSP) ||
                       (state_r == S_CMD_WR_RSP);

  // -------------------------------------------------------------------------
  // Status surface.
  // -------------------------------------------------------------------------
  assign bar_busy_o           = (state_r != S_IDLE) && (state_r != S_DONE) &&
                                (state_r != S_ERROR);
  assign enum_done_o          = (state_r == S_DONE);
  assign bar_error_o          = (state_r == S_ERROR);
  assign bar_error_code_o     = error_code_r;
  assign err_credit_blocked_o = credit_blocked_r;
  assign bar_count_o          = {1'b0, slot_r};
  assign io_bar_mask_o        = io_bar_mask_r;

  always_comb begin
    bar_valid_o    = '0;
    bar_is_64_o    = '0;
    bar_prefetch_o = '0;
    bar_size_o     = '0;
    bar_addr_o     = '0;
    for (int i = 0; i < BAR_SLOTS; i++) begin
      bar_valid_o[i]         = slot_valid_r[i];
      bar_is_64_o[i]         = slot_is64_r[i];
      bar_prefetch_o[i]      = slot_prefetch_r[i];
      bar_size_o[i*64 +: 64] = slot_size_r[i];
      bar_addr_o[i*64 +: 64] = slot_addr_r[i];
    end
  end

  // -------------------------------------------------------------------------
  // Elaboration guards. $warning, never $error -- see the header.
  // -------------------------------------------------------------------------
  initial begin
    // SystemVerilog has no adjacent-string-literal concatenation, so these are
    // one line each rather than the wrapped C form.
    if (MEM_BAR_WINDOW == 64'd0)
      $warning("pcie_enum_bar: MEM_BAR_WINDOW is 0 -- every memory BAR will fault with ENUM_ERR_BAR_WINDOW.");
    if ((MEM_BAR_BASE & (BAR_MEM_MIN_BYTES - 64'd1)) != 64'd0)
      $warning("pcie_enum_bar: MEM_BAR_BASE %0h is not 128-byte aligned; the first BAR lands above it via the alignment round-up.", MEM_BAR_BASE);
  end

endmodule
