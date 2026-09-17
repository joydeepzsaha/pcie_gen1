// ---------------------------------------------------------------------------
// tlp_request_tracker -- tag allocation, completion matching, and (since this
// commit) the PCIe Completion Timeout mechanism.
//
// ---------------------------------------------------------------------------
// SS COMPLETION TIMEOUT (PCIe Base 2.1 SS2.8, p.152)
// ---------------------------------------------------------------------------
//
// SS2.8 makes the Completion Timeout mechanism the REQUESTER's responsibility
// and names Root Complexes explicitly: "PCI Express device Functions that issue
// Requests requiring Completions must implement the Completion Timeout
// mechanism", and it "is activated for each Request that requires one or more
// Completions when the Request is transmitted". Before this commit a tag was
// freed only by a matching completion or by reset, so a request to a device
// that never answered held its tag forever and, after TAG_COUNT such requests,
// wedged the requester with no error output.
//
// SS TIMEOUT VALUE -- CPL_TIMEOUT_CYCLES is SIM-PRACTICAL, NOT SPEC-REAL.
//
//   The architected values live in the Device Control 2 register, SS7.8.16
//   Table 7-25 (pp.549-550): a Function that does not implement Completion
//   Timeout programmability "must hardwire this field to 0000b and is required
//   to implement a timeout value in the range 50 us to 50 ms", and the spec
//   adds "It is strongly recommended that the Completion Timeout mechanism not
//   expire in less than 10 ms". The programmable ranges are A 50 us-10 ms,
//   B 10 ms-250 ms, C 250 ms-4 s, D 4 s-64 s.
//
//   SS63 #7e, CONFORMANCE DEFECT #7 -- THE DEFAULT WAS BELOW THE ARCHITECTED
//   MINIMUM, NOT MERELY BELOW THE RECOMMENDATION.
//
//   The default was 4096 cycles. The paragraph here used to justify that as
//   "16.4 us at a 250 MHz link clock ... chosen so a simulation can observe a
//   timeout in a few microseconds", which conceded only that it sat below the
//   RECOMMENDED 10 ms floor. That understated it. SS7.8.16 Table 7-25 makes
//   50 us a REQUIREMENT, not a recommendation -- "is required to implement a
//   timeout value in the range 50 us to 50 ms" -- and at the 8 ns clock this
//   design actually runs at, 4096 cycles is 32.8 us. Below the floor of the
//   required range, so non-conformant at the rate the RTL is built for.
//
//   The default is now 6250 = 50 us / 8 ns: the architected MINIMUM, exactly.
//   It is the smallest value that is conformant at this clock, which keeps a
//   simulation able to observe a timeout while no longer shipping a
//   spec-violating default.
//
//   ⚠️ IT WAS MEASURED, NOT ARGUED. SS63 #7e timed a real CfgRd0 -> CplD round
//   trip through two PHYs and the codec bridge at 5122 cycles = 41.0 us. The
//   old 4096 expired BEFORE that legitimate completion returned and reported
//   ENUM_ERR_TIMEOUT -- the link was inside spec and the timeout was not.
//
//   ⚠️ 6250 IS THE MINIMUM AND MINIMUMS ARE TIGHT. It clears that measured
//   round trip by only 1128 cycles. tb_pcie_fullstack deliberately overrides to
//   65536 for exactly this reason; a bench that needs headroom must ask for it
//   rather than rely on the shipped floor.
//
//   A value near the "strongly recommended" 10 ms (1 250 000 cycles at 8 ns)
//   and a Device Control 2 register to program it remain Stage-H work.
//
//   CPL_TIMEOUT_CYCLES = 0 DISABLES the mechanism entirely and restores exactly
//   the pre-timeout behaviour. This mirrors an architected control: SS7.8.16
//   bit 4, "Completion Timeout Disable -- When Set, this bit disables the
//   Completion Timeout mechanism" (p.550).
//
// SS TIMER SEMANTICS. Per-tag age is measured from ALLOCATION, and the timer
//   RESTARTS on every matched completion handshake for that tag -- including a
//   partial completion of a split read, and including one the malformed-CPL
//   guard below rejects (that CPL leaves the tag in flight). SS2.8 is SILENT on
//   whether the timer may restart: its multi-completion Note governs only
//   whether returned data may be kept or discarded, not the timer. Restart is
//   therefore implementation-defined rather than spec-permitted, and this is
//   the forgiving choice.
//
// SS TAG DISPOSITION: QUARANTINE, NOT IMMEDIATE RECYCLE.
//
//   FREE --> IN_FLIGHT --(timeout)--> ZOMBIE --> FREE
//
//   A ZOMBIE tag is NOT allocatable, and it still MATCHES a late completion --
//   which it drains silently: no result on the result interface, no
//   unexpected_completion_o, no byte-count checking. It returns to FREE on
//   either a late completion whose last-CPL condition holds (the same
//   expression that drives result_last_o, i.e. RC descriptor bit 30 --
//   pcie_rc_if.sv:261) or a SECOND expiry of the same interval, whichever comes
//   first. Immediate recycle was rejected: a late completion landing on a
//   reused tag would be delivered against the WRONG request, silently.
//
//   Suppressing unexpected_completion_o for a zombie is a deliberate deviation.
//   A completion for a tag no longer outstanding is by construction an
//   Unexpected Completion (SS6.2.3.2.4.5, pp.374-375), but that section classes
//   it as an Advisory Non-Fatal Error and explicitly warns that reporting it
//   can "interfere with Requester recovery". late_cpl_valid_o carries strictly
//   more information than the generic unexpected report would have.
//
//   A timed-out request is FAILED. SS2.8 p.152: "If some, but not all,
//   requested data is returned before the Completion Timeout timer expires, the
//   Requester is permitted to keep or to discard the data that was returned."
//   Any partial data already delivered is the client FSM's to interpret; this
//   module's job is to free the tag safely and report the event.
//
// SS THE PAYLOAD IS NOT THIS MODULE'S PROBLEM. The completion port here is
//   HEADER-ONLY: one handshake carrying a header and a 13-bit byte count. A
//   completion's payload beats bypass the tracker entirely (tlp_layer.sv:
//   254-259, routed by route_completion_r). The zombie drain is therefore
//   beat-free, and the beats of a late completion are swallowed by the drain
//   that already exists in pcie_rc_if.sv:341-343 -- in S_IDLE with no result
//   behind them, which is exactly the state a zombie completion leaves. No
//   byte-accounting code is added anywhere by this mechanism.
//
// SS MECHANISM. One free-running 32-bit counter plus a per-tag allocation
//   timestamp, with the expiry check walked ROUND-ROBIN, one tag per cycle.
//   That costs a single subtractor and comparator instead of TAG_COUNT wide
//   ones. Worst-case detection latency is TAG_COUNT cycles, which is irrelevant
//   at 4096-cycle granularity. Modular subtraction makes the counter's 2^32
//   wraparound correct for any CPL_TIMEOUT_CYCLES < 2^31.
//
//   The scan and the completion path can address the same tag in the same
//   cycle, so the scan is GUARDED to be mutually exclusive with a matched
//   completion rather than relying on last-assignment-wins: a completion that
//   lands in the exact expiry cycle wins, and no false timeout is reported for
//   a tag that actually completed. Allocation cannot collide -- it only picks a
//   tag that is neither active nor zombie, and the scan only fires on one that
//   is.
//
// SS outstanding_o COUNTS ZOMBIES. It is documented as "non-posted requests
//   currently holding a tag" (pcie_rq_rc_top.sv:285-287) and a zombie is still
//   holding one -- it cannot be allocated. With CPL_TIMEOUT_CYCLES = 0 no tag
//   ever becomes a zombie and the count is bit-identical to the pre-timeout
//   popcount of active_r.
// ---------------------------------------------------------------------------
`timescale 1ns/1ps
module tlp_request_tracker
  import tlp_pkg::*;
#(
    parameter int TAG_COUNT = 32,
    parameter int CONTEXT_WIDTH = 16,
    // Cycles a tag may stay outstanding before it is timed out. 0 disables the
    // Completion Timeout mechanism entirely. See the header for why 4096 is a
    // simulation convenience and not a spec-conformant value.
    parameter int unsigned CPL_TIMEOUT_CYCLES = 32'd6250
) (
    input  logic                     clk_i,
    input  logic                     rst_i,
    input  logic                     extended_tag_enable_i,

    input  logic                     allocate_valid_i,
    output logic                     allocate_ready_o,
    input  logic [15:0]              allocate_requester_id_i,
    input  logic [12:0]              allocate_byte_count_i,
    input  logic [63:0]              allocate_address_i,
    input  logic [CONTEXT_WIDTH-1:0] allocate_context_i,
    input  logic                     allocate_expects_data_i,
    output logic [7:0]               allocate_tag_o,

    input  logic                     completion_valid_i,
    output logic                     completion_ready_o,
    input  tlp_header_t              completion_header_i,
    input  logic [12:0]              completion_payload_bytes_i,

    output logic                     result_valid_o,
    input  logic                     result_ready_i,
    output logic [CONTEXT_WIDTH-1:0] result_context_o,
    output logic [2:0]               result_status_o,
    output logic                     result_last_o,
    output logic                     unexpected_completion_o,
    output tlp_error_e               completion_error_code_o,

    // Completion Timeout sideband. Both are 1-cycle strobes with the tag valid
    // in the same cycle -- the same correlation discipline as the allocation
    // tap tlp_layer.sv:184-185 raises as pcie_rq_tag_o / pcie_rq_tag_vld_o.
    // cpl_timeout_valid_o fires once per IN_FLIGHT -> ZOMBIE transition;
    // late_cpl_valid_o fires when a ZOMBIE tag drains a late completion.
    output logic                     cpl_timeout_valid_o,
    output logic [7:0]               cpl_timeout_tag_o,
    output logic                     late_cpl_valid_o,
    output logic [7:0]               late_cpl_tag_o,

    output logic [$clog2(TAG_COUNT+1)-1:0] outstanding_o
);

  logic [TAG_COUNT-1:0] active_r;
  logic [15:0] requester_id_r [0:TAG_COUNT-1];
  logic [12:0] remaining_r [0:TAG_COUNT-1];
  logic [CONTEXT_WIDTH-1:0] context_r [0:TAG_COUNT-1];
  logic expects_data_r [0:TAG_COUNT-1];
  logic [6:0] next_lower_address_r [0:TAG_COUNT-1];
  logic result_valid_r;
  logic [CONTEXT_WIDTH-1:0] result_context_r;
  logic [2:0] result_status_r;
  logic result_last_r;
  logic unexpected_r;
  localparam int TAG_INDEX_WIDTH = TAG_COUNT <= 1 ? 1 : $clog2(TAG_COUNT);
  integer search_index;
  integer reset_index;
  integer active_count;
  logic tag_found;
  logic completion_match;
  logic [TAG_INDEX_WIDTH-1:0] completion_index;

  // ---- Completion Timeout state (see header) --------------------------------
  localparam logic [31:0] TIMEOUT_LIMIT = CPL_TIMEOUT_CYCLES;
  logic [TAG_COUNT-1:0] zombie_r;              // timed out, quarantined
  logic [31:0] cycle_counter_r;                // free-running, wraps mod 2^32
  logic [31:0] alloc_time_r [0:TAG_COUNT-1];   // per-tag timer origin
  logic [TAG_INDEX_WIDTH-1:0] scan_index_r;    // round-robin expiry walk
  logic [31:0] scan_age;
  logic scan_expired;
  logic completion_fire;
  logic completion_last;

  always_comb begin
    tag_found = 1'b0;
    allocate_tag_o = '0;
    // A ZOMBIE tag is not allocatable: that quarantine is the whole point of
    // not recycling a timed-out tag immediately.
    for (search_index = 0; search_index < TAG_COUNT; search_index = search_index + 1) begin
      if (!tag_found && !active_r[search_index] && !zombie_r[search_index] &&
          (extended_tag_enable_i || search_index < 32)) begin
        tag_found = 1'b1;
        allocate_tag_o = search_index[7:0];
      end
    end
    allocate_ready_o = tag_found;

    // A ZOMBIE tag still matches, so a late completion is caught here rather
    // than falling through to the unexpected-completion branch.
    completion_match = 1'b0;
    completion_index = '0;
    for (search_index = 0; search_index < TAG_COUNT; search_index = search_index + 1) begin
      if (!completion_match && (active_r[search_index] || zombie_r[search_index]) &&
          completion_header_i.tag == search_index[7:0] &&
          completion_header_i.requester_id == requester_id_r[search_index]) begin
        completion_match = 1'b1;
        completion_index = search_index[TAG_INDEX_WIDTH-1:0];
      end
    end
    completion_ready_o = !result_valid_r || result_ready_i;
    completion_fire = completion_valid_i && completion_ready_o;

    // The last-CPL-of-request condition. Identical to what drives result_last_o
    // below, and to RC descriptor bit 30 (pcie_rc_if.sv:261) -- the zombie
    // release condition is deliberately the same test, not a second one.
    completion_last = !expects_data_r[completion_index] ||
                      completion_header_i.completion_status != TLP_CPL_SC ||
                      completion_payload_bytes_i >= remaining_r[completion_index];

    // Round-robin expiry check, one tag per cycle. Guarded against the
    // completion path so the two can never write the same tag in one cycle.
    scan_age = cycle_counter_r - alloc_time_r[scan_index_r];
    scan_expired = (TIMEOUT_LIMIT != 32'd0) &&
                   (active_r[scan_index_r] || zombie_r[scan_index_r]) &&
                   (scan_age >= TIMEOUT_LIMIT) &&
                   !(completion_fire && completion_match &&
                     completion_index == scan_index_r);

    // Zombies still hold their tag, so they still count as outstanding.
    active_count = 0;
    for (search_index = 0; search_index < TAG_COUNT; search_index = search_index + 1)
      active_count = active_count + (active_r[search_index] | zombie_r[search_index]);
    outstanding_o = active_count[$clog2(TAG_COUNT+1)-1:0];
  end

  assign result_valid_o = result_valid_r;
  assign result_context_o = result_context_r;
  assign result_status_o = result_status_r;
  assign result_last_o = result_last_r;
  assign unexpected_completion_o = unexpected_r;

  always_ff @(posedge clk_i) begin
    if (rst_i) begin
      active_r         <= '0;
      result_valid_r   <= 1'b0;
      result_context_r <= '0;
      result_status_r  <= '0;
      result_last_r    <= 1'b0;
      unexpected_r     <= 1'b0;
      completion_error_code_o <= TLP_ERR_NONE;
      zombie_r            <= '0;
      cycle_counter_r     <= '0;
      scan_index_r        <= '0;
      cpl_timeout_valid_o <= 1'b0;
      cpl_timeout_tag_o   <= '0;
      late_cpl_valid_o    <= 1'b0;
      late_cpl_tag_o      <= '0;
      for (reset_index = 0; reset_index < TAG_COUNT; reset_index = reset_index + 1) begin
        requester_id_r[reset_index] <= '0;
        remaining_r[reset_index]    <= '0;
        context_r[reset_index]      <= '0;
        expects_data_r[reset_index] <= 1'b0;
        next_lower_address_r[reset_index] <= '0;
        alloc_time_r[reset_index]   <= '0;
      end
    end else begin
      unexpected_r <= 1'b0;
      completion_error_code_o <= TLP_ERR_NONE;
      cpl_timeout_valid_o <= 1'b0;
      late_cpl_valid_o    <= 1'b0;
      cycle_counter_r <= cycle_counter_r + 32'd1;
      scan_index_r    <= (scan_index_r == TAG_INDEX_WIDTH'(TAG_COUNT - 1)) ?
                         '0 : scan_index_r + TAG_INDEX_WIDTH'(1);

      // Expiry walk. IN_FLIGHT -> ZOMBIE reports; ZOMBIE -> FREE is silent.
      // Either way the timestamp is rewritten, so the zombie interval is one
      // more full CPL_TIMEOUT_CYCLES rather than needing a second timer.
      if (scan_expired) begin
        alloc_time_r[scan_index_r] <= cycle_counter_r;
        if (active_r[scan_index_r]) begin
          active_r[scan_index_r]  <= 1'b0;
          zombie_r[scan_index_r]  <= 1'b1;
          cpl_timeout_valid_o     <= 1'b1;
          cpl_timeout_tag_o       <= 8'(scan_index_r);
        end else begin
          zombie_r[scan_index_r]            <= 1'b0;
          remaining_r[scan_index_r]         <= '0;
          expects_data_r[scan_index_r]      <= 1'b0;
          next_lower_address_r[scan_index_r] <= '0;
        end
      end

      if (result_valid_r && result_ready_i)
        result_valid_r <= 1'b0;

      if (allocate_valid_i && allocate_ready_o) begin
        alloc_time_r[allocate_tag_o[TAG_INDEX_WIDTH-1:0]]   <= cycle_counter_r;
        active_r[allocate_tag_o[TAG_INDEX_WIDTH-1:0]]       <= 1'b1;
        requester_id_r[allocate_tag_o[TAG_INDEX_WIDTH-1:0]] <= allocate_requester_id_i;
        remaining_r[allocate_tag_o[TAG_INDEX_WIDTH-1:0]]    <= allocate_byte_count_i;
        context_r[allocate_tag_o[TAG_INDEX_WIDTH-1:0]]      <= allocate_context_i;
        expects_data_r[allocate_tag_o[TAG_INDEX_WIDTH-1:0]] <= allocate_expects_data_i;
        next_lower_address_r[allocate_tag_o[TAG_INDEX_WIDTH-1:0]] <=
            allocate_address_i[6:0];
      end

      if (completion_fire) begin
        // The timer restarts on ANY matched completion for the tag, including
        // one the malformed guard below rejects (that CPL leaves the tag in
        // flight) and including a late CPL for a zombie. See header.
        if (completion_match)
          alloc_time_r[completion_index] <= cycle_counter_r;

        if (!completion_match) begin
          unexpected_r <= 1'b1;
          completion_error_code_o <= TLP_ERR_UNEXPECTED_COMPLETION;
        end else if (zombie_r[completion_index]) begin
          // Late completion for a quarantined tag: drained silently. No result,
          // no unexpected report, and no byte-count checking -- the request it
          // belonged to has already been failed.
          late_cpl_valid_o <= 1'b1;
          late_cpl_tag_o   <= completion_header_i.tag;
          if (completion_last) begin
            zombie_r[completion_index]            <= 1'b0;
            remaining_r[completion_index]         <= '0;
            expects_data_r[completion_index]      <= 1'b0;
            next_lower_address_r[completion_index] <= '0;
          end else begin
            remaining_r[completion_index] <=
                remaining_r[completion_index] - completion_payload_bytes_i;
            next_lower_address_r[completion_index] <=
                next_lower_address_r[completion_index] + completion_payload_bytes_i[6:0];
          end
        end else if ((expects_data_r[completion_index] &&
                      completion_header_i.completion_status == TLP_CPL_SC &&
                      (completion_payload_bytes_i == 0 ||
                       completion_payload_bytes_i > remaining_r[completion_index] ||
                       completion_header_i.byte_count != remaining_r[completion_index] ||
                       completion_header_i.lower_address != next_lower_address_r[completion_index])) ||
                     (!expects_data_r[completion_index] && completion_payload_bytes_i != 0)) begin
          unexpected_r <= 1'b1;
          completion_error_code_o <= TLP_ERR_COMPLETION_OVERFLOW;
        end else begin
          result_valid_r   <= 1'b1;
          result_context_r <= context_r[completion_index];
          result_status_r  <= completion_header_i.completion_status;
          result_last_r    <= completion_last;
          if (completion_last) begin
            active_r[completion_index] <= 1'b0;
            remaining_r[completion_index] <= '0;
            expects_data_r[completion_index] <= 1'b0;
          end else begin
            remaining_r[completion_index] <=
                remaining_r[completion_index] - completion_payload_bytes_i;
            next_lower_address_r[completion_index] <=
                next_lower_address_r[completion_index] + completion_payload_bytes_i[6:0];
          end
        end
      end
    end
  end

endmodule
