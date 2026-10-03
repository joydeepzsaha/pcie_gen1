// ---------------------------------------------------------------------------
// tlp_request_tracker -- tags, Completion matching and Completion Timeout
//
// Original author: Joydeep Saha
// Modified by: Kourosh Ghahramani
// Silicon Systems Research Lab, University of Washington
//
// Purpose
//   Hands out a tag for each Non-Posted request, matches every received
//   Completion to its request by Tag and Requester ID, checks its byte count
//   and Lower Address, and reports one result per Completion that passes. A
//   request's timer starts at allocation and restarts at its handoff to the
//   Data Link Layer and at each matched Completion; at CPL_TIMEOUT_CYCLES the
//   request times out (PCIe Base Spec r2.1, §2.8). Its tag stays quarantined
//   until a late Completion ends the request or one more interval passes.
//
// Interfaces
//   Allocate    allocate_*, extended_tag_enable_i: one tag per handshake, the
//               lowest free one; tags 32 and up only while the enable is set.
//   Handoff     sent_valid_i, sent_tag_i: the request with this tag has gone
//               to the Data Link Layer.
//   Completion  completion_valid_i, completion_ready_o, completion_header_i,
//               completion_payload_bytes_i: a Completion header and the
//               payload bytes it carries; the payload bypasses this module.
//   Result      result_*: the request's context, the Completion Status and
//               whether the request is finished; held until result_ready_i.
//   Errors      unexpected_completion_o, completion_error_code_o: one cycle.
//   Timeout     cpl_timeout_*: a request timed out. late_cpl_*: a quarantined
//               tag drained a late Completion. One-cycle strobes with the tag.
//   Count       outstanding_o: tags in flight or quarantined.
//
// Clock and reset
//   clk_i only. rst_i is synchronous and active high; it frees every tag with
//   no result and no timeout report.
//
// Limitations
//   A zero-length read never completes successfully: its Successful
//   Completion is rejected. A request held at the credit gate times out
//   unsent, and one interval later its tag can be reallocated while the
//   request is still queued.
//
// References
//   PCIe Base Spec r2.1, §2.2.6.2
//   PCIe Base Spec r2.1, §2.3.1.1
//   PCIe Base Spec r2.1, §2.3.2
//   PCIe Base Spec r2.1, §2.8
//   PCIe Base Spec r2.1, §7.8.16
// ---------------------------------------------------------------------------
`timescale 1ns/1ps
module tlp_request_tracker
  import tlp_pkg::*;
#(
    parameter int TAG_COUNT = 32,
    parameter int CONTEXT_WIDTH = 16,
    // Completion Timeout in clock cycles; tlp_pkg's default is 10 ms at 8 ns.
    // 0 disables the mechanism, as Completion Timeout Disable does (PCIe Base
    // Spec r2.1, §7.8.16). There is no Device Control 2 register to set it.
    parameter int unsigned CPL_TIMEOUT_CYCLES = tlp_pkg::CPL_TIMEOUT_DEFAULT_CYCLES
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

    // The request with this tag left tlp_layer for the Data Link Layer
    // (tlp_layer's handoff tap); see sent_restart.
    input  logic                     sent_valid_i,
    input  logic [7:0]               sent_tag_i,

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

    // One-cycle strobes with the tag in the same cycle. cpl_timeout_valid_o
    // fires once as a tag goes from in flight to zombie; late_cpl_valid_o
    // fires for each Completion a zombie tag drains.
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

  // Completion Timeout. A tag is free, in flight (active_r) or a zombie
  // (zombie_r): timed out and quarantined. A zombie cannot be allocated and
  // still matches a late Completion, which it drains; it becomes free on the
  // Completion that would have finished its request, or after one more
  // CPL_TIMEOUT_CYCLES with no matched Completion. A tag freed at once could
  // be reallocated, and a late Completion would then match the new request.
  //
  // One free-running counter and a timestamp per tag; the expiry check walks
  // the tags round-robin, one per cycle, so one subtractor and comparator
  // serve them all and an expiry is seen at most TAG_COUNT cycles late. The
  // modular age is correct while CPL_TIMEOUT_CYCLES is below 2^31.
  localparam logic [31:0] TIMEOUT_LIMIT = CPL_TIMEOUT_CYCLES;
  logic [TAG_COUNT-1:0] zombie_r;              // timed out, quarantined
  logic [31:0] cycle_counter_r;                // free-running, wraps mod 2^32
  logic [31:0] alloc_time_r [0:TAG_COUNT-1];   // per-tag timer origin
  logic [TAG_INDEX_WIDTH-1:0] scan_index_r;    // round-robin expiry walk
  logic [31:0] scan_age;
  logic scan_expired;
  logic completion_fire;
  logic completion_last;
  logic sent_restart;
  logic [TAG_INDEX_WIDTH-1:0] sent_index;

  always_comb begin
    tag_found = 1'b0;
    allocate_tag_o = '0;
    // The lowest tag that is neither in flight nor a zombie. Tags 32 and up
    // only while extended_tag_enable_i is set, as the Extended Tag Field
    // Enable bit requires (PCIe Base Spec r2.1, §2.2.6.2).
    for (search_index = 0; search_index < TAG_COUNT; search_index = search_index + 1) begin
      if (!tag_found && !active_r[search_index] && !zombie_r[search_index] &&
          (extended_tag_enable_i || search_index < 32)) begin
        tag_found = 1'b1;
        allocate_tag_o = search_index[7:0];
      end
    end
    allocate_ready_o = tag_found;

    // A zombie tag still matches, so its late Completion is drained rather
    // than reported as unexpected.
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
    // One result register: a Completion is taken when it is empty or being
    // read in this cycle.
    completion_ready_o = !result_valid_r || result_ready_i;
    completion_fire = completion_valid_i && completion_ready_o;

    // This Completion finishes its request: a request that expects no data, a
    // status other than SC, or the remaining bytes all delivered. It drives
    // result_last_o and also frees a zombie, so both use one test.
    completion_last = !expects_data_r[completion_index] ||
                      completion_header_i.completion_status != TLP_CPL_SC ||
                      completion_payload_bytes_i >= remaining_r[completion_index];

    // A matched Completion for the scanned tag in the same cycle wins: the
    // tag is not timed out in that cycle, and the scan and the Completion
    // never write one tag together. Allocation cannot collide: it takes only
    // free tags, and the scan only in-flight and zombie ones.
    scan_age = cycle_counter_r - alloc_time_r[scan_index_r];
    scan_expired = (TIMEOUT_LIMIT != 32'd0) &&
                   (active_r[scan_index_r] || zombie_r[scan_index_r]) &&
                   (scan_age >= TIMEOUT_LIMIT) &&
                   !(completion_fire && completion_match &&
                     completion_index == scan_index_r);

    // A zombie still holds its tag, so it counts.
    active_count = 0;
    for (search_index = 0; search_index < TAG_COUNT; search_index = search_index + 1)
      active_count = active_count + (active_r[search_index] | zombie_r[search_index]);
    outstanding_o = active_count[$clog2(TAG_COUNT+1)-1:0];
  end

  // The timer starts when a Request is transmitted (PCIe Base Spec r2.1,
  // §2.8). A tag is allocated before tlp_vc_buffer and the credit gate, so
  // the handoff restarts the timer and a request sent before it times out
  // gets the whole interval. A request never handed off still times out
  // CPL_TIMEOUT_CYCLES after allocation, outside §2.8: pcie_cfg_txn has no
  // timeout of its own, so this ends a request held at the credit gate, and
  // pcie_enum_scan reports a timeout seen with tlp_layer's tx_fc_blocked_o
  // high as ENUM_ERR_CREDIT_STARVED. Only an in-flight tag restarts, and not
  // in the cycle the scan times it out.
  //
  // Kept out of the always_comb above: tb_tlp_request_tracker by default
  // wires sent_tag_i to allocate_tag_o, and decoding it in the block that
  // drives allocate_tag_o makes Verilator 5.050 report a block-level loop
  // (UNOPTFLAT, fatal: tb_tlp.core does not pass -Wno-fatal), though none
  // exists: only the always_ff reads sent_restart.
  assign sent_index   = sent_tag_i[TAG_INDEX_WIDTH-1:0];
  assign sent_restart = sent_valid_i && (32'(sent_tag_i) < TAG_COUNT) &&
                        active_r[sent_index] &&
                        !(scan_expired && scan_index_r == sent_index);

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

      // In flight to zombie is reported; zombie to free is silent. Either way
      // the timestamp is rewritten, so the zombie interval is one more
      // CPL_TIMEOUT_CYCLES without a second timer.
      if (scan_expired) begin
        alloc_time_r[scan_index_r] <= cycle_counter_r;
        if (active_r[scan_index_r]) begin
          active_r[scan_index_r]  <= 1'b0;
          zombie_r[scan_index_r]  <= 1'b1;
          cpl_timeout_valid_o     <= 1'b1;
          cpl_timeout_tag_o       <= 8'(scan_index_r);
        end else begin
          // A zombie whose request was never handed off is freed here too;
          // its tag can then be reallocated while that request is still
          // queued, so two requests can go out with one Tag, which §2.2.6.2
          // forbids among outstanding requests.
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

      if (sent_restart)
        alloc_time_r[sent_index] <= cycle_counter_r;

      if (completion_fire) begin
        // Every matched Completion restarts the timer, including one the
        // check below rejects (the tag stays in flight) and a late one for a
        // zombie. §2.8 does not say whether a Completion restarts it.
        if (completion_match)
          alloc_time_r[completion_index] <= cycle_counter_r;

        // No outstanding request has this Transaction ID: an Unexpected
        // Completion (PCIe Base Spec r2.1, §2.3.2).
        if (!completion_match) begin
          unexpected_r <= 1'b1;
          completion_error_code_o <= TLP_ERR_UNEXPECTED_COMPLETION;
        end else if (zombie_r[completion_index]) begin
          // A late Completion for a zombie: no result, no byte-count check,
          // and late_cpl_valid_o instead of unexpected_completion_o, although
          // §2.3.2 makes a Completion with no outstanding request an
          // Unexpected Completion. Data delivered before the timeout stays
          // delivered; §2.8 lets the Requester keep or discard it. On the Root
          // Complex, pcie_rc_if drops the payload in S_IDLE.
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
        // A Successful Completion whose payload, Byte Count or Lower Address
        // does not continue the request, or data for a request that expects
        // none: handled as an Unexpected Completion, which §2.3.2 permits. The
        // tag stays in flight. A zero-length read's Successful Completion lands
        // here: its Byte Count is 1 (§2.3.1.1, Table 2-31), and tlp_requester
        // registers 4 bytes for it, so the read never completes successfully.
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
