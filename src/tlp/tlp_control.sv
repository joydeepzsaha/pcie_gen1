// ---------------------------------------------------------------------------
// tlp_control -- transmit arbiter between requests and completions
//
// Original author: Joydeep Saha
// Modified by: Kourosh Ghahramani
// Silicon Systems Research Lab, University of Washington
//
// Purpose
//   Merges the request stream from tlp_requester and the completion stream
//   from tlp_completion_generator into the single header and payload stream
//   that tlp_generator turns into TLPs. A header handshake on a TLP with
//   data locks the grant until that TLP's last payload beat. Between TLPs
//   the two sources alternate when both are waiting, except that a
//   Completion without Relaxed Ordering is held while a Memory Write
//   Request is waiting (PCIe Base Spec r2.1, §2.4.1).
//
// Interfaces
//   Requests     requester_header_*, requester_data_*, requester_keep_i:
//                headers and payload from tlp_requester.
//   Completions  completion_header_*, completion_data_*, completion_keep_i:
//                headers and payload from tlp_completion_generator.
//   Generator    generator_header_*, generator_data_*, generator_keep_o: the
//                selected stream, to tlp_generator.
//
// Clock and reset
//   clk_i only. rst_i is synchronous and active high. After reset
//   prefer_completion_r favours the Completion side.
//
// References
//   PCIe Base Spec r2.1, §2.2.6.3
//   PCIe Base Spec r2.1, §2.4.1
// ---------------------------------------------------------------------------
`timescale 1ns/1ps
module tlp_control
  import tlp_pkg::*;
#(
    parameter int DATA_WIDTH = 32,
    parameter int KEEP_WIDTH = DATA_WIDTH / 8
) (
    input  logic                  clk_i,
    input  logic                  rst_i,

    input  tlp_header_t           requester_header_i,
    input  logic                  requester_header_valid_i,
    output logic                  requester_header_ready_o,
    input  logic [DATA_WIDTH-1:0] requester_data_i,
    input  logic [KEEP_WIDTH-1:0] requester_keep_i,
    input  logic                  requester_data_valid_i,
    input  logic                  requester_data_last_i,
    output logic                  requester_data_ready_o,

    input  tlp_header_t           completion_header_i,
    input  logic                  completion_header_valid_i,
    output logic                  completion_header_ready_o,
    input  logic [DATA_WIDTH-1:0] completion_data_i,
    input  logic [KEEP_WIDTH-1:0] completion_keep_i,
    input  logic                  completion_data_valid_i,
    input  logic                  completion_data_last_i,
    output logic                  completion_data_ready_o,

    output tlp_header_t           generator_header_o,
    output logic                  generator_header_valid_o,
    input  logic                  generator_header_ready_i,
    output logic [DATA_WIDTH-1:0] generator_data_o,
    output logic [KEEP_WIDTH-1:0] generator_keep_o,
    output logic                  generator_data_valid_o,
    output logic                  generator_data_last_o,
    input  logic                  generator_data_ready_i
);

  logic locked_r;
  logic select_completion_r;
  logic prefer_completion_r;
  logic selected_completion;
  logic requester_posted_pending;
  logic completion_may_pass_posted;

  // A Completion may pass a Posted Request only under an exception of PCIe
  // Base Spec r2.1, §2.4.1 (Table 2-33, row D, column 2); the one used here
  // is Relaxed Ordering set in the Completion. tlp_requester sends no
  // Messages, so its only Posted Request is an MWr. While one is waiting, a
  // Completion without Relaxed Ordering is not selected and the MWr goes
  // first; prefer_completion_r is then set, so the Completion wins the next
  // contended header unless another MWr is waiting. A Posted Request may pass
  // a Completion (row A, column 5), so sending the MWr first is allowed
  // whichever of the two arrived first.
  //
  // The exception for I/O and Configuration Write Completions is not used:
  // the spec grants it only to a component certain of the Request type, and
  // a Completion header does not carry it. The IDO exception is not used.
  // Each exception only permits passing, so holding the Completion conforms.
  always_comb begin
    requester_posted_pending = requester_header_valid_i &&
        (requester_header_i.tlp_type == TLP_TYPE_MEM) &&
        tlp_has_data(requester_header_i.fmt);
    // Relaxed Ordering is Attr[1], which tlp_generator sends at dw0[21];
    // attributes[0] is No Snoop (PCIe Base Spec r2.1, §2.2.6.3).
    completion_may_pass_posted = completion_header_i.attributes[1];

    // While locked_r is set the grant is frozen and only payload moves;
    // otherwise only headers move.
    selected_completion = locked_r ? select_completion_r :
        (completion_header_valid_i &&
         (!requester_header_valid_i || prefer_completion_r) &&
         (!requester_posted_pending || completion_may_pass_posted));
    generator_header_o = selected_completion ? completion_header_i : requester_header_i;
    generator_header_valid_o = !locked_r &&
        (selected_completion ? completion_header_valid_i : requester_header_valid_i);
    completion_header_ready_o = !locked_r && selected_completion && generator_header_ready_i;
    requester_header_ready_o = !locked_r && !selected_completion && generator_header_ready_i;

    generator_data_o = selected_completion ? completion_data_i : requester_data_i;
    generator_keep_o = selected_completion ? completion_keep_i : requester_keep_i;
    generator_data_valid_o = locked_r &&
        (selected_completion ? completion_data_valid_i : requester_data_valid_i);
    generator_data_last_o = selected_completion ? completion_data_last_i : requester_data_last_i;
    completion_data_ready_o = locked_r && selected_completion && generator_data_ready_i;
    requester_data_ready_o = locked_r && !selected_completion && generator_data_ready_i;
  end

  always_ff @(posedge clk_i) begin
    if (rst_i) begin
      locked_r <= 1'b0;
      select_completion_r <= 1'b0;
      prefer_completion_r <= 1'b1;
    end else begin
      if (!locked_r && generator_header_valid_o && generator_header_ready_i) begin
        // The other source is preferred at the next contended header.
        prefer_completion_r <= !selected_completion;
        if (tlp_has_data(generator_header_o.fmt)) begin
          locked_r <= 1'b1;
          select_completion_r <= selected_completion;
        end
      end
      if (locked_r && generator_data_valid_o && generator_data_ready_i && generator_data_last_o)
        locked_r <= 1'b0;
    end
  end

endmodule
