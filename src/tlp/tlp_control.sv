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

  // ---- Table 2-33 Row D x Col 2: a Completion must not pass a Posted Request
  //
  // Base 2.1 §2.4.1 p.122-123 and D2a p.124: "A Completion must not pass a
  // Posted Request unless D2b applies.  If the Relaxed Ordering attribute bit
  // is not set, then a Read Completion cannot pass a previously enqueued
  // Memory Write or Message Request."
  //
  // Before this term the arbiter alternated unconditionally -- posted-ness was
  // simply not an input to the grant, so a Completion won any contended cycle
  // in which prefer_completion_r happened to be set.  The defect was an
  // ABSENCE, not a wrong comparison, which is why the fix is one added
  // conjunct rather than a restructure.
  //
  // D2b is the exception and is honoured: "A Completion with RO Set is
  // permitted to pass a Posted Request."  RO is Attr[1].  ⚠️ Attr is SPLIT
  // across two header bytes and the halves are not adjacent -- attributes[2]
  // is IDO (dw0[10]) and attributes[1:0] are RO and No Snoop (dw0[21:20]); see
  // tlp_generator.sv:66-78 and tlp_parser.sv:125.  Reading attributes[0] here
  // would gate on No Snoop and look entirely plausible while being wrong, and
  // a round-trip test cannot see that class -- M-2 caught one such misplacement
  // in this tree already.
  //
  // ⚠️ D2b's OTHER exception is deliberately NOT implemented.  It also permits
  // an I/O or Configuration Write Completion to pass a Posted Request
  // regardless of RO, but Row E x Col 2 is "Y/N", so blocking those is equally
  // conformant -- and footnote 28 p.124 warns a component "must not apply this
  // rule ... unless it is certain of the associated Request type", which an
  // arbiter looking only at a Completion header is not.  Blocking is the safe
  // half of a permission.
  //
  // Posted here is Memory Write ONLY.  tlp_classifier.sv:28-36 classifies
  // TLP_TYPE_MEM with data as POSTED; Messages fall to its unsupported arm and
  // never reach this arbiter as posted traffic.
  //
  // No deadlock: when a Completion is blocked, selected_completion is 0, so
  // the posted request is granted and drains, and prefer_completion_r is then
  // loaded with 1 so the Completion wins the next contended cycle.  A5a p.124
  // separately permits a Posted Request to pass a Completion, so ordering the
  // two this way is conformant in both directions.
  always_comb begin
    requester_posted_pending = requester_header_valid_i &&
        (requester_header_i.tlp_type == TLP_TYPE_MEM) &&
        tlp_has_data(requester_header_i.fmt);
    completion_may_pass_posted = completion_header_i.attributes[1];

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
