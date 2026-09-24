`timescale 1ns/1ps
module tb_tlp_request_tracker #(
    // Overridden per target (FuseSoC vlogparam applies to the toplevel).
    //
    // ⭐ §63 #7g-2 step 3 (Kourosh Q1; D-7G.2): THIS IS A VISIBLE SIMULATION
    // OVERRIDE, NOT A COPY OF THE RTL DEFAULT.  The shipped default is 10 ms =
    // 1,250,000 cycles (tlp_pkg::CPL_TIMEOUT_DEFAULT_CYCLES); `dut` runs at this
    // bench value so t1b and the 64-/0-cycle siblings stay short, and
    // `dut_default_witness` below -- which omits the parameter -- carries the
    // shipped value into t1c, which pins it.
    //
    // History: from §63 #7e to 7g-2 this line WAS a hand copy of the RTL
    // default and had to track it ("IF YOU CHANGE tlp_request_tracker.sv's
    // DEFAULT, CHANGE THIS TOO"): verilate_tlp_cpl_timeout_default sets no
    // `parameters:` entry, so t1b pinned this copy, never the RTL -- found only
    // when §63 #7e moved the default 4096 -> 6250 and t1b fired at k=4127.
    // 7g-1 added the witness (t1c: copy == RTL, both fired at 6271).  7g-2 ends
    // the copying: the two values now DIFFER BY DESIGN, and t1c pins the RTL's
    // directly against the spec's 10 ms.
    parameter int unsigned CPL_TIMEOUT_CYCLES = 32'd6250
);
  import tlp_pkg::*;
  logic clk_i = 0;
  logic rst_i;
  logic extended_tag_enable;
  logic allocate_valid;
  logic allocate_ready;
  logic [15:0] allocate_requester_id;
  logic [12:0] allocate_byte_count;
  logic [63:0] allocate_address;
  logic [15:0] allocate_context;
  logic allocate_expects_data;
  logic [7:0] allocate_tag;
  logic completion_valid;
  logic completion_ready;
  logic [15:0] completion_requester_id;
  logic [7:0] completion_tag;
  logic [2:0] completion_status;
  logic [12:0] completion_payload_bytes;
  logic [12:0] completion_byte_count;
  logic [6:0] completion_lower_address;
  logic result_valid;
  logic result_ready;
  logic [15:0] result_context;
  logic [2:0] result_status;
  logic result_last;
  logic unexpected_completion;
  logic [4:0] completion_error_code;
  logic       cpl_timeout_valid;
  logic [7:0] cpl_timeout_tag;
  logic       late_cpl_valid;
  logic [7:0] late_cpl_tag;
  logic [5:0] outstanding;
  tlp_header_t completion_header;

  always_comb begin
    completion_header = '0;
    completion_header.requester_id = completion_requester_id;
    completion_header.tag = completion_tag;
    completion_header.completion_status = completion_status;
    completion_header.byte_count = completion_byte_count;
    completion_header.lower_address = completion_lower_address;
  end

  tlp_request_tracker #(.TAG_COUNT(32), .CONTEXT_WIDTH(16),
                       .CPL_TIMEOUT_CYCLES(CPL_TIMEOUT_CYCLES)) dut (
      .clk_i(clk_i), .rst_i(rst_i), .extended_tag_enable_i(extended_tag_enable),
      .allocate_valid_i(allocate_valid), .allocate_ready_o(allocate_ready),
      .allocate_requester_id_i(allocate_requester_id),
      .allocate_byte_count_i(allocate_byte_count), .allocate_address_i(allocate_address),
      .allocate_context_i(allocate_context),
      .allocate_expects_data_i(allocate_expects_data), .allocate_tag_o(allocate_tag),
      .completion_valid_i(completion_valid), .completion_ready_o(completion_ready),
      .completion_header_i(completion_header),
      .completion_payload_bytes_i(completion_payload_bytes),
      .result_valid_o(result_valid), .result_ready_i(result_ready),
      .result_context_o(result_context), .result_status_o(result_status),
      .result_last_o(result_last), .unexpected_completion_o(unexpected_completion),
      .completion_error_code_o(completion_error_code),
      .cpl_timeout_valid_o(cpl_timeout_valid), .cpl_timeout_tag_o(cpl_timeout_tag),
      .late_cpl_valid_o(late_cpl_valid), .late_cpl_tag_o(late_cpl_tag),
      .outstanding_o(outstanding)
  );

  // ===========================================================================
  //  §63 #7g-1 -- THE DEFAULT WITNESS (D-7G.2, registered at #7e and #7f).
  //
  //  The parameter at the top of this file is a HAND COPY of
  //  tlp_request_tracker.sv's default, and SystemVerilog gives no way to read
  //  another module's parameter default -- so the copy cannot be removed, only
  //  WITNESSED.  This instance omits `.CPL_TIMEOUT_CYCLES` entirely, so it
  //  elaborates with the RTL's ACTUAL shipped default whatever that is.
  //
  //  It shares every input net with `dut`, so the two see identical stimulus,
  //  and row `t1c` asserts they raise `cpl_timeout_valid_o` on the SAME cycle.
  //  If someone moves the RTL default and not this file's copy -- exactly what
  //  happened at #7e, where the drift was found only because an unrelated row
  //  failed at k=4127 -- the two instances diverge and t1c goes red, naming the
  //  drift instead of leaving it to be inferred.
  //
  //  ⚠️ The witness is NOT a second DUT: nothing else may assert about it.  Its
  //  only job is to carry the shipped default into a comparison.  Note it is
  //  only meaningful on the target that leaves the bench parameter alone
  //  (verilate_tlp_cpl_timeout_default); the siblings override the parameter on
  //  purpose, so t1c skips itself there rather than asserting a false equality.
  // ===========================================================================
  logic        w_allocate_ready, w_completion_ready, w_result_valid, w_result_last;
  logic        w_unexpected_completion, w_cpl_timeout_valid, w_late_cpl_valid;
  logic [ 7:0] w_allocate_tag, w_cpl_timeout_tag, w_late_cpl_tag;
  logic [15:0] w_result_context;
  logic [ 2:0] w_result_status;
  // §63 #7g-1: FIVE bits, matching tlp_request_tracker's `tlp_error_e` port and
  // the existing `completion_error_code` net above -- caught by the rewritten
  // waiver on its first run, which is the fence working on the day it landed.
  logic [ 4:0] w_completion_error_code;
  logic [ 5:0] w_outstanding;

  tlp_request_tracker #(.TAG_COUNT(32), .CONTEXT_WIDTH(16)) dut_default_witness (
      .clk_i(clk_i), .rst_i(rst_i), .extended_tag_enable_i(extended_tag_enable),
      .allocate_valid_i(allocate_valid), .allocate_ready_o(w_allocate_ready),
      .allocate_requester_id_i(allocate_requester_id),
      .allocate_byte_count_i(allocate_byte_count), .allocate_address_i(allocate_address),
      .allocate_context_i(allocate_context),
      .allocate_expects_data_i(allocate_expects_data), .allocate_tag_o(w_allocate_tag),
      .completion_valid_i(completion_valid), .completion_ready_o(w_completion_ready),
      .completion_header_i(completion_header),
      .completion_payload_bytes_i(completion_payload_bytes),
      .result_valid_o(w_result_valid), .result_ready_i(result_ready),
      .result_context_o(w_result_context), .result_status_o(w_result_status),
      .result_last_o(w_result_last), .unexpected_completion_o(w_unexpected_completion),
      .completion_error_code_o(w_completion_error_code),
      .cpl_timeout_valid_o(w_cpl_timeout_valid), .cpl_timeout_tag_o(w_cpl_timeout_tag),
      .late_cpl_valid_o(w_late_cpl_valid), .late_cpl_tag_o(w_late_cpl_tag),
      .outstanding_o(w_outstanding)
  );
endmodule
