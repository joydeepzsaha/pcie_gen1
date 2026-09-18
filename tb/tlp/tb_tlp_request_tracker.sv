`timescale 1ns/1ps
module tb_tlp_request_tracker #(
    // Overridden per target (FuseSoC vlogparam applies to the toplevel).
    // 4096 is the tracker's own default; the timeout targets override it.
    // ⚠️⚠️ §63 #7e: THIS IS A DUPLICATE OF THE RTL DEFAULT AND MUST TRACK IT.
    //
    // verilate_tlp_cpl_timeout_default sets no `parameters:` entry, so it
    // elaborates with THIS default -- not tlp_request_tracker.sv's. The row it
    // runs, t1b, described itself as exercising "the value the RTL ships with".
    // THAT WAS FALSE for the whole life of the row: it pinned this bench-local
    // copy. When §63 #7e moved the RTL default 4096 -> 6250 the row failed,
    // firing at k=4127, which is how the duplication was found at all.
    //
    // The siblings need the parameter to exist (verilate_tlp_cpl_timeout passes
    // 64, _off passes 0, both via fusesoc `parameters:`), so it cannot simply
    // be deleted, and SystemVerilog gives no way to read another module's
    // parameter default. The duplication is therefore STRUCTURAL, and the only
    // defence is that it is now loud instead of silent.
    //
    // !! IF YOU CHANGE tlp_request_tracker.sv's DEFAULT, CHANGE THIS TOO.
    // Registered to #7f: a default-witness instance that takes no parameter
    // override would let t1b test the real thing instead of a copy.
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
endmodule
