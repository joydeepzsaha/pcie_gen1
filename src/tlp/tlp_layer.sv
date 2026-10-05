// ---------------------------------------------------------------------------
// tlp_layer -- PCIe Transaction Layer for VC0
//
// Original author: Joydeep Saha
// Modified by: Kourosh Ghahramani
// Silicon Systems Research Lab, University of Washington
//
// Purpose
//   The Transaction Layer between a client and the Data Link Layer, used by
//   the Endpoint (pcie_endpoint_top) and the Root Complex (pcie_rq_rc_top).
//   On transmit, tlp_requester turns commands into requests, with tags from
//   tlp_request_tracker, and tlp_completion_generator forms Completions;
//   tlp_control merges the two, tlp_generator builds the TLPs, and
//   tlp_vc_buffer holds each TLP until tlp_credit_manager has credit for it.
//   On receive, tlp_parser checks each TLP. A request goes to the target
//   port with its BAR or configuration decode; a Completion goes to the
//   client and to tlp_request_tracker, which matches it to its request.
//
// Interfaces
//   Link         link_up_i: low holds every clocked submodule in reset.
//                transmit_enable_i: low, no TLP starts out of tlp_vc_buffer.
//   Identity     requester_id_i, completer_id_i: the Requester ID of the
//                requests and the Completer ID of the Completions it sends.
//                bus_number_i, device_number_i, function_number_i: matched
//                by tlp_config_decoder.
//   Config       memory_enable_i, extended_tag_enable_i, max_payload_bytes_i,
//                max_read_bytes_i, rcb_128b_i: configuration inputs of
//                tlp_bar_decoder, tlp_requester, tlp_request_tracker and
//                tlp_completion_generator.
//   Credits      fc_initialized_i, fc_update_valid_i, fc_*_i: the peer's
//                credit limits and the FC-initialized flag, from the DLL.
//   DLL          s_dllp_axis_*: received TLPs; tuser is not read.
//                m_dllp_axis_*: TLPs to send; tuser is 0. One DW per beat.
//   Command      command_*: requests to send. allocated_tag_o,
//                allocated_tag_valid_o: the tag of each Non-Posted TLP.
//   Target       target_*: received requests and their decode.
//   Completer    completion_request_*: Completions to send.
//   Completions  received_completion_*: received Completions. result_*: the
//                matched request's context and status, and its end.
//   Status       malformed_o, rx_error_*, rx_ecrc_error_o: receive errors.
//                tx_error_*, credit_error_o: transmit errors.
//                tx_fc_blocked_o: the next TLP waits at the credit gate.
//                vc_overflow_o, unexpected_completion_o,
//                completion_error_code_o, cpl_timeout_*, late_cpl_*,
//                outstanding_o: from tlp_vc_buffer and tlp_request_tracker.
//
// Clock and reset
//   clk_i only. rst_i is synchronous and active high. layer_reset (rst_i, or
//   link_up_i low) resets every clocked submodule: outstanding tags are
//   dropped with no result and no timeout report, and command_ready_o,
//   completion_request_ready_o and s_dllp_axis_tready stay high, so what
//   they take then is lost.
//
// Limitations
//   VC0 only, and DATA_WIDTH must be 32. One transmit FIFO: a TLP waiting for
//   credit holds every TLP behind it (tlp_vc_buffer), so a Posted Request or
//   Completion cannot pass a blocked Non-Posted Request (§2.4.1, Table 2-33).
//   A Non-Posted command lost while link_up_i is low gets no Unsupported
//   Request Completion, which a Downstream Port must return (§2.9.1). The
//   handoff tap decodes DW0 correctly only with PCIE_WIRE_ORDER set.
//
// Structure
//   Handoff tap       the sent Non-Posted Tags, for the Completion Timeout.
//   Internal streams  the wires between the submodules; the allocation tap.
//   Receive routing   requests to the target port, Completions to the client
//                     and tlp_request_tracker; payload steering.
//   Transmit control  transmit gating, the error merge, the class register.
//   Receive path      tlp_parser, tlp_classifier, tlp_bar_decoder,
//                     tlp_config_decoder.
//   Transmit path     tlp_requester, tlp_request_tracker,
//                     tlp_completion_generator, tlp_control, tlp_generator,
//                     tlp_vc_buffer, tlp_credit_manager.
//
// References
//   PCIe Base Spec r2.1, §2.2.9
//   PCIe Base Spec r2.1, §2.4.1
//   PCIe Base Spec r2.1, §2.6.1
//   PCIe Base Spec r2.1, §2.8
//   PCIe Base Spec r2.1, §2.9.1
// ---------------------------------------------------------------------------
`timescale 1ns/1ps
module tlp_layer
  import tlp_pkg::*;
#(
    parameter int DATA_WIDTH = 32,
    parameter int KEEP_WIDTH = DATA_WIDTH / 8,
    parameter int USER_WIDTH = 3,
    parameter int TAG_COUNT = 32,
    parameter int CONTEXT_WIDTH = 16,
    // Completion Timeout in cycles, for tlp_request_tracker; 0 disables it.
    // The default is 10 ms at an 8 ns clock.
    parameter int unsigned CPL_TIMEOUT_CYCLES = tlp_pkg::CPL_TIMEOUT_DEFAULT_CYCLES,
    parameter int VC_PACKET_DEPTH = 4,
    // Byte order of header DW1 to DW3 on the DLL streams; see tlp_generator.
    parameter bit PCIE_WIRE_ORDER = 1'b0,
    // BAR windows for tlp_bar_decoder; the default is one 4 KB window at 0.
    parameter int BAR_COUNT = 2,
    parameter logic [BAR_COUNT*64-1:0] BAR_BASE = '0,
    parameter logic [BAR_COUNT*64-1:0] BAR_MASK = {{(BAR_COUNT-1){64'd0}}, 64'hffff_ffff_ffff_f000},
    parameter logic [BAR_COUNT-1:0] BAR_ENABLE = {{(BAR_COUNT-1){1'b0}}, 1'b1}
) (
    input  logic                     clk_i,
    input  logic                     rst_i,
    // ---- link, identity and configuration ----------------------------------
    input  logic                     link_up_i,
    input  logic                     transmit_enable_i,
    input  logic [15:0]              requester_id_i,
    input  logic [15:0]              completer_id_i,
    input  logic [7:0]               bus_number_i,
    input  logic [4:0]               device_number_i,
    input  logic [2:0]               function_number_i,
    input  logic                     memory_enable_i,
    input  logic                     extended_tag_enable_i,
    input  logic [12:0]              max_payload_bytes_i,
    input  logic [12:0]              max_read_bytes_i,
    input  logic                     rcb_128b_i,
    // ---- credits from the Data Link Layer ----------------------------------
    input  logic                     fc_initialized_i,
    input  logic                     fc_update_valid_i,
    input  logic [7:0]               fc_ph_i,
    input  logic [11:0]              fc_pd_i,
    input  logic [7:0]               fc_nph_i,
    input  logic [11:0]              fc_npd_i,
    input  logic [7:0]               fc_cplh_i,
    input  logic [11:0]              fc_cpld_i,

    // ---- TLPs from the Data Link Layer -------------------------------------
    input  logic [DATA_WIDTH-1:0]    s_dllp_axis_tdata,
    input  logic [KEEP_WIDTH-1:0]    s_dllp_axis_tkeep,
    input  logic                     s_dllp_axis_tvalid,
    input  logic                     s_dllp_axis_tlast,
    input  logic [USER_WIDTH-1:0]    s_dllp_axis_tuser,
    output logic                     s_dllp_axis_tready,

    // ---- TLPs to the Data Link Layer ---------------------------------------
    output logic [DATA_WIDTH-1:0]    m_dllp_axis_tdata,
    output logic [KEEP_WIDTH-1:0]    m_dllp_axis_tkeep,
    output logic                     m_dllp_axis_tvalid,
    output logic                     m_dllp_axis_tlast,
    output logic [USER_WIDTH-1:0]    m_dllp_axis_tuser,
    input  logic                     m_dllp_axis_tready,

    // ---- commands ----------------------------------------------------------
    input  logic                     command_valid_i,
    output logic                     command_ready_o,
    input  tlp_cmd_e                 command_i,
    input  logic [63:0]              command_address_i,
    input  logic [12:0]              command_byte_count_i,
    input  logic [2:0]               command_tc_i,
    input  logic [2:0]               command_attr_i,
    input  logic [CONTEXT_WIDTH-1:0] command_context_i,
    input  logic                     command_prefix_valid_i,
    input  logic [31:0]              command_prefix_i,
    input  logic                     command_ecrc_enable_i,
    input  logic [DATA_WIDTH-1:0]    command_data_i,
    input  logic [KEEP_WIDTH-1:0]    command_keep_i,
    input  logic                     command_data_valid_i,
    input  logic                     command_data_last_i,
    output logic                     command_data_ready_o,
    output logic                     command_error_valid_o,
    output tlp_error_e               command_error_code_o,
    // The tag tlp_request_tracker has just given a Non-Posted TLP of a
    // command, with a one-cycle strobe; the Completion returns the same tag.
    // The tag follows command acceptance by a cycle or more, in REQ_TAG of
    // tlp_requester, so it has its own strobe rather than coming with
    // command_ready_o. A posted write takes no tag, and a command split into
    // several Non-Posted TLPs strobes once per TLP, each with its own tag.
    output logic [7:0]               allocated_tag_o,
    output logic                     allocated_tag_valid_o,

    // ---- received requests -------------------------------------------------
    output logic                     target_request_valid_o,
    input  logic                     target_request_ready_i,
    output tlp_header_t              target_request_header_o,
    output tlp_class_e               target_request_class_o,
    output logic                     target_memory_o,
    output logic                     target_config_o,
    output logic                     target_config_hit_o,
    output logic                     target_config_type_one_o,
    output logic [11:0]              target_config_offset_o,
    output logic                     target_read_o,
    output logic                     target_write_o,
    output logic                     target_unsupported_o,
    output logic                     target_bar_hit_o,
    output logic                     target_bar_overlap_o,
    output logic [((BAR_COUNT <= 1) ? 1 : $clog2(BAR_COUNT))-1:0] target_bar_o,
    output logic [63:0]              target_offset_o,
    output logic [DATA_WIDTH-1:0]    target_data_o,
    output logic [KEEP_WIDTH-1:0]    target_keep_o,
    output logic                     target_data_valid_o,
    output logic                     target_data_last_o,
    input  logic                     target_data_ready_i,

    // ---- Completions to send -----------------------------------------------
    input  logic                     completion_request_valid_i,
    output logic                     completion_request_ready_o,
    input  tlp_header_t              completion_request_header_i,
    input  logic [2:0]               completion_request_status_i,
    input  logic [12:0]              completion_request_byte_count_i,
    input  logic [6:0]               completion_request_lower_address_i,
    input  logic                     completion_request_ecrc_enable_i,
    input  logic [DATA_WIDTH-1:0]    completion_request_data_i,
    input  logic [KEEP_WIDTH-1:0]    completion_request_keep_i,
    input  logic                     completion_request_data_valid_i,
    input  logic                     completion_request_data_last_i,
    output logic                     completion_request_data_ready_o,

    // ---- received Completions and results ----------------------------------
    output logic                     received_completion_valid_o,
    input  logic                     received_completion_ready_i,
    output tlp_header_t              received_completion_header_o,
    output logic [DATA_WIDTH-1:0]    received_completion_data_o,
    output logic [KEEP_WIDTH-1:0]    received_completion_keep_o,
    output logic                     received_completion_data_valid_o,
    output logic                     received_completion_data_last_o,
    input  logic                     received_completion_data_ready_i,

    output logic                     result_valid_o,
    input  logic                     result_ready_i,
    output logic [CONTEXT_WIDTH-1:0] result_context_o,
    output logic [2:0]               result_status_o,
    output logic                     result_last_o,
    // ---- status ------------------------------------------------------------
    output logic                     malformed_o,
    output logic                     rx_error_valid_o,
    output tlp_error_e               rx_error_code_o,
    output logic                     rx_ecrc_error_o,
    output logic                     tx_error_valid_o,
    output tlp_error_e               tx_error_code_o,
    output logic                     tx_fc_blocked_o,
    output logic                     credit_error_o,
    output logic                     vc_overflow_o,
    output logic                     unexpected_completion_o,
    output tlp_error_e               completion_error_code_o,
    // From tlp_request_tracker.
    output logic                     cpl_timeout_valid_o,
    output logic [7:0]               cpl_timeout_tag_o,
    output logic                     late_cpl_valid_o,
    output logic [7:0]               late_cpl_tag_o,
    output logic [$clog2(TAG_COUNT+1)-1:0] outstanding_o
);

  localparam int BAR_INDEX_WIDTH = BAR_COUNT <= 1 ? 1 : $clog2(BAR_COUNT);
  logic layer_reset;
  tlp_header_t parsed_header;
  logic parsed_header_valid;
  logic parsed_header_ready;
  logic [DATA_WIDTH-1:0] parsed_data;
  logic [KEEP_WIDTH-1:0] parsed_keep;
  logic parsed_data_valid;
  logic parsed_data_last;
  logic parsed_data_ready;
  tlp_class_e parsed_class;
  logic parsed_memory, parsed_config, parsed_completion;
  logic parsed_read, parsed_write, parsed_unsupported;
  logic [12:0] parsed_request_span;
  logic [BAR_INDEX_WIDTH-1:0] decoded_bar;
  logic route_completion_r;

  // -------------------------------------------------------------------------
  // Handoff tap
  // -------------------------------------------------------------------------
  // Tells tlp_request_tracker the tag of each request passed to the Data Link
  // Layer, so that it restarts that request's Completion Timeout: the timer
  // starts when a Request is transmitted (PCIe Base Spec r2.1, §2.8), and
  // m_dllp_axis, after the credit gate, is the last point this layer sees.
  // Each beat is one DW. A TLP Prefix (Fmt 100b) is skipped, DW0 gives Fmt
  // and Type, and DW1 carries the Tag in bits [15:8]. Only a request that
  // needs a Completion holds a tag: MRd, MRdLk, I/O, Configuration and
  // AtomicOp Requests. A Completion this layer sends carries the remote
  // Requester's tag, which could equal a tag of this layer, so it is never
  // reported.
  logic [31:0] tx_header_dw;
  logic [1:0]  tx_hdr_dw_r;        // non-prefix DWs seen in this TLP, saturating at 2
  logic        tx_np_request_r;    // DW0 named a request that needs a Completion
  logic        tx_dw0_np_request;
  logic        tx_fire;
  logic        sent_valid;
  logic [7:0]  sent_tag;

  assign tx_fire = m_dllp_axis_tvalid && m_dllp_axis_tready;

  always_comb begin
    // DW0 is sent with byte 0 in bits [7:0], so Fmt and Type are in bits
    // [31:24] only after the reversal that PCIE_WIRE_ORDER selects; with it
    // clear, this decode and the prefix test read Length[7:0] on the DW0
    // beat. DW1 is right in both cases.
    tx_header_dw = m_dllp_axis_tdata[31:0];
    if (PCIE_WIRE_ORDER)
      tx_header_dw = {m_dllp_axis_tdata[7:0], m_dllp_axis_tdata[15:8],
                      m_dllp_axis_tdata[23:16], m_dllp_axis_tdata[31:24]};
    unique case (tx_header_dw[28:24])
      TLP_TYPE_MEM, TLP_TYPE_MEM_LOCK: tx_dw0_np_request = !tlp_has_data(tx_header_dw[31:29]);
      TLP_TYPE_IO, TLP_TYPE_CFG0, TLP_TYPE_CFG1,
      TLP_TYPE_FETCH_ADD, TLP_TYPE_SWAP, TLP_TYPE_CAS: tx_dw0_np_request = 1'b1;
      default: tx_dw0_np_request = 1'b0;
    endcase
  end

  always_ff @(posedge clk_i) begin : tx_handoff_tap
    if (layer_reset) begin
      tx_hdr_dw_r     <= 2'd0;
      tx_np_request_r <= 1'b0;
    end else if (tx_fire) begin
      if (m_dllp_axis_tlast) begin
        tx_hdr_dw_r <= 2'd0;
      end else if (tx_hdr_dw_r == 2'd0) begin
        if (tx_header_dw[31:29] != TLP_FMT_PREFIX) begin
          tx_hdr_dw_r     <= 2'd1;              // this beat is DW0
          tx_np_request_r <= tx_dw0_np_request;
        end
      end else if (tx_hdr_dw_r == 2'd1) begin
        tx_hdr_dw_r <= 2'd2;                    // this beat is DW1
      end
    end
  end

  // Raised on the DW1 beat of a request that needs a Completion: the earliest
  // beat that carries its Tag.
  assign sent_valid = tx_fire && (tx_hdr_dw_r == 2'd1) && tx_np_request_r;
  assign sent_tag   = tx_header_dw[15:8];

  // -------------------------------------------------------------------------
  // Internal streams
  // -------------------------------------------------------------------------
  // The wires between the submodules: tlp_requester and
  // tlp_completion_generator into tlp_control, tlp_control into
  // tlp_generator, tlp_generator into tlp_vc_buffer, and tlp_vc_buffer to
  // the credit gate. The tx_packet_* registers carry each TLP's class, Length
  // and has-data flag from the tlp_generator header handshake to
  // tlp_vc_buffer.
  tlp_header_t requester_header;
  logic requester_header_valid, requester_header_ready;
  logic [DATA_WIDTH-1:0] requester_data;
  logic [KEEP_WIDTH-1:0] requester_keep;
  logic requester_data_valid, requester_data_last, requester_data_ready;
  logic tag_valid, tag_ready;
  logic tracker_completion_ready;
  logic [12:0] completion_payload_bytes;
  logic [7:0] allocated_tag;
  // The allocation tap: the strobe is the tag handshake on which
  // tlp_request_tracker marks the tag in flight, and allocated_tag is its
  // combinational allocate_tag_o, so the pair names the committed tag.
  assign allocated_tag_o       = allocated_tag;
  assign allocated_tag_valid_o = tag_valid && tag_ready;
  logic [15:0] tag_requester_id;
  logic [12:0] tag_byte_count;
  logic [CONTEXT_WIDTH-1:0] tag_context;
  logic tag_expects_data;

  tlp_header_t completion_header;
  logic completion_header_valid, completion_header_ready;
  logic [DATA_WIDTH-1:0] completion_data;
  logic [KEEP_WIDTH-1:0] completion_keep;
  logic completion_data_valid, completion_data_last, completion_data_ready;

  tlp_header_t generator_header;
  logic generator_header_valid, generator_header_ready;
  logic [DATA_WIDTH-1:0] generator_data;
  logic [KEEP_WIDTH-1:0] generator_keep;
  logic generator_data_valid, generator_data_last, generator_data_ready;
  logic [DATA_WIDTH-1:0] generated_axis_data;
  logic [KEEP_WIDTH-1:0] generated_axis_keep;
  logic generated_axis_valid, generated_axis_last;
  logic [USER_WIDTH-1:0] generated_axis_user;
  logic generated_axis_ready;
  logic vc_input_ready;
  logic vc_packet_valid, vc_packet_ready;
  tlp_credit_class_e vc_packet_credit_class;
  logic [11:0] vc_packet_data_credits;
  logic credit_request_ready;
  tlp_class_e tx_packet_class_r;
  logic [10:0] tx_packet_length_r;
  logic tx_packet_has_data_r;
  logic requester_error_valid, completion_error_valid;
  tlp_error_e requester_error_code, completion_error_code;

  // -------------------------------------------------------------------------
  // Receive routing
  // -------------------------------------------------------------------------
  // layer_reset holds every clocked submodule in reset while link_up_i is
  // low. A received header goes to the target port, or, for a Completion,
  // to the client and tlp_request_tracker together: each sees it valid only
  // while the other is ready, so both take it in the same cycle. The payload
  // that tlp_parser replays after the header follows route_completion_r.
  // target_unsupported_o adds the decode results to tlp_classifier's verdict:
  // a Memory Request that matches no enabled BAR window, or more than one
  // (none matches while memory_enable_i is low), or a Configuration Request
  // not addressed to this Function.
  assign layer_reset = rst_i || !link_up_i;
  assign received_completion_header_o = parsed_header;
  assign received_completion_valid_o = parsed_header_valid && parsed_completion &&
                                         tracker_completion_ready;
  assign target_request_header_o = parsed_header;
  assign target_request_class_o = parsed_class;
  assign target_request_valid_o = parsed_header_valid && !parsed_completion;
  assign target_memory_o = parsed_memory;
  assign target_config_o = parsed_config;
  assign target_read_o = parsed_read;
  assign target_write_o = parsed_write;
  assign target_unsupported_o = parsed_unsupported ||
                                (parsed_memory && (!target_bar_hit_o || target_bar_overlap_o)) ||
                                (parsed_config && !target_config_hit_o);

  // The bytes the BAR decode covers. A zero-length read (Length 1, no Byte
  // Enable set) is decoded at its address alone.
  always_comb begin
    parsed_request_span = {parsed_header.length_dw, 2'b00};
    if (!tlp_has_data(parsed_header.fmt) && parsed_header.length_dw == 1 &&
        parsed_header.first_be == 0 && parsed_header.last_be == 0)
      parsed_request_span = 1;
  end

  assign parsed_header_ready = parsed_completion ?
      (received_completion_ready_i && tracker_completion_ready) : target_request_ready_i;

  // The payload bytes a Completion carries: Length in bytes, less the bytes
  // ahead of Lower Address in the first DW, and no more than Byte Count,
  // which counts the bytes still due (PCIe Base Spec r2.1, §2.2.9).
  always_comb begin
    completion_payload_bytes = {parsed_header.length_dw, 2'b00} -
                               {11'd0, parsed_header.lower_address[1:0]};
    if (parsed_header.byte_count < completion_payload_bytes)
      completion_payload_bytes = parsed_header.byte_count;
  end

  assign target_data_o = parsed_data;
  assign target_keep_o = parsed_keep;
  assign target_data_valid_o = parsed_data_valid && !route_completion_r;
  assign target_data_last_o = parsed_data_last;
  assign received_completion_data_o = parsed_data;
  assign received_completion_keep_o = parsed_keep;
  assign received_completion_data_valid_o = parsed_data_valid && route_completion_r;
  assign received_completion_data_last_o = parsed_data_last;
  assign parsed_data_ready = route_completion_r ?
      received_completion_data_ready_i : target_data_ready_i;

  // Set at each header handshake, one cycle before tlp_parser replays the
  // payload; cleared after its last beat.
  always_ff @(posedge clk_i) begin
    if (layer_reset) begin
      route_completion_r <= 1'b0;
    end else begin
      if (parsed_header_valid && parsed_header_ready)
        route_completion_r <= parsed_completion;
      if (parsed_data_valid && parsed_data_ready && parsed_data_last)
        route_completion_r <= 1'b0;
    end
  end

  // -------------------------------------------------------------------------
  // Transmit control
  // -------------------------------------------------------------------------
  // A stored TLP leaves tlp_vc_buffer only with credit, transmit_enable_i and
  // link_up_i; the credit gate is asked only while the other two hold, so
  // tx_fc_blocked_o means a TLP waits for credit or for fc_initialized_i.
  // tx_error_code_o takes one code per cycle, by priority: tlp_requester's,
  // then tlp_completion_generator's, then TLP_ERR_CREDIT_UNDERFLOW for
  // tlp_credit_manager's error_o. The tx_packet_* registers give
  // tlp_vc_buffer each TLP's Flow Control class and Length (PCIe Base Spec
  // r2.1, §2.6.1).
  assign generated_axis_ready = vc_input_ready;
  assign vc_packet_ready = credit_request_ready && transmit_enable_i && link_up_i;
  assign command_error_valid_o = requester_error_valid;
  assign command_error_code_o = requester_error_code;
  assign tx_error_valid_o = requester_error_valid || completion_error_valid || credit_error_o;
  assign tx_error_code_o = tlp_error_e'(requester_error_valid ? requester_error_code :
                           completion_error_valid ? completion_error_code :
                           credit_error_o ? TLP_ERR_CREDIT_UNDERFLOW : TLP_ERR_NONE);

  // Registered at the tlp_generator header handshake; tlp_vc_buffer samples
  // it with the TLP's first beat, which leaves tlp_generator later.
  always_ff @(posedge clk_i) begin
    if (layer_reset) begin
      tx_packet_class_r <= TLP_CLASS_NON_POSTED;
      tx_packet_length_r <= '0;
      tx_packet_has_data_r <= 1'b0;
    end else if (generator_header_valid && generator_header_ready) begin
      tx_packet_length_r <= generator_header.length_dw;
      tx_packet_has_data_r <= tlp_has_data(generator_header.fmt);
      if (generator_header.tlp_type == TLP_TYPE_CPL ||
          generator_header.tlp_type == TLP_TYPE_CPL_LOCK)
        tx_packet_class_r <= TLP_CLASS_COMPLETION;
      else if (generator_header.tlp_type == TLP_TYPE_MEM && tlp_has_data(generator_header.fmt))
        tx_packet_class_r <= TLP_CLASS_POSTED;
      else
        tx_packet_class_r <= TLP_CLASS_NON_POSTED;
    end
  end

  // -------------------------------------------------------------------------
  // Receive path
  // -------------------------------------------------------------------------
  // tlp_parser takes TLPs from the Data Link Layer and offers each header
  // only after its checks; malformed_o, rx_error_* and rx_ecrc_error_o are
  // its reports. tlp_classifier, tlp_bar_decoder and tlp_config_decoder
  // decode that header combinationally for the target port and for the
  // routing above.
  tlp_parser #(
      .DATA_WIDTH(DATA_WIDTH), .KEEP_WIDTH(KEEP_WIDTH), .USER_WIDTH(USER_WIDTH),
      .PCIE_WIRE_ORDER(PCIE_WIRE_ORDER)
  ) parser_inst (
      .clk_i(clk_i), .rst_i(layer_reset),
      .s_axis_tdata(s_dllp_axis_tdata), .s_axis_tkeep(s_dllp_axis_tkeep),
      .s_axis_tvalid(s_dllp_axis_tvalid), .s_axis_tlast(s_dllp_axis_tlast),
      .s_axis_tuser(s_dllp_axis_tuser), .s_axis_tready(s_dllp_axis_tready),
      .header_o(parsed_header), .header_valid_o(parsed_header_valid),
      .header_ready_i(parsed_header_ready),
      .payload_tdata_o(parsed_data), .payload_tkeep_o(parsed_keep),
      .payload_tvalid_o(parsed_data_valid), .payload_tlast_o(parsed_data_last),
      .payload_tready_i(parsed_data_ready), .malformed_o(malformed_o),
      .error_valid_o(rx_error_valid_o), .error_code_o(rx_error_code_o),
      .ecrc_error_o(rx_ecrc_error_o)
  );

  tlp_classifier classifier_inst (
      .header_i(parsed_header), .class_o(parsed_class),
      .memory_request_o(parsed_memory), .config_request_o(parsed_config),
      .completion_o(parsed_completion), .read_request_o(parsed_read),
      .write_request_o(parsed_write), .unsupported_o(parsed_unsupported)
  );

  tlp_bar_decoder #(
      .BAR_COUNT(BAR_COUNT), .BAR_BASE(BAR_BASE), .BAR_MASK(BAR_MASK), .BAR_ENABLE(BAR_ENABLE)
  ) bar_decoder_inst (
      .address_i(parsed_header.address), .length_bytes_i(parsed_request_span),
      .memory_enable_i(memory_enable_i), .hit_o(target_bar_hit_o),
      .overlap_o(target_bar_overlap_o), .bar_o(decoded_bar), .offset_o(target_offset_o)
  );
  assign target_bar_o = decoded_bar;

  tlp_config_decoder config_decoder_inst (
      .header_i(parsed_header), .bus_number_i(bus_number_i),
      .device_number_i(device_number_i), .function_number_i(function_number_i),
      .hit_o(target_config_hit_o), .type_one_o(target_config_type_one_o),
      .register_offset_o(target_config_offset_o)
  );

  // -------------------------------------------------------------------------
  // Transmit path
  // -------------------------------------------------------------------------
  // tlp_requester turns commands into request headers and takes a tag from
  // tlp_request_tracker for each Non-Posted TLP; tlp_completion_generator
  // forms Completions. tlp_control merges the two, tlp_generator sends the
  // TLPs into tlp_vc_buffer, and tlp_credit_manager gates each TLP out of it.
  // tlp_request_tracker also matches received Completions, from
  // parsed_header and completion_payload_bytes.
  tlp_requester #(
      .DATA_WIDTH(DATA_WIDTH), .KEEP_WIDTH(KEEP_WIDTH), .CONTEXT_WIDTH(CONTEXT_WIDTH)
  ) requester_inst (
      .clk_i(clk_i), .rst_i(layer_reset), .requester_id_i(requester_id_i),
      .max_payload_bytes_i(max_payload_bytes_i), .max_read_bytes_i(max_read_bytes_i),
      .command_valid_i(command_valid_i), .command_ready_o(command_ready_o),
      .command_i(command_i), .command_address_i(command_address_i),
      .command_byte_count_i(command_byte_count_i), .command_tc_i(command_tc_i),
      .command_attr_i(command_attr_i), .command_context_i(command_context_i),
      .command_prefix_valid_i(command_prefix_valid_i), .command_prefix_i(command_prefix_i),
      .command_ecrc_enable_i(command_ecrc_enable_i),
      .command_data_i(command_data_i), .command_keep_i(command_keep_i),
      .command_data_valid_i(command_data_valid_i), .command_data_last_i(command_data_last_i),
      .command_data_ready_o(command_data_ready_o),
      .tag_request_valid_o(tag_valid), .tag_request_ready_i(tag_ready), .tag_i(allocated_tag),
      .tag_requester_id_o(tag_requester_id), .tag_byte_count_o(tag_byte_count),
      .tag_context_o(tag_context), .tag_expects_data_o(tag_expects_data),
      .packet_header_o(requester_header),
      .packet_header_valid_o(requester_header_valid), .packet_header_ready_i(requester_header_ready),
      .packet_data_o(requester_data), .packet_keep_o(requester_keep),
      .packet_data_valid_o(requester_data_valid), .packet_data_last_o(requester_data_last),
      .packet_data_ready_i(requester_data_ready),
      .command_error_valid_o(requester_error_valid),
      .command_error_code_o(requester_error_code)
  );

  tlp_request_tracker #(
      .TAG_COUNT(TAG_COUNT), .CONTEXT_WIDTH(CONTEXT_WIDTH),
      .CPL_TIMEOUT_CYCLES(CPL_TIMEOUT_CYCLES)
  ) tracker_inst (
      .clk_i(clk_i), .rst_i(layer_reset), .extended_tag_enable_i(extended_tag_enable_i),
      .allocate_valid_i(tag_valid), .allocate_ready_o(tag_ready),
      .allocate_requester_id_i(tag_requester_id), .allocate_byte_count_i(tag_byte_count),
      // The Lower Address tlp_request_tracker expects first. Only a Memory
      // Read Completion carries one; in I/O and Configuration Completions it
      // is 0 (PCIe Base Spec r2.1, §2.2.9). A Configuration Request's address
      // holds ID and register fields, not a byte address.
      .allocate_address_i(requester_header.tlp_type == TLP_TYPE_MEM ?
                          requester_header.address : 64'd0),
      .allocate_context_i(tag_context), .allocate_expects_data_i(tag_expects_data),
      .allocate_tag_o(allocated_tag),
      .sent_valid_i(sent_valid), .sent_tag_i(sent_tag),
      .completion_valid_i(parsed_header_valid && parsed_completion && received_completion_ready_i),
      .completion_ready_o(tracker_completion_ready), .completion_header_i(parsed_header),
      .completion_payload_bytes_i(completion_payload_bytes),
      .result_valid_o(result_valid_o), .result_ready_i(result_ready_i),
      .result_context_o(result_context_o), .result_status_o(result_status_o),
      .result_last_o(result_last_o), .unexpected_completion_o(unexpected_completion_o),
      .completion_error_code_o(completion_error_code_o),
      .cpl_timeout_valid_o(cpl_timeout_valid_o), .cpl_timeout_tag_o(cpl_timeout_tag_o),
      .late_cpl_valid_o(late_cpl_valid_o), .late_cpl_tag_o(late_cpl_tag_o),
      .outstanding_o(outstanding_o)
  );

  tlp_completion_generator #(
      .DATA_WIDTH(DATA_WIDTH), .KEEP_WIDTH(KEEP_WIDTH)
  ) completion_generator_inst (
      .clk_i(clk_i), .rst_i(layer_reset), .completer_id_i(completer_id_i),
      .max_payload_bytes_i(max_payload_bytes_i), .rcb_128b_i(rcb_128b_i),
      .request_valid_i(completion_request_valid_i), .request_ready_o(completion_request_ready_o),
      .request_header_i(completion_request_header_i),
      .request_status_i(completion_request_status_i),
      .request_byte_count_i(completion_request_byte_count_i),
      .request_lower_address_i(completion_request_lower_address_i),
      .request_ecrc_enable_i(completion_request_ecrc_enable_i),
      .request_data_i(completion_request_data_i), .request_keep_i(completion_request_keep_i),
      .request_data_valid_i(completion_request_data_valid_i),
      .request_data_last_i(completion_request_data_last_i),
      .request_data_ready_o(completion_request_data_ready_o),
      .packet_header_o(completion_header), .packet_header_valid_o(completion_header_valid),
      .packet_header_ready_i(completion_header_ready), .packet_data_o(completion_data),
      .packet_keep_o(completion_keep), .packet_data_valid_o(completion_data_valid),
      .packet_data_last_o(completion_data_last), .packet_data_ready_i(completion_data_ready),
      .error_valid_o(completion_error_valid), .error_code_o(completion_error_code)
  );

  tlp_control #(
      .DATA_WIDTH(DATA_WIDTH), .KEEP_WIDTH(KEEP_WIDTH)
  ) control_inst (
      .clk_i(clk_i), .rst_i(layer_reset),
      .requester_header_i(requester_header), .requester_header_valid_i(requester_header_valid),
      .requester_header_ready_o(requester_header_ready), .requester_data_i(requester_data),
      .requester_keep_i(requester_keep), .requester_data_valid_i(requester_data_valid),
      .requester_data_last_i(requester_data_last), .requester_data_ready_o(requester_data_ready),
      .completion_header_i(completion_header), .completion_header_valid_i(completion_header_valid),
      .completion_header_ready_o(completion_header_ready), .completion_data_i(completion_data),
      .completion_keep_i(completion_keep), .completion_data_valid_i(completion_data_valid),
      .completion_data_last_i(completion_data_last), .completion_data_ready_o(completion_data_ready),
      .generator_header_o(generator_header), .generator_header_valid_o(generator_header_valid),
      .generator_header_ready_i(generator_header_ready), .generator_data_o(generator_data),
      .generator_keep_o(generator_keep), .generator_data_valid_o(generator_data_valid),
      .generator_data_last_o(generator_data_last), .generator_data_ready_i(generator_data_ready)
  );

  tlp_generator #(
      .DATA_WIDTH(DATA_WIDTH), .KEEP_WIDTH(KEEP_WIDTH), .USER_WIDTH(USER_WIDTH),
      .PCIE_WIRE_ORDER(PCIE_WIRE_ORDER)
  ) generator_inst (
      .clk_i(clk_i), .rst_i(layer_reset), .header_i(generator_header),
      .header_valid_i(generator_header_valid), .header_ready_o(generator_header_ready),
      .payload_tdata_i(generator_data), .payload_tkeep_i(generator_keep),
      .payload_tvalid_i(generator_data_valid), .payload_tlast_i(generator_data_last),
      .payload_tready_o(generator_data_ready), .m_axis_tdata(generated_axis_data),
      .m_axis_tkeep(generated_axis_keep), .m_axis_tvalid(generated_axis_valid),
      .m_axis_tlast(generated_axis_last), .m_axis_tuser(generated_axis_user),
      .m_axis_tready(generated_axis_ready)
  );

  tlp_vc_buffer #(
      .DATA_WIDTH(DATA_WIDTH), .KEEP_WIDTH(KEEP_WIDTH), .USER_WIDTH(USER_WIDTH),
      .PACKET_DEPTH(VC_PACKET_DEPTH)
  ) vc_buffer_inst (
      .clk_i(clk_i), .rst_i(layer_reset),
      .s_axis_tdata(generated_axis_data), .s_axis_tkeep(generated_axis_keep),
      .s_axis_tvalid(generated_axis_valid), .s_axis_tlast(generated_axis_last),
      .s_axis_tuser(generated_axis_user), .s_axis_tready(vc_input_ready),
      .s_packet_class_i(tx_packet_class_r), .s_packet_length_dw_i(tx_packet_length_r),
      .s_packet_has_data_i(tx_packet_has_data_r),
      .packet_valid_o(vc_packet_valid), .packet_ready_i(vc_packet_ready),
      .packet_credit_class_o(vc_packet_credit_class),
      .packet_data_credits_o(vc_packet_data_credits),
      .m_axis_tdata(m_dllp_axis_tdata), .m_axis_tkeep(m_dllp_axis_tkeep),
      .m_axis_tvalid(m_dllp_axis_tvalid), .m_axis_tlast(m_dllp_axis_tlast),
      .m_axis_tuser(m_dllp_axis_tuser), .m_axis_tready(m_dllp_axis_tready),
      .overflow_o(vc_overflow_o)
  );

  tlp_credit_manager credit_manager_inst (
      .clk_i(clk_i), .rst_i(layer_reset), .fc_initialized_i(fc_initialized_i),
      .fc_update_valid_i(fc_update_valid_i), .fc_ph_i(fc_ph_i), .fc_pd_i(fc_pd_i),
      .fc_nph_i(fc_nph_i), .fc_npd_i(fc_npd_i), .fc_cplh_i(fc_cplh_i),
      .fc_cpld_i(fc_cpld_i),
      .request_valid_i(vc_packet_valid && transmit_enable_i && link_up_i),
      .request_ready_o(credit_request_ready), .request_class_i(vc_packet_credit_class),
      .request_data_credits_i(vc_packet_data_credits), .blocked_o(tx_fc_blocked_o),
      .error_o(credit_error_o), .posted_header_available_o(),
      .posted_data_available_o(), .nonposted_header_available_o(),
      .nonposted_data_available_o(), .completion_header_available_o(),
      .completion_data_available_o()
  );

endmodule
