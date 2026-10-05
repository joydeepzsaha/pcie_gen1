// ---------------------------------------------------------------------------
// tlp_requester -- turns a command into one or more request TLP headers
//
// Original author: Joydeep Saha
// Modified by: Kourosh Ghahramani
// Silicon Systems Research Lab, University of Washington
//
// Purpose
//   Accepts one memory, I/O or Configuration read or write command and emits
//   its request headers, passing write data through beside them. A memory
//   command is split so that no TLP crosses a 4 KB boundary or spans more
//   than max_payload_bytes_i (writes) or max_read_bytes_i (reads). Each
//   Non-Posted TLP takes a tag from tlp_request_tracker first.
//
// Interfaces
//   Config   requester_id_i: the Requester ID of every request.
//            max_payload_bytes_i, max_read_bytes_i: the split limits; 0 is 128.
//   Command  command_valid_i, command_ready_o, command_*: taken in REQ_IDLE.
//   Data     command_data_*, command_keep_i: write data, tlast at command end.
//   Tag      tag_*: the allocation handshake with tlp_request_tracker.
//   Packet   packet_*: headers and write data, to tlp_control.
//   Error    command_error_*: one cycle of TLP_ERR_BAD_LENGTH for a rejected
//            command, TLP_ERR_LOCAL_PAYLOAD for a misplaced data tlast.
//
// Clock and reset
//   clk_i only. rst_i is synchronous and active high.
//
// Limitations
//   TC and Attr come from the command, though PCIe Base Spec r2.1, §2.2.7
//   requires TC 000b and Attr[1:0] 00b in I/O and Configuration Requests.
//   Byte Enables are contiguous. A zero-length read registers 4 expected
//   bytes, but its Completion carries Byte Count 1 (§2.3.1.1, Table 2-31);
//   tlp_request_tracker rejects that Completion.
//
// References
//   PCIe Base Spec r2.1, §2.2.2
//   PCIe Base Spec r2.1, §2.2.4.1
//   PCIe Base Spec r2.1, §2.2.5
//   PCIe Base Spec r2.1, §2.2.7
//   PCIe Base Spec r2.1, §2.3.1.1
// ---------------------------------------------------------------------------
`timescale 1ns/1ps
module tlp_requester
  import tlp_pkg::*;
#(
    parameter int DATA_WIDTH = 32,
    parameter int KEEP_WIDTH = DATA_WIDTH / 8,
    parameter int CONTEXT_WIDTH = 16
) (
    input  logic                     clk_i,
    input  logic                     rst_i,
    input  logic [15:0]              requester_id_i,
    input  logic [12:0]              max_payload_bytes_i,
    input  logic [12:0]              max_read_bytes_i,

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

    output logic                     tag_request_valid_o,
    input  logic                     tag_request_ready_i,
    input  logic [7:0]               tag_i,
    output logic [15:0]              tag_requester_id_o,
    output logic [12:0]              tag_byte_count_o,
    output logic [CONTEXT_WIDTH-1:0] tag_context_o,
    output logic                     tag_expects_data_o,

    output tlp_header_t              packet_header_o,
    output logic                     packet_header_valid_o,
    input  logic                     packet_header_ready_i,
    output logic [DATA_WIDTH-1:0]    packet_data_o,
    output logic [KEEP_WIDTH-1:0]    packet_keep_o,
    output logic                     packet_data_valid_o,
    output logic                     packet_data_last_o,
    input  logic                     packet_data_ready_i,
    output logic                     command_error_valid_o,
    output tlp_error_e               command_error_code_o
);

  // REQ_IDLE    takes a command and sizes its first TLP.
  // REQ_TAG     waits for a tag from tlp_request_tracker; Non-Posted only.
  // REQ_HEADER  offers the header; a read moves on to its next TLP or to
  //             REQ_IDLE, a write to REQ_DATA.
  // REQ_DATA    passes one TLP's write data, then moves on to the next TLP
  //             or to REQ_IDLE.
  typedef enum logic [2:0] {REQ_IDLE, REQ_TAG, REQ_HEADER, REQ_DATA} req_state_e;
  req_state_e state_r;
  tlp_cmd_e command_r;
  logic [63:0] address_r;
  logic [12:0] remaining_r;
  logic [12:0] segment_bytes_r;
  logic [12:0] segment_sent_r;
  logic [2:0] tc_r;
  logic [2:0] attr_r;
  logic [CONTEXT_WIDTH-1:0] context_r;
  logic [7:0] tag_r;
  logic prefix_valid_r;
  logic [31:0] prefix_r;
  logic ecrc_enable_r;
  tlp_header_t header_c;
  logic command_has_data;
  logic command_non_posted;
  logic [12:0] accepted_bytes;
  logic expected_data_last;
  logic request_last;
  integer lane;

  // Command-class predicates. The configuration, I/O and read or write tests
  // below go through them, and each lists its members explicitly, so a
  // tlp_cmd_e member that is not listed (TLP_CMD_MSG, TLP_CMD_MSG_DATA)
  // matches none of them. command_non_posted and the REQ_IDLE state select
  // compare with TLP_CMD_MEM_WRITE directly, and command_limit and the
  // REQ_IDLE zero-length check with TLP_CMD_MEM_READ.
  function automatic logic command_is_config(input tlp_cmd_e command);
    return command == TLP_CMD_CFG_READ0 || command == TLP_CMD_CFG_WRITE0 ||
           command == TLP_CMD_CFG_READ1 || command == TLP_CMD_CFG_WRITE1;
  endfunction

  // Type 1, used only by the tlp_type select. The Configuration rules (one
  // DW, 4 bytes, 3 DW header) apply to Type 0 and Type 1 alike; only the Type
  // field differs (TLP_TYPE_CFG1). An explicit member list keeps it
  // independent of the ordinals.
  function automatic logic command_is_config1(input tlp_cmd_e command);
    return command == TLP_CMD_CFG_READ1 || command == TLP_CMD_CFG_WRITE1;
  endfunction

  // I/O Read and Write, sent as TLP_TYPE_IO.
  function automatic logic command_is_io(input tlp_cmd_e command);
    return command == TLP_CMD_IO_READ || command == TLP_CMD_IO_WRITE;
  endfunction

  // The 1 DW request classes (command_limit, the REQ_IDLE length check).
  function automatic logic command_is_config_or_io(input tlp_cmd_e command);
    return command_is_config(command) || command_is_io(command);
  endfunction

  // Commands whose Completion carries data (tag_expects_data_o).
  function automatic logic command_is_read(input tlp_cmd_e command);
    return command == TLP_CMD_MEM_READ || command == TLP_CMD_CFG_READ0 ||
           command == TLP_CMD_IO_READ  || command == TLP_CMD_CFG_READ1;
  endfunction

  // Commands that carry data (command_has_data).
  function automatic logic command_is_write(input tlp_cmd_e command);
    return command == TLP_CMD_MEM_WRITE || command == TLP_CMD_CFG_WRITE0 ||
           command == TLP_CMD_IO_WRITE  || command == TLP_CMD_CFG_WRITE1;
  endfunction

  // The most bytes one TLP may span: 4 for I/O and Configuration (1 DW,
  // PCIe Base Spec r2.1, §2.2.7), max_read_bytes_i for an MRd (§2.2.7) and
  // max_payload_bytes_i for a write (§2.2.2); 128 when the input is 0.
  function automatic logic [12:0] command_limit(input tlp_cmd_e command);
    if (command_is_config_or_io(command))
      return 13'd4;
    if (command == TLP_CMD_MEM_READ)
      return max_read_bytes_i == 0 ? 13'd128 : max_read_bytes_i;
    return max_payload_bytes_i == 0 ? 13'd128 : max_payload_bytes_i;
  endfunction

  // The bytes of the next TLP: what remains, cut so that its DW span, which
  // includes the address[1:0] bytes ahead of it in the first DW, stays
  // within limit, and so that it does not cross a 4 KB boundary (PCIe Base
  // Spec r2.1, §2.2.7).
  function automatic logic [12:0] calculate_segment(
      input logic [63:0] address,
      input logic [12:0] remaining,
      input logic [12:0] limit
  );
    logic [12:0] value;
    logic [12:0] boundary;
    logic [12:0] aligned_limit;
    boundary = 13'd4096 - {1'b0, address[11:0]};
    aligned_limit = limit > {11'd0, address[1:0]} ?
                    limit - {11'd0, address[1:0]} : 13'd1;
    value = remaining;
    if (value > aligned_limit)
      value = aligned_limit;
    if (value > boundary)
      value = boundary;
    return value;
  endfunction

  always_comb begin
    command_has_data   = command_is_write(command_r);
    command_non_posted = command_r != TLP_CMD_MEM_WRITE;
    accepted_bytes = '0;
    for (lane = 0; lane < KEEP_WIDTH; lane = lane + 1)
      accepted_bytes = accepted_bytes + command_keep_i[lane];

    header_c = '0;
    // A 4 DW header only for an address at or above 4 GB (PCIe Base Spec
    // r2.1, §2.2.4.1).
    header_c.fmt = command_has_data ?
        (address_r[63:32] == 0 ? TLP_FMT_3DW_DATA : TLP_FMT_4DW_DATA) :
        (address_r[63:32] == 0 ? TLP_FMT_3DW_NO_DATA : TLP_FMT_4DW_NO_DATA);
    // A command with no arm below, TLP_CMD_MSG or TLP_CMD_MSG_DATA, would be
    // sent as an MRd.
    header_c.tlp_type = TLP_TYPE_MEM;
    if (command_is_config(command_r)) begin
      header_c.tlp_type = command_is_config1(command_r) ? TLP_TYPE_CFG1
                                                        : TLP_TYPE_CFG0;
      header_c.fmt = command_has_data ? TLP_FMT_3DW_DATA : TLP_FMT_3DW_NO_DATA;
    end else if (command_is_io(command_r)) begin
      header_c.tlp_type = TLP_TYPE_IO;
      header_c.fmt = command_has_data ? TLP_FMT_3DW_DATA : TLP_FMT_3DW_NO_DATA;
    end
    header_c.traffic_class = tc_r;
    header_c.attributes    = attr_r;
    // A zero-length read is 1 DW with both Byte Enables 0000b (PCIe Base
    // Spec r2.1, §2.2.5).
    header_c.length_dw     = segment_bytes_r == 0 ? 11'd1 :
        11'((segment_bytes_r + {11'd0, address_r[1:0]} + 13'd3) >> 2);
    header_c.requester_id  = requester_id_i;
    header_c.tag           = tag_r;
    header_c.first_be      = tlp_first_be(address_r[1:0], segment_bytes_r);
    header_c.last_be       = tlp_last_be(address_r[1:0], segment_bytes_r);
    header_c.address       = address_r;
    header_c.prefix_present = prefix_valid_r;
    header_c.prefix         = prefix_r;
    header_c.digest_present = ecrc_enable_r;
  end

  assign command_ready_o = state_r == REQ_IDLE;
  assign tag_request_valid_o = state_r == REQ_TAG;
  assign tag_requester_id_o = requester_id_i;
  // 4 for a zero-length read; see Limitations.
  assign tag_byte_count_o = segment_bytes_r == 0 ? 13'd4 : segment_bytes_r;
  assign tag_context_o = context_r;
  assign tag_expects_data_o = command_is_read(command_r);
  assign packet_header_o = header_c;
  assign packet_header_valid_o = state_r == REQ_HEADER;
  assign packet_data_o = command_data_i;
  assign packet_keep_o = command_keep_i;
  assign packet_data_valid_o = state_r == REQ_DATA && command_data_valid_i;
  assign expected_data_last = segment_sent_r + accepted_bytes >= segment_bytes_r;
  // The last beat of the whole command: this beat closes the current TLP and
  // no TLP follows (remaining_r <= segment_bytes_r). command_data_last_i
  // marks the end of the command, so it is checked against this, not against
  // expected_data_last.
  assign request_last = expected_data_last && (remaining_r <= segment_bytes_r);
  // An early command_data_last_i still closes the TLP being sent, which then
  // carries less data than its Length field; command_error_valid_o reports
  // the mismatch.
  assign packet_data_last_o = expected_data_last || command_data_last_i;
  assign command_data_ready_o = state_r == REQ_DATA && packet_data_ready_i;

  always_ff @(posedge clk_i) begin
    if (rst_i) begin
      state_r         <= REQ_IDLE;
      command_r       <= TLP_CMD_MEM_READ;
      address_r       <= '0;
      remaining_r     <= '0;
      segment_bytes_r <= '0;
      segment_sent_r  <= '0;
      tc_r            <= '0;
      attr_r          <= '0;
      context_r       <= '0;
      tag_r           <= '0;
      prefix_valid_r  <= 1'b0;
      prefix_r        <= '0;
      ecrc_enable_r   <= 1'b0;
      command_error_valid_o <= 1'b0;
      command_error_code_o <= TLP_ERR_NONE;
    end else begin
      command_error_valid_o <= 1'b0;
      command_error_code_o <= TLP_ERR_NONE;
      unique case (state_r)
        REQ_IDLE: if (command_valid_i && command_ready_o) begin
          // Only an MRd may have a byte count of 0 (a zero-length read). An
          // I/O or Configuration Request carries exactly 1 DW (PCIe Base Spec
          // r2.1, §2.2.7), but its Byte Enables need not enable all 4 bytes,
          // so any byte count that fits in the addressed DW, 4 - address[1:0]
          // or less, is accepted. Its length_dw is then 1, and
          // calculate_segment never splits it.
          if ((command_byte_count_i == 0 && command_i != TLP_CMD_MEM_READ) ||
              (command_is_config_or_io(command_i) &&
               command_byte_count_i > (13'd4 - {11'd0, command_address_i[1:0]}))) begin
            command_error_valid_o <= 1'b1;
            command_error_code_o <= TLP_ERR_BAD_LENGTH;
          end else begin
            command_r   <= command_i;
            address_r   <= command_address_i;
            remaining_r <= command_byte_count_i;
            tc_r        <= command_tc_i;
            attr_r      <= command_attr_i;
            context_r   <= command_context_i;
            prefix_valid_r <= command_prefix_valid_i;
            prefix_r       <= command_prefix_i;
            ecrc_enable_r  <= command_ecrc_enable_i;
            segment_bytes_r <= calculate_segment(command_address_i, command_byte_count_i,
                                                 command_limit(command_i));
            segment_sent_r <= '0;
            state_r <= command_i == TLP_CMD_MEM_WRITE ? REQ_HEADER : REQ_TAG;
          end
        end

        REQ_TAG: if (tag_request_ready_i) begin
          tag_r <= tag_i;
          state_r <= REQ_HEADER;
        end

        // A read takes a new tag for each TLP.
        REQ_HEADER: if (packet_header_ready_i) begin
          if (command_has_data) begin
            state_r <= REQ_DATA;
          end else if (remaining_r > segment_bytes_r) begin
            address_r <= address_r + {51'd0, segment_bytes_r};
            remaining_r <= remaining_r - segment_bytes_r;
            segment_bytes_r <= calculate_segment(address_r + {51'd0, segment_bytes_r},
                remaining_r - segment_bytes_r, command_limit(command_r));
            state_r <= REQ_TAG;
          end else begin
            state_r <= REQ_IDLE;
          end
        end

        REQ_DATA: if (command_data_valid_i && command_data_ready_o) begin
          segment_sent_r <= segment_sent_r + accepted_bytes;
          if (command_data_last_i != request_last)
            begin
              command_error_valid_o <= 1'b1;
              command_error_code_o <= TLP_ERR_LOCAL_PAYLOAD;
            end
          if (command_data_last_i && !expected_data_last) begin
            // The source ended before the byte count of the command. The
            // command ends here, after this beat closes the TLP, so that both
            // interfaces are ready for the next command.
            state_r <= REQ_IDLE;
          end else if (expected_data_last) begin
            // A command_data_last_i on the last beat of a TLP that is not the
            // command's last is reported but does not end the command: the
            // next TLP's header goes out and REQ_DATA waits for more data.
            if (remaining_r > segment_bytes_r) begin
              address_r <= address_r + {51'd0, segment_bytes_r};
              remaining_r <= remaining_r - segment_bytes_r;
              segment_bytes_r <= calculate_segment(address_r + {51'd0, segment_bytes_r},
                  remaining_r - segment_bytes_r, command_limit(command_r));
              segment_sent_r <= '0;
              state_r <= command_non_posted ? REQ_TAG : REQ_HEADER;
            end else begin
              state_r <= REQ_IDLE;
            end
          end
        end

        default: state_r <= REQ_IDLE;
      endcase
    end
  end

endmodule
