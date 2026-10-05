// ---------------------------------------------------------------------------
// tlp_pkg -- Transaction Layer types, encodings and helper functions
//
// Purpose
//   Shared definitions for the Transaction Layer in src/tlp and for the
//   Endpoint and Root Complex modules that drive it: TLP field encodings,
//   the decoded header, the command and error codes of tlp_layer's ports,
//   and the arithmetic for lengths, byte enables, credits and the ECRC.
//
// Contents
//   Parameters  TLP_DATA_WIDTH, TLP_KEEP_WIDTH, TLP_MAX_PAYLOAD_BYTES: not
//               used by any module. CPL_TIMEOUT_DEFAULT_CYCLES: the default
//               Completion Timeout.
//   Encodings   tlp_fmt_e (Fmt), tlp_type_e (Type), tlp_cpl_status_e
//               (Completion Status), as in the TLP header.
//   Codes       tlp_class_e (Posted, Non-Posted, Completion, unsupported),
//               tlp_cmd_e (the tlp_requester commands), tlp_credit_class_e
//               (the credit pools), tlp_error_e (the error codes).
//   Header      tlp_header_t: the fields of a Memory, I/O, Configuration or
//               Completion header, plus one TLP Prefix and the TLP Digest.
//               length_dw is the DW count (1024 for a Length field of 0, 0
//               for a Completion without data); byte_count is 4096 for a
//               Byte Count field of 0.
//   Functions   Fmt tests (tlp_has_data, tlp_is_4dw); Length encoding
//               (tlp_encode_length, tlp_decode_length); payload bytes and
//               credits (tlp_payload_bytes, tlp_data_credits,
//               tlp_credit_class); ECRC steps (tlp_crc32_byte, tlp_crc32_dw);
//               contiguous Byte Enables (tlp_first_be, tlp_last_be).
//
// References
//   PCIe Base Spec r2.1, §2.2.1
//   PCIe Base Spec r2.1, §2.2.5
//   PCIe Base Spec r2.1, §2.2.9
//   PCIe Base Spec r2.1, §2.6.1
//   PCIe Base Spec r2.1, §2.7.1
//   PCIe Base Spec r2.1, §7.8.16
// ---------------------------------------------------------------------------
`timescale 1ns/1ps
package tlp_pkg;

  parameter int TLP_DATA_WIDTH = 32;
  parameter int TLP_KEEP_WIDTH = TLP_DATA_WIDTH / 8;
  parameter int TLP_MAX_PAYLOAD_BYTES = 4096;

  typedef enum logic [2:0] {
    TLP_FMT_3DW_NO_DATA = 3'b000,
    TLP_FMT_4DW_NO_DATA = 3'b001,
    TLP_FMT_3DW_DATA    = 3'b010,
    TLP_FMT_4DW_DATA    = 3'b011,
    TLP_FMT_PREFIX      = 3'b100
  } tlp_fmt_e;

  typedef enum logic [4:0] {
    TLP_TYPE_MEM       = 5'b00000,
    TLP_TYPE_MEM_LOCK  = 5'b00001,
    TLP_TYPE_IO        = 5'b00010,
    TLP_TYPE_CFG0      = 5'b00100,
    TLP_TYPE_CFG1      = 5'b00101,
    TLP_TYPE_CPL       = 5'b01010,
    TLP_TYPE_CPL_LOCK  = 5'b01011,
    TLP_TYPE_FETCH_ADD = 5'b01100,
    TLP_TYPE_SWAP      = 5'b01101,
    TLP_TYPE_CAS       = 5'b01110
  } tlp_type_e;

  typedef enum logic [1:0] {
    TLP_CLASS_POSTED,
    TLP_CLASS_NON_POSTED,
    TLP_CLASS_COMPLETION,
    TLP_CLASS_UNSUPPORTED
  } tlp_class_e;

  typedef enum logic [2:0] {
    TLP_CPL_SC  = 3'b000,
    TLP_CPL_UR  = 3'b001,
    TLP_CPL_CRS = 3'b010,
    TLP_CPL_CA  = 3'b100
  } tlp_cpl_status_e;

  typedef enum logic [3:0] {
    TLP_CMD_MEM_READ,
    TLP_CMD_MEM_WRITE,
    TLP_CMD_CFG_READ0,
    TLP_CMD_CFG_WRITE0,
    TLP_CMD_IO_READ,
    TLP_CMD_IO_WRITE,
    TLP_CMD_CFG_READ1,
    TLP_CMD_CFG_WRITE1,
    // TLP_CMD_MSG and TLP_CMD_MSG_DATA have no datapath: nothing drives
    // them, and every command_is_* predicate in tlp_requester is false for
    // them. tlp_requester would send either as an MRd, since its tlp_type
    // select has no Message arm, and would treat it as Non-Posted, since
    // command_non_posted is true for every command but TLP_CMD_MEM_WRITE,
    // although Messages are Posted; a Message datapath has to change both.
    // New members go at the end: the cocotb benches bind the ordinals as
    // integers (CMD_CFG_READ1 = 6 in test_tlp_conf_cfg1.py).
    TLP_CMD_MSG,
    TLP_CMD_MSG_DATA
  } tlp_cmd_e;

  typedef enum logic [1:0] {
    TLP_CREDIT_POSTED,
    TLP_CREDIT_NON_POSTED,
    TLP_CREDIT_COMPLETION
  } tlp_credit_class_e;

  typedef enum logic [4:0] {
    TLP_ERR_NONE,
    TLP_ERR_TRUNCATED_HEADER,
    TLP_ERR_EARLY_EOP,
    TLP_ERR_LATE_EOP,
    TLP_ERR_BAD_KEEP,
    TLP_ERR_BAD_FMT_TYPE,
    TLP_ERR_BAD_LENGTH,
    TLP_ERR_BAD_BYTE_ENABLE,
    TLP_ERR_BAD_ADDRESS_FORMAT,
    TLP_ERR_ECRC,
    TLP_ERR_UNEXPECTED_COMPLETION,
    TLP_ERR_COMPLETION_OVERFLOW,
    TLP_ERR_CREDIT_UNDERFLOW,
    TLP_ERR_LOCAL_PAYLOAD,
    TLP_ERR_VC_OVERFLOW
  } tlp_error_e;

  typedef struct packed {
    logic [2:0]  fmt;
    logic [4:0]  tlp_type;
    logic [2:0]  traffic_class;
    logic [2:0]  attributes;
    logic        digest_present;
    logic        poisoned;
    logic        th;
    logic [1:0]  address_type;
    logic [10:0] length_dw;
    logic [15:0] requester_id;
    logic [15:0] completer_id;
    logic [7:0]  tag;
    logic [3:0]  first_be;
    logic [3:0]  last_be;
    logic [63:0] address;
    logic [2:0]  completion_status;
    logic        byte_count_modified;
    logic [12:0] byte_count;
    logic [6:0]  lower_address;
    logic        prefix_present;
    logic [31:0] prefix;
    logic [31:0] digest;
  } tlp_header_t;

  // Fmt 010b or 011b: the TLP carries a data payload.
  function automatic logic tlp_has_data(input logic [2:0] fmt);
    return fmt == TLP_FMT_3DW_DATA || fmt == TLP_FMT_4DW_DATA;
  endfunction

  // Fmt 001b or 011b: a 4 DW header.
  function automatic logic tlp_is_4dw(input logic [2:0] fmt);
    return fmt == TLP_FMT_4DW_NO_DATA || fmt == TLP_FMT_4DW_DATA;
  endfunction

  // A Length field of 0 means 1024 DW (PCIe Base Spec r2.1, §2.2.1).
  function automatic logic [9:0] tlp_encode_length(input logic [10:0] length_dw);
    return length_dw == 11'd1024 ? 10'd0 : length_dw[9:0];
  endfunction

  // The inverse of tlp_encode_length for 1 to 1024 DW.
  function automatic logic [10:0] tlp_decode_length(input logic [9:0] encoded);
    return encoded == 10'd0 ? 11'd1024 : {1'b0, encoded};
  endfunction

  // Bytes in length_dw DWs.
  function automatic logic [12:0] tlp_payload_bytes(input logic [10:0] length_dw);
    return {length_dw, 2'b00};
  endfunction

  // A data credit is 4 DW, and a TLP takes its Length divided by 4, rounded
  // up (PCIe Base Spec r2.1, §2.6.1).
  function automatic logic [11:0] tlp_data_credits(input logic [10:0] length_dw);
    logic [12:0] bytes;
    bytes = tlp_payload_bytes(length_dw);
    return 12'((bytes + 13'd15) >> 4);
  endfunction

  // The credit pool of a TLP class; TLP_CLASS_UNSUPPORTED maps to
  // Non-Posted.
  function automatic tlp_credit_class_e tlp_credit_class(input tlp_class_e packet_class);
    case (packet_class)
      TLP_CLASS_POSTED:     return TLP_CREDIT_POSTED;
      TLP_CLASS_COMPLETION: return TLP_CREDIT_COMPLETION;
      default:              return TLP_CREDIT_NON_POSTED;
    endcase
  endfunction

  // One byte of the ECRC: reflected CRC-32, polynomial EDB8_8320h (the bit
  // reverse of 04C1_1DB7h), data bit 0 first (PCIe Base Spec r2.1, §2.7.1).
  function automatic logic [31:0] tlp_crc32_byte(
      input logic [31:0] crc_in,
      input logic [7:0] data_in
  );
    logic [31:0] crc;
    integer bit_index;
    crc = crc_in;
    for (bit_index = 0; bit_index < 8; bit_index = bit_index + 1) begin
      if (crc[0] ^ data_in[bit_index])
        crc = (crc >> 1) ^ 32'hedb8_8320;
      else
        crc = crc >> 1;
    end
    return crc;
  endfunction

  // The bytes of one DW whose keep_in bit is set, lane 0 first.
  function automatic logic [31:0] tlp_crc32_dw(
      input logic [31:0] crc_in,
      input logic [31:0] data_in,
      input logic [3:0] keep_in
  );
    logic [31:0] crc;
    integer byte_index;
    crc = crc_in;
    for (byte_index = 0; byte_index < 4; byte_index = byte_index + 1)
      if (keep_in[byte_index])
        crc = tlp_crc32_byte(crc, data_in[byte_index*8 +: 8]);
    return crc;
  endfunction

  // 1st DW BE for byte_length bytes starting at byte address_low of the
  // first DW: the lanes that hold a byte of the range, 0000b for a length
  // of 0.
  function automatic logic [3:0] tlp_first_be(
      input logic [1:0] address_low,
      input logic [12:0] byte_length
  );
    logic [3:0] mask;
    integer lane;
    integer first_lane;
    integer end_lane;
    mask = '0;
    first_lane = address_low;
    end_lane = address_low + byte_length;
    for (lane = 0; lane < 4; lane = lane + 1)
      if (lane >= first_lane && lane < end_lane)
        mask[lane] = 1'b1;
    return mask;
  endfunction

  // Last DW BE for the same range: 0000b when the range fits in one DW, as
  // a 1 DW Request requires, else the lanes of the last DW up to the last
  // byte (PCIe Base Spec r2.1, §2.2.5).
  function automatic logic [3:0] tlp_last_be(
      input logic [1:0] address_low,
      input logic [12:0] byte_length
  );
    logic [2:0] end_offset;
    logic [13:0] end_position;
    if (({11'd0, address_low} + byte_length) <= 13'd4)
      return 4'b0000;
    end_position = {12'd0,address_low} + {1'b0,byte_length};
    end_offset = {1'b0,end_position[1:0]};
    return end_offset == 0 ? 4'b1111 : (4'b1111 >> (4-end_offset));
  endfunction

  // The default of every CPL_TIMEOUT_CYCLES parameter in src: 10 ms in
  // cycles of an 8 ns clock. A Function without Completion Timeout
  // programmability must time out between 50 us and 50 ms, and a timeout of
  // at least 10 ms is strongly recommended (PCIe Base Spec r2.1, §7.8.16).
  // test_tlp_cpl_timeout_default.py checks this value through an instance
  // that leaves CPL_TIMEOUT_CYCLES at its default.
  localparam int unsigned CPL_TIMEOUT_DEFAULT_CYCLES = 10_000_000 / 8;

endpackage
