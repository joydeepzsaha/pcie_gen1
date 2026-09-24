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
    // RESERVED ENCODINGS -- declared, never decoded.  Nothing in this tree
    // drives ordinal 8 or 9, no test constructs one, and no datapath decodes
    // one.  The six command_is_* predicates in tlp_requester each return 0 for
    // both, because each is an explicit member list and an unlisted member
    // matches no term.
    //
    // They exist so the command encoding is ALREADY the union of this tree's
    // set and the endpoint branch's, which lets tlp_pkg.sv merge as "take
    // ours".  The alternative -- adopting the other branch's numbering -- would
    // move CFG_READ1/CFG_WRITE1 off 6 and 7, and three bench files bind those
    // two ordinals as Python integers while eight more bind ordinals 0..5.
    // See docs/recon/RECON_MERGE.md SSR1 for the collision and docs/findings/M1_FINDINGS.md for the gate.
    //
    // !! WARNING to whoever gives these a datapath: the requester FAILS OPEN on
    // them today.  command_non_posted is derived as "!= TLP_CMD_MEM_WRITE", so
    // a message would read as non-posted although messages are posted; and the
    // tlp_type select has no message arm, so header_c.tlp_type falls through to
    // TLP_TYPE_MEM and a message command would be emitted as a well-formed
    // Memory Read.  That is pre-existing out-of-range behaviour -- see the
    // comment above command_is_config in tlp_requester.sv, which records the
    // same failure mode -- and it is harmless ONLY while these stay undriven.
    // Adding a message datapath means fixing both, not just adding arms.
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

  function automatic logic tlp_has_data(input logic [2:0] fmt);
    return fmt == TLP_FMT_3DW_DATA || fmt == TLP_FMT_4DW_DATA;
  endfunction

  function automatic logic tlp_is_4dw(input logic [2:0] fmt);
    return fmt == TLP_FMT_4DW_NO_DATA || fmt == TLP_FMT_4DW_DATA;
  endfunction

  function automatic logic [9:0] tlp_encode_length(input logic [10:0] length_dw);
    return length_dw == 11'd1024 ? 10'd0 : length_dw[9:0];
  endfunction

  function automatic logic [10:0] tlp_decode_length(input logic [9:0] encoded);
    return encoded == 10'd0 ? 11'd1024 : {1'b0, encoded};
  endfunction

  function automatic logic [12:0] tlp_payload_bytes(input logic [10:0] length_dw);
    return {length_dw, 2'b00};
  endfunction

  function automatic logic [11:0] tlp_data_credits(input logic [10:0] length_dw);
    logic [12:0] bytes;
    bytes = tlp_payload_bytes(length_dw);
    return 12'((bytes + 13'd15) >> 4);
  endfunction

  function automatic tlp_credit_class_e tlp_credit_class(input tlp_class_e packet_class);
    case (packet_class)
      TLP_CLASS_POSTED:     return TLP_CREDIT_POSTED;
      TLP_CLASS_COMPLETION: return TLP_CREDIT_COMPLETION;
      default:              return TLP_CREDIT_NON_POSTED;
    endcase
  endfunction

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

  // sec 63 #7g-2 step 3 (Kourosh Q1): the SHIPPED Completion Timeout, 10 ms --
  // Base 2.1 sec 7.8.15 p.545 / sec 7.8.16 p.550: required range 50 us - 50 ms,
  // "strongly recommended that the Completion Timeout mechanism not expire in
  // less than 10 ms" -- at the design's 8 ns link clock (D-7G.2's
  // CLK_PERIOD_NS default).  ONE source: every CPL_TIMEOUT_CYCLES default in
  // src/ names this.  Was 6250 = 50 us, the floor (sec 63 #7e).  A bench that
  // must SEE a timeout overrides it visibly (D-7G.2); tb/tlp's t1c pins it.
  localparam int unsigned CPL_TIMEOUT_DEFAULT_CYCLES = 10_000_000 / 8;

endpackage
