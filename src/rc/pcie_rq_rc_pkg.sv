// ---------------------------------------------------------------------------
// pcie_rq_rc_pkg -- descriptor types for the PG213-style host interfaces
//
// Author: Kourosh Ghahramani
// Silicon Systems Research Lab, University of Washington
//
// Purpose
//   Defines the descriptor layouts, field encodings and error codes of the
//   four host-side AXI4-Stream interfaces (pcie_rq_if, pcie_rc_if,
//   pcie_cq_if, pcie_cc_if), and the byte-count arithmetic that pcie_rq_if
//   and pcie_cq_if share. The types describe the PG213 user interface, not
//   the Transaction Layer, whose own types are in tlp_pkg.
//
// Contents
//   RQ            rq_descriptor_t, rq_req_type_e, rq_error_e.
//   RC            rc_descriptor_t, rc_cpl_status_e, rc_desc_error_e,
//                 rc_error_e.
//   Byte enables  rq_popcount4, rq_be_offset, rq_byte_count.
//   CQ            cq_descriptor_t, cq_req_type_e, cq_error_e.
//   CC            cc_descriptor_t, cc_error_e.
//
// References
//   PG213, Table 52
//   PG213, Table 57
//   PG213, Table 58
//   PG213, Table 60
//   PG213, Table 61
//   PG213, Table 65
//   PCIe Base Spec r2.1, §2.2.5
//   PCIe Base Spec r2.1, §2.2.9
//   PCIe Base Spec r2.1, §2.3.1.1
// ---------------------------------------------------------------------------
package pcie_rq_rc_pkg;

  // -------------------------------------------------------------------------
  // RQ descriptor, 128 bits (PG213, Tables 60 and 61)
  // -------------------------------------------------------------------------
  // Declared MSB-first, so each field's bit range reads as in the tables. The
  // Memory/I/O form and the Configuration form differ in the address slot:
  //
  //   address[63:12] : Memory/IO address bits, or Reserved for Configuration
  //   address[11:8]  : Memory/IO address bits, or Ext Reg Number
  //   address[7:2]   : Memory/IO address bits, or Register Number
  //   address[1:0]   : Address Type (AT) for Memory/IO, Reserved for Config
  //   completer_id   : the target BDF of a Configuration Request
  typedef struct packed {
    logic        force_ecrc;       // [127]     not read; no request carries ECRC
    logic [2:0]  attr;             // [126:124] -> command_attr_i
    logic [2:0]  tc;               // [123:121] -> command_tc_i
    logic        requester_id_en;  // [120]     not read; TL uses requester_id_i
    logic [15:0] completer_id;     // [119:104] target BDF
    logic [7:0]  tag;              // [103:96]  not read; tags are core-managed
    logic [15:0] requester_id;     // [95:80]   not read; TL uses requester_id_i
    logic        poisoned;         // [79]
    logic [3:0]  req_type;         // [78:75]
    logic [10:0] dword_count;      // [74:64]
    logic [63:0] address;          // [63:0]    see the note above
  } rq_descriptor_t;

  // -------------------------------------------------------------------------
  // Request Type, RQ descriptor [78:75] (PG213, Table 57)
  // -------------------------------------------------------------------------
  // All sixteen encodings are named, so no value falls outside the enum. The
  // eight mapped ones become tlp_cmd_e commands; pcie_rq_if rejects the rest.
  // The names of 1101b to 1111b do not match Table 57, which assigns them to
  // a Vendor-Defined Message, an ATS Message and a reserved code.
  typedef enum logic [3:0] {
    RQ_MEM_READ        = 4'b0000,  // -> TLP_CMD_MEM_READ
    RQ_MEM_WRITE       = 4'b0001,  // -> TLP_CMD_MEM_WRITE
    RQ_IO_READ         = 4'b0010,  // -> TLP_CMD_IO_READ
    RQ_IO_WRITE        = 4'b0011,  // -> TLP_CMD_IO_WRITE
    RQ_MEM_FETCH_ADD   = 4'b0100,  // rejected: tlp_cmd_e has no AtomicOp
    RQ_MEM_SWAP        = 4'b0101,  // rejected
    RQ_MEM_CAS         = 4'b0110,  // rejected
    RQ_MEM_RD_LOCKED   = 4'b0111,  // rejected: tlp_cmd_e has no Locked Read
    RQ_CFG_READ0       = 4'b1000,  // -> TLP_CMD_CFG_READ0
    RQ_CFG_READ1       = 4'b1001,  // -> TLP_CMD_CFG_READ1
    RQ_CFG_WRITE0      = 4'b1010,  // -> TLP_CMD_CFG_WRITE0
    RQ_CFG_WRITE1      = 4'b1011,  // -> TLP_CMD_CFG_WRITE1
    RQ_MSG_ROUTED      = 4'b1100,  // rejected: any Message but ATS and vendor-defined
    RQ_MSG_ID          = 4'b1101,  // rejected: Vendor-Defined Message in Table 57
    RQ_MSG_VENDOR      = 4'b1110,  // rejected: ATS Message in Table 57
    RQ_MSG_ATS         = 4'b1111   // rejected: reserved in Table 57
  } rq_req_type_e;

  // -------------------------------------------------------------------------
  // RQ rejection reasons
  // -------------------------------------------------------------------------
  // Why pcie_rq_if refused a descriptor. Reported on rq_error_code_o with the
  // one-cycle rq_protocol_error_o pulse. RQ_ERR_NONE is never presented with
  // the pulse asserted.
  typedef enum logic [3:0] {
    RQ_ERR_NONE            = 4'd0,
    RQ_ERR_REQ_TYPE        = 4'd1,  // Request Type outside the eight mapped
    RQ_ERR_DWORD_COUNT     = 4'd2,  // Dword Count 0 or > 1024
    RQ_ERR_CFG_DWORD_COUNT = 4'd3,  // Configuration request with Dword Count != 1
    RQ_ERR_CFG_IO_FIT      = 4'd4,  // config/IO request does not fit one Dword
    RQ_ERR_4KB             = 4'd5,  // request crosses a 4 KB boundary
    RQ_ERR_ADDRESS_TYPE    = 4'd6,  // AT != 00 on a Memory/IO request
    RQ_ERR_POISON_CFG_WR   = 4'd7,  // poisoned Configuration write
    RQ_ERR_BYTE_COUNT_FIT  = 4'd8,  // byte count above 4096; never reported
    RQ_ERR_BE_MISMATCH     = 4'd9,  // byte enables not reproducible by the TL
    RQ_ERR_ZERO_LENGTH     = 4'd10, // Dword Count 1 with first_be == 0
    RQ_ERR_EARLY_LAST      = 4'd11, // tlast before the Dword Count was met
    RQ_ERR_MISSING_LAST    = 4'd12  // beats continue past the Dword Count
  } rq_error_e;

  // -------------------------------------------------------------------------
  // RC descriptor, 96 bits (PG213, Table 65)
  // -------------------------------------------------------------------------
  // Declared MSB-first, like rq_descriptor_t. pcie_rc_if pushes desc[31:0],
  // desc[63:32], desc[95:64] and then the payload Dwords into
  // pcie_axis_dw_upsize, so beat 0 is {payload DW0, desc DW2, desc DW1,
  // desc DW0} and every later beat is offset by one Dword. That is PG213's
  // Dword-aligned RC layout, with no rotation logic in either module.
  typedef struct packed {
    logic        rsvd3;              // [95]
    logic [2:0]  attr;               // [94:92] 92 No Snoop, 93 RO, 94 IDO
    logic [2:0]  tc;                 // [91:89]
    logic        rsvd2;              // [88]
    logic [15:0] completer_id;       // [87:72]
    logic [7:0]  tag;                // [71:64]
    logic [15:0] requester_id;       // [63:48]
    logic        rsvd1;              // [47]
    logic        poisoned;           // [46]
    logic [2:0]  completion_status;  // [45:43] rc_cpl_status_e
    logic [10:0] dword_count;        // [42:32] payload Dwords in this packet
    logic        rsvd0;              // [31]
    logic        request_completed;  // [30]    last Completion of the request
    logic        locked_read;        // [29]
    logic [12:0] byte_count;         // [28:16] remaining, including this CPL
    logic [3:0]  error_code;         // [15:12] rc_desc_error_e
    logic [11:0] lower_address;      // [11:0]
  } rc_descriptor_t;

  // -------------------------------------------------------------------------
  // Completion Status, RC descriptor [45:43]
  // -------------------------------------------------------------------------
  // The Completion Status encodings of the Completion header (PCIe Base Spec
  // r2.1, §2.2.9), the same values as tlp_pkg::tlp_cpl_status_e, so
  // pcie_rc_if copies the parsed field unchanged. CRS is carried as itself:
  // pcie_cfg_txn retries a Configuration Request that completes with CRS.
  typedef enum logic [2:0] {
    RC_CPL_SC  = 3'b000,  // Successful Completion
    RC_CPL_UR  = 3'b001,  // Unsupported Request
    RC_CPL_CRS = 3'b010,  // Configuration Request Retry Status
    RC_CPL_CA  = 3'b100   // Completer Abort
  } rc_cpl_status_e;

  // -------------------------------------------------------------------------
  // Error code, RC descriptor [15:12] (PG213, Table 65)
  // -------------------------------------------------------------------------
  // RC_DESC_ERR_BAD_LENGTH is declared and never driven. tlp_request_tracker
  // produces no result for a Successful Completion whose payload, Byte Count
  // or Lower Address disagrees with its request, including one with no data
  // where data is expected, nor for any completion that carries data where
  // none is expected; it pulses unexpected_completion_o with
  // TLP_ERR_COMPLETION_OVERFLOW instead, so such a completion never becomes
  // an RC packet. pcie_rc_if could not detect the no-data case itself: whether
  // a request expects data is kept inside tlp_request_tracker.
  typedef enum logic [3:0] {
    RC_DESC_ERR_NORMAL     = 4'b0000,  // no error
    RC_DESC_ERR_POISONED   = 4'b0001,  // the CPL was poisoned (EP set)
    RC_DESC_ERR_BAD_STATUS = 4'b0010,  // terminated by UR / CA / CRS
    RC_DESC_ERR_BAD_LENGTH = 4'b0011   // no data, or byte count overrun
  } rc_desc_error_e;

  // -------------------------------------------------------------------------
  // RC payload-stream errors
  // -------------------------------------------------------------------------
  // Why pcie_rc_if flagged the completion payload stream, the counterpart of
  // rq_error_e. Reported on rc_error_code_o with the one-cycle
  // rc_protocol_error_o pulse; RC_ERR_NONE is never presented with the pulse.
  // A completion that tlp_request_tracker rejects is reported on
  // rc_unexpected_completion_o and rc_completion_error_code_o instead.
  typedef enum logic [3:0] {
    RC_ERR_NONE         = 4'd0,
    RC_ERR_EARLY_LAST   = 4'd1,  // payload ended before the header's Dword Count
    RC_ERR_MISSING_LAST = 4'd2,  // payload beats continued past the Dword Count
    RC_ERR_ORPHAN_DATA  = 4'd3   // payload with no result behind it -- drained
  } rc_error_e;

  // -------------------------------------------------------------------------
  // Byte-enable arithmetic
  // -------------------------------------------------------------------------
  // tlp_requester rebuilds a request's byte enables from command_address[1:0]
  // and the byte count (tlp_first_be, tlp_last_be), so pcie_rq_if must hand it
  // an (offset, byte count) pair that reproduces the descriptor's byte enables
  // exactly. rq_be_offset gives the offset and rq_byte_count the count;
  // pcie_rq_if checks the round trip before it launches a command.

  // Number of set bits in a byte-enable nibble (0..4).
  function automatic logic [3:0] rq_popcount4(input logic [3:0] be);
    rq_popcount4 = {3'd0, be[0]} + {3'd0, be[1]} + {3'd0, be[2]} + {3'd0, be[3]};
  endfunction

  // Position of the least-significant set bit of first_be -- the byte offset
  // within the addressed Dword, which becomes command_address[1:0]. Defined as
  // 0 for first_be == 0. With Dword Count 1 that case is rejected separately
  // (RQ_ERR_ZERO_LENGTH), because the byte-enable round trip does not catch
  // it: tlp_first_be(0, 0) is also 0, so the comparison agrees.
  function automatic logic [1:0] rq_be_offset(input logic [3:0] be);
    if      (be[0]) rq_be_offset = 2'd0;
    else if (be[1]) rq_be_offset = 2'd1;
    else if (be[2]) rq_be_offset = 2'd2;
    else if (be[3]) rq_be_offset = 2'd3;
    else            rq_be_offset = 2'd0;
  endfunction

  // Number of enabled bytes, from the Dword Count and the two byte-enable
  // nibbles. Piecewise on purpose -- a single formula is wrong at n == 1:
  //
  //   n == 1 : only first_be participates; last_be must be 0000 (PCIe Base
  //            Spec r2.1, §2.2.5), and the general formula's (n-2)*4 term
  //            underflows.
  //   n == 2 : the general formula with a zero middle term.
  //   n >= 3 : first Dword partial, (n-2) whole Dwords, last Dword partial.
  //
  // Worst case is n = 1024 -> 4 + 4088 + 4 = 4096, inside the 13-bit
  // command_byte_count_i. The result equals the Total Byte Count of PCIe Base
  // Spec r2.1, §2.3.1.1 only for contiguous byte enables with a non-zero
  // first_be; that table counts the span of the enables, not the set bits.
  function automatic logic [12:0] rq_byte_count(input logic [10:0] n,
                                                input logic [3:0]  first_be,
                                                input logic [3:0]  last_be);
    logic [12:0] middle;
    if (n <= 11'd1) begin
      rq_byte_count = {9'd0, rq_popcount4(first_be)};
    end else begin
      middle = (n == 11'd2) ? 13'd0 : 13'({n - 11'd2, 2'b00});
      rq_byte_count = {9'd0, rq_popcount4(first_be)} + middle +
                      {9'd0, rq_popcount4(last_be)};
    end
  endfunction

  // -------------------------------------------------------------------------
  // CQ descriptor, 128 bits, Memory/I/O/AtomicOp form (PG213, Table 52)
  // -------------------------------------------------------------------------
  // Declared MSB-first, like the other descriptors. Four Dwords fill one
  // 128-bit beat: pcie_cq_if pushes desc[31:0] to desc[127:96] and then the
  // payload into pcie_axis_dw_upsize, so beat 0 is the whole descriptor and
  // the payload starts Dword-aligned at beat 1, as PG213 places it.
  typedef struct packed {
    logic        rsvd1;          // [127]
    logic [2:0]  attr;           // [126:124] 124 No Snoop, 125 RO, 126 IDO
    logic [2:0]  tc;             // [123:121]
    logic [5:0]  bar_aperture;   // [120:115] pcie_cq_if's CQ_BAR_APERTURE
    logic [2:0]  bar_id;         // [114:112] the matched BAR index
    logic [7:0]  target_function;// [111:104] 0 -- single-function Root Complex
    logic [7:0]  tag;            // [103:96]
    logic [15:0] requester_id;   // [95:80]   the requesting device's ID
    logic        rsvd0;          // [79]
    logic [3:0]  req_type;       // [78:75]   cq_req_type_e
    logic [10:0] dword_count;    // [74:64]
    logic [61:0] address;        // [63:2]
    logic [1:0]  address_type;   // [1:0]     AT, from the request header
  } cq_descriptor_t;

  // -------------------------------------------------------------------------
  // Request Type, CQ descriptor [78:75] (PG213, Table 57)
  // -------------------------------------------------------------------------
  // Named for the request kinds tlp_validator admits. It rejects AtomicOps
  // and Messages before they reach tlp_layer's target port, so no encoding is
  // named for them. Only CQ_MEM_READ and CQ_MEM_WRITE ever reach a CQ packet,
  // because pcie_cq_if drops every I/O and Configuration Request. Table 57
  // marks the Configuration types as requester-side only and assigns 1001b to
  // a Type 1 read and 1010b to a Type 0 write; below, 1001b is CQ_CFG_WRITE0
  // and 1010b is CQ_CFG_READ1.
  typedef enum logic [3:0] {
    CQ_MEM_READ    = 4'b0000,
    CQ_MEM_WRITE   = 4'b0001,
    CQ_IO_READ     = 4'b0010,
    CQ_IO_WRITE    = 4'b0011,
    CQ_CFG_READ0   = 4'b1000,
    CQ_CFG_WRITE0  = 4'b1001,
    CQ_CFG_READ1   = 4'b1010,
    CQ_CFG_WRITE1  = 4'b1011
  } cq_req_type_e;

  // -------------------------------------------------------------------------
  // CQ drop reasons
  // -------------------------------------------------------------------------
  // Why pcie_cq_if did not deliver an inbound request, or delivered it
  // malformed. Reported on cq_error_code_o with the one-cycle cq_dropped_o
  // pulse, so every request offered to pcie_cq_if either becomes a CQ packet
  // or is reported. CQ_DROP_NONE is never presented with the pulse.
  typedef enum logic [3:0] {
    CQ_DROP_NONE        = 4'd0,
    CQ_DROP_UNSUPPORTED = 4'd1,  // I/O or Config -- owed a UR Completion
    CQ_DROP_NO_BAR      = 4'd2,  // Memory request that matched no enabled BAR
    CQ_DROP_BAR_OVERLAP = 4'd3,  // matched more than one BAR -- ambiguous
    CQ_DROP_EARLY_LAST  = 4'd4,  // payload ended before the header's Dword Count
    CQ_DROP_MISSING_LAST= 4'd5   // payload continued past the Dword Count
  } cq_error_e;

  // -------------------------------------------------------------------------
  // CC descriptor, 96 bits (PG213, Table 58)
  // -------------------------------------------------------------------------
  // Declared MSB-first, like the other descriptors. pcie_cc_if reads none of
  // the fields marked "not read" below:
  //   target_function, completer_bus, completer_id_enable: the Completer ID
  //     comes from tlp_completion_generator's completer_id_i, which
  //     pcie_rq_rc_top drives from its own completer_id_i input. Table 58
  //     asks a Root Port to set Completer ID Enable and supply the ID here.
  //   address_type: tlp_completion_generator clears the whole header, and a
  //     Completion's AT field must be 00b (PCIe Base Spec r2.1, §2.2.9).
  //   locked_read: tlp_completion_generator emits only TLP_TYPE_CPL.
  //   poisoned: tlp_completion_generator never sets the poisoned bit.
  typedef struct packed {
    logic        force_ecrc;         // [95]    -> request_ecrc_enable_i
    logic [2:0]  attr;               // [94:92] 92 No Snoop, 93 RO, 94 IDO
    logic [2:0]  tc;                 // [91:89]
    logic        completer_id_enable;// [88]    not read
    logic [7:0]  completer_bus;      // [87:80] not read
    logic [7:0]  target_function;    // [79:72] not read
    logic [7:0]  tag;                // [71:64]
    logic [15:0] requester_id;       // [63:48]
    logic        rsvd2;              // [47]
    logic        poisoned;           // [46]    not read
    logic [2:0]  completion_status;  // [45:43] SC 000 / UR 001 / CA 100 only
    logic [10:0] dword_count;        // [42:32] payload Dwords in this packet
    logic        rsvd1;              // [31]
    logic        rsvd0;              // [30]
    logic        locked_read;        // [29]    not read
    logic [12:0] byte_count;         // [28:16] remaining, including this Cpl
    logic [5:0]  rsvd_hi;            // [15:10]
    logic [1:0]  address_type;       // [9:8]   not read
    logic        rsvd_bit7;          // [7]
    logic [6:0]  lower_address;      // [6:0]
  } cc_descriptor_t;

  // -------------------------------------------------------------------------
  // CC rejection reasons
  // -------------------------------------------------------------------------
  // Why pcie_cc_if rejected a host completion descriptor. Reported on
  // cc_error_code_o with the one-cycle cc_protocol_error_o pulse.
  typedef enum logic [3:0] {
    CC_ERR_NONE         = 4'd0,
    CC_ERR_BAD_STATUS   = 4'd1,  // status not one of SC / UR / CA (Table 58)
    CC_ERR_EARLY_LAST   = 4'd2,  // packet ended inside the descriptor or before its count
    CC_ERR_MISSING_LAST = 4'd3,  // payload continued past the descriptor's count
    CC_ERR_DATA_ON_ERROR= 4'd4   // non-SC completion arrived carrying payload
  } cc_error_e;

endpackage
