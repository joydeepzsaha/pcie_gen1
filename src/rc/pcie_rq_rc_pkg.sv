// ---------------------------------------------------------------------------
// pcie_rq_rc_pkg -- RC-surface types for the PG213 requester/completer
// descriptors (Commit 2a).
//
// These are DESCRIPTOR types, not engine types. They describe the shape of the
// Xilinx PG213 user interface, which is an artefact of the wrapper we are
// building, not of the PCIe Transaction Layer. tlp_pkg stays untouched: it is
// append-only and owned by the TL, and nothing in here belongs there.
//
// Sources for every constant below:
//   RQ descriptor field map ..... PG213 v1.3, Fig 42 / Table 61 (Configuration)
//                                 and Fig 41 / Table 60 (Memory/IO), via the
//                                 addendum -- cited as such, not read from the
//                                 PDF, which is still unavailable.
//   Request Type encodings ...... PG213 v1.3, Table 57.
//   RC descriptor field map ..... PG213 v1.3, Fig 56 / Table 65, via the
//                                 addendum -- cited as such.
//   Byte-enable rules ........... PCIe Base Spec 2.1 SS2.2.7.
//
// The byte-count arithmetic here is the wrapper's half of the contract with
// tlp_requester: the TL re-derives the byte enables internally from
// command_address[1:0] and the byte count (tlp_requester.sv:167-168 calling
// tlp_pkg::tlp_first_be / tlp_last_be), so the wrapper must produce an
// (offset, byte_count) pair that reproduces the descriptor's byte enables
// exactly. rq_byte_count() below is that pair's second half; pcie_rq_if checks
// the round trip before launching anything.
// ---------------------------------------------------------------------------
package pcie_rq_rc_pkg;

  // -------------------------------------------------------------------------
  // RQ descriptor -- 128 b, PG213 v1.3 Table 60/61.
  //
  // Declared MSB-first so the packed struct's bit numbering is literally the
  // table's. Fields that differ between the Configuration and Memory/IO forms
  // share a slot and are re-interpreted by the consumer:
  //
  //   address[63:12] : Memory/IO address bits, or Reserved for Configuration
  //   address[11:8]  : Memory/IO address bits, or Ext Reg Number
  //   address[7:2]   : Memory/IO address bits, or Register Number
  //   address[1:0]   : Address Type (AT) for Memory/IO, Reserved for Config
  //   completer_id   : the target BDF for Configuration and ID-routed requests
  // -------------------------------------------------------------------------
  typedef struct packed {
    logic        force_ecrc;       // [127]     ignored -- ECRC is TL-generated
    logic [2:0]  attr;             // [126:124] -> command_attr_i
    logic [2:0]  tc;               // [123:121] -> command_tc_i
    logic        requester_id_en;  // [120]     ignored -- TL uses requester_id_i
    logic [15:0] completer_id;     // [119:104] target BDF
    logic [7:0]  tag;              // [103:96]  IGNORED -- tags are core-managed
    logic [15:0] requester_id;     // [95:80]   IGNORED -- TL uses requester_id_i
    logic        poisoned;         // [79]
    logic [3:0]  req_type;         // [78:75]
    logic [10:0] dword_count;      // [74:64]
    logic [63:0] address;          // [63:0]    see the note above
  } rq_descriptor_t;

  // -------------------------------------------------------------------------
  // Request Type, PG213 v1.3 Table 57. All sixteen encodings are named so the
  // field is total and no value can fall outside the enum -- the eight mapped
  // ones become tlp_cmd_e, the other eight are rejected by pcie_rq_if.
  // -------------------------------------------------------------------------
  typedef enum logic [3:0] {
    RQ_MEM_READ        = 4'b0000,  // -> TLP_CMD_MEM_READ
    RQ_MEM_WRITE       = 4'b0001,  // -> TLP_CMD_MEM_WRITE
    RQ_IO_READ         = 4'b0010,  // -> TLP_CMD_IO_READ
    RQ_IO_WRITE        = 4'b0011,  // -> TLP_CMD_IO_WRITE
    RQ_MEM_FETCH_ADD   = 4'b0100,  // reject -- no atomic command path
    RQ_MEM_SWAP        = 4'b0101,  // reject
    RQ_MEM_CAS         = 4'b0110,  // reject
    RQ_MEM_RD_LOCKED   = 4'b0111,  // reject -- locked reads out of scope
    RQ_CFG_READ0       = 4'b1000,  // -> TLP_CMD_CFG_READ0
    RQ_CFG_READ1       = 4'b1001,  // -> TLP_CMD_CFG_READ1  (Stage D-2)
    RQ_CFG_WRITE0      = 4'b1010,  // -> TLP_CMD_CFG_WRITE0
    RQ_CFG_WRITE1      = 4'b1011,  // -> TLP_CMD_CFG_WRITE1 (Stage D-2)
    RQ_MSG_ROUTED      = 4'b1100,  // reject -- messages out of scope
    RQ_MSG_ID          = 4'b1101,  // reject
    RQ_MSG_VENDOR      = 4'b1110,  // reject
    RQ_MSG_ATS         = 4'b1111   // reject -- ATS/reserved
  } rq_req_type_e;

  // -------------------------------------------------------------------------
  // Why a descriptor was refused. Reported on rq_error_code_o alongside the
  // one-cycle rq_protocol_error_o pulse. RQ_ERR_NONE is never presented with
  // the pulse asserted.
  // -------------------------------------------------------------------------
  typedef enum logic [3:0] {
    RQ_ERR_NONE            = 4'd0,
    RQ_ERR_REQ_TYPE        = 4'd1,  // Request Type outside the mapped six
    RQ_ERR_DWORD_COUNT     = 4'd2,  // Dword Count 0 or > 1024
    RQ_ERR_CFG_DWORD_COUNT = 4'd3,  // Configuration request with Dword Count != 1
    RQ_ERR_CFG_IO_FIT      = 4'd4,  // config/IO request does not fit one Dword
    RQ_ERR_4KB             = 4'd5,  // request crosses a 4 KB boundary
    RQ_ERR_ADDRESS_TYPE    = 4'd6,  // AT != 00 on a Memory/IO request
    RQ_ERR_POISON_CFG_WR   = 4'd7,  // poisoned Configuration write
    RQ_ERR_BYTE_COUNT_FIT  = 4'd8,  // byte count exceeds command_byte_count_i
    RQ_ERR_BE_MISMATCH     = 4'd9,  // byte enables not reproducible by the TL
    RQ_ERR_ZERO_LENGTH     = 4'd10, // Dword Count 1 with first_be == 0
    RQ_ERR_EARLY_LAST      = 4'd11, // tlast before the Dword Count was met
    RQ_ERR_MISSING_LAST    = 4'd12  // beats continue past the Dword Count
  } rq_error_e;

  // -------------------------------------------------------------------------
  // RC descriptor -- 96 b / 3 Dwords, PG213 v1.3 Table 65.
  //
  // Declared MSB-first for the same reason as rq_descriptor_t: the packed
  // struct's bit numbering is literally the table's, so a field's position can
  // be checked against the table by eye.
  //
  // Three Dwords into a four-Dword beat is deliberate. pcie_rc_if pushes
  // desc[31:0], desc[63:32], desc[95:64], payload0, payload1, ... into
  // pcie_axis_dw_upsize in that order and the gearbox's plain concatenation
  // reproduces PG213's Dword-aligned layout -- beat 0 is
  // {payload DW0, desc DW2, desc DW1, desc DW0} and every later beat is offset
  // by one Dword. There is no rotation logic and no DESC_DW parameter anywhere;
  // the gearbox stays descriptor-blind (pcie_axis_dw_upsize.sv:27-30).
  // -------------------------------------------------------------------------
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
    logic [10:0] dword_count;        // [42:32] payload Dwords IN THIS packet
    logic        rsvd0;              // [31]
    logic        request_completed;  // [30]    last CPL of the REQUEST
    logic        locked_read;        // [29]
    logic [12:0] byte_count;         // [28:16] remaining, including this CPL
    logic [3:0]  error_code;         // [15:12] rc_desc_error_e
    logic [11:0] lower_address;      // [11:0]
  } rc_descriptor_t;

  // -------------------------------------------------------------------------
  // Completion Status, RC descriptor [45:43].
  //
  // Bit-identical to tlp_pkg::tlp_cpl_status_e (tlp_pkg.sv:36-41) and to the
  // PCIe CPL header's Completion Status field, so pcie_rc_if forwards the
  // parsed header's field verbatim with no recoding. Minted here anyway because
  // the RC surface owns its own descriptor field types -- the same rule that
  // keeps rq_req_type_e out of tlp_pkg -- and because a reader checking the
  // descriptor against Table 65 should find the encoding named next to the
  // struct rather than one package away.
  //
  // CRS (010) is NOT an oddity to be folded into "some error": an NVMe device
  // may legally answer early Configuration reads with CRS, and Commit 2b's
  // enumeration FSM has to see it to know to retry. It is carried faithfully.
  // -------------------------------------------------------------------------
  typedef enum logic [2:0] {
    RC_CPL_SC  = 3'b000,  // Successful Completion
    RC_CPL_UR  = 3'b001,  // Unsupported Request
    RC_CPL_CRS = 3'b010,  // Configuration Request Retry Status
    RC_CPL_CA  = 3'b100   // Completer Abort
  } rc_cpl_status_e;

  // -------------------------------------------------------------------------
  // Error Code, RC descriptor [15:12], PG213 v1.3 Table 65.
  //
  // RC_DESC_ERR_BAD_LENGTH IS UNREACHABLE THROUGH tlp_layer, by construction
  // rather than by omission -- see the KNOWN_GAPS block in pcie_rc_if.sv. The
  // request tracker refuses to produce a result at all for a completion with no
  // data when data was expected, or with a byte count that overruns what is
  // outstanding (tlp_request_tracker.sv:315 and :341-342): it raises
  // unexpected_r and reports TLP_ERR_COMPLETION_OVERFLOW on
  // completion_error_code_o instead. A completion that would earn Error Code
  // 0011 therefore never becomes an RC packet. The encoding is named so the
  // field is total and so the standalone bench can drive the condition
  // directly.
  // -------------------------------------------------------------------------
  typedef enum logic [3:0] {
    RC_DESC_ERR_NORMAL     = 4'b0000,  // no error
    RC_DESC_ERR_POISONED   = 4'b0001,  // the CPL was poisoned (EP set)
    RC_DESC_ERR_BAD_STATUS = 4'b0010,  // terminated by UR / CA / CRS
    RC_DESC_ERR_BAD_LENGTH = 4'b0011   // no data, or byte count overrun
  } rc_desc_error_e;

  // -------------------------------------------------------------------------
  // Why pcie_rc_if flagged the completion stream. Reported on rc_error_code_o
  // alongside the one-cycle rc_protocol_error_o pulse; the counterpart of
  // rq_error_e on the requester side. RC_ERR_NONE is never presented with the
  // pulse asserted.
  //
  // These describe the TL-facing PAYLOAD stream, not the completion itself: a
  // malformed completion is the tracker's business and arrives on
  // rc_unexpected_completion_o / rc_completion_error_code_o instead.
  // -------------------------------------------------------------------------
  typedef enum logic [3:0] {
    RC_ERR_NONE         = 4'd0,
    RC_ERR_EARLY_LAST   = 4'd1,  // payload ended before the header's Dword Count
    RC_ERR_MISSING_LAST = 4'd2,  // payload beats continued past the Dword Count
    RC_ERR_ORPHAN_DATA  = 4'd3   // payload with no result behind it -- drained
  } rc_error_e;

  // -------------------------------------------------------------------------
  // Byte-enable arithmetic
  // -------------------------------------------------------------------------

  // Number of set bits in a byte-enable nibble (0..4).
  function automatic logic [3:0] rq_popcount4(input logic [3:0] be);
    rq_popcount4 = {3'd0, be[0]} + {3'd0, be[1]} + {3'd0, be[2]} + {3'd0, be[3]};
  endfunction

  // Position of the least-significant set bit of first_be -- the byte offset
  // within the addressed Dword, which becomes command_address[1:0]. Defined as
  // 0 for first_be == 0; that case is rejected separately (RQ_ERR_ZERO_LENGTH),
  // because a zero first_be is NOT caught by the byte-enable round trip:
  // tlp_first_be(0, 0) is also 0, so the comparison agrees and lets it through.
  function automatic logic [1:0] rq_be_offset(input logic [3:0] be);
    if      (be[0]) rq_be_offset = 2'd0;
    else if (be[1]) rq_be_offset = 2'd1;
    else if (be[2]) rq_be_offset = 2'd2;
    else if (be[3]) rq_be_offset = 2'd3;
    else            rq_be_offset = 2'd0;
  endfunction

  // Bytes actually transferred, from the Dword Count and the two byte-enable
  // nibbles. Piecewise on purpose -- a single formula is wrong at n == 1:
  //
  //   n == 1 : only first_be participates; last_be must be 0000 (PCIe 2.1
  //            SS2.2.7), and the general formula's (n-2)*4 term underflows.
  //   n == 2 : the general formula with a zero middle term.
  //   n >= 3 : first Dword partial, (n-2) whole Dwords, last Dword partial.
  //
  // Worst case is n = 1024 -> 4 + 4088 + 4 = 4096, inside the 13-bit
  // command_byte_count_i.
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
  // CQ descriptor -- 128 b / 4 Dwords, PG213 v1.3 Table 52 (p. 146), the
  // Memory / I/O / Atomic form.  Stage F-1.
  //
  // Declared MSB-first so the packed struct's bit numbering is literally the
  // table's, the same convention rq_descriptor_t and rc_descriptor_t use.
  //
  // Being exactly four Dwords, this descriptor fills one 128-bit beat on its
  // own: fed desc[31:0], desc[63:32], desc[95:64], desc[127:96], payload0, ...
  // into pcie_axis_dw_upsize, beat 0 comes out as the whole descriptor and the
  // payload starts Dword-aligned at beat 1.  That IS PG213's Dword-aligned CQ
  // layout, with no rotation logic and no DESC_DW parameter -- the gearbox
  // stays descriptor-blind, exactly as the RQ/RC pair already arranged.
  // -------------------------------------------------------------------------
  typedef struct packed {
    logic        rsvd1;          // [127]
    logic [2:0]  attr;           // [126:124] 124 No Snoop, 125 RO, 126 IDO
    logic [2:0]  tc;             // [123:121]
    logic [5:0]  bar_aperture;   // [120:115] see CQ_BAR_APERTURE below
    logic [2:0]  bar_id;         // [114:112] the matched BAR index
    logic [7:0]  target_function;// [111:104] 0 -- single-function Root Complex
    logic [7:0]  tag;            // [103:96]
    logic [15:0] requester_id;   // [95:80]   the DEVICE's BDF, echoed back
    logic        rsvd0;          // [79]
    logic [3:0]  req_type;       // [78:75]   cq_req_type_e
    logic [10:0] dword_count;    // [74:64]
    logic [61:0] address;        // [63:2]
    logic [1:0]  address_type;   // [1:0]     AT, from the request header
  } cq_descriptor_t;

  // -------------------------------------------------------------------------
  // Request Type, CQ descriptor [78:75], PG213 v1.3 Table 57.
  //
  // Only the eight this Root Complex can receive and classify are named with
  // their meaning; the rest are reserved here because tlp_validator rejects
  // every other type before it reaches the completer surface (the Stage F-1
  // route census).  A Message never gets this far, which is why there is no
  // message encoding: adding one without also changing tlp_validator would be
  // a field that no stimulus can produce.
  // -------------------------------------------------------------------------
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
  // Why pcie_cq_if refused to deliver an inbound request to the host.
  // Reported on cq_error_code_o alongside the one-cycle cq_dropped_o pulse.
  //
  // ⭐ cq_dropped_o IS THE ANTI-A4 PORT.  §41.1 A4 was that an inbound request
  // is consumed and discarded with NO strobe -- indistinguishable from one
  // that was never sent.  Every path that does not put a CQ packet on the host
  // interface must raise one of these instead, so "nothing happened" stops
  // being a reachable state.  CQ_DROP_NONE is never presented with the pulse.
  // -------------------------------------------------------------------------
  typedef enum logic [3:0] {
    CQ_DROP_NONE        = 4'd0,
    CQ_DROP_UNSUPPORTED = 4'd1,  // I/O or Config -- owed a UR Completion
    CQ_DROP_NO_BAR      = 4'd2,  // Memory request that matched no enabled BAR
    CQ_DROP_BAR_OVERLAP = 4'd3,  // matched more than one BAR -- ambiguous
    CQ_DROP_EARLY_LAST  = 4'd4,  // payload ended before the header's Dword Count
    CQ_DROP_MISSING_LAST= 4'd5   // payload continued past the Dword Count
  } cq_error_e;

  // -------------------------------------------------------------------------
  // CC descriptor -- 96 b / 3 Dwords, PG213 v1.3 Table 58 (p. 168-169).
  // Stage F-1.  Declared MSB-first, same convention as the other three.
  //
  // Three fields are DELIBERATELY not forwarded to the Transaction Layer, and
  // the reasons differ:
  //
  //   target_function / completer_bus / completer_id_enable
  //       tlp_completion_generator takes the Completer ID from its own
  //       completer_id_i port, which pcie_rq_rc_top drives from the Root
  //       Complex's configured BDF.  PG213 says a Root Port "must set
  //       Completer ID Enable to 1'b1" and supply the BDF in the descriptor;
  //       honouring that would give the host a second way to set our own
  //       identity, which is a forbidden second owner.  pcie_cc_if $warnings
  //       if the bit is 0 rather than silently disagreeing with the table.
  //
  //   address_type
  //       the generator emits Completions, and a Completion's AT field is
  //       reserved (Base 2.1 SS2.2.9 p. 97).
  //
  //   locked_read
  //       TLP_TYPE_CPL_LOCK has no origination path in this design, the same
  //       KNOWN_GAP pcie_rc_if already records on its own bit 29.
  // -------------------------------------------------------------------------
  typedef struct packed {
    logic        force_ecrc;         // [95]    -> request_ecrc_enable_i
    logic [2:0]  attr;               // [94:92] 92 No Snoop, 93 RO, 94 IDO
    logic [2:0]  tc;                 // [91:89]
    logic        completer_id_enable;// [88]    not forwarded -- see above
    logic [7:0]  completer_bus;      // [87:80] not forwarded
    logic [7:0]  target_function;    // [79:72] not forwarded
    logic [7:0]  tag;                // [71:64]
    logic [15:0] requester_id;       // [63:48]
    logic        rsvd2;              // [47]
    logic        poisoned;           // [46]
    logic [2:0]  completion_status;  // [45:43] SC 000 / UR 001 / CA 100 only
    logic [10:0] dword_count;        // [42:32] payload Dwords IN THIS packet
    logic        rsvd1;              // [31]
    logic        rsvd0;              // [30]
    logic        locked_read;        // [29]    not forwarded
    logic [12:0] byte_count;         // [28:16] remaining, including this Cpl
    logic [5:0]  rsvd_hi;            // [15:10]
    logic [1:0]  address_type;       // [9:8]   not forwarded
    logic        rsvd_bit7;          // [7]
    logic [6:0]  lower_address;      // [6:0]
  } cc_descriptor_t;

  // -------------------------------------------------------------------------
  // Why pcie_cc_if rejected a host completion descriptor.
  // -------------------------------------------------------------------------
  typedef enum logic [3:0] {
    CC_ERR_NONE         = 4'd0,
    CC_ERR_BAD_STATUS   = 4'd1,  // status not one of SC / UR / CA (Table 58)
    CC_ERR_EARLY_LAST   = 4'd2,  // payload ended before the descriptor's count
    CC_ERR_MISSING_LAST = 4'd3,  // payload continued past the descriptor's count
    CC_ERR_DATA_ON_ERROR= 4'd4   // non-SC completion arrived carrying payload
  } cc_error_e;

endpackage
