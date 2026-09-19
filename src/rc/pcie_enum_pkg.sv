// ---------------------------------------------------------------------------
// pcie_enum_pkg -- enumeration ENGINE and POLICY types for the Root Complex.
// Commit 2b-1.
//
// WHY THIS IS NOT IN pcie_rq_rc_pkg
//
// pcie_rq_rc_pkg states its own scope in its header: "These are DESCRIPTOR
// types, not engine types. They describe the shape of the Xilinx PG213 user
// interface." Everything here is the other thing -- PCI Configuration Space
// register numbers, transaction outcome classification, retry policy. None of
// it describes a descriptor field, so none of it belongs there. It is the same
// rule that keeps rq_req_type_e out of tlp_pkg (pcie_rq_rc_pkg.sv:143-146),
// applied one layer up.
//
// A second, practical reason: all six existing RC targets compile
// pcie_rq_rc_pkg. Leaving that file byte-identical means Commit 2b cannot
// perturb them at all. This package is purely additive.
//
// This package IMPORTS nothing and is imported alongside pcie_rq_rc_pkg by the
// enumeration RTL. Descriptor shapes are reused from there, never duplicated.
//
// ===========================================================================
// SS CITATION STATUS -- read before adding a constant
// ===========================================================================
//
// Every value here carries one of the tags established in
// docs/predictions/SPEC_PREDICTIONS_ENUM.md SS0.2:
//
//   [BASE]      PCI Express Base Specification Rev 2.1, section + page. Golden.
//   [PCI3]      PCI Local Bus Specification 3.0, section + page + line. Golden
//               since Commit 2b-3 put it on the shelf at
//               /home/kourosh/openPCIE/0.doc/pci-local-bus-3.0.txt. This is the
//               DISCHARGED form of the tag below; see :197 for the discharge
//               note and the page-numbering correction it required.
//   [PCI3-REF]  the historical form: the normative source is PCI 3.0, which
//               Base 2.1 incorporates by reference (definitions p.30-31) but
//               which was NOT on the shelf when a constant was first needed, so
//               the constant was withheld rather than cited loosely.
//
// !! HISTORICAL -- THIS RESTRICTION WAS LIFTED IN COMMIT 2b-3.
//
// What follows is why the [PCI3-REF] constants were withheld when this package
// was written. They are no longer withheld: PCI 3.0 reached the shelf in
// Commit 2b-3, the debt was discharged (:197), and the constants below now
// carry full [PCI3] anchors. The paragraph is kept because it records WHY the
// package was shaped this way, which is not recoverable from the code.
//
// The Command register's Memory Space Enable (bit 1) and I/O Space Enable
// (bit 0) are [PCI3-REF]: Base 2.1's Table 7-3 (SS7.5.1.1 p.485-487) maps only
// the bits whose PCI Express interpretation differs, and it STARTS AT BIT 2
// (Bus Master Enable, which IS [BASE]). The BAR bit layout and the
// write-all-ones sizing algorithm are [PCI3-REF] for the same reason -- Base
// 2.1 SS7.5.2.1 p.491-492 gives only usage policy and the 128-byte minimum.
//
// Those constants were deliberately NOT added ahead of the acquisition decision
// recorded in docs/predictions/SPEC_PREDICTIONS_ENUM.md SS9. They arrived with Commit 2b-3, by
// which time PCI 3.0 was on the shelf. Adding them earlier would have put
// uncited numbers in the tree for an increment that did not use them.
//
// The REGISTER NUMBERS below are a different matter: they are read straight off
// Base 2.1 Figure 7-5 p.491, the Type 0 Configuration Space Header, and are
// [BASE] in full.
// ---------------------------------------------------------------------------
package pcie_enum_pkg;

  // -------------------------------------------------------------------------
  // Configuration Space register numbers, Type 0 header.
  // [BASE] PCIe Base 2.1 SS7.5.2 Figure 7-5 p.491.
  //
  // A Configuration Request addresses a DWORD, not a byte: the header carries
  // Register Number[5:0] and Extended Register Number[3:0] (SS2.2.7 p.79), and
  // the byte within that Dword is selected by the byte enables. So
  // reg_num == byte_offset >> 2 throughout.
  // -------------------------------------------------------------------------
  localparam logic [5:0] CFG_REG_VENDOR_DEVICE  = 6'h00;  // 00h Vendor/Device ID
  localparam logic [5:0] CFG_REG_COMMAND_STATUS = 6'h01;  // 04h Command/Status
  localparam logic [5:0] CFG_REG_REVISION_CLASS = 6'h02;  // 08h Revision/Class Code
  localparam logic [5:0] CFG_REG_CACHE_HEADER   = 6'h03;  // 0Ch CLS/MLT/HdrType/BIST
  localparam logic [5:0] CFG_REG_BAR0           = 6'h04;  // 10h
  localparam logic [5:0] CFG_REG_BAR1           = 6'h05;  // 14h
  localparam logic [5:0] CFG_REG_BAR2           = 6'h06;  // 18h
  localparam logic [5:0] CFG_REG_BAR3           = 6'h07;  // 1Ch
  localparam logic [5:0] CFG_REG_BAR4           = 6'h08;  // 20h
  localparam logic [5:0] CFG_REG_BAR5           = 6'h09;  // 24h

  // No Extended Configuration Space is reached by 2b: every register above is
  // inside the first 256 bytes, so Extended Register Number is always zero.
  localparam logic [3:0] CFG_EXT_REG_NONE = 4'h0;

  // -------------------------------------------------------------------------
  // Byte enables for the transactions Commit 2b issues.
  // [BASE] SS2.2.5 p.67 (First/Last DW BE rules); SS2.2.7 p.79 pins Last DW BE
  // to 0000b for every Configuration Request, so only first_be is ever chosen.
  // -------------------------------------------------------------------------
  // Whole Dword. Used for the Vendor/Device ID probe: SS2.3.2 p.121 and the
  // Implementation Note p.113 require the post-reset probe to access BOTH BYTES
  // of the Vendor ID field, which sets a floor of 4'b0011; reading the whole
  // Dword satisfies that floor and returns Device ID in the same completion.
  localparam logic [3:0] CFG_BE_DWORD     = 4'b1111;
  // Bytes 0-1 of a Dword -- the Command register half of register 1.
  localparam logic [3:0] CFG_BE_LOWER_HALF = 4'b0011;
  // Byte 2 of a Dword -- Header Type, at byte offset 0Eh inside register 3.
  localparam logic [3:0] CFG_BE_BYTE2      = 4'b0100;

  // Every Configuration Request: Length must be 1 DW and Last DW BE must be
  // 0000b (SS2.2.7 p.79). Both are structural here, not runtime choices.
  localparam logic [10:0] CFG_DWORD_COUNT = 11'd1;
  localparam logic [3:0]  CFG_LAST_BE     = 4'b0000;

  // -------------------------------------------------------------------------
  // How one configuration transaction ended.
  //
  // These are OUTCOMES, not policies. The same outcome means different things
  // at different points in an enumeration -- TXN_UR on a Vendor-ID probe means
  // "no device at that BDF" (SS2.3.2 Implementation Note p.122, which names the
  // existence probe explicitly), while TXN_UR on any later access to a device
  // that has already answered is a fault. pcie_cfg_txn reports; the sequencer
  // decides. Keeping that split is why the primitive is its own module.
  //
  // There is deliberately NO reserved-status outcome. Base 2.1 SS2.3.2 p.122:
  // "Completions with a Reserved Completion Status value are treated as if the
  // Completion Status was Unsupported Request (UR)." A reserved encoding is
  // therefore indistinguishable from UR by design, and inventing a sixth code
  // would invite a consumer to treat it differently than the spec allows.
  // rsp_status_raw_o carries the untranslated encoding for logging.
  // -------------------------------------------------------------------------
  typedef enum logic [2:0] {
    TXN_OK             = 3'd0,  // Successful Completion; read data valid
    TXN_UR             = 3'd1,  // Unsupported Request, or a Reserved status
    TXN_CA             = 3'd2,  // Completer Abort
    TXN_CRS_EXHAUSTED  = 3'd3,  // CRS returned CRS_RETRY_MAX times over
    TXN_TIMEOUT        = 3'd4   // completion timeout; tag quarantined upstream
  } txn_outcome_e;

  // -------------------------------------------------------------------------
  // CRS retry policy defaults. Both are IMPLEMENTATION-DEFINED.
  //
  // Base 2.1 SS2.3.2 p.121 makes Root Complex handling of a CRS Completion
  // "implementation specific", requires the not-CRS-Software-Visible case to
  // "re-issue the Configuration Request as a new Request", and explicitly
  // permits a bound: "A Root Complex implementation may choose to limit the
  // number of Configuration Request/CRS Completion Status loops" (p.121-122).
  // So the cap is sanctioned by the spec; its VALUE is not specified anywhere.
  //
  // These are simulation-convenience values. ⚠️ CPL_TIMEOUT_CYCLES USED TO BE
  // CITED HERE AS A FELLOW TRAVELLER AT 4096; SS63 #7e found that one was not
  // merely inconvenient but NON-CONFORMANT at this design's 8 ns clock (32.8 us
  // against SS7.8.16's required 50 us floor) and moved it to 6250. These two
  // CRS constants have NOT been re-examined against the spec and keep their
  // original status. Real hardware must
  // tolerate the PCI/PCI-X Trhfa recovery window (SS2.3.2 Implementation Note
  // p.113); choosing a real pair belongs with the Device Control 2 programming
  // that is Stage-H work.
  //
  // !! THE RELATIONSHIP IS THE REAL CONSTRAINT, NOT EITHER NUMBER.
  // P-CRS-BUDGET (docs/predictions/SPEC_PREDICTIONS_ENUM.md SS5.2):
  //     CRS_RETRY_MAX * CRS_BACKOFF_CYCLES < CPL_TIMEOUT_CYCLES
  // If a retry storm can outlast the completion timeout, a device that is
  // merely slow to initialise becomes indistinguishable from a dead one.
  // 16 * 64 = 1024, comfortably inside 6250 -- §63 #7e raised CPL_TIMEOUT_CYCLES
  // from 4096, and the guard holds at both values. pcie_cfg_txn checks this at
  // elaboration so an override cannot silently break it.
  // -------------------------------------------------------------------------
  localparam int unsigned CRS_RETRY_MAX_DEFAULT     = 16;
  localparam int unsigned CRS_BACKOFF_CYCLES_DEFAULT = 64;

  // -------------------------------------------------------------------------
  // SS PRESENCE SCAN (Commit 2b-2)
  // -------------------------------------------------------------------------

  // ⭐ DEVICE 0 ONLY. There is no device-number loop, and this is a derived
  // constant rather than a tuning parameter.
  //
  // [BASE] SS7.3.1 p.479 twice: (1) "Configuration Requests specifying all other
  // Device Numbers (1-31) must be terminated by the Switch Downstream Port or
  // the Root Port with an Unsupported Request Completion Status" -- in a
  // conventional Root Complex that UR is synthesised by the Downstream Port and
  // the request never reaches the wire, but THIS DESIGN HAS NO SUCH LOGIC (CQ/CC
  // is tied off, pcie_rq_rc_top.sv:96-99), so such a request would actually be
  // transmitted, which the rule forbids. And (2) "Non-ARI Devices must respond
  // to all Type 0 Configuration Read Requests, REGARDLESS of the Device Number
  // specified in the Request" -- so if one were transmitted, the attached device
  // would answer it. A 0-31 sweep on a direct-attach link would therefore
  // discover the SAME DEVICE 32 TIMES, not one device and 31 absences.
  //
  // Scanning device 0 only satisfies SS7.3.1 BY CONSTRUCTION: no request naming
  // device 1-31 is ever formed, so there is nothing to terminate. Root-Port
  // termination logic becomes relevant only when a switch can sit below the
  // port, which is Commits 3/4.
  //
  // Full derivation: docs/predictions/SPEC_PREDICTIONS_ENUM.md SSD.1.
  localparam int unsigned DEVICES_TO_SCAN = 1;

  // Header Type lives in byte 2 of register 3 -- byte offset 0Eh.
  // [BASE] Figure 7-5 p.491 (and Figure 7-4 p.484, Figure 7-6 p.492).
  localparam int HDR_TYPE_LSB = 16;   // bits [23:16] of the register-3 Dword

  // Header Type bit fields.
  // [PCI3] PCI 3.0 SS6.2.1 p.216 :10685. Base 2.1 shows Header Type only in the
  // figures above and never defines its bits.
  //
  // !! THE [PCI3-REF] DEBT THAT USED TO SIT HERE IS DISCHARGED (Commit 2b-3).
  // PCI Local Bus 3.0 is now on the spec shelf at
  //   /home/kourosh/openPCIE/0.doc/pci-local-bus-3.0.txt
  // and every constant below carries a section + page + line anchor. The full
  // anchor table, and the page-numbering correction it needed (page markers in
  // this extraction are FOOTERS, so content after marker N is on page N+1), are
  // in docs/predictions/SPEC_PREDICTIONS_ENUM.md SSE.0.
  //
  // Base 2.1 DOES independently establish what the two layouts MEAN, which is
  // what actually justifies the FSM's behaviour: SS7.5.2 p.491 titles the Type 0
  // header as the one for "PCI Express device Functions", SS7.5.3 p.492 titles
  // Type 1 as the one for "Switch and Root Complex virtual PCI Bridges". Only
  // the numeric encoding was ever owed to PCI 3.0.
  localparam int         HDR_MULTIFUNCTION_BIT = 7;
  localparam logic [6:0] HDR_LAYOUT_TYPE0 = 7'h00;  // endpoint Function
  localparam logic [6:0] HDR_LAYOUT_TYPE1 = 7'h01;  // PCI-PCI bridge

  // -------------------------------------------------------------------------
  // SS BAR SIZING, ASSIGNMENT AND ENABLE (Commit 2b-3)
  //
  // Every constant in this section is [PCI3], cited to section + page + line.
  // Derivations in docs/predictions/SPEC_PREDICTIONS_ENUM.md SSE.1-SSE.7.
  // -------------------------------------------------------------------------

  // The candidate register window. A Type 0 header has SIX Base Address
  // registers at offsets 10h..24h == registers 4..9 -- [BASE] Figure 7-5 p.491.
  //
  // ⭐ THE WINDOW IS CLOSED AT 9, AND THAT IS A DECISION, NOT AN OVERSIGHT.
  // The Expansion ROM Base Address register sits at offset 30h == register 12
  // ([PCI3] SS6.2.5.2 p.227 :11283, :11287) and behaves ALMOST like a BAR --
  // :11290 "functions exactly like a 32-bit Base Address register except that
  // the encoding (and usage) of the bottom bits is different", and p.228 :11307
  // gives it the same all-ones sizing procedure. That near-identity is exactly
  // why it needs an explicit exclusion rather than silence: its bit 0 is
  // Expansion ROM Enable, NOT a Memory Space Indicator (Figure 6-7, p.228
  // :11318, :11323), so feeding it to the SSE.2 decoder would produce a WRONG
  // answer rather than a harmless one. Asserted on the wire as P-NO-ROM.
  localparam logic [5:0] CFG_REG_BAR_FIRST = CFG_REG_BAR0;   // 4
  localparam logic [5:0] CFG_REG_BAR_LAST  = CFG_REG_BAR5;   // 9
  // Six registers, hence at most six BARs -- and at most six only if every one
  // of them is 32-bit. A 64-bit BAR occupies two registers but is ONE BAR.
  localparam int unsigned BAR_SLOTS = 6;

  // Base Address register bit layout -- [PCI3] SS6.2.5.1 p.225-226.
  //
  // The load-bearing sentence is p.226 :11205, "Bits 0-3 are read-only." That
  // one clause is what makes all-ones sizing work at all: the write cannot
  // disturb the type/prefetch encoding, so the readback still identifies the
  // BAR while its upper bits report the size.
  localparam int BAR_BIT_IO        = 0;  // 1 = I/O space, 0 = memory  -- :11187, :11190
  localparam int BAR_TYPE_LSB      = 1;  // bits [2:1], Table 6-4      -- :11207
  localparam int BAR_BIT_PREFETCH  = 3;  // memory BARs only           -- :11193

  // Table 6-4 Bits 2/1 Encoding, ALL FOUR ROWS -- [PCI3] p.226 :11207.
  //
  // !! BOTH reserved encodings are decoded, not just 01. Footnote 46 (:11243)
  // records that 01 formerly meant "below 1 MB" and that "System software should
  // recognize this encoding and handle appropriately" -- it is a LEGACY encoding,
  // not a free slot, and 11 is Reserved in exactly the same way. Neither can
  // legitimately appear on a PCI Express endpoint, and treating an unknown type
  // as 32-bit would silently mis-size the device, so both are faults.
  localparam logic [1:0] BAR_TYPE_32BIT     = 2'b00;
  localparam logic [1:0] BAR_TYPE_RESERVED1 = 2'b01;
  localparam logic [1:0] BAR_TYPE_64BIT     = 2'b10;
  localparam logic [1:0] BAR_TYPE_RESERVED3 = 2'b11;

  // Sizing masks. Two bits wide for I/O, four for memory -- a real spec
  // distinction, not an off-by-one: an I/O BAR has no prefetch bit and no type
  // field, so its bits 3:2 are ORDINARY ADDRESS BITS.
  //   memory: bits 3:0 read-only                        -- :11205
  //   I/O:    bit 0 hardwired 1, bit 1 reserved-reads-0 -- :11187
  localparam logic [31:0] BAR_MEM_MASK = ~32'hF;
  localparam logic [31:0] BAR_IO_MASK  = ~32'h3;

  // The all-ones probe value -- [PCI3] SS6.2.5.1 p.226 :11222: "writing a value
  // of all 1's to the register and then reading the value back".
  localparam logic [31:0] BAR_PROBE_ALL_ONES = 32'hFFFF_FFFF;

  // ⭐ THE MINIMUM LEGAL MEMORY BAR IS 128 BYTES, NOT 16.
  //
  // Two specifications disagree and the tighter one governs:
  //   PCI 3.0    "a single memory size that is a power of 2 from 16 bytes to
  //              2 GB"                        -- SS6.2.5.1 p.226 :11219
  //   PCIe 2.1   "The minimum Memory Space address range requested by a BAR is
  //              128 bytes."                  -- [BASE] SS7.5.2.1 p.491-492
  //
  // This design enumerates a PCI Express link. PCI 3.0 is incorporated only for
  // definitions Base 2.1 does not restate; where Base 2.1 states a STRICTER
  // requirement it wins. A 16-byte memory BAR is legal PCI and illegal PCIe.
  //
  // !! AND THE DISAGREEMENT IS WHAT MAKES A MIS-DECODE DETECTABLE. The upper
  // half of a 64-bit size field reads back FFFFFFFF; decoded as a standalone
  // 32-bit BAR that is ~FFFFFFF0 + 1 = 16 bytes -- precisely PCI 3.0's minimum,
  // which is precisely what PCIe forbids. So a 64-bit pair mis-decoded as two
  // independent 32-bit BARs trips this floor BY SPEC rather than because a test
  // happened to look. docs/predictions/SPEC_PREDICTIONS_ENUM.md SSE.4.1.
  localparam logic [63:0] BAR_MEM_MIN_BYTES = 64'd128;

  // Command register bits -- [PCI3] SS6.2.2 Table 6-1 p.218.
  //
  // ⭐ ALL THREE ARE 0 AFTER RESET (":10761", ":10764", ":10767" each say "State
  // after RST# is 0"), AND THAT IS THE FACT THAT MAKES ALL-ONES SIZING SAFE. At
  // enumeration time the device's decoders are off, so a BAR momentarily holding
  // FFFFFFFF -- naming an enormous range -- cannot make the device claim any
  // transaction. A device with Memory Space Enable already set would briefly
  // decode that range. The same fact is what makes it safe to assign BAR N while
  // BAR N+2 is still unprobed, and to write the two halves of a 64-bit pair
  // non-atomically. ONE invariant covers all three: SSE.5.
  localparam int CMD_BIT_IO_ENABLE     = 0;  // :10761
  localparam int CMD_BIT_MEM_ENABLE    = 1;  // :10764
  localparam int CMD_BIT_BUS_MASTER    = 2;  // :10767, and [BASE] Table 7-3

  // The value written at the end of enumeration: Memory Space Enable + Bus
  // Master Enable. I/O Space Enable stays 0 because no I/O BAR is ever assigned
  // (SSE.7.2), so enabling I/O decode would advertise ranges that were never
  // programmed. Bits 15:3 stay 0: SERR#/parity/interrupt policy is Stage H.
  localparam logic [31:0] CMD_ENABLE_VALUE =
      (32'd1 << CMD_BIT_MEM_ENABLE) | (32'd1 << CMD_BIT_BUS_MASTER);   // 0x0006

  // -------------------------------------------------------------------------
  // SS BRIDGE BUS-NUMBER ASSIGNMENT (Stage D)
  //
  // The Type 1 Configuration Space Header -- [BASE] SS7.5.3 Figure 7-6 p.492,
  // the map of record for Switch and Root Complex virtual PCI Bridges (its
  // SS7.5.3 p.493 scope note).  NOT PCI 3.0: SS6.1 p.214 defers Header Type
  // 01h to the PCI-to-PCI Bridge Architecture Specification, which is not on
  // the shelf.  Settled in docs/predictions/SPEC_PREDICTIONS_STAGE_D.md SS0.2 -- do not
  // re-derive.
  // -------------------------------------------------------------------------

  // Register 6 == byte offset 18h in a TYPE 1 header:
  //   {Secondary Latency Timer[31:24], Subordinate[23:16], Secondary[15:8],
  //    Primary[7:0]}
  //
  // !! P4.7: THIS IS WHERE A TYPE 0 HEADER'S BAR2 SITS.  A Type 1 header has
  // TWO BARs (10h/14h, SS7.5.3.1 p.493), not six -- a six-BAR sizing sweep
  // pointed at a bridge would write all-ones HERE and destroy the bus-number
  // assignment, silently, after every preceding probe passed.  A BAR stage
  // must never be pointed at a Type 1 Function in Stage D.
  localparam logic [5:0] CFG_REG_BUS_NUMBER = 6'h06;

  // The values, forced pairwise-apart per docs/predictions/SPEC_PREDICTIONS_STAGE_D.md P5.2:
  // Secondary is non-zero, != primary, and NOT primary+1 (so off-by-one from
  // the parent is distinguishable from correct); Subordinate != Secondary (so
  // writing the same value into both fields is caught by the whole-Dword
  // golden).  All [DESIGN]: Stage D fixes them a priori -- a general
  // enumerator discovers Subordinate by descending and needs TWO writes per
  // bridge (P5.7); this single-write shape is Stage-D-specific.
  localparam logic [7:0] SEC_BUS_NUMBER = 8'h05;
  localparam logic [7:0] SUB_BUS_NUMBER = 8'h09;

  // Secondary Latency Timer: [BASE] SS7.5.3.3 p.493 -- "does not apply to PCI
  // Express.  It must be read-only and hardwired to 00h."  The whole-Dword
  // write necessarily drives the byte; 00h is the only defensible value, and
  // a read-back golden must expect 00h REGARDLESS of what was written (P4.2).
  localparam logic [7:0] SEC_LATENCY_TIMER_WDATA = 8'h00;

  // -------------------------------------------------------------------------
  // Why the scan stopped badly.
  //
  // There is deliberately NO code for "device absent": absence is not an error.
  // On a point-to-point link with link_up_i asserted a device is always
  // attached, so absence means "nothing here to enumerate" -- an Unsupported
  // Request to the Function 0 probe (SS7.3.1 p.479, SS7.3.3 p.480) -- and lands in
  // the normal terminal state with device_present_o low.
  //
  // Nor is there a code for "unsupported device": a Type 1 header is a valid
  // device that answered correctly and that this commit cannot enumerate yet.
  // Reporting it as an error would conflate "the link misbehaved" with "the
  // topology is richer than I handle", and only the first is a fault.
  //
  // !! WIDENED FROM [2:0] TO [3:0] IN COMMIT 2b-3. The four BAR-phase faults
  // below need codes 5..8 and would not fit. Codes 0..4 keep their values
  // BYTE-IDENTICALLY, so every existing scan assertion is unaffected; the two
  // scan shims widen their flattening cast by one bit and nothing else moves.
  // Each BAR fault gets its OWN code rather than sharing one, because SSE.1,
  // SSE.4 and SSE.7.1 each independently ask for "a distinct code" -- and
  // because SSE.9's EF8 requires the allocator test to assert WHICH fault fired,
  // not merely that enum_error_o rose.
  // -------------------------------------------------------------------------
  typedef enum logic [3:0] {
    ENUM_ERR_NONE          = 4'd0,
    // UR on anything AFTER the probe: a device that answered register 0 has no
    // business rejecting a legal configuration read of register 3.
    ENUM_ERR_UR_POST_PROBE = 4'd1,
    ENUM_ERR_CA            = 4'd2,  // Completer Abort, any phase
    ENUM_ERR_CRS_EXHAUSTED = 4'd3,  // CRS_RETRY_MAX retries all returned CRS
    // Completion timeout, any phase -- INCLUDING the probe. Absence answers
    // with UR (SS2.3.2 Implementation Note p.122, which names the device
    // existence probe explicitly); silence is a reported error that "should
    // never occur under normal operating conditions" (SS2.8 p.152). The two are
    // different events and the FSM must not merge them.
    ENUM_ERR_TIMEOUT       = 4'd4,

    // ---- BAR phase (Commit 2b-3) -------------------------------------------
    // A memory BAR whose Type field [2:1] decoded to a RESERVED encoding, 01 or
    // 11 -- [PCI3] Table 6-4 p.226 :11207. Neither can legitimately appear on a
    // PCI Express endpoint. Guessing 32-bit would silently mis-size the device.
    ENUM_ERR_BAR_TYPE      = 4'd5,
    // The decoded size is not a legal PCIe memory BAR size: either below the
    // 128-byte floor ([BASE] SS7.5.2.1 p.491-492) or not a power of two
    // ([PCI3] p.226 :11226, "all address spaces used are a power of two in size
    // and are naturally aligned"). Both mean the same thing operationally --
    // THE DECODE ITSELF IS WRONG -- so they share one code. Continuing would
    // assign an address derived from a size just proved untrustworthy, and the
    // natural-alignment mask ~(size-1) is meaningless for a non-power-of-two.
    ENUM_ERR_BAR_SIZE      = 4'd6,
    // Allocating this BAR would run past MEM_BAR_BASE + MEM_BAR_WINDOW.
    // NEVER a silent wraparound: a wrapped allocation produces OVERLAPPING BARs
    // that appear to enumerate successfully and fail much later, in Stage F, as
    // data corruption. SSE.7.1.
    ENUM_ERR_BAR_WINDOW    = 4'd7,
    // A 32-bit BAR was allocated an address that does not fit in 32 bits.
    // Reachable only when MEM_BAR_BASE is set above 4 GB, which is a legitimate
    // parameterization for a 64-bit-BAR-only device -- so the case is real, and
    // the alternative is truncating the upper half and producing an overlap.
    // Distinct from ENUM_ERR_BAR_WINDOW: the window is fine, the REGISTER is
    // too narrow to name the address. (Beyond SSE's named set; see the module
    // header, SS ONE FAULT SSE DOES NOT NAME.)
    ENUM_ERR_BAR_ADDR32    = 4'd8,

    // ---- sec 63 #7f, #19 (D-P3.5) --------------------------------------------
    // A completion timeout on a request the transmitter's credit gate was
    // HOLDING when the timer expired.  tlp_request_tracker measures per-tag
    // age from ALLOCATION, and allocation precedes the credit gate
    // (pcie_enum_scan.sv header), so such a request times out WITHOUT EVER
    // HAVING BEEN TRANSMITTED.  Until this code it was reported as
    // ENUM_ERR_TIMEOUT with err_credit_blocked_o as a side annotation, and a
    // reader of enum_error_code_o alone saw "dead device" -- which is exactly
    // what the full stack reported for F18, where #18 (the shared DLL never
    // returning NP credit) starved the 17th non-posted request (Phase 2e).
    // err_credit_blocked_o is UNCHANGED and still set alongside; the
    // annotation input tx_fc_blocked_i still steers no next-state expression
    // (pinned by S15) -- it now chooses between two NAMES for one terminal
    // fault, which is reporting, not control flow.  No timer changes here:
    // the completion timer running from allocation is registered to #7g.
    ENUM_ERR_CREDIT_STARVED = 4'd9
  } enum_error_e;

endpackage
