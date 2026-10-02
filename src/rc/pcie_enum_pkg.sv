// ---------------------------------------------------------------------------
// pcie_enum_pkg -- types and constants of the Root Complex enumeration engine
//
// Author: Kourosh Ghahramani
// Silicon Systems Research Lab, University of Washington
//
// Purpose
//   Shared by pcie_cfg_txn, pcie_enum_scan, pcie_enum_bar, pcie_enum_bus and
//   the tops that instantiate them. It imports nothing; the PG213 descriptor
//   types the engine also uses come from pcie_rq_rc_pkg.
//
// Contents
//   Registers     CFG_REG_*: Type 0 header register numbers (byte offset / 4),
//                 and CFG_EXT_REG_NONE.
//   Byte enables  CFG_BE_*, plus CFG_DWORD_COUNT and CFG_LAST_BE, which are
//                 fixed for every Configuration Request.
//   Outcome       txn_outcome_e: how one configuration transaction ended.
//   CRS retry     CRS_RETRY_MAX_DEFAULT, CRS_BACKOFF_CYCLES_DEFAULT.
//   Presence      DEVICES_TO_SCAN; Header Type position and layout codes.
//   BARs          candidate window, bit fields, Type encodings, sizing masks,
//                 the all-ones probe value and the 128-byte minimum.
//   Command       Command register bits and CMD_ENABLE_VALUE.
//   Bridge        register 6 of a Type 1 header and the values written to it.
//   Errors        enum_error_e: why enumeration stopped with an error.
//
// References
//   PCI Local Bus Spec r3.0, §6.1
//   PCI Local Bus Spec r3.0, §6.2.1
//   PCI Local Bus Spec r3.0, §6.2.2
//   PCI Local Bus Spec r3.0, §6.2.5
//   PCIe Base Spec r2.1, §2.2.7
//   PCIe Base Spec r2.1, §2.3.2
//   PCIe Base Spec r2.1, §2.8
//   PCIe Base Spec r2.1, §6.6.1
//   PCIe Base Spec r2.1, §7.3.1
//   PCIe Base Spec r2.1, §7.3.3
//   PCIe Base Spec r2.1, §7.5.1
//   PCIe Base Spec r2.1, §7.5.2
//   PCIe Base Spec r2.1, §7.5.3
// ---------------------------------------------------------------------------
package pcie_enum_pkg;

  // -------------------------------------------------------------------------
  // Configuration Space register numbers, Type 0 header
  // -------------------------------------------------------------------------
  // Offsets from PCIe Base Spec r2.1, §7.5.2, Figure 7-5. A Configuration
  // Request addresses a Dword: its header carries Register Number[5:0] and
  // Extended Register Number[3:0] (PCIe Base Spec r2.1, §2.2.7), and the byte
  // enables select bytes within that Dword. A register number is therefore
  // the byte offset divided by 4.
  localparam logic [5:0] CFG_REG_VENDOR_DEVICE  = 6'h00;  // 00h Vendor/Device ID
  localparam logic [5:0] CFG_REG_COMMAND_STATUS = 6'h01;  // 04h Command/Status
  localparam logic [5:0] CFG_REG_REVISION_CLASS = 6'h02;  // 08h Revision/class code
  localparam logic [5:0] CFG_REG_CACHE_HEADER   = 6'h03;  // 0Ch CLS/MLT/HdrType/BIST
  localparam logic [5:0] CFG_REG_BAR0           = 6'h04;  // 10h
  localparam logic [5:0] CFG_REG_BAR1           = 6'h05;  // 14h
  localparam logic [5:0] CFG_REG_BAR2           = 6'h06;  // 18h
  localparam logic [5:0] CFG_REG_BAR3           = 6'h07;  // 1Ch
  localparam logic [5:0] CFG_REG_BAR4           = 6'h08;  // 20h
  localparam logic [5:0] CFG_REG_BAR5           = 6'h09;  // 24h

  // Every register the engine accesses lies in the first 256 bytes, so the
  // Extended Register Number is always zero.
  localparam logic [3:0] CFG_EXT_REG_NONE = 4'h0;

  // -------------------------------------------------------------------------
  // Byte enables
  // -------------------------------------------------------------------------
  // Last DW BE is 0000b for every Configuration Request (PCIe Base Spec r2.1,
  // §2.2.7), so only the first Dword's enables are ever chosen.
  //
  // The whole Dword: every access except the Command write. On the Vendor ID
  // probe it returns the Device ID in the same completion.
  localparam logic [3:0] CFG_BE_DWORD     = 4'b1111;
  // Bytes 0-1: the Command register, the lower half of register 1.
  localparam logic [3:0] CFG_BE_LOWER_HALF = 4'b0011;
  // Byte 2: Header Type, at offset 0Eh in register 3.
  localparam logic [3:0] CFG_BE_BYTE2      = 4'b0100;

  // Length is 1 Dword and Last DW BE is 0000b for every Configuration Request
  // (PCIe Base Spec r2.1, §2.2.7), so neither is a runtime choice.
  localparam logic [10:0] CFG_DWORD_COUNT = 11'd1;
  localparam logic [3:0]  CFG_LAST_BE     = 4'b0000;

  // -------------------------------------------------------------------------
  // How one configuration transaction ended
  // -------------------------------------------------------------------------
  // These are outcomes, not policy. TXN_UR on the Vendor ID probe means there
  // is nothing to enumerate; TXN_UR on a later access to a device that has
  // already answered is a fault. pcie_cfg_txn reports the outcome, and the
  // stage that issued the request decides what it means.
  //
  // There is no outcome for a Reserved Completion Status: the Requester treats
  // one as Unsupported Request (PCIe Base Spec r2.1, §2.3.2), so it reports as
  // TXN_UR. pcie_cfg_txn's rsp_status_raw_o carries the encoding as received.
  typedef enum logic [2:0] {
    TXN_OK             = 3'd0,  // Successful Completion; read data valid
    TXN_UR             = 3'd1,  // Unsupported Request, or a Reserved status
    TXN_CA             = 3'd2,  // Completer Abort
    TXN_CRS_EXHAUSTED  = 3'd3,  // CRS on the request and on every reissue
    TXN_TIMEOUT        = 3'd4   // completion timeout; tag quarantined upstream
  } txn_outcome_e;

  // -------------------------------------------------------------------------
  // CRS retry defaults
  // -------------------------------------------------------------------------
  // Root Complex handling of a Configuration Request Retry Status completion
  // is implementation specific, and a Root Complex may limit how many times
  // it reissues the request (PCIe Base Spec r2.1, §2.3.2); pcie_cfg_txn
  // reissues it as a new Request. The specification gives no value for
  // either constant, and neither is tied to the 1.0 s a Root Complex must
  // allow after a Conventional Reset before it judges a device broken (PCIe
  // Base Spec r2.1, §6.6.1).
  //
  // pcie_cfg_txn warns at elaboration when CRS_RETRY_MAX * CRS_BACKOFF_CYCLES
  // is not below its CPL_TIMEOUT_CYCLES; 16 * 64 = 1024 is far below the
  // default. The warning has no run-time counterpart (see pcie_cfg_txn).
  localparam int unsigned CRS_RETRY_MAX_DEFAULT     = 16;
  localparam int unsigned CRS_BACKOFF_CYCLES_DEFAULT = 64;

  // -------------------------------------------------------------------------
  // Presence scan
  // -------------------------------------------------------------------------
  // Device 0 only. pcie_enum_scan has no device-number loop; this constant
  // records that, and no module reads it. A Root Port associates only Device
  // 0 with the device on its Link and must answer a request naming Devices
  // 1-31 with UR (PCIe Base Spec r2.1, §7.3.1). Nothing in this design does
  // that: pcie_rq_if forwards a Type 0 request to any Device Number, with a
  // warning. A non-ARI device also answers every Type 0 Configuration Read
  // whatever its Device Number (PCIe Base Spec r2.1, §7.3.1), so a sweep of
  // Devices 0-31 over one Link would find the same device 32 times.
  localparam int unsigned DEVICES_TO_SCAN = 1;

  // Header Type is byte 2 of register 3, offset 0Eh (PCIe Base Spec r2.1,
  // §7.5.1, Figure 7-4).
  localparam int HDR_TYPE_LSB = 16;   // bits [23:16] of the register-3 Dword

  // Header Type bit 7 is the multi-function bit; bits 6:0 give the layout of
  // the rest of the header, 00h for the Type 0 layout and 01h for a
  // PCI-to-PCI bridge (PCI Local Bus Spec r3.0, §6.2.1). In PCI Express a
  // device Function has a Type 0 header (PCIe Base Spec r2.1, §7.5.2), and
  // the virtual PCI Bridges of Switches and Root Complexes have a Type 1
  // header (PCIe Base Spec r2.1, §7.5.3).
  localparam int         HDR_MULTIFUNCTION_BIT = 7;
  localparam logic [6:0] HDR_LAYOUT_TYPE0 = 7'h00;  // device Function
  localparam logic [6:0] HDR_LAYOUT_TYPE1 = 7'h01;  // PCI-PCI bridge

  // -------------------------------------------------------------------------
  // BAR sizing, assignment and enable
  // -------------------------------------------------------------------------
  // The candidate window: the six Base Address registers of a Type 0 header,
  // offsets 10h-24h, registers 4-9 (PCIe Base Spec r2.1, §7.5.2). It ends at
  // register 9 on purpose. The Expansion ROM Base Address register (offset
  // 30h, register 12) is sized the same way, but its bit 0 is the Expansion
  // ROM Enable bit, not a space indicator (PCI Local Bus Spec r3.0,
  // §6.2.5.2), so pcie_enum_bar's decode would misread it.
  localparam logic [5:0] CFG_REG_BAR_FIRST = CFG_REG_BAR0;   // 4
  localparam logic [5:0] CFG_REG_BAR_LAST  = CFG_REG_BAR5;   // 9
  // Six registers allow at most six BARs; a 64-bit BAR occupies two registers
  // and counts as one.
  localparam int unsigned BAR_SLOTS = 6;

  // Base Address register bits (PCI Local Bus Spec r3.0, §6.2.5.1). Bits 3:0
  // of a memory BAR are read-only, so the all-ones sizing write leaves the
  // type and prefetch bits intact and the readback still identifies the BAR.
  localparam int BAR_BIT_IO        = 0;  // 1 = I/O space, 0 = memory
  localparam int BAR_TYPE_LSB      = 1;  // bits [2:1], Table 6-4
  localparam int BAR_BIT_PREFETCH  = 3;  // memory BARs only

  // Memory BAR bits 2:1 (PCI Local Bus Spec r3.0, §6.2.5.1, Table 6-4): 00b
  // is 32-bit, 10b is 64-bit, and 01b and 11b are reserved. 01b is the
  // encoding earlier versions of PCI used for space below 1 MB. pcie_enum_bar
  // faults on both reserved encodings rather than guess a width.
  localparam logic [1:0] BAR_TYPE_32BIT     = 2'b00;
  localparam logic [1:0] BAR_TYPE_RESERVED1 = 2'b01;
  localparam logic [1:0] BAR_TYPE_64BIT     = 2'b10;
  localparam logic [1:0] BAR_TYPE_RESERVED3 = 2'b11;

  // Sizing masks. Bits 3:0 of a memory BAR are read-only encoding bits. An
  // I/O BAR has no type or prefetch field: bit 0 is hardwired to 1, bit 1 is
  // reserved and reads 0, and bits 3:2 are address bits (PCI Local Bus Spec
  // r3.0, §6.2.5.1).
  localparam logic [31:0] BAR_MEM_MASK = ~32'hF;
  localparam logic [31:0] BAR_IO_MASK  = ~32'h3;

  // Written to a BAR to size it: the readback holds 0 in every address bit
  // the BAR does not decode (PCI Local Bus Spec r3.0, §6.2.5.1).
  localparam logic [31:0] BAR_PROBE_ALL_ONES = 32'hFFFF_FFFF;

  // The smallest memory BAR is 128 bytes. PCI allows 16 bytes (PCI Local Bus
  // Spec r3.0, §6.2.5.1); PCI Express raises the minimum to 128 (PCIe Base
  // Spec r2.1, §7.5.2.1), and this engine enumerates a PCI Express Link.
  localparam logic [63:0] BAR_MEM_MIN_BYTES = 64'd128;

  // Command register bits (PCI Local Bus Spec r3.0, §6.2.2, Table 6-1). All
  // three are 0 after reset, so the device responds to no memory or I/O
  // access while its BARs are sized and assigned: a BAR briefly holding
  // FFFFFFFFh, a BAR assigned before a later one is probed, and a 64-bit BAR
  // between its two writes are all harmless. pcie_enum_bar therefore writes
  // the Command register last. PCIe Base Spec r2.1, §7.5.1.1 gives Bus
  // Master Enable its PCI Express meaning.
  localparam int CMD_BIT_IO_ENABLE     = 0;
  localparam int CMD_BIT_MEM_ENABLE    = 1;
  localparam int CMD_BIT_BUS_MASTER    = 2;

  // The last write of enumeration: Memory Space Enable and Bus Master
  // Enable. I/O Space Enable stays 0 because pcie_enum_bar assigns no I/O
  // BAR. Bits 15:3 stay 0.
  localparam logic [31:0] CMD_ENABLE_VALUE =
      (32'd1 << CMD_BIT_MEM_ENABLE) | (32'd1 << CMD_BIT_BUS_MASTER);   // 0x0006

  // -------------------------------------------------------------------------
  // Bridge bus-number assignment
  // -------------------------------------------------------------------------
  // The layout used below is the Type 1 Configuration Space Header of Switch
  // and Root Complex virtual PCI Bridges (PCIe Base Spec r2.1, §7.5.3, Figure
  // 7-6). PCI Local Bus Spec r3.0, §6.1 defers the Type 1 layout to the
  // PCI-to-PCI Bridge Architecture Specification, so PCIe Base Spec r2.1,
  // §7.5.3 is the reference.

  // Register 6, offset 18h, of a Type 1 header holds {Secondary Latency
  // Timer[31:24], Subordinate Bus Number[23:16], Secondary Bus Number[15:8],
  // Primary Bus Number[7:0]}. A Type 1 header has only two BARs, at 10h and
  // 14h (PCIe Base Spec r2.1, §7.5.3.1); in a Type 0 header register 6 is
  // BAR2. A six-BAR sizing pass aimed at a bridge would write all-ones over
  // its bus numbers, so pcie_enum_bar configures only a device whose header
  // is Type 0.
  localparam logic [5:0] CFG_REG_BUS_NUMBER = 6'h06;

  // Fixed in advance, not discovered: one write per bridge is enough only
  // because Subordinate is known before the secondary bus is scanned (see
  // pcie_enum_bus). The two values differ, so a value written into the wrong
  // byte of register 6 shows in the written Dword.
  localparam logic [7:0] SEC_BUS_NUMBER = 8'h05;
  localparam logic [7:0] SUB_BUS_NUMBER = 8'h09;

  // PCI Express has no use for the Secondary Latency Timer: it is hardwired
  // to 00h and read-only (PCIe Base Spec r2.1, §7.5.3.3). The whole-Dword
  // write still carries the byte, so it carries 00h, and a readback returns
  // 00h whatever was written.
  localparam logic [7:0] SEC_LATENCY_TIMER_WDATA = 8'h00;

  // -------------------------------------------------------------------------
  // Why enumeration stopped with an error
  // -------------------------------------------------------------------------
  // There is no code for an absent device: a UR to the Function 0 probe ends
  // the scan normally, with device_present_o low (PCIe Base Spec r2.1,
  // §7.3.3). There is none for an unsupported device either: a header layout
  // other than Type 0 belongs to a device that answered correctly, and the
  // scan ends normally with unsupported_device_o high.
  //
  // The BAR-phase faults have codes of their own, so enum_error_code_o alone
  // separates a bad Type, a bad size, an exhausted window and a 32-bit BAR
  // that cannot hold its address.
  typedef enum logic [3:0] {
    ENUM_ERR_NONE          = 4'd0,
    // UR on any access after a successful probe: a device that answered
    // register 0 has no reason to reject a legal access to its own registers.
    ENUM_ERR_UR_POST_PROBE = 4'd1,
    ENUM_ERR_CA            = 4'd2,  // Completer Abort, any phase
    ENUM_ERR_CRS_EXHAUSTED = 4'd3,  // CRS on the request and on every reissue
    // Completion timeout, in any phase including the probe. An absent device
    // answers with UR, while the completion timeout is meant to fire only when
    // no Completion can be expected (PCIe Base Spec r2.1, §2.8), so the two
    // are reported apart.
    ENUM_ERR_TIMEOUT       = 4'd4,

    // ---- BAR phase ---------------------------------------------------------
    // A memory BAR declares a reserved Type, 01b or 11b (PCI Local Bus Spec
    // r3.0, §6.2.5.1, Table 6-4), or declares 64 bits in BAR5, which has no
    // register after it to pair with.
    ENUM_ERR_BAR_TYPE      = 4'd5,
    // The decoded size is below the 128-byte minimum (PCIe Base Spec r2.1,
    // §7.5.2.1) or is not a power of two (PCI Local Bus Spec r3.0,
    // §6.2.5.1). Either way the decode cannot be trusted, and the
    // natural-alignment mask size - 1 means nothing for a size that is not a
    // power of two.
    ENUM_ERR_BAR_SIZE      = 4'd6,
    // The BAR would end past MEM_BAR_BASE + MEM_BAR_WINDOW. The allocator
    // never wraps: a wrapped allocation would give overlapping BARs.
    ENUM_ERR_BAR_WINDOW    = 4'd7,
    // A 32-bit BAR would not lie entirely below 4 GB. Possible only when
    // MEM_BAR_BASE + MEM_BAR_WINDOW is above 4 GB, which suits a device whose
    // BARs are all 64-bit; truncating the address instead could overlap
    // whatever lies low in memory. The window has room; the register is too
    // narrow.
    ENUM_ERR_BAR_ADDR32    = 4'd8,

    // ---- completion timeout while credit-blocked ---------------------------
    // A completion timeout reported while tx_fc_blocked_i was high, which
    // indicates the request was still waiting at the Transaction Layer's
    // credit gate. tlp_request_tracker starts a request's timer at tag
    // allocation, which precedes the credit gate, and restarts it when the
    // request is handed to the Data Link Layer, so a request held at the gate
    // for the whole timeout interval times out without being transmitted.
    // err_credit_blocked_o is set with this code. tx_fc_blocked_i only
    // chooses between this code and ENUM_ERR_TIMEOUT; it steers no state
    // transition.
    ENUM_ERR_CREDIT_STARVED = 4'd9
  } enum_error_e;

endpackage
