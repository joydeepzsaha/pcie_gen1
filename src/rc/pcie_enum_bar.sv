// ---------------------------------------------------------------------------
// pcie_enum_bar -- BAR sizing, assignment and Command enable for one device
//
// Author: Kourosh Ghahramani
// Silicon Systems Research Lab, University of Washington
//
// Purpose
//   Configures the device a pcie_enum_scan found, if it is present and its
//   header is Type 0. For each candidate register 4-9 it writes all-ones,
//   reads the register back and decodes it. An unimplemented register or an
//   I/O BAR is skipped; a 32-bit or 64-bit memory BAR is sized, given the
//   next naturally aligned address from MEM_BAR_BASE up, and written. After
//   the last candidate it writes the Command register to enable Memory Space
//   and Bus Master. Every transaction goes through the pcie_cfg_txn instance
//   in pcie_enum_top.
//
// Interfaces
//   Control       bar_start_i: a level, sampled in S_IDLE only.
//   Scan verdict  device_present_i, unsupported_device_i: the verdict of the
//                 scan that found the device.
//   Annotation    tx_fc_blocked_i: qualifies a timeout report only.
//   Status        bar_busy_o, enum_done_o, bar_error_o, bar_error_code_o,
//                 err_credit_blocked_o: done and error are terminal.
//   Result        bar_count_o, and bar_valid_o, bar_is_64_o, bar_prefetch_o,
//                 bar_size_o, bar_addr_o per BAR slot in discovery order;
//                 io_bar_mask_o per candidate register.
//   Transaction   cmd_*, rsp_*: the command and response ports of pcie_cfg_txn.
//
// Clock and reset
//   clk_i only. rst_i is synchronous and active high, and is the only way out
//   of S_DONE and S_ERROR.
//
// Limitations
//   Memory BARs only: an I/O BAR is logged in io_bar_mask_o and left
//   unassigned, and I/O Space Enable stays 0. The Expansion ROM Base Address
//   register is not sized. Memory decode is not disabled before sizing: the
//   module relies on Memory Space Enable being 0, its value after reset
//   (PCI Local Bus Spec r3.0, §6.2.2), and on running once per reset.
//
// Structure
//   Ports
//   Allocator limits
//   State machine
//   Readback decode
//   Allocation
//   Outcome classification
//   Sequencer
//   Command port
//   Status
//   Elaboration checks
//
// References
//   PCI Local Bus Spec r3.0, §6.2.2
//   PCI Local Bus Spec r3.0, §6.2.5.1
//   PCIe Base Spec r2.1, §7.5.1.2
//   PCIe Base Spec r2.1, §7.5.2
//   PCIe Base Spec r2.1, §7.5.2.1
// ---------------------------------------------------------------------------
`timescale 1ns/1ps
module pcie_enum_bar
  import pcie_rq_rc_pkg::*;
  import pcie_enum_pkg::*;
#(
    // Where memory BARs are allocated from, ascending. Not the BAR_BASE of
    // tlp_bar_decoder and tlp_layer, which sets the decode of received
    // requests.
    parameter logic [63:0] MEM_BAR_BASE   = 64'h0000_0000_8000_0000,
    // Allocation past MEM_BAR_BASE + MEM_BAR_WINDOW is a terminal error
    // (ENUM_ERR_BAR_WINDOW), never a wraparound, which would give
    // overlapping BARs. That relies on the sum being at most 2^64, which
    // nothing checks (see Allocation).
    parameter logic [63:0] MEM_BAR_WINDOW = 64'h0000_0000_1000_0000
) (
    input  logic                        clk_i,
    input  logic                        rst_i,

    // ---- control -------------------------------------------------------------
    // A level, not a pulse: pcie_enum_top drives it from bar_enable_i &&
    // scan_done_o, or for the second bus from bridge_enable_i &&
    // sec_scan_done_o. Sampled in S_IDLE only.
    input  logic                        bar_start_i,
    // Only a present device with a Type 0 header is configured. An absent
    // device, or a header layout other than Type 0, ends in S_DONE with no
    // transaction and bar_count_o 0: neither is an error.
    input  logic                        device_present_i,
    input  logic                        unsupported_device_i,

    // ---- annotation input, not control flow --------------------------------
    // Read only to choose a timeout's error code and err_credit_blocked_o; no
    // state transition depends on it.
    input  logic                        tx_fc_blocked_i,

    // ---- status surface ------------------------------------------------------
    // Every output below is stable from the cycle enum_done_o or bar_error_o
    // rises until reset.
    output logic                        bar_busy_o,
    output logic                        enum_done_o,
    output logic                        bar_error_o,
    output enum_error_e                 bar_error_code_o,
    output logic                        err_credit_blocked_o,

    // BARs programmed, not registers consumed: a 64-bit BAR occupies two
    // registers and counts once.
    output logic [3:0]                  bar_count_o,
    // Indexed by BAR slot in discovery order, not by register number.
    output logic [BAR_SLOTS-1:0]        bar_valid_o,
    output logic [BAR_SLOTS-1:0]        bar_is_64_o,
    output logic [BAR_SLOTS-1:0]        bar_prefetch_o,
    output logic [BAR_SLOTS*64-1:0]     bar_size_o,
    output logic [BAR_SLOTS*64-1:0]     bar_addr_o,
    // The I/O BARs that were skipped. Indexed by candidate register, not by
    // BAR slot: bit k means register CFG_REG_BAR_FIRST + k read back with bit
    // 0 set. An I/O BAR takes no slot, so a slot index could not name it.
    output logic [BAR_SLOTS-1:0]        io_bar_mask_o,

    // ---- pcie_cfg_txn command port -------------------------------------------
    // There is no BDF output and no Type select: pcie_enum_top drives
    // cmd_bdf_i from the device_bdf_o of the scan that found the device, and
    // cmd_type1_i by bus level. No tag reaches this module (see
    // pcie_enum_scan).
    output logic                        cmd_valid_o,
    input  logic                        cmd_ready_i,
    output logic                        cmd_write_o,
    output logic [5:0]                  cmd_reg_num_o,
    output logic [3:0]                  cmd_ext_reg_o,
    output logic [3:0]                  cmd_first_be_o,
    output logic [31:0]                 cmd_wdata_o,

    // ---- pcie_cfg_txn response port ------------------------------------------
    input  logic                        rsp_valid_i,
    output logic                        rsp_ready_o,
    input  txn_outcome_e                rsp_outcome_i,
    input  logic [31:0]                 rsp_rdata_i
);

  // -------------------------------------------------------------------------
  // Allocator limits
  // -------------------------------------------------------------------------
  // MEM_BAR_LIMIT is computed in 65 bits, so the sum of the two parameters
  // cannot wrap. A 32-bit Base Address register can hold only addresses
  // below 4 GB.
  localparam logic [64:0] MEM_BAR_LIMIT =
      {1'b0, MEM_BAR_BASE} + {1'b0, MEM_BAR_WINDOW};
  localparam logic [64:0] ADDR32_LIMIT = 65'h0_0000_0001_0000_0000;

  // -------------------------------------------------------------------------
  // State machine
  // -------------------------------------------------------------------------
  //   State              Does                     Exit
  //   S_IDLE             waits                    bar_start_i: S_CHECK
  //   S_CHECK            checks the verdict       configurable: S_SIZE_WR;
  //                                               else S_DONE
  //   S_SIZE_WR(_RSP)    all-ones to cand_r       S_SIZE_RD
  //   S_SIZE_RD(_RSP)    reads cand_r back and    skip: next candidate; 64-bit:
  //                      decodes it               S_UP_WR; 32-bit: S_ASSIGN_LO
  //   S_UP_WR(_RSP)      all-ones to cand_r + 1   S_UP_RD
  //   S_UP_RD(_RSP)      reads cand_r + 1 back,   S_ASSIGN_LO
  //                      sizes and allocates
  //   S_ASSIGN_LO(_RSP)  writes address[31:0]     64-bit: S_ASSIGN_UP;
  //                                               32-bit: next candidate
  //   S_ASSIGN_UP(_RSP)  writes address[63:32]    next candidate
  //   S_CMD_WR(_RSP)     writes CMD_ENABLE_VALUE  S_DONE
  //   S_DONE             terminal, no error       reset
  //   S_ERROR            terminal, error          reset
  // Each command state moves to its _RSP state on cmd_ready_i, and each _RSP
  // state acts on rsp_valid_i. Any outcome other than TXN_OK, or a failed
  // decode, size or allocation check, goes to S_ERROR. Skip means an I/O BAR
  // or an unimplemented register. Next candidate means S_SIZE_WR for the next
  // register (the one after the pair, for a 64-bit BAR), or S_CMD_WR once
  // past CFG_REG_BAR_LAST. There is no timer here: a missing completion
  // arrives as TXN_TIMEOUT from the completion timeout in tlp_request_tracker.
  typedef enum logic [4:0] {
    S_IDLE,
    S_CHECK,
    S_SIZE_WR,
    S_SIZE_WR_RSP,
    S_SIZE_RD,
    S_SIZE_RD_RSP,
    S_UP_WR,
    S_UP_WR_RSP,
    S_UP_RD,
    S_UP_RD_RSP,
    S_ASSIGN_LO,
    S_ASSIGN_LO_RSP,
    S_ASSIGN_UP,
    S_ASSIGN_UP_RSP,
    S_CMD_WR,
    S_CMD_WR_RSP,
    S_DONE,
    S_ERROR
  } bar_state_e;

  bar_state_e  state_r;

  logic [5:0]  cand_r;              // candidate register, CFG_REG_BAR_FIRST..LAST
  logic [2:0]  slot_r;              // next BAR slot to fill
  logic [64:0] cursor_r;            // allocator cursor, 65-bit to catch overflow
  logic [31:0] enc_lo_r;            // masked lower half of a 64-bit pair
  logic        is64_r;              // the BAR being assigned right now is 64-bit
  logic        prefetch_r;
  logic [63:0] size_r;
  logic [63:0] addr_r;
  enum_error_e error_code_r;
  logic        credit_blocked_r;

  logic [BAR_SLOTS-1:0] io_bar_mask_r;

  logic        slot_valid_r    [BAR_SLOTS];
  logic        slot_is64_r     [BAR_SLOTS];
  logic        slot_prefetch_r [BAR_SLOTS];
  logic [63:0] slot_size_r     [BAR_SLOTS];
  logic [63:0] slot_addr_r     [BAR_SLOTS];

  // -------------------------------------------------------------------------
  // Readback decode
  // -------------------------------------------------------------------------
  // Combinational from the response port (PCI Local Bus Spec r3.0,
  // §6.2.5.1). The all-ones write must come first: an implemented 32-bit
  // non-prefetchable memory BAR whose address bits hold 0 reads 00000000h,
  // like an unimplemented register, which is hardwired to zero. Only the
  // readback after the write tells them apart.
  //
  // rb_zero tests the whole register, not the masked size bits. After the
  // write, the lower half of a 64-bit BAR of 4 GB or more reads 00000004h or
  // 0000000Ch, with every size bit 0; a masked test would call it
  // unimplemented.
  wire        rb_is_io     = rsp_rdata_i[BAR_BIT_IO];
  wire [1:0]  rb_type      = rsp_rdata_i[BAR_TYPE_LSB +: 2];
  wire        rb_prefetch  = rsp_rdata_i[BAR_BIT_PREFETCH];
  wire [31:0] rb_enc_mem   = rsp_rdata_i & BAR_MEM_MASK;
  wire        rb_zero      = (rsp_rdata_i == 32'h0);
  wire        rb_type_bad  = (rb_type == BAR_TYPE_RESERVED1) ||
                             (rb_type == BAR_TYPE_RESERVED3);

  // A 32-bit memory BAR's size, from the masked readback.
  wire [63:0] size32 = {32'h0, (~rb_enc_mem) + 32'd1};
  // A 64-bit BAR's size. The mask applies to the lower half only: bits 3:0
  // are the lower register's read-only field, and the upper register is 32
  // address bits.
  wire [63:0] enc64  = {rsp_rdata_i, enc_lo_r};
  wire [63:0] size64 = (~enc64) + 64'd1;

  // -------------------------------------------------------------------------
  // Allocation
  // -------------------------------------------------------------------------
  // Combinational from cursor_r and the size being placed. The address is
  // cursor_r rounded up to a multiple of the size, because every BAR is a
  // power of two in size and naturally aligned (PCI Local Bus Spec r3.0,
  // §6.2.5.1). The sums are 65 bits wide, so a round-up or an end address
  // past 2^64 fails window_bad instead of wrapping to a plausible low
  // address, as long as MEM_BAR_BASE + MEM_BAR_WINDOW is at most 2^64. With
  // a larger sum a 64-bit BAR can pass window_bad past 2^64, and its address
  // is written truncated to 64 bits, which also wraps cursor_r.
  //
  // The BAR being placed is a pair exactly when the upper half has just been
  // read back; every other allocation is of a 32-bit BAR.
  wire        pair_now     = (state_r == S_UP_RD_RSP);
  wire [63:0] alloc_size   = pair_now ? size64 : size32;

  wire [64:0] alloc_size65 = {1'b0, alloc_size};
  wire [64:0] align_mask65 = alloc_size65 - 65'd1;
  wire [64:0] alloc_addr65 = (cursor_r + align_mask65) & ~align_mask65;
  wire [64:0] alloc_end65  = alloc_addr65 + alloc_size65;

  // A decoded size is legal only if it is at least BAR_MEM_MIN_BYTES and a
  // power of two; see ENUM_ERR_BAR_SIZE in pcie_enum_pkg.
  wire        size_legal   = (alloc_size >= BAR_MEM_MIN_BYTES) &&
                             ((alloc_size & (alloc_size - 64'd1)) == 64'd0);
  wire        window_bad   = (alloc_end65 > MEM_BAR_LIMIT);
  // Keyed on pair_now rather than on is64_r, because on the 32-bit path is64_r
  // has not been written yet when the check is evaluated.
  wire        addr32_bad   = !pair_now && (alloc_end65 > ADDR32_LIMIT);

  // -------------------------------------------------------------------------
  // Outcome classification
  // -------------------------------------------------------------------------
  // One policy, unlike pcie_enum_scan's two: every transaction here goes to a
  // device that has answered its probe and its Header Type read, so every
  // outcome other than TXN_OK is a fault. All seven response states use this
  // function, so they cannot drift apart. credit_blocked is tx_fc_blocked_i
  // in the cycle the outcome is reported, the same sample
  // err_credit_blocked_o records.
  function automatic enum_error_e fault_code(input txn_outcome_e outcome,
                                             input logic         credit_blocked);
    case (outcome)
      TXN_CA:            fault_code = ENUM_ERR_CA;
      TXN_CRS_EXHAUSTED: fault_code = ENUM_ERR_CRS_EXHAUSTED;
      // A timeout while the credit gate holds the request is credit
      // starvation, not an unresponsive device.
      TXN_TIMEOUT:       fault_code = credit_blocked ? ENUM_ERR_CREDIT_STARVED
                                                    : ENUM_ERR_TIMEOUT;
      // TXN_UR. A device that answered its probe has no reason to reject a
      // legal configuration access.
      default:           fault_code = ENUM_ERR_UR_POST_PROBE;
    endcase
  endfunction

  // -------------------------------------------------------------------------
  // Sequencer
  // -------------------------------------------------------------------------
  // Walks cand_r from CFG_REG_BAR_FIRST to CFG_REG_BAR_LAST. The upper
  // register of a 64-bit BAR is written and read only after the lower one has
  // decoded as 64-bit, so no all-ones write goes past CFG_REG_BAR_LAST.
  // That order costs nothing: pcie_enum_top has one pcie_cfg_txn, so the four
  // transactions are serial in any order.
  //
  // Every sizing and assignment write precedes the Command write, which is
  // the last transaction. Until then Memory Space Enable is 0, so the two
  // halves of a 64-bit BAR may be written in either order; lower then upper
  // follows the probe order.

  // The next candidate after one register, and after the two registers of a
  // 64-bit BAR.
  wire [6:0] cand_after_one  = {1'b0, cand_r} + 7'd1;
  wire [6:0] cand_after_pair = {1'b0, cand_r} + 7'd2;

  always_ff @(posedge clk_i) begin
    if (rst_i) begin
      state_r          <= S_IDLE;
      cand_r           <= CFG_REG_BAR_FIRST;
      slot_r           <= 3'd0;
      cursor_r         <= {1'b0, MEM_BAR_BASE};
      enc_lo_r         <= 32'h0;
      is64_r           <= 1'b0;
      prefetch_r       <= 1'b0;
      size_r           <= 64'h0;
      addr_r           <= 64'h0;
      error_code_r     <= ENUM_ERR_NONE;
      credit_blocked_r <= 1'b0;
      io_bar_mask_r    <= '0;
      for (int i = 0; i < BAR_SLOTS; i++) begin
        slot_valid_r[i]    <= 1'b0;
        slot_is64_r[i]     <= 1'b0;
        slot_prefetch_r[i] <= 1'b0;
        slot_size_r[i]     <= 64'h0;
        slot_addr_r[i]     <= 64'h0;
      end
    end else begin
      unique case (state_r)

        S_IDLE: begin
          if (bar_start_i) begin
            cand_r           <= CFG_REG_BAR_FIRST;
            slot_r           <= 3'd0;
            cursor_r         <= {1'b0, MEM_BAR_BASE};
            error_code_r     <= ENUM_ERR_NONE;
            credit_blocked_r <= 1'b0;
            io_bar_mask_r    <= '0;
            state_r          <= S_CHECK;
          end
        end

        // An absent device, or one whose header is not Type 0, has nothing for
        // this module to configure: S_DONE, not an error, and no transaction
        // at all, not even the Command write.
        S_CHECK: begin
          state_r <= (device_present_i && !unsupported_device_i) ? S_SIZE_WR
                                                                 : S_DONE;
        end

        // ---- size: all-ones write ------------------------------------------
        S_SIZE_WR: if (cmd_ready_i) state_r <= S_SIZE_WR_RSP;

        S_SIZE_WR_RSP: begin
          if (rsp_valid_i) begin
            if (rsp_outcome_i == TXN_OK) begin
              state_r <= S_SIZE_RD;
            end else begin
              error_code_r     <= fault_code(rsp_outcome_i, tx_fc_blocked_i);
              credit_blocked_r <= (rsp_outcome_i == TXN_TIMEOUT) && tx_fc_blocked_i;
              state_r          <= S_ERROR;
            end
          end
        end

        // ---- size: readback and decode --------------------------------------
        S_SIZE_RD: if (cmd_ready_i) state_r <= S_SIZE_RD_RSP;

        S_SIZE_RD_RSP: begin
          if (rsp_valid_i) begin
            if (rsp_outcome_i != TXN_OK) begin
              error_code_r     <= fault_code(rsp_outcome_i, tx_fc_blocked_i);
              credit_blocked_r <= (rsp_outcome_i == TXN_TIMEOUT) && tx_fc_blocked_i;
              state_r          <= S_ERROR;
            end else if (rb_is_io) begin
              // Tested before the Type field, because an I/O BAR's bits 2:1
              // are an address bit and a reserved bit, not a Type (PCI Local
              // Bus Spec r3.0, §6.2.5.1). Skip and log: no I/O BAR is
              // assigned, and I/O Space Enable stays 0, so the device never
              // decodes I/O space.
              io_bar_mask_r[cand_r - CFG_REG_BAR_FIRST] <= 1'b1;
              if (cand_after_one > {1'b0, CFG_REG_BAR_LAST}) begin
                state_r <= S_CMD_WR;
              end else begin
                cand_r  <= cand_after_one[5:0];
                state_r <= S_SIZE_WR;
              end
            end else if (rb_zero) begin
              // Unimplemented: hardwired to zero, consumes no address space.
              if (cand_after_one > {1'b0, CFG_REG_BAR_LAST}) begin
                state_r <= S_CMD_WR;
              end else begin
                cand_r  <= cand_after_one[5:0];
                state_r <= S_SIZE_WR;
              end
            end else if (rb_type_bad) begin
              error_code_r <= ENUM_ERR_BAR_TYPE;
              state_r      <= S_ERROR;
            end else if (rb_type == BAR_TYPE_64BIT) begin
              // BAR5 has no register after it to pair with: offset 28h is the
              // Cardbus CIS Pointer (PCIe Base Spec r2.1, §7.5.2). Following
              // the Type would write all-ones to register 10, past
              // CFG_REG_BAR_LAST.
              if (cand_r == CFG_REG_BAR_LAST) begin
                error_code_r <= ENUM_ERR_BAR_TYPE;
                state_r      <= S_ERROR;
              end else begin
                enc_lo_r   <= rb_enc_mem;
                prefetch_r <= rb_prefetch;
                is64_r     <= 1'b1;
                state_r    <= S_UP_WR;
              end
            end else begin
              // 32-bit memory BAR: size it and place it now.
              is64_r     <= 1'b0;
              prefetch_r <= rb_prefetch;
              if (!size_legal) begin
                error_code_r <= ENUM_ERR_BAR_SIZE;
                state_r      <= S_ERROR;
              end else if (window_bad) begin
                error_code_r <= ENUM_ERR_BAR_WINDOW;
                state_r      <= S_ERROR;
              end else if (addr32_bad) begin
                error_code_r <= ENUM_ERR_BAR_ADDR32;
                state_r      <= S_ERROR;
              end else begin
                size_r  <= alloc_size;
                addr_r  <= alloc_addr65[63:0];
                state_r <= S_ASSIGN_LO;
              end
            end
          end
        end

        // ---- size: the upper half of a 64-bit pair ---------------------------
        S_UP_WR: if (cmd_ready_i) state_r <= S_UP_WR_RSP;

        S_UP_WR_RSP: begin
          if (rsp_valid_i) begin
            if (rsp_outcome_i == TXN_OK) begin
              state_r <= S_UP_RD;
            end else begin
              error_code_r     <= fault_code(rsp_outcome_i, tx_fc_blocked_i);
              credit_blocked_r <= (rsp_outcome_i == TXN_TIMEOUT) && tx_fc_blocked_i;
              state_r          <= S_ERROR;
            end
          end
        end

        S_UP_RD: if (cmd_ready_i) state_r <= S_UP_RD_RSP;

        S_UP_RD_RSP: begin
          if (rsp_valid_i) begin
            if (rsp_outcome_i != TXN_OK) begin
              error_code_r     <= fault_code(rsp_outcome_i, tx_fc_blocked_i);
              credit_blocked_r <= (rsp_outcome_i == TXN_TIMEOUT) && tx_fc_blocked_i;
              state_r          <= S_ERROR;
            end else if (!size_legal) begin
              error_code_r <= ENUM_ERR_BAR_SIZE;
              state_r      <= S_ERROR;
            end else if (window_bad) begin
              error_code_r <= ENUM_ERR_BAR_WINDOW;
              state_r      <= S_ERROR;
            end else begin
              // addr32_bad cannot apply: this BAR is 64 bits wide by decode.
              size_r  <= alloc_size;
              addr_r  <= alloc_addr65[63:0];
              state_r <= S_ASSIGN_LO;
            end
          end
        end

        // ---- assign ----------------------------------------------------------
        S_ASSIGN_LO: if (cmd_ready_i) state_r <= S_ASSIGN_LO_RSP;

        S_ASSIGN_LO_RSP: begin
          if (rsp_valid_i) begin
            if (rsp_outcome_i != TXN_OK) begin
              error_code_r     <= fault_code(rsp_outcome_i, tx_fc_blocked_i);
              credit_blocked_r <= (rsp_outcome_i == TXN_TIMEOUT) && tx_fc_blocked_i;
              state_r          <= S_ERROR;
            end else if (is64_r) begin
              state_r <= S_ASSIGN_UP;
            end else begin
              // The slot is committed only after the device has accepted the
              // address write, so bar_valid_o means programmed, not merely
              // decoded.
              slot_valid_r[slot_r]    <= 1'b1;
              slot_is64_r[slot_r]     <= 1'b0;
              slot_prefetch_r[slot_r] <= prefetch_r;
              slot_size_r[slot_r]     <= size_r;
              slot_addr_r[slot_r]     <= addr_r;
              slot_r                  <= slot_r + 3'd1;
              cursor_r                <= {1'b0, addr_r} + {1'b0, size_r};
              if (cand_after_one > {1'b0, CFG_REG_BAR_LAST}) begin
                state_r <= S_CMD_WR;
              end else begin
                cand_r  <= cand_after_one[5:0];
                state_r <= S_SIZE_WR;
              end
            end
          end
        end

        S_ASSIGN_UP: if (cmd_ready_i) state_r <= S_ASSIGN_UP_RSP;

        S_ASSIGN_UP_RSP: begin
          if (rsp_valid_i) begin
            if (rsp_outcome_i != TXN_OK) begin
              error_code_r     <= fault_code(rsp_outcome_i, tx_fc_blocked_i);
              credit_blocked_r <= (rsp_outcome_i == TXN_TIMEOUT) && tx_fc_blocked_i;
              state_r          <= S_ERROR;
            end else begin
              slot_valid_r[slot_r]    <= 1'b1;
              slot_is64_r[slot_r]     <= 1'b1;
              slot_prefetch_r[slot_r] <= prefetch_r;
              slot_size_r[slot_r]     <= size_r;
              slot_addr_r[slot_r]     <= addr_r;
              slot_r                  <= slot_r + 3'd1;
              cursor_r                <= {1'b0, addr_r} + {1'b0, size_r};
              // A 64-bit BAR occupies two registers.
              if (cand_after_pair > {1'b0, CFG_REG_BAR_LAST}) begin
                state_r <= S_CMD_WR;
              end else begin
                cand_r  <= cand_after_pair[5:0];
                state_r <= S_SIZE_WR;
              end
            end
          end
        end

        // ---- enable ----------------------------------------------------------
        // Entered only from the candidate-advance arms above, once the next
        // candidate is past CFG_REG_BAR_LAST, so the Command write follows
        // every sizing and assignment write.
        S_CMD_WR: if (cmd_ready_i) state_r <= S_CMD_WR_RSP;

        S_CMD_WR_RSP: begin
          if (rsp_valid_i) begin
            if (rsp_outcome_i == TXN_OK) begin
              state_r <= S_DONE;
            end else begin
              error_code_r     <= fault_code(rsp_outcome_i, tx_fc_blocked_i);
              credit_blocked_r <= (rsp_outcome_i == TXN_TIMEOUT) && tx_fc_blocked_i;
              state_r          <= S_ERROR;
            end
          end
        end

        // Terminal states hold until reset. Enumeration runs once after
        // link-up, and a status that could change during a rerun would let a
        // consumer sample it mid-sequence.
        S_DONE:  state_r <= S_DONE;
        S_ERROR: state_r <= S_ERROR;
      endcase
    end
  end

  // -------------------------------------------------------------------------
  // Command port
  // -------------------------------------------------------------------------
  // Every field is driven here, constants included, rather than tied off in
  // pcie_enum_top, so the handoff mux there selects a complete command port
  // from each stage.
  assign cmd_valid_o = (state_r == S_SIZE_WR)   || (state_r == S_SIZE_RD)   ||
                       (state_r == S_UP_WR)     || (state_r == S_UP_RD)     ||
                       (state_r == S_ASSIGN_LO) || (state_r == S_ASSIGN_UP) ||
                       (state_r == S_CMD_WR);

  assign cmd_write_o = (state_r == S_SIZE_WR)   || (state_r == S_UP_WR)     ||
                       (state_r == S_ASSIGN_LO) || (state_r == S_ASSIGN_UP) ||
                       (state_r == S_CMD_WR);

  // The upper half of a 64-bit BAR is always cand_r + 1; everything else
  // addresses cand_r, except the Command write, which addresses register 1.
  assign cmd_reg_num_o =
      (state_r == S_CMD_WR)                          ? CFG_REG_COMMAND_STATUS :
      ((state_r == S_UP_WR) || (state_r == S_UP_RD) ||
       (state_r == S_ASSIGN_UP))                     ? cand_after_one[5:0]    :
                                                       cand_r;

  assign cmd_ext_reg_o = CFG_EXT_REG_NONE;

  // 0011b for the Command write, 1111b for every other access. The Command
  // write enables only bytes 0-1, the Command register; bytes 2-3 are the
  // Status register, whose RW1C bits it does not write (PCIe Base Spec r2.1,
  // §7.5.1.2). The scan stages drive 1111b in every state, so a handoff mux
  // in pcie_enum_top that merged stages instead of selecting one would show
  // here, as a Command write with 1111b.
  assign cmd_first_be_o = (state_r == S_CMD_WR) ? CFG_BE_LOWER_HALF : CFG_BE_DWORD;

  assign cmd_wdata_o =
      (state_r == S_CMD_WR)    ? CMD_ENABLE_VALUE   :
      (state_r == S_ASSIGN_LO) ? addr_r[31:0]       :
      (state_r == S_ASSIGN_UP) ? addr_r[63:32]      :
                                 BAR_PROBE_ALL_ONES;

  // A handshake, not a strobe: pcie_cfg_txn holds rsp_valid_o until it is
  // consumed, so an outcome cannot be missed.
  assign rsp_ready_o = (state_r == S_SIZE_WR_RSP)   || (state_r == S_SIZE_RD_RSP)   ||
                       (state_r == S_UP_WR_RSP)     || (state_r == S_UP_RD_RSP)     ||
                       (state_r == S_ASSIGN_LO_RSP) || (state_r == S_ASSIGN_UP_RSP) ||
                       (state_r == S_CMD_WR_RSP);

  // -------------------------------------------------------------------------
  // Status
  // -------------------------------------------------------------------------
  // The slot registers are flattened into the bar_*_o vectors: slot i is bit
  // i, or bits [i*64 +: 64]. A slot is valid only once its address write has
  // completed.
  assign bar_busy_o           = (state_r != S_IDLE) && (state_r != S_DONE) &&
                                (state_r != S_ERROR);
  assign enum_done_o          = (state_r == S_DONE);
  assign bar_error_o          = (state_r == S_ERROR);
  assign bar_error_code_o     = error_code_r;
  assign err_credit_blocked_o = credit_blocked_r;
  assign bar_count_o          = {1'b0, slot_r};
  assign io_bar_mask_o        = io_bar_mask_r;

  always_comb begin
    bar_valid_o    = '0;
    bar_is_64_o    = '0;
    bar_prefetch_o = '0;
    bar_size_o     = '0;
    bar_addr_o     = '0;
    for (int i = 0; i < BAR_SLOTS; i++) begin
      bar_valid_o[i]         = slot_valid_r[i];
      bar_is_64_o[i]         = slot_is64_r[i];
      bar_prefetch_o[i]      = slot_prefetch_r[i];
      bar_size_o[i*64 +: 64] = slot_size_r[i];
      bar_addr_o[i*64 +: 64] = slot_addr_r[i];
    end
  end

  // -------------------------------------------------------------------------
  // Elaboration checks
  // -------------------------------------------------------------------------
  // They warn and let elaboration continue. A MEM_BAR_BASE that is not
  // 128-byte aligned still works: the first BAR is rounded up to its own
  // alignment.
  initial begin
    if (MEM_BAR_WINDOW == 64'd0)
      $warning("pcie_enum_bar: MEM_BAR_WINDOW is 0 -- every memory BAR will fault with ENUM_ERR_BAR_WINDOW.");
    if ((MEM_BAR_BASE & (BAR_MEM_MIN_BYTES - 64'd1)) != 64'd0)
      $warning("pcie_enum_bar: MEM_BAR_BASE %0h is not 128-byte aligned; the first BAR lands above it via the alignment round-up.", MEM_BAR_BASE);
  end

endmodule
