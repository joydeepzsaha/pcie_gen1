// ---------------------------------------------------------------------------
// pcie_enum_scan -- presence scan of one bus: Vendor ID probe and Header Type
//
// Author: Kourosh Ghahramani
// Silicon Systems Research Lab, University of Washington
//
// Purpose
//   Reads register 0 (Vendor ID, Device ID) of Device 0, Function 0 on
//   scan_bus_i and, if a device answers, register 3 for its Header Type. It
//   reports whether a device is present and whether its header has the Type
//   0 layout that pcie_enum_bar can configure. Both reads go through the
//   pcie_cfg_txn instance in pcie_enum_top, which instantiates this module
//   twice: for the first bus, and for the bus behind a bridge.
//
// Interfaces
//   Control       scan_start_i: sampled in S_IDLE only. scan_bus_i: the bus.
//   Status        scan_busy_o, scan_done_o, scan_error_o, scan_error_code_o,
//                 err_credit_blocked_o: done and error are terminal.
//   Verdict       device_present_o, unsupported_device_o, device_bdf_o,
//                 vendor_id_o, device_id_o, header_type_o, multifunction_o:
//                 held from scan_done_o until reset; device_bdf_o follows
//                 scan_bus_i.
//   Annotation    tx_fc_blocked_i: qualifies a timeout report only.
//   Transaction   cmd_*, rsp_*: the command and response ports of pcie_cfg_txn.
//
// Clock and reset
//   clk_i only. rst_i is synchronous and active high, and is the only way out
//   of S_DONE, S_UNSUPPORTED and S_ERROR.
//
// Limitations
//   Device 0, Function 0 only: no device-number loop (see DEVICES_TO_SCAN in
//   pcie_enum_pkg), and the other Functions of a multi-function device are
//   not probed.
//
// References
//   PCIe Base Spec r2.1, §2.3.2
//   PCIe Base Spec r2.1, §7.3.1
//   PCIe Base Spec r2.1, §7.3.3
//   PCIe Base Spec r2.1, §7.5.1
// ---------------------------------------------------------------------------
`timescale 1ns/1ps
// No parameters: the AXIS widths, the CRS policy and CPL_TIMEOUT_CYCLES are
// parameters of pcie_cfg_txn, which pcie_enum_top sets.
module pcie_enum_scan
  import pcie_rq_rc_pkg::*;
  import pcie_enum_pkg::*;
(
    input  logic                        clk_i,
    input  logic                        rst_i,

    // ---- control -----------------------------------------------------------
    // Sampled in S_IDLE only. The scan runs once per reset: its terminal
    // states hold until reset.
    input  logic                        scan_start_i,
    // The bus to probe. Device and Function are always 0.
    input  logic [7:0]                  scan_bus_i,

    // ---- status surface ----------------------------------------------------
    // scan_done_o marks a terminal outcome without error: a device found,
    // nothing to enumerate, or a device whose header is not Type 0
    // (unsupported_device_o). A consumer that wants a configurable Type 0
    // device tests
    //     scan_done_o && device_present_o && !unsupported_device_o
    output logic                        scan_busy_o,
    output logic                        scan_done_o,
    output logic                        scan_error_o,
    output enum_error_e                 scan_error_code_o,
    // Diagnostic only, valid with scan_error_o on a timeout: tx_fc_blocked_i
    // was high when the timeout was reported, so the request was probably
    // never transmitted. The same sample selects ENUM_ERR_CREDIT_STARVED.
    output logic                        err_credit_blocked_o,

    output logic                        device_present_o,
    output logic                        unsupported_device_o,
    output logic [15:0]                 device_bdf_o,
    output logic [15:0]                 vendor_id_o,
    output logic [15:0]                 device_id_o,
    output logic [7:0]                  header_type_o,
    output logic                        multifunction_o,

    // ---- annotation input, not control flow --------------------------------
    // Read only to choose a timeout's error code and err_credit_blocked_o; no
    // state transition depends on it.
    input  logic                        tx_fc_blocked_i,

    // ---- pcie_cfg_txn command port -----------------------------------------
    // There is no cmd_bdf_o and no cmd_type1_o: pcie_enum_top drives
    // cmd_bdf_i from this module's device_bdf_o, and cmd_type1_i by bus level.
    // No tag reaches this module either: tlp_request_tracker allocates a tag
    // before the request passes the credit gate, so a tag strobe does not
    // show that a request was transmitted.
    output logic                        cmd_valid_o,
    input  logic                        cmd_ready_i,
    output logic                        cmd_write_o,
    output logic [5:0]                  cmd_reg_num_o,
    output logic [3:0]                  cmd_ext_reg_o,
    output logic [3:0]                  cmd_first_be_o,
    output logic [31:0]                 cmd_wdata_o,

    // ---- pcie_cfg_txn response port ----------------------------------------
    input  logic                        rsp_valid_i,
    output logic                        rsp_ready_o,
    input  txn_outcome_e                rsp_outcome_i,
    input  logic [31:0]                 rsp_rdata_i
);

  // -------------------------------------------------------------------------
  // Command and response wiring
  // -------------------------------------------------------------------------
  // The state machine uses the internal names below; the ports lead to
  // pcie_enum_top's handoff mux and from there to pcie_cfg_txn, which carries
  // one request at a time.
  logic         cmd_valid;
  logic         cmd_ready;
  logic         rsp_valid;
  logic         rsp_ready;
  txn_outcome_e rsp_outcome;
  logic [31:0]  rsp_rdata;

  logic [5:0]   cmd_reg_num;

  assign cmd_valid_o   = cmd_valid;
  assign cmd_ready     = cmd_ready_i;
  assign cmd_reg_num_o = cmd_reg_num;

  assign rsp_valid     = rsp_valid_i;
  assign rsp_ready_o   = rsp_ready;
  assign rsp_outcome   = rsp_outcome_i;
  assign rsp_rdata     = rsp_rdata_i;

  // Driven here rather than tied off in pcie_enum_top, so the handoff mux
  // there selects a complete command port from each stage. Both scan
  // transactions are whole-Dword reads.
  assign cmd_write_o    = 1'b0;
  assign cmd_ext_reg_o  = CFG_EXT_REG_NONE;
  assign cmd_first_be_o = CFG_BE_DWORD;
  assign cmd_wdata_o    = 32'd0;

  // -------------------------------------------------------------------------
  // Scan state machine
  // -------------------------------------------------------------------------
  //   State          Does                   Exit
  //   S_IDLE         waits                  scan_start_i: S_PROBE_CMD
  //   S_PROBE_CMD    offers the read of     cmd_ready: S_PROBE_RSP
  //                  register 0
  //   S_PROBE_RSP    probe policy           OK: S_HDR_CMD; UR (absent): S_DONE;
  //                                         other: S_ERROR
  //   S_HDR_CMD      offers the read of     cmd_ready: S_HDR_RSP
  //                  register 3
  //   S_HDR_RSP      post-probe policy      OK: S_DONE for Type 0, else
  //                                         S_UNSUPPORTED; other: S_ERROR
  //   S_DONE         terminal, no error     reset
  //   S_UNSUPPORTED  terminal, no error     reset
  //   S_ERROR        terminal, error        reset
  // The two reads have separate response states, so the two UR policies sit
  // in separate case arms rather than behind a phase flag. There is no timer
  // here: a missing completion arrives as TXN_TIMEOUT from the completion
  // timeout in tlp_request_tracker.
  typedef enum logic [2:0] {
    S_IDLE,
    S_PROBE_CMD,
    S_PROBE_RSP,
    S_HDR_CMD,
    S_HDR_RSP,
    S_DONE,
    S_UNSUPPORTED,
    S_ERROR
  } scan_state_e;

  scan_state_e state_r;

  logic [15:0] vendor_id_r, device_id_r;
  logic [7:0]  header_type_r;
  logic        present_r;
  enum_error_e error_code_r;
  logic        credit_blocked_r;

  // Header Type is byte 2 of register 3 (PCIe Base Spec r2.1, §7.5.1, Figure
  // 7-4). Bit 7 is the multi-function bit; bits 6:0 give the layout.
  wire [7:0]  hdr_byte    = rsp_rdata[HDR_TYPE_LSB +: 8];
  wire [6:0]  hdr_layout  = hdr_byte[6:0];
  wire        hdr_is_type0 = (hdr_layout == HDR_LAYOUT_TYPE0);

  // Register 0 is {Device ID[31:16], Vendor ID[15:0]}.
  wire [15:0] probe_vendor = rsp_rdata[15:0];
  wire [15:0] probe_device = rsp_rdata[31:16];

  // Maps a failed outcome to its error code. Both response states call it
  // for every outcome but TXN_OK and TXN_UR, which they handle themselves, so
  // the two cannot drift apart. credit_blocked is tx_fc_blocked_i in the
  // cycle the outcome is reported, the same sample err_credit_blocked_o
  // records.
  function automatic enum_error_e fault_code(input txn_outcome_e outcome,
                                             input logic         credit_blocked);
    case (outcome)
      TXN_CA:            fault_code = ENUM_ERR_CA;
      TXN_CRS_EXHAUSTED: fault_code = ENUM_ERR_CRS_EXHAUSTED;
      // A timeout while the credit gate holds the request is credit
      // starvation, not an unresponsive device.
      TXN_TIMEOUT:       fault_code = credit_blocked ? ENUM_ERR_CREDIT_STARVED
                                                    : ENUM_ERR_TIMEOUT;
      default:           fault_code = ENUM_ERR_UR_POST_PROBE;
    endcase
  endfunction

  always_ff @(posedge clk_i) begin
    if (rst_i) begin
      state_r          <= S_IDLE;
      vendor_id_r      <= '0;
      device_id_r      <= '0;
      header_type_r    <= '0;
      present_r        <= 1'b0;
      error_code_r     <= ENUM_ERR_NONE;
      credit_blocked_r <= 1'b0;
    end else begin
      unique case (state_r)
        S_IDLE: begin
          if (scan_start_i) begin
            vendor_id_r      <= '0;
            device_id_r      <= '0;
            header_type_r    <= '0;
            present_r        <= 1'b0;
            error_code_r     <= ENUM_ERR_NONE;
            credit_blocked_r <= 1'b0;
            state_r          <= S_PROBE_CMD;
          end
        end

        S_PROBE_CMD: if (cmd_ready) state_r <= S_PROBE_RSP;

        // ---- probe policy: UR means nothing to enumerate -------------------
        S_PROBE_RSP: begin
          if (rsp_valid) begin
            case (rsp_outcome)
              TXN_OK: begin
                vendor_id_r <= probe_vendor;
                device_id_r <= probe_device;
                present_r   <= 1'b1;
                state_r     <= S_HDR_CMD;
              end
              // Absent, not an error: a UR to the Function 0 probe means no
              // Function 0 Configuration Space answers at this Device Number
              // (PCIe Base Spec r2.1, §7.3.3). Absence is decided from TXN_UR
              // alone; a Successful Completion carrying FFFFh is reported as
              // present with that Vendor ID. The all-1s read value of PCIe Base
              // Spec r2.1, §2.3.2 is one a Root Complex synthesises for
              // software from the UR that this module sees directly.
              TXN_UR: begin
                present_r <= 1'b0;
                state_r   <= S_DONE;
              end
              default: begin
                error_code_r     <= fault_code(rsp_outcome, tx_fc_blocked_i);
                credit_blocked_r <= (rsp_outcome == TXN_TIMEOUT) && tx_fc_blocked_i;
                state_r          <= S_ERROR;
              end
            endcase
          end
        end

        S_HDR_CMD: if (cmd_ready) state_r <= S_HDR_RSP;

        // ---- post-probe policy: UR is a fault ------------------------------
        S_HDR_RSP: begin
          if (rsp_valid) begin
            case (rsp_outcome)
              TXN_OK: begin
                header_type_r <= hdr_byte;
                // Any layout other than Type 0 (a bridge, or a CardBus or
                // reserved layout) belongs to a device that answered
                // correctly but that pcie_enum_bar cannot configure: terminal,
                // not an error. After the first-level scan, pcie_enum_bus
                // configures a Type 1 bridge when bridge_enable_i is set.
                state_r       <= hdr_is_type0 ? S_DONE : S_UNSUPPORTED;
              end
              // A device that answered register 0 has no reason to reject a
              // legal configuration read of register 3.
              TXN_UR: begin
                error_code_r <= ENUM_ERR_UR_POST_PROBE;
                state_r      <= S_ERROR;
              end
              default: begin
                error_code_r     <= fault_code(rsp_outcome, tx_fc_blocked_i);
                credit_blocked_r <= (rsp_outcome == TXN_TIMEOUT) && tx_fc_blocked_i;
                state_r          <= S_ERROR;
              end
            endcase
          end
        end

        // Terminal states hold until reset, S_DONE and S_UNSUPPORTED as well
        // as S_ERROR. Enumeration runs once after link-up, and a status that
        // could change during a rescan would let a consumer sample it
        // mid-sequence.
        S_DONE:        state_r <= S_DONE;
        S_UNSUPPORTED: state_r <= S_UNSUPPORTED;
        S_ERROR:       state_r <= S_ERROR;
      endcase
    end
  end

  assign cmd_valid   = (state_r == S_PROBE_CMD) || (state_r == S_HDR_CMD);
  assign cmd_reg_num = (state_r == S_HDR_CMD) ? CFG_REG_CACHE_HEADER
                                              : CFG_REG_VENDOR_DEVICE;
  // A handshake, not a strobe: pcie_cfg_txn holds rsp_valid_o until it is
  // consumed, so an outcome cannot be missed.
  assign rsp_ready   = (state_r == S_PROBE_RSP) || (state_r == S_HDR_RSP);

  // {Bus[15:8], Device[7:3], Function[2:0]}, with Device and Function fixed at
  // 0; DEVICES_TO_SCAN in pcie_enum_pkg says why there is no device loop.
  assign device_bdf_o = {scan_bus_i, 5'd0, 3'd0};

  assign scan_busy_o          = (state_r != S_IDLE) && (state_r != S_DONE) &&
                                (state_r != S_UNSUPPORTED) && (state_r != S_ERROR);
  assign scan_done_o          = (state_r == S_DONE) || (state_r == S_UNSUPPORTED);
  assign scan_error_o         = (state_r == S_ERROR);
  assign scan_error_code_o    = error_code_r;
  assign err_credit_blocked_o = credit_blocked_r;

  assign device_present_o     = present_r;
  assign unsupported_device_o = (state_r == S_UNSUPPORTED);
  assign vendor_id_o          = vendor_id_r;
  assign device_id_o          = device_id_r;
  assign header_type_o        = header_type_r;
  assign multifunction_o      = header_type_r[HDR_MULTIFUNCTION_BIT];

endmodule
