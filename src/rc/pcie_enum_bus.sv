// ---------------------------------------------------------------------------
// pcie_enum_bus -- bus-number assignment for one PCI-to-PCI bridge level
//
// Author: Kourosh Ghahramani
// Silicon Systems Research Lab, University of Washington
//
// Purpose
//   Runs once, after the first-level scan (pcie_enum_scan) ends without error
//   (scan_done_o), when pcie_enum_top's bridge_enable_i is set. When the scan
//   found a device with a Type 1 header, this module issues one Type 0
//   Configuration Write to the bridge's bus-number register (offset 18h) and,
//   once that write completes successfully, hands off to the second-level
//   scan. Any other verdict bypasses this stage without a transaction. The
//   transaction itself, including CRS retries, is carried out by the shared
//   pcie_cfg_txn instance in pcie_enum_top.
//
// Interfaces
//   Control       bus_start_i: sampled in S_IDLE only.
//   Scan verdict  device_present_i, unsupported_device_i, header_type_i,
//                 bridge_bus_i: the first scan's verdict and bus number.
//   Status        bus_busy_o, bus_bypassed_o, bus_error_o, bus_error_code_o,
//                 err_credit_blocked_o: bypass and error are terminal.
//   Handoff       bus_done_o, sec_bus_o, bus_type1_o: decoded from S_DONE.
//   Annotation    tx_fc_blocked_i: qualifies a timeout report only.
//   Transaction   cmd_*, rsp_*: the command and response ports of pcie_cfg_txn.
//
// Clock and reset
//   clk_i only. rst_i is synchronous and active high, and is the only way out
//   of S_DONE, S_BYPASS and S_ERROR.
//
// Limitations
//   One bridge level: one write, one secondary bus, one downstream scan and
//   BAR pair; a deeper hierarchy is not supported.
//
// References
//   PCI Local Bus Spec r3.0, §3.2.2.3
//   PCIe Base Spec r2.1, §7.3.3
//   PCIe Base Spec r2.1, §7.5.3.2
//   PCIe Base Spec r2.1, §7.5.3.3
// ---------------------------------------------------------------------------
`timescale 1ns/1ps
// No parameters: everything this module needs is fixed policy in pcie_enum_pkg.
module pcie_enum_bus
  import pcie_rq_rc_pkg::*;
  import pcie_enum_pkg::*;
(
    input  logic                        clk_i,
    input  logic                        rst_i,

    // ---- control -----------------------------------------------------------
    // pcie_enum_top raises this when the first scan ends without error
    // (scan_done_o) and bridge_enable_i is set. Whether there is anything to
    // do is decided here from the verdict inputs, so a Type 0 device passes
    // through without a transaction or a handoff.
    input  logic                        bus_start_i,
    // Stable whenever bus_start_i can rise: the scan's terminal states hold
    // until reset.
    input  logic                        device_present_i,
    input  logic                        unsupported_device_i,
    input  logic [7:0]                  header_type_i,
    // The bus the first scan probed, written as the Primary Bus Number. PCI
    // Express Functions do not use that register, but it is read-write and is
    // written with the correct value (PCIe Base Spec r2.1, §7.5.3.2).
    input  logic [7:0]                  bridge_bus_i,

    // ---- status surface ----------------------------------------------------
    output logic                        bus_busy_o,
    // The handoff. pcie_enum_top starts the second scan on this level, so no
    // Type 1 request can precede the completion of the bus-number write.
    output logic                        bus_done_o,
    // No device, or a header layout other than Type 1 (01h): no transaction
    // was issued.
    output logic                        bus_bypassed_o,
    output logic                        bus_error_o,
    output enum_error_e                 bus_error_code_o,
    // Diagnostic only, valid with bus_error_o on a timeout.
    output logic                        err_credit_blocked_o,

    // Both are 0 until S_DONE, so a consumer cannot probe a bus the bridge
    // does not route yet.
    output logic [7:0]                  sec_bus_o,
    output logic                        bus_type1_o,

    // ---- annotation input, not control flow --------------------------------
    input  logic                        tx_fc_blocked_i,

    // ---- pcie_cfg_txn command port -----------------------------------------
    // There is no BDF output: while this module owns the port, the BDF mux in
    // pcie_enum_top selects the first scan's device_bdf_o, which is the bridge.
    output logic                        cmd_valid_o,
    input  logic                        cmd_ready_i,
    output logic                        cmd_write_o,
    output logic                        cmd_type1_o,
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

  typedef enum logic [2:0] {
    S_IDLE,     // waiting for bus_start_i
    S_WR_CMD,   // offering the one CfgWr0 to register 6 (18h)
    S_WR_RSP,   // classifying its outcome
    S_DONE,     // terminal: bridge configured, handoff asserted
    S_BYPASS,   // terminal: nothing here for this stage
    S_ERROR     // terminal, sticky, reset-only
  } bus_state_e;

  bus_state_e  state_r;
  enum_error_e error_code_r;
  logic        credit_blocked_r;

  // Bit 7 of the header type is the multi-function bit, not part of the layout
  // code, and is masked as pcie_enum_scan masks it; a multi-function bridge is
  // still a bridge. unsupported_device_i covers every layout other than Type 0,
  // so the layout is checked here as well.
  wire hdr_is_type1 = (header_type_i[6:0] == HDR_LAYOUT_TYPE1);
  wire eligible     = device_present_i && unsupported_device_i && hdr_is_type1;

  // Maps a failed outcome to its error code, as pcie_enum_scan's fault_code
  // does after its probe phase. credit_blocked is tx_fc_blocked_i in the cycle
  // the outcome is reported, the same sample err_credit_blocked_o records.
  function automatic enum_error_e fault_code(input txn_outcome_e outcome,
                                             input logic         credit_blocked);
    case (outcome)
      TXN_CA:            fault_code = ENUM_ERR_CA;
      TXN_CRS_EXHAUSTED: fault_code = ENUM_ERR_CRS_EXHAUSTED;
      // A timeout while the credit gate holds the request is credit
      // starvation, not an unresponsive bridge.
      TXN_TIMEOUT:       fault_code = credit_blocked ? ENUM_ERR_CREDIT_STARVED
                                                    : ENUM_ERR_TIMEOUT;
      default:           fault_code = ENUM_ERR_UR_POST_PROBE;
    endcase
  endfunction

  always_ff @(posedge clk_i) begin
    if (rst_i) begin
      state_r          <= S_IDLE;
      error_code_r     <= ENUM_ERR_NONE;
      credit_blocked_r <= 1'b0;
    end else begin
      unique case (state_r)
        S_IDLE: begin
          if (bus_start_i) begin
            error_code_r     <= ENUM_ERR_NONE;
            credit_blocked_r <= 1'b0;
            state_r          <= eligible ? S_WR_CMD : S_BYPASS;
          end
        end

        S_WR_CMD: if (cmd_ready_i) state_r <= S_WR_RSP;

        // Every outcome is final. CRS never arrives here, because pcie_cfg_txn
        // retries it, and there is no timer here: a missing completion arrives
        // as TXN_TIMEOUT from the completion timeout in tlp_request_tracker.
        S_WR_RSP: begin
          if (rsp_valid_i) begin
            case (rsp_outcome_i)
              TXN_OK: state_r <= S_DONE;
              // The bridge has already answered the scan's two reads, so it is
              // present, and a UR to a legal write of its own register 6 is a
              // fault, not an absent device.
              TXN_UR: begin
                error_code_r <= ENUM_ERR_UR_POST_PROBE;
                state_r      <= S_ERROR;
              end
              default: begin
                error_code_r     <= fault_code(rsp_outcome_i, tx_fc_blocked_i);
                credit_blocked_r <= (rsp_outcome_i == TXN_TIMEOUT) && tx_fc_blocked_i;
                state_r          <= S_ERROR;
              end
            endcase
          end
        end

        // Terminal states hold until reset. Enumeration runs once after link-up,
        // and a status that could change during a rescan would let a consumer
        // sample it mid-sequence.
        S_DONE:   state_r <= S_DONE;
        S_BYPASS: state_r <= S_BYPASS;
        S_ERROR:  state_r <= S_ERROR;
      endcase
    end
  end

  // -------------------------------------------------------------------------
  // The bus-number write
  // -------------------------------------------------------------------------
  // One whole-Dword write (first_be 1111b): Primary, Secondary and Subordinate
  // Bus Number and the Secondary Latency Timer share register 6, so no field
  // needs a read-modify-write.
  assign cmd_valid_o    = (state_r == S_WR_CMD);
  assign cmd_write_o    = 1'b1;
  // Type 0: the write targets the bridge itself, on the bus directly behind
  // the port, and Type 1 is required only for a target on another bus (PCI
  // Local Bus Spec r3.0, §3.2.2.3). A Type 1 request would instead be routed
  // by the bus-number range that this write has not yet set (PCIe Base Spec
  // r2.1, §7.3.3).
  assign cmd_type1_o    = 1'b0;
  assign cmd_reg_num_o  = CFG_REG_BUS_NUMBER;
  assign cmd_ext_reg_o  = CFG_EXT_REG_NONE;
  assign cmd_first_be_o = CFG_BE_DWORD;
  // The Secondary Latency Timer byte is 00h because the register is read-only
  // 00h on PCI Express (PCIe Base Spec r2.1, §7.5.3.3).
  assign cmd_wdata_o    = {SEC_LATENCY_TIMER_WDATA, SUB_BUS_NUMBER,
                           SEC_BUS_NUMBER, bridge_bus_i};

  assign rsp_ready_o    = (state_r == S_WR_RSP);

  // -------------------------------------------------------------------------
  // Status
  // -------------------------------------------------------------------------
  assign bus_busy_o           = (state_r == S_WR_CMD) || (state_r == S_WR_RSP);
  // A bridge forwards a Type 1 request only for a bus inside the range its
  // Secondary and Subordinate Bus Numbers assign, and answers UR otherwise
  // (PCIe Base Spec r2.1, §7.3.3). This write sets that range, so the handoff
  // is decoded from S_DONE, entered only when the write's completion is
  // TXN_OK. One write is enough because SUB_BUS_NUMBER is fixed in advance.
  assign bus_done_o           = (state_r == S_DONE);
  assign bus_bypassed_o       = (state_r == S_BYPASS);
  assign bus_error_o          = (state_r == S_ERROR);
  assign bus_error_code_o     = error_code_r;
  assign err_credit_blocked_o = credit_blocked_r;

  assign sec_bus_o            = bus_done_o ? SEC_BUS_NUMBER : 8'h00;
  assign bus_type1_o          = bus_done_o;

endmodule
