// ---------------------------------------------------------------------------
// pcie_enum_top -- Root Complex enumeration engine: five stages, one primitive
//
// Author: Kourosh Ghahramani
// Silicon Systems Research Lab, University of Washington
//
// Purpose
//   Holds the single pcie_cfg_txn and the five stages that take turns on it:
//   pcie_enum_scan and pcie_enum_bar for the bus on scan_bus_i and, behind a
//   bridge found there, pcie_enum_bus to set its bus numbers and a second
//   scan and BAR pair for its secondary bus. A static mux gives the command
//   port to one stage at a time, selected by bridge_enable_i and the terminal
//   outputs of the stages before it. Apart from its instances, the module has
//   no state.
//
// Interfaces
//   Control       scan_start_i, scan_bus_i: the first scan's start and bus.
//                 bar_enable_i, bridge_enable_i: levels. The handoff mux also
//                 reads bridge_enable_i, so it must not change during a run.
//   First bus     scan_*_o, device_*, unsupported_device_o, vendor_id_o,
//                 header_type_o, multifunction_o: the first scan's status and
//                 verdict. enum_done_o, bar_*_o, io_bar_mask_o: the
//                 first-level BAR stage's status and results.
//   Errors        enum_error_o, enum_error_code_o: any stage's error.
//                 err_credit_blocked_o: from the stage that timed out.
//   Bridge path   bus_done_o, bus_bypassed_o: pcie_enum_bus wrote the bus
//                 numbers, or had nothing to do. sec_*: the second scan and
//                 BAR stage, on the secondary bus.
//   Annotation    tx_fc_blocked_i: qualifies timeout reports only.
//   Socket        s_axis_rq_*, pcie_rq_tag_*, m_axis_rc_*, cpl_timeout_*:
//                 pcie_rq_rc_top's requester side, straight to pcie_cfg_txn.
//
// Clock and reset
//   clk_i only. rst_i is synchronous and active high; each stage runs once
//   per reset.
//
// Limitations
//   One bridge level, and Device 0, Function 0 on each bus. bar_enable_i
//   gates only the first-level BAR stage; the second-level one starts on
//   bridge_enable_i. With bar_enable_i high on the bridge path, enum_done_o
//   rises without a transaction, before the secondary bus is configured;
//   sec_enum_done_o reports the second level.
//
// Structure
//   Ports
//   Mux wires
//   First bus: presence scan
//   First bus: BAR stage
//   Bridge path: bus numbers, second scan, second BAR stage
//   Handoff mux
//   Combined status
//   Transaction primitive
//
// References
//   PCI Local Bus Spec r3.0, §3.2.2.3
//   PCIe Base Spec r2.1, §7.3.3
// ---------------------------------------------------------------------------
`timescale 1ns/1ps
module pcie_enum_top
  import pcie_rq_rc_pkg::*;
  import pcie_enum_pkg::*;
#(
    // The first six are forwarded to pcie_cfg_txn and documented there.
    parameter int AXIS_DATA_WIDTH = 128,
    parameter int AXIS_KEEP_WIDTH = AXIS_DATA_WIDTH / 32,
    parameter int AXIS_USER_WIDTH = 60,
    parameter int unsigned CRS_RETRY_MAX      = CRS_RETRY_MAX_DEFAULT,
    parameter int unsigned CRS_BACKOFF_CYCLES = CRS_BACKOFF_CYCLES_DEFAULT,
    parameter int unsigned CPL_TIMEOUT_CYCLES = tlp_pkg::CPL_TIMEOUT_DEFAULT_CYCLES,
    // Forwarded verbatim to pcie_enum_bar; documented there.
    parameter logic [63:0] MEM_BAR_BASE       = 64'h0000_0000_8000_0000,
    parameter logic [63:0] MEM_BAR_WINDOW     = 64'h0000_0000_1000_0000
) (
    input  logic                        clk_i,
    input  logic                        rst_i,

    // ---- control -----------------------------------------------------------
    input  logic                        scan_start_i,
    input  logic [7:0]                  scan_bus_i,
    // A level. Low stops the first bus at presence detection: the first-level
    // BAR stage stays in S_IDLE, emits no transaction, and enum_done_o stays
    // low. The presence-scan benches in tb/rc tie it and bridge_enable_i low,
    // so nothing follows the scan's reads.
    input  logic                        bar_enable_i,
    // A level. Low keeps pcie_enum_bus and the second scan and BAR stages in
    // S_IDLE, so the run ends with the first bus whatever the scan found.
    // High, a Type 1 discovery continues into bus-number assignment and the
    // scan and configuration of the device on the secondary bus.
    input  logic                        bridge_enable_i,

    // ---- status surface: presence phase ------------------------------------
    output logic                        scan_busy_o,
    output logic                        scan_done_o,
    output logic                        scan_error_o,
    output enum_error_e                 scan_error_code_o,
    // Diagnostic, from the stage that reported a timeout. At most one stage
    // can be in error (see Combined status), so the OR has at most one live
    // input.
    output logic                        err_credit_blocked_o,

    output logic                        device_present_o,
    output logic                        unsupported_device_o,
    output logic [15:0]                 device_bdf_o,
    output logic [15:0]                 vendor_id_o,
    output logic [15:0]                 device_id_o,
    output logic [7:0]                  header_type_o,
    output logic                        multifunction_o,

    // ---- status surface: BAR phase -----------------------------------------
    output logic                        bar_busy_o,
    output logic                        enum_done_o,
    // Any stage's error. scan_error_o above reports the first scan's alone.
    output logic                        enum_error_o,
    output enum_error_e                 enum_error_code_o,

    output logic [3:0]                  bar_count_o,
    output logic [BAR_SLOTS-1:0]        bar_valid_o,
    output logic [BAR_SLOTS-1:0]        bar_is_64_o,
    output logic [BAR_SLOTS-1:0]        bar_prefetch_o,
    output logic [BAR_SLOTS*64-1:0]     bar_size_o,
    output logic [BAR_SLOTS*64-1:0]     bar_addr_o,
    output logic [BAR_SLOTS-1:0]        io_bar_mask_o,

    // ---- status surface: bridge path, second bus level ---------------------
    // What the bridge path finds, with a sec_ prefix for the secondary bus.
    // Errors from any bridge-path stage fold into enum_error_o and
    // enum_error_code_o.
    output logic                        bus_done_o,
    output logic                        bus_bypassed_o,
    output logic                        sec_scan_done_o,
    output logic                        sec_device_present_o,
    output logic                        sec_unsupported_device_o,
    output logic [15:0]                 sec_device_bdf_o,
    output logic [15:0]                 sec_vendor_id_o,
    output logic [15:0]                 sec_device_id_o,
    output logic [7:0]                  sec_header_type_o,
    output logic                        sec_multifunction_o,
    output logic                        sec_enum_done_o,
    output logic [3:0]                  sec_bar_count_o,
    output logic [BAR_SLOTS-1:0]        sec_bar_valid_o,
    output logic [BAR_SLOTS-1:0]        sec_bar_is_64_o,
    output logic [BAR_SLOTS-1:0]        sec_bar_prefetch_o,
    output logic [BAR_SLOTS*64-1:0]     sec_bar_size_o,
    output logic [BAR_SLOTS*64-1:0]     sec_bar_addr_o,
    output logic [BAR_SLOTS-1:0]        sec_io_bar_mask_o,

    // ---- annotation input, not control flow --------------------------------
    input  logic                        tx_fc_blocked_i,

    // ---- pcie_rq_rc_top socket: Requester Request --------------------------
    output logic [AXIS_DATA_WIDTH-1:0]  s_axis_rq_tdata_o,
    output logic [AXIS_KEEP_WIDTH-1:0]  s_axis_rq_tkeep_o,
    output logic                        s_axis_rq_tvalid_o,
    output logic                        s_axis_rq_tlast_o,
    output logic [AXIS_USER_WIDTH-1:0]  s_axis_rq_tuser_o,
    input  logic                        s_axis_rq_tready_i,

    // ---- pcie_rq_rc_top socket: core-managed tag ---------------------------
    input  logic [7:0]                  pcie_rq_tag_i,
    input  logic                        pcie_rq_tag_vld_i,

    // ---- pcie_rq_rc_top socket: Requester Completion -----------------------
    input  logic [AXIS_DATA_WIDTH-1:0]  m_axis_rc_tdata_i,
    input  logic [AXIS_KEEP_WIDTH-1:0]  m_axis_rc_tkeep_i,
    input  logic                        m_axis_rc_tvalid_i,
    input  logic                        m_axis_rc_tlast_i,
    output logic                        m_axis_rc_tready_o,

    // ---- pcie_rq_rc_top socket: completion timeout sideband ----------------
    input  logic                        cpl_timeout_valid_i,
    input  logic [7:0]                  cpl_timeout_tag_i
);

  // -------------------------------------------------------------------------
  // Mux wires
  // -------------------------------------------------------------------------
  // First the command and response bus between the handoff mux and
  // pcie_cfg_txn, then each stage's own ports, the mux inputs. Every stage
  // shares rsp_outcome and rsp_rdata; only rsp_valid and cmd_ready are gated
  // per stage.
  logic         cmd_valid;
  logic         cmd_ready;
  logic         cmd_write;
  logic [5:0]   cmd_reg_num;
  logic [3:0]   cmd_ext_reg;
  logic [3:0]   cmd_first_be;
  logic [31:0]  cmd_wdata;

  logic         rsp_valid;
  logic         rsp_ready;
  txn_outcome_e rsp_outcome;
  logic [31:0]  rsp_rdata;

  // The first-bus stages' ports.
  logic         scan_cmd_valid,  bar_cmd_valid;
  logic         scan_cmd_ready,  bar_cmd_ready;
  logic         scan_cmd_write,  bar_cmd_write;
  logic [5:0]   scan_cmd_reg_num, bar_cmd_reg_num;
  logic [3:0]   scan_cmd_ext_reg, bar_cmd_ext_reg;
  logic [3:0]   scan_cmd_first_be, bar_cmd_first_be;
  logic [31:0]  scan_cmd_wdata,  bar_cmd_wdata;

  logic         scan_rsp_valid,  bar_rsp_valid;
  logic         scan_rsp_ready,  bar_rsp_ready;

  logic         scan_credit_blocked, bar_credit_blocked;
  logic         bar_error;
  enum_error_e  scan_error_code, bar_error_code;

  // The bridge-path stages' ports.
  logic         bus_cmd_valid,   scan2_cmd_valid,   bar2_cmd_valid;
  logic         bus_cmd_ready,   scan2_cmd_ready,   bar2_cmd_ready;
  logic         bus_cmd_write,   scan2_cmd_write,   bar2_cmd_write;
  logic         bus_cmd_type1;
  logic [5:0]   bus_cmd_reg_num, scan2_cmd_reg_num, bar2_cmd_reg_num;
  logic [3:0]   bus_cmd_ext_reg, scan2_cmd_ext_reg, bar2_cmd_ext_reg;
  logic [3:0]   bus_cmd_first_be, scan2_cmd_first_be, bar2_cmd_first_be;
  logic [31:0]  bus_cmd_wdata,   scan2_cmd_wdata,   bar2_cmd_wdata;

  logic         bus_rsp_valid,   scan2_rsp_valid,   bar2_rsp_valid;
  logic         bus_rsp_ready,   scan2_rsp_ready,   bar2_rsp_ready;

  logic         bus_error,       scan2_error,       bar2_error;
  enum_error_e  bus_error_code,  scan2_error_code,  bar2_error_code;
  logic         bus_credit_blocked, scan2_credit_blocked, bar2_credit_blocked;
  logic [7:0]   bus_sec_bus;
  logic         bus_type1;

  // -------------------------------------------------------------------------
  // First bus: presence scan
  // -------------------------------------------------------------------------
  // Probes Device 0 of scan_bus_i. Its device_bdf_o addresses every
  // first-level transaction, including the bus-number write to a bridge it
  // finds.
  pcie_enum_scan u_scan (
      .clk_i(clk_i),
      .rst_i(rst_i),

      .scan_start_i(scan_start_i),
      .scan_bus_i  (scan_bus_i),

      .scan_busy_o         (scan_busy_o),
      .scan_done_o         (scan_done_o),
      .scan_error_o        (scan_error_o),
      .scan_error_code_o   (scan_error_code),
      .err_credit_blocked_o(scan_credit_blocked),

      .device_present_o    (device_present_o),
      .unsupported_device_o(unsupported_device_o),
      .device_bdf_o        (device_bdf_o),
      .vendor_id_o         (vendor_id_o),
      .device_id_o         (device_id_o),
      .header_type_o       (header_type_o),
      .multifunction_o     (multifunction_o),

      .tx_fc_blocked_i(tx_fc_blocked_i),

      .cmd_valid_o   (scan_cmd_valid),
      .cmd_ready_i   (scan_cmd_ready),
      .cmd_write_o   (scan_cmd_write),
      .cmd_reg_num_o (scan_cmd_reg_num),
      .cmd_ext_reg_o (scan_cmd_ext_reg),
      .cmd_first_be_o(scan_cmd_first_be),
      .cmd_wdata_o   (scan_cmd_wdata),

      .rsp_valid_i  (scan_rsp_valid),
      .rsp_ready_o  (scan_rsp_ready),
      .rsp_outcome_i(rsp_outcome),
      .rsp_rdata_i  (rsp_rdata)
  );

  assign scan_error_code_o = scan_error_code;

  // -------------------------------------------------------------------------
  // First bus: BAR stage
  // -------------------------------------------------------------------------
  // Starts when bar_enable_i is high and the scan is done. Its verdict inputs
  // are stable by then: scan_done_o is terminal, and bar_start_i cannot rise
  // before it. pcie_enum_bar samples the verdict in S_CHECK.
  pcie_enum_bar #(
      .MEM_BAR_BASE  (MEM_BAR_BASE),
      .MEM_BAR_WINDOW(MEM_BAR_WINDOW)
  ) u_bar (
      .clk_i(clk_i),
      .rst_i(rst_i),

      .bar_start_i         (bar_enable_i && scan_done_o),
      .device_present_i    (device_present_o),
      .unsupported_device_i(unsupported_device_o),

      .tx_fc_blocked_i(tx_fc_blocked_i),

      .bar_busy_o          (bar_busy_o),
      .enum_done_o         (enum_done_o),
      .bar_error_o         (bar_error),
      .bar_error_code_o    (bar_error_code),
      .err_credit_blocked_o(bar_credit_blocked),

      .bar_count_o   (bar_count_o),
      .bar_valid_o   (bar_valid_o),
      .bar_is_64_o   (bar_is_64_o),
      .bar_prefetch_o(bar_prefetch_o),
      .bar_size_o    (bar_size_o),
      .bar_addr_o    (bar_addr_o),
      .io_bar_mask_o (io_bar_mask_o),

      .cmd_valid_o   (bar_cmd_valid),
      .cmd_ready_i   (bar_cmd_ready),
      .cmd_write_o   (bar_cmd_write),
      .cmd_reg_num_o (bar_cmd_reg_num),
      .cmd_ext_reg_o (bar_cmd_ext_reg),
      .cmd_first_be_o(bar_cmd_first_be),
      .cmd_wdata_o   (bar_cmd_wdata),

      .rsp_valid_i  (bar_rsp_valid),
      .rsp_ready_o  (bar_rsp_ready),
      .rsp_outcome_i(rsp_outcome),
      .rsp_rdata_i  (rsp_rdata)
  );

  // -------------------------------------------------------------------------
  // Bridge path: bus numbers, second scan, second BAR stage
  // -------------------------------------------------------------------------
  // pcie_enum_bus, then a second scan and BAR pair for the bridge's
  // secondary bus. Each instance runs once per reset and none is re-armed,
  // so there is one bridge level and no recursion. The scan and BAR modules
  // are the ones used on the first bus, with no parameter or port changed;
  // only the wiring differs.
  pcie_enum_bus u_bus (
      .clk_i(clk_i),
      .rst_i(rst_i),

      // Started when the first scan is done and bridge_enable_i is set.
      // Whether there is a bridge to configure is decided inside, from the
      // verdict.
      .bus_start_i         (bridge_enable_i && scan_done_o),
      .device_present_i    (device_present_o),
      .unsupported_device_i(unsupported_device_o),
      .header_type_i       (header_type_o),
      .bridge_bus_i        (scan_bus_i),

      .bus_busy_o          (),
      .bus_done_o          (bus_done_o),
      .bus_bypassed_o      (bus_bypassed_o),
      .bus_error_o         (bus_error),
      .bus_error_code_o    (bus_error_code),
      .err_credit_blocked_o(bus_credit_blocked),
      .sec_bus_o           (bus_sec_bus),
      .bus_type1_o         (bus_type1),

      .tx_fc_blocked_i(tx_fc_blocked_i),

      .cmd_valid_o   (bus_cmd_valid),
      .cmd_ready_i   (bus_cmd_ready),
      .cmd_write_o   (bus_cmd_write),
      .cmd_type1_o   (bus_cmd_type1),
      .cmd_reg_num_o (bus_cmd_reg_num),
      .cmd_ext_reg_o (bus_cmd_ext_reg),
      .cmd_first_be_o(bus_cmd_first_be),
      .cmd_wdata_o   (bus_cmd_wdata),

      .rsp_valid_i  (bus_rsp_valid),
      .rsp_ready_o  (bus_rsp_ready),
      .rsp_outcome_i(rsp_outcome),
      .rsp_rdata_i  (rsp_rdata)
  );

  // The second scan, for the secondary bus. Its scan_bus_i is pcie_enum_bus's
  // sec_bus_o, 0 until bus_done_o and the secondary bus number after, and its
  // start is bus_done_o itself, so no probe goes out before the bridge routes
  // that bus.
  pcie_enum_scan u_scan2 (
      .clk_i(clk_i),
      .rst_i(rst_i),

      .scan_start_i(bus_done_o),
      .scan_bus_i  (bus_sec_bus),

      .scan_busy_o         (),
      .scan_done_o         (sec_scan_done_o),
      .scan_error_o        (scan2_error),
      .scan_error_code_o   (scan2_error_code),
      .err_credit_blocked_o(scan2_credit_blocked),

      .device_present_o    (sec_device_present_o),
      .unsupported_device_o(sec_unsupported_device_o),
      .device_bdf_o        (sec_device_bdf_o),
      .vendor_id_o         (sec_vendor_id_o),
      .device_id_o         (sec_device_id_o),
      .header_type_o       (sec_header_type_o),
      .multifunction_o     (sec_multifunction_o),

      .tx_fc_blocked_i(tx_fc_blocked_i),

      .cmd_valid_o   (scan2_cmd_valid),
      .cmd_ready_i   (scan2_cmd_ready),
      .cmd_write_o   (scan2_cmd_write),
      .cmd_reg_num_o (scan2_cmd_reg_num),
      .cmd_ext_reg_o (scan2_cmd_ext_reg),
      .cmd_first_be_o(scan2_cmd_first_be),
      .cmd_wdata_o   (scan2_cmd_wdata),

      .rsp_valid_i  (scan2_rsp_valid),
      .rsp_ready_o  (scan2_rsp_ready),
      .rsp_outcome_i(rsp_outcome),
      .rsp_rdata_i  (rsp_rdata)
  );

  // The second BAR stage, for the device the second scan found. It allocates
  // from the same window: on the bridge path the first-level BAR stage
  // configures nothing (S_CHECK sees the unsupported verdict and ends without
  // a transaction), so no address is given out twice.
  //
  // It addresses the second scan's device (cmd_bdf below), so it never sizes
  // the bridge itself, whose register 6 holds the bus numbers that a six-BAR
  // sizing pass would overwrite.
  pcie_enum_bar #(
      .MEM_BAR_BASE  (MEM_BAR_BASE),
      .MEM_BAR_WINDOW(MEM_BAR_WINDOW)
  ) u_bar2 (
      .clk_i(clk_i),
      .rst_i(rst_i),

      .bar_start_i         (bridge_enable_i && sec_scan_done_o),
      .device_present_i    (sec_device_present_o),
      .unsupported_device_i(sec_unsupported_device_o),

      .tx_fc_blocked_i(tx_fc_blocked_i),

      .bar_busy_o          (),
      .enum_done_o         (sec_enum_done_o),
      .bar_error_o         (bar2_error),
      .bar_error_code_o    (bar2_error_code),
      .err_credit_blocked_o(bar2_credit_blocked),

      .bar_count_o   (sec_bar_count_o),
      .bar_valid_o   (sec_bar_valid_o),
      .bar_is_64_o   (sec_bar_is_64_o),
      .bar_prefetch_o(sec_bar_prefetch_o),
      .bar_size_o    (sec_bar_size_o),
      .bar_addr_o    (sec_bar_addr_o),
      .io_bar_mask_o (sec_io_bar_mask_o),

      .cmd_valid_o   (bar2_cmd_valid),
      .cmd_ready_i   (bar2_cmd_ready),
      .cmd_write_o   (bar2_cmd_write),
      .cmd_reg_num_o (bar2_cmd_reg_num),
      .cmd_ext_reg_o (bar2_cmd_ext_reg),
      .cmd_first_be_o(bar2_cmd_first_be),
      .cmd_wdata_o   (bar2_cmd_wdata),

      .rsp_valid_i  (bar2_rsp_valid),
      .rsp_ready_o  (bar2_rsp_ready),
      .rsp_outcome_i(rsp_outcome),
      .rsp_rdata_i  (rsp_rdata)
  );

  // -------------------------------------------------------------------------
  // Handoff mux
  // -------------------------------------------------------------------------
  // One arm per stage, each owning the port for a span set by terminal outputs:
  //   scan1   until scan_done_o
  //   bar1    after it, when the bridge path is not taken
  //   bus     after it, on the bridge path, until bus_done_o; if
  //           pcie_enum_bus bypasses or fails, it keeps the port, idle,
  //           until reset
  //   scan2   bridge path, after bus_done_o, until sec_scan_done_o
  //   bar2    bridge path, after sec_scan_done_o
  // Exactly one own_* is true at any time. Every select term is a terminal
  // output that rises at most once per reset and then holds, or
  // bridge_enable_i, so while bridge_enable_i is held ownership only moves
  // forward. There is no arbiter and no registered state.

  // The bridge path is taken when bridge_enable_i is set and the first scan
  // found a header that is not Type 0; pcie_enum_bus then checks for Type 1.
  wire bridge_path = bridge_enable_i && unsupported_device_o;

  wire own_scan1 = !scan_done_o;
  wire own_bar1  = scan_done_o && !bridge_path;
  wire own_bus   = scan_done_o && bridge_path && !bus_done_o;
  wire own_scan2 = bridge_path && bus_done_o && !sec_scan_done_o;
  wire own_bar2  = bridge_path && bus_done_o && sec_scan_done_o;

  // Select, never merge: the scan stages drive cmd_first_be = 1111b in every
  // state, so OR-ing the ports would turn a BAR stage's 0011b Command write
  // into a whole-Dword write.
  assign cmd_valid    = own_scan1 ? scan_cmd_valid
                      : own_bar1  ? bar_cmd_valid
                      : own_bus   ? bus_cmd_valid
                      : own_scan2 ? scan2_cmd_valid
                                  : bar2_cmd_valid;
  assign cmd_write    = own_scan1 ? scan_cmd_write
                      : own_bar1  ? bar_cmd_write
                      : own_bus   ? bus_cmd_write
                      : own_scan2 ? scan2_cmd_write
                                  : bar2_cmd_write;
  assign cmd_reg_num  = own_scan1 ? scan_cmd_reg_num
                      : own_bar1  ? bar_cmd_reg_num
                      : own_bus   ? bus_cmd_reg_num
                      : own_scan2 ? scan2_cmd_reg_num
                                  : bar2_cmd_reg_num;
  assign cmd_ext_reg  = own_scan1 ? scan_cmd_ext_reg
                      : own_bar1  ? bar_cmd_ext_reg
                      : own_bus   ? bus_cmd_ext_reg
                      : own_scan2 ? scan2_cmd_ext_reg
                                  : bar2_cmd_ext_reg;
  assign cmd_first_be = own_scan1 ? scan_cmd_first_be
                      : own_bar1  ? bar_cmd_first_be
                      : own_bus   ? bus_cmd_first_be
                      : own_scan2 ? scan2_cmd_first_be
                                  : bar2_cmd_first_be;
  assign cmd_wdata    = own_scan1 ? scan_cmd_wdata
                      : own_bar1  ? bar_cmd_wdata
                      : own_bus   ? bus_cmd_wdata
                      : own_scan2 ? scan2_cmd_wdata
                                  : bar2_cmd_wdata;
  assign rsp_ready    = own_scan1 ? scan_rsp_ready
                      : own_bar1  ? bar_rsp_ready
                      : own_bus   ? bus_rsp_ready
                      : own_scan2 ? scan2_rsp_ready
                                  : bar2_rsp_ready;

  // Type 0 for every first-level stage, Type 1 for the second level. A
  // first-level target is on the bus directly below the port, and Type 1 is
  // needed only for a target on another bus (PCI Local Bus Spec r3.0,
  // §3.2.2.3); pcie_enum_bus's cmd_type1_o is a constant 0 for this reason.
  // The second level is the bridge's secondary bus, and the bridge turns a
  // Type 1 request for that bus into Type 0 (PCIe Base Spec r2.1, §7.3.3).
  // Every Type 1 request the engine issues comes from the scan2 and bar2 arms.
  wire cmd_type1 = own_bus ? bus_cmd_type1
                 : (own_scan2 || own_bar2);

  // Selected by bus level, never by stage: the first scan's device_bdf_o for
  // every first-level stage, including pcie_enum_bus, whose target is the
  // bridge that scan found, and the second scan's for the second level. The
  // stages of one level therefore always address the same device.
  wire [15:0] cmd_bdf = (own_scan2 || own_bar2) ? sec_device_bdf_o
                                                : device_bdf_o;

  // The back channels are gated the same way. A stage that does not own the
  // port sees cmd_ready = 0 and rsp_valid = 0, so it cannot complete a
  // handshake on traffic that is not its own.
  assign scan_cmd_ready  = own_scan1 ? cmd_ready : 1'b0;
  assign bar_cmd_ready   = own_bar1  ? cmd_ready : 1'b0;
  assign bus_cmd_ready   = own_bus   ? cmd_ready : 1'b0;
  assign scan2_cmd_ready = own_scan2 ? cmd_ready : 1'b0;
  assign bar2_cmd_ready  = own_bar2  ? cmd_ready : 1'b0;
  assign scan_rsp_valid  = own_scan1 ? rsp_valid : 1'b0;
  assign bar_rsp_valid   = own_bar1  ? rsp_valid : 1'b0;
  assign bus_rsp_valid   = own_bus   ? rsp_valid : 1'b0;
  assign scan2_rsp_valid = own_scan2 ? rsp_valid : 1'b0;
  assign bar2_rsp_valid  = own_bar2  ? rsp_valid : 1'b0;

  // -------------------------------------------------------------------------
  // Combined status
  // -------------------------------------------------------------------------
  // At most one stage can be in error. Each stage starts only once the stage
  // before it has ended without error, and of the two stages that start from
  // scan_done_o, the one off the path taken ends without a transaction if it
  // starts at all. The error code is selected in pipeline order.
  assign enum_error_o         = scan_error_o || bar_error || bus_error ||
                                scan2_error || bar2_error;
  assign enum_error_code_o    = scan_error_o ? scan_error_code
                              : bar_error    ? bar_error_code
                              : bus_error    ? bus_error_code
                              : scan2_error  ? scan2_error_code
                                             : bar2_error_code;
  assign err_credit_blocked_o = scan_credit_blocked || bar_credit_blocked ||
                                bus_credit_blocked || scan2_credit_blocked ||
                                bar2_credit_blocked;

  // -------------------------------------------------------------------------
  // Transaction primitive
  // -------------------------------------------------------------------------
  // The only pcie_cfg_txn: one configuration request is in flight at a time
  // because the netlist has one, not because the stages cooperate.
  // pcie_cfg_txn says why one is enough.
  pcie_cfg_txn #(
      .AXIS_DATA_WIDTH   (AXIS_DATA_WIDTH),
      .AXIS_KEEP_WIDTH   (AXIS_KEEP_WIDTH),
      .AXIS_USER_WIDTH   (AXIS_USER_WIDTH),
      .CRS_RETRY_MAX     (CRS_RETRY_MAX),
      .CRS_BACKOFF_CYCLES(CRS_BACKOFF_CYCLES),
      .CPL_TIMEOUT_CYCLES(CPL_TIMEOUT_CYCLES)
  ) u_txn (
      .clk_i(clk_i),
      .rst_i(rst_i),

      .cmd_valid_i   (cmd_valid),
      .cmd_ready_o   (cmd_ready),
      .cmd_write_i   (cmd_write),
      // From the mux above: 0 for the first level, 1 for the second.
      .cmd_type1_i   (cmd_type1),
      // Selected by bus level, never by stage; see the mux above.
      .cmd_bdf_i     (cmd_bdf),
      .cmd_reg_num_i (cmd_reg_num),
      .cmd_ext_reg_i (cmd_ext_reg),
      .cmd_first_be_i(cmd_first_be),
      .cmd_wdata_i   (cmd_wdata),

      .rsp_valid_o     (rsp_valid),
      .rsp_ready_i     (rsp_ready),
      .rsp_outcome_o   (rsp_outcome),
      .rsp_rdata_o     (rsp_rdata),
      // Unconnected: the stages read only rsp_outcome and rsp_rdata.
      .rsp_status_raw_o(),
      .crs_retries_o   (),

      .s_axis_rq_tdata_o (s_axis_rq_tdata_o),
      .s_axis_rq_tkeep_o (s_axis_rq_tkeep_o),
      .s_axis_rq_tvalid_o(s_axis_rq_tvalid_o),
      .s_axis_rq_tlast_o (s_axis_rq_tlast_o),
      .s_axis_rq_tuser_o (s_axis_rq_tuser_o),
      .s_axis_rq_tready_i(s_axis_rq_tready_i),

      .pcie_rq_tag_i    (pcie_rq_tag_i),
      .pcie_rq_tag_vld_i(pcie_rq_tag_vld_i),

      .m_axis_rc_tdata_i (m_axis_rc_tdata_i),
      .m_axis_rc_tkeep_i (m_axis_rc_tkeep_i),
      .m_axis_rc_tvalid_i(m_axis_rc_tvalid_i),
      .m_axis_rc_tlast_i (m_axis_rc_tlast_i),
      .m_axis_rc_tready_o(m_axis_rc_tready_o),

      .cpl_timeout_valid_i(cpl_timeout_valid_i),
      .cpl_timeout_tag_i  (cpl_timeout_tag_i)
  );

endmodule
