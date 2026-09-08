`timescale 1ns/1ps

// Declarations-plus-DUT wrapper for pcie_enum_dl_top, shaped on
// tb_pcie_rc_dl_top.sv (the FC aliases) and tb_pcie_enum_bridge_tlp.sv (the
// enum-type flattening).  No far-end model here: the far end is Python
// (test_pcie_enum_dl_top.py) on s_phy_axis / m_phy_axis, behind the real
// data-link layer.
//
// SS WHY THIS WRAPPER EXISTS AT ALL.  Two reasons, both cocotb limitations
// rather than design ones:
//   1. cocotb cannot read a port whose type is a SystemVerilog enum, so the
//      eight enum-typed status outputs are flattened to plain vectors below.
//      Each is annotated with the tb_pcie_enum_bridge_tlp.sv line it
//      reproduces -- this is a port of proven code, not new code.
//   2. Three signals the tests need are INTERNAL in pcie_enum_dl_top by
//      design: the PG213 RQ/RC socket is the seam between the two children and
//      is deliberately not on the surface (DESIGN SS3.1).  They are reached
//      here by hierarchical reference, the same mechanism tb_pcie_rc_dl_top.sv
//      already uses and Verilator already supports under --public-flat-rw.
module tb_pcie_enum_dl_top;
  import tlp_pkg::*;
  import pcie_rq_rc_pkg::*;
  import pcie_enum_pkg::*;

  // ---- the shared parameter set, one name each (RECON_REFRESH SS3.1) -------
  // These four are the ONLY parameters both children carry.  The DUT passes
  // each to both; test (e) asserts that it did.
  localparam int AXIS_DATA_WIDTH = 128;
  localparam int AXIS_KEEP_WIDTH = AXIS_DATA_WIDTH / 32;
  localparam int AXIS_USER_WIDTH = 60;
  localparam int unsigned CPL_TIMEOUT_CYCLES = 32'd4096;

  // ---- child-specific -----------------------------------------------------
  localparam int TAG_COUNT = 32;               // pcie_rc_dl_top's tested default
  localparam int unsigned CRS_RETRY_MAX      = 3;
  localparam int unsigned CRS_BACKOFF_CYCLES = 8;

  logic clk_i;
  logic rst_i;

  // ---- link state ---------------------------------------------------------
  logic phy_link_up_i;
  logic idle_valid_i;
  logic transmit_enable_i;

  // ---- PHY-facing streams: where the Python far end sits ------------------
  logic [31:0] s_phy_axis_tdata;
  logic [3:0]  s_phy_axis_tkeep;
  logic        s_phy_axis_tvalid;
  logic        s_phy_axis_tlast;
  logic [2:0]  s_phy_axis_tuser;
  logic        s_phy_axis_tready;
  logic [31:0] m_phy_axis_tdata;
  logic [3:0]  m_phy_axis_tkeep;
  logic        m_phy_axis_tvalid;
  logic        m_phy_axis_tlast;
  logic [2:0]  m_phy_axis_tuser;
  logic        m_phy_axis_tready;

  // ---- identity and negotiated limits -------------------------------------
  logic [15:0] requester_id_i;
  logic [15:0] completer_id_i;
  logic [7:0]  bus_number_i;
  logic [4:0]  device_number_i;
  logic [2:0]  function_number_i;
  logic        memory_enable_i;
  logic        extended_tag_enable_i;
  logic [12:0] max_payload_bytes_i;
  logic [12:0] max_read_bytes_i;
  logic        rcb_128b_i;

  logic [7:0] cfg_bus_number_o;
  logic [4:0] cfg_device_number_o;
  logic [2:0] cfg_function_number_o;

  // ---- start-gate status, real ports since the start-gate rung -------------
  logic       fc_init_done_o;
  logic       ok_to_issue_o;

  // ---- enumeration control ------------------------------------------------
  logic       scan_start_i;
  logic [7:0] scan_bus_i;
  logic       bar_enable_i;
  logic       bridge_enable_i;

  // ---- presence-phase status, enums flattened for cocotb ------------------
  logic        scan_busy_o;
  logic        scan_done_o;
  logic        scan_error_o;
  enum_error_e scan_error_code;
  logic [3:0]  scan_error_code_o;
  assign scan_error_code_o = 4'(scan_error_code);       // tb_pcie_enum_bridge_tlp.sv:74
  logic        err_credit_blocked_o;

  logic        device_present_o;
  logic        unsupported_device_o;
  logic [15:0] device_bdf_o;
  logic [15:0] vendor_id_o;
  logic [15:0] device_id_o;
  logic [7:0]  header_type_o;
  logic        multifunction_o;

  // ---- BAR-phase status, enums flattened for cocotb -----------------------
  logic        bar_busy_o;
  logic        enum_done_o;
  logic        enum_error_o;
  enum_error_e enum_error_code;
  logic [3:0]  enum_error_code_o;
  assign enum_error_code_o = 4'(enum_error_code);       // tb_pcie_enum_bridge_tlp.sv:92

  logic [3:0]              bar_count_o;
  logic [BAR_SLOTS-1:0]    bar_valid_o;
  logic [BAR_SLOTS-1:0]    bar_is_64_o;
  logic [BAR_SLOTS-1:0]    bar_prefetch_o;
  logic [BAR_SLOTS*64-1:0] bar_size_o;
  logic [BAR_SLOTS*64-1:0] bar_addr_o;
  logic [BAR_SLOTS-1:0]    io_bar_mask_o;

  // ---- bridge-path status (Stage D).  Declared in full even though this
  // rung drives bridge_enable_i low: an unconnected real output costs a
  // PINMISSING waiver, and this codebase keeps PINMISSING enabled for genuine
  // omissions (pcie_rc_dl_top.sv:326-328).
  logic                    bus_done_o;
  logic                    bus_bypassed_o;
  logic                    sec_scan_done_o;
  logic                    sec_device_present_o;
  logic                    sec_unsupported_device_o;
  logic [15:0]             sec_device_bdf_o;
  logic [15:0]             sec_vendor_id_o;
  logic [15:0]             sec_device_id_o;
  logic [7:0]              sec_header_type_o;
  logic                    sec_multifunction_o;
  logic                    sec_enum_done_o;
  logic [3:0]              sec_bar_count_o;
  logic [BAR_SLOTS-1:0]    sec_bar_valid_o;
  logic [BAR_SLOTS-1:0]    sec_bar_is_64_o;
  logic [BAR_SLOTS-1:0]    sec_bar_prefetch_o;
  logic [BAR_SLOTS*64-1:0] sec_bar_size_o;
  logic [BAR_SLOTS*64-1:0] sec_bar_addr_o;
  logic [BAR_SLOTS-1:0]    sec_io_bar_mask_o;

  // ---- TL / RQ / RC error surface, enums flattened for cocotb -------------
  // ---- completer surface (Stage F-3) --------------------------------------
  // Declared and connected so the new DUT ports are not left unbound: there is
  // no PINMISSING waiver in lint/waiver.vlt, so an unconnected pin would emit a
  // warning and move the gate artifact without any behaviour changing.
  localparam int CQ_USER_WIDTH = 88;
  localparam int CC_USER_WIDTH = 33;

  logic [AXIS_DATA_WIDTH-1:0] m_axis_cq_tdata;
  logic [AXIS_KEEP_WIDTH-1:0] m_axis_cq_tkeep;
  logic                       m_axis_cq_tvalid;
  logic                       m_axis_cq_tlast;
  logic [CQ_USER_WIDTH-1:0]   m_axis_cq_tuser;
  logic                       m_axis_cq_tready;

  logic [AXIS_DATA_WIDTH-1:0] s_axis_cc_tdata;
  logic [AXIS_KEEP_WIDTH-1:0] s_axis_cc_tkeep;
  logic                       s_axis_cc_tvalid;
  logic                       s_axis_cc_tlast;
  logic [CC_USER_WIDTH-1:0]   s_axis_cc_tuser;
  logic                       s_axis_cc_tready;

  logic       cq_dropped_o;
  logic [3:0] cq_error_code_o;
  logic       cq_gearbox_error_o;
  logic       cc_protocol_error_o;
  logic [3:0] cc_error_code_o;
  logic       cc_gearbox_error_o;

  logic       rq_protocol_error_o;
  rq_error_e  rq_error_code;
  logic [3:0] rq_error_code_o;
  assign rq_error_code_o = 4'(rq_error_code);           // tb_pcie_enum_bridge_tlp.sv:159
  logic       rq_gearbox_error_o;

  logic       rc_unexpected_completion_o;
  tlp_error_e rc_completion_error_code;
  logic [4:0] rc_completion_error_code_o;
  assign rc_completion_error_code_o = 5'(rc_completion_error_code); // ...:165

  logic       rc_protocol_error_o;
  rc_error_e  rc_error_code;
  logic [3:0] rc_error_code_o;
  assign rc_error_code_o = 4'(rc_error_code);           // tb_pcie_enum_bridge_tlp.sv:170
  logic       rc_gearbox_error_o;

  logic       command_error_valid_o;
  tlp_error_e command_error_code;
  logic [4:0] command_error_code_o;
  assign command_error_code_o = 5'(command_error_code); // tb_pcie_enum_bridge_tlp.sv:176

  logic       malformed_o;
  logic       rx_error_valid_o;
  tlp_error_e rx_error_code;
  logic [4:0] rx_error_code_o;
  assign rx_error_code_o = 5'(rx_error_code);           // tb_pcie_enum_bridge_tlp.sv:182
  logic       rx_ecrc_error_o;

  logic       tx_error_valid_o;
  tlp_error_e tx_error_code;
  logic [4:0] tx_error_code_o;
  assign tx_error_code_o = 5'(tx_error_code);           // tb_pcie_enum_bridge_tlp.sv:188

  logic       tx_fc_blocked_o;
  logic       credit_error_o;
  logic       vc_overflow_o;
  logic       cpl_timeout_valid_o;
  logic [7:0] cpl_timeout_tag_o;
  logic       late_cpl_valid_o;
  logic [7:0] late_cpl_tag_o;
  logic [$clog2(TAG_COUNT+1)-1:0] outstanding_o;

  // =========================================================================
  // Verification-only visibility, by hierarchical reference.
  // =========================================================================
  // The FC seam.  fc_initialized_o is the FILTER OUTPUT, the wire
  // u_rc.fc_initialized_i is driven by, so the shared initialize_flow_control
  // helper waits on the TRANSACTION LAYER's view of FC init.
  //
  // IT IS NO LONGER A HIERARCHICAL REACH.  It is an alias of the real port
  // fc_init_done_o, kept under the old name only because the shared helper
  // reads dut.fc_initialized_o by that name
  // (test_pcie_endpoint_top.py:170).  The reach into the FC-init filter
  // register (pcie_rc_dl_top.sv:181) that used to be here existed because that
  // signal was not on pcie_rc_dl_top's port list; the start-gate rung put it
  // there, and gating now happens in the DUT rather than in the bench.
  //
  // fc_initialized_dll is the DLL's raw, glitching output and STAYS a reach:
  // it is deliberately not a port, because nothing outside verification should
  // consume an unfiltered FC-init.
  wire        fc_initialized_dll = dut.u_rcdl.dl_fc_initialized;
  wire        fc_initialized_o   = fc_init_done_o;
  wire        fc_update_valid_o  = dut.u_rcdl.dl_fc_update_valid;
  wire [7:0]  fc_ph_o   = dut.u_rcdl.dl_fc_ph;
  wire [11:0] fc_pd_o   = dut.u_rcdl.dl_fc_pd;
  wire [7:0]  fc_nph_o  = dut.u_rcdl.dl_fc_nph;
  wire [11:0] fc_npd_o  = dut.u_rcdl.dl_fc_npd;
  wire [7:0]  fc_cplh_o = dut.u_rcdl.dl_fc_cplh;
  wire [11:0] fc_cpld_o = dut.u_rcdl.dl_fc_cpld;

  // The PG213 seam.  These three are INTERNAL in pcie_enum_dl_top on purpose --
  // the socket disappears from the surface because pcie_enum_top is the only
  // master (DESIGN SS3.1).  They are aliased here for one reason: it lets
  // enum_tb_common's Mon transfer to this bench UNCHANGED.  Mon samples
  // pcie_rq_tag_vld_o/pcie_rq_tag_o (enum_tb_common.py:890-891) and
  // s_axis_rq_tvalid (:911), and every other surface it reads is a real
  // top-level port here.  Observation only; nothing drives these.
  wire [7:0] pcie_rq_tag_o     = dut.rq_tag;
  wire       pcie_rq_tag_vld_o = dut.rq_tag_vld;
  wire       s_axis_rq_tvalid  = dut.rq_tvalid;

  // =========================================================================
  // The DUT.  Explicit named connections throughout -- .* cannot be used
  // because the eight enum-typed ports connect to differently-named nets.
  // =========================================================================
  pcie_enum_dl_top #(
      .AXIS_DATA_WIDTH   (AXIS_DATA_WIDTH),
      .AXIS_USER_WIDTH   (AXIS_USER_WIDTH),
      .CPL_TIMEOUT_CYCLES(CPL_TIMEOUT_CYCLES),
      .TAG_COUNT         (TAG_COUNT),
      .CRS_RETRY_MAX     (CRS_RETRY_MAX),
      .CRS_BACKOFF_CYCLES(CRS_BACKOFF_CYCLES)
  ) dut (
      .clk_i(clk_i),
      .rst_i(rst_i),

      .phy_link_up_i    (phy_link_up_i),
      .idle_valid_i     (idle_valid_i),
      .transmit_enable_i(transmit_enable_i),

      .s_phy_axis_tdata (s_phy_axis_tdata),
      .s_phy_axis_tkeep (s_phy_axis_tkeep),
      .s_phy_axis_tvalid(s_phy_axis_tvalid),
      .s_phy_axis_tlast (s_phy_axis_tlast),
      .s_phy_axis_tuser (s_phy_axis_tuser),
      .s_phy_axis_tready(s_phy_axis_tready),
      .m_phy_axis_tdata (m_phy_axis_tdata),
      .m_phy_axis_tkeep (m_phy_axis_tkeep),
      .m_phy_axis_tvalid(m_phy_axis_tvalid),
      .m_phy_axis_tlast (m_phy_axis_tlast),
      .m_phy_axis_tuser (m_phy_axis_tuser),
      .m_phy_axis_tready(m_phy_axis_tready),

      .requester_id_i       (requester_id_i),
      .completer_id_i       (completer_id_i),
      .bus_number_i         (bus_number_i),
      .device_number_i      (device_number_i),
      .function_number_i    (function_number_i),
      .memory_enable_i      (memory_enable_i),
      .extended_tag_enable_i(extended_tag_enable_i),
      .max_payload_bytes_i  (max_payload_bytes_i),
      .max_read_bytes_i     (max_read_bytes_i),
      .rcb_128b_i           (rcb_128b_i),

      .cfg_bus_number_o     (cfg_bus_number_o),
      .cfg_device_number_o  (cfg_device_number_o),
      .cfg_function_number_o(cfg_function_number_o),

      .fc_init_done_o(fc_init_done_o),
      .ok_to_issue_o (ok_to_issue_o),

      .scan_start_i   (scan_start_i),
      .scan_bus_i     (scan_bus_i),
      .bar_enable_i   (bar_enable_i),
      .bridge_enable_i(bridge_enable_i),

      .scan_busy_o         (scan_busy_o),
      .scan_done_o         (scan_done_o),
      .scan_error_o        (scan_error_o),
      .scan_error_code_o   (scan_error_code),
      .err_credit_blocked_o(err_credit_blocked_o),
      .device_present_o    (device_present_o),
      .unsupported_device_o(unsupported_device_o),
      .device_bdf_o        (device_bdf_o),
      .vendor_id_o         (vendor_id_o),
      .device_id_o         (device_id_o),
      .header_type_o       (header_type_o),
      .multifunction_o     (multifunction_o),

      .bar_busy_o       (bar_busy_o),
      .enum_done_o      (enum_done_o),
      .enum_error_o     (enum_error_o),
      .enum_error_code_o(enum_error_code),
      .bar_count_o      (bar_count_o),
      .bar_valid_o      (bar_valid_o),
      .bar_is_64_o      (bar_is_64_o),
      .bar_prefetch_o   (bar_prefetch_o),
      .bar_size_o       (bar_size_o),
      .bar_addr_o       (bar_addr_o),
      .io_bar_mask_o    (io_bar_mask_o),

      .bus_done_o              (bus_done_o),
      .bus_bypassed_o          (bus_bypassed_o),
      .sec_scan_done_o         (sec_scan_done_o),
      .sec_device_present_o    (sec_device_present_o),
      .sec_unsupported_device_o(sec_unsupported_device_o),
      .sec_device_bdf_o        (sec_device_bdf_o),
      .sec_vendor_id_o         (sec_vendor_id_o),
      .sec_device_id_o         (sec_device_id_o),
      .sec_header_type_o       (sec_header_type_o),
      .sec_multifunction_o     (sec_multifunction_o),
      .sec_enum_done_o         (sec_enum_done_o),
      .sec_bar_count_o         (sec_bar_count_o),
      .sec_bar_valid_o         (sec_bar_valid_o),
      .sec_bar_is_64_o         (sec_bar_is_64_o),
      .sec_bar_prefetch_o      (sec_bar_prefetch_o),
      .sec_bar_size_o          (sec_bar_size_o),
      .sec_bar_addr_o          (sec_bar_addr_o),
      .sec_io_bar_mask_o       (sec_io_bar_mask_o),

      .m_axis_cq_tdata (m_axis_cq_tdata),  .m_axis_cq_tkeep (m_axis_cq_tkeep),
      .m_axis_cq_tvalid(m_axis_cq_tvalid), .m_axis_cq_tlast (m_axis_cq_tlast),
      .m_axis_cq_tuser (m_axis_cq_tuser),  .m_axis_cq_tready(m_axis_cq_tready),
      .s_axis_cc_tdata (s_axis_cc_tdata),  .s_axis_cc_tkeep (s_axis_cc_tkeep),
      .s_axis_cc_tvalid(s_axis_cc_tvalid), .s_axis_cc_tlast (s_axis_cc_tlast),
      .s_axis_cc_tuser (s_axis_cc_tuser),  .s_axis_cc_tready(s_axis_cc_tready),
      .cq_dropped_o    (cq_dropped_o),     .cq_error_code_o (cq_error_code_o),
      .cq_gearbox_error_o(cq_gearbox_error_o),
      .cc_protocol_error_o(cc_protocol_error_o),
      .cc_error_code_o (cc_error_code_o),
      .cc_gearbox_error_o(cc_gearbox_error_o),

      .rq_protocol_error_o       (rq_protocol_error_o),
      .rq_error_code_o           (rq_error_code),
      .rq_gearbox_error_o        (rq_gearbox_error_o),
      .rc_unexpected_completion_o(rc_unexpected_completion_o),
      .rc_completion_error_code_o(rc_completion_error_code),
      .rc_protocol_error_o       (rc_protocol_error_o),
      .rc_error_code_o           (rc_error_code),
      .rc_gearbox_error_o        (rc_gearbox_error_o),
      .command_error_valid_o     (command_error_valid_o),
      .command_error_code_o      (command_error_code),
      .malformed_o               (malformed_o),
      .rx_error_valid_o          (rx_error_valid_o),
      .rx_error_code_o           (rx_error_code),
      .rx_ecrc_error_o           (rx_ecrc_error_o),
      .tx_error_valid_o          (tx_error_valid_o),
      .tx_error_code_o           (tx_error_code),
      .tx_fc_blocked_o           (tx_fc_blocked_o),
      .credit_error_o            (credit_error_o),
      .vc_overflow_o             (vc_overflow_o),
      .cpl_timeout_valid_o       (cpl_timeout_valid_o),
      .cpl_timeout_tag_o         (cpl_timeout_tag_o),
      .late_cpl_valid_o          (late_cpl_valid_o),
      .late_cpl_tag_o            (late_cpl_tag_o),
      .outstanding_o             (outstanding_o)
  );

endmodule
