`timescale 1ns/1ps

// Declarations-plus-DUT wrapper for pcie_rc_dl_top, shaped on
// tb_pcie_endpoint_top.sv.  No far-end model here: the far end is Python
// (test_pcie_rc_dl_top.py) on s_phy_axis/m_phy_axis.
module tb_pcie_rc_dl_top;
  import tlp_pkg::*;
  import pcie_rq_rc_pkg::*;

  localparam int AXIS_DATA_WIDTH = 128;
  localparam int AXIS_KEEP_WIDTH = 4;
  localparam int AXIS_USER_WIDTH = 60;
  localparam int TAG_COUNT = 32;

  logic clk_i;
  logic rst_i;
  logic phy_link_up_i;
  logic idle_valid_i;
  logic transmit_enable_i;

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

  logic [AXIS_DATA_WIDTH-1:0] s_axis_rq_tdata;
  logic [AXIS_KEEP_WIDTH-1:0] s_axis_rq_tkeep;
  logic                       s_axis_rq_tvalid;
  logic                       s_axis_rq_tlast;
  logic [AXIS_USER_WIDTH-1:0] s_axis_rq_tuser;
  logic                       s_axis_rq_tready;

  logic [7:0] pcie_rq_tag_o;
  logic       pcie_rq_tag_vld_o;

  logic [AXIS_DATA_WIDTH-1:0] m_axis_rc_tdata;
  logic [AXIS_KEEP_WIDTH-1:0] m_axis_rc_tkeep;
  logic                       m_axis_rc_tvalid;
  logic                       m_axis_rc_tlast;
  logic                       m_axis_rc_tready;

  logic [7:0] cfg_bus_number_o;
  logic [4:0] cfg_device_number_o;
  logic [2:0] cfg_function_number_o;

  // ---- completer surface (Stage F-3) --------------------------------------
  // The DUT is instantiated with .*, so these declarations ARE the connection:
  // without them the new ports have no matching name and elaboration fails.
  // Undriven here on purpose -- the CC slave inputs stay at their initial value
  // until the rows that drive them land.
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
  rq_error_e  rq_error_code_o;
  logic       rq_gearbox_error_o;
  logic       rc_unexpected_completion_o;
  tlp_error_e rc_completion_error_code_o;
  logic       rc_protocol_error_o;
  rc_error_e  rc_error_code_o;
  logic       rc_gearbox_error_o;
  logic       command_error_valid_o;
  tlp_error_e command_error_code_o;
  logic       malformed_o;
  logic       rx_error_valid_o;
  tlp_error_e rx_error_code_o;
  logic       rx_ecrc_error_o;
  logic       tx_error_valid_o;
  tlp_error_e tx_error_code_o;
  logic       tx_fc_blocked_o;
  logic       credit_error_o;
  logic       vc_overflow_o;
  logic       cpl_timeout_valid_o;
  logic [7:0] cpl_timeout_tag_o;
  logic       late_cpl_valid_o;
  logic [7:0] late_cpl_tag_o;
  logic [5:0] outstanding_o;

  // Start-gate status ports.  Declared explicitly rather than left to .*'s
  // implicit-net rule, matching every other signal in this wrapper.
  logic       fc_init_done_o;
  logic       ok_to_issue_o;

  // Verification-only visibility of the FC seam between the two instances.
  // fc_initialized_o is the FILTER OUTPUT -- the wire u_rc.fc_initialized_i is
  // driven by -- so the shared initialize_flow_control helper waits on the
  // TL's view of FC init.
  //
  // IT IS NO LONGER A HIERARCHICAL REACH.  It is an alias of the real port
  // fc_init_done_o, kept under the old name only because the shared helper
  // reads dut.fc_initialized_o by that name (test_pcie_endpoint_top.py:170).
  //
  // fc_initialized_dll is the DLL's raw, glitching output and STAYS a reach:
  // it is deliberately not a port, because nothing outside verification should
  // consume an unfiltered FC-init.  It is test (a)'s negative control.
  wire        fc_initialized_dll = dut.dl_fc_initialized;
  wire        fc_initialized_o   = fc_init_done_o;
  wire        fc_update_valid_o  = dut.dl_fc_update_valid;
  wire [7:0]  fc_ph_o   = dut.dl_fc_ph;
  wire [11:0] fc_pd_o   = dut.dl_fc_pd;
  wire [7:0]  fc_nph_o  = dut.dl_fc_nph;
  wire [11:0] fc_npd_o  = dut.dl_fc_npd;
  wire [7:0]  fc_cplh_o = dut.dl_fc_cplh;
  wire [11:0] fc_cpld_o = dut.dl_fc_cpld;

  pcie_rc_dl_top #(
      .AXIS_DATA_WIDTH(AXIS_DATA_WIDTH),
      .AXIS_KEEP_WIDTH(AXIS_KEEP_WIDTH),
      .AXIS_USER_WIDTH(AXIS_USER_WIDTH),
      .TAG_COUNT(TAG_COUNT)
  ) dut (
      .*
  );

endmodule
