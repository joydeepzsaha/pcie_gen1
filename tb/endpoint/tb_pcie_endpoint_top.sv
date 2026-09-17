`timescale 1ns/1ps

module tb_pcie_endpoint_top;
  import tlp_pkg::*;

  localparam int DATA_WIDTH = 32;
  localparam int KEEP_WIDTH = 4;
  // §63 #7d: 5, not 3 -- at 3 frame_symbols' 4-bit K-position mask truncates and
  // the END Symbol loses its K flag. The default moved too; this override is
  // explicit, so the default alone would not have reached this bench.
  localparam int USER_WIDTH = 5;
  localparam int CONTEXT_WIDTH = 16;
  localparam int TLP_HEADER_WIDTH = $bits(tlp_header_t);
  // Mirrors pcie_endpoint_top.sv:32. The instantiation below does not override
  // MAX_NUM_LANES, so this must track that default; it sizes only the inert PHY
  // nets declared just above the DUT.
  localparam int MAX_NUM_LANES = 1;

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

  logic memory_enable_i;
  logic extended_tag_enable_i;
  logic [12:0] max_payload_bytes_i;
  logic [12:0] max_read_bytes_i;
  logic rcb_128b_i;

  logic command_valid_i;
  logic command_ready_o;
  tlp_cmd_e command_i;
  logic [63:0] command_address_i;
  logic [12:0] command_byte_count_i;
  logic [2:0] command_tc_i;
  logic [2:0] command_attr_i;
  logic [15:0] command_context_i;
  logic command_prefix_valid_i;
  logic [31:0] command_prefix_i;
  logic command_ecrc_enable_i;
  logic [31:0] command_data_i;
  logic [3:0] command_keep_i;
  logic command_data_valid_i;
  logic command_data_last_i;
  logic command_data_ready_o;
  logic command_error_valid_o;
  tlp_error_e command_error_code_o;

  logic target_request_valid_o;
  logic target_request_ready_i;
  wire [TLP_HEADER_WIDTH-1:0] target_request_header_o;
  tlp_class_e target_request_class_o;
  logic target_memory_o;
  logic target_config_o;
  logic target_config_hit_o;
  logic target_config_type_one_o;
  logic [11:0] target_config_offset_o;
  logic target_read_o;
  logic target_write_o;
  logic target_unsupported_o;
  logic target_bar_hit_o;
  logic target_bar_overlap_o;
  logic target_bar_o;
  logic [63:0] target_offset_o;
  logic [31:0] target_data_o;
  logic [3:0] target_keep_o;
  logic target_data_valid_o;
  logic target_data_last_o;
  logic target_data_ready_i;

  logic completion_request_valid_i;
  logic completion_request_ready_o;
  logic [TLP_HEADER_WIDTH-1:0] completion_request_header_i;
  logic [2:0] completion_request_status_i;
  logic [12:0] completion_request_byte_count_i;
  logic [6:0] completion_request_lower_address_i;
  logic completion_request_ecrc_enable_i;
  logic [31:0] completion_request_data_i;
  logic [3:0] completion_request_keep_i;
  logic completion_request_data_valid_i;
  logic completion_request_data_last_i;
  logic completion_request_data_ready_o;

  logic received_completion_valid_o;
  logic received_completion_ready_i;
  wire [TLP_HEADER_WIDTH-1:0] received_completion_header_o;
  logic [31:0] received_completion_data_o;
  logic [3:0] received_completion_keep_o;
  logic received_completion_data_valid_o;
  logic received_completion_data_last_o;
  logic received_completion_data_ready_i;
  logic result_valid_o;
  logic result_ready_i;
  logic [15:0] result_context_o;
  logic [2:0] result_status_o;
  logic result_last_o;

  logic [7:0] cfg_bus_number_o;
  logic [4:0] cfg_device_number_o;
  logic [2:0] cfg_function_number_o;
  logic fc_initialized_o;
  logic fc_update_valid_o;
  logic [7:0] fc_ph_o;
  logic [11:0] fc_pd_o;
  logic [7:0] fc_nph_o;
  logic [11:0] fc_npd_o;
  logic [7:0] fc_cplh_o;
  logic [11:0] fc_cpld_o;
  logic malformed_o;
  logic rx_error_valid_o;
  tlp_error_e rx_error_code_o;
  logic rx_ecrc_error_o;
  logic tx_error_valid_o;
  tlp_error_e tx_error_code_o;
  logic tx_fc_blocked_o;
  logic credit_error_o;
  logic vc_overflow_o;
  logic unexpected_completion_o;
  tlp_error_e completion_error_code_o;
  // Completion Timeout sideband. Observed-only here -- nothing in this bench
  // asserts on them yet. Tag widths are a fixed 8 bits all the way down
  // (pcie_endpoint_top.sv:146,148 <- tlp_layer.sv:154,156 <-
  // tlp_request_tracker.sv:147,149); they are not TAG_COUNT-parameterised the
  // way outstanding_o below is.
  logic cpl_timeout_valid_o;
  logic [7:0] cpl_timeout_tag_o;
  logic late_cpl_valid_o;
  logic [7:0] late_cpl_tag_o;
  logic [5:0] outstanding_o;

  // Keep packed structs at the DUT boundary, but expose flat vectors to
  // cocotb. VCS otherwise publishes packed structs as hierarchy objects
  // without an aggregate .value handle.
  tlp_header_t completion_request_header_s;
  tlp_header_t target_request_header_s;
  tlp_header_t received_completion_header_s;

  assign completion_request_header_s =
      tlp_header_t'(completion_request_header_i);
  assign target_request_header_o = target_request_header_s;
  assign received_completion_header_o = received_completion_header_s;

  // Verification-only visibility of the boundary between the two layers.
  wire [31:0] mid_tx_axis_tdata = dut.tlp_to_dll_tdata;
  wire [3:0]  mid_tx_axis_tkeep = dut.tlp_to_dll_tkeep;
  wire        mid_tx_axis_tvalid = dut.tlp_to_dll_tvalid;
  wire        mid_tx_axis_tlast = dut.tlp_to_dll_tlast;
  wire [2:0]  mid_tx_axis_tuser = dut.tlp_to_dll_tuser;
  wire        mid_tx_axis_tready = dut.tlp_to_dll_tready;
  wire [31:0] mid_rx_axis_tdata = dut.dll_to_tlp_tdata;
  wire [3:0]  mid_rx_axis_tkeep = dut.dll_to_tlp_tkeep;
  wire        mid_rx_axis_tvalid = dut.dll_to_tlp_tvalid;
  wire        mid_rx_axis_tlast = dut.dll_to_tlp_tlast;
  wire [2:0]  mid_rx_axis_tuser = dut.dll_to_tlp_tuser;
  wire        mid_rx_axis_tready = dut.dll_to_tlp_tready;

  // Integrated Gen1 logical-PHY boundary (pcie_endpoint_top.sv:53-78), added at
  // 58965ea. The DUT binds these through `.*`, so they must exist here even
  // though this bench never exercises them: INTEGRATED_GEN1_PHY defaults to 1'b0
  // (pcie_endpoint_top.sv:30), and every one of these ports is read only inside
  // `generate if (INTEGRATED_GEN1_PHY)` at :356-630, which does not elaborate.
  //
  // Inputs are tied to a constant idle. That is unobservable, not merely
  // harmless: the arm that does elaborate, gen_packet_phy_compatibility at
  // :631-663, reads none of them -- it touches only s_phy_axis_*,
  // m_phy_axis_tready, phy_link_up_i and idle_valid_i.
  //
  // Outputs are left undriven here; that same arm drives all sixteen to
  // constants at :648-663, so they are deterministic rather than floating.
  // Nothing in this bench reads them.
  wire                              pipe_rx_usr_clk_i = '0;
  wire                              pipe_tx_usr_clk_i = '0;
  wire [(MAX_NUM_LANES*20)-1:0]     phy_rx_symbol_i = '0;
  wire [MAX_NUM_LANES-1:0]          phy_rx_symbol_valid_i = '0;
  wire [MAX_NUM_LANES-1:0]          phy_phystatus_i = '0;
  wire                              phy_phystatus_rst_i = '0;
  // Active high, so '0 reads as "not electrically idle". Unobservable for the
  // reason above; it is the brief's default idle, not a modelling claim.
  wire [MAX_NUM_LANES-1:0]          phy_rxelecidle_i = '0;
  wire [(MAX_NUM_LANES*3)-1:0]      phy_rxstatus_i = '0;

  wire [(MAX_NUM_LANES*20)-1:0]     phy_tx_symbol_o;
  wire [MAX_NUM_LANES-1:0]          phy_tx_symbol_valid_o;
  wire                              phy_txdetectrx_o;
  wire [MAX_NUM_LANES-1:0]          phy_txelecidle_o;
  wire [MAX_NUM_LANES-1:0]          phy_txcompliance_o;
  wire [MAX_NUM_LANES-1:0]          phy_rxpolarity_o;
  wire [1:0]                        phy_powerdown_o;
  wire [2:0]                        phy_rate_o;
  wire [2:0]                        phy_txmargin_o;
  wire                              phy_txswing_o;
  wire                              phy_txdeemph_o;
  wire [5:0]                        phy_pipe_width_o;
  wire                              phy_link_up_o;
  wire [19:0]                       ltssm_state_o;
  wire [MAX_NUM_LANES-1:0]          phy_rx_code_error_o;
  wire [MAX_NUM_LANES-1:0]          phy_rx_disparity_error_o;
  // Rung 8 / F1: the DUT is connected with .*, so a new port needs its wire here.
  wire [MAX_NUM_LANES-1:0]          phy_tx_illegal_k_o;

  pcie_endpoint_top #(
      .DATA_WIDTH(DATA_WIDTH),
      .KEEP_WIDTH(KEEP_WIDTH),
      .USER_WIDTH(USER_WIDTH),
      .CONTEXT_WIDTH(CONTEXT_WIDTH),
      .BAR_COUNT(1),
      .BAR_BASE(64'h0000_0000_0000_0000),
      .BAR_MASK(64'hffff_ffff_ffff_f000),
      .BAR_ENABLE(1'b1),
      .REPLAY_TIMER_CYCLES(64),
      .MAX_REPLAY_ATTEMPTS(2)
  ) dut (
      .completion_request_header_i(completion_request_header_s),
      .target_request_header_o(target_request_header_s),
      .received_completion_header_o(received_completion_header_s),
      .*
  );

endmodule
