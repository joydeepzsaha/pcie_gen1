`timescale 1ns/1ps

// PCIe endpoint top level.  The packet PHY interface remains available for
// protocol-level verification.  INTEGRATED_GEN1_PHY adds the existing LTSSM,
// logical PHY, Gen1 scrambler and 8b/10b codec below the single Data Link
// Layer, leaving only a vendor-independent symbol/transceiver-control boundary.
module pcie_endpoint_top
  import tlp_pkg::*;
  import pcie_phy_pkg::*;
#(
    parameter int DATA_WIDTH = 32,
    parameter int KEEP_WIDTH = DATA_WIDTH / 8,
    parameter int USER_WIDTH = 3,
    parameter int TAG_COUNT = 32,
    parameter int CONTEXT_WIDTH = 16,
    // Completion Timeout; 0 disables. See tlp_request_tracker.sv header.
    parameter int unsigned CPL_TIMEOUT_CYCLES = 32'd4096,
    parameter int VC_PACKET_DEPTH = 4,
    parameter int BAR_COUNT = 2,
    parameter logic [BAR_COUNT*64-1:0] BAR_BASE = '0,
    parameter logic [BAR_COUNT*64-1:0] BAR_MASK =
        {{(BAR_COUNT-1){64'd0}}, 64'hffff_ffff_ffff_f000},
    parameter logic [BAR_COUNT-1:0] BAR_ENABLE =
        {{(BAR_COUNT-1){1'b0}}, 1'b1},
    parameter int RX_FIFO_SIZE = 3,
    parameter int RETRY_TLP_SIZE = 3,
    parameter int MAX_PAYLOAD_SIZE = 256,
    parameter int REPLAY_TIMER_CYCLES = 16'h0aa0,
    parameter int MAX_REPLAY_ATTEMPTS = 2,
    parameter bit INTEGRATED_GEN1_PHY = 1'b0,
    parameter int PHY_CLK_RATE = 125,
    parameter int MAX_NUM_LANES = 1,
    // Shorten LTSSM training in simulation only.  Forwarded to the integrated
    // pcie_ltssm_downstream below, which scales TwelveMsTimeOut (:111),
    // OneMsTimeOut (:114) and MinTS1sPolling (:121, 1024 -> 24).  Note that
    // TwentyFourMsTimeOut, TwoMsTimeOut and FourtyEightMsTimeOut are NOT
    // scaled, so a bench needing those still pays full price.
    //
    // Default 0 keeps the previous hardcoded behaviour exactly.  It is exposed
    // because a cross-wired RC<->EP bench trains this LTSSM against real 12 ms
    // timers otherwise -- 1.5 M cycles at 8 ns for Detect alone -- which is what
    // made such a bench unaffordable rather than unreachable.
    parameter int SIM_FAST_LINK = 0
) (
    input  logic                     clk_i,
    input  logic                     rst_i,
    input  logic                     phy_link_up_i,
    input  logic                     idle_valid_i,
    input  logic                     transmit_enable_i,

    input  logic [DATA_WIDTH-1:0]    s_phy_axis_tdata,
    input  logic [KEEP_WIDTH-1:0]    s_phy_axis_tkeep,
    input  logic                     s_phy_axis_tvalid,
    input  logic                     s_phy_axis_tlast,
    input  logic [USER_WIDTH-1:0]    s_phy_axis_tuser,
    output logic                     s_phy_axis_tready,
    output logic [DATA_WIDTH-1:0]    m_phy_axis_tdata,
    output logic [KEEP_WIDTH-1:0]    m_phy_axis_tkeep,
    output logic                     m_phy_axis_tvalid,
    output logic                     m_phy_axis_tlast,
    output logic [USER_WIDTH-1:0]    m_phy_axis_tuser,
    input  logic                     m_phy_axis_tready,

    // Integrated Gen1 logical-PHY boundary.  Each lane transfers two 10-bit
    // symbols per 125 MHz user-clock cycle (the existing 16-bit PIPE width).
    input  logic                     pipe_rx_usr_clk_i,
    input  logic                     pipe_tx_usr_clk_i,
    input  logic [(MAX_NUM_LANES*20)-1:0] phy_rx_symbol_i,
    input  logic [MAX_NUM_LANES-1:0] phy_rx_symbol_valid_i,
    output logic [(MAX_NUM_LANES*20)-1:0] phy_tx_symbol_o,
    output logic [MAX_NUM_LANES-1:0] phy_tx_symbol_valid_o,
    input  logic [MAX_NUM_LANES-1:0] phy_phystatus_i,
    input  logic                     phy_phystatus_rst_i,
    input  logic [MAX_NUM_LANES-1:0] phy_rxelecidle_i,
    input  logic [(MAX_NUM_LANES*3)-1:0] phy_rxstatus_i,
    output logic                     phy_txdetectrx_o,
    output logic [MAX_NUM_LANES-1:0] phy_txelecidle_o,
    output logic [MAX_NUM_LANES-1:0] phy_txcompliance_o,
    output logic [MAX_NUM_LANES-1:0] phy_rxpolarity_o,
    output logic [1:0]               phy_powerdown_o,
    output logic [2:0]               phy_rate_o,
    output logic [2:0]               phy_txmargin_o,
    output logic                     phy_txswing_o,
    output logic                     phy_txdeemph_o,
    output logic [5:0]               phy_pipe_width_o,
    output logic                     phy_link_up_o,
    output logic [19:0]              ltssm_state_o,
    output logic [MAX_NUM_LANES-1:0] phy_rx_code_error_o,
    output logic [MAX_NUM_LANES-1:0] phy_rx_disparity_error_o,
    // Transmit-side dual of phy_rx_code_error_o: the encoder was asked for a K
    // code-group Base 2.1 Appendix B does not define.  Same shape and same
    // valid-gating as the RX pair below.
    output logic [MAX_NUM_LANES-1:0] phy_tx_illegal_k_o,

    input  logic                     memory_enable_i,
    input  logic                     extended_tag_enable_i,
    input  logic [12:0]              max_payload_bytes_i,
    input  logic [12:0]              max_read_bytes_i,
    input  logic                     rcb_128b_i,

    input  logic                     command_valid_i,
    output logic                     command_ready_o,
    input  tlp_cmd_e                 command_i,
    input  logic [63:0]              command_address_i,
    input  logic [12:0]              command_byte_count_i,
    input  logic [2:0]               command_tc_i,
    input  logic [2:0]               command_attr_i,
    input  logic [CONTEXT_WIDTH-1:0] command_context_i,
    input  logic                     command_prefix_valid_i,
    input  logic [31:0]              command_prefix_i,
    input  logic                     command_ecrc_enable_i,
    input  logic [DATA_WIDTH-1:0]    command_data_i,
    input  logic [KEEP_WIDTH-1:0]    command_keep_i,
    input  logic                     command_data_valid_i,
    input  logic                     command_data_last_i,
    output logic                     command_data_ready_o,
    output logic                     command_error_valid_o,
    output tlp_error_e               command_error_code_o,

    output logic                     target_request_valid_o,
    input  logic                     target_request_ready_i,
    output tlp_header_t              target_request_header_o,
    output tlp_class_e               target_request_class_o,
    output logic                     target_memory_o,
    output logic                     target_config_o,
    output logic                     target_config_hit_o,
    output logic                     target_config_type_one_o,
    output logic [11:0]              target_config_offset_o,
    output logic                     target_read_o,
    output logic                     target_write_o,
    output logic                     target_unsupported_o,
    output logic                     target_bar_hit_o,
    output logic                     target_bar_overlap_o,
    output logic [((BAR_COUNT <= 1) ? 1 : $clog2(BAR_COUNT))-1:0] target_bar_o,
    output logic [63:0]              target_offset_o,
    output logic [DATA_WIDTH-1:0]    target_data_o,
    output logic [KEEP_WIDTH-1:0]    target_keep_o,
    output logic                     target_data_valid_o,
    output logic                     target_data_last_o,
    input  logic                     target_data_ready_i,

    input  logic                     completion_request_valid_i,
    output logic                     completion_request_ready_o,
    input  tlp_header_t              completion_request_header_i,
    input  logic [2:0]               completion_request_status_i,
    input  logic [12:0]              completion_request_byte_count_i,
    input  logic [6:0]               completion_request_lower_address_i,
    input  logic                     completion_request_ecrc_enable_i,
    input  logic [DATA_WIDTH-1:0]    completion_request_data_i,
    input  logic [KEEP_WIDTH-1:0]    completion_request_keep_i,
    input  logic                     completion_request_data_valid_i,
    input  logic                     completion_request_data_last_i,
    output logic                     completion_request_data_ready_o,

    output logic                     received_completion_valid_o,
    input  logic                     received_completion_ready_i,
    output tlp_header_t              received_completion_header_o,
    output logic [DATA_WIDTH-1:0]    received_completion_data_o,
    output logic [KEEP_WIDTH-1:0]    received_completion_keep_o,
    output logic                     received_completion_data_valid_o,
    output logic                     received_completion_data_last_o,
    input  logic                     received_completion_data_ready_i,

    output logic                     result_valid_o,
    input  logic                     result_ready_i,
    output logic [CONTEXT_WIDTH-1:0] result_context_o,
    output logic [2:0]               result_status_o,
    output logic                     result_last_o,

    output logic [7:0]               cfg_bus_number_o,
    output logic [4:0]               cfg_device_number_o,
    output logic [2:0]               cfg_function_number_o,
    output logic                     fc_initialized_o,
    output logic                     fc_update_valid_o,
    output logic [7:0]               fc_ph_o,
    output logic [11:0]              fc_pd_o,
    output logic [7:0]               fc_nph_o,
    output logic [11:0]              fc_npd_o,
    output logic [7:0]               fc_cplh_o,
    output logic [11:0]              fc_cpld_o,
    output logic                     malformed_o,
    output logic                     rx_error_valid_o,
    output tlp_error_e               rx_error_code_o,
    output logic                     rx_ecrc_error_o,
    output logic                     tx_error_valid_o,
    output tlp_error_e               tx_error_code_o,
    output logic                     tx_fc_blocked_o,
    output logic                     credit_error_o,
    output logic                     vc_overflow_o,
    output logic                     unexpected_completion_o,
    output tlp_error_e               completion_error_code_o,
    // Completion Timeout sideband. See tlp_request_tracker.sv header.
    output logic                     cpl_timeout_valid_o,
    output logic [7:0]               cpl_timeout_tag_o,
    output logic                     late_cpl_valid_o,
    output logic [7:0]               late_cpl_tag_o,
    output logic [$clog2(TAG_COUNT+1)-1:0] outstanding_o
);

  logic [DATA_WIDTH-1:0] tlp_to_dll_tdata;
  logic [KEEP_WIDTH-1:0] tlp_to_dll_tkeep;
  logic                  tlp_to_dll_tvalid;
  logic                  tlp_to_dll_tlast;
  logic [USER_WIDTH-1:0] tlp_to_dll_tuser;
  logic                  tlp_to_dll_tready;

  logic [DATA_WIDTH-1:0] dll_to_tlp_tdata;
  logic [KEEP_WIDTH-1:0] dll_to_tlp_tkeep;
  logic                  dll_to_tlp_tvalid;
  logic                  dll_to_tlp_tlast;
  logic [USER_WIDTH-1:0] dll_to_tlp_tuser;
  logic                  dll_to_tlp_tready;

  logic [15:0] function_id;

  logic [DATA_WIDTH-1:0] dll_phy_rx_tdata;
  logic [KEEP_WIDTH-1:0] dll_phy_rx_tkeep;
  logic                  dll_phy_rx_tvalid;
  logic                  dll_phy_rx_tlast;
  logic [USER_WIDTH-1:0] dll_phy_rx_tuser;
  logic                  dll_phy_rx_tready;

  logic [DATA_WIDTH-1:0] dll_phy_tx_tdata;
  logic [KEEP_WIDTH-1:0] dll_phy_tx_tkeep;
  logic                  dll_phy_tx_tvalid;
  logic                  dll_phy_tx_tlast;
  logic [USER_WIDTH-1:0] dll_phy_tx_tuser;
  logic                  dll_phy_tx_tready;

  logic protocol_link_up;
  logic protocol_idle_valid;

  assign function_id = {
      cfg_bus_number_o, cfg_device_number_o, cfg_function_number_o
  };

  tlp_layer #(
      .DATA_WIDTH(DATA_WIDTH),
      .KEEP_WIDTH(KEEP_WIDTH),
      .USER_WIDTH(USER_WIDTH),
      .TAG_COUNT(TAG_COUNT),
      .CONTEXT_WIDTH(CONTEXT_WIDTH),
      .CPL_TIMEOUT_CYCLES(CPL_TIMEOUT_CYCLES),
      .VC_PACKET_DEPTH(VC_PACKET_DEPTH),
      .PCIE_WIRE_ORDER(1'b1),
      .BAR_COUNT(BAR_COUNT),
      .BAR_BASE(BAR_BASE),
      .BAR_MASK(BAR_MASK),
      .BAR_ENABLE(BAR_ENABLE)
  ) tlp_layer_inst (
      .clk_i(clk_i),
      .rst_i(rst_i),
      .link_up_i(protocol_link_up),
      .transmit_enable_i(transmit_enable_i),
      .requester_id_i(function_id),
      .completer_id_i(function_id),
      .bus_number_i(cfg_bus_number_o),
      .device_number_i(cfg_device_number_o),
      .function_number_i(cfg_function_number_o),
      .memory_enable_i(memory_enable_i),
      .extended_tag_enable_i(extended_tag_enable_i),
      .max_payload_bytes_i(max_payload_bytes_i),
      .max_read_bytes_i(max_read_bytes_i),
      .rcb_128b_i(rcb_128b_i),
      .fc_initialized_i(fc_initialized_o),
      .fc_update_valid_i(fc_update_valid_o),
      .fc_ph_i(fc_ph_o),
      .fc_pd_i(fc_pd_o),
      .fc_nph_i(fc_nph_o),
      .fc_npd_i(fc_npd_o),
      .fc_cplh_i(fc_cplh_o),
      .fc_cpld_i(fc_cpld_o),
      .s_dllp_axis_tdata(dll_to_tlp_tdata),
      .s_dllp_axis_tkeep(dll_to_tlp_tkeep),
      .s_dllp_axis_tvalid(dll_to_tlp_tvalid),
      .s_dllp_axis_tlast(dll_to_tlp_tlast),
      .s_dllp_axis_tuser(dll_to_tlp_tuser),
      .s_dllp_axis_tready(dll_to_tlp_tready),
      .m_dllp_axis_tdata(tlp_to_dll_tdata),
      .m_dllp_axis_tkeep(tlp_to_dll_tkeep),
      .m_dllp_axis_tvalid(tlp_to_dll_tvalid),
      .m_dllp_axis_tlast(tlp_to_dll_tlast),
      .m_dllp_axis_tuser(tlp_to_dll_tuser),
      .m_dllp_axis_tready(tlp_to_dll_tready),
      .command_valid_i(command_valid_i),
      .command_ready_o(command_ready_o),
      .command_i(command_i),
      .command_address_i(command_address_i),
      .command_byte_count_i(command_byte_count_i),
      .command_tc_i(command_tc_i),
      .command_attr_i(command_attr_i),
      .command_context_i(command_context_i),
      .command_prefix_valid_i(command_prefix_valid_i),
      .command_prefix_i(command_prefix_i),
      .command_ecrc_enable_i(command_ecrc_enable_i),
      .command_data_i(command_data_i),
      .command_keep_i(command_keep_i),
      .command_data_valid_i(command_data_valid_i),
      .command_data_last_i(command_data_last_i),
      .command_data_ready_o(command_data_ready_o),
      .command_error_valid_o(command_error_valid_o),
      .command_error_code_o(command_error_code_o),
      // RQ-side tag strobe. Deliberately unused at this top: the tag is
      // consumed by pcie_rq_if on the RC path, not here. Named-empty rather
      // than omitted so PINMISSING stays enabled for real omissions.
      .allocated_tag_o(),
      .allocated_tag_valid_o(),
      .target_request_valid_o(target_request_valid_o),
      .target_request_ready_i(target_request_ready_i),
      .target_request_header_o(target_request_header_o),
      .target_request_class_o(target_request_class_o),
      .target_memory_o(target_memory_o),
      .target_config_o(target_config_o),
      .target_config_hit_o(target_config_hit_o),
      .target_config_type_one_o(target_config_type_one_o),
      .target_config_offset_o(target_config_offset_o),
      .target_read_o(target_read_o),
      .target_write_o(target_write_o),
      .target_unsupported_o(target_unsupported_o),
      .target_bar_hit_o(target_bar_hit_o),
      .target_bar_overlap_o(target_bar_overlap_o),
      .target_bar_o(target_bar_o),
      .target_offset_o(target_offset_o),
      .target_data_o(target_data_o),
      .target_keep_o(target_keep_o),
      .target_data_valid_o(target_data_valid_o),
      .target_data_last_o(target_data_last_o),
      .target_data_ready_i(target_data_ready_i),
      .completion_request_valid_i(completion_request_valid_i),
      .completion_request_ready_o(completion_request_ready_o),
      .completion_request_header_i(completion_request_header_i),
      .completion_request_status_i(completion_request_status_i),
      .completion_request_byte_count_i(completion_request_byte_count_i),
      .completion_request_lower_address_i(completion_request_lower_address_i),
      .completion_request_ecrc_enable_i(completion_request_ecrc_enable_i),
      .completion_request_data_i(completion_request_data_i),
      .completion_request_keep_i(completion_request_keep_i),
      .completion_request_data_valid_i(completion_request_data_valid_i),
      .completion_request_data_last_i(completion_request_data_last_i),
      .completion_request_data_ready_o(completion_request_data_ready_o),
      .received_completion_valid_o(received_completion_valid_o),
      .received_completion_ready_i(received_completion_ready_i),
      .received_completion_header_o(received_completion_header_o),
      .received_completion_data_o(received_completion_data_o),
      .received_completion_keep_o(received_completion_keep_o),
      .received_completion_data_valid_o(received_completion_data_valid_o),
      .received_completion_data_last_o(received_completion_data_last_o),
      .received_completion_data_ready_i(received_completion_data_ready_i),
      .result_valid_o(result_valid_o),
      .result_ready_i(result_ready_i),
      .result_context_o(result_context_o),
      .result_status_o(result_status_o),
      .result_last_o(result_last_o),
      .malformed_o(malformed_o),
      .rx_error_valid_o(rx_error_valid_o),
      .rx_error_code_o(rx_error_code_o),
      .rx_ecrc_error_o(rx_ecrc_error_o),
      .tx_error_valid_o(tx_error_valid_o),
      .tx_error_code_o(tx_error_code_o),
      .tx_fc_blocked_o(tx_fc_blocked_o),
      .credit_error_o(credit_error_o),
      .vc_overflow_o(vc_overflow_o),
      .unexpected_completion_o(unexpected_completion_o),
      .completion_error_code_o(completion_error_code_o),
      .cpl_timeout_valid_o(cpl_timeout_valid_o), .cpl_timeout_tag_o(cpl_timeout_tag_o),
      .late_cpl_valid_o(late_cpl_valid_o), .late_cpl_tag_o(late_cpl_tag_o),
      .outstanding_o(outstanding_o)
  );

  generate
    if (INTEGRATED_GEN1_PHY) begin : gen_integrated_gen1_phy
      logic [(MAX_NUM_LANES*DATA_WIDTH)-1:0] phy_txdata;
      logic [MAX_NUM_LANES-1:0]              phy_txdata_valid;
      logic [(4*MAX_NUM_LANES)-1:0]          phy_txdatak;
      logic [(2*MAX_NUM_LANES)-1:0]          phy_txsync_header;
      logic [MAX_NUM_LANES-1:0]              phy_txstart_block;

      logic [(MAX_NUM_LANES*DATA_WIDTH)-1:0] phy_rxdata;
      logic [(4*MAX_NUM_LANES)-1:0]          phy_rxdatak;
      logic [(2*MAX_NUM_LANES)-1:0]          phy_rxsync_header;
      logic [MAX_NUM_LANES-1:0]              phy_rxstart_block;

      logic [MAX_NUM_LANES-1:0]              tx_running_disparity;
      logic [MAX_NUM_LANES-1:0]              rx_running_disparity;
      logic [MAX_NUM_LANES-1:0]              lane_status;
      logic [MAX_NUM_LANES-1:0]              active_lanes;
      logic [5:0]                            num_active_lanes;
      logic [5:0]                            pipe_width;
      logic                                  link_up_rx;
      logic [1:0]                            link_up_sync;
      logic [1:0]                            idle_valid_sync;
      logic                                  txdetectrx_d;
      logic                                  txdetectrx_rise;
      logic [MAX_NUM_LANES-1:0]              ts1_valid;
      logic [MAX_NUM_LANES-1:0]              ts2_valid;
      logic [MAX_NUM_LANES-1:0]              idle_valid;
      logic [MAX_NUM_LANES-1:0]              polarity_inverted;
      logic                                  ordered_set_transmitted;
      logic                                  send_ordered_set;
      logic                                  ltssm_txcompliance;
      rate_speed_e                           current_data_rate;
      pcie_ordered_set_t [MAX_NUM_LANES-1:0] rx_ordered_set;
      pcie_ordered_set_t [MAX_NUM_LANES-1:0] tx_ordered_set;
      gen_os_struct_t                        gen_os_control;

      // The compatibility packet ports are deliberately quiescent while the
      // integrated logical PHY owns the Data Link Layer's physical AXI stream.
      assign s_phy_axis_tready = 1'b0;
      assign m_phy_axis_tdata  = '0;
      assign m_phy_axis_tkeep  = '0;
      assign m_phy_axis_tvalid = 1'b0;
      assign m_phy_axis_tlast  = 1'b0;
      assign m_phy_axis_tuser  = '0;

      assign phy_tx_symbol_valid_o = phy_txdata_valid;
      assign phy_rxsync_header     = '0;
      assign phy_rxstart_block     = '0;
      assign phy_pipe_width_o      = pipe_width;
      assign phy_link_up_o         = link_up_rx;
      assign phy_rate_o            = current_data_rate - 1'b1;
      assign phy_txswing_o         = 1'b0;
      assign phy_txcompliance_o    = {MAX_NUM_LANES{ltssm_txcompliance}};

      assign protocol_link_up   = link_up_sync[1];
      assign protocol_idle_valid = idle_valid_sync[1];

      // Synchronize the LTSSM indications consumed by the core-clock TLP/DLL
      // logic.  The packet datapaths themselves cross through the PHY FIFOs.
      always_ff @(posedge clk_i) begin
        if (rst_i) begin
          link_up_sync    <= '0;
          idle_valid_sync <= '0;
        end else begin
          link_up_sync    <= {link_up_sync[0], link_up_rx};
          idle_valid_sync <= {idle_valid_sync[0], |idle_valid};
        end
      end

      always_ff @(posedge pipe_rx_usr_clk_i) begin
        if (rst_i || phy_phystatus_rst_i) begin
          txdetectrx_d    <= 1'b0;
          lane_status     <= '0;
          num_active_lanes <= '0;
        end else begin
          txdetectrx_d <= phy_txdetectrx_o;
          for (int lane = 0; lane < MAX_NUM_LANES; lane++) begin
            if (phy_phystatus_i[lane] &&
                phy_rxstatus_i[lane*3 +: 3] == 3'b011) begin
              lane_status[lane] <= 1'b1;
            end
            if (lane_status[lane]) begin
              num_active_lanes <= lane + 1;
            end
          end
          if (txdetectrx_rise) begin
            lane_status      <= '0;
            num_active_lanes <= '0;
          end
        end
      end

      assign txdetectrx_rise = phy_txdetectrx_o && !txdetectrx_d;

      // The existing codec is combinational.  Running disparity is retained
      // once per lane and chained across both symbols in each Gen1 PIPE word.
      for (genvar lane = 0; lane < MAX_NUM_LANES; lane++) begin : gen_8b10b_lane
        logic [2:0] tx_disparity;
        logic [2:0] rx_disparity;
        logic [1:0] rx_code_error;
        logic [1:0] rx_disparity_error;
        logic [1:0] tx_illegal_k;

        assign tx_disparity[0] = tx_running_disparity[lane];
        assign rx_disparity[0] = rx_running_disparity[lane];
        assign phy_rxdata[lane*DATA_WIDTH+16 +: 16] = '0;
        assign phy_rxdatak[lane*4+2 +: 2] = '0;

        for (genvar symbol = 0; symbol < 2; symbol++) begin : gen_8b10b_symbol
          encode_8b10b tx_encoder_inst (
              .datain ({phy_txdatak[lane*4+symbol],
                        phy_txdata[lane*DATA_WIDTH+symbol*8 +: 8]}),
              .dispin (tx_disparity[symbol]),
              .dataout(phy_tx_symbol_o[lane*20+symbol*10 +: 10]),
              .dispout(tx_disparity[symbol+1]),
              .illegal_k_o(tx_illegal_k[symbol])
          );

          decode_8b10b rx_decoder_inst (
              .datain  (phy_rx_symbol_i[lane*20+symbol*10 +: 10]),
              .dispin  (rx_disparity[symbol]),
              .dataout ({phy_rxdatak[lane*4+symbol],
                         phy_rxdata[lane*DATA_WIDTH+symbol*8 +: 8]}),
              .dispout (rx_disparity[symbol+1]),
              .code_err(rx_code_error[symbol]),
              .disp_err(rx_disparity_error[symbol])
          );
        end

        always_ff @(posedge pipe_tx_usr_clk_i) begin
          if (rst_i || phy_phystatus_rst_i) begin
            tx_running_disparity[lane] <= 1'b0;
          end else if (phy_txdata_valid[lane]) begin
            // Gen1 lane_management uses a 16-bit PIPE width.
            tx_running_disparity[lane] <= tx_disparity[2];
          end
        end

        always_ff @(posedge pipe_rx_usr_clk_i) begin
          if (rst_i || phy_phystatus_rst_i) begin
            rx_running_disparity[lane] <= 1'b0;
          end else if (phy_rx_symbol_valid_i[lane]) begin
            rx_running_disparity[lane] <= rx_disparity[2];
          end
        end

        assign phy_rx_code_error_o[lane] =
            phy_rx_symbol_valid_i[lane] && |rx_code_error;
        assign phy_rx_disparity_error_o[lane] =
            phy_rx_symbol_valid_i[lane] && |rx_disparity_error;
        assign phy_tx_illegal_k_o[lane] =
            phy_txdata_valid[lane] && |tx_illegal_k;
      end

      phy_receive #(
          .CLK_RATE(PHY_CLK_RATE),
          .MAX_NUM_LANES(MAX_NUM_LANES),
          .DATA_WIDTH(DATA_WIDTH),
          .STRB_WIDTH(KEEP_WIDTH),
          .KEEP_WIDTH(KEEP_WIDTH),
          .USER_WIDTH(USER_WIDTH)
      ) phy_receive_inst (
          .clk_i(clk_i),
          .rst_i(rst_i || phy_phystatus_rst_i),
          .en_i(1'b1),
          .link_up_i(link_up_rx),
          .pipe_rx_usr_clk_i(pipe_rx_usr_clk_i),
          .pipe_data_i(phy_rxdata),
          .pipe_data_valid_i(phy_rx_symbol_valid_i),
          .pipe_data_k_i(phy_rxdatak),
          .pipe_sync_header_i(phy_rxsync_header),
          .pipe_block_start_i(phy_rxstart_block),
          .pipe_width_i(pipe_width),
          .num_active_lanes_i(num_active_lanes),
          .ordered_set_o(rx_ordered_set),
          .ts1_valid_o(ts1_valid),
          .ts2_valid_o(ts2_valid),
          .idle_valid_o(idle_valid),
          .polarity_inverted_o(polarity_inverted),
          .curr_data_rate_i(current_data_rate),
          .m_dllp_axis_tdata(dll_phy_rx_tdata),
          .m_dllp_axis_tkeep(dll_phy_rx_tkeep),
          .m_dllp_axis_tvalid(dll_phy_rx_tvalid),
          .m_dllp_axis_tlast(dll_phy_rx_tlast),
          .m_dllp_axis_tuser(dll_phy_rx_tuser),
          .m_dllp_axis_tready(dll_phy_rx_tready)
      );

      phy_transmit #(
          .CLK_RATE(PHY_CLK_RATE),
          .MAX_NUM_LANES(MAX_NUM_LANES),
          .DATA_WIDTH(DATA_WIDTH),
          .STRB_WIDTH(KEEP_WIDTH),
          .KEEP_WIDTH(KEEP_WIDTH),
          .USER_WIDTH(USER_WIDTH)
      ) phy_transmit_inst (
          .clk_i(clk_i),
          .pipe_rx_usr_clk_i(pipe_rx_usr_clk_i),
          .pipe_tx_usr_clk_i(pipe_tx_usr_clk_i),
          .rst_i(rst_i || phy_phystatus_rst_i),
          .en_i(1'b1),
          .link_up_i(link_up_rx),
          .pipe_data_o(phy_txdata),
          .pipe_data_valid_o(phy_txdata_valid),
          .pipe_data_k_o(phy_txdatak),
          .pipe_sync_header_o(phy_txsync_header),
          .pipe_txstart_block_o(phy_txstart_block),
          .pipe_width_o(pipe_width),
          .num_active_lanes_i(num_active_lanes),
          .send_ordered_set_i(send_ordered_set),
          .ordered_set_i(tx_ordered_set),
          .curr_data_rate_i(current_data_rate),
          .ordered_set_tranmitted_o(ordered_set_transmitted),
          .gen_os_ctrl_i(gen_os_control),
          .s_dllp_axis_tdata(dll_phy_tx_tdata),
          .s_dllp_axis_tkeep(dll_phy_tx_tkeep),
          .s_dllp_axis_tvalid(dll_phy_tx_tvalid),
          .s_dllp_axis_tlast(dll_phy_tx_tlast),
          .s_dllp_axis_tuser(dll_phy_tx_tuser),
          .s_dllp_axis_tready(dll_phy_tx_tready)
      );

      pcie_ltssm_downstream #(
          .CLK_RATE(PHY_CLK_RATE),
          .MAX_NUM_LANES(MAX_NUM_LANES),
          .DATA_WIDTH(DATA_WIDTH),
          .KEEP_WIDTH(KEEP_WIDTH),
          .USER_WIDTH(USER_WIDTH),
          .SIM_FAST_LINK(SIM_FAST_LINK),
          .IS_ROOT_PORT(1'b0),
          .IS_UPSTREAM(1'b1),
          .MAX_SUPPORTED_RATE(gen1)
      ) endpoint_ltssm_inst (
          .clk_i(pipe_rx_usr_clk_i),
          .rst_i(rst_i || phy_phystatus_rst_i),
          .en_i(1'b1),
          .link_up_o(link_up_rx),
          .is_timeout_i(1'b0),
          .recovery_i(1'b0),
          .error_o(),
          .success_o(),
          .error_loopback_o(),
          .error_disable_o(),
          .ts1_valid_i(ts1_valid),
          .ts2_valid_i(ts2_valid),
          .idle_valid_i(idle_valid),
          .polarity_inverted_i(polarity_inverted),
          .phy_rxstatus_i(phy_rxstatus_i),
          .phy_phystatus_i(phy_phystatus_i),
          .phy_phystatus_rst_i(phy_phystatus_rst_i),
          .phy_txdetectrx_o(phy_txdetectrx_o),
          .phy_txelecidle_o(phy_txelecidle_o),
          .phy_txdeemph_o(phy_txdeemph_o),
          .phy_powerdown_o(phy_powerdown_o),
          .phy_txcompliance_o(ltssm_txcompliance),
          .phy_rxpolarity_o(phy_rxpolarity_o),
          .phy_txmargin_o(phy_txmargin_o),
          .lanes_ts2_satisfied_i('0),
          .config_copmlete_ts2_i('0),
          .from_l0_i(1'b0),
          .receiver_detected_i(lane_status),
          .phy_rxelecidle_i(phy_rxelecidle_i),
          .tx_enter_elec_idle_o(),
          .ltssm_state_o(ltssm_state_o),
          .goto_cfg_o(),
          .goto_detect_o(),
          .ordered_set_tranmitted_i(ordered_set_transmitted),
          .send_ordered_set_o(send_ordered_set),
          .active_lanes_o(active_lanes),
          .gen_os_ctrl_o(gen_os_control),
          .ordered_set_i(rx_ordered_set),
          .preset_coeff_o(),
          .ordered_set_o(tx_ordered_set),
          .extended_synch_i(1'b0),
          .directed_speed_change_i(1'b0),
          .lane_status_i(lane_status),
          .curr_data_rate_o(current_data_rate),
          .data_rate_o(),
          .changed_speed_recovery_o()
      );
    end else begin : gen_packet_phy_compatibility
      assign dll_phy_rx_tdata  = s_phy_axis_tdata;
      assign dll_phy_rx_tkeep  = s_phy_axis_tkeep;
      assign dll_phy_rx_tvalid = s_phy_axis_tvalid;
      assign dll_phy_rx_tlast  = s_phy_axis_tlast;
      assign dll_phy_rx_tuser  = s_phy_axis_tuser;
      assign s_phy_axis_tready = dll_phy_rx_tready;

      assign m_phy_axis_tdata  = dll_phy_tx_tdata;
      assign m_phy_axis_tkeep  = dll_phy_tx_tkeep;
      assign m_phy_axis_tvalid = dll_phy_tx_tvalid;
      assign m_phy_axis_tlast  = dll_phy_tx_tlast;
      assign m_phy_axis_tuser  = dll_phy_tx_tuser;
      assign dll_phy_tx_tready = m_phy_axis_tready;

      assign protocol_link_up                = phy_link_up_i;
      assign protocol_idle_valid             = idle_valid_i;
      assign phy_tx_symbol_o                 = '0;
      assign phy_tx_symbol_valid_o           = '0;
      assign phy_txdetectrx_o                = 1'b0;
      assign phy_txelecidle_o                = '1;
      assign phy_txcompliance_o              = '0;
      assign phy_rxpolarity_o                = '0;
      assign phy_powerdown_o                 = 2'b11;
      assign phy_rate_o                      = 3'b000;
      assign phy_txmargin_o                  = '0;
      assign phy_txswing_o                   = 1'b0;
      assign phy_txdeemph_o                  = 1'b0;
      assign phy_pipe_width_o                = 6'd16;
      assign phy_link_up_o                   = phy_link_up_i;
      assign ltssm_state_o                   = '0;
      assign phy_rx_code_error_o             = '0;
      assign phy_rx_disparity_error_o        = '0;
      assign phy_tx_illegal_k_o              = '0;
    end
  endgenerate

  pcie_datalink_layer #(
      .DATA_WIDTH(DATA_WIDTH),
      .STRB_WIDTH(KEEP_WIDTH),
      .KEEP_WIDTH(KEEP_WIDTH),
      .USER_WIDTH(USER_WIDTH),
      .RX_FIFO_SIZE(RX_FIFO_SIZE),
      .RETRY_TLP_SIZE(RETRY_TLP_SIZE),
      .MAX_PAYLOAD_SIZE(MAX_PAYLOAD_SIZE),
      .REPLAY_TIMER_CYCLES(REPLAY_TIMER_CYCLES),
      .MAX_REPLAY_ATTEMPTS(MAX_REPLAY_ATTEMPTS)
  ) datalink_layer_inst (
      .clk_i(clk_i),
      .rst_i(rst_i),
      .s_tlp_axis_tdata(tlp_to_dll_tdata),
      .s_tlp_axis_tkeep(tlp_to_dll_tkeep),
      .s_tlp_axis_tvalid(tlp_to_dll_tvalid),
      .s_tlp_axis_tlast(tlp_to_dll_tlast),
      .s_tlp_axis_tuser(tlp_to_dll_tuser),
      .s_tlp_axis_tready(tlp_to_dll_tready),
      .m_tlp_axis_tdata(dll_to_tlp_tdata),
      .m_tlp_axis_tkeep(dll_to_tlp_tkeep),
      .m_tlp_axis_tvalid(dll_to_tlp_tvalid),
      .m_tlp_axis_tlast(dll_to_tlp_tlast),
      .m_tlp_axis_tuser(dll_to_tlp_tuser),
      .m_tlp_axis_tready(dll_to_tlp_tready),
      .s_phy_axis_tdata(dll_phy_rx_tdata),
      .s_phy_axis_tkeep(dll_phy_rx_tkeep),
      .s_phy_axis_tvalid(dll_phy_rx_tvalid),
      .s_phy_axis_tlast(dll_phy_rx_tlast),
      .s_phy_axis_tuser(dll_phy_rx_tuser),
      .s_phy_axis_tready(dll_phy_rx_tready),
      .m_phy_axis_tdata(dll_phy_tx_tdata),
      .m_phy_axis_tkeep(dll_phy_tx_tkeep),
      .m_phy_axis_tvalid(dll_phy_tx_tvalid),
      .m_phy_axis_tlast(dll_phy_tx_tlast),
      .m_phy_axis_tuser(dll_phy_tx_tuser),
      .m_phy_axis_tready(dll_phy_tx_tready),
      .phy_link_up_i(protocol_link_up),
      .fc_initialized_o(fc_initialized_o),
      .fc_update_valid_o(fc_update_valid_o),
      .fc_ph_o(fc_ph_o),
      .fc_pd_o(fc_pd_o),
      .fc_nph_o(fc_nph_o),
      .fc_npd_o(fc_npd_o),
      .fc_cplh_o(fc_cplh_o),
      .fc_cpld_o(fc_cpld_o),
      .idle_valid_i(protocol_idle_valid),
      .cfg_bus_number_o(cfg_bus_number_o),
      .cfg_device_number_o(cfg_device_number_o),
      .cfg_function_number_o(cfg_function_number_o),
      .ext_tag_enable_o(),
      .rcb_128b_o(),
      .max_read_request_size_o(),
      .max_payload_size_o(),
      .msix_enable_o(),
      .msix_mask_o(),
      .status_error_cor_i(rx_error_valid_o || rx_ecrc_error_o),
      .status_error_uncor_i(tx_error_valid_o || malformed_o),
      .rx_cpl_stall_i(!received_completion_ready_i)
  );

endmodule
