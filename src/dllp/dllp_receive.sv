// ---------------------------------------------------------------------------
//! @title dllp_receive
//! @author Idris Somoye
//! Receive half of the Data Link Layer: splits the Physical Layer stream
//! into TLPs and DLLPs and processes each.
//
// Purpose
//   axis_user_demux splits the receive stream by tuser. DLLPs go to
//   dllp_handler, which checks and decodes them. TLPs go to dllp2tlp, which
//   checks sequence number and LCRC, requests an Ack or Nak, and buffers each
//   good TLP; dllp_fc_update transmits that Ack or Nak and the UpdateFC
//   DLLPs. Good TLPs then pass through pcie_cfg_wrapper, which forwards every
//   TLP and also answers CfgRd0 and CfgWr0.
//
// Interfaces
//   Link          link_status_i: from pcie_datalink_init. phy_link_up_i:
//                 Physical LinkUp, for dllp_handler.
//   Input         s_axis_*: the Physical Layer receive stream.
//   TLP output    m_axis_dllp2tlp_*: every received TLP; CfgRd0 and CfgWr0
//                 also go to the configuration handler in pcie_cfg_wrapper.
//   Config        m_cpl_from_cfg_*: completions from pcie_cfg_wrapper.
//                 cfg_bus_number_o, cfg_device_number_o,
//                 cfg_function_number_o: from pcie_cfg_wrapper.
//   DLLP output   m_axis_dllp2phy_*: Ack, Nak and UpdateFC DLLPs from
//                 dllp_fc_update.
//   Received      seq_num_o, seq_num_vld_o, seq_num_acknack_o,
//                 fc1_values_stored_o, fc2_values_stored_o, tx_fc_*_o,
//                 update_fc_o, first_feature_exchange_dllp_received_o: from
//                 dllp_handler. first_tlp_valid_o: from axis_user_demux.
//
// Clock and reset
//   clk_i only. rst_i is synchronous and active high; pcie_datalink_layer
//   also asserts it while the link is down.
//
// Limitations
//   pcie_cfg_wrapper's hwif_in is tied to 0 and its hwif_out is not read.
//   The body parameters ID_ENABLE to KEEP_ENABLE and the localparams
//   UserIsTlp and UserIsDllp are not used.
// ---------------------------------------------------------------------------
module dllp_receive
  import pcie_datalink_pkg::*;
  import pcie_config_reg_pkg::*;
#(
    parameter int DATA_WIDTH = 32,
    parameter int STRB_WIDTH = DATA_WIDTH / 8,
    parameter int KEEP_WIDTH = STRB_WIDTH,
    parameter int USER_WIDTH = 4,
    parameter int MAX_PAYLOAD_SIZE = 1,
    // Link clock period in ns, for dllp_fc_update's UpdateFC timer.
    parameter int CLK_PERIOD_NS = 8,
    parameter int RX_FIFO_SIZE = 2
) (
    input  logic                               clk_i,
    input  logic                               rst_i,
    input  pcie_dl_status_e                    link_status_i,
    input  logic                               phy_link_up_i,

    // ---- receive stream from the Physical Layer ----------------------------
    input  logic            [(DATA_WIDTH)-1:0] s_axis_tdata,
    input  logic            [(KEEP_WIDTH)-1:0] s_axis_tkeep,
    input  logic                               s_axis_tvalid,
    input  logic                               s_axis_tlast,
    input  logic            [(USER_WIDTH)-1:0] s_axis_tuser,
    output logic                               s_axis_tready,

    // ---- TLPs to the Transaction Layer -------------------------------------
    output logic            [(DATA_WIDTH)-1:0] m_axis_dllp2tlp_tdata,
    output logic            [(KEEP_WIDTH)-1:0] m_axis_dllp2tlp_tkeep,
    output logic                               m_axis_dllp2tlp_tvalid,
    output logic                               m_axis_dllp2tlp_tlast,
    output logic            [(USER_WIDTH)-1:0] m_axis_dllp2tlp_tuser,
    input  logic                               m_axis_dllp2tlp_tready,

    // ---- DLLPs to the Physical Layer: Ack, Nak, UpdateFC -------------------
    output logic            [(DATA_WIDTH)-1:0] m_axis_dllp2phy_tdata,
    output logic            [(KEEP_WIDTH)-1:0] m_axis_dllp2phy_tkeep,
    output logic                               m_axis_dllp2phy_tvalid,
    output logic                               m_axis_dllp2phy_tlast,
    output logic            [(USER_WIDTH)-1:0] m_axis_dllp2phy_tuser,
    input  logic                               m_axis_dllp2phy_tready,

    // ---- configuration completions and captured ID -------------------------
    output logic            [(DATA_WIDTH)-1:0] m_cpl_from_cfg_tdata,
    output logic            [(KEEP_WIDTH)-1:0] m_cpl_from_cfg_tkeep,
    output logic                               m_cpl_from_cfg_tvalid,
    output logic                               m_cpl_from_cfg_tlast,
    output logic            [(USER_WIDTH)-1:0] m_cpl_from_cfg_tuser,
    input  logic                               m_cpl_from_cfg_tready,

    output logic [ 7:0] cfg_bus_number_o,
    output logic [ 4:0] cfg_device_number_o,
    output logic [ 2:0] cfg_function_number_o,

    // ---- from dllp_handler and axis_user_demux -----------------------------
    output logic [11:0] seq_num_o,
    output logic        seq_num_vld_o,
    output logic        seq_num_acknack_o,
    output logic        fc1_values_stored_o,
    output logic        fc2_values_stored_o,
    output logic        first_tlp_valid_o,
    output logic [ 7:0] tx_fc_ph_o,
    output logic [11:0] tx_fc_pd_o,
    output logic [ 7:0] tx_fc_nph_o,
    output logic [11:0] tx_fc_npd_o,
    output logic [ 7:0] tx_fc_cplh_o,
    output logic [11:0] tx_fc_cpld_o,
    output logic        update_fc_o,
    output logic        first_feature_exchange_dllp_received_o
);

  localparam int UserIsTlp = 1;
  localparam int UserIsDllp = 0;
  // Used only by pcie_cfg_wrapper.
  parameter int TLP_DATA_WIDTH = 256;
  parameter int TLP_STRB_WIDTH = TLP_DATA_WIDTH / 32;
  parameter int TLP_HDR_WIDTH = 128;

  parameter int ID_ENABLE = 0;
  parameter int ID_WIDTH = 8;
  parameter int DEST_ENABLE = 0;
  parameter int DEST_WIDTH = 8;
  parameter int USER_ENABLE = 1;
  parameter int LAST_ENABLE = 1;
  parameter int ARB_TYPE_ROUND_ROBIN = 0;
  parameter int ARB_LSB_HIGH_PRIORITY = 1;
  parameter int M_COUNT = 2;
  parameter int KEEP_ENABLE = (DATA_WIDTH > 8);

  logic                                     dllp_ready;
  logic                                     tlp_ready;
  logic                                     start_flow_control;
  logic                                     start_flow_control_ack;
  logic                  [            15:0] next_transmit_seq;
  logic                                     tlp_nullified;
  // CREDITS_ALLOCATED, dllp2tlp -> dllp_fc_update
  logic                  [             7:0] ph_credits_allocated;
  logic                  [            11:0] pd_credits_allocated;
  logic                  [             7:0] nph_credits_allocated;
  logic                  [            11:0] npd_credits_allocated;


  logic                  [(DATA_WIDTH)-1:0] tlp_axis_tdata;
  logic                  [(KEEP_WIDTH)-1:0] tlp_axis_tkeep;
  logic                                     tlp_axis_tvalid;
  logic                                     tlp_axis_tlast;
  logic                  [(USER_WIDTH)-1:0] tlp_axis_tuser;
  logic                                     tlp_axis_tready;


  logic                  [(DATA_WIDTH)-1:0] dllp_axis_tdata;
  logic                  [(KEEP_WIDTH)-1:0] dllp_axis_tkeep;
  logic                                     dllp_axis_tvalid;
  logic                                     dllp_axis_tlast;
  logic                  [(USER_WIDTH)-1:0] dllp_axis_tuser;
  logic                                     dllp_axis_tready;

  logic                  [  DATA_WIDTH-1:0] tlp_to_mac_tdata;
  logic                  [  KEEP_WIDTH-1:0] tlp_to_mac_tkeep;
  logic                                     tlp_to_mac_tvalid;
  logic                                     tlp_to_mac_tlast;
  logic                  [  USER_WIDTH-1:0] tlp_to_mac_tuser;
  logic                                     tlp_to_mac_tready;


  logic                  [  DATA_WIDTH-1:0] dllp_fc_tdata;
  logic                  [  KEEP_WIDTH-1:0] dllp_fc_tkeep;
  logic                                     dllp_fc_tvalid;
  logic                                     dllp_fc_tlast;
  logic                  [  USER_WIDTH-1:0] dllp_fc_tuser;
  logic                                     dllp_fc_tready;

  pcie_config_reg__in_t                     hwif_in;
  pcie_config_reg__out_t                    hwif_out;


  assign hwif_in = '{default: 'd0};

  axis_user_demux #(
      .DATA_WIDTH      (DATA_WIDTH),
      .STRB_WIDTH      (STRB_WIDTH),
      .KEEP_WIDTH      (KEEP_WIDTH),
      .USER_WIDTH      (USER_WIDTH),
      .MAX_PAYLOAD_SIZE(MAX_PAYLOAD_SIZE),
      .RX_FIFO_SIZE    (RX_FIFO_SIZE)
  ) axis_user_demux_inst (
      .clk_i             (clk_i),
      .rst_i             (rst_i),
      .link_status_i     (link_status_i),
      .first_tlp_valid_o (first_tlp_valid_o),
      .s_axis_tdata      (s_axis_tdata),
      .s_axis_tkeep      (s_axis_tkeep),
      .s_axis_tvalid     (s_axis_tvalid),
      .s_axis_tlast      (s_axis_tlast),
      .s_axis_tuser      (s_axis_tuser),
      .s_axis_tready     (s_axis_tready),
      .m_tlp_axis_tdata  (tlp_axis_tdata),
      .m_tlp_axis_tkeep  (tlp_axis_tkeep),
      .m_tlp_axis_tvalid (tlp_axis_tvalid),
      .m_tlp_axis_tlast  (tlp_axis_tlast),
      .m_tlp_axis_tuser  (tlp_axis_tuser),
      .m_tlp_axis_tready (tlp_axis_tready),
      .m_dllp_axis_tdata (dllp_axis_tdata),
      .m_dllp_axis_tkeep (dllp_axis_tkeep),
      .m_dllp_axis_tvalid(dllp_axis_tvalid),
      .m_dllp_axis_tlast (dllp_axis_tlast),
      .m_dllp_axis_tuser (dllp_axis_tuser),
      .m_dllp_axis_tready(dllp_axis_tready)
  );


  //dllp handler instance
  dllp_handler #(
      .DATA_WIDTH(DATA_WIDTH),
      .STRB_WIDTH(STRB_WIDTH),
      .KEEP_WIDTH(KEEP_WIDTH),
      .USER_WIDTH(USER_WIDTH)
  ) dllp_handler_inst (
      .clk_i              (clk_i),
      .rst_i              (rst_i),
      .phy_link_up_i      (phy_link_up_i),
      .s_axis_tdata       (dllp_axis_tdata),
      .s_axis_tkeep       (dllp_axis_tkeep),
      .s_axis_tvalid      (dllp_axis_tvalid),
      .s_axis_tlast       (dllp_axis_tlast),
      .s_axis_tuser       (dllp_axis_tuser),
      .s_axis_tready      (dllp_axis_tready),
      .seq_num_o          (seq_num_o),
      .seq_num_vld_o      (seq_num_vld_o),
      .seq_num_acknack_o  (seq_num_acknack_o),
      .fc1_values_stored_o(fc1_values_stored_o),
      .fc2_values_stored_o(fc2_values_stored_o),
      .tx_fc_ph_o         (tx_fc_ph_o),
      .tx_fc_pd_o         (tx_fc_pd_o),
      .tx_fc_nph_o        (tx_fc_nph_o),
      .tx_fc_npd_o        (tx_fc_npd_o),
      .tx_fc_cplh_o       (tx_fc_cplh_o),
      .tx_fc_cpld_o       (tx_fc_cpld_o),
      .update_fc_o        (update_fc_o),
      .first_feature_exchange_dllp_received_o (first_feature_exchange_dllp_received_o)
  );

  //dllp flow control update module instance
  dllp_fc_update #(
      .DATA_WIDTH      (DATA_WIDTH),
      .STRB_WIDTH      (STRB_WIDTH),
      .KEEP_WIDTH      (KEEP_WIDTH),
      .USER_WIDTH      (USER_WIDTH),
      .MAX_PAYLOAD_SIZE(MAX_PAYLOAD_SIZE),
      .CLK_PERIOD_NS   (CLK_PERIOD_NS)
  ) dllp_fc_update_inst (
      .clk_i                   (clk_i),
      .rst_i                   (rst_i),
      .link_status_i           (link_status_i),
      .start_flow_control_i    (start_flow_control),
      .start_flow_control_ack_o(start_flow_control_ack),
      .next_transmit_seq_i     (next_transmit_seq),
      .tlp_nullified_i         (tlp_nullified),
      .ph_credits_allocated_i  (ph_credits_allocated),
      .pd_credits_allocated_i  (pd_credits_allocated),
      .nph_credits_allocated_i (nph_credits_allocated),
      .npd_credits_allocated_i (npd_credits_allocated),
      .m_axis_tdata            (m_axis_dllp2phy_tdata),
      .m_axis_tkeep            (m_axis_dllp2phy_tkeep),
      .m_axis_tvalid           (m_axis_dllp2phy_tvalid),
      .m_axis_tlast            (m_axis_dllp2phy_tlast),
      .m_axis_tuser            (m_axis_dllp2phy_tuser),
      .m_axis_tready           (m_axis_dllp2phy_tready)
  );

  //dllp2tlp converter
  dllp2tlp #(
      .DATA_WIDTH      (DATA_WIDTH),
      .STRB_WIDTH      (STRB_WIDTH),
      .KEEP_WIDTH      (KEEP_WIDTH),
      .USER_WIDTH      (USER_WIDTH),
      .MAX_PAYLOAD_SIZE(MAX_PAYLOAD_SIZE),
      .RX_FIFO_SIZE    (RX_FIFO_SIZE)
  ) dllp2tlp_inst (
      .clk_i                   (clk_i),
      .rst_i                   (rst_i),
      .link_status_i           (link_status_i),
      .s_axis_tdata            (tlp_axis_tdata),
      .s_axis_tkeep            (tlp_axis_tkeep),
      .s_axis_tvalid           (tlp_axis_tvalid),
      .s_axis_tlast            (tlp_axis_tlast),
      .s_axis_tuser            (tlp_axis_tuser),
      .s_axis_tready           (tlp_axis_tready),
      .start_flow_control_o    (start_flow_control),
      .start_flow_control_ack_i(start_flow_control_ack),
      .next_transmit_seq_o     (next_transmit_seq),
      .tlp_nullified_o         (tlp_nullified),
      .ph_credits_allocated_o  (ph_credits_allocated),
      .pd_credits_allocated_o  (pd_credits_allocated),
      .nph_credits_allocated_o (nph_credits_allocated),
      .npd_credits_allocated_o (npd_credits_allocated),
      .m_tlp_axis_tdata        (tlp_to_mac_tdata),
      .m_tlp_axis_tkeep        (tlp_to_mac_tkeep),
      .m_tlp_axis_tvalid       (tlp_to_mac_tvalid),
      .m_tlp_axis_tlast        (tlp_to_mac_tlast),
      .m_tlp_axis_tuser        (tlp_to_mac_tuser),
      .m_tlp_axis_tready       (tlp_to_mac_tready)
  );


  pcie_cfg_wrapper #(
      .DATA_WIDTH    (DATA_WIDTH),
      .STRB_WIDTH    (STRB_WIDTH),
      .KEEP_WIDTH    (KEEP_WIDTH),
      .USER_WIDTH    (USER_WIDTH),
      .TLP_DATA_WIDTH(TLP_DATA_WIDTH),
      .TLP_STRB_WIDTH(TLP_STRB_WIDTH),
      .TLP_HDR_WIDTH (TLP_HDR_WIDTH)
  ) pcie_cfg_wrapper_inst (
      .clk_i        (clk_i),
      .rst_i        (rst_i),
      .s_axis_tdata (tlp_to_mac_tdata),
      .s_axis_tkeep (tlp_to_mac_tkeep),
      .s_axis_tvalid(tlp_to_mac_tvalid),
      .s_axis_tlast (tlp_to_mac_tlast),
      .s_axis_tuser (tlp_to_mac_tuser),
      .s_axis_tready(tlp_to_mac_tready),

      .cpl_axis_tdata (m_cpl_from_cfg_tdata),
      .cpl_axis_tkeep (m_cpl_from_cfg_tkeep),
      .cpl_axis_tvalid(m_cpl_from_cfg_tvalid),
      .cpl_axis_tlast (m_cpl_from_cfg_tlast),
      .cpl_axis_tuser (m_cpl_from_cfg_tuser),
      .cpl_axis_tready(m_cpl_from_cfg_tready),

      .cfg_bus_number_o     (cfg_bus_number_o),
      .cfg_device_number_o  (cfg_device_number_o),
      .cfg_function_number_o(cfg_function_number_o),

      .m_tlp_axis_tdata (m_axis_dllp2tlp_tdata),
      .m_tlp_axis_tkeep (m_axis_dllp2tlp_tkeep),
      .m_tlp_axis_tvalid(m_axis_dllp2tlp_tvalid),
      .m_tlp_axis_tlast (m_axis_dllp2tlp_tlast),
      .m_tlp_axis_tuser (m_axis_dllp2tlp_tuser),
      .m_tlp_axis_tready(m_axis_dllp2tlp_tready),
      .hwif_in          (hwif_in),
      .hwif_out         (hwif_out)
  );


  // pcie_datalink_layer arbitrates m_axis_dllp2phy_* and m_cpl_from_cfg_*
  // with its other transmit streams; nothing is merged here.


  // Waveform dump, only when COCOTB_SIM is defined.
`ifdef COCOTB_SIM
  initial begin
    $dumpfile("dllp_receive.fst");
    $dumpvars(0, dllp_receive);
    #1;
  end
`endif

endmodule
