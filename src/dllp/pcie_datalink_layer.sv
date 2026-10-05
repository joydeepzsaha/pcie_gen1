// ---------------------------------------------------------------------------
// pcie_datalink_layer -- PCIe Data Link Layer for VC0
//
//! @title pcie_datalink_layer
//! @author Idris Somoye
// Modified by: Kourosh Ghahramani
// Silicon Systems Research Lab, University of Washington
//
// Purpose
//   Connects the Data Link Layer for VC0 between the Transaction Layer and
//   the PHY: pcie_datalink_init (link state), pcie_flow_ctrl_init (InitFC
//   DLLPs), dllp_transmit (sequence number, LCRC, retry buffer) and
//   dllp_receive (receive path, Ack, Nak and UpdateFC DLLPs, configuration
//   completions). Two arbiters merge the TLP sources and the PHY streams.
//
// Interfaces
//   TLP           s_tlp_axis_*: TLPs to send, ahead of dllp_receive's
//                 configuration completions. m_tlp_axis_*: received TLPs.
//   PHY in/out    s_phy_axis_*: received TLPs and DLLPs. m_phy_axis_*: DLLPs
//                 and framed TLPs; tuser bit 1 marks a TLP.
//   Link          phy_link_up_i: Physical LinkUp. idle_valid_i: passed to
//                 pcie_flow_ctrl_init.
//   Flow control  fc_initialized_o: pcie_flow_ctrl_init has left FC_INIT2
//                 and the peer's InitFC2 values are stored. fc_*_o: the
//                 peer's credits. fc_update_valid_o: one cycle for each
//                 received UpdateFC and for the first stored InitFC2 set.
//   Config        cfg_*_number_o: from dllp_receive. ext_tag_enable_o through
//                 msix_mask_o are tied to 0. status_error_cor_i,
//                 status_error_uncor_i and rx_cpl_stall_i are not used.
//
// Clock and reset
//   clk_i only. rst_i is active high; pcie_datalink_init applies it
//   asynchronously, every other block synchronously. soft_reset, high while
//   pcie_datalink_init is in ST_DL_INACTIVE, also resets every submodule
//   except pcie_datalink_init and tlp_arbiter_mux_inst; the retry buffer is
//   discarded with it (PCIe Base Spec r2.1, §3.2.1).
//
// References
//   PCIe Base Spec r2.1, §3.2.1
//   PCIe Base Spec r2.1, §3.5.2.1
//   PCIe Base Spec r2.1, §7.8.4
// ---------------------------------------------------------------------------
`timescale 1ns / 1ps

module pcie_datalink_layer
  import pcie_datalink_pkg::*;
#(
    parameter int DATA_WIDTH = 32,
    parameter int STRB_WIDTH = DATA_WIDTH / 8,
    parameter int KEEP_WIDTH = STRB_WIDTH,
    parameter int USER_WIDTH = 3,
    parameter int S_COUNT = 2,
    parameter int RX_FIFO_SIZE = 3,
    parameter int RETRY_TLP_SIZE = 3,
    parameter int MAX_PAYLOAD_SIZE = 256,
    // Link clock period in ns. The default REPLAY_TIMER_CYCLES and the timers
    // of pcie_flow_ctrl_init and dllp_fc_update are derived from it.
    parameter int CLK_PERIOD_NS = 8,
    // The Max_Payload_Size in bytes and the operating Link width that select
    // the REPLAY_TIMER limit in Table 3-4 (PCIe Base Spec r2.1, §3.5.2.1). 128
    // is the Device Control reset value (PCIe Base Spec r2.1, §7.8.4).
    // MAX_PAYLOAD_SIZE above sizes buffers and does not enter the timer.
    parameter int REPLAY_MPS_BYTES = 128,
    parameter int REPLAY_LINK_WIDTH = 1,
    // 1.75 times the Table 3-4 value, in the upper half of its -0%/+100%
    // tolerance: 622 cycles at x1, 128 bytes and 8 ns (replay_timer_cycles).
    parameter int REPLAY_TIMER_CYCLES =
        pcie_datalink_pkg::replay_timer_cycles(REPLAY_MPS_BYTES, REPLAY_LINK_WIDTH, CLK_PERIOD_NS),
    // Replays before REPLAY_NUM rolls over. With 3, the fourth replay
    // initiation rolls the 2-bit REPLAY_NUM from 11b to 00b and requests a
    // Link retrain (PCIe Base Spec r2.1, §3.5.2.1); retry_management holds the
    // replay in ST_WAIT_RETRAIN.
    parameter int MAX_REPLAY_ATTEMPTS = 3
) (
    input  logic                  clk_i,
    input  logic                  rst_i,
    // ---- TLPs to send ------------------------------------------------------
    input  logic [DATA_WIDTH-1:0] s_tlp_axis_tdata,
    input  logic [KEEP_WIDTH-1:0] s_tlp_axis_tkeep,
    input  logic                  s_tlp_axis_tvalid,
    input  logic                  s_tlp_axis_tlast,
    input  logic [USER_WIDTH-1:0] s_tlp_axis_tuser,
    output logic                  s_tlp_axis_tready,
    // ---- received TLPs -----------------------------------------------------
    output logic [DATA_WIDTH-1:0] m_tlp_axis_tdata,
    output logic [KEEP_WIDTH-1:0] m_tlp_axis_tkeep,
    output logic                  m_tlp_axis_tvalid,
    output logic                  m_tlp_axis_tlast,
    output logic [USER_WIDTH-1:0] m_tlp_axis_tuser,
    input  logic                  m_tlp_axis_tready,
    // ---- from the PHY ------------------------------------------------------
    input  logic [DATA_WIDTH-1:0] s_phy_axis_tdata,
    input  logic [KEEP_WIDTH-1:0] s_phy_axis_tkeep,
    input  logic                  s_phy_axis_tvalid,
    input  logic                  s_phy_axis_tlast,
    input  logic [USER_WIDTH-1:0] s_phy_axis_tuser,
    output logic                  s_phy_axis_tready,
    // ---- to the PHY --------------------------------------------------------
    output logic [DATA_WIDTH-1:0] m_phy_axis_tdata,
    output logic [KEEP_WIDTH-1:0] m_phy_axis_tkeep,
    output logic                  m_phy_axis_tvalid,
    output logic                  m_phy_axis_tlast,
    output logic [USER_WIDTH-1:0] m_phy_axis_tuser,
    input  logic                  m_phy_axis_tready,
    // ---- link and flow control ---------------------------------------------
    input  logic                  phy_link_up_i,
    output logic                  fc_initialized_o,
    output logic                  fc_update_valid_o,
    output logic [7:0]            fc_ph_o,
    output logic [11:0]           fc_pd_o,
    output logic [7:0]            fc_nph_o,
    output logic [11:0]           fc_npd_o,
    output logic [7:0]            fc_cplh_o,
    output logic [11:0]           fc_cpld_o,
    input  logic                  idle_valid_i,

    // ---- configuration, from dllp_receive ----------------------------------
    output logic [7:0] cfg_bus_number_o,
    output logic [4:0] cfg_device_number_o,
    output logic [2:0] cfg_function_number_o,

    // ---- tied to 0 ---------------------------------------------------------
    output logic       ext_tag_enable_o,
    output logic       rcb_128b_o,
    output logic [2:0] max_read_request_size_o,
    output logic [2:0] max_payload_size_o,
    output logic       msix_enable_o,
    output logic       msix_mask_o,
    // ---- not used ----------------------------------------------------------
    input  logic       status_error_cor_i,
    input  logic       status_error_uncor_i,
    input  logic       rx_cpl_stall_i,
    // ---- Link retrain ------------------------------------------------------
    // On a REPLAY_NUM rollover the Physical Layer is asked to retrain the Link,
    // and the replay waits until retraining completes (PCIe Base Spec r2.1,
    // §3.5.2.1). link_retrain_req_o is a level on clk_i, held until retraining
    // is seen. link_retraining_i: the LTSSM is in Recovery or Configuration,
    // synchronised to clk_i by the instantiating top; 0 where there is no
    // LTSSM.
    output logic       link_retrain_req_o,
    input  logic       link_retraining_i = 1'b0
);


  // Arbiter settings for both axis_arb_mux instances, except that
  // arbiter_mux_inst sets ARB_LSB_HIGH_PRIORITY to 0. M_COUNT is not used.
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

  // pcie_flow_ctrl_init's InitFC and UpdateFC DLLPs
  logic            [(DATA_WIDTH)-1:0] phy_fc_axis_tdata;
  logic            [(KEEP_WIDTH)-1:0] phy_fc_axis_tkeep;
  logic                               phy_fc_axis_tvalid;
  logic                               phy_fc_axis_tlast;
  logic            [  USER_WIDTH-1:0] phy_fc_axis_tuser;
  logic                               phy_fc_axis_tready;
  // dllp_receive's Ack, Nak and UpdateFC DLLPs
  logic            [(DATA_WIDTH)-1:0] phy_rx_axis_tdata;
  logic            [(KEEP_WIDTH)-1:0] phy_rx_axis_tkeep;
  logic                               phy_rx_axis_tvalid;
  logic                               phy_rx_axis_tlast;
  logic            [  USER_WIDTH-1:0] phy_rx_axis_tuser;
  logic                               phy_rx_axis_tready;
  // dllp_transmit's framed TLPs, new and replayed
  logic            [(DATA_WIDTH)-1:0] phy_tlp_axis_tdata;
  logic            [(KEEP_WIDTH)-1:0] phy_tlp_axis_tkeep;
  logic                               phy_tlp_axis_tvalid;
  logic                               phy_tlp_axis_tlast;
  logic            [  USER_WIDTH-1:0] phy_tlp_axis_tuser;
  logic                               phy_tlp_axis_tready;


  // dllp_receive's configuration completions
  logic            [  DATA_WIDTH-1:0] cpl_from_cfg_tdata;
  logic            [  KEEP_WIDTH-1:0] cpl_from_cfg_tkeep;
  logic                               cpl_from_cfg_tvalid;
  logic                               cpl_from_cfg_tlast;
  logic            [  USER_WIDTH-1:0] cpl_from_cfg_tuser;
  logic                               cpl_from_cfg_tready;


  // The TLP arbiter's output, into dllp_transmit
  logic            [  DATA_WIDTH-1:0] tx_tlp_tdata;
  logic            [  KEEP_WIDTH-1:0] tx_tlp_tkeep;
  logic                               tx_tlp_tvalid;
  logic                               tx_tlp_tlast;
  logic            [  USER_WIDTH-1:0] tx_tlp_tuser;
  logic                               tx_tlp_tready;

  //tlp ack/nak
  logic            [            11:0] seq_num;
  logic                               seq_num_vld;
  logic                               seq_num_acknack;
  //flow control
  logic            [             7:0] tx_fc_ph;
  logic            [            11:0] tx_fc_pd;
  logic            [             7:0] tx_fc_nph;
  logic            [            11:0] tx_fc_npd;
  logic            [             7:0] tx_fc_cplh;
  logic            [            11:0] tx_fc_cpld;
  logic                               update_fc;
  logic                               init_ack;
  // ack_nack, ack_nack_vld and ack_seq_num are not used: received Acks and
  // Naks reach dllp_transmit on seq_num, seq_num_vld and seq_num_acknack.
  logic                               ack_nack;
  logic                               ack_nack_vld;
  logic                               ack_seq_num;
  logic                               init_flow_control;
  logic                               soft_reset;
  // The REPLAY_TIMER start event, from tlp_sent_tracker below.
  logic                               tlp_sent;
  logic [                       11:0] tlp_sent_seq;
  logic                               phy_tx_mid_r;
  logic                               phy_tx_is_tlp_r;
  logic [                       11:0] phy_tx_seq_r;
  logic                               fc1_values_stored;
  logic                               fc2_values_stored;
  logic                               fc2_values_sent;
  logic                               fc_init_done;
  logic                               fc2_values_stored_reg;
  logic                               first_feature_exchange_dllp_received;




  (* syn_keep = "true", mark_debug = "true" *) pcie_dl_status_e                    link_status;
  logic                               first_tlp_valid;


  assign fc_initialized_o = fc2_values_sent && fc2_values_stored;
  assign fc_update_valid_o = update_fc || fc_init_done;
  assign fc_ph_o = tx_fc_ph;
  assign fc_pd_o = tx_fc_pd;
  assign fc_nph_o = tx_fc_nph;
  assign fc_npd_o = tx_fc_npd;
  assign fc_cplh_o = tx_fc_cplh;
  assign fc_cpld_o = tx_fc_cpld;

  pcie_datalink_init #() pcie_datalink_init_inst (
      .clk_i              (clk_i),
      .rst_i              (rst_i),
      .phy_link_up_i      (phy_link_up_i),
      .init_flow_control_o(init_flow_control),
      .soft_reset_o       (soft_reset),
      .link_status_o      (link_status),
      .fc1_values_stored_i(fc1_values_stored),
      .fc2_values_stored_i(fc2_values_stored),
      .init_ack_i         (init_ack)
  );


  pcie_flow_ctrl_init #(
      .DATA_WIDTH      (DATA_WIDTH),
      .STRB_WIDTH      (STRB_WIDTH),
      .KEEP_WIDTH      (KEEP_WIDTH),
      .USER_WIDTH      (USER_WIDTH),
      .MAX_PAYLOAD_SIZE(MAX_PAYLOAD_SIZE),
      .CLK_PERIOD_NS   (CLK_PERIOD_NS)
  ) pcie_flow_ctrl_init_inst (
      .clk_i               (clk_i),
      .rst_i               (rst_i || soft_reset),
      .start_flow_control_i(init_flow_control),
      .first_tlp_valid_i   (first_tlp_valid),
      .idle_valid_i        (idle_valid_i),
      .fc1_values_stored_i (fc1_values_stored),
      .fc2_values_stored_i (fc2_values_stored),
      .update_fc_i         (update_fc),
      .first_feature_exchange_dllp_received_i(first_feature_exchange_dllp_received),
      .m_axis_tdata        (phy_fc_axis_tdata),
      .m_axis_tkeep        (phy_fc_axis_tkeep),
      .m_axis_tvalid       (phy_fc_axis_tvalid),
      .m_axis_tlast        (phy_fc_axis_tlast),
      .m_axis_tuser        (phy_fc_axis_tuser),
      .m_axis_tready       (phy_fc_axis_tready),
      .fc2_values_sent_o   (fc2_values_sent),
      .init_ack_o          (init_ack)
  );

  dllp_transmit #(
      .DATA_WIDTH(DATA_WIDTH),
      .STRB_WIDTH(STRB_WIDTH),
      .KEEP_WIDTH(KEEP_WIDTH),
      .USER_WIDTH(USER_WIDTH),
      .MAX_PAYLOAD_SIZE(MAX_PAYLOAD_SIZE),
      .S_COUNT(S_COUNT),
      .RETRY_TLP_SIZE(RETRY_TLP_SIZE),
      .REPLAY_TIMER_CYCLES(REPLAY_TIMER_CYCLES),
      .MAX_REPLAY_ATTEMPTS(MAX_REPLAY_ATTEMPTS)
  ) dllp_transmit_inst (
      .clk_i         (clk_i),
      .rst_i         (rst_i || soft_reset),
      .tlp_sent_i    (tlp_sent),
      .tlp_sent_seq_i(tlp_sent_seq),
      .s_axis_tdata  (tx_tlp_tdata),
      .s_axis_tkeep  (tx_tlp_tkeep),
      .s_axis_tvalid (tx_tlp_tvalid),
      .s_axis_tlast  (tx_tlp_tlast),
      .s_axis_tuser  (tx_tlp_tuser),
      .s_axis_tready (tx_tlp_tready),
      .m_axis_tdata  (phy_tlp_axis_tdata),
      .m_axis_tkeep  (phy_tlp_axis_tkeep),
      .m_axis_tvalid (phy_tlp_axis_tvalid),
      .m_axis_tlast  (phy_tlp_axis_tlast),
      .m_axis_tuser  (phy_tlp_axis_tuser),
      .m_axis_tready (phy_tlp_axis_tready),
      .ack_nack_i    (seq_num_acknack),
      .ack_nack_vld_i(seq_num_vld),
      .ack_seq_num_i (seq_num),
      .tx_fc_ph_i    (tx_fc_ph),
      .tx_fc_pd_i    (tx_fc_pd),
      .tx_fc_nph_i   (tx_fc_nph),
      .tx_fc_npd_i   (tx_fc_npd),
      .tx_fc_cplh_i  (tx_fc_cplh),
      .tx_fc_cpld_i  (tx_fc_cpld),
      .update_fc_i   (update_fc || fc_init_done),
      .link_retrain_req_o(link_retrain_req_o),
      .link_retraining_i (link_retraining_i)
  );


  dllp_receive #(
      .DATA_WIDTH(DATA_WIDTH),
      .STRB_WIDTH(STRB_WIDTH),
      .KEEP_WIDTH(KEEP_WIDTH),
      .USER_WIDTH(USER_WIDTH),
      .MAX_PAYLOAD_SIZE(MAX_PAYLOAD_SIZE),
      .RX_FIFO_SIZE(RX_FIFO_SIZE),
      .CLK_PERIOD_NS(CLK_PERIOD_NS)
  ) dllp_receive_inst (
      .clk_i                 (clk_i),
      .rst_i                 (rst_i || soft_reset),
      .link_status_i         (link_status),
      .phy_link_up_i         (phy_link_up_i),
      .s_axis_tdata          (s_phy_axis_tdata),
      .s_axis_tkeep          (s_phy_axis_tkeep),
      .s_axis_tvalid         (s_phy_axis_tvalid),
      .s_axis_tlast          (s_phy_axis_tlast),
      .s_axis_tuser          (s_phy_axis_tuser),
      .s_axis_tready         (s_phy_axis_tready),
      .m_axis_dllp2tlp_tdata (m_tlp_axis_tdata),
      .m_axis_dllp2tlp_tkeep (m_tlp_axis_tkeep),
      .m_axis_dllp2tlp_tvalid(m_tlp_axis_tvalid),
      .m_axis_dllp2tlp_tlast (m_tlp_axis_tlast),
      .m_axis_dllp2tlp_tuser (m_tlp_axis_tuser),
      .m_axis_dllp2tlp_tready(m_tlp_axis_tready),
      .m_axis_dllp2phy_tdata (phy_rx_axis_tdata),
      .m_axis_dllp2phy_tkeep (phy_rx_axis_tkeep),
      .m_axis_dllp2phy_tvalid(phy_rx_axis_tvalid),
      .m_axis_dllp2phy_tlast (phy_rx_axis_tlast),
      .m_axis_dllp2phy_tuser (phy_rx_axis_tuser),
      .m_axis_dllp2phy_tready(phy_rx_axis_tready),
      .m_cpl_from_cfg_tdata  (cpl_from_cfg_tdata),
      .m_cpl_from_cfg_tkeep  (cpl_from_cfg_tkeep),
      .m_cpl_from_cfg_tvalid (cpl_from_cfg_tvalid),
      .m_cpl_from_cfg_tlast  (cpl_from_cfg_tlast),
      .m_cpl_from_cfg_tuser  (cpl_from_cfg_tuser),
      .m_cpl_from_cfg_tready (cpl_from_cfg_tready),
      .cfg_bus_number_o      (cfg_bus_number_o),
      .cfg_device_number_o   (cfg_device_number_o),
      .cfg_function_number_o (cfg_function_number_o),
      
      // From dllp_handler, except first_tlp_valid_o (axis_user_demux)
      .seq_num_o             (seq_num),
      .seq_num_vld_o         (seq_num_vld),
      .seq_num_acknack_o     (seq_num_acknack),
      .fc1_values_stored_o   (fc1_values_stored),
      .fc2_values_stored_o   (fc2_values_stored),
      .first_tlp_valid_o     (first_tlp_valid),
      .tx_fc_ph_o            (tx_fc_ph),
      .tx_fc_pd_o            (tx_fc_pd),
      .tx_fc_nph_o           (tx_fc_nph),
      .tx_fc_npd_o           (tx_fc_npd),
      .tx_fc_cplh_o          (tx_fc_cplh),
      .tx_fc_cpld_o          (tx_fc_cpld),
      .update_fc_o           (update_fc),
      .first_feature_exchange_dllp_received_o (first_feature_exchange_dllp_received)
  );


  axis_arb_mux #(
      .S_COUNT              (3),
      .DATA_WIDTH           (DATA_WIDTH),
      .KEEP_ENABLE          (KEEP_ENABLE),
      .KEEP_WIDTH           (KEEP_WIDTH),
      .ID_ENABLE            (ID_ENABLE),
      .S_ID_WIDTH           (ID_WIDTH),
      .DEST_ENABLE          (DEST_ENABLE),
      .DEST_WIDTH           (DEST_WIDTH),
      .USER_ENABLE          (USER_ENABLE),
      .USER_WIDTH           (USER_WIDTH),
      .LAST_ENABLE          (LAST_ENABLE),
      .ARB_TYPE_ROUND_ROBIN (ARB_TYPE_ROUND_ROBIN),
      // Input order is {receive DLLP, flow-control DLLP, TLP}, highest
      // priority first: dllp_receive's Ack, Nak and UpdateFC DLLPs, then
      // pcie_flow_ctrl_init's, then TLPs. Ack and Nak ahead of TLPs is the
      // order PCIe Base Spec r2.1, §3.5.2.1 recommends.
      .ARB_LSB_HIGH_PRIORITY(0)
  ) arbiter_mux_inst (
      .clk          (clk_i),
      .rst          (rst_i || soft_reset),
      // AXI inputs
      .s_axis_tdata ({phy_rx_axis_tdata, phy_fc_axis_tdata, phy_tlp_axis_tdata}),
      .s_axis_tkeep ({phy_rx_axis_tkeep, phy_fc_axis_tkeep, phy_tlp_axis_tkeep}),
      .s_axis_tvalid({phy_rx_axis_tvalid, phy_fc_axis_tvalid, phy_tlp_axis_tvalid}),
      .s_axis_tready({phy_rx_axis_tready, phy_fc_axis_tready, phy_tlp_axis_tready}),
      .s_axis_tlast ({phy_rx_axis_tlast, phy_fc_axis_tlast, phy_tlp_axis_tlast}),
      .s_axis_tid   (),
      .s_axis_tdest (),
      .s_axis_tuser ({phy_rx_axis_tuser, phy_fc_axis_tuser, phy_tlp_axis_tuser}),
      // AXI output
      .m_axis_tdata (m_phy_axis_tdata),
      .m_axis_tkeep (m_phy_axis_tkeep),
      .m_axis_tvalid(m_phy_axis_tvalid),
      .m_axis_tready(m_phy_axis_tready),
      .m_axis_tlast (m_phy_axis_tlast),
      .m_axis_tid   (),
      .m_axis_tdest (),
      .m_axis_tuser (m_phy_axis_tuser)
  );

  // -------------------------------------------------------------------------
  // TLP sent
  // -------------------------------------------------------------------------
  // The REPLAY_TIMER starts at the last Symbol of a TLP transmission or
  // retransmission (PCIe Base Spec r2.1, §3.5.2.1). The last point this layer
  // owns is m_phy_axis, the PHY arbiter's output, which new and replayed TLPs
  // both pass, so tlp_sent is the handshake of a TLP frame's last beat there.
  // tuser bit 1 marks a TLP frame (UserIsTlp in axis_user_demux). The first
  // beat carries the sequence number as {tdata[3:0], tdata[15:8]}, the layout
  // tlp2dllp builds and dllp2tlp parses. With DATA_WIDTH = 32 a framed TLP is
  // at least five beats (sequence number, 3 DW header and LCRC: 18 bytes), so
  // its first beat is never its last, and tlp_sent_seq always comes from the
  // register loaded on the first beat.
  always_ff @(posedge clk_i) begin : tlp_sent_tracker
    if (rst_i || soft_reset) begin
      phy_tx_mid_r    <= 1'b0;
      phy_tx_is_tlp_r <= 1'b0;
      phy_tx_seq_r    <= '0;
    end else if (m_phy_axis_tvalid && m_phy_axis_tready) begin
      if (!phy_tx_mid_r) begin
        phy_tx_is_tlp_r <= m_phy_axis_tuser[1];
        phy_tx_seq_r    <= {m_phy_axis_tdata[3:0], m_phy_axis_tdata[15:8]};
      end
      phy_tx_mid_r <= !m_phy_axis_tlast;
    end
  end

  assign tlp_sent     = m_phy_axis_tvalid && m_phy_axis_tready && m_phy_axis_tlast &&
                        phy_tx_mid_r && phy_tx_is_tlp_r;
  assign tlp_sent_seq = phy_tx_seq_r;


  // Port 0, the TLPs from s_tlp_axis, wins over port 1, dllp_receive's
  // configuration completions (ARB_LSB_HIGH_PRIORITY = 1). This arbiter is
  // reset by rst_i only.
  axis_arb_mux #(
      .S_COUNT              (2),
      .DATA_WIDTH           (DATA_WIDTH),
      .KEEP_ENABLE          (KEEP_ENABLE),
      .KEEP_WIDTH           (KEEP_WIDTH),
      .ID_ENABLE            (ID_ENABLE),
      .S_ID_WIDTH           (ID_WIDTH),
      .DEST_ENABLE          (DEST_ENABLE),
      .DEST_WIDTH           (DEST_WIDTH),
      .USER_ENABLE          (USER_ENABLE),
      .USER_WIDTH           (USER_WIDTH),
      .LAST_ENABLE          (LAST_ENABLE),
      .ARB_TYPE_ROUND_ROBIN (ARB_TYPE_ROUND_ROBIN),
      .ARB_LSB_HIGH_PRIORITY(ARB_LSB_HIGH_PRIORITY)
  ) tlp_arbiter_mux_inst (
      .clk          (clk_i),
      .rst          (rst_i),
      // AXI inputs
      .s_axis_tdata ({cpl_from_cfg_tdata, s_tlp_axis_tdata}),
      .s_axis_tkeep ({cpl_from_cfg_tkeep, s_tlp_axis_tkeep}),
      .s_axis_tvalid({cpl_from_cfg_tvalid, s_tlp_axis_tvalid}),
      .s_axis_tready({cpl_from_cfg_tready, s_tlp_axis_tready}),
      .s_axis_tlast ({cpl_from_cfg_tlast, s_tlp_axis_tlast}),
      .s_axis_tid   (),
      .s_axis_tdest (),
      .s_axis_tuser ({cpl_from_cfg_tuser, s_tlp_axis_tuser}),
      // AXI output
      .m_axis_tdata (tx_tlp_tdata),
      .m_axis_tkeep (tx_tlp_tkeep),
      .m_axis_tvalid(tx_tlp_tvalid),
      .m_axis_tready(tx_tlp_tready),
      .m_axis_tlast (tx_tlp_tlast),
      .m_axis_tid   (),
      .m_axis_tdest (),
      .m_axis_tuser (tx_tlp_tuser)
  );

  // fc_init_done is one cycle on the rising edge of fc2_values_stored: it
  // loads the peer's InitFC2 credits into dllp_transmit and is reported on
  // fc_update_valid_o.
  always_ff @(posedge clk_i) begin
    if (rst_i || soft_reset)
      fc2_values_stored_reg <= 1'b0;
    else
      fc2_values_stored_reg <= fc2_values_stored;
  end


  assign ext_tag_enable_o        = '0;
  assign rcb_128b_o              = '0;
  assign max_read_request_size_o = '0;
  assign max_payload_size_o      = '0;
  assign msix_enable_o           = '0;
  assign msix_mask_o             = '0;
  assign fc_init_done            = fc2_values_stored && !fc2_values_stored_reg;

endmodule
