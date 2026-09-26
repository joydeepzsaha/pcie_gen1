`timescale 1ns / 1ps

//! @title pcie_datalink_layer
//! @author Idris Somoye
//
//! Implements a pcie datalink layer.
//! Module accepts TLPs in AXI-Stream format and converts them into packets to be
//! transmitted to the PHY logical layer.
//!
//!
//! Module accepts packets from the phy logical layer and either handles them as DLLPs,
//! or converts them to TLPs to be output as AXI-Stream packets
//!
//! Module implements virtual channel 0 (default).
module pcie_datalink_layer
  import pcie_datalink_pkg::*;
#(
    // Parameters
    parameter int DATA_WIDTH = 32,
    parameter int STRB_WIDTH = DATA_WIDTH / 8,
    parameter int KEEP_WIDTH = STRB_WIDTH,
    parameter int USER_WIDTH = 3,
    parameter int S_COUNT = 2,
    parameter int RX_FIFO_SIZE = 3,
    parameter int RETRY_TLP_SIZE = 3,
    parameter int MAX_PAYLOAD_SIZE = 256,
    // sec 63 #7g-2 D-7G.2: the link-clock period in ns, the ONE source every
    // cycle-count timer in this layer derives from.  Default 8 = 125 MHz.
    parameter int CLK_PERIOD_NS = 8,
    // sec 63 #7g-2 Q3: the Table 3-4 row the REPLAY_TIMER is derived from --
    // the Device Control Max_Payload_Size it assumes (reset default 128 B,
    // sec 7.8.4 p.510) and the operating Link width.  NOT MAX_PAYLOAD_SIZE
    // above, which sizes buffers.
    parameter int REPLAY_MPS_BYTES = 128,
    parameter int REPLAY_LINK_WIDTH = 1,
    // Upper half of Table 3-4's -0%/+100% window: 1.75 x 711 ST = 622 cycles
    // at x1 / MPS 128 / 8 ns (pcie_datalink_pkg::replay_timer_cycles).  Was the
    // literal 16'hAA0 = 2,720, 3.8x the ceiling.
    parameter int REPLAY_TIMER_CYCLES =
        pcie_datalink_pkg::replay_timer_cycles(REPLAY_MPS_BYTES, REPLAY_LINK_WIDTH, CLK_PERIOD_NS),
    // sec 63 #7g-2 Q4: Base 2.1 sec 3.5.2.1 p.174 -- three replays proceed; the
    // fourth initiation rolls REPLAY_NUM 11b -> 00b and must RETRAIN the Link.
    // No DLL -> LTSSM retrain path exists yet (registered to the GTH/link-
    // recovery rung), so retry_management errors out there instead.  Was 2.
    parameter int MAX_REPLAY_ATTEMPTS = 3
) (
    input  logic                  clk_i,              // Clock signal
    input  logic                  rst_i,              // Reset signal
    //TLP AXIS inputs
    input  logic [DATA_WIDTH-1:0] s_tlp_axis_tdata,
    input  logic [KEEP_WIDTH-1:0] s_tlp_axis_tkeep,
    input  logic                  s_tlp_axis_tvalid,
    input  logic                  s_tlp_axis_tlast,
    input  logic [USER_WIDTH-1:0] s_tlp_axis_tuser,
    output logic                  s_tlp_axis_tready,
    //TLP AXIS output
    output logic [DATA_WIDTH-1:0] m_tlp_axis_tdata,
    output logic [KEEP_WIDTH-1:0] m_tlp_axis_tkeep,
    output logic                  m_tlp_axis_tvalid,
    output logic                  m_tlp_axis_tlast,
    output logic [USER_WIDTH-1:0] m_tlp_axis_tuser,
    input  logic                  m_tlp_axis_tready,
    //DLLP AXIS inputs
    // COMING FROM THE PHYSICAL LAYER
    input  logic [DATA_WIDTH-1:0] s_phy_axis_tdata,
    input  logic [KEEP_WIDTH-1:0] s_phy_axis_tkeep,
    input  logic                  s_phy_axis_tvalid,
    input  logic                  s_phy_axis_tlast,
    input  logic [USER_WIDTH-1:0] s_phy_axis_tuser,
    output logic                  s_phy_axis_tready,
    //PHY -> DLLP AXIS output
    // GOING TO THE PHYSICAL LAYER
    output logic [DATA_WIDTH-1:0] m_phy_axis_tdata,
    output logic [KEEP_WIDTH-1:0] m_phy_axis_tkeep,
    output logic                  m_phy_axis_tvalid,
    output logic                  m_phy_axis_tlast,
    output logic [USER_WIDTH-1:0] m_phy_axis_tuser,
    input  logic                  m_phy_axis_tready,
    //Configuration
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

    output logic [7:0] cfg_bus_number_o,
    output logic [4:0] cfg_device_number_o,
    output logic [2:0] cfg_function_number_o,

    output logic       ext_tag_enable_o,
    output logic       rcb_128b_o,
    output logic [2:0] max_read_request_size_o,
    output logic [2:0] max_payload_size_o,
    output logic       msix_enable_o,
    output logic       msix_mask_o,
    //Status
    input  logic       status_error_cor_i,
    input  logic       status_error_uncor_i,
    //Control and status
    input  logic       rx_cpl_stall_i,
    // sec 63 #7k: Base 2.1 sec 3.5.2.1 p.174 -- on REPLAY_NUM rollover "the
    // Transmitter signals the Physical Layer to retrain the Link, and waits for
    // the completion of retraining before proceeding with the replay".
    // link_retrain_req_o: a level, clk_i, held until retraining is seen.
    // link_retraining_i: the LTSSM is in Recovery or Configuration, already
    // synchronised to clk_i by the instantiating top; defaults to 0 (never
    // retraining) where no LTSSM exists.
    output logic       link_retrain_req_o,
    input  logic       link_retraining_i = 1'b0
);


  //localparam int SLAVE_COUNT = 2;
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

  //RETRY AXIS output
  logic            [(DATA_WIDTH)-1:0] phy_fc_axis_tdata;
  logic            [(KEEP_WIDTH)-1:0] phy_fc_axis_tkeep;
  logic                               phy_fc_axis_tvalid;
  logic                               phy_fc_axis_tlast;
  logic            [  USER_WIDTH-1:0] phy_fc_axis_tuser;
  logic                               phy_fc_axis_tready;
  //DLLP AXIS output
  logic            [(DATA_WIDTH)-1:0] phy_rx_axis_tdata;
  logic            [(KEEP_WIDTH)-1:0] phy_rx_axis_tkeep;
  logic                               phy_rx_axis_tvalid;
  logic                               phy_rx_axis_tlast;
  logic            [  USER_WIDTH-1:0] phy_rx_axis_tuser;
  logic                               phy_rx_axis_tready;
  //TLP AXIS output
  logic            [(DATA_WIDTH)-1:0] phy_tlp_axis_tdata;
  logic            [(KEEP_WIDTH)-1:0] phy_tlp_axis_tkeep;
  logic                               phy_tlp_axis_tvalid;
  logic                               phy_tlp_axis_tlast;
  logic            [  USER_WIDTH-1:0] phy_tlp_axis_tuser;
  logic                               phy_tlp_axis_tready;


  logic            [  DATA_WIDTH-1:0] cpl_from_cfg_tdata;
  logic            [  KEEP_WIDTH-1:0] cpl_from_cfg_tkeep;
  logic                               cpl_from_cfg_tvalid;
  logic                               cpl_from_cfg_tlast;
  logic            [  USER_WIDTH-1:0] cpl_from_cfg_tuser;
  logic                               cpl_from_cfg_tready;


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
  logic                               ack_nack;
  logic                               ack_nack_vld;
  logic                               ack_seq_num;
  //Ports
  logic                               init_flow_control;
  logic                               soft_reset;
  // sec 63 #7g-2 Q3: the REPLAY_TIMER's start event, observed where the DLL
  // hands a TLP to the PHY (see the block after the PHY arbiter).
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

  //dllp transmit
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


  //dllp receive
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
      
      // DLLP handler outputs
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
      // Input order is {receive DLLP, flow-control DLLP, TLP}; ACK/NAK and
      // other receive-generated DLLPs must win shared-PHY arbitration.
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

  // ===========================================================================
  // sec 63 #7g-2 step 2 (Kourosh Q3): the REPLAY_TIMER's start event -- "the
  // last Symbol of any TLP transmission or retransmission" (Base 2.1 sec
  // 3.5.2.1 p.170), taken at the last point the DLL owns: the handshake of a
  // TLP frame's last beat on m_phy_axis, the output of the arbiter above.  New
  // TLPs and retransmissions both leave through it.  tuser[1] is UserIsTlp
  // (axis_user_demux.sv); a link TLP's first beat carries its sequence number
  // as {tdata[3:0], tdata[15:8]} (the parse dllp2tlp.sv applies on receive),
  // and a link TLP is >= 5 beats, so its first beat is never its last and the
  // sequence always comes from the register captured on that first beat.
  // ===========================================================================
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

  always_ff @(posedge clk_i) begin
    if (rst_i || soft_reset)
      fc2_values_stored_reg <= 1'b0;
    else
      fc2_values_stored_reg <= fc2_values_stored;
  end


//   assign bus_num_o               = '0;
  assign ext_tag_enable_o        = '0;
  assign rcb_128b_o              = '0;
  assign max_read_request_size_o = '0;
  assign max_payload_size_o      = '0;
  assign msix_enable_o           = '0;
  assign msix_mask_o             = '0;
  assign fc_init_done            = fc2_values_stored && !fc2_values_stored_reg;

  //   initial begin
  //     $dumpfile("dllp_core.fst");
  //     $dumpvars(0, pcie_datalink_layer);
  //   end
endmodule
