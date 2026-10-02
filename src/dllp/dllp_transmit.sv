// ---------------------------------------------------------------------------
// dllp_transmit -- Data Link Layer TLP transmit path with retry buffer
//
//! @title dllp_transmit
//! @author Idris Somoye
//
// Purpose
//   Frames each TLP with its sequence number and LCRC once the peer's flow
//   control credits allow it (tlp2dllp), keeps a copy in the retry buffer
//   (retry_transmit) until an Ack or Nak acknowledges it, and replays it on a
//   Nak or a REPLAY_TIMER expiry (retry_management). An arbiter merges
//   replays and new TLPs onto m_axis, replays first.
//
// Interfaces
//   TLP input     s_axis_*: TLPs from pcie_datalink_layer's TLP arbiter.
//   TLP output    m_axis_*: framed TLPs, new and replayed, to
//                 pcie_datalink_layer's PHY arbiter.
//   Ack/Nak       ack_nack_i (1 = Ack), ack_nack_vld_i, ack_seq_num_i: a
//                 received Ack or Nak DLLP, from dllp_handler.
//   Sent          tlp_sent_i, tlp_sent_seq_i: a TLP's last beat has left the
//                 Data Link Layer; starts that TLP's REPLAY_TIMER.
//   Credits       tx_fc_*_i, update_fc_i: the peer's credit limits, taken by
//                 tlp2dllp while update_fc_i is high.
//   Retrain       link_retrain_req_o: REPLAY_NUM has rolled over and a Link
//                 retrain is requested. link_retraining_i: the LTSSM is in
//                 Recovery or Configuration; 0 where there is no LTSSM.
//
// Clock and reset
//   clk_i only. rst_i is synchronous and active high; pcie_datalink_layer
//   also asserts it while the link is down.
//
// Limitations
//   New TLPs are not blocked during a replay; the arbiter only keeps frames
//   whole. retry_transmit stores tlp2dllp's output on tvalid alone, without
//   m_axis_tlp2dllp_tready, so a beat held by the arbiter is stored again.
//   RAM_ADDR_WIDTH, RAM_DATA_WIDTH and S_COUNT reach submodules unused.
//
// References
//   PCIe Base Spec r2.1, §3.5.2.1
// ---------------------------------------------------------------------------
module dllp_transmit
  import pcie_datalink_pkg::*;
#(
    parameter int DATA_WIDTH       = 32,
    parameter int STRB_WIDTH       = DATA_WIDTH / 8,
    parameter int KEEP_WIDTH       = STRB_WIDTH,
    parameter int USER_WIDTH       = 1,
    parameter int S_COUNT          = 1,
    parameter int MAX_PAYLOAD_SIZE = 256,
    // Number of retry slots, each holding one TLP.
    parameter int RETRY_TLP_SIZE   = 3,
    // pcie_datalink_layer passes both, the timer derived from its CLK_PERIOD_NS
    // and a Table 3-4 row (PCIe Base Spec r2.1, §3.5.2.1). The default uses the
    // x1, 128-byte Max_Payload_Size entry and 8 ns, for standalone use.
    parameter int REPLAY_TIMER_CYCLES = pcie_datalink_pkg::replay_timer_cycles(128, 1, 8),
    parameter int MAX_REPLAY_ATTEMPTS = 3
) (
    input logic clk_i,
    input logic rst_i,
    // A TLP's last beat has left the Data Link Layer (pcie_datalink_layer's
    // m_phy_axis), with the sequence number from its first beat. It starts or
    // restarts that TLP's REPLAY_TIMER in retry_management.
    input logic        tlp_sent_i,
    input logic [11:0] tlp_sent_seq_i,


    // ---- TLP input ---------------------------------------------------------
    input  logic [DATA_WIDTH-1:0] s_axis_tdata,
    input  logic [KEEP_WIDTH-1:0] s_axis_tkeep,
    input  logic                  s_axis_tvalid,
    input  logic                  s_axis_tlast,
    input  logic [USER_WIDTH-1:0] s_axis_tuser,
    output logic                  s_axis_tready,
    // ---- framed TLP output -------------------------------------------------
    output logic [DATA_WIDTH-1:0] m_axis_tdata,
    output logic [KEEP_WIDTH-1:0] m_axis_tkeep,
    output logic                  m_axis_tvalid,
    output logic                  m_axis_tlast,
    output logic [USER_WIDTH-1:0] m_axis_tuser,
    input  logic                  m_axis_tready,
    // ---- received Ack or Nak -----------------------------------------------
    input  logic                  ack_nack_i,
    input  logic                  ack_nack_vld_i,
    input  logic [          11:0] ack_seq_num_i,
    // ---- peer credit limits ------------------------------------------------
    input  logic [           7:0] tx_fc_ph_i,
    input  logic [          11:0] tx_fc_pd_i,
    input  logic [           7:0] tx_fc_nph_i,
    input  logic [          11:0] tx_fc_npd_i,
    input  logic [           7:0] tx_fc_cplh_i,
    input  logic [          11:0] tx_fc_cpld_i,
    input  logic                  update_fc_i,
    // On a REPLAY_NUM rollover the Link is retrained before the replay
    // proceeds (PCIe Base Spec r2.1, §3.5.2.1). link_retrain_req_o is
    // retry_management's retry_err_o; link_retraining_i goes down to it.
    output logic                  link_retrain_req_o,
    input  logic                  link_retraining_i = 1'b0
);


  // MaxTlpHdrSizeDW through RAM_ADDR_WIDTH compute RAM_DATA_WIDTH and
  // RAM_ADDR_WIDTH, which go to retry_management, retry_transmit and
  // tlp2dllp; none of them uses either. axis_retry_fifo sizes the retry buffer.
  parameter int MaxTlpHdrSizeDW = 4;
  parameter int RAM_DATA_WIDTH = DATA_WIDTH;
  parameter int MaxTlpTotalSizeDW = MaxTlpHdrSizeDW + MAX_PAYLOAD_SIZE + 1;
  parameter int MinRxBufferSize = MaxTlpTotalSizeDW * (RETRY_TLP_SIZE);
  parameter int RAM_ADDR_WIDTH = $clog2(MinRxBufferSize);
  parameter int KEEP_ENABLE = (DATA_WIDTH > 8);
  parameter int ID_ENABLE = 0;
  parameter int ID_WIDTH = 8;
  parameter int DEST_ENABLE = 0;
  parameter int DEST_WIDTH = 8;
  parameter int USER_ENABLE = 1;
  parameter int LAST_ENABLE = 1;
  parameter int ARB_TYPE_ROUND_ROBIN = 0;
  parameter int ARB_LSB_HIGH_PRIORITY = 1;


  // retry_transmit's replayed frames
  logic [  (DATA_WIDTH)-1:0] m_axis_retry_tdata;
  logic [  (KEEP_WIDTH)-1:0] m_axis_retry_tkeep;
  logic                      m_axis_retry_tvalid;
  logic                      m_axis_retry_tlast;
  logic [    USER_WIDTH-1:0] m_axis_retry_tuser;
  logic                      m_axis_retry_tready;
  // tlp2dllp's framed TLPs
  logic [  (DATA_WIDTH)-1:0] m_axis_tlp2dllp_tdata;
  logic [  (KEEP_WIDTH)-1:0] m_axis_tlp2dllp_tkeep;
  logic                      m_axis_tlp2dllp_tvalid;
  logic                      m_axis_tlp2dllp_tlast;
  logic [    USER_WIDTH-1:0] m_axis_tlp2dllp_tuser;
  logic                      m_axis_tlp2dllp_tready;
  // The sequence number of the TLP tlp2dllp is framing (its seq_num_o), not
  // ACKD_SEQ.
  logic [              11:0] ackd_transmit_seq;
  logic                      dllp_valid;
  logic                      retry_available;
  logic [               7:0] retry_index;
  logic                      retry_err;
  assign link_retrain_req_o = retry_err;
  logic [RETRY_TLP_SIZE-1:0] retry_valid;
  logic [RETRY_TLP_SIZE-1:0] retry_ack;
  logic [RETRY_TLP_SIZE-1:0] retry_complete;

  retry_management #(
      .DATA_WIDTH(DATA_WIDTH),
      .STRB_WIDTH(STRB_WIDTH),
      .KEEP_WIDTH(KEEP_WIDTH),
      .USER_WIDTH(USER_WIDTH),
      .S_COUNT(1),
      .RAM_ADDR_WIDTH(RAM_ADDR_WIDTH),
      .RAM_DATA_WIDTH(RAM_DATA_WIDTH),
      .MAX_PAYLOAD_SIZE(MAX_PAYLOAD_SIZE),
      .RETRY_TLP_SIZE(RETRY_TLP_SIZE),
      .REPLAY_TIMER_CYCLES(REPLAY_TIMER_CYCLES),
      .MAX_REPLAY_ATTEMPTS(MAX_REPLAY_ATTEMPTS)
  ) retry_management_inst (
      .clk_i            (clk_i),
      .rst_i            (rst_i),
      .tlp_sent_i       (tlp_sent_i),
      .tlp_sent_seq_i   (tlp_sent_seq_i),
      //seq num
      .tx_seq_num_i     (ackd_transmit_seq),
      .tx_valid_i       (dllp_valid),
      //retry
      .retry_available_o(retry_available),
      .retry_index_o    (retry_index),
      .retry_err_o      (retry_err),
      .link_retraining_i(link_retraining_i),
      .retry_valid_o    (retry_valid),
      .retry_ack_i      (retry_ack),
      .retry_complete_i (retry_complete),
      //ack/nack from dllp
      .ack_nack_i       (ack_nack_i),
      .ack_nack_vld_i   (ack_nack_vld_i),
      .ack_seq_num_i    (ack_seq_num_i)
  );

  retry_transmit #(
      .DATA_WIDTH(DATA_WIDTH),
      .STRB_WIDTH(STRB_WIDTH),
      .KEEP_WIDTH(KEEP_WIDTH),
      .USER_WIDTH(USER_WIDTH),
      .S_COUNT(S_COUNT),
      .MAX_PAYLOAD_SIZE(MAX_PAYLOAD_SIZE),
      .RAM_DATA_WIDTH(RAM_DATA_WIDTH),
      .RETRY_TLP_SIZE(RETRY_TLP_SIZE),
      .RAM_ADDR_WIDTH(RAM_ADDR_WIDTH)
  ) retry_transmit_inst (
      .clk_i            (clk_i),
      .rst_i            (rst_i),
      .retry_valid_i    (retry_valid),
      .retry_ack_o      (retry_ack),
      .retry_complete_o (retry_complete),
      .retry_available_i(retry_available),
      .retry_index_i    (retry_index),
      //tlp2dllp's output, observed and stored for replay in slot retry_index.
      //retry_index can change while a frame is written: at the end of the
      //dllp_valid cycle, and when an Ack or Nak frees a lower slot. Beats on
      //m_axis_tlp2dllp after the change go to the new slot.
      .s_axis_tdata     (m_axis_tlp2dllp_tdata),
      .s_axis_tkeep     (m_axis_tlp2dllp_tkeep),
      .s_axis_tvalid    (m_axis_tlp2dllp_tvalid),
      .s_axis_tlast     (m_axis_tlp2dllp_tlast),
      .s_axis_tuser     (m_axis_tlp2dllp_tuser),
      .s_axis_tready    (),
      //axis out, to the arbiter
      .m_axis_tdata     (m_axis_retry_tdata),
      .m_axis_tkeep     (m_axis_retry_tkeep),
      .m_axis_tvalid    (m_axis_retry_tvalid),
      .m_axis_tlast     (m_axis_retry_tlast),
      .m_axis_tuser     (m_axis_retry_tuser),
      .m_axis_tready    (m_axis_retry_tready)
  );


  tlp2dllp #(
      .DATA_WIDTH(DATA_WIDTH),
      .KEEP_WIDTH(KEEP_WIDTH),
      .USER_WIDTH(USER_WIDTH),
      .RAM_ADDR_WIDTH(RAM_ADDR_WIDTH),
      .RAM_DATA_WIDTH(RAM_DATA_WIDTH),
      .MAX_PAYLOAD_SIZE(MAX_PAYLOAD_SIZE),
      .S_COUNT(S_COUNT)
  ) tlp2dllp_inst (
      .clk_i            (clk_i),
      .rst_i            (rst_i),
      //axis tlp in
      .s_axis_tdata     (s_axis_tdata),
      .s_axis_tkeep     (s_axis_tkeep),
      .s_axis_tvalid    (s_axis_tvalid),
      .s_axis_tlast     (s_axis_tlast),
      .s_axis_tuser     (s_axis_tuser),
      .s_axis_tready    (s_axis_tready),
      //axis out, to the arbiter and retry_transmit
      .m_axis_tdata     (m_axis_tlp2dllp_tdata),
      .m_axis_tkeep     (m_axis_tlp2dllp_tkeep),
      .m_axis_tvalid    (m_axis_tlp2dllp_tvalid),
      .m_axis_tlast     (m_axis_tlp2dllp_tlast),
      .m_axis_tuser     (m_axis_tlp2dllp_tuser),
      .m_axis_tready    (m_axis_tlp2dllp_tready),
      //sequence number
      .seq_num_o        (ackd_transmit_seq),
      .dllp_valid_o     (dllp_valid),
      .retry_available_i(retry_available),
      .retry_index_i    (retry_index),
      //flow control
      .tx_fc_ph_i       (tx_fc_ph_i),
      .tx_fc_pd_i       (tx_fc_pd_i),
      .tx_fc_nph_i      (tx_fc_nph_i),
      .tx_fc_npd_i      (tx_fc_npd_i),
      .tx_fc_cplh_i     (tx_fc_cplh_i),
      .tx_fc_cpld_i     (tx_fc_cpld_i),
      .update_fc_i      (update_fc_i)
  );


  // Port 0 is retry_transmit and port 1 tlp2dllp. With fixed priority and
  // ARB_LSB_HIGH_PRIORITY = 1 a replay is granted before a new TLP, the order
  // the transmit priority list recommends (PCIe Base Spec r2.1, §3.5.2.1).
  // A grant holds until tlast, so frames never interleave.
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
  ) arbiter_mux_inst (
      .clk          (clk_i),
      .rst          (rst_i),
      // AXI inputs
      .s_axis_tdata ({m_axis_tlp2dllp_tdata, m_axis_retry_tdata}),
      .s_axis_tkeep ({m_axis_tlp2dllp_tkeep, m_axis_retry_tkeep}),
      .s_axis_tvalid({m_axis_tlp2dllp_tvalid, m_axis_retry_tvalid}),
      .s_axis_tready({m_axis_tlp2dllp_tready, m_axis_retry_tready}),
      .s_axis_tlast ({m_axis_tlp2dllp_tlast, m_axis_retry_tlast}),
      .s_axis_tid   (),
      .s_axis_tdest (),
      .s_axis_tuser ({m_axis_tlp2dllp_tuser, m_axis_retry_tuser}),
      // AXI output
      .m_axis_tdata (m_axis_tdata),
      .m_axis_tkeep (m_axis_tkeep),
      .m_axis_tvalid(m_axis_tvalid),
      .m_axis_tready(m_axis_tready),
      .m_axis_tlast (m_axis_tlast),
      .m_axis_tid   (),
      .m_axis_tdest (),
      .m_axis_tuser (m_axis_tuser)
  );


endmodule
