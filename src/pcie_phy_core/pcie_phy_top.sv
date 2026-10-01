// ---------------------------------------------------------------------------
// pcie_phy_top -- Data Link Layer, LTSSM and logical PHY behind one PIPE port
//
// Original author: Idris Somoye
// Modified by: Kourosh Ghahramani
// Silicon Systems Research Lab, University of Washington
//
// Purpose
//   Joins pcie_datalink_layer, pcie_ltssm_downstream, phy_receive and
//   phy_transmit between a PIPE PHY such as PG239 and a TLP stream. It also
//   records receiver detection per lane, carries link_up and the retrain
//   handshake across clock domains, and drives the PG239 assist signals.
//
// Interfaces
//   TLP stream    s_tlp_axis_*, m_tlp_axis_*: to and from the Transaction
//                 Layer, through pcie_datalink_layer.
//   Status        fc_*: pcie_datalink_layer's flow-control outputs.
//                 cfg_*_number_o, link_up_o, pipe_width_o, ltssm_debug_state.
//   PIPE          phy_tx*, phy_rx*, PHY command and status: PIPE_DATA_WIDTH
//                 data bits and PIPE_DATA_WIDTH / 8 K flags per lane.
//   Assist        as_mac_in_detect, as_cdr_hold_req (PG239, Table 14).
//   Unused        tx_elec_idle, phy_ready_en, phy_rxdata_valid and the
//                 equalisation inputs are not read; phy_txswing and the
//                 equalisation outputs are not driven.
//
// Clock and reset
//   clk_i runs pcie_datalink_layer; the LTSSM and phy_receive, up to its
//   output FIFO, run on pipe_rx_usr_clk_i; phy_transmit takes all three
//   clocks. The DLL and the LTSSM derive their timers from the one
//   CLK_PERIOD_NS. rst_i is active high. phy_phystatus_rst, high until
//   the PHY's resets complete (PG239, Table 10), also resets the LTSSM,
//   phy_receive, phy_transmit, lane_status and as_mac_in_detect.
//
// Structure
//   Ports and declarations
//   link_up into the clk_i domain
//   Retrain handshake between pcie_datalink_layer and the LTSSM
//   Receiver detection
//   Receive path, transmit path and LTSSM
//   PG239 assist signals
//   Data Link Layer
//
// References
//   PCIe Base Spec r2.1, §3.5.2.1
//   PCIe Base Spec r2.1, §4.2.6.5
//   PG239, Table 7: RX Data Signals for UltraScale+ Devices
//   PG239, Table 9: Command Signals
//   PG239, Table 10: Status Signals
//   PG239, Table 14: Assist Signal
// ---------------------------------------------------------------------------
module pcie_phy_top
  import pcie_phy_pkg::*;
#(
    parameter int CLK_RATE      = 0, parameter int CLK_PERIOD_NS = (CLK_RATE != 0) ? (1000 / ((CLK_RATE != 0) ? CLK_RATE : 1)) : 8, //! Clock period in ns for the DLL and LTSSM timers; if not set, 1000 / CLK_RATE (MHz) when CLK_RATE is non-zero, else 8
    parameter int MAX_NUM_LANES = 1,               //! Maximum number of lanes module can support
    parameter int DATA_WIDTH    = 32,              //! AXIS width to the DLL, not the PIPE width
    // Per-lane PIPE data width at phy_txdata and phy_rxdata, with
    // PIPE_DATA_WIDTH / 8 K flags. 16 at Gen1: bits 31:16 are used at Gen3
    // only and are ignored at Gen1 and Gen2 (PG239, Table 7). phy_transmit and
    // phy_receive convert between it and their 32-bit, 4-K-flag container.
    parameter int PIPE_DATA_WIDTH = 16,
    parameter int STRB_WIDTH    = DATA_WIDTH / 8,
    parameter int KEEP_WIDTH    = STRB_WIDTH,
    parameter int USER_WIDTH    = 5,
    // IS_ROOT_PORT, LINK_NUM and SIM_FAST_LINK go to pcie_ltssm_downstream;
    // IS_UPSTREAM, CROSSLINK_EN and UPCONFIG_EN are not used.
    parameter int IS_ROOT_PORT = 0,
    parameter int LINK_NUM      = 0,
    parameter int IS_UPSTREAM   = 0,               // not used
    parameter int CROSSLINK_EN  = 0,               // not used
    parameter int UPCONFIG_EN   = 0,               // not used
    parameter int SIM_FAST_LINK = 0                //shorten training only in simulation
) (
    input  logic                                    clk_i,              //! Data Link Layer clock
    input  logic                                    rst_i,              //! Reset signal
    input  logic                                    en_i,
    input  logic                                    pipe_rx_usr_clk_i,
    input  logic                                    pipe_tx_usr_clk_i,
    // ---- Data Link Layer flow-control status -------------------------------
    // pcie_datalink_layer's eight fc_* outputs, passed through. A Transaction
    // Layer needs more than fc_initialized_o: tlp_credit_manager loads its
    // credit limits only on an fc_update_valid_o pulse, and until one arrives
    // tlp_layer sends no TLP.
    output logic                                    fc_initialized_o,
    output logic                                    fc_update_valid_o,
    output logic [                             7:0] fc_ph_o,
    output logic [                            11:0] fc_pd_o,
    output logic [                             7:0] fc_nph_o,
    output logic [                            11:0] fc_npd_o,
    output logic [                             7:0] fc_cplh_o,
    output logic [                            11:0] fc_cpld_o,
    // ---- PIPE transmit data ------------------------------------------------
    output logic [(MAX_NUM_LANES*PIPE_DATA_WIDTH)-1:0] phy_txdata,
    output logic [               MAX_NUM_LANES-1:0] phy_txdata_valid,
    output logic [(MAX_NUM_LANES*PIPE_DATA_WIDTH/8)-1:0] phy_txdatak,
    output logic [               MAX_NUM_LANES-1:0] phy_txstart_block,
    output logic [           (2*MAX_NUM_LANES)-1:0] phy_txsync_header,
    // ---- PIPE receive data -------------------------------------------------
    input  logic [(MAX_NUM_LANES*PIPE_DATA_WIDTH)-1:0] phy_rxdata,
    input  logic [               MAX_NUM_LANES-1:0] phy_rxdata_valid,   // not read; Gen3 and above
    input  logic [(MAX_NUM_LANES*PIPE_DATA_WIDTH/8)-1:0] phy_rxdatak,
    input  logic [               MAX_NUM_LANES-1:0] phy_rxstart_block,
    input  logic [           (2*MAX_NUM_LANES)-1:0] phy_rxsync_header,
    // PHY Command
    output wire                                     phy_txdetectrx,
    output wire  [               MAX_NUM_LANES-1:0] phy_txelecidle,
    output wire  [               MAX_NUM_LANES-1:0] phy_txcompliance,
    output wire  [               MAX_NUM_LANES-1:0] phy_rxpolarity,
    output wire  [                             1:0] phy_powerdown,
    output wire  [                             2:0] phy_rate,


    // PHY Status
    input  wire [     MAX_NUM_LANES-1:0] phy_rxvalid,
    input  wire [     MAX_NUM_LANES-1:0] phy_phystatus,
    input  wire                          phy_phystatus_rst,
    input  wire [     MAX_NUM_LANES-1:0] phy_rxelecidle,
    (* mark_debug = "true", keep = "true" *) input  wire [ (MAX_NUM_LANES*3)-1:0] phy_rxstatus,
    // TX Driver; phy_txswing is not driven
    output wire [                   2:0] phy_txmargin,
    output wire                          phy_txswing,
    output wire                          phy_txdeemph,
    // TX Equalization (Gen3/4): outputs not driven, inputs not read
    output wire [ (MAX_NUM_LANES*2)-1:0] phy_txeq_ctrl,
    output wire [ (MAX_NUM_LANES*4)-1:0] phy_txeq_preset,
    output wire [ (MAX_NUM_LANES*6)-1:0] phy_txeq_coeff,
    input  wire [                   5:0] phy_txeq_fs,
    input  wire [                   5:0] phy_txeq_lf,
    input  wire [(MAX_NUM_LANES*18)-1:0] phy_txeq_new_coeff,
    input  wire [     MAX_NUM_LANES-1:0] phy_txeq_done,
    // RX Equalization (Gen3/4): outputs not driven, inputs not read
    output wire [ (MAX_NUM_LANES*2)-1:0] phy_rxeq_ctrl,
    output wire [ (MAX_NUM_LANES*4)-1:0] phy_rxeq_txpreset,
    input  wire [     MAX_NUM_LANES-1:0] phy_rxeq_preset_sel,
    input  wire [(MAX_NUM_LANES*18)-1:0] phy_rxeq_new_txcoeff,
    input  wire [     MAX_NUM_LANES-1:0] phy_rxeq_adapt_done,
    input  wire [     MAX_NUM_LANES-1:0] phy_rxeq_done,
    output wire [                 8-1:0] pipe_width_o,


    output logic [7:0] cfg_bus_number_o,
    output logic [4:0] cfg_device_number_o,
    output logic [2:0] cfg_function_number_o,

    // PG239 assist signals
    output reg as_mac_in_detect,
    output reg as_cdr_hold_req,

    // Debug output: the LTSSM state in bits 19:0

    output wire [20:0] ltssm_debug_state,

    // Bringup Control Inputs: not read
    input wire tx_elec_idle,
    input wire phy_ready_en,


    output logic link_up_o,


    // ---- TLP stream from the Transaction Layer -----------------------------
    input  logic [DATA_WIDTH-1:0] s_tlp_axis_tdata,
    input  logic [KEEP_WIDTH-1:0] s_tlp_axis_tkeep,
    input  logic                  s_tlp_axis_tvalid,
    input  logic                  s_tlp_axis_tlast,
    input  logic [USER_WIDTH-1:0] s_tlp_axis_tuser,
    output logic                  s_tlp_axis_tready,
    // ---- TLP stream to the Transaction Layer -------------------------------
    output logic [DATA_WIDTH-1:0] m_tlp_axis_tdata,
    output logic [KEEP_WIDTH-1:0] m_tlp_axis_tkeep,
    output logic                  m_tlp_axis_tvalid,
    output logic                  m_tlp_axis_tlast,
    output logic [USER_WIDTH-1:0] m_tlp_axis_tuser,
    input  logic                  m_tlp_axis_tready
);


  // Sizing passed to pcie_datalink_layer.
  parameter int RX_FIFO_SIZE = 3;
  parameter int RETRY_TLP_SIZE = 3;
  parameter int MAX_PAYLOAD_SIZE = 256;


  // link_up is the LTSSM's, on pipe_rx_usr_clk_i; link_up_100MHz is its copy
  // on clk_i, whatever clk_i's period. symbol6, lane_number, link_number,
  // training_ctrl and rate_id are never used.
  logic                                      link_up;
  logic                                      link_up_100MHz;
  ts_symbol6_union_t [    MAX_NUM_LANES-1:0] symbol6;
  logic              [(MAX_NUM_LANES*8)-1:0] lane_number;
  logic              [(MAX_NUM_LANES*8)-1:0] link_number;
  (* mark_debug = "true", keep = "true" *) logic              [    MAX_NUM_LANES-1:0] ts1_valid;
  (* mark_debug = "true", keep = "true" *) logic              [    MAX_NUM_LANES-1:0] ts2_valid;
  (* mark_debug = "true", keep = "true" *) logic              [    MAX_NUM_LANES-1:0] idle_valid;
  logic              [    MAX_NUM_LANES-1:0] polarity_inverted;
  training_ctrl_t    [    MAX_NUM_LANES-1:0] training_ctrl;
  rate_speed_e                               curr_data_rate;
  // One Ordered Set per lane, so each lane carries its own Lane Number.
  pcie_ordered_set_t [    MAX_NUM_LANES-1:0] ordered_set;
  (* mark_debug = "true", keep = "true" *) logic                                      ordered_set_tranmitted;
  logic                                      send_ordered_set;
  rate_id_t          [    MAX_NUM_LANES-1:0] rate_id;
  logic              [                  5:0] pipe_width;
  // A local register despite its _i suffix; see Receiver detection below.
  logic              [                  5:0] num_active_lanes_i;

  assign pipe_width_o = pipe_width;


  pcie_ordered_set_t [MAX_NUM_LANES-1:0] rx_ordered_set;
  logic              [   DATA_WIDTH-1:0] m_dllp_axis_tdata;
  logic              [   KEEP_WIDTH-1:0] m_dllp_axis_tkeep;
  logic                                  m_dllp_axis_tvalid;
  logic                                  m_dllp_axis_tlast;
  logic              [   USER_WIDTH-1:0] m_dllp_axis_tuser;
  logic                                  m_dllp_axis_tready;


  logic              [   DATA_WIDTH-1:0] s_dllp_axis_tdata;
  logic              [   KEEP_WIDTH-1:0] s_dllp_axis_tkeep;
  logic                                  s_dllp_axis_tvalid;
  logic                                  s_dllp_axis_tlast;
  logic              [   USER_WIDTH-1:0] s_dllp_axis_tuser;
  logic                                  s_dllp_axis_tready;
  (* mark_debug = "true", keep = "true" *) gen_os_struct_t                        gen_os_ctrl;
  // Driven by the LTSSM and never read.
  logic              [MAX_NUM_LANES-1:0] active_lanes;
  (* mark_debug = "true", keep = "true" *) logic              [MAX_NUM_LANES-1:0] lane_status;

  logic                                       phy_txdetectrx_detect_upper_edge_r;
  logic                                       phy_txdetectrx_detect_upper_edge;

  // -------------------------------------------------------------------------
  // link_up into the clk_i domain
  // -------------------------------------------------------------------------
  // A 1-bit async_fifo, written and read on every clock, carries link_up from
  // pipe_rx_usr_clk_i into clk_i for pcie_datalink_layer's phy_link_up_i.
  // link_up_o is the pipe_rx_usr_clk_i level itself.
  async_fifo #(
        .DSIZE(1),
        .ASIZE(2)
  ) os_transmitted_async_fifo_inst (
      .wclk(pipe_rx_usr_clk_i),
      .wrst_n(!rst_i ),
      .winc('1),
      .wdata(link_up),
      .wfull(),
      .awfull(),
      .rclk(clk_i),
      .rrst_n(!rst_i),
      .rinc('1),
      .rdata(link_up_100MHz),
      .rempty(),
      .arempty()
  );

  // 000b, Gen1 (PG239, Table 9), at gen1. The subtraction does not map the
  // other rates: gen2 gives 010b, the Gen3 code.
  assign phy_rate  = curr_data_rate - 1'b1;
  assign link_up_o = link_up;

  // -------------------------------------------------------------------------
  // Retrain handshake between pcie_datalink_layer and the LTSSM
  // -------------------------------------------------------------------------
  // When REPLAY_NUM rolls over, the Data Link Layer has the Physical Layer
  // retrain the Link and waits for retraining to complete (PCIe Base Spec
  // r2.1, §3.5.2.1); in L0 the LTSSM goes to Recovery when directed
  // (§4.2.6.5). The DLL runs on clk_i and the LTSSM on pipe_rx_usr_clk_i,
  // so both directions are levels through two-flop synchronisers. The DLL
  // holds its request until it sees ltssm_retraining and then drops it, so
  // the request survives the crossing, and the LTSSM, which reads
  // recovery_i only in L0, retrains once per request. ltssm_retraining
  // also holds the DLL's REPLAY_TIMER, which must not advance in Recovery
  // or Configuration (§3.5.2.1).
  logic       dll_retrain_req;   // clk_i: REPLAY_NUM rolled over, retrain requested
  logic [1:0] retrain_req_sync;  // -> pipe_rx_usr_clk_i, onto the LTSSM's recovery_i
  logic       ltssm_retraining;  // pipe_rx_usr_clk_i: LTSSM in Recovery or Configuration
  logic [1:0] retraining_sync;   // -> clk_i, onto the DLL's link_retraining_i
  assign ltssm_retraining = (ltssm_debug_state[4:0] == 5'b00100) ||  // the Recovery family
                            (ltssm_debug_state[4:0] == 5'b00011);    // the Configuration family
  always_ff @(posedge pipe_rx_usr_clk_i) begin : retrain_req_synchroniser
    if (rst_i) retrain_req_sync <= '0;
    else       retrain_req_sync <= {retrain_req_sync[0], dll_retrain_req};
  end
  always_ff @(posedge clk_i) begin : retraining_synchroniser
    if (rst_i) retraining_sync <= '0;
    else       retraining_sync <= {retraining_sync[0], ltssm_retraining};
  end


  // -------------------------------------------------------------------------
  // Receiver detection
  // -------------------------------------------------------------------------
  // lane_status records, per lane, that phy_phystatus pulsed with rxstatus
  // 011b, Receiver detected (PG239, Table 10). It is cleared by reset and at
  // the start of each detection, the rising edge of phy_txdetectrx.
  // num_active_lanes_i is one more than the highest detected lane. The
  // LTSSM reads lane_status; phy_receive and phy_transmit read
  // num_active_lanes_i.
  always_comb begin : detect_phy_txdetectrx_upper_edge
    if (~phy_txdetectrx_detect_upper_edge_r && phy_txdetectrx) begin
        phy_txdetectrx_detect_upper_edge = '1;
    end
    else begin
        phy_txdetectrx_detect_upper_edge = '0;
    end
  end

  always_ff @(posedge pipe_rx_usr_clk_i) begin 
    if (rst_i) begin
      phy_txdetectrx_detect_upper_edge_r <= 0;
    end else begin
      phy_txdetectrx_detect_upper_edge_r <= phy_txdetectrx;
    end
  end

  always_ff @(posedge pipe_rx_usr_clk_i) begin
    if (rst_i || phy_phystatus_rst || phy_txdetectrx_detect_upper_edge) begin
      lane_status        <= '0;
      num_active_lanes_i <= '0;
    end else begin
      for (int i = 0; i < MAX_NUM_LANES; i++) begin
        if (phy_phystatus[i] && phy_rxstatus[i*3+:3] == 3'b011) begin
          lane_status[i] <= '1;
        end
      end
      for (int i = 0; i < MAX_NUM_LANES; i++) begin
        if (lane_status[i]) begin
          num_active_lanes_i <= i + 1;
        end
      end
    end
  end

  // -------------------------------------------------------------------------
  // Receive path, transmit path and LTSSM
  // -------------------------------------------------------------------------
  // The LTSSM runs on pipe_rx_usr_clk_i, as does phy_receive up to its output
  // FIFO, which hands packets to pcie_datalink_layer on clk_i. pipe_width
  // comes from phy_transmit and curr_data_rate from the LTSSM.
  phy_receive #(
      .CLK_RATE     (1000 / CLK_PERIOD_NS),  // not used by phy_receive
      .MAX_NUM_LANES(MAX_NUM_LANES),
      .DATA_WIDTH   (DATA_WIDTH),
      .STRB_WIDTH   (STRB_WIDTH),
      .KEEP_WIDTH   (KEEP_WIDTH),
      .USER_WIDTH   (USER_WIDTH),
      .PIPE_DATA_WIDTH(PIPE_DATA_WIDTH)
  ) phy_receive_inst (
      .clk_i             (clk_i),
      .rst_i             (rst_i || phy_phystatus_rst),
      .pipe_rx_usr_clk_i (pipe_rx_usr_clk_i),
      .en_i              (en_i),
      .link_up_i         (link_up),
      .pipe_data_i       (phy_rxdata),
      // The Gen1 qualifier is phy_rxvalid: symbol lock and valid data, Gen1
      // and Gen2 only (PG239, Table 10). phy_rxdata_valid is for Gen3 and
      // above (PG239, Table 7) and is not read.
      .pipe_data_valid_i (phy_rxvalid),
      .pipe_data_k_i     (phy_rxdatak),
      .pipe_sync_header_i(phy_rxsync_header),
      .pipe_block_start_i(phy_rxstart_block),
      .pipe_width_i      (pipe_width),
      .num_active_lanes_i(num_active_lanes_i),
      .ts1_valid_o       (ts1_valid),
      .ts2_valid_o       (ts2_valid),
      .idle_valid_o      (idle_valid),
      .polarity_inverted_o(polarity_inverted),
      .ordered_set_o     (rx_ordered_set),
      .curr_data_rate_i  (curr_data_rate),
      .m_dllp_axis_tdata (m_dllp_axis_tdata),
      .m_dllp_axis_tkeep (m_dllp_axis_tkeep),
      .m_dllp_axis_tvalid(m_dllp_axis_tvalid),
      .m_dllp_axis_tlast (m_dllp_axis_tlast),
      .m_dllp_axis_tuser (m_dllp_axis_tuser),
      .m_dllp_axis_tready(m_dllp_axis_tready)
  );


  phy_transmit #(
      .CLK_RATE     (1000 / CLK_PERIOD_NS),  // not used by phy_transmit
      .MAX_NUM_LANES(MAX_NUM_LANES),
      .DATA_WIDTH   (DATA_WIDTH),
      .STRB_WIDTH   (STRB_WIDTH),
      .KEEP_WIDTH   (KEEP_WIDTH),
      .USER_WIDTH   (USER_WIDTH),
      .PIPE_DATA_WIDTH(PIPE_DATA_WIDTH)
  ) phy_transmit_inst (
      .clk_i                   (clk_i),
      .pipe_rx_usr_clk_i       (pipe_rx_usr_clk_i),
      .pipe_tx_usr_clk_i       (pipe_tx_usr_clk_i),
      .rst_i                   (rst_i || phy_phystatus_rst),
      .en_i                    (en_i),
      .link_up_i               (link_up),
      .pipe_data_o             (phy_txdata),
      .pipe_data_valid_o       (phy_txdata_valid),
      .pipe_data_k_o           (phy_txdatak),
      .pipe_sync_header_o      (phy_txsync_header),
      .pipe_txstart_block_o    (phy_txstart_block),
      .pipe_width_o            (pipe_width),
      .gen_os_ctrl_i           (gen_os_ctrl),
      .num_active_lanes_i      (num_active_lanes_i),
      .send_ordered_set_i      (send_ordered_set),
      // The LTSSM's per-lane Ordered Sets; each carries its lane's Lane Number.
      .ordered_set_i           (ordered_set),
      .curr_data_rate_i        (curr_data_rate),
      .ordered_set_tranmitted_o(ordered_set_tranmitted),
      .s_dllp_axis_tdata       (s_dllp_axis_tdata),
      .s_dllp_axis_tkeep       (s_dllp_axis_tkeep),
      .s_dllp_axis_tvalid      (s_dllp_axis_tvalid),
      .s_dllp_axis_tlast       (s_dllp_axis_tlast),
      .s_dllp_axis_tuser       (s_dllp_axis_tuser),
      .s_dllp_axis_tready      (s_dllp_axis_tready)
  );


  pcie_ltssm_downstream #(
      .CLK_PERIOD_NS(CLK_PERIOD_NS),
      .MAX_NUM_LANES(MAX_NUM_LANES),
      .DATA_WIDTH   (DATA_WIDTH),
      .KEEP_WIDTH   (KEEP_WIDTH),
      .USER_WIDTH   (USER_WIDTH),
      .SIM_FAST_LINK(SIM_FAST_LINK),
      .IS_ROOT_PORT (IS_ROOT_PORT),
      .LINK_NUM     (LINK_NUM)
  ) pcie_ltssm_downstream_inst (
      .clk_i              (pipe_rx_usr_clk_i),
      .rst_i              (rst_i || phy_phystatus_rst),
      .en_i               (en_i),
      .link_up_o          (link_up),
      .is_timeout_i       (),
      .recovery_i         (retrain_req_sync[1]),  // the retrain request, synchronised
      .error_o            (),
      .success_o          (),
      .error_loopback_o   (),
      .error_disable_o    (),
      .ts1_valid_i        (ts1_valid),
      .ts2_valid_i        (ts2_valid),
      .idle_valid_i       (idle_valid),
      .polarity_inverted_i(polarity_inverted),
      .phy_rxstatus_i     (phy_rxstatus),
      .phy_phystatus_i    (phy_phystatus),
      .phy_phystatus_rst_i(phy_phystatus_rst),
      .phy_txdetectrx_o   (phy_txdetectrx),
      .active_lanes_o     (active_lanes),
      .phy_txelecidle_o   (phy_txelecidle),
      .phy_txdeemph_o     (phy_txdeemph),
      .phy_powerdown_o    (phy_powerdown),
      .phy_txcompliance_o (phy_txcompliance),
      .phy_rxpolarity_o   (phy_rxpolarity),
      .phy_txmargin_o     (phy_txmargin),

      .lanes_ts2_satisfied_i   (),
      .config_copmlete_ts2_i   (),
      .from_l0_i               (),
      .receiver_detected_i     (lane_status),
      .phy_rxelecidle_i        (phy_rxelecidle),
      .tx_enter_elec_idle_o    (),
      .goto_cfg_o              (),
      .goto_detect_o           (),
      .gen_os_ctrl_o           (gen_os_ctrl),
      .preset_coeff_o          (),
      .extended_synch_i        (),
      .directed_speed_change_i ('0),
      .lane_status_i           (lane_status),
      .curr_data_rate_o        (curr_data_rate),
      .data_rate_o             (),
      .ltssm_state_o           (ltssm_debug_state[19:0]),
      .ordered_set_i           (rx_ordered_set),
      .ordered_set_tranmitted_i(ordered_set_tranmitted),
      .ordered_set_o           (ordered_set),
      .send_ordered_set_o      (send_ordered_set),
      .changed_speed_recovery_o()
  );
  // The LTSSM state is 20 bits and drives ltssm_debug_state[19:0]; bit 20 is
  // driven 0 here so that no bit of the port is left undriven.
  assign ltssm_debug_state[20] = 1'b0;

  // -------------------------------------------------------------------------
  // PG239 assist signals
  // -------------------------------------------------------------------------
  // PG239 (Table 14) asks for as_mac_in_detect high in Detect.Quiet and
  // Detect.Active, and as_cdr_hold_req high in Recovery.Speed, L1.Entry,
  // L1.Idle, Loopback.Speed and Loopback.Entry, mapped onto the states the
  // MAC implements. as_mac_in_detect is a registered decode of the Detect
  // states (low five state bits 00001); ST_IDLE, the reset state, is not
  // one of them, so it is 0 in reset. as_cdr_hold_req is tied 0:
  // pcie_ltssm_downstream never enters ST_L1 or ST_LOOPBACK, and
  // ST_RECOVERY_SPEED is not decoded here.
  always_ff @(posedge pipe_rx_usr_clk_i) begin : assist_mac_in_detect
    if (rst_i || phy_phystatus_rst) as_mac_in_detect <= 1'b0;
    else                            as_mac_in_detect <= (ltssm_debug_state[4:0] == 5'b00001);
  end
  assign as_cdr_hold_req = 1'b0;

  // -------------------------------------------------------------------------
  // Data Link Layer
  // -------------------------------------------------------------------------
  // On clk_i, reset by rst_i alone. idle_valid reaches it from
  // pipe_rx_usr_clk_i without a synchroniser; inside, it feeds only
  // pcie_flow_ctrl_init's idle_count_r, which nothing reads.
  pcie_datalink_layer #(
      .DATA_WIDTH      (DATA_WIDTH),
      .STRB_WIDTH      (STRB_WIDTH),
      .KEEP_WIDTH      (KEEP_WIDTH),
      .USER_WIDTH      (USER_WIDTH),
      .RX_FIFO_SIZE    (RX_FIFO_SIZE),
      .RETRY_TLP_SIZE  (RETRY_TLP_SIZE),
      .MAX_PAYLOAD_SIZE(MAX_PAYLOAD_SIZE),
      .CLK_PERIOD_NS   (CLK_PERIOD_NS)
  ) pcie_datalink_layer_inst (
      .clk_i                  (clk_i),
      .rst_i                  (rst_i),
      // TLP stream, to and from this module's s_tlp_axis_* and m_tlp_axis_*.
      .s_tlp_axis_tdata       (s_tlp_axis_tdata),
      .s_tlp_axis_tkeep       (s_tlp_axis_tkeep),
      .s_tlp_axis_tvalid      (s_tlp_axis_tvalid),
      .s_tlp_axis_tlast       (s_tlp_axis_tlast),
      .s_tlp_axis_tuser       (s_tlp_axis_tuser),
      .s_tlp_axis_tready      (s_tlp_axis_tready),
      .m_tlp_axis_tdata       (m_tlp_axis_tdata),
      .m_tlp_axis_tkeep       (m_tlp_axis_tkeep),
      .m_tlp_axis_tvalid      (m_tlp_axis_tvalid),
      .m_tlp_axis_tlast       (m_tlp_axis_tlast),
      .m_tlp_axis_tuser       (m_tlp_axis_tuser),
      .m_tlp_axis_tready      (m_tlp_axis_tready),
      // Received packets, from phy_receive.
      .s_phy_axis_tdata       (m_dllp_axis_tdata),
      .s_phy_axis_tkeep       (m_dllp_axis_tkeep),
      .s_phy_axis_tvalid      (m_dllp_axis_tvalid),
      .s_phy_axis_tlast       (m_dllp_axis_tlast),
      .s_phy_axis_tuser       (m_dllp_axis_tuser),
      .s_phy_axis_tready      (m_dllp_axis_tready),
      // Packets to send, to phy_transmit.
      .m_phy_axis_tdata       (s_dllp_axis_tdata),
      .m_phy_axis_tkeep       (s_dllp_axis_tkeep),
      .m_phy_axis_tvalid      (s_dllp_axis_tvalid),
      .m_phy_axis_tlast       (s_dllp_axis_tlast),
      .m_phy_axis_tuser       (s_dllp_axis_tuser),
      .m_phy_axis_tready      (s_dllp_axis_tready),
      .cfg_bus_number_o       (cfg_bus_number_o),
      .cfg_device_number_o    (cfg_device_number_o),
      .cfg_function_number_o  (cfg_function_number_o),
      .phy_link_up_i          (link_up_100MHz),
      .fc_initialized_o       (fc_initialized_o),
      .fc_update_valid_o      (fc_update_valid_o),
      .fc_ph_o                (fc_ph_o),
      .fc_pd_o                (fc_pd_o),
      .fc_nph_o               (fc_nph_o),
      .fc_npd_o               (fc_npd_o),
      .fc_cplh_o              (fc_cplh_o),
      .fc_cpld_o              (fc_cpld_o),
      .idle_valid_i           (idle_valid),
      .ext_tag_enable_o       (),
      .rcb_128b_o             (),
      .max_read_request_size_o(),
      .max_payload_size_o     (),
      .msix_enable_o          (),
      .msix_mask_o            (),
      .status_error_cor_i     (),
      .status_error_uncor_i   (),
      .rx_cpl_stall_i         (),
      .link_retrain_req_o     (dll_retrain_req),     // see the retrain handshake
      .link_retraining_i      (retraining_sync[1])
  );

endmodule
