// ---------------------------------------------------------------------------
// pcie_ltssm_downstream -- Link Training and Status State Machine for one Port
//
//! @title pcie_ltssm_downstream
//! @author Idris Somoye
//
// Purpose
//   The Physical Layer LTSSM of one PCI Express Port: Detect, Polling,
//   Configuration, L0 and Recovery, with the Link trained at 2.5 GT/s. It
//   drives the PIPE control outputs, chooses the Ordered Set os_generator
//   transmits, and counts the TS1, TS2 and idle data received on each Lane.
//   IS_ROOT_PORT selects the Configuration role: the Root Port originates
//   Link number LINK_NUM and assigns Lane numbers; the Endpoint adopts the
//   Link number it receives and returns the Lane numbers assigned to it.
//
// Interfaces
//   Control       en_i: ST_IDLE starts training only while it is high;
//                 pcie_endpoint_top ties it to 1. recovery_i: in L0, enters
//                 Recovery; both tops connect the Data Link Layer's retrain
//                 request. directed_speed_change_i: likewise, tied to 0 by both
//                 tops. extended_synch_i: routes Recovery.RcvrLock through
//                 ST_RECOVERY_EXT_SYNCH; pcie_phy_top leaves it unconnected and
//                 pcie_endpoint_top ties it to 0.
//   PIPE          phy_*_o: detection, Electrical Idle, power state, polarity,
//                 de-emphasis; phy_txcompliance_o and phy_txmargin_o are 0.
//                 phy_phystatus_i, phy_rxstatus_i, phy_rxelecidle_i: detection
//                 results and receiver Electrical Idle, per Lane.
//                 phy_phystatus_rst_i: clears active_lanes_o.
//   Detection     receiver_detected_i: the Lanes that detected a Receiver.
//   Received      ts1_valid_i, ts2_valid_i, idle_valid_i, ordered_set_i,
//                 polarity_inverted_i: per Lane, from the receive path.
//   Transmit      gen_os_ctrl_o, ordered_set_o, send_ordered_set_o: the
//                 Ordered Set request to os_generator. ordered_set_tranmitted_i
//                 pulses once per Ordered Set sent; many exits wait for it.
//   Status        link_up_o, ltssm_state_o, active_lanes_o, curr_data_rate_o.
//                 error_o (sticky), success_o, goto_detect_o, goto_cfg_o and
//                 tx_enter_elec_idle_o: connected by neither pcie_phy_top nor
//                 pcie_endpoint_top.
//   Unused        is_timeout_i, lanes_ts2_satisfied_i, config_copmlete_ts2_i,
//                 from_l0_i, lane_status_i: not read. error_loopback_o,
//                 error_disable_o, preset_coeff_o, data_rate_o,
//                 changed_speed_recovery_o: not driven.
//
// Clock and reset
//   clk_i only; both tops connect the PIPE receive user clock. rst_i is
//   synchronous and active high, and both tops OR phy_phystatus_rst into it.
//   CLK_PERIOD_NS sets every timeout. SIM_FAST_LINK = 1 divides the 12 ms and
//   1 ms timeouts by 1000 and lowers MinTS1sPolling to 24.
//
// Limitations
//   L0s, L1, L2, Disabled, Loopback and Hot Reset are declared and never
//   entered; Polling.Compliance only raises error_o. No crosslink,
//   upconfiguration, autonomous width change or Lane reversal. Lanes that
//   detected no Receiver are not put in Electrical Idle. Recovery.Speed
//   requests Electrical Idle only on tx_enter_elec_idle_o, which neither top
//   connects. The rate-change and equalization states are incomplete
//   (gen_ts_os builds a training set only at gen1 and gen2) and never reached:
//   the rate Recovery.RcvrCfg computes is always gen1, so curr_data_rate_o
//   stays gen1 and ST_DETECT_WAIT_ONE_MS is never reached either. While en_i
//   stays high, lane_num_echo keeps its Lane numbers until rst_i, so after a
//   return to ST_IDLE the Endpoint sends them in place of Lane PAD in its
//   Polling and Configuration.Linkwidth.Start training sets.
//
// Structure
//   Timeouts; state encoding; declarations and output assignments; Electrical
//   Idle exit detection; Link number and rate selection; registers; state
//   timer; active Lanes; the state machine (ltssm_combo); per-Lane receive
//   counters (gen_cnt_ts1); per-Lane Ordered Set output.
//
// References
//   PCIe Base Spec r2.1, §4.2.2
//   PCIe Base Spec r2.1, §4.2.4.4
//   PCIe Base Spec r2.1, §4.2.5
//   PCIe Base Spec r2.1, §4.2.6
//   PG239, Table 9: Command Signals
//   PG239, Table 10: Status Signals
// ---------------------------------------------------------------------------
module pcie_ltssm_downstream
  import pcie_phy_pkg::*;
#(
    parameter int CLK_PERIOD_NS = 8,                  //! Link clock period, ns; sets every timeout
    parameter int MAX_NUM_LANES = 4,                  //! Maximum number of lanes module can support
    // DATA_WIDTH, KEEP_WIDTH, USER_WIDTH, IS_UPSTREAM, CROSSLINK_EN,
    // UPCONFIG_EN and MAX_SUPPORTED_RATE are not used in this module.
    parameter int DATA_WIDTH    = 32,                 //! Not used
    parameter int KEEP_WIDTH    = DATA_WIDTH / 8,
    parameter int USER_WIDTH    = $bits(phy_user_t),
    parameter int SIM_FAST_LINK = 0,

    // 1: Root Port, originates LINK_NUM and assigns Lane numbers. 0: Endpoint.
    parameter int          IS_ROOT_PORT = 0,
    parameter int          LINK_NUM           = 0,
    parameter int          IS_UPSTREAM        = 0,
    parameter int          CROSSLINK_EN       = 0,    //crosslink not supported
    parameter int          UPCONFIG_EN        = 0,    //upconfig not supported
    parameter rate_speed_e MAX_SUPPORTED_RATE = gen1
) (
    input  logic                         clk_i,                //! Link clock, period CLK_PERIOD_NS
    input  logic                         rst_i,                //! Synchronous, active high
    // ---- control and status ------------------------------------------------
    input  logic                         en_i,
    output logic                         link_up_o,
    input  logic                         is_timeout_i,         // not read
    input  logic                         recovery_i,
    output logic                         error_o,
    output logic                         success_o,
    output logic                         error_loopback_o,     // not driven
    output logic                         error_disable_o,      // not driven
    // ---- received Ordered Sets ---------------------------------------------
    input  logic [    MAX_NUM_LANES-1:0] ts1_valid_i,
    input  logic [    MAX_NUM_LANES-1:0] ts2_valid_i,
    input  logic [    MAX_NUM_LANES-1:0] idle_valid_i,
    input  logic [    MAX_NUM_LANES-1:0] polarity_inverted_i,
    // ---- PIPE --------------------------------------------------------------
    input  logic [(MAX_NUM_LANES*3)-1:0] phy_rxstatus_i,
    input  logic [    MAX_NUM_LANES-1:0] phy_phystatus_i,
    input  logic                         phy_phystatus_rst_i,
    output logic                         phy_txdetectrx_o,

    output logic [MAX_NUM_LANES-1:0] phy_txelecidle_o,
    output logic                     phy_txdeemph_o,
    output logic [              1:0] phy_powerdown_o,
    output logic                     phy_txcompliance_o,
    output logic [MAX_NUM_LANES-1:0] phy_rxpolarity_o,
    output logic [              2:0] phy_txmargin_o,
    input  logic [MAX_NUM_LANES-1:0] lanes_ts2_satisfied_i,    // not read
    input  logic [MAX_NUM_LANES-1:0] config_copmlete_ts2_i,    // not read
    input  logic                     from_l0_i,                // not read

    // Lanes that detected a Receiver. Both tops latch it from phystatus with
    // rxstatus 011b and clear it when a new detection starts, so it holds the
    // result of the last detection.
    input  logic [MAX_NUM_LANES-1:0] receiver_detected_i,

    // Holds all lanes where Receiver is in EI
    input  logic [MAX_NUM_LANES-1:0] phy_rxelecidle_i,

    // ---- state and transmit control ----------------------------------------
    output logic [MAX_NUM_LANES-1:0] tx_enter_elec_idle_o,
    output logic [              19:0] ltssm_state_o,
    // Sticky until rst_i. goto_detect_o is set when the Recovery.RcvrLock
    // timeout exits to Detect; the arm that sets goto_cfg_o is unreachable.
    output logic                     goto_cfg_o,
    output logic                     goto_detect_o,
    input  logic                     ordered_set_tranmitted_i,
    // Registered transmit_ordered_set. While it is high, os_generator does not
    // repeat its current Ordered Set: at the next Ordered Set boundary it
    // returns to its ST_IDLE and reloads ordered_set_o if gen_os_ctrl_o.valid.
    output logic                     send_ordered_set_o,
    output logic [MAX_NUM_LANES-1:0] active_lanes_o,

    output gen_os_struct_t                        gen_os_ctrl_o,
    //training set configuration signals
    input  pcie_tsos_t        [MAX_NUM_LANES-1:0] ordered_set_i,
    output presets_coeff_t    [MAX_NUM_LANES-1:0] preset_coeff_o,    // not driven
    output pcie_ordered_set_t [MAX_NUM_LANES-1:0] ordered_set_o,
    input  logic                                  extended_synch_i,
    // Both tops tie it to 0.
    input  logic                                  directed_speed_change_i,
    input  logic              [MAX_NUM_LANES-1:0] lane_status_i,     // not read
    output rate_speed_e                           curr_data_rate_o,
    output rate_id_t                              data_rate_o,       // not driven
    output logic                                  changed_speed_recovery_o  // not driven
);

  // -------------------------------------------------------------------------
  // Timeouts
  // -------------------------------------------------------------------------
  // Timeout lengths in clk_i cycles, derived from CLK_PERIOD_NS. timer_r
  // saturates at FourtyEightMsTimeOut, the longest. SIM_FAST_LINK shortens
  // only TwelveMsTimeOut and OneMsTimeOut (to 12 us and 1 us) and
  // MinTS1sPolling; the 2 ms, 24 ms and 48 ms timeouts keep their length.
  localparam int ClockPeriodNs = CLK_PERIOD_NS;
  localparam longint TwentyFourMsTimeOut = (24 * (10 ** 6)) / ClockPeriodNs;
  localparam longint FourtyEightMsTimeOut = (48 * (10 ** 6)) / ClockPeriodNs;
  localparam longint TwelveMsTimeOut = SIM_FAST_LINK ? (12 * (10 ** 4)) / (ClockPeriodNs *10): 
  (12 * (10 ** 6)) / ClockPeriodNs;
  localparam longint TwoMsTimeOut = (2 * (10 ** 6)) / ClockPeriodNs;
  localparam longint OneMsTimeOut = SIM_FAST_LINK ? (1 * (10 ** 4)) / (ClockPeriodNs *10): (1 * (10 ** 6)) / ClockPeriodNs;
  localparam int SixUsTimeOut = (6 * (10 ** 3)) / ClockPeriodNs;
  localparam int EigthHundredNanoSecondTimeOut = (800) / ClockPeriodNs;
  localparam int TwentyNanoSeconds = 20* (10 **0)/ ClockPeriodNs;  // not used
  // Polling.Active's primary exit and its 24 ms branch each need at least 1024
  // transmitted TS1s (PCIe Base Spec r2.1, §4.2.6.2.1); SIM_FAST_LINK lowers
  // the count to 24.
  localparam int MinTS1sPolling = SIM_FAST_LINK ? 24 : 1024;

  // -------------------------------------------------------------------------
  // State encoding
  // -------------------------------------------------------------------------
  // Bits [4:0] name the top-level state and the bits above them the substate,
  // so a compare of bits [4:0] covers a state and all its substates. link_up_c
  // decodes Recovery this way, and pcie_phy_top and pcie_endpoint_top decode
  // ltssm_state_o the same way. A trailing hex value is the encoding as
  // ltssm_state_o shows it. The state table is in the state machine's section
  // header.
  typedef enum logic [19:0] {
    ST_IDLE                           = 20'b00000000000000000000,
    ST_DETECT                         = 20'b00000000000000000001,
    ST_POLLING                        = 20'b00000000000000000010, // 02
    ST_CONFIGURATION                  = 20'b00000000000000000011,
    ST_RECOVERY                       = 20'b00000000000000000100, // 04
    ST_L0                             = 20'b00000000000000000101, // 05
    ST_L0s                            = 20'b00000000000000000110,
    ST_L1                             = 20'b00000000000000000111,
    ST_L2                             = 20'b00000000000000001000,
    ST_DISABLED                       = 20'b00000000000000001001,
    ST_LOOPBACK                       = 20'b00000000000000001010,
    ST_HOT_RESET                      = 20'b00000000000000001011,

    ST_DETECT_WAIT_ONE_MS             = 20'b00000000000000100001, // 21
    ST_DETECT_QUIET                   = 20'b00000000000001000001, // 41
    ST_DETECT_ACTIVE                  = 20'b00000000000001100001, // 61
    ST_DETECT_RX                      = 20'b00000000000010000001, // 81

    ST_POLLING_ACTIVE                 = 20'b00000000000000100010, // 22
    ST_POLLING_CONFIGURATION          = 20'b00000000000001000010, // 42
    ST_POLLING_COMPLIANCE             = 20'b00000000000001100010, // 62

    ST_CONFIGURATION_LINKWIDTH_START  = 20'b00000000000000100011, // 23
    ST_CONFIGURATION_LINKWIDTH_ACCEPT = 20'b00000000000001000011,
    ST_CONFIGURATION_LANENUM_ACCEPT   = 20'b00000000000001100011,
    ST_CONFIGURATION_LANENUM_WAIT     = 20'b00000000000010000011,
    ST_CONFIGURATION_COMPLETE         = 20'b00000000000010100011,
    ST_CONFIGURATION_IDLE             = 20'b00000000000011100011, // E3

    ST_RECOVERY_RCVR_LOCK             = 20'b00000000000000100100, // 24
    ST_RECOVERY_RCVR_LOCK_TIMEOUT     = 20'b00000000000001000100, // 44
    ST_RECOVERY_EQUAL                 = 20'b00000000000001100100, // 64
    ST_RECOVERY_SPEED                 = 20'b00000000000010000100, // 84
    ST_RECOVERY_SPEED_WAIT            = 20'b00000000000010100100, // A4
    ST_RECOVERY_SPEED_EIEOS           = 20'b00000000000011000100, // C4
    ST_RECOVERY_RCVR_CFG              = 20'b00000000000011100100, // E4
    ST_RECOVERY_IDLE                  = 20'b00000000000100000100, //104
    ST_RECOVERY_COMPLETE              = 20'b00000000000100100100, //124
    ST_RECOVERY_EXT_SYNCH             = 20'b00000000000101000100, //144
    ST_RECOVERY_SEND_SDS              = 20'b00000000000101100100, //164
    ST_RECOVERY_EQUAL_PHASE_0         = 20'b00000000000110000100, //184
    ST_RECOVERY_EQUAL_PHASE_1         = 20'b00000000000110100100, //1A4
    ST_RECOVERY_EQUAL_PHASE_2         = 20'b00000000000111000100, //1C4
    ST_RECOVERY_EQUAL_PHASE_3         = 20'b00000000000111100100  //1E4
  } ltssm_state_e;

  // Equalization status. Only equal_complete and phase1_successful are ever
  // set; nothing reads the phase flags.
  typedef struct packed {
    logic equal_complete;
    logic link_equal_req;
    logic phase3_successful;
    logic phase2_successful;
    logic phase1_successful;
    logic phase0_successful;
  } equal_t;

  // -------------------------------------------------------------------------
  // Declarations and output assignments
  // -------------------------------------------------------------------------
  // Most state is a _c / _r pair: an always_comb block computes _c and main_seq
  // registers it. Never updated, never read, or both: axis_pkt_cnt, try_cnt
  // (held at 0), equalization_done_8gb, start_equalization_w_preset,
  // lane_status, ordered_set_tx_in_process, preset_coeff, rate_id and
  // lane_num_satisfied. equal_req is read but never driven.
  ltssm_state_e                               curr_state;
  ltssm_state_e                               next_state;
  pcie_ordered_set_t                          ordered_set_c;
  pcie_ordered_set_t                          ordered_set_r;
  logic              [                  63:0] timer_c;
  logic              [                  63:0] timer_r;
  logic                                       error_c;
  logic                                       error_r;
  logic                                       success_c;
  logic                                       success_r;
  logic                                       goto_detect_c;
  logic                                       goto_cfg_c;

  logic              [     MAX_NUM_LANES-1:0] lane_active_c;
  logic              [     MAX_NUM_LANES-1:0] lane_active_r;



  logic              [     MAX_NUM_LANES-1:0] at_least_one_ts1_ts2;
  logic              [     MAX_NUM_LANES-1:0] equal_req;
  logic              [                   7:0] axis_pkt_cnt_c;
  logic              [                   7:0] axis_pkt_cnt_r;
  logic              [                   7:0] try_cnt_c;
  logic              [                   7:0] try_cnt_r;
  rate_id_t                                   curr_data_rate_c;
  rate_id_t                                   curr_data_rate_r;
  rate_id_t                                   last_data_rate_c;
  rate_id_t                                   last_data_rate_r;
  logic                                       successful_speed_negotiation_c;
  logic                                       successful_speed_negotiation_r;
  logic                                       changed_speed_recovery_c;
  logic                                       changed_speed_recovery_r;
  logic                                       equalization_done_8gb_c;
  logic                                       equalization_done_8gb_r;
  logic                                       start_equalization_w_preset_c;
  logic                                       start_equalization_w_preset_r;

  //!link training helper signals
  logic              [     MAX_NUM_LANES-1:0] link_width_satisfied;
  logic              [     MAX_NUM_LANES-1:0] speed_change_bit_set;
  logic              [                   7:0] link_number_selected;
  logic              [(MAX_NUM_LANES *8)-1:0] link_number_selected_per_lane;
  // Per Lane, the Lane number the peer assigned: any non-PAD Lane number
  // received in Configuration.Lanenum.Wait or Lanenum.Accept. Only rst_i, and
  // ST_IDLE while en_i is low, return it to PAD_; with en_i high it keeps its
  // value across training attempts. Only per_lane_ordered_set_o reads it, for
  // the Endpoint, so it changes what is transmitted and no transition.
  logic              [(MAX_NUM_LANES *8)-1:0] lane_num_echo;
  logic              [   MAX_NUM_LANES-1 : 0] lane_link_number_selected;
  logic              [     MAX_NUM_LANES-1:0] link_lanes_formed;
  logic              [     MAX_NUM_LANES-1:0] lane_num_formed;
  logic              [     MAX_NUM_LANES-1:0] lane_num_satisfied;

  logic              [                  15:0] ordered_set_sent_cnt_c;
  (* mark_debug = "true" *) logic              [                  15:0] ordered_set_sent_cnt_r;

  logic              [     MAX_NUM_LANES-1:0] link_lanes_nums_match;
  logic              [     MAX_NUM_LANES-1:0] link_lane_reconfig;

  logic              [     MAX_NUM_LANES-1:0] ts1_lanenum_wait_satisfied;

  // Per Lane, two consecutive TS1s with Link and Lane numbers PAD: the exit to
  // Detect of Configuration.Linkwidth.Accept and Lanenum.Accept (PCIe Base Spec
  // r2.1, §4.2.6.3.2 and §4.2.6.3.3). An inactive Lane reports 1 so it
  // cannot block the AND-reduction; for that reason the consumers also require
  // |lane_active_r, without which the reduction would hold with no Lane active.
  logic              [     MAX_NUM_LANES-1:0] lanes_all_pad;

  logic              [                   7:0] idle_to_rlock_transitioned_c;
  logic              [                   7:0] idle_to_rlock_transitioned_r;

  logic              [     MAX_NUM_LANES-1:0] lane_status_c;
  logic              [     MAX_NUM_LANES-1:0] lane_status_r;

  // holds last "receiver detected" lines for ST_DETECT_RX state
  logic              [     MAX_NUM_LANES-1:0] lanes_detected_c;
  logic              [     MAX_NUM_LANES-1:0] lanes_detected_r;

  // holds last "receiver elecidle" lines
  logic              [     MAX_NUM_LANES-1:0] phy_rxelecidle_r;
  logic              [     MAX_NUM_LANES-1:0] phy_rxelecidle_exit_detected;

  // Per Lane, an exit from Electrical Idle seen since entering Polling.Active.
  // The 24 ms branch of Polling.Active needs one on a Lane that detected a
  // Receiver (PCIe Base Spec r2.1, §4.2.6.2.1); phy_rxelecidle_exit_detected
  // is a one-cycle pulse, so this register holds it. It is cleared in every
  // other state, not only by rst_i, so the exit that ends Detect.Quiet does
  // not count.
  logic              [     MAX_NUM_LANES-1:0] polling_ei_exit_seen_r;
  logic              [     MAX_NUM_LANES-1:0] polling_ei_exit_seen_c;

  // Ordered Sets transmitted since entering Polling.Active, for its primary
  // exit, which counts TS1s from entry (PCIe Base Spec r2.1, §4.2.6.2.1). The
  // 24 ms branch counts only TS1s sent after one TS1 was received, and keeps
  // ordered_set_sent_cnt_r. Without that receive gate a partner that sent
  // nothing would reach the branch's last arm and raise error_o. 16 bits, like
  // ordered_set_sent_cnt_r: MinTS1sPolling = 1024 does not fit in 8.
  logic              [                  15:0] polling_tx_cnt_r;
  logic              [                  15:0] polling_tx_cnt_c;

  // phy_phystatus_i one cycle late. Both tops latch receiver_detected_i from
  // the same phystatus pulse, so the latched result is valid when the Detect
  // states act on phy_phystatus_r.
  logic              [     MAX_NUM_LANES-1:0] phy_phystatus_r;


  logic              [     MAX_NUM_LANES-1:0] phy_rxpolarity_c;
  logic              [     MAX_NUM_LANES-1:0] phy_rxpolarity_r;
  logic              [                15:0] polarity_lockout_timer_c;
  logic              [                15:0] polarity_lockout_timer_r;


  logic                                       link_up_c;
  logic                                       link_up_r;


  (* mark_debug = "true" *) logic              [     MAX_NUM_LANES-1:0] single_idle_received;
  (* mark_debug = "true" *) logic              [     MAX_NUM_LANES-1:0] single_ts1_received;
  (* mark_debug = "true" *) logic              [     MAX_NUM_LANES-1:0] single_ts2_received;
  (* mark_debug = "true" *) logic              [     MAX_NUM_LANES-1:0] link_idle_satisfied;

  //training sequence satisfy signals
  logic              [     MAX_NUM_LANES-1:0] lanes_ts1_satisfied;
  logic              [     MAX_NUM_LANES-1:0] lanes_ts2_satisfied;
  logic              [     MAX_NUM_LANES-1:0] lanes_idle_satisfied;

  logic              [     MAX_NUM_LANES-1:0] ts1_cnt_satisfied;
  logic              [     MAX_NUM_LANES-1:0] ts2_cnt_satisfied;
  logic                                       transmit_ordered_set;
  logic                                       ordered_set_tx_in_process_c;
  logic                                       ordered_set_tx_in_process_r;
  ts2_symbol6_t                               ts2_symbol6;
  rate_id_t                                   rate_id;
  rate_speed_e                                max_rate;
  rate_speed_e       [     MAX_NUM_LANES-1:0] max_rate_per_lane;
  logic              [     MAX_NUM_LANES-1:0] lane_max_rate_asserted;
  rate_speed_e                                max_supported_rate_c;
  rate_speed_e                                max_supported_rate_r;
  logic                                       equalization_requested;

  gen_os_struct_t                             gen_os_ctrl_c;
  gen_os_struct_t                             gen_os_ctrl_r;
  presets_coeff_t    [     MAX_NUM_LANES-1:0] preset_coeff_c;
  presets_coeff_t    [     MAX_NUM_LANES-1:0] preset_coeff_r;
  equal_t                                     equal_status_c;
  equal_t                                     equal_status_r;

  assign active_lanes_o         = lane_active_r;
  assign ltssm_state_o          = curr_state;
  // equal_req is never driven; the working term is equal_complete, which is 0
  // until ST_RECOVERY_EQUAL_PHASE_1 sets it.
  assign equalization_requested = (equal_req != '0 | !(equal_status_r.equal_complete));
  assign phy_rxpolarity_o       = phy_rxpolarity_r;
  assign link_up_o              = link_up_r;
  // error_o is sticky: error_c defaults to error_r and nothing assigns it 0, so
  // it holds from the first training failure until rst_i. success_o is a
  // level: success_c defaults to 0, so it is high throughout ST_L0 and for one
  // cycle on a few successful exits.
  assign error_o                = error_r;
  assign success_o              = success_r;

  // -------------------------------------------------------------------------
  // Electrical Idle exit detection
  // -------------------------------------------------------------------------
  // phy_rxelecidle_exit_detected[i] is a one-cycle pulse when Lane i leaves
  // receiver Electrical Idle: phy_rxelecidle_r is phy_rxelecidle_i one cycle
  // late. Detect.Quiet exits on it, and polling_ei_exit_seen_r accumulates it
  // in Polling.Active.
  always_comb begin : detect_phy_rxelecidle_exit_detected
    for (int i = 0; i < MAX_NUM_LANES; i++) begin
      if (phy_rxelecidle_r[i] && ~phy_rxelecidle_i[i]) begin
        phy_rxelecidle_exit_detected[i] = '1;
      end
      else begin
        phy_rxelecidle_exit_detected[i] = '0;
      end
    end
  end

  // -------------------------------------------------------------------------
  // Link number and rate selection
  // -------------------------------------------------------------------------
  // link_number_selected is the Link number this Port transmits and matches in
  // Configuration. The Root Port holds LINK_NUM from rst_i. The Endpoint
  // latches the number received in Linkwidth.Start on the Lane the per-Lane
  // block selects: lane_link_number_selected is set only on the lowest-numbered
  // Lane whose link_width_satisfied is set. max_rate takes the rate reported
  // through lane_max_rate_asserted, which only Lane 0 raises. The flag_lane and
  // flag_rate tests are always true: only bits below i can be set before
  // iteration i.
  always_ff @(posedge clk_i) begin : gen_link_number
    if (rst_i) begin
      link_number_selected <= IS_ROOT_PORT ? LINK_NUM[7:0] : '0;
      max_rate             <= gen1;
    end else begin
      logic [MAX_NUM_LANES-1:0] flag_lane;
      logic [MAX_NUM_LANES-1:0] flag_rate;
      flag_lane = '0;
      flag_rate = '0;
      for (int i = 0; i < MAX_NUM_LANES; i++) begin
        if (i == 0) begin
          // The Root Port never latches a received Link number: it keeps
          // LINK_NUM.
          if (!IS_ROOT_PORT && lane_link_number_selected[i]) begin
            link_number_selected <= link_number_selected_per_lane[8*i+:8];
          end

          if (lane_max_rate_asserted[i]) begin
            max_rate <= max_rate_per_lane[i];
          end
        end else begin

          if (!IS_ROOT_PORT && lane_link_number_selected[i] && ((flag_lane >> i) == '0)) begin
            link_number_selected <= link_number_selected_per_lane[8*i+:8];
            flag_lane[i] = '1;
          end

          if (lane_max_rate_asserted[i] && (flag_rate >> i) == '0) begin
            max_rate <= max_rate_per_lane[i];
            flag_rate[i] = '1;
          end
        end
      end

    end
  end

  // -------------------------------------------------------------------------
  // Registers
  // -------------------------------------------------------------------------
  // main_seq registers the module-level _c values of the always_comb blocks
  // below, the delayed copies phy_rxelecidle_r and phy_phystatus_r, and the
  // outputs goto_detect_o, goto_cfg_o and send_ordered_set_o. rst_i returns
  // the machine to ST_IDLE at gen1 with no Ordered Set requested. The per-Lane
  // counters have their own registers in gen_cnt_ts1.
  //! main sequential block
  always_ff @(posedge clk_i) begin : main_seq
    if (rst_i) begin
      curr_state                     <= ST_IDLE;
      timer_r                        <= '0;
      error_r                        <= '0;
      success_r                      <= '0;
      lane_status_r                  <= '0;
      ordered_set_sent_cnt_r         <= '0;
      axis_pkt_cnt_r                 <= '0;
      try_cnt_r                      <= '0;
      changed_speed_recovery_r       <= '0;
      goto_detect_o                  <= '0;
      goto_cfg_o                     <= '0;
      link_up_r                      <= '0;
      lane_status_r                  <= '0;
      lanes_detected_r               <= '0;
      ordered_set_tx_in_process_r    <= '0;
      lane_active_r                  <= '0;
      equalization_done_8gb_r        <= '0;
      gen_os_ctrl_r.valid            <= '0;
      start_equalization_w_preset_r  <= '1;
      last_data_rate_r               <= gen1_basic;
      curr_data_rate_r               <= gen1_basic;
      preset_coeff_r                 <= '0;
      equal_status_r                 <= '0;
      send_ordered_set_o             <= '0;
      ordered_set_r                  <= pcie_ordered_set_t'('0);
      successful_speed_negotiation_r <= '0;
      idle_to_rlock_transitioned_r   <= '0;
      max_supported_rate_r           <= gen1;
      phy_rxpolarity_r               <= '0;
      polarity_lockout_timer_r       <= '0;
      gen_os_ctrl_r                  <= '0;
      phy_rxelecidle_r               <= '0;
      phy_phystatus_r                <= '0;
      polling_ei_exit_seen_r         <= '0;
      polling_tx_cnt_r               <= '0;
    end else begin
      curr_state                     <= next_state;
      phy_rxelecidle_r               <= phy_rxelecidle_i;
      timer_r                        <= timer_c;
      error_r                        <= error_c;
      success_r                      <= success_c;
      lane_status_r                  <= lane_status_c;
      ordered_set_sent_cnt_r         <= ordered_set_sent_cnt_c;
      axis_pkt_cnt_r                 <= axis_pkt_cnt_c;
      try_cnt_r                      <= try_cnt_c;
      last_data_rate_r               <= last_data_rate_c;
      changed_speed_recovery_r       <= changed_speed_recovery_c;
      goto_detect_o                  <= goto_detect_c;
      goto_cfg_o                     <= goto_cfg_c;
      link_up_r                      <= link_up_c;
      lane_status_r                  <= lane_status_c;
      lanes_detected_r               <= lanes_detected_c;
      curr_data_rate_r               <= curr_data_rate_c;
      lane_active_r                  <= lane_active_c;
      equalization_done_8gb_r        <= equalization_done_8gb_c;
      ordered_set_tx_in_process_r    <= ordered_set_tx_in_process_c;
      equal_status_r                 <= equal_status_c;
      start_equalization_w_preset_r  <= start_equalization_w_preset_c;
      send_ordered_set_o             <= transmit_ordered_set;
      ordered_set_r                  <= ordered_set_c;
      successful_speed_negotiation_r <= successful_speed_negotiation_c;
      idle_to_rlock_transitioned_r   <= idle_to_rlock_transitioned_c;
      max_supported_rate_r           <= max_supported_rate_c;
      phy_rxpolarity_r               <= phy_rxpolarity_c;
      polarity_lockout_timer_r       <= polarity_lockout_timer_c;
      gen_os_ctrl_r                  <= gen_os_ctrl_c;
      phy_phystatus_r                <= phy_phystatus_i;
      polling_ei_exit_seen_r         <= polling_ei_exit_seen_c;
      polling_tx_cnt_r               <= polling_tx_cnt_c;
    end
  end

  // -------------------------------------------------------------------------
  // State timer
  // -------------------------------------------------------------------------
  // timer_r counts clk_i cycles in the current state and saturates at
  // FourtyEightMsTimeOut, the longest timeout. It restarts on every state
  // change except into ST_RECOVERY_RCVR_LOCK_TIMEOUT, which evaluates the
  // 24 ms exits of Recovery.RcvrLock on the same count. Despite the block
  // name, ordered_set_sent_cnt is updated in ltssm_combo, not here.
  always_comb begin : timer_and_ordered_set_counter
    timer_c = timer_r;
    if (next_state != curr_state && (next_state != ST_RECOVERY_RCVR_LOCK_TIMEOUT)) begin
      timer_c = '0;
    end else begin
      timer_c = (timer_r >= FourtyEightMsTimeOut) ? FourtyEightMsTimeOut : timer_r + 1;
    end
  end

  // -------------------------------------------------------------------------
  // Active Lanes
  // -------------------------------------------------------------------------
  // A Lane becomes active when the PHY reports a Receiver on it: phystatus with
  // rxstatus 011b (PG239, Table 10: Status Signals). It stays active until
  // rst_i or phy_phystatus_rst_i, so a later detection never removes a Lane.
  // lane_active_r masks the per-Lane flags that the state machine
  // AND-reduces, and is active_lanes_o.
  always_comb begin : lane_status
    lane_active_c = lane_active_r;
    if (phy_phystatus_rst_i) begin
      lane_active_c = '0;
    end else begin
      for (int i = 0; i < MAX_NUM_LANES; i++) begin
        if (phy_phystatus_i[i] && phy_rxstatus_i[3*i+:3] == 3'b011) begin
          lane_active_c[i] = '1;
        end
      end
    end
  end

  // -------------------------------------------------------------------------
  // State machine
  // -------------------------------------------------------------------------
  // ltssm_combo computes next_state and the _c values from curr_state, timer_r
  // and the per-Lane flags of gen_cnt_ts1. Most _c values default to their
  // register, gen_os_ctrl_c and ordered_set_c included, so a field a state does
  // not write keeps its value; error_c, goto_detect_c and goto_cfg_c are never
  // cleared. The progress exits of Polling.Active and of
  // Configuration.Linkwidth.Start through Configuration.Complete are tested
  // only in a cycle where ordered_set_tranmitted_i pulses, at an Ordered Set
  // boundary; their timeouts and all-PAD exits are not. A timeout guarded by
  // (next_state == curr_state) yields to an exit taken in the same cycle.
  // ST_IDLE is the reset state and the target of every failure; with en_i high
  // it moves on in one cycle, so it acts as the path to Detect.
  //
  // In the table, names drop the ST_ prefix and -> gives the exits. EI is
  // Electrical Idle, a TS is a TS1 or TS2, a time is the timeout of that
  // length, sent counts transmitted Ordered Sets, and a detected Lane is one
  // that detected a Receiver. Each case arm's comment has the details. The
  // rate never leaves gen1 (see the Recovery.RcvrCfg speed arm), so the
  // DETECT_WAIT_ONE_MS, RECOVERY_SPEED* and RECOVERY_EQUAL* rows are never
  // reached.
  //
  // State                          Action; -> exits
  // IDLE                           Waits for en_i. -> DETECT_QUIET, or DETECT_WAIT_ONE_MS if the
  //                                rate is not gen1.
  // DETECT_WAIT_ONE_MS             EI, P1; sets gen1 at 1 ms. -> DETECT_QUIET.
  // DETECT_QUIET                   EI, P1. -> DETECT_ACTIVE on an EI exit on any Lane, or at 12 ms.
  // DETECT_ACTIVE                  Receiver detection. -> POLLING if every Lane detects, DETECT_RX
  //                                if some do, else DETECT_QUIET; IDLE at 24 ms.
  // DETECT_RX                      At 12 ms, detection again. -> POLLING if the same Lanes detect,
  //                                else DETECT_QUIET. No timeout.
  // POLLING                        One cycle; requests TS1s. -> POLLING_ACTIVE.
  // POLLING_ACTIVE                 TS1s. -> POLLING_CONFIGURATION after MinTS1sPolling sent and
  //                                eight TSs on every detected Lane; at 24 ms, see the arm.
  // POLLING_COMPLIANCE             Not implemented. -> IDLE with error.
  // POLLING_CONFIGURATION          TS2s. -> CONFIGURATION_LINKWIDTH_START when lanes_ts2_satisfied
  //                                is set on any Lane and 16 sent; IDLE with error at 48 ms.
  // CONFIGURATION_LINKWIDTH_START  TS1s. -> CONFIGURATION_LINKWIDTH_ACCEPT on two matching TS1s on
  //                                any Lane; IDLE with error at 24 ms.
  // CONFIGURATION_LINKWIDTH_ACCEPT -> CONFIGURATION_LANENUM_WAIT on two matching TS1s on any Lane;
  //                                IDLE with error on all-PAD TS1s or at 2 ms.
  // CONFIGURATION_LANENUM_WAIT     -> CONFIGURATION_LANENUM_ACCEPT on two TSs with a new Lane
  //                                number on any Lane; IDLE with error at 2 ms.
  // CONFIGURATION_LANENUM_ACCEPT   -> CONFIGURATION_COMPLETE on two matching TSs on any Lane and
  //                                8 sent; IDLE with error on all-PAD TS1s or at 2 ms.
  // CONFIGURATION_COMPLETE         TS2s. -> CONFIGURATION_IDLE after eight matching TS2s on every
  //                                active Lane and 16 sent; IDLE with error at 2 ms.
  // CONFIGURATION_IDLE             LinkUp, idle. -> L0 after eight idle on every active Lane and
  //                                16 sent; at 2 ms, RECOVERY_RCVR_LOCK or IDLE with error.
  // L0                             LinkUp, idle. -> RECOVERY_RCVR_LOCK on a TS on any Lane,
  //                                recovery_i or directed_speed_change_i.
  // RECOVERY                       TS1s after 10 cycles. -> RECOVERY_RCVR_LOCK.
  // RECOVERY_RCVR_LOCK             -> RECOVERY_RCVR_CFG or RECOVERY_EXT_SYNCH on eight TSs per
  //                                active Lane; RECOVERY_EQUAL at gen3; the next row at 24 ms.
  // RECOVERY_RCVR_LOCK_TIMEOUT     One cycle. -> RECOVERY_RCVR_CFG, RECOVERY_SPEED, or IDLE with
  //                                error and goto_detect_o.
  // RECOVERY_EXT_SYNCH             With extended_synch_i; TS1s. -> RECOVERY_RCVR_CFG after
  //                                1024 sent.
  // RECOVERY_RCVR_CFG              TS2s. -> RECOVERY_IDLE on eight TS2s per active Lane, 16 sent;
  //                                RECOVERY_SPEED on a speed change; at 48 ms, IDLE (see the arm).
  // RECOVERY_SPEED                 Requests EI. -> RECOVERY_SPEED_WAIT when every active Lane's
  //                                receiver is in EI and 2 sent; IDLE at 48 ms.
  // RECOVERY_SPEED_WAIT            New rate 800 ns after a success, old rate 6 us after a failure.
  //                                -> RECOVERY_RCVR_LOCK, or RECOVERY_SPEED_EIEOS from gen3 up.
  // RECOVERY_SPEED_EIEOS           EIEOS. -> RECOVERY_RCVR_LOCK after 8 sent.
  // RECOVERY_IDLE                  -> L0 on eight idle per active Lane, 16 sent; on a TS with Lane
  //                                PAD, CONFIGURATION_LINKWIDTH_START; at 2 ms, RECOVERY or IDLE.
  // RECOVERY_EQUAL                 One cycle; TS1s with EC 01b. -> RECOVERY_EQUAL_PHASE_1.
  // RECOVERY_EQUAL_PHASE_1         TS1s with EC 01b, an EIEOS every 32. -> RECOVERY on two such
  //                                TS1s per active Lane; RECOVERY_SPEED at 24 ms.
  // Never entered: DETECT, CONFIGURATION, L0s, L1, L2, DISABLED, LOOPBACK, HOT_RESET,
  //   RECOVERY_COMPLETE, RECOVERY_SEND_SDS, RECOVERY_EQUAL_PHASE_0, _2 and _3.
  always_comb begin : ltssm_combo
    next_state                     = curr_state;
    error_c                        = error_r;
    success_c                      = '0;
    lane_status_c                  = lane_status_r;
    lanes_detected_c               = lanes_detected_r;
    ordered_set_sent_cnt_c         = ordered_set_sent_cnt_r;
    try_cnt_c                      = try_cnt_r;
    last_data_rate_c               = last_data_rate_r;
    goto_detect_c                  = goto_detect_o;
    goto_cfg_c                     = goto_cfg_o;
    tx_enter_elec_idle_o           = '0;
    curr_data_rate_c               = curr_data_rate_r;
    ts2_symbol6                    = '0;
    // LinkUp is 1b in every Recovery substate (PCIe Base Spec r2.1, §4.2.6,
    // Table 4-7); ST_CONFIGURATION_IDLE and ST_L0 also set it.
    link_up_c                      = (curr_state[4:0] == 5'b00100);
    ordered_set_c                  = ordered_set_r;
    changed_speed_recovery_c       = changed_speed_recovery_r;
    successful_speed_negotiation_c = successful_speed_negotiation_r;
    idle_to_rlock_transitioned_c   = idle_to_rlock_transitioned_r;
    equalization_done_8gb_c        = equalization_done_8gb_r;
    start_equalization_w_preset_c  = start_equalization_w_preset_r;
    transmit_ordered_set           = '0;
    rate_id                        = last_data_rate_r;
    max_supported_rate_c           = max_supported_rate_r;
    gen_os_ctrl_c                  = gen_os_ctrl_r;
    equal_status_c                 = equal_status_r;
    phy_txdetectrx_o               = '0;
    phy_txelecidle_o               = '0;
    phy_powerdown_o                = '0;
    phy_txdeemph_o                 = '1;
    phy_txcompliance_o             = '0;
    phy_rxpolarity_c               = phy_rxpolarity_r;
    // Accumulates in Polling.Active and clears in every other state, so only
    // exits since entering Polling.Active count.
    polling_ei_exit_seen_c         = (curr_state == ST_POLLING_ACTIVE)
                                   ? (polling_ei_exit_seen_r | phy_rxelecidle_exit_detected)
                                   : '0;
    // Every Ordered Set transmitted since entering Polling.Active, saturating.
    polling_tx_cnt_c               = (curr_state == ST_POLLING_ACTIVE)
                                   ? ((ordered_set_tranmitted_i && (polling_tx_cnt_r < 16'hFFFF))
                                      ? polling_tx_cnt_r + 16'd1 : polling_tx_cnt_r)
                                   : '0;
    polarity_lockout_timer_c       = (polarity_lockout_timer_r > 0) ? polarity_lockout_timer_r - 1 : 0;
    phy_txmargin_o                 = '0;
    case (curr_state)
      // Detect.Quiet needs the transmitter in Electrical Idle at 2.5 GT/s. A
      // Port at another rate first spends OneMsTimeOut in ST_DETECT_WAIT_ONE_MS
      // and changes the rate there (PCIe Base Spec r2.1, §4.2.6.1.1). Detect.Quiet
      // also resets idle_to_rlock_transitioned, done here on the way in.
      ST_IDLE: begin
        if (en_i) begin
          idle_to_rlock_transitioned_c = '0;
          gen_os_ctrl_c                = '0;
          phy_txelecidle_o             = '1;
          phy_powerdown_o              = 2'b10;
          if (curr_data_rate_r.rate != gen1) begin
            next_state = ST_DETECT_WAIT_ONE_MS;
          end else begin
            next_state = ST_DETECT_QUIET;
          end
        end
      end
      // Entered only when the rate is not gen1, which never happens: see the
      // Recovery.RcvrCfg speed arm.
      ST_DETECT_WAIT_ONE_MS: begin
        phy_powerdown_o  = 2'b10;
        phy_txelecidle_o = '1;
        if (timer_r >= OneMsTimeOut) begin
          curr_data_rate_c.rate = gen1;
          next_state = ST_DETECT_QUIET;
        end
      end
      // Detect.Quiet (PCIe Base Spec r2.1, §4.2.6.1.1): the transmitter is in
      // Electrical Idle, and the exit is a 12 ms timeout or an Electrical Idle
      // exit on any Lane, which phy_rxelecidle_exit_detected pulses for one cycle.
      ST_DETECT_QUIET: begin
        phy_txelecidle_o = '1;
        phy_powerdown_o  = 2'b10;
        phy_txdeemph_o   = '0;

        if (((|phy_rxelecidle_exit_detected) || (timer_r >= TwelveMsTimeOut))) begin
          next_state    = ST_DETECT_ACTIVE;
        end
      end
      // Detect.Active (PCIe Base Spec r2.1, §4.2.6.1.2). phy_txdetectrx_o in P1
      // requests a Receiver detection, which phystatus completes (PG239, Table 9:
      // Command Signals).
      ST_DETECT_ACTIVE: begin
        phy_txdetectrx_o = '1;
        phy_powerdown_o  = 2'b10;

        if (|phy_phystatus_r) begin
          if (|receiver_detected_i) begin
            if (&receiver_detected_i) begin
              success_c        = '1;
              lanes_detected_c = receiver_detected_i;
              next_state       = ST_POLLING;
            end else begin
              lanes_detected_c = receiver_detected_i;
              next_state       = ST_DETECT_RX;
            end
          end else begin
            // No Receiver on any Lane: back to Detect.Quiet directly, not
            // through ST_IDLE, which stops while en_i is low.
            next_state = ST_DETECT_QUIET;
          end
        end else if (timer_r >= TwentyFourMsTimeOut) begin
          // A watchdog for a phystatus that never arrives; the spec defines no
          // timeout here, and this one goes through ST_IDLE.
          next_state =  ST_IDLE;
        end
      end
      // Some but not all Lanes detected a Receiver: detect again after 12 ms
      // (PCIe Base Spec r2.1, §4.2.6.1.2). The Lanes without a Receiver are not
      // put in Electrical Idle.
      ST_DETECT_RX: begin
        if (timer_r >= TwelveMsTimeOut) begin
          phy_txdetectrx_o = '1;
          phy_powerdown_o  = 2'b10;
          if (|phy_phystatus_r) begin
            if ((lanes_detected_r == receiver_detected_i)) begin
              success_c        = '1;
              lanes_detected_c = receiver_detected_i;
              next_state       = ST_POLLING;
            end else begin
              // A different set of Lanes is a retry, not a training failure:
              // no error_c, and Detect.Quiet rather than ST_IDLE.
              next_state = ST_DETECT_QUIET;
            end
          end
        end else if (timer_r >= TwentyFourMsTimeOut) begin
          // Unreachable: this arm runs only while timer_r < TwelveMsTimeOut.
          next_state = ST_IDLE;
        end
      end
      // One cycle: start TS1s with Link and Lane PAD at gen1 for Polling.Active.
      ST_POLLING: begin
        next_state             = ST_POLLING_ACTIVE;
        ordered_set_sent_cnt_c = '0;
        gen_os_ctrl_c          = '0;
        gen_os_ctrl_c.gen_idle = '0;
        gen_os_ctrl_c.valid    = '1;
        gen_os_ctrl_c.gen_ts1  = '1;
        transmit_ordered_set   = '1;
        ordered_set_c = gen_ts_os( gen1, TS1);
      end
      // Polling.Active (PCIe Base Spec r2.1, §4.2.6.2.1). The primary exit and
      // the 24 ms branch use different transmit counters: see polling_tx_cnt_r.
      ST_POLLING_ACTIVE: begin
        if (ordered_set_tranmitted_i) begin
          // The 24 ms branch counts TS1s sent after one TS1 was received.
          if (|single_ts1_received ) begin
          ordered_set_sent_cnt_c = ordered_set_sent_cnt_r + 1;
          end
          // Polarity is configured in Polling (PCIe Base Spec r2.1, §4.2.5.2,
          // §4.2.4.4). Each detection toggles phy_rxpolarity_r, so detections
          // are ignored for the next 1000 cycles (8 us at CLK_PERIOD_NS = 8).
          if (|polarity_inverted_i && (polarity_lockout_timer_r == 0)) begin
            phy_rxpolarity_c = phy_rxpolarity_r ^ polarity_inverted_i;
            polarity_lockout_timer_c = 16'd1000;
          end

          // Primary exit: MinTS1sPolling sent since entry, and eight qualifying
          // TS1s, or eight TS2s, with Link and Lane PAD on every Lane that
          // detected a Receiver.
          if ((polling_tx_cnt_r >= MinTS1sPolling)) begin
              if (&lanes_ts1_satisfied || &lanes_ts2_satisfied) begin
                ordered_set_sent_cnt_c = '0;
                gen_os_ctrl_c.gen_ts1 = '0;
                gen_os_ctrl_c.gen_ts2 = '1;
                ordered_set_c = gen_ts_os( gen1, TS2);
                transmit_ordered_set = '1;
                next_state = ST_POLLING_CONFIGURATION;
              end
          end
          if ((timer_r >= TwentyFourMsTimeOut) && (ordered_set_sent_cnt_r >= MinTS1sPolling)) begin
            ordered_set_sent_cnt_c = '0;
            // The 24 ms branch reaches Polling.Configuration only if a Lane
            // has the training sequences and a Lane that detected a Receiver
            // has seen an Electrical Idle exit since entering Polling.Active.
            // The spec leaves the number of such Lanes to the implementation;
            // this design requires one. The primary exit has no Electrical
            // Idle condition.
            if ((|lanes_ts1_satisfied || |lanes_ts2_satisfied) &&
                (|(polling_ei_exit_seen_r & receiver_detected_i))) begin
              gen_os_ctrl_c.gen_ts1 = '0;
              gen_os_ctrl_c.gen_ts2 = '1;
              ordered_set_c = gen_ts_os( gen1, TS2);
              transmit_ordered_set = '1;
              next_state = ST_POLLING_CONFIGURATION;
            end else if (|lanes_ts1_satisfied) begin
              // lanes_ts1_satisfied is set on some Lane but no Electrical Idle
              // exit was seen: case (a) of the exit to Polling.Compliance, taken
              // here only with that flag. Case (b), TS1s with Compliance Receive
              // set and Loopback clear, is not detected: such TS1s are not counted.
              next_state = ST_POLLING_COMPLIANCE;
            end else begin
              // No lanes_ts1_satisfied and the Polling.Configuration test above
              // failed: error_c records it, and the timeout below goes to
              // ST_IDLE in this cycle (the spec's exit to Detect).
              error_c = 1'b1;
            end
          end
        end  // end of: if (ordered_set_tranmitted_i)

        // Outside the ordered_set_tranmitted_i test, so it fires even if the
        // transmitter stops completing Ordered Sets. Unless an Ordered Set
        // completes in the cycle where timer_r first reaches
        // TwentyFourMsTimeOut, this takes ST_IDLE then and the 24 ms branch
        // above never runs.
        if ((timer_r >= TwentyFourMsTimeOut) && (next_state == curr_state)) begin
          next_state = ST_IDLE;
        end
      end
      // Not implemented: no compliance pattern is sent (phy_txcompliance_o is
      // always 0). Reported as a training failure.
      ST_POLLING_COMPLIANCE: begin
        error_c    = '1;
        next_state = ST_IDLE;
      end
      // Polling.Configuration (PCIe Base Spec r2.1, §4.2.6.2.3).
      ST_POLLING_CONFIGURATION: begin
        // The Receiver inverts polarity here too. Not gated on
        // ordered_set_tranmitted_i, unlike Polling.Active: it is a receive-side
        // action.
        if (|polarity_inverted_i && (polarity_lockout_timer_r == 0)) begin
          phy_rxpolarity_c = phy_rxpolarity_r ^ polarity_inverted_i;
          polarity_lockout_timer_c = 16'd1000;
        end

        // TS2s sent after one TS2 was received.
        if (ordered_set_tranmitted_i && |single_ts2_received) begin
            ordered_set_sent_cnt_c = ordered_set_sent_cnt_r + 1'b1;
        end

        if (|lanes_ts2_satisfied && ordered_set_sent_cnt_r >= 8'h10) begin
          success_c = '1;
          ordered_set_sent_cnt_c = '0;
          gen_os_ctrl_c.gen_ts1 = '1;
          gen_os_ctrl_c.gen_ts2 = '0;
          transmit_ordered_set = '1;
          // Configuration.Linkwidth.Start: the Root Port sends its Link number
          // with Lane PAD, the Endpoint sends Link and Lane PAD (PCIe Base Spec
          // r2.1, §4.2.6.3.1.1, §4.2.6.3.1.2).
          ordered_set_c = IS_ROOT_PORT
              ? gen_ts_os( gen1, TS1, train_seq_e'(link_number_selected))
              : gen_ts_os( gen1, TS1);
          next_state = ST_CONFIGURATION_LINKWIDTH_START;
        end
        else if (timer_r >= FourtyEightMsTimeOut)
        begin
          error_c    = '1;
          next_state = ST_IDLE;
        end

      end
      // Never entered: no transition targets ST_CONFIGURATION.
      ST_CONFIGURATION: begin
        if (ordered_set_tranmitted_i) begin
          ordered_set_sent_cnt_c = ordered_set_sent_cnt_r + 1'b1;
          if (ordered_set_sent_cnt_r >= 4) begin
            gen_os_ctrl_c.gen_ts1  = '1;
            ordered_set_sent_cnt_c = '0;
            transmit_ordered_set = '1;
            next_state             = ST_CONFIGURATION_LINKWIDTH_START;
          end
        end
      end
      // Configuration.Linkwidth.Start (PCIe Base Spec r2.1, §4.2.6.3.1). No
      // crosslink, so the 16-32 TS1 crosslink rule does not apply, and no
      // Disable or Loopback exit.
      ST_CONFIGURATION_LINKWIDTH_START: begin
        if (ordered_set_tranmitted_i) begin
          ordered_set_sent_cnt_c = ordered_set_sent_cnt_r + 1'b1;
          // Two consecutive TS1s with Lane PAD and an acceptable Link number on
          // any Lane: see the per-Lane ST_CONFIGURATION_LINKWIDTH_START arm.
          if ((|link_width_satisfied)) begin
            ordered_set_sent_cnt_c = '0;
            transmit_ordered_set   = '1;
            ordered_set_c = gen_ts_os( gen1, TS1, train_seq_e'(link_number_selected));
            next_state = ST_CONFIGURATION_LINKWIDTH_ACCEPT;
          end
        end  // end of: if (ordered_set_tranmitted_i)

        if ((timer_r >= TwentyFourMsTimeOut) && (next_state == curr_state)) begin
          error_c    = '1;
          next_state = ST_IDLE;
        end
      end
      // Configuration.Linkwidth.Accept (PCIe Base Spec r2.1, §4.2.6.3.2).
      ST_CONFIGURATION_LINKWIDTH_ACCEPT: begin
        gen_os_ctrl_c.valid = '1;
        if ((ordered_set_tranmitted_i)) begin
          ordered_set_sent_cnt_c = ordered_set_sent_cnt_r + 1'b1;
          // A Link can be formed once any Lane has two consecutive TS1s with
          // link_number_selected. The spec sets no transmit count for this
          // substate. There is no check that the forming Lanes are contiguous,
          // and the Endpoint does not check that the Lane number is non-PAD.
          if ((|link_lanes_formed)) begin
            ordered_set_sent_cnt_c = '0;
            gen_os_ctrl_c.gen_ts1  = '1;
            gen_os_ctrl_c.gen_ts2  = '0;
            transmit_ordered_set   = '1;
            // The Root Port leaves with Lane numbers assigned (PCIe Base Spec
            // r2.1, §4.2.6.3.2.1): 0 here, each Lane's physical index after
            // per_lane_ordered_set_o. Its Lanenum.Wait exit needs a changed
            // Lane number back, which the Endpoint returns from lane_num_echo.
            // The Endpoint's template keeps Lane PAD.
            ordered_set_c = IS_ROOT_PORT
                ? gen_ts_os( gen1, TS1, train_seq_e'(link_number_selected), train_seq_e'(0))
                : gen_ts_os( gen1, TS1, train_seq_e'(link_number_selected));
            next_state = ST_CONFIGURATION_LANENUM_WAIT;
          end
        end  // end of: if (ordered_set_tranmitted_i)

        // Exit to Detect when all active Lanes receive two consecutive TS1s with
        // Link and Lane PAD. |lane_active_r is not a spec term: see lanes_all_pad.
        // The exit to Detect when no Link can be configured is not
        // implemented. The spec does not require error_c on these exits.
        if ((&lanes_all_pad) && (|lane_active_r) && (next_state == curr_state)) begin
          error_c    = '1;
          next_state = ST_IDLE;
        end

        if ((timer_r >= TwoMsTimeOut) && (next_state == curr_state)) begin
          error_c    = '1;
          next_state = ST_IDLE;
        end
      end
      // Configuration.Lanenum.Accept (PCIe Base Spec r2.1, §4.2.6.3.3).
      ST_CONFIGURATION_LANENUM_ACCEPT: begin
        if (ordered_set_tranmitted_i) begin
          ordered_set_sent_cnt_c = ordered_set_sent_cnt_r + 1'b1;
          // Two consecutive TS1s or TS2s whose Link and Lane numbers match on
          // any Lane. The eight-Ordered-Set wait is not a spec condition.
          if (|link_lanes_nums_match && ordered_set_sent_cnt_r >= 8'h8) begin
            transmit_ordered_set  = '1;
            gen_os_ctrl_c.gen_ts1 = '0;
            gen_os_ctrl_c.gen_ts2 = '1;
            ordered_set_c = gen_ts_os( gen1, TS2, train_seq_e'(link_number_selected), train_seq_e'(0));
            ordered_set_sent_cnt_c = '0;
            next_state = ST_CONFIGURATION_COMPLETE;
          end
          // Unreachable: link_lane_reconfig implies link_lanes_nums_match, so
          // the arm above is always taken first.
          else if (|link_lane_reconfig && ordered_set_sent_cnt_r >= 8'h8)
          begin
            next_state = ST_CONFIGURATION_LANENUM_WAIT;
          end
        end  // end of: if (ordered_set_tranmitted_i)

        // Exit to Detect on all-PAD TS1s, as in Linkwidth.Accept. The exit for
        // a Link that cannot be configured is not implemented.
        if ((&lanes_all_pad) && (|lane_active_r) && (next_state == curr_state)) begin
          error_c    = '1;
          next_state = ST_IDLE;
        end

        // The spec states no timeout for this substate. This 2 ms watchdog is an
        // addition that bounds the stay when no exit condition arrives.
        if ((timer_r >= TwoMsTimeOut) && (next_state == curr_state)) begin
          error_c    = '1;
          next_state = ST_IDLE;
        end
      end
      // Configuration.Lanenum.Wait (PCIe Base Spec r2.1, §4.2.6.3.4). Its all-PAD
      // exit to Detect is not implemented here.
      ST_CONFIGURATION_LANENUM_WAIT: begin
        if (ordered_set_tranmitted_i) begin
          ordered_set_sent_cnt_c = ordered_set_sent_cnt_r + 1'b1;
          // Two consecutive TS1s or TS2s on any Lane whose Lane number differs
          // from the one saved in Linkwidth.Accept (lane_in_save).
          if ((|ts1_lanenum_wait_satisfied)) begin
            ordered_set_sent_cnt_c = 0;
            gen_os_ctrl_c.gen_ts1  = '1;
            gen_os_ctrl_c.gen_ts2  = '0;
            transmit_ordered_set   = '1;
            gen_os_ctrl_c.set_lane = '1;
            ordered_set_c = IS_ROOT_PORT
                ? gen_ts_os( gen1, TS1, train_seq_e'(link_number_selected), train_seq_e'(0))
                : gen_ts_os( gen1, TS1, train_seq_e'(link_number_selected));
            next_state = ST_CONFIGURATION_LANENUM_ACCEPT;
          end
        end  // end of: if (ordered_set_tranmitted_i)

        if ((timer_r >= TwoMsTimeOut) && (next_state == curr_state)) begin
          error_c    = '1;
          next_state = ST_IDLE;
        end
      end
      // Configuration.Complete (PCIe Base Spec r2.1, §4.2.6.3.5). N_FTS, Lane
      // de-skew and the Disable Scrambling bit are not handled here.
      ST_CONFIGURATION_COMPLETE: begin
        if (ordered_set_tranmitted_i) begin
          // TS2s sent after one TS2 was received.
          if (|single_ts2_received) begin
          ordered_set_sent_cnt_c = ordered_set_sent_cnt_r + 1'b1;
          end
          // Eight matching TS2s on every active Lane: see lane_num_formed.
          if (&lane_num_formed && (ordered_set_sent_cnt_r >= 8'd16)) begin
            ordered_set_sent_cnt_c = '0;

            transmit_ordered_set   = '1;
            ordered_set_c = gen_zeros();
            gen_os_ctrl_c.gen_ts2  = '0;
            gen_os_ctrl_c.gen_ts1  = '0;
            gen_os_ctrl_c.gen_idle = '1;
            next_state             = ST_CONFIGURATION_IDLE;
          end
        end  // end of: if (ordered_set_tranmitted_i)

        if ((timer_r >= TwoMsTimeOut) && (next_state == curr_state)) begin
          error_c    = '1;
          next_state = ST_IDLE;
        end
      end
      // Configuration.Idle (PCIe Base Spec r2.1, §4.2.6.3.6): idle data,
      // LinkUp = 1b.
      ST_CONFIGURATION_IDLE: begin
        link_up_c = '1;
        // Ordered Set periods sent after one idle was received.
        if (|single_idle_received && ordered_set_tranmitted_i) begin
          ordered_set_sent_cnt_c = ordered_set_sent_cnt_r + 1;
        end
        // Eight idle on every active Lane (see link_idle_satisfied) and 16 sent.
        if ((&link_idle_satisfied) && (ordered_set_sent_cnt_r >= 8'd16)) begin
          success_c                    = '1;
          ordered_set_sent_cnt_c       = '0;
          gen_os_ctrl_c.gen_ts1        = '0;
          gen_os_ctrl_c.gen_ts2        = '0;
          gen_os_ctrl_c.gen_idle       = '0;
          gen_os_ctrl_c.valid          = '0;
          transmit_ordered_set         = '1;
          idle_to_rlock_transitioned_c = '0;
          next_state                   = ST_L0;
        end
        else if (timer_r >= TwoMsTimeOut)
        begin
          if (idle_to_rlock_transitioned_r < 8'hFF) begin
            // The spec allows one diversion to Recovery.RcvrLock: the variable
            // is set on the way, and the next timeout goes to Detect. At gen1
            // and gen2 the register goes straight to FFh, which does that; at
            // other rates it counts up, allowing up to 255 diversions. The
            // compare is on the rate field: rate_id_t holds it in bits [5:1],
            // so the whole struct never equals gen1 or gen2.
            if (curr_data_rate_r.rate == gen1 || curr_data_rate_r.rate == gen2) begin
              idle_to_rlock_transitioned_c = 8'hFF;
            end else begin
              idle_to_rlock_transitioned_c = idle_to_rlock_transitioned_r + 1;
            end
            // Recovery.RcvrLock transmits TS1s (PCIe Base Spec r2.1,
            // §4.2.6.4.1). gen_os_ctrl_c and ordered_set_c hold their values by
            // default, so without these writes Configuration.Idle's idle request
            // would carry into Recovery.RcvrLock.
            gen_os_ctrl_c.gen_ts1  = '1;
            gen_os_ctrl_c.gen_ts2  = '0;
            gen_os_ctrl_c.gen_idle = '0;
            gen_os_ctrl_c.valid    = '1;
            transmit_ordered_set   = '1;
            ordered_set_c = gen_ts_os(curr_data_rate_r.rate, TS1,
                    train_seq_e'(link_number_selected), train_seq_e'(0), last_data_rate_c);
            next_state = ST_RECOVERY_RCVR_LOCK;
          end else begin
            idle_to_rlock_transitioned_c = '1;
            error_c                      = '1;
            next_state                   = ST_IDLE;
          end
        end
      end
      // L0 (PCIe Base Spec r2.1, §4.2.6.5): LinkUp = 1b. idle_to_rlock_transitioned
      // clears on every cycle here, where the spec clears it on a received STP
      // or SDP Symbol.
      ST_L0: begin
        link_up_c = '1;
        success_c = '1;
        idle_to_rlock_transitioned_c = '0;

        // Logical Idle between packets: idle data is the byte 00h, scrambled
        // (PCIe Base Spec r2.1, §4.2.2). gen_zeros() gives 16 zero Symbols, and
        // with gen_idle set os_generator marks none of them as K Symbols.
        //
        // transmit_ordered_set stays low here. While send_ordered_set_o is
        // high, os_generator returns to its ST_IDLE at every Ordered Set
        // boundary, where a valid request resets its SKP interval counter, so
        // no SKP Ordered Set would ever be scheduled. SKP Ordered Sets must
        // continue during idle data. os_generator reads send_ltssm_os_i nowhere
        // else.
        gen_os_ctrl_c.gen_idle       = '1;
        gen_os_ctrl_c.gen_ts1        = '0;
        gen_os_ctrl_c.gen_ts2        = '0;
        gen_os_ctrl_c.valid          = '1;
        ordered_set_c                = gen_zeros();

        // To Recovery on a TS1 or TS2 received on any Lane, or when directed.
        // recovery_i is the Data Link Layer's retrain request, a level
        // synchronised by the top.
        if (|ts1_valid_i || |ts2_valid_i || (directed_speed_change_i && !changed_speed_recovery_r) || recovery_i)
        begin
          gen_os_ctrl_c.gen_ts1 = '1;
          // gen_idle must be cleared on the way out: os_generator clears every
          // K flag when gen_idle is set, after applying the TS mask, so the
          // TS1's COM would go out as scrambled data.
          gen_os_ctrl_c.gen_idle = '0;
          gen_os_ctrl_c.valid = '1;
          transmit_ordered_set = '1;
          ordered_set_c = gen_ts_os( curr_data_rate_r.rate, TS1, train_seq_e'(link_number_selected),
                  train_seq_e'(0), last_data_rate_c);
          ordered_set_sent_cnt_c = '0;
          next_state = ST_RECOVERY_RCVR_LOCK;
        end
      end
      // Entered from Recovery.Idle's timeout and from equalization. After 10
      // cycles, requests TS1s for Recovery.RcvrLock.
      ST_RECOVERY: begin

        if (timer_r >= 8'h0A) begin
          // temp_rate_id is written and never read.
          rate_id_t temp_rate_id;
          temp_rate_id = gen3_basic;
          gen_os_ctrl_c.gen_ts1 = '1;
          gen_os_ctrl_c.valid = '1;
          // Sets speed_change while the last rate is above gen1 and no speed
          // negotiation has succeeded. try_cnt_r is never incremented, so its
          // limit of three never applies.
          if ((last_data_rate_r.rate > gen1) && (try_cnt_r < 8'h3) && !successful_speed_negotiation_r)
          begin
            last_data_rate_c.speed_change = '1;
            temp_rate_id.speed_change = '1;
          end
          transmit_ordered_set = '1;
          ordered_set_c = gen_ts_os( curr_data_rate_r.rate, TS1, train_seq_e'(link_number_selected),
                   train_seq_e'(0), last_data_rate_c);
          ordered_set_sent_cnt_c = '0;
          next_state             = ST_RECOVERY_RCVR_LOCK;
        end
      end
      // Recovery.RcvrLock (PCIe Base Spec r2.1, §4.2.6.4.1). The exit counts
      // eight TS1s or TS2s per active Lane without checking that they are
      // consecutive or that their Link, Lane and speed_change fields match.
      ST_RECOVERY_RCVR_LOCK: begin
        ts2_symbol6 = '0;
        // Equalization at gen3, unless an assignment below overrides it.
        if (equalization_requested && curr_data_rate_r.rate == gen3) begin
          next_state = ST_RECOVERY_EQUAL;
        end
        // A received speed_change bit is echoed in the transmitted TS1s. Each
        // Lane's bit comes from its latest TS1 or TS2, not from eight in a row.
        if (|speed_change_bit_set && !changed_speed_recovery_r) begin
          last_data_rate_c.speed_change = '1;
          transmit_ordered_set = '1;
          ordered_set_c = gen_ts_os( curr_data_rate_r.rate, TS1, train_seq_e'(link_number_selected),
                   train_seq_e'(0), last_data_rate_c);
        end
        if (&(ts1_cnt_satisfied | ts2_cnt_satisfied)) begin
          ordered_set_sent_cnt_c = '0;
          if (extended_synch_i) begin
            next_state = ST_RECOVERY_EXT_SYNCH;
          end else begin
            if (max_rate >= gen3) begin
              ts2_symbol6.req_equal = '1;
            end
            gen_os_ctrl_c.gen_ts1 = '0;
            gen_os_ctrl_c.gen_ts2 = '1;
            transmit_ordered_set  = '1;
            ordered_set_c = gen_ts_os( curr_data_rate_r.rate, TS2, train_seq_e'(link_number_selected),
                     train_seq_e'(0), last_data_rate_r, '0, ts2_symbol6);
            next_state = ST_RECOVERY_RCVR_CFG;
          end
        end
        else if (timer_r >= TwentyFourMsTimeOut)
        begin
          next_state = ST_RECOVERY_RCVR_LOCK_TIMEOUT;
        end
      end
      // The 24 ms exits of Recovery.RcvrLock (PCIe Base Spec r2.1, §4.2.6.4.1),
      // decided in one cycle. timer_r and the per-Lane counts carry over from
      // Recovery.RcvrLock.
      ST_RECOVERY_RCVR_LOCK_TIMEOUT: begin
        // Recovery.RcvrCfg: eight TS1s or TS2s on an active Lane, a received
        // speed_change bit, and a rate above gen1 in use or advertised.
        if ((|((ts1_cnt_satisfied | ts2_cnt_satisfied) & lane_active_r) && (|speed_change_bit_set)) && (
            curr_data_rate_r.rate != gen1 ||
            max_rate != gen1 || last_data_rate_r.rate != gen1))
        begin
          ts2_symbol6 = '0;
          // Empty: no equalization request is set here.
          if (max_rate >= gen3) begin
          end
          transmit_ordered_set = '1;
          ordered_set_c = gen_ts_os( rate_speed_e'(last_data_rate_r.rate), TS2, train_seq_e'(link_number_selected),
                   train_seq_e'(0), last_data_rate_r, '0, ts2_symbol6);
          next_state = ST_RECOVERY_RCVR_CFG;
        end else begin
          // Recovery.Speed: from a rate above gen1 before any rate change, or
          // after a rate change in this Recovery.
          if (!changed_speed_recovery_r && curr_data_rate_r.rate != gen1) begin
            transmit_ordered_set = '1;
            ordered_set_c = gen_ts_os( rate_speed_e'(last_data_rate_r.rate), TS2,
                     train_seq_e'(link_number_selected), train_seq_e'(0), last_data_rate_r, '0, ts2_symbol6);
            next_state = ST_RECOVERY_SPEED;
          end else if (changed_speed_recovery_r) begin
            next_state = ST_RECOVERY_SPEED;
          // Unreachable: the arm above already takes changed_speed_recovery_r,
          // so goto_cfg_o never rises. The spec's exit to Configuration, with
          // changed_speed_recovery = 0b, has no arm here.
          end else if (changed_speed_recovery_r && (|at_least_one_ts1_ts2)) begin
            error_c    = '1;
            goto_cfg_c = '1;
            next_state = ST_IDLE;
          end else begin
            // Otherwise Detect, reported on error_o and goto_detect_o.
            error_c       = '1;
            goto_detect_c = '1;
            next_state    = ST_IDLE;
          end
        end
      end
      // Gen3 equalization, which PCIe Base Spec r2.1 does not define. At gen3,
      // gen_ts_os returns all zeros, so the TS1s requested here carry no
      // training set. Never reached, as the rate never becomes gen3.
      ST_RECOVERY_EQUAL: begin
        // Only the ec field is assigned.
        ts1_symbol6_t temp_ts6;
        ordered_set_sent_cnt_c = '0;
        equal_status_c         = '0;
        gen_os_ctrl_c.valid    = '1;
        gen_os_ctrl_c.gen_ts2  = '0;
        gen_os_ctrl_c.gen_ts1  = '1;
        temp_ts6.ec            = 2'b01;
        transmit_ordered_set   = '1;
        ordered_set_c = gen_ts_os( curr_data_rate_r.rate, TS1, train_seq_e'(link_number_selected),
                 train_seq_e'(0), last_data_rate_c,, temp_ts6);
        next_state = ST_RECOVERY_EQUAL_PHASE_1;
      end
      // An EIEOS every 32 Ordered Sets, each followed by TS1s with EC 01b. The
      // rate is set to gen3 at each EIEOS; entry already requires gen3.
      ST_RECOVERY_EQUAL_PHASE_1: begin
        if (ordered_set_tranmitted_i) begin
          ordered_set_sent_cnt_c = ordered_set_sent_cnt_r + 1'b1;
          if (ordered_set_sent_cnt_r == 32'd31) begin
            gen_os_ctrl_c.gen_ts1    = '0;
            gen_os_ctrl_c.gen3_eieos = '1;
            transmit_ordered_set = '1;
            gen_eieos(ordered_set_c, max_supported_rate_r);
            ordered_set_sent_cnt_c = '0;
            curr_data_rate_c.rate  = gen3;
          end
          if (ordered_set_sent_cnt_r == '0) begin
            ts1_symbol6_t temp_ts6;
            temp_ts6                 = '0;
            gen_os_ctrl_c.gen3_eieos = '0;
            gen_os_ctrl_c.gen_ts1    = '1;
            temp_ts6.ec              = 2'b01;
            transmit_ordered_set     = '1;
            ordered_set_c = gen_ts_os( curr_data_rate_r.rate, TS1, train_seq_e'(link_number_selected),
                     train_seq_e'(0), last_data_rate_c,, temp_ts6);
          end
        end
        // Two TS1s with EC 01b on every active Lane and fewer on every inactive
        // one. Phases 2 and 3 are skipped.
        if (&(ts1_lanenum_wait_satisfied ^ ~lane_active_r)) begin
          equal_status_c.equal_complete = '1;
          equal_status_c.phase1_successful = '1;
          next_state = ST_RECOVERY;
        end else if (timer_r >= TwentyFourMsTimeOut) begin
          next_state = ST_RECOVERY_SPEED;
        end
      end
      // Never entered: nothing assigns ST_RECOVERY_EQUAL_PHASE_2.
      ST_RECOVERY_EQUAL_PHASE_2: begin
        if (ts1_cnt_satisfied) begin
          ts1_symbol6_t temp_ts6;
          temp_ts6.ec = 2'b11;
          transmit_ordered_set = '1;
          ordered_set_c = gen_ts_os( curr_data_rate_r.rate, TS1, train_seq_e'(link_number_selected),
                   train_seq_e'(0), last_data_rate_c,, temp_ts6);
          next_state = ST_RECOVERY_EQUAL_PHASE_3;
        end else if (timer_r >= TwentyFourMsTimeOut) begin
          next_state = ST_RECOVERY_SPEED;
        end
      end
      // Never entered: only ST_RECOVERY_EQUAL_PHASE_2 leads here.
      ST_RECOVERY_EQUAL_PHASE_3: begin
        if (ts1_cnt_satisfied) begin
          gen_os_ctrl_c = '0;
          next_state = ST_RECOVERY_RCVR_LOCK;
        end else if (timer_r >= TwentyFourMsTimeOut) begin
          next_state = ST_RECOVERY_SPEED;
        end
      end
      // With the Extended Synch bit set, at least 1024 TS1s precede
      // Recovery.RcvrCfg (PCIe Base Spec r2.1, §4.2.6.4.1). No TS2 is built on
      // the way out: ts2_symbol6 is computed and not used, so Recovery.RcvrCfg
      // starts with the TS1 request still in place.
      ST_RECOVERY_EXT_SYNCH: begin
        gen_os_ctrl_c.valid = '1;
        gen_os_ctrl_c.gen_ts1 = '1;
        gen_os_ctrl_c.set_lane = '1;
        if (ordered_set_tranmitted_i) begin
          ordered_set_sent_cnt_c = ordered_set_sent_cnt_r + 1'b1;
        end
        if (ordered_set_sent_cnt_r >= 12'd1024) begin
          ts2_symbol6            = '0;
          ordered_set_sent_cnt_c = '0;
          if (max_rate == gen3) begin
            ts2_symbol6.req_equal = '1;
          end
          next_state = ST_RECOVERY_RCVR_CFG;
        end
      end
      // Recovery.RcvrCfg (PCIe Base Spec r2.1, §4.2.6.4.3). Not implemented: the
      // exit to Configuration and the exits to Recovery.Speed on Electrical Idle.
      ST_RECOVERY_RCVR_CFG: begin
        // Ordered Sets sent after a TS2 was received on any Lane: here
        // at_least_one_ts1_ts2 follows the per-Lane TS2 count.
        if (ordered_set_tranmitted_i && at_least_one_ts1_ts2) begin
          ordered_set_sent_cnt_c = ordered_set_sent_cnt_r + 1'b1;
        end
        // To Recovery.Idle: eight TS2s on all configured Lanes with no
        // speed_change, and 16 sent. ts2_cnt_satisfied is already 1 on an
        // inactive Lane, so the AND-reduction takes no lane_active_r mask;
        // masking would make every inactive Lane's term 0 and block a
        // reduced-width Link.
        if(((&ts2_cnt_satisfied)
            && (speed_change_bit_set=='0)
            && ordered_set_sent_cnt_r >= 8'd16) && ordered_set_tranmitted_i)
        begin
          successful_speed_negotiation_c = '0;
          gen_os_ctrl_c                  = '0;
          gen_os_ctrl_c.valid            = '1;
          gen_os_ctrl_c.gen_eios         = '0;
          gen_os_ctrl_c.gen_idle         = '1;
          gen_os_ctrl_c.gen_ts1          = '0;
          gen_os_ctrl_c.gen_ts2          = '0;
          ordered_set_sent_cnt_c         = '0;
          next_state                     = ST_RECOVERY_IDLE;
          transmit_ordered_set           = '1;
          ordered_set_c                  = gen_zeros();
        end
        // To Recovery.Speed below gen3: eight TS2s and a speed_change bit
        // received, a rate above gen1 in use or advertised, and 32 sent. The ||
        // reduces each vector to one bit, so with any Lane inactive the TS2
        // term is lane_active_r[0] alone. An EIOS goes out before Electrical
        // Idle (PCIe Base Spec r2.1, §4.2.6.4.2). The new rate,
        // max_supported_rate_c, starts from last_data_rate_r.rate (Lane 0
        // active) or max_supported_rate_r and can only fall to max_rate. Both
        // start at gen1 and only ST_RECOVERY_SPEED_WAIT, reached only after a
        // result above gen1, raises a rate, so the result is always gen1 and the
        // exit is Recovery.Idle, with the EIOS template and gen_eios still set.
        if((|((ts1_cnt_satisfied || ts2_cnt_satisfied) & lane_active_r)) &&
            (|speed_change_bit_set) &&  (curr_data_rate_r.rate < gen3) &&
            (curr_data_rate_r.rate > gen1 || max_rate > gen1) &&
            ordered_set_sent_cnt_r >= 16'd32)
        begin
          ordered_set_sent_cnt_c = '0;
          for (int i = 0; i < MAX_NUM_LANES; i++) begin
            if (lane_active_r[i]) begin
              if (i == '0) begin
                max_supported_rate_c = last_data_rate_r.rate;
              end else begin
                max_supported_rate_c = max_rate > max_supported_rate_c ? max_supported_rate_c :
                  max_rate;
              end
            end
          end
          if (max_supported_rate_c == gen1) begin
            next_state = ST_RECOVERY_IDLE;
            successful_speed_negotiation_c = '0;
          end else begin
            next_state = ST_RECOVERY_SPEED;
            successful_speed_negotiation_c = '1;
          end
          gen_os_ctrl_c          = '0;
          gen_os_ctrl_c.valid    = '1;
          gen_os_ctrl_c.gen_eios = '1;
          ordered_set_sent_cnt_c = '0;
          transmit_ordered_set   = '1;
          gen_eios(ordered_set_c, curr_data_rate_r.rate);
        end
        // At gen3 and above: eight TS1s or TS2s on every active Lane,
        // speed_change_bit_set clear on every active Lane and set on every
        // inactive one, and 128 sent.
        else if(&(ts1_cnt_satisfied | ts2_cnt_satisfied) && curr_data_rate_r.rate >= gen3
                && (&(speed_change_bit_set ^ lane_active_r)) && ordered_set_sent_cnt_r >= 32'd128)
        begin
          ordered_set_sent_cnt_c = '0;
          for (int i = 0; i < MAX_NUM_LANES; i++) begin
            if (lane_active_r[i]) begin
              if (i == '0) begin
                max_supported_rate_c = last_data_rate_r.rate;
              end else begin
                max_supported_rate_c = max_rate > max_supported_rate_c ? max_supported_rate_c :
                  max_rate;
              end
            end
          end
          gen_os_ctrl_c                  = '0;
          gen_os_ctrl_c.valid            = '1;
          gen_os_ctrl_c.gen_eios         = '1;
          successful_speed_negotiation_c = max_supported_rate_c != gen1;
          transmit_ordered_set           = '1;
          gen_eios(ordered_set_c, curr_data_rate_r.rate);
          next_state = ST_RECOVERY_SPEED;
        end
        // The spec's 48 ms exit goes to Detect. At gen3 and above this design
        // tries Recovery.Idle first while idle_to_rlock_transitioned_r < FFh.
        if (timer_r >= FourtyEightMsTimeOut) begin
          if (curr_data_rate_r.rate == gen1 || curr_data_rate_r.rate == gen2) begin
            next_state = ST_IDLE;
          end else if (idle_to_rlock_transitioned_r < 8'hFF && curr_data_rate_r.rate >= gen3) begin
            changed_speed_recovery_c = '0;
            next_state = ST_RECOVERY_IDLE;
          end else begin
            next_state = ST_IDLE;
          end;
        end
      end
      // Recovery.Speed (PCIe Base Spec r2.1, §4.2.6.4.2). Never reached, like
      // the two states after it: every way in needs a rate above gen1 or an
      // earlier visit (see the Recovery.RcvrCfg speed arm). The Electrical Idle
      // request goes out only on tx_enter_elec_idle_o; phy_txelecidle_o stays 0.
      // gen_os_ctrl_c.valid drops when every active Lane's receiver is in
      // Electrical Idle and ordered_set_sent_cnt_r is at least 2, a count that
      // only entry from Recovery.RcvrCfg clears. After that entry gen_ts1 is set
      // over Recovery.RcvrCfg's EIOS template: see per_lane_ordered_set_o.
      ST_RECOVERY_SPEED: begin
        tx_enter_elec_idle_o = '1;
        gen_os_ctrl_c.gen_ts1 = '1;
        gen_os_ctrl_c.set_lane = '1;
        gen_os_ctrl_c.valid = '1;
        if (ordered_set_tranmitted_i) begin
          ordered_set_sent_cnt_c = ordered_set_sent_cnt_r + 1'b1;
        end
        if (&(phy_rxelecidle_i | ~lane_active_r) && ordered_set_sent_cnt_r >= 2) begin
          gen_os_ctrl_c.valid = '0;
          next_state = ST_RECOVERY_SPEED_WAIT;
        end
        // The spec's 48 ms exit to Detect.
        if (timer_r >= FourtyEightMsTimeOut) begin
          next_state = ST_IDLE;
        end
      end
      // The Electrical Idle time of Recovery.Speed: at least 800 ns after a
      // successful speed negotiation, 6 us after a failed one (PCIe Base Spec
      // r2.1, §4.2.6.4.2). A success sets the new rate and changed_speed_recovery;
      // a failure keeps the current rate, where the spec returns to the rate
      // Recovery was entered at, or 2.5 GT/s.
      ST_RECOVERY_SPEED_WAIT: begin
        if (successful_speed_negotiation_r) begin
          last_data_rate_c = '0;
          if (timer_r >= EigthHundredNanoSecondTimeOut) begin
            curr_data_rate_c.rate    = max_supported_rate_r;
            last_data_rate_c.rate    = max_supported_rate_r;
            changed_speed_recovery_c = '1;
            if (max_supported_rate_r >= gen3) begin
              gen_os_ctrl_c.valid      = '1;
              gen_os_ctrl_c.gen3_eieos = '1;
              next_state               = ST_RECOVERY_SPEED_EIEOS;
              transmit_ordered_set     = '1;
              gen_eieos(ordered_set_c, max_supported_rate_r);
              ordered_set_sent_cnt_c = '0;
            end else begin
              next_state = ST_RECOVERY_RCVR_LOCK;
              transmit_ordered_set = '1;
              ordered_set_c = gen_ts_os( last_data_rate_c.rate, TS1,
                       train_seq_e'(link_number_selected), train_seq_e'(0), last_data_rate_c);
            end
          end
        end else if (timer_r >= SixUsTimeOut) begin
          changed_speed_recovery_c = '0;
          curr_data_rate_c         = curr_data_rate_r;
          // Assigns the 5-bit rate to the whole 8-bit rate_id_t, which places
          // it in bits [4:0], not in the rate field [5:1]. The gen_ts_os call
          // below then reads .rate as that rate shifted right by one bit.
          last_data_rate_c         = curr_data_rate_r.rate;
          transmit_ordered_set     = '1;
          ordered_set_c = gen_ts_os( last_data_rate_c.rate, TS1, train_seq_e'(link_number_selected),
                   train_seq_e'(0), last_data_rate_c);
          next_state = ST_RECOVERY_RCVR_LOCK;
        end
      end
      // Entered from ST_RECOVERY_SPEED_WAIT at a new rate of gen3 or above,
      // after the Electrical Idle time. Sends the EIEOS template built there for
      // eight Ordered Sets with only gen_os_ctrl_c.valid set, then loads the
      // gen_ts_os TS1 result for Recovery.RcvrLock (all zeros at gen3 and
      // above), still without gen_ts1.
      ST_RECOVERY_SPEED_EIEOS: begin
        gen_os_ctrl_c = '0;
        gen_os_ctrl_c.valid = '1;
        if (ordered_set_tranmitted_i) begin
          ordered_set_sent_cnt_c = ordered_set_sent_cnt_r + 1'b1;
        end
        if (ordered_set_sent_cnt_r >= 8'h8) begin
          next_state = ST_RECOVERY_RCVR_LOCK;
          transmit_ordered_set = '1;
          ordered_set_c = gen_ts_os( last_data_rate_r.rate, TS1, train_seq_e'(link_number_selected),
                   train_seq_e'(0), last_data_rate_r);
        end
      end
      // Recovery.Idle (PCIe Base Spec r2.1, §4.2.6.4.4): idle data. The directed
      // exits (Disabled, Hot Reset, Configuration, Loopback) and the exits on
      // received Disable Link, Hot Reset and Loopback bits are not implemented.
      ST_RECOVERY_IDLE: begin
        gen_os_ctrl_c.valid = '1;
        // Ordered Set periods sent after idle was received on any Lane.
        if (ordered_set_tranmitted_i) begin
          if (single_idle_received) begin
            ordered_set_sent_cnt_c = ordered_set_sent_cnt_r + 1'b1;
          end
        end
        // To L0: eight idle on all configured Lanes, and 16 sent.
        // lanes_idle_satisfied is 1 on an inactive Lane, so a Lane outside a
        // reduced-width Link does not block the AND-reduction.
        if (((&lanes_idle_satisfied) && ordered_set_sent_cnt_r >= 8'd16)) begin
        gen_os_ctrl_c                = '0;
        gen_os_ctrl_c.valid          = '0;
        next_state                   = ST_L0;
        idle_to_rlock_transitioned_c = '0;
        // To Configuration on a TS1 or TS2 with Lane PAD on any Lane. The spec
        // asks for two consecutive TS1s; at_least_one_ts1_ts2 rises on the first.
        end else if (at_least_one_ts1_ts2) begin
          gen_os_ctrl_c.valid        = '1;
          ordered_set_sent_cnt_c     = '0;
          gen_os_ctrl_c.gen_ts1      = '1;
          gen_os_ctrl_c.gen_ts2      = '0;
          transmit_ordered_set       = '1;
          ordered_set_c = gen_ts_os(gen1, TS1);
          next_state             = ST_CONFIGURATION_LINKWIDTH_START;
        end else if (timer_r >= TwoMsTimeOut) begin
          if (idle_to_rlock_transitioned_r != '1) begin
            gen_os_ctrl_c.valid    = '0;
            ordered_set_sent_cnt_c = '0;
            // One diversion to Recovery.RcvrLock, through ST_RECOVERY; the next
            // 2 ms timeout goes to Detect. At gen1 and gen2 the register goes
            // straight to FFh, as in Configuration.Idle. At other rates it is
            // not changed, so the diversion repeats.
            if (curr_data_rate_r.rate == gen1) begin
              idle_to_rlock_transitioned_c = '1;
            end
            if (curr_data_rate_r.rate == gen2) begin
              idle_to_rlock_transitioned_c = '1;
            end
            next_state = ST_RECOVERY;
          end
          else
          begin
            gen_os_ctrl_c.valid    = '0;
            ordered_set_sent_cnt_c = '0;
            next_state             = ST_IDLE;
          end
        end
      end
      // Never entered: nothing assigns ST_RECOVERY_SEND_SDS.
      ST_RECOVERY_SEND_SDS: begin
        gen_os_ctrl_c.valid = '1;
        if (ordered_set_tranmitted_i) begin
          idle_to_rlock_transitioned_c = '0;
          gen_os_ctrl_c.valid          = '0;
          next_state                   = ST_L0;
        end
      end
      default: begin
      end
    endcase
  end


  // -------------------------------------------------------------------------
  // Per-Lane receive counters
  // -------------------------------------------------------------------------
  // One gen_cnt_ts1 instance per Lane counts, in each state, the received TS1s,
  // TS2s and idle data that qualify there, and registers the flags the state
  // machine reads (link_width_satisfied, ts1_cnt_satisfied and the rest). The
  // meaning of ts1_cnt and ts2_cnt depends on the state: see each case arm. A
  // flag is registered from a counter, so it lags that counter by one cycle.
  // The counters clear on every state change except into
  // ST_RECOVERY_RCVR_LOCK_TIMEOUT, which keeps Recovery.RcvrLock's counts.
  // Most counters hold at their limit once they reach it, so a later mismatch
  // does not clear them. Flags that the state machine AND-reduces are 1 on a
  // Lane outside the Link (lane_active_r = 0, or for the Polling flags no
  // Receiver detected), so that Lane does not block the reduction.
  for (genvar lane = 0; lane < MAX_NUM_LANES; lane++) begin : gen_cnt_ts1
    (* mark_debug = "true" *) logic              [7:0] ts1_cnt;
    (* mark_debug = "true" *) logic              [7:0] ts2_cnt;
    logic              [7:0] idle_cnt;

    // Consecutive TS1s with Link and Lane PAD, for the all-PAD exits. Unlike
    // ts1_cnt, which holds at its limit, pad_cnt clears on any other TS1 or on
    // a TS2, so it counts consecutive Ordered Sets. ts2_cnt is not reused
    // because link_width_satisfied and link_lanes_nums_match read it.
    logic              [1:0] pad_cnt;

    // The Lane number received with the matching Link number in
    // Linkwidth.Accept; Lanenum.Wait waits for a different one.
    logic              [7:0] lane_in_save;
    // Read only in Recovery.RcvrCfg, where it marks a TS2 run in progress. Its
    // write in Linkwidth.Start never reaches that read, because the transition
    // clear comes between.
    logic                    first_ts1;
    // Symbol 6 and the data rate identifier of the TS2 run in progress, for
    // the identical-identifier checks of Recovery.RcvrCfg and
    // Configuration.Complete.
    ts_symbol6_union_t       temp_ts6;
    rate_id_t                temp_rate_id;
    logic                    lane_speed_change_bit;

    // Signals used for combinatorial logic block
    logic [7:0] ts1_cnt_c, ts2_cnt_c, idle_cnt_c;
    logic [1:0] pad_cnt_c;
    logic first_ts1_c;

    logic single_idle_received_c;
    logic single_ts1_received_c;
    logic single_ts2_received_c;

    logic lane_link_number_selected_c;
    logic lane_max_rate_asserted_c;
    logic lane_speed_change_bit_c;

    logic [7:0] link_number_selected_per_lane_c;
    logic [7:0] lane_in_save_c;
    logic [7:0] lane_num_echo_c;
    ts_symbol6_union_t temp_ts6_c;

    rate_id_t temp_rate_id_c;

    rate_speed_e max_rate_per_lane_c;


    // The flags the state machine reads, registered from this Lane's counters.
    always_ff @(posedge clk_i) begin : output_registers
      if (rst_i) begin
        link_width_satisfied[lane]       <= '0;
        link_lanes_formed[lane]          <= '0;
        ts1_lanenum_wait_satisfied[lane] <= '0;
        lanes_all_pad[lane]              <= '0;
        link_lanes_nums_match[lane]      <= '0;
        link_lane_reconfig[lane]         <= '0;
        lane_num_formed[lane]            <= '0;
        link_idle_satisfied[lane]        <= '0;
        ts1_cnt_satisfied[lane]          <= '0;
        ts2_cnt_satisfied[lane]          <= '0;
        at_least_one_ts1_ts2[lane]       <= '0;
        lanes_ts1_satisfied[lane]        <= '0;
        lanes_ts2_satisfied[lane]        <= '0;
        lanes_idle_satisfied[lane]       <= '0;
        speed_change_bit_set[lane]       <= '0;
      end else begin
        // Configuration: two consecutive matching training sets. ts2_cnt is
        // not counted in Linkwidth.Start, so link_width_satisfied follows
        // ts1_cnt there.
        link_width_satisfied[lane]       <= (ts1_cnt >= 8'h2) | (ts2_cnt == 8'h2);
        link_lanes_formed[lane]          <= (ts1_cnt >= 8'h2);
        ts1_lanenum_wait_satisfied[lane] <= (ts1_cnt >= 8'h2);
        lanes_all_pad[lane]              <= lane_active_r[lane] ? (pad_cnt >= 2'd2) : '1;
        link_lanes_nums_match[lane]      <= (ts1_cnt >= 8'h2) | (ts2_cnt >= 8'h2);
        link_lane_reconfig[lane]         <= (ts1_cnt >= 8'h2);
        lane_num_formed[lane]            <= lane_active_r[lane] ? (ts2_cnt == 8'h8) : '1;
        // In ST_CONFIGURATION_IDLE ts1_cnt counts idle data, so this is the
        // Configuration.Idle exit; Recovery.Idle uses idle_cnt and
        // lanes_idle_satisfied instead.
        link_idle_satisfied[lane]        <= lane_active_r[lane] ? (ts1_cnt >= 8'h8) : '1;
        ts1_cnt_satisfied[lane]          <= lane_active_r[lane] ? (ts1_cnt == 8'h8) : '1;
        ts2_cnt_satisfied[lane]          <= lane_active_r[lane] ? (ts2_cnt == 8'h8) : '1;
        // Registered from the next counter values, so unlike the flags above
        // it does not lag the counters.
        at_least_one_ts1_ts2[lane]       <= (ts1_cnt_c != '0) | (ts2_cnt_c != '0);
        // Polling: masked by the Lanes that detected a Receiver, as the spec
        // states the Polling.Active exits. A Lane without one reports 1, so
        // while such a Lane exists the |-reductions of these flags in
        // Polling.Active's 24 ms branch and Polling.Configuration's exit hold.
        lanes_ts1_satisfied[lane]        <= receiver_detected_i[lane] ? (ts1_cnt == 8'h8) : '1;
        lanes_ts2_satisfied[lane]        <= receiver_detected_i[lane] ? (ts2_cnt == 8'h8) : '1;
        lanes_idle_satisfied[lane]       <= lane_active_r[lane] ? (idle_cnt >= 8'h8) : '1;
        speed_change_bit_set[lane]       <= lane_speed_change_bit != '0;
      end

    end

    // The counters and the per-Lane captures.
    always_ff @(posedge clk_i) begin
      if (rst_i) begin
        ts1_cnt                                  <= '0;
        ts2_cnt                                  <= '0;
        idle_cnt                                 <= '0;
        pad_cnt                                  <= '0;
        first_ts1                                 <= '0;
        link_number_selected_per_lane[lane*8+:8] <= '0;
        lane_in_save                             <= PAD_;
        lane_num_echo[lane*8+:8]                  <= PAD_;
        single_idle_received[lane]               <= '0;
        single_ts1_received[lane]                <= '0;
        single_ts2_received[lane]                <= '0;
        temp_ts6                                 <= '0;
        lane_speed_change_bit                    <= '0;
        max_rate_per_lane[lane]                  <= gen1;
        lane_max_rate_asserted[lane]             <= '0;
      end else begin
        ts1_cnt <= ts1_cnt_c;
        ts2_cnt <= ts2_cnt_c;
        idle_cnt <= idle_cnt_c;
        pad_cnt <= pad_cnt_c;
        first_ts1 <= first_ts1_c;

        single_idle_received[lane] <= single_idle_received_c;
        single_ts1_received[lane]  <= single_ts1_received_c;
        single_ts2_received[lane]  <= single_ts2_received_c;

        lane_speed_change_bit <= lane_speed_change_bit_c;

        lane_link_number_selected[lane] <= lane_link_number_selected_c;
        lane_max_rate_asserted[lane]    <= lane_max_rate_asserted_c;

        link_number_selected_per_lane[lane*8+:8] <= link_number_selected_per_lane_c;
        lane_in_save <= lane_in_save_c;
        lane_num_echo[lane*8+:8] <= lane_num_echo_c;
        max_rate_per_lane[lane] <= max_rate_per_lane_c;
        temp_ts6 <= temp_ts6_c;
        temp_rate_id <= temp_rate_id_c;

      end
    end


    // Next counter values. Everything holds by default, except the select
    // strobes lane_link_number_selected_c and lane_max_rate_asserted_c.
    always_comb begin
      ts1_cnt_c  = ts1_cnt;
      ts2_cnt_c  = ts2_cnt;
      idle_cnt_c = idle_cnt;
      pad_cnt_c  = pad_cnt;
      first_ts1_c = first_ts1;

      single_idle_received_c = single_idle_received[lane];
      single_ts1_received_c  = single_ts1_received[lane];
      single_ts2_received_c  = single_ts2_received[lane];

      lane_speed_change_bit_c = lane_speed_change_bit;

      lane_link_number_selected_c = '0;
      lane_max_rate_asserted_c    = '0;

      link_number_selected_per_lane_c = link_number_selected_per_lane[lane*8+:8];
      lane_in_save_c = lane_in_save;
      lane_num_echo_c = lane_num_echo[lane*8+:8];
      max_rate_per_lane_c = max_rate_per_lane[lane];

      temp_ts6_c = temp_ts6;
      temp_rate_id_c = temp_rate_id;

      // On a state change the counters, first_ts1, single_*_received and
      // lane_speed_change_bit clear, so each state counts from its own entry.
      // The captures (lane_in_save, lane_num_echo, temp_ts6, temp_rate_id, the
      // per-Lane Link number and rate) are kept. The case arms below run only
      // in a cycle without a state change, or with a change into
      // ST_RECOVERY_RCVR_LOCK_TIMEOUT.
      if (next_state != curr_state &&
          next_state != ST_RECOVERY_RCVR_LOCK_TIMEOUT) begin

        ts1_cnt_c  = '0;
        ts2_cnt_c  = '0;
        idle_cnt_c = '0;
        pad_cnt_c  = '0;
        first_ts1_c = '0;

        single_idle_received_c = '0;
        single_ts1_received_c  = '0;
        single_ts2_received_c  = '0;

        lane_speed_change_bit_c = '0;

      end else begin

        case (curr_state)

          // Runs only while ST_IDLE waits for en_i. With en_i high ST_IDLE
          // changes state at once and the transition clear runs instead, which
          // keeps lane_num_echo.
          ST_IDLE: begin
            ts1_cnt_c ='0;
            ts2_cnt_c ='0;
            idle_cnt_c ='0;
            first_ts1_c ='0;

            single_idle_received_c ='0;
            single_ts1_received_c  ='0;
            single_ts2_received_c  ='0;

            lane_num_echo_c = PAD_;
          end

          // ts1_cnt and ts2_cnt: consecutive qualifying TS1s and TS2s.
          ST_POLLING_ACTIVE: begin
            if (ts1_valid_i[lane]) begin
              single_ts1_received_c = '1;

              // A TS1 counts only with Link and Lane PAD and either Compliance
              // Receive (Symbol 5 bit 4) at 0b or Loopback (Symbol 5 bit 2) at
              // 1b (PCIe Base Spec r2.1, §4.2.6.2.1). training_ctrl_t has no
              // Compliance Receive field; the bit is rsvd[4]. A TS2 needs only
              // Link and Lane PAD.
              if ((ordered_set_i[lane].link_num == PAD) &&
                  (ordered_set_i[lane].lane_num == PAD) &&
                  ((ordered_set_i[lane].train_ctrl.rsvd[4] == 1'b0) ||
                    ordered_set_i[lane].train_ctrl.loopback)) begin
                ts1_cnt_c = (ts1_cnt >= 8'h8) ? 8'h8 : ts1_cnt + 1;
              end else begin
                ts1_cnt_c = ts1_cnt >= 8'h8 ? 8'h8 : '0;
              end
            end else if (ts2_valid_i[lane]) begin
              single_ts2_received_c = '1;

              if ((ordered_set_i[lane].link_num == PAD) && (ordered_set_i[lane].lane_num == PAD)) begin 
                ts2_cnt_c = (ts2_cnt >= 8'h8) ? 8'h8 : ts2_cnt + 1;
              end else begin
                ts2_cnt_c = ts2_cnt >= 8'h8 ? 8'h8 : '0;
              end
            end
          end

          // ts2_cnt: consecutive TS2s with Link and Lane PAD.
          ST_POLLING_CONFIGURATION: begin
            if (ts2_valid_i[lane]) begin
              single_ts2_received_c ='1;

              if ((ordered_set_i[lane].link_num == PAD) && (ordered_set_i[lane].lane_num == PAD))
                ts2_cnt_c = (ts2_cnt >= 8'h8) ? 8'h8 : ts2_cnt + 1;
              else
                ts2_cnt_c = ts2_cnt >= 8'h8 ? 8'h8 : '0;
            end
          end

          // ts1_cnt: every TS1 or TS2, up to eight, with no check of its Link,
          // Lane or speed_change fields. Lane 0 also reports the highest rate
          // received, and each Lane its latest speed_change bit.
          ST_RECOVERY_RCVR_LOCK,
          ST_RECOVERY_RCVR_LOCK_TIMEOUT: begin
            if (ts1_valid_i[lane]) begin
              single_ts1_received_c ='1;
            end else if (ts2_valid_i[lane]) begin
              single_ts2_received_c ='1;
            end

            if (ts1_valid_i[lane] || ts2_valid_i[lane]) begin

              if (lane == '0) begin
                max_rate_per_lane_c =
                  (ordered_set_i[lane].rate_id.rate > max_rate)
                  ? ordered_set_i[lane].rate_id.rate : max_rate;

                lane_max_rate_asserted_c ='1;
              end

              ts1_cnt_c = ts1_cnt >= 8'h8 ? 8'h8 : ts1_cnt + 1;

              lane_speed_change_bit_c =
                ordered_set_i[lane].rate_id.speed_change;
            end
          end

          // ts2_cnt: consecutive TS2s with the same Symbol 6 and data rate
          // identifier as the run's first (at gen3, also with req_equal set).
          // The first TS2 of a run (first_ts1 = 0) is always counted. Link and
          // Lane numbers are not checked.
          ST_RECOVERY_RCVR_CFG: begin
            if (ts2_valid_i[lane]) begin
              single_ts2_received_c ='1;

              if ((temp_ts6 == ordered_set_i[lane].ts_s6) &&
                  ((curr_data_rate_r.rate < gen3) ||
                  ((curr_data_rate_r.rate >= gen3) &&
                    ordered_set_i[lane].ts_s6.ts2.req_equal)) &&
                  (temp_rate_id == ordered_set_i[lane].rate_id) ||
                  !first_ts1) begin
                temp_ts6_c = ordered_set_i[lane].ts_s6;
                ts2_cnt_c = (ts2_cnt >= 8'h8) ? 8'h8 : ts2_cnt + 1;
                first_ts1_c ='1;
                temp_rate_id_c = ordered_set_i[lane].rate_id;

                lane_speed_change_bit_c =
                  ordered_set_i[lane].rate_id.speed_change;

              end else begin
                ts2_cnt_c = (ts2_cnt >= 8'h8) ? 8'h8 : '0;
                first_ts1_c ='0;
                lane_speed_change_bit_c ='0;
              end
            end
          end

          // ts1_cnt: TS1s with EC 01b, up to two.
          ST_RECOVERY_EQUAL_PHASE_1: begin
            if (ts1_valid_i[lane]) begin
              single_ts1_received_c ='1;

              if (ordered_set_i[lane].ts_s6.ts1.ec == 2'b01) begin
                ts1_cnt_c = (ts1_cnt >= 8'h2) ? 8'h2 : ts1_cnt + 1;
                single_idle_received_c ='0;
              end
            end
          end

          // idle_cnt: consecutive idle data. ts2_cnt: TS1s and TS2s with Lane
          // PAD, the exit to Configuration.
          ST_RECOVERY_IDLE: begin
            if (idle_valid_i[lane]) begin
              single_idle_received_c ='1;
              idle_cnt_c = (idle_cnt >= 8'h8) ? 8'h8 : idle_cnt + 1;

            end else if (ts1_valid_i[lane] || ts2_valid_i[lane]) begin
              idle_cnt_c = idle_cnt >= 8'h8 ? 8'h8 : '0;
            end

            if ((ts1_valid_i[lane] || ts2_valid_i[lane]) &&
                (ordered_set_i[lane].lane_num == PAD)) begin
              ts2_cnt_c = (ts2_cnt >= 8'h8) ? 8'h8 : ts2_cnt + 1;
              single_idle_received_c ='0;
            end
          end

          // ts1_cnt: consecutive TS1s with Lane PAD and an acceptable Link
          // number, up to two.
          ST_CONFIGURATION_LINKWIDTH_START: begin
            if (ts1_valid_i[lane]) begin
              single_ts1_received_c ='1;

              if ((ordered_set_i[lane].link_num == PAD) &&
                  (ordered_set_i[lane].lane_num == PAD)) begin
                first_ts1_c ='1;
              end


              // The Root Port needs its own Link number back; the Endpoint
              // accepts any non-PAD Link number (PCIe Base Spec r2.1,
              // §4.2.6.3.1.1, §4.2.6.3.1.2). Any other TS1 clears the count
              // below two, so the two must be consecutive.
              if ((IS_ROOT_PORT ? (ordered_set_i[lane].link_num == link_number_selected)
                                 : (ordered_set_i[lane].link_num != PAD)) &&
                  (ordered_set_i[lane].lane_num == PAD)) begin
                ts1_cnt_c = (ts1_cnt >= 8'h2) ? 8'h2 : ts1_cnt + 1;
              end else begin
                ts1_cnt_c = (ts1_cnt >= 8'h2) ? 8'h2 :'0;
              end
            end

            if (link_width_satisfied[lane]) begin
              // The lowest-numbered satisfied Lane supplies the Link number.
              // (1 << lane) - 1 masks the Lanes below this one and is 0 for
              // Lane 0, so Lane 0 needs no special case and no [lane-1:0]
              // part-select, which would be [-1:0] there.
              if ((link_width_satisfied & ((1 << lane) - 1)) == '0) begin
                link_number_selected_per_lane_c = ordered_set_i[lane].link_num;
                lane_link_number_selected_c ='1;
              end
            end
          end

          // ts1_cnt: consecutive TS1s carrying link_number_selected, up to two.
          // pad_cnt: consecutive TS1s with Link and Lane PAD.
          ST_CONFIGURATION_LINKWIDTH_ACCEPT: begin
            if (ts1_valid_i[lane]) begin
              single_ts1_received_c ='1;

              // The Link number must match the one this Port transmits; the
              // Lane number is saved for the Lanenum.Wait comparison.
              if (ordered_set_i[lane].link_num == link_number_selected) begin
                ts1_cnt_c = (ts1_cnt >= 8'h2) ? 8'h2 : ts1_cnt + 1;
                lane_in_save_c = ordered_set_i[lane].lane_num;
              end else begin
                ts1_cnt_c = (ts1_cnt >= 8'h2) ? 8'h2 : '0;
              end

              if ((ordered_set_i[lane].link_num == PAD) &&
                  (ordered_set_i[lane].lane_num == PAD)) begin
                pad_cnt_c = (pad_cnt >= 2'd2) ? 2'd2 : pad_cnt + 2'd1;
              end else begin
                pad_cnt_c = '0;
              end
            end else if (ts2_valid_i[lane]) begin
              // A TS2 breaks a run of consecutive TS1s.
              pad_cnt_c = '0;
            end
          end

          // ts1_cnt: consecutive TS1s or TS2s with a non-PAD Link number and a
          // Lane number other than lane_in_save, up to two.
          ST_CONFIGURATION_LANENUM_WAIT: begin
            if (ts1_valid_i[lane]) begin
              single_ts1_received_c ='1;
            end else if (ts2_valid_i[lane]) begin
              single_ts2_received_c ='1;
            end

            if (ts1_valid_i[lane] || ts2_valid_i[lane]) begin
              if ((ordered_set_i[lane].link_num != PAD) &&
                  (ordered_set_i[lane].lane_num != lane_in_save)) begin
                ts1_cnt_c = (ts1_cnt >= 8'h2) ? 8'h2 : ts1_cnt + 1;
              end else begin
                ts1_cnt_c = (ts1_cnt >= 8'h2) ? 8'h2 : '0;
              end
              // The Lane number the peer assigned, for the Endpoint to return;
              // only a non-PAD value is an assignment. Capture starts in this
              // substate and continues in Lanenum.Accept.
              if (ordered_set_i[lane].lane_num != PAD) begin
                lane_num_echo_c = ordered_set_i[lane].lane_num;
              end
            end
          end

          // ts1_cnt: consecutive TS1s or TS2s whose Link and Lane numbers match,
          // up to two. pad_cnt as in Linkwidth.Accept.
          ST_CONFIGURATION_LANENUM_ACCEPT: begin
            if (ts1_valid_i[lane])
              single_ts1_received_c ='1;
            else if (ts2_valid_i[lane])
              single_ts2_received_c ='1;

            if (ts1_valid_i[lane] || ts2_valid_i[lane]) begin
              // The Root Port assigned each Lane its physical index
              // (per_lane_ordered_set_o) and needs that index back; the
              // Endpoint accepts any non-PAD Lane number.
              if ((ordered_set_i[lane].link_num == link_number_selected) &&
                  (IS_ROOT_PORT ? (ordered_set_i[lane].lane_num == lane)
                                 : (ordered_set_i[lane].lane_num != PAD))) begin

                ts1_cnt_c = (ts1_cnt >= 8'h2) ? 8'h2 : ts1_cnt + 1;

                if (lane == '0) begin
                  max_rate_per_lane_c =
                    (ordered_set_i[lane].rate_id.rate > max_rate)
                    ? ordered_set_i[lane].rate_id.rate : max_rate;

                  lane_max_rate_asserted_c ='1;
                end

              end else begin
                ts1_cnt_c = (ts1_cnt >= 8'h2) ? 8'h2 : '0;
              end
              if (ordered_set_i[lane].lane_num != PAD) begin
                lane_num_echo_c = ordered_set_i[lane].lane_num;
              end
            end

            // The same pad_cnt as Linkwidth.Accept: the transition clear zeroes
            // it between the two substates.
            if (ts1_valid_i[lane]) begin
              if ((ordered_set_i[lane].link_num == PAD) &&
                  (ordered_set_i[lane].lane_num == PAD)) begin
                pad_cnt_c = (pad_cnt >= 2'd2) ? 2'd2 : pad_cnt + 2'd1;
              end else begin
                pad_cnt_c = '0;
              end
            end else if (ts2_valid_i[lane]) begin
              pad_cnt_c = '0;
            end
          end

          // ts2_cnt: consecutive TS2s with matching Link and Lane numbers and an
          // identical data rate identifier, up to eight.
          ST_CONFIGURATION_COMPLETE: begin
            if (ts2_valid_i[lane]) begin
              single_ts2_received_c ='1;

              // The whole rate_id_t is compared, not only its rate field: the
              // identifiers must be identical including the Link Upconfigure
              // Capability bit, Symbol 4 bit 6 (PCIe Base Spec r2.1,
              // §4.2.6.3.5.1). temp_rate_id holds the identifier of the run's
              // first TS2. ts2_cnt == 0 marks that first TS2: the transition
              // clear and a mismatch both zero the count. first_ts1 is not used
              // here.
              if ((ordered_set_i[lane].link_num == link_number_selected) &&
                  (ordered_set_i[lane].lane_num == lane) &&
                  ((ts2_cnt == '0) ||
                   (temp_rate_id == ordered_set_i[lane].rate_id))) begin
                ts2_cnt_c = (ts2_cnt >= 8'h8) ? 8'h8 : ts2_cnt + 1;
                ts1_cnt_c = '0;
                temp_rate_id_c = ordered_set_i[lane].rate_id;
              end else begin
                ts1_cnt_c = '0;
                ts2_cnt_c = (ts2_cnt >= 8'h8) ? 8'h8 : '0;
              end
            end
          end

          // ts1_cnt: consecutive idle data, up to eight; a TS1 or TS2 clears a
          // count below eight.
          ST_CONFIGURATION_IDLE: begin
            if (idle_valid_i[lane]) begin
              single_idle_received_c ='1;
              ts1_cnt_c = (ts1_cnt >= 8'h8) ? 8'h8 : ts1_cnt + 1;

            end else if (ts1_valid_i[lane] || ts2_valid_i[lane]) begin
              ts1_cnt_c = (ts1_cnt >= 8'h8) ? 8'h8 : '0;
            end
          end

          default: ;

        endcase
      end
    end
  end

  // -------------------------------------------------------------------------
  // Per-Lane Ordered Set output
  // -------------------------------------------------------------------------
  // The state machine builds one Ordered Set template (ordered_set_r) for all
  // Lanes. This block copies it to every Lane of ordered_set_o and replaces
  // only the Lane number (Symbol 2), the one field that differs between Lanes.
  // It does so only while gen_ts1 or gen_ts2 is set, so Symbol 2 of idle data,
  // an EIOS or an EIEOS keeps its pattern; Recovery.Speed, which sets gen_ts1
  // over an EIOS template, is never reached. Every TS template holds Lane
  // number PAD or 0. The Root Port replaces a non-PAD value with each Lane's
  // physical index; its templates carry 0 from Configuration.Linkwidth.Accept's
  // exit. The Endpoint replaces the template's value with lane_num_echo once
  // that is not PAD. With MAX_NUM_LANES = 1 the Root Port's index is 0, the
  // value its templates already hold. There is no Lane reversal and no
  // contiguity check: a non-contiguous group of Lanes gets physical indices,
  // not 0 to n-1.
  always_comb begin : per_lane_ordered_set_o
    pcie_tsos_t tmpl;
    logic       tx_ts;
    tmpl  = pcie_tsos_t'(ordered_set_r);
    tx_ts = (gen_os_ctrl_r.gen_ts1 || gen_os_ctrl_r.gen_ts2);
    for (int l = 0; l < MAX_NUM_LANES; l++) begin
      pcie_tsos_t t;
      t = tmpl;
      if (IS_ROOT_PORT) begin
        // The Root Port assigns: Lane numbers 0 to n-1 on a contiguous Link
        // (PCIe Base Spec r2.1, §4.2.6.3.2.1).
        if (tx_ts && (tmpl.lane_num != train_seq_e'(PAD_))) begin
          t.lane_num = l[7:0];
        end
      end else begin
        // The Endpoint returns the Lane number it was assigned, and the
        // template's value until it has one. On a Link without Lane reversal
        // this equals the Lane's physical index.
        if (tx_ts && (lane_num_echo[l*8+:8] != train_seq_e'(PAD_))) begin
          t.lane_num = lane_num_echo[l*8+:8];
        end
      end
      ordered_set_o[l] = pcie_ordered_set_t'(t);
    end
  end

  assign curr_data_rate_o = curr_data_rate_r.rate;
  assign gen_os_ctrl_o    = gen_os_ctrl_r;

endmodule
