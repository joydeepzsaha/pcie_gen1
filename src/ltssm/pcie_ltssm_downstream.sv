//! @title pcie_ltssm_downstream
//! @author Idris Somoye
//! Module implements the pcie physical layer link training state machine.
//! master axis bus.
//!
//! Module does not support upconfig!
//!
//! Module does not support crosslink!
//!
//! Module does not support autonomous lane-width reconfiguration: a link
//! width change requires a full retrain from Detect (there is no live-link
//! path that renegotiates width without first dropping back through
//! Detect/Configuration).
//!
//! Module does not support lane reversal.
module pcie_ltssm_downstream
  import pcie_phy_pkg::*;
#(
    parameter int CLK_PERIOD_NS = 8,                  //! 63 #7g-2 D-7G.2: link-clock period in ns, the ONE source for every timeout below (:108)
    parameter int MAX_NUM_LANES = 4,                  //! Maximum number of lanes module can support
    // TLP data width
    parameter int DATA_WIDTH    = 32,                 //! AXIS data width
    // TLP keep width
    parameter int KEEP_WIDTH    = DATA_WIDTH / 8,
    parameter int USER_WIDTH    = $bits(phy_user_t),
    parameter int SIM_FAST_LINK = 0,

    parameter int          IS_ROOT_PORT = 0,
    parameter int          LINK_NUM           = 0,
    parameter int          IS_UPSTREAM        = 0,    //downstream by default
    parameter int          CROSSLINK_EN       = 0,    //crosslink not supported
    parameter int          UPCONFIG_EN        = 0,    //upconfig not supported
    parameter rate_speed_e MAX_SUPPORTED_RATE = gen1
) (
    input  logic                         clk_i,                //! 100MHz clock signal
    input  logic                         rst_i,                //! Reset signal
    // !Control
    input  logic                         en_i,
    output logic                         link_up_o,
    input  logic                         is_timeout_i,
    input  logic                         recovery_i,
    output logic                         error_o,
    output logic                         success_o,
    output logic                         error_loopback_o,
    output logic                         error_disable_o,
    input  logic [    MAX_NUM_LANES-1:0] ts1_valid_i,
    input  logic [    MAX_NUM_LANES-1:0] ts2_valid_i,
    input  logic [    MAX_NUM_LANES-1:0] idle_valid_i,
    input  logic [    MAX_NUM_LANES-1:0] polarity_inverted_i,
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
    // input  logic [      MAX_NUM_LANES-1:0] lane_active_i,
    input  logic [MAX_NUM_LANES-1:0] lanes_ts2_satisfied_i,
    input  logic [MAX_NUM_LANES-1:0] config_copmlete_ts2_i,
    input  logic                     from_l0_i,

    // Holds all lanes where a Receiver has been detected
    input  logic [MAX_NUM_LANES-1:0] receiver_detected_i,

    // Holds all lanes where Receiver is in EI
    // These should all act together for use???
    input  logic [MAX_NUM_LANES-1:0] phy_rxelecidle_i,

    output logic [MAX_NUM_LANES-1:0] tx_enter_elec_idle_o,
    output logic [              19:0] ltssm_state_o,
    output logic                     goto_cfg_o,
    output logic                     goto_detect_o,
    input  logic                     ordered_set_tranmitted_i,
    output logic                     send_ordered_set_o,
    output logic [MAX_NUM_LANES-1:0] active_lanes_o,

    output gen_os_struct_t                        gen_os_ctrl_o,
    //training set configuration signals
    input  pcie_tsos_t        [MAX_NUM_LANES-1:0] ordered_set_i,
    output presets_coeff_t    [MAX_NUM_LANES-1:0] preset_coeff_o,
    output pcie_ordered_set_t [MAX_NUM_LANES-1:0] ordered_set_o,
    // input  ts_symbol6_union_t [MAX_NUM_LANES-1:0] symbol6_i,
    // input  training_ctrl_t    [MAX_NUM_LANES-1:0] training_ctrl_i,
    // input  rate_id_t          [MAX_NUM_LANES-1:0] rate_id_i,
    input  logic                                  extended_synch_i,
    // output logic                                  gen_os_o,
    //TODO: this needs to be computed from ts1's/ ts2's with
    //speed change bit or sw active
    input  logic                                  directed_speed_change_i,
    input  logic              [MAX_NUM_LANES-1:0] lane_status_i,
    output rate_speed_e                           curr_data_rate_o,
    output rate_id_t                              data_rate_o,
    output logic                                  changed_speed_recovery_o
    // //! @virtualbus master_axis_bus @dir out
    // output logic              [   DATA_WIDTH-1:0] m_axis_tdata,
    // output logic              [   KEEP_WIDTH-1:0] m_axis_tkeep,
    // output logic                                  m_axis_tvalid,
    // output logic                                  m_axis_tlast,
    // output logic              [   USER_WIDTH-1:0] m_axis_tuser,
    // input  logic                                  m_axis_tready
    //! @end
);

  localparam int ClockPeriodNs = CLK_PERIOD_NS;  // 63 #7g-2: was 1000/CLK_RATE; the CLK_RATE alias left when pcie_endpoint_top.sv switched to .CLK_PERIOD_NS (Joy-approved). Edited IN PLACE: :111 onward carries -lines waivers.
  localparam longint TwentyFourMsTimeOut = (24 * (10 ** 6)) / ClockPeriodNs;
  localparam longint FourtyEightMsTimeOut = (48 * (10 ** 6)) / ClockPeriodNs;
  localparam longint TwelveMsTimeOut = SIM_FAST_LINK ? (12 * (10 ** 4)) / (ClockPeriodNs *10): 
  (12 * (10 ** 6)) / ClockPeriodNs;
  localparam longint TwoMsTimeOut = (2 * (10 ** 6)) / ClockPeriodNs;
  localparam longint OneMsTimeOut = SIM_FAST_LINK ? (1 * (10 ** 4)) / (ClockPeriodNs *10): (1 * (10 ** 6)) / ClockPeriodNs;
  localparam int SixUsTimeOut = (6 * (10 ** 3)) / ClockPeriodNs;
  localparam int EigthHundredNanoSecondTimeOut = (800) / ClockPeriodNs;
  localparam int TwentyNanoSeconds = 20* (10 **0)/ ClockPeriodNs;  //(20 * (10** -9)); //)) / int'((1 / (CLK_RATE * $pow(10, 6))));
  // PCIe requires 1024 transmitted TS1s.  The cocotb link-up test uses the
  // fast-simulation mode so the same state transition can be exercised
  // inside its 25 us timeout.
  localparam int MinTS1sPolling = SIM_FAST_LINK ? 24 : 1024;

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

  typedef struct packed {
    logic equal_complete;
    logic link_equal_req;
    logic phase3_successful;
    logic phase2_successful;
    logic phase1_successful;
    logic phase0_successful;
  } equal_t;

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
  //! internal_axis_signals
  // logic              [   DATA_WIDTH-1:0] ltssm_axis_tdata;
  // logic              [   KEEP_WIDTH-1:0] ltssm_axis_tkeep;
  // logic                                  ltssm_axis_tvalid;
  // logic                                  ltssm_axis_tlast;
  // logic              [   USER_WIDTH-1:0] ltssm_axis_tuser;
  // logic                                  ltssm_axis_tready;

  //!link training helper signals
  logic              [     MAX_NUM_LANES-1:0] link_width_satisfied;
  logic              [     MAX_NUM_LANES-1:0] speed_change_bit_set;
  logic              [                   7:0] link_number_selected;
  logic              [(MAX_NUM_LANES *8)-1:0] link_number_selected_per_lane;
  // EP (IS_ROOT_PORT=0) reactive Lane-Number echo: per-lane capture of the
  // Lane Number the downstream/root peer assigned on each lane, latched in
  // Configuration.Lanenum from ordered_set_i[lane].lane_num. PAD until an
  // assignment is received; then the EP transmits it back (see the per-lane
  // output stage). PURE OUTPUT PATH: written from an input, read only by
  // ordered_set_o -- never by any FSM exit condition -- so it cannot change
  // EP state/timing (EP regression stays byte-identical, same argument as the
  // per-lane output stage itself).
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

  // C9 / C16 (Base 2.1 4.2.6.3.2.1 p.230 and 4.2.6.3.3.1 p.233; tracker SS54 #11).
  // Both substates name the same non-timeout route to Detect: "all Lanes receive
  // two consecutive TS1 Ordered Sets with Link and Lane numbers set to PAD
  // (K23.7)".  Neither was implemented; Linkwidth.Accept had only the 2 ms limb
  // and Lanenum.Accept had neither spec exit.
  //
  // Gated by lane_active_r like link_idle_satisfied / ts1_cnt_satisfied /
  // ts2_cnt_satisfied, so an inactive Lane on a reduced-width link contributes a
  // trivial '1' to the &-reduction instead of blocking it forever.  ⚠️ That gating
  // is exactly why the consumers below ALSO test (|lane_active_r): with no Lane
  // active the &-reduction is trivially true and the exit would fire on entry.
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

  // P6 (Base 2.1 4.2.6.2.1 p.221 limb (ii); tracker SS54 #8).  The 24 ms
  // Polling.Active branch reaches Polling.Configuration only if, IN ADDITION to
  // the training-sequence limb, "at least a predetermined number of Lanes that
  // detected a Receiver during Detect have detected an exit from Electrical Idle
  // at least once SINCE ENTERING POLLING.ACTIVE".
  //
  // phy_rxelecidle_exit_detected is a ONE-CYCLE pulse, and until this fix it was
  // sampled at exactly one site (:569, inside Detect.Quiet) and nowhere in
  // Polling at all -- so there was nothing to test the limb against.  This
  // register is that memory.
  //
  // ⚠️ It is cleared whenever curr_state != ST_POLLING_ACTIVE, not merely on
  // rst_i, and that is load-bearing: EVERY bench toggles phy_rxelecidle_i 1->0
  // during Detect.Quiet to trigger :569's exit.  A reset-only clear would let
  // that Detect-era edge satisfy the limb and the fix would be INERT.  The spec's
  // own words are what settle it -- "since entering Polling.Active".
  logic              [     MAX_NUM_LANES-1:0] polling_ei_exit_seen_r;
  logic              [     MAX_NUM_LANES-1:0] polling_ei_exit_seen_c;

  // P4 (Base 2.1 4.2.6.2.1; tracker SS54 #8).  THE SPEC STATES THE
  // TRANSMIT-COUNT LIMB TWICE, WITH DIFFERENT QUALIFIERS, AND THEY NEED
  // DIFFERENT COUNTERS:
  //
  //   p.220, the PRIMARY exit -- "after at least 1024 TS1 Ordered Sets were
  //   transmitted, and all Lanes ... receive eight consecutive training
  //   sequences ...".  Counted FROM ENTRY TO POLLING.ACTIVE, with no dependence
  //   on what has been received.
  //
  //   p.221, inside the 24 ms branch -- "a minimum of 1024 TS1 Ordered Sets are
  //   transmitted AFTER RECEIVING ONE TS1 Ordered Set."
  //
  // ordered_set_sent_cnt_r applies the TIMEOUT BRANCH's qualifier at its
  // increment (the |single_ts1_received gate below), and BOTH consumers read it.
  // That gate is not a defect -- it is p.221's qualifier, correctly applied to
  // the branch that has it.  The defect was that the PRIMARY exit read the same
  // counter, so it demanded a further MinTS1sPolling transmissions the spec does
  // not ask for.  ordered_set_sent_cnt_r therefore stays exactly as it is and
  // remains the 24 ms branch's counter; this is the primary exit's.
  //
  // ⚠️ Deleting the |single_ts1_received gate -- the obvious one-line "fix" --
  // was REJECTED and the reason is measurable, not stylistic: it makes the 24 ms
  // branch satisfiable on a link whose partner never responded, which then takes
  // that branch's else arm and asserts error_c where today it is never reached.
  // error_r is STICKY and error_o has been a gate-observed port since fix-arc 1,
  // so that would redden one-sided `error_o == 0` rows in three other benches.
  // Mutant MP4b applies that form deliberately to keep this a measurement.
  // See evidence/fix-arc-6/PREDICTIONS_P4.md sections 1c and 2.
  //
  // ⚠️ WIDTH: [15:0], matching ordered_set_sent_cnt_r (:242-:243) and NOT the
  // [7:0] of the per-lane ts1_cnt/ts2_cnt counters.  MinTS1sPolling is 1024 when
  // SIM_FAST_LINK=0 and does not fit in eight bits; a saturating [7:0] counter
  // could never reach it and the primary exit would deadlock on exactly the two
  // realtimer_linkup rows.
  logic              [                  15:0] polling_tx_cnt_r;
  logic              [                  15:0] polling_tx_cnt_c;

  // Need to pipeline phy_phystatus_i
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
  // rate_id = last_data_rate_r;
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
  assign equalization_requested = (equal_req != '0 | !(equal_status_r.equal_complete));
  assign phy_rxpolarity_o       = phy_rxpolarity_r;
  assign link_up_o              = link_up_r;
  // error_o and success_o were declared at :42-:43 and never driven, so the
  // FSM's 12 error_c raise sites reached no port and no integrator could
  // observe a training failure.  Note the two are not symmetric: error_c
  // defaults to error_r (:490) and no site ever assigns it 0, so error_o is
  // STICKY once raised and clears only on rst_i; success_c defaults to 0
  // (:491), so success_o is a level, high throughout ST_L0.
  assign error_o                = error_r;
  assign success_o              = success_r;

 
  always_comb begin : detect_phy_rxelecidle_exit_detected
    for (int i = 0; i < MAX_NUM_LANES; i++) begin
      // If last cycle lane was in elecidle and this cycle it is not,
      // => exit detected
      if (phy_rxelecidle_r[i] && ~phy_rxelecidle_i[i]) begin
        phy_rxelecidle_exit_detected[i] = '1;
      end
      else begin
        phy_rxelecidle_exit_detected[i] = '0;
      end
    end
  end

  always_ff @(posedge clk_i) begin : gen_link_number
    if (rst_i) begin
      // RC (IS_ROOT_PORT=1) originates the Link Number as LINK_NUM; EP
      // (IS_ROOT_PORT=0) starts at '0 and latches from the RX side below,
      // unchanged from before.
      link_number_selected <= IS_ROOT_PORT ? LINK_NUM[7:0] : '0;
      max_rate             <= gen1;
    end else begin
      logic [MAX_NUM_LANES-1:0] flag_lane;
      logic [MAX_NUM_LANES-1:0] flag_rate;
      flag_lane = '0;
      flag_rate = '0;
      for (int i = 0; i < MAX_NUM_LANES; i++) begin
        if (i == 0) begin
          // !IS_ROOT_PORT guard: RC never re-latches link_number_selected
          // from the RX side -- it already holds LINK_NUM from reset above.
          // Fix #6's EP latch (below) is untouched when IS_ROOT_PORT=0.
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
      // for(i = 0; i < MAX_NUM_LANES; i++) begin
      //   preset_coeff_r.rx_preset <=
      //   tx_preset <=
      //   pre_cursor
      //   cursor_coef
      // end
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
    //non-resetable
  end


  always_comb begin : timer_and_ordered_set_counter
    timer_c = timer_r;
    // ordered_set_sent_cnt_c = ordered_set_sent_cnt_r;
    if (next_state != curr_state && (next_state != ST_RECOVERY_RCVR_LOCK_TIMEOUT)) begin
      timer_c = '0;
      // ordered_set_sent_cnt_c = '0;
    end else begin
      // if (ordered_set_tranmitted_i) begin
      //   ordered_set_sent_cnt_c = ordered_set_sent_cnt_r;
      // end
      timer_c = (timer_r >= FourtyEightMsTimeOut) ? FourtyEightMsTimeOut : timer_r + 1;
    end
  end


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



  always_comb begin : ltssm_combo
    next_state                     = curr_state;
    // timer_c                        = timer_r;
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
    link_up_c                      = (curr_state[4:0] == 5'b00100);  // 63 #7k: Table 4-7 p.216, LinkUp = 1b in every Recovery substate; L0 / Config.Idle set it below
    //ordered set
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
    // P6: accumulate while IN Polling.Active, force 0 otherwise -- see the
    // declaration for why the clear condition is the state and not rst_i.
    polling_ei_exit_seen_c         = (curr_state == ST_POLLING_ACTIVE)
                                   ? (polling_ei_exit_seen_r | phy_rxelecidle_exit_detected)
                                   : '0;
    // P4: every transmitted Ordered Set counted FROM ENTRY to Polling.Active,
    // saturating.  Same state-scoped shape as the accumulator above.
    polling_tx_cnt_c               = (curr_state == ST_POLLING_ACTIVE)
                                   ? ((ordered_set_tranmitted_i && (polling_tx_cnt_r < 16'hFFFF))
                                      ? polling_tx_cnt_r + 16'd1 : polling_tx_cnt_r)
                                   : '0;
    polarity_lockout_timer_c       = (polarity_lockout_timer_r > 0) ? polarity_lockout_timer_r - 1 : 0;
    phy_txmargin_o                 = '0;
    // gen_os_ctrl_c                  = '0;
    case (curr_state)
      //*********************************************************
      // Idle
      //*********************************************************
      // In ST_DETECT_QUIET we need to transmitter to be in  Electrical Idle.
      // Furthermore, the data rate needs to be set to gen1. If that is not the case already, transmit
      // the old rate for one ms (happening in ST_DETECT_WAIT_ONE_MS) and then set the current_data_rate to gen1. 
      // Then proceed to ST_DETECT_QUIET.
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
      //*********************************************************
      // Detect.Wait.One.Ms
      //*********************************************************
      // Only necessary if data rate is greater than Gen1. This should also set datarate to gen1.
      ST_DETECT_WAIT_ONE_MS: begin
        phy_powerdown_o  = 2'b10;
        phy_txelecidle_o = '1;
        if (timer_r >= OneMsTimeOut) begin
          curr_data_rate_c.rate = gen1;
          next_state = ST_DETECT_QUIET;
        end
      end
      //*********************************************************
      // Detect.Quiet
      //*********************************************************
      // In this state we need to transmit EIs.
      // We leave this state either if 12ms are over, or, if we detect that any receiving lane exits electrical idle.
      // phy_rxelecidle_exit_detected will be 1 for exactly one cycle if any lane exited electrical idle between cycles.
      // Requires an "exit electrical idle" detection
      ST_DETECT_QUIET: begin
        phy_txelecidle_o = '1;
        phy_powerdown_o  = 2'b10;
        phy_txdeemph_o   = '0;

        if (((|phy_rxelecidle_exit_detected) || (timer_r >= TwelveMsTimeOut))) begin
          next_state    = ST_DETECT_ACTIVE;
        end
      end
      //*********************************************************
      // Detect.Active
      //*********************************************************
      // Requires reciever detection to transition to ST_POLLING
      // Receiver detection is triggered in the PIPE. For this, phy_txdetectrx_o has to be set/kept at 1.
      // We then listen on the phy_phystatus_r signal to wait for the receiver detection to finish. 
      // Oddly engough this takes aroun 130 cycles. 
      // The result is stored in receiver_detected_i. If on all lanes a receiver was detected we can transition to ST_POLLING.
      // If only some lanes detect a receiver we go to ST_DETECT_RX. If no receiver were detected we go back to ST_IDLE=>ST_DETECT_QUIET.
      ST_DETECT_ACTIVE: begin
        //bounded timeout counter
        phy_txdetectrx_o = '1;
        phy_powerdown_o  = 2'b10;

        // Wait for receiver detection to finish
        if (|phy_phystatus_r) begin
          if (|receiver_detected_i) begin
            if (&receiver_detected_i) begin
              success_c        = '1;
              // timer_c          = '0;
              lanes_detected_c = receiver_detected_i;
              next_state       = ST_POLLING;
            end else begin
              lanes_detected_c = receiver_detected_i;
              next_state       = ST_DETECT_RX;
            end 
          end else begin
            // Base 2.1 4.2.6.1.2 p.219: "Next state is Detect.Quiet if a
            // Receiver is not detected on any Lanes."  This arm used to go to
            // ST_IDLE -- the RTL's de facto reset hub, target of 19 arcs, whose
            // ONLY exit is `if (en_i)` (:525) and which additionally clears
            // gen_os_ctrl_c and idle_to_rlock_transitioned.  With en_i
            // deasserted the LTSSM stopped there permanently, on a path the
            // spec requires to retry.  tracker sec 54 #8 (oracle D7).
            next_state = ST_DETECT_QUIET;
          end
        end else if (timer_r >= TwentyFourMsTimeOut) begin
          // NOT changed with D7, deliberately: this is the 24 ms watchdog for a
          // phystatus that never arrives -- a failsafe, not a spec limb -- and
          // no oracle covers it.  D11 (verilate_ltssm_24ms) exercises this arm
          // and expects today's behaviour.  See PREDICTIONS_D7.md sec 2.
          next_state =  ST_IDLE; // Should technically be ST_DETECT_QIUET
        end
      end
      //*********************************************************
      // Detect.Recever.Detection
      //*********************************************************
      // In this state we need to wait 12ms, and then perform another receiver detection.
      // If the same lanes detect a receiver we can go to ST_POLLING (technically, all undetected lanes need to transition to electrical idle...)
      // If the lanes change we go back to ST_IDLE.
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
              // Base 2.1 4.2.6.1.2 p.219: when the second Receiver Detection
              // finds a DIFFERENT set of Lanes, "the next state is Detect.Quiet"
              // -- an ordinary retry, not a training failure.  This arm used to
              // do two non-conformant things at once: detour through ST_IDLE
              // (whose only exit is `if (en_i)` at :525, so a deasserted en_i
              // stopped the LTSSM permanently on a retry path) and raise
              // error_c, reporting a failure the spec does not consider one.
              // Both are removed.  tracker sec 54 #8 (oracle D10).
              //
              // The error_c removal was BLOCKED until fix-arc 6b: verilate_ltssm_obs
              // provoked its error_o oracle through THIS site, so deleting the
              // raise would have broken the witness for sec 54 #2.  obs was
              // re-anchored first, in its own commit, to :934's Lanenum.Wait
              // timeout -- a site whose recorded verdict is *conforms* (oracle
              // C13) and which is on no open-defect list.
              // evidence/fix-arc-6/FINDINGS_D10_COUPLING.md.
              next_state = ST_DETECT_QUIET;
            end
          end
        end else if (timer_r >= TwentyFourMsTimeOut) begin
          next_state = ST_IDLE;
        end
      end
      //*********************************************************
      // Polling
      //*********************************************************
      ST_POLLING: begin
        // timer_c                = '0;
        next_state             = ST_POLLING_ACTIVE;
        ordered_set_sent_cnt_c = '0;
        gen_os_ctrl_c          = '0;
        // gen_os_ctrl_c.gen_idle = '1;
        // gen_os_ctrl_c.gen_idle = '1;
        gen_os_ctrl_c.gen_idle = '0;
        gen_os_ctrl_c.valid    = '1;
        gen_os_ctrl_c.gen_ts1  = '1;
        transmit_ordered_set   = '1;
        ordered_set_c = gen_ts_os( gen1, TS1);
      end
      //*********************************************************
      // Polling.Active
      //*********************************************************
      ST_POLLING_ACTIVE: begin
        //bounded timeout counter
        // timer_c = (timer_r >= TwentyFourMsTimeOut) ? TwentyFourMsTimeOut : timer_r + 1;
        //The Transmitter must wait for its TX common mode to settle before exiting from Electrical
        //Idle and transmitting the TS1 Ordered Sets.
        // Phy transmitter handles common mode settling, will throttle with tready
        //check if timer reached or TSOS sent count met
        //check if last packet in frame
        if (ordered_set_tranmitted_i) begin
          // Only start counting after receiving one TS1
          if (|single_ts1_received ) begin
          ordered_set_sent_cnt_c = ordered_set_sent_cnt_r + 1;
          end
          if (|polarity_inverted_i && (polarity_lockout_timer_r == 0)) begin
            phy_rxpolarity_c = phy_rxpolarity_r ^ polarity_inverted_i;
            polarity_lockout_timer_c = 16'd1000; // ~10us lockout
          end

          // P4: the PRIMARY exit counts from ENTRY to Polling.Active (p.220);
          // the 24 ms branch below keeps ordered_set_sent_cnt_r, which carries
          // p.221's "after receiving one TS1" qualifier.  Two limbs, two
          // counters.  See polling_tx_cnt_r's declaration.
          if ((polling_tx_cnt_r >= MinTS1sPolling)) begin
              if (&lanes_ts1_satisfied || &lanes_ts2_satisfied) begin
                ordered_set_sent_cnt_c = '0;
                //build ts2 ordered set
                gen_os_ctrl_c.gen_ts1 = '0;
                gen_os_ctrl_c.gen_ts2 = '1;
                // ordered_set_c = gen_ts_os( gen1, TS1,PAD_,PAD_,'1);
                ordered_set_c = gen_ts_os( gen1, TS2);
                transmit_ordered_set = '1;
                //goto cofig
                next_state = ST_POLLING_CONFIGURATION;
              end
          end
          if ((timer_r >= TwentyFourMsTimeOut) && (ordered_set_sent_cnt_r >= MinTS1sPolling)) begin
            //reset counts
            // timer_c                = '0;
            ordered_set_sent_cnt_c = '0;
            // P6 (Base 2.1 4.2.6.2.1 p.221; tracker SS54 #8).  This branch
            // reaches Polling.Configuration only if BOTH limbs hold: the
            // training-sequence limb below AND limb (ii), "at least a
            // predetermined number of Lanes that detected a Receiver during
            // Detect have detected an exit from Electrical Idle at least once
            // since entering Polling.Active".  Limb (ii) was absent entirely.
            //
            // ⚠️ "a predetermined number" is implementation-defined; this design
            // fixes it at ONE -- the weakest conforming choice, and the
            // |-reduction this file already uses for every other "any Lane"
            // condition.  Stated as a choice, not read out of the spec.
            //
            // ⚠️ receiver_detected_i is the mask, NOT lane_active_r: the spec
            // says "Lanes that detected a Receiver during Detect", which is
            // literally that port, and it is the same mask lanes_ts1_satisfied /
            // lanes_ts2_satisfied are built from.
            //
            // ⚠️ :693's PRIMARY exit is deliberately NOT given this conjunct --
            // p.220 states no Electrical Idle limb on it.  A symmetric edit would
            // have been a new defect.
            //
            // ⚠️ PREDICTED CONSEQUENCE, registered before it was measured: adding
            // this conjunct BREAKS THE SUBSUMPTION that made the else-if below
            // dead code.  |lanes_ts1_satisfied used to imply this test, so
            // ST_POLLING_COMPLIANCE was structurally unreachable (Rung 10a /
            // CENSUS_LTSSM section 2.1).  It is now reachable exactly when the
            // training limb holds and the Electrical Idle limb does not -- which
            // is precisely the case p.221(a) routes to Polling.Compliance.  The
            // dead arm was written for this and has been waiting for its guard.
            // Oracle P7's "unreachable" verdict is superseded.  See
            // evidence/fix-arc-6/PREDICTIONS_P6.md section 4.
            //check if ts1 reqs satisfied AND the Electrical Idle limb
            if ((|lanes_ts1_satisfied || |lanes_ts2_satisfied) &&
                (|(polling_ei_exit_seen_r & receiver_detected_i))) begin
              //build ts2 ordered set
              gen_os_ctrl_c.gen_ts1 = '0;
              gen_os_ctrl_c.gen_ts2 = '1;
              // ordered_set_c = gen_ts_os( gen1, TS1,PAD_,PAD_,'1);
              ordered_set_c = gen_ts_os( gen1, TS2);
              transmit_ordered_set = '1;
              //goto cofig
              next_state = ST_POLLING_CONFIGURATION;
            end else if (|lanes_ts1_satisfied) begin
              // TODO: This should be entered when a 24 ms timeout is reached, 1024 TS1s were sent and
              // Any lane received 8 consecutive TS1s with the copmbliance rceive bit of symbol 5 == 1 and loopback bit == 0
              next_state = ST_POLLING_COMPLIANCE;
            end else begin
              // Neither lanes_ts1_satisfied nor lanes_ts2_satisfied is set on
              // any lane -- the link partner never responded at all during
              // Polling.Active. next_state is left alone here (still ==
              // curr_state), so the generic 24ms watchdog below still sends
              // us to ST_IDLE either way; this just makes sure error_o
              // distinguishes "no response at all" from the other paths
              // through this state instead of silently falling through.
              // (1'b1, not '1 -- Verilator 5.050 hits a parser edge case
              // with the unsized literal as the sole statement in a bare
              // else-begin block at this exact position; functionally
              // identical for a 1-bit reg.)
              error_c = 1'b1;
            end
          end
        end  // end of: if (ordered_set_tranmitted_i)

        // 24ms Polling watchdog. Must NOT be gated behind ordered_set_tranmitted_i:
        // a stalled TX handshake is exactly the failure this failsafe exists to
        // catch, and gating it there defeats its purpose (Bug 4).
        // The (next_state == curr_state) guard ensures the watchdog only fires
        // when no success path above has already claimed a transition -- without
        // it, this check would clobber a legitimate ST_POLLING_CONFIGURATION
        // transition that happened to occur at >= 24ms.
        if ((timer_r >= TwentyFourMsTimeOut) && (next_state == curr_state)) begin
          next_state = ST_IDLE;
        end
      end
      //*********************************************************
      // Polling.Compliance: NOT IMPLEMENTED
      //*********************************************************
      ST_POLLING_COMPLIANCE: begin
        //not implemented
        //assert error and go back to deteect low
        error_c    = '1;
        next_state = ST_IDLE;
      end
      //-----------------------------------------------------------
      //  Polling.Configuration
      //-----------------------------------------------------------
      ST_POLLING_CONFIGURATION: begin
        // P9 (Base 2.1 4.2.6.2.3 p.224; tracker SS54 #11): "Receiver must invert
        // polarity if necessary (see Section 4.2.4.4)" is a requirement OF THIS
        // SUBSTATE.  The capability existed only in Polling.Active, so an
        // inversion first needed here was never applied.
        //
        // ⚠️ This ADDS a writer; it does NOT move the Polling.Active one.  p.220
        // places the same requirement there, and
        // verilate_config_gaps::run_test_p9_polarity_in_polling_config asserts as
        // its POSITIVE CONTROL that Polling.Active still inverts -- precisely so
        // that "did not change" here cannot be a vacuous green.  A move turns that
        // control red.
        //
        // ⚠️ Deliberately NOT gated on ordered_set_tranmitted_i, unlike its
        // Polling.Active twin.  Polarity inversion is a RECEIVER action; gating a
        // receive-side response on a transmit handshake is the shape of Bug 4 and
        // Bug 5, both already fixed out of this file.
        if (|polarity_inverted_i && (polarity_lockout_timer_r == 0)) begin
          phy_rxpolarity_c = phy_rxpolarity_r ^ polarity_inverted_i;
          polarity_lockout_timer_c = 16'd1000; // ~10us lockout
        end

        //bounded timeout counter
        if (ordered_set_tranmitted_i && |single_ts2_received) begin
            ordered_set_sent_cnt_c = ordered_set_sent_cnt_r + 1'b1;
        end

        if (|lanes_ts2_satisfied && ordered_set_sent_cnt_r >= 8'h10) begin
          //assert success
          success_c = '1;
          //reset counts
          // timer_c    = '0;
          ordered_set_sent_cnt_c = '0;
          gen_os_ctrl_c.gen_ts1 = '1;
          gen_os_ctrl_c.gen_ts2 = '0;
          transmit_ordered_set = '1;
          // RC originates LINK_NUM from the first TS1 of Configuration;
          // EP still offers PAD/PAD until it has something to latch (fix #6).
          ordered_set_c = IS_ROOT_PORT
              ? gen_ts_os( gen1, TS1, train_seq_e'(link_number_selected))
              : gen_ts_os( gen1, TS1);
          //goto wait low
          next_state = ST_CONFIGURATION_LINKWIDTH_START;
        end  //check timeout count
        else if (timer_r >= FourtyEightMsTimeOut)
        begin
          // timer_c    = '0;
          //assert error.
          error_c    = '1;
          //goto wait low
          next_state = ST_IDLE;
        end

      end
      //-----------------------------------------------------------
      //  Configuration
      //-----------------------------------------------------------
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
      //-----------------------------------------------------------
      //  Configuration.Linkwidth.Start
      //-----------------------------------------------------------
      ST_CONFIGURATION_LINKWIDTH_START: begin
        // if (ordered_set_sent_cnt_r) begin
        //   transmit_ordered_set = '1;
        //   ordered_set_c = gen_ts_os( gen1, TS1, train_seq_e'(LINK_NUM));
        // end
        // gen_os_ctrl_c.valid = '1;
        // gen_os_ctrl_c.gen_ts1 = '1;
        if (ordered_set_tranmitted_i) begin
          ordered_set_sent_cnt_c = ordered_set_sent_cnt_r + 1'b1;
          //check if pcie state continue scenario satisfied
          // TODO: Either only wait for two consecutive TS1s with correct link number (|link_width_satisfied)
          // Or send 16-32 TS1s to support crosslink??? ((|link_width_satisfied) && (ordered_set_sent_cnt_r >= 8'h10))
          if ((|link_width_satisfied)) begin
            //reset ordered set sent counter
            ordered_set_sent_cnt_c = '0;
            transmit_ordered_set   = '1;
            //build next ordered set
            ordered_set_c = gen_ts_os( gen1, TS1, train_seq_e'(link_number_selected));
            //goto next pcie ltssm state
            next_state = ST_CONFIGURATION_LINKWIDTH_ACCEPT;
          end
        end  // end of: if (ordered_set_tranmitted_i)

        if ((timer_r >= TwentyFourMsTimeOut) && (next_state == curr_state)) begin
          //assert error
          error_c    = '1;
          //goto detect
          next_state = ST_IDLE;
        end
      end
      //-----------------------------------------------------------
      //  Configuration.Linkwidth.Accept
      //-----------------------------------------------------------
      ST_CONFIGURATION_LINKWIDTH_ACCEPT: begin
        // gen_os_ctrl_c.gen_ts1 = '1;
        //bounded counter for timeout scenario
        gen_os_ctrl_c.valid = '1;
        if ((ordered_set_tranmitted_i)) begin
          ordered_set_sent_cnt_c = ordered_set_sent_cnt_r + 1'b1;
          //check if pcie state continue scenario satisfied.
          //Advance once any lane has formed (>=2 consecutive matching TS1s);
          //the responding subset is then selected per-lane via lane_active_r
          //gating downstream. (The old `!(^link_lanes_formed)` parity gate was
          //deleted -- see commit message: it required an EVEN number of formed
          //lanes, which rejects x1, and parity does not express "one contiguous
          //link" anyway.)
          //TODO(contiguity): no check that the formed lanes are contiguous from
          //lane 0 / constitute a single link; needed for fragmentation and
          //crosslink rejection, unimplemented.
          //
          // C8 (Base 2.1 4.2.6.3.2.1 p.230; tracker SS54 #11; row
          // verilate_config_c8).  The exit condition is the FORMING CONDITION
          // ALONE -- "If a configured Link can be formed with at least one group
          // of Lanes that received two consecutive TS1 Ordered Sets with the same
          // received Link number ... The next state is
          // Configuration.Lanenum.Wait."  This substate states NO
          // transmitted-Ordered-Set count.  That silence is meaningful rather
          // than merely absent: Polling.Active (1024) and Configuration.Complete
          // (16 after receiving one) both state theirs explicitly.
          // link_lanes_formed[lane] <= (ts1_cnt >= 8'h2) already implements the
          // whole condition on its own.
          //
          // The `ordered_set_sent_cnt_r >= 8'h08` gate removed here was
          // unsourced and delayed the exit by eight transmissions (measured at
          // 8-9 pulses against a spec budget of 2).  It was CONSERVATIVE -- it
          // delayed, it never skipped -- which is why it sat open behind rows
          // that all looked green.
          //
          // Lanenum.Accept's two `>= 8'h8` gates below (:898, :908) are
          // DELIBERATELY LEFT ALONE: they belong to oracles C14 and C15, whose
          // recorded verdicts are "conforms" and "conforms (loosely)", and no
          // confirmed divergence names them.
          if ((|link_lanes_formed)) begin
            ordered_set_sent_cnt_c = '0;
            gen_os_ctrl_c.gen_ts1  = '1;
            gen_os_ctrl_c.gen_ts2  = '0;
            transmit_ordered_set   = '1;
            // This exit build feeds the ordered set transmitted during
            // Configuration.Lanenum.Wait -- the state where a downstream/root
            // port assigns Lane numbers. RC must therefore already carry an
            // assigned Lane number here (0 at x1), not PAD, or it sits in
            // Lanenum.Wait transmitting PAD forever and its peer never changes
            // its lane number -> 2ms timeout -> error -> ST_IDLE. EP still
            // offers PAD until Complete (unchanged).
            // TODO(x4): per-lane lane number assignment requires per-lane TX path.
            ordered_set_c = IS_ROOT_PORT
                ? gen_ts_os( gen1, TS1, train_seq_e'(link_number_selected), train_seq_e'(0))
                : gen_ts_os( gen1, TS1, train_seq_e'(link_number_selected));
            next_state = ST_CONFIGURATION_LANENUM_WAIT;
          end
        end  // end of: if (ordered_set_tranmitted_i)

        // C9 (Base 2.1 4.2.6.3.2.1 p.230): "The next state is Detect after a 2 ms
        // timeout OR if no Link can be configured OR if all Lanes receive two
        // consecutive TS1 Ordered Sets with Link and Lane numbers set to PAD
        // (K23.7)."  Only the timeout limb existed; the all-PAD limb is added
        // here.  Without it a partner that withdraws its Link number cannot tear
        // the link down promptly -- it must wait out the full 2 ms.
        //
        // ⚠️ The "no Link can be configured" limb is NOT implemented.  It has no
        // observable form at x1, no row measures it, and inventing one is a new
        // feature rather than a fix.  Registered as owed, like C12/C19/C20.
        //
        // ⚠️ (|lane_active_r) is an ANTI-VACUITY guard, not a spec term -- see
        // lanes_all_pad's declaration.  error_c is raised for consistency with
        // every other Configuration->Detect arc here and with SS54 #2's premise
        // that a training failure must be observable; p.230 does not require it.
        if ((&lanes_all_pad) && (|lane_active_r) && (next_state == curr_state)) begin
          error_c    = '1;
          next_state = ST_IDLE;
        end

        if ((timer_r >= TwoMsTimeOut) && (next_state == curr_state)) begin
          error_c    = '1;
          next_state = ST_IDLE;
        end
      end
      //-----------------------------------------------------------
      // Configuration.Lanenum.Accept
      //-----------------------------------------------------------
      ST_CONFIGURATION_LANENUM_ACCEPT: begin
        //bounded counter for timeout scenario
        if (ordered_set_tranmitted_i) begin
          ordered_set_sent_cnt_c = ordered_set_sent_cnt_r + 1'b1;
          //check if lanes can be formed
          if (|link_lanes_nums_match && ordered_set_sent_cnt_r >= 8'h8) begin
            //build ts2 ordered set
            transmit_ordered_set  = '1;
            gen_os_ctrl_c.gen_ts1 = '0;
            gen_os_ctrl_c.gen_ts2 = '1;
            ordered_set_c = gen_ts_os( gen1, TS2, train_seq_e'(link_number_selected), train_seq_e'(0));
            ordered_set_sent_cnt_c = '0;
            //goto config complete
            next_state = ST_CONFIGURATION_COMPLETE;
          end  //check reconfiguration scenario
          else if (|link_lane_reconfig && ordered_set_sent_cnt_r >= 8'h8)
          begin
            next_state = ST_CONFIGURATION_LANENUM_WAIT;
          end
        end  // end of: if (ordered_set_tranmitted_i)

        // C16 (Base 2.1 4.2.6.3.3.1 p.233): "The next state is Detect if no Link
        // can be configured or if all Lanes receive two consecutive TS1 Ordered
        // Sets with Link and Lane numbers set to PAD (K23.7)."  Neither limb
        // existed.  The all-PAD one is added here; "no Link can be configured" is
        // owed, as in Linkwidth.Accept above.
        if ((&lanes_all_pad) && (|lane_active_r) && (next_state == curr_state)) begin
          error_c    = '1;
          next_state = ST_IDLE;
        end

        // ⚠️ The 2 ms watchdog below is EXTRA-SPEC -- p.233 states no timeout for
        // this substate -- and it is DELIBERATELY KEPT.  Two measured reasons,
        // not a preference:
        //
        //  * verilate_config_timeout::run_test_config_lanenum_accept_timeout is an
        //    ORDINARY PASS row (Bug 5 regression coverage, nothing to do with
        //    SS54 #11).  It walks here, silences the TX handshake, and REQUIRES
        //    this watchdog to reach ST_IDLE inside 250 000 cycles.  Deleting it
        //    turns a green row red -- a fix breaking another row's witness, which
        //    is the coupling SS54.W's setup-route axis exists to prevent.
        //  * Rung 10a classified Detect.Active's structurally identical
        //    extra-spec watchdog (oracle D11) as "conformant-but-added" -- an
        //    ADDITION, not a violation -- and verilate_ltssm_24ms CHARACTERISES
        //    it rather than holding it red.  Same class, same treatment.
        //
        // C16 therefore closes as HALF a fix: the missing exit is added, the added
        // watchdog is closed documented-correct.  See
        // evidence/fix-arc-6/PREDICTIONS_C9_C16_C18_P9.md section 0.
        if ((timer_r >= TwoMsTimeOut) && (next_state == curr_state)) begin
          //assert error
          error_c    = '1;
          //reset counter
          //goto detect
          next_state = ST_IDLE;
        end
      end
      //-----------------------------------------------------------
      //  Configuration.Lanenum.Wait
      //-----------------------------------------------------------
      ST_CONFIGURATION_LANENUM_WAIT: begin
        if (ordered_set_tranmitted_i) begin
          //check if lane wait exit scenario satisfied
          ordered_set_sent_cnt_c = ordered_set_sent_cnt_r + 1'b1;
          if ((|ts1_lanenum_wait_satisfied)) begin
            // timer_c = '0;
            ordered_set_sent_cnt_c = 0;
            gen_os_ctrl_c.gen_ts1  = '1;
            gen_os_ctrl_c.gen_ts2  = '0;
            transmit_ordered_set   = '1;
            gen_os_ctrl_c.set_lane = '1;
            // RC assigns Lane Number 0 here (x1 only -- constant, not
            // per-lane). EP still offers PAD until COMPLETE (unchanged).
            // TODO(x4): per-lane lane number assignment requires per-lane TX path.
            ordered_set_c = IS_ROOT_PORT
                ? gen_ts_os( gen1, TS1, train_seq_e'(link_number_selected), train_seq_e'(0))
                : gen_ts_os( gen1, TS1, train_seq_e'(link_number_selected));
            //goto lanenum accept
            next_state = ST_CONFIGURATION_LANENUM_ACCEPT;
          end
        end  // end of: if (ordered_set_tranmitted_i)

        if ((timer_r >= TwoMsTimeOut) && (next_state == curr_state)) begin
          //assert error
          error_c    = '1;
          //goto detect
          next_state = ST_IDLE;
        end
      end
      //-----------------------------------------------------------
      //  Configuration.Complete
      //-----------------------------------------------------------
      ST_CONFIGURATION_COMPLETE: begin
        if (ordered_set_tranmitted_i) begin
          if (|single_ts2_received) begin
          ordered_set_sent_cnt_c = ordered_set_sent_cnt_r + 1'b1;
          end
          //check exit scenario
          if (&lane_num_formed && (ordered_set_sent_cnt_r >= 8'd16)) begin
            //decrement counts
            ordered_set_sent_cnt_c = '0;

            //build idle ordered set
            transmit_ordered_set   = '1;
            ordered_set_c = gen_zeros();
            gen_os_ctrl_c.gen_ts2  = '0;
            gen_os_ctrl_c.gen_ts1  = '0;
            gen_os_ctrl_c.gen_idle = '1;
            //goto config idle
            next_state             = ST_CONFIGURATION_IDLE;
          end
        end  // end of: if (ordered_set_tranmitted_i)

        if ((timer_r >= TwoMsTimeOut) && (next_state == curr_state)) begin
          //assert error
          error_c    = '1;
          //goto idle
          next_state = ST_IDLE;
        end
      end
      //-----------------------------------------------------------
      //  Configuration.Idle
      //-----------------------------------------------------------
      ST_CONFIGURATION_IDLE: begin
        link_up_c = '1;
        //check if idle received
        if (|single_idle_received && ordered_set_tranmitted_i) begin
          //start counting idle OS sent
          ordered_set_sent_cnt_c = ordered_set_sent_cnt_r + 1;
        end
        //check if number of idle OS received and idle OS sent
        if ((&link_idle_satisfied) && (ordered_set_sent_cnt_r >= 8'd16)) begin
          //assert success.. tells ltssm hierarchy to move to its next state            
          success_c                    = '1;
          //reset counters
          ordered_set_sent_cnt_c       = '0;
          gen_os_ctrl_c.gen_ts1        = '0;
          gen_os_ctrl_c.gen_ts2        = '0;
          gen_os_ctrl_c.gen_idle       = '0;
          gen_os_ctrl_c.valid          = '0;
          transmit_ordered_set         = '1;
          idle_to_rlock_transitioned_c = '0;
          //goto wait for ena low
          next_state                   = ST_L0;
        end  //check timeout counter
        else if (timer_r >= TwoMsTimeOut)
        begin
          if (idle_to_rlock_transitioned_r < 8'hFF) begin
            // Compare the rate FIELD, not the whole rate_id_t.  The struct is
            // {speed_change[7], autonomous_change[6], rate[5:1], rsvd0[0]}
            // (pcie_phy_pkg.sv:247-252), so a rate_id_t carrying gen1 holds
            // gen1<<1 == 2 while the bare enum zero-extends to 1: the struct
            // form was identically false for every rate, the 8'hFF saturation
            // below was dead, and the else-branch increment admitted 255
            // diversions to Recovery.RcvrLock where Base 2.1 4.2.6.3.6 p.237
            // permits one.  :530, :1327, :1477 and :1480 all compare the field.
            if (curr_data_rate_r.rate == gen1 || curr_data_rate_r.rate == gen2) begin
              idle_to_rlock_transitioned_c = 8'hFF;
            end else begin
              idle_to_rlock_transitioned_c = idle_to_rlock_transitioned_r + 1;
            end
            // Build the ordered set this exit is about to transmit.
            //
            // This arm wrote NEITHER gen_os_ctrl_c NOR ordered_set_c, and both
            // are sticky (defaulted to their registered value at the top of the
            // block).  So the FSM arrived in Recovery.RcvrLock still carrying
            // Configuration.Idle's own control word -- gen_idle=1, gen_ts1=0,
            // measured -- and transmitted IDLE where Base 2.1 p.239 requires
            // Recovery.RcvrLock to transmit TS1 Ordered Sets.  Rung 10c A3-4.
            //
            // Note the contrast one branch up: the SUCCESS path at :975-:978
            // clears all four control bits before leaving.  Only the timeout
            // path forgot, which is why the defect is invisible on a link that
            // trains normally and appears only after a 2 ms Configuration.Idle
            // timeout.
            //
            // Shape copied from ST_L0's own entry into the SAME state
            // (:1020-:1024 pre-edit) rather than invented -- that is the
            // in-tree precedent for "enter RcvrLock correctly".
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
            //assert error
            error_c                      = '1;
            //goto wait low
            next_state                   = ST_IDLE;
          end
        end
      end
      //-----------------------------------------------------------
      //  L0
      //-----------------------------------------------------------
      ST_L0: begin
        link_up_c = '1;
        success_c = '1;
        idle_to_rlock_transitioned_c = '0;

        // ===================================================================
        // sec 63 #7j-2 -- L0 KEEPS LOGICAL IDLE REQUESTED.
        //
        // Base 2.1 sec 4.2.2 p.195: "When no packet information or special
        // Ordered Sets are being transmitted, the Transmitter is in the
        // Logical Idle state.  During this time idle data must be
        // transmitted.  The idle data must consist of the data byte 0 (00
        // Hexadecimal), scrambled according to the rules of Section 4.2.3 ..."
        // and, in the same section, "Logical Idle is defined to be a period of
        // one or more Symbol Times when no information ... is being
        // Transmitted/Received.  Unlike Electrical Idle, during Logical Idle
        // the Idle Symbol (00h) is being transmitted and received."
        //
        // The machinery for this already existed and was already spec-exact:
        // gen_zeros() (pcie_phy_pkg.sv:548) -> gen_idle -> os_generator.sv:285
        // clears special_k so all 16 Symbols go out as DATA 00h ->
        // gen1_scramble -> Base 2.1 Table B p.700, matched byte-for-byte over
        // all 304 published bytes at offset 0.  What was missing was a
        // REQUEST: entry to L0 clears gen_idle at :1214 and, before this
        // change, ST_L0 contained no gen_idle reference at all -- established
        // by parsing all 31 state arms, not by a bare grep.  Idle was an entry
        // action of two substates (:1183 Configuration.Idle, :1539
        // Recovery.Idle) rather than the Transmitter's default.
        //
        // So between packets in L0 the link fell silent: the PIPE carried a
        // stale scrambled word, frozen and repeated, with valid low.
        //
        // !! THE STROBE IS DELIBERATELY NOT ASSERTED HERE, and that is a
        // measured decision, not a tidy-up.  ST_L0 used to set
        // transmit_ordered_set = '1 unconditionally on this line.  With the
        // strobe high, os_generator's ST_SEND streaming lock (:363) breaks at
        // every Ordered-Set boundary and the FSM returns to ST_IDLE -- where
        // `if (gen_os_ctrl_i.valid) D.skp_cnt = '0` (:203) RESETS THE SKP
        // TIMER.  Under a continuous idle request that happens every ~8
        // cycles, so skp_cnt can never reach SkpIntervalCounts (678) and the
        // SKP schedule is starved outright: measured ZERO SKP Ordered Sets in
        // 2000 cycles with the strobe, two without it.  Base 2.1 sec 4.2.2
        // p.195 is explicit that it may not be -- "During transmission of the
        // idle data, the SKP Ordered Set must continue to be transmitted as
        // specified in Section 4.2.7."  test_7j2_idle.py's C4 is that row.
        //
        // Dropping the strobe costs nothing else: send_ordered_set_o feeds
        // exactly one consumer, os_generator's send_ltssm_os_i, and that port
        // is read in exactly one expression -- the streaming-lock condition.
        // ===================================================================
        gen_os_ctrl_c.gen_idle       = '1;
        gen_os_ctrl_c.gen_ts1        = '0;
        gen_os_ctrl_c.gen_ts2        = '0;
        gen_os_ctrl_c.valid          = '1;
        ordered_set_c                = gen_zeros();

        if (|ts1_valid_i || |ts2_valid_i || (directed_speed_change_i && !changed_speed_recovery_r) || recovery_i)  // 63 #7k: p.248 "Recovery if directed" -- the DLL's REPLAY_NUM rollover (a level, synchronised by the top)
        begin
          gen_os_ctrl_c.gen_ts1 = '1;
          // sec 63 #7j-2, CONSTRAINT 4 -- gen_idle handling on LEAVING L0 is
          // unchanged, which now takes an explicit clear because the state
          // above sets it and gen_os_ctrl_c is sticky (:587).
          //
          // !! Forgetting this does not degrade training, it BREAKS it.
          // os_generator's ST_BUILD applies gen_idle AFTER the TS K-mask and
          // wipes it wholesale (`if (gen_os_ctrl_i.gen_idle) D.special_k = '0`,
          // os_generator.sv:285), so the TS1's Symbol-0 COM would be
          // transmitted as SCRAMBLED DATA and the peer's 16-Symbol matcher
          // would never see an Ordered Set at all.
          gen_os_ctrl_c.gen_idle = '0;
          gen_os_ctrl_c.valid = '1;
          transmit_ordered_set = '1;
          ordered_set_c = gen_ts_os( curr_data_rate_r.rate, TS1, train_seq_e'(link_number_selected),
                  train_seq_e'(0), last_data_rate_c);
          ordered_set_sent_cnt_c = '0;
          next_state = ST_RECOVERY_RCVR_LOCK;
        end
      end
      //-----------------------------------------------------------
      //  Recovery
      //-----------------------------------------------------------
      ST_RECOVERY: begin
        // timer_c = timer_r + 1'b1;

        if (timer_r >= 8'h0A) begin
          rate_id_t temp_rate_id;
          // timer_c = '0;
          temp_rate_id = gen3_basic;
          gen_os_ctrl_c.gen_ts1 = '1;
          // gen_os_ctrl_c.set_lane = '1;
          gen_os_ctrl_c.valid = '1;
          //if data rate is gen1 and we've tried three times stay at gen1
          if ((last_data_rate_r.rate > gen1) && (try_cnt_r < 8'h3) && !successful_speed_negotiation_r)
          begin
            last_data_rate_c.speed_change = '1;
            temp_rate_id.speed_change = '1;
          end
          transmit_ordered_set = '1;
          ordered_set_c = gen_ts_os( curr_data_rate_r.rate, TS1, train_seq_e'(link_number_selected),
                   train_seq_e'(0), last_data_rate_c);
          ordered_set_sent_cnt_c = '0;
          // if (recovery_i && !is_timeout_i) begin
          // ordered_set_c.rate_id[6] = '1;
          // end
          next_state             = ST_RECOVERY_RCVR_LOCK;
        end
      end
      //-----------------------------------------------------------
      //  Recovery.Lock
      //-----------------------------------------------------------
      ST_RECOVERY_RCVR_LOCK: begin
        //bounded counter for timeout scenario
        ts2_symbol6 = '0;
        if (equalization_requested && curr_data_rate_r.rate == gen3) begin
          next_state = ST_RECOVERY_EQUAL;
        end
        if (|speed_change_bit_set && !changed_speed_recovery_r) begin
          last_data_rate_c.speed_change = '1;
          transmit_ordered_set = '1;
          ordered_set_c = gen_ts_os( curr_data_rate_r.rate, TS1, train_seq_e'(link_number_selected),
                   train_seq_e'(0), last_data_rate_c);
        end
        if (&(ts1_cnt_satisfied | ts2_cnt_satisfied)) begin
          //deassert valid and reset counter
          ordered_set_sent_cnt_c = '0;
          // timer_c                = '0;
          if (extended_synch_i) begin
            //goto next pcie ltssm state
            next_state = ST_RECOVERY_EXT_SYNCH;
          end else begin
            //build next ordered set
            if (max_rate >= gen3) begin
              ts2_symbol6.req_equal = '1;
            end
            gen_os_ctrl_c.gen_ts1 = '0;
            gen_os_ctrl_c.gen_ts2 = '1;
            transmit_ordered_set  = '1;
            ordered_set_c = gen_ts_os( curr_data_rate_r.rate, TS2, train_seq_e'(link_number_selected),
                     train_seq_e'(0), last_data_rate_r, '0, ts2_symbol6);
            //goto next pcie ltssm state
            next_state = ST_RECOVERY_RCVR_CFG;
          end
        end  //check timeout counter
        else if (timer_r >= TwentyFourMsTimeOut)
        begin
          next_state = ST_RECOVERY_RCVR_LOCK_TIMEOUT;
        end
      end
      //-----------------------------------------------------------
      //  Recovery.Rcvr.Lock.Timeout
      //-----------------------------------------------------------
      ST_RECOVERY_RCVR_LOCK_TIMEOUT: begin
        //check secondary config transition
        if ((|((ts1_cnt_satisfied | ts2_cnt_satisfied) & lane_active_r) && (|speed_change_bit_set)) && (
            curr_data_rate_r.rate != gen1 ||
            max_rate != gen1 || last_data_rate_r.rate != gen1))
        begin
          //build next ordered set
          ts2_symbol6 = '0;
          if (max_rate >= gen3) begin
            // ts2_symbol6.req_equal = '1;
          end
          transmit_ordered_set = '1;
          ordered_set_c = gen_ts_os( rate_speed_e'(last_data_rate_r.rate), TS2, train_seq_e'(link_number_selected),
                   train_seq_e'(0), last_data_rate_r, '0, ts2_symbol6);
          //goto next pcie ltssm state
          next_state = ST_RECOVERY_RCVR_CFG;
        end else begin
          if (!changed_speed_recovery_r && curr_data_rate_r.rate != gen1) begin
            transmit_ordered_set = '1;
            ordered_set_c = gen_ts_os( rate_speed_e'(last_data_rate_r.rate), TS2,
                     train_seq_e'(link_number_selected), train_seq_e'(0), last_data_rate_r, '0, ts2_symbol6);
            //goto next pcie ltssm state
            next_state = ST_RECOVERY_SPEED;
          end else if (changed_speed_recovery_r) begin
            //goto next pcie ltssm state
            next_state = ST_RECOVERY_SPEED;
          end else if (changed_speed_recovery_r && (|at_least_one_ts1_ts2)) begin
            //assert error
            error_c    = '1;
            goto_cfg_c = '1;
            //goto detect
            next_state = ST_IDLE;
          end else begin
            //assert error
            error_c       = '1;
            goto_detect_c = '1;
            //goto detect
            next_state    = ST_IDLE;
          end
        end
      end
      ST_RECOVERY_EQUAL: begin
        ts1_symbol6_t temp_ts6;
        ordered_set_sent_cnt_c = '0;
        equal_status_c         = '0;
        // gen_os_ctrl_c          = '0;
        gen_os_ctrl_c.valid    = '1;
        gen_os_ctrl_c.gen_ts2  = '0;
        gen_os_ctrl_c.gen_ts1  = '1;
        // last_data_rate_c.speed_change = '1;
        temp_ts6.ec            = 2'b01;
        transmit_ordered_set   = '1;
        ordered_set_c = gen_ts_os( curr_data_rate_r.rate, TS1, train_seq_e'(link_number_selected),
                 train_seq_e'(0), last_data_rate_c,, temp_ts6);
        next_state = ST_RECOVERY_EQUAL_PHASE_1;
      end
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
        if (&(ts1_lanenum_wait_satisfied ^ ~lane_active_r)) begin
          equal_status_c.equal_complete = '1;
          equal_status_c.phase1_successful = '1;
          //skip phase 2 and 3
          //next_state = ST_RECOVERY_EQUAL_PHASE_2;
          next_state = ST_RECOVERY;
        end else if (timer_r >= TwentyFourMsTimeOut) begin
          next_state = ST_RECOVERY_SPEED;
          // timer_c = '0;
        end
      end
      ST_RECOVERY_EQUAL_PHASE_2: begin
        // timer_c = (timer_r >= TwentyFourMsTimeOut) ? TwentyFourMsTimeOut : timer_r + 1;
        if (ts1_cnt_satisfied) begin
          ts1_symbol6_t temp_ts6;
          temp_ts6.ec = 2'b11;
          transmit_ordered_set = '1;
          ordered_set_c = gen_ts_os( curr_data_rate_r.rate, TS1, train_seq_e'(link_number_selected),
                   train_seq_e'(0), last_data_rate_c,, temp_ts6);
          // next_state = ST_RECOVERY_EQUAL;
          // timer_c = '0;
          next_state = ST_RECOVERY_EQUAL_PHASE_3;
        end else if (timer_r >= TwentyFourMsTimeOut) begin
          next_state = ST_RECOVERY_SPEED;
          // timer_c = '0;
        end
      end
      ST_RECOVERY_EQUAL_PHASE_3: begin
        if (ts1_cnt_satisfied) begin
          gen_os_ctrl_c = '0;
          next_state = ST_RECOVERY_RCVR_LOCK;
        end else if (timer_r >= TwentyFourMsTimeOut) begin
          next_state = ST_RECOVERY_SPEED;
          // timer_c = '0;
        end
      end
      //-----------------------------------------------------------
      //  Recovery.Ext.Synch
      //-----------------------------------------------------------
      ST_RECOVERY_EXT_SYNCH: begin
        gen_os_ctrl_c.valid = '1;
        gen_os_ctrl_c.gen_ts1 = '1;
        gen_os_ctrl_c.set_lane = '1;
        //check if last packet in frame
        if (ordered_set_tranmitted_i) begin
          ordered_set_sent_cnt_c = ordered_set_sent_cnt_r + 1'b1;
        end
        //check if pcie state continue scenario satisfied
        if (ordered_set_sent_cnt_r >= 12'd1024) begin
          ts2_symbol6            = '0;
          //deassert valid and reset counter
          ordered_set_sent_cnt_c = '0;
          // timer_c                = '0;
          //build next ordered set
          if (max_rate == gen3) begin
            ts2_symbol6.req_equal = '1;
          end
          // ordered_set_c = gen_ts_os( last_data_rate_r.rate, TS2, link_num_i, lane_num_i, last_data_rate_r, '0,
          //          ts_symbol6_union_t);
          next_state = ST_RECOVERY_RCVR_CFG;
        end
      end
      //recovery speed scenario
      //8 TS2 Ordered on any lane sets with speed_change bit...at_least_one_ts1_ts2
      // and 8 TS2 OS are standard i.e no IEQUES TS2 if gen1/gen2
      //
      //8 consecutive EQ TS2 recived on all configured lanes, speed_change bit
      //set to 1
      //8 consecutive EQ TS2 OS
      //-----------------------------------------------------------
      //  Recovery.Rcvr.Cfg
      //-----------------------------------------------------------
      ST_RECOVERY_RCVR_CFG: begin
        //bounded counter for timeout scenario
        // gen_os_ctrl_c.gen_ts1 = '1;
        // gen_os_ctrl_c.set_lane = '1;
        // timer_c = (timer_r >= TwentyFourMsTimeOut) ? TwentyFourMsTimeOut : timer_r + 1;
        // gen_os_ctrl_c.valid = '1;
        if (ordered_set_tranmitted_i && at_least_one_ts1_ts2) begin
          ordered_set_sent_cnt_c = ordered_set_sent_cnt_r + 1'b1;
        end
        //recovery idle scenario
        // ALL configured Lanes, not any: Base 2.1 4.2.6.4.3 p.244 requires
        // eight consecutive TS2 "on all configured Lanes".  The `|` let one
        // Lane of four leave RcvrCfg.  The bare `&` is the spec form here
        // because ts2_cnt_satisfied is ALREADY lane-gated at :1613 (an
        // inactive Lane yields '1); keeping the `& lane_active_r` under a
        // &-reduction would zero every inactive Lane's term and hang a
        // reduced-width link -- the mirror of the trap at :1471/:1623.
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
          // timer_c                        = '0;
          ordered_set_sent_cnt_c         = '0;
          next_state                     = ST_RECOVERY_IDLE;
          transmit_ordered_set           = '1;
          ordered_set_c                  = gen_zeros();
        end
        if((|((ts1_cnt_satisfied || ts2_cnt_satisfied) & lane_active_r)) &&
            (|speed_change_bit_set) &&  (curr_data_rate_r.rate < gen3) &&
            (curr_data_rate_r.rate > gen1 || max_rate > gen1) &&
            ordered_set_sent_cnt_r >= 16'd32)
        begin
          // timer_c                = '0;
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
          // timer_c = '0;
          gen_os_ctrl_c          = '0;
          gen_os_ctrl_c.valid    = '1;
          gen_os_ctrl_c.gen_eios = '1;
          ordered_set_sent_cnt_c = '0;
          transmit_ordered_set   = '1;
          gen_eios(ordered_set_c, curr_data_rate_r.rate);
        end
        else if(&(ts1_cnt_satisfied | ts2_cnt_satisfied) && curr_data_rate_r.rate >= gen3
                && (&(speed_change_bit_set ^ lane_active_r)) && ordered_set_sent_cnt_r >= 32'd128)
        begin
          // timer_c                = '0;
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
      //-----------------------------------------------------------
      //  Recovery.Speed
      //-----------------------------------------------------------
      ST_RECOVERY_SPEED: begin
        tx_enter_elec_idle_o = '1;
        gen_os_ctrl_c.gen_ts1 = '1;
        gen_os_ctrl_c.set_lane = '1;
        // curr_data_rate_c.rate = max_supported_rate_r;
        //bounded counter for timeout scenario
        // timer_c = (timer_r >= TwentyFourMsTimeOut) ? TwentyFourMsTimeOut : timer_r + 1;
        gen_os_ctrl_c.valid = '1;
        // if (ordered_set_tranmitted_i) begin
        //   ordered_set_sent_cnt_c = ordered_set_sent_cnt_r + 1'b1;
        // end

        // if (curr_data_rate_r.rate == gen1 || curr_data_rate_r.rate == gen3) begin
        //   if (ordered_set_sent_cnt_r >= 8'h1) begin
        //     gen_os_ctrl_c.valid = '0;
        //   end
        //   if (&(phy_rxelecidle_i | ~lane_active_r)) begin
        //     //bounded counter for timeout scenario
        //     gen_os_ctrl_c.valid = '0;
        //     next_state = ST_RECOVERY_SPEED_WAIT;
        //   end
        // end else begin
        //   if (ordered_set_sent_cnt_r >= 8'h2) begin
        //     gen_os_ctrl_c.valid = '0;
        //   end
        //   if (&(phy_rxelecidle_i | ~lane_active_r)) begin
        //     gen_os_ctrl_c.valid = '0;
        //     //bounded counter for timeout scenario
        //     next_state = ST_RECOVERY_SPEED_WAIT;
        //   end
        // end
        if (ordered_set_tranmitted_i) begin
          ordered_set_sent_cnt_c = ordered_set_sent_cnt_r + 1'b1;
        end
        if (&(phy_rxelecidle_i | ~lane_active_r) && ordered_set_sent_cnt_r >= 2) begin
          gen_os_ctrl_c.valid = '0;
          //bounded counter for timeout scenario
          next_state = ST_RECOVERY_SPEED_WAIT;
        end
        //check timeout counter
        if (timer_r >= FourtyEightMsTimeOut) begin
          next_state = ST_IDLE;
        end
      end
      //-----------------------------------------------------------
      //  Recovery.Speed.Wait
      //-----------------------------------------------------------
      ST_RECOVERY_SPEED_WAIT: begin
        //bounded counter for timeout scenario
        // timer_c = (timer_r >= TwentyFourMsTimeOut) ? TwentyFourMsTimeOut : timer_r + 1;
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
          last_data_rate_c         = curr_data_rate_r.rate;
          transmit_ordered_set     = '1;
          ordered_set_c = gen_ts_os( last_data_rate_c.rate, TS1, train_seq_e'(link_number_selected),
                   train_seq_e'(0), last_data_rate_c);
          next_state = ST_RECOVERY_RCVR_LOCK;
        end
      end
      //-----------------------------------------------------------
      //  Recovery.Speed.Eieos
      //-----------------------------------------------------------
      //this state exists to ensure that eieos is transmitted before going into tx elec idle
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
      //-----------------------------------------------------------
      //  Recovery.Idle
      //-----------------------------------------------------------
      ST_RECOVERY_IDLE: begin
        //bounded counter for timeout scenario
        // timer_c = (timer_r >= TwentyFourMsTimeOut) ? TwentyFourMsTimeOut : timer_r + 1;
        gen_os_ctrl_c.valid = '1;
        if (ordered_set_tranmitted_i) begin
          if (single_idle_received) begin
            ordered_set_sent_cnt_c = ordered_set_sent_cnt_r + 1'b1;
          end
        end
        // ALL configured Lanes, not any: Base 2.1 4.2.6.4.4 p.246 requires
        // eight consecutive Symbol Times of Idle "on all configured Lanes".
        // The `|` let one Lane of four declare the link trained.  Reducing
        // with `&` needs the lane gate added in the same commit --
        // lanes_idle_satisfied was the only member of its family not gated by
        // lane_active_r, so `&` alone would wait forever on a Lane that is not
        // part of a reduced-width link.  The gate is at :1623.
        if (((&lanes_idle_satisfied) && ordered_set_sent_cnt_r >= 8'd16)) begin
        gen_os_ctrl_c                = '0;
        gen_os_ctrl_c.valid          = '0;
        next_state                   = ST_L0;
        idle_to_rlock_transitioned_c = '0;
        end else if (at_least_one_ts1_ts2) begin
          // timer_c                = '0;
          gen_os_ctrl_c.valid        = '1;
          ordered_set_sent_cnt_c     = '0;
          gen_os_ctrl_c.gen_ts1      = '1;
          gen_os_ctrl_c.gen_ts2      = '0;
          transmit_ordered_set       = '1;
          ordered_set_c = gen_ts_os(gen1, TS1);
          next_state             = ST_CONFIGURATION_LINKWIDTH_START;
        end else if (timer_r >= TwoMsTimeOut) begin
          //goto recovery scenario
          if (idle_to_rlock_transitioned_r != '1) begin
            // timer_c                = '0;
            gen_os_ctrl_c.valid    = '0;
            ordered_set_sent_cnt_c = '0;
            //check data rate for retry options
            // Saturate at Gen1 exactly as the Gen2 arm below does.  Base 2.1
            // 4.2.6.4.4 p.246 makes idle_to_rlock_transitioned a 0b/1b
            // variable: one diversion to Recovery.RcvrLock, and the next 2 ms
            // timeout goes to Detect.  Incrementing against the `!= '1` guard
            // at :1465 spent 255 timeouts -- roughly 510 ms -- before reaching
            // the else arm, and disagreed with Configuration.Idle's own
            // treatment of the same variable at :987-:991.
            if (curr_data_rate_r.rate == gen1) begin
              idle_to_rlock_transitioned_c = '1;
            end
            if (curr_data_rate_r.rate == gen2) begin
              idle_to_rlock_transitioned_c = '1;
            end
            next_state = ST_RECOVERY;
          end  //goto detect
          else
          begin
            // timer_c                = '0;
            gen_os_ctrl_c.valid    = '0;
            ordered_set_sent_cnt_c = '0;
            next_state             = ST_IDLE;
          end
        end
      end
      //-----------------------------------------------------------
      //  Recovery.Send.SDS
      //-----------------------------------------------------------
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


  //-----------------------------------------------------------
  //  Lane based Ordered set handling logic
  //-----------------------------------------------------------
  for (genvar lane = 0; lane < MAX_NUM_LANES; lane++) begin : gen_cnt_ts1
    //local helper counters
    (* mark_debug = "true" *) logic              [7:0] ts1_cnt;
    (* mark_debug = "true" *) logic              [7:0] ts2_cnt;
    logic              [7:0] idle_cnt;

    // C9/C16: consecutive all-PAD TS1 on this Lane.  Needs its own counter --
    // neither existing one can express the condition.  ts1_cnt SATURATES rather
    // than clearing on a mismatch in both Accept arms, which would turn "two
    // consecutive" into "two ever"; ts2_cnt is unwritten in Linkwidth.Accept but
    // is read by link_width_satisfied and link_lanes_nums_match, so repurposing
    // it would leak into two other substates' exits.
    // ⚠️ pad_cnt therefore RESETS to 0 on a mismatch -- deliberately unlike its
    // neighbours in the very same arms.  That difference IS the word
    // "consecutive".
    logic              [1:0] pad_cnt;

    logic              [7:0] lane_in_save;
    logic                    first_ts1;
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


    always_ff @(posedge clk_i) begin : output_registers
      if (rst_i) begin
        //determine if TS1 req satisfied for lane by its count
        link_width_satisfied[lane]       <= '0;
        //determine if TS1 req satisfied for lane by its count
        link_lanes_formed[lane]          <= '0;
        //determine if TS1 req satisfied
        ts1_lanenum_wait_satisfied[lane] <= '0;
        lanes_all_pad[lane]              <= '0;
        link_lanes_nums_match[lane]      <= '0;
        link_lane_reconfig[lane]         <= '0;
        lane_num_formed[lane]            <= '0;
        //determine if TS1 req satisfied for lane by its count
        link_idle_satisfied[lane]        <= '0;
        ts1_cnt_satisfied[lane]          <= '0;
        ts2_cnt_satisfied[lane]          <= '0;
        at_least_one_ts1_ts2[lane]       <= '0;
        //assignments for state exit scenarios
        lanes_ts1_satisfied[lane]        <= '0;
        lanes_ts2_satisfied[lane]        <= '0;
        lanes_idle_satisfied[lane]       <= '0;
        speed_change_bit_set[lane]       <= '0;
      end else begin
        //determine if TS1 req satisfied for lane by its count
        link_width_satisfied[lane]       <= (ts1_cnt >= 8'h2) | (ts2_cnt == 8'h2);
        //determine if TS1 req satisfied for lane by its count
        link_lanes_formed[lane]          <= (ts1_cnt >= 8'h2);
        //determine if TS1 req satisfied
        ts1_lanenum_wait_satisfied[lane] <= (ts1_cnt >= 8'h2);
        //C9/C16: two consecutive all-PAD TS1 on this Lane.  lane_active_r gate as
        //per link_idle_satisfied below; see the declaration for why the consumers
        //also require (|lane_active_r).
        lanes_all_pad[lane]              <= lane_active_r[lane] ? (pad_cnt >= 2'd2) : '1;
        link_lanes_nums_match[lane]      <= (ts1_cnt >= 8'h2) | (ts2_cnt >= 8'h2);
        link_lane_reconfig[lane]         <= (ts1_cnt >= 8'h2);
        lane_num_formed[lane]            <= lane_active_r[lane] ? (ts2_cnt == 8'h8) : '1;
        //determine if TS1 req satisfied for lane by its count
        //(ts1_cnt is repurposed as the idle count while curr_state ==
        //ST_CONFIGURATION_IDLE -- see that state's per-lane block, same
        //convention ST_RECOVERY_IDLE uses for its own counter -- so this is
        //not a mixup with idle_cnt/lanes_idle_satisfied, which belong to
        //ST_RECOVERY_IDLE's separate exit condition instead.) Gated by
        //lane_active_r like its siblings above/below so an inactive lane on
        //a reduced-width link contributes a trivial '1' to the &-reduction
        //at ST_CONFIGURATION_IDLE's exit check instead of blocking it
        //forever.
        link_idle_satisfied[lane]        <= lane_active_r[lane] ? (ts1_cnt >= 8'h8) : '1;
        ts1_cnt_satisfied[lane]          <= lane_active_r[lane] ? (ts1_cnt == 8'h8) : '1;
        ts2_cnt_satisfied[lane]          <= lane_active_r[lane] ? (ts2_cnt == 8'h8) : '1;
        at_least_one_ts1_ts2[lane]       <= (ts1_cnt_c != '0) | (ts2_cnt_c != '0);
        //assignments for state exit scenarios
        lanes_ts1_satisfied[lane]        <= receiver_detected_i[lane] ? (ts1_cnt == 8'h8) : '1;
        lanes_ts2_satisfied[lane]        <= receiver_detected_i[lane] ? (ts2_cnt == 8'h8) : '1;
        // Gated by lane_active_r like link_idle_satisfied/ts1_cnt_satisfied/
        // ts2_cnt_satisfied above, so an inactive Lane on a reduced-width link
        // contributes a trivial '1' to the &-reduction at ST_RECOVERY_IDLE's
        // exit (:1471) instead of blocking it forever.  This was the only
        // member of the family without the gate.
        lanes_idle_satisfied[lane]       <= lane_active_r[lane] ? (idle_cnt >= 8'h8) : '1;
        speed_change_bit_set[lane]       <= lane_speed_change_bit != '0;
      end

    end

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


    always_comb begin
      // =========================
      // DEFAULTS (HOLD)
      // =========================
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
      lane_num_echo_c = lane_num_echo[lane*8+:8];  // hold captured echo value
      max_rate_per_lane_c = max_rate_per_lane[lane];

      temp_ts6_c = temp_ts6;
      temp_rate_id_c = temp_rate_id;

      // =========================
      // GLOBAL TRANSITION RESET
      // =========================
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

          // =========================
          ST_IDLE: begin
            ts1_cnt_c ='0;
            ts2_cnt_c ='0;
            idle_cnt_c ='0;
            first_ts1_c ='0;

            single_idle_received_c ='0;
            single_ts1_received_c  ='0;
            single_ts2_received_c  ='0;

            lane_num_echo_c = PAD_;  // clear echo for a fresh training attempt
          end

          // =========================
          ST_POLLING_ACTIVE: begin
            if (ts1_valid_i[lane]) begin
              single_ts1_received_c = '1;

              // P3 (Base 2.1 4.2.6.2.1 p.220; tracker SS54 #11).  A TS1 qualifies
              // toward the eight consecutive training sequences only under
              //   (a) Lane and Link numbers PAD *and the Compliance Receive bit
              //       (Symbol 5 bit 4) is 0b*, or
              //   (b) Lane and Link numbers PAD *and the Loopback bit (bit 2)
              //       is 1b*.
              // The complementary case -- Compliance Receive 1b with Loopback 0b
              // -- satisfies NEITHER and is p.221's Polling.Compliance trigger.
              // It was being counted anyway, so the substate accepted a superset.
              // (~CR | LB) below is exactly (a) OR (b) with the shared PAD/PAD
              // conjunct factored out.
              //
              // ⚠️ Symbol 5 bit 4 is addressed POSITIONALLY, and that is forced:
              // training_ctrl_t (pcie_phy_pkg.sv:209-215) is
              //   {rsvd[7:4], scramble[3], loopback[2], dis_link[1], hot_rst[0]}
              // and has NO member for Compliance Receive -- the bit falls inside
              // rsvd, whose declared range makes rsvd[4] exactly that bit.  Naming
              // it properly is a pcie_phy_pkg change every PHY consumer of
              // training_ctrl_t sees; registered as owed rather than bundled in
              // here.  The same gap is why :720's Polling.Compliance arm below
              // stays an over-approximation (oracle P7).
              //
              // ⚠️ The ts2_valid_i arm below is DELIBERATELY untouched: p.220's
              // limb (c) is "TS2 with Lane and Link numbers set to PAD" and
              // carries no Symbol 5 condition.  A symmetric edit there would have
              // injected a defect.
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

          // =========================
          ST_POLLING_CONFIGURATION: begin
            if (ts2_valid_i[lane]) begin
              single_ts2_received_c ='1;

              if ((ordered_set_i[lane].link_num == PAD) && (ordered_set_i[lane].lane_num == PAD))
                ts2_cnt_c = (ts2_cnt >= 8'h8) ? 8'h8 : ts2_cnt + 1;
              else
                ts2_cnt_c = ts2_cnt >= 8'h8 ? 8'h8 : '0;
            end
          end

          // =========================
          ST_RECOVERY_RCVR_LOCK,
          ST_RECOVERY_RCVR_LOCK_TIMEOUT: begin
            //wait for incoming ts1-os...//skip if threshhold already reached
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

          // =========================
          ST_RECOVERY_RCVR_CFG: begin
            //wait for incoming ts1-os...//skip if threshhold already reached
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

          // =========================
          ST_RECOVERY_EQUAL_PHASE_1: begin
            if (ts1_valid_i[lane]) begin
              single_ts1_received_c ='1;

              if (ordered_set_i[lane].ts_s6.ts1.ec == 2'b01) begin
                ts1_cnt_c = (ts1_cnt >= 8'h2) ? 8'h2 : ts1_cnt + 1;
                single_idle_received_c ='0;
              end
            end
          end

          // =========================
          ST_RECOVERY_IDLE: begin
            //wait for incoming ts1-os...//skip if threshhold already reached
            //using ts1_cnt as idle count
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

          // =========================
          ST_CONFIGURATION_LINKWIDTH_START: begin
            //wait for incoming ts1-os...//skip if threshhold already reached
            if (ts1_valid_i[lane]) begin
              single_ts1_received_c ='1;

              if ((ordered_set_i[lane].link_num == PAD) &&
                  (ordered_set_i[lane].lane_num == PAD)) begin
                first_ts1_c ='1;
              end

              
              //check that link number is not pad and that lane number is pad
              //RC already knows its own LINK_NUM (originated, not latched --
              //see gen_link_number) and needs the peer to echo exactly that
              //value back, not just any non-PAD value; EP shape unchanged.
              if ((IS_ROOT_PORT ? (ordered_set_i[lane].link_num == link_number_selected)
                                 : (ordered_set_i[lane].link_num != PAD)) &&
                  (ordered_set_i[lane].lane_num == PAD)) begin
                //incrment ts1 count
                ts1_cnt_c = (ts1_cnt >= 8'h2) ? 8'h2 : ts1_cnt + 1;
              end else begin
                //reset ts1 cnt... this ensures that the TS1-OS are consecutive per the spec
                ts1_cnt_c = (ts1_cnt >= 8'h2) ? 8'h2 :'0;
              end
            end

            //check if consecutive TS1's satisfied for this lane
            if (link_width_satisfied[lane]) begin
              //select link number by choosing the lowest-numbered lane that
              //is currently satisfied -- not hardcoded to lane 0.
              //(1<<lane)-1 masks off every bit at position >= lane, leaving
              //just the lanes below it; for lane==0 this mask is 0 so the
              //check is trivially true (there is no lower lane), which is
              //why no separate lane==0 special case is needed here (and
              //avoids an invalid link_width_satisfied[lane-1:0] part-select
              //at lane==0, since `lane` is a genvar -- elaborated per
              //instance, not a runtime index -- and that range would
              //elaborate to [-1:0] for that instance).
              if ((link_width_satisfied & ((1 << lane) - 1)) == '0) begin
                link_number_selected_per_lane_c = ordered_set_i[lane].link_num;
                lane_link_number_selected_c ='1;
              end
            end
          end

          // =========================
          ST_CONFIGURATION_LINKWIDTH_ACCEPT: begin
            //wait for incoming ts1-os...//skip if threshhold already reached
            if (ts1_valid_i[lane]) begin
              single_ts1_received_c ='1;

              //check that incoming link number matches the "link_number_selected"
              //that we are now transmitting and that lane number is different
              //from the one stored when we entered this state
              if (ordered_set_i[lane].link_num == link_number_selected) begin
                //increment count
                ts1_cnt_c = (ts1_cnt >= 8'h2) ? 8'h2 : ts1_cnt + 1;
                lane_in_save_c = ordered_set_i[lane].lane_num;
              end else begin
                ts1_cnt_c = (ts1_cnt >= 8'h2) ? 8'h2 : '0;
              end

              //C9 (p.230): the COMPLEMENTARY condition -- two consecutive TS1
              //with BOTH Link and Lane numbers PAD.  Counted separately from
              //ts1_cnt because that counter saturates on a mismatch just above,
              //which cannot express "consecutive".
              if ((ordered_set_i[lane].link_num == PAD) &&
                  (ordered_set_i[lane].lane_num == PAD)) begin
                pad_cnt_c = (pad_cnt >= 2'd2) ? 2'd2 : pad_cnt + 2'd1;
              end else begin
                pad_cnt_c = '0;
              end
            end else if (ts2_valid_i[lane]) begin
              //a non-TS1 Ordered Set breaks the run of CONSECUTIVE TS1
              pad_cnt_c = '0;
            end
          end

          // =========================
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
              //EP reactive echo capture (TX-only, see lane_num_echo decl):
              //latch the Lane Number the downstream/root peer assigned on this
              //lane. Only a non-PAD value is a real assignment; capturing here
              //(not earlier) is why the EP transmits PAD until it has actually
              //been assigned a number -- which structurally prevents it from
              //announcing while the peer is still in Linkwidth.Accept.
              if (ordered_set_i[lane].lane_num != PAD) begin
                lane_num_echo_c = ordered_set_i[lane].lane_num;
              end
            end
          end

          // =========================
          ST_CONFIGURATION_LANENUM_ACCEPT: begin
            if (ts1_valid_i[lane])
              single_ts1_received_c ='1;
            else if (ts2_valid_i[lane])
              single_ts2_received_c ='1;

            if (ts1_valid_i[lane] || ts2_valid_i[lane]) begin
              //RC assigned this lane its physical index (see the per-lane
              //output stage) and confirms the peer echoed exactly that value.
              //`== lane` generalises the old x1 `== 8'h0` to x4 (lane==0 at x1,
              //so bit-identical there) and matches the COMPLETE check below.
              //EP still accepts any non-PAD lane number, unchanged.
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
              //EP reactive echo capture (continue holding/refreshing the
              //assigned Lane Number through Lanenum.Accept).
              if (ordered_set_i[lane].lane_num != PAD) begin
                lane_num_echo_c = ordered_set_i[lane].lane_num;
              end
            end

            //C16 (p.233): two consecutive all-PAD TS1 -- same counter and same
            //reasoning as the Linkwidth.Accept arm above.  One counter serves
            //both substates: the global transition reset zeroes it on every state
            //change, so they cannot interfere.
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

          // =========================
          ST_CONFIGURATION_COMPLETE: begin
            if (ts2_valid_i[lane]) begin
              single_ts2_received_c ='1;

              // C18 (Base 2.1 4.2.6.3.5.1 p.235; tracker SS54 #11): the eight
              // consecutive TS2 must carry matching non-PAD Link and Lane numbers
              // AND "identical data rate identifiers (including identical Link
              // Upconfigure Capability (Symbol 4 bit 6))".  Only Link and Lane
              // were compared.
              //
              // ⚠️ This compares the WHOLE rate_id_t on purpose, and it is NOT
              // SS54 #1's trap in reverse.  There the spec named one field
              // (.rate) and the code compared the struct; here p.235 names the
              // BYTE and its parenthetical explicitly pulls a second bit in.
              //
              // temp_rate_id holds the identifier of the TS2 that OPENED the
              // current run.  (ts2_cnt == '0) is the run-opener: the global
              // transition reset zeroes ts2_cnt on entry and a mismatch zeroes it
              // again, so "count is 0" is exactly "no run in progress".  Preferred
              // over first_ts1, which carries its own meaning in
              // Linkwidth.Start and RcvrCfg -- this way there is no cross-state
              // coupling at all.
              //
              // Mirrors ST_RECOVERY_RCVR_CFG's existing idiom rather than
              // inventing one; see evidence/fix-arc-6/REDERIVE_SITES_FA6b.md
              // section 4 on why oracle R11 read that site as absent.
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

          // =========================
          ST_CONFIGURATION_IDLE: begin
            //wait for incoming ts1-os...//skip if threshhold already reached
            //using ts1_cnt as idle count
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

  // ======================================================================
  //  Per-lane ordered-set output -- THE SINGLE POINT OF PER-LANE DIVERGENCE
  // ======================================================================
  //  The whole FSM builds ONE 128-bit template (ordered_set_r) per cycle; a
  //  downstream/root port must, however, transmit a DIFFERENT Lane Number on
  //  each lane during Configuration (PCIe Base, Configuration.Lanenum). This
  //  block is the only place the single template fans out per-lane, and the
  //  Lane Number byte is the ONLY field that ever differs between lanes --
  //  Link Number, rate, TS type, everything else is broadcast identically.
  //
  //  Widening the OUTPUT (ordered_set_o -> array) rather than the ~20 build
  //  sites (ordered_set_c) keeps every gen_ts_os/gen_eios/gen_eieos/gen_zeros
  //  call untouched and confines x4 to this stage. See the Step-0 design note.
  //
  //  When per-lane assignment fires (physical lane index l):
  //    * gen_ts1|gen_ts2 : only a TS1/TS2 ordered set carries a Lane Number.
  //      Gating here means idle (gen_idle -> gen_zeros), EIOS and EIEOS are
  //      NEVER touched -- their byte 2 is pattern data, not a lane number.
  //    * template lane_num != PAD : the FSM only puts a non-PAD lane number in
  //      the template once it has decided to assign one. This is what keeps the
  //      two roles correct WITHOUT an IS_ROOT_PORT test here: the RC exit
  //      builds carry train_seq_e'(0) (non-PAD) from Lanenum.Wait on, so the RC
  //      assigns per-lane from Lanenum.Wait; the EP builds carry PAD until its
  //      COMPLETE-feeding build (line ~853), so the EP only diverges per-lane
  //      at Complete (which is all a *spec* upstream port needs -- and note the
  //      repo's EP does not yet advertise lane numbers earlier; that is a
  //      separate EP-side gap).
  //
  //  x1 (MAX_NUM_LANES=1): l is always 0, and every template that reaches this
  //  block with lane_num != PAD already holds 0, so t.lane_num = 0 is a no-op
  //  -- bit-identical to the previous `assign ordered_set_o = ordered_set_r`.
  //
  //  Lane reversal (optional per spec, unimplemented): a reversed link would
  //  map assigned-number -> reversed-physical-lane HERE (t.lane_num = f(l))
  //  and the RX `lane_num == lane` checks would compare against f(l).
  //  TODO(lane-reversal): not implemented; contiguous, non-reversed only.
  //  TODO(contiguity): no fragmentation/contiguity check on the forming lanes
  //  (see the deleted Linkwidth.Accept parity gate); non-contiguous responders
  //  get physical-index lane numbers, not sequential 0..N-1.
  always_comb begin : per_lane_ordered_set_o
    pcie_tsos_t tmpl;
    logic       tx_ts;
    tmpl  = pcie_tsos_t'(ordered_set_r);
    tx_ts = (gen_os_ctrl_r.gen_ts1 || gen_os_ctrl_r.gen_ts2);
    for (int l = 0; l < MAX_NUM_LANES; l++) begin
      pcie_tsos_t t;
      t = tmpl;
      if (IS_ROOT_PORT) begin
        // Root/downstream port ASSIGNS: each active lane gets its physical
        // index once the FSM has put a non-PAD lane number in the template
        // (Lanenum.Wait onward). Sequential 0..N-1 for a contiguous link.
        if (tx_ts && (tmpl.lane_num != train_seq_e'(PAD_))) begin
          t.lane_num = l[7:0];
        end
      end else begin
        // Endpoint/upstream port ECHOES: transmit back the Lane Number the
        // root assigned on this lane (captured in lane_num_echo). PAD until an
        // assignment has been received -- so the EP never advertises a Lane
        // Number before it has one, which is what removes the premature-
        // announcement deadlock. Note: because lane_num_echo == physical index
        // for a non-reversed contiguous link, the emitted value matches what
        // the RC assigned by construction; under lane reversal (unsupported)
        // the captured value would differ and this true echo is required.
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
