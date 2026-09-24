//! @title tb_ltssm_b2b
//! Back-to-back Root-Complex <-> Endpoint LTSSM harness (x1).
//!
//! Two pcie_ltssm_downstream instances -- one IS_ROOT_PORT=1 (LINK_NUM=1),
//! one IS_ROOT_PORT=0 -- cross-wired to each other. No Python drives the
//! protocol: the ordered sets each side transmits are fed to the other side's
//! receive path by the shim below, and the two state machines negotiate
//! Configuration entirely on their own. This is the first check of the RC
//! path against an independently-parameterised EP instance rather than against
//! a Python model of the protocol.
//!
//! WHY A THIN SHIM AND NOT THE REAL phy_transmit/phy_receive DATAPATH
//! -----------------------------------------------------------------
//! In real integration ts1_valid_i/ts2_valid_i/idle_valid_i and the received
//! ordered_set_i are produced by ordered_set_handler (inside phy_receive),
//! and ordered_set_tranmitted_i by os_generator (inside phy_transmit). But
//! ordered_set_handler does not consume a pcie_ordered_set_t -- it consumes a
//! *serialized, scrambled PIPE byte stream* (data_in_i[31:0], data_k_in_i,
//! sync_header, pipe_width) across two user clocks, produced by os_generator +
//! scrambler + async FIFOs in phy_transmit. Using the real ordered_set_handler
//! therefore requires instantiating the entire phy_transmit + phy_receive
//! datapath per side and cross-wiring a bit-exact serial loopback, including
//! scrambler LFSR sync and TX/RX clock-domain crossing. That is ~80% of
//! pcie_phy_top twice; its scrambler-sync and CDC paths are exactly where the
//! git history shows pre-existing datapath bugs ("wierd fail at the end",
//! "infinite gen idle"). A hang there would be ambiguous between a datapath
//! bug and an RC-LTSSM bug, defeating the purpose of this check. So we keep an
//! explicit, minimal shim that replicates ONLY the two decode facts the LTSSM
//! actually depends on, each cited against the real module:
//!
//!   * ts1_valid / ts2_valid: ordered_set_handler.sv checks that ts-id bytes
//!     6..9 of the received ordered set all equal TS1 (resp. TS2)
//!     (see ordered_set_handler.sv ~lines 380-395:
//!      ordered_set_out_r[8*6+:8]==TS1 && ...[8*7]==TS1 && ...[8*8] && ...[8*9]).
//!     gen_ts_os() writes TSOS_ into exactly those bytes (ts_s6..ts_s9), so we
//!     replicate that four-byte compare directly on the peer's ordered_set_o.
//!   * idle_valid: in Configuration.Idle the LTSSM's COMPLETE exit build emits
//!     gen_zeros() (an all-zero ordered set) and holds gen_os_ctrl.gen_idle=1;
//!     ordered_set_handler recognises idle by decoding that zero/IDL byte
//!     stream. An all-zero ordered set is indistinguishable from the reset
//!     value on a struct, so we key idle off the transmitter's own
//!     gen_os_ctrl_o.gen_idle (the very signal that, in real integration,
//!     drives os_generator to emit the idle pattern the receiver then decodes).
//!
//! ordered_set_tranmitted_i is os_generator.os_sent_o, a 1-cycle pulse per
//! complete ordered-set frame streamed onto the PIPE bus (os_generator.sv
//! ~line 229). The minimal faithful model is a periodic 1-cycle beat, which is
//! also exactly what the validated Python os_tx_pulser does. Both sides
//! transmit at the same OS rate, so one shared beat drives both.
//!
//! Inputs tied off match how pcie_phy_top leaves them: is_timeout_i,
//! recovery_i, lanes_ts2_satisfied_i, config_copmlete_ts2_i, from_l0_i,
//! extended_synch_i, lane_status_i are unconnected/unused in the RTL body;
//! directed_speed_change_i and polarity_inverted_i are '0 (no speed change,
//! no polarity inversion in a clean loopback).

module tb_ltssm_b2b
  import pcie_phy_pkg::*;
#(
    parameter int MAX_NUM_LANES  = 1,
    parameter int LINK_NUM       = 1,
    parameter int SIM_FAST_LINK  = 1,
    // Cycles per ordered set -- the os_generator TX-done cadence model.
    parameter int OS_BEAT_PERIOD = 4
) (
    input logic clk_i,
    input logic rst_i,
    input logic en_i,

    // ---- PHY-level bring-up, driven by Python and shared to both PHYs ----
    // (receiver detection + phystatus + elec-idle: the only things Python
    //  touches; it never touches ordered_set_i on either side.)
    input logic [    MAX_NUM_LANES-1:0] phy_rxelecidle_drv_i,
    input logic [    MAX_NUM_LANES-1:0] receiver_detected_drv_i,
    input logic [(MAX_NUM_LANES*3)-1:0] phy_rxstatus_drv_i,
    input logic [    MAX_NUM_LANES-1:0] phy_phystatus_drv_i,

    // ---- observation ----
    output logic        rc_link_up_o,
    output logic        ep_link_up_o,
    output logic [19:0] rc_ltssm_state_o,
    output logic [19:0] ep_ltssm_state_o,
    output logic        os_beat_o
);

  // ---------------------------------------------------------------------
  //  Ordered-set beat: models os_generator.os_sent_o (one pulse per OS).
  // ---------------------------------------------------------------------
  logic [7:0] beat_cnt;
  logic       os_beat;
  always_ff @(posedge clk_i) begin
    if (rst_i) beat_cnt <= '0;
    else       beat_cnt <= (beat_cnt >= OS_BEAT_PERIOD - 1) ? '0 : beat_cnt + 1'b1;
  end
  assign os_beat   = (beat_cnt == OS_BEAT_PERIOD - 1);
  assign os_beat_o = os_beat;

  // ---------------------------------------------------------------------
  //  Per-instance nets
  // ---------------------------------------------------------------------
  // ordered_set_o is now per-lane (the DUT assigns a distinct Lane Number per
  // lane in Configuration); one struct per lane.
  pcie_ordered_set_t [MAX_NUM_LANES-1:0] rc_ordered_set_o;
  pcie_ordered_set_t [MAX_NUM_LANES-1:0] ep_ordered_set_o;
  gen_os_struct_t                        rc_gen_os_ctrl_o;
  gen_os_struct_t                        ep_gen_os_ctrl_o;

  pcie_tsos_t [MAX_NUM_LANES-1:0]        rc_ordered_set_i;
  pcie_tsos_t [MAX_NUM_LANES-1:0]        ep_ordered_set_i;

  logic [MAX_NUM_LANES-1:0]              rc_ts1_valid_i, rc_ts2_valid_i, rc_idle_valid_i;
  logic [MAX_NUM_LANES-1:0]              ep_ts1_valid_i, ep_ts2_valid_i, ep_idle_valid_i;

  // ---------------------------------------------------------------------
  //  Decode shim: replicate ordered_set_handler's ts-id byte-6..9 compare.
  // ---------------------------------------------------------------------
  function automatic logic is_tsos(input pcie_ordered_set_t os, input logic [7:0] tsid);
    return (os.symbols[6] == tsid) && (os.symbols[7] == tsid)
        && (os.symbols[8] == tsid) && (os.symbols[9] == tsid);
  endfunction

  // Receive content: each side's ordered_set_i[l] is the peer's ordered_set_o[l]
  // -- PER-LANE, so lane l receives exactly what the peer transmitted on lane l
  // (crucially, the peer's per-lane Lane Number). Python never drives these.
  always_comb begin
    for (int l = 0; l < MAX_NUM_LANES; l++) begin
      ep_ordered_set_i[l] = pcie_tsos_t'(rc_ordered_set_o[l]);  // EP rx RC's lane l
      rc_ordered_set_i[l] = pcie_tsos_t'(ep_ordered_set_o[l]);  // RC rx EP's lane l
    end
  end

  // Valid strobes: one decoded ordered set per beat, per lane, per direction.
  // TS type (ts-id bytes 6..9) is identical across a peer's lanes -- only the
  // Lane Number byte differs -- but we decode per-lane for faithfulness.
  always_comb begin
    for (int l = 0; l < MAX_NUM_LANES; l++) begin
      ep_ts1_valid_i[l]  = os_beat & is_tsos(rc_ordered_set_o[l], TS1);
      ep_ts2_valid_i[l]  = os_beat & is_tsos(rc_ordered_set_o[l], TS2);
      ep_idle_valid_i[l] = os_beat & rc_gen_os_ctrl_o.gen_idle;
      rc_ts1_valid_i[l]  = os_beat & is_tsos(ep_ordered_set_o[l], TS1);
      rc_ts2_valid_i[l]  = os_beat & is_tsos(ep_ordered_set_o[l], TS2);
      rc_idle_valid_i[l] = os_beat & ep_gen_os_ctrl_o.gen_idle;
    end
  end

  // ---------------------------------------------------------------------
  //  Root-Complex instance (IS_ROOT_PORT=1, LINK_NUM=1)
  // ---------------------------------------------------------------------
  pcie_ltssm_downstream #(
      // 63 #7g-2 commit 1 (D-7G.2): pins the 10 ns this bench has always elaborated
      .CLK_PERIOD_NS(10),
      .MAX_NUM_LANES(MAX_NUM_LANES),
      .SIM_FAST_LINK(SIM_FAST_LINK),
      .IS_ROOT_PORT (1),
      .LINK_NUM     (LINK_NUM)
  ) rc_inst (
      .clk_i                   (clk_i),
      .rst_i                   (rst_i),
      .en_i                    (en_i),
      .link_up_o               (rc_link_up_o),
      .is_timeout_i            ('0),
      .recovery_i              ('0),
      .error_o                 (),
      .success_o               (),
      .error_loopback_o        (),
      .error_disable_o         (),
      .ts1_valid_i             (rc_ts1_valid_i),
      .ts2_valid_i             (rc_ts2_valid_i),
      .idle_valid_i            (rc_idle_valid_i),
      .polarity_inverted_i     ('0),
      .phy_rxstatus_i          (phy_rxstatus_drv_i),
      .phy_phystatus_i         (phy_phystatus_drv_i),
      .phy_phystatus_rst_i     ('0),
      .phy_txdetectrx_o        (),
      .phy_txelecidle_o        (),
      .phy_txdeemph_o          (),
      .phy_powerdown_o         (),
      .phy_txcompliance_o      (),
      .phy_rxpolarity_o        (),
      .phy_txmargin_o          (),
      .lanes_ts2_satisfied_i   ('0),
      .config_copmlete_ts2_i   ('0),
      .from_l0_i               ('0),
      .receiver_detected_i     (receiver_detected_drv_i),
      .phy_rxelecidle_i        (phy_rxelecidle_drv_i),
      .tx_enter_elec_idle_o    (),
      .ltssm_state_o           (rc_ltssm_state_o),
      .goto_cfg_o              (),
      .goto_detect_o           (),
      .ordered_set_tranmitted_i(os_beat),
      .send_ordered_set_o      (),
      .active_lanes_o          (),
      .gen_os_ctrl_o           (rc_gen_os_ctrl_o),
      .ordered_set_i           (rc_ordered_set_i),
      .preset_coeff_o          (),
      .ordered_set_o           (rc_ordered_set_o),
      .extended_synch_i        ('0),
      .directed_speed_change_i ('0),
      .lane_status_i           ('0),
      .curr_data_rate_o        (),
      .data_rate_o             (),
      .changed_speed_recovery_o()
  );

  // ---------------------------------------------------------------------
  //  Endpoint instance (IS_ROOT_PORT=0)
  // ---------------------------------------------------------------------
  pcie_ltssm_downstream #(
      // 63 #7g-2 commit 1 (D-7G.2): pins the 10 ns this bench has always elaborated
      .CLK_PERIOD_NS(10),
      .MAX_NUM_LANES(MAX_NUM_LANES),
      .SIM_FAST_LINK(SIM_FAST_LINK),
      .IS_ROOT_PORT (0),
      .LINK_NUM     (0)
  ) ep_inst (
      .clk_i                   (clk_i),
      .rst_i                   (rst_i),
      .en_i                    (en_i),
      .link_up_o               (ep_link_up_o),
      .is_timeout_i            ('0),
      .recovery_i              ('0),
      .error_o                 (),
      .success_o               (),
      .error_loopback_o        (),
      .error_disable_o         (),
      .ts1_valid_i             (ep_ts1_valid_i),
      .ts2_valid_i             (ep_ts2_valid_i),
      .idle_valid_i            (ep_idle_valid_i),
      .polarity_inverted_i     ('0),
      .phy_rxstatus_i          (phy_rxstatus_drv_i),
      .phy_phystatus_i         (phy_phystatus_drv_i),
      .phy_phystatus_rst_i     ('0),
      .phy_txdetectrx_o        (),
      .phy_txelecidle_o        (),
      .phy_txdeemph_o          (),
      .phy_powerdown_o         (),
      .phy_txcompliance_o      (),
      .phy_rxpolarity_o        (),
      .phy_txmargin_o          (),
      .lanes_ts2_satisfied_i   ('0),
      .config_copmlete_ts2_i   ('0),
      .from_l0_i               ('0),
      .receiver_detected_i     (receiver_detected_drv_i),
      .phy_rxelecidle_i        (phy_rxelecidle_drv_i),
      .tx_enter_elec_idle_o    (),
      .ltssm_state_o           (ep_ltssm_state_o),
      .goto_cfg_o              (),
      .goto_detect_o           (),
      .ordered_set_tranmitted_i(os_beat),
      .send_ordered_set_o      (),
      .active_lanes_o          (),
      .gen_os_ctrl_o           (ep_gen_os_ctrl_o),
      .ordered_set_i           (ep_ordered_set_i),
      .preset_coeff_o          (),
      .ordered_set_o           (ep_ordered_set_o),
      .extended_synch_i        ('0),
      .directed_speed_change_i ('0),
      .lane_status_i           ('0),
      .curr_data_rate_o        (),
      .data_rate_o             (),
      .changed_speed_recovery_o()
  );

endmodule
