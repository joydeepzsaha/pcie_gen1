`timescale 1ns/1ps

// ===========================================================================
// tb_pcie_rc_ep_wrap -- the Root Complex vertical and the Endpoint vertical,
// in one netlist, cross-wired at the Data Link Layer's PHY-facing stream.
//
// THIS IS THE FIRST TIME THE TWO VERTICALS MEET IN SIMULATION.  Every prior
// bench on either side had a Python far end: the RC's enumeration rows ran
// against ConfigDevice/BarSpaceCompleter, and the endpoint's five rows ran
// against a Python link partner.  Here there is NO MODEL ANYWHERE IN THE
// PACKET PATH.  An enumeration request leaves pcie_enum_top, is framed,
// sequenced and LCRC'd by the RC's data link layer, crosses twelve wires, and
// is de-framed, credit-accounted and ANSWERED by the endpoint's data link
// layer -- which is where the endpoint's configuration space actually lives.
//
// SS WHY THE SEAM IS AT THE DLL AND NOT AT THE PHY.
// pcie_phy_top instantiates pcie_datalink_layer (alongside phy_receive,
// phy_transmit and pcie_ltssm_downstream), so stacking a PHY under
// pcie_rc_dl_top would give the RC vertical TWO data link layers.  The
// through-the-fabric-PHY shape is a separate rung and needs the DL tops
// decomposed first.  Recorded in ~/pcie_docs/evidence/rc-ep-bench/
// PHASE0_RECON.md SS1.4.
//
// SS THE SEAM IS EXACT, AND THAT IS A MEASUREMENT, NOT A CONVENIENCE.
// pcie_enum_dl_top and pcie_endpoint_top expose the same twelve PHY-side
// signals under the same names, the same widths and opposed directions --
// tdata 32, tkeep 4, tuser 3.  No shim, no width conversion, no renaming.
//
// !! BUT THE TWO SIDES ARE PARAMETERISED DIFFERENTLY AND AGREE ONLY BY
// COINCIDENCE OF DEFAULTS.  pcie_enum_dl_top declares its PHY stream with
// LITERALS ([31:0], [3:0], [2:0]); pcie_endpoint_top declares its with
// DATA_WIDTH / KEEP_WIDTH / USER_WIDTH.  They line up because the endpoint's
// defaults are 32 / 4 / 3.  An endpoint elaborated at any other DATA_WIDTH
// would mis-connect here and NOTHING IN EITHER src/ FILE SAYS SO, which is
// why this comment is at the instantiation the mistake would be made at.
// (The identical asymmetry exists one directory away in the PHY: phy_transmit
// wires the scrambler with a literal 32 while phy_receive wires the same
// module with DATA_WIDTH -- WIDTH_RECON.md SS2.5.)
//
// SS THE START GATE IS THE BENCH'S, AND IT NEEDS BOTH SIDES.
// pcie_enum_dl_top gates scan_start_i internally on its own fc_init_done_o
// (the shape (iii) rung, tracker SS44), so the RC cannot issue before ITS flow
// control is up.  That is necessary and NOT sufficient here: fc_init_done_o is
// the RC's filtered view and says nothing about whether the ENDPOINT has
// finished its own InitFC exchange.  Every previous bench got this for free
// because a Python far end is ready by construction.  The cocotb side
// therefore waits on ep_fc_initialized_o as well.
//
// SS NAMING RULE.  RC-side nets keep the names tb_pcie_enum_dl_top.sv uses, so
// the enumeration helpers (Mon, wait_frames, assert_golden_on_the_wire) and
// the start-gate helpers transfer unchanged.  EVERY endpoint-side net is
// prefixed ep_.  The only unprefixed shared nets are the five driven to both
// DUTs on purpose: clk_i, rst_i, phy_link_up_i, idle_valid_i,
// transmit_enable_i -- one clock, one reset, no clock-domain crossing.
//
// SS THE CROSSED NETS ARE THE RC'S OWN PORT NETS.  No intermediate wires are
// declared: the endpoint's s_phy_axis_* inputs are connected directly to the
// nets the RC's m_phy_axis_* outputs drive, and vice versa.  Twelve crossings,
// zero extra declarations, and the direction opposition is visible at the two
// instantiation sites rather than asserted in a comment.
// ===========================================================================
module tb_pcie_rc_ep_wrap;
  import tlp_pkg::*;
  import pcie_rq_rc_pkg::*;
  import pcie_enum_pkg::*;

  // ---- RC-side parameters, mirroring tb_pcie_enum_dl_top.sv ---------------
  localparam int AXIS_DATA_WIDTH = 128;
  localparam int AXIS_KEEP_WIDTH = AXIS_DATA_WIDTH / 32;
  localparam int AXIS_USER_WIDTH = 60;
  localparam int unsigned CPL_TIMEOUT_CYCLES = 32'd4096;
  localparam int TAG_COUNT = 32;
  localparam int unsigned CRS_RETRY_MAX      = 3;
  localparam int unsigned CRS_BACKOFF_CYCLES = 8;
  localparam int CQ_USER_WIDTH = 88;
  localparam int CC_USER_WIDTH = 33;

  // ---- EP-side parameters -------------------------------------------------
  // !! LEFT AT pcie_endpoint_top's OWN DEFAULTS ON PURPOSE.  tb_pcie_endpoint_
  // top.sv overrides BAR_COUNT to 1 and REPLAY_TIMER_CYCLES to 64 to make its
  // own replay row cheap.  This bench is asking what a REAL Root Complex sees
  // when it enumerates this endpoint, so the endpoint must be the one the
  // module header describes, not one shaped for a previous test:
  //   BAR_COUNT           2
  //   BAR_MASK            {64'd0, 64'hffff_ffff_ffff_f000}   -> BAR0 = 4 KB
  //   BAR_ENABLE          {1'b0, 1'b1}                       -> BAR0 only
  //   REPLAY_TIMER_CYCLES 16'h0aa0 = 2720
  //   INTEGRATED_GEN1_PHY 1'b0  -- no PHY, no LTSSM, no second DLL
  // The BAR numbers above are what tlp_layer's decoder CLAIMS.  They are not
  // what the endpoint's configuration space REPORTS, and the gap between the
  // two is a row (PHASE0_RECON.md SS2.4).
  localparam int EP_DATA_WIDTH    = 32;
  localparam int EP_KEEP_WIDTH    = 4;
  localparam int EP_USER_WIDTH    = 3;
  localparam int EP_CONTEXT_WIDTH = 16;
  localparam int EP_BAR_COUNT     = 2;
  localparam int EP_MAX_NUM_LANES = 1;
  localparam int TLP_HEADER_WIDTH = $bits(tlp_header_t);

  // =========================================================================
  // Shared stimulus -- driven to BOTH tops.
  // =========================================================================
  logic clk_i;
  logic rst_i;
  logic phy_link_up_i;
  logic idle_valid_i;
  logic transmit_enable_i;

  // =========================================================================
  // The seam.  Declared as wire because each is driven by a module OUTPUT on
  // one side and read as a module INPUT on the other -- there is no procedural
  // driver anywhere in this file, and a `logic` here would invite one.
  // =========================================================================
  // RC -> EP (RC drives tdata/tkeep/tvalid/tlast/tuser; EP drives tready)
  wire [31:0] m_phy_axis_tdata;
  wire [3:0]  m_phy_axis_tkeep;
  wire        m_phy_axis_tvalid;
  wire        m_phy_axis_tlast;
  wire [2:0]  m_phy_axis_tuser;
  wire        m_phy_axis_tready;

  // =========================================================================
  // THE TIE-BREAK INJECTOR -- a two-way mux on the ENDPOINT's receive stream.
  // =========================================================================
  // !! THIS EXISTS ONLY BECAUSE OF A MEASURED DEFECT, AND IT IS THE NARROWEST
  // THING THAT WORKS AROUND IT.  pcie_flow_ctrl_init never originates an
  // InitFC1 -- it leaves ST_IDLE only on fc1_values_stored_i or
  // first_feature_exchange_dllp_received_i, both of which are outputs of
  // dllp_receive and therefore set only by RECEIVING.  Two of them on one link
  // deadlock; rcep_fc_init_completes_unaided is the row that says so, and it
  // runs with inj_sel LOW so it still sees the real behaviour.
  //
  // !! WHAT THIS IS NOT.  It is NOT a far-end model and it must never become
  // one.  It speaks exactly once, before flow control exists, to break a
  // symmetry the RTL cannot break itself.  The instant inj_sel falls the
  // endpoint's receive stream is wired to the Root Complex's transmit stream
  // and every subsequent packet -- every CfgRd0, every Completion, every
  // UpdateFC -- crosses RTL to RTL.  The enumeration rows assert inj_sel is
  // low across their whole measurement window and count the beats that crossed,
  // so a regression that left the injector enabled could not read as a pass.
  //
  // While inj_sel is high the Root Complex's transmitter is held off with
  // tready low rather than being allowed to interleave.  That is not a
  // throttle: at that moment the RC is itself parked in ST_IDLE and has nothing
  // to send, and holding it off keeps the injected DLLP the only thing on the
  // endpoint's receive stream, so the tie-break cannot race real traffic.
  logic        inj_sel;
  logic [31:0] inj_tdata;
  logic [3:0]  inj_tkeep;
  logic        inj_tvalid;
  logic        inj_tlast;
  logic [2:0]  inj_tuser;
  wire         inj_tready;

  wire [31:0] ep_rx_tdata  = inj_sel ? inj_tdata  : m_phy_axis_tdata;
  wire [3:0]  ep_rx_tkeep  = inj_sel ? inj_tkeep  : m_phy_axis_tkeep;
  wire        ep_rx_tvalid = inj_sel ? inj_tvalid : m_phy_axis_tvalid;
  wire        ep_rx_tlast  = inj_sel ? inj_tlast  : m_phy_axis_tlast;
  wire [2:0]  ep_rx_tuser  = inj_sel ? inj_tuser  : m_phy_axis_tuser;
  wire        ep_rx_tready;

  assign inj_tready        = inj_sel ? ep_rx_tready : 1'b0;
  assign m_phy_axis_tready = inj_sel ? 1'b0 : ep_rx_tready;
  // EP -> RC (EP drives tdata/tkeep/tvalid/tlast/tuser; RC drives tready)
  wire [31:0] s_phy_axis_tdata;
  wire [3:0]  s_phy_axis_tkeep;
  wire        s_phy_axis_tvalid;
  wire        s_phy_axis_tlast;
  wire [2:0]  s_phy_axis_tuser;
  wire        s_phy_axis_tready;

  // =========================================================================
  // RC-side surface.  Names and flattening reproduced from
  // tb_pcie_enum_dl_top.sv so the existing helpers transfer unchanged.
  // =========================================================================
  logic [15:0] requester_id_i;
  logic [15:0] completer_id_i;
  logic [7:0]  bus_number_i;
  logic [4:0]  device_number_i;
  logic [2:0]  function_number_i;
  logic        memory_enable_i;
  logic        extended_tag_enable_i;
  logic [12:0] max_payload_bytes_i;
  logic [12:0] max_read_bytes_i;
  logic        rcb_128b_i;

  logic [7:0] cfg_bus_number_o;
  logic [4:0] cfg_device_number_o;
  logic [2:0] cfg_function_number_o;

  logic fc_init_done_o;
  logic ok_to_issue_o;

  logic       scan_start_i;
  logic [7:0] scan_bus_i;
  logic       bar_enable_i;
  logic       bridge_enable_i;

  logic        scan_busy_o;
  logic        scan_done_o;
  logic        scan_error_o;
  enum_error_e scan_error_code;
  logic [3:0]  scan_error_code_o;
  assign scan_error_code_o = 4'(scan_error_code);
  logic        err_credit_blocked_o;

  logic        device_present_o;
  logic        unsupported_device_o;
  logic [15:0] device_bdf_o;
  logic [15:0] vendor_id_o;
  logic [15:0] device_id_o;
  logic [7:0]  header_type_o;
  logic        multifunction_o;

  logic        bar_busy_o;
  logic        enum_done_o;
  logic        enum_error_o;
  enum_error_e enum_error_code;
  logic [3:0]  enum_error_code_o;
  assign enum_error_code_o = 4'(enum_error_code);

  logic [3:0]              bar_count_o;
  logic [BAR_SLOTS-1:0]    bar_valid_o;
  logic [BAR_SLOTS-1:0]    bar_is_64_o;
  logic [BAR_SLOTS-1:0]    bar_prefetch_o;
  logic [BAR_SLOTS*64-1:0] bar_size_o;
  logic [BAR_SLOTS*64-1:0] bar_addr_o;
  logic [BAR_SLOTS-1:0]    io_bar_mask_o;

  logic                    bus_done_o;
  logic                    bus_bypassed_o;
  logic                    sec_scan_done_o;
  logic                    sec_device_present_o;
  logic                    sec_unsupported_device_o;
  logic [15:0]             sec_device_bdf_o;
  logic [15:0]             sec_vendor_id_o;
  logic [15:0]             sec_device_id_o;
  logic [7:0]              sec_header_type_o;
  logic                    sec_multifunction_o;
  logic                    sec_enum_done_o;
  logic [3:0]              sec_bar_count_o;
  logic [BAR_SLOTS-1:0]    sec_bar_valid_o;
  logic [BAR_SLOTS-1:0]    sec_bar_is_64_o;
  logic [BAR_SLOTS-1:0]    sec_bar_prefetch_o;
  logic [BAR_SLOTS*64-1:0] sec_bar_size_o;
  logic [BAR_SLOTS*64-1:0] sec_bar_addr_o;
  logic [BAR_SLOTS-1:0]    sec_io_bar_mask_o;

  // The RC's completer surface.  Stage F-3 carried this to the top of the
  // enum stack; before that it was tied off inside pcie_rq_rc_top.  It is the
  // row-4 observation point: an endpoint-originated MemWr lands here.
  logic [AXIS_DATA_WIDTH-1:0] m_axis_cq_tdata;
  logic [AXIS_KEEP_WIDTH-1:0] m_axis_cq_tkeep;
  logic                       m_axis_cq_tvalid;
  logic                       m_axis_cq_tlast;
  logic [CQ_USER_WIDTH-1:0]   m_axis_cq_tuser;
  logic                       m_axis_cq_tready;

  logic [AXIS_DATA_WIDTH-1:0] s_axis_cc_tdata;
  logic [AXIS_KEEP_WIDTH-1:0] s_axis_cc_tkeep;
  logic                       s_axis_cc_tvalid;
  logic                       s_axis_cc_tlast;
  logic [CC_USER_WIDTH-1:0]   s_axis_cc_tuser;
  logic                       s_axis_cc_tready;

  logic       cq_dropped_o;
  logic [3:0] cq_error_code_o;
  logic       cq_gearbox_error_o;
  logic       cc_protocol_error_o;
  logic [3:0] cc_error_code_o;
  logic       cc_gearbox_error_o;

  logic       rq_protocol_error_o;
  rq_error_e  rq_error_code;
  logic [3:0] rq_error_code_o;
  assign rq_error_code_o = 4'(rq_error_code);
  logic       rq_gearbox_error_o;

  logic       rc_unexpected_completion_o;
  tlp_error_e rc_completion_error_code;
  logic [4:0] rc_completion_error_code_o;
  assign rc_completion_error_code_o = 5'(rc_completion_error_code);

  logic       rc_protocol_error_o;
  rc_error_e  rc_error_code;
  logic [3:0] rc_error_code_o;
  assign rc_error_code_o = 4'(rc_error_code);
  logic       rc_gearbox_error_o;

  logic       command_error_valid_o;
  tlp_error_e command_error_code;
  logic [4:0] command_error_code_o;
  assign command_error_code_o = 5'(command_error_code);

  logic       malformed_o;
  logic       rx_error_valid_o;
  tlp_error_e rx_error_code;
  logic [4:0] rx_error_code_o;
  assign rx_error_code_o = 5'(rx_error_code);
  logic       rx_ecrc_error_o;

  logic       tx_error_valid_o;
  tlp_error_e tx_error_code;
  logic [4:0] tx_error_code_o;
  assign tx_error_code_o = 5'(tx_error_code);

  logic       tx_fc_blocked_o;
  logic       credit_error_o;
  logic       vc_overflow_o;
  logic       cpl_timeout_valid_o;
  logic [7:0] cpl_timeout_tag_o;
  logic       late_cpl_valid_o;
  logic [7:0] late_cpl_tag_o;
  logic [$clog2(TAG_COUNT+1)-1:0] outstanding_o;

  // RC verification-only visibility, reproduced from tb_pcie_enum_dl_top.sv.
  // fc_initialized_o is the FILTER output (an alias of the real port), kept
  // under this name because the shared helper reads it by that name.
  // fc_initialized_dll is the RC DLL's raw, glitching output and stays a reach.
  wire        fc_initialized_dll = u_rc.u_rcdl.dl_fc_initialized;
  wire        fc_initialized_o   = fc_init_done_o;
  wire        fc_update_valid_o  = u_rc.u_rcdl.dl_fc_update_valid;
  wire [7:0]  fc_ph_o   = u_rc.u_rcdl.dl_fc_ph;
  wire [11:0] fc_pd_o   = u_rc.u_rcdl.dl_fc_pd;
  wire [7:0]  fc_nph_o  = u_rc.u_rcdl.dl_fc_nph;
  wire [11:0] fc_npd_o  = u_rc.u_rcdl.dl_fc_npd;
  wire [7:0]  fc_cplh_o = u_rc.u_rcdl.dl_fc_cplh;
  wire [11:0] fc_cpld_o = u_rc.u_rcdl.dl_fc_cpld;

  // The PG213 seam, internal to pcie_enum_dl_top by design; aliased so
  // enum_tb_common's Mon transfers unchanged.  Observation only.
  wire [7:0] pcie_rq_tag_o     = u_rc.rq_tag;
  wire       pcie_rq_tag_vld_o = u_rc.rq_tag_vld;
  wire       s_axis_rq_tvalid  = u_rc.rq_tvalid;

  // =========================================================================
  // The flow-control initialisation FSM, BOTH SIDES.
  // =========================================================================
  // These exist because "FC init did not complete" is not a finding -- WHICH
  // STATE EACH SIDE IS PARKED IN is.  pcie_flow_ctrl_init leaves ST_IDLE only
  // when fc1_values_stored_i or first_feature_exchange_dllp_received_i is
  // asserted, and BOTH of those are outputs of dllp_receive, i.e. both are set
  // only by RECEIVING a DLLP from the peer.  With two of these facing each
  // other neither can go first.  A row that only reported "no FC init" would
  // leave the reader to guess between a deadlock, a stuck ready and a reset
  // problem; these two states distinguish them in the log.
  //
  // Cast to a vector: cocotb cannot read a port or signal whose type is a
  // SystemVerilog enum (flow_control_state_e is `enum logic [4:0]`), the same
  // limitation that forces the enum flattening on the RC surface above.
  wire [4:0] rc_fc_state = 5'(u_rc.u_rcdl.u_dl.pcie_flow_ctrl_init_inst.curr_state);
  wire [4:0] ep_fc_state = 5'(u_ep.datalink_layer_inst.pcie_flow_ctrl_init_inst.curr_state);
  wire       rc_start_fc = u_rc.u_rcdl.u_dl.init_flow_control;
  wire       ep_start_fc = u_ep.datalink_layer_inst.init_flow_control;

  // =========================================================================
  // EP-side surface.  Declarations mirror tb_pcie_endpoint_top.sv (a shape
  // already proven to elaborate under Verilator), every net prefixed ep_.
  // =========================================================================
  logic ep_memory_enable_i;
  logic ep_extended_tag_enable_i;
  logic [12:0] ep_max_payload_bytes_i;
  logic [12:0] ep_max_read_bytes_i;
  logic ep_rcb_128b_i;

  logic ep_command_valid_i;
  logic ep_command_ready_o;
  tlp_cmd_e ep_command_i;
  logic [63:0] ep_command_address_i;
  logic [12:0] ep_command_byte_count_i;
  logic [2:0] ep_command_tc_i;
  logic [2:0] ep_command_attr_i;
  logic [EP_CONTEXT_WIDTH-1:0] ep_command_context_i;
  logic ep_command_prefix_valid_i;
  logic [31:0] ep_command_prefix_i;
  logic ep_command_ecrc_enable_i;
  logic [EP_DATA_WIDTH-1:0] ep_command_data_i;
  logic [EP_KEEP_WIDTH-1:0] ep_command_keep_i;
  logic ep_command_data_valid_i;
  logic ep_command_data_last_i;
  logic ep_command_data_ready_o;
  logic ep_command_error_valid_o;
  tlp_error_e ep_command_error_code_o;

  logic ep_target_request_valid_o;
  logic ep_target_request_ready_i;
  tlp_class_e ep_target_request_class_o;
  logic ep_target_memory_o;
  logic ep_target_config_o;
  logic ep_target_config_hit_o;
  logic ep_target_config_type_one_o;
  logic [11:0] ep_target_config_offset_o;
  logic ep_target_read_o;
  logic ep_target_write_o;
  logic ep_target_unsupported_o;
  logic ep_target_bar_hit_o;
  logic ep_target_bar_overlap_o;
  logic [((EP_BAR_COUNT <= 1) ? 1 : $clog2(EP_BAR_COUNT))-1:0] ep_target_bar_o;
  logic [63:0] ep_target_offset_o;
  logic [EP_DATA_WIDTH-1:0] ep_target_data_o;
  logic [EP_KEEP_WIDTH-1:0] ep_target_keep_o;
  logic ep_target_data_valid_o;
  logic ep_target_data_last_o;
  logic ep_target_data_ready_i;

  logic ep_completion_request_valid_i;
  logic ep_completion_request_ready_o;
  logic [2:0] ep_completion_request_status_i;
  logic [12:0] ep_completion_request_byte_count_i;
  logic [6:0] ep_completion_request_lower_address_i;
  logic ep_completion_request_ecrc_enable_i;
  logic [EP_DATA_WIDTH-1:0] ep_completion_request_data_i;
  logic [EP_KEEP_WIDTH-1:0] ep_completion_request_keep_i;
  logic ep_completion_request_data_valid_i;
  logic ep_completion_request_data_last_i;
  logic ep_completion_request_data_ready_o;

  logic ep_received_completion_valid_o;
  logic ep_received_completion_ready_i;
  logic [EP_DATA_WIDTH-1:0] ep_received_completion_data_o;
  logic [EP_KEEP_WIDTH-1:0] ep_received_completion_keep_o;
  logic ep_received_completion_data_valid_o;
  logic ep_received_completion_data_last_o;
  logic ep_received_completion_data_ready_i;

  logic ep_result_valid_o;
  logic ep_result_ready_i;
  logic [EP_CONTEXT_WIDTH-1:0] ep_result_context_o;
  logic [2:0] ep_result_status_o;
  logic ep_result_last_o;

  logic [7:0] ep_cfg_bus_number_o;
  logic [4:0] ep_cfg_device_number_o;
  logic [2:0] ep_cfg_function_number_o;

  // !! HAZARD ROW A LIVES ON THIS SIGNAL.  This is the endpoint's RAW
  // fc_initialized_o.  The RC filters its equivalent through fc_init_sticky_r
  // inside pcie_rc_dl_top (tracker SS41.2); the ENDPOINT HAS NO SUCH FILTER, so
  // the 1->0->1 glitch of tracker SS36.2 -- pcie_flow_ctrl_init dropping
  // fc2_values_sent_o across the four ST_UPDATE_* states -- is visible here
  // exactly as the specification forbids.  Base 2.1 SS3.2.1 pp.158-159.
  logic ep_fc_initialized_o;
  logic ep_fc_update_valid_o;
  logic [7:0] ep_fc_ph_o;
  logic [11:0] ep_fc_pd_o;
  logic [7:0] ep_fc_nph_o;
  logic [11:0] ep_fc_npd_o;
  logic [7:0] ep_fc_cplh_o;
  logic [11:0] ep_fc_cpld_o;

  logic ep_malformed_o;
  logic ep_rx_error_valid_o;
  tlp_error_e ep_rx_error_code_o;
  logic ep_rx_ecrc_error_o;
  logic ep_tx_error_valid_o;
  tlp_error_e ep_tx_error_code_o;
  logic ep_tx_fc_blocked_o;
  logic ep_credit_error_o;
  logic ep_vc_overflow_o;
  logic ep_unexpected_completion_o;
  tlp_error_e ep_completion_error_code_o;
  logic ep_cpl_timeout_valid_o;
  logic [7:0] ep_cpl_timeout_tag_o;
  logic ep_late_cpl_valid_o;
  logic [7:0] ep_late_cpl_tag_o;
  logic [$clog2(32+1)-1:0] ep_outstanding_o;

  // Packed structs stay at the DUT boundary; flat vectors face cocotb.
  tlp_header_t ep_completion_request_header_s;
  tlp_header_t ep_target_request_header_s;
  tlp_header_t ep_received_completion_header_s;

  logic [TLP_HEADER_WIDTH-1:0] ep_completion_request_header_i;
  wire  [TLP_HEADER_WIDTH-1:0] ep_target_request_header_o;
  wire  [TLP_HEADER_WIDTH-1:0] ep_received_completion_header_o;

  assign ep_completion_request_header_s =
      tlp_header_t'(ep_completion_request_header_i);
  assign ep_target_request_header_o = ep_target_request_header_s;
  assign ep_received_completion_header_o = ep_received_completion_header_s;

  // The integrated-PHY boundary.  INTEGRATED_GEN1_PHY defaults to 1'b0, so the
  // generate arm that reads these does not elaborate; the arm that DOES
  // elaborate (gen_packet_phy_compatibility) reads none of the inputs and
  // drives all the outputs to constants.  Declared because the ports exist.
  wire                             ep_pipe_rx_usr_clk_i = '0;
  wire                             ep_pipe_tx_usr_clk_i = '0;
  wire [(EP_MAX_NUM_LANES*20)-1:0] ep_phy_rx_symbol_i = '0;
  wire [EP_MAX_NUM_LANES-1:0]      ep_phy_rx_symbol_valid_i = '0;
  wire [EP_MAX_NUM_LANES-1:0]      ep_phy_phystatus_i = '0;
  wire                             ep_phy_phystatus_rst_i = '0;
  wire [EP_MAX_NUM_LANES-1:0]      ep_phy_rxelecidle_i = '0;
  wire [(EP_MAX_NUM_LANES*3)-1:0]  ep_phy_rxstatus_i = '0;

  wire [(EP_MAX_NUM_LANES*20)-1:0] ep_phy_tx_symbol_o;
  wire [EP_MAX_NUM_LANES-1:0]      ep_phy_tx_symbol_valid_o;
  wire                             ep_phy_txdetectrx_o;
  wire [EP_MAX_NUM_LANES-1:0]      ep_phy_txelecidle_o;
  wire [EP_MAX_NUM_LANES-1:0]      ep_phy_txcompliance_o;
  wire [EP_MAX_NUM_LANES-1:0]      ep_phy_rxpolarity_o;
  wire [1:0]                       ep_phy_powerdown_o;
  wire [2:0]                       ep_phy_rate_o;
  wire [2:0]                       ep_phy_txmargin_o;
  wire                             ep_phy_txswing_o;
  wire                             ep_phy_txdeemph_o;
  wire [5:0]                       ep_phy_pipe_width_o;
  wire                             ep_phy_link_up_o;
  wire [19:0]                      ep_ltssm_state_o;
  wire [EP_MAX_NUM_LANES-1:0]      ep_phy_rx_code_error_o;
  wire [EP_MAX_NUM_LANES-1:0]      ep_phy_rx_disparity_error_o;
  wire [EP_MAX_NUM_LANES-1:0]      ep_phy_tx_illegal_k_o;

  // =========================================================================
  // The Root Complex vertical: enumeration engine + TL + DLL.
  // =========================================================================
  pcie_enum_dl_top #(
      .AXIS_DATA_WIDTH   (AXIS_DATA_WIDTH),
      .AXIS_USER_WIDTH   (AXIS_USER_WIDTH),
      .CPL_TIMEOUT_CYCLES(CPL_TIMEOUT_CYCLES),
      .TAG_COUNT         (TAG_COUNT),
      .CRS_RETRY_MAX     (CRS_RETRY_MAX),
      .CRS_BACKOFF_CYCLES(CRS_BACKOFF_CYCLES)
  ) u_rc (
      .clk_i(clk_i),
      .rst_i(rst_i),

      .phy_link_up_i    (phy_link_up_i),
      .idle_valid_i     (idle_valid_i),
      .transmit_enable_i(transmit_enable_i),

      // ---- the seam, RC side ----------------------------------------------
      .s_phy_axis_tdata (s_phy_axis_tdata),
      .s_phy_axis_tkeep (s_phy_axis_tkeep),
      .s_phy_axis_tvalid(s_phy_axis_tvalid),
      .s_phy_axis_tlast (s_phy_axis_tlast),
      .s_phy_axis_tuser (s_phy_axis_tuser),
      .s_phy_axis_tready(s_phy_axis_tready),
      .m_phy_axis_tdata (m_phy_axis_tdata),
      .m_phy_axis_tkeep (m_phy_axis_tkeep),
      .m_phy_axis_tvalid(m_phy_axis_tvalid),
      .m_phy_axis_tlast (m_phy_axis_tlast),
      .m_phy_axis_tuser (m_phy_axis_tuser),
      .m_phy_axis_tready(m_phy_axis_tready),

      .requester_id_i       (requester_id_i),
      .completer_id_i       (completer_id_i),
      .bus_number_i         (bus_number_i),
      .device_number_i      (device_number_i),
      .function_number_i    (function_number_i),
      .memory_enable_i      (memory_enable_i),
      .extended_tag_enable_i(extended_tag_enable_i),
      .max_payload_bytes_i  (max_payload_bytes_i),
      .max_read_bytes_i     (max_read_bytes_i),
      .rcb_128b_i           (rcb_128b_i),

      .cfg_bus_number_o     (cfg_bus_number_o),
      .cfg_device_number_o  (cfg_device_number_o),
      .cfg_function_number_o(cfg_function_number_o),

      .fc_init_done_o(fc_init_done_o),
      .ok_to_issue_o (ok_to_issue_o),

      .scan_start_i   (scan_start_i),
      .scan_bus_i     (scan_bus_i),
      .bar_enable_i   (bar_enable_i),
      .bridge_enable_i(bridge_enable_i),

      .scan_busy_o         (scan_busy_o),
      .scan_done_o         (scan_done_o),
      .scan_error_o        (scan_error_o),
      .scan_error_code_o   (scan_error_code),
      .err_credit_blocked_o(err_credit_blocked_o),
      .device_present_o    (device_present_o),
      .unsupported_device_o(unsupported_device_o),
      .device_bdf_o        (device_bdf_o),
      .vendor_id_o         (vendor_id_o),
      .device_id_o         (device_id_o),
      .header_type_o       (header_type_o),
      .multifunction_o     (multifunction_o),

      .bar_busy_o       (bar_busy_o),
      .enum_done_o      (enum_done_o),
      .enum_error_o     (enum_error_o),
      .enum_error_code_o(enum_error_code),
      .bar_count_o      (bar_count_o),
      .bar_valid_o      (bar_valid_o),
      .bar_is_64_o      (bar_is_64_o),
      .bar_prefetch_o   (bar_prefetch_o),
      .bar_size_o       (bar_size_o),
      .bar_addr_o       (bar_addr_o),
      .io_bar_mask_o    (io_bar_mask_o),

      .bus_done_o              (bus_done_o),
      .bus_bypassed_o          (bus_bypassed_o),
      .sec_scan_done_o         (sec_scan_done_o),
      .sec_device_present_o    (sec_device_present_o),
      .sec_unsupported_device_o(sec_unsupported_device_o),
      .sec_device_bdf_o        (sec_device_bdf_o),
      .sec_vendor_id_o         (sec_vendor_id_o),
      .sec_device_id_o         (sec_device_id_o),
      .sec_header_type_o       (sec_header_type_o),
      .sec_multifunction_o     (sec_multifunction_o),
      .sec_enum_done_o         (sec_enum_done_o),
      .sec_bar_count_o         (sec_bar_count_o),
      .sec_bar_valid_o         (sec_bar_valid_o),
      .sec_bar_is_64_o         (sec_bar_is_64_o),
      .sec_bar_prefetch_o      (sec_bar_prefetch_o),
      .sec_bar_size_o          (sec_bar_size_o),
      .sec_bar_addr_o          (sec_bar_addr_o),
      .sec_io_bar_mask_o       (sec_io_bar_mask_o),

      .m_axis_cq_tdata (m_axis_cq_tdata),  .m_axis_cq_tkeep (m_axis_cq_tkeep),
      .m_axis_cq_tvalid(m_axis_cq_tvalid), .m_axis_cq_tlast (m_axis_cq_tlast),
      .m_axis_cq_tuser (m_axis_cq_tuser),  .m_axis_cq_tready(m_axis_cq_tready),
      .s_axis_cc_tdata (s_axis_cc_tdata),  .s_axis_cc_tkeep (s_axis_cc_tkeep),
      .s_axis_cc_tvalid(s_axis_cc_tvalid), .s_axis_cc_tlast (s_axis_cc_tlast),
      .s_axis_cc_tuser (s_axis_cc_tuser),  .s_axis_cc_tready(s_axis_cc_tready),
      .cq_dropped_o    (cq_dropped_o),     .cq_error_code_o (cq_error_code_o),
      .cq_gearbox_error_o(cq_gearbox_error_o),
      .cc_protocol_error_o(cc_protocol_error_o),
      .cc_error_code_o (cc_error_code_o),
      .cc_gearbox_error_o(cc_gearbox_error_o),

      .rq_protocol_error_o       (rq_protocol_error_o),
      .rq_error_code_o           (rq_error_code),
      .rq_gearbox_error_o        (rq_gearbox_error_o),
      .rc_unexpected_completion_o(rc_unexpected_completion_o),
      .rc_completion_error_code_o(rc_completion_error_code),
      .rc_protocol_error_o       (rc_protocol_error_o),
      .rc_error_code_o           (rc_error_code),
      .rc_gearbox_error_o        (rc_gearbox_error_o),
      .command_error_valid_o     (command_error_valid_o),
      .command_error_code_o      (command_error_code),
      .malformed_o               (malformed_o),
      .rx_error_valid_o          (rx_error_valid_o),
      .rx_error_code_o           (rx_error_code),
      .rx_ecrc_error_o           (rx_ecrc_error_o),
      .tx_error_valid_o          (tx_error_valid_o),
      .tx_error_code_o           (tx_error_code),
      .tx_fc_blocked_o           (tx_fc_blocked_o),
      .credit_error_o            (credit_error_o),
      .vc_overflow_o             (vc_overflow_o),
      .cpl_timeout_valid_o       (cpl_timeout_valid_o),
      .cpl_timeout_tag_o         (cpl_timeout_tag_o),
      .late_cpl_valid_o          (late_cpl_valid_o),
      .late_cpl_tag_o            (late_cpl_tag_o),
      .outstanding_o             (outstanding_o)
  );

  // =========================================================================
  // The Endpoint vertical: TL + DLL, and -- the finding that shaped the row
  // list -- THE CONFIGURATION SPACE, which lives inside the DATA LINK LAYER
  // (dllp_receive instantiates pcie_cfg_wrapper).  A CfgRd0 arriving from the
  // wire is answered by the DLL and never reaches this endpoint's Transaction
  // Layer at all.  PHASE0_RECON.md SS2.1.
  //
  // !! THE SEAM IS CROSSED HERE.  s_phy_axis_* below binds to the nets the RC
  // DRIVES from m_phy_axis_*, and m_phy_axis_* below binds to the nets the RC
  // READS as s_phy_axis_*.  TX to RX, both ways.
  // =========================================================================
  pcie_endpoint_top #(
      .DATA_WIDTH   (EP_DATA_WIDTH),
      .KEEP_WIDTH   (EP_KEEP_WIDTH),
      .USER_WIDTH   (EP_USER_WIDTH),
      .CONTEXT_WIDTH(EP_CONTEXT_WIDTH),
      .MAX_NUM_LANES(EP_MAX_NUM_LANES)
  ) u_ep (
      .clk_i(clk_i),
      .rst_i(rst_i),

      .phy_link_up_i    (phy_link_up_i),
      .idle_valid_i     (idle_valid_i),
      .transmit_enable_i(transmit_enable_i),

      // ---- the seam, EP side: CROSSED ---------------------------------------
      // ep_rx_* is the RC's transmit stream whenever inj_sel is low, which is
      // every cycle of every measurement window in this bench.
      .s_phy_axis_tdata (ep_rx_tdata),
      .s_phy_axis_tkeep (ep_rx_tkeep),
      .s_phy_axis_tvalid(ep_rx_tvalid),
      .s_phy_axis_tlast (ep_rx_tlast),
      .s_phy_axis_tuser (ep_rx_tuser),
      .s_phy_axis_tready(ep_rx_tready),
      .m_phy_axis_tdata (s_phy_axis_tdata),
      .m_phy_axis_tkeep (s_phy_axis_tkeep),
      .m_phy_axis_tvalid(s_phy_axis_tvalid),
      .m_phy_axis_tlast (s_phy_axis_tlast),
      .m_phy_axis_tuser (s_phy_axis_tuser),
      .m_phy_axis_tready(s_phy_axis_tready),

      .pipe_rx_usr_clk_i    (ep_pipe_rx_usr_clk_i),
      .pipe_tx_usr_clk_i    (ep_pipe_tx_usr_clk_i),
      .phy_rx_symbol_i      (ep_phy_rx_symbol_i),
      .phy_rx_symbol_valid_i(ep_phy_rx_symbol_valid_i),
      .phy_tx_symbol_o      (ep_phy_tx_symbol_o),
      .phy_tx_symbol_valid_o(ep_phy_tx_symbol_valid_o),
      .phy_phystatus_i      (ep_phy_phystatus_i),
      .phy_phystatus_rst_i  (ep_phy_phystatus_rst_i),
      .phy_rxelecidle_i     (ep_phy_rxelecidle_i),
      .phy_rxstatus_i       (ep_phy_rxstatus_i),
      .phy_txdetectrx_o     (ep_phy_txdetectrx_o),
      .phy_txelecidle_o     (ep_phy_txelecidle_o),
      .phy_txcompliance_o   (ep_phy_txcompliance_o),
      .phy_rxpolarity_o     (ep_phy_rxpolarity_o),
      .phy_powerdown_o      (ep_phy_powerdown_o),
      .phy_rate_o           (ep_phy_rate_o),
      .phy_txmargin_o       (ep_phy_txmargin_o),
      .phy_txswing_o        (ep_phy_txswing_o),
      .phy_txdeemph_o       (ep_phy_txdeemph_o),
      .phy_pipe_width_o     (ep_phy_pipe_width_o),
      .phy_link_up_o        (ep_phy_link_up_o),
      .ltssm_state_o        (ep_ltssm_state_o),
      .phy_rx_code_error_o  (ep_phy_rx_code_error_o),
      .phy_rx_disparity_error_o(ep_phy_rx_disparity_error_o),
      .phy_tx_illegal_k_o   (ep_phy_tx_illegal_k_o),

      .memory_enable_i      (ep_memory_enable_i),
      .extended_tag_enable_i(ep_extended_tag_enable_i),
      .max_payload_bytes_i  (ep_max_payload_bytes_i),
      .max_read_bytes_i     (ep_max_read_bytes_i),
      .rcb_128b_i           (ep_rcb_128b_i),

      .command_valid_i       (ep_command_valid_i),
      .command_ready_o       (ep_command_ready_o),
      .command_i             (ep_command_i),
      .command_address_i     (ep_command_address_i),
      .command_byte_count_i  (ep_command_byte_count_i),
      .command_tc_i          (ep_command_tc_i),
      .command_attr_i        (ep_command_attr_i),
      .command_context_i     (ep_command_context_i),
      .command_prefix_valid_i(ep_command_prefix_valid_i),
      .command_prefix_i      (ep_command_prefix_i),
      .command_ecrc_enable_i (ep_command_ecrc_enable_i),
      .command_data_i        (ep_command_data_i),
      .command_keep_i        (ep_command_keep_i),
      .command_data_valid_i  (ep_command_data_valid_i),
      .command_data_last_i   (ep_command_data_last_i),
      .command_data_ready_o  (ep_command_data_ready_o),
      .command_error_valid_o (ep_command_error_valid_o),
      .command_error_code_o  (ep_command_error_code_o),

      .target_request_valid_o  (ep_target_request_valid_o),
      .target_request_ready_i  (ep_target_request_ready_i),
      .target_request_header_o (ep_target_request_header_s),
      .target_request_class_o  (ep_target_request_class_o),
      .target_memory_o         (ep_target_memory_o),
      .target_config_o         (ep_target_config_o),
      .target_config_hit_o     (ep_target_config_hit_o),
      .target_config_type_one_o(ep_target_config_type_one_o),
      .target_config_offset_o  (ep_target_config_offset_o),
      .target_read_o           (ep_target_read_o),
      .target_write_o          (ep_target_write_o),
      .target_unsupported_o    (ep_target_unsupported_o),
      .target_bar_hit_o        (ep_target_bar_hit_o),
      .target_bar_overlap_o    (ep_target_bar_overlap_o),
      .target_bar_o            (ep_target_bar_o),
      .target_offset_o         (ep_target_offset_o),
      .target_data_o           (ep_target_data_o),
      .target_keep_o           (ep_target_keep_o),
      .target_data_valid_o     (ep_target_data_valid_o),
      .target_data_last_o      (ep_target_data_last_o),
      .target_data_ready_i     (ep_target_data_ready_i),

      .completion_request_valid_i        (ep_completion_request_valid_i),
      .completion_request_ready_o        (ep_completion_request_ready_o),
      .completion_request_header_i       (ep_completion_request_header_s),
      .completion_request_status_i       (ep_completion_request_status_i),
      .completion_request_byte_count_i   (ep_completion_request_byte_count_i),
      .completion_request_lower_address_i(ep_completion_request_lower_address_i),
      .completion_request_ecrc_enable_i  (ep_completion_request_ecrc_enable_i),
      .completion_request_data_i         (ep_completion_request_data_i),
      .completion_request_keep_i         (ep_completion_request_keep_i),
      .completion_request_data_valid_i   (ep_completion_request_data_valid_i),
      .completion_request_data_last_i    (ep_completion_request_data_last_i),
      .completion_request_data_ready_o   (ep_completion_request_data_ready_o),

      .received_completion_valid_o     (ep_received_completion_valid_o),
      .received_completion_ready_i     (ep_received_completion_ready_i),
      .received_completion_header_o    (ep_received_completion_header_s),
      .received_completion_data_o      (ep_received_completion_data_o),
      .received_completion_keep_o      (ep_received_completion_keep_o),
      .received_completion_data_valid_o(ep_received_completion_data_valid_o),
      .received_completion_data_last_o (ep_received_completion_data_last_o),
      .received_completion_data_ready_i(ep_received_completion_data_ready_i),

      .result_valid_o  (ep_result_valid_o),
      .result_ready_i  (ep_result_ready_i),
      .result_context_o(ep_result_context_o),
      .result_status_o (ep_result_status_o),
      .result_last_o   (ep_result_last_o),

      .cfg_bus_number_o     (ep_cfg_bus_number_o),
      .cfg_device_number_o  (ep_cfg_device_number_o),
      .cfg_function_number_o(ep_cfg_function_number_o),
      .fc_initialized_o     (ep_fc_initialized_o),
      .fc_update_valid_o    (ep_fc_update_valid_o),
      .fc_ph_o              (ep_fc_ph_o),
      .fc_pd_o              (ep_fc_pd_o),
      .fc_nph_o             (ep_fc_nph_o),
      .fc_npd_o             (ep_fc_npd_o),
      .fc_cplh_o            (ep_fc_cplh_o),
      .fc_cpld_o            (ep_fc_cpld_o),
      .malformed_o          (ep_malformed_o),
      .rx_error_valid_o     (ep_rx_error_valid_o),
      .rx_error_code_o      (ep_rx_error_code_o),
      .rx_ecrc_error_o      (ep_rx_ecrc_error_o),
      .tx_error_valid_o     (ep_tx_error_valid_o),
      .tx_error_code_o      (ep_tx_error_code_o),
      .tx_fc_blocked_o      (ep_tx_fc_blocked_o),
      .credit_error_o       (ep_credit_error_o),
      .vc_overflow_o        (ep_vc_overflow_o),
      .unexpected_completion_o(ep_unexpected_completion_o),
      .completion_error_code_o(ep_completion_error_code_o),
      .cpl_timeout_valid_o  (ep_cpl_timeout_valid_o),
      .cpl_timeout_tag_o    (ep_cpl_timeout_tag_o),
      .late_cpl_valid_o     (ep_late_cpl_valid_o),
      .late_cpl_tag_o       (ep_late_cpl_tag_o),
      .outstanding_o        (ep_outstanding_o)
  );

endmodule
