// ===========================================================================
// pcie_rc_top -- the Root Complex as ONE netlist, engine to PIPE seam.
//
// Three instantiations and no fourth:
//
//     pcie_enum_top   u_enum   the enumeration engine
//     pcie_rq_rc_top  u_tl     the Transaction Layer
//     pcie_phy_top    u_phy    Data Link Layer + LTSSM + logical PHY
//
// Every earlier RC top stopped somewhere above the wire.  pcie_enum_dl_top is
// engine + TL + DLL and terminates on an AXIS byte stream; pcie_phy_top is
// DLL + LTSSM + PHY and terminates on the 32+4 PIPE interface.  Both carry a
// pcie_datalink_layer, so stacking one on the other would instantiate the Data
// Link Layer TWICE.  This module drops pcie_enum_dl_top's half and keeps
// pcie_phy_top's, which is what makes the result a single stack from the
// enumeration engine to the PIPE pins rather than two stacks glued together.
//
// == THE SEAM ==============================================================
//
// The TL<->DLL seam is the same module on both sides -- pcie_datalink_layer --
// so its AXIS half matches in width, direction, tuser layout and handshake BY
// CONSTRUCTION, not by agreement.  That is the whole reason this composition is
// cheap.  See ~/pcie_docs/evidence/fullstack/FINDINGS_PHASE0.md SS0a.
//
// !! THE FLOW-CONTROL HALF DID NOT MATCH, AND THAT WAS THIS RUNG'S FIRST STOP.
// pcie_datalink_layer publishes EIGHT fc_* outputs.  pcie_phy_top connected one
// (fc_initialized_o) and omitted seven.  Without them the Transaction Layer is
// silent and reports no error -- pcie_rq_rc_top.sv:29-46 names that failure
// mode and calls it regression RC1.  The seven were added to pcie_phy_top as
// pass-throughs under decision D-FS.1; this module is the first consumer.
//
// !! AND NOTE WHICH ONE WAS ALREADY WIRED.  fc_initialized_o -- so "did FC init
// complete?" answers YES while nothing can be transmitted.  A smoke test built
// on the obvious signal would have passed over a mute stack.
//
// == FC-INIT IS CONSUMED UNFILTERED =========================================
//
// pcie_rc_dl_top holds fc_init_sticky_r, a sticky bit that hides a 1->0->1
// glitch in the DLL's raw fc_initialized_o while pcie_flow_ctrl_init walked its
// ST_UPDATE_* states.  That glitch was fixed AT THE SOURCE (conformance defect
// #3, Base 2.1 p.158 / p.161: FC-init completion is a one-way event), so the
// filter is redundant rather than wrong.
//
// This module does not carry it.  It wires u_phy.fc_initialized_o straight to
// the TL and to the start gate, which makes it THE FIRST UNFILTERED CONSUMER IN
// EITHER VERTICAL.  If the source fix is complete this is safe; if it is not,
// this module is where that shows, and row 1 is the row that says so.
// pcie_rc_dl_top keeps the filter and keeps its own gate targets -- removing it
// there is CL-2, not this rung.
//
// == THE START GATE, AND WHY THE LATCH COMES WITH IT ========================
//
// pcie_enum_dl_top.sv:39 records that a bare scan_start_i && fc_init_done would
// ANNIHILATE a start request arriving before FC init rather than delay it,
// because a requester is entitled to PULSE the start.  Tag allocation sits
// UPSTREAM of the credit gate and the completion timer measures from
// ALLOCATION, so an early start produces a request that is tagged, parked, and
// times out having never been transmitted.  The latch is reproduced here for
// the same reason it exists there -- nothing in the engine remembers a request
// that was masked away.
//
// == THE RQ SOCKET HAS TWO MASTERS NOW ======================================
//
// pcie_enum_top is the only master on the RQ socket inside pcie_enum_dl_top --
// a claim scoped to THAT top, and its header now says so.  Here it still owns
// the socket until enumeration finishes; after that an external requester does.
// The handoff is a SIXTH ARM in the same static terminal-level idiom the engine
// already uses internally for its five stages (pcie_enum_top.sv:590-657),
// including the back-channel gating: a non-owner is told the primitive is
// permanently busy and permanently silent, so it cannot complete a handshake
// against traffic that is not its own.  The engine is not modified.
//
// == WHAT THIS MODULE DELIBERATELY DOES NOT DO ==============================
//
// No base/limit aperture ports.  The aperture is parameters all the way down --
// tlp_bar_decoder's BAR_BASE/BAR_MASK/BAR_ENABLE and pcie_rq_rc_top's
// HOST_MEM_SIZE -- so there is no register for a port to sit in front of, and a
// base/limit PORT would be a dead pin.  Making it live is the comparator
// rewrite (mask -> base/LIMIT), a datapath change to a module the Endpoint
// shares, and it is its own rung.  The aperture is exposed as PARAMETERS here.
//
// ===========================================================================

module pcie_rc_top
  import tlp_pkg::*;
  import pcie_rq_rc_pkg::*;
  import pcie_enum_pkg::*;
#(
    // ---- Transaction Layer / requester surface -----------------------------
    parameter int AXIS_DATA_WIDTH = 128,
    parameter int AXIS_KEEP_WIDTH = AXIS_DATA_WIDTH / 32,
    parameter int AXIS_USER_WIDTH = 60,
    parameter int CONTEXT_WIDTH   = 16,
    parameter int TAG_COUNT       = 32,
    parameter int unsigned CPL_TIMEOUT_CYCLES = 32'd4096,
    parameter int unsigned CRS_RETRY_MAX      = 3,
    parameter int unsigned CRS_BACKOFF_CYCLES = 8,
    parameter int CQ_USER_WIDTH   = 88,
    parameter int CC_USER_WIDTH   = 33,

    // ---- PHY / LTSSM -------------------------------------------------------
    parameter int CLK_RATE      = 125,
    parameter int MAX_NUM_LANES = 1,
    parameter int PHY_DATA_WIDTH = 32,
    parameter int PHY_USER_WIDTH = 5,
    parameter int IS_ROOT_PORT  = 1,
    parameter int LINK_NUM      = 0,
    parameter int SIM_FAST_LINK = 0
) (
    input  logic                        clk_i,
    input  logic                        rst_i,
    input  logic                        en_i,
    input  logic                        pipe_rx_usr_clk_i,
    input  logic                        pipe_tx_usr_clk_i,
    input  logic                        transmit_enable_i,

    // =======================================================================
    // PIPE seam -- 32+4, pre-8b/10b.  This module contains no scrambler and no
    // codec; both sit outside, which is why the RC↔EP bench has to instantiate
    // them to reach an Endpoint that carries its own.
    // =======================================================================
    output logic [(MAX_NUM_LANES*PHY_DATA_WIDTH)-1:0] phy_txdata,
    output logic [MAX_NUM_LANES-1:0]                  phy_txdata_valid,
    output logic [(4*MAX_NUM_LANES)-1:0]              phy_txdatak,
    output logic [MAX_NUM_LANES-1:0]                  phy_txstart_block,
    output logic [(2*MAX_NUM_LANES)-1:0]              phy_txsync_header,
    input  logic [(MAX_NUM_LANES*PHY_DATA_WIDTH)-1:0] phy_rxdata,
    input  logic [MAX_NUM_LANES-1:0]                  phy_rxdata_valid,
    input  logic [(4*MAX_NUM_LANES)-1:0]              phy_rxdatak,
    input  logic [MAX_NUM_LANES-1:0]                  phy_rxstart_block,
    input  logic [(2*MAX_NUM_LANES)-1:0]              phy_rxsync_header,

    // ---- PIPE command / status --------------------------------------------
    output wire                          phy_txdetectrx,
    output wire [MAX_NUM_LANES-1:0]      phy_txelecidle,
    output wire [MAX_NUM_LANES-1:0]      phy_txcompliance,
    output wire [MAX_NUM_LANES-1:0]      phy_rxpolarity,
    output wire [1:0]                    phy_powerdown,
    output wire [2:0]                    phy_rate,
    input  wire [MAX_NUM_LANES-1:0]      phy_rxvalid,
    input  wire [MAX_NUM_LANES-1:0]      phy_phystatus,
    input  wire                          phy_phystatus_rst,
    input  wire [MAX_NUM_LANES-1:0]      phy_rxelecidle,
    input  wire [(MAX_NUM_LANES*3)-1:0]  phy_rxstatus,
    output wire [2:0]                    phy_txmargin,
    output wire                          phy_txswing,
    output wire                          phy_txdeemph,
    output wire [8-1:0]                  pipe_width_o,

    // ---- PIPE equalization (Gen3/4 -- inert at Gen1, carried for shape) ----
    output wire [(MAX_NUM_LANES*2)-1:0]  phy_txeq_ctrl,
    output wire [(MAX_NUM_LANES*4)-1:0]  phy_txeq_preset,
    output wire [(MAX_NUM_LANES*6)-1:0]  phy_txeq_coeff,
    input  wire [5:0]                    phy_txeq_fs,
    input  wire [5:0]                    phy_txeq_lf,
    input  wire [(MAX_NUM_LANES*18)-1:0] phy_txeq_new_coeff,
    input  wire [MAX_NUM_LANES-1:0]      phy_txeq_done,
    output wire [(MAX_NUM_LANES*2)-1:0]  phy_rxeq_ctrl,
    output wire [(MAX_NUM_LANES*4)-1:0]  phy_rxeq_txpreset,
    input  wire [MAX_NUM_LANES-1:0]      phy_rxeq_preset_sel,
    input  wire [(MAX_NUM_LANES*18)-1:0] phy_rxeq_new_txcoeff,
    input  wire [MAX_NUM_LANES-1:0]      phy_rxeq_adapt_done,
    input  wire [MAX_NUM_LANES-1:0]      phy_rxeq_done,

    // ---- PHY bring-up / observation ---------------------------------------
    input  wire                          tx_elec_idle,
    input  wire                          phy_ready_en,
    output reg                           as_mac_in_detect,
    output reg                           as_cdr_hold_req,
    output wire [20:0]                   ltssm_debug_state,

    // =======================================================================
    // Link and flow-control state.  link_up_o is an OUTPUT here; the same
    // condition was an INPUT (phy_link_up_i) on every earlier RC top, because
    // the LTSSM that decides it now lives inside this module.
    // =======================================================================
    output logic                        link_up_o,
    output logic                        fc_initialized_o,   // UNFILTERED
    output logic                        fc_init_done_o,     // == fc_initialized_o
    output logic                        ok_to_issue_o,

    // ---- Root Complex identity (fixed BDF, 00:00.0) ------------------------
    input  logic [15:0]                 requester_id_i,
    input  logic [15:0]                 completer_id_i,
    input  logic [7:0]                  bus_number_i,
    input  logic [4:0]                  device_number_i,
    input  logic [2:0]                  function_number_i,
    input  logic                        memory_enable_i,
    input  logic                        extended_tag_enable_i,
    input  logic [12:0]                 max_payload_bytes_i,
    input  logic [12:0]                 max_read_bytes_i,
    input  logic                        rcb_128b_i,
    output logic [7:0]                  cfg_bus_number_o,
    output logic [4:0]                  cfg_device_number_o,
    output logic [2:0]                  cfg_function_number_o,

    // ---- Enumeration control ----------------------------------------------
    input  logic                        scan_start_i,
    input  logic [7:0]                  scan_bus_i,
    input  logic                        bar_enable_i,
    input  logic                        bridge_enable_i,

    // ---- Enumeration results, level 1 --------------------------------------
    output logic                        scan_busy_o,
    output logic                        scan_done_o,
    output logic                        scan_error_o,
    output enum_error_e                 scan_error_code_o,
    output logic                        err_credit_blocked_o,
    output logic                        device_present_o,
    output logic                        unsupported_device_o,
    output logic [15:0]                 device_bdf_o,
    output logic [15:0]                 vendor_id_o,
    output logic [15:0]                 device_id_o,
    output logic [7:0]                  header_type_o,
    output logic                        multifunction_o,
    output logic                        bar_busy_o,
    output logic                        enum_done_o,
    output logic                        enum_error_o,
    output enum_error_e                 enum_error_code_o,
    output logic [3:0]                  bar_count_o,
    output logic [BAR_SLOTS-1:0]        bar_valid_o,
    output logic [BAR_SLOTS-1:0]        bar_is_64_o,
    output logic [BAR_SLOTS-1:0]        bar_prefetch_o,
    output logic [BAR_SLOTS*64-1:0]     bar_size_o,
    output logic [BAR_SLOTS*64-1:0]     bar_addr_o,
    output logic [BAR_SLOTS-1:0]        io_bar_mask_o,

    // ---- Enumeration results, level 2 (bridge path) ------------------------
    output logic                        bus_done_o,
    output logic                        bus_bypassed_o,
    output logic                        sec_scan_done_o,
    output logic                        sec_device_present_o,
    output logic                        sec_unsupported_device_o,
    output logic [15:0]                 sec_device_bdf_o,
    output logic [15:0]                 sec_vendor_id_o,
    output logic [15:0]                 sec_device_id_o,
    output logic [7:0]                  sec_header_type_o,
    output logic                        sec_multifunction_o,
    output logic                        sec_enum_done_o,
    output logic [3:0]                  sec_bar_count_o,
    output logic [BAR_SLOTS-1:0]        sec_bar_valid_o,
    output logic [BAR_SLOTS-1:0]        sec_bar_is_64_o,
    output logic [BAR_SLOTS-1:0]        sec_bar_prefetch_o,
    output logic [BAR_SLOTS*64-1:0]     sec_bar_size_o,
    output logic [BAR_SLOTS*64-1:0]     sec_bar_addr_o,
    output logic [BAR_SLOTS-1:0]        sec_io_bar_mask_o,

    // =======================================================================
    // The requester surface -- row 4's entry point, and Stage G's.
    // The engine owns this socket until enum_done_o; after that these pins do.
    // =======================================================================
    input  logic [AXIS_DATA_WIDTH-1:0]  s_axis_rq_tdata,
    input  logic [AXIS_KEEP_WIDTH-1:0]  s_axis_rq_tkeep,
    input  logic                        s_axis_rq_tvalid,
    input  logic                        s_axis_rq_tlast,
    input  logic [AXIS_USER_WIDTH-1:0]  s_axis_rq_tuser,
    output logic                        s_axis_rq_tready,
    output logic [AXIS_DATA_WIDTH-1:0]  m_axis_rc_tdata,
    output logic [AXIS_KEEP_WIDTH-1:0]  m_axis_rc_tkeep,
    output logic                        m_axis_rc_tvalid,
    output logic                        m_axis_rc_tlast,
    input  logic                        m_axis_rc_tready,
    output logic [7:0]                  pcie_rq_tag_o,
    output logic                        pcie_rq_tag_vld_o,
    output logic                        rq_engine_owns_o,

    // ---- Completer surface -------------------------------------------------
    output logic [AXIS_DATA_WIDTH-1:0]  m_axis_cq_tdata,
    output logic [AXIS_KEEP_WIDTH-1:0]  m_axis_cq_tkeep,
    output logic                        m_axis_cq_tvalid,
    output logic                        m_axis_cq_tlast,
    output logic [CQ_USER_WIDTH-1:0]    m_axis_cq_tuser,
    input  logic                        m_axis_cq_tready,
    input  logic [AXIS_DATA_WIDTH-1:0]  s_axis_cc_tdata,
    input  logic [AXIS_KEEP_WIDTH-1:0]  s_axis_cc_tkeep,
    input  logic                        s_axis_cc_tvalid,
    input  logic                        s_axis_cc_tlast,
    input  logic [CC_USER_WIDTH-1:0]    s_axis_cc_tuser,
    output logic                        s_axis_cc_tready,
    output logic                        cq_dropped_o,
    output logic [3:0]                  cq_error_code_o,
    output logic                        cq_gearbox_error_o,
    output logic                        cc_protocol_error_o,
    output logic [3:0]                  cc_error_code_o,
    output logic                        cc_gearbox_error_o,

    // ---- Error surface -----------------------------------------------------
    output logic                        rq_protocol_error_o,
    output rq_error_e                   rq_error_code_o,
    output logic                        rq_gearbox_error_o,
    output logic                        rc_unexpected_completion_o,
    output tlp_error_e                  rc_completion_error_code_o,
    output logic                        rc_protocol_error_o,
    output rc_error_e                   rc_error_code_o,
    output logic                        rc_gearbox_error_o,
    output logic                        command_error_valid_o,
    output tlp_error_e                  command_error_code_o,
    output logic                        malformed_o,
    output logic                        rx_error_valid_o,
    output tlp_error_e                  rx_error_code_o,
    output logic                        rx_ecrc_error_o,
    output logic                        tx_error_valid_o,
    output tlp_error_e                  tx_error_code_o,
    output logic                        tx_fc_blocked_o,
    output logic                        credit_error_o,
    output logic                        vc_overflow_o,
    output logic                        cpl_timeout_valid_o,
    output logic [7:0]                  cpl_timeout_tag_o,
    output logic                        late_cpl_valid_o,
    output logic [7:0]                  late_cpl_tag_o,
    output logic [$clog2(TAG_COUNT+1)-1:0] outstanding_o
);

  // =========================================================================
  // The TL<->DLL seam is fixed at the DLL's native 32-bit Dword-serial shape,
  // exactly as pcie_rc_dl_top.sv:210-213 fixes it.
  // =========================================================================
  localparam int TL_DATA_WIDTH = 32;
  localparam int TL_KEEP_WIDTH = 4;
  localparam int TL_USER_WIDTH = 3;

  logic [TL_DATA_WIDTH-1:0] tl_to_dl_tdata;
  logic [TL_KEEP_WIDTH-1:0] tl_to_dl_tkeep;
  logic                     tl_to_dl_tvalid;
  logic                     tl_to_dl_tlast;
  logic [TL_USER_WIDTH-1:0] tl_to_dl_tuser;
  logic                     tl_to_dl_tready;

  logic [TL_DATA_WIDTH-1:0] dl_to_tl_tdata;
  logic [TL_KEEP_WIDTH-1:0] dl_to_tl_tkeep;
  logic                     dl_to_tl_tvalid;
  logic                     dl_to_tl_tlast;
  logic [TL_USER_WIDTH-1:0] dl_to_tl_tuser;
  logic                     dl_to_tl_tready;

  // pcie_phy_top is instantiated at its TESTED USER_WIDTH (5), not narrowed to
  // the TL's 3, because every LTSSM and PHY gate row exercises it at 5 and this
  // rung is an integration, not a re-parameterisation of the PHY.  The width
  // adaptation below is provably harmless: tuser is INERT across this seam in
  // both directions -- outbound the DLL overwrites it (tlp2dllp.sv:263), and
  // inbound tlp_parser never reads it (pcie_rc_dl_top.sv:216-218).  It is
  // carried for AXIS shape only.
  logic [PHY_USER_WIDTH-1:0] tl_to_dl_tuser_w;
  assign tl_to_dl_tuser_w = {{(PHY_USER_WIDTH-TL_USER_WIDTH){1'b0}}, tl_to_dl_tuser};
  logic [PHY_USER_WIDTH-1:0] dl_to_tl_tuser_w;
  assign dl_to_tl_tuser = dl_to_tl_tuser_w[TL_USER_WIDTH-1:0];

  // ---- DLL flow-control status (D-FS.1's seven, plus the one that existed) --
  logic        dl_fc_initialized;
  logic        dl_fc_update_valid;
  logic [7:0]  dl_fc_ph;
  logic [11:0] dl_fc_pd;
  logic [7:0]  dl_fc_nph;
  logic [11:0] dl_fc_npd;
  logic [7:0]  dl_fc_cplh;
  logic [11:0] dl_fc_cpld;
  logic        phy_link_up;

  // =========================================================================
  // Outward view of link and flow-control state.
  //
  // ok_to_issue_o carries THREE of the four conjuncts of the real parking
  // condition (tlp_layer.sv:280); the fourth -- that a credit update has
  // actually loaded non-zero credits -- is not a standing level and is not
  // reconstructible from a port.  pcie_rc_dl_top.sv:259-265 makes the same
  // three-of-four statement, and this module reproduces it against the
  // UNFILTERED wire rather than against a sticky bit.
  // =========================================================================
  assign fc_initialized_o = dl_fc_initialized;
  assign fc_init_done_o   = dl_fc_initialized;
  assign link_up_o        = phy_link_up;
  assign ok_to_issue_o    = dl_fc_initialized && transmit_enable_i && phy_link_up;

  // =========================================================================
  // The start gate, and the latch that has to come with it.
  // pcie_enum_dl_top.sv:39 and :318-334 -- reproduced here because the reason
  // it exists is unchanged: a start request may be a one-cycle PULSE, and a
  // bare AND would annihilate rather than delay it.
  // =========================================================================
  logic start_pending_r;
  always_ff @(posedge clk_i) begin
    if (rst_i)                   start_pending_r <= 1'b0;
    else if (scan_busy_o)        start_pending_r <= 1'b0;
    else if (scan_start_i)       start_pending_r <= 1'b1;
  end

  logic scan_start_gated;
  assign scan_start_gated = fc_init_done_o && (scan_start_i || start_pending_r);

  // =========================================================================
  // The RQ socket's sixth arm.
  //
  // enum_done_o is a TERMINAL level -- it rises at most once per run -- which
  // is the same property every select in pcie_enum_top's own five-arm mux has
  // (pcie_enum_top.sv:590-596).  The back channels are gated exactly as that
  // mux gates its own (:652-657): the non-owner sees a primitive that is
  // permanently busy and permanently silent, so it cannot complete a handshake
  // against traffic that is not its own.
  //
  // The engine is NOT modified, and its internal handoff is not redesigned.
  // =========================================================================
  logic                       rq_engine_owns;
  assign rq_engine_owns   = !enum_done_o;
  assign rq_engine_owns_o = rq_engine_owns;

  logic [AXIS_DATA_WIDTH-1:0] enum_rq_tdata;
  logic [AXIS_KEEP_WIDTH-1:0] enum_rq_tkeep;
  logic                       enum_rq_tvalid;
  logic                       enum_rq_tlast;
  logic [AXIS_USER_WIDTH-1:0] enum_rq_tuser;
  logic                       enum_rq_tready;

  logic [AXIS_DATA_WIDTH-1:0] tl_rq_tdata;
  logic [AXIS_KEEP_WIDTH-1:0] tl_rq_tkeep;
  logic                       tl_rq_tvalid;
  logic                       tl_rq_tlast;
  logic [AXIS_USER_WIDTH-1:0] tl_rq_tuser;
  logic                       tl_rq_tready;

  assign tl_rq_tdata  = rq_engine_owns ? enum_rq_tdata  : s_axis_rq_tdata;
  assign tl_rq_tkeep  = rq_engine_owns ? enum_rq_tkeep  : s_axis_rq_tkeep;
  assign tl_rq_tvalid = rq_engine_owns ? enum_rq_tvalid : s_axis_rq_tvalid;
  assign tl_rq_tlast  = rq_engine_owns ? enum_rq_tlast  : s_axis_rq_tlast;
  assign tl_rq_tuser  = rq_engine_owns ? enum_rq_tuser  : s_axis_rq_tuser;

  assign enum_rq_tready   = rq_engine_owns ? tl_rq_tready : 1'b0;
  assign s_axis_rq_tready = rq_engine_owns ? 1'b0         : tl_rq_tready;

  // The RC (completion) return path is NOT muxed.  Completions are routed by
  // tag, and the engine reads the same stream it always did; the external port
  // observes it.  A requester that issues after enum_done_o gets its
  // completions on m_axis_rc_* like any other consumer.
  logic [AXIS_DATA_WIDTH-1:0] tl_rc_tdata;
  logic [AXIS_KEEP_WIDTH-1:0] tl_rc_tkeep;
  logic                       tl_rc_tvalid;
  logic                       tl_rc_tlast;
  logic                       tl_rc_tready;
  logic                       enum_rc_tready;

  assign m_axis_rc_tdata  = tl_rc_tdata;
  assign m_axis_rc_tkeep  = tl_rc_tkeep;
  assign m_axis_rc_tvalid = tl_rc_tvalid;
  assign m_axis_rc_tlast  = tl_rc_tlast;
  assign tl_rc_tready     = rq_engine_owns ? enum_rc_tready : m_axis_rc_tready;

  logic [7:0] tl_rq_tag;
  logic       tl_rq_tag_vld;
  assign pcie_rq_tag_o     = tl_rq_tag;
  assign pcie_rq_tag_vld_o = tl_rq_tag_vld;

  // =========================================================================
  // The enumeration engine.
  // =========================================================================
  pcie_enum_top #(
      .AXIS_DATA_WIDTH   (AXIS_DATA_WIDTH),
      .AXIS_KEEP_WIDTH   (AXIS_KEEP_WIDTH),
      .AXIS_USER_WIDTH   (AXIS_USER_WIDTH),
      .CRS_RETRY_MAX     (CRS_RETRY_MAX),
      .CRS_BACKOFF_CYCLES(CRS_BACKOFF_CYCLES),
      .CPL_TIMEOUT_CYCLES(CPL_TIMEOUT_CYCLES)
  ) u_enum (
      .clk_i(clk_i),
      .rst_i(rst_i),

      .scan_start_i   (scan_start_gated),
      .scan_bus_i     (scan_bus_i),
      .bar_enable_i   (bar_enable_i),
      .bridge_enable_i(bridge_enable_i),

      .scan_busy_o         (scan_busy_o),
      .scan_done_o         (scan_done_o),
      .scan_error_o        (scan_error_o),
      .scan_error_code_o   (scan_error_code_o),
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
      .enum_error_code_o(enum_error_code_o),
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

      // Annotation, NOT control flow -- pcie_enum_top.sv port comment.
      .tx_fc_blocked_i(tx_fc_blocked_o),

      .s_axis_rq_tdata_o (enum_rq_tdata),
      .s_axis_rq_tkeep_o (enum_rq_tkeep),
      .s_axis_rq_tvalid_o(enum_rq_tvalid),
      .s_axis_rq_tlast_o (enum_rq_tlast),
      .s_axis_rq_tuser_o (enum_rq_tuser),
      .s_axis_rq_tready_i(enum_rq_tready),

      .pcie_rq_tag_i    (tl_rq_tag),
      .pcie_rq_tag_vld_i(tl_rq_tag_vld),

      .m_axis_rc_tdata_i (tl_rc_tdata),
      .m_axis_rc_tkeep_i (tl_rc_tkeep),
      .m_axis_rc_tvalid_i(tl_rc_tvalid),
      .m_axis_rc_tlast_i (tl_rc_tlast),
      .m_axis_rc_tready_o(enum_rc_tready),

      .cpl_timeout_valid_i(cpl_timeout_valid_o),
      .cpl_timeout_tag_i  (cpl_timeout_tag_o)
  );

  // =========================================================================
  // The Transaction Layer.  Its fc_* inputs come STRAIGHT from the DLL inside
  // pcie_phy_top -- no sticky filter between them (Decision 2), and only
  // reachable at all because of D-FS.1.
  // =========================================================================
  pcie_rq_rc_top #(
      .AXIS_DATA_WIDTH(AXIS_DATA_WIDTH),
      .AXIS_KEEP_WIDTH(AXIS_KEEP_WIDTH),
      .AXIS_USER_WIDTH(AXIS_USER_WIDTH),
      .TL_DATA_WIDTH  (TL_DATA_WIDTH),
      .TL_KEEP_WIDTH  (TL_KEEP_WIDTH),
      .TL_USER_WIDTH  (TL_USER_WIDTH),
      .CONTEXT_WIDTH  (CONTEXT_WIDTH),
      .TAG_COUNT      (TAG_COUNT),
      .CPL_TIMEOUT_CYCLES(CPL_TIMEOUT_CYCLES),
      .PCIE_WIRE_ORDER(1'b1),
      .CQ_USER_WIDTH  (CQ_USER_WIDTH),
      .CC_USER_WIDTH  (CC_USER_WIDTH)
  ) u_tl (
      .clk_i(clk_i),
      .rst_i(rst_i),

      .link_up_i        (phy_link_up),
      .transmit_enable_i(transmit_enable_i),
      .fc_initialized_i (dl_fc_initialized),
      .fc_update_valid_i(dl_fc_update_valid),
      .fc_ph_i  (dl_fc_ph),   .fc_pd_i  (dl_fc_pd),
      .fc_nph_i (dl_fc_nph),  .fc_npd_i (dl_fc_npd),
      .fc_cplh_i(dl_fc_cplh), .fc_cpld_i(dl_fc_cpld),

      .requester_id_i   (requester_id_i),
      .completer_id_i   (completer_id_i),
      .bus_number_i     (bus_number_i),
      .device_number_i  (device_number_i),
      .function_number_i(function_number_i),
      .memory_enable_i  (memory_enable_i),
      .extended_tag_enable_i(extended_tag_enable_i),
      .max_payload_bytes_i  (max_payload_bytes_i),
      .max_read_bytes_i     (max_read_bytes_i),
      .rcb_128b_i           (rcb_128b_i),

      .s_axis_rq_tdata (tl_rq_tdata),
      .s_axis_rq_tkeep (tl_rq_tkeep),
      .s_axis_rq_tvalid(tl_rq_tvalid),
      .s_axis_rq_tlast (tl_rq_tlast),
      .s_axis_rq_tuser (tl_rq_tuser),
      .s_axis_rq_tready(tl_rq_tready),

      .pcie_rq_tag_o    (tl_rq_tag),
      .pcie_rq_tag_vld_o(tl_rq_tag_vld),

      .m_axis_rc_tdata (tl_rc_tdata),
      .m_axis_rc_tkeep (tl_rc_tkeep),
      .m_axis_rc_tvalid(tl_rc_tvalid),
      .m_axis_rc_tlast (tl_rc_tlast),
      .m_axis_rc_tready(tl_rc_tready),

      .s_dllp_axis_tdata (dl_to_tl_tdata),
      .s_dllp_axis_tkeep (dl_to_tl_tkeep),
      .s_dllp_axis_tvalid(dl_to_tl_tvalid),
      .s_dllp_axis_tlast (dl_to_tl_tlast),
      .s_dllp_axis_tuser (dl_to_tl_tuser),
      .s_dllp_axis_tready(dl_to_tl_tready),

      .m_dllp_axis_tdata (tl_to_dl_tdata),
      .m_dllp_axis_tkeep (tl_to_dl_tkeep),
      .m_dllp_axis_tvalid(tl_to_dl_tvalid),
      .m_dllp_axis_tlast (tl_to_dl_tlast),
      .m_dllp_axis_tuser (tl_to_dl_tuser),
      .m_dllp_axis_tready(tl_to_dl_tready),

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

      .rq_protocol_error_o(rq_protocol_error_o),
      .rq_error_code_o    (rq_error_code_o),
      .rq_gearbox_error_o (rq_gearbox_error_o),
      .rc_unexpected_completion_o(rc_unexpected_completion_o),
      .rc_completion_error_code_o(rc_completion_error_code_o),
      .rc_protocol_error_o(rc_protocol_error_o),
      .rc_error_code_o    (rc_error_code_o),
      .rc_gearbox_error_o (rc_gearbox_error_o),
      .command_error_valid_o(command_error_valid_o),
      .command_error_code_o (command_error_code_o),
      .malformed_o     (malformed_o),
      .rx_error_valid_o(rx_error_valid_o),
      .rx_error_code_o (rx_error_code_o),
      .rx_ecrc_error_o (rx_ecrc_error_o),
      .tx_error_valid_o(tx_error_valid_o),
      .tx_error_code_o (tx_error_code_o),
      .tx_fc_blocked_o (tx_fc_blocked_o),
      .credit_error_o  (credit_error_o),
      .vc_overflow_o   (vc_overflow_o),
      .cpl_timeout_valid_o(cpl_timeout_valid_o),
      .cpl_timeout_tag_o  (cpl_timeout_tag_o),
      .late_cpl_valid_o   (late_cpl_valid_o),
      .late_cpl_tag_o     (late_cpl_tag_o),
      .outstanding_o      (outstanding_o)
  );

  // =========================================================================
  // Data Link Layer + LTSSM + logical PHY.  This is the ONLY DLL in the design.
  // =========================================================================
  pcie_phy_top #(
      .CLK_RATE     (CLK_RATE),
      .MAX_NUM_LANES(MAX_NUM_LANES),
      .DATA_WIDTH   (PHY_DATA_WIDTH),
      .USER_WIDTH   (PHY_USER_WIDTH),
      .IS_ROOT_PORT (IS_ROOT_PORT),
      .LINK_NUM     (LINK_NUM),
      .IS_UPSTREAM  (0),
      .CROSSLINK_EN (0),
      .UPCONFIG_EN  (0),
      .SIM_FAST_LINK(SIM_FAST_LINK)
  ) u_phy (
      .clk_i            (clk_i),
      .rst_i            (rst_i),
      .en_i             (en_i),
      .pipe_rx_usr_clk_i(pipe_rx_usr_clk_i),
      .pipe_tx_usr_clk_i(pipe_tx_usr_clk_i),

      // ---- the eight, seven of which exist because of D-FS.1 --------------
      .fc_initialized_o (dl_fc_initialized),
      .fc_update_valid_o(dl_fc_update_valid),
      .fc_ph_o  (dl_fc_ph),   .fc_pd_o  (dl_fc_pd),
      .fc_nph_o (dl_fc_nph),  .fc_npd_o (dl_fc_npd),
      .fc_cplh_o(dl_fc_cplh), .fc_cpld_o(dl_fc_cpld),

      .phy_txdata       (phy_txdata),
      .phy_txdata_valid (phy_txdata_valid),
      .phy_txdatak      (phy_txdatak),
      .phy_txstart_block(phy_txstart_block),
      .phy_txsync_header(phy_txsync_header),
      .phy_rxdata       (phy_rxdata),
      .phy_rxdata_valid (phy_rxdata_valid),
      .phy_rxdatak      (phy_rxdatak),
      .phy_rxstart_block(phy_rxstart_block),
      .phy_rxsync_header(phy_rxsync_header),

      .phy_txdetectrx  (phy_txdetectrx),
      .phy_txelecidle  (phy_txelecidle),
      .phy_txcompliance(phy_txcompliance),
      .phy_rxpolarity  (phy_rxpolarity),
      .phy_powerdown   (phy_powerdown),
      .phy_rate        (phy_rate),

      .phy_rxvalid      (phy_rxvalid),
      .phy_phystatus    (phy_phystatus),
      .phy_phystatus_rst(phy_phystatus_rst),
      .phy_rxelecidle   (phy_rxelecidle),
      .phy_rxstatus     (phy_rxstatus),

      .phy_txmargin(phy_txmargin),
      .phy_txswing (phy_txswing),
      .phy_txdeemph(phy_txdeemph),

      .phy_txeq_ctrl       (phy_txeq_ctrl),
      .phy_txeq_preset     (phy_txeq_preset),
      .phy_txeq_coeff      (phy_txeq_coeff),
      .phy_txeq_fs         (phy_txeq_fs),
      .phy_txeq_lf         (phy_txeq_lf),
      .phy_txeq_new_coeff  (phy_txeq_new_coeff),
      .phy_txeq_done       (phy_txeq_done),
      .phy_rxeq_ctrl       (phy_rxeq_ctrl),
      .phy_rxeq_txpreset   (phy_rxeq_txpreset),
      .phy_rxeq_preset_sel (phy_rxeq_preset_sel),
      .phy_rxeq_new_txcoeff(phy_rxeq_new_txcoeff),
      .phy_rxeq_adapt_done (phy_rxeq_adapt_done),
      .phy_rxeq_done       (phy_rxeq_done),
      .pipe_width_o        (pipe_width_o),

      .cfg_bus_number_o     (cfg_bus_number_o),
      .cfg_device_number_o  (cfg_device_number_o),
      .cfg_function_number_o(cfg_function_number_o),

      .as_mac_in_detect(as_mac_in_detect),
      .as_cdr_hold_req (as_cdr_hold_req),
      .ltssm_debug_state(ltssm_debug_state),

      .tx_elec_idle(tx_elec_idle),
      .phy_ready_en(phy_ready_en),
      .link_up_o   (phy_link_up),

      .s_tlp_axis_tdata (tl_to_dl_tdata),
      .s_tlp_axis_tkeep (tl_to_dl_tkeep),
      .s_tlp_axis_tvalid(tl_to_dl_tvalid),
      .s_tlp_axis_tlast (tl_to_dl_tlast),
      .s_tlp_axis_tuser (tl_to_dl_tuser_w),
      .s_tlp_axis_tready(tl_to_dl_tready),

      .m_tlp_axis_tdata (dl_to_tl_tdata),
      .m_tlp_axis_tkeep (dl_to_tl_tkeep),
      .m_tlp_axis_tvalid(dl_to_tl_tvalid),
      .m_tlp_axis_tlast (dl_to_tl_tlast),
      .m_tlp_axis_tuser (dl_to_tl_tuser_w),
      .m_tlp_axis_tready(dl_to_tl_tready)
  );

endmodule
