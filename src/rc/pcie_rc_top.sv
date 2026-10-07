// ---------------------------------------------------------------------------
// pcie_rc_top -- Root Complex from the enumeration engine to the PIPE
//
// Author: Kourosh Ghahramani
// Silicon Systems Research Lab, University of Washington
//
// Purpose
//   The Root Complex as one netlist, in three instances:
//     pcie_enum_top   u_enum  enumeration engine
//     pcie_rq_rc_top  u_tl    Transaction Layer with PG213-style interfaces
//     pcie_phy_top    u_phy   Data Link Layer, LTSSM and logical PHY
//   It ends on a PIPE interface of PHY_DATA_WIDTH data bits and
//   PHY_DATA_WIDTH / 8 K flags per lane, before 8b/10b encoding: the
//   scrambler is inside u_phy, the 8b/10b codec outside this module. u_phy
//   holds the only pcie_datalink_layer. The engine starts only after FC
//   initialization and owns the RQ socket until enum_done_o; s_axis_rq_*
//   owns it after that.
//
// Interfaces
//   Control       en_i: the LTSSM leaves ST_IDLE only while it is high.
//                 transmit_enable_i: a term of u_tl's transmit gate.
//   PIPE          phy_tx*, phy_rx*, PIPE command, status and equalization
//                 signals: to and from the PHY, through u_phy.
//   PHY assist    as_mac_in_detect, as_cdr_hold_req: assist outputs to the
//                 PHY (PG239, Table 14). tx_elec_idle, phy_ready_en: not read.
//                 ltssm_debug_state: the LTSSM state.
//   Link status   link_up_o, fc_initialized_o, fc_init_done_o, ok_to_issue_o:
//                 link and FC-init levels and the standing transmit conditions.
//   Identity      requester_id_i to rcb_128b_i: inputs to u_tl.
//                 cfg_*_number_o: observation only.
//   Enumeration   scan_start_i, scan_bus_i, bar_enable_i, bridge_enable_i and
//                 the level-1 and level-2 results: u_enum's ports.
//   Requester     s_axis_rq_*, m_axis_rc_*, pcie_rq_tag_*, rq_engine_owns_o:
//                 u_tl's requester socket, shared with u_enum.
//   Completer     m_axis_cq_*, s_axis_cc_*: u_tl's.
//   Status        rq_*, rc_*, cq_*, cc_* and the Transaction Layer error and
//                 Completion Timeout outputs: u_tl's.
//
// Clock and reset
//   clk_i runs u_enum, u_tl and the Data Link Layer; the LTSSM runs on
//   pipe_rx_usr_clk_i, and u_phy's PHY paths use all three clocks. The Data
//   Link Layer and the LTSSM derive their timers from CLK_PERIOD_NS. rst_i
//   is active high and synchronous, except in u_phy's pcie_datalink_init and
//   clock-crossing FIFOs (async_fifo, axis_async_fifo), which reset
//   asynchronously. A low link_up_o also resets u_tl's Transaction Layer;
//   u_enum is reset by rst_i only. phy_phystatus_rst also resets the LTSSM
//   and the logical PHY inside u_phy.
//
// Limitations
//   - The RQ socket changes hands on enum_done_o, which pcie_enum_top takes
//     from its first BAR stage. With bar_enable_i and bridge_enable_i set and
//     a Type 1 device, that stage ends two cycles after the first scan, so
//     the engine loses the socket before its bridge-path requests and the
//     bus-number write never reaches u_tl. With bar_enable_i low, or after a
//     scan or first-BAR-stage error, the socket never leaves the engine.
//   - u_tl keeps pcie_rq_rc_top's default host-memory aperture; this module
//     passes no aperture parameter.
//   - u_phy drives neither phy_txswing nor the equalization outputs, and
//     reads neither the equalization inputs, tx_elec_idle, phy_ready_en nor
//     phy_rxdata_valid.
//   - The hold of CFG_HOLD_CYCLES counts from fc_init_done_o and covers the
//     engine's Configuration Requests only. Requests the host issues through
//     s_axis_rq_* after enum_done_o are not held.
//   - link_up_o is the LTSSM's level on pipe_rx_usr_clk_i. It reaches
//     ok_to_issue_o and u_tl's link_up_i, on clk_i, with no synchronizer;
//     the Data Link Layer's copy crosses through async_fifo in u_phy.
//
// Structure
//   Ports
//   Transaction Layer to Data Link Layer seam
//   Link and flow-control status
//   Start gate
//   RQ socket handoff
//   Enumeration engine
//   Transaction Layer
//   Data Link Layer, LTSSM and logical PHY
//
// References
//   PG239, Table 5: TX Data Signals for Ultrascale+ Devices Interface Ports
//   PG239, Table 12: TX Equalization Signals for Gen3 and Above Rate
//   PG239, Table 13: RX Equalization Signals for Gen3 and Above Rate
//   PG239, Table 14: Assist Signal
// ---------------------------------------------------------------------------

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
    // The default is 10 ms at an 8 ns clk_i.
    parameter int unsigned CPL_TIMEOUT_CYCLES = tlp_pkg::CPL_TIMEOUT_DEFAULT_CYCLES,
    parameter int unsigned CRS_RETRY_MAX      = 3,
    parameter int unsigned CRS_BACKOFF_CYCLES = 8,
    // pcie_cfg_txn's hold, counted from fc_init_done_o. 0 disables it.
    parameter int unsigned CFG_HOLD_CYCLES    = 0,
    parameter int CQ_USER_WIDTH   = 88,
    parameter int CC_USER_WIDTH   = 33,

    // ---- PHY / LTSSM -------------------------------------------------------
    // Clock period in ns, the one source for the Data Link Layer timers (on
    // clk_i) and the LTSSM timers (on pipe_rx_usr_clk_i) in u_phy.
    parameter int CLK_PERIOD_NS = 8,
    parameter int MAX_NUM_LANES = 1,
    // Per-lane PIPE data width at phy_txdata and phy_rxdata, with
    // PHY_DATA_WIDTH / 8 K flags. 16 at Gen1, where the PHY ignores bits
    // [31:16] (PG239, Table 5). u_phy takes it as PIPE_DATA_WIDTH; its
    // DATA_WIDTH is the Dword bus to the Transaction Layer.
    parameter int PHY_DATA_WIDTH = 16,
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

    // ---- PIPE data ---------------------------------------------------------
    // PHY_DATA_WIDTH data bits and PHY_DATA_WIDTH / 8 K flags per lane, before
    // 8b/10b encoding. The scrambler is inside u_phy; the 8b/10b codec is
    // outside this module.
    output logic [(MAX_NUM_LANES*PHY_DATA_WIDTH)-1:0] phy_txdata,
    output logic [MAX_NUM_LANES-1:0]                  phy_txdata_valid,
    output logic [(MAX_NUM_LANES*PHY_DATA_WIDTH/8)-1:0] phy_txdatak,
    output logic [MAX_NUM_LANES-1:0]                  phy_txstart_block,
    output logic [(2*MAX_NUM_LANES)-1:0]              phy_txsync_header,
    input  logic [(MAX_NUM_LANES*PHY_DATA_WIDTH)-1:0] phy_rxdata,
    input  logic [MAX_NUM_LANES-1:0]                  phy_rxdata_valid,
    input  logic [(MAX_NUM_LANES*PHY_DATA_WIDTH/8)-1:0] phy_rxdatak,
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

    // ---- PIPE equalization -------------------------------------------------
    // Gen3 and Gen4 only on UltraScale+ devices (PG239, Table 12 and Table
    // 13). u_phy neither drives these outputs nor reads these inputs.
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

    // ---- link and flow-control state ---------------------------------------
    // link_up_o comes from the LTSSM in u_phy. fc_initialized_o is the Data
    // Link Layer's level, unfiltered.
    output logic                        link_up_o,
    output logic                        fc_initialized_o,   // unfiltered
    output logic                        fc_init_done_o,     // == fc_initialized_o
    output logic                        ok_to_issue_o,

    // ---- Root Complex identity and limits ----------------------------------
    // Inputs to u_tl. cfg_*_number_o are the numbers u_phy's Data Link Layer
    // stored from a received Type 0 Configuration Write: observation only.
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

    // ---- requester interfaces ----------------------------------------------
    // u_enum owns the RQ socket until enum_done_o rises and s_axis_rq_* owns
    // it after that (RQ socket handoff below); rq_engine_owns_o says which.
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

  // -------------------------------------------------------------------------
  // Transaction Layer to Data Link Layer seam
  // -------------------------------------------------------------------------
  // The wires between u_tl's DLL streams and u_phy's TLP streams, fixed at
  // the Data Link Layer's 32-bit Dword stream as in pcie_rc_dl_top.
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

  // u_phy takes tuser at PHY_USER_WIDTH and u_tl at TL_USER_WIDTH. tuser
  // carries nothing across this seam: tlp2dllp overwrites it on the way out
  // and tlp_parser does not read it on the way in, so zero-extending one way
  // and truncating the other loses nothing.
  logic [PHY_USER_WIDTH-1:0] tl_to_dl_tuser_w;
  assign tl_to_dl_tuser_w = {{(PHY_USER_WIDTH-TL_USER_WIDTH){1'b0}}, tl_to_dl_tuser};
  logic [PHY_USER_WIDTH-1:0] dl_to_tl_tuser_w;
  assign dl_to_tl_tuser = dl_to_tl_tuser_w[TL_USER_WIDTH-1:0];

  // ---- the Data Link Layer's flow-control status, u_phy to u_tl ------------
  logic        dl_fc_initialized;
  logic        dl_fc_update_valid;
  logic [7:0]  dl_fc_ph;
  logic [11:0] dl_fc_pd;
  logic [7:0]  dl_fc_nph;
  logic [11:0] dl_fc_npd;
  logic [7:0]  dl_fc_cplh;
  logic [11:0] dl_fc_cpld;
  logic        phy_link_up;

  // -------------------------------------------------------------------------
  // Link and flow-control status
  // -------------------------------------------------------------------------
  // fc_initialized_o and fc_init_done_o are the Data Link Layer's level with
  // no filter: pcie_flow_ctrl_init holds fc2_values_sent_o from CHECK_FC2's
  // exit and dllp_handler's InitFC2 flags stay set, so the level falls only
  // on reset or link-down. ok_to_issue_o is the standing part of tlp_layer's
  // transmit gate, as in pcie_rc_dl_top; the credit part depends on the
  // packet waiting and has no class-independent level.
  assign fc_initialized_o = dl_fc_initialized;
  assign fc_init_done_o   = dl_fc_initialized;
  assign link_up_o        = phy_link_up;
  assign ok_to_issue_o    = dl_fc_initialized && transmit_enable_i && phy_link_up;

  // -------------------------------------------------------------------------
  // Start gate
  // -------------------------------------------------------------------------
  // The engine starts only after FC initialization. tlp_requester allocates a
  // tag before the credit gate, so a request issued earlier would be tagged
  // and held in the VC buffer, and tlp_request_tracker times it out without
  // its being transmitted if the gate stays shut for CPL_TIMEOUT_CYCLES.
  // start_pending_r remembers a scan_start_i that arrives while the gate is
  // shut, so a one-cycle start is delayed rather than lost; pcie_enum_scan
  // samples its start only in S_IDLE. The latch clears once the scan is
  // busy. Unlike the one in pcie_enum_dl_top, it is not cleared by a
  // link-down.
  logic start_pending_r;
  always_ff @(posedge clk_i) begin
    if (rst_i)                   start_pending_r <= 1'b0;
    else if (scan_busy_o)        start_pending_r <= 1'b0;
    else if (scan_start_i)       start_pending_r <= 1'b1;
  end

  logic scan_start_gated;
  assign scan_start_gated = fc_init_done_o && (scan_start_i || start_pending_r);

  // -------------------------------------------------------------------------
  // RQ socket handoff
  // -------------------------------------------------------------------------
  // u_enum owns u_tl's RQ socket until enum_done_o rises; s_axis_rq_* owns it
  // after. enum_done_o holds until reset (pcie_enum_bar's S_DONE), so the
  // socket changes hands at most once: the same static select on a terminal
  // level that pcie_enum_top uses between its five stages. The back channel
  // is gated too: the side that does not own the socket sees tready at 0,
  // so it cannot complete a handshake against traffic that is not its own.
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

  // The RC return path is not muxed: u_enum and m_axis_rc_* both see u_tl's
  // completion stream, and only the owner's tready reaches u_tl. Before
  // enum_done_o the external port observes the engine's completions.
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

  // -------------------------------------------------------------------------
  // Enumeration engine
  // -------------------------------------------------------------------------
  // pcie_enum_top: device scan, BAR assignment and the bridge path, on u_tl's
  // RQ and RC sockets through the handoff above. Its start is
  // scan_start_gated, and u_tl's cpl_timeout_* end a request that gets no
  // completion. A low link_up_o resets tlp_request_tracker inside u_tl but
  // not u_enum, so a request outstanding at a link-down gets no timeout.
  pcie_enum_top #(
      .AXIS_DATA_WIDTH   (AXIS_DATA_WIDTH),
      .AXIS_KEEP_WIDTH   (AXIS_KEEP_WIDTH),
      .AXIS_USER_WIDTH   (AXIS_USER_WIDTH),
      .CRS_RETRY_MAX     (CRS_RETRY_MAX),
      .CRS_BACKOFF_CYCLES(CRS_BACKOFF_CYCLES),
      .CPL_TIMEOUT_CYCLES(CPL_TIMEOUT_CYCLES),
      .CFG_HOLD_CYCLES   (CFG_HOLD_CYCLES)
  ) u_enum (
      .clk_i(clk_i),
      .rst_i(rst_i),

      // DL_Active (PCIe Base Spec r3.0, §6.7.3.3), on clk_i and low again at
      // a link-down, so the hold restarts at each link-up.
      .link_active_i(fc_init_done_o),

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

      // Annotation only: it qualifies a timeout report, not control flow.
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

  // -------------------------------------------------------------------------
  // Transaction Layer
  // -------------------------------------------------------------------------
  // pcie_rq_rc_top with PCIE_WIRE_ORDER = 1, the byte order of the Data Link
  // Layer streams. Its link_up_i and fc_* inputs come straight from u_phy,
  // with no filter.
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

  // -------------------------------------------------------------------------
  // Data Link Layer, LTSSM and logical PHY
  // -------------------------------------------------------------------------
  // pcie_phy_top, which holds the only pcie_datalink_layer in this module.
  // IS_UPSTREAM, CROSSLINK_EN and UPCONFIG_EN are tied 0, and pcie_phy_top
  // does not use them.
  pcie_phy_top #(
      .CLK_PERIOD_NS(CLK_PERIOD_NS),
      .MAX_NUM_LANES(MAX_NUM_LANES),
      .DATA_WIDTH   (TL_DATA_WIDTH),    // the Dword bus to u_tl
      .PIPE_DATA_WIDTH(PHY_DATA_WIDTH),  // per-lane PIPE data width
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

      // ---- flow-control status to u_tl ------------------------------------
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
