// ---------------------------------------------------------------------------
// pcie_rc_gth_top -- Root Complex on the PG239 PCIe PHY IP, one lane
//
// Author: Kourosh Ghahramani
// Silicon Systems Research Lab, University of Washington
//
// Purpose
//   Puts pcie_rc_top (u_rc: enumeration engine, Transaction Layer, Data Link
//   Layer, LTSSM and logical PHY with its scrambler) on pg239_gen1_x1
//   (u_pg239), an instance of the AMD PCIe PHY IP, which provides 8b/10b
//   encoding and decoding, the RX elastic buffer and the GTH transceiver
//   (PG239, Table 1). The PIPE between them carries 16 data bits and 2 K
//   flags per lane. This module also holds the reference-clock buffers
//   PG239 needs and the reset synchroniser for u_rc.
//
//   fpga/zcu102/ip_pg239.tcl generates pg239_gen1_x1: x1 at 2.5 GT/s, a
//   100 MHz reference clock on MGTREFCLK0 of bank 130, lane 0 in GTH Quad
//   130; the generated IP is not committed. No .core file lists this module:
//   it needs that IP, the AMD IBUFDS_GTE4 and BUFG_GT primitives and the
//   XPM macro xpm_cdc_async_rst. tb/gth/tb_pcie_rc_gth.sv simulates it.
//
// Interfaces
//   Board         sys_clk_p, sys_clk_n: the 100 MHz reference clock.
//                 sys_rst_n: PCIe PERST#, active low. pci_exp_*: the lane.
//   Clock out     pclk_o: phy_pclk, the clock of u_rc's ports.
//   Control       en_i, transmit_enable_i: to u_rc.
//   Debug         dbg_*_o: copies of PIPE command, status and assist wires,
//                 the IP's phy_phystatus_rst and gt_gtpowergood.
//   Root Complex  ltssm_debug_state and link_up_o to outstanding_o: u_rc's
//                 ports, as in pcie_rc_top.
//
// Clock and reset
//   phy_pclk, 125 MHz at Gen1 (PG239, Table 4), is the only design clock: it
//   runs u_rc's clk_i and both PIPE user clocks, so CLK_PERIOD_NS is 8. u_rc
//   is reset by rc_rst, which u_rc_rst_sync drives on phy_pclk from PERST#
//   and phy_phystatus_rst (Reset of u_rc).
//
// Post-reset timing
//   The board's values are this module's parameter defaults, in phy_pclk
//   cycles: no Configuration Request until 100 ms after DL_Active
//   (CFG_HOLD_CYCLES), every CRS reissued until 1.0 s after it
//   (CRS_WINDOW_CYCLES), reissues 1 ms apart (CRS_BACKOFF_CYCLES), and a
//   budget of CRS_RETRY_MAX for a CRS first seen after the window (PCIe Base
//   Spec r2.1, §6.6.1). pcie_rc_gth_zcu102 passes none of them.
//
// Limitations
//   - One lane at Gen1: MAX_NUM_LANES and PHY_DATA_WIDTH are fixed here.
//   - u_rc drives neither phy_txswing nor the equalization controls; the
//     IP's inputs are tied to their idle or default values instead.
//
// Structure
//   Ports
//   Reference clock
//   PIPE between u_rc and u_pg239
//   Reset of u_rc
//   Root Complex
//   PG239 PHY
//   Debug taps
//
// References
//   PG239, Table 1: Default Features Supported
//   PG239, Table 4: Clock and Reset Signals
//   PG239, Table 5: TX Data Signals for Ultrascale+ Devices Interface Ports
//   PG239, Table 7: RX Data Signals for UltraScale+ Devices
//   PG239, Table 9: Command Signals
//   PG239, Table 10: Status Signals
//   PG239, Table 11: TX Driver Signals for Gen1 and Gen2
//   PG239, Table 12: TX Equalization Signals for Gen3 and Above Rate
//   PG239, Table 13: RX Equalization Signals for Gen3 and Above Rate
//   PG239, Table 14: Assist Signal
//   PG239, Table 16: GT Specific Ports For UltraScale+ Devices Only
//   UG576, Table 2-1: Reference Clock Input Ports (IBUFDS_GTE3/4)
//   PCIe Base Spec r2.1, §6.6.1
// ---------------------------------------------------------------------------

module pcie_rc_gth_top
  import tlp_pkg::*;
  import pcie_rq_rc_pkg::*;
  import pcie_enum_pkg::*;
#(
    parameter int AXIS_DATA_WIDTH = 128,
    parameter int AXIS_KEEP_WIDTH = AXIS_DATA_WIDTH / 32,
    parameter int AXIS_USER_WIDTH = 60,
    parameter int CONTEXT_WIDTH   = 16,
    parameter int TAG_COUNT       = 32,
    parameter int unsigned CPL_TIMEOUT_CYCLES = tlp_pkg::CPL_TIMEOUT_DEFAULT_CYCLES,
    parameter int unsigned CRS_RETRY_MAX      = 3,
    parameter int unsigned CRS_BACKOFF_CYCLES = 125_000,        // 1 ms
    parameter int unsigned CFG_HOLD_CYCLES    = 12_500_000,     // 100 ms
    parameter int unsigned CRS_WINDOW_CYCLES  = 125_000_000,    // 1.0 s
    parameter int CQ_USER_WIDTH   = 88,
    parameter int CC_USER_WIDTH   = 33,
    parameter int PHY_USER_WIDTH  = 5,
    parameter int IS_ROOT_PORT    = 1,
    parameter int LINK_NUM        = 0,
    parameter int SIM_FAST_LINK   = 0
) (
    // ---- the board side ----------------------------------------------------
    input  wire                         sys_clk_p,     // 100 MHz, MGTREFCLK0 of bank 130
    input  wire                         sys_clk_n,
    input  wire                         sys_rst_n,     // PCIe PERST#, active low
    output wire [0:0]                   pci_exp_txp,
    output wire [0:0]                   pci_exp_txn,
    input  wire [0:0]                   pci_exp_rxp,
    input  wire [0:0]                   pci_exp_rxn,
    output wire                         pclk_o,        // = phy_pclk, the design clock

    input  logic                        en_i,
    input  logic                        transmit_enable_i,
    output wire [20:0]                  ltssm_debug_state,

    // ---- debug taps --------------------------------------------------------
    // Copies of wires at u_pg239's ports, for the ILA in
    // fpga/zcu102/pcie_rc_gth_zcu102.sv. Nothing here reads them.
    output wire                         dbg_phy_phystatus_o,
    output wire                         dbg_phy_phystatus_rst_o,
    output wire [2:0]                   dbg_phy_rxstatus_o,
    output wire                         dbg_phy_rxvalid_o,
    output wire                         dbg_phy_rxelecidle_o,
    output wire                         dbg_as_mac_in_detect_o,
    output wire                         dbg_phy_txdetectrx_o,
    output wire                         dbg_phy_txelecidle_o,
    output wire [1:0]                   dbg_phy_powerdown_o,
    output wire                         dbg_gt_gtpowergood_o,

    // ---- link and flow-control state ---------------------------------------
    // link_up_o comes from the LTSSM in u_rc. fc_initialized_o is the Data
    // Link Layer's level, unfiltered.
    output logic                        link_up_o,
    output logic                        fc_initialized_o,   // unfiltered
    output logic                        fc_init_done_o,     // == fc_initialized_o
    output logic                        ok_to_issue_o,

    // ---- Root Complex identity and limits ----------------------------------
    // Inputs to u_rc's Transaction Layer. cfg_*_number_o are the numbers the
    // Data Link Layer stored from a received Type 0 Configuration Write:
    // observation only.
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
    // u_rc's engine owns the RQ socket until enum_done_o rises and
    // s_axis_rq_* owns it after that; rq_engine_owns_o says which.
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

  // The IP instance is x1 Gen1 with a 16-bit PIPE; these are not choices here.
  localparam int MAX_NUM_LANES  = 1;
  localparam int PHY_DATA_WIDTH = 16;

  // -------------------------------------------------------------------------
  // Reference clock
  // -------------------------------------------------------------------------
  // PG239 takes the reference clock twice: phy_gtrefclk straight from
  // IBUFDS_GTE4, and phy_refclk from a BUFG_GT (PG239, Table 4). The BUFG_GT
  // is fed from IBUFDS_GTE4's ODIV2 output, which is the output that drives a
  // BUFG_GT (UG576, Table 2-1), and its CE is gt_gtpowergood, as PG239
  // requires (PG239, Table 16).
  wire        sys_clk_gt;        // IBUFDS_GTE4.O     -> phy_gtrefclk
  wire        sys_clk_div2;      // IBUFDS_GTE4.ODIV2 -> BUFG_GT
  wire        sys_clk_bufg;      // BUFG_GT.O         -> phy_refclk
  wire [0:0]  gt_gtpowergood;
  wire        phy_pclk;

  IBUFDS_GTE4 refclk_ibuf (.O(sys_clk_gt), .ODIV2(sys_clk_div2), .I(sys_clk_p), .CEB(1'b0), .IB(sys_clk_n));
  BUFG_GT bufg_gt_sysclk (.CE(gt_gtpowergood[0]), .CEMASK(1'b0), .CLR(1'b0), .CLRMASK(1'b0),
                          .DIV(3'd0), .I(sys_clk_div2), .O(sys_clk_bufg));

  assign pclk_o = phy_pclk;

  // -------------------------------------------------------------------------
  // PIPE between u_rc and u_pg239
  // -------------------------------------------------------------------------
  // Where the widths differ: the IP's phy_txdata and phy_rxdata are 64 bits,
  // of which Gen1 uses [15:0] (PG239, Table 5 and Table 7); its
  // phy_rxstart_block is 2 bits, of which u_rc takes bit 0; and only bits
  // [1:0] of u_rc's 3-bit phy_rate are connected to the IP.
  wire [(MAX_NUM_LANES*PHY_DATA_WIDTH)-1:0]   phy_txdata;
  wire [MAX_NUM_LANES-1:0]                    phy_txdata_valid;
  wire [(MAX_NUM_LANES*PHY_DATA_WIDTH/8)-1:0] phy_txdatak;
  wire [MAX_NUM_LANES-1:0]                    phy_txstart_block;
  wire [(2*MAX_NUM_LANES)-1:0]                phy_txsync_header;
  wire [63:0]                                 pg_rxdata;          // IP width; [15:0] used
  wire [(MAX_NUM_LANES*PHY_DATA_WIDTH/8)-1:0] phy_rxdatak;
  wire [MAX_NUM_LANES-1:0]                    phy_rxdata_valid;
  wire [1:0]                                  pg_rxstart_block;   // IP width; [0] used
  wire [(2*MAX_NUM_LANES)-1:0]                phy_rxsync_header;

  wire                                        phy_txdetectrx;
  wire [MAX_NUM_LANES-1:0]                    phy_txelecidle;
  wire [MAX_NUM_LANES-1:0]                    phy_txcompliance;
  wire [MAX_NUM_LANES-1:0]                    phy_rxpolarity;
  wire [1:0]                                  phy_powerdown;
  wire [2:0]                                  phy_rate;           // 3 bits here; [1:0] go to the IP
  wire [MAX_NUM_LANES-1:0]                    phy_rxvalid;
  wire [MAX_NUM_LANES-1:0]                    phy_phystatus;
  wire                                        phy_phystatus_rst;
  wire [MAX_NUM_LANES-1:0]                    phy_rxelecidle;
  wire [(MAX_NUM_LANES*3)-1:0]                phy_rxstatus;
  wire [2:0]                                  phy_txmargin;
  wire                                        phy_txdeemph;

  wire [5:0]                                  phy_txeq_fs;
  wire [5:0]                                  phy_txeq_lf;
  wire [(MAX_NUM_LANES*18)-1:0]               phy_txeq_new_coeff;
  wire [MAX_NUM_LANES-1:0]                    phy_txeq_done;
  wire [MAX_NUM_LANES-1:0]                    phy_rxeq_preset_sel;
  wire [(MAX_NUM_LANES*18)-1:0]               phy_rxeq_new_txcoeff;
  wire [MAX_NUM_LANES-1:0]                    phy_rxeq_adapt_done;
  wire [MAX_NUM_LANES-1:0]                    phy_rxeq_done;

  wire                                        as_mac_in_detect;
  wire                                        as_cdr_hold_req;

  // -------------------------------------------------------------------------
  // Reset of u_rc
  // -------------------------------------------------------------------------
  // rc_rst_req is PERST# low or phy_phystatus_rst high; phy_phystatus_rst is
  // high from reset until the PHY and GT resets complete (PG239, Table 10).
  // u_rc_rst_sync, an xpm_cdc_async_rst with DEST_SYNC_FF = RST_SYNC_STAGES,
  // takes the request as src_arst and drives rc_rst with phy_pclk as
  // dest_clk. In Vivado 2023.2's XPM it asserts its output asynchronously and
  // releases it DEST_SYNC_FF dest_clk edges after src_arst falls: rc_rst is
  // high whenever the request is, so u_rc stays in reset until PG239 is
  // ready, and rc_rst comes from a synchroniser flop, not a gate on two
  // unsynchronised signals. rc_rst is the only reset u_rc sees, on rst_i and
  // on phy_phystatus_rst; inside u_rc it reaches asynchronous resets
  // (pcie_datalink_init, async_fifo, axis_async_fifo) and synchronous ones.

  // At least 3: axis_async_fifo's reset synchroniser in phy_receive is three
  // flops deep (s_rst_sync1_reg to s_rst_sync3_reg), all on phy_pclk here.
  // In Vivado 2023.2's XPM, the last RST_SYNC_STAGES edges before release
  // follow the request's fall, so they are running edges even if phy_pclk
  // stopped while the request was high.
  localparam int RST_SYNC_STAGES = 4;

  wire rc_rst_req;   // ~PERST# | phy_phystatus_rst: into u_rc_rst_sync only
  wire rc_rst;       // what u_rc sees, on rst_i and on phy_phystatus_rst

  assign rc_rst_req = ~sys_rst_n | phy_phystatus_rst;

  // Vivado 2023.2's XPM attaches a scoped constraint to xpm_cdc_async_rst, a
  // false path through src_arst, so pcie_rc_gth_zcu102.xdc needs no line for
  // this input.
  xpm_cdc_async_rst #(
      .DEST_SYNC_FF   (RST_SYNC_STAGES),
      .INIT_SYNC_FF   (0),
      .RST_ACTIVE_HIGH(1)
  ) u_rc_rst_sync (
      .src_arst (rc_rst_req),
      .dest_clk (phy_pclk),
      .dest_arst(rc_rst)
  );

  // -------------------------------------------------------------------------
  // Root Complex
  // -------------------------------------------------------------------------
  // pcie_rc_top, clocked by phy_pclk on all three of its clocks. tx_elec_idle
  // and phy_ready_en are tied 0, and u_rc's phy_txswing, pipe_width_o and
  // equalization outputs are left open.
  pcie_rc_top #(
      .AXIS_DATA_WIDTH   (AXIS_DATA_WIDTH),
      .AXIS_KEEP_WIDTH   (AXIS_KEEP_WIDTH),
      .AXIS_USER_WIDTH   (AXIS_USER_WIDTH),
      .CONTEXT_WIDTH     (CONTEXT_WIDTH),
      .TAG_COUNT         (TAG_COUNT),
      .CPL_TIMEOUT_CYCLES(CPL_TIMEOUT_CYCLES),
      .CRS_RETRY_MAX     (CRS_RETRY_MAX),
      .CRS_BACKOFF_CYCLES(CRS_BACKOFF_CYCLES),
      .CFG_HOLD_CYCLES   (CFG_HOLD_CYCLES),
      .CRS_WINDOW_CYCLES (CRS_WINDOW_CYCLES),
      .CQ_USER_WIDTH     (CQ_USER_WIDTH),
      .CC_USER_WIDTH     (CC_USER_WIDTH),
      .CLK_PERIOD_NS     (8),              // phy_pclk, 125 MHz at Gen1
      .MAX_NUM_LANES     (MAX_NUM_LANES),
      .PHY_DATA_WIDTH    (PHY_DATA_WIDTH),
      .PHY_USER_WIDTH    (PHY_USER_WIDTH),
      .IS_ROOT_PORT      (IS_ROOT_PORT),
      .LINK_NUM          (LINK_NUM),
      .SIM_FAST_LINK     (SIM_FAST_LINK)
  ) u_rc (
      .clk_i            (phy_pclk),
      .rst_i            (rc_rst),          // see Reset of u_rc
      .en_i             (en_i),
      .pipe_rx_usr_clk_i(phy_pclk),
      .pipe_tx_usr_clk_i(phy_pclk),
      .transmit_enable_i(transmit_enable_i),

      .phy_txdata       (phy_txdata),
      .phy_txdata_valid (phy_txdata_valid),
      .phy_txdatak      (phy_txdatak),
      .phy_txstart_block(phy_txstart_block),
      .phy_txsync_header(phy_txsync_header),
      .phy_rxdata       (pg_rxdata[15:0]),
      .phy_rxdata_valid (phy_rxdata_valid),
      .phy_rxdatak      (phy_rxdatak),
      .phy_rxstart_block(pg_rxstart_block[0]),
      .phy_rxsync_header(phy_rxsync_header),

      .phy_txdetectrx  (phy_txdetectrx),
      .phy_txelecidle  (phy_txelecidle),
      .phy_txcompliance(phy_txcompliance),
      .phy_rxpolarity  (phy_rxpolarity),
      .phy_powerdown   (phy_powerdown),
      .phy_rate        (phy_rate),

      .phy_rxvalid      (phy_rxvalid),
      .phy_phystatus    (phy_phystatus),
      // rc_rst rather than the IP's phy_phystatus_rst: inside u_rc this port
      // only resets state. pcie_phy_top ORs it with rst_i into resets that
      // include asynchronous presets, and the LTSSM clears lane_active on it.
      // So rst_i and this port both come from u_rc_rst_sync, whose request
      // includes phy_phystatus_rst.
      .phy_phystatus_rst(rc_rst),          // see the comment above
      .phy_rxelecidle   (phy_rxelecidle),
      .phy_rxstatus     (phy_rxstatus),

      .phy_txmargin(phy_txmargin),
      .phy_txswing (),                     // no driver in u_rc; the IP input is tied
      .phy_txdeemph(phy_txdeemph),
      .pipe_width_o(),

      .phy_txeq_ctrl       (),             // no driver in u_rc; tied at the IP
      .phy_txeq_preset     (),
      .phy_txeq_coeff      (),
      .phy_txeq_fs         (phy_txeq_fs),
      .phy_txeq_lf         (phy_txeq_lf),
      .phy_txeq_new_coeff  (phy_txeq_new_coeff),
      .phy_txeq_done       (phy_txeq_done),
      .phy_rxeq_ctrl       (),
      .phy_rxeq_txpreset   (),
      .phy_rxeq_preset_sel (phy_rxeq_preset_sel),
      .phy_rxeq_new_txcoeff(phy_rxeq_new_txcoeff),
      .phy_rxeq_adapt_done (phy_rxeq_adapt_done),
      .phy_rxeq_done       (phy_rxeq_done),

      .tx_elec_idle     (1'b0),            // read nowhere in pcie_phy_top
      .phy_ready_en     (1'b0),            // read nowhere in pcie_phy_top
      .as_mac_in_detect (as_mac_in_detect),
      .as_cdr_hold_req  (as_cdr_hold_req),
      .ltssm_debug_state(ltssm_debug_state),

      .link_up_o                   (link_up_o),
      .fc_initialized_o            (fc_initialized_o),
      .fc_init_done_o              (fc_init_done_o),
      .ok_to_issue_o               (ok_to_issue_o),
      .requester_id_i              (requester_id_i),
      .completer_id_i              (completer_id_i),
      .bus_number_i                (bus_number_i),
      .device_number_i             (device_number_i),
      .function_number_i           (function_number_i),
      .memory_enable_i             (memory_enable_i),
      .extended_tag_enable_i       (extended_tag_enable_i),
      .max_payload_bytes_i         (max_payload_bytes_i),
      .max_read_bytes_i            (max_read_bytes_i),
      .rcb_128b_i                  (rcb_128b_i),
      .cfg_bus_number_o            (cfg_bus_number_o),
      .cfg_device_number_o         (cfg_device_number_o),
      .cfg_function_number_o       (cfg_function_number_o),
      .scan_start_i                (scan_start_i),
      .scan_bus_i                  (scan_bus_i),
      .bar_enable_i                (bar_enable_i),
      .bridge_enable_i             (bridge_enable_i),
      .scan_busy_o                 (scan_busy_o),
      .scan_done_o                 (scan_done_o),
      .scan_error_o                (scan_error_o),
      .scan_error_code_o           (scan_error_code_o),
      .err_credit_blocked_o        (err_credit_blocked_o),
      .device_present_o            (device_present_o),
      .unsupported_device_o        (unsupported_device_o),
      .device_bdf_o                (device_bdf_o),
      .vendor_id_o                 (vendor_id_o),
      .device_id_o                 (device_id_o),
      .header_type_o               (header_type_o),
      .multifunction_o             (multifunction_o),
      .bar_busy_o                  (bar_busy_o),
      .enum_done_o                 (enum_done_o),
      .enum_error_o                (enum_error_o),
      .enum_error_code_o           (enum_error_code_o),
      .bar_count_o                 (bar_count_o),
      .bar_valid_o                 (bar_valid_o),
      .bar_is_64_o                 (bar_is_64_o),
      .bar_prefetch_o              (bar_prefetch_o),
      .bar_size_o                  (bar_size_o),
      .bar_addr_o                  (bar_addr_o),
      .io_bar_mask_o               (io_bar_mask_o),
      .bus_done_o                  (bus_done_o),
      .bus_bypassed_o              (bus_bypassed_o),
      .sec_scan_done_o             (sec_scan_done_o),
      .sec_device_present_o        (sec_device_present_o),
      .sec_unsupported_device_o    (sec_unsupported_device_o),
      .sec_device_bdf_o            (sec_device_bdf_o),
      .sec_vendor_id_o             (sec_vendor_id_o),
      .sec_device_id_o             (sec_device_id_o),
      .sec_header_type_o           (sec_header_type_o),
      .sec_multifunction_o         (sec_multifunction_o),
      .sec_enum_done_o             (sec_enum_done_o),
      .sec_bar_count_o             (sec_bar_count_o),
      .sec_bar_valid_o             (sec_bar_valid_o),
      .sec_bar_is_64_o             (sec_bar_is_64_o),
      .sec_bar_prefetch_o          (sec_bar_prefetch_o),
      .sec_bar_size_o              (sec_bar_size_o),
      .sec_bar_addr_o              (sec_bar_addr_o),
      .sec_io_bar_mask_o           (sec_io_bar_mask_o),
      .s_axis_rq_tdata             (s_axis_rq_tdata),
      .s_axis_rq_tkeep             (s_axis_rq_tkeep),
      .s_axis_rq_tvalid            (s_axis_rq_tvalid),
      .s_axis_rq_tlast             (s_axis_rq_tlast),
      .s_axis_rq_tuser             (s_axis_rq_tuser),
      .s_axis_rq_tready            (s_axis_rq_tready),
      .m_axis_rc_tdata             (m_axis_rc_tdata),
      .m_axis_rc_tkeep             (m_axis_rc_tkeep),
      .m_axis_rc_tvalid            (m_axis_rc_tvalid),
      .m_axis_rc_tlast             (m_axis_rc_tlast),
      .m_axis_rc_tready            (m_axis_rc_tready),
      .pcie_rq_tag_o               (pcie_rq_tag_o),
      .pcie_rq_tag_vld_o           (pcie_rq_tag_vld_o),
      .rq_engine_owns_o            (rq_engine_owns_o),
      .m_axis_cq_tdata             (m_axis_cq_tdata),
      .m_axis_cq_tkeep             (m_axis_cq_tkeep),
      .m_axis_cq_tvalid            (m_axis_cq_tvalid),
      .m_axis_cq_tlast             (m_axis_cq_tlast),
      .m_axis_cq_tuser             (m_axis_cq_tuser),
      .m_axis_cq_tready            (m_axis_cq_tready),
      .s_axis_cc_tdata             (s_axis_cc_tdata),
      .s_axis_cc_tkeep             (s_axis_cc_tkeep),
      .s_axis_cc_tvalid            (s_axis_cc_tvalid),
      .s_axis_cc_tlast             (s_axis_cc_tlast),
      .s_axis_cc_tuser             (s_axis_cc_tuser),
      .s_axis_cc_tready            (s_axis_cc_tready),
      .cq_dropped_o                (cq_dropped_o),
      .cq_error_code_o             (cq_error_code_o),
      .cq_gearbox_error_o          (cq_gearbox_error_o),
      .cc_protocol_error_o         (cc_protocol_error_o),
      .cc_error_code_o             (cc_error_code_o),
      .cc_gearbox_error_o          (cc_gearbox_error_o),
      .rq_protocol_error_o         (rq_protocol_error_o),
      .rq_error_code_o             (rq_error_code_o),
      .rq_gearbox_error_o          (rq_gearbox_error_o),
      .rc_unexpected_completion_o  (rc_unexpected_completion_o),
      .rc_completion_error_code_o  (rc_completion_error_code_o),
      .rc_protocol_error_o         (rc_protocol_error_o),
      .rc_error_code_o             (rc_error_code_o),
      .rc_gearbox_error_o          (rc_gearbox_error_o),
      .command_error_valid_o       (command_error_valid_o),
      .command_error_code_o        (command_error_code_o),
      .malformed_o                 (malformed_o),
      .rx_error_valid_o            (rx_error_valid_o),
      .rx_error_code_o             (rx_error_code_o),
      .rx_ecrc_error_o             (rx_ecrc_error_o),
      .tx_error_valid_o            (tx_error_valid_o),
      .tx_error_code_o             (tx_error_code_o),
      .tx_fc_blocked_o             (tx_fc_blocked_o),
      .credit_error_o              (credit_error_o),
      .vc_overflow_o               (vc_overflow_o),
      .cpl_timeout_valid_o         (cpl_timeout_valid_o),
      .cpl_timeout_tag_o           (cpl_timeout_tag_o),
      .late_cpl_valid_o            (late_cpl_valid_o),
      .late_cpl_tag_o              (late_cpl_tag_o),
      .outstanding_o               (outstanding_o)
  );

  // -------------------------------------------------------------------------
  // PG239 PHY
  // -------------------------------------------------------------------------
  // pg239_gen1_x1, generated by fpga/zcu102/ip_pg239.tcl. as_mac_in_detect
  // and as_cdr_hold_req come from u_rc: pcie_phy_top raises as_mac_in_detect
  // in the LTSSM's Detect states and ties as_cdr_hold_req to 0 (PG239,
  // Table 14). Inputs u_rc does not drive are tied to their idle or default
  // values.
  pg239_gen1_x1 u_pg239 (
      .phy_refclk        (sys_clk_bufg),
      .phy_gtrefclk      (sys_clk_gt),
      .phy_rst_n         (sys_rst_n),
      .phy_pclk          (phy_pclk),
      .phy_coreclk       (),
      .phy_userclk       (),
      .phy_mcapclk       (),

      .phy_txdata        ({48'd0, phy_txdata}),   // [63:16] unused at Gen1 (PG239, Table 5)
      .phy_txdatak       (phy_txdatak),
      .phy_txdata_valid  (phy_txdata_valid),
      .phy_txstart_block (phy_txstart_block),
      .phy_txsync_header (phy_txsync_header),
      .phy_rxp           (pci_exp_rxp),
      .phy_rxn           (pci_exp_rxn),
      .phy_txp           (pci_exp_txp),
      .phy_txn           (pci_exp_txn),
      .phy_rxdata        (pg_rxdata),
      .phy_rxdatak       (phy_rxdatak),
      .phy_rxdata_valid  (phy_rxdata_valid),
      .phy_rxstart_block (pg_rxstart_block),
      .phy_rxsync_header (phy_rxsync_header),

      .phy_txdetectrx    (phy_txdetectrx),
      .phy_txelecidle    (phy_txelecidle),
      .phy_txcompliance  (phy_txcompliance),
      .phy_rxpolarity    (phy_rxpolarity),
      .phy_powerdown     (phy_powerdown),
      .phy_rate          (phy_rate[1:0]),         // Gen1 is 0 (PG239, Table 9)
      .phy_rxvalid       (phy_rxvalid),
      .phy_phystatus     (phy_phystatus),
      .phy_phystatus_rst (phy_phystatus_rst),
      .phy_rxelecidle    (phy_rxelecidle),
      .phy_rxstatus      (phy_rxstatus),

      .phy_txmargin      (phy_txmargin),
      .phy_txswing       (1'b0),                  // full swing, the default (PG239, Table 11)
      .phy_txdeemph      (phy_txdeemph),

      .phy_txeq_ctrl     (2'b00),                 // idle (PG239, Table 12)
      .phy_txeq_preset   (4'd0),                  // read only if txeq_ctrl = 01b (PG239, Table 12)
      .phy_txeq_coeff    (6'd0),                  // read only if txeq_ctrl = 10b (PG239, Table 12)
      .phy_txeq_fs       (phy_txeq_fs),
      .phy_txeq_lf       (phy_txeq_lf),
      .phy_txeq_new_coeff(phy_txeq_new_coeff),
      .phy_txeq_done     (phy_txeq_done),
      .phy_rxeq_ctrl     (2'b00),                 // idle (PG239, Table 13)
      .phy_rxeq_txpreset (4'd0),                  // Gen3 and Gen4 only (PG239, Table 13)
      .phy_rxeq_preset_sel (phy_rxeq_preset_sel),
      .phy_rxeq_new_txcoeff(phy_rxeq_new_txcoeff),
      .phy_rxeq_adapt_done (phy_rxeq_adapt_done),
      .phy_rxeq_done       (phy_rxeq_done),

      .as_mac_in_detect  (as_mac_in_detect),
      .as_cdr_hold_req   (as_cdr_hold_req),
      .gt_gtpowergood    (gt_gtpowergood)
  );

  // -------------------------------------------------------------------------
  // Debug taps
  // -------------------------------------------------------------------------
  // Lane 0 of PIPE command, status and assist wires between u_rc and
  // u_pg239, plus the IP's own phy_phystatus_rst (not rc_rst) and
  // gt_gtpowergood.
  assign dbg_phy_phystatus_o     = phy_phystatus[0];
  assign dbg_phy_phystatus_rst_o = phy_phystatus_rst;
  assign dbg_phy_rxstatus_o      = phy_rxstatus;
  assign dbg_phy_rxvalid_o       = phy_rxvalid[0];
  assign dbg_phy_rxelecidle_o    = phy_rxelecidle[0];
  assign dbg_as_mac_in_detect_o  = as_mac_in_detect;
  assign dbg_phy_txdetectrx_o    = phy_txdetectrx;
  assign dbg_phy_txelecidle_o    = phy_txelecidle[0];
  assign dbg_phy_powerdown_o     = phy_powerdown;
  assign dbg_gt_gtpowergood_o    = gt_gtpowergood[0];

endmodule
