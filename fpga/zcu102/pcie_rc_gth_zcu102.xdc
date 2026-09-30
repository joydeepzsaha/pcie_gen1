# ---------------------------------------------------------------------------
# pcie_rc_gth_zcu102.xdc -- pcie_rc_gth_top on the ZCU102 (xczu9eg-ffvb1156-2-e)
# sec 63 #5 (the GTH rung), sub-rung 8-3.  Read BEFORE synth_design, so that
# synthesis is timing-driven.  The debug hub's clock is set after synthesis,
# in pcie_rc_gth_zcu102_debug.xdc, because the hub does not exist before it.
#
# Every pin cites its UG1182 (v1.7) row as "Table, page (txt line)", with
# txt = pcie_docs/book/ug1182-zcu102-eval-bd.txt.  Each row was cross-checked
# against the board file zcu102/3.4/part0_pins.xml (BF) in
# evidence/ug1182-recon/FMC_GTH_MAP.md.  ⚠️ GT pins carry NO IOSTANDARD
# (Table 3-37 note 2: "MGT connections I/O standard not applicable").  BF's
# LVCMOS18 on GT rows is a placeholder and is not copied.
# ---------------------------------------------------------------------------

# ===== PINS ================================================================

# --- PCIe lane 0 = FMC HPC1 (J4) DP0 = GTH Quad 130 channel 0 --------------
# The channel's LOC comes from PG239's own XDC (GTHE4_CHANNEL_X0Y12).  These
# PACKAGE_PINs are a second key on the same fact: if the IP's derivation and
# the board table ever disagree, placement fails instead of routing a lane to
# the wrong pins.
set_property PACKAGE_PIN F29 [get_ports {pci_exp_txp[0]}] ;# MGTHTXP0_130  FMC_HPC1_DP0_C2M_P  J4.C2   Table 3-37 p.87 (4107)
set_property PACKAGE_PIN F30 [get_ports {pci_exp_txn[0]}] ;# MGTHTXN0_130  FMC_HPC1_DP0_C2M_N  J4.C3   Table 3-37 p.87 (4108)
set_property PACKAGE_PIN E31 [get_ports {pci_exp_rxp[0]}] ;# MGTHRXP0_130  FMC_HPC1_DP0_M2C_P  J4.C6   Table 3-37 p.87 (4109)
set_property PACKAGE_PIN E32 [get_ports {pci_exp_rxn[0]}] ;# MGTHRXN0_130  FMC_HPC1_DP0_M2C_N  J4.C7   Table 3-37 p.87 (4110)

# --- the PCIe refclk: FMC HPC1 GBTCLK0_M2C -> MGTREFCLK0_130 (A5) ----------
# Series capacitor coupled on the board (Table 3-37 note 1).  PG239 derives
# refclk1_location = Bank_130_MGTREFCLK0 from the lane-0 bank alone
# (PHASE0_8-2.md sec 3); ip_pg239.tcl asserts it.
set_property PACKAGE_PIN G27 [get_ports sys_clk_p]        ;# MGTREFCLK0P_130  FMC_HPC1_GBTCLK0_M2C_C_P  J4.D4  Table 3-37 p.87 (4124)
set_property PACKAGE_PIN G28 [get_ports sys_clk_n]        ;# MGTREFCLK0N_130  FMC_HPC1_GBTCLK0_M2C_C_N  J4.D5  Table 3-37 p.87 (4125)

# --- the debug clock: CLK_125, SI5341B U69, fixed 125 MHz -------------------
# Not a design clock.  It clocks the debug hub, vio_free, ila_free, the POR and
# the PCLK-gap witness (pcie_rc_gth_zcu102.sv, CLOCKS).  UG1182 Table 3-12
# p.44 lists it as a fixed-frequency clock: no I2C and no board setup, so it
# runs whether or not the FMC adapter supplies a refclk.
set_property PACKAGE_PIN G21 [get_ports clk125_p]         ;# CLK_125_P  U69.45  Table 3-13 p.44 (2151); BF part0_pins.xml:14
set_property PACKAGE_PIN F21 [get_ports clk125_n]         ;# CLK_125_N  U69.44  Table 3-13 p.44 (2152); BF part0_pins.xml:15
set_property IOSTANDARD LVDS_25 [get_ports {clk125_p clk125_n}]   ;# Table 3-13 "I/O Standard" column; BF agrees

# ===== CLOCKS ==============================================================

# The PCIe refclk, 100 MHz (PG239 p.11 "100 MHz (default)"; the IP's
# phy_refclk_freq).  PG239's own example design names it sys_clk at 10 ns
# (xilinx_pcie_phy.xdc).  Vivado derives everything inside PG239 from it:
# TXOUTCLK through the CPLL, and PCLK = TXOUTCLK / 2 at bufg_gt_pclk (DIV is
# fixed by the IP's set_case_analysis).
create_clock -name sys_clk -period 10.000 [get_ports sys_clk_p]

# ⭐ PCLK -- THE ONE DESIGN CLOCK (D-7B.1).  125 MHz at Gen1 (PG239 p.11).  The
# auto-derived clock is renamed and not redefined: Vivado keeps its waveform,
# and this line only names it, so the uncertainty below and every report can
# refer to it.
create_generated_clock -name pclk [get_pins {u_rc_gth/u_pg239/inst/diablo_gt.diablo_gt_phy_wrapper/phy_clk_i/bufg_gt_pclk/O}]

# The #7b-adopted uncertainty (xdc_D.xdc, D-7B.4): -setup ONLY, because a bare
# set_clock_uncertainty also applies to hold and makes 0.100 ns of every hold
# number a constraint artifact.
set_clock_uncertainty -setup 0.100 [get_clocks pclk]

# The debug clock.
create_clock -name clk125 -period 8.000 [get_ports clk125_p]

# ⚠️ NO set_clock_groups IN THIS FILE (D-7B.1).  PG239's own
# pg239_gen1_x1_late.xdc carries four, all between the IP's phy_refclk and its
# pclk / intclk.  They are AMD's constraints on AMD's internal crossings
# (e.g. the as_mac_in_detect synchroniser on phy_refclk), they are scoped to
# the IP instance, and they are kept and reported, not overridden.  No path of
# ours lies between those clocks: u_rc is entirely on pclk.
#
# ⚠️ No placeholder I/O delays.  #7b's I/O model constrained an out-of-context
# unit's FABRIC ports.  This top has none: its only pins are GT serial pins,
# GT refclk pins and a clock.  The whole fabric surface is on chip (the
# board top's header).

# ===== CROSSINGS (all of them; pcie_rc_gth_zcu102.sv, CLOCKS) ================

# pclk -> clk125: the gap witness's divider bit into its synchroniser.  The
# bound is one clk125 period of datapath, clock skew excluded.  The path is
# analysed, not cut.
set_max_delay -datapath_only -from [get_cells {pclk_div_reg[1]}] -to [get_cells tick_meta_reg] 8.000

# pclk -> clk125: link_up and phy_phystatus_rst into their synchronisers.
set_max_delay -datapath_only -from [get_clocks pclk] -to [get_cells {fs_meta_reg[*]}] 8.000

# intclk -> clk125: gt_gtpowergood into its synchroniser (8-3 R1).  PG239's
# gt_gtpowergood port is NOT the GT's GTPOWERGOOD: it is DBG_GTPOWERGOOD =
# !rst_txpisopd (..._gt_phy_wrapper.v:1089), the inverse of the reset FSM's
# txpisopd_r register (..._gt_phy_rst.v:152), clocked by the IP's intclk.
# The routed path is txpisopd_r_reg (FDSE) -> PG239's LUT1 inverter ->
# fs_meta_reg[0] -> fs_sync_reg[0], both clk125 flops ASYNC_REG.  It is bound
# cell to cell, the narrowest form, and not clock to clock.  Unbounded, it was
# timed as synchronous between unrelated clocks (TIMING-6 / TIMING-7).
set_max_delay -datapath_only \
    -from [get_cells {u_rc_gth/u_pg239/inst/diablo_gt.diablo_gt_phy_wrapper/phy_rst_i/txpisopd_r_reg}] \
    -to   [get_cells {fs_meta_reg[0]}] 8.000

# clk125 -> everything: sys_rst_n (PERST#).  This is PG239's example design's
# `set_false_path -from [get_ports sys_rst_n]`, with our register in place of
# its port.  sys_rst_n_r reaches two things, and both re-time it
# (pcie_rc_gth_zcu102.sv, RESET): PG239's phy_rst_n, synchronised inside the IP
# (rst_n_internal_i, on its refclk; 8-3 named rst_psrst_n_r, which is the
# phy_phystatus_rst synchroniser on pclk), and the RC's reset request, which
# feeds only u_rc_rst_sync (xpm_cdc_async_rst, G0).  That synchroniser's input
# carries XPM's own scoped false path through src_arst, so no line for it is
# added here.
set_false_path -from [get_cells sys_rst_n_r_reg]
