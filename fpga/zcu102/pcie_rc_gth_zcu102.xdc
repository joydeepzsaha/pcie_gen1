# ---------------------------------------------------------------------------
# pcie_rc_gth_zcu102.xdc -- pins, clocks and crossings for pcie_rc_gth_zcu102
#
# Author: Kourosh Ghahramani
# Silicon Systems Research Lab, University of Washington
#
# Purpose
#   Constrains the board top pcie_rc_gth_zcu102 on the ZCU102
#   (xczu9eg-ffvb1156-2-e). The debug hub's clock is set in
#   pcie_rc_gth_zcu102_debug.xdc.
#
# Contents
#   Pins       the GTH lane and its reference clock on FMC HPC1, and the
#              CLK_125 debug clock.
#   Clocks     sys_clk, the 100 MHz reference clock; pclk, PG239's PCLK,
#              which clocks the RC in u_rc_gth; clk125, the debug clock.
#   Crossings  the clock crossings of pcie_rc_gth_zcu102.sv: each signal into
#              clk125 is bounded, and every path from PERST# is cut.
#
# Usage
#   Read before synth_design: Vivado 2023.2 synthesis is timing-driven by
#   default and uses the clocks defined here.
#
# References
#   PG239, Table 4: Clock and Reset Signals
#   PG239, Clock Frequencies
#   UG1182, Table 3-12: ZCU102 Board Clock Sources
#   UG1182, Table 3-13: Clock Connections, Source to XCZU9EG MPSoC
#   UG1182, Table 3-37: ZCU102 GTH Bank 130 Interface Connections
#   UG576, Table 5-1: GTH Transceiver Quad Pin Descriptions
# ---------------------------------------------------------------------------

# ---------------------------------------------------------------------------
# Pins
# ---------------------------------------------------------------------------
# Locations are from UG1182: Table 3-37 for the lane and its reference clock,
# Table 3-13 for CLK_125. The comment after each PACKAGE_PIN gives the device
# pin name and FMC HPC1 (J4) pin of a GTH pin, or the net name and SI5341B
# (U69) pin of CLK_125. The ZCU102 board file in Vivado 2023.2 (zcu102,
# version 3.4) gives the same locations. GT pins take no IOSTANDARD: the I/O
# standard does not apply to MGT connections (UG1182, Table 3-37, note 2),
# although that board file lists LVCMOS18 for them.

# PCIe lane 0: FMC HPC1 DP0 (nets FMC_HPC1_DP0_C2M_P/N, FMC_HPC1_DP0_M2C_P/N)
# on GTH bank 130, channel 0. In the PG239 IP that Vivado 2023.2 generates,
# the GT core's XDC sets its GTH channel's LOC to GTHE4_CHANNEL_X0Y12, the
# lane0_gt_location that ip_pg239.tcl asserts. Each GTH channel has its own
# serial pad pairs (UG576, Table 5-1), so these pins must agree with that LOC.
set_property PACKAGE_PIN F29 [get_ports {pci_exp_txp[0]}] ;# MGTHTXP0_130  J4.C2
set_property PACKAGE_PIN F30 [get_ports {pci_exp_txn[0]}] ;# MGTHTXN0_130  J4.C3
set_property PACKAGE_PIN E31 [get_ports {pci_exp_rxp[0]}] ;# MGTHRXP0_130  J4.C6
set_property PACKAGE_PIN E32 [get_ports {pci_exp_rxn[0]}] ;# MGTHRXN0_130  J4.C7

# The PCIe reference clock: FMC HPC1 GBTCLK0_M2C (nets
# FMC_HPC1_GBTCLK0_M2C_C_P/N) into MGTREFCLK0 of bank 130, series capacitor
# coupled on the board (UG1182, Table 3-37, note 1). ip_pg239.tcl does not set
# refclk1_location, and stops with an error unless the IP's value is
# Bank_130_MGTREFCLK0.
set_property PACKAGE_PIN G27 [get_ports sys_clk_p]        ;# MGTREFCLK0P_130  J4.D4
set_property PACKAGE_PIN G28 [get_ports sys_clk_n]        ;# MGTREFCLK0N_130  J4.D5

# The debug clock: CLK_125, one of the board's fixed-frequency clocks, from
# the SI5341B clock generator (UG1182, Table 3-12). It does not come from the
# FMC card, so it runs whether or not the adapter supplies a reference clock.
# It clocks the logic that must keep running while PG239 is in reset, and not
# the RC (pcie_rc_gth_zcu102.sv, Debug clock).
set_property PACKAGE_PIN G21 [get_ports clk125_p]         ;# CLK_125_P  U69.45
set_property PACKAGE_PIN F21 [get_ports clk125_n]         ;# CLK_125_N  U69.44
set_property IOSTANDARD LVDS_25 [get_ports {clk125_p clk125_n}]   ;# UG1182, Table 3-13

# ---------------------------------------------------------------------------
# Clocks
# ---------------------------------------------------------------------------
# sys_clk and clk125 are primary clocks on input ports; pclk is the clock at
# the output of PG239's bufg_gt_pclk, derived from sys_clk. The XDC of the
# PG239 IP that Vivado 2023.2 generates also creates intclk as a primary
# clock of 1000 ns on its bufg_gt_intclk, so timing analysis does not derive
# intclk from sys_clk, although that buffer is fed from TXOUTCLK as
# bufg_gt_pclk is.

# The PCIe reference clock, 100 MHz: PG239's default (PG239, Table 4) and the
# phy_refclk_freq that ip_pg239.tcl asserts. The name and period are those of
# PG239's example constraint (PG239, Clock Frequencies).
create_clock -name sys_clk -period 10.000 [get_ports sys_clk_p]

# pclk: PG239's phy_pclk, 125 MHz at Gen1 (PG239, Table 4), derived from
# sys_clk through TXOUTCLK and bufg_gt_pclk, whose DIV the PG239 IP's XDC
# fixes at 001b, a divide by 2 (Vivado 2023.2, BUFG_GT simulation model).
# The commands below refer to it as pclk. The create_generated_clock help
# gives this form, -name and a pin only, for renaming a clock derived at an
# MMCM, PLL or BUFR, and does not list BUFG_GT (Vivado 2023.2,
# create_generated_clock).
create_generated_clock -name pclk [get_pins {u_rc_gth/u_pg239/inst/diablo_gt.diablo_gt_phy_wrapper/phy_clk_i/bufg_gt_pclk/O}]

# -setup only: clock uncertainty is otherwise used in hold analysis as well
# (Vivado 2023.2, set_clock_uncertainty).
set_clock_uncertainty -setup 0.100 [get_clocks pclk]

# The debug clock, CLK_125 at 125 MHz (UG1182, Table 3-12).
create_clock -name clk125 -period 8.000 [get_ports clk125_p]

# No set_clock_groups here. The PG239 IP's late XDC (Vivado 2023.2), scoped
# to the IP, declares phy_refclk asynchronous to pclk and to intclk. That
# covers the pclk outputs of u_rc that the IP re-times on phy_refclk:
# as_mac_in_detect (sync_mac_detect) and phy_rate (sync_phy_rate). u_rc has
# no clock other than pclk.
#
# No set_input_delay or set_output_delay: the board top's only ports are the
# GT lane, the GT reference clock and CLK_125, so it has no fabric data pins.

# ---------------------------------------------------------------------------
# Crossings
# ---------------------------------------------------------------------------
# The clock crossings of pcie_rc_gth_zcu102.sv. Without a timing exception,
# Vivado times a crossing as a synchronous path. The synchroniser inputs are
# bounded rather than grouped, because set_clock_groups and set_false_path
# take precedence over set_max_delay on the same path (Vivado 2023.2, Timing
# Constraints Wizard help, Asynchronous Clock Domain Crossings). Each signal
# into clk125 enters a two-flop ASYNC_REG synchroniser and is bounded at one
# clk125 period, 8 ns, with -datapath_only, which excludes clock skew and
# jitter and makes the hold check a false path (Vivado 2023.2,
# set_max_delay). The one signal out of clk125, PERST#, is cut.

# pclk to clk125: the PCLK-gap witness's pclk_div[1] into tick_meta.
set_max_delay -datapath_only -from [get_cells {pclk_div_reg[1]}] -to [get_cells tick_meta_reg] 8.000

# pclk to clk125: link_up and phy_phystatus_rst into fs_meta[2] and
# fs_meta[1]. The input of fs_meta[0] comes from intclk and is bounded below.
set_max_delay -datapath_only -from [get_clocks pclk] -to [get_cells {fs_meta_reg[*]}] 8.000

# intclk to clk125: gt_gtpowergood into fs_meta[0]. In the PG239 IP that
# Vivado 2023.2 generates, gt_gtpowergood is not the GT's GTPOWERGOOD but the
# inverse of txpisopd_r, a register of the reset module's power-on state
# machine on intclk, so the bound starts at that register.
set_max_delay -datapath_only \
    -from [get_cells {u_rc_gth/u_pg239/inst/diablo_gt.diablo_gt_phy_wrapper/phy_rst_i/txpisopd_r_reg}] \
    -to   [get_cells {fs_meta_reg[0]}] 8.000

# clk125 to the rest: every path from sys_rst_n_r, PERST# for u_rc_gth, is
# cut, its paths to vio_free and ila_free included, as Vivado 2023.2's PG239
# example design cuts every path from its sys_rst_n port. Its loads outside
# clk125 re-time it (pcie_rc_gth_zcu102.sv, Power-on reset and PERST#), and
# Vivado 2023.2's XPM adds a false path through u_rc_rst_sync's src_arst.
set_false_path -from [get_cells sys_rst_n_r_reg]
