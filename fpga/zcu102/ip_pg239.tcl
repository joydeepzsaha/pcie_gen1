# ---------------------------------------------------------------------------
# ip_pg239.tcl -- creates pg239_gen1_x1, the PG239 PHY of pcie_rc_gth_top
#
# Author: Kourosh Ghahramani
# Silicon Systems Research Lab, University of Washington
#
# Purpose
#   Defines make_pg239, which creates and configures pg239_gen1_x1, the
#   customization of the AMD PCIe PHY IP (pcie_phy v1.0, PG239) that
#   pcie_rc_gth_top instantiates as u_pg239: one lane at 2.5 GT/s on channel
#   1 of GTH bank 129 (FMC HPC1 DP5), with a 100 MHz reference clock. It sets
#   five parameters and checks seven values. The defaults and generated values
#   named below are those of pcie_phy v1.0 in Vivado 2023.2.
#
# Usage
#   In Vivado 2023.2, with a current project for the ZCU102's part
#   (xczu9eg-ffvb1156-2-e): source ip_pg239.tcl ; make_pg239 <dir>
#   create_ip writes the IP into dir, which make_pg239 creates, and adds it
#   to the current project. The caller generates its output products; no
#   generated IP file is committed. Each value read back is printed as one
#   line, PG239|<property>|<value>. make_pg239 returns the IP.
#
# Limitations
#   phy_async_en false costs RX elastic-buffer latency: CLK_COR_MIN_LAT, the
#   buffer's minimum latency, is 17 instead of 10 (UG576, Table 4-37). The
#   generated GT Wizard core also differs in RXBUF_THRESH_UNDFLW, 8 instead
#   of 1, PCIE3_CLK_COR_MIN_LAT and MAX_LAT, 4 and 8 instead of 0 and 4, and
#   one bit each of RXCDR_CFG2, RXCDR_CFG2_GEN2 and PCIE_TXPCS_CFG_GEN3.
#   The reference clock the IP is configured with is not the one the board
#   uses (Reference clock, below); no generated file reads it.
#
# References
#   PG239, Table 4: Clock and Reset Signals
#   PG239, Basic Tab
#   PG239, Table 18: PLL Type
#   PG239, GT Selection Tab
#   PG239, Advanced Settings Tab
#   UG1182, Table 3-36: ZCU102 GTH Bank 129 Interface Connections
#   UG1182, Table 3-37: ZCU102 GTH Bank 130 Interface Connections
#   UG576, Single External Reference Clock Use Model
#   UG576, Table 4-37: RX Clock Correction Attributes
# ---------------------------------------------------------------------------

proc make_pg239 {dir} {
  file mkdir $dir
  create_ip -name pcie_phy -vendor xilinx.com -library ip -version 1.0 \
            -module_name pg239_gen1_x1 -dir $dir
  set ip [get_ips pg239_gen1_x1]
  # These five are the only parameters set here, in this order; Vivado
  # derives or defaults the rest.
  # Gen1, the rate pcie_rc_gth_top runs at; the IP's default is 8.0_GT/s.
  set_property CONFIG.phy_max_speed 2.5_GT/s     $ip
  # The GUI value is the inverse of the HDL parameter PHY_ASYNC_EN (pcie_phy
  # v1.0, Vivado 2023.2). false sets PHY_REFCLK_MODE 1: reference clocks
  # without SSC up to 600 ppm apart, 0 ppm included. true, the default, sets
  # mode 0: one common reference clock, 0 ppm (PG239, Advanced Settings Tab).
  set_property CONFIG.phy_async_en  false        $ip
  # Channel 1 of GTH bank 129, GTHE4_CHANNEL_X0Y9, is FMC HPC1 DP5 (UG1182,
  # Table 3-36), where the HiTech Global HTG-FMC-PCIE-RC puts its slot's lane
  # 0. Unset, lane0_gt_location is channel 0 of the bank, X0Y8 (DP4); the IP
  # accepts X0Y8 to X0Y11 for bank 129.
  set_property CONFIG.lane0_gt_bank     GTH_Quad_129       $ip
  set_property CONFIG.lane0_gt_location GTHE4_CHANNEL_X0Y9 $ip
  # Reference clock: the board's is FMC HPC1 GBTCLK0_M2C on MGTREFCLK0 of
  # bank 130 (G27/G28), where pcie_rc_gth_zcu102.xdc places pcie_rc_gth_top's
  # IBUFDS_GTE4. The IP accepts only bank 129's two reference clocks for lane
  # 0 in bank 129, but its generated logic and constraints do not read the
  # value: the generated GT always takes the clock on GTREFCLK0 with
  # CPLLREFCLKSEL 001, and Vivado 2023.2 routes it from bank 130 over the
  # south reference-clock routing (UG576, Single External Reference Clock Use
  # Model). So the IP's value, Bank_129_MGTREFCLK0, is set here only so that
  # it is fixed, and is checked below.
  set_property CONFIG.refclk1_location  Bank_129_MGTREFCLK0 $ip

  # pll_type is fixed at CPLL for Gen1 (PG239, Table 18). The lane count and
  # the 100 MHz reference clock are the ones pcie_rc_gth_top and
  # pcie_rc_gth_zcu102.xdc assume. A value that differs stops the script here,
  # before synthesis.
  foreach {p want} {
      CONFIG.lane0_gt_location GTHE4_CHANNEL_X0Y9
      CONFIG.refclk1_location  Bank_129_MGTREFCLK0
      CONFIG.phy_lane          X1
      CONFIG.phy_refclk_freq   100_MHz
      CONFIG.pll_type          CPLL
      CONFIG.phy_async_en      false
      CONFIG.phy_max_speed     2.5_GT/s
  } {
    set got [get_property $p $ip]
    if {$got ne $want} { error "make_pg239: $p = '$got', expected '$want'" }
    puts "PG239|$p|$got"
  }
  return $ip
}
