# ---------------------------------------------------------------------------
# ip_pg239.tcl -- the PG239 (AMD PCIe PHY IP v1.0) instance pcie_rc_gth_top
# instantiates: pg239_gen1_x1.  sec 63 #5, 8-3.  A part must be set.
#
#   source ip_pg239.tcl ; make_pg239 <dir>
#
# Configuration only.  The IP is generated in the caller's run directory and
# nothing generated is committed.  It lives in this repository so that a
# netlist's IP configuration and its RTL are pinned by ONE commit (sec 22.90).
# 8-2's generator (pcie_docs evidence/gth-8/8-2/scripts/p0b_*.tcl) stays there
# as the record of what 8-2 simulated.
#
# The USER properties -- the complete non-default set:
#   phy_max_speed  2.5_GT/s      Gen1, the RC's only rate (PG239 p.30; default 8.0_GT/s)
#   lane0_gt_bank  GTH_Quad_130  FMC HPC1 DP0 (D-8.5; UG1182 Table 3-37 p.87)
#   phy_async_en   false         8-3's change from 8-2, below
#
# phy_async_en.  The GUI value is the INVERSE of the HDL parameter it sets,
# measured by generating both (PHASE0_8-3.md sec 4):
#   true  (the default; 8-2) -> PHY_ASYNC_EN("FALSE"): the common-clock
#         elastic buffer (CLK_COR_MIN_LAT 10, RXBUF_THRESH_UNDFLW 1)
#   false (8-3)              -> PHY_ASYNC_EN("TRUE"): separate-refclk margins
#         (CLK_COR_MIN_LAT 17, RXBUF_THRESH_UNDFLW 8, PCIE3_CLK_COR_MIN/MAX_LAT
#         4/8, and one RXCDR_CFG2 bit)
# PG239 p.34 gives RX_PPM_OFFSET 0 for common clock and 600 for SRNS.  8-3
# takes the separate-refclk form because the common-clock claim is unproven:
#   * A5's adapter (Opsero OP063) has "2x 100MHz oscillators", and whether the
#     FPGA's copy and the SSD's copy come from ONE oscillator is UNKNOWN
#     (ug1182-recon/ADAPTER_OP063.md sec 7 Q1).
#   * the Si570 fallback (MGTREFCLK0_129) is a separate refclk by construction.
# The separate-refclk buffer is correct at 0 ppm as well.  The common-clock
# buffer is correct only at 0 ppm.  The cost is RX elastic-buffer latency.
#
# Derived values are READ BACK and asserted.  A bank or refclk that moves is
# an error here, not a surprise in place_design.
# ---------------------------------------------------------------------------

proc make_pg239 {dir} {
  file mkdir $dir
  create_ip -name pcie_phy -vendor xilinx.com -library ip -version 1.0 \
            -module_name pg239_gen1_x1 -dir $dir
  set ip [get_ips pg239_gen1_x1]
  set_property CONFIG.lane0_gt_bank GTH_Quad_130 $ip
  set_property CONFIG.phy_max_speed 2.5_GT/s     $ip
  set_property CONFIG.phy_async_en  false        $ip

  foreach {p want} {
      CONFIG.lane0_gt_location GTHE4_CHANNEL_X0Y12
      CONFIG.refclk1_location  Bank_130_MGTREFCLK0
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
