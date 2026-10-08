# ---------------------------------------------------------------------------
# ip_ibert.tcl -- creates ibert_dp5, a GTH IBERT on the lane bitstream R uses
#
# Author: Kourosh Ghahramani
# Silicon Systems Research Lab, University of Washington
#
# Purpose
#   Defines make_ibert, which creates and configures ibert_dp5, a
#   customization of the AMD IBERT for UltraScale GTH transceivers
#   (ibert_ultrascale_gth v1.4): GTH bank 129, whose channel 1,
#   GTHE4_CHANNEL_X0Y9, is FMC HPC1 DP5, at 2.5 Gb/s on the channel PLL, with
#   the 100 MHz reference clock on MGTREFCLK0 of bank 130 (FMC HPC1
#   GBTCLK0_M2C, G27/G28) and CLK_125 (G21/F21) as its system clock: the lane,
#   the reference clock and the line rate of pcie_rc_gth_zcu102. Its example
#   design is a standalone bitstream for checking the lane in the Hardware
#   Manager (PLL lock, near-end loopback, eye scan); it drives no PERST#.
#   It sets eleven parameters and checks them.
#
# Usage
#   In Vivado 2023.2, with a current project for the ZCU102's part
#   (xczu9eg-ffvb1156-2-e): source ip_ibert.tcl ; make_ibert <dir>
#   create_ip writes the IP into dir, which make_ibert creates, and adds it
#   to the current project. The caller generates the IP and opens its example
#   design (open_example_project), which carries the pins; no generated file
#   is committed. Each value read back is printed as one line,
#   IBERT|<property>|<value>. make_ibert returns the IP.
#
# Limitations
#   The IP enables whole quads: all four channels of bank 129 are in the
#   design, not only X0Y9. Its quad index 2 is bank 129 on this part (index 1
#   is bank 128, whose reference-clock choices are banks 128 to 130).
#
# References
#   UG1182, Table 3-12: ZCU102 Board Clock Sources
#   UG1182, Table 3-13: Clock Connections, Source to XCZU9EG MPSoC
#   UG1182, Table 3-36: ZCU102 GTH Bank 129 Interface Connections
#   UG1182, Table 3-37: ZCU102 GTH Bank 130 Interface Connections
#   UG576, Single External Reference Clock Use Model
# ---------------------------------------------------------------------------

proc make_ibert {dir} {
  file mkdir $dir
  create_ip -name ibert_ultrascale_gth -vendor xilinx.com -library ip -version 1.4 \
            -module_name ibert_dp5 -dir $dir
  set ip [get_ips ibert_dp5]
  # One protocol: 2.5 Gb/s on the channel PLL (CPLL), as PG239 runs Gen1, with
  # a 100 MHz reference clock. Index 2 is bank 129 (Limitations); its
  # reference clock is bank 130's MGTREFCLK0, as in pcie_rc_gth_zcu102.xdc. The
  # quad and its reference clock are set in one call: set one at a time, the
  # IP refuses each intermediate state.
  set_property -dict {
      CONFIG.C_PROTOCOL_MAXLINERATE_1   2.5
      CONFIG.C_PROTOCOL_PLL_1           CPLL
      CONFIG.C_PROTOCOL_REFCLK_FREQUENCY_1 100
  } $ip
  set_property -dict {
      CONFIG.C_PROTOCOL_QUAD1          None
      CONFIG.C_PROTOCOL_QUAD2          Custom_1_/_2.5_Gbps
      CONFIG.C_REFCLK_SOURCE_QUAD_2    MGTREFCLK0_130
  } $ip
  # The system clock: CLK_125, 125 MHz, LVDS_25 on G21/F21 (UG1182, Tables
  # 3-12 and 3-13), the board's free-running clock.
  set_property -dict {
      CONFIG.C_SYSCLK_FREQUENCY         125
      CONFIG.C_SYSCLK_IO_PIN_STD        LVDS_25
      CONFIG.C_SYSCLK_IO_PIN_LOC_P      G21
      CONFIG.C_SYSCLK_IO_PIN_LOC_N      F21
      CONFIG.C_SYSCLK_IS_DIFF           1
  } $ip
  foreach {p want} {
      CONFIG.C_PROTOCOL_MAXLINERATE_1      2.5
      CONFIG.C_PROTOCOL_PLL_1              CPLL
      CONFIG.C_PROTOCOL_REFCLK_FREQUENCY_1 100
      CONFIG.C_PROTOCOL_QUAD1              None
      CONFIG.C_PROTOCOL_QUAD2              Custom_1_/_2.5_Gbps
      CONFIG.C_REFCLK_SOURCE_QUAD_2        MGTREFCLK0_130
      CONFIG.C_SYSCLK_FREQUENCY            125
      CONFIG.C_SYSCLK_IO_PIN_STD           LVDS_25
      CONFIG.C_SYSCLK_IO_PIN_LOC_P         G21
      CONFIG.C_SYSCLK_IO_PIN_LOC_N         F21
      CONFIG.C_SYSCLK_IS_DIFF              1
  } {
    set got [get_property $p $ip]
    if {$got ne $want} { error "make_ibert: $p = '$got', expected '$want'" }
    puts "IBERT|$p|$got"
  }
  return $ip
}
