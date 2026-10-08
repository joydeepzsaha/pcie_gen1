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
#   make_ibert sets eleven parameters and checks them; make_ibert_example
#   opens the IP's example design and adds the one constraint it needs on
#   this board (the debug hub's clock divider off).
#
# Usage
#   In Vivado 2023.2, with a current project for the ZCU102's part
#   (xczu9eg-ffvb1156-2-e): source ip_ibert.tcl ; set ip [make_ibert <dir>]
#   create_ip writes the IP into dir, which make_ibert creates, and adds it
#   to the current project. Each value read back is printed as one line,
#   IBERT|<property>|<value>. make_ibert returns the IP. The caller generates
#   it (generate_target all), then calls make_ibert_example <ip> <ex_dir>,
#   which writes the example design into ex_dir, closes the current project,
#   leaves the example project open and returns its .xpr path. The example
#   design carries the pins; no generated file is committed.
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
#   Vivado 2023.2, Debug Hub IP xsdbm v3.0 (C_ENABLE_CLK_DIVIDER)
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

# Opens ip's example design in ex_dir (open_example_project) and adds one
# constraint file to it, read after the example's own XDC (implementation
# only, PROCESSING_ORDER LATE): the debug hub's clock divider off. The
# example's XDC turns it on (C_ENABLE_CLK_DIVIDER true), and the divider's
# MMCM cannot then be paired with the system clock's input: CLK_125 enters on
# G21/F21, HDGC pins of HD bank 47, and Vivado 2023.2 stops in placement
# (Place 30-681, "Sub-optimal placement for a global clock-capable IO pin and
# MMCM pair"). Without the divider the hub runs straight from CLK_125 at
# 125 MHz, as in pcie_rc_gth_zcu102 (pcie_rc_gth_zcu102_debug.xdc). Closes
# the current project, leaves the example project open, and returns its .xpr.
proc make_ibert_example {ip ex_dir} {
  open_example_project -force -dir $ex_dir $ip
  close_project
  set xpr [glob $ex_dir/*/*.xpr]
  open_project $xpr
  set f [file join [file dirname $xpr] ibert_dp5_dbg_hub.xdc]
  set fh [open $f w]
  puts $fh "# make_ibert_example (fpga/zcu102/ip_ibert.tcl): the debug hub runs from CLK_125"
  puts $fh "# at 125 MHz without its clock divider, whose MMCM cannot be paired with"
  puts $fh "# CLK_125's HD-bank pin (Place 30-681). Read after the example's XDC."
  puts $fh "set_property C_ENABLE_CLK_DIVIDER false \[get_debug_cores dbg_hub\]"
  close $fh
  add_files -fileset constrs_1 -norecurse $f
  set_property USED_IN {implementation} [get_files $f]
  set_property PROCESSING_ORDER LATE [get_files $f]
  puts "IBERT|example|$xpr|constraint=[file tail $f]|used_in=[get_property USED_IN [get_files $f]]|order=[get_property PROCESSING_ORDER [get_files $f]]"
  return $xpr
}
