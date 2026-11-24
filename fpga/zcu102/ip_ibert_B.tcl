# ip_ibert_B.tcl -- IBERT-B (LR2 group C close, Kourosh's approval 2026-11-19): fpga/zcu102/ip_ibert.tcl at 70fc59c (= b9ddd27,
# 09b3a7f8) with the reference clock changed and nothing else: C_PROTOCOL_REFCLK_FREQUENCY_1 100 -> 156.25 and
# C_REFCLK_SOURCE_QUAD_2 MGTREFCLK0_130 -> MGTREFCLK0_129, in the set and in the check (four lines). The refclk is the ZCU102's
# own USER_MGT_SI570_CLOCK1 (U56 Si570 through the U51 SI53340 buffer) on L27/L28 = MGTREFCLK0P/N_129, 156.250 MHz at power-up
# (UG1182 v1.7 Table 3-36 p.86 and note 3; "Programmable User MGT Clock" p.48), in X0Y9's own quad. The comments below are the
# A recipe's, unchanged: where they say 100 MHz / bank 130 / GBTCLK0, read 156.25 MHz / bank 129 / the Si570.
# ---------------------------------------------------------------------------
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
#   GBTCLK0_M2C, G27/G28) and CLK_125 (G21/F21), divided to 100 MHz by an
#   MMCM, as its system clock: the lane,
#   the reference clock and the line rate of pcie_rc_gth_zcu102. Its example
#   design is a standalone bitstream for checking the lane in the Hardware
#   Manager (PLL lock, near-end loopback, eye scan); it drives no PERST#.
#   make_ibert sets eleven parameters and checks them; make_ibert_example
#   opens the IP's example design and adapts it to this board: an MMCM gives
#   the IBERT a 100 MHz system clock (its CPLL calibration assumes 100 MHz),
#   the system clock's pin keeps CLK_125's period and a relaxed route to the
#   MMCM, and the debug hub's clock divider is off.
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
      CONFIG.C_PROTOCOL_REFCLK_FREQUENCY_1 156.25
  } $ip
  set_property -dict {
      CONFIG.C_PROTOCOL_QUAD1          None
      CONFIG.C_PROTOCOL_QUAD2          Custom_1_/_2.5_Gbps
      CONFIG.C_REFCLK_SOURCE_QUAD_2    MGTREFCLK0_129
  } $ip
  # The system clock: CLK_125, 125 MHz, LVDS_25 on G21/F21 (UG1182, Tables
  # 3-12 and 3-13), the board's free-running clock, divided to 100 MHz by an
  # MMCM in make_ibert_example, so C_SYSCLK_FREQUENCY is 100: the frequency
  # the IBERT logic runs at, which its CPLL calibration assumes.
  set_property -dict {
      CONFIG.C_SYSCLK_FREQUENCY         100
      CONFIG.C_SYSCLK_IO_PIN_STD        LVDS_25
      CONFIG.C_SYSCLK_IO_PIN_LOC_P      G21
      CONFIG.C_SYSCLK_IO_PIN_LOC_N      F21
      CONFIG.C_SYSCLK_IS_DIFF           1
  } $ip
  foreach {p want} {
      CONFIG.C_PROTOCOL_MAXLINERATE_1      2.5
      CONFIG.C_PROTOCOL_PLL_1              CPLL
      CONFIG.C_PROTOCOL_REFCLK_FREQUENCY_1 156.25
      CONFIG.C_PROTOCOL_QUAD1              None
      CONFIG.C_PROTOCOL_QUAD2              Custom_1_/_2.5_Gbps
      CONFIG.C_REFCLK_SOURCE_QUAD_2        MGTREFCLK0_129
      CONFIG.C_SYSCLK_FREQUENCY            100
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

# Opens ip's example design in ex_dir (open_example_project) and adapts it to
# this board with files of the recipe's own; no generated file is edited.
#
# 1. The IBERT's system clock is 100 MHz, made from CLK_125 by an MMCM. The
#    IBERT's CPLL calibration counts TXOUTCLK / 4 over 16,000 cycles of the
#    system (DRP) clock and accepts 5000 +/- 50: constants the IP generates
#    for a 100 MHz clock whatever C_SYSCLK_FREQUENCY says (pcie_docs
#    evidence/bringup/IBERT_B_CPLL_CAL.md). Fed CLK_125 directly it counts
#    about 4000, and every channel fails its calibration, as IBERT-B did on
#    the board on 2026-11-24. The example's top is copied with an MMCM
#    (125 MHz x 8 / 10 = 100 MHz, VCO 1.0 GHz) and a BUFGCE, enabled by the
#    MMCM's LOCKED, between the system clock's input buffer and the IBERT's
#    clk; make_ibert sets C_SYSCLK_FREQUENCY to 100 to match.
# 2. The example's XDC is copied with two changes. D_CLK, the clock on the
#    system clock's port, is 8.000 ns, CLK_125's period (the generated XDC
#    derives it from C_SYSCLK_FREQUENCY, now the MMCM's output frequency).
#    CLOCK_DEDICATED_ROUTE FALSE on the input buffer's output: CLK_125 enters
#    on G21/F21, HDGC pins of HD bank 47, which have no clock management tile
#    of their own, and Vivado 2023.2 cannot pair such a pin with an MMCM on a
#    dedicated route (Place 30-681, met by the debug hub's divider MMCM in
#    the first IBERT build of section 63 #23). General routing adds jitter to
#    the MMCM's input only; its 100 MHz output clocks the IBERT's DRP and
#    debug logic, not a serial lane.
# 3. A constraint file read after the example's XDC, implementation only:
#    the debug hub's clock divider off (its MMCM would meet the same
#    Place 30-681).
# Each copy asserts that its one substitution matched exactly once. The
# generated files are disabled in the example project, not edited. Closes the
# current project, leaves the example project open, and returns its .xpr.
proc make_ibert_example {ip ex_dir} {
  open_example_project -force -dir $ex_dir $ip
  close_project
  set xpr [glob $ex_dir/*/*.xpr]
  open_project $xpr
  set dir [file dirname $xpr]
  set top [get_property top [current_fileset]]
  # 1. the top, with the MMCM
  set gen_v [get_files -of_objects [get_filesets sources_1] "*/${top}.v"]
  if {[llength $gen_v] != 1} { error "make_ibert_example: ${top}.v found [llength $gen_v] times" }
  set fh [open $gen_v r] ; set src [read $fh] ; close $fh
  if {[regsub -all {\.clk\(gth_sysclk_i\)} $src {.clk(gth_sysclk_100)} src] != 1} {
    error "make_ibert_example: .clk(gth_sysclk_i) is not in $gen_v exactly once"
  }
  if {[llength [regexp -all -inline -indices {\n[ \t]*[A-Za-z_0-9]+[ \t]+u_ibert_gth_core\M} $src]] != 1} {
    error "make_ibert_example: the IBERT instance u_ibert_gth_core is not in $gen_v exactly once"
  }
  regexp -indices {\n[ \t]*[A-Za-z_0-9]+[ \t]+u_ibert_gth_core\M} $src at
  set mmcm {
  // make_ibert_example (fpga/zcu102/ip_ibert*.tcl): the IBERT's system clock
  // at 100 MHz from CLK_125 (125 MHz x 8 / 10, VCO 1.0 GHz). The BUFGCE holds
  // the clock off until the MMCM has locked.
  wire gth_sysclk_100_mmcm, gth_sysclk_100, gth_sysclk_mmcm_fb, gth_sysclk_mmcm_locked;
  MMCME4_BASE #(
      .CLKIN1_PERIOD   (8.000),
      .DIVCLK_DIVIDE   (1),
      .CLKFBOUT_MULT_F (8.000),
      .CLKOUT0_DIVIDE_F(10.000)
  ) u_sysclk_mmcm (
      .CLKIN1  (gth_sysclk_i),
      .CLKFBIN (gth_sysclk_mmcm_fb),
      .CLKFBOUT(gth_sysclk_mmcm_fb),
      .CLKOUT0 (gth_sysclk_100_mmcm),
      .LOCKED  (gth_sysclk_mmcm_locked),
      .PWRDWN  (1'b0),
      .RST     (1'b0)
  );
  BUFGCE u_sysclk_100_bufg (.I(gth_sysclk_100_mmcm), .CE(gth_sysclk_mmcm_locked), .O(gth_sysclk_100));
}
  set src [string replace $src [lindex $at 0] [lindex $at 0] "\n$mmcm"]
  set new_v [file join $dir ${top}_sysclk100.v]
  set fh [open $new_v w] ; puts -nonewline $fh $src ; close $fh
  add_files -fileset sources_1 -norecurse $new_v
  set_property IS_ENABLED false [get_files $gen_v]
  set_property top $top [current_fileset]
  # 2. the example's XDC, with D_CLK at 8.000 ns and the relaxed route
  set gen_x [get_files -of_objects [get_filesets constrs_1] "*/${top}.xdc"]
  if {[llength $gen_x] != 1} { error "make_ibert_example: ${top}.xdc found [llength $gen_x] times" }
  set fh [open $gen_x r] ; set xdc [read $fh] ; close $fh
  if {[regsub -all {create_clock -name D_CLK -period [0-9.]+ } $xdc {create_clock -name D_CLK -period 8.000 } xdc] != 1} {
    error "make_ibert_example: the D_CLK create_clock is not in $gen_x exactly once"
  }
  append xdc "\n# make_ibert_example (fpga/zcu102/ip_ibert*.tcl): D_CLK above is CLK_125, 8.000 ns, whatever\n"
  append xdc "# C_SYSCLK_FREQUENCY says (100, the MMCM's output). G21/F21 are HDGC pins of HD bank 47, with no\n"
  append xdc "# MMCM of their own: a dedicated route to the MMCM fails placement (Place 30-681). General routing adds\n"
  append xdc "# jitter to the MMCM's input only; its 100 MHz output clocks the IBERT's DRP and debug logic, not a lane.\n"
  append xdc "set_property CLOCK_DEDICATED_ROUTE FALSE \[get_nets gth_sysclk_i\]\n"
  set new_x [file join $dir ${top}_sysclk100.xdc]
  set fh [open $new_x w] ; puts -nonewline $fh $xdc ; close $fh
  add_files -fileset constrs_1 -norecurse $new_x
  set_property IS_ENABLED false [get_files $gen_x]
  # 3. the debug hub without its clock divider, read late, implementation only
  set f [file join $dir ibert_dp5_dbg_hub.xdc]
  set fh [open $f w]
  puts $fh "# make_ibert_example (fpga/zcu102/ip_ibert*.tcl): the debug hub without its clock divider, whose"
  puts $fh "# MMCM cannot be paired with CLK_125's HD-bank pin (Place 30-681). Read after the example's XDC."
  puts $fh "set_property C_ENABLE_CLK_DIVIDER false \[get_debug_cores dbg_hub\]"
  close $fh
  add_files -fileset constrs_1 -norecurse $f
  set_property USED_IN {implementation} [get_files $f]
  set_property PROCESSING_ORDER LATE [get_files $f]
  puts "IBERT|example|$xpr|top=$top|top_file=[file tail $new_v]|xdc=[file tail $new_x]|disabled=[file tail $gen_v],[file tail $gen_x]|hub_constraint=[file tail $f]"
  return $xpr
}
