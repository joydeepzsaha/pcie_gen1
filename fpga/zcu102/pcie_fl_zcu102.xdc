# ---------------------------------------------------------------------------
# pcie_fl_zcu102.xdc -- pins and timing constraints of pcie_fl_zcu102
#
# Author: Kourosh Ghahramani
# Silicon Systems Research Lab, University of Washington
#
# Purpose
#   Places the 125 MHz clock and the four LEDs of pcie_fl_zcu102 on the
#   ZCU102 (xczu9eg-ffvb1156-2-e) and constrains its one clock. The design
#   has no transceiver, no reference clock and no pin on an FMC connector.
#
# Usage
#   Read before synth_design. The debug hub's clock is in
#   pcie_rc_gth_zcu102_debug.xdc, read after synth_design; it connects the
#   hub to the net clk125, which this top also has.
#
# References
#   UG1182, Table 3-13: Clock Connections, Source to XCZU9EG U1
#   UG1182, Table 3-33: XCZU9EG U1 to GPIO Connections
#   UG1182, Table 3-2: I/O Voltage Rails
# ---------------------------------------------------------------------------

# CLK_125, from the SI5341B (U69): LVDS_25, fixed at 125 MHz.
set_property PACKAGE_PIN G21 [get_ports clk125_p]         ;# CLK_125_P  U69.45
set_property PACKAGE_PIN F21 [get_ports clk125_n]         ;# CLK_125_N  U69.44
set_property IOSTANDARD LVDS_25 [get_ports {clk125_p clk125_n}]
create_clock -name clk125 -period 8.000 [get_ports clk125_p]

# GPIO LEDs 0 to 3, active high, in bank 44 (3.3 V).
set_property PACKAGE_PIN AG14 [get_ports {led[0]}]        ;# GPIO_LED_0  DS38.2
set_property PACKAGE_PIN AF13 [get_ports {led[1]}]        ;# GPIO_LED_1  DS37.2
set_property PACKAGE_PIN AE13 [get_ports {led[2]}]        ;# GPIO_LED_2  DS39.2
set_property PACKAGE_PIN AJ14 [get_ports {led[3]}]        ;# GPIO_LED_3  DS40.2
set_property IOSTANDARD LVCMOS33 [get_ports {led[*]}]

# The LEDs are read by eye. The output delay gives each port a constraint,
# which keeps check_timing's no_output_delay at 0 (Vivado 2023.2 counts a
# port that has only a false path); the false path then removes the paths
# from timing.
set_output_delay -clock [get_clocks clk125] 0.000 [get_ports {led[*]}]
set_false_path -to [get_ports {led[*]}]
