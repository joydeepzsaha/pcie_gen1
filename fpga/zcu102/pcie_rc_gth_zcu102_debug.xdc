# ---------------------------------------------------------------------------
# pcie_rc_gth_zcu102_debug.xdc -- the debug hub's clock for pcie_rc_gth_zcu102
#
# Author: Kourosh Ghahramani
# Silicon Systems Research Lab, University of Washington
# Based on: example/debog_output.xdc at tag pre-cleanup (Idris Somoye)
#
# Purpose
#   Clocks the Vivado debug hub (dbg_hub) from clk125 and sets its input
#   clock frequency, clock divider and user scan chain. The hub connects the
#   JTAG boundary-scan interface to every debug core, including vio_free,
#   which releases PERST# (Vivado 2023.2, connect_debug_cores). The hub has
#   to be active from configuration on, so its clock should be free running
#   (Vivado 2023.2, Debug Hub IP xsdbm v3.0). clk125 is a fixed-frequency
#   board clock; PCLK stops while PERST# holds PG239 in reset
#   (pcie_rc_gth_zcu102.sv, Debug clock).
#
# Usage
#   Read after synth_design. dbg_hub is not in the RTL: Vivado adds it when
#   the design has ILA cores (Vivado 2023.2, get_debug_cores).
#
# References
#   UG1182, Table 3-12: ZCU102 Board Clock Sources
#   UG576, TX Programmable Divider
# ---------------------------------------------------------------------------

# 125 MHz is the frequency of clk125; the xsdbm v3.0 default is 300 MHz. The
# disabled clock divider and user scan chain 1 are the xsdbm v3.0 defaults in
# Vivado 2023.2.
set_property C_CLK_INPUT_FREQ_HZ 125000000 [get_debug_cores dbg_hub]
set_property C_ENABLE_CLK_DIVIDER false     [get_debug_cores dbg_hub]
set_property C_USER_SCAN_CHAIN 1            [get_debug_cores dbg_hub]
connect_debug_port dbg_hub/clk [get_nets clk125]
