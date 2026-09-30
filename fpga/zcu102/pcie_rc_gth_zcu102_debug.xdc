# ---------------------------------------------------------------------------
# pcie_rc_gth_zcu102_debug.xdc -- the debug hub.  Read AFTER synth_design,
# because the hub does not exist before it.  sec 63 #5, 8-3.
#
# The hub runs on clk125 and never on pclk.  PCLK stops through PG239's reset
# (HANDSHAKE sec 7), and a hub on a stopped clock drops every core off the JTAG
# chain, including vio_free, the only core that can release PERST#.
# ---------------------------------------------------------------------------
set_property C_CLK_INPUT_FREQ_HZ 125000000 [get_debug_cores dbg_hub]
set_property C_ENABLE_CLK_DIVIDER false     [get_debug_cores dbg_hub]
set_property C_USER_SCAN_CHAIN 1            [get_debug_cores dbg_hub]
connect_debug_port dbg_hub/clk [get_nets clk125]
