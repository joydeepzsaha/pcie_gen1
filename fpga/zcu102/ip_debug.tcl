# ---------------------------------------------------------------------------
# ip_debug.tcl -- creates the four debug cores of pcie_rc_gth_zcu102
#
# Author: Kourosh Ghahramani
# Silicon Systems Research Lab, University of Washington
#
# Purpose
#   Defines make_debug_ips, which creates and configures the two ILA v6.2
#   and two VIO v3.0 cores that pcie_rc_gth_zcu102 instantiates:
#     ila_pclk  on pclk    the LTSSM state, link_up, fc_initialized and the
#                          PIPE taps, with a PCLK timestamp and a store strobe
#     vio_pclk  on pclk    the RC's runtime controls out, its status in
#     ila_free  on clk125  the PCLK-gap witness
#     vio_free  on clk125  PERST# and the gap-statistics clear out, the gap
#                          statistics and free_status in
#   The caller generates the cores' output products; no generated file is
#   committed.
#
# Usage
#   In Vivado 2023.2, with a current project for the ZCU102's part
#   (xczu9eg-ffvb1156-2-e): source ip_debug.tcl ; make_debug_ips <dir>
#   create_ip writes the cores into dir, which make_debug_ips creates, and
#   adds each core to the current project.
#
# Limitations
#   Each probe width here must equal the width of the signal that
#   pcie_rc_gth_zcu102.sv connects to that probe; nothing here checks it.
#
# References
#   ILA v6.2 and VIO v3.0 customization parameters, Vivado 2023.2
# ---------------------------------------------------------------------------

# Creates ILA v6.2 core name in dir, depth samples deep, with one probe per
# element of widths, at that width.
proc _ila {dir name depth widths} {
  create_ip -name ila -vendor xilinx.com -library ip -version 6.2 -module_name $name -dir $dir
  set ip [get_ips $name]
  # C_EN_STRG_QUAL 1 enables Capture Control, which lets a capture condition select the samples
  # stored. The capture condition always uses one comparator per probe, so each probe needs
  # at least 2, the count ALL_PROBE_SAME_MU_CNT sets (ILA v6.2 in Vivado 2023.2).
  set d [list CONFIG.C_NUM_OF_PROBES [llength $widths] CONFIG.C_DATA_DEPTH $depth \
             CONFIG.C_EN_STRG_QUAL 1 CONFIG.C_ADV_TRIGGER true CONFIG.ALL_PROBE_SAME_MU_CNT 2 \
             CONFIG.C_INPUT_PIPE_STAGES 0]
  set i 0
  foreach w $widths { lappend d CONFIG.C_PROBE${i}_WIDTH $w ; incr i }
  set_property -dict $d $ip
}

# Creates VIO v3.0 core name in dir with one input probe per element of in_widths and one
# output probe per element of out_widths, each at that width. Each output's initial value
# is the matching element of out_inits, in hex.
proc _vio {dir name in_widths out_widths out_inits} {
  create_ip -name vio -vendor xilinx.com -library ip -version 3.0 -module_name $name -dir $dir
  set ip [get_ips $name]
  set d [list CONFIG.C_NUM_PROBE_IN [llength $in_widths] CONFIG.C_NUM_PROBE_OUT [llength $out_widths] \
             CONFIG.C_EN_PROBE_IN_ACTIVITY 1]
  set i 0
  foreach w $in_widths { lappend d CONFIG.C_PROBE_IN${i}_WIDTH $w ; incr i }
  set i 0
  foreach w $out_widths v $out_inits {
    lappend d CONFIG.C_PROBE_OUT${i}_WIDTH $w CONFIG.C_PROBE_OUT${i}_INIT_VAL $v
    incr i
  }
  set_property -dict $d $ip
}

# Creates ila_pclk, ila_free, vio_free and vio_pclk in dir. Probe numbers and signal names
# below are the connections in pcie_rc_gth_zcu102.sv, "Debug cores".
proc make_debug_ips {dir} {
  file mkdir $dir
  # ila_pclk probes: 0 ltssm_debug_state[20:0], 1 link_up, 2 fc_initialized, 3 dbg_phystatus,
  # 4 dbg_phystatus_rst, 5 dbg_rxstatus[2:0], 6 dbg_rxvalid, 7 dbg_rxelecidle,
  # 8 dbg_as_mac_in_detect, 9 dbg_txdetectrx, 10 dbg_txelecidle, 11 dbg_powerdown[1:0],
  # 12 pclk_ts[39:0], 13 ila_store. In the Hardware Manager, set its capture mode to BASIC and
  # its capture condition to probe13 == 1 (pcie_rc_gth_zcu102.sv, "ila_pclk capture control").
  _ila $dir ila_pclk 8192 {21 1 1 1 1 3 1 1 1 1 1 2 40 1}
  # ila_free probes: 0 gap_cnt[15:0], 1 tick_sync, 2 free_status[4:0]
  # (pcie_rc_gth_zcu102.sv, "PCLK-gap witness").
  _ila $dir ila_free 8192 {16 1 5}
  # vio_free inputs: 0 gap_max[15:0], 1 gap_events[15:0], 2 free_status[4:0]. Outputs:
  # 0 vio_perst_n and 1 vio_gap_clear, both starting at 0. With vio_perst_n at 0, PERST#
  # stays asserted after configuration until the VIO console writes 1
  # (pcie_rc_gth_zcu102.sv, "Power-on reset and PERST#").
  _vio $dir vio_free {16 16 5} {1 1} {0x0 0x0}
  # vio_pclk inputs 0 to 30: the RC's status, grouped as in pcie_rc_gth_zcu102.sv,
  # "vio_pclk probe map". Each 384-bit BAR size or address bus takes a 256-bit and a
  # 128-bit probe, because a VIO v3.0 input probe is at most 256 bits wide (Vivado 2023.2).
  # Outputs: 0 vio_en, 1 vio_transmit_enable, 2 vio_scan_start, 3 vio_scan_bus[7:0],
  # 4 vio_bar_enable, 5 vio_bridge_enable. They start at the constants tb_pcie_rc_gth
  # drives on the same RC inputs: 1, 1, 0, 00h, 0 and 0.
  _vio $dir vio_pclk \
    {4 16 10 4 4 16 16 16 8 4 24 256 128 256 128 7 16 16 16 8 4 24 256 128 256 128 10 4 19 36 22} \
    {1 1 1 8 1 1} {0x1 0x1 0x0 0x00 0x0 0x0}
}
