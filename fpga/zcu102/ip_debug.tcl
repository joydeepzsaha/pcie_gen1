# ---------------------------------------------------------------------------
# ip_debug.tcl -- the four debug cores of pcie_rc_gth_zcu102 (sec 63 #5, 8-3).
#
#   source ip_debug.tcl ; make_debug_ips <dir>      (a part must be set)
#
# Configuration only: create_ip + set_property.  Generation happens in the
# caller's run directory; nothing generated is committed.  Probe widths are
# the board top's; a mismatch is an elaboration error there, not a silent
# truncation, because every probe is connected at its full declared width.
#
#   ila_pclk  on pclk  : LTSSM state, link_up, the PIPE status the brief names
#   vio_pclk  on pclk  : the RC's runtime controls out, its status in
#   ila_free  on clk125: the PCLK-gap witness (HANDSHAKE sec 7)
#   vio_free  on clk125: PERST#, and the gap statistics
# ---------------------------------------------------------------------------

proc _ila {dir name depth widths} {
  create_ip -name ila -vendor xilinx.com -library ip -version 6.2 -module_name $name -dir $dir
  set ip [get_ips $name]
  set d [list CONFIG.C_NUM_OF_PROBES [llength $widths] CONFIG.C_DATA_DEPTH $depth \
             CONFIG.C_EN_STRG_QUAL 1 CONFIG.C_ADV_TRIGGER true CONFIG.ALL_PROBE_SAME_MU_CNT 2 \
             CONFIG.C_INPUT_PIPE_STAGES 0]
  set i 0
  foreach w $widths { lappend d CONFIG.C_PROBE${i}_WIDTH $w ; incr i }
  set_property -dict $d $ip
}

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

proc make_debug_ips {dir} {
  file mkdir $dir
  # ila_pclk: probe0 ltssm_debug_state[20:0], 1 link_up, 2 fc_initialized, 3 phystatus,
  # 4 phystatus_rst, 5 rxstatus[2:0], 6 rxvalid, 7 rxelecidle, 8 as_mac_in_detect,
  # 9 txdetectrx, 10 txelecidle, 11 powerdown[1:0],
  # 12 pclk_ts[39:0], 13 ila_store -- G0 A-dbg: depth 8192, capture condition probe13 == 1
  # (the board top's ILA_PCLK CAPTURE CONTROL)
  _ila $dir ila_pclk 8192 {21 1 1 1 1 3 1 1 1 1 1 2 40 1}
  # ila_free: probe0 gap_cnt[15:0], 1 pclk_tick, 2 free_status[4:0]
  _ila $dir ila_free 8192 {16 1 5}
  # vio_free: in0 gap_max[15:0], in1 gap_events[15:0], in2 free_status[4:0];
  #           out0 perst_n (INIT 0, G0 A-dbg: PERST# is held after configuration until
  #           the VIO writes 1; bitstream A had INIT 1), out1 gap_clear (INIT 0)
  _vio $dir vio_free {16 16 5} {1 1} {0x0 0x0}
  # vio_pclk: in0..in30 the RC's status groups (pcie_rc_gth_zcu102.sv, "vio_pclk probe map");
  #           out0 en, out1 transmit_enable, out2 scan_start, out3 scan_bus[7:0],
  #           out4 bar_enable, out5 bridge_enable -- INIT = tb_pcie_rc_gth.sv's values
  _vio $dir vio_pclk \
    {4 16 10 4 4 16 16 16 8 4 24 256 128 256 128 7 16 16 16 8 4 24 256 128 256 128 10 4 19 36 22} \
    {1 1 1 8 1 1} {0x1 0x1 0x0 0x00 0x0 0x0}
}
