# ---------------------------------------------------------------------------
# ip_debug_fl.tcl -- creates the three debug cores of pcie_fl_zcu102
#
# Author: Kourosh Ghahramani
# Silicon Systems Research Lab, University of Washington
#
# Purpose
#   Defines make_fl_debug_ips, which creates and configures the ILA v6.2 and
#   the two VIO v3.0 cores that pcie_fl_zcu102 instantiates, all on clk125:
#     ila_fl  both LTSSM states, both Data Link Layer states, the PIPE data
#             and K flags in both directions, the Symbols at the Endpoint,
#             the link's sideband, a timestamp and a store strobe
#     vio_fl  reset release, counter clear, automatic enumeration and the
#             link's fault injection out; link status, the Vendor and Device
#             ID read, the sticky counters and the event times in
#     vio_rc  the Root Complex's runtime controls out and its status in, with
#             the probe map of pcie_rc_gth_zcu102's vio_pclk
#   The caller generates the cores' output products; no generated file is
#   committed.
#
# Usage
#   In Vivado 2023.2, with a current project for the ZCU102's part
#   (xczu9eg-ffvb1156-2-e): source ip_debug_fl.tcl ; make_fl_debug_ips <dir>
#   This file sources ip_debug.tcl, in its own directory, for the _ila and
#   _vio procedures. create_ip writes the cores into dir, which
#   make_fl_debug_ips creates, and adds each core to the current project.
#   vio_fl's run starts at 0: the stacks stay in reset after configuration
#   until the VIO console writes 1.
#
# Limitations
#   Each probe width here must equal the width of the signal that
#   pcie_fl_zcu102.sv connects to that probe; nothing here checks it.
#
# References
#   ILA v6.2 and VIO v3.0 customization parameters, Vivado 2023.2
# ---------------------------------------------------------------------------

source [file join [file dirname [info script]] ip_debug.tcl]

# Creates ila_fl, vio_fl and vio_rc in dir. Probe numbers and signal names
# below are the connections in pcie_fl_zcu102.sv, "Debug cores".
proc make_fl_debug_ips {dir} {
  file mkdir $dir
  # ila_fl probes: 0 rc_ltssm[20:0], 1 ep_ltssm[19:0], 2 rc_dl_state[2:0], 3 ep_dl_state[2:0],
  # 4 link_levels[3:0], 5 rc_txdata[15:0], 6 rc_txdatak[1:0], 7 rc_txdata_valid, 8 rc_rxdata[15:0],
  # 9 rc_rxdatak[1:0], 10 rc_rxvalid, 11 ep_rx_symbol[19:0], 12 ep_rx_symbol_valid,
  # 13 ep_tx_symbol[19:0], 14 ep_tx_symbol_valid, 15 sideband[7:0], 16 events[7:0],
  # 17 ila_ts[39:0], 18 ila_store. For a whole bring-up, set the capture mode to BASIC and the
  # capture condition to probe18 == 1; for the PIPE data around an event, ALWAYS and a trigger
  # (pcie_fl_zcu102.sv, "ila_fl capture control").
  _ila $dir ila_fl 8192 {21 20 3 3 4 16 2 1 16 2 1 20 1 20 1 8 8 40 1}
  # vio_fl inputs: 0 fl_status[14:0], 1 vendor_id[15:0], 2 device_id[15:0], 3 rc_ltssm[20:0],
  # 4 ep_ltssm[19:0], 5 {ep_dl_state, rc_dl_state}[5:0]; 6 to 24 the sticky counters, 16 bits
  # each, in cnt_e's order: 6 reset releases, 7 and 8 link-ups (Root Complex, Endpoint), 9 and
  # 10 link-downs, 11 and 12 Recovery entries, 13 and 14 replays, 15 and 16 retrain requests,
  # 17 and 18 Naks received, 19 and 20 Naks scheduled, 21 and 22 the Root Complex direction's
  # code and disparity errors, 23 and 24 the Endpoint's; 25 t_cnt[31:0]; 26 to 31 the event
  # times, 32 bits each: link up (Root Complex, Endpoint), Flow Control initialized (Root
  # Complex, Endpoint), scan done, enumeration done; 32 {starve_ep_cnt, starve_cnt}[31:0], the
  # DLLPs each blackout corrupted since the reset; 33 inj_end_byte[7:0]. Outputs: 0 run,
  # starting at 0 (the stacks in reset); 1 clear, starting at 0; 2 auto_scan, starting at 1;
  # 3 inj_en, 4 inj_pkt[7:0], 5 inj_mode[1:0], 6 inj_off[7:0], 7 inj_bit[3:0], 8 starve_en,
  # 9 starve_ep_en, all starting at 0 (pcie_fl_zcu102.sv, "Fault injection controls").
  _vio $dir vio_fl \
    {15 16 16 21 20 6 16 16 16 16 16 16 16 16 16 16 16 16 16 16 16 16 16 16 16 32 32 32 32 32 32 32 32 8} \
    {1 1 1 1 8 2 8 4 1 1} {0x0 0x0 0x1 0x0 0x00 0x0 0x00 0x0 0x0 0x0}
  # vio_rc inputs 0 to 30: the Root Complex's status, with the widths and grouping of vio_pclk
  # (ip_debug.tcl; pcie_fl_zcu102.sv, "vio_rc probe map"). Outputs: 0 vio_en, 1
  # vio_transmit_enable, 2 vio_scan_start, 3 vio_scan_bus[7:0], 4 vio_bar_enable, 5
  # vio_bridge_enable, starting at 1, 1, 0, 00h, 1 and 0: bar_enable starts at 1 here, so that
  # enumeration runs to enum_done without the console.
  _vio $dir vio_rc \
    {4 16 10 4 4 16 16 16 8 4 24 256 128 256 128 7 16 16 16 8 4 24 256 128 256 128 10 4 19 36 22} \
    {1 1 1 8 1 1} {0x1 0x1 0x0 0x00 0x1 0x0}
}
