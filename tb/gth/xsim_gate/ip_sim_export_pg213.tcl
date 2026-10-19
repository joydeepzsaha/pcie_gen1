# ---------------------------------------------------------------------------
# ip_sim_export_pg213.tcl -- generates AMD's PCIe Endpoint (PG213) and its
# example design, and writes their xsim file list, for the xsim gate's pg213
# rows (xsim_gate.sh, sec 63 #27)
#
# Author: Kourosh Ghahramani
# Silicon Systems Research Lab, University of Washington
#
# Purpose
#   The far end of tb/gth/tb_pcie_rc_gth_pg213.sv: pcie4_uscale_plus v1.3, a
#   Gen1 x1 Endpoint, inside the IP's own example design top
#   (xilinx_pcie4_uscale_ep, with its PIO application), unmodified. The part is
#   xczu7ev-ffvc1156-2-e, not the RC's ZU9EG: the ZU9EG has no PCIe hard block.
#   Nothing it generates is committed.
#
#   The example's simulation fileset holds the vendor's own testbench (board,
#   a Root Port model and its tasks). This script removes those files from that
#   fileset, makes the example top the simulation top and lets Vivado write the
#   compile list, so the list holds the Endpoint's files only, in Vivado's
#   order, with its include directories.
#
# Usage
#   vivado -mode batch -source ip_sim_export_pg213.tcl -tclargs <out_dir>
#   Run from <out_dir>. The example project lands in <out_dir>/ex/pg213_ep_ex;
#   the list is the EXP|prj line's file, with paths relative to its directory.
#   It prints the IP's settings as EXP|cfg lines and EXP|end last; the gate
#   reads that line, not Vivado's exit code.
# ---------------------------------------------------------------------------
set OUT [lindex $argv 0]
create_project -force pg213_prj $OUT/prj -part xczu7ev-ffvc1156-2-e
create_ip -name pcie4_uscale_plus -vendor xilinx.com -library ip -version 1.3 -module_name pg213_ep
set ip [get_ips pg213_ep]
# Gen1 x1 Endpoint (also this part's defaults); everything else is the IP's default.
set_property -dict [list CONFIG.PL_LINK_CAP_MAX_LINK_SPEED {2.5_GT/s} CONFIG.PL_LINK_CAP_MAX_LINK_WIDTH {X1} \
  CONFIG.device_port_type {PCI_Express_Endpoint_device}] $ip
set shown [list CONFIG.PL_LINK_CAP_MAX_LINK_SPEED CONFIG.PL_LINK_CAP_MAX_LINK_WIDTH CONFIG.device_port_type \
  CONFIG.REF_CLK_FREQ CONFIG.axisten_if_width CONFIG.axisten_freq CONFIG.PF0_DEVICE_ID CONFIG.PF0_CLASS_CODE \
  CONFIG.pf0_bar0_enabled CONFIG.pf0_bar0_type CONFIG.pf0_bar0_64bit CONFIG.pf0_bar0_size CONFIG.pf0_bar0_scale \
  CONFIG.pf0_bar1_enabled CONFIG.pf0_bar2_enabled CONFIG.pf0_bar3_enabled CONFIG.pf0_bar4_enabled \
  CONFIG.pf0_bar5_enabled CONFIG.select_quad CONFIG.mode_selection]
foreach p [lsort [lsearch -all -inline -nocase [list_property $ip] CONFIG.*VENDOR_ID*]] { lappend shown $p }
foreach p $shown {
  if {[catch {puts "EXP|cfg|$p=[get_property $p $ip]"}]} { puts "EXP|cfg|$p=<none>" }
}
generate_target all $ip
puts "EXP|generated"
open_example_project -force -dir $OUT/ex $ip
close_project
open_project $OUT/ex/pg213_ep_ex/pg213_ep_ex.xpr
set sim [get_filesets sim_1]
set tb [get_files -norecurse -of_objects $sim]
foreach f $tb { puts "EXP|tb_removed|[file tail $f]" }
remove_files -fileset sim_1 $tb
set_property top xilinx_pcie4_uscale_ep $sim
set_property top_lib xil_defaultlib $sim
launch_simulation -scripts_only -simset sim_1
set prj [glob -nocomplain $OUT/ex/pg213_ep_ex/pg213_ep_ex.sim/sim_1/behav/xsim/xilinx_pcie4_uscale_ep_vlog.prj]
if {[llength $prj] != 1} { puts "EXP|error|no compile list"; exit 1 }
puts "EXP|prj|$prj"
puts "EXP|end"
