# ---------------------------------------------------------------------------
# ip_sim_export.tcl -- generates the board top's five IPs and exports them
# for xsim, for the xsim gate (xsim_gate.sh)
#
# Author: Kourosh Ghahramani
# Silicon Systems Research Lab, University of Washington
#
# Purpose
#   Creates pg239_gen1_x1, ila_pclk, ila_free, vio_pclk and vio_free from the
#   generators of the tree under test, fpga/zcu102/ip_pg239.tcl and
#   ip_debug.tcl, and nothing else, then writes their simulation targets and
#   an xsim export. Nothing it generates is committed.
#
# Usage
#   vivado -mode batch -source ip_sim_export.tcl -tclargs <tree> <out_dir>
#   Run from <out_dir>. The IPs land in <out_dir>/ip and the export in
#   <out_dir>/export_sim/<ip>/xsim. It prints EXP|end last; the gate reads
#   that line, not Vivado's exit code.
# ---------------------------------------------------------------------------
set TREE [lindex $argv 0]
set OUT  [lindex $argv 1]
set_part xczu9eg-ffvb1156-2-e
source $TREE/fpga/zcu102/ip_pg239.tcl
source $TREE/fpga/zcu102/ip_debug.tcl
make_pg239 $OUT/ip
make_debug_ips $OUT/ip
foreach ip [lsort [get_ips]] {
  generate_target {simulation} $ip
  puts "EXP|gen|$ip"
}
# Non-project mode creates no ip_user_files directory, so the export reads the
# IP directories directly.
export_simulation -of_objects [get_ips] -simulator xsim -directory $OUT/export_sim \
  -use_ip_compiled_libs -force
puts "EXP|end"
