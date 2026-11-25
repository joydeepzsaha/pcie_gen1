# ---------------------------------------------------------------------------
# ip_pg239_refclk.tcl -- PG239 for the two reference clock fallbacks of
#                        pcie_rc_gth_zcu102
#
# Author: Kourosh Ghahramani
# Silicon Systems Research Lab, University of Washington
#
# Purpose
#   Defines make_pg239_refclk, which creates the PG239 instance of
#   make_pg239 (ip_pg239.tcl) and then sets its refclk1_location for a
#   fallback reference clock:
#     r1p  Bank_129_MGTREFCLK1, the SMA pair J79 / J80
#     r4   Bank_129_MGTREFCLK0, the Si570: make_pg239's own value, unchanged
#   Nothing else of the configuration differs. In the IP that Vivado 2023.2
#   generates, refclk1_location is read by no HDL: the pin the reference
#   clock enters on is the package pin of the board top's sys_clk_p port,
#   which pcie_rc_gth_zcu102_refclk_r1p.xdc or _r4.xdc sets.
#
# Usage
#   In Vivado 2023.2, with a current project for the ZCU102's part:
#   source ip_pg239_refclk.tcl ; make_pg239_refclk <dir> <r1p|r4>
#   This file sources ip_pg239.tcl, in its own directory.
#
# References
#   PG239, Table 4: Clock and Reset Signals
#   UG1182, Table 3-36: ZCU102 GTH Bank 129 Interface Connections
# ---------------------------------------------------------------------------

source [file join [file dirname [info script]] ip_pg239.tcl]

proc make_pg239_refclk {dir variant} {
  switch -- $variant {
    r1p     { set loc Bank_129_MGTREFCLK1 }
    r4      { set loc Bank_129_MGTREFCLK0 }
    default { error "make_pg239_refclk: variant = '$variant', expected r1p or r4" }
  }
  set ip [make_pg239 $dir]
  set_property CONFIG.refclk1_location $loc $ip
  set got [get_property CONFIG.refclk1_location $ip]
  if {$got ne $loc} { error "make_pg239_refclk: CONFIG.refclk1_location = '$got', expected '$loc'" }
  puts "PG239|CONFIG.refclk1_location|$got|variant=$variant"
  return $ip
}
