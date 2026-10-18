#!/bin/bash
# ---------------------------------------------------------------------------
# xsim_gate.sh -- the xsim gate for pcie_rc_gth_top and the ZCU102 board top
#
# Author: Kourosh Ghahramani
# Silicon Systems Research Lab, University of Washington
#
# Purpose
#   Verilator cannot elaborate the GTH-bearing tops (the PG239 simulation
#   model is encrypted), so they are outside the Verilator gate.
#   This gate runs their three xsim benches, tb/gth/tb_pcie_rc_gth.sv,
#   tb/gth/tb_pcie_rc_gth_zcu102.sv and tb/gth/tb_pcie_rc_gth_pg213.sv, as a
#   COLD run of one commit, and reduces each row to one line of a small
#   artifact. The gate passes only if that artifact is byte-identical to the
#   expected file.
#
#   !! Every snapshot is elaborated with xelab --debug all (sec 63 #27). xsim
#   2023.2 drops the value-change events of the second instance of
#   tlp_validator in tlp_layer (sec 63 #25, a tool trap; HANDSHAKE sec 5), so a
#   bench that delivers TLPs to the RC needs the flag; the pg213 bench does.
#   The other rows deliver none, and their lines are the same with and without
#   it (A_XSIM_CPL.md F-A3).
#
# Usage
#   xsim_gate.sh <commit> <run_dir> [<expected>]
#   <run_dir> must not exist. <expected> defaults to xsim_gate.expected
#   beside this script. Exit 0 = GATE PASS, 1 = GATE FAIL, 2 or 3 = the gate
#   could not run. Run a private copy of this directory: bash reads a script
#   as it runs it, so editing a script a live run is executing corrupts it.
#
# What a run does, all under <run_dir>
#   1. tree/: git archive of <commit>, plus each submodule's git archive at
#      its gitlink, from $REPO. No worktree, so nothing is written to $REPO.
#   2. rc_top.f: the sources `fusesoc run --setup --target=lint ::rc_top_core`
#      stages from tree/, each hash-matched to its file in tree/.
#   3. ip/: the five IPs from tree/fpga/zcu102/ip_pg239.tcl and ip_debug.tcl,
#      exported for xsim (ip_sim_export.tcl). ep/: AMD's PCIe Endpoint (PG213)
#      and its example design on its own part, with its compile list
#      (ip_sim_export_pg213.tcl).
#   4. xl/, xz/ and xp/: the snapshots, tb_pcie_rc_gth on PG239 alone,
#      tb_pcie_rc_gth_zcu102 on all five IPs, the latter twice: at the bench's
#      POR_CYCLES and at 1250 (zcu102_por), and tb_pcie_rc_gth_pg213 on PG239
#      and PG213. A module defined by two different files in one snapshot's
#      compile lists stops the gate (xvlog would keep the last one silently).
#      Then the rows, one log each in logs/.
#   5. artifact.txt: xsim_gate_rows.py, after its self-test passes. Then cmp
#      against <expected>.
#   Times, md5s, tool return codes and the tree id go to stamp.txt, which is
#   never compared. xsim exits 0 on a kernel FATAL_ERROR, so no verdict reads
#   an exit code: the rows read FATAL and END from the logs.
#
# The rows (each bench's header gives its plusargs)
#   loop          loop bench, +FAREND=loop
#   commafree     loop bench, +FAREND=commafree, 200 us: must not reach L0
#   swap          loop bench, +FAREND=swap (P and N swapped), capped at 1 ms
#   zcu102_rel    board top, PERST# held to 20 us, then G0 X1's VIO pulse
#   zcu102_pulse  the same log: the second training, after the pulse
#   zcu102_hold   board top, PERST# never released in 100 us: must not reach L0
#   zcu102_r2     board top, G0's R2: PCLK stopped, PERST# written, re-train
#   gt_site       the loop log: the GT site the generated IP was made for,
#                 from its GT Wizard's channel map at time 0 (sec 63 #23)
#   zcu102_perst_pin  the zcu102_rel and zcu102_hold logs: the slot's PERST#
#                 pin is ~sys_rst_n_r at every clk125 edge, and its edges
#                 (sec 63 #23)
#   zcu102_por    board top with POR_CYCLES 1250, released at time 0: the PERST#
#                 release moves from zcu102_r2's by exactly 625 clk125 periods
#                 (sec 63 #23)
#   pg213         the RC (SIM_FAST_LINK 0) against PG213 on the serial pins:
#                 both reach L0 once and stay, FC init (sec 63 #27)
#   pg213_enum    the same log: the scan finds PG213 (10EEh / 9011h), the
#                 enumeration sizes and assigns its one BAR, and PG213's own
#                 Command bits read memory and bus master enabled
#
# When a later rung legitimately changes a time, xsim_gate.expected changes
# in that rung's commit, and the commit message gives the reason.
# ---------------------------------------------------------------------------
set -u
C=${1:?commit}; RUN=${2:?run_dir}
HERE=$(cd "$(dirname "$0")" && pwd)
EXP=${3:-$HERE/xsim_gate.expected}
REPO=${REPO:-/home/kourosh/pcie_endpoint}
VIVADO_SETTINGS=${VIVADO_SETTINGS:-/home/kourosh/tools/Xilinx/Vivado/2023.2/settings64.sh}
CONDA_BIN=${CONDA_BIN:-/homes/kourosh/miniconda3/envs/pcie/bin}
PY="$CONDA_BIN/python3 -B"

[ -e "$RUN" ] && { echo "GATE ERROR: $RUN exists; a cold run starts from nothing"; exit 2; }
[ -f "$EXP" ] || { echo "GATE ERROR: no expected file $EXP"; exit 2; }
mkdir -p "$RUN"/{tree,ip,ep,xl,xz,xp,logs} || exit 2
RUN=$(cd "$RUN" && pwd); T=$RUN/tree; S=$RUN/stamp.txt
st()  { echo "$1 $(date -u +%FT%TZ) ${2:-}" >> "$S"; }
die() { st ERROR "$1"; echo "GATE ERROR: $1"; exit 3; }

SHA=$(git -C "$REPO" rev-parse --verify "$C^{commit}") || die "no commit $C"
st START "commit=$SHA src=$(git -C "$REPO" rev-parse "$SHA:src") host=$(hostname -s)"
( cd "$HERE" && md5sum xsim_gate.sh xsim_gate_rows.py ip_sim_export.tcl ip_sim_export_pg213.tcl ) | sed 's/^/SCRIPT /' >> "$S"
echo "SCRIPT $(md5sum < "$EXP" | cut -c1-32)  expected=$EXP" >> "$S"

# ---- 1. the tree -------------------------------------------------------------
git -C "$REPO" archive "$SHA" | tar -x -C "$T" || die "git archive"
git -C "$REPO" ls-tree -r "$SHA" | awk '$2 == "commit" {print $3, $4}' > "$RUN/gitlinks.txt"
while read -r sub p; do
  mkdir -p "$T/$p" && git -C "$REPO/$p" archive "$sub" | tar -x -C "$T/$p" || die "submodule $p"
  echo "SUBMODULE $p $sub" >> "$S"
done < "$RUN/gitlinks.txt"
st TREE "files=$(find "$T" -type f | wc -l)"

# ---- 2. rc_top.f -------------------------------------------------------------
# A fusesoc.conf naming tree/ itself: an inherited one would stage another tree.
printf '[library.pcie-endpoint-controller]\nlocation = %s\nsync-uri = ./\nsync-type = local\nauto-sync = true\n' \
  "$T" > "$T/fusesoc.conf"
( cd "$T" && PATH=$CONDA_BIN:$PATH timeout 900 fusesoc run --setup --target=lint ::rc_top_core ) \
  > "$RUN/logs/stage_fusesoc.log" 2>&1 || die "fusesoc --setup"
VC=$(ls "$T"/build/rc_top_core_1.0.0/lint/*.vc 2>/dev/null)
[ "$(echo "$VC" | wc -w)" = 1 ] || die "staged .vc: '$VC'"
$PY "$HERE/xsim_gate_rows.py" rcf "$VC" "$(dirname "$VC")" "$T" "$RUN/rc_top.f" "$RUN/rc_top.f.provenance" \
  >> "$S" 2>&1 || die "rc_top.f provenance"
st STAGED

# ---- 3. the IPs --------------------------------------------------------------
( source "$VIVADO_SETTINGS" && cd "$RUN/ip" && \
  vivado -mode batch -nojournal -log ip.log -source "$HERE/ip_sim_export.tcl" -tclargs "$T" "$RUN/ip" ) \
  > "$RUN/logs/ip_console.log" 2>&1
grep -q '^EXP|end' "$RUN/ip/ip.log" || die "IP export (ip/ip.log has no EXP|end)"
st IP
( source "$VIVADO_SETTINGS" && cd "$RUN/ep" && \
  vivado -mode batch -nojournal -log ep.log -source "$HERE/ip_sim_export_pg213.tcl" -tclargs "$RUN/ep" ) \
  > "$RUN/logs/ep_console.log" 2>&1
grep -q '^EXP|end' "$RUN/ep/ep.log" || die "PG213 export (ep/ep.log has no EXP|end)"
grep '^EXP|cfg|' "$RUN/ep/ep.log" | sed 's/^EXP|cfg|/PG213 /' >> "$S"
st EP

# ---- 4. the snapshots --------------------------------------------------------
ip_prj() {  # <out> <ip>...: the exports' own file lists; paths made absolute; glbl.v once, last
  local out=$1; shift; : > "$out"
  for ip; do
    sed -e "s#\"\.\./\.\./\.\./#\"$RUN/ip/#" -e '/glbl.v/d' -e '/^nosort/d' -e '/^#/d' \
        "$RUN/ip/export_sim/$ip/xsim/vlog.prj" >> "$out"
  done
  echo "verilog xil_defaultlib \"$RUN/ip/export_sim/pg239_gen1_x1/xsim/glbl.v\"" >> "$out"
  echo "nosort" >> "$out"
}
rc_prj() {  # <out> <tree file>...: rc_top.f, all as SystemVerilog as Verilator reads it, then ours
  local out=$1; shift
  { echo "# rc_top.f + the tops + the bench"
    while read -r f; do [ -n "$f" ] && echo "sv xil_defaultlib \"$f\""; done < "$RUN/rc_top.f"
    for f; do echo "sv xil_defaultlib \"$T/$f\""; done
    echo "nosort"; } > "$out"
}
ep_prj() {  # <out>: PG213's compile list; paths made absolute; its glbl.v dropped (ip_prj adds PG239's)
  local src base
  src=$(sed -n 's/^EXP|prj|//p' "$RUN/ep/ep.log")
  base=$(cd "$(dirname "$src")/../../../.." && pwd) || return 1
  sed -e "s#\"\.\./\.\./\.\./\.\./#\"$base/#g" -e '/glbl.v/d' -e '/^nosort/d' -e '/^#/d' "$src" > "$1"
  echo "nosort" >> "$1"
}
compile() {  # <dir> <snapshot> <bench module>: ip_vlog.prj, rc_vlog.prj and ep_vlog.prj if there is one
  ( source "$VIVADO_SETTINGS" && cd "$1" && \
    xvlog --incr --relax -prj ip_vlog.prj > xvlog_ip.log 2>&1; a=$?; \
    xvlog --incr --relax -prj rc_vlog.prj > xvlog_rc.log 2>&1; b=$?; \
    e=; [ -f ep_vlog.prj ] && { xvlog --incr --relax -prj ep_vlog.prj > xvlog_ep.log 2>&1; e=" xvlog_ep=$?"; }; \
    xelab --incr --relax --mt 8 --debug all --timescale 1ns/1ps \
          -L gtwizard_ultrascale_v1_7_17 -L xil_defaultlib -L unisims_ver -L unimacro_ver -L secureip -L xpm \
          --snapshot "$2" "xil_defaultlib.$3" xil_defaultlib.glbl > xelab.log 2>&1; c=$?; \
    echo "xvlog_ip=$a xvlog_rc=$b$e xelab=$c" )
}
ip_prj "$RUN/xl/ip_vlog.prj" pg239_gen1_x1
rc_prj "$RUN/xl/rc_vlog.prj" src/rc/pcie_rc_gth_top.sv tb/gth/tb_pcie_rc_gth.sv
st COMPILE_XL "$(compile "$RUN/xl" tb_rc_gth tb_pcie_rc_gth)"
elab_generic() {  # <dir> <snapshot> <bench module> <name=value>: another snapshot of a compiled bench
  ( source "$VIVADO_SETTINGS" && cd "$1" && \
    xelab --incr --relax --mt 8 --debug all --timescale 1ns/1ps \
          -L gtwizard_ultrascale_v1_7_17 -L xil_defaultlib -L unisims_ver -L unimacro_ver -L secureip -L xpm \
          -generic_top "$4" --snapshot "$2" "xil_defaultlib.$3" xil_defaultlib.glbl > "xelab_$2.log" 2>&1; \
    echo "xelab_$2=$?" )
}
ip_prj "$RUN/xz/ip_vlog.prj" pg239_gen1_x1 ila_pclk ila_free vio_pclk vio_free
rc_prj "$RUN/xz/rc_vlog.prj" src/rc/pcie_rc_gth_top.sv fpga/zcu102/pcie_rc_gth_zcu102.sv tb/gth/tb_pcie_rc_gth_zcu102.sv
st COMPILE_XZ "$(compile "$RUN/xz" tb_zcu102 tb_pcie_rc_gth_zcu102)"
st COMPILE_XZ_POR "$(elab_generic "$RUN/xz" tb_zcu102_por tb_pcie_rc_gth_zcu102 POR_CYCLES=1250)"
ip_prj "$RUN/xp/ip_vlog.prj" pg239_gen1_x1
ep_prj "$RUN/xp/ep_vlog.prj" || die "PG213 compile list"
rc_prj "$RUN/xp/rc_vlog.prj" src/rc/pcie_rc_gth_top.sv tb/gth/tb_pcie_rc_gth_pg213.sv
$PY "$HERE/xsim_gate_rows.py" modules "$RUN/xp"/{ip,ep,rc}_vlog.prj >> "$S" 2>&1 || die "a module defined twice in xp/"
st COMPILE_XP "$(compile "$RUN/xp" tb_pg213 tb_pcie_rc_gth_pg213)"
for d in xl xz xp; do
  grep -q '^ERROR' "$RUN/$d"/xvlog_*.log "$RUN/$d"/xelab*.log && die "compile errors in $d/"
done

# ---- 5. the rows -------------------------------------------------------------
row() {  # <row> <dir> <snapshot> <plusarg>...
  local name=$1 dir=$2 snap=$3; shift 3
  local args=(); for a; do args+=(--testplusarg "$a"); done
  st "ROW_$name" "$( source "$VIVADO_SETTINGS" && cd "$RUN/$dir" && \
    xsim "$snap" -R "${args[@]}" > "$RUN/logs/$name.log" 2>&1; echo "xsim_rc=$?" )"
}
row loop        xl tb_rc_gth FAREND=loop
row commafree   xl tb_rc_gth FAREND=commafree MAX_US=200
row swap        xl tb_rc_gth FAREND=swap MAX_US=1000
row zcu102_rel  xz tb_zcu102 MAX_US=400 FR_US=40 PULSE=1 PERST_REL_US=20
row zcu102_hold xz tb_zcu102 PERST_REL_US=1000 MAX_US=100
row zcu102_r2   xz tb_zcu102 MAX_US=400 FR_US=40 R2=1
row zcu102_por  xz tb_zcu102_por MAX_US=400 PULSE=0
row pg213       xp tb_pg213 MAX_US=600
( cd "$RUN/logs" && md5sum ./*.log ) | sed 's/^/LOG /' >> "$S"

# ---- 6. the artifact and the verdict -------------------------------------------
$PY "$HERE/xsim_gate_rows.py" selftest > "$RUN/selftest.txt" 2>&1 || die "self-test (selftest.txt)"
$PY "$HERE/xsim_gate_rows.py" artifact "$T/src/ltssm/pcie_ltssm_downstream.sv" "$RUN/logs" \
  > "$RUN/artifact.txt" 2>> "$S" || die "artifact"
echo "ARTIFACT $(md5sum < "$RUN/artifact.txt" | cut -c1-32) rows=$(wc -l < "$RUN/artifact.txt")" >> "$S"
if cmp -s "$RUN/artifact.txt" "$EXP"; then
  st END "GATE PASS"; echo "GATE PASS ($(wc -l < "$RUN/artifact.txt") rows)"; exit 0
fi
st END "GATE FAIL"
echo "GATE FAIL: the artifact differs from $EXP"
diff "$EXP" "$RUN/artifact.txt" | grep '^[<>]' | sed -e 's/^</  expected:/' -e 's/^>/  measured:/'
exit 1
