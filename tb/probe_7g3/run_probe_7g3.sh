#!/bin/bash
# sec 63 #7g-3 Phase 1 -- run ONE gate target with a bind probe appended to its
# staged .vc.  NOT a gate row.
#
# Why not `fusesoc run --build --run` after editing the .vc: fusesoc re-exports
# the work tree on every stage it runs and regenerates the .vc, which silently
# DROPPED the appended probe on the first attempt (0 probe files, run green).
# So only the SETUP stage is fusesoc's; build and run replicate edalize 2.x
# exactly: build = the work dir's own Makefile (`verilator -f <vc> ...` then
# `make -f Vtop.mk`), run = `./Vtop` in the work dir with the four variables
# edalize/flows/sim.py:114-119 sets (COCOTB_TEST_MODULES, MODULE,
# LIBPYTHON_LOC, PYGPI_PYTHON_BIN).  Plusargs are not used by any target run
# here; the script refuses a target whose .eda.yml declares any.
#
# Usage: run_probe_7g3.sh <target> <core> <probe.sv[,probe2.sv]|-> <build_root> [TESTCASE]
set -u
T="$1"; CORE="$2"; PROBE="$3"; BR="$4"; TC="${5:-}"
REPO=/home/kourosh/pcie_endpoint
command -v fusesoc >/dev/null 2>&1 || export PATH="/homes/kourosh/miniconda3/envs/pcie/bin:$PATH"
_vinc="$(dirname "$(command -v fusesoc)")/../include"
[ -f "$_vinc/lz4.h" ] && export CPATH="$_vinc${CPATH:+:$CPATH}"
cd "$REPO" || exit 1
python3 lint/check_waiver.py lint/waiver.vlt || { echo "WAIVER_INVALID"; exit 3; }
mkdir -p "$BR"
LOG="$BR/$T.log"
timeout 900 fusesoc run --setup --build-root="$BR" --target="$T" "$CORE" > "$LOG.setup" 2>&1
WD=$(find "$BR" -mindepth 2 -maxdepth 2 -type d -name "$T" | head -1)
VC=$(find "$WD" -maxdepth 1 -name '*.vc' | head -1)
EDA=$(find "$WD" -maxdepth 1 -name '*.eda.yml' | head -1)
[ -z "$VC" ] && { echo "SETUP_FAIL $T"; exit 4; }
MOD=$(python3 -c "import yaml,sys; d=yaml.safe_load(open('$EDA')); print(d.get('flow_options',{}).get('cocotb_module',''))")
# Optional, for Phase-1 measurement modules that are not in any core:
#   MODULE_OVERRIDE=<py module>  EXTRA_FILES=<a.py,b.py>  (copied into the work dir)
if [ -n "${MODULE_OVERRIDE:-}" ]; then MOD="$MODULE_OVERRIDE"; fi
for xf in ${EXTRA_FILES//,/ }; do cp "$xf" "$WD/"; done
PLUS=$(python3 -c "import yaml; d=yaml.safe_load(open('$EDA')); print(sum(1 for p in d.get('parameters',{}).values() if p.get('paramtype')=='plusarg'))")
[ "$PLUS" != "0" ] && { echo "PLUSARGS_PRESENT $T -- not supported"; exit 5; }
if [ "$PROBE" != "-" ]; then
  for pf in ${PROBE//,/ }; do
    cp "$pf" "$WD/"
    echo "$(basename "$pf")" >> "$VC"
  done
fi
( cd "$WD" && find src -type f \( -name '*.sv' -o -name '*.v' -o -name '*.svh' \) | sort | while read -r f; do
    b=$(basename "$f"); m=$(md5sum < "$f" | cut -d' ' -f1)
    hit=$(find "$REPO/src" "$REPO/tb" -name "$b" -type f -exec md5sum {} + 2>/dev/null | awk -v m="$m" '$1==m' | head -1)
    [ -n "$hit" ] && echo "MATCH $f" || echo "NOMATCH $f"
  done ) > "$BR/$T.provenance"
echo "MODULE=$MOD EXTRA_FILES=${EXTRA_FILES:-} HEAD=$(git rev-parse HEAD) src_tree=$(git rev-parse HEAD:src) dirty_src=$(git status --porcelain src | wc -l) probe=$PROBE probe_md5=$( [ "$PROBE" != "-" ] && cat ${PROBE//,/ } | md5sum | cut -d' ' -f1)" >> "$BR/$T.provenance"
( cd "$WD" && timeout 3600 make > "$LOG.build" 2>&1 ); brc=$?
LP=$(cocotb-config --libpython); PB=$(cocotb-config --python-bin)
( cd "$WD" && env COCOTB_TEST_MODULES="$MOD" MODULE="$MOD" LIBPYTHON_LOC="$LP" PYGPI_PYTHON_BIN="$PB" ${TC:+TESTCASE="$TC"} timeout 7200 ./Vtop > "$LOG" 2>&1 ); rc=$?
echo "BUILD_RC=$brc RC=$rc $(grep -o 'TESTS=[0-9]* PASS=[0-9]* FAIL=[0-9]* SKIP=[0-9]*' "$LOG" | tail -1) NOMATCH=$(grep -c '^NOMATCH' "$BR/$T.provenance") WARN=$(grep -cE '%Warning|%Error' "$LOG.build") WD=$WD"
