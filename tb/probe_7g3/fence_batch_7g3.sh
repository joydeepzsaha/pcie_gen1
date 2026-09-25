#!/bin/bash
# sec 63 #7g-3: run the frame_symbols fence (and, on the two stacked targets, the
# stack probe) over the whole 10-target frame_symbols radius, in parallel, one
# build root per target.  Usage: fence_batch_7g3.sh <out_root>
set -u
OUT="$1"; mkdir -p "$OUT"
P=/home/kourosh/pcie_endpoint/tb/probe_7g3
FS=$P/probe_7g3_fs.sv; ST=$P/probe_7g3_stack.sv
run() { "$P/run_probe_7g3.sh" "$1" "$2" "$3" "$OUT/$1" > "$OUT/$1.result" 2>&1; }
run verilate_tx_golden          fusesoc:pcie:tb_phy_transmit   $FS &
run verilate_tx_x4              fusesoc:pcie:tb_phy_transmit   $FS &
run verilate_phy_transmit_stall fusesoc:pcie:tb_phy_transmit   $FS &
run verilate_tx_os_golden       fusesoc:pcie:tb_phy_tx_golden  $FS &
run verilate_tx_skp             fusesoc:pcie:tb_phy_tx_golden  $FS &
run verilate_tx_cdc             fusesoc:pcie:tb_phy_tx_golden  $FS &
run verilate_tx_framing         fusesoc:pcie:tb_phy_tx_golden  $FS &
run verilate_7j2_idle           fusesoc:pcie:tb_phy_tx_golden  $FS &
run verilate_rc_top             fusesoc:pcie:tb_rc             $FS,$ST &
run verilate_fullstack          fusesoc:pcie:tb_fullstack      $FS,$ST &
wait
echo BATCH_DONE
for f in "$OUT"/*.result; do echo "$(basename "$f" .result): $(tail -1 "$f")"; done
