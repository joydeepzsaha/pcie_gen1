"""§63 #7g-2 Phase 1 -- the timer measurements at the full stack. NOT A GATE ROW.

Target `verilate_fullstack_probe7g2` runs this module against the same
composition `verilate_fullstack` builds, plus `probe_7g2.sv` (bind). Every
measurement is a RAW EVENT in a per-instance file written by the probe; this
module only drives the stimulus and writes its own markers to a JSON file.
Nothing is paired or scored here -- that happens offline, behind a known-answer
self-test (§22.92), in pcie_docs evidence/cleanup-7g/analyse_7g2.py.

Predictions were committed before this file ran: pcie_docs
evidence/cleanup-7g/PREDICTIONS_7G2.md (aaf21ea).

!! ONE TB PER TEST (tracker §5): cocotb cancels every task a test started when
the test ends, including the Clock. Each test calls bring_up(), which builds a
fresh TB.
"""
import json
import os

import cocotb
from cocotb.triggers import ClockCycles
from cocotb.utils import get_sim_time

from test_pcie_fullstack import bring_up, run_enumeration_fs, _log_enum_fs

# Longer than TWO periodic-UpdateFC intervals (200,000 cycles each, census
# §0(a)) after enumeration, so the idle tail contains at least two periodic
# events on each stack and one interval between them.
IDLE_TAIL = int(os.environ.get("PR7G2_IDLE_TAIL", "450000"))


def _marker(name, **kv):
    rec = {"marker": name, "t_ns": get_sim_time("ns")}
    rec.update(kv)
    with open("pr7g2_py.jsonl", "a") as f:
        f.write(json.dumps(rec) + "\n")


@cocotb.test()
async def g7g2_enumerate_then_idle(dut):
    """Bring up, enumerate once, then stay idle for IDLE_TAIL cycles.

    Markers: reset release (bring_up returns 5 cycles after it), the
    enumeration pulse, the enumeration verdict, the end of the idle tail. Every
    timer measurement is in the probe's files.
    """
    if os.path.exists("pr7g2_py.jsonl"):
        os.remove("pr7g2_py.jsonl")
    tb, _m, _p, _c, _pa, _s, _d, tasks = await bring_up(dut)
    _marker("bring_up_returned")
    _marker("enum_start")
    r = await run_enumeration_fs(dut)
    _log_enum_fs(dut, r)
    _marker("enum_end", **{k: int(v) for k, v in r.items()})
    for t in tasks:
        await t
    _marker("monitors_done")
    await ClockCycles(dut.clk_i, IDLE_TAIL)
    _marker("idle_end", idle_tail=IDLE_TAIL)
    # Non-vacuity only: the measurement needs a completed enumeration to have
    # traffic to measure. Everything else is scored offline.
    assert r["enum_done"] and not r["enum_error"], f"enumeration did not complete: {r}"

