"""sec 63 #7k Phase 1 -- NOT A GATE ROW, IN NO CORE.  What the UNMODIFIED tree
does AFTER the blackout lifts (C7K-7, and P2's "later TLPs continue").

The W1 row ends at its pinned assertion, so on the red tree it never watches
past the release.  This module drives the IDENTICAL stimulus -- same helpers,
imported from test_pcie_fullstack, same constants -- up to the release, then
keeps capturing for P1_AFTER cycles and lets the enumeration run to its own
end.  Run through run_probe_7k.sh with MODULE_OVERRIDE=test_7k_p1_after and
EXTRA_FILES=<this file>, so probe_7k.sv records the same window from inside.
"""
import cocotb
from cocotb.triggers import RisingEdge, ClockCycles

from test_pcie_fullstack import (
    bring_up, run_enumeration_fs, _log_enum_fs, _i, W1Capture, w1_selftest,
    w1_starved, w1_recovery_entries, W1_FC_WAIT, W1_SETTLE, W1_GUARD,
    W1_SENDS_TO_ROLLOVER, W1_ROLL_MARGIN, W1_RECOVERY_BUDGET, W1_WINDOW,
)

P1_AFTER = 120000   # cycles after release: past the RC's completion timeout budget


@cocotb.test()
async def p1_after_release(dut):
    w1_selftest()
    tb, _m, _p, _c, _pa, _s, _d, tasks = await bring_up(dut)
    cap = W1Capture(dut)
    ctask = cocotb.start_soon(cap.run(dut.clk_i, W1_WINDOW))
    for _ in range(W1_FC_WAIT):
        await RisingEdge(dut.clk_i)
        if _i(dut.rc_fc_initialized_o) and _i(dut.ep_fc_initialized_o):
            break
    await ClockCycles(dut.clk_i, W1_SETTLE)
    dut.starve_en.value = 1
    await RisingEdge(dut.clk_i)
    arm = cap.cycles
    enum = {}

    async def enumerate_():
        enum.update(await run_enumeration_fs(dut))
        enum["end_cycle"] = cap.cycles

    etask = cocotb.start_soon(enumerate_())
    while True:
        await RisingEdge(dut.clk_i)
        st = w1_starved(cap.ev["rc"]["sent"], arm)
        if st and len(st[1]) >= W1_SENDS_TO_ROLLOVER:
            break
        assert cap.cycles - arm <= W1_GUARD
    roll = st[1][W1_SENDS_TO_ROLLOVER - 1] + W1_ROLL_MARGIN
    while cap.cycles < roll + W1_RECOVERY_BUDGET:
        await RisingEdge(dut.clk_i)
    dut.starve_en.value = 0
    await RisingEdge(dut.clk_i)
    release = cap.cycles
    dut._log.info("P1AFTER arm=%d roll=%d release=%d", arm, roll, release)
    while cap.cycles < release + P1_AFTER and not enum:
        await RisingEdge(dut.clk_i)
    await ClockCycles(dut.clk_i, 2000)
    cap.stop = True
    await ctask
    cap.census(dut, "P1AFTER", arm, release)
    for side in ("rc", "ep"):
        e = cap.ev[side]
        dut._log.info("P1AFTER %s after release: slots=%s", side.upper(),
                      [x for x in e["slot"] if x[0] >= release][:40])
        dut._log.info("P1AFTER %s after release: sent=%s", side.upper(),
                      [x for x in e["sent"] if x[0] >= release][:40])
        dut._log.info("P1AFTER %s after release: acknak_in=%s", side.upper(),
                      [x for x in e["acknak"] if x[0] >= release][:40])
        dut._log.info("P1AFTER %s after release: occ=%s err=%s", side.upper(),
                      [x for x in e["occ"] if x[0] >= release][:40],
                      [x for x in e["err"] if x[0] >= release][:10])
        dut._log.info("P1AFTER %s after release: dllp_tx n=%d first=%s", side.upper(),
                      len([1 for c, t in e["dllp_tx"] if c >= release]),
                      [(c, hex(t)) for c, t in e["dllp_tx"] if c >= release][:12])
        dut._log.info("P1AFTER %s after release: deliveries=%s nrs=%s", side.upper(),
                      [c for c in e["deliv"] if c >= release][:40],
                      [x for x in e["nrs"] if x[0] >= release][:40])
    if enum:
        _log_enum_fs(dut, enum)
    dut._log.info("P1AFTER enum_end_cycle=%s cycles=%d rc_recovery=%s ep_recovery=%s",
                  enum.get("end_cycle"), cap.cycles,
                  w1_recovery_entries(cap.ev["rc"]["lt"], arm),
                  w1_recovery_entries(cap.ev["ep"]["lt"], arm))
