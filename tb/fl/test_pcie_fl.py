"""test_pcie_fl -- the board top pcie_fl_zcu102, driven as its VIO console drives it.

The toplevel is the board top itself (fpga/zcu102/pcie_fl_zcu102.sv): the Root
Complex and the Endpoint linked in fabric by pipe_fabric_link. Only its vendor
cells are stand-ins (fl_vendor_stubs.sv). The bench has no port to drive but
the clock. Everything else goes through the two VIO stand-ins, as on the board:
it writes vio_fl's run to release the reset and reads the status, the Vendor
and Device ID, the sticky counters and the event times from the VIOs' inputs.

Sampling: every monitor reads after RisingEdge with no ReadOnly, so it sees the
values from before the edge, the values the design's registers captured at it.

Tests, in order:
  fl_debug_recipe_matches_the_bench_cores   the stand-ins have the probe widths
      and initial values of ip_debug_fl.tcl, and the design is in reset
  fl_trains_and_enumerates_after_reset_release   one reset release: both ends
      reach L0, Flow Control initializes, the scan reads the Vendor and Device
      ID, enumeration completes, and only the link-up counters move
  fl_reset_release_is_repeatable   three more resets from the console, one of
      them a one-cycle pulse: each trains and enumerates again, at the same
      cycle counts
"""
import os
import re

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, RisingEdge

CLK_NS = 8                 # CLK_125
VENDOR_ID = 0x1234         # pcie_config_reg's constants, which the scan reads
DEVICE_ID = 0x00FF
DETECT_LATENCY = 4         # pipe_fabric_link's default
IDLE_EXIT_EDGES = 6        # pipe_fabric_link's default
RST_MIN_CYCLES = 16        # pcie_fl_zcu102's default
SETTLE_CYCLES = 2000       # sampled after enumeration, for a late event

# vio_fl's probe map (pcie_fl_zcu102.sv, "Debug cores").
STATUS_BITS = ["run_q", "rst", "rc_link_up", "ep_link_up", "rc_fc", "ep_fc",
               "scan_done", "enum_done", "scan_error", "enum_error",
               "device_present", "inj_fired", "enc_illegal_k", "dec_code_err",
               "dec_disp_err"]
COUNTERS = ["release", "rc_linkup", "ep_linkup", "rc_linkdown", "ep_linkdown",
            "rc_recovery", "ep_recovery", "rc_replay", "ep_replay",
            "rc_retrain", "ep_retrain", "rc_nak_rx", "ep_nak_rx",
            "rc_nak_sched", "ep_nak_sched", "rc_rx_code", "rc_rx_disp",
            "ep_rx_code", "ep_rx_disp"]                 # probe_in6 .. probe_in24
TIMES = ["rc_linkup", "ep_linkup", "rc_fc", "ep_fc", "scan_done", "enum_done"]  # probe_in26 .. 31
COUNTER_BASE = 6
T_CNT_PROBE = 25
TIME_BASE = 26
# The counters a clean bring-up moves, and by how much per reset release.
MOVES_PER_RELEASE = {"release": 1, "rc_linkup": 1, "ep_linkup": 1}


def _i(sig):
    """A handle's value as an int."""
    return int(sig.value)


class Console:
    """The two VIOs, read and written at their ports."""

    def __init__(self, dut):
        self.dut = dut
        self.fl = dut.u_vio_fl
        self.rc = dut.u_vio_rc

    def status(self):
        v = _i(self.fl.probe_in0)
        return {n: (v >> i) & 1 for i, n in enumerate(STATUS_BITS)}

    def counters(self):
        return {n: _i(getattr(self.fl, f"probe_in{COUNTER_BASE + i}"))
                for i, n in enumerate(COUNTERS)}

    def times(self):
        return {n: _i(getattr(self.fl, f"probe_in{TIME_BASE + i}"))
                for i, n in enumerate(TIMES)}

    def ids(self):
        """(Vendor ID, Device ID) as vio_fl shows them and as vio_rc does."""
        return ((_i(self.fl.probe_in1), _i(self.fl.probe_in2)),
                (_i(self.rc.probe_in6), _i(self.rc.probe_in7)))

    def run(self, v):
        self.fl.probe_out0.value = v

    async def clear(self):
        self.fl.probe_out1.value = 1
        await ClockCycles(self.dut.clk125_p, 4)
        self.fl.probe_out1.value = 0
        await ClockCycles(self.dut.clk125_p, 4)


class Watch:
    """Per-edge record of one reset release: the four status levels, the
    link's sideband at both ends and the reset. The levels and the Electrical
    Idle record start at the first edge that samples the reset low after one
    that sampled it high, so a Watch started while the link is still up
    records the next bring-up, not the old link."""

    LEVELS = ["rc_link_up", "ep_link_up", "rc_fc", "ep_fc"]

    def __init__(self, dut):
        self.dut = dut
        self.n = 0                       # edges sampled
        self.armed = False               # the reset has been sampled high
        self.saw_low = {k: False for k in self.LEVELS}
        self.rose_at = {k: None for k in self.LEVELS}
        self.fell = {k: False for k in self.LEVELS}
        self.rst_edges = 0               # edges at which rst_link was sampled high
        self.first_run_edge = None       # first edge at which it was sampled low
        self.idle_high = []              # edges, out of reset, with RxElecIdle high
        self.idle_differs = 0            # edges at which the two ends' levels differed
        self.end = {e: {"prev": 0, "rise": [], "phy": [], "bad_status": 0}
                    for e in ("rc", "ep")}

    def sample(self):
        d = self.dut
        i = self.n
        self.n += 1
        in_reset = _i(d.rst_link)
        if in_reset:
            self.armed = True
            self.rst_edges += 1
        if self.armed and not in_reset:
            if self.first_run_edge is None:
                self.first_run_edge = i
            lv = {"rc_link_up": _i(d.rc_link_up), "ep_link_up": _i(d.ep_link_up),
                  "rc_fc": _i(d.rc_fc_initialized), "ep_fc": _i(d.ep_fc_initialized)}
            for k, v in lv.items():
                if not v:
                    if self.rose_at[k] is None:
                        self.saw_low[k] = True
                    else:
                        self.fell[k] = True
                elif self.rose_at[k] is None:
                    self.rose_at[k] = i
            if _i(d.rc_rxelecidle):
                self.idle_high.append(i)
        if _i(d.rc_rxelecidle) != _i(d.ep_rxelecidle):
            self.idle_differs += 1
        for e in ("rc", "ep"):
            s = self.end[e]
            cur = _i(getattr(d, f"{e}_txdetectrx"))
            phy = _i(getattr(d, f"{e}_phystatus"))
            rxs = _i(getattr(d, f"{e}_rxstatus"))
            if cur and not s["prev"]:
                s["rise"].append(i)
            s["prev"] = cur
            if phy:
                s["phy"].append(i)
            if rxs != (0b011 if phy else 0):
                s["bad_status"] += 1

    def answered(self, e):
        """The request edges pipe_fabric_link answers: a rising TxDetectRx,
        unless an answer to an earlier one is pending or being given."""
        out = []
        for r in self.end[e]["rise"]:
            if not out or r > out[-1] + DETECT_LATENCY + 1:
                out.append(r)
        return out


def _budget(dut):
    """Cycles allowed for one bring-up. FL_BUDGET_CYCLES overrides it."""
    env = os.environ.get("FL_BUDGET_CYCLES")
    if env:
        return int(env)
    try:
        hold = _i(dut.CFG_HOLD_CYCLES)
        fast = _i(dut.SIM_FAST_LINK)
    except Exception as exc:    # the parameters are not visible on this build
        dut._log.info("FL_BUDGET parameters not readable (%r); board budget used", exc)
        hold, fast = 12_500_000, 0
    return 600_000 + 2 * hold + (0 if fast else 12_000_000)


async def run_until(dut, watch, cond, cycles):
    """Sample every edge until cond() is true or cycles have passed; return
    whether cond() was met."""
    for _ in range(cycles):
        await RisingEdge(dut.clk125_p)
        watch.sample()
        if cond():
            return True
    return False


async def bring_up(dut, con, hold_low=40, low=None):
    """One reset from the console: run 0 for hold_low cycles (or the exact
    `low` cycles, with no status check), then 1, then wait for enumeration to
    end. Returns the Watch, started while the stacks were still in reset."""
    clk = dut.clk125_p
    watch = Watch(dut)
    con.run(0)
    if low is None:
        await ClockCycles(clk, hold_low)
        st = con.status()
        assert st["rst"] == 1 and st["run_q"] == 0, f"run 0 did not assert the reset: {st}"
        assert not (st["rc_link_up"] or st["ep_link_up"] or st["rc_fc"] or st["ep_fc"]
                    or st["scan_done"] or st["enum_done"]), (
            f"a status level survived the reset: {st}")
        assert _i(dut.led) == 0, f"LEDs {_i(dut.led):#x} in reset"
        assert all(v == 0 for v in con.times().values()), (
            f"event times not zeroed by the reset: {con.times()}")
    else:
        for _ in range(low):
            await RisingEdge(clk)
            watch.sample()
    con.run(1)

    def ended():
        # Only after this reset has been released: until the reset lands, the
        # status is still the previous bring-up's.
        if watch.first_run_edge is None:
            return False
        st = con.status()
        return st["enum_done"] or st["enum_error"] or st["scan_error"]

    met = await run_until(dut, watch, ended, _budget(dut))
    await run_until(dut, watch, lambda: False, SETTLE_CYCLES)
    watch.met = met
    return watch


def report(dut, tag, con, watch):
    """Log the release's record before any assertion."""
    st, cn, tm = con.status(), con.counters(), con.times()
    dut._log.info("FL_STATUS %s %s", tag, " ".join(f"{k}={v}" for k, v in st.items()))
    dut._log.info("FL_IDS %s vio_fl=%04x:%04x vio_rc=%04x:%04x", tag,
                  *con.ids()[0], *con.ids()[1])
    dut._log.info("FL_COUNTERS %s %s", tag, " ".join(f"{k}={v}" for k, v in cn.items()))
    dut._log.info("FL_TIMES %s %s t_cnt=%d", tag, " ".join(f"{k}={v}" for k, v in tm.items()),
                  _i(getattr(con.fl, f"probe_in{T_CNT_PROBE}")))
    dut._log.info("FL_LTSSM %s rc=%#x ep=%#x dl_states=%#o leds=%#x", tag,
                  _i(con.fl.probe_in3), _i(con.fl.probe_in4), _i(con.fl.probe_in5), _i(dut.led))
    for e in ("rc", "ep"):
        s = watch.end[e]
        dut._log.info("FL_DETECT %s %s requests=%s answered=%s phystatus=%s bad_status=%d",
                      tag, e, s["rise"], watch.answered(e), s["phy"], s["bad_status"])
    dut._log.info("FL_RESET %s edges_in_reset=%d first_run_edge=%s idle_high=%s idle_differs=%d "
                  "edges=%d ended=%s", tag, watch.rst_edges, watch.first_run_edge,
                  watch.idle_high, watch.idle_differs, watch.n, watch.met)
    return st, cn, tm


def check_bring_up(dut, con, watch, releases):
    """The claims of one clean bring-up; `releases` is how many reset releases
    the counters have seen since their last clear."""
    st, cn, tm = con.status(), con.counters(), con.times()

    # ---- the link trained and Flow Control initialized, both ends ----------
    for k in Watch.LEVELS:
        assert watch.saw_low[k], f"non-vacuity failed: {k} was never sampled low"
        assert watch.rose_at[k] is not None, f"{k} never rose"
        assert not watch.fell[k], f"{k} fell after rising, with the stacks out of reset"
    # ---- enumeration, and the IDs the console reads ------------------------
    assert watch.met, "enumeration did not end inside the budget"
    assert st["scan_done"] and not st["scan_error"], f"the scan did not complete: {st}"
    assert st["device_present"], "the Endpoint was not detected"
    assert st["enum_done"] and not st["enum_error"], f"enumeration did not complete: {st}"
    (vid_fl, did_fl), (vid_rc, did_rc) = con.ids()
    assert (vid_fl, did_fl) == (VENDOR_ID, DEVICE_ID), (
        f"vio_fl shows {vid_fl:04x}:{did_fl:04x}, expected {VENDOR_ID:04x}:{DEVICE_ID:04x}")
    assert (vid_rc, did_rc) == (VENDOR_ID, DEVICE_ID), (
        f"vio_rc shows {vid_rc:04x}:{did_rc:04x}, expected {VENDOR_ID:04x}:{DEVICE_ID:04x}")
    assert _i(dut.led) == 0xF, f"LEDs {_i(dut.led):#x}, expected all four on"
    assert _i(con.fl.probe_in3) & 0x1F == 5 and _i(con.fl.probe_in4) & 0x1F == 5, (
        "an LTSSM is not in L0 after enumeration")
    assert _i(con.fl.probe_in5) == 0o44, (
        f"Data Link Layer states {_i(con.fl.probe_in5):#o}, expected DL_Active at both ends")
    assert not (st["inj_fired"] or st["enc_illegal_k"] or st["dec_code_err"] or st["dec_disp_err"]), (
        f"a link flag is set on a clean bring-up: {st}")
    # ---- the counters: only the link-up counters move ----------------------
    want = {k: MOVES_PER_RELEASE.get(k, 0) * releases for k in COUNTERS}
    assert cn == want, (
        "counters moved that a clean bring-up does not move: "
        + ", ".join(f"{k}={cn[k]} (expected {want[k]})" for k in COUNTERS if cn[k] != want[k]))
    # ---- the event times: all latched, in a possible order ------------------
    assert all(v > 0 for v in tm.values()), f"an event time was not latched: {tm}"
    assert tm["rc_linkup"] < tm["rc_fc"] < tm["scan_done"] <= tm["enum_done"], f"order: {tm}"
    assert tm["ep_linkup"] < tm["ep_fc"] < tm["scan_done"], f"order: {tm}"
    # ---- the link's sideband ------------------------------------------------
    for e in ("rc", "ep"):
        s = watch.end[e]
        asked = watch.answered(e)
        assert len(asked) >= 1, f"non-vacuity failed: the {e} end never requested receiver detection"
        assert s["phy"] == [r + DETECT_LATENCY + 1 for r in asked], (
            f"{e}: PhyStatus sampled at {s['phy']}, requests at {asked}: each request must be "
            f"answered once, {DETECT_LATENCY + 1} edges later")
        assert s["bad_status"] == 0, f"{e}: RxStatus was not 011b with PhyStatus and 000b without"
    f0 = watch.first_run_edge
    assert watch.idle_high == list(range(f0, f0 + IDLE_EXIT_EDGES)), (
        f"RxElecIdle sampled high at {watch.idle_high}; expected the {IDLE_EXIT_EDGES} edges "
        f"from {f0}")
    assert watch.idle_differs == 0, "the two ends' RxElecIdle levels differed"


def _recipe(path, kind, name):
    """The brace lists of one _ila / _vio call in ip_debug_fl.tcl."""
    text = open(path).read().replace("\\\n", " ")
    text = "\n".join(l for l in text.split("\n") if not l.lstrip().startswith("#"))
    m = re.search(r"^\s*_%s \$dir %s\b([^\n]*)$" % (kind, name), text, re.M)
    assert m, f"no _{kind} call for {name} in {path}"
    return [[t for t in g.split()] for g in re.findall(r"\{([^}]*)\}", m.group(1))]


@cocotb.test()
async def fl_debug_recipe_matches_the_bench_cores(dut):
    """The bench's ILA and VIO stand-ins have the probe counts, widths and
    initial values of fpga/zcu102/ip_debug_fl.tcl, and with those initial
    values the design sits in reset.

    The board top connects every probe at the stand-in's width (Verilator
    stops on a width mismatch), so this ties the top's connections to the
    recipe a board build generates the real cores from. It runs first, before
    any test writes a VIO output.
    """
    cocotb.start_soon(Clock(dut.clk125_p, CLK_NS, units="ns").start())
    await ClockCycles(dut.clk125_p, 20)
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "ip_debug_fl.tcl")

    (ila_w,) = _recipe(path, "ila", "ila_fl")
    got = [len(getattr(dut.u_ila_fl, f"probe{i}")) for i in range(len(ila_w))]
    assert got == [int(w) for w in ila_w], f"ila_fl widths {got} != recipe {ila_w}"
    assert not hasattr(dut.u_ila_fl, f"probe{len(ila_w)}"), "ila_fl has a probe the recipe lacks"

    checked = len(ila_w)
    for name in ("vio_fl", "vio_rc"):
        in_w, out_w, out_init = _recipe(path, "vio", name)
        core = getattr(dut, f"u_{name}")
        got = [len(getattr(core, f"probe_in{i}")) for i in range(len(in_w))]
        assert got == [int(w) for w in in_w], f"{name} input widths {got} != recipe {in_w}"
        got = [len(getattr(core, f"probe_out{i}")) for i in range(len(out_w))]
        assert got == [int(w) for w in out_w], f"{name} output widths {got} != recipe {out_w}"
        got = [_i(getattr(core, f"probe_out{i}")) for i in range(len(out_w))]
        assert got == [int(v, 16) for v in out_init], (
            f"{name} initial values {got} != recipe {out_init}")
        assert not hasattr(core, f"probe_in{len(in_w)}"), f"{name} has an input the recipe lacks"
        assert not hasattr(core, f"probe_out{len(out_w)}"), f"{name} has an output the recipe lacks"
        checked += len(in_w) + len(out_w)
    assert checked == 19 + 32 + 3 + 31 + 6, f"non-vacuity failed: {checked} probes compared"

    con = Console(dut)
    st = con.status()
    assert st["rst"] == 1 and st["run_q"] == 0, f"not in reset at the initial values: {st}"
    assert all(v == 0 for v in con.counters().values()), f"counters not zero: {con.counters()}"
    assert _i(dut.led) == 0, f"LEDs {_i(dut.led):#x} before any release"
    dut._log.info("FL_RECIPE %d probes compared; in reset at the initial values", checked)


@cocotb.test()
async def fl_trains_and_enumerates_after_reset_release(dut):
    """One reset release from the console brings the whole stack up.

    Claims, each able to fail on its own:
      1. both LTSSMs raise link up, each seen low first, and neither drops it;
      2. Flow Control initializes at both ends and stays initialized;
      3. the Root Complex's scan, started by the top itself, reads Vendor ID
         1234h and Device ID 00FFh from the Endpoint's configuration space,
         and both VIOs show them;
      4. enumeration completes (enum_done), the four LEDs are on, both LTSSMs
         are in L0 and both Data Link Layers in DL_Active;
      5. of the sticky counters only the reset-release and the two link-up
         counters moved, by one each: no link-down, Recovery entry, replay,
         retrain request, Nak or receive error;
      6. the six event times are latched, in a possible order;
      7. pipe_fabric_link answered each receiver-detection request once,
         DETECT_LATENCY + 1 edges later, with RxStatus 011b only then, and
         held RxElecIdle for exactly IDLE_EXIT_EDGES edges after the reset.
    """
    cocotb.start_soon(Clock(dut.clk125_p, CLK_NS, units="ns").start())
    con = Console(dut)
    await con.clear()
    watch = await bring_up(dut, con)
    report(dut, "first", con, watch)
    check_bring_up(dut, con, watch, releases=1)
    tm = con.times()
    dut._log.info("FL_ROW first release: link up rc=%d ep=%d, FC rc=%d ep=%d, scan done %d, "
                  "enum done %d (clk125 cycles from the release)",
                  tm["rc_linkup"], tm["ep_linkup"], tm["rc_fc"], tm["ep_fc"],
                  tm["scan_done"], tm["enum_done"])


@cocotb.test()
async def fl_reset_release_is_repeatable(dut):
    """The console can cycle the reset, and each cycle is the same bring-up.

    After a counter clear, three resets: two with run held at 0 for 40
    cycles, the third with run at 0 for a single cycle. Each must pass every
    claim of the first test, with the counters at one more release and one
    more link-up per end and nothing else. The third shows that the top
    stretches a short request to at least RST_MIN_CYCLES of reset. The six
    event times must be the same in all three: the bring-up is a function of
    the reset release alone.
    """
    cocotb.start_soon(Clock(dut.clk125_p, CLK_NS, units="ns").start())
    con = Console(dut)
    con.run(0)
    await ClockCycles(dut.clk125_p, 40)
    await con.clear()
    assert all(v == 0 for v in con.counters().values()), (
        f"the clear left a counter: {con.counters()}")

    seen = []
    for n, low in ((1, None), (2, None), (3, 1)):
        watch = await bring_up(dut, con, low=low)
        report(dut, f"cycle{n}", con, watch)
        check_bring_up(dut, con, watch, releases=n)
        if low is not None:
            assert watch.rst_edges >= RST_MIN_CYCLES, (
                f"a {low}-cycle run = 0 gave {watch.rst_edges} edges of reset, "
                f"fewer than RST_MIN_CYCLES = {RST_MIN_CYCLES}")
        seen.append(con.times())
    assert seen[0] == seen[1] == seen[2], f"the event times differ between resets: {seen}"

    await con.clear()
    assert all(v == 0 for v in con.counters().values()), (
        f"the clear left a counter: {con.counters()}")
    st = con.status()
    assert st["rc_link_up"] and st["ep_link_up"] and st["enum_done"], (
        f"the counter clear disturbed the link: {st}")
    dut._log.info("FL_ROW three resets, each %s", seen[0])
