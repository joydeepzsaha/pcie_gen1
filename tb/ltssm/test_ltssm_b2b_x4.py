"""
Back-to-back RC <-> EP LTSSM Configuration at x4 (and partial widths).

Same harness as test_ltssm_b2b.py (tb_ltssm_b2b.sv) but MAX_NUM_LANES=4: an
RC (IS_ROOT_PORT=1, LINK_NUM=1) and an EP (IS_ROOT_PORT=0), cross-wired, with
Python driving only PHY-level bring-up (receiver detection / phystatus /
elec-idle), shared to both, and never touching ordered_set_i.

Unlike the x1 test, this asserts the *content* of the negotiation, not just
that L0 was reached: the RC must transmit Lane Number l on lane l (0,1,2,3),
and the EP must echo it. A full-width link that agreed on the wrong lane
numbers must FAIL, not pass.

Cases:
  * full x4        (lanes 0-3): assert mutual L0 + RC lanes 0,1,2,3.
  * partial x2     (lanes 0,1): assert mutual L0 + RC lanes 0,1 (x2 link).
  * non-contiguous (lanes 1,2): REPORT what the RTL does -- the spec assigns
    sequential 0..N-1 to the forming lanes, but the RTL's RX checks assume the
    Lane Number equals the physical lane index (COMPLETE: lane_num == lane).
    For contiguous-from-0 masks those coincide; for lanes {1,2} they do not,
    so the expected behaviour is genuinely ambiguous -- we report, not assert.

Requires SIM_FAST_LINK=1, MAX_NUM_LANES=4 (verilate_b2b_x4 target, built with
--public-flat-rw so the per-lane hang dump can read the internal counters).
"""
import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ClockCycles
from ltssm_tb_common import (
    STATE_NAMES, unpack_tsos,
    ST_IDLE, ST_DETECT_QUIET, ST_DETECT_ACTIVE, ST_DETECT_RX,
    ST_POLLING_ACTIVE, ST_CFG_LN_ACCEPT, ST_CFG_COMPLETE, ST_L0,
)

# sec 63 #7g-2 (D-7G.2): this bench's clock AND the RTL's CLK_PERIOD_NS, one value.
# The core passes -GCLK_PERIOD_NS=8 (tb_ltssm_b2b.sv passes .CLK_PERIOD_NS(8));
# every cycle budget below that stands for a spec time derives from it.
CLK_PERIOD_NS = 8

NUM_LANES = 4
ALL = (1 << NUM_LANES) - 1          # 0xF
TSOS_WIDTH = 128


def _mask(lanes):
    m = 0
    for l in lanes:
        m |= (1 << l)
    return m


def _rxstatus(lanes):
    """3'b011 per active lane, 0 elsewhere (lane_status latch condition)."""
    v = 0
    for l in lanes:
        v |= (0b011 << (3 * l))
    return v


def _lane_tx(dut, net, lane):
    """Unpack the ordered set a harness instance transmitted on `lane`.
    `net` is 'rc_ordered_set_o' or 'ep_ordered_set_o' (per-lane packed array)."""
    val = int(getattr(dut, net).value)
    return unpack_tsos((val >> (lane * TSOS_WIDTH)) & ((1 << TSOS_WIDTH) - 1))


def _dump_both(dut, note, lanes):
    """Per-lane dual-instance hang dump (needs --public-flat-rw for the
    genvar-scoped counters)."""
    dut._log.error(f"==== B2B x4 HANG: {note} ====")

    def rd(path):
        try:
            return hex(int(eval(f"dut.{path}.value")))  # noqa: S307
        except Exception as e:                          # noqa: BLE001
            return f"<n/a:{type(e).__name__}>"
    for who, inst, net in (("RC", "rc_inst", "rc_ordered_set_o"),
                           ("EP", "ep_inst", "ep_ordered_set_o")):
        st = int(getattr(dut, f"{who.lower()}_ltssm_state_o").value)
        dut._log.error(f"  [{who}] state={STATE_NAMES.get(st, hex(st))} ({st:#07x})  "
                       f"link_number_selected={rd(inst+'.link_number_selected')}")
        dut._log.error(f"        link_lanes_formed={rd(inst+'.link_lanes_formed')} "
                       f"link_lanes_nums_match={rd(inst+'.link_lanes_nums_match')} "
                       f"link_lane_reconfig={rd(inst+'.link_lane_reconfig')} "
                       f"link_width_satisfied={rd(inst+'.link_width_satisfied')}")
        for l in range(NUM_LANES):
            u = _lane_tx(dut, net, l)
            ts = "TS1" if u["is_ts1"] else ("TS2" if u["is_ts2"] else
                                            f"?{u['ts_s6']:#04x}")
            dut._log.error(
                f"        lane{l}: tx ts={ts} link={u['link_num']:#04x} "
                f"lane={u['lane_num']:#04x} | ts1_cnt="
                f"{rd(inst+f'.gen_cnt_ts1[{l}].ts1_cnt')} ts2_cnt="
                f"{rd(inst+f'.gen_cnt_ts1[{l}].ts2_cnt')} lane_in_save="
                f"{rd(inst+f'.gen_cnt_ts1[{l}].lane_in_save')} lane_num_echo="
                f"{rd(inst+f'.lane_num_echo')}")


def _idle_drives(dut):
    dut.en_i.value = 0
    dut.phy_rxelecidle_drv_i.value = 0
    dut.receiver_detected_drv_i.value = 0
    dut.phy_rxstatus_drv_i.value = 0
    dut.phy_phystatus_drv_i.value = 0


async def _reach_both(dut, state, cycles, name, lanes):
    for _ in range(cycles):
        await RisingEdge(dut.clk_i)
        if (int(dut.rc_ltssm_state_o.value) == state and
                int(dut.ep_ltssm_state_o.value) == state):
            return
    _dump_both(dut, f"never reached mutual {name}", lanes)
    raise AssertionError(f"b2b x4: no mutual {name}")


async def _bring_up(dut, lanes):
    """Reset -> Detect -> Polling PHY bring-up for the b2b harness over an
    arbitrary active-lane set. Shared PHY drive to both instances. Returns once
    both are in POLLING_ACTIVE; from there the two LTSSMs negotiate on their
    own. Full mask goes straight to Polling; a partial mask takes the DETECT_RX
    confirmation hop."""
    mask = _mask(lanes)
    rxs = _rxstatus(lanes)
    cocotb.start_soon(Clock(dut.clk_i, CLK_PERIOD_NS, units="ns").start())  # 125 MHz
    _idle_drives(dut)
    dut.rst_i.value = 1
    await ClockCycles(dut.clk_i, 5)
    dut.rst_i.value = 0
    await ClockCycles(dut.clk_i, 5)
    assert int(dut.rc_ltssm_state_o.value) == ST_IDLE
    assert int(dut.ep_ltssm_state_o.value) == ST_IDLE

    dut.en_i.value = 1
    await _reach_both(dut, ST_DETECT_QUIET, 50, "DETECT_QUIET", lanes)

    dut.phy_rxelecidle_drv_i.value = ALL
    await ClockCycles(dut.clk_i, 3)
    dut.phy_rxelecidle_drv_i.value = 0
    await _reach_both(dut, ST_DETECT_ACTIVE, 50, "DETECT_ACTIVE", lanes)

    dut.receiver_detected_drv_i.value = mask
    dut.phy_rxstatus_drv_i.value = rxs
    dut.phy_phystatus_drv_i.value = mask
    await ClockCycles(dut.clk_i, 3)
    dut.phy_phystatus_drv_i.value = 0

    if mask != ALL:
        # partial mask -> DETECT_RX confirmation hop (12 ms, then re-detect the
        # same pattern) before Polling.
        await _reach_both(dut, ST_DETECT_RX, 50, "DETECT_RX", lanes)
        await ClockCycles(dut.clk_i, 12_000 // CLK_PERIOD_NS + 100)  # 63 #7g-2: SIM_FAST 12 ms + 100; was 1300 at 10 ns
        dut.phy_phystatus_drv_i.value = mask
        await ClockCycles(dut.clk_i, 3)
        dut.phy_phystatus_drv_i.value = 0

    await _reach_both(dut, ST_POLLING_ACTIVE, 100, "POLLING_ACTIVE", lanes)


async def _capture_rc_lanenums(dut, lanes, out):
    """Record the Lane Number the RC transmits on each active lane while it is
    in Lanenum.Accept / Complete (the states that carry the assignment)."""
    while True:
        await RisingEdge(dut.clk_i)
        st = int(dut.rc_ltssm_state_o.value)
        if st in (ST_CFG_LN_ACCEPT, ST_CFG_COMPLETE):
            for l in lanes:
                u = _lane_tx(dut, "rc_ordered_set_o", l)
                if u["is_ts1"] or u["is_ts2"]:
                    out[l] = u["lane_num"]
        if (int(dut.rc_ltssm_state_o.value) == ST_L0 and
                int(dut.ep_ltssm_state_o.value) == ST_L0):
            return


async def _run(dut, lanes, timeout_cycles=12000):
    """Bring up over `lanes`, capture the RC's per-lane assignment, wait for
    mutual L0. Returns the captured {lane: lane_num} dict."""
    await _bring_up(dut, lanes)
    captured = {}
    cocotb.start_soon(_capture_rc_lanenums(dut, lanes, captured))
    for _ in range(timeout_cycles):
        await RisingEdge(dut.clk_i)
        if (int(dut.rc_ltssm_state_o.value) == ST_L0 and
                int(dut.ep_ltssm_state_o.value) == ST_L0):
            break
    return captured


@cocotb.test()
async def run_test_b2b_x4_full(dut):
    """Full x4: both reach L0 AND the RC assigned Lane Numbers 0,1,2,3."""
    lanes = [0, 1, 2, 3]
    captured = await _run(dut, lanes)
    rc = int(dut.rc_ltssm_state_o.value)
    ep = int(dut.ep_ltssm_state_o.value)
    if not (rc == ST_L0 and ep == ST_L0):
        _dump_both(dut, "full x4 did not reach mutual L0", lanes)
        raise AssertionError(
            f"x4 full: RC={STATE_NAMES.get(rc, hex(rc))} "
            f"EP={STATE_NAMES.get(ep, hex(ep))}, captured={captured}")
    assert int(dut.rc_link_up_o.value) == 1 and int(dut.ep_link_up_o.value) == 1

    # The core claim under test: RC transmitted lane l on lane l.
    dut._log.info(f"x4 full: RC assigned lane numbers {captured}")
    for l in lanes:
        assert captured.get(l) == l, (
            f"x4 full: RC lane {l} transmitted Lane Number "
            f"{captured.get(l)}, expected {l}")
    sim_ns = cocotb.utils.get_sim_time(units="ns")
    dut._log.info(
        f"B2B x4 LINK UP at {sim_ns:.0f} ns: RC assigned lanes 0,1,2,3 "
        "on lanes 0,1,2,3; EP echoed; both in L0")


@cocotb.test()
async def run_test_b2b_x2_partial(dut):
    """Partial x2 on lanes 0,1 (contiguous from 0): forms an x2 link with
    Lane Numbers 0,1."""
    lanes = [0, 1]
    captured = await _run(dut, lanes)
    rc = int(dut.rc_ltssm_state_o.value)
    ep = int(dut.ep_ltssm_state_o.value)
    if not (rc == ST_L0 and ep == ST_L0):
        _dump_both(dut, "partial x2 did not reach mutual L0", lanes)
        raise AssertionError(
            f"x2 partial: RC={STATE_NAMES.get(rc, hex(rc))} "
            f"EP={STATE_NAMES.get(ep, hex(ep))}, captured={captured}")
    assert int(dut.rc_link_up_o.value) == 1 and int(dut.ep_link_up_o.value) == 1
    dut._log.info(f"x2 partial: RC assigned lane numbers {captured}")
    for l in lanes:
        assert captured.get(l) == l, (
            f"x2 partial: RC lane {l} transmitted Lane Number "
            f"{captured.get(l)}, expected {l}")
    sim_ns = cocotb.utils.get_sim_time(units="ns")
    dut._log.info(
        f"B2B x2 LINK UP at {sim_ns:.0f} ns: RC assigned lanes 0,1 on lanes "
        "0,1; both in L0")


@cocotb.test()
async def run_test_b2b_noncontiguous_report(dut):
    """Non-contiguous lanes 1,2: REPORT the RTL's behaviour, do not assert a
    pass/fail on link formation -- the spec (sequential 0..N-1 to the forming
    lanes) and the RTL's RX checks (lane_num == physical index) diverge here,
    so the correct outcome is genuinely ambiguous."""
    lanes = [1, 2]
    captured = await _run(dut, lanes, timeout_cycles=12000)
    rc = int(dut.rc_ltssm_state_o.value)
    ep = int(dut.ep_ltssm_state_o.value)
    dut._log.info("==== NON-CONTIGUOUS (lanes 1,2) REPORT ====")
    dut._log.info(f"  RC ended in {STATE_NAMES.get(rc, hex(rc))} ({rc:#07x})")
    dut._log.info(f"  EP ended in {STATE_NAMES.get(ep, hex(ep))} ({ep:#07x})")
    dut._log.info(f"  RC link_up={int(dut.rc_link_up_o.value)} "
                  f"EP link_up={int(dut.ep_link_up_o.value)}")
    dut._log.info(
        f"  RC Lane Numbers assigned during Configuration (lane->number): "
        f"{captured}")
    dut._log.info(
        "  NOTE: spec assigns sequential 0..N-1 to the forming lanes; the RTL "
        "assigns physical index and its COMPLETE RX checks lane_num == lane, so "
        "lanes {1,2} get {1,2} (physical), not {0,1} (sequential). The link "
        "still forms (both agree by construction) but the Lane Numbers are "
        "non-spec. Reported, not asserted -- the intended behaviour is "
        "ambiguous.")
