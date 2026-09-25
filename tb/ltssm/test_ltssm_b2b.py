"""
Back-to-back RC <-> EP LTSSM link-up (x1).

Two pcie_ltssm_downstream instances inside tb_ltssm_b2b.sv -- one
IS_ROOT_PORT=1 (LINK_NUM=1), one IS_ROOT_PORT=0 -- are cross-wired to each
other through the harness shim. Python drives ONLY PHY-level bring-up
(receiver detection, phystatus, elec-idle exit), shared to both sides, and
then waits. It never touches ordered_set_i on either instance: the two state
machines negotiate Detect -> Polling -> Configuration -> L0 entirely on their
own. Both must reach L0 with link_up asserted.

This is the first check of the IS_ROOT_PORT=1 path against an independently
parameterised EP instance rather than against a Python model of the peer.
Requires SIM_FAST_LINK=1, MAX_NUM_LANES=1 (verilate_b2b target).
"""
import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ClockCycles
from ltssm_tb_common import (
    STATE_NAMES, unpack_tsos,
    ST_IDLE, ST_DETECT_QUIET, ST_DETECT_ACTIVE, ST_POLLING_ACTIVE, ST_L0,
)

# sec 63 #7g-2 (D-7G.2): this bench's clock AND the RTL's CLK_PERIOD_NS, one value.
# The core passes -GCLK_PERIOD_NS=8 (tb_ltssm_b2b.sv passes .CLK_PERIOD_NS(8));
# every cycle budget below that stands for a spec time derives from it.
CLK_PERIOD_NS = 8

LANE0 = 0x1          # x1 lane mask
RXSTATUS_OK = 0b011  # one lane, "receiver ready"


def _snap(dut, who):
    """Unpacked ordered_set_o + internal exit-condition signals for one side."""
    inst = "rc_inst" if who == "RC" else "ep_inst"
    os_net = "rc_ordered_set_o" if who == "RC" else "ep_ordered_set_o"
    st = int(getattr(dut, f"{who.lower()}_ltssm_state_o").value)

    def rd(path):
        try:
            return hex(int(eval(f"dut.{path}.value")))  # noqa: S307
        except Exception as e:                          # noqa: BLE001
            return f"<n/a:{type(e).__name__}>"
    u = unpack_tsos(int(getattr(dut, os_net).value))
    ts = "TS1" if u["is_ts1"] else ("TS2" if u["is_ts2"] else f"?{u['ts_s6']:#04x}")
    dut._log.error(
        f"  [{who}] state={STATE_NAMES.get(st, hex(st))} ({st:#07x})  "
        f"tx: ts={ts} link={u['link_num']:#04x} lane={u['lane_num']:#04x}")
    dut._log.error(
        f"       link_number_selected={rd(inst+'.link_number_selected')} "
        f"lane_in_save={rd(inst+'.gen_cnt_ts1[0].lane_in_save')} "
        f"ts1_cnt={rd(inst+'.gen_cnt_ts1[0].ts1_cnt')} "
        f"ts2_cnt={rd(inst+'.gen_cnt_ts1[0].ts2_cnt')}")
    dut._log.error(
        f"       link_lanes_formed={rd(inst+'.link_lanes_formed')} "
        f"link_lanes_nums_match={rd(inst+'.link_lanes_nums_match')} "
        f"link_width_satisfied={rd(inst+'.link_width_satisfied')}")


def _dump_both(dut, note):
    dut._log.error(f"==== B2B HANG: {note} ====")
    _snap(dut, "RC")
    _snap(dut, "EP")


async def _state_trace(dut):
    """Log every state change on both instances until both reach L0."""
    rc_prev = ep_prev = None
    while True:
        await RisingEdge(dut.clk_i)
        rc = int(dut.rc_ltssm_state_o.value)
        ep = int(dut.ep_ltssm_state_o.value)
        t = cocotb.utils.get_sim_time(units="ns")
        if rc != rc_prev:
            dut._log.info(f"{t:8.0f}ns  RC -> {STATE_NAMES.get(rc, hex(rc))}")
            rc_prev = rc
        if ep != ep_prev:
            dut._log.info(f"{t:8.0f}ns  EP -> {STATE_NAMES.get(ep, hex(ep))}")
            ep_prev = ep
        if rc == ST_L0 and ep == ST_L0:
            return


async def _wait_both_l0(dut, timeout_cycles):
    """Wait until both instances are in L0. On timeout, dump both and fail."""
    for _ in range(timeout_cycles):
        await RisingEdge(dut.clk_i)
        if (int(dut.rc_ltssm_state_o.value) == ST_L0 and
                int(dut.ep_ltssm_state_o.value) == ST_L0):
            return
    _dump_both(dut, f"not both in L0 after {timeout_cycles} cycles")
    raise AssertionError("b2b did not reach mutual L0")


@cocotb.test()
async def run_test_b2b_linkup(dut):
    cocotb.start_soon(Clock(dut.clk_i, CLK_PERIOD_NS, units="ns").start())  # 125 MHz

    # ---- idle all PHY drives, reset both instances ----
    dut.en_i.value = 0
    dut.phy_rxelecidle_drv_i.value = 0
    dut.receiver_detected_drv_i.value = 0
    dut.phy_rxstatus_drv_i.value = 0
    dut.phy_phystatus_drv_i.value = 0
    dut.rst_i.value = 1
    await ClockCycles(dut.clk_i, 5)
    dut.rst_i.value = 0
    await ClockCycles(dut.clk_i, 5)

    assert int(dut.rc_ltssm_state_o.value) == ST_IDLE, "RC not in IDLE after reset"
    assert int(dut.ep_ltssm_state_o.value) == ST_IDLE, "EP not in IDLE after reset"
    assert int(dut.rc_link_up_o.value) == 0
    assert int(dut.ep_link_up_o.value) == 0

    # ---- enable both; PHY drives shared to both sides ----
    cocotb.start_soon(_state_trace(dut))
    dut.en_i.value = 1

    # IDLE -> DETECT_QUIET (both)
    for _ in range(50):
        await RisingEdge(dut.clk_i)
        if (int(dut.rc_ltssm_state_o.value) == ST_DETECT_QUIET and
                int(dut.ep_ltssm_state_o.value) == ST_DETECT_QUIET):
            break
    else:
        _dump_both(dut, "never reached mutual DETECT_QUIET")
        raise AssertionError("no DETECT_QUIET")

    # DETECT_QUIET -> DETECT_ACTIVE via elec-idle exit edge (1 -> 0)
    dut.phy_rxelecidle_drv_i.value = LANE0
    await ClockCycles(dut.clk_i, 3)
    dut.phy_rxelecidle_drv_i.value = 0
    for _ in range(50):
        await RisingEdge(dut.clk_i)
        if (int(dut.rc_ltssm_state_o.value) == ST_DETECT_ACTIVE and
                int(dut.ep_ltssm_state_o.value) == ST_DETECT_ACTIVE):
            break
    else:
        _dump_both(dut, "never reached mutual DETECT_ACTIVE")
        raise AssertionError("no DETECT_ACTIVE")

    # DETECT_ACTIVE -> POLLING: receiver detected + phystatus pulse.
    # rxstatus must read 3'b011 before the phystatus pulse (lane_active latch).
    dut.receiver_detected_drv_i.value = LANE0
    dut.phy_rxstatus_drv_i.value = RXSTATUS_OK
    dut.phy_phystatus_drv_i.value = LANE0
    await ClockCycles(dut.clk_i, 3)
    dut.phy_phystatus_drv_i.value = 0
    for _ in range(100):
        await RisingEdge(dut.clk_i)
        if (int(dut.rc_ltssm_state_o.value) == ST_POLLING_ACTIVE and
                int(dut.ep_ltssm_state_o.value) == ST_POLLING_ACTIVE):
            break
    else:
        _dump_both(dut, "never reached mutual POLLING_ACTIVE")
        raise AssertionError("no POLLING_ACTIVE")

    # ---- from here the two LTSSMs negotiate on their own (no Python) ----
    await _wait_both_l0(dut, 8000)

    # hold: both must stay in L0 with link_up
    await ClockCycles(dut.clk_i, 50)
    assert int(dut.rc_ltssm_state_o.value) == ST_L0, "RC fell out of L0"
    assert int(dut.ep_ltssm_state_o.value) == ST_L0, "EP fell out of L0"
    assert int(dut.rc_link_up_o.value) == 1, "RC link_up not asserted"
    assert int(dut.ep_link_up_o.value) == 1, "EP link_up not asserted"

    sim_ns = cocotb.utils.get_sim_time(units="ns")
    dut._log.info(
        f"B2B LINK UP at {sim_ns:.0f} ns: RC (IS_ROOT_PORT=1) and "
        f"EP (IS_ROOT_PORT=0) both reached L0 with no protocol driving")
