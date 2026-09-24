"""
Configuration substates that Rung 10a predicted CONFORM -- tested, with controls.

Rung 10a read Base 2.1 section 4.2.6 against the RTL and predicted 30 of 49
oracles conform. Every oracle Rung 10b had executed before this file diverged,
because the divergences were deliberately tested first. That left the
conformance half of the prediction resting on unexecuted reading. This file
attacks the other side: four Configuration oracles predicted to conform.

  C3  4.2.6.3.1.1 p.227 -- Linkwidth.Start -> Linkwidth.Accept "immediately
      after any of those same Downstream Lanes receive two consecutive TS1
      Ordered Sets with a non-PAD Link number that matches any of the
      transmitted Link numbers, and with a Lane number set to PAD (K23.7)".
  C11 4.2.6.3.4.1 p.234 -- Lanenum.Wait -> Lanenum.Accept if Lanes "receive two
      consecutive TS1 Ordered Sets which have a Lane number different from when
      the Lane first entered Configuration.Lanenum.Wait, and not all the Lanes'
      Link numbers are set to PAD".
  C14 4.2.6.3.3.1 p.232 -- Lanenum.Accept -> Complete "if two consecutive TS1
      Ordered Sets are received with non-PAD Link and non-PAD Lane numbers that
      match all the non-PAD Link and non-PAD Lane numbers ... being transmitted".
  C25 4.2.6.3.6 p.237 -- Configuration.Idle -> L0 "if eight consecutive Symbol
      Times of Idle data are received on all configured Lanes and 16 Idle data
      Symbols are sent after receiving one Idle data Symbol".

EVERY TEST HERE SHIPS A NEGATIVE CONTROL, and that is the point of the file.
Rung 10a found four assertions in this very testbench directory that cannot
fail (they read error_o, which is never driven). A "conforms" test is the
easiest place in the world to add a fifth: drive the link, watch it come up,
assert it came up. So each oracle below is exercised twice from independent
resets -- once with the spec condition deliberately UNMET, asserting the DUT
does NOT advance, and once with it met, asserting it does. If a negative
control fails to bite, the positive result means nothing and must be reported
as measuring nothing.

Counters are reset on every state change (:1659-1665), and a mismatching TS1
SATURATES rather than clearing once ts1_cnt has reached 2 (:1833, :1870), so
negative and positive cases are run from separate resets rather than
back-to-back inside one walk.

Requires SIM_FAST_LINK=1, MAX_NUM_LANES=1, IS_ROOT_PORT=1, LINK_NUM=1
(verilate_config_conforms target).
"""
import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, RisingEdge, Timer
from ltssm_tb_common import *  # noqa

# sec 63 #7g-2 (D-7G.2): this bench's clock AND the RTL's CLK_PERIOD_NS, one value.
# The core passes -GCLK_PERIOD_NS=8 (tb_ltssm_b2b.sv passes .CLK_PERIOD_NS(8));
# every cycle budget below that stands for a spec time derives from it.
CLK_PERIOD_NS = 8

LINK = LINK_NUM              # 0x01, the Link Number the RC originates
WRONG_LINK = 0x5A            # non-PAD and not LINK -- must be rejected
WRONG_LANE = 0x01            # RC assigns lane 0 at x1, so 1 must be rejected

# How long a negative control watches for an advance that must not happen.
# Comfortably longer than any positive case needs (those complete in tens of
# cycles) and far short of the 2 ms / 200 000-cycle substate watchdogs, so a
# "did not advance" result cannot be a timeout in disguise.
NEG_WATCH = 5000


def state(dut):
    return int(dut.ltssm_state_o.value)


def sname(s):
    return f"{STATE_NAMES.get(s, hex(s))} ({s:#07x})"


async def hold(dut, ts, **echo):
    """Present one ordered set on the RX side with the given strobe."""
    dut.ordered_set_i.value = pack_tsos(**echo)
    dut.ts1_valid_i.value = LANE0_MASK if ts == "ts1" else 0
    dut.ts2_valid_i.value = LANE0_MASK if ts == "ts2" else 0
    dut.idle_valid_i.value = LANE0_MASK if ts == "idle" else 0


async def must_not_advance(dut, from_state, what):
    """Negative control: assert the FSM stays put for NEG_WATCH cycles."""
    for _ in range(NEG_WATCH):
        await RisingEdge(dut.clk_i)
        await Timer(1, units="ps")
        if state(dut) != from_state:
            raise AssertionError(
                f"NEGATIVE CONTROL FAILED -- {what}: the DUT advanced out of "
                f"{sname(from_state)} into {sname(state(dut))} on a stimulus "
                f"that does not satisfy the spec condition. Either the RTL is "
                f"more permissive than Base 2.1, or this control is not "
                f"actually withholding the condition.")
    dut._log.info(f"negative control bit: {what} -- stayed in "
                  f"{sname(from_state)} for {NEG_WATCH} cycles")


async def fresh_walk(dut, stop_at):
    """Reset, then walk Detect -> Polling -> Configuration, RETURNING as soon
    as `stop_at` is entered and BEFORE any stimulus that would leave it."""
    drive_idle_inputs(dut)
    dut.rst_i.value = 1
    await ClockCycles(dut.clk_i, 5)
    dut.rst_i.value = 0
    await ClockCycles(dut.clk_i, 5)

    dut.en_i.value = 1
    await wait_state(dut, ST_DETECT_QUIET, 50, "DETECT_QUIET")
    dut.phy_rxelecidle_i.value = LANE0_MASK
    await ClockCycles(dut.clk_i, 3)
    dut.phy_rxelecidle_i.value = 0
    await wait_state(dut, ST_DETECT_ACTIVE, 50, "DETECT_ACTIVE")

    dut.receiver_detected_i.value = LANE0_MASK
    dut.phy_rxstatus_i.value = RXSTATUS_OK_X1
    dut.phy_phystatus_i.value = LANE0_MASK
    await ClockCycles(dut.clk_i, 3)
    dut.phy_phystatus_i.value = 0
    cocotb.start_soon(os_tx_pulser(dut))
    await wait_state(dut, ST_POLLING_ACTIVE, 200, "POLLING_ACTIVE")
    if stop_at == ST_POLLING_ACTIVE:
        return

    await hold(dut, "ts1", link_num=None, lane_num=None)
    await wait_state(dut, ST_POLLING_CONFIG, 4000, "POLLING_CONFIGURATION")
    if stop_at == ST_POLLING_CONFIG:
        return

    await hold(dut, "ts2", link_num=None, lane_num=None)
    await wait_state(dut, ST_CFG_LW_START, 4000, "CFG_LINKWIDTH_START")
    if stop_at == ST_CFG_LW_START:
        return

    await hold(dut, "ts1", link_num=LINK, lane_num=None)
    await wait_state(dut, ST_CFG_LW_ACCEPT, 8000, "CFG_LINKWIDTH_ACCEPT")
    if stop_at == ST_CFG_LW_ACCEPT:
        return

    # Linkwidth.Accept latches lane_in_save from this echo (:1868); it is PAD
    # here, which is what C11's "different from when the Lane first entered
    # Lanenum.Wait" is measured against.
    await hold(dut, "ts1", link_num=LINK, lane_num=None)
    await wait_state(dut, ST_CFG_LN_WAIT, 8000, "CFG_LANENUM_WAIT")
    if stop_at == ST_CFG_LN_WAIT:
        return

    await hold(dut, "ts1", link_num=LINK, lane_num=0)
    await wait_state(dut, ST_CFG_LN_ACCEPT, 8000, "CFG_LANENUM_ACCEPT")
    if stop_at == ST_CFG_LN_ACCEPT:
        return

    await hold(dut, "ts1", link_num=LINK, lane_num=0)
    await wait_state(dut, ST_CFG_COMPLETE, 8000, "CFG_COMPLETE")
    if stop_at == ST_CFG_COMPLETE:
        return

    await hold(dut, "ts2", link_num=LINK, lane_num=0)
    await wait_state(dut, ST_CFG_IDLE, 8000, "CFG_IDLE")
    if stop_at == ST_CFG_IDLE:
        return

    raise AssertionError(f"fresh_walk: unsupported stop_at {sname(stop_at)}")


def clk(dut):
    cocotb.start_soon(Clock(dut.clk_i, CLK_PERIOD_NS, units="ns").start())
    n_bits = len(dut.ordered_set_i)
    assert n_bits == 128, (
        f"-GMAX_NUM_LANES=1 did not reach the DUT: ordered_set_i is {n_bits} "
        f"bits (x1 expects 128)")


# =====================================================================
# C3 -- Linkwidth.Start -> Linkwidth.Accept
# =====================================================================
@cocotb.test()
async def run_test_c3_linkwidth_start(dut):
    clk(dut)

    # Negative control (a): non-PAD Link number that does NOT match the one the
    # RC transmits. p.227 requires a match; :1826 tests
    # `link_num == link_number_selected`.
    await fresh_walk(dut, ST_CFG_LW_START)
    await hold(dut, "ts1", link_num=WRONG_LINK, lane_num=None)
    await must_not_advance(dut, ST_CFG_LW_START,
                           f"C3(a) Link={WRONG_LINK:#04x} does not match the "
                           f"transmitted Link={LINK:#04x}")

    # Negative control (b): correct Link but a non-PAD Lane number. p.227
    # requires "a Lane number set to PAD (K23.7)"; :1828 tests it.
    await fresh_walk(dut, ST_CFG_LW_START)
    await hold(dut, "ts1", link_num=LINK, lane_num=0)
    await must_not_advance(dut, ST_CFG_LW_START,
                           "C3(b) Lane number is non-PAD, spec requires PAD")

    # Positive: matching non-PAD Link, PAD Lane.
    await fresh_walk(dut, ST_CFG_LW_START)
    await hold(dut, "ts1", link_num=LINK, lane_num=None)
    await wait_state(dut, ST_CFG_LW_ACCEPT, 8000,
                     "CFG_LINKWIDTH_ACCEPT (C3 positive)")
    dut._log.info("C3 CONFORMS: advanced on matching Link + PAD Lane, and both "
                  "negative controls held")


# =====================================================================
# C11 -- Lanenum.Wait -> Lanenum.Accept
# =====================================================================
@cocotb.test()
async def run_test_c11_lanenum_wait(dut):
    clk(dut)

    # Negative control: a Lane number EQUAL to the one latched when the lane
    # entered (lane_in_save == PAD, captured at :1868 during Linkwidth.Accept).
    # p.234 is explicit that the Lane number "must have changed from when the
    # Lanes most recently entered Configuration.Lanenum.Wait".
    await fresh_walk(dut, ST_CFG_LN_WAIT)
    await hold(dut, "ts1", link_num=LINK, lane_num=None)   # PAD == lane_in_save
    await must_not_advance(dut, ST_CFG_LN_WAIT,
                           "C11 Lane number unchanged from entry (still PAD)")

    # Positive: a Lane number different from entry, Link not PAD.
    await fresh_walk(dut, ST_CFG_LN_WAIT)
    await hold(dut, "ts1", link_num=LINK, lane_num=0)
    await wait_state(dut, ST_CFG_LN_ACCEPT, 8000,
                     "CFG_LANENUM_ACCEPT (C11 positive)")
    dut._log.info("C11 CONFORMS: advanced only once the Lane number changed "
                  "from its entry value")


# =====================================================================
# C14 -- Lanenum.Accept -> Configuration.Complete
# =====================================================================
@cocotb.test()
async def run_test_c14_lanenum_accept(dut):
    clk(dut)

    # Negative control: the RC assigned Lane 0 (x1), so an echo of Lane 1 does
    # not "match all the non-PAD ... Lane numbers being transmitted" (p.232).
    await fresh_walk(dut, ST_CFG_LN_ACCEPT)
    await hold(dut, "ts1", link_num=LINK, lane_num=WRONG_LANE)
    await must_not_advance(dut, ST_CFG_LN_ACCEPT,
                           f"C14 Lane={WRONG_LANE} does not match the assigned "
                           f"Lane 0")

    # Positive: exact echo of both assigned numbers.
    await fresh_walk(dut, ST_CFG_LN_ACCEPT)
    await hold(dut, "ts1", link_num=LINK, lane_num=0)
    await wait_state(dut, ST_CFG_COMPLETE, 8000,
                     "CFG_COMPLETE (C14 positive)")
    dut._log.info("C14 CONFORMS: advanced only on an exact Link+Lane echo")


# =====================================================================
# C25 -- Configuration.Idle -> L0
# =====================================================================
@cocotb.test()
async def run_test_c25_config_idle_to_l0(dut):
    clk(dut)

    # Negative control: withhold Idle data entirely. p.237's exit needs eight
    # consecutive Symbol Times of Idle; with none, L0 must not be reached.
    # NEG_WATCH is far short of the 2 ms timeout, so this cannot be the
    # timeout path in disguise.
    await fresh_walk(dut, ST_CFG_IDLE)
    await hold(dut, "none")
    await must_not_advance(dut, ST_CFG_IDLE,
                           "C25 no Idle data received at all")

    # Positive: drive Idle data.
    await fresh_walk(dut, ST_CFG_IDLE)
    await hold(dut, "idle", link_num=None, lane_num=None)
    await wait_state(dut, ST_L0, 8000, "L0 (C25 positive)")
    assert int(dut.link_up_o.value) == 1, "link_up_o not asserted in L0"
    dut._log.info("C25 CONFORMS: reached L0 on Idle data, and did not without it")
