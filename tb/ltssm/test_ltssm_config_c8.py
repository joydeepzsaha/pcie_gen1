"""
Configuration.Linkwidth.Accept adds an ordered-set gate the spec does not have.

Base 2.1 Rev 2.1, Section 4.2.6.3.2.1 (p.230), Downstream Lanes:

    "If a configured Link can be formed with at least one group of Lanes that
     received two consecutive TS1 Ordered Sets with the same received Link
     number (non-PAD and matching one that was transmitted by the Downstream
     Lanes), TS1 Ordered Sets are transmitted with the same Link number and
     unique non-PAD Lane numbers are assigned to all these same Lanes. The
     next state is Configuration.Lanenum.Wait."

The forming condition is TWO CONSECUTIVE matching TS1 and nothing else. There
is no "N Ordered Sets transmitted" requirement anywhere in this substate --
unlike Polling.Active (1024) or Configuration.Complete (16 after receiving
one), where the spec states such a count explicitly.

The RTL implements the two-consecutive part correctly --
link_lanes_formed[lane] <= (ts1_cnt >= 8'h2) -- and then added a second,
unsourced gate:

          if ((|link_lanes_formed) &&
          ordered_set_sent_cnt_r >= 8'h08)

STATUS: FIXED (fix-arc 6b). The unsourced conjunct was removed and the exit is
now the forming condition alone; the expect_fail marker came off in the same
commit (rule 22.75 -- a marker left on reports `failed` for a passing row, and
rule 22.77's corollary: until it comes off, the gate cannot witness the revert
mutant either). Prediction and pre-edit census:
evidence/fix-arc-6/PREDICTIONS_C8.md. Mutant MC8 restores the conjunct.

WHAT THIS TEST MEASURES
Walk to Configuration.Linkwidth.Accept as the Root Complex, echo the matching
TS1 that satisfies the spec's forming condition, and count ordered-set
transmissions until the FSM advances to Configuration.Lanenum.Wait.

  Spec-conformant DUT: advances on the two consecutive TS1, so at most a pulse
                       or two of slack.
  Pre-fix-arc-6b DUT:  waited for a further eight transmissions.

The assertion states the spec value (oracle C8, evidence/rung10/ORACLES_LTSSM.md).
It was an expect_fail row recording the divergence until fix-arc 6b closed it.

SEVERITY, pre-committed in evidence/rung10/PREDICTIONS_R10B_D7_P3_C8.md: this
is a CONSERVATIVE divergence -- it delays, it never skips -- so unless the
delay can push the substate past its own 2 ms watchdog at :851 it is low
severity, unlike P4 (compounds with a slow partner) or C26a (a liveness
defect). The measured pulse count below is what decides that: eight
transmissions at roughly four cycles each is ~32 cycles against a 200 000
cycle watchdog, three orders of magnitude of headroom.

Requires SIM_FAST_LINK=1, MAX_NUM_LANES=1, IS_ROOT_PORT=1, LINK_NUM=1
(verilate_config_c8 target).
"""
import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, RisingEdge, Timer
from ltssm_tb_common import *  # noqa

# sec 63 #7g-2 (D-7G.2): this bench's clock AND the RTL's CLK_PERIOD_NS, one value.
# The core passes -GCLK_PERIOD_NS=8 (tb_ltssm_b2b.sv passes .CLK_PERIOD_NS(8));
# every cycle budget below that stands for a spec time derives from it.
CLK_PERIOD_NS = 8

LINK = LINK_NUM

# The spec-conformant budget: the forming condition is two consecutive TS1,
# which the per-lane counter consumes at one per clock -- far quicker than the
# ~4-cycle transmit-pulse period -- so a conformant DUT needs a pulse or two.
SPEC_BUDGET = 2
CEILING = 200          # hard stop so a hang fails loudly


class TxCounter:
    """Counts rising edges of ordered_set_tranmitted_i, so the free-running
    os_tx_pulser can still supply the handshake while this test measures it."""

    def __init__(self):
        self.n = 0


async def count_tx(dut, ctr):
    prev = 0
    while True:
        await RisingEdge(dut.clk_i)
        await Timer(1, units="ps")
        cur = int(dut.ordered_set_tranmitted_i.value)
        if cur and not prev:
            ctr.n += 1
        prev = cur


def state(dut):
    return int(dut.ltssm_state_o.value)


def sname(s):
    return f"{STATE_NAMES.get(s, hex(s))} ({s:#07x})"


@cocotb.test()
async def run_test_config_c8(dut):
    cocotb.start_soon(Clock(dut.clk_i, CLK_PERIOD_NS, units="ns").start())

    n_bits = len(dut.ordered_set_i)
    assert n_bits == 128, (
        f"-GMAX_NUM_LANES=1 did not reach the DUT: ordered_set_i is {n_bits} "
        f"bits (x1 expects 128)")

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

    ctr = TxCounter()
    cocotb.start_soon(os_tx_pulser(dut))
    cocotb.start_soon(count_tx(dut, ctr))

    await wait_state(dut, ST_POLLING_ACTIVE, 200, "POLLING_ACTIVE")
    dut.ordered_set_i.value = pack_tsos(link_num=None, lane_num=None)
    dut.ts1_valid_i.value = LANE0_MASK
    await wait_state(dut, ST_POLLING_CONFIG, 4000, "POLLING_CONFIGURATION")

    dut.ts1_valid_i.value = 0
    dut.ts2_valid_i.value = LANE0_MASK
    await wait_state(dut, ST_CFG_LW_START, 4000, "CFG_LINKWIDTH_START")

    # Linkwidth.Start -> Linkwidth.Accept: echo the RC's own Link number with a
    # PAD Lane number, which is what p.227 requires to advance.
    dut.ts2_valid_i.value = 0
    dut.ordered_set_i.value = pack_tsos(link_num=LINK, lane_num=None)
    dut.ts1_valid_i.value = LANE0_MASK
    await wait_state(dut, ST_CFG_LW_ACCEPT, 8000, "CFG_LINKWIDTH_ACCEPT")

    # ---- the measurement ----
    # The echo is already the matching TS1, so the spec's "two consecutive TS1
    # with the same non-PAD Link number" is satisfied within a couple of clocks
    # of entry -- well inside one transmit-pulse period.
    start = ctr.n
    pulses = 0
    while state(dut) == ST_CFG_LW_ACCEPT and pulses < CEILING:
        await RisingEdge(dut.clk_i)
        await Timer(1, units="ps")
        pulses = ctr.n - start

    reached = state(dut)
    dut._log.info(
        f"left Configuration.Linkwidth.Accept after {pulses} ordered-set "
        f"transmissions, into {sname(reached)}")

    assert reached == ST_CFG_LN_WAIT, (
        f"expected Configuration.Lanenum.Wait, got {sname(reached)}")

    assert pulses <= SPEC_BUDGET, (
        f"C8 (Base 2.1 4.2.6.3.2.1, p.230): the Linkwidth.Accept exit "
        f"condition is 'two consecutive TS1 Ordered Sets with the same "
        f"received Link number' and nothing more -- the substate states no "
        f"transmitted-Ordered-Set count, unlike Polling.Active (1024) or "
        f"Configuration.Complete (16 after receiving one). The forming "
        f"condition was met on entry, so a conformant DUT advances within "
        f"{SPEC_BUDGET} transmissions; this DUT took {pulses}, matching the "
        f"unsourced `ordered_set_sent_cnt_r >= 8'h08` gate that fix-arc 6b "
        f"removed from ST_CONFIGURATION_LINKWIDTH_ACCEPT's exit.")
