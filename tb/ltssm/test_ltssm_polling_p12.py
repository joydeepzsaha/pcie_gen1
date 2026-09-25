"""
Polling.Configuration's 48 ms timeout: oracle P12, predicted to CONFORM.

Base 2.1 Rev 2.1, Section 4.2.6.2.3 (p.225):

    "Otherwise, next state is Detect after a 48 ms timeout."

pcie_ltssm_downstream.sv:753 uses FourtyEightMsTimeOut, defined at :110 as
(48 * 10**6) / ClockPeriodNs = 4 800 000 cycles at CLK_RATE=100.

WHY THE NEGATIVE CONTROL IS THE LOAD-BEARING HALF
Asserting only "the DUT eventually reaches Detect" would pass for ANY timeout
value -- 1 ms, 48 ms or 480 ms -- and would therefore prove nothing about the
constant. That is exactly the tautology class Rung 10a found four instances of
in this directory. So the test brackets the constant from BOTH sides:

  * still in Polling.Configuration at EARLY_CHECK cycles  -> not early;
  * has left by LATE_BOUND cycles                          -> not late.

Together those pin the timer to the 48 ms neighbourhood rather than merely
observing that a timeout exists.

COST, stated in the pre-registered predictions: FourtyEightMsTimeOut is NOT
scaled by SIM_FAST_LINK (evidence/rung10/CENSUS_LTSSM.md section 6), so this
row runs a real 4.8 M cycles -- roughly eight minutes of wall clock. It gets
its own target so that cost is visible in the per-target profile rather than
buried inside a multi-test row, and so the 10b close can decide on cost grounds
whether to keep it in the gate.

Requires SIM_FAST_LINK=1, MAX_NUM_LANES=1 (verilate_polling_p12 target).
"""
import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, RisingEdge, Timer
from ltssm_tb_common import *  # noqa

# sec 63 #7g-2 (D-7G.2): this bench's clock AND the RTL's CLK_PERIOD_NS, one value.
# The core passes -GCLK_PERIOD_NS=8 (tb_ltssm_b2b.sv passes .CLK_PERIOD_NS(8));
# every cycle budget below that stands for a spec time derives from it.
CLK_PERIOD_NS = 8

# pcie_ltssm_downstream.sv:110 -- (48 * 10**6) / ClockPeriodNs, unscaled.
FORTY_EIGHT_MS_CYCLES = 48_000_000 // CLK_PERIOD_NS   # 63 #7g-2: was 4_800_000 (10 ns)

# Bracket. EARLY_CHECK sits ~2 % below the constant: late enough that a
# grossly-short timer (say 24 ms, or a SIM_FAST_LINK-scaled one) would already
# have fired, early enough not to race the real expiry.
EARLY_CHECK = FORTY_EIGHT_MS_CYCLES * 47 // 48
LATE_BOUND = FORTY_EIGHT_MS_CYCLES * 17 // 16   # ~6 % above; a timer materially longer fails here

POLL = 512                    # coarse poll; both states here persist far longer

# Any Detect-family state counts as "went to Detect": ST_IDLE is the RTL's
# stand-in and with en_i high it moves on to ST_DETECT_QUIET within a cycle.
DETECT_FAMILY = {ST_IDLE, ST_DETECT_WAIT_ONE_MS, ST_DETECT_QUIET,
                 ST_DETECT_ACTIVE, ST_DETECT_RX}


def state(dut):
    return int(dut.ltssm_state_o.value)


def sname(s):
    return f"{STATE_NAMES.get(s, hex(s))} ({s:#07x})"


@cocotb.test()
async def run_test_p12_polling_config_timeout(dut):
    cocotb.start_soon(Clock(dut.clk_i, CLK_PERIOD_NS, units="ns").start())
    assert len(dut.ordered_set_i) == 128, "-GMAX_NUM_LANES=1 did not reach the DUT"

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

    dut.ordered_set_i.value = pack_tsos(link_num=None, lane_num=None)
    dut.ts1_valid_i.value = LANE0_MASK
    await wait_state(dut, ST_POLLING_CONFIG, 4000, "POLLING_CONFIGURATION")

    # ---- withhold TS2 entirely so only the 48 ms timer can end the state ----
    dut.ts1_valid_i.value = 0
    dut.ts2_valid_i.value = 0
    dut.idle_valid_i.value = 0
    dut.ordered_set_i.value = 0
    dut._log.info(
        f"in Polling.Configuration with no TS2; expecting Detect at "
        f"~{FORTY_EIGHT_MS_CYCLES} cycles (48 ms at 125 MHz, unscaled)")

    # ---- negative control: must NOT have fired early ----
    waited = 0
    while waited < EARLY_CHECK:
        await ClockCycles(dut.clk_i, POLL)
        waited += POLL
        if state(dut) != ST_POLLING_CONFIG:
            raise AssertionError(
                f"NEGATIVE CONTROL FAILED: left Polling.Configuration after "
                f"only ~{waited} cycles, into {sname(state(dut))}. Base 2.1 "
                f"p.225 specifies a 48 ms timeout = {FORTY_EIGHT_MS_CYCLES} "
                f"cycles at 100 MHz; anything this early means the constant "
                f"is wrong or a different exit fired.")
    dut._log.info(
        f"negative control bit: still in Polling.Configuration at ~{waited} "
        f"cycles ({100.0 * waited / FORTY_EIGHT_MS_CYCLES:.1f}% of 48 ms)")

    # ---- positive: must fire by the late bound ----
    landed = None
    while waited < LATE_BOUND:
        await ClockCycles(dut.clk_i, POLL)
        waited += POLL
        if state(dut) != ST_POLLING_CONFIG:
            await Timer(1, units="ps")
            landed = state(dut)
            break

    assert landed is not None, (
        f"still in Polling.Configuration at ~{waited} cycles; the 48 ms "
        f"timeout ({FORTY_EIGHT_MS_CYCLES} cycles) did not fire by the "
        f"{LATE_BOUND}-cycle bound")

    dut._log.info(
        f"left Polling.Configuration at ~{waited} cycles into {sname(landed)} "
        f"({100.0 * waited / FORTY_EIGHT_MS_CYCLES:.1f}% of the nominal 48 ms)")

    assert landed in DETECT_FAMILY, (
        f"Base 2.1 4.2.6.2.3 p.225: 'Otherwise, next state is Detect after a "
        f"48 ms timeout.' The DUT went to {sname(landed)} instead.")

    dut._log.info("P12 CONFORMS: Polling.Configuration timed out to Detect, "
                  "and the timer is bracketed to the 48 ms neighbourhood from "
                  "both sides")
