"""
The two oracles behind the unscaled 24 ms watchdog: D11 and P6.

TwentyFourMsTimeOut (:109) is NOT scaled by SIM_FAST_LINK
(evidence/rung10/CENSUS_LTSSM.md section 6), so each test here runs a real
2 400 000 cycles. That is why they are split into their own target: the cost is
visible per-row rather than buried, and the 10b close can weigh it.

D11  4.2.6.1.2 p.219 -- Detect.Active's exits are stated exhaustively: Polling
     if a Receiver is detected on all unconfigured Lanes, Detect.Quiet if on
     none, and the wait-12ms-and-retry path if on some but not all. THE SPEC
     GIVES THIS SUBSTATE NO TIMEOUT. :594 adds a 24 ms watchdog to ST_IDLE.

     This is NOT an expect_fail. Rung 10a classified it as the single
     "extra-spec watchdog (conformant-but-added)": the spec has no escape if
     the PHY never completes a detection, so a defensive bound is an addition
     rather than a violation. The test therefore CHARACTERISES it, and brackets
     the constant from both sides so the 24 ms value is pinned rather than the
     mere existence of a timeout observed -- the same discipline P12 uses.

P6   4.2.6.2.1 p.221 -- the 24 ms Polling.Active branch reaches
     Polling.Configuration only if BOTH:
       (i)  any Lane that detected a Receiver got eight consecutive training
            sequences ... and 1024 TS1 transmitted after receiving one; AND
       (ii) "At least a predetermined number of Lanes that detected a Receiver
            during Detect have detected an exit from Electrical Idle at least
            once since entering Polling.Active."
     and otherwise, per (a) on the same page, goes to Polling.Compliance if
     "less than the predetermined number of Lanes from (ii) above have detected
     an exit from Electrical Idle since entering Polling.Active".

     Limb (ii) WAS ABSENT from the RTL: the 24 ms branch tested only
     lanes_ts1_satisfied / lanes_ts2_satisfied, and the LTSSM sampled
     phy_rxelecidle_exit_detected in exactly one place -- inside Detect.Quiet --
     and never during Polling at all.

     STATUS: FIXED (fix-arc 6b). A polling_ei_exit_seen_r register now
     accumulates Electrical Idle exits WHILE IN Polling.Active (cleared on
     leaving it, not merely on reset -- every bench toggles phy_rxelecidle_i
     during Detect.Quiet, so a reset-only clear would make the fix inert), and
     the 24 ms branch requires it. The expect_fail marker came off in the same
     commit (rule 22.75).

     *** The p6 row now lands in ST_IDLE via ST_POLLING_COMPLIANCE rather than
     in Polling.Configuration, and that is a PREDICTED consequence, not a
     surprise: adding the conjunct breaks the subsumption that made the
     else-if arm below it dead code, so ST_POLLING_COMPLIANCE becomes reachable
     for the first time in this design and lands exactly where p.221(a) says.
     Oracle P7's "unreachable" verdict is superseded. Its body is still "not
     implemented" (error_c, then ST_IDLE), and :720's guard is still an
     over-approximation because training_ctrl_t has no Compliance Receive
     member. Both are registered as owed. See
     evidence/fix-arc-6/PREDICTIONS_P6.md section 4. Mutants MP6 (drop the
     conjunct) and MP6b (clear the accumulator on reset only -- the inertness
     probe).

WHY P6 IS NOT VACUOUS, stated explicitly because it tests an ABSENCE:
  * It runs at x4 and drives eight consecutive TS1 on lanes 0b0011 only, so the
    PRIMARY exit at :660 (&lanes_ts1_satisfied, "all Lanes") is blocked. That
    blocking is not assumed -- it was measured last increment by the R63 test
    in test_ltssm_x4_oracles.py, and a mutation of :660 kills that row.
  * With the primary exit blocked, the ONLY way out toward
    Polling.Configuration is the 24 ms branch at :672-685, which is the branch
    limb (ii) belongs to.
  * phy_rxelecidle_i is never toggled after entering Polling.Active, so ZERO
    lanes have "detected an exit from Electrical Idle since entering
    Polling.Active" and p.221(a) routes to Polling.Compliance.
    (The bring-up does toggle it during DETECT -- but the spec condition is
    measured since entering Polling.Active, and in any case the RTL only reads
    that signal inside Detect.Quiet.)

  So an advance to Polling.Configuration can only mean limb (ii) was not
  consulted.

  Note the destination the spec wants, Polling.Compliance, is itself
  structurally unreachable in this design (evidence/rung10/CENSUS_LTSSM.md
  section 2.1) -- so as with oracle P3 there is no correct destination
  available even in principle. This row pins the near half.

Requires SIM_FAST_LINK=1, MAX_NUM_LANES=4 (verilate_ltssm_24ms target).
"""
import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, RisingEdge, Timer
from ltssm_tb_common import *  # noqa

# sec 63 #7g-2 (D-7G.2): this bench's clock AND the RTL's CLK_PERIOD_NS, one value.
# The core passes -GCLK_PERIOD_NS=8 (tb_ltssm_b2b.sv passes .CLK_PERIOD_NS(8));
# every cycle budget below that stands for a spec time derives from it.
CLK_PERIOD_NS = 8

X4 = 0xF
SUBSET = 0b0011

# pcie_ltssm_downstream.sv:109 -- (24 * 10**6) / ClockPeriodNs, unscaled.
TWENTY_FOUR_MS = 24_000_000 // CLK_PERIOD_NS   # 63 #7g-2: was 2_400_000 (10 ns); same ratios below
EARLY_CHECK = TWENTY_FOUR_MS * 47 // 48        # ~2% below: a grossly-short timer already fired
LATE_BOUND = TWENTY_FOUR_MS * 13 // 12         # ~8% above
POLL = 512


def state(dut):
    return int(dut.ltssm_state_o.value)


def sname(s):
    return f"{STATE_NAMES.get(s, hex(s))} ({s:#07x})"


def clk(dut):
    cocotb.start_soon(Clock(dut.clk_i, CLK_PERIOD_NS, units="ns").start())
    assert len(dut.ordered_set_i) == 4 * TSOS_WIDTH, \
        "-GMAX_NUM_LANES=4 did not reach the DUT; P6 is vacuous at x1"


async def reset_to_detect_active(dut):
    drive_idle_inputs(dut)
    dut.rst_i.value = 1
    await ClockCycles(dut.clk_i, 5)
    dut.rst_i.value = 0
    await ClockCycles(dut.clk_i, 5)
    dut.en_i.value = 1
    await wait_state(dut, ST_DETECT_QUIET, 50, "DETECT_QUIET")
    dut.phy_rxelecidle_i.value = X4
    await ClockCycles(dut.clk_i, 3)
    dut.phy_rxelecidle_i.value = 0
    await wait_state(dut, ST_DETECT_ACTIVE, 50, "DETECT_ACTIVE")


# =====================================================================
# D11 -- Detect.Active's extra-spec 24 ms watchdog, characterised
# =====================================================================
@cocotb.test()
async def run_test_d11_detect_active_watchdog(dut):
    clk(dut)
    await reset_to_detect_active(dut)

    # Never complete a detection: phy_phystatus_i stays 0, so |phy_phystatus_r
    # is false and the spec's three exits are all unavailable. Under Base 2.1
    # the FSM would wait here indefinitely; this RTL bounds it.
    dut.phy_phystatus_i.value = 0
    dut.receiver_detected_i.value = 0
    dut._log.info(f"in Detect.Active with no detection ever completing; the "
                  f"spec gives this substate no timeout, the RTL adds one at "
                  f"~{TWENTY_FOUR_MS} cycles (:594)")

    waited = 0
    while waited < EARLY_CHECK:
        await ClockCycles(dut.clk_i, POLL)
        waited += POLL
        if state(dut) != ST_DETECT_ACTIVE:
            raise AssertionError(
                f"the Detect.Active watchdog fired after only ~{waited} "
                f"cycles, into {sname(state(dut))}. :594 uses "
                f"TwentyFourMsTimeOut = {TWENTY_FOUR_MS} cycles; anything this "
                f"early means the constant is not what the census recorded.")
    dut._log.info(f"still in Detect.Active at ~{waited} cycles "
                  f"({100.0 * waited / TWENTY_FOUR_MS:.1f}% of 24 ms)")

    landed = None
    while waited < LATE_BOUND:
        await ClockCycles(dut.clk_i, POLL)
        waited += POLL
        if state(dut) != ST_DETECT_ACTIVE:
            await Timer(1, units="ps")
            landed = state(dut)
            break

    assert landed is not None, (
        f"still in Detect.Active at ~{waited} cycles; the 24 ms watchdog at "
        f":594 did not fire by the {LATE_BOUND}-cycle bound")

    dut._log.info(
        f"D11 CHARACTERISED: Detect.Active's extra-spec watchdog fired at "
        f"~{waited} cycles ({100.0 * waited / TWENTY_FOUR_MS:.1f}% of 24 ms) "
        f"into {sname(landed)}. Base 2.1 p.219 states no timeout for this "
        f"substate; this is an addition, not a violation -- without it a PHY "
        f"that never completes a detection would hang the LTSSM forever.")


# =====================================================================
# P6 -- the 24 ms Polling.Active branch ignores the Electrical Idle limb
# =====================================================================
async def reach_polling_active_x4(dut, pulsed):
    """Reach Polling.Active with either a realistic pulsed TX handshake or one
    held continuously high. Which of the two is used decides whether the 24 ms
    branch at :672 can execute at all -- see run_test_p6a below."""
    await reset_to_detect_active(dut)
    dut.receiver_detected_i.value = X4
    dut.phy_rxstatus_i.value = RXSTATUS_ALL_OK
    dut.phy_phystatus_i.value = X4
    await ClockCycles(dut.clk_i, 3)
    dut.phy_phystatus_i.value = 0
    if pulsed:
        cocotb.start_soon(os_tx_pulser(dut))
    else:
        dut.ordered_set_tranmitted_i.value = 1
    await wait_state(dut, ST_POLLING_ACTIVE, 300, "POLLING_ACTIVE")


@cocotb.test()
async def run_test_p6a_24ms_branch_loses_the_race(dut):
    """The 24 ms Polling.Active success branch is unreachable with a realistic
    pulsed TX handshake -- the unconditional watchdog beats it to the cycle.

    This test did not exist when the increment was planned. It was written
    after oracle P6's prediction was REFUTED: P6 predicted an advance to
    Polling.Configuration and the FSM went to Detect instead. Per the standing
    rule that a refuted prediction requires measurement rather than a new
    guess, this pins the cause.

    Both exits are evaluated in the same always_comb pass:

      :649  if (ordered_set_tranmitted_i) begin
      :672    if ((timer_r >= TwentyFourMsTimeOut) && (cnt >= MinTS1sPolling))
      :677      if (|lanes_ts1_satisfied || ...)  next_state = POLLING_CONFIG
      ...
      :714  if ((timer_r >= TwentyFourMsTimeOut) && (next_state == curr_state))
      :715    next_state = ST_IDLE;

    :714 is OUTSIDE the handshake gate and fires on the very first cycle
    timer_r reaches the threshold. :672 is INSIDE it. os_tx_pulser drives the
    handshake high one cycle in four, so unless that pulse lands exactly on the
    threshold cycle, :714's guard (next_state == curr_state) is still true and
    the watchdog claims the transition. The 24 ms success branch then never
    runs, because the state change resets timer_r.

    Consequence: :672-:704 -- the whole 24 ms Polling.Configuration branch,
    the Polling.Compliance arm at :689, and the error_c at :702 -- is
    effectively DEAD for any TX handshake that is not continuously asserted.
    """
    clk(dut)
    await reach_polling_active_x4(dut, pulsed=True)

    dut.phy_rxelecidle_i.value = 0
    dut.ordered_set_i.value = pack_tsos_all_lanes(link_num=None, lane_num=None)
    dut.ts1_valid_i.value = SUBSET

    waited = 0
    landed = None
    while waited < LATE_BOUND:
        await ClockCycles(dut.clk_i, POLL)
        waited += POLL
        if state(dut) != ST_POLLING_ACTIVE:
            await Timer(1, units="ps")
            landed = state(dut)
            break

    assert landed is not None, "never left Polling.Active"
    dut._log.info(f"P6a: with a PULSED handshake, left Polling.Active at "
                  f"~{waited} cycles into {sname(landed)}")

    assert landed != ST_POLLING_CONFIG, (
        f"expected the :714 watchdog to win the race and send us to Detect, "
        f"but the FSM reached Polling.Configuration -- the pulse must have "
        f"aligned with the threshold cycle. Re-run; if this is stable the "
        f"race analysis in this docstring is wrong.")
    dut._log.info(
        "P6a MEASURED: the unconditional 24 ms watchdog at :714 beat the "
        "handshake-gated success branch at :672. With a pulsed TX handshake "
        "the entire :672-:704 block -- including the Polling.Compliance arm "
        "and the error_c at :702 -- cannot execute.")


@cocotb.test()
async def run_test_p6_polling_elec_idle_limb(dut):
    clk(dut)
    # Hold the TX handshake continuously high so the 24 ms branch at :672 CAN
    # execute (see run_test_p6a: with a pulsed handshake it never does, and
    # this oracle would be untestable rather than divergent).
    await reach_polling_active_x4(dut, pulsed=False)

    # Eight consecutive TS1 on a strict subset: the primary exit at :660 is
    # blocked (measured by R63 in test_ltssm_x4_oracles.py), so only the 24 ms
    # branch can advance us. Electrical idle is held steady -- no lane detects
    # an EI exit at any point after entering Polling.Active.
    dut.phy_rxelecidle_i.value = 0
    dut.ordered_set_i.value = pack_tsos_all_lanes(link_num=None, lane_num=None)
    dut.ts1_valid_i.value = SUBSET
    dut._log.info(
        f"Polling.Active: TS1 on lanes {SUBSET:#06b} of {X4:#06b} (primary "
        f"exit blocked), TX handshake held CONTINUOUSLY HIGH so the 24 ms "
        f"branch at :672 can execute, phy_rxelecidle_i held steady so ZERO "
        f"lanes have seen an Electrical Idle exit since entering this substate")

    waited = 0
    landed = None
    while waited < LATE_BOUND:
        await ClockCycles(dut.clk_i, POLL)
        waited += POLL
        if state(dut) != ST_POLLING_ACTIVE:
            await Timer(1, units="ps")
            landed = state(dut)
            break

    assert landed is not None, (
        f"never left Polling.Active within {LATE_BOUND} cycles; expected the "
        f"24 ms branch at :672 to resolve one way or the other")

    err = int(dut.error_o.value)
    dut._log.info(f"P6: left Polling.Active at ~{waited} cycles "
                  f"({100.0 * waited / TWENTY_FOUR_MS:.1f}% of 24 ms) into "
                  f"{sname(landed)}, error_o = {err}")

    assert landed != ST_POLLING_CONFIG, (
        f"P6 (Base 2.1 4.2.6.2.1, p.221): the 24 ms Polling.Active branch "
        f"reaches Polling.Configuration only if, in addition to the training "
        f"sequence limb, 'at least a predetermined number of Lanes that "
        f"detected a Receiver during Detect have detected an exit from "
        f"Electrical Idle at least once since entering Polling.Active'. Zero "
        f"lanes did so here, which per (a) on the same page routes to "
        f"Polling.Compliance -- yet the DUT advanced to Polling.Configuration "
        f"at ~{waited} cycles. Limb (ii) is absent from the RTL: the 24 ms "
        f"branch tests only lanes_ts1_satisfied/lanes_ts2_satisfied, and "
        f"phy_rxelecidle_exit_detected is sampled at exactly one site, inside "
        f"Detect.Quiet, never during Polling.")

    # *** WHICH ARM CAUGHT IT -- added fix-arc 6b, and it is not a formality. ***
    # `landed != ST_POLLING_CONFIG` alone is satisfied by TWO different
    # mechanisms that this coarse (POLL=512) loop cannot tell apart, because
    # both end in ST_IDLE and then bounce straight to Detect.Quiet:
    #
    #   (a) the spec route -- the Electrical Idle limb blocks the success arm,
    #       the else-if below it sends us to ST_POLLING_COMPLIANCE, whose body
    #       raises error_c and goes to ST_IDLE;
    #   (b) the fallback -- the 24 ms branch does nothing at all and the
    #       unconditional watchdog claims the transition to ST_IDLE.
    #
    # (b) is what run_test_p6a measures, and it lands in exactly the same state.
    # So without this check the row would pass on a DUT where the fix did
    # nothing. error_o discriminates: the watchdog does NOT raise error_c, and
    # ST_POLLING_COMPLIANCE does. error_r is sticky and clears only on rst_i, so
    # a 1 here means Polling.Compliance was entered.
    #
    # This also MEASURES the consequence predicted in
    # evidence/fix-arc-6/PREDICTIONS_P6.md section 4: adding the limb breaks the
    # subsumption that made the Polling.Compliance arm dead code, so oracle P7's
    # "structurally unreachable" verdict is superseded. Without this assertion
    # that claim would rest on reading the RTL rather than on a measurement.
    assert err == 1, (
        f"P6: the FSM left Polling.Active into {sname(landed)} without raising "
        f"error_o. Both the Polling.Compliance route and the bare 24 ms "
        f"watchdog land there, and only the former raises error_c -- so "
        f"error_o == 0 means the Electrical Idle limb did NOT divert this to "
        f"Polling.Compliance and the row is passing for the wrong reason.")
