"""
Rung 10c, Item 2 -- Recovery oracles R9, R13, R14, R15.

ORACLE SOURCE
  evidence/rung10/ORACLES_LTSSM.md section O-R, derived from PCI Express Base
  Specification Rev 2.1 section 4.2.6.4 "Recovery": RcvrCfg is 4.2.6.4.3
  (printed p.243-245), Idle is 4.2.6.4.4 (p.245-246).

WHY THIS TARGET IS x4
  R9 and R13 are ANY-vs-ALL reduction defects. At MAX_NUM_LANES=1 a `|` and a
  `&` over a one-bit vector are the same expression, so an x1 target would pass
  no matter which the RTL used -- the Rung 1 LTSSM_ARCS.md section 6.3 class.
  Both rows present stimulus on lane 0 ONLY while all four lanes are configured,
  which is the only way to tell the two reductions apart. check_geometry()
  asserts the width first so a -G that failed to reach the DUT fails the row
  instead of faking a pass.

THE FOUR DIVERGENCES as first recorded in Rung 10c, one expect_fail row each
(10b rule: one divergent assertion per row, never mixed with a conforming one).
Each entry carries its own STATUS line; a row whose defect has been fixed keeps
its assertion and loses its marker.

  R9  (p.244) "Next state is Recovery.Idle if eight consecutive TS2 Ordered
      Sets are received on ALL configured Lanes..."
      The exit reduced with `|(ts2_cnt_satisfied & lane_active_r)` -- ANY
      active Lane.
      STATUS: FIXED (fix-arc 1, Phase 3B) -- :1269 now reduces with a bare
      `&ts2_cnt_satisfied`, which is the spec form because ts2_cnt_satisfied is
      already lane-gated at :1613; the explicit `& lane_active_r` had to go
      with the `|`, or every inactive Lane would zero the reduction.

  R13 (p.246) "Next state is L0 if eight consecutive Symbol Times of Idle data
      are received on ALL configured Lanes and 16 Idle data Symbols are sent
      after receiving one Idle data Symbol."
      The exit reduced with `|lanes_idle_satisfied` -- ANY Lane. And unlike its
      siblings, lanes_idle_satisfied was NOT gated by lane_active_r, so the fix
      was two changes, not one.
      STATUS: FIXED (fix-arc 1, Phase 3A) -- :1471 reduces with `&` and :1623
      gates the operand, in one commit. The half-fix (reduction without the
      gate) hangs a reduced-width link; gate row 63
      verilate_recovery_partial_lanes is the oracle that catches it, and a
      mutant confirmed it fails 2 of its 3 tests with `never reached L0 (via
      Recovery)` rather than hanging the harness.

  R14a (p.245) "...two CONSECUTIVE TS1 Ordered Sets..."
      The exit at :1476 fires on at_least_one_ts1_ts2, which in this state is
      armed by ts2_cnt != 0 -- a threshold of ONE, not two. This row drives a
      single TS1 with Lane = PAD (both spec-correct) so the count limb is the
      only thing under test.

  R14b (p.245) "...two consecutive TS1 Ordered Sets..." -- the TYPE limb.
      :1844 arms the same counter on (ts1_valid_i || ts2_valid_i), so a TS2
      also sends the link back to Configuration. This row drives TWO TS2 with
      Lane = PAD (spec-correct count, spec-correct Lane) so the type limb is
      the only thing under test.

      NOTE: the Lane-number limb of R14 CONFORMS -- :1845 requires
      ordered_set_i[lane].lane_num == PAD before the counter moves at all. An
      earlier draft of the oracle claimed it was missing; it is not. Both rows
      below therefore use PAD Lane numbers, which is what makes them isolate
      the count and type limbs rather than accidentally re-testing the Lane
      check.

  R15 (p.246) "Otherwise, after a 2 ms timeout: if the idle_to_rlock_transitioned
      variable is 0b, the next state is Recovery.RcvrLock. The
      idle_to_rlock_transitioned variable is set to 1b upon transitioning to
      Recovery.RcvrLock. Else the next state is Detect."
      The variable is binary, so the SECOND 2 ms timeout must reach Detect.
      :1457's guard is `!= '1`, i.e. != 8'hFF, and at Gen1 the arm below it
      INCREMENTED instead of saturating -- so it took 255 timeouts, not one.
      This was C26a's mirror image, and unlike C26a it was live at Gen1 with no
      precondition. See ORACLES_LTSSM.md R15a.
      STATUS: FIXED (fix-arc 1, Phase 1) -- the Gen1 arm now saturates, as its
      own Gen2 arm two lines below always did. Marker removed in the fix commit
      (rule 22.75). Flipping it did NOT change the T/A row: cocotb reports an
      expect_fail raise as STATUS=PASS, and the test diverged on the very cycle
      it now conforms on. Only the raw log moved, from "passed: failed as
      expected" to "passed".

NEGATIVE CONTROL
  test_recovery_idle_reached is an ordinary PASS row running the identical
  drive sequence from an independent reset. An expect_fail row goes green if
  ANYTHING in it raises -- including a broken setup. If the control is red, the
  expect_fail rows above prove nothing and are void.

Requires SIM_FAST_LINK=1, MAX_NUM_LANES=4 (verilate_ltssm_ridle target).
"""
import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, RisingEdge, Timer
from ltssm_tb_common import *  # noqa

# sec 63 #7g-2 (D-7G.2): this bench's clock AND the RTL's CLK_PERIOD_NS, one value.
# The core passes -GCLK_PERIOD_NS=8 (tb_ltssm_b2b.sv passes .CLK_PERIOD_NS(8));
# every cycle budget below that stands for a spec time derives from it.
CLK_PERIOD_NS = 8

LANE0 = 0x1

# TwoMsTimeOut is NOT SIM_FAST_LINK-scaled: (2*10**6)/ClockPeriodNs with
# ClockPeriodNs=10 => 200_000 cycles (pcie_ltssm_downstream.sv:113). Budget one
# timeout plus slack; R15 waits for two of them.
TWO_MS_CYCLES = 2_000_000 // CLK_PERIOD_NS   # 63 #7g-2: was 200_000 (10 ns)
TIMEOUT_SLACK = 20_000

# Long enough for idle_cnt to saturate (8) and ordered_set_sent_cnt to pass 16
# under os_tx_pulser's 1-in-4 duty cycle, with generous slack.
SETTLE_CYCLES = 400


def state(dut):
    return int(dut.ltssm_state_o.value)


def sname(s):
    return STATE_NAMES.get(s, hex(s))


def check_geometry(dut):
    n = len(dut.ordered_set_i)
    assert n == 512, (
        f"-GMAX_NUM_LANES=4 did not reach the DUT: ordered_set_i is {n} bits "
        f"(x4 expects 4*128=512); an ANY-vs-ALL row at x1 would be vacuous")


async def drive_to_rcvr_cfg(dut):
    """Reset -> L0 -> Recovery.RcvrLock -> Recovery.RcvrCfg. Returns in RcvrCfg.

    Mirrors test_ltssm_recovery.py's sequence: partner-initiated retrain with
    no speed change (rate stays gen1, speed_change bit 0 throughout).
    """
    cocotb.start_soon(Clock(dut.clk_i, CLK_PERIOD_NS, units="ns").start())
    check_geometry(dut)
    await bring_up_link(dut)
    assert state(dut) == ST_L0, f"setup did not reach L0, got {sname(state(dut))}"

    dut.ordered_set_i.value = pack_tsos_all_lanes(
        link_num=LINK_NUM, lane_num="index", rate=GEN1_RATE, speed_change=0)
    dut.ts1_valid_i.value = ALL
    await wait_state(dut, ST_RECOVERY_RCVR_LOCK, 100, "RECOVERY_RCVR_LOCK")
    await wait_state(dut, ST_RECOVERY_RCVR_CFG, 500, "RECOVERY_RCVR_CFG")
    dut.ts1_valid_i.value = 0


async def drive_to_recovery_idle(dut):
    """...and on into Recovery.Idle by presenting TS2 on ALL lanes."""
    await drive_to_rcvr_cfg(dut)
    dut.ts2_valid_i.value = ALL
    await wait_state(dut, ST_RECOVERY_IDLE, 2000, "RECOVERY_IDLE")
    dut.ts2_valid_i.value = 0
    # The global transition reset (:1631 onward) clears every per-lane count on
    # the state change, so we enter with ts1_cnt = ts2_cnt = idle_cnt = 0.


# ==========================================================================
#  Negative control -- ordinary PASS row.
# ==========================================================================

@cocotb.test()
async def test_recovery_idle_reached(dut):
    """Control: the shared drive sequence really does reach Recovery.Idle.

    Without this, every expect_fail row below could be green because the setup
    broke rather than because the DUT diverged.
    """
    await drive_to_recovery_idle(dut)
    assert state(dut) == ST_RECOVERY_IDLE, (
        f"control failed: expected RECOVERY_IDLE, got {sname(state(dut))}")
    dut._log.info("CONTROL OK: reached RECOVERY_IDLE with all 4 lanes configured")


# ==========================================================================
#  R9 -- RcvrCfg -> Recovery.Idle must require ALL configured Lanes.
# ==========================================================================

@cocotb.test()
async def test_r9_rcvrcfg_exit_requires_all_lanes(dut):
    """R9 (p.244): 8 consecutive TS2 on ALL configured Lanes.

    Four lanes are configured; TS2 is presented on lane 0 only. A conforming
    DUT stays in RcvrCfg. Before fix-arc 1 this DUT left, because the exit
    reduced with `|`. STATUS: FIXED (fix-arc 1, Phase 3B) -- :1269 now reduces
    with a bare `&`, which is the spec form here because ts2_cnt_satisfied is
    already lane-gated at :1613; the `& lane_active_r` had to go with the `|`,
    or every inactive Lane would zero the reduction.
    """
    await drive_to_rcvr_cfg(dut)

    dut.ts2_valid_i.value = LANE0          # lane 0 only, 3 lanes silent
    await ClockCycles(dut.clk_i, SETTLE_CYCLES)
    await Timer(1, units="ps")

    got = state(dut)
    dut._log.info(
        f"R9: after TS2 on lane 0 only, state = {sname(got)} "
        f"(spec requires staying in RECOVERY_RCVR_CFG)")
    assert got == ST_RECOVERY_RCVR_CFG, (
        f"R9 violated: 8 TS2 on 1 of 4 configured Lanes moved the FSM to "
        f"{sname(got)}; Base 2.1 p.244 requires ALL configured Lanes")


# ==========================================================================
#  R13 -- Recovery.Idle -> L0 must require ALL configured Lanes.
# ==========================================================================

@cocotb.test()
async def test_r13_idle_to_l0_requires_all_lanes(dut):
    """R13 (p.246): 8 consecutive Idle Symbol Times on ALL configured Lanes.

    Four lanes are configured; Idle is presented on lane 0 only. A conforming
    DUT stays in Recovery.Idle. Before fix-arc 1 this DUT reached L0, because
    the exit reduced with `|` over a lanes_idle_satisfied that was never
    lane-gated. STATUS: FIXED (fix-arc 1, Phase 3A) -- :1471 now reduces with
    `&` and :1623 gates the operand by lane_active_r. Both edits were required:
    the reduction alone would hang a reduced-width link, which is what row 63
    verilate_recovery_partial_lanes exists to catch.
    """
    await drive_to_recovery_idle(dut)

    dut.idle_valid_i.value = LANE0         # lane 0 only, 3 lanes silent
    await ClockCycles(dut.clk_i, SETTLE_CYCLES)
    await Timer(1, units="ps")

    got = state(dut)
    dut._log.info(
        f"R13: after Idle on lane 0 only, state = {sname(got)} "
        f"(spec requires staying in RECOVERY_IDLE)")
    assert got == ST_RECOVERY_IDLE, (
        f"R13 violated: Idle on 1 of 4 configured Lanes moved the FSM to "
        f"{sname(got)}; Base 2.1 p.246 requires ALL configured Lanes")


# ==========================================================================
#  R14a -- the "2 consecutive" limb.
# ==========================================================================

@cocotb.test(expect_fail=True)
async def test_r14a_config_exit_requires_two_consecutive(dut):
    """R14 (p.245), count limb: TWO consecutive TS1 are required.

    Drives exactly ONE TS1 with Lane = PAD -- spec-correct type, spec-correct
    Lane number -- so the only thing that can move this DUT is the threshold.
    A conforming DUT stays put after one. This DUT leaves for Configuration,
    because at_least_one_ts1_ts2 fires at a count of 1 (:1578, :1805).
    """
    await drive_to_recovery_idle(dut)

    dut.ordered_set_i.value = pack_tsos_all_lanes(
        link_num=LINK_NUM, lane_num=None, rate=GEN1_RATE)   # lane_num=None -> PAD
    dut.ts1_valid_i.value = ALL
    await ClockCycles(dut.clk_i, 1)        # exactly one TS1
    dut.ts1_valid_i.value = 0
    await ClockCycles(dut.clk_i, 40)
    await Timer(1, units="ps")

    got = state(dut)
    dut._log.info(
        f"R14a: after ONE TS1/PAD, state = {sname(got)} "
        f"(spec requires two consecutive before Configuration)")
    assert got == ST_RECOVERY_IDLE, (
        f"R14a violated: a single TS1 with Lane=PAD moved the FSM to "
        f"{sname(got)}; Base 2.1 p.245 requires TWO consecutive")


# ==========================================================================
#  R14b -- the TS-type limb.
# ==========================================================================

@cocotb.test(expect_fail=True)
async def test_r14b_config_exit_requires_ts1_not_ts2(dut):
    """R14 (p.245), type limb: the trigger is TS1, not TS2.

    Drives TWO consecutive TS2 with Lane = PAD -- spec-correct count,
    spec-correct Lane -- so the only thing that can move this DUT is its
    acceptance of the wrong TS type at :1803.
    """
    await drive_to_recovery_idle(dut)

    dut.ordered_set_i.value = pack_tsos_all_lanes(
        link_num=LINK_NUM, lane_num=None, rate=GEN1_RATE)   # PAD Lane
    dut.ts2_valid_i.value = ALL
    await ClockCycles(dut.clk_i, 2)        # two consecutive TS2
    dut.ts2_valid_i.value = 0
    await ClockCycles(dut.clk_i, 40)
    await Timer(1, units="ps")

    got = state(dut)
    dut._log.info(
        f"R14b: after two TS2/PAD, state = {sname(got)} "
        f"(spec names TS1 as the only trigger)")
    assert got == ST_RECOVERY_IDLE, (
        f"R14b violated: TS2 Ordered Sets moved the FSM to {sname(got)}; "
        f"Base 2.1 p.245 names TS1 as the Configuration trigger")


# ==========================================================================
#  R15 -- the second 2 ms timeout must reach Detect.
# ==========================================================================

@cocotb.test()
async def test_r15_second_timeout_reaches_detect(dut):
    """R15 (p.246): idle_to_rlock_transitioned is BINARY -- one retry only.

    Timeout 1: Recovery.Idle -> Recovery (RcvrLock). Spec sets the variable to
    1b here. Timeout 2 must therefore reach Detect.

    Before fix-arc 1 this DUT incremented instead of saturating at Gen1, against
    a guard of != 8'hFF (:1457), so the second timeout diverted to Recovery again
    -- and would have kept doing so ~255 times, roughly 510 ms. The Gen1 arm now
    saturates like the Gen2 arm beside it, so the second timeout reaches ST_IDLE
    via :1476 and this row asserts the spec outcome directly.

    Cost: two unscaled 2 ms timeouts = ~400k cycles.
    """
    await drive_to_recovery_idle(dut)

    # ---- timeout 1: present nothing and let the 2 ms timer expire ----
    await wait_state(dut, ST_RECOVERY, TWO_MS_CYCLES + TIMEOUT_SLACK,
                     "ST_RECOVERY (first 2 ms timeout)")
    dut._log.info("R15: first 2 ms timeout diverted to Recovery, as the spec allows")

    # ---- drive back around to Recovery.Idle for the second timeout.
    # Nothing on this loop clears idle_to_rlock_transitioned: its clearing
    # sites are :534 (ST_IDLE), :980 and :1017 (L0 entry/L0), :1475
    # (Recovery.Idle -> L0) and :1522 (SendSDS -> L0), none of which we pass.
    dut.ordered_set_i.value = pack_tsos_all_lanes(
        link_num=LINK_NUM, lane_num="index", rate=GEN1_RATE, speed_change=0)
    dut.ts1_valid_i.value = ALL
    await wait_state(dut, ST_RECOVERY_RCVR_LOCK, 500, "RECOVERY_RCVR_LOCK (2nd)")
    await wait_state(dut, ST_RECOVERY_RCVR_CFG, 500, "RECOVERY_RCVR_CFG (2nd)")
    dut.ts1_valid_i.value = 0
    dut.ts2_valid_i.value = ALL
    await wait_state(dut, ST_RECOVERY_IDLE, 2000, "RECOVERY_IDLE (2nd)")
    dut.ts2_valid_i.value = 0

    # ---- timeout 2: watch which state we land in ----
    start = state(dut)
    assert start == ST_RECOVERY_IDLE
    for _ in range(TWO_MS_CYCLES + TIMEOUT_SLACK):
        await RisingEdge(dut.clk_i)
        await Timer(1, units="ps")
        got = state(dut)
        if got != ST_RECOVERY_IDLE:
            dut._log.info(
                f"R15: second 2 ms timeout left Recovery.Idle for {sname(got)} "
                f"(spec requires Detect / ST_IDLE)")
            assert got == ST_IDLE, (
                f"R15 violated: the second 2 ms timeout went to {sname(got)}; "
                f"Base 2.1 p.246 makes idle_to_rlock_transitioned binary, so "
                f"the second timeout must reach Detect")
            return
    raise AssertionError("R15 setup: second 2 ms timeout never fired")
