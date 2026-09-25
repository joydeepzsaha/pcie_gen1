"""
Fix-arc 1, Phase 2 -- the LTSSM's error/success outputs, observed at the port.

WHAT IS BROKEN
  pcie_ltssm_downstream declares `error_o` (:42) and `success_o` (:43) and
  drives NEITHER. Internally the FSM maintains error_c/error_r (:185-:186,
  registered at :416) and success_c/success_r (:187-:188, :417); the raise
  sites are real -- 12 of them for error_c -- but nothing connects them to a
  port. Measured at 7419631:

      $ grep -n 'error_o\\|success_o' src/ltssm/pcie_ltssm_downstream.sv
      42:    output logic  error_o,
      43:    output logic  success_o,
      695:              // us to ST_IDLE either way; this just makes sure error_o

  -- two declarations and one comment, no driver. So an integrator cannot
  observe a training failure or a training success, and four pre-existing gate
  assertions that read `error_o` (test_ltssm_partial_lanes.py:207,:304 and
  test_ltssm_recovery_partial_lanes.py:92,:136) are tautologies: they compare a
  permanently-zero port against zero. That is the `[unsound]` ledger tag on gate
  rows 58 and 63 (Tracker section 55), and repairing it is the point of this
  bench.

  Defect register: FINDINGS_LTSSM.md section 2 Block B, entries B1 and B2;
  Tracker section 54 row 2.

WHAT THIS BENCH ASSERTS -- one divergent assertion per expect_fail row
  obs_error   `error_o` rises after a training failure the FSM already detects.
  obs_success `success_o` is high while the link is up in L0.

  Both were expect_fail on unfixed RTL, where the ports read 0 no matter what
  the FSM did. STATUS: FIXED (fix-arc 1, Phase 2) -- `assign error_o = error_r;`
  and `assign success_o = success_r;` landed at :320-:321 and both markers were
  removed in that same commit (rule 22.75). Flipping them did not move either
  gate row: cocotb reports an expect_fail raise as STATUS=PASS, and each test
  samples at a point fixed by its drive sequence, not by the sampled value.

THE PROVOCATION, and why this one -- ** RE-ANCHORED IN FIX-ARC 6b **
  Until fix-arc 6b this bench provoked error_c at :622 (was :614), the Detect.Rx
  Lane-set mismatch. That was the cheapest reachable site, because reaching it
  costs one TwelveMsTimeOut and that timer IS SIM_FAST_LINK scaled (:111, 1200
  cycles) where almost every other raise site sits behind an unscaled 2/24/48 ms
  timeout (200 000 / 2.4 M / 4.8 M cycles).

  ** But :622 is tracker sec 54 #8's oracle D10 -- an OPEN DEFECT scheduled for
  removal. So this bench's oracle for sec 54 #2 was anchored to another register
  row's bug: fixing D10 would have turned the control and B1 red, and two
  register rows were in direct conflict. Documented in
  evidence/fix-arc-6/FINDINGS_D10_COUPLING.md.

  The replacement site is chosen to be SPEC-CONFORMANT BY RECORDED VERDICT
  rather than merely reachable, so that no future fix can un-anchor it again:

      :931  ST_CONFIGURATION_LANENUM_WAIT, the 2 ms timeout
      :932    if ((timer_r >= TwoMsTimeOut) && (next_state == curr_state)) begin
      :934      error_c    = '1;
      :936      next_state = ST_IDLE;

  Oracle C13 (evidence/rung10/ORACLES_LTSSM.md:99), Base 2.1 4.2.6.3.4 p.234
  "Next state is Detect after 2 ms", verdict *conforms*. Lanenum.Wait appears on
  neither sec 54 #8 nor #11. All 12 error_c sites were classified before the
  choice: evidence/fix-arc-6/SITE_SELECTION_OBS.md.

  ** COST, recorded rather than hidden. Every error_c site behind a scaled timer
  lives inside ST_DETECT_RX -- the state being vacated -- so re-anchoring
  necessarily buys an unscaled timeout. This bench goes from ~3 k cycles to
  ~200 k. 2 ms is the cheapest conformant option; C5 (24 ms) and P12 (48 ms) are
  the alternatives. That is the price of decoupling the register.

  x4 is still required, and now for a different reason. The old provocation
  needed it because ST_DETECT_RX is unreachable at x1. This one needs it because
  the four gate assertions this bench exists to make sound
  (test_ltssm_partial_lanes.py:207,:304 and
  test_ltssm_recovery_partial_lanes.py:92,:136) are themselves x4 rows; keeping
  the geometry identical keeps the repair and its beneficiaries on one config.

WHY error_o AND success_o NEED DIFFERENT ASSERTION SHAPES
  They are not symmetric, despite the register listing them as one line each:

      :482   error_c   = error_r;     <- defaults to its own registered value
      :483   success_c = '0;          <- defaults to 0

  No site anywhere assigns error_c to 0 (`grep -E "error_c\\s*=\\s*('0|1'b0|0)"`
  finds nothing), so error_r is STICKY: it latches on the first error and clears
  only on rst_i. success_r is a LEVEL that tracks the state -- high throughout
  ST_L0 (:999), plus one cycle at each Detect success (:583, :610).

  Hence: error_o may be sampled any time after the provocation; success_o must
  be sampled while the FSM is in L0.

NEGATIVE CONTROL
  An expect_fail row goes green if ANYTHING in it raises, including a broken
  setup (10b's D9/D10 lesson, rule 22.66). test_obs_control_provocation_reaches_error_site
  is an ordinary PASS row that runs the same drive sequence and asserts the
  FSM's OBSERVABLE state behaviour -- that it leaves ST_DETECT_RX for ST_IDLE
  shortly after the 12 ms timeout. That transition has only two sources: :615,
  the error arm, and :619 at TwentyFourMsTimeOut, which is 2.4 M cycles away and
  cannot be what fired inside a 1500-cycle window. So the control witnesses that
  :614 executed, using only ports that are driven today -- which is the only way
  to witness it on unfixed RTL, error_r itself not being a port.
  If this control is red, the two rows below prove nothing and are void.

Requires SIM_FAST_LINK=1, MAX_NUM_LANES=4 (verilate_ltssm_obs target).
"""
import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, Timer
from ltssm_tb_common import *  # noqa

# sec 63 #7g-2 (D-7G.2): this bench's clock AND the RTL's CLK_PERIOD_NS, one value.
# The core passes -GCLK_PERIOD_NS=8 (tb_ltssm_b2b.sv passes .CLK_PERIOD_NS(8));
# every cycle budget below that stands for a spec time derives from it.
CLK_PERIOD_NS = 8

# Same definitions as test_ltssm_partial_lanes.py:81-95, repeated rather than
# imported: cross-importing one cocotb test module from another makes the
# importer's tests run twice under some runners.
def _mask(active_lanes):
    m = 0
    for lane in active_lanes:
        m |= (1 << lane)
    return m


def _rxstatus_mask(active_lanes):
    """phy_rxstatus_i is MAX_NUM_LANES*3 bits; 3'b011 per active lane, all-zero
    for inactive lanes (lane_status checks phy_rxstatus_i[3*i+:3] == 3'b011,
    pcie_ltssm_downstream.sv:448)."""
    v = 0
    for lane in active_lanes:
        v |= (0b011 << (3 * lane))
    return v


# pcie_ltssm_downstream.sv:113 -- TwoMsTimeOut = (2*10**6)/ClockPeriodNs, and it
# is NOT SIM_FAST_LINK scaled (contrast :111/:114, which are). 200 000 cycles at
# ClockPeriodNs=10. This is the dominant cost of this bench and the price of
# anchoring the oracle to a conformant site; see drive_to_lanenum_wait_timeout.
TWO_MS_CYCLES = 2_000_000 // CLK_PERIOD_NS   # 63 #7g-2: was 200_000 (10 ns)
SETTLE = 150


def state(dut):
    return int(dut.ltssm_state_o.value)


def sname(s):
    return STATE_NAMES.get(s, hex(s))


def check_geometry(dut):
    n = len(dut.ordered_set_i)
    assert n == 512, (
        f"-GMAX_NUM_LANES=4 did not reach the DUT: ordered_set_i is {n} bits "
        f"(x4 expects 4*128=512); ST_DETECT_RX is unreachable at x1, so this "
        f"bench would be vacuous")


async def drive_to_lanenum_wait_timeout(dut):
    """Reset -> ... -> Configuration.Lanenum.Wait, then HOLD until its 2 ms
    timeout raises error_c at :934 and the FSM leaves for ST_IDLE.

    Returns (cycles_waited, landed_state) measured from arrival in Lanenum.Wait.

    ** WHY THIS SITE, and not the Detect.Rx one this bench used until fix-arc 6b.

    The original provocation drove a Detect.Rx Lane-set mismatch so :622 (was
    :614) raised error_c.  That site is tracker sec 54 #8's oracle D10 -- an OPEN
    DEFECT scheduled for removal -- so this bench's oracle for sec 54 #2 was
    anchored to another register row's bug, and fixing D10 would have turned two
    of these three rows red.  Two register rows in direct conflict.  Documented in
    evidence/fix-arc-6/FINDINGS_D10_COUPLING.md.

    The replacement is chosen to be SPEC-CONFORMANT BY RECORDED VERDICT, not
    merely reachable: oracle C13 (evidence/rung10/ORACLES_LTSSM.md:99), Base 2.1
    4.2.6.3.4 p.234 "Next state is Detect after 2 ms", verdict *conforms*.  It is
    on no open-defect list, so no future fix can un-anchor it.  Site selection
    across all 12 error_c sites: evidence/fix-arc-6/SITE_SELECTION_OBS.md.

    ** COST, recorded rather than hidden.  Every error_c site behind a
    SIM_FAST_LINK-scaled timer lives inside ST_DETECT_RX -- the state being
    vacated -- so re-anchoring necessarily buys an unscaled timeout.  TwoMsTimeOut
    (:113) is 200 000 cycles where TwelveMsTimeOut was 1 200.  That is the price
    of decoupling the register, and 2 ms is the cheapest conformant option; the
    alternatives are 24 ms (C5) and 48 ms (P12).

    ** WHY ARRIVAL IN ST_IDLE IDENTIFIES THE SITE.  Lanenum.Wait has exactly two
    exits: :928 to CFG_LANENUM_ACCEPT (two consecutive TS1 carrying a CHANGED Lane
    number) and :932's 2 ms timeout to ST_IDLE.  We hold the Lane number at its
    entry value, so the first can never fire and reaching ST_IDLE from here is
    unambiguous.  That is a STRONGER control than the old one, which had to argue
    that a 24 ms alternative was too far away to be the cause.
    """
    cocotb.start_soon(Clock(dut.clk_i, CLK_PERIOD_NS, units="ns").start())
    check_geometry(dut)
    drive_idle_inputs(dut)
    dut.rst_i.value = 1
    await ClockCycles(dut.clk_i, 5)
    dut.rst_i.value = 0
    await ClockCycles(dut.clk_i, 5)
    assert state(dut) == ST_IDLE, f"post-reset state is {sname(state(dut))}"

    # ---- IDLE -> DETECT_QUIET ----
    dut.en_i.value = 1
    await wait_state(dut, ST_DETECT_QUIET, 50, "DETECT_QUIET")

    # ---- DETECT_QUIET -> DETECT_ACTIVE on the elec-idle exit edge (1 -> 0) ----
    dut.phy_rxelecidle_i.value = ALL
    await ClockCycles(dut.clk_i, 3)
    dut.phy_rxelecidle_i.value = 0
    await wait_state(dut, ST_DETECT_ACTIVE, 50, "DETECT_ACTIVE")

    # ---- ALL four Lanes detect -> :582's &receiver_detected_i -> POLLING.
    # Deliberately NOT the partial mask the old provocation used: that took the
    # Detect.Rx hop, which is the very state being vacated. ----
    dut.receiver_detected_i.value = ALL
    dut.phy_rxstatus_i.value = _rxstatus_mask([0, 1, 2, 3])
    dut.phy_phystatus_i.value = ALL
    await ClockCycles(dut.clk_i, 3)
    dut.phy_phystatus_i.value = 0
    cocotb.start_soon(os_tx_pulser(dut))
    await wait_state(dut, ST_POLLING_ACTIVE, 400, "POLLING_ACTIVE")

    # ---- Polling.Active -> Polling.Configuration on eight TS1 PAD/PAD ----
    dut.ordered_set_i.value = pack_tsos_all_lanes(link_num=None, lane_num=None)
    dut.ts1_valid_i.value = ALL
    await wait_state(dut, ST_POLLING_CONFIG, 6000, "POLLING_CONFIGURATION")

    # ---- Polling.Configuration -> Configuration.Linkwidth.Start on TS2 ----
    dut.ts1_valid_i.value = 0
    dut.ts2_valid_i.value = ALL
    await wait_state(dut, ST_CFG_LW_START, 6000, "CFG_LINKWIDTH_START")

    # ---- Linkwidth.Start -> Linkwidth.Accept on TS1 with a non-PAD Link ----
    dut.ts2_valid_i.value = 0
    dut.ordered_set_i.value = pack_tsos_all_lanes(link_num=LINK_NUM, lane_num=None)
    dut.ts1_valid_i.value = ALL
    await wait_state(dut, ST_CFG_LW_ACCEPT, 12000, "CFG_LINKWIDTH_ACCEPT")

    # ---- ... -> Lanenum.Wait, where the Lane number seen on entry is latched
    # into lane_in_save (:1868). We keep presenting THAT SAME value, so C11's
    # "Lane number different from entry" can never be satisfied and the only
    # remaining exit is the 2 ms timeout. ----
    await wait_state(dut, ST_CFG_LN_WAIT, 12000, "CFG_LANENUM_WAIT")
    dut._log.info(
        "OBS setup: in Configuration.Lanenum.Wait, holding the entry Lane "
        "number so the only reachable exit is :932's 2 ms timeout")

    # ---- the provocation: simply wait. TwoMsTimeOut (:113) is UNSCALED. ----
    waited = 0
    landed = state(dut)
    for i in range(TWO_MS_CYCLES + SETTLE):
        await ClockCycles(dut.clk_i, 1)
        await Timer(1, units="ps")
        landed = state(dut)
        if landed != ST_CFG_LN_WAIT:
            waited = i + 1
            break
    dut._log.info(
        f"OBS provocation: left CFG_LANENUM_WAIT after {waited} cycles -> "
        f"{sname(landed)} (2 ms = {TWO_MS_CYCLES} cycles); :934 raised error_c")
    return waited, landed


# ==========================================================================
#  Negative control -- ordinary PASS row.
# ==========================================================================

@cocotb.test()
async def test_obs_control_provocation_reaches_error_site(dut):
    """Control: the drive sequence really does execute :934.

    :934 and :936 are the same statement pair, so witnessing the state change at
    :936 witnesses the error_c raise at :934.

    ** This control is STRONGER than the Detect.Rx one it replaces.  Lanenum.Wait
    has exactly TWO exits -- :928 to CFG_LANENUM_ACCEPT, which needs two
    consecutive TS1 carrying a Lane number DIFFERENT from the one latched on
    entry, and :932's 2 ms timeout to ST_IDLE.  The setup holds the entry Lane
    number, so the first is unreachable by construction and arrival in ST_IDLE
    identifies :934 uniquely.  The old control could only argue that its
    alternative (a 24 ms timeout) was too far away to be the cause; this one
    excludes the alternative outright.

    ** Non-vacuity (tracker sec 22.82): the wait is bounded at TWO_MS_CYCLES +
    SETTLE and the window is asserted to have been ENTERED (waited > 0) and to
    have closed near the 2 ms mark, so a run that never reached Lanenum.Wait, or
    that left it instantly by some other arc, fails here rather than passing
    silently.
    """
    waited, landed = await drive_to_lanenum_wait_timeout(dut)
    assert landed == ST_IDLE, (
        f"control failed: the 2 ms timeout should take :936 to ST_IDLE, got "
        f"{sname(landed)} -- the provocation did not reach :934 and the two "
        f"rows below it are void")
    # The exit must happen AT the timeout, not before it: anything materially
    # earlier means some other arc fired and this is not C13's site.
    assert TWO_MS_CYCLES * 0.9 < waited <= TWO_MS_CYCLES + SETTLE, (
        f"control failed: left CFG_LANENUM_WAIT after {waited} cycles, outside "
        f"the window around TwoMsTimeOut = {TWO_MS_CYCLES}; the exit taken was "
        f"not :932's timeout")
    dut._log.info(
        f"CONTROL OK: :934/:936 executed -- CFG_LANENUM_WAIT -> ST_IDLE after "
        f"{waited} cycles (TwoMsTimeOut = {TWO_MS_CYCLES}), the only other exit "
        f"held unreachable by holding the entry Lane number")


# ==========================================================================
#  B1 -- error_o must report a training failure the FSM already detected.
# ==========================================================================

@cocotb.test()
async def test_obs_error_o_reports_training_failure(dut):
    """B1: error_o must report the training failure the FSM detected at :934.

    The control above proves :934 ran. error_r is sticky (:490 defaults error_c
    to error_r and no site clears it), so by the time this samples, error_r has
    been 1 since the provocation, and :320 puts it on the port. Before fix-arc 1
    this read 0 no matter what the FSM did.

    ** The FAILURE PATH IS REAL, which is the property that had to survive the
    fix-arc-6b re-anchor.  Configuration.Lanenum.Wait timing out after 2 ms is a
    genuine training failure -- Base 2.1 4.2.6.3.4 p.234 sends it to Detect -- so
    error_o is being asked to report something the FSM legitimately detected, not
    a signal forced by the bench.  The previous anchor (a Detect.Rx Lane-set
    mismatch, :622) was equally real, but sat on an OPEN DEFECT scheduled for
    removal; this one sits on an arc whose recorded verdict is *conforms*
    (oracle C13), so no future fix can pull it out from under this row.
    """
    _waited, landed = await drive_to_lanenum_wait_timeout(dut)
    assert landed == ST_IDLE, (
        f"setup did not reach the error site (landed in {sname(landed)}); see "
        f"the control row")
    await ClockCycles(dut.clk_i, 20)

    got = int(dut.error_o.value)
    dut._log.info(
        f"B1: after a Configuration.Lanenum.Wait 2 ms timeout, error_o = {got} "
        f"(the FSM raised error_c at :934; a driven port would read 1)")
    assert got == 1, (
        "B1 violated: error_o reads 0 after a training failure the FSM itself "
        "detected at pcie_ltssm_downstream.sv:934. The port is declared at :42 "
        "and never assigned -- error_c/error_r reach no port at all, so no "
        "integrator can observe a training failure, and the four assertions on "
        "gate rows 58/63 that read error_o are tautologies. Fix: "
        "assign error_o = error_r;")


# ==========================================================================
#  B2 -- success_o must report a trained link.
# ==========================================================================

@cocotb.test()
async def test_obs_success_o_reports_link_trained(dut):
    """B2: success_o must report a trained link.

    success_c is unconditional in ST_L0 (:999) and success_r registers it
    (:425), so the port at :321 reads 1 for as long as the FSM stays in L0.
    This samples while link_up_o is high, which pins the two together.
    Before fix-arc 1 it read 0 with the link up.
    """
    cocotb.start_soon(Clock(dut.clk_i, CLK_PERIOD_NS, units="ns").start())
    check_geometry(dut)
    await bring_up_link(dut)
    assert state(dut) == ST_L0, f"setup did not reach L0, got {sname(state(dut))}"
    await ClockCycles(dut.clk_i, 20)
    assert state(dut) == ST_L0, "fell out of L0 before the sample"
    assert int(dut.link_up_o.value) == 1, "link_up_o low in L0 -- setup is broken"

    got = int(dut.success_o.value)
    dut._log.info(
        f"B2: in L0 with link_up_o=1, success_o = {got} "
        f"(the FSM holds success_c at :999; a driven port would read 1)")
    assert got == 1, (
        "B2 violated: success_o reads 0 in ST_L0 while link_up_o reads 1. The "
        "port is declared at :43 and never assigned, so a trained link is "
        "unobservable on it. Fix: assign success_o = success_r;")
