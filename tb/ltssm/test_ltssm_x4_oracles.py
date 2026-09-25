"""
The oracles that only exist at x4: D9/D10, the |-vs-& reduction, and per-lane
Lane Number assignment.

Every Rung-10b bench before this one runs at MAX_NUM_LANES=1, where three
distinct questions collapse into nothing:

  * ST_DETECT_RX is entered only when SOME BUT NOT ALL lanes detect a Receiver
    (:582 uses &receiver_detected_i where :581 uses |). At one lane those
    reductions are identical, so the state is structurally unreachable and
    oracles D9/D10 cannot be exercised at all.
  * |x, &x and x[0] coincide at one bit, so a mutation swapping an OR reduction
    for an AND reduction is an EQUIVALENT MUTANT. Rung 1's LTSSM_ARCS.md
    section 6.3 recorded this and required that any test for it "must be x4 AND
    must be added to the gate, or it measures nothing". This file is the first
    to satisfy both.
  * The per-lane output stage is a documented no-op at x1 -- the RTL says so
    itself at :2004-2006 ("l is always 0 ... bit-identical to the previous
    assign ordered_set_o = ordered_set_r").

ORACLES EXERCISED

  D9  4.2.6.1.2 p.219 -- "The next state is Polling if exactly the same Lanes
      detect a Receiver as the first Receiver Detection sequence."
      PREDICTED CONFORMS.

  D10 4.2.6.1.2 p.219 -- "Otherwise, the next state is Detect.Quiet."
      :614-615 goes to ST_IDLE and additionally asserts error_c, where the spec
      treats this as an ordinary retry. PREDICTED DIVERGES.
      NOTE (CORRECTED fix-arc 6b): this used to read "the error_c half is
      untestable at ANY width -- error_o is never driven". That was true when
      written and is STALE: fix-arc 1 added `assign error_o = error_r;` (:320).
      The error_c half IS observable now. It is still not asserted HERE -- this
      row's single divergent assertion is the ST_IDLE detour -- but the reason
      is one-assertion-per-row, not unobservability.
      Superseded text follows: the error_c half is untestable at ANY width -- error_o is never
      driven (evidence/rung10/CENSUS_LTSSM.md section 3) -- so only the state
      divergence is asserted here. Recorded, not silently dropped.

  R63 4.2.6.2.1 p.220 -- the primary Polling.Active exit needs eight consecutive
      training sequences on ALL Lanes that detected a Receiver. :660 uses
      &lanes_ts1_satisfied. This test drives a strict subset of lanes and
      requires the FSM to stay put, which is the measurement that distinguishes
      a genuine AND from an OR. Discharges Rung 1 section 6.3.
      PREDICTED CONFORMS.

  C7/A4 4.2.6.3.2.1 p.230 -- "unique non-PAD Lane numbers are assigned to all
      these same Lanes ... must range from 0 to n-1, be assigned sequentially".
      Sampled directly off ordered_set_o, per lane. PREDICTED CONFORMS.

Requires SIM_FAST_LINK=1, MAX_NUM_LANES=4, IS_ROOT_PORT=1, LINK_NUM=1
(verilate_ltssm_x4_oracles target).
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
X4 = 0xF                    # all four lanes
SUBSET = 0b0011             # lanes 0,1 -- a strict, non-empty subset of X4
CHANGED = 0b0111            # lanes 0,1,2 -- differs from SUBSET

# Short relative to every watchdog in play. Polling.Active's 24 ms branch
# (:672, which correctly uses | per p.221) needs 2.4 M cycles and is NOT
# SIM_FAST_LINK scaled, so a 5000-cycle "did not advance" cannot be that
# timeout in disguise. Detect.Rx's 12 ms wait IS scaled (1200 cycles fast).
NEG_WATCH = 5000
TWELVE_MS_FAST = 12_000 // CLK_PERIOD_NS   # 63 #7g-2: (12*10**4)/(ClockPeriodNs*10); was 1200 at 10 ns


def state(dut):
    return int(dut.ltssm_state_o.value)


def sname(s):
    return f"{STATE_NAMES.get(s, hex(s))} ({s:#07x})"


def lane_of(dut, lane):
    """Unpack ordered_set_o for one lane. ordered_set_o is
    pcie_ordered_set_t [MAX_NUM_LANES-1:0], so lane l occupies bits
    [l*128 +: 128] of the flattened value."""
    whole = int(dut.ordered_set_o.value)
    return unpack_tsos((whole >> (lane * TSOS_WIDTH)) & ((1 << TSOS_WIDTH) - 1))


def clk(dut):
    cocotb.start_soon(Clock(dut.clk_i, CLK_PERIOD_NS, units="ns").start())
    n_bits = len(dut.ordered_set_i)
    assert n_bits == 4 * TSOS_WIDTH, (
        f"-GMAX_NUM_LANES=4 did not reach the DUT: ordered_set_i is {n_bits} "
        f"bits (x4 expects {4 * TSOS_WIDTH}; x1 would be {TSOS_WIDTH}). "
        f"Every oracle in this file is vacuous at x1, so this check is a "
        f"precondition, not decoration.")


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


async def present_detection(dut, lanes):
    """Complete one receiver-detection sequence reporting `lanes`."""
    dut.receiver_detected_i.value = lanes
    dut.phy_rxstatus_i.value = RXSTATUS_ALL_OK
    dut.phy_phystatus_i.value = X4
    await ClockCycles(dut.clk_i, 3)
    dut.phy_phystatus_i.value = 0


async def reach_polling_active(dut, detected=X4):
    await reset_to_detect_active(dut)
    await present_detection(dut, detected)
    cocotb.start_soon(os_tx_pulser(dut))
    await wait_state(dut, ST_POLLING_ACTIVE, 300, "POLLING_ACTIVE")


# =====================================================================
# D9 / D10 -- Detect.Rx's two exits
#
# These are deliberately SEPARATE tests, and that is a correctness requirement,
# not tidiness. D10 must be an expect_fail row, and expect_fail asks only "did
# this test fail?" -- it cannot tell an intended divergence from an unrelated
# breakage. Carrying D9's conforming assertion inside the same expect_fail test
# would let a D9 failure masquerade as the D10 result and report green.
# (Caught in exactly that form during Rung 10b: the combined test reported
# "failed as expected" while never reaching either of its own log lines.)
# =====================================================================
@cocotb.test()
async def run_test_d9_detect_rx_same_lanes(dut):
    """D9, the conforming half -- ordinary PASS row."""
    clk(dut)

    await reset_to_detect_active(dut)
    await present_detection(dut, SUBSET)      # some but not all -> Detect.Rx
    await wait_state(dut, ST_DETECT_RX, 200, "DETECT_RX")
    dut._log.info(f"entered Detect.Rx on a partial detection "
                  f"({SUBSET:#06b}) -- unreachable at x1")

    dut.phy_phystatus_i.value = 0
    await ClockCycles(dut.clk_i, TWELVE_MS_FAST + 200)
    await present_detection(dut, SUBSET)      # identical set

    # Wait for ST_POLLING_ACTIVE, not ST_POLLING: the latter is a one-cycle
    # pass-through (:625-637, unconditional next_state) and is easy to sample
    # past. Reaching Polling.Active proves the Detect.Rx -> Polling arc fired.
    await wait_state(dut, ST_POLLING_ACTIVE, 600, "POLLING_ACTIVE (D9)")
    dut._log.info("D9 CONFORMS: identical Lane set on the second detection "
                  "advanced to Polling")


@cocotb.test()
async def run_test_d10_detect_rx_changed_lanes(dut):
    """D10 -- FIXED in fix-arc 6b; this is now the guard row.

    Carried expect_fail from Rung 10b until Detect.Rx's changed-Lane-set exit was
    retargeted to ST_DETECT_QUIET and its error_c removed (both halves, since
    p.219 calls this an ordinary retry rather than a failure).  The fix was
    blocked for one commit by a coupling: verilate_ltssm_obs provoked its error_o
    oracle through this very site.  obs was re-anchored first.
    evidence/fix-arc-6/FINDINGS_D10_COUPLING.md.
    """
    clk(dut)

    await reset_to_detect_active(dut)
    await present_detection(dut, SUBSET)
    await wait_state(dut, ST_DETECT_RX, 200, "DETECT_RX")

    dut.phy_phystatus_i.value = 0
    await ClockCycles(dut.clk_i, TWELVE_MS_FAST + 200)

    # Present the second (different) detection WITHOUT awaiting it, then sample
    # from the very next edge. present_detection() awaits three cycles before
    # returning, by which time the exit has already been taken -- sampling
    # after it would start the trace in ST_IDLE and make the assertion below
    # vacuous. (That is precisely what happened on the first run of this test.)
    dut.receiver_detected_i.value = CHANGED
    dut.phy_rxstatus_i.value = RXSTATUS_ALL_OK
    dut.phy_phystatus_i.value = X4

    seq = []
    for i in range(400):
        await RisingEdge(dut.clk_i)
        await Timer(1, units="ps")
        if i == 3:
            dut.phy_phystatus_i.value = 0
        s = state(dut)
        if not seq or s != seq[-1]:
            seq.append(s)
        if len(seq) >= 3:
            break
    dut._log.info("D10 state sequence: " + " -> ".join(sname(s) for s in seq))

    # Guard the guard: the trace must BEGIN in Detect.Rx, or the capture window
    # opened too late and whatever follows proves nothing.
    assert seq[0] == ST_DETECT_RX, (
        f"capture started in {sname(seq[0])}, not Detect.Rx -- the sampling "
        f"window opened after the exit was taken, so this test would be "
        f"vacuous. Trace: {' -> '.join(sname(s) for s in seq)}")

    assert ST_IDLE not in seq, (
        f"D10 (Base 2.1 4.2.6.1.2, p.219): when the second Receiver Detection "
        f"finds a different Lane set, 'the next state is Detect.Quiet'. The "
        f"DUT went {' -> '.join(sname(s) for s in seq)}, detouring through "
        f"ST_IDLE (pcie_ltssm_downstream.sv:623, was :615), whose only exit is "
        f"`if (en_i)` at :525. The same site also asserts error_c where the "
        f"spec calls this an ordinary retry; that half is OBSERVABLE since "
        f"fix-arc 1 drove error_o (:320) and is simply not asserted here, one "
        f"divergent assertion per row.")


# =====================================================================
# R63 -- Polling.Active's primary exit is a genuine AND over lanes
# =====================================================================
@cocotb.test()
async def run_test_r63_polling_all_lanes(dut):
    clk(dut)

    # ---- negative control: eight consecutive TS1 on a STRICT SUBSET ----
    # All four lanes detected a Receiver, so lanes_ts1_satisfied[2] and [3]
    # stay 0 and &lanes_ts1_satisfied is false. Under an OR reduction this
    # would advance; under the spec's "all Lanes" it must not.
    await reach_polling_active(dut, detected=X4)
    dut.ordered_set_i.value = pack_tsos_all_lanes(link_num=None, lane_num=None)
    dut.ts1_valid_i.value = SUBSET
    for _ in range(NEG_WATCH):
        await RisingEdge(dut.clk_i)
        await Timer(1, units="ps")
        if state(dut) != ST_POLLING_ACTIVE:
            raise AssertionError(
                f"NEGATIVE CONTROL FAILED (Rung 1 section 6.3): TS1 driven on "
                f"lanes {SUBSET:#06b} only, yet the FSM advanced to "
                f"{sname(state(dut))}. Base 2.1 p.220 requires eight "
                f"consecutive training sequences on ALL Lanes that detected a "
                f"Receiver; :660's &lanes_ts1_satisfied is behaving as an OR.")
    dut._log.info(
        f"negative control bit: TS1 on lanes {SUBSET:#06b} of {X4:#06b} did "
        f"not advance Polling.Active in {NEG_WATCH} cycles -- the reduction "
        f"at :660 is a genuine AND")

    # ---- positive: all four lanes ----
    await reach_polling_active(dut, detected=X4)
    dut.ordered_set_i.value = pack_tsos_all_lanes(link_num=None, lane_num=None)
    dut.ts1_valid_i.value = X4
    await wait_state(dut, ST_POLLING_CONFIG, 8000,
                     "POLLING_CONFIGURATION (R63 positive)")
    dut._log.info("R63 CONFORMS: Polling.Active's primary exit requires all "
                  "Lanes that detected a Receiver. Rung 1 section 6.3 "
                  "discharged -- x4 and in the gate.")


# =====================================================================
# C7 / A4 -- per-lane Lane Number assignment on the wire
# =====================================================================
@cocotb.test()
async def run_test_c7_a4_per_lane_lanenum(dut):
    clk(dut)

    await reach_polling_active(dut, detected=X4)
    dut.ordered_set_i.value = pack_tsos_all_lanes(link_num=None, lane_num=None)
    dut.ts1_valid_i.value = X4
    await wait_state(dut, ST_POLLING_CONFIG, 8000, "POLLING_CONFIGURATION")

    dut.ts1_valid_i.value = 0
    dut.ts2_valid_i.value = X4
    await wait_state(dut, ST_CFG_LW_START, 8000, "CFG_LINKWIDTH_START")

    # ---- negative control: before any Lane number is assigned, every lane
    #      must carry PAD. A stage that unconditionally wrote t.lane_num = l
    #      would fail here and pass the positive half below. ----
    await RisingEdge(dut.clk_i)
    await Timer(1, units="ps")
    early = [lane_of(dut, l)["lane_num"] for l in range(4)]
    dut._log.info(f"Linkwidth.Start per-lane lane_num on the wire: "
                  f"{[hex(v) for v in early]}")
    assert all(v == PAD for v in early), (
        f"NEGATIVE CONTROL FAILED: in Configuration.Linkwidth.Start the "
        f"Transmitter must send 'Lane numbers to PAD (K23.7)' (Base 2.1 "
        f"p.226), but the wire carries {[hex(v) for v in early]}. The "
        f"per-lane stage at :2027 is supposed to assign only once the "
        f"template holds a non-PAD Lane number.")

    # ---- walk to Lanenum.Wait, where the RC does assign ----
    dut.ts2_valid_i.value = 0
    dut.ordered_set_i.value = pack_tsos_all_lanes(link_num=LINK, lane_num=None)
    dut.ts1_valid_i.value = X4
    await wait_state(dut, ST_CFG_LW_ACCEPT, 8000, "CFG_LINKWIDTH_ACCEPT")
    await wait_state(dut, ST_CFG_LN_WAIT, 8000, "CFG_LANENUM_WAIT")

    await RisingEdge(dut.clk_i)
    await Timer(1, units="ps")
    got = [lane_of(dut, l)["lane_num"] for l in range(4)]
    links = [lane_of(dut, l)["link_num"] for l in range(4)]
    dut._log.info(f"Lanenum.Wait per-lane lane_num on the wire: {got}; "
                  f"link_num: {[hex(v) for v in links]}")

    assert got == [0, 1, 2, 3], (
        f"C7/A4 (Base 2.1 4.2.6.3.2.1, p.230): assigned Lane numbers 'must "
        f"range from 0 to n-1, be assigned sequentially'. The wire carries "
        f"{got}. The per-lane fan-out at :2015-2045 is the only place the "
        f"single 128-bit template diverges per lane, so a failure localises "
        f"there.")

    assert all(v == LINK for v in links), (
        f"the Link number must be broadcast identically on every lane "
        f"(p.230: Lane numbers differ, Link number does not); got "
        f"{[hex(v) for v in links]}")

    dut._log.info("C7/A4 CONFORMS: unique sequential Lane numbers 0..3 per "
                  "lane, Link number broadcast, and PAD before assignment")
