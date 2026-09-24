"""
Four cheap Configuration/Polling gaps: C18, C9, C16, P9.

C18  4.2.6.3.5.1 p.235 -- Configuration.Complete -> Configuration.Idle
     "immediately after all Lanes that are transmitting TS2 Ordered Sets
      receive eight consecutive TS2 Ordered Sets with matching Lane and Link
      numbers (non-PAD) AND IDENTICAL DATA RATE IDENTIFIERS (including
      identical Link Upconfigure Capability (Symbol 4 bit 6)), and 16 TS2
      Ordered Sets are sent after receiving one TS2 Ordered Set."
     :1945 compares link_num and lane_num only. The data rate identifier is
     never compared, so eight TS2 carrying DIFFERENT rate_id bytes still
     satisfy the exit.

C9   4.2.6.3.2.1 p.230 -- Configuration.Linkwidth.Accept: "The next state is
     Detect after a 2 ms timeout OR IF NO LINK CAN BE CONFIGURED OR IF ALL
     LANES RECEIVE TWO CONSECUTIVE TS1 ORDERED SETS WITH LINK AND LANE NUMBERS
     SET TO PAD (K23.7)." :851 implements only the timeout limb.

C16  4.2.6.3.3.1 p.233 -- Configuration.Lanenum.Accept: "The next state is
     Detect IF NO LINK CAN BE CONFIGURED OR IF ALL LANES RECEIVE TWO
     CONSECUTIVE TS1 ORDERED SETS WITH LINK AND LANE NUMBERS SET TO PAD."
     Note what is ABSENT from that sentence: a timeout. The spec gives this
     substate none, yet :880 adds a 2 ms one. So C16 is a divergence in both
     directions -- a missing exit and an added one.

P9   4.2.6.2.3 p.224 -- Polling.Configuration: "Receiver must invert polarity
     if necessary (see Section 4.2.4.4)." The RTL implements polarity
     inversion only in Polling.Active (:654-657); nothing in the
     ST_POLLING_CONFIGURATION arm touches phy_rxpolarity_c.

P9 SHIPS A POSITIVE CONTROL AND THAT IS NOT OPTIONAL. Asserting
"phy_rxpolarity_o did not change in Polling.Configuration" would pass on a DUT
that can never change it at all -- a vacuous green of exactly the class Rung
10a found four of in this directory. The control drives the same stimulus in
Polling.Active and requires the output TO change, which proves the test can
observe an inversion and therefore that the Polling.Configuration result means
"wrong state" rather than "not implemented anywhere".

All four assertions state the SPEC behaviour, so all four were expect_fail rows
until fix-arc 6b closed them.

STATUS: ALL FOUR FIXED (fix-arc 6b), markers removed in the same commit as the
src/ edits (rule 22.75; and 22.77's corollary -- until a marker comes off, the
gate cannot witness the revert mutant either).  Pre-edit census and predictions:
evidence/fix-arc-6/PREDICTIONS_C9_C16_C18_P9.md.  Mutants MC9/MC16/MC18/MP9.

  C18  the rate-identifier conjunct now sits beside the Link/Lane compare in
       ST_CONFIGURATION_COMPLETE's per-lane receive arm, mirroring
       ST_RECOVERY_RCVR_CFG's existing temp_rate_id idiom.
  C9   the all-PAD limb was added to Configuration.Linkwidth.Accept.
  C16  the all-PAD limb was added to Configuration.Lanenum.Accept.  ⚠️ Its
       extra-spec 2 ms watchdog was KEPT, deliberately: an ordinary PASS row --
       verilate_config_timeout::run_test_config_lanenum_accept_timeout --
       requires it to reach ST_IDLE, and Rung 10a already classified the
       structurally identical Detect.Active watchdog (D11) as
       "conformant-but-added" rather than a violation.  So C16 is HALF a fix,
       and that is recorded rather than glossed.
  P9   polarity inversion was ADDED to ST_POLLING_CONFIGURATION -- not moved,
       because the positive control below requires Polling.Active to keep it.

Requires SIM_FAST_LINK=1, MAX_NUM_LANES=1, IS_ROOT_PORT=1, LINK_NUM=1
(verilate_config_gaps target).
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

# Short relative to every watchdog in play: the Configuration substates use
# TwoMsTimeOut = 200 000 cycles (unscaled), so a 5000-cycle "did not advance"
# is two orders of magnitude short of the timeout and cannot be it in disguise.
NEG_WATCH = 5000

# Polarity lockout after an inversion is 1000 cycles (:656), so allow for it.
POL_WATCH = 3000


def state(dut):
    return int(dut.ltssm_state_o.value)


def sname(s):
    return f"{STATE_NAMES.get(s, hex(s))} ({s:#07x})"


def clk(dut):
    cocotb.start_soon(Clock(dut.clk_i, CLK_PERIOD_NS, units="ns").start())
    assert len(dut.ordered_set_i) == TSOS_WIDTH, \
        "-GMAX_NUM_LANES=1 did not reach the DUT"


async def walk(dut, stop_at):
    """Reset -> ... -> stop_at, returning before the exit stimulus is applied."""
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
    await wait_state(dut, ST_POLLING_ACTIVE, 300, "POLLING_ACTIVE")
    if stop_at == ST_POLLING_ACTIVE:
        return

    dut.ordered_set_i.value = pack_tsos(link_num=None, lane_num=None)
    dut.ts1_valid_i.value = LANE0_MASK
    await wait_state(dut, ST_POLLING_CONFIG, 4000, "POLLING_CONFIGURATION")
    if stop_at == ST_POLLING_CONFIG:
        return

    dut.ts1_valid_i.value = 0
    dut.ts2_valid_i.value = LANE0_MASK
    await wait_state(dut, ST_CFG_LW_START, 4000, "CFG_LINKWIDTH_START")

    dut.ts2_valid_i.value = 0
    dut.ordered_set_i.value = pack_tsos(link_num=LINK, lane_num=None)
    dut.ts1_valid_i.value = LANE0_MASK
    await wait_state(dut, ST_CFG_LW_ACCEPT, 8000, "CFG_LINKWIDTH_ACCEPT")
    if stop_at == ST_CFG_LW_ACCEPT:
        return

    await wait_state(dut, ST_CFG_LN_WAIT, 8000, "CFG_LANENUM_WAIT")
    dut.ordered_set_i.value = pack_tsos(link_num=LINK, lane_num=0)
    await wait_state(dut, ST_CFG_LN_ACCEPT, 8000, "CFG_LANENUM_ACCEPT")
    if stop_at == ST_CFG_LN_ACCEPT:
        return

    await wait_state(dut, ST_CFG_COMPLETE, 8000, "CFG_COMPLETE")
    if stop_at == ST_CFG_COMPLETE:
        return
    raise AssertionError(f"walk: unsupported stop_at {sname(stop_at)}")


async def stayed_put(dut, st, cycles, what):
    """Return True if the FSM never left `st`; raise nothing (the caller
    decides whether staying is the spec answer or the divergence)."""
    for _ in range(cycles):
        await RisingEdge(dut.clk_i)
        await Timer(1, units="ps")
        if state(dut) != st:
            dut._log.info(f"{what}: left {sname(st)} into {sname(state(dut))}")
            return False
    dut._log.info(f"{what}: stayed in {sname(st)} for {cycles} cycles")
    return True


# =====================================================================
# C18 -- Configuration.Complete ignores the data rate identifier
# =====================================================================
@cocotb.test()
async def run_test_c18_complete_rate_id(dut):
    clk(dut)
    await walk(dut, ST_CFG_COMPLETE)

    # Alternate the rate_id byte between gen1 and gen1-with-speed_change so no
    # eight consecutive TS2 share an identical data rate identifier.
    dut.ts1_valid_i.value = 0
    dut.ts2_valid_i.value = LANE0_MASK
    left = False
    for i in range(NEG_WATCH):
        dut.ordered_set_i.value = pack_tsos(
            link_num=LINK, lane_num=0, rate=GEN1_RATE,
            speed_change=(i // 4) & 1)
        await RisingEdge(dut.clk_i)
        await Timer(1, units="ps")
        if state(dut) != ST_CFG_COMPLETE:
            left = True
            break
    reached = state(dut)
    dut._log.info(f"C18: TS2 with alternating rate_id -> "
                  f"{'left into ' + sname(reached) if left else 'stayed put'}")

    assert not left, (
        f"C18 (Base 2.1 4.2.6.3.5.1, p.235): the Configuration.Complete exit "
        f"requires eight consecutive TS2 with matching Link and Lane numbers "
        f"AND 'identical data rate identifiers'. The TS2 stream here "
        f"alternates its rate_id byte every four cycles, so no eight "
        f"consecutive Ordered Sets carry identical identifiers -- yet the DUT "
        f"advanced to {sname(reached)}. :1945 compares link_num and lane_num "
        f"only; the data rate identifier is never examined.")


# =====================================================================
# C9 -- Linkwidth.Accept's all-PAD exit to Detect
# =====================================================================
@cocotb.test()
async def run_test_c9_linkwidth_accept_all_pad(dut):
    clk(dut)
    await walk(dut, ST_CFG_LW_ACCEPT)

    # "all Lanes receive two consecutive TS1 Ordered Sets with Link and Lane
    # numbers set to PAD" -- the spec's non-timeout route to Detect.
    dut.ordered_set_i.value = pack_tsos(link_num=None, lane_num=None)
    dut.ts1_valid_i.value = LANE0_MASK
    stayed = await stayed_put(dut, ST_CFG_LW_ACCEPT, NEG_WATCH, "C9")

    assert not stayed, (
        f"C9 (Base 2.1 4.2.6.3.2.1, p.230): 'The next state is Detect after a "
        f"2 ms timeout or if no Link can be configured or if all Lanes receive "
        f"two consecutive TS1 Ordered Sets with Link and Lane numbers set to "
        f"PAD (K23.7).' All-PAD TS1 were driven for {NEG_WATCH} cycles and the "
        f"DUT stayed in Configuration.Linkwidth.Accept. :851 implements only "
        f"the 2 ms timeout limb, so the link cannot be torn down promptly when "
        f"the partner withdraws its Link number -- it must wait out the full "
        f"2 ms (200 000 cycles) instead.")


# =====================================================================
# C16 -- Lanenum.Accept's all-PAD exit to Detect
# =====================================================================
@cocotb.test()
async def run_test_c16_lanenum_accept_all_pad(dut):
    clk(dut)
    await walk(dut, ST_CFG_LN_ACCEPT)

    dut.ordered_set_i.value = pack_tsos(link_num=None, lane_num=None)
    dut.ts1_valid_i.value = LANE0_MASK
    stayed = await stayed_put(dut, ST_CFG_LN_ACCEPT, NEG_WATCH, "C16")

    assert not stayed, (
        f"C16 (Base 2.1 4.2.6.3.3.1, p.233): 'The next state is Detect if no "
        f"Link can be configured or if all Lanes receive two consecutive TS1 "
        f"Ordered Sets with Link and Lane numbers set to PAD (K23.7).' "
        f"All-PAD TS1 were driven for {NEG_WATCH} cycles and the DUT stayed "
        f"in Configuration.Lanenum.Accept. Note that the quoted sentence "
        f"contains NO timeout -- the spec gives this substate none -- yet "
        f":880 adds a 2 ms one. C16 therefore diverges in both directions: a "
        f"missing exit and an added one.")


# =====================================================================
# P9 -- polarity inversion is implemented in the wrong state
# =====================================================================
@cocotb.test()
async def run_test_p9_polarity_in_polling_config(dut):
    clk(dut)

    # ---- POSITIVE CONTROL: the same stimulus in Polling.Active DOES invert.
    #      Without this, "did not change" below would pass on a DUT that can
    #      never change phy_rxpolarity_o at all.
    await walk(dut, ST_POLLING_ACTIVE)
    before = int(dut.phy_rxpolarity_o.value)
    dut.polarity_inverted_i.value = LANE0_MASK
    flipped = False
    for _ in range(POL_WATCH):
        await RisingEdge(dut.clk_i)
        await Timer(1, units="ps")
        if int(dut.phy_rxpolarity_o.value) != before:
            flipped = True
            break
    assert flipped, (
        f"POSITIVE CONTROL FAILED: polarity_inverted_i asserted in "
        f"Polling.Active did not change phy_rxpolarity_o (still "
        f"{before:#x}) within {POL_WATCH} cycles. Without an observable "
        f"inversion anywhere, the Polling.Configuration check below would be "
        f"vacuous, so this test cannot report on P9.")
    dut._log.info(f"positive control bit: phy_rxpolarity_o flipped "
                  f"{before:#x} -> {int(dut.phy_rxpolarity_o.value):#x} in "
                  f"Polling.Active (:654-657)")

    # ---- the measurement: the same stimulus in Polling.Configuration ----
    await walk(dut, ST_POLLING_CONFIG)
    dut.polarity_inverted_i.value = 0
    await ClockCycles(dut.clk_i, 10)
    before2 = int(dut.phy_rxpolarity_o.value)
    dut.polarity_inverted_i.value = LANE0_MASK
    flipped2 = False
    for _ in range(POL_WATCH):
        await RisingEdge(dut.clk_i)
        await Timer(1, units="ps")
        if state(dut) != ST_POLLING_CONFIG:
            break
        if int(dut.phy_rxpolarity_o.value) != before2:
            flipped2 = True
            break
    dut._log.info(f"P9: in Polling.Configuration phy_rxpolarity_o "
                  f"{'flipped' if flipped2 else 'did NOT change'} "
                  f"(started {before2:#x})")

    assert flipped2, (
        f"P9 (Base 2.1 4.2.6.2.3, p.224): 'Receiver must invert polarity if "
        f"necessary' is a requirement OF POLLING.CONFIGURATION. "
        f"polarity_inverted_i was asserted in that state and "
        f"phy_rxpolarity_o never changed. The only writer of phy_rxpolarity_c "
        f"is in the ST_POLLING_ACTIVE arm (:654-657) -- proven reachable by "
        f"the positive control above -- so the capability exists but is "
        f"placed in the wrong state, and a polarity inversion first needed "
        f"during Polling.Configuration is never applied.")
