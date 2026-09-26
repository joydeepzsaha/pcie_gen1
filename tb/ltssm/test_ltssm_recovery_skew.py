"""
Recovery with a DIVERGENT lane mask: lane_active_r ⊋ receiver_detected_i.

WHY THIS FILE EXISTS
--------------------
pcie_ltssm_downstream.sv has two families of per-lane "satisfied" signals that
differ only in which mask gates them, and they sit three lines apart:

    1612  ts1_cnt_satisfied[lane]   <= lane_active_r[lane]       ? (ts1_cnt == 8'h8) : '1;
    1613  ts2_cnt_satisfied[lane]   <= lane_active_r[lane]       ? (ts2_cnt == 8'h8) : '1;
    1616  lanes_ts1_satisfied[lane] <= receiver_detected_i[lane] ? (ts1_cnt == 8'h8) : '1;
    1617  lanes_ts2_satisfied[lane] <= receiver_detected_i[lane] ? (ts2_cnt == 8'h8) : '1;

ST_RECOVERY_RCVR_CFG's exit at :1269 reduces over `ts2_cnt_satisfied` -- the
lane_active_r-gated one.  That is the spec-correct choice: Base 2.1 §4.2.6.4.3
p.244 requires eight consecutive TS2 "on all configured Lanes", and
lane_active_r is this RTL's notion of *configured* (latched at :478 on
`phy_phystatus_i[i] && phy_rxstatus_i[3*i+:3] == 3'b011`, sticky until
phy_phystatus_rst_i, exported as active_lanes_o at :309).  receiver_detected_i
is an independent input port, read only in ST_DETECT_ACTIVE and ST_DETECT_RX.

Before this file, NO bench in tb/ltssm/ distinguished the two.  Every bench
either drove receiver_detected_i and the phy_rxstatus_i/phy_phystatus_i pair
from the same lane list -- making the gates indistinguishable -- or produced
the OPPOSITE skew (lane_active_r ⊋ receiver_detected_i) only in
test_ltssm_x4_oracles / test_ltssm_b2b_x4, which never enter Recovery.  Since
every reader of ts2_cnt_satisfied is in a Recovery state (:1073, :1104, :1269,
:1286, :1318), the gate choice was untestable and two mutants proved it:

    MG1  :1613 gate lane_active_r -> receiver_detected_i   -- SURVIVED
    MG2  :1269 operand -> the look-alike lanes_ts2_satisfied -- SURVIVED

(evidence/fix-arc-1/FINDINGS_GATE_CHOICE.md, run against 7 targets.)  Anyone
later "tidying" :1613 to match its lanes_* neighbours would introduce a silent
divergence from p.244 that the whole suite would wave through.  This file is
the row that stops that; §5 of those findings specifies it.

WHAT IT DOES
------------
Creates the one combination nothing had: all four lanes CONFIGURED
(phy_rxstatus_i = RXSTATUS_ALL_OK + phy_phystatus_i = ALL) while only lane 0
ever reports a DETECTED RECEIVER (receiver_detected_i = 0x1), then traverses to
ST_RECOVERY_RCVR_CFG and offers eight TS2 on lane 0 alone.

Spec-correct RTL must STAY in RcvrCfg -- three configured Lanes are unsatisfied.
Under MG1 or MG2 those three are excused as '1 and the FSM LEAVES.

The partial receiver_detected_i also routes bring-up through ST_DETECT_RX
(:589's `|receiver_detected_i` without `&receiver_detected_i`), which re-confirms
the same mask after TwelveMsTimeOut -- so this file exercises that hop with a
mask that, unlike test_ltssm_partial_lanes.py's, does NOT match lane_active_r.

These are ORDINARY PASS rows.  Nothing here is expect_fail: the RTL conforms,
and it conforms *because* of fix-arc 1's :1269 repair (`&ts2_cnt_satisfied`,
commit 4e561ad).  This row is what proves the repair picked the right operand
and the right gate, which the gate record alone cannot show (§22.77).
"""
import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles
from ltssm_tb_common import *  # noqa

# sec 63 #7g-2 (D-7G.2): this bench's clock AND the RTL's CLK_PERIOD_NS, one value.
# The core passes -GCLK_PERIOD_NS=8 (tb_ltssm_b2b.sv passes .CLK_PERIOD_NS(8));
# every cycle budget below that stands for a spec time derives from it.
CLK_PERIOD_NS = 8

# receiver_detected_i is presented on lane 0 ONLY, for the entire run, while
# phy_rxstatus_i/phy_phystatus_i configure all four.  That inequality is the
# whole point of the file -- see the module docstring.
RD_MASK = 0x1

# ST_DETECT_RX will not re-examine phystatus until timer_r >= TwelveMsTimeOut,
# which is (12*10**4)/(ClockPeriodNs*10) = 1200 cycles at SIM_FAST_LINK=1
# (pcie_ltssm_downstream.sv:111, :613).  Same margin test_ltssm_partial_lanes.py
# uses for the same hop.
DETECT_RX_WAIT = 12_000 // CLK_PERIOD_NS + 100   # 63 #7g-2: SIM_FAST 12 ms + 100; was 1300 at 10 ns

# The hold window.  Sized against the SLOWEST term of :1269 that is not the one
# under test: ordered_set_sent_cnt_r must reach 16, and it advances once per
# os_tx_pulser pulse (one every 4 cycles), so ~64 cycles.  400 is ~6x that, so
# "still in RcvrCfg at the end" cannot be explained by that counter lagging.
HOLD_CYCLES = 400

# Budget for the control arm's exit.  Lanes 1-3 need 8 TS2 (~8-10 cycles) and
# then :1269 fires on the next transmitted-pulse (<=4).  200 is a wide margin
# that still distinguishes "left promptly" from "left eventually".
RELEASE_BUDGET = 200


async def _bring_up_skewed(dut):
    """Reset -> ... -> L0 with lane_active_r = ALL FOUR but
    receiver_detected_i = lane 0 only.

    Mirrors ltssm_tb_common.bring_up_link() step for step, with exactly two
    differences: receiver_detected_i is RD_MASK rather than ALL, and the
    resulting partial-detect path takes the ST_DETECT_RX confirmation hop.
    Every TS/idle strobe stays at ALL, because lane_active_r is ALL and the
    Configuration substates gate on lane_active_r.
    """
    drive_idle_inputs(dut)
    dut.rst_i.value = 1
    await ClockCycles(dut.clk_i, 5)
    dut.rst_i.value = 0
    await ClockCycles(dut.clk_i, 5)

    assert int(dut.ltssm_state_o.value) == ST_IDLE
    assert int(dut.link_up_o.value) == 0

    # ---- IDLE -> DETECT_QUIET ----
    dut.en_i.value = 1
    await wait_state(dut, ST_DETECT_QUIET, 50, "DETECT_QUIET")

    # ---- DETECT_QUIET -> DETECT_ACTIVE via elec-idle exit edge (1 -> 0) ----
    dut.phy_rxelecidle_i.value = ALL
    await ClockCycles(dut.clk_i, 3)
    dut.phy_rxelecidle_i.value = 0
    await wait_state(dut, ST_DETECT_ACTIVE, 50, "DETECT_ACTIVE")
    assert int(dut.phy_txdetectrx_o.value) == 1, "RC must request rx-detect"

    # ---- DETECT_ACTIVE: THE SKEW.  rxstatus/phystatus say all four lanes are
    #      configured (lane_active_r latches combinationally on the pair, so
    #      rxstatus must already read 3'b011 per lane BEFORE the phystatus
    #      pulse); receiver_detected_i says lane 0 only. ----
    dut.receiver_detected_i.value = RD_MASK
    dut.phy_rxstatus_i.value = RXSTATUS_ALL_OK
    dut.phy_phystatus_i.value = ALL
    await ClockCycles(dut.clk_i, 3)
    dut.phy_phystatus_i.value = 0
    cocotb.start_soon(os_tx_pulser(dut))

    # A partial receiver_detected_i takes :589's ST_DETECT_RX branch instead of
    # going straight to ST_POLLING -- `|receiver_detected_i` is true but
    # `&receiver_detected_i` is not.
    await wait_state(dut, ST_DETECT_RX, 50, "DETECT_RX")
    assert int(dut.active_lanes_o.value) == ALL, (
        f"lane_active_r={int(dut.active_lanes_o.value):#06b} at DETECT_RX -- the "
        f"skew did not take: rxstatus/phystatus should have configured all four "
        f"lanes independently of receiver_detected_i"
    )

    # ---- DETECT_RX -> POLLING: after TwelveMsTimeOut, re-present the SAME
    #      receiver_detected_i mask so :617's (lanes_detected_r ==
    #      receiver_detected_i) holds.  Keep rxstatus at ALL_OK so the
    #      re-pulse does not disturb lane_active_r. ----
    await ClockCycles(dut.clk_i, DETECT_RX_WAIT)
    dut.phy_phystatus_i.value = ALL
    await ClockCycles(dut.clk_i, 3)
    dut.phy_phystatus_i.value = 0
    await wait_state(dut, ST_POLLING_ACTIVE, 100, "POLLING_ACTIVE (post DETECT_RX)")

    # ---- POLLING_ACTIVE -> POLLING_CONFIGURATION ----
    dut.ordered_set_i.value = pack_tsos_all_lanes(link_num=None, lane_num=None)
    dut.ts1_valid_i.value = ALL
    await wait_state(dut, ST_POLLING_CONFIG, 2000, "POLLING_CONFIGURATION")

    # ---- POLLING_CONFIGURATION -> CFG_LINKWIDTH_START (TS2s, PAD/PAD) ----
    dut.ts1_valid_i.value = 0
    dut.ts2_valid_i.value = ALL
    await wait_state(dut, ST_CFG_LW_START, 2000, "CFG_LINKWIDTH_START")

    # ---- LINKWIDTH_START -> ACCEPT: TS1 with link_num REAL, lane PAD ----
    dut.ts2_valid_i.value = 0
    dut.ordered_set_i.value = pack_tsos_all_lanes(link_num=LINK_NUM, lane_num=None)
    dut.ts1_valid_i.value = ALL
    await wait_state(dut, ST_CFG_LW_ACCEPT, 1000, "CFG_LINKWIDTH_ACCEPT")

    # ---- ACCEPT -> LANENUM_WAIT ----
    await wait_state(dut, ST_CFG_LN_WAIT, 1000, "CFG_LANENUM_WAIT")

    # ---- LANENUM_WAIT -> LANENUM_ACCEPT: lane_num changes from saved PAD ----
    dut.ordered_set_i.value = pack_tsos_all_lanes(link_num=LINK_NUM, lane_num="index")
    await wait_state(dut, ST_CFG_LN_ACCEPT, 1000, "CFG_LANENUM_ACCEPT")

    # ---- LANENUM_ACCEPT -> COMPLETE ----
    await wait_state(dut, ST_CFG_COMPLETE, 1000, "CFG_COMPLETE")

    # ---- COMPLETE -> CFG_IDLE ----
    dut.ts1_valid_i.value = 0
    dut.ts2_valid_i.value = ALL
    await wait_state(dut, ST_CFG_IDLE, 2000, "CFG_IDLE")

    # ---- CFG_IDLE -> L0 ----
    dut.ts2_valid_i.value = 0
    dut.idle_valid_i.value = ALL
    await wait_state(dut, ST_L0, 2000, "L0")
    dut.idle_valid_i.value = 0


async def _enter_rcvr_cfg(dut):
    """L0 -> ST_RECOVERY_RCVR_LOCK -> ST_RECOVERY_RCVR_CFG on TS1s.

    ST_L0 exits on `|ts1_valid_i || |ts2_valid_i || ...` (:936-:951); RCVR_LOCK
    exits at :1073 on `&(ts1_cnt_satisfied | ts2_cnt_satisfied)`, which is
    lane_active_r-gated, so all four lanes must reach ts1_cnt == 8.  Speed
    change is deliberately 0 throughout -- :1269's `speed_change_bit_set == '0'`
    term must be satisfiable for the hold to be attributable to the reduction
    under test.
    """
    dut.ordered_set_i.value = pack_tsos_all_lanes(
        link_num=LINK_NUM, lane_num="index", rate=GEN1_RATE, speed_change=0)
    dut.ts1_valid_i.value = ALL
    await wait_state(dut, ST_RECOVERY_RCVR_LOCK, 100, "RECOVERY_RCVR_LOCK")
    await wait_state(dut, ST_RECOVERY_RCVR_CFG, 500, "RECOVERY_RCVR_CFG")
    dut.ts1_valid_i.value = 0


@cocotb.test()
async def test_recovery_skew_masks_diverge_at_l0(dut):
    """Precondition row: the divergent mask is actually created.

    Stands alone so that a failure to set up the skew reports itself here,
    instead of surfacing as a confusing hold failure in the row below.  Asserts
    the link trains to L0 with all four lanes CONFIGURED while only lane 0 ever
    reported a detected receiver -- the combination no bench in tb/ltssm/ had.
    """
    cocotb.start_soon(Clock(dut.clk_i, CLK_PERIOD_NS, units="ns").start())
    await _bring_up_skewed(dut)

    await ClockCycles(dut.clk_i, 50)
    assert int(dut.ltssm_state_o.value) == ST_L0, "fell out of L0"
    assert int(dut.link_up_o.value) == 1, "link_up_o not asserted in L0"
    assert int(dut.active_lanes_o.value) == ALL, (
        f"active_lanes_o={int(dut.active_lanes_o.value):#06b}, expected {ALL:#06b} -- "
        f"lane_active_r must follow phy_rxstatus_i/phy_phystatus_i, NOT "
        f"receiver_detected_i"
    )
    assert int(dut.receiver_detected_i.value) == RD_MASK, (
        f"receiver_detected_i={int(dut.receiver_detected_i.value):#06b}, expected "
        f"{RD_MASK:#06b} -- the drive sequence must hold the skew all the way to L0"
    )
    assert int(dut.active_lanes_o.value) != int(dut.receiver_detected_i.value), \
        "the two masks are equal -- this bench would then measure nothing"
    assert int(dut.error_o.value) == 0, "error_o asserted while link is up"
    dut._log.info(
        f"SKEW ESTABLISHED AT L0: active_lanes_o="
        f"{int(dut.active_lanes_o.value):#06b} (all four CONFIGURED) vs "
        f"receiver_detected_i={int(dut.receiver_detected_i.value):#06b} (lane 0 only) "
        f"-- the divergent-mask direction no prior bench drove"
    )


@cocotb.test()
async def test_recovery_skew_rcvrcfg_needs_all_configured_lanes(dut):
    """Base 2.1 §4.2.6.4.3 p.244: RcvrCfg exits on eight consecutive TS2 "on all
    configured Lanes".  With four Lanes configured and TS2 offered on lane 0
    only, the FSM must STAY.

    Two arms in ONE drive sequence, deliberately:

      HOLD    -- TS2 on lane 0 only; assert still in RCVR_CFG after HOLD_CYCLES.
      RELEASE -- widen to all four; assert it leaves promptly.

    The RELEASE arm is not decoration.  It is the measurement that makes the
    HOLD arm mean something: it proves :1269's other three terms
    (speed_change_bit_set == 0, ordered_set_sent_cnt_r >= 16,
    ordered_set_tranmitted_i) were ALREADY satisfied during the hold, so the
    hold is attributable to `&ts2_cnt_satisfied` and not to a co-blocking term
    that had simply not matured.  Splitting the arms into separate rows would
    put them in separate runs and lose that.  §22.66's one-assertion-per-row
    rule binds expect_fail rows; this row is an ordinary PASS.

    Kills MG1 (:1613 gate -> receiver_detected_i) and MG2 (:1269 operand ->
    lanes_ts2_satisfied): under either, lanes 1-3 are excused as '1 and the HOLD
    assertion fires.
    """
    cocotb.start_soon(Clock(dut.clk_i, CLK_PERIOD_NS, units="ns").start())
    await _bring_up_skewed(dut)
    assert int(dut.active_lanes_o.value) == ALL
    await _enter_rcvr_cfg(dut)

    # ---- HOLD: eight TS2 on lane 0 alone.  ts2_cnt saturates at 8 (:1804's
    #      `(ts2_cnt >= 8'h8) ? 8'h8 : ts2_cnt + 1`), so lane 0's term is
    #      satisfied and STAYS satisfied for the whole window; lanes 1-3 never
    #      leave 0 while lane_active_r keeps them in the reduction. ----
    dut.ts2_valid_i.value = RD_MASK
    await ClockCycles(dut.clk_i, HOLD_CYCLES)

    held = int(dut.ltssm_state_o.value)
    assert held == ST_RECOVERY_RCVR_CFG, (
        f"left ST_RECOVERY_RCVR_CFG for "
        f"{STATE_NAMES.get(held, hex(held))} after {HOLD_CYCLES} cycles with TS2 on "
        f"lane 0 only, while active_lanes_o="
        f"{int(dut.active_lanes_o.value):#06b} says four Lanes are configured. "
        f"Base 2.1 4.2.6.4.3 p.244 requires eight consecutive TS2 on ALL "
        f"configured Lanes. This is the signature of :1269 reducing over a "
        f"receiver_detected_i-gated signal (MG1's gate swap or MG2's operand "
        f"swap) instead of the lane_active_r-gated ts2_cnt_satisfied."
    )
    # §63 #7k (§22.87): this line asserted link_up_o == 0 mid-Recovery -- the
    # non-conformant value -- until the LinkUp fix.  Base 2.1 Table 4-7 p.216:
    # LinkUp = 1b in Recovery; the Data Link Layer monitors it (§3.2.1 p.159).
    assert int(dut.link_up_o.value) == 1, "link_up_o fell mid-Recovery (Table 4-7 p.216: LinkUp = 1b)"
    dut._log.info(
        f"HOLD VERIFIED: parked in RECOVERY_RCVR_CFG for {HOLD_CYCLES} cycles "
        f"with TS2 on lane 0 of 4 configured -- three configured Lanes unsatisfied"
    )

    # ---- RELEASE (control arm): satisfy all four.  If this does NOT exit
    #      promptly, the hold above proved nothing -- some other term of :1269
    #      was blocking and the test is vacuous. ----
    dut.ts2_valid_i.value = ALL
    await wait_state(dut, ST_RECOVERY_IDLE, RELEASE_BUDGET,
                     "RECOVERY_IDLE (control arm: all four Lanes satisfied)")
    dut._log.info(
        f"RELEASE VERIFIED: left RECOVERY_RCVR_CFG within {RELEASE_BUDGET} cycles "
        f"once all four configured Lanes had eight TS2 -- so speed_change_bit_set, "
        f"ordered_set_sent_cnt_r >= 16 and ordered_set_tranmitted_i were already "
        f"satisfied during the hold, and the hold is attributable to "
        f"&ts2_cnt_satisfied alone"
    )

    # ---- and the width survived the retrain ----
    assert int(dut.active_lanes_o.value) == ALL, \
        "active_lanes_o changed across Recovery"
    assert int(dut.error_o.value) == 0, "error_o asserted across a clean Recovery"
    dut._log.info("RCVR_CFG ALL-CONFIGURED-LANES ORACLE VERIFIED (kills MG1 and MG2)")
