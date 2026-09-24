"""
Multi-lane / partial-lane bring-up coverage for pcie_ltssm_downstream.

Every existing test in tb/ltssm/ drives all MAX_NUM_LANES lanes uniformly
(receiver_detected_i / phy_rxstatus_i / ts1_valid_i / ts2_valid_i / idle_valid_i
all = ALL, or all = 0). None of them exercise:
  - the |receiver_detected_i / &receiver_detected_i split in ST_DETECT_ACTIVE
    (pcie_ltssm_downstream.sv:552-599) that routes a partial-lane response
    through ST_DETECT_RX for a second confirmation instead of straight to
    ST_POLLING, or
  - "a link can be formed with a subset of the lanes that are responding" in
    Configuration.

KNOWN-SUSPECT LOGIC (read directly from the RTL, not guessed):
pcie_ltssm_downstream.sv:1735-1766, the per-lane ST_CONFIGURATION_LINKWIDTH_START
block, picks the link number to adopt via:

    if (link_width_satisfied[lane]) begin
      if ((lane == 0) || (link_width_satisfied[lane:0] == '0)) begin
        link_number_selected_per_lane_c = ordered_set_i[lane].link_num;
        lane_link_number_selected_c = '1;
      end
    end

This is entered only when link_width_satisfied[lane] is already 1, so the
slice link_width_satisfied[lane:0] always includes a set bit (bit `lane`
itself) and can never read as '0 -- the "or" branch is dead for every lane
except lane 0. In other words this block can *only* ever select the link
number from lane 0, regardless of which lanes are actually active. If lane 0
never satisfies link_width_satisfied (because it never detected a receiver /
never went active), link_number_selected latches nothing and stays at its
reset value ('0) forever -- see the always_ff at line 313
("gen_link_number"), which only updates link_number_selected when
lane_link_number_selected[i] fires for *some* i, and per the above, i=0 is
the only lane that can ever fire it.

Net effect predicted from inspection alone: any scenario where lane 0 is
*not* among the active lanes should stall in
ST_CONFIGURATION_LINKWIDTH_ACCEPT (pcie_ltssm_downstream.sv:774-799) --
the TX side keeps echoing link_number_selected==0 while the "far end"
(this testbench) echoes back the real link number, so
`ordered_set_i[lane].link_num == link_number_selected` never matches, ts1_cnt
never reaches the ST_CONFIGURATION_LINKWIDTH_ACCEPT exit threshold, and the
substate's own TwoMsTimeOut fallback (line 795-798) eventually fires
error_o=1 / next_state=ST_IDLE.

This means the risk is NOT specific to "lane 0 physically missing" as a
concept -- it is specific to whether lane 0 happens to be a member of the
active set. test_ltssm_2of4_lanes_high ([2,3] active) is just as exposed to
this as test_ltssm_lane0_absent ([1,2,3] active), while
test_ltssm_2of4_lanes_low ([0,1] active) is not, because lane 0 is present.
That is the whole point of pairing _high/_low: same lane count, different
exposure to the bug.

Separately, ST_CONFIGURATION_IDLE's exit condition (line 901,
`if ((&link_idle_satisfied) && ...)`) reduces over *all* MAX_NUM_LANES bits
of link_idle_satisfied with no lane_active_r mask -- unlike the sibling
signals lane_num_formed / ts1_cnt_satisfied / ts2_cnt_satisfied, which are
all explicitly gated `lane_active_r[lane] ? real_check : '1` (lines
1502-1506) so inactive lanes don't block an AND-reduction. link_idle_satisfied
has no such gating (line 1504: `link_idle_satisfied[lane] <= (ts1_cnt >=
8'h8);`, unconditional), so it looks like CFG_IDLE structurally requires an
idle handshake on all 4 physical lanes to exit, even for a negotiated
reduced-width link. If real, this would stall *every* partial-lane test
(not just the lane-0 ones) in ST_CONFIGURATION_IDLE. Flagged here as a
second, distinct suspect -- confirm/refute from the actual run rather than
from this comment.

Per instructions: do not modify the RTL and do not loosen these assertions
to make them pass. They encode the spec-correct expectation (reduced-width
link comes up, link_up_o=1, active_lanes_o reflects only the responding
lanes). If the DUT can't do that yet, the test should fail loudly at the
exact stuck state -- that failure IS the report.
"""
import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles
from ltssm_tb_common import *  # noqa

# sec 63 #7g-2 (D-7G.2): this bench's clock AND the RTL's CLK_PERIOD_NS, one value.
# The core passes -GCLK_PERIOD_NS=8 (tb_ltssm_b2b.sv passes .CLK_PERIOD_NS(8));
# every cycle budget below that stands for a spec time derives from it.
CLK_PERIOD_NS = 8


def _mask(active_lanes):
    m = 0
    for lane in active_lanes:
        m |= (1 << lane)
    return m


def _rxstatus_mask(active_lanes):
    """phy_rxstatus_i is MAX_NUM_LANES*3 bits wide; 3'b011 per active lane,
    all-zero (not the 3'b011 magic value) for inactive lanes -- see
    lane_status's `phy_rxstatus_i[3*i+:3] == 3'b011` check, pcie_ltssm_downstream.sv:448."""
    v = 0
    for lane in active_lanes:
        v |= (0b011 << (3 * lane))
    return v


async def _bring_up_partial(dut, active_lanes):
    """Reset -> Detect -> ... -> L0 using only `active_lanes` as
    receiver-detected / PIPE-status-ok; all other lanes are left fully idle
    (0) for every input the whole run. Mirrors bring_up_link()'s sequencing
    exactly, generalized from ALL to an arbitrary lane mask, plus the
    ST_DETECT_RX confirmation hop that only a partial (not full, not empty)
    mask takes (pcie_ltssm_downstream.sv:552-599). Raises AssertionError (via
    wait_state) at whatever state the FSM actually gets stuck in if it
    doesn't make it to L0 -- that is the intended failure-reporting
    mechanism, not a bug in this helper.
    """
    mask = _mask(active_lanes)
    assert 0 < mask < ALL, "_bring_up_partial is for genuinely partial masks only"
    rxstatus = _rxstatus_mask(active_lanes)

    cocotb.start_soon(Clock(dut.clk_i, CLK_PERIOD_NS, units="ns").start())  # 125 MHz
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

    # ---- DETECT_ACTIVE: only `active_lanes` report a receiver ----
    dut.receiver_detected_i.value = mask
    dut.phy_rxstatus_i.value = rxstatus
    dut.phy_phystatus_i.value = mask
    await ClockCycles(dut.clk_i, 3)
    dut.phy_phystatus_i.value = 0
    cocotb.start_soon(os_tx_pulser(dut))

    # Partial (not all) receiver_detected_i takes the ST_DETECT_RX branch
    # (pcie_ltssm_downstream.sv:565-568) instead of going straight to
    # ST_POLLING.
    await wait_state(dut, ST_DETECT_RX, 50, "DETECT_RX")

    # ST_DETECT_RX requires timer_r >= TwelveMsTimeOut (under SIM_FAST_LINK=1:
    # (12*10**4)/(ClockPeriodNs*10) = 12_000 // CLK_PERIOD_NS, 1500 at 8 ns)
    # before it will even look at a phystatus pulse again.  Wait past it with
    # margin, then re-assert the *same* lane pattern so
    # (lanes_detected_r == receiver_detected_i) succeeds.
    # sec 63 #7g-2: was the literal 1300 = 1200 + 100 at 10 ns; at 8 ns the
    # literal lands BEFORE the timer and the pulse is ignored (measured: 4 of 5
    # rows here and 3 of 3 in recovery_partial_lanes red).
    await ClockCycles(dut.clk_i, 12_000 // CLK_PERIOD_NS + 100)
    dut.phy_phystatus_i.value = mask
    await ClockCycles(dut.clk_i, 3)
    dut.phy_phystatus_i.value = 0
    await wait_state(dut, ST_POLLING_ACTIVE, 100, "POLLING_ACTIVE (post DETECT_RX)")

    # ---- POLLING_ACTIVE -> POLLING_CONFIGURATION: TS1s on active lanes only ----
    dut.ordered_set_i.value = pack_tsos_all_lanes(link_num=None, lane_num=None)
    dut.ts1_valid_i.value = mask
    await wait_state(dut, ST_POLLING_CONFIG, 2000, "POLLING_CONFIGURATION")

    # ---- POLLING_CONFIGURATION -> CFG_LINKWIDTH_START (TS2s, PAD/PAD) ----
    dut.ts1_valid_i.value = 0
    dut.ts2_valid_i.value = mask
    await wait_state(dut, ST_CFG_LW_START, 2000, "CFG_LINKWIDTH_START")

    # ---- LINKWIDTH_START -> ACCEPT: TS1 with link_num REAL, lane PAD ----
    dut.ts2_valid_i.value = 0
    dut.ordered_set_i.value = pack_tsos_all_lanes(link_num=LINK_NUM, lane_num=None)
    dut.ts1_valid_i.value = mask
    await wait_state(dut, ST_CFG_LW_ACCEPT, 1000, "CFG_LINKWIDTH_ACCEPT")

    # ---- ACCEPT -> LANENUM_WAIT (link_lanes_formed; keep same TS1s) ----
    await wait_state(dut, ST_CFG_LN_WAIT, 1000, "CFG_LANENUM_WAIT")

    # ---- LANENUM_WAIT -> LANENUM_ACCEPT: lane_num changes from saved PAD ----
    dut.ordered_set_i.value = pack_tsos_all_lanes(link_num=LINK_NUM, lane_num="index")
    await wait_state(dut, ST_CFG_LN_ACCEPT, 1000, "CFG_LANENUM_ACCEPT")

    # ---- LANENUM_ACCEPT -> COMPLETE (link matches, lane != PAD) ----
    await wait_state(dut, ST_CFG_COMPLETE, 1000, "CFG_COMPLETE")

    # ---- COMPLETE -> CFG_IDLE: endpoint sends TS2 w/ matching link+lane ----
    dut.ts1_valid_i.value = 0
    dut.ts2_valid_i.value = mask
    await wait_state(dut, ST_CFG_IDLE, 2000, "CFG_IDLE")

    # ---- CFG_IDLE -> L0: endpoint sends idles ----
    dut.ts2_valid_i.value = 0
    dut.idle_valid_i.value = mask
    await wait_state(dut, ST_L0, 2000, "L0")
    dut.idle_valid_i.value = 0


async def _assert_stable_link_up(dut, active_lanes):
    """Same check run_test_linkup.py uses: stop all training traffic and
    confirm the link holds in L0 rather than the transient link_up_c=1
    that ST_CONFIGURATION_IDLE also asserts (pcie_ltssm_downstream.sv:894)
    momentarily on its own."""
    await ClockCycles(dut.clk_i, 50)
    assert int(dut.ltssm_state_o.value) == ST_L0, "fell out of L0"
    assert int(dut.link_up_o.value) == 1, "link_up_o not asserted in L0"
    got = int(dut.active_lanes_o.value)
    want = _mask(active_lanes)
    assert got == want, f"active_lanes_o={got:#06b}, expected {want:#06b} (lanes {active_lanes})"
    assert int(dut.error_o.value) == 0, "error_o asserted while link is up"


@cocotb.test()
async def test_ltssm_2of4_lanes_high(dut):
    """Only lanes 2,3 detect a receiver (0,1 never active). Expect a x2 link
    at L0 with active_lanes_o reflecting lanes 2,3 only. NOTE: lane 0 is
    *not* in this active set, so per the header comment this is expected to
    be exposed to the same link_number_selected bug as
    test_ltssm_lane0_absent, not just the _low variant."""
    active_lanes = [2, 3]
    await _bring_up_partial(dut, active_lanes)
    await _assert_stable_link_up(dut, active_lanes)
    dut._log.info("2-of-4 (lanes 2,3) partial-lane link-up verified")


@cocotb.test()
async def test_ltssm_2of4_lanes_low(dut):
    """Only lanes 0,1 detect a receiver. Paired with _high to isolate
    whether lane index matters when it shouldn't -- lane 0 IS in the active
    set here, unlike _high."""
    active_lanes = [0, 1]
    await _bring_up_partial(dut, active_lanes)
    await _assert_stable_link_up(dut, active_lanes)
    dut._log.info("2-of-4 (lanes 0,1) partial-lane link-up verified")


@cocotb.test()
async def test_ltssm_lane0_absent(dut):
    """Lanes 1,2,3 active, lane 0 NOT active. Targeted stress test for the
    link_number_selected bug described in the module docstring
    (pcie_ltssm_downstream.sv:1761): link_number_selected can only ever be
    latched from lane 0's per-lane comb block, so if lane 0 never goes
    active it should stall in ST_CONFIGURATION_LINKWIDTH_ACCEPT."""
    active_lanes = [1, 2, 3]
    await _bring_up_partial(dut, active_lanes)
    await _assert_stable_link_up(dut, active_lanes)
    dut._log.info("3-of-4 (lanes 1,2,3), lane 0 absent, link-up verified")


@cocotb.test()
async def test_ltssm_1of4_lane_only(dut):
    """Only lane 3 active -- degenerate x1 case via the partial-lane path
    (not the existing all-4 x1-style coverage). Lane 0 is absent here too."""
    active_lanes = [3]
    await _bring_up_partial(dut, active_lanes)
    await _assert_stable_link_up(dut, active_lanes)
    dut._log.info("1-of-4 (lane 3 only) partial-lane link-up verified")


@cocotb.test()
async def test_ltssm_no_lanes_stays_detect(dut):
    """Negative control: zero lanes ever detect a receiver. Must never
    assert link_up_o or error_o -- should just keep cycling Detect
    substates, not race off to Polling/Configuration or flag an error."""
    cocotb.start_soon(Clock(dut.clk_i, CLK_PERIOD_NS, units="ns").start())
    drive_idle_inputs(dut)
    dut.rst_i.value = 1
    await ClockCycles(dut.clk_i, 5)
    dut.rst_i.value = 0
    await ClockCycles(dut.clk_i, 5)

    dut.en_i.value = 1
    await wait_state(dut, ST_DETECT_QUIET, 50, "DETECT_QUIET")

    # Exit elec-idle so we actually reach DETECT_ACTIVE and can pulse
    # phystatus with zero receivers detected (the interesting no-receiver
    # path, pcie_ltssm_downstream.sv:569-571), rather than just waiting out
    # the 12ms elec-idle timeout with no assertions made either way.
    dut.phy_rxelecidle_i.value = ALL
    await ClockCycles(dut.clk_i, 3)
    dut.phy_rxelecidle_i.value = 0
    await wait_state(dut, ST_DETECT_ACTIVE, 50, "DETECT_ACTIVE")

    dut.receiver_detected_i.value = 0
    dut.phy_rxstatus_i.value = 0
    dut.phy_phystatus_i.value = ALL
    await ClockCycles(dut.clk_i, 3)
    dut.phy_phystatus_i.value = 0

    # No receiver anywhere -> next_state = ST_IDLE (line 569-571), then
    # straight back to ST_DETECT_QUIET on the next cycle since en_i is
    # still 1 and curr_data_rate_r.rate == gen1.
    await wait_state(dut, ST_DETECT_QUIET, 20, "DETECT_QUIET (retry after no-receiver)")

    assert int(dut.ltssm_state_o.value) != ST_POLLING_ACTIVE, \
        "FSM proceeded to Polling despite receiver_detected_i==0"

    # Sit here for a while and confirm nothing ever comes up.
    await ClockCycles(dut.clk_i, 5000)

    final_state = int(dut.ltssm_state_o.value)
    assert final_state in (ST_IDLE, ST_DETECT_QUIET, ST_DETECT_ACTIVE), (
        f"expected to remain in a Detect substate with zero active lanes, "
        f"got {STATE_NAMES.get(final_state, hex(final_state))}"
    )
    assert int(dut.link_up_o.value) == 0, "link_up_o asserted with zero active lanes"
    assert int(dut.error_o.value) == 0, \
        "error_o unexpectedly asserted with zero active lanes (should just idle in Detect)"
    assert int(dut.active_lanes_o.value) == 0, \
        "active_lanes_o non-zero with zero active lanes"
    dut._log.info("NO-LANES NEGATIVE CONTROL VERIFIED: stays in Detect, no link_up/error")
