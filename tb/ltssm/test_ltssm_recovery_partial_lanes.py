"""
Recovery under reduced-width conditions.

test_ltssm_recovery.py only ever exercises L0 -> Recovery -> L0 at full
width (all 4 lanes active) -- the same blind spot that hid Bug A
(link_number_selected only latchable from lane 0) and Bug B
(ST_CONFIGURATION_IDLE requiring idle on all 4 physical lanes) in
Configuration. This file is the Recovery-side counterpart: bring a link up
at reduced width via _bring_up_partial() (imported from
test_ltssm_partial_lanes.py, same helper, same fileset), force re-entry
into Recovery from L0 the same way test_ltssm_recovery.py does (TS1/TS2
while in L0 -- see pcie_ltssm_downstream.sv:936-951, ST_L0's own body,
which exits on `|ts1_valid_i || |ts2_valid_i || (directed_speed_change_i
&& !changed_speed_recovery_r)` -- deliberately NOT the speed-change
trigger, which is out of scope here same as it was for the Configuration
audit), and walk RCVR_LOCK -> RCVR_CFG -> RCVR_IDLE -> L0.

Per the prior audit, RCVR_LOCK's and RCVR_CFG's exit conditions
(pcie_ltssm_downstream.sv:996, 1185-1201) both AND-reduce over
ts1_cnt_satisfied/ts2_cnt_satisfied, which ARE lane_active_r-gated
(`lane_active_r[lane] ? real_check : '1`, lines 1505-1506) -- the same
gating pattern Bug B's fix (link_idle_satisfied) now also follows -- so on
paper this should already work correctly at reduced width without any RTL
change. These tests confirm that empirically.

Also covers the two edge cases from the audit follow-up:
  - an originally-INACTIVE lane sending traffic during Recovery should be
    a no-op (lane_active_r doesn't change without a phy_phystatus_rst_i
    pulse, which nothing here asserts, so the gated satisfied-checks keep
    ignoring that lane regardless of what it does)
  - an originally-ACTIVE lane going silent during Recovery should NOT let
    the FSM quietly complete Recovery at a smaller width -- per the module
    header (autonomous lane-width reconfiguration is not supported), the
    expected behavior is a stall in RCVR_LOCK (eventually timing out to
    RCVR_LOCK_TIMEOUT at the real 24ms/2.4M-cycle TwentyFourMsTimeOut,
    which this test does not wait out -- it only confirms the stall within
    a budget far larger than the ~10s-of-cycles a real completion takes).
"""
import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles
from ltssm_tb_common import *  # noqa
from test_ltssm_partial_lanes import _bring_up_partial, _mask

# sec 63 #7g-2 (D-7G.2): this bench's clock AND the RTL's CLK_PERIOD_NS, one value.
# The core passes -GCLK_PERIOD_NS=8 (tb_ltssm_b2b.sv passes .CLK_PERIOD_NS(8));
# every cycle budget below that stands for a spec time derives from it.
CLK_PERIOD_NS = 8


async def _bring_up_and_check(dut, active_lanes):
    await _bring_up_partial(dut, active_lanes)
    assert int(dut.ltssm_state_o.value) == ST_L0
    assert int(dut.link_up_o.value) == 1
    assert int(dut.active_lanes_o.value) == _mask(active_lanes)
    dut._log.info(f"setup complete: L0 at reduced width, lanes={active_lanes}")


@cocotb.test()
async def test_ltssm_recovery_2of4_lanes(dut):
    """L0 -> Recovery -> L0 at reduced width (lanes 1,3), no speed change.
    Mirrors test_ltssm_recovery.py's run_test_recovery_no_speed_change,
    generalized from ALL to a partial mask throughout."""
    cocotb.start_soon(Clock(dut.clk_i, CLK_PERIOD_NS, units="ns").start())
    active_lanes = [1, 3]
    mask = _mask(active_lanes)
    await _bring_up_and_check(dut, active_lanes)

    # ---- L0 -> RECOVERY_RCVR_LOCK: partner starts sending TS1s on the
    #      SAME active lanes only ----
    dut.ordered_set_i.value = pack_tsos_all_lanes(
        link_num=LINK_NUM, lane_num="index", rate=GEN1_RATE, speed_change=0)
    dut.ts1_valid_i.value = mask
    await wait_state(dut, ST_RECOVERY_RCVR_LOCK, 100, "RECOVERY_RCVR_LOCK")

    # ---- RCVR_LOCK -> RCVR_CFG: needs ts1_cnt==8 on every ACTIVE lane
    #      (lane_active_r-gated at line 1505-1506; inactive lanes trivially
    #      satisfy) ----
    await wait_state(dut, ST_RECOVERY_RCVR_CFG, 500, "RECOVERY_RCVR_CFG")

    # ---- RCVR_CFG -> RECOVERY_IDLE: partner switches to TS2s ----
    dut.ts1_valid_i.value = 0
    dut.ts2_valid_i.value = mask
    await wait_state(dut, ST_RECOVERY_IDLE, 2000, "RECOVERY_IDLE")

    # ---- RECOVERY_IDLE -> L0: partner sends idles only ----
    dut.ts2_valid_i.value = 0
    dut.idle_valid_i.value = mask
    await wait_state(dut, ST_L0, 2000, "L0 (via Recovery)")

    dut.idle_valid_i.value = 0
    await ClockCycles(dut.clk_i, 50)
    assert int(dut.ltssm_state_o.value) == ST_L0, "fell out of L0 after Recovery"
    assert int(dut.link_up_o.value) == 1, "link_up dropped after Recovery"
    assert int(dut.active_lanes_o.value) == mask, \
        "active_lanes_o changed across Recovery -- width did not survive retrain"
    assert int(dut.error_o.value) == 0
    dut._log.info("RECOVERY AT REDUCED WIDTH (lanes 1,3) VERIFIED: same width in and out")


@cocotb.test()
async def test_ltssm_recovery_inactive_lane_joins(dut):
    """Edge case: an originally-inactive lane (lane 1) sends TS1/TS2/idle
    traffic throughout Recovery alongside the real active lanes (0,2).
    Expect this to be a no-op -- lane_active_r isn't re-latched without a
    phy_phystatus_rst_i pulse (never asserted here), so the gated
    ts1_cnt_satisfied/ts2_cnt_satisfied checks should keep ignoring lane 1
    regardless of what it does, and active_lanes_o should still read the
    original mask after Recovery completes."""
    cocotb.start_soon(Clock(dut.clk_i, CLK_PERIOD_NS, units="ns").start())
    active_lanes = [0, 2]
    mask = _mask(active_lanes)
    intruder_lane = 1
    joined_mask = mask | _mask([intruder_lane])
    await _bring_up_and_check(dut, active_lanes)

    dut.ordered_set_i.value = pack_tsos_all_lanes(
        link_num=LINK_NUM, lane_num="index", rate=GEN1_RATE, speed_change=0)
    # drive TS1s on the real active lanes AND the intruder lane
    dut.ts1_valid_i.value = joined_mask
    await wait_state(dut, ST_RECOVERY_RCVR_LOCK, 100, "RECOVERY_RCVR_LOCK")
    await wait_state(dut, ST_RECOVERY_RCVR_CFG, 500, "RECOVERY_RCVR_CFG")

    dut.ts1_valid_i.value = 0
    dut.ts2_valid_i.value = joined_mask
    await wait_state(dut, ST_RECOVERY_IDLE, 2000, "RECOVERY_IDLE")

    dut.ts2_valid_i.value = 0
    dut.idle_valid_i.value = joined_mask
    await wait_state(dut, ST_L0, 2000, "L0 (via Recovery, intruder lane active)")

    dut.idle_valid_i.value = 0
    await ClockCycles(dut.clk_i, 50)
    assert int(dut.ltssm_state_o.value) == ST_L0
    assert int(dut.link_up_o.value) == 1
    assert int(dut.active_lanes_o.value) == mask, (
        f"active_lanes_o={int(dut.active_lanes_o.value):#06b} picked up the "
        f"intruder lane (expected {mask:#06b}) -- lane_active_r should not "
        f"change without a phy_phystatus_rst_i pulse"
    )
    assert int(dut.error_o.value) == 0
    dut._log.info("INACTIVE-LANE-JOINS EDGE CASE VERIFIED: intruder lane ignored, width unchanged")


@cocotb.test()
async def test_ltssm_recovery_active_lane_drops(dut):
    """Edge case: of the 3 originally-active lanes (1,2,3), lane 2 goes
    silent partway into Recovery (no more TS1/TS2 from it). Per the module
    header, autonomous lane-width reconfiguration is not supported, so the
    FSM should NOT quietly complete Recovery at a smaller width -- it
    should stall in RCVR_LOCK (line 996's `&(ts1_cnt_satisfied |
    ts2_cnt_satisfied)` can never go true again for lane 2, since
    lane_active_r[2] stays 1 and nothing is gating that lane's requirement
    away). It would eventually reach RCVR_LOCK_TIMEOUT at the real
    TwentyFourMsTimeOut (2,400,000 cycles, not SIM_FAST_LINK-scaled -- see
    test_ltssm_config_timeout.py's own note on this same constant) -- this
    test does not wait that out, it only confirms the stall within a
    budget far larger than the ~10s-of-cycles a real RCVR_CFG transition
    takes, which is enough to distinguish "sane stall pending the real
    watchdog" from "silently completed at reduced width" or an
    RTL-level hang in some other state."""
    cocotb.start_soon(Clock(dut.clk_i, CLK_PERIOD_NS, units="ns").start())
    active_lanes = [1, 2, 3]
    dropped_lane = 2
    mask = _mask(active_lanes)
    await _bring_up_and_check(dut, active_lanes)

    dut.ordered_set_i.value = pack_tsos_all_lanes(
        link_num=LINK_NUM, lane_num="index", rate=GEN1_RATE, speed_change=0)
    dut.ts1_valid_i.value = mask
    await wait_state(dut, ST_RECOVERY_RCVR_LOCK, 100, "RECOVERY_RCVR_LOCK")

    # Let lane 2's ts1_cnt start climbing for a couple cycles, then drop it.
    await ClockCycles(dut.clk_i, 3)
    remaining_mask = mask & ~_mask([dropped_lane])
    dut.ts1_valid_i.value = remaining_mask

    # Should NOT reach RCVR_CFG on this budget -- lane 2 (still counted
    # active) can never satisfy ts1_cnt_satisfied/ts2_cnt_satisfied again.
    # 2000 cycles is far more than the ~10s-of-cycles a real RCVR_CFG
    # transition takes (confirmed by the other two tests above), so still
    # being parked in RCVR_LOCK here is the "sane stall" signal, not a
    # false negative from an under-budgeted wait.
    await ClockCycles(dut.clk_i, 2000)
    final_state = int(dut.ltssm_state_o.value)
    assert final_state == ST_RECOVERY_RCVR_LOCK, (
        f"expected to still be stalled in RECOVERY_RCVR_LOCK 2000 cycles after "
        f"an active lane went silent, got "
        f"{STATE_NAMES.get(final_state, hex(final_state))}"
    )
    assert int(dut.link_up_o.value) == 0, "link_up_o should not be asserted mid-Recovery"
    assert int(dut.active_lanes_o.value) == mask, \
        "active_lanes_o should not have silently dropped the stalled lane"
    dut._log.info("ACTIVE-LANE-DROPS EDGE CASE VERIFIED: sane stall, no silent width reduction")
