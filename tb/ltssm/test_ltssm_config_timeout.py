"""
Bug 5 regression coverage across Configuration's substates: each substate's
timeout fallback (next_state = ST_IDLE) used to sit inside
if (ordered_set_tranmitted_i), same shape as the Polling.Active Bug 4 --
so a silent TX handshake (no ordered_set_tranmitted_i pulses) would hang
forever in that substate instead of timing out. Confirmed by inspection and
by these 5 tests hanging pre-fix; now that the fix moved each timeout check
out from behind the ordered_set_tranmitted_i gate (guarded by
next_state == curr_state, same as Bug 4), all 5 must reach ST_IDLE within
budget instead of hanging.

ST_CONFIGURATION_IDLE is a structurally-different control -- its timeout
check was never gated the same way -- and is expected to PASS whether or
not the Bug 5 fix is applied.

NOTE on budgets: ST_CONFIGURATION_LINKWIDTH_START's timeout constant is
TwentyFourMsTimeOut (24ms), not TwoMsTimeOut (2ms) like the other four, and
it is NOT scaled by SIM_FAST_LINK (confirmed: only TwelveMsTimeOut /
OneMsTimeOut / MinTS1sPolling are) -- it's a real 2,400,000 cycles either
way. It therefore uses its own LINKWIDTH_START_TIMEOUT_BUDGET (24ms +
margin) instead of the shared 250_000-cycle (2ms + margin) budget used by
the other four TwoMsTimeOut substates and the control. 
"""
import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ClockCycles
from ltssm_tb_common import *  # noqa

# sec 63 #7g-2 (D-7G.2): this bench's clock AND the RTL's CLK_PERIOD_NS, one value.
# The core passes -GCLK_PERIOD_NS=8 (tb_ltssm_b2b.sv passes .CLK_PERIOD_NS(8));
# every cycle budget below that stands for a spec time derives from it.
CLK_PERIOD_NS = 8

CONFIG_TIMEOUT_BUDGET = 2_500_000 // CLK_PERIOD_NS  # 2ms + margin, for the TwoMsTimeOut substates (63 #7g-2: was 250_000 at 10 ns)

# TwentyFourMsTimeOut is NOT scaled by SIM_FAST_LINK (confirmed: only
# TwelveMsTimeOut/OneMsTimeOut/MinTS1sPolling are) -- it's a real 2,400,000
# cycles. CONFIG_TIMEOUT_BUDGET is far too short to ever observe it firing,
# fix or no fix, so LINKWIDTH_START needs its own budget to be a meaningful
# post-fix check rather than trivially failing on budget alone. Matches the
# same "24ms + margin" sizing already used in test_ltssm_polling_timeout.py.
LINKWIDTH_START_TIMEOUT_BUDGET = 25_000_000 // CLK_PERIOD_NS  # 63 #7g-2: was 2_500_000 at 10 ns


async def bring_up_to_polling_active(dut):
    """Reset -> Detect -> Polling.Active."""
    cocotb.start_soon(Clock(dut.clk_i, CLK_PERIOD_NS, units="ns").start())
    drive_idle_inputs(dut)
    dut.rst_i.value = 1
    await ClockCycles(dut.clk_i, 5)
    dut.rst_i.value = 0
    await ClockCycles(dut.clk_i, 5)

    dut.en_i.value = 1
    await wait_state(dut, ST_DETECT_QUIET, 50, "DETECT_QUIET")

    dut.phy_rxelecidle_i.value = ALL
    await ClockCycles(dut.clk_i, 3)
    dut.phy_rxelecidle_i.value = 0
    await wait_state(dut, ST_DETECT_ACTIVE, 50, "DETECT_ACTIVE")

    dut.receiver_detected_i.value = ALL
    dut.phy_rxstatus_i.value = RXSTATUS_ALL_OK
    dut.phy_phystatus_i.value = ALL
    await ClockCycles(dut.clk_i, 3)
    dut.phy_phystatus_i.value = 0
    await wait_state(dut, ST_POLLING_ACTIVE, 100, "POLLING_ACTIVE")


async def reach_config_substate(dut, target_state, target_name):
    """From POLLING_ACTIVE, walk the normal happy path (same sequence as
    bring_up_link) up through Configuration, with a *killable* TX pulser.
    Stops (kills) the pulser the instant target_state is reached, so
    ordered_set_tranmitted_i goes silent exactly upon entering the substate
    under test -- proving whether that substate's own timeout logic ever
    gets a chance to run without further TX activity."""
    await bring_up_to_polling_active(dut)
    pulser = cocotb.start_soon(os_tx_pulser(dut))

    def stop_pulsing():
        pulser.kill()
        dut.ordered_set_tranmitted_i.value = 0

    dut.ordered_set_i.value = pack_tsos_all_lanes(link_num=None, lane_num=None)
    dut.ts1_valid_i.value = ALL
    if target_state == ST_POLLING_CONFIG:
        await wait_state(dut, target_state, 2000, target_name)
        stop_pulsing()
        return
    await wait_state(dut, ST_POLLING_CONFIG, 2000, "POLLING_CONFIGURATION")

    dut.ts1_valid_i.value = 0
    dut.ts2_valid_i.value = ALL
    if target_state == ST_CFG_LW_START:
        await wait_state(dut, target_state, 2000, target_name)
        stop_pulsing()
        return
    await wait_state(dut, ST_CFG_LW_START, 2000, "CFG_LINKWIDTH_START")

    dut.ts2_valid_i.value = 0
    dut.ordered_set_i.value = pack_tsos_all_lanes(link_num=LINK_NUM, lane_num=None)
    dut.ts1_valid_i.value = ALL
    if target_state == ST_CFG_LW_ACCEPT:
        await wait_state(dut, target_state, 2000, target_name)
        stop_pulsing()
        return
    await wait_state(dut, ST_CFG_LW_ACCEPT, 1000, "CFG_LINKWIDTH_ACCEPT")

    if target_state == ST_CFG_LN_WAIT:
        await wait_state(dut, target_state, 1000, target_name)
        stop_pulsing()
        return
    await wait_state(dut, ST_CFG_LN_WAIT, 1000, "CFG_LANENUM_WAIT")

    dut.ordered_set_i.value = pack_tsos_all_lanes(link_num=LINK_NUM, lane_num="index")
    if target_state == ST_CFG_LN_ACCEPT:
        await wait_state(dut, target_state, 1000, target_name)
        stop_pulsing()
        return
    await wait_state(dut, ST_CFG_LN_ACCEPT, 1000, "CFG_LANENUM_ACCEPT")

    if target_state == ST_CFG_COMPLETE:
        await wait_state(dut, target_state, 1000, target_name)
        stop_pulsing()
        return
    await wait_state(dut, ST_CFG_COMPLETE, 1000, "CFG_COMPLETE")

    dut.ts1_valid_i.value = 0
    dut.ts2_valid_i.value = ALL
    if target_state == ST_CFG_IDLE:
        await wait_state(dut, target_state, 2000, target_name)
        dut.ts2_valid_i.value = 0
        stop_pulsing()
        return
    await wait_state(dut, ST_CFG_IDLE, 2000, "CFG_IDLE")
    dut.ts2_valid_i.value = 0
    stop_pulsing()


# ---- 5 substates: pre-fix these hung; post-fix they must reach ST_IDLE ----

@cocotb.test()
async def run_test_config_linkwidth_start_timeout(dut):
    await reach_config_substate(dut, ST_CFG_LW_START, "CFG_LINKWIDTH_START")
    # NOTE: uses LINKWIDTH_START_TIMEOUT_BUDGET (24ms + margin), not
    # CONFIG_TIMEOUT_BUDGET -- see the constant's comment above.
    await wait_state(dut, ST_IDLE, LINKWIDTH_START_TIMEOUT_BUDGET,
                      "ST_IDLE (Config.LinkwidthStart timeout, TX silent)")
    dut._log.info("CONFIG.LINKWIDTH_START TIMEOUT VERIFIED (not gated)")


@cocotb.test()
async def run_test_config_linkwidth_accept_timeout(dut):
    await reach_config_substate(dut, ST_CFG_LW_ACCEPT, "CFG_LINKWIDTH_ACCEPT")
    await wait_state(dut, ST_IDLE, CONFIG_TIMEOUT_BUDGET,
                      "ST_IDLE (Config.LinkwidthAccept timeout, TX silent)")
    dut._log.info("CONFIG.LINKWIDTH_ACCEPT TIMEOUT VERIFIED (not gated)")


@cocotb.test()
async def run_test_config_lanenum_wait_timeout(dut):
    await reach_config_substate(dut, ST_CFG_LN_WAIT, "CFG_LANENUM_WAIT")
    await wait_state(dut, ST_IDLE, CONFIG_TIMEOUT_BUDGET,
                      "ST_IDLE (Config.LanenumWait timeout, TX silent)")
    dut._log.info("CONFIG.LANENUM_WAIT TIMEOUT VERIFIED (not gated)")


@cocotb.test()
async def run_test_config_lanenum_accept_timeout(dut):
    await reach_config_substate(dut, ST_CFG_LN_ACCEPT, "CFG_LANENUM_ACCEPT")
    await wait_state(dut, ST_IDLE, CONFIG_TIMEOUT_BUDGET,
                      "ST_IDLE (Config.LanenumAccept timeout, TX silent)")
    dut._log.info("CONFIG.LANENUM_ACCEPT TIMEOUT VERIFIED (not gated)")


@cocotb.test()
async def run_test_config_complete_timeout(dut):
    await reach_config_substate(dut, ST_CFG_COMPLETE, "CFG_COMPLETE")
    await wait_state(dut, ST_IDLE, CONFIG_TIMEOUT_BUDGET,
                      "ST_IDLE (Config.Complete timeout, TX silent)")
    dut._log.info("CONFIG.COMPLETE TIMEOUT VERIFIED (not gated)")


# ---- control: ST_CONFIGURATION_IDLE, structurally different -- expected to PASS ----
#
# NOTE: ST_CONFIGURATION_IDLE's timeout does NOT go to ST_IDLE on the first
# hit -- since curr_data_rate_r.rate == gen1 throughout this test,
# idle_to_rlock_transitioned_c jumps straight to 8'hFF and
# next_state = ST_RECOVERY_RCVR_LOCK fires immediately (only a *second*
# timeout would reach ST_IDLE). Asserting for ST_IDLE here was the original
# (wrong) expectation. Fixed to assert the real immediate target.

@cocotb.test()
async def run_test_config_idle_timeout_control(dut):
    await reach_config_substate(dut, ST_CFG_IDLE, "CFG_IDLE")
    await wait_state(dut, ST_RECOVERY_RCVR_LOCK, CONFIG_TIMEOUT_BUDGET,
                      "ST_RECOVERY_RCVR_LOCK (Config.Idle timeout, TX silent) [CONTROL]")
    dut._log.info("CONFIG.IDLE TIMEOUT CONTROL VERIFIED (correctly not gated)")
