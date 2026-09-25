"""
Phase 7A -- isolated real-timer (SIM_FAST_LINK=0) timeout test.

Only three constants change at real magnitude vs fast-sim (see TIMER_CONSTANTS.md):
TwelveMsTimeOut and OneMsTimeOut (/1000) and MinTS1sPolling (/~42.7). The 24/48/2
ms watchdogs are UNSCALED -- already exercised at real magnitude by the existing
verilate_polling_timeout / verilate_config_timeout tests -- so re-testing them
here would duplicate. OneMsTimeOut's state (Detect.Wait.One.Ms) is unreachable in
a Gen1 link. That leaves TwelveMsTimeOut as the one watchdog never run at real
magnitude: Detect.Quiet's 12 ms "proceed anyway" timer (the happy path always
exits Detect.Quiet on an electrical-idle exit long before 12 ms).

Prediction (falsifiability, committed before the run):
  ClockPeriodNs = 1000/CLK_RATE = 10 ns; TwelveMsTimeOut(real) = 12e6/10 =
  1,200,000 cycles. timer_r resets to 0 on entry to Detect.Quiet and increments
  +1/cycle; the exit fires when timer_r >= 1,200,000. Measured from first observing
  Detect.Quiet to first observing Detect.Active, the delta must be 1,200,000 cycles
  (+1-2 for the registered state update / sampling). Assert within [1,200,000,
  1,200,006]. An overflowing / wrapping / early-firing counter would miss this.
"""
import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ClockCycles
from ltssm_tb_common import (
    drive_idle_inputs, STATE_NAMES, ST_DETECT_QUIET, ST_DETECT_ACTIVE,
)

# sec 63 #7g-2 (D-7G.2): bench clock AND the RTL's CLK_PERIOD_NS (-GCLK_PERIOD_NS=8 in the core).
CLK_PERIOD_NS = 8
TWELVE_MS_CYCLES = 12_000_000 // CLK_PERIOD_NS   # 1,500,000 -- spec 12 ms @125 MHz


@cocotb.test()
async def test_detect_quiet_12ms_real(dut):
    cocotb.start_soon(Clock(dut.clk_i, CLK_PERIOD_NS, units="ns").start())

    # param-reach: SIM_FAST_LINK=0 confirmed indirectly by the fire cycle itself
    # (fast-sim would fire at 1,200 cycles, not 1,200,000).
    drive_idle_inputs(dut)
    dut.rst_i.value = 1
    await ClockCycles(dut.clk_i, 5)
    dut.rst_i.value = 0
    await ClockCycles(dut.clk_i, 5)

    # IDLE -> Detect.Quiet (Gen1 => skips Detect.Wait.One.Ms)
    dut.en_i.value = 1
    for _ in range(50):
        await RisingEdge(dut.clk_i)
        if int(dut.ltssm_state_o.value) == ST_DETECT_QUIET:
            break
    assert int(dut.ltssm_state_o.value) == ST_DETECT_QUIET, "never reached Detect.Quiet"

    # CRITICAL: hold phy_rxelecidle_i CONSTANT (0) so there is no 1->0 exit edge;
    # the only way out of Detect.Quiet is now the 12 ms timer.
    dut.phy_rxelecidle_i.value = 0
    t_entry = cocotb.utils.get_sim_time(units="ns")

    # jump to just before the predicted fire, then poll the exact cycle.
    await ClockCycles(dut.clk_i, TWELVE_MS_CYCLES - 100)
    assert int(dut.ltssm_state_o.value) == ST_DETECT_QUIET, (
        "Detect.Quiet exited EARLY (before ~12 ms) -- counter fired early / wrapped")

    fired = False
    for _ in range(400):
        await RisingEdge(dut.clk_i)
        if int(dut.ltssm_state_o.value) == ST_DETECT_ACTIVE:
            fired = True
            break
    t_fire = cocotb.utils.get_sim_time(units="ns")
    st = int(dut.ltssm_state_o.value)
    assert fired, (f"Detect.Quiet did NOT time out within 12 ms + 400 cycles "
                   f"(stuck in {STATE_NAMES.get(st, hex(st))}) -- counter never "
                   f"reached the real 1,200,000 threshold (overflow/width bug?)")

    delta_cycles = round((t_fire - t_entry) / CLK_PERIOD_NS)
    dut._log.warning(
        f"[Detect.Quiet 12ms] predicted fire = {TWELVE_MS_CYCLES} cycles; "
        f"observed = {delta_cycles} cycles ({(t_fire - t_entry)/1e6:.3f} ms sim)")
    assert TWELVE_MS_CYCLES <= delta_cycles <= TWELVE_MS_CYCLES + 6, (
        f"Detect.Quiet 12 ms timer fired at {delta_cycles} cycles, expected "
        f"{TWELVE_MS_CYCLES} (+<=6). Off by {delta_cycles - TWELVE_MS_CYCLES}.")
    dut._log.info("[Detect.Quiet 12ms] OK: real 1,200,000-cycle timer fired on time")
