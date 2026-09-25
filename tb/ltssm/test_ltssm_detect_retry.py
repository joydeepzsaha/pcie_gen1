"""
Detect no-receiver-found retry: DETECT_ACTIVE with receiver_detected_i=0
should loop back to Detect.Quiet (via ST_IDLE) rather than proceeding to
Polling, then succeed normally on a second attempt.
"""
import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ClockCycles
from ltssm_tb_common import *  # noqa

# sec 63 #7g-2 (D-7G.2): this bench's clock AND the RTL's CLK_PERIOD_NS, one value.
# The core passes -GCLK_PERIOD_NS=8 (tb_ltssm_b2b.sv passes .CLK_PERIOD_NS(8));
# every cycle budget below that stands for a spec time derives from it.
CLK_PERIOD_NS = 8

@cocotb.test()
async def run_test_detect_no_receiver_then_retry(dut):
    cocotb.start_soon(Clock(dut.clk_i, CLK_PERIOD_NS, units="ns").start())
    drive_idle_inputs(dut)
    dut.rst_i.value = 1
    await ClockCycles(dut.clk_i, 5)
    dut.rst_i.value = 0
    await ClockCycles(dut.clk_i, 5)

    dut.en_i.value = 1
    await wait_state(dut, ST_DETECT_QUIET, 50, "DETECT_QUIET")

    # exit elec-idle to reach DETECT_ACTIVE
    dut.phy_rxelecidle_i.value = ALL
    await ClockCycles(dut.clk_i, 3)
    dut.phy_rxelecidle_i.value = 0
    await wait_state(dut, ST_DETECT_ACTIVE, 50, "DETECT_ACTIVE")

    # ---- FIRST ATTEMPT: pulse phystatus with NO receiver detected ----
    dut.receiver_detected_i.value = 0
    dut.phy_phystatus_i.value = ALL
    await ClockCycles(dut.clk_i, 3)
    dut.phy_phystatus_i.value = 0

    # ST_IDLE is a single-cycle waypoint -- don't try to catch it directly,
    # just confirm the retry landed back in Detect.Quiet (the real proof
    # the no-receiver path looped back rather than proceeding to Polling).
    await wait_state(dut, ST_DETECT_QUIET, 20, "DETECT_QUIET (retry after no-receiver)")

    # negative check: confirm it did NOT proceed to Polling on this pulse
    assert int(dut.ltssm_state_o.value) != ST_POLLING_ACTIVE, \
        "FSM proceeded to Polling despite receiver_detected_i==0"

    # ---- SECOND ATTEMPT: real receiver present this time ----
    dut.phy_rxelecidle_i.value = ALL
    await ClockCycles(dut.clk_i, 3)
    dut.phy_rxelecidle_i.value = 0
    await wait_state(dut, ST_DETECT_ACTIVE, 50, "DETECT_ACTIVE (2nd)")

    dut.receiver_detected_i.value = ALL
    dut.phy_rxstatus_i.value = RXSTATUS_ALL_OK
    dut.phy_phystatus_i.value = ALL
    await ClockCycles(dut.clk_i, 3)
    dut.phy_phystatus_i.value = 0

    await wait_state(dut, ST_POLLING_ACTIVE, 100, "POLLING_ACTIVE (recovered)")
    dut._log.info("DETECT RETRY VERIFIED: no-receiver correctly retries, "
                  "then proceeds normally once a receiver is present")
