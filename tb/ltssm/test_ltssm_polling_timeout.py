"""
Polling.Active 24ms timeout: if TS1/TS2 requirements are never satisfied,
the FSM must time out and retry from ST_IDLE rather than hang.
NOTE: TwentyFourMsTimeOut is NOT scaled by SIM_FAST_LINK -- this test
takes several real-world minutes. That is expected, not a hang.
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
async def run_test_polling_timeout(dut):
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

    # Deliberately send NOTHING (no TS1/TS2 ever) -- ordered_set_tranmitted_i
    # stays 0, so ordered_set_sent_cnt never advances and lanes_ts1/ts2
    # satisfied never latch. Only the raw 24ms wall timer can end this.
    # ~2.4M cycles -- expect several real-world minutes.
    await wait_state(dut, ST_IDLE, 25_000_000 // CLK_PERIOD_NS, "ST_IDLE (24ms Polling timeout)")
    dut._log.info("POLLING TIMEOUT VERIFIED: 24ms timeout correctly "
                  "returns to ST_IDLE instead of hanging in Polling.Active")
