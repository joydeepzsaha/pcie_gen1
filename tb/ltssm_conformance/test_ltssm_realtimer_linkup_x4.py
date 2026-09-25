"""
Phase 7A capstone -- end-to-end real-timer (SIM_FAST_LINK=0) link-up, x4.

Mirrors the validated x4 fast driver (ltssm_tb_common.bring_up_link) but with
budgets sized for the real MinTS1sPolling=1024 in Polling.Active. DUT default mode
(IS_ROOT_PORT=0); TB drives TS1/TS2/idle on all 4 lanes and assigns per-lane Lane
Numbers. Detect is driven past its 12 ms timer via an electrical-idle exit.

Prediction (committed before run): Polling.Active is gated by MinTS1sPolling=1024
(a transmitted-OS *count*, independent of lane count), os_tx_pulser every 4 cycles
=> ~4096 cycles ~= 41 us dominates, same as x1. Total time-to-L0 predicted ~45 us.
Assert 4,000 < cycles-to-L0 < 12,000 (40-120 us at the original 10 ns clock); log exact.
"""
import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles
from ltssm_tb_common import (
    drive_idle_inputs, wait_state, os_tx_pulser, pack_tsos_all_lanes,
    ST_IDLE, ST_DETECT_QUIET, ST_DETECT_ACTIVE, ST_POLLING_ACTIVE, ST_POLLING_CONFIG,
    ST_CFG_LW_START, ST_CFG_LW_ACCEPT, ST_CFG_LN_WAIT, ST_CFG_LN_ACCEPT,
    ST_CFG_COMPLETE, ST_CFG_IDLE, ST_L0, ALL, RXSTATUS_ALL_OK, LINK_NUM,
)

# sec 63 #7g-2 (D-7G.2): this bench's clock AND the RTL's CLK_PERIOD_NS, one value.
# The core passes -GCLK_PERIOD_NS=8 (tb_ltssm_b2b.sv passes .CLK_PERIOD_NS(8));
# every cycle budget below that stands for a spec time derives from it.
CLK_PERIOD_NS = 8

BIG = 30_000


@cocotb.test()
async def test_realtimer_linkup_x4(dut):
    cocotb.start_soon(Clock(dut.clk_i, CLK_PERIOD_NS, units="ns").start())

    nb = len(dut.ordered_set_i)
    assert nb == 512, f"expected x4 (512b ordered_set_i), got {nb}"

    drive_idle_inputs(dut)
    dut.rst_i.value = 1
    await ClockCycles(dut.clk_i, 5)
    dut.rst_i.value = 0
    await ClockCycles(dut.clk_i, 5)
    assert int(dut.ltssm_state_o.value) == ST_IDLE

    t0 = cocotb.utils.get_sim_time(units="ns")

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
    cocotb.start_soon(os_tx_pulser(dut))
    await wait_state(dut, ST_POLLING_ACTIVE, 100, "POLLING_ACTIVE")

    tp = cocotb.utils.get_sim_time(units="ns")
    dut.ordered_set_i.value = pack_tsos_all_lanes(link_num=None, lane_num=None)
    dut.ts1_valid_i.value = ALL
    await wait_state(dut, ST_POLLING_CONFIG, BIG, "POLLING_CONFIGURATION")
    dut._log.warning(f"[real x4] Polling.Active dwell = "
                     f"{(cocotb.utils.get_sim_time(units='ns') - tp)/1000:.2f} us")

    dut.ts1_valid_i.value = 0
    dut.ts2_valid_i.value = ALL
    await wait_state(dut, ST_CFG_LW_START, BIG, "CFG_LINKWIDTH_START")

    dut.ts2_valid_i.value = 0
    dut.ordered_set_i.value = pack_tsos_all_lanes(link_num=LINK_NUM, lane_num=None)
    dut.ts1_valid_i.value = ALL
    await wait_state(dut, ST_CFG_LW_ACCEPT, BIG, "CFG_LINKWIDTH_ACCEPT")
    await wait_state(dut, ST_CFG_LN_WAIT, BIG, "CFG_LANENUM_WAIT")

    dut.ordered_set_i.value = pack_tsos_all_lanes(link_num=LINK_NUM, lane_num="index")
    await wait_state(dut, ST_CFG_LN_ACCEPT, BIG, "CFG_LANENUM_ACCEPT")
    await wait_state(dut, ST_CFG_COMPLETE, BIG, "CFG_COMPLETE")

    dut.ts1_valid_i.value = 0
    dut.ts2_valid_i.value = ALL
    await wait_state(dut, ST_CFG_IDLE, BIG, "CFG_IDLE")
    dut.ts2_valid_i.value = 0
    dut.idle_valid_i.value = ALL
    await wait_state(dut, ST_L0, BIG, "L0")
    dut.idle_valid_i.value = 0

    dt_us = (cocotb.utils.get_sim_time(units="ns") - t0) / 1000.0
    assert int(dut.link_up_o.value) == 1, "link_up_o not asserted in L0"
    dut._log.warning(f"[real x4] time-to-L0 = {dt_us:.2f} us (predicted ~45 us)")
    # sec 63 #7g-2: the window is a CYCLE claim (~1024 TS1 x 4 cycles ~= 4,096
    # cycles dominates) that was written in us at the old 10 ns clock
    # (40-120 us).  At 8 ns the same cycles take 34.9 us and the us window
    # failed a correct link.  Stated in cycles, the quantity predicted.
    dt_cyc = dt_us * 1000.0 / CLK_PERIOD_NS
    assert 4_000 < dt_cyc < 12_000, (
        f"real x4 time-to-L0 = {dt_cyc:.0f} cycles ({dt_us:.2f} us) outside "
        f"predicted 4,000-12,000 cycle window")
    dut._log.info("[real x4] REAL-TIMER LINK UP OK")
