"""
Phase 7A capstone -- end-to-end real-timer (SIM_FAST_LINK=0) link-up, x1, RC mode.

Confirms the DUT reaches L0 with real constants, not just isolated watchdogs. The
only real-magnitude cost on the happy path is MinTS1sPolling=1024 in Polling.Active
(Detect is driven past its 12 ms timer via an electrical-idle exit; the 24/48/2 ms
watchdogs are never hit on the happy path). Reactive RC walk (DUT originates
LINK_NUM + lane 0; TB plays its Endpoint, samples and echoes) with budgets sized
for the real 1024-TS1 Polling.Active.

Prediction (committed before run): os_tx_pulser pulses every 4 cycles; Polling.Active
needs ~1024 pulses => ~4096 cycles ~= 41 us, which dominates. Detect+Polling.Config+
Config add ~a few hundred cycles. Total time-to-L0 predicted ~45 us (vs ~6 us in
fast-sim where MinTS1sPolling=24). Assert 4,000 < cycles-to-L0 < 12,000 (40-120 us at the
original 10 ns clock; sec 63 #7g-2 moved the bench to 8 ns) and log the exact value.
"""
import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ClockCycles
from ltssm_tb_common import (
    drive_idle_inputs, wait_state, os_tx_pulser, pack_tsos, STATE_NAMES, LINK_NUM,
    ST_IDLE, ST_DETECT_QUIET, ST_DETECT_ACTIVE, ST_POLLING_ACTIVE, ST_POLLING_CONFIG,
    ST_CFG_LW_START, ST_CFG_LW_ACCEPT, ST_CFG_LN_WAIT, ST_CFG_LN_ACCEPT,
    ST_CFG_COMPLETE, ST_CFG_IDLE, ST_L0, LANE0_MASK, RXSTATUS_OK_X1,
)

# sec 63 #7g-2 (D-7G.2): this bench's clock AND the RTL's CLK_PERIOD_NS, one value.
# The core passes -GCLK_PERIOD_NS=8 (tb_ltssm_b2b.sv passes .CLK_PERIOD_NS(8));
# every cycle budget below that stands for a spec time derives from it.
CLK_PERIOD_NS = 8

PAD_LANE = None
BIG = 30_000   # cycle budget covering real MinTS1sPolling=1024 in Polling.Active


@cocotb.test()
async def test_realtimer_linkup_x1(dut):
    cocotb.start_soon(Clock(dut.clk_i, CLK_PERIOD_NS, units="ns").start())

    nb = len(dut.ordered_set_i)
    assert nb == 128, f"expected x1 (128b ordered_set_i), got {nb}"

    drive_idle_inputs(dut)
    dut.rst_i.value = 1
    await ClockCycles(dut.clk_i, 5)
    dut.rst_i.value = 0
    await ClockCycles(dut.clk_i, 5)
    assert int(dut.ltssm_state_o.value) == ST_IDLE

    t0 = cocotb.utils.get_sim_time(units="ns")

    # Detect (drive elec-idle exit so we skip the 12 ms timer)
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
    await wait_state(dut, ST_POLLING_ACTIVE, 100, "POLLING_ACTIVE")

    # Polling.Active: real MinTS1sPolling=1024 -> this is the long stretch.
    tp = cocotb.utils.get_sim_time(units="ns")
    dut.ordered_set_i.value = pack_tsos(link_num=None, lane_num=None)
    dut.ts1_valid_i.value = LANE0_MASK
    await wait_state(dut, ST_POLLING_CONFIG, BIG, "POLLING_CONFIGURATION")
    tpc = cocotb.utils.get_sim_time(units="ns")
    dut._log.warning(f"[real] Polling.Active dwell = {(tpc - tp)/1000:.2f} us "
                     f"(real MinTS1sPolling=1024)")

    # Polling.Config -> Config.Linkwidth.Start
    dut.ts1_valid_i.value = 0
    dut.ts2_valid_i.value = LANE0_MASK
    await wait_state(dut, ST_CFG_LW_START, BIG, "CFG_LINKWIDTH_START")

    # Config reactive echo (link=LINK, lane PAD then 0), same sequence as x1 RC linkup
    dut.ts2_valid_i.value = 0
    dut.ordered_set_i.value = pack_tsos(link_num=LINK_NUM, lane_num=PAD_LANE)
    dut.ts1_valid_i.value = LANE0_MASK
    await wait_state(dut, ST_CFG_LW_ACCEPT, BIG, "CFG_LINKWIDTH_ACCEPT")
    await wait_state(dut, ST_CFG_LN_WAIT, BIG, "CFG_LANENUM_WAIT")

    dut.ordered_set_i.value = pack_tsos(link_num=LINK_NUM, lane_num=0)   # lane changes
    await wait_state(dut, ST_CFG_LN_ACCEPT, BIG, "CFG_LANENUM_ACCEPT")
    await wait_state(dut, ST_CFG_COMPLETE, BIG, "CFG_COMPLETE")

    # Complete -> Idle (TS2), Idle -> L0 (idles)
    dut.ts1_valid_i.value = 0
    dut.ts2_valid_i.value = LANE0_MASK
    await wait_state(dut, ST_CFG_IDLE, BIG, "CFG_IDLE")
    dut.ts2_valid_i.value = 0
    dut.ordered_set_i.value = 0
    dut.idle_valid_i.value = LANE0_MASK
    await wait_state(dut, ST_L0, BIG, "L0")
    dut.idle_valid_i.value = 0

    t_l0 = cocotb.utils.get_sim_time(units="ns")
    dt_us = (t_l0 - t0) / 1000.0
    assert int(dut.link_up_o.value) == 1, "link_up_o not asserted in L0"
    dut._log.warning(f"[real] x1 time-to-L0 = {dt_us:.2f} us "
                     f"(predicted ~45 us, dominated by Polling.Active 1024 TS1)")
    # sec 63 #7g-2: the window is a CYCLE claim (~1024 TS1 x 4 cycles ~= 4,096
    # cycles dominates) that was written in us at the old 10 ns clock
    # (40-120 us).  At 8 ns the same cycles take 34.9 us and the us window
    # failed a correct link.  Stated in cycles, the quantity predicted.
    dt_cyc = dt_us * 1000.0 / CLK_PERIOD_NS
    assert 4_000 < dt_cyc < 12_000, (
        f"real x1 time-to-L0 = {dt_cyc:.0f} cycles ({dt_us:.2f} us) outside "
        f"predicted 4,000-12,000 cycle window")
    dut._log.info("[real] x1 REAL-TIMER LINK UP OK")
