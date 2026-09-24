"""
Recovery test: L0 -> Recovery.RcvrLock -> Recovery.RcvrCfg -> Recovery.Idle -> L0.
Scenario: partner-initiated retrain, NO speed change (rate stays gen1,
speed_change bit = 0 throughout). Requires SIM_FAST_LINK=1.
"""
import cocotb
from cocotb.triggers import RisingEdge, ClockCycles
from cocotb.clock import Clock
from ltssm_tb_common import *   # noqa

# sec 63 #7g-2 (D-7G.2): this bench's clock AND the RTL's CLK_PERIOD_NS, one value.
# The core passes -GCLK_PERIOD_NS=8 (tb_ltssm_b2b.sv passes .CLK_PERIOD_NS(8));
# every cycle budget below that stands for a spec time derives from it.
CLK_PERIOD_NS = 8

LINK_NUM = 0x01

@cocotb.test()
async def run_test_recovery_no_speed_change(dut):
    cocotb.start_soon(Clock(dut.clk_i, CLK_PERIOD_NS, units="ns").start())
    await bring_up_link(dut)
    assert int(dut.ltssm_state_o.value) == ST_L0
    assert int(dut.link_up_o.value) == 1
    dut._log.info("setup complete: in L0")

    # ---- L0 -> RECOVERY_RCVR_LOCK: partner starts sending TS1s ----
    # ST_L0 exits on (|ts1_valid_i || |ts2_valid_i || directed_speed_change_i).
    # Use TS1s (not directed_speed_change_i) to stay on the no-speed-change path.
    dut.ordered_set_i.value = pack_tsos_all_lanes(
        link_num=LINK_NUM, lane_num="index", rate=GEN1_RATE, speed_change=0)
    dut.ts1_valid_i.value = ALL
    await wait_state(dut, ST_RECOVERY_RCVR_LOCK, 100, "RECOVERY_RCVR_LOCK")

    # ---- RCVR_LOCK -> RCVR_CFG: need ts1_cnt==8 on every active lane,
    #      extended_synch_i=0, and speed_change bit clear.
    await wait_state(dut, ST_RECOVERY_RCVR_CFG, 500, "RECOVERY_RCVR_CFG")

    # ---- RCVR_CFG -> RECOVERY_IDLE: partner switches to TS2s.
    #      Exit needs |(ts2_cnt_satisfied & lane_active_r), speed_change_bit_set==0,
    #      and ordered_set_sent_cnt >= 16. Keep rate_id + ts_s6 CONSTANT --
    #      the RTL resets ts2_cnt if rate_id/ts_s6 change between TS2s.
    dut.ts1_valid_i.value = 0
    dut.ts2_valid_i.value = ALL
    await wait_state(dut, ST_RECOVERY_IDLE, 2000, "RECOVERY_IDLE")

    # ---- RECOVERY_IDLE -> L0: partner sends idles only.
    #      NOTE: expected to be the fragile step -- see at_least_one_ts1_ts2 race.
    dut.ts2_valid_i.value = 0
    dut.idle_valid_i.value = ALL
    await wait_state(dut, ST_L0, 2000, "L0 (via Recovery)")

    # ---- confirm link stays up ----
    dut.idle_valid_i.value = 0
    await ClockCycles(dut.clk_i, 50)
    assert int(dut.ltssm_state_o.value) == ST_L0, "fell out of L0 after Recovery"
    assert int(dut.link_up_o.value) == 1, "link_up dropped after Recovery"
    dut._log.info("RECOVERY VERIFIED: L0 -> Recovery -> L0, no speed change")
