"""
Polling.Active 24ms timeout with a HEALTHY TX handshake: partner never sends
TS1/TS2, but our own ordered_set_tranmitted_i keeps pulsing normally. This is
the path that already worked before the Bug 4 fix (the watchdog check used to
live inside the ordered_set_tranmitted_i gate) -- it must keep passing
after the fix, proving the fix didn't disturb the previously-working case.
NOTE: TwentyFourMsTimeOut is NOT scaled by SIM_FAST_LINK -- this test
takes several real-world minutes. That is expected, not a hang.

FRAGILITY (mutant MP4b's oracle below): its kill depends on os_tx_pulser's
period, TwentyFourMsTimeOut and the clock period staying as they are -- the
24 ms branch sits inside the ordered_set_tranmitted_i gate while the generic
watchdog does not, so error_c has a ONE-CYCLE window and changing any of the
three makes this row silently vacuous while it keeps passing.
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
async def run_test_polling_timeout_tx_ok(dut):
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

    # Partner is silent (no TS1/TS2 ever), but our own TX handshake is healthy.
    # ordered_set_sent_cnt never advances (it's gated on single_ts1_received),
    # so no success path can fire -- only the 24ms watchdog can end this.
    cocotb.start_soon(os_tx_pulser(dut))

    await wait_state(dut, ST_IDLE, 25_000_000 // CLK_PERIOD_NS, "ST_IDLE (24ms timeout, TX healthy)")

    # Owed row for mutant MP4b (tracker sec 54 #8; evidence/fix-arc-6/
    # MUTANTS_BATCH_A.md sec 3c).  MP4b applies the REJECTED one-line form of the
    # P4 fix -- it deletes the `if (|single_ts1_received)` gate around
    # ordered_set_sent_cnt_c in ST_POLLING_ACTIVE.  With that gate gone the
    # counter advances on every TX pulse even though the partner never sent a
    # TS1, so the 24 ms branch becomes satisfiable on a link that never
    # responded; neither lanes_ts1_satisfied nor lanes_ts2_satisfied is set, and
    # control reaches the branch's else arm, which raises error_c.
    #
    # error_o is STICKY (assign error_o = error_r), so reading it once here
    # covers the whole run.  It is the only observable: MP4b does not change the
    # state trajectory -- the generic 24 ms watchdog sends us to ST_IDLE either
    # way -- and it does not change the sim end time.
    assert int(dut.error_o.value) == 0, (
        "the 24 ms Polling.Active watchdog must not raise error_c; a 1 here means "
        "ordered_set_sent_cnt_r advanced without a received TS1 and the 24 ms "
        "branch took its else arm -- the coupling MP4b injects")

    dut._log.info("POLLING TIMEOUT (TX healthy) VERIFIED")
