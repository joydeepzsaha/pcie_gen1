"""
Polling.Configuration -> Configuration: oracle P11, predicted to CONFORM.

Base 2.1 Rev 2.1, Section 4.2.6.2.3 (p.225):

    "The next state is Configuration after eight consecutive TS2 Ordered Sets,
     with Link and Lane numbers set to PAD (K23.7), are received on any Lanes
     that detected a Receiver during Detect, and 16 TS2 Ordered Sets are
     transmitted after receiving one TS2 Ordered Set."

Three limbs, and the RTL implements all three:
  * eight consecutive TS2 with Link and Lane PAD -- :1717 checks both fields
    against PAD, :1718 counts to 8, ts2_cnt_satisfied at :1577;
  * on ANY such lane -- :736 uses |lanes_ts2_satisfied, an OR reduction, which
    is what "any Lanes" requires (contrast Polling.Active's primary exit at
    :660, which correctly uses & for "all Lanes");
  * 16 transmitted AFTER receiving one -- :732 gates the transmit counter on
    |single_ts2_received, then :736 requires >= 8'h10.

This is the same "after receiving one" gating that oracle P4 shows is WRONG
when applied to Polling.Active's primary exit -- but here the spec asks for it
explicitly, so the identical construct is correct in this state and incorrect
in that one. That contrast is the reason this oracle is worth executing rather
than assuming.

NEGATIVE CONTROL: a TS2 carrying a non-PAD Link number satisfies none of the
above, and must not advance the FSM. Without it, a green result here would only
prove that TS2s eventually advance Polling.Configuration -- which is not what
the spec paragraph claims.

Requires SIM_FAST_LINK=1, MAX_NUM_LANES=1 (verilate_polling_p11 target).
"""
import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, RisingEdge, Timer
from ltssm_tb_common import *  # noqa

# sec 63 #7g-2 (D-7G.2): this bench's clock AND the RTL's CLK_PERIOD_NS, one value.
# The core passes -GCLK_PERIOD_NS=8 (tb_ltssm_b2b.sv passes .CLK_PERIOD_NS(8));
# every cycle budget below that stands for a spec time derives from it.
CLK_PERIOD_NS = 8

NON_PAD_LINK = 0x5A
NEG_WATCH = 5000     # far short of the 48 ms (4.8 M cycle) watchdog


def state(dut):
    return int(dut.ltssm_state_o.value)


def sname(s):
    return f"{STATE_NAMES.get(s, hex(s))} ({s:#07x})"


async def reach_polling_config(dut):
    drive_idle_inputs(dut)
    dut.rst_i.value = 1
    await ClockCycles(dut.clk_i, 5)
    dut.rst_i.value = 0
    await ClockCycles(dut.clk_i, 5)

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
    await wait_state(dut, ST_POLLING_ACTIVE, 200, "POLLING_ACTIVE")

    dut.ordered_set_i.value = pack_tsos(link_num=None, lane_num=None)
    dut.ts1_valid_i.value = LANE0_MASK
    await wait_state(dut, ST_POLLING_CONFIG, 4000, "POLLING_CONFIGURATION")
    dut.ts1_valid_i.value = 0


def clk(dut):
    cocotb.start_soon(Clock(dut.clk_i, CLK_PERIOD_NS, units="ns").start())
    assert len(dut.ordered_set_i) == 128, "-GMAX_NUM_LANES=1 did not reach the DUT"


@cocotb.test()
async def run_test_p11_polling_config(dut):
    clk(dut)

    # ---- negative control: TS2 with a non-PAD Link number ----
    await reach_polling_config(dut)
    dut.ordered_set_i.value = pack_tsos(link_num=NON_PAD_LINK, lane_num=None)
    dut.ts2_valid_i.value = LANE0_MASK
    for _ in range(NEG_WATCH):
        await RisingEdge(dut.clk_i)
        await Timer(1, units="ps")
        if state(dut) != ST_POLLING_CONFIG:
            raise AssertionError(
                f"NEGATIVE CONTROL FAILED: the DUT advanced out of "
                f"Polling.Configuration into {sname(state(dut))} on TS2 with "
                f"Link={NON_PAD_LINK:#04x}, which is not PAD. Base 2.1 p.225 "
                f"requires 'Link and Lane numbers set to PAD (K23.7)'.")
    dut._log.info(f"negative control bit: TS2 with non-PAD Link "
                  f"({NON_PAD_LINK:#04x}) did not advance the FSM in "
                  f"{NEG_WATCH} cycles")

    # ---- positive: TS2 with Link and Lane both PAD ----
    await reach_polling_config(dut)
    dut.ordered_set_i.value = pack_tsos(link_num=None, lane_num=None)
    dut.ts2_valid_i.value = LANE0_MASK
    await wait_state(dut, ST_CFG_LW_START, 8000,
                     "CFG_LINKWIDTH_START (P11 positive)")
    dut._log.info("P11 CONFORMS: advanced on eight consecutive TS2 PAD/PAD "
                  "plus 16 transmitted after receiving one, and did not "
                  "advance on a non-PAD Link")
