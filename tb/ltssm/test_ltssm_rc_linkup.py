"""
Root-Complex-mode link-up test for pcie_ltssm_downstream (IS_ROOT_PORT=1).

The DUT plays the Root Complex / downstream-facing port: it *originates*
LINK_NUM and assigns Lane Number 0 (x1). This testbench plays its Endpoint
link partner -- it samples what the DUT actually transmits on ordered_set_o,
asserts each Configuration ordered set matches the spec for the state the DUT
is in, then echoes it back on ordered_set_i. Nothing about the transmitted
sequence is scripted; a DUT emitting the wrong link/lane/TS-type fails an
assertion rather than being rubber-stamped.

This is the first test that exercises the IS_ROOT_PORT=1 RC behavior added in
commits 5974dc6 and cd65885. Requires SIM_FAST_LINK=1, MAX_NUM_LANES=1,
IS_ROOT_PORT=1, LINK_NUM=1 (verilate_rc_linkup target).
"""
import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles
from ltssm_tb_common import *  # noqa

# sec 63 #7g-2 (D-7G.2): this bench's clock AND the RTL's CLK_PERIOD_NS, one value.
# The core passes -GCLK_PERIOD_NS=8 (tb_ltssm_b2b.sv passes .CLK_PERIOD_NS(8));
# every cycle budget below that stands for a spec time derives from it.
CLK_PERIOD_NS = 8


@cocotb.test()
async def run_test_rc_linkup(dut):
    cocotb.start_soon(Clock(dut.clk_i, CLK_PERIOD_NS, units="ns").start())  # 125 MHz

    # ---- confirm the -G parameter overrides reached the DUT ----
    # ordered_set_i is pcie_tsos_t[MAX_NUM_LANES-1:0]; one lane = 128 bits.
    # If -GMAX_NUM_LANES=1 didn't take, this would be 512 (the x4 default).
    n_bits = len(dut.ordered_set_i)
    assert n_bits == 128, (
        f"-GMAX_NUM_LANES=1 did not reach the DUT: ordered_set_i is {n_bits} "
        f"bits (x1 expects 128; x4 default is 512)")
    dut._log.info(f"param-reach: ordered_set_i={n_bits} bits confirms x1. "
                  "IS_ROOT_PORT=1/LINK_NUM=1 are confirmed by the RC "
                  "link-number assertions below (EP default transmits PAD "
                  "and would fail them).")

    # reset -> enable -> full Detect/Polling/Configuration walk as RC.
    await bring_up_link_rc(dut)

    reached = int(dut.ltssm_state_o.value)
    sim_ns = cocotb.utils.get_sim_time(units="ns")
    dut._log.info(
        f"RC reached {STATE_NAMES.get(reached, hex(reached))} "
        f"({reached:#07x}) at {sim_ns:.0f} ns")

    assert reached == ST_L0, "RC did not reach L0"
    assert int(dut.link_up_o.value) == 1, "link_up_o not asserted in L0"

    # ---- stay in L0: stop training traffic, link must hold ----
    await ClockCycles(dut.clk_i, 50)
    assert int(dut.ltssm_state_o.value) == ST_L0, "RC fell out of L0"
    assert int(dut.link_up_o.value) == 1, "link_up_o dropped in L0"
    dut._log.info(
        "RC LINK UP: DUT originated LINK_NUM + lane 0, "
        "reactive EP echo walked Detect->Polling->Config->L0")
