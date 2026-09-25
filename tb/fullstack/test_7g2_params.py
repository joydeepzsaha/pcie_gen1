"""§63 #7g-2 Phase 1, C-7G2-10 -- the SHIPPED-DEFAULTS elaboration. NOT A GATE ROW.

Target `verilate_fullstack_7g2_shipped` rebuilds tb_pcie_fullstack with
SIM_FAST_LINK=0 and the RC's CPL_TIMEOUT_CYCLES=6250 -- the values the product
tops ship with -- plus probe_7g2.sv. The measurement is the build's own
Vtop___024root.h (every elaborated parameter of every instance) and the probe's
PARAM lines. This test only runs 20 clock cycles so the probe's initial blocks
execute; it asserts nothing about the link, which would take ~12 ms of real
Detect.Quiet to train at these values.
"""
import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge


@cocotb.test()
async def g7g2_shipped_params(dut):
    cocotb.start_soon(Clock(dut.clk_i, 8, units="ns").start())
    dut.rst_i.value = 1
    for _ in range(20):
        await RisingEdge(dut.clk_i)
