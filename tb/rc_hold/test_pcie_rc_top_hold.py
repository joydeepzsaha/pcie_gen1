"""test_pcie_rc_top_hold -- the hold in pcie_rc_top counts from DL_Active

Author: Kourosh Ghahramani
Silicon Systems Research Lab, University of Washington

Purpose
    Row e of §63 #22, on tb_pcie_rc_top_hold: pcie_rc_top over the PIPE
    loopback of test_pcie_rc_top, which trains to L0 and completes FC
    initialization on its own echo. Rows a-d drive the engine's reference
    from their bench; this row is the only one that checks which signal
    pcie_rc_top connects to it. Base 2.1 §6.6.1 is the rule and Base 3.0
    §6.7.3.3 names the event, DL_Active, which is fc_init_done_o.

    The row is red until the hold commit (C2), pinned at one assertion
    (§22.93).

References
    PCIe Base Spec r2.1, §6.6.1
    PCIe Base Spec r3.0, §6.7.3.3
"""

import cocotb
from cocotb.triggers import ClockCycles, ReadOnly, RisingEdge

from test_pcie_rc_top import TB, pipe_loopback, pipe_receiver_detect

# The loopback completes FC init near cycle 4,400 (test_pcie_rc_top row 1);
# the bound leaves room for the hold and the first request.
RUN_CYCLES = 12000


def pinned_red(dut, row, state, detail):
    """§22.93: the marker sweep43.sh copies into the gate's .diag."""
    dut._log.info("PINNED_RED|%s|%s|%s", row, state, detail)


class Timeline:
    """Cycle of the first rise of fc_init_done_o and scan_busy_o, and of the
    engine's first request handshake into u_tl (enum_rq_tvalid and
    enum_rq_tready). One sampler, read in ReadOnly after each edge."""

    def __init__(self, dut):
        self.dut = dut
        self.cycle = 0
        self.fc_rise = None
        self.busy_rise = None
        self.first_rq = None

    async def run(self, cycles):
        rc = self.dut.u_rc
        for _ in range(cycles):
            await RisingEdge(self.dut.clk_i)
            self.cycle += 1
            await ReadOnly()
            if self.fc_rise is None and int(rc.fc_init_done_o.value):
                self.fc_rise = self.cycle
            if self.busy_rise is None and int(rc.scan_busy_o.value):
                self.busy_rise = self.cycle
            if (self.first_rq is None and int(rc.enum_rq_tvalid.value)
                    and int(rc.enum_rq_tready.value)):
                self.first_rq = self.cycle
            if self.first_rq is not None:
                return


@cocotb.test(expect_fail=True)   # §63 #22 -- flips in C2 (the hold)
async def e_rc_top_hold_counts_from_dl_active(dut):
    """Row e. RED BEFORE FIX.

    scan_start_i is pulsed before link-up, so pcie_rc_top's start_pending_r
    holds it and the scan goes busy when fc_init_done_o rises. The engine's
    first request into the Transaction Layer must then wait the hold, counted
    from that rise.

    Today the request follows the rise by a few cycles. Pinned: the first
    request is no earlier than fc_init_done_o's rise + hold.
    """
    row = "e_rc_top_hold_counts_from_dl_active"
    try:
        hold = int(dut.bench_cfg_hold_cycles.value)
        tb = TB(dut)
        await tb.reset()
        cocotb.start_soon(pipe_loopback(dut))
        cocotb.start_soon(pipe_receiver_detect(dut))
        tl = Timeline(dut)
        sampler = cocotb.start_soon(tl.run(RUN_CYCLES))
        # The start, before the link is enabled: the start_pending_r path.
        await ClockCycles(dut.clk_i, 2)
        dut.scan_start_i.value = 1
        await ClockCycles(dut.clk_i, 1)
        dut.scan_start_i.value = 0
        await ClockCycles(dut.clk_i, 2)
        dut.en_i.value = 1
        dut.phy_ready_en.value = 1
        dut.transmit_enable_i.value = 1
        await sampler
        assert tl.fc_rise is not None, "fc_init_done_o never rose"
        assert tl.busy_rise is not None and tl.busy_rise - tl.fc_rise in (1, 2), (
            f"scan_busy_o rose at {tl.busy_rise}, fc_init_done_o at {tl.fc_rise}: "
            "the start gate, not the hold, would be what delays the request")
        assert tl.first_rq is not None, (
            f"no request reached u_tl in {RUN_CYCLES} cycles (fc at {tl.fc_rise})")
        first = tl.first_rq - tl.fc_rise
        dut._log.info("DIAG e: fc_init_done_o rose at %d, scan_busy_o at %d, "
                      "first request at fc%+d, hold %d",
                      tl.fc_rise, tl.busy_rise, first, hold)
    except Exception as exc:   # noqa: BLE001 -- §22.93
        pinned_red(dut, row, "NOT_REACHED", repr(exc))
        return
    pinned_red(dut, row, "REACHED", f"first=fc{first:+d} hold={hold}")
    assert first >= hold, (
        f"the engine's first request reached u_tl at fc_init_done_o{first:+d}, "
        f"inside the {hold}-cycle hold")
