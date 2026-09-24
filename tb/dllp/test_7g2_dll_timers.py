"""§63 #7g-2 Phase 1, C-7G2-6 and C-7G2-11 -- DLL timers that need a SILENT far
end. NOT A GATE ROW.

Target `measure_7g2_dll` runs this module against pcie_datalink_layer, the same
toplevel verilate_dll_comprehensive elaborates, with the far end in Python. Two
timers cannot be made to fire at the full stack, because a real peer always
answers: the REPLAY_TIMER (needs an Ack that never comes) and the InitFC1
originate interval (needs an FC_INIT1 peer that never speaks).

Predictions were committed before this ran (pcie_docs PREDICTIONS_7G2.md,
aaf21ea). Nothing is scored here; each test writes raw events to a JSON file
and the analysis is offline, behind a known-answer self-test (§22.92).

SAMPLING. The monitor awaits RisingEdge and then reads bare values, which in
this simulator are the PRE-edge values -- the values the flops sample at that
edge (§22.89). Every event therefore carries the index of the edge at which it
took effect, and every signal is sampled the same way, so differences between
event indices are exact edge counts.
"""
import json

import cocotb
from cocotb.triggers import RisingEdge

from test_dll_comprehensive import (
    TB,
    build_memory_write,
    link_up_silent,
    send_flow_control_initialization,
    send_frame_with_timeout,
)


class RawMon:
    """Per-edge raw event capture: m_phy_axis packets, retry_management strobes,
    and pcie_flow_ctrl_init's DL_Init input."""

    def __init__(self, dut):
        self.dut = dut
        self.rm = dut.dllp_transmit_inst.retry_management_inst
        self.fci = dut.pcie_flow_ctrl_init_inst
        self.ev = []
        self.stop = False

    async def run(self, max_edges):
        d, rm, fci = self.dut, self.rm, self.fci
        in_pkt = False
        prev = {"dlinit": 0, "rv": 0, "err": 0}
        for n in range(max_edges):
            await RisingEdge(d.clk_i)
            if self.stop:
                break
            if int(d.rst_i.value):
                continue
            s = int(fci.start_flow_control_i.value)
            if s and not prev["dlinit"]:
                self.ev.append(("DLINIT", n))
            prev["dlinit"] = s
            if int(d.m_phy_axis_tvalid.value) and int(d.m_phy_axis_tready.value):
                if not in_pkt:
                    self.ev.append(("TX_FIRST", n, int(d.m_phy_axis_tuser.value),
                                    int(d.m_phy_axis_tdata.value)))
                last = int(d.m_phy_axis_tlast.value)
                if last:
                    self.ev.append(("TX_LAST", n, int(d.m_phy_axis_tuser.value)))
                in_pkt = not last
            if int(rm.tx_valid_i.value):
                self.ev.append(("TXV", n, int(rm.tx_seq_num_i.value)))
            rv = int(rm.retry_valid_o.value)
            if rv & ~prev["rv"]:
                self.ev.append(("REPLAY", n, rv & ~prev["rv"]))
            prev["rv"] = rv
            er = int(rm.retry_err_o.value)
            if er and not prev["err"]:
                self.ev.append(("ERR", n))
            prev["err"] = er


def _dump(name, mon):
    with open(f"pr7g2_dll_{name}.json", "w") as f:
        json.dump(mon.ev, f)


@cocotb.test()
async def g7g2_initfc1_originate_silent_peer(dut):
    """C-7G2-11: DL_Init rise -> first InitFC1-P, and the repeat spacing, with a
    far end that never transmits anything."""
    tb = TB(dut)
    mon = RawMon(dut)
    task = cocotb.start_soon(mon.run(8000))
    await link_up_silent(tb)
    await task
    _dump("initfc1", mon)
    assert any(e[0] == "DLINIT" for e in mon.ev), "non-vacuity: DL_Init never rose"


@cocotb.test()
async def g7g2_replay_timer_ack_withheld(dut):
    """C-7G2-6: one TLP transmitted after FC init, and NO Ack ever returned.
    Captures tx_valid, every replay rise, retry_err, and every packet on the
    PHY-facing stream for 12,000 edges."""
    tb = TB(dut)
    await tb.reset()
    dut.idle_valid_i.value = 1
    dut.phy_link_up_i.value = 1
    await tb.wait_cycles(50)
    await send_flow_control_initialization(tb)
    for _ in range(50000):
        await RisingEdge(dut.clk_i)
        if int(dut.fc_initialized_o.value):
            break
    assert int(dut.fc_initialized_o.value), "fc_initialized_o never rose"
    await tb.wait_cycles(100)
    mon = RawMon(dut)
    task = cocotb.start_soon(mon.run(12000))
    raw_tlp, _ = build_memory_write(payload_length=16, tag=0x62)
    await send_frame_with_timeout(tb.tlp_source, raw_tlp, "7g2 TLP left unacknowledged")
    await task
    _dump("replay", mon)
    assert any(e[0] == "TXV" for e in mon.ev), "non-vacuity: the TLP never reached retry_management"
