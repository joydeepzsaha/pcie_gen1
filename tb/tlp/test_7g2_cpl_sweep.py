"""§63 #7g-2 Phase 1, C-7G2-7 -- where 6,250 actually fires. NOT A GATE ROW.

Target `measure_7g2_cpl_sweep` elaborates tb_tlp_request_tracker with NO
parameter override, so `dut` runs the bench copy (6250) and
`dut_default_witness` runs tlp_request_tracker.sv's own shipped default -- the
pair t1c already compares at one phase.

The tracker checks expiry ROUND-ROBIN, one tag per cycle (tlp_request_tracker
.sv:246-250), so a tag allocated when the scan index reads s0 cannot be declared
expired until the scan comes back to it. Reading the source gives
    k = 6250 + ((tag - s0 - 10) mod 32)          (6250 mod 32 = 10)
for the edge at which the strobe registers, i.e. a 32-cycle window [6250, 6281]
and exactly-6250 in one phase of 32. That is a PREDICTION about the artifact
(pcie_docs PREDICTIONS_7G2.md, aaf21ea), not a measurement. This module
measures it: for every phase p = 0..31 it resets, idles p cycles, allocates one
tag, and counts edges to the strobe on both instances.

Measurement discipline (the same as t1b/t1c, so the numbers are comparable to
t1c's k=6271): stimulus changes 1 ps after an edge; every observation is taken
1 ps after an edge, so it reads the value the edge just committed. `k` is the
number of edges after the allocation handshake edge at which the strobe is
first seen high -- the edge that registered it.

s0 is read from the tracker's own scan_index_r (public via --public-flat-rw)
in the cycle the handshake is presented, i.e. the value the handshake edge
samples. Never mirrored in Python (the tracker-contract lesson: a mirror was
wrong twice).
"""
import json

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, Timer

TAG_COUNT = 32
LIMIT = 6250
CLK_NS = 10          # this bench's clock; cycles are what is measured
RID = 0x1234
INPUTS = ("allocate_valid", "completion_valid", "extended_tag_enable",
          "allocate_requester_id", "allocate_byte_count", "allocate_address",
          "allocate_context", "allocate_expects_data", "completion_requester_id",
          "completion_tag", "completion_status", "completion_payload_bytes",
          "completion_byte_count", "completion_lower_address")


async def _one_phase(dut, p):
    dut.rst_i.value = 1
    for name in INPUTS:
        getattr(dut, name).value = 0
    dut.result_ready.value = 1
    for _ in range(3):
        await RisingEdge(dut.clk_i)
    await Timer(1, units="ps")
    dut.rst_i.value = 0
    for _ in range(1 + p):
        await RisingEdge(dut.clk_i)
    await Timer(1, units="ps")

    dut.allocate_requester_id.value = RID
    dut.allocate_byte_count.value = 4
    dut.allocate_expects_data.value = 1
    dut.allocate_valid.value = 1
    await Timer(1, units="ps")
    assert int(dut.allocate_ready.value) and int(dut.w_allocate_ready.value)
    tag = int(dut.allocate_tag.value)
    wtag = int(dut.w_allocate_tag.value)
    s0 = int(dut.dut.scan_index_r.value)
    ws0 = int(dut.dut_default_witness.scan_index_r.value)
    await RisingEdge(dut.clk_i)          # the handshake edge
    await Timer(1, units="ps")
    dut.allocate_valid.value = 0

    k_dut = k_wit = None
    for k in range(1, LIMIT + TAG_COUNT + 8):
        await RisingEdge(dut.clk_i)
        await Timer(1, units="ps")
        if int(dut.cpl_timeout_valid.value) and k_dut is None:
            k_dut = k
        if int(dut.w_cpl_timeout_valid.value) and k_wit is None:
            k_wit = k
    return {"p": p, "tag": tag, "wtag": wtag, "s0": s0, "ws0": ws0,
            "k_dut": k_dut, "k_wit": k_wit}


@cocotb.test()
async def g7g2_cpl_timeout_phase_sweep(dut):
    cocotb.start_soon(Clock(dut.clk_i, CLK_NS, units="ns").start())
    rows = []
    for p in range(TAG_COUNT):
        r = await _one_phase(dut, p)
        rows.append(r)
        dut._log.info("PR7G2_SWEEP " + json.dumps(r, sort_keys=True))
    with open("pr7g2_cpl_sweep.json", "w") as f:
        json.dump(rows, f, indent=1)
    # Non-vacuity only (§22.82): the bench instance fired in every phase.
    # sec 63 #7g-2 step 3: the witness now runs the SHIPPED 10 ms (1,250,000
    # cycles), far past this sweep's window, so k_wit is None by design; the
    # 10 ms value is pinned by test_tlp_cpl_timeout_default.t1c instead.
    assert all(r["k_dut"] is not None for r in rows), rows
