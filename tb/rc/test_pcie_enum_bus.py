"""Stage D increment 3 -- pcie_enum_bus standalone (N1..N10).

The DUT is the bridge bus-number sequencer plus the one real pcie_cfg_txn it
drives; the bench plays the pcie_rq_rc_top socket via enum_tb_common.Socket
(which asserts its own physical-ordering invariants) and drives the scan's
verdict surface directly, so every branch of the eligibility check is
reachable without a scan in the loop.

What this target owns: the ONE write's golden (whole descriptor, payload,
derived routing Dword), its Type (0 -- Trap C), the outcome classification
including every fault path, the ORDERING of the handoff against the write's
completion, and the bypass/single-shot structure.  What it does not own: the
widened mux and the on-wire TLPs, which are increment 4's integration target.

!! F3.5: the no-settle variants are MANDATORY here.  Timeout and
late-completion events live under this FSM; the 2b-3 "no timer, null result"
exemption does not carry over.  N6/N7 are those variants.

Spec cited (read, not assumed):
  Type 1 header, 18h Dword ......... PCIe Base 2.1 SS7.5.3 Figure 7-6 p.492
  Secondary Latency Timer RO 00h ... PCIe Base 2.1 SS7.5.3.3 p.493 (P4.2)
  Primary functionally inert ....... PCIe Base 2.1 SS7.5.3.2 p.493 (P4.3)
  The write is Type 0 .............. [PCI30] SS3.2.2.3.x p.49 (the target is
                                     on the LOCAL bus); Trap C, SS8.3
  Reset-state bridge URs Type 1 .... PCIe Base 2.1 SS7.3.3 p.481 (P5.1)
  CRS on the write is normal ....... PCIe Base 2.1 SS2.3.2 p.121 (P6.1/P6.3)
  Ordering claim is SCOPED ......... P5.7: acceptance criterion for THIS
                                     sequencer, not a spec check
"""

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ReadOnly, RisingEdge

from enum_tb_common import (
    BRIDGE_BDF, BUS_NUM_WDATA, CLK_NS, SEC_BUS,
    CFG_BE_DWORD, CFG_REG_BUS_NUMBER,
    CPL_CA, CPL_CRS, CPL_SC, CPL_UR,
    ENUM_ERR_CA, ENUM_ERR_CRS_EXHAUSTED, ENUM_ERR_NONE, ENUM_ERR_TIMEOUT,
    ENUM_ERR_CREDIT_STARVED,
    ENUM_ERR_UR_POST_PROBE,
    RQ_CFG_WRITE0,
    Socket,
    assert_rq_descriptor, cfg_wire_dw2, decode_rq_desc, err_name,
)

CRS_RETRY_MAX = 3               # tb_pcie_enum_bus.sv override
BRIDGE_BUS = 0x01

HDR_BRIDGE = 0x01               # Type 1 layout
HDR_BRIDGE_MF = 0x81            # Type 1 layout, multi-function bit set
HDR_ENDPOINT = 0x00
HDR_CARDBUS = 0x02              # a header layout nothing here enumerates


# ==========================================================================
# Harness
# ==========================================================================
async def init(dut, tag_delay=2):
    cocotb.start_soon(Clock(dut.clk_i, CLK_NS, units="ns").start())
    dut.rst_i.value = 1
    dut.bus_start_i.value = 0
    dut.device_present_i.value = 0
    dut.unsupported_device_i.value = 0
    dut.header_type_i.value = 0
    dut.bridge_bus_i.value = BRIDGE_BUS
    dut.tx_fc_blocked_i.value = 0
    dut.s_axis_rq_tready_i.value = 1
    dut.pcie_rq_tag_i.value = 0
    dut.pcie_rq_tag_vld_i.value = 0
    dut.m_axis_rc_tdata_i.value = 0
    dut.m_axis_rc_tkeep_i.value = 0
    dut.m_axis_rc_tvalid_i.value = 0
    dut.m_axis_rc_tlast_i.value = 0
    dut.cpl_timeout_valid_i.value = 0
    dut.cpl_timeout_tag_i.value = 0
    for _ in range(4):
        await RisingEdge(dut.clk_i)
    dut.rst_i.value = 0
    await RisingEdge(dut.clk_i)
    sock = Socket(dut, tag_delay=tag_delay)
    sock.start()
    await RisingEdge(dut.clk_i)
    return sock


async def start_bus(dut, present=1, unsupported=1, header=HDR_BRIDGE):
    """Present the scan's verdict and pulse bus_start_i for one cycle.

    The verdict inputs stay driven afterwards -- in the real assembly they
    are the scan's terminal-state outputs and are stable by construction.
    """
    dut.device_present_i.value = present
    dut.unsupported_device_i.value = unsupported
    dut.header_type_i.value = header
    await RisingEdge(dut.clk_i)
    dut.bus_start_i.value = 1
    await RisingEdge(dut.clk_i)
    dut.bus_start_i.value = 0


async def settle(dut, cycles=20):
    for _ in range(cycles):
        await RisingEdge(dut.clk_i)


async def status(dut):
    await ReadOnly()
    snap = {
        "busy": int(dut.bus_busy_o.value),
        "done": int(dut.bus_done_o.value),
        "bypassed": int(dut.bus_bypassed_o.value),
        "error": int(dut.bus_error_o.value),
        "code": int(dut.bus_error_code_o.value),
        "blocked": int(dut.err_credit_blocked_o.value),
        "sec_bus": int(dut.sec_bus_o.value),
        "type1": int(dut.bus_type1_o.value),
    }
    await RisingEdge(dut.clk_i)
    return snap


async def wait_terminal(dut, cycles=2000):
    for _ in range(cycles):
        await ReadOnly()
        reached = (int(dut.bus_done_o.value) or int(dut.bus_error_o.value)
                   or int(dut.bus_bypassed_o.value))
        await RisingEdge(dut.clk_i)
        if reached:
            return
    raise AssertionError("pcie_enum_bus never reached a terminal state")


def assert_bus_number_write(req, what=""):
    """⭐ The one golden of this stage: the whole 18h write, all three layers.

    Whole 128-bit descriptor (a field-subset check is Trap A's blind set),
    the payload Dword, and the DERIVED routing Dword -- the descriptor's
    Completer ID and address fields laid out exactly as tlp_generator will
    emit DW2, compared against cfg_wire_dw2(bridge, 0, 0, reg 6).
    """
    assert req.write, f"{what}the 18h transaction must be a WRITE with payload"
    assert_rq_descriptor(req.desc, req.tuser, write=True, bdf=BRIDGE_BDF,
                         reg_num=CFG_REG_BUS_NUMBER, first_be=CFG_BE_DWORD,
                         type1=False, what=what)
    # Trap C, stated on its own even though the whole-word compare above
    # already pins it: transaction #3 is Type 0.  The bridge is on the LOCAL
    # bus; a CfgWr1 here would be answered UR by a spec-faithful bridge
    # (Secondary still 00h -- SS7.3.3 p.481 case 3).
    req_type = (req.desc >> 75) & 0xF
    assert req_type == RQ_CFG_WRITE0, (
        f"{what}the bus-number write went out with req_type {req_type:#06b} "
        "-- it must be a CfgWr0 (Trap C, SS8.3: the bridge is on the local "
        "bus, and a Type 1 write to 18h gets UR from a reset-state bridge)")
    assert req.payload == [BUS_NUM_WDATA], (
        f"{what}payload {[hex(w) for w in req.payload]} != "
        f"[{BUS_NUM_WDATA:#010x}] -- every byte of the 18h Dword is a "
        "distinct value (P5.2) precisely so a swapped or duplicated field "
        "shows here")
    fields = decode_rq_desc(req.desc)
    derived_dw2 = (fields["completer_id"] << 16) | (fields["address"] & 0xFFC)
    golden_dw2 = cfg_wire_dw2(BRIDGE_BUS, 0, 0, CFG_REG_BUS_NUMBER)
    assert derived_dw2 == golden_dw2, (
        f"{what}derived routing Dword {derived_dw2:#010x} != "
        f"cfg_wire_dw2 golden {golden_dw2:#010x}")


def assert_no_handoff(snap, what=""):
    """The handoff surface is fully de-asserted: done, sec_bus AND type1."""
    assert snap["done"] == 0, f"{what}bus_done_o asserted: {snap}"
    assert snap["sec_bus"] == 0, \
        f"{what}sec_bus_o presents {snap['sec_bus']:#04x} : {snap}"
    assert snap["type1"] == 0, f"{what}bus_type1_o asserted: {snap}"


# ==========================================================================
# N1 -- the one write, its golden, and the handoff ordering
# ==========================================================================
@cocotb.test()
async def n1_bus_number_write_golden_and_handoff_order(dut):
    """One CfgWr0 to register 6, whole-Dword, 0x00090501 -- and NO handoff
    strobe before that write's completion is classified.

    The ordering half is the F3.2 analogue at this level: a bridge at reset
    URs every Type 1 (Secondary = 00h), so a sequencer that handed off before
    the write completed would launch the secondary probe into a bridge that
    cannot route it.  Falsified against a deliberately reordered FSM -- the
    kill is recorded in the commit message.
    """
    sock = await init(dut)
    await start_bus(dut)
    await sock.wait_for(1)
    req = sock.requests[0]
    assert_bus_number_write(req, "N1 ")
    assert req.tkeep == 0xF, f"descriptor beat tkeep {req.tkeep:#x}"
    assert req.beats[1][1] == 0x1, "payload beat tkeep must be one Dword"

    # ⭐ THE ORDERING CHECK.  The write is on the wire but NOT completed; the
    # handoff surface must be fully quiet.  This is the assertion the
    # reordered-FSM falsification must fail.
    await settle(dut, 10)
    snap = await status(dut)
    assert snap["busy"] == 1, f"N1 not busy while awaiting the completion: {snap}"
    assert_no_handoff(snap, "N1 (write in flight): ")

    await sock.complete(req, status=CPL_SC)
    await wait_terminal(dut)
    snap = await status(dut)
    assert snap["done"] == 1 and snap["error"] == 0 and snap["bypassed"] == 0, \
        f"N1 outcome: {snap}"
    assert snap["sec_bus"] == SEC_BUS, \
        f"sec_bus_o {snap['sec_bus']:#04x} != {SEC_BUS:#04x}"
    assert snap["type1"] == 1, "bus_type1_o must assert with the handoff"
    assert snap["busy"] == 0 and snap["code"] == ENUM_ERR_NONE

    await settle(dut, 40)
    assert len(sock.requests) == 1, (
        f"{len(sock.requests)} packets for the bus-number phase -- it owns "
        f"exactly ONE transaction: {sock.requests}")


# ==========================================================================
# N2 -- CRS on the 18h write is normal (P6.1); the reissue is exact (P6.3)
# ==========================================================================
@cocotb.test()
async def n2_crs_retry_reissues_identically(dut):
    """CRS -> backoff -> byte-identical reissue -> SC -> handoff.

    A bridge is as entitled to a self-initialisation period as any device;
    the sequencer never sees the CRS (the primitive retries phase-blind),
    and the handoff still waits for the final SC.
    """
    sock = await init(dut)
    await start_bus(dut)
    await sock.wait_for(1)
    await sock.complete(sock.requests[0], status=CPL_CRS)

    await sock.wait_for(2)
    first, retry = sock.requests[0], sock.requests[1]
    assert retry.desc == first.desc and retry.payload == first.payload, (
        "the CRS reissue differs from the original -- a retry must repeat "
        "the request exactly (P6.3)")
    assert_bus_number_write(retry, "N2 retry ")

    # Still no handoff: a CRS'd write has NOT completed.
    snap = await status(dut)
    assert_no_handoff(snap, "N2 (between CRS and SC): ")

    await sock.complete(retry, status=CPL_SC)
    await wait_terminal(dut)
    snap = await status(dut)
    assert snap["done"] == 1 and snap["sec_bus"] == SEC_BUS, f"N2: {snap}"
    assert len(sock.requests) == 2


# ==========================================================================
# N3 / N4 / N5 -- the fault classifications
# ==========================================================================
@cocotb.test()
async def n3_ur_post_discovery_is_a_fault(dut):
    """UR on the 18h write is ENUM_ERR_UR_POST_PROBE, never 'absent'.

    The probe-phase 'UR means nothing here' policy does NOT apply: this
    stage only runs after the bridge answered two probe reads, so it is
    known present and a UR to a legal write of its own register is a fault.
    """
    sock = await init(dut)
    await start_bus(dut)
    await sock.wait_for(1)
    await sock.complete(sock.requests[0], status=CPL_UR)
    await wait_terminal(dut)
    snap = await status(dut)
    assert snap["error"] == 1 and snap["code"] == ENUM_ERR_UR_POST_PROBE, \
        f"N3 expected ENUM_ERR_UR_POST_PROBE, got {err_name(snap['code'])}: {snap}"
    assert_no_handoff(snap, "N3 (errored): ")
    await settle(dut, 40)
    assert len(sock.requests) == 1, "a UR must not be retried"


@cocotb.test()
async def n4_completer_abort_is_a_fault(dut):
    """CA classifies as ENUM_ERR_CA."""
    sock = await init(dut)
    await start_bus(dut)
    await sock.wait_for(1)
    await sock.complete(sock.requests[0], status=CPL_CA)
    await wait_terminal(dut)
    snap = await status(dut)
    assert snap["error"] == 1 and snap["code"] == ENUM_ERR_CA, \
        f"N4 expected ENUM_ERR_CA, got {err_name(snap['code'])}: {snap}"
    assert_no_handoff(snap, "N4: ")


@cocotb.test()
async def n5_crs_exhaustion_is_a_fault(dut):
    """CRS every time -> ENUM_ERR_CRS_EXHAUSTED after the bounded retries."""
    sock = await init(dut)
    await start_bus(dut)
    for attempt in range(CRS_RETRY_MAX + 1):
        await sock.wait_for(attempt + 1)
        await sock.complete(sock.requests[attempt], status=CPL_CRS)
    await wait_terminal(dut)
    snap = await status(dut)
    assert snap["error"] == 1 and snap["code"] == ENUM_ERR_CRS_EXHAUSTED, \
        f"N5 expected ENUM_ERR_CRS_EXHAUSTED, got {err_name(snap['code'])}: {snap}"
    assert_no_handoff(snap, "N5: ")
    assert len(sock.requests) == CRS_RETRY_MAX + 1, "the retry loop is unbounded"


# ==========================================================================
# N6 / N7 -- ⭐ the MANDATORY no-settle variants (F3.5)
# ==========================================================================
@cocotb.test()
async def n6_timeout_without_credit_annotation(dut):
    """The write never completes; the tracker's strobe ends it as a timeout.

    tx_fc_blocked_i is LOW, so err_credit_blocked_o must stay low -- the
    annotation must not fire on a timeout that credit did not cause.
    """
    sock = await init(dut)
    await start_bus(dut)
    await sock.wait_for(1)
    await settle(dut, 10)
    await sock.fire_timeout(sock.requests[0].tag)
    await wait_terminal(dut)
    snap = await status(dut)
    assert snap["error"] == 1 and snap["code"] == ENUM_ERR_TIMEOUT, \
        f"N6 expected ENUM_ERR_TIMEOUT, got {err_name(snap['code'])}: {snap}"
    assert snap["blocked"] == 0, \
        "err_credit_blocked_o annotated a timeout with tx_fc_blocked_i low"
    assert_no_handoff(snap, "N6: ")
    await settle(dut, 40)
    assert len(sock.requests) == 1, "a timeout must not trigger a reissue"


@cocotb.test()
async def n7_timeout_with_credit_annotation_and_late_completion(dut):
    """Credit-flavoured timeout annotated; the late completion is inert.

    The second half is the late-completion event living under this FSM: a
    stale completion bearing the dead tag arrives AFTER the terminal state
    and must perturb nothing -- transparency, proved rather than assumed.
    """
    sock = await init(dut)
    await start_bus(dut)
    await sock.wait_for(1)
    dead = sock.requests[0]
    dut.tx_fc_blocked_i.value = 1
    await sock.fire_timeout(dead.tag)
    await wait_terminal(dut)
    dut.tx_fc_blocked_i.value = 0
    snap = await status(dut)
    # sec 63 #7f #19: credit-flavoured timeouts carry their own code.
    assert snap["error"] == 1 and snap["code"] == ENUM_ERR_CREDIT_STARVED, (
        f"N7 expected ENUM_ERR_CREDIT_STARVED, got {err_name(snap['code'])}: {snap}")
    assert snap["blocked"] == 1, (
        "err_credit_blocked_o low on a timeout with tx_fc_blocked_i asserted "
        "-- the annotation is the only thing distinguishing credit starvation "
        "from a dead bridge")

    # The late completion for the quarantined tag: drained upstream in the
    # real stack; here it reaches the primitive, which is idle and must
    # ignore it -- and the SEQUENCER's terminal state must not move.
    await sock.complete(dead, status=CPL_SC)
    await settle(dut, 40)
    later = await status(dut)
    assert later == snap, (
        f"the status surface moved on a late completion:\n  was {snap}\n"
        f"  now {later}")
    assert len(sock.requests) == 1


# ==========================================================================
# N8 / N9 -- eligibility: the bypass paths and the multifunction bridge
# ==========================================================================
@cocotb.test()
async def n8_bypass_paths_emit_nothing(dut):
    """No device / a Type 0 device / a non-bridge layout: terminal S_BYPASS,
    ZERO transactions, no handoff -- the direct-attach path never sees this
    stage take ownership.
    """
    cases = [
        ("absent",   dict(present=0, unsupported=0, header=0x00)),
        ("type0",    dict(present=1, unsupported=0, header=HDR_ENDPOINT)),
        ("cardbus",  dict(present=1, unsupported=1, header=HDR_CARDBUS)),
    ]
    for name, kwargs in cases:
        sock = await init(dut)
        await start_bus(dut, **kwargs)
        await wait_terminal(dut)
        snap = await status(dut)
        assert snap["bypassed"] == 1 and snap["error"] == 0, \
            f"N8 [{name}]: {snap}"
        assert_no_handoff(snap, f"N8 [{name}]: ")
        await settle(dut, 40)
        assert len(sock.requests) == 0, (
            f"N8 [{name}]: {len(sock.requests)} transaction(s) emitted for a "
            f"verdict this stage must bypass: {sock.requests}")


@cocotb.test()
async def n9_multifunction_bridge_still_classifies(dut):
    """Header 81h -- Type 1 layout with the multi-function bit -- proceeds.

    Classification only (P4.5): bit 7 is masked exactly as the scan masks
    it.  Enumerating functions 1-7 is out of scope; Function stays 0 by
    construction on both sides (P5.3).
    """
    sock = await init(dut)
    await start_bus(dut, header=HDR_BRIDGE_MF)
    await sock.wait_for(1)
    assert_bus_number_write(sock.requests[0], "N9 ")
    await sock.complete(sock.requests[0], status=CPL_SC)
    await wait_terminal(dut)
    snap = await status(dut)
    assert snap["done"] == 1 and snap["sec_bus"] == SEC_BUS, f"N9: {snap}"


# ==========================================================================
# N10 -- poison when idle, and single-shot after every terminal
# ==========================================================================
@cocotb.test()
async def n10_poison_when_idle_and_single_shot(dut):
    """Garbage on the response path while idle moves nothing; a second
    bus_start_i after S_DONE is ignored (re-entry is reset-only)."""
    sock = await init(dut)

    # Poison: a full completion packet arrives with NOTHING started.
    await sock.complete(None, tag=0x77, status=CPL_SC, payload=[0xBAD0BAD0],
                        dword_count=1)
    await settle(dut, 20)
    snap = await status(dut)
    assert snap["busy"] == 0 and snap["error"] == 0 and snap["done"] == 0, \
        f"N10 idle state moved on a stray completion: {snap}"
    assert len(sock.requests) == 0

    # Happy path to S_DONE...
    await start_bus(dut)
    await sock.wait_for(1)
    await sock.complete(sock.requests[0], status=CPL_SC)
    await wait_terminal(dut)
    first = await status(dut)
    assert first["done"] == 1

    # ...then a second start pulse.  Single-shot: sampled in S_IDLE only,
    # and the terminal states self-loop until reset.
    dut.bus_start_i.value = 1
    await RisingEdge(dut.clk_i)
    dut.bus_start_i.value = 0
    await settle(dut, 60)
    later = await status(dut)
    assert later == first, (
        f"N10: the terminal state moved on a re-start:\n  was {first}\n"
        f"  now {later}")
    assert len(sock.requests) == 1, (
        "N10: a second bus_start_i produced a second write -- the stage "
        "re-armed, which the single-shot invariant forbids")
