"""Commit 2b-2 -- pcie_enum_scan standalone (S1..S13).

The DUT is the presence sequencer plus the one pcie_cfg_txn it instantiates.
The Python bench plays their shared socket (enum_tb_common.Socket, which asserts
its own ordering invariants -- see that module).

WHAT THIS TARGET OWNS: the phase-dependent policy.  The primitive's own
behaviour is verilate_enum_txn's subject and is not re-tested here; what is
tested here is that the SAME outcome means different things in different phases,
that every fault path is terminal and sticky, and that the status surface is
stable and non-vacuous.

!! NO TEST HERE ASSERTS ANY PROPERTY OF A TAG VALUE.  Tag values are a property
of the model, not of the design: the real tracker recycles a tag as soon as a
completion carrying Request Completed retires it (PG213 :4257), which is exactly
what happened on a CRS retry in Commit 2b-1's test i3 and made an inequality
assertion there wrong.  The socket hands out incrementing tags; nothing may lean
on that.

Spec cited (read, not assumed):
  Device 0 only, and why ............ PCIe Base 2.1 SS7.3.1 p.479
  Endpoint config routing / UR ...... PCIe Base 2.1 SS7.3.3 p.480
  UR is the device-existence answer . PCIe Base 2.1 SS2.3.2 IN p.122
  Completion timeout is an error .... PCIe Base 2.1 SS2.8 p.152
  Type 0 header layout / offsets .... PCIe Base 2.1 Figure 7-5 p.491
  Header Type bit fields ............ [PCI3-REF] PCI 3.0 SS6.1 -- see SSD.3
Full derivation: docs/predictions/SPEC_PREDICTIONS_ENUM.md SSD.
"""

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ReadOnly, RisingEdge

from enum_tb_common import (
    BDF, CLK_NS, DEVICE, HDR_TYPE0, HDR_TYPE0_MF, REG0, SCAN_BUS, VENDOR,
    ENUM_ERR_CA, ENUM_ERR_CRS_EXHAUSTED, ENUM_ERR_NONE,
    ENUM_ERR_TIMEOUT, ENUM_ERR_UR_POST_PROBE, ENUM_ERR_CREDIT_STARVED,
    CFG_BE_DWORD, CFG_REG_CACHE_HEADER, CFG_REG_VENDOR_DEVICE,
    CPL_CA, CPL_CRS, CPL_SC, CPL_UR,
    Socket, assert_rq_descriptor,
)


CRS_RETRY_MAX = 3          # tb_pcie_enum_scan.sv override


# pcie_enum_pkg::enum_error_e
# CLK_NS / BDF / ENUM_ERR_* now come from enum_tb_common -- byte-identical
# across every enum bench, and the BAR benches would have made a fourth copy.

ERR_NAME = {
    ENUM_ERR_NONE: "ENUM_ERR_NONE",
    ENUM_ERR_UR_POST_PROBE: "ENUM_ERR_UR_POST_PROBE",
    ENUM_ERR_CA: "ENUM_ERR_CA",
    ENUM_ERR_CRS_EXHAUSTED: "ENUM_ERR_CRS_EXHAUSTED",
    ENUM_ERR_TIMEOUT: "ENUM_ERR_TIMEOUT",
}

# Register 0 payload: {Device ID[31:16], Vendor ID[15:0]}

# Register 3 payload: {BIST[31:24], Header Type[23:16], MLT[15:8], CLS[7:0]}
def reg3(header_type, bist=0x00, mlt=0x00, cls=0x10):
    return (bist << 24) | ((header_type & 0xFF) << 16) | (mlt << 8) | cls


HDR_TYPE1 = 0x01           # PCI-PCI bridge -- valid, but not enumerable by 2b


# ==========================================================================
# Harness
# ==========================================================================
async def init(dut, tag_delay=2, bus=SCAN_BUS):
    cocotb.start_soon(Clock(dut.clk_i, CLK_NS, units="ns").start())
    dut.rst_i.value = 1
    dut.scan_start_i.value = 0
    dut.scan_bus_i.value = bus
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


async def start_scan(dut):
    dut.scan_start_i.value = 1
    await RisingEdge(dut.clk_i)
    dut.scan_start_i.value = 0


async def settle(dut, cycles=30):
    for _ in range(cycles):
        await RisingEdge(dut.clk_i)


async def wait_terminal(dut, cycles=6000):
    """Block until the scan reaches any terminal state.

    Both this and status() below sample under ReadOnly but RETURN from a
    writable phase, so a caller can drive a signal immediately afterwards.
    Returning straight out of ReadOnly makes the next write raise
    "scheduled during a read-only sync phase", which is a bench fault that
    looks nothing like the DUT behaviour under test.
    """
    for _ in range(cycles):
        await ReadOnly()
        reached = int(dut.scan_done_o.value) or int(dut.scan_error_o.value)
        await RisingEdge(dut.clk_i)
        if reached:
            return
    raise AssertionError("the scan never reached a terminal state")


async def status(dut):
    await ReadOnly()
    snapshot = {
        "busy": int(dut.scan_busy_o.value),
        "done": int(dut.scan_done_o.value),
        "error": int(dut.scan_error_o.value),
        "code": int(dut.scan_error_code_o.value),
        "credit_blocked": int(dut.err_credit_blocked_o.value),
        "present": int(dut.device_present_o.value),
        "unsupported": int(dut.unsupported_device_o.value),
        "bdf": int(dut.device_bdf_o.value),
        "vendor": int(dut.vendor_id_o.value),
        "device": int(dut.device_id_o.value),
        "header_type": int(dut.header_type_o.value),
        "mf": int(dut.multifunction_o.value),
    }
    await RisingEdge(dut.clk_i)
    return snapshot


def err_name(code):
    return ERR_NAME.get(code, f"<unknown {code}>")


async def run_probe(sock, dut, status_=CPL_SC, data=REG0):
    """Start the scan and answer the Vendor/Device ID probe."""
    await start_scan(dut)
    await sock.wait_for(1)
    probe = sock.requests[0]
    assert_rq_descriptor(probe.desc, probe.tuser, write=False, bdf=BDF,
                         reg_num=CFG_REG_VENDOR_DEVICE, first_be=CFG_BE_DWORD,
                         what="probe ")
    await sock.complete(probe, status=status_, data=data)
    return probe


async def run_header(sock, dut, status_=CPL_SC, header_type=HDR_TYPE0):
    """Answer the Header Type read that follows a successful probe."""
    await sock.wait_for(2)
    hdr = sock.requests[1]
    assert_rq_descriptor(hdr.desc, hdr.tuser, write=False, bdf=BDF,
                         reg_num=CFG_REG_CACHE_HEADER, first_be=CFG_BE_DWORD,
                         what="header ")
    await sock.complete(hdr, status=status_, data=reg3(header_type))
    return hdr


# ==========================================================================
# S1 / S2 -- the two normal outcomes
# ==========================================================================
@cocotb.test()
async def s1_device_found(dut):
    """A device answers both reads: present, IDs captured, Type 0, done."""
    sock = await init(dut)
    await run_probe(sock, dut)
    await run_header(sock, dut, header_type=HDR_TYPE0)
    await wait_terminal(dut)
    st = await status(dut)

    assert st["done"] == 1 and st["error"] == 0, f"terminal state wrong: {st}"
    assert st["present"] == 1, "the device answered its probe but is not reported present"
    assert st["unsupported"] == 0, "a Type 0 header is enumerable"
    assert st["vendor"] == VENDOR, f"vendor_id_o {st['vendor']:#06x} != {VENDOR:#06x}"
    assert st["device"] == DEVICE, f"device_id_o {st['device']:#06x} != {DEVICE:#06x}"
    assert st["header_type"] == HDR_TYPE0, f"header_type_o {st['header_type']:#04x}"
    assert st["mf"] == 0, "the multi-function bit was clear in the response"
    assert st["bdf"] == BDF, f"device_bdf_o {st['bdf']:#06x} != {BDF:#06x}"
    assert st["code"] == ENUM_ERR_NONE
    assert st["busy"] == 0, "scan_busy_o still high in a terminal state"

    # ⭐ EXACTLY TWO TRANSACTIONS. Base 2.1 SS7.3.1 p.479 makes device 0 the only
    # device number that may be probed on this link, so there is no sweep.
    await settle(dut, 200)
    assert len(sock.requests) == 2, (
        f"{len(sock.requests)} config requests emitted, expected exactly 2 "
        f"(probe + header type). A device-number sweep would emit far more, and "
        f"SS7.3.1 p.479 forbids naming devices 1-31: {sock.requests}")


@cocotb.test()
async def s2_nothing_to_enumerate(dut):
    """UR on the probe is ABSENT, not an error -- and the scan still completes.

    On a point-to-point link with link_up_i asserted a device is always
    attached, so UR to the Function 0 probe means "nothing here to enumerate"
    (Base 2.1 SS7.3.1 p.479 for an unimplemented Function, SS7.3.3 p.480 for the
    general Endpoint rule).

    !! THE ASSERTIONS BELOW DELIBERATELY INCLUDE done AND error (prediction DF4).
    device_present_o is RESET-LOW, so asserting only "present == 0" would be
    satisfied by an FSM that never ran at all.
    """
    sock = await init(dut)
    await run_probe(sock, dut, status_=CPL_UR)
    await wait_terminal(dut)
    st = await status(dut)

    assert st["done"] == 1, "absence is a NORMAL completion of the scan"
    assert st["error"] == 0, (
        f"absence reported as an error ({err_name(st['code'])}); UR on the probe "
        "is how the spec signals a device with nothing to enumerate")
    assert st["present"] == 0, "a UR probe response must not report a device"
    assert st["code"] == ENUM_ERR_NONE
    assert st["vendor"] == 0 and st["device"] == 0, \
        "no IDs may be captured from a UR completion -- it carries no data"

    await settle(dut, 200)
    assert len(sock.requests) == 1, (
        f"{len(sock.requests)} requests -- no Header Type read may follow an "
        "absent device")


# ==========================================================================
# S3 -- the row that justifies the two-module split
# ==========================================================================
@cocotb.test()
async def s3_ur_after_the_probe_is_a_fault(dut):
    """The SAME wire event, one phase later, is an error.

    This is the entire reason pcie_cfg_txn and pcie_enum_scan are separate
    modules: TXN_UR means "nothing to enumerate" during the probe and "fault"
    afterwards, and the difference is context the primitive deliberately does
    not have.  A device that answered register 0 has no business rejecting a
    legal configuration read of register 3.
    """
    sock = await init(dut)
    await run_probe(sock, dut)                        # answers, so present
    await run_header(sock, dut, status_=CPL_UR)       # then rejects
    await wait_terminal(dut)
    st = await status(dut)

    assert st["error"] == 1, "UR after the probe must be a fault"
    assert st["code"] == ENUM_ERR_UR_POST_PROBE, \
        f"error code {err_name(st['code'])}, expected ENUM_ERR_UR_POST_PROBE"
    assert st["done"] == 0, "an errored scan is not done"
    # The device WAS present -- the probe succeeded -- and that stays reported.
    assert st["present"] == 1, \
        "the probe succeeded, so the device is present regardless of the fault"
    assert st["vendor"] == VENDOR, "the captured IDs survive a later fault"


# ==========================================================================
# S4 / S5 / S6 -- faults, in both phases
# ==========================================================================
@cocotb.test()
async def s4_completer_abort_in_either_phase(dut):
    """CA is a fault in both phases and carries the same code."""
    sock = await init(dut)
    await run_probe(sock, dut, status_=CPL_CA)
    await wait_terminal(dut)
    st = await status(dut)
    assert st["error"] == 1 and st["code"] == ENUM_ERR_CA, \
        f"CA on the probe gave {err_name(st['code'])}"
    assert st["present"] == 0, "a CA probe response is not a present device"


@cocotb.test()
async def s5_completer_abort_after_the_probe(dut):
    sock = await init(dut)
    await run_probe(sock, dut)
    await run_header(sock, dut, status_=CPL_CA)
    await wait_terminal(dut)
    st = await status(dut)
    assert st["error"] == 1 and st["code"] == ENUM_ERR_CA, \
        f"CA on the header read gave {err_name(st['code'])}"


@cocotb.test()
async def s6_crs_exhausted_is_a_fault(dut):
    """A device that answers CRS forever is a fault, in either phase.

    The bounded loop itself belongs to pcie_cfg_txn and is tested there (E8);
    what this checks is that the sequencer maps TXN_CRS_EXHAUSTED to a fault
    rather than to absence -- a device answering CRS is present and initialising,
    which is the opposite of absent.
    """
    sock = await init(dut)
    await start_scan(dut)
    for attempt in range(CRS_RETRY_MAX + 1):
        await sock.wait_for(attempt + 1)
        await sock.complete(sock.requests[attempt], status=CPL_CRS)
    await wait_terminal(dut)
    st = await status(dut)

    assert st["error"] == 1, "CRS exhaustion is a fault"
    assert st["code"] == ENUM_ERR_CRS_EXHAUSTED, \
        f"error code {err_name(st['code'])}, expected ENUM_ERR_CRS_EXHAUSTED"
    assert st["present"] == 0, "a device stuck in CRS was never identified"


# ==========================================================================
# S7 -- the Phase-1 derivation, made falsifiable
# ==========================================================================
@cocotb.test()
async def s7_timeout_during_the_probe_is_a_fault_not_absence(dut):
    """A completion timeout on the probe is an ERROR, never "device absent".

    THE PHASE-1 DERIVATION (docs/predictions/SPEC_PREDICTIONS_ENUM.md SS5.3), made falsifiable.
    Base 2.1 assigns the two events to different mechanisms and the FSM must not
    merge them:

      * absence answers with a UR -- SS2.3.2 Implementation Note p.122 names the
        device-existence probe explicitly;
      * silence is a reported error that "should never occur under normal
        operating conditions" -- SS2.8 p.152.

    Probing is normal operation.  If a timeout meant absence, SS2.8's "should
    never occur" would be violated by every scan of an empty slot.
    """
    sock = await init(dut)
    await start_scan(dut)
    await sock.wait_for(1)
    probe = sock.requests[0]

    await settle(dut, 10)
    await sock.fire_timeout(probe.tag)
    await wait_terminal(dut)
    st = await status(dut)

    assert st["error"] == 1, (
        "a probe timeout was NOT reported as an error -- if it was treated as "
        "absence, the Phase-1 derivation has been inverted (SS2.8 p.152 makes a "
        "completion timeout a reported error; absence answers with UR)")
    assert st["code"] == ENUM_ERR_TIMEOUT, \
        f"error code {err_name(st['code'])}, expected ENUM_ERR_TIMEOUT"
    assert st["done"] == 0, "an errored scan is not done"
    assert st["present"] == 0


# ==========================================================================
# S8 -- Type 1 is not an error
# ==========================================================================
@cocotb.test()
async def s8_type1_header_is_unsupported_not_an_error(dut):
    """A bridge answered correctly; we simply cannot enumerate it yet.

    Reporting a Type 1 header as an error would conflate "the link misbehaved"
    with "the topology is richer than I handle", and only the first is a fault.
    Base 2.1 SS7.5.3 p.492 defines the Type 1 header as the one used by "Switch and
    Root Complex virtual PCI Bridges"; walking one is Commits 3/4.
    """
    sock = await init(dut)
    await run_probe(sock, dut)
    await run_header(sock, dut, header_type=HDR_TYPE1)
    await wait_terminal(dut)
    st = await status(dut)

    assert st["error"] == 0, (
        f"a Type 1 header was reported as an error ({err_name(st['code'])}); it "
        "is a valid device this commit cannot enumerate, not a fault")
    assert st["done"] == 1, "the scan completed -- it just found a bridge"
    assert st["unsupported"] == 1, "unsupported_device_o must mark the Type 1 exit"
    assert st["present"] == 1, "the bridge is present; it answered both reads"
    assert st["header_type"] == HDR_TYPE1, (
        f"header_type_o {st['header_type']:#04x} -- the RAW byte must be reported "
        "so a consumer can tell 01h from 02h without the FSM enumerating codes "
        "it does not act on")


# ==========================================================================
# S9 -- the multi-function bit, both ways
# ==========================================================================
@cocotb.test()
async def s9_multifunction_bit_captured_both_ways(dut):
    """MF set and clear, from the correct bit of the correct byte.

    Driven both ways in one test so that neither a stuck-at output nor a
    wrong-bit extraction can pass: HDR_TYPE0_MF differs from HDR_TYPE0 in
    exactly bit 7, and the layout bits stay 00h in both.
    """
    sock = await init(dut)
    await run_probe(sock, dut)
    await run_header(sock, dut, header_type=HDR_TYPE0_MF)
    await wait_terminal(dut)
    st = await status(dut)

    assert st["mf"] == 1, (
        f"multifunction_o low for header type {HDR_TYPE0_MF:#04x} -- bit 7 is "
        "the multi-function bit ([PCI3-REF], SSD.3)")
    assert st["header_type"] == HDR_TYPE0_MF
    assert st["unsupported"] == 0, (
        "a multi-function endpoint is still Type 0 -- the MF bit is not part of "
        "the layout field, so masking it off wrongly would land here")
    assert st["done"] == 1 and st["error"] == 0


# ==========================================================================
# S10 -- status stability
# ==========================================================================
@cocotb.test()
async def s10_status_is_stable_and_terminal(dut):
    """Once terminal, the surface does not move and no further TLP is emitted.

    Enumeration is single-shot after link-up.  A status surface that could be
    re-entered would let a consumer sample it mid-rescan; a scan that kept
    issuing would burn credit for nothing.
    """
    sock = await init(dut)
    await run_probe(sock, dut)
    await run_header(sock, dut, header_type=HDR_TYPE0)
    await wait_terminal(dut)
    first = await status(dut)
    packets = len(sock.requests)

    # Poke it: re-raise scan_start_i, which a careless integrator might.
    for _ in range(3):
        await start_scan(dut)
        await settle(dut, 50)

    later = await status(dut)
    assert later == first, (
        f"the status surface moved after scan_done_o:\n  was  {first}\n  now  {later}")
    assert len(sock.requests) == packets, (
        f"{len(sock.requests) - packets} further config requests were emitted "
        "after the scan completed -- the terminal state is not terminal")


@cocotb.test()
async def s11_error_is_sticky(dut):
    """SCAN_ERROR is reset-only; scan_start_i cannot clear it."""
    sock = await init(dut)
    await run_probe(sock, dut, status_=CPL_CA)
    await wait_terminal(dut)
    st = await status(dut)
    assert st["error"] == 1 and st["code"] == ENUM_ERR_CA

    for _ in range(3):
        await start_scan(dut)
        await settle(dut, 50)
    again = await status(dut)
    assert again["error"] == 1 and again["code"] == ENUM_ERR_CA, \
        f"the sticky error cleared itself: {again}"
    assert again["done"] == 0, "an errored scan must never report done"
    assert len(sock.requests) == 1, "an errored scan must not reissue"


# ==========================================================================
# S12 -- the all-1s question, settled and asserted
# ==========================================================================
@cocotb.test()
async def s12_vendor_id_ffff_on_success_is_present(dut):
    """A Successful Completion carrying FFFFFFFF reports a PRESENT device.

    docs/predictions/SPEC_PREDICTIONS_ENUM.md SSD.5.1.  Base 2.1 SS2.3.2 Implementation Note p.122
    has a Root Complex synthesise an all-1s read value "when UR Completion
    Status is returned", FOR SOFTWARE ABOVE IT.  This FSM sits where that
    synthesis would be performed, not consumed -- it sees the UR directly, as
    TXN_UR.

    So absence is signalled by TXN_UR and by nothing else, and vendor_id FFFFh
    must NOT be reinterpreted as absence.  Re-deriving absence from a sentinel
    would discard information the spec took care to keep distinguishable.

    This test exists because the silent-conversion bug would pass an unasserted
    design: nothing else in the suite drives FFFFFFFF.
    """
    sock = await init(dut)
    await run_probe(sock, dut, data=0xFFFFFFFF)
    await run_header(sock, dut, header_type=HDR_TYPE0)
    await wait_terminal(dut)
    st = await status(dut)

    assert st["present"] == 1, (
        "a device that answered with a Successful Completion was reported "
        "ABSENT because its Vendor ID was FFFFh. Absence is signalled by UR "
        "(SSD.5.1); an SC is an SC whatever data it carries")
    assert st["vendor"] == 0xFFFF and st["device"] == 0xFFFF, \
        f"the reported IDs were altered: {st['vendor']:#06x}/{st['device']:#06x}"
    assert st["error"] == 0 and st["done"] == 1


# ==========================================================================
# S13 -- back-pressure on both internal handshakes
# ==========================================================================
@cocotb.test()
async def s13_arbitrary_stalls_resume_correctly(dut):
    """Long stalls on the RQ path must not perturb the sequence.

    The sequencer has no timer, so an arbitrarily long hold-off is normal.  The
    stall here (300 cycles) is far longer than the whole happy path, so any
    hidden cycle assumption would have fired.
    """
    sock = await init(dut)

    sock.stall_beats(300)
    await start_scan(dut)
    await settle(dut, 120)
    assert len(sock.requests) == 0, \
        "a request was emitted while s_axis_rq_tready was low"
    st = await status(dut)
    assert st["busy"] == 1, "the scan should report busy while stalled"
    assert st["done"] == 0 and st["error"] == 0, \
        "a credit-style stall is not an outcome"

    await sock.wait_for(1)
    probe = sock.requests[0]
    assert_rq_descriptor(probe.desc, probe.tuser, write=False, bdf=BDF,
                         reg_num=CFG_REG_VENDOR_DEVICE, first_be=CFG_BE_DWORD,
                         what="post-stall probe ")
    await sock.complete(probe, status=CPL_SC, data=REG0)

    sock.stall_beats(300)
    await run_header(sock, dut, header_type=HDR_TYPE0)
    await wait_terminal(dut)
    st = await status(dut)
    assert st["done"] == 1 and st["error"] == 0, f"stalled scan ended wrong: {st}"
    assert st["vendor"] == VENDOR and st["header_type"] == HDR_TYPE0
    assert len(sock.requests) == 2, "the stalls caused a duplicate request"


# ==========================================================================
# S14 / S15 -- the credit annotation, and the proof it is only an annotation
#
# docs/predictions/SPEC_PREDICTIONS_ENUM.md SSD.6.  err_credit_blocked_o records tx_fc_blocked_i at
# the moment a TXN_TIMEOUT is reported; it must never steer.  Master brief SS4.1
# forbids a credit signal gating control flow, and these two tests are what make
# that falsifiable.
#
# !! THE TWO CASES ARE COMPARED AGAINST A STATED GOLDEN, NOT AGAINST EACH OTHER.
# Running the scan twice and diffing the results is self-comparison -- it proves
# the two runs agree, which they would even if both were wrong the same way.
# Both are instead required to equal TIMEOUT_GOLDEN below, which was written from
# the SSD.5/SSD.6 policy rather than observed.
#
# The annotation matters because of a bound the sequencer cannot remove: the
# tracker measures per-tag age from ALLOCATION, which precedes the credit gate,
# so a request starved of credit past CPL_TIMEOUT_CYCLES times out having never
# been transmitted (Commit 2b-1 test i9).  All a client can do is say "this
# timeout smells like credit".
# ==========================================================================

# The complete status surface after a completion timeout on the probe, derived
# from SSD.5 (TXN_TIMEOUT -> ERROR in either phase) and SSD.6 (annotation only).
# `credit_blocked` is supplied per case; everything else is invariant.
TIMEOUT_GOLDEN = {
    "busy": 0,            # terminal
    "done": 0,            # an errored scan is never done
    "error": 1,
    "code": ENUM_ERR_TIMEOUT,
    "present": 0,         # a timed-out probe identified nothing
    "unsupported": 0,
    "bdf": BDF,
    "vendor": 0,
    "device": 0,
    "header_type": 0,
    "mf": 0,
}


async def _probe_timeout_case(dut, blocked):
    """Run one scan whose probe times out, with tx_fc_blocked_i forced."""
    sock = await init(dut)
    dut.tx_fc_blocked_i.value = blocked
    await start_scan(dut)
    await sock.wait_for(1)
    await settle(dut, 10)
    await sock.fire_timeout(sock.requests[0].tag)
    await wait_terminal(dut)
    st = await status(dut)

    # sec 63 #7f #19 (D-P3.5): a timeout the credit gate caused is reported
    # under its OWN code.  Everything else in the surface is unchanged, and
    # S15 still pins that the FLOW is identical -- only the name of the fault
    # differs between the two cases.  RED before the #19 src commit: the
    # engine reported ENUM_ERR_TIMEOUT with the annotation set, which is what
    # the full stack showed for F18's credit starvation (Phase 2e).
    expected = dict(TIMEOUT_GOLDEN, credit_blocked=blocked,
                    code=ENUM_ERR_CREDIT_STARVED if blocked else ENUM_ERR_TIMEOUT)
    assert st == expected, (
        f"tx_fc_blocked_i={blocked}: status surface does not match the SSD.5/SSD.6 "
        f"golden\n  observed {st}\n  expected {expected}")
    assert len(sock.requests) == 1, "a timed-out probe must not be reissued"
    return st


@cocotb.test()
async def s14_credit_annotation_set_when_blocked(dut):
    """Timeout with tx_fc_blocked_i HIGH: ERROR, annotated, and NAMED
    ENUM_ERR_CREDIT_STARVED (sec 63 #7f #19)."""
    await _probe_timeout_case(dut, blocked=1)


@cocotb.test()
async def s15_credit_annotation_clear_and_flow_identical(dut):
    """Timeout with tx_fc_blocked_i LOW: the SAME ERROR, unannotated.

    Together with S14 this pins that tx_fc_blocked_i appears in no next-state
    expression: both runs must equal TIMEOUT_GOLDEN in every field except the
    annotation bit, so a design that steered on credit would break one of them.
    """
    await _probe_timeout_case(dut, blocked=0)


# ==========================================================================
# S16 -- the timeout fired at the EARLIEST legal moment
#
# ADDED IN RESPONSE TO A SURVIVING BENCH MUTATION.  Defeating socket invariant 2
# (a timeout strobe may not fire for a tag that has not been strobed) passed
# S1..S15: every other timeout test calls settle() before firing, which happens
# to give the tag strobe time to land, so the invariant was never load-bearing.
#
# This is the same shape as Commit 2b-1, where e9 passed only because it settled
# first and e10 -- which did not -- caught the equivalent bug.  A guard that is
# never exercised is not a guard.
# ==========================================================================
@cocotb.test()
async def s16_timeout_at_the_earliest_legal_moment(dut):
    """Fire the timeout the instant the request is observed, with no settling.

    The socket must still hold it until the tag has actually been strobed --
    tlp_request_tracker cannot time out a tag it has not allocated.  Without
    that ordering the strobe lands before the primitive has latched its tag, is
    silently missed, and the scan hangs instead of erroring.

    The outcome must be identical to the settled case (S15), because WHEN the
    timeout is delivered is a property of the model, not of the design.
    """
    sock = await init(dut)
    await start_scan(dut)
    await sock.wait_for(1)

    # Deliberately NO settle() here -- that is the whole point of the test.
    await sock.fire_timeout(sock.requests[0].tag)
    await wait_terminal(dut)
    st = await status(dut)

    assert st == dict(TIMEOUT_GOLDEN, credit_blocked=0), (
        f"a timeout delivered at the earliest legal moment gave a different "
        f"result from the settled case\n  observed {st}\n"
        f"  expected {dict(TIMEOUT_GOLDEN, credit_blocked=0)}")
    assert len(sock.requests) == 1, "a timed-out probe must not be reissued"
