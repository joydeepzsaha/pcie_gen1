"""Commit 2b-2 integration -- pcie_enum_scan behind a REAL pcie_rq_rc_top (K1..K6).

    scan_start_i -> pcie_enum_scan -> pcie_cfg_txn -> pcie_rq_if -> tlp_layer
                 -> TX DLLP -> [completer] -> RX DLLP -> ... -> status surface

The standalone target owns the phase-dependent policy against an invented
socket.  This one owns what only the real stack can answer: that the two
descriptors the sequencer builds become the TLPs the spec says they should, that
a CRS-first device still completes its probe, and that the scan behaves under
realistic credit rather than saturation.

! FLOW CONTROL.  Nothing is emitted and no error is reported until link_up_i,
transmit_enable_i, fc_initialized_i and one fc_update_valid_i pulse with
non-zero credits are all present -- regression RC1.  The RC1 control itself was
run once, in Commit 2b-1's i8, and is deliberately not repeated.

!! ZERO MEANS INFINITE at FC init (Base 2.1 SS2.6.1 p.138, fn 33 p.137), so
starving a pool takes a small FINITE advertisement that is never replenished.

Spec cited (read, not assumed):
  Device 0 only ..................... PCIe Base 2.1 SS7.3.1 p.479
  Configuration Request header ...... PCIe Base 2.1 SS2.2.7 p.79-80, Figure 2-18
  Minimum FC advertisements ......... PCIe Base 2.1 Table 2-37 p.137-138
  Completion timeout is an error .... PCIe Base 2.1 SS2.8 p.152
  Type 0 header offsets ............. PCIe Base 2.1 Figure 7-5 p.491
RTL cited:
  timer runs from ALLOCATION ........ src/tlp/tlp_request_tracker.sv:39
  credit gate, downstream of it ..... src/tlp/tlp_layer.sv:280
  orphan-data report, once per Dword  src/rc/pcie_rc_if.sv:403-405
"""

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ReadOnly, RisingEdge

from enum_tb_common import (
    BDF, BUS, CLK_NS, CPL_TIMEOUT_CYCLES, DEV, FN, RID,
    DEVICE, HDR_TYPE0, REG0, SCAN_BUS, VENDOR, reg3,
    ENUM_ERR_CA, ENUM_ERR_CRS_EXHAUSTED, ENUM_ERR_NONE,
    ENUM_ERR_TIMEOUT, ENUM_ERR_UR_POST_PROBE, ENUM_ERR_CREDIT_STARVED, err_name,
    CFG_BE_DWORD, CFG_REG_CACHE_HEADER, CFG_REG_VENDOR_DEVICE,
    CPL_CRS, CPL_SC, CPL_UR,
    RC_ERR_ORPHAN_DATA,
    CreditDrip, Mon, TlpRequest,
    assert_cfg_tlp_on_wire, set_credits,
    cfg_wire_dw0, cfg_wire_dw1, cfg_wire_dw2,
    cpl_dw0, cpl_dw1, cpl_dw2, dw0_length,
)


# CLK_NS / CPL_TIMEOUT_CYCLES / RID / BDF / BUS,DEV,FN and the ENUM_ERR_*
# codes now come from enum_tb_common -- all byte-identical across benches, and
# the BAR benches would otherwise have made a third and fourth copy.





DEFAULT_SPACE = {CFG_REG_VENDOR_DEVICE: REG0,
                 CFG_REG_CACHE_HEADER: reg3(HDR_TYPE0)}


# ==========================================================================
# SS THE COMPLETER -- with a real (tiny) configuration space
#
# The four-name interface is PRESERVED VERBATIM -- .start(), .seen,
# .wait_for(n), .complete(req, ...) -- because that is the surface Joy's
# protocol-checking endpoint model is meant to drop into.  serve() is an
# ADDITIONAL convenience for multi-transaction sequences and does not replace
# any of the four.
#
# ⛔ THE DEFAULT ARM IS UR, NOT SILENCE.  A completer that simply ignored a
# register it does not model would drive the sequencer into a completion
# timeout and a sticky ERROR, and it would look exactly like an FSM bug.  Base
# 2.1 SS7.3.3 p.480 is explicit: a Type 0 request that does not address "a valid
# local Configuration Space of an implemented Function" must "follow rules for
# handling Unsupported Requests".  ur_default_hits counts the arm so a test can
# assert it was actually exercised rather than merely present.
# ==========================================================================
class ConfigSpaceCompleter:
    def __init__(self, dut, space=None, crs_once=(), silent_regs=()):
        self.dut = dut
        self.space = dict(DEFAULT_SPACE if space is None else space)
        self.crs_once = set(crs_once)      # answer CRS the first time, then SC
        self.silent_regs = set(silent_regs)  # answer nothing at all
        self.seen = []
        self.ur_default_hits = 0
        self._partial = []
        self._answered = 0

    def start(self):
        cocotb.start_soon(self._watch_tx())

    def serve(self):
        """Auto-answer every observed request from the configuration space."""
        cocotb.start_soon(self._serve())

    async def _watch_tx(self):
        d = self.dut
        while True:
            await RisingEdge(d.clk_i)
            await ReadOnly()
            if int(d.rst_i.value):
                continue
            if int(d.m_dllp_axis_tvalid.value) and int(d.m_dllp_axis_tready.value):
                self._partial.append(int(d.m_dllp_axis_tdata.value))
                if int(d.m_dllp_axis_tlast.value):
                    self.seen.append(TlpRequest(self._partial))
                    self._partial = []

    async def _serve(self):
        while True:
            await RisingEdge(self.dut.clk_i)
            while self._answered < len(self.seen):
                req = self.seen[self._answered]
                self._answered += 1
                if req.reg_num in self.silent_regs:
                    continue                      # deliberate silence -> timeout
                if req.reg_num in self.crs_once:
                    self.crs_once.discard(req.reg_num)
                    await self.complete(req, status=CPL_CRS)
                elif req.reg_num in self.space:
                    await self.complete(req, status=CPL_SC,
                                        data=self.space[req.reg_num])
                else:
                    # SS7.3.3 p.480 -- unimplemented Function/register: UR.
                    self.ur_default_hits += 1
                    await self.complete(req, status=CPL_UR)

    async def wait_for(self, count, cycles=8000):
        for _ in range(cycles):
            await RisingEdge(self.dut.clk_i)
            if len(self.seen) >= count:
                return
        raise AssertionError(
            f"expected {count} request TLPs on the wire, saw {len(self.seen)} "
            f"({self.seen}) -- FC credits, or the scan never issued?")

    async def complete(self, req, status=CPL_SC, data=None, byte_count=None):
        has_data = req.is_read and status == CPL_SC
        if byte_count is None:
            byte_count = 4
        words = [
            cpl_dw0(has_data=has_data, length_dw=1 if has_data else 0),
            cpl_dw1(BDF, status, byte_count=byte_count),
            cpl_dw2(RID, req.tag, lower_address=0),
        ]
        if has_data:
            words.append(0xD0000000 | req.tag if data is None else data)
        await self.inject(words)

    async def inject(self, words):
        d = self.dut
        for index, word in enumerate(words):
            d.s_dllp_axis_tdata.value = word
            d.s_dllp_axis_tkeep.value = 0xF
            d.s_dllp_axis_tlast.value = 1 if index == len(words) - 1 else 0
            d.s_dllp_axis_tvalid.value = 1
            for _ in range(20000):
                await ReadOnly()
                fired = int(d.s_dllp_axis_tready.value) == 1
                await RisingEdge(d.clk_i)
                if fired:
                    break
            else:
                raise AssertionError("s_dllp_axis_tready never asserted -- RX wedged")
        d.s_dllp_axis_tvalid.value = 0
        d.s_dllp_axis_tlast.value = 0


# ==========================================================================
# Monitor
# ==========================================================================
# ==========================================================================
# Flow control + harness
# ==========================================================================
async def init(dut, credits=None, space=None, crs_once=(), silent_regs=(),
               serve=True):
    cocotb.start_soon(Clock(dut.clk_i, CLK_NS, units="ns").start())
    dut.rst_i.value = 1
    dut.link_up_i.value = 0
    dut.transmit_enable_i.value = 0
    dut.scan_start_i.value = 0
    dut.scan_bus_i.value = SCAN_BUS
    dut.s_dllp_axis_tdata.value = 0
    dut.s_dllp_axis_tkeep.value = 0
    dut.s_dllp_axis_tvalid.value = 0
    dut.s_dllp_axis_tlast.value = 0
    dut.s_dllp_axis_tuser.value = 0
    dut.m_dllp_axis_tready.value = 1
    dut.requester_id_i.value = RID
    dut.completer_id_i.value = 0
    dut.bus_number_i.value = 0
    dut.device_number_i.value = 0
    dut.function_number_i.value = 0
    dut.memory_enable_i.value = 1
    dut.extended_tag_enable_i.value = 0
    dut.max_payload_bytes_i.value = 128
    dut.max_read_bytes_i.value = 128
    dut.rcb_128b_i.value = 0
    dut.fc_initialized_i.value = 0
    dut.fc_update_valid_i.value = 0
    set_credits(dut, ph=0, pd=0, nph=0, npd=0, cplh=0, cpld=0)
    for _ in range(4):
        await RisingEdge(dut.clk_i)
    dut.rst_i.value = 0
    dut.link_up_i.value = 1
    dut.transmit_enable_i.value = 1

    set_credits(dut, **(credits or {}))
    dut.fc_initialized_i.value = 1
    dut.fc_update_valid_i.value = 1        # THE INIT STROBE
    await RisingEdge(dut.clk_i)
    await RisingEdge(dut.clk_i)
    if credits is not None:
        dut.fc_update_valid_i.value = 0    # a drip test pulses its own totals
    for _ in range(4):
        await RisingEdge(dut.clk_i)

    mon = Mon(dut)
    mon.start()
    completer = ConfigSpaceCompleter(dut, space=space, crs_once=crs_once,
                                     silent_regs=silent_regs)
    completer.start()
    if serve:
        completer.serve()
    await RisingEdge(dut.clk_i)
    return mon, completer


async def start_scan(dut):
    dut.scan_start_i.value = 1
    await RisingEdge(dut.clk_i)
    dut.scan_start_i.value = 0


async def settle(dut, cycles=40):
    for _ in range(cycles):
        await RisingEdge(dut.clk_i)


async def wait_terminal(dut, cycles=CPL_TIMEOUT_CYCLES + 2000):
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


def assert_on_wire(req, *, reg_num, tag, what=""):
    """2b-2 binding of enum_tb_common.assert_cfg_tlp_on_wire.

    Both scan transactions are whole-Dword READS, so write/first_be are bound
    here rather than passed at each call site.  require_device0 is ON: SS7.3.1
    p.479 device-0-only is THIS bench's subject (SSD.1), which is exactly why the
    shared helper makes it opt-in instead of always-on.
    """
    assert_cfg_tlp_on_wire(req, write=False, reg_num=reg_num,
                           first_be=CFG_BE_DWORD, tag=tag, what=what,
                           require_device0=True)


# ==========================================================================
# K1 -- the whole scan, on the wire
# ==========================================================================
@cocotb.test()
async def k1_full_scan_end_to_end(dut):
    """A device is found and identified, and both TLPs match the SSD.4 goldens."""
    mon, completer = await init(dut)
    await start_scan(dut)
    await wait_terminal(dut)
    st = await status(dut)

    assert st["done"] == 1 and st["error"] == 0, f"scan ended wrong: {st}"
    assert st["present"] == 1 and st["unsupported"] == 0
    assert st["vendor"] == VENDOR and st["device"] == DEVICE, \
        f"IDs {st['vendor']:#06x}/{st['device']:#06x} through the real stack"
    assert st["header_type"] == HDR_TYPE0 and st["mf"] == 0
    assert st["bdf"] == BDF

    # ⭐ EXACTLY TWO TLPs, and both must be device 0. A device-number sweep
    # would emit far more and SS7.3.1 p.479 forbids naming devices 1-31.
    assert len(completer.seen) == 2, (
        f"{len(completer.seen)} TLPs on the wire, expected exactly 2 "
        f"(probe + header type): {completer.seen}")
    probe, hdr = completer.seen
    assert mon.tags_presented == [probe.tag, hdr.tag], (
        f"header tags {[hex(probe.tag), hex(hdr.tag)]} do not match the strobes "
        f"{[hex(t) for t in mon.tags_presented]}")
    assert_on_wire(probe, reg_num=CFG_REG_VENDOR_DEVICE, tag=probe.tag, what="K1 probe ")
    assert_on_wire(hdr, reg_num=CFG_REG_CACHE_HEADER, tag=hdr.tag, what="K1 header ")
    assert completer.ur_default_hits == 0, "nothing should have hit the UR arm"
    await settle(dut)
    assert int(dut.outstanding_o.value) == 0, "a tag was left outstanding"
    mon.clean()


# ==========================================================================
# K2 -- absence through the real stack, and the completer's UR arm
# ==========================================================================
@cocotb.test()
async def k2_absent_device_through_the_stack(dut):
    """An empty configuration space answers UR, and the scan completes clean.

    ⛔ THIS IS THE TEST THAT EXERCISES THE COMPLETER'S UR DEFAULT ARM, and it
    asserts the arm was hit.  A completer whose default was SILENCE would drive
    the scan into a completion timeout and a sticky ERROR that looks exactly
    like an FSM bug -- the trap flagged in Phase 1.
    """
    mon, completer = await init(dut, space={})     # nothing implemented
    await start_scan(dut)
    await wait_terminal(dut)
    st = await status(dut)

    assert completer.ur_default_hits >= 1, (
        "the completer's UR default arm was never exercised, so this test does "
        "not prove what it claims to")
    assert st["done"] == 1, "absence is a NORMAL completion of the scan"
    assert st["error"] == 0, f"absence reported as an error: {st}"
    assert st["present"] == 0
    assert st["vendor"] == 0 and st["device"] == 0
    assert len(completer.seen) == 1, (
        f"{len(completer.seen)} TLPs -- no Header Type read may follow an "
        "absent device")
    mon.clean()


# ==========================================================================
# K3 -- a device that answers CRS first
# ==========================================================================
@cocotb.test()
async def k3_crs_first_device_completes_the_probe(dut):
    """An NVMe-like device stalls its first probe with CRS, then answers.

    Base 2.1 SS2.3.2 Implementation Note p.113: a device may legally answer early
    Configuration Requests with CRS while it initialises, and "the Root Complex
    must re-issue the Configuration Request using a hardware mechanism".  This
    FSM is that mechanism.
    """
    mon, completer = await init(dut, crs_once=(CFG_REG_VENDOR_DEVICE,))
    await start_scan(dut)
    await wait_terminal(dut)
    st = await status(dut)

    assert st["done"] == 1 and st["error"] == 0, f"CRS-first scan ended wrong: {st}"
    assert st["present"] == 1
    assert st["vendor"] == VENDOR and st["device"] == DEVICE

    # Three TLPs: the CRS'd probe, its reissue, and the header read.
    assert len(completer.seen) == 3, (
        f"{len(completer.seen)} TLPs, expected 3 (probe, retry, header): "
        f"{completer.seen}")
    first, retry, hdr = completer.seen
    assert first.reg_num == CFG_REG_VENDOR_DEVICE
    assert retry.reg_num == CFG_REG_VENDOR_DEVICE, "the retry must repeat the probe"
    assert hdr.reg_num == CFG_REG_CACHE_HEADER
    assert retry.dw0 == first.dw0 and retry.dw2 == first.dw2, \
        "the reissued header differs from the original outside the tag"
    # NOTE: retry.tag may legitimately EQUAL first.tag -- the CRS carried
    # Request Completed, which is exactly when PG213 :4257 makes reuse safe.
    # Nothing here asserts anything about tag VALUES.
    assert len(mon.tags_presented) == 3, \
        f"{len(mon.tags_presented)} tag strobes for 3 emitted requests"
    mon.clean()


# ==========================================================================
# K4 -- the Table 2-37 minimum credit, replenished cumulatively
# ==========================================================================
@cocotb.test()
async def k4_small_credit_drip(dut):
    """NPH=1, NPD=1, CPLH/CPLD infinite -- the smallest legal peer.

    Base 2.1 Table 2-37 p.137-138 sets the minimum initial advertisement at one
    NPH unit and makes infinite CPLH/CPLD mandatory for an Endpoint, so this is
    not a stress vector: it is what the enumeration FSM must work against.

    The drip is slower than the transaction rate so the credit really binds;
    blocked_seen asserts that it did, because a small-credit test in which
    nothing ever blocks is a saturated test wearing a costume.
    """
    mon, completer = await init(dut, credits=dict(
        ph=1, pd=8, nph=1, npd=1, cplh=0, cpld=0))
    drip = CreditDrip(dut, nph=1, npd=1, period=40)
    drip.start()

    await start_scan(dut)
    await wait_terminal(dut)
    st = await status(dut)

    assert st["done"] == 1 and st["error"] == 0, \
        f"the scan did not complete under minimum credit: {st}"
    assert st["present"] == 1 and st["vendor"] == VENDOR
    assert len(completer.seen) == 2
    assert mon.blocked_seen, (
        "tx_fc_blocked_o never asserted with NPH=1 and a 40-cycle drip -- the "
        "credit was not binding, so this proves nothing about small credit")
    assert drip.updates > 0, "the drip never ran"
    assert int(dut.outstanding_o.value) == 0
    mon.clean()


# ==========================================================================
# K5 -- FINDING 2, reproduced one layer up
# ==========================================================================
@cocotb.test()
async def k5_credit_starvation_fabricates_a_timeout(dut):
    """Credit starvation past CPL_TIMEOUT_CYCLES ends the scan as a TIMEOUT.

    THE FINDING-2 SIGNATURE (docs/predictions/SPEC_PREDICTIONS_ENUM.md SSD.6), asserted exactly.

    tlp_request_tracker measures per-tag age from ALLOCATION (:39) and
    allocation sits upstream of the credit gate (tlp_requester.sv:138 vs
    tlp_layer.sv:280).  So a request the transmitter never sent still ages out,
    and the sequencer sees a completion timeout indistinguishable from a dead
    device.  err_credit_blocked_o is the only thing that says otherwise.

    Vector: NPH is FINITE and never replenished.  A config read consumes NPH=1,
    so the probe spends the only credit and the Header Type read can never be
    transmitted.
    """
    mon, completer = await init(dut, credits=dict(
        ph=1, pd=8, nph=1, npd=0xFFF, cplh=0, cpld=0))

    await start_scan(dut)
    await completer.wait_for(1)                 # the probe goes out and is answered
    await settle(dut, 200)

    # The header read is now stuck: no NPH credit remains and nothing returns it.
    await ReadOnly()
    assert int(dut.tx_fc_blocked_o.value) == 1, \
        "the Header Type read should be credit-blocked"
    await RisingEdge(dut.clk_i)
    assert len(completer.seen) == 1, (
        f"{len(completer.seen)} TLPs -- the header read was transmitted despite "
        "having no NPH credit, which would invalidate this test")

    await mon.wait_timeouts(1)
    await wait_terminal(dut)
    st = await status(dut)

    assert st["error"] == 1, "a fabricated timeout must still end the scan"
    # sec 63 #7f #19: the code itself now distinguishes them.
    assert st["code"] == ENUM_ERR_CREDIT_STARVED, (
        f"error code {err_name(st['code'])}, expected ENUM_ERR_CREDIT_STARVED for "
        "a timeout the credit gate caused")
    assert st["credit_blocked"] == 1, (
        "err_credit_blocked_o low -- the annotation that accompanies "
        "ENUM_ERR_CREDIT_STARVED did not fire")
    assert st["done"] == 0
    assert st["present"] == 1, "the probe had already succeeded"
    assert len(completer.seen) == 1, (
        "the timed-out request never reached the wire, which is the whole point")

    # A completion timeout is a sideband strobe: nothing on the error surface
    # fires, which is exactly what makes this hard to diagnose in the field.
    assert mon.rq_errors == [] and mon.command_errors == [] and mon.tx_errors == []
    assert mon.credit_errors == 0
    assert mon.unexpected == []


# ==========================================================================
# K6 -- a late completion drains without perturbing the status surface
# ==========================================================================
@cocotb.test()
async def k6_late_completion_does_not_perturb_the_scan(dut):
    """A drained late completion is not a fault, and does not move the surface.

    ⚠️ SCOPE NOTE. The brief asks for "late CPL mid-scan -> scan continues".
    That is UNREACHABLE by construction: late_cpl_valid_o fires only for a tag
    in ZOMBIE quarantine, which requires a completion timeout first, and a
    timeout is terminal for this sequencer (SSD.5).  With one transaction in
    flight at a time there is no second tag to keep going on.

    What IS reachable, and is what matters, is the transparency property: the
    orphan-data burst must not add an error, change the error code, or move any
    status output.  That is asserted here, with the exact per-Dword count.
    """
    mon, completer = await init(dut, silent_regs=(CFG_REG_CACHE_HEADER,))

    await start_scan(dut)
    await completer.wait_for(2)                 # probe answered, header ignored
    hdr = completer.seen[1]

    await mon.wait_timeouts(1)
    assert mon.timeouts == [hdr.tag], (
        f"timeout named {[hex(t) for t in mon.timeouts]}, expected the header "
        f"read's tag {hdr.tag:#04x}")
    await wait_terminal(dut)
    before = await status(dut)
    assert before["error"] == 1 and before["code"] == ENUM_ERR_TIMEOUT

    # The late completion arrives carrying FOUR Dwords against a 1-Dword
    # request: length and per-request accounting are forced apart on purpose.
    late_payload = [0x1111_1111, 0x2222_2222, 0x3333_3333, 0x4444_4444]
    await completer.inject([
        cpl_dw0(has_data=True, length_dw=len(late_payload)),
        cpl_dw1(BDF, CPL_SC, byte_count=4 * len(late_payload)),
        cpl_dw2(RID, hdr.tag, lower_address=0),
    ] + late_payload)

    await mon.wait_lates(1)
    assert mon.lates == [hdr.tag], \
        f"late_cpl named {[hex(t) for t in mon.lates]}, expected {hdr.tag:#04x}"
    await settle(dut, 80)

    assert mon.rc_errors == [RC_ERR_ORPHAN_DATA] * len(late_payload), (
        f"expected {len(late_payload)} orphan-data reports, one per drained "
        f"Dword; got {mon.rc_errors}")
    assert mon.unexpected == [], "a drained late completion is not unexpected"

    after = await status(dut)
    assert after == before, (
        f"the status surface moved when the late completion drained:\n"
        f"  before {before}\n  after  {after}")
    assert len(completer.seen) == 2, "the drain caused a reissue"


# ==========================================================================
# K7 -- a device that never answers its first probe
#
# ADDED IN RESPONSE TO A SURVIVING MUTATION.  Making tx_fc_blocked_i steer the
# PROBE-phase timeout arm passed K1..K6, because both existing timeout tests
# (K5, K6) time out during the HEADER phase -- the integration suite never
# exercised a probe-phase timeout at all.  The standalone suite does (S7, S14,
# S15, S16), but standalone coverage of a path does not excuse an integration
# blind spot on the same path: that asymmetry is exactly what both targets exist
# to measure.
#
# It is also the most realistic hardware failure of the three: a device that is
# electrically present -- the link trained, so link_up_i is asserted -- but
# never answers configuration reads at all.  That is distinct from "absent"
# (which answers UR, SS7.3.1 p.479) and must be reported as a fault.
# ==========================================================================
@cocotb.test()
async def k7_dead_device_times_out_on_the_probe(dut):
    """The link is up but nothing ever answers register 0.

    Base 2.1 SS2.8 p.152 makes a completion timeout a reported error that
    "should never occur under normal operating conditions"; absence answers with
    UR (SS2.3.2 IN p.122).  A silent device is therefore a fault, not an absence,
    and the scan must say so rather than reporting an empty slot.

    Credit is saturated here, so err_credit_blocked_o must stay LOW -- this
    timeout is real, not fabricated by starvation, and the diagnostic has to
    tell the two apart or it is worthless.
    """
    mon, completer = await init(dut, silent_regs=(CFG_REG_VENDOR_DEVICE,))

    await start_scan(dut)
    await completer.wait_for(1)
    probe = completer.seen[0]
    assert_on_wire(probe, reg_num=CFG_REG_VENDOR_DEVICE, tag=probe.tag,
                   what="K7 probe ")

    await mon.wait_timeouts(1)
    assert mon.timeouts == [probe.tag], (
        f"timeout named {[hex(t) for t in mon.timeouts]}, expected the probe's "
        f"tag {probe.tag:#04x}")
    await wait_terminal(dut)
    st = await status(dut)

    assert st["error"] == 1, (
        "a silent device was not reported as a fault -- if this came back done, "
        "a timeout is being treated as absence and the Phase-1 derivation "
        "(SSD.5, SS2.8 p.152) has been inverted")
    assert st["code"] == ENUM_ERR_TIMEOUT, f"error code {st['code']}"
    assert st["done"] == 0
    assert st["present"] == 0, "nothing ever identified itself"
    assert st["vendor"] == 0 and st["device"] == 0
    assert st["credit_blocked"] == 0, (
        "err_credit_blocked_o asserted with saturated credit -- the diagnostic "
        "must distinguish a real timeout from a starvation-fabricated one, and "
        "here there was no starvation")

    assert len(completer.seen) == 1, "a timed-out probe must not be reissued"
    assert mon.rq_errors == [] and mon.command_errors == [] and mon.tx_errors == []
    assert mon.credit_errors == 0 and mon.unexpected == []


# ==========================================================================
# K8 -- a CRS retry starved of credit, timing out IN THE PROBE PHASE
#
# ADDED AFTER K7 FAILED TO KILL THE MUTATION IT WAS WRITTEN FOR.  Making
# tx_fc_blocked_i steer the probe-phase timeout arm still passed K1..K7: K5 and
# K6 time out during the HEADER phase, and K7 times out during the probe but
# with saturated credit, so tx_fc_blocked_i was low and the mutated branch was
# never taken.  The killing combination is a probe-phase timeout WHILE
# credit-blocked, and nothing produced it.
#
# This does: a device that answers CRS, behind a peer advertising the Table 2-37
# minimum of one NPH credit and never returning it.  The first probe attempt
# spends the credit; the CRS retry -- still the probe phase -- can never be
# transmitted, and ages out from allocation.
#
# It is also the nastiest realistic combination in the whole increment: a slow
# device and a stingy peer together, producing a timeout that looks like a dead
# device on a probe that was answered.
# ==========================================================================
@cocotb.test()
async def k8_crs_retry_starved_of_credit_times_out_in_probe(dut):
    """CRS then credit starvation: a probe-phase timeout, correctly annotated.

    A config read consumes NPH=1 (tlp_vc_buffer.sv:91 charges data credits only
    for packets with a payload), so with NPH advertised as a finite 1 and never
    replenished, attempt 2 of the probe is unsendable.  It still gets a tag --
    allocation precedes the credit gate -- and the tracker ages it from
    allocation, so CPL_TIMEOUT_CYCLES later it times out having never been
    transmitted.

    err_credit_blocked_o must be SET here and CLEAR in K7, which is what makes
    it a diagnostic rather than decoration.
    """
    mon, completer = await init(dut,
                               credits=dict(ph=1, pd=8, nph=1, npd=0xFFF,
                                            cplh=0, cpld=0),
                               crs_once=(CFG_REG_VENDOR_DEVICE,))

    await start_scan(dut)
    await completer.wait_for(1)
    first = completer.seen[0]
    assert first.reg_num == CFG_REG_VENDOR_DEVICE, "the first TLP is the probe"

    # The CRS is answered by serve(); the retry then has no NPH credit left.
    await settle(dut, 300)
    await ReadOnly()
    assert int(dut.tx_fc_blocked_o.value) == 1, (
        "the CRS retry should be credit-blocked -- if it is not, the NPH pool "
        "was replenished from somewhere and this test proves nothing")
    await RisingEdge(dut.clk_i)
    assert len(completer.seen) == 1, (
        f"{len(completer.seen)} TLPs -- the retry reached the wire despite "
        "having no NPH credit")

    await mon.wait_timeouts(1)
    await wait_terminal(dut)
    st = await status(dut)

    assert st["error"] == 1, "a starved CRS retry must still end the scan"
    assert st["code"] == ENUM_ERR_CREDIT_STARVED, (
        f"error code {err_name(st['code'])}, expected ENUM_ERR_CREDIT_STARVED "
        "(sec 63 #7f #19)")
    assert st["credit_blocked"] == 1, (
        "err_credit_blocked_o low on a timeout that WAS credit-starved -- "
        "compare K7, where it must be low because the device was simply dead. "
        "If it does not distinguish the two, it is not a diagnostic")
    assert st["done"] == 0, "a timeout is terminal, and it is not success"
    assert st["present"] == 0, "the probe never returned an ID -- only a CRS"
    assert len(completer.seen) == 1
    assert mon.rq_errors == [] and mon.command_errors == [] and mon.tx_errors == []
    assert mon.credit_errors == 0
