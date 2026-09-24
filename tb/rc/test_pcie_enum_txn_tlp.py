"""Commit 2b-1 integration -- pcie_cfg_txn behind a REAL pcie_rq_rc_top (I1..I8).

    cmd_* -> pcie_cfg_txn -> pcie_rq_if -> tlp_layer -> TX DLLP -> [completer]
    [completer] -> RX DLLP -> tlp_layer -> pcie_rc_if -> pcie_cfg_txn -> rsp_*

The standalone target owns the primitive's behaviour against an invented
socket.  This one owns what only the real stack can answer: that the descriptor
the primitive builds becomes the TLP the spec says it should, that the tag it
latches is the one the tracker allocated and put in the header, that the
timeout it reacts to is the tracker's own, and that everything still works when
credit is scarce rather than saturated.

! FLOW CONTROL.  The DUT emits nothing and reports NO error until link_up_i,
transmit_enable_i, fc_initialized_i and one fc_update_valid_i pulse with
non-zero credits are all present (tlp_layer.sv:249, tlp_credit_manager.sv:53-54,
66-83).  Every "N packets" assertion here would otherwise be vacuously
satisfied by silence -- regression RC1, which I8 reproduces deliberately once.

!! ZERO MEANS INFINITE.  An advertisement of 00h/000h made AT FC INITIALISATION
means INFINITE credit for that type (Base 2.1 SS2.6.1 p.138, footnote 33 p.137;
tlp_credit_manager.sv:106-120).  Starving a pool therefore needs a small FINITE
advertisement with no replenishment -- never a zero one.  I7 pins that
semantics so the inverted test cannot be rebuilt; I6 is the real starvation.

Spec cited (read, not assumed):
  Configuration Request header ...... PCIe Base 2.1 SS2.2.7 p.79-80, Figure 2-18
  Minimum FC advertisements ......... PCIe Base 2.1 Table 2-37 p.137-138
  Zero at init means infinite ....... PCIe Base 2.1 SS2.6.1 p.138, fn 33 p.137
  Completion timeout is an error .... PCIe Base 2.1 SS2.8 p.152
  Bit 30 / Request Completed ........ PG213 v1.3 :4049
RTL cited:
  DW0 assembly ...................... src/tlp/tlp_generator.sv, the dw0 assembly
  DW1 = {rid, tag, last_be, first_be}  src/tlp/tlp_generator.sv, the dw1 assembly
  config DW2 = {address[31:2], 00} .. src/tlp/tlp_generator.sv, the dw2 assembly
  orphan-data report, once per Dword   src/rc/pcie_rc_if.sv:403-405
"""

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ReadOnly, RisingEdge

from enum_tb_common import (
    BDF, BUS, CLK_NS, CPL_TIMEOUT_CYCLES, DEV, FN, RID,
    CFG_BE_DWORD, CFG_BE_LOWER_HALF,
    CFG_REG_BAR0, CFG_REG_CACHE_HEADER, CFG_REG_COMMAND_STATUS,
    CFG_REG_VENDOR_DEVICE,
    CPL_CRS, CPL_SC, CPL_UR,
    RC_ERR_ORPHAN_DATA,
    TXN_NAME, TXN_OK, TXN_TIMEOUT, TXN_UR,
    CreditDrip, Mon, TlpRequest,
    assert_cfg_tlp_on_wire, outcome_name, set_credits,
    cfg_wire_dw0, cfg_wire_dw1, cfg_wire_dw2,
    cpl_dw0, cpl_dw1, cpl_dw2, dw0_length,
)

CRS_RETRY_MAX = 3            # tb_pcie_enum_txn_tlp.sv override

# CLK_NS / CPL_TIMEOUT_CYCLES / RID / BDF / BUS,DEV,FN now come from
# enum_tb_common -- they were byte-identical in all four enum benches.


# ==========================================================================
# SS THE COMPLETER
#
# Watches the DLL-facing TX stream, parses each emitted request far enough to
# know its tag and whether it wants data, and injects a matching Cpl/CplD on RX.
#
# The four-name interface is PRESERVED VERBATIM from test_pcie_rq_rc_top.py --
# .start(), .seen, .wait_for(n), .complete(req, ...) -- because that is the
# surface Joy's protocol-checking endpoint model is meant to drop into.  Keeping
# it identical makes the handoff a page of documentation instead of a redesign.
#
# It checks NOTHING about the request: it is a stimulus source, not a
# verification model.  Every assertion in this file is made by the test against
# a hand-derived golden, never by the completer against the DUT.
# ==========================================================================
class ConfigCompleter:
    def __init__(self, dut, requester_id=RID, completer_id=BDF):
        self.dut = dut
        self.requester_id = requester_id
        self.completer_id = completer_id
        self.seen = []
        self._partial = []

    def start(self):
        cocotb.start_soon(self._watch_tx())

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

    async def wait_for(self, count, cycles=6000):
        for _ in range(cycles):
            await RisingEdge(self.dut.clk_i)
            if len(self.seen) >= count:
                return
        raise AssertionError(
            f"expected {count} request TLPs on the wire, saw {len(self.seen)} "
            f"({self.seen}) -- FC credits, or the primitive never issued?")

    async def complete(self, req, status=CPL_SC, data=None, byte_count=None):
        has_data = req.is_read and status == CPL_SC
        if byte_count is None:
            byte_count = 4
        words = [
            cpl_dw0(has_data=has_data, length_dw=1 if has_data else 0),
            cpl_dw1(self.completer_id, status, byte_count=byte_count),
            # Lower Address is 0 for every non-Memory-Read completion and the
            # tracker requires exactly that (tlp_layer.sv:371-378).
            cpl_dw2(self.requester_id, req.tag, lower_address=0),
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
# Monitor -- every error surface, concurrently
# ==========================================================================
# ==========================================================================
# Flow control
# ==========================================================================
async def init(dut, credits=None, fc_init=True):
    """Bring the link up.  `credits` overrides the saturated default."""
    cocotb.start_soon(Clock(dut.clk_i, CLK_NS, units="ns").start())
    dut.rst_i.value = 1
    dut.link_up_i.value = 0
    dut.transmit_enable_i.value = 0
    dut.cmd_valid_i.value = 0
    dut.cmd_write_i.value = 0
    dut.cmd_type1_i.value = 0
    dut.cmd_bdf_i.value = 0
    dut.cmd_reg_num_i.value = 0
    dut.cmd_ext_reg_i.value = 0
    dut.cmd_first_be_i.value = 0
    dut.cmd_wdata_i.value = 0
    dut.rsp_ready_i.value = 0
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

    if fc_init:
        set_credits(dut, **(credits or {}))
        dut.fc_initialized_i.value = 1
        dut.fc_update_valid_i.value = 1     # THE INIT STROBE
        await RisingEdge(dut.clk_i)
        await RisingEdge(dut.clk_i)
        # Held high for the saturated default (constant limit, plenty of room);
        # a drip test lowers it and pulses its own increasing totals.
        if credits is not None:
            dut.fc_update_valid_i.value = 0
    for _ in range(4):
        await RisingEdge(dut.clk_i)

    mon = Mon(dut)
    mon.start()
    completer = ConfigCompleter(dut)
    completer.start()
    await RisingEdge(dut.clk_i)
    return mon, completer


# ==========================================================================
# Command / response helpers
# ==========================================================================
async def send_cmd(dut, write, reg_num, first_be=CFG_BE_DWORD, wdata=0,
                   bdf=BDF, ext_reg=0, limit=2000, type1=False):
    dut.cmd_write_i.value = 1 if write else 0
    dut.cmd_type1_i.value = 1 if type1 else 0
    dut.cmd_bdf_i.value = bdf
    dut.cmd_reg_num_i.value = reg_num
    dut.cmd_ext_reg_i.value = ext_reg
    dut.cmd_first_be_i.value = first_be
    dut.cmd_wdata_i.value = wdata
    dut.cmd_valid_i.value = 1
    for _ in range(limit):
        await ReadOnly()
        fired = int(dut.cmd_ready_o.value) == 1
        await RisingEdge(dut.clk_i)
        if fired:
            break
    else:
        raise AssertionError("cmd_ready_o never asserted -- primitive stuck")
    dut.cmd_valid_i.value = 0


async def recv_rsp(dut, cycles=CPL_TIMEOUT_CYCLES + 2000):
    for _ in range(cycles):
        await ReadOnly()
        if int(dut.rsp_valid_o.value):
            got = {
                "outcome": int(dut.rsp_outcome_o.value),
                "rdata": int(dut.rsp_rdata_o.value),
                "status_raw": int(dut.rsp_status_raw_o.value),
                "crs_retries": int(dut.crs_retries_o.value),
            }
            await RisingEdge(dut.clk_i)
            dut.rsp_ready_i.value = 1
            await RisingEdge(dut.clk_i)
            dut.rsp_ready_i.value = 0
            return got
        await RisingEdge(dut.clk_i)
    raise AssertionError("rsp_valid_o never asserted")


async def settle(dut, cycles=40):
    for _ in range(cycles):
        await RisingEdge(dut.clk_i)




def assert_on_wire(req, *, write, reg_num, first_be, tag, what="", type1=False):
    """2b-1 binding of enum_tb_common.assert_cfg_tlp_on_wire.

    The goldens and the five assertions are shared; only the choice of which
    optional properties this bench owns is local.  require_device0 stays OFF
    here: device-0-only is SS7.3.1's property and the presence scan's subject,
    not pcie_cfg_txn's, and turning it on would silently widen what these
    tests assert.  type1 (Stage D) passes through to the CFG1 DW0 golden.
    """
    assert_cfg_tlp_on_wire(req, write=write, reg_num=reg_num,
                           first_be=first_be, tag=tag, what=what, type1=type1)


# ==========================================================================
# I1 / I2 -- end to end, with the emitted TLP asserted on the wire
# ==========================================================================
@cocotb.test()
async def i1_config_read_end_to_end(dut):
    """A CfgRd0 all the way to the link and back, header asserted against spec."""
    mon, completer = await init(dut)

    await send_cmd(dut, write=False, reg_num=CFG_REG_VENDOR_DEVICE,
                   first_be=CFG_BE_DWORD)
    await completer.wait_for(1)
    req = completer.seen[0]

    # The tag in the header must be the tag the socket presented -- the whole
    # point of pcie_rq_tag_o is that it is the value that physically went out.
    assert mon.tags_presented == [req.tag], (
        f"header tag {req.tag:#04x} but pcie_rq_tag_o presented "
        f"{[hex(t) for t in mon.tags_presented]}")
    assert_on_wire(req, write=False, reg_num=CFG_REG_VENDOR_DEVICE,
                   first_be=CFG_BE_DWORD, tag=req.tag, what="I1 ")
    assert req.payload == [], "a CfgRd0 carries no payload"
    assert int(dut.outstanding_o.value) == 1

    await completer.complete(req, status=CPL_SC, data=0x8086_A80A)
    rsp = await recv_rsp(dut)

    assert rsp["outcome"] == TXN_OK, \
        f"outcome {outcome_name(rsp['outcome'])}, expected TXN_OK"
    assert rsp["rdata"] == 0x8086_A80A, f"read data {rsp['rdata']:#010x}"
    await settle(dut)
    assert int(dut.outstanding_o.value) == 0, "the tag was not retired"
    assert len(completer.seen) == 1, f"{len(completer.seen)} TLPs for one command"
    mon.clean()


@cocotb.test()
async def i2_config_write_end_to_end(dut):
    """A byte-granular CfgWr0 all the way to the link and back.

    first_be = 0011 with reg 1 is the Command register's low half -- the shape
    Commit 2b-3 will use for Memory Space + Bus Master enable.  It is also the
    case that proves byte-granular config writes survive the whole stack.
    """
    mon, completer = await init(dut)

    await send_cmd(dut, write=True, reg_num=CFG_REG_COMMAND_STATUS,
                   first_be=CFG_BE_LOWER_HALF, wdata=0x00000006)
    await completer.wait_for(1)
    req = completer.seen[0]

    assert_on_wire(req, write=True, reg_num=CFG_REG_COMMAND_STATUS,
                   first_be=CFG_BE_LOWER_HALF, tag=req.tag, what="I2 ")
    assert req.payload == [0x00000006], \
        f"payload {[hex(w) for w in req.payload]}, expected [0x6]"
    assert req.first_be == CFG_BE_LOWER_HALF, (
        f"1st DW BE {req.first_be:#06b} != {CFG_BE_LOWER_HALF:#06b} -- the byte "
        "enables did not survive to the wire")

    # A config write is NON-POSTED: it takes a tag and it does get a completion.
    assert mon.tags_presented == [req.tag], \
        "a CfgWr0 must present a tag -- only posted writes do not"

    await completer.complete(req, status=CPL_SC)
    rsp = await recv_rsp(dut)
    assert rsp["outcome"] == TXN_OK, \
        f"outcome {outcome_name(rsp['outcome'])}, expected TXN_OK"
    assert rsp["rdata"] == 0, "a config write completion carries no data"
    mon.clean()


# ==========================================================================
# I3 -- CRS retry through the real stack
# ==========================================================================
@cocotb.test()
async def i3_crs_retry_through_the_stack(dut):
    """CRS then SC, with both TLPs asserted on the wire.

    The reissue must be byte-identical in its header EXCEPT for the tag: a CRS
    terminates the request (Base 2.1 SS2.3.2 IN p.113), so the retry is a new
    request and the tracker gives it a new tag.
    """
    mon, completer = await init(dut)

    await send_cmd(dut, write=False, reg_num=CFG_REG_VENDOR_DEVICE)
    await completer.wait_for(1)
    first = completer.seen[0]
    await completer.complete(first, status=CPL_CRS)

    await completer.wait_for(2)
    retry = completer.seen[1]

    assert retry.dw0 == first.dw0, "the reissued DW0 differs"
    assert retry.dw2 == first.dw2, "the reissued routing Dword differs"
    assert retry.first_be == first.first_be, "the reissued byte enables differ"
    # !! THE RETRY MAY LEGITIMATELY REUSE THE SAME TAG, AND HERE IT DOES.
    #
    # The standalone bench hands out incrementing tags, so E7 can assert that
    # the retry's tag differs.  That is a property of THAT MODEL, not of the
    # design: the real tracker recycles.  The CRS completion carried Request
    # Completed, which is exactly the condition under which PG213 :4257 makes
    # reuse safe -- "the user logic should not reassign a tag allocated to a
    # request until it has received a Completion Descriptor ... with the
    # Request Completed bit set".  Once that has happened the tag is free, and
    # the tracker hands the lowest free tag straight back.
    #
    # Asserting tag inequality here would be asserting a bench artefact.  What
    # must hold is that the reissue went through ALLOCATION again rather than
    # reusing a stale latched value -- and the second strobe is what proves it.
    assert mon.tags_presented == [first.tag, retry.tag], (
        f"tag strobes {[hex(t) for t in mon.tags_presented]} do not match the "
        f"two headers {[hex(first.tag), hex(retry.tag)]}")
    assert len(mon.tags_presented) == 2, (
        f"{len(mon.tags_presented)} tag strobes for two emitted requests -- the "
        "retry did not go through allocation")
    assert_on_wire(retry, write=False, reg_num=CFG_REG_VENDOR_DEVICE,
                   first_be=CFG_BE_DWORD, tag=retry.tag, what="I3 retry ")

    await completer.complete(retry, status=CPL_SC, data=0x1B36_1AF4)
    rsp = await recv_rsp(dut)

    assert rsp["outcome"] == TXN_OK, \
        f"outcome {outcome_name(rsp['outcome'])}, expected TXN_OK"
    assert rsp["rdata"] == 0x1B36_1AF4
    assert rsp["crs_retries"] == 1, f"crs_retries_o = {rsp['crs_retries']}"
    assert len(completer.seen) == 2, f"{len(completer.seen)} TLPs, expected 2"
    mon.clean()


# ==========================================================================
# I4 -- the real completion timeout, then a late completion
# ==========================================================================
@cocotb.test()
async def i4_timeout_then_late_completion(dut):
    """Nobody answers; the tracker times out; the late completion drains clean.

    This is the whole timeout story through the real stack:
      * the primitive reports TXN_TIMEOUT off the tracker's own strobe;
      * a late completion for the quarantined tag raises late_cpl_valid_o and
        exactly one RC_ERR_ORPHAN_DATA per drained Dword
        (pcie_rc_if.sv:403-405) -- and NEITHER is a fault;
      * nothing wedges: a fresh transaction still round-trips afterwards.

    The orphan count is asserted EXACTLY, V9-style.  "No failure" would pass
    even if the drain sized itself from the request instead of from the
    completion and left three beats stuck in the receive path.
    """
    mon, completer = await init(dut)

    await send_cmd(dut, write=False, reg_num=CFG_REG_VENDOR_DEVICE)
    await completer.wait_for(1)
    dead = completer.seen[0]
    assert mon.tags_presented == [dead.tag]

    await mon.wait_timeouts(1)
    assert mon.timeouts == [dead.tag], (
        f"timeout named {[hex(t) for t in mon.timeouts]}, but the request went "
        f"out on tag {dead.tag:#04x}")

    rsp = await recv_rsp(dut)
    assert rsp["outcome"] == TXN_TIMEOUT, \
        f"outcome {outcome_name(rsp['outcome'])}, expected TXN_TIMEOUT"
    assert rsp["rdata"] == 0

    # The late completion arrives, carrying FOUR Dwords against a 1-Dword
    # request: length and per-request accounting are forced apart on purpose.
    late_payload = [0x1111_1111, 0x2222_2222, 0x3333_3333, 0x4444_4444]
    await completer.inject([
        cpl_dw0(has_data=True, length_dw=len(late_payload)),
        cpl_dw1(BDF, CPL_SC, byte_count=4 * len(late_payload)),
        cpl_dw2(RID, dead.tag, lower_address=0),
    ] + late_payload)

    await mon.wait_lates(1)
    assert mon.lates == [dead.tag], \
        f"late_cpl named {[hex(t) for t in mon.lates]}, expected {dead.tag:#04x}"
    await settle(dut, 80)
    assert mon.rc_errors == [RC_ERR_ORPHAN_DATA] * len(late_payload), (
        f"expected {len(late_payload)} orphan-data reports, one per drained "
        f"Dword; got {mon.rc_errors}")
    assert mon.unexpected == [], "a drained late completion is not unexpected"
    assert int(dut.rsp_valid_o.value) == 0, \
        "the drained late completion produced a response"
    mon.rc_errors.clear()

    # Nothing wedged.
    await send_cmd(dut, write=False, reg_num=CFG_REG_BAR0)
    await completer.wait_for(2)
    live = completer.seen[1]
    await completer.complete(live, status=CPL_SC, data=0x5EED_0003)
    rsp2 = await recv_rsp(dut)
    assert rsp2["outcome"] == TXN_OK, "the stack did not recover from the drain"
    assert rsp2["rdata"] == 0x5EED_0003
    assert len(mon.timeouts) == 1, "the second request was answered in time"
    mon.clean(allow_timeouts=True, allow_orphans=True)


# ==========================================================================
# I5 -- realistic small credit, with a cumulative drip
# ==========================================================================
@cocotb.test()
async def i5_small_credit_drip(dut):
    """NPH=1, NPD=1, CPLH/CPLD infinite -- the Table 2-37 minimum, replenished.

    Base 2.1 Table 2-37 p.137-138 sets the minimum initial advertisement at one
    NPH unit and one NPD unit, and makes infinite CPLH/CPLD MANDATORY for an
    Endpoint.  So this vector is not a stress choice, it is the smallest legal
    peer -- and it is what the enumeration FSM must work against.

    The drip is deliberately slower than the transaction rate so the credit
    really does bind; `blocked_seen` asserts that it did, because a small-credit
    test in which nothing ever blocks is a saturated test wearing a costume.
    """
    mon, completer = await init(dut, credits=dict(
        ph=1, pd=8, nph=1, npd=1, cplh=0, cpld=0))
    drip = CreditDrip(dut, nph=1, npd=1, period=40)
    drip.start()

    # A mix of reads and writes: a read consumes NPH only, a write NPH and NPD.
    plan = [(False, CFG_REG_VENDOR_DEVICE, 0),
            (True,  CFG_REG_BAR0, 0xFFFFFFFF),
            (False, CFG_REG_BAR0, 0),
            (True,  CFG_REG_COMMAND_STATUS, 0x00000006),
            (False, CFG_REG_CACHE_HEADER, 0)]
    for index, (write, reg, wdata) in enumerate(plan):
        await send_cmd(dut, write=write, reg_num=reg, wdata=wdata,
                       first_be=CFG_BE_DWORD)
        await completer.wait_for(index + 1)
        await completer.complete(completer.seen[index], status=CPL_SC,
                                 data=0xA000_0000 | index)
        rsp = await recv_rsp(dut)
        assert rsp["outcome"] == TXN_OK, (
            f"transaction {index} ({'write' if write else 'read'} reg {reg:#04x}) "
            f"ended {outcome_name(rsp['outcome'])} under minimum credit")
        if not write:
            assert rsp["rdata"] == (0xA000_0000 | index)

    assert len(completer.seen) == len(plan), \
        f"{len(completer.seen)} TLPs for {len(plan)} commands"
    assert mon.blocked_seen, (
        "tx_fc_blocked_o never asserted with NPH=1/NPD=1 and a 40-cycle drip -- "
        "the credit was not actually binding, so this test proves nothing about "
        "small credit")
    assert drip.updates > 0, "the drip never ran"
    # Single outstanding by construction: with NPH=1 there is never room for
    # two, and the primitive never tries.
    assert int(dut.outstanding_o.value) == 0
    mon.clean()


# ==========================================================================
# I6 -- P-NPD1-STALL: the REAL credit starvation signature
# ==========================================================================
@cocotb.test()
async def i6_finite_npd_starves_writes_not_reads(dut):
    """A finite NPD with no replenishment stalls writes indefinitely, silently.

    THE PREDICTED SIGNATURE (docs/predictions/SPEC_PREDICTIONS_ENUM.md SS2.4, P-NPD1-STALL):
    tx_fc_blocked_o held, ZERO error strobes anywhere, no TLP on the wire, and
    the primitive simply waiting.  It is indistinguishable from a hung FSM
    unless you know to look at tx_fc_blocked_o -- which is exactly why it is
    written down here rather than discovered during 2b-2.

    A config READ consumes NPH only (tlp_vc_buffer.sv:91 charges data credits
    only when the packet has data), so reads keep flowing while writes starve.
    That asymmetry is the diagnostic.
    """
    mon, completer = await init(dut, credits=dict(
        ph=1, pd=8, nph=0xFF, npd=1, cplh=0, cpld=0))

    # 1. The one available NPD credit is spent by a write.
    await send_cmd(dut, write=True, reg_num=CFG_REG_BAR0, wdata=0xFFFFFFFF)
    await completer.wait_for(1)
    await completer.complete(completer.seen[0], status=CPL_SC)
    rsp = await recv_rsp(dut)
    assert rsp["outcome"] == TXN_OK, "the first write should have had credit"

    # 2. A READ still flows: it needs no data credit at all.
    await send_cmd(dut, write=False, reg_num=CFG_REG_VENDOR_DEVICE)
    await completer.wait_for(2)
    await completer.complete(completer.seen[1], status=CPL_SC, data=0x1234_5678)
    rsp = await recv_rsp(dut)
    assert rsp["outcome"] == TXN_OK, (
        "a config READ stalled under NPD starvation -- reads consume NPH only")
    assert rsp["rdata"] == 0x1234_5678

    # 3. A second WRITE has no data credit and must stall, silently.
    await send_cmd(dut, write=True, reg_num=CFG_REG_BAR0, wdata=0xFFFFFFFF)
    await settle(dut, 400)
    assert len(completer.seen) == 2, (
        f"{len(completer.seen)} TLPs on the wire -- a write was transmitted with "
        "no NPD credit available")
    await ReadOnly()
    assert int(dut.tx_fc_blocked_o.value) == 1, (
        "tx_fc_blocked_o is low while a write is stalled -- the stall is not "
        "credit, so it is a real hang")
    assert int(dut.rsp_valid_o.value) == 0, "no response is possible yet"
    await RisingEdge(dut.clk_i)
    mon.clean()   # THE POINT: a credit stall raises NO error, of any kind.

    # 4. Replenish -- cumulatively -- and it completes.
    dut.fc_npd_i.value = 2
    dut.fc_update_valid_i.value = 1
    await RisingEdge(dut.clk_i)
    dut.fc_update_valid_i.value = 0
    await completer.wait_for(3)
    await completer.complete(completer.seen[2], status=CPL_SC)
    rsp = await recv_rsp(dut)
    assert rsp["outcome"] == TXN_OK, \
        "the stalled write did not complete after credit was returned"
    mon.clean()


# ==========================================================================
# I7 -- P-NPD-INF: zero at init means INFINITE
# ==========================================================================
@cocotb.test()
async def i7_zero_advertisement_means_infinite(dut):
    """fc_npd_i = 0 at FC init makes NPD UNLIMITED, not empty.

    Base 2.1 SS2.6.1 p.138 and footnote 33 p.137: an advertisement of 00h/000h
    made at Flow Control initialisation is "interpreted as infinite by the
    Transmitter, which will, therefore, never throttle".  Implemented at
    tlp_credit_manager.sv:106-120, latched at init and never re-evaluated.

    THIS TEST EXISTS TO PREVENT A TEST.  The obvious way to write a credit
    starvation case -- advertise zero and watch the writes wedge -- produces
    the exact opposite behaviour and passes while proving nothing.  Both the
    Commit-2b brief and docs/recon/RECON_commit2b.md SS2.3 predicted the wedge before this
    was checked against the RTL.  I6 is the real starvation; this is the vacuum,
    pinned so nobody rebuilds the inverted version.
    """
    mon, completer = await init(dut, credits=dict(
        ph=1, pd=8, nph=0xFF, npd=0, cplh=0, cpld=0))

    # Four writes, each consuming a data credit from a pool advertised as zero.
    for index in range(4):
        await send_cmd(dut, write=True, reg_num=CFG_REG_BAR0, wdata=0xFFFF_0000 | index)
        await completer.wait_for(index + 1)
        assert completer.seen[index].payload == [0xFFFF_0000 | index]
        await completer.complete(completer.seen[index], status=CPL_SC)
        rsp = await recv_rsp(dut)
        assert rsp["outcome"] == TXN_OK, (
            f"write {index} ended {outcome_name(rsp['outcome'])} -- if this "
            "stalled, zero-means-infinite has regressed and every credit test "
            "in the suite needs re-reading")

    assert not mon.blocked_seen, (
        "tx_fc_blocked_o asserted with NPD advertised as 0 (= INFINITE). Either "
        "the credit manager stopped honouring Base 2.1 SS2.6.1 p.138 fn 33, or a "
        "different pool ran out")
    assert len(completer.seen) == 4
    mon.clean()


# ==========================================================================
# I8 -- RC1 negative control. Once, and never again.
# ==========================================================================
@cocotb.test()
async def i8_rc1_no_flow_control_init_is_silent(dut):
    """Without FC init the stack emits NOTHING and reports NOTHING.

    The recognisable RC1 signature.  The dangerous part is the second half: the
    RQ interface still accepts the descriptor and no error output ever fires, so
    the failure looks exactly like a broken primitive.

    !! THE TEST MUST PROVE THE PRIMITIVE TRIED (prediction F4).  With no FSM
    activity at all, "zero TLPs" is trivially true and the control is vacuous --
    so s_axis_rq_tvalid being seen high is asserted BEFORE the silence is.
    """
    mon, completer = await init(dut, fc_init=False)

    await send_cmd(dut, write=False, reg_num=CFG_REG_VENDOR_DEVICE)
    await settle(dut, 400)

    assert mon.rq_tvalid_seen, (
        "the primitive never drove s_axis_rq_tvalid -- this control proves "
        "nothing about flow control if nothing was ever offered to the stack")
    assert completer.seen == [], (
        f"{len(completer.seen)} TLPs emitted without FC initialisation")
    assert int(dut.rsp_valid_o.value) == 0, "a response appeared from nowhere"

    # !! MEASURED CORRECTION TO THE DOCUMENTED SOCKET CONTRACT.
    #
    # pcie_rq_rc_top.sv:33 lists the RC1 signature as "no TLP on m_dllp_axis_*,
    # no pulse on any error output, NO TAG ON pcie_rq_tag_o".  The last clause
    # is WRONG, and this is where it was caught: a tag IS allocated and IS
    # presented, with flow control uninitialised.
    #
    # The reason is structural.  The credit gate sits at the VC-buffer-to-
    # transmit boundary -- vc_packet_ready = credit_request_ready &&
    # transmit_enable_i && link_up_i (tlp_layer.sv:280) -- which is DOWNSTREAM
    # of allocation.  tlp_requester enters REQ_TAG as soon as the command is
    # accepted and raises tag_request_valid_o there (tlp_requester.sv:138,
    # :211), referencing neither fc_initialized_i nor any credit signal.  So the
    # tag is handed out, the TLP is assembled, and only then does it park in the
    # VC buffer with nothing to spend.
    #
    # CONSEQUENCE FOR THE 2b-2 SEQUENCER: a tag strobe is NOT evidence that a
    # request reached the link.  pcie_cfg_txn never treats it as such -- it
    # waits for a completion or the timeout, never for the tag alone -- and the
    # sequencer must not either.
    assert len(mon.tags_presented) == 1, (
        f"expected exactly one tag strobe, saw {mon.tags_presented}. A tag IS "
        "allocated without credit (tlp_requester.sv:138 vs tlp_layer.sv:280); "
        "if this count changed, the allocation/credit ordering moved")

    # ...and the silence really is total: no error output names the problem.
    mon.clean()
    assert mon.rc_errors == [], f"RC errors: {mon.rc_errors}"

    # Now bring flow control up and the very same command completes, which is
    # what proves the silence was FC and not a broken primitive.
    set_credits(dut)
    dut.fc_initialized_i.value = 1
    dut.fc_update_valid_i.value = 1
    await completer.wait_for(1)
    req = completer.seen[0]
    assert_on_wire(req, write=False, reg_num=CFG_REG_VENDOR_DEVICE,
                   first_be=CFG_BE_DWORD, tag=req.tag, what="I8 post-init ")
    await completer.complete(req, status=CPL_SC, data=0x1AF4_1000)
    rsp = await recv_rsp(dut)
    assert rsp["outcome"] == TXN_OK, \
        "the command did not complete once flow control came up"
    assert rsp["rdata"] == 0x1AF4_1000
    mon.clean()


# ==========================================================================
# I9 -- the completion timer runs from ALLOCATION, not from transmission
#
# ADDED AFTER AN INHERITED-STACK SURPRISE FOUND BY I8.  Tag allocation happens
# upstream of the credit gate, and tlp_request_tracker.sv:39 measures per-tag
# age "from ALLOCATION".  Those two facts together mean a request stalled on
# credit for longer than CPL_TIMEOUT_CYCLES times out WITHOUT EVER HAVING BEEN
# TRANSMITTED.
#
# That matters because the Commit-2b master brief SS4.1 requires the enumeration
# FSM to tolerate tx_fc_blocked_o "for arbitrary spans -- no cycle-count
# assumptions". Against this stack that requirement cannot be met above ~4096
# cycles of continuous credit starvation, no matter how the FSM is written: the
# tracker will fire a timeout the FSM has no way to distinguish from a dead
# device.  Pinned here so the limit is a known quantity rather than a field
# failure.
# ==========================================================================
@cocotb.test()
async def i9_credit_stall_longer_than_the_timeout(dut):
    """A write starved of credit past CPL_TIMEOUT_CYCLES times out un-transmitted.

    PREDICTED, before running: the second write is accepted, its tag is
    allocated immediately (I8 established that allocation does not wait for
    credit), the TLP parks in the VC buffer, and CPL_TIMEOUT_CYCLES later the
    tracker times out a request that never reached the link.  No TLP appears on
    the wire during the stall, and no error output fires -- a completion timeout
    is a sideband strobe, not an error.
    """
    mon, completer = await init(dut, credits=dict(
        ph=1, pd=8, nph=0xFF, npd=1, cplh=0, cpld=0))

    # Spend the single NPD credit.
    await send_cmd(dut, write=True, reg_num=CFG_REG_BAR0, wdata=0xFFFFFFFF)
    await completer.wait_for(1)
    await completer.complete(completer.seen[0], status=CPL_SC)
    rsp = await recv_rsp(dut)
    assert rsp["outcome"] == TXN_OK

    # This one has no data credit and will never be transmitted.
    await send_cmd(dut, write=True, reg_num=CFG_REG_BAR0, wdata=0xDEADBEEF)
    await settle(dut, 200)
    await ReadOnly()
    assert int(dut.tx_fc_blocked_o.value) == 1, "the write should be credit-blocked"
    await RisingEdge(dut.clk_i)
    tags_at_stall = list(mon.tags_presented)
    assert len(tags_at_stall) == 2, (
        f"expected a tag to be allocated for the stalled write, saw "
        f"{tags_at_stall} -- allocation is supposed to precede the credit gate")

    # Ride out the whole timeout interval with the TLP still stuck.
    await mon.wait_timeouts(1, cycles=CPL_TIMEOUT_CYCLES + 800)
    assert mon.timeouts == [tags_at_stall[1]], (
        f"timeout named {[hex(t) for t in mon.timeouts]}, expected the stalled "
        f"write's tag {tags_at_stall[1]:#04x}")
    assert len(completer.seen) == 1, (
        f"{len(completer.seen)} TLPs on the wire -- the stalled write was "
        "transmitted after all, which would invalidate this whole test")

    rsp = await recv_rsp(dut)
    assert rsp["outcome"] == TXN_TIMEOUT, (
        f"outcome {outcome_name(rsp['outcome'])} -- a credit-starved request "
        "that outlives CPL_TIMEOUT_CYCLES is reported as a completion timeout, "
        "indistinguishable from a dead device")

    # A completion timeout is a sideband strobe; nothing on the error surface
    # fires, which is precisely what makes this hard to diagnose in the field.
    assert mon.rq_errors == [] and mon.command_errors == [] and mon.tx_errors == []
    assert mon.credit_errors == 0


# ==========================================================================
# I10 / I11 -- Stage D increment 1: Type 1 through the REAL stack
#
# The standalone half (E15..E17) proves the primitive's descriptor; these two
# prove the descriptor becomes the CFG1 TLP the spec says -- dw0[4:0] = 00101,
# nothing else moved (Base 2.1 Table 2-3 p.58, SS2.2.7 p.79).  Post-D-2 the
# RQ surface admits req_type 1001/1011, so a clean error surface is part of
# the assertion.  Structurally non-falsifiable pre-change (no port); the
# mutation kills are recorded in the commit message.
# ==========================================================================
@cocotb.test()
async def i10_cfg1_end_to_end_on_wire(dut):
    """CfgRd1 and CfgWr1 to the link and back, whole-DW0 goldens (Trap A)."""
    mon, completer = await init(dut)

    await send_cmd(dut, write=False, reg_num=CFG_REG_VENDOR_DEVICE,
                   first_be=CFG_BE_DWORD, type1=True)
    await completer.wait_for(1)
    rd = completer.seen[0]
    assert_on_wire(rd, write=False, reg_num=CFG_REG_VENDOR_DEVICE,
                   first_be=CFG_BE_DWORD, tag=rd.tag, type1=True,
                   what="I10 CfgRd1 ")
    assert rd.tlp_type == 0b00101, \
        f"tlp_type {rd.tlp_type:#07b} != 00101 (CfgRd1, Table 2-3 p.58)"
    assert rd.payload == [], "a CfgRd1 carries no payload"
    await completer.complete(rd, status=CPL_SC, data=0x1017_15B3)
    rsp = await recv_rsp(dut)
    assert rsp["outcome"] == TXN_OK
    assert rsp["rdata"] == 0x1017_15B3

    await send_cmd(dut, write=True, reg_num=CFG_REG_COMMAND_STATUS,
                   first_be=CFG_BE_LOWER_HALF, wdata=0x00000006, type1=True)
    await completer.wait_for(2)
    wr = completer.seen[1]
    assert_on_wire(wr, write=True, reg_num=CFG_REG_COMMAND_STATUS,
                   first_be=CFG_BE_LOWER_HALF, tag=wr.tag, type1=True,
                   what="I10 CfgWr1 ")
    assert wr.payload == [0x00000006], \
        f"payload {[hex(w) for w in wr.payload]}, expected [0x6]"
    await completer.complete(wr, status=CPL_SC)
    rsp = await recv_rsp(dut)
    assert rsp["outcome"] == TXN_OK
    mon.clean()


@cocotb.test()
async def i11_crs_retry_preserves_type1_on_wire(dut):
    """A CRS'd CfgRd1's reissue is still a CfgRd1 on the wire (P6.3).

    The whole-DW0 compare between the two emitted TLPs is the discriminating
    check: a retry path that decayed to Type 0 emits a well-formed CfgRd0 that
    every field-subset assertion accepts (Trap A) -- only dw0 bit 0 tells.
    """
    mon, completer = await init(dut)

    await send_cmd(dut, write=False, reg_num=CFG_REG_VENDOR_DEVICE, type1=True)
    dut.cmd_type1_i.value = 0        # the retry must come from the latch
    await completer.wait_for(1)
    first = completer.seen[0]
    assert first.tlp_type == 0b00101
    await completer.complete(first, status=CPL_CRS)

    await completer.wait_for(2)
    retry = completer.seen[1]
    assert retry.dw0 == first.dw0, (
        f"retry DW0 {retry.dw0:#010x} != original {first.dw0:#010x} -- the "
        "CRS reissue changed the header, and if bit 0 is the difference it "
        "decayed to Type 0")
    assert retry.tlp_type == 0b00101, \
        f"retry tlp_type {retry.tlp_type:#07b} -- the reissue decayed to Type 0"
    assert retry.dw2 == first.dw2, "the reissued routing Dword differs"
    assert_on_wire(retry, write=False, reg_num=CFG_REG_VENDOR_DEVICE,
                   first_be=CFG_BE_DWORD, tag=retry.tag, type1=True,
                   what="I11 retry ")

    await completer.complete(retry, status=CPL_SC, data=0x1AF4_1100)
    rsp = await recv_rsp(dut)
    assert rsp["outcome"] == TXN_OK
    assert rsp["rdata"] == 0x1AF4_1100
    assert rsp["crs_retries"] == 1
    mon.clean()


# ==========================================================================
# I12 -- §63 #7g-2 step 3 (Kourosh Q1): the Completion Timeout runs from the
# SEND, not from the tag allocation.
#
# Base 2.1 §2.8 p.152: the Completion Timeout mechanism "is activated for each
# Request that requires one or more Completions when the Request is
# transmitted".  The tracker measured age from ALLOCATION, which precedes the
# credit gate, so a request that waited g cycles for credit and was then sent
# had only CPL_TIMEOUT_CYCLES - g left from its transmission (FINDINGS_7G2_
# PHASE1.md §3.3: no constant bounds g).  Q1 (REFINED): the timer starts at
# allocation and RESTARTS at the TL->DLL handoff, so every transmitted request
# gets its full interval from the send.
#
# ⚠️ What this row does NOT change: a request that is NEVER sent is still
# aborted CPL_TIMEOUT_CYCLES after allocation -- that is PROJECT POLICY, not a
# §2.8 clause (§2.8 governs transmitted requests only), and it is what keeps
# #19's ENUM_ERR_CREDIT_STARVED reachable.  i9, k5, k8 and e4 pin that half.
# ==========================================================================
I12_STALL_CYCLES = 2000
"""How long the request waits for data credit before it is sent.  Below
CPL_TIMEOUT_CYCLES, so the allocation-relative abort cannot fire first, and far
above the handoff's own few cycles, so the two start events are unmistakable."""


@cocotb.test(expect_fail=True)  # §63 #7g-2 R-C1: RED until the Q1 fix; pinned (§22.93)
async def i12_stalled_then_sent_request_gets_full_interval_from_the_send(dut):
    """A CfgWr0 stalled I12_STALL_CYCLES for data credit, then sent and never
    answered, times out AT LEAST CPL_TIMEOUT_CYCLES after its TLP's first beat
    left the Transaction Layer -- and within one tag-scan period plus a few
    cycles of that.

    Raw per-cycle capture (§22.92): the allocation strobe (pcie_rq_tag_vld_o),
    the first beat of every TLP at the TL's output (m_dllp_axis, the TL -> DLL
    seam of this bench), and the timeout strobe.  Paired after the run.

    NON-VACUITY, inside the guard: exactly two TLPs reach the wire; the second
    is the stalled write and it left >= I12_STALL_CYCLES after its tag was
    allocated; tx_fc_blocked_o was high during the stall; the timeout names
    that write's tag; the primitive reports TXN_TIMEOUT for it.

    ⚠️⚠️ RED WHEN WRITTEN (tree d711bd0), predicted in PREDICTIONS_7G2_CPL.md:
    the timeout fires CPL_TIMEOUT_CYCLES after ALLOCATION, i.e. ~2,100 cycles
    after the send.  Pinned: it may fail ONLY at the assertion after REACHED.
    """
    ROW = "i12_stalled_then_sent_request_gets_full_interval_from_the_send"
    try:
        credits = dict(ph=1, pd=8, nph=0xFF, npd=1, cplh=0, cpld=0)
        mon, completer = await init(dut, credits=credits)
        cap = {"alloc": [], "first": [], "timeout": [], "blocked": []}
        stop = [False]

        async def capture():
            n, in_pkt = 0, False
            while not stop[0]:
                await RisingEdge(dut.clk_i)
                await ReadOnly()
                n += 1
                if int(dut.pcie_rq_tag_vld_o.value):
                    cap["alloc"].append((n, int(dut.pcie_rq_tag_o.value)))
                if int(dut.m_dllp_axis_tvalid.value) and int(dut.m_dllp_axis_tready.value):
                    if not in_pkt:
                        cap["first"].append(n)
                    in_pkt = not int(dut.m_dllp_axis_tlast.value)
                if int(dut.cpl_timeout_valid_o.value):
                    cap["timeout"].append((n, int(dut.cpl_timeout_tag_o.value)))
                if int(dut.tx_fc_blocked_o.value):
                    cap["blocked"].append(n)

        task = cocotb.start_soon(capture())

        # Spend the single NPD credit on an answered write.
        await send_cmd(dut, write=True, reg_num=CFG_REG_BAR0, wdata=0xFFFFFFFF)
        await completer.wait_for(1)
        await completer.complete(completer.seen[0], status=CPL_SC)
        rsp = await recv_rsp(dut)
        assert rsp["outcome"] == TXN_OK, "the credit-spending write must complete"

        # The write under test: no data credit, so it is allocated and parked.
        await send_cmd(dut, write=True, reg_num=CFG_REG_BAR0, wdata=0xDEADBEEF)
        for _ in range(200):
            await RisingEdge(dut.clk_i)
            if len(cap["alloc"]) >= 2:
                break
        assert len(cap["alloc"]) == 2, f"the stalled write's tag was never allocated: {cap['alloc']}"
        await settle(dut, I12_STALL_CYCLES)
        assert len(completer.seen) == 1, "the stalled write reached the wire during the stall"

        # Return one NPD credit, cumulatively (CreditDrip's discipline): 1 -> 2.
        set_credits(dut, **dict(credits, npd=2))
        dut.fc_update_valid_i.value = 1
        await RisingEdge(dut.clk_i)
        dut.fc_update_valid_i.value = 0
        await completer.wait_for(2)          # it is sent now; nobody answers
        await mon.wait_timeouts(1, cycles=CPL_TIMEOUT_CYCLES + 900)
        rsp = await recv_rsp(dut)
        stop[0] = True
        await task

        alloc_n, tag = cap["alloc"][1]
        assert len(cap["first"]) == 2, f"expected 2 TLPs at the TL output, saw {cap['first']}"
        sent_n = cap["first"][1]
        fire_n, fire_tag = cap["timeout"][0]
        dut._log.info("I12: stalled write tag %#04x allocated @%d, first beat left the TL @%d "
                      "(stall %d), timeout @%d = %d after the send, %d after allocation; "
                      "blocked cycles %d",
                      tag, alloc_n, sent_n, sent_n - alloc_n, fire_n, fire_n - sent_n,
                      fire_n - alloc_n, len(cap["blocked"]))
        assert sent_n - alloc_n >= I12_STALL_CYCLES, (
            f"NON-VACUITY: the write left the TL {sent_n - alloc_n} cycles after allocation, "
            f"not after the {I12_STALL_CYCLES}-cycle stall")
        assert any(alloc_n < b < sent_n for b in cap["blocked"]), (
            "NON-VACUITY: tx_fc_blocked_o never rose during the stall")
        assert fire_tag == tag, f"the timeout named tag {fire_tag:#04x}, not {tag:#04x}"
        assert rsp["outcome"] == TXN_TIMEOUT, (
            f"outcome {outcome_name(rsp['outcome'])}; an unanswered sent request times out")
        from_send = fire_n - sent_n
    except Exception as exc:  # §22.93 expect_fail hygiene
        dut._log.info("PINNED_RED|%s|NOT_REACHED|%r", ROW, exc)
        dut._log.error("row failed BEFORE its pinned assertion: %r -- returning normally "
                       "so expect_fail reports a gate FAIL", exc)
        return
    dut._log.info("PINNED_RED|%s|REACHED|from_send=%d", ROW, from_send)
    # THE ONE PINNED ASSERTION: the full interval, measured from the send.
    assert CPL_TIMEOUT_CYCLES <= from_send <= CPL_TIMEOUT_CYCLES + 8 + 8, (
        f"the Completion Timeout fired {from_send} cycles after the request was sent; "
        f"Base 2.1 §2.8 p.152 starts it 'when the Request is transmitted', so it must fire "
        f"in [{CPL_TIMEOUT_CYCLES}, {CPL_TIMEOUT_CYCLES + 16}] (one TAG_COUNT=8 scan + hops)")
