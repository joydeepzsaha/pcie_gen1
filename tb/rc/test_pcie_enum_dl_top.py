"""pcie_enum_dl_top -- a whole bus enumeration through the REAL data link layer.

The first target in which NO PYTHON SITS BETWEEN THE ENUMERATOR AND THE WIRE.
pcie_enum_top issues the configuration requests; pcie_rc_dl_top frames them,
numbers them and LCRCs them; the credit that gates them came from a real InitFC
exchange rather than a bench assignment.  The Python that remains is the far
end -- the device being enumerated -- which is where a model belongs.

Everything is observed on m_phy_axis / s_phy_axis and the top's own status
ports.  The only exceptions are the verification-only aliases in the wrapper
(tb_pcie_enum_dl_top.sv), and since the start-gate rung there is exactly one
reason left for them: the PG213 socket is internal by design, so Mon needs
three observation points.  The FC seam used to be the second reason -- it was
reached hierarchically because pcie_rc_dl_top's FC-init filter register
(pcie_rc_dl_top.sv:181) was not a port.  It is a port now (fc_init_done_o), the
gate lives in the DUT, and dut.fc_initialized_o is kept only as an alias of it,
under the old name, because the shared initialize_flow_control helper reads it
by that name.

Design record, predictions and the scored recon:
  ~/pcie_docs/evidence/enum-stack/DESIGN_ENUM_STACK_TOP.md
  ~/pcie_docs/evidence/enum-stack/RECON_REFRESH_588f634.md
  ~/pcie_docs/evidence/enum-stack/PREDICTIONS_ENUM_DL.md

Spec cited (read, not assumed):
  FC init completes once per link-up ... PCIe Base 2.1 SS3.3.1 p.160
  Minimum FC advertisements .......... PCIe Base 2.1 Table 2-37 p.137-138
  Configuration Request header ....... PCIe Base 2.1 SS2.2.7 p.79-80
  Completion timeout is an error ..... PCIe Base 2.1 SS2.8 p.152
  Ack/Nak and replay ................. PCIe Base 2.1 SS3.5-3.6
RTL cited:
  header Dword byte mapping .......... src/tlp/tlp_generator.sv:62-81, :124
  the conditional swap, DW1-3 only ... src/tlp/tlp_generator.sv:97-101
  3DW headers skip TX_DW3 ............ src/tlp/tlp_generator.sv:182-186
  RX is symmetric (DW0 raw) .......... src/tlp/tlp_parser.sv:56-61, :142
  the FC-init filter, now a port ..... src/rc/pcie_rc_dl_top.sv:181
  the start gate itself .............. src/rc/pcie_enum_dl_top.sv, start_pending_r
  S_IDLE re-samples every cycle ...... src/rc/pcie_enum_scan.sv:346
  timer runs from ALLOCATION ......... src/tlp/tlp_request_tracker.sv:39
"""

import cocotb
from cocotb.triggers import ReadOnly, RisingEdge
from cocotb.utils import get_sim_time

from cocotbext.pcie.core.dllp import DllpType
from cocotbext.pcie.core.tlp import Tlp, TlpType

from test_pcie_endpoint_top import (
    MIN_CREDIT_EP,
    PHY_USER_IS_DLLP,
    PHY_USER_IS_TLP,
    add_sequence_and_lcrc,
    build_fc_dllp,
    initialize_flow_control,
    send_axis,
)
from test_pcie_rc_dl_top import RcDlTB
from test_pcie_enum_bar_tlp import (
    ACCEPT_BAR_SIZE, BarSpaceCompleter, acceptance_device,
    assert_acceptance_outcome, assert_command_last, assert_rom_untouched,
    status,
)
from test_pcie_enum_bridge_tlp import DIRECT_GOLDEN_SEQUENCE, on_wire, render
from enum_tb_common import (
    BAR_MEM64, CFG_BE_DWORD, CFG_REG_BAR0, CFG_REG_COMMAND_STATUS,
    CFG_REG_VENDOR_DEVICE, CMD_ENABLE_VALUE, ENUM_ERR_NONE, ENUM_ERR_TIMEOUT,
    HDR_TYPE0, RID, SCAN_BUS,
    BarSpec, ConfigDevice, Mon, TlpRequest,
    assert_sequence, cfg_wire_dw0, cfg_wire_dw1, cfg_wire_dw2, err_name,
    expect_count, nonempty,
)

import zlib

# A configuration request and a completion are both THREE-Dword headers.
CFG_HDR_DW = 3

# The target device's BDF, matching enum_tb_common's completer conventions.
TARGET_BDF = 0x0100


# ==========================================================================
# SS THE FRAME SHIM -- the single largest risk item in this rung
#
# ⭐ THE TRANSFORM IS NON-UNIFORM, AND IT IS HEADER-RELATIVE, NOT INDEX-RELATIVE.
#
# The TL's Dwords are not all laid out the same way on the wire, and a uniform
# byte-swap corrupts the ones that are not.  From the RTL, not from a document:
#
#   * DW0 is BUILT byte-mapped -- "byte N at dw0[8N+7:8N]", tlp_generator.sv:70,
#     the mapping every other field in that block uses -- and is emitted
#     UNSWAPPED (:124, m_axis_tdata = dw0).
#   * DW1..DW3 are FIELD-mapped (requester_id in the high bits) and ARE swapped
#     when PCIE_WIRE_ORDER=1 (:97-101), which pcie_rc_dl_top hard-wires (:197).
#   * The payload passes through formatted_data with NO swap (:141, :143).
#
# ⛔ AND DW3 IS NOT A HEADER DWORD HERE.  tlp_generator.sv:182-186:
#
#       TX_DW2: if (output_fire) begin
#         if (tlp_is_4dw(header_r.fmt))  state_r <= TX_DW3;
#         else                           state_r <= TX_PAYLOAD_START;
#
# A configuration request is a 3DW header, so TX_DW3 is NEVER ENTERED and beat
# 3 is the first PAYLOAD Dword -- little-endian.  Both source documents for this
# rung publish a table reading "DW1...DW3 big-endian" AND "payload (DW3+ of a
# 3DW header) little-endian", which cannot both hold at index 3.  The RTL
# settles it and this is what the RTL says.  Recorded in
# RECON_REFRESH_588f634.md SS2.3 before a line of this file was written.
#
# !! WHY IT MATTERS RATHER THAN BEING PEDANTRY.  0xFFFFFFFF is a FIXED POINT of
# a byte swap, and eleven of the seventeen golden rows carry it or no payload at
# all.  Getting the rule wrong therefore breaks exactly three rows -- B5
# (0x80000000 -> 0x00000080), B6 and B15 (0x00000006 -> 0x06000000) -- which
# reads precisely like a BAR-assignment FSM bug and not like a bench bug.
#
# The RX direction is symmetric and needs no second rule: tlp_parser.sv:142
# reads DW0 from raw s_axis_tdata while :165-206 read the swapped header_dw,
# and the payload memory is unswapped.  One rule, inverted.
#
# Byte order within a beat is little-endian, which is this bench family's AXIS
# convention -- test_pcie_endpoint_top.py:90, word_bytes().
# ==========================================================================
def _beat_is_big_endian(index, header_dw):
    """True iff Dword `index` is a field-mapped header Dword (hence swapped)."""
    return 1 <= index < header_dw


def dwords_from_body(body, header_dw=CFG_HDR_DW):
    """TLP body bytes off the data link layer -> the Dword list TlpRequest wants.

    `body` is what RcDlTB.recv_tlp_frame() returns: the frame with the two-byte
    sequence number and the four-byte LCRC already stripped, LCRC verified.
    """
    assert len(body) % 4 == 0, f"TLP body is not a whole number of Dwords: {len(body)}"
    return [
        int.from_bytes(
            body[n:n + 4],
            "big" if _beat_is_big_endian(n // 4, header_dw) else "little")
        for n in range(0, len(body), 4)
    ]


def body_from_dwords(words, header_dw=CFG_HDR_DW):
    """The inverse: a Dword list -> TLP body bytes for injection.

    Same rule, inverted -- see the RX symmetry note above.
    """
    return b"".join(
        int(w).to_bytes(
            4, "big" if _beat_is_big_endian(index, header_dw) else "little")
        for index, w in enumerate(words)
    )


def request_from_body(body):
    """One observed frame body as the TlpRequest the goldens are written over."""
    return TlpRequest(dwords_from_body(body))


# ==========================================================================
# SS THE SELF-TEST -- runs at import, BEFORE ANY DUT EXISTS
#
# House style: enum_tb_common._selftest_type1_one_bit:383 and
# _selftest_bridged_topology:1637.  A shim whose guard has never been seen
# firing is not known to work.
#
# ⭐ TWO INDEPENDENT DECODERS, ONE WIRE.  cocotbext.pcie's Tlp.unpack is already
# green on this exact frame body in the 46/324 gate -- test_pcie_rc_dl_top.py
# applies it at :290, :457, :518, :548, :606 and :626 -- so it is an oracle this
# rung did not write and cannot accidentally agree with.
#
# ⛔ THE PAYLOADS ARE CHOSEN TO BE DISCRIMINATING.  0xFFFFFFFF is a fixed point
# of a byte swap and proves nothing on its own.  0x80000000 and CMD_ENABLE_VALUE
# (0x00000006) are the two values that separate the correct rule from the
# published one, so they are the ones asserted.
# ==========================================================================
def _selftest_frame_shim():
    cases = [
        ("CfgRd0 vendor/device", False, CFG_REG_VENDOR_DEVICE, CFG_BE_DWORD, None),
        ("CfgWr0 BAR0 sizing (swap fixed point)", True, CFG_REG_BAR0,
         CFG_BE_DWORD, 0xFFFFFFFF),
        ("CfgWr0 BAR0 assign (DISCRIMINATING)", True, CFG_REG_BAR0,
         CFG_BE_DWORD, 0x80000000),
        ("CfgWr0 Command enable (DISCRIMINATING)", True, CFG_REG_COMMAND_STATUS,
         0b0011, CMD_ENABLE_VALUE),
    ]

    for what, write, reg, first_be, payload in cases:
        tag = 0x21
        words = [
            cfg_wire_dw0(write),
            cfg_wire_dw1(RID, tag, first_be),
            cfg_wire_dw2(SCAN_BUS, 0, 0, reg),
        ]
        if payload is not None:
            words.append(payload)

        body = body_from_dwords(words)

        # -- direction 1: our encoder vs. the independent decoder -------------
        tlp = Tlp.unpack(body)
        assert tlp.fmt_type == (TlpType.CFG_WRITE_0 if write
                                else TlpType.CFG_READ_0), (
            f"{what}: Tlp.unpack read fmt_type {tlp.fmt_type}, expected a "
            f"Cfg{'Wr' if write else 'Rd'}0 -- the DW0 rule is wrong")
        assert int(tlp.requester_id) == RID, (
            f"{what}: Requester ID {int(tlp.requester_id):#06x} != {RID:#06x} "
            "-- the DW1 rule is wrong")
        assert tlp.tag == tag, f"{what}: tag {tlp.tag:#04x} != {tag:#04x}"
        assert tlp.first_be == first_be, (
            f"{what}: first_be {tlp.first_be:#06b} != {first_be:#06b}")
        assert int(tlp.completer_id) == TARGET_BDF, (
            f"{what}: completer BDF {int(tlp.completer_id):#06x} != "
            f"{TARGET_BDF:#06x} -- the DW2 rule is wrong")
        assert tlp.address == reg * 4, (
            f"{what}: register address {tlp.address:#x} != {reg * 4:#x}")
        if payload is None:
            assert not bytes(tlp.data), f"{what}: a read carries payload"
        else:
            assert bytes(tlp.data) == payload.to_bytes(4, "little"), (
                f"{what}: Tlp.unpack sees payload {bytes(tlp.data).hex()}, "
                f"expected {payload.to_bytes(4, 'little').hex()} -- ⛔ THIS IS "
                "THE DW3 RULE.  A uniform big-endian rule for DW1..DW3 lands "
                "here and nowhere else.")

        # -- direction 2: round trip through our own pair ---------------------
        assert dwords_from_body(body) == words, (
            f"{what}: dwords_from_body(body_from_dwords(w)) != w\n"
            f"  in  {[hex(x) for x in words]}\n"
            f"  out {[hex(x) for x in dwords_from_body(body)]}")

        # -- direction 3: the field view the GOLDENS actually compare ---------
        req = request_from_body(body)
        assert on_wire(req) == (0, SCAN_BUS, write, reg, first_be, payload), (
            f"{what}: on_wire() gives {on_wire(req)}, expected "
            f"{(0, SCAN_BUS, write, reg, first_be, payload)}")

    # -- and that a full frame survives the sequence+LCRC wrapper -------------
    words = [cfg_wire_dw0(True), cfg_wire_dw1(RID, 0x05, CFG_BE_DWORD),
             cfg_wire_dw2(SCAN_BUS, 0, 0, CFG_REG_BAR0), 0x80000000]
    frame = add_sequence_and_lcrc(0x123, body_from_dwords(words))
    assert int.from_bytes(frame[:2], "big") & 0xFFF == 0x123
    assert int.from_bytes(frame[-4:], "little") == (
        zlib.crc32(frame[:-4]) & 0xFFFFFFFF), "self-test frame LCRC does not close"
    assert dwords_from_body(frame[2:-4]) == words, (
        "a framed body did not survive strip -> transform")


_selftest_frame_shim()


# ==========================================================================
# SS THE HARNESS
# ==========================================================================
class EnumDlTB(RcDlTB):
    """RcDlTB's clock, PHY streams and DLL helpers, plus the enum surface.

    Everything DUT-facing that already existed is inherited unchanged:
    recv_tlp_frame() (skips DLLP frames, verifies LCRC), ack(), nak() and
    send_tlp() (adds sequence + LCRC).  Only reset() is overridden, because
    this top's surface is not pcie_rc_dl_top's: the PG213 RQ/RC socket is gone
    (it is the internal seam) and the enumeration control ports are new.
    """

    async def reset(self, scan_bus=SCAN_BUS, bar_enable=1, bridge_enable=0):
        d = self.dut
        d.rst_i.value = 1
        d.phy_link_up_i.value = 0
        d.idle_valid_i.value = 0
        d.transmit_enable_i.value = 0

        # A Root Complex's Requester ID is its own BDF and stays a top-level
        # input; enum's bus assignment does NOT feed back here
        # (pcie_rc_dl_top.sv:17-20).  RID is what enum_tb_common's completer
        # addresses its completions to, so the two must agree or every
        # completion would be classified unexpected.
        d.requester_id_i.value = RID
        d.completer_id_i.value = 0x0000
        d.bus_number_i.value = 0
        d.device_number_i.value = 0
        d.function_number_i.value = 0
        d.memory_enable_i.value = 1
        d.extended_tag_enable_i.value = 0
        d.max_payload_bytes_i.value = 128
        d.max_read_bytes_i.value = 128
        d.rcb_128b_i.value = 0

        # Enumeration control.  scan_start_i is a LEVEL-triggered pulse the
        # integrator sequences -- this module does not gate it, and the reason
        # is the whole subject of the start-gate negative control.
        d.scan_start_i.value = 0
        d.scan_bus_i.value = scan_bus
        d.bar_enable_i.value = bar_enable
        d.bridge_enable_i.value = bridge_enable

        for _ in range(8):
            await RisingEdge(d.clk_i)
        d.rst_i.value = 0
        d.phy_link_up_i.value = 1
        d.idle_valid_i.value = 1
        d.transmit_enable_i.value = 1
        for _ in range(8):
            await RisingEdge(d.clk_i)

    async def start_enum(self):
        """One scan_start_i pulse.

        A PULSE, deliberately, and it is now load-bearing in a way it was not
        before: the DUT's start gate has to REMEMBER a request made while the
        gate is shut, and a pulse is the stimulus that can tell a latch from a
        bare AND.  A level-held start would pass either implementation.
        test_start_gate_rtl depends on this staying a pulse.
        """
        self.dut.scan_start_i.value = 1
        await RisingEdge(self.dut.clk_i)
        self.dut.scan_start_i.value = 0
        await RisingEdge(self.dut.clk_i)

    async def wait_fc_init(self, cycles=4000):
        """Bounded, EXPLICIT re-check of the Transaction Layer's view of FC init.

        The rule is Base 2.1 SS3.3.1 p.160: FC initialisation completes once per
        link-up and a transmitter holds no credit until it does.
        fc_initialized_o is the wrapper's alias for the DUT port fc_init_done_o
        -- the filtered signal that drives u_rc.fc_initialized_i, NOT the DLL's
        raw, glitching output.

        ⚠️ THIS CALL HAS NEVER BEEN WHAT APPLIES THE GATE, and it is now doubly
        so.  The mutation census (M7) swapped it with start_enum() and ALL FIVE
        TESTS STILL PASSED, because initialize_flow_control() already ends with
        `await wait_high(dut.clk_i, dut.fc_initialized_o)`
        (test_pcie_endpoint_top.py:170) -- by the time it returns, FC init has
        completed and no reordering of the two lines after it is observable.

        Since the start-gate rung the gate is in the RTL, so this call is not
        even the bench's last line of defence: starting early is now SAFE, and
        test_start_gate_rtl proves it by doing exactly that on purpose.  The
        call is kept as an explicit restatement of a precondition at the site a
        reader looks for it, and for its bounded-wait diagnostic when the
        InitFC exchange genuinely fails.  It is NOT load-bearing.
        """
        for _ in range(cycles):
            await RisingEdge(self.dut.clk_i)
            if self.dut.fc_initialized_o.value.is_resolvable and \
                    int(self.dut.fc_initialized_o.value):
                return
        raise AssertionError(
            f"fc_init_done_o did not assert within {cycles} cycles -- the "
            "InitFC exchange never completed, so no TLP could ever be sent")


class PhyCompleter(BarSpaceCompleter):
    """BarSpaceCompleter with its two DUT-facing methods moved to the DLL boundary.

    ⭐ EVERYTHING ELSE TRANSFERS UNCHANGED, and that is the point of the split
    enum_tb_common describes at :1552-1554: the policy is pure.  _serve(),
    _answer() -- with its silent / UR-injected / CRS-once / write / read arms --
    complete(), wait_for(), start(), and the whole ConfigDevice write-mask model
    with real BAR sizing semantics are inherited verbatim.  Only the two methods
    that touch a bus are replaced:

        _watch_tx : m_dllp_axis, one Dword per beat  ->  m_phy_axis frames
        inject    : s_dllp_axis, one Dword per beat  ->  send_tlp (seq + LCRC)

    !! THE PROMPT ACK IS LOAD-BEARING, not politeness.  An unAcked TLP is
    replayed after the replay timer and the replay is a second frame on the
    wire; every frame-counting assertion in this file would then be off by one
    or more.  test_pcie_rc_dl_top.py:150-155 records the same finding.

    ⭐ AND IT RETURNS NON-POSTED CREDIT, because a completion does not.
    Every one of the seventeen configuration requests is NON-POSTED, so each
    consumes one NPH.  A CplD frees the far end's buffer but advertises
    nothing: it is the UpdateFC-NP DLLP that returns the credit
    (SS2.6.1.2 p.141, and test_pcie_rc_dl_top.py:465-470 does exactly this).
    Under MIN_CREDIT_EP the pool is ONE, so without this the enumeration
    deadlocks after the first request and the test would fail for a bench
    reason wearing an RTL costume.  The advertisement is CUMULATIVE and must
    increase; zero at init means infinite, so it starts from the profile's own
    value and counts up from there.

    The credit is returned AFTER the completion, not before, which is both
    legal and what makes the credit stall observable rather than raced away.
    """

    def __init__(self, tb, nph_initial=32, nak_at=None, credit_return_delay=0,
                 **kwargs):
        super().__init__(tb.dut, **kwargs)
        self.tb = tb
        self.frames = []          # raw framed bytes, for the byte-identical NAK check
        self.replays = []         # frames dropped as duplicates of an earlier sequence
        self.nak_at = nak_at      # frame index to NAK instead of Ack
        self.naks_sent = 0
        self.completions_sent = 0
        self.credit_return_delay = credit_return_delay
        self._nph_cum = nph_initial
        self._rx_next_seq = 0

    async def _watch_tx(self):
        while True:
            seq, body, frame = await self.tb.recv_tlp_frame()

            # A DLL receiver discards a duplicate sequence number rather than
            # handing it up twice (Base 2.1 SS3.6).  Modelling that here is what
            # makes "the enumerator sees ONE completion" a real property of the
            # NAK test rather than an artefact of the bench never replaying.
            if seq != self._rx_next_seq:
                self.replays.append(frame)
                await self.tb.ack((self._rx_next_seq - 1) & 0xFFF)
                continue

            index = len(self.frames)
            self.frames.append(frame)
            self._rx_next_seq = (seq + 1) & 0xFFF
            self.seen.append(request_from_body(body))

            if self.nak_at is not None and index == self.nak_at:
                # Force one replay: report the PREVIOUS sequence as the last
                # good one.  The frame has already been accepted above, so the
                # enumeration proceeds and the replay arrives as a duplicate.
                self.naks_sent += 1
                await self.tb.nak((seq - 1) & 0xFFF)
            else:
                await self.tb.ack(seq)

    async def inject(self, words):
        await self.tb.send_tlp(body_from_dwords(words))
        self.completions_sent += 1
        # credit_return_delay: how long the far end holds the request buffer
        # before advertising it again.  Zero is the prompt case; see the
        # measured finding in test_full_enumeration_min_credit's docstring for
        # why the credit-gated test does NOT use zero.
        for _ in range(self.credit_return_delay):
            await RisingEdge(self.tb.dut.clk_i)
        self._nph_cum = (self._nph_cum + 1) & 0xFF
        await send_axis(
            self.tb.phy_source,
            build_fc_dllp(DllpType.UPDATE_FC_NP, hdr_fc=self._nph_cum,
                          data_fc=self._nph_cum),
            PHY_USER_IS_DLLP,
        )


async def settle(dut, cycles=40):
    for _ in range(cycles):
        await RisingEdge(dut.clk_i)


async def wait_frames(dut, completer, count, cycles=20000):
    """Bounded wait on frames observed at the DLL boundary."""
    for _ in range(cycles):
        await RisingEdge(dut.clk_i)
        if len(completer.frames) >= count:
            return
    raise AssertionError(
        f"expected {count} TLP frames on m_phy_axis, saw "
        f"{len(completer.frames)} after {cycles} cycles -- FC credits, or the "
        "sequence never issued?")


async def wait_enum(dut, cycles=60000):
    """Bounded wait for the enumeration to terminate, either way."""
    for _ in range(cycles):
        await RisingEdge(dut.clk_i)
        await ReadOnly()
        if int(dut.enum_done_o.value) or int(dut.enum_error_o.value):
            return
    raise AssertionError(
        f"neither enum_done_o nor enum_error_o asserted within {cycles} cycles")


async def bring_up(dut, credits=None, **completer_kwargs):
    """Reset, InitFC, far end attached, then start.

    Returns (tb, completer, mon).

    ⚠️ THE ORDERING HERE IS NO LONGER THE GATE, and that is the change the
    start-gate rung made.  This helper used to sequence InitFC before the start
    because it HAD to -- the FC-init filter (pcie_rc_dl_top.sv:181) was not on
    that module's port list, so the DUT could not gate itself and the
    integrator had to.  The DUT gates
    itself now (pcie_enum_dl_top's start_pending_r), so this ordering is merely
    the natural one, not a correctness requirement.  test_start_gate_rtl
    deliberately inverts it and still enumerates.

    Keeping the order means every other test in this file exercises the gate's
    TRANSPARENT path -- start arrives after FC init, the latch never sets --
    which is what makes their sim times a regression check on the gate being
    free when it should be.

    Mon starts before scan_start_i so no error strobe can be missed; it reads
    only real top-level ports plus the seam aliases the wrapper provides.
    """
    tb = EnumDlTB(dut)
    await tb.reset()
    mon = Mon(dut)
    mon.start()
    completer = PhyCompleter(
        tb,
        device=completer_kwargs.pop("device", None) or acceptance_device(),
        nph_initial=(credits or {}).get(DllpType.INIT_FC2_NP, (32, 256))[0],
        **completer_kwargs)
    completer.start()
    completer.serve()
    await initialize_flow_control(dut, tb.phy_source, credits)
    await tb.wait_fc_init()
    await tb.start_enum()
    return tb, completer, mon


def assert_golden_on_the_wire(completer, what=""):
    """The seventeen emitted TLPs against the Stage C/D direct-attach golden.

    The golden is DIRECT_GOLDEN_SEQUENCE (test_pcie_enum_bridge_tlp.py:102) --
    field-level tuples over TlpRequest, hand-transcribed from the prediction
    document before the RTL existed, and carrying the type and bus columns.
    Nothing in it depends on how the Dwords were obtained, which is why it
    transfers across the DLL boundary unchanged once the shim exists.
    """
    seen = nonempty(completer.seen, f"{what}no TLP reached the wire at all")
    assert_sequence([on_wire(r) for r in seen], DIRECT_GOLDEN_SEQUENCE,
                    f"{what}direct-attach on-wire sequence", render=render)


# ==========================================================================
# (1) The headline: a whole enumeration, on the wire, through the real DLL
# ==========================================================================
@cocotb.test()
async def test_full_enumeration_on_wire(dut):
    """Scores D-P1.  Observation point: m_phy_axis frames, LCRC verified and
    stripped, transformed to Dwords and compared field-for-field against
    DIRECT_GOLDEN_SEQUENCE; then the top's own status ports.

    ⭐ WHAT THIS PROVES THAT NO EXISTING TARGET DOES.  Two things: the
    configuration requests survive real LCRC framing and sequence numbering
    byte-for-byte, and the credit that gated them came from a real InitFC
    exchange rather than set_credits().

    Positive controls, so a vacuous pass is not available: nonempty() on the
    observation set, an exact frame count, and the full captured BAR table.
    """
    tb, completer, mon = await bring_up(dut)
    await wait_enum(dut)

    snap = await status(dut)
    assert snap["done"] == 1 and snap["error"] == 0, (
        f"enumeration ended {err_name(snap['code'])}, not done: {snap}")

    assert_golden_on_the_wire(completer)
    expect_count(completer.frames, 17,
                 "TLP frames on m_phy_axis for a full direct-attach enumeration")
    assert completer.replays == [], (
        f"{len(completer.replays)} unexpected replay(s) on a clean run -- an "
        "unAcked frame would be retransmitted and counted twice")

    # The device table, against the Stage C acceptance goldens: a 16 KB 64-bit
    # prefetchable pair at BAR0/1 assigned at MEM_BAR_BASE.
    assert_acceptance_outcome(snap)
    assert_command_last(completer)
    assert_rom_untouched(completer)
    mon.clean()


# ==========================================================================
# (2) The same golden under the Table 2-37 Endpoint minimum
# ==========================================================================
@cocotb.test()
async def test_full_enumeration_min_credit(dut):
    """Scores D-P2.  Observation points: m_phy_axis frame count and ordering,
    tx_fc_blocked_o, and the fc_* readback after init.

    The spec-visible claim is that NPH = 1 (Base 2.1 Table 2-37 p.137, the
    Endpoint minimum) changes the TIMING and nothing else: the same seventeen
    transactions, in the same order, with the same payloads.

    ⭐ THE STALL IS COUNTED FROM FC-GATED STATES, NOT FROM CYCLES.  What is
    asserted is the number of times tx_fc_blocked_o ROSE -- each rise is one
    request the credit gate actually held -- not how long it stayed high, which
    would pin a latency rather than a behaviour.

    !! The enumerator cannot pace itself and must not be expected to.  It reads
    neither outstanding_o nor any tag -- structurally, the sequencers have no
    port on which they could see one (pcie_enum_scan.sv:145-150) -- so it
    issues, and the host's credit gate holds the TLP in the VC buffer.  That is
    why the observation is on the wire, where the spec can see it.

    ⛔ MEASURED, AND IT FALSIFIED THE PREDICTION: WITH A PROMPTLY-RETURNING FAR
    END, NPH=1 IS NEVER BINDING AND THE STALL COUNT IS ZERO.  Prediction D-P2
    said at least one stall would be observable; the first run measured none,
    with the golden and the frame count both correct.  The reason is that the
    enumerator is SINGLE-OUTSTANDING three times over -- cmd_ready_o is low from
    acceptance until the response is consumed (pcie_cfg_txn.sv:490), there is
    exactly one pcie_cfg_txn instance, and the credit gate sits behind both.  So
    request N+1 is not offered until the completion for N has been consumed, and
    a far end that advertises the freed buffer in the same breath as the
    completion has always restored the credit before anything wants it.

    That is a true statement about the design and NOT a reason to delete the
    assertion: it means the one-deep pool is never exercised by enumeration
    traffic, so a test that merely ran and passed would say nothing about the
    credit gate at all.  What makes NPH=1 binding is the far end holding its
    buffer for a while, which is legal and realistic.  credit_return_delay does
    exactly that, and it changes only the timing -- the seventeen-row golden
    below is asserted unchanged, which is the actual claim.
    """
    tb, completer, mon = await bring_up(dut, MIN_CREDIT_EP,
                                        credit_return_delay=64)

    # Negative control on the profile itself: prove the minima actually took,
    # or "no second frame in flight" would be trivially true under NPH=32.
    await ReadOnly()
    assert int(dut.fc_ph_o.value) == 1, "PH is not the Table 2-37 minimum"
    assert int(dut.fc_nph_o.value) == 1, "NPH is not the Table 2-37 minimum"
    assert int(dut.fc_cplh_o.value) == 0, "CPLH is not zero-encoded infinite"
    await RisingEdge(dut.clk_i)

    stalls = [0]
    stop = [False]
    peak_in_flight = [0]
    violations = []

    async def watch_blocked():
        prev = 0
        cycle = 0
        while not stop[0]:
            await RisingEdge(dut.clk_i)
            await ReadOnly()
            cycle += 1
            now = int(dut.tx_fc_blocked_o.value)
            if now and not prev:
                stalls[0] += 1
            prev = now
            # Frames emitted and not yet answered.  Completions are counted
            # AFTER the send, so this is the largest value consistent with the
            # far end's own record -- the conservative direction for an upper
            # bound.
            in_flight = len(completer.frames) - completer.completions_sent
            peak_in_flight[0] = max(peak_in_flight[0], in_flight)
            if in_flight > 1:
                violations.append(
                    f"cycle {cycle}: {in_flight} configuration frames on the "
                    "wire unanswered; a one-deep non-posted header pool "
                    "permits exactly one")

    cocotb.start_soon(watch_blocked())
    await wait_enum(dut)
    stop[0] = True

    assert violations == [], (
        "two config frames were in flight at once under NPH=1:\n  "
        + "\n  ".join(violations[:8]))

    snap = await status(dut)
    assert snap["done"] == 1 and snap["error"] == 0, (
        f"enumeration under the credit minimum ended {err_name(snap['code'])}: "
        f"{snap}")

    assert_golden_on_the_wire(completer, "MIN_CREDIT_EP: ")
    expect_count(completer.frames, 17,
                 "MIN_CREDIT_EP: TLP frames on m_phy_axis")
    assert stalls[0] >= 1, (
        "tx_fc_blocked_o never rose under NPH=1 even with the far end holding "
        f"its buffer for {completer.credit_return_delay} cycles -- the credit "
        "gate was not exercised, so this run says nothing about it")
    assert_acceptance_outcome(snap, "MIN_CREDIT_EP: ")
    # allow_timeouts stays FALSE: pacing must not fabricate a completion
    # timeout, and the timers run from ALLOCATION, not from transmission.
    mon.clean()
    dut._log.info(
        f"D-P2 measurement: {stalls[0]} credit stall(s), peak "
        f"{peak_in_flight[0]} frame(s) in flight (prompt-return control "
        "measured 0 stalls -- see docstring)")


# ==========================================================================
# (3) A NAK inside the sequence: byte-identical replay, no duplicate above
# ==========================================================================
@cocotb.test()
async def test_nak_inside_sequence(dut):
    """Scores D-P3.  Observation point: m_phy_axis.

    ⭐ FRAME 8 IS CHOSEN, NOT ARBITRARY.  It is the BAR2 sizing write -- a CfgWr
    CARRYING A PAYLOAD.  A NAK on a payload-free CfgRd would compare a replay
    whose payload Dwords do not exist, and would pass against a DLL that
    replayed the header correctly and dropped the data.

    Two independent claims, because the byte comparison alone does not cover
    the second: the replayed frame is byte-identical INCLUDING its sequence
    number and LCRC (a data-link property), and the Transaction Layer above
    sees the transaction ONCE (an integration property -- a duplicate delivered
    upward would add an eighteenth row to the golden).
    """
    nak_index = 8
    assert DIRECT_GOLDEN_SEQUENCE[nak_index][2], \
        "premise: the NAKed frame must be a WRITE so the replay carries payload"

    tb, completer, mon = await bring_up(dut, nak_at=nak_index)
    await wait_enum(dut)

    snap = await status(dut)
    assert snap["done"] == 1 and snap["error"] == 0, (
        f"enumeration did not survive one NAK: {err_name(snap['code'])} {snap}")

    assert completer.naks_sent == 1, (
        f"the bench sent {completer.naks_sent} NAK(s) -- if it sent none, this "
        "test proves nothing and passes vacuously")
    replays = expect_count(completer.replays, 1,
                           "replayed frames after one NAK")
    assert replays[0] == completer.frames[nak_index], (
        "the replayed frame is not byte-identical to the original:\n"
        f"  first  {completer.frames[nak_index].hex()}\n"
        f"  replay {replays[0].hex()}")

    # The TL saw it once: the golden is unchanged and still seventeen rows.
    assert_golden_on_the_wire(completer, "after one NAK: ")
    expect_count(completer.frames, 17, "accepted TLP frames after one NAK")
    assert_acceptance_outcome(snap, "after one NAK: ")
    mon.clean()


# ==========================================================================
# (4) ⭐ THE START GATE, IN RTL -- this test used to assert the opposite
# ==========================================================================
@cocotb.test()
async def test_start_gate_rtl(dut):
    """An early scan_start_i is HELD by the RTL, then honoured when FC init lands.

    ⭐ THIS TEST USED TO ASSERT THE OPPOSITE, and the pair of measurements is
    the point of the whole rung.

    It was test_start_gate_negative_control, and it scored D-P4.  The STIMULUS
    is unchanged -- scan_start_i pulsed while phy_link_up_i is high but BEFORE
    flow control has initialised, the mistake a self-starting top gated on
    link-up would make.  What changed is the verdict.  Before the gate existed
    the first CfgRd was TAGGED, PARKED in the VC buffer, and TIMED OUT HAVING
    NEVER BEEN TRANSMITTED; the test passed by asserting ENUM_ERR_TIMEOUT and
    spent 33116.00 ns doing it.  That number is the before-picture, recorded at
    ENUM_gate_fec4e68.txt:374 and reproduced at the port-only commit, which
    changed no behaviour.

    The causal chain it demonstrated is all still in the RTL:
      * a transmitter holds no credit until FC init completes -- Base 2.1
        SS3.3.1 p.160, quoted at pcie_rc_dl_top.sv:176-177;
      * tag allocation sits UPSTREAM of the credit gate --
        pcie_enum_scan.sv:137-144;
      * the completion timer measures from ALLOCATION --
        tlp_request_tracker.sv:39.
    pcie_enum_dl_top now refuses to hand the start to the engine until
    fc_init_done_o is high, so the chain is never ENTERED.

    !! ACT 2 IS THE ONE THAT MATTERS.  Without the hold window this is just a
    slow test_full_enumeration_on_wire.  And within act 2 the assertion with
    teeth is tags_presented == [], NOT the frame count: the OLD RTL also
    produced no frame in this window -- that was the old test's own headline
    assertion -- so "no frame" cannot distinguish gate-works from
    hazard-still-present.  A tag WAS handed out under the old RTL
    (pcie_rq_rc_top.sv:49-56 documents exactly this, and 2b-1 test i8 measured
    it).  The absence of a tag strobe is what proves the request never entered
    the engine at all, which is a strictly stronger claim than the old test's.
    """
    tb = EnumDlTB(dut)
    await tb.reset()
    mon = Mon(dut)
    mon.start()
    completer = PhyCompleter(tb, device=acceptance_device())
    completer.start()
    completer.serve()

    # ---- act 1: the early start ------------------------------------------
    # NO initialize_flow_control yet.  The link is up; that is the whole point,
    # and it is not enough.
    await ReadOnly()
    assert int(dut.phy_link_up_i.value) == 1, "premise: the link must be up"
    assert int(dut.fc_init_done_o.value) == 0, (
        "premise: flow control must NOT be initialised yet, or this test is "
        "just a slow version of test 1")
    await RisingEdge(dut.clk_i)
    await tb.start_enum()

    # ---- act 2: THE HOLD ---------------------------------------------------
    # 4000 cycles, the same window the old negative control used, so the two
    # measurements are taken over the same span.  CPL_TIMEOUT_CYCLES is 4096,
    # so this window is deliberately shorter than the timeout it is proving
    # never arms -- the timeout assertion below is about the timer never
    # STARTING, which the tag assertion already implies.
    HOLD = 4000
    for _ in range(HOLD):
        await RisingEdge(dut.clk_i)
        await ReadOnly()
        assert mon.tags_presented == [], (
            "a tag was allocated while the start gate was shut. The gate must "
            "stop the request ENTERING the engine; a tag strobe means it "
            "entered and is now parked, which is exactly the hazard this rung "
            "closes (pcie_rq_rc_top.sv:49-56)")
        assert completer.frames == [], (
            "a TLP reached m_phy_axis before flow control initialised")
        assert not mon.timeouts, (
            "the completion timer ran while the start gate was shut -- it "
            "measures from ALLOCATION, so this means a tag was allocated")
        assert int(dut.enum_error_o.value) == 0, (
            "the engine reported an error while its start was still held")
        assert int(dut.scan_busy_o.value) == 0, (
            "the scan FSM left S_IDLE while its start was still held")

    # ---- act 3: the release ------------------------------------------------
    # Stamp both edges so the ordering claim is a measurement, not an inference
    # from ordering of awaits.
    stamps = {}

    async def _stamp():
        while len(stamps) < 2:
            await RisingEdge(dut.clk_i)
            await ReadOnly()
            if "fc" not in stamps and int(dut.fc_init_done_o.value):
                stamps["fc"] = get_sim_time("ns")
            if "frame" not in stamps and completer.frames:
                stamps["frame"] = get_sim_time("ns")

    cocotb.start_soon(_stamp())

    await initialize_flow_control(dut, tb.phy_source)
    await wait_enum(dut, cycles=60000)
    snap = await status(dut)

    assert snap["done"] == 1 and snap["error"] == 0, (
        "the held start was released but enumeration did not succeed: "
        f"done={snap['done']} error={err_name(snap['code'])}")
    # The full device table, against the same Stage C acceptance goldens test 1
    # checks -- a held start must produce an IDENTICAL result, not merely a
    # successful-looking one.
    assert_acceptance_outcome(snap, "after a held start: ")
    assert_golden_on_the_wire(completer, "after a held start: ")

    assert "fc" in stamps and "frame" in stamps, f"a stamp never landed: {stamps}"
    assert stamps["frame"] > stamps["fc"], (
        f"the first frame left at {stamps['frame']} ns but fc_init_done_o did "
        f"not rise until {stamps['fc']} ns -- the gate did not order them")

    dut._log.info(
        f"start gate: held {HOLD} cycles with no tag and no frame; "
        f"fc_init_done_o at {stamps['fc']} ns, first frame at "
        f"{stamps['frame']} ns (before the gate: no frame ever, "
        f"ENUM_ERR_TIMEOUT at 33116 ns)")


# ==========================================================================
# (5) Parameter coherence across the two children
# ==========================================================================
@cocotb.test()
async def test_parameter_coherence(dut):
    """Scores D-P5.  Read from the ELABORATED design through hierarchical
    references -- no plusargs, no bench-side restatement of the value.

    ⚠️ THE ITEM IS COHERENCE, NOT TIMER ARBITRATION.  There is ONE completion
    timer in the system, not two.  pcie_cfg_txn's CPL_TIMEOUT_CYCLES arms no
    counter at all -- pcie_cfg_txn.sv:92-98 says so, and its ONLY use is the
    elaboration-time P-CRS-BUDGET guard at :222-225, which checks that the CRS
    retry budget cannot outlast the timeout.  The timer that actually fires is
    tlp_request_tracker's, reached through pcie_rc_dl_top:196 -> u_rc ->
    tlp_layer:374.

    So the two copies must agree, or the guard silently validates a number no
    timer uses and a slow device is misreported as dead with no warning.  The
    wrapper makes that structural by passing one parameter to both children;
    this test proves the structure survived elaboration.

    ⛔ AND A SILENT GUARD IS INDISTINGUISHABLE FROM AN ABSENT ONE.  The guard
    does not fire in this configuration (3 * 8 = 24 < 4096) and does not fire
    at the library defaults either (16 * 64 = 1024 < 4096), so its silence
    proves nothing on its own.  What is asserted here is the input it is
    checking, at both ends.
    """
    tb = EnumDlTB(dut)
    await tb.reset()

    def param(path, name):
        obj = dut
        for part in path.split("."):
            obj = getattr(obj, part)
        return int(getattr(obj, name).value)

    # The four shared parameters, read on BOTH children.
    shared = {
        "AXIS_DATA_WIDTH": 128,
        "AXIS_KEEP_WIDTH": 4,
        "AXIS_USER_WIDTH": 60,
        "CPL_TIMEOUT_CYCLES": 4096,
    }
    for name, expected in shared.items():
        enum_value = param("dut.u_enum", name)
        rcdl_value = param("dut.u_rcdl", name)
        assert enum_value == rcdl_value, (
            f"{name} differs across the seam: u_enum={enum_value}, "
            f"u_rcdl={rcdl_value} -- one localparam must feed both")
        assert enum_value == expected, (
            f"{name} elaborated to {enum_value}, expected {expected}")

    # ⭐ The end-to-end claim, at the two places that matter: the value the
    # P-CRS-BUDGET guard checks, and the value the only real timer counts to.
    guard_value = param("dut.u_enum.u_txn", "CPL_TIMEOUT_CYCLES")
    timer_value = param("dut.u_rcdl.u_rc.u_tlp_layer.tracker_inst",
                        "CPL_TIMEOUT_CYCLES")
    assert guard_value == timer_value == shared["CPL_TIMEOUT_CYCLES"], (
        f"the P-CRS-BUDGET guard checks {guard_value} while the completion "
        f"timer counts to {timer_value} -- the guard is validating a number no "
        "timer uses")

    # The CRS budget the guard exists to bound, from the elaborated design.
    retries = param("dut.u_enum.u_txn", "CRS_RETRY_MAX")
    backoff = param("dut.u_enum.u_txn", "CRS_BACKOFF_CYCLES")

    # ⭐ ADDED AFTER THE MUTATION CENSUS (M4).  Swapping these two parameters
    # (3 <-> 8) SURVIVED the original five tests: the product is unchanged, so
    # the budget check below could not see it, and no test this rung exercises
    # the CRS path that would feel the difference (design test (c1), deferred).
    # A product-only check is blind to any pair with the same product.
    #
    # The oracle is the design record, not the DUT: DESIGN_ENUM_STACK_TOP SS3.4
    # pins 3 and 8 to match the three seam benches
    # (tb_pcie_enum_bridge_tlp.sv:37-38) and to satisfy the guard against 4096.
    assert retries == 3, (
        f"CRS_RETRY_MAX elaborated to {retries}, expected 3 -- the seam "
        "benches' value (DESIGN SS3.4)")
    assert backoff == 8, (
        f"CRS_BACKOFF_CYCLES elaborated to {backoff}, expected 8 -- the seam "
        "benches' value (DESIGN SS3.4)")
    assert retries * backoff < timer_value, (
        f"P-CRS-BUDGET: {retries} retries x {backoff} cycles >= the "
        f"{timer_value}-cycle timeout -- CRS retries could outlast it")

    # ⭐ THE SHARED SET IS EXACTLY FOUR, and this is the assertion that makes
    # "no parameter left implicitly different between the two sides" checkable
    # rather than a claim.  A parameter present on both children but passed by
    # the wrapper to only one would show up here as a name that resolves on
    # both and is therefore NOT in the child-specific list.
    def has(path, name):
        obj = dut
        for part in path.split("."):
            obj = getattr(obj, part)
        return hasattr(obj, name)

    for name in ("TAG_COUNT", "CONTEXT_WIDTH"):
        assert has("dut.u_rcdl", name), f"{name} should exist on u_rcdl"
        assert not has("dut.u_enum", name), (
            f"{name} resolves on u_enum too -- the shared set is larger than "
            "the four this wrapper unifies, and the extra one is unmanaged")
    for name in ("CRS_RETRY_MAX", "CRS_BACKOFF_CYCLES", "MEM_BAR_BASE",
                 "MEM_BAR_WINDOW"):
        assert has("dut.u_enum", name), f"{name} should exist on u_enum"
        assert not has("dut.u_rcdl", name), (
            f"{name} resolves on u_rcdl too -- the shared set is larger than "
            "the four this wrapper unifies, and the extra one is unmanaged")

    assert param("dut.u_rcdl", "TAG_COUNT") == 32, (
        "TAG_COUNT is not pcie_rc_dl_top's tested default")


# ==========================================================================
# (6) UR on the probe is ABSENCE, not an error -- through the real DLL
# ==========================================================================
@cocotb.test()
async def test_probe_ur_is_absence_not_error(dut):
    """A UR to the Function 0 probe exits NORMALLY with device_present_o low.

    Oracle is the spec, not the RTL: Base 2.1 SS7.3.1 p.479 (an unimplemented
    Function in an ARI Device) and SS7.3.3 p.480 (the general Endpoint rule).
    pcie_enum_scan.sv:110-131 asserts the same thing in a comment; that comment
    is the CLAIM UNDER TEST and is not evidence for itself.

    !! WHY ABSENCE CANNOT MEAN "NO DEVICE ON THE LINK".  This is
    point-to-point, and phy_link_up_i is asserted, so a device IS attached
    whenever this scan runs.  A UR to the Function 0 probe therefore means
    "nothing here to enumerate" -- which is a terminal, NON-ERROR outcome, and
    is why there is no ENUM_ERR code for it.

    Why run it here as well as at unit level: test_pcie_enum_scan.py's S12 pins
    the companion property against a zero-latency socket.  This runs the same
    spec rule through the real data link layer, where the completion is a
    framed TLP carrying a sequence number and an LCRC and arrives through the
    very credit gate the start gate controls.  Zero-latency models are blind to
    ordering.
    """
    tb, completer, mon = await bring_up(dut, ur_regs={CFG_REG_VENDOR_DEVICE})
    await wait_enum(dut, cycles=60000)
    await ReadOnly()

    # A guard never seen firing is not known to work.
    assert completer.ur_injected_hits >= 1, (
        "the UR arm never fired, so this test proves nothing about UR "
        "handling -- the injection missed the register the probe reads")

    assert int(dut.device_present_o.value) == 0, (
        "a UR to the Function 0 probe must report the device ABSENT "
        "(SS7.3.1 p.479, SS7.3.3 p.480)")
    assert int(dut.enum_error_o.value) == 0, (
        "UR on the probe was classified as an ERROR ("
        f"{err_name(int(dut.enum_error_code_o.value))}); it is a NORMAL exit. "
        "Absence is the one thing a UR to the probe means, and the design "
        "deliberately has no ENUM_ERR code for it")

    dut._log.info(
        f"probe UR: present={int(dut.device_present_o.value)}, "
        f"ur_injected_hits={completer.ur_injected_hits}, no error")


# ==========================================================================
# (7) ⭐ THE PIN: FFFFh from a Successful Completion is PRESENT, not absent
# ==========================================================================
@cocotb.test()
async def test_vendor_ffff_on_success_is_present(dut):
    """An SC carrying FFFFFFFF reports a PRESENT device with Vendor ID FFFFh.

    Base 2.1 SS2.3.2 Implementation Note p.122 has a Root Complex synthesise an
    all-1s read value "when UR Completion Status is returned for a
    Configuration Read Request", FOR SOFTWARE ABOVE IT.  This stack sits where
    that synthesis would be PERFORMED, not consumed -- it sees the UR itself,
    as TXN_UR.  So absence is signalled by UR and by nothing else, and an SC is
    an SC whatever data it carries.  Re-deriving absence from the sentinel
    would discard information the spec took care to keep distinguishable.

    Paired deliberately with test (6): together they are the two halves of one
    claim, and a design that collapses them passes neither.

    ⚠️ THE PAYLOAD IS A FIXED POINT OF THE DWORD TRANSFORM, UNAVOIDABLY.
    FFFFFFFF is byte-reversal-invariant, so this register ALONE cannot show the
    transform ran.  The usual rule -- pick a fixed-point-free payload -- cannot
    be satisfied here, because the value IS the property under test.  Coverage
    comes from the companion read in the same probe: register 3 answers
    reg3(HDR_TYPE0) = 0x00000010, which is NOT byte-reversal-invariant, so the
    header-type assertion below fails if the transform is broken.  That pairing
    is why both assertions live in one test rather than two.
    """
    device = ConfigDevice(
        bars={CFG_REG_BAR0: BarSpec(BAR_MEM64, ACCEPT_BAR_SIZE, prefetch=True)},
        vendor=0xFFFF, device=0xFFFF)
    tb, completer, mon = await bring_up(dut, device=device)
    await wait_enum(dut, cycles=60000)
    await ReadOnly()

    assert int(dut.device_present_o.value) == 1, (
        "a device that answered with a Successful Completion was reported "
        "ABSENT because its Vendor ID was FFFFh. Absence is signalled by UR; "
        "an SC is an SC whatever data it carries (SS2.3.2 p.122)")
    assert int(dut.vendor_id_o.value) == 0xFFFF, (
        f"the reported Vendor ID was altered: "
        f"{int(dut.vendor_id_o.value):#06x}")
    assert int(dut.device_id_o.value) == 0xFFFF, (
        f"the reported Device ID was altered: "
        f"{int(dut.device_id_o.value):#06x}")

    # The non-fixed-point half of the pair: this one proves the Dword transform
    # actually ran, which FFFFFFFF is structurally incapable of showing.
    assert int(dut.header_type_o.value) == HDR_TYPE0, (
        f"header type read back {int(dut.header_type_o.value):#04x}, expected "
        f"{HDR_TYPE0:#04x} -- this is the register that is NOT a byte-reversal "
        "fixed point, so a wrong value here means the Dword transform, not the "
        "sentinel logic")
    assert int(dut.enum_error_o.value) == 0, (
        f"enumeration errored: {err_name(int(dut.enum_error_code_o.value))}")

    dut._log.info(
        "FFFFh sentinel: present=1, vendor=0xffff, device=0xffff, "
        f"header_type={int(dut.header_type_o.value):#04x} (the "
        "non-fixed-point companion)")


# ==========================================================================
# (8) ok_to_issue_o -- CLOSES MUTATION GAP M4
# ==========================================================================
@cocotb.test()
async def test_ok_to_issue_tracks_all_three_conjuncts(dut):
    """ok_to_issue_o is the three-term conjunction, and is NOT fc_init_done_o.

    ⭐ THIS TEST EXISTS BECAUSE THE MUTATION CENSUS FOUND NOTHING WATCHING THE
    PORT.  M4 replaced the whole expression with 1'b1 and ALL SEVEN tests
    passed, on both rows that instantiate the module.  An output no test reads
    is an output with no contract, however carefully its declaration is worded.

    What the port means (pcie_rc_dl_top's declaration argues it at length): the
    STANDING preconditions for transmission.  The real parking decision is
    tlp_layer.sv:280,

        vc_packet_ready = credit_request_ready && transmit_enable_i && link_up_i

    and this port carries the three terms that are state.  The fourth,
    credit_request_ready, is request-qualified rather than state and is
    deliberately absent -- see that declaration for why exporting it would need
    a request class to be chosen.

    !! THE ASSERTION THAT MATTERS IS THE THIRD ONE.  Anyone reading the name
    would guess ok_to_issue_o is an alias of fc_init_done_o; the recon found it
    is not, and this is the point that proves it -- FC init is complete, so
    fc_init_done_o is HIGH, while transmit_enable_i is low and ok_to_issue_o is
    therefore LOW.  A design that collapsed the two ports would pass every other
    assertion in this file and fail here.
    """
    tb = EnumDlTB(dut)
    await tb.reset()

    # (a) before FC init: no conjunct is satisfied.
    await ReadOnly()
    assert int(dut.fc_init_done_o.value) == 0, "premise: FC init not complete"
    assert int(dut.ok_to_issue_o.value) == 0, (
        "ok_to_issue_o is high before flow control initialised -- a "
        "transmitter holds no credit until FC init completes "
        "(Base 2.1 SS3.3.1 p.160)")

    completer = PhyCompleter(tb, device=acceptance_device())
    completer.start()
    completer.serve()
    await initialize_flow_control(dut, tb.phy_source)
    await tb.wait_fc_init()

    # (b) all three satisfied.
    await ReadOnly()
    assert int(dut.fc_init_done_o.value) == 1
    assert int(dut.ok_to_issue_o.value) == 1, (
        "FC init is complete, transmit is enabled and the link is up, but "
        "ok_to_issue_o is low")

    # (c) ⭐ THE DISCRIMINATING POINT: transmit_enable_i alone drops it, while
    #     fc_init_done_o stays high.  This is what separates the two ports.
    #     The RisingEdge leaves the read-only phase (b) put us in -- cocotb
    #     refuses a write scheduled during ReadOnly.
    await RisingEdge(dut.clk_i)
    dut.transmit_enable_i.value = 0
    await RisingEdge(dut.clk_i)
    await ReadOnly()
    assert int(dut.fc_init_done_o.value) == 1, (
        "premise: dropping transmit_enable_i must not disturb FC init")
    assert int(dut.ok_to_issue_o.value) == 0, (
        "ok_to_issue_o stayed high with transmit_enable_i low. It is NOT an "
        "alias of fc_init_done_o -- it carries the class-independent conjuncts "
        "of the real parking decision (tlp_layer.sv:280), and transmit_enable_i "
        "is one of them")
    await RisingEdge(dut.clk_i)
    dut.transmit_enable_i.value = 1

    # (d) the link conjunct, which is not redundant with the sticky bit: the
    #     sticky is REGISTERED and lags a link drop by a cycle, while the gate
    #     this port mirrors is combinational.
    await RisingEdge(dut.clk_i)
    dut.phy_link_up_i.value = 0
    await RisingEdge(dut.clk_i)
    await ReadOnly()
    assert int(dut.ok_to_issue_o.value) == 0, (
        "ok_to_issue_o stayed high with the link down")

    dut._log.info("ok_to_issue_o: 0 before FC init, 1 after, 0 on "
                  "!transmit_enable_i with fc_init_done_o still 1, 0 on "
                  "link-down")


# ==========================================================================
# (9) A link drop disarms a pending start -- CLOSES MUTATION GAP M6
# ==========================================================================
@cocotb.test()
async def test_link_drop_disarms_pending_start(dut):
    """A start held across a link drop is DISCARDED, not carried over.

    ⭐ THIS TEST EXISTS BECAUSE A COMMENT CLAIMED IT AND NOTHING CHECKED IT.
    start_pending_r is cleared by `rst_i || !phy_link_up_i`, and the RTL comment
    says that is so "a link that drops mid-wait does not leave a stale request
    armed for the next link-up".  Mutation M6 deleted the !phy_link_up_i term
    and all seven tests still passed, so the claim was decoration.

    Why the behaviour is right: FC initialisation is entered on entrance to
    DL_Init and completes once per link-up (Base 2.1 SS3.3.1 p.160), so a link
    that drops and returns is a NEW link-up with its own FC init.  A scan
    request made against the previous one has no standing -- the topology may
    have changed underneath it -- and the integrator has to ask again.

    The failure this prevents is a self-starting enumeration nobody requested,
    which is the same class of surprise the whole rung exists to remove.
    """
    tb = EnumDlTB(dut)
    await tb.reset()
    mon = Mon(dut)
    mon.start()
    completer = PhyCompleter(tb, device=acceptance_device())
    completer.start()
    completer.serve()

    # Arm the latch: a start while the gate is shut.
    await ReadOnly()
    assert int(dut.fc_init_done_o.value) == 0, "premise: gate must be shut"
    await RisingEdge(dut.clk_i)
    await tb.start_enum()

    # Bounce the link.  The sticky clears, and so must the pending request.
    dut.phy_link_up_i.value = 0
    for _ in range(16):
        await RisingEdge(dut.clk_i)
    dut.phy_link_up_i.value = 1
    for _ in range(16):
        await RisingEdge(dut.clk_i)

    # A fresh FC init for the new link-up.
    await initialize_flow_control(dut, tb.phy_source)
    await tb.wait_fc_init()

    # Nothing may start on its own.  Bounded window, then assert idle.
    for _ in range(2000):
        await RisingEdge(dut.clk_i)
        await ReadOnly()
        assert int(dut.scan_busy_o.value) == 0, (
            "a scan started after a link bounce although the only start "
            "request was made against the PREVIOUS link-up -- the pending "
            "request was not disarmed by the link drop")
    await ReadOnly()
    assert mon.tags_presented == [], (
        "a tag was allocated for a request nobody made on this link-up")
    assert completer.frames == [], (
        "a TLP reached the wire for a request nobody made on this link-up")
    assert int(dut.enum_done_o.value) == 0 and int(dut.enum_error_o.value) == 0, (
        "the engine reached a terminal state without ever being started")

    dut._log.info(
        "link bounce: pending start discarded; no scan, no tag, no frame in "
        "2000 cycles after the new FC init")


# ==========================================================================
# (10) fc_init_done_o is MONOTONIC across an enumeration -- and (11) the
#      control that proves the monitor can see the opposite
# ==========================================================================
class FcInitWatch:
    """Continuous sampler for fc_init_done_o.  Records every edge, both ways.

    ⭐ WHY A SAMPLER AND NOT AN ASSERTION AT THE END.  Every existing check on
    this port in this file is a POINT check -- test_start_gate_rtl reads it
    while the gate is shut (:774) and stamps its first rise (:814);
    test_ok_to_issue_tracks_all_three_conjuncts reads it once more after
    dropping transmit_enable_i (:1129); test_link_drop_disarms_pending_start
    reads it while shut (:1186).  Not one of them samples it CONTINUOUSLY, so
    nothing in the suite would notice a transient 1 -> 0 -> 1 in between.  That
    gap is the rule debt this pair closes; it is a debt, not a defect, and the
    positive row is expected to pass on the clean tree.

    The property is worth pinning precisely because the RAW signal underneath
    is NOT monotonic.  pcie_rc_dl_top.sv:202-212 records that the DLL's
    fc_initialized_o glitches 1 -> 0 -> 1 while pcie_flow_ctrl_init walks
    ST_UPDATE_P .. ST_UPDATE_NP_CRC, over a window of four STATES that is NOT
    bounded at four cycles because each is gated on fc_axis_tready.
    fc_init_sticky_r (:213-216) is the filter that exists to hide exactly that,
    and fc_init_done_o (:223) is its outward face.  So "monotonic" here is a
    claim about the FILTER, and the thing it filters is a real oscillation.

    Sampling is RisingEdge-then-ReadOnly, never a bare read after the edge: a
    bare read returns the PRE-edge value and would report the register's old
    state, which for a sticky bit is precisely the value that never falls.  A
    monitor with that bug would pass this rule vacuously and for ever.

    Unresolvable samples are skipped rather than counted as 0, so the X the
    register holds before reset does not register as a fall.
    """

    def __init__(self, dut):
        self.dut = dut
        self.rose_at = None
        self.falls = []          # sim time (ns) of every 1 -> 0 observed
        self.samples = 0         # RESOLVABLE samples only -- the window's size

    def start(self):
        cocotb.start_soon(self._run())

    async def _run(self):
        prev = None
        while True:
            await RisingEdge(self.dut.clk_i)
            await ReadOnly()
            raw = self.dut.fc_init_done_o.value
            if not raw.is_resolvable:
                continue
            now = int(raw)
            self.samples += 1
            if prev is not None and now > prev and self.rose_at is None:
                self.rose_at = get_sim_time("ns")
            if prev is not None and now < prev:
                self.falls.append(get_sim_time("ns"))
            prev = now


@cocotb.test()
async def test_fc_init_done_monotonic_across_enumeration(dut):
    """fc_init_done_o must never fall while the link stays up.

    Base 2.1 §3.3.1 p.160: for VC0, FC_INIT1 is entered only on "Entrance to
    DL_Init state" -- FC initialisation completes ONCE per link-up and is not
    re-entered without a link-down.  So for the whole of an enumeration, with
    phy_link_up_i held high throughout, the filtered view of that state has
    exactly one edge: 0 -> 1.  A second edge in either direction would mean the
    Transaction Layer's credit gate reopened mid-enumeration.

    ⚠️ EXPECTED TO PASS ON THE CLEAN TREE.  This is a RULE DEBT, not a defect:
    the behaviour is already right and nothing asserted it.  A row that goes red
    here would mean the sticky filter is not doing its job, which would be a
    genuine finding -- but it is not the outcome this row is written to expect.

    NON-VACUITY (§22.82) is carried by three independent facts, because "a
    signal never fell" is exactly the shape that passes when nothing happened:
      1. the rise is REQUIRED to have been observed -- a run where
         fc_init_done_o never asserted would otherwise satisfy "never fell";
      2. the sample count is asserted against a floor, so the window is known to
         be long rather than assumed to be;
      3. the enumeration is required to have COMPLETED and to have put the
         golden sequence on the wire, so the window provably covered real work
         and not an idle DUT.
    The separate control row (11) closes the fourth hole -- that the monitor is
    capable of reporting a fall at all.
    """
    watch = FcInitWatch(dut)
    watch.start()          # before bring_up: the clock starts inside it, and
                           # RisingEdge simply waits for the first edge, so the
                           # rise itself is inside the observed window
    tb, completer, mon = await bring_up(dut)
    await wait_enum(dut)
    snap = await status(dut)

    # --- non-vacuity 3: the window covered a real, successful enumeration ---
    assert snap["done"] == 1 and snap["error"] == 0, (
        f"enumeration ended {err_name(snap['code'])}, not done: {snap} -- the "
        "monotonicity window must cover real work, not a stalled run")
    assert_golden_on_the_wire(completer)

    # --- non-vacuity 1: the property is not being asserted over an empty set ---
    assert watch.rose_at is not None, (
        "fc_init_done_o never rose during the whole enumeration, so "
        "'it never fell' is a claim about a signal that was flat at 0 -- "
        "vacuous, and the InitFC exchange never completed")

    # --- non-vacuity 2: the window is long, measured rather than assumed ---
    # A real enumeration measures 1179 resolvable samples here.  The floor is
    # set at 500, not just under the measured value: this guard exists to catch
    # a window of a HANDFUL of cycles -- the vacuous case -- and a floor sitting
    # 15% below the measurement would convert ordinary timing drift into a
    # spurious red without catching anything a 500 floor misses.
    assert watch.samples > 500, (
        f"only {watch.samples} resolvable samples -- too short a window for "
        "'never fell' to mean anything about an enumeration")

    # --- the property itself ---
    assert watch.falls == [], (
        f"fc_init_done_o fell {len(watch.falls)} time(s), at {watch.falls} ns, "
        "while phy_link_up_i was held high.  Base 2.1 §3.3.1 p.160: FC init "
        "completes once per link-up, so the filtered view must have exactly "
        "one edge.  A fall reopens the TL's credit gate mid-enumeration -- and "
        "it means fc_init_sticky_r (pcie_rc_dl_top.sv:213-216) is passing "
        "through the raw glitch it exists to hide")

    mon.clean()
    dut._log.info(
        f"fc_init_done_o monotonic: rose once at {watch.rose_at} ns, zero "
        f"falls across {watch.samples} sampled cycles of a completed "
        "enumeration")


@cocotb.test()
async def test_fc_init_done_watch_reports_a_fall_on_link_drop(dut):
    """CONTROL for the row above.  The same monitor, made to see a 1 -> 0.

    ⭐ THE CONTROL MUST NOT BE DERIVED FROM THE SIGNAL UNDER TEST (§22.80).
    The provocation here is phy_link_up_i, which is an INPUT to the sticky bit
    -- `if (rst_i || !phy_link_up_i) fc_init_sticky_r <= 1'b0`
    (pcie_rc_dl_top.sv:214) -- and not a function of fc_init_done_o's output.
    So this row is a statement about the MONITOR's sensitivity, established
    through a different signal than the one whose behaviour row (10) asserts.

    Without it, row (10) is unfalsifiable in the way that matters: a monitor
    that never reports a fall under ANY stimulus would pass row (10) for ever
    and would report nothing if the sticky filter were later broken.  This row
    is the reason row (10)'s green is worth having.

    It also re-states, as a measurement, the RTL comment that the sticky bit is
    cleared by a link drop -- the same claim test_link_drop_disarms_pending_start
    relies on for start_pending_r, checked here for fc_init_done_o itself.
    """
    watch = FcInitWatch(dut)
    watch.start()
    tb, completer, mon = await bring_up(dut)
    await wait_enum(dut)

    # Premise: we are in exactly the state row (10) ends in -- risen, no fall.
    assert watch.rose_at is not None, (
        "premise: fc_init_done_o must have risen before a fall can be provoked")
    assert watch.falls == [], (
        f"premise: no fall may have happened yet, saw {watch.falls} ns")
    before = len(watch.falls)

    # ⚠️ LEAVE THE READ-ONLY PHASE BEFORE WRITING.  wait_enum() returns from
    # INSIDE `await ReadOnly()` (:465-466), so its caller resumes in the
    # read-only sync phase and the very next signal write dies with "Write to
    # object phy_link_up_i was scheduled during a read-only sync phase".  The
    # first version of this row did exactly that.  It is loud rather than
    # silent, which is the good case -- but it is the same shape as the
    # ReadOnly/RisingEdge confusion the file already warns about for READS, and
    # test_link_drop_disarms_pending_start (:1185-1187) steps past it the same
    # way for the same reason.
    await RisingEdge(dut.clk_i)

    # Provoke, through an input to the sticky bit rather than through anything
    # derived from its output.
    dut.phy_link_up_i.value = 0
    for _ in range(16):
        await RisingEdge(dut.clk_i)

    assert len(watch.falls) > before, (
        "phy_link_up_i was dropped for 16 cycles and the watcher reported NO "
        "fall of fc_init_done_o.  Either the sticky bit is not cleared by a "
        "link drop -- contradicting pcie_rc_dl_top.sv:214 -- or the monitor "
        "cannot see a fall at all, in which case the monotonicity row it "
        "shares code with is vacuous and its green means nothing")

    mon.clean()
    dut._log.info(
        f"control: monitor reported a fall at {watch.falls[before]} ns when "
        "phy_link_up_i dropped -- row (10)'s zero-fall result is therefore a "
        "measurement, not a blind spot")
