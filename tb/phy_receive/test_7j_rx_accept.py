"""§63 #7j-1 — receive-path acceptance for a continuous Logical Idle stream.

Toplevel: `phy_receive`.  These are the rung's RX acceptance rows, and the whole
bench is spec-anchored: every expected byte comes from `rx_golden`, which is
built from Base 2.1 Appendix C.1 and is checked here, before any DUT byte is
read, against **both** golden tables printed on Base 2.1 p.700 — the 128 LFSR
states and the 304 scrambled-Logical-Idle bytes (D-7J.3).

Oracles
  O-7J1-K  known answer: `rx_golden` reproduces both p.700 tables exactly.
  O-7J1-A  §4.2.2 p.195 "Receivers must ignore incoming Logical Idle data":
           a continuous scrambled-idle stream produces no AXIS beat at all.
  O-7J1-B  §4.2.2 p.195 framing: a DLLP is framed by SDP ... END.  A receiver
           that has begun to deliver a packet must finish it -- a beat with
           `tlast` must follow -- or deliver nothing.  **A `tlast`-less
           fragment is neither, and it is what this rung fixes.**
  O-7J1-C  two packets separated by an idle gap of any length both arrive
           intact and nothing is delivered during the gap.

⚠️ Why the gap sweep in O-7J1-C is a FENCE and not a red row: it was measured
green before it was written (`FINDINGS_7J_PHASE1.md`), at every gap from 0 to 14.
It is here to stay green across #7j-2, which changes what fills those gaps.
"""
import json
import os

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ClockCycles

from rx_golden import (COM, SDP, END, TS1_ID, GEN1, Descrambler,
                       ts_ordered_set, advance, xor_mask)

PIPE_WIDTH = 8
_TABLES = json.load(open(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                      "p700_tables.json")))
LFSR_TAB = [_TABLES["lfsr"][str(i)] for i in range(128)]
IDLE_TAB = [_TABLES["idle"][str(i)] for i in range(304)]


# ------------------------------------------------------------------ harness
async def setup(dut):
    cocotb.start_soon(Clock(dut.clk_i, 10, units="ns").start())
    cocotb.start_soon(Clock(dut.pipe_rx_usr_clk_i, 10, units="ns").start())
    dut.rst_i.value = 1
    dut.en_i.value = 0
    dut.link_up_i.value = 0
    dut.pipe_data_i.value = 0
    dut.pipe_data_valid_i.value = 0
    dut.pipe_data_k_i.value = 0
    dut.pipe_sync_header_i.value = 0
    dut.pipe_block_start_i.value = 0
    dut.pipe_width_i.value = PIPE_WIDTH
    dut.num_active_lanes_i.value = 1
    dut.curr_data_rate_i.value = GEN1
    dut.m_dllp_axis_tready.value = 1
    await ClockCycles(dut.pipe_rx_usr_clk_i, 10)
    dut.rst_i.value = 0
    dut.en_i.value = 1
    dut.link_up_i.value = 1
    await ClockCycles(dut.pipe_rx_usr_clk_i, 6)


def start_axis_monitor(dut, beats):
    async def mon():
        while True:
            await RisingEdge(dut.clk_i)
            if int(dut.m_dllp_axis_tvalid.value) & 1:
                beats.append((int(dut.m_dllp_axis_tdata.value) & 0xFFFFFFFF,
                              int(dut.m_dllp_axis_tkeep.value) & 0xF,
                              int(dut.m_dllp_axis_tlast.value) & 1))
    return cocotb.start_soon(mon())


async def drive(dut, syms):
    for b, k in syms:
        dut.pipe_data_i.value = b & 0xFF
        dut.pipe_data_k_i.value = k & 0x1
        dut.pipe_data_valid_i.value = 1
        await RisingEdge(dut.pipe_rx_usr_clk_i)


async def quiesce(dut, n):
    dut.pipe_data_valid_i.value = 0
    await ClockCycles(dut.pipe_rx_usr_clk_i, n)


def idle_run(tx, n):
    """n Symbols of scrambled Logical Idle -- data 00h, K=0 (§4.2.2 p.195)."""
    return [(tx.symbol(0x00, is_k=False), 0) for _ in range(n)]


def ts1_prefix(tx, link=0x05, lane=0x00):
    ts = ts_ordered_set(TS1_ID, link=link, lane=lane)
    out = [(tx.symbol(ts[0][0], ts[0][1]), ts[0][1])]
    out += [(tx.symbol(b, k, True), k) for b, k in ts[1:]]
    return out


def dllp_frame(tx, payload):
    out = [(tx.symbol(SDP, is_k=True), 1)]
    out += [(tx.symbol(p, is_k=False), 0) for p in payload]
    out += [(tx.symbol(END, is_k=True), 1)]
    return out


def payload_of(beats):
    """The payload bytes a run of AXIS beats carries, tkeep-masked."""
    out = []
    for data, keep, _ in beats:
        for i in range(4):
            if (keep >> i) & 1:
                out.append((data >> (8 * i)) & 0xFF)
    return out


def split_packets(beats):
    """Group beats into packets at each `tlast`.  A trailing group with no
    `tlast` is returned separately -- that group is the defect O-7J1-B is
    about, so it must be visible and not silently folded in."""
    packets, cur = [], []
    for b in beats:
        cur.append(b)
        if b[2]:
            packets.append(cur)
            cur = []
    return packets, cur


# ------------------------------------------------------------------ O-7J1-K
@cocotb.test()
async def known_answer_rx_golden_reproduces_both_p700_tables(dut):
    """The oracle proves itself before it is used on anything (§22.92, D-7J.3).

    Base 2.1 App. C.1 p.697 makes the reference implementation's OUTPUT
    normative -- "they must all produce the same output as that shown here" --
    and p.700 prints two tables of it.  Both are checked, plus the COM and SKP
    rules of §4.2.3 p.199 that the tables alone do not exercise.
    """
    # Table A -- 128 LFSR states following a reset.
    lfsr = Descrambler.SEED
    for i, want in enumerate(LFSR_TAB):
        assert lfsr == want, \
            "Table A p.700 state %d: model %04X, spec %04X" % (i, lfsr, want)
        lfsr = advance(lfsr)

    # Table B -- 304 bytes of scrambled Logical Idle.
    d = Descrambler()
    for i, want in enumerate(IDLE_TAB):
        got = d.symbol(0x00, is_k=False)
        assert got == want, \
            "Table B p.700 byte %d: model %02X, spec %02X" % (i, got, want)

    # §4.2.3 p.199 bullet 5 -- a COM re-initialises, so the sequence restarts.
    d = Descrambler()
    for _ in range(37):
        d.symbol(0x00, is_k=False)
    assert d.symbol(COM, is_k=True) == COM, "a COM must pass through unscrambled"
    restart = [d.symbol(0x00, is_k=False) for _ in range(8)]
    assert restart == IDLE_TAB[:8], \
        "after a COM the idle sequence must restart at Table B[0]; got %s" % restart

    # §4.2.3 p.199 bullet 2 -- SKP neither scrambles nor advances.
    d = Descrambler()
    seq = []
    for i in range(12):
        seq.append(d.symbol(0x00, is_k=False))
        if i == 5:
            for _ in range(3):
                d.symbol(0x1C, is_k=True)      # SKP
    assert seq == IDLE_TAB[:12], \
        "3 SKPs must not shift the idle sequence; got %s" % seq

    # The negative control (§22.81): a WRONG seed must fail the same check, or
    # the three assertions above would pass for a model that ignored its input.
    bad = Descrambler()
    bad.lfsr = 0xFFFE
    assert [bad.symbol(0x00, is_k=False) for _ in range(8)] != IDLE_TAB[:8], \
        "a perturbed seed reproduced Table B -- the check is vacuous"

    dut._log.info("O-7J1-K: 128 LFSR states + 304 idle bytes + COM + SKP rules, "
                  "and the perturbed-seed control fails as it must")


# ------------------------------------------------------------------ O-7J1-A
@cocotb.test()
async def continuous_logical_idle_delivers_nothing(dut):
    """§4.2.2 p.195: "Receivers must ignore incoming Logical Idle data."

    320 Symbols of spec-golden scrambled idle, valid high throughout.  Nothing
    may reach the AXIS port, and the receiver must still be in sync afterwards
    -- which the positive half proves by decoding a packet at the end (§22.81:
    a negative assertion pairs with a positive through the same path).
    """
    await setup(dut)
    tx = Descrambler()
    beats = []
    start_axis_monitor(dut, beats)

    await drive(dut, [(tx.symbol(COM, is_k=True), 1)])
    await drive(dut, idle_run(tx, 320))
    assert not beats, \
        "%d AXIS beat(s) delivered during Logical Idle: %s" % (len(beats), beats)
    idle_only = len(beats)

    payload = [0xDE, 0xAD, 0xBE, 0xEF, 0x12, 0x34]
    await drive(dut, dllp_frame(tx, payload))
    await drive(dut, idle_run(tx, 40))
    await quiesce(dut, 40)

    assert len(beats) > idle_only, \
        "no packet decoded after 320 idle Symbols -- the receiver lost sync, " \
        "so the zero-beat check above was vacuous"
    packets, dangling = split_packets(beats)
    assert packets and payload_of(packets[0]) == payload, \
        "payload after idle: got %s want %s" % (payload_of(packets[0]), payload)
    dut._log.info("O-7J1-A: 0 beats across 320 idle Symbols, then the packet "
                  "decoded intact -- sync held")


# ------------------------------------------------------------------ O-7J1-C
@cocotb.test()
async def two_packets_survive_an_idle_gap_of_any_length(dut):
    """Both packets arrive intact and nothing is delivered during the gap, at
    every gap length from 0 to 14 Symbols -- past the receive pipeline's depth.

    FENCE row: measured green before it was written.  It is here so that #7j-2,
    which changes what fills these gaps, cannot change what comes out of them.
    """
    A = [0xDE, 0xAD, 0xBE, 0xEF, 0x12, 0x34]
    B = [0xC0, 0xFF, 0xEE, 0x99, 0x77, 0x55]
    for gap in range(0, 15):
        await setup(dut)
        tx = Descrambler()
        beats = []
        h = start_axis_monitor(dut, beats)

        await drive(dut, [(tx.symbol(COM, is_k=True), 1)] + ts1_prefix(tx))
        await drive(dut, dllp_frame(tx, A))
        if gap:
            await drive(dut, idle_run(tx, gap))
        await drive(dut, dllp_frame(tx, B))
        await drive(dut, idle_run(tx, 40))
        await quiesce(dut, 60)
        h.kill()

        packets, dangling = split_packets(beats)
        assert not dangling, \
            "gap=%d: %d beat(s) left without tlast: %s" % (gap, len(dangling), dangling)
        assert len(packets) == 2, \
            "gap=%d: expected 2 packets, got %d (%s)" % (gap, len(packets), beats)
        got_a, got_b = payload_of(packets[0]), payload_of(packets[1])
        assert got_a == A, "gap=%d: first payload %s want %s" % (gap, got_a, A)
        assert got_b == B, "gap=%d: second payload %s want %s" % (gap, got_b, B)
    dut._log.info("O-7J1-C: 15 gap lengths, both packets intact, no dangling beat")


# ------------------------------------------------------------------ §22.93
def pinned_red(dut, row, state, detail):
    """§22.93.  cocotb's `expect_fail` turns ANY exception into a PASS, so a row
    can be red for a reason unrelated to the defect it pins and the gate cannot
    tell.  Everything before the pinned assertion runs inside a try/except; an
    exception there is logged NOT_REACHED and the row RETURNS NORMALLY, which
    under `expect_fail` is a gate FAIL.  Then the REACHED marker, then the
    pinned assertion -- the only statement allowed to raise.  `sweep43.sh`
    copies these into its `.diag` as `PINNED|` rows.
    """
    dut._log.info("PINNED_RED|%s|%s|%s", row, state, detail)


# ------------------------------------------------------------------ O-7J1-B
@cocotb.test(expect_fail=True)   # §63 #7j-1 -- flips in the commit that fixes it
async def a_packet_is_delivered_whole_or_not_at_all(dut):
    """O-7J1-B.  RED BEFORE FIX.

    Base 2.1 §4.2.2 p.195 frames a DLLP as `SDP` ... `END`.  A receiver may
    deliver that packet or drop it, but it may not deliver *part* of it: a
    consumer that has taken beats with no `tlast` is left waiting for an end
    that never comes, and on an AXI-Stream link that stalls the channel.

    **RED BEFORE FIX, and here is exactly why** (measured, Phase 1):

      trailing valid Symbols   0   2   4   6   8  10 ...
      AXIS beats delivered     0   0   1   1   2   2

    At 4 and 6 the receiver emits `tdata=efbeadde tkeep=f tlast=0` and never
    the terminating beat -- a fragment.  The cause is **not** the scrambler:
    the `END` reaches `block_alignment`'s output on the same cycle (c=32) with
    a 4-Symbol tail as with an 8-Symbol one.  It is the two gatherers below it,
    neither of which treats `END` as a boundary:

      * `pack_data.sv:124-128` publishes only when a full 32-bit word has been
        accumulated (`Q.count + bytes_per_packet >= BytesPerTransfer`), so a
        packet whose `END` lands in a partial word waits for bytes that belong
        to the NEXT packet;
      * `data_handler.sv` `ST_TX` re-aligns across a one-word skid
        (`data_r >> ...` merged with `data_i`), so it needs word N+1 in hand to
        emit word N.

    The pinned assertion is the dangling-beat check.  Everything above it --
    the oracle, the drive, the decode of the complete case -- is inside the
    guard, so this row can only be red for the defect it names (§22.93).
    """
    ROW = "a_packet_is_delivered_whole_or_not_at_all"
    payload = [0xDE, 0xAD, 0xBE, 0xEF, 0x12, 0x34]
    try:
        # The oracle proves itself here too: this row must not be able to pass
        # or fail on a broken model (§22.92).
        d = Descrambler()
        assert [d.symbol(0x00, is_k=False) for _ in range(8)] == IDLE_TAB[:8], \
            "rx_golden disagrees with Table B p.700"

        observed = []
        for tail in (0, 2, 4, 6, 8, 12):
            await setup(dut)
            tx = Descrambler()
            beats = []
            h = start_axis_monitor(dut, beats)
            await drive(dut, [(tx.symbol(COM, is_k=True), 1)] + ts1_prefix(tx))
            await drive(dut, dllp_frame(tx, payload))
            if tail:
                await drive(dut, idle_run(tx, tail))
            # The stream STOPS here.  That is the condition under test, so the
            # window must not supply the very beats whose absence is the point.
            await quiesce(dut, 60)
            h.kill()
            packets, dangling = split_packets(beats)
            observed.append((tail, len(beats), len(packets), len(dangling)))
            dut._log.info("7J1[whole-or-nothing] tail=%d beats=%d packets=%d "
                          "dangling=%d %s", tail, len(beats), len(packets),
                          len(dangling), beats)

        # Positive half (§22.81): the long-tail case must deliver the payload,
        # or "no dangling beat" could be satisfied by delivering nothing ever.
        await setup(dut)
        tx = Descrambler()
        beats = []
        h = start_axis_monitor(dut, beats)
        await drive(dut, [(tx.symbol(COM, is_k=True), 1)] + ts1_prefix(tx))
        await drive(dut, dllp_frame(tx, payload))
        await drive(dut, idle_run(tx, 40))
        await quiesce(dut, 60)
        h.kill()
        packets, _ = split_packets(beats)
        assert packets and payload_of(packets[0]) == payload, \
            "the long-tail control did not deliver the payload, so the " \
            "dangling-beat check below would be vacuous"
    except Exception as exc:                                    # noqa: BLE001
        pinned_red(dut, ROW, "NOT_REACHED", "%s: %s" % (type(exc).__name__, exc))
        return

    detail = " ".join("tail=%d:beats=%d,pkts=%d,dangling=%d" % o for o in observed)
    pinned_red(dut, ROW, "REACHED", detail)
    bad = [(t, n) for t, _, _, n in observed if n]
    assert not bad, (
        "the receiver delivered a packet FRAGMENT with no tlast at trailing "
        "lengths %s (dangling beats %s).  Base 2.1 §4.2.2 p.195 frames a DLLP "
        "SDP..END; a receiver must deliver the whole packet or none of it, "
        "because a consumer holding beats with no tlast is stalled forever."
        % ([t for t, _ in bad], [n for _, n in bad])
    )
