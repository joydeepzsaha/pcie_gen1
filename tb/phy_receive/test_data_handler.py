"""data_handler -- the module's FIRST bench. §63 #7d, defect B.

!! WRITTEN AGAINST THE BROKEN RTL AND RED BEFORE ANY FIX. Same order as
block_alignment's bench at #7c: a row that only ever existed after a fix cannot
tell you the fix did anything.

== WHY THIS BENCH AND NOT A pack_data BENCH ==============================
BRIEF_7D aimed its bench 2a at `pack_data`. Phase 1 measured that `pack_data`
has NO tkeep and NO tlast port in either direction -- its interface is
data_i/data_valid_i/data_k_i/sync_header_i -> data_o/data_valid_o/data_k_o/
sync_header_o/fifo_wr_o. The AXIS boundary signals are BORN HERE, in
data_handler (phy_receive.sv:263-289). Rows 1-4 of the briefed 2a are therefore
translated onto this module; they could not have been written against
`pack_data` at all. A `pack_data` bench is still owed as #7e debt.

== THE ORACLE, AND WHY IT IS NOT dllp_handler.sv:129 =====================
A DLLP is 4 DLLP bytes + a 16-bit CRC, framed by SDP and END (Base 2.1 §3.4).
On a 32-bit word that is SIX payload bytes once SDP and END are stripped:

    wire:      SDP  D0 D1 D2 | D3  C0 C1  END      (8 Symbols)
    expected:  tkeep 0xF (D0 D1 D2 D3), then tlast with tkeep 0x3 (C0 C1)

`0x3` is confirmed from THREE independent directions, which is why this bench
does not inherit it from one reading:
  1. Base 2.1 §3.4 -- 4 + 2 payload bytes.
  2. dllp_handler.sv:129 -- dllp_crc_word_valid requires tlast && tkeep==2'b11.
  3. ⭐ MEASURED on this project's own TRANSMIT path, §63 #7d Phase 1: both
     DLLs hand phy_transmit tlast with tkeep 0x3 -- RC 21,290 of 21,290,
     EP 21,272 of 21,272, no other value. The receive path must reconstruct
     the shape the transmit path emits.

== HISTORY -- measured on the BROKEN RTL, at 018b28e =====================
These are PRE-FIX numbers (§22.87: they expire the moment the rows flip).
In the full stack, over 120,030 clocks, data_handler emitted:

    EP side   tlast 21,272, tkeep on tlast {0x7: 21,272}   -- never 0x3
    RC side   tlast      0                                  -- no END arrived

The RC's zero was defect A (the Endpoint transmitted no END Symbol at all;
fixed at eb2e662/9ecabee). With END arriving, the RC's data_handler emits
tkeep on tlast {0x7: 21,252} -- the EP's signature exactly. Defect B is in
SHARED code and was merely MASKED on the RC.

`0x7` is seven payload bytes for a six-byte DLLP: END is being counted as
payload. Row `dh_tkeep_on_tlast_is_0x3_never_0x7` is the whole defect in one
row, and it is the row that must be RED first.
"""

import os
import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ClockCycles, ReadOnly

CLK_NS = 8  # 125 MHz, the rate both PHYs actually run at (§63 #7d Phase 1.1)

# Gen1 8b/10b control codes, Base 2.1 §4.2.4.12 Table 4-3.
SDP = 0x5C  # K28.2, DLLP start
STP = 0xFB  # K27.7, TLP start -- §63 #7e; until then this bench drove the DLLP
            # arm only and had no need of it
END = 0xFD  # K29.7, frame end
COM = 0xBC  # K28.5
IDL = 0x7C  # K28.3, logical idle


def word(b0, b1, b2, b3):
    """Pack four bytes little-endian, byte 0 in bits [7:0] -- the convention
    data_handler indexes with data_i[8*byte_idx+:8]."""
    return (b3 << 24) | (b2 << 16) | (b1 << 8) | b0


def kmask(*positions):
    m = 0
    for p in positions:
        m |= 1 << p
    return m


def dllp_stream(n, pad=0x00):
    """N back-to-back DLLPs IN THE ALIGNMENT THE STACK ACTUALLY DELIVERS.

    ⚠️⚠️ THE FIRST VERSION OF THIS BENCH ASSUMED `SDP D0 D1 D2 | D3 C0 C1 END`
    -- SDP at byte 0, END at byte 3 -- and data_handler passed it, emitting the
    correct tkeep 0x3. The full stack emits 0x7. The assumption was the bug in
    the bench, not a second defect in the DUT.

    MEASURED at data_handler's input in tb_pcie_fullstack (§63 #7d), both sides,
    every frame:  SDP_at_byte[2] = 21,274 · END_at_byte[1] = 21,272
    and nowhere else -- byte 2 and byte 1 are the ONLY positions either code
    ever occupies.

    That fixes the packing: a frame's END and the NEXT frame's SDP share one
    word, which is why the stack runs at ~2.02 beats per frame and not 3:

        word A(i):  [ C1(i-1) , END , SDP , D0(i) ]   k at bytes 1 and 2
        word B(i):  [ D1(i)   , D2(i) , D3(i) , C0(i) ]   k = 0

    With END at byte_idx 1, data_handler.sv:247's `4'hF >> byte_idx` is
    `4'hF >> 1` = 0x7 -- the measured value, reproduced exactly.

    The ORACLE is unchanged: payload is D0 D1 D2 D3 C0 C1, six bytes, so the
    AXIS output must be tkeep 0xF then tlast with tkeep 0x3.
    """
    frames = [((0x10 + i) & 0xFF, (0x20 + i) & 0xFF,
               (0x30 + i) & 0xFF, (0x40 + i) & 0xFF,
               (0xA0 + i) & 0xFF, (0xB0 + i) & 0xFF) for i in range(n)]
    words = []
    for i, (d0, d1, d2, d3, c0, c1) in enumerate(frames):
        if i == 0:
            # no previous frame: pad the two bytes below SDP
            words.append((word(pad, pad, SDP, d0), kmask(2)))
        else:
            pc1 = frames[i - 1][5]
            words.append((word(pc1, END, SDP, d0), kmask(1, 2)))
        words.append((word(d1, d2, d3, c0), 0))
    # flush the final frame's C1 and END
    words.append((word(frames[-1][5], END, pad, pad), kmask(1)))
    return words


def idle_word():
    return (word(IDL, IDL, IDL, IDL), kmask(0, 1, 2, 3))


class TB:
    """ONE TB per test. cocotb kills every coroutine when a test ends, the
    Clock coroutine included, so the clock is started once here and the object
    is not shared across tests."""

    def __init__(self, dut):
        self.dut = dut
        self._clk = cocotb.start_soon(Clock(dut.clk_i, CLK_NS, units="ns").start())

    async def reset(self, idle_input=True):
        d = self.dut
        d.rst_i.value = 1
        d.phy_link_up_i.value = 0
        d.phy_fifo_empty_i.value = 0
        d.lane_reverse_i.value = 0
        d.curr_data_rate_i.value = 0  # gen1
        d.data_i.value = 0
        d.data_valid_i.value = 0
        d.data_k_i.value = 0
        d.sync_header_i.value = 0
        d.pipe_width_i.value = 4
        d.num_active_lanes_i.value = 1
        d.m_dllp_axis_tready.value = 1
        await ClockCycles(d.clk_i, 5)
        d.rst_i.value = 0
        d.phy_link_up_i.value = 1
        await ClockCycles(d.clk_i, 2)


class Collector:
    """!! SAMPLES IN ReadOnly, never immediately after RisingEdge -- a bare read
    after RisingEdge is PRE-edge and returns the previous cycle's value.

    !! AND IT LEAVES A WRITABLE PHASE. A collector that ends inside ReadOnly
    makes the CALLER's next write throw; run() ends on a RisingEdge."""

    def __init__(self, dut):
        self.dut = dut
        self.beats = []   # (tdata, tkeep, tlast)

    async def run(self, cycles):
        d = self.dut
        for _ in range(cycles):
            await RisingEdge(d.clk_i)
            await ReadOnly()
            if d.m_dllp_axis_tvalid.value == 1 and d.m_dllp_axis_tready.value == 1:
                self.beats.append(
                    (int(d.m_dllp_axis_tdata.value),
                     int(d.m_dllp_axis_tkeep.value),
                     int(d.m_dllp_axis_tlast.value))
                )
        await RisingEdge(d.clk_i)

    @property
    def lasts(self):
        return [b for b in self.beats if b[2] == 1]

    @property
    def keeps_on_last(self):
        return [b[1] for b in self.lasts]


async def drive(dut, words, gap=0):
    """Drive (data, k) word pairs onto the lane-parallel input."""
    for (data, k) in words:
        dut.data_i.value = data
        dut.data_k_i.value = k
        dut.data_valid_i.value = 1
        await RisingEdge(dut.clk_i)
        for _ in range(gap):
            dut.data_valid_i.value = 0
            await RisingEdge(dut.clk_i)
    dut.data_valid_i.value = 0
    dut.data_k_i.value = 0


# =============================================================================
# Row 6 of the briefed 2a -- the apparatus control. Green by construction on the
# CURRENT RTL, so a green sweep cannot be a silently dead bench.
#
# #7c's P0 caught nothing but would have caught everything if that rung's
# misdiagnosis had gone the other way.
# =============================================================================
@cocotb.test()
async def dh_apparatus_control_idle_produces_no_axis_traffic(dut):
    """With no valid input, data_handler emits no AXIS beat. True of the broken
    RTL and of any correct one -- this row must never be the one that fails."""
    tb = TB(dut)
    await tb.reset()
    col = Collector(dut)
    await col.run(60)
    assert len(col.beats) == 0, (
        f"apparatus control: idle input produced {len(col.beats)} AXIS beats"
    )


# =============================================================================
# Row 1 of the briefed 2a -- the eight-Symbol frame. THE WHOLE DEFECT IN ONE ROW.
# =============================================================================
@cocotb.test()
async def dh_eight_symbol_frame_is_0xF_then_tlast_0x3(dut):
    """SDP + 4 DLLP bytes + 2 CRC bytes + END must yield tkeep 0xF, then tlast
    with tkeep 0x3.

    HISTORY, broken RTL: tkeep on tlast was 0x7, on every one of 21,272 frames
    in the full stack. 0x7 is seven payload bytes for a six-byte DLLP."""
    tb = TB(dut)
    await tb.reset()
    c = Collector(dut)
    col = cocotb.start_soon(c.run(60))
    await drive(dut, dllp_stream(1))
    await col

    assert len(c.lasts) == 1, f"expected exactly one tlast, got {len(c.lasts)}"
    keeps = c.keeps_on_last
    assert keeps == [0x3], (
        f"tkeep on tlast is {[hex(k) for k in keeps]}, expected [0x3]. "
        f"0x7 means the END Symbol is being counted as payload -- seven bytes "
        f"for a six-byte DLLP. Oracle: Base 2.1 §3.4, dllp_handler.sv:129, and "
        f"the measured transmit path (§63 #7d Phase 1)."
    )
    non_last = [b for b in c.beats if b[2] == 0]
    assert all(b[1] == 0xF for b in non_last), (
        f"non-final beats must carry tkeep 0xF, got "
        f"{[hex(b[1]) for b in non_last]}"
    )


# =============================================================================
# Row 3 of the briefed 2a -- the byte count on the last beat, stated as a
# NEGATIVE so that a row accepting "some non-zero tkeep" cannot pass.
# =============================================================================
@cocotb.test()
async def dh_tkeep_on_tlast_is_0x3_never_0x7(dut):
    """The EP signature. Written so that today's RTL fails it: a row asserting
    only "tkeep != 0" would pass the broken module."""
    tb = TB(dut)
    await tb.reset()
    c = Collector(dut)
    col = cocotb.start_soon(c.run(60))
    await drive(dut, dllp_stream(2))
    await col

    keeps = c.keeps_on_last
    assert 0x7 not in keeps, (
        f"tkeep 0x7 on tlast -- END counted as payload. Got "
        f"{[hex(k) for k in keeps]}"
    )
    assert keeps and all(k == 0x3 for k in keeps), (
        f"every tlast must carry tkeep 0x3, got {[hex(k) for k in keeps]}"
    )


# =============================================================================
# Row 2 of the briefed 2a -- tlast exists at all, and exactly once per frame.
# This was the RC signature before defect A was fixed.
# =============================================================================
@cocotb.test()
async def dh_tlast_count_equals_frame_count(dut):
    """Over a window containing N complete DLLPs, tlast asserts exactly N times."""
    tb = TB(dut)
    await tb.reset()
    n = 4
    words = dllp_stream(n)
    c = Collector(dut)
    col = cocotb.start_soon(c.run(120))
    await drive(dut, words)
    await col

    assert len(c.lasts) == n, (
        f"expected {n} tlast for {n} frames, got {len(c.lasts)}"
    )


# =============================================================================
# Row 4 of the briefed 2a -- back-to-back frames, no gap between END and SDP.
# =============================================================================
@cocotb.test()
async def dh_back_to_back_frames_no_gap(dut):
    """END immediately followed by the next SDP, no idle between. Framing must
    not depend on a gap it is not guaranteed."""
    tb = TB(dut)
    await tb.reset()
    n = 3
    words = dllp_stream(n)
    c = Collector(dut)
    col = cocotb.start_soon(c.run(120))
    await drive(dut, words, gap=0)
    await col

    assert len(c.lasts) == n, (
        f"back-to-back: expected {n} tlast, got {len(c.lasts)}"
    )
    assert all(k == 0x3 for k in c.keeps_on_last), (
        f"back-to-back: tkeep on tlast {[hex(k) for k in c.keeps_on_last]}, "
        f"expected all 0x3"
    )


# =============================================================================
# Row 7 of the briefed 2a -- reset durability. P7's lesson from #7c, where a
# stale-data leak was confidently blamed on a racing clock and was the DUT's
# reset.
# =============================================================================
@cocotb.test()
async def dh_reset_is_durable_with_idle_input(dut):
    """Reset while the input is idle; the first beats after release must not be
    pre-reset contents.

    !! block_alignment could NOT be reset while its input was idle (#7c, and
    rst_i does not reset it at all -- the latch reloads on release). This row
    exists because that is a live failure mode in this exact path."""
    tb = TB(dut)
    await tb.reset()
    # Push a frame through, then reset with the input held idle.
    await drive(dut, dllp_stream(1))
    await ClockCycles(dut.clk_i, 5)
    dut.rst_i.value = 1
    await ClockCycles(dut.clk_i, 6)
    dut.rst_i.value = 0
    await ClockCycles(dut.clk_i, 2)

    c = Collector(dut)
    col = cocotb.start_soon(c.run(40))
    await col
    assert len(c.beats) == 0, (
        f"after reset with idle input, {len(c.beats)} beats appeared -- "
        f"pre-reset contents survived the reset"
    )


# =============================================================================
# §63 #7e -- THE TLP ARM. data_handler's TLP path had never been driven by any
# bench; every row above frames with SDP and exercises the DLLP arm only.
#
# == WHY A SWEEP AND NOT ONE HAND-PICKED FRAME =============================
#
# ⚠️ THIS BENCH'S OWN HISTORY IS THE REASON. dllp_stream()'s docstring records
# that its first version assumed `SDP at byte 0, END at byte 3`, data_handler
# PASSED it, and the assumption was the bug in the bench. Picking one TLP
# alignment out of the air would repeat that mistake exactly.
#
# So the row does not pick. It sweeps every STP byte position 0..3 against both
# payload lengths the full stack actually carries, and reports the whole table
# before asserting anything. Whatever the stack's true packing is, it is one of
# these eight cells, and the table says what the module does in all of them.
#
# == THE TWO LENGTHS, FROM BASE 2.1 §3.5 ===================================
#
# On the link a TLP is: STP + 2 B sequence number + header (+ data) + 4 B LCRC
# + END. The two that cross this link today, MEASURED at §63 #7e Phase 1:
#
#   CfgRd0  3 DW header, no data  -> 2 + 12 + 4     = 18 B payload
#   CplD    3 DW header, 1 DW data -> 2 + 12 + 4 + 4 = 22 B payload
#
# Both leave 2 valid bytes in the final 32-bit beat (18 % 4 == 22 % 4 == 2), so
# the oracle for both is: ceil(n/4) beats, 0xF on every beat but the last, one
# tlast, tkeep 0x3 on it.
#
# == PRE-FIX, MEASURED IN THE FULL STACK AT 6436f0e (§22.87) ===============
#
#   EP, 18 B inbound:  20 beats / 4 packets, tlast 4, tkeep 0x3   -- correct
#   RC, 22 B inbound:  16 beats / 4 packets, tlast 0              -- F17
#
# The RC emits FOUR beats where six are due and never asserts tlast, so the
# Completion never becomes a packet and enumeration times out.
# =============================================================================


def tlp_stream(payload_len, stp_pos=0, pad=0x00):
    """One STP-framed TLP: STP, `payload_len` payload bytes, END.

    `stp_pos` is the byte index of STP within its 32-bit word, so the caller can
    sweep the alignment instead of assuming one. Payload bytes are distinct and
    non-K so a truncation shows up as missing CONTENT, not just a short count.
    """
    syms = [(pad, 0)] * stp_pos + [(STP, 1)]
    syms += [(((0x10 + i) & 0xFF), 0) for i in range(payload_len)]
    syms += [(END, 1)]
    while len(syms) % 4:
        syms.append((pad, 0))

    # ⚠️⚠️ TRAILING IDLE IS LOAD-BEARING, NOT COSMETIC -- AND ITS ABSENCE
    # MANUFACTURED A FALSE DEFECT IN THIS ROW'S FIRST RUN.
    #
    # data_handler emits a beat assembled from `word_count_r` carry-over bytes
    # of the REGISTERED word plus the low bytes of the current one. When END
    # falls in the carry-over region, :241's loop correctly declines it (those
    # bytes belong to the NEXT beat) and :268's loop catches it one cycle later
    # off data_k_r. That second loop needs ONE MORE VALID INPUT WORD to run.
    #
    # Without trailing idle, `stp_at_byte=0` produces a stream that is an exact
    # multiple of four symbols, so no padding word is appended, so the flush
    # cycle never arrives -- and the row reported "beats=4 tlast=0", which reads
    # exactly like F17. For stp_at_byte 1/2/3 the padding to a word boundary
    # supplied that cycle by accident and the same RTL looked correct.
    #
    # A real link is never silent after a TLP: END is followed by IDL/COM or the
    # next frame. Modelling that is what makes this a measurement of the module
    # rather than of the stimulus. Two words, because one is the flush and the
    # second proves nothing further is emitted.
    #
    # This is the same class as dllp_stream()'s recorded trap -- the bench's
    # assumption, not the DUT -- caught the second time by asking why only the
    # word-aligned cells failed.
    syms += [(IDL, 1)] * 8

    words = []
    for w in range(0, len(syms), 4):
        chunk = syms[w:w + 4]
        words.append((
            word(chunk[0][0], chunk[1][0], chunk[2][0], chunk[3][0]),
            kmask(*[i for i in range(4) if chunk[i][1]]),
        ))
    return words


def _tlp_oracle(payload_len):
    """(beats, tkeep_on_last) required by Base 2.1 §3.5 for an n-byte payload."""
    beats = (payload_len + 3) // 4
    rem = payload_len % 4
    return beats, (0xF if rem == 0 else (1 << rem) - 1)


@cocotb.test()
async def dh_tlp_arm_sweep_alignment_and_length(dut):
    """CHARACTERISATION + ORACLE for the TLP arm, all 4 alignments x 2 lengths.

    ⚠️⚠️ RED WHEN WRITTEN (§63 #7e, F17). Prints the full table first so a
    failure names WHICH cells break rather than only the first one -- the same
    diagnostics-before-verdicts rule tb_pcie_fullstack's _run_and_report uses.
    """
    table = {}
    for payload_len in (18, 22):
        for stp_pos in range(4):
            tb = TB(dut)
            await tb.reset()
            c = Collector(dut)
            col = cocotb.start_soon(c.run(80))
            await drive(dut, tlp_stream(payload_len, stp_pos))
            await col
            table[(payload_len, stp_pos)] = (
                len(c.beats), len(c.lasts), [hex(k) for k in c.keeps_on_last])

    for (n, p), (beats, lasts, keeps) in sorted(table.items()):
        want_beats, want_keep = _tlp_oracle(n)
        ok = (beats == want_beats and lasts == 1 and keeps == [hex(want_keep)])
        dut._log.info(
            "TLPSWEEP payload=%2dB stp_at_byte=%d -> beats=%d tlast=%d "
            "tkeep_on_last=%s  | want beats=%d tlast=1 tkeep=%s  %s",
            n, p, beats, lasts, keeps, want_beats, hex(want_keep),
            "OK" if ok else "**MISMATCH**")

    bad = []
    for (n, p), (beats, lasts, keeps) in sorted(table.items()):
        want_beats, want_keep = _tlp_oracle(n)
        if beats != want_beats or lasts != 1 or keeps != [hex(want_keep)]:
            bad.append(
                f"payload={n}B stp_at_byte={p}: got beats={beats} tlast={lasts} "
                f"tkeep={keeps}, want beats={want_beats} tlast=1 "
                f"tkeep=['{hex(want_keep)}']")

    assert not bad, (
        "data_handler's TLP arm does not reconstruct an STP-framed TLP:\n  "
        + "\n  ".join(bad)
        + "\n\nOracle: Base 2.1 §3.5 -- an n-byte link payload is ceil(n/4) "
          "beats with (n mod 4) valid bytes on the last. F17: in the full "
          "stack the RC's 22 B Completion emits 4 beats and no tlast, so it "
          "never becomes a packet and enumeration reports ENUM_ERR_TIMEOUT."
    )


@cocotb.test()
async def dh_tlp_arm_tlast_exists_at_all(dut):
    """The minimal F17 row: an STP-framed TLP must produce exactly one tlast.

    Separate from the sweep on purpose. The sweep can fail for a tkeep reason
    OR a tlast reason; this one fails ONLY if the packet never terminates, which
    is the specific thing that makes the Root Complex drop every Completion.
    Kept narrow so that a later partial fix cannot leave it ambiguous.
    """
    tb = TB(dut)
    await tb.reset()
    c = Collector(dut)
    col = cocotb.start_soon(c.run(80))
    await drive(dut, tlp_stream(22, stp_pos=0))
    await col

    assert len(c.beats) > 0, (
        "NON-VACUITY: the TLP arm produced no AXIS beats at all, so this row "
        "asserted nothing about tlast"
    )
    assert len(c.lasts) == 1, (
        f"an STP-framed 22 B TLP produced {len(c.lasts)} tlast over "
        f"{len(c.beats)} beats, expected exactly 1. Without tlast the packet "
        "never completes: axis_user_demux forwards the beats, dllp2tlp never "
        "sees a packet boundary, and nothing reaches the Transaction Layer."
    )


# =============================================================================
# §63 #7g-3 D2 -- the registered-word END arm, registered since #7d as
# "data_handler.sv:256" (now :297-298) and carried as a suspect for four rungs.
#
# ⭐ IT IS NOT A DEFECT, and this row is what says so from now on.  The arm emits
#     tkeep = 4'hF >> ((BytesPerTransfer - word_count_r) + (BytesPerTransfer - b))
# for an END in carry-over byte b of the registered word, which is
# word_count_r + b - 4 ones: the payload bytes before END, i.e. correct.  Every
# legal Gen1 frame is 0 mod 4 Symbols long (DLLP 8, TLP 4n + 8), so END lands at
# b = s - 1 (mod 4) for a start at byte s, and it is in the carry-over region
# ONLY when s = 0 (count 2 -> 0x3).  Measured at #7g-3 Phase 1: 16 / 16 cells,
# and in the gate `verilate_rc_top` drives this arm 63 times (its far end puts
# SDP at byte 0) while the full stack never does (every frame at s = 2).
#
# NON-VACUITY is asserted, not assumed: the arm is read off its own RTL guard in
# ReadOnly (ST_TX = 1, a beat taken, !data_start_r, END/EDB K in data_r), and it
# must fire exactly once in each s = 0 cell and never at s = 1..3.  A formula
# mutant at :298 fails the s = 0 cells with 0x7 (FIX-phase prediction F-3).
#
# ⚠️ Stimulus is cycle-for-cycle Phase 1's test_7g3_dh256 (3,672 ns), so this
# row's duration is predictable; keep it that way or re-predict the gate.
# =============================================================================
LIDL_7G3 = 0x00  # Logical Idle: a data Symbol 00h, K = 0 (Base 2.1 §4.2.3 p.199)


def _frame_syms_7g3(kind, n, s):
    start = SDP if kind == 'DLLP' else STP
    plen = 6 if kind == 'DLLP' else 2 + 4 * n + 4
    payload = [((0x10 + i) & 0xFF) for i in range(plen)]
    syms = [(LIDL_7G3, 0)] * s + [(start, 1)] + [(p, 0) for p in payload] + [(END, 1)]
    syms += [(LIDL_7G3, 0)] * 8
    while len(syms) % 4:
        syms.append((LIDL_7G3, 0))
    words = []
    for w in range(0, len(syms), 4):
        c = syms[w:w + 4]
        words.append((word(c[0][0], c[1][0], c[2][0], c[3][0]),
                      kmask(*[i for i in range(4) if c[i][1]])))
    return payload, words


async def _run_cell_7g3(dut, tb, kind, n, s):
    await tb.reset()
    payload, words = _frame_syms_7g3(kind, n, s)
    beats, arm = [], []
    done = False

    async def watch():
        while not done:
            await RisingEdge(dut.clk_i)
            await ReadOnly()
            if (int(dut.curr_state.value) == 1 and int(dut.data_handler_axis_tready.value) == 1
                    and int(dut.data_valid_i.value) != 0 and int(dut.data_start_r.value) == 0):
                dr, kr = int(dut.data_r.value), int(dut.data_k_r.value)
                for b in range(4):
                    if (kr >> b) & 1 and ((dr >> (8 * b)) & 0xFF) in (0xFD, 0xFE):
                        arm.append(b)
            if int(dut.m_dllp_axis_tvalid.value) and int(dut.m_dllp_axis_tready.value):
                beats.append((int(dut.m_dllp_axis_tdata.value), int(dut.m_dllp_axis_tkeep.value),
                              int(dut.m_dllp_axis_tlast.value)))

    cocotb.start_soon(watch())
    for (d, k) in words:
        dut.data_i.value = d
        dut.data_k_i.value = k
        dut.data_valid_i.value = 1
        await RisingEdge(dut.clk_i)
    dut.data_valid_i.value = 0
    dut.data_k_i.value = 0
    for _ in range(12):
        await RisingEdge(dut.clk_i)
    done = True
    await RisingEdge(dut.clk_i)
    await RisingEdge(dut.clk_i)
    out = []
    for (td, tk, tl) in beats:
        for b in range(4):
            if (tk >> b) & 1:
                out.append((td >> (8 * b)) & 0xFF)
    lasts = [b for b in beats if b[2] == 1]
    return dict(kind=kind, n=n, s=s, beats=len(beats), lasts=len(lasts),
                keeps=[b[1] for b in lasts], payload_ok=(out == payload), arm=arm)


@cocotb.test()
async def dh_registered_end_arm_every_legal_frame_all_alignments(dut):
    """Every legal frame (DLLP; TLP n = 3, 4, 5 DW) at every start byte s = 0..3
    leaves data_handler as ONE packet: exactly one tlast, tkeep 0x3 on it, and
    the payload byte-exact.  The registered-word END arm (registered as
    data_handler:256) carries the END exactly when s = 0, and only then.

    Oracle: Base 2.1 §3.4 / §3.5 framing (DLLP 6 payload bytes; TLP 2 + 4n + 4),
    so the tlast beat always holds 2 bytes (payload = 4n + 6 = 2 mod 4)."""
    tb = TB(dut)  # ONE clock for the whole row; reset() per cell
    cells = []
    for kind, n in (('DLLP', 0), ('TLP', 3), ('TLP', 4), ('TLP', 5)):
        for s in range(4):
            cells.append(await _run_cell_7g3(dut, tb, kind, n, s))
    for c in cells:
        dut._log.info(
            f"D2|{c['kind']}|n={c['n']}|s={c['s']}|beats={c['beats']}|tlast={c['lasts']}"
            f"|tkeep_last={[hex(k) for k in c['keeps']]}|payload_ok={int(c['payload_ok'])}"
            f"|reg_arm={c['arm']}")
    bad = [c for c in cells if c['lasts'] != 1 or c['keeps'] != [0x3] or not c['payload_ok']]
    assert not bad, (
        f"{len(bad)} of {len(cells)} legal frames did not leave as one packet with tkeep 0x3 "
        f"and exact payload: {[(c['kind'], c['n'], c['s'], [hex(k) for k in c['keeps']]) for c in bad]}"
    )
    arm_wrong = [c for c in cells if c['arm'] != ([3] if c['s'] == 0 else [])]
    assert not arm_wrong, (
        "NON-VACUITY / alignment: the registered-word END arm must fire exactly once, at byte 3, "
        "in every s = 0 cell and never at s = 1..3; "
        f"got {[(c['kind'], c['n'], c['s'], c['arm']) for c in arm_wrong]}"
    )
