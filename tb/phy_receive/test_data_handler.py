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
