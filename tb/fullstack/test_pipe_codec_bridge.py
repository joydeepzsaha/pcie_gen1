"""pipe_codec_bridge alone -- SS63 #7b Phase 2's probe rows.

!! THESE ROWS RUN BEFORE THE BRIDGE TOUCHES A DUT, AND THAT ORDER IS THE POINT.
A codec bug found here costs one 3-second run; the same bug found inside the
full-stack bench looks like "Joy's Endpoint does not train" and costs a day.

FOUR ROWS, and the fourth is the one that matters:

  1. round trip -- a character stream returns byte-identical, K flags included
  2. K28.5 keeps its K flag, with a NEGATIVE CONTROL through the same path
  3. running disparity does not walk, checked at the WIRE not at the round trip
  4. valid-gating -- a stream with gaps returns identically to one without

Row 4 exists because the header names disparity gating as the one way to get
this bridge wrong: the EP advances its own running disparity only on a valid
beat (pcie_endpoint_top.sv:488-503), so a bridge that advances on an idle beat
desynchronises and every later symbol decodes as a disparity error. Row 4 is a
DIFFERENTIAL check -- same characters, once dense and once sparse, outputs
compared -- so it cannot pass by the gaps simply not mattering.

!! ROW 1 ALONE WOULD NOT CATCH AN ENCODER AND DECODER WRONG IN OPPOSITE WAYS.
encode followed by decode is an identity on the character stream, so a
consistent pair of errors round-trips perfectly. Row 3 therefore asserts about
the SYMBOLS on the wire (b_symbol_o), which is why that port is exposed at all.

!! ONE TB PER TEST. cocotb cancels every task a test started when that test
ends, including the Clock coroutine. A shared TB has a dead clock and the next
RisingEdge never returns, which reads as a reset bug in the DUT.
"""

import random

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ClockCycles

CLK_NS = 8  # 125 MHz -- the real Gen1 PCLK

# Base 2.1 Appendix B. K28.5 is COM, the comma every ordered set starts with.
K28_5 = 0xBC
K28_0 = 0x1C
K27_7 = 0xFB  # STP
K29_7 = 0xFD  # END

# Round-trip latency: one cycle for the encode register, one for the decode
# register. Asserted in row 1 rather than assumed, so a latency change fails
# loudly instead of silently shifting every comparison.
BRIDGE_LATENCY = 2


class TB:
    def __init__(self, dut):
        self.dut = dut
        cocotb.start_soon(Clock(dut.clk_i, CLK_NS, units="ns").start())

    async def reset(self):
        d = self.dut
        d.rst_i.value = 1
        d.a_txdata_i.value = 0
        d.a_txdatak_i.value = 0
        d.a_txdata_valid_i.value = 0
        await ClockCycles(d.clk_i, 5)
        d.rst_i.value = 0
        await ClockCycles(d.clk_i, 2)


async def drive(dut, beats):
    """Drive (char0, k0, char1, k1, valid) tuples, one per cycle, then idle.

    Assigns AFTER the edge, so each tuple is stable for exactly one cycle.
    """
    for c0, k0, c1, k1, v in beats:
        await RisingEdge(dut.clk_i)
        dut.a_txdata_i.value = (c1 << 8) | c0
        dut.a_txdatak_i.value = (k1 << 1) | k0
        dut.a_txdata_valid_i.value = v
    await RisingEdge(dut.clk_i)
    dut.a_txdata_valid_i.value = 0


class RxCollector:
    """Collects returned characters, gated on the returned valid.

    !! DATA AND VALID ARE SAMPLED AT THE SAME PHASE, which is what keeps them
    aligned (SS22.89). The absolute phase does not matter here -- a uniform
    one-edge offset shifts both equally -- but a SPLIT phase would silently
    pair each character with its neighbour's valid.
    """

    def __init__(self, dut):
        self.dut = dut
        self.chars = []
        self.valid_cycles = 0

    async def run(self, cycles):
        for _ in range(cycles):
            await RisingEdge(self.dut.clk_i)
            if int(self.dut.a_rxdata_valid_o.value):
                d = int(self.dut.a_rxdata_o.value)
                k = int(self.dut.a_rxdatak_o.value)
                self.valid_cycles += 1
                self.chars.append((d & 0xFF, k & 1, (d >> 8) & 0xFF, (k >> 1) & 1))
                # The bridge drives the unused upper half to zero rather than
                # leaving it undriven; assert that rather than masking it away,
                # because an X here propagates into the RC's block_alignment.
                assert (d >> 16) == 0, f"upper 16 data bits not zero: {d:#010x}"
                assert (k >> 2) == 0, f"upper 2 K flags not zero: {k:#x}"


class SymbolCollector:
    """Collects the 10-bit symbols on the wire, gated on the symbol valid.

    Row 3's subject. Also computes the running digital sum, which is what
    "disparity does not walk" actually means.
    """

    def __init__(self, dut):
        self.dut = dut
        self.symbols = []

    async def run(self, cycles):
        for _ in range(cycles):
            await RisingEdge(self.dut.clk_i)
            if int(self.dut.b_symbol_valid_o.value):
                s = int(self.dut.b_symbol_o.value)
                self.symbols.append(s & 0x3FF)
                self.symbols.append((s >> 10) & 0x3FF)


def check_no_codec_errors(dut, where):
    assert int(dut.enc_illegal_k_o.value) == 0, (
        f"{where}: the encoder was asked for a K code-group Base 2.1 Appendix B "
        f"does not define"
    )
    assert int(dut.dec_code_err_o.value) == 0, (
        f"{where}: decode_8b10b reported code_err -- a 10-bit group that is not "
        f"a valid code group reached the decoder"
    )
    assert int(dut.dec_disp_err_o.value) == 0, (
        f"{where}: decode_8b10b reported disp_err -- the running-disparity "
        f"chains desynchronised, which is the valid-gating defect"
    )


# ---------------------------------------------------------------------------
# Row 1 -- the round trip
# ---------------------------------------------------------------------------
@cocotb.test()
async def bridge_round_trips_byte_identical(dut):
    """A character stream returns byte-identical, K flags included.

    NON-VACUITY, three limbs, because a bridge that returned nothing at all
    would otherwise pass an empty comparison:
      - at least 200 beats came back;
      - the stream contained at least 40 distinct data characters;
      - the stream contained at least one K character.
    """
    tb = TB(dut)
    await tb.reset()

    rng = random.Random(0x7B01)
    beats = []
    for _ in range(256):
        # A realistic mix: mostly data, with the four K characters the link
        # actually uses appearing often enough to be checked.
        if rng.random() < 0.15:
            c0, k0 = rng.choice([K28_5, K28_0, K27_7, K29_7]), 1
        else:
            c0, k0 = rng.randrange(256), 0
        if rng.random() < 0.15:
            c1, k1 = rng.choice([K28_5, K28_0, K27_7, K29_7]), 1
        else:
            c1, k1 = rng.randrange(256), 0
        beats.append((c0, k0, c1, k1, 1))

    rx = RxCollector(dut)
    mon = cocotb.start_soon(rx.run(len(beats) + 3 * BRIDGE_LATENCY + 20))
    await drive(dut, beats)
    await mon

    sent = [(c0, k0, c1, k1) for c0, k0, c1, k1, v in beats if v]
    got = rx.chars

    # --- non-vacuity ------------------------------------------------------
    assert len(got) >= 200, (
        f"non-vacuity failed: only {len(got)} beats returned; the comparison "
        f"below would be nearly empty"
    )
    distinct = {c for c0, k0, c1, k1 in sent for c in (c0, c1)}
    assert len(distinct) >= 40, (
        f"non-vacuity failed: only {len(distinct)} distinct characters in the "
        f"stimulus"
    )
    assert any(k0 or k1 for _, k0, _, k1 in sent), (
        "non-vacuity failed: the stimulus contained no K character, so the K "
        "flag limb of this row asserts nothing"
    )

    assert len(got) == len(sent), (
        f"beat count changed across the bridge: sent {len(sent)}, got "
        f"{len(got)}. The bridge must not create or drop beats"
    )
    for i, (s, g) in enumerate(zip(sent, got)):
        assert s == g, (
            f"beat {i} differs: sent (c0={s[0]:#04x} k0={s[1]} c1={s[2]:#04x} "
            f"k1={s[3]}), got (c0={g[0]:#04x} k0={g[1]} c1={g[2]:#04x} "
            f"k1={g[3]})"
        )

    check_no_codec_errors(dut, "round trip")
    dut._log.info(
        "ROW 1: %d beats round-tripped byte-identical, %d distinct characters, "
        "K flags preserved", len(got), len(distinct),
    )


# ---------------------------------------------------------------------------
# Row 2 -- K28.5 keeps its K flag, with a negative control
# ---------------------------------------------------------------------------
@cocotb.test()
async def bridge_preserves_k_flag_with_negative_control(dut):
    """K28.5 survives as a K character, and 0xBC-as-DATA survives as data.

    !! THE NEGATIVE CONTROL IS THE ROW (SS22.81). "K28.5 came back with k=1" is
    satisfied by a bridge that sets k=1 on everything. Driving the SAME byte
    value 0xBC with k=0 through the SAME path, and requiring it back with k=0,
    is what makes the positive limb mean anything.

    Second limb: the two must occupy DIFFERENT code groups on the wire. K28.5
    and D28.5 share a byte value and are distinct 10-bit groups; if the bridge
    ignored the K bit they would be identical there. This is checked at the
    wire because the round trip alone cannot see it.
    """
    tb = TB(dut)
    await tb.reset()

    # Beat 0: K28.5 as a K. Beat 1: the same byte as DATA.
    beats = [(K28_5, 1, K28_5, 1, 1), (K28_5, 0, K28_5, 0, 1)]

    rx = RxCollector(dut)
    sym = SymbolCollector(dut)
    mon_rx = cocotb.start_soon(rx.run(len(beats) + 3 * BRIDGE_LATENCY + 20))
    mon_sym = cocotb.start_soon(sym.run(len(beats) + 3 * BRIDGE_LATENCY + 20))
    await drive(dut, beats)
    await mon_rx
    await mon_sym

    assert len(rx.chars) == 2, f"expected 2 beats back, got {len(rx.chars)}"

    c0, k0, c1, k1 = rx.chars[0]
    assert (c0, k0) == (K28_5, 1) and (c1, k1) == (K28_5, 1), (
        f"K28.5 did not survive as a K character: got c0={c0:#04x} k0={k0} "
        f"c1={c1:#04x} k1={k1}"
    )

    # --- the negative control, through the same path ----------------------
    c0, k0, c1, k1 = rx.chars[1]
    assert (c0, k0) == (K28_5, 0) and (c1, k1) == (K28_5, 0), (
        f"the negative control failed: 0x{K28_5:02x} driven as DATA came back "
        f"as c0={c0:#04x} k0={k0} c1={c1:#04x} k1={k1}. If k0/k1 are 1 the "
        f"bridge is asserting K regardless of input and the positive limb "
        f"above proves nothing"
    )

    # --- and they are the code groups Appendix B names --------------------
    #
    # Bit order is the codec's own: dataout[0]=a .. [4]=e, [5]=i, [6]=f, [7]=g,
    # [8]=h, [9]=j (decode_8b10b.sv:15-24). Both symbols below are the RD-
    # variants, which is what the bridge's reset value of 0 selects.
    #
    #   K28.5 RD-  abcdei fghj = 001111 1010 -> 0x17c   <- the COMMA
    #   D28.5 RD-  abcdei fghj = 001110 1010 -> 0x15c
    #
    # They differ in ONE BIT (i), and that bit is what makes K28.5 the only
    # group carrying the 0011111/1100000 singular comma pattern. Asserting the
    # exact values rather than merely "they differ" is what makes this row
    # spec-golden: an encoder that swapped the two, or emitted some third
    # distinct group, would satisfy inequality and fail here.
    K28_5_RD_MINUS = 0x17C
    D28_5_RD_MINUS = 0x15C

    assert len(sym.symbols) >= 4, (
        f"non-vacuity failed: only {len(sym.symbols)} symbols observed on the "
        f"wire"
    )
    k_group = sym.symbols[0]
    d_group = sym.symbols[2]
    assert k_group != d_group, (
        f"K28.5 and D28.5 encoded to the SAME code group {k_group:#05x}; the "
        f"encoder is ignoring the K bit and the round trip cannot see it"
    )
    assert k_group == K28_5_RD_MINUS, (
        f"K28.5 at RD- encoded to {k_group:#05x}, Base 2.1 Appendix B says "
        f"{K28_5_RD_MINUS:#05x} (001111 1010)"
    )
    assert d_group == D28_5_RD_MINUS, (
        f"D28.5 at RD- encoded to {d_group:#05x}, Base 2.1 Appendix B says "
        f"{D28_5_RD_MINUS:#05x} (001110 1010)"
    )

    check_no_codec_errors(dut, "K flag")
    dut._log.info(
        "ROW 2: K28.5 -> group 0x%03x, D28.5 -> group 0x%03x, both round-trip "
        "with the correct K flag", k_group, d_group,
    )


# ---------------------------------------------------------------------------
# Row 3 -- running disparity does not walk
# ---------------------------------------------------------------------------
@cocotb.test()
async def bridge_running_disparity_does_not_walk(dut):
    """Running disparity is +/-1 after every code group, and no disp_err fires.

    !! CHECKED AT THE WIRE, NOT AT THE ROUND TRIP. The round trip is an
    identity whatever the disparity does, so it is blind to this.

    The property is Base 2.1 Appendix B's defining one: after every complete
    10-bit code group the running disparity is either +1 or -1, never anything
    else. RD starts at -1, which is what the bridge's reset value of 0 means --
    encode_8b10b's own port comment reads "0 = neg disp; 1 = pos disp".

    !! THE FIRST VERSION OF THIS ROW ACCUMULATED (ones - zeros) FROM ZERO AND
    REQUIRED THE SUM TO VISIT BOTH SIGNS. It failed, and it was right to: that
    accumulator measures RD - RD_initial, so with RD_initial = -1 it ranges over
    {0, +2} and can NEVER go negative. The non-vacuity limb was unsatisfiable by
    construction and the row would have been impossible to pass honestly.
    Tracking RD itself instead is both the spec's own statement and strictly
    stronger -- |RD| <= 1 after every group, rather than |sum| <= 2.

    NON-VACUITY: both polarities must actually be visited, which is now a real
    requirement rather than an impossible one.
    """
    tb = TB(dut)
    await tb.reset()

    rng = random.Random(0x7B03)
    beats = [(rng.randrange(256), 0, rng.randrange(256), 0, 1) for _ in range(512)]
    # A comma every so often, which is what a real link does.
    for i in range(0, len(beats), 32):
        beats[i] = (K28_5, 1, beats[i][2], 0, 1)

    sym = SymbolCollector(dut)
    mon = cocotb.start_soon(sym.run(len(beats) + 3 * BRIDGE_LATENCY + 20))
    await drive(dut, beats)
    await mon

    assert len(sym.symbols) >= 1000, (
        f"non-vacuity failed: only {len(sym.symbols)} symbols on the wire"
    )

    rd = -1  # encode_8b10b: dispin 0 == negative disparity; the bridge resets to 0
    seen_pos = False
    seen_neg = False
    neutral = 0
    for i, s in enumerate(sym.symbols):
        ones = bin(s).count("1")
        imbalance = ones - (10 - ones)
        assert imbalance in (-2, 0, 2), (
            f"symbol {i} ({s:#05x}) has {ones} ones; a valid 10-bit code group "
            f"has 4, 5 or 6, so this is not a code group at all"
        )
        if imbalance == 0:
            neutral += 1
        rd += imbalance
        assert rd in (-1, 1), (
            f"running disparity reached {rd} after symbol {i} ({s:#05x}); "
            f"Base 2.1 Appendix B requires +1 or -1 after every code group, so "
            f"the disparity chain has walked"
        )
        if rd > 0:
            seen_pos = True
        else:
            seen_neg = True

    assert seen_pos and seen_neg, (
        f"non-vacuity failed: running disparity never visited both polarities "
        f"(pos={seen_pos} neg={seen_neg}), so the bound above is trivial"
    )

    check_no_codec_errors(dut, "disparity walk")
    dut._log.info(
        "ROW 3: %d symbols, RD stayed in {-1,+1} throughout, both polarities "
        "visited, %d disparity-neutral groups, zero disp_err",
        len(sym.symbols), neutral,
    )


# ---------------------------------------------------------------------------
# Row 4 -- valid-gating, differentially
# ---------------------------------------------------------------------------
@cocotb.test()
async def bridge_disparity_is_gated_on_valid(dut):
    """The same characters, dense and sparse, return identically.

    !! THIS IS THE ROW THE BRIDGE EXISTS TO PASS. The EP advances its own
    running disparity only on a valid beat (pcie_endpoint_top.sv:488-503). A
    bridge that advances on an IDLE beat desynchronises from it and every later
    symbol decodes as a disparity error -- a failure that reads as a codec bug
    and is a gating bug.

    DIFFERENTIAL, so it cannot pass by the gaps not mattering: the identical
    character sequence is driven twice, once with valid high on every cycle and
    once with idle cycles interleaved, and the two returned sequences must be
    equal. If disparity advanced on idle beats the sparse run would diverge
    from the dense one at the first gap.

    The measured duty on the real RC seam is ~80%, not 100% (Phase 0 SS1.2), so
    gaps are the NORMAL case here rather than a contrived one.
    """
    rng = random.Random(0x7B04)
    chars = [(rng.randrange(256), 0, rng.randrange(256), 0) for _ in range(192)]
    for i in range(0, len(chars), 16):
        chars[i] = (K28_5, 1, chars[i][2], 0)

    async def run_once(gapped):
        tb = TB(dut)
        await tb.reset()
        beats = []
        for idx, (c0, k0, c1, k1) in enumerate(chars):
            if gapped and idx % 5 in (3, 4):
                # Idle beats carry a DIFFERENT character with valid low, so a
                # bridge that ignored the valid would be caught by the data as
                # well as by the disparity.
                beats.append((0x5A, 0, 0xA5, 0, 0))
            beats.append((c0, k0, c1, k1, 1))
        rx = RxCollector(dut)
        mon = cocotb.start_soon(rx.run(len(beats) + 3 * BRIDGE_LATENCY + 20))
        await drive(dut, beats)
        await mon
        check_no_codec_errors(dut, "gapped" if gapped else "dense")
        return rx.chars

    dense = await run_once(False)
    sparse = await run_once(True)

    assert len(dense) == len(chars), (
        f"dense run returned {len(dense)} beats, expected {len(chars)}"
    )
    assert len(sparse) == len(chars), (
        f"sparse run returned {len(sparse)} beats, expected {len(chars)}. If "
        f"this is larger, the bridge passed idle beats through and the valid "
        f"is not gating the datapath"
    )
    assert dense == list(chars), "the dense run did not round-trip correctly"

    for i, (d, s) in enumerate(zip(dense, sparse)):
        assert d == s, (
            f"beat {i} diverges between the dense and sparse runs: dense={d} "
            f"sparse={s}. The disparity chain advanced on an idle beat, which "
            f"is exactly the defect that desynchronises this bridge from the "
            f"Endpoint's own disparity registers"
        )

    dut._log.info(
        "ROW 4: %d beats identical between a 100%%-duty run and one with 40%% "
        "idle beats interleaved; disparity is gated on valid", len(dense),
    )
