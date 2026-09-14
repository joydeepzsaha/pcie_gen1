"""block_alignment -- the module's FIRST bench. SS63 #7c Phase 1.

!! THIS BENCH IS WRITTEN AGAINST THE BROKEN RTL AND IS RED BEFORE THE FIX.
That order is the point: rows that only ever existed after a fix cannot tell you
the fix did anything. Every row below is expected to fail on the RTL as it
stands at 8aadb4c, and the fix is what turns them green.

!! WHY THE MODULE HAD NO BENCH UNTIL NOW, which is itself a finding. Census at
SS63 #7b: no gate target drives block_alignment as toplevel. The module carries
148 of the design's 149 latch primitives and a live functional defect, and it
has never been exercised on its own -- only transitively, through phy_receive,
by rows asserting things about other modules.

== THE DEFECT THESE ROWS PIN =============================================

block_alignment.sv's `always_comb` has NO `D = Q;` default and writes
D.data_valid ONLY inside `if (phy_link_up_i & |data_valid_i)`. So the struct
LATCHES, and data_valid_o -- once high -- never falls. Measured in the
full-stack bench: 30,000 valid beats out of a 30,000-cycle window, against
19,760 supplied by its input. Downstream, pack_data re-samples the held value
and writes the same 2-byte PIPE beat into both halves of a 4-byte word, which
destroys DLLP delineation and is why flow-control initialisation never
completes (F16).

The sibling module says it in words. gen1_scramble.sv:110-118, already fixed for
this class: "Stage 0's valid is the record of WHETHER THIS CLOCK CARRIED A
SYMBOL ... Moving it inside the guard would make it hold high through idle,
which is exactly the duplicate-and-drop behaviour ... registered as tracker
sec 54 4b."

== THE FIVE ACCEPTANCE PROPERTIES (Kourosh, SS63 #7c) =====================

  1. valid out == valid in        -- beat counts equal over the window
  2. no duplicates                -- no output beat repeats its predecessor
                                     unless the input did
  3. no drops                     -- every input word appears at the output
  4. data preserved through idle  -- an idle gap does not corrupt the word
                                     either side of it
  5. valid falls after latency    -- data_valid_o returns low once the pipeline
                                     has drained, and stays low while idle

!! ROWS 2 AND 3 ARE DIFFERENTIAL AGAINST THE INPUT, not absolute. A stream that
legitimately repeats a word must not be scored as a duplicate -- otherwise the
row measures the stimulus rather than the DUT.

!! ONE TB PER TEST. cocotb cancels every task a test started when that test
ends, including the Clock coroutine.
"""

import random

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ClockCycles

CLK_NS = 8
LANES = 1
PIPE_WIDTH_GEN1 = 16          # bits per lane per beat; lane_management.sv:45
NUM_PIPELINES = 4             # block_alignment.sv:35 -- the pipeline depth
GEN1 = 1                      # rate_speed_e'(gen1)


class TB:
    def __init__(self, dut):
        self.dut = dut
        cocotb.start_soon(Clock(dut.clk_i, CLK_NS, units="ns").start())

    async def reset(self):
        d = self.dut
        d.rst_i.value = 1
        d.phy_link_up_i.value = 0
        d.lane_reverse_i.value = 0
        d.curr_data_rate_i.value = GEN1
        d.data_i.value = 0
        d.data_valid_i.value = 0
        d.data_k_i.value = 0
        d.sync_header_i.value = 0
        d.pipe_width_i.value = PIPE_WIDTH_GEN1
        d.num_active_lanes_i.value = LANES
        await ClockCycles(d.clk_i, 5)
        d.rst_i.value = 0
        d.phy_link_up_i.value = 1
        await ClockCycles(d.clk_i, 2)


class Collector:
    """Samples the input and output streams at the SAME phase, continuously.

    !! SAME PHASE IS WHAT KEEPS THEM COMPARABLE (SS22.89). The absolute phase
    does not matter -- a uniform offset shifts both -- but a split phase would
    pair each word with its neighbour's valid and every count below would be
    off by the pipeline depth for the wrong reason.
    """

    def __init__(self, dut):
        self.dut = dut
        self.in_beats = []      # (data, k) on cycles where data_valid_i is high
        self.out_beats = []     # (data, k) on cycles where data_valid_o is high
        self.out_valid_trace = []
        self.in_valid_trace = []

    async def run(self, cycles):
        d = self.dut
        for _ in range(cycles):
            await RisingEdge(d.clk_i)
            iv = int(d.data_valid_i.value) & 1
            ov = int(d.data_valid_o.value) & 1
            self.in_valid_trace.append(iv)
            self.out_valid_trace.append(ov)
            if iv:
                self.in_beats.append((int(d.data_i.value) & 0xFFFFFFFF,
                                      int(d.data_k_i.value) & 0xF))
            if ov:
                self.out_beats.append((int(d.data_o.value) & 0xFFFFFFFF,
                                       int(d.data_k_o.value) & 0xF))


def dup_runs(beats):
    """Count beats identical to their immediate predecessor."""
    return sum(1 for i in range(1, len(beats)) if beats[i] == beats[i - 1])


async def drive(dut, words):
    """Drive (data, k, valid) one per cycle, then go idle."""
    for data, k, v in words:
        await RisingEdge(dut.clk_i)
        dut.data_i.value = data
        dut.data_k_i.value = k
        dut.data_valid_i.value = v
    await RisingEdge(dut.clk_i)
    dut.data_valid_i.value = 0
    dut.data_i.value = 0
    dut.data_k_i.value = 0


def make_stream(seed, n, duty):
    """A stream with a known valid duty and NO accidental repeats.

    Consecutive live words are forced distinct, so row 2 can attribute any
    output repeat to the DUT rather than to the stimulus.
    """
    rng = random.Random(seed)
    words = []
    prev = None
    for _ in range(n):
        if rng.random() < duty:
            while True:
                d = rng.randrange(1, 1 << 32)
                if d != prev:
                    break
            prev = d
            words.append((d, 0, 1))
        else:
            words.append((0, 0, 0))
    return words


# ---------------------------------------------------------------------------
@cocotb.test()
async def ba_valid_out_equals_valid_in(dut):
    """Property 1 -- the module must not invent or swallow beats.

    ON THE BROKEN RTL: data_valid_o latches high and the output count runs to
    the full window length. Expect roughly window-vs-duty, e.g. ~1000 vs ~660.
    """
    tb = TB(dut)
    await tb.reset()
    words = make_stream(0x7C01, 600, duty=0.66)
    col = Collector(dut)
    mon = cocotb.start_soon(col.run(len(words) + 4 * NUM_PIPELINES + 30))
    await drive(dut, words)
    await mon

    n_in, n_out = len(col.in_beats), len(col.out_beats)
    dut._log.info("BA-P1 in_beats=%d out_beats=%d", n_in, n_out)
    assert n_in > 100, f"non-vacuity: only {n_in} input beats driven"
    assert n_out == n_in, (
        f"block_alignment emitted {n_out} valid beats for {n_in} input beats "
        f"(delta {n_out - n_in}). data_valid_o must mark 'this clock carried a "
        f"Symbol', one-for-one with the input"
    )


@cocotb.test()
async def ba_no_duplicate_beats(dut):
    """Property 2 -- differential: output repeats must not exceed input repeats.

    ON THE BROKEN RTL: ~35-40% of output beats repeat their predecessor,
    because the held value is re-sampled on every idle cycle.
    """
    tb = TB(dut)
    await tb.reset()
    words = make_stream(0x7C02, 600, duty=0.66)
    col = Collector(dut)
    mon = cocotb.start_soon(col.run(len(words) + 4 * NUM_PIPELINES + 30))
    await drive(dut, words)
    await mon

    d_in, d_out = dup_runs(col.in_beats), dup_runs(col.out_beats)
    dut._log.info("BA-P2 input dups=%d of %d, output dups=%d of %d",
                  d_in, len(col.in_beats), d_out, len(col.out_beats))
    assert len(col.out_beats) > 100, "non-vacuity: too few output beats"
    assert d_in == 0, (
        f"the STIMULUS contains {d_in} repeats, so this row would be measuring "
        f"the stimulus rather than the DUT -- make_stream is meant to prevent it"
    )
    assert d_out == 0, (
        f"{d_out} of {len(col.out_beats)} output beats repeat their predecessor "
        f"while the input contains none. The held value is being re-sampled on "
        f"cycles that carried no Symbol"
    )


@cocotb.test()
async def ba_no_dropped_words(dut):
    """Property 3 -- every input word appears at the output, in order.

    Compared as SEQUENCES, not as sets: order is part of the contract, and a
    set comparison would pass a module that reordered.
    """
    tb = TB(dut)
    await tb.reset()
    words = make_stream(0x7C03, 600, duty=0.66)
    col = Collector(dut)
    mon = cocotb.start_soon(col.run(len(words) + 4 * NUM_PIPELINES + 30))
    await drive(dut, words)
    await mon

    dut._log.info("BA-P3 in=%d out=%d", len(col.in_beats), len(col.out_beats))
    assert len(col.in_beats) > 100, "non-vacuity"
    missing = [b for b in col.in_beats if b not in col.out_beats]
    assert not missing, (
        f"{len(missing)} input words never appeared at the output; first is "
        f"{missing[0][0]:#010x}"
    )
    assert col.out_beats == col.in_beats, (
        f"the output sequence is not the input sequence. in[0:4]="
        f"{[hex(b[0]) for b in col.in_beats[:4]]} out[0:4]="
        f"{[hex(b[0]) for b in col.out_beats[:4]]}"
    )


@cocotb.test()
async def ba_data_preserved_across_an_idle_gap(dut):
    """Property 4 -- an idle gap must not corrupt the words either side of it.

    Two live words with a deliberate gap between them, driven twice: once dense
    and once with the gap. The returned sequences must be IDENTICAL. Differential
    for the same reason row 2 is: a gap that 'does not matter' is a measurement,
    not an assumption.
    """
    tb = TB(dut)
    await tb.reset()
    payload = [(0xA1B2C3D4, 0, 1), (0x0F1E2D3C, 0, 1), (0x55AA55AA, 0, 1),
               (0xDEADBEEF, 0, 1)]

    async def run_once(gap):
        t = TB(dut)
        await t.reset()
        seq = []
        for w in payload:
            seq.append(w)
            for _ in range(gap):
                seq.append((0, 0, 0))
        c = Collector(dut)
        m = cocotb.start_soon(c.run(len(seq) + 4 * NUM_PIPELINES + 30))
        await drive(dut, seq)
        await m
        return c.out_beats

    dense = await run_once(0)
    sparse = await run_once(3)
    dut._log.info("BA-P4 dense=%s sparse=%s",
                  [hex(b[0]) for b in dense], [hex(b[0]) for b in sparse])
    expect = [(w[0], w[1]) for w in payload]
    assert dense == expect, (
        f"dense run altered the data: got {[hex(b[0]) for b in dense]}")
    assert sparse == expect, (
        f"an idle gap corrupted the stream: got {[hex(b[0]) for b in sparse]}, "
        f"expected {[hex(x[0]) for x in expect]}")


@cocotb.test()
async def ba_valid_falls_after_pipeline_latency(dut):
    """Property 5 -- data_valid_o returns LOW once the pipeline has drained.

    ON THE BROKEN RTL: it never falls. This is the property that fails most
    directly and it is the one the fix is really about -- 'no Symbol here' has
    to be a state the output can express.

    The window is bounded by a SIGNAL-derived condition (input idle since beat
    N) rather than a bare count, and the drain allowance is NumPipelines plus a
    small margin, stated rather than tuned.
    """
    tb = TB(dut)
    await tb.reset()
    words = [(0x11111111 + i, 0, 1) for i in range(8)]
    col = Collector(dut)
    TAIL = 40
    mon = cocotb.start_soon(col.run(len(words) + TAIL))
    await drive(dut, words)
    await mon

    # The input went idle at index len(words); allow the pipeline to drain.
    drain_from = len(words) + NUM_PIPELINES + 4
    tail = col.out_valid_trace[drain_from:]
    high = sum(tail)
    dut._log.info(
        "BA-P5 out_valid high on %d of %d cycles after drain; last 12=%s",
        high, len(tail), col.out_valid_trace[-12:])
    assert len(tail) > 10, "non-vacuity: drain window too short to mean anything"
    assert high == 0, (
        f"data_valid_o was high on {high} of {len(tail)} cycles AFTER the input "
        f"went idle and the pipeline had {NUM_PIPELINES} cycles to drain. It "
        f"latches high: 'no Symbol here' is not expressible, which is the "
        f"defect (F16 root cause, block_alignment.sv:90)"
    )
