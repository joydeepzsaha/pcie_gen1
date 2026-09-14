"""block_alignment -- the module's FIRST bench. SS63 #7c Phases 1 and 2'.

!! THIS BENCH WAS WRITTEN AGAINST THE BROKEN RTL AND WAS RED BEFORE THE FIX.
That order is the point: rows that only ever existed after a fix cannot tell you
the fix did anything.

!! EVERY "HISTORY, measured on the broken RTL" NOTE BELOW IS A PRE-FIX NUMBER.
They are kept as history, not as a description of the module you are reading
now -- SS22.87, a red row's premises expire the moment it flips, and here those
premises live in docstrings rather than in an expect_fail marker, where nothing
greppable would have found them.

    broken RTL (c62cc98)          1 / 8   -- P0 only
    shape (b), TAKEN              8 / 8
    shape (c), rejected           4 / 8   -- stranded NumPipelines beats/burst
    shape (a), rejected earlier   2 / 5   -- on the 5-row Phase-1 bench

Verilator LATCH warnings on this module alone: 3 before the fix, 0 after.

!! WHY THE MODULE HAD NO BENCH UNTIL NOW, which is itself a finding. Census at
SS63 #7b: no gate target drives block_alignment as toplevel. The module carries
148 of the design's 149 latch primitives and a live functional defect, and it
has never been exercised on its own -- only transitively, through phy_receive,
by rows asserting things about other modules.

== THE DEFECT THESE ROWS PINNED -- FIXED, KEPT AS THE RECORD ==============

!! PAST TENSE THROUGHOUT THIS SECTION. It describes the RTL as it was at
c62cc98, which is what the rows were built against.

block_alignment.sv's `always_comb` had NO `D = Q;` default and wrote
D.data_valid ONLY inside `if (phy_link_up_i & |data_valid_i)`. So the struct
LATCHED, and data_valid_o -- once high -- never fell. Measured in the
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

== !! WHAT THIS MODULE ACTUALLY DOES, MEASURED AT PHASE 2' ================

!! IT DOES NOT ALIGN ANYTHING. Every alignment transform in the file -- the
lane-reversal remap, the pipewidth byte gather, the lane/byte index arithmetic
-- is COMMENTED OUT (block_alignment.sv:113-177). Extracting only live
statements from the always_comb leaves a 4-deep shift of {data, data_k,
data_valid, sync_header} and nothing else. `pipewidth_bytes`,
`pipewidth_shift_idx` and `lanes_shift_idx` are computed at :91-93 and READ BY
NO LIVE STATEMENT; `lane_number`, `byte_number` and `lane_idx` are declared and
never assigned outside comments. `lane_reverse_i` is tied to '0 at the sole
instantiation (phy_receive.sv:192).

So the module's contract, as built, is TRANSPORT: deliver each input beat once,
in order, NumPipelines clocks later, and say so on data_valid_o. That is what
P0..P7 measure. A golden "aligned output" would be the identity, so a row
comparing against one would be measuring nothing.

== !! A SECOND, LATENT DEFECT IN THE SAME FILE -- NOT FIXED HERE ==========

block_alignment.sv:108 reads `D.sync_header[pipeline_idx-1]`, where every
sibling line reads `Q.*[pipeline_idx-1]`. `D` makes it a combinational fan-out
chain rather than a pipeline stage, so all four stages take stage 0's value in
the SAME clock and sync_header_o is delayed by ONE cycle while data_o is
delayed by four. It is LATENT, not live: both integrated tops tie the port to
'0 (pcie_endpoint_top.sv:416), so the chain propagates a constant. Registered,
deliberately NOT repaired in this rung -- SS63 #7c's src/ radius is the valid
pipeline. No row below depends on sync_header.

== THE ACCEPTANCE PROPERTIES =============================================

  P0. APPARATUS CONTROL -- was the one GREEN row before the fix
  P1. valid out == valid in        -- beat counts equal over the window
  P2. no duplicates                -- no output beat repeats its predecessor
                                      unless the input did
  P3. no drops                     -- every input word appears at the output
  P4. data preserved through idle  -- an idle gap does not corrupt the word
                                      either side of it
  P5. valid falls after latency    -- data_valid_o returns low once the
                                      pipeline has drained, and stays low
  P6. realistic burst gaps         -- transport holds under the full-stack
                                      gap SHAPE, not just a uniform duty
  P7. reset is durable             -- after rst_i releases with the input idle,
                                      the pipeline stays empty instead of
                                      reloading its pre-reset contents

!! P0 IS A NEGATIVE CONTROL FOR THE BENCH, NOT FOR THE DUT, and before the fix
it was the only row here that had to be GREEN. With the input NEVER idle the
guard was true on every clock, so even the broken RTL was a correct 4-deep
delay line and the defect was unobservable. If P0 ever fails, the apparatus is
lying and no score from P1..P7 means anything -- including the shape scores
that D-7C.1 was settled on.

!! THE APPARATUS WAS REBUILT AT PHASE 2', AND THE FIRST DIAGNOSIS OF WHY WAS
WRONG. Two things were found and they are NOT the same kind of thing. Keeping
them apart matters, because one of them turned out to be a DUT defect that had
been about to be "fixed" out of the bench.

  APPARATUS, real, fixed. `ba_data_preserved_across_an_idle_gap` constructed
  TB three times in one test -- once in the body, once per run_once -- so three
  Clock coroutines drove one clk_i, against this file's own "ONE TB PER TEST"
  note. And the collector sampled immediately after RisingEdge, in the same
  phase the driver writes. Both are now fixed: one TB per test, and sampling
  moved into ReadOnly.

  NOT APPARATUS -- THE DUT. P4's dense capture opened on
    ['0xb20eba0d', '0xb20eba0d', '0x73e314cb', '0x73e314cb', '0xa1b2c3d4', ...]
  and the first four of those are P3's LAST TWO input words, each emitted
  twice. The first reading was that a racing clock had leaked them. It had not:
  removing the extra clocks changed the capture NOT AT ALL. Running P4 alone
  under TESTCASE= does clear it, but only because an isolated run has no prior
  traffic to leak -- absence of a symptom for want of a cause, read as a fix.

  What it actually is, measured across the reset boundary: rst_i DOES clear Q
  while it is held (dv_o=0, data_o=0 on all five reset cycles), and one clock
  after release Q is back to 0xb20eba0d with data_valid_o high. THE LATCH
  DEFEATS THE RESET. D is not driven on idle cycles, so it still holds the
  pre-reset contents; Q <= D on the first clock after release reloads exactly
  what the reset just cleared. This module cannot be reset while its input is
  idle, which is the only state a reset is ever asserted in.

  That is a THIRD symptom of the missing `D = Q;`, alongside the 148 latches
  and the stuck valid, and it is the one nothing had a row for. It is now P7.

!! THE LESSON, recorded because it nearly cost the shape scores. P4 is one of
the two rows that DISCRIMINATE shape (b) from shape (c). Had the leak been
"fixed" by resetting harder or by starting the collector later -- both of which
were on the table while the apparatus story held -- the bench would have
silently stopped observing a real defect, and would have gone on to arbitrate
D-7C.1 with that blindness built in. The tell was that removing the supposed
cause did not move the measurement.

!! ROWS P2 AND P3 ARE DIFFERENTIAL AGAINST THE INPUT, not absolute. A stream
that legitimately repeats a word must not be scored as a duplicate -- otherwise
the row measures the stimulus rather than the DUT.
"""

import os
import random

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ClockCycles, ReadOnly

CLK_NS = 8
LANES = 1
PIPE_WIDTH_GEN1 = 16          # bits per lane per beat; lane_management.sv:45
NUM_PIPELINES = 4             # block_alignment.sv:35 -- the pipeline depth
GEN1 = 1                      # rate_speed_e'(gen1)

# Duty of the captured slice replayed by P6 -- see fullstack_gap_trace.txt.
# !! BRIEF_7C cites 19,760/30,000 = 0.6587 for this same signal. That window is
# not anchored at link-up; the first 30,000 post-L0 cycles measure 0.6917 and
# the whole 57,711-cycle post-L0 trace 0.7434. Same signal, different anchor.
TRACE_DUTY = 15955 / 24000            # 0.6648


class TB:
    """!! ONE PER TEST. cocotb cancels every task a test started when that test
    ends, including the Clock coroutine -- so the clock must be started once,
    inside the test that uses it, and never re-started while that test runs.
    """

    def __init__(self, dut):
        self.dut = dut
        self._clk = cocotb.start_soon(Clock(dut.clk_i, CLK_NS, units="ns").start())

    async def reset(self):
        """Re-callable. Does NOT touch the clock -- that is the Phase-1 bug."""
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

    !! SAMPLING IS IN ReadOnly, not immediately after RisingEdge. ReadOnly is
    entered once every write scheduled for this timestep has settled, so the
    collector cannot race the driver. The Phase-1 collector read in the same
    phase the driver wrote, which is what let P4's capture open on the previous
    test's residue.
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
            await ReadOnly()
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
        # !! LEAVE THE SCHEDULER IN A WRITABLE PHASE. The loop above ends inside
        # ReadOnly, and cocotb raises "attempt to write during ReadOnly" for the
        # first signal write after this coroutine is awaited -- which is what a
        # caller that resets and drives again does immediately. Costs one cycle
        # and makes `await collector` safe to follow with a write.
        await RisingEdge(d.clk_i)


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

    Consecutive live words are forced distinct, so P2 can attribute any output
    repeat to the DUT rather than to the stimulus.
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


def load_fullstack_gap_trace():
    """The REAL valid cadence, replayed from a capture -- not a model of it.

    fullstack_gap_trace.txt is a contiguous post-L0 slice of data_valid_i taken
    at this module's own input inside verilate_fullstack. See that file's header
    for provenance. Returns a list of 0/1, one per cycle.
    """
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        "fullstack_gap_trace.txt")
    with open(path) as f:
        rle = [ln for ln in f if not ln.startswith("#")][-1]
    trace = []
    for pair in rle.split():
        v, n = pair.split(":")
        trace.extend([int(v)] * int(n))
    return trace


def words_from_trace(seed, trace):
    """Attach distinct payload words to a captured valid cadence.

    The CADENCE is the captured data; the payload is synthetic because the
    captured stream's own words are scrambled Symbols and would carry
    legitimate repeats, which P2's differential check is built to exclude.
    Consecutive live words are forced distinct, as in make_stream.
    """
    rng = random.Random(seed)
    words = []
    prev = None
    for v in trace:
        if v:
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
async def ba_apparatus_control_no_idle_is_a_clean_delay_line(dut):
    """P0 -- APPARATUS CONTROL. Must be GREEN on the BROKEN RTL.

    With data_valid_i high on EVERY clock the guard at block_alignment.sv:96 is
    always true, the struct never latches, and the broken module is a correct
    4-deep delay line. So this row says nothing about the defect -- it says the
    BENCH can read a correct stream correctly.

    !! IF THIS ROW IS RED, EVERY OTHER SCORE IN THIS FILE IS VOID, including
    the shape scores D-7C.1 settled on.

    !! AND ITS DISCRIMINATING POWER IS NOW SPENT -- said plainly rather than
    left for a reader to discover. Against the BROKEN RTL this row was the one
    green in a field of red, which is what made it a control. Against the fixed
    RTL it asserts a strict subset of what P1 and P3 assert, so it can no longer
    fail alone. It is kept because it is the row that would catch a FUTURE
    apparatus regression -- a collector sampling in the wrong phase, a driver
    racing it -- before that regression could be read as a DUT result.
    """
    tb = TB(dut)
    await tb.reset()
    words = make_stream(0x7C00, 400, duty=1.0)
    col = Collector(dut)
    mon = cocotb.start_soon(col.run(len(words) + 2))
    await drive(dut, words)
    await mon

    n_in, n_out = len(col.in_beats), len(col.out_beats)
    dut._log.info("BA-P0 in_beats=%d out_beats=%d (control: must be GREEN "
                  "before the fix)", n_in, n_out)
    assert n_in > 100, f"non-vacuity: only {n_in} input beats driven"
    assert n_out > 100, f"non-vacuity: only {n_out} output beats seen"
    assert col.out_beats == col.in_beats[:n_out], (
        f"APPARATUS FAILURE, not a DUT failure. With no idle cycles at all the "
        f"broken RTL is a correct delay line, so the output must be a prefix-"
        f"delayed copy of the input. in[0:4]="
        f"{[hex(b[0]) for b in col.in_beats[:4]]} out[0:4]="
        f"{[hex(b[0]) for b in col.out_beats[:4]]}. Every other row in this "
        f"file is void until this one is green"
    )


@cocotb.test()
async def ba_valid_out_equals_valid_in(dut):
    """P1 -- the module must not invent or swallow beats.

    HISTORY, measured on the broken RTL: data_valid_o latched high and the
    output count ran to the full window length -- 406 in, 646 out. Fixed: 406/406.
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
    """P2 -- differential: output repeats must not exceed input repeats.

    HISTORY, measured on the broken RTL: 259 of 646 output beats repeated their
    predecessor -- 40% -- because the held value was re-sampled on every idle
    cycle. Fixed: 0 of 406, against 0 in the stimulus.
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
    """P3 -- every input word appears at the output, in order.

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
    """P4 -- an idle gap must not corrupt the words either side of it.

    Two live words with a deliberate gap between them, driven twice: once dense
    and once with the gap. The returned sequences must be IDENTICAL. Differential
    for the same reason P2 is: a gap that 'does not matter' is a measurement,
    not an assumption.

    !! ONE TB, THREE RESETS. The Phase-1 version constructed TB inside
    run_once, so this test ran three concurrent Clock coroutines on one clk_i
    and its dense capture opened on the PREVIOUS test's data. The clock is now
    started once, here, and run_once only resets and drives.
    """
    tb = TB(dut)
    payload = [(0xA1B2C3D4, 0, 1), (0x0F1E2D3C, 0, 1), (0x55AA55AA, 0, 1),
               (0xDEADBEEF, 0, 1)]

    async def run_once(gap):
        await tb.reset()
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
    """P5 -- data_valid_o returns LOW once the pipeline has drained.

    HISTORY, measured on the broken RTL: it never fell -- high on 32 of 32
    cycles after the drain. This is the property that failed most directly and
    it is the one the fix is really about -- 'no Symbol here' has to be a state
    the output can express.

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


@cocotb.test()
async def ba_transport_holds_under_realistic_burst_gaps(dut):
    """P6 -- transport under the full-stack gap SHAPE, not just its duty.

    !! WHY THIS ROW EXISTS AND WHY IT IS NOT AN 'ALIGNMENT' ROW. D-7C.1 asked
    for a sixth property checking "the module's actual job" -- alignment
    correctness on gapped input, against a golden. There is no alignment: every
    transform in block_alignment.sv is commented out and the live module is a
    4-deep delay line (see this file's header). A golden aligned output would
    be the identity, so such a row would measure nothing. The module's job, as
    built, is TRANSPORT, and the thing P1..P5 do not cover is the gap SHAPE.

    !! THE CADENCE HERE IS CAPTURED, NOT MODELLED, and capturing it is what
    made the row worth writing. The first draft of P6 synthesised bursts from a
    plausible run-length menu -- live runs of 2..14, idle runs of 1..8. The
    capture refuted it. The real post-L0 cadence at this module's input is a
    near-deterministic FOUR-VALID / ONE-IDLE repeat (3,919 live runs of exactly
    4 and 3,948 idle runs of exactly 1 in this window), punctuated during the
    early FC-init regime by a few very long idle gaps -- 677 cycles, five
    times, plus 537 and 175. Neither feature is anything a menu would have
    produced, and BOTH matter: the 4:1 cadence is what the module actually
    sees for the whole of steady state, and the 677-cycle gaps are where a
    pipeline that advances on idle and one that freezes on idle diverge by
    hundreds of beats rather than by one.

    P1..P3 drive a Bernoulli stream whose idle runs are almost all length 1;
    P4 drives one gap width. Neither reaches the long-gap case, and a uniform
    -duty stimulus can score two shapes identically while being blind to the
    thing that separates them.
    """
    tb = TB(dut)
    await tb.reset()
    trace = load_fullstack_gap_trace()
    words = words_from_trace(0x7C06, trace)
    col = Collector(dut)
    mon = cocotb.start_soon(col.run(len(words) + 4 * NUM_PIPELINES + 30))
    await drive(dut, words)
    await mon

    duty = sum(1 for w in words if w[2]) / len(words)
    n_in, n_out = len(col.in_beats), len(col.out_beats)
    dut._log.info("BA-P6 captured cadence: cycles=%d duty=%.4f in=%d out=%d",
                  len(words), duty, n_in, n_out)

    # Stimulus checks first -- a row that measures its own stimulus is worse
    # than no row, so the shape is asserted before the DUT is scored.
    assert len(trace) > 20000, (
        f"the captured trace is only {len(trace)} cycles; the long-gap regime "
        f"and the steady-state cadence must both be inside the window")
    assert abs(duty - TRACE_DUTY) < 0.01, (
        f"replayed duty {duty:.4f} does not match the captured trace's "
        f"{TRACE_DUTY:.4f} -- the replay is not reproducing the capture")
    assert dup_runs(col.in_beats) == 0, "stimulus contains repeats"
    assert n_in > 300, f"non-vacuity: only {n_in} input beats driven"

    assert n_out == n_in, (
        f"under realistic burst gaps block_alignment emitted {n_out} beats for "
        f"{n_in} input beats (delta {n_out - n_in})")
    assert col.out_beats == col.in_beats, (
        f"under realistic burst gaps the output sequence is not the input "
        f"sequence. in[0:4]={[hex(b[0]) for b in col.in_beats[:4]]} "
        f"out[0:4]={[hex(b[0]) for b in col.out_beats[:4]]}")


@cocotb.test()
async def ba_reset_is_durable_with_an_idle_input(dut):
    """P7 -- rst_i must actually reset. Found at Phase 2', previously unmeasured.

    !! THE LATCH DEFEATS THE RESET. Measured on the broken RTL, driving traffic,
    then asserting rst_i for five cycles with the input idle:

        in reset  cyc0..4 : dv_o=0  data_o=0x00000000     <- cleared, correctly
        post reset cyc0   : dv_o=1  data_o=0xb20eba0d     <- and back again

    block_alignment.sv:80's always_ff clears Q under rst_i, but D is written
    only inside the always_comb's guard, so on an idle cycle D is a latch still
    holding the PRE-RESET contents. The first clock after release does Q <= D
    and reloads exactly what the reset cleared.

    !! AND THE INPUT IS ALWAYS IDLE WHEN RESET IS ASSERTED -- that is what reset
    is for -- so this is not a corner case, it is the only case. In the stack it
    means the receiver comes out of reset already holding stale Symbols with
    data_valid_o high, before any real data has arrived.

    A third symptom of the one missing `D = Q;`, alongside the 148 latches and
    the stuck valid. Both shapes under D-7C.1 carry `D = Q;` and both should
    turn this row green; it is here because nothing else in the file would
    notice if they did not.
    """
    tb = TB(dut)
    await tb.reset()

    # Put real traffic through, so there is something to fail to clear.
    #
    # !! THE PRECONDITION IS SIGNAL-BOUND, NOT COUNT-BOUND (SS22.89), and the
    # first version of this row got that wrong in a way that made it an UNFAIR
    # oracle. It waited a fixed NUM_PIPELINES+2 cycles and then assumed the
    # pipeline was still loaded. That holds for a pipeline that FREEZES on idle
    # -- the broken RTL, and shape (c) -- and is false for one that ADVANCES on
    # idle -- shape (b) -- which drains itself during the wait. Shape (b) then
    # failed this row on `assert 0 != 0`, its own non-vacuity guard, and the
    # bare score read as a defect in the shape. Poll for the loaded state
    # instead, and reset from whenever it actually arrives.
    words = [(0xC0DE0000 + i, 0, 1) for i in range(12)]
    await drive(dut, words)

    dirty = 0
    for _ in range(2 * NUM_PIPELINES + 4):
        await RisingEdge(dut.clk_i)
        await ReadOnly()
        dirty = int(dut.data_o.value) & 0xFFFFFFFF
        if dirty:
            break
    await RisingEdge(dut.clk_i)
    assert dirty != 0, (
        f"non-vacuity: the pipeline never presented a non-zero word after the "
        f"drive, so there would be nothing for the reset to fail to clear")

    # Reset with the input idle -- the only state a reset is ever asserted in.
    dut.rst_i.value = 1
    dut.data_valid_i.value = 0
    dut.data_i.value = 0
    await ClockCycles(dut.clk_i, 5)
    await ReadOnly()
    in_reset_valid = int(dut.data_valid_o.value) & 1
    in_reset_data = int(dut.data_o.value) & 0xFFFFFFFF
    await RisingEdge(dut.clk_i)
    dut.rst_i.value = 0

    # Sample the release window continuously -- a bare read after RisingEdge
    # would be a single phased observation of a multi-cycle claim (SS22.89).
    after_valid, after_data = [], []
    for _ in range(8):
        await RisingEdge(dut.clk_i)
        await ReadOnly()
        after_valid.append(int(dut.data_valid_o.value) & 1)
        after_data.append(int(dut.data_o.value) & 0xFFFFFFFF)

    dut._log.info(
        "BA-P7 pre-reset data_o=%#010x | in reset dv_o=%d data_o=%#010x | "
        "after release dv_o=%s data_o=%s",
        dirty, in_reset_valid, in_reset_data, after_valid,
        [hex(d) for d in after_data])

    assert in_reset_valid == 0 and in_reset_data == 0, (
        f"the reset does not even hold: dv_o={in_reset_valid} "
        f"data_o={in_reset_data:#010x} WHILE rst_i is asserted")
    assert sum(after_valid) == 0, (
        f"data_valid_o went high on {sum(after_valid)} of 8 cycles after rst_i "
        f"released, with the input still idle. The pipeline reloaded its "
        f"pre-reset contents: D is a latch on idle cycles, so Q <= D restores "
        f"what the reset cleared (block_alignment.sv:90, missing `D = Q;`)")
    assert not any(after_data), (
        f"data_o returned to {[hex(d) for d in after_data]} after the reset "
        f"released; pre-reset value was {dirty:#010x}")
