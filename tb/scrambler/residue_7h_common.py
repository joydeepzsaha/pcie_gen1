"""§63 #7h Phase 1 — shared machinery for the residue measurement.

MEASUREMENT ONLY.  Nothing here asserts a spec property; every row that uses it
reports numbers.  The fix shape is chat's to choose at the Phase 2 STOP
(D-7H.2), so this file must not encode a preferred answer.

WHAT #21 IS
-----------
#7f measured: a TLP enters the TX scrambler stage as 10 or 12 contiguous valid
cycles, 7 words come out, and the rest is released one cycle before the next COM
on a free-running 679-cycle grid.  This file measures the same thing at the unit
seam, where the schedule is the bench's own and nothing is inferred.

§22.92 SHAPE
------------
The DUT is driven and sampled raw: one record per clock, no classification in
SystemVerilog and none inside the sampling loop.  Every derived quantity is a
pure function of the trace, computed below, and every row opens with the
known-answer self-test in `known_answer()`.

THE KNOWN ANSWER, AND WHY IT USES K CODES
-----------------------------------------
The payload is scrambled, so a presented data word cannot be recognised at the
output by value.  Base 2.1 §4.2.3 p.199 — "All special Symbols (K codes) are not
scrambled" — gives a marker that survives the transform unchanged, and
`data_k_out_o` rides the same 4-deep gated struct pipeline as the data
(gen1_scramble.sv: `D.data_k[i] = Q.data_k[i-1]`, inside `if (data_valid_i)`).

So each beat is tagged with a distinct `data_k` value and located at the output
by that tag.  The known answer is hand-derivable and independent of the
scrambler's arithmetic: with valid held high continuously, a beat presented on
clock t appears at `data_out_o` on clock t+LATENCY, where LATENCY = DEPTH-1 = 3
shift hops (derived below).  `known_answer()` checks exactly that, and a row
whose known-answer check fails reports nothing else.

⚠️ A bare read after RisingEdge is PRE-edge (§22.89); Timer(1 ps) lands
post-edge.  Inherited from align_common.run() and kept identical here.
"""
from cocotb.triggers import ClockCycles, RisingEdge, Timer

# lane_management.sv:45 -- PipeWidthGen1 = 16, the integrated-path value.
# At pipe_width 16 the datapath carries 2 Symbols per clock, so "beat" below
# means one clock, not one Symbol.
PIPE_WIDTH = 16

# gen1_scramble.sv:18 -- NumPipelines, the number of pipeline STAGES.
DEPTH = 4

# The number of clocks from "the bench presented a Symbol" to "that Symbol is at
# data_out_o", under continuous valid.  It is DEPTH-1, not DEPTH, and the two
# are different questions:
#
#   stage 0 is loaded on the SAME edge the Symbol is presented
#           (D.data[0] = data_in_i), so the post-edge sample of clock t already
#           has Q.data[0] == w.
#   stages 1, 2, 3 are three further shift hops (D.data[i] = Q.data[i-1]), so
#           three more edges put w in Q.data[3], which is what data_out_o reads.
#
#   => 3 hops = DEPTH-1 clocks of latency, while PRIMING still takes DEPTH
#      presentations before the output carries a real word (presentation
#      n_pres-3 exists only once n_pres >= 4).  align_common.advance_values()
#      uses `n_pres >= DEPTH` for the priming question and is consistent with
#      this; the two constants are simply not the same constant.
#
# ⚠️ INSTRUMENT FAULT, CAUGHT BY THE SELF-TEST, RECORDED RATHER THAN QUIETLY
# FIXED (§22.92).  The first version of this file asserted latency == DEPTH and
# all five rows failed their known-answer check before reporting a single
# number.  That is the rule working as intended: the wrong constant surfaced as
# a loud failure instead of a silent 1-clock offset in every table of the Phase
# 2 report.  The value below is re-derived from the RTL above, NOT read back
# from the DUT (§22.49 -- a DUT-mirror would make this check vacuous).
LATENCY = DEPTH - 1

# Tags.  Any non-zero data_k value works; these are distinct per beat position
# so a tag identifies WHICH beat, not merely that a beat was marked.
TAG_FIRST = 0b0001      # stands in for STP / SDP
TAG_LAST = 0b0010       # stands in for END
TAG_NONE = 0b0000


def payload(n, salt=0):
    """n distinct, non-repeating D-Symbol words.

    Deterministic (no RNG) so two runs are identical by construction, and
    non-constant so an output that merely holds its value cannot be mistaken
    for a correct one (§22.53).  `salt` separates one packet from the next in
    the two-packet case.
    """
    return [((0x11 * (i + 1)) ^ (0xA5C3 << 8) ^ (i * 0x01010101) ^ (salt * 0x9E3779B9))
            & 0xFFFFFFFF for i in range(n)]


def packet(n, salt=0):
    """A packet-shaped beat list: [(data, k)], first beat tagged, last beat tagged.

    n is the number of CLOCKS, matching #7f's "10 or 12 contiguous valid cycles".
    """
    words = payload(n, salt)
    out = []
    for i, w in enumerate(words):
        if i == 0:
            k = TAG_FIRST
        elif i == n - 1:
            k = TAG_LAST
        else:
            k = TAG_NONE
        out.append((w, k))
    return out


async def reset(dut):
    dut.rst_i.value = 1
    dut.data_valid_i.value = 0
    dut.data_in_i.value = 0
    dut.data_k_in_i.value = 0
    dut.pipe_width_i.value = PIPE_WIDTH
    # Present only on the `scrambler` wrapper; `gen1_scramble` has no such ports.
    for name, val in (("lane_number", 0), ("sync_header_i", 0),
                      ("block_start_i", 0), ("curr_data_rate_i", 0)):
        if hasattr(dut, name):
            getattr(dut, name).value = val
    await ClockCycles(dut.clk_i, 5)
    dut.rst_i.value = 0
    await ClockCycles(dut.clk_i, 2)


async def drive(dut, schedule):
    """Drive a schedule of (data, k, valid) triples, one per clock.

    Returns one raw record per clock.  No classification happens here: the
    record is what the pins were, plus the bench's own drive record, and every
    question asked of it later is answered in Python (§22.92).

        clk       index of this clock, 0-based, after reset
        presented did the bench assert data_valid_i on THIS clock
        n_pres    presentations on clocks 0..this one inclusive
        in_k      the tag the bench presented on this clock (0 if none)
        data      data_out_o sampled post-edge
        k         data_k_out_o sampled post-edge
        valid     data_valid_o sampled post-edge
    """
    await reset(dut)
    trace = []
    n_pres = 0
    for clk, (d, k, v) in enumerate(schedule):
        dut.data_in_i.value = d
        dut.data_k_in_i.value = k
        dut.data_valid_i.value = 1 if v else 0
        await RisingEdge(dut.clk_i)
        await Timer(1, units="ps")
        if v:
            n_pres += 1
        trace.append({
            "clk": clk,
            "presented": bool(v),
            "n_pres": n_pres,
            "in_k": k if v else 0,
            "data": int(dut.data_out_o.value),
            "k": int(dut.data_k_out_o.value),
            "valid": bool(int(dut.data_valid_o.value) & 1),
        })
    return trace


# ---------------------------------------------------------------------------
#  Schedule builders.  Each returns (schedule, marks) where marks records the
#  clock index of every tagged presentation, so the analysis never has to
#  re-derive where the bench put something.
# ---------------------------------------------------------------------------

def sched_lone(n, lead_idle, trail_idle, salt=0):
    """1-a / 1-c: one packet of n clocks, idle either side."""
    sched, marks = [], {}
    for _ in range(lead_idle):
        sched.append((0, 0, False))
    for i, (d, k) in enumerate(packet(n, salt)):
        if k:
            marks.setdefault(k, len(sched))
        sched.append((d, k, True))
    for _ in range(trail_idle):
        sched.append((0, 0, False))
    return sched, marks


def sched_two_back_to_back(n1, n2, lead_idle, trail_idle):
    """1-b: two packets with NO idle between them -- H7H-2's discriminator.

    The second packet's tags are offset by 4 so the two packets' first/last
    beats are distinguishable in one trace.
    """
    sched, marks = [], {}
    for _ in range(lead_idle):
        sched.append((0, 0, False))
    for i, (d, k) in enumerate(packet(n1, salt=0)):
        if k:
            marks.setdefault(k, len(sched))
        sched.append((d, k, True))
    for i, (d, k) in enumerate(packet(n2, salt=1)):
        k2 = (k << 2) if k else 0          # 0b0100 / 0b1000
        if k2:
            marks.setdefault(k2, len(sched))
        sched.append((d, k2, True))
    for _ in range(trail_idle):
        sched.append((0, 0, False))
    return sched, marks


def sched_packet_then_os(n, lead_idle, gap, os_beats, trail_idle):
    """A packet, then a quiet gap, then a burst of valid beats standing in for
    the next Ordered Set.

    This is the unit-seam analogue of #7f's "released one cycle before the next
    COM": the OS is modelled only as *a source of valid beats*, with no COM
    semantics, because whether the ordered set matters AS an ordered set or
    merely as valid traffic is exactly what 1-b and this schedule separate.
    """
    sched, marks = [], {}
    for _ in range(lead_idle):
        sched.append((0, 0, False))
    for i, (d, k) in enumerate(packet(n, salt=0)):
        if k:
            marks.setdefault(k, len(sched))
        sched.append((d, k, True))
    for _ in range(gap):
        sched.append((0, 0, False))
    marks["os"] = len(sched)
    for i in range(os_beats):
        sched.append((0xC5C5C5C5, 0b1000, True))     # tagged, distinct from packet tags
    for _ in range(trail_idle):
        sched.append((0, 0, False))
    return sched, marks


# ---------------------------------------------------------------------------
#  Derived quantities -- pure functions of the trace.
# ---------------------------------------------------------------------------

def published(trace):
    """The beats the DUT announced: valid high."""
    return [r for r in trace if r["valid"]]


def tag_out_clk(trace, tag):
    """The clock on which a tag appears at data_k_out_o, or None.

    Searched over ALL clocks, not only published ones, so that a tag which
    reaches the output pin while valid is low is still found -- that distinction
    is a measurement, not an assumption.
    """
    for r in trace:
        if r["k"] == tag:
            return r["clk"]
    return None


def tag_out_clk_published(trace, tag):
    """The clock on which a tag appears at data_k_out_o WITH valid high."""
    for r in trace:
        if r["k"] == tag and r["valid"]:
            return r["clk"]
    return None


def real_published(trace):
    """Published beats whose CONTENT is a Symbol the bench actually presented.

    The output at clock t carries whatever was presented at clock t-LATENCY
    (under the pipeline's own advance rule), so a published beat at t is real
    iff clock t-LATENCY was a presentation.  Computed entirely from the bench's
    OWN drive record -- never by comparing a data value against the DUT, which
    is impossible here anyway because the payload is scrambled (§22.49: a
    control must not be computed from the signal under test).
    """
    pres = {r["clk"] for r in trace if r["presented"]}
    return [r for r in trace if r["valid"] and (r["clk"] - LATENCY) in pres]


def stale_published(trace):
    """Published beats whose content was NOT presented -- the leading transient.

    These are the beats `test_scrambler_align.py`'s clause (d) row calls
    `fabricated`: valid is a 1-stage copy while the data needs LATENCY hops, so
    the first LATENCY-1 beats of any burst republish whatever the frozen
    pipeline was holding.
    """
    pres = {r["clk"] for r in trace if r["presented"]}
    return [r for r in trace if r["valid"] and (r["clk"] - LATENCY) not in pres]


def stranded(trace):
    """Presented beats whose content NEVER appeared on a published beat.

    A beat presented at clock t reaches data_out_o at t+LATENCY; it is seen by a
    consumer only if valid was high then.  This is the residue #21 is about,
    counted rather than inferred.
    """
    pub = {r["clk"] for r in trace if r["valid"]}
    return [r for r in trace if r["presented"] and (r["clk"] + LATENCY) not in pub]


def residue_at(trace, last_presented_clk):
    """How many presented beats had NOT yet reached data_out_o when the packet's
    last beat was presented.

    The pipeline is valid-gated, so after k presentations the output carries
    presentation k-DEPTH.  At the clock the last beat is presented, n_pres
    presentations have happened and n_pres-DEPTH+1 of them have emerged, so the
    residue is the difference.  Reported, never assumed: every row prints the
    measured tag clocks beside it.
    """
    rec = next(r for r in trace if r["clk"] == last_presented_clk)
    emerged = max(0, rec["n_pres"] - DEPTH + 1)
    return max(0, rec["n_pres"] - emerged)


def known_answer(dut, label):
    """The self-test every row opens with (§22.92).

    Hand-derived claim, independent of the scrambler's arithmetic: drive a
    continuous run of tagged beats with valid never dropping, and the tag
    presented on clock t must appear at data_k_out_o on clock t+DEPTH.

    Returns an async callable so the row can await it with its own clock
    already started.
    """
    async def _run():
        n = 12
        sched, marks = sched_lone(n, lead_idle=0, trail_idle=0)
        # Append DEPTH extra valid beats so the last tag is guaranteed to have
        # somewhere to emerge -- the known answer is about latency, not about
        # whether the tail is held.
        sched = sched + [(0, 0, True)] * DEPTH
        trace = await drive(dut, sched)
        first_in = marks[TAG_FIRST]
        last_in = marks[TAG_LAST]
        first_out = tag_out_clk(trace, TAG_FIRST)
        last_out = tag_out_clk(trace, TAG_LAST)
        dut._log.info(
            f"KA[{label}]: DEPTH={DEPTH} LATENCY={LATENCY} "
            f"first tag in@{first_in} out@{first_out} "
            f"last tag in@{last_in} out@{last_out}"
        )
        assert first_out is not None and last_out is not None, (
            f"known-answer FAILED: a tag never reached data_k_out_o "
            f"(first={first_out}, last={last_out}) -- the tag mechanism does not "
            f"work on this toplevel and no number from this row means anything"
        )
        assert first_out - first_in == LATENCY, (
            f"known-answer FAILED: first tag latency {first_out - first_in}, "
            f"expected LATENCY={LATENCY} (= DEPTH-1 shift hops) under continuous valid"
        )
        assert last_out - last_in == LATENCY, (
            f"known-answer FAILED: last tag latency {last_out - last_in}, "
            f"expected LATENCY={LATENCY} (= DEPTH-1 shift hops) under continuous valid"
        )
        dut._log.info(f"KA[{label}] OK: both tags at exactly LATENCY={LATENCY} clocks "
                      f"under continuous valid")
        return trace
    return _run


def report(dut, label, trace, marks):
    """One log line per number a report will later quote (§22.67, §22.68)."""
    pub = published(trace)
    dut._log.info(
        f"7H[{label}]: clocks={len(trace)} presented={trace[-1]['n_pres']} "
        f"published={len(pub)} real={len(real_published(trace))} "
        f"stale={len(stale_published(trace))} stranded={len(stranded(trace))} "
        f"marks={ {hex(k) if isinstance(k,int) else k: v for k, v in marks.items()} }"
    )
    for tag, name in ((TAG_FIRST, "first"), (TAG_LAST, "last"),
                      (0b0100, "p2first"), (0b1000, "p2last/os")):
        any_clk = tag_out_clk(trace, tag)
        pub_clk = tag_out_clk_published(trace, tag)
        if any_clk is not None or pub_clk is not None:
            dut._log.info(f"7H[{label}]:   tag {name} ({tag:#06b}) "
                          f"out@{any_clk} published@{pub_clk}")
    return pub
