"""§63 #7h Phase 1 — does a packet's tail wait for the next ordered-set slot?

MEASUREMENT ONLY (D-7H.1, D-7H.2).  Zero `src/` edits in this phase.  Each row
below reports numbers and asserts only two things: the known-answer self-test
that licenses the numbers, and a non-vacuity check that the stimulus reached the
DUT (§22.82).  **No row asserts a spec property** — the fix shape is chat's to
choose at the Phase 2 STOP, and a row that asserted the answer here would be
deciding it.

Toplevel: `scrambler`, the wrapper that `phy_transmit.sv:153` instantiates on
the TX path and `phy_receive.sv:143` instantiates on the RX path.  ⭐ The two
directions are the SAME MODULE, not merely the same class, so one row measures
both — which is why D-7H.3 names the ownership unit "shared PHY TX/RX".

WHAT THE ROWS COVER
-------------------
  1-a   lone 10-beat and lone 12-beat packet after long idle      (H7H-1, C7H-1)
  1-b   two packets back-to-back                                  (H7H-2, C7H-3)
  1-c   isolated 4-beat DLLP-shaped packet, quiet either side     (H7H-3)
  1-d   the same at the RX seam + what releases the tail          (H7H-4, C7H-4)

Predictions being scored: PREDICTIONS_7H.md, committed before this file existed.
"""
import cocotb
from cocotb.clock import Clock

from residue_7h_common import (DEPTH, LATENCY, TAG_FIRST, TAG_LAST, drive,
                               packet,
                               known_answer, published, real_published, report,
                               residue_at, sched_lone, sched_packet_then_os,
                               sched_two_back_to_back, stale_published,
                               stranded, tag_out_clk, tag_out_clk_published)


def pinned_red(dut, row, state, detail=""):
    """§22.93 — an expect_fail row must fail AT its one named, pinned assertion.

    cocotb's expect_fail turns ANY exception into a PASS, so a row can be red
    for a reason unrelated to the defect it pins and the gate cannot tell.
    Everything before the pinned assertion runs inside a try/except; an
    exception there is logged NOT_REACHED and the row RETURNS NORMALLY, which
    under expect_fail is a gate FAIL.  Then the REACHED marker, then the pinned
    assertion -- the only statement allowed to raise.  sweep43.sh copies these
    into its .diag as PINNED| rows.
    """
    dut._log.info("PINNED_RED|%s|%s|%s", row, state, detail)


# "After a long idle" / "quiet either side".  700 exceeds the 679-cycle COM grid
# #7f measured, so a packet placed here cannot be adjacent to one by accident.
LONG_IDLE = 720


@cocotb.test()
async def test_7h_known_answer(dut):
    """The self-test that opens every analysis (§22.92).

    Under continuous valid a tag presented on clock t must appear at
    data_k_out_o on clock t+DEPTH.  If this fails, no other number in this file
    means anything.
    """
    cocotb.start_soon(Clock(dut.clk_i, 10, units="ns").start())
    await known_answer(dut, "self-test")()


@cocotb.test()
async def test_7h_1a_lone_packet_10_and_12(dut):
    """1-a — a lone packet after a long idle: how many beats come out, and when.

    Scores H7H-1 (7 words out, tail held) and C7H-1 (residue is exactly
    DEPTH-1 = 3 and does NOT scale with packet length: 10 -> 7 out, 12 -> 9 out).

    The discriminating number is the RESIDUE, not the count of beats published:
    #7f recorded "7 words come out" together with "residue does not depend on
    length", and those two are consistent only if the 7 is the 10-beat case.
    """
    cocotb.start_soon(Clock(dut.clk_i, 10, units="ns").start())
    await known_answer(dut, "1-a")()

    for n in (10, 12):
        sched, marks = sched_lone(n, lead_idle=LONG_IDLE, trail_idle=LONG_IDLE)
        trace = await drive(dut, sched)
        report(dut, f"1-a n={n}", trace, marks)

        # Non-vacuity: the packet really was presented, and the idle really was
        # idle (§22.82).
        assert trace[-1]["n_pres"] == n, (
            f"n={n}: bench presented {trace[-1]['n_pres']} beats, meant to present {n}"
        )
        lead = [r for r in trace if r["clk"] < marks[TAG_FIRST]]
        assert not any(r["presented"] for r in lead), \
            f"n={n}: a beat was presented during the leading idle -- idle did not open"

        last_in = marks[TAG_LAST]
        last_out_any = tag_out_clk(trace, TAG_LAST)
        last_out_pub = tag_out_clk_published(trace, TAG_LAST)
        first_out_pub = tag_out_clk_published(trace, TAG_FIRST)
        pub = published(trace)
        resid = residue_at(trace, last_in)

        dut._log.info(
            f"7H[1-a n={n}] MEASURED: presented={n} published={len(pub)} "
            f"real={len(real_published(trace))} stale={len(stale_published(trace))} "
            f"STRANDED={len(stranded(trace))} residue_at_last_presented={resid} "
            f"END tag in@{last_in} reached_output@{last_out_any} "
            f"published@{last_out_pub} STP published@{first_out_pub}"
        )
        if last_out_pub is None:
            dut._log.info(
                f"7H[1-a n={n}] the END tag was NEVER published in {len(trace)} "
                f"clocks -- the tail is still in the pipeline at end of trace"
            )
        else:
            dut._log.info(
                f"7H[1-a n={n}] END published {last_out_pub - last_in} clocks "
                f"after it was presented (continuous-valid latency is DEPTH={DEPTH})"
            )


@cocotb.test()
async def test_7h_1b_two_packets_back_to_back(dut):
    """1-b — H7H-2's discriminator, and C7H-3's.

    Send a second packet immediately behind the first.  If the tail is released
    by the arrival of further VALID BEATS (whatever carries them), the first
    packet's END emerges DEPTH clocks after it was presented, exactly as under
    continuous valid, and nothing waits for an ordered-set slot.

    If instead the release needs an ordered set AS an ordered set, the first
    END is still held here.
    """
    cocotb.start_soon(Clock(dut.clk_i, 10, units="ns").start())
    await known_answer(dut, "1-b")()

    sched, marks = sched_two_back_to_back(10, 10, lead_idle=LONG_IDLE,
                                          trail_idle=LONG_IDLE)
    trace = await drive(dut, sched)
    report(dut, "1-b", trace, marks)

    assert trace[-1]["n_pres"] == 20, \
        f"bench presented {trace[-1]['n_pres']} beats, meant to present 20"

    p1_last_in = marks[TAG_LAST]
    p1_last_out = tag_out_clk_published(trace, TAG_LAST)
    p2_last_in = marks[0b1000]
    p2_last_out = tag_out_clk_published(trace, 0b1000)

    dut._log.info(
        f"7H[1-b] MEASURED: packet-1 END in@{p1_last_in} published@{p1_last_out} "
        f"(latency {None if p1_last_out is None else p1_last_out - p1_last_in}); "
        f"packet-2 END in@{p2_last_in} published@{p2_last_out} "
        f"(latency {None if p2_last_out is None else p2_last_out - p2_last_in}); "
        f"continuous-valid latency is LATENCY={LATENCY}"
    )


@cocotb.test()
async def test_7h_1c_isolated_dllp(dut):
    """1-c — an isolated 4-beat DLLP-shaped packet, >= 700 quiet cycles either side.

    Scores H7H-3 (an isolated DLLP IS deferred the same way, 1 word out and 3
    held) against its counter (if it is not deferred, H7H-2 loses as the
    mechanism).

    ⚠️ #7f's "DLLP tail: zero deferral x5" is NOT evidence -- it came from the
    probe that mis-paired ENDs, on a back-to-back InitFC stream (brief §What is
    known).  Isolated-DLLP behaviour is unmeasured until this row runs.
    """
    cocotb.start_soon(Clock(dut.clk_i, 10, units="ns").start())
    await known_answer(dut, "1-c")()

    n = 4
    sched, marks = sched_lone(n, lead_idle=LONG_IDLE, trail_idle=LONG_IDLE)
    trace = await drive(dut, sched)
    report(dut, "1-c", trace, marks)

    assert trace[-1]["n_pres"] == n, \
        f"bench presented {trace[-1]['n_pres']} beats, meant to present {n}"

    last_in = marks[TAG_LAST]
    last_out_pub = tag_out_clk_published(trace, TAG_LAST)
    first_out_pub = tag_out_clk_published(trace, TAG_FIRST)
    pub = published(trace)
    dut._log.info(
        f"7H[1-c] MEASURED: presented={n} published={len(pub)} "
        f"real={len(real_published(trace))} STRANDED={len(stranded(trace))} "
        f"SDP published@{first_out_pub} END in@{last_in} published@{last_out_pub} "
        f"residue_at_last_presented={residue_at(trace, last_in)}"
    )


@cocotb.test()
async def test_7h_1d_what_releases_the_tail(dut):
    """1-d — is the release caused by valid beats as such, or by an ordered set?

    A packet, then a quiet gap, then a burst of plain valid beats carrying no
    COM and no ordered-set semantics at all.  If the held tail emerges on those
    beats, the ordered set is not a scheduler -- it is merely the only source of
    valid traffic during idle, which is C7H-3's claim and the reason #7f saw
    release correlate with the 679-cycle COM grid.

    Swept over burst sizes 1, 2, 3 and 4 so the number of valid beats needed to
    flush the tail is measured rather than inferred.
    """
    cocotb.start_soon(Clock(dut.clk_i, 10, units="ns").start())
    await known_answer(dut, "1-d")()

    for os_beats in (1, 2, 3, 4):
        sched, marks = sched_packet_then_os(10, lead_idle=LONG_IDLE, gap=200,
                                            os_beats=os_beats, trail_idle=200)
        trace = await drive(dut, sched)
        report(dut, f"1-d os={os_beats}", trace, marks)

        last_in = marks[TAG_LAST]
        os_at = marks["os"]
        last_out_pub = tag_out_clk_published(trace, TAG_LAST)
        dut._log.info(
            f"7H[1-d os={os_beats}] MEASURED: END in@{last_in}, "
            f"gap 200 clocks, burst of {os_beats} valid beats starts@{os_at}, "
            f"END published@{last_out_pub} "
            f"({'NOT published' if last_out_pub is None else str(last_out_pub - os_at) + ' clocks after the burst began'})"
        )


# ==========================================================================
#  ACCEPTANCE (chat, 2026-09-19).  These REPLACE D-7H.4(a), which Phase 1
#  showed has two limbs of which only one is violated: "no ordered set inside"
#  is already satisfied (zero K symbols strictly between STP and END, both
#  directions), so a row written against it would pass today and mislead.
#
#  ⭐ BOTH WERE GREEN under the self-draining chain (A+B+C, tag
#  evidence/7h-self-drain-C) and are RED again here, deliberately.
#
#  Kourosh took OPTION 3 on 2026-09-20: #21's fix is deferred to #7j (Logical
#  Idle), and A+B+C were reverted by revert commits.  The reason is not that
#  the fix failed -- it met all three acceptance criteria -- but that it moved
#  10 gap rows across verilate_scrambler_stall, verilate_scrambler_kgap and
#  verilate_scrambler_align, and keeping it would have meant loosening FA-2 and
#  FA-4 oracles to fit a timing-dependent key schedule, against D-7H.5.
#
#  The mechanism, in one line: the XOR takes its key from the LIVE LFSR at the
#  moment a word crosses stage 2->3, so once the chain drains, the key schedule
#  follows the DRAIN rather than the INPUT.  Logical Idle removes the gaps at
#  their cause, so under #7j the tension does not arise.
#
#  ⚠️ So these two rows are expect_fail, pinned per §22.93 to the bodies
#  MEASURED TODAY at 2de81bd's src/: a lone 10-beat packet publishes 7 real
#  words with 3 stale and 3 STRANDED, and its END is never published in 720
#  trailing idle cycles.  They carry to #7j and are its acceptance rows.
#
#  ⚠️ §22.87: their premises expire the moment Logical Idle lands.  Flipping
#  them means REWRITING THESE BODIES, not deleting the markers -- the span
#  assertion in particular opens with `lone_span is None`, which is a statement
#  about a DEFECT and is false once the defect is gone.
# ==========================================================================

@cocotb.test(expect_fail=True)  # §63 #7h -> #7j (Kourosh, 2026-09-20)
async def test_7h_acceptance_i_lone_packet_publishes_its_end(dut):
    """ACCEPTANCE (i) — a lone packet with nothing behind it publishes its END.

    RED BEFORE FIX: measured in 1-a, the END tag never reaches data_out_o in
    720 idle cycles at either packet length.  The words are not late, they are
    stranded: with no further valid beats the chain never shifts again.

    This is the acceptance row for the "no gap" limb of the old D-7H.4(a),
    stated as a property of the DUT rather than of the wire, so it can be
    measured at the unit seam where the schedule is the bench's own.
    """
    ROW = "test_7h_acceptance_i_lone_packet_publishes_its_end"
    cocotb.start_soon(Clock(dut.clk_i, 10, units="ns").start())
    try:
        await known_answer(dut, "acc-i")()
        measured = []
        for n in (10, 12, 4):
            sched, marks = sched_lone(n, lead_idle=LONG_IDLE, trail_idle=LONG_IDLE)
            trace = await drive(dut, sched)
            report(dut, f"acc-i n={n}", trace, marks)
            if trace[-1]["n_pres"] != n:
                raise AssertionError(
                    f"n={n}: bench presented {trace[-1]['n_pres']} beats, meant {n}")
            measured.append((n, marks[TAG_LAST],
                             tag_out_clk_published(trace, TAG_LAST),
                             len(stranded(trace)),
                             len(real_published(trace))))
            dut._log.info(
                f"7H[acc-i n={n}] END in@{measured[-1][1]} published@{measured[-1][2]} "
                f"real={measured[-1][4]} stranded={measured[-1][3]}")
    except Exception as exc:                                    # noqa: BLE001
        pinned_red(dut, ROW, "NOT_REACHED", f"{type(exc).__name__}: {exc}")
        return

    detail = " ".join(f"n={n}:end_pub={pub},stranded={st},real={rl}"
                      for n, _, pub, st, rl in measured)
    pinned_red(dut, ROW, "REACHED", detail)
    unpublished = [(n, st) for n, _, pub, st, _ in measured if pub is None]
    assert not unpublished, (
        f"the packet's END was NEVER published in {LONG_IDLE} trailing idle "
        f"cycles, at lengths {[n for n, _ in unpublished]} "
        f"(stranded beats {[st for _, st in unpublished]}).  A packet with "
        f"nothing behind it does not get transmitted."
    )


@cocotb.test(expect_fail=True)  # §63 #7h -> #7j (Kourosh, 2026-09-20)
async def test_7h_acceptance_ii_span_independent_of_following_traffic(dut):
    """ACCEPTANCE (ii) — STP→END span identical with and without following traffic.

    RED BEFORE FIX: with traffic behind, 1-b measured packet 1's END published
    at latency 3 (span 9 for a 10-beat packet).  With nothing behind, 1-a
    measured the END never published at all.  The span therefore depends on what
    comes AFTER the packet, which is the defect stated as an invariant.

    ⚠️ Measured between the two PUBLISHED framing tags, not from the
    presentation clocks: the claim is about what a consumer sees on the wire,
    and D-7H.4(a) was a statement about the PIPE TX.  Only timing may move
    (chat's fence); this row says the SPAN may not.
    """
    ROW = "test_7h_acceptance_ii_span_independent_of_following_traffic"
    cocotb.start_soon(Clock(dut.clk_i, 10, units="ns").start())
    try:
        await known_answer(dut, "acc-ii")()
        n = 10
        # (a) nothing behind
        sched, marks = sched_lone(n, lead_idle=LONG_IDLE, trail_idle=LONG_IDLE)
        lone = await drive(dut, sched)
        report(dut, "acc-ii lone", lone, marks)
        lone_stp = tag_out_clk_published(lone, TAG_FIRST)
        lone_end = tag_out_clk_published(lone, TAG_LAST)

        # (b) a second packet immediately behind
        sched2, marks2 = sched_two_back_to_back(n, n, lead_idle=LONG_IDLE,
                                                trail_idle=LONG_IDLE)
        behind = await drive(dut, sched2)
        report(dut, "acc-ii behind", behind, marks2)
        beh_stp = tag_out_clk_published(behind, TAG_FIRST)
        beh_end = tag_out_clk_published(behind, TAG_LAST)

        dut._log.info(
            f"7H[acc-ii] lone: STP@{lone_stp} END@{lone_end}; "
            f"with-traffic: STP@{beh_stp} END@{beh_end}")

        # Non-vacuity belongs INSIDE the guard: a packet whose STP never
        # published would make the span question meaningless, and that is a
        # broken bench, not the defect this row pins.
        if lone_stp is None or beh_stp is None or beh_end is None:
            raise AssertionError(
                f"a framing tag was not published at all "
                f"(lone STP={lone_stp}, behind STP={beh_stp}, behind END={beh_end})")
        lone_span = None if lone_end is None else lone_end - lone_stp
        beh_span = beh_end - beh_stp
    except Exception as exc:                                    # noqa: BLE001
        pinned_red(dut, ROW, "NOT_REACHED", f"{type(exc).__name__}: {exc}")
        return

    pinned_red(dut, ROW, "REACHED",
               f"lone_span={lone_span} behind_span={beh_span}")
    assert lone_span == beh_span, (
        f"STP->END span is {lone_span} with nothing behind the packet "
        f"({'the END was never published' if lone_span is None else 'cycles'}) "
        f"and {beh_span} with a packet behind it -- the span depends on "
        f"following traffic"
    )
    dut._log.info(f"7H[acc-ii] OK: span {lone_span} both ways")


# ==========================================================================
#  THE CLOSED FENCE.  Chat's Phase-3 fence asks for the published-valid word
#  sequence to be byte-identical old tree vs new tree, "only timing may move".
#
#  ⚠️ THE FOUR MIRROR TARGETS CANNOT DISCHARGE THAT CLAIM, AND THE REASON IS
#  MEASURED, NOT ARGUED.  test_phy_transmit_stall and test_scrambler_align both
#  OBSERVE the signal this rung changes, so their stimulus responds to it: the
#  scrambler's INPUT dump moved too (188 -> 189 beats for phy_transmit_stall,
#  378 -> 285 for scrambler_align).  A byte-identical output was never available
#  from a bench whose input is a function of the output.  fence_tx_framing is
#  the exception and IS byte-identical (13 of 13), because frame_symbols sits
#  upstream and nothing it sees changed.
#
#  So this row is the closed version: a FIXED schedule, written here, that no
#  DUT signal can perturb.  Run it on the old tree and the new tree and the
#  input is identical BY CONSTRUCTION, which is what makes the output diff
#  attributable to the fix alone (§22.80 -- a control must not be computed from
#  the signal under test).
#
#  It asserts nothing about old-vs-new; it DUMPS.  The comparison is offline,
#  in scripts_7h_fence_diff.py, which opens with its own known-answer test.
# ==========================================================================

@cocotb.test()
async def test_7h_fence_closed_stimulus_dump(dut):
    """The fence, on a stimulus nothing can perturb.  Dumps; asserts nothing.

    The schedule deliberately exercises every case the fix touches: a lone
    packet (tail stranded on the old tree), a mid-stream gap (the FA-4 K-gap
    case), K codes across that gap, and back-to-back packets (the case that
    always worked).  If the scrambler's arithmetic moved anywhere, it moved
    here.
    """
    cocotb.start_soon(Clock(dut.clk_i, 10, units="ns").start())
    await known_answer(dut, "fence")()

    sched = []
    # (1) a lone 10-beat packet after idle -- the stranded-tail case
    for _ in range(40):
        sched.append((0, 0, False))
    for d, k in packet(10, salt=3):
        sched.append((d, k, True))
    for _ in range(40):
        sched.append((0, 0, False))
    # (2) a packet with a mid-stream gap -- FA-4's K-gap shape
    pk = packet(12, salt=5)
    for i, (d, k) in enumerate(pk):
        if i == 6:
            for _ in range(7):
                sched.append((0, 0, False))
        sched.append((d, k, True))
    for _ in range(40):
        sched.append((0, 0, False))
    # (3) two packets back to back -- the case that always worked
    for d, k in packet(8, salt=7):
        sched.append((d, k, True))
    for d, k in packet(8, salt=9):
        sched.append((d, k, True))
    # (4) enough trailing idle for a self-draining chain to finish
    for _ in range(40):
        sched.append((0, 0, False))

    trace = await drive(dut, sched)
    pub = published(trace)
    dut._log.info(f"7H[fence] schedule={len(sched)} clocks, presented="
                  f"{trace[-1]['n_pres']}, published={len(pub)}")
    for i, r in enumerate(pub):
        dut._log.info(f"FENCE7HC seq={i} cyc={r['clk']} "
                      f"data=0x{r['data']:08x} k=0x{r['k']:01x}")
    dut._log.info(f"7H[fence] END of dump: {len(pub)} published beats")
