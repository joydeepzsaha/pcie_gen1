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
#  MEASURED at 2de81bd's src/: a lone 10-beat packet publishes 7 real words
#  with 3 stale and 3 STRANDED, and its END is never published in 720 trailing
#  idle cycles.
#
#  ==========================================================================
#  §63 #7j-2 -- RE-PREMISED, IN THE COMMIT THAT MOVED THE RTL (D-7J.4, §22.87).
#  THEY STAY expect_fail, AND THE REASON IS WORTH STATING EXACTLY.
#
#  The block above ends "They carry to #7j and are its acceptance rows."  That
#  sentence is the premise that expired, and #7j-2 is where it expired.  It was
#  written on the assumption that whatever fixed #21 would be visible HERE.
#  #7j-2's fix is not: it is one LTSSM state and one arbitration decision, and
#  neither is in this bench's hierarchy -- `gen1_scramble` is the toplevel of
#  verilate_7h_residue and neither pcie_ltssm_downstream nor lane_management is
#  instantiated under it.
#
#  ⭐ AND THAT IS NOT AN ACCIDENT OF PLACEMENT, IT IS THE FIX'S SHAPE.  These
#  rows supply their own trailing schedule -- `sched_lone(..., trail_idle=
#  LONG_IDLE)`, where "idle" means valid LOW.  #7j-2 does not change what
#  gen1_scramble does when valid goes low; it changes the fact that on a
#  conformant Link in L0 VALID NEVER GOES LOW AT ALL (Base 2.1 §4.2.2 p.195,
#  every Symbol Time carries a Symbol).  The defect these rows pin is real and
#  is still there; #7j-2 makes its TRIGGER unreachable in L0 rather than
#  removing the mechanism.  A bench that manufactures the trigger still sees it.
#
#  ⚠️ So the honest reading of these two rows changes, and their colour does
#  not: they stop being "#7j's acceptance rows" and become the standing
#  characterisation of the frozen chain, for the states where the wire may
#  legitimately go quiet -- Electrical Idle, L0s, recovery.  They are
#  registered to #7g with the receive-side fragment item (#7j-1), which is the
#  same defect seen from the other end.
#
#  ⚠️ THE ACCEPTANCE ROWS FOR #7j-2 ARE ELSEWHERE, and they are named here so
#  nobody re-derives this: tb/phy_tx_golden/test_7j2_idle.py C1-C6 at the
#  phy_transmit seam, tb/ltssm/test_ltssm_l0.py's two new rows at the LTSSM
#  seam, and test_pcie_fullstack.py's acceptance (a) and (b) at the full stack.
#  What C6 measures is this defect's own signature on a link with nothing
#  behind the packet: STP published at 512 and END at 1368, 856 cycles later,
#  against 3 cycles once Logical Idle fills the gap.
#
#  ⚠️ Their BODIES are unchanged, deliberately.  §22.87 says flipping a row
#  means rewriting its body -- these rows do not flip, so rewriting the
#  measurement would destroy the red-before-fix record for #7g without buying
#  anything.  What is rewritten is the premise, which is this comment.
# ==========================================================================

# ==========================================================================
#  §63 #7g-1 -- FLIPPED TO ORDINARY ROWS (D-7G.8, Kourosh 2026-09-23).
#
#  These two were permanently-`expect_fail` rows pinned to a mechanism that
#  #7j-2 made UNREACHABLE IN L0.  §22.84 says register status and test
#  existence are independent axes; a red row about a trigger that no
#  conformant link can produce sits in the worst corner of that grid -- the
#  artifact shows no red (an `expect_fail` row reports STATUS=PASS, §22.77)
#  and the register shows `open`, so both surfaces read as "nothing here".
#
#  ⚠️ §22.87: FLIPPING A ROW MEANS REWRITING ITS BODY, NOT DELETING THE
#  MARKER.  The old bodies asserted the DEFECT -- "the END was never published
#  in 720 trailing idle cycles" -- and that premise is still true of the
#  schedule they drove.  Keeping the body and dropping `expect_fail` would
#  turn a correct measurement into a failing row.  So the bodies below are new:
#  they assert the CHAIN PROPERTY positively, on the schedule a conformant link
#  actually presents.
#
#  THE PROPERTY, stated once: `gen1_scramble`'s XOR takes its key from the live
#  LFSR as a word crosses stage 2->3, so a word needs THREE further valid beats
#  to reach the output.  On a conformant Link in L0 those beats always exist --
#  Base 2.1 §4.2.2 p.195, every Symbol Time carries a Symbol, which is what
#  #7j-2 made true of this design by generating Logical Idle.  So the invariant
#  is: WITH >= 3 FOLLOWING VALID BEATS, END PUBLISHES AT LATENCY 3, and the
#  STP->END span does not depend on what follows.
#
#  ⚠️ What is NOT claimed: that the drain defect is fixed.  It is not.  With
#  valid LOW behind the packet the chain still strands its last three words,
#  and that is registered for the states where the wire may legitimately go
#  quiet -- Electrical Idle, L0s, Recovery -- together with #7j-1's
#  receive-side fragment, which is the same defect from the other end.  The
#  old red rows' measurement is preserved in this file's history and in
#  `REPORT_7H_PHASE3_STOP.md`; deleting a red row does not delete its evidence.
# ==========================================================================


@cocotb.test()  # §63 #7g-1, was expect_fail through #7h/#7j (D-7G.8)
async def test_7h_end_publishes_at_latency_3_with_following_valid_beats(dut):
    """The chain publishes a packet's END at latency 3 once it is fed.

    Drives a packet and then >= 3 further valid beats -- the L0 schedule, where
    Logical Idle guarantees a Symbol every Symbol Time -- and asserts the END
    tag reaches the output exactly 3 clocks after it was presented, at three
    packet lengths.  This is the positive statement of the property the old
    acceptance row (i) pinned from its failing side.
    """
    cocotb.start_soon(Clock(dut.clk_i, 10, units="ns").start())
    await known_answer(dut, "g1-latency")()
    measured = []
    for n in (10, 12, 4):
        sched, marks = sched_packet_then_os(
            n, lead_idle=LONG_IDLE, gap=0, os_beats=3, trail_idle=LONG_IDLE)
        trace = await drive(dut, sched)
        report(dut, f"g1-latency n={n}", trace, marks)
        assert trace[-1]["n_pres"] == len(
            [b for b in sched if b[2]]), (
            f"n={n}: bench presented {trace[-1]['n_pres']} valid beats, "
            f"schedule holds {len([b for b in sched if b[2]])}")
        pres = marks[TAG_LAST]
        pub = tag_out_clk_published(trace, TAG_LAST)
        assert pub is not None, (
            f"n={n}: END was never published although {3} valid beats follow "
            f"the packet -- the L0 invariant this row asserts does not hold")
        measured.append((n, pres, pub, pub - pres))
        dut._log.info(f"7G1[latency n={n}] END in@{pres} published@{pub} "
                      f"latency={pub - pres}")
    bad = [(n, lat) for n, _, _, lat in measured if lat != 3]
    assert not bad, (
        f"END published at latency {[l for _, l in bad]} at lengths "
        f"{[n for n, _ in bad]}, expected 3 at every length -- the chain is "
        f"3 stages deep and each stage costs exactly one valid beat")


@cocotb.test()  # §63 #7g-1, was expect_fail through #7h/#7j (D-7G.8)
async def test_7h_span_is_invariant_once_the_chain_is_fed(dut):
    """STP->END span does not depend on what follows, once the chain is fed.

    The old acceptance row (ii) compared a packet with NOTHING behind it
    against one with a packet behind it, and the span differed because the
    first case never published its END at all.  That comparison is about the
    drain, not about the span.  This row makes the comparison the invariant
    actually claims: two schedules that BOTH keep valid high behind the packet
    -- three filler beats in one, a whole second packet in the other -- must
    produce the same STP->END span.

    ⚠️ Measured between the two PUBLISHED framing tags, not the presentation
    clocks: the claim is about what a consumer sees on the wire.
    """
    cocotb.start_soon(Clock(dut.clk_i, 10, units="ns").start())
    await known_answer(dut, "g1-span")()
    n = 10

    sched_f, marks_f = sched_packet_then_os(
        n, lead_idle=LONG_IDLE, gap=0, os_beats=3, trail_idle=LONG_IDLE)
    filled = await drive(dut, sched_f)
    report(dut, "g1-span filler", filled, marks_f)
    f_stp = tag_out_clk_published(filled, TAG_FIRST)
    f_end = tag_out_clk_published(filled, TAG_LAST)

    sched_b, marks_b = sched_two_back_to_back(
        n, n, lead_idle=LONG_IDLE, trail_idle=LONG_IDLE)
    behind = await drive(dut, sched_b)
    report(dut, "g1-span behind", behind, marks_b)
    b_stp = tag_out_clk_published(behind, TAG_FIRST)
    b_end = tag_out_clk_published(behind, TAG_LAST)

    dut._log.info(f"7G1[span] filler: STP@{f_stp} END@{f_end}; "
                  f"behind: STP@{b_stp} END@{b_end}")

    # Non-vacuity: a span computed from a tag that never published is not a
    # span (§22.82).  This is an assertion, not a guard -- if a tag is missing
    # on either schedule the invariant is untestable and the row must say so.
    assert None not in (f_stp, f_end, b_stp, b_end), (
        f"a framing tag was not published (filler STP={f_stp} END={f_end}, "
        f"behind STP={b_stp} END={b_end}) -- the span is undefined")

    f_span, b_span = f_end - f_stp, b_end - b_stp
    assert f_span == b_span, (
        f"STP->END span is {f_span} with three filler beats behind the packet "
        f"and {b_span} with a whole packet behind it -- once the chain is fed "
        f"the span must not depend on WHAT feeds it")
    dut._log.info(f"7G1[span] OK: span {f_span} both ways")


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
