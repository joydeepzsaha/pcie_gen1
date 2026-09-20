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
                               known_answer, published, real_published, report,
                               residue_at, sched_lone, sched_packet_then_os,
                               sched_two_back_to_back, stale_published,
                               stranded, tag_out_clk, tag_out_clk_published)

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
#  Both rows are RED before the fix and must go green after it.  They are
#  ordinary rows, NOT expect_fail: a red row that reports PASS cannot witness
#  its own flip (§22.77), and these two exist precisely to witness one.
#
#  They are deliberately NOT in sweep43.sh yet.  A red row landing in the gate
#  would make the gate red for the commits between the test and the fix; they
#  join the gate at Phase 5, green, which is also what makes them mutation
#  -testable oracles (§22.77 corollary).
# ==========================================================================

@cocotb.test()
async def test_7h_acceptance_i_lone_packet_publishes_its_end(dut):
    """ACCEPTANCE (i) — a lone packet with nothing behind it publishes its END.

    RED BEFORE FIX: measured in 1-a, the END tag never reaches data_out_o in
    720 idle cycles at either packet length.  The words are not late, they are
    stranded: with no further valid beats the chain never shifts again.

    This is the acceptance row for the "no gap" limb of the old D-7H.4(a),
    stated as a property of the DUT rather than of the wire, so it can be
    measured at the unit seam where the schedule is the bench's own.
    """
    cocotb.start_soon(Clock(dut.clk_i, 10, units="ns").start())
    await known_answer(dut, "acc-i")()

    for n in (10, 12, 4):
        sched, marks = sched_lone(n, lead_idle=LONG_IDLE, trail_idle=LONG_IDLE)
        trace = await drive(dut, sched)
        report(dut, f"acc-i n={n}", trace, marks)

        assert trace[-1]["n_pres"] == n, \
            f"n={n}: bench presented {trace[-1]['n_pres']} beats, meant to present {n}"

        last_in = marks[TAG_LAST]
        last_out = tag_out_clk_published(trace, TAG_LAST)
        dut._log.info(f"7H[acc-i n={n}] END in@{last_in} published@{last_out} "
                      f"stranded={len(stranded(trace))}")
        assert last_out is not None, (
            f"n={n}: the packet's END was NEVER published in "
            f"{LONG_IDLE} trailing idle cycles -- {len(stranded(trace))} presented "
            f"beats never reached data_out_o at all.  A packet with nothing "
            f"behind it does not get transmitted."
        )


@cocotb.test()
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
    cocotb.start_soon(Clock(dut.clk_i, 10, units="ns").start())
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
        f"with-traffic: STP@{beh_stp} END@{beh_end}"
    )

    assert lone_stp is not None and beh_stp is not None, \
        f"a packet's STP was not published at all (lone={lone_stp}, behind={beh_stp})"
    assert lone_end is not None, (
        f"the lone packet's END was never published, so its STP->END span does "
        f"not exist -- the span depends on what follows the packet, which is "
        f"exactly what this row forbids"
    )
    lone_span = lone_end - lone_stp
    beh_span = beh_end - beh_stp
    assert lone_span == beh_span, (
        f"STP->END span is {lone_span} with nothing behind the packet and "
        f"{beh_span} with a packet behind it -- the span depends on following "
        f"traffic"
    )
    dut._log.info(f"7H[acc-ii] OK: span {lone_span} both ways")
