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
