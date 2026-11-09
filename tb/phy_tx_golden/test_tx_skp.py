"""Rung 9 -- SKP Ordered Set bench for phy_transmit (Gen1, x1).

Toplevel: phy_transmit, MAX_NUM_LANES=1, with link_up_i RAISED.

⚠️  THIS PATH HAD NEVER EXECUTED.  os_generator's SKP timer advances only under
    `if (link_up_i)` (os_generator.sv:139-141) and is tested only in ST_IDLE
    (os_generator.sv:158).  All three pre-Rung-9 TX benches hold link_up_i at 0
    and never raise it (test_phy_transmit_tx.py:84, ..._x4.py:80,
    test_os_generator_k_mask.py:64), and os_generator is elaborated by exactly
    those three gate rows.  ST_SKP was therefore dead in all 71 rows of the
    03cea650 baseline.  Every assertion in this file is the first of its kind.

ORACLES (evidence/rung9/ORACLES_PHY_TX.md)
    O-7  "The transmitted SKP Ordered Set is: one COM Symbol followed by three
         consecutive SKP Symbols."         Base 2.1 sec 4.2.7.1 p.261
    O-8  "The SKP Ordered Set shall be scheduled for insertion at an interval
         between 1180 and 1538 Symbol Times."   Base 2.1 sec 4.2.7.1 p.261
    O-7b "Scheduled SKP Ordered Sets shall be transmitted if a packet or Ordered
         Set is not already in progress."       Base 2.1 sec 4.2.7.1 p.261

skp_due_during_a_packet_goes_before_the_next_packet (O-7b, in L0 with packets;
LR2 group C) is green from the SKP deferral's fix.
"""
import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ClockCycles, Timer

COM = 0xBC          # K28.5  (pkg:100)
SKP = 0x1C          # K28.0  (pkg:106)
TS1 = 0x4A
PAD = 0xF7
N_FTS = 0xFF
GEN1 = 0x01
GEN1_BASIC = 0x02

G_VALID   = 1 << 0
G_GEN_TS1 = 1 << 1
G_SET_LANE = 1 << 9

# Base 2.1 sec 4.2.7.1 p.261
SKP_MIN_SYMBOL_TIMES = 1180
SKP_MAX_SYMBOL_TIMES = 1538


def pack_tsos(link_num=PAD, lane_num=PAD, ts_disc=TS1):
    b = [COM, link_num & 0xFF, lane_num & 0xFF, N_FTS, GEN1_BASIC, 0x00] \
        + [ts_disc & 0xFF] * 10
    v = 0
    for i, bv in enumerate(b):
        v |= bv << (8 * i)
    return v


async def start_clocks(dut):
    cocotb.start_soon(Clock(dut.clk_i, 10, units="ns").start())
    cocotb.start_soon(Clock(dut.pipe_rx_usr_clk_i, 10, units="ns").start())
    cocotb.start_soon(Clock(dut.pipe_tx_usr_clk_i, 10, units="ns").start())


async def reset(dut):
    dut.rst_i.value = 1
    dut.en_i.value = 0
    dut.link_up_i.value = 0
    dut.num_active_lanes_i.value = 1
    dut.send_ordered_set_i.value = 0
    dut.ordered_set_i.value = 0
    dut.gen_os_ctrl_i.value = 0
    dut.curr_data_rate_i.value = GEN1
    dut.s_dllp_axis_tdata.value = 0
    dut.s_dllp_axis_tkeep.value = 0
    dut.s_dllp_axis_tvalid.value = 0
    dut.s_dllp_axis_tlast.value = 0
    dut.s_dllp_axis_tuser.value = 0
    await ClockCycles(dut.pipe_tx_usr_clk_i, 8)
    dut.rst_i.value = 0
    await ClockCycles(dut.pipe_tx_usr_clk_i, 4)
    dut.en_i.value = 1


def syms_per_word(dut):
    """Symbols per PIPE word, read from the DUT's own pipe_width_o.  At Gen1 this
    is 16 >> 3 = 2, so one pipe_tx_usr_clk cycle is TWO Symbol Times -- the
    conversion factor the interval assertion needs, and the reason it is read
    rather than hardcoded."""
    return max(1, min(4, int(dut.pipe_width_o.value) >> 3))


async def collect(dut, cycles):
    """(symbol, is_k, symbol_time) for lane 0.  symbol_time counts Symbol Times
    from the start of collection, using the DUT's PIPE width."""
    out = []
    t = 0
    for _ in range(cycles):
        await RisingEdge(dut.pipe_tx_usr_clk_i)
        word = int(dut.pipe_data_o.value) & 0xFFFFFFFF
        dk = int(dut.pipe_data_k_o.value) & 0xF
        n = syms_per_word(dut)
        for j in range(n):
            out.append(((word >> (8 * j)) & 0xFF, (dk >> j) & 1, t))
            t += 1
    return out


def skp_starts(stream):
    """Symbol-time index of every COM that begins a COM,SKP,SKP,SKP group."""
    hits = []
    for i in range(len(stream) - 3):
        s0, k0, t0 = stream[i]
        if s0 == COM and k0 == 1:
            nxt = [(s, k) for s, k, _ in stream[i + 1:i + 4]]
            if nxt == [(SKP, 1)] * 3:
                hits.append(t0)
    return hits


def fmt(stream, a, b):
    return " ".join("%02x%s" % (s, "K" if k else "") for s, k, _ in stream[a:b])


# ------------------------------------------------------------------ O-7

@cocotb.test()
async def skp_ordered_set_composition(dut):
    """O-7: Base 2.1 sec 4.2.7.1 p.261 -- "The transmitted SKP Ordered Set is:
    one COM Symbol followed by three consecutive SKP Symbols."

    COM is K28.5 = BCh, SKP is K28.0 = 1Ch (pcie_phy_pkg.sv:100,106).  All four
    are K codes, so sec 4.2.3 p.199 keeps them unscrambled and they must appear
    literally on the wire.

    First execution of ST_SKP in the project's history.
    """
    await start_clocks(dut)
    await reset(dut)
    dut.gen_os_ctrl_i.value = 0          # park os_generator in ST_IDLE
    dut.link_up_i.value = 1              # arm os_generator.sv:139-141

    # ⚠️ Window WIDENED in FA-5b -- a TEST change forced by a DESIGN fix, not a
    # fix to make a row pass.  SS54 #9(C) moved the SKP interval from 354 to a
    # spec-legal 1358 Symbol Times, so the old 600-clock (1200 Symbol Time)
    # window no longer reaches the first SKP.  PREDICTIONS_34B.md C3
    # pre-registered this row going red for want of window.
    #
    # ⚠️ AND the first widening -- one interval plus margin -- was STILL too
    # short, which is the more useful half of the lesson.  The first SKP does not
    # arrive one interval after link_up_i; it arrives about THREE.  That was
    # already visible in the pre-fix numbers had anyone looked: at 354 the first
    # landed at Symbol Time 1080 ~= 3 x 354, and the old 1200-Symbol-Time window
    # caught it only just.  At 1358 the first lands at 4092 ~= 3 x 1358 -- the
    # same structure, scaled.
    #
    # So the budget is taken from the SIBLING row that is proven to reach two
    # SKPs (the O-8 interval row uses four spec-maximum intervals), rather than
    # invented here.  Derived from the spec bound, so it stays correct for any
    # conforming constant.
    cycles = 4 * SKP_MAX_SYMBOL_TIMES // 2 + 64     # 3140 clocks = 6280 Symbol Times
    stream = await collect(dut, cycles)
    hits = skp_starts(stream)
    assert hits, "no COM,SKP,SKP,SKP group in %d Symbol Times after link_up_i " \
                 "(os_generator.sv:158 fires at 176 counts): %s" \
                 % (len(stream), fmt(stream, 0, 24))
    i = next(n for n, (s, k, t) in enumerate(stream) if t == hits[0])
    dut._log.info("SKP Ordered Set at Symbol Time %d: %s" % (hits[0], fmt(stream, i, i + 4)))
    got = [(s, k) for s, k, _ in stream[i:i + 4]]
    want = [(COM, 1), (SKP, 1), (SKP, 1), (SKP, 1)]
    assert got == want, "SKP OS is %s, sec 4.2.7.1 p.261 wants bcK 1cK 1cK 1cK" \
                        % fmt(stream, i, i + 4)
    dut._log.info("ST_SKP reached and its composition matches sec 4.2.7.1 p.261")


# ------------------------------------------------------------------ O-8

@cocotb.test()
async def skp_scheduling_interval_is_1180_to_1538_symbol_times(dut):
    """O-8: Base 2.1 sec 4.2.7.1 p.261 -- "The SKP Ordered Set shall be scheduled
    for insertion at an interval between 1180 and 1538 Symbol Times."

    Base 3.0 repeats the same two numbers; MindShare p.391 states them and
    defines the unit ("a Symbol time is the time required to send one Symbol and
    is 10 bit times, so at 2.5 GT/s, a Symbol time is 4ns").

    The interval is MEASURED here -- SKP COM to SKP COM on the wire, converted to
    Symbol Times through the DUT's own pipe_width_o -- and then compared to the
    window.  Nothing is hardcoded except the two spec numbers.

    WAS a predicted divergence (Rung 9, PREDICTIONS_R9.md sec 3, O-8) and carried
    expect_fail until FA-5b.  os_generator fired at 0xB0 = 176 counts of
    pipe_rx_usr_clk_i, scheduling one SKP every 354 Symbol Times -- roughly 3.3x
    more often than the 1180 floor.  Over-frequent SKPs are not a silent
    inefficiency: every one costs four Symbol Times of link bandwidth, and
    sec 4.2.7.2 p.261 only obliges a Receiver to tolerate an AVERAGE interval
    "between 1180 to 1538 Symbol Times".

    SS54 #9(C) moved the constant to 0x2A6 = 678, solved from the MEASURED
    relation interval = 2N + 2 (two Symbol Times per Gen1 PIPE-16 clock, plus one
    clock for ST_SKP itself).  Measured after the fix: a single interval of
    exactly 1358 Symbol Times, the window centre, predicted before the run.

    ⚠️ Rung 9's docstring said "about 356"; the artifact says 354, eight times
    identically.  The register's 354 was right.
    """
    await start_clocks(dut)
    await reset(dut)
    dut.gen_os_ctrl_i.value = 0
    dut.link_up_i.value = 1

    # Two SKPs are needed for one interval.  Bound the wait at four nominal
    # spec-maximum intervals so a design that never emits one fails fast.
    budget_symbol_times = 4 * SKP_MAX_SYMBOL_TIMES
    cycles = budget_symbol_times // 2 + 64
    stream = await collect(dut, cycles)
    hits = skp_starts(stream)
    dut._log.info("SKP Ordered Sets at Symbol Times: %s (window %d Symbol Times)"
                  % (hits[:8], len(stream)))
    assert len(hits) >= 2, \
        "need two SKP Ordered Sets to measure an interval; saw %d in %d Symbol Times" \
        % (len(hits), len(stream))

    intervals = [b - a for a, b in zip(hits, hits[1:])]
    measured = intervals[0]
    dut._log.info("measured SKP intervals (Symbol Times): %s" % intervals)
    dut._log.info("spec window sec 4.2.7.1 p.261: %d .. %d Symbol Times"
                  % (SKP_MIN_SYMBOL_TIMES, SKP_MAX_SYMBOL_TIMES))
    assert SKP_MIN_SYMBOL_TIMES <= measured <= SKP_MAX_SYMBOL_TIMES, \
        "SKP scheduled every %d Symbol Times; sec 4.2.7.1 p.261 requires %d..%d " \
        "(os_generator.sv:158 counts to 0xB0 = 176 clocks, and a Gen1 PIPE-16 " \
        "clock is %d Symbol Times)" \
        % (measured, SKP_MIN_SYMBOL_TIMES, SKP_MAX_SYMBOL_TIMES, syms_per_word(dut))


# ------------------------------------------------------------------ O-7b

@cocotb.test()
async def skp_does_not_interrupt_an_ordered_set(dut):
    """O-7b: Base 2.1 sec 4.2.7.1 p.261 -- "Scheduled SKP Ordered Sets shall be
    transmitted if a packet or Ordered Set is not already in progress, otherwise
    they are accumulated and then inserted consecutively at the next packet or
    Ordered Set boundary."

    A SKP that lands inside a TS Ordered Set destroys it: the receiver's
    ordered-set handler is matching 16 consecutive Symbols from a COM, and four
    foreign Symbols in the middle make the TS unrecognisable.  So with link_up_i
    raised AND a TS1 stream running, no COM,SKP,SKP,SKP group may appear between
    a TS COM and its Symbol 15.

    SS WHY THIS PASSES CHANGED IN FA-5c, AND THE OLD REASON WAS STRONGER.
    It used to pass because os_generator tested skp_cnt only in ST_IDLE and so
    STRUCTURALLY could not pre-empt ST_SEND -- the row could not fail.  SS54 #9(A)
    gave the second half of the spec sentence an implementation: a pending SKP
    now wins the ST_SEND set-complete boundary.  So this row now passes because
    the diversion is placed AT a boundary, where tlast is already asserted and
    the set in flight completes -- a real property that a wrong placement would
    break.  Mutant ME (divert without the boundary test) reddens it, which is
    what makes it load-bearing rather than decorative.

    SS THE WINDOW WAS WIDENED, AND A MUTANT IS WHY.  It used to collect 700
    clocks = 1400 Symbol Times, and under continuous training the first SKP
    arrives at Symbol Time 1422 -- 22 Symbol Times late.  So this row saw ZERO
    SKPs and its "no SKP landed inside a set" verdict was VACUOUSLY true: there
    was nothing to land.  That was invisible while SS54 #9(A) was open, because
    nothing was emitted under load at all and the emptiness looked like the
    defect rather than like a blind window.

    It was caught by mutant ME -- divert to ST_SKP WITHOUT the set-complete
    boundary test, i.e. the rejected one-line version of #9(A) that truncates a
    TS mid-set.  ME SURVIVED the original window: 4 of 4 rows green, because the
    only row that could see the violation never reached a SKP.  The budget is now
    taken from the same spec bound O-7c uses, so the window provably contains
    SKPs and the assertion has something to be true ABOUT.
    """
    await start_clocks(dut)
    await reset(dut)
    dut.link_up_i.value = 1
    dut.ordered_set_i.value = pack_tsos(link_num=0x05, lane_num=0x00)
    dut.gen_os_ctrl_i.value = G_VALID | G_GEN_TS1 | G_SET_LANE

    stream = await collect(dut, 2 * SKP_MAX_SYMBOL_TIMES // 2 + 128)
    skps = skp_starts(stream)

    # Every TS Ordered Set is a K-coded COM followed by 15 symbols that are not
    # a SKP group.  Walk the stream and check no SKP start falls strictly inside
    # one of those 16-Symbol windows.
    ts_starts = []
    for i in range(len(stream) - 16):
        s, k, t = stream[i]
        if s == COM and k == 1 and stream[i + 1][0] != SKP:
            ts_starts.append(t)

    violations = [(ts, sk) for ts in ts_starts for sk in skps if ts < sk < ts + 16]
    dut._log.info("TS Ordered Sets seen: %d   SKP Ordered Sets seen: %d"
                  % (len(ts_starts), len(skps)))
    dut._log.info("SKP groups landing inside a TS Ordered Set: %d" % len(violations))
    assert skps, (
        "no SKP Ordered Set appeared in %d Symbol Times, so this row's 'none "
        "landed inside a set' verdict would be VACUOUS.  That is the state "
        "mutant ME survived in; the window is sized from sec 4.2.7.1 p.261's "
        "bound precisely so it cannot recur." % len(stream))
    assert ts_starts, "no TS Ordered Set on the wire -- stimulus did not take"
    assert not violations, \
        "sec 4.2.7.1 p.261: a SKP Ordered Set was inserted inside an Ordered Set " \
        "already in progress at Symbol Times %s" % violations[:4]


# ----------------------------------------------------------------- O-7c

@cocotb.test()
async def skp_is_emitted_between_ordered_sets_during_training(dut):
    """O-7c: the OTHER half of sec 4.2.7.1 p.261's sentence.

    "Scheduled SKP Ordered Sets shall be transmitted if a packet or Ordered Set
    is not already in progress, otherwise they are accumulated and then inserted
    consecutively at the next packet or Ordered Set boundary."

    O-7b asserts the first clause -- a SKP must not land INSIDE a set.  This row
    asserts the second: once one is due, it must actually be INSERTED at the next
    boundary.  A design that never emits one satisfies O-7b vacuously, which is
    exactly the state this row was written to end.

    PREDICTED DIVERGENCE (tracker SS54 #9(A)).  os_generator tests skp_cnt only in
    ST_IDLE, and ST_SEND's streaming lock keeps the FSM out of ST_IDLE while the
    LTSSM holds a steady command -- so during continuous training the state that
    could schedule a SKP is never visited.  Measured on the FA-5 baseline: 85
    Ordered Sets and ZERO SKPs over 1400 Symbol Times, more than the 1180 floor,
    so at least one was due.  The starvation is not a tendency, it is total.

    The contrast that makes it a defect rather than a slow timer: with NO ordered
    set requested, the same DUT emits SKPs on a perfect 1358-Symbol-Time cadence
    (row O-8).  The timer and ST_SKP work; they are unreachable under load.

    ⚠️ The budget is DERIVED from the spec bound, not invented -- the same
    discipline O-7's second widening had to learn.  Two spec-maximum intervals of
    Symbol Times leaves room for at least two SKPs, so the row does not sit one
    clock from its own edge.

    ONE divergent assertion (SS22.66): a SKP Ordered Set is emitted during
    continuous training.  Whether it lands legally is O-7b's claim, not this one.

    SS FLIPPED by SS54 #9(A): a pending SKP now wins the ST_SEND set-complete
    boundary.  Measured after the fix -- 205 Ordered Sets and 2 SKP Ordered Sets
    at Symbol Times 1422 and 2786, an interval of 1364, inside sec 4.2.7.1
    p.261's 1180..1538 window.  Mutant MD (revert the diversion) re-reddens it.
    """
    await start_clocks(dut)
    await reset(dut)
    dut.link_up_i.value = 1
    dut.ordered_set_i.value = pack_tsos(link_num=0x05, lane_num=0x00)
    dut.gen_os_ctrl_i.value = G_VALID | G_GEN_TS1 | G_SET_LANE

    cycles = 2 * SKP_MAX_SYMBOL_TIMES // 2 + 128
    stream = await collect(dut, cycles)
    skps = skp_starts(stream)
    ts_starts = [t for i, (s, k, t) in enumerate(stream[:len(stream) - 16])
                 if s == COM and k == 1 and stream[i + 1][0] != SKP]

    dut._log.info("O-7c over %d Symbol Times: TS Ordered Sets=%d  SKP Ordered Sets=%d"
                  % (len(stream), len(ts_starts), len(skps)))
    dut._log.info("O-7c SKP Ordered Sets at Symbol Times: %s" % skps[:8])
    assert ts_starts, "no TS Ordered Set on the wire -- stimulus did not take"
    assert skps, (
        "no SKP Ordered Set was emitted in %d Symbol Times of continuous TS1, "
        "though sec 4.2.7.1 p.261's window closes at %d.  Scheduled SKPs must be "
        "'accumulated and then inserted consecutively at the next packet or "
        "Ordered Set boundary'; %d Ordered Set boundaries went by."
        % (len(stream), SKP_MAX_SYMBOL_TIMES, len(ts_starts)))


# ---------------------------------------- O-7b with packets (LR2 group C)
STP, SDP, END = 0xFB, 0x5C, 0xFD
G_GEN_IDLE = 1 << 7
P1_BEATS = 64          # 256 bytes: STP + 254 + END, 128 clocks on a 16-bit PIPE
P2_BEATS = 6
MARGIN = 48            # clocks from the due time to P1's END: twice the idle-only path latency (23 measured)



async def reset_idle(dut):
    """reset(), then L0's request: link_up_i raised and Logical Idle requested
    continuously (test_7j2_idle.py's request_idle())."""
    await reset(dut)
    dut.link_up_i.value = 1
    dut.gen_os_ctrl_i.value = G_VALID | G_GEN_IDLE


class Wire:
    """Valid-gated wire events per clock (post-edge): SKP Ordered Set starts, STP/SDP, END."""
    def __init__(self, dut):
        self.dut, self.c, self.skp, self.stp, self.end = dut, 0, [], [], []
        self.task = cocotb.start_soon(self._mon())

    async def _mon(self):
        dut = self.dut
        while True:
            await RisingEdge(dut.pipe_tx_usr_clk_i)
            await Timer(1, units="ps")
            if int(dut.pipe_data_valid_o.value) & 1:
                w = int(dut.pipe_data_o.value) & 0xFFFF
                k = int(dut.pipe_data_k_o.value) & 0x3
                b0, b1 = w & 0xFF, (w >> 8) & 0xFF
                if (k & 1) and b0 == COM and (k & 2) and b1 == SKP:
                    self.skp.append(self.c)
                for i, b in ((0, b0), (1, b1)):
                    if (k >> i) & 1 and b in (STP, SDP):
                        self.stp.append(self.c)
                    if (k >> i) & 1 and b == END:
                        self.end.append(self.c)
            self.c += 1


async def send(dut, beats, tag):
    """Offer one TLP of `beats` beats on s_dllp_axis; returns when its last beat is accepted."""
    for b in range(beats):
        last = b == beats - 1
        dut.s_dllp_axis_tdata.value = (0x11223344 * (tag + 1) + b) & 0xFFFFFFFF
        dut.s_dllp_axis_tkeep.value = 0x3 if last else 0xF
        dut.s_dllp_axis_tuser.value = 0
        dut.s_dllp_axis_tlast.value = 1 if last else 0
        dut.s_dllp_axis_tvalid.value = 1
        while True:
            await RisingEdge(dut.pipe_tx_usr_clk_i)
            if int(dut.s_dllp_axis_tready.value) & 1:
                break
    dut.s_dllp_axis_tvalid.value = 0
    dut.s_dllp_axis_tlast.value = 0


@cocotb.test()
async def skp_due_during_a_packet_goes_before_the_next_packet(dut):
    """O-7b, Base 2.1 sec 4.2.7.1 p.261: a SKP that falls due while packet P1
    is on the wire goes at P1's END, the next packet boundary -- before packet
    P2, which was waiting there. Logical Idle is requested throughout (L0).

    "Due" is defined on the wire from the row's own control limb: with Logical
    Idle and no packet, consecutive SKP Ordered Sets leave every I clocks (I
    measured, not assumed). In the main limb a 64-beat P1 is placed so that the
    next SKP falls due (previous SKP + I) while P1 is on the wire, at least
    MARGIN clocks before P1's END, and a 6-beat P2 is offered while P1 is still
    being accepted, so P2 is waiting when P1 ends.
    Red until the SKP deferral's fix: the Ordered Set path held 8 beats of
    Logical Idle ahead of the SKP and did not move during P1, so P2 went first
    (the SKP due at 2,756 went at 2,896, after P2's STP at 2,868).

    Asserts: the SKP precedes P2. Non-vacuity (§22.82): the control saw a
    stable I; P1 straddles the due time with MARGIN to spare; P2 was offered
    before P1's END; a SKP and P2 both reached the wire after P1. Wire events
    are valid-gated."""
    row = 'skp_due_during_a_packet_goes_before_the_next_packet'
    await start_clocks(dut)
    # ---- control: the idle-only SKP interval I and its phase
    await reset_idle(dut)
    w = Wire(dut)
    while len(w.skp) < 3:
        await RisingEdge(dut.pipe_tx_usr_clk_i)
    gaps = [b - a for a, b in zip(w.skp, w.skp[1:])]
    assert len(set(gaps)) == 1, f"control: idle-only SKP interval not stable: {gaps}"
    I = gaps[0]
    dut._log.info(f"DEFER control: idle-only SKP interval {I} clocks ({2 * I} Symbol Times), SKPs at {w.skp}")
    # ---- main: P1 straddles the next due time, P2 waits at P1's END
    prev = w.skp[-1]
    due = prev + I
    offer = due - 40
    while w.c < offer:
        await RisingEdge(dut.pipe_tx_usr_clk_i)
    await send(dut, P1_BEATS, 1)
    p2_offered_at = w.c
    cocotb.start_soon(send(dut, P2_BEATS, 2))
    while w.c < due + 4 * I // 3:
        await RisingEdge(dut.pipe_tx_usr_clk_i)
    w.task.kill()
    p1_stp = next(x for x in w.stp if x >= offer)
    p1_end = next(e for e in w.end if e > p1_stp)
    p2_stp = next((x for x in w.stp if x > p1_end), None)
    skp_after = next((x for x in w.skp if x > prev), None)
    dut._log.info(f"DEFER main: prev SKP {prev}, due {due}, P1 STP {p1_stp} END {p1_end}, P2 offered by "
                  f"{p2_offered_at} STP {p2_stp}, next SKP {skp_after}; SKP-SKP "
                  f"{None if skp_after is None else skp_after - prev} clocks")
    assert p1_stp < due and p1_end - due >= MARGIN, (
        f"premise: P1 (STP {p1_stp}, END {p1_end}) does not straddle the due time {due} with {MARGIN} clocks "
        f"to spare")
    assert p2_offered_at <= p1_end, f"premise: P2 was offered at {p2_offered_at}, after P1's END {p1_end}"
    assert p2_stp is not None and skp_after is not None, (
        f"premise: P2 STP {p2_stp} / next SKP {skp_after} not seen")
    dut._log.info(f"MEASURE|{row}|SKP due {due}; P1 on the wire {p1_stp}-{p1_end}; P2 STP {p2_stp}; "
                  f"next SKP {skp_after}")
    assert skp_after < p2_stp, (
        f"the SKP due at {due} (P1 on the wire {p1_stp}-{p1_end}) went at {skp_after}, after P2 (STP {p2_stp}), "
        f"which was waiting at P1's END: Base 2.1 sec 4.2.7.1 p.261 sends it at P1's END, the next packet boundary")
