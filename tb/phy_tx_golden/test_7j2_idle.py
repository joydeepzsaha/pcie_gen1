"""§63 #7j-2 -- ACCEPTANCE rows for the four constraints, at the `phy_transmit`
seam.  Base 2.1 §4.2.2 p.195 and §4.2.7.1 p.261.

#7j Phase 1 established that the Transmitter already emits byte-exact scrambled
Logical Idle whenever `gen_idle` is asserted, and that the defect is one state
wide: nothing requests it during L0.  #7j-2's shape is to request it there.

⚠️ THESE ROWS DRIVE THE REQUEST FROM THE BENCH, because `phy_transmit` does not
contain the LTSSM.  What they measure is therefore NOT "does ST_L0 ask for
idle" -- that is `test_ltssm_l0.py`'s question -- but the one the request
raises downstream and that no existing row covers: **with Logical Idle
requested continuously, does the link still work?**

That is where #7j-2's real risk lives.  Phase 1 measured it: under a continuous
request `lane_management` never leaves ST_LANE_MNGT_TX_PHY (0 visits to ST_IDLE
in 1600 cycles), its DLLP `tready` is high for **0** cycles while a packet
waits **1468**, and no STP and no END ever reach the wire.  The packet path is
starved outright.  Rows C1-C3 are red on that, and they are red on the
UNCHANGED tree the moment the request is made.

⚠️ §22.81 -- every negative assertion here pairs with a positive row through
the same path: C6 offers the identical packet with no idle requested and
watches it leave.  Without C6, C1's silence could not be told from a bench that
never presented a packet properly.

Detector: K codes are NOT scrambled (§4.2.3 p.199), so framing Symbols are
visible at `pipe_data_o` without descrambling.  STP = K27.7 = 0xFB opens a
packet, END = K29.7 = 0xFD closes it, COM = K28.5 = 0xBC and SKP = K28.0 = 0x1C
are the Ordered-Set Symbols, and Logical Idle carries K = 0 on every Symbol.

⚠️ Every wire claim below is VALID-GATED.  With valid low the bus HOLDS its last
word, so an ungated census counts one frozen Symbol once per cycle -- Phase 1's
first pass reported 631 COMs where the DUT had sent one.  The ungated window is
what the wire CARRIES; the valid-gated one is what was SENT.
"""
import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ClockCycles

STP, SDP, END, COM, SKP = 0xFB, 0x5C, 0xFD, 0xBC, 0x1C

G_VALID    = 1 << 0
G_GEN_TS1  = 1 << 1
G_GEN_IDLE = 1 << 7

DLLP_W0 = 0xEFBEADDE
DLLP_W1 = 0x00003412

# An idle block is 16 Symbols = 4 words of 32 bits, and at gen1 the PIPE is 16
# bits wide so lane_management drains one word every 2 cycles: 8 cycles per
# block.  Add the two AXIS register stages between os_generator and
# lane_management and one more block of slack, and the bound is 32 cycles.
# ⚠️ This is a DESIGN-derived bound, stated before the fix exists; the measured
# value is reported by the row whether it passes or fails, so the bound can be
# tightened from evidence rather than from taste.
START_LATENCY_BOUND = 32

# Longer than SkpIntervalCounts (0x2A6 = 678) by a margin, so a window that
# sees no SKP has genuinely starved rather than merely been short.
SKP_WINDOW = 2000


def symbols(word, k):
    return [((word >> (8 * i)) & 0xFF, (k >> i) & 1) for i in range(4)]


def k_syms(word, k):
    return [b for b, isk in symbols(word, k) if isk]


async def start_clocks(dut):
    cocotb.start_soon(Clock(dut.clk_i, 10.0, units="ns").start())
    cocotb.start_soon(Clock(dut.pipe_rx_usr_clk_i, 10.0, units="ns").start())
    cocotb.start_soon(Clock(dut.pipe_tx_usr_clk_i, 10.0, units="ns").start())


async def reset(dut, link_up=1):
    dut.rst_i.value = 1
    dut.en_i.value = 0
    dut.link_up_i.value = 0
    dut.num_active_lanes_i.value = 1
    dut.send_ordered_set_i.value = 0
    dut.ordered_set_i.value = 0
    dut.gen_os_ctrl_i.value = 0
    dut.curr_data_rate_i.value = 1
    dut.s_dllp_axis_tdata.value = 0
    dut.s_dllp_axis_tkeep.value = 0
    dut.s_dllp_axis_tvalid.value = 0
    dut.s_dllp_axis_tlast.value = 0
    dut.s_dllp_axis_tuser.value = 0
    await ClockCycles(dut.pipe_tx_usr_clk_i, 8)
    dut.rst_i.value = 0
    await ClockCycles(dut.pipe_tx_usr_clk_i, 4)
    dut.en_i.value = 1
    dut.link_up_i.value = link_up


def request_idle(dut, strobe=0):
    """What a fixed ST_L0 asks for: Logical Idle, continuously.

    `ordered_set_i` is the all-zero `gen_zeros()` template; `os_generator`
    clears `special_k` for `gen_idle` (os_generator.sv:285) so all 16 Symbols
    go out as DATA 00h and the scrambler turns them into Base 2.1 Table B p.700.
    """
    dut.ordered_set_i.value = 0
    dut.gen_os_ctrl_i.value = G_VALID | G_GEN_IDLE
    dut.send_ordered_set_i.value = strobe


async def offer_packet(dut):
    dut.s_dllp_axis_tdata.value = DLLP_W0
    dut.s_dllp_axis_tkeep.value = 0xF
    dut.s_dllp_axis_tuser.value = 0
    dut.s_dllp_axis_tlast.value = 0
    dut.s_dllp_axis_tvalid.value = 1


async def capture(dut, cycles, offer_at=None):
    """Sample continuously from BEFORE the event (§22.89) and return the trace.

    ⚠️ `offer_at` presents the packet INSIDE the window, in steady state.  An
    earlier version of this bench offered it before the window opened, at the
    same instant as the idle request: `lane_management` was still in ST_IDLE
    with an EMPTY Ordered-Set FIFO, so the packet won a race that steady-state
    L0 never offers and the row reported an acceptance that said nothing.  A
    window that does not contain the event it was opened for is the #7h lesson
    in its cheapest form.
    """
    trace, beat = [], 0
    for c in range(cycles):
        if offer_at is not None and c == offer_at:
            await offer_packet(dut)
        await RisingEdge(dut.pipe_tx_usr_clk_i)
        rdy = int(dut.s_dllp_axis_tready.value) & 0x1
        tvl = int(dut.s_dllp_axis_tvalid.value) & 0x1
        trace.append(dict(
            c=c, w=int(dut.pipe_data_o.value) & 0xFFFFFFFF,
            k=int(dut.pipe_data_k_o.value) & 0xF,
            v=int(dut.pipe_data_valid_o.value) & 0x1,
            rdy=rdy, tvl=tvl))
        if rdy and tvl:
            beat += 1
            if beat == 1:
                dut.s_dllp_axis_tdata.value = DLLP_W1
                dut.s_dllp_axis_tkeep.value = 0x3
                dut.s_dllp_axis_tlast.value = 1
            else:
                dut.s_dllp_axis_tvalid.value = 0
                dut.s_dllp_axis_tlast.value = 0
    return trace


def sent(trace):
    """Valid-gated view: what was SENT, not what the bus carried."""
    return [e for e in trace if e['v']]


def find(trace, sym):
    return [e['c'] for e in sent(trace) if sym in k_syms(e['w'], e['k'])]


def report(dut, tag, trace):
    s = sent(trace)
    dut._log.info(
        "7J2[%s] cycles=%d valid=%d/%d STP=%s END=%s COM=%d SKP=%d"
        % (tag, len(trace), len(s), len(trace), find(trace, STP)[:3],
           find(trace, END)[:3], len(find(trace, COM)), len(find(trace, SKP))))


# ======================================================================= C1
@cocotb.test()
async def test_c1_idle_yields_to_a_waiting_packet(dut):
    """CONSTRAINT 1 (packet limb) -- Logical Idle must yield to a waiting packet.

    RED BEFORE FIX: measured in Phase 1.  Under a continuous idle request the
    packet is accepted at `phy_transmit`'s own AXIS port -- `tready` asserts
    immediately, which is why a port-level row alone is not enough -- and then
    starves one FIFO downstream: `lane_management` never leaves
    ST_LANE_MNGT_TX_PHY, its DLLP `tready` is high for 0 cycles, and no STP and
    no END reach the wire in 1500 cycles.

    Base 2.1 §4.2.2 p.195 defines Logical Idle as what is transmitted "when no
    packet information or special Ordered Sets are being transmitted".  Idle is
    what the link does INSTEAD of a packet, so a packet must always be able to
    displace it.
    """
    await start_clocks(dut)
    await reset(dut)
    request_idle(dut)
    trace = await capture(dut, 2000, offer_at=500)
    report(dut, "C1", trace)
    stp, end = find(trace, STP), find(trace, END)
    assert stp and end, (
        f"the packet never reached the wire: STP={stp} END={end} in 1500 "
        f"cycles after it was offered, while Logical Idle was requested "
        f"continuously.  Logical Idle does not yield to a waiting packet.")


# ======================================================================= C2
@cocotb.test()
async def test_c2_no_idle_word_between_stp_and_end(dut):
    """CONSTRAINT 2 -- no idle word may appear between a packet's STP and END.

    ⚠️ NON-VACUITY IS ASSERTED FIRST AND SEPARATELY (§22.82).  A contiguity
    claim over a span that does not exist is vacuously true, and on the
    unchanged tree the span does NOT exist -- so this row would pass for the
    worst possible reason.  The first assertion is that the span is there; only
    then is contiguity claimed.

    "Contiguous" is stated as: every cycle from STP to END inclusive carries
    valid, and every Symbol in that span belongs to the packet -- no Ordered
    Set and no idle block is interleaved.  Interleaving is detected by a COM
    inside the span, which is the only way a foreign block can start.
    """
    await start_clocks(dut)
    await reset(dut)
    request_idle(dut)
    trace = await capture(dut, 2000, offer_at=500)
    report(dut, "C2", trace)
    stp, end = find(trace, STP), find(trace, END)
    assert stp and end, (
        f"NON-VACUITY: no STP/END span exists to test (STP={stp} END={end}).  "
        f"This row asserts nothing about contiguity until a packet is framed "
        f"on the wire at all.")
    a, b = stp[0], end[0]
    assert b > a, f"END at {b} precedes STP at {a}"
    span = [e for e in trace if a <= e['c'] <= b]
    low = [e['c'] for e in span if not e['v']]
    assert not low, (
        f"valid dropped inside the packet at cycles {low[:8]} "
        f"(STP@{a} END@{b}) -- Symbol Times inside a packet carried no Symbol")
    foreign = [e['c'] for e in span if COM in k_syms(e['w'], e['k'])]
    assert not foreign, (
        f"an Ordered Set was interleaved inside the packet at cycles "
        f"{foreign[:8]} (STP@{a} END@{b})")


# ======================================================================= C3
@cocotb.test()
async def test_c3_packet_start_latency_is_bounded(dut):
    """CONSTRAINT 3 -- the wait for a packet to start is a small constant.

    RED BEFORE FIX: the latency is unbounded today, which is the same defect C1
    states and a different claim about it -- C1 says the packet arrives, C3
    says it arrives SOON.  A fix that drained the idle FIFO before yielding
    would satisfy C1 and fail this.

    The bound is derived from the design, not from taste: an idle block is 16
    Symbols = 4 words, `lane_management` drains one word every 2 cycles at
    gen1, so a block is 8 cycles; two AXIS register stages and one block of
    slack give 32.  The measured value is logged either way.
    """
    await start_clocks(dut)
    await reset(dut)
    request_idle(dut)
    trace = await capture(dut, 2000, offer_at=500)
    report(dut, "C3", trace)
    stp = find(trace, STP)
    offered = next((e['c'] for e in trace if e['tvl']), None)
    assert offered is not None, "NON-VACUITY: the bench never offered a packet"
    assert stp, (
        f"NON-VACUITY: the packet never started on the wire at all (offered@"
        f"{offered}), so its start latency is not a number -- it is unbounded.")
    lat = stp[0] - offered
    dut._log.info(f"7J2[C3] offered@{offered} STP@{stp[0]} latency={lat} cycles")
    assert lat <= START_LATENCY_BOUND, (
        f"packet start latency {lat} cycles exceeds the bound "
        f"{START_LATENCY_BOUND}; idle yields, but not promptly")


# ======================================================================= C4
@cocotb.test()
async def test_c4_skp_continues_under_continuous_idle(dut):
    """CONSTRAINT 1 (SKP limb) -- Base 2.1 §4.2.2 p.195: "During transmission
    of the idle data, the SKP Ordered Set must continue to be transmitted as
    specified in Section 4.2.7."

    ⚠️ This row is GREEN BEFORE AND AFTER, and says so rather than pretending
    to be a red-before-fix row.  It is a REGRESSION GUARD, and it guards a real
    hazard that Phase 1 measured: `os_generator`'s ST_IDLE resets `skp_cnt`
    whenever `gen_os_ctrl_i.valid` is high (os_generator.sv:203).  If ST_L0
    keeps its unconditional `transmit_ordered_set` strobe, the streaming lock
    breaks at every Ordered-Set boundary, the FSM returns to ST_IDLE every ~8
    cycles, and the SKP timer is reset before it can ever reach 678 -- measured
    ZERO SKP Ordered Sets in 2000 cycles with the strobe, two without it.

    So the row's power comes from the mutant "ST_L0 keeps the strobe", not from
    the unchanged tree, and that mutant is a shape somebody would plausibly
    write.  The strobe is driven LOW here because that is what the fix does.
    """
    await start_clocks(dut)
    await reset(dut)
    request_idle(dut, strobe=0)
    trace = await capture(dut, SKP_WINDOW)
    report(dut, "C4", trace)
    com, skp = find(trace, COM), find(trace, SKP)
    assert com and skp, (
        f"no SKP Ordered Set in {SKP_WINDOW} cycles of continuous Logical Idle "
        f"(COM={len(com)} SKP={len(skp)}); the SKP schedule is starved, and "
        f"Base 2.1 §4.2.2 p.195 requires it to continue during idle data")


# ======================================================================= C5
@cocotb.test()
async def test_c5_valid_never_drops_under_continuous_idle(dut):
    """ACCEPTANCE (a) at this seam -- in Logical Idle every Symbol Time carries
    a Symbol (§4.2.2 p.195).

    ⚠️ Also green before and after, and it is the FENCE for the whole rung: the
    yield C1 asks for must not be bought by routing the packet through
    `lane_management`'s ST_IDLE, whose `data_valid_c` default is '0
    (lane_management.sv:297).  A hand-off through ST_IDLE would drop valid for
    one cycle at every packet boundary and this row would redden.  That is
    exactly the mutant it guards against.
    """
    await start_clocks(dut)
    await reset(dut)
    request_idle(dut)
    trace = await capture(dut, 1000, offer_at=500)
    report(dut, "C5", trace)
    first = next((e['c'] for e in trace if e['v']), None)
    assert first is not None, "NON-VACUITY: valid never rose at all"
    after = [e for e in trace if e['c'] >= first]
    low = [e['c'] for e in after if not e['v']]
    assert not low, (
        f"valid dropped at cycles {low[:12]} (of {len(low)}) after first rising "
        f"at {first}, while Logical Idle was requested continuously -- those "
        f"Symbol Times carried no Symbol")


# ======================================================================= C6
@cocotb.test()
async def test_c6_control_packet_leaves_with_no_idle_requested(dut):
    """THE POSITIVE CONTROL (§22.81).  Same packet, same port, same window, no
    idle requested.  Without it, C1-C3's silence could not be told from a bench
    that never presented a packet correctly in the first place.

    ⚠️ It is also the row that catches a wrong DETECTOR.  Phase 1's first pass
    looked for SDP and found none anywhere, including here -- and an END with
    no start is impossible, which is what showed the start Symbol was STP.
    """
    await start_clocks(dut)
    await reset(dut)
    dut.gen_os_ctrl_i.value = 0
    dut.send_ordered_set_i.value = 0
    trace = await capture(dut, 2000, offer_at=500)
    report(dut, "C6", trace)
    stp, end = find(trace, STP), find(trace, END)
    assert stp and end, (
        f"CONTROL FAILED: with no idle requested the packet still did not "
        f"reach the wire (STP={stp} END={end}).  Nothing this bench says about "
        f"the idle case means anything until this row passes.")
