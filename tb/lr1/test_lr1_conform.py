"""test_lr1_conform -- pcie_rc_top against the LR1 far end: conforming and legal-variation rows

Author: Kourosh Ghahramani
Silicon Systems Research Lab, University of Washington

Purpose
    LR1-a steps 2 and 3 (BRIEF_LR1 §5): the Root Complex trains to L0,
    completes flow-control initialization and enumerates a device it has never
    been built against -- the LR1 far end, an upstream port written from Base
    2.1 and independent of the repository's RTL (D4) -- with the far end
    checking everything the RC transmits (D6). Pins only (D3); SIM_FAST_LINK =
    0 (RULINGS_LR1 D).

    conform_default   every knob at its default (lr1_farend.Knobs).
    vary_*            one knob changed: a behaviour Base 2.1 allows and our
                      Endpoint does not do (D7; PHASE0_LR1 §5.3; RULINGS_LR1
                      E, G, I; RULINGS_A 6), each row witnessing that its
                      stimulus fired.

A green row asserts: the RC reaches L0 (link_up_o) and FC init
(fc_init_done_o); enumeration ends with enum_done_o, no error, device 0 present
with the far end's IDs and its BAR0; the far end reached L0 and DL_Active; no
far-end check flags anything outside KNOWN; every check in NONVACUOUS examined
something.

A row whose knob reproduces a registered defect is an expect_fail row that
fails only at its one pinned assertion (§22.93): the run and its non-vacuity
checks are inside a guard that logs PINNED_RED|<row>|NOT_REACHED and returns,
and PINNED_RED|<row>|REACHED is logged just before the pinned assertion.

KNOWN holds the RC non-conformances that are registered and expected here:
    SKP_GAP_TRAINING  no SKP Ordered Set before LinkUp (BRIEF_LR1 §0, P2) --
                      registered; its pinned red row is skp_before_linkup.
A flag in KNOWN is reported and counted, not failed: the row asserts it is the
only kind present, so a new kind fails the row.

References
    PCIe Base Spec r2.1, §4.2.6, §4.2.7, §3.3.1, §3.5, §2.6.1, §7.5
"""
import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles

from lr1_farend import FarEnd, Knobs
import lr1_cfg as CFG

CLK_NS = 8
MAX_CYCLES = 80000           # 640 us; the default row needs about a third of it
TAIL_CYCLES = 6250           # 50 us after enum_done_o: an UpdateFC window and more

KNOWN = {'SKP_GAP_TRAINING'}

# Checks that must have examined at least one item in a conforming run (§22.82).
NONVACUOUS = ('TS_FIELDS', 'POLL_TS1_COUNT', 'TS2_AFTER_RX', 'PKT_BEFORE_L0', 'SKP_FORMAT', 'SKP_GAP',
              'DLLP_CRC', 'DLLP_FIELDS', 'LCRC', 'SEQ', 'TLP_FORMAT', 'ACK_SEQ', 'ACK_LATENCY',
              'FC_INIT_ORDER', 'FC_INIT_PERIOD', 'FC_UPDATE_PERIOD', 'FC_OVERFLOW')


class Bench:
    def __init__(self, dut):
        self.dut = dut
        cocotb.start_soon(Clock(dut.clk_i, CLK_NS, units='ns').start())

    async def reset(self, fe):
        d = self.dut
        d.rst_i.value = 1
        d.en_i.value = 0
        d.transmit_enable_i.value = 0
        d.tx_elec_idle.value = 0
        d.phy_ready_en.value = 0
        d.scan_start_i.value = 0
        d.scan_bus_i.value = 0
        d.bar_enable_i.value = 0
        d.bridge_enable_i.value = 0
        d.s_axis_rq_tdata.value = 0
        d.s_axis_rq_tkeep.value = 0
        d.s_axis_rq_tvalid.value = 0
        d.s_axis_rq_tlast.value = 0
        d.s_axis_rq_tuser.value = 0
        d.m_axis_rc_tready.value = 1
        fe.drive_reset_values()
        await ClockCycles(d.clk_i, 10)
        d.rst_i.value = 0
        await ClockCycles(d.clk_i, 2)
        # the host: start one scan with the BAR stage (pcie_rc_top latches a
        # start that arrives before FC init, start_pending_r)
        d.bar_enable_i.value = 1
        d.scan_start_i.value = 1
        await ClockCycles(d.clk_i, 1)
        d.scan_start_i.value = 0
        d.en_i.value = 1
        d.transmit_enable_i.value = 1


class Rise:
    """Cycle a level first read 1 (sampled by the far end's clock loop)."""

    def __init__(self):
        self.at = {}

    def sample(self, dut, cycle, names):
        for n in names:
            if n not in self.at and int(getattr(dut, n).value):
                self.at[n] = cycle


async def run_row(dut, knobs, name, until=None, max_cycles=MAX_CYCLES, tail=TAIL_CYCLES, after=None):
    """Reset, run the far end until `until(fe, rise)` (default: enumeration
    ended), then `after(fe, rise)` (a coroutine, optional), then `tail` more
    clocks; returns (fe, chk, rise, kinds)."""
    tb = Bench(dut)
    fe = FarEnd(dut, knobs)
    await tb.reset(fe)
    rise = Rise()
    names = ('link_up_o', 'fc_init_done_o', 'scan_done_o', 'enum_done_o', 'enum_error_o', 'scan_error_o')
    orig = fe.step

    def step():
        orig()
        rise.sample(dut, fe.cycle, names)
    fe.step = step
    if until is None:
        until = lambda f, r: 'enum_done_o' in r.at or 'enum_error_o' in r.at
    await fe.run(max_cycles, until=lambda: until(fe, rise))
    if after is not None:
        await after(fe, rise)
    await fe.run(tail)
    chk = fe.finish()
    kinds = {}
    for f in chk.flags:
        kinds[f['id']] = kinds.get(f['id'], 0) + 1
    dut._log.info('ROW|%s|rc_rise=%s' % (name, sorted(rise.at.items(), key=lambda x: x[1])))
    dut._log.info('ROW|%s|farend_ltssm=%s' % (name, fe.ltssm.history))
    dut._log.info('ROW|%s|farend_dll=%s stats=%s' % (name, fe.dll.history, fe.dll.stats))
    dut._log.info('ROW|%s|flags=%s' % (name, sorted(kinds.items())))
    dut._log.info('ROW|%s|checked=%s' % (name, sorted(chk.n.items())))
    dut._log.info('ROW|%s|notes=%s ltssm_events=%s' % (name, chk.notes, fe.ltssm.events))
    dut._log.info('ROW|%s|measured=%s' % (name, chk.summary()))
    dut._log.info('ROW|%s|farend_sent: com_lanes=%s skp_ns=%s skp_gaps=%s b2b=%d ack_lat=%s ts_counts=%s' % (
        name, fe.com_lanes, sorted(fe.skp_ns), (min(fe.skp_gaps), max(fe.skp_gaps)) if fe.skp_gaps else None,
        fe.b2b, (min(fe.dll.ack_lat), max(fe.dll.ack_lat)) if fe.dll.ack_lat else None, fe.ltssm.ts_counts))
    dut._log.info('ROW|%s|requests=%d cycles=%d' % (name, len(fe.tl.requests), fe.cycle))
    for cyc, t, st in fe.tl.requests:
        dut._log.info('REQ|%s|%d|%s|reg=%s|be=%s|data=%s|status=%s' % (
            name, cyc, t.get('name'), '%03x' % t['reg'] if 'reg' in t else '-', t.get('first_be', '-'),
            t['payload'][:4].hex() if t.get('payload') else '-', st))
    return fe, chk, rise, kinds


def assert_conforming(dut, fe, chk, rise, kinds, bar_size, prefetch):
    for n in ('link_up_o', 'fc_init_done_o', 'enum_done_o'):
        assert n in rise.at, '%s never rose (rises: %s)' % (n, rise.at)
    assert 'enum_error_o' not in rise.at and 'scan_error_o' not in rise.at, 'enumeration error: %s' % rise.at
    assert int(dut.device_present_o.value) == 1, 'device_present_o = 0'
    assert int(dut.vendor_id_o.value) == CFG.VENDOR_ID, 'vendor_id_o %04x' % int(dut.vendor_id_o.value)
    assert int(dut.device_id_o.value) == CFG.DEVICE_ID, 'device_id_o %04x' % int(dut.device_id_o.value)
    assert int(dut.header_type_o.value) == 0x00, 'header_type_o %02x' % int(dut.header_type_o.value)
    valid = int(dut.bar_valid_o.value)
    is64 = int(dut.bar_is_64_o.value)
    pre = int(dut.bar_prefetch_o.value)
    size0 = int(dut.bar_size_o.value) & ((1 << 64) - 1)
    assert valid & 1 and is64 & 1, 'BAR0 valid %x is_64 %x' % (valid, is64)
    assert (pre & 1) == int(prefetch), 'BAR0 prefetch %x' % pre
    assert size0 == bar_size, 'BAR0 size %x, far end %x' % (size0, bar_size)
    assert fe.ltssm.history[-1][1] == 'L0' and 'L0' in [s for _, s in fe.ltssm.history], \
        'far end LTSSM %s' % fe.ltssm.history
    assert fe.dll.state == 'ACTIVE', 'far end DLL %s' % fe.dll.state
    unknown = {k: v for k, v in kinds.items() if k not in KNOWN}
    assert not unknown, 'far-end checks flagged %s; first: %s' % (
        unknown, [f for f in chk.flags if f['id'] in unknown][:3])
    vac = [c for c in NONVACUOUS if not chk.n.get(c)]
    assert not vac, 'checks that examined nothing: %s' % vac


@cocotb.test()
async def conform_default(dut):
    """Row 0: every knob at its default."""
    k = Knobs()
    fe, chk, rise, kinds = await run_row(dut, k, 'conform_default')
    assert_conforming(dut, fe, chk, rise, kinds, k.bar_size, k.bar_prefetchable)


async def green_row(dut, name, **kw):
    k = Knobs(**kw)
    fe, chk, rise, kinds = await run_row(dut, k, name)
    assert_conforming(dut, fe, chk, rise, kinds, k.bar_size, k.bar_prefetchable)
    return fe, chk


def pinned_red(dut, row, state, detail):
    dut._log.info('PINNED_RED|%s|%s|%s' % (row, state, detail))


# ---------------------------------------------------------------- green rows
@cocotb.test()
async def vary_com_lane0(dut):
    """RULINGS_A 6's variation: COM in byte 0 ([7:0])."""
    fe, chk = await green_row(dut, 'vary_com_lane0', com_lane=0)
    assert fe.com_lanes[1] == 0 and fe.com_lanes[0] > 1000, 'COM lanes driven %s' % fe.com_lanes


@cocotb.test()
async def vary_ack_late(dut):
    """V7: the far end Acks 100 clocks after a TLP's END, near Table 3-6's 237 Symbol Times."""
    fe, chk = await green_row(dut, 'vary_ack_late', ack_delay_cycles=100)
    lat = fe.dll.ack_lat
    assert lat and min(lat) >= 100 and max(lat) <= 118, 'far-end Ack latency %s cycles' % (lat,)


@cocotb.test()
async def vary_back_to_back(dut):
    """V9: packets back to back, no idle between them (§4.2.2 p.196)."""
    fe, chk = await green_row(dut, 'vary_back_to_back', pkt_gap=0)
    assert fe.b2b >= 10, 'only %d packets followed another with no idle' % fe.b2b


@cocotb.test()
async def vary_skp_min(dut):
    """V10: SKP Ordered Sets every 1180 Symbol Times (§4.2.7.1 p.261)."""
    fe, chk = await green_row(dut, 'vary_skp_min', skp_interval=1180)
    g = fe.skp_gaps
    assert len(g) >= 10 and min(g) >= 1180 and max(g) <= 1180 + 16, 'far-end SKP gaps %s..%s' % (min(g), max(g))


@cocotb.test()
async def vary_skp_max(dut):
    """V10: SKP Ordered Sets every 1538 Symbol Times."""
    fe, chk = await green_row(dut, 'vary_skp_max', skp_interval=1538)
    g = fe.skp_gaps
    assert len(g) >= 10 and min(g) >= 1538 and max(g) <= 1538 + 16, 'far-end SKP gaps %s..%s' % (min(g), max(g))


@cocotb.test()
async def vary_skp_count(dut):
    """V11: SKP OS of 1-5 SKP at the RC's pins (§4.2.7.2 p.262), as an elastic buffer leaves them."""
    fe, chk = await green_row(dut, 'vary_skp_count', skp_counts=(1, 2, 3, 4, 5))
    assert fe.skp_ns == {1, 2, 3, 4, 5}, 'SKP counts sent %s' % fe.skp_ns
    assert fe.com_lanes[0] > 100 and fe.com_lanes[1] > 100, 'COM lanes %s' % fe.com_lanes


@cocotb.test()
async def vary_ts_extra(dut):
    """V12: more than the minimum training sets and idle (p.220, p.225, p.236, p.237)."""
    fe, chk = await green_row(dut, 'vary_ts_extra', poll_ts1_min=2048, ts2_after_rx=32, idle_after_rx=32,
                              cfg_lw_extra_ts1=8)
    counts = dict(fe.ltssm.ts_counts)
    assert counts.get('POLL_ACTIVE', 0) >= 2048, 'TS1 in Polling.Active %s' % counts


@cocotb.test()
async def vary_credit_max(dut):
    """V4(b): the largest finite advertisement, PH / NPH 7Fh, PD / NPD 7FFh (p.138)."""
    fe, chk = await green_row(dut, 'vary_credit_max',
                              adv_credits={'P': (0x7F, 0x7FF), 'NP': (0x7F, 0x7FF), 'CPL': (0, 0)})
    assert fe.chk.n.get('FC_OVERFLOW', 0) > 0


@cocotb.test()
async def vary_updatefc_extra(dut):
    """V5: UpdateFC on every release, and UpdateFC-Cpl 0/0 every 10 us (p.138, p.143)."""
    fe, chk = await green_row(dut, 'vary_updatefc_extra', updatefc_every_release=True,
                              updatefc_cpl_zero_period=10 * 125)
    assert fe.dll.stats['updatefc_tx'] >= 25, 'UpdateFCs sent %d' % fe.dll.stats['updatefc_tx']


@cocotb.test()
async def vary_redundant_ack(dut):
    """V8: an Ack naming ACKD_SEQ whenever idle 2 us, FFFh before any TLP (p.180)."""
    fe, chk = await green_row(dut, 'vary_redundant_ack', redundant_ack_period=2 * 125)
    st = fe.dll.stats
    assert st.get('redundant_ack', 0) >= 5 and st.get('redundant_ack_fff', 0) >= 1, 'redundant Acks %s' % st


@cocotb.test()
async def vary_nullified(dut):
    """V18: the first three Completions each sent nullified first (EDB, inverted LCRC, p.174)."""
    fe, chk = await green_row(dut, 'vary_nullified', nullify_count=3)
    assert fe.dll.stats.get('nullified', 0) == 3, 'nullified TLPs sent %s' % fe.dll.stats


@cocotb.test()
async def vary_crs(dut):
    """V19: CRS for 200 clocks after the first Configuration Request (p.110), inside the RC's 3 reissues."""
    fe, chk = await green_row(dut, 'vary_crs', crs_cycles=200)
    crs = sum(1 for _, _, st in fe.tl.requests if st == CFG.C.CPL_CRS)
    assert crs >= 1, 'no CRS Completion was sent'


@cocotb.test()
async def vary_bar(dut):
    """V22: BAR0 prefetchable and 128 B, the minimum (p.491; PCI 3.0 §6.2.5.1)."""
    await green_row(dut, 'vary_bar', bar_prefetchable=True, bar_size=128)


# ------------------------------------------------------- pinned red rows
@cocotb.test(expect_fail=True)
async def vary_credit_infinite(dut):
    """V4(c), ruling E: infinite P and NP credit (00h / 000h, p.138). §63 #26:
    tlp2dllp reads an infinite advertisement as zero.

    PINNED: the RC's first Configuration Request reaches phy_txdata within
    4,000 clocks of fc_init_done_o (row 0: 137 clocks)."""
    row = 'vary_credit_infinite'
    try:
        k = Knobs(adv_credits={'P': (0, 0), 'NP': (0, 0), 'CPL': (0, 0)})
        until = lambda f, r: f.chk.rc_seq_next > 0 or (
            'fc_init_done_o' in r.at and f.cycle - r.at['fc_init_done_o'] > 4000)
        fe, chk, rise, kinds = await run_row(dut, k, row, until=until, tail=0)
        assert 'fc_init_done_o' in rise.at, 'FC init never completed: the row is vacuous'
        assert all(v == (0, 0) for v in fe.dll.adv.values()), 'the far end did not advertise infinite credit'
        first = fe.first_rc_tlp
        detail = 'fc_init_done_o at %d, first RC TLP at %s' % (rise.at['fc_init_done_o'], first)
    except Exception as exc:
        pinned_red(dut, row, 'NOT_REACHED', repr(exc))
        return
    pinned_red(dut, row, 'REACHED', detail)
    assert first is not None and first - rise.at['fc_init_done_o'] <= 4000, \
        'no Configuration Request left the RC in 4,000 clocks after FC init (%s)' % detail


@cocotb.test()
async def vary_initfc1_late(dut):
    """V1 (F2, §63 #14): the far end holds its first InitFC1 set 4,200 clocks
    after DL_Init (33.6 us; time in Configuration does not count, p.161), so
    the RC's InitFC1 sets the far end's FI1 first and it sends only InitFC2.
    Received InitFC1 and InitFC2 DLLPs both record FC values and set FI1
    (p.161), so the RC's FI1 comes from InitFC2 alone. Red until F2's fix:
    the RC took FI1 from InitFC1 only and stayed in FC_INIT1.

    Asserts a green row's claims, plus: the stimulus held for the whole run
    (InitFC2 sent, no InitFC1), the RC sent InitFC2, and fc_init_done_o rose
    within 12,500 clocks (100 us, under 3 x 34 us) of the far end's first InitFC2."""
    row = 'vary_initfc1_late'
    k = Knobs(fc_init_start_delay=4200)
    fe, chk, rise, kinds = await run_row(dut, k, row)
    t2 = fe.first_sent.get('InitFC2')
    assert t2 is not None, 'the far end never sent InitFC2'
    assert fe.first_sent.get('InitFC1') is None, 'the far end sent InitFC1: the stimulus did not hold'
    assert_conforming(dut, fe, chk, rise, kinds, k.bar_size, k.bar_prefetchable)
    assert chk.fc_phase == 'InitFC2', 'the RC sent no InitFC2 (its last Init DLLP was %s)' % chk.fc_phase
    done = rise.at['fc_init_done_o']
    dut._log.info('MEASURE|%s|first InitFC2 to the RC at %d, fc_init_done_o at %d' % (row, t2, done))
    assert done - t2 <= 12500, 'fc_init_done_o %d clocks after the far end\'s first InitFC2 (%d)' % (done - t2, t2)


@cocotb.test(expect_fail=True)
async def vary_stop_initfc2(dut):
    """Ruling G (N4, registered with §63 #14): the far end holds its first
    InitFC2 set 4,000 clocks (32 us) and stops sending InitFC2 when its FI2 is
    set, so the RC receives no InitFC2 at all -- only UpdateFCs, and FI2 is set
    on any UpdateFC (p.162).

    PINNED: fc_init_done_o rises within 3,000 clocks of the far end's first
    UpdateFC reaching the RC."""
    row = 'vary_stop_initfc2'
    try:
        k = Knobs(stop_initfc2_on_fi2=True, fc2_first_delay_cycles=4000)
        until = lambda f, r: 'fc_init_done_o' in r.at or (
            f.first_sent.get('UpdateFC') is not None and f.cycle - f.first_sent['UpdateFC'] > 3000)
        fe, chk, rise, kinds = await run_row(dut, k, row, until=until, tail=0)
        assert fe.first_sent.get('InitFC1') is not None, 'the far end sent no InitFC1'
        assert fe.first_sent.get('InitFC2') is None, 'the far end sent InitFC2: the stimulus did not hold'
        tu = fe.first_sent.get('UpdateFC')
        assert tu is not None, 'the far end sent no UpdateFC'
        done = rise.at.get('fc_init_done_o')
        detail = 'first UpdateFC to the RC at %d, fc_init_done_o at %s' % (tu, done)
    except Exception as exc:
        pinned_red(dut, row, 'NOT_REACHED', repr(exc))
        return
    pinned_red(dut, row, 'REACHED', detail)
    assert done is not None and done - tu <= 3000, 'FC init never completed (%s)' % detail


# ------------------------------------------------------------- measurement
async def direct_recovery(fe, place, hold=0, parity=None):
    """Direct Recovery from L0 at a chosen SKP phase. place = 'skp_before_ts4'
    puts the far end's next scheduled SKP OS (1180-1538 interval kept) directly
    before its 4th TS1; 'no_skp' leaves at least 12 TS1 before the next SKP.
    Returns the cycle Recovery was entered."""
    k = fe.k

    def ready():
        if parity is not None and fe.cycle % 4 != parity:
            return False
        left = k.skp_interval - fe.skp_count - len(fe.txq)
        if place == 'skp_before_ts4':
            return fe.ltssm.state == 'L0' and 33 <= left <= 47
        return fe.ltssm.state == 'L0' and left >= 12 * 16 + 32
    await fe.run(20000, until=ready, every=1)
    fe.ltssm.direct('REC_LOCK', fe.cycle, hold=hold)
    return fe.cycle


def rcvrlock_ts_needed(fe, t0):
    """(start times of the far end's TS1 from t0, far-end SKP times from t0,
    time of the RC's first TS2 after t0), all in Symbol Times at the pins."""
    t_rc_ts2 = next((t for t, st in fe.rc_states if t >= t0 and st == 0xE4), None)   # RcvrCfg
    ts = [t for t, k, st in fe.tx_ts if t >= t0 and k == 'TS1'][:24]
    skp = [t for t in fe.skp_times if t >= t0][:2]
    return ts, skp, t_rc_ts2


@cocotb.test(expect_fail=True)
async def vary_recovery_skp_tx(dut):
    """Ruling H, transmit (§63 #29): after enumeration the far end directs
    Recovery (an upstream port may, p.248) and holds Recovery.RcvrLock 4,000
    clocks (it may last 24 ms, p.240), so the RC transmits training sets
    across its own SKP Ordered Sets (scheduled in every non-EI state, p.261).
    Training-set Symbols are never scrambled (p.199), so the set after a SKP
    OS must arrive as a TS1 / TS2.

    PINNED: no training set the RC sends directly after one of its SKP
    Ordered Sets in the held Recovery is malformed."""
    row = 'vary_recovery_skp_tx'
    res = {}
    try:
        async def after(fe, rise):
            t0 = await direct_recovery(fe, 'no_skp', hold=4000)
            await fe.run(60000, until=lambda: fe.ltssm.state == 'L0')
            res['window'] = (2 * t0, 2 * fe.cycle)
        fe, chk, rise, kinds = await run_row(dut, Knobs(), row, after=after, tail=1000)
        assert 'enum_done_o' in rise.at, 'enumeration did not finish before Recovery was directed'
        w = res['window']
        after_skp = [k for t, k in fe.chk.after_skp if w[0] <= t <= w[1]]
        ts_after = [k for k in after_skp if k in ('TS1', 'TS2', 'OS_BAD')]
        assert len(ts_after) >= 2, 'fewer than two RC SKP OS followed by a training set: %s' % after_skp
        assert fe.ltssm.history[-1][1] == 'L0', 'the directed Recovery did not complete: %s' % fe.ltssm.history[-4:]
        bad = ts_after.count('OS_BAD')
        bad_syms = [f['detail'] for f in fe.chk.flags if f['id'] == 'FRAMING'][:1]
        detail = 'RC SKP OS followed by a training set: %d, malformed: %d; first: %s' % (
            len(ts_after), bad, bad_syms)
    except Exception as exc:
        pinned_red(dut, row, 'NOT_REACHED', repr(exc))
        return
    pinned_red(dut, row, 'REACHED', detail)
    assert bad == 0, detail


@cocotb.test(expect_fail=True)
async def vary_recovery_skp_rx(dut):
    """Ruling H, receive (§63 #29): two far-end-directed Recoveries after
    enumeration, differential. The RC leaves Recovery.RcvrLock on 8
    consecutive TS1 (p.240), and a SKP Ordered Set is not an interruption
    (p.262). Control: no far-end SKP OS before or among its first 12 TS1.
    Variant: the far end's next scheduled SKP OS (interval kept inside
    1180-1538) directly before its 4th TS1. The RC's decision is timed from
    its state report on ltssm_debug_state (a pin; RcvrCfg = E4h in
    pcie_ltssm_downstream's state table) -- its first TS2 on phy_txdata waits
    for its own 16-Symbol TS boundary and would hide a one-TS effect.
    Measured in development at four clock phases each: 34 Symbol Times from
    the far end's 8th TS1 to RcvrCfg without the SKP, 50 with it.

    PINNED: the RC enters RcvrCfg no later after the far end's 8th TS1 with
    the SKP than without it, within 4 Symbol Times."""
    row = 'vary_recovery_skp_rx'
    res = {}
    try:
        async def after(fe, rise):
            for place in ('no_skp', 'skp_before_ts4'):
                c0 = await direct_recovery(fe, place, parity=0)
                await fe.run(60000, until=lambda: fe.ltssm.state == 'L0')
                res[place] = rcvrlock_ts_needed(fe, 2 * c0)
                await fe.run(1000)
        fe, chk, rise, kinds = await run_row(dut, Knobs(), row, after=after, tail=1000)
        assert 'enum_done_o' in rise.at, 'enumeration did not finish before Recovery was directed'
        lag = {}
        for place in ('no_skp', 'skp_before_ts4'):
            ts, skp, t_cfg = res[place]
            assert t_cfg is not None and len(ts) >= 12, '%s: RcvrCfg %s, %d TS1' % (place, t_cfg, len(ts))
            if place == 'no_skp':
                assert not skp or skp[0] > ts[11] or skp[0] + 4 < ts[0], 'control has a SKP OS among its TS1: %s' % skp
            else:
                assert skp and skp[0] + 4 == ts[3], 'the SKP OS is not directly before TS1 #4: %s %s' % (skp, ts[:5])
            lag[place] = t_cfg - (ts[7] + 15)
        detail = 'RcvrCfg after the far end\'s 8th TS1: %d Symbol Times without the SKP, %d with it' % (
            lag['no_skp'], lag['skp_before_ts4'])
    except Exception as exc:
        pinned_red(dut, row, 'NOT_REACHED', repr(exc))
        return
    pinned_red(dut, row, 'REACHED', detail)
    assert lag['skp_before_ts4'] <= lag['no_skp'] + 4, detail


# ------------------------------------------- SKP before LinkUp (LR2 0c)
@cocotb.test(expect_fail=True)
async def skp_before_linkup(dut):
    """BRIEF_LR1 §0 P2, the SKP row of PHASE0_LR1 §6, every knob at its
    default. §4.2.7.1 pp.261-262: a SKP Ordered Set is scheduled every
    1180-1538 Symbol Times in every state but Electrical Idle, training
    included, and does not interrupt consecutive training sets (p.201). The
    RC sends none before LinkUp (KNOWN's SKP_GAP_TRAINING).

    Non-vacuity: the RC sent a TS1, link_up_o rose, and the RC sent a SKP
    Ordered Set after it.

    PINNED (PHASE0_LR1 §6): the largest gap between the RC's first TS1 COM,
    its SKP COMs and its first SKP COM after link_up_o rises is at most
    1538 + 15 Symbol Times (the 15 allow an ordered set in progress). The
    window does not end at LinkUp, which would make the row hinge on the far
    end's latency."""
    row = 'skp_before_linkup'
    limit = CFG.C.SKP_INTERVAL_MAX + 15
    try:
        fe, chk, rise, kinds = await run_row(dut, Knobs(), row)
        assert chk.first_ts1_t is not None, 'the RC sent no TS1'
        assert 'link_up_o' in rise.at, 'link_up_o never rose (rises: %s)' % rise.at
        t_up = 2 * rise.at['link_up_o']
        after = [t for t in chk.rc_skp_t if t > t_up]
        assert after, 'no RC SKP Ordered Set after link_up_o rose (Symbol Time %d)' % t_up
        marks = [chk.first_ts1_t] + [t for t in chk.rc_skp_t if chk.first_ts1_t < t <= after[0]]
        worst = max(b - a for a, b in zip(marks, marks[1:]))
        detail = ('first TS1 COM at %d, link_up_o at %d, SKP COMs up to the first after it %s; largest gap %d '
                  'Symbol Times (limit %d)' % (chk.first_ts1_t, t_up, marks[1:8], worst, limit))
        dut._log.info('MEASURE|%s|%s' % (row, detail))
    except Exception as exc:
        pinned_red(dut, row, 'NOT_REACHED', repr(exc))
        return
    pinned_red(dut, row, 'REACHED', detail)
    assert worst <= limit, 'no SKP Ordered Set while the RC trains: ' + detail
