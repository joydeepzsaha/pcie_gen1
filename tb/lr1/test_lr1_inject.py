"""test_lr1_inject -- pcie_rc_top against the LR1 far end: error injection and provocations

Author: Kourosh Ghahramani
Silicon Systems Research Lab, University of Washington

Purpose
    LR1-b (BRIEF_LR1 §5; RULINGS_LR1A 4: the F1 and F3 provocations, the C3
    burst, the Reserved-bit and newer-revision-partner DLLP rows, and the D8
    injection rows). The far end of LR1-a, with lr1_inject's one-shot
    injections, puts errors and legal-but-unusual traffic on the link toward
    the RC; each row asserts what Base 2.1 requires the RC to do, at the
    pins (D3), with section and page.

    inject_independence  the LR1-b files obey D4 (imports)
    inject_control       every injection off: the row conforms as LR1-a's
                         conform_default does (the injecting far end is
                         transparent)
    inj_*                D8: one error toward the RC
    rsvd_*, newer_*      the Reserved-bit and newer-revision-partner DLLPs
                         (RULINGS_LR1 F)
    prov_*               D9: the provocations of registered defects (prov_c3,
                         which needs a 256-byte payload build, is in
                         test_lr1_c3)

A row that reproduces a registered defect is an expect_fail row that fails
only at its one pinned assertion (§22.93), as in test_lr1_conform: the run
and its non-vacuity checks sit inside a guard that logs
PINNED_RED|<row>|NOT_REACHED, and PINNED_RED|<row>|REACHED is logged just
before the pinned assertion.

References
    PCIe Base Spec r2.1, §3.4.1, §3.5.2.1, §3.5.2.2, §3.5.3.1, §4.2.2, §4.2.4.6
"""
import ast
import os

import cocotb

import lr1_cfg as CFG
from lr1_inject import InjFarEnd, InjKnobs
from test_lr1_conform import Bench, Rise, NONVACUOUS

MAX_CYCLES = 80000
TAIL_CYCLES = 6250
KNOWN = {'SKP_GAP_TRAINING'}
RISES = ('link_up_o', 'fc_init_done_o', 'scan_done_o', 'enum_done_o', 'enum_error_o', 'scan_error_o')
LEVELS_DOWN = ('link_up_o', 'fc_init_done_o')


class Falls:
    """Cycle a level first read 0 after it had read 1."""

    def __init__(self):
        self.at = {}
        self.up = set()

    def sample(self, dut, cycle, names):
        for n in names:
            v = int(getattr(dut, n).value)
            if v:
                self.up.add(n)
            elif n in self.up and n not in self.at:
                self.at[n] = cycle


class Edges:
    """(cycle, value) of each change of some RC outputs, from reset release."""

    def __init__(self, names):
        self.names = names
        self.at = {n: [] for n in names}
        self.last = {}

    def sample(self, dut, cycle):
        for n in self.names:
            v = int(getattr(dut, n).value)
            if self.last.get(n) != v:
                self.last[n] = v
                self.at[n].append((cycle, v))

    def rises(self, n):
        return [c for c, v in self.at[n] if v == 1]


EDGES = ('rc_unexpected_completion_o', 'rx_error_valid_o', 'malformed_o', 'rc_protocol_error_o',
         'tx_fc_blocked_o', 'cpl_timeout_valid_o')


async def run_row(dut, knobs, name, until=None, max_cycles=MAX_CYCLES, tail=TAIL_CYCLES, after=None):
    """Reset, run until `until(fe, rise)` (default: enumeration ended), then
    `after(fe, rise)` (a coroutine, optional), then `tail` more clocks.
    Returns fe with fe.rise, fe.falls, fe.edges, fe.kinds attached."""
    tb = Bench(dut)
    fe = InjFarEnd(dut, knobs)
    await tb.reset(fe)
    rise, falls, edges = Rise(), Falls(), Edges(EDGES)
    orig = fe.step

    def step():
        orig()
        rise.sample(dut, fe.cycle, RISES)
        falls.sample(dut, fe.cycle, LEVELS_DOWN)
        edges.sample(dut, fe.cycle)
    fe.step = step
    fe.rise, fe.falls, fe.edges = rise, falls, edges
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
    fe.kinds = kinds
    log = dut._log.info
    log('ROW|%s|rc_rise=%s falls=%s' % (name, sorted(rise.at.items(), key=lambda x: x[1]), falls.at))
    log('ROW|%s|edges=%s' % (name, {k: (len(v), v[:4]) for k, v in edges.at.items() if len(v) > 1}))
    log('ROW|%s|farend_ltssm=%s' % (name, fe.ltssm.history))
    log('ROW|%s|farend_dll=%s stats=%s' % (name, fe.dll.history, fe.dll.stats))
    log('ROW|%s|injected=%s' % (name, fe.injected))
    log('ROW|%s|flags=%s' % (name, sorted(kinds.items())))
    log('ROW|%s|first_flags=%s' % (name, [(f['id'], f['t'], f['detail']) for f in chk.flags
                                          if f['id'] not in KNOWN][:4]))
    log('ROW|%s|checked=%s' % (name, sorted(chk.n.items())))
    log('ROW|%s|requests=%d cycles=%d rc_tlps=%d' % (name, len(fe.tl.requests), fe.cycle, len(fe.rc_tlps)))
    return fe


def enumerated(dut, fe):
    """The RC enumerated the far end: the conforming outcome of every D8 row."""
    rise = fe.rise
    for n in ('link_up_o', 'fc_init_done_o', 'enum_done_o'):
        assert n in rise.at, '%s never rose (rises: %s)' % (n, rise.at)
    assert 'enum_error_o' not in rise.at and 'scan_error_o' not in rise.at, 'enumeration error: %s' % rise.at
    assert int(dut.vendor_id_o.value) == CFG.VENDOR_ID and int(dut.device_id_o.value) == CFG.DEVICE_ID, \
        'IDs %04x:%04x' % (int(dut.vendor_id_o.value), int(dut.device_id_o.value))
    assert int(dut.bar_valid_o.value) & 1, 'BAR0 not valid'
    assert not fe.falls.at, 'a level fell: %s' % fe.falls.at
    assert fe.dll.state == 'ACTIVE', 'far end DLL %s' % fe.dll.state


def flags_only(fe, allowed=()):
    unknown = {k: v for k, v in fe.kinds.items() if k not in KNOWN | set(allowed)}
    assert not unknown, 'far-end checks flagged %s; first: %s' % (
        unknown, [f for f in fe.chk.flags if f['id'] in unknown][:3])


def pinned_red(dut, row, state, detail):
    dut._log.info('PINNED_RED|%s|%s|%s' % (row, state, detail))


def injected(fe, key):
    hits = [(t, d) for t, k, d in fe.injected if k == key]
    assert len(hits) == 1, 'injection %s happened %d times: %s' % (key, len(hits), fe.injected)
    return hits[0][0]


# ------------------------------------------------------------- the D4 row
# The same lists as test_lr1_kat.kat_independence.
STDLIB_OK = {'ast', 'binascii', 'collections', 'dataclasses', 'enum', 'functools', 'hashlib', 'heapq',
             'itertools', 'json', 'math', 'os', 'random', 'struct', 'sys', 'zlib', 'typing'}
COCOTB_OK = {'cocotb', 'cocotb.triggers', 'cocotb.clock', 'cocotb.handle', 'cocotb.result', 'cocotb.utils'}


@cocotb.test()
async def inject_independence(dut):
    """D4: every lr1_*.py beside this file -- lr1_host and lr1_inject among
    them -- imports only the standard library, cocotb and lr1_*."""
    here = os.path.dirname(os.path.abspath(__file__))
    mods = sorted(f for f in os.listdir(here) if f.startswith('lr1_') and f.endswith('.py'))
    assert 'lr1_inject.py' in mods and 'lr1_host.py' in mods, mods
    bad = []
    for m in mods:
        with open(os.path.join(here, m)) as fh:
            tree = ast.parse(fh.read())
        for node in ast.walk(tree):
            names = []
            if isinstance(node, ast.Import):
                names = [a.name for a in node.names]
            elif isinstance(node, ast.ImportFrom):
                names = [node.module or '']
            bad += [(m, n) for n in names if not (n in STDLIB_OK or n in COCOTB_OK or n.startswith('lr1_'))]
    dut._log.info('INDEPENDENCE|files=%d|%s' % (len(mods), ','.join(mods)))
    assert not bad, 'imports outside D4: %s' % bad


# ---------------------------------------------------------- the control
@cocotb.test()
async def inject_control(dut):
    """Every injection off. The injecting far end must be transparent: the RC
    enumerates it with only the KNOWN flag, and nothing was injected."""
    fe = await run_row(dut, InjKnobs(), 'inject_control')
    enumerated(dut, fe)
    flags_only(fe)
    assert not fe.injected, fe.injected
    vac = [c for c in NONVACUOUS if not fe.chk.n.get(c)]
    assert not vac, 'checks that examined nothing: %s' % vac


# --------------------------------------------------------- record helpers
# Times are Symbol Times at the pins (t = 2 x cycle + lane), as in lr1_rx.
RC_SEQ = 3                    # the RC's fourth TLP: a Configuration Request in the middle of enumeration
FE_SEQ = 2                    # the far end's third TLP: a Completion in the middle of enumeration
HOLD = 2000                   # cycles a Completion is held when a row needs the RC's timer to run alone


def fe_end(fe, seq, replay=False):
    """Time of the last Symbol of the far end's TLP `seq` (its first transmission, or its first resend)."""
    return next((t for t, s, r in fe.fe_tlp_ends if s == seq and r == replay), None)


def rc_tlp(fe, seq, first=True):
    """(start, end) of the RC's TLP `seq`: its first transmission, or its first resend."""
    return next(((t, e) for t, e, s, f in fe.rc_tlps if s == seq and f == first), None)


def first_acknak_after(fe, t):
    return next(iter(fe.rc_acknaks(after=t + 1)), None)


def covers(seq_acked, seq):
    return (seq_acked - seq) % 4096 < 2048


def bad_tlp_row(fe, seq, wire_seq=None):
    """A far-end TLP the RC must not accept (p.182-183): the first Ack / Nak
    the RC sends after its last Symbol is a Nak naming seq - 1, or at least no
    Ack before the far end resends it covers it; the resend is then Acked.
    Returns (detail, no Ack covered it before the resend, the first RC Ack /
    Nak after it, the RC Acks covering the resend)."""
    t1 = fe_end(fe, seq if wire_seq is None else wire_seq)    # the first transmission, by its number on the wire
    t2 = fe_end(fe, seq, replay=True)
    assert t1 is not None, 'the far end never sent seq %03x' % seq
    assert t2 is not None, 'the far end never resent seq %03x (no Nak, and its own REPLAY_TIMER did not run?)' % seq
    before = [(t, k, s) for t, k, s in fe.rc_acknaks(after=t1 + 1) if t < t2]
    acked_early = [(t, k, s) for t, k, s in before if k == 'Ack' and covers(s, seq)]
    first = before[0] if before else None
    after = [(t, k, s) for t, k, s in fe.rc_acknaks(after=t2 + 1) if k == 'Ack' and covers(s, seq)]
    detail = 'seq %03x END at %d, resent END at %d; RC Ack/Nak between: %s; first Ack covering it after the resend: %s' % (
        seq, t1, t2, before[:3], after[:1])
    return detail, not acked_early, first, after


# ================================================================ D8 rows
@cocotb.test()
async def inj_bad_lcrc(dut):
    """D8: the far end's third TLP (a Completion) with one LCRC bit inverted.
    §3.5.3.1 p.183: the TLP is corrupt -- discard it, schedule a Nak. The
    far end then resends it (on the Nak, or on its REPLAY_TIMER) and the RC
    must accept the resend.

    Asserts: no RC Ack covers it before the resend; the first RC Ack / Nak
    after it is a Nak naming seq - 1; an Ack covers the resend; enumeration
    completes."""
    fe = await run_row(dut, InjKnobs(bad_lcrc_seq=FE_SEQ), 'inj_bad_lcrc')
    injected(fe, 'bad_lcrc')
    detail, ok, first, after = bad_tlp_row(fe, FE_SEQ)
    dut._log.info('MEASURE|inj_bad_lcrc|%s' % detail)
    assert ok, 'the RC acknowledged a TLP with a bad LCRC: ' + detail
    assert first is not None and first[1] == 'Nak' and first[2] == FE_SEQ - 1, 'no Nak(seq - 1): ' + detail
    assert after, 'the resend was never acknowledged: ' + detail
    enumerated(dut, fe)
    flags_only(fe)


@cocotb.test()
async def inj_seq_ahead(dut):
    """D8: the far end sends its third TLP under the next sequence number --
    out of sequence, as if one TLP were lost. §3.5.3.1 p.183: discard it and
    schedule a Nak ("report TLP missing"); the resend under the right number
    is accepted.

    Asserts as inj_bad_lcrc."""
    fe = await run_row(dut, InjKnobs(seq_ahead_seq=FE_SEQ), 'inj_seq_ahead')
    injected(fe, 'seq_ahead')
    detail, ok, first, after = bad_tlp_row(fe, FE_SEQ, wire_seq=FE_SEQ + 1)
    dut._log.info('MEASURE|inj_seq_ahead|%s' % detail)
    assert ok, 'the RC acknowledged an out-of-sequence TLP: ' + detail
    assert first is not None and first[1] == 'Nak' and first[2] == FE_SEQ - 1, 'no Nak(seq - 1): ' + detail
    assert after, 'the resend was never acknowledged: ' + detail
    enumerated(dut, fe)
    flags_only(fe)


@cocotb.test()
async def inj_dup(dut):
    """D8: the far end sends its third TLP again, unchanged, after the RC
    acknowledged it. §3.5.3.1 p.183: a duplicate -- discard it and schedule
    an Ack (not a Nak); it must not reach the Transaction Layer a second
    time.

    Asserts: the first RC Ack / Nak after the duplicate is an Ack covering
    it; rc_unexpected_completion_o never rises; enumeration completes."""
    fe = await run_row(dut, InjKnobs(dup_seq=FE_SEQ), 'inj_dup')
    injected(fe, 'dup')
    t2 = fe_end(fe, FE_SEQ, replay=True)
    assert t2 is not None, 'the duplicate was never sent'
    first = first_acknak_after(fe, t2)
    detail = 'duplicate END at %d; first RC Ack/Nak after it %s' % (t2, first)
    dut._log.info('MEASURE|inj_dup|%s' % detail)
    assert first is not None and first[1] == 'Ack' and covers(first[2], FE_SEQ), detail
    assert not fe.edges.rises('rc_unexpected_completion_o'), 'an unexpected Completion: %s' % fe.edges.at
    enumerated(dut, fe)
    flags_only(fe)


@cocotb.test()
async def inj_no_end(dut):
    """D8: the far end's third TLP with its END replaced by a Logical Idle
    Symbol (a missing END). Checking the framing rules is optional (§4.2.2
    p.196), but the LCRC check is not (p.183): the RC must not accept the TLP
    as sent, and must accept its resend.

    Asserts: no RC Ack covers it before the resend; an Ack covers the
    resend; enumeration completes."""
    fe = await run_row(dut, InjKnobs(no_end_seq=FE_SEQ), 'inj_no_end')
    injected(fe, 'no_end')
    detail, ok, first, after = bad_tlp_row(fe, FE_SEQ)
    dut._log.info('MEASURE|inj_no_end|%s' % detail)
    assert ok, 'the RC acknowledged a TLP with no END: ' + detail
    assert after, 'the resend was never acknowledged: ' + detail
    enumerated(dut, fe)
    flags_only(fe)


@cocotb.test(expect_fail=True)
async def inj_rxerr(dut):
    """D8: a PIPE receive error. The far end's PHY reports RxStatus 100b
    (8b/10b decode error, PG239 Table 10) on the word carrying Symbol 8 of its
    third TLP; the Symbol's value is unchanged, so only the status carries
    the error. §4.2.4.6 p.209: 8b/10b decode errors must be checked and
    trigger a Receiver Error; §3.5.3.1 p.182: on a Receiver Error discard the
    TLP being received and schedule a Nak.

    !! PG239's data at a decode error is not documented; LR1-c measures it
    (BRIEF_LR1 D12) and corrects this row if the real PHY drives the seam
    differently.

    PINNED: no RC Ack covers the TLP before the far end resends it."""
    row = 'inj_rxerr'
    try:
        fe = await run_row(dut, InjKnobs(rxerr_seq=FE_SEQ, rxerr_code=0b100), row)
        injected(fe, 'rxerr')
        assert fe.rxerr_words, 'no word was driven with the error status'
        t1 = fe_end(fe, FE_SEQ)
        early = [(t, k, s) for t, k, s in fe.rc_acknaks(after=t1 + 1)
                 if k == 'Ack' and covers(s, FE_SEQ)][:1]
        t2 = fe_end(fe, FE_SEQ, replay=True)
        acked_early = bool(early) and (t2 is None or early[0][0] < t2)
        detail = 'RxStatus 100b at %s; seq %03x END at %d; first RC Ack covering it %s; resend END %s' % (
            fe.rxerr_words[:1], FE_SEQ, t1, early, t2)
        dut._log.info('MEASURE|%s|%s' % (row, detail))
        enumerated(dut, fe)
    except Exception as exc:
        pinned_red(dut, row, 'NOT_REACHED', repr(exc))
        return
    pinned_red(dut, row, 'REACHED', detail)
    assert not acked_early, 'the RC accepted a TLP received with a decode error: ' + detail


@cocotb.test()
async def inj_nak(dut):
    """D8: the far end Naks the RC's fourth TLP (a Configuration Request), as
    a receiver that saw it corrupted would (p.183). §3.5.2.1 p.174: a Nak
    initiates a replay of every unacknowledged TLP, oldest first, same
    content (the far end's REPLAY check).

    Asserts: the RC sends the TLP again, starting less than 711 Symbol Times
    (Table 3-4 p.176, x1, 128 B) after its first transmission ended -- so
    the Nak, not REPLAY_TIMER, started it; enumeration completes."""
    fe = await run_row(dut, InjKnobs(nak_rc_seq=RC_SEQ), 'inj_nak')
    injected(fe, 'nak_rc')
    a = rc_tlp(fe, RC_SEQ)
    b = rc_tlp(fe, RC_SEQ, first=False)
    t_nak = fe.first_sent.get('Nak')
    detail = 'RC seq %03x %s, resent %s; far-end Nak driven at cycle %s' % (RC_SEQ, a, b, t_nak)
    dut._log.info('MEASURE|inj_nak|%s' % detail)
    assert a is not None and b is not None, 'no resend: ' + detail
    assert b[0] - a[1] < 711, 'the resend started %d Symbol Times after the first ended: ' % (b[0] - a[1]) + detail
    enumerated(dut, fe)
    flags_only(fe)


async def timer_replay_row(dut, row, knobs, key):
    """The RC's REPLAY_TIMER alone: no Ack it accepts arrives for RC_SEQ, and
    the Completion of that request is held, so nothing else acknowledges it.
    Table 3-4 p.176: x1, 128 B, 711 Symbol Times, tolerance -0% / +100%,
    from the TLP's last Symbol to the first Symbol of its retransmission,
    allowing any packet already in progress (p.176)."""
    fe = await run_row(dut, knobs, row)
    injected(fe, key)
    a = rc_tlp(fe, RC_SEQ)
    b = rc_tlp(fe, RC_SEQ, first=False)
    detail = 'RC seq %03x %s, resent %s' % (RC_SEQ, a, b)
    limit = None
    if a is not None and b is not None:
        gap = b[0] - a[1]
        inprog = [(s, e) for s, e in fe.chk.units if s <= a[1] + 1422 <= e]
        limit = 1422 + (inprog[0][1] - (a[1] + 1422) + 1 if inprog else 0)
        detail += '; %d Symbol Times from its last Symbol to the resend (window 711..%d)' % (gap, limit)
    dut._log.info('MEASURE|%s|%s' % (row, detail))
    return fe, a, b, detail, limit


@cocotb.test()
async def inj_no_ack(dut):
    """D8: a dropped Ack. The far end does not acknowledge the RC's fourth TLP
    and holds its Completion 2,000 clocks. §3.5.2.1 p.175: with no Ack or Nak
    for a period exceeding Table 3-4's, the Transmitter initiates a replay.

    Asserts: the RC resends the TLP 711 to 1,422 Symbol Times after its last
    Symbol (plus any packet in progress, p.176); enumeration completes."""
    fe, a, b, detail, limit = await timer_replay_row(
        dut, 'inj_no_ack', InjKnobs(no_ack_rc_seq=RC_SEQ, hold_cpl_rc_seq=RC_SEQ, hold_cpl_cycles=HOLD), 'no_ack_rc')
    assert a is not None and b is not None, 'no resend: ' + detail
    assert 711 <= b[0] - a[1] <= limit, detail
    enumerated(dut, fe)
    flags_only(fe)


@cocotb.test()
async def inj_bad_crc_ack(dut):
    """D8: a corrupt Ack. The far end's Ack for the RC's fourth TLP has one CRC
    bit inverted, and the Completion is held 2,000 clocks. §3.5.2.2 p.179: a
    corrupt DLLP is discarded -- so the RC must behave as if no Ack came, and
    replay on REPLAY_TIMER (p.175, Table 3-4 p.176).

    Asserts as inj_no_ack."""
    fe, a, b, detail, limit = await timer_replay_row(
        dut, 'inj_bad_crc_ack', InjKnobs(bad_crc_ack_seq=RC_SEQ, hold_cpl_rc_seq=RC_SEQ, hold_cpl_cycles=HOLD),
        'bad_crc_ack')
    assert a is not None and b is not None, 'no resend -- the corrupt Ack was accepted? ' + detail
    assert 711 <= b[0] - a[1] <= limit, detail
    enumerated(dut, fe)
    flags_only(fe)


def recovery_flags_registered(fe):
    """Flags a Recovery brings with it that are registered elsewhere, each
    attributed: SKP_GAP_L0 across a Recovery (the RC's SKP counter restarts on
    every LTSSM request, PHASE0 N2; HANDSHAKE §7's SKP row), and a malformed
    set directly after an RC SKP OS (§63 #29, transmit) with the
    TS_INTERRUPT it causes. Returns the kinds so attributed; fails on any
    FRAMING flag that is not a set directly after an RC SKP OS."""
    bad_after_skp = {t for t, k in fe.chk.after_skp if k == 'OS_BAD'}
    framing = [f for f in fe.chk.flags if f['id'] == 'FRAMING']
    stray = [f for f in framing if f['t'] not in bad_after_skp]
    assert not stray, 'FRAMING flags not explained by #29: %s' % stray[:3]
    return {'SKP_GAP_L0', 'FRAMING', 'TS_INTERRUPT'}


async def direct_recovery_after(fe, n_cpl, hold=0):
    """Direct Recovery (an upstream port may, §4.2.6.5 p.248) in the clock the
    far end drives the last Symbol of its n_cpl-th TLP. Returns the cycle."""
    await fe.run(60000, until=lambda: len([1 for _, _, r in fe.fe_tlp_ends if not r]) >= n_cpl, every=1)
    fe.ltssm.direct('REC_LOCK', fe.cycle, hold=hold)
    return fe.cycle


@cocotb.test()
async def inj_recovery_l0(dut):
    """D8: TS1 in L0. In the middle of enumeration (after the far end's fifth
    Completion, plus 300 clocks) the far end directs Recovery. LinkUp stays 1
    through Recovery (Table 4-7 p.216), so the Data Link Layer stays
    DL_Active (§3.2.1 p.158); the RC trains through Recovery back to L0 and
    enumeration completes.

    Asserts: link_up_o and fc_init_done_o never fall; the RC sent training
    sets after its first L0 and returned to sending packets; enumeration
    completes; any SKP_GAP_L0 / FRAMING / TS_INTERRUPT flag is a registered
    one (recovery_flags_registered)."""
    res = {}

    async def go(fe, rise):
        await fe.run(60000, until=lambda: len([1 for _, _, r in fe.fe_tlp_ends if not r]) >= 5, every=1)
        await fe.run(300)
        res['t0'] = fe.cycle
        fe.ltssm.direct('REC_LOCK', fe.cycle)
        await fe.run(80000, until=lambda: 'enum_done_o' in rise.at or 'enum_error_o' in rise.at)
    fe = await run_row(dut, InjKnobs(), 'inj_recovery_l0', until=lambda f, r: 'fc_init_done_o' in r.at, after=go)
    rc_ts = [t for t, k in fe.rc_ts if t >= 2 * res['t0']]
    later = [t for t, k in fe.rc_units if k in ('DLLP', 'TLP') and rc_ts and t > rc_ts[-1]]
    detail = 'Recovery directed at cycle %d; RC training sets after it %d (%s..%s); RC packets after them %d' % (
        res['t0'], len(rc_ts), rc_ts[:1], rc_ts[-1:], len(later))
    dut._log.info('MEASURE|inj_recovery_l0|%s' % detail)
    assert len(rc_ts) >= 8 and later, detail
    enumerated(dut, fe)
    flags_only(fe, recovery_flags_registered(fe))


@cocotb.test(expect_fail=True)
async def inj_recovery_waiting_tlp(dut):
    """D8: a waiting TLP when Recovery starts. The far end directs Recovery in
    the clock it drives the last Symbol of its fifth Completion, so the RC's
    next Configuration Request is formed while the link enters Recovery.
    §4.2.4.1 p.201: training sequences are interrupted only by SKP Ordered
    Sets; §4.2.6.4.4 p.246: in Recovery.Idle the Transmitter sends Idle data,
    and L0 follows only after 8 idle Symbol Times are received and 16 idle
    Symbols sent after receiving one (lr1_inject.recovery_packets). The
    request goes out after Recovery and enumeration completes. HANDSHAKE §7
    registers the mechanism (F10: the PHY does not throttle the Data Link
    Layer in Recovery).

    PINNED: no RC packet between two of its training sets, and none in
    Recovery.Idle before both conditions hold."""
    res = {}

    async def go(fe, rise):
        res['t0'] = await direct_recovery_after(fe, 5)
        await fe.run(80000, until=lambda: 'enum_done_o' in rise.at or 'enum_error_o' in rise.at)
    row = 'inj_recovery_waiting_tlp'
    try:
        fe = await run_row(dut, InjKnobs(), row, until=lambda f, r: 'fc_init_done_o' in r.at, after=go)
        t0 = 2 * res['t0']
        between, early, info = fe.recovery_packets(t0, 2 * fe.cycle)
        detail = 'Recovery directed at %d; %s; between training sets %s; early in Recovery.Idle %s' % (
            t0, info, between, early)
        dut._log.info('MEASURE|%s|%s' % (row, detail))
        assert info['first_ts'] is not None, 'the RC sent no training set: ' + detail
        enumerated(dut, fe)
    except Exception as exc:
        pinned_red(dut, row, 'NOT_REACHED', repr(exc))
        return
    pinned_red(dut, row, 'REACHED', detail)
    assert not between and not early, detail


@cocotb.test(expect_fail=True)
async def inj_no_ack_mwr128(dut):
    """D8: a dropped Ack on a 128-byte host write -- a Max_Payload_Size TLP
    (37 four-byte beats with its sequence number and LCRC). After enumeration
    the host writes 128 B into BAR0 and the far end does not acknowledge it;
    nothing else is sent to the RC. §3.5.2.1 p.175 and Table 3-4 p.176: the
    RC must retransmit it 711 to 1,422 Symbol Times after its last Symbol.

    Added after prov_f1's first development attempt (A at 128 B) drew no
    replay at all; ledger :426 (§63 #19, before NVMe traffic) reads that the
    retry copy is written on tvalid without tready, so a back-pressured TLP
    of about 36 beats or more is stored wrong. Measured in development: the
    RC replays a lost-Ack write of 4 to 56 B (1,237 Symbol Times) and none of
    60 B or more.

    PINNED: the RC resends the write 711 to 1,422 Symbol Times (plus any
    packet in progress) after its last Symbol."""
    row = 'inj_no_ack_mwr128'
    res = {}
    try:
        from lr1_host import Host

        async def go(fe, rise):
            host = Host(dut, lambda: fe.cycle)
            host.idle()
            base = int(dut.bar_addr_o.value) & ((1 << 64) - 1)
            seq = fe.chk.rc_seq_next
            res['seq'] = seq
            fe.k.no_ack_rc_seq = seq
            await fe.run(200)
            h = cocotb.start_soon(host.mwr(base, bytes(range(128))))
            await fe.run(4000, until=lambda: (rc_tlp(fe, seq) or (0, None))[1] is not None and h.done(), every=1)
            await fe.run(3000)
        fe = await run_row(dut, InjKnobs(adv_credits={'P': (0x10, 0x100), 'NP': (1, 1), 'CPL': (0, 0)}), row,
                           after=go, tail=500)
        seq = res['seq']
        injected(fe, 'no_ack_rc')
        a = rc_tlp(fe, seq)
        b = rc_tlp(fe, seq, first=False)
        assert a is not None and a[1] - a[0] >= 140, 'the write did not go out as one 128-byte TLP: %s' % (a,)
        inprog = [(s0, e) for s0, e in fe.chk.units if s0 <= a[1] + 1422 <= e]
        limit = 1422 + (inprog[0][1] - (a[1] + 1422) + 1 if inprog else 0)
        ok = b is not None and 711 <= b[0] - a[1] <= limit
        detail = 'RC seq %03x %s (%d Symbols), resent %s; RC units after it %s' % (
            seq, a, a[1] - a[0] + 1, b, [(t, k) for t, k in fe.rc_units if t > a[1]][:8])
        dut._log.info('MEASURE|%s|%s' % (row, detail))
    except Exception as exc:
        pinned_red(dut, row, 'NOT_REACHED', repr(exc))
        return
    pinned_red(dut, row, 'REACHED', detail)
    assert ok, 'no resend in the window 711..%d: ' % limit + detail


# ===================================================== Reserved-bit DLLPs
@cocotb.test(expect_fail=True)
async def rsvd_ack(dut):
    """§63 #28 (RULINGS_LR1 F): the far end's Ack for the RC's fourth TLP
    carries 01h in its Reserved byte 1 (CRC over the bytes sent), and the
    Completion is held 2,000 clocks so no other Ack follows. §3.5.2.2 p.179:
    "Non-zero values in Reserved fields are ignored" (also §3.4.1 p.164), so
    the Ack acknowledges the TLP.

    PINNED: the RC does not send the TLP again within 1,500 clocks of the
    Ack (a dropped Ack would replay it after REPLAY_TIMER, 622 clocks)."""
    row = 'rsvd_ack'
    try:
        fe = await run_row(dut, InjKnobs(rsvd_ack_seq=RC_SEQ, hold_cpl_rc_seq=RC_SEQ, hold_cpl_cycles=HOLD), row)
        t_ack = injected(fe, 'rsvd_ack')
        injected(fe, 'hold_cpl')
        a = rc_tlp(fe, RC_SEQ)
        b = rc_tlp(fe, RC_SEQ, first=False)
        resent = b is not None and b[0] <= 2 * (t_ack + 1500)
        detail = 'Ack with Reserved byte 01h at cycle %d; RC seq %03x %s, resent %s' % (t_ack, RC_SEQ, a, b)
        dut._log.info('MEASURE|%s|%s' % (row, detail))
        enumerated(dut, fe)
    except Exception as exc:
        pinned_red(dut, row, 'NOT_REACHED', repr(exc))
        return
    pinned_red(dut, row, 'REACHED', detail)
    assert not resent, 'the RC dropped an Ack with a Reserved bit set and replayed: ' + detail


# ======================================== the newer-revision partner (ruling F)
@cocotb.test(expect_fail=True)
async def newer_scale(dut):
    """Ruling F: a newer-revision partner sets the two Reserved fields of its
    InitFC1 DLLPs (byte 1 [7:6], byte 2 [5:4] -- where a later revision's
    scaled flow control puts HdrScale / DataScale; RULINGS_LR1 F records that
    the shelf cannot confirm that revision's text). Base 2.1 p.164 and p.179:
    Reserved fields are ignored, so FC init completes.

    !! The RC records FI1 from InitFC1 only (F2, §63 #14): if it also drops
    these InitFC1s (§63 #28), the far end's clean InitFC2s cannot set its
    FI1. This row turns green when either defect is fixed.

    PINNED: fc_init_done_o rises within 12,500 clocks (3 x 34 us) of the far
    end's first InitFC1 reaching the RC."""
    row = 'newer_scale'
    try:
        k = InjKnobs(fc1_rsvd={'rsvd1': 0b01, 'rsvd2': 0b01})
        until = lambda f, r: 'fc_init_done_o' in r.at or (
            f.first_sent.get('InitFC1') is not None and f.cycle - f.first_sent['InitFC1'] > 12500)
        fe = await run_row(dut, k, row, until=until, tail=0)
        injected(fe, 'fc1_rsvd')
        t1 = fe.first_sent.get('InitFC1')
        assert t1 is not None, 'the far end sent no InitFC1'
        assert fe.chk.n.get('FC_INIT_ORDER', 0) > 0, 'the RC sent no InitFC'
        done = fe.rise.at.get('fc_init_done_o')
        detail = 'first InitFC1 (Reserved fields set) at %d, fc_init_done_o at %s' % (t1, done)
        dut._log.info('MEASURE|%s|%s' % (row, detail))
    except Exception as exc:
        pinned_red(dut, row, 'NOT_REACHED', repr(exc))
        return
    pinned_red(dut, row, 'REACHED', detail)
    assert done is not None and done - t1 <= 12500, 'FC init never completed: ' + detail


async def unknown_dllp_row(dut, row, typ):
    fe = await run_row(dut, InjKnobs(unknown_dllp=typ), row)
    t = injected(fe, 'unknown_dllp')
    sent = fe.first_sent.get('Unknown')
    t1 = fe.first_sent.get('InitFC1')
    detail = 'DLLP %02xh driven at %s, the far end\'s first InitFC1 at %s; rises %s' % (typ, sent, t1, fe.rise.at)
    dut._log.info('MEASURE|%s|%s' % (row, detail))
    assert sent is not None and t1 is not None and sent <= t1, 'the DLLP did not precede InitFC1: ' + detail
    enumerated(dut, fe)
    flags_only(fe)


@cocotb.test()
async def newer_dllp02(dut):
    """Ruling F: a newer-revision partner's DLLP of type 02h (a later
    revision's Data Link Feature DLLP; Reserved in Base 2.1 Table 3-1 p.165)
    once at DL_Init, before its first InitFC1. §3.5.2.2 p.179: a DLLP with an
    unsupported type encoding is discarded without further action. ⚠️ The RC
    decodes 02h as "Feature_Exchange" (RULINGS_LR1 F, code's note).

    Asserts: FC init completes and enumeration completes, with only the
    KNOWN flag."""
    await unknown_dllp_row(dut, 'newer_dllp02', 0x02)


@cocotb.test()
async def newer_dllp01(dut):
    """Ruling F: a DLLP of type 01h, Reserved in Table 3-1 p.165 and not
    decoded by the RC, once at DL_Init before the first InitFC1. p.179:
    discarded without further action.

    Asserts as newer_dllp02."""
    await unknown_dllp_row(dut, 'newer_dllp01', 0x01)


# ========================================================== provocations
