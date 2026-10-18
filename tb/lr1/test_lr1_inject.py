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
