#!/usr/bin/env python3
"""sec 63 #7k Phase 1 -- offline pairing for the probe_7k raw-event logs.

sec 22.92: probe_7k.sv emits raw events only; every pairing, classification
and count happens here, and this file opens with a hand-derived known-answer
self-test that must pass before any real log is read.

Inputs, one file per instance in the target's work dir:
  pr7k_rm.<%m>.log   retry_management  S/B/E/T/A events
  pr7k_lt.<%m>.log   LTSSM             L/U events
  pr7k_dll.<%m>.log  datalink layer    U/D/F/H/X events

Usage:
  analyse_7k.py selftest
  analyse_7k.py w1 <work_dir> <arm_ps> <release_ps>   (probe $time units)
"""
import sys, os, glob

TCK = 8000   # one 8 ns clock in probe $time units: the probes print PICOSECONDS (measured:
             # the RC's first send after arm logs 55984000 = row cycle 6978 + 20 = 6998 x 8000)
RM_NAMES = {0: 'IDLE', 1: 'CNT_RETRY', 2: 'REPLAY', 3: 'WAIT_REPLAY', 4: 'RETRY_ERR'}
RM_CNT_RETRY, RM_REPLAY, RM_ERR = 1, 2, 4
DL_ACTIVE = 4
FC_CHECK_FC2 = 16
INITFC = {0x40, 0x50, 0x60, 0xC0, 0xD0, 0xE0}
RECOVERY = 0x04


def family(st):
    return st & 0x1F


def read(path):
    ev = []
    for ln in open(path):
        f = ln.split()
        if len(f) >= 2:
            ev.append((f[0], int(f[1]), f[2:]))
    return ev


def side_of(path):
    """RC or EP from the %m instance path in the file name."""
    return 'rc' if '.u_rc.' in path else 'ep' if '.u_ep.' in path else '?'


# --------------------------------------------------------------------------
# pairing -- pure functions over parsed events
# --------------------------------------------------------------------------
def slot_timeline(rm_ev):
    """{slot: [(t, state, replay_cnt)]} from S events."""
    out = {}
    for tag, t, f in rm_ev:
        if tag == 'S':
            out.setdefault(int(f[0]), []).append((t, int(f[1]), int(f[2])))
    return out


def sends(rm_ev):
    return [(t, int(f[0], 16)) for tag, t, f in rm_ev if tag == 'T']


def starved_seq(rm_ev, after):
    s = [(t, q) for t, q in sends(rm_ev) if t >= after]
    if not s:
        return None
    q0 = s[0][1]
    return q0, [t for t, q in s if q == q0]


def timer_expiries(tl, acks, after):
    """CNT_RETRY -> REPLAY or CNT_RETRY -> RETRY_ERR with no Ack/Nak on that
    edge's cycle or the one before: the REPLAY_TIMER branch (:269-283)."""
    out = []
    ack_t = {t for t, _, _ in acks}
    for (t0, s0, _), (t1, s1, c1) in zip(tl, tl[1:]):
        if t1 >= after and s0 == RM_CNT_RETRY and s1 in (RM_REPLAY, RM_ERR) and \
                not ({t1, t1 - TCK} & ack_t):
            out.append((t1, RM_NAMES[s1], c1))
    return out


def recovery_entries(lt_ev, after):
    out, prev = [], None
    for tag, t, f in lt_ev:
        if tag != 'L':
            continue
        st = int(f[0], 16)
        if family(st) == RECOVERY and (prev is None or family(prev) != RECOVERY) and t >= after:
            out.append(t)
        prev = st
    return out


def changes(ev, tag, after):
    return [(t, int(f[0])) for g, t, f in ev if g == tag and t >= after]


def handler_window(dll_ev, lo, hi):
    acc = rej = 0
    prev = None
    for tag, t, f in dll_ev:
        if tag != 'H':
            continue
        st = int(f[0])
        if lo <= t <= hi:
            if st == 2 and prev != 2:
                acc += 1
            if st == 0 and prev == 1:
                rej += 1
        prev = st
    return acc, rej


# --------------------------------------------------------------------------
def selftest():
    k = TCK
    rm = [('S', 0, ['0', '0', '0']), ('S', 100 * k, ['0', '1', '0']),
          ('T', 100 * k, ['01a']), ('S', 722 * k, ['0', '2', '1']), ('S', 724 * k, ['0', '3', '1']),
          ('T', 732 * k, ['01a']), ('S', 733 * k, ['0', '1', '1']),
          ('S', 1355 * k, ['0', '4', '1']), ('A', 2000 * k, ['1', '01a'])]
    tl = slot_timeline(rm)[0]
    assert [x[1] for x in tl] == [0, 1, 2, 3, 1, 4], 'SELFTEST slot timeline'
    assert starved_seq(rm, 50 * k) == (0x1a, [100 * k, 732 * k]), 'SELFTEST starved seq'
    acks = [(t, int(f[0]), int(f[1], 16)) for g, t, f in rm if g == 'A']
    assert timer_expiries(tl, acks, 0) == [(722 * k, 'REPLAY', 1), (1355 * k, 'RETRY_ERR', 1)], \
        'SELFTEST expiries'
    assert timer_expiries(tl, [(1354 * k, 1, 0x1a)], 0) == [(722 * k, 'REPLAY', 1)], \
        'SELFTEST an Ack one cycle before the edge is not a timer expiry'
    lt = [('L', 0, ['00000']), ('L', 800, ['00005']), ('U', 808, ['1']),
          ('L', 9000, ['00024']), ('L', 9400, ['000e4']), ('L', 9800, ['00104']),
          ('L', 9900, ['00005'])]
    assert recovery_entries(lt, 0) == [9000], 'SELFTEST recovery entry'
    assert recovery_entries(lt, 9001) == [], 'SELFTEST recovery after'
    dll = [('H', 0, ['0']), ('H', 10, ['1']), ('H', 18, ['2']), ('H', 26, ['0']),
           ('H', 40, ['1']), ('H', 48, ['0'])]
    assert handler_window(dll, 0, 100) == (1, 1), 'SELFTEST handler'
    assert handler_window(dll, 30, 100) == (0, 1), 'SELFTEST handler lo'
    print('SELFTEST PASS')


def w1(wd, arm_ns, rel_ns):
    selftest()
    files = {k: sorted(glob.glob(os.path.join(wd, f'pr7k_{k}.*.log'))) for k in ('rm', 'lt', 'dll')}
    for k, v in files.items():
        print(f'{k}: {[os.path.basename(p) for p in v]}')
    by = {}
    for k, paths in files.items():
        for p in paths:
            by.setdefault(side_of(p), {})[k] = read(p)
    for s in ('rc', 'ep'):
        d = by.get(s, {})
        rm, lt, dll = d.get('rm', []), d.get('lt', []), d.get('dll', [])
        acks = [(t, int(f[0]), int(f[1], 16)) for g, t, f in rm if g == 'A']
        print(f'== {s.upper()}')
        sq = starved_seq(rm, arm_ns)
        print(f'  first TLP after arm: {sq and hex(sq[0])} sends(ns)={sq and sq[1][:8]}')
        if sq:
            g = [b - a for a, b in zip(sq[1], sq[1][1:])]
            print(f'  send gaps (cycles): {[x // TCK for x in g][:8]}')
        for slot, tl in sorted(slot_timeline(rm).items()):
            ex = timer_expiries(tl, acks, arm_ns)
            path = [(t, RM_NAMES[st], c) for t, st, c in tl if t >= arm_ns - 100]
            print(f'  slot {slot}: expiries={ex}')
            print(f'           path={path[:20]}')
        print(f'  retry_err_o: {changes(rm, "E", 0)}')
        print(f'  retrys_r (buffer): {[(t, f[0]) for g, t, f in rm if g == "B" and t >= arm_ns - 100][:20]}')
        print(f'  acks/naks in after arm: {[a for a in acks if a[0] >= arm_ns][:20]}')
        print(f'  LTSSM recovery entries after arm: {recovery_entries(lt, arm_ns)}')
        print(f'  LTSSM link_up_o changes: {changes(lt, "U", 0)}')
        print(f'  DLL link_up_i changes: {changes(dll, "U", 0)}')
        print(f'  DLCMSM changes: {changes(dll, "D", 0)}')
        print(f'  FC-init changes after arm: {changes(dll, "F", arm_ns)[:12]}')
        x = [(t, f[0]) for g, t, f in dll if g == 'X' and t >= arm_ns]
        print(f'  DLLPs TX after arm: n={len(x)} initfc={[e for e in x if int(e[1], 16) & 0xF8 in INITFC]} '
              f'types={sorted({e[1] for e in x})}')
        print(f'  dllp_handler [arm+50 cycles, release]: accepted/crc_rejected = '
              f'{handler_window(dll, arm_ns + 50 * TCK, rel_ns)}')


if __name__ == '__main__':
    if sys.argv[1] == 'selftest':
        selftest()
    elif sys.argv[1] == 'w1':
        w1(sys.argv[2], int(sys.argv[3]), int(sys.argv[4]))
