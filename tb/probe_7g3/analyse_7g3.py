#!/usr/bin/env python3
"""sec 63 #7g-3 Phase 1 -- offline pairing for the probe_7g3 raw-event logs.

sec 22.92: the SV probes emit raw events only; every pairing, classification
and count happens here, and this file opens with a hand-derived known-answer
self-test that must pass before any real log is read.

Inputs are the per-instance files the probes write into a target's work dir:
  pr7g3_fs.<%m>.log   frame_symbols  I/T/B events         (C-7G3-5, G7-5)
  pr7g3_t2d.<%m>.log  tlp2dllp       W/S/K/P/M events     (C-7G3-1/2/3)
  pr7g3_dh.<%m>.log   data_handler   V/U/K/R/O events     (C-7G3-8/16, G7-7)
  pr7g3_lt.<%m>.log   LTSSM          L events             (C-7G3-16, G7-7)

Usage:
  analyse_7g3.py selftest
  analyse_7g3.py stack <work_dir>
  analyse_7g3.py fence <old_root> <new_root>
"""
import sys, os, re, glob, hashlib, collections

L0 = 0x00005
RECOVERY_BASE = 0x4          # every Recovery substate ends in ...0100 (low nibble 4)
STATE_NAMES = {
    0x00000: 'IDLE', 0x00005: 'L0', 0x000E3: 'CFG_IDLE',
    0x00021: 'DET_WAIT1MS', 0x00041: 'DET_QUIET', 0x00061: 'DET_ACTIVE', 0x00081: 'DET_RX',
    0x00022: 'POLL_ACTIVE', 0x00042: 'POLL_CFG', 0x00062: 'POLL_COMPL',
    0x00023: 'CFG_LW_START', 0x00043: 'CFG_LW_ACCEPT', 0x00063: 'CFG_LN_ACCEPT',
    0x00083: 'CFG_LN_WAIT', 0x000A3: 'CFG_COMPLETE',
}


def family(st):
    """LTSSM top-level family from the 20-bit encoding: the low 5 bits name
    the family (pcie_ltssm_downstream.sv:123-167)."""
    return {0: 'IDLE', 1: 'DETECT', 2: 'POLLING', 3: 'CONFIGURATION', 4: 'RECOVERY',
            5: 'L0', 6: 'L0s', 7: 'L1', 8: 'L2', 9: 'DISABLED', 10: 'LOOPBACK',
            11: 'HOT_RESET'}.get(st & 0x1F, f'?{st:05x}')


# --------------------------------------------------------------------------
# parsing
# --------------------------------------------------------------------------
def read_events(path):
    ev = []
    for ln in open(path):
        f = ln.split()
        if len(f) < 2:
            continue
        ev.append((f[0], int(f[1]), f[2:]))
    return ev


def ltssm_intervals(lt_events, t_end):
    """[(t0, t1, state)] from raw L events."""
    out = []
    for i, (_, t, f) in enumerate(lt_events):
        st = int(f[0], 16)
        t1 = lt_events[i + 1][1] if i + 1 < len(lt_events) else t_end
        out.append((t, t1, st))
    return out


def state_at(iv, t):
    for t0, t1, st in iv:
        if t0 <= t < t1:
            return st
    return iv[-1][2] if iv else None


# --------------------------------------------------------------------------
# data_handler + LTSSM analysis (C-7G3-8, C-7G3-16, G7-7)
# --------------------------------------------------------------------------
def analyse_dh(dh_ev, lt_ev, period=None):
    t_end = max([e[1] for e in dh_ev] + [e[1] for e in lt_ev] + [0]) + 1
    iv = ltssm_intervals(lt_ev, t_end)
    # valid runs from V edges
    vedges = [(t, int(f[0])) for tag, t, f in dh_ev if tag == 'V']
    if period is None:
        gaps = sorted(b[0] - a[0] for a, b in zip(vedges, vedges[1:]) if b[0] > a[0])
        period = gaps[0] if gaps else 1
    lows = []                      # (t_start, t_stop) of valid-low stretches
    for i, (t, v) in enumerate(vedges):
        if v == 0:
            t1 = vedges[i + 1][0] if i + 1 < len(vedges) else t_end
            lows.append((t, t1))
    # link_up
    uedges = [(t, int(f[0])) for tag, t, f in dh_ev if tag == 'U']
    first_up = next((t for t, v in uedges if v == 1), None)
    # valid-low per LTSSM family, AFTER first link-up (before it, pack_data is
    # held off by phy_link_up_i by design and there is no traffic)
    low_by_fam = collections.Counter()
    maxrun_by_fam = collections.Counter()
    if first_up is not None:
        for a, b in lows:
            a2 = max(a, first_up)
            if b <= a2:
                continue
            # split the stretch across LTSSM intervals
            for t0, t1, st in iv:
                lo, hi = max(a2, t0), min(b, t1)
                if hi > lo:
                    cyc = (hi - lo) // period
                    fam = family(st)
                    low_by_fam[fam] += cyc
                    maxrun_by_fam[fam] = max(maxrun_by_fam[fam], cyc)
    # frames: start K, end K, tlast-out
    kev = [(t, int(f[0]), f[1].lower()) for tag, t, f in dh_ev if tag == 'K']
    starts = [(t, b, s) for t, b, s in kev if s in ('5c', 'fb')]
    ends = [(t, b, s) for t, b, s in kev if s in ('fd', 'fe')]
    tlasts = [t for tag, t, f in dh_ev if tag == 'O' and f[1] == '1']
    align = collections.Counter((s, b) for t, b, s in starts)
    reg_arm = [(t, int(f[0])) for tag, t, f in dh_ev if tag == 'R']
    # END-in -> next tlast-out latency, and strand check: a valid-low stretch
    # LONGER than one cycle inside (END-in, tlast-out].  One-cycle lows are the
    # 16-bit PIPE's alternate-cycle cadence into pack_data.
    lat = collections.Counter()
    strands = []
    j = 0
    for te, b, s in ends:
        while j < len(tlasts) and tlasts[j] < te:
            j += 1
        if j >= len(tlasts):
            strands.append((te, None, 'NO_TLAST_AFTER'))
            continue
        to = tlasts[j]
        lat[(to - te) // period] += 1
        for a, bb in lows:
            if a < to and bb > te and (min(bb, to) - max(a, te)) // period > 1:
                strands.append((te, to, family(state_at(iv, te)) if iv else '?'))
                break
    fams_visited = []
    for t0, t1, st in iv:
        f = family(st)
        if not fams_visited or fams_visited[-1] != f:
            fams_visited.append(f)
    return {
        'period': period,
        'first_link_up': first_up,
        'ltssm_families_in_order': fams_visited,
        'recovery_visits': sum(1 for _, _, st in iv if family(st) == 'RECOVERY'),
        'valid_low_cycles_after_linkup_by_family': dict(low_by_fam),
        'max_valid_low_run_by_family': dict(maxrun_by_fam),
        'starts': len(starts), 'ends': len(ends), 'tlasts_out': len(tlasts),
        'start_alignment': dict(align),
        'registered_arm_firings': len(reg_arm),
        'registered_arm_bytes': dict(collections.Counter(b for _, b in reg_arm)),
        'end_to_tlast_latency_cycles': dict(sorted(lat.items())),
        'strands': strands,
    }


# --------------------------------------------------------------------------
# tlp2dllp (C-7G3-1/2/3)
# --------------------------------------------------------------------------
def analyse_t2d(ev):
    uw = next((int(f[0].split('=')[1]) for tag, t, f in ev if tag == 'W'), None)
    out = {'USER_WIDTH': uw}
    for tag in 'SKPM':
        vals = [int(f[0], 16) for tg, t, f in ev if tg == tag]
        out[tag] = {'beats': len(vals),
                    'values': dict(collections.Counter(f'{v:02x}' for v in vals)),
                    'bits43_nonzero': sum(1 for v in vals if v & 0x18)}
    return out


# --------------------------------------------------------------------------
# fence (C-7G3-5)
# --------------------------------------------------------------------------
def fence_summary(path):
    ev = read_events(path)
    b = sum(1 for e in ev if e[0] == 'B')
    tg = sum(1 for e in ev if e[0] == 'T')
    md5 = hashlib.md5(open(path, 'rb').read()).hexdigest()
    return b, tg, md5


def fence(old_root, new_root):
    rows = []
    olds = sorted(glob.glob(os.path.join(old_root, '**', 'pr7g3_fs.*.log'), recursive=True))
    for o in olds:
        rel = os.path.relpath(o, old_root)
        n = os.path.join(new_root, rel)
        bo, to, mo = fence_summary(o)
        if not os.path.exists(n):
            rows.append((rel, bo, to, mo, None, None, None, 'MISSING_NEW'))
            continue
        bn, tn, mn = fence_summary(n)
        rows.append((rel, bo, to, mo, bn, tn, mn, 'IDENTICAL' if mo == mn else 'DIFFER'))
    return rows


# --------------------------------------------------------------------------
# known-answer self-test (hand-derived; must pass before real data is read)
# --------------------------------------------------------------------------
def selftest():
    P = 8000
    # LTSSM: DETECT at 0, L0 from 100 cycles on
    lt = [('L', 0, ['00001']), ('L', 100 * P, ['00005'])]
    dh = [('V', 0, ['0']), ('U', 0, ['0']), ('U', 100 * P, ['1'])]
    # alternate-cycle valid from cycle 100 to 200
    for c in range(100, 200):
        dh.append(('V', c * P, ['1' if c % 2 == 0 else '0']))
    # frame 1: SDP at byte 2 cycle 110, END byte 1 at cycle 114, tlast out 117
    dh += [('K', 110 * P, ['2', '5c']), ('K', 114 * P, ['1', 'fd']), ('O', 117 * P, ['3', '1', '01'])]
    # frame 2: STP at byte 0 cycle 120, END byte 3 at 124, registered arm at 126, tlast 127
    dh += [('K', 120 * P, ['0', 'fb']), ('K', 124 * P, ['3', 'fd']), ('R', 126 * P, ['3', 'wc=3']),
           ('O', 127 * P, ['3', '1', '02'])]
    # valid stops (low for 50 cycles) from 200; frame 3 END at 198, tlast at 251 -> STRAND
    dh += [('K', 196 * P, ['2', '5c']), ('K', 198 * P, ['1', 'fd'])]
    dh.append(('V', 250 * P, ['1']))
    dh.append(('O', 251 * P, ['3', '1', '01']))
    dh.sort(key=lambda e: e[1])
    r = analyse_dh(dh, lt, period=P)
    exp = {
        'first_link_up': 100 * P,
        'recovery_visits': 0,
        'starts': 3, 'ends': 3, 'tlasts_out': 3,
        'start_alignment': {('5c', 2): 2, ('fb', 0): 1},
        'registered_arm_firings': 1,
        'end_to_tlast_latency_cycles': {3: 2, 53: 1},
    }
    for k, v in exp.items():
        assert r[k] == v, (k, r[k], v)
    # 49 one-cycle lows (odd cycles 101..197) + the stretch from cycle 199
    # (the last alternate low, which never rises) to 250 = 51 cycles
    assert r['valid_low_cycles_after_linkup_by_family'] == {'L0': 49 + 51}, r['valid_low_cycles_after_linkup_by_family']
    assert r['max_valid_low_run_by_family'] == {'L0': 51}, r['max_valid_low_run_by_family']
    assert len(r['strands']) == 1 and r['strands'][0][0] == 198 * P and r['strands'][0][2] == 'L0', r['strands']
    # tlp2dllp
    t = analyse_t2d([('W', 0, ['USER_WIDTH=5']), ('S', 1, ['1a', '0']), ('S', 2, ['02', '1']),
                     ('K', 3, ['02']), ('M', 4, ['02', '1'])])
    assert t['USER_WIDTH'] == 5 and t['S']['bits43_nonzero'] == 1 and t['M']['values'] == {'02': 1}, t
    print('SELFTEST PASS')


def main():
    if sys.argv[1] == 'selftest':
        selftest()
        return
    selftest()
    if sys.argv[1] == 'stack':
        wd = sys.argv[2]
        lts = {os.path.basename(p): read_events(p) for p in glob.glob(os.path.join(wd, 'pr7g3_lt.*.log'))}
        for p in sorted(glob.glob(os.path.join(wd, 'pr7g3_t2d.*.log'))):
            print('T2D', os.path.basename(p), analyse_t2d(read_events(p)))
        for p in sorted(glob.glob(os.path.join(wd, 'pr7g3_dh.*.log'))):
            name = os.path.basename(p)
            # the LTSSM of the SAME stack: longest common instance-path prefix
            inst = name[len('pr7g3_dh.'):]
            best = max(lts, key=lambda k: len(os.path.commonprefix([k[len('pr7g3_lt.'):], inst])))
            r = analyse_dh(read_events(p), lts[best])
            print('DH', name, 'LTSSM=', best)
            for k, v in r.items():
                if k == 'strands':
                    print('   strands:', len(v), v[:10])
                else:
                    print('  ', k, v)
    elif sys.argv[1] == 'fence':
        for row in fence(sys.argv[2], sys.argv[3]):
            print('FENCE', *row)


if __name__ == '__main__':
    main()
