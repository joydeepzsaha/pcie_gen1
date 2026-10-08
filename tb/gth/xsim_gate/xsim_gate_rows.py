#!/usr/bin/env python3
"""xsim_gate_rows.py -- the xsim gate's analysis: one artifact line per row.

Author: Kourosh Ghahramani
Silicon Systems Research Lab, University of Washington

Usage
  xsim_gate_rows.py selftest
      Known-answer checks of every row rule, each with a negative control.
      Exits 1 on any failure. xsim_gate.sh runs it before reading a real log.
  xsim_gate_rows.py rcf <vc> <work_dir> <tree> <out_f> <out_provenance>
      Writes rc_top.f: every .v/.sv source named in the staged .vc, in .vc
      order, as absolute paths. Hash-matches each against the file of the same
      name in <tree>. Exits 1 on any mismatched or unresolved file.
  xsim_gate_rows.py artifact <pcie_ltssm_downstream.sv> <log_dir>
      Prints the artifact: one line per row of ROWS, in that order.

Artifact line
  <row>|<PASS or FAIL>|<final LTSSM state>|<L0 time>|<FC init time>
  Times are integer ps of simulation time, and '-' means none. The final state
  is the state at the log's END line, named from the ST_* encoding parsed from
  the tree's own pcie_ltssm_downstream.sv: 'X' if unknown, 'NOLOG' if the log
  is missing. Rows that are not a training use the fields as follows:
    gt_site  <row>|<verdict>|<GT site(s) of the generated IP>|-|-

The benches print raw lines (EV|t|name|hex, END|t|reason, CFG|0|key|value,
VREL, PULSE, R2, XCHK, XU; see each bench's header). Every rule reads those
lines only. It never reads xsim's exit code: xsim 2023.2 exits 0 on a kernel
FATAL_ERROR, so a log containing FATAL, or one with no END line, fails its row.
"""
import collections
import hashlib
import os
import re
import sys

# (row, log basename): zcu102_pulse is the second training in zcu102_rel's log.
ROWS = [('loop', 'loop'), ('commafree', 'commafree'), ('swap', 'swap'),
        ('zcu102_rel', 'zcu102_rel'), ('zcu102_pulse', 'zcu102_rel'),
        ('zcu102_hold', 'zcu102_hold'), ('zcu102_r2', 'zcu102_r2'),
        ('gt_site', 'loop')]

# sec 63 #23: lane 0 on FMC HPC1 DP5 = GTHE4_CHANNEL_X0Y9, the bit of the GT
# Wizard's channel map and its master channel index.
GT_SITE = 9

# The signal names each bench prints.
LOOP_SIG = {'ltssm': 'ltssm_state', 'fc': 'fc_initialized', 'rst': 'rc_rst_i'}
ZCU_SIG = {'ltssm': 'ila_pclk.ltssm', 'fc': 'ila_pclk.fc_init', 'rst': 'rc_rst_i'}

# G0's R4_v2: the two Gen3 block-framing outputs nothing drives may read
# unknown; the six never-driven outputs are the positive control.
R4_ALLOWED = {'phy_txstart_block', 'phy_txsync_header'}
R4_CONTROL = 6


def ltssm_names(path):
    """ST_<NAME> = 20'b<bits> from pcie_ltssm_downstream.sv -> {code: NAME}."""
    names = {}
    for name, bits in re.findall(r"\bST_(\w+)\s*=\s*20'b([01]+)", open(path).read()):
        names[int(bits, 2)] = name
    return names


class Log:
    def __init__(self, text):
        self.ev = []
        self.cfg = {}
        self.end = None
        self.vrel = []
        self.pulse = []
        self.r2 = []
        self.xchk = []
        self.xu = []
        self.fatal = 'FATAL' in text
        for line in text.splitlines():
            p = line.split('|')
            try:
                if p[0] == 'EV' and len(p) == 4:
                    self.ev.append((int(p[1]), p[2], p[3]))
                elif p[0] == 'CFG' and len(p) == 4:
                    self.cfg[p[2]] = p[3]
                elif p[0] == 'END' and len(p) >= 3 and self.end is None:
                    self.end = (int(p[1]), p[2])
                elif p[0] == 'VREL' and len(p) == 4:
                    self.vrel.append(int(p[1]))
                elif p[0] == 'PULSE' and len(p) == 3:
                    self.pulse.append((int(p[1]), p[2]))
                elif p[0] == 'R2' and len(p) == 4:
                    self.r2.append((int(p[1]), p[2], p[3]))
                elif p[0] == 'XCHK' and len(p) == 6:
                    kv = dict(f.split('=') for f in p[3:])
                    self.xchk.append((int(p[1]), p[2], {k: int(v) for k, v in kv.items()}))
                elif p[0] == 'XU' and len(p) == 4:
                    self.xu.append((int(p[1]), p[2], p[3]))
            except ValueError:
                continue                      # a stray line that only looks like ours

    def values(self, sig):
        return [(t, v) for t, n, v in self.ev if n == sig]

    def first(self, sig, pred, after=-1):
        """First time > after at which sig takes a value satisfying pred."""
        return next((t for t, v in self.values(sig) if t > after and pred(v)), None)

    def at(self, sig, t):
        """The value sig holds at time t (its last change at or before t)."""
        v = None
        for tt, vv in self.values(sig):
            if tt <= t:
                v = vv
        return v


def code(v):
    return None if v is None or re.search(r'[xXzZ]', v) else int(v, 16) & 0xFFFFF


def state_name(names, v):
    c = code(v)
    if c is None:
        return 'X'
    return names.get(c, '0x%05x' % c)


def gt_sites(v):
    """The set bits of a hex channel map, as sorted indices; None if unknown."""
    if v is None or re.search(r'[xXzZ]', v):
        return None
    n = int(v, 16)
    return [b for b in range(n.bit_length()) if n >> b & 1]


def r4_ok(log):
    """R4_v2: at least one XCHK; every one with the control count; only allowed names unknown."""
    return (bool(log.xchk) and all(d.get('ctl_unknown') == R4_CONTROL for _, _, d in log.xchk)
            and all(name in R4_ALLOWED for _, _, name in log.xu))


def evaluate(row, log, names):
    """-> (verdict, final, l0, fc) for one row."""
    if log is None:
        return False, 'NOLOG', None, None
    if row == 'gt_site':
        en = next((v for t, n, v in log.ev if n == 'gt_channel_enable' and t == 0), None)
        mi = next((v for t, n, v in log.ev if n == 'gt_master_channel_idx' and t == 0), None)
        bits, master = gt_sites(en), gt_sites(mi)
        sites = 'X' if bits is None else ('+'.join('X0Y%d' % b for b in bits) or '-')
        master_idx = None if master is None else int(mi, 16)
        ok = ((not log.fatal) and log.end is not None and bits == [GT_SITE] and master_idx == GT_SITE)
        return ok, sites, None, None
    sig = ZCU_SIG if row.startswith('zcu102') else LOOP_SIG
    l0c = next(c for c, n in names.items() if n == 'L0')
    is_l0 = lambda v: code(v) == l0c
    is1 = lambda v: v == '1'
    is0 = lambda v: v == '0'
    base = (not log.fatal) and log.end is not None
    final = state_name(names, log.at(sig['ltssm'], log.end[0])) if log.end else 'X'
    reason = log.end[1] if log.end else ''

    if row in ('loop', 'swap'):
        l0 = log.first(sig['ltssm'], is_l0)
        fc = log.first(sig['fc'], is1)
        ok = (base and log.cfg.get('FAREND') == row and reason.startswith('fc_initialized+')
              and l0 is not None and fc is not None and l0 < fc and final == 'L0')
        if row == 'swap':
            ok = ok and any(n == 'farend_swap' and v == '1' and t == 0 for t, n, v in log.ev)
        return ok, final, l0, fc

    if row == 'commafree':
        l0 = log.first(sig['ltssm'], is_l0)
        fc = log.first(sig['fc'], is1)
        ok = (base and log.cfg.get('FAREND') == 'commafree' and reason == 'max_time'
              and any(n == 'farend_commafree' for _, n, _ in log.ev) and l0 is None and fc is None)
        return ok, final, l0, fc

    if row == 'zcu102_rel':
        vrel = log.vrel[0] if len(log.vrel) == 1 else None
        p0 = next((t for t, v in log.pulse if v == '0'), None)
        if vrel is None or p0 is None:
            return False, final, None, None
        held = [v for t, v in log.values(sig['rst']) if 1 <= t <= vrel]
        l0 = log.first(sig['ltssm'], is_l0, vrel)
        fc = log.first(sig['fc'], is1, vrel)
        ok = (base and log.cfg.get('PERST_REL_US') == '20' and vrel == 20_000_000
              and reason.startswith('second fc_initialized+')
              and log.first(sig['ltssm'], is_l0) == l0 and log.first(sig['fc'], is1) == fc
              and bool(held) and all(v == '1' for v in held)
              and l0 is not None and fc is not None and l0 < fc < p0 and r4_ok(log))
        return ok, final, l0, fc

    if row == 'zcu102_pulse':
        p0 = next((t for t, v in log.pulse if v == '0'), None)
        p1 = next((t for t, v in log.pulse if v == '1'), None)
        if p0 is None or p1 is None:
            return False, final, None, None
        fall = log.first(sig['fc'], is0, p0)
        l0 = log.first(sig['ltssm'], is_l0, p1)
        fc = log.first(sig['fc'], is1, fall) if fall is not None else None
        ok = (base and reason.startswith('second fc_initialized+') and fall is not None
              and l0 is not None and fc is not None and fall < l0 < fc and final == 'L0' and r4_ok(log))
        return ok, final, l0, fc

    if row == 'zcu102_hold':
        rst = [v for t, v in log.values(sig['rst']) if t >= 1]
        l0 = log.first(sig['ltssm'], is_l0)
        fc = log.first(sig['fc'], is1)
        ok = (base and log.cfg.get('PERST_REL_US') == '1000' and reason == 'MAX_US' and not log.vrel
              and bool(rst) and all(v == '1' for v in rst) and l0 is None and fc is None)
        return ok, final, l0, fc

    if row == 'zcu102_r2':
        ev = {(what, v): t for t, what, v in log.r2}
        stop, rel = ev.get(('pclk_ce', '0')), ev.get(('pclk_ce', 'released'))
        w0 = ev.get(('perst_n', '0'))
        if stop is None or rel is None or w0 is None:
            return False, final, None, None
        assert_t = log.first(sig['rst'], is1, w0)       # the asynchronous assert, PCLK stopped
        fall = log.first(sig['fc'], is0, stop)
        l0 = log.first(sig['ltssm'], is_l0, fall) if fall is not None else None
        fc = log.first(sig['fc'], is1, fall) if fall is not None else None
        ok = (base and log.cfg.get('R2') == '1' and reason.startswith('R2 fc_initialized+')
              and assert_t is not None and assert_t < rel and fall is not None
              and l0 is not None and fc is not None and l0 < fc and final == 'L0' and r4_ok(log))
        return ok, final, l0, fc

    raise ValueError('no rule for row ' + row)


def line(row, result):
    ok, final, l0, fc = result
    return '%s|%s|%s|%s|%s' % (row, 'PASS' if ok else 'FAIL', final,
                               '-' if l0 is None else l0, '-' if fc is None else fc)


def artifact(sv, log_dir):
    names = ltssm_names(sv)
    for row, base in ROWS:
        p = os.path.join(log_dir, base + '.log')
        log = Log(open(p, errors='replace').read()) if os.path.isfile(p) else None
        print(line(row, evaluate(row, log, names)))


def rcf(vc, work, tree, out_f, out_prov):
    files = []
    for tok in open(vc).read().split():
        if tok.endswith(('.v', '.sv')) and not tok.startswith(('-', '+')):
            files.append(os.path.normpath(tok if os.path.isabs(tok) else os.path.join(work, tok)))
    md5 = lambda p: hashlib.md5(open(p, 'rb').read()).hexdigest()
    by = collections.defaultdict(list)
    for d, _, fs in os.walk(tree):
        if os.path.relpath(d, tree).split(os.sep)[0] == 'build':
            continue                          # the staged copies themselves
        for f in fs:
            if f.endswith(('.v', '.sv')):
                p = os.path.join(d, f)
                by[f].append((md5(p), os.path.relpath(p, tree)))
    rows, matched, bad = [], 0, 0
    for p in files:
        m = md5(p)
        cands = by.get(os.path.basename(p), [])
        hit = [r for cm, r in cands if cm == m]
        if hit:
            matched += 1
            rows.append('%s  %s' % (m, hit[0]))
        else:
            bad += 1
            rows.append('%s  %s  %s' % (m, os.path.basename(p), 'MISMATCH' if cands else 'UNRESOLVED'))
    open(out_f, 'w').write('\n'.join(files) + '\n')
    open(out_prov, 'w').write('# files=%d matched=%d bad=%d\n' % (len(files), matched, bad) + '\n'.join(rows) + '\n')
    print('RCF files=%d matched=%d bad=%d' % (len(files), matched, bad))
    return 0 if bad == 0 and files else 1


# ---------------------------------------------------------------------------
# self-test: synthetic logs in the benches' own line format
# ---------------------------------------------------------------------------
NAMES = {0x0: 'IDLE', 0x22: 'POLLING_ACTIVE', 0x42: 'POLLING_CONFIGURATION', 0x5: 'L0'}


def _loop(farend='loop', end='END|89000000|fc_initialized+10000000', l0=True, fc=True, extra=()):
    s = ['CFG|0|FAREND|%s' % farend, 'EV|0|ltssm_state|0xxxxx', 'EV|110001|ltssm_state|000000',
         'EV|36024405|ltssm_state|000022']
    if farend == 'swap':
        s.append('EV|0|farend_swap|1')
    if farend == 'commafree':
        s.append('EV|36016405|farend_commafree|1')
    if l0:
        s.append('EV|44440405|ltssm_state|000005')
    if fc:
        s.append('EV|78512405|fc_initialized|1')
    s += list(extra)
    if end:
        s.append(end)
    return '\n'.join(s)


def _zcu(rel=20, pulse=True, xu=('phy_txstart_block',), ctl=6, held_drop=False, r2=False, end=None):
    s = ['CFG|0|PERST_REL_US|%d' % rel, 'CFG|0|R2|%d' % (1 if r2 else 0),
         'EV|1|rc_rst_i|1', 'EV|1|ila_pclk.ltssm|0xxxxx', 'EV|1|ila_pclk.fc_init|0']
    if held_drop:
        s.append('EV|10000000|rc_rst_i|0')
    t0 = rel * 1_000_000
    s += ['VREL|%d|perst_n|1' % t0, 'EV|%d|rc_rst_i|0' % (t0 + 18_000_000),
          'EV|%d|ila_pclk.ltssm|000005' % (t0 + 39_440_405), 'EV|%d|ila_pclk.fc_init|1' % (t0 + 73_512_405),
          'XCHK|%d|fc_init|checked=109|unknown=%d|ctl_unknown=%d' % (t0 + 73_512_405, len(xu), ctl)]
    s += ['XU|%d|fc_init|%s' % (t0 + 73_512_405, n) for n in xu]
    if r2:
        s += ['R2|98512405|pclk_ce|0', 'R2|99512405|perst_n|0', 'EV|99513123|rc_rst_i|1',
              'R2|99712405|perst_n|1', 'R2|100712405|pclk_ce|released', 'EV|101985205|ila_pclk.fc_init|0',
              'EV|101985205|ila_pclk.ltssm|000000', 'EV|139120405|ila_pclk.ltssm|000005',
              'EV|173192405|ila_pclk.fc_init|1']
        s.append(end or 'END|183192405|R2 fc_initialized+10000000')
    elif pulse:
        s += ['PULSE|98512405|0', 'EV|98520405|ila_pclk.ltssm|000000', 'EV|98520405|ila_pclk.fc_init|0',
              'PULSE|108512405|1', 'EV|147864405|ila_pclk.ltssm|000005', 'EV|181936405|ila_pclk.fc_init|1']
        s.append(end or 'END|191936405|second fc_initialized+10000000')
    return '\n'.join(s)


def _hold(release=False, drop=False):
    s = ['CFG|0|PERST_REL_US|1000', 'EV|0|rc_rst_i|x', 'EV|1|rc_rst_i|1', 'EV|1|ila_pclk.ltssm|0xxxxx',
         'EV|110001|ila_pclk.ltssm|000000']
    if drop:
        s.append('EV|50000000|rc_rst_i|0')
    if release:
        s.append('VREL|60000000|perst_n|1')
    s.append('END|100000000|MAX_US')
    return '\n'.join(s)


def _gt(bit, master, also=None):
    n = 1 << bit | (0 if also is None else 1 << also)
    return ('EV|0|gt_channel_enable|%048x' % n, 'EV|0|gt_master_channel_idx|%s' % master)


def selftest():
    ev = lambda row, text: evaluate(row, Log(text), NAMES)
    cases = [
        ('loop_pass', line('loop', ev('loop', _loop())), 'loop|PASS|L0|44440405|78512405'),
        ('loop_no_end_fails', ev('loop', _loop(end=None))[0], False),
        ('loop_fatal_fails', ev('loop', _loop(extra=('FATAL_ERROR: Vivado Simulator kernel',)))[0], False),
        ('loop_no_fc_fails', ev('loop', _loop(fc=False, end='END|400000000|max_time'))[0], False),
        ('loop_wrong_farend_fails', ev('loop', _loop(farend='swap'))[0], False),
        ('swap_pass', line('swap', ev('swap', _loop(farend='swap'))), 'swap|PASS|L0|44440405|78512405'),
        ('swap_without_marker_fails', ev('swap', _loop(farend='swap').replace('EV|0|farend_swap|1', ''))[0], False),
        ('swap_no_l0_fails', ev('swap', _loop(farend='swap', l0=False, fc=False, end='END|1000000000|max_time'))[0], False),
        ('commafree_pass', line('commafree', ev('commafree', _loop('commafree', 'END|200000000|max_time', False, False))),
         'commafree|PASS|POLLING_ACTIVE|-|-'),
        ('commafree_l0_fails', ev('commafree', _loop('commafree', 'END|200000000|max_time', True, False))[0], False),
        ('commafree_no_switch_fails',
         ev('commafree', _loop('commafree', 'END|200000000|max_time', False, False).replace('farend_commafree', 'zz'))[0], False),
        ('rel_pass', line('zcu102_rel', ev('zcu102_rel', _zcu())), 'zcu102_rel|PASS|L0|59440405|93512405'),
        ('rel_r4_extra_x_fails', ev('zcu102_rel', _zcu(xu=('phy_txstart_block', 'link_up_o')))[0], False),
        ('rel_r4_control_fails', ev('zcu102_rel', _zcu(ctl=5))[0], False),
        ('rel_not_held_fails', ev('zcu102_rel', _zcu(held_drop=True))[0], False),
        ('rel_wrong_release_fails', ev('zcu102_rel', _zcu(rel=0))[0], False),
        ('pulse_pass', line('zcu102_pulse', ev('zcu102_pulse', _zcu())), 'zcu102_pulse|PASS|L0|147864405|181936405'),
        ('pulse_no_second_fails', ev('zcu102_pulse', _zcu(end='END|400000000|MAX_US').replace(
            'EV|181936405|ila_pclk.fc_init|1', ''))[0], False),
        ('hold_pass', line('zcu102_hold', ev('zcu102_hold', _hold())), 'zcu102_hold|PASS|IDLE|-|-'),
        ('hold_released_fails', ev('zcu102_hold', _hold(release=True))[0], False),
        ('hold_reset_drop_fails', ev('zcu102_hold', _hold(drop=True))[0], False),
        ('r2_pass', line('zcu102_r2', ev('zcu102_r2', _zcu(rel=0, r2=True))), 'zcu102_r2|PASS|L0|139120405|173192405'),
        ('r2_no_async_assert_fails', ev('zcu102_r2', _zcu(rel=0, r2=True).replace('EV|99513123|rc_rst_i|1', ''))[0], False),
        ('r2_no_retrain_fails', ev('zcu102_r2', _zcu(rel=0, r2=True).replace('EV|173192405|ila_pclk.fc_init|1', ''))[0], False),
        ('gt_site_pass', line('gt_site', ev('gt_site', _loop(extra=_gt(9, '00000009')))), 'gt_site|PASS|X0Y9|-|-'),
        ('gt_site_quad130_fails', line('gt_site', ev('gt_site', _loop(extra=_gt(12, '0000000c')))),
         'gt_site|FAIL|X0Y12|-|-'),
        ('gt_site_master_fails', ev('gt_site', _loop(extra=_gt(9, '0000000c')))[0], False),
        ('gt_site_two_bits_fails', line('gt_site', ev('gt_site', _loop(extra=_gt(9, '00000009', also=8)))),
         'gt_site|FAIL|X0Y8+X0Y9|-|-'),
        ('gt_site_missing_fails', line('gt_site', ev('gt_site', _loop())), 'gt_site|FAIL|X|-|-'),
        ('gt_site_no_end_fails', ev('gt_site', _loop(end=None, extra=_gt(9, '00000009')))[0], False),
        ('nolog_fails', line('loop', evaluate('loop', None, NAMES)), 'loop|FAIL|NOLOG|-|-'),
        ('unknown_state_is_X', state_name(NAMES, '0xxxxx'), 'X'),
        ('unlisted_state_is_hex', state_name(NAMES, '000041'), '0x00041'),
    ]
    bad = 0
    for name, got, want in cases:
        ok = got == want
        bad += not ok
        print('SELFTEST|%s|%s|got=%r|want=%r' % ('PASS' if ok else 'FAIL', name, got, want))
    print('SELFTEST|%s|%d cases' % ('ALL_PASS' if bad == 0 else 'FAILED', len(cases)))
    return 1 if bad else 0


def main(a):
    if a[:1] == ['selftest']:
        return selftest()
    if a[:1] == ['rcf'] and len(a) == 6:
        return rcf(*a[1:])
    if a[:1] == ['artifact'] and len(a) == 3:
        artifact(*a[1:])
        return 0
    sys.stderr.write(__doc__)
    return 2


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
