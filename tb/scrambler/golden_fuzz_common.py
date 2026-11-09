"""Spec-golden fuzz of gen1_scramble -- the model, the stimulus and the classes behind verilate_scrambler_golden.

Every expected value is COMPUTED FROM THE SPECIFICATION (Base 2.1 sec 4.2.3 p.199) by golden() below; none is
captured from the DUT.  The rules, as skpseed_common.py quotes them: the COM initializes the LFSR to FFFFh; the
LFSR advances eight serial shifts for every Symbol except SKP; K Symbols are not scrambled; TS1/TS2 Symbols are not
scrambled (sec 4.2.4.1 p.201); every other data Symbol is XORed with the LFSR's output.  advance() and mask() are
skpseed_common.py's, which reuses byte_scramble.sv's Rung-3-proven transition table; that file is IMPORTED and
never modified (verilate_scrambler_skpseed depends on it).

THE STIMULUS is deterministic: build(seed * 1000 + width, width, mode) and the idle-clock stream Random(seed * 7 + 1)
are the LR2 group C Phase 0 fuzz's (fuzz2.py) unchanged, so a seed here is the same stream there.  Units are TS1,
TS2 (PAD or numbered Link/Lane), SKP Ordered Sets of 1-5 SKP, Logical Idle runs, DLLPs and TLPs; a random leading
offset puts the first COM in every byte position.  Modes: 'train' (TS and SKP only), 'l0' (no TS), 'mix'.

THE CLASSES are read from the golden stream's structure alone (classify()):
  ts_post_skp   a TS Symbol of the first TS after a SKP Ordered Set (only Logical Idle between): #29's class
  d_skp_com     a data Symbol after a SKP Ordered Set sharing its word with the next COM, that COM opening a TS
  d_ts_com      a data Symbol right after a TS's 16th Symbol sharing its word with the next COM
                (the 16-Symbol end arm's class when that COM opens a TS)
  d_skp_word    a data Symbol in the word of a SKP Ordered Set's last SKP
  d_pre_skpcom  a data Symbol in the word of, and before, a COM that opens a SKP Ordered Set
  ts_other, d_other, k: everything else
"""
import random
import collections
import bisect

from skpseed_common import advance, mask

COM, SKP, PAD, SDP, STP, END = 0xBC, 0x1C, 0xF7, 0x5C, 0xFB, 0xFD
DEPTH = 3   # a word presented on valid clock j is published on valid clock j + DEPTH (gen1_scramble's stage 3)


def unit(kind, rng):
    if kind in ('TS1', 'TS2'):
        tid = 0x4A if kind == 'TS1' else 0x45
        pad = rng.random() < 0.3
        s = [(COM, 1, 0)]
        s += [(PAD, 1, 1), (PAD, 1, 1)] if pad else [(0x00, 0, 1), (rng.choice([0, 1, 2, 3]), 0, 1)]
        s += [(0xFF, 0, 1), (0x02, 0, 1), (0x00, 0, 1)] + [(tid, 0, 1)] * 10
        return s
    if kind == 'SKP':
        return [(COM, 1, 0)] + [(SKP, 1, 0)] * rng.randint(1, 5)
    if kind == 'IDLE1':
        return [(0x00, 0, 0)]
    if kind == 'IDLE':
        return [(0x00, 0, 0)] * rng.randint(1, 9)
    if kind == 'DLLP':
        return [(SDP, 1, 0)] + [(rng.randrange(256), 0, 0) for _ in range(6)] + [(END, 1, 0)]
    if kind == 'TLP':
        return [(STP, 1, 0)] + [(rng.randrange(256), 0, 0) for _ in range(rng.randint(10, 30))] + [(END, 1, 0)]


def golden(syms):
    """Base 2.1 sec 4.2.3 p.199, Symbol by Symbol."""
    lfsr, out = 0xFFFF, []
    for b, k, ts in syms:
        if k:
            out.append(b)
            if b == COM:
                lfsr = 0xFFFF
            elif b != SKP:
                lfsr = advance(lfsr)
        else:
            out.append(b if ts else b ^ mask(lfsr))
            lfsr = advance(lfsr)
    return out


def build(seed, width, mode):
    """fuzz2.py's generator, unchanged. Returns (syms, tags, off); a tag is (unit, kind, pos_in_unit, prev_kind)."""
    rng = random.Random(seed)
    syms, tags = [], []
    off = rng.randint(0, width // 8 - 1)
    for _ in range(off):
        syms.append((0x00, 0, 0)); tags.append((-1, 'FILL', 0, None))
    prev = None
    for ui in range(60):
        if mode == 'train':
            kind = rng.choice(['TS1', 'TS1', 'TS2', 'SKP'])
        elif mode == 'l0':
            kind = rng.choice(['SKP', 'IDLE', 'DLLP', 'TLP', 'IDLE'])
        else:
            kind = rng.choice(['TS1', 'TS2', 'SKP', 'SKP', 'IDLE', 'DLLP', 'TLP'])
            if prev == 'SKP' and rng.random() < 0.5:
                kind = rng.choice(['TS1', 'IDLE1'])
        if ui == 0:
            kind = 'TS1'
        for i, s in enumerate(unit(kind, rng)):
            syms.append(s); tags.append((ui, kind, i, prev))
        prev = kind
    syms, tags = pad_flush(syms, tags, width, prev)
    return syms, tags, off


def pad_flush(syms, tags, width, prev=None):
    nb = width // 8
    while len(syms) % nb:
        syms.append((0x00, 0, 0)); tags.append((-1, 'FILL', 0, prev))
    for _ in range(4 * nb):
        syms.append((0x00, 0, 0)); tags.append((-1, 'FLUSH', 0, prev))
    return syms, tags


def directed(width):
    """Exhaustive small cases: every COM byte, SKP counts 1-5, 0-4 Logical Idle Symbols between a SKP Ordered Set
    and the next TS, and 1-4 between two TSs. Returns a list of (name, syms, tags)."""
    out = []

    def ts(kind='TS1'):
        tid = 0x4A if kind == 'TS1' else 0x45
        return [(COM, 1, 0)] + [(0x00, 0, 1), (0x00, 0, 1), (0xFF, 0, 1), (0x02, 0, 1), (0x00, 0, 1)] + [(tid, 0, 1)] * 10

    for off in range(width // 8):
        for n in range(1, 6):
            for m in range(0, 5):
                units = [('FILLU', [(0, 0, 0)] * off), ('TS1', ts()), ('TS1', ts()),
                         ('SKP', [(COM, 1, 0)] + [(SKP, 1, 0)] * n), ('IDLE', [(0, 0, 0)] * m),
                         ('TS2', ts('TS2')), ('TS1', ts()), ('IDLE', [(0, 0, 0)] * 6)]
                out.append(('skp_off%d_n%d_m%d' % (off, n, m),) + flatten(units, width))
        for m in range(1, 5):
            units = [('FILLU', [(0, 0, 0)] * off), ('TS1', ts()), ('IDLE', [(0, 0, 0)] * m), ('TS2', ts('TS2')),
                     ('IDLE', [(0, 0, 0)] * m), ('TS1', ts()), ('IDLE', [(0, 0, 0)] * 6)]
            out.append(('end_off%d_m%d' % (off, m),) + flatten(units, width))
    return out


def flatten(units, width):
    syms, tags, prev, ui = [], [], None, 0
    for kind, u in units:
        if not u:
            continue
        if kind == 'FILLU':
            for s in u:
                syms.append(s); tags.append((-1, 'FILL', 0, None))
            continue
        for i, s in enumerate(u):
            syms.append(s); tags.append((ui, kind, i, prev))
        prev = kind; ui += 1
    return pad_flush(syms, tags, width, prev)


def classify(syms, tags, width):
    """One class per Symbol from the first COM on, and a coverage key for the classes that have one."""
    nb = width // 8
    n = len(syms)
    cls, cov = ['pre'] * n, [None] * n
    first = next(i for i, s in enumerate(syms) if s[0] == COM and s[1])
    units = collections.OrderedDict()
    for i, t in enumerate(tags):
        if t[0] >= 0:
            units.setdefault(t[0], []).append(i)
    order = list(units)
    pos = {u: j for j, u in enumerate(order)}
    kind = {u: tags[units[u][0]][1] for u in order}
    com_pos = [i for i, s in enumerate(syms) if s[1] and s[0] == COM]

    def next_com(i):
        j = bisect.bisect_right(com_pos, i)
        return com_pos[j] if j < len(com_pos) else None

    for i in range(first, n):
        b, k, ts = syms[i]
        t = tags[i]
        if t[1] in ('FILL', 'FLUSH'):
            cls[i] = 'fill'
        elif k:
            cls[i] = 'k'
        elif ts:
            u = t[0]
            com_i = units[u][0]
            j = pos[u] - 1
            while j >= 0 and kind[order[j]] in ('IDLE', 'IDLE1'):
                j -= 1
            if j >= 0 and kind[order[j]] == 'SKP':
                cls[i] = 'ts_post_skp'
                cov[i] = (com_i % nb, len(units[order[j]]) - 1)
            else:
                cls[i] = 'ts_other'
        else:
            pcs = [c for c in com_pos if c < i]
            pu = tags[pcs[-1]][0] if pcs else None
            pk = kind.get(pu)
            pend = units[pu][-1] if pu is not None else None
            nc = next_com(i)
            nk = kind.get(tags[nc][0]) if nc is not None else None
            same = nc is not None and nc // nb == i // nb
            if pk == 'SKP' and same and nk in ('TS1', 'TS2') and i // nb in (pend // nb, pend // nb + 1):
                cls[i] = 'd_skp_com'
                cov[i] = (nc % nb, len(units[pu]) - 1)
            elif same and nk == 'SKP':
                cls[i] = 'd_pre_skpcom'
            elif pk in ('TS1', 'TS2') and same and i == pend + 1:
                cls[i] = 'd_ts_com'
                cov[i] = (nc % nb, nk)
            elif pk == 'SKP' and pend is not None and i // nb == pend // nb:
                cls[i] = 'd_skp_word'
            else:
                cls[i] = 'd_other'
    return cls, cov, first


def lines_for(syms, width, gap, seed):
    """One (valid, k, data) per clock: every word of syms in order, with idle clocks (valid low, random k/data)
    before a word with probability `gap` each, from Random(seed * 7 + 1) as in fuzz2.py."""
    rng = random.Random(seed * 7 + 1)
    nb = width // 8
    out = []
    for w in range(0, len(syms), nb):
        while gap and rng.random() < gap:
            out.append((0, rng.randrange(16), rng.randrange(1 << 32)))
        d = k = 0
        for j, (b, kk, _) in enumerate(syms[w:w + nb]):
            d |= b << (8 * j); k |= kk << j
        out.append((1, k, d))
    return out


def per_symbol(published, width, nwords):
    """Published words (one per valid clock) -> output byte per input Symbol (word m is published DEPTH valid clocks
    later)."""
    nb = width // 8
    res = []
    for m in range(nwords):
        if m + DEPTH >= len(published):
            break
        o = published[m + DEPTH]
        for j in range(nb):
            res.append((o >> (8 * j)) & 0xFF)
    return res
