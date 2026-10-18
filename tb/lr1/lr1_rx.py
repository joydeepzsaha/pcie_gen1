"""lr1_rx -- the far end's receiver: the RC's Symbol stream into ordered sets and packets

Author: Kourosh Ghahramani
Silicon Systems Research Lab, University of Washington

The far end samples phy_txdata / phy_txdatak at the seam (lr1_phy) and feeds
each Symbol here in wire order. RcStream descrambles it (lr1_scrambler) and
delineates it:

  ordered sets   COM-led: TS1 / TS2 (16 Symbols, Tables 4-2 / 4-3), SKP OS
                 (COM + 1..5 SKP, §4.2.7.2 p.262), EIOS (Table 4-4), FTS OS
  packets        STP ... END / EDB (TLP), SDP + 6 + END (DLLP) -- §4.2.2
                 pp.195-196, Figures 4-6 / 4-7 p.197
  Logical Idle   data Symbols outside both; 00h after descrambling (p.195)

Each unit is reported to the listeners as an event dict with the index of its
first Symbol in the stream ('sym') and its time 't' in Symbol Times at the pins (2 x cycle + byte lane). Symbols
before the first COM are discarded (no Symbol lock yet). Anything the framing
rules do not allow is reported as kind 'FRAME_ERR' with a reason; the stream
then resynchronises at the next COM, STP or SDP.

Nothing here reads the RTL; the rules are the spec's.
"""
import lr1_consts as C
import lr1_scrambler as SCR
import lr1_symbols as SYM


class RcStream:
    def __init__(self):
        self.rx = SCR.RxDescrambler()
        self.listeners = []
        self.sym = 0                 # index of the next Symbol
        self.locked = False
        self.mode = 'IDLE'           # IDLE | OS | TLP | DLLP
        self.buf = []                # raw (byte, k) of the unit in progress
        self.plain = []              # descrambled data bytes of a packet
        self.start = None            # (sym, t) of the unit in progress
        self.idle_run = 0            # consecutive Logical Idle Symbols
        self.idle_start = None
        self.t = 0

    def on(self, fn):
        self.listeners.append(fn)

    def _emit(self, ev):
        for fn in self.listeners:
            fn(ev)

    def _end_idle(self):
        if self.idle_run:
            self._emit(dict(kind='IDLE_RUN', n=self.idle_run, sym=self.idle_start[0], t=self.idle_start[1]))
            self.idle_run = 0

    def feed(self, byte, k, t):
        self.t = t
        p, k, in_ts = self.rx.feed(byte, k)
        idx = self.sym
        self.sym += 1
        if not self.locked:
            if k and byte == C.COM:
                self.locked = True
            else:
                return
        self._step(byte, k, p, in_ts, idx, t)

    # ------------------------------------------------------------------
    def _begin(self, mode, byte, k, idx, t):
        self.mode = mode
        self.buf = [(byte, k)]
        self.plain = []
        self.start = (idx, t)

    def _step(self, raw, k, p, in_ts, idx, t):
        if self.mode == 'IDLE':
            self._dispatch(raw, k, p, idx, t)
            return
        if self.mode == 'OS':
            self._os_step(raw, k, p, idx, t)
            return
        if self.mode == 'DLLP':
            if k:
                if raw == C.END and len(self.plain) == 6:
                    self._emit(dict(kind='DLLP', b6=bytes(self.plain), sym=self.start[0], t=self.start[1]))
                    self.mode = 'IDLE'
                    return
                self._frame_err('DLLP interrupted by K %02x after %d bytes' % (raw, len(self.plain)))
                self.mode = 'IDLE'
                self._dispatch(raw, k, p, idx, t)
                return
            self.plain.append(p)
            if len(self.plain) > 6:
                self._frame_err('DLLP longer than 6 bytes before END')
                self.mode = 'IDLE'
            return
        if self.mode == 'TLP':
            if k:
                if raw in (C.END, C.EDB):
                    self._emit(dict(kind='TLP', body=bytes(self.plain), end='END' if raw == C.END else 'EDB',
                                    sym=self.start[0], t=self.start[1], end_sym=idx, end_t=t))
                    self.mode = 'IDLE'
                    return
                self._frame_err('TLP interrupted by K %02x after %d bytes' % (raw, len(self.plain)))
                self.mode = 'IDLE'
                self._dispatch(raw, k, p, idx, t)
                return
            self.plain.append(p)
            if len(self.plain) > 4 + 2 + 4096 + 16 + 4:
                self._frame_err('TLP longer than any legal TLP')
                self.mode = 'IDLE'

    def _dispatch(self, raw, k, p, idx, t):
        """A Symbol at a unit boundary."""
        if not k:
            if self.idle_run == 0:
                self.idle_start = (idx, t)
            self.idle_run += 1
            if p != 0x00:
                self._emit(dict(kind='IDLE_BAD', value=p, sym=idx, t=t))
            if self.idle_run == C.RX_CONSECUTIVE_IDLE:
                self._emit(dict(kind='IDLE8', sym=idx, t=t))
            return
        self._end_idle()
        if raw == C.COM:
            self._begin('OS', raw, k, idx, t)
        elif raw == C.STP:
            self._begin('TLP', raw, k, idx, t)
        elif raw == C.SDP:
            self._begin('DLLP', raw, k, idx, t)
        else:
            self._frame_err('K %s outside a packet or ordered set' % C.K_NAMES.get(raw, '%02x' % raw), idx, t)

    def _os_step(self, raw, k, p, idx, t):
        b = self.buf
        b.append((raw, k))
        s1 = b[1]
        if s1 == (C.SKP, 1):
            if (raw, k) == (C.SKP, 1) and len(b) <= 1 + C.SKP_RX_MAX:
                return
            if (raw, k) == (C.SKP, 1):
                self._frame_err('SKP OS with more than %d SKP' % C.SKP_RX_MAX)
                self.mode = 'IDLE'
                return
            n = len(b) - 2
            self._emit(dict(kind='SKP', n=n, sym=self.start[0], t=self.start[1]))
            self.mode = 'IDLE'
            self._dispatch(raw, k, p, idx, t)
            return
        if s1 in ((C.IDL, 1), (C.FTS, 1)):
            want = s1[0]
            if (raw, k) != (want, 1) and len(b) <= 4:
                self._frame_err('%s OS broken at Symbol %d' % (C.K_NAMES[want], len(b) - 1))
                self.mode = 'IDLE'
                self._dispatch(raw, k, p, idx, t)
                return
            if len(b) == 4:
                self._emit(dict(kind='EIOS' if want == C.IDL else 'FTS', sym=self.start[0], t=self.start[1]))
                self.mode = 'IDLE'
            return
        # a training set: Symbols 1-2 are PAD or data, 3-15 data (Tables 4-2 / 4-3)
        pos = len(b) - 1
        bad = (k and raw == C.COM) or (k and not (pos in (1, 2) and raw == C.PAD))
        if bad:
            self._emit(dict(kind='OS_BAD', syms=list(b[:-1]), sym=self.start[0], t=self.start[1],
                            reason='K %02x at Symbol %d' % (raw, pos)))
            self.mode = 'IDLE'
            self._dispatch(raw, k, p, idx, t)
            return
        if len(b) == C.TS_LEN:
            f = _parse_ts(b)
            if f is None:
                self._emit(dict(kind='OS_BAD', syms=list(b), sym=self.start[0], t=self.start[1],
                                reason='not a TS1 / TS2'))
            else:
                f.update(kind='TS', sym=self.start[0], t=self.start[1])
                self._emit(f)
            self.mode = 'IDLE'

    def _frame_err(self, reason, idx=None, t=None):
        s = self.start if idx is None else (idx, t)
        self._emit(dict(kind='FRAME_ERR', reason=reason, sym=s[0], t=s[1]))


def _parse_ts(syms):
    f = SYM.parse_ts(syms)
    if f is None:
        return None
    return dict(ts=f['kind'], link=f['link'], lane=f['lane'], n_fts=f['n_fts'], rate=f['rate'], tc=f['tc'],
                inverted=f['inverted'])
