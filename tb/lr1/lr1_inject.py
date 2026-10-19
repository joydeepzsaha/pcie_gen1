"""lr1_inject -- LR1-b: the far end's error injections and the records the rows assert on

Author: Kourosh Ghahramani
Silicon Systems Research Lab, University of Washington

LR1-b (BRIEF_LR1 D8, D9; RULINGS_LR1 F; RULINGS_LR1A 4) puts errors and
provocations on the link toward the RC. Each injection is one-shot and keyed
on a sequence number, so a row names exactly which packet it changes:

  knob              what the far end sends instead                         rule the RC must follow
  bad_lcrc_seq      its TLP with this sequence number, first transmission,  §3.5.3.1 p.183: discard, Nak
                    with one LCRC bit inverted
  seq_ahead_seq     that TLP under sequence number + 1                       p.183: out of sequence, Nak
  dup_seq           that TLP once more, unchanged, after its Ack             p.183: duplicate, discard, Ack
  no_end_seq        that TLP with its END replaced by a Logical Idle Symbol  p.196 (framing check optional),
                                                                             p.183 (LCRC): never accepted as sent
  rxerr_seq         that TLP with RxStatus `rxerr_code` (100b: 8b/10b        §4.2.4.6 p.209, §3.5.3.1 p.182:
                    decode error, PG239 Table 10) on the word carrying its   a Receiver Error; discard, Nak
                    Symbol `rxerr_sym`; the Symbol itself unchanged
  nak_rc_seq        a Nak for the RC's TLP with this sequence number, which  §3.5.2.1 p.174: replay
                    is discarded as if it arrived corrupted
  no_ack_rc_seq     no Ack for that TLP until the RC sends it again          p.175, Table 3-4 p.176: replay
  bad_crc_ack_seq   the first Ack covering that TLP, with one CRC bit        §3.5.2.2 p.179: discarded, so
                    inverted                                                 the same as no Ack
  rsvd_ack_seq      the first Ack covering that TLP, Reserved byte 1 = 01h,  p.179 "Non-zero values in
                    CRC over the bytes sent                                  Reserved fields are ignored"
  hold_cpl_rc_seq   the Completion of the RC's request carried in that TLP,  Completer timing (no bound in
                    `hold_cpl_cycles` late                                   2.1, Table 7-25 p.550 is the
                                                                             RC's own timeout)
  fc1_rsvd          the Reserved bits named, set in InitFC1 only             ruling F: a newer partner's
                    (byte 1 [7:6], byte 2 [5:4])                             scale fields
  unknown_dllp      one DLLP of this type byte (data 0) at DL_Init, before   p.179: an unsupported type is
                    the first InitFC1                                        discarded without action
  mps_supported     256: Device Capabilities Max_Payload_Size Supported      §7.8.3 p.504
                    001b, so 256-byte TLPs from the RC are legal

Sequence numbers start at 000h on each side (§3.5.2.1 p.170): the far end's
TLP 000h is its first Completion, the RC's TLP 000h its first Configuration
Request. A knob set to -1 is off.

Records kept for the rows, all at the pins (Symbol Times, t = 2 x cycle +
lane), besides those of lr1_farend and lr1_checks:
  rc_dllps     (t, parsed DLLP) of every DLLP the RC sent, CRC good or bad
  rc_tlps      (t, end_t, seq, first) of every TLP the RC sent
  rc_units     (t, kind) of every unit the RC sent after its first L0:
               'TS1' / 'TS2' / 'OS_BAD' (a malformed COM-led set) / 'SKP' /
               'DLLP' / 'TLP' / 'IDLE' (a run of Logical Idle) / 'FRAME_ERR'
               (a unit the framing rules do not allow, lr1_rx)
  fe_tlp_ends  (t, seq, replay) of every far-end TLP's last Symbol
  injected     (t, knob, detail) of each injection as it happened
"""
from dataclasses import dataclass, field

import lr1_cfg as CFG
import lr1_dll as DLL
import lr1_symbols as SYM
from lr1_farend import FarEnd, Knobs


class RxErrSym(int):
    """A data Symbol the far end's PHY reports with an RxStatus error code."""
    code = 0b100


@dataclass
class InjKnobs(Knobs):
    bad_lcrc_seq: int = -1
    seq_ahead_seq: int = -1
    dup_seq: int = -1
    no_end_seq: int = -1
    rxerr_seq: int = -1
    rxerr_code: int = 0b100
    rxerr_sym: int = 8
    nak_rc_seq: int = -1
    no_ack_rc_seq: int = -1
    bad_crc_ack_seq: int = -1
    rsvd_ack_seq: int = -1
    hold_cpl_rc_seq: int = -1
    hold_cpl_cycles: int = 0
    fc1_rsvd: dict = field(default_factory=dict)
    unknown_dllp: int = -1
    mps_supported: int = 128             # 256: Device Capabilities Max_Payload_Size Supported 001b (p.504)


def _seq_of_frame(frame):
    return ((frame[1][0] & 0x0F) << 8) | frame[2][0]


class InjDll(DLL.Dll):
    def __init__(self, knobs, log, cycle_fn):
        super().__init__(knobs, log, cycle_fn)
        self.injected = []
        self.done = set()
        self.dup_copy = None
        self.last_rx_seq = None
        self.acks_sent = []            # (cycle, seq) of every Ack the far end builds
        self.script = []               # a scripted burst: ('TLP', tlp bytes) / ('DLLP', name, 4 bytes), sent first
        self.script_sent = []          # (cycle, name) as each scripted packet is built
        self.hold_updatefc_p = None    # no UpdateFC-P of the far end's own before this cycle

    def _once(self, key, detail=''):
        if key in self.done:
            return False
        self.done.add(key)
        self.injected.append((self.now(), key, detail))
        self.log('INJECT|%d|%s|%s' % (self.now(), key, detail))
        return True

    # ------------------------------------------------------------- DL_Init
    def link_up(self, up):
        was = self.state
        injected, done = self.injected, self.done
        super().link_up(up)                                # DL_Inactive re-runs __init__: keep the record
        self.injected, self.done = injected, done
        if was == 'INACTIVE' and self.state == 'FC_INIT1' and self.k.unknown_dllp >= 0 and \
                self._once('unknown_dllp', 'type %02xh' % self.k.unknown_dllp):
            self.dllp_q.appendleft(('Unknown', bytes([self.k.unknown_dllp & 0xFF, 0, 0, 0])))

    def tick(self, cycle):
        if self.k.fc1_rsvd and self.state == 'FC_INIT1':
            if self.fc_next is not None and cycle >= self.fc_next:
                self._once('fc1_rsvd', 'InitFC1 sets with Reserved fields %s' % self.k.fc1_rsvd)
            saved = self.k.fc_rsvd
            self.k.fc_rsvd = self.k.fc1_rsvd
            try:
                super().tick(cycle)
            finally:
                self.k.fc_rsvd = saved
            return
        super().tick(cycle)

    # ------------------------------------------------------------- receive
    def on_tlp(self, ev):
        body = ev['body']
        if ev['end'] == 'END' and len(body) >= 2 and self.state == 'ACTIVE':
            seq = ((body[0] & 0x0F) << 8) | body[1]
            if seq == self.k.nak_rc_seq and seq == self.nrs and self._once('nak_rc', 'RC seq %03x' % seq):
                self._nak()
                return None
        t = super().on_tlp(ev)
        if t is not None:
            self.last_rx_seq = (self.nrs - 1) % 4096
            if self.last_rx_seq == self.k.no_ack_rc_seq and self._once('no_ack_rc', 'RC seq %03x' %
                                                                        self.last_rx_seq):
                self.ack_due = None
        return t

    def _acknak(self, kind, seq):
        super()._acknak(kind, seq)
        s = self.k.dup_seq
        if s >= 0 and self.dup_copy is not None and (self.ackd - s) % 4096 < 2048 and \
                self._once('dup', 'far-end seq %03x sent again after its Ack' % s):
            self.replay_q.append((s, self.dup_copy))

    # ------------------------------------------------------------ transmit
    def next_packet(self, cycle):
        if self.script and self.state == 'ACTIVE':
            item = self.script.pop(0)
            if item[0] == 'TLP':
                tlp = item[1]
                seq = self.nts
                self.nts = (self.nts + 1) % 4096
                self.retry.append((seq, tlp))
                self._spend(tlp)
                self.stats['tlp_tx'] += 1
                if self.replay_timer is None:
                    self.replay_timer = cycle
                self.script_sent.append((cycle, 'TLP-%03x' % seq))
                return 'TLP', SYM.frame_tlp(seq, tlp)
            self.script_sent.append((cycle, item[1]))
            return 'DLLP-' + item[1], SYM.frame_dllp(item[2])
        if self.hold_updatefc_p is not None and cycle < self.hold_updatefc_p:
            self.upd_now.discard('P')
            if 'P' in self.upd_due:
                self.upd_due['P'] = max(self.upd_due['P'], self.hold_updatefc_p)
        pkt = super().next_packet(cycle)
        if pkt is None:
            return None
        kind, frame = pkt
        frame = list(frame)
        if kind == 'TLP':
            seq = _seq_of_frame(frame)
            tlp = self.retry[-1][1]
            if seq == self.k.dup_seq:
                self.dup_copy = tlp
            if seq == self.k.bad_lcrc_seq and self._once('bad_lcrc', 'far-end seq %03x' % seq):
                b, k = frame[-2]
                frame[-2] = (b ^ 0x01, k)
            if seq == self.k.seq_ahead_seq and self._once('seq_ahead', 'far-end seq %03x sent as %03x'
                                                          % (seq, (seq + 1) % 4096)):
                frame = SYM.frame_tlp((seq + 1) % 4096, tlp)
            if seq == self.k.no_end_seq and self._once('no_end', 'far-end seq %03x' % seq):
                frame[-1] = (0x00, 0)
            if seq == self.k.rxerr_seq and self._once('rxerr', 'far-end seq %03x Symbol %d RxStatus %s'
                                                      % (seq, self.k.rxerr_sym, bin(self.k.rxerr_code))):
                b, k = frame[self.k.rxerr_sym]
                e = RxErrSym(b)
                e.code = self.k.rxerr_code
                frame[self.k.rxerr_sym] = (e, k)
        elif kind == 'Ack':
            b4 = bytes(b for b, _ in frame[1:5])
            aseq = ((b4[2] & 0x0F) << 8) | b4[3]
            self.acks_sent.append((cycle, aseq))
            for key, s in (('bad_crc_ack', self.k.bad_crc_ack_seq), ('rsvd_ack', self.k.rsvd_ack_seq)):
                if s >= 0 and key not in self.done and (aseq - s) % 4096 < 2048:
                    self._once(key, 'Ack %03x covering RC seq %03x' % (aseq, s))
                    if key == 'bad_crc_ack':
                        b, k = frame[-2]
                        frame[-2] = (b ^ 0x01, k)
                    else:
                        frame = SYM.frame_dllp(SYM.dllp_ack(aseq, rsvd1=0x01))
        return kind, frame


class InjCompleter(CFG.Completer):
    def __init__(self, knobs, log, cycle_fn, dll):
        super().__init__(knobs, log, cycle_fn)
        self.dll = dll
        if knobs.mps_supported == 256:
            self.cfg.val[0x4C] = (self.cfg.val[0x4C] & ~0x7) | 0x1

    def handle(self, t):
        out = super().handle(t)
        s = self.k.hold_cpl_rc_seq
        if s >= 0 and self.dll.last_rx_seq == s and out and 'hold_cpl' not in self.dll.done:
            self.dll._once('hold_cpl', 'Completion of RC seq %03x held %d cycles' % (s, self.k.hold_cpl_cycles))
            rel = self.now() + self.k.hold_cpl_cycles
            self.held.extend((rel, c) for c in out)
            return []
        return out


class InjFarEnd(FarEnd):
    def __init__(self, dut, knobs=None, log=None):
        super().__init__(dut, knobs or InjKnobs(), log)
        self.dll = InjDll(self.k, self.log, lambda: self.cycle)
        self.tl = InjCompleter(self.k, self.log, lambda: self.cycle, self.dll)
        self.dll.tl = self.tl
        self.rc_dllps = []
        self.rc_tlps = []
        self.rc_units = []
        self.fe_tlp_ends = []
        self.rxerr_words = []
        self.rc_idle_runs = []         # (t, n) of each RC Logical Idle run after its first L0
        self.fe_rec_idle = []          # (t, n) of the far end's idle Symbols in Recovery.Idle

    @property
    def injected(self):
        return self.dll.injected

    def _on_rc_event(self, ev):
        kind = ev['kind']
        t = ev['t']
        if kind == 'DLLP':
            self.rc_dllps.append((t, SYM.parse_dllp(ev['b6'])))
        elif kind == 'TLP':
            body = ev['body']
            seq = ((body[0] & 0x0F) << 8) | body[1] if len(body) >= 2 else None
            first = seq is not None and not any(s == seq for _, _, s, _ in self.rc_tlps)
            self.rc_tlps.append((t, ev.get('end_t'), seq, first))
        if self.chk.rc_l0_t is not None:
            if kind == 'TS':
                self.rc_units.append((t, ev['ts']))
            elif kind in ('OS_BAD', 'SKP', 'DLLP', 'TLP', 'FRAME_ERR'):
                self.rc_units.append((t, kind))
            elif kind == 'IDLE_RUN':
                self.rc_units.append((t, 'IDLE'))
                self.rc_idle_runs.append((t, ev['n']))
        super()._on_rc_event(ev)

    def _pop_word(self, t0):
        errs = [e for e in list(self.txq)[:2] if isinstance(e[0], RxErrSym)]
        out = super()._pop_word(t0)
        if errs:
            self.rxstatus_now = errs[0][0].code
            self.rxerr_words.append((t0, errs[0][0].code))
        return out

    def _tag(self, tag, t):
        if tag[0] == 'TLP_END':
            self.fe_tlp_ends.append((t, tag[1], tag[2]))
        elif tag[0] == 'IDLE' and self.ltssm.state == 'REC_IDLE':
            self.fe_rec_idle.append((t, tag[1]))
        super()._tag(tag, t)

    # ---------------------------------------------------- derived records
    def rc_acknaks(self, after=0):
        """(t, 'Ack' / 'Nak', seq) of each RC Ack / Nak with a good CRC at or after t."""
        return [(t, d['kind'], d['seq']) for t, d in self.rc_dllps
                if t >= after and d['crc_ok'] and d['kind'] in ('Ack', 'Nak')]

    def packets_between_training_sets(self, t0, t1):
        """RC DLLPs / TLPs that sit between two of its training sets (a SKP OS
        between them is allowed, p.201; a malformed COM-led set counts as a
        set), within [t0, t1]. Returns (count, list of (t, kind))."""
        u = [(t, k) for t, k in self.rc_units if t0 <= t <= t1]
        out = []
        last_ts = False
        pending = []
        for t, k in u:
            if k in ('TS1', 'TS2', 'OS_BAD'):
                if last_ts and pending:
                    out.extend(pending)
                last_ts = True
                pending = []
            elif k == 'SKP':
                continue
            elif k in ('DLLP', 'TLP'):
                if last_ts:
                    pending.append((t, k))
            elif k == 'IDLE':
                last_ts = False
                pending = []
        return len(out), out

    def recovery_packets(self, t0, t1):
        """The RC's packets in a Recovery that starts at t0 (its first training
        set after L0) and is over by t1, against §4.2.4.1 p.201 (training
        sequences are interrupted only by SKP Ordered Sets) and §4.2.6.4.4
        p.246 (Recovery.Idle sends Idle data; L0 only after 8 consecutive idle
        Symbol Times are received and 16 idle Symbols sent after receiving
        one). Returns (packets between two training sets, packets after the
        last training set but before both Recovery.Idle conditions held,
        a dict of the times used)."""
        n_between, between = self.packets_between_training_sets(t0, t1)
        ts = [t for t, k in self.rc_units if t0 <= t <= t1 and k in ('TS1', 'TS2', 'OS_BAD')]
        info = dict(first_ts=ts[0] if ts else None, last_ts=ts[-1] if ts else None)
        if not ts:
            return between, [], info
        last_end = ts[-1] + 15
        fe_idle = [t for t, n in self.fe_rec_idle if t >= t0]
        info['fe_first_idle'] = fe_idle[0] if fe_idle else None
        # the far end's 8th idle Symbol reaching the RC: two Symbols a word, one word a clock
        info['fe_idle8'] = fe_idle[0] + 8 if fe_idle else None
        pk = [(t, k) for t, k in self.rc_units if last_end < t <= t1 and k in ('DLLP', 'TLP')]
        early = []
        if pk:
            tp = pk[0][0]
            sent = sum(max(0, min(t + n, tp) - max(t, info['fe_first_idle'] or tp))
                       for t, n in self.rc_idle_runs if last_end < t < tp)
            info.update(first_pkt=tp, rc_idle_after_rx=sent)
            if info['fe_idle8'] is None or tp < info['fe_idle8'] or sent < 16:
                early = pk[:1]
        return between, early, info
