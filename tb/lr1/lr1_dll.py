"""lr1_dll -- the far end's Data Link Layer, from Base 2.1 chapter 3 and §2.6.1

Author: Kourosh Ghahramani
Silicon Systems Research Lab, University of Washington

  DL_Inactive -> DL_Init on LinkUp (§3.2.1 p.158). FC_INIT1: the InitFC1 set,
  P, NP, Cpl in that order, repeated at least every 34 us; FI1 once values for
  P, NP and Cpl are recorded from InitFC1 or InitFC2 (§3.3.1 p.161). FC_INIT2:
  the InitFC2 set; FI2 on any InitFC2, UpdateFC or TLP; then DL_Active
  (pp.161-162).

  Transmit side (§3.5.2.1 pp.170-178): NEXT_TRANSMIT_SEQ from 000h, ACKD_SEQ
  FFFh, a retry buffer, REPLAY_TIMER (Table 3-4 p.176), replay of every
  unacknowledged TLP on a Nak or on expiry, in order. An Ack or Nak whose
  sequence number is neither ACKD_SEQ nor an unacknowledged TLP's is a Data
  Link Protocol Error (p.180) -- recorded, the DLLP discarded.

  Receive side (§3.5.3.1 pp.182-186): a TLP ending in EDB with the inverted
  LCRC is nullified and silently discarded (p.183); a bad LCRC, or a sequence
  number ahead of NEXT_RCV_SEQ, schedules a Nak; a duplicate is discarded and
  Acked; an expected one is accepted and Acked. Ack / Nak carry
  NEXT_RCV_SEQ - 1.

  Flow control (§2.6.1 pp.137-143): the far end advertises its credits in
  InitFC; consumes them for each TLP it accepts; releases them when its
  Transaction Layer has processed the TLP (at once); and schedules an UpdateFC
  when a header type had no credit left, or a data type had less than
  Max_Payload_Size, and units were released (p.142), plus at least every
  30 us per non-infinite type in L0 (p.143). It obeys the RC's advertised
  limits for the TLPs it sends.

Timing is in clock cycles of 8 ns (2 Symbol Times each).
"""
from collections import deque

import lr1_consts as C
import lr1_crc as CRC
import lr1_symbols as SYM
import lr1_tlp as TLP

US = 125
FIELD_MOD = {'H': 1 << C.FC_HDR_BITS, 'D': 1 << C.FC_DATA_BITS}


def tlp_credits(fields):
    """(type, header credits, data credits) a TLP consumes (§2.6.1 p.139)."""
    name = fields.get('name', '')
    ndw = len(fields.get('payload', b'')) // 4
    data = (ndw + 3) // 4
    if name in ('MWr', 'Msg', 'MsgD'):
        return 'P', 1, data
    if name in ('Cpl', 'CplD'):
        return 'CPL', 1, data
    return 'NP', 1, data           # MRd, IORd/Wr, CfgRd/Wr (writes carry one data credit)


class Dll:
    def __init__(self, knobs, log, cycle_fn):
        self.k = knobs
        self.log = log
        self.now = cycle_fn
        self.state = 'INACTIVE'
        # receiver accounting (what the far end advertises)
        self.adv = dict(knobs.adv_credits)              # {'P': (h, d), 'NP': (h, d), 'CPL': (h, d)}
        self.alloc = {t: list(v) for t, v in self.adv.items()}
        self.rcvd = {t: [0, 0] for t in self.adv}
        # transmitter accounting (what the RC advertised)
        self.recorded = {}                               # type -> (h, d) during FC_INIT1
        self.limit = {}
        self.consumed = {t: [0, 0] for t in C.FC_TYPES}
        self.fi1 = self.fi2 = False
        # sequence numbers
        self.nts = 0
        self.ackd = 0xFFF
        self.nrs = 0
        self.retry = deque()                             # (seq, tlp)
        self.replay_q = deque()
        self.replay_timer = None                         # cycle the timer started
        self.replay_num = 0
        self.nak_scheduled = False
        self.ack_due = None                              # cycle by which an Ack must go
        self.ack_seq = None
        self.dllp_q = deque()                            # (kind, b4)
        self.tlp_q = deque()                             # TLPs from the Transaction Layer
        self.fc_next = None                              # cycle the next Init set is due
        self.upd_due = {}                                # type -> cycle of the next periodic UpdateFC
        self.upd_now = set()
        self.tl = None                                   # the Transaction Layer (lr1_cfg.Completer)
        self.errors = []                                 # (cycle, text): Data Link Protocol Errors etc.
        self.history = [(0, 'INACTIVE')]
        self.stats = dict(tlp_rx=0, tlp_tx=0, ack_tx=0, nak_tx=0, replays=0, updatefc_tx=0, initfc_tx=0)
        self.fc_checked = 0
        self.nullify_left = knobs.nullify_count       # V18
        self.null_pending = False
        self.last_ack_tx = None                       # V8
        self.ack_lat = []                             # cycles from a TLP's END to the far end's Ack
        self.ack_first_end = None
        self.cpl0_due = None                          # V5: UpdateFC-Cpl 0/0

    # ------------------------------------------------------------------ state
    def _goto(self, st):
        if st != self.state:
            self.log('DLL|%d|%s->%s' % (self.now(), self.state, st))
            self.state = st
            self.history.append((self.now(), st))

    def link_up(self, up):
        if up and self.state == 'INACTIVE':
            self._goto('FC_INIT1')
            self.fc_next = self.now() + self.k.fc_init_start_delay
        elif not up and self.state != 'INACTIVE':
            tl, errors = self.tl, self.errors
            self.__init__(self.k, self.log, self.now)      # DL_Inactive: reset all state (p.158)
            self.tl, self.errors = tl, errors

    def active(self):
        return self.state == 'ACTIVE'

    # ---------------------------------------------------------------- receive
    def on_dllp(self, d):
        """d: lr1_symbols.parse_dllp() of a DLLP from the RC with a good CRC."""
        kind = d['kind']
        if self.state == 'INACTIVE':
            return
        if kind in ('InitFC1', 'InitFC2') and self.state == 'FC_INIT1':
            if d['vc'] == 0:
                self.recorded[d['fc']] = (d['hdr'], d['data'])
                if all(t in self.recorded for t in C.FC_TYPES):
                    self.fi1 = True
                    self.limit = {t: list(v) for t, v in self.recorded.items()}
                    self._goto('FC_INIT2')
                    self.fc_next = self.now() + self.k.fc2_first_delay_cycles
                    if self.k.drop_initfc_set_on_flag:
                        self.dllp_q = deque(x for x in self.dllp_q if not x[0].startswith('InitFC1'))
            return
        if self.state == 'FC_INIT2' and ((kind == 'InitFC2' and d['vc'] == 0) or kind == 'UpdateFC'):
            self._fi2()
        if self.state != 'ACTIVE':
            return
        if kind == 'UpdateFC' and d['vc'] == 0:
            self.limit[d['fc']] = [d['hdr'], d['data']]
        elif kind in ('Ack', 'Nak'):
            self._acknak(kind, d['seq'])

    def _fi2(self):
        self.fi2 = True
        self._goto('ACTIVE')
        if self.k.stop_initfc2_on_fi2:
            self.dllp_q = deque(x for x in self.dllp_q if not x[0].startswith('InitFC2'))
        t = self.now()
        for typ, (h, dd) in self.adv.items():
            if h or dd:
                self.upd_due[typ] = t + self.k.updatefc_period
        if self.k.updatefc_cpl_zero_period:
            self.cpl0_due = t

    def _unacked(self):
        return (self.nts - 1 - self.ackd) % C.SEQ_MOD

    def _acknak(self, kind, seq):
        n_out = self._unacked()
        dist = (seq - self.ackd) % C.SEQ_MOD          # 0 = ACKD_SEQ, 1..n_out = an unacked TLP
        if dist > n_out:
            self.errors.append((self.now(), '%s seq %03x is neither ACKD_SEQ %03x nor unacknowledged (NTS %03x)'
                                % (kind, seq, self.ackd, self.nts)))
            return
        if dist:
            while self.retry and ((self.retry[0][0] - self.ackd - 1) % C.SEQ_MOD) < dist:
                self.retry.popleft()
            self.ackd = seq
            self.replay_num = 0
            self.replay_timer = self.now() if self.retry else None
        if kind == 'Nak':
            self._start_replay()

    def _start_replay(self):
        if self.retry:
            self.replay_q = deque(self.retry)
            self.stats['replays'] += 1
            self.replay_num = (self.replay_num + 1) % 4
        self.replay_timer = self.now() if self.retry else None

    def on_tlp(self, ev):
        """ev: an lr1_rx 'TLP' event. Returns the parsed TLP if accepted."""
        if self.state == 'FC_INIT2':
            self._fi2()
        if self.state != 'ACTIVE':
            return None                                  # DL_Down: discard, do not Ack (p.158)
        body = ev['body']
        if ev['end'] == 'EDB':
            if CRC.lcrc_nullified(body):
                return None                              # nullified: silently discarded (p.183)
            self._nak()
            return None
        if not CRC.lcrc_ok(body):
            self._nak()
            return None
        seq = ((body[0] & 0x0F) << 8) | body[1]
        if seq == self.nrs:
            self.nrs = (self.nrs + 1) % C.SEQ_MOD
            self.nak_scheduled = False
            self._schedule_ack(ev)
            tlp = TLP.parse(body[2:-4])
            self.stats['tlp_rx'] += 1
            self._consume(tlp)
            return tlp
        if (self.nrs - seq) % C.SEQ_MOD <= 2048:
            self._schedule_ack(ev)                       # duplicate (p.184)
            return None
        self._nak()
        return None

    def _schedule_ack(self, ev):
        if self.ack_due is None:
            self.ack_due = ev['end_t'] // 2 + self.k.ack_delay_cycles
            self.ack_first_end = ev['end_t'] // 2
        self.ack_seq = (self.nrs - 1) % C.SEQ_MOD

    def _nak(self):
        if not self.nak_scheduled:
            self.nak_scheduled = True
            self.dllp_q.appendleft(('Nak', SYM.dllp_nak((self.nrs - 1) % C.SEQ_MOD)))
            self.stats['nak_tx'] += 1

    # -------------------------------------------------------------- credits
    def _consume(self, tlp):
        """CREDITS_RECEIVED, and the optional Receiver Overflow check (p.142):
        (CREDITS_ALLOCATED - CREDITS_RECEIVED) mod 2^n >= 2^n / 2 after this TLP.
        A violation is recorded (the RC sent beyond the far end's credit); the
        TLP is still processed so the row can continue."""
        typ, h, d = tlp_credits(tlp)
        r = self.rcvd[typ]
        r[0] = (r[0] + h) % FIELD_MOD['H']
        r[1] = (r[1] + d) % FIELD_MOD['D']
        ah, ad = self.adv[typ]
        self.fc_checked += 1
        for i, (adv, key) in enumerate(((ah, 'H'), (ad, 'D'))):
            if adv and (self.alloc[typ][i] - r[i]) % FIELD_MOD[key] >= FIELD_MOD[key] // 2:
                self.errors.append((self.now(), 'Receiver overflow: %s%s credits received %d, allocated %d'
                                    % (typ, key, r[i], self.alloc[typ][i])))

    def release(self, tlp):
        """The Transaction Layer has processed `tlp`: free its credits and
        schedule the UpdateFC that p.142 requires."""
        typ, h, d = tlp_credits(tlp)
        ah, ad = self.adv[typ]
        if ah == 0 and ad == 0:
            return                                       # infinite: nothing to update
        a = self.alloc[typ]
        avail_h = (a[0] - self.rcvd[typ][0] + h) % FIELD_MOD['H']   # before this TLP was counted
        avail_d = (a[1] - self.rcvd[typ][1] + d) % FIELD_MOD['D']
        mps_units = self.k.max_payload // 16
        need = (ah and (avail_h - h) % FIELD_MOD['H'] == 0) or (ad and (avail_d - d) % FIELD_MOD['D'] < mps_units)
        a[0] = (a[0] + (h if ah else 0)) % FIELD_MOD['H']
        a[1] = (a[1] + (d if ad else 0)) % FIELD_MOD['D']
        if need or self.k.updatefc_every_release:
            self.upd_now.add(typ)

    def can_send(self, tlp):
        typ, h, d = tlp_credits(TLP.parse(tlp))
        lim = self.limit.get(typ)
        if lim is None:
            return False
        c = self.consumed[typ]
        okh = lim[0] == 0 and not self._init_nonzero(typ, 0) or \
            (lim[0] - (c[0] + h)) % FIELD_MOD['H'] <= FIELD_MOD['H'] // 2
        okd = (lim[1] == 0 and not self._init_nonzero(typ, 1)) or d == 0 or \
            (lim[1] - (c[1] + d)) % FIELD_MOD['D'] <= FIELD_MOD['D'] // 2
        return okh and okd

    def _init_nonzero(self, typ, i):
        return self.recorded.get(typ, (0, 0))[i] != 0

    def _spend(self, tlp):
        typ, h, d = tlp_credits(TLP.parse(tlp))
        c = self.consumed[typ]
        c[0] = (c[0] + h) % FIELD_MOD['H']
        c[1] = (c[1] + d) % FIELD_MOD['D']

    # --------------------------------------------------------------- transmit
    def send_tlp(self, tlp):
        self.tlp_q.append(bytes(tlp))

    def tick(self, cycle):
        if self.state in ('FC_INIT1', 'FC_INIT2') and self.fc_next is not None and cycle >= self.fc_next:
            ph = 'InitFC1' if self.state == 'FC_INIT1' else 'InitFC2'
            build = SYM.initfc1 if ph == 'InitFC1' else SYM.initfc2
            for t in C.FC_TYPES:
                h, d = self.adv[t]
                self.dllp_q.append((ph + '-' + t, build(t, h, d, **self.k.fc_rsvd)))
                self.stats['initfc_tx'] += 1
            self.fc_next = cycle + self.k.fc_set_period
        if self.state == 'ACTIVE':
            for typ, due in self.upd_due.items():
                if cycle >= due:
                    self.upd_now.add(typ)
            if self.cpl0_due is not None and cycle >= self.cpl0_due:
                # Cpl is advertised infinite: an UpdateFC may still be sent, with
                # zero credit fields (p.138)
                self.dllp_q.append(('UpdateFC-CPL0', SYM.updatefc('CPL', 0, 0)))
                self.stats['updatefc_tx'] += 1
                self.cpl0_due = cycle + self.k.updatefc_cpl_zero_period
            if self.replay_timer is not None and cycle - self.replay_timer >= self.k.replay_timer_cycles:
                self._start_replay()

    def next_packet(self, cycle):
        """The next DLLP or TLP to transmit, as (kind, Symbol list), by the
        priority of the implementation note in §3.5.2.1: Nak, Ack, FC, replay,
        new TLPs."""
        if self.dllp_q and self.dllp_q[0][0] == 'Nak':
            k, b4 = self.dllp_q.popleft()
            return k, SYM.frame_dllp(b4)
        if self.ack_due is not None and cycle >= self.ack_due and self.state == 'ACTIVE':
            seq = self.ack_seq
            self.ack_due = None
            self.stats['ack_tx'] += 1
            b4 = SYM.dllp_ack(seq, **self.k.ack_rsvd)
            self.last_ack_tx = cycle
            self.ack_lat.append(cycle - self.ack_first_end)
            return 'Ack', SYM.frame_dllp(b4)
        if self.dllp_q:
            k, b4 = self.dllp_q.popleft()
            return k, SYM.frame_dllp(b4)
        if self.state == 'ACTIVE' and self.upd_now:
            typ = sorted(self.upd_now)[0]
            self.upd_now.discard(typ)
            h, d = self.alloc[typ]
            self.upd_due[typ] = cycle + self.k.updatefc_period
            self.stats['updatefc_tx'] += 1
            return 'UpdateFC-' + typ, SYM.frame_dllp(SYM.updatefc(typ, h, d))
        if self.state != 'ACTIVE':
            return None
        if self.replay_q:
            seq, tlp = self.replay_q.popleft()
            if self.replay_timer is None:
                self.replay_timer = cycle
            return 'TLP-replay', SYM.frame_tlp(seq, tlp)
        if self.tlp_q and self.can_send(self.tlp_q[0]) and self._unacked() < 2047:
            if self.nullify_left and not self.null_pending:
                # V18: send the TLP nullified first -- EDB and the inverted LCRC, under
                # NEXT_TRANSMIT_SEQ, which is not incremented, and no retry copy (p.171,
                # p.174); the TLP itself follows
                self.null_pending = True
                self.nullify_left -= 1
                self.stats['nullified'] = self.stats.get('nullified', 0) + 1
                return 'TLP-null', SYM.frame_tlp(self.nts, self.tlp_q[0], nullify=True)
            self.null_pending = False
            tlp = self.tlp_q.popleft()
            seq = self.nts
            self.nts = (self.nts + 1) % C.SEQ_MOD
            self.retry.append((seq, tlp))
            self._spend(tlp)
            self.stats['tlp_tx'] += 1
            if self.replay_timer is None:
                self.replay_timer = cycle
            return 'TLP', SYM.frame_tlp(seq, tlp, nullify=False)
        if self.k.redundant_ack_period and (self.last_ack_tx is None or
                                            cycle - self.last_ack_tx >= self.k.redundant_ack_period):
            # V8: an Ack naming ACKD_SEQ -- FFFh before any TLP -- is not an error (p.180)
            self.last_ack_tx = cycle
            self.stats['ack_tx'] += 1
            seq = (self.nrs - 1) % C.SEQ_MOD
            self.stats['redundant_ack'] = self.stats.get('redundant_ack', 0) + 1
            if seq == 0xFFF:
                self.stats['redundant_ack_fff'] = self.stats.get('redundant_ack_fff', 0) + 1
            return 'Ack-redundant', SYM.frame_dllp(SYM.dllp_ack(seq))
        return None

    def pending(self):
        return bool(self.dllp_q or self.tlp_q or self.replay_q or self.upd_now or
                    (self.ack_due is not None))
