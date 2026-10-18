"""lr1_checks -- the far end checks what the RC transmits (BRIEF_LR1 D6)

Author: Kourosh Ghahramani
Silicon Systems Research Lab, University of Washington

Every check reads only the seam: the RC's Symbol stream (lr1_rx events), the
PIPE command pins, ltssm_debug_state (the RC's own state report, used only to
open the FC-init window) and the far end's own transmit timeline. Each check
counts what it examined, so a row can prove it was not vacuous (§22.82), and
records a flag per violation:

  id                  rule                                                     cite                     grade
  TS_FIELDS           TS Symbols 1-5: Link / Lane PAD or this link's numbers;  Tables 4-2/4-3 pp.201-205 K
                      2.5 GT/s supported; speed_change 0; Training Control 0
  POLL_TS1_COUNT      >= 1024 TS1 (PAD/PAD) before Polling.Configuration       §4.2.6.2.1 p.220          K
  TS2_AFTER_RX        >= 16 TS2 sent after receiving one (Polling.Config.,     p.225, p.235              R (lenient: counted
                      Config.Complete)                                                                    from the far end's
                                                                                                          drive cycle)
  PKT_BEFORE_L0       no packet before 16 idle Symbols sent after receiving    §4.2.6.3.6 p.237          R
                      one, and 8 received
  TS_INTERRUPT        TS sequences interrupted only by SKP Ordered Sets        §4.2.4.1 p.201            K
  SKP_FORMAT          COM + 3 SKP                                              §4.2.7.1 p.261            K
  SKP_GAP_TRAINING /  SKP OS scheduled every 1180-1538 Symbol Times in every   §4.2.7.1 pp.261-262       R (MindShare
  SKP_GAP_L0          non-EI state; a unit in progress may defer it                                       pp.391-392 agrees)
  SKP_GAP_SHORT       no gap below 1180 less one ordered set                    §4.2.7.1 p.261           R
  IDLE_DATA           Logical Idle is 00h                                      §4.2.2 p.195              K
  FRAMING             STP/SDP ... END framing; no stray K code; whole TS       §4.2.2 pp.195-196         K
  DLLP_CRC            every DLLP's CRC                                         §3.4.1 p.167              K
  DLLP_FIELDS         a defined type (Table 3-1), VC0, Reserved fields 0       p.165, p.164, p.32        K
  LCRC                every TLP's LCRC (and not nullified)                     §3.5.2.1 p.171            K
  SEQ                 sequence numbers from 000h, +1 per new TLP               §3.5.2.1 p.170            K
  REPLAY              a replay only after a Nak or REPLAY_TIMER (>= 711        §3.5.2.1 pp.174-176       K
                      Symbol Times, -0%), same content, oldest first
  TLP_FORMAT          size matches Length; Configuration Requests TC0,         §2.2.7 p.79, p.57         K
                      Attr 0, AT 0, Length 1, Last BE 0; Reserved bits 0
  ACK_LATENCY         an Ack within Table 3-6 (237 Symbol Times at MPS 128,    §3.5.3.1 pp.186-187       K
                      x1) of a TLP's last Symbol, plus any packet in progress
  ACK_SEQ             Ack / Nak name an outstanding TLP or ACKD_SEQ            §3.5.2.1 p.180            K
  FC_INIT_ORDER       InitFC1 / InitFC2 sets in the order P, NP, Cpl           §3.3.1 pp.161-162         K
  FC_INIT_PERIOD      a set at least every 34 us (time in Configuration and    §3.3.1 p.161              K
                      Recovery excluded)
  FC_UPDATE_PERIOD    UpdateFC per non-infinite type at least every 30 us      §2.6.1.2 p.143            K
                      (+50%) in L0
  FC_OVERFLOW         no TLP beyond the credits the far end advertised         §2.6.1.2 p.142            K
  PIN_RXPOLARITY      phy_rxpolarity stays 0 (no inverted Symbols are sent)    §4.2.4.4 p.208            R
  PIN_TX_EI_L0        phy_txelecidle stays 0 from L0 on                        §4.2.6.5 p.247            R
  N3 (logged only)    phy_txelecidle 0 during receiver detection               §4.3.5.7 p.301            R
"""
import lr1_consts as C
import lr1_crc as CRC
import lr1_symbols as SYM
import lr1_tlp as TLP

US = 125


class Checks:
    def __init__(self, knobs, log):
        self.k = knobs
        self.log = log
        self.flags = []                 # dict(id, t, detail)
        self.notes = []                 # logged, not failed
        self.n = {}                     # check id -> count examined
        self.stats = {}                 # measured values, for the record
        # ordered sets
        self.first_ts1_t = None
        self.poll_ts1 = 0
        self.poll_done = False
        self.last_unit = None           # ('TS' | 'SKP' | other, event)
        self.after_ts = None            # the non-SKP thing that followed a TS
        self.fe_first_ts2_pad = None    # far-end drive time (Symbol Times) of its first PAD TS2
        self.fe_first_ts2_num = None
        self.rc_ts2_pad_after = 0
        self.rc_ts2_num_after = 0
        self.ts2pad_phase_done = False
        self.ts2num_phase_done = False
        self.rc_first_idle_t = None     # the RC's first Logical Idle after its last TS2
        self.rc_idle_after = 0
        self.fe_first_idle = None
        self.fe_idle8 = None
        self.first_pkt_seen = False
        self.in_training = True
        # SKP
        self.skp_last = None            # Symbol index of the last SKP COM (or the first TS1 COM)
        self.skp_last_training = True
        self.units = []                 # (start_sym, end_sym) of the last few units
        # DLL
        self.rc_seq_next = 0
        self.rc_tlps = {}               # seq -> body
        self.rc_last_new = {}           # seq -> t of its first transmission
        self.fe_nak_t = None
        self.fe_acked_upto_t = {}       # time of the far end's last Ack covering progress
        self.fe_tlp_end = {}            # far-end seq -> t (Symbol Times) its END reached the RC
        self.ack_pending = []           # far-end seqs awaiting an Ack
        self.fc_phase = None
        self.fc_seq = []                # recent InitFC types of the current phase
        self.fc_last_set_t = None
        self.fc_window_open = None
        self.fc_done_t = None
        self.rc_adv = {}
        self.upd_last = {}
        # pins
        self.rc_l0_t = None
        self.rc_linkup_t = None

    # ------------------------------------------------------------- helpers
    def flag(self, cid, t, detail):
        self.flags.append(dict(id=cid, t=t, detail=detail))
        if sum(1 for f in self.flags if f['id'] == cid) <= 3:
            self.log('FLAG|%s|%s|%s' % (cid, t, detail))

    def count(self, cid, n=1):
        self.n[cid] = self.n.get(cid, 0) + n

    def stat(self, key, v):
        self.stats.setdefault(key, []).append(v)

    def summary(self):
        out = {}
        for key, vs in sorted(self.stats.items()):
            out[key] = (len(vs), min(vs), max(vs))
        return out

    # ------------------------------------------------------------ the stream
    def on_event(self, ev):
        kind = ev['kind']
        t = ev['t']
        if kind == 'TS':
            self._ts(ev, t)
        elif kind == 'SKP':
            self._skp(ev, t)
        elif kind == 'IDLE_RUN':
            self._idle_run(ev, t)
        elif kind == 'IDLE_BAD':
            self.count('IDLE_DATA')
            self.flag('IDLE_DATA', t, 'Logical Idle descrambled to %02xh' % ev['value'])
        elif kind == 'IDLE8':
            pass
        elif kind in ('FRAME_ERR', 'OS_BAD'):
            self.count('FRAMING')
            self.flag('FRAMING', t, ev.get('reason', kind))
        elif kind == 'DLLP':
            self._dllp(ev, t)
        elif kind == 'TLP':
            self._tlp(ev, t)
        if kind not in ('IDLE8', 'IDLE_BAD'):
            self._interrupt(kind, ev, t)
        if kind in ('TS', 'SKP', 'DLLP', 'TLP', 'EIOS', 'FTS', 'OS_BAD'):
            end = ev.get('end_t', t + {'TS': 16, 'SKP': 1 + ev.get('n', 3), 'DLLP': 8}.get(kind, 4) - 1)
            self.units.append((t, end))
            if len(self.units) > 64:
                del self.units[:32]

    def _interrupt(self, kind, ev, t):
        """TS_INTERRUPT: a TS, then anything but SKP, then a TS (p.201)."""
        if kind == 'SKP':
            return
        if kind == 'TS':
            if self.after_ts is not None and self.last_unit == 'TS':
                self.count('TS_INTERRUPT')
                self.flag('TS_INTERRUPT', t, 'training sets interrupted by %s' % self.after_ts)
            self.after_ts = None
            self.last_unit = 'TS'
            return
        if self.last_unit == 'TS':
            if self.after_ts is None:
                self.after_ts = kind if kind != 'IDLE_RUN' else 'Logical Idle x%d' % ev['n']
            # a run of idle long enough to be a state change (Configuration.Idle,
            # Recovery.Idle) ends the TS sequence
            if kind == 'IDLE_RUN' and ev['n'] >= C.RX_CONSECUTIVE_IDLE:
                self.last_unit = None
                self.after_ts = None
            elif kind in ('DLLP', 'TLP'):
                self.last_unit = None
                self.after_ts = None

    def _ts(self, ev, t):
        self.count('TS_FIELDS')
        bad = []
        if ev['link'] not in (None, self.k.rc_link_num):
            bad.append('Link %r' % (ev['link'],))
        if ev['lane'] not in (None, 0):
            bad.append('Lane %r' % (ev['lane'],))
        if ev['rate'] is None or not (ev['rate'] & C.RATE_2G5) or ev['rate'] & C.RATE_BIT_SPEED_CHANGE:
            bad.append('rate %r' % (ev['rate'],))
        if ev['tc'] != 0:
            bad.append('Training Control %r' % (ev['tc'],))
        if ev['n_fts'] is None:
            bad.append('N_FTS is a K code')
        if ev['inverted']:
            bad.append('inverted identifiers')
        if bad:
            self.flag('TS_FIELDS', t, '%s: %s' % (ev['ts'], ', '.join(bad)))
        pad = ev['link'] is None and ev['lane'] is None
        if self.first_ts1_t is None and ev['ts'] == 'TS1':
            self.first_ts1_t = t
            self.skp_last = t
            self.skp_last_training = True
        # Polling.Active count
        if not self.poll_done:
            if ev['ts'] == 'TS1' and pad:
                self.poll_ts1 += 1
            elif ev['ts'] == 'TS2' and pad:
                self.poll_done = True
                self.count('POLL_TS1_COUNT')
                self.stat('poll_ts1', self.poll_ts1)
                if self.poll_ts1 < C.POLLING_ACTIVE_MIN_TS1:
                    self.flag('POLL_TS1_COUNT', t, '%d TS1 before the first TS2' % self.poll_ts1)
        # TS2 after receiving one: Polling.Configuration (PAD) and Config.Complete (numbered)
        if ev['ts'] == 'TS2' and pad and self.fe_first_ts2_pad is not None and t >= self.fe_first_ts2_pad:
            self.rc_ts2_pad_after += 1
        if ev['ts'] == 'TS1' and not pad and not self.ts2pad_phase_done and self.poll_done:
            self.ts2pad_phase_done = True
            self.count('TS2_AFTER_RX')
            self.stat('ts2_after_rx_polling', self.rc_ts2_pad_after)
            if self.rc_ts2_pad_after < C.TX_AFTER_RX_TS:
                self.flag('TS2_AFTER_RX', t, 'Polling.Configuration: %d TS2 after receiving one' % self.rc_ts2_pad_after)
        if ev['ts'] == 'TS2' and not pad and self.fe_first_ts2_num is not None and t >= self.fe_first_ts2_num:
            self.rc_ts2_num_after += 1

    def _idle_run(self, ev, t):
        if self.in_training and self.poll_done and self.ts2pad_phase_done and ev['n'] >= 2:
            # the RC left Configuration.Complete for Configuration.Idle
            self.in_training = False
            self.rc_first_idle_t = t
            if not self.ts2num_phase_done:
                self.ts2num_phase_done = True
                self.count('TS2_AFTER_RX')
                self.stat('ts2_after_rx_config', self.rc_ts2_num_after)
                if self.rc_ts2_num_after < C.TX_AFTER_RX_TS:
                    self.flag('TS2_AFTER_RX', t, 'Configuration.Complete: %d TS2 after receiving one'
                              % self.rc_ts2_num_after)
        if not self.first_pkt_seen and self.fe_first_idle is not None:
            # idle Symbols the RC sent after the far end's first idle reached it
            start = max(t, self.fe_first_idle)
            end = t + ev['n']
            if end > start:
                self.rc_idle_after += end - start

    def _skp(self, ev, t):
        self.count('SKP_FORMAT')
        if ev['n'] != C.SKP_TX_COUNT:
            self.flag('SKP_FORMAT', t, 'SKP OS with %d SKP' % ev['n'])
        self._skp_gap(t, closing=True)
        self.skp_last = t
        self.skp_last_training = self.in_training

    def _skp_gap(self, sym, closing):
        """The gap from the last SKP COM (or the first TS1 COM) to `sym`, all in
        Symbol Times at the pins; a unit in progress at the 1538 deadline
        defers the SKP to its end (p.261)."""
        t = sym
        if self.skp_last is None:
            return
        gap = sym - self.skp_last
        deadline = self.skp_last + C.SKP_INTERVAL_MAX
        allowed = deadline
        for s, e in self.units:
            if s <= deadline <= e:
                allowed = max(allowed, e + 1)
        allowed += 1                                # Symbol alignment at two Symbols per clock
        if closing or sym > allowed:
            cid = 'SKP_GAP_TRAINING' if self.skp_last_training else 'SKP_GAP_L0'
            self.count('SKP_GAP')
            self.stat('skp_gap_training' if self.skp_last_training else 'skp_gap_l0', gap)
            if sym > allowed:
                self.flag(cid, t, 'SKP gap %d Symbol Times from Symbol %d (limit %d)'
                          % (gap, self.skp_last, allowed - self.skp_last))
                if not closing:
                    self.skp_last = sym
            if closing and gap < C.SKP_INTERVAL_MIN - C.TS_LEN:
                self.flag('SKP_GAP_SHORT', t, 'SKP gap %d Symbol Times' % gap)

    def end_of_run(self, t):
        """Close the open SKP gap, the FC-init window and the UpdateFC windows
        at the end of a row."""
        if self.skp_last is not None and t - self.skp_last > C.SKP_INTERVAL_MAX + 64:
            self._skp_gap(t, closing=False)
        if self.fc_done_t is None and self.fc_last_set_t is not None and (t - self.fc_last_set_t) / 2 > 34 * US:
            self.count('FC_INIT_PERIOD')
            self.flag('FC_INIT_PERIOD', t, 'no Init set for %.1f us at the end of the run'
                      % ((t - self.fc_last_set_t) / 2 / US))
        if self.fc_done_t is not None:
            for typ, last in self.upd_last.items():
                if (t - last) / 2 > 45 * US:
                    self.count('FC_UPDATE_PERIOD')
                    self.flag('FC_UPDATE_PERIOD', t, 'no UpdateFC-%s for %.1f us at the end of the run'
                              % (typ, (t - last) / 2 / US))

    # ----------------------------------------------------------------- DLLPs
    def _dllp(self, ev, t):
        self._first_packet(t)
        b6 = ev['b6']
        self.count('DLLP_CRC')
        if not CRC.dllp_ok(b6):
            self.flag('DLLP_CRC', t, 'DLLP %s with a bad CRC' % b6.hex())
            return None
        d = SYM.parse_dllp(b6)
        self.count('DLLP_FIELDS')
        if d['kind'] in ('Reserved', 'Vendor') or d.get('vc', 0) != 0 or d['rsvd_nonzero']:
            self.flag('DLLP_FIELDS', t, 'DLLP %s: %s%s' % (b6.hex(), d['kind'], ' with Reserved bits set'
                                                            if d['rsvd_nonzero'] else ''))
        kind = d['kind']
        if kind in ('InitFC1', 'InitFC2'):
            self._fc_init(d, t)
        elif kind == 'UpdateFC':
            if d['fc'] in self.upd_last:
                gap = (t - self.upd_last[d['fc']]) / 2
                self.count('FC_UPDATE_PERIOD')
                self.stat('updatefc_gap_cycles_' + d['fc'], gap)
                if gap > 45 * US:
                    self.flag('FC_UPDATE_PERIOD', t, 'UpdateFC-%s after %.1f us' % (d['fc'], gap / US))
            self.upd_last[d['fc']] = t
        elif kind in ('Ack', 'Nak'):
            self._ack(d, t)
        return d

    def _fc_init(self, d, t):
        ph = d['kind']
        if self.fc_phase != ph:
            self.fc_phase = ph
            self.fc_seq = []
        if ph == 'InitFC1':
            self.rc_adv[d['fc']] = (d['hdr'], d['data'])
        self.count('FC_INIT_ORDER')
        exp = C.FC_TYPES[len(self.fc_seq) % 3]
        if d['fc'] != exp:
            self.flag('FC_INIT_ORDER', t, '%s-%s where %s-%s was due' % (ph, d['fc'], ph, exp))
        self.fc_seq.append(d['fc'])
        if d['fc'] == 'CPL':
            start = self.fc_last_set_t if self.fc_last_set_t is not None else self.fc_window_open
            if start is not None:
                self.count('FC_INIT_PERIOD')
                self.stat('fc_set_gap_cycles', (t - start) // 2)
                if (t - start) / 2 > 34 * US:
                    self.flag('FC_INIT_PERIOD', t, '%s set %.1f us after the previous one (or L0)'
                              % (ph, (t - start) / 2 / US))
            self.fc_last_set_t = t

    def _ack(self, d, t):
        seq = d['seq']
        self.count('ACK_SEQ')
        covered = [s for s in self.ack_pending if (seq - s) % C.SEQ_MOD < 2048]
        for s in covered:
            end = self.fe_tlp_end[s]
            lim = C.ACK_LATENCY_X1[self.k.rc_max_payload]
            allowed = end + lim
            for us, ue in self.units:
                if us <= allowed <= ue:
                    allowed = max(allowed, ue + 1)
            self.count('ACK_LATENCY')
            self.stat('ack_latency_st', t - end)
            if d['kind'] == 'Ack' and t > allowed + 1:
                self.flag('ACK_LATENCY', t, 'Ack of seq %03x %d Symbol Times after its END (limit %d)'
                          % (s, t - end, allowed - end))
        self.ack_pending = [s for s in self.ack_pending if s not in covered]

    # ------------------------------------------------------------------ TLPs
    def _tlp(self, ev, t):
        self._first_packet(t)
        body = ev['body']
        self.count('LCRC')
        if ev['end'] != 'END' or not CRC.lcrc_ok(body):
            self.flag('LCRC', t, 'TLP ending %s with %s LCRC' % (ev['end'], 'a good' if CRC.lcrc_ok(body) else
                                                                  'a bad'))
            return
        seq = ((body[0] & 0x0F) << 8) | body[1]
        if body[0] & 0xF0:
            self.flag('FRAMING', t, 'Reserved bits of the sequence-number field set')
        tlp = body[2:-4]
        self.count('SEQ')
        if seq == self.rc_seq_next:
            self.rc_seq_next = (seq + 1) % C.SEQ_MOD
            self.rc_tlps[seq] = tlp
            self.rc_last_new[seq] = t
        elif seq in self.rc_tlps:
            self.count('REPLAY')
            why = []
            if self.rc_tlps[seq] != tlp:
                why.append('content differs')
            first = self.rc_last_new[seq]
            timer_ok = (t - first) >= C.REPLAY_TIMER_X1[self.k.rc_max_payload]
            nak_ok = self.fe_nak_t is not None and self.fe_nak_t <= t
            if not (timer_ok or nak_ok):
                why.append('no Nak and only %d Symbol Times after its first transmission' % (t - first))
            if why:
                self.flag('REPLAY', t, 'replay of seq %03x: %s' % (seq, '; '.join(why)))
        else:
            self.flag('SEQ', t, 'seq %03x where %03x was due' % (seq, self.rc_seq_next))
        f = TLP.parse(tlp)
        self.count('TLP_FORMAT')
        bad = []
        if not f.get('size_ok'):
            bad.append('size does not match Length')
        if f.get('rsvd_dw0'):
            bad.append('Reserved bits in DW0')
        if f.get('name', '').startswith('Cfg'):
            if f['tc'] or f['attr'] or f['at'] or f['length'] != 1 or f['last_be'] or (f['raw'][10] & 0xF0) or \
                    (f['raw'][11] & 0x03):
                bad.append('Configuration Request field rules (TC %d Attr %d AT %d Length %d Last BE %x)'
                           % (f['tc'], f['attr'], f['at'], f['length'], f['last_be']))
        if bad:
            self.flag('TLP_FORMAT', t, '%s: %s' % (f.get('name'), ', '.join(bad)))
        return f

    def _first_packet(self, t):
        if self.first_pkt_seen:
            return
        self.first_pkt_seen = True
        self.count('PKT_BEFORE_L0')
        self.stat('idle_before_first_packet', self.rc_idle_after)
        why = []
        if self.rc_idle_after < C.TX_AFTER_RX_IDLE:
            why.append('%d idle Symbols sent after receiving one' % self.rc_idle_after)
        if self.fe_idle8 is None or t < self.fe_idle8:
            why.append('before 8 idle Symbol Times were received')
        if why:
            self.flag('PKT_BEFORE_L0', t, '; '.join(why))

    # -------------------------------------------------- far-end timeline
    def fe_ts2(self, numbered, t):
        if numbered and self.fe_first_ts2_num is None:
            self.fe_first_ts2_num = t
        if not numbered and self.fe_first_ts2_pad is None:
            self.fe_first_ts2_pad = t

    def fe_idle(self, t, n_total):
        if self.fe_first_idle is None:
            self.fe_first_idle = t
        if n_total >= C.RX_CONSECUTIVE_IDLE and self.fe_idle8 is None:
            self.fe_idle8 = t

    def fe_tlp(self, seq, end_t, replay):
        self.fe_tlp_end[seq] = end_t + 2          # sampled by the RC at the next edge
        if seq not in self.ack_pending:
            self.ack_pending.append(seq)

    def fe_nak(self, t):
        self.fe_nak_t = t

    # ------------------------------------------------------------------ pins
    def pins(self, t, txelecidle, powerdown, txdetectrx, rxpolarity, txcompliance, ltssm, fc_init_done):
        if fc_init_done and self.fc_done_t is None:
            self.fc_done_t = t
            for typ in C.FC_TYPES:
                h, dd = self.rc_adv.get(typ, (0, 0))
                if h or dd:
                    self.upd_last[typ] = t
        if txelecidle:
            self.skp_last = None                 # time in Electrical Idle does not count (p.262)
        if txdetectrx and txelecidle == 0:
            if not any(n[0] == 'N3' for n in self.notes):
                self.notes.append(('N3', t, 'phy_txelecidle 0 while phy_txdetectrx 1 (logged, R)'))
        if rxpolarity:
            self.count('PIN_RXPOLARITY')
            self.flag('PIN_RXPOLARITY', t, 'phy_rxpolarity asserted')
        if txcompliance:
            self.count('PIN_RXPOLARITY')
            self.flag('PIN_RXPOLARITY', t, 'phy_txcompliance asserted')
        if ltssm == 0x05 and self.rc_l0_t is None:
            self.rc_l0_t = t
            if self.fc_window_open is None:
                self.fc_window_open = t
        if self.rc_l0_t is not None and txelecidle:
            self.flag('PIN_TX_EI_L0', t, 'phy_txelecidle asserted after L0 (state %05x)' % ltssm)
