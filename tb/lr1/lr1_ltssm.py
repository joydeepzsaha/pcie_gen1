"""lr1_ltssm -- the far end's LTSSM: an upstream port at 2.5 GT/s, x1, from Base 2.1 §4.2.6

Author: Kourosh Ghahramani
Silicon Systems Research Lab, University of Washington

States and their rules, each from the upstream-lane text of Base 2.1:

  DETECT        Electrical Idle. The far end's own receiver detection is not
                visible at the seam; it is modelled as succeeding `detect_delay`
                cycles after the RC is seen in Electrical Idle at P1
                (§4.2.6.1 pp.218-219).
  POLL_ACTIVE   TS1, Link and Lane PAD. -> POLL_CONFIG after >= 1024 TS1 sent
                and 8 consecutive TS1 (PAD, Compliance Receive 0) or TS2 (PAD)
                received. 24 ms -> DETECT (§4.2.6.2.1 pp.220-221).
  POLL_CONFIG   TS2, PAD. -> CFG_LW_START after 8 consecutive TS2 (PAD)
                received and 16 TS2 sent after receiving one. 48 ms -> DETECT
                (§4.2.6.2.3 pp.224-225).
  CFG_LW_START  TS1, PAD. 2 consecutive TS1 with Link != PAD and Lane = PAD:
                adopt the Link number -> CFG_LW_ACCEPT. 24 ms -> DETECT
                (§4.2.6.3.1.2 pp.228-230).
  CFG_LW_ACCEPT TS1, Link N, Lane PAD. 2 consecutive TS1 with Link N and a
                non-PAD Lane: adopt the Lane number -> CFG_LN_WAIT. 2 ms, or 2
                TS1 PAD/PAD -> DETECT (§4.2.6.3.2.2 pp.230-231).
  CFG_LN_WAIT   TS1, Link N, Lane n. 2 consecutive TS2, or 2 TS1 with a changed
                Lane -> CFG_LN_ACCEPT; the upstream port may wait up to 1 ms
                first (`lanenum_wait_cycles`). 2 ms, or 2 TS1 PAD/PAD -> DETECT
                (§4.2.6.3.4.2 p.234).
  CFG_LN_ACCEPT TS1, Link N, Lane n. 2 consecutive TS2 matching Link N and Lane
                n -> CFG_COMPLETE (§4.2.6.3.3.2 p.233).
  CFG_COMPLETE  TS2, Link N, Lane n. 8 consecutive TS2 matching, identical rate
                identifiers, and 16 consecutive TS2 sent after receiving one ->
                CFG_IDLE. 2 ms -> DETECT (§4.2.6.3.5.2 pp.236-237).
  CFG_IDLE      Logical Idle, LinkUp = 1. 8 consecutive idle Symbol Times
                received and 16 idle Symbols sent after receiving one -> L0.
                2 ms -> REC_LOCK once, then DETECT (§4.2.6.3.6 p.237).
  L0            packets, idle, SKP. A TS1 or TS2 received -> REC_LOCK
                (§4.2.6.5 pp.247-248).
  REC_LOCK      TS1 with the configured numbers. 8 consecutive TS1 / TS2
                matching -> REC_CFG. 24 ms -> CONFIG-or-DETECT
                (§4.2.6.4.1 pp.239-240).
  REC_CFG       TS2. 8 consecutive TS2 matching and 16 sent after receiving
                one -> REC_IDLE; 8 TS1 with other numbers -> CFG_LW_START.
                48 ms -> DETECT (§4.2.6.4.3 pp.243-245).
  REC_IDLE      idle. 8 idle Symbol Times received and 16 sent after
                receiving one -> L0; 2 TS1 with Lane PAD -> CFG_LW_START.
                2 ms -> REC_LOCK once, then DETECT (§4.2.6.4.4 pp.245-246).

SKP Ordered Sets interrupt nothing (§4.2.7.1 p.262); the transmit scheduler
(lr1_farend) inserts them between units in every non-EI state, training
included. Disabled, Loopback, Hot Reset, L0s, L1, L2 and Polling.Compliance are
not implemented: a request for them is recorded as an event, not obeyed.

next_unit() returns the next transmit unit for the current state as a list of
(byte, k, in_ts); the caller scrambles it.
"""
import lr1_consts as C
import lr1_symbols as SYM

US = 125          # cycles per microsecond at the 8 ns clock
MS = 1000 * US

TIMEOUT = {'POLL_ACTIVE': 24 * MS, 'POLL_CONFIG': 48 * MS, 'CFG_LW_START': 24 * MS, 'CFG_LW_ACCEPT': 2 * MS,
           'CFG_LN_WAIT': 2 * MS, 'CFG_LN_ACCEPT': 2 * MS, 'CFG_COMPLETE': 2 * MS, 'CFG_IDLE': 2 * MS,
           'REC_LOCK': 24 * MS, 'REC_CFG': 48 * MS, 'REC_IDLE': 2 * MS}


class Ltssm:
    def __init__(self, knobs, log):
        self.k = knobs
        self.log = log
        self.state = 'DETECT'
        self.entered = 0
        self.history = [(0, 'DETECT')]
        self.link = None
        self.lane = None
        self.linkup = False
        self.idle_to_rlock = False
        self.consec = []             # TS events since entry / last interruption
        self.ts_sent = 0             # TS sent in this state
        self.rx_one = False          # the relevant TS (or idle) received since entry
        self.sent_after_rx = 0       # TS2 / idle Symbols sent after rx_one
        self.ei_seen_at = None       # cycle the RC was first seen in EI at P1
        self.rx_stream = None        # lr1_rx.RcStream, for the Logical Idle run
        self.events = []             # requests the far end does not implement
        self._lnwait_ready = None

    # ------------------------------------------------------------ transitions
    def goto(self, st, cycle):
        if st == self.state:
            return
        self.log('LTSSM|%d|%s->%s' % (cycle, self.state, st))
        self.state = st
        self.entered = cycle
        self.history.append((cycle, st))
        self.consec = []
        self.ts_sent = 0
        self.rx_one = False
        self.sent_after_rx = 0
        self._lnwait_ready = None
        if st == 'CFG_IDLE':
            self.linkup = True
        if st in ('DETECT',):
            self.linkup = False
            self.link = self.lane = None

    # ---------------------------------------------------------------- receive
    def on_event(self, ev, cycle):
        kind = ev['kind']
        if kind == 'SKP':
            return                                   # not an interruption (p.262)
        if kind != 'TS':
            if kind in ('IDLE_RUN', 'OS_BAD', 'FRAME_ERR', 'DLLP', 'TLP', 'EIOS', 'FTS', 'IDLE_BAD'):
                self.consec = []
            if kind == 'EIOS' and self.state == 'L0':
                self.events.append((cycle, 'EIOS in L0: L0s / L1 not implemented'))
            return
        self.consec.append(ev)
        if len(self.consec) > 32:
            del self.consec[:-16]
        st = self.state
        n = len(self.consec)
        last = self.consec[-1]

        def all_last(m, pred):
            return n >= m and all(pred(e) for e in self.consec[-m:])

        pad = lambda e: e['link'] is None and e['lane'] is None
        if st == 'POLL_ACTIVE':
            ok = lambda e: pad(e) and ((e['ts'] == 'TS1' and not (e['tc'] & C.TC_COMPLIANCE_RX)) or e['ts'] == 'TS2')
            if all_last(C.RX_CONSECUTIVE_TS, ok) and self.ts_sent >= C.POLLING_ACTIVE_MIN_TS1:
                self.goto('POLL_CONFIG', cycle)
        elif st == 'POLL_CONFIG':
            if last['ts'] == 'TS2' and pad(last):
                self.rx_one = True
            if all_last(C.RX_CONSECUTIVE_TS, lambda e: e['ts'] == 'TS2' and pad(e)) and \
                    self.sent_after_rx >= C.TX_AFTER_RX_TS:
                self.goto('CFG_LW_START', cycle)
        elif st == 'CFG_LW_START':
            if all_last(2, lambda e: e['ts'] == 'TS1' and e['tc'] & (C.TC_DISABLE | C.TC_LOOPBACK)):
                self.events.append((cycle, 'Disable / Loopback requested: not implemented'))
            if all_last(2, lambda e: e['ts'] == 'TS1' and isinstance(e['link'], int) and e['lane'] is None):
                if self.consec[-1]['link'] == self.consec[-2]['link']:
                    self.link = self.consec[-1]['link']
                    self.goto('CFG_LW_ACCEPT', cycle)
        elif st == 'CFG_LW_ACCEPT':
            if all_last(2, lambda e: e['ts'] == 'TS1' and pad(e)):
                self.goto('DETECT', cycle)
            elif all_last(2, lambda e: e['ts'] == 'TS1' and e['link'] == self.link and isinstance(e['lane'], int)):
                if self.consec[-1]['lane'] == self.consec[-2]['lane']:
                    self.lane = self.consec[-1]['lane']      # x1: match the received Lane number
                    self.goto('CFG_LN_WAIT', cycle)
        elif st == 'CFG_LN_WAIT':
            if all_last(2, lambda e: e['ts'] == 'TS1' and pad(e)):
                self.goto('DETECT', cycle)
            elif all_last(2, lambda e: e['ts'] == 'TS2') or \
                    all_last(2, lambda e: e['ts'] == 'TS1' and isinstance(e['lane'], int) and e['lane'] != self.lane
                             and e['link'] is not None):
                self._lnwait_ready = cycle
        elif st == 'CFG_LN_ACCEPT':
            if all_last(2, lambda e: e['ts'] == 'TS2' and e['link'] == self.link and e['lane'] == self.lane):
                self.goto('CFG_COMPLETE', cycle)
            elif all_last(2, lambda e: e['ts'] == 'TS1' and pad(e)):
                self.goto('DETECT', cycle)
        elif st == 'CFG_COMPLETE':
            if last['ts'] == 'TS2':
                self.rx_one = True
            m = C.RX_CONSECUTIVE_TS
            if all_last(m, lambda e: e['ts'] == 'TS2' and e['link'] == self.link and e['lane'] == self.lane) and \
                    len({e['rate'] for e in self.consec[-m:]}) == 1 and self.sent_after_rx >= C.TX_AFTER_RX_TS:
                self.goto('CFG_IDLE', cycle)
        elif st == 'L0':
            self.goto('REC_LOCK', cycle)
            self.consec = [ev]
        elif st == 'REC_LOCK':
            m = C.RX_CONSECUTIVE_TS
            if all_last(m, lambda e: e['link'] == self.link and e['lane'] == self.lane and
                        not (e['rate'] & C.RATE_BIT_SPEED_CHANGE)):
                self.goto('REC_CFG', cycle)
        elif st == 'REC_CFG':
            if last['ts'] == 'TS2':
                self.rx_one = True
            m = C.RX_CONSECUTIVE_TS
            if all_last(m, lambda e: e['ts'] == 'TS2' and e['link'] == self.link and e['lane'] == self.lane) and \
                    self.sent_after_rx >= C.TX_AFTER_RX_TS:
                self.goto('REC_IDLE', cycle)
            elif all_last(m, lambda e: e['ts'] == 'TS1' and (e['link'] != self.link or e['lane'] != self.lane)):
                self.goto('CFG_LW_START', cycle)
        elif st == 'REC_IDLE':
            if all_last(2, lambda e: e['ts'] == 'TS1' and e['lane'] is None):
                self.goto('CFG_LW_START', cycle)
            elif all_last(2, lambda e: e['ts'] == 'TS1' and e['tc'] & C.TC_HOT_RESET):
                self.events.append((cycle, 'Hot Reset requested: not implemented'))

    # ------------------------------------------------------------------ clock
    def tick(self, cycle, rc_ei_p1):
        st = self.state
        if st == 'DETECT':
            if rc_ei_p1:
                if self.ei_seen_at is None:
                    self.ei_seen_at = cycle
                if cycle - self.ei_seen_at >= self.k.detect_delay:
                    self.goto('POLL_ACTIVE', cycle)
            else:
                self.ei_seen_at = None
            return
        if st == 'CFG_LN_WAIT' and self._lnwait_ready is not None:
            if cycle - self.entered >= self.k.lanenum_wait_cycles:
                self._lnwait_ready = None
                self.goto('CFG_LN_ACCEPT', cycle)
                return
        if st in ('CFG_IDLE', 'REC_IDLE'):
            run = self.rx_stream.idle_run if self.rx_stream else 0
            if run >= 1:
                self.rx_one = True
            if run >= C.RX_CONSECUTIVE_IDLE and self.sent_after_rx >= C.TX_AFTER_RX_IDLE:
                self.goto('L0', cycle)
                return
        t = TIMEOUT.get(st)
        if t is not None and cycle - self.entered >= t:
            self.events.append((cycle, 'timeout in %s' % st))
            if st in ('CFG_IDLE', 'REC_IDLE') and not self.idle_to_rlock:
                self.idle_to_rlock = True
                self.goto('REC_LOCK', cycle)
            elif st == 'REC_LOCK' and self.consec:
                self.goto('CFG_LW_START', cycle)
            else:
                self.goto('DETECT', cycle)

    # --------------------------------------------------------------- transmit
    def transmitting(self):
        return self.state != 'DETECT'

    def _ts(self, ident, link, lane):
        self.ts_sent += 1
        if self.rx_one and ident == C.TS2_ID:
            self.sent_after_rx += 1
        return [(b, k, i > 0) for i, (b, k) in enumerate(
            SYM.ts(ident, link=link, lane=lane, n_fts=self.k.n_fts, rate=self.k.rate_id, tc=0))]

    def next_unit(self):
        """The next training set, or None in the states that send idle / packets."""
        st = self.state
        if st == 'POLL_ACTIVE':
            return self._ts(C.TS1_ID, None, None)
        if st == 'POLL_CONFIG':
            return self._ts(C.TS2_ID, None, None)
        if st == 'CFG_LW_START':
            return self._ts(C.TS1_ID, None, None)
        if st == 'CFG_LW_ACCEPT':
            return self._ts(C.TS1_ID, self.link, None)
        if st in ('CFG_LN_WAIT', 'CFG_LN_ACCEPT', 'REC_LOCK'):
            return self._ts(C.TS1_ID, self.link, self.lane)
        if st in ('CFG_COMPLETE', 'REC_CFG'):
            return self._ts(C.TS2_ID, self.link, self.lane)
        return None

    def idle_sent(self, n):
        if self.state in ('CFG_IDLE', 'REC_IDLE') and self.rx_one:
            self.sent_after_rx += n
