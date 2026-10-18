"""lr1_farend -- the LR1 far end: an upstream port at pcie_rc_top's PIPE seam

Author: Kourosh Ghahramani
Silicon Systems Research Lab, University of Washington

Composition (all lr1_*; BRIEF_LR1 D4):
  PHY side       this file: the PIPE contract of the RC's PHY, modelled from the
                 PHY's side and never copied from the RC's outputs (HANDSHAKE §5,
                 8-2): receiver detection answered as a handshake on each rising
                 edge of phy_txdetectrx with phy_phystatus for one clock and
                 phy_rxstatus 011b (PG239 Table 10 p.16); phy_rxelecidle 1 until
                 the far end transmits, phy_rxvalid 1 while it does;
                 phy_rxdata_valid 0 (Gen3 and above only, PG239 Table 7 p.13).
                 The RC's transmit pins are sampled on every clock with
                 phy_txelecidle 0 at P0, whatever phy_txdata_valid says (Gen3
                 only, PG239 Table 5 p.12). [7:0] is the first Symbol on the
                 wire. The far end delivers COM in byte 1 ([15:8]) by default, as
                 PG239 does (RULINGS_A 6), and in byte 0 as the variation.
  receiver       lr1_rx (delineation), lr1_checks (D6)
  LTSSM          lr1_ltssm (upstream port, §4.2.6)
  DLL            lr1_dll (FC init, sequence numbers, Ack / Nak, replay, UpdateFC)
  TL             lr1_cfg (one Function, Type 0 space, UR for the rest)
  transmit       this file: one unit at a time (a training set, a SKP OS, a
                 framed packet, or two idle Symbols), scrambled on the way out;
                 SKP OS scheduled every `skp_interval` Symbol Times in every
                 non-EI state, training included, inserted at the next unit
                 boundary (§4.2.7.1 pp.261-262); packets only in L0 (§4.2.6.3.6:
                 Configuration.Idle sends Idle data); `pkt_gap` idle Symbols
                 after each packet (0 = back to back, §4.2.2 p.196).

Every stream the far end builds is even in length between COMs, so the byte
lane of each COM is fixed by the first word (com_lane).

The far end observes and drives PINS ONLY (D3).
"""
from collections import deque
from dataclasses import dataclass, field

import cocotb
from cocotb.triggers import RisingEdge

import lr1_cfg as CFG
import lr1_checks as CHK
import lr1_consts as C
import lr1_dll as DLL
import lr1_ltssm as LT
import lr1_rx as RX
import lr1_scrambler as SCR
import lr1_symbols as SYM

US = 125


@dataclass
class Knobs:
    # PHY
    com_lane: int = 1                  # RULINGS_A 6: COM in byte 1 by default
    detect_delay: int = 100            # cycles from the RC in EI at P1 to the far end's TS1
    detect_latency: int = 4            # phy_phystatus answers this many clocks after the request
    # LTSSM
    n_fts: int = 0xFF
    rate_id: int = C.RATE_2G5
    lanenum_wait_cycles: int = 0       # V13: up to 1 ms (p.234)
    poll_ts1_min: int = C.POLLING_ACTIVE_MIN_TS1   # V12: at least 1024 (p.220)
    ts2_after_rx: int = C.TX_AFTER_RX_TS           # V12: at least 16 (p.225, p.236)
    idle_after_rx: int = C.TX_AFTER_RX_IDLE        # V12: at least 16 (p.237)
    cfg_lw_extra_ts1: int = 0                      # V12: PAD TS1 sent after the Link number is seen
    # transmit scheduler
    skp_interval: int = 1260           # Symbol Times, inside 1180..1538 (p.261)
    skp_counts: tuple = (3,)           # V11: SKP per OS at the RC's pins, cycled (1..5 legal, p.262)
    pkt_gap: int = 8                   # idle Symbols after a packet (even)
    # Data Link Layer
    adv_credits: dict = field(default_factory=lambda: {
        'P': (C.MIN_PH, C.min_pd(128)), 'NP': (C.MIN_NPH, C.MIN_NPD), 'CPL': (0, 0)})   # Table 2-37 p.137
    max_payload: int = 128
    fc_init_start_delay: int = 0
    fc_set_period: int = 2 * US        # an Init set every 2 us (limit 34 us, p.161)
    drop_initfc_set_on_flag: bool = False
    stop_initfc2_on_fi2: bool = False  # ruling G
    fc2_first_delay_cycles: int = 0    # the first InitFC2 set this long after FI1 (limit 34 us)
    updatefc_cpl_zero_period: int = 0  # V5: UpdateFC-Cpl 0/0 this often (p.138)
    redundant_ack_period: int = 0      # V8: an Ack naming ACKD_SEQ when idle this long (p.180)
    nullify_count: int = 0             # V18: the first N TLPs each sent nullified first (p.174)
    updatefc_period: int = 20 * US     # limit 30 us (p.143)
    updatefc_every_release: bool = False
    ack_delay_cycles: int = 0          # V7
    replay_timer_cycles: int = (C.REPLAY_TIMER_X1[128] * 3 // 2) // 2   # 711 ST +50 %, in cycles
    fc_rsvd: dict = field(default_factory=dict)
    ack_rsvd: dict = field(default_factory=dict)
    # Transaction Layer
    bar_size: int = 0x1000
    bar_prefetchable: bool = False
    crs_cycles: int = 0                # V19
    cpl_delay_cycles: int = 0          # V20
    # what the checks expect of the RC
    rc_link_num: int = 0
    rc_max_payload: int = 128


class FarEnd:
    def __init__(self, dut, knobs=None, log=None):
        self.dut = dut
        self.k = knobs or Knobs()
        self._log = log or dut._log.info
        self.lines = []
        self.cycle = 0
        self.rx = RX.RcStream()
        self.ltssm = LT.Ltssm(self.k, self.log)
        self.ltssm.rx_stream = self.rx
        self.dll = DLL.Dll(self.k, self.log, lambda: self.cycle)
        self.tl = CFG.Completer(self.k, self.log, lambda: self.cycle)
        self.dll.tl = self.tl
        self.chk = CHK.Checks(self.k, self.log)
        self.rx.on(self._on_rc_event)
        self.scr = SCR.TxScrambler()
        self.txq = deque()             # (byte, k, in_ts, tag)
        self.skp_count = 0
        self.skp_due = False
        self.skp_sent = 0
        self.rxstatus_now = 0
        # witnesses: what the far end actually sent (a row proves its knob fired)
        self.com_lanes = [0, 0]
        self.skp_gaps = []
        self.skp_ns = set()
        self.skp_times = []
        self.last_skp_t = None
        self.b2b = 0
        self.last_was_pkt = False
        self.first_sent = {}           # 'InitFC1' / 'InitFC2' / 'UpdateFC' / ... -> cycle first driven
        self.tx_ts = []                # (t, 'TS1' / 'TS2', state) of each training set driven in Recovery
        self.rc_ts = []                # (t, 'TS1' / 'TS2') of each RC training set after L0 was first reached
        self.rc_states = []            # (t, ltssm_debug_state) at each change
        self.rc_state_last = None
        self.first_rc_tlp = None       # cycle the RC's first TLP started
        self.gap_left = 0
        self.fe_idle_total = 0
        self.was_tx = False
        self.prev_detect = 0
        self.phystatus_at = None
        self.detect_answers = 0

    def log(self, s):
        self.lines.append(s)
        self._log(s)

    # ------------------------------------------------------- receive side
    def _on_rc_event(self, ev):
        if ev['kind'] == 'TS' and self.chk.rc_l0_t is not None:
            self.rc_ts.append((ev['t'], ev['ts']))
        self.chk.on_event(ev)
        self.ltssm.on_event(ev, self.cycle)
        kind = ev['kind']
        if kind == 'DLLP':
            d = SYM.parse_dllp(ev['b6'])
            if d['crc_ok']:
                self.dll.on_dllp(d)
        elif kind == 'TLP':
            if self.first_rc_tlp is None:
                self.first_rc_tlp = self.cycle
            t = self.dll.on_tlp(ev)
            if t is not None:
                for c in self.tl.handle(t):
                    self.dll.send_tlp(c)
                self.dll.release(t)

    # ------------------------------------------------------ transmit side
    def _push(self, syms, tag=None):
        for i, (b, k, in_ts) in enumerate(syms):
            self.txq.append((b, k, in_ts, tag if i == 0 else None))

    def _fill(self):
        st = self.ltssm.state
        while len(self.txq) < 2:
            if self.skp_due:
                self.skp_due = False
                self.skp_count = 0
                n = self.k.skp_counts[self.skp_sent % len(self.k.skp_counts)]
                self.skp_sent += 1
                self._push([(b, k, False) for b, k in SYM.skp_os(n)], ('SKP', n))
                self.last_was_pkt = False
                continue
            unit = self.ltssm.next_unit()
            if unit is not None:
                ts2 = unit[6][0] == C.TS2_ID
                numbered = not unit[1][1]
                if st.startswith('REC'):
                    self._push(unit, ('RECTS', 'TS2' if ts2 else 'TS1', st))
                else:
                    self._push(unit, ('TS2', numbered) if ts2 else None)
                self.last_was_pkt = False
                continue
            pkt = None
            if st == 'L0' and self.gap_left <= 0:
                pkt = self.dll.next_packet(self.cycle)
            if pkt is not None:
                kind, frame = pkt
                if self.last_was_pkt:
                    self.b2b += 1
                self.last_was_pkt = True
                syms = [(b, k, False) for b, k in frame]
                if kind in ('TLP', 'TLP-replay'):
                    seq = ((frame[1][0] & 0xF) << 8) | frame[2][0]
                    for i, s in enumerate(syms):
                        tag = ('TLP_END', seq, kind == 'TLP-replay') if i == len(syms) - 1 else None
                        self.txq.append(s + (tag,))
                else:
                    self._push(syms, ('NAK',) if kind == 'Nak' else ('DLLP', kind.split('-')[0]))
                self.gap_left = self.k.pkt_gap
                continue
            self._push([(0x00, 0, False), (0x00, 0, False)], ('IDLE', 2))
            self.last_was_pkt = False
            self.gap_left -= 2
            self.ltssm.idle_sent(2)

    def _pop_word(self, t0):
        """Two Symbols for this clock, scrambled; t0 = this word's time in
        Symbol Times as the RC will sample it."""
        out = []
        for lane in range(2):
            b, k, in_ts, tag = self.txq.popleft()
            if k and b == C.COM:
                self.com_lanes[lane] += 1
            out.append((self.scr.symbol(b, k, in_ts), k))
            self.skp_count += 1
            if self.skp_count >= self.k.skp_interval:
                self.skp_due = True
            if tag is not None:
                self._tag(tag, t0 + lane)
        return out

    def _tag(self, tag, t):
        kind = tag[0]
        if kind == 'TS2':
            self.chk.fe_ts2(tag[1], t)
        elif kind == 'IDLE':
            if self.ltssm.state in ('CFG_IDLE', 'L0'):
                self.fe_idle_total += tag[1]
                self.chk.fe_idle(t, self.fe_idle_total)
        elif kind == 'TLP_END':
            self.chk.fe_tlp(tag[1], t, tag[2])
        elif kind == 'NAK':
            self.chk.fe_nak(t)
            self.first_sent.setdefault('Nak', t // 2)
        elif kind == 'DLLP':
            self.first_sent.setdefault(tag[1], t // 2)
        elif kind == 'RECTS':
            self.tx_ts.append((t, tag[1], tag[2]))
        if kind == 'SKP':
            self.skp_ns.add(tag[1])
            self.skp_times.append(t)
            if self.last_skp_t is not None:
                self.skp_gaps.append(t - self.last_skp_t)
            self.last_skp_t = t
        if kind == 'SKP' and tag[1] != C.SKP_TX_COUNT:
            # the PHY's elastic buffer added (001b) or removed (010b) SKP (PG239 Table 10 p.16)
            self.rxstatus_now = 0b001 if tag[1] > C.SKP_TX_COUNT else 0b010

    # --------------------------------------------------------------- clock
    def drive_reset_values(self):
        d = self.dut
        d.phy_rxdata.value = 0
        d.phy_rxdatak.value = 0
        d.phy_rxvalid.value = 0
        d.phy_rxdata_valid.value = 0
        d.phy_rxstart_block.value = 0
        d.phy_rxsync_header.value = 0
        d.phy_phystatus.value = 0
        d.phy_phystatus_rst.value = 0
        d.phy_rxelecidle.value = 1
        d.phy_rxstatus.value = 0

    def step(self):
        d = self.dut
        self.cycle += 1
        cyc = self.cycle
        txei = int(d.phy_txelecidle.value)
        pd = int(d.phy_powerdown.value)
        det = int(d.phy_txdetectrx.value)
        # receiver detection: a handshake on the request's rising edge
        if det and not self.prev_detect and pd == 2:
            self.phystatus_at = cyc + self.k.detect_latency
        self.prev_detect = det
        if self.phystatus_at is not None and cyc == self.phystatus_at:
            d.phy_phystatus.value = 1
            d.phy_rxstatus.value = 0b011
            self.detect_answers += 1
        elif self.phystatus_at is not None and cyc == self.phystatus_at + 1:
            d.phy_phystatus.value = 0
            d.phy_rxstatus.value = 0
            self.phystatus_at = None
        # the RC's transmit stream
        if txei == 0 and pd == 0:
            w = int(d.phy_txdata.value)
            kk = int(d.phy_txdatak.value)
            self.rx.feed(w & 0xFF, kk & 1, 2 * cyc)
            self.rx.feed((w >> 8) & 0xFF, (kk >> 1) & 1, 2 * cyc + 1)
        lt = int(d.ltssm_debug_state.value) & 0xFFFFF
        if lt != self.rc_state_last:
            self.rc_state_last = lt
            self.rc_states.append((2 * cyc, lt))     # the RC's own state report, for measurements only
        self.chk.pins(2 * cyc, txei, pd, det, int(d.phy_rxpolarity.value), int(d.phy_txcompliance.value),
                      lt, int(d.fc_init_done_o.value))
        # LTSSM, DLL, TL
        self.ltssm.tick(cyc, txei == 1 and pd == 2)
        self.dll.link_up(self.ltssm.linkup)
        self.dll.tick(cyc)
        for c in self.tl.due():
            self.dll.send_tlp(c)
        # the far end's transmit stream
        if self.ltssm.transmitting():
            if not self.was_tx:
                self.was_tx = True
                self.scr = SCR.TxScrambler()
                self.txq.clear()
                self.skp_count = 0
                if self.k.com_lane == 1:
                    self.txq.append((0x00, 0, False, None))   # one Symbol ahead of the first COM
            self._fill()
            self.rxstatus_now = 0
            (b0, k0), (b1, k1) = self._pop_word(2 * (cyc + 1))
            if self.phystatus_at is None:
                d.phy_rxstatus.value = self.rxstatus_now
            d.phy_rxdata.value = (b1 << 8) | b0
            d.phy_rxdatak.value = (k1 << 1) | k0
            d.phy_rxvalid.value = 1
            d.phy_rxelecidle.value = 0
        else:
            if self.was_tx:
                self.was_tx = False
            d.phy_rxvalid.value = 0
            d.phy_rxelecidle.value = 1
            d.phy_rxdata.value = 0
            d.phy_rxdatak.value = 0

    async def run(self, max_cycles, until=None, every=64):
        """Run the far end for up to max_cycles clocks, or until until() is true
        (checked every `every` clocks)."""
        clk = self.dut.clk_i
        for i in range(max_cycles):
            await RisingEdge(clk)
            self.step()
            if until is not None and i % every == 0 and until():
                return True
        return False

    def finish(self):
        self.chk.end_of_run(2 * self.cycle)
        self.chk.count('FC_OVERFLOW', self.dll.fc_checked)
        for t, text in self.dll.errors:
            cid = 'ACK_SEQ' if 'ACKD_SEQ' in text else 'FC_OVERFLOW' if 'overflow' in text else 'DLL'
            self.chk.flag(cid, 2 * t, text)
        return self.chk
