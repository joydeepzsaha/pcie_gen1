"""lr1_symbols -- ordered sets, framing and DLLPs, as (byte, k) Symbol lists.

Author: Kourosh Ghahramani
Silicon Systems Research Lab, University of Washington

A Symbol is (byte, k): k = 1 for a K code (lr1_consts, Table 4-1 p.194).
Symbols are listed in wire order: the first element is the first Symbol on the
Lane.

Ordered sets (Tables 4-2 / 4-3 pp.201-205, Table 4-4 p.205, §4.2.7 p.261):
  ts(ident, link, lane, n_fts, rate, tc)   16 Symbols; link / lane None = PAD
  skp_os(n)                                COM + n SKP
  eios()                                   COM IDL IDL IDL
Framing at x1 (§4.2.2 pp.195-196; Figures 4-6 and 4-7 p.197):
  frame_tlp(seq, tlp)    STP . {0000b, Seq[11:8]} . Seq[7:0] . TLP . LCRC . END
                         (EDB and the inverted LCRC when nullified, p.174)
  frame_dllp(b4)         SDP . 4 bytes . CRC (2) . END
DLLPs (Table 3-1 p.165; Figures 3-5..3-8 p.166):
  Ack / Nak:   byte 0 type, byte 1 Reserved, byte 2 {Reserved[7:4],
               Seq[11:8]}, byte 3 Seq[7:0]
  FC DLLPs:    byte 0 type | VC, byte 1 {R[7:6], HdrFC[7:2]},
               byte 2 {HdrFC[1:0], R[5:4], DataFC[11:8]}, byte 3 DataFC[7:0]
  parse_dllp() returns the fields and whether any Reserved bit is set.

Independent of the RTL; known answers in test_lr1_kat.py.
"""
import lr1_consts as C
import lr1_crc as CRC


# ------------------------------------------------------------- ordered sets
def ts(ident, link=None, lane=None, n_fts=0xFF, rate=C.RATE_2G5, tc=0x00):
    s1 = (C.PAD, 1) if link is None else (link & 0xFF, 0)
    s2 = (C.PAD, 1) if lane is None else (lane & 0xFF, 0)
    return [(C.COM, 1), s1, s2, (n_fts & 0xFF, 0), (rate & 0xFF, 0), (tc & 0xFF, 0)] + \
           [(ident & 0xFF, 0)] * 10


def ts1(**kw):
    return ts(C.TS1_ID, **kw)


def ts2(**kw):
    return ts(C.TS2_ID, **kw)


def skp_os(n=C.SKP_TX_COUNT):
    return [(C.COM, 1)] + [(C.SKP, 1)] * n


def eios():
    return list(C.EIOS)


def idle(n):
    """Logical Idle: the Idle data Symbol 00h (§4.2.2 p.195), scrambled on the wire."""
    return [(0x00, 0)] * n


def parse_ts(syms):
    """Fields of a 16-Symbol training set, or None if it is not a TS1 / TS2.

    Returns dict(kind, link, lane, n_fts, rate, tc, inverted). link / lane are
    None for PAD; 'K..' strings for any other K code (malformed)."""
    if len(syms) < C.TS_LEN or syms[0] != (C.COM, 1):
        return None
    ids = syms[6:16]
    if any(k for _, k in ids) or len({b for b, _ in ids}) != 1:
        return None
    ident = ids[0][0]
    kind = {C.TS1_ID: 'TS1', C.TS2_ID: 'TS2', C.TS1_ID_INV: 'TS1', C.TS2_ID_INV: 'TS2'}.get(ident)
    if kind is None:
        return None

    def num(sym):
        b, k = sym
        if k:
            return None if b == C.PAD else 'K%02x' % b
        return b
    return dict(kind=kind, link=num(syms[1]), lane=num(syms[2]),
                n_fts=None if syms[3][1] else syms[3][0],
                rate=None if syms[4][1] else syms[4][0],
                tc=None if syms[5][1] else syms[5][0],
                inverted=ident in (C.TS1_ID_INV, C.TS2_ID_INV))


# ------------------------------------------------------------------ framing
def frame_tlp(seq, tlp, nullify=False):
    body = CRC.seq_bytes(seq) + bytes(tlp)
    crc = CRC.nullified_lcrc(seq, tlp) if nullify else CRC.lcrc(seq, tlp)
    return [(C.STP, 1)] + [(b, 0) for b in body + crc] + [((C.EDB if nullify else C.END), 1)]


def frame_dllp(b4):
    b4 = bytes(b4)
    return [(C.SDP, 1)] + [(b, 0) for b in b4 + CRC.dllp_crc(b4)] + [(C.END, 1)]


# -------------------------------------------------------------------- DLLPs
def dllp_acknak(typ, seq, rsvd1=0, rsvd2=0):
    seq &= 0xFFF
    return bytes([typ, rsvd1 & 0xFF, ((rsvd2 & 0xF) << 4) | (seq >> 8), seq & 0xFF])


def dllp_ack(seq, **kw):
    return dllp_acknak(C.DLLP_ACK, seq, **kw)


def dllp_nak(seq, **kw):
    return dllp_acknak(C.DLLP_NAK, seq, **kw)


def dllp_fc(typ, hdr, data, vc=0, rsvd1=0, rsvd2=0):
    """typ: the full type byte's high nibble value (e.g. 0x40 InitFC1-P)."""
    hdr &= 0xFF
    data &= 0xFFF
    return bytes([(typ & 0xF8) | (vc & 0x7),
                  ((rsvd1 & 0x3) << 6) | (hdr >> 2),
                  ((hdr & 0x3) << 6) | ((rsvd2 & 0x3) << 4) | (data >> 8),
                  data & 0xFF])


def initfc1(fct, hdr, data, **kw):
    return dllp_fc(C.DLLP_INITFC1[fct], hdr, data, **kw)


def initfc2(fct, hdr, data, **kw):
    return dllp_fc(C.DLLP_INITFC2[fct], hdr, data, **kw)


def updatefc(fct, hdr, data, **kw):
    return dllp_fc(C.DLLP_UPDATEFC[fct], hdr, data, **kw)


_FC_BY_HIGH = {}
for _phase, _tbl in (('InitFC1', C.DLLP_INITFC1), ('InitFC2', C.DLLP_INITFC2), ('UpdateFC', C.DLLP_UPDATEFC)):
    for _fct, _t in _tbl.items():
        _FC_BY_HIGH[_t] = (_phase, _fct)


def parse_dllp(b6):
    """Decode six DLLP bytes. Returns a dict with 'crc_ok', 'kind' and fields.

    kind: Ack, Nak, InitFC1, InitFC2, UpdateFC, PM, Vendor, Reserved.
    'rsvd_nonzero' is True when a field Figures 3-5..3-8 mark Reserved is set.
    """
    b6 = bytes(b6)
    d = dict(raw=b6, crc_ok=CRC.dllp_ok(b6), type=b6[0])
    t = b6[0]
    if t in (C.DLLP_ACK, C.DLLP_NAK):
        d.update(kind='Ack' if t == C.DLLP_ACK else 'Nak',
                 seq=((b6[2] & 0x0F) << 8) | b6[3],
                 rsvd_nonzero=bool(b6[1] or (b6[2] & 0xF0)))
    elif (t & 0xF8) in _FC_BY_HIGH:
        phase, fct = _FC_BY_HIGH[t & 0xF8]
        d.update(kind=phase, fc=fct, vc=t & 0x7,
                 hdr=((b6[1] & 0x3F) << 2) | (b6[2] >> 6),
                 data=((b6[2] & 0x0F) << 8) | b6[3],
                 rsvd_nonzero=bool((b6[1] & 0xC0) or (b6[2] & 0x30)))
    elif t in (C.DLLP_PM_ENTER_L1, C.DLLP_PM_ENTER_L23, C.DLLP_PM_AS_REQ_L1, C.DLLP_PM_REQ_ACK):
        d.update(kind='PM', rsvd_nonzero=bool(b6[1] or b6[2] or b6[3]))
    elif t == C.DLLP_VENDOR:
        d.update(kind='Vendor', rsvd_nonzero=False)
    else:
        d.update(kind='Reserved', rsvd_nonzero=False)
    return d
