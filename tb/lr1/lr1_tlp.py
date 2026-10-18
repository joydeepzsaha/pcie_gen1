"""lr1_tlp -- TLP headers, built and parsed from the spec's figures.

Author: Kourosh Ghahramani
Silicon Systems Research Lab, University of Washington

Base 2.1 p.57 "Fields Present in All TLP Headers" (byte 0 Fmt[7:5] Type[4:0];
byte 1 TC[6:4], Attr[2] bit 2, TH bit 0; byte 2 TD bit 7, EP bit 6, Attr[1:0]
bits 5:4, AT bits 3:2, Length[9:8] bits 1:0; byte 3 Length[7:0]); Table 2-2
p.57 (Fmt), Table 2-3 p.58 (Fmt and Type); Request headers: byte 4-5 Requester
ID, byte 6 Tag, byte 7 {Last DW BE[7:4], 1st DW BE[3:0]} (pp.77-80);
Configuration (p.80): byte 8 Bus, byte 9 {Device[7:3], Function[2:0]}, byte 10
{Reserved[7:4], Ext Register[3:0]}, byte 11 {Register[7:2], R[1:0]};
Completion (p.98): bytes 4-5 Completer ID, byte 6 {Status[7:5], BCM[4],
Byte Count[11:8]}, byte 7 Byte Count[7:0], bytes 8-9 Requester ID, byte 10
Tag, byte 11 {R[7], Lower Address[6:0]}; Table 2-28 p.98 (Completion Status).
Multi-byte fields are big-endian (byte 0 is transmitted first, §2.2 figures).

Independent of the RTL; known answers in test_lr1_kat.py (MindShare Fig. A-6,
pcievhost, cocotbext-pcie).
"""
import lr1_consts as C


def bdf(bus, dev, fn):
    return ((bus & 0xFF) << 8) | ((dev & 0x1F) << 3) | (fn & 0x7)


def dw0(fmt, typ, length, tc=0, td=0, ep=0, attr=0, at=0, th=0):
    return bytes([((fmt & 0x7) << 5) | (typ & 0x1F),
                  ((tc & 0x7) << 4) | (((attr >> 2) & 1) << 2) | (th & 1),
                  ((td & 1) << 7) | ((ep & 1) << 6) | ((attr & 0x3) << 4) | ((at & 0x3) << 2) |
                  ((length >> 8) & 0x3),
                  length & 0xFF])


def cfg_req(write, type1, req_id, tag, bus, dev, fn, reg, first_be=0xF, data=None, td=0):
    """Configuration Request (p.80). reg = byte address of the Dword (0-0xFFC)."""
    fmt = C.FMT_3DW_D if write else C.FMT_3DW
    typ = C.TYPE_CFG1 if type1 else C.TYPE_CFG0
    h = dw0(fmt, typ, 1, td=td)
    h += bytes([req_id >> 8, req_id & 0xFF, tag & 0xFF, first_be & 0xF])
    h += bytes([bus & 0xFF, ((dev & 0x1F) << 3) | (fn & 0x7), (reg >> 8) & 0x0F, reg & 0xFC])
    if write:
        h += bytes(data)
    return h


def cpl(completer_id, status, req_id, tag, byte_count, lower_addr=0, data=None, td=0, bcm=0):
    """Completion (p.98). data None -> Cpl; bytes -> CplD (Length from len)."""
    if data is None:
        h = dw0(C.FMT_3DW, C.TYPE_CPL, 0, td=td)
    else:
        assert len(data) % 4 == 0
        h = dw0(C.FMT_3DW_D, C.TYPE_CPL, (len(data) // 4) & 0x3FF, td=td)
    bc = byte_count & 0xFFF                  # 4096 encodes as 000h (p.97)
    h += bytes([completer_id >> 8, completer_id & 0xFF,
                ((status & 0x7) << 5) | ((bcm & 1) << 4) | (bc >> 8), bc & 0xFF])
    h += bytes([req_id >> 8, req_id & 0xFF, tag & 0xFF, lower_addr & 0x7F])
    if data is not None:
        h += bytes(data)
    return h


def mem_req(write, req_id, tag, addr, length_dw, first_be=0xF, last_be=None, data=None, td=0):
    """Memory Request, 32- or 64-bit addressing (pp.77-78)."""
    four = addr >= (1 << 32)
    if last_be is None:
        last_be = 0 if length_dw == 1 else 0xF
    fmt = (C.FMT_4DW_D if four else C.FMT_3DW_D) if write else (C.FMT_4DW if four else C.FMT_3DW)
    h = dw0(fmt, C.TYPE_MEM, length_dw & 0x3FF, td=td)
    h += bytes([req_id >> 8, req_id & 0xFF, tag & 0xFF, ((last_be & 0xF) << 4) | (first_be & 0xF)])
    h += addr.to_bytes(8, 'big')[:-1] + bytes([addr & 0xFC]) if four else \
        bytes([(addr >> 24) & 0xFF, (addr >> 16) & 0xFF, (addr >> 8) & 0xFF, addr & 0xFC])
    if write:
        h += bytes(data)
    return h


def parse(tlp):
    """Header fields of a TLP (bytes). Returns a dict; 'name' is the Table 2-3
    name or 'Reserved'. 'payload' holds the data, 'digest' the ECRC if TD."""
    t = bytes(tlp)
    if len(t) < 12:
        return dict(name='Short', raw=t)
    fmt, typ = t[0] >> 5, t[0] & 0x1F
    length = ((t[2] & 0x3) << 8) | t[3]
    d = dict(raw=t, fmt=fmt, type=typ, tc=(t[1] >> 4) & 7, td=t[2] >> 7, ep=(t[2] >> 6) & 1,
             attr=((t[2] >> 4) & 3) | (((t[1] >> 2) & 1) << 2), at=(t[2] >> 2) & 3,
             length=length, rsvd_dw0=bool(t[1] & 0x8A))   # byte 1 bits 7, 3, 1 (p.57)
    hlen = 16 if fmt in (C.FMT_4DW, C.FMT_4DW_D) else 12
    has_data = fmt in (C.FMT_3DW_D, C.FMT_4DW_D)
    ndata = (length or 1024) * 4 if has_data else 0
    d['hlen'] = hlen
    d['payload'] = t[hlen:hlen + ndata]
    d['digest'] = t[hlen + ndata:hlen + ndata + 4] if d['td'] else b''
    d['size_ok'] = len(t) == hlen + ndata + (4 if d['td'] else 0)
    if typ in (C.TYPE_CFG0, C.TYPE_CFG1) and fmt in (C.FMT_3DW, C.FMT_3DW_D):
        d.update(name=('CfgWr' if has_data else 'CfgRd') + ('1' if typ == C.TYPE_CFG1 else '0'),
                 req_id=(t[4] << 8) | t[5], tag=t[6], last_be=t[7] >> 4, first_be=t[7] & 0xF,
                 bus=t[8], dev=t[9] >> 3, fn=t[9] & 7, reg=((t[10] & 0xF) << 8) | (t[11] & 0xFC))
    elif typ == C.TYPE_CPL and fmt in (C.FMT_3DW, C.FMT_3DW_D):
        bc = ((t[6] & 0xF) << 8) | t[7]
        d.update(name='CplD' if has_data else 'Cpl', completer_id=(t[4] << 8) | t[5],
                 status=t[6] >> 5, bcm=(t[6] >> 4) & 1, byte_count=bc or 4096,
                 req_id=(t[8] << 8) | t[9], tag=t[10], lower_addr=t[11] & 0x7F)
    elif typ == C.TYPE_MEM and fmt in (C.FMT_3DW, C.FMT_4DW, C.FMT_3DW_D, C.FMT_4DW_D):
        addr = int.from_bytes(t[8:16] if hlen == 16 else t[8:12], 'big') & ~0x3
        d.update(name='MWr' if has_data else 'MRd', req_id=(t[4] << 8) | t[5], tag=t[6],
                 last_be=t[7] >> 4, first_be=t[7] & 0xF, addr=addr)
    elif typ == C.TYPE_IO and fmt in (C.FMT_3DW, C.FMT_3DW_D):
        d.update(name='IOWr' if has_data else 'IORd', req_id=(t[4] << 8) | t[5], tag=t[6])
    elif (typ & 0x18) == 0x10 and fmt in (C.FMT_4DW, C.FMT_4DW_D):
        d.update(name='MsgD' if has_data else 'Msg', routing=typ & 7, req_id=(t[4] << 8) | t[5],
                 tag=t[6], msg_code=t[7])
    else:
        d.update(name='Reserved')
    return d
