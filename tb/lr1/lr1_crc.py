"""lr1_crc -- the two link CRCs, written literally from the spec text.

Author: Kourosh Ghahramani
Silicon Systems Research Lab, University of Washington

  dllp_crc(b4)        16-bit DLLP CRC field as the two bytes that follow the
                      four DLLP bytes on the wire.
                      Base 2.1 §3.4.1 p.167, Table 3-2 and Figure 3-11 p.168.
  lcrc(seq, tlp)      32-bit LCRC field as the four bytes that follow the TLP.
                      Base 2.1 §3.5.2.1 pp.171-173, Table 3-3 p.172, figure
                      "Calculation of LCRC" p.173.
  nullified_lcrc      the inverted LCRC of a nullified TLP, §3.5.2.1 p.174.

Both are the bit-serial form the spec describes: a register shifted MSB-first,
fed with bit 0 of each byte first, starting from the seed; the result is
complemented and then mapped into the field with each byte's bits reversed
(Tables 3-2 / 3-3: result bit 8g+j -> field bit 8g+(7-j)). The field is placed
on the wire most significant byte first (Figures 3-5..3-8: "Byte 4 > 16-bit
CRC"; the LCRC figure: LCRC byte 0 = field[31:24]).

Nothing here is derived from the RTL. Known answers: test_lr1_kat.py.
"""
import lr1_consts as C


def _serial(data, poly, width, seed):
    top = 1 << (width - 1)
    mask = (1 << width) - 1
    c = seed
    for b in data:
        for i in range(8):                       # bit 0 -> bit 7 of each byte
            fb = ((b >> i) & 1) ^ (1 if c & top else 0)
            c = (c << 1) & mask
            if fb:
                c ^= poly
    return c


def _map_field(result, width):
    """Tables 3-2 / 3-3: result bit 8g+j goes to field bit 8g+(7-j)."""
    f = 0
    for k in range(width):
        if (result >> k) & 1:
            g, j = divmod(k, 8)
            f |= 1 << (8 * g + 7 - j)
    return f


def dllp_crc(b4):
    """Two CRC bytes for the four DLLP bytes b4 (byte 4 = field[15:8])."""
    b4 = bytes(b4)
    if len(b4) != 4:
        raise ValueError('a DLLP CRC covers exactly 4 bytes')
    r = _serial(b4, C.DLLP_CRC_POLY, 16, C.DLLP_CRC_SEED) ^ 0xFFFF
    f = _map_field(r, 16)
    return bytes([(f >> 8) & 0xFF, f & 0xFF])


def seq_bytes(seq):
    """The two bytes after STP: {4'b0000 Reserved, Seq[11:8]}, Seq[7:0]
    (Base 2.1 figure "Calculation of LCRC" p.173, framing p.197)."""
    seq &= 0xFFF
    return bytes([(seq >> 8) & 0x0F, seq & 0xFF])


def lcrc_over(data):
    """The LCRC field bytes over `data` = sequence bytes + TLP."""
    r = _serial(bytes(data), C.LCRC_POLY, 32, C.LCRC_SEED) ^ 0xFFFFFFFF
    return _map_field(r, 32).to_bytes(4, 'big')


def lcrc(seq, tlp):
    return lcrc_over(seq_bytes(seq) + bytes(tlp))


def nullified_lcrc(seq, tlp):
    """§3.5.2.1 p.174: a nullified TLP carries the LCRC inverted, ends in EDB."""
    return bytes(x ^ 0xFF for x in lcrc(seq, tlp))


def dllp_ok(b6):
    return len(b6) == 6 and dllp_crc(b6[:4]) == bytes(b6[4:6])


def lcrc_ok(seqb_tlp_lcrc):
    """True when the last four bytes are the LCRC of everything before them."""
    d = bytes(seqb_tlp_lcrc)
    return len(d) >= 6 and lcrc_over(d[:-4]) == d[-4:]


def lcrc_nullified(seqb_tlp_lcrc):
    d = bytes(seqb_tlp_lcrc)
    return len(d) >= 6 and bytes(x ^ 0xFF for x in lcrc_over(d[:-4])) == d[-4:]
