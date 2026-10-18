"""lr1_scrambler -- the 2.5 GT/s scrambler, written from the spec text.

Author: Kourosh Ghahramani
Silicon Systems Research Lab, University of Washington

Base 2.1 §4.2.3 pp.198-199 and Figure 4-10 p.200 ("LFSR with Scrambling
Polynomial"): G(X) = X^16 + X^5 + X^4 + X^3 + 1, seed FFFFh. The character is
processed serially: LFSR output D15 is XORed with data bit D0, the LFSR is
advanced, and so on to D7. The figure's feedback taps D15 into D0, D3, D4 and
D5.

Symbol rules (p.199 bullet list; Appendix C.1 code pp.697-698):
  - COM initialises the LFSR (to FFFFh) and is not scrambled; the next Symbol
    is scrambled with the seed ("Immediately after a COM exits the Transmit
    LFSR, the LFSR ... is initialized"; App. C returns before advancing).
  - The LFSR advances eight serial shifts for every Symbol except SKP.
  - K codes are not scrambled.
  - Data Symbols inside a TS1 / TS2 (Symbols 1-15) are not scrambled, but do
    advance the LFSR (App. C: the TrainingSequence flag gates only the XOR).
  - Scrambling is never disabled by this far end (the RC does not implement
    Disable Scrambling; ORACLES_LTSSM C20).

The scrambler and descrambler are the same function (App. C p.698, "NOTE THAT
THE DESCRAMBLE ROUTINE IS IDENTICAL"). The receive side must know which data
Symbols are TS bodies: after a COM, a TS is the only Ordered Set whose Symbol 1
is a data Symbol or PAD (SKP OS: SKP; EIOS: IDL; FTS OS: FTS -- Tables 4-2..4-4,
§4.2.7), so RxDescrambler marks the 15 Symbols after such a COM as TS body.

Known answers: test_lr1_kat.py (Appendix C tables p.700, and PG213's own
transmit stream, ruling C).
"""
import lr1_consts as C

_STEP_CACHE = {}


def lfsr_byte(lfsr):
    """(xor mask for one data byte, LFSR after eight serial shifts)."""
    hit = _STEP_CACHE.get(lfsr)
    if hit is not None:
        return hit
    l, mask = lfsr, 0
    for i in range(8):
        mask |= ((l >> 15) & 1) << i            # D15 meets data bit i
        fb = (l >> 15) & 1
        l = (l << 1) & 0xFFFF
        if fb:
            l ^= 0x0039                          # taps into D0, D3, D4, D5
    _STEP_CACHE[lfsr] = (mask, l)
    return mask, l


class Scrambler:
    """One direction's LFSR. symbol(byte, k, in_ts) returns the processed byte."""

    def __init__(self):
        self.lfsr = 0xFFFF

    def symbol(self, byte, k, in_ts=False):
        if k and byte == C.COM:
            self.lfsr = 0xFFFF
            return byte
        if k and byte == C.SKP:
            return byte
        mask, nxt = lfsr_byte(self.lfsr)
        out = byte if (k or in_ts) else (byte ^ mask)
        self.lfsr = nxt
        return out


class TxScrambler(Scrambler):
    """The transmit side: the caller says which Symbols are TS bodies."""


class RxDescrambler(Scrambler):
    """The receive side: finds TS bodies itself (see the module docstring).

    feed(byte, k) -> (plain_byte, k, in_ts)
    """

    def __init__(self):
        super().__init__()
        self._after_com = False
        self._ts_left = 0

    def feed(self, byte, k):
        if k and byte == C.COM:
            self._after_com = True
            self._ts_left = 0
            return self.symbol(byte, k), k, False
        if self._after_com:
            self._after_com = False
            if (not k) or byte == C.PAD:
                self._ts_left = C.TS_LEN - 1
        in_ts = self._ts_left > 0
        if in_ts:
            self._ts_left -= 1
        return self.symbol(byte, k, in_ts), k, in_ts
