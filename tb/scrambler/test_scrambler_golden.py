"""verilate_scrambler_golden -- gen1_scramble against the Base 2.1 sec 4.2.3 golden, two classes.

Toplevel: gen1_scramble.  Every expected byte comes from golden_fuzz_common.golden(), which applies sec 4.2.3 p.199
Symbol by Symbol (COM initializes the LFSR; SKP does not advance it; K and TS1/TS2 Symbols are not scrambled); none
is captured from the DUT.  Stimulus: an exhaustive directed sweep plus SEEDS deterministic fuzz streams per
(width, mode, idle-clock rate), at both pipe widths the module supports (8 and 16) and with the first COM in every
byte position.  Each published word is read post-edge on the clock that presented a Symbol (a bare read after
RisingEdge is pre-edge), and word m is published DEPTH presented clocks later; data_valid_o is not used, because at
this toplevel it holds through an idle clock (test_gen1_align.py).

Row 1, #29: the SKP-end arm.  Every TS Symbol of the first TS after a SKP Ordered Set is sent unscrambled, and a data
Symbol between that SKP Ordered Set and the TS's COM, in the COM's word, is scrambled.  Coverage asserted: every
(COM byte, SKP count 1-5) cell.  Red at d19a557 on the TS limb.
Row 2, the 16-Symbol end arm (§63 #34): the data Symbol right after a TS's 16th Symbol, in the word of the next TS's
COM, is scrambled.  Red at d19a557 (the arm's word-wide COM guard leaves it unscrambled).
Row 1 is green from #29's fix (rev 3); row 2 is expect_fail and PINNED until §63 #34's.

Two more data-Symbol classes are measured and logged, not asserted: one in the word of a SKP Ordered Set's last SKP
(§63 #36) and one before a COM that opens a SKP Ordered Set (§63 #35).  Both are registered; each alters one Logical
Idle Symbol's value, which receivers ignore (sec 4.2.2 p.195).
"""
import collections

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, RisingEdge, Timer

from golden_fuzz_common import (build, classify, directed, golden, lines_for, per_symbol)

SEEDS = 400
GAPS = (0.0, 0.3)


def pinned_red(dut, row, state, detail=""):
    """§22.93 -- an expect_fail row must fail AT its one named, pinned assertion.

    cocotb's expect_fail turns ANY exception into a PASS, so a row can be red
    for a reason unrelated to the defect it pins and the gate cannot tell.
    Everything before the pinned assertion runs inside a try/except; an
    exception there is logged NOT_REACHED and the row RETURNS NORMALLY, which
    under expect_fail is a gate FAIL.  Then the REACHED marker, then the pinned
    assertion -- the only statement allowed to raise.  sweep43.sh copies these
    into its .diag as PINNED| rows.
    """
    dut._log.info("PINNED_RED|%s|%s|%s", row, state, detail)


async def reset(dut, width):
    dut.rst_i.value = 1
    dut.data_valid_i.value = 0
    dut.data_in_i.value = 0
    dut.data_k_in_i.value = 0
    dut.pipe_width_i.value = width
    await ClockCycles(dut.clk_i, 3)
    dut.rst_i.value = 0


async def drive(dut, lines):
    """One (valid, k, data) per clock; returns data_out_o after every clock that presented a Symbol."""
    out = []
    for v, k, d in lines:
        dut.data_valid_i.value = v
        dut.data_k_in_i.value = k
        dut.data_in_i.value = d
        await RisingEdge(dut.clk_i)
        await Timer(1, units="ps")
        if v:
            out.append(int(dut.data_out_o.value))
    return out


async def measure(dut, streams, width):
    """streams: (name, syms, tags, gap, seed). Returns per-class totals, mismatches, coverage and examples."""
    pop, bad, cov, eg = collections.Counter(), collections.Counter(), collections.Counter(), {}
    for name, syms, tags, gap, seed in streams:
        await reset(dut, width)
        lines = lines_for(syms, width, gap, seed)
        pub = await drive(dut, lines)
        got = per_symbol(pub, width, sum(1 for l in lines if l[0]))
        g = golden(syms)
        cls, cv, first = classify(syms, tags, width)
        assert len(got) >= len(syms) - 4 * (width // 8), f"{name}: {len(got)} Symbols published of {len(syms)}"
        for i in range(first, min(len(got), len(syms))):
            c = cls[i]
            pop[c] += 1
            if cv[i] is not None:
                cov[(c, cv[i])] += 1
            if got[i] != g[i]:
                bad[c] += 1
                eg.setdefault(c, f"{name} Symbol {i}: sent {got[i]:02x}, golden {g[i]:02x}, input {syms[i][0]:02x}")
    return pop, bad, cov, eg


def streams_for(width, modes):
    s = [(n, syms, tags, 0.0, 1) for n, syms, tags in directed(width)]
    for mode in modes:
        for gap in GAPS:
            for seed in range(SEEDS):
                syms, tags, _ = build(seed * 1000 + width, width, mode)
                s.append((f"w{width}_{mode}_g{int(gap * 10)}_s{seed}", syms, tags, gap, seed))
    return s


def log(dut, width, pop, bad, eg):
    for c in sorted(pop):
        dut._log.info(f"width {width} class {c:13s} Symbols {pop[c]:7d} mismatched {bad[c]:6d} {eg.get(c, '')}")


@cocotb.test()
async def test_golden_post_skp_ts_and_d_before_com(dut):
    """#29 (Base 2.1 sec 4.2.3 p.199, sec 4.2.4.1 p.201): after a SKP Ordered Set, the next TS is sent unscrambled at
    every COM byte and SKP count 1-5, and a data Symbol in that TS's COM word is scrambled.

    Red until #29's fix: the SKP-end arm re-enabled scrambling under a TS whose COM followed the SKP Ordered Set, and
    192,585 of 210,605 such TS Symbols at 8 bits and 203,555 of 209,440 at 16 were XORed.

    Asserts: no ts_post_skp Symbol (8 and 16 bits) and no d_skp_com Symbol (16 bits) differs from the golden.
    Non-vacuity: every (COM byte, SKP count 1-5) cell holds a TS after a SKP Ordered Set at both widths, and at 16 bits
    a [D, COM] word follows a SKP Ordered Set of every SKP count."""
    row = 'test_golden_post_skp_ts_and_d_before_com'
    cocotb.start_soon(Clock(dut.clk_i, 10, units="ns").start())
    fails, counts = [], []
    for width in (8, 16):
        pop, bad, cov, eg = await measure(dut, streams_for(width, ('train', 'mix')), width)
        log(dut, width, pop, bad, eg)
        cells = [(b, n) for b in range(width // 8) for n in range(1, 6)]
        empty = [c for c in cells if cov[('ts_post_skp', c)] == 0]
        assert not empty, f"width {width}: no TS after a SKP Ordered Set at (COM byte, SKP count) {empty}"
        if width == 16:
            empty = [n for n in range(1, 6) if cov[('d_skp_com', (1, n))] == 0]
            assert not empty, f"width 16: no [D, COM] word after a SKP Ordered Set of {empty} SKP"
        for c in ('ts_post_skp', 'd_skp_com'):
            if pop[c]:
                counts.append(f"width {width} {c} {bad[c]} of {pop[c]}")
            if bad[c]:
                fails.append(f"width {width} {c}: {bad[c]} of {pop[c]} Symbols differ from the golden; first: {eg[c]}")
    dut._log.info("MEASURE|%s|%s", row, "; ".join(counts))
    assert not fails, "; ".join(fails)


@cocotb.test(expect_fail=True)
async def test_golden_end_arm_d_before_com(dut):
    """The 16-Symbol end arm, §63 #34 (Base 2.1 sec 4.2.3 p.199): the data Symbol after a TS's 16th Symbol, in the next
    TS's COM word, is scrambled. 16 bits only: at 8 bits a data Symbol never shares a word with a COM.

    Non-vacuity, before the pinned point: at least 20 such data Symbols.

    PINNED: none of them differs from the golden."""
    row = 'test_golden_end_arm_d_before_com'
    cocotb.start_soon(Clock(dut.clk_i, 10, units="ns").start())
    width = 16
    try:
        pop, bad, cov, eg = await measure(dut, streams_for(width, ('mix',)), width)
        log(dut, width, pop, bad, eg)
        n = sum(v for (c, _), v in cov.items() if c == 'd_ts_com')
        assert n >= 20, f"only {n} data Symbols between a TS's 16th Symbol and the next TS's COM in one word"
        detail = f"width 16 d_ts_com {bad['d_ts_com']} of {pop['d_ts_com']}"
    except Exception as exc:
        pinned_red(dut, row, 'NOT_REACHED', repr(exc))
        return
    pinned_red(dut, row, 'REACHED', detail)
    assert bad['d_ts_com'] == 0, (f"{bad['d_ts_com']} of {pop['d_ts_com']} data Symbols after a TS, in the next "
                                  f"TS's COM word, differ from the golden; first: {eg['d_ts_com']}")
