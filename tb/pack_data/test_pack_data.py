"""§63 #7g-1 — `pack_data`'s first unit rows.

WHAT THIS MODULE ACTUALLY IS, measured rather than read: a BYTE GATHERER.  Only
`ST_IDLE` is live — `ST_SEND_DATA`, `ST_GEN3_TLP`, `ST_GEN3_DLLP` and
`ST_LAST_DATA` are declared and their bodies are entirely commented out — so the
five-state enum describes an FSM that does not exist.  In `ST_IDLE` it copies
`pipe_width_i >> 3` bytes per valid cycle into a `DATA_WIDTH/8`-byte accumulator
and raises `data_valid_o` for one cycle when the accumulator fills.

⚠️ These are CHARACTERISATION rows, not spec-golden ones.  They pin what the
module does today so a change to it is visible; they do not claim Base 2.1 says
it should.  A spec-golden `pack_data` bench would be its own rung.
"""
import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, Timer

CLK_NS = 10
PIPE_W = 8          # Gen1 PIPE width, bits — 1 byte per lane per cycle
LANES = 1
BPT = 4             # DATA_WIDTH/8 — bytes the accumulator holds

SDP, END, STP, COM = 0x5C, 0xFD, 0xFB, 0xBC


async def setup(dut, link_up=1):
    cocotb.start_soon(Clock(dut.clk_i, CLK_NS, units="ns").start())
    dut.rst_i.value = 1
    dut.phy_link_up_i.value = 0
    dut.lane_reverse_i.value = 0
    dut.curr_data_rate_i.value = 0
    dut.data_i.value = 0
    dut.data_valid_i.value = 0
    dut.data_k_i.value = 0
    dut.sync_header_i.value = 0
    dut.pipe_width_i.value = PIPE_W
    dut.num_active_lanes_i.value = LANES
    for _ in range(3):
        await RisingEdge(dut.clk_i)
    dut.rst_i.value = 0
    dut.phy_link_up_i.value = link_up
    await RisingEdge(dut.clk_i)
    await Timer(1, units="ps")


async def present(dut, byte, k, valid=1):
    """Present one byte-lane beat and step one clock."""
    dut.data_i.value = byte
    dut.data_k_i.value = (1 if k else 0)
    dut.data_valid_i.value = valid
    await RisingEdge(dut.clk_i)
    await Timer(1, units="ps")
    return int(dut.data_valid_o.value) & 1, int(dut.data_o.value), int(dut.data_k_o.value)


@cocotb.test()
async def t1_gathers_four_bytes_then_strobes_valid_once(dut):
    """Four valid input beats produce exactly ONE output word.

    Non-vacuity (§22.82): the row fails if valid never rises, which would make
    "exactly one" true for the wrong reason.
    """
    await setup(dut)
    rises = []
    for i, b in enumerate((0x11, 0x22, 0x33, 0x44)):
        v, d, k = await present(dut, b, 0)
        if v:
            rises.append((i, d, k))
    # drain a few idle cycles; valid must not rise again
    for _ in range(4):
        v, d, k = await present(dut, 0, 0, valid=0)
        assert not v, "data_valid_o rose with data_valid_i low"
    assert len(rises) == 1, (
        f"expected exactly one data_valid_o strobe over four gathered bytes, "
        f"saw {len(rises)} at {[r[0] for r in rises]}")
    _, word, _ = rises[0]
    dut._log.info(f"7G1[pack t1] gathered word=0x{word & 0xFFFFFFFF:08x}")


@cocotb.test()
async def t2_bytes_appear_in_presentation_order(dut):
    """The gathered word carries the four bytes in the order presented."""
    await setup(dut)
    seq = (0xA1, 0xB2, 0xC3, 0xD4)
    word = None
    for b in seq:
        v, d, _ = await present(dut, b, 0)
        if v:
            word = d & 0xFFFFFFFF
    assert word is not None, "no word was published for four valid beats"
    got = [(word >> (8 * i)) & 0xFF for i in range(BPT)]
    assert got == list(seq), (
        f"byte order: presented {[hex(x) for x in seq]}, "
        f"word 0x{word:08x} holds {[hex(x) for x in got]}")


@cocotb.test()
async def t3_k_flags_travel_with_their_bytes(dut):
    """SDP and END keep their K flag, in the byte lane they were presented in.

    This is the row behind test_pcie_fullstack.py's claim that `pack_data`
    "preserves SDP and END exactly" — which until now was a reading of the
    source, not a measurement.
    """
    await setup(dut)
    seq = ((SDP, 1), (0x5A, 0), (0xA5, 0), (END, 1))
    word = kout = None
    for b, k in seq:
        v, d, kk = await present(dut, b, k)
        if v:
            word, kout = d & 0xFFFFFFFF, kk & 0xF
    assert word is not None, "no word published"
    got_b = [(word >> (8 * i)) & 0xFF for i in range(BPT)]
    got_k = [(kout >> i) & 1 for i in range(BPT)]
    assert got_b == [s[0] for s in seq], f"bytes {[hex(x) for x in got_b]}"
    assert got_k == [s[1] for s in seq], (
        f"K flags {got_k}, expected {[s[1] for s in seq]} — a K code and its "
        f"flag parted company inside the gatherer")
    dut._log.info(f"7G1[pack t3] word=0x{word:08x} k={got_k}")


@cocotb.test()
async def t4_link_down_publishes_nothing(dut):
    """With `phy_link_up_i` low the gatherer neither accumulates nor publishes.

    The positive control (§22.81) is t1, which uses the identical stimulus with
    the link up and does publish.
    """
    await setup(dut, link_up=0)
    for b in (0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88):
        v, _, _ = await present(dut, b, 0)
        assert not v, "data_valid_o rose while phy_link_up_i was low"


@cocotb.test()
async def t5_valid_is_a_one_cycle_strobe_not_a_level(dut):
    """Across two full words, valid rises exactly twice.

    `D.data_valid = '0` is reasserted every cycle at the top of the comb block,
    so the output is a strobe.  If someone turns it into a level this row goes
    red — and a level would make every downstream `if (data_valid)` fire four
    times per word.
    """
    await setup(dut)
    n = 0
    for b in range(8):
        v, _, _ = await present(dut, 0x10 + b, 0)
        n += (1 if v else 0)
    assert n == 2, f"valid rose {n} times over two words, expected exactly 2"
