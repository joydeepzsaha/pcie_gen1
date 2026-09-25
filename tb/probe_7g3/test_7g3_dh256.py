"""sec 63 #7g-3 Phase 1 -- C-7G3-7: is data_handler's registered-word END arm
(registered as data_handler:256, now :297-298) ever wrong on a legal frame?

MEASUREMENT, not a gate row: every cell prints one `M7G3|...` line and the test
asserts only that the apparatus ran.  Scoring is done offline from those lines.

Cells: DLLP (SDP + 6 + END = 8 Symbols) and TLP n = 3, 4, 5 DW
(STP + 2 + 4n + 4 + END = 4n + 8 Symbols), each at start byte s = 0..3, followed
by eight Symbols of Logical Idle (data 00h, not K: Base 2.1 §4.2.3 p.199, and the
L0 stream since #7j-2), then padded to a word boundary with more Logical Idle.

Witness of the arm: its own guard, read off the RTL each cycle in ReadOnly --
ST_TX (=1), data_handler_axis_tready, data_valid_i, !data_start_r, and an END/EDB
K in data_r at byte b.  The same guard probe_7g3_stack.sv's `R` event uses.
Runs on verilate_data_handler's toplevel (data_handler, MAX_NUM_LANES=1,
USER_WIDTH=5, --public-flat-rw) via run_probe_7g3.sh MODULE_OVERRIDE.
"""
import cocotb
from cocotb.triggers import RisingEdge, ReadOnly
from test_data_handler import TB, word, kmask, SDP, STP, END

LIDL = 0x00  # Logical Idle: a data Symbol 00h, K = 0


def frame_syms(kind, n, s):
    start = SDP if kind == 'DLLP' else STP
    plen = 6 if kind == 'DLLP' else 2 + 4 * n + 4
    payload = [((0x10 + i) & 0xFF) for i in range(plen)]
    syms = [(LIDL, 0)] * s + [(start, 1)] + [(p, 0) for p in payload] + [(END, 1)]
    syms += [(LIDL, 0)] * 8
    while len(syms) % 4:
        syms.append((LIDL, 0))
    words = []
    for w in range(0, len(syms), 4):
        c = syms[w:w + 4]
        words.append((word(c[0][0], c[1][0], c[2][0], c[3][0]),
                      kmask(*[i for i in range(4) if c[i][1]])))
    return payload, words


async def run_cell(dut, tb, kind, n, s):
    await tb.reset()
    payload, words = frame_syms(kind, n, s)
    beats, arm = [], []
    done = False

    async def watch():
        while not done:
            await RisingEdge(dut.clk_i)
            await ReadOnly()
            if (int(dut.curr_state.value) == 1 and int(dut.data_handler_axis_tready.value) == 1
                    and int(dut.data_valid_i.value) != 0 and int(dut.data_start_r.value) == 0):
                dr, kr = int(dut.data_r.value), int(dut.data_k_r.value)
                for b in range(4):
                    if (kr >> b) & 1 and ((dr >> (8 * b)) & 0xFF) in (0xFD, 0xFE):
                        arm.append(b)
            if int(dut.m_dllp_axis_tvalid.value) and int(dut.m_dllp_axis_tready.value):
                beats.append((int(dut.m_dllp_axis_tdata.value), int(dut.m_dllp_axis_tkeep.value),
                              int(dut.m_dllp_axis_tlast.value)))

    w = cocotb.start_soon(watch())
    for (d, k) in words:
        dut.data_i.value = d
        dut.data_k_i.value = k
        dut.data_valid_i.value = 1
        await RisingEdge(dut.clk_i)
    dut.data_valid_i.value = 0
    dut.data_k_i.value = 0
    for _ in range(12):
        await RisingEdge(dut.clk_i)
    done = True
    await RisingEdge(dut.clk_i)
    await RisingEdge(dut.clk_i)
    out = []
    for (td, tk, tl) in beats:
        for b in range(4):
            if (tk >> b) & 1:
                out.append((td >> (8 * b)) & 0xFF)
    lasts = [b for b in beats if b[2] == 1]
    ok = (out == payload)
    dut._log.info(
        f"M7G3|{kind}|n={n}|s={s}|frame_syms={len(payload) + 2}|beats={len(beats)}"
        f"|tlast={len(lasts)}|tkeep_last={','.join(hex(b[1]) for b in lasts)}"
        f"|payload_ok={int(ok)}|reg_arm={len(arm)}|reg_arm_bytes={','.join(map(str, arm))}")
    return len(beats)


@cocotb.test()
async def m7g3_registered_end_arm_sweep(dut):
    total = 0
    tb = TB(dut)  # ONE clock for the whole test; reset() per cell
    for kind, n in (('DLLP', 0), ('TLP', 3), ('TLP', 4), ('TLP', 5)):
        for s in range(4):
            total += await run_cell(dut, tb, kind, n, s)
    assert total > 0, "apparatus: no AXIS beat in any cell"
