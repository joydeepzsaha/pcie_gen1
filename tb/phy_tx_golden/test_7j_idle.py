"""§63 #7j Phase 1 -- MEASUREMENT ONLY.  What does the Transmitter actually put
on the wire during Logical Idle, and with what `valid`?

Every row here is a probe, not an oracle: it emits raw timestamped events on
one line each and asserts nothing about them (§22.92 -- SV/bench probes emit raw
events; all pairing, classification and arithmetic happen offline in Python).
The offline half is `pcie_docs/evidence/fullstack/analyse_7j_phase1.py`, and it
opens with the known-answer test against the two published tables on Base 2.1
p.700 before it reads a single DUT byte.

⚠️ Why this bench exists at all when `test_tx_os_golden.py` already drives
`G_GEN_IDLE`: that row's `capture()` is UNGATED, and its docstring still says
"lane_management.sv:571 ties data_valid_o to '1 unconditionally".  FA-2 made
`data_valid_o` honest (`lane_management.sv:606`), so an ungated window now
samples FREEZES as if they were events -- the #7h lesson, restated.  This bench
records `valid` alongside every word so the offline half can separate "what the
wire carried" from "what was actually sent".

Event grammar, one per line, all fields hex unless noted:

    EV|<row>|<cycle>|<time_ns>|<word32>|<k4>|<valid>

"""
import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ClockCycles

COM = 0xBC
PAD = 0xF7
TS1 = 0x4A
GEN1 = 0x01
GEN1_BASIC = 0x02

G_VALID    = 1 << 0
G_GEN_TS1  = 1 << 1
G_GEN_IDLE = 1 << 7


def pack(com=COM, link_num=PAD, lane_num=PAD, ts_disc=TS1,
         n_fts=0xFF, rate_id=GEN1_BASIC, train_ctrl=0x00):
    b = [com, link_num, lane_num, n_fts, rate_id, train_ctrl] + [ts_disc] * 10
    v = 0
    for i, bv in enumerate(b):
        v |= (bv & 0xFF) << (8 * i)
    return v


async def start_clocks(dut):
    cocotb.start_soon(Clock(dut.clk_i, 10.0, units="ns").start())
    cocotb.start_soon(Clock(dut.pipe_rx_usr_clk_i, 10.0, units="ns").start())
    cocotb.start_soon(Clock(dut.pipe_tx_usr_clk_i, 10.0, units="ns").start())


async def reset(dut, link_up=0):
    dut.rst_i.value = 1
    dut.en_i.value = 0
    dut.link_up_i.value = 0
    dut.num_active_lanes_i.value = 1
    dut.send_ordered_set_i.value = 0
    dut.ordered_set_i.value = 0
    dut.gen_os_ctrl_i.value = 0
    dut.curr_data_rate_i.value = GEN1
    dut.s_dllp_axis_tdata.value = 0
    dut.s_dllp_axis_tkeep.value = 0
    dut.s_dllp_axis_tvalid.value = 0
    dut.s_dllp_axis_tlast.value = 0
    dut.s_dllp_axis_tuser.value = 0
    await ClockCycles(dut.pipe_tx_usr_clk_i, 8)
    dut.rst_i.value = 0
    await ClockCycles(dut.pipe_tx_usr_clk_i, 4)
    dut.en_i.value = 1
    dut.link_up_i.value = link_up


async def record(dut, row, cycles):
    """Sample CONTINUOUSLY from before the event (§22.89) and emit raw lines.

    No filtering, no pairing, no arithmetic -- the window is a plain cycle count
    starting where the caller stands, and `valid` is recorded rather than used.
    """
    for c in range(cycles):
        await RisingEdge(dut.pipe_tx_usr_clk_i)
        word = int(dut.pipe_data_o.value) & 0xFFFFFFFF
        dk = int(dut.pipe_data_k_o.value) & 0xF
        try:
            val = int(dut.pipe_data_valid_o.value) & 0x1
        except Exception:
            val = -1
        dut._log.info("EV|%s|%d|%.1f|%08x|%x|%d"
                      % (row, c, cocotb.utils.get_sim_time(units="ns"),
                         word, dk, val))


# --------------------------------------------------------------- M1
@cocotb.test()
async def m1_idle_stream_raw(dut):
    """Drive gen_idle once, then stand still and record 200 cycles.

    Answers: what bytes, what K flags, and what `valid` does the Transmitter
    put out when the LTSSM asks for Logical Idle -- and for how long.
    """
    await start_clocks(dut)
    await reset(dut)
    dut.ordered_set_i.value = pack(com=0x00, link_num=0x00, lane_num=0x00,
                                   n_fts=0x00, rate_id=0x00, train_ctrl=0x00,
                                   ts_disc=0x00)
    dut.gen_os_ctrl_i.value = G_VALID | G_GEN_IDLE
    await record(dut, "M1", 200)


# --------------------------------------------------------------- M2
@cocotb.test()
async def m2_quiescent_raw(dut):
    """The control: no traffic requested at all, 200 cycles.

    This is what an L0 gap looks like today.  Pairs with M1 (§22.81) -- without
    it, M1's stream could not be told from whatever the block does at rest.
    """
    await start_clocks(dut)
    await reset(dut)
    await record(dut, "M2", 200)


# --------------------------------------------------------------- M3
@cocotb.test()
async def m3_ts1_then_quiescent_raw(dut):
    """A TS1 ordered set, then silence.  Records the transition into a gap, so
    the offline half can see what the last live word was and what is held after
    it -- the "stale scrambled word, frozen and repeated" of #7h, re-measured
    here with `valid` recorded alongside rather than inferred.
    """
    await start_clocks(dut)
    await reset(dut)
    dut.ordered_set_i.value = pack(link_num=0x05, lane_num=0x00, ts_disc=TS1)
    dut.gen_os_ctrl_i.value = G_VALID | G_GEN_TS1
    await ClockCycles(dut.pipe_tx_usr_clk_i, 2)
    dut.gen_os_ctrl_i.value = 0
    await record(dut, "M3", 200)


# --------------------------------------------------------------- M4
@cocotb.test()
async def m4_idle_held_long_raw(dut):
    """gen_idle held asserted for the whole window, 400 cycles, link_up raised.

    M1 asks "what happens when idle is requested once".  This asks "what happens
    if the request never drops" -- the shape a continuous L0 filler would need --
    and with link_up_i high, so os_generator's SKP timer is armed
    (os_generator.sv:139) and any SKP/idle interaction is inside the window.
    """
    await start_clocks(dut)
    await reset(dut, link_up=1)
    dut.ordered_set_i.value = pack(com=0x00, link_num=0x00, lane_num=0x00,
                                   n_fts=0x00, rate_id=0x00, train_ctrl=0x00,
                                   ts_disc=0x00)
    dut.gen_os_ctrl_i.value = G_VALID | G_GEN_IDLE
    await record(dut, "M4", 400)
