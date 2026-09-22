"""§63 #7j-2 Phase 1 -- MEASUREMENT ONLY.  What does a CONTINUOUS Logical Idle
request do to the packet path?

#7j Phase 1 established that the Transmitter already emits byte-exact scrambled
Logical Idle whenever `gen_idle` is asserted, and that the defect is one state
wide: nothing requests it during L0.  #7j-2's shape is to request it there.

This bench asks the question that shape raises and that no existing row covers:
**when idle is requested continuously, can a packet still get out?**

Every row is a probe, not an oracle (§22.92).  Each emits one raw timestamped
line per cycle and asserts nothing; all pairing, classification and arithmetic
happen offline in `pcie_docs/evidence/fullstack/analyse_7j2_arb.py`, which opens
with a known-answer self-test before it reads a single DUT byte.

⚠️ THE SEAM IS `phy_transmit`, NOT the LTSSM, and that is deliberate.  The
question is about what happens DOWNSTREAM of the request -- in `os_generator`
and `lane_management` -- so it is measured where the request is an input the
bench drives, rather than through a state machine that also has to be steered
into L0.  §22.85: this is a property of the route from `gen_os_ctrl_i` to
`pipe_data_o`, and it is measured on that route.

Event grammar, one per line, all fields hex unless noted:

    EV|<row>|<cycle>|<time_ns>|<word32>|<k4>|<valid>|<dllp_tready>|<dllp_tvalid>|<os_sent>

K codes are NOT scrambled (Base 2.1 §4.2.3 p.199), so the framing Symbols are
visible at `pipe_data_o` without descrambling: SDP = K28.2 = 0x5C opens a DLLP
and END = K29.7 = 0xFD closes it.  Logical Idle carries K = 0 on every Symbol.
That is the detector the offline half uses, and it needs no model.
"""
import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ClockCycles

G_VALID    = 1 << 0
G_GEN_TS1  = 1 << 1
G_GEN_IDLE = 1 << 7

# A DLLP payload, distinctive on purpose so the offline half can tell it from
# anything the DUT invents.  Same bytes #7j-1's fragment sweep used.
DLLP_W0 = 0xEFBEADDE     # DE AD BE EF, LSB-first on the wire
DLLP_W1 = 0x00003412     # 12 34


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
    dut.curr_data_rate_i.value = 1          # gen1
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


def zeros_os():
    """`gen_zeros()`'s template: 16 Symbols of 00h.  os_generator clears
    special_k for gen_idle (os_generator.sv:285), so every Symbol goes out as
    DATA 00h and the scrambler turns it into Base 2.1 Table B p.700."""
    return 0


async def present_dllp(dut):
    """Hold a 2-beat DLLP at the AXIS port and leave it there.

    ⚠️ The packet is PRESENTED and never withdrawn.  A row that offered it for
    a fixed number of cycles and gave up could not tell "never accepted" from
    "offered too briefly" -- the #7h window lesson.  Presentation is continuous;
    the offline half reads acceptance off `tready`.
    """
    dut.s_dllp_axis_tdata.value = DLLP_W0
    dut.s_dllp_axis_tkeep.value = 0xF
    dut.s_dllp_axis_tuser.value = 0
    dut.s_dllp_axis_tlast.value = 0
    dut.s_dllp_axis_tvalid.value = 1


async def record(dut, row, cycles, offer_at=None):
    """Sample continuously from BEFORE the event (§22.89); emit raw lines only.

    `offer_at` presents the DLLP at that cycle INSIDE the window rather than
    before it.  ⚠️ That parameter exists because the first version of this bench
    presented the packet before the window opened, at the same instant as the
    idle request: `lane_management` was still in ST_IDLE with an EMPTY
    Ordered-Set FIFO, so the packet won a race that steady-state L0 never
    offers, and both rows reported an acceptance that said nothing about the
    question.  A window that does not contain the event it was opened for is
    the #7h lesson in its cheapest form, and it cost one run here.
    """
    beat = 0
    for c in range(cycles):
        if offer_at is not None and c == offer_at:
            await present_dllp(dut)
        await RisingEdge(dut.pipe_tx_usr_clk_i)
        word = int(dut.pipe_data_o.value) & 0xFFFFFFFF
        dk = int(dut.pipe_data_k_o.value) & 0xF
        val = int(dut.pipe_data_valid_o.value) & 0x1
        rdy = int(dut.s_dllp_axis_tready.value) & 0x1
        tvl = int(dut.s_dllp_axis_tvalid.value) & 0x1
        ossent = int(dut.ordered_set_tranmitted_o.value) & 0x1
        dut._log.info("EV|%s|%d|%.1f|%08x|%x|%d|%d|%d|%d"
                      % (row, c, cocotb.utils.get_sim_time(units="ns"),
                         word, dk, val, rdy, tvl, ossent))
        # Advance the AXIS beat on an accepted handshake, so a packet that IS
        # accepted completes rather than repeating beat 0 forever.  Driven from
        # the recorded handshake, not from a schedule (§22.80: the control is
        # not computed from the signal under test -- this is the DUT's own
        # handshake, which is what an AXI-Stream source is obliged to follow).
        if rdy and tvl:
            beat += 1
            if beat == 1:
                dut.s_dllp_axis_tdata.value = DLLP_W1
                dut.s_dllp_axis_tkeep.value = 0x3
                dut.s_dllp_axis_tlast.value = 1
            else:
                dut.s_dllp_axis_tvalid.value = 0
                dut.s_dllp_axis_tlast.value = 0


# --------------------------------------------------------------- N1
@cocotb.test()
async def n1_continuous_idle_no_strobe_with_packet(dut):
    """Idle requested continuously with send_ordered_set_i LOW, packet offered.

    This is the shape #7j-2's fix produces if ST_L0 holds the request and drops
    the strobe: os_generator's ST_SEND streaming lock (os_generator.sv:363)
    should hold and emit Ordered Sets back to back.

    The question this row answers is whether the DLLP ever leaves.
    """
    await start_clocks(dut)
    await reset(dut, link_up=1)
    dut.ordered_set_i.value = zeros_os()
    dut.gen_os_ctrl_i.value = G_VALID | G_GEN_IDLE
    dut.send_ordered_set_i.value = 0
    await record(dut, "N1", 2000, offer_at=500)


# --------------------------------------------------------------- N2
@cocotb.test()
async def n2_continuous_idle_with_strobe_with_packet(dut):
    """Same, but send_ordered_set_i HIGH -- which is what ST_L0 does TODAY,
    unconditionally, at pcie_ltssm_downstream.sv:1278.

    Pairs with N1 so the strobe is varied ALONE (the 2x2 lesson of #7j-1: a
    comparison that varies two things at once is worthless).
    """
    await start_clocks(dut)
    await reset(dut, link_up=1)
    dut.ordered_set_i.value = zeros_os()
    dut.gen_os_ctrl_i.value = G_VALID | G_GEN_IDLE
    dut.send_ordered_set_i.value = 1
    await record(dut, "N2", 2000, offer_at=500)


# --------------------------------------------------------------- N3
@cocotb.test()
async def n3_no_idle_with_packet(dut):
    """THE POSITIVE CONTROL (§22.81).  No idle requested; same packet, same
    port, same window.  Without this row, N1/N2's silence could not be told
    from a bench that never presented a packet correctly in the first place.
    """
    await start_clocks(dut)
    await reset(dut, link_up=1)
    dut.gen_os_ctrl_i.value = 0
    dut.send_ordered_set_i.value = 0
    await record(dut, "N3", 2000, offer_at=500)


# --------------------------------------------------------------- N4
@cocotb.test()
async def n4_continuous_idle_no_packet_no_strobe(dut):
    """Idle requested continuously, NO packet.  Measures the wire duty cycle
    alone, so the valid-never-drops claim (acceptance (a)) is separated from
    the arbitration question.  send_ordered_set_i LOW.
    """
    await start_clocks(dut)
    await reset(dut, link_up=1)
    dut.ordered_set_i.value = zeros_os()
    dut.gen_os_ctrl_i.value = G_VALID | G_GEN_IDLE
    dut.send_ordered_set_i.value = 0
    await record(dut, "N4", 2000)


# --------------------------------------------------------------- N5
@cocotb.test()
async def n5_continuous_idle_no_packet_with_strobe(dut):
    """As N4 but send_ordered_set_i HIGH.  N4/N5 vary the strobe alone on the
    no-packet route, which is what separates K3/K4 (os_generator's own duty)
    from K5 (what survives lane_management's 2-cycle drain)."""
    await start_clocks(dut)
    await reset(dut, link_up=1)
    dut.ordered_set_i.value = zeros_os()
    dut.gen_os_ctrl_i.value = G_VALID | G_GEN_IDLE
    dut.send_ordered_set_i.value = 1
    await record(dut, "N5", 2000)
