"""
Polling.Active accepts the eight consecutive TS1 without ever reading Symbol 5.

Base 2.1 Rev 2.1, Section 4.2.6.2.1 (p.220) -- the eight consecutive training
sequences that qualify the Polling.Configuration exit must satisfy ANY of:

  (a) TS1 with Lane and Link numbers set to PAD (K23.7) and the Compliance
      Receive bit (bit 4 of Symbol 5) is 0b.
  (b) TS1 with Lane and Link numbers set to PAD (K23.7) and the Loopback bit
      (bit 2 of Symbol 5) is 1b.
  (c) TS2 with Lane and Link numbers set to PAD (K23.7).

and p.221 routes the complementary case to Polling.Compliance:

  "(b) any Lane that detected a Receiver during Detect received eight
   consecutive TS1 Ordered Sets ... with the Lane and Link numbers set to PAD
   (K23.7), the Compliance Receive bit (bit 4 of Symbol 5) is 1b, and the
   Loopback bit (bit 2 of Symbol 5) is 0b."

Before the fix, Polling.Active's and Polling.Configuration's receive arms
tested only

    (ordered_set_i[lane].link_num == PAD) && (ordered_set_i[lane].lane_num == PAD)

Symbol 5 is never read. So a TS1 with Compliance Receive = 1b and Loopback = 0b
-- which satisfies NONE of (a)/(b)/(c), and is precisely the Polling.Compliance
trigger -- is counted toward the eight consecutive anyway.

THE FIELD DOES NOT EXIST IN THE RTL
training_ctrl_t (pcie_phy_pkg.sv:209-215) is

    { rsvd[7:4], scramble[3], loopback[2], dis_link[1], hot_rst[0] }

The Compliance Receive bit is Symbol 5 bit 4, which lies inside rsvd. The RTL
has no named member for it, so this is not a missing check on a modelled field
-- the field itself was never modelled. The S5_* masks this test drives come
from Base 2.1 Table 4-2, not from the struct.

WHAT THIS TEST MEASURES
Sit in Polling.Active and answer with TS1, Link = Lane = PAD, Symbol 5 = 0x10
(Compliance Receive = 1, Loopback = 0).

  Spec-conformant DUT: these do not qualify, so Polling.Active is NOT exited
                       toward Polling.Configuration.
  This DUT:            exits to Polling.Configuration.

The assertion states the spec outcome (oracle P3,
evidence/rung10/ORACLES_LTSSM.md). It was an expect_fail row recording the
divergence until fix-arc 6b closed it.

STATUS: FIXED (fix-arc 6b). ST_POLLING_ACTIVE's per-lane receive arm now also
requires (Compliance Receive == 0b) OR (Loopback == 1b), which is p.220's (a)
OR (b) with the shared PAD/PAD conjunct factored out; the marker came off in
the same commit (rule 22.75). The bit is still addressed positionally as
train_ctrl.rsvd[4] because training_ctrl_t STILL has no member for it -- that
package change is registered as owed, not taken. Pre-edit census and
predictions: evidence/fix-arc-6/PREDICTIONS_P3.md. Mutant MP3 drops the
conjunct.

Note the interaction with oracle P7: the state the spec wants here --
Polling.Compliance -- is structurally unreachable in this design
(evidence/rung10/CENSUS_LTSSM.md section 2.1: its only entry at :689 sits in an
else-if subsumed by :677). So there is no correct destination available even in
principle; this test pins the near half of that gap, that the disqualifying
Ordered Sets are accepted.

Requires SIM_FAST_LINK=1 (MinTS1sPolling = 24), MAX_NUM_LANES=1
(verilate_polling_p3 target).
"""
import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, RisingEdge, Timer
from ltssm_tb_common import *  # noqa

# sec 63 #7g-2 (D-7G.2): this bench's clock AND the RTL's CLK_PERIOD_NS, one value.
# The core passes -GCLK_PERIOD_NS=8 (tb_ltssm_b2b.sv passes .CLK_PERIOD_NS(8));
# every cycle budget below that stands for a spec time derives from it.
CLK_PERIOD_NS = 8

# Long enough for MinTS1sPolling=24 transmit pulses at ~4 cycles each, with
# generous slack. Far short of the 24 ms (2.4 M cycle) watchdog, which is NOT
# SIM_FAST_LINK scaled and is not what this test is about.
SETTLE_CYCLES = 4000


def state(dut):
    return int(dut.ltssm_state_o.value)


def sname(s):
    return f"{STATE_NAMES.get(s, hex(s))} ({s:#07x})"


@cocotb.test()
async def run_test_polling_p3(dut):
    cocotb.start_soon(Clock(dut.clk_i, CLK_PERIOD_NS, units="ns").start())

    n_bits = len(dut.ordered_set_i)
    assert n_bits == 128, (
        f"-GMAX_NUM_LANES=1 did not reach the DUT: ordered_set_i is {n_bits} "
        f"bits (x1 expects 128)")

    drive_idle_inputs(dut)
    dut.rst_i.value = 1
    await ClockCycles(dut.clk_i, 5)
    dut.rst_i.value = 0
    await ClockCycles(dut.clk_i, 5)

    dut.en_i.value = 1
    await wait_state(dut, ST_DETECT_QUIET, 50, "DETECT_QUIET")
    dut.phy_rxelecidle_i.value = LANE0_MASK
    await ClockCycles(dut.clk_i, 3)
    dut.phy_rxelecidle_i.value = 0
    await wait_state(dut, ST_DETECT_ACTIVE, 50, "DETECT_ACTIVE")

    dut.receiver_detected_i.value = LANE0_MASK
    dut.phy_rxstatus_i.value = RXSTATUS_OK_X1
    dut.phy_phystatus_i.value = LANE0_MASK
    await ClockCycles(dut.clk_i, 3)
    dut.phy_phystatus_i.value = 0
    cocotb.start_soon(os_tx_pulser(dut))
    await wait_state(dut, ST_POLLING_ACTIVE, 200, "POLLING_ACTIVE")

    # ---- answer with DISQUALIFYING TS1: PAD/PAD, Compliance Receive = 1b,
    #      Loopback = 0b. Satisfies none of p.220's (a), (b) or (c). ----
    s5 = S5_COMPLIANCE_RCV
    assert s5 & S5_LOOPBACK == 0, "Loopback must stay 0b or condition (b) applies"
    dut.ordered_set_i.value = pack_tsos(link_num=None, lane_num=None,
                                        train_ctrl=s5)
    dut.ts1_valid_i.value = LANE0_MASK
    dut._log.info(
        f"answering with TS1 PAD/PAD, Symbol 5 = {s5:#04x} "
        f"(Compliance Receive=1, Loopback=0) -- disqualifies under p.220")

    reached = None
    for _ in range(SETTLE_CYCLES):
        await RisingEdge(dut.clk_i)
        await Timer(1, units="ps")
        if state(dut) != ST_POLLING_ACTIVE:
            reached = state(dut)
            break

    if reached is None:
        dut._log.info("stayed in Polling.Active -- spec-conformant")
    else:
        dut._log.info(f"left Polling.Active into {sname(reached)}")

    assert reached is None, (
        f"P3 (Base 2.1 4.2.6.2.1, p.220): the eight consecutive training "
        f"sequences must satisfy one of (a) TS1/PAD with Compliance Receive "
        f"(Symbol 5 bit 4) = 0b, (b) TS1/PAD with Loopback (bit 2) = 1b, or "
        f"(c) TS2/PAD. This stimulus is TS1/PAD with Compliance Receive = 1b "
        f"and Loopback = 0b, which satisfies none of them -- p.221 routes it "
        f"to Polling.Compliance instead. The DUT counted it anyway and left "
        f"for {sname(reached)}. Cause: :1696 tests only link_num and lane_num "
        f"against PAD and never reads Symbol 5; training_ctrl_t "
        f"(pcie_phy_pkg.sv:209-215) does not even model the Compliance "
        f"Receive bit, which falls inside its rsvd[7:4] field.")
