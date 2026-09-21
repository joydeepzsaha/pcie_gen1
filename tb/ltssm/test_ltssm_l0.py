"""
Rung 10c, Item 2 -- L0 oracles L1, L5, L11 (and L3's trigger half).

ORACLE SOURCE
  evidence/rung10/ORACLES_LTSSM.md section O-L, derived from PCI Express Base
  Specification Rev 2.1 section 4.2.6.5 "L0", printed pages 247-249.

  NOTE ON THE SECTION NUMBER: 10c corrected an inherited error here. Recovery
  is 4.2.6.4 (p.239-246) and L0 is 4.2.6.5 (p.247-249) -- earlier notes had the
  two swapped. See evidence/rung10/A3_A7_DISPOSITION.md section 3b.

WHAT THIS TEST MEASURES (all four rows are CONFORMING -- no expect_fail here)

  L1  "LinkUp = 1b" (p.247)
      -> link_up_o must be 1 for every cycle spent in L0.

  L11 "This is the normal operational state" (p.247), i.e. no training Ordered
      Set may be transmitted while in L0.
      -> gen_ts1, gen_ts2 and gen_eios must all be 0 for every cycle in L0.

      ⚠️⚠️ THIS ROW'S PROXY WAS REWRITTEN IN §63 #7j-2, AND THE ORACLE WAS NOT.
      It used to assert `gen_os_ctrl_o.valid == 0`, and the premise underneath
      that proxy was that the ONLY thing `valid` can request is a training
      Ordered Set -- true of the world the row was written in, where L0 asked
      for nothing at all.  #7j-2 makes L0 request LOGICAL IDLE continuously
      (Base 2.1 §4.2.2 p.195: "When no packet information or special Ordered
      Sets are being transmitted ... idle data must be transmitted"), which
      raises `valid` permanently while transmitting no Ordered Set whatsoever
      -- Logical Idle is data 00h, not an Ordered Set.

      So the old assertion would have failed on RTL that is MORE conformant
      than the RTL it was written against.  That is §22.87 exactly: a green
      row's body encodes a premise about the world, and the premise expires.
      The oracle survives untouched; the new form asserts it directly by
      naming the three training-Ordered-Set request bits, which is both
      stronger and closer to p.247's own words.

      The old note that the invariant was "inherited rather than enforced" also
      expires: ST_L0 now writes gen_ts1/gen_ts2 explicitly rather than relying
      on whichever predecessor ran last.

  L5  "Next state is Recovery if a TS1 or TS2 Ordered Set is received on any
      configured Lane" (p.248)
      -> two independent rows, one for TS1 and one for TS2, each from its own
         reset. The RTL goes to ST_RECOVERY_RCVR_LOCK, which is Recovery's
         first substate, so targeting it satisfies "next state is Recovery".

  L3  the trigger half only: directed_speed_change_i in L0 -> RCVR_LOCK.
      The spec (p.247) also requires both sides to support > 2.5 GT/s and the
      Link to be in DL_Active, and requires changed_speed_recovery to be reset
      to 0b. The RTL checks neither precondition and *gates on*
      changed_speed_recovery instead of resetting it -- that divergence is
      recorded in ORACLES_LTSSM.md L3 by inspection and is NOT asserted here.
      This row asserts only that the trigger moves the FSM, which conforms.

NOT COVERED HERE, and why (recorded rather than silently dropped):
  * L2 -- "on receipt of an STP or SDP Symbol, idle_to_rlock_transitioned is
    reset to 0b". The module has no STP or SDP input port, so the antecedent
    cannot be driven at the port boundary. :1001 resets the variable
    unconditionally on every cycle in L0, which is a superset of the spec
    behaviour. Untestable-with-reason, like D10's error_c half.
  * L4, L6 -- "Recovery if directed to change Link width" / "if directed, or
    EI inferred without an EIOS". Neither is implemented; there is no
    width-change input and no EI-inference logic in ST_L0.
  * L7, L8, L9, L10 -- the L0s / L1 / L2 exits. ST_L0s, ST_L1 and ST_L2 each
    have ZERO next_state assignments anywhere in the file, so these states are
    structurally unreachable and no stimulus can reach them.

Requires SIM_FAST_LINK=1, MAX_NUM_LANES=4 (verilate_ltssm_l0 target).
No src/ edit.
"""
import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, RisingEdge, Timer
from ltssm_tb_common import *  # noqa

# ---- gen_os_struct_t bit positions, from src/packages/pcie_phy_pkg.sv:268-284.
# Packed struct, first field declared = MSB, so counting up from bit 0:
#   valid 0, gen_ts1 1, gen_ts2 2, gen2_eieos 3, gen3_eieos 4, gen_eios 5,
#   gen_skp 6, gen_idle 7, set_link 8, set_lane 9, set_speed_change 10,
#   link_number [18:11], rate_id [26:19], ts6_sym [34:27]  => 35 bits total.
# GEN_OS_WIDTH is asserted against the real port below, so a struct change
# breaks the row loudly instead of silently shifting every mask.
GEN_OS_WIDTH = 35
B_VALID = 0
B_GEN_TS1 = 1
B_GEN_TS2 = 2
B_GEN_EIOS = 5
B_GEN_IDLE = 7

# How long to watch L0 before declaring it stable. L0 has no timeout of its own
# (no timer arm in ST_L0), so this is slack, not a budget.
L0_WATCH_CYCLES = 200

# Slack for the L0 -> RCVR_LOCK arc, which is a single combinational hop.
EXIT_CYCLES = 50


def state(dut):
    return int(dut.ltssm_state_o.value)


def sname(s):
    return STATE_NAMES.get(s, hex(s))


def ctrl_bit(dut, pos):
    return (int(dut.gen_os_ctrl_o.value) >> pos) & 1


def check_geometry(dut):
    """Prove the -G overrides actually reached the DUT before trusting a row."""
    n = len(dut.ordered_set_i)
    assert n == 512, (
        f"-GMAX_NUM_LANES=4 did not reach the DUT: ordered_set_i is {n} bits "
        f"(x4 expects 4*128=512)")
    w = len(dut.gen_os_ctrl_o)
    assert w == GEN_OS_WIDTH, (
        f"gen_os_struct_t is {w} bits, expected {GEN_OS_WIDTH}; the bit "
        f"positions in this file are stale -- re-derive from pcie_phy_pkg.sv")


async def setup_to_l0(dut):
    cocotb.start_soon(Clock(dut.clk_i, 10, units="ns").start())
    check_geometry(dut)
    await bring_up_link(dut)
    assert state(dut) == ST_L0, f"setup did not end in L0, got {sname(state(dut))}"
    dut._log.info("SETUP: reached L0")


@cocotb.test()
async def test_l0_linkup_and_quiet(dut):
    """L1 + L11: in L0, link_up_o == 1 and gen_os_ctrl_o.valid == 0, every cycle.

    Both are conforming oracles, so they share one normal row.
    """
    await setup_to_l0(dut)

    for i in range(L0_WATCH_CYCLES):
        await RisingEdge(dut.clk_i)
        await Timer(1, units="ps")   # post-edge: a bare read here is PRE-edge
        if state(dut) != ST_L0:
            raise AssertionError(
                f"left L0 unprovoked after {i} cycles -> {sname(state(dut))}")
        lu = int(dut.link_up_o.value)
        assert lu == 1, f"L1 violated: link_up_o={lu} in L0 at cycle {i}"
        t1 = ctrl_bit(dut, B_GEN_TS1)
        t2 = ctrl_bit(dut, B_GEN_TS2)
        eios = ctrl_bit(dut, B_GEN_EIOS)
        assert (t1, t2, eios) == (0, 0, 0), (
            f"L11 violated: gen_ts1={t1} gen_ts2={t2} gen_eios={eios} in L0 at "
            f"cycle {i} -- a TRAINING Ordered Set is being presented for "
            f"transmission in the operational state")

    dut._log.info(
        f"L1 OK: link_up_o held 1 for {L0_WATCH_CYCLES} cycles in L0")
    dut._log.info(
        f"L11 OK: no training Ordered Set requested for {L0_WATCH_CYCLES} "
        f"cycles in L0 (gen_ts1/gen_ts2/gen_eios all 0)")


@cocotb.test()
async def test_l0_exit_on_ts1(dut):
    """L5: a TS1 received in L0 moves the FSM to Recovery (RcvrLock)."""
    await setup_to_l0(dut)

    dut.ordered_set_i.value = pack_tsos_all_lanes(
        link_num=LINK_NUM, lane_num="index", rate=GEN1_RATE)
    dut.ts1_valid_i.value = ALL
    await wait_state(dut, ST_RECOVERY_RCVR_LOCK, EXIT_CYCLES,
                     "RECOVERY_RCVR_LOCK")
    dut._log.info("L5 OK (TS1): L0 -> RECOVERY_RCVR_LOCK")


@cocotb.test()
async def test_l0_exit_on_ts2(dut):
    """L5: a TS2 received in L0 moves the FSM to Recovery (RcvrLock).

    Independent reset from the TS1 row -- not a continuation of it.
    """
    await setup_to_l0(dut)

    dut.ordered_set_i.value = pack_tsos_all_lanes(
        link_num=LINK_NUM, lane_num="index", rate=GEN1_RATE)
    dut.ts2_valid_i.value = ALL
    await wait_state(dut, ST_RECOVERY_RCVR_LOCK, EXIT_CYCLES,
                     "RECOVERY_RCVR_LOCK")
    dut._log.info("L5 OK (TS2): L0 -> RECOVERY_RCVR_LOCK")


@cocotb.test()
async def test_l0_exit_on_directed_speed_change(dut):
    """L3 (trigger half only): directed_speed_change_i in L0 -> RcvrLock.

    No TS is presented, so this isolates the third disjunct of :1002 from the
    ts1_valid_i / ts2_valid_i ones exercised above.
    """
    await setup_to_l0(dut)

    assert int(dut.ts1_valid_i.value) == 0 and int(dut.ts2_valid_i.value) == 0, \
        "TS strobes must be clear so this row isolates directed_speed_change_i"
    dut.directed_speed_change_i.value = 1
    await wait_state(dut, ST_RECOVERY_RCVR_LOCK, EXIT_CYCLES,
                     "RECOVERY_RCVR_LOCK")
    dut._log.info("L3 OK (trigger): directed_speed_change_i moved L0 -> RCVR_LOCK")


# ==========================================================================
#  §63 #7j-2 -- Logical Idle in L0.  Base 2.1 §4.2.2 p.195.
#
#  ⚠️ RED BEFORE FIX.  #7j Phase 1 located the defect to one state: entry to L0
#  clears gen_os_ctrl_c.gen_idle (:1214) and ST_L0 contains no gen_idle
#  reference at all -- established by parsing all 31 state arms, not by a bare
#  grep.  The Transmitter's Logical Idle machinery already exists and is
#  byte-exact against Table B p.700; what is missing is a request.
# ==========================================================================

@cocotb.test()
async def test_l0_requests_logical_idle(dut):
    """§63 #7j-2 -- in L0 the LTSSM must keep Logical Idle requested.

    Base 2.1 §4.2.2 p.195: "When no packet information or special Ordered Sets
    are being transmitted, the Transmitter is in the Logical Idle state.
    During this time idle data must be transmitted."  L0 is the state in which
    that is true for most Symbol Times, so L0 is where the request belongs.

    RED BEFORE FIX: measured in #7j Phase 1 -- gen_idle is cleared on entry to
    L0 (:1214) and never re-asserted, so between packets the link falls silent
    and the PIPE carries a stale scrambled word with valid low.

    Three limbs, because a request is three things and a row that checked one
    would pass on a half-built one:
      * gen_idle asserted, every cycle;
      * valid asserted, so os_generator actually latches it (:203);
      * the template is the all-zero gen_zeros(), because Logical Idle is the
        data byte 00h and anything else would put other bytes on the wire.
    """
    await setup_to_l0(dut)

    for i in range(L0_WATCH_CYCLES):
        await RisingEdge(dut.clk_i)
        await Timer(1, units="ps")
        if state(dut) != ST_L0:
            raise AssertionError(
                f"left L0 unprovoked after {i} cycles -> {sname(state(dut))}")
        gi = ctrl_bit(dut, B_GEN_IDLE)
        v = ctrl_bit(dut, B_VALID)
        os_word = int(dut.ordered_set_o.value)
        assert gi == 1, (
            f"gen_idle={gi} in L0 at cycle {i} -- nothing is asking the "
            f"Transmitter for Logical Idle, so the link goes quiet between "
            f"packets (Base 2.1 §4.2.2 p.195)")
        assert v == 1, (
            f"gen_os_ctrl_o.valid={v} in L0 at cycle {i} -- gen_idle is set "
            f"but os_generator never latches it (os_generator.sv:203)")
        assert os_word == 0, (
            f"ordered_set_o={os_word:#x} in L0 at cycle {i} -- Logical Idle is "
            f"the data byte 00h; the template must be gen_zeros()")

    dut._log.info(
        f"7J2 OK: L0 held gen_idle=1 valid=1 with the all-zero template for "
        f"{L0_WATCH_CYCLES} cycles")


@cocotb.test()
async def test_l0_exit_clears_gen_idle(dut):
    """§63 #7j-2, CONSTRAINT 4 -- gen_idle handling on LEAVING L0 is unchanged.

    ⚠️ This is a REGRESSION GUARD and it guards a real hazard, which is why it
    is worth its cost.  gen_os_ctrl_c is sticky (:587), so once ST_L0 sets
    gen_idle it stays set unless the exit clears it.  The exit branch builds a
    TS1 for Recovery.RcvrLock -- and os_generator's ST_BUILD applies gen_idle
    AFTER the TS mask, wiping it wholesale (`if (gen_os_ctrl_i.gen_idle)
    D.special_k = '0;`, os_generator.sv:285).  A TS1 whose COM is not K-marked
    is scrambled like data and is unrecognisable to the peer's matcher, so
    forgetting the clear does not degrade training -- it breaks it.

    The row asserts both halves together, because gen_idle=0 alone is
    satisfied by an exit that never happened:
      * the FSM reached Recovery.RcvrLock (non-vacuity);
      * gen_ts1 is 1 there and gen_idle is 0.
    """
    await setup_to_l0(dut)

    dut.ordered_set_i.value = pack_tsos_all_lanes(
        link_num=LINK_NUM, lane_num="index", rate=GEN1_RATE)
    dut.ts1_valid_i.value = ALL
    await wait_state(dut, ST_RECOVERY_RCVR_LOCK, EXIT_CYCLES,
                     "RECOVERY_RCVR_LOCK")
    await ClockCycles(dut.clk_i, 2)
    await Timer(1, units="ps")

    gi = ctrl_bit(dut, B_GEN_IDLE)
    t1 = ctrl_bit(dut, B_GEN_TS1)
    dut._log.info(f"7J2[C4] on entry to RcvrLock from L0: gen_ts1={t1} gen_idle={gi}")
    assert t1 == 1, (
        f"NON-VACUITY: gen_ts1={t1} on entry to Recovery.RcvrLock -- the exit "
        f"did not build a TS1 at all, so this row cannot say anything about "
        f"whether gen_idle was cleared alongside it")
    assert gi == 0, (
        f"gen_idle={gi} survived the exit from L0 into Recovery.RcvrLock.  "
        f"os_generator.sv:285 wipes special_k wholesale for gen_idle, so the "
        f"TS1's COM would be transmitted as scrambled data and the peer's "
        f"16-Symbol matcher would never see the Ordered Set")
