"""
Configuration.Idle: how many times the 2 ms timeout may divert to Recovery.

Base 2.1 Rev 2.1, Section 4.2.6.3.6 (p.237):

    "Otherwise, after a minimum 2 ms timeout:
       - If the idle_to_rlock_transitioned variable is 0b, the next state is
         Recovery.RcvrLock.
           - The idle_to_rlock_transitioned variable is set to 1b upon
             transitioning to Recovery.RcvrLock.
       - Else the next state is Detect."

idle_to_rlock_transitioned is a ONE-BIT variable: exactly one diversion to
Recovery.RcvrLock is permitted, and the next 2 ms timeout must go to Detect.

pcie_ltssm_downstream.sv:976-991 implements it as an 8-bit counter and intends
to saturate it at FFh on the Gen1/Gen2 path, which would reproduce the spec's
one-shot behaviour:

    else if (timer_r >= TwoMsTimeOut) begin
      if (idle_to_rlock_transitioned_r < 8'hFF) begin
        if (curr_data_rate_r == gen1 || curr_data_rate_r == gen2) begin   // :979
          idle_to_rlock_transitioned_c = 8'hFF;
        end else begin
          idle_to_rlock_transitioned_c = idle_to_rlock_transitioned_r + 1;
        end
        next_state = ST_RECOVERY_RCVR_LOCK;
      end else begin ... next_state = ST_IDLE; end

THE :979 GUARD IS DEAD CODE. curr_data_rate_r is a rate_id_t, whose layout is
(pcie_phy_pkg.sv:247-252)

    { speed_change[7], autonomous_change[6], rate[5:1], rsvd0[0] }

so `rate` sits one bit ABOVE the LSB and a rate_id_t carrying gen1 has the
integer value gen1 << 1 == 2 (gen1_basic = 8'b000_00010), while the bare enum
gen1 = 5'b00001 zero-extends to 8'd1 for the comparison. 2 != 1. The same holds
for gen2 (6 vs 3). Measured directly with Verilator 5.050:

    gen1_basic = 2 ; gen1 = 1 ; (r == gen1) = 0 ; (r.rate == gen1) = 1

Every other rate test in the file compares the FIELD -- :530, :1319, :1462 and
:1466 all write curr_data_rate_r.rate -- so :979 is the outlier, and it is off
by exactly one left shift.

Consequence: the saturation never happens, the else-branch increments instead,
and the guard `idle_to_rlock_transitioned_r < 8'hFF` admits 255 diversions to
Recovery.RcvrLock where the spec permits one.

WHAT THIS TEST MEASURES
  1. Walk to Configuration.Idle as the Root Complex, withhold Idle data, and
     let the 2 ms timeout fire.  -> timeout #1
  2. Drive the Recovery loop back round to Configuration.Idle. That path
     (RcvrLock -> RcvrCfg -> Recovery.Idle -> Linkwidth.Start -> ... -> Idle)
     never clears idle_to_rlock_transitioned: the only sites that clear it are
     :526 (ST_IDLE), :972 (Config.Idle success), :1001 (L0), :1445
     (Recovery.Idle -> L0) and :1486 (Send.SDS), none of which is on it.
  3. Withhold Idle data again and let the 2 ms timeout fire.  -> timeout #2

  Spec-conformant DUT: timeout #2 goes to Detect.
  This DUT: timeout #2 goes to Recovery.RcvrLock again.

Timeout #1 CANNOT distinguish the two -- the spec sets the flag to 1b and the
buggy increment yields 0+1 = 1, which are the same observable. The divergence
is only visible from timeout #2 onward; do not "simplify" this test down to one
timeout.

STATUS: FIXED (fix-arc 1, Phase 1).
  :979 now compares curr_data_rate_r.rate, so the saturation at :980 is live and
  the second timeout reaches Detect. This was an expect_fail row for oracle
  C26/C26a (evidence/rung10/ORACLES_LTSSM.md) from Rung 10b until the fix landed;
  the marker was removed in the same commit as the fix, per the fix-arc contract
  (rule 22.75).

  NOTE for anyone reading a gate record: flipping this row did NOT change its
  T/A row. cocotb reports an expect_fail raise as STATUS=PASS, and this test
  diverged on the very cycle it now conforms on, so both STATUS and SIM TIME are
  the same before and after. The observable difference is in the raw log only:
  "passed: failed as expected (result was AssertionError)" became "passed".
  The proof that the fix works is the mutation in evidence/fix-arc-1/, not the
  gate hash.

NOTE ON RUNTIME: TwoMsTimeOut is NOT scaled by SIM_FAST_LINK
(evidence/rung10/CENSUS_LTSSM.md section 6), so each timeout costs a real
200 000 cycles. This test is long by construction.

Requires SIM_FAST_LINK=1, MAX_NUM_LANES=1, IS_ROOT_PORT=1, LINK_NUM=1
(verilate_cfgidle_c26a target).
"""
import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, RisingEdge, Timer
from ltssm_tb_common import *  # noqa

# sec 63 #7g-2 (D-7G.2): this bench's clock AND the RTL's CLK_PERIOD_NS, one value.
# The core passes -GCLK_PERIOD_NS=8 (tb_ltssm_b2b.sv passes .CLK_PERIOD_NS(8));
# every cycle budget below that stands for a spec time derives from it.
CLK_PERIOD_NS = 8

LINK = LINK_NUM

# pcie_ltssm_downstream.sv:113 -- (2 * 10**6) / ClockPeriodNs, NOT SIM_FAST_LINK
# scaled. 200 000 cycles at CLK_RATE=100 (ClockPeriodNs = 10).
TWO_MS_CYCLES = 2_000_000 // CLK_PERIOD_NS   # 63 #7g-2: was 200_000 (10 ns)
TIMEOUT_BUDGET = TWO_MS_CYCLES + 20_000     # slack for entry/settling

# Any Detect-family state counts as "went to Detect": ST_IDLE is the RTL's
# stand-in for Detect.Quiet, and with en_i asserted it leaves ST_IDLE for
# ST_DETECT_QUIET on the next cycle, so a coarse poll may land on either.
DETECT_FAMILY = {ST_IDLE, ST_DETECT_WAIT_ONE_MS, ST_DETECT_QUIET,
                 ST_DETECT_ACTIVE, ST_DETECT_RX}

POLL = 64   # coarse poll interval; every state below persists far longer


def state(dut):
    return int(dut.ltssm_state_o.value)


def sname(s):
    return f"{STATE_NAMES.get(s, hex(s))} ({s:#07x})"


async def wait_leave(dut, from_state, budget, what):
    """Poll coarsely until the FSM leaves `from_state`. Returns the state it
    landed in. Coarse polling is safe here because every state this test waits
    out is held for thousands of cycles."""
    waited = 0
    while waited < budget:
        await ClockCycles(dut.clk_i, POLL)
        waited += POLL
        if state(dut) != from_state:
            await Timer(1, units="ps")   # post-edge read
            return state(dut), waited
    raise AssertionError(
        f"{what}: never left {sname(from_state)} within {budget} cycles")


async def walk_config_to_idle(dut):
    """Configuration.Linkwidth.Start -> ... -> Configuration.Idle, as the RC,
    with the TB echoing what the DUT transmits. Mirrors bring_up_link_rc's
    Configuration steps; kept local so this test can stop AT Config.Idle
    instead of continuing into L0 (which would clear the very variable under
    test, at :1001)."""
    steps = [
        (ST_CFG_LW_ACCEPT, "CFG_LINKWIDTH_ACCEPT", "ts1",
         dict(link_num=LINK, lane_num=None)),
        (ST_CFG_LN_WAIT,   "CFG_LANENUM_WAIT",     "ts1",
         dict(link_num=LINK, lane_num=None)),
        (ST_CFG_LN_ACCEPT, "CFG_LANENUM_ACCEPT",   "ts1",
         dict(link_num=LINK, lane_num=0)),
        (ST_CFG_COMPLETE,  "CFG_COMPLETE",         "ts1",
         dict(link_num=LINK, lane_num=0)),
        (ST_CFG_IDLE,      "CFG_IDLE",             "ts2",
         dict(link_num=LINK, lane_num=0)),
    ]
    for nxt, name, strobe, echo in steps:
        dut.ordered_set_i.value = pack_tsos(**echo)
        dut.ts1_valid_i.value = LANE0_MASK if strobe == "ts1" else 0
        dut.ts2_valid_i.value = LANE0_MASK if strobe == "ts2" else 0
        await wait_state(dut, nxt, 8000, name)


async def quiesce_in_cfg_idle(dut):
    """In Configuration.Idle, withhold Idle data so the success exit at :962
    cannot fire and only the 2 ms timeout can end the state."""
    dut.ts1_valid_i.value = 0
    dut.ts2_valid_i.value = 0
    dut.idle_valid_i.value = 0
    dut.ordered_set_i.value = 0


@cocotb.test()
async def run_test_cfgidle_c26a(dut):
    cocotb.start_soon(Clock(dut.clk_i, CLK_PERIOD_NS, units="ns").start())

    n_bits = len(dut.ordered_set_i)
    assert n_bits == 128, (
        f"-GMAX_NUM_LANES=1 did not reach the DUT: ordered_set_i is {n_bits} "
        f"bits (x1 expects 128)")

    # ---- reset -> Detect -> Polling -> Configuration.Linkwidth.Start ----
    drive_idle_inputs(dut)
    dut.rst_i.value = 1
    await ClockCycles(dut.clk_i, 5)
    dut.rst_i.value = 0
    await ClockCycles(dut.clk_i, 5)
    assert state(dut) == ST_IDLE

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

    dut.ordered_set_i.value = pack_tsos(link_num=None, lane_num=None)
    dut.ts1_valid_i.value = LANE0_MASK
    await wait_state(dut, ST_POLLING_CONFIG, 4000, "POLLING_CONFIGURATION")

    dut.ts1_valid_i.value = 0
    dut.ts2_valid_i.value = LANE0_MASK
    await wait_state(dut, ST_CFG_LW_START, 4000, "CFG_LINKWIDTH_START")

    await walk_config_to_idle(dut)
    dut._log.info("reached Configuration.Idle (first time)")

    # ================= timeout #1 =================
    await quiesce_in_cfg_idle(dut)
    landed, waited = await wait_leave(
        dut, ST_CFG_IDLE, TIMEOUT_BUDGET, "Config.Idle timeout #1")
    dut._log.info(
        f"timeout #1 fired after ~{waited} cycles -> {sname(landed)}")
    assert landed == ST_RECOVERY_RCVR_LOCK, (
        f"timeout #1 should divert to Recovery.RcvrLock (Base 2.1 p.237, "
        f"idle_to_rlock_transitioned == 0b on the first timeout); got "
        f"{sname(landed)}")

    # ================= Recovery loop back to Configuration.Idle =================
    # Recovery.RcvrLock (:1057): eight consecutive TS1/TS2 on the active lane.
    # speed_change must stay 0 so :1051 and the RcvrCfg speed branches stay out.
    dut.ordered_set_i.value = pack_tsos(link_num=LINK, lane_num=0,
                                        rate=GEN1_RATE, speed_change=0)
    dut.ts1_valid_i.value = LANE0_MASK
    dut.ts2_valid_i.value = 0
    await wait_state(dut, ST_RECOVERY_RCVR_CFG, 8000, "RECOVERY_RCVR_CFG")

    # Recovery.RcvrCfg (:1246): eight consecutive TS2 with a stable ts_s6 and
    # rate_id, no speed_change, and >= 16 transmitted ordered sets.
    dut.ts1_valid_i.value = 0
    dut.ts2_valid_i.value = LANE0_MASK
    await wait_state(dut, ST_RECOVERY_IDLE, 8000, "RECOVERY_IDLE")

    # Recovery.Idle: take the :1446 at_least_one_ts1_ts2 arm back to
    # Configuration.Linkwidth.Start, NOT the :1441 idle arm to L0 (which would
    # clear idle_to_rlock_transitioned at :1445 and destroy the measurement).
    # :1803 only counts a TS whose lane_num is PAD, so PAD it must be.
    dut.idle_valid_i.value = 0
    dut.ordered_set_i.value = pack_tsos(link_num=LINK, lane_num=None,
                                        rate=GEN1_RATE, speed_change=0)
    dut.ts2_valid_i.value = 0
    dut.ts1_valid_i.value = LANE0_MASK
    await wait_state(dut, ST_CFG_LW_START, 8000, "CFG_LINKWIDTH_START (2nd)")

    await walk_config_to_idle(dut)
    dut._log.info("reached Configuration.Idle (second time), "
                  "idle_to_rlock_transitioned not cleared on the way")

    # ================= timeout #2 -- the measurement =================
    await quiesce_in_cfg_idle(dut)
    landed2, waited2 = await wait_leave(
        dut, ST_CFG_IDLE, TIMEOUT_BUDGET, "Config.Idle timeout #2")
    dut._log.info(
        f"timeout #2 fired after ~{waited2} cycles -> {sname(landed2)}")

    assert landed2 in DETECT_FAMILY, (
        f"C26a (Base 2.1 4.2.6.3.6, p.237): idle_to_rlock_transitioned is a "
        f"one-bit variable -- it is set to 1b on the FIRST diversion to "
        f"Recovery.RcvrLock, so the SECOND 2 ms Configuration.Idle timeout "
        f"must go to Detect. This DUT went to {sname(landed2)} instead. "
        f"Cause: pcie_ltssm_downstream.sv:979 compares the whole rate_id_t "
        f"struct (curr_data_rate_r == gen1) instead of the field "
        f"(curr_data_rate_r.rate == gen1, as :530/:1319/:1462 do). rate sits "
        f"at bits [5:1] above an rsvd0 LSB, so the struct holds gen1<<1 == 2 "
        f"while the enum zero-extends to 1; the comparison is identically "
        f"false, the 8'hFF saturation at :980 is dead code, and the counter "
        f"merely increments -- admitting 255 diversions where the spec "
        f"permits one.")
