"""
Rung 10c, Item 3 -- the A3 pattern hunt: ordered-set control/template coherence
across state exits.

ORACLE (ORACLES_LTSSM.md section A3, restated)
  In every exit block, gen_os_ctrl_c and ordered_set_c must describe the same
  ordered set -- and by the convention the code states in its own words at
  :836-:842, the set required by the NEXT state.

WHY THE WHOLE CONTROL WORD IS ASSERTED, NOT ONE FIELD
  gen_os_ctrl_c defaults to gen_os_ctrl_r (:505) and ordered_set_c defaults to
  ordered_set_r (:496). Both are STICKY. An exit block that writes neither
  hands the next state whatever the previous state left behind, so a test that
  checked only the field it expected to change would pass on stale state. Each
  row below reads the TS-type fields together (gen_ts1, gen_ts2, gen_idle) and
  reports all of them.

THE TWO LIVE SKEWS (evidence/rung10/PREDICTIONS_A3_HUNT.md)

  A3-4 (new)  ST_CONFIGURATION_IDLE's 2 ms timeout exit, :976-:991.
      The L0 exit beside it (:962-:975) clears gen_ts1/gen_ts2/gen_idle/valid
      before leaving. The timeout exit writes NO gen_os_ctrl_c field and NO
      ordered_set_c before :984 next_state = ST_RECOVERY_RCVR_LOCK, so both
      stay at Configuration.Idle's own values: gen_idle=1, valid=1, and the
      gen_zeros() Idle template from :935.
      Recovery.RcvrLock (oracle R1, Base 2.1 p.239) requires TS1 Ordered Sets.
      RcvrLock's own arm builds a template only on the speed-change path
      (:1054, N/A-Gen1) or on the way out (:1072), so after this entry the DUT
      transmits Idle for the whole time it sits in RcvrLock.
      Of RcvrLock's three Gen1-reachable entries, :1010 (from L0) and :1039
      (from ST_RECOVERY) both build TS1; only :984 does not.

  A3-2 (pre-registered)  ST_RECOVERY_EXT_SYNCH's exit, :1203-:1224.
      Control is set on entry to valid=1, gen_ts1=1, set_lane=1 (:1204-:1206)
      and never changed on the way out; the matching build is COMMENTED OUT at
      :1221. The exit targets ST_RECOVERY_RCVR_CFG, which by oracle R8
      (p.243) must transmit TS2. Control says TS1, next state needs TS2.

NOT COVERED HERE, and why:
  * A3-1 (Lanenum.Wait exit, :898-:907: ctrl set_lane=1 but the EP template
    carries PAD) is a FIELD-level skew, not a missing-build one, and it is
    EP-role only. Recorded in the predictions file; a different class of row.
  * A3-3 (Speed.EIEOS wipe at :1417) is N/A-Gen3 -- see
    evidence/rung10/A3_A7_DISPOSITION.md section 2.
  * The ten remaining no-build arms are excluded by structure (unreachable) or
    scope (N/A-Gen1/Gen3); the triage table is in the predictions file.

Each expect_fail row is paired with an ordinary PASS control that proves the
drive sequence reached the state under test, because an expect_fail row goes
green if ANYTHING in it raises -- including a broken setup.

Requires SIM_FAST_LINK=1, MAX_NUM_LANES=4 (verilate_ltssm_a3 target).
No src/ edit.
"""
import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, RisingEdge, Timer
from ltssm_tb_common import *  # noqa

# sec 63 #7g-2 (D-7G.2): this bench's clock AND the RTL's CLK_PERIOD_NS, one value.
# The core passes -GCLK_PERIOD_NS=8 (tb_ltssm_b2b.sv passes .CLK_PERIOD_NS(8));
# every cycle budget below that stands for a spec time derives from it.
CLK_PERIOD_NS = 8

# ---- gen_os_struct_t bit positions, from src/packages/pcie_phy_pkg.sv:268-284.
# Packed struct, first field declared = MSB; counting up from bit 0:
#   valid 0, gen_ts1 1, gen_ts2 2, gen2_eieos 3, gen3_eieos 4, gen_eios 5,
#   gen_skp 6, gen_idle 7, set_link 8, set_lane 9, set_speed_change 10,
#   link_number [18:11], rate_id [26:19], ts6_sym [34:27]  => 35 bits.
GEN_OS_WIDTH = 35
B_VALID, B_GEN_TS1, B_GEN_TS2 = 0, 1, 2
B_GEN_IDLE, B_SET_LANE = 7, 9

# TwoMsTimeOut is NOT SIM_FAST_LINK-scaled: (2*10**6)/10 = 200_000 cycles
# (pcie_ltssm_downstream.sv:113).
TWO_MS_CYCLES = 2_000_000 // CLK_PERIOD_NS   # 63 #7g-2: was 200_000 (10 ns)
TIMEOUT_SLACK = 20_000

# ST_RECOVERY_EXT_SYNCH exits on ordered_set_sent_cnt_r >= 1024 (:1212).
# os_tx_pulser drives ordered_set_tranmitted_i one cycle in four, so ~4100
# cycles; budget generously.
EXT_SYNCH_CYCLES = 20_000


def state(dut):
    return int(dut.ltssm_state_o.value)


def sname(s):
    return STATE_NAMES.get(s, hex(s))


def ctrl(dut):
    return int(dut.gen_os_ctrl_o.value)


def ctrl_fields(dut):
    """The TS-type fields, read together -- never one at a time (stickiness)."""
    c = ctrl(dut)
    return {
        "valid":    (c >> B_VALID) & 1,
        "gen_ts1":  (c >> B_GEN_TS1) & 1,
        "gen_ts2":  (c >> B_GEN_TS2) & 1,
        "gen_idle": (c >> B_GEN_IDLE) & 1,
        "set_lane": (c >> B_SET_LANE) & 1,
    }


def fmt(f):
    return " ".join(f"{k}={v}" for k, v in f.items())


def check_geometry(dut):
    n = len(dut.ordered_set_i)
    assert n == 512, (
        f"-GMAX_NUM_LANES=4 did not reach the DUT: ordered_set_i is {n} bits")
    w = len(dut.gen_os_ctrl_o)
    assert w == GEN_OS_WIDTH, (
        f"gen_os_struct_t is {w} bits, expected {GEN_OS_WIDTH}; the bit "
        f"positions in this file are stale -- re-derive from pcie_phy_pkg.sv")


async def bring_up_link_to_cfg_idle(dut):
    """bring_up_link()'s sequence, stopped at CFG_IDLE with no idles driven.

    bring_up_link() itself carries on into L0 by presenting idles; this stops
    one state earlier so Configuration.Idle's 2 ms timeout arm is the exit
    taken instead of its idle-driven L0 exit.
    """
    drive_idle_inputs(dut)
    dut.rst_i.value = 1
    await ClockCycles(dut.clk_i, 5)
    dut.rst_i.value = 0
    await ClockCycles(dut.clk_i, 5)

    dut.en_i.value = 1
    await wait_state(dut, ST_DETECT_QUIET, 50, "DETECT_QUIET")
    dut.phy_rxelecidle_i.value = ALL
    await ClockCycles(dut.clk_i, 3)
    dut.phy_rxelecidle_i.value = 0
    await wait_state(dut, ST_DETECT_ACTIVE, 50, "DETECT_ACTIVE")

    dut.receiver_detected_i.value = ALL
    dut.phy_rxstatus_i.value = RXSTATUS_ALL_OK
    dut.phy_phystatus_i.value = ALL
    await ClockCycles(dut.clk_i, 3)
    dut.phy_phystatus_i.value = 0
    cocotb.start_soon(os_tx_pulser(dut))
    await wait_state(dut, ST_POLLING_ACTIVE, 100, "POLLING_ACTIVE")

    dut.ordered_set_i.value = pack_tsos_all_lanes(link_num=None, lane_num=None)
    dut.ts1_valid_i.value = ALL
    await wait_state(dut, ST_POLLING_CONFIG, 2000, "POLLING_CONFIGURATION")

    dut.ts1_valid_i.value = 0
    dut.ts2_valid_i.value = ALL
    await wait_state(dut, ST_CFG_LW_START, 2000, "CFG_LINKWIDTH_START")

    dut.ts2_valid_i.value = 0
    dut.ordered_set_i.value = pack_tsos_all_lanes(link_num=LINK_NUM, lane_num=None)
    dut.ts1_valid_i.value = ALL
    await wait_state(dut, ST_CFG_LW_ACCEPT, 1000, "CFG_LINKWIDTH_ACCEPT")
    await wait_state(dut, ST_CFG_LN_WAIT, 1000, "CFG_LANENUM_WAIT")

    dut.ordered_set_i.value = pack_tsos_all_lanes(link_num=LINK_NUM, lane_num="index")
    await wait_state(dut, ST_CFG_LN_ACCEPT, 1000, "CFG_LANENUM_ACCEPT")
    await wait_state(dut, ST_CFG_COMPLETE, 1000, "CFG_COMPLETE")

    dut.ts1_valid_i.value = 0
    dut.ts2_valid_i.value = ALL
    await wait_state(dut, ST_CFG_IDLE, 2000, "CFG_IDLE")
    # Stop here: present nothing further, so the 2 ms timeout arm is reached
    # instead of the idle-driven L0 exit.
    dut.ts2_valid_i.value = 0


# ==========================================================================
#  A3-4 -- Configuration.Idle's timeout exit leaves the control word stale.
# ==========================================================================

@cocotb.test()
async def test_a3_4_control_reaches_rcvrlock(dut):
    """Control for A3-4: Configuration.Idle's 2 ms timeout really reaches RcvrLock."""
    cocotb.start_soon(Clock(dut.clk_i, CLK_PERIOD_NS, units="ns").start())
    check_geometry(dut)
    await bring_up_link_to_cfg_idle(dut)
    assert state(dut) == ST_CFG_IDLE

    await wait_state(dut, ST_RECOVERY_RCVR_LOCK, TWO_MS_CYCLES + TIMEOUT_SLACK,
                     "RECOVERY_RCVR_LOCK (via CFG_IDLE 2 ms timeout)")
    dut._log.info(
        "A3-4 CONTROL OK: Configuration.Idle 2 ms timeout -> RECOVERY_RCVR_LOCK")


@cocotb.test()
async def test_a3_4_rcvrlock_control_word_describes_ts1(dut):
    """A3-4: on entering RcvrLock from Configuration.Idle, control must say TS1.
    CLOSED -- tracker sec 54 #7.

    Base 2.1 p.239 (oracle R1): Recovery.RcvrLock transmits TS1 Ordered Sets.

    WAS a predicted divergence (Rung 10c, A3-4) and carried expect_fail until
    FA-5b.  The DUT arrived with Configuration.Idle's Idle control still in
    place, because the 2 ms timeout arm wrote neither gen_os_ctrl_c nor
    ordered_set_c and both are sticky -- measured gen_idle=1, gen_ts1=0.  The
    fix builds the TS1 control word and template in that arm, copying the shape
    of ST_L0's own entry into the same state.

    ⚠️ 10c cited the arm as ":976-:991"; that numbering is STALE -- FA-1's own
    ab776cf inserted an eight-line comment above it.  At the time of the fix the
    state is :962-:1009 and the timeout arm :984-:1008.

    ⚠️ The contrast that explains why this survived so long: the SUCCESS path one
    branch up clears all four control bits before leaving.  Only the timeout path
    forgot, so the defect is invisible on a link that trains normally and appears
    only after a 2 ms Configuration.Idle timeout.

    Paired with test_a3_4_control_reaches_rcvrlock, which proves the timeout
    really reaches this state -- so this row cannot pass for a setup reason
    (tracker sec 22.81).
    """
    cocotb.start_soon(Clock(dut.clk_i, CLK_PERIOD_NS, units="ns").start())
    check_geometry(dut)
    await bring_up_link_to_cfg_idle(dut)

    await wait_state(dut, ST_RECOVERY_RCVR_LOCK, TWO_MS_CYCLES + TIMEOUT_SLACK,
                     "RECOVERY_RCVR_LOCK (via CFG_IDLE 2 ms timeout)")
    await ClockCycles(dut.clk_i, 2)
    await Timer(1, units="ps")

    f = ctrl_fields(dut)
    dut._log.info(f"A3-4: control word on entry to RcvrLock -> {fmt(f)}")
    assert f["gen_ts1"] == 1 and f["gen_idle"] == 0, (
        f"A3-4 violated: entered Recovery.RcvrLock with control word {fmt(f)}; "
        f"Base 2.1 p.239 requires TS1 Ordered Sets, but Configuration.Idle's "
        f"timeout exit (:976-:991) left the Idle control in place")


@cocotb.test()
async def test_a3_4_rcvrlock_ordered_set_template_is_ts1(dut):
    """A3-4, the TEMPLATE half: the ordered set itself must be a TS1, not Idle.

    ⚠️ WHY THIS ROW EXISTS.  It was added in FA-5b to answer a SURVIVING mutant,
    not to re-test a covered claim (tracker sec 22.34).  M7b drops ONLY the
    `ordered_set_c = gen_ts_os(...)` line from the fix, keeping the control
    flags, and test_a3_4_rcvrlock_control_word_describes_ts1 stayed GREEN --
    measured, 4/4.  So that row reads the gen_os_ctrl_o FLAGS and nothing else,
    and the template half of the fix had no oracle at all.

    The two halves are genuinely different behaviours, which is why an
    equivalence argument was not available:

      * the flags tell os_generator to BUILD a TS1
      * ordered_set_o carries WHAT to build -- Link Number, Lane Number, rate,
        and Symbols 6-15's identifier

    Without the template write, Recovery.RcvrLock is entered with
    Configuration.Idle's gen_zeros() Idle template still in place (this file's
    own header records it), so the DUT would announce TS1 and emit an all-zero
    ordered set.

    ORACLE: Base 2.1 Table 4-2 pp.201-202 -- Symbols "6 - 15  D10.2" are the TS1
    Identifier, D10.2 = 4Ah.  Ten Symbols, all of them.

    Paired with test_a3_4_control_reaches_rcvrlock, which proves the 2 ms
    timeout really reaches this state (tracker sec 22.81).
    """
    cocotb.start_soon(Clock(dut.clk_i, CLK_PERIOD_NS, units="ns").start())
    check_geometry(dut)
    await bring_up_link_to_cfg_idle(dut)

    await wait_state(dut, ST_RECOVERY_RCVR_LOCK, TWO_MS_CYCLES + TIMEOUT_SLACK,
                     "RECOVERY_RCVR_LOCK (via CFG_IDLE 2 ms timeout)")
    await ClockCycles(dut.clk_i, 2)
    await Timer(1, units="ps")

    # Lane 0's template is the low 128 bits of ordered_set_o (16 Symbols x 8b).
    os_all = int(dut.ordered_set_o.value)
    lane0 = os_all & ((1 << 128) - 1)
    syms = [(lane0 >> (8 * s)) & 0xFF for s in range(16)]
    ident = syms[6:16]
    dut._log.info("A3-4 template on entry to RcvrLock -> Symbols 0-5 %s  6-15 %s"
                  % (" ".join("%02x" % s for s in syms[:6]),
                     " ".join("%02x" % s for s in ident)))

    assert ident == [TS1] * 10, (
        "A3-4 template violated: entered Recovery.RcvrLock with Symbols 6-15 = "
        "%s; Base 2.1 Table 4-2 pp.201-202 requires all ten to be the TS1 "
        "Identifier D10.2 = %02xh.  Configuration.Idle's timeout exit must "
        "write ordered_set_c, not only the control flags."
        % (" ".join("%02x" % s for s in ident), TS1))


# ==========================================================================
#  A3-2 -- Recovery.ExtSynch's exit leaves the control word saying TS1.
# ==========================================================================

async def drive_to_ext_synch(dut):
    """Reset -> L0 -> RcvrLock with extended_synch_i set -> ExtSynch."""
    cocotb.start_soon(Clock(dut.clk_i, CLK_PERIOD_NS, units="ns").start())
    check_geometry(dut)
    await bring_up_link(dut)
    assert state(dut) == ST_L0

    dut.extended_synch_i.value = 1
    dut.ordered_set_i.value = pack_tsos_all_lanes(
        link_num=LINK_NUM, lane_num="index", rate=GEN1_RATE, speed_change=0)
    dut.ts1_valid_i.value = ALL
    await wait_state(dut, ST_RECOVERY_RCVR_LOCK, 100, "RECOVERY_RCVR_LOCK")
    await wait_state(dut, ST_RECOVERY_EXT_SYNCH, 500, "RECOVERY_EXT_SYNCH")


@cocotb.test()
async def test_a3_2_control_reaches_rcvrcfg(dut):
    """Control for A3-2: extended_synch_i reaches ExtSynch, which exits to RcvrCfg.

    Also confirms oracle R4: the exit is gated on 1024 ordered sets (:1212),
    which is exactly Base 2.1 p.240's Extended Synch minimum.
    """
    await drive_to_ext_synch(dut)
    dut._log.info("A3-2 CONTROL: reached RECOVERY_EXT_SYNCH")

    await wait_state(dut, ST_RECOVERY_RCVR_CFG, EXT_SYNCH_CYCLES,
                     "RECOVERY_RCVR_CFG (after 1024 ordered sets)")
    dut._log.info(
        "A3-2 CONTROL OK: ExtSynch -> RCVR_CFG after the 1024-OS gate (R4 conforms)")


@cocotb.test(expect_fail=True)
async def test_a3_2_rcvrcfg_control_word_describes_ts2(dut):
    """A3-2: on entering RcvrCfg from ExtSynch, control must say TS2.

    Base 2.1 p.243 (oracle R8): Recovery.RcvrCfg transmits TS2 Ordered Sets.
    ExtSynch sets gen_ts1=1 on entry (:1205) and never revises it on the way
    out; the matching ordered_set_c build is commented out at :1221.
    """
    await drive_to_ext_synch(dut)
    await wait_state(dut, ST_RECOVERY_RCVR_CFG, EXT_SYNCH_CYCLES,
                     "RECOVERY_RCVR_CFG")
    await ClockCycles(dut.clk_i, 2)
    await Timer(1, units="ps")

    f = ctrl_fields(dut)
    dut._log.info(f"A3-2: control word on entry to RcvrCfg -> {fmt(f)}")
    assert f["gen_ts2"] == 1 and f["gen_ts1"] == 0, (
        f"A3-2 violated: entered Recovery.RcvrCfg with control word {fmt(f)}; "
        f"Base 2.1 p.243 requires TS2, but ExtSynch's exit never revised the "
        f"TS1 control it set at :1205 and its template build is commented out "
        f"at :1221")
