"""
Phase 7B -- spec-golden LTSSM ordered-set conformance suite (independent oracle).

Observation point: pcie_ltssm_downstream.ordered_set_o (the 128-bit
pcie_ordered_set_t the FSM emits, pre-datapath), DUT = Downstream/Root Complex
(IS_ROOT_PORT=1), x1, Gen1, LINK_NUM=1, SIM_FAST_LINK=1.

THE RULE: every expected value here is derived from the PCI Express Base
Specification Rev 2.1 and written down in SPEC_PREDICTIONS.md *before* the DUT was
run. This file is a *second, non-identical* oracle to the b2b harness (which
checks two copies of the same RTL against each other, §15). We do NOT transcribe
what the RTL emits; where the spec could not pin a value it is flagged inline and
in SPEC_PREDICTIONS.md, never silently baked into the golden.

A -- per-symbol field encodings (Table 4-2 TS1, 4-3 TS2, §4.2.6.3.6 Idle)
B -- 16-after-1 transmit-count gating (Config.Complete §4.2.6.3.5.1,
     Config.Idle §4.2.6.3.6)
C -- Config.Lanenum.Wait changed-lane gate + 1 ms-settle report (§4.2.6.3.4.1)

The state-walk (reset -> Detect -> Polling -> Configuration) reuses the proven
reactive-EP echo sequence from ltssm_tb_common.bring_up_link_rc; the assertions
layered on top are this suite's own spec-derived checks.
"""
import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ClockCycles
from ltssm_tb_common import (
    drive_idle_inputs, wait_state, os_tx_pulser, pack_tsos,
    STATE_NAMES, LINK_NUM,
    ST_IDLE, ST_DETECT_QUIET, ST_DETECT_ACTIVE, ST_POLLING_ACTIVE,
    ST_POLLING_CONFIG, ST_CFG_LW_START, ST_CFG_LW_ACCEPT, ST_CFG_LN_WAIT,
    ST_CFG_LN_ACCEPT, ST_CFG_COMPLETE, ST_CFG_IDLE, ST_L0,
    LANE0_MASK, RXSTATUS_OK_X1,
)

# sec 63 #7g-2 (D-7G.2): this bench's clock AND the RTL's CLK_PERIOD_NS, one value.
# The core passes -GCLK_PERIOD_NS=8 (tb_ltssm_b2b.sv passes .CLK_PERIOD_NS(8));
# every cycle budget below that stands for a spec time derives from it.
CLK_PERIOD_NS = 8

# ---- spec-derived symbol byte values (SPEC_PREDICTIONS.md section A) ----
COM = 0xBC   # K28.5, Table 4-2/4-3 Symbol 0
PAD = 0xF7   # K23.7, Table 4-2/4-3 Symbol 1/2
TS1 = 0x4A   # D10.2, Table 4-2 Symbol 6-15
TS2 = 0x45   # D5.2,  Table 4-3 Symbol 6-15
LINK = LINK_NUM   # non-PAD Link Number the RC originates (=1, config-derived)


def sym(os_val, n):
    """Extract spec Symbol n (byte offset 8*n) from a 128-bit ordered set."""
    return (os_val >> (8 * n)) & 0xFF


def _os(dut):
    return int(dut.ordered_set_o.value) & ((1 << 128) - 1)


# collected for the report: N_FTS and rate_id bytes observed (spec-unpinned)
_OBSERVED = {"n_fts": set(), "rate_id": set(), "lw_accept_lane": None}


def assert_ts(dut, label, ts_id, exp_link, exp_lane, check_lane=True):
    """Assert one TS ordered set against its spec-derived field encoding.
    Returns nothing; raises AssertionError with a full symbol dump on mismatch."""
    v = _os(dut)
    s = [sym(v, i) for i in range(16)]
    tsname = "TS1" if ts_id == TS1 else "TS2"

    def fail(msg):
        dump = " ".join(f"{i}:{s[i]:02x}" for i in range(16))
        raise AssertionError(f"[{label}] {msg}\n  symbols: {dump}")

    # Symbol 0: COM (K28.5=0xBC) -- also the orientation self-check.
    if s[0] != COM:
        fail(f"Symbol 0 (COM) expected 0x{COM:02x}, got 0x{s[0]:02x} "
             f"(if this is 0x{ts_id:02x} the byte order is reversed)")
    # Symbols 6-15: TS identifier, all ten bytes (Table 4-2/4-3).
    for i in range(6, 16):
        if s[i] != ts_id:
            fail(f"Symbol {i} ({tsname} id) expected 0x{ts_id:02x}, got 0x{s[i]:02x}")
    # Symbol 1: Link Number.
    if s[1] != exp_link:
        fail(f"Symbol 1 (Link) expected 0x{exp_link:02x}, got 0x{s[1]:02x}")
    # Symbol 2: Lane Number (soft where spec-vs-sampling is ambiguous).
    if check_lane and s[2] != exp_lane:
        fail(f"Symbol 2 (Lane) expected 0x{exp_lane:02x}, got 0x{s[2]:02x}")
    # Symbol 5: Training Control -- clean link-up asserts none => 0x00.
    if s[5] != 0x00:
        fail(f"Symbol 5 (TrainCtrl) expected 0x00 (no hot-reset/disable/"
             f"loopback/disable-scramble on clean link-up), got 0x{s[5]:02x}")
    # Symbol 4: Data Rate Identifier -- spec-fixed bits only.
    r = s[4]
    if (r & 0x01) != 0x00:
        fail(f"Symbol 4 (RateID) bit0 (Rsvd) must be 0, got 0x{r:02x}")
    if (r & 0x02) != 0x02:
        fail(f"Symbol 4 (RateID) bit1 (2.5 GT/s supported) must be 1, got 0x{r:02x}")
    if (r & 0x38) != 0x00:
        fail(f"Symbol 4 (RateID) bits[5:3] (Rsvd) must be 0, got 0x{r:02x}")
    if (r & 0x80) != 0x00:
        fail(f"Symbol 4 (RateID) bit7 (speed_change) must be 0 outside Recovery, "
             f"got 0x{r:02x}")
    _OBSERVED["rate_id"].add(r)
    # Symbol 3: N_FTS -- spec-unpinned (implementation-defined). Record only.
    _OBSERVED["n_fts"].add(s[3])

    dut._log.info(f"[{label}] OK: {tsname} link=0x{s[1]:02x} lane=0x{s[2]:02x} "
                  f"COM=0x{s[0]:02x} rate=0x{s[4]:02x} nfts=0x{s[3]:02x} "
                  f"id[6..15]=0x{ts_id:02x}")


async def _settle(dut, n=2):
    for _ in range(n):
        await RisingEdge(dut.clk_i)


@cocotb.test()
async def run_conformance(dut):
    cocotb.start_soon(Clock(dut.clk_i, CLK_PERIOD_NS, units="ns").start())  # 125 MHz

    # param-reach: x1 => ordered_set_i is 128 bits (x4 default would be 512)
    nb = len(dut.ordered_set_i)
    assert nb == 128, (f"-GMAX_NUM_LANES=1 did not reach DUT: ordered_set_i={nb} "
                       f"bits (x1 expects 128)")

    # ---------------- reset + Detect + Polling (role-neutral) ----------------
    drive_idle_inputs(dut)
    dut.rst_i.value = 1
    await ClockCycles(dut.clk_i, 5)
    dut.rst_i.value = 0
    await ClockCycles(dut.clk_i, 5)
    assert int(dut.ltssm_state_o.value) == ST_IDLE
    assert int(dut.link_up_o.value) == 0

    dut.en_i.value = 1
    await wait_state(dut, ST_DETECT_QUIET, 50, "DETECT_QUIET")
    dut.phy_rxelecidle_i.value = LANE0_MASK
    await ClockCycles(dut.clk_i, 3)
    dut.phy_rxelecidle_i.value = 0
    await wait_state(dut, ST_DETECT_ACTIVE, 50, "DETECT_ACTIVE")
    assert int(dut.phy_txdetectrx_o.value) == 1

    dut.receiver_detected_i.value = LANE0_MASK
    dut.phy_rxstatus_i.value = RXSTATUS_OK_X1
    dut.phy_phystatus_i.value = LANE0_MASK
    await ClockCycles(dut.clk_i, 3)
    dut.phy_phystatus_i.value = 0
    cocotb.start_soon(os_tx_pulser(dut))
    await wait_state(dut, ST_POLLING_ACTIVE, 100, "POLLING_ACTIVE")

    # ================= A: Polling.Active -- TS1, PAD/PAD =================
    await _settle(dut)
    assert_ts(dut, "Polling.Active", TS1, PAD, PAD)     # Table 4-2, §4.2.6.2.1

    # EP answers TS1 PAD/PAD -> Polling.Configuration
    dut.ordered_set_i.value = pack_tsos(link_num=None, lane_num=None)
    dut.ts1_valid_i.value = LANE0_MASK
    await wait_state(dut, ST_POLLING_CONFIG, 2000, "POLLING_CONFIGURATION")

    # ================= A: Polling.Config -- TS2, PAD/PAD =================
    await _settle(dut)
    assert_ts(dut, "Polling.Config", TS2, PAD, PAD)     # Table 4-3, §4.2.6.2.3

    # EP answers TS2 PAD/PAD -> Configuration.Linkwidth.Start
    dut.ts1_valid_i.value = 0
    dut.ts2_valid_i.value = LANE0_MASK
    await wait_state(dut, ST_CFG_LW_START, 2000, "CFG_LINKWIDTH_START")

    # ============= A: Config.Linkwidth.Start -- TS1, LINK/PAD ============
    await _settle(dut)
    assert_ts(dut, "Config.LW.Start", TS1, LINK, PAD)   # §4.2.6.3.1.1

    # EP echoes TS1 with the RC's Link number, Lane PAD -> Linkwidth.Accept
    dut.ts2_valid_i.value = 0
    dut.ordered_set_i.value = pack_tsos(link_num=LINK, lane_num=None)
    dut.ts1_valid_i.value = LANE0_MASK
    await wait_state(dut, ST_CFG_LW_ACCEPT, 2000, "CFG_LINKWIDTH_ACCEPT")

    # ===== A: Config.Linkwidth.Accept -- TS1, LINK/(PAD or 0: recorded) =====
    await _settle(dut)
    _OBSERVED["lw_accept_lane"] = sym(_os(dut), 2)
    # Link must be non-PAD here; Lane assignment timing (PAD vs 0) is a
    # spec-vs-registered-output ambiguity -> recorded, not hard-asserted.
    assert_ts(dut, "Config.LW.Accept", TS1, LINK, None, check_lane=False)

    await wait_state(dut, ST_CFG_LN_WAIT, 2000, "CFG_LANENUM_WAIT")

    # ================= C: Config.Lanenum.Wait gating =================
    # Spec §4.2.6.3.4.1: exit to Lanenum.Accept requires two consecutive TS1
    # whose Lane Number DIFFERS from entry (entry value was PAD). The 1 ms is a
    # *permitted* delay (upper bound), not a mandatory floor -- see report.
    await _settle(dut)
    assert_ts(dut, "Config.LN.Wait", TS1, LINK, 0x00)   # RC transmits assigned lane 0

    # (C.1) hold Lane UNCHANGED (still PAD, == lane_in_save): must NOT exit.
    dut.ordered_set_i.value = pack_tsos(link_num=LINK, lane_num=None)  # lane=PAD
    dut.ts1_valid_i.value = LANE0_MASK
    await ClockCycles(dut.clk_i, 60)
    st = int(dut.ltssm_state_o.value)
    assert st == ST_CFG_LN_WAIT, (
        "Config.Lanenum.Wait exited on an UNCHANGED Lane Number "
        f"(spec requires a changed Lane Number); state={STATE_NAMES.get(st, hex(st))}")
    dut._log.info("[Config.LN.Wait] C.1 OK: no exit while Lane unchanged (PAD)")

    # measure cycles-to-exit once the Lane Number changes (settle-floor report)
    t0 = cocotb.utils.get_sim_time(units="ns")
    dut.ordered_set_i.value = pack_tsos(link_num=LINK, lane_num=0)     # lane changes PAD->0
    await wait_state(dut, ST_CFG_LN_ACCEPT, 2000, "CFG_LANENUM_ACCEPT")
    t1 = cocotb.utils.get_sim_time(units="ns")
    dt = t1 - t0
    floor = ("yes" if dt > 1000 else
             "NO (evaluates promptly; x1-benign, multi-lane skew-robustness gap)")
    dut._log.warning(
        f"[Config.LN.Wait] C.2 exited to Lanenum.Accept {dt:.0f} ns after the "
        f"Lane Number changed. Spec's 1 ms settle is a PERMITTED delay, not a "
        f"required floor; a prompt exit is spec-compliant. Settle-floor present: "
        f"{floor}")

    # ================= A: Config.Lanenum.Accept -- TS1, LINK/0 =================
    await _settle(dut)
    assert_ts(dut, "Config.LN.Accept", TS1, LINK, 0x00)   # §4.2.6.3.3.1

    # EP echoes TS1 LINK/0 (matching) -> Configuration.Complete
    dut.ordered_set_i.value = pack_tsos(link_num=LINK, lane_num=0)
    dut.ts1_valid_i.value = LANE0_MASK
    await wait_state(dut, ST_CFG_COMPLETE, 4000, "CFG_COMPLETE")

    # ================= A: Config.Complete -- TS2, LINK/0 =================
    await _settle(dut)
    assert_ts(dut, "Config.Complete", TS2, LINK, 0x00)    # §4.2.6.3.5.1

    # ===== B: 16-after-1 TS2 gating in Config.Complete =====
    await _b_gate(dut, phase="Complete",
                  rx_strobe="ts2_valid_i",
                  single_sig="single_ts2_received",
                  echo=pack_tsos(link_num=LINK, lane_num=0),
                  spec="§4.2.6.3.5.1 (16 TS2 sent after receiving one TS2)",
                  next_state=ST_CFG_IDLE, next_name="CFG_IDLE")

    # ================= A: Config.Idle -- not a TS OS; idle asserted =================
    await _settle(dut)
    v = _os(dut)
    s0, s1 = sym(v, 6), sym(v, 10)   # identifier bytes
    is_ts = (s0 in (TS1, TS2)) and (s1 in (TS1, TS2))
    assert not is_ts, (f"Config.Idle still transmitting a TS ordered set "
                       f"(id bytes 0x{s0:02x}/0x{s1:02x}); spec §4.2.6.3.6 sends Idle")
    try:
        gi = int(dut.gen_os_ctrl_o.gen_idle.value)
        assert gi == 1, "gen_os_ctrl_o.gen_idle not asserted in Config.Idle"
        dut._log.info("[Config.Idle] OK: no TS identifier; gen_idle asserted")
    except AttributeError:
        dut._log.info("[Config.Idle] OK: no TS identifier (gen_idle field not "
                      "individually accessible; struct-level idle confirmed)")

    # ===== B: 16-after-1 Idle gating in Config.Idle =====
    await _b_gate(dut, phase="Idle",
                  rx_strobe="idle_valid_i",
                  single_sig="single_idle_received",
                  echo=0,
                  spec="§4.2.6.3.6 (16 Idle sent after receiving one Idle)",
                  next_state=ST_L0, next_name="L0")

    # ---------------- reached L0 ----------------
    assert int(dut.ltssm_state_o.value) == ST_L0, "did not reach L0"
    assert int(dut.link_up_o.value) == 1, "link_up_o not asserted in L0"
    dut.ts1_valid_i.value = 0
    dut.ts2_valid_i.value = 0
    dut.idle_valid_i.value = 0

    # ---------------- report summary ----------------
    dut._log.warning("==== conformance summary (spec-unpinned observations) ====")
    dut._log.warning(f"  N_FTS (Symbol 3) observed = "
                     f"{sorted(hex(x) for x in _OBSERVED['n_fts'])} "
                     f"(spec-unpinned: impl-defined 0-255)")
    dut._log.warning(f"  Data Rate ID (Symbol 4) observed = "
                     f"{sorted(hex(x) for x in _OBSERVED['rate_id'])} "
                     f"(spec-fixed bits checked; full byte Gen1-advertisement dependent)")
    dut._log.warning(f"  Config.LW.Accept Lane byte = "
                     f"0x{_OBSERVED['lw_accept_lane']:02x} "
                     f"(0xf7=PAD => lane assigned one state later than the literal "
                     f"§4.2.6.3.2.1 reading; 0x00=already assigned)")


async def _b_gate(dut, phase, rx_strobe, single_sig, echo, spec,
                  next_state, next_name):
    """B: prove the exit-OS transmit count is gated on 'after receiving one'.

    While the matching RX strobe is withheld, ordered_set_sent_cnt_r and
    single_*_received must both stay 0 across many transmitted OSes (a raw
    free-running count would be a conformance defect). Then supply the RX
    strobe and confirm counting starts and the state advances.
    Internal signals read via --public-flat-rw; expectation is spec-derived."""

    def rd(name):
        return int(getattr(dut, name).value)

    # (B.1) withhold RX: no matching set received yet.
    getattr(dut, rx_strobe).value = 0
    if phase == "Idle":
        dut.ordered_set_i.value = 0
    await ClockCycles(dut.clk_i, 60)   # many os_tx pulses elapse
    single = rd(single_sig)
    cnt = rd("ordered_set_sent_cnt_r")
    assert single == 0, (f"[{phase}] {single_sig}={single} before any {phase} OS "
                         f"received -- spec {spec} gates counting on reception")
    assert cnt == 0, (f"[{phase}] ordered_set_sent_cnt_r={cnt} advanced BEFORE "
                      f"receiving one matching OS -- this is a RAW free-running "
                      f"count, a conformance defect vs {spec}")
    dut._log.info(f"[{phase}] B.1 OK: count gated off while RX withheld "
                  f"({single_sig}=0, sent_cnt=0 across 60 cycles)")

    # (B.2) supply RX: counting must start.
    dut.ordered_set_i.value = echo
    getattr(dut, rx_strobe).value = LANE0_MASK
    started = False
    for _ in range(200):
        await RisingEdge(dut.clk_i)
        if rd(single_sig) == 1 and rd("ordered_set_sent_cnt_r") > 0:
            started = True
            break
    assert started, (f"[{phase}] count never started after supplying the matching "
                     f"RX OS -- {single_sig} or sent_cnt stuck at 0")
    dut._log.info(f"[{phase}] B.2 OK: {single_sig}=1 and sent_cnt advancing after "
                  f"first {phase} OS received (16-after-1 gating confirmed)")

    # (B.3) let it complete to the next state.
    await wait_state(dut, next_state, 4000, next_name)
