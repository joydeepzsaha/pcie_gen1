"""pcie_flow_ctrl_init -- the FC_INIT2 exit, against Base 2.1 §3.3.1 ONLY.

§63 #7d, probe phase. ⚠️ THE ROW IS RED TODAY AND IS MEANT TO BE.

== THE ORACLE IS THE SPEC, NOT THE RTL ====================================
Base 2.1 §3.3.1 leaves FC_INIT2 on:

    full FC2 set SENT  ∧  ( InitFC2 received ∨ UpdateFC received ∨ TLP received )

The RTL's gate is pcie_flow_ctrl_init.sv:401:

    fc2_values_stored_i && (update_fc_r == '1 || idle_count_r >= 16'h60)

`fc2_values_stored_i` is the "InitFC2 received" limb -- the FC2 values are stored
precisely because the peer's InitFC2 arrived. Per §3.3.1 that limb ALONE, with
the FC2 set sent, is sufficient. The RTL instead makes it a CONJUNCT with an
extra term of its own invention:

    update_fc_r            -- "UpdateFC received", a genuine §3.3.1 limb, but the
                              spec offers it as an ALTERNATIVE, not an additional
                              requirement
    idle_count_r >= 0x60   -- an idle-symbol timeout with NO counterpart anywhere
                              in §3.3.1

⭐ CANDIDATE CONFORMANCE DEFECT #6, same family as #4.

== WHY THIS ROW EXISTS, measured ==========================================
§63 #7d probe phase, 2026-09-16:

  full stack (tb_pcie_fullstack)     idle_count_r never leaves 0 (idle_valid_i
                                     does pulse -- 254 cycles RC, 418 EP -- but
                                     never accumulates), update_fc_i high on
                                     ZERO cycles. Exit term NEVER true; the FSM
                                     loops ST_FC2..CHECK_FC2 7,074 times;
                                     ST_FC_COMPLETE is never entered and
                                     fc_initialized_o never rises. Row 1b red.

  direct-wired (tb_pcie_rc_ep, #33)  idle_valid_i is TIED TO link_up by the bench
                                     (test_pcie_rc_ep.py:180), so idle_count_r
                                     crosses 0x60 in ~96 cycles. Measured
                                     idle_ge60_first=4453 vs update_fc_first=4469
                                     -- the bench exits on idle_count_r, 16
                                     cycles BEFORE update_fc_r is ever high.

So the ONLY thing that has ever completed FC init in this project is the
non-spec idle-timeout fallback, satisfied by a bench that drives the input
continuously in a way no real PHY does. THAT is what this row removes.

== WHAT THIS ROW DOES ====================================================
Drives the spec's condition and NOTHING ELSE: the FC2 set is sent,
fc2_values_stored_i is asserted (InitFC2 received), and BOTH extra terms are
held OFF -- idle_valid_i low so idle_count_r cannot accumulate, update_fc_i low
so update_fc_r cannot set. Per §3.3.1 the FSM must leave FC_INIT2 and assert
fc2_values_sent_o. Today it does not.

!! The companion row drives the SAME stimulus plus the idle fallback, and is
GREEN today -- it is the control that proves the bench can drive this FSM at
all, so a red spec row cannot be dismissed as a dead apparatus.
"""

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ClockCycles, ReadOnly

CLK_NS = 8  # 125 MHz, the rate the link actually runs at

# From pcie_flow_ctrl_init.sv's flow_control_state_e, verified by elaboration in
# the §63 #7d probe phase (the FSM histogram named these by index).
ST_IDLE = 0
CHECK_FC1 = 7
ST_FC2 = 8
CHECK_FC2 = 16
ST_UPDATE_P = 17
ST_FC_COMPLETE = 21


class TB:
    """ONE TB per test -- cocotb kills every coroutine when a test ends, the
    Clock included."""

    def __init__(self, dut):
        self.dut = dut
        self._clk = cocotb.start_soon(Clock(dut.clk_i, CLK_NS, units="ns").start())

    async def reset(self):
        d = self.dut
        d.rst_i.value = 1
        d.start_flow_control_i.value = 0
        d.fc1_values_stored_i.value = 0
        d.fc2_values_stored_i.value = 0
        d.first_tlp_valid_i.value = 0
        d.idle_valid_i.value = 0
        d.update_fc_i.value = 0
        d.first_feature_exchange_dllp_received_i.value = 0
        d.m_axis_tready.value = 1
        await ClockCycles(d.clk_i, 5)
        d.rst_i.value = 0
        await ClockCycles(d.clk_i, 2)


async def run_fc_init(dut, cycles, idle_valid=0, update_fc=0):
    """Drive FC init with the peer's InitFC1/InitFC2 both received.

    Returns (reached_complete, fc2_sent_cycles, states_seen).
    """
    d = dut
    d.start_flow_control_i.value = 1
    d.fc1_values_stored_i.value = 1
    d.fc2_values_stored_i.value = 1      # <- the spec's "InitFC2 received" limb
    d.idle_valid_i.value = idle_valid
    d.update_fc_i.value = update_fc

    states_seen = set()
    fc2_sent_cycles = 0
    for _ in range(cycles):
        await RisingEdge(d.clk_i)
        await ReadOnly()
        states_seen.add(int(d.curr_state.value))
        if d.fc2_values_sent_o.value == 1:
            fc2_sent_cycles += 1
    await RisingEdge(d.clk_i)
    return (ST_FC_COMPLETE in states_seen or ST_UPDATE_P in states_seen,
            fc2_sent_cycles, states_seen)


# =============================================================================
# THE CONTROL -- green today. Proves the apparatus drives this FSM.
# =============================================================================
@cocotb.test()
async def fcinit_control_idle_fallback_completes(dut):
    """With the NON-SPEC idle fallback driven (idle_valid_i held high, exactly as
    tb_pcie_rc_ep does), the FSM leaves FC_INIT2 and asserts fc2_values_sent_o.

    GREEN TODAY. If this row ever goes red, the spec row below is not evidence
    about anything -- read this one first."""
    tb = TB(dut)
    await tb.reset()
    left, sent, states = await run_fc_init(dut, 6000, idle_valid=1)
    assert left, (
        f"control: with the idle fallback the FSM should leave FC_INIT2; "
        f"states seen = {sorted(states)}"
    )
    assert sent > 0, "control: fc2_values_sent_o never asserted"


# =============================================================================
# THE SPEC ROW -- RED TODAY. Base 2.1 §3.3.1 and nothing else.
# =============================================================================
@cocotb.test()
async def fcinit_spec_3_3_1_exit_needs_only_initfc2_received(dut):
    """§3.3.1: FC2 set sent ∧ InitFC2 received is SUFFICIENT to leave FC_INIT2.

    Both of the RTL's extra terms are held off: idle_valid_i low (so
    idle_count_r cannot reach 0x60) and update_fc_i low (so update_fc_r cannot
    set). fc2_values_stored_i is high -- the peer's InitFC2 arrived.

    ⚠️ RED ON CURRENT RTL. :401 makes the extra term a CONJUNCT, so the FSM
    loops ST_FC2..CHECK_FC2 forever and never asserts fc2_values_sent_o.
    Candidate conformance defect #6."""
    tb = TB(dut)
    await tb.reset()
    left, sent, states = await run_fc_init(dut, 6000, idle_valid=0, update_fc=0)

    assert left, (
        f"§3.3.1 violation: FC2 set sent and InitFC2 received (fc2_values_stored_i "
        f"high) is sufficient to leave FC_INIT2, but the FSM never reached "
        f"ST_UPDATE_P or ST_FC_COMPLETE. States seen = {sorted(states)}. "
        f"pcie_flow_ctrl_init.sv:401 additionally requires "
        f"(update_fc_r || idle_count_r >= 0x60), which §3.3.1 does not."
    )
    assert sent > 0, (
        "§3.3.1 violation: fc2_values_sent_o never asserted, so "
        "fc_initialized_o = fc2_values_sent && fc2_values_stored can never rise."
    )
