"""RC <-> EP integration bench -- the two verticals in one netlist.

THE FIRST TARGET WITH NO PYTHON ANYWHERE IN THE PACKET PATH.  Every earlier
bench on either side supplied the far end from Python: the RC's enumeration
rows ran against ConfigDevice/BarSpaceCompleter, and the endpoint's five rows
ran against a Python link partner.  Here the RC's data link layer talks to the
ENDPOINT's data link layer across twelve wires, and the endpoint's
configuration space -- which lives inside its DLL -- answers for itself.

⚠️⚠️ AND THAT IS EXACTLY WHY THIS BENCH FOUND WHAT IT FOUND.  Two real data
link layers facing each other USED TO DEADLOCK: neither would transmit its
InitFC1 until it had received the peer's.  Every previous bench masked it
because Python always spoke first.  That was conformance defect #4, and it is
FIXED -- pcie_flow_ctrl_init now originates the InitFC1 triple on entering
DL_Init, per Base 2.1 SS3.3.1 p.161.

⚠️ THE HISTORY IS KEPT DELIBERATELY, BUT IT IS HISTORY.  Do not read the
paragraph above as a live description of the RTL.  What this bench asserts now
is the opposite claim: rcep_fc_init_completes_unaided requires both sides to
reach FC init with the injector never asserted, and bring_up() no longer primes
by default, so every enumeration row travels the real RTL-to-RTL bring-up path
and would go red if the originate path were reverted.

⚠️ WHY RcDlTB IS NOT INHERITED, THOUGH EVERY OTHER RC BENCH INHERITS IT.
RcDlTB.__init__ attaches an AxiStreamSource to s_phy_axis and an
AxiStreamSink to m_phy_axis.  In this netlist BOTH of those streams are driven
by RTL -- s_phy_axis by the endpoint's m_phy_axis_*, m_phy_axis by the RC --
so attaching a source would put a second driver on a net the endpoint already
drives and resolve it to X.  The far end is the point of this bench; a harness
that models it would defeat it.  Only the PURE-PYTHON helpers are imported.

⚠️ MIN_CREDIT_EP IS NOT USED, AND CANNOT BE.  It is a dictionary of InitFC
DLLP field values that only takes effect when the bench plays the far end.
Here both sides advertise pcie_datalink_pkg's HdrMinCredits (16) and
PdMinCredits (64) from their own RTL, and no port or parameter of
pcie_endpoint_top can change that.  The control axis for these rows is
therefore ROUTE, not credit.  Recorded in
~/pcie_docs/evidence/rc-ep-bench/DESIGN.md SS1.

Spec cited (read, not assumed):
  FC init completes once per link-up ... PCIe Base 2.1 SS3.3.1 p.160
  FC init is not re-entered while up .. PCIe Base 2.1 SS3.2.1 pp.158-159
  Vendor ID / Device ID ............... PCIe Base 2.1 SS7.5.1.1 p.484
  Header Type ......................... PCIe Base 2.1 SS7.5.1.9
  BARs define the claimed range ....... PCIe Base 2.1 SS7.5.2.1 p.488
RTL cited (by signal, never by line -- CL-1 policy):
  the FC-init FSM that will not start . src/dllp/pcie_flow_ctrl_init.sv,
                                        ST_IDLE / fc1_values_stored_i /
                                        first_feature_exchange_dllp_received_i
  both release conditions are RX-only . src/dllp/dllp_receive.sv,
                                        fc1_values_stored_o and
                                        first_feature_exchange_dllp_received_o
  the endpoint's config space ......... src/dllp/dllp_receive.sv,
                                        pcie_cfg_wrapper_inst
  the completion path back to the wire  src/dllp/pcie_datalink_layer.sv,
                                        cpl_from_cfg_*
  the FC-init glitch source ........... src/dllp/pcie_flow_ctrl_init.sv,
                                        fc2_values_sent_o
  the RC's filter, which the EP lacks . src/rc/pcie_rc_dl_top.sv,
                                        fc_init_sticky_r
  advertised credits, both sides ...... src/packages/pcie_datalink_pkg.sv,
                                        HdrMinCredits / PdMinCredits
"""

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ReadOnly, RisingEdge
from cocotb.utils import get_sim_time

from cocotbext.axi import AxiStreamBus, AxiStreamSource
from cocotbext.pcie.core.dllp import DllpType

# Reused for their PURE DATA only: the DLLP builders and the tuser encoding.
# Nothing from these modules drives the seam -- see the module docstring.
from test_pcie_endpoint_top import (
    PHY_USER_IS_DLLP,
    build_fc_dllp,
    send_axis,
)

CLK_NS = 8  # 125 MHz, matching every other RC/EP bench (test_pcie_rc_dl_top.py)

# A Root Complex's Requester ID is its own BDF, fixed at 00:00.0.
RID = 0x0000
SCAN_BUS = 0

# pcie_flow_ctrl_init's flow_control_state_e, first enumerator.  The wrapper
# casts curr_state to a 5-bit vector because cocotb cannot read an enum.
ST_IDLE = 0


class RcEpTB:
    """Clock, reset, and the two DUT surfaces.  No far-end model, by design."""

    def __init__(self, dut):
        self.dut = dut
        cocotb.start_soon(Clock(dut.clk_i, CLK_NS, units="ns").start())
        # Attached to the INJECTOR's own signals, which nothing else drives.
        # It is not a far end; see tie_break() and the wrapper's header.
        self.inj = AxiStreamSource(
            AxiStreamBus.from_prefix(dut, "inj"), dut.clk_i, dut.rst_i
        )

    async def reset(self, link_up=1, scan_bus=SCAN_BUS, bar_enable=1):
        d = self.dut
        d.rst_i.value = 1
        d.phy_link_up_i.value = 0
        d.idle_valid_i.value = 0
        d.transmit_enable_i.value = 0

        # ---- RC identity and negotiated limits ---------------------------
        d.requester_id_i.value = RID
        d.completer_id_i.value = 0x0000
        d.bus_number_i.value = 0
        d.device_number_i.value = 0
        d.function_number_i.value = 0
        d.memory_enable_i.value = 1
        d.extended_tag_enable_i.value = 0
        d.max_payload_bytes_i.value = 128
        d.max_read_bytes_i.value = 128
        d.rcb_128b_i.value = 0

        # ---- RC enumeration control --------------------------------------
        d.inj_sel.value = 0
        d.scan_start_i.value = 0
        d.scan_bus_i.value = scan_bus
        d.bar_enable_i.value = bar_enable
        d.bridge_enable_i.value = 0

        # ---- RC completer surface: accept everything, answer nothing yet --
        d.m_axis_cq_tready.value = 1
        d.s_axis_cc_tdata.value = 0
        d.s_axis_cc_tkeep.value = 0
        d.s_axis_cc_tvalid.value = 0
        d.s_axis_cc_tlast.value = 0
        d.s_axis_cc_tuser.value = 0

        # ---- EP surface ---------------------------------------------------
        d.ep_memory_enable_i.value = 1
        d.ep_extended_tag_enable_i.value = 0
        d.ep_max_payload_bytes_i.value = 128
        d.ep_max_read_bytes_i.value = 128
        d.ep_rcb_128b_i.value = 0

        d.ep_command_valid_i.value = 0
        d.ep_command_i.value = 0
        d.ep_command_address_i.value = 0
        d.ep_command_byte_count_i.value = 0
        d.ep_command_tc_i.value = 0
        d.ep_command_attr_i.value = 0
        d.ep_command_context_i.value = 0
        d.ep_command_prefix_valid_i.value = 0
        d.ep_command_prefix_i.value = 0
        d.ep_command_ecrc_enable_i.value = 0
        d.ep_command_data_i.value = 0
        d.ep_command_keep_i.value = 0
        d.ep_command_data_valid_i.value = 0
        d.ep_command_data_last_i.value = 0

        d.ep_target_request_ready_i.value = 1
        d.ep_target_data_ready_i.value = 1
        d.ep_completion_request_valid_i.value = 0
        d.ep_completion_request_header_i.value = 0
        d.ep_completion_request_status_i.value = 0
        d.ep_completion_request_byte_count_i.value = 0
        d.ep_completion_request_lower_address_i.value = 0
        d.ep_completion_request_ecrc_enable_i.value = 0
        d.ep_completion_request_data_i.value = 0
        d.ep_completion_request_keep_i.value = 0
        d.ep_completion_request_data_valid_i.value = 0
        d.ep_completion_request_data_last_i.value = 0
        d.ep_received_completion_ready_i.value = 1
        d.ep_received_completion_data_ready_i.value = 1
        d.ep_result_ready_i.value = 1

        for _ in range(8):
            await RisingEdge(d.clk_i)
        d.rst_i.value = 0
        d.phy_link_up_i.value = link_up
        d.idle_valid_i.value = link_up
        d.transmit_enable_i.value = 1
        for _ in range(8):
            await RisingEdge(d.clk_i)


def _i(sig):
    """Integer value of a signal, or None if unresolvable."""
    v = sig.value
    return int(v) if v.is_resolvable else None


async def watch_fc_init(dut, cycles):
    """Run for `cycles` and report everything needed to tell WHY, not just that.

    Returns a dict.  The three fields that distinguish a deadlock from a stall
    from a reset problem are the two FSM states and the beat counts: a deadlock
    parks both FSMs in ST_IDLE with zero beats, a stalled ready parks one side
    mid-sequence, and a reset problem shows start_flow_control low.
    """
    out = {
        "rc_fc_init_at": None, "ep_fc_init_at": None,
        "rc_to_ep_beats": 0, "ep_to_rc_beats": 0,
        "rc_states": set(), "ep_states": set(),
        "rc_start_fc_high": False, "ep_start_fc_high": False,
        # inj_sel sampled EVERY cycle, not spot-checked at the end: the
        # "unaided" claim is that the injector was never asserted at any point
        # in the window, and a point sample cannot say that.
        "inj_ever_high": False,
        "cycles": cycles,
    }
    for _ in range(cycles):
        await RisingEdge(dut.clk_i)
        await ReadOnly()
        if out["rc_fc_init_at"] is None and _i(dut.fc_init_done_o):
            out["rc_fc_init_at"] = get_sim_time("ns")
        if out["ep_fc_init_at"] is None and _i(dut.ep_fc_initialized_o):
            out["ep_fc_init_at"] = get_sim_time("ns")
        if _i(dut.m_phy_axis_tvalid) and _i(dut.m_phy_axis_tready):
            out["rc_to_ep_beats"] += 1
        if _i(dut.s_phy_axis_tvalid) and _i(dut.s_phy_axis_tready):
            out["ep_to_rc_beats"] += 1
        out["rc_states"].add(_i(dut.rc_fc_state))
        out["ep_states"].add(_i(dut.ep_fc_state))
        if _i(dut.rc_start_fc):
            out["rc_start_fc_high"] = True
        if _i(dut.ep_start_fc):
            out["ep_start_fc_high"] = True
        if _i(dut.inj_sel):
            out["inj_ever_high"] = True
    return out


def _report(dut, w):
    dut._log.info(
        "FC init after %d cycles: RC at %s, EP at %s | beats RC->EP %d, "
        "EP->RC %d | RC fc states seen %s, EP fc states seen %s | "
        "start_flow_control RC %s EP %s",
        w["cycles"], w["rc_fc_init_at"], w["ep_fc_init_at"],
        w["rc_to_ep_beats"], w["ep_to_rc_beats"],
        sorted(s for s in w["rc_states"] if s is not None),
        sorted(s for s in w["ep_states"] if s is not None),
        w["rc_start_fc_high"], w["ep_start_fc_high"],
    )


# ==========================================================================
# GREEN, AND IT IS THE WITNESS FOR CONFORMANCE DEFECT #4'S FIX
# ==========================================================================
# SS3.3.1 originate interval, restated here so the row's arithmetic is
# auditable without opening the RTL: pcie_flow_ctrl_init's FcInitWaitPeriod is
# 4250 cycles, which at the wrapper's 8 ns clock is 34 us -- the bound Base 2.1
# SS3.3.1 p.161 sets ("must be transmitted at least once every 34 us").
FC_ORIGINATE_NS = 4250 * 8            # 34_000 ns, one originate interval
FC_ORIGINATE_WINDOW_NS = 2 * FC_ORIGINATE_NS   # 68_000 ns, two intervals


@cocotb.test()
async def rcep_fc_init_completes_unaided(dut):
    """Two real data link layers, facing each other, both reach FC init unaided.

    Base 2.1 SS3.3.1 p.161: on entering DL_Init a Port enters FC_INIT1 and
    TRANSMITS the InitFC1 triple -- P first, NP second, Cpl third -- "at least
    once every 34 us".  Receiving governs only the EXIT ("Set Flag FI1" / "Exit
    to FC_INIT2 if Flag FI1 has been set"), never the entry.  Figure 3-3 p.163
    draws one side entering the sequence before the other has said anything.

    ⚠️⚠️ THIS ROW WAS RED AND ITS BODY HAS BEEN REWRITTEN, NOT ITS DECORATOR
    DELETED (SS22.87).  Read this before touching it.

    WHAT IT USED TO ASSERT, AND WHY THOSE PREMISES ARE NOW DEAD.  Until the
    originate fix, pcie_flow_ctrl_init left ST_IDLE only on fc1_values_stored_i
    or first_feature_exchange_dllp_received_i -- both outputs of dllp_receive,
    both set only by RECEIVING.  The FSM answered an InitFC1 and never
    originated one, so two of them on one link deadlocked.  This row pinned
    that failure mode with four positive-signature assertions: ST_IDLE the ONLY
    state either FSM occupied, and ZERO beats in either direction.  ALL FOUR
    ARE NOW FALSE, and they are false because the defect is fixed -- which is
    exactly the trap SS22.87 exists for.  Deleting expect_fail without rewriting
    the body would have failed this row on its own dead premises and read as a
    regression in the fix.

    WHAT IT ASSERTS NOW.  The same specification requirement, from the other
    side: both Ports DO complete flow-control initialisation, with nothing
    external ever driving either receive stream.

    NON-VACUITY (SS22.82), and this row needs it more than most, because "both
    sides came up" is exactly what a bench that quietly primed the link would
    also show.  Four independent checks make that reading impossible:

      1. inj_sel LOW for every cycle of the window -- sampled each cycle, not
         spot-checked.  The injector is the ONLY path by which anything outside
         the two DLLs can reach a receive stream, so this is what makes the
         claim "unaided" rather than "initialised somehow".
      2. start_flow_control high on BOTH sides -- both were commanded to start,
         so a pass cannot come from one side never having been asked.
      3. Beats crossed in BOTH directions -- real DLLPs on the wire, not two
         FSMs declaring victory independently.
      4. ⭐ FC init lands in [34 us, 68 us).  THIS IS THE ONE THAT IDENTIFIES
         THE MECHANISM.  The lower bound is one full originate interval, so the
         row cannot pass if something primed the link early -- a primed link
         completes in well under 1 us, as every Python-driven bench in this
         repo does.  The upper bound is two intervals, so this is the FIRST
         originate and not a later repeat.  Together they say the timer did it.
    """
    tb = RcEpTB(dut)
    await tb.reset()

    w = await watch_fc_init(dut, 20000)
    _report(dut, w)

    # --- non-vacuity: nothing outside the two DLLs touched the link ---------
    assert not w["inj_ever_high"], \
        "inj_sel went high during the window -- the injector primed the link, " \
        "so this row measures nothing about unaided bring-up"
    assert w["rc_start_fc_high"], \
        "the RC was never commanded to start flow control -- this is not a " \
        "bring-up result, it is a reset or link-state problem"
    assert w["ep_start_fc_high"], \
        "the endpoint was never commanded to start flow control -- this is " \
        "not a bring-up result, it is a reset or link-state problem"
    assert w["rc_to_ep_beats"] > 0 and w["ep_to_rc_beats"] > 0, (
        "flow control reported complete but the seam carried no traffic in "
        f"both directions ({w['rc_to_ep_beats']} RC->EP, "
        f"{w['ep_to_rc_beats']} EP->RC) -- DLLPs must actually have crossed"
    )
    assert w["rc_states"] > {ST_IDLE} and w["ep_states"] > {ST_IDLE}, (
        f"an FC FSM never left ST_IDLE (RC {sorted(w['rc_states'])}, "
        f"EP {sorted(w['ep_states'])}) -- neither side originated anything"
    )

    # --- the specification requirement, which is what this row is for -------
    assert w["rc_fc_init_at"] is not None and w["ep_fc_init_at"] is not None, (
        "Base 2.1 SS3.3.1 p.161: on entering DL_Init both Ports must transmit "
        "InitFC1 and complete flow-control initialisation.  One did not: "
        f"RC fc_init_done_o {w['rc_fc_init_at']}, "
        f"EP fc_initialized_o {w['ep_fc_init_at']}"
    )

    # --- and it was the ORIGINATE TIMER that did it, not an early prime -----
    for who, at in (("RC", w["rc_fc_init_at"]), ("EP", w["ep_fc_init_at"])):
        assert FC_ORIGINATE_NS <= at < FC_ORIGINATE_WINDOW_NS, (
            f"{who} completed FC init at {at} ns, outside the first originate "
            f"interval [{FC_ORIGINATE_NS}, {FC_ORIGINATE_WINDOW_NS}) ns.  "
            "Below the lower bound something primed the link and this row is "
            "not measuring unaided bring-up; at or above the upper bound the "
            "first InitFC1 triple was missed and a later repeat carried it, "
            "which is a different behaviour from the one asserted here."
        )


# The seven-DLLP InitFC sequence, at whatever credits the builder defaults to.
# The VALUES do not matter here: this DLLP exists to move the endpoint's FSM
# out of ST_IDLE, and the credits that govern the link afterwards are the ones
# BOTH SIDES advertise from pcie_datalink_pkg.
_TIE_BREAK_SEQUENCE = (
    DllpType.INIT_FC1_P,
    DllpType.INIT_FC1_NP,
    DllpType.INIT_FC1_CPL,
    DllpType.INIT_FC2_P,
    DllpType.INIT_FC2_NP,
    DllpType.INIT_FC2_CPL,
)


async def tie_break(tb):
    """Break the InitFC deadlock, then get out of the way.

    ⚠️ THIS IS A WORKAROUND FOR A DEFECT, NOT A MODEL OF A LINK PARTNER, and
    the distinction is the whole reason this bench is worth anything.
    rcep_fc_init_completes_unaided documents why it is needed: neither DLL
    originates an InitFC1, so on a two-peer link neither ever starts.

    The injector speaks into the ENDPOINT's receive stream only, only while
    inj_sel is high, and only before flow control exists.  Once the endpoint's
    FSM leaves ST_IDLE it transmits its own InitFC1/InitFC2 at the Root
    Complex, which releases the RC's FSM in turn -- so the RC is brought up BY
    THE ENDPOINT, in RTL, not by this function.

    inj_sel is dropped before returning.  Every row that calls this then
    asserts inj_sel stayed low for its whole measurement window, so nothing
    downstream of here can be attributed to the injector.
    """
    d = tb.dut
    d.inj_sel.value = 1
    await RisingEdge(d.clk_i)
    for dllp_type in _TIE_BREAK_SEQUENCE:
        await send_axis(tb.inj, build_fc_dllp(dllp_type), PHY_USER_IS_DLLP)
        for _ in range(24):
            await RisingEdge(d.clk_i)
    d.inj_sel.value = 0
    await RisingEdge(d.clk_i)


async def bring_up(dut, cycles=20000, use_injector=False):
    """Reset and wait for FC init on BOTH sides.  By default, UNAIDED.

    Returns (tb, rc_at, ep_at).  Raises with both FSM states named if either
    side fails to come up -- the diagnostic that told a deadlock from a stall
    in the first place.

    ⚠️ THE DEFAULT CHANGED WHEN CONFORMANCE DEFECT #4 WAS FIXED, AND THAT IS
    THE POINT.  This used to call tie_break() unconditionally, because neither
    DLL would originate an InitFC1 and the link could not come up without an
    injected one.  tie_break's own docstring says it is "A WORKAROUND FOR A
    DEFECT, NOT A MODEL OF A LINK PARTNER".  The defect is fixed, so the
    workaround is off by default and the callers now exercise the real
    RTL-to-RTL bring-up path.

    ⚠️ AND THAT IS WHAT KEEPS THE CALLERS' inj_sel ASSERTIONS HONEST.  Leaving
    the injector on would not have BROKEN those assertions -- it would have made
    them worse than broken, it would have made them VACUOUS: "the injector was
    not used during the measurement window" asserts nothing once the link cannot
    come up without the injector having been used just before it.  Worse, a
    primed link masks the fix entirely -- every one of these rows would still
    pass with the originate path reverted, so they could not witness a
    regression of defect #4.  Unaided, they can.

    use_injector=True is kept for rcep_tie_break_brings_both_sides_up, which is
    the one row whose SUBJECT is the injector path.  Keeping exactly one caller
    on it is what stops tie_break and its inj_sel checks from going dead.
    """
    tb = RcEpTB(dut)
    await tb.reset()
    if use_injector:
        await tie_break(tb)

    rc_at = None
    ep_at = None
    tb.inj_high_during_wait = False
    for _ in range(cycles):
        await RisingEdge(dut.clk_i)
        await ReadOnly()
        if _i(dut.inj_sel):
            tb.inj_high_during_wait = True
        if rc_at is None and _i(dut.fc_init_done_o):
            rc_at = get_sim_time("ns")
        if ep_at is None and _i(dut.ep_fc_initialized_o):
            ep_at = get_sim_time("ns")
        if rc_at is not None and ep_at is not None:
            dut._log.info("bring-up: RC FC init %s ns, EP FC init %s ns",
                          rc_at, ep_at)
            # Leave the ReadOnly phase before returning: the caller writes
            # scan_start_i, and cocotb forbids a write scheduled in ReadOnly.
            await RisingEdge(dut.clk_i)
            return tb, rc_at, ep_at
    raise AssertionError(
        f"FC init did not complete on both sides: "
        f"RC {rc_at}, EP {ep_at}; RC fc state {_i(dut.rc_fc_state)}, "
        f"EP fc state {_i(dut.ep_fc_state)}"
    )


@cocotb.test()
async def rcep_tie_break_brings_both_sides_up(dut):
    """After ONE injected InitFC sequence, both peers reach FC init in RTL.

    The positive half of the deadlock row.  It proves three things the red row
    cannot: that the seam carries traffic in both directions, that the endpoint
    -- once started -- does transmit InitFC of its own, and that the Root
    Complex is brought up BY THE ENDPOINT rather than by the bench.

    NON-VACUITY (SS22.82).  The claim is not "FC init happened" but "FC init
    happened WITHOUT the injector".  So the row asserts inj_sel is low for the
    whole window after the tie-break, and that beats crossed EP->RC in that
    window -- which can only be endpoint-originated traffic, because the
    injector drives the endpoint's RECEIVE stream and can put nothing on its
    transmit stream.
    """
    tb, rc_at, ep_at = await bring_up(dut, use_injector=True)

    ep_to_rc = 0
    rc_to_ep = 0
    inj_ever_high = False
    for _ in range(400):
        await RisingEdge(dut.clk_i)
        await ReadOnly()
        if _i(dut.inj_sel):
            inj_ever_high = True
        if _i(dut.s_phy_axis_tvalid) and _i(dut.s_phy_axis_tready):
            ep_to_rc += 1
        if _i(dut.m_phy_axis_tvalid) and _i(dut.m_phy_axis_tready):
            rc_to_ep += 1

    dut._log.info("post-tie-break window: EP->RC %d beats, RC->EP %d beats, "
                  "inj_sel ever high: %s", ep_to_rc, rc_to_ep, inj_ever_high)

    assert not inj_ever_high, \
        "the injector was still enabled during the measurement window"
    assert not tb.inj_high_during_wait, (
        "the injector was still enabled while FC init was being observed -- "
        "this row would then prove nothing about RTL-to-RTL bring-up"
    )
    assert rc_at is not None, "the RC never reached FC init"
    assert ep_at is not None, "the endpoint never reached FC init"


async def run_enumeration(dut, tb, cycles=60000):
    """Pulse scan_start_i and wait for enum_done_o or an error.

    scan_start_i is a PULSE on purpose: pcie_enum_dl_top's start gate has to
    REMEMBER a request made while flow control is still down, and a pulse is
    the stimulus that distinguishes a latch from a bare AND (tracker SS44).
    """
    d = dut
    d.scan_start_i.value = 1
    await RisingEdge(d.clk_i)
    d.scan_start_i.value = 0

    frames = 0
    inj_high = False
    for _ in range(cycles):
        await RisingEdge(d.clk_i)
        await ReadOnly()
        if _i(d.inj_sel):
            inj_high = True
        if _i(d.m_phy_axis_tvalid) and _i(d.m_phy_axis_tready) \
                and _i(d.m_phy_axis_tlast):
            frames += 1
        if _i(d.enum_done_o) or _i(d.enum_error_o) or _i(d.scan_error_o):
            break
    tb.enum_frames = frames
    tb.enum_inj_high = inj_high
    await ReadOnly()
    return {
        "enum_done": _i(d.enum_done_o),
        "enum_error": _i(d.enum_error_o),
        "enum_error_code": _i(d.enum_error_code_o),
        "scan_done": _i(d.scan_done_o),
        "scan_error": _i(d.scan_error_o),
        "scan_error_code": _i(d.scan_error_code_o),
        "device_present": _i(d.device_present_o),
        "unsupported": _i(d.unsupported_device_o),
        "device_bdf": _i(d.device_bdf_o),
        "vendor_id": _i(d.vendor_id_o),
        "device_id": _i(d.device_id_o),
        "header_type": _i(d.header_type_o),
        "multifunction": _i(d.multifunction_o),
        "bar_count": _i(d.bar_count_o),
        "bar_valid": _i(d.bar_valid_o),
        "bar_size": _i(d.bar_size_o),
        "io_bar_mask": _i(d.io_bar_mask_o),
        "frames": frames,
        # The RC's error surface, so a timeout row can say WHY rather than
        # only that the completion never arrived.
        "cpl_timeout_valid": _i(d.cpl_timeout_valid_o),
        "cpl_timeout_tag": _i(d.cpl_timeout_tag_o),
        "rc_unexpected_cpl": _i(d.rc_unexpected_completion_o),
        "rc_protocol_error": _i(d.rc_protocol_error_o),
        "rc_error_code": _i(d.rc_error_code_o),
        "malformed": _i(d.malformed_o),
        "rx_error_valid": _i(d.rx_error_valid_o),
        "rx_error_code": _i(d.rx_error_code_o),
        "ep_malformed": _i(d.ep_malformed_o),
        "ep_rx_error_valid": _i(d.ep_rx_error_valid_o),
        "ep_tx_error_valid": _i(d.ep_tx_error_valid_o),
    }


def _log_enum(dut, r):
    dut._log.info(
        "enumeration: done=%s error=%s(code %s) scan_done=%s scan_error=%s"
        "(code %s) present=%s unsupported=%s bdf=%s VID=%s DID=%s "
        "hdr_type=%s multifn=%s bar_count=%s bar_valid=%s io_mask=%s "
        "frames=%s",
        r["enum_done"], r["enum_error"], r["enum_error_code"], r["scan_done"],
        r["scan_error"], r["scan_error_code"], r["device_present"],
        r["unsupported"],
        None if r["device_bdf"] is None else hex(r["device_bdf"]),
        None if r["vendor_id"] is None else hex(r["vendor_id"]),
        None if r["device_id"] is None else hex(r["device_id"]),
        None if r["header_type"] is None else hex(r["header_type"]),
        r["multifunction"], r["bar_count"],
        None if r["bar_valid"] is None else hex(r["bar_valid"]),
        None if r["io_bar_mask"] is None else hex(r["io_bar_mask"]),
        r["frames"],
    )
    dut._log.info(
        "  RC errors: cpl_timeout=%s(tag %s) unexpected_cpl=%s "
        "protocol_err=%s(code %s) malformed=%s rx_err=%s(code %s) | "
        "EP errors: malformed=%s rx_err=%s tx_err=%s",
        r["cpl_timeout_valid"], r["cpl_timeout_tag"], r["rc_unexpected_cpl"],
        r["rc_protocol_error"], r["rc_error_code"], r["malformed"],
        r["rx_error_valid"], r["rx_error_code"],
        r["ep_malformed"], r["ep_rx_error_valid"], r["ep_tx_error_valid"],
    )
    if r["bar_size"] is not None:
        for slot in range(6):
            size = (r["bar_size"] >> (64 * slot)) & ((1 << 64) - 1)
            if size:
                dut._log.info("  BAR%d size reported = 0x%x (%d bytes)",
                              slot, size, size)


@cocotb.test()
async def rcep_enumeration_reads_real_ep_config(dut):
    """Our enumeration engine reads the ENDPOINT's real configuration space.

    The rung's headline positive.  Every value asserted here comes from
    src/pcie_cfg/pcie_config_reg.sv, never from a Python model:
      Vendor ID  0x1234   (Base 2.1 SS7.5.1.1 p.484)
      Device ID  0x00FF
      Header Type 0x00, single function (SS7.5.1.9)

    ⚠️ THE CONFIGURATION SPACE IS INSIDE THE ENDPOINT'S DATA LINK LAYER, not
    its Transaction Layer: dllp_receive instantiates pcie_cfg_wrapper, which
    answers the request and emits the Completion on its own cpl_axis_* port,
    which pcie_datalink_layer muxes back onto the transmit path as
    cpl_from_cfg_*.  So this round trip never touches the endpoint's TL.

    NON-VACUITY (SS22.82).  Two checks, because a green enumeration that never
    put a packet on the wire would be worthless: TLP frames must have crossed
    RC->EP, and inj_sel must have been low for the whole window -- so every one
    of those frames was answered by RTL.
    """
    tb, _, _ = await bring_up(dut)
    r = await run_enumeration(dut, tb)
    _log_enum(dut, r)

    assert not tb.enum_inj_high, \
        "the injector was enabled during enumeration -- this row proves nothing"
    assert tb.enum_frames > 0, \
        "enumeration reported a result with ZERO frames on the wire"

    assert r["device_present"] == 1, \
        f"the endpoint was not detected (scan_error_code {r['scan_error_code']})"
    assert r["vendor_id"] == 0x1234, (
        f"Vendor ID {r['vendor_id']:#06x} != 0x1234, the constant in "
        "pcie_config_reg.sv's readback path"
    )
    assert r["device_id"] == 0x00FF, (
        f"Device ID {r['device_id']:#06x} != 0x00ff"
    )
    assert r["header_type"] == 0x00, (
        f"Header Type {r['header_type']:#04x} != 0x00 (Type 0, single function)"
    )
    assert r["multifunction"] == 0, "the endpoint reported multi-function"


@cocotb.test()
async def rcep_enumeration_blocked_when_link_down(dut):
    """The same start command, with the link down, must produce nothing.

    ⭐ THE CONTROL FOR THE ROW ABOVE, and it is a ROUTE difference rather than
    a value difference (SS22.80: a control must not be computed from the signal
    under test).  The brief specified a credit-profile control instead --
    MIN_CREDIT_EP at default credits -- and that is not buildable here: both
    peers advertise pcie_datalink_pkg's HdrMinCredits/PdMinCredits from RTL and
    no port can change them, so a 'default credits' control would have been a
    duplicate of the row it was meant to control.

    NON-VACUITY (SS22.82).  The row asserts the start command WAS issued (the
    engine's start latch is armed) so that a bench which simply forgot to pulse
    scan_start_i could not pass as a control.
    """
    tb = RcEpTB(dut)
    await tb.reset(link_up=0)
    await tie_break(tb)

    dut.scan_start_i.value = 1
    await RisingEdge(dut.clk_i)
    dut.scan_start_i.value = 0

    frames = 0
    for _ in range(20000):
        await RisingEdge(dut.clk_i)
        await ReadOnly()
        if _i(dut.m_phy_axis_tvalid) and _i(dut.m_phy_axis_tready) \
                and _i(dut.m_phy_axis_tlast):
            frames += 1

    await ReadOnly()
    dut._log.info("link-down control: frames=%d present=%s enum_done=%s "
                  "fc_init_done=%s", frames, _i(dut.device_present_o),
                  _i(dut.enum_done_o), _i(dut.fc_init_done_o))

    assert _i(dut.fc_init_done_o) == 0, \
        "the RC reported FC init with the link down"
    assert frames == 0, f"{frames} TLP frames crossed with the link down"
    assert _i(dut.device_present_o) == 0, \
        "a device was detected with the link down"
    assert _i(dut.enum_done_o) == 0, \
        "enumeration completed with the link down"


# The endpoint's two BAR images, both read from src/ and both cited by row 3'.
#
#   what tlp_layer CLAIMS   pcie_endpoint_top's BAR_MASK/BAR_ENABLE defaults:
#                           BAR0 = 4 KB, BAR1 DISABLED
#   what the RC IS TOLD     pcie_config_reg.sv's readback path: BAR0 and BAR1
#                           both a CONSTANT 0xFFF00000, which sizes as 1 MB
EP_CLAIMED_BAR0_BYTES = 0x1000       # from BAR_MASK 0xffff_ffff_ffff_f000
EP_CLAIMED_BAR_COUNT  = 1            # from BAR_ENABLE {1'b0, 1'b1}


@cocotb.test(expect_fail=True)
async def rcep_bar_image_matches_claimed_aperture(dut):
    """The BARs the RC reads must describe the range the endpoint answers on.

    Base 2.1 SS7.5.2.1 p.488: a Base Address Register defines the address range
    the Function responds to.  It is not advisory and it is not decorative --
    it is the ONLY thing an RC has to go on when it allocates address space.

    ⚠️⚠️ THIS ROW IS RED BY MEASUREMENT.  READ THIS BEFORE FLIPPING IT
    (SS22.87 -- the premises below EXPIRE when the endpoint is fixed, so a flip
    means rewriting the body, not deleting the decorator).

    THE ENDPOINT CARRIES TWO DISAGREEING BAR IMAGES:

      1. pcie_config_reg.sv's readback path returns a CONSTANT 0xFFF00000 for
         base_address_register_0 AND base_ddress_register_1 -- read-only, not
         writable.  A sizing probe therefore reports 1 MB on each, and the RC
         sees TWO valid 1 MB memory BARs.
      2. pcie_endpoint_top's BAR_MASK/BAR_ENABLE defaults configure tlp_layer's
         decoder for ONE enabled BAR of 4 KB.  That is the aperture the
         endpoint actually answers on.

    So the RC is told 1 MB and the endpoint answers on 4 KB -- a 256x
    overstatement -- and is told about a second BAR that is not decoded at all.
    An RC that trusted the config space would map 2 MB, place another device in
    what it believed was free space beyond the first 4 KB, and see silent drops.

    ⚠️ NEITHER VERTICAL'S OWN ROWS CAN SEE THIS, and that is structural rather
    than an oversight.  The endpoint's tests drive tlp_layer through the
    PARAMETERS; the RC's enumeration rows size BARs against a Python
    BarSpaceCompleter whose masks the bench itself writes.  The two images are
    only ever compared when a real RC enumerates a real endpoint, which is this
    netlist and nothing before it.

    ⚠️ BAR2-BAR5 are a THIRD shape, recorded but not asserted here: their
    readback is plain writable storage (field_storage.base_ddress_register_N),
    so a sizing probe reads back whatever it wrote.  They behave as scratch
    registers, not BARs.

    NON-VACUITY (SS22.82).  The row asserts the enumeration actually reached the
    BAR phase and reported a size -- a run that never sized anything would
    otherwise "fail" for the wrong reason and read as this finding.
    """
    tb, _, _ = await bring_up(dut)
    r = await run_enumeration(dut, tb)
    _log_enum(dut, r)

    assert not tb.enum_inj_high, "the injector was enabled during enumeration"
    assert r["device_present"] == 1, "no device to size"
    assert r["bar_valid"], "the BAR phase reported no valid BAR at all"

    bar0_reported = r["bar_size"] & ((1 << 64) - 1)
    n_reported = bin(r["bar_valid"]).count("1")

    dut._log.info(
        "BAR image comparison: RC is told BAR0 = %d bytes across %d BAR(s); "
        "the endpoint's decoder claims %d bytes across %d BAR(s)",
        bar0_reported, n_reported,
        EP_CLAIMED_BAR0_BYTES, EP_CLAIMED_BAR_COUNT,
    )

    assert bar0_reported == EP_CLAIMED_BAR0_BYTES, (
        f"BAR0: the configuration space reports {bar0_reported} bytes "
        f"(0x{bar0_reported:x}) but tlp_layer's decoder claims only "
        f"{EP_CLAIMED_BAR0_BYTES} bytes (0x{EP_CLAIMED_BAR0_BYTES:x}) -- "
        f"a {bar0_reported // EP_CLAIMED_BAR0_BYTES}x overstatement"
    )
    assert n_reported == EP_CLAIMED_BAR_COUNT, (
        f"the RC was told {n_reported} BARs are implemented; only "
        f"{EP_CLAIMED_BAR_COUNT} is enabled in tlp_layer's decoder"
    )


@cocotb.test(expect_fail=True)
async def rcep_enumeration_completes_without_timeout(dut):
    """A full enumeration of a live endpoint must finish without a timeout.

    ⚠️ RED BY MEASUREMENT.  The PRESENCE phase succeeds completely -- scan_done
    asserts, scan_error stays low, and the RC reads the endpoint's real Vendor
    ID, Device ID and Header Type off the wire (that is
    rcep_enumeration_reads_real_ep_config, which is green).  The BAR phase then
    sizes BAR0 and BAR1 and reports both.  Enumeration nevertheless ends with
    enum_error_o asserted and enum_error_code_o = ENUM_ERR_TIMEOUT (4'd4): a
    Completion the engine was waiting for never arrived.

    WHAT IS ESTABLISHED, AND WHAT IS NOT.  Established: the timeout is real,
    it is after the sizing reads, and tens of TLP frames crossed the seam
    before it.  NOT established: which request went unanswered, or why.  The
    endpoint's write path does construct a Completion -- pcie_config_handler
    reaches ST_SEND_CPL_TLP from ST_CFG_WR_ACK via gen_cpl -- so "writes are
    never completed" is REFUTED as a whole-path explanation and the fault is
    narrower than that.

    ⚠️ THIS ROW IS DELIBERATELY A CHARACTERISATION, NOT A DIAGNOSIS.  Naming a
    mechanism it has not proven would put a guess in the artifact where a
    measurement belongs, and the next rung would inherit the guess.  What it
    pins is the boundary: presence works, sizing works, completion of the BAR
    phase does not.  The error surface is logged so the follow-on rung starts
    from data.

    NON-VACUITY (SS22.82).  The row asserts the presence phase SUCCEEDED and
    that frames crossed, so a bench that never brought the link up could not
    produce this failure signature.
    """
    tb, _, _ = await bring_up(dut)
    r = await run_enumeration(dut, tb)
    _log_enum(dut, r)

    assert not tb.enum_inj_high, "the injector was enabled during enumeration"
    assert r["frames"] > 0, "no frames crossed the seam"
    assert r["scan_done"] == 1 and r["scan_error"] == 0, (
        "the presence phase did not succeed, so this is not the BAR-phase "
        "timeout this row characterises"
    )
    assert r["device_present"] == 1, "no device was found"

    assert r["enum_error"] == 0 and r["enum_done"] == 1, (
        f"enumeration did not complete: enum_done={r['enum_done']}, "
        f"enum_error={r['enum_error']}, code={r['enum_error_code']} "
        f"(4 = ENUM_ERR_TIMEOUT), after {r['frames']} frames on the wire"
    )
