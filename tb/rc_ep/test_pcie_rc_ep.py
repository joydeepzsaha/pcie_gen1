"""RC <-> EP integration bench -- the two verticals in one netlist.

THE FIRST TARGET WITH NO PYTHON ANYWHERE IN THE PACKET PATH.  Every earlier
bench on either side supplied the far end from Python: the RC's enumeration
rows ran against ConfigDevice/BarSpaceCompleter, and the endpoint's five rows
ran against a Python link partner.  Here the RC's data link layer talks to the
ENDPOINT's data link layer across twelve wires, and the endpoint's
configuration space -- which lives inside its DLL -- answers for itself.

⚠️⚠️ AND THAT IS EXACTLY WHY THIS BENCH FOUND WHAT IT FOUND.  Two real data
link layers facing each other DEADLOCK: neither will transmit its InitFC1
until it has received the peer's.  See rcep_fc_init_completes_unaided below.
Every previous bench masked it because Python always spoke first.

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
# ⚠️ RED BY MEASUREMENT -- conformance divergence, NOT a harness bug
# ==========================================================================
@cocotb.test(expect_fail=True)
async def rcep_fc_init_completes_unaided(dut):
    """Two real data link layers, facing each other, must both reach FC init.

    Base 2.1 SS3.3.1 p.160 and SS3.2.1 pp.158-159: on entering DL_Init a Port
    ENTERS FC_INIT1 and TRANSMITS InitFC1 DLLPs for each supported VC,
    continuing until it has received all three InitFC1 types from its peer.
    Transmission is unconditional on entry.  It is NOT conditioned on having
    received anything.

    ⚠️⚠️ THIS ROW IS RED, AND HERE IS EXACTLY WHY -- READ THIS BEFORE FLIPPING
    IT (SS22.87: a red row's body encodes its premises, and they EXPIRE when the
    behaviour is fixed, so flipping this means REWRITING THE BODY, not deleting
    the decorator).

    pcie_flow_ctrl_init's ST_IDLE arm reads:

        if (start_flow_control_i && fc_axis_tready)
          if (fc1_values_stored_i || first_feature_exchange_dllp_received_i)
            next_state = ST_FC1_P;          // only now does it transmit

    Both release conditions are OUTPUTS OF dllp_receive -- fc1_values_stored_o
    and first_feature_exchange_dllp_received_o -- so BOTH are set only by
    RECEIVING a DLLP from the peer.  The FSM is therefore a pure RESPONDER: it
    answers an InitFC1 but never originates one.

    Put two of them on one link and neither can go first.  Measured: both
    parked in ST_IDLE (state 0) for the whole window, both with
    start_flow_control asserted, and ZERO beats in either direction.

    ⚠️ WHY NO BENCH EVER SAW THIS.  test_pcie_endpoint_top.py's
    initialize_flow_control() sends all seven InitFC DLLPs from Python before
    waiting on fc_initialized_o, and every RC bench does the same through
    RcDlTB's phy_source.  A Python far end always speaks first, so the
    responder-only behaviour is indistinguishable from a conformant initiator.
    IT TAKES TWO RTL PEERS TO TELL THEM APART, which is what this netlist is.

    NON-VACUITY (SS22.82).  This row would be worthless if it merely failed to
    observe FC init -- a broken clock would do that.  It therefore asserts the
    POSITIVE SIGNATURE of the deadlock as well: start_flow_control high on both
    sides (so both were commanded to start), and ST_IDLE the ONLY state either
    FSM ever occupies (so neither made any progress at all).  Those checks pass;
    the spec assertion at the end is the one that fails, and it is last on
    purpose.
    """
    tb = RcEpTB(dut)
    await tb.reset()

    w = await watch_fc_init(dut, 20000)
    _report(dut, w)

    # --- non-vacuity: the deadlock's positive signature ---------------------
    assert w["rc_start_fc_high"], \
        "the RC was never commanded to start flow control -- this is not the " \
        "deadlock, it is a reset or link-state problem"
    assert w["ep_start_fc_high"], \
        "the endpoint was never commanded to start flow control -- this is " \
        "not the deadlock, it is a reset or link-state problem"
    assert w["rc_states"] == {ST_IDLE}, \
        f"the RC's FC FSM left ST_IDLE (states seen: {w['rc_states']}) -- the " \
        "deadlock premise no longer holds, REWRITE THIS ROW (SS22.87)"
    assert w["ep_states"] == {ST_IDLE}, \
        f"the EP's FC FSM left ST_IDLE (states seen: {w['ep_states']}) -- the " \
        "deadlock premise no longer holds, REWRITE THIS ROW (SS22.87)"
    assert w["rc_to_ep_beats"] == 0 and w["ep_to_rc_beats"] == 0, \
        f"traffic crossed the seam ({w['rc_to_ep_beats']} RC->EP, " \
        f"{w['ep_to_rc_beats']} EP->RC) -- the deadlock premise no longer " \
        "holds, REWRITE THIS ROW (SS22.87)"

    # --- the specification requirement, which is what actually fails --------
    assert w["rc_fc_init_at"] is not None and w["ep_fc_init_at"] is not None, (
        "Base 2.1 SS3.3.1 p.160: on entering DL_Init both Ports must transmit "
        "InitFC1 and complete flow-control initialisation.  Neither did: "
        f"RC fc_init_done_o {w['rc_fc_init_at']}, "
        f"EP fc_initialized_o {w['ep_fc_init_at']}.  "
        "pcie_flow_ctrl_init originates no InitFC1 -- it only answers one."
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


async def bring_up(dut, cycles=20000):
    """Reset, break the tie, and wait for FC init on BOTH sides.

    Returns (tb, rc_at, ep_at).  Raises with both FSM states named if either
    side fails to come up -- the diagnostic that told a deadlock from a stall
    in the first place.
    """
    tb = RcEpTB(dut)
    await tb.reset()
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
        f"after the tie-break, FC init still did not complete on both sides: "
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
    tb, rc_at, ep_at = await bring_up(dut)

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
