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
