"""test_pcie_fullstack -- the Root Complex stack against the Endpoint stack

Author: Kourosh Ghahramani
Silicon Systems Research Lab, University of Washington

Under test
    tb_pcie_fullstack: pcie_rc_top (u_rc: enumeration engine, Transaction
    Layer, Data Link Layer, LTSSM and logical PHY) and pcie_endpoint_top
    (u_ep, INTEGRATED_GEN1_PHY = 1: Transaction Layer, Data Link Layer with
    the configuration space, LTSSM, logical PHY and 8b/10b codec), joined by
    pipe_codec_bridge (u_bridge), which encodes the RC's 16 data and 2 K PIPE
    bits per beat into the EP's two 10-bit symbols and decodes the other way.
Stimulus
    One 125 MHz clock drives both stacks and the bridge. Python answers each
    MAC's PIPE sideband (receiver detect, electrical idle); every other bit of
    the datapath is RTL. Tests raise en_i, pulse scan_start_i to enumerate,
    issue Configuration Requests on the RC's s_axis_rq_* after enumeration,
    and arm the bridge's error injector (inj_*) or DLLP blackouts (starve_en,
    starve_ep_en). Everything else is read from the bench top's own signals or
    hierarchically through dut.u_rc and dut.u_ep.
A pass means
    Both LTSSMs reach L0 through the codec with no code or disparity error,
    and both Data Link Layers complete Flow Control initialization and stay
    initialized. The RC enumerates the Endpoint's configuration space, and the
    Data Link Layer rules each test names hold on the stack it observes:
    credit release, UpdateFC scheduling, Ack/Nak, replay, nullified TLPs and
    retraining. In L0 both transmitters keep PIPE TX valid high, and the RC's
    stream descrambles to packets and Logical Idle, with SKP Ordered Sets at
    the required spacing. Configuration Requests above offset FFh reach their
    own registers, and the PIPE seam is 16 data + 2 K bits per lane.
Limitations
    One lane, Gen1 only. The Endpoint originates no request. The bench top
    brings out no Completion Status, tag or completion-timeout signal of the
    RC: the enumeration tests judge those through the engine's results, and
    the extended-configuration tests read them hierarchically.
Structure
    Constants, bench driver and PIPE sideband (TB, receiver_detect)
    Bring-up monitors, bring_up, _run_and_report
    Link training and Flow Control initialization
    Enumeration across two PHYs
    TLP path at each Data Link Layer input
    CfgRd0 round-trip timeline
    Credit release and UpdateFC scheduling
    Replay
    L0 transmit: valid, Logical Idle and SKP spacing
    LCRC, sequence and nullified-TLP injection
    Periodic UpdateFC
    REPLAY_NUM rollover and Recovery; Endpoint-initiated Recovery
    Extended configuration space
    PIPE seam width
References
    PCIe Base Spec r2.1, §2.2.1
    PCIe Base Spec r2.1, §2.2.7
    PCIe Base Spec r2.1, §2.2.9
    PCIe Base Spec r2.1, §2.6.1.2
    PCIe Base Spec r2.1, §3.2.1
    PCIe Base Spec r2.1, §3.3.1
    PCIe Base Spec r2.1, §3.4
    PCIe Base Spec r2.1, §3.5.1
    PCIe Base Spec r2.1, §3.5.2.1
    PCIe Base Spec r2.1, §3.5.3.1
    PCIe Base Spec r2.1, §4.2.2
    PCIe Base Spec r2.1, §4.2.3
    PCIe Base Spec r2.1, §4.2.6.4
    PCIe Base Spec r2.1, §4.2.6.5
    PCIe Base Spec r2.1, §4.2.7.1
    PCIe Base Spec r2.1, §4.2.7.2
    PCIe Base Spec r2.1, §7.2
    PCIe Base Spec r2.1, §7.3.2
    PCIe Base Spec r2.1, §7.5.1
    PCIe Base Spec r2.1, §7.9.1
    PCIe Base Spec r2.1, Appendix C.1
    PCI Local Bus Spec r3.0, §6.1
    PCI Local Bus Spec r3.0, §6.2.5.1
    PG239, Table 4: Clock and Reset Signals
    PG239, Table 5 and Table 7
    PG213, Table 60, Table 61 and Table 65
"""

import os
import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ClockCycles, ReadOnly

CLK_NS = 8  # 125 MHz, the PIPE clock at Gen1 (PG239, Table 4: Clock and Reset Signals)

RXSTATUS_RECEIVER_DETECTED = 0b011   # the RxStatus code both MACs' detect latches match
DETECT_LATENCY_CYCLES = 4            # PHY turnaround before PhyStatus answers

# Special Symbols as bytes on the codec's plaintext side (PCIe Base Spec r2.1,
# Table 4-1). Only K_COM is read.
K_STP = 0xFB   # K27.7 -- start of a TLP
K_END = 0xFD   # K29.7 -- end of a TLP
K_COM = 0xBC   # K28.5 -- comma

WINDOW = 60000  # cycles each bring-up monitor samples


# ---------------------------------------------------------------------------
# Bench driver and PIPE sideband
# ---------------------------------------------------------------------------
# TB starts the clock and runs the reset sequence. receiver_detect and
# phy_presence stand in for the two PHYs' sideband: each stack is a MAC, so
# each needs a PHY to answer its receiver-detect request and to report a
# receiver that is not electrically idle. The symbol datapath between the
# stacks is RTL (the bridge).
class TB:
    """Clock and reset for one test. Each test builds its own TB, because the
    clock is a task the test starts and cocotb ends a test's tasks with it."""
    def __init__(self, dut):
        """Start the CLK_NS clock on clk_i."""
        self.dut = dut
        cocotb.start_soon(Clock(dut.clk_i, CLK_NS, units="ns").start())

    async def reset(self):
        """Hold rst_i for 10 cycles with every bench input idle, release it,
        and wait 5 more cycles."""
        d = self.dut
        d.rst_i.value = 1
        d.en_i.value = 0
        d.transmit_enable_i.value = 0
        d.tx_elec_idle.value = 0
        d.phy_ready_en.value = 0

        d.scan_start_i.value = 0
        d.scan_bus_i.value = 0
        d.bar_enable_i.value = 0
        d.bridge_enable_i.value = 0

        # Both PHY sidebands start idle: no receiver seen, electrically idle.
        d.rc_phy_phystatus.value = 0
        d.rc_phy_phystatus_rst.value = 0
        d.rc_phy_rxelecidle.value = 1
        d.rc_phy_rxstatus.value = 0

        d.ep_phy_phystatus.value = 0
        d.ep_phy_phystatus_rst.value = 0
        d.ep_phy_rxelecidle.value = 1
        d.ep_phy_rxstatus.value = 0

        d.s_axis_rq_tdata.value = 0
        d.s_axis_rq_tkeep.value = 0
        d.s_axis_rq_tvalid.value = 0
        d.s_axis_rq_tlast.value = 0
        d.s_axis_rq_tuser.value = 0

        await ClockCycles(d.clk_i, 10)
        d.rst_i.value = 0
        await ClockCycles(d.clk_i, 5)


async def receiver_detect(dut, txdetectrx, phystatus, rxstatus):
    """One end's half of the PIPE receiver-detect handshake.

    Each MAC clears its detect latch on the rising edge of its own
    phy_txdetectrx and sets it only when it later sees phy_phystatus with
    rxstatus == 3'b011 (pcie_phy_top for the RC, pcie_endpoint_top for the EP),
    so the answer is a one-cycle PhyStatus pulse after the request edge. Both
    stacks are MACs, so this runs at both ends; answering only the RC would
    leave the Endpoint's LTSSM in Detect.
    """
    prev = 0
    while True:
        await RisingEdge(dut.clk_i)
        cur = int(txdetectrx.value)
        if cur and not prev:
            for _ in range(DETECT_LATENCY_CYCLES):
                await RisingEdge(dut.clk_i)
            rxstatus.value = RXSTATUS_RECEIVER_DETECTED
            phystatus.value = 1
            await RisingEdge(dut.clk_i)
            phystatus.value = 0
            rxstatus.value = 0
            cur = int(txdetectrx.value)
        prev = cur


async def phy_presence(dut):
    """Both ends see a live, non-idle partner once the bench is running.

    The datapath itself is RTL-to-RTL through the bridge; this drives only the
    sideband levels a transceiver would.
    """
    while True:
        await RisingEdge(dut.clk_i)
        dut.rc_phy_rxelecidle.value = 0
        dut.ep_phy_rxelecidle.value = 0


# ---------------------------------------------------------------------------
# Bring-up monitors
# ---------------------------------------------------------------------------
# Each monitor samples its signals on every rising edge for a fixed number of
# cycles and reports afterwards; nothing it records is judged during the run.
# A bare read after RisingEdge returns the value from before the edge, the
# value the DUT's flops sampled, and every monitor reads with that phase.
# bring_up starts all but TlpPathWitness before en_i rises and returns them;
# _run_and_report waits for them and logs every census.
class Monotonic:
    """Continuous sampler for a signal expected to rise once and stay high.

    It samples every cycle from before the event, instead of waiting for the
    event and then reading, so the low period before the rise is observed
    rather than assumed. It records whether the signal was seen low before
    rising, whether and at which sampled cycle it rose, and whether it fell
    after rising.
    """

    def __init__(self, handle):
        """Wrap one handle; nothing observed yet."""
        self.h = handle
        self.saw_low = False
        self.rose = False
        self.fell_after_rise = False
        self.rise_cycle = None

    async def run(self, clk, cycles):
        """Sample the handle on each of `cycles` rising edges."""
        for n in range(cycles):
            await RisingEdge(clk)
            if int(self.h.value) == 0:
                if not self.rose:
                    self.saw_low = True
                else:
                    self.fell_after_rise = True
            elif not self.rose:
                self.rose = True
                self.rise_cycle = n


class BothEndsProbe:
    """Hierarchical probe into both Flow Control initializers.

    Records how far each side got: link up at its DLL, DL_Init
    (start_flow_control_i), the peer's InitFC1 values stored, the peer's InitFC2
    values stored. diagnose() names the first missing step for one side, so a
    failure names the end at fault and where it stopped.
    """

    def __init__(self, dut):
        """Handles on both DLLs and both pcie_flow_ctrl_init instances."""
        # The two stacks name their Data Link Layer instance differently:
        # pcie_phy_top uses pcie_datalink_layer_inst and pcie_endpoint_top uses
        # datalink_layer_inst.
        rc = dut.u_rc.u_phy.pcie_datalink_layer_inst
        ep = dut.u_ep.datalink_layer_inst
        self.rc_fci = rc.pcie_flow_ctrl_init_inst
        self.ep_fci = ep.pcie_flow_ctrl_init_inst
        self.rc_dll = rc
        self.ep_dll = ep
        self.s = {k: False for k in (
            "rc_link", "rc_start", "rc_fc1", "rc_fc2",
            "ep_link", "ep_start", "ep_fc1", "ep_fc2",
        )}
        self.rc_states = set()
        self.ep_states = set()

    async def run(self, clk, cycles):
        """Record, per side, whether link-up, start_flow_control_i and the FC1
        and FC2 stored flags were ever high, and every FSM state seen."""
        for _ in range(cycles):
            await RisingEdge(clk)
            if int(self.rc_dll.phy_link_up_i.value):
                self.s["rc_link"] = True
            if int(self.ep_dll.phy_link_up_i.value):
                self.s["ep_link"] = True
            for pfx, fci, states in (("rc", self.rc_fci, self.rc_states),
                                     ("ep", self.ep_fci, self.ep_states)):
                if int(fci.start_flow_control_i.value):
                    self.s[pfx + "_start"] = True
                if int(fci.fc1_values_stored_i.value):
                    self.s[pfx + "_fc1"] = True
                if int(fci.fc2_values_stored_i.value):
                    self.s[pfx + "_fc2"] = True
                states.add(int(fci.curr_state.value))

    def report(self, dut):
        """Log the probe's flags and FSM states for both sides."""
        dut._log.info(
            "PROBE RC: link=%s start_fc=%s fc1_stored=%s fc2_stored=%s states=%s",
            self.s["rc_link"], self.s["rc_start"], self.s["rc_fc1"],
            self.s["rc_fc2"], sorted(self.rc_states),
        )
        dut._log.info(
            "PROBE EP: link=%s start_fc=%s fc1_stored=%s fc2_stored=%s states=%s",
            self.s["ep_link"], self.s["ep_start"], self.s["ep_fc1"],
            self.s["ep_fc2"], sorted(self.ep_states),
        )

    def diagnose(self, side):
        """The first of the four steps that `side` never reached, or that flow
        control completed, as a sentence."""
        if not self.s[side + "_link"]:
            return (f"{side.upper()}: its DLL never saw link up -- the link-up "
                    f"path INTO the DLL is the suspect, not flow control")
        if not self.s[side + "_start"]:
            return (f"{side.upper()}: link up but never entered DL_Init, so "
                    f"flow-control initialisation was never attempted")
        if not self.s[side + "_fc1"]:
            return (f"{side.upper()}: originated InitFC1 but never received one "
                    f"-- the echo was not recovered, i.e. the far end never "
                    f"answered or the codec bridge corrupted it")
        if not self.s[side + "_fc2"]:
            return (f"{side.upper()}: stored InitFC1 but never reached FC2")
        return f"{side.upper()}: flow control completed"


class DatapathCensus:
    """Where does a DLLP stop? Counts valid beats at four points.

    RC -> EP: characters the RC puts on its PIPE TX (rc_phy_txdata_valid),
    symbols arriving at the EP's symbol seam (seam_symbol_valid), and beats the
    EP's phy_receive hands its DLL (dll_phy_rx_tvalid). EP -> RC: beats the
    RC's phy_receive hands its DLL (m_dllp_axis_tvalid). The stage at which the
    counts stop says which boundary the traffic stops at.
    """

    def __init__(self, dut):
        """Handles on the four points; zeroed counters."""
        self.dut = dut
        self.rc_tx_beats = 0        # RC put characters on the wire
        self.ep_rx_sym_beats = 0    # they arrived at the EP's symbol seam
        self.ep_dll_rx_beats = 0    # beats the EP's phy_receive handed its DLL
        self.ep_tx_beats = 0        # not counted by run()
        self.rc_dll_rx_beats = 0    # beats the RC's phy_receive handed its DLL
        self.ep_dll_rx = dut.u_ep.dll_phy_rx_tvalid
        self.rc_dll_rx = dut.u_rc.u_phy.m_dllp_axis_tvalid

    async def run(self, clk, cycles):
        """Count valid beats at each point for `cycles` cycles."""
        d = self.dut
        for _ in range(cycles):
            await RisingEdge(clk)
            if int(d.rc_phy_txdata_valid.value):
                self.rc_tx_beats += 1
            if int(d.seam_symbol_valid.value):
                self.ep_rx_sym_beats += 1
            if int(self.ep_dll_rx.value):
                self.ep_dll_rx_beats += 1
            if int(self.rc_dll_rx.value):
                self.rc_dll_rx_beats += 1

    def report(self, dut):
        """Log the counts, RC -> EP first."""
        dut._log.info(
            "DATAPATH RC->EP: rc_tx_beats=%d  ep_rx_symbol_beats=%d  "
            "ep_dll_rx_beats=%d", self.rc_tx_beats, self.ep_rx_sym_beats,
            self.ep_dll_rx_beats,
        )
        dut._log.info(
            "DATAPATH EP->RC: rc_dll_rx_beats=%d", self.rc_dll_rx_beats)


class DllpAcceptance:
    """Where inside dllp_handler does an arriving DLLP stop being accepted?

    dllp_handler accepts a DLLP in three steps, each counted here:
      dllp_first_word_valid  tkeep all ones and not tlast
      dllp_crc_word_valid    tlast with tkeep 2'b11
      the CRC compare        crc_reversed == tdata[15:0] on a CRC word
    Counting all three separates a framing that never presents a DLLP from a
    DLLP that is presented and fails its CRC, which are faults in different
    modules. The three fc1_*_stored_r flags are counted too, because
    fc1_values_stored_o is their AND and one missing class differs from all
    three missing.
    """

    def __init__(self, dut, side):
        """Handle on one stack's dllp_handler; zeroed counters."""
        if side == "ep":
            h = dut.u_ep.datalink_layer_inst.dllp_receive_inst.dllp_handler_inst
        else:
            h = (dut.u_rc.u_phy.pcie_datalink_layer_inst
                 .dllp_receive_inst.dllp_handler_inst)
        self.h = h
        self.side = side
        self.first_word = 0
        self.crc_word = 0
        self.crc_match = 0
        self.np = 0
        self.p = 0
        self.c = 0

    async def run(self, clk, cycles):
        """Count, for `cycles` cycles, the cycles on which each acceptance step
        and each FC1 stored flag is high."""
        h = self.h
        for _ in range(cycles):
            await RisingEdge(clk)
            if int(h.dllp_first_word_valid.value):
                self.first_word += 1
            cw = int(h.dllp_crc_word_valid.value)
            if cw:
                self.crc_word += 1
                if int(h.crc_reversed.value) == (
                        int(h.skid_s_axis_tdata.value) & 0xFFFF):
                    self.crc_match += 1
            if int(h.fc1_np_stored_r.value):
                self.np += 1
            if int(h.fc1_p_stored_r.value):
                self.p += 1
            if int(h.fc1_c_stored_r.value):
                self.c += 1

    def report(self, dut):
        """Log the counts on one DLLP line."""
        dut._log.info(
            "DLLP-%s: first_word=%d crc_word=%d crc_MATCH=%d | "
            "fc1_stored np=%d p=%d c=%d",
            self.side.upper(), self.first_word, self.crc_word, self.crc_match,
            self.np, self.p, self.c,
        )


class ScramblerLockstep:
    """Are the RC's transmit LFSR and the EP's receive LFSR in lockstep?

    Ordered sets are not scrambled and DLLPs are, so TS1 and TS2 cross and both
    LTSSMs reach L0 whatever the scramblers do, while a receive LFSR out of step
    with the transmit LFSR turns every DLLP into noise that fails its CRC with
    the framing Symbols intact (PCIe Base Spec r2.1, §4.2.3). A link that trains
    but accepts no DLLP therefore points at scrambler lockstep. The COM Symbol
    initializes both LFSRs (§4.2.3), so after training they should advance on
    the same events; the probe counts each LFSR's advances and sweeps the lag
    between the two values.
    """

    def __init__(self, dut):
        """Handles on the two LFSRs; zeroed counters and the lag-sweep history."""
        self.tx = (dut.u_rc.u_phy.phy_transmit_inst
                   .gen_lane_scramble[0].scrambler_inst
                   .gen1_scramble_inst.Q)
        # On the EP side phy_receive_inst is inside pcie_endpoint_top's generate
        # block gen_integrated_gen1_phy, so its path carries that label; the
        # RC's pcie_phy_top has no such block.
        self.rx = (dut.u_ep.gen_integrated_gen1_phy.phy_receive_inst
                   .gen_lane_descramble[0].descrambler_inst
                   .gen1_scramble_inst.Q)
        self.samples = 0
        self.tx_advances = 0
        self.rx_advances = 0
        # Lag sweep: if the receive LFSR equals the transmit LFSR delayed by a
        # fixed lag, the two are in lockstep and only the latency differs. If no
        # lag gives a high match rate, they advance on different events.
        self.max_lag = 12
        self.lag_hits = [0] * (self.max_lag + 1)
        self.tx_hist = []
        self.tx_trace = []
        self.rx_trace = []

    async def run(self, clk, cycles, start_after):
        """Sample both LFSRs every cycle of `cycles` after the first
        start_after."""
        prev_t = None
        prev_r = None
        for n in range(cycles):
            await RisingEdge(clk)
            if n < start_after:
                continue
            t = int(self.tx.lfsr_in.value)
            r = int(self.rx.lfsr_in.value)
            self.samples += 1
            if prev_t is not None and t != prev_t:
                self.tx_advances += 1
            if prev_r is not None and r != prev_r:
                self.rx_advances += 1
            self.tx_hist.append(t)
            if len(self.tx_hist) > self.max_lag + 1:
                self.tx_hist.pop(0)
            # tx_hist[-1] is lag 0, tx_hist[-1-k] is lag k
            for k in range(self.max_lag + 1):
                if len(self.tx_hist) > k and self.tx_hist[-1 - k] == r:
                    self.lag_hits[k] += 1
            if len(self.tx_trace) < 20:
                self.tx_trace.append(t)
                self.rx_trace.append(r)
            prev_t, prev_r = t, r

    def report(self, dut):
        """Log the advance rates, the lag sweep, the best lag and the first 20
        values of each LFSR."""
        if not self.samples:
            dut._log.info("SCRAMBLER: no samples")
            return
        s = self.samples
        dut._log.info(
            "SCRAMBLER advances over %d samples: tx=%d (%.1f%%) rx=%d (%.1f%%)",
            s, self.tx_advances, 100.0 * self.tx_advances / s,
            self.rx_advances, 100.0 * self.rx_advances / s,
        )
        best = max(range(self.max_lag + 1), key=lambda k: self.lag_hits[k])
        dut._log.info(
            "SCRAMBLER lag sweep (match%% by lag): %s",
            " ".join(f"{k}:{100.0 * self.lag_hits[k] / s:.1f}"
                     for k in range(self.max_lag + 1)),
        )
        dut._log.info(
            "SCRAMBLER best lag = %d at %.1f%% -- %s", best,
            100.0 * self.lag_hits[best] / s,
            "PHASE problem, fixable in the bench"
            if self.lag_hits[best] > 0.8 * s else
            "NO lag explains it: the LFSRs advance on DIFFERENT EVENTS",
        )
        dut._log.info("SCRAMBLER tx_lfsr[0:20]=%s",
                      [f"{v:04x}" for v in self.tx_trace])
        dut._log.info("SCRAMBLER rx_lfsr[0:20]=%s",
                      [f"{v:04x}" for v in self.rx_trace])


class CodecHealth:
    """Sticky codec error census, both directions, sampled continuously."""

    def __init__(self, dut):
        """Every flag starts clear."""
        self.dut = dut
        self.br_enc_illegal_k = False
        self.br_dec_code_err = False
        self.br_dec_disp_err = False
        self.ep_rx_code_error = False
        self.ep_rx_disparity_error = False
        self.ep_tx_illegal_k = False

    async def run(self, clk, cycles):
        """Latch each codec error flag that is ever high during `cycles` cycles."""
        d = self.dut
        for _ in range(cycles):
            await RisingEdge(clk)
            if int(d.br_enc_illegal_k.value):
                self.br_enc_illegal_k = True
            if int(d.br_dec_code_err.value):
                self.br_dec_code_err = True
            if int(d.br_dec_disp_err.value):
                self.br_dec_disp_err = True
            if int(d.ep_phy_rx_code_error.value):
                self.ep_rx_code_error = True
            if int(d.ep_phy_rx_disparity_error.value):
                self.ep_rx_disparity_error = True
            if int(d.ep_phy_tx_illegal_k.value):
                self.ep_tx_illegal_k = True

    def report(self, dut):
        """Log the six flags."""
        dut._log.info(
            "CODEC: bridge enc_illegal_k=%s dec_code_err=%s dec_disp_err=%s | "
            "EP rx_code_err=%s rx_disp_err=%s tx_illegal_k=%s",
            self.br_enc_illegal_k, self.br_dec_code_err, self.br_dec_disp_err,
            self.ep_rx_code_error, self.ep_rx_disparity_error,
            self.ep_tx_illegal_k,
        )

    def assert_clean(self, dut):
        """Log the six flags, then assert that neither decoder saw a disparity
        error."""
        dut._log.info(
            "CODEC: bridge enc_illegal_k=%s dec_code_err=%s dec_disp_err=%s | "
            "EP rx_code_err=%s rx_disp_err=%s tx_illegal_k=%s",
            self.br_enc_illegal_k, self.br_dec_code_err, self.br_dec_disp_err,
            self.ep_rx_code_error, self.ep_rx_disparity_error,
            self.ep_tx_illegal_k,
        )
        assert not self.br_dec_disp_err, (
            "the bridge's decoder reported a disparity error: the running-"
            "disparity chains desynchronised from the Endpoint's"
        )
        assert not self.ep_rx_disparity_error, (
            "the Endpoint's decoder reported a disparity error: the bridge's "
            "ENCODE disparity desynchronised from the Endpoint's rx disparity"
        )


class TlpPathWitness:
    """The TLP path at one stack's DLL AXIS input.

    The seam is dllp_receive_inst.s_axis_*, the Data Link Layer's inbound AXIS
    and the first point inside the DLL where a TLP is a packet. Beats are
    classified as TLP by s_axis_tuser[1], the bit axis_user_demux names
    UserIsTlp and routes on, so the witness classifies as the DUT does. It
    counts TLP beats and packets (tlast), beats per packet, tkeep on the last
    beat, TLPs delivered upward on the DLL's m_tlp_axis, and the cycles on which
    dllp2tlp's tlp_nullified_o, its latched Nak decision, is high.

    Every read is bare after RisingEdge: the pre-edge value, which is what the
    DUT's flops sample at that edge and the right phase for counting AXIS
    handshakes.
    """

    def __init__(self, dut, side):
        """Handles on the stack's DLL, dllp_receive and dllp2tlp; zeroed counts."""
        if side == "ep":
            dll = dut.u_ep.datalink_layer_inst
        else:
            dll = dut.u_rc.u_phy.pcie_datalink_layer_inst
        self.side = side
        self.dll = dll
        self.rx = dll.dllp_receive_inst
        self.d2t = dll.dllp_receive_inst.dllp2tlp_inst
        self.in_beats = 0          # TLP beats accepted at the DLL AXIS input
        self.in_pkts = 0           # ... that completed with tlast
        self.beat_hist = {}        # beats-per-packet histogram
        self.keep_on_last = {}
        self.up_beats = 0          # delivered to this stack's Transaction Layer
        self.up_pkts = 0
        self.nullified = 0         # cycles with tlp_nullified_o high
        self._cur = 0

    async def run(self, clk, cycles):
        """Count handshakes and nullified cycles for `cycles` cycles."""
        rx, d2t, dll = self.rx, self.d2t, self.dll
        for _ in range(cycles):
            await RisingEdge(clk)
            if int(rx.s_axis_tvalid.value) and int(rx.s_axis_tready.value):
                if (int(rx.s_axis_tuser.value) >> 1) & 1:
                    self.in_beats += 1
                    self._cur += 1
                    if int(rx.s_axis_tlast.value):
                        self.in_pkts += 1
                        k = int(rx.s_axis_tkeep.value)
                        self.keep_on_last[k] = self.keep_on_last.get(k, 0) + 1
                        self.beat_hist[self._cur] = (
                            self.beat_hist.get(self._cur, 0) + 1)
                        self._cur = 0
            if int(dll.m_tlp_axis_tvalid.value) and int(dll.m_tlp_axis_tready.value):
                self.up_beats += 1
                if int(dll.m_tlp_axis_tlast.value):
                    self.up_pkts += 1
            if int(d2t.tlp_nullified_o.value):
                self.nullified += 1

    def report(self, dut):
        """Log the counts and histograms on one TLPWIT line."""
        dut._log.info(
            "TLPWIT %-2s DLL-AXIS-IN tlp_beats=%d tlp_pkts=%d beats_per_pkt=%s "
            "tkeep_on_last=%s | UP-TO-TL beats=%d pkts=%d | nullified=%d",
            self.side, self.in_beats, self.in_pkts,
            {k: v for k, v in sorted(self.beat_hist.items())},
            {hex(k): v for k, v in sorted(self.keep_on_last.items())},
            self.up_beats, self.up_pkts, self.nullified,
        )


async def bring_up(dut, window=WINDOW):
    """Reset, start both PHY sideband models and every monitor, then raise en_i,
    phy_ready_en and transmit_enable_i.

    Returns (tb, mons, probe, codec, path, scram, (dllp_rc, dllp_ep), tasks).
    Every monitor starts before en_i rises, so the low period of each monitored
    signal falls inside its window instead of being assumed.
    """
    tb = TB(dut)
    await tb.reset()

    cocotb.start_soon(receiver_detect(
        dut, dut.rc_phy_txdetectrx, dut.rc_phy_phystatus, dut.rc_phy_rxstatus))
    cocotb.start_soon(receiver_detect(
        dut, dut.ep_phy_txdetectrx, dut.ep_phy_phystatus, dut.ep_phy_rxstatus))
    cocotb.start_soon(phy_presence(dut))

    mons = {
        "rc_link": Monotonic(dut.rc_link_up_o),
        "ep_link": Monotonic(dut.ep_phy_link_up_o),
        "rc_fc": Monotonic(dut.rc_fc_initialized_o),
        "ep_fc": Monotonic(dut.ep_fc_initialized_o),
    }
    probe = BothEndsProbe(dut)
    codec = CodecHealth(dut)
    path = DatapathCensus(dut)
    scram = ScramblerLockstep(dut)
    dllp_ep = DllpAcceptance(dut, "ep")
    dllp_rc = DllpAcceptance(dut, "rc")

    tasks = [cocotb.start_soon(m.run(dut.clk_i, window)) for m in mons.values()]
    tasks.append(cocotb.start_soon(probe.run(dut.clk_i, window)))
    tasks.append(cocotb.start_soon(codec.run(dut.clk_i, window)))
    tasks.append(cocotb.start_soon(path.run(dut.clk_i, window)))
    # The scrambler probe ignores the first 3,000 cycles, which are meant to
    # cover link training: there every COM re-seeds both LFSRs (PCIe Base
    # Spec r2.1, §4.2.3) and a mismatch means nothing. Nothing here checks
    # that training has ended by cycle 3,000.
    tasks.append(cocotb.start_soon(scram.run(dut.clk_i, window, 3000)))
    tasks.append(cocotb.start_soon(dllp_ep.run(dut.clk_i, window)))
    tasks.append(cocotb.start_soon(dllp_rc.run(dut.clk_i, window)))

    await ClockCycles(dut.clk_i, 5)
    dut.en_i.value = 1
    dut.phy_ready_en.value = 1
    dut.transmit_enable_i.value = 1

    return tb, mons, probe, codec, path, scram, (dllp_rc, dllp_ep), tasks


async def _run_and_report(dut):
    """Bring up, run the monitors to the end of the window, and log every
    census before any assertion.

    Diagnostics come before verdicts: a test whose first failing assertion hid
    the census that explains it would need a second run to learn what the first
    already knew. Both link-training tests use this, so a failing test's log
    carries the same evidence as a passing one.
    """
    tb, mons, probe, codec, path, scram, dllps, tasks = await bring_up(dut)
    for t in tasks:
        await t

    probe.report(dut)
    codec.report(dut)
    path.report(dut)
    scram.report(dut)
    for dl in dllps:
        dl.report(dut)
    for name, m in mons.items():
        dut._log.info("MON %-8s saw_low=%s rose=%s rise_cycle=%s fell_after=%s",
                      name, m.saw_low, m.rose, m.rise_cycle, m.fell_after_rise)
    return mons, probe, codec, path, scram, dllps


# ---------------------------------------------------------------------------
# Link training and Flow Control initialization
# ---------------------------------------------------------------------------
# Both tests use _run_and_report: one bring-up over WINDOW cycles with every
# monitor running and every census logged, then the assertions. The first
# test checks training, the codec, the bridge and the scramblers; the second
# checks that Flow Control initialization completes on both stacks and that
# fc_initialized_o then stays high.
@cocotb.test()
async def fullstack_both_stacks_train_to_l0_through_the_codec(dut):
    """Two logical PHYs face each other through the 8b/10b codec and both link
    up.

    Five claims, each able to fail on its own:
      1. both LTSSMs raise link up across the encoded seam (pcie_ltssm_downstream
         sets it in Configuration.Idle, L0 and Recovery);
      2. both Data Link Layers enter DL_Init (start_flow_control_i rises);
      3. the codec is clean both ways: no code error and no disparity error
         (the illegal-K flags are logged, not asserted);
      4. the bridge neither creates nor drops beats;
      5. the two scramblers advance in lockstep.
    Claims 4 and 5 cover only the RC -> EP path, from the RC's transmit
    scrambler through the bridge to the EP's receive descrambler.
    Non-vacuity: each link-up signal is seen low before it rises, so a stack
    that came out of reset with it high would fail, and more than 1,000 beats
    cross the seam.
    """
    mons, probe, codec, path, scram, dllps = await _run_and_report(dut)

    # ---- 1. both trained -------------------------------------------------
    assert mons["rc_link"].saw_low, (
        "non-vacuity failed: rc_link_up_o was never observed low")
    assert mons["rc_link"].rose, (
        f"the RC's LTSSM never reached L0 through the bridge. "
        f"{probe.diagnose('rc')}")
    assert mons["ep_link"].saw_low, (
        "non-vacuity failed: ep_phy_link_up_o was never observed low")
    assert mons["ep_link"].rose, (
        f"Joy's Endpoint never reached L0 through the bridge. This is a STOP, "
        f"not a patch target. {probe.diagnose('ep')}")

    # ---- 2. both originated ----------------------------------------------
    assert probe.s["rc_start"], f"RC never entered DL_Init. {probe.diagnose('rc')}"
    assert probe.s["ep_start"], f"EP never entered DL_Init. {probe.diagnose('ep')}"

    # ---- 3. the codec is clean both ways ---------------------------------
    codec.assert_clean(dut)
    assert not codec.br_dec_code_err, (
        "the bridge's decoder saw a 10-bit group that is not a valid code "
        "group -- the Endpoint's encoder emitted something illegal")
    assert not codec.ep_rx_code_error, (
        "the Endpoint's decoder saw an invalid code group -- the bridge's "
        "encoder emitted something illegal")

    # ---- 4. the bridge is transparent ------------------------------------
    # One beat of slack: the bridge registers, so the last beat of the window
    # has been launched but not yet landed. Asserting equality would fail on a
    # correct bridge for a window-edge reason.
    assert abs(path.rc_tx_beats - path.ep_rx_sym_beats) <= 1, (
        f"the bridge is not transparent: {path.rc_tx_beats} beats in, "
        f"{path.ep_rx_sym_beats} symbol beats out"
    )
    assert path.ep_rx_sym_beats > 1000, (
        f"non-vacuity failed: only {path.ep_rx_sym_beats} beats crossed the "
        f"seam, so claim 4 is nearly empty")

    # ---- 5. the scramblers advance together ------------------------------
    # Advance counts, not values: the receive LFSR trails the transmit LFSR by
    # the link's pipeline latency, so equal values on the same cycle would be
    # the wrong property. Equal advance counts say both step on the same events.
    assert scram.samples > 0, "non-vacuity failed: the scrambler probe never ran"
    drift = abs(scram.tx_advances - scram.rx_advances)
    assert drift <= 4, (
        f"the transmit and receive LFSRs advanced a different number of times "
        f"({scram.tx_advances} vs {scram.rx_advances}, drift {drift}). They are "
        f"stepping on different events, and every scrambled byte after the "
        f"first divergence is noise")

    dut._log.info(
        "ROW 1a: RC L0 at cycle %s, EP L0 at cycle %s; %d beats across the "
        "encoded seam, zero codec errors, LFSR advance drift %d",
        mons["rc_link"].rise_cycle, mons["ep_link"].rise_cycle,
        path.ep_rx_sym_beats, drift,
    )


@cocotb.test()
async def fullstack_completes_fc_init_both_ways(dut):
    """Flow Control initialization completes on both stacks and stays complete.

    Each stack's fc_initialized_o must be seen low, then rise, then never fall
    inside the window. Completion is a one-way event: FC_INIT2 signals
    completion and exits, and DL_Active is left only when Physical LinkUp falls
    (PCIe Base Spec r2.1, §3.3.1, §3.2.1). Neither pcie_rc_top nor
    pcie_endpoint_top filters its DLL's fc_initialized_o, so a glitch at the
    source would show here.
    """
    mons, probe, codec, path, scram, dllps = await _run_and_report(dut)

    assert mons["rc_fc"].saw_low, (
        "non-vacuity failed: rc_fc_initialized_o was never observed low, so "
        "this row cannot distinguish 'completed' from 'asserted out of reset'")
    assert mons["rc_fc"].rose, (
        f"the RC never completed flow-control initialisation. "
        f"{probe.diagnose('rc')}")
    assert mons["ep_fc"].saw_low, (
        "non-vacuity failed: ep_fc_initialized_o was never observed low")
    assert mons["ep_fc"].rose, (
        f"Joy's Endpoint never completed flow-control initialisation. "
        f"{probe.diagnose('ep')}")

    # ---- and it stays high ----------------------------------------------
    assert not mons["rc_fc"].fell_after_rise, (
        "rc_fc_initialized_o FELL after rising. Base 2.1 p.158/p.161 make "
        "FC-init completion a one-way event, and this is its first consumer "
        "with no fc_init_sticky_r to hide a glitch -- so the source fix for "
        "conformance defect #3 is incomplete")
    assert not mons["ep_fc"].fell_after_rise, (
        "ep_fc_initialized_o FELL after rising -- the same one-way event, on "
        "the Endpoint side")

    dut._log.info(
        "ROW 1b: FC init completed both ways (RC at %s, EP at %s)",
        mons["rc_fc"].rise_cycle, mons["ep_fc"].rise_cycle,
    )


# ---------------------------------------------------------------------------
# Enumeration across two PHYs
# ---------------------------------------------------------------------------
# The RC's enumeration engine issues real CfgRd0 and CfgWr0 requests and the
# Endpoint's own configuration space answers them: pcie_config_reg inside
# pcie_cfg_wrapper, which sits in the EP's Data Link Layer (dllp_receive).
# Every expected value comes from that RTL, not from a Python model.
# run_enumeration_fs raises bar_enable_i, pulses scan_start_i and waits for
# enum_done_o, enum_error_o or scan_error_o; _log_enum_fs logs the result. The
# bench brings out no Completion Status, tag or completion-timeout signal of
# the RC, so these tests judge Completions by what the engine produced.


def _i(sig):
    """A handle's value as an int."""
    return int(sig.value)


ENUM_CYCLES = 400000
"""§63 #7e: the enumeration window, sized from the MEASURED round trip.

⚠️ 60,000 CYCLES WAS NEVER ENOUGH ONCE ENUMERATION ACTUALLY PROGRESSED, and
that only became visible after F17's two bench defects were fixed.

Row 7 measured one CfgRd0 -> CplD round trip at 5,122 cycles through two real
PHYs and the codec bridge. A full enumeration is not one round trip: it is the
presence scan plus, per BAR, a write-ones and a read-back (PCI 3.0 §6.2.5.1).
At ~5 k cycles each, six BARs is already well past 60 k. The measured symptom
matched exactly -- scan_done=1, present=1, VID/DID correct, and bar_count
stalled at 2 of 6 when the window expired with enum_done still low.

So the old default was not a timeout in the DUT; it was the bench refusing to
wait for a link whose latency it had never measured. Sized here at 400,000 --
roughly 78 round trips, comfortably past a six-BAR enumeration.
"""


async def run_enumeration_fs(dut, cycles=ENUM_CYCLES):
    """Raise bar_enable_i, pulse scan_start_i, and wait for enum_done_o,
    enum_error_o or scan_error_o; return the engine's results as a dict.

    A one-cycle scan_start_i pulse is enough even while flow control is still
    down: pcie_rc_top latches it (start_pending_r) and starts the engine once
    FC init completes. bar_enable_i must be high, because pcie_enum_top starts
    the BAR phase on bar_enable_i && scan_done_o; with it low enum_done_o never
    rises. TB.reset() leaves it 0, so it is raised here.

    The function returns inside a ReadOnly phase, so a caller that drives a
    signal next must first wait for a clock edge. The "frames" entry is always
    0.
    """
    d = dut
    d.bar_enable_i.value = 1
    # ENUM_DELAY_K, a diagnostic environment variable (default 0): when set to K,
    # wait for rc_fc_initialized_o and then K more cycles before the start pulse.
    # The delay is counted from FC init because the start request is latched
    # until FC init anyway, so a delay from reset that ends before FC init
    # would move nothing.
    # With K = 0 this block does nothing.
    _k = int(os.environ.get("ENUM_DELAY_K", "0"))
    if _k:
        for _ in range(200000):
            await RisingEdge(d.clk_i)
            await ReadOnly()
            if _i(d.rc_fc_initialized_o):
                break
        await RisingEdge(d.clk_i)
        dut._log.info("PR7F_K fc_init seen; delaying first request by K=%d" % _k)
        for _ in range(_k):
            await RisingEdge(d.clk_i)
    d.scan_start_i.value = 1
    await RisingEdge(d.clk_i)
    d.scan_start_i.value = 0

    frames = 0
    for _ in range(cycles):
        await RisingEdge(d.clk_i)
        await ReadOnly()
        if _i(d.enum_done_o) or _i(d.enum_error_o) or _i(d.scan_error_o):
            break
    await ReadOnly()
    return {
        "enum_done": _i(d.enum_done_o),
        "enum_error": _i(d.enum_error_o),
        "enum_error_code": _i(d.enum_error_code_o),
        "scan_done": _i(d.scan_done_o),
        "scan_error": _i(d.scan_error_o),
        "scan_error_code": _i(d.scan_error_code_o),
        "device_present": _i(d.device_present_o),
        "vendor_id": _i(d.vendor_id_o),
        "device_id": _i(d.device_id_o),
        "header_type": _i(d.header_type_o),
        "multifunction": _i(d.multifunction_o),
        "bar_count": _i(d.bar_count_o),
        "bar_valid": _i(d.bar_valid_o),
        "bar_size": _i(d.bar_size_o),
        "unsupported": _i(d.unsupported_device_o),
        "frames": frames,
    }


def _log_enum_fs(dut, r):
    """Log the enumeration result as one ENUM line."""
    dut._log.info(
        "ENUM done=%s error=%s(code %s) scan_done=%s scan_error=%s(code %s) | "
        "present=%s VID=%#06x DID=%#06x hdr=%#04x mf=%s | "
        "bar_count=%s bar_valid=%#x bar_size=%#x",
        r["enum_done"], r["enum_error"], r["enum_error_code"],
        r["scan_done"], r["scan_error"], r["scan_error_code"],
        r["device_present"], r["vendor_id"], r["device_id"],
        r["header_type"], r["multifunction"],
        r["bar_count"], r["bar_valid"], r["bar_size"],
    )


@cocotb.test()
async def fullstack_cfgrd0_reads_vid_did_across_two_phys(dut):
    """The RC's enumeration engine reads the Endpoint's configuration header.

    The expected values are the constants pcie_config_reg returns, not values
    from Python: Vendor ID 1234h, Device ID 00FFh, Header Type 00h (Type 0,
    single function), the header fields of PCIe Base Spec r2.1, §7.5.1. The
    Completion comes from pcie_cfg_wrapper in the EP's Data Link Layer
    (dllp_receive), not from its Transaction Layer.

    Only the scan phase is judged; the BAR-sizing test covers the BAR phase.
    Non-vacuity: scan_done_o is high with scan_error_o low and the device is
    present. A timed-out scan would leave the ID registers at reset (0000h),
    which the specific non-zero constants then reject.
    """
    tb, _mons, _probe, _codec, _path, _scram, _dllps, _tasks = await bring_up(dut)
    r = await run_enumeration_fs(dut)
    _log_enum_fs(dut, r)

    assert r["scan_done"] and not r["scan_error"], (
        f"the SCAN phase did not complete: scan_done={r['scan_done']} "
        f"scan_error={r['scan_error']} code={r['scan_error_code']}. This is the "
        "non-vacuity guard -- a timed-out scan leaves the ID registers at reset "
        "and every value below would read 0x0000"
    )
    assert r["device_present"] == 1, (
        f"the Endpoint was not detected (scan_error_code {r['scan_error_code']})"
    )
    assert r["vendor_id"] == 0x1234, (
        f"Vendor ID {r['vendor_id']:#06x} != 0x1234, the constant in "
        "pcie_config_reg.sv's readback path"
    )
    assert r["device_id"] == 0x00FF, f"Device ID {r['device_id']:#06x} != 0x00ff"
    assert r["header_type"] == 0x00, (
        f"Header Type {r['header_type']:#04x} != 0x00 (Type 0, single function)"
    )
    assert r["multifunction"] == 0, "the Endpoint reported multi-function"


@cocotb.test()
async def fullstack_bar0_sizes_to_4kb(dut):
    """BAR0 sizes to exactly what the Endpoint's configuration space encodes:
    1 MB, whatever the test's name says.

    The engine writes all ones to BAR0 and reads it back (PCI Local Bus Spec
    r3.0, §6.2.5.1). pcie_config_reg returns the constant FFF00000h for BAR0
    and for BAR1: memory, 32-bit, not prefetchable, lowest address bit read
    back as 1 is bit 20, so 1 MB.
    The 4 KB in the name is the aperture of pcie_endpoint_top's default
    BAR_MASK, which configures tlp_layer's BAR decoder, not the configuration
    space; BAR1 is in the configuration space, but its decoder is off
    (BAR_ENABLE). This test checks the RC's half: across two PHYs and the
    bridge, the engine sizes what the far end encodes. BAR1's size is logged,
    not asserted, so the test does not vouch for the Endpoint's undecoded BAR1.
    """
    tb, _mons, _probe, _codec, _path, _scram, _dllps, _tasks = await bring_up(dut)
    r = await run_enumeration_fs(dut)
    _log_enum_fs(dut, r)

    assert r["enum_done"] and not r["enum_error"], (
        f"enumeration did not complete: code={r['enum_error_code']}"
    )
    assert r["bar_count"] >= 1, f"no BARs reported (bar_count={r['bar_count']})"
    assert r["bar_valid"] & 0x1, (
        f"BAR0 not marked valid (bar_valid={r['bar_valid']:#x})"
    )
    m64 = (1 << 64) - 1
    bar0_size = r["bar_size"] & m64
    bar1_size = (r["bar_size"] >> 64) & m64
    dut._log.info("ROW 3: BAR0 size = %#x, BAR1 size = %#x (reported, not asserted), "
                  "bar_count=%d bar_valid=%#x", bar0_size, bar1_size,
                  r["bar_count"], r["bar_valid"])
    assert bar0_size == 0x100000, (
        f"BAR0 sized to {bar0_size:#x}; pcie_config_reg.sv's BAR0 readback constant "
        "0xFFF00000 encodes 1 MB (0x100000) -- the write-ones/read-back protocol "
        "(PCI 3.0 §6.2.5.1) must recover exactly the size the config space encodes"
    )


@cocotb.test()
async def fullstack_memwr_memrd_round_trip(dut):
    """The non-posted round trip on the RC's requester (RQ) arm works across two
    PHYs: enumeration completes, its scan phase completes, and the Endpoint is
    not reported unsupported.

    Despite the name, no Memory Write or Memory Read is issued. The bench
    brings out neither cpl_timeout_valid_o nor rc_unexpected_completion_o, and
    this test builds no MemWr or MemRd on s_axis_rq_*. It checks the path a
    MemRd would take, minus the opcode: the engine's CfgRd0 and CfgWr0 leave on
    the RQ arm and their Completions come back.
    """
    tb, _mons, _probe, _codec, _path, _scram, _dllps, _tasks = await bring_up(dut)
    r = await run_enumeration_fs(dut)
    _log_enum_fs(dut, r)

    assert r["enum_done"] and not r["enum_error"], (
        "enumeration must complete before a MemRd can be judged: "
        f"code={r['enum_error_code']}"
    )
    assert r["scan_done"] and not r["scan_error"], (
        f"the scan phase did not complete cleanly: done={r['scan_done']} "
        f"error={r['scan_error']} code={r['scan_error_code']}"
    )
    assert not r["unsupported"], (
        "the Endpoint was reported UNSUPPORTED, so the Completion that came "
        "back was not one the requester could accept"
    )


@cocotb.test()
async def fullstack_completion_tag_and_status(dut):
    """Completions returned across the link carry a tracked tag and Successful
    Completion status, judged by their effect.

    Completion Status 000b is Successful Completion (PCIe Base Spec r2.1,
    §2.2.9). The test does not read the Status field or the tag off the wire.
    It asserts that the engine consumed every Completion of a full enumeration
    (enum_done_o, no error) and produced the right VID and DID from them, and
    that the Endpoint was not reported unsupported. pcie_cfg_txn completes a
    request only on a Completion carrying its tag, so an unmatched tag leaves
    the request to time out and ends enumeration in an error; CA, or UR after
    the probe, also ends it in an error; and UR to the probe leaves the ID
    registers at 0000h. The direct signals (rc_unexpected_completion_o, the
    Status field) are not brought out by the bench.
    """
    tb, _mons, _probe, _codec, _path, _scram, _dllps, _tasks = await bring_up(dut)
    r = await run_enumeration_fs(dut)
    _log_enum_fs(dut, r)

    assert r["enum_done"] and not r["enum_error"], (
        f"enumeration did not complete: code={r['enum_error_code']}"
    )
    assert r["vendor_id"] == 0x1234 and r["device_id"] == 0x00FF, (
        f"the engine consumed Completions but produced VID={r['vendor_id']:#06x} "
        f"DID={r['device_id']:#06x}. Correct header values are only producible "
        "from Completions the requester matched to its outstanding tags"
    )
    assert not r["unsupported"], (
        "unsupported_device_o -- a Completion came back that the requester "
        "could not accept"
    )


# ---------------------------------------------------------------------------
# TLP path at each Data Link Layer input
# ---------------------------------------------------------------------------
# Two tests run the same TlpPathWitness, against the shared dllp_receive and
# axis_user_demux, one instance per stack, during one enumeration. The EP-side
# test watches the RC's Configuration Requests arrive; the RC-side test
# watches the EP's Completions. The shape checks (beats per packet, tkeep on
# the last beat, no Nak decision) and the RTL are the same, so a difference
# in those results is a difference in what reaches each DLL. The delivery
# check differs: the RC may discard a replayed Completion. Each witness
# samples the first WINDOW cycles.


async def _tlp_witness(dut, side):
    """Bring up, enumerate, and return that stack's TLP-path witness."""
    wit = TlpPathWitness(dut, side)
    tb, _m, _p, _c, _pa, _s, _d, tasks = await bring_up(dut)
    wtask = cocotb.start_soon(wit.run(dut.clk_i, WINDOW))
    r = await run_enumeration_fs(dut)
    _log_enum_fs(dut, r)
    for t in tasks:
        await t
    await wtask
    wit.report(dut)
    return wit


@cocotb.test()
async def fullstack_witness_ep_dll_tlp_path(dut):
    """The RC's Configuration Requests at the EP's DLL input are well formed,
    and every one is delivered upward.

    On the link a TLP is a 2-byte sequence number, the TLP and a 4-byte LCRC
    (PCIe Base Spec r2.1, §3.5.1, Figure 3-12). A CfgRd0 is 2 + 12 + 4 = 18
    bytes, five 32-bit beats with two valid bytes on the last (tkeep 0x3); a
    CfgWr0 adds one data DW, 22 bytes in six beats, also ending in tkeep 0x3.
    So every packet is 5 or 6 beats with tkeep 0x3 on the last. The EP's
    tlp_nullified_o (dllp2tlp's latched Nak decision) is never high, and the
    EP's DLL delivers as many TLPs upward as arrive. Non-vacuity: at least one
    TLP arrives.
    """
    wit = await _tlp_witness(dut, "ep")

    assert wit.in_pkts >= 1, (
        "NON-VACUITY: no TLP reached the Endpoint's DLL AXIS input at all, so "
        "this row asserted nothing. Enumeration never issued a CfgRd0, or the "
        "RC->EP direction broke upstream of the DLL."
    )
    assert set(wit.beat_hist) <= {5, 6}, (
        f"beats-per-packet {wit.beat_hist}; Base 2.1 §3.5 makes a link TLP "
        "2 B sequence number + 3 DW header + 4 B LCRC = 18 B = 5 beats without "
        "a data payload, or 22 B = 6 beats with 1 DW. Anything else is not a "
        "config-space TLP shape"
    )
    assert set(wit.keep_on_last) == {0x3}, (
        f"tkeep on the last beat {[hex(k) for k in wit.keep_on_last]}, expected "
        "0x3: 18 B leaves two valid bytes in the fifth beat"
    )
    assert wit.nullified == 0, (
        f"the Endpoint's LCRC check nullified {wit.nullified} TLPs"
    )
    assert wit.up_pkts == wit.in_pkts, (
        f"{wit.in_pkts} TLPs arrived at the Endpoint's DLL and {wit.up_pkts} "
        "were delivered to its Transaction Layer"
    )


@cocotb.test()
async def fullstack_witness_rc_dll_tlp_path(dut):
    """The EP's Completions at the RC's DLL input are well formed, and the RC
    delivers at least one and never more than arrive.

    A CplD with one data DW is 2 + 12 + 4 + 4 = 22 bytes on the link, six beats
    ending in tkeep 0x3; a Completion without data is 18 bytes, five beats
    (PCIe Base Spec r2.1, §3.5.1). The RC's tlp_nullified_o is never high.
    Fewer deliveries than arrivals are allowed: a replayed TLP carries a
    sequence number already received, and the receiver discards it (§3.5.3.1),
    so the check is 1 <= delivered <= arrived. Non-vacuity: at least one TLP
    beat and one complete TLP arrive.
    """
    wit = await _tlp_witness(dut, "rc")

    assert wit.in_beats >= 1, (
        "NON-VACUITY: not one TLP beat reached the Root Complex's DLL AXIS "
        "input. That would be a DIFFERENT and worse defect than F17 -- the "
        "Completion would not have crossed the link at all."
    )
    assert wit.in_pkts >= 1, (
        f"{wit.in_beats} TLP beats arrived at the RC's DLL AXIS input but "
        f"{wit.in_pkts} completed with tlast -- no Completion ever became a "
        "packet. That is a DIFFERENT defect from the one this row pins"
    )
    assert set(wit.beat_hist) <= {5, 6}, (
        f"beats-per-packet {wit.beat_hist}; Base 2.1 §3.5 makes a link TLP "
        "2 B sequence number + 3 DW header + 4 B LCRC = 18 B = 5 beats without "
        "a data payload, or 22 B = 6 beats with 1 DW. Anything else is not a "
        "config-space TLP shape"
    )
    assert set(wit.keep_on_last) == {0x3}, (
        f"tkeep on the last beat {[hex(k) for k in wit.keep_on_last]}, expected "
        "0x3: 22 B leaves two valid bytes in the sixth beat"
    )
    assert wit.nullified == 0, (
        f"the RC's LCRC check nullified {wit.nullified} Completions"
    )
    assert wit.up_pkts >= 1, (
        f"{wit.in_pkts} well-formed Completions arrived at the RC's DLL and "
        f"{wit.up_pkts} reached its Transaction Layer -- none got through at "
        "all, which is a delivery defect rather than duplicate suppression"
    )
    assert wit.up_pkts <= wit.in_pkts, (
        f"{wit.up_pkts} Completions delivered upward but only {wit.in_pkts} "
        "arrived -- the DLL invented one"
    )


# ---------------------------------------------------------------------------
# CfgRd0 round-trip timeline
# ---------------------------------------------------------------------------
# F17Timeline stamps, per test, the cycle of each leg of the round trip: the
# RC's request leaving its TL, arriving at the EP's DLL and passing up, the
# EP's Completion, its arrival at the RC's DLL and delivery to the RC's TL,
# and enum_done_o or enum_error_o. The test asserts only that the RC's TL
# handed its DLL a TLP and that enumeration completed or errored; the
# timeline is logged.


class F17Timeline:
    """Cycle stamps for one CfgRd0 -> CplD round trip, both stacks."""

    def __init__(self, dut):
        """Handles on both DLLs and their receive paths; one empty list of cycles
        per leg."""
        self.dut = dut
        rc = dut.u_rc.u_phy.pcie_datalink_layer_inst
        ep = dut.u_ep.datalink_layer_inst
        self.rc_dll = rc
        self.rc_rx = rc.dllp_receive_inst
        self.ep_rx = ep.dllp_receive_inst
        # event name -> list of cycles
        self.ev = {k: [] for k in (
            "rc_cfgrd0_out",      # RC TL hands the request to its DLL (tlast)
            "ep_cfgrd0_in",       # EP DLL inbound TLP completes (tlast)
            "ep_cfgrd0_up",       # EP DLL passes it up toward its TL (tlast)
            "ep_cpl_generated",   # EP config space emits the Completion (tlast)
            "rc_cpl_in",          # RC DLL inbound TLP completes (tlast)
            "rc_cpl_up",          # RC DLL delivers it to the RC's TL (tlast)
            "enum_error",         # the engine gives up
            "enum_done",
        )}
        # The first word of every inbound TLP at the RC is kept. Bytes 0-1 hold
        # the DLL sequence number, so a replay (the same number, which the
        # receiver discards, PCIe Base Spec r2.1, §3.5.3.1) can be told from a
        # new TLP.
        self.rc_in_first_word = []
        self._rc_pending = None

    def _hs(self, v, r, last):
        """True on a valid, ready, last beat."""
        return int(v.value) and int(r.value) and int(last.value)

    async def run(self, clk, cycles):
        """Stamp every leg's handshakes for `cycles` cycles."""
        d, rc, rcrx, eprx = self.dut, self.rc_dll, self.rc_rx, self.ep_rx
        prev_err = prev_done = 0
        for n in range(cycles):
            await RisingEdge(clk)
            if self._hs(rc.s_tlp_axis_tvalid, rc.s_tlp_axis_tready,
                        rc.s_tlp_axis_tlast):
                self.ev["rc_cfgrd0_out"].append(n)
            if (self._hs(eprx.s_axis_tvalid, eprx.s_axis_tready,
                         eprx.s_axis_tlast)
                    and (int(eprx.s_axis_tuser.value) >> 1) & 1):
                self.ev["ep_cfgrd0_in"].append(n)
            if self._hs(eprx.m_axis_dllp2tlp_tvalid, eprx.m_axis_dllp2tlp_tready,
                        eprx.m_axis_dllp2tlp_tlast):
                self.ev["ep_cfgrd0_up"].append(n)
            if self._hs(eprx.m_cpl_from_cfg_tvalid, eprx.m_cpl_from_cfg_tready,
                        eprx.m_cpl_from_cfg_tlast):
                self.ev["ep_cpl_generated"].append(n)
            if (int(rcrx.s_axis_tvalid.value) and int(rcrx.s_axis_tready.value)
                    and (int(rcrx.s_axis_tuser.value) >> 1) & 1):
                if self._rc_pending is None:
                    self._rc_pending = int(rcrx.s_axis_tdata.value)
                if int(rcrx.s_axis_tlast.value):
                    self.ev["rc_cpl_in"].append(n)
                    self.rc_in_first_word.append(self._rc_pending)
                    self._rc_pending = None
            if self._hs(rc.m_tlp_axis_tvalid, rc.m_tlp_axis_tready,
                        rc.m_tlp_axis_tlast):
                self.ev["rc_cpl_up"].append(n)
            e, dn = int(d.enum_error_o.value), int(d.enum_done_o.value)
            if e and not prev_err:
                self.ev["enum_error"].append(n)
            if dn and not prev_done:
                self.ev["enum_done"].append(n)
            prev_err, prev_done = e, dn

    def report(self, dut):
        """Log each leg's stamps, how many RC arrivals and deliveries came
        before enum_error_o, the inbound first words, and the latency of the
        first round trip."""
        for k in ("rc_cfgrd0_out", "ep_cfgrd0_in", "ep_cfgrd0_up",
                  "ep_cpl_generated", "rc_cpl_in", "rc_cpl_up",
                  "enum_error", "enum_done"):
            v = self.ev[k]
            dut._log.info("F17TL %-17s n=%d cycles=%s", k, len(v), v[:12])
        err = self.ev["enum_error"][0] if self.ev["enum_error"] else None
        if err is not None:
            for k in ("rc_cpl_in", "rc_cpl_up"):
                before = [c for c in self.ev[k] if c <= err]
                after = [c for c in self.ev[k] if c > err]
                dut._log.info(
                    "F17TL VERDICT %-11s before_enum_error=%d after=%d",
                    k, len(before), len(after))
        dut._log.info(
            "F17TL RC_INBOUND_FIRST_WORDS %s  (bytes 0-1 are the DLL sequence "
            "number, Base 2.1 §3.5; identical values = a REPLAY, which the "
            "receiver must discard)",
            [hex(w) for w in self.rc_in_first_word])
        if self.ev["rc_cfgrd0_out"] and self.ev["rc_cpl_in"]:
            dut._log.info(
                "F17TL ROUND_TRIP first_request_out=%d first_completion_in=%d "
                "latency_cycles=%d",
                self.ev["rc_cfgrd0_out"][0], self.ev["rc_cpl_in"][0],
                self.ev["rc_cpl_in"][0] - self.ev["rc_cfgrd0_out"][0])


@cocotb.test()
async def fullstack_f17_timeline(dut):
    """When does each leg of the CfgRd0 round trip happen?

    Only non-vacuity is asserted: the RC's TL handed at least one TLP to its
    DLL, and enumeration completed or errored inside the window. The log
    carries the timeline; when enumeration errors, its VERDICT lines count the
    Completions that reached the RC's DLL and TL before and after enum_error_o.
    """
    # The timeline runs for ENUM_CYCLES, not WINDOW, so that it covers a whole
    # enumeration and the enum_done_o or enum_error_o event it stamps.
    tl = F17Timeline(dut)
    tb, _m, _p, _c, _pa, _s, _d, tasks = await bring_up(dut)
    ttask = cocotb.start_soon(tl.run(dut.clk_i, ENUM_CYCLES))
    r = await run_enumeration_fs(dut)
    _log_enum_fs(dut, r)
    for t in tasks:
        await t
    await ttask
    tl.report(dut)

    assert tl.ev["rc_cfgrd0_out"], (
        "NON-VACUITY: the RC's Transaction Layer never handed a TLP to its DLL, "
        "so no round trip existed to time"
    )
    assert tl.ev["enum_error"] or tl.ev["enum_done"], (
        "NON-VACUITY: enumeration neither completed nor errored inside the "
        "window, so 'before/after the engine gave up' has no referent"
    )


# ---------------------------------------------------------------------------
# Credit release and UpdateFC scheduling
# ---------------------------------------------------------------------------
# A Receiver's CREDITS_ALLOCATED starts at its advertisement and grows as its
# Transaction Layer frees buffer space. Once every advertised unit of a
# non-infinite NPH, NPD, PH or CPLH type has been consumed, an UpdateFC for
# that type must be scheduled when processed TLPs free one or more units, and
# UpdateFCs may be sent more often (PCIe Base Spec r2.1, §2.6.1.2). Without
# that, the RC's credit limit for the EP stays at the 16 NP headers advertised
# at FC init, and its 17th non-posted request waits behind the credit gate.
# W18Capture records raw (cycle, word) tuples and every pairing is done after
# the run; each test first runs the decoders on hand-derived vectors
# (w_selftest). Every signal read is a port of an existing module, as a bare
# read after RisingEdge.

# -- DLLP type byte (pcie_datalink_pkg::dllp_type_e). Bits [2:0] carry the VC,
# which is 0 here, so a type compare masks them: (word & 0xF8) == TYPE.
DLLP_INITFC1_P, DLLP_INITFC1_NP, DLLP_INITFC1_CPL = 0x40, 0x50, 0x60
DLLP_INITFC2_P, DLLP_INITFC2_NP, DLLP_INITFC2_CPL = 0xC0, 0xD0, 0xE0
DLLP_UPDATEFC_P, DLLP_UPDATEFC_NP, DLLP_UPDATEFC_CPL = 0x80, 0x90, 0xA0

# -- the credits the shared DLL advertises at FC init, pcie_datalink_pkg's
# HdrMinCredits and PdMinCredits. The tests read the advertisement from the
# wire; the known-answer vectors are derived from these values.
HDR_MIN_CREDITS = 16
PD_MIN_CREDITS = 64

W_TAIL = 2000
"""Cycles the W monitors keep sampling after enumeration returns.

A release-triggered UpdateFC follows the release it reports by a few cycles
plus arbitration; a register step follows its handshake by one. Two thousand
cycles is two orders of magnitude more than either needs and is short next to
the 5,122-cycle round trip the engine waits out before it returns anyway."""

W2_RELEASE_BOUND = 256
"""§63 #7g-2: cycles from an NP release to the first UpdateFC-NP carrying it.
The release trigger answers in a few cycles plus arbitration behind a CplD;
the periodic timer answers in ~3,750.  256 separates the two with an order of
magnitude either side, so MR-7F2 cannot pass on the periodic refresh."""

COM_WINDOW = 20000
"""Cycles of RC PIPE-TX sampling for W4's COM grid, opened when
rc_fc_initialized_o rises. 3.1 fitted the period at 679 cycles; twenty
thousand cycles holds ~29 periods, enough to see the dominant gap."""


def pinned_red(dut, row, state, detail=""):
    """Log one pipe-separated marker line naming a test, a state and a detail.
    No test in this file calls it."""
    dut._log.info("PINNED_RED|%s|%s|%s", row, state, detail)


def decode_fc_dllp_word(word):
    """First AXIS word of an InitFC or UpdateFC DLLP -> (type, HdrFC, DataFC).

    The layout is pcie_datalink_pkg::dllp_fc_t, little-endian on the 32-bit
    AXIS:
      [7:0]   type            (byte 0)
      [13:8]  HdrFC[7:2]      (byte 1 bits 5:0)
      [23:22] HdrFC[1:0]      (byte 2 bits 7:6)
      [19:16] DataFC[11:8]    (byte 2 bits 3:0)
      [31:24] DataFC[7:0]     (byte 3)
    PCIe Base Spec r2.1, Figure 3-6 to Figure 3-8 give the same byte layout; the
    package's send_fc_init is the builder this inverts.
    """
    t = word & 0xFF
    hdr = (((word >> 8) & 0x3F) << 2) | ((word >> 22) & 0x3)
    data = (((word >> 16) & 0xF) << 8) | ((word >> 24) & 0xFF)
    return t, hdr, data


def decode_tlp_dw0(word):
    """A TLP's DW0 as dllp2tlp presents it on m_tlp_axis -> (fmt_type, length).

    pcie_datalink_pkg::pcie_tlp_header_dw0_t is packed {byte3, byte2, byte1,
    byte0}, so byte0 (Fmt/Type) is at [7:0] and Length is {byte2[1:0], byte3} =
    {[17:16], [31:24]} (PCIe Base Spec r2.1, §2.2.1, Figure 2-5). dllp2tlp
    classifies on this word (the casez in ST_TLP_STREAM), and
    pcie_datalink_layer's s_tlp_axis carries it from the TL in the same layout
    (tlp2dllp reads byte0 of it).
    """
    ft = word & 0xFF
    length = (((word >> 16) & 0x3) << 8) | ((word >> 24) & 0xFF)
    return ft, length


def decode_link_first_word(word):
    """First AXIS word of an inbound link TLP at dllp2tlp's input -> (seq, fmt_type).

    On the link a TLP is a 2-byte sequence number, the TLP and the LCRC (PCIe
    Base Spec r2.1, §3.5.1, Figure 3-12). dllp2tlp's ST_IDLE reads the sequence
    as {tdata[3:0], tdata[15:8]}, and the TLP's own bytes start at [16]: [23:16]
    is Fmt/Type. A non-zero reserved nibble [7:4] marks the frame nullified.
    """
    seq = ((word & 0xF) << 8) | ((word >> 8) & 0xFF)
    ft = (word >> 16) & 0xFF
    return seq, ft


def tlp_credit_class(fmt_type):
    """Fmt/Type -> the FC class dllp2tlp charges it to, or None.

    Mirrors dllp2tlp's casez (PCIe Base Spec r2.1, Table 2-36): NPH for
    non-posted requests without data, NPD for non-posted requests with data
    (which also consume one NPH), PH for messages without data, PD for MWr and
    MsgD, CPLH and CPLD for Completions without and with data.
    """
    fmt = (fmt_type >> 5) & 0x7
    typ = fmt_type & 0x1F
    has_data = bool(fmt & 0x2)
    if typ in (0x00, 0x01):                 # MRd/MRdLk (no data) or MWr (data)
        return "PD" if has_data else "NPH"
    if typ in (0x02, 0x04, 0x05, 0x1B):     # IO, Cfg0, Cfg1, TCfg
        return "NPD" if has_data else "NPH"
    if (typ & 0x18) == 0x10:                # Msg 1_0rrr
        return "PD" if has_data else "PH"
    if typ in (0x0A, 0x0B):                 # Cpl/CplLk, CplD/CplDLk
        return "CPLD" if has_data else "CPLH"
    if typ in (0x0C, 0x0D, 0x0E):           # FetchAdd, Swap, CAS
        return "NPD"
    return None


def is_np_header(fmt_type):
    """True for a TLP that consumes an NPH credit (class NPH or NPD)."""
    return tlp_credit_class(fmt_type) in ("NPH", "NPD")


def gap_histogram(cycles):
    """Sorted event cycles -> {gap: count}."""
    h = {}
    for a, b in zip(cycles, cycles[1:]):
        h[b - a] = h.get(b - a, 0) + 1
    return h


def w_selftest():
    """Known-answer test of the credit decoders, run first by the tests that
    decode captured DLLPs or TLPs.

    The vectors are derived by hand from the bit layouts above, not captured
    from the DUT, so a decoder that shares a mistake with the DUT still fails.
    """
    # InitFC1-NP, HdrFC 16, DataFC 64: bytes 50 04 00 40 -> LE word 0x40000450
    assert decode_fc_dllp_word(0x40000450) == (DLLP_INITFC1_NP, 16, 64), \
        "SELFTEST decode_fc_dllp_word(0x40000450)"
    # UpdateFC-NP, HdrFC 17 (splits 4 into [13:8] and 1 into [23:22]), DataFC 64:
    # byte1 = 0x04, byte2 = 0x40, byte3 = 0x40 -> 0x40400490
    assert decode_fc_dllp_word(0x40400490) == (DLLP_UPDATEFC_NP, 17, 64), \
        "SELFTEST decode_fc_dllp_word(0x40400490)"
    # CfgRd0, Length 1: bytes 04 00 00 01 -> LE 0x01000004
    assert decode_tlp_dw0(0x01000004) == (0x04, 1), "SELFTEST decode_tlp_dw0 CfgRd0"
    # CplD, Length 1 -> 0x0100004A; CfgWr0 Length 1 -> 0x01000044;
    # Length 0x3FF = {11b, 0xFF}: byte2 low bits 11 -> [17:16], byte3 0xFF
    assert decode_tlp_dw0(0x0100004A) == (0x4A, 1), "SELFTEST decode_tlp_dw0 CplD"
    assert decode_tlp_dw0(0xFF030044) == (0x44, 0x3FF), "SELFTEST decode_tlp_dw0 length"
    # link first word: seq 0, CplD -> 0x004A0000
    assert decode_link_first_word(0x004A0000) == (0, 0x4A), "SELFTEST link word seq 0"
    # seq 0x123: tdata[3:0]=1, tdata[15:8]=0x23; CfgRd0 at [23:16]
    assert decode_link_first_word(0x00042301) == (0x123, 0x04), "SELFTEST link word seq 0x123"
    for ft, cls in ((0x04, "NPH"), (0x44, "NPD"), (0x4A, "CPLD"), (0x0A, "CPLH"),
                    (0x40, "PD"), (0x60, "PD"), (0x30, "PH"), (0x00, "NPH"),
                    (0x20, "NPH"), (0x02, "NPH"), (0x42, "NPD"), (0x4C, "NPD")):
        assert tlp_credit_class(ft) == cls, f"SELFTEST tlp_credit_class({ft:#04x})"
    assert gap_histogram([0, 679, 1358, 1400]) == {679: 2, 42: 1}, "SELFTEST gap_histogram"


def _first_attr(handle, names):
    """Resolve the first of `names` that exists under `handle`, as (name,
    handle).

    The NP-header allocated-credit port is looked up as nph_credits_allocated_o,
    dllp2tlp's port, then as nph_credits_consumed_o, which dllp2tlp does not
    have. The name found is logged, so the record says which port was
    read.
    """
    for n in names:
        try:
            return n, getattr(handle, n)
        except AttributeError:
            continue
    raise AttributeError(f"none of {names} under {handle._path}")


def _dll(dut, side):
    """One stack's Data Link Layer instance: pcie_endpoint_top names it
    datalink_layer_inst and pcie_phy_top names it pcie_datalink_layer_inst."""
    return (dut.u_ep.datalink_layer_inst if side == "ep"
            else dut.u_rc.u_phy.pcie_datalink_layer_inst)


class W18Capture:
    """Raw captures on one stack's DLL for the credit tests.

    Three streams, all raw:
      dllp_tx   (cycle, first word) of every DLLP this DLL hands its PHY:
                m_phy_axis with tuser bit 0, axis_user_demux's UserIsDllp
      release   (cycle, DW0) of every TLP handshaken out of dllp2tlp toward the
                configuration block and the TL, stamped at tlast: the handshake
                at which dllp2tlp adds the TLP's credits to CREDITS_ALLOCATED
      alloc_ev  (cycle, value) at every change of the NP-header allocated
                register (dllp2tlp's nph_credits_allocated_o)
    """

    def __init__(self, dut, side):
        """Handles on the stack's DLL, its dllp2tlp and the NP-header allocated port."""
        self.side = side
        self.dll = _dll(dut, side)
        self.d2t = self.dll.dllp_receive_inst.dllp2tlp_inst
        self.alloc_name, self.alloc = _first_attr(
            self.d2t, ("nph_credits_allocated_o", "nph_credits_consumed_o"))
        self.dllp_tx = []
        self.release = []
        self.alloc_ev = []
        self.cycles = 0

    async def run(self, clk, max_cycles, stop):
        """Record the three streams every cycle until stop[0] is set or
        max_cycles pass."""
        dll, d2t, alloc = self.dll, self.d2t, self.alloc
        in_pkt = False
        in_rel = False
        rel_dw0 = None
        prev = None
        for n in range(max_cycles):
            await RisingEdge(clk)
            if stop[0]:
                break
            self.cycles = n
            if int(dll.m_phy_axis_tvalid.value) and int(dll.m_phy_axis_tready.value):
                if not in_pkt and (int(dll.m_phy_axis_tuser.value) & 1):
                    self.dllp_tx.append((n, int(dll.m_phy_axis_tdata.value)))
                in_pkt = not int(dll.m_phy_axis_tlast.value)
            if int(d2t.m_tlp_axis_tvalid.value) and int(d2t.m_tlp_axis_tready.value):
                if not in_rel:
                    rel_dw0 = int(d2t.m_tlp_axis_tdata.value)
                if int(d2t.m_tlp_axis_tlast.value):
                    self.release.append((n, rel_dw0))
                    in_rel = False
                else:
                    in_rel = True
            v = int(alloc.value)
            if v != prev:
                self.alloc_ev.append((n, v))
                prev = v

    # -- derived views, computed after the run, never during it -------------
    def initfc1_np(self):
        """(cycle, type, HdrFC, DataFC) of every InitFC1-NP this DLL sent."""
        return [(c,) + decode_fc_dllp_word(w) for c, w in self.dllp_tx
                if (w & 0xF8) == DLLP_INITFC1_NP]

    def updatefc(self, dllp_type):
        """(cycle, type, HdrFC, DataFC) of every DLLP of dllp_type this DLL sent."""
        return [(c,) + decode_fc_dllp_word(w) for c, w in self.dllp_tx
                if (w & 0xF8) == dllp_type]

    def np_releases(self):
        """Cycles of the released TLPs that consume an NPH credit."""
        return [c for c, dw0 in self.release if is_np_header(decode_tlp_dw0(dw0)[0])]

    def report(self, dut, tag):
        """Log the DLLPs sent by type, the releases by class, the register's
        changes and the UpdateFC-NP and UpdateFC-P lists."""
        types = {}
        for _, w in self.dllp_tx:
            types[w & 0xF8] = types.get(w & 0xF8, 0) + 1
        classes = {}
        for _, dw0 in self.release:
            k = tlp_credit_class(decode_tlp_dw0(dw0)[0])
            classes[k] = classes.get(k, 0) + 1
        dut._log.info("%s %s: sampled %d cycles; register=%s; dllps_tx=%d by type %s",
                      tag, self.side.upper(), self.cycles, self.alloc_name,
                      len(self.dllp_tx), {hex(k): v for k, v in sorted(types.items())})
        dut._log.info("%s %s: releases=%d by class %s; np_releases=%d first=%s last=%s",
                      tag, self.side.upper(), len(self.release), classes,
                      len(self.np_releases()), self.np_releases()[:1],
                      self.np_releases()[-1:])
        dut._log.info("%s %s: alloc_ev n=%d %s", tag, self.side.upper(),
                      len(self.alloc_ev), self.alloc_ev[:20])
        dut._log.info("%s %s: UpdateFC-NP (cycle,type,HdrFC,DataFC) %s", tag,
                      self.side.upper(), self.updatefc(DLLP_UPDATEFC_NP)[:24])
        dut._log.info("%s %s: UpdateFC-P  (cycle,type,HdrFC,DataFC) %s", tag,
                      self.side.upper(), self.updatefc(DLLP_UPDATEFC_P)[:24])


async def _run_w_row(dut, mon, tail=W_TAIL):
    """Bring up, start the raw monitor, enumerate, keep sampling for `tail`."""
    tb, _m, _p, _c, _pa, _s, _d, tasks = await bring_up(dut)
    stop = [False]
    mtask = cocotb.start_soon(mon.run(dut.clk_i, ENUM_CYCLES + tail, stop))
    r = await run_enumeration_fs(dut)
    _log_enum_fs(dut, r)
    await ClockCycles(dut.clk_i, tail)
    stop[0] = True
    await mtask
    for t in tasks:
        await t
    return r


@cocotb.test()
async def fullstack_w1_ep_credits_allocated_advance_on_release(dut):
    """The EP DLL's NP-header CREDITS_ALLOCATED starts at its InitFC
    advertisement and steps by one at each release of an NP TLP toward its
    Transaction Layer, never before.

    CREDITS_ALLOCATED counts the credits granted since initialization and grows
    as the Receiver's Transaction Layer frees buffer space by processing
    received TLPs (PCIe Base Spec r2.1, §2.6.1.2). In this DLL the release point
    is dllp2tlp's m_tlp_axis handshake at tlast, when the TLP leaves the DLL's
    receive FIFO for pcie_cfg_wrapper and the TL.

    Three raw captures on the EP's DLL are paired after the run: the EP's
    InitFC1-NP, decoded for the advertised HdrFC (which must equal
    HDR_MIN_CREDITS); every TLP released from dllp2tlp, classified by its DW0;
    and every change of nph_credits_allocated_o. The register's first value is
    the advertisement. For each later step at cycle c to value v,
    (v - advertised) must equal the number of NP releases strictly before c: a
    step ahead of its release counts buffer space as free while the TLP still
    holds it, so the Transmitter could send, within the advertised credit, a
    TLP the Receiver has no room for. The final value is the advertisement
    plus all NP releases. Non-vacuity: at least two NP releases.
    """
    w_selftest()
    cap = W18Capture(dut, "ep")
    await _run_w_row(dut, cap)
    cap.report(dut, "W1")

    init = cap.initfc1_np()
    assert init, ("NON-VACUITY: the Endpoint's DLL transmitted no InitFC1-NP, so "
                  "there is no advertisement to compare the register against")
    advertised = init[0][2]
    assert advertised == HDR_MIN_CREDITS, (
        f"the EP advertised HdrFC={advertised} in InitFC1-NP; pcie_datalink_pkg's "
        f"HdrMinCredits is {HDR_MIN_CREDITS} -- the wire disagrees with the constant")

    np_rel = cap.np_releases()
    assert len(np_rel) >= 2, (
        f"NON-VACUITY: {len(np_rel)} NP TLPs were released by the EP's DLL; at least "
        "two are needed to see the register STEP rather than merely hold a value")
    assert cap.alloc_ev, "the allocated register was never sampled"
    assert cap.alloc_ev[0][1] == advertised, (
        f"CREDITS_ALLOCATED must start at the InitFC advertisement ({advertised}); "
        f"the register's first value was {cap.alloc_ev[0][1]}")

    ahead = []
    for c, v in cap.alloc_ev[1:]:
        rel_before = sum(1 for r in np_rel if r < c)
        if ((v - advertised) & 0xFF) != rel_before:
            ahead.append((c, v, rel_before))
    dut._log.info("W1 VERDICT: %d register steps, %d NP releases, %d steps not "
                  "explained by prior releases: %s", len(cap.alloc_ev) - 1,
                  len(np_rel), len(ahead), ahead[:8])
    assert not ahead, (
        f"{len(ahead)} of {len(cap.alloc_ev) - 1} steps of {cap.alloc_name} are "
        f"not explained by the NP releases before them -- first: at cycle "
        f"{ahead[0][0]} the register read {ahead[0][1]} (= advertised + "
        f"{(ahead[0][1] - advertised) & 0xFF}) with only {ahead[0][2]} NP TLPs "
        "released. CREDITS_ALLOCATED counted buffer space as available before "
        "the TLP occupying it had been processed (Base 2.1 §2.6.1.2 p.141)")
    final = (cap.alloc_ev[-1][1] - advertised) & 0xFF
    assert final == len(np_rel), (
        f"final CREDITS_ALLOCATED - advertised = {final}, but {len(np_rel)} NP "
        "TLPs were released")


@cocotb.test()
async def fullstack_w2_ep_updatefc_np_scheduled_on_release(dut):
    """After the EP's DLL releases NP credit it promptly sends an UpdateFC-NP
    carrying it; the last UpdateFC-NP of the run carries every release; and no
    UpdateFC-NP advertises more than the releases before it.

    Once every advertised unit of a non-infinite NPH, NPD, PH or CPLH type has
    been consumed, an UpdateFC must be scheduled when processed TLPs free one
    or more units of that type, and UpdateFCs may be sent more often (PCIe
    Base Spec r2.1, §2.6.1.2). This test holds the EP's DLL to an UpdateFC-NP
    after every NP release, whether or not the advertisement was used up. The
    30 us periodic UpdateFC in the same section is a separate rule, tested by
    the periodic-UpdateFC tests. With the captures of the CREDITS_ALLOCATED
    test (W18Capture), UpdateFC-NP DLLPs are decoded for HdrFC and DataFC and
    paired with the NP release cycles:
      1. after the first NP release, some UpdateFC-NP carries HdrFC above the
         advertisement;
      2. the last UpdateFC-NP carries the advertisement plus all releases;
      3. no UpdateFC-NP's HdrFC exceeds the advertisement plus the releases
         before it;
      4. HdrFC never decreases (modulo 256);
      5. the last DataFC carries the data credits of every NPD release;
      6. each release is carried by an UpdateFC-NP within W2_RELEASE_BOUND
         cycles, far shorter than the periodic interval.

    Posted credits work the same way, but enumeration sends the EP no posted
    TLP, so UpdateFC-P is logged, not asserted.
    """
    w_selftest()
    cap = W18Capture(dut, "ep")
    await _run_w_row(dut, cap)
    cap.report(dut, "W2")

    init = cap.initfc1_np()
    assert init, "NON-VACUITY: no InitFC1-NP transmitted by the EP's DLL"
    advertised = init[0][2]
    np_rel = cap.np_releases()
    assert np_rel, "NON-VACUITY: the EP's DLL released no NP TLP, so no credit was owed"
    upd = cap.updatefc(DLLP_UPDATEFC_NP)
    assert upd, ("NON-VACUITY: no UpdateFC-NP was transmitted at all -- the update "
                 "machinery never ran, so this row would be measuring its absence "
                 "rather than its trigger")

    after_first = [u for u in upd if u[0] > np_rel[0]]
    carrying = [u for u in after_first if ((u[2] - advertised) & 0xFF) >= 1]
    dut._log.info("W2 VERDICT: advertised=%d np_releases=%d updatefc_np=%d "
                  "(after first release: %d, carrying released credit: %d) "
                  "last_hdrfc=%s", advertised, len(np_rel), len(upd),
                  len(after_first), len(carrying), upd[-1][2])
    assert carrying, (
        f"{len(np_rel)} NP credits were released by the EP's DLL (first at cycle "
        f"{np_rel[0]}) and NO UpdateFC-NP transmitted afterwards carries any of "
        f"them: {len(upd)} UpdateFC-NP on the wire, HdrFC values "
        f"{[u[2] for u in upd]}, all equal to the InitFC advertisement "
        f"{advertised}. Base 2.1 §2.6.1.2 p.142 requires an UpdateFC to be "
        "scheduled each time one or more units are made available by TLPs "
        "processed. This is #18: the Root Complex's CREDIT_LIMIT never moves")
    assert ((upd[-1][2] - advertised) & 0xFF) == len(np_rel), (
        f"the last UpdateFC-NP advertises {upd[-1][2]} = advertised + "
        f"{(upd[-1][2] - advertised) & 0xFF}, but {len(np_rel)} NP credits were "
        "released -- credit is still owed at the end of the run")
    over = [(c, h, sum(1 for r in np_rel if r < c)) for c, _, h, _ in upd
            if ((h - advertised) & 0xFF) > sum(1 for r in np_rel if r < c)]
    assert not over, (
        f"{len(over)} UpdateFC-NP advertised more credit than had been released "
        f"before it (cycle, HdrFC, releases_before): {over[:6]}")
    hdrs = [u[2] for u in upd]
    assert all(((b - a) & 0xFF) < 0x80 for a, b in zip(hdrs, hdrs[1:])), (
        f"UpdateFC-NP HdrFC went backwards: {hdrs}")

    # The data half of the same rule: each NPD release returns
    # Roundup(Length / 4) data credits (PCIe Base Spec r2.1, Table 2-36), and
    # Length 0 means 1024 DW, 256 credits. The expectation is computed from
    # each captured Length.
    init_data = init[0][3]
    npd_credits = 0
    for _, dw0 in cap.release:
        ft, length = decode_tlp_dw0(dw0)
        if tlp_credit_class(ft) == "NPD":
            npd_credits += 256 if length == 0 else (length + 3) // 4
    dut._log.info("W2 DATAFC: advertised=%d npd_credits_released=%d last_datafc=%d",
                  init_data, npd_credits, upd[-1][3])
    assert ((upd[-1][3] - init_data) & 0xFFF) == npd_credits, (
        f"the last UpdateFC-NP advertises DataFC {upd[-1][3]} = advertised + "
        f"{(upd[-1][3] - init_data) & 0xFFF}, but {npd_credits} NPD credits were "
        "released -- the data half of the advertisement did not follow the header half")

    # The periodic UpdateFC-NP, about every UFC_NOMINAL cycles, also carries
    # CREDITS_ALLOCATED as it stands, so it can satisfy the checks above with
    # no release trigger at all. So each release must be carried by an
    # UpdateFC-NP within W2_RELEASE_BOUND cycles, far below the periodic
    # interval.
    lat = []
    for i, rel in enumerate(np_rel):
        need = (advertised + i + 1) & 0xFF
        carrier = next((u for u in upd if u[0] > rel and ((u[2] - need) & 0xFF) < 0x80), None)
        lat.append(None if carrier is None else carrier[0] - rel)
    dut._log.info("W2 LATENCY: release -> first UpdateFC-NP carrying it, cycles: %s", lat)
    late = [(np_rel[i], l) for i, l in enumerate(lat) if l is None or l > W2_RELEASE_BOUND]
    assert not late, (
        f"{len(late)} NP releases were not carried by an UpdateFC-NP within "
        f"{W2_RELEASE_BOUND} cycles (release cycle, latency): {late[:6]}. p.142's "
        "release clause schedules the UpdateFC when the credit is made available; "
        "a periodic refresh arriving later is not that")


@cocotb.test()
async def fullstack_w3_rc_never_credit_blocked(dut):
    """err_credit_blocked_o never rises while the RC enumerates.

    pcie_rc_top's err_credit_blocked_o is the enumeration engine's note that a
    completion timeout was reported while tx_fc_blocked was high, that is,
    while the request sat behind the Transaction Layer's credit gate. It is
    sampled every cycle from bring-up until W_TAIL cycles after enumeration
    returns. The RC credit manager's remaining NP-header credit
    (nonposted_header_available_o) is logged from FC init on: its minimum, the
    cycles at zero and the refills.

    Non-vacuity is a count: at least HDR_MIN_CREDITS non-posted requests (the
    EP's NP-header advertisement) reached the RC's DLL, and either more did or
    tx_fc_blocked_o was high at some point, so the advertised pool was used up
    at least once. Requests are counted at the DLL input, downstream of the
    credit gate (tlp_credit_manager, in the Transaction Layer), which a starved
    request never reaches; hence the tx_fc_blocked_o alternative.
    """
    w_selftest()

    class W3Capture:
        """Raw per-cycle capture for the credit-starvation test."""
        def __init__(self, dut):
            """Handles on the RC's error annotation, its credit gate, its credit
            manager and its DLL input."""
            self.blocked = dut.u_rc.err_credit_blocked_o
            self.fcblk = dut.u_rc.tx_fc_blocked_o
            self.rc = _dll(dut, "rc")
            # The RC credit manager's live NP-header remainder: logged (minimum,
            # cycles at zero, refills), never asserted.
            self.avail = dut.u_rc.u_tl.u_tlp_layer.credit_manager_inst.nonposted_header_available_o
            # Counted from rc_fc_initialized_o on: before FC init the limit is 0,
            # so the remainder reads 0 through bring-up and a minimum taken from
            # cycle 0 would say nothing about refills.
            self.fc_init = dut.rc_fc_initialized_o
            self.avail_min = None
            self.avail_zero_cycles = 0
            self.avail_refills = 0
            self.blocked_cycles = []
            self.fcblk_cycles = 0
            self.np_requests = []
            self.cycles = 0

        async def run(self, clk, max_cycles, stop):
            """Sample every cycle until stop[0] is set or max_cycles pass."""
            rc = self.rc
            in_pkt = False
            prev_avail = None
            for n in range(max_cycles):
                await RisingEdge(clk)
                if stop[0]:
                    break
                self.cycles = n
                if int(self.blocked.value):
                    self.blocked_cycles.append(n)
                if int(self.fcblk.value):
                    self.fcblk_cycles += 1
                a = int(self.avail.value)
                if int(self.fc_init.value):
                    if self.avail_min is None or a < self.avail_min:
                        self.avail_min = a
                    if a == 0:
                        self.avail_zero_cycles += 1
                    if prev_avail is not None and a > prev_avail:
                        self.avail_refills += 1
                    prev_avail = a
                if int(rc.s_tlp_axis_tvalid.value) and int(rc.s_tlp_axis_tready.value):
                    if not in_pkt:
                        ft = decode_tlp_dw0(int(rc.s_tlp_axis_tdata.value))[0]
                        if is_np_header(ft):
                            self.np_requests.append(n)
                    in_pkt = not int(rc.s_tlp_axis_tlast.value)

    cap = W3Capture(dut)
    r = await _run_w_row(dut, cap)
    dut._log.info("W3 VERDICT: np_requests=%d err_credit_blocked cycles=%d (first %s) "
                  "tx_fc_blocked cycles=%d enum_done=%s enum_error=%s code=%s "
                  "bar_count=%s | nph_available (post FC init) min=%s zero_cycles=%d refills=%d",
                  len(cap.np_requests), len(cap.blocked_cycles),
                  cap.blocked_cycles[:1], cap.fcblk_cycles, r["enum_done"],
                  r["enum_error"], r["enum_error_code"], r["bar_count"],
                  cap.avail_min, cap.avail_zero_cycles, cap.avail_refills)
    assert len(cap.np_requests) >= HDR_MIN_CREDITS, (
        f"NON-VACUITY: only {len(cap.np_requests)} non-posted requests reached "
        f"the RC's DLL, fewer than the {HDR_MIN_CREDITS} header credits the EP "
        "advertises at FC init -- the credit pool was never even exhausted, so "
        "starvation could not have occurred and this row asserted nothing")
    assert len(cap.np_requests) > HDR_MIN_CREDITS or cap.fcblk_cycles > 0, (
        f"NON-VACUITY: exactly {HDR_MIN_CREDITS} non-posted requests reached the "
        "DLL and tx_fc_blocked_o was never high -- no 17th request was attempted, "
        "so nothing could have starved")
    assert not cap.blocked_cycles, (
        f"err_credit_blocked_o asserted at cycle {cap.blocked_cycles[0]} (high for "
        f"{len(cap.blocked_cycles)} sampled cycles; tx_fc_blocked_o high for "
        f"{cap.fcblk_cycles}) after {len(cap.np_requests)} non-posted requests: "
        f"the RC ran dry of NP header credit. enum_error={r['enum_error']} "
        f"code={r['enum_error_code']} bar_count={r['bar_count']}")


# ---------------------------------------------------------------------------
# Replay
# ---------------------------------------------------------------------------
# A TLP is replayed when a Nak arrives or when REPLAY_TIMER expires before its
# Ack, and a Receiver discards a duplicate (PCIe Base Spec r2.1, §3.5.2.1,
# §3.5.3.1). With the bridge's injector and blackouts off, the bridge passes
# every Symbol unchanged, so neither replay machine should fire. W4Capture
# records the RC's inbound link TLPs with their sequence numbers, the EP's
# transmitted sequence numbers, both stacks' replay strobes and the COM
# Symbols on the RC's PIPE TX; all pairing is done after the run.
@cocotb.test()
async def fullstack_w4_ep_does_not_replay_every_tlp(dut):
    """No DLL sequence number arrives at the RC's DLL more than once, and
    neither stack's replay machine fires.

    Replay is a recovery path: a TLP is replayed when its Ack does not arrive
    in time (PCIe Base Spec r2.1, §3.5.2.1). With no DLLP lost, every inbound
    sequence number must be distinct, and the EP's and the RC's retry_valid_o
    must never rise.

    Captured raw and paired after the run:
      rc_rx      (cycle, first word) of every inbound link TLP at the RC's
                 dllp2tlp input; the sequence number is in that word
                 (known answer 0x004A0000 -> sequence 0, CplD)
      ep_tx      (cycle, seq) of every TLP the EP's retry_management takes
      ep_replay  (cycle, mask) at every rise of the EP's retry_valid_o
      rc_replay  the same on the RC
      com        cycles with a K28.5 on the RC's PIPE TX, in a COM_WINDOW
                 opened when rc_fc_initialized_o rises (logged only)
    The first inbound TLP must decode to sequence 0 and Fmt/Type 4Ah: the EP's
    first TLP is the CplD to the first CfgRd0. If it does not, the decoder or
    the traffic has changed and the rest of the check is not trusted.
    """
    w_selftest()

    class W4Capture:
        """Raw per-cycle capture for the replay test."""
        def __init__(self, dut):
            """Handles on the RC's dllp2tlp and both retry_management instances."""
            self.dut = dut
            rc, ep = _dll(dut, "rc"), _dll(dut, "ep")
            self.rc_d2t = rc.dllp_receive_inst.dllp2tlp_inst
            self.ep_rm = ep.dllp_transmit_inst.retry_management_inst
            self.rc_rm = rc.dllp_transmit_inst.retry_management_inst
            self.rc_rx = []
            self.ep_tx = []
            self.ep_replay = []
            self.rc_replay = []
            self.com = []
            self.com_open = None
            self.cycles = 0

        async def run(self, clk, max_cycles, stop):
            """Record inbound first words, EP sends, replay rises and COM cycles
            until stop[0] is set or max_cycles pass."""
            d, d2t, eprm, rcrm = self.dut, self.rc_d2t, self.ep_rm, self.rc_rm
            in_pkt = False
            ep_prev = rc_prev = 0
            for n in range(max_cycles):
                await RisingEdge(clk)
                if stop[0]:
                    break
                self.cycles = n
                if int(d2t.s_axis_tvalid.value) and int(d2t.s_axis_tready.value):
                    if not in_pkt:
                        self.rc_rx.append((n, int(d2t.s_axis_tdata.value)))
                    in_pkt = not int(d2t.s_axis_tlast.value)
                if int(eprm.tx_valid_i.value):
                    self.ep_tx.append((n, int(eprm.tx_seq_num_i.value)))
                ev = int(eprm.retry_valid_o.value)
                if ev & ~ep_prev:
                    self.ep_replay.append((n, ev & ~ep_prev))
                ep_prev = ev
                rv = int(rcrm.retry_valid_o.value)
                if rv & ~rc_prev:
                    self.rc_replay.append((n, rv & ~rc_prev))
                rc_prev = rv
                if self.com_open is None:
                    if int(d.rc_fc_initialized_o.value):
                        self.com_open = n
                elif n - self.com_open < COM_WINDOW:
                    if int(d.rc_phy_txdata_valid.value):
                        k = int(d.rc_phy_txdatak.value)
                        w = int(d.rc_phy_txdata.value)
                        for b in range(2):   # 16-bit PIPE at Gen1: 2 Symbols/beat
                            if (k >> b) & 1 and ((w >> (8 * b)) & 0xFF) == K_COM:
                                self.com.append(n)
                                break

    cap = W4Capture(dut)
    await _run_w_row(dut, cap)

    seqs = [decode_link_first_word(w) for _, w in cap.rc_rx]
    by_seq = {}
    for (n, _), (s, ft) in zip(cap.rc_rx, seqs):
        by_seq.setdefault(s, []).append(n)
    dups = {s: c for s, c in by_seq.items() if len(c) > 1}
    spacing = sorted(c[1] - c[0] for c in dups.values())
    com_h = gap_histogram(cap.com)
    top = sorted(com_h.items(), key=lambda kv: -kv[1])[:4]
    dut._log.info("W4 rc_rx=%d distinct_seq=%d duplicated_seq=%d dup_spacing(min/median/max)=%s "
                  "| ep_tx=%d ep_replays=%d rc_replays=%d | com_window_open=%s "
                  "com_events=%d top_gaps=%s", len(cap.rc_rx), len(by_seq), len(dups),
                  (spacing[0], spacing[len(spacing) // 2], spacing[-1]) if spacing else None,
                  len(cap.ep_tx), len(cap.ep_replay), len(cap.rc_replay), cap.com_open,
                  len(cap.com), top)
    dut._log.info("W4 first rc_rx words: %s", [(n, hex(w)) for n, w in cap.rc_rx[:6]])

    # Non-vacuity, then a check that the decoder reads the first TLP as expected.
    assert len(cap.rc_rx) >= 2, "NON-VACUITY: fewer than two inbound TLPs at the RC's DLL"
    assert seqs[0] == (0, 0x4A), (
        f"the first inbound TLP at the RC decodes to seq={seqs[0][0]} fmt_type="
        f"{seqs[0][1]:#04x}; #7e/#7f measured the EP's first TLP as the CplD to "
        "the first CfgRd0 with DLL sequence 0 (first word 0x004A0000). Either the "
        "decoder or the link has changed and the rest of this verdict is unsafe")
    verdict = ("GREEN -- no duplicate sequence number and no EP replay; C17 LOST, "
               "report it" if not dups and not cap.ep_replay else
               f"RED -- {len(dups)} of {len(by_seq)} sequence numbers arrived twice, "
               f"{len(cap.ep_replay)} EP replays for {len(cap.ep_tx)} TLPs, "
               f"{len(cap.rc_replay)} RC replays")
    dut._log.info("W4 VERDICT: %s", verdict)
    dut._log.info("W4 dups=%d ep_replays=%d rc_replays=%d",
                  len(dups), len(cap.ep_replay), len(cap.rc_replay))
    assert not dups and not cap.ep_replay, (
        f"{len(dups)} of {len(by_seq)} DLL sequence numbers arrived at the RC's DLL "
        f"more than once (duplicate spacing {spacing[:4]} cycles); the EP's replay "
        f"machine fired {len(cap.ep_replay)} times for {len(cap.ep_tx)} TLPs and the "
        f"RC's {len(cap.rc_replay)} times. Base 2.1 §3.5.2.1: replay is recovery, "
        "not steady state. #21 -> #7h")
    # The RC's replay machine must not fire either: its REPLAY_TIMER is 622
    # cycles (replay_timer_cycles(128, 1, 8)), and on this link every RC TLP
    # must be Acked before that.
    assert not cap.rc_replay, (
        f"the RC's replay machine fired {len(cap.rc_replay)} times on a clean link "
        f"({len(cap.rc_replay)} of its TLPs outlived the REPLAY_TIMER without an Ack)")


# ---------------------------------------------------------------------------
# L0 transmit: valid, Logical Idle and SKP spacing
# ---------------------------------------------------------------------------
# In L0 a transmitter with nothing to send is in Logical Idle and sends the
# Idle Symbol 00h, and SKP Ordered Sets continue during idle (PCIe Base Spec
# r2.1, §4.2.2, §4.2.7.1). Here the idle and ordered-set requests come from
# the real LTSSMs in L0, with two PHYs facing each other through the bridge;
# tb/phy_tx_golden/test_7j2_idle.py drives the same requests from its own
# bench at the phy_transmit boundary instead. Each test samples every cycle
# from before bring-up and opens its window on the EP LTSSM's longest
# contiguous run in ST_L0 (_longest_run), plus SETTLE cycles.

def _l0_window(events, first, last):
    """The (cycle, ...) events with first <= cycle <= last."""
    return [e for e in events if first <= e[0] <= last]


def _longest_run(samples, want):
    """Longest contiguous run of `want` in [(cycle, value)], as (first, last,
    len); (None, None, 0) when `want` never occurs.

    Taking min() and max() of the cycles that read `want` would let a single
    stray sample before the link is up stretch the window back over training.
    The longest contiguous run cannot be moved by an outlier. The valid test
    and gth81_w4_check also require it to hold more than 90% of the samples
    of the state.
    """
    best = (None, None, 0)
    cur_first, cur_len = None, 0
    for n, v in samples:
        if v == want:
            if cur_first is None:
                cur_first = n
            cur_len += 1
            if cur_len > best[2]:
                best = (cur_first, n, cur_len)
        else:
            cur_first, cur_len = None, 0
    return best


@cocotb.test()
async def fullstack_7j2_pipe_tx_valid_never_drops_in_l0(dut):
    """In L0 the PIPE TX valid never drops, on either stack.

    In Logical Idle a transmitter sends idle data (PCIe Base Spec r2.1, §4.2.2),
    so every Symbol Time carries a Symbol and valid stays high. The RC's valid
    is rc_phy_txdata_valid; the EP's is the bench's ep_tx_symbol_valid.

    The window opens on the EP LTSSM's ST_L0, not on link_up: link_up is also
    high in Configuration.Idle (pcie_ltssm_downstream), so a window opened on
    link_up would include that state. Samples are taken every cycle from before
    that point rather than started when the state is seen. Non-vacuity: the
    longest ST_L0 run is longer than 4 x SETTLE cycles and holds more than 90%
    of all ST_L0 samples.
    """
    ST_L0 = 0x00005
    SETTLE = 64

    rc_v, ep_v, state = [], [], []
    done = False

    async def sample():
        """Record RC valid, EP valid and EP LTSSM state every cycle until done."""
        n = 0
        while not done:
            await RisingEdge(dut.clk_i)
            n += 1
            rc_v.append((n, int(dut.rc_phy_txdata_valid.value) & 1))
            ep_v.append((n, int(dut.ep_tx_symbol_valid.value) & 1))
            state.append((n, int(dut.ep_ltssm_state_o.value)))

    task = cocotb.start_soon(sample())
    await bring_up(dut)
    await ClockCycles(dut.clk_i, WINDOW)
    done = True
    await ClockCycles(dut.clk_i, 2)
    task.kill()

    n_l0 = sum(1 for _, s in state if s == ST_L0)
    run_first, run_last, run_len = _longest_run(state, ST_L0)
    dut._log.info("7J2[acc-a] samples=%d  ST_L0 cycles=%d  longest contiguous "
                  "run=[%s, %s] len=%d", len(state), n_l0,
                  run_first, run_last, run_len)
    assert run_len > SETTLE * 4, (
        f"NON-VACUITY: the EP LTSSM's longest unbroken stay in ST_L0 "
        f"(0x{ST_L0:05x}) was {run_len} cycles; there is no L0 window to "
        f"measure valid in")
    assert run_len > 0.9 * n_l0, (
        f"NON-VACUITY: the longest contiguous ST_L0 run is {run_len} cycles "
        f"but {n_l0} samples read ST_L0 in total -- the state is not settled "
        f"and this window is not the one this row means")

    first, last = run_first + SETTLE, run_last
    rc_low = [n for n, v in _l0_window(rc_v, first, last) if not v]
    ep_low = [n for n, v in _l0_window(ep_v, first, last) if not v]
    dut._log.info("7J2[acc-a] L0 window [%d, %d] = %d cycles | RC valid-low %d "
                  "| EP valid-low %d", first, last, last - first + 1,
                  len(rc_low), len(ep_low))
    assert not rc_low, (
        f"RC PIPE TX valid dropped on {len(rc_low)} cycles inside L0 "
        f"(first at {rc_low[:8]}); those Symbol Times carried no Symbol, "
        f"against Base 2.1 §4.2.2 p.195")
    assert not ep_low, (
        f"EP PIPE TX valid dropped on {len(ep_low)} cycles inside L0 "
        f"(first at {ep_low[:8]})")


@cocotb.test()
async def fullstack_7j2_l0_stream_descrambles_to_packets_and_idle(dut):
    """An independent Python model descrambles the RC's L0 transmit stream to
    packets and Idle Symbols 00h, with no residue.

    The model is rx_golden.Descrambler, written from PCIe Base Spec r2.1,
    §4.2.3 and Appendix C.1, sharing no code with the RTL. Every Symbol the RC
    transmits with valid high inside the L0 window is descrambled from the
    first COM on; outside a packet (STP or SDP up to END or EDB) every data
    Symbol must be 00h. Residue would mean the transmitter sends something that
    is neither a packet nor Logical Idle (§4.2.2).

    A link that drops valid between packets leaves almost no Symbols in a
    valid-gated window, so it fails the non-vacuity check (more than 1,000
    Symbols) rather than being judged on residue in a stream it did not send.
    At least one Idle Symbol must also be seen.
    """
    import rx_golden

    ST_L0 = 0x00005
    SETTLE = 64
    COM_B, SKP_B, STP_B, SDP_B, END_B, EDB_B = 0xBC, 0x1C, 0xFB, 0x5C, 0xFD, 0xFE

    syms, state, done = [], [], False

    async def sample():
        """Record the EP LTSSM state every cycle, and the RC's two transmitted
        Symbols with their K flags on every valid cycle, until done."""
        n = 0
        while not done:
            await RisingEdge(dut.clk_i)
            n += 1
            state.append((n, int(dut.ep_ltssm_state_o.value)))
            if int(dut.rc_phy_txdata_valid.value) & 1:
                w = int(dut.rc_phy_txdata.value)
                k = int(dut.rc_phy_txdatak.value)
                # PHY_DATA_WIDTH is 16 at gen1: two Symbols per beat, lane 0.
                for b in range(2):
                    syms.append((n, (w >> (8 * b)) & 0xFF, (k >> b) & 1))

    task = cocotb.start_soon(sample())
    await bring_up(dut)
    await ClockCycles(dut.clk_i, WINDOW)
    done = True
    await ClockCycles(dut.clk_i, 2)
    task.kill()

    run_first, run_last, run_len = _longest_run(state, ST_L0)
    assert run_len > SETTLE * 4, (
        f"NON-VACUITY: the EP LTSSM's longest unbroken stay in ST_L0 was "
        f"{run_len} cycles")
    first, last = run_first + SETTLE, run_last
    window = [(b, k) for n, b, k in syms if first <= n <= last]
    dut._log.info("7J2[acc-b] L0 window [%d, %d]; %d Symbols transmitted",
                  first, last, len(window))
    assert len(window) > 1000, (
        f"NON-VACUITY: only {len(window)} Symbols were TRANSMITTED in a "
        f"{last - first + 1}-cycle L0 window.  Valid-gated, a link that goes "
        f"quiet between packets has almost nothing in it -- which is the "
        f"defect, not a measurement of residue.")

    # The model starts at the first COM in the window, which resets its LFSR as
    # it resets the DUT's: at an arbitrary offset the LFSR state is unknown, and
    # a model started there would report residue of its own.
    try:
        start = next(i for i, (b, k) in enumerate(window) if k and b == COM_B)
    except StopIteration:
        raise AssertionError(
            "NON-VACUITY: no COM in the L0 window, so the model has no point "
            "at which its LFSR is known to agree with the DUT's")

    d = rx_golden.Descrambler()
    in_pkt, residue, idle, pkt, ctrl = False, [], 0, 0, 0
    for b, k in window[start:]:
        out = d.symbol(b, k, in_ts=False)
        if k:
            ctrl += 1
            if out in (STP_B, SDP_B):
                in_pkt = True
            elif out in (END_B, EDB_B):
                in_pkt = False
            continue
        if in_pkt:
            pkt += 1
        elif out == 0x00:
            idle += 1
        else:
            residue.append(out)

    total = pkt + idle + len(residue)
    dut._log.info("7J2[acc-b] from COM@%d: control=%d packet=%d idle00=%d "
                  "residue=%d (%.3f %%) first_residue=%s",
                  start, ctrl, pkt, idle, len(residue),
                  100.0 * len(residue) / max(total, 1),
                  [hex(x) for x in residue[:8]])
    assert idle > 0, "NON-VACUITY: not one Idle Symbol 00h in the whole L0 window"
    assert not residue, (
        f"{len(residue)} of {total} descrambled data Symbols outside a packet "
        f"were not the Idle Symbol 00h (first: {[hex(x) for x in residue[:8]]}). "
        f"The Transmitter is emitting something that is neither a packet nor "
        f"Logical Idle.")


@cocotb.test()
async def fullstack_7j2_skp_keeps_its_spec_spacing_in_l0(dut):
    """In L0, SKP Ordered Sets keep their spacing and none is placed inside a
    packet.

    A SKP Ordered Set is scheduled every 1180 to 1538 Symbol Times; one that
    falls due during a packet is held and sent at the next packet or Ordered
    Set boundary (PCIe Base Spec r2.1, §4.2.7.1), and SKP continues during idle
    data (§4.2.2). Here pcie_ltssm_downstream drives the ordered-set request in
    L0. os_generator restarts its SKP interval counter (skp_cnt) in ST_IDLE
    whenever an ordered-set request is valid, so an LTSSM that kept the
    request valid at every return to ST_IDLE would keep skp_cnt below
    SkpIntervalCounts and starve the schedule; the count check catches that.

    Asserted: at least three SKP Ordered Sets in the window; no SKP Symbol
    between a packet's STP or SDP and its END or EDB; and the median interval
    inside [1180, 1538]. The median is used because a deferred SKP legitimately
    lengthens single gaps, and a Receiver need only tolerate that range as an
    average (§4.2.7.2). Every gap is logged.
    """
    ST_L0, SETTLE = 0x00005, 64
    COM_B, SKP_B, STP_B, SDP_B, END_B, EDB_B = 0xBC, 0x1C, 0xFB, 0x5C, 0xFD, 0xFE
    SPEC_LO, SPEC_HI = 1180, 1538          # Symbol Times, PCIe Base Spec r2.1, §4.2.7.1

    syms, state, done = [], [], False

    async def sample():
        """Record the EP LTSSM state every cycle, and the RC's two transmitted
        Symbols with their K flags on every valid cycle, until done."""
        n = 0
        while not done:
            await RisingEdge(dut.clk_i)
            n += 1
            state.append((n, int(dut.ep_ltssm_state_o.value)))
            if int(dut.rc_phy_txdata_valid.value) & 1:
                w = int(dut.rc_phy_txdata.value)
                k = int(dut.rc_phy_txdatak.value)
                for b in range(2):          # 16-bit PIPE at gen1: 2 Symbols/beat
                    syms.append((n, (w >> (8 * b)) & 0xFF, (k >> b) & 1))

    task = cocotb.start_soon(sample())
    await bring_up(dut)
    await ClockCycles(dut.clk_i, WINDOW)
    done = True
    await ClockCycles(dut.clk_i, 2)
    task.kill()

    run_first, run_last, run_len = _longest_run(state, ST_L0)
    assert run_len > SETTLE * 4, (
        f"NON-VACUITY: the EP LTSSM's longest unbroken stay in ST_L0 was "
        f"{run_len} cycles")
    first, last = run_first + SETTLE, run_last

    # One index per transmitted Symbol: the Symbols are valid-gated, so a cycle
    # that carried no Symbol adds nothing to an interval.
    window = [(b, k) for n, b, k in syms if first <= n <= last]
    skp_at, in_pkt, skp_in_pkt = [], False, []
    for t, (b, k) in enumerate(window):
        if k and b == SKP_B:
            skp_at.append(t)
            if in_pkt:
                skp_in_pkt.append(t)
        elif k and b in (STP_B, SDP_B):
            in_pkt = True
        elif k and b in (END_B, EDB_B):
            in_pkt = False

    # One Ordered Set is COM + three SKP, so group consecutive SKP Symbols.
    starts = [t for i, t in enumerate(skp_at) if i == 0 or t - skp_at[i - 1] > 8]
    gaps = [b - a for a, b in zip(starts, starts[1:])]
    gaps_sorted = sorted(gaps)
    median = gaps_sorted[len(gaps_sorted) // 2] if gaps_sorted else None
    dut._log.info("7J2[skp] L0 window [%d, %d] = %d Symbol Times | SKP OS=%d "
                  "| gaps min/median/max=%s/%s/%s | inside-packet=%d",
                  first, last, len(window), len(starts),
                  gaps_sorted[0] if gaps_sorted else None, median,
                  gaps_sorted[-1] if gaps_sorted else None, len(skp_in_pkt))

    # (i) the schedule runs at all.
    assert len(starts) >= 3, (
        f"NON-VACUITY / SCHEDULE STARVED: only {len(starts)} SKP Ordered Sets "
        f"in {len(window)} Symbol Times of L0.  Base 2.1 §4.2.7.1 p.261 "
        f"schedules one every 1180-1538, so a window this long must carry "
        f"several.  §4.2.2 p.195: the SKP Ordered Set must continue to be "
        f"transmitted during idle data.")

    # (ii) a SKP is never placed between a packet's STP and its END.
    assert not skp_in_pkt, (
        f"{len(skp_in_pkt)} SKP Symbols were transmitted INSIDE a packet "
        f"(Symbol Times {skp_in_pkt[:8]}).  Base 2.1 §4.2.7.1 p.261: a "
        f"scheduled SKP is inserted at the next packet boundary, never within "
        f"one -- foreign Symbols inside a packet make it undecodable.")

    # (iii) and the spacing is the spec's.
    assert SPEC_LO <= median <= SPEC_HI, (
        f"median SKP interval is {median} Symbol Times, outside Base 2.1 "
        f"§4.2.7.1 p.261's [{SPEC_LO}, {SPEC_HI}] window (all gaps: "
        f"{gaps_sorted[:12]})")


# ---------------------------------------------------------------------------
# LCRC, sequence and nullified-TLP injection
# ---------------------------------------------------------------------------
# These tests corrupt the wire between the two stacks and let the Endpoint's
# receive path react. The injector is pipe_codec_bridge's, on the RC -> EP
# direction before encoding, so an altered byte is still a legal symbol:
# INJ_FLIP flips one bit of the Nth beat after STP; INJ_NULLIFY turns END into
# EDB and inverts the four LCRC bytes, a nullified TLP as a transmitter makes
# one (PCIe Base Spec r2.1, §3.5.2.1); INJ_EDB_BAD turns END into EDB and
# leaves the LCRC alone. Each test arms it through the bench's inj_* signals,
# runs one enumeration and disarms it. No transmitter in either stack emits
# EDB (only data_handler, on the receive side, detects it), so the injector is
# the only source of nullified and EDB-terminated frames.

INJ_FLIP, INJ_NULLIFY, INJ_EDB_BAD = 0, 1, 2   # pipe_codec_bridge inj_mode_i values

# LCRC_END_BYTE is the expected position of the armed packet's last LCRC byte,
# counted in data bytes from its first, which the bridge reports on
# inj_end_byte_o. 21 makes the packet 22 bytes, with the LCRC in bytes 18 to
# 21. The LCRC is the four data bytes before END and is not beat-aligned, so
# INJ_NULLIFY addresses it by data byte, not by beat. The nullified-TLP
# test asserts the reported position, so a packet of another length fails
# that test rather than passing with other bytes inverted.
LCRC_END_BYTE   = 21
LCRC_FIRST_BYTE = LCRC_END_BYTE - 3
TARGET_PKT      = 3   # the third STP-framed packet the RC sends after arming


async def _run_injected(dut, mode, off=0, bit=0, pkt=TARGET_PKT):
    """Arm the bridge injector, enumerate, and return the EP DLL's observations.

    Everything is read from dllp2tlp's own registers rather than recomputed:
    response_is_nak_r is its latched Nak decision (published as
    tlp_nullified_o), response_seq_r the sequence number that decision carries,
    and nak_scheduled_r the NAK_SCHEDULED flag (PCIe Base Spec r2.1, §3.5.3.1).
    Returns one dict: Nak count and sequence numbers, cycles with NAK_SCHEDULED
    set, TLPs delivered upward, whether the injector fired, the armed packet's
    END position and the enumeration result.
    """
    tb, _m, _p, _c, _pa, _s, _d, tasks = await bring_up(dut)

    dut.inj_pkt.value  = pkt
    dut.inj_mode.value = mode
    dut.inj_off.value  = off
    dut.inj_bit.value  = bit
    dut.inj_en.value   = 1

    ep = _dll(dut, "ep").dllp_receive_inst.dllp2tlp_inst
    obs = {"naks": 0, "seq": [], "nak_sched": 0, "delivered": 0}

    async def watch():
        """Count Nak decisions, NAK_SCHEDULED cycles and deliveries until
        obs["stop"] is set."""
        prev_nak = 0
        while not obs.get("stop"):
            await RisingEdge(dut.clk_i)
            # A bare read after RisingEdge, as in every monitor here. Counts of
            # rises and of cycles do not depend on that one-cycle phase.
            n = int(ep.response_is_nak_r.value)
            if n and not prev_nak:
                obs["naks"] += 1
                obs["seq"].append(int(ep.response_seq_r.value))
            prev_nak = n
            if int(ep.nak_scheduled_r.value):
                obs["nak_sched"] += 1
            if (int(ep.m_tlp_axis_tvalid.value) and
                    int(ep.m_tlp_axis_tready.value) and
                    int(ep.m_tlp_axis_tlast.value)):
                obs["delivered"] += 1

    w = cocotb.start_soon(watch())
    r = await run_enumeration_fs(dut)
    await ClockCycles(dut.clk_i, 200)
    obs["stop"] = True
    await w
    dut.inj_en.value = 0
    for t in tasks:
        await t
    obs["fired"] = int(dut.inj_fired.value)
    obs["end_byte"] = int(dut.inj_end_byte.value)
    obs["enum"] = r
    return obs


async def _run_clean(dut):
    """The same run with the injector never armed: pkt=0 matches no packet.
    No test calls it."""
    return await _run_injected(dut, INJ_FLIP, off=0, bit=0, pkt=0)


@cocotb.test()
async def fullstack_7i_injected_header_error_is_naked_and_replayed(dut):
    """A bit error in a TLP header is Nak'd once, and TLPs are still delivered.

    INJ_FLIP flips bit 5 of the third beat after STP of the armed packet,
    inside the TLP header. A TLP whose LCRC does not match is discarded and, if
    NAK_SCHEDULED is clear, a Nak is scheduled at once; Ack and Nak DLLPs carry
    NEXT_RCV_SEQ - 1 (PCIe Base Spec r2.1, §3.5.3.1). Asserted: the injector
    fired, the EP decided exactly one Nak, and at least one TLP was delivered.
    Delivery of the replayed TLP itself is not checked separately.
    """
    obs = await _run_injected(dut, INJ_FLIP, off=3, bit=5)
    assert obs["fired"] == 1, "the injector never fired -- the row is vacuous"
    assert obs["naks"] == 1, f"expected exactly one Nak, saw {obs['naks']}"
    assert obs["delivered"] >= 1, "nothing was ever delivered -- the link is broken"


@cocotb.test()
async def fullstack_7i_injected_lcrc_error_is_naked_and_replayed(dut):
    """A bit error in data byte 17 of the armed packet is Nak'd once, and TLPs
    are still delivered.

    INJ_FLIP flips bit 2 of the ninth beat after STP of the armed packet
    (TARGET_PKT). frame_symbols puts STP in byte 0 of a word and
    lane_management sends each word as two beats from byte 0, so by
    pipe_codec_bridge's byte count beat k carries data bytes 2k-1 and 2k and
    bit 2 is in data byte 17. Asserted, as for the header error (PCIe Base
    Spec r2.1, §3.5.3.1): the injector fired, the EP decided exactly one Nak,
    and at least one TLP was delivered.
    """
    obs = await _run_injected(dut, INJ_FLIP, off=9, bit=2)
    assert obs["fired"] == 1, "the injector never fired -- the row is vacuous"
    assert obs["naks"] == 1, f"expected exactly one Nak, saw {obs['naks']}"
    assert obs["delivered"] >= 1, "nothing was ever delivered -- the link is broken"


@cocotb.test()
async def fullstack_7i_injected_sequence_error_is_naked_and_replayed(dut):
    """A bit error in a sequence number byte is Nak'd once, and TLPs are still
    delivered.

    INJ_FLIP flips bit 1 of the first beat after STP, a sequence number byte.
    The LCRC covers the sequence number (PCIe Base Spec r2.1, §3.5.2.1), so the
    one fault fails both the LCRC check and the NEXT_RCV_SEQ check in dllp2tlp's
    ST_CHECK_CRC, and the outcome must still be a single Nak.
    """
    obs = await _run_injected(dut, INJ_FLIP, off=1, bit=1)
    assert obs["fired"] == 1, "the injector never fired -- the row is vacuous"
    assert obs["naks"] == 1, f"expected exactly one Nak, saw {obs['naks']}"
    assert obs["delivered"] >= 1, "nothing was ever delivered -- the link is broken"


@cocotb.test()
async def fullstack_7i_nullified_tlp_is_discarded_silently(dut):
    """A nullified TLP is discarded silently: no Nak, and NAK_SCHEDULED is never
    set.

    A TLP that ends in EDB and whose LCRC is the bitwise inverse of the
    computed value is discarded, and this is not an error (PCIe Base Spec r2.1,
    §3.5.3.1). A transmitter nullifies a TLP by sending the LCRC without its
    final inversion and ending it with EDB (§3.5.2.1). INJ_NULLIFY builds that
    frame from the armed packet: END becomes EDB and the four LCRC bytes, from
    LCRC_FIRST_BYTE, are inverted. The armed packet's END position is asserted
    first, so the inverted bytes are known to be the LCRC.

    The Nak count is the assertion, not the delivery count: a receiver that
    treated the frame as corrupt would not deliver it either, but it would Nak
    it.
    """
    obs = await _run_injected(dut, INJ_NULLIFY, off=LCRC_FIRST_BYTE)
    assert obs["fired"] == 1, "the injector never fired -- the row is vacuous"
    assert obs["end_byte"] == LCRC_END_BYTE, (
        f"the armed packet's END is at data byte {obs['end_byte']}, not "
        f"{LCRC_END_BYTE}, so LCRC_FIRST_BYTE addresses the wrong four bytes "
        "and this row would be testing nothing"
    )
    assert obs["naks"] == 0, (
        f"a nullified TLP produced {obs['naks']} Nak(s); §3.5.3.1 p.182 says "
        "discarding it 'is not considered an error'"
    )
    assert obs["nak_sched"] == 0, (
        "NAK_SCHEDULED was set for a nullified TLP; p.182 sets that flag only "
        "on the corrupt-EDB and bad-LCRC limbs, not on this one"
    )


@cocotb.test()
async def fullstack_7i_edb_with_non_inverted_lcrc_is_naked(dut):
    """An EDB-terminated TLP whose LCRC is not inverted is corrupt and is Nak'd
    once.

    When the end Symbol is EDB but the LCRC is not the inverse of the computed
    value, the TLP is corrupt: it is discarded and, if NAK_SCHEDULED is clear,
    a Nak is scheduled (PCIe Base Spec r2.1, §3.5.3.1). EDB alone does not make
    a frame nullified; the inverted LCRC is what tells a deliberate
    nullification from a corrupted frame. INJ_EDB_BAD turns END into EDB and
    leaves the LCRC alone. A receiver that ignored EDB would see an ordinary
    TLP with a good LCRC and deliver it without a Nak, which the expected Nak
    count of one rules out.
    """
    obs = await _run_injected(dut, INJ_EDB_BAD, off=LCRC_FIRST_BYTE)
    assert obs["fired"] == 1, "the injector never fired -- the row is vacuous"
    assert obs["naks"] == 1, (
        f"an EDB frame with a NON-inverted LCRC produced {obs['naks']} Nak(s); "
        "§3.5.3.1 p.182's second EDB bullet requires exactly one. "
        "0 is the pre-commit-C value and means the frame was DELIVERED."
    )


# ---------------------------------------------------------------------------
# Periodic UpdateFC
# ---------------------------------------------------------------------------
# In L0, an UpdateFC for each enabled type of non-infinite credit must be
# scheduled at least once every 30 us, -0%/+50% (PCIe Base Spec r2.1,
# §2.6.1.2). At the 8 ns clock that is 3,750 cycles nominal and 5,625 at most,
# per type. Cpl credit is advertised infinite, so the rule binds P and NP.
# dllp_fc_update keeps one timer per type, restarted only by that type's own
# UpdateFC. U7G2Capture records (cycle, first word) of every DLLP each DLL
# hands its PHY; everything is classified after the run.

UFC_NOMINAL = 30_000 // CLK_NS    # 3,750 cycles: the bench's hand copy of the RTL's
                                  # FcWaitPeriod derivation, 30 us / CLK_PERIOD_NS
UFC_CEILING = 45_000 // CLK_NS    # 5,625 cycles: 30 us +50 %, the rule's ceiling
UFC_HOP = 2
"""Cycles from the timer reaching FcWaitPeriod to the UpdateFC's own handshake:
ST_IDLE sees the timer at its limit and moves to ST_UPDATE_P, whose beat is
accepted the next cycle; the timer restarts from 0 on that handshake.  So the
idle interval between two UpdateFCs of one type is FcWaitPeriod + 2.  DERIVED
from the fix's RTL, not measured -- the default-witness row R-U2 is what checks
it, and a disagreement is a finding about this derivation."""
UFC_EXPECT_INTERVAL = UFC_NOMINAL + UFC_HOP     # 3,752
U_WINDOW = WINDOW - 1000
"""Cycles the U capture runs after bring_up() returns.  It ends inside bring_up's
own WINDOW so these rows cost exactly what W1-W4 cost, and it leaves ~49,000
cycles of idle after enumeration: long enough that an in-spec timer must emit
at least twelve UpdateFCs of each type there, and a 250,000-cycle one none."""
U_IDLE_SETTLE = 500
"""Cycles after enumeration returns before R-U2's idle window opens: the last
Acks and release-triggered UpdateFCs of enumeration are not idle behaviour."""


def u_selftest():
    """Known-answer test of the periodic-UpdateFC constants and helpers."""
    assert UFC_NOMINAL == 3750 and UFC_CEILING == 5625, "SELFTEST UFC window at 8 ns"
    assert UFC_NOMINAL <= UFC_EXPECT_INTERVAL <= UFC_CEILING, "SELFTEST pin inside window"
    assert _max_gap([100, 3852, 7604], 100, 9000) == 3752, "SELFTEST _max_gap interior"
    assert _max_gap([], 100, 9000) == 8900, "SELFTEST _max_gap empty = whole window"
    assert _max_gap([200], 100, 9000) == 8800, "SELFTEST _max_gap tail"
    assert _mode([3752, 3752, 3754, 3750]) == 3752, "SELFTEST _mode"
    # an Ack DLLP's first word is type 00h with the sequence in [31:24]/[19:16]
    assert (0x01000000 & 0xFF) == 0x00 and (0x40400490 & 0xF8) == DLLP_UPDATEFC_NP, \
        "SELFTEST type-byte masks"


def _max_gap(events, start, end):
    """Largest interval in [start] + events + [end]: a window with NO event of
    the type is one gap the whole window long, so absence is never vacuous."""
    ev = [start] + sorted(events) + [end]
    return max(b - a for a, b in zip(ev, ev[1:]))


def _mode(values):
    """The most common value, the smallest of any tie; None for no values."""
    counts = {}
    for v in values:
        counts[v] = counts.get(v, 0) + 1
    return max(sorted(counts), key=lambda v: counts[v]) if counts else None


class U7G2Capture:
    """Raw captures for the periodic-UpdateFC tests, both stacks: (cycle, first
    word) of every DLLP each DLL hands its PHY (m_phy_axis with tuser bit 0, the
    seam and convention W18Capture uses), and the first cycle each stack's
    fc_initialized_o reads high. Every signal is a bare read after RisingEdge,
    so the offsets between them are not skewed by the sampler."""

    SIDES = ("rc", "ep")

    def __init__(self, dut):
        """Handles on both DLLs and both fc_initialized_o outputs."""
        self.dll = {s: _dll(dut, s) for s in self.SIDES}
        self.fc = {"rc": dut.rc_fc_initialized_o, "ep": dut.ep_fc_initialized_o}
        self.dllp_tx = {s: [] for s in self.SIDES}
        self.fc_rise = {s: None for s in self.SIDES}
        self.fc_first_sample = {s: None for s in self.SIDES}
        self.cycles = 0
        self.enum_end = None

    async def run(self, clk, max_cycles):
        """Sample both stacks on each of max_cycles rising edges."""
        in_pkt = {s: False for s in self.SIDES}
        for n in range(max_cycles):
            await RisingEdge(clk)
            self.cycles = n
            for s, dll in self.dll.items():
                f = int(self.fc[s].value)
                if self.fc_first_sample[s] is None:
                    self.fc_first_sample[s] = f
                if self.fc_rise[s] is None and f:
                    self.fc_rise[s] = n
                if int(dll.m_phy_axis_tvalid.value) and int(dll.m_phy_axis_tready.value):
                    if not in_pkt[s] and (int(dll.m_phy_axis_tuser.value) & 1):
                        self.dllp_tx[s].append((n, int(dll.m_phy_axis_tdata.value)))
                    in_pkt[s] = not int(dll.m_phy_axis_tlast.value)

    # -- derived views, computed after the run, never during it -------------
    def of_type(self, side, dllp_type, after):
        """Cycles after `after` at which `side` sent a DLLP of dllp_type."""
        return [c for c, w in self.dllp_tx[side] if (w & 0xF8) == dllp_type and c > after]

    def acks(self, side, after):
        """Cycles after `after` at which `side` sent an Ack (type 00h)."""
        return [c for c, w in self.dllp_tx[side] if (w & 0xFF) == 0x00 and c > after]

    def report(self, dut, tag):
        """Log, per stack, the FC-init rise, the DLLP and Ack counts and the
        first UpdateFC-P and UpdateFC-NP cycles."""
        for s in self.SIDES:
            rise = self.fc_rise[s]
            after = rise if rise is not None else 0
            p = self.of_type(s, DLLP_UPDATEFC_P, after)
            np_ = self.of_type(s, DLLP_UPDATEFC_NP, after)
            dut._log.info("%s %s: sampled %d cycles; fc_rise=%s (first sample %s); "
                          "enum_end=%s; dllps_tx=%d; acks after rise=%d; "
                          "UpdateFC-P n=%d first %s; UpdateFC-NP n=%d first %s",
                          tag, s.upper(), self.cycles, rise, self.fc_first_sample[s],
                          self.enum_end, len(self.dllp_tx[s]), len(self.acks(s, after)),
                          len(p), p[:6], len(np_), np_[:6])


async def _run_u_capture(dut):
    """Bring up, start the raw capture, enumerate once, then idle to the end of
    the capture window. The enumeration is the traffic the gap test runs
    through; the idle tail is what the interval test measures."""
    tb, _m, _p, _c, _pa, _s, _d, tasks = await bring_up(dut)
    cap = U7G2Capture(dut)
    mtask = cocotb.start_soon(cap.run(dut.clk_i, U_WINDOW))
    r = await run_enumeration_fs(dut)
    cap.enum_end = cap.cycles
    _log_enum_fs(dut, r)
    await mtask
    for t in tasks:
        await t
    return cap, r


@cocotb.test()
async def fullstack_7g2_u1_updatefc_per_type_gap_bounded_under_traffic(dut):
    """From the rise of fc_initialized_o to the end of the window, on both
    stacks, no gap between consecutive UpdateFC DLLPs of the same type (P, NP)
    exceeds UFC_CEILING, 5,625 cycles (45 us).

    The window includes the enumeration traffic. dllp_fc_update restarts each
    type's timer only on that type's UpdateFC, so neither Acks nor releases of
    the other type may delay it. The window also includes the idle tail, where
    only the timer matters. Each type's gaps are anchored at the
    fc_initialized_o rise and at the window's last cycle, so a type that is
    never sent counts as one gap the whole window long (_max_gap).

    Non-vacuity: fc_initialized_o reads low first and then rises on both
    stacks; at least 4 x UFC_CEILING cycles follow the rise, so a pass needs at
    least three UpdateFCs of each type on each stack; enumeration completes;
    and each stack sends at least one Ack after the rise.
    """
    w_selftest()
    u_selftest()
    cap, r = await _run_u_capture(dut)
    cap.report(dut, "7G2[U1]")
    gaps = {}
    for s in U7G2Capture.SIDES:
        rise = cap.fc_rise[s]
        assert cap.fc_first_sample[s] == 0 and rise is not None, (
            f"NON-VACUITY: {s.upper()} fc_initialized_o first sample "
            f"{cap.fc_first_sample[s]}, rise {rise} -- it must read low, then rise")
        assert cap.cycles - rise >= 4 * UFC_CEILING, (
            f"NON-VACUITY: only {cap.cycles - rise} cycles after the {s.upper()} "
            f"rise, fewer than 4 x {UFC_CEILING}")
        assert cap.acks(s, rise), (
            f"NON-VACUITY: the {s.upper()} DLL transmitted no Ack after FC init, "
            "so there was no traffic for the timer to survive")
        for name, t in (("P", DLLP_UPDATEFC_P), ("NP", DLLP_UPDATEFC_NP)):
            gaps[(s, name)] = _max_gap(cap.of_type(s, t, rise), rise, cap.cycles)
    assert r["enum_done"] and not r["enum_error"], (
        f"NON-VACUITY: enumeration did not complete (done={r['enum_done']} "
        f"error={r['enum_error']} code={r['enum_error_code']})")
    bad = {f"{s}.{n}": g for (s, n), g in gaps.items() if g > UFC_CEILING}
    dut._log.info("7G2[U1] VERDICT: max gap per (stack, type) %s; ceiling %d; "
                  "over the ceiling %s",
                  {f"{s}.{n}": g for (s, n), g in gaps.items()}, UFC_CEILING, bad)
    assert not bad, (
        f"UpdateFC gaps over p.143's {UFC_CEILING}-cycle (45 us) ceiling: {bad}. "
        "Base 2.1 §2.6.1.2 p.143 requires an UpdateFC for EACH type at least "
        "once every 30 us (-0%/+50%) while in L0")


@cocotb.test()
async def fullstack_7g2_u2_updatefc_idle_interval_default_witness(dut):
    """On an idle link at the 8 ns clock, consecutive UpdateFCs of each type on
    each stack are 3,750 to 5,625 cycles apart (30 to 45 us), and the most
    common interval is exactly UFC_EXPECT_INTERVAL = 30 us / 8 ns + 2 = 3,752
    cycles.

    The mode, rather than every interval, is pinned to the expected value. The
    measurement is at the DLL -> PHY seam, where a cycle of back-pressure shifts
    one handshake and moves the two intervals beside it by equal and opposite
    amounts; an occasional stall leaves the mode unchanged, while a different
    FcWaitPeriod moves every interval. Every interval must still be inside the
    window.

    Idle is from U_IDLE_SETTLE cycles after enumeration returns to the end of
    the window. Non-vacuity: enumeration completes, and the idle span is at
    least 3 x UFC_CEILING cycles, so an in-spec timer gives at least two
    intervals per (stack, type).
    """
    w_selftest()
    u_selftest()
    cap, r = await _run_u_capture(dut)
    cap.report(dut, "7G2[U2]")
    assert cap.enum_end is not None and r["enum_done"] and not r["enum_error"], (
        "NON-VACUITY: enumeration did not complete, so there is no idle span")
    idle_from = cap.enum_end + U_IDLE_SETTLE
    assert cap.cycles - idle_from >= 3 * UFC_CEILING, (
        f"NON-VACUITY: the idle span is only {cap.cycles - idle_from} cycles")
    ivs = {}
    for s in U7G2Capture.SIDES:
        for name, t in (("P", DLLP_UPDATEFC_P), ("NP", DLLP_UPDATEFC_NP)):
            ev = cap.of_type(s, t, idle_from)
            ivs[f"{s}.{name}"] = [b - a for a, b in zip(ev, ev[1:])]
    bad = {k: v for k, v in ivs.items()
           if len(v) < 2 or not all(UFC_NOMINAL <= g <= UFC_CEILING for g in v)
           or _mode(v) != UFC_EXPECT_INTERVAL}
    dut._log.info("7G2[U2] VERDICT: idle from cycle %d to %d; intervals per "
                  "(stack, type) %s; modes %s; expected mode %d in [%d, %d]; bad %s",
                  idle_from, cap.cycles, ivs,
                  {k: _mode(v) for k, v in ivs.items()}, UFC_EXPECT_INTERVAL,
                  UFC_NOMINAL, UFC_CEILING, sorted(bad))
    assert not bad, (
        f"idle UpdateFC intervals off the shipped value: "
        f"{ {k: (len(v), _mode(v), v[:4]) for k, v in bad.items()} } -- expected >= 2 "
        f"per (stack, type), each in [{UFC_NOMINAL}, {UFC_CEILING}], mode "
        f"{UFC_EXPECT_INTERVAL} (FcWaitPeriod = 30 us / CLK_PERIOD_NS, + {UFC_HOP})")


# ---------------------------------------------------------------------------
# REPLAY_NUM rollover and Recovery
# ---------------------------------------------------------------------------
# A REPLAY_NUM rollover (11b to 00b) makes the Transmitter ask the Physical
# Layer for a retrain, and the replay waits until retraining completes; Data
# Link Layer state, the retry buffer included, survives unless LinkUp falls
# (PCIe Base Spec r2.1, §3.5.2.1). The stimulus is the bridge's
# DLLP blackout. With starve_en set, the bridge flips one bit of the first
# data byte after every SDP from the EP, so the RC's dllp_handler drops every
# EP DLLP on its CRC while TLPs still flow both ways; the DLLP type is
# scrambled at the seam, so Acks cannot be singled out. The first TLP the RC
# sends after arming is never Acked: it is sent 1 + 3 times, and the fourth
# REPLAY_TIMER expiry rolls REPLAY_NUM over.

# The rollover time is taken from the DLL's transmit handoffs (tlp_sent, a
# TLP's last beat accepted on m_phy_axis), not from the replay state machine:
# the fourth handoff of the starved sequence number plus W1_ROLL_MARGIN.
# LTSSM state codes are pcie_ltssm_downstream's ltssm_state_e.
W1_ROW = "fullstack_7k_w1_starved_link_recovers"
W1_LTSSM_L0 = 0x00005
W1_RECOVERY_FAMILY = 0x04      # every Recovery substate: state[4:0] == 5'b00100
W1_RCVR_LOCK, W1_RCVR_CFG, W1_RCVR_IDLE = 0x00024, 0x000E4, 0x00104
W1_RM_ERR = 4                  # retry_management ST_RETRY_ERR (retry_st_e)
W1_DL_ACTIVE = 4               # pcie_datalink_init ST_DL_ACTIVE
W1_FC_CHECK_FC2 = 16           # pcie_flow_ctrl_init: states <= this are initialisation
W1_INITFC = (DLLP_INITFC1_P, DLLP_INITFC1_NP, DLLP_INITFC1_CPL,
             DLLP_INITFC2_P, DLLP_INITFC2_NP, DLLP_INITFC2_CPL)
W1_REPLAY_TIMER = 622          # pcie_datalink_pkg::replay_timer_cycles(128, 1, 8), both stacks
W1_SENDS_TO_ROLLOVER = 4       # 1 + MAX_REPLAY_ATTEMPTS (3): the 4th expiry rolls over
W1_ROLL_MARGIN = 700           # 4th last beat -> rollover: the timer plus slack
W1_RECOVERY_BUDGET = 2000      # rollover -> RC Recovery entry
W1_FC_WAIT = 60000
W1_SETTLE = 200                # past the post-init UpdateFC pair before arming
W1_GUARD = 60000               # arm -> 4th send
W1_EDGE = 50                   # a DLLP in flight across arm/release is not the window's
W1_TAIL = 20000                # Recovery entry -> the checks that follow Recovery
W1_WINDOW = 200000             # the capture's own bound; the test stops it earlier


def w1_family(st):
    """The LTSSM state's major-state code, bits [4:0]."""
    return st & 0x1F


def w1_recovery_entries(trans, after):
    """Cycles at which the LTSSM enters the Recovery family from outside it,
    at or after `after`.  `trans` is [(cycle, state)] at every change."""
    out, prev = [], None
    for c, st in trans:
        inside = w1_family(st) == W1_RECOVERY_FAMILY
        if inside and (prev is None or w1_family(prev) != W1_RECOVERY_FAMILY) and c >= after:
            out.append(c)
        prev = st
    return out


def w1_starved(sends, after):
    """(seq, [cycles]) of the first sequence number handed to the PHY at or
    after `after`, with every handoff of that same number -- original and
    replays alike.  None if nothing was sent."""
    first = next(((c, s) for c, s in sends if c >= after), None)
    if first is None:
        return None
    return first[1], [c for c, s in sends if s == first[1] and c >= after]


def w1_handler_window(trans, lo, hi):
    """dllp_handler verdicts in [lo, hi]: (accepted, crc_rejected).  Accept =
    entry to ST_PROCESS_DLLP (2); reject = ST_CHECK_CRC (1) -> ST_IDLE (0)."""
    acc = rej = 0
    prev = None
    for c, st in trans:
        if lo <= c <= hi:
            if st == 2 and prev != 2:
                acc += 1
            if st == 0 and prev == 1:
                rej += 1
        prev = st
    return acc, rej


def w1_selftest():
    """Known-answer test of the rollover tests' helpers on hand-derived traces,
    run first by both rollover tests."""
    assert w1_family(W1_RCVR_LOCK) == W1_RECOVERY_FAMILY and \
        w1_family(W1_RCVR_IDLE) == W1_RECOVERY_FAMILY and \
        w1_family(W1_LTSSM_L0) != W1_RECOVERY_FAMILY and \
        w1_family(0x000E3) != W1_RECOVERY_FAMILY, "SELFTEST family decode"
    tr = [(0, 0x00000), (100, W1_LTSSM_L0), (500, W1_RCVR_LOCK), (560, W1_RCVR_CFG),
          (600, W1_RCVR_IDLE), (640, W1_LTSSM_L0), (900, W1_RCVR_LOCK)]
    assert w1_recovery_entries(tr, 0) == [500, 900], "SELFTEST recovery entries"
    assert w1_recovery_entries(tr, 501) == [900], "SELFTEST entries after"
    sends = [(10, 7), (40, 8), (700, 8), (1350, 8), (1400, 9), (2000, 8)]
    assert w1_starved(sends, 20) == (8, [40, 700, 1350, 2000]), "SELFTEST starved seq"
    assert w1_starved(sends, 3000) is None, "SELFTEST nothing sent"
    ht = [(5, 0), (10, 1), (11, 2), (12, 0), (20, 1), (21, 0), (30, 1), (31, 0)]
    # 0->1->2 at 10/11 is the one accept; 1->0 at 21 and at 31 are the two rejects
    assert w1_handler_window(ht, 0, 40) == (1, 2), "SELFTEST handler window"
    assert w1_handler_window(ht, 15, 40) == (0, 2), "SELFTEST handler window lo"


class W1Capture:
    """Raw per-cycle captures on both stacks for the rollover tests. Every
    signal is a bare read after RisingEdge, as in U7G2Capture, so no two events
    are skewed by the sampler. Every list holds (cycle, value...) at a change
    or an event; nothing is paired or counted during the run."""

    SIDES = ("rc", "ep")

    def __init__(self, dut):
        """Handles on each stack's LTSSM, DLL state machines, retry slots and
        receive path; empty event lists."""
        self.dut = dut
        self.dll = {s: _dll(dut, s) for s in self.SIDES}
        self.ltssm = {"rc": dut.u_rc.u_phy.pcie_ltssm_downstream_inst.curr_state,
                      "ep": dut.u_ep.gen_integrated_gen1_phy.endpoint_ltssm_inst.curr_state}
        self.rm = {s: self.dll[s].dllp_transmit_inst.retry_management_inst
                   for s in self.SIDES}
        self.slot_sig = {s: [self.rm[s].gen_retry_counters[i].curr_state for i in range(3)]
                         for s in self.SIDES}
        self.d2t = {s: self.dll[s].dllp_receive_inst.dllp2tlp_inst for s in self.SIDES}
        self.hdl = {s: self.dll[s].dllp_receive_inst.dllp_handler_inst for s in self.SIDES}
        self.cycles = 0
        self.stop = False
        k = ("lt", "lu", "dl", "fci", "sent", "dllp_tx", "acknak", "slot", "err",
             "occ", "nrs", "deliv", "hdl", "nak_rise", "lcrc_bad")
        self.ev = {s: {n: [] for n in k} for s in self.SIDES}
        self.starve = []
        self.starve_ep = []

    async def run(self, clk, max_cycles):
        """Record changes and events on both stacks every cycle, until stop is
        set or max_cycles pass."""
        last = {}
        in_pkt = {s: False for s in self.SIDES}

        def change(key, s, name, val):
            """Append (cycle, val) to ev[s][name] when val differs from the
            last value seen under key."""
            if last.get(key) != val:
                last[key] = val
                self.ev[s][name].append((self.cycles, val))

        for n in range(max_cycles):
            await RisingEdge(clk)
            self.cycles = n
            for s in self.SIDES:
                dll = self.dll[s]
                change((s, "lt"), s, "lt", int(self.ltssm[s].value))
                change((s, "lu"), s, "lu", int(dll.phy_link_up_i.value))
                change((s, "dl"), s, "dl", int(dll.pcie_datalink_init_inst.curr_state.value))
                change((s, "fci"), s, "fci", int(dll.pcie_flow_ctrl_init_inst.curr_state.value))
                change((s, "err"), s, "err", int(self.rm[s].retry_err_o.value))
                change((s, "occ"), s, "occ", int(self.rm[s].retrys_r.value))
                change((s, "nrs"), s, "nrs", int(self.d2t[s].next_expected_seq_num_r.value))
                change((s, "hdl"), s, "hdl", int(self.hdl[s].curr_state.value))
                for i, sig in enumerate(self.slot_sig[s]):
                    v = int(sig.value)
                    if last.get((s, "slot", i)) != v:
                        last[(s, "slot", i)] = v
                        self.ev[s]["slot"].append((n, i, v))
                if int(dll.tlp_sent.value):
                    self.ev[s]["sent"].append((n, int(dll.tlp_sent_seq.value)))
                if int(dll.seq_num_vld.value):
                    self.ev[s]["acknak"].append(
                        (n, int(dll.seq_num_acknack.value), int(dll.seq_num.value)))
                if int(dll.m_phy_axis_tvalid.value) and int(dll.m_phy_axis_tready.value):
                    if not in_pkt[s] and (int(dll.m_phy_axis_tuser.value) & 1):
                        self.ev[s]["dllp_tx"].append((n, int(dll.m_phy_axis_tdata.value) & 0xFF))
                    in_pkt[s] = not int(dll.m_phy_axis_tlast.value)
                d = self.d2t[s]
                if int(d.m_tlp_axis_tvalid.value) and int(d.m_tlp_axis_tready.value) and \
                        int(d.m_tlp_axis_tlast.value):
                    self.ev[s]["deliv"].append(n)
                # Every Nak this receiver decides on (a rise of response_is_nak_r)
                # and every frame whose LCRC fails on entry to ST_CHECK_CRC (4).
                # Nullified and EDB frames are excluded: dllp2tlp discards them
                # on purpose.
                nk = int(d.response_is_nak_r.value)
                if nk and not last.get((s, "nak")):
                    self.ev[s]["nak_rise"].append(n)
                last[(s, "nak")] = nk
                st = int(d.curr_state.value)
                if st == 4 and last.get((s, "d2t_st")) != 4 and \
                        not int(d.lcrc_matches.value) and \
                        not int(d.tlp_nullified_r.value) and not int(d.frame_is_edb_r.value):
                    self.ev[s]["lcrc_bad"].append(n)
                last[(s, "d2t_st")] = st
            v = int(self.dut.starve_cnt.value)
            if not self.starve or self.starve[-1][1] != v:
                self.starve.append((n, v))
            v = int(self.dut.starve_ep_cnt.value)
            if not self.starve_ep or self.starve_ep[-1][1] != v:
                self.starve_ep.append((n, v))
            if self.stop:
                return

    def census(self, dut, tag, arm, release):
        """Log every observation the rollover tests rest on, before any assertion."""
        for s in self.SIDES:
            e = self.ev[s]
            rec = w1_recovery_entries(e["lt"], arm)
            dut._log.info(
                "%s %s: lt=%s | recovery_entries_after_arm=%s | link_up=%s | dlcmsm=%s | "
                "fci=%s | err=%s | occ=%s", tag, s.upper(),
                [(c, hex(v)) for c, v in e["lt"] if c >= arm - 10][:24], rec,
                e["lu"], e["dl"], [x for x in e["fci"] if x[0] >= arm - 10][:12],
                e["err"], [x for x in e["occ"] if x[0] >= arm - 10][:24])
            dut._log.info(
                "%s %s: slots=%s", tag, s.upper(),
                [x for x in e["slot"] if x[0] >= arm - 10][:48])
            dut._log.info(
                "%s %s: sent_after_arm=%s", tag, s.upper(),
                [x for x in e["sent"] if x[0] >= arm][:32])
            dut._log.info(
                "%s %s: acknak_in_after_arm=%s", tag, s.upper(),
                [x for x in e["acknak"] if x[0] >= arm][:32])
            dut._log.info(
                "%s %s: dllp_tx_after_arm=%s", tag, s.upper(),
                [(c, hex(t)) for c, t in e["dllp_tx"] if c >= arm][:40])
            dut._log.info(
                "%s %s: nrs=%s | deliveries_after_arm=%s", tag, s.upper(),
                [x for x in e["nrs"] if x[0] >= arm - 10][:24],
                [c for c in e["deliv"] if c >= arm][:24])
            acc, rej = w1_handler_window(e["hdl"], arm + W1_EDGE, release)
            dut._log.info("%s %s: dllp_handler in [arm+%d, release] accepted=%d crc_rejected=%d",
                          tag, s.upper(), W1_EDGE, acc, rej)
        dut._log.info("%s starve_cnt=%s starve_ep_cnt=%s | arm=%s release=%s cycles=%s", tag,
                      self.starve, self.starve_ep, arm, release, self.cycles)


@cocotb.test()
async def fullstack_7k_w1_starved_link_recovers(dut):
    """A starved link retrains and carries on, losing nothing.

    The B -> A blackout (starve_en) is armed once both stacks finish FC init,
    then the RC enumerates so that it originates TLPs. The first TLP it sends
    after arming, sequence number S, is never Acked: it is sent 1 + 3 times and
    the fourth REPLAY_TIMER expiry rolls REPLAY_NUM over. The blackout is lifted
    when the RC LTSSM enters Recovery, or W1_RECOVERY_BUDGET cycles after the
    rollover if it never does.

    Non-vacuity first: S was sent exactly four times before the rollover, at
    REPLAY_TIMER spacing; the blackout corrupted at least one DLLP; and inside
    the blackout the RC's dllp_handler accepted no DLLP and rejected at least
    one on its CRC. Then the RC LTSSM must enter Recovery after the rollover,
    within W1_RECOVERY_BUDGET cycles (PCIe Base Spec r2.1, §3.5.2.1; from L0
    the next state is Recovery when directed, §4.2.6.5). After that:
      * the RC goes Recovery.RcvrLock -> RcvrCfg -> Idle -> L0 (§4.2.6.4);
      * each stack enters Recovery exactly once, the EP no earlier than the RC;
      * S is sent again after L0, and an Ack covering S reaches the RC;
      * the EP's NEXT_RCV_SEQ passes S, and the EP delivers no duplicate;
      * on both stacks link_up to the DLL never falls (LinkUp is 1b in
        Recovery, Table 4-7), the DLCMSM stays DL_Active (§3.2.1), flow
        control is not re-initialized and no InitFC DLLP is sent;
      * no retry slot on either stack enters ST_RETRY_ERR;
      * the RC's retrain request (retry_err_o) rises once and falls before
        Recovery.RcvrCfg, so it cannot send the LTSSM round a second time;
      * in the whole test the EP decides no Nak, fails no LCRC check and sends
        no Nak DLLP: entering Recovery truncated no TLP (a Transmitter may
        complete a TLP or DLLP in progress, §4.2.6.5).
    """
    detail = ""
    w1_selftest()
    tb, _m, _p, _c, _pa, _s, _d, tasks = await bring_up(dut)
    cap = W1Capture(dut)
    ctask = cocotb.start_soon(cap.run(dut.clk_i, W1_WINDOW))
    for _ in range(W1_FC_WAIT):
        await RisingEdge(dut.clk_i)
        if _i(dut.rc_fc_initialized_o) and _i(dut.ep_fc_initialized_o):
            break
    else:
        raise AssertionError("FC init did not complete on both stacks")
    await ClockCycles(dut.clk_i, W1_SETTLE)
    dut.starve_en.value = 1
    await RisingEdge(dut.clk_i)
    arm = cap.cycles
    etask = cocotb.start_soon(run_enumeration_fs(dut))

    starved = None
    while True:
        await RisingEdge(dut.clk_i)
        starved = w1_starved(cap.ev["rc"]["sent"], arm)
        if starved and len(starved[1]) >= W1_SENDS_TO_ROLLOVER:
            break
        if cap.cycles - arm > W1_GUARD:
            raise AssertionError(f"the starved TLP reached only {starved} sends "
                                 f"in {W1_GUARD} cycles")
    seq, sends = starved
    roll = sends[W1_SENDS_TO_ROLLOVER - 1] + W1_ROLL_MARGIN
    while cap.cycles < roll + W1_RECOVERY_BUDGET:
        await RisingEdge(dut.clk_i)
        if w1_recovery_entries(cap.ev["rc"]["lt"], arm):
            break
    dut.starve_en.value = 0
    await RisingEdge(dut.clk_i)
    release = cap.cycles

    cap.census(dut, "7K[W1]", arm, release)
    seq_sends = w1_starved(cap.ev["rc"]["sent"], arm)[1]
    pre_pin = [c for c in seq_sends if c <= roll]
    gaps = [b - a for a, b in zip(pre_pin, pre_pin[1:])]
    corrupted = [v for c, v in cap.starve if c <= release][-1] - \
        [v for c, v in cap.starve if c <= arm][-1]
    acc, rej = w1_handler_window(cap.ev["rc"]["hdl"], arm + W1_EDGE, release)
    rc_rec = w1_recovery_entries(cap.ev["rc"]["lt"], arm)
    detail = (f"S={seq} sends_before_roll={pre_pin} gaps={gaps} roll={roll} "
              f"arm={arm} release={release} corrupted={corrupted} "
              f"rc_accepted={acc} rc_crc_rejected={rej} rc_recovery={rc_rec}")
    dut._log.info("7K[W1] %s", detail)
    # -- non-vacuity: the stimulus reached the DUT -------------------------
    assert len(pre_pin) == W1_SENDS_TO_ROLLOVER, (
        f"S was sent {len(pre_pin)} times before the rollover, not "
        f"{W1_SENDS_TO_ROLLOVER}: the starve did not starve S")
    assert all(W1_REPLAY_TIMER <= g <= W1_REPLAY_TIMER + 100 for g in gaps), (
        f"replay gaps {gaps} are not REPLAY_TIMER ({W1_REPLAY_TIMER}) + the "
        f"replay path: the replays are not timer-driven")
    assert corrupted >= 1, "the blackout corrupted nothing"
    assert acc == 0 and rej >= 1, (
        f"the RC's dllp_handler accepted {acc} DLLPs inside the blackout "
        f"(rejected {rej}): the starve leaks")
    assert rc_rec and pre_pin[-1] + W1_REPLAY_TIMER <= rc_rec[0] <= roll + W1_RECOVERY_BUDGET, (
        f"the RC LTSSM did not enter Recovery after REPLAY_NUM rolled over "
        f"(4th expiry ~cycle {pre_pin[-1] + W1_REPLAY_TIMER}, entries {rc_rec}): "
        f"Base 2.1 §3.5.2.1 p.174 -- on rollover the Transmitter signals the "
        f"Physical Layer to retrain the Link")

    # ---- the rest: reached only once the RC has entered Recovery ----------
    t_rec = rc_rec[0]
    while cap.cycles < t_rec + W1_TAIL:
        await RisingEdge(dut.clk_i)
    cap.stop = True
    await ctask
    cap.census(dut, "7K[W1-post]", arm, release)
    e_rc, e_ep = cap.ev["rc"], cap.ev["ep"]
    rc_path = [st for c, st in e_rc["lt"] if c >= t_rec]
    if rc_path and rc_path[0] == 0x00004:   # ST_RECOVERY, the entry state, is allowed
        rc_path = rc_path[1:]
    assert rc_path[:4] == [W1_RCVR_LOCK, W1_RCVR_CFG, W1_RCVR_IDLE, W1_LTSSM_L0], (
        f"RC Recovery path {[hex(x) for x in rc_path[:6]]}, expected RcvrLock -> "
        f"RcvrCfg -> Idle -> L0 (§4.2.6.4 pp.239-246)")
    ep_rec = w1_recovery_entries(e_ep["lt"], arm)
    rc_rec_all = w1_recovery_entries(e_rc["lt"], arm)   # recounted over the whole tail
    assert len(rc_rec_all) == 1 and len(ep_rec) == 1 and ep_rec[0] >= t_rec, (
        f"Recovery entries after arm RC={rc_rec_all} EP={ep_rec}: expected exactly one "
        f"each (the starved count), the EP's following the RC's")
    t_l0 = next(c for c, st in e_rc["lt"] if c > t_rec and st == W1_LTSSM_L0)
    assert any(c > t_l0 for c in w1_starved(e_rc["sent"], arm)[1]), (
        "S was not transmitted again after the RC returned to L0: the deferred "
        "replay never proceeded")
    assert any(c > t_l0 and a == 1 and ((q - seq) & 0xFFF) < 0x800
               for c, a, q in e_rc["acknak"]), (
        "no Ack covering S reached the RC after Recovery")
    req_up = [c for c, v in e_rc["err"] if c >= arm and v == 1]
    req_dn = [c for c, v in e_rc["err"] if c >= arm and v == 0]
    t_cfg = next(c for c, st in e_rc["lt"] if c > t_rec and st == W1_RCVR_CFG)
    dut._log.info("7K[W1] request rises=%s falls=%s rcvr_cfg=%s | EP nak_rise=%s lcrc_bad=%s "
                  "| RC nak_rise=%s lcrc_bad=%s", req_up, req_dn, t_cfg, e_ep["nak_rise"],
                  e_ep["lcrc_bad"], e_rc["nak_rise"], e_rc["lcrc_bad"])
    assert len(req_up) == 1 and [f for f in req_dn if req_up[0] < f < t_cfg], (
        f"RC retrain request rose at {req_up} and fell at {req_dn}; it must rise once and "
        f"be low before Recovery.RcvrCfg at {t_cfg} (no re-trigger)")
    assert not e_ep["nak_rise"] and not e_ep["lcrc_bad"] and \
        not [c for c, t in e_ep["dllp_tx"] if t == 0x10], (
        f"the EP Nak'd {e_ep['nak_rise']} / failed an LCRC at {e_ep['lcrc_bad']}: Recovery "
        f"entry truncated a TLP (§4.2.6.5 p.248)")
    for s, e in (("rc", e_rc), ("ep", e_ep)):
        assert not [c for c, v in e["lu"] if c >= arm and v == 0], (
            f"{s.upper()}: link_up to the DLL fell (Table 4-7 p.216: LinkUp = 1b in Recovery)")
        assert not [c for c, v in e["dl"] if c >= arm and v != W1_DL_ACTIVE], (
            f"{s.upper()}: the DLCMSM left DL_Active (§3.2.1 p.159)")
        assert not [c for c, v in e["fci"] if c >= arm and v <= W1_FC_CHECK_FC2], (
            f"{s.upper()}: flow control re-initialised")
        assert not [c for c, t in e["dllp_tx"] if c >= arm and (t & 0xF8) in W1_INITFC], (
            f"{s.upper()}: an InitFC DLLP was transmitted after arming")
        assert not [x for x in e["slot"] if x[0] >= arm and x[2] == W1_RM_ERR], (
            f"{s.upper()}: a retry slot parked in ST_RETRY_ERR")
    ep_adv = [c for c, v in e_ep["nrs"] if c >= arm]
    ep_del = [c for c in e_ep["deliv"] if c >= arm]
    assert any(v == ((seq + 1) & 0xFFF) for c, v in e_ep["nrs"] if c >= arm), (
        f"the EP's NEXT_RCV_SEQ never passed S={seq}: S was never delivered")
    assert len(ep_del) == len(ep_adv), (
        f"the EP delivered {len(ep_del)} TLPs for {len(ep_adv)} sequence advances: "
        f"a duplicate reached its Transaction Layer")


# ---------------------------------------------------------------------------
# Endpoint-initiated Recovery
# ---------------------------------------------------------------------------
# The mirror of the RC-initiated test: the A -> B blackout (starve_ep_en)
# keeps every RC DLLP from the EP, so the EP's first TLP after arming is never
# Acked and the EP's REPLAY_NUM rolls over. The EP's DLL requests a retrain,
# the EP's LTSSM goes to Recovery, and the RC, in L0, follows because it
# receives TS1 Ordered Sets (PCIe Base Spec r2.1, §4.2.6.5). The test is
# about the RC as the follower and asserts nothing about the RC's own retrain
# request.

W4_ROW = "fullstack_7k_w4_ep_initiated_recovery_rc_follows"


@cocotb.test()
async def fullstack_7k_w4_ep_initiated_recovery_rc_follows(dut):
    """The EP retrains, the RC follows, and the RC loses nothing.

    Once both stacks finish FC init the A -> B blackout is armed and the RC
    enumerates. The EP's first TLP after arming, a Completion, is sent 1 + 3
    times and the fourth expiry rolls its REPLAY_NUM over. The blackout is
    lifted when the EP's LTSSM enters Recovery, or W1_RECOVERY_BUDGET cycles
    after the rollover if it never does.

    Non-vacuity first, as in the RC-initiated test but on the EP; then both
    stacks must enter Recovery, the RC no earlier than the EP. After that:
      * the RC enters Recovery exactly once after arming;
      * link_up to the RC's DLL never falls and its DLCMSM stays DL_Active
        (PCIe Base Spec r2.1, Table 4-7, §3.2.1); the RC sends no InitFC DLLP;
      * the EP returns to L0; every TLP the RC sent after arming is covered by
        an Ack the RC received; an Ack reaches the RC after the EP is back in
        L0; and the RC's retry buffer is empty at the end.
    """
    detail = ""
    w1_selftest()
    tb, _m, _p, _c, _pa, _s, _d, tasks = await bring_up(dut)
    cap = W1Capture(dut)
    ctask = cocotb.start_soon(cap.run(dut.clk_i, W1_WINDOW))
    for _ in range(W1_FC_WAIT):
        await RisingEdge(dut.clk_i)
        if _i(dut.rc_fc_initialized_o) and _i(dut.ep_fc_initialized_o):
            break
    else:
        raise AssertionError("FC init did not complete on both stacks")
    await ClockCycles(dut.clk_i, W1_SETTLE)
    dut.starve_ep_en.value = 1
    await RisingEdge(dut.clk_i)
    arm = cap.cycles
    etask = cocotb.start_soon(run_enumeration_fs(dut))

    starved = None
    while True:
        await RisingEdge(dut.clk_i)
        starved = w1_starved(cap.ev["ep"]["sent"], arm)
        if starved and len(starved[1]) >= W1_SENDS_TO_ROLLOVER:
            break
        if cap.cycles - arm > W1_GUARD:
            raise AssertionError(f"the EP's starved TLP reached only {starved} sends "
                                 f"in {W1_GUARD} cycles")
    seq, sends = starved
    roll = sends[W1_SENDS_TO_ROLLOVER - 1] + W1_ROLL_MARGIN
    while cap.cycles < roll + W1_RECOVERY_BUDGET:
        await RisingEdge(dut.clk_i)
        if w1_recovery_entries(cap.ev["ep"]["lt"], arm):
            break
    dut.starve_ep_en.value = 0
    await RisingEdge(dut.clk_i)
    release = cap.cycles
    # The RC's Recovery entry follows the EP's TS1s across both PHYs and the
    # bridge, so wait for it, at most W1_RECOVERY_BUDGET cycles, before judging.
    while cap.cycles < release + W1_RECOVERY_BUDGET and \
            not w1_recovery_entries(cap.ev["rc"]["lt"], arm):
        await RisingEdge(dut.clk_i)
    cap.census(dut, "7K[W4]", arm, release)
    pre_pin = [c for c in w1_starved(cap.ev["ep"]["sent"], arm)[1] if c <= roll]
    gaps = [b - a for a, b in zip(pre_pin, pre_pin[1:])]
    corrupted = [v for c, v in cap.starve_ep if c <= release][-1] - \
        [v for c, v in cap.starve_ep if c <= arm][-1]
    acc, rej = w1_handler_window(cap.ev["ep"]["hdl"], arm + W1_EDGE, release)
    ep_rec = w1_recovery_entries(cap.ev["ep"]["lt"], arm)
    rc_rec = w1_recovery_entries(cap.ev["rc"]["lt"], arm)
    detail = (f"EP S={seq} sends_before_roll={pre_pin} gaps={gaps} roll={roll} arm={arm} "
              f"release={release} corrupted={corrupted} ep_accepted={acc} "
              f"ep_crc_rejected={rej} ep_recovery={ep_rec} rc_recovery={rc_rec}")
    dut._log.info("7K[W4] %s", detail)
    # -- non-vacuity: the stimulus reached the EP --------------------------
    assert len(pre_pin) == W1_SENDS_TO_ROLLOVER, (
        f"the EP's TLP was sent {len(pre_pin)} times before the rollover, not "
        f"{W1_SENDS_TO_ROLLOVER}")
    assert all(W1_REPLAY_TIMER <= g <= W1_REPLAY_TIMER + 100 for g in gaps), (
        f"EP replay gaps {gaps} are not REPLAY_TIMER-driven")
    assert corrupted >= 1, "the A -> B blackout corrupted nothing"
    assert acc == 0 and rej >= 1, (
        f"the EP's dllp_handler accepted {acc} DLLPs inside the blackout "
        f"(rejected {rej}): the starve leaks")
    assert ep_rec and rc_rec and ep_rec[0] <= rc_rec[0], (
        f"the RC did not follow an EP-initiated Recovery (EP entries {ep_rec}, RC entries "
        f"{rc_rec}): §4.2.6.5 p.248 -- L0 goes to Recovery when a TS1 is received")

    # ---- the rest: reached only once the RC has followed into Recovery ----
    while cap.cycles < rc_rec[0] + W1_TAIL:
        await RisingEdge(dut.clk_i)
    cap.stop = True
    await ctask
    cap.census(dut, "7K[W4-post]", arm, release)
    e_rc, e_ep = cap.ev["rc"], cap.ev["ep"]
    rc_all = w1_recovery_entries(e_rc["lt"], arm)
    assert len(rc_all) == 1, f"RC Recovery entries after arm {rc_all}, expected exactly 1"
    assert not [c for c, v in e_rc["lu"] if c >= arm and v == 0], (
        "RC: link_up to the DLL fell (Table 4-7 p.216: LinkUp = 1b in Recovery)")
    assert not [c for c, v in e_rc["dl"] if c >= arm and v != W1_DL_ACTIVE], (
        "RC: the DLCMSM left DL_Active (§3.2.1 p.159)")
    assert not [c for c, t in e_rc["dllp_tx"] if c >= arm and (t & 0xF8) in W1_INITFC], (
        "RC: an InitFC DLLP was transmitted after arming")
    t_ep_l0 = next((c for c, st in e_ep["lt"] if c > ep_rec[0] and st == W1_LTSSM_L0), None)
    rc_sent = [(c, q) for c, q in e_rc["sent"] if c >= arm]
    acks = [(c, q) for c, a, q in e_rc["acknak"] if a == 1]
    unacked = [(c, q) for c, q in rc_sent
               if not any(ca > c and ((qa - q) & 0xFFF) < 0x800 for ca, qa in acks)]
    dut._log.info("7K[W4] ep_back_in_l0=%s rc_sent_after_arm=%s unacked=%s occ_end=%s",
                  t_ep_l0, rc_sent[:12], unacked, e_rc["occ"][-1:])
    assert t_ep_l0 is not None, "the EP never returned to L0"
    assert rc_sent and not unacked, (
        f"RC TLPs sent after arming and never Acked: {unacked}")
    assert any(ca > t_ep_l0 for ca, _ in acks), (
        "no Ack reached the RC after the EP returned to L0")
    assert e_rc["occ"] and e_rc["occ"][-1][1] == 0, (
        f"the RC's retry buffer is not empty at the end: {e_rc['occ'][-3:]}")


# ---------------------------------------------------------------------------
# Extended configuration space
# ---------------------------------------------------------------------------
# A Function's configuration space is 4096 bytes (PCIe Base Spec r2.1, §7.2),
# addressed by {Extended Register Number, Register Number} with the extended
# number the more significant (§7.3.2). These tests issue CfgRd0 and CfgWr0 on
# the RC's s_axis_rq_* after enumeration, when pcie_rc_top hands the RQ arm
# from the engine to the host (rq_engine_owns_o low); the engine itself sets
# Extended Register Number to 0 in every stage. Completions leave on
# u_rc.m_axis_rc_*, which the bench leaves unconnected with tready tied to 1,
# so they are read hierarchically. Offsets 200h and 210h are used because an
# address cut to 9 bits would alias them onto VID/DID and BAR0, which hold
# known non-zero values; 100h, 1FCh and FFCh read 0 either way.

X7L_RQ_CFG_READ0 = 0b1000   # pcie_rq_rc_pkg::RQ_CFG_READ0
X7L_RQ_CFG_WRITE0 = 0b1010  # pcie_rq_rc_pkg::RQ_CFG_WRITE0
X7L_BDF = 0x0000            # scan_bus_i = 0, so pcie_enum_scan's device_bdf_o is 00:00.0
X7L_VID_DID = 0x00FF1234    # pcie_config_reg.sv's readback of offset 0x000
X7L_REQ_WINDOW = 20000      # cycles allowed for each request's completion
X7L_ERR_STROBES = (
    "rq_protocol_error_o", "rq_gearbox_error_o", "rc_protocol_error_o",
    "rc_gearbox_error_o", "rc_unexpected_completion_o", "command_error_valid_o",
    "cpl_timeout_valid_o", "late_cpl_valid_o",
)

# The no-alias test's reads and the value each must return. 000h is the
# positive control; 100h is the empty Extended Capability header (PCIe Base
# Spec r2.1, §7.9.1); 1FCh and FFCh are unimplemented; 200h and 210h would
# alias onto VID/DID and BAR0 under a 9-bit address. An unimplemented register
# reads 0 (PCI Local Bus Spec r3.0, §6.1).
X7L_W1_EXPECT = (
    (0x000, X7L_VID_DID),
    (0x100, 0x00000000),
    (0x1FC, 0x00000000),
    (0xFFC, 0x00000000),
    (0x200, 0x00000000),
    (0x210, 0x00000000),
)
X7L_W1_SEQ = tuple(("rd", off, 0) for off, _v in X7L_W1_EXPECT)


def x7l_rq_desc(req_type, dword_count, address=0, completer_id=0):
    """RQ descriptor (PG213, Table 60 and Table 61), built as rq_desc() in
    tb/rc/test_pcie_rq_rc_top.py builds it; copied rather than imported across
    bench directories. The tag field [103:96] is left 0: the core assigns tags."""
    v = address & ((1 << 64) - 1)
    v |= (dword_count & 0x7FF) << 64
    v |= (req_type & 0xF) << 75
    v |= (completer_id & 0xFFFF) << 104
    return v


def x7l_cfg_desc_address(offset):
    """Byte offset -> the configuration form of the descriptor address,
    {ext_reg[11:8], reg_num[7:2], 00} (pcie_rq_if.sv's config address assembly)."""
    return offset & 0xFFC


def x7l_decode_rc_desc(v):
    """Decode the 96-bit RC descriptor (PG213, Table 65), as decode_rc_desc()
    in tb/rc/test_pcie_rq_rc_top.py does."""
    return {
        "lower_address": v & 0xFFF,
        "error_code": (v >> 12) & 0xF,
        "byte_count": (v >> 16) & 0x1FFF,
        "request_completed": (v >> 30) & 1,
        "dword_count": (v >> 32) & 0x7FF,
        "status": (v >> 43) & 0x7,
        "requester_id": (v >> 48) & 0xFFFF,
        "tag": (v >> 64) & 0xFF,
        "completer_id": (v >> 72) & 0xFFFF,
    }


def x7l_link_tlp(beats):
    """[(tdata, tkeep)] of one link packet at a DLL AXIS input -> (seq, TLP bytes).

    32-bit little-endian beats, byte 0 = tdata[7:0], as decode_link_first_word
    reads them. On the link a TLP is a 2-byte sequence number, the TLP and a
    4-byte LCRC (PCIe Base Spec r2.1, §3.5.1, Figure 3-12), so the TLP's own
    bytes start at stream byte 2. The LCRC tail is left on; every field the
    decoders read is at a fixed header index.
    """
    stream = []
    for tdata, tkeep in beats:
        for i in range(4):
            if (tkeep >> i) & 1:
                stream.append((tdata >> (8 * i)) & 0xFF)
    return ((stream[0] & 0xF) << 8) | stream[1], stream[2:]


def x7l_le32(b):
    """Four bytes, least significant first, as one 32-bit value."""
    return b[0] | (b[1] << 8) | (b[2] << 16) | (b[3] << 24)


def x7l_decode_cfg_req(t):
    """TLP bytes of a Configuration Request (PCIe Base Spec r2.1, Figure 2-18).

    byte 0 Fmt/Type; bytes 2-3 Length; bytes 4-5 Requester ID; 6 Tag;
    7 {Last BE, First BE}; 8 Bus; 9 {Device[7:3], Function[2:0]};
    10 {Reserved[7:4], Ext Register Number[3:0]}; 11 {Register Number[7:2], R}.
    """
    has_data = bool(t[0] & 0x40)
    return {
        "fmt_type": t[0], "length": ((t[2] & 0x3) << 8) | t[3],
        "tag": t[6], "last_be": t[7] >> 4, "first_be": t[7] & 0xF,
        "bus": t[8], "dev": t[9] >> 3, "fn": t[9] & 0x7,
        "rsvd": t[10] >> 4, "ext_reg": t[10] & 0xF, "reg": t[11] >> 2, "r": t[11] & 0x3,
        "offset": ((t[10] & 0xF) << 8) | (t[11] & 0xFC),
        "data": x7l_le32(t[12:16]) if has_data else None,
    }


def x7l_decode_cpl(t):
    """TLP bytes of a Completion (PCIe Base Spec r2.1, §2.2.9, Figure 2-27).

    4-5 Completer ID; 6 {Status[7:5], BCM[4], Byte Count[11:8]}; 7 Byte
    Count[7:0]; 8-9 Requester ID; 10 Tag; 11 {R, Lower Address[6:0]}; 12-15
    the data DW, little-endian (a configuration register's byte 0 is the TLP's).
    """
    has_data = bool(t[0] & 0x40)
    return {
        "fmt_type": t[0], "length": ((t[2] & 0x3) << 8) | t[3],
        "completer_id": (t[4] << 8) | t[5], "status": t[6] >> 5, "bcm": (t[6] >> 4) & 1,
        "byte_count": ((t[6] & 0xF) << 8) | t[7],
        "requester_id": (t[8] << 8) | t[9], "tag": t[10], "lower_address": t[11] & 0x7F,
        "data": x7l_le32(t[12:16]) if has_data else None,
    }


def x7l_packets(beats):
    """[(cycle, tdata, tkeep, tlast, ...)] -> [(first_cycle, [(tdata, tkeep)], first_extra)]."""
    out, cur, first = [], [], None
    for b in beats:
        if not cur:
            first = b
        cur.append((b[1], b[2]))
        if b[3]:
            out.append((first[0], cur, first[4:]))
            cur = []
    return out


def x7l_selftest():
    """Known-answer test of the extended-configuration decoders, run first by
    each of these tests.

    Vectors A to D are byte streams laid out by hand from Figure 2-18 and
    Figure 2-27 of PCIe Base Spec r2.1 and packed into little-endian words by
    hand; E and F are descriptor literals written by hand from PG213. None is
    captured from the DUT or produced by the encoder the decoder inverts.
    """
    # A. CfgRd0 00:00.0 offset 0xFFC, seq 0x005, tag 0x03, FirstBE 1111. TLP:
    #    04 00 00 01 | 00 00 03 0F | 00 00 0F FC, LCRC 11 22 33 44.
    #    Stream 00 05 04 00 00 01 00 00 03 0F 00 00 0F FC 11 22 33 44.
    a = [(0x00040500, 0xF), (0x00000100, 0xF), (0x00000F03, 0xF),
         (0x2211FC0F, 0xF), (0x00004433, 0x3)]
    seq, t = x7l_link_tlp(a)
    r = x7l_decode_cfg_req(t)
    assert seq == 0x005 and r["fmt_type"] == 0x04 and r["length"] == 1, "SELFTEST A hdr"
    assert (r["tag"], r["first_be"], r["last_be"]) == (0x03, 0xF, 0x0), "SELFTEST A dw1"
    assert (r["ext_reg"], r["reg"], r["offset"]) == (0xF, 0x3F, 0xFFC), "SELFTEST A offset"
    assert (r["bus"], r["dev"], r["fn"], r["data"]) == (0, 0, 0, None), "SELFTEST A bdf"
    # B. CfgWr0 01:02.3 offset 0x24C, seq 0x123, tag 0x07, data 0xA5A50003. TLP:
    #    44 00 00 01 | 00 00 07 0F | 01 13 02 4C | 03 00 A5 A5, LCRC 55 66 77 88.
    b = [(0x00442301, 0xF), (0x00000100, 0xF), (0x13010F07, 0xF),
         (0x00034C02, 0xF), (0x6655A5A5, 0xF), (0x00008877, 0x3)]
    seq, t = x7l_link_tlp(b)
    r = x7l_decode_cfg_req(t)
    assert seq == 0x123 and r["fmt_type"] == 0x44, "SELFTEST B hdr"
    assert (r["bus"], r["dev"], r["fn"]) == (1, 2, 3), "SELFTEST B bdf"
    assert (r["ext_reg"], r["reg"], r["offset"]) == (0x2, 0x13, 0x24C), "SELFTEST B offset"
    assert r["data"] == 0xA5A50003, "SELFTEST B data"
    # C. CplD from 00:00.0, SC, Byte Count 4, tag 0x03, LA 0, data 0x00FF1234,
    #    seq 0x007. TLP: 4A 00 00 01 | 00 00 00 04 | 00 00 03 00 | 34 12 FF 00.
    c = [(0x004A0700, 0xF), (0x00000100, 0xF), (0x00000400, 0xF),
         (0x12340003, 0xF), (0xBC9A00FF, 0xF), (0x0000F0DE, 0x3)]
    seq, t = x7l_link_tlp(c)
    r = x7l_decode_cpl(t)
    assert seq == 0x007 and r["fmt_type"] == 0x4A and r["length"] == 1, "SELFTEST C hdr"
    assert (r["status"], r["bcm"], r["byte_count"]) == (0, 0, 4), "SELFTEST C status/bc"
    assert (r["tag"], r["lower_address"], r["data"]) == (0x03, 0, 0x00FF1234), "SELFTEST C data"
    # D. Cpl from 01:00.0, status UR (001b), Byte Count 4, tag 0x09, no data,
    #    seq 0x00A. TLP: 0A 00 00 00 | 01 00 20 04 | 00 00 09 00.
    d = [(0x000A0A00, 0xF), (0x00010000, 0xF), (0x00000420, 0xF),
         (0xBBAA0009, 0xF), (0x0000DDCC, 0x3)]
    seq, t = x7l_link_tlp(d)
    r = x7l_decode_cpl(t)
    assert seq == 0x00A and r["fmt_type"] == 0x0A and r["data"] is None, "SELFTEST D hdr"
    assert (r["completer_id"], r["status"], r["byte_count"], r["tag"]) == \
        (0x0100, 1, 4, 0x09), "SELFTEST D fields"
    # E. RC descriptor (PG213, Table 65): BC 4, request_completed, 1 DW, SC, tag 3,
    #    with the data DW in [127:96] of the same 128-bit beat.
    v = x7l_decode_rc_desc(0x00000003_00000001_40040000)
    assert (v["byte_count"], v["request_completed"], v["dword_count"], v["status"],
            v["tag"], v["error_code"]) == (4, 1, 1, 0, 3, 0), "SELFTEST E rc desc"
    # F. the descriptor golden: CfgRd0 at 0x100 to 00:00.0 -> address 0x100,
    #    dword_count 1 at [74:64] (bit 64), req_type 1000b at [78:75] (bit 78),
    #    written out as a literal: nibble 19 = 0x4, nibble 16 = 0x1.
    assert x7l_rq_desc(X7L_RQ_CFG_READ0, 1, x7l_cfg_desc_address(0x100), 0) == \
        0x4001_0000_0000_0000_0100, "SELFTEST F rq desc"
    assert x7l_cfg_desc_address(0xFFF) == 0xFFC, "SELFTEST F address bits [1:0] reserved"


class X7LCapture:
    """Raw events for the extended-configuration tests, each a bare read after
    RisingEdge: the value the DUT's flops sampled at that edge, the right phase
    for counting AXIS handshakes.

      ep_in   beats handshaken into the EP's DLL (dllp_receive s_axis_*): the
              Configuration Requests as they arrive off the link
      rc_in   beats handshaken into the RC's DLL: the Completions
      rc_cpl  beats on u_rc.m_axis_rc_* (the bench ties tready to 1)
      err     (cycle, name) of every RC error or timeout strobe
      axil    the EP configuration block's AXI-lite handshakes: the wrapper's
              32-bit address beside pcie_config_reg's own 12-bit port, and the
              write data and strobes the register file receives

    axil is an internal tap. The configuration-write test asserts on its
    write-address handshakes; the wire-encoding test decodes only ep_in.
    """

    def __init__(self, dut):
        """Handles on both DLLs' receive paths and the EP's configuration block."""
        self.dut = dut
        self.ep_rx = _dll(dut, "ep").dllp_receive_inst
        self.rc_rx = _dll(dut, "rc").dllp_receive_inst
        self.ep_cfg = self.ep_rx.pcie_cfg_wrapper_inst
        self.ep_reg = self.ep_cfg.pcie_config_reg_inst
        self.ev = {"ep_in": [], "rc_in": [], "rc_cpl": [], "err": [], "axil": []}
        self.rc_pkts = 0
        self.cycle = 0
        self.stop = False

    async def run(self, clk):
        """Record every event, one cycle at a time, until stop is set."""
        rc = self.dut.u_rc
        strobes = [(nm, getattr(rc, nm)) for nm in X7L_ERR_STROBES]
        while not self.stop:
            await RisingEdge(clk)
            self.cycle += 1
            n = self.cycle
            for name, rx in (("ep_in", self.ep_rx), ("rc_in", self.rc_rx)):
                if int(rx.s_axis_tvalid.value) and int(rx.s_axis_tready.value):
                    self.ev[name].append((n, int(rx.s_axis_tdata.value),
                                          int(rx.s_axis_tkeep.value),
                                          int(rx.s_axis_tlast.value),
                                          int(rx.s_axis_tuser.value)))
            if int(rc.m_axis_rc_tvalid.value):
                last = int(rc.m_axis_rc_tlast.value)
                self.ev["rc_cpl"].append((n, int(rc.m_axis_rc_tdata.value),
                                          int(rc.m_axis_rc_tkeep.value), last))
                if last:
                    self.rc_pkts += 1
            for nm, h in strobes:
                if int(h.value):
                    self.ev["err"].append((n, nm))
            w, g = self.ep_cfg, self.ep_reg
            if int(w.s_axil_awvalid.value) and int(w.s_axil_awready.value):
                self.ev["axil"].append((n, "aw", int(w.s_axil_awaddr.value),
                                        int(g.s_axil_awaddr.value)))
            if int(w.s_axil_wvalid.value) and int(w.s_axil_wready.value):
                self.ev["axil"].append((n, "w", int(w.s_axil_wdata.value),
                                        int(w.s_axil_wstrb.value)))
            if int(w.s_axil_arvalid.value) and int(w.s_axil_arready.value):
                self.ev["axil"].append((n, "ar", int(w.s_axil_araddr.value),
                                        int(g.s_axil_araddr.value)))
            if int(w.s_axil_rvalid.value) and int(w.s_axil_rready.value):
                self.ev["axil"].append((n, "r", int(w.s_axil_rdata.value),
                                        int(w.s_axil_rresp.value)))

    def dump(self, dut):
        """Log every raw event as one RAW7L line, for analysis outside the run."""
        for n, d, k, l, u in self.ev["ep_in"]:
            dut._log.info("RAW7L|ep_in|%d|%08x|%x|%d|%x", n, d, k, l, u)
        for n, d, k, l, u in self.ev["rc_in"]:
            dut._log.info("RAW7L|rc_in|%d|%08x|%x|%d|%x", n, d, k, l, u)
        for n, d, k, l in self.ev["rc_cpl"]:
            dut._log.info("RAW7L|rc_cpl|%d|%032x|%x|%d", n, d, k, l)
        for n, nm in self.ev["err"]:
            dut._log.info("RAW7L|err|%d|%s", n, nm)
        for n, kind, a, b in self.ev["axil"]:
            dut._log.info("RAW7L|axil|%d|%s|%08x|%08x", n, kind, a, b)


async def x7l_send_rq(dut, beats, limit=4000):
    """beats: (tdata, tkeep, tlast, tuser) on the host RQ AXIS.
    The handshake shape of tb/rc/test_pcie_rq_rc_top.py send_rq()."""
    for data, keep, last, user in beats:
        dut.s_axis_rq_tdata.value = data
        dut.s_axis_rq_tkeep.value = keep
        dut.s_axis_rq_tlast.value = last
        dut.s_axis_rq_tuser.value = user
        dut.s_axis_rq_tvalid.value = 1
        for _ in range(limit):
            await ReadOnly()
            fired = int(dut.s_axis_rq_tready.value) == 1
            await RisingEdge(dut.clk_i)
            if fired:
                break
        else:
            raise AssertionError("s_axis_rq_tready never asserted -- stalled")
    dut.s_axis_rq_tvalid.value = 0
    dut.s_axis_rq_tlast.value = 0


async def x7l_run_sequence(dut, seq):
    """Issue `seq` one request at a time on s_axis_rq_* and capture raw events.

    Returns (cap, reqs): reqs[i] = (op, offset, wdata, issue_cycle, done_cycle).
    One request is outstanding at a time, so the i-th m_axis_rc packet, the i-th
    distinct Configuration Request at the EP and the i-th Completion at the RC
    all belong to seq[i].
    """
    # run_enumeration_fs returns inside a ReadOnly phase, where cocotb refuses a
    # write, so step to the next clock edge before driving.
    await RisingEdge(dut.clk_i)
    cap = X7LCapture(dut)
    ctask = cocotb.start_soon(cap.run(dut.clk_i))
    reqs = []
    for op, off, wdata in seq:
        before = cap.rc_pkts
        rtype = X7L_RQ_CFG_WRITE0 if op == "wr" else X7L_RQ_CFG_READ0
        desc = x7l_rq_desc(rtype, 1, x7l_cfg_desc_address(off), X7L_BDF)
        user = 0x0F   # {Last DW BE 0000b, First DW BE 1111b} (PCIe Base Spec r2.1, §2.2.7)
        t0 = cap.cycle
        dut._log.info("RAW7L|req|%d|%s|%03x|%08x", t0, op, off, wdata)
        if op == "wr":
            await x7l_send_rq(dut, [(desc, 0xF, 0, user), (wdata, 0x1, 1, 0)])
        else:
            await x7l_send_rq(dut, [(desc, 0xF, 1, user)])
        for _ in range(X7L_REQ_WINDOW):
            await RisingEdge(dut.clk_i)
            if cap.rc_pkts > before:
                break
        else:
            raise AssertionError(
                f"no m_axis_rc packet for {op} {off:#05x} in {X7L_REQ_WINDOW} cycles")
        reqs.append((op, off, wdata, t0, cap.cycle))
    await ClockCycles(dut.clk_i, 200)   # let trailing Acks / UpdateFCs land
    cap.stop = True
    await ctask
    cap.dump(dut)
    return cap, reqs


def x7l_decode(cap, reqs):
    """Pair the raw streams with the requests. Returns a list of per-request dicts."""
    rc_pk = x7l_packets(cap.ev["rc_cpl"])
    ep = [p for p in x7l_packets(cap.ev["ep_in"]) if p[2][0] & 0x2]   # UserIsTlp
    rc = [p for p in x7l_packets(cap.ev["rc_in"]) if p[2][0] & 0x2]
    ep_req, seen = [], {}
    for n, beats, _ in ep:
        seq, t = x7l_link_tlp(beats)
        if t[0] in (0x04, 0x44):
            if seq in seen:
                seen[seq] += 1          # a replay of a request already counted
                continue
            seen[seq] = 1
            ep_req.append((n, seq, x7l_decode_cfg_req(t)))
    rc_cpl = []
    for n, beats, _ in rc:
        seq, t = x7l_link_tlp(beats)
        if t[0] in (0x4A, 0x0A):
            rc_cpl.append((n, seq, x7l_decode_cpl(t)))
    out = []
    for i, (op, off, wdata, t0, t1) in enumerate(reqs):
        words = []
        for tdata, tkeep in rc_pk[i][1]:
            for dw in range(4):
                if (tkeep >> dw) & 1:
                    words.append((tdata >> (32 * dw)) & 0xFFFFFFFF)
        desc = x7l_decode_rc_desc(words[0] | (words[1] << 32) | (words[2] << 64))
        out.append({
            "op": op, "off": off, "wdata": wdata, "t_issue": t0, "t_done": t1,
            "rc_desc": desc, "rc_data": words[3] if len(words) > 3 else None,
            "wire": ep_req[i][2] if i < len(ep_req) else None,
            "cpl": rc_cpl[i][2] if i < len(rc_cpl) else None,
        })
    return out, ep_req, rc_cpl, {s: c for s, c in seen.items() if c > 1}


def x7l_report(dut, tag, rows, dups, err):
    """Log one line per request (the request as it crossed the wire, its
    Completion and the RC descriptor), then the replays and error strobes."""
    for i, r in enumerate(rows):
        w, c, d = r["wire"] or {}, r["cpl"] or {}, r["rc_desc"]
        dut._log.info(
            "%s|%02d|%s|%03x|wire ft=%s ext=%s reg=%s bdf=%s:%s.%s fbe=%s lbe=%s len=%s"
            "|cpl ft=%s st=%s bc=%s la=%s cid=%s|rc st=%s err=%s bc=%s dw=%s|data=%s",
            tag, i, r["op"], r["off"],
            hex(w.get("fmt_type", -1)), hex(w.get("ext_reg", -1)), hex(w.get("reg", -1)),
            w.get("bus"), w.get("dev"), w.get("fn"), w.get("first_be"), w.get("last_be"),
            w.get("length"), hex(c.get("fmt_type", -1)), c.get("status"),
            c.get("byte_count"), c.get("lower_address"), c.get("completer_id"),
            d["status"], d["error_code"], d["byte_count"], d["dword_count"],
            "%08x" % r["rc_data"] if r["rc_data"] is not None else "-")
    dut._log.info("%s|replays=%s|errors=%s", tag, dups, err)


@cocotb.test()
async def fullstack_7l_w1_no_config_space_alias(dut):
    """Reads across the 4 KB configuration space return their own registers,
    not aliases.

    A Function's configuration space is 4096 bytes (PCIe Base Spec r2.1, §7.2);
    an unimplemented register reads 0 and completes normally (PCI Local Bus
    Spec r3.0, §6.1); with no extended capabilities the header at 100h is 0
    (§7.9.1). The reads and their values are X7L_W1_EXPECT. 200h and 210h are
    the reads that detect aliasing: cut to 9 bits, their addresses land on
    VID/DID (00FF1234h) and BAR0 (FFF00000h). 100h, 1FCh and FFCh read 0
    whether or not the address is cut.

    Also asserted: every request completes with Successful Completion status,
    the distinct requests arrive on the wire at the EP at their own offsets,
    in order (a replay is counted once), and no RC error strobe fires.
    """
    await x7l_prologue(dut)
    cap, reqs = await x7l_run_sequence(dut, X7L_W1_SEQ)
    rows, ep_req, _rc_cpl, dups = x7l_decode(cap, reqs)
    x7l_report(dut, "X7L_W1", rows, dups, cap.ev["err"])
    assert len(rows) == len(X7L_W1_SEQ), (
        f"{len(rows)} completions for {len(X7L_W1_SEQ)} requests")
    assert all(x["rc_desc"]["status"] == 0 and x["rc_desc"]["error_code"] == 0
               for x in rows), "a request was not completed with SC"
    assert [q[2]["offset"] for q in ep_req] == [off for off, _v in X7L_W1_EXPECT], (
        f"the requests did not arrive at the EP at their own offsets: "
        f"{[hex(q[2]['offset']) for q in ep_req]}")
    assert not cap.ev["err"], f"RC error strobes: {cap.ev['err'][:8]}"
    got = {r["off"]: r["rc_data"] for r in rows}
    bad = [(hex(off), hex(got[off]), hex(v)) for off, v in X7L_W1_EXPECT if got[off] != v]
    assert not bad, f"(offset, returned, expected): {bad}"


async def x7l_prologue(dut):
    """Self-test, bring-up, enumeration. Afterwards the RQ arm must be the host's."""
    x7l_selftest()
    up = await bring_up(dut)
    r = await run_enumeration_fs(dut)
    _log_enum_fs(dut, r)
    assert r["enum_done"] and not r["enum_error"], (
        f"enumeration must complete before the RQ arm is the host's: "
        f"code={r['enum_error_code']}")
    assert int(dut.rq_engine_owns_o.value) == 0, "the engine still owns the RQ arm"
    return up


def x7l_axil_by_request(cap, reqs, kind):
    """The EP config block's AXI-lite handshakes of one kind ('aw', 'w', 'ar', 'r'),
    attributed to the request whose [issue, completion] window holds them. One
    request is outstanding at a time, so the windows do not overlap."""
    return [[e for e in cap.ev["axil"] if e[1] == kind and t0 <= e[0] <= t1]
            for _op, _off, _wd, t0, t1 in reqs]


# The wire-encoding test's reads.
X7L_W2_SEQ = (("rd", 0x000, 0), ("rd", 0x100, 0), ("rd", 0xFFC, 0))


@cocotb.test()
async def fullstack_7l_w2_config_request_carries_ext_register_number(dut):
    """The Configuration Request on the wire carries the full 10-bit register
    address.

    DW2 of a Configuration Request is {Bus, Device, Function, Reserved, Ext
    Register Number[3:0], Register Number[5:0], R} (PCIe Base Spec r2.1,
    Figure 2-18), with the extended number the more significant (§7.3.2). So
    100h is Ext Register 1h / Register 00h and FFCh is Ext Register Fh /
    Register 3Fh. Each request is also checked as a 1-DW CfgRd0 with First DW
    BE 1111b to 00:00.0 with zero reserved bits, 000h included.

    The fields are decoded from the wire, not from an internal signal: from the
    beats handshaken into the EP's DLL (dllp_receive s_axis_*), selected by
    tuser[1] (UserIsTlp) and read past the 2-byte sequence number by
    x7l_link_tlp, whose known-answer test runs first. What this checks is the
    RC's request builder (pcie_rq_if's configuration address assembly).
    """
    await x7l_prologue(dut)
    cap, reqs = await x7l_run_sequence(dut, X7L_W2_SEQ)
    rows, ep_req, _rc_cpl, dups = x7l_decode(cap, reqs)
    x7l_report(dut, "X7L_W2", rows, dups, cap.ev["err"])
    assert len(ep_req) == len(X7L_W2_SEQ), (
        f"{len(ep_req)} distinct Configuration Requests at the EP for "
        f"{len(X7L_W2_SEQ)} issued")
    for (_op, off, _wd), (_n, _seq, w) in zip(X7L_W2_SEQ, ep_req):
        assert (w["fmt_type"], w["length"], w["first_be"], w["last_be"]) == \
            (0x04, 1, 0xF, 0x0), f"{off:#05x}: not a 1-DW CfgRd0 on the wire: {w}"
        assert (w["bus"], w["dev"], w["fn"], w["rsvd"], w["r"]) == (0, 0, 0, 0, 0), (
            f"{off:#05x}: BDF / reserved bits wrong on the wire: {w}")
        assert (w["ext_reg"], w["reg"]) == (off >> 8, (off >> 2) & 0x3F), (
            f"{off:#05x} went out as ExtReg {w['ext_reg']:#x} / Reg {w['reg']:#04x}")
    w100, wffc = ep_req[1][2], ep_req[2][2]
    assert (w100["ext_reg"], w100["reg"]) == (0x1, 0x00), (
        f"0x100 must be ExtReg 0x1 / Reg 0x00 (Figure 2-18 p.80), got {w100}")
    assert (wffc["ext_reg"], wffc["reg"]) == (0xF, 0x3F), (
        f"0xFFC must be ExtReg 0xF / Reg 0x3F (Figure 2-18 p.80), got {wffc}")


# The configuration-write test's writes: an implemented register, and an
# offset that a 9-bit address would alias onto it.
X7L_W4_SEQ = (("wr", 0x04C, 0x00000003), ("wr", 0x24C, 0x0000A55A))


@cocotb.test()
async def fullstack_7l_w4_config_write_reaches_its_own_offset(dut):
    """A CfgWr0 reaches the register file at its own offset, not at
    (offset & 1FFh).

    The witness is internal: the address on pcie_config_reg's own write-address
    port (pcie_cfg_wrapper_inst.pcie_config_reg_inst.s_axil_awaddr), not a
    readback. A readback cannot see an aliased write here, because
    pcie_config_decode passes a zero payload (rx_tlp_data) and every
    configuration write therefore stores 0. A write to a reserved register
    must be a no-op (PCI Local Bus Spec r3.0, §6.1), which a write aliased
    onto an implemented register is not; 24Ch would land on 04Ch,
    pcie_config_reg's link_control_3_register.

    04Ch, an implemented register, is the positive pair: it must arrive as 04Ch
    through the same path. Both writes must complete with Successful Completion
    status, arrive at the EP on the wire at their own offsets, and make exactly
    one write-address handshake each at the register file.
    """
    await x7l_prologue(dut)
    cap, reqs = await x7l_run_sequence(dut, X7L_W4_SEQ)
    rows, ep_req, _rc_cpl, dups = x7l_decode(cap, reqs)
    x7l_report(dut, "X7L_W4", rows, dups, cap.ev["err"])
    aw = x7l_axil_by_request(cap, reqs, "aw")
    for i, a in enumerate(aw):
        dut._log.info("X7L_W4|aw|%d|%s", i,
                      [(n, f"{w32:#010x}", f"{port:#05x}") for n, _k, w32, port in a])
    assert all(x["rc_desc"]["status"] == 0 and x["rc_desc"]["error_code"] == 0
               for x in rows), "a write was not completed with SC"
    assert [q[2]["offset"] for q in ep_req] == [off for _o, off, _w in X7L_W4_SEQ], (
        f"the writes did not arrive at the EP at their own offsets: "
        f"{[hex(q[2]['offset']) for q in ep_req]}")
    assert [len(a) for a in aw] == [1] * len(X7L_W4_SEQ), (
        f"write-address handshakes per request: {[len(a) for a in aw]}")
    assert [a[0][3] for a in aw] == [off for _o, off, _w in X7L_W4_SEQ], (
        f"the register file's write-address port received "
        f"{[hex(a[0][3]) for a in aw]} for writes to "
        f"{[hex(off) for _o, off, _w in X7L_W4_SEQ]}")


# ---------------------------------------------------------------------------
# PIPE seam width
# ---------------------------------------------------------------------------
# At Gen1 the PHY IP's PIPE carries 16 data bits and 2 K bits per lane; the
# upper bits are ignored (PG239, Table 5 and Table 7). Both stacks' PIPE ports
# are 16 + 2 bits per lane (the RC through PHY_DATA_WIDTH, the EP through its
# PipeDataWidth), while each PHY keeps a 32-bit symbol container inside:
# phy_transmit passes the container's low half to the port, and phy_receive
# zero-extends the port into the container. The DLL-facing Dword bus beside
# the seam stays 32 bits. The three tests check the elaborated widths, that
# the EP receives the same frames as over a 32-bit seam, and that the unused
# container half is zero in L0.

GTH81_SEAM_BITS = 16   # PG239, Table 5: 16 data bits per lane at Gen1
GTH81_SEAM_K = 2       # PG239, Table 5: phy_txdatak[1:0] at Gen1 and Gen2
GTH81_DWORD_BITS = 32  # the DLL-facing Dword bus, which keeps its width

GTH81_K_STP, GTH81_K_SDP = 0xFB, 0x5C      # PCIe Base Spec r2.1, Table 4-1: STP K27.7, SDP K28.2
GTH81_K_END, GTH81_K_EDB = 0xFD, 0xFE      # END K29.7, EDB K30.7


def gth81_widths(dut):
    """Every PIPE-seam width in both stacks, and the Dword bus beside it.

    Each width is len() of the handle, the elaborated size of the net, so it
    reflects the parameters as elaborated rather than a declaration. Returns
    (seam data widths, seam K widths, Dword widths), each keyed by path.
    """
    rc, phy = dut.u_rc, dut.u_rc.u_phy
    ep = dut.u_ep.gen_integrated_gen1_phy
    seam = {
        "bench.rc_phy_txdata": len(dut.rc_phy_txdata),
        "pcie_rc_top.phy_txdata": len(rc.phy_txdata),
        "pcie_rc_top.phy_rxdata": len(rc.phy_rxdata),
        "pcie_phy_top.phy_txdata": len(phy.phy_txdata),
        "pcie_phy_top.phy_rxdata": len(phy.phy_rxdata),
        "rc.phy_transmit.pipe_data_o": len(phy.phy_transmit_inst.pipe_data_o),
        "rc.phy_receive.pipe_data_i": len(phy.phy_receive_inst.pipe_data_i),
        "ep.phy_txdata": len(ep.phy_txdata),
        "ep.phy_rxdata": len(ep.phy_rxdata),
        "ep.phy_transmit.pipe_data_o": len(ep.phy_transmit_inst.pipe_data_o),
        "ep.phy_receive.pipe_data_i": len(ep.phy_receive_inst.pipe_data_i),
    }
    seam_k = {
        "bench.rc_phy_txdatak": len(dut.rc_phy_txdatak),
        "pcie_rc_top.phy_txdatak": len(rc.phy_txdatak),
        "pcie_rc_top.phy_rxdatak": len(rc.phy_rxdatak),
        "pcie_phy_top.phy_txdatak": len(phy.phy_txdatak),
        "pcie_phy_top.phy_rxdatak": len(phy.phy_rxdatak),
        "rc.phy_transmit.pipe_data_k_o": len(phy.phy_transmit_inst.pipe_data_k_o),
        "rc.phy_receive.pipe_data_k_i": len(phy.phy_receive_inst.pipe_data_k_i),
        "ep.phy_txdatak": len(ep.phy_txdatak),
        "ep.phy_rxdatak": len(ep.phy_rxdatak),
        "ep.phy_transmit.pipe_data_k_o": len(ep.phy_transmit_inst.pipe_data_k_o),
        "ep.phy_receive.pipe_data_k_i": len(ep.phy_receive_inst.pipe_data_k_i),
    }
    dword = {
        "rc.pcie_phy_top.s_tlp_axis_tdata": len(phy.s_tlp_axis_tdata),
        "rc.pcie_phy_top.m_tlp_axis_tdata": len(phy.m_tlp_axis_tdata),
        "ep.datalink_layer.s_tlp_axis_tdata": len(dut.u_ep.datalink_layer_inst.s_tlp_axis_tdata),
        "ep.datalink_layer.s_phy_axis_tdata": len(dut.u_ep.datalink_layer_inst.s_phy_axis_tdata),
    }
    return seam, seam_k, dword


def gth81_frames(ev):
    """[(cycle, data16, k2)] -> (frames, open_tail).

    Byte 0 of a beat is the first Symbol, and K flag bit s belongs to byte s, as
    in pipe_codec_bridge and the EP's own codec. A frame opens on a K-flagged
    STP or SDP and closes on a K-flagged END or EDB; its record is (kind, data
    bytes as hex, closer, first cycle). Data Symbols outside a frame (Logical
    Idle, TS bodies) are skipped, and so are COM and SKP outside a frame. A K
    Symbol inside a frame, or a second opener, closes the frame as BROKEN, so a
    malformed frame is reported rather than skipped. open_tail is True when the
    capture ends inside a frame.
    """
    frames, cur = [], None
    for n, d, k in ev:
        for s in (0, 1):
            b, isk = (d >> (8 * s)) & 0xFF, (k >> s) & 1
            if isk and b in (GTH81_K_STP, GTH81_K_SDP):
                if cur is not None:
                    frames.append(("BROKEN", bytes(cur[1]).hex(), "opener", cur[2]))
                cur = ("TLP" if b == GTH81_K_STP else "DLLP", [], n)
            elif cur is not None:
                if isk and b in (GTH81_K_END, GTH81_K_EDB):
                    frames.append((cur[0], bytes(cur[1]).hex(),
                                   "END" if b == GTH81_K_END else "EDB", cur[2]))
                    cur = None
                elif isk:
                    frames.append(("BROKEN", bytes(cur[1]).hex(), f"K{b:02x}", cur[2]))
                    cur = None
                else:
                    cur[1].append(b)
    return frames, cur is not None


def gth81_diff(ref, live):
    """Byte identity of two frame lists, ignoring cycles.  [] means identical."""
    a = [f[:3] for f in ref]
    b = [f[:3] for f in live]
    out = []
    if len(a) != len(b):
        out.append(f"frame count: reference {len(a)}, live {len(b)}")
    for i, (x, y) in enumerate(zip(a, b)):
        if x != y:
            out.append(f"frame {i}: reference {x} live {y}")
            if len(out) >= 6:
                break
    return out


def gth81_selftest():
    """Known-answer test of gth81_frames and gth81_diff, run first by the
    frame-comparison test.

    Beats are laid out by hand as (cycle, byte1 << 8 | byte0, k1 << 1 | k0).
    """
    # A. idle, then STP 00 01 02 03 END split across beats, a COM outside, then
    #    SDP aa bb END in one and a half beats.
    ev = [(1, 0x0000, 0), (2, 0x00FB, 0b01), (3, 0x0201, 0), (4, 0xFD03, 0b10),
          (5, 0x00BC, 0b01), (6, 0xAA5C, 0b01), (7, 0xFDBB, 0b10)]
    fr, tail = gth81_frames(ev)
    assert [f[:3] for f in fr] == [("TLP", "00010203", "END"), ("DLLP", "aabb", "END")], \
        f"SELFTEST A frames {fr}"
    assert [f[3] for f in fr] == [2, 6] and not tail, f"SELFTEST A cycles/tail {fr} {tail}"
    # B. a K inside a frame closes it BROKEN; an EDB closer is kept as EDB;
    #    an unterminated frame at the end is reported as an open tail.
    ev = [(1, 0x11FB, 0b01), (2, 0xBC22, 0b10), (3, 0x33FB, 0b01), (4, 0xFE44, 0b10),
          (5, 0x555C, 0b01)]
    fr, tail = gth81_frames(ev)
    assert [f[:3] for f in fr] == [("BROKEN", "1122", "Kbc"), ("TLP", "3344", "EDB")], \
        f"SELFTEST B frames {fr}"
    assert tail, "SELFTEST B open tail"
    # C. a K-flagged data value is a K; the same byte unflagged is data.
    fr, _ = gth81_frames([(1, 0xFBFB, 0b01), (2, 0x00FD, 0b01)])
    assert [f[:3] for f in fr] == [("TLP", "fb", "END")], f"SELFTEST C {fr}"
    # D. the differ: identity, one byte, one missing frame, cycles ignored.
    r = [("TLP", "0001", "END", 5), ("DLLP", "aabb", "END", 9)]
    assert gth81_diff(r, [("TLP", "0001", "END", 7), ("DLLP", "aabb", "END", 99)]) == []
    assert len(gth81_diff(r, [("TLP", "0001", "END", 5), ("DLLP", "aabc", "END", 9)])) == 1
    assert gth81_diff(r, r[:1])[0].startswith("frame count"), "SELFTEST D count"
    assert gth81_diff(r, [])[0] == "frame count: reference 2, live 0", "SELFTEST D empty"


class GTH81WireCapture:
    """Raw capture of every valid beat the EP receives, at its own 8b/10b
    decoder output.

    The EP's gen_8b10b_lane decoders turn phy_rx_symbol_i (two 10-bit symbols
    per lane, from the bridge) into phy_rxdata[15:0] and phy_rxdatak[1:0]
    combinationally, so the bytes and phy_rx_symbol_valid_i belong to the same
    cycle and are sampled in one phase (a bare read after RisingEdge, as in
    every capture here). Only lane 0's 16 data bits and two K flags are kept.
    """

    def __init__(self, dut):
        """Handles on the EP's integrated PHY and its received-symbol valid."""
        self.dut = dut
        self.ep = dut.u_ep.gen_integrated_gen1_phy
        self.valid = dut.u_ep.phy_rx_symbol_valid_i
        self.ev = []
        self.cycle = 0
        self.stop = False

    async def run(self, clk):
        """Append (cycle, data, k) for every valid beat until stop is set."""
        while not self.stop:
            await RisingEdge(clk)
            self.cycle += 1
            if int(self.valid.value) & 1:
                self.ev.append((self.cycle, int(self.ep.phy_rxdata.value) & 0xFFFF,
                                int(self.ep.phy_rxdatak.value) & 0x3))


def gth81_write_reference(path, frames, dut):
    """Write the 32-wide reference as a Python module (GTH81_W2_WRITE mode only)."""
    with open(path, "w") as f:
        f.write('"""§63 #5 8-1 W2: the 32-wide reference -- every TLP/DLLP frame the EP\n')
        f.write("received, bring-up through enum_done, on the PRE-EDIT tree (32+4 seam).\n\n")
        f.write("GENERATED by fullstack_gth81_w2_ep_wire_frames_match_32_wide with\n")
        f.write("GTH81_W2_WRITE set. Do not edit by hand; provenance is in\n")
        f.write('pcie_docs evidence/gth-8/8-1/W2_REFERENCE.md.\n"""\n\n')
        f.write("FRAMES = (\n")
        for kind, hx, end, n in frames:
            f.write(f"    ({kind!r}, {hx!r}, {end!r}, {n}),\n")
        f.write(")\n")
    dut._log.info("GTH81_W2 wrote %d frames to %s", len(frames), path)


@cocotb.test()
async def fullstack_gth81_w1_pipe_seam_is_16_plus_2(dut):
    """Every PIPE-seam port and bus is 16 data + 2 K bits per lane in both
    stacks, and the DLL-facing Dword bus beside it stays 32 bits.

    At Gen1 only phy_txdata[15:0] and phy_txdatak[1:0] are used (PG239,
    Table 5). Widths are len() of each handle, the elaborated size, because
    lint/waiver.vlt waives WIDTHEXPAND for whole files: a 32-bit net left on
    the seam would zero-extend without a warning. The Dword widths are asserted
    beside the seam widths, so narrowing the DLL bus cannot pass for narrowing
    the seam. Nothing is reset or run: the widths are read after two clock
    cycles.
    """
    TB(dut)
    await ClockCycles(dut.clk_i, 2)
    seam, seam_k, dword = gth81_widths(dut)
    for name, v in {**seam, **seam_k, **dword}.items():
        dut._log.info("GTH81_W1|%s|%d", name, v)
    bad = {k: v for k, v in dword.items() if v != GTH81_DWORD_BITS}
    assert not bad, f"the DLL-facing Dword bus moved (it must stay 32): {bad}"
    bad = {k: v for k, v in seam.items() if v != GTH81_SEAM_BITS}
    assert not bad, f"PIPE seam data not 16 per lane: {bad}"
    bad = {k: v for k, v in seam_k.items() if v != GTH81_SEAM_K}
    assert not bad, f"PIPE seam K not 2 per lane: {bad}"


@cocotb.test()
async def fullstack_gth81_w2_ep_wire_frames_match_32_wide(dut):
    """Every TLP and DLLP the EP receives from bring-up to enum_done is
    byte-identical to what it received over a 32-bit PIPE seam.

    The reference, gth81_w2_reference.FRAMES, is this test's own capture from a
    build with a 32 + 4 seam. When the GTH81_W2_WRITE environment variable
    names an output file, the test writes its capture there as the reference
    (gth81_write_reference) instead of comparing. The capture is the EP's own
    decoder output (GTH81WireCapture), scrambled data between K framing, so a
    byte that either stack's width conversion dropped or moved changes a frame
    or stops the link.

    Compared: kind, every data byte and the closer, in order. Cycles are only
    reported as identical or not. The window closes on enum_done: enumeration
    must complete, and at least one TLP and one DLLP must arrive (non-vacuity).

    The reference fixes the traffic of enumeration. A change that alters that
    traffic moves this test, and the reference is then regenerated with
    GTH81_W2_WRITE, not edited by hand.
    """
    gth81_selftest()
    cap = GTH81WireCapture(dut)
    task = cocotb.start_soon(cap.run(dut.clk_i))
    await bring_up(dut)
    r = await run_enumeration_fs(dut)
    _log_enum_fs(dut, r)
    await RisingEdge(dut.clk_i)
    cap.stop = True
    await RisingEdge(dut.clk_i)
    task.kill()
    frames, tail = gth81_frames(cap.ev)
    kinds = {}
    for f in frames:
        kinds[f[0]] = kinds.get(f[0], 0) + 1
    dut._log.info("GTH81_W2 beats=%d frames=%d kinds=%s open_tail=%s enum_done=%s",
                  len(cap.ev), len(frames), kinds, tail, r["enum_done"])
    out = os.environ.get("GTH81_W2_WRITE", "")
    diffs = []
    if not out:
        # The diff is computed and logged before any assertion, so a run that
        # fails the enumeration check still records what the EP received.
        import gth81_w2_reference as ref  # staged by tb_fullstack.core (cocotb_fullstack)
        diffs = gth81_diff(ref.FRAMES, frames)
        same_cycles = [f[3] for f in ref.FRAMES] == [f[3] for f in frames]
        dut._log.info("GTH81_W2 reference=%d live=%d byte_identical=%s cycles_identical=%s",
                      len(ref.FRAMES), len(frames), not diffs, same_cycles)
        for d in diffs:
            dut._log.info("GTH81_W2|diff|%s", d)
    assert r["enum_done"] and not r["enum_error"], (
        f"enumeration did not complete (code {r['enum_error_code']}); the window "
        f"this row compares never closed; the EP received {len(frames)} frames {kinds}")
    assert kinds.get("TLP", 0) >= 1 and kinds.get("DLLP", 0) >= 1, (
        f"NON-VACUITY: the EP received {kinds}; W2 needs at least one TLP and one DLLP")
    if out:
        gth81_write_reference(out, frames, dut)
        return
    assert not diffs, f"the EP received different frames than over the 32-wide seam: {diffs}"


# The dropped-half test: its L0 window and the six capture points.
GTH81_ST_L0 = 0x00005        # pcie_ltssm_downstream ST_L0
GTH81_L0_MIN = 1000          # an L0 stay shorter than this is not a window (cycles)

# (point, stack): lane 0's upper container half is read at each point.
GTH81_W4_POINTS = (
    ("rc.tx", "rc"),       # phy_transmit.scr_data_out   -- the half the TX port drops
    ("rc.rx_in", "rc"),    # phy_receive.desc_data_in     -- the half the RX port zero-fills
    ("rc.rx_out", "rc"),   # phy_receive.descrambler_data -- the same half after the descrambler
    ("ep.tx", "ep"),
    ("ep.rx_in", "ep"),
    ("ep.rx_out", "ep"),
)


def gth81_w4_check(samples, st_l0=GTH81_ST_L0, l0_min=GTH81_L0_MIN):
    """samples: [(cycle, rc_state, ep_state, {point: (hi16, khi2)})].

    Per stack, the window is the longest contiguous run of that stack's own
    LTSSM in ST_L0 (_longest_run), and the run must hold more than 90% of the
    stack's ST_L0 samples. Returns (windows, violations, pre_l0, vacuity):
      windows      {stack: (first, last, len, total_l0)}
      violations   [(point, cycle, hi, khi)] inside the stack's window
      pre_l0       {point: count of nonzero samples outside the window}
      vacuity      [reason] for a stack whose longest ST_L0 run is shorter than
                   l0_min or does not dominate
    """
    windows, vacuity = {}, []
    for stack, col in (("rc", 1), ("ep", 2)):
        st = [(s[0], s[col]) for s in samples]
        first, last, n = _longest_run(st, st_l0)
        total = sum(1 for _, v in st if v == st_l0)
        windows[stack] = (first, last, n, total)
        if n < l0_min:
            vacuity.append(f"{stack}: longest ST_L0 run {n} < {l0_min} cycles")
        elif n <= 0.9 * total:
            vacuity.append(f"{stack}: longest ST_L0 run {n} does not dominate {total}")
    violations, pre = [], {p: 0 for p, _ in GTH81_W4_POINTS}
    for s in samples:
        n = s[0]
        for p, stack in GTH81_W4_POINTS:
            hi, khi = s[3][p]
            if not (hi or khi):
                continue
            first, last = windows[stack][0], windows[stack][1]
            if first is not None and first <= n <= last:
                violations.append((p, n, hi, khi))
            else:
                pre[p] += 1
    return windows, violations, pre, vacuity


def gth81_w4_selftest():
    """Known-answer test of gth81_w4_check on hand-built sample traces, run
    first by the dropped-half test."""
    z = {p: (0, 0) for p, _ in GTH81_W4_POINTS}

    def trace(n_pre, n_l0, rc_pre_outlier=False, poke=None):
        """n_pre training samples, then n_l0 ST_L0 samples; optionally one RC
        ST_L0 outlier at cycle 2, and nonzero values from poke by cycle."""
        out = []
        for n in range(1, n_pre + n_l0 + 1):
            st = GTH81_ST_L0 if n > n_pre else 0x00003
            rc_st = GTH81_ST_L0 if (rc_pre_outlier and n == 2) else st
            vals = dict(z)
            if poke and n in poke:
                vals.update(poke[n])
            out.append((n, rc_st, st, vals))
        return out
    # A. clean: both windows [11, 1210], no violations, nothing pre-L0.
    w, v, pre, vac = gth81_w4_check(trace(10, 1200))
    assert w["rc"][:3] == (11, 1210, 1200) and w["ep"][:3] == (11, 1210, 1200), f"SELFTEST A {w}"
    assert not v and not vac and not any(pre.values()), f"SELFTEST A {v} {vac} {pre}"
    # B. a nonzero dropped half in L0 -- data on rc.tx, K only on ep.rx_out.
    w, v, pre, vac = gth81_w4_check(trace(10, 1200, poke={500: {"rc.tx": (0x1234, 0)},
                                                          501: {"ep.rx_out": (0, 0b10)}}))
    assert v == [("rc.tx", 500, 0x1234, 0), ("ep.rx_out", 501, 0, 0b10)], f"SELFTEST B {v}"
    # C. nonzero only before L0: no violation, one pre-L0 count.
    w, v, pre, vac = gth81_w4_check(trace(10, 1200, poke={5: {"rc.rx_in": (1, 0)}}))
    assert not v and pre["rc.rx_in"] == 1, f"SELFTEST C {v} {pre}"
    # D. no L0 at all, and an L0 too short: both are vacuity, never a pass.
    assert gth81_w4_check(trace(50, 0))[3], "SELFTEST D no L0"
    assert gth81_w4_check(trace(10, 999))[3], "SELFTEST D short L0"
    # E. one early outlier ST_L0 sample on the RC must not open its window early.
    w, v, pre, vac = gth81_w4_check(trace(10, 1200, rc_pre_outlier=True,
                                          poke={3: {"rc.tx": (7, 0)}}))
    assert w["rc"][:2] == (11, 1210) and not v and pre["rc.tx"] == 1, f"SELFTEST E {w} {v}"


class GTH81SeamCapture:
    """Raw capture, every cycle from before bring-up: both LTSSM states and, at
    each of the six points, lane 0's upper data half [31:16] and upper K pair
    [3:2] of the 32/4 container. Every signal is a bare read after RisingEdge,
    so the states and the halves share one phase.

    The RC's state is pcie_rc_top.ltssm_debug_state[19:0] and the EP's is the
    bench's ep_ltssm_state_o; both come from pcie_ltssm_downstream's
    ltssm_state_o.
    """

    def __init__(self, dut):
        """Handles on the six capture points and the two LTSSM states."""
        rc, ep = dut.u_rc.u_phy, dut.u_ep.gen_integrated_gen1_phy
        self.st_rc, self.st_ep = dut.u_rc.ltssm_debug_state, dut.ep_ltssm_state_o
        self.h = {
            "rc.tx": (rc.phy_transmit_inst.scr_data_out, rc.phy_transmit_inst.scr_data_k_out),
            "rc.rx_in": (rc.phy_receive_inst.desc_data_in, rc.phy_receive_inst.desc_data_k_in),
            "rc.rx_out": (rc.phy_receive_inst.descrambler_data, rc.phy_receive_inst.descrambler_data_k),
            "ep.tx": (ep.phy_transmit_inst.scr_data_out, ep.phy_transmit_inst.scr_data_k_out),
            "ep.rx_in": (ep.phy_receive_inst.desc_data_in, ep.phy_receive_inst.desc_data_k_in),
            "ep.rx_out": (ep.phy_receive_inst.descrambler_data, ep.phy_receive_inst.descrambler_data_k),
        }
        self.samples = []
        self.stop = False

    async def run(self, clk):
        """Append one sample per cycle until stop is set."""
        n = 0
        while not self.stop:
            await RisingEdge(clk)
            n += 1
            vals = {p: ((int(d.value) >> 16) & 0xFFFF, (int(k.value) >> 2) & 0x3)
                    for p, (d, k) in self.h.items()}
            self.samples.append((n, int(self.st_rc.value) & 0xFFFFF,
                                 int(self.st_ep.value) & 0xFFFFF, vals))


@cocotb.test()
async def fullstack_gth81_w4_dropped_half_is_zero_at_both_conversion_points(dut):
    """On every L0 cycle, in both stacks, the upper half of the 32/4 symbol
    container is zero at both conversion points.

    Transmit: the PIPE port takes scr_data_out[15:0]; the test reads the
    dropped data [31:16] and K [3:2]. Receive: the port is zero-extended into
    desc_data_in, whose upper half is zero by construction and is asserted all
    the same, and the same half of the descrambler's output is read, where a
    scrambler that wrote the unused bytes would show. Bits [31:16] are Gen3-only
    (PG239, Table 5 and Table 7). gen1_scramble handles only bytes below
    pipe_width >> 3 and lane_management fills only those bytes, so zero is
    expected everywhere.

    The window per stack is the longest contiguous ST_L0 run of that stack's own
    LTSSM, sampled from before bring-up; it must dominate the stack's ST_L0
    samples and last at least GTH81_L0_MIN cycles (non-vacuity). Nonzero
    samples outside the window (training) are logged, not asserted. The run is
    bring-up and one full enumeration, closed on enum_done.
    """
    gth81_w4_selftest()
    cap = GTH81SeamCapture(dut)
    task = cocotb.start_soon(cap.run(dut.clk_i))
    await bring_up(dut)
    r = await run_enumeration_fs(dut)
    _log_enum_fs(dut, r)
    await RisingEdge(dut.clk_i)
    cap.stop = True
    await RisingEdge(dut.clk_i)
    task.kill()
    windows, violations, pre, vacuity = gth81_w4_check(cap.samples)
    dut._log.info("GTH81_W4 samples=%d windows=%s", len(cap.samples), windows)
    dut._log.info("GTH81_W4 nonzero outside L0 (reported): %s", pre)
    for v in violations[:16]:
        dut._log.info("GTH81_W4|violation|%s|%d|%04x|%x", *v)
    assert r["enum_done"] and not r["enum_error"], (
        f"enumeration did not complete (code {r['enum_error_code']})")
    assert not vacuity, f"NON-VACUITY: {vacuity}"
    assert not violations, (
        f"{len(violations)} post-L0 samples with a nonzero dropped half, first "
        f"{violations[:4]} (point, cycle, data[31:16], k[3:2])")
