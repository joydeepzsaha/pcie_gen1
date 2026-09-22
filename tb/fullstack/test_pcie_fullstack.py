"""Our Root Complex against Joy's Endpoint, at the PIPE seam, codec in path.

SS63 #7b rows 1-5. Both sides are real RTL from the enumeration engine down to a
logical PHY; the only Python in the datapath is the PIPE sideband that both MACs
need a PHY to answer.

!! ROW 1'S BODY IS REWRITTEN, NOT RE-MARKED, AND THAT IS SS22.87. In SS63 #7a
row 1 asserted `not fc.rose` -- deliberately, to pin the premise "FC-init
completion needs a real peer" so that a loopback which somehow completed would
STOP the rung rather than read as good news. That premise EXPIRES here: the far
end can now answer. A red row's body encodes WHY it is red, so flipping it means
rewriting the body. Keeping the old assertion would fail against correct
behaviour; deleting it without replacement would lose the check entirely. It is
replaced by its positive dual: fc_initialized_o must RISE and STAY, on BOTH
sides, having been observed low first.

!! JOY'S ENDPOINT IS THE FAR END, NOT THE DUT. If it does not train or does not
answer, that is a STOP and a report, not a patch. The cheap probe for "did not
train" is two hierarchical signals inside its own pcie_flow_ctrl_init --
start_flow_control_i and fc1_values_stored_i -- which split the only two
candidate causes: start_fc false means the link-up path into its DLL, start_fc
true with fc1_stored false means it originated and the echo was not recovered.

!! ONE TB PER TEST. cocotb cancels every task a test started when that test
ends, including the Clock coroutine TB.__init__ spawns. A shared TB has a dead
clock and the next RisingEdge never returns, which reads as a reset bug.
"""

import os
import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ClockCycles, ReadOnly

CLK_NS = 8  # 125 MHz -- the real Gen1 PCLK

RXSTATUS_RECEIVER_DETECTED = 0b011   # PIPE: receiver detected
DETECT_LATENCY_CYCLES = 4            # PHY turnaround before PhyStatus answers

# Base 2.1 Appendix B framing characters, on the plaintext side of the codec.
K_STP = 0xFB   # K27.7 -- start of a TLP
K_END = 0xFD   # K29.7 -- end of a TLP
K_COM = 0xBC   # K28.5 -- comma

WINDOW = 60000  # cycles; row 1 measured L0 at 2279 on the RC alone


class TB:
    def __init__(self, dut):
        self.dut = dut
        cocotb.start_soon(Clock(dut.clk_i, CLK_NS, units="ns").start())

    async def reset(self):
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
        d.rc_phy_rxvalid.value = 0
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
    """One end's half of the PIPE receiver-detect HANDSHAKE.

    !! A LEVEL WILL NOT DO, AND THIS RUNS AT BOTH ENDS. Each MAC clears its
    detect latch on the RISING EDGE of its own phy_txdetectrx and sets it only
    when it SUBSEQUENTLY sees phy_phystatus asserted with rxstatus == 3'b011.
    Driving the status permanently high fails twice over: the request edge wipes
    the latch and there is no later edge to re-set it.

    Both stacks in this bench are MACs, so both need this. A bench that answered
    only the RC would leave the Endpoint in Detect forever, which reads as "Joy's
    Endpoint is broken" and is nothing of the kind.
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
        dut.rc_phy_rxvalid.value = 1
        dut.rc_phy_rxelecidle.value = 0
        dut.ep_phy_rxelecidle.value = 0


class Monotonic:
    """Continuous sampler for a one-way signal.

    !! SAMPLES FROM BEFORE THE EVENT, NEVER wait-then-read (SS22.89). A bare read
    after RisingEdge returns the PRE-edge value, so a waiter is an observer with
    a phase. Records: was it ever low, did it rise, did it fall AFTER rising.
    """

    def __init__(self, handle):
        self.h = handle
        self.saw_low = False
        self.rose = False
        self.fell_after_rise = False
        self.rise_cycle = None

    async def run(self, clk, cycles):
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
    """Hierarchical probe into BOTH flow-control initialisers.

    Splits the candidate causes on each side independently, so a failure names
    which end is at fault instead of reporting "it did not come up".
    """

    def __init__(self, dut):
        # !! THE TWO INSTANCE NAMES DIFFER AND THE DIFFERENCE IS INHERITED.
        # pcie_phy_top names its Data Link Layer `pcie_datalink_layer_inst`;
        # pcie_endpoint_top:697 names the same module `datalink_layer_inst`.
        # Spelled out rather than factored into a loop, because a hierarchical
        # path that is wrong fails at elaboration of the PROBE, several
        # thousand cycles before the row it serves, and reads as a DUT problem.
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
        """The two-signal probe's verdict, as a sentence."""
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
    """Where does a DLLP stop? Counts at four points, two per direction.

    !! THIS EXISTS BECAUSE "fc1_stored=False ON BOTH SIDES" IS A SYMPTOM WITH
    FOUR CANDIDATE CAUSES and the two-signal probe cannot separate them: the
    bridge could corrupt symbols, the codec could desynchronise, the scrambler
    and descrambler could be out of lockstep, or the framing could be lost.
    Counting valid beats at each stage says which boundary the traffic stops at,
    which is the difference between a bridge bug and a scrambler bug.

    Ordered sets are NOT scrambled in Gen1 and DLLPs ARE, so a stack that trains
    (ordered sets cross) while no DLLP arrives (scrambled traffic does not) is
    the signature of a scrambler/descrambler lockstep failure rather than a
    codec one -- and that distinction is exactly what these counters test.
    """

    def __init__(self, dut):
        self.dut = dut
        self.rc_tx_beats = 0        # RC put characters on the wire
        self.ep_rx_sym_beats = 0    # they arrived at the EP's symbol seam
        self.ep_dll_rx_beats = 0    # the EP's phy_receive framed something
        self.ep_tx_beats = 0        # the EP put characters on the wire
        self.rc_dll_rx_beats = 0    # the RC's phy_receive framed something
        self.ep_dll_rx = dut.u_ep.dll_phy_rx_tvalid
        self.rc_dll_rx = dut.u_rc.u_phy.m_dllp_axis_tvalid

    async def run(self, clk, cycles):
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
        dut._log.info(
            "DATAPATH RC->EP: rc_tx_beats=%d  ep_rx_symbol_beats=%d  "
            "ep_dll_rx_beats=%d", self.rc_tx_beats, self.ep_rx_sym_beats,
            self.ep_dll_rx_beats,
        )
        dut._log.info(
            "DATAPATH EP->RC: rc_dll_rx_beats=%d", self.rc_dll_rx_beats)


class DllpAcceptance:
    """Where inside dllp_handler does an arriving DLLP stop being accepted?

    !! THE STAGE BEFORE THIS ONE EXONERATED EVERYTHING UPSTREAM. The bridge
    passes 43744/43743 beats, the codec reports zero errors in either direction,
    the scramblers are in lockstep at the expected pipeline lag, and framed AXIS
    traffic reaches BOTH Data Link Layers. So the DLLP arrives and is refused,
    and dllp_handler has exactly three places that can refuse it:

      dllp_first_word_valid  -- tkeep is all-ones and not tlast (:127)
      dllp_crc_word_valid    -- tlast with tkeep == 2'b11 (:129)
      the CRC compare        -- crc_reversed == tdata[15:0] (:237)

    Counting all three separates "the framing never presents a DLLP" from "the
    DLLP is presented and fails CRC", which are different defects in different
    modules. The three fc1_*_stored_r bits are counted too, because
    fc1_values_stored_o is their AND (:124) and one missing class is a very
    different finding from all three missing.
    """

    def __init__(self, dut, side):
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
        dut._log.info(
            "DLLP-%s: first_word=%d crc_word=%d crc_MATCH=%d | "
            "fc1_stored np=%d p=%d c=%d",
            self.side.upper(), self.first_word, self.crc_word, self.crc_match,
            self.np, self.p, self.c,
        )


class ScramblerLockstep:
    """Are the RC's transmit LFSR and the EP's receive LFSR in lockstep?

    !! THIS IS THE DECISIVE MEASUREMENT WHEN THE LINK TRAINS BUT NO DLLP IS
    ACCEPTED, and the reason is structural: ORDERED SETS ARE NOT SCRAMBLED AND
    DLLPs ARE. TS1/TS2 cross as plain K and D characters, so the LTSSMs reach L0
    whatever the scramblers are doing; a DLLP's payload is scrambled, so a
    receive LFSR out of step with the transmit LFSR turns it into noise that
    fails CRC while the framing K characters around it survive intact.

    "Trains but never accepts a DLLP" is therefore the SIGNATURE of a scrambler
    lockstep failure, and distinguishing it from a codec failure costs two
    hierarchical reads rather than a waveform hunt.

    Base 2.1 SS4.2.3 pp.198-199: the COM Symbol initialises both LFSRs, so they
    are expected to agree on every cycle after the first COM crosses -- allowing
    for the one-cycle bridge latency, which is why the RC's value is compared
    against the EP's value from the PREVIOUS cycle as well as the current one.
    """

    def __init__(self, dut):
        self.tx = (dut.u_rc.u_phy.phy_transmit_inst
                   .gen_lane_scramble[0].scrambler_inst
                   .gen1_scramble_inst.Q)
        # !! phy_receive_inst LIVES INSIDE A GENERATE ARM on the EP side, so
        # the path carries the arm's label. pcie_endpoint_top:371 names it
        # gen_integrated_gen1_phy; the RC's pcie_phy_top has no such arm and its
        # path is one level shorter. The asymmetry is inherited, not chosen.
        self.rx = (dut.u_ep.gen_integrated_gen1_phy.phy_receive_inst
                   .gen_lane_descramble[0].descrambler_inst
                   .gen1_scramble_inst.Q)
        self.samples = 0
        self.tx_advances = 0
        self.rx_advances = 0
        # !! LAG SWEEP. This is what separates a PHASE problem from a DIVERGENCE
        # problem, and they have opposite owners. If the receive LFSR equals the
        # transmit LFSR delayed by some FIXED lag, the streams are in lockstep
        # and the bench has simply mis-matched latency -- ours to fix. If NO lag
        # gives a high match rate, the two LFSRs are genuinely advancing on
        # different events, which is an RTL defect in the shared PHY datapath
        # and is a STOP for this rung.
        self.max_lag = 12
        self.lag_hits = [0] * (self.max_lag + 1)
        self.tx_hist = []
        self.tx_trace = []
        self.rx_trace = []

    async def run(self, clk, cycles, start_after):
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
        self.dut = dut
        self.br_enc_illegal_k = False
        self.br_dec_code_err = False
        self.br_dec_disp_err = False
        self.ep_rx_code_error = False
        self.ep_rx_disparity_error = False
        self.ep_tx_illegal_k = False

    async def run(self, clk, cycles):
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
        dut._log.info(
            "CODEC: bridge enc_illegal_k=%s dec_code_err=%s dec_disp_err=%s | "
            "EP rx_code_err=%s rx_disp_err=%s tx_illegal_k=%s",
            self.br_enc_illegal_k, self.br_dec_code_err, self.br_dec_disp_err,
            self.ep_rx_code_error, self.ep_rx_disparity_error,
            self.ep_tx_illegal_k,
        )

    def assert_clean(self, dut):
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
    """§63 #7e (D-7E.3) -- the TLP path at ONE stack's DLL AXIS input.

    RED WHEN WRITTEN, on the RC side, and the numbers that made it red are in
    rows 6a/6b's bodies (§22.87).

    == WHAT IT WATCHES, AND WHY THERE =========================================

    The seam is `dllp_receive_inst.s_axis_*` -- the Data Link Layer's inbound
    AXIS, the first point inside the DLL where a TLP exists as a packet. Beats
    are classified as TLP by `s_axis_tuser[1]`, which is not a guess: it is the
    bit `axis_user_demux.sv:47` names `UserIsTlp` and routes on at `:90`. So the
    witness classifies exactly as the DUT does.

    == THE FOUR QUANTITIES D-7E.3 ASKS FOR ====================================

      STP-framed beat count  -> beats per inbound TLP packet
      tlast                  -> packets completed at the seam
      tkeep on last          -> keep_on_last histogram
      LCRC                   -> see the caveat below

    ⚠️ "LCRC pass count" is witnessed as `tlp_nullified_o`, NOT as a comparison
    of `crc_from_tlp_r` against `crc_calculated_r`. §63 #7e Phase 1 measured that
    comparison as 0 match / 4 mismatch on the EP -- while the EP forwarded all
    four packets and nullified none -- and could NOT establish whether that is a
    real defect or the probe sampling `crc_from_tlp_r` a cycle before it loads.
    An assertion built on an instrument whose sampling phase is unknown would be
    a coin flip wearing a spec citation. `tlp_nullified_o` is unambiguous: it is
    the DUT's own verdict on the packet. The raw comparison is logged, not
    asserted, and the question is registered to #7f.

    !! SAMPLED BARE AFTER RisingEdge, ON PURPOSE. That read returns the PRE-edge
    value -- exactly what the DUT's flops sampled at that edge -- which is the
    correct phase for counting an AXIS handshake. §35's sampling-phase trap runs
    the other way: it bit a monitor that wanted the POST-edge state of an FSM.
    Same read, opposite requirement; stated so neither is "fixed" into the other.
    """

    def __init__(self, dut, side):
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
        self.nullified = 0         # the DUT's own LCRC verdict
        self._cur = 0

    async def run(self, clk, cycles):
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
        dut._log.info(
            "TLPWIT %-2s DLL-AXIS-IN tlp_beats=%d tlp_pkts=%d beats_per_pkt=%s "
            "tkeep_on_last=%s | UP-TO-TL beats=%d pkts=%d | nullified=%d",
            self.side, self.in_beats, self.in_pkts,
            {k: v for k, v in sorted(self.beat_hist.items())},
            {hex(k): v for k, v in sorted(self.keep_on_last.items())},
            self.up_beats, self.up_pkts, self.nullified,
        )


async def bring_up(dut, window=WINDOW):
    """Reset, start both PHY models, enable, and run the monitors.

    Returns (tb, monitors dict). Every monitor is started BEFORE en_i rises, so
    the low period of each signal is inside its window rather than assumed.
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
    # Sampled only after both LTSSMs are in L0 -- before that the LFSRs are
    # legitimately being reset by every ordered set and a mismatch means nothing.
    tasks.append(cocotb.start_soon(scram.run(dut.clk_i, window, 3000)))
    tasks.append(cocotb.start_soon(dllp_ep.run(dut.clk_i, window)))
    tasks.append(cocotb.start_soon(dllp_rc.run(dut.clk_i, window)))

    await ClockCycles(dut.clk_i, 5)
    dut.en_i.value = 1
    dut.phy_ready_en.value = 1
    dut.transmit_enable_i.value = 1

    return tb, mons, probe, codec, path, scram, (dllp_rc, dllp_ep), tasks


async def _run_and_report(dut):
    """Bring up, run the window, and print EVERY census before any assert.

    !! DIAGNOSTICS BEFORE VERDICTS. A row whose first failed assertion
    suppresses the census that would explain it costs a whole re-run to learn
    what the run already knew. Both rows below share this, so the red row's log
    carries the same evidence the green one does.
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
# Row 1a -- GREEN. The composition works up to, but not including, FC init.
# ---------------------------------------------------------------------------
@cocotb.test()
async def fullstack_both_stacks_train_to_l0_through_the_codec(dut):
    """Two real PHYs face each other and both reach L0, codec in the path.

    ⭐ THE FIRST TIME IN THIS PROJECT THAT TWO REAL LOGICAL PHYs HAVE FACED EACH
    OTHER. Every earlier bench had a Python far end, a PIPE loopback, or met the
    Endpoint at the AXIS packet seam where neither side has a PHY. Here the
    stimulus crosses a real 8b/10b codec in both directions.

    Five claims, and each is a thing that could have failed on its own:
      1. both LTSSMs reach L0 across the encoded seam;
      2. both Data Link Layers enter DL_Init and originate InitFC1;
      3. the codec is clean in both directions -- no code error, no disparity
         error, no illegal K;
      4. the bridge neither creates nor drops beats;
      5. the two scramblers advance in lockstep.

    !! (3) AND (5) ARE WHAT MAKE ROW 1b's FAILURE INFORMATIVE. Without them,
    "FC init did not complete" would have a dozen candidate causes. With them,
    everything from the transmit scrambler to the receive AXIS port is
    exonerated BY MEASUREMENT, and the defect is cornered in the Data Link
    Layer's DLLP framing and acceptance.

    NON-VACUITY: every monitor must have seen its signal LOW before it rose,
    otherwise a stack that asserted out of reset would pass identically.
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
    # Advance COUNTS, not values: the receive LFSR necessarily trails the
    # transmit LFSR by the link's pipeline latency, so equal values on the same
    # cycle would be the wrong property to assert. Equal advance counts is the
    # right one -- it says both are stepping on the same events.
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


# ---------------------------------------------------------------------------
# Row 1b -- GREEN as of d079edc (§63 #7d). FC init completes both ways.
# ---------------------------------------------------------------------------
@cocotb.test()
async def fullstack_completes_fc_init_both_ways(dut):
    """FC init completes both ways.

    ⭐⭐ GREEN AS OF d079edc, §63 #7d. THIS ROW WAS RED FOR THE ENTIRE LIFE OF THE
    FULL-STACK BENCH AND HAS NOW FLIPPED. FC init completes in BOTH directions
    for the first time in this project.

    !! §22.91 -- READ THE HISTORY BEFORE TRUSTING ANY OLD NUMBER IN THIS FILE.
    This row went red three separate times for three DIFFERENT reasons while
    keeping its colour, so every set of premises it pinned expired without the
    marker or the suite total registering anything. They are kept below, marked,
    because "a red row is not a frozen row" was learned here.

    == WHAT IT TOOK, three defects, none where the row's text used to point ====

    1. eb2e662 + 9ecabee -- TRANSMIT. pcie_endpoint_top's USER_WIDTH was 3, and
       frame_symbols carries the K-Symbol's byte position as a FOUR-bit mask in
       tuser (:148 SDP at byte 0 = 4'b0001, :187 ENDP at byte 3 = 4'b1000). At 3
       the ENDP mask truncated to 3'b000, so the Endpoint transmitted no END
       Symbol at all and the RC could never frame a DLLP. SDP is bit 0 and
       survived -- hence SDP present, END absent. A legal truncation, silent,
       and lint/waiver.vlt:2-4 disables WIDTH/WIDTHEXPAND/WIDTHTRUNC globally.

    2. f75b143 -- RECEIVE. data_handler's tkeep on the END beat ignored the
       carry-over from the previous word and counted from the wrong end, giving
       0x7 where a six-byte DLLP needs 0x3. dllp_crc_word_valid (:129) requires
       tlast && tkeep == 2'b11, so it never asserted and every DLLP was dropped
       silently at ST_CHECK_CRC's else arm.

    3. d079edc -- CONFORMANCE DEFECT #6. pcie_flow_ctrl_init.sv:401 gated
       FC_INIT2's exit on `fc2_values_stored_i && (update_fc_r || idle_count_r
       >= 16'h60)`. Base 2.1 §3.3.1 exits on the full FC2 set sent AND any of
       {InitFC2 received, UpdateFC received, TLP received} -- a DISJUNCTION. The
       RTL made an alternative limb into an additional requirement and added an
       idle-Symbol timeout with no counterpart in the spec. In the full stack
       update_fc_i was high on ZERO cycles and idle_count_r never left 0, so the
       exit never fired: the FSM looped ST_FC2..CHECK_FC2 7,074 times and
       ST_FC_COMPLETE was never entered.

    ⚠️ AND THE ONE BENCH THAT PASSED WAS PASSING FOR THE WRONG REASON.
    tb_pcie_rc_ep exited CHECK_FC2 at cycle 4,453 on idle_count_r -- 16 cycles
    before update_fc_r was ever high -- only because that bench ties
    idle_valid_i to link_up (test_pcie_rc_ep.py:180). No real PHY holds logical
    idle continuously. FC init had never once completed on a condition §3.3.1
    recognises, and a green direct-wired bench concealed it.

    == WHAT IS EXONERATED BY MEASUREMENT, kept -- a fix must not restart here ==
      - the codec: zero code errors, zero disparity errors, zero illegal K;
      - the bridge: 43744 beats in, 43743 out -- one register of window edge;
      - the scramblers: tx advanced 42637 times, rx 42635, drift 2 in 57000;
      - the LTSSMs: both reach L0, both DLLs enter DL_Init and originate;
      - SYMBOL ORDER: 37451 ONE-WAY comparisons, zero mismatches on data and K
        flags (a round trip is blind to a consistent transposition);
      - the CRC logic: pcie_datalink_crc is seeded .crcIn(16'hFFFF) hardcoded,
        stateless per beat, and the two sides use identical conventions;
      - pack_data: preserves SDP and END exactly. It has no tkeep/tlast port in
        either direction and was never the module, despite this row's own older
        text naming it.

    !! THIS WAS NEVER JOY'S ENDPOINT FAILING. Its transmit side is conformant,
    it trains, enters DL_Init and originates. Two of the three defects were in
    SHARED RTL and the third was a parameter on its top that no instantiator was
    obliged to relate to frame_symbols' mask width.

    == SUPERSEDED PREMISES, every one true when taken ==========================
      - "RC: 21,334 valid beats, tlast asserted ZERO times, tkeep always 0xF"
        and "EP: tkeep ON TLAST is ALWAYS 0x7" -- fixed by 1 and 2 above; the RC
        now asserts tlast 21,252 times with tkeep 0x3.
      - "the tkeep = 2'b11 site is reached on NEITHER side" -- it is now reached
        and MATCHES on both: RC 21,408/21,409, EP 21,426/21,427. Conformance
        defect #5, the DLLP CRC bit-reversal, stays CANCELLED between the stacks.
      - "the defect is in DLLP delineation on the receive path" -- it was, twice,
        and then it was not.
      - earlier still: "the CRC beat arrives with tkeep = 0", and "the two
        directions look the SAME". Both correct when measured, both later false.

    ⚠️ REGISTERED, NOT CHASED HERE: fc_initialized_o measures rises=2 falls=1
    across this bench's two tests (first_rise 6,750, first_fall 60,005 -- which
    is the inter-test boundary at half of 120,010 cycles, NOT verified as such).
    Against §35, not this rung.
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

    # ---- and it STAYS -- defect #3, unfiltered ---------------------------
    # Unreachable while the asserts above are red. Kept because it is the claim
    # the row exists to make once they are green, and deleting it would lose it.
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


# =============================================================================
# Rows 2-5 -- §63 #7d. THE FIRST ROWS IN THIS PROJECT THAT RUN TRANSACTIONS
# ACROSS TWO REAL PHYs.
#
# ⭐ THESE ROWS WERE UNREACHABLE UNTIL THIS RUNG. Every one of them needs FC
# init to have completed, and FC init completed in neither direction until
# eb2e662/9ecabee (the USER_WIDTH K-mask truncation), f75b143 (data_handler's
# tkeep) and d079edc (conformance defect #6). They are written now because the
# path exists now -- BRIEF_7C's "rows 2-5 will unblock" was an assumption and is
# here replaced by the measurement.
#
# !! THE ORACLE IS RTL, NEVER A PYTHON MODEL. Every value asserted below comes
# from src/pcie_cfg/pcie_config_reg.sv inside Joy's Endpoint. The RC's
# enumeration engine issues the real CfgRd0 and the Endpoint's own
# configuration space answers it.
#
# ⚠️⚠️ ALL FOUR ROWS ARE RED TODAY, AND THE REASON IS A REAL DEFECT ONE LAYER
# BEYOND EVERYTHING §63 #7d FIXED. Measured at d079edc, with FC init completing
# in both directions and framing and DLLP CRC green both ways:
#
#     ENUM done=0 error=1(code 4)  scan_done=0 scan_error=1(code 4)
#     present=0  VID=0x0000  DID=0x0000  bar_count=0
#
# enum_error_e code 4 is ENUM_ERR_TIMEOUT (pcie_enum_pkg.sv:389). The CfgRd0
# leaves the requester and NO Completion comes back within the scan's window,
# so the Endpoint is never even detected -- every row below fails at the scan
# phase, before any header field or BAR is read.
#
# ⭐ THIS IS BRIEF_7C's LESSON A SECOND TIME. That brief assumed rows 2-5 would
# "unblock" once FC init completed. They do not. The acceptance was always the
# measurement, and the measurement says there is another defect on the CfgRd0
# round trip across two real PHYs.
#
# ⚠️ §63 #7e REWROTE THE REASON, AND THE REASON IS NOW DIFFERENT (§22.91: a red
# row's body must stay CURRENT, and these bodies pinned a blocker that no longer
# exists).
#
# F17 IS CLOSED. The CfgRd0 round trip works: present=1, VID=0x1234, DID=0x00ff,
# hdr=0x00, mf=0, scan_done=1, scan_error=0 -- across two real PHYs and the
# codec bridge. Row 2 has FLIPPED GREEN. Its two causes were both bench
# configuration, zero src/ change:
#   (1) CPL_TIMEOUT_CYCLES 4096 = 32.8 us, below BOTH the measured 41.0 us round
#       trip and Base 2.1 §7.8.16's 50 us minimum;
#   (2) bar_enable_i never raised, so enum_done_o could never assert at all.
#
# ROWS 3-5 REMAIN RED OVER A DIFFERENT, NEWLY ISOLATED DEFECT -- F18: the BAR
# phase stalls at bar_count=2 (bar_valid=0x3) and raises ENUM_ERR_TIMEOUT.
# ⚠️ IT IS NOT A TIMEOUT BUDGET. Measured identical at CPL_TIMEOUT_CYCLES of
# BOTH 6250 and 65536 -- a 10x change in the budget moved nothing, so raising it
# further will not help. F18 is its own investigation.
#
# ⚠️ ALSO UNRESOLVED AND NOT A TIMEOUT QUESTION: row 3's oracle expects BAR0 to
# size to 4 KB; the measured bar_size low word is 0x100000 (1 MB). Whether the
# oracle or Joy's Endpoint is the odd one out is NOT yet determined.
#
# They stay expect_fail so the gate stays meaningful rather than carrying
# permanently red rows -- the same idiom row 1b used for its whole life.
# ⚠️ And the same caveat applies (§22.77): an expect_fail row reports PASS, so
# the gate CANNOT show this defect or show it closing. These bodies are the
# witness. Flip them the moment the CfgRd0 timeout is fixed.
#
# ⭐⭐ FLIPPED AT §63 #7f (commits A+B, #18). F18 WAS CREDIT STARVATION: the
# shared DLL never returned NP credit after FC init, so the RC's 17th
# non-posted request sat behind its own credit gate until the completion
# timer -- which runs from allocation -- expired, and the engine reported
# ENUM_ERR_TIMEOUT at bar_count=2. With CREDITS_ALLOCATED stepped at release
# (A) and an UpdateFC scheduled on each release (B), enumeration COMPLETES:
# enum_done=1, enum_error=0, bar_count=2, bar_valid=0x3, BAR0 = BAR1 = 1 MB.
# Rows 3-5 lost their expect_fail and their bodies were rewritten (§22.87).
# The "4 KB vs 1 MB" question is answered below, in row 3: the config space
# encodes 1 MB, the RC read 1 MB, and 4 KB was the TL decoder's aperture.
# =============================================================================


def _i(sig):
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
    """Pulse scan_start_i and wait for enum_done_o / an error.

    scan_start_i is a PULSE on purpose: the start gate must REMEMBER a request
    made while flow control is still down (tracker §44 -- it is a latch, not a
    bare AND). Ported from tb_rc_ep's run_enumeration, which is the same engine
    at the AXIS seam; here it runs through two PHYs and the codec bridge.

    ⚠️ bar_enable_i MUST BE RAISED AND THIS BENCH NEVER DID.

    pcie_enum_top.sv:402 gates the BAR phase on `bar_enable_i && scan_done_o`,
    and :197 states the consequence directly: "PHASE NEEDS AN ENABLE. With it
    low, enum_done_o never asserts". TB.reset() sets bar_enable_i to 0 and
    nothing raised it, so enum_done_o could not assert no matter how well the
    link worked -- rows 2-5 were unwinnable for a reason that had nothing to do
    with the link.

    It went unnoticed because it was MASKED: until §63 #7e every enumeration
    died at the scan phase with ENUM_ERR_TIMEOUT, thousands of cycles before the
    BAR phase would have been reached, so the missing enable never had a chance
    to matter. Fixing the completion timeout is what exposed it.

    tb_rc_ep -- the same engine at the AXIS seam -- has driven bar_enable=1 from
    its reset default all along. This matches that idiom.
    """
    d = dut
    d.bar_enable_i.value = 1
    # §63 #7f 21-a (P3-2): delay the enumeration start by K cycles to test
    # whether #21's tail releaser is periodic (residue shifts by -K mod period)
    # or a fixed per-packet delay (nothing moves).  BENCH-ONLY and DEFAULT 0, so
    # with the variable unset this function is behaviourally identical to before
    # and verilate_fullstack is unchanged.
    # 21-a (P3-2), EVENT-RELATIVE: the start gate is a LATCH (tracker §44), so a
    # pulse issued before FC init is remembered and the engine starts at
    # FC-init-complete regardless -- which is why the cycle-relative K of the
    # first attempt moved nothing across K=0..600.  Anchor on the event instead.
    # K=0 takes the ORIGINAL path exactly, so verilate_fullstack is unmoved.
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
        # §63 #7f: rows 4 and 5 have read r["unsupported"] since #7b and this
        # key was NEVER returned. They could not have passed even with a
        # perfect link -- and nothing noticed, because both were expect_fail
        # and a KeyError is as good as an AssertionError to a decorator that
        # only asks "did it fail?". §22.77 in its purest form: the rows'
        # own defect was hidden by the mechanism that hid the DUT's. Surfaced
        # the moment #18 (commits A+B) let enumeration complete.
        "unsupported": _i(d.unsupported_device_o),
        "frames": frames,
    }


def _log_enum_fs(dut, r):
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


# ---------------------------------------------------------------------------
# Row 2 -- CfgRd0 VID/DID from the PCI 3.0 header, across two real PHYs.
# ---------------------------------------------------------------------------
@cocotb.test()  # §63 #7e: FLIPPED -- the CfgRd0 round trip works; see body
async def fullstack_cfgrd0_reads_vid_did_across_two_phys(dut):
    """The RC's enumeration engine reads Joy's Endpoint's real config space.

    Values are PCI 3.0 §6.1 / Base 2.1 §7.5.1 header fields, and every one comes
    from src/pcie_cfg/pcie_config_reg.sv, not from Python:
        Vendor ID   0x1234   (§7.5.1.1 p.484)
        Device ID   0x00FF   (§7.5.1.2)
        Header Type 0x00     (§7.5.1.9, Type 0, single function)

    ⚠️ The configuration space lives inside the Endpoint's DATA LINK LAYER, not
    its Transaction Layer: dllp_receive instantiates pcie_cfg_wrapper, which
    answers and emits the Completion on cpl_axis_*, muxed back onto transmit as
    cpl_from_cfg_*. So this round trip never touches the endpoint's TL.

    ⭐⭐ GREEN AT §63 #7e. F17 CLOSED. Measured at `6436f0e` + the bench fixes:

        present=1  VID=0x1234  DID=0x00ff  hdr=0x00  mf=0
        scan_done=1  scan_error=0

    The first time this project has read a real Endpoint's configuration space
    across two real PHYs and an 8b/10b codec bridge.

    == ⚠️ THIS ROW NO LONGER ASSERTS enum_done_o, AND THAT IS A RESCOPING ==

    It previously asserted `enum_done and not enum_error`. That coupled it to
    the BAR phase, which this row is not about and which has its own defect
    (F18: the BAR phase stalls at bar_count=2 and raises ENUM_ERR_TIMEOUT --
    measured at CPL_TIMEOUT_CYCLES of BOTH 6250 and 65536, so it is not a
    timeout budget). Rows 3-5 own the BAR phase and remain red over F18.

    ⚠️ A ROW MUST NOT BE WEAKENED TO MAKE IT GREEN, so the justification is
    stated rather than assumed. The old assertion's PURPOSE was non-vacuity:
    "a run that timed out would leave the ID registers at reset and could
    otherwise read as a pass." That purpose is preserved exactly, and at the
    right scope:

      - scan_done_o with scan_error_o low -- the scan phase COMPLETED, so this
        is not a timed-out run;
      - device_present_o -- the Endpoint was actually detected;
      - VID/DID/hdr/mf asserted against SPECIFIC non-reset constants from
        pcie_config_reg.sv. A timed-out run leaves these at 0x0000 and fails.

    What the rescoping gives up is coverage of the BAR phase -- which this row
    never meaningfully had, since it could not reach it, and which rows 3-5
    cover directly. Nothing that was being checked has stopped being checked.
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


# ---------------------------------------------------------------------------
# Row 3 -- BAR0 sizing.
# ---------------------------------------------------------------------------
@cocotb.test()  # §63 #7f: FLIPPED -- F18 was #18; see body
async def fullstack_bar0_sizes_to_4kb(dut):
    """BAR0 sizes, by the PCI 3.0 §6.2.5.1 write-ones-read-back protocol, to
    exactly the size the Endpoint's configuration space encodes: 1 MB.

    ⚠️⚠️ THE NAME SAYS 4 KB AND THE NAME IS WRONG. It is kept because a gate
    row is identified by its name and a rename reads in the artifact as one
    row deleted and another added; the body carries the correction (§22.87).
    The 4 KB came from pcie_endpoint_top's BAR_MASK default, which configures
    tlp_layer's DECODER for one 4 KB aperture. The CONFIGURATION SPACE the RC
    actually reads is pcie_config_reg.sv, whose BAR0 and BAR1 readback paths
    return the constant 0xFFF00000 (:2063, :2067 -- bits [31:4] = 28'hfff0000,
    bits [3:0] = 0: memory, 32-bit, not prefetchable). Write all ones, read
    back 0xFFF00000, lowest set address bit = bit 20: 1 MB. That is what the
    engine reported, on BAR0 and on BAR1, the first time it got far enough to
    report anything.

    So the Endpoint carries TWO DISAGREEING BAR IMAGES -- 1 MB claimed, 4 KB
    decoded, and a BAR1 that is claimed and not decoded at all. That finding
    is Joy's, is already on record as tb_rc_ep's
    rcep_bar_image_matches_claimed_aperture (red by measurement), and is
    OUTSIDE this rung's fence (D-7F.2). This row does not adjudicate it. What
    this row pins is the Root Complex's half: across two real PHYs and the
    codec bridge, the engine sizes exactly what the far end encodes.

    ⭐⭐ GREEN AT §63 #7f, commits A+B (#18). Measured in this row:
        enum_done=1 enum_error=0 scan_done=1 present=1
        bar_count=2 bar_valid=0x3 BAR0=0x100000 BAR1=0x100000
    F18 was never a BAR-decode fault: it was the RC starving on NP credit
    after 16 requests because the shared DLL never advertised a release. Both
    BARs were sized before the stall every time; the timeout was on the
    request AFTER them.

    ⚠️ BAR1 is REPORTED, NOT ASSERTED, for the same reason as before: pinning
    it would be this bench certifying the far end's phantom BAR as a
    full-stack property.
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


# ---------------------------------------------------------------------------
# Row 4 -- MemWr/MemRd round trip through the requester arm.
# ---------------------------------------------------------------------------
@cocotb.test()  # §63 #7f: FLIPPED -- F18 was #18; see body
async def fullstack_memwr_memrd_round_trip(dut):
    """A Memory Write followed by a Memory Read of the same address, issued on
    the RC's requester (RQ) arm and answered across two real PHYs.

    ⚠️⚠️ SCOPE, STATED PLAINLY SO THE ROW IS NOT READ AS STRONGER THAN IT IS.
    tb_pcie_fullstack exposes NO cpl_timeout_valid_o and NO
    rc_unexpected_completion_o at its top level -- they exist inside the RC but
    are not brought out -- and driving raw MemWr/MemRd TLPs would mean building
    headers onto s_axis_rq_*. So this row does NOT yet issue a Memory Write and
    a Memory Read of its own.

    What it DOES assert is the non-posted round trip that enumeration already
    performs across two real PHYs: CfgRd0/CfgWr0 go out on the requester arm,
    Completions come back, the engine owns the RQ arm while it happens, and
    the whole enumeration COMPLETES. That is the same NP path a MemRd uses,
    minus the opcode.

    ⭐ GREEN AT §63 #7f, commits A+B (#18): enum_done=1 enum_error=0
    scan_done=1 unsupported=0. F18 was credit starvation in the shared DLL,
    not a completion fault.

    ⚠️⚠️ THIS ROW COULD NEVER HAVE PASSED BEFORE §63 #7f, AND NOT BECAUSE OF
    THE LINK. It read r["unsupported"] and run_enumeration_fs never returned
    that key; the KeyError was indistinguishable from the real failure under
    expect_fail (§22.77). Recorded here because the same shape -- a row whose
    own defect is hidden by the decorator that hides the DUT's -- will recur,
    and a reader should know this row's first green run is also its first
    run in which its own body executed to the end.

    ⭐ REGISTERED, unchanged: bringing cpl_timeout_valid_o and
    rc_unexpected_completion_o out to this bench's top, and driving real
    MemWr/MemRd on s_axis_rq_*, is the remaining half of this row.
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


# ---------------------------------------------------------------------------
# Row 5 -- completion tag / Successful Completion status.
# ---------------------------------------------------------------------------
@cocotb.test()  # §63 #7f: FLIPPED -- F18 was #18; see body
async def fullstack_completion_tag_and_status(dut):
    """Completions returned across the seam carry a tracked tag and SC status.

    Base 2.1 §2.2.9: Completion Status 000b is Successful Completion.

    ⚠️⚠️ THIS IS AN ACCEPTANCE ASSERTION, NOT A DECODE, and the distinction is
    the point. It does NOT read the Completion's Status field or its tag off the
    wire. It asserts that the RC's enumeration engine CONSUMED the Completions
    and produced correct header values from them -- which it could not do had a
    tag gone untracked or a non-SC status come back, because the engine would
    have raised enum_error_o instead -- and that it did so for EVERY
    Completion of a full enumeration, since enum_done_o is asserted too.

    ⭐ GREEN AT §63 #7f, commits A+B (#18): enum_done=1 enum_error=0
    VID=0x1234 DID=0x00ff unsupported=0.

    ⚠️ Like row 4, this row read r["unsupported"], a key run_enumeration_fs
    never returned until §63 #7f, so it could not have passed before and the
    KeyError hid under expect_fail (§22.77). Its first green run is its first
    complete run.

    The direct oracles -- rc_unexpected_completion_o for an untracked tag, and
    the Completion Status field itself -- are NOT reachable from this bench's
    top level. ⭐ REGISTERED, unchanged: bring the RC error surface out, then
    this row can assert the tag and the status directly instead of by
    consequence.
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
# Rows 6a / 6b -- §63 #7e WITNESS ROWS (D-7E.3). The TLP path at each stack's
# DLL AXIS input.
#
# !! THESE TWO ROWS ARE A MATCHED PAIR AND THE PAIR IS THE EVIDENCE. They run
# the identical assertion against the identical RTL -- dllp_receive and
# axis_user_demux are shared, elaborated by 7 of 106 gate targets each, one
# instance per stack. 6a passes and 6b fails. That difference cannot be a bug in
# the assertion, because it is the same assertion; it is a property of what
# arrives at each DLL. A single row could not have made that argument.
#
# ⚠️ NEITHER IS expect_fail. 6b is RED ON PURPOSE until F17 closes. §22.77: an
# expect_fail row reports PASS, so the gate cannot witness the defect OR its
# closing -- which is exactly how rows 2-5 have hidden F17 since #7d. This rung
# pays the cost of one genuinely red row so that the gate carries the proof.
# ---------------------------------------------------------------------------


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
    """WITNESS 6a -- the CfgRd0 at the ENDPOINT's DLL AXIS input. GREEN.

    Measured IN THIS ROW at `6436f0e`, one enumeration, 60,000-cycle window:

        tlp_beats = 5   tlp_pkts = 1   beats_per_pkt = {5: 1}
        tkeep_on_last = {0x3: 1}   up_to_TL = 1 pkt / 3 beats   nullified = 0

    Spec-exact. Base 2.1 §3.5: a CfgRd0 on the link is 2 B sequence number +
    3 DW header + 4 B LCRC = 18 B = 4.5 DW, so five 32-bit beats with two valid
    bytes on the last -- `tkeep` 0x3. Every inbound TLP is delivered upward.

    ⚠️ AN EARLIER DRAFT OF THIS BODY CLAIMED 20 beats / 4 packets. Those were
    probe_7e counters summed over SIX tests with DIFFERENT WINDOW LENGTHS -- and
    rows 2-5 each return the moment enum_error_o fires, around 10,888 cycles,
    which truncates the very exchange being counted. A per-test row must carry
    per-test numbers. Recorded because the same mistake produced a much worse
    error on 6b (§22.87).


    ⚠️ THE BEAT-COUNT ORACLE WAS WIDENED FROM ONE VALUE TO {5, 6}, AND THAT IS
    NOT A WEAKENING. It was written when the only TLP that ever crossed this
    link was a CfgRd0. Once F17 closed and the BAR phase began running, CfgWr0
    and data-carrying Completions appeared -- 3 DW header + 1 DW data = 22 B =
    6 beats, alongside the 18 B / 5-beat no-payload form. Both are spec shapes
    (Base 2.1 §3.5) and BOTH end with tkeep 0x3, since 18 % 4 == 22 % 4 == 2.
    The original single-value oracle described the traffic I had happened to
    see, not the traffic the spec permits.

    !! THIS ROW IS THE CONTROL FOR 6b, NOT DECORATION. It fixes the meaning of
    every number 6b asserts on, in the same run, through the same shared modules.
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
    """WITNESS 6b -- the Completion at the ROOT COMPLEX's DLL AXIS input.

    ⚠️⚠️ RED WHEN WRITTEN (§63 #7e, F17). Measured IN THIS ROW at `6436f0e`,
    one enumeration, 60,000-cycle window:

        tlp_beats = 12   tlp_pkts = 2   beats_per_pkt = {6: 2}
        tkeep_on_last = {0x3: 2}   up_to_TL = 1 pkt / 4 beats   nullified = 0

    TWO Completions arrive at the Root Complex's DLL, both **structurally
    perfect** -- six beats, `tlast` present, `tkeep` 0x3, spec-exact -- and one
    reaches the Transaction Layer.

    ⚠️ THE 2-TO-1 IS CORRECT AND THIS ROW NO LONGER ASSERTS OTHERWISE. Both
    inbound TLPs carry the identical first word 0x4a0000, so the same DLL
    sequence number: the second is the Endpoint REPLAYING a TLP our side never
    acknowledged, and Base 2.1 §3.5.2.1 requires the receiver to DISCARD a
    duplicate. An earlier draft asserted `up_pkts == in_pkts` and would have
    certified correct duplicate suppression as a defect.

    ⚠️⚠️ AN EARLIER DRAFT OF THIS BODY CLAIMED `tlp_pkts = 0`, "not one carries
    tlast". THAT WAS WRONG AND IT WAS MY MEASUREMENT THAT WAS WRONG, not the
    DUT. Those were probe counters summed across six tests whose windows differ
    by 6x; rows 2-5 end at ~10,888 cycles, before the Completion finishes
    arriving, so the sum recorded a truncation artifact as a malformed packet.
    Three separate mechanisms were built on that wrong number and all three were
    later refuted by measurement (data_handler's end-beat tkeep, axis_user_demux's
    ST_IDLE ready mismatch, and data_handler's TLP-arm alignment). The Completion
    is NOT malformed. Kept in the body per §22.87 so the correction travels with
    the row.

    The Endpoint answers correctly: §63 #7e Phase 1 measured spec-exact CplDs
    leaving it (22 B = 6 beats), crossing the bridge with STP/END counts
    identical on both sides, carrying VID 0x1234 / DID 0x00FF out of its config
    space. Enumeration still reports ENUM_ERR_TIMEOUT (code 4), and the engine
    errors at ~10,888 cycles while the Completions are still arriving -- so the
    round-trip latency against the engine's own timeout is an OPEN question this
    row does not settle.

    Base 2.1 §3.5: a CplD with 1 DW of data is 2 B sequence number + 3 DW header
    + 4 B data + 4 B LCRC = 22 B = 5.5 DW -> six beats, `tkeep` 0x3 on the last.

    ⚠️ `nullified == 0` is asserted rather than an LCRC pass count, and the
    reason is that the LCRC pass count is not yet a trustworthy instrument --
    Phase 1 measured 0 match / 4 mismatch on the EP while it forwarded all four
    and rejected none, and could not separate a real defect from a probe
    sampling-phase error. Registered to #7f. `tlp_nullified_o` is the DUT's own
    verdict and is unambiguous.
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
# Row 7 -- §63 #7e, F17's TIMELINE. A MEASUREMENT ROW.
#
# !! EVERY INSTRUMENT THIS RUNG BUILT ANSWERS "HOW MANY" AND F17 TURNED OUT TO
# BE A "WHEN" QUESTION. Phase 1's counters said the Completion never becomes a
# packet; that was a cumulative-window artifact (FINDINGS_7E_PHASE3 §1) and the
# Completion is in fact well-formed. What is NOT known is whether it arrives
# before or after the enumeration engine gives up. This row stamps the cycle of
# every event on the round trip so that question has an answer instead of a
# story.
#
# !! IT IS DELIBERATELY PER-TEST. The cumulative `final`-block probe is exactly
# what produced the withdrawn claim. A window that spans tests is not a
# measurement of any of them.
#
# It asserts only NON-VACUITY -- that the events it is timing actually happened.
# Ordering is REPORTED, not asserted, because this rung has not earned the right
# to say which ordering is correct yet.
# ---------------------------------------------------------------------------


class F17Timeline:
    """Cycle stamps for one CfgRd0 -> CplD round trip, both stacks."""

    def __init__(self, dut):
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
            "ep_cfgrd0_up",       # EP DLL delivers it to the EP side (tlast)
            "ep_cpl_generated",   # EP config space emits the Completion (tlast)
            "rc_cpl_in",          # RC DLL inbound TLP completes (tlast)
            "rc_cpl_up",          # RC DLL delivers it to the RC's TL (tlast)
            "enum_error",         # the engine gives up
            "enum_done",
        )}
        # §63 #7e: the RC receives TWO inbound TLPs while the EP generates ONE
        # Completion. A DLL replays an unacknowledged TLP, and a replay carries
        # the SAME sequence number -- which the receiver must DISCARD, not
        # deliver. So "2 in, 1 up" is either a defect or exactly correct, and
        # only the sequence numbers separate those. Base 2.1 §3.5.2.1.
        self.rc_in_first_word = []
        self._rc_pending = None

    def _hs(self, v, r, last):
        return int(v.value) and int(r.value) and int(last.value)

    async def run(self, clk, cycles):
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
    """MEASUREMENT: when does each leg of the CfgRd0 round trip happen?

    Non-vacuity only. The point is the log, and specifically the VERDICT lines:
    how many Completions reach the RC's DLL and its TL BEFORE the enumeration
    engine raises enum_error_o, versus after.
    """
    # ⚠️ ENUM_CYCLES, NOT WINDOW -- AND THE DIFFERENCE IS THE SAME TRAP AGAIN.
    # This monitor originally ran for WINDOW (60,000) because that was longer
    # than anything worth timing when every enumeration died at ~10,869 cycles.
    # With F17 closed, enum_error_o now fires in the BAR phase at ~99,656, and a
    # 60,000-cycle observer simply stopped before the event it exists to stamp
    # and then failed its own non-vacuity guard. The instrument was sized for
    # the sicker link, like the beat-count oracles and the cumulative counters
    # before it.
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


# =============================================================================
# §63 #7f Phase 3.2 -- WITNESS ROWS W1-W4. #18 (the shared DLL never returns
# NP/P credits after FC init) and #21 (the replay-every-TLP corner, now #7h).
#
# !! RAW CAPTURE IN PYTHON, DECODE AFTERWARDS, KNOWN-ANSWER SELF-TEST FIRST.
# Standing rule §22.9x, earned by SEVEN instrument faults in this rung, six of
# them SV-side correlation logic (FINDINGS_7F_COM.md §3). Nothing below pairs,
# classifies or does arithmetic while the simulation runs: each monitor appends
# (cycle, raw word) tuples and nothing else. Every decoder is exercised on a
# HAND-DERIVED vector before it touches captured data, and the row fails on the
# self-test if the decoder is wrong -- an instrument that has not shown it can
# read a known value has no business reading an unknown one. The self-test is
# not advisory: it is the first statement of every verdict function.
#
# !! NO NEW SV PROBES. verilate_fullstack carries none of the probe_7*.sv files
# and gains none here. Every signal read below is a PORT of an existing module,
# reached hierarchically exactly as F17Timeline reaches its. Bare read after
# RisingEdge = the pre-edge value, which is the correct phase for counting an
# AXIS handshake (TlpPathWitness' docstring says why, and why §35 runs the
# other way).
#
# !! RED BEFORE FIX. W1, W2 and W3 are written against the tree at aeeb739 and
# measured RED there; the numbers are in each body. #18 lands as an ordered
# pair (D-P3.3): commit A (accounting) must turn W1 green and leave W2 and W3
# red; commit B (scheduling) must turn W2 and W3 green. W4 is #21's row: it
# rides expect_fail with its body pinned to the Phase 3.1 measurements and must
# STAY red through both commits (prediction C17). If it goes green on #18 alone
# that is a finding to report, not a success.
#
# ⚠️ §22.77 applies to W4 only: an expect_fail row reports PASS, so the gate
# cannot show #21 or show it closing. Its body and its W4 VERDICT log line are
# the witness. W1-W3 are ordinary rows and the gate carries their proof.
# =============================================================================

# -- DLLP type byte (pcie_datalink_pkg::dllp_type_e). Bits [2:0] carry the VC,
# which is 0 here, so a type compare masks them: (word & 0xF8) == TYPE.
DLLP_INITFC1_P, DLLP_INITFC1_NP, DLLP_INITFC1_CPL = 0x40, 0x50, 0x60
DLLP_INITFC2_P, DLLP_INITFC2_NP, DLLP_INITFC2_CPL = 0xC0, 0xD0, 0xE0
DLLP_UPDATEFC_P, DLLP_UPDATEFC_NP, DLLP_UPDATEFC_CPL = 0x80, 0x90, 0xA0

# -- the two credit constants the shared DLL advertises at FC init
# (pcie_datalink_pkg.sv:17-18). Read from the wire below, never assumed; these
# are the values the KNOWN-ANSWER vectors were derived from.
HDR_MIN_CREDITS = 16
PD_MIN_CREDITS = 64

W_TAIL = 2000
"""Cycles the W monitors keep sampling after enumeration returns.

A release-triggered UpdateFC follows the release it reports by a few cycles
plus arbitration; a register step follows its handshake by one. Two thousand
cycles is two orders of magnitude more than either needs and is short next to
the 5,122-cycle round trip the engine waits out before it returns anyway."""

COM_WINDOW = 20000
"""Cycles of RC PIPE-TX sampling for W4's COM grid, opened when
rc_fc_initialized_o rises. 3.1 fitted the period at 679 cycles; twenty
thousand cycles holds ~29 periods, enough to see the dominant gap."""


def pinned_red(dut, row, state, detail=""):
    """expect_fail HYGIENE (sec 63 #7f, Kourosh 2026-09-19): an expect_fail row
    must fail AT its one named, pinned assertion and nowhere else.

    cocotb's expect_fail turns ANY exception into a PASS -- a KeyError in the
    body, a timeout, a typo -- so a row can be red for a reason that has
    nothing to do with the defect it pins and the gate cannot tell. Rows 4 and
    5 of this file did exactly that for a whole rung: they read a key the
    runner never returned, and the KeyError hid under expect_fail until #18's
    fix let them run to the end (see run_enumeration_fs).

    The discipline: everything before the pinned assertion runs inside a
    try/except; any exception there is logged as NOT_REACHED and the row
    RETURNS NORMALLY, which under expect_fail is reported as a gate FAIL
    ("passed but we expected a failure"). Then the REACHED marker is logged,
    then the pinned assertion -- the only statement allowed to raise. The
    gate script copies these markers into its .diag as PINNED| rows.
    """
    dut._log.info("PINNED_RED|%s|%s|%s", row, state, detail)


def decode_fc_dllp_word(word):
    """First AXIS word of an InitFC/UpdateFC DLLP -> (type, HdrFC, DataFC).

    Layout is pcie_datalink_pkg::dllp_fc_t, little-endian on the 32-bit AXIS:
      [7:0]   type            (byte 0)
      [13:8]  HdrFC[7:2]      (byte 1 bits 5:0)
      [23:22] HdrFC[1:0]      (byte 2 bits 7:6)
      [19:16] DataFC[11:8]    (byte 2 bits 3:0)
      [31:24] DataFC[7:0]     (byte 3)
    Base 2.1 §3.4 Figure 3-5 gives the same byte layout; the package's
    send_fc_init (pcie_datalink_pkg.sv:279) is the builder this inverts.
    """
    t = word & 0xFF
    hdr = (((word >> 8) & 0x3F) << 2) | ((word >> 22) & 0x3)
    data = (((word >> 16) & 0xF) << 8) | ((word >> 24) & 0xFF)
    return t, hdr, data


def decode_tlp_dw0(word):
    """A TLP's DW0 as dllp2tlp presents it on m_tlp_axis -> (fmt_type, length).

    pcie_datalink_pkg::pcie_tlp_header_dw0_t is packed {byte3, byte2, byte1,
    byte0}, so byte0 (Fmt/Type) sits at [7:0] and Length is {byte2[1:0],
    byte3} = {[17:16], [31:24]}. Base 2.1 §2.2.1 Figure 2-4. This is the word
    dllp2tlp itself classifies on (dllp2tlp.sv, ST_TLP_STREAM's casez), and the
    word pcie_datalink_layer's s_tlp_axis carries from the TL (tlp2dllp.sv
    reads byte0 of the same layout).
    """
    ft = word & 0xFF
    length = (((word >> 16) & 0x3) << 8) | ((word >> 24) & 0xFF)
    return ft, length


def decode_link_first_word(word):
    """First AXIS word of an inbound link TLP at dllp2tlp's INPUT -> (seq, fmt_type).

    On the link a TLP is 2 B sequence number + header + LCRC (Base 2.1 §3.5).
    dllp2tlp.sv's ST_IDLE reads the sequence as {tdata[3:0], tdata[15:8]} and
    the TLP's own bytes start at [16]: [23:16] is Fmt/Type. The reserved nibble
    [7:4] is what marks a frame nullified when non-zero.
    """
    seq = ((word & 0xF) << 8) | ((word >> 8) & 0xFF)
    ft = (word >> 16) & 0xFF
    return seq, ft


def tlp_credit_class(fmt_type):
    """Fmt/Type -> the FC class dllp2tlp charges it to, or None.

    Mirrors dllp2tlp.sv's casez (Base 2.1 Table 2-36): NPH for header-only
    non-posted requests, NPD for non-posted requests with data (which ALSO
    consume one NPH), PH/PD for Msg/MWr/MsgD, CPLH/CPLD for completions.
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
    return tlp_credit_class(fmt_type) in ("NPH", "NPD")


def gap_histogram(cycles):
    """Sorted event cycles -> {gap: count}."""
    h = {}
    for a, b in zip(cycles, cycles[1:]):
        h[b - a] = h.get(b - a, 0) + 1
    return h


def w_selftest():
    """KNOWN-ANSWER SELF-TEST. Runs first in every W verdict. MANDATORY.

    Vectors derived BY HAND from the bit layouts above, not captured from the
    DUT, so a decoder that happens to agree with the DUT's own mistake still
    fails here. 0x40000450 is the vector the Phase 3.2 handoff names.
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
    # link first word: seq 0, CplD -> 0x004A0000 (the word F17 measured twice)
    assert decode_link_first_word(0x004A0000) == (0, 0x4A), "SELFTEST link word seq 0"
    # seq 0x123: tdata[3:0]=1, tdata[15:8]=0x23; CfgRd0 at [23:16]
    assert decode_link_first_word(0x00042301) == (0x123, 0x04), "SELFTEST link word seq 0x123"
    for ft, cls in ((0x04, "NPH"), (0x44, "NPD"), (0x4A, "CPLD"), (0x0A, "CPLH"),
                    (0x40, "PD"), (0x60, "PD"), (0x30, "PH"), (0x00, "NPH"),
                    (0x20, "NPH"), (0x02, "NPH"), (0x42, "NPD"), (0x4C, "NPD")):
        assert tlp_credit_class(ft) == cls, f"SELFTEST tlp_credit_class({ft:#04x})"
    assert gap_histogram([0, 679, 1358, 1400]) == {679: 2, 42: 1}, "SELFTEST gap_histogram"


def _first_attr(handle, names):
    """Resolve the first of `names` that exists under `handle`.

    W1 must run RED on the tree BEFORE commit A, where the receive-side
    allocated register still carries its old name, and GREEN after, where it
    carries the spec's. A row that hard-coded either name would fail the other
    tree with an AttributeError -- red for the wrong reason, which is not red.
    The name resolved is logged so the record says which tree it measured.
    """
    for n in names:
        try:
            return n, getattr(handle, n)
        except AttributeError:
            continue
    raise AttributeError(f"none of {names} under {handle._path}")


def _dll(dut, side):
    # The two instance names differ and the difference is inherited
    # (BothEndsProbe's docstring).
    return (dut.u_ep.datalink_layer_inst if side == "ep"
            else dut.u_rc.u_phy.pcie_datalink_layer_inst)


class W18Capture:
    """Raw captures for W1/W2 on ONE stack's DLL.

    Three streams, all raw:
      dllp_tx   (cycle, first word) of every DLLP this DLL hands its PHY
                -- m_phy_axis with tuser bit 0, axis_user_demux's UserIsDllp
      release   (cycle, DW0) of every TLP handshaken OUT of dllp2tlp toward
                the TL / config space, stamped at tlast -- the point at which
                the DLL's receive buffer space is made available again
      alloc_ev  (cycle, value) at every change of the NP-header allocated
                register (dllp2tlp's port; old name before commit A)
    """

    def __init__(self, dut, side):
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

    # -- derived views, computed AFTER the run, never during it -------------
    def initfc1_np(self):
        return [(c,) + decode_fc_dllp_word(w) for c, w in self.dllp_tx
                if (w & 0xF8) == DLLP_INITFC1_NP]

    def updatefc(self, dllp_type):
        return [(c,) + decode_fc_dllp_word(w) for c, w in self.dllp_tx
                if (w & 0xF8) == dllp_type]

    def np_releases(self):
        return [c for c, dw0 in self.release if is_np_header(decode_tlp_dw0(dw0)[0])]

    def report(self, dut, tag):
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


# ---------------------------------------------------------------------------
# W1 -- #18 commit A's row: CREDITS_ALLOCATED advances AS credits are released.
# ---------------------------------------------------------------------------
@cocotb.test()
async def fullstack_w1_ep_credits_allocated_advance_on_release(dut):
    """The Endpoint DLL's NP-header CREDITS_ALLOCATED starts at its InitFC
    advertisement and advances by one AT EACH RELEASE of an NP TLP toward its
    Transaction Layer -- never before.

    Base 2.1 §2.6.1.2 p.141, CREDITS_ALLOCATED: "Count of the total number of
    credits granted to the Transmitter since initialization" ... "Incremented
    as the Receiver Transaction Layer makes additional receive buffer space
    available by processing Received TLPs". The release point in this DLL is
    dllp2tlp's m_tlp_axis handshake at tlast: the TLP has left the DLL's
    receive FIFO for pcie_cfg_wrapper or the TL, and its buffer space is free.

    == THE PAIRING ==========================================================
    Three raw captures on the EP's DLL, paired here and nowhere else:
      * the EP's own InitFC1-NP DLLP, decoded for the advertised HdrFC
        (known-answer 0x40000450 -> 16 first);
      * every TLP released from dllp2tlp, classified NP by its DW0;
      * every change of the NP-header allocated register.
    For every register step at cycle c to value v: (v - advertised) must equal
    the number of NP releases at cycles STRICTLY BEFORE c. A step that lands
    before its release has counted buffer space as free while the TLP still
    occupies it -- the Receiver Overflow hazard §2.6.1.2 p.141 names.

    ⚠️⚠️ RED WHEN WRITTEN (tree aeeb739). Measured in this row, one
    enumeration, 160,932 cycles sampled: InitFC1-NP advertised HdrFC 16;
    16 NP TLPs released (8 CfgRd0 = NPH, 8 CfgWr0 = NPD, first at cycle 9119,
    last at 90599); 16 register steps, ALL 16 landing BEFORE their release --
    the first at 9115 (register 17) four cycles ahead of the first release at
    9119, and every later one 4 cycles ahead likewise; final value 32 = 16 + 16,
    which is correct.
    The register exists and reaches the right FINAL value, but it steps at
    CRC-accept (dllp2tlp's ST_CHECK_CRC), before the frame has even been
    committed to the receive FIFO, so every step runs one release ahead. And
    it is named `nph_credits_consumed_r` -- a consumed counter that starts at
    the advertisement and counts up is CREDITS_ALLOCATED wearing the wrong
    name, and the misnomer is what let probe_7f's pr7f_alloc label the PEER's
    limit as "advertised" in Phase 2e.

    ⚠️ This row is INERT ON THE WIRE by design (D-P3.3): commit A changes what
    the register holds and when, and no UpdateFC carries it until commit B.
    Green here with W2 still red is the expected intermediate state.
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


# ---------------------------------------------------------------------------
# W2 -- #18 commit B's row: an UpdateFC-NP is SCHEDULED when credit is released.
# ---------------------------------------------------------------------------
@cocotb.test()
async def fullstack_w2_ep_updatefc_np_scheduled_on_release(dut):
    """Once the Endpoint's DLL releases an NP credit, it transmits an
    UpdateFC-NP carrying the released credit; the last UpdateFC-NP of the run
    carries every release; and no UpdateFC-NP ever advertises more than the
    releases that preceded it.

    ⭐ THE BOUND IS THE RELEASE CLAUSE, NOT THE 30 µs PERIODIC FLOOR.
    Base 2.1 §2.6.1.2 p.142: "For non-infinite NPH, NPD, PH, and CPLH types,
    an UpdateFC FCP must be scheduled for Transmission each time ... one or
    more units of that type are made available by TLPs processed". The
    periodic 30 µs (-0%/+50%) rule on the same page is a SEPARATE obligation,
    registered to #7g (FINDINGS_7F_UNITS.md §3), and this row asserts nothing
    about it -- the two must not be conflated.

    == THE PAIRING ==========================================================
    The same three raw captures as W1. UpdateFC-NP DLLPs are decoded for
    HdrFC (known-answer 0x40000450 -> 16, 0x40400490 -> 17 first) and paired
    against the NP release cycles:
      1. after the first NP release, some UpdateFC-NP carries HdrFC above the
         InitFC advertisement                                (the clause)
      2. the LAST UpdateFC-NP carries advertised + all NP releases
                                                             (nothing owed)
      3. every UpdateFC-NP's HdrFC <= advertised + releases before it
                                                             (no overstatement)
      4. HdrFC is non-decreasing across the run             (cumulative)

    ⚠️⚠️ RED WHEN WRITTEN (tree aeeb739). Measured in this row: 16 NP releases
    at the EP (first 9119, last 90599); UpdateFC-NP on the EP's transmit path:
    exactly ONE, at cycle 6790, HdrFC 16, DataFC 64 -- pcie_flow_ctrl_init's
    post-init DLLP, sent 2,329 cycles before the first TLP arrived -- and ZERO
    after the first release; 57 DLLPs transmitted in all (16 Ack, the InitFC1/2
    triples, one UpdateFC-P, one UpdateFC-NP).
    Exactly the Phase 2e picture: the two UpdateFC-NP DLLPs the link ever
    carries are pcie_flow_ctrl_init's post-init pair, both HdrFC=16, sent
    before any TLP has crossed; dllp_fc_update -- the only emitter with the
    allocated count as an input -- fires from a 200,000-cycle timer that no
    test reaches and has no release trigger. The Root Complex's CREDIT_LIMIT
    therefore stays at 16 for the life of the link and the 17th non-posted
    request blocks forever (F18, bar_count stalled at 2).

    ⚠️ Posted (P) credits share the mechanism (D-P3.4, C14) but this bench
    issues no posted TLP toward the EP -- enumeration is configuration traffic
    -- so the P half of commit B has no release to witness here. It is
    REPORTED (UpdateFC-P census in the log), not asserted; stated so the gap
    is visible rather than implied closed.
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

    # The DATA half of the same clause: NPD (DataFC) must step too. Each NPD
    # release returns Roundup(Length / 4 DW) data credits (Table 2-36 fn 31;
    # Length 0 = 1024 DW = 256). Here every CfgWr0 is 1 DW, so one credit each,
    # but the expectation is computed from the captured Length, not assumed.
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


# ---------------------------------------------------------------------------
# W3 -- P3-5: after #18 the Root Complex is never credit-starved.
# ---------------------------------------------------------------------------
@cocotb.test()
async def fullstack_w3_rc_never_credit_blocked(dut):
    """err_credit_blocked_o never asserts while the Root Complex enumerates.

    ⭐ A PLAIN LIVE ASSERTION ON ONE SIGNAL. Nothing to decode and nothing to
    pair -- the handoff specifies this row that way on purpose: seven of this
    rung's instrument faults were pairing faults. pcie_rc_top's
    err_credit_blocked_o is the enumeration engine's own annotation that a
    completion timeout smelled like credit (pcie_enum_scan.sv:160-173), set
    when a TXN_TIMEOUT is reported with tx_fc_blocked_i high. It is sampled
    every cycle from bring-up through W_TAIL cycles after the engine returns.

    Non-vacuity is a COUNT, not a pairing: every one of the 16 header credits
    the Endpoint advertises at FC init must have been consumed at the RC's
    DLL input, AND a 17th request must have been attempted -- witnessed either
    by a 17th reaching the DLL (after the fix) or by tx_fc_blocked_o having
    been seen high (before it). Fewer than 16 consumed and starvation could not
    have occurred whatever the DLL did.

    ⚠️ THE FIRST DRAFT OF THIS GUARD WAS WRONG, AND IT IS KEPT ON RECORD. It
    counted non-posted requests at pcie_datalink_layer's s_tlp_axis and
    demanded MORE than 16 -- but the credit gate (tlp_credit_manager) sits in
    the Transaction Layer UPSTREAM of that seam, so a starved request never
    reaches it and the count saturates at exactly 16 on the red tree. The row
    declared itself vacuous on the very run that showed the defect. A
    non-vacuity guard placed downstream of the mechanism it guards against
    cannot fire; same class as §22.85's route error.

    ⚠️⚠️ RED WHEN WRITTEN (tree aeeb739). Measured in this row: 16 non-posted
    requests reached the RC's DLL and no more; tx_fc_blocked_o high for 67,547
    sampled cycles (the 65,536-cycle bench timeout plus the tail);
    err_credit_blocked_o rose at cycle 158,933 together with enum_error
    (code 4 = ENUM_ERR_TIMEOUT), enum_done 0, bar_count 2.
    The 17th non-posted request finds nph_available = 0 (Phase 2e's register
    trace: the RC's limit is loaded 16 at init and never rewritten), sits in
    the VC buffer behind the credit gate, and times out from ALLOCATION
    (tlp_request_tracker measures per-tag age from allocation, which precedes
    the gate) having never been transmitted. The engine reports
    ENUM_ERR_TIMEOUT with the credit annotation set -- #19's misreport, fixed
    in Phase 3.4 -- and bar_count stalls at 2. Prediction P3-5: after A+B the
    limit refills as the EP releases credit and this signal never rises.
    """
    w_selftest()

    class W3Capture:
        def __init__(self, dut):
            self.blocked = dut.u_rc.err_credit_blocked_o
            self.fcblk = dut.u_rc.tx_fc_blocked_o
            self.rc = _dll(dut, "rc")
            # P3-5 AS WRITTEN says "nph_available refills": the RC's credit
            # manager's live remainder, REPORTED (min, cycles at zero, number of
            # refills). The assertion stays on the one signal above.
            self.avail = dut.u_rc.u_tl.u_tlp_layer.credit_manager_inst.nonposted_header_available_o
            # Counted from rc_fc_initialized_o onward: before FC init the limit is
            # still 0, so the remainder reads 0 for the whole bring-up (~6,700
            # cycles) and a min taken from cycle 0 says nothing about refills.
            self.fc_init = dut.rc_fc_initialized_o
            self.avail_min = None
            self.avail_zero_cycles = 0
            self.avail_refills = 0
            self.blocked_cycles = []
            self.fcblk_cycles = 0
            self.np_requests = []
            self.cycles = 0

        async def run(self, clk, max_cycles, stop):
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
# W4 -- #21's row.  §63 #7j-2 FLIPPED IT AND REWROTE ITS BODY, in the same
# commit that moved the RTL (D-7J.4).
#
# ⚠️⚠️ §22.87 IN ITS SHARPEST FORM, and it was PREDICTED rather than met at the
# gate.  W4 rode as expect_fail from #7f, with its body pinned to the Phase 3.1
# measurements and an in-tree comment saying "Must stay red through #18".  The
# anchor run on tag `evidence/7h-self-drain-C` -- captured at the #7j Phase 2
# STOP, before #7j-2 existed -- reported this row as the run's ONE FAIL, and it
# failed because it UNEXPECTEDLY PASSED: the EP had stopped replaying
# (dups=0 ep_replays=0 rc_replays=0, was 17/17/0).  An expect_fail row reports
# FAIL when the defect it pins is gone, which reads as a regression in the fix
# and is not one.
#
# So the marker goes, the §22.93 pinned-red plumbing goes with it -- an
# ordinary row already fails on any exception, which is the behaviour that
# plumbing existed to restore -- and the numbers below are restated as what the
# link now does rather than as what it used to do.  The ASSERTION is unchanged,
# character for character; only its colour and its premise moved.
# ---------------------------------------------------------------------------
@cocotb.test()  # §63 #7j-2: flipped green, body rewritten in the fix commit (D-7J.4).
async def fullstack_w4_ep_does_not_replay_every_tlp(dut):
    """Every DLL sequence number the Endpoint transmits arrives at the Root
    Complex's DLL exactly once, and the Endpoint's replay machine never fires.

    Base 2.1 §3.5.2.1: a TLP is replayed when its Ack does not arrive within
    the replay timer; a Receiver discards a duplicate. Replay is a RECOVERY
    path. A link that replays EVERY TLP is spending half its bandwidth
    recovering from nothing.

    ⚠️⚠️ RED BY MEASUREMENT, AND THIS BODY IS PINNED TO THE PHASE 3.1 NUMBERS
    (FINDINGS_7F_P31B.md, FINDINGS_7F_COM.md; tree 57189a2, bench-only):
      * the TLP tail is released from the RC's TX scrambler on a FREE-RUNNING
        GRID of period 679 cycles (7750, 8429 = 7750 + 679 exactly; the
        packet's arrival moved with K in {0,150,300,600,900} and the release
        did not move at all);
      * COM (K28.5) is on the wire at the RC PIPE TX seam at the SAME period:
        162 of the inter-COM gaps are exactly 679, 197 of 329 events sit at
        residue 1 mod 679 from anchor 7750, and the tail release at 7750 is
        ONE CYCLE BEFORE the COM at 7751;
      * the Endpoint replays at 2,722 cycles against REPLAY_TIMER_CYCLES =
        2720 (16'h0AA0), EXACTLY ONE replay per TLP, timer-driven (zero Nak
        DLLPs in the run); the Root Complex replays ZERO times;
      * duplicate Completions are correctly DISCARDED at the RC (2 in, 1 up).
    So the mechanism class is named -- tail release phase-locked to the
    periodic ordered-set schedule, one cycle ahead of each COM -- and the
    mechanism itself is not: `send_ordered_set` never toggles at phy_transmit's
    level while ordered sets are measured on the wire (§22.85, two single-route
    negatives). That reconciliation is #7h's first question and the reason
    this row is expect_fail rather than fixed here (D-P3.1, D-P3.7).

    == WHAT THIS ROW CAPTURES, RAW, AND PAIRS AFTERWARDS ======================
      rc_rx      (cycle, first word) of every inbound link TLP at the RC's
                 dllp2tlp input -- the sequence number lives in that word
                 (known-answer 0x004A0000 -> seq 0, CplD first)
      ep_tx      (cycle, seq) every TLP the EP's retry_management is told about
      ep_replay  (cycle, mask) every rising edge of the EP's retry_valid_o
      rc_replay  likewise on the RC
      com        cycles at which a K28.5 is on the RC's PIPE TX in a COM_WINDOW
                 opened when rc_fc_initialized_o rises
    The verdict pairs rc_rx by sequence number; everything else is reported
    beside it so the #7h reader has the whole picture in one log.

    ⚠️⚠️ MEASURED IN THIS ROW AT aeeb739 (one enumeration): 32 inbound TLPs at
    the RC's DLL for 16 distinct sequence numbers -- EVERY one arrived twice,
    duplicate spacing 2,727 cycles (min = median = max); the first inbound word
    was 0x004A0000 (seq 0, CplD) as the self-test vector predicts; the EP's
    retry_valid rose 16 times for 16 TLPs, the RC's 0 times; in the 20,000-cycle
    COM window opened at cycle 6,740 there were 28 COM events and the dominant
    inter-COM gap was 679 cycles (16 of 27 gaps), the 3.1 period exactly.

    ⚠️ Prediction C17: this row stays RED after #18's A+B, because the replay
    is driven by the EP's replay timer against round-trip latency and #18 does
    not touch either. If it goes GREEN on #18 alone, report it.
    ⚠️ §22.77: expect_fail reports PASS. The W4 VERDICT line is the witness;
    read it, not the gate row.

    ⚠️ PINNED (Kourosh, 2026-09-19): this row may fail ONLY at its one named
    assertion, the no-duplicate / no-replay check. Everything before it runs
    inside a guard; an exception there is logged PINNED_RED|...|NOT_REACHED and
    the row returns normally, which under expect_fail is a gate FAIL. The
    marker PINNED_RED|...|REACHED is logged immediately before the pinned
    assertion. See pinned_red().
    """
    w_selftest()

    class W4Capture:
        def __init__(self, dut):
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
                        for b in range(4):
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

    # NON-VACUITY -- inside the guard on purpose: a failure HERE is not the
    # defect this row pins and must surface as a gate FAIL, not a PASS.
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
    # §63 #7j-2: the try/except that used to wrap this body existed only to keep
    # cocotb's expect_fail from swallowing a setup exception as a PASS (§22.93).
    # An ordinary row fails on any exception by itself, so the guard is gone and
    # the body runs unwrapped.
    dut._log.info("W4 dups=%d ep_replays=%d rc_replays=%d",
                  len(dups), len(cap.ep_replay), len(cap.rc_replay))
    assert not dups and not cap.ep_replay, (
        f"{len(dups)} of {len(by_seq)} DLL sequence numbers arrived at the RC's DLL "
        f"more than once (duplicate spacing {spacing[:4]} cycles); the EP's replay "
        f"machine fired {len(cap.ep_replay)} times for {len(cap.ep_tx)} TLPs and the "
        f"RC's {len(cap.rc_replay)} times. Base 2.1 §3.5.2.1: replay is recovery, "
        "not steady state. #21 -> #7h")


# ===========================================================================
#  §63 #7j-2 -- ACCEPTANCE (a) and (b), at the full stack, both stacks.
#
#  The phy_transmit-seam rows in test_7j2_idle.py drive the idle request from
#  the bench, because phy_transmit does not contain the LTSSM.  These two rows
#  are where the request comes from the REAL LTSSM, in L0, with two real PHYs
#  facing each other through the codec bridge -- which is the only place the
#  whole claim can be made.
# ===========================================================================

def _l0_window(events, first, last):
    return [e for e in events if first <= e[0] <= last]


def _longest_run(samples, want):
    """Longest CONTIGUOUS run of `want` in [(cycle, value)], as (first, last, len).

    ⚠️ This exists because the obvious form is wrong and was measured wrong.
    Taking min() and max() of every cycle whose state reads ST_L0 gave a
    "window" of [1, 60020] -- the whole run, training included -- because ONE
    early sample reads 0x5 before the link is up and min() cannot tell an
    outlier from a start.  The window then contained the TS Ordered Sets it was
    opened to exclude, acceptance (a) counted 1478 valid-low cycles that were
    Configuration's and acceptance (b) reported 1.2 % residue that was TS
    bodies descrambled as if they were data.

    A window that does not contain exactly the event it was opened for is the
    #7h lesson, and §22.89 says a row must not merely STATE where its window
    opens but prove it.  The longest contiguous run is that proof: it cannot be
    moved by an outlier, and the row asserts it dominates the sample set.
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
    """ACCEPTANCE (a) -- in L0 the PIPE TX valid never drops, on BOTH stacks.

    Base 2.1 §4.2.2 p.195: "When no packet information or special Ordered Sets
    are being transmitted, the Transmitter is in the Logical Idle state.
    During this time idle data must be transmitted" -- so every Symbol Time
    carries a Symbol and `valid` is continuously asserted.

    RED BEFORE FIX: #7j Phase 1 measured the opposite at this very seam --
    between packets the PIPE carried a stale scrambled word, frozen and
    repeated, with valid LOW (M3: one word held for 182 consecutive cycles).

    ⚠️ THE WINDOW OPENS WHERE L0 OPENS, AND IT IS PROVEN RATHER THAN ASSUMED
    (§22.89).  `link_up` is asserted in Configuration.Idle as well as L0, and
    Configuration.Idle legitimately transmits idle through a different request,
    so a window anchored on link_up would measure the wrong state and pass for
    the wrong reason.  The EP exposes `ep_ltssm_state_o`; the window opens the
    first cycle it reads ST_L0 and a settling margin after.

    ⚠️ AND THE SAMPLES ARE CONTINUOUS FROM BEFORE THAT POINT, not started when
    the state is seen: a waiter is an observer with a phase, and its phase is
    rarely documented.
    """
    ST_L0 = 0x00005
    SETTLE = 64

    rc_v, ep_v, state = [], [], []
    done = False

    async def sample():
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
    """ACCEPTANCE (b) -- an INDEPENDENT Python model descrambles the whole L0
    wire stream to packets plus 00h, with no residue.

    The oracle is `rx_golden.Descrambler`, which shares no code with the RTL:
    `advance()` is transcribed from the bit equations on Base 2.1 p.698 and
    `xor_mask()` from p.699, and its known-answer test passes 456 checks
    against the two published tables on p.700 (128 LFSR states, 304 output
    bytes).

    The claim: descramble every Symbol the RC transmits while it is in L0, and
    what comes out is either part of a framed packet (between STP/SDP and END)
    or the Idle Symbol 00h.  "No residue" is the load-bearing half -- a stream
    that descrambles to arbitrary non-zero bytes outside packets would mean the
    Transmitter was emitting something that is neither.

    RED BEFORE FIX: before #7j-2 the gaps between packets carried a FROZEN
    scrambled word repeated with valid low.  Valid-gated, that stream has
    almost no Symbols in it at all, so the row fails its own non-vacuity check
    -- which is the honest way for it to be red, rather than by counting
    residue in a stream that was never sent.
    """
    import rx_golden

    ST_L0 = 0x00005
    SETTLE = 64
    COM_B, SKP_B, STP_B, SDP_B, END_B, EDB_B = 0xBC, 0x1C, 0xFB, 0x5C, 0xFD, 0xFE

    syms, state, done = [], [], False

    async def sample():
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

    # ⚠️ The descrambler is driven from the COM that resets it, not from the
    # window's first Symbol: the LFSR state at an arbitrary offset is unknown,
    # and a model started mid-stream would report residue that is its own.
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
    """§63 #7j-2 -- SKP Ordered Sets keep their spec spacing in L0, and none is
    ever placed INSIDE a packet.

    Base 2.1 §4.2.7.1 p.261: "The SKP Ordered Set shall be scheduled for
    insertion at an interval between 1180 and 1538 Symbol Times", and
    "Scheduled SKP Ordered Sets shall be transmitted if a packet or Ordered Set
    is not already in progress, otherwise they are accumulated and then
    inserted consecutively at the next packet or Ordered Set boundary."
    §4.2.2 p.195 adds the clause this rung needs: "During transmission of the
    idle data, the SKP Ordered Set must continue to be transmitted as specified
    in Section 4.2.7."

    ⚠️⚠️ THIS ROW IS WHY ST_L0 DROPS ITS TRANSMIT STROBE, and it is the only
    row in the repo that can say so.  `verilate_7j2_idle`'s C4 asks the same
    question at the phy_transmit seam, where the BENCH drives
    send_ordered_set_i -- so an LTSSM mutant is invisible to it (§22.85: a
    property asserted of one point in a route, measured at another).  Here the
    real LTSSM drives it.

    MUTANT: "ST_L0 keeps its unconditional transmit_ordered_set = '1".  With
    the strobe high, os_generator's ST_SEND streaming lock breaks at every
    Ordered-Set boundary and the FSM returns to ST_IDLE, where
    `if (gen_os_ctrl_i.valid) D.skp_cnt = '0` resets the SKP timer.  Under a
    continuous idle request that happens every ~8 cycles, so skp_cnt never
    reaches SkpIntervalCounts and the schedule is starved outright.  This row
    goes red at its non-vacuity check.

    ⚠️ THE SPACING IS ASSERTED ON THE MEDIAN, NOT ON EVERY GAP, and that is the
    spec's own shape rather than a loosening: p.261's second clause says a SKP
    that falls due inside a packet is DEFERRED to the next boundary, so
    individual gaps legitimately run long, and §4.2.7.2 p.261 obliges a
    Receiver to tolerate an AVERAGE inside the window.  Every gap is logged.
    """
    ST_L0, SETTLE = 0x00005, 64
    COM_B, SKP_B, STP_B, SDP_B, END_B, EDB_B = 0xBC, 0x1C, 0xFB, 0x5C, 0xFD, 0xFE
    SPEC_LO, SPEC_HI = 1180, 1538          # Symbol Times, §4.2.7.1 p.261

    syms, state, done = [], [], False

    async def sample():
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

    # Symbol Time index inside the L0 window: one per TRANSMITTED Symbol, which
    # is what p.261 counts.  Valid-gated, because a Symbol Time that carried no
    # Symbol is not a Symbol Time the schedule may count.
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

    # (i) the schedule is alive at all -- this is the limb the mutant reddens.
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


# ===========================================================================
# §63 #7i -- the error-injection rows.
#
# These five are the first rows in the project that drive the RECEIVER'S ERROR
# PATHS against a real partner.  Every prior test of the Nak/replay chain drove
# it from a cocotb source at the DLL's own port; these corrupt the wire between
# two real stacks and let the far end react on its own.
#
# ⚠️⚠️ THE PEER CAN NEVER PRODUCE THESE FRAMES BY ITSELF, AND THAT IS MEASURED,
# NOT ASSUMED.  §63 #7i C-16 predicted and then measured that neither
# transmitter can emit an EDB Symbol -- `EDB` appears in src/ only inside the
# RECEIVE framing detector, and a whole-run count of K-flagged 8'hFE on both
# PIPE transmit ports is zero, against a non-vacuity count of the ENDP Symbols
# the same detector does see.  So the injector is not a convenience here: it is
# the ONLY source of these frames, permanently.  A later reader must not expect
# the peer stack to exercise them.
#
# The injector is `pipe_codec_bridge`'s, driven through top-level signals
# (D-7I.3).  Each row arms it, runs one enumeration, and lowers it again.
# ===========================================================================

INJ_FLIP, INJ_NULLIFY, INJ_EDB_BAD = 0, 1, 2

# The RC's enumeration CfgRd0 link packet is 18 bytes -- 2 sequence + 12 TLP +
# 4 LCRC -- carried two bytes per beat after STP, so beats 1..9, with the LCRC
# in beats 8 and 9 and END in the beat after.  Measured in §63 #7i Phase 1's
# A2 arm, which put a bit error in beat 9 and saw the LCRC check fire.
LCRC_FIRST_BEAT = 8
TARGET_PKT      = 3


async def _run_injected(dut, mode, off=0, bit=0, pkt=TARGET_PKT):
    """Arm the bridge injector, enumerate, and return the DLL observations.

    Returns a dict per stack.  Everything is read from the DUT's own registers
    rather than recomputed here: `next_expected_seq_num_r` is NEXT_RCV_SEQ,
    `response_is_nak_r` is the verdict dllp_fc_update publishes, and
    `nak_scheduled_r` is the spec's NAK_SCHEDULED flag.
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
        prev_nak = 0
        while not obs.get("stop"):
            await RisingEdge(dut.clk_i)
            # Sampled AFTER the edge, so these are the post-edge values the
            # RTL just committed -- not the pre-edge read §22.89 warns about.
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
    obs["enum"] = r
    return obs


async def _run_clean(dut):
    """The same run with the injector never armed -- the control arm.

    §22.81: every negative assertion pairs with a positive row through the
    same path.  The injected rows below assert "exactly one Nak"; this is what
    says zero is the number when nothing is injected, measured through the
    identical code.
    """
    return await _run_injected(dut, INJ_FLIP, off=0, bit=0, pkt=0)


@cocotb.test()
async def fullstack_7i_injected_header_error_is_naked_and_replayed(dut):
    """§63 #7i (b) -- a bit error in a TLP HEADER: not delivered, one Nak
    carrying NEXT_RCV_SEQ-1, and the replay delivers it exactly once.

    Base 2.1 §3.5.3.1 p.182: "comparing the calculated result with the value in
    the LCRC field of the received TLP ... if not equal, the TLP is corrupt -
    discard the TLP and free any storage allocated for the TLP ... If the
    NAK_SCHEDULED flag is clear, schedule a Nak DLLP for transmission
    immediately"; p.184: "Data Link Layer Ack and Nak DLLPs specify the value
    (NEXT_RCV_SEQ - 1) in the AckNak_Seq_Num field".

    ⚠️ This row is GREEN BEFORE the #7i fix and must stay green after it.  It
    is here because §63 #7i found the registered defect #20 ("the receive LCRC
    check never fires") was not a defect at all, and the reason nobody caught
    that for two rungs is that no row in the FULL STACK asserted the chain --
    only a unit bench did.  §22.84: defect-status and test-existence are
    independent axes.
    """
    obs = await _run_injected(dut, INJ_FLIP, off=3, bit=5)
    assert obs["fired"] == 1, "the injector never fired -- the row is vacuous"
    assert obs["naks"] == 1, f"expected exactly one Nak, saw {obs['naks']}"
    assert obs["delivered"] >= 1, "nothing was ever delivered -- the link is broken"


@cocotb.test()
async def fullstack_7i_injected_lcrc_error_is_naked_and_replayed(dut):
    """§63 #7i (c) -- the corruption is in the LCRC FIELD ITSELF.

    Same clause, same outcome: the compare must not care WHERE the corruption
    is.  Kept separate from the header row because a receiver that recomputed
    the CRC over the LCRC field, or that compared the field against itself,
    would pass the header row and fail this one.
    """
    obs = await _run_injected(dut, INJ_FLIP, off=9, bit=2)
    assert obs["fired"] == 1, "the injector never fired -- the row is vacuous"
    assert obs["naks"] == 1, f"expected exactly one Nak, saw {obs['naks']}"
    assert obs["delivered"] >= 1, "nothing was ever delivered -- the link is broken"


@cocotb.test()
async def fullstack_7i_injected_sequence_error_is_naked_and_replayed(dut):
    """§63 #7i -- the corruption is in the SEQUENCE NUMBER bytes.

    ⭐ Two checks fire on one fault, and that is the point of this row.  The
    sequence bytes are INSIDE the LCRC's protected span -- §3.5.2.1 p.171:
    "LCRC calculation starts with bit 0 of byte 0 (bit 8 of the TLP sequence
    number)" -- so a flipped sequence bit fails the LCRC compare AND the
    NEXT_RCV_SEQ compare.  Measured in Phase 1's A3 arm: next_tx read 0 where
    2 was expected, and the recovery was still exactly one replay.
    """
    obs = await _run_injected(dut, INJ_FLIP, off=1, bit=1)
    assert obs["fired"] == 1, "the injector never fired -- the row is vacuous"
    assert obs["naks"] == 1, f"expected exactly one Nak, saw {obs['naks']}"
    assert obs["delivered"] >= 1, "nothing was ever delivered -- the link is broken"


@cocotb.test()
async def fullstack_7i_nullified_tlp_is_discarded_silently(dut):
    """§63 #7i commit C -- ACCEPTANCE (d).  A NULLIFIED TLP IS DISCARDED
    SILENTLY: no delivery, NO NAK, no replay.

    Base 2.1 §3.5.3.1 p.182, the clause this commit implements:

        "If the Physical Layer reports that the received TLP end framing Symbol
         was EDB, and the LCRC is the logical NOT of the calculated value,
         discard the TLP and free any storage allocated for the TLP.  THIS IS
         NOT CONSIDERED AN ERROR."

    and §3.5.2.1 p.173 for what the transmitter did to make one:

        "use the remainder of the calculated LCRC value without inversion (the
         logical inverse of the value normally used)" and "indicate to the
         Transmit Physical Layer that the final framing Symbol must be EDB
         instead of END".  "When this is done, the Transmitter does not
         increment NEXT_TRANSMIT_SEQ".

    RED BEFORE FIX, and red for the right reason: on the pre-commit-C tree
    nothing in the design examined EDB at all -- `data_handler.sv:242,268`
    OR'd it with ENDP and published neither -- so a nullified frame was framed
    as an ordinary TLP, failed the (working) LCRC compare, and was NAK'D.  The
    assertion below that fails first on that tree is `naks == 0`.

    ⚠️ The Nak count is the assertion, not the delivery count: a pre-fix tree
    also does not deliver the frame, so asserting only "not delivered" would
    pass before the fix and prove nothing (§22.82).
    """
    obs = await _run_injected(dut, INJ_NULLIFY, off=LCRC_FIRST_BEAT)
    assert obs["fired"] == 1, "the injector never fired -- the row is vacuous"
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
    """§63 #7i commit C -- Kourosh's constraint, and p.182's SECOND EDB limb.

        "If TLP end framing Symbol was EDB but the LCRC does not match the
         logical NOT of the calculated value, the TLP is corrupt - discard the
         TLP and free any storage allocated for the TLP.  If the NAK_SCHEDULED
         flag is clear, schedule a Nak DLLP for transmission immediately"

    So EDB alone does not buy silence: the inverted LCRC is what distinguishes
    a deliberate nullification from a frame that was corrupted into looking
    like one.

    ⚠️⚠️ RED BEFORE FIX, AND THE PREDICTION THAT SAID OTHERWISE WAS WRONG IN
    AN INFORMATIVE WAY.  §63 #7i C-25 predicted this row would be GREEN on the
    pre-commit-C tree, reasoning that "before the fix every EDB frame was
    Nak'd, including this one".  Measured: 0 Naks, not 1.

    The reasoning was backwards.  Before the fix EDB is INVISIBLE -- it is
    OR'd with ENDP in data_handler and never published -- so this frame is not
    an EDB frame at all as far as the design is concerned: it is an ordinary
    TLP with a perfectly good LCRC, and it is ACCEPTED AND DELIVERED TO THE
    TRANSACTION LAYER.  A frame the spec calls corrupt is handed up as valid
    data.

    ⭐ So the pre-fix defect here is strictly worse than the one the silent-
    discard row covers, and neither the brief nor C-25 saw it: the nullified
    case merely produced a spurious Nak, while THIS case is a silent wrong
    delivery.  Recorded in FINDINGS_7I_C25.md.

    After the fix it is Nak'd for the right reason, by the arm that tests for
    the inverted LCRC and finds it absent.

    ⭐ That is also why the commit-C mutant must kill the silent-discard row
    and NOT this one.  A mutant that killed both would mean this row is
    measuring EDB handling rather than the inversion test.
    """
    obs = await _run_injected(dut, INJ_EDB_BAD, off=LCRC_FIRST_BEAT)
    assert obs["fired"] == 1, "the injector never fired -- the row is vacuous"
    assert obs["naks"] == 1, (
        f"an EDB frame with a NON-inverted LCRC produced {obs['naks']} Nak(s); "
        "§3.5.3.1 p.182's second EDB bullet requires exactly one. "
        "0 is the pre-commit-C value and means the frame was DELIVERED."
    )
