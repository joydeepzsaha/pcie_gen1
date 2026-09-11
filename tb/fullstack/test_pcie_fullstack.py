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

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ClockCycles

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
# Row 1b -- RED. FC init does not complete, and the body pins WHY.
# ---------------------------------------------------------------------------
@cocotb.test(expect_fail=True)
async def fullstack_completes_fc_init_both_ways(dut):
    """FC init completes both ways. ⚠️ RED ON CURRENT RTL -- measured, not feared.

    !! READ THIS BODY BEFORE FLIPPING THE ROW (SS22.87). It encodes WHY it is
    red, and those premises expire when the defect is fixed.

    ⭐⭐ THE DATA ARRIVES INTACT AND THE FRAMING IS LOST. Measured 2026-09-11,
    evidence/fullstack/FINDINGS_F16.md:

        the RC transmits   payload 0x40000440, then CRC 0x8ef8 in a tlast beat
                           with tkeep == 2'b11 -- correctly shaped
        the EP computes    crc_reversed = 0x8ef8   <- IDENTICAL

    So the four payload bytes reach the receiving CRC engine byte-for-byte
    intact, through scrambler, codec, bridge and descrambler. The compare fails
    anyway, because the beat that should carry the CRC arrives with tkeep = 0
    instead of 2'b11, and the CRC bytes turn up INSIDE a following full-width
    word (0x8ef840f8, whose upper half is the CRC). dllp_crc_word_valid (:129)
    never asserts, ST_CHECK_CRC falls to its else arm (:241), and the DLLP is
    dropped SILENTLY -- no counter, no error output.

    The receive path is handing dllp_handler a CONTINUOUS FULL-WIDTH STREAM
    where the transmitter sent DISCRETE TWO-BEAT FRAMES.

    !! WHAT IS EXONERATED BY MEASUREMENT, so a fix does not start in the wrong
    module (row 1a asserts the first four; F16 adds the rest):
      - the codec: zero code errors, zero disparity errors, zero illegal K;
      - the bridge: 43744 beats in, 43743 out -- one register of window edge;
      - the scramblers: tx advanced 42637 times, rx 42635, drift 2 in 57000;
      - the LTSSMs: both reach L0, both DLLs enter DL_Init and originate;
      - SYMBOL ORDER: 37451 ONE-WAY comparisons across both directions, zero
        mismatches on data and K flags (a round trip is blind to a consistent
        transposition; this is not a round trip);
      - the CRC logic: pcie_datalink_crc is seeded .crcIn(16'hFFFF) hardcoded,
        stateless per beat, so there is no accumulator to pollute; and the
        transmit and receive sides use arithmetically identical conventions.

    So the defect is in DLLP DELINEATION on the receive path, between
    phy_receive and dllp_handler. It is SHARED RTL rather than either party's
    own top, and it is reachable for the first time here because this is the
    first bench in the project where two real logical PHYs face each other.

    !! THIS IS NOT JOY'S ENDPOINT FAILING. Its transmit side is conformant --
    TXCAP-EP matches TXCAP-RC beat for beat, CRCs included -- and it trains,
    enters DL_Init and originates. A report saying "the Endpoint does not
    answer" would be true and deeply misleading.

    ⚠️ AN EARLIER VERSION OF THIS BODY SAID "the Endpoint sees 156 well-framed
    DLLPs ... and ALL 156 fail the CRC compare", and claimed the two directions
    failed differently. BOTH CLAIMS WERE WRONG. 156 was a count of
    dllp_crc_word_valid, which is a COMBINATIONAL tkeep/tlast SHAPE PREDICATE
    evaluated on every beat and gated on neither UserIsDllp nor the FSM state --
    counting a predicate is not counting an event. Re-measured on the handler's
    own acceptance conditions: 13771 DLLP-marked beats at the EP, 13846 at the
    RC, zero completed frames either side. The two sides look the SAME, and the
    asymmetry that motivated "two defects" was an artifact of the wrong counter.

    THE NEXT PROBE, so the next session does not re-derive it: dump the
    descrambled byte stream with K flags at the EP's phy_receive input across
    one DLLP, and follow tkeep/tlast through block_alignment -> pack_data ->
    dllp_receive. One question: where is the END character's tkeep = 2'b11
    generated on the receive side, and is that code reached at all?
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
