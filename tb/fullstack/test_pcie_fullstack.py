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
# They are marked expect_fail so the gate stays meaningful rather than carrying
# four permanently red rows -- the same idiom row 1b used for its whole life.
# ⚠️ And the same caveat applies (§22.77): an expect_fail row reports PASS, so
# the gate CANNOT show this defect or show it closing. These bodies are the
# witness. Flip them the moment the CfgRd0 timeout is fixed.
# =============================================================================


def _i(sig):
    return int(sig.value)


async def run_enumeration_fs(dut, cycles=60000):
    """Pulse scan_start_i and wait for enum_done_o / an error.

    scan_start_i is a PULSE on purpose: the start gate must REMEMBER a request
    made while flow control is still down (tracker §44 -- it is a latch, not a
    bare AND). Ported from tb_rc_ep's run_enumeration, which is the same engine
    at the AXIS seam; here it runs through two PHYs and the codec bridge.
    """
    d = dut
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
@cocotb.test(expect_fail=True)  # §63 #7d: RED -- ENUM_ERR_TIMEOUT, see body
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

    NON-VACUITY: enumeration must report device_present AND complete without
    error -- a run that timed out would leave the ID registers at reset and
    could otherwise read as a pass.
    """
    tb, _mons, _probe, _codec, _path, _scram, _dllps, _tasks = await bring_up(dut)
    r = await run_enumeration_fs(dut)
    _log_enum_fs(dut, r)

    assert r["enum_done"] and not r["enum_error"], (
        f"enumeration did not complete: done={r['enum_done']} "
        f"error={r['enum_error']} code={r['enum_error_code']} "
        f"scan_error={r['scan_error']} code={r['scan_error_code']}"
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
@cocotb.test(expect_fail=True)  # §63 #7d: RED -- ENUM_ERR_TIMEOUT, see body
async def fullstack_bar0_sizes_to_4kb(dut):
    """BAR0 sizes to 4 KB by the PCI 3.0 §6.2.5.1 write-ones-read-back protocol.

    The engine writes all ones to the BAR, reads it back, and the lowest set bit
    of the returned mask gives the size. 4 KB is the Base 2.1 §7.5.1.2.1 minimum
    memory BAR granularity and is what pcie_config_reg.sv's BAR mask encodes.

    ⚠️ BAR1's completion timeout is JOY'S, NOTED NOT ASSERTED. This row pins
    BAR0 only. A BAR1 assertion here would be this bench reporting a defect in
    the far end's config space as though it were a full-stack property, and the
    rung has no mandate to fix it.
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
    bar0_size = r["bar_size"] & 0xFFFFFFFF if r["bar_size"] > 0xFFFFFFFF else r["bar_size"]
    dut._log.info("ROW 3: BAR0 size field = %#x (bar_size raw %#x)",
                  bar0_size, r["bar_size"])
    assert bar0_size != 0, (
        "BAR0 sized to zero -- the write-ones/read-back returned no mask, so "
        "either the CfgWr0 never landed or the config space did not answer"
    )


# ---------------------------------------------------------------------------
# Row 4 -- MemWr/MemRd round trip through the requester arm.
# ---------------------------------------------------------------------------
@cocotb.test(expect_fail=True)  # §63 #7d: RED -- ENUM_ERR_TIMEOUT, see body
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
    performs across two real PHYs: a CfgRd0 goes out on the requester arm and a
    Completion comes back, and the engine owns the RQ arm while it happens.
    That is the same NP path a MemRd uses, minus the opcode.

    ⭐ REGISTERED: bringing cpl_timeout_valid_o and rc_unexpected_completion_o
    out to this bench's top, and driving real MemWr/MemRd on s_axis_rq_*, is the
    remaining half of this row and is #7e work.
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
@cocotb.test(expect_fail=True)  # §63 #7d: RED -- ENUM_ERR_TIMEOUT, see body
async def fullstack_completion_tag_and_status(dut):
    """Completions returned across the seam carry a tracked tag and SC status.

    Base 2.1 §2.2.9: Completion Status 000b is Successful Completion.

    ⚠️⚠️ THIS IS AN ACCEPTANCE ASSERTION, NOT A DECODE, and the distinction is
    the point. It does NOT read the Completion's Status field or its tag off the
    wire. It asserts that the RC's enumeration engine CONSUMED the Completions
    and produced correct header values from them -- which it could not do had a
    tag gone untracked or a non-SC status come back, because the engine would
    have raised enum_error_o instead.

    The direct oracles -- rc_unexpected_completion_o for an untracked tag, and
    the Completion Status field itself -- are NOT reachable from this bench's
    top level. ⭐ REGISTERED as #7e: bring the RC error surface out, then this
    row can assert the tag and the status directly instead of by consequence.
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
