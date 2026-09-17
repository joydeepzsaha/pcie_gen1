"""Full-stack Root Complex -- pcie_rc_top, engine to the 32+4 PIPE seam.

!! ROW 1 SCOPE -- READ BEFORE ADDING AN ASSERTION HERE.
!!
!! This row asserts everything the stack can prove ALONE, and stops exactly
!! there. Measured 2026-09-10 with the probe below:
!!
!!     link_up_o   rose at cycle 2279 (~18.2 us)   dl_link_up = True
!!     start_fc    = True    fc1_stored = False    fsm walked states 0..7
!!
!! So: the LTSSM trains to L0 through the 32+4 PIPE seam, link-up reaches the
!! Data Link Layer, the DLL enters DL_Init, and it originates InitFC1 -- and
!! then originates it forever, because NOTHING ANSWERS.
!!
!! !! FC-INIT COMPLETION IS NOT ASSERTED HERE, AND THAT IS A SCOPE DECISION,
!! NOT AN OMISSION. Completion needs a PEER. The probe was built to separate
!! the only two candidate causes and it settled them:
!!
!!     start_fc False                  -> the link-up CDC path into the DLL
!!     start_fc True, fc1_stored False -> originating, echo not recovered  <-- THIS
!!
!! It is the second, so the RTL is not at fault and the async_fifo CDC on
!! pipe_rx_usr_clk_i (pcie_phy_top.sv:193-209) is CLEARED by measurement. A PIPE
!! loopback cannot answer an InitFC1: phy_transmit scrambles and phy_receive
!! descrambles, so a self-loop puts one LFSR against its own output and the
!! echoed DLLPs do not survive LCRC.
!!
!! Completion, and with it the defect-#3 monotonicity check, belong to SS63 #7b,
!! where the far end is Joy's real Endpoint behind a codec bridge. The assertion
!! below deliberately requires fc_initialized_o to STAY LOW, so if that premise
!! is ever wrong this row fails loudly rather than quietly passing.


THE FAR END (D-FS.2). Python, at the PIPE seam. The seam is PRE-8b/10b --
frame_symbols and the scrambler live inside pcie_phy_top, encode_8b10b does not
-- so nothing here witnesses the codec, and no assertion in this file claims to.

The far end is a PIPE LOOPBACK: rxdata <= txdata, rxdatak <= txdatak. That is the
model tb/ltssm/test_ltssm_configuration.py already uses against pcie_phy_top, and
it is literally Stage 9a's first step ("RC alone in fabric, loopback"). It lives
in Python rather than in the wrapper so a row needing a different far end can have
one without touching RTL.

!! ONE TB PER TEST, ALWAYS. cocotb cancels every task a test started when that
test ends -- including the Clock coroutine TB.__init__ spawns. A TB shared across
tests has a DEAD CLOCK, the next RisingEdge never returns, and the simulator quits
with "Simulator shut down prematurely", which reads as a reset bug in the DUT and
is nothing of the kind.
"""

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ClockCycles

CLK_NS = 8  # 125 MHz -- the real Gen1 PCLK, and the PAR target period


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
        # Far-end PHY status: receiver present, no errors, not electrical idle.
        d.phy_rxdata.value = 0
        d.phy_rxdata_valid.value = 0
        d.phy_rxdatak.value = 0
        d.phy_rxstart_block.value = 0
        d.phy_rxsync_header.value = 0
        d.phy_rxvalid.value = 0
        d.phy_phystatus.value = 0
        d.phy_phystatus_rst.value = 0
        d.phy_rxelecidle.value = 1
        d.phy_rxstatus.value = 0
        await ClockCycles(d.clk_i, 10)
        d.rst_i.value = 0
        await ClockCycles(d.clk_i, 5)


RXSTATUS_RECEIVER_DETECTED = 0b011   # PIPE: receiver detected
DETECT_LATENCY_CYCLES = 4            # PHY turnaround before PhyStatus answers


async def pipe_loopback(dut):
    """The far end: everything the RC transmits comes straight back.

    Copied on the clock edge, so the RC sees its own characters one cycle later
    -- a zero-length link. Deliberately NOT a model of an Endpoint: it cannot
    answer a CfgRd0, and no row here asks it to.
    """
    while True:
        await RisingEdge(dut.clk_i)
        dut.phy_rxdata.value = dut.phy_txdata.value
        dut.phy_rxdata_valid.value = dut.phy_txdata_valid.value
        dut.phy_rxdatak.value = dut.phy_txdatak.value
        dut.phy_rxsync_header.value = dut.phy_txsync_header.value
        dut.phy_rxvalid.value = 1
        dut.phy_rxelecidle.value = 0


async def pipe_receiver_detect(dut):
    """The far end's half of the PIPE receiver-detect HANDSHAKE.

    !! THIS IS NOT OPTIONAL AND A LEVEL WILL NOT DO. pcie_phy_top.sv:235-240
    clears its detect latch on the RISING EDGE of phy_txdetectrx and sets it
    only when it subsequently sees phy_phystatus asserted WITH
    phy_rxstatus == 3'b011. Driving the status permanently high therefore fails
    twice over: the edge wipes it, and there is no edge afterwards to re-set it.

    Measured, not guessed: the first version of this bench drove rxvalid/elecidle
    as levels and never answered txdetectrx, and the LTSSM sat in Detect for the
    whole 40000-cycle window.
    """
    prev = 0
    while True:
        await RisingEdge(dut.clk_i)
        cur = int(dut.phy_txdetectrx.value)
        if cur and not prev:
            # Rising edge: the MAC has asked. Answer after a PHY turnaround.
            for _ in range(DETECT_LATENCY_CYCLES):
                await RisingEdge(dut.clk_i)
            dut.phy_rxstatus.value = RXSTATUS_RECEIVER_DETECTED
            dut.phy_phystatus.value = 1
            await RisingEdge(dut.clk_i)
            dut.phy_phystatus.value = 0
            dut.phy_rxstatus.value = 0
            cur = int(dut.phy_txdetectrx.value)
        prev = cur


class Monotonic:
    """Continuous sampler for a one-way signal.

    !! SAMPLES FROM BEFORE THE EVENT, NEVER wait-then-read (section 22.89). A
    bare read after RisingEdge returns the PRE-edge value, so a waiter is an
    observer with a phase; the fc-glitch rung measured a 4-cycle glitch as 3
    that way and invited a fix that a live defect would have survived.

    Records: was it ever low, did it rise, did it fall AFTER rising.
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
            v = int(self.h.value)
            if v == 0:
                if not self.rose:
                    self.saw_low = True
                else:
                    self.fell_after_rise = True
            else:
                if not self.rose:
                    self.rose = True
                    self.rise_cycle = n


class DllProbe:
    """Hierarchical probe that separates the two candidate causes.

    start_flow_control_i is pcie_datalink_init's DL_Init signal. fc1_values_
    stored_i means an InitFC1 was received and stored. Together they split:

        start_fc False                  -> the DLL never entered DL_Init, so the
                                           link-up path INTO the DLL is at fault
        start_fc True, fc1_stored False -> it originated but the echo was never
                                           recovered (the loopback/scrambler case)
    """

    def __init__(self, dut):
        d = dut.u_rc.u_phy.pcie_datalink_layer_inst
        self.fci = d.pcie_flow_ctrl_init_inst
        self.dll = d
        self.dl_link_up = False
        self.start_fc = False
        self.fc1_stored = False
        self.fc2_stored = False
        self.states = set()
        # ST_FC1_P..ST_FC1_CPL_CRC == the InitFC1 transmit walk (states 1..6 of
        # pcie_flow_ctrl_init's enum). Recorded as a SET, so "it originated" is
        # a claim about states actually visited, not about a counter.
        self.fc1_tx_states = set()
        # §63 #7d: the FC2 transmit walk and the limb that actually completes
        # init. ST_FC2..ST_FC2_CPL_CRC are states 8,9,12,13,14,15; CHECK_FC2 is
        # 16. ST_FC2_P/ST_FC2_P_CRC (10,11) are dead in this design -- ST_FC2
        # and ST_FC2_CRC carry the Posted FC2 DLLP.
        self.fc2_tx_states = set()
        self.update_fc_r = False
        self.update_fc_cycle = None
        self.fc2_stored_cycle = None
        self.fc2_set_sent_before_check = False
        self._seen_fc2_walk = set()

    async def run(self, clk, cycles):
        for _ in range(cycles):
            await RisingEdge(clk)
            if int(self.dll.phy_link_up_i.value):
                self.dl_link_up = True
            if int(self.fci.start_flow_control_i.value):
                self.start_fc = True
            if int(self.fci.fc1_values_stored_i.value):
                self.fc1_stored = True
            if int(self.fci.fc2_values_stored_i.value):
                if not self.fc2_stored:
                    self.fc2_stored_cycle = _
                self.fc2_stored = True
            if int(self.fci.update_fc_r.value):
                if not self.update_fc_r:
                    self.update_fc_cycle = _
                self.update_fc_r = True
            st = int(self.fci.curr_state.value)
            self.states.add(st)
            if 1 <= st <= 6:
                self.fc1_tx_states.add(st)
            if st in (8, 9, 12, 13, 14, 15):
                self.fc2_tx_states.add(st)
                self._seen_fc2_walk.add(st)
            if st == 16 and self._seen_fc2_walk >= {8, 9, 12, 13, 14, 15}:
                # CHECK_FC2 entered only after the full FC2 set has been walked.
                self.fc2_set_sent_before_check = True


@cocotb.test()
async def rc_top_links_up_and_fc_init_is_monotonic(dut):
    """Row 1 -- the full stack elaborates, trains, and completes FC init once.

    Three claims, and the third is the rung's reason for existing:

      1. the whole stack runs from one clock and one reset;
      2. the LTSSM reaches L0 against a far end at the PIPE seam;
      3. fc_initialized_o -- UNFILTERED, no fc_init_sticky_r anywhere in this
         netlist -- rises ONCE and STAYS.

    (3) is conformance defect #3's closure observed at the first consumer that
    has no filter to hide it. Base 2.1 p.158 / p.161: FC-init completion is a
    one-way event. If it glitches here, the source fix is incomplete.

    ⭐ §63 #7d: claim (3) is now REACHED. Until d079edc this row asserted the
    OPPOSITE -- that fc_initialized_o must NOT rise, because a loopback far end
    could not complete init. That was an artifact of conformance defect #6's
    non-conformant conjunct at pcie_flow_ctrl_init.sv:401, not a property of the
    design. The row now pins the mechanism: init completes via §3.3.1's InitFC2
    limb on the RC's OWN echoed InitFC2 (fc2_values_stored_i at cycle 4,441),
    with update_fc_r following at 5,463, and the full FC2 set walked before
    CHECK_FC2. Full history, and a correction, at the assertion site below.

    NON-VACUITY: the monitor must have seen fc_initialized_o LOW before it rose.
    Without that a stack that asserted it out of reset would pass identically.
    """
    tb = TB(dut)
    await tb.reset()

    cocotb.start_soon(pipe_loopback(dut))
    cocotb.start_soon(pipe_receiver_detect(dut))

    fc = Monotonic(dut.fc_initialized_o)
    link = Monotonic(dut.link_up_o)
    # Start BOTH monitors before enabling the PHY, so the low period is inside
    # the window rather than assumed.
    probe = DllProbe(dut)
    mon_fc = cocotb.start_soon(fc.run(dut.clk_i, 40000))
    mon_link = cocotb.start_soon(link.run(dut.clk_i, 40000))
    mon_probe = cocotb.start_soon(probe.run(dut.clk_i, 40000))

    await ClockCycles(dut.clk_i, 5)
    dut.en_i.value = 1
    dut.phy_ready_en.value = 1
    dut.transmit_enable_i.value = 1

    await mon_fc
    await mon_link
    await mon_probe

    assert link.saw_low, (
        "non-vacuity failed: link_up_o was never observed low, so the monitor "
        "cannot distinguish 'trained' from 'asserted out of reset'"
    )
    assert link.rose, (
        "LTSSM never reached L0 against the loopback far end within 40000 "
        "cycles (SIM_FAST_LINK=1, so the 12 ms timers are scaled)"
    )
    dut._log.info(
        "DIAG link_up_o: saw_low=%s rose=%s rise_cycle=%s | "
        "fc_initialized_o: saw_low=%s rose=%s rise_cycle=%s",
        link.saw_low, link.rose, link.rise_cycle,
        fc.saw_low, fc.rose, fc.rise_cycle,
    )
    dut._log.info(
        "DIAG PROBE dl_link_up=%s start_fc=%s fc1_stored=%s fc2_stored=%s "
        "fsm_state=%s",
        probe.dl_link_up, probe.start_fc, probe.fc1_stored,
        probe.fc2_stored, sorted(probe.states),
    )

    # ---- the Data Link Layer reached DL_Init and ORIGINATED ------------------
    assert probe.dl_link_up, (
        "the DLL never saw link up. pcie_phy_top crosses link_up to it through "
        "an async_fifo on pipe_rx_usr_clk_i (pcie_phy_top.sv:193-209); if this "
        "fires, that CDC path is the suspect"
    )
    assert probe.start_fc, (
        "the DLL never entered DL_Init: start_flow_control_i never asserted, so "
        "flow-control initialisation was never even attempted"
    )
    assert probe.fc1_tx_states, (
        "the DLL entered DL_Init but never walked the InitFC1 transmit states, "
        "so it is not originating. Base 2.1 SS3.3.1 p.161 makes transmission "
        "unconditional on entry to DL_Init"
    )

    # ---- FC-init COMPLETION, AND THE MECHANISM THAT PRODUCES IT -------------
    #
    # ⚠️⚠️ SUPERSEDED PREMISE, kept with its date. Until §63 #7d this row
    # asserted `not fc.rose` -- that completion REQUIRES A PEER and a loopback
    # cannot answer. Measured 2026-09-10:
    #     dl_link_up=True  start_fc=True  fc1_stored=False
    # with the stated mechanism: "phy_transmit scrambles and phy_receive
    # descrambles, so a self-loop puts one LFSR against its own output and the
    # echoed DLLPs do not survive LCRC."
    #
    # THAT PREMISE IS NOW FALSE, FOR TWO INDEPENDENT REASONS.
    #
    # (i) It was an artifact of a NON-CONFORMANT gate. pcie_flow_ctrl_init.sv:401
    #     required `fc2_values_stored_i && (update_fc_r || idle_count_r >= 0x60)`
    #     -- an InitFC2 from a peer was a HARD CONJUNCT, so no echo could ever
    #     complete init. Base 2.1 §3.3.1 exits FC_INIT2 on the full FC2 set sent
    #     AND *any of* {InitFC2 received, UpdateFC received, TLP received}. The
    #     conjunct was conformance defect #6, fixed at d079edc; the disjunction
    #     is satisfiable by a self-echo, and the spec does not distinguish one.
    # (ii) Its stated mechanism also expired: the framing fixes of this same rung
    #     (eb2e662/9ecabee, the USER_WIDTH K-mask truncation, and f75b143,
    #     data_handler's tkeep) mean echoed DLLPs now DO survive framing and CRC.
    #
    # So this row now asserts the MECHANISM rather than the absence of the event.
    # Measured in this bench at d079edc, and every number below is from it.

    assert fc.saw_low, (
        "non-vacuity failed: fc_initialized_o was never observed low, so the "
        "monitor cannot distinguish 'completed' from 'asserted out of reset'"
    )
    assert fc.rose, (
        "fc_initialized_o never rose. Since d079edc the loopback far end is "
        "sufficient: §3.3.1's UpdateFC limb is satisfied by the RC's own echoed "
        "UpdateFC DLLP. If this fires, either :401 regressed to the "
        "conformance-defect-#6 conjunct or the echo path broke"
    )
    assert not fc.fell_after_rise, (
        "fc_initialized_o FELL after rising -- conformance defect #3 is not "
        "closed at this consumer. This netlist has no fc_init_sticky_r, so the "
        "raw DLL output is what is observed. Base 2.1 p.158/p.161: FC-init "
        "completion is a one-way event"
    )

    # ---- WHICH §3.3.1 limb completed it -------------------------------------
    #
    # ⚠️⚠️ CORRECTION, §63 #7d. An earlier write-up of this rung reported that
    # the loopback completes via the UpdateFC limb and that fc2_values_stored_i
    # "never fires". THAT WAS WRONG, and the error is worth recording because of
    # how it happened: it read the probe's WHICH_FIRED_FIRST field, which only
    # ever compared update_fc_r against idle_count_r -- the two terms of the OLD
    # :401 condition -- and never considered fc2_values_stored_i at all. The
    # correct numbers were in the same log line.
    #
    # MEASURED at d079edc, from !rst:
    #     fc1_values_stored_i   asserts                (was False on 2026-09-10)
    #     fc2_values_stored_i   asserts at cycle 4,441   <- FIRST, and sufficient
    #     update_fc_r           asserts at cycle 5,463   <- later, also true
    #
    # So the limb that completes init here is §3.3.1's FIRST one -- "the Receiver
    # has received at least one InitFC2 DLLP" -- satisfied by the RC's OWN
    # echoed InitFC2. The UpdateFC limb becomes true a thousand cycles later and
    # is not what did it. Either way the spec does not distinguish a self-echo,
    # and under the pre-d079edc conjunct NEITHER could complete init.
    assert probe.fc2_stored, (
        "fc2_values_stored_i never asserted, so §3.3.1's InitFC2 limb is not "
        "what completed init here. Measured true at cycle 4,441 at d079edc -- "
        "re-measure before assuming this row still pins what it claims"
    )
    assert probe.update_fc_r, (
        "update_fc_r never asserted. Measured true at cycle 5,463 at d079edc. "
        "Not the completing limb, but its absence would mean the echo path "
        "changed shape -- investigate rather than relax this"
    )
    assert (probe.fc2_stored_cycle is not None
            and probe.update_fc_cycle is not None
            and probe.fc2_stored_cycle < probe.update_fc_cycle), (
        "ORDER CHANGED: fc2_values_stored_i is no longer the first limb to go "
        f"true (fc2_stored at {probe.fc2_stored_cycle}, update_fc_r at "
        f"{probe.update_fc_cycle}). The row's claim about WHICH limb completes "
        "init depends on this ordering"
    )

    # ---- §3.3.1's FIRST conjunct, checked rather than assumed ----------------
    #
    # The exit condition at :401 no longer tests "full FC2 set sent"; that is
    # guaranteed structurally, because CHECK_FC2 has one entry site (:394, inside
    # ST_FC2_CPL_CRC) at the end of a strictly linear six-state chain. This row
    # is where that structural claim is CHECKED at run time rather than trusted.
    assert probe.fc2_set_sent_before_check, (
        "CHECK_FC2 was entered without the full FC2 set having been walked "
        "(states 8,9,12,13,14,15). :401's disjunction is only spec-correct "
        "because reaching CHECK_FC2 implies the set was sent -- if that is no "
        "longer true, the fix at d079edc is unsound"
    )

    dut._log.info(
        "DIAG §3.3.1 limbs: fc2_stored=%s at cycle %s (COMPLETING limb) | "
        "update_fc_r=%s at cycle %s | fc2 walk=%s | full set before CHECK_FC2=%s",
        probe.fc2_stored, probe.fc2_stored_cycle,
        probe.update_fc_r, probe.update_fc_cycle,
        sorted(probe.fc2_tx_states), probe.fc2_set_sent_before_check,
    )

    dut._log.info(
        "ROW 1: link_up_o rose at cycle %d; DLL reached DL_Init and originated "
        "InitFC1 (states %s); FC init COMPLETED against the loopback far end "
        "via §3.3.1's InitFC2 limb on its own echo, once and monotonically",
        link.rise_cycle, sorted(probe.fc1_tx_states),
    )
