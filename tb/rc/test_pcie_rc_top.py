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
                self.fc2_stored = True
            st = int(self.fci.curr_state.value)
            self.states.add(st)
            if 1 <= st <= 6:
                self.fc1_tx_states.add(st)


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

    # ---- FC-init COMPLETION is NOT asserted here, and that is deliberate ------
    #
    # It requires a PEER. Measured 2026-09-10 with the probe below:
    #     dl_link_up=True  start_fc=True  fc1_stored=False
    # i.e. the RC originates InitFC1 for the whole window and never receives one.
    # A PIPE loopback cannot answer: phy_transmit scrambles and phy_receive
    # descrambles, so a self-loop puts one LFSR against its own output and the
    # echoed DLLPs do not survive LCRC.
    #
    # This is a property of the FAR END, not of the RTL -- the CDC path into the
    # DLL was the other candidate and the probe cleared it. Completion, and with
    # it the defect-#3 monotonicity check, move to SS63 #7b, where the far end is
    # Joy's real Endpoint behind a codec bridge.
    assert not fc.rose, (
        "UNEXPECTED: fc_initialized_o rose against a loopback far end. That "
        "would mean the DLL accepted its own echoed InitFC1, and the #7b "
        "premise -- that completion needs a real peer -- is wrong. Investigate "
        "before treating this as good news"
    )

    dut._log.info(
        "ROW 1: link_up_o rose at cycle %d; DLL reached DL_Init and originated "
        "InitFC1 (states %s); FC-init completion deferred to #7b (needs a peer)",
        link.rise_cycle, sorted(probe.fc1_tx_states),
    )
