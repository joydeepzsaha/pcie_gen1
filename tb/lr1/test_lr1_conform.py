"""test_lr1_conform -- pcie_rc_top against the LR1 far end, conforming behaviour

Author: Kourosh Ghahramani
Silicon Systems Research Lab, University of Washington

Purpose
    LR1-a step 2 (BRIEF_LR1 §5): the Root Complex trains to L0, completes
    flow-control initialization and enumerates a device it has never been
    built against -- the LR1 far end, an upstream port written from Base 2.1
    and independent of the repository's RTL (D4) -- with the far end checking
    everything the RC transmits (D6). Pins only (D3); SIM_FAST_LINK = 0
    (RULINGS_LR1 D).

    conform_default   every knob at its default (lr1_farend.Knobs). Asserts:
                      the RC reaches L0 (link_up_o) and FC init (fc_init_done_o);
                      enumeration ends with enum_done_o, no error, device 0
                      present with the far end's IDs and one 64-bit memory BAR
                      of the far end's size; the far end itself reached L0 and
                      DL_Active; no far-end check flags anything outside the
                      registered set KNOWN; every check examined something.

KNOWN holds the RC non-conformances that are registered and expected here:
    SKP_GAP_TRAINING  no SKP Ordered Set before LinkUp (BRIEF_LR1 §0, P2) --
                      registered; its pinned red row is LR1-b's (D9).
A flag in KNOWN is reported and counted, not failed: the row asserts it is the
only kind present, so a new kind fails the row.

References
    PCIe Base Spec r2.1, §4.2.6, §3.3.1, §3.5, §2.6.1, §7.5
"""
import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles

from lr1_farend import FarEnd, Knobs
import lr1_cfg as CFG

CLK_NS = 8
MAX_CYCLES = 80000           # 640 us; the default row needs about a third of it
TAIL_CYCLES = 6250           # 50 us after enum_done_o: an UpdateFC window and more

KNOWN = {'SKP_GAP_TRAINING'}

# Checks that must have examined at least one item in a conforming run (§22.82).
NONVACUOUS = ('TS_FIELDS', 'POLL_TS1_COUNT', 'TS2_AFTER_RX', 'PKT_BEFORE_L0', 'SKP_FORMAT', 'SKP_GAP',
              'DLLP_CRC', 'DLLP_FIELDS', 'LCRC', 'SEQ', 'TLP_FORMAT', 'ACK_SEQ', 'ACK_LATENCY',
              'FC_INIT_ORDER', 'FC_INIT_PERIOD', 'FC_UPDATE_PERIOD', 'FC_OVERFLOW')


class Bench:
    def __init__(self, dut):
        self.dut = dut
        cocotb.start_soon(Clock(dut.clk_i, CLK_NS, units='ns').start())

    async def reset(self, fe):
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
        d.s_axis_rq_tdata.value = 0
        d.s_axis_rq_tkeep.value = 0
        d.s_axis_rq_tvalid.value = 0
        d.s_axis_rq_tlast.value = 0
        d.s_axis_rq_tuser.value = 0
        d.m_axis_rc_tready.value = 1
        fe.drive_reset_values()
        await ClockCycles(d.clk_i, 10)
        d.rst_i.value = 0
        await ClockCycles(d.clk_i, 2)
        # the host: start one scan with the BAR stage (pcie_rc_top latches a
        # start that arrives before FC init, start_pending_r)
        d.bar_enable_i.value = 1
        d.scan_start_i.value = 1
        await ClockCycles(d.clk_i, 1)
        d.scan_start_i.value = 0
        d.en_i.value = 1
        d.transmit_enable_i.value = 1


class Rise:
    """Cycle a level first read 1 (sampled by the far end's clock loop)."""

    def __init__(self):
        self.at = {}

    def sample(self, dut, cycle, names):
        for n in names:
            if n not in self.at and int(getattr(dut, n).value):
                self.at[n] = cycle


async def run_row(dut, knobs, name):
    tb = Bench(dut)
    fe = FarEnd(dut, knobs)
    await tb.reset(fe)
    rise = Rise()
    names = ('link_up_o', 'fc_init_done_o', 'scan_done_o', 'enum_done_o', 'enum_error_o', 'scan_error_o')
    orig = fe.step

    def step():
        orig()
        rise.sample(dut, fe.cycle, names)
    fe.step = step
    await fe.run(MAX_CYCLES, until=lambda: 'enum_done_o' in rise.at or 'enum_error_o' in rise.at)
    await fe.run(TAIL_CYCLES)
    chk = fe.finish()
    kinds = {}
    for f in chk.flags:
        kinds[f['id']] = kinds.get(f['id'], 0) + 1
    dut._log.info('ROW|%s|rc_rise=%s' % (name, sorted(rise.at.items(), key=lambda x: x[1])))
    dut._log.info('ROW|%s|farend_ltssm=%s' % (name, fe.ltssm.history))
    dut._log.info('ROW|%s|farend_dll=%s stats=%s' % (name, fe.dll.history, fe.dll.stats))
    dut._log.info('ROW|%s|flags=%s' % (name, sorted(kinds.items())))
    dut._log.info('ROW|%s|checked=%s' % (name, sorted(chk.n.items())))
    dut._log.info('ROW|%s|notes=%s ltssm_events=%s' % (name, chk.notes, fe.ltssm.events))
    dut._log.info('ROW|%s|measured=%s' % (name, chk.summary()))
    dut._log.info('ROW|%s|requests=%d cycles=%d' % (name, len(fe.tl.requests), fe.cycle))
    for cyc, t, st in fe.tl.requests:
        dut._log.info('REQ|%s|%d|%s|reg=%s|be=%s|data=%s|status=%s' % (
            name, cyc, t.get('name'), '%03x' % t['reg'] if 'reg' in t else '-', t.get('first_be', '-'),
            t['payload'][:4].hex() if t.get('payload') else '-', st))
    return fe, chk, rise, kinds


def assert_conforming(dut, fe, chk, rise, kinds, bar_size, prefetch):
    for n in ('link_up_o', 'fc_init_done_o', 'enum_done_o'):
        assert n in rise.at, '%s never rose (rises: %s)' % (n, rise.at)
    assert 'enum_error_o' not in rise.at and 'scan_error_o' not in rise.at, 'enumeration error: %s' % rise.at
    assert int(dut.device_present_o.value) == 1, 'device_present_o = 0'
    assert int(dut.vendor_id_o.value) == CFG.VENDOR_ID, 'vendor_id_o %04x' % int(dut.vendor_id_o.value)
    assert int(dut.device_id_o.value) == CFG.DEVICE_ID, 'device_id_o %04x' % int(dut.device_id_o.value)
    assert int(dut.header_type_o.value) == 0x00, 'header_type_o %02x' % int(dut.header_type_o.value)
    valid = int(dut.bar_valid_o.value)
    is64 = int(dut.bar_is_64_o.value)
    pre = int(dut.bar_prefetch_o.value)
    size0 = int(dut.bar_size_o.value) & ((1 << 64) - 1)
    assert valid & 1 and is64 & 1, 'BAR0 valid %x is_64 %x' % (valid, is64)
    assert (pre & 1) == int(prefetch), 'BAR0 prefetch %x' % pre
    assert size0 == bar_size, 'BAR0 size %x, far end %x' % (size0, bar_size)
    assert fe.ltssm.history[-1][1] == 'L0' and 'L0' in [s for _, s in fe.ltssm.history], \
        'far end LTSSM %s' % fe.ltssm.history
    assert fe.dll.state == 'ACTIVE', 'far end DLL %s' % fe.dll.state
    unknown = {k: v for k, v in kinds.items() if k not in KNOWN}
    assert not unknown, 'far-end checks flagged %s; first: %s' % (
        unknown, [f for f in chk.flags if f['id'] in unknown][:3])
    vac = [c for c in NONVACUOUS if not chk.n.get(c)]
    assert not vac, 'checks that examined nothing: %s' % vac


@cocotb.test()
async def conform_default(dut):
    """Row 0: every knob at its default."""
    k = Knobs()
    fe, chk, rise, kinds = await run_row(dut, k, 'conform_default')
    assert_conforming(dut, fe, chk, rise, kinds, k.bar_size, k.bar_prefetchable)
