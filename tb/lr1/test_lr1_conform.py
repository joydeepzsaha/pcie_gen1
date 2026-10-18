"""test_lr1_conform -- pcie_rc_top against the LR1 far end: conforming and legal-variation rows

Author: Kourosh Ghahramani
Silicon Systems Research Lab, University of Washington

Purpose
    LR1-a steps 2 and 3 (BRIEF_LR1 §5): the Root Complex trains to L0,
    completes flow-control initialization and enumerates a device it has never
    been built against -- the LR1 far end, an upstream port written from Base
    2.1 and independent of the repository's RTL (D4) -- with the far end
    checking everything the RC transmits (D6). Pins only (D3); SIM_FAST_LINK =
    0 (RULINGS_LR1 D).

    conform_default   every knob at its default (lr1_farend.Knobs).
    vary_*            one knob changed: a behaviour Base 2.1 allows and our
                      Endpoint does not do (D7; PHASE0_LR1 §5.3; RULINGS_LR1
                      E, G, I; RULINGS_A 6), each row witnessing that its
                      stimulus fired.

A green row asserts: the RC reaches L0 (link_up_o) and FC init
(fc_init_done_o); enumeration ends with enum_done_o, no error, device 0 present
with the far end's IDs and its BAR0; the far end reached L0 and DL_Active; no
far-end check flags anything outside KNOWN; every check in NONVACUOUS examined
something.

A row whose knob reproduces a registered defect is an expect_fail row that
fails only at its one pinned assertion (§22.93): the run and its non-vacuity
checks are inside a guard that logs PINNED_RED|<row>|NOT_REACHED and returns,
and PINNED_RED|<row>|REACHED is logged just before the pinned assertion.

KNOWN holds the RC non-conformances that are registered and expected here:
    SKP_GAP_TRAINING  no SKP Ordered Set before LinkUp (BRIEF_LR1 §0, P2) --
                      registered; its pinned red row is LR1-b's (D9).
A flag in KNOWN is reported and counted, not failed: the row asserts it is the
only kind present, so a new kind fails the row.

References
    PCIe Base Spec r2.1, §4.2.6, §4.2.7, §3.3.1, §3.5, §2.6.1, §7.5
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


async def run_row(dut, knobs, name, until=None, max_cycles=MAX_CYCLES, tail=TAIL_CYCLES, after=None):
    """Reset, run the far end until `until(fe, rise)` (default: enumeration
    ended), then `after(fe, rise)` (a coroutine, optional), then `tail` more
    clocks; returns (fe, chk, rise, kinds)."""
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
    if until is None:
        until = lambda f, r: 'enum_done_o' in r.at or 'enum_error_o' in r.at
    await fe.run(max_cycles, until=lambda: until(fe, rise))
    if after is not None:
        await after(fe, rise)
    await fe.run(tail)
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
    dut._log.info('ROW|%s|farend_sent: com_lanes=%s skp_ns=%s skp_gaps=%s b2b=%d ack_lat=%s ts_counts=%s' % (
        name, fe.com_lanes, sorted(fe.skp_ns), (min(fe.skp_gaps), max(fe.skp_gaps)) if fe.skp_gaps else None,
        fe.b2b, (min(fe.dll.ack_lat), max(fe.dll.ack_lat)) if fe.dll.ack_lat else None, fe.ltssm.ts_counts))
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


async def green_row(dut, name, **kw):
    k = Knobs(**kw)
    fe, chk, rise, kinds = await run_row(dut, k, name)
    assert_conforming(dut, fe, chk, rise, kinds, k.bar_size, k.bar_prefetchable)
    return fe, chk


def pinned_red(dut, row, state, detail):
    dut._log.info('PINNED_RED|%s|%s|%s' % (row, state, detail))


# ---------------------------------------------------------------- green rows
@cocotb.test()
async def vary_com_lane0(dut):
    """RULINGS_A 6's variation: COM in byte 0 ([7:0])."""
    fe, chk = await green_row(dut, 'vary_com_lane0', com_lane=0)
    assert fe.com_lanes[1] == 0 and fe.com_lanes[0] > 1000, 'COM lanes driven %s' % fe.com_lanes


@cocotb.test()
async def vary_ack_late(dut):
    """V7: the far end Acks 100 clocks after a TLP's END, near Table 3-6's 237 Symbol Times."""
    fe, chk = await green_row(dut, 'vary_ack_late', ack_delay_cycles=100)
    lat = fe.dll.ack_lat
    assert lat and min(lat) >= 100 and max(lat) <= 118, 'far-end Ack latency %s cycles' % (lat,)


@cocotb.test()
async def vary_back_to_back(dut):
    """V9: packets back to back, no idle between them (§4.2.2 p.196)."""
    fe, chk = await green_row(dut, 'vary_back_to_back', pkt_gap=0)
    assert fe.b2b >= 10, 'only %d packets followed another with no idle' % fe.b2b


@cocotb.test()
async def vary_skp_min(dut):
    """V10: SKP Ordered Sets every 1180 Symbol Times (§4.2.7.1 p.261)."""
    fe, chk = await green_row(dut, 'vary_skp_min', skp_interval=1180)
    g = fe.skp_gaps
    assert len(g) >= 10 and min(g) >= 1180 and max(g) <= 1180 + 16, 'far-end SKP gaps %s..%s' % (min(g), max(g))


@cocotb.test()
async def vary_skp_max(dut):
    """V10: SKP Ordered Sets every 1538 Symbol Times."""
    fe, chk = await green_row(dut, 'vary_skp_max', skp_interval=1538)
    g = fe.skp_gaps
    assert len(g) >= 10 and min(g) >= 1538 and max(g) <= 1538 + 16, 'far-end SKP gaps %s..%s' % (min(g), max(g))


@cocotb.test()
async def vary_skp_count(dut):
    """V11: SKP OS of 1-5 SKP at the RC's pins (§4.2.7.2 p.262), as an elastic buffer leaves them."""
    fe, chk = await green_row(dut, 'vary_skp_count', skp_counts=(1, 2, 3, 4, 5))
    assert fe.skp_ns == {1, 2, 3, 4, 5}, 'SKP counts sent %s' % fe.skp_ns
    assert fe.com_lanes[0] > 100 and fe.com_lanes[1] > 100, 'COM lanes %s' % fe.com_lanes


@cocotb.test()
async def vary_ts_extra(dut):
    """V12: more than the minimum training sets and idle (p.220, p.225, p.236, p.237)."""
    fe, chk = await green_row(dut, 'vary_ts_extra', poll_ts1_min=2048, ts2_after_rx=32, idle_after_rx=32,
                              cfg_lw_extra_ts1=8)
    counts = dict(fe.ltssm.ts_counts)
    assert counts.get('POLL_ACTIVE', 0) >= 2048, 'TS1 in Polling.Active %s' % counts


@cocotb.test()
async def vary_credit_max(dut):
    """V4(b): the largest finite advertisement, PH / NPH 7Fh, PD / NPD 7FFh (p.138)."""
    fe, chk = await green_row(dut, 'vary_credit_max',
                              adv_credits={'P': (0x7F, 0x7FF), 'NP': (0x7F, 0x7FF), 'CPL': (0, 0)})
    assert fe.chk.n.get('FC_OVERFLOW', 0) > 0


@cocotb.test()
async def vary_updatefc_extra(dut):
    """V5: UpdateFC on every release, and UpdateFC-Cpl 0/0 every 10 us (p.138, p.143)."""
    fe, chk = await green_row(dut, 'vary_updatefc_extra', updatefc_every_release=True,
                              updatefc_cpl_zero_period=10 * 125)
    assert fe.dll.stats['updatefc_tx'] >= 25, 'UpdateFCs sent %d' % fe.dll.stats['updatefc_tx']


@cocotb.test()
async def vary_redundant_ack(dut):
    """V8: an Ack naming ACKD_SEQ whenever idle 2 us, FFFh before any TLP (p.180)."""
    fe, chk = await green_row(dut, 'vary_redundant_ack', redundant_ack_period=2 * 125)
    st = fe.dll.stats
    assert st.get('redundant_ack', 0) >= 5 and st.get('redundant_ack_fff', 0) >= 1, 'redundant Acks %s' % st


@cocotb.test()
async def vary_nullified(dut):
    """V18: the first three Completions each sent nullified first (EDB, inverted LCRC, p.174)."""
    fe, chk = await green_row(dut, 'vary_nullified', nullify_count=3)
    assert fe.dll.stats.get('nullified', 0) == 3, 'nullified TLPs sent %s' % fe.dll.stats


@cocotb.test()
async def vary_crs(dut):
    """V19: CRS for 200 clocks after the first Configuration Request (p.110), inside the RC's 3 reissues."""
    fe, chk = await green_row(dut, 'vary_crs', crs_cycles=200)
    crs = sum(1 for _, _, st in fe.tl.requests if st == CFG.C.CPL_CRS)
    assert crs >= 1, 'no CRS Completion was sent'


@cocotb.test()
async def vary_bar(dut):
    """V22: BAR0 prefetchable and 128 B, the minimum (p.491; PCI 3.0 §6.2.5.1)."""
    await green_row(dut, 'vary_bar', bar_prefetchable=True, bar_size=128)
