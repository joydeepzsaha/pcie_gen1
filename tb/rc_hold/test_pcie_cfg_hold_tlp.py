"""test_pcie_cfg_hold_tlp -- the post-reset hold, the CRS window and the timeout reissue

Author: Kourosh Ghahramani
Silicon Systems Research Lab, University of Washington

Purpose
    Rows a-d and f of §63 #22, on tb_pcie_cfg_hold_tlp: pcie_enum_top in front
    of the real pcie_rq_rc_top, with this module playing the Data Link Layer
    and a far end whose answer to each Configuration Request depends on when
    the request left. Every time is counted in clk_i cycles from the rise of
    fc_initialized_i, the bench's DL_Active.

    a  no Configuration Request before the hold expires
    b  a device answering CRS that becomes ready inside the window is
       enumerated
    c  CRS for the whole window ends in ENUM_ERR_CRS_EXHAUSTED, after it
    d  the hold re-arms after a link drop
    f  inside the window a request that times out is reissued; after it, the
       timeout is reported as ENUM_ERR_TIMEOUT

    Each row is red until the commit that fixes it (§22.75), pinned at one
    assertion (§22.93): an exception before it logs PINNED_RED|<row>|
    NOT_REACHED and the row returns normally, which expect_fail reports as a
    FAIL.

Structure
    Bench values      read from the bench_* wires
    Bench             sampler, far end, link control
    Rows              a, b, c, d, f

References
    PCIe Base Spec r2.1, §2.3.1
    PCIe Base Spec r2.1, §2.3.2
    PCIe Base Spec r2.1, §2.8
    PCIe Base Spec r2.1, §6.6.1
    PCIe Base Spec r3.0, §6.7.3.3
"""

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ReadOnly, RisingEdge

from enum_tb_common import (
    BDF, CLK_NS, RID, SCAN_BUS, VENDOR, DEVICE, REG0, HDR_TYPE0, reg3,
    CFG_REG_VENDOR_DEVICE, CFG_REG_CACHE_HEADER,
    CPL_CRS, CPL_SC, CPL_UR,
    ENUM_ERR_CRS_EXHAUSTED, ENUM_ERR_TIMEOUT, err_name,
    TlpRequest, set_credits,
    cpl_dw0, cpl_dw1, cpl_dw2,
)

SPACE = {CFG_REG_VENDOR_DEVICE: REG0, CFG_REG_CACHE_HEADER: reg3(HDR_TYPE0)}


def pinned_red(dut, row, state, detail):
    """§22.93: the marker sweep43.sh copies into the gate's .diag."""
    dut._log.info("PINNED_RED|%s|%s|%s", row, state, detail)


# ---------------------------------------------------------------------------
# Bench
# ---------------------------------------------------------------------------
class Bench:
    """One sampler for every event, so all times share one cycle count.

    The sampler counts rising edges and reads in the ReadOnly phase after
    each: request TLPs leaving the Transaction Layer, the rise of
    fc_initialized_i, completion timeouts and the scan's first terminal
    state. The far end answers each request by policy(req, cycle), where
    cycle is the request's last beat: "sc" (from SPACE, UR elsewhere), "crs"
    or "silent".
    """

    def __init__(self, dut):
        self.dut = dut
        self.cycle = 0
        self.policy = lambda req, cycle: "sc"
        self.hold = int(dut.bench_cfg_hold_cycles.value)
        self.window = int(dut.bench_crs_window_cycles.value)
        self.backoff = int(dut.bench_crs_backoff_cycles.value)
        self.retry_max = int(dut.bench_crs_retry_max.value)
        self.cpl_timeout = int(dut.bench_cpl_timeout_cycles.value)
        self.clear()

    def clear(self):
        self.tlps = []           # (cycle, TlpRequest)
        self.answers = []        # (cycle, TlpRequest, "sc" | "crs" | "silent")
        self.rises = []          # cycles at which fc_initialized_i rose
        self.timeouts = []       # cycles of cpl_timeout_valid_o
        self.terminal = None     # (cycle, done, error, code)
        self._partial = []
        self._fc_prev = 0
        self._answered = 0

    @property
    def rise(self):
        return self.rises[-1]

    def start(self):
        cocotb.start_soon(self._sample())
        cocotb.start_soon(self._serve())

    async def _sample(self):
        d = self.dut
        while True:
            await RisingEdge(d.clk_i)
            self.cycle += 1
            await ReadOnly()
            if int(d.rst_i.value):
                self._partial = []
                self._fc_prev = 0
                continue
            fc = int(d.fc_initialized_i.value)
            if fc and not self._fc_prev:
                self.rises.append(self.cycle)
            self._fc_prev = fc
            if int(d.m_dllp_axis_tvalid.value) and int(d.m_dllp_axis_tready.value):
                self._partial.append(int(d.m_dllp_axis_tdata.value))
                if int(d.m_dllp_axis_tlast.value):
                    self.tlps.append((self.cycle, TlpRequest(self._partial)))
                    self._partial = []
            if int(d.cpl_timeout_valid_o.value):
                self.timeouts.append(self.cycle)
            if self.terminal is None and (int(d.scan_done_o.value) or
                                          int(d.scan_error_o.value)):
                self.terminal = (self.cycle, int(d.scan_done_o.value),
                                 int(d.scan_error_o.value),
                                 int(d.scan_error_code_o.value))

    async def _serve(self):
        while True:
            await RisingEdge(self.dut.clk_i)
            while self._answered < len(self.tlps):
                cycle, req = self.tlps[self._answered]
                self._answered += 1
                kind = self.policy(req, cycle)
                self.answers.append((cycle, req, kind))
                if kind == "silent":
                    continue
                if kind == "crs":
                    await self._complete(req, CPL_CRS)
                elif req.reg_num in SPACE:
                    await self._complete(req, CPL_SC, SPACE[req.reg_num])
                else:
                    await self._complete(req, CPL_UR)

    async def _complete(self, req, status, data=None):
        has_data = req.is_read and status == CPL_SC
        words = [cpl_dw0(has_data=has_data, length_dw=1 if has_data else 0),
                 cpl_dw1(BDF, status, byte_count=4),
                 cpl_dw2(RID, req.tag, lower_address=0)]
        if has_data:
            words.append(data)
        d = self.dut
        for index, word in enumerate(words):
            d.s_dllp_axis_tdata.value = word
            d.s_dllp_axis_tkeep.value = 0xF
            d.s_dllp_axis_tlast.value = 1 if index == len(words) - 1 else 0
            d.s_dllp_axis_tvalid.value = 1
            for _ in range(20000):
                await ReadOnly()
                fired = int(d.s_dllp_axis_tready.value) == 1
                await RisingEdge(d.clk_i)
                if fired:
                    break
            else:
                raise AssertionError("s_dllp_axis_tready never asserted")
        d.s_dllp_axis_tvalid.value = 0
        d.s_dllp_axis_tlast.value = 0

    # ---- link control -----------------------------------------------------
    async def link_up(self, start_scan=False):
        """Raise the link and DL_Active together, with one credit strobe.

        With start_scan, scan_start_i pulses in the same cycle, so the scan
        is waiting from the moment the reference rises.
        """
        d = self.dut
        await RisingEdge(d.clk_i)
        d.link_up_i.value = 1
        d.transmit_enable_i.value = 1
        set_credits(d)
        d.fc_initialized_i.value = 1
        d.fc_update_valid_i.value = 1
        if start_scan:
            d.scan_start_i.value = 1
        await RisingEdge(d.clk_i)
        d.scan_start_i.value = 0
        await RisingEdge(d.clk_i)
        d.fc_update_valid_i.value = 0

    async def link_down(self):
        d = self.dut
        await RisingEdge(d.clk_i)
        d.link_up_i.value = 0
        d.fc_initialized_i.value = 0

    async def wait_cycles(self, n):
        for _ in range(n):
            await RisingEdge(self.dut.clk_i)

    async def wait_until(self, cycle):
        while self.cycle < cycle:
            await RisingEdge(self.dut.clk_i)

    async def wait_terminal(self, by_cycle):
        """Wait for the scan's terminal state; raise if none by by_cycle."""
        while self.terminal is None:
            if self.cycle >= by_cycle:
                raise AssertionError(
                    f"no terminal state by cycle {by_cycle} "
                    f"(rise {self.rises}, {len(self.tlps)} requests)")
            await RisingEdge(self.dut.clk_i)

    def rel(self, cycle):
        return cycle - self.rise

    def describe(self):
        reqs = [(self.rel(c), f"reg{r.reg_num}", k) for c, r, k in self.answers]
        return (f"rises={self.rises} terminal={self.terminal} "
                f"timeouts={[self.rel(c) for c in self.timeouts]} "
                f"requests={reqs[:8]}{'...' if len(reqs) > 8 else ''} n={len(reqs)}")


async def make_bench(dut):
    """Reset with the link down; return a started Bench."""
    cocotb.start_soon(Clock(dut.clk_i, CLK_NS, units="ns").start())
    d = dut
    d.rst_i.value = 1
    d.link_up_i.value = 0
    d.transmit_enable_i.value = 0
    d.scan_start_i.value = 0
    d.scan_bus_i.value = SCAN_BUS
    d.s_dllp_axis_tdata.value = 0
    d.s_dllp_axis_tkeep.value = 0
    d.s_dllp_axis_tvalid.value = 0
    d.s_dllp_axis_tlast.value = 0
    d.s_dllp_axis_tuser.value = 0
    d.m_dllp_axis_tready.value = 1
    d.requester_id_i.value = RID
    d.completer_id_i.value = 0
    d.bus_number_i.value = 0
    d.device_number_i.value = 0
    d.function_number_i.value = 0
    d.memory_enable_i.value = 1
    d.extended_tag_enable_i.value = 0
    d.max_payload_bytes_i.value = 128
    d.max_read_bytes_i.value = 128
    d.rcb_128b_i.value = 0
    d.fc_initialized_i.value = 0
    d.fc_update_valid_i.value = 0
    set_credits(d, ph=0, pd=0, nph=0, npd=0, cplh=0, cpld=0)
    await RisingEdge(d.clk_i)
    bench = Bench(dut)
    bench.start()
    await bench.wait_cycles(4)
    d.rst_i.value = 0
    await bench.wait_cycles(20)
    return bench


def scan_snapshot(dut):
    return {"done": int(dut.scan_done_o.value),
            "error": int(dut.scan_error_o.value),
            "code": err_name(int(dut.scan_error_code_o.value)),
            "present": int(dut.device_present_o.value),
            "vendor": int(dut.vendor_id_o.value),
            "device": int(dut.device_id_o.value)}


# ---------------------------------------------------------------------------
# a -- no Configuration Request before the hold expires
# ---------------------------------------------------------------------------
@cocotb.test(expect_fail=True)   # §63 #22 -- flips in C2 (the hold)
async def a_hold_no_cfg_request_before_hold(dut):
    """Row a. RED BEFORE FIX.

    Base 2.1 §6.6.1 p.411: no Configuration Request until 100 ms after the
    end of reset, measured, where the RC cannot see that reset, from an event
    known to follow it; Base 3.0 §6.7.3.3 p.524 names DL_Active. The scan
    starts in the cycle fc_initialized_i rises and the far end answers
    everything.

    Today the probe leaves a few cycles after the rise and the scan
    completes. Pinned: the first request leaves no earlier than rise + hold.
    """
    row = "a_hold_no_cfg_request_before_hold"
    try:
        b = await make_bench(dut)
        await b.link_up(start_scan=True)
        await b.wait_terminal(b.rise + b.hold + 8000)
        st = scan_snapshot(dut)
        assert b.tlps, f"no request left at all: {b.describe()}"
        assert st["done"] == 1 and st["present"] == 1, f"scan: {st} {b.describe()}"
        first = b.rel(b.tlps[0][0])
        dut._log.info("DIAG a: first request at rise+%d, hold %d; %s",
                      first, b.hold, b.describe())
    except Exception as exc:   # noqa: BLE001 -- §22.93
        pinned_red(dut, row, "NOT_REACHED", repr(exc))
        return
    pinned_red(dut, row, "REACHED", f"first=rise+{first} hold={b.hold}")
    assert first >= b.hold, (
        f"the first Configuration Request left at rise+{first}, inside the "
        f"{b.hold}-cycle hold")


# ---------------------------------------------------------------------------
# b -- CRS, then ready, inside the window
# ---------------------------------------------------------------------------
@cocotb.test(expect_fail=True)   # §63 #22 -- flips in C3 (the CRS window)
async def b_crs_late_ready_inside_window_enumerates(dut):
    """Row b. RED BEFORE FIX.

    Base 2.1 §6.6.1 p.411: the RC must allow 1.0 s after reset before it
    judges a device that does not return a Successful Completion broken, and
    §2.3.1 p.110 lets a device answer CRS during that time. The far end
    answers CRS to every request that leaves before rise + hold + 6,000 and
    a Successful Completion after.

    Today the budget of CRS_RETRY_MAX reissues runs out long before that and
    the scan ends ENUM_ERR_CRS_EXHAUSTED. Pinned: the scan is done and found
    the device.
    """
    row = "b_crs_late_ready_inside_window_enumerates"
    try:
        b = await make_bench(dut)
        b.policy = lambda req, cycle: (
            "crs" if cycle < b.rise + b.hold + 6000 else "sc")
        await b.link_up(start_scan=True)
        await b.wait_terminal(b.rise + b.window + 8000)
        st = scan_snapshot(dut)
        crs = sum(1 for _, _, k in b.answers if k == "crs")
        assert crs >= 1, f"the far end answered no CRS: {b.describe()}"
        dut._log.info("DIAG b: scan %s; %d CRS answered; %s", st, crs, b.describe())
    except Exception as exc:   # noqa: BLE001 -- §22.93
        pinned_red(dut, row, "NOT_REACHED", repr(exc))
        return
    pinned_red(dut, row, "REACHED", f"done={st['done']} present={st['present']} "
               f"code={st['code']} crs={crs}")
    assert st["done"] == 1 and st["present"] == 1, (
        f"a device that answered CRS until rise+{b.hold + 6000} was not "
        f"enumerated: {st}")


# ---------------------------------------------------------------------------
# c -- CRS for the whole window
# ---------------------------------------------------------------------------
@cocotb.test(expect_fail=True)   # §63 #22 -- flips in C3 (the CRS window)
async def c_crs_whole_window_reports_crs_exhausted(dut):
    """Row c. RED BEFORE FIX.

    A device that answers CRS for the whole window is reported with the
    existing ENUM_ERR_CRS_EXHAUSTED (§2.3.2 p.121 lets the RC limit the
    loops), but no earlier than rise + window, so the judgement falls at
    least 1.0 s after DL_Active on the board.

    Today the report comes after CRS_RETRY_MAX reissues, about a thousand
    cycles after the rise. Pinned: the report is at or after rise + window.
    """
    row = "c_crs_whole_window_reports_crs_exhausted"
    try:
        b = await make_bench(dut)
        b.policy = lambda req, cycle: "crs"
        await b.link_up(start_scan=True)
        await b.wait_terminal(b.rise + b.window + 8000)
        st = scan_snapshot(dut)
        assert st["error"] == 1 and st["code"] == err_name(ENUM_ERR_CRS_EXHAUSTED), \
            f"scan: {st} {b.describe()}"
        report = b.rel(b.terminal[0])
        dut._log.info("DIAG c: report at rise+%d, window %d; %s",
                      report, b.window, b.describe())
    except Exception as exc:   # noqa: BLE001 -- §22.93
        pinned_red(dut, row, "NOT_REACHED", repr(exc))
        return
    pinned_red(dut, row, "REACHED", f"report=rise+{report} window={b.window}")
    assert report >= b.window, (
        f"ENUM_ERR_CRS_EXHAUSTED at rise+{report}, before the {b.window}-cycle "
        f"window closed")


# ---------------------------------------------------------------------------
# d -- the hold re-arms after a link drop
# ---------------------------------------------------------------------------
@cocotb.test(expect_fail=True)   # §63 #22 -- flips in C2 (the hold)
async def d_hold_rearms_after_link_drop(dut):
    """Row d. RED BEFORE FIX.

    The scan starts at the first rise. At rise + hold/2 the link and DL_Active
    drop for 500 cycles and come back with a new credit strobe. Base 2.1
    §6.6.1 p.410 counts DL_Down as a hot reset, so the hold starts again from
    the second rise.

    Today the scan has finished before the drop. Pinned: the first request
    leaves no earlier than the second rise + hold.
    """
    row = "d_hold_rearms_after_link_drop"
    try:
        b = await make_bench(dut)
        await b.link_up(start_scan=True)
        rise1 = b.rise
        await b.wait_until(rise1 + b.hold // 2)
        await b.link_down()
        await b.wait_cycles(500)
        await b.link_up()
        rise2 = b.rise
        assert rise2 > rise1, f"no second rise: {b.rises}"
        await b.wait_terminal(rise2 + b.hold + 8000)
        assert b.tlps, f"no request left at all: {b.describe()}"
        first = b.tlps[0][0] - rise2
        dut._log.info("DIAG d: rise1 %d rise2 %d, first request at rise2%+d; %s",
                      rise1, rise2, first, b.describe())
    except Exception as exc:   # noqa: BLE001 -- §22.93
        pinned_red(dut, row, "NOT_REACHED", repr(exc))
        return
    pinned_red(dut, row, "REACHED", f"first=rise2{first:+d} hold={b.hold}")
    assert first >= b.hold, (
        f"the first Configuration Request left at rise2{first:+d}; the hold "
        f"did not restart at the second link-up")


# ---------------------------------------------------------------------------
# f -- a timeout inside the window is reissued
# ---------------------------------------------------------------------------
@cocotb.test(expect_fail=True)   # §63 #22 -- flips in C4 (the timeout reissue)
async def f_timeout_inside_window_reissued(dut):
    """Row f. RED BEFORE FIX.

    The 1.0 s of Base 2.1 §6.6.1 p.411 covers a device that "fails to return
    a Successful Completion", which includes one that returns nothing. The
    far end ignores every request that leaves before rise + hold + 3,000 and
    answers the rest; tlp_request_tracker times the first one out (§2.8).

    Today the timeout ends the scan with ENUM_ERR_TIMEOUT. Pinned: the scan
    is done and found the device.
    """
    row = "f_timeout_inside_window_reissued"
    try:
        b = await make_bench(dut)
        b.policy = lambda req, cycle: (
            "silent" if cycle < b.rise + b.hold + 3000 else "sc")
        await b.link_up(start_scan=True)
        await b.wait_terminal(b.rise + b.window + 2 * b.cpl_timeout)
        st = scan_snapshot(dut)
        assert b.timeouts, f"no completion timeout: {b.describe()}"
        dut._log.info("DIAG f: scan %s; %s", st, b.describe())
    except Exception as exc:   # noqa: BLE001 -- §22.93
        pinned_red(dut, row, "NOT_REACHED", repr(exc))
        return
    pinned_red(dut, row, "REACHED", f"done={st['done']} present={st['present']} "
               f"code={st['code']} timeouts={len(b.timeouts)}")
    assert st["done"] == 1 and st["present"] == 1, (
        f"a device silent until rise+{b.hold + 3000} was not enumerated: {st}")
