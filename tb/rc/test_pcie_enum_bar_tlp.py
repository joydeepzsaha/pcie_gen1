"""Commit 2b-3 integration -- the WHOLE enumeration behind a REAL pcie_rq_rc_top.

    scan_start_i -> pcie_enum_scan -> [handoff] -> pcie_enum_bar
                 -> pcie_cfg_txn -> pcie_rq_if -> tlp_layer
                 -> TX DLLP -> [completer] -> RX DLLP -> ... -> enum_done_o

⭐ E1 IS THE HEADLINE OF STAGE C: an NVMe-like endpoint -- a 64-bit prefetchable
BAR0/1 pair, one probe answered CRS before SC -- enumerated end to end from flow
control initialisation to enum_done_o, with every one of the seventeen emitted
TLPs asserted on the wire against docs/predictions/SPEC_PREDICTIONS_ENUM.md SSE.8, PAYLOAD DWORDS
INCLUDED, and every captured value against goldens pinned before the RTL existed.
It runs twice: saturated credit, and under the Table 2-37 minimum drip.

!! WHY THE PAYLOAD IS NOT OPTIONAL. SSE.8's B1 and B5 have BYTE-IDENTICAL
descriptors -- both CfgWr0 to register 4 with first_be=1111. The all-ones sizing
write and the assignment write are indistinguishable on the header alone, so a
test asserting only descriptors would pass against an FSM that emitted the sizing
write twice and never assigned anything. SSE.9 EF3, and the most likely way to
build a vacuously-passing BAR bench.

!! ZERO MEANS INFINITE at FC init (Base 2.1 SS2.6.1 p.138, fn 33 p.137), so
starving a pool takes a small FINITE advertisement that is never replenished --
and a replenishing drip must advertise a CUMULATIVE INCREASING total, because
fc_*_i is the raw CREDITS_ALLOCATED off the wire (SS2.6.1.2 p.141).

Spec cited (read, not assumed):
  Configuration Request header ...... PCIe Base 2.1 SS2.2.7 p.79-80, Figure 2-18
  Minimum FC advertisements ......... PCIe Base 2.1 Table 2-37 p.137-138
  CRS handling / re-issue ........... PCIe Base 2.1 SS2.3.2 p.121
  Completion timeout is an error .... PCIe Base 2.1 SS2.8 p.152
  128-byte minimum BAR .............. PCIe Base 2.1 SS7.5.2.1 p.491-492
  Type 0 header offsets ............. PCIe Base 2.1 Figure 7-5 p.491
  BAR layout / sizing / Command ..... [PCI3] SS6.2.5.1 p.225-226, SS6.2.2 p.218
RTL cited:
  timer runs from ALLOCATION ........ src/tlp/tlp_request_tracker.sv:39
  credit gate, downstream of it ..... src/tlp/tlp_layer.sv:280
  orphan-data report, once per Dword  src/rc/pcie_rc_if.sv:403-405
Full derivation: docs/predictions/SPEC_PREDICTIONS_ENUM.md SSE.
"""

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ReadOnly, RisingEdge

from enum_tb_common import (
    BAR_MEM32, BAR_MEM64, BAR_SLOTS, BDF, BUS, CLK_NS, CPL_TIMEOUT_CYCLES,
    DEV, FN, RID, DEVICE, HDR_TYPE0, MEM_BAR_BASE, REG0, SCAN_BUS, VENDOR,
    CMD_ENABLE_VALUE,
    ENUM_ERR_CRS_EXHAUSTED, ENUM_ERR_NONE, ENUM_ERR_TIMEOUT, ENUM_ERR_CREDIT_STARVED,
    ENUM_ERR_UR_POST_PROBE,
    CFG_BE_DWORD, CFG_BE_LOWER_HALF,
    CFG_REG_BAR0, CFG_REG_BAR1, CFG_REG_BAR2, CFG_REG_BAR3, CFG_REG_BAR4,
    CFG_REG_BAR5, CFG_REG_CACHE_HEADER, CFG_REG_COMMAND_STATUS,
    CFG_REG_EXPANSION_ROM, CFG_REG_VENDOR_DEVICE,
    CPL_CRS, CPL_SC, CPL_UR,
    RC_ERR_ORPHAN_DATA, TLP_ERR_UNEXPECTED_COMPLETION,
    BarSpec, ConfigDevice, CreditDrip, Mon, TlpRequest,
    assert_cfg_tlp_on_wire, assert_sequence, err_name, expect_count, nonempty,
    cfg_wire_dw0, cfg_wire_dw1, cfg_wire_dw2,
    cpl_dw0, cpl_dw1, cpl_dw2, reg3, set_credits,
)


KB = 1024
MASK64 = (1 << 64) - 1

# ==========================================================================
# SS THE ACCEPTANCE DEVICE -- SSE.3.4 / SSE.7.4, pinned before the RTL existed
#
# NVMe-like: one 16 KB 64-bit PREFETCHABLE pair at BAR0/1, BAR2-5 unimplemented.
# Base 2.1 makes this the EXPECTED shape rather than a corner case -- prefetchable
# BARs must support 64-bit addressing (SS7.5.2.1 p.491-492) and a compliant memory
# BAR should be prefetchable.
# ==========================================================================
ACCEPT_BAR_SIZE = 16 * KB
ACCEPT_BAR_ADDR = MEM_BAR_BASE                 # 0x0000_0000_8000_0000


def acceptance_device():
    return ConfigDevice(
        bars={CFG_REG_BAR0: BarSpec(BAR_MEM64, ACCEPT_BAR_SIZE, prefetch=True)})


# ⭐ THE SSE.8 SEQUENCE, HAND-TRANSCRIBED FROM THE TABLE.
#
# Seventeen transactions: the two presence-scan reads, then B1..B15. Each entry
# is (write, reg_num, first_be, payload) -- the payload is a FIELD, not an
# afterthought, for the reason in the module docstring.
#
# Deliberately NOT generated from a model of the FSM: a generator would agree
# with an FSM bug by construction. These are the numbers in the document.
GOLDEN_SEQUENCE = [
    # presence scan -- SSD.4
    (False, CFG_REG_VENDOR_DEVICE, 0b1111, None),
    (False, CFG_REG_CACHE_HEADER,  0b1111, None),
    # SSE.8 B1..B6 -- the pair
    (True,  CFG_REG_BAR0, 0b1111, 0xFFFFFFFF),     # B1  sizing, low
    (False, CFG_REG_BAR0, 0b1111, None),           # B2  -> 0xFFFFC00C
    (True,  CFG_REG_BAR1, 0b1111, 0xFFFFFFFF),     # B3  sizing, high
    (False, CFG_REG_BAR1, 0b1111, None),           # B4  -> 0xFFFFFFFF
    (True,  CFG_REG_BAR0, 0b1111, 0x80000000),     # B5  assign, low
    (True,  CFG_REG_BAR1, 0b1111, 0x00000000),     # B6  assign, high
    # SSE.8 B7..B14 -- the four unimplemented candidates
    (True,  CFG_REG_BAR2, 0b1111, 0xFFFFFFFF),     # B7
    (False, CFG_REG_BAR2, 0b1111, None),           # B8
    (True,  CFG_REG_BAR3, 0b1111, 0xFFFFFFFF),     # B9
    (False, CFG_REG_BAR3, 0b1111, None),           # B10
    (True,  CFG_REG_BAR4, 0b1111, 0xFFFFFFFF),     # B11
    (False, CFG_REG_BAR4, 0b1111, None),           # B12
    (True,  CFG_REG_BAR5, 0b1111, 0xFFFFFFFF),     # B13
    (False, CFG_REG_BAR5, 0b1111, None),           # B14
    # SSE.8 B15 -- the enable, structurally last
    (True,  CFG_REG_COMMAND_STATUS, 0b0011, CMD_ENABLE_VALUE),
]

# The readbacks SSE.3.4 predicts for the two sizing probes.
GOLDEN_BAR0_READBACK = 0xFFFFC00C
GOLDEN_BAR1_READBACK = 0xFFFFFFFF


def on_wire(req):
    """One observed TLP as the golden tuple shape, payload included."""
    return (not req.is_read, req.reg_num, req.first_be,
            req.payload[0] if req.payload else None)


def render(item):
    write, reg, fbe, payload = item
    kind = "CfgWr0" if write else "CfgRd0"
    data = "-" if payload is None else f"{payload:#010x}"
    return f"{kind}(reg={reg:#04x}, fbe={fbe:#06b}, data={data})"


# ==========================================================================
# SS THE COMPLETER
#
# ⭐ THE FOUR-NAME INTERFACE IS PRESERVED VERBATIM -- .start(), .seen,
# .wait_for(n), .complete(req, ...) -- because that is the surface Joy's
# protocol-checking endpoint model is meant to drop into.  serve() remains an
# ADDITIONAL convenience and replaces none of the four.
#
# What is new relative to 2b-2's ConfigSpaceCompleter is that this one WRITES:
# it drives a full ConfigDevice with real Base Address register write-mask
# semantics.  A completer that echoed BAR writes verbatim would make sizing
# return garbage and the DUT look broken -- see ConfigDevice in enum_tb_common,
# and assert_mask_exercised, which every test here calls.
#
# ⛔ THE DEFAULT ARM IS UR, NOT SILENCE.  Base 2.1 SS7.3.3 p.480: a Type 0 request
# that does not address "a valid local Configuration Space of an implemented
# Function" must "follow rules for handling Unsupported Requests".  A completer
# that quietly ignored an unmodelled register would drive the sequencer into a
# completion timeout and look exactly like an FSM bug.
# ==========================================================================
class BarSpaceCompleter:
    def __init__(self, dut, device=None, crs_once=(), silent_regs=(), ur_regs=()):
        self.dut = dut
        self.dev = device if device is not None else ConfigDevice()
        self.crs_once = set(crs_once)        # answer CRS the first time, then SC
        self.silent_regs = set(silent_regs)  # answer nothing at all
        # ⭐ EXPLICIT UR INJECTION, rather than deleting a register from the
        # model and relying on the default arm. Deleting does not work: the
        # all-ones SIZING WRITE arrives first and ConfigDevice.write() creates
        # the entry, so the later read finds a value and never reaches the UR
        # arm. Measured -- e6 failed "the UR arm never fired" until this existed.
        self.ur_regs = set(ur_regs)
        self.ur_injected_hits = 0
        self.seen = []
        self.ur_default_hits = 0
        self.crs_hits = 0
        self.silent_hits = 0
        self._partial = []
        self._answered = 0

    # ---- the four names ----------------------------------------------------
    def start(self):
        cocotb.start_soon(self._watch_tx())

    async def wait_for(self, count, cycles=40000):
        for _ in range(cycles):
            await RisingEdge(self.dut.clk_i)
            if len(self.seen) >= count:
                return
        raise AssertionError(
            f"expected {count} request TLPs on the wire, saw {len(self.seen)} "
            f"({self.seen}) -- FC credits, or the enumeration never issued?")

    async def complete(self, req, status=CPL_SC, data=None, byte_count=None):
        has_data = req.is_read and status == CPL_SC
        if byte_count is None:
            byte_count = 4
        words = [
            cpl_dw0(has_data=has_data, length_dw=1 if has_data else 0),
            cpl_dw1(BDF, status, byte_count=byte_count),
            cpl_dw2(RID, req.tag, lower_address=0),
        ]
        if has_data:
            words.append(0xD0000000 | req.tag if data is None else data)
        await self.inject(words)

    # ---- the convenience ---------------------------------------------------
    def serve(self):
        cocotb.start_soon(self._serve())

    async def _watch_tx(self):
        d = self.dut
        while True:
            await RisingEdge(d.clk_i)
            await ReadOnly()
            if int(d.rst_i.value):
                continue
            if int(d.m_dllp_axis_tvalid.value) and int(d.m_dllp_axis_tready.value):
                self._partial.append(int(d.m_dllp_axis_tdata.value))
                if int(d.m_dllp_axis_tlast.value):
                    self.seen.append(TlpRequest(self._partial))
                    self._partial = []

    async def _serve(self):
        while True:
            await RisingEdge(self.dut.clk_i)
            while self._answered < len(self.seen):
                req = self.seen[self._answered]
                self._answered += 1
                await self._answer(req)

    async def _answer(self, req):
        reg = req.reg_num
        if reg in self.silent_regs:
            self.silent_hits += 1
            return                                   # deliberate silence
        if reg in self.ur_regs:
            self.ur_injected_hits += 1
            await self.complete(req, status=CPL_UR)
            return
        if reg in self.crs_once:
            self.crs_once.discard(reg)
            self.crs_hits += 1
            await self.complete(req, status=CPL_CRS)
            return
        if not req.is_read:
            self.dev.write(reg, req.payload[0] if req.payload else 0,
                           req.first_be)
            await self.complete(req, status=CPL_SC)
            return
        value = self.dev.read(reg)
        if value is None:
            self.ur_default_hits += 1                # SS7.3.3 p.480
            await self.complete(req, status=CPL_UR)
        else:
            await self.complete(req, status=CPL_SC, data=value)

    async def inject(self, words):
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
                raise AssertionError("s_dllp_axis_tready never asserted -- RX wedged")
        d.s_dllp_axis_tvalid.value = 0
        d.s_dllp_axis_tlast.value = 0


# ==========================================================================
# Harness
# ==========================================================================
async def init(dut, credits=None, device=None, crs_once=(), silent_regs=(),
               ur_regs=(), serve=True, bar_enable=1):
    cocotb.start_soon(Clock(dut.clk_i, CLK_NS, units="ns").start())
    dut.rst_i.value = 1
    dut.link_up_i.value = 0
    dut.transmit_enable_i.value = 0
    dut.scan_start_i.value = 0
    dut.scan_bus_i.value = SCAN_BUS
    dut.bar_enable_i.value = bar_enable
    dut.s_dllp_axis_tdata.value = 0
    dut.s_dllp_axis_tkeep.value = 0
    dut.s_dllp_axis_tvalid.value = 0
    dut.s_dllp_axis_tlast.value = 0
    dut.s_dllp_axis_tuser.value = 0
    dut.m_dllp_axis_tready.value = 1
    dut.requester_id_i.value = RID
    dut.completer_id_i.value = 0
    dut.bus_number_i.value = 0
    dut.device_number_i.value = 0
    dut.function_number_i.value = 0
    dut.memory_enable_i.value = 1
    dut.extended_tag_enable_i.value = 0
    dut.max_payload_bytes_i.value = 128
    dut.max_read_bytes_i.value = 128
    dut.rcb_128b_i.value = 0
    dut.fc_initialized_i.value = 0
    dut.fc_update_valid_i.value = 0
    set_credits(dut, ph=0, pd=0, nph=0, npd=0, cplh=0, cpld=0)
    for _ in range(4):
        await RisingEdge(dut.clk_i)
    dut.rst_i.value = 0
    dut.link_up_i.value = 1
    dut.transmit_enable_i.value = 1

    set_credits(dut, **(credits or {}))
    dut.fc_initialized_i.value = 1
    dut.fc_update_valid_i.value = 1        # THE INIT STROBE
    await RisingEdge(dut.clk_i)
    await RisingEdge(dut.clk_i)
    if credits is not None:
        dut.fc_update_valid_i.value = 0    # a drip test pulses its own totals
    for _ in range(4):
        await RisingEdge(dut.clk_i)

    mon = Mon(dut)
    mon.start()
    completer = BarSpaceCompleter(dut, device=device, crs_once=crs_once,
                                  silent_regs=silent_regs, ur_regs=ur_regs)
    completer.start()
    if serve:
        completer.serve()
    await RisingEdge(dut.clk_i)
    return mon, completer


async def start_enum(dut):
    dut.scan_start_i.value = 1
    await RisingEdge(dut.clk_i)
    dut.scan_start_i.value = 0


async def settle(dut, cycles=40):
    """LOCAL BY DESIGN. Not an early-exit loop: the default IS sim time."""
    for _ in range(cycles):
        await RisingEdge(dut.clk_i)


async def wait_enum(dut, cycles=60000):
    for _ in range(cycles):
        await ReadOnly()
        reached = int(dut.enum_done_o.value) or int(dut.enum_error_o.value)
        await RisingEdge(dut.clk_i)
        if reached:
            return
    raise AssertionError("enumeration never reached a terminal state")


async def status(dut):
    await ReadOnly()
    sizes = int(dut.bar_size_o.value)
    addrs = int(dut.bar_addr_o.value)
    snap = {
        "scan_done": int(dut.scan_done_o.value),
        "present": int(dut.device_present_o.value),
        "vendor": int(dut.vendor_id_o.value),
        "device": int(dut.device_id_o.value),
        "busy": int(dut.bar_busy_o.value),
        "done": int(dut.enum_done_o.value),
        "error": int(dut.enum_error_o.value),
        "code": int(dut.enum_error_code_o.value),
        "blocked": int(dut.err_credit_blocked_o.value),
        "count": int(dut.bar_count_o.value),
        "valid": int(dut.bar_valid_o.value),
        "is64": int(dut.bar_is_64_o.value),
        "prefetch": int(dut.bar_prefetch_o.value),
        "io_mask": int(dut.io_bar_mask_o.value),
        "size": [(sizes >> (64 * i)) & MASK64 for i in range(BAR_SLOTS)],
        "addr": [(addrs >> (64 * i)) & MASK64 for i in range(BAR_SLOTS)],
    }
    await RisingEdge(dut.clk_i)
    return snap


def assert_acceptance_outcome(snap, what=""):
    """Every captured value, against the SSE.7.4 goldens."""
    assert snap["done"] == 1 and snap["error"] == 0, \
        f"{what}enumeration ended {err_name(snap['code'])}: {snap}"
    assert snap["scan_done"] == 1 and snap["present"] == 1, \
        f"{what}the presence phase did not find the device: {snap}"
    assert snap["vendor"] == VENDOR and snap["device"] == DEVICE, \
        f"{what}identity {snap['vendor']:#06x}:{snap['device']:#06x} wrong"
    assert snap["count"] == 1, (
        f"{what}bar_count_o {snap['count']} != 1 -- a 64-bit pair occupies two "
        "registers but is ONE BAR (SSE.7.4)")
    assert snap["valid"] == 0b000001, f"{what}bar_valid_o {snap['valid']:#08b}"
    assert snap["is64"] == 0b000001, f"{what}bar_is_64_o {snap['is64']:#08b}"
    assert snap["prefetch"] == 0b000001, f"{what}bar_prefetch_o {snap['prefetch']:#08b}"
    assert snap["io_mask"] == 0, f"{what}io_bar_mask_o {snap['io_mask']:#08b} != 0"
    assert snap["size"][0] == ACCEPT_BAR_SIZE, \
        f"{what}bar_size_o[0] {snap['size'][0]:#x} != {ACCEPT_BAR_SIZE:#x}"
    assert snap["addr"][0] == ACCEPT_BAR_ADDR, \
        f"{what}bar_addr_o[0] {snap['addr'][0]:#x} != {ACCEPT_BAR_ADDR:#x}"
    assert snap["blocked"] == 0, f"{what}err_credit_blocked_o set on a clean run"


def assert_wire_sequence(completer, what=""):
    """⭐ Every emitted TLP against SSE.8, payload Dwords included.

    Two layers, deliberately: the whole-sequence compare catches an extra,
    missing or reordered transaction, and assert_cfg_tlp_on_wire re-derives each
    header Dword from Base 2.1 SS2.2.7 p.79-80 and Figure 2-18 p.80. The first
    would pass against a DUT that got every header field wrong in the same way;
    the second would pass against a DUT that emitted the right TLPs in the wrong
    order.
    """
    seen = nonempty(completer.seen, f"{what}no TLP reached the wire at all")
    assert_sequence([on_wire(r) for r in seen], GOLDEN_SEQUENCE,
                    f"{what}SSE.8 on-wire sequence", render=render)
    for index, req in enumerate(seen):
        write, reg, first_be, _payload = GOLDEN_SEQUENCE[index]
        assert_cfg_tlp_on_wire(req, write=write, reg_num=reg, first_be=first_be,
                               tag=req.tag,
                               what=f"{what}TLP {index} ({req!r}): ")


def assert_rom_untouched(completer, what=""):
    """P-NO-ROM, SSE.7.3 -- on the wire, over the whole run."""
    seen = nonempty(completer.seen, f"{what}P-NO-ROM over an empty set")
    stray = [r for r in seen if r.reg_num == CFG_REG_EXPANSION_ROM]
    assert not stray, (
        f"{what}register {CFG_REG_EXPANSION_ROM} (offset 30h, the Expansion ROM "
        f"Base Address register) was accessed: {stray}")


def assert_command_last(completer, what=""):
    """P-CMD-LAST, SSE.5.1 -- on the wire, not from a state variable."""
    seen = nonempty(completer.seen, f"{what}P-CMD-LAST over an empty set")
    hits = [i for i, r in enumerate(seen)
            if (not r.is_read) and r.reg_num == CFG_REG_COMMAND_STATUS]
    assert len(hits) == 1, \
        f"{what}P-CMD-LAST: {len(hits)} writes to register 1, expected 1: {seen}"
    assert hits[0] == len(seen) - 1, (
        f"{what}P-CMD-LAST: the Command write is at index {hits[0]} of "
        f"{len(seen)}; {len(seen) - 1 - hits[0]} request(s) followed it")


# ==========================================================================
# E1 -- ⭐ THE ACCEPTANCE TEST, saturated credit
# ==========================================================================
@cocotb.test()
async def e1_nvme_endpoint_enumerated_end_to_end(dut):
    """An NVMe-like endpoint, FC init to enum_done_o, every DW on the wire.

    One probe is answered CRS before SC -- the Header Type read -- because a real
    endpoint that has just come out of reset does exactly that, and Base 2.1
    SS2.3.2 p.121 requires the Root Complex to re-issue rather than fail.

    The CRS makes the run harder in a way worth stating: the retry is a NEW
    request with a NEW tag, so a bench that leaned on tag values anywhere would
    break here. None does -- I3's rule.
    """
    dev = acceptance_device()
    mon, completer = await init(dut, device=dev,
                                crs_once=(CFG_REG_CACHE_HEADER,))
    await start_enum(dut)
    await wait_enum(dut)
    snap = await status(dut)

    assert_acceptance_outcome(snap)
    assert completer.crs_hits == 1, \
        f"the CRS arm fired {completer.crs_hits}x, expected exactly once"

    # ⭐ Every emitted DW against SSE.8. The CRS retry means the wire carries
    # eighteen TLPs for seventeen logical transactions, so the retry is dropped
    # before comparing -- and its presence is asserted rather than assumed.
    seen = nonempty(completer.seen, "nothing reached the wire")
    hdr_reads = [r for r in seen if r.is_read and r.reg_num == CFG_REG_CACHE_HEADER]
    expect_count(hdr_reads, 2, "Header Type reads (one CRS'd, one answered)")
    del completer.seen[completer.seen.index(hdr_reads[0])]

    assert_wire_sequence(completer, "E1: ")
    assert_command_last(completer, "E1: ")
    assert_rom_untouched(completer, "E1: ")

    # The two sizing readbacks the device actually returned -- SSE.3.4.
    fresh = acceptance_device()
    fresh.write(CFG_REG_BAR0, 0xFFFFFFFF)
    fresh.write(CFG_REG_BAR1, 0xFFFFFFFF)
    assert fresh.read(CFG_REG_BAR0) == GOLDEN_BAR0_READBACK, \
        f"BAR0 sizing readback {fresh.read(CFG_REG_BAR0):#010x} != SSE.3.4 golden"
    assert fresh.read(CFG_REG_BAR1) == GOLDEN_BAR1_READBACK, \
        f"BAR1 sizing readback {fresh.read(CFG_REG_BAR1):#010x} != SSE.3.4 golden"

    # The device really was programmed and really was enabled.
    dev.assert_mask_exercised("E1: ")
    assert dev.bar_written(CFG_REG_BAR0) == 0x80000000, \
        f"BAR0 holds {dev.bar_written(CFG_REG_BAR0):#010x}"
    assert dev.bar_written(CFG_REG_BAR1) == 0x00000000, \
        f"BAR1 holds {dev.bar_written(CFG_REG_BAR1):#010x}"
    assert dev.command == CMD_ENABLE_VALUE, (
        f"Command register {dev.command:#06x} != {CMD_ENABLE_VALUE:#06x} "
        "(Memory Space Enable | Bus Master Enable, [PCI3] Table 6-1 p.218)")
    assert dev.command & 0x1 == 0, \
        "I/O Space Enable is set, but no I/O BAR was assigned (SSE.6)"

    mon.clean()
    assert completer.ur_default_hits == 0, \
        "the completer answered UR to a register the acceptance device models"


# ==========================================================================
# E2 -- ⭐ THE SAME DEVICE, under the Table 2-37 minimum credit drip
# ==========================================================================
@cocotb.test()
async def e2_nvme_endpoint_under_the_minimum_credit_drip(dut):
    """The acceptance run again with NPH=1, NPD=1 and a cumulative drip.

    Table 2-37 p.137-138 is the spec MINIMUM a receiver may advertise, so this is
    derived rather than chosen: NPH=1, NPD=1, and CPLH/CPLD advertised as all-zero
    because INFINITE COMPLETION CREDIT IS MANDATORY FOR AN ENDPOINT.

    !! THE DRIP MUST ADVERTISE A CUMULATIVE INCREASING TOTAL. fc_*_i is the raw
    CREDITS_ALLOCATED off the wire (SS2.6.1.2 p.141) with no arithmetic anywhere on
    the path, so a drip that re-pulses a constant is saying "I have still only
    ever allocated N" and blocks the transmitter forever once N are consumed --
    a deadlock indistinguishable from the DUT bug it would be hiding.

    Seventeen serialized transactions on one non-posted header credit is the real
    subject: every one of them has to wait for a fresh UpdateFC.
    """
    dev = acceptance_device()
    mon, completer = await init(
        dut, device=dev,
        credits={"ph": 0, "pd": 0, "nph": 1, "npd": 1, "cplh": 0, "cpld": 0})
    drip = CreditDrip(dut, nph=1, npd=1, period=40, step=1)
    drip.start()

    await start_enum(dut)
    await wait_enum(dut)
    snap = await status(dut)

    assert_acceptance_outcome(snap, "E2 (credit drip): ")
    assert_wire_sequence(completer, "E2: ")
    assert_command_last(completer, "E2: ")
    assert_rom_untouched(completer, "E2: ")
    dev.assert_mask_exercised("E2: ")
    assert dev.command == CMD_ENABLE_VALUE, \
        f"Command register {dev.command:#06x} under the credit drip"

    # The drip was actually load-bearing: the transmitter really did block, and
    # the drip really did have to replenish it. Without both, this test is E1.
    assert mon.blocked_seen, (
        "tx_fc_blocked_o never asserted, so credit never actually ran out and "
        "this test did not exercise the drip at all -- it is a duplicate of E1")
    assert drip.updates >= len(GOLDEN_SEQUENCE) - 1, (
        f"the drip issued {drip.updates} UpdateFCs for {len(GOLDEN_SEQUENCE)} "
        "transactions; with NPH=1 each transaction needs its own")
    mon.clean()


# ==========================================================================
# E3 -- CRS in the BAR phase specifically
# ==========================================================================
@cocotb.test()
async def e3_crs_mid_bar_phase_retries_then_succeeds(dut):
    """A CRS to a BAR sizing write is re-issued and the enumeration completes.

    E1's CRS lands in the presence phase. This one lands on register 4 -- the
    first BAR transaction -- so the retry happens with the BAR sequencer owning
    the command port, which is a different owner of a different FSM.
    """
    dev = acceptance_device()
    mon, completer = await init(dut, device=dev, crs_once=(CFG_REG_BAR0,))
    await start_enum(dut)
    await wait_enum(dut)
    snap = await status(dut)

    assert_acceptance_outcome(snap, "E3: ")
    assert completer.crs_hits == 1, f"CRS arm fired {completer.crs_hits}x"

    # The retry is a NEW request (SS2.3.2 p.121), so register 4 sees one extra.
    reg4 = [r for r in completer.seen if r.reg_num == CFG_REG_BAR0]
    expect_count(reg4, 4, "register 4 TLPs (sizing write x2 after CRS, read, assign)")
    assert_command_last(completer, "E3: ")
    dev.assert_mask_exercised("E3: ")
    mon.clean()


# ==========================================================================
# E4 -- ⭐ credit starvation mid BAR phase: the Finding-2 signature, 3rd sighting
# ==========================================================================
@cocotb.test()
async def e4_credit_starvation_mid_bar_phase(dut):
    """A finite NP advertisement, never replenished, times out mid-BAR-phase.

    ⭐ THE FINDING-2 SIGNATURE, CONFIRMED A THIRD TIME AND NOW IN A THIRD PHASE.
    tlp_request_tracker measures per-tag age from ALLOCATION (:39) and allocation
    precedes the credit gate (tlp_layer.sv:280, tlp_requester.sv:138). So a
    request starved of credit for longer than CPL_TIMEOUT_CYCLES TIMES OUT WITHOUT
    EVER HAVING BEEN TRANSMITTED, and is indistinguishable from a dead device.

    The expected signature is therefore ENUM_ERR_CREDIT_STARVED *with*
    err_credit_blocked_o (sec 63 #7f #19: the code names the cause, the
    annotation stays; tx_fc_blocked_i is still never control flow). No FSM
    above this stack can ride the bound out; fixing it means raising
    CPL_TIMEOUT_CYCLES toward the ~10 ms the spec recommends, which is Stage H.

    !! ZERO WOULD NOT WORK. Advertising 0 at FC init means INFINITE (SS2.6.1
    p.138, fn 33 p.137), so the starving advertisement must be small and FINITE
    with no replenishment -- exactly 3 non-posted headers, consumed by the scan's
    two reads and the first BAR write, leaving the BAR phase wedged.
    """
    dev = acceptance_device()
    mon, completer = await init(
        dut, device=dev,
        credits={"ph": 0, "pd": 0, "nph": 3, "npd": 3, "cplh": 0, "cpld": 0})
    await start_enum(dut)
    await wait_enum(dut)
    snap = await status(dut)

    assert snap["error"] == 1, f"credit starvation did not error: {snap}"
    assert snap["code"] == ENUM_ERR_CREDIT_STARVED, (
        f"expected ENUM_ERR_CREDIT_STARVED, got {err_name(snap['code'])}. A request "
        "starved of credit past CPL_TIMEOUT_CYCLES times out having never been "
        "transmitted (tlp_request_tracker.sv:39 vs tlp_layer.sv:280), and "
        "sec 63 #7f #19 makes the engine say so instead of ENUM_ERR_TIMEOUT")
    assert snap["blocked"] == 1, (
        "err_credit_blocked_o is LOW on a timeout that was caused by credit. "
        "That annotation is the only thing distinguishing this from a dead "
        "device, and it is the whole of Finding 2")
    assert snap["done"] == 0, "enum_done_o alongside a timeout"

    # It really was starvation: the transmitter blocked, and the run stopped
    # PART WAY THROUGH the BAR phase rather than before it started.
    assert mon.blocked_seen, "tx_fc_blocked_o never asserted -- credit never ran out"
    seen = nonempty(completer.seen, "nothing reached the wire before starving")
    assert len(seen) < len(GOLDEN_SEQUENCE), (
        f"all {len(seen)} transactions were emitted, so nothing was starved")
    assert any(r.reg_num >= CFG_REG_BAR0 for r in seen), (
        "the run starved before the BAR phase began; this test claims to starve "
        f"DURING it. Emitted: {seen}")
    await mon.wait_timeouts(1)

    # Annotation, NOT control flow: the timeout is reported once and the FSM is
    # terminal, not retrying.
    before = len(completer.seen)
    await settle(dut, 400)
    assert len(completer.seen) == before, \
        "further requests were emitted after the timeout -- the FSM retried"


# ==========================================================================
# E5 -- a late completion and an orphan burst
# ==========================================================================
@cocotb.test()
async def e5_late_completion_and_orphan_burst(dut):
    """Stale completion data after enumeration finishes: counted, and inert.

    pcie_rc_if reports orphan data ONCE PER DWORD (:403-405), so the count is
    exact and worth asserting exactly -- an inequality would pass against a
    report that fired once or a hundred times.
    """
    dev = acceptance_device()
    mon, completer = await init(dut, device=dev)
    await start_enum(dut)
    await wait_enum(dut)
    first = await status(dut)
    assert_acceptance_outcome(first, "E5 (before the orphan burst): ")

    # Four Dwords of payload for a tag nothing is waiting on.
    orphan_dwords = 4
    last = completer.seen[-1]
    await completer.inject([
        cpl_dw0(has_data=True, length_dw=orphan_dwords),
        cpl_dw1(BDF, CPL_SC, byte_count=4 * orphan_dwords),
        cpl_dw2(RID, (last.tag + 0x40) & 0xFF, lower_address=0),
    ] + [0xDEAD0000 | i for i in range(orphan_dwords)])
    await settle(dut, 200)

    later = await status(dut)
    assert later == first, (
        f"the status surface moved after an orphan burst:\n  was  {first}\n"
        f"  now  {later}")

    expect_count(mon.rc_errors, orphan_dwords,
                 "orphan-data reports (pcie_rc_if.sv:403-405, one per Dword)")
    assert all(code == RC_ERR_ORPHAN_DATA for code in mon.rc_errors), \
        f"unexpected RC error codes among the orphan reports: {mon.rc_errors}"

    # ⭐ AND A SECOND, DIFFERENT REPORT, WHICH I DID NOT PREDICT AND WHICH IS
    # CORRECT. The tracker reports the packet ONCE on rc_unexpected_completion_o
    # (tlp_request_tracker.sv:316) because no allocated tag matches it, while
    # pcie_rc_if reports its DATA once per Dword. Two surfaces describing two
    # different facts about one packet -- "a completion arrived for nobody" and
    # "here is how much payload had nowhere to go".
    #
    # The first run of this test asserted only the per-Dword count and then
    # demanded silence everywhere else, so it FAILED on a correct stack. Asserted
    # explicitly now rather than waived: an exact count and an exact code, so a
    # regression that reported the packet twice would still be caught.
    expect_count(mon.unexpected, 1,
                 "unexpected-completion reports (once per PACKET, not per Dword)")
    assert mon.unexpected[0] == TLP_ERR_UNEXPECTED_COMPLETION, (
        f"unexpected-completion code {mon.unexpected[0]}, expected "
        f"{TLP_ERR_UNEXPECTED_COMPLETION} (TLP_ERR_UNEXPECTED_COMPLETION)")

    # Everything else stays silent.
    mon.clean(allow_orphans=True, allow_unexpected=True)
    assert dev.command == CMD_ENABLE_VALUE, \
        "the Command register changed after the orphan burst"


# ==========================================================================
# E6 -- a fault in the BAR phase, through the real stack
# ==========================================================================
@cocotb.test()
async def e6_ur_mid_bar_phase_through_the_real_stack(dut):
    """A device that answers its probe and then rejects a BAR read is a fault.

    The standalone target proves the policy against an invented socket; this
    proves the same outcome survives a real completion travelling back up
    tlp_layer, pcie_rc_if and the tracker.
    """
    # BAR0 is a plain 32-bit BAR here so the UR lands on a register the FSM has
    # a live decode for, rather than mid-pair.
    dev = ConfigDevice(bars={CFG_REG_BAR0: BarSpec(BAR_MEM32, 4 * KB)})
    mon, completer = await init(dut, device=dev, ur_regs=(CFG_REG_BAR2,))

    await start_enum(dut)
    await wait_enum(dut)
    snap = await status(dut)

    assert completer.ur_injected_hits >= 1, \
        "the UR arm never fired -- this test is vacuous"
    assert snap["error"] == 1 and snap["code"] == ENUM_ERR_UR_POST_PROBE, \
        f"expected ENUM_ERR_UR_POST_PROBE, got {err_name(snap['code'])}: {snap}"
    assert snap["blocked"] == 0, "err_credit_blocked_o annotated a UR"
    mon.clean()


# ==========================================================================
# E7 -- the handoff, on the wire, through the real stack
# ==========================================================================
@cocotb.test()
async def e7_command_write_byte_enables_on_the_wire(dut):
    """first_be == 0011 in the Command write's actual TLP header Dword 1.

    The standalone b24 asserts this at the RQ descriptor. Here it is asserted in
    DW1 of the Configuration Write as tlp_generator assembled it (:80), which is
    what the completer would really see -- and it is still the only observable
    that distinguishes a mux which SELECTS from one which MERGES, because
    pcie_enum_scan drives a hard 1111 unqualified by state.
    """
    dev = acceptance_device()
    mon, completer = await init(dut, device=dev)
    await start_enum(dut)
    await wait_enum(dut)
    snap = await status(dut)
    assert_acceptance_outcome(snap, "E7: ")

    seen = nonempty(completer.seen, "E7: nothing on the wire")
    cmd = expect_count(
        [r for r in seen if (not r.is_read) and r.reg_num == CFG_REG_COMMAND_STATUS],
        1, "E7 Command write TLPs")[0]

    assert cmd.first_be == CFG_BE_LOWER_HALF, (
        f"the Command write's on-wire first_be is {cmd.first_be:#06b}, expected "
        f"{CFG_BE_LOWER_HALF:#06b}. 1111 is what pcie_enum_scan drives "
        "unconditionally, so seeing it here means the handoff mux MERGED the two "
        "stages' command ports instead of SELECTING one -- and a whole-Dword "
        "write would clear the Status register's write-1-to-clear bits")
    assert cmd.dw1 == cfg_wire_dw1(RID, cmd.tag, CFG_BE_LOWER_HALF), \
        f"Command write DW1 {cmd.dw1:#010x} != golden"
    expect_count(cmd.payload, 1, "E7 Command write payload Dwords")
    assert cmd.payload[0] == CMD_ENABLE_VALUE, \
        f"Command write payload {cmd.payload[0]:#010x} != {CMD_ENABLE_VALUE:#010x}"

    # Every other transaction is 1111, which is what makes 0011 diagnostic.
    others = [r for r in seen if r.reg_num != CFG_REG_COMMAND_STATUS]
    assert others and all(r.first_be == CFG_BE_DWORD for r in others), \
        "a non-Command transaction used a byte enable other than 1111"
    mon.clean()
