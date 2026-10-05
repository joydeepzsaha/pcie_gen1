"""test_pcie_rq_rc_top -- cocotb tests of the Root Complex transaction-layer top

Author: Kourosh Ghahramani
Silicon Systems Research Lab, University of Washington

Under test
    pcie_rq_rc_top, through the bench top tb_pcie_rq_rc_top, which sets
    TAG_COUNT = 8 and CPL_TIMEOUT_CYCLES = 6250 and leaves the host aperture
    at its default of 4 GB at address 0. Four interface modules surround one
    tlp_layer:
        host RQ AXIS -> pcie_rq_if -> tlp_layer -> TX stream (m_dllp_axis_*)
        RX stream (s_dllp_axis_*) -> tlp_layer -> pcie_rc_if -> host RC AXIS
        RX stream -> tlp_layer -> pcie_cq_if -> host CQ AXIS (m_axis_cq_*)
        host CC AXIS (s_axis_cc_*) -> pcie_cc_if -> tlp_layer -> TX stream
Stimulus
    Python drives the 4 ns clock, reset, link state, credit limits and
    identity inputs, writes RQ descriptors and CC completions on the host
    streams, and plays the Data Link Layer on both DLL streams: it answers
    the requests the DUT transmits (ConfigCompleter) and sends inbound
    requests as a device would (inject_rx). init() raises link_up_i,
    transmit_enable_i and fc_initialized_i and loads finite credit limits.
    Without the three inputs and a credit load tlp_layer transmits nothing,
    and a request held back by one of the three inputs raises no error.
A pass means
    Header and descriptor fields match goldens built by hand from PG213 or
    the PCIe Base Spec, tags match the tag the DUT put on the wire, and the
    error strobes a test records stay silent unless it expects one.
Limitations
    ConfigCompleter checks nothing about the requests it answers. The credit
    limits are never the limiter, so flow-control gating is not exercised.
    One configuration only: RCB 64 bytes, MPS 128 bytes, the default host
    aperture.
Structure
    Constants
    Descriptor goldens: RQ and RC descriptors, request and Completion Dwords
    The completer: Request and ConfigCompleter
    Harness: the Rc recorder, init, send_rq, cfg_read, cfg_write
    Requester round trips: CfgRd0, CfgWr0, out of order, backpressure, CRS, UR
    Completion timeout
    Type 1 configuration round trip
    Inbound requests: RX helpers, UR answers, the Message control
    CQ path: delivery to the host or a drop strobe
    CC path: host completions become Completions on the wire
    Posted drops and UR interleaving
    RCB splitting
    Transmit ordering
    The host accept window
    test_pcie_rc_dl_top.py and test_pcie_enum_dl_top.py import helpers from
    this module (Rc, CqWatch, rq_desc, cc_desc, send_rq, send_cc and others).
References
    PG213, Table 10
    PG213, Table 52
    PG213, Table 57
    PG213, Table 58
    PG213, Table 60
    PG213, Table 61
    PG213, Table 65
    PG213, Figure 34
    PCIe Base Spec r2.1, §2.2.1
    PCIe Base Spec r2.1, §2.2.4.1
    PCIe Base Spec r2.1, §2.2.5
    PCIe Base Spec r2.1, §2.2.6.3
    PCIe Base Spec r2.1, §2.2.7
    PCIe Base Spec r2.1, §2.2.8
    PCIe Base Spec r2.1, §2.2.9
    PCIe Base Spec r2.1, §2.3.1
    PCIe Base Spec r2.1, §2.3.1.1
    PCIe Base Spec r2.1, §2.4.1
    PCIe Base Spec r2.1, §7.3.3
    PCIe Base Spec r2.1, §7.5.3
"""

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ReadOnly, RisingEdge

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------
# Bench parameters, and the field encodings the goldens below are built from.
# Each encoding is copied from the package that defines it (tlp_pkg or
# pcie_rq_rc_pkg) and must change with it. RID is the Root Complex's own
# Requester ID. COMPLETER is the far-end completer's BDF in the requester
# tests; the completer-path tests also drive it on completer_id_i as the Root
# Complex's own Completer ID. The inbound-request, CQ and CC constants sit
# further down, ahead of the tests that use them.
CLK_NS = 4

# The CPL_TIMEOUT_CYCLES override in tb_pcie_rq_rc_top; the two must match.
# The completion-timeout tests wait out a whole interval, which at the shipped
# default (tlp_pkg::CPL_TIMEOUT_DEFAULT_CYCLES, 1,250,000 cycles) would make
# each wait 200 times longer; what they check does not depend on the value. The
# window adds margin for tlp_request_tracker's round-robin expiry scan. The
# shipped default is checked by test_tlp_cpl_timeout_default.py in tb/tlp.
BENCH_CPL_TIMEOUT_CYCLES = 6250
BENCH_TIMEOUT_WINDOW = BENCH_CPL_TIMEOUT_CYCLES + 304

# The bench instantiates the DUT with TAG_COUNT = 8 (tb_pcie_rq_rc_top.sv), so
# the backpressure test exhausts the tags after eight requests.
TAG_COUNT = 8

# pcie_rq_rc_pkg::rq_req_type_e
RQ_MEM_READ = 0b0000
RQ_MEM_WRITE = 0b0001
RQ_CFG_READ0 = 0b1000
RQ_CFG_WRITE0 = 0b1010
# Type 1 configuration requests
RQ_CFG_READ1 = 0b1001
RQ_CFG_WRITE1 = 0b1011

# tlp_pkg::tlp_fmt_e / tlp_type_e
FMT_3DW_NO_DATA = 0b000
FMT_3DW_DATA = 0b010
TYPE_CFG0 = 0b00100
TYPE_CFG1 = 0b00101
TYPE_CPL = 0b01010

# tlp_pkg::tlp_cpl_status_e == PG213 Completion Status == RC descriptor [45:43]
CPL_SC = 0b000
CPL_UR = 0b001
CPL_CRS = 0b010

# pcie_rq_rc_pkg::rc_desc_error_e
EC_NORMAL = 0b0000
EC_BAD_STATUS = 0b0010

# pcie_rq_rc_pkg::rc_error_e
RC_ERR_ORPHAN_DATA = 3

RID = 0x1234        # the Root Complex's own requester_id_i
COMPLETER = 0x0100  # the completer's BDF: bus 1, device 0, function 0


# ---------------------------------------------------------------------------
# Descriptor goldens
# ---------------------------------------------------------------------------
# Builders and decoders for the RQ descriptor (PG213, Table 60 and Table 61),
# the RC descriptor (PG213, Table 65), and the request and Completion header
# Dwords on the DLL streams. They are written from the tables and from
# tlp_pkg, never read back from the DUT, so a field the DUT misplaces fails
# the comparison instead of agreeing with itself. Every Dword here is in the
# DUT's host Dword order (PCIE_WIRE_ORDER is left at 0 by the bench).
def rq_desc(req_type, dword_count, address=0, completer_id=0, tc=0, attr=0):
    """The 128-bit RQ descriptor (PG213, Table 60 and Table 61).

    The Tag field [103:96] is left 0: pcie_rq_if never reads it, because
    tlp_request_tracker allocates the tag.
    """
    v = address & ((1 << 64) - 1)
    v |= (dword_count & 0x7FF) << 64
    v |= (req_type & 0xF) << 75
    v |= (completer_id & 0xFFFF) << 104
    v |= (tc & 0x7) << 121
    v |= (attr & 0x7) << 124
    return v


def cfg_desc_address(reg_num, ext_reg=0):
    """Configuration form of the RQ descriptor address: {ext_reg, reg_num, 00}."""
    return ((ext_reg & 0xF) << 8) | ((reg_num & 0x3F) << 2)


def tuser(first_be, last_be):
    """s_axis_rq_tuser for one request: first_be in [3:0], last_be in [7:4]."""
    return ((last_be & 0xF) << 4) | (first_be & 0xF)


def cfg_wire_dw2(bus, dev, fn, reg_num, ext_reg=0):
    """The config-request address DW as the generator emits it.

    {bus[31:24], device[23:19], function[18:16], ext_reg[11:8], reg[7:2], 00}
    (tlp_generator's dw2 assembly). pcie_rq_if takes the BDF from the RQ
    descriptor's Completer ID field, not from its address, so a config
    request needs completer_id set and this golden carries the BDF.
    """
    return (((bus & 0xFF) << 24) | ((dev & 0x1F) << 19) | ((fn & 0x7) << 16)
            | ((ext_reg & 0xF) << 8) | ((reg_num & 0x3F) << 2))


def decode_rc_desc(v):
    """PG213 Table 65, the 96-bit RC descriptor."""
    return {
        "lower_address": v & 0xFFF,
        "error_code": (v >> 12) & 0xF,
        "byte_count": (v >> 16) & 0x1FFF,
        "locked": (v >> 29) & 1,
        "request_completed": (v >> 30) & 1,
        "dword_count": (v >> 32) & 0x7FF,
        "status": (v >> 43) & 0x7,
        "poisoned": (v >> 46) & 1,
        "requester_id": (v >> 48) & 0xFFFF,
        "tag": (v >> 64) & 0xFF,
        "completer_id": (v >> 72) & 0xFFFF,
        "tc": (v >> 89) & 0x7,
        "attr": (v >> 92) & 0x7,
    }


def dw0_length(dw0):
    """Recover length_dw from a TX DW0 (inverse of tlp_generator.sv, the dw0 assembly)."""
    enc = ((dw0 >> 24) & 0xFF) | (((dw0 >> 16) & 0x3) << 8)
    return 1024 if enc == 0 else enc


def cpl_dw0(has_data, length_dw, tc=0, attr=0):
    """Completion DW0, laid out as tlp_parser's RX_FIRST state reads it."""
    fmt = FMT_3DW_DATA if has_data else FMT_3DW_NO_DATA
    enc = length_dw & 0x3FF
    v = (fmt << 5) | TYPE_CPL
    v |= ((attr >> 2) & 0x1) << 10
    v |= (tc & 0x7) << 12
    v |= (attr & 0x3) << 20
    v |= ((enc >> 8) & 0x3) << 16
    v |= (enc & 0xFF) << 24
    return v & 0xFFFFFFFF


def cpl_dw1(completer_id, status, byte_count, bcm=0):
    """{completer_id[31:16], status[15:13], BCM[12], byte_count[11:0]}."""
    return (((completer_id & 0xFFFF) << 16) | ((status & 0x7) << 13)
            | ((bcm & 1) << 12) | (byte_count & 0xFFF))


def cpl_dw2(requester_id, tag, lower_address):
    """{requester_id[31:16], tag[15:8], lower_address[6:0]}."""
    return (((requester_id & 0xFFFF) << 16) | ((tag & 0xFF) << 8)
            | (lower_address & 0x7F))


# ---------------------------------------------------------------------------
# The completer
# ---------------------------------------------------------------------------
# A minimal config completer on the DLL streams. It parses each TLP the DUT
# emits on the TX stream enough to know its tag and whether it wants data;
# complete() injects the matching Cpl or CplD on the RX stream. It checks
# nothing about the request: it is a stimulus source, not a checker. The
# tests use four names, which test_pcie_enum_txn_tlp.py also keeps:
#     .start()                     spawn the TX watcher
#     .seen                        list of Request objects in emission order,
#                                  one per TLP off the wire
#     await .wait_for(n)           block until n requests have been observed
#     await .complete(req, ...)    inject one completion for that request
# The late-completion test also calls ._inject directly. Neither the DUT nor
# tb_pcie_rq_rc_top contains any completer logic.
class Request:
    """One TLP leaving the Transaction Layer: a request or a Completion."""

    def __init__(self, dwords):
        """Decode the header fields from the TLP's Dwords."""
        dw0, dw1, dw2 = dwords[0], dwords[1], dwords[2]
        self.dwords = dwords
        self.fmt = (dw0 >> 5) & 0x7
        self.tlp_type = dw0 & 0x1F
        self.length_dw = dw0_length(dw0)
        self.requester_id = (dw1 >> 16) & 0xFFFF
        self.tag = (dw1 >> 8) & 0xFF
        self.last_be = (dw1 >> 4) & 0xF
        self.first_be = dw1 & 0xF
        self.cfg_address = dw2
        self.reg_num = (dw2 >> 2) & 0x3F
        self.payload = dwords[3:]
        # A request wants data back exactly when it carried none going out.
        # For the configuration and memory requests these tests issue, that
        # separates reads from writes.
        self.is_read = (self.fmt & 0b010) == 0

    def __repr__(self):
        """Short form for assertion messages; it prints Cfg...0 for any type."""
        kind = "Rd" if self.is_read else "Wr"
        return (f"Cfg{kind}0(tag={self.tag:#04x}, reg={self.reg_num:#04x}, "
                f"len={self.length_dw}, fbe={self.first_be:#06b})")


class ConfigCompleter:
    """Minimal, swappable config completer; see the section comment above."""

    def __init__(self, dut, requester_id=RID, completer_id=COMPLETER):
        """Hold the IDs the completions carry; nothing is observed yet."""
        self.dut = dut
        self.requester_id = requester_id
        self.completer_id = completer_id
        self.seen = []
        self._partial = []

    def start(self):
        """Spawn the TX watcher."""
        cocotb.start_soon(self._watch_tx())

    async def _watch_tx(self):
        """Collect each TLP accepted on the TX stream as a Request in .seen."""
        d = self.dut
        while True:
            await RisingEdge(d.clk_i)
            await ReadOnly()
            if int(d.rst_i.value):
                continue
            if int(d.m_dllp_axis_tvalid.value) and int(d.m_dllp_axis_tready.value):
                self._partial.append(int(d.m_dllp_axis_tdata.value))
                if int(d.m_dllp_axis_tlast.value):
                    self.seen.append(Request(self._partial))
                    self._partial = []

    async def wait_for(self, count, cycles=400):
        """Block until `count` request TLPs have been seen, or fail."""
        for _ in range(cycles):
            await RisingEdge(self.dut.clk_i)
            if len(self.seen) >= count:
                return
        raise AssertionError(
            f"expected {count} request TLPs on the wire, saw {len(self.seen)} "
            f"({self.seen}) -- FC credits, or tags exhausted?")

    async def complete(self, req, status=CPL_SC, data=None, byte_count=None):
        """Inject the completion answering `req`.

        A read gets a CplD carrying `data` (default: a value derived from the
        tag, so a mis-paired payload is visible); a write gets a data-less Cpl.
        A non-SC status always answers with a Cpl and no data, because a read
        Completion with any other status carries none (PCIe Base Spec r2.1,
        §2.2.1, Table 2-3).

        For an SC read, Byte Count must equal the bytes tlp_request_tracker
        still expects for the tag; it is not checked otherwise. The default 4
        is the value every configuration Completion carries (PCIe Base Spec
        r2.1, §2.2.9).
        """
        has_data = req.is_read and status == CPL_SC
        if byte_count is None:
            byte_count = 4
        words = [
            cpl_dw0(has_data=has_data, length_dw=1 if has_data else 0),
            cpl_dw1(self.completer_id, status, byte_count=byte_count),
            # Lower Address is 0 for every Completion except a Memory Read
            # Completion (PCIe Base Spec r2.1, §2.2.9). tlp_layer seeds the
            # expected Lower Address with 0 for every non-Memory request, and
            # tlp_request_tracker rejects an SC CplD that disagrees.
            cpl_dw2(self.requester_id, req.tag, lower_address=0),
        ]
        if has_data:
            words.append(0xD0000000 | req.tag if data is None else data)
        await self._inject(words)

    async def _inject(self, words):
        """Drive one TLP into the RX stream, one Dword per accepted beat."""
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


# ---------------------------------------------------------------------------
# Harness
# ---------------------------------------------------------------------------
# Rc samples the host RC stream and the requester-side strobes every cycle
# after reset: RC packets, pcie_rq_tag_o, the RQ, RC, unexpected-completion
# and command error codes, and the completion-timeout and late-completion
# tags. Rc.clean() asserts the strobes stayed silent. init() resets the DUT,
# raises the link and credit inputs, and starts both Rc and ConfigCompleter;
# every test calls it once, first. send_rq writes RQ AXIS beats with the
# valid/ready handshake, and cfg_read and cfg_write wrap it for one
# configuration request each.
class Rc:
    """Records RC packets and the error/status surface, concurrently."""

    def __init__(self, dut):
        """Start with every record empty."""
        self.dut = dut
        self.packets = []
        self._partial = []
        self.tags_presented = []
        self.rq_errors = []
        self.rc_errors = []
        self.unexpected = []
        self.command_errors = []
        # Completion Timeout sideband from tlp_request_tracker. Recorded for
        # every test, so a test that answers every request can assert through
        # clean() that it stayed silent.
        self.timeouts = []
        self.lates = []

    def start(self):
        """Spawn the sampling coroutine."""
        cocotb.start_soon(self._run())

    async def _run(self):
        """Sample the RC stream and every strobe at each clock after reset."""
        d = self.dut
        while True:
            await RisingEdge(d.clk_i)
            await ReadOnly()
            if int(d.rst_i.value):
                continue
            if int(d.m_axis_rc_tvalid.value) and int(d.m_axis_rc_tready.value):
                self._partial.append((int(d.m_axis_rc_tdata.value),
                                      int(d.m_axis_rc_tkeep.value),
                                      int(d.m_axis_rc_tlast.value)))
                if int(d.m_axis_rc_tlast.value):
                    self.packets.append(self._partial)
                    self._partial = []
            if int(d.pcie_rq_tag_vld_o.value):
                self.tags_presented.append(int(d.pcie_rq_tag_o.value))
            if int(d.rq_protocol_error_o.value):
                self.rq_errors.append(int(d.rq_error_code_o.value))
            if int(d.rc_protocol_error_o.value):
                self.rc_errors.append(int(d.rc_error_code_o.value))
            if int(d.rc_unexpected_completion_o.value):
                self.unexpected.append(int(d.rc_completion_error_code_o.value))
            if int(d.command_error_valid_o.value):
                self.command_errors.append(int(d.command_error_code_o.value))
            if int(d.cpl_timeout_valid_o.value):
                self.timeouts.append(int(d.cpl_timeout_tag_o.value))
            if int(d.late_cpl_valid_o.value):
                self.lates.append(int(d.late_cpl_tag_o.value))

    async def wait_timeouts(self, count, cycles=BENCH_TIMEOUT_WINDOW):
        """Block until `count` completion-timeout strobes have been seen.

        A strobe arrives CPL_TIMEOUT_CYCLES after the request's handoff to the
        Data Link Layer, plus up to one round of tlp_request_tracker's
        one-tag-per-cycle expiry scan. The default window is derived from
        BENCH_CPL_TIMEOUT_CYCLES, so it moves with the bench's timeout value
        and still fails a strobe that is grossly late.
        """
        for _ in range(cycles):
            await RisingEdge(self.dut.clk_i)
            if len(self.timeouts) >= count:
                return
        raise AssertionError(
            f"expected {count} cpl_timeout strobes, saw {len(self.timeouts)} "
            f"({self.timeouts}) after {cycles} cycles")

    async def wait_lates(self, count, cycles=200):
        """Block until `count` late-completion strobes have been seen."""
        for _ in range(cycles):
            await RisingEdge(self.dut.clk_i)
            if len(self.lates) >= count:
                return
        raise AssertionError(
            f"expected {count} late_cpl strobes, saw {len(self.lates)} ({self.lates})")

    async def wait_packets(self, count, cycles=1500):
        """Block until `count` RC packets have been recorded."""
        for _ in range(cycles):
            await RisingEdge(self.dut.clk_i)
            if len(self.packets) >= count:
                return
        raise AssertionError(
            f"expected {count} RC packets, saw {len(self.packets)}")

    def clean(self, allow_timeouts=False):
        """Assert that no error strobe fired, and no timeout unless allowed."""
        assert self.rq_errors == [], f"RQ protocol errors: {self.rq_errors}"
        assert self.rc_errors == [], f"RC protocol errors: {self.rc_errors}"
        assert self.unexpected == [], f"unexpected completions: {self.unexpected}"
        assert self.command_errors == [], f"TL command errors: {self.command_errors}"
        if not allow_timeouts:
            # A test that answers its requests must not trip the completion
            # timeout. This is the check that fails if CPL_TIMEOUT_CYCLES is
            # set below what these tests need; wait_timeouts fails if it is set
            # above its window.
            assert self.timeouts == [], \
                f"completion timeout fired for tags {self.timeouts} in a test that answers"
            assert self.lates == [], f"late completions drained: {self.lates}"


def packet_dwords(beats):
    """Flatten (tdata, tkeep, tlast) beats into the Dwords tkeep marks valid."""
    words = []
    for tdata, tkeep, _last in beats:
        for dword in range(4):
            if (tkeep >> dword) & 1:
                words.append((tdata >> (32 * dword)) & 0xFFFFFFFF)
    return words


def split_packet(beats):
    """(96-bit descriptor, [payload Dwords])."""
    words = packet_dwords(beats)
    assert len(words) >= 3, f"RC packet shorter than a descriptor: {words}"
    return words[0] | (words[1] << 32) | (words[2] << 64), words[3:]


def init_flow_control(dut):
    """Load the largest finite credit limit into every VC0 pool.

    Until fc_initialized_i is set and an fc_update_valid_i strobe has loaded
    the limits, tlp_credit_manager holds request_ready_o low and the DUT
    transmits nothing, so a check that nothing reached the wire would pass
    without testing anything. fc_update_valid_i stays high, so every later
    cycle reloads the same limits as an update. Flow control has its own
    bench (tb_tlp_credit_manager); here the pools must never be the limiter.
    """
    dut.fc_initialized_i.value = 1
    dut.fc_update_valid_i.value = 1
    dut.fc_ph_i.value = 0xFF
    dut.fc_pd_i.value = 0xFFF
    dut.fc_nph_i.value = 0xFF
    dut.fc_npd_i.value = 0xFFF
    dut.fc_cplh_i.value = 0xFF
    dut.fc_cpld_i.value = 0xFFF


async def init(dut, rc_ready=1):
    """Start the clock, reset the DUT, bring the link and credits up.

    Every input gets its starting value during reset; link_up_i also holds
    tlp_layer in reset while it is low. Returns the running
    (Rc, ConfigCompleter) pair.
    Each call starts another Clock on clk_i, so a test calls it once.
    """
    cocotb.start_soon(Clock(dut.clk_i, CLK_NS, units="ns").start())
    dut.rst_i.value = 1
    dut.link_up_i.value = 0
    dut.transmit_enable_i.value = 0
    dut.s_axis_rq_tdata.value = 0
    dut.s_axis_rq_tkeep.value = 0
    dut.s_axis_rq_tvalid.value = 0
    dut.s_axis_rq_tlast.value = 0
    dut.s_axis_rq_tuser.value = 0
    dut.s_dllp_axis_tdata.value = 0
    dut.s_dllp_axis_tkeep.value = 0
    dut.s_dllp_axis_tvalid.value = 0
    dut.s_dllp_axis_tlast.value = 0
    dut.s_dllp_axis_tuser.value = 0
    dut.m_dllp_axis_tready.value = 1
    dut.m_axis_rc_tready.value = rc_ready
    # The completer-side host surface is idle but accepting: the host takes
    # every CQ packet and sends no CC packet until a test drives one.
    dut.m_axis_cq_tready.value = 1
    dut.s_axis_cc_tdata.value = 0
    dut.s_axis_cc_tkeep.value = 0
    dut.s_axis_cc_tvalid.value = 0
    dut.s_axis_cc_tlast.value = 0
    dut.s_axis_cc_tuser.value = 0
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
    for name in ("fc_ph_i", "fc_pd_i", "fc_nph_i", "fc_npd_i",
                 "fc_cplh_i", "fc_cpld_i"):
        getattr(dut, name).value = 0
    for _ in range(4):
        await RisingEdge(dut.clk_i)
    dut.rst_i.value = 0
    dut.link_up_i.value = 1
    dut.transmit_enable_i.value = 1
    init_flow_control(dut)
    for _ in range(4):
        await RisingEdge(dut.clk_i)

    rc = Rc(dut)
    rc.start()
    completer = ConfigCompleter(dut)
    completer.start()
    await RisingEdge(dut.clk_i)
    return rc, completer


async def send_rq(dut, beats, limit=4000):
    """beats: (tdata, tkeep, tlast, tuser) on the host RQ AXIS."""
    for data, keep, last, user in beats:
        dut.s_axis_rq_tdata.value = data
        dut.s_axis_rq_tkeep.value = keep
        dut.s_axis_rq_tlast.value = 1 if last else 0
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


async def cfg_read(dut, reg_num, first_be=0xF):
    """Issue one CfgRd0 on the host RQ interface."""
    desc = rq_desc(RQ_CFG_READ0, dword_count=1,
                   address=cfg_desc_address(reg_num), completer_id=COMPLETER)
    await send_rq(dut, [(desc, 0xF, True, tuser(first_be, 0x0))])


async def cfg_write(dut, reg_num, data, first_be=0xF):
    """Issue one CfgWr0 on the host RQ interface: descriptor beat + one Dword."""
    desc = rq_desc(RQ_CFG_WRITE0, dword_count=1,
                   address=cfg_desc_address(reg_num), completer_id=COMPLETER)
    await send_rq(dut, [
        (desc, 0xF, False, tuser(first_be, 0x0)),
        (data, 0x1, True, 0),
    ])


async def settle(dut, cycles=30):
    """Wait `cycles` clock edges."""
    for _ in range(cycles):
        await RisingEdge(dut.clk_i)


# ---------------------------------------------------------------------------
# Requester round trips
# ---------------------------------------------------------------------------
# Configuration requests from the host RQ stream, answered by ConfigCompleter
# and returned on the host RC stream. Each test checks RC descriptor fields
# (decode_rc_desc) against goldens, the descriptor's tag against the tag on
# the wire, and that outstanding_o returns to 0 once every request is
# answered. The cases: a CfgRd0, a one-byte CfgWr0, four reads answered out
# of order, tag exhaustion under RC backpressure, and CRS and UR completions.
# The verilate_rq_if_tlp and verilate_rc_if_tlp targets run the same
# requester path without pcie_cq_if and pcie_cc_if, and neither holds
# m_axis_rc_tready low; the tag-exhaustion test here does.
@cocotb.test()
async def v1_cfgrd0_round_trip(dut):
    """RQ descriptor in -> completer returns CplD -> RC packet out.

    Reads one config register and checks the data, the tag and the
    descriptor fields on the way back, and that the tag is released.
    """
    rc, completer = await init(dut)
    assert int(dut.outstanding_o.value) == 0, "a fresh DUT holds no tags"

    await cfg_read(dut, reg_num=0x00)          # Vendor/Device ID
    await completer.wait_for(1)
    req = completer.seen[0]
    assert req.tlp_type == TYPE_CFG0, f"emitted type {req.tlp_type:#07b} != CfgRd0"
    assert req.length_dw == 1, f"a config request is always 1 Dword, got {req.length_dw}"
    assert req.requester_id == RID, \
        f"Requester ID {req.requester_id:#06x} != requester_id_i {RID:#06x}"
    assert int(dut.outstanding_o.value) == 1, "a CfgRd0 must hold a tag"

    # The tag presented to the host must be the tag that went on the wire.
    assert rc.tags_presented == [req.tag], \
        (f"pcie_rq_tag_o {[hex(t) for t in rc.tags_presented]} != the tag in the "
         f"emitted header {req.tag:#04x}")

    read_data = 0x8086100E
    await completer.complete(req, status=CPL_SC, data=read_data)
    await rc.wait_packets(1)

    desc, payload = split_packet(rc.packets[0])
    f = decode_rc_desc(desc)
    assert f["tag"] == req.tag, \
        f"RC descriptor Tag {f['tag']:#04x} != the tag on the wire {req.tag:#04x}"
    assert f["status"] == CPL_SC, f"status {f['status']:#05b} != SC"
    assert f["error_code"] == EC_NORMAL, f"error code {f['error_code']:#06b} != 0000"
    assert f["request_completed"] == 1, "a single-CPL request must set bit 30"
    assert f["dword_count"] == 1, f"Dword Count {f['dword_count']} != 1"
    assert f["byte_count"] == 4, f"Byte Count {f['byte_count']} != 4"
    assert f["requester_id"] == RID
    assert f["completer_id"] == COMPLETER
    assert f["lower_address"] == 0, \
        f"config completion Lower Address {f['lower_address']:#05x} != 0"
    assert payload == [read_data], \
        f"read data {[hex(w) for w in payload]} != [{read_data:#010x}]"

    await settle(dut)
    assert int(dut.outstanding_o.value) == 0, "the tag did not retire"
    rc.clean()


@cocotb.test()
async def v2_byte_granular_cfgwr0(dut):
    """A one-byte config write at offset 0x19: first_be=0010, exactly one TLP.

    Offset 0x19 is byte 1 of the Dword at 0x18, the Secondary Bus Number of
    a Type 1 header (PCIe Base Spec r2.1, §7.5.3), so register number 6 and
    first_be 0010. If pcie_rq_if widened this to a whole-Dword write, it
    would also write the three neighbouring bytes of the register.
    """
    rc, completer = await init(dut)

    await cfg_write(dut, reg_num=0x18 >> 2, data=0x0000_5A00, first_be=0x2)
    await completer.wait_for(1)

    assert len(completer.seen) == 1, \
        f"a 1-Dword config write must emit exactly one TLP, got {len(completer.seen)}"
    req = completer.seen[0]
    assert req.tlp_type == TYPE_CFG0, f"emitted type {req.tlp_type:#07b} != CfgWr0"
    assert not req.is_read, "a CfgWr0 must carry data"
    assert req.first_be == 0b0010, \
        (f"first_be {req.first_be:#06b} != 0010 -- a byte-granular config write "
         "was widened, which would clobber neighbouring config bytes")
    assert req.last_be == 0b0000, f"last_be {req.last_be:#06b} != 0000 for N=1"
    assert req.length_dw == 1, f"length_dw {req.length_dw} != 1"
    # bus 1, device 0, function 0 from COMPLETER; register 6; [1:0] forced 0.
    golden_dw2 = cfg_wire_dw2(bus=1, dev=0, fn=0, reg_num=0x18 >> 2)
    assert req.cfg_address == golden_dw2, \
        (f"config address DW {req.cfg_address:#010x} != {golden_dw2:#010x} -- "
         "the request is addressed at the wrong BDF or register")
    assert int(dut.outstanding_o.value) == 1, "a non-posted write must hold a tag"

    # A config-write completion carries no data.
    await completer.complete(req, status=CPL_SC)
    await rc.wait_packets(1)

    beats = rc.packets[0]
    desc, payload = split_packet(beats)
    f = decode_rc_desc(desc)
    assert f["status"] == CPL_SC, f"status {f['status']:#05b} != SC"
    assert f["error_code"] == EC_NORMAL
    assert f["dword_count"] == 0, \
        f"Dword Count {f['dword_count']} != 0 for a Cpl with no data"
    assert payload == [], f"a write completion carried payload {payload}"
    assert f["tag"] == req.tag
    assert f["request_completed"] == 1
    assert len(beats) == 1, f"descriptor-only packet must be one beat, got {len(beats)}"
    assert beats[0][1] == 0b0111, \
        f"3 descriptor Dwords -> tkeep 0b0111, got {beats[0][1]:#06b}"
    assert beats[0][2] == 1, "descriptor-only packet must assert tlast on beat 0"

    await settle(dut)
    assert int(dut.outstanding_o.value) == 0, "the tag did not retire"
    rc.clean()


@cocotb.test()
async def v3_out_of_order_completions(dut):
    """Four requests in flight, answered 3,1,0,2.  Each RC packet must carry
    its own request's tag and its own payload.

    A design that paired completions with requests by arrival order rather
    than by tag passes every in-order test and fails here. Each completion
    carries a payload derived from its own slot, so a cross-assignment shows
    up in the data as well as in the descriptor.
    """
    rc, completer = await init(dut)

    regs = (0x00, 0x08, 0x10, 0x2C)
    for reg in regs:
        await cfg_read(dut, reg_num=reg >> 2)
    await completer.wait_for(len(regs))

    assert len(completer.seen) == len(regs), \
        f"{len(completer.seen)} TLPs emitted, expected {len(regs)}"
    assert int(dut.outstanding_o.value) == len(regs), \
        f"outstanding_o {int(dut.outstanding_o.value)} != {len(regs)}"

    tags = [r.tag for r in completer.seen]
    assert len(set(tags)) == len(regs), \
        (f"tags {[hex(t) for t in tags]} are not distinct -- with tags reused the "
         "test could not tell correct pairing from constant pairing")
    assert rc.tags_presented == tags, \
        (f"pcie_rq_tag_o sequence {[hex(t) for t in rc.tags_presented]} != the tags "
         f"in the emitted headers {[hex(t) for t in tags]}")

    # Deliberately neither the issue order nor its reverse, so neither
    # "positional" nor "reverse-positional" pairing survives it.
    order = [3, 1, 0, 2]
    expected_data = {slot: 0xBEEF0000 | slot for slot in order}
    for slot in order:
        await completer.complete(completer.seen[slot], status=CPL_SC,
                                 data=expected_data[slot])
    await rc.wait_packets(len(regs))

    assert len(rc.packets) == len(regs), \
        f"{len(rc.packets)} RC packets delivered, expected {len(regs)}"

    for position, slot in enumerate(order):
        desc, payload = split_packet(rc.packets[position])
        f = decode_rc_desc(desc)
        assert f["tag"] == tags[slot], \
            (f"RC packet {position}: Tag {f['tag']:#04x} != the tag of the request "
             f"it answers {tags[slot]:#04x} -- completions are being paired with "
             "requests positionally, not by tag")
        assert payload == [expected_data[slot]], \
            (f"RC packet {position} (tag {f['tag']:#04x}) carried payload "
             f"{[hex(w) for w in payload]}, which belongs to another completion")
        assert f["status"] == CPL_SC and f["error_code"] == EC_NORMAL
        assert f["request_completed"] == 1

    delivered = [decode_rc_desc(split_packet(p)[0])["tag"] for p in rc.packets]
    assert sorted(delivered) == sorted(tags), \
        f"delivered tags {[hex(t) for t in delivered]} != issued {[hex(t) for t in tags]}"

    await settle(dut)
    assert int(dut.outstanding_o.value) == 0, "not every tag retired"
    rc.clean()


@cocotb.test()
async def v4_backpressure_tag_exhaustion_recovery(dut):
    """Hold m_axis_rc_tready low, exhaust the tags, then release.

    The loop under test: RQ -> tag allocation -> completion -> RC drain ->
    tag release -> RQ resumes. The stall must reach the host as ordinary
    AXI-Stream backpressure on s_axis_rq_tready, without deadlock or loss,
    and everything still pending when ready rises must be delivered exactly
    once.
    """
    rc, completer = await init(dut, rc_ready=0)

    # ---- fill every tag -------------------------------------------------
    for index in range(TAG_COUNT):
        await cfg_read(dut, reg_num=index)
    await completer.wait_for(TAG_COUNT)
    assert int(dut.outstanding_o.value) == TAG_COUNT, \
        f"outstanding_o {int(dut.outstanding_o.value)} != {TAG_COUNT}"
    tags = [r.tag for r in completer.seen]
    assert len(set(tags)) == TAG_COUNT, f"tags not distinct: {[hex(t) for t in tags]}"

    # ---- with no tags left, the RQ interface must back-pressure the host --
    #
    # Two extra requests are absorbed without a TLP: tlp_requester accepts
    # one and waits in REQ_TAG for a tag, and pcie_rq_if holds the next
    # descriptor until tlp_requester is ready again. Acceptance must stop
    # there, so this issues more than that and requires the sender to still
    # be blocked.
    extra = 4
    sender = cocotb.start_soon(_issue_many(dut, start_reg=TAG_COUNT, count=extra))
    await settle(dut, 120)

    assert len(completer.seen) == TAG_COUNT, \
        (f"{len(completer.seen)} TLPs emitted with only {TAG_COUNT} tags -- a "
         "request went out without a tag behind it")
    assert int(dut.s_axis_rq_tready.value) == 0, \
        ("s_axis_rq_tready is still high with every tag allocated -- the tag "
         "shortage is not reaching the host as backpressure")
    assert not sender.done(), \
        (f"all {extra} extra requests were accepted with no free tag -- the RQ "
         "interface is buffering without bound instead of back-pressuring")

    # ---- answer them all while the consumer is still stalled -------------
    injector = cocotb.start_soon(_complete_all(completer, completer.seen[:TAG_COUNT]))
    await settle(dut, 200)

    assert rc.packets == [], \
        (f"{len(rc.packets)} RC packets transferred with m_axis_rc_tready low -- "
         "the master is ignoring backpressure")

    # ---- release, and everything must drain ------------------------------
    dut.m_axis_rc_tready.value = 1
    await rc.wait_packets(TAG_COUNT, cycles=4000)
    await injector
    await settle(dut, 60)

    assert len(rc.packets) == TAG_COUNT, \
        (f"{len(rc.packets)} RC packets after the drain, expected exactly "
         f"{TAG_COUNT} -- a completion was lost or duplicated")
    delivered = [decode_rc_desc(split_packet(p)[0])["tag"] for p in rc.packets]
    assert sorted(delivered) == sorted(tags), \
        (f"delivered tags {[hex(t) for t in delivered]} != issued "
         f"{[hex(t) for t in tags]} -- completions lost, duplicated or re-tagged")
    for packet in rc.packets:
        desc, payload = split_packet(packet)
        f = decode_rc_desc(desc)
        assert payload == [0xD0000000 | f["tag"]], \
            (f"tag {f['tag']:#04x} arrived with payload {[hex(w) for w in payload]}, "
             "which belongs to another completion")

    # ---- and the host must be able to make progress again ----------------
    await sender
    await completer.wait_for(TAG_COUNT + extra)
    assert len(completer.seen) == TAG_COUNT + extra, \
        (f"{len(completer.seen)} TLPs emitted after the drain, expected "
         f"{TAG_COUNT + extra} -- tags were not released and the RQ never resumed")

    for req in completer.seen[TAG_COUNT:]:
        await completer.complete(req, status=CPL_SC)
    await rc.wait_packets(TAG_COUNT + extra, cycles=2000)
    await settle(dut, 60)
    assert int(dut.outstanding_o.value) == 0, \
        f"outstanding_o {int(dut.outstanding_o.value)} != 0 after everything drained"
    rc.clean()


async def _issue_many(dut, start_reg, count):
    """Issue `count` CfgRd0s to consecutive registers from `start_reg`."""
    for index in range(count):
        await cfg_read(dut, reg_num=(start_reg + index) & 0x3F, first_be=0xF)


async def _complete_all(completer, requests):
    """Answer each request in order with an SC completion."""
    for req in requests:
        await completer.complete(req, status=CPL_SC)


@cocotb.test()
async def v5_crs_completion(dut):
    """Configuration Request Retry Status carried faithfully to the descriptor.

    After a reset, a device may answer a Configuration Request with CRS
    (PCIe Base Spec r2.1, §2.3.1). The client must see CRS as CRS:
    pcie_cfg_txn retries on it, and a generic error would end enumeration of
    a device that is still initialising.
    """
    rc, completer = await init(dut)

    await cfg_read(dut, reg_num=0x00)
    await completer.wait_for(1)
    req = completer.seen[0]

    await completer.complete(req, status=CPL_CRS)
    await rc.wait_packets(1)

    desc, payload = split_packet(rc.packets[0])
    f = decode_rc_desc(desc)
    assert f["status"] == CPL_CRS, \
        (f"Completion Status {f['status']:#05b} != 010 -- CRS did not survive the "
         "trip and Commit 2b would read it as something else")
    assert f["error_code"] == EC_BAD_STATUS, \
        f"Error Code {f['error_code']:#06b} != 0010 for CRS"
    assert f["tag"] == req.tag
    assert f["request_completed"] == 1, "CRS terminates the request"
    assert f["dword_count"] == 0 and payload == [], "CRS carries no data"

    await settle(dut)
    assert int(dut.outstanding_o.value) == 0, "CRS must retire the tag"
    rc.clean()


@cocotb.test()
async def v6_ur_completion(dut):
    """Unsupported Request carried faithfully; the tag is released.

    A Configuration Request to an unimplemented Function is answered with UR
    (PCIe Base Spec r2.1, §7.3.3), so enumeration sees UR on every probe of a
    Function that is not there. A UR that did not release its tag would leak
    one tag per such probe.
    """
    rc, completer = await init(dut)

    await cfg_read(dut, reg_num=0x00)
    await completer.wait_for(1)
    req = completer.seen[0]
    assert int(dut.outstanding_o.value) == 1

    await completer.complete(req, status=CPL_UR)
    await rc.wait_packets(1)

    desc, payload = split_packet(rc.packets[0])
    f = decode_rc_desc(desc)
    assert f["status"] == CPL_UR, \
        f"Completion Status {f['status']:#05b} != 001 -- UR did not survive the trip"
    assert f["error_code"] == EC_BAD_STATUS, \
        f"Error Code {f['error_code']:#06b} != 0010 for UR"
    assert f["request_completed"] == 1, \
        "bit 30 must be set -- UR terminates the request"
    assert f["tag"] == req.tag
    assert f["dword_count"] == 0 and payload == [], "UR carries no data"

    await settle(dut)
    assert int(dut.outstanding_o.value) == 0, \
        "UR must release the tag -- an enumeration scan would exhaust the pool"
    rc.clean()

    # A second probe must get a tag, i.e. the pool really did recover.
    await cfg_read(dut, reg_num=0x00)
    await completer.wait_for(2)
    await completer.complete(completer.seen[1], status=CPL_UR)
    await rc.wait_packets(2)
    await settle(dut)
    assert int(dut.outstanding_o.value) == 0
    rc.clean()


# ---------------------------------------------------------------------------
# Completion timeout
# ---------------------------------------------------------------------------
# The mechanism itself is tested cycle-exact at CPL_TIMEOUT_CYCLES = 64 by
# the verilate_tlp_cpl_timeout target in tb/tlp. These tests check what only
# the assembled top shows: the cpl_timeout_* and late_cpl_* strobes reach
# the top-level ports, their tags match pcie_rq_tag_o, answered and
# unanswered requests do not disturb each other, and the payload of a late
# completion drains without wedging the receive path. A timed-out tag is
# quarantined: it still counts in outstanding_o and is not allocated again
# until a late completion with its last-completion condition, or a second
# interval, releases it. CPL_TIMEOUT_CYCLES is a cycle count; this bench runs
# at 4 ns, so 6250 cycles here is 25 us.
@cocotb.test()
async def v7_config_read_times_out(dut):
    """A CfgRd0 nobody answers times out, visibly, at the top level.

    The tag in the strobe must be the tag pcie_rq_tag_o presented when the
    request went out, which is how a client ties the timeout to its request.
    The interface must keep accepting requests: only one of TAG_COUNT tags is
    consumed, so recovery here does not depend on the quarantine expiring.
    """
    rc, completer = await init(dut)

    await cfg_read(dut, reg_num=0x00)
    await completer.wait_for(1)
    req = completer.seen[0]
    assert rc.tags_presented == [req.tag]
    assert int(dut.outstanding_o.value) == 1

    # Nobody answers.
    await rc.wait_timeouts(1)
    assert rc.timeouts == [req.tag], (
        f"timeout reported tag {[hex(t) for t in rc.timeouts]}, but the request went "
        f"out on tag {req.tag:#04x} (pcie_rq_tag_o said {[hex(t) for t in rc.tags_presented]})")
    assert rc.packets == [], "a timed-out request must produce NO RC packet"
    assert rc.unexpected == [], "a timeout is not an unexpected completion"
    assert int(dut.outstanding_o.value) == 1, \
        "the quarantined tag still counts as outstanding"

    # ...and the interface is still alive.
    await cfg_read(dut, reg_num=0x04)
    await completer.wait_for(2)
    req2 = completer.seen[1]
    assert req2.tag != req.tag, \
        f"the quarantined tag {req.tag:#04x} must not be handed out again"
    await completer.complete(req2, status=CPL_SC, data=0xC0FFEE00)
    await rc.wait_packets(1)
    desc, payload = split_packet(rc.packets[0])
    f = decode_rc_desc(desc)
    assert f["tag"] == req2.tag, "the second request completed against the wrong tag"
    assert payload == [0xC0FFEE00]
    assert len(rc.timeouts) == 1, "the answered request must not also time out"
    rc.clean(allow_timeouts=True)


@cocotb.test()
async def v8_mixed_answered_and_unanswered(dut):
    """Answered and unanswered requests in flight together do not mix.

    Three reads go out on three distinct tags and only the middle one is
    answered. If every request carried the same tag, the tag checks would
    prove nothing, so the test asserts the tags are distinct before relying
    on them.
    """
    rc, completer = await init(dut)

    for reg in (0x00, 0x04, 0x08):
        await cfg_read(dut, reg_num=reg)
    await completer.wait_for(3)
    reqs = completer.seen[:3]
    tags = [r.tag for r in reqs]
    assert len(set(tags)) == 3, f"the three requests must hold distinct tags, got {tags}"
    assert int(dut.outstanding_o.value) == 3

    answered = reqs[1]
    await completer.complete(answered, status=CPL_SC, data=0xA5A5_0001)
    await rc.wait_packets(1)
    desc, payload = split_packet(rc.packets[0])
    f = decode_rc_desc(desc)
    assert f["tag"] == answered.tag, \
        f"RC descriptor tag {f['tag']:#04x} != the answered request's {answered.tag:#04x}"
    assert payload == [0xA5A5_0001], f"payload {[hex(w) for w in payload]} mis-paired"
    assert rc.timeouts == [], "the answered request completed well inside the interval"
    await settle(dut)
    assert int(dut.outstanding_o.value) == 2, "the answered tag retired"

    # The other two expire; the answered one must not.
    await rc.wait_timeouts(2)
    assert sorted(rc.timeouts) == sorted([reqs[0].tag, reqs[2].tag]), (
        f"timed out {[hex(t) for t in rc.timeouts]}, expected exactly "
        f"{[hex(reqs[0].tag), hex(reqs[2].tag)]}")
    assert answered.tag not in rc.timeouts, \
        "an answered request must never raise a completion timeout"
    assert len(rc.packets) == 1, "no RC packet for a timed-out request"
    assert int(dut.outstanding_o.value) == 2, "two quarantined tags still count"
    assert rc.lates == [], "no completion arrived for the timed-out tags"
    rc.clean(allow_timeouts=True)


@cocotb.test()
async def v9_multibeat_late_completion_drains(dut):
    """A multi-beat late completion drains without wedging anything.

    The request is a 1-Dword config read, but the late completion carries
    four Dwords, so its Length and the request's byte count disagree. A drain
    that sized itself from the request would leave beats behind and stall
    the receive path. tlp_request_tracker does no byte-count checking for a
    quarantined tag, and pcie_rc_if's orphan drain (S_IDLE) swallows the
    beats, printing a simulator warning per Dword; that output is expected.
    """
    rc, completer = await init(dut)

    await cfg_read(dut, reg_num=0x00)
    await completer.wait_for(1)
    req = completer.seen[0]
    await rc.wait_timeouts(1)
    assert rc.timeouts == [req.tag]
    assert rc.packets == []

    late_payload = [0x1111_1111, 0x2222_2222, 0x3333_3333, 0x4444_4444]
    await completer._inject([
        cpl_dw0(has_data=True, length_dw=len(late_payload)),
        cpl_dw1(COMPLETER, CPL_SC, byte_count=4 * len(late_payload)),
        cpl_dw2(RID, req.tag, lower_address=0),
    ] + late_payload)

    await rc.wait_lates(1)
    assert rc.lates == [req.tag], \
        f"late_cpl reported {[hex(t) for t in rc.lates]}, expected {req.tag:#04x}"
    await settle(dut, 60)
    assert rc.packets == [], \
        f"a drained late completion must emit NO RC packet, got {len(rc.packets)}"
    assert rc.unexpected == [], "a drained late completion is not an unexpected completion"

    # pcie_rc_if reports RC_ERR_ORPHAN_DATA once per drained Dword, so the
    # count is the number of payload beats the drain consumed. Four in, four
    # reported: the drain followed the completion's own Length, not the
    # 1-Dword request behind the tag. A drain sized from the request would
    # report 1 here and leave three beats stuck in the receive path.
    assert rc.rc_errors == [RC_ERR_ORPHAN_DATA] * len(late_payload), (
        f"expected {len(late_payload)} orphan-data reports, one per drained Dword; "
        f"got {rc.rc_errors}")
    rc.rc_errors.clear()
    assert int(dut.outstanding_o.value) == 0, \
        "the bit-30 late completion returned the tag to the pool"

    # Nothing wedged: a fresh request/completion still round-trips, on the
    # reused tag.
    await cfg_read(dut, reg_num=0x0C)
    await completer.wait_for(2)
    req2 = completer.seen[1]
    assert req2.tag == req.tag, \
        f"the released tag should be reused, got {req2.tag:#04x} not {req.tag:#04x}"
    await completer.complete(req2, status=CPL_SC, data=0x5EED_0001)
    await rc.wait_packets(1)
    desc, payload = split_packet(rc.packets[0])
    f = decode_rc_desc(desc)
    assert f["tag"] == req2.tag
    assert payload == [0x5EED_0001], \
        f"post-drain round trip returned {[hex(w) for w in payload]}"
    assert len(rc.lates) == 1, "only one late drain"
    assert len(rc.timeouts) == 1, "the second request was answered in time"
    rc.clean(allow_timeouts=True)


# ---------------------------------------------------------------------------
# Type 1 configuration round trip
# ---------------------------------------------------------------------------
# A CfgRd1 and a CfgWr1 from the host RQ stream, each with a BDF other than
# COMPLETER's, so the Completer ID field visibly reaches the address Dword.
# The request side checks the whole DW0, because the Type field's bit 0 is
# all that separates a Type 1 request from a Type 0 one. The completion side
# checks the same RC descriptor fields as the Type 0 round trip: Type 1
# completions are ordinary Cpl and CplD TLPs and take the same return path.
@cocotb.test()
async def v10_cfg1_round_trip(dut):
    """A CfgRd1's CplD and a CfgWr1's Cpl correlate by tag and decode
    identically to the Type 0 path.

    The CfgRd1 DW0 must be 0x01000005 (Fmt 000b, Type 00101b, Length 1) and
    the CfgWr1 DW0 0x01000045 (Fmt 010b); the address Dword must carry the
    descriptor's BDF and register numbers.
    """
    rc, completer = await init(dut)
    bus, dev, fn, reg, ext = 0x2A, 0x03, 0x5, 0x11, 0x2
    bdf = (bus << 8) | (dev << 3) | fn

    # ---- CfgRd1 with a CplD ----
    await send_rq(dut, [(rq_desc(RQ_CFG_READ1, 1,
                                 address=cfg_desc_address(reg, ext),
                                 completer_id=bdf),
                         0xF, True, tuser(0xF, 0x0))])
    await completer.wait_for(1)
    req = completer.seen[0]
    assert req.dwords[0] == 0x01000005, \
        f"CfgRd1 DW0 {req.dwords[0]:#010x} != 0x01000005 " \
        f"(dw0[4:0]={req.dwords[0] & 0x1F:#07b})"
    assert req.tlp_type == TYPE_CFG1 and req.length_dw == 1
    assert req.cfg_address == cfg_wire_dw2(bus, dev, fn, reg, ext), \
        f"CFG1 DW2 {req.cfg_address:#010x} lost the distinct BDF"
    assert rc.tags_presented == [req.tag], "tag correlation surface changed"

    read_data = 0xD1D2_0001
    await completer.complete(req, status=CPL_SC, data=read_data)
    await rc.wait_packets(1)
    desc, payload = split_packet(rc.packets[0])
    f = decode_rc_desc(desc)
    assert f["tag"] == req.tag, f"RC Tag {f['tag']:#04x} != {req.tag:#04x}"
    assert f["status"] == CPL_SC and f["error_code"] == EC_NORMAL
    assert f["request_completed"] == 1 and f["dword_count"] == 1
    assert f["byte_count"] == 4 and f["lower_address"] == 0
    assert payload == [read_data], \
        f"read data {[hex(w) for w in payload]} != [{read_data:#010x}]"

    # ---- CfgWr1 with a data-less Cpl ----
    value = 0xC0FFEE22
    await send_rq(dut, [(rq_desc(RQ_CFG_WRITE1, 1,
                                 address=cfg_desc_address(reg, ext),
                                 completer_id=bdf),
                         0xF, False, tuser(0xF, 0x0)),
                        (value, 0x1, True, 0)])
    await completer.wait_for(2)
    wr = completer.seen[1]
    assert wr.dwords[0] == 0x01000045, \
        f"CfgWr1 DW0 {wr.dwords[0]:#010x} != 0x01000045"
    assert wr.payload == [value]
    await completer.complete(wr, status=CPL_SC)
    await rc.wait_packets(2)
    desc, payload = split_packet(rc.packets[1])
    f = decode_rc_desc(desc)
    assert f["tag"] == wr.tag and f["status"] == CPL_SC
    assert f["dword_count"] == 0 and payload == [], \
        "a write completion carries no data"

    await settle(dut)
    assert int(dut.outstanding_o.value) == 0, "both CFG1 tags must retire"
    rc.clean()


# ---------------------------------------------------------------------------
# Inbound requests
# ---------------------------------------------------------------------------
# Requests a device sends upstream, injected on the RX stream with inject_rx.
# The later blocks reuse the builders below (memrd_tlp, iord_tlp, cfgrd0_tlp
# and the 64-bit forms); memwr_tlp sits in the CQ block. The tests here watch
# the TX stream for the Completion the Root Complex owes: an inbound I/O or
# Configuration request is not supported, so it gets a UR Completion
# (PCIe Base Spec r2.1, §2.3.1). Each such test has a control that watches a
# different signal, the RX acceptance handshake and malformed_o /
# rx_error_valid_o through RxWatch, to show the stimulus is well-formed and
# accepted; without it, a missing Completion could be a rejected stimulus. A
# Message is rejected by tlp_validator before it reaches the completer path,
# and its control records that.

# The device's own BDF. Distinct from RID (this Root Complex) and from
# COMPLETER, so a completion echoing the wrong one is visible rather than
# accidentally equal.
DEVICE_RID = 0x0300

# Header encodings used only by these tests: TYPE_MEM and TYPE_IO as in
# tlp_pkg::tlp_type_e, the 4DW formats as in tlp_fmt_e, and the Message type
# routed to the Root Complex (r[2:0] = 000; PCIe Base Spec r2.1, §2.2.8,
# Table 2-18), which tlp_pkg does not define.
TYPE_MEM = 0b00000
TYPE_IO = 0b00010
TYPE_MSG = 0b10000
FMT_4DW_NO_DATA = 0b001
FMT_4DW_DATA = 0b011

# tlp_pkg::tlp_error_e ordinal (tlp_pkg.sv, the tlp_error_e declaration)
TLP_ERR_BAD_FMT_TYPE = 5

# Inside the host aperture that pcie_rq_rc_top passes to tlp_layer as BAR 0
# (4 GB at address 0 by default), so a Memory request here is delivered on
# CQ. Its low 7 bits are 0, so it is also an RCB-aligned start.
BAR0_ADDRESS = 0x100

# The first address above the default host aperture (4 GB at address 0). It
# needs the 64-bit format, so the tests that use it send 4DW requests. Every
# test using it expects a request here to be dropped, so if HOST_MEM_BASE or
# HOST_MEM_SIZE ever puts the window over this address, or the window becomes
# a base/limit range that covers it, those tests must move above the new
# limit.
OUT_OF_APERTURE_ADDRESS = 0x1_0000_0000


def req_dw0(fmt, tlp_type, length_dw, tc=0, attr=0):
    """Request DW0 as tlp_parser reads it (the RX_FIRST field extraction).

    Bit-for-bit the layout of tlp_generator's dw0 assembly, including the
    split Attr field: Attr[2] at bit 10 and Attr[1:0] at bits [21:20], which
    are not adjacent in the header (PCIe Base Spec r2.1, §2.2.6.3).
    """
    enc = 0 if length_dw == 1024 else (length_dw & 0x3FF)
    v = ((fmt & 0x7) << 5) | (tlp_type & 0x1F)
    v |= ((attr >> 2) & 0x1) << 10
    v |= (tc & 0x7) << 12
    v |= ((enc >> 8) & 0x3) << 16
    v |= (attr & 0x3) << 20
    v |= (enc & 0xFF) << 24
    return v & 0xFFFFFFFF


def req_dw1(requester_id, tag, first_be, last_be):
    """{requester_id[31:16], tag[15:8], last_be[7:4], first_be[3:0]}."""
    return (((requester_id & 0xFFFF) << 16) | ((tag & 0xFF) << 8)
            | ((last_be & 0xF) << 4) | (first_be & 0xF))


def mem_dw2(address):
    """3DW Memory request address DW; the parser forces [1:0] to 0."""
    return address & 0xFFFFFFFC


def decode_cpl(dwords):
    """Decode a Completion off the TX wire (tlp_generator's CPL dw1/dw2 arms).

    Returns None if the TLP is not a Completion, so a caller can say "no
    completion was emitted" without guessing at field positions.
    """
    if len(dwords) < 3:
        return None
    dw0, dw1, dw2 = dwords[0], dwords[1], dwords[2]
    if (dw0 & 0x1F) != TYPE_CPL:
        return None
    has_data = ((dw0 >> 5) & 0b010) != 0
    # An encoded Length of 0 means 1024 Dwords for a TLP with data, which is
    # what dw0_length() implements. A Cpl without data carries no payload, so
    # an encoded 0 is read as Length 0 here, as tlp_parser does for a
    # data-less Completion. The general rule would read a UR Completion as
    # 1024 Dwords long.
    enc = ((dw0 >> 24) & 0xFF) | (((dw0 >> 16) & 0x3) << 8)
    length_dw = 0 if (not has_data and enc == 0) else dw0_length(dw0)
    return {
        "fmt": (dw0 >> 5) & 0x7,
        "length_dw": length_dw,
        "has_data": has_data,
        "completer_id": (dw1 >> 16) & 0xFFFF,
        "status": (dw1 >> 13) & 0x7,
        "bcm": (dw1 >> 12) & 0x1,
        "byte_count": dw1 & 0xFFF,
        "requester_id": (dw2 >> 16) & 0xFFFF,
        "tag": (dw2 >> 8) & 0xFF,
        "lower_address": dw2 & 0x7F,
        "payload": dwords[3:],
    }


class RxWatch:
    """Records the RX-side error surface.

    This is the observation point for the controls: it reads malformed_o,
    rx_error_valid_o and rx_error_code_o, none of which is computed from the
    TX stream the UR tests assert about.
    """

    def __init__(self, dut):
        """Start with no errors recorded."""
        self.dut = dut
        self.errors = []
        self.malformed = 0

    def start(self):
        """Spawn the sampling coroutine."""
        cocotb.start_soon(self._run())

    async def _run(self):
        """Record each rx_error code and count each malformed_o cycle."""
        d = self.dut
        while True:
            await RisingEdge(d.clk_i)
            await ReadOnly()
            if int(d.rst_i.value):
                continue
            if int(d.rx_error_valid_o.value):
                self.errors.append(int(d.rx_error_code_o.value))
            if int(d.malformed_o.value):
                self.malformed += 1


async def inject_rx(dut, words, limit=20000):
    """Drive one TLP into the DUT's RX (DLL-facing) stream, Dword-serial.

    Returns the number of Dwords the DUT accepted. The controls use it to
    show the Transaction Layer took the whole request, since a request it
    never took says nothing about how it is answered.
    """
    accepted = 0
    for index, word in enumerate(words):
        dut.s_dllp_axis_tdata.value = word
        dut.s_dllp_axis_tkeep.value = 0xF
        dut.s_dllp_axis_tlast.value = 1 if index == len(words) - 1 else 0
        dut.s_dllp_axis_tvalid.value = 1
        for _ in range(limit):
            await ReadOnly()
            fired = int(dut.s_dllp_axis_tready.value) == 1
            await RisingEdge(dut.clk_i)
            if fired:
                accepted += 1
                break
        else:
            raise AssertionError(
                f"s_dllp_axis_tready never asserted on Dword {index} -- RX wedged")
    dut.s_dllp_axis_tvalid.value = 0
    dut.s_dllp_axis_tlast.value = 0
    return accepted


def memrd_tlp(tag, address=BAR0_ADDRESS, length_dw=1, first_be=0xF, last_be=0x0):
    """Inbound 3DW Memory Read, as a DMA-ing device would send it upstream.

    length_dw == 1 requires last_be == 0 (PCIe Base Spec r2.1, §2.2.5), and
    tlp_validator enforces it, so the defaults are a legal single-Dword read.
    """
    return [req_dw0(FMT_3DW_NO_DATA, TYPE_MEM, length_dw),
            req_dw1(DEVICE_RID, tag, first_be, last_be),
            mem_dw2(address)]


def mem64_dws(address):
    """The two address Dwords of a 4DW Memory request header.

    Byte 8 carries Address[63:32] and byte 12 carries Address[31:2] (PCIe
    Base Spec r2.1, §2.2.7, Figure 2-15), so DW2 is the high half and DW3 the
    low half. tlp_parser reads them in that order (RX_DW2, then RX_DW3).
    """
    return [(address >> 32) & 0xFFFFFFFF, address & 0xFFFFFFFC]


def memrd64_tlp(tag, address=OUT_OF_APERTURE_ADDRESS, length_dw=1,
                first_be=0xF, last_be=0x0):
    """Inbound 4DW (64-bit address) Memory Read.

    The address must be 4 GB or above. A Requester must use the 32-bit
    format below 4 GB (PCIe Base Spec r2.1, §2.2.4.1), and tlp_validator
    rejects a 64-bit Memory request with address[63:32] == 0 as
    TLP_ERR_BAD_ADDRESS_FORMAT. Such a request never reaches the BAR decode,
    so it would be dropped for its format while seeming to test the window.
    """
    return ([req_dw0(FMT_4DW_NO_DATA, TYPE_MEM, length_dw),
             req_dw1(DEVICE_RID, tag, first_be, last_be)]
            + mem64_dws(address))


def memwr64_tlp(tag, address=OUT_OF_APERTURE_ADDRESS, payload=(0xA5A5_0001,),
                first_be=0xF, last_be=0x0):
    """Inbound 4DW (64-bit address) Memory Write; see memrd64_tlp on 4 GB."""
    n = len(payload)
    return ([req_dw0(FMT_4DW_DATA, TYPE_MEM, n),
             req_dw1(DEVICE_RID, tag, first_be, last_be)]
            + mem64_dws(address) + list(payload))


def iord_tlp(tag, address=0x40):
    """Inbound I/O Read.

    Length is always 1 Dword (PCIe Base Spec r2.1, §2.2.7).
    """
    return [req_dw0(FMT_3DW_NO_DATA, TYPE_IO, 1),
            req_dw1(DEVICE_RID, tag, 0xF, 0x0),
            mem_dw2(address)]


def cfgrd0_tlp(tag, reg_num=0x00):
    """Inbound Configuration Read Type 0 -- a device sending Cfg upstream.

    Well-formed but out of place: only the Host Bridge originates
    Configuration Requests (PCIe Base Spec r2.1, §7.3.3). The Root Complex
    must answer it with UR, not drop it.
    """
    return [req_dw0(FMT_3DW_NO_DATA, TYPE_CFG0, 1),
            req_dw1(DEVICE_RID, tag, 0xF, 0x0),
            (reg_num & 0x3F) << 2]


def msg_tlp(tag):
    """Inbound Message, 4DW no-data.  Rejected by tlp_validator by type."""
    return [req_dw0(FMT_4DW_NO_DATA, TYPE_MSG, 0),
            req_dw1(DEVICE_RID, tag, 0x0, 0x0),
            0x00000000,
            0x00000000]


async def cpls_on_wire(completer):
    """Every Completion the RC put on the TX wire, in emission order."""
    return [c for c in (decode_cpl(r.dwords) for r in completer.seen) if c]


@cocotb.test()
async def a4_control_inbound_memrd_is_accepted(dut):
    """Control: an inbound Memory Read is well-formed and taken.

    All three Dwords are accepted on the RX handshake and neither malformed_o
    nor rx_error_valid_o fires. The CQ and CC read-path tests below send the
    same read, so this test separates a rejected stimulus from a failure on
    those paths.
    """
    rc, completer = await init(dut)
    rx = RxWatch(dut)
    rx.start()

    accepted = await inject_rx(dut, memrd_tlp(tag=0x11))
    await settle(dut, 60)

    assert accepted == 3, \
        f"the TL accepted {accepted} of 3 Dwords -- the read was not consumed"
    assert rx.errors == [], \
        f"a legal inbound MemRd was reported malformed: rx_error codes {rx.errors}"
    assert rx.malformed == 0, \
        f"malformed_o fired {rx.malformed} time(s) on a legal inbound MemRd"


# No test expects the Root Complex to answer an inbound Memory Read by
# itself: the data is in host memory, so the request goes to the host on CQ
# and the host answers on CC (f1_cc_descriptor_becomes_cpld_on_the_wire).
# A UR Completion, by contrast, is synthesised without the host.


@cocotb.test()
async def a4_control_inbound_io_and_cfg_are_accepted(dut):
    """Control for both UR tests: inbound I/O and Cfg requests are accepted.

    An inbound I/O Read and an inbound CfgRd0 are both well-formed TLPs that
    tlp_validator admits, so they reach the completer path and are consumed
    there. Neither is reported malformed, so a missing UR in the two tests
    below is a completer-path failure, not a parser rejection.
    """
    rc, completer = await init(dut)
    rx = RxWatch(dut)
    rx.start()

    assert await inject_rx(dut, iord_tlp(tag=0x21)) == 3
    await settle(dut, 40)
    assert await inject_rx(dut, cfgrd0_tlp(tag=0x22)) == 3
    await settle(dut, 60)

    assert rx.errors == [], \
        f"I/O or Cfg reported malformed -- codes {rx.errors}; these types are legal TLPs"
    assert rx.malformed == 0, f"malformed_o fired {rx.malformed} time(s)"


@cocotb.test()
async def a4_inbound_io_returns_ur(dut):
    """An inbound I/O Read must be answered with a UR Completion.

    A Request whose type the Completer does not support is an Unsupported
    Request, and one that needs a Completion gets Completion Status UR;
    Completer Abort is the wrong status for it (PCIe Base Spec r2.1,
    §2.3.1). The UR Completion is a Cpl and carries no data (PCIe Base Spec
    r2.1, §2.2.1, Table 2-3).
    """
    rc, completer = await init(dut)

    tag = 0x21
    await inject_rx(dut, iord_tlp(tag=tag))
    await settle(dut, 200)

    cpls = await cpls_on_wire(completer)
    assert len(cpls) == 1, \
        f"expected 1 UR Completion answering the inbound I/O Read, saw {len(cpls)}"
    c = cpls[0]
    assert c["status"] == CPL_UR, f"status {c['status']:#05b} != UR"
    assert c["requester_id"] == DEVICE_RID and c["tag"] == tag
    assert not c["has_data"], "a UR Completion carries no data (§2.2.9 p. 97)"
    assert c["length_dw"] == 0, f"Length {c['length_dw']} != 0 for a UR Completion"


@cocotb.test()
async def a4_inbound_cfg_returns_ur(dut):
    """An inbound Configuration Read must be answered with UR.

    Configuration Requests do not travel upstream (PCIe Base Spec r2.1,
    §7.3.3), but this one still needs a Completion, so it is terminated with
    UR (PCIe Base Spec r2.1, §2.3.1), never dropped. A dropped request would
    leave the device waiting for its own Completion Timeout.
    """
    rc, completer = await init(dut)

    tag = 0x22
    await inject_rx(dut, cfgrd0_tlp(tag=tag))
    await settle(dut, 200)

    cpls = await cpls_on_wire(completer)
    assert len(cpls) == 1, \
        f"expected 1 UR Completion answering the inbound CfgRd0, saw {len(cpls)}"
    c = cpls[0]
    assert c["status"] == CPL_UR, f"status {c['status']:#05b} != UR"
    assert c["requester_id"] == DEVICE_RID and c["tag"] == tag
    assert not c["has_data"] and c["length_dw"] == 0


@cocotb.test()
async def a4_control_msg_is_already_strobed(dut):
    """Control: an inbound Message is rejected and reported, not completed.

    tlp_validator admits only MEM, IO, CFG0, CFG1, CPL and CPL_LOCK types, so
    a Message is rejected by type and tlp_parser reports it on malformed_o
    and rx_error_valid_o with TLP_ERR_BAD_FMT_TYPE. It never reaches the
    completer path, so it gets neither a CQ packet nor a Completion.

    Because Messages already strobe rx_error_valid_o, a check that nothing
    inbound is silently dropped cannot be written against rx_error_valid_o:
    it would pass for Messages and say nothing about the Memory path.
    """
    rc, completer = await init(dut)
    rx = RxWatch(dut)
    rx.start()

    await inject_rx(dut, msg_tlp(tag=0x31))
    await settle(dut, 80)

    assert rx.malformed >= 1, \
        "an inbound Message must be reported malformed, not silently consumed"
    assert TLP_ERR_BAD_FMT_TYPE in rx.errors, (
        f"expected TLP_ERR_BAD_FMT_TYPE ({TLP_ERR_BAD_FMT_TYPE}) in {rx.errors} -- "
        "the Message must be rejected by fmt/type, which is what makes it a "
        "reported case rather than an A4 silent discard")

    cpls = await cpls_on_wire(completer)
    assert cpls == [], \
        f"a rejected Message must not be answered with a Completion, saw {cpls}"


# ---------------------------------------------------------------------------
# CQ path
# ---------------------------------------------------------------------------
# pcie_cq_if takes each inbound request from tlp_layer and either delivers it
# to the host as a CQ packet on m_axis_cq_* or raises cq_dropped_o with a
# reason code: only a Memory request inside the host aperture is delivered.
# CqWatch records both outcomes, the first-beat tuser and cc_protocol_error_o.
# Goldens come from PG213, Table 52 (the descriptor), Table 57 (Request
# Type) and Table 10 (the tuser byte enables), and from the PCIe Base Spec
# r2.1, §2.2.5 for the byte enables themselves. A dropped non-posted request
# also gets a UR Completion through pcie_cc_if; a4_inbound_io_returns_ur and
# a4_inbound_cfg_returns_ur check that.

# PG213 Table 57, as pcie_rq_rc_pkg::cq_req_type_e names them
CQ_MEM_READ = 0b0000
CQ_MEM_WRITE = 0b0001
CQ_IO_READ = 0b0010
CQ_CFG_READ0 = 0b1000

# pcie_rq_rc_pkg::cc_error_e
CC_ERR_BAD_STATUS = 1

# pcie_rq_rc_pkg::cq_error_e
CQ_DROP_UNSUPPORTED = 1
CQ_DROP_NO_BAR = 2

# pcie_rq_rc_top's HOST_MEM_APERTURE, derived from HOST_MEM_SIZE: 32 == 4 GB.
# Fixed here, not read back, so a change to either the window or the
# descriptor field is caught. pcie_rq_rc_top derives both the BAR mask and
# this field from HOST_MEM_SIZE; f3_aperture_edge_pair checks the mask's edge
# at the same 4 GB.
CQ_APERTURE = 32


def decode_cq_desc(v):
    """PG213 Table 52 -- the 128-bit / 4-Dword Completer Request descriptor."""
    return {
        "address_type": v & 0x3,
        "address": (v >> 2) << 2 & ((1 << 64) - 1),
        "dword_count": (v >> 64) & 0x7FF,
        "req_type": (v >> 75) & 0xF,
        "requester_id": (v >> 80) & 0xFFFF,
        "tag": (v >> 96) & 0xFF,
        "target_function": (v >> 104) & 0xFF,
        "bar_id": (v >> 112) & 0x7,
        "bar_aperture": (v >> 115) & 0x3F,
        "tc": (v >> 121) & 0x7,
        "attr": (v >> 124) & 0x7,
    }


class CqWatch:
    """Collects CQ packets and cq_dropped_o strobes.

    Records both, so a test can assert that every inbound request produced
    exactly one of them: a CQ packet or a drop strobe.
    """

    def __init__(self, dut):
        """Start with every record empty."""
        self.dut = dut
        self.packets = []      # list of (descriptor_int, [payload Dwords])
        self.drops = []        # cq_error_code_o values
        self.cc_errors = []    # cc_error_code_o values, on cc_protocol_error_o
        self.tusers = []       # m_axis_cq_tuser sampled on the first beat
        self._partial = []
        self._user = None

    def start(self):
        """Spawn the sampling coroutine."""
        cocotb.start_soon(self._run())

    async def _run(self):
        """Sample drops, CC errors and CQ beats; split packets at Dword 4."""
        d = self.dut
        while True:
            await RisingEdge(d.clk_i)
            await ReadOnly()
            if int(d.rst_i.value):
                continue
            if int(d.cq_dropped_o.value):
                self.drops.append(int(d.cq_error_code_o.value))
            if int(d.cc_protocol_error_o.value):
                self.cc_errors.append(int(d.cc_error_code_o.value))
            if int(d.m_axis_cq_tvalid.value) and int(d.m_axis_cq_tready.value):
                if not self._partial:
                    self._user = int(d.m_axis_cq_tuser.value)
                self._partial.append((int(d.m_axis_cq_tdata.value),
                                      int(d.m_axis_cq_tkeep.value)))
                if int(d.m_axis_cq_tlast.value):
                    words = []
                    for tdata, tkeep in self._partial:
                        for dword in range(4):
                            if (tkeep >> dword) & 1:
                                words.append((tdata >> (32 * dword)) & 0xFFFFFFFF)
                    desc = (words[0] | (words[1] << 32)
                            | (words[2] << 64) | (words[3] << 96))
                    self.packets.append((desc, words[4:]))
                    self.tusers.append(self._user)
                    self._partial = []

    async def wait_packets(self, count, cycles=600):
        """Block until `count` CQ packets have been recorded."""
        for _ in range(cycles):
            await RisingEdge(self.dut.clk_i)
            if len(self.packets) >= count:
                return
        raise AssertionError(
            f"expected {count} CQ packet(s), saw {len(self.packets)}")

    async def wait_drops(self, count, cycles=600):
        """Block until `count` cq_dropped_o strobes have been recorded."""
        for _ in range(cycles):
            await RisingEdge(self.dut.clk_i)
            if len(self.drops) >= count:
                return
        raise AssertionError(
            f"expected {count} cq_dropped_o strobe(s), saw {len(self.drops)}")


def memwr_tlp(tag, address=BAR0_ADDRESS, payload=(0xA5A5_0001,),
              first_be=0xF, last_be=0x0):
    """Inbound 3DW Memory Write, as a DMA-ing device would send it upstream."""
    n = len(payload)
    return ([req_dw0(FMT_3DW_DATA, TYPE_MEM, n),
             req_dw1(DEVICE_RID, tag, first_be, last_be),
             mem_dw2(address)] + list(payload))


@cocotb.test()
async def f1_inbound_memwr_reaches_cq(dut):
    """An inbound MemWr becomes a CQ packet, payload intact.

    The descriptor fields are checked against PG213, Table 52, and Request
    Type against Table 57.
    """
    rc, completer = await init(dut)
    cq = CqWatch(dut)
    cq.start()

    payload = (0xDEAD_0001, 0xDEAD_0002)
    tag = 0x41
    await inject_rx(dut, memwr_tlp(tag=tag, address=BAR0_ADDRESS, payload=payload,
                                   first_be=0xF, last_be=0xF))
    await cq.wait_packets(1)

    desc, data = cq.packets[0]
    f = decode_cq_desc(desc)
    assert f["req_type"] == CQ_MEM_WRITE, \
        f"Request Type {f['req_type']:#06b} != CQ_MEM_WRITE (PG213 Table 57)"
    assert f["address"] == BAR0_ADDRESS, \
        f"Address {f['address']:#x} != {BAR0_ADDRESS:#x}"
    assert f["dword_count"] == len(payload), \
        f"Dword Count {f['dword_count']} != {len(payload)}"
    assert f["requester_id"] == DEVICE_RID, \
        f"Requester ID {f['requester_id']:#06x} != the device's {DEVICE_RID:#06x}"
    assert f["tag"] == tag, f"Tag {f['tag']:#04x} != {tag:#04x}"
    assert f["target_function"] == 0, "single-function Root Complex"
    assert f["bar_aperture"] == CQ_APERTURE, \
        f"BAR Aperture {f['bar_aperture']} != {CQ_APERTURE} (4 KB)"
    assert list(data) == list(payload), \
        f"payload {[hex(w) for w in data]} != {[hex(w) for w in payload]}"
    assert cq.drops == [], f"a deliverable write must not strobe a drop: {cq.drops}"


@cocotb.test()
async def f1_inbound_memrd_reaches_cq(dut):
    """A read becomes a descriptor-only CQ packet.

    For a Memory Read the Dword Count is the size to be read (PG213, Table
    52), so the descriptor carries a non-zero count with no payload behind
    it. That asymmetry with the write test is the point of having both.
    """
    rc, completer = await init(dut)
    cq = CqWatch(dut)
    cq.start()

    tag = 0x42
    await inject_rx(dut, memrd_tlp(tag=tag, address=BAR0_ADDRESS, length_dw=1))
    await cq.wait_packets(1)

    desc, data = cq.packets[0]
    f = decode_cq_desc(desc)
    assert f["req_type"] == CQ_MEM_READ, \
        f"Request Type {f['req_type']:#06b} != CQ_MEM_READ"
    assert f["dword_count"] == 1, f"Dword Count {f['dword_count']} != 1"
    assert data == [], f"a read carries no payload, saw {[hex(w) for w in data]}"
    assert f["tag"] == tag and f["requester_id"] == DEVICE_RID
    assert cq.drops == []


@cocotb.test()
async def f1_cq_tuser_carries_byte_enables(dut):
    """first_be / last_be reach the host on m_axis_cq_tuser.

    PG213, Table 10 puts first_be[3:0] at tuser[3:0] and last_be[3:0] at
    tuser[7:4]; pcie_cq_if drives them on the first beat. PCIe Base Spec
    r2.1, §2.2.5 defines the fields themselves.

    The two writes carry different byte enables, so a module that hardwired
    tuser to one value, or left it 0, cannot pass both checks. 0xF/0xF and
    0x3/0xC share no nibble between the two writes.
    """
    rc, completer = await init(dut)
    cq = CqWatch(dut)
    cq.start()

    await inject_rx(dut, memwr_tlp(tag=0x43, payload=(1, 2),
                                   first_be=0xF, last_be=0xF))
    await cq.wait_packets(1)
    assert (cq.tusers[0] & 0xF) == 0xF, \
        f"tuser first_be {cq.tusers[0] & 0xF:#06b} != 0b1111"
    assert ((cq.tusers[0] >> 4) & 0xF) == 0xF, \
        f"tuser last_be {(cq.tusers[0] >> 4) & 0xF:#06b} != 0b1111"

    await inject_rx(dut, memwr_tlp(tag=0x44, payload=(3, 4),
                                   first_be=0x3, last_be=0xC))
    await cq.wait_packets(2)
    assert (cq.tusers[1] & 0xF) == 0x3, \
        f"tuser first_be {cq.tusers[1] & 0xF:#06b} != 0b0011"
    assert ((cq.tusers[1] >> 4) & 0xF) == 0xC, \
        f"tuser last_be {(cq.tusers[1] >> 4) & 0xF:#06b} != 0b1100"


@cocotb.test()
async def f1_unsupported_inbound_strobes_cq_dropped(dut):
    """An I/O request the host cannot serve is reported on cq_dropped_o.

    Nothing is delivered to the host, because pcie_cq_if delivers only Memory
    requests, but the request must not vanish either: it raises cq_dropped_o
    with CQ_DROP_UNSUPPORTED. Its UR Completion is checked by
    a4_inbound_io_returns_ur.

    The check is on cq_dropped_o, not on rx_error_valid_o: tlp_parser strobes
    rx_error_valid_o for Messages already, so a check written against it
    would say nothing about the completer path.
    """
    rc, completer = await init(dut)
    cq = CqWatch(dut)
    cq.start()

    await inject_rx(dut, iord_tlp(tag=0x45))
    await cq.wait_drops(1)

    assert cq.drops == [CQ_DROP_UNSUPPORTED], \
        f"expected CQ_DROP_UNSUPPORTED ({CQ_DROP_UNSUPPORTED}), saw {cq.drops}"
    assert cq.packets == [], \
        "an unsupported request must not be delivered to the host as a CQ packet"


@cocotb.test()
async def f1_memory_outside_every_bar_is_dropped_not_delivered(dut):
    """A Memory request outside the accept window is reported, not delivered.

    Delivering it would hand the host an address outside its aperture, and
    dropping it silently would hide it. It raises cq_dropped_o with
    CQ_DROP_NO_BAR, a code separate from the unsupported-type case.

    The address is OUT_OF_APERTURE_ADDRESS (4 GB), the first address above
    the default host aperture. An address at or above 4 GB needs the 64-bit
    format, so this is a 4DW request; see memrd64_tlp. If the aperture is
    ever made to cover 4 GB, this test must move with
    OUT_OF_APERTURE_ADDRESS.
    """
    rc, completer = await init(dut)
    cq = CqWatch(dut)
    cq.start()

    await inject_rx(dut, memrd64_tlp(tag=0x46, address=OUT_OF_APERTURE_ADDRESS))
    await cq.wait_drops(1)

    assert cq.drops == [CQ_DROP_NO_BAR], (
        f"expected CQ_DROP_NO_BAR ({CQ_DROP_NO_BAR}), saw {cq.drops} -- if this "
        f"is a format error the Mem64 header was built wrong, not the aperture")
    assert cq.packets == []


@cocotb.test()
async def f1_no_inbound_request_is_silently_discarded(dut):
    """No inbound request is silently discarded.

    For a mixed batch of inbound requests, deliverable and not, every one
    must produce exactly one of a CQ packet or a cq_dropped_o strobe: never
    neither, which would be a silent discard, and never both, which would
    report it twice.

    The check is a count identity over the whole batch rather than a
    per-case assertion, so a request class that starts being discarded
    fails here even if it has no test of its own. The undeliverable Memory
    entry is a 4DW request at OUT_OF_APERTURE_ADDRESS, which gives the
    expected split of two deliverable and three undeliverable requests.
    """
    rc, completer = await init(dut)
    cq = CqWatch(dut)
    cq.start()

    batch = [
        memrd_tlp(tag=0x50, address=BAR0_ADDRESS),            # deliverable
        memwr_tlp(tag=0x51, address=BAR0_ADDRESS + 0x40),     # deliverable
        iord_tlp(tag=0x52),                                   # unsupported
        cfgrd0_tlp(tag=0x53),                                 # unsupported
        memrd64_tlp(tag=0x54,                                 # outside the window
                    address=OUT_OF_APERTURE_ADDRESS),
    ]
    for tlp in batch:
        await inject_rx(dut, tlp)
        await settle(dut, 40)
    await settle(dut, 200)

    accounted = len(cq.packets) + len(cq.drops)
    assert accounted == len(batch), (
        f"{len(batch)} inbound requests, but only {accounted} accounted for "
        f"({len(cq.packets)} CQ packets + {len(cq.drops)} drop strobes) -- "
        "an unaccounted request is §41.1 A4")
    assert len(cq.packets) == 2, \
        f"exactly the two BAR-matching Memory requests are deliverable, saw {len(cq.packets)}"
    assert len(cq.drops) == 3, \
        f"exactly three requests are undeliverable, saw {len(cq.drops)}"


# ---------------------------------------------------------------------------
# CC path
# ---------------------------------------------------------------------------
# pcie_cc_if turns a host CC packet into tlp_layer's completion request,
# tlp_completion_generator builds the Cpl or CplD from it, and tlp_generator
# emits that on the TX stream; pcie_cc_if also synthesises the UR Completions
# pcie_cq_if asks for. The bench builds CC descriptors from PG213, Table 58
# (cc_desc) and checks each emitted Completion header against PCIe Base Spec
# r2.1, §2.2.9 (decode_cpl), so the tests check the mapping between the two
# documents. Tests that check the Completer ID drive completer_id_i with
# COMPLETER first.

def cc_desc(status, byte_count, lower_address, requester_id, tag,
            dword_count, tc=0, attr=0, force_ecrc=0, completer_id_enable=1):
    """PG213 Table 58 -- the 96-bit Completer Completion descriptor."""
    v = lower_address & 0x7F
    v |= (byte_count & 0x1FFF) << 16
    v |= (dword_count & 0x7FF) << 32
    v |= (status & 0x7) << 43
    v |= (requester_id & 0xFFFF) << 48
    v |= (tag & 0xFF) << 64
    v |= (completer_id_enable & 1) << 88
    v |= (tc & 0x7) << 89
    v |= (attr & 0x7) << 92
    v |= (force_ecrc & 1) << 95
    return v


async def send_cc(dut, desc, payload=(), limit=4000):
    """Drive one CC packet: 3 descriptor Dwords then payload, 128 bits a beat.

    Beat 0 carries descriptor Dwords 0..2 plus the first payload Dword, the
    Dword-aligned layout of PG213, Figure 34 for a 128-bit interface.
    """
    words = [desc & 0xFFFFFFFF, (desc >> 32) & 0xFFFFFFFF,
             (desc >> 64) & 0xFFFFFFFF] + list(payload)
    beats = []
    for i in range(0, len(words), 4):
        chunk = words[i:i + 4]
        data = 0
        keep = 0
        for j, w in enumerate(chunk):
            data |= (w & 0xFFFFFFFF) << (32 * j)
            keep |= 1 << j
        beats.append((data, keep, i + 4 >= len(words)))
    for data, keep, last in beats:
        dut.s_axis_cc_tdata.value = data
        dut.s_axis_cc_tkeep.value = keep
        dut.s_axis_cc_tlast.value = 1 if last else 0
        dut.s_axis_cc_tvalid.value = 1
        for _ in range(limit):
            await ReadOnly()
            fired = int(dut.s_axis_cc_tready.value) == 1
            await RisingEdge(dut.clk_i)
            if fired:
                break
        else:
            raise AssertionError("s_axis_cc_tready never asserted -- CC wedged")
    dut.s_axis_cc_tvalid.value = 0
    dut.s_axis_cc_tlast.value = 0


@cocotb.test()
async def f1_cc_descriptor_becomes_cpld_on_the_wire(dut):
    """The read path end to end: MemRd -> CQ -> CC -> CplD on the wire.

    The device reads, the host answers, and a Completion goes back out.
    These fields of the emitted header are checked against PCIe Base Spec
    r2.1, §2.2.9 and against the CC descriptor the bench built from PG213,
    Table 58:

      Requester ID / Tag   echoed from the request
      Completer ID         the Root Complex's own BDF, from completer_id_i,
                           not anything the host put in the descriptor
      Byte Count           bytes remaining including this Completion
      Lower Address        low 7 bits of the first byte returned
      BCM                  0; only PCI-X completers set it

    The Completer ID check shows the Transaction Layer, not the host, owns
    the Root Complex's identity: pcie_cc_if does not forward the
    descriptor's Completer Bus, Target Function or Completer ID Enable fields.
    """
    rc, completer = await init(dut)
    dut.completer_id_i.value = COMPLETER
    cq = CqWatch(dut)
    cq.start()

    tag, addr = 0x61, BAR0_ADDRESS
    await inject_rx(dut, memrd_tlp(tag=tag, address=addr, length_dw=1))
    await cq.wait_packets(1)

    # The host reads its own memory and answers. Byte Count 4, one Dword.
    value = 0x5EED_0061
    await send_cc(dut, cc_desc(status=CPL_SC, byte_count=4,
                               lower_address=addr & 0x7F,
                               requester_id=DEVICE_RID, tag=tag,
                               dword_count=1),
                  payload=(value,))
    await settle(dut, 300)

    cpls = await cpls_on_wire(completer)
    assert len(cpls) == 1, f"expected 1 CplD on the wire, saw {len(cpls)}"
    c = cpls[0]
    assert c["requester_id"] == DEVICE_RID, \
        f"Requester ID {c['requester_id']:#06x} != {DEVICE_RID:#06x}"
    assert c["tag"] == tag, f"Tag {c['tag']:#04x} != {tag:#04x}"
    assert c["completer_id"] == COMPLETER, (
        f"Completer ID {c['completer_id']:#06x} != our configured "
        f"{COMPLETER:#06x} -- the TL owns our identity, not the host")
    assert c["status"] == CPL_SC
    assert c["has_data"], "an SC Memory Read Completion carries data"
    assert c["byte_count"] == 4, f"Byte Count {c['byte_count']} != 4"
    assert c["lower_address"] == (addr & 0x7F), \
        f"Lower Address {c['lower_address']:#04x} != {addr & 0x7F:#04x}"
    assert c["bcm"] == 0, "BCM must be 0"
    assert c["payload"] == [value], \
        f"payload {[hex(w) for w in c['payload']]} != [{value:#010x}]"


@cocotb.test()
async def f1_cc_rejects_illegal_completion_status(dut):
    """The negative pair of the test above: a bad status is refused.

    PG213, Table 58 allows three Completion Status values on this interface:
    SC, UR and CA. CRS is not among them: the Root Complex receives CRS
    Completions (pcie_rc_if carries them to the host) but never sends one. A
    host that asks for CRS is refused with CC_ERR_BAD_STATUS and nothing
    goes on the wire.

    Without this test, the one above cannot tell a design that builds the
    Completion the descriptor asked for from one that builds a Completion
    whatever the descriptor said.
    """
    rc, completer = await init(dut)
    cq = CqWatch(dut)
    cq.start()

    tag = 0x62
    await inject_rx(dut, memrd_tlp(tag=tag, address=BAR0_ADDRESS, length_dw=1))
    await cq.wait_packets(1)

    await send_cc(dut, cc_desc(status=CPL_CRS, byte_count=4, lower_address=0,
                               requester_id=DEVICE_RID, tag=tag, dword_count=0))
    await settle(dut, 200)

    assert cq.cc_errors == [CC_ERR_BAD_STATUS], (
        f"expected one CC_ERR_BAD_STATUS ({CC_ERR_BAD_STATUS}) strobe, saw "
        f"{cq.cc_errors} -- the refusal must be REPORTED, not silent")
    cpls = await cpls_on_wire(completer)
    assert cpls == [], \
        f"a CRS Completion must never be originated by a Root Complex, saw {cpls}"


# ---------------------------------------------------------------------------
# Posted drops and UR interleaving
# ---------------------------------------------------------------------------
# Two boundaries between pcie_cq_if's drop path and pcie_cc_if's UR path.
# The first test drops a posted request: pcie_cq_if's offered_non_posted
# keeps a dropped Memory Write from being offered for a UR, and the test
# fails if that term is forced true. The second has a host CC packet in
# flight while a UR is pending: pcie_cc_if's S_DESC state takes the UR only
# at dw_idx_r == 0, between host packets, so a synthesised Completion never
# lands inside the host's descriptor.

@cocotb.test()
async def f1_dropped_posted_write_gets_no_completion(dut):
    """A dropped posted request is reported but never completed.

    A Memory Write is a Posted Request and needs no Completion (PCIe Base
    Spec r2.1, §2.4.1 and §2.2.9). So an undeliverable MemWr must raise
    cq_dropped_o and put nothing on the wire, while an undeliverable MemRd,
    which is non-posted, must also get a UR Completion.

    The pair is the point. A check that only the write produces no
    Completion would also pass against a design that completes nothing; the
    read arm in the same test is what makes the absence meaningful. Both
    arms use one address, OUT_OF_APERTURE_ADDRESS (4DW requests at 4 GB), so
    the read controls for the write; they must keep sharing it.
    """
    rc, completer = await init(dut)
    cq = CqWatch(dut)
    cq.start()

    # --- posted: a Memory Write outside the accept window ---
    await inject_rx(dut, memwr64_tlp(tag=0x70, address=OUT_OF_APERTURE_ADDRESS,
                                     payload=(0xBADD_0001,), first_be=0xF))
    await cq.wait_drops(1)
    await settle(dut, 200)

    assert cq.drops == [CQ_DROP_NO_BAR], \
        f"expected CQ_DROP_NO_BAR for the undeliverable write, saw {cq.drops}"
    assert cq.packets == [], "an out-of-window write must not be delivered"
    cpls = await cpls_on_wire(completer)
    assert cpls == [], (
        f"a POSTED request must never be completed (Base 2.1 §2.1.2 p. 55), "
        f"but {len(cpls)} Completion(s) went out: {cpls}")

    # --- non-posted control, same address, same drop reason, opposite obligation ---
    await inject_rx(dut, memrd64_tlp(tag=0x71, address=OUT_OF_APERTURE_ADDRESS))
    await cq.wait_drops(2)
    await settle(dut, 200)

    cpls = await cpls_on_wire(completer)
    assert len(cpls) == 1, (
        f"the non-posted read at the same bad address IS owed a UR Completion, "
        f"saw {len(cpls)} -- if this is 0 the design has stopped completing "
        f"everything and the posted assertion above proved nothing")
    assert cpls[0]["status"] == CPL_UR and cpls[0]["tag"] == 0x71


@cocotb.test()
async def f1_ur_does_not_corrupt_a_concurrent_host_completion(dut):
    """A synthesised UR never interleaves into a host descriptor.

    pcie_cc_if lets a pending UR preempt the host's CC stream, but only at
    dw_idx_r == 0, a packet boundary. Without that guard the UR could be
    taken after one or two Dwords of the host's descriptor were consumed;
    collection would then resume at the wrong Dword, and the host's
    Completion would go out with mangled fields.

    The window is the two cycles in which the descriptor is half collected,
    and the bench cannot aim at it directly, so the arrival order is swept.
    Each iteration starts an I/O read, which becomes the pending UR once
    tlp_parser has taken its three Dwords, and starts the host's CC answer
    `offset` cycles later, moving the rise of ur_valid_i across the host
    packet.

    init() is called once for the whole sweep, because each call starts
    another Clock on clk_i. Every iteration checks the host CplD's Requester
    ID, Tag, Completer ID, Byte Count and payload, and that the UR still
    appears, so a corruption of those fields at any offset fails the test.
    """
    rc, completer = await init(dut)
    dut.completer_id_i.value = COMPLETER
    cq = CqWatch(dut)
    cq.start()

    seen = 0
    for offset in range(16):
        tag = 0x80 + offset
        addr = BAR0_ADDRESS
        # length_dw > 1 requires both byte enables non-zero (PCIe Base Spec
        # r2.1, §2.2.5), and tlp_validator enforces it, or the read is
        # rejected as malformed and no CQ packet is ever produced.
        await inject_rx(dut, memrd_tlp(tag=tag, address=addr, length_dw=4,
                                       first_be=0xF, last_be=0xF))
        await cq.wait_packets(len(cq.packets) + 1)

        payload = tuple(0x1234_0000 | (offset << 8) | i for i in range(4))
        io_task = cocotb.start_soon(inject_rx(dut, iord_tlp(tag=0xF0 + offset)))
        for _ in range(offset):
            await RisingEdge(dut.clk_i)
        await send_cc(dut, cc_desc(status=CPL_SC, byte_count=16,
                                   lower_address=addr & 0x7F,
                                   requester_id=DEVICE_RID, tag=tag,
                                   dword_count=4),
                      payload=payload)
        await io_task
        await settle(dut, 400)

        cpls = await cpls_on_wire(completer)
        fresh = cpls[seen:]
        seen = len(cpls)
        sc = [c for c in fresh if c["status"] == CPL_SC]
        ur = [c for c in fresh if c["status"] == CPL_UR]

        dut._log.info(f"DIAG offset={offset} fresh={len(fresh)} "
                      + " | ".join(
                          f"st={x['status']} rid={x['requester_id']:#06x} "
                          f"cid={x['completer_id']:#06x} tag={x['tag']:#04x} "
                          f"len={x['length_dw']} bc={x['byte_count']} "
                          f"pl={[hex(w) for w in x['payload']]}" for x in fresh))
        assert len(sc) == 1, (
            f"offset {offset}: expected exactly 1 SC Completion for the host's "
            f"answer, saw {len(sc)} -- a UR taken mid-descriptor corrupts it")
        c = sc[0]
        assert c["requester_id"] == DEVICE_RID, (
            f"offset {offset}: host CplD Requester ID {c['requester_id']:#06x} "
            f"!= {DEVICE_RID:#06x} -- descriptor collection resumed at the "
            f"wrong Dword")
        assert c["tag"] == tag, \
            f"offset {offset}: host CplD Tag {c['tag']:#04x} != {tag:#04x}"
        assert c["completer_id"] == COMPLETER, \
            f"offset {offset}: Completer ID {c['completer_id']:#06x} corrupted"
        assert c["byte_count"] == 16, \
            f"offset {offset}: Byte Count {c['byte_count']} != 16"
        assert c["payload"] == list(payload), (
            f"offset {offset}: host payload {[hex(w) for w in c['payload']]} "
            f"!= {[hex(w) for w in payload]}")
        assert len(ur) == 1, \
            f"offset {offset}: the I/O read is still owed its UR, saw {len(ur)}"


# ---------------------------------------------------------------------------
# RCB splitting
# ---------------------------------------------------------------------------
# tlp_completion_generator splits one host completion into CplDs that neither
# cross a Read Completion Boundary nor exceed MPS (completion_segment).
# init() sets RCB to 64 bytes (rcb_128b_i = 0; a Root Complex's RCB is 64 or
# 128 bytes) and MPS to 128 bytes. The goldens are derived by hand from PCIe
# Base Spec r2.1, §2.3.1.1 (a split happens only at naturally aligned RCB
# boundaries, and Byte Count is the bytes remaining including this
# Completion) and PG213, Table 58 (Lower Address is the low 7 bits of this
# Completion's first byte), never read back from the DUT.

async def read_and_answer(dut, cq, completer, tag, address, total_bytes):
    """Inbound MemRd of `total_bytes`, answered by the host in one CC packet.

    The host hands over a single logical completion (status, total Byte
    Count, starting Lower Address, whole payload) and the Transaction Layer
    decides how many CplDs that becomes. Returns the payload it sent, so the
    caller can check the split preserved it end to end. `completer` is not
    used.
    """
    n_dw = total_bytes // 4
    await inject_rx(dut, memrd_tlp(tag=tag, address=address, length_dw=n_dw,
                                   first_be=0xF, last_be=0xF))
    await cq.wait_packets(len(cq.packets) + 1)
    payload = tuple(0xCB00_0000 | i for i in range(n_dw))
    await send_cc(dut, cc_desc(status=CPL_SC, byte_count=total_bytes,
                               lower_address=address & 0x7F,
                               requester_id=DEVICE_RID, tag=tag,
                               dword_count=n_dw),
                  payload=payload)
    await settle(dut, 600)
    return payload


def check_split(cpls, expected, tag, payload):
    """Assert the emitted CplD sequence matches a hand-derived golden.

    `expected` is [(length_dw, byte_count, lower_address), ...] in emission
    order.  Also checks the payload is partitioned across the segments in order
    with nothing lost, duplicated or reordered -- a split that got the headers
    right and the data wrong would otherwise pass.
    """
    assert len(cpls) == len(expected), (
        f"expected {len(expected)} Completion(s) for this request, saw "
        f"{len(cpls)}: {[(c['length_dw'], c['byte_count'], c['lower_address']) for c in cpls]}")
    seen = []
    for i, (c, (elen, ebc, ela)) in enumerate(zip(cpls, expected)):
        assert c["status"] == CPL_SC, f"segment {i}: status {c['status']} != SC"
        assert c["tag"] == tag, f"segment {i}: Tag {c['tag']:#04x} != {tag:#04x}"
        assert c["requester_id"] == DEVICE_RID, \
            f"segment {i}: Requester ID {c['requester_id']:#06x} != {DEVICE_RID:#06x}"
        assert c["length_dw"] == elen, \
            f"segment {i}: Length {c['length_dw']} DW != {elen} DW"
        assert c["byte_count"] == ebc, (
            f"segment {i}: Byte Count {c['byte_count']} != {ebc} -- PG213 Table 58 "
            f"wants the bytes REMAINING including this Completion")
        assert c["lower_address"] == ela, (
            f"segment {i}: Lower Address {c['lower_address']} != {ela} -- Base 2.1 "
            f"§2.3.1.1 p. 112 aligns segments to the RCB grid, not to the transfer start")
        seen.extend(c["payload"])
    assert seen == list(payload), (
        f"the split lost, duplicated or reordered payload:\n  got  {[hex(w) for w in seen]}\n"
        f"  want {[hex(w) for w in payload]}")


@cocotb.test()
async def cc_multi_rcb_split_aligned(dut):
    """128 B from an RCB-aligned start splits into two 64 B Completions.

    With RCB = 64, a 128 B read starting on a boundary is two full segments
    (PCIe Base Spec r2.1, §2.3.1.1). Byte Count counts down (128 then 64) and
    Lower Address counts up (0 then 64), as PG213, Table 58 defines them.
    """
    rc, completer = await init(dut)
    dut.completer_id_i.value = COMPLETER
    cq = CqWatch(dut)
    cq.start()

    tag, addr = 0x90, BAR0_ADDRESS          # 0x100 -> lower_address 0, aligned
    payload = await read_and_answer(dut, cq, completer, tag, addr, 128)
    check_split(await cpls_on_wire(completer),
                [(16, 128, 0), (16, 64, 64)], tag, payload)


@cocotb.test()
async def cc_multi_rcb_split_unaligned(dut):
    """128 B starting 16 B into an RCB splits 48 / 64 / 16.

    This is the test that separates the rules. An implementation that splits
    every 64 bytes from the start of the transfer produces 64/64 here and
    still passes cc_multi_rcb_split_aligned. Only a segment clamped to the
    distance to the next naturally aligned RCB boundary yields 48 first
    (PCIe Base Spec r2.1, §2.3.1.1).

    It is also the only test whose first segment is neither MPS nor a full
    RCB, and the only one whose final Lower Address wraps: 16+48+64 = 128,
    truncated to 7 bits = 0.
    """
    rc, completer = await init(dut)
    dut.completer_id_i.value = COMPLETER
    cq = CqWatch(dut)
    cq.start()

    tag, addr = 0x91, BAR0_ADDRESS + 16     # lower_address 16
    payload = await read_and_answer(dut, cq, completer, tag, addr, 128)
    check_split(await cpls_on_wire(completer),
                [(12, 128, 16), (16, 80, 64), (4, 16, 0)], tag, payload)


@cocotb.test()
async def cc_within_rcb_does_not_split(dut):
    """The negative pair: a read that fits inside one RCB is one Completion.

    Without this test the two split tests cannot tell a design that splits
    at the RCB boundary from one that always splits. 64 B from an aligned
    start exactly fills one RCB and must not be divided.
    """
    rc, completer = await init(dut)
    dut.completer_id_i.value = COMPLETER
    cq = CqWatch(dut)
    cq.start()

    tag, addr = 0x92, BAR0_ADDRESS
    payload = await read_and_answer(dut, cq, completer, tag, addr, 64)
    check_split(await cpls_on_wire(completer), [(16, 64, 0)], tag, payload)


# ---------------------------------------------------------------------------
# Transmit ordering
# ---------------------------------------------------------------------------
# tlp_control arbitrates between the requester's headers (host RQ) and the
# completion generator's headers (host CC and synthesised UR) for the one
# tlp_generator. It alternates through prefer_completion_r, and while a
# Memory Write header is pending it grants a Completion only if the
# Completion's Relaxed Ordering bit is set (PCIe Base Spec r2.1, §2.4.1,
# Table 2-33, entries D2a and D2b). The first test checks the rule, the
# second that the Relaxed Ordering exception is honoured, and the third that
# two Memory Writes keep their issue order. Each reads the TX order from
# completer.seen, starting at the index recorded before its stimulus.
@cocotb.test()
async def ordering_completion_behind_posted(dut):
    """A Completion must not pass a queued Posted Request.

    PCIe Base Spec r2.1, §2.4.1, Table 2-33: Row D (Read Completion) against
    Col 2 (Posted Request) is "No" when Relaxed Ordering is clear.

    A violation needs a cycle in which the requester header and the
    completion header are both valid, prefer_completion_r is 1 and locked_r
    is clear. A posted MemWr as the first request cannot set that up:
    tlp_control holds locked_r through its payload, and tlp_requester
    streams that payload before it can present the next header, so the
    second write and the Completion never contend in one cycle.

    The first request is therefore a Memory Read, which has no payload. Its
    grant sets prefer_completion_r to 1 and keeps tlp_generator busy while it
    is emitted, and tlp_requester is then free to present the next header:

        RQ MemRd (first)    -> granted; prefer_completion_r <- 1; busy
        RQ MemWr            -> header pending, waiting for tlp_generator
        CC CplD             -> header pending too
        MemRd done          -> both valid, prefer = 1: the posted-pending
                               term must hold the CplD back

    The overlap lasts a few cycles, so the CC arrival is swept, and the test
    checks the order on every iteration.
    """
    rc, completer = await init(dut)
    dut.completer_id_i.value = COMPLETER
    cq = CqWatch(dut)
    cq.start()

    for offset in range(6):
        tag = 0xA0 + offset
        await inject_rx(dut, memrd_tlp(tag=tag, address=BAR0_ADDRESS, length_dw=1))
        await cq.wait_packets(len(cq.packets) + 1)

        base = len(completer.seen)

        # The first request: a non-posted read. Its grant sets
        # prefer_completion_r to 1 and leaves the requester free.
        await send_rq(dut, [(rq_desc(RQ_MEM_READ, 1, address=0x2000),
                             0xF, True, tuser(0xF, 0x0))])
        # The posted write whose ordering is under test. It is issued before
        # the completion, so the spec requires it on the wire first.
        w = cocotb.start_soon(send_rq(dut, [
            (rq_desc(RQ_MEM_WRITE, 1, address=0x2100 + 0x10 * offset),
             0xF, False, tuser(0xF, 0x0)),
            (0xA1A1_0000 | offset, 0x1, True, 0)]))
        for _ in range(offset):
            await RisingEdge(dut.clk_i)
        c = cocotb.start_soon(send_cc(dut, cc_desc(
            status=CPL_SC, byte_count=4, lower_address=BAR0_ADDRESS & 0x7F,
            requester_id=DEVICE_RID, tag=tag, dword_count=1),
            payload=(0x5EED_0000 | offset,)))
        await w
        await c
        await settle(dut, 400)

        order = []
        for r in completer.seen[base:]:
            if decode_cpl(r.dwords):
                order.append("CPL")
            elif (r.dwords[0] & 0x1F) == TYPE_MEM:
                order.append("MEMWR" if (r.dwords[0] >> 5) & 0b010 else "MEMRD")
        assert "MEMWR" in order and "CPL" in order, \
            f"offset {offset}: premise -- both must reach the wire, saw {order}"
        assert order.index("MEMWR") < order.index("CPL"), (
            f"offset {offset}: Base 2.1 Table 2-33 Row D / Col 2 -- a Completion "
            f"must not pass a queued Posted Request. Wire order was {order}; the "
            f"MemWr was issued first and the Completion overtook it.")


@cocotb.test()
async def ordering_ro_completion_may_pass_posted(dut):
    """A Completion with Relaxed Ordering set may pass a queued Posted Request.

    Table 2-33 entry D2b permits a Completion with RO set to pass a Posted
    Request (PCIe Base Spec r2.1, §2.4.1). It is a permission, not a
    requirement: blocking such a Completion is equally conformant. This test
    checks the design's choice to honour it and is not a conformance check.

    It fails if tlp_control holds back every Completion while a Memory Write
    header is pending, whatever its RO bit. ordering_completion_behind_posted
    passes against such a design, so only the pair separates a correct
    arbiter from one that blocks everything.

    A bit misplaced the same way in cc_desc and in tlp_control would still
    pass, so the attribute path is listed here:
      cc_desc(attr=) puts it at descriptor bits [94:92]
      cc_descriptor_t (pcie_rq_rc_pkg) names bit 93 RO, as PG213, Table 58
      pcie_cc_if copies desc_r.attr into the header's attributes unchanged
      tlp_generator packs attributes[1:0] into dw0[21:20]; attributes[1] is RO
      tlp_control's RO exception reads attributes[1]
    So attr=0b010 sets Relaxed Ordering and nothing else.
    """
    rc, completer = await init(dut)
    dut.completer_id_i.value = COMPLETER
    cq = CqWatch(dut)
    cq.start()

    cpl_first = 0
    for offset in range(6):
        tag = 0xB0 + offset
        await inject_rx(dut, memrd_tlp(tag=tag, address=BAR0_ADDRESS, length_dw=1))
        await cq.wait_packets(len(cq.packets) + 1)

        base = len(completer.seen)

        # The same set-up as ordering_completion_behind_posted: a non-posted
        # read without data goes first, because a posted first request holds
        # locked_r and keeps the requester busy, so nothing would contend.
        await send_rq(dut, [(rq_desc(RQ_MEM_READ, 1, address=0x3000),
                             0xF, True, tuser(0xF, 0x0))])
        w = cocotb.start_soon(send_rq(dut, [
            (rq_desc(RQ_MEM_WRITE, 1, address=0x3100 + 0x10 * offset),
             0xF, False, tuser(0xF, 0x0)),
            (0xB1B1_0000 | offset, 0x1, True, 0)]))
        for _ in range(offset):
            await RisingEdge(dut.clk_i)
        # The only difference from ordering_completion_behind_posted:
        # Relaxed Ordering set.
        c = cocotb.start_soon(send_cc(dut, cc_desc(
            status=CPL_SC, byte_count=4, lower_address=BAR0_ADDRESS & 0x7F,
            requester_id=DEVICE_RID, tag=tag, dword_count=1, attr=0b010),
            payload=(0x5EED_0000 | offset,)))
        await w
        await c
        await settle(dut, 400)

        order = []
        for r in completer.seen[base:]:
            if decode_cpl(r.dwords):
                order.append("CPL")
            elif (r.dwords[0] & 0x1F) == TYPE_MEM:
                order.append("MEMWR" if (r.dwords[0] >> 5) & 0b010 else "MEMRD")
        assert "MEMWR" in order and "CPL" in order, \
            f"offset {offset}: premise -- both must reach the wire, saw {order}"
        if order.index("CPL") < order.index("MEMWR"):
            cpl_first += 1

    # Not every offset: where the two never contend, the write legitimately
    # goes first, and demanding CPL-first everywhere would assert a race
    # rather than a rule. The claim is that the exception is honoured
    # somewhere in the sweep, which is false if RO is ignored and the
    # Completion is always held back.
    assert cpl_first > 0, (
        "in 6 swept offsets an RO-set Completion never once passed the queued "
        "posted write.  Base 2.1 Table 2-33 D2b permits it to, and Decision 3 "
        "says this design honours that permission -- so the posted-aware term "
        "added for F-2 is over-blocking: it is gating on the pending posted "
        "header without excepting Relaxed Ordering")

    dut._log.info(
        f"RO exception honoured on {cpl_first} of 6 offsets (unfixed tree: "
        "every Completion passes, so this row only becomes discriminating "
        "once tlp_control is posted-aware)")


@cocotb.test()
async def ordering_posted_does_not_pass_posted(dut):
    """Two Memory Writes must reach the wire in issue order.

    A Posted Request with Relaxed Ordering clear must not pass another Posted
    Request (PCIe Base Spec r2.1, §2.4.1, Table 2-33, entry A2a); the table
    ties this strong write ordering to the Producer-Consumer model.

    The order holds because of the datapath's shape, not an ordering
    decision. tlp_control never sees two posted headers at once:
    tlp_requester accepts one command at a time and presents the second
    write's header only after the first has streamed its payload. Anything
    that lets two posted headers be pending together (a bypass path, a
    second port, a reorder buffer, or an arbiter that queues instead of
    blocking) removes that guarantee without any visible change, and this
    test checks the order directly.

    The writes are told apart by address, read from header Dword 2, not by
    count: a test that only counted packets would pass under any permutation.
    """
    rc, completer = await init(dut)
    dut.completer_id_i.value = COMPLETER
    cq = CqWatch(dut)
    cq.start()

    ADDRS = [0x4000, 0x4010, 0x4020, 0x4030]
    base = len(completer.seen)

    for i, a in enumerate(ADDRS):
        await send_rq(dut, [
            (rq_desc(RQ_MEM_WRITE, 1, address=a), 0xF, False, tuser(0xF, 0x0)),
            (0xC0C0_0000 | i, 0x1, True, 0)])
    await settle(dut, 600)

    seen = []
    for r in completer.seen[base:]:
        if (r.dwords[0] & 0x1F) == TYPE_MEM and (r.dwords[0] >> 5) & 0b010:
            seen.append(r.dwords[2])

    # All four must have reached the wire. The order check below would also
    # fail otherwise; this one reports a missing write apart from a reordered
    # one.
    assert len(seen) == len(ADDRS), (
        f"expected {len(ADDRS)} Memory Writes on the wire, saw {len(seen)}: "
        f"{[hex(x) for x in seen]} -- the ordering claim below is only "
        "meaningful over the complete set")
    assert seen == ADDRS, (
        f"Memory Writes were issued to {[hex(a) for a in ADDRS]} but reached "
        f"the wire as {[hex(x) for x in seen]}.  Base 2.1 Table 2-33 Row A / "
        "Col 2 a): a Posted Request with RO clear must not pass another Posted "
        "Request.  Something now lets two posted headers be in flight at once "
        "-- tlp_requester's serial behaviour was the only thing enforcing this")

    dut._log.info(
        f"posted-vs-posted order preserved across {len(ADDRS)} writes "
        "(structural: tlp_requester is serial, so the arbiter never sees two)")


# ---------------------------------------------------------------------------
# The host accept window
# ---------------------------------------------------------------------------
# An inbound Memory request from a device is DMA into host memory. A Root
# Port decides whether to claim a request as a virtual PCI-to-PCI bridge
# would, from its configuration (PCIe Base Spec r2.1, §2.3.1, implementation
# note on requests terminated as Unsupported Requests). Here the decision is
# a fixed host aperture (HOST_MEM_BASE, HOST_MEM_SIZE) that pcie_rq_rc_top
# passes to tlp_layer as BAR 0 and tlp_layer forwards to tlp_bar_decoder.
# These tests check a write to host memory, the window's upper edge, and a
# request crossing a 4 KB boundary. No test sends a 4DW request inside the
# window: at the default the whole window lies below 4 GB, where
# tlp_validator rejects the 64-bit format.
@cocotb.test()
async def f2_memwr_to_host_address_is_delivered_on_cq(dut):
    """A device's DMA write to host memory must reach CQ, not be dropped.

    An inbound Memory Write from an Endpoint targets host memory, so it is
    judged against the host aperture rather than an Endpoint-style BAR, and
    an address inside the aperture is delivered on CQ with its address, tag
    and payload intact.
    """
    rc, completer = await init(dut)
    cq = CqWatch(dut)
    cq.start()

    # A host address well above the first 4 KB, inside the default 4 GB
    # host aperture.
    HOST_ADDRESS = 0x8000_0000
    await inject_rx(dut, memwr_tlp(tag=0x80, address=HOST_ADDRESS,
                                   payload=(0xD00D_0001,), first_be=0xF))
    await cq.wait_packets(1)
    await settle(dut, 200)

    # --- the claim: delivered, not judged against a BAR table ---
    assert cq.drops == [], (
        f"an upstream Memory Write to host address {HOST_ADDRESS:#x} was "
        f"dropped ({cq.drops}) instead of being delivered on CQ.  A Root "
        "Complex does not own BARs in the upstream direction -- this is DMA "
        "into host memory, and the BAR decode has no jurisdiction over it.  "
        "Since Stage F-3 the RC is built with a host aperture "
        "(HOST_MEM_BASE/HOST_MEM_SIZE) rather than tlp_layer's BAR defaults, "
        "so a drop here means the aperture is not reaching tlp_bar_decoder")

    desc, data = cq.packets[0]
    f = decode_cq_desc(desc)
    assert f["address"] == HOST_ADDRESS, \
        f"CQ Address {f['address']:#x} != {HOST_ADDRESS:#x}"
    assert f["tag"] == 0x80, f"Tag {f['tag']:#04x} != 0x80"
    assert list(data) == [0xD00D_0001], (
        f"payload {[hex(x) for x in data]} != [0xd00d0001] -- the write was "
        "delivered and its data was not")


# The last Dword-aligned address inside a 4 GB window based at 0. A 1-Dword
# request here ends at 0xFFFF_FFFF, still inside; one Dword further is 4 GB
# and needs the 64-bit format, which is OUT_OF_APERTURE_ADDRESS.
HOST_APERTURE_LAST_DWORD = 0xFFFF_FFFC


@cocotb.test()
async def f3_aperture_edge_pair(dut):
    """The accept window's edge, checked from both sides through one path.

    Arm A: the last Dword inside the window is delivered on CQ.
    Arm B: the first address outside it is dropped with CQ_DROP_NO_BAR.

    The pairing is the point. Arm A alone cannot tell a correct window from
    one that accepts everything (a BAR_MASK of all zeros passes it), and arm
    B alone cannot tell it from one that accepts nothing. Both arms run
    through the same inject_rx -> tlp_parser -> tlp_bar_decoder ->
    pcie_cq_if path, so the test shows the window both accepting and
    rejecting on one build.

    Arm B is a 4DW request because 4 GB does not fit a 32-bit address; see
    memrd64_tlp. Arm A is 3DW because 0xFFFF_FFFC fits, and the 64-bit form
    there would be rejected as malformed. If the aperture is ever made to
    cover 4 GB, arm B must move with OUT_OF_APERTURE_ADDRESS.
    """
    rc, completer = await init(dut)
    cq = CqWatch(dut)
    cq.start()

    # --- arm A: inside, and as close to the edge as a Dword can sit ---
    payload = (0xEDA0_0001,)
    await inject_rx(dut, memwr_tlp(tag=0x90, address=HOST_APERTURE_LAST_DWORD,
                                   payload=payload, first_be=0xF))
    await cq.wait_packets(1)

    assert cq.drops == [], (
        f"the last Dword inside the window was dropped ({cq.drops}) instead of "
        f"delivered -- at {HOST_APERTURE_LAST_DWORD:#x} the window's upper edge "
        "is off by at least one Dword, or the aperture was never passed down")
    desc, data = cq.packets[0]
    f = decode_cq_desc(desc)
    assert f["address"] == HOST_APERTURE_LAST_DWORD, \
        f"CQ Address {f['address']:#x} != {HOST_APERTURE_LAST_DWORD:#x}"
    assert list(data) == list(payload), \
        f"payload {[hex(x) for x in data]} != {[hex(x) for x in payload]}"

    # --- arm B: the first address outside, one Dword further on ---
    await inject_rx(dut, memrd64_tlp(tag=0x91, address=OUT_OF_APERTURE_ADDRESS))
    await cq.wait_drops(1)

    assert cq.drops == [CQ_DROP_NO_BAR], (
        f"the first address outside the window must be dropped with "
        f"CQ_DROP_NO_BAR, saw {cq.drops} -- if this is empty the window has no "
        "upper edge at all and arm A above proved nothing")
    assert len(cq.packets) == 1, (
        f"an out-of-window request was delivered on CQ ({len(cq.packets)} "
        "packets total, expected only arm A's)")


@cocotb.test()
async def f3_memwr_crossing_4kb_boundary_is_accepted(dut):
    """An inbound write that crosses a 4 KB boundary is accepted.

    This records the design's behaviour, not a requirement. A request must
    not cross a 4 KB boundary, but a Receiver checks that only optionally
    (PCIe Base Spec r2.1, §2.2.7), so accepting and rejecting both conform.

    There is no explicit inbound check. tlp_bar_decoder's end_match tests
    the request's last byte against the same mask as its first, so it
    rejects a request that runs past the end of the window. A 4 KB window
    therefore rejects every 4 KB crossing; with the 4 GB host aperture this
    write is accepted. pcie_rq_if's RQ_ERR_4KB check covers the
    outbound path only. If an inbound check is ever added, this test must
    change with it, and its failure then is not a regression.
    """
    rc, completer = await init(dut)
    cq = CqWatch(dut)
    cq.start()

    # 0x0FF8 + 4 Dwords spans 0x0FF8..0x1007, crossing the 4 KB boundary at
    # 0x1000 by two Dwords. Both ends are inside a 4 GB window at 0.
    CROSSING_ADDRESS = 0x0000_0FF8
    payload = (0xC705_0001, 0xC705_0002, 0xC705_0003, 0xC705_0004)
    await inject_rx(dut, memwr_tlp(tag=0x92, address=CROSSING_ADDRESS,
                                   payload=payload, first_be=0xF, last_be=0xF))
    await cq.wait_packets(1)

    assert cq.drops == [], (
        f"the 4 KB-crossing write was dropped ({cq.drops}).  That is a "
        "CONFORMING outcome under §2.2.7 -- this row is a characterisation, so "
        "if a later rung restores the check, flip this row deliberately rather "
        "than treating the failure as a regression")
    desc, data = cq.packets[0]
    f = decode_cq_desc(desc)
    assert f["address"] == CROSSING_ADDRESS, \
        f"CQ Address {f['address']:#x} != {CROSSING_ADDRESS:#x}"
    assert list(data) == list(payload), \
        f"payload {[hex(x) for x in data]} != {[hex(x) for x in payload]}"
