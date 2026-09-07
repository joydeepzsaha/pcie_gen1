"""Commit 2a-iii -- pcie_rq_rc_top acceptance (V1..V6).

The assembled Root Complex requester surface, driven the way Commit 2b will
drive it:

    host RQ AXIS -> pcie_rq_if -> tlp_layer -> TX DLLP -> [completer]
    [completer] -> RX DLLP -> tlp_layer -> pcie_rc_if -> host RC AXIS

The two wrapper targets (verilate_rq_if / verilate_rc_if) own the cycle-accurate
cases, and the two integration targets (verilate_rq_if_tlp / verilate_rc_if_tlp)
own the on-wire goldens and the tag round trip.  This target owns the question
neither of those can answer: does the assembled thing behave like a requester
when several requests are in flight, when completions come back out of order,
and when the consumer stops consuming.  V3 and V4 are the load-bearing ones.

! FLOW CONTROL.  The DUT emits nothing, and reports NO error, until link_up_i,
transmit_enable_i and fc_initialized_i are set and at least one
fc_update_valid_i pulse has loaded non-zero credits (tlp_layer.sv:249,
tlp_credit_manager.sv:53-54, 66-83).  Every "N packets" assertion below would
otherwise be vacuously satisfied by silence.  This was regression RC1.

RTL cited (read, not assumed):
  DW0 assembly ..................... src/tlp/tlp_generator.sv, the dw0 assembly
  DW1 = {rid, tag, last_be, first_be}  src/tlp/tlp_generator.sv, the dw1 assembly
  config DW2 = {address[31:2],00} .. src/tlp/tlp_generator.sv, the dw2 assembly
  CPL parse, DW1/DW2 fields ........ src/tlp/tlp_parser.sv:163-189
  tracker match + accounting ....... src/tlp/tlp_request_tracker.sv:123-155
  Lower Address seeded 0 for
    non-memory requests ............ src/tlp/tlp_layer.sv:371-378
  RC descriptor field map .......... src/rc/pcie_rq_rc_pkg.sv, rc_descriptor_t
"""

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ReadOnly, RisingEdge

CLK_NS = 4

# The bench instantiates the DUT with TAG_COUNT = 8 (tb_pcie_rq_rc_top.sv), so
# V4 reaches tag exhaustion in a short test rather than a slow one.
TAG_COUNT = 8

# pcie_rq_rc_pkg::rq_req_type_e
RQ_MEM_READ = 0b0000
RQ_MEM_WRITE = 0b0001
RQ_CFG_READ0 = 0b1000
RQ_CFG_WRITE0 = 0b1010
# Stage D-2 Type 1 pair
RQ_CFG_READ1 = 0b1001
RQ_CFG_WRITE1 = 0b1011

# tlp_pkg::tlp_fmt_e / tlp_type_e (tlp_pkg.sv:8-27)
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

# pcie_rq_rc_pkg::rc_error_e (pcie_rq_rc_pkg.sv:190-194)
RC_ERR_ORPHAN_DATA = 3

RID = 0x1234        # the Root Complex's own requester_id_i
COMPLETER = 0x0100  # the completer's BDF: bus 1, device 0, function 0


# ==========================================================================
# Descriptor goldens -- hand-derived from PG213 v1.3 Tables 60/61 and 65,
# never read back from the DUT.
# ==========================================================================
def rq_desc(req_type, dword_count, address=0, completer_id=0, tc=0, attr=0):
    """PG213 Table 60/61 RQ descriptor.  Tag [103:96] is ignored (core-managed)."""
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
    return ((last_be & 0xF) << 4) | (first_be & 0xF)


def cfg_wire_dw2(bus, dev, fn, reg_num, ext_reg=0):
    """The config-request address DW as the generator emits it.

    {bus[31:24], device[23:19], function[18:16], ext_reg[11:8], reg[7:2], 00}
    (tlp_generator.sv, the dw2 assembly).  The BDF comes from the RQ descriptor's Completer
    ID field, NOT from the address -- which is why a config request needs
    completer_id set and why this golden carries it.
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
    """CPL DW0 as the parser reads it back (tlp_parser.sv:145-147, 150-155)."""
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


# ==========================================================================
# SS THE COMPLETER
#
# A deliberately minimal config completer: it watches the DLL-facing TX stream,
# parses each emitted request enough to know its tag and whether it wants data,
# and builds a matching Cpl/CplD to inject on RX.  It checks NOTHING about the
# request -- it is a stimulus source, not a verification model.
#
# It is meant to be REPLACED.  Joy is building a protocol-checking endpoint
# verification model (Patrick's directive, 2026-07-27) that is intended to take
# over this role.  The interface a replacement must present is small:
#
#     .start()                     spawn the TX watcher
#     .seen                        list of Request(tag, is_read, reg, ...) in
#                                  emission order, one per TLP off the wire
#     await .wait_for(n)           block until n requests have been observed
#     await .complete(req, ...)    inject one completion for that request
#
# Everything below the class is written against those four names only, so a
# swap is: import the new model, construct it instead, keep the calls.  Nothing
# in the RTL or the shim knows the completer exists.
# ==========================================================================
class Request:
    """One request TLP observed leaving the Transaction Layer."""

    def __init__(self, dwords):
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
        # A request "wants data back" iff it carried none going out.  For the
        # config requests this target issues that is exactly read vs write.
        self.is_read = (self.fmt & 0b010) == 0

    def __repr__(self):
        kind = "Rd" if self.is_read else "Wr"
        return (f"Cfg{kind}0(tag={self.tag:#04x}, reg={self.reg_num:#04x}, "
                f"len={self.length_dw}, fbe={self.first_be:#06b})")


class ConfigCompleter:
    """Minimal, swappable config completer.  See SS THE COMPLETER above."""

    def __init__(self, dut, requester_id=RID, completer_id=COMPLETER):
        self.dut = dut
        self.requester_id = requester_id
        self.completer_id = completer_id
        self.seen = []
        self._partial = []

    def start(self):
        cocotb.start_soon(self._watch_tx())

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
                    self.seen.append(Request(self._partial))
                    self._partial = []

    async def wait_for(self, count, cycles=400):
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
        A non-SC status always answers with no data, which is what a real
        completer does -- UR and CRS terminate the request.

        Byte Count must equal what the tracker still expects for an SC read
        (tlp_request_tracker.sv:127-135); it is unchecked otherwise.
        """
        has_data = req.is_read and status == CPL_SC
        if byte_count is None:
            byte_count = 4
        words = [
            cpl_dw0(has_data=has_data, length_dw=1 if has_data else 0),
            cpl_dw1(self.completer_id, status, byte_count=byte_count),
            # Lower Address is 0 for every non-Memory-Read completion, and the
            # tracker requires exactly that (tlp_layer.sv:371-378).
            cpl_dw2(self.requester_id, req.tag, lower_address=0),
        ]
        if has_data:
            words.append(0xD0000000 | req.tag if data is None else data)
        await self._inject(words)

    async def _inject(self, words):
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
class Rc:
    """Records RC packets and the error/status surface, concurrently."""

    def __init__(self, dut):
        self.dut = dut
        self.packets = []
        self._partial = []
        self.tags_presented = []
        self.rq_errors = []
        self.rc_errors = []
        self.unexpected = []
        self.command_errors = []
        # Completion Timeout sideband (tlp_request_tracker.sv).  Recorded for
        # every test, so V1..V6 assert its SILENCE via clean() below.
        self.timeouts = []
        self.lates = []

    def start(self):
        cocotb.start_soon(self._run())

    async def _run(self):
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

    async def wait_timeouts(self, count, cycles=4400):
        """Block until `count` completion-timeout strobes have been seen.

        The default 4096-cycle timeout plus one TAG_COUNT scan period is the
        real bound; 4400 gives it room without hiding a gross regression.
        """
        for _ in range(cycles):
            await RisingEdge(self.dut.clk_i)
            if len(self.timeouts) >= count:
                return
        raise AssertionError(
            f"expected {count} cpl_timeout strobes, saw {len(self.timeouts)} "
            f"({self.timeouts}) after {cycles} cycles")

    async def wait_lates(self, count, cycles=200):
        for _ in range(cycles):
            await RisingEdge(self.dut.clk_i)
            if len(self.lates) >= count:
                return
        raise AssertionError(
            f"expected {count} late_cpl strobes, saw {len(self.lates)} ({self.lates})")

    async def wait_packets(self, count, cycles=1500):
        for _ in range(cycles):
            await RisingEdge(self.dut.clk_i)
            if len(self.packets) >= count:
                return
        raise AssertionError(
            f"expected {count} RC packets, saw {len(self.packets)}")

    def clean(self, allow_timeouts=False):
        assert self.rq_errors == [], f"RQ protocol errors: {self.rq_errors}"
        assert self.rc_errors == [], f"RC protocol errors: {self.rc_errors}"
        assert self.unexpected == [], f"unexpected completions: {self.unexpected}"
        assert self.command_errors == [], f"TL command errors: {self.command_errors}"
        if not allow_timeouts:
            # Behaviour-neutrality, enforced rather than argued: no test that
            # answers its requests may trip the completion timeout.  If the
            # default CPL_TIMEOUT_CYCLES is ever lowered below what these tests
            # need, this is what says so.
            assert self.timeouts == [], \
                f"completion timeout fired for tags {self.timeouts} in a test that answers"
            assert self.lates == [], f"late completions drained: {self.lates}"


def packet_dwords(beats):
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
    """Saturate the VC0 credit pool.

    Without this the credit manager holds request_ready_o low forever
    (tlp_credit_manager.sv:53-54, 66-83) and the DUT transmits nothing, with no
    error.  This target exercises the assembled requester, not flow control --
    which has its own tb_tlp_credit_manager bench -- so the pool is held
    saturated and must never be the limiter.
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
    # Stage F-1 completer surface. Held idle-but-accepting: the host is ready
    # for CQ packets and sends no CC descriptors, which is the quiescent state
    # every pre-F-1 test implicitly assumed.
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
    for _ in range(cycles):
        await RisingEdge(dut.clk_i)


# ==========================================================================
# V1 -- CfgRd0 round trip
# ==========================================================================
@cocotb.test()
async def v1_cfgrd0_round_trip(dut):
    """RQ descriptor in -> completer returns CplD -> RC packet out.

    The base case Commit 2b's enumeration is built from: read a config
    register, get the data back, get the tag back, and get the tag released.
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


# ==========================================================================
# V2 -- byte-granular CfgWr0, the Commit-2b bus-number shape
# ==========================================================================
@cocotb.test()
async def v2_byte_granular_cfgwr0(dut):
    """A one-byte config write at offset 0x19: first_be=0010, exactly one TLP.

    This is the shape Commit 2b writes a Secondary Bus Number with.  Offset
    0x19 is byte 1 of the Dword at 0x18, so register number 6 and first_be
    0010.  If the wrapper widened this to a full-Dword write it would clobber
    the three neighbouring bytes of a live bridge's config space.
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


# ==========================================================================
# V3 -- four outstanding, completions returned OUT OF ORDER
# ==========================================================================
@cocotb.test()
async def v3_out_of_order_completions(dut):
    """Four requests in flight, answered 3,1,0,2.  Each RC packet must carry
    its OWN request's tag and its OWN payload.

    This is what enumeration does against a slow device, and it is the single
    property the whole commit exists to provide.  A wrapper that paired
    completions with requests positionally -- by arrival order rather than by
    tag -- passes every in-order test and fails here.  Each completion carries
    a payload derived from its own tag, so a cross-assignment shows up in the
    data as well as in the descriptor.
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

    # Deliberately neither in order nor reversed: 3,1,0,2 has no fixed point
    # and is not a reversal, so neither "positional" nor "reverse-positional"
    # pairing survives it.
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


# ==========================================================================
# V4 -- RC backpressure -> tag pressure -> recovery
# ==========================================================================
@cocotb.test()
async def v4_backpressure_tag_exhaustion_recovery(dut):
    """Hold m_axis_rc_tready low, exhaust the tags, then release.

    The full flow-control loop: RQ -> tag allocation -> completion -> RC drain
    -> tag release -> RQ resumes.  The properties that matter are that the
    stall propagates BACKWARDS as ordinary AXI-Stream backpressure rather than
    deadlocking or dropping, and that everything still standing when ready
    rises is delivered exactly once.
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
    # A BOUNDED amount of buffering here is legal and expected: the next
    # request is launched into the requester, which then parks in REQ_TAG with
    # no tag available (tlp_requester.sv:211, 215-218), and pcie_rq_if can hold
    # one more descriptor behind it waiting for command_ready_o.  So two extra
    # requests are absorbed without any TLP being emitted.  What must NOT
    # happen is unbounded acceptance, so this issues more than the pipeline can
    # swallow and requires the sender to still be blocked.
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
    for index in range(count):
        await cfg_read(dut, reg_num=(start_reg + index) & 0x3F, first_be=0xF)


async def _complete_all(completer, requests):
    for req in requests:
        await completer.complete(req, status=CPL_SC)


# ==========================================================================
# V5 -- CRS
# ==========================================================================
@cocotb.test()
async def v5_crs_completion(dut):
    """Configuration Request Retry Status carried faithfully to the descriptor.

    An NVMe device may legally answer an early Configuration read with CRS, and
    Commit 2b has to see it to know to RETRY rather than to conclude the
    function is absent.  Folding it into a generic "error" would make
    enumeration give up on a device that was merely still initialising.
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


# ==========================================================================
# V6 -- UR
# ==========================================================================
@cocotb.test()
async def v6_ur_completion(dut):
    """Unsupported Request carried faithfully; the tag is released.

    Enumeration probing an absent device hits this constantly -- every empty
    slot and every unimplemented function answers UR.  A UR that did not
    release its tag would exhaust the tag pool within one bus scan.
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


# ==========================================================================
# SS COMPLETION TIMEOUT (V7..V9)
#
# The standalone target verilate_tlp_cpl_timeout owns the cycle-exact
# mechanism at CPL_TIMEOUT_CYCLES=64.  These three own what only the assembled
# design can answer: that the strobes reach the TOP-LEVEL ports the Commit 2b
# FSM will watch, that they correlate with pcie_rq_tag_o, that answered and
# unanswered requests do not contaminate each other, and that a late
# completion's PAYLOAD BEATS drain without wedging the receive path.
#
# These run at the shipped 4096-cycle default -- deliberately, since the
# default is what 2b will actually see.  Each timeout therefore costs ~16.4 us
# of simulation at CLK_NS=4.
# ==========================================================================
@cocotb.test()
async def v7_config_read_times_out(dut):
    """V-T1: a CfgRd0 nobody answers times out, visibly, at the top level.

    The tag in the strobe must be the tag pcie_rq_tag_o presented when the
    request went out -- that correlation is the whole point of the sideband.
    The interface must keep accepting requests: only one of TAG_COUNT tags was
    consumed, so recovery here does NOT depend on the quarantine expiring.
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
    """V-T2: answered and unanswered requests in flight together do not mix.

    Three reads go out on three DISTINCT tags and only the middle one is
    answered.  An assertion over "tag is always 0" would prove nothing, so the
    test asserts the tags are distinct before relying on them.
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
    """V-T3 / T6: a MULTI-BEAT late completion drains without wedging anything.

    This is the RC3/RC5 bug class -- header-declared length versus payload
    actually sent.  The request was a 1-Dword config read, but the late
    completion carries FOUR Dwords: length and any per-request byte accounting
    are forced apart, so a drain that sized itself from the request would
    under-consume and stall the receive path.  The tracker skips byte-count
    checking for a quarantined tag by policy, and the beats are swallowed by
    the orphan drain in pcie_rc_if.sv:341-343 (which $warnings once per Dword;
    that output is expected here, not a failure).
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

    # THE BYTE-ACCOUNTING ASSERTION.  pcie_rc_if reports RC_ERR_ORPHAN_DATA once
    # per orphaned Dword (pcie_rc_if.sv:404-405), so the count IS the number of
    # payload beats the drain consumed.  Four in, four reported: the drain
    # followed the completion's own length and not the 1-Dword request behind
    # the tag.  A drain that sized itself from the request would report 1 here
    # and leave three beats stuck in the receive path.
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


# ==========================================================================
# V10 -- Stage D-2, F2.5: CFG1 completions return like any config completion
# ==========================================================================
@cocotb.test()
async def v10_cfg1_round_trip(dut):
    """F2.5: a CfgRd1's CplD and a CfgWr1's Cpl correlate by tag and decode
    identically to the Type 0 path.

    Recorded in docs/predictions/SPEC_PREDICTIONS_STAGE_D.md SS7.3 as a NON-FALSIFIABLE row:
    nothing emitted CFG1 through this surface before D-2, so there is no
    meaningful pre-change run -- this test exists post-change only.  The
    request side asserts the whole DW0 (Trap A: dw0[4:0] = 00101 is the only
    bit that distinguishes this from the long-green Type 0 round trip); the
    completion side asserts the same RC-descriptor decode V1 pins for CFG0.
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


# ==========================================================================
# SS STAGE F-1, PHASE 2 -- THE A4 COMPLETER ORACLES
#
# §41.1 A4: pcie_rq_rc_top is requester-only.  An inbound Memory request from a
# DMA-ing device is accepted and discarded in the same cycle with no error
# strobe, and an inbound Memory Read is never completed.  These rows are the
# falsifiable form of that defect.
#
# WHY THESE ROWS OBSERVE THE WIRE AND NOT A CQ PORT.  The host-side CQ/CC
# interface does not exist yet -- it is Phase 3.  A bench cannot reference a
# port that has not been declared, so the rows that can be written BEFORE any
# src/ change are exactly the ones whose oracle is spec-visible on the link:
# "a Memory Read is answered by a Completion", "an unsupported request is
# answered by a UR Completion".  Those are properties of PCIe, not of our
# wrapper's port list, so they are the right things to assert first (§22.75 --
# spec-golden, never written from RTL behaviour).  The CQ/CC DESCRIPTOR field
# maps (PG213 Tables 52/58) are unit-level and land with their own targets in
# Phase 3.
#
# CONTROLS.  Each expect_fail row is paired with an ordinary PASS row whose
# observation point is INDEPENDENT of the signal under test (§22.80).  The
# signal under test is the TX stream m_dllp_axis_*; the controls observe the RX
# acceptance handshake and malformed_o / rx_error_valid_o instead.  Without
# them an expect_fail row cannot distinguish "the RC failed to complete a
# request it accepted" -- the defect -- from "the stimulus was malformed and
# correctly rejected", which would assert nothing about A4 at all.
#
# ! THE MSG ROW IS A CONTROL, NOT AN A4 ROW.  Phase 0's route census found that
# tlp_validator rejects every Message type, so the parser diverts a Msg to
# RX_DROP and ALREADY strobes rx_error_valid_o.  A Msg is therefore not
# silently discarded and is not an A4 case.  a4_control_msg_is_already_strobed
# pins that, so that a future "nothing is silently dropped" assertion cannot be
# written against rx_error_valid_o and pass vacuously (Phase 0 §8.5).
#
# Spec anchors, all read from the shelf, page numbers from the PDF of record:
#   Base 2.1 §2.2.7  p. 76   Memory, I/O and Configuration Request Rules
#   Base 2.1 §2.2.9  p. 97   Completion Rules -- RID/Tag echo, Byte Count,
#                            Lower Address, BCM
#   Base 2.1 §2.3.1  p. 107  "If the Request requires Completion, a Completion
#                            Status of UR is returned"
#   Base 2.1 §2.3.2  p. 120  Completion Status encodings
# ==========================================================================

# The DMA-ing device's own BDF.  Distinct from RID (this Root Complex) and from
# COMPLETER, so a completion echoing the wrong one is visible rather than
# accidentally equal.
DEVICE_RID = 0x0300

# tlp_pkg::tlp_type_e additions used only by these rows
TYPE_MEM = 0b00000
TYPE_IO = 0b00010
TYPE_MSG = 0b10000
FMT_4DW_NO_DATA = 0b001

# tlp_pkg::tlp_error_e ordinal (tlp_pkg.sv, the tlp_error_e declaration)
TLP_ERR_BAD_FMT_TYPE = 5

# tlp_layer's BAR defaults, which pcie_rq_rc_top does NOT override: BAR0 only,
# base 0, mask 0xffff_ffff_ffff_f000 -- one 4 KB aperture at address 0.  Any
# address below 0x1000 is a BAR hit.  See the KNOWN_GAP note in the Stage F-1
# findings: the wrapper hardcodes these, so the RC's BAR map has never been
# anything else (§22.43).
BAR0_ADDRESS = 0x100


def req_dw0(fmt, tlp_type, length_dw, tc=0, attr=0):
    """Request DW0 as tlp_parser reads it (the RX_FIRST field extraction).

    Bit-for-bit the inverse of tlp_generator's dw0 assembly, including the split
    Attr field: Attr[2] at bit 10, Attr[1:0] at bits [21:20] (Base 2.1 §2.2.6.3
    p. 73 -- "attribute bit 2 is not adjacent to bits 1 and 0").
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
    # ! A Completion does NOT use the request Length encoding.  For requests an
    # encoded 0 means 1024 Dwords, which is what dw0_length() implements.  For a
    # Completion with no data an encoded 0 means Length 0, and tlp_parser
    # carries exactly that special case (its CPL length_dw arm).  Decoding a UR
    # Completion with the request rule reads Length 1024 and calls a correct
    # Completion broken -- which is precisely what happened at Stage F-1
    # commit 4 before this helper was fixed.
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

    This is the INDEPENDENT observation point for the A4 controls (§22.80): it
    reads malformed_o / rx_error_valid_o / rx_error_code_o, none of which is
    computed from the TX stream the expect_fail rows assert about.
    """

    def __init__(self, dut):
        self.dut = dut
        self.errors = []
        self.malformed = 0

    def start(self):
        cocotb.start_soon(self._run())

    async def _run(self):
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

    Returns the number of Dwords the DUT accepted.  A caller uses that as the
    acceptance control: a request the TL never took is not evidence about A4.
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

    length_dw == 1 requires last_be == 0 (Base 2.1 §2.2.5 p. 67, and
    tlp_validator enforces it), so the defaults are a legal single-Dword read.
    """
    return [req_dw0(FMT_3DW_NO_DATA, TYPE_MEM, length_dw),
            req_dw1(DEVICE_RID, tag, first_be, last_be),
            mem_dw2(address)]


def iord_tlp(tag, address=0x40):
    """Inbound I/O Read.  Length is always 1 Dword (Base 2.1 §2.2.7 p. 76)."""
    return [req_dw0(FMT_3DW_NO_DATA, TYPE_IO, 1),
            req_dw1(DEVICE_RID, tag, 0xF, 0x0),
            mem_dw2(address)]


def cfgrd0_tlp(tag, reg_num=0x00):
    """Inbound Configuration Read Type 0 -- a device sending Cfg upstream.

    Malformed by intent: an Endpoint has no business originating a
    Configuration request.  The RC must answer UR, not drop it.
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


# --------------------------------------------------------------------------
# A4-C1 / A4-1: inbound Memory Read
# --------------------------------------------------------------------------
@cocotb.test()
async def a4_control_inbound_memrd_is_accepted(dut):
    """CONTROL for a4_inbound_memrd_returns_cpld.  Ordinary PASS.

    Proves the stimulus is well-formed and the Transaction Layer TAKES it: all
    three Dwords are accepted on the RX handshake and neither malformed_o nor
    rx_error_valid_o fires.  Observation point is the RX error surface, which is
    independent of the TX stream the paired row asserts about (§22.80).

    Without this row, a4_inbound_memrd_returns_cpld failing would be equally
    consistent with "the read was rejected as malformed" -- which is not the A4
    defect and would need a different fix.
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


# --------------------------------------------------------------------------
# ⚠️ RETIRED: a4_inbound_memrd_returns_cpld
#
# This row existed here from Phase 2 until Stage F-1 commit 3, as an
# expect_fail asserting that an inbound Memory Read produces a CplD.  ITS
# ORACLE WAS WRONG and it is retired rather than flipped.
#
# The row injected a MemRd and expected a Completion with NO host involvement.
# No correct completer does that: for a Memory Read the DATA belongs to the
# host's memory, so the Root Complex delivers the request on CQ and the host
# answers on CC.  A Root Complex that synthesised a CplD by itself would be
# returning data it had never read.  The row could therefore never have flipped
# -- it would have stayed red through F-2 and beyond, reading like an open
# defect when it was a mis-stated oracle.
#
# The property it MEANT to assert is asserted correctly, and more thoroughly,
# by f1_cc_descriptor_becomes_cpld_on_the_wire below: MemRd in, CQ out, CC
# back, CplD on the wire, with every header field checked against Base 2.1
# §2.2.9 p. 97 and the Completer ID proven to come from completer_id_i rather
# than from anything the host supplied.
#
# Recorded rather than silently deleted, per §22.77's point that an
# expect_fail row's status is not self-evidencing: a row that disappears
# between two gates has to say why, or the count moves with no explanation.
#
# The UR rows below are NOT affected -- a UR completion IS synthesised by the
# Root Complex with no host involvement, which is exactly why those two can
# flip at commit 4 and this one could not.
# --------------------------------------------------------------------------


# --------------------------------------------------------------------------
# A4-C2 / A4-2 / A4-3: unsupported inbound requests get UR, not silence
# --------------------------------------------------------------------------
@cocotb.test()
async def a4_control_inbound_io_and_cfg_are_accepted(dut):
    """CONTROL for both UR rows.  Ordinary PASS.

    An inbound I/O Read and an inbound CfgRd0 are both well-formed TLPs that
    tlp_validator ADMITS (Phase 0 route census rows 5-7), so they reach the
    completer surface and are consumed there.  Neither is reported malformed.

    This is what makes the two UR rows below assertions about A4 rather than
    about parser legality.
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
    """An inbound I/O Read must be answered with a UR Completion.  expect_fail.

    Base 2.1 §2.3.1 p. 107: "If the Request Type is not supported ... the
    Request is an Unsupported Request ... If the Request requires Completion, a
    Completion Status of UR is returned."  Completer Abort is explicitly the
    wrong status here.  §2.2.9 p. 97: a Completion with a status other than SC
    carries no data and has Length 0.

    FLIPPED at Stage F-1 commit 4; the expect_fail marker is removed, which is
    what makes this a mutation-testable oracle rather than a status line
    (§22.77).
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
    """An inbound Configuration Read must be answered with UR.  expect_fail.

    A device originating a Configuration request upstream is out of spec, but
    the RC's obligation is unchanged: the request is non-posted and requires a
    Completion, so it is terminated with UR (Base 2.1 §2.3.1 p. 107), never
    dropped.  Dropping it makes the device wait for its own Completion Timeout.

    FLIPPED at Stage F-1 commit 4; the expect_fail marker is removed, which is
    what makes this a mutation-testable oracle rather than a status line
    (§22.77).
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


# --------------------------------------------------------------------------
# A4-C3: the Msg row is a CONTROL, and records why Msg is out of scope
# --------------------------------------------------------------------------
@cocotb.test()
async def a4_control_msg_is_already_strobed(dut):
    """Ordinary PASS.  An inbound Message is NOT an A4 case.

    Phase 0's route census: tlp_validator admits only MEM, IO, CFG0, CFG1, CPL
    and CPL_LOCK, so every Message type is rejected by type and the parser
    diverts it to RX_DROP, raising malformed_o / rx_error_valid_o with
    TLP_ERR_BAD_FMT_TYPE.  A Message is therefore reported, not silently
    discarded, and closing A4 does not close it.

    This row exists to FORBID a vacuous control.  A future "nothing inbound is
    silently dropped" assertion written against rx_error_valid_o would pass
    today for Messages and say nothing about the Memory path, which is the
    actual defect.  Pinning the Msg behaviour here means that assertion has to
    find an independent observation point (Phase 0 §8.5).

    Giving a Message a UR Completion instead requires tlp_validator and
    tlp_parser to admit Message headers and route them to the completer.  That
    is a registered item with its own scope, not F-1 (decision 4).
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


# ==========================================================================
# SS STAGE F-1 COMMIT 2 -- THE CQ PATH
#
# pcie_cq_if now drives target_request_ready_i / target_data_ready_i, so an
# inbound request either becomes a CQ packet on m_axis_cq_* or raises
# cq_dropped_o with a reason code.  These rows assert both halves.
#
# Oracles are PG213 v1.3 Table 52 (p. 146) for the descriptor, Table 57 for the
# Request Type encoding, Table 10 for the tuser sideband, and Base 2.1 §2.2.5
# p. 67 for the byte enables.  Goldens are hand-derived from those tables and
# never read back from the DUT.
# ==========================================================================

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

# pcie_rq_rc_top's CQ_BAR_APERTURE default: 12 == 4 KB == tlp_layer's default
# BAR_MASK.  Asserted, not read back, so a silent change to either is caught.
CQ_APERTURE = 12


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

    Records BOTH so a test can assert the exclusive-or that closes A4: an
    inbound request produces a CQ packet or a drop strobe, never neither.
    """

    def __init__(self, dut):
        self.dut = dut
        self.packets = []      # list of (descriptor_int, [payload Dwords])
        self.drops = []        # cq_error_code_o values
        self.cc_errors = []    # cc_error_code_o values, on cc_protocol_error_o
        self.tusers = []       # m_axis_cq_tuser sampled on the first beat
        self._partial = []
        self._user = None

    def start(self):
        cocotb.start_soon(self._run())

    async def _run(self):
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
        for _ in range(cycles):
            await RisingEdge(self.dut.clk_i)
            if len(self.packets) >= count:
                return
        raise AssertionError(
            f"expected {count} CQ packet(s), saw {len(self.packets)}")

    async def wait_drops(self, count, cycles=600):
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
    """A4's write half: an inbound MemWr becomes a CQ packet, payload intact.

    PG213 Table 52 p. 146 for every descriptor field; Table 57 for Request
    Type.  This is the row that was impossible before Stage F-1: the write used
    to be consumed and discarded with no strobe and nothing on any port.
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

    PG213 Table 52: for Memory Reads the Dword Count is the size to be READ, so
    the descriptor carries a non-zero count with NO payload behind it.  That
    asymmetry with the write row is the point of having both.
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

    PG213 Table 10: first_be[3:0] at tuser[3:0], last_be[3:0] at tuser[7:4],
    valid in the first beat of the packet.  Base 2.1 §2.2.5 p. 67 defines the
    fields themselves.

    A DISCRIMINATING pair: the two writes differ ONLY in their byte enables, so
    a module that hardwired tuser -- or dropped it, which is the easy mistake --
    passes neither.  0xF/0xF and 0x3/0xC are chosen so no nibble is shared
    between the two rows and no value equals its own complement.
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
    """The anti-A4 strobe: an I/O request the host cannot serve is REPORTED.

    Nothing is delivered to the host -- I/O is not a Memory request and there
    is no BAR to land it in -- but the request must not vanish either.  It
    raises cq_dropped_o with CQ_DROP_UNSUPPORTED, which is what makes the drop
    observable.  §2.3.1 p. 107 says such a request is additionally owed a UR
    Completion; that is commit 4, and until then this row asserts only that the
    silence is gone.

    ! The control is on cq_dropped_o, deliberately NOT on rx_error_valid_o
    (§22.80).  The parser strobes rx_error_valid_o for Messages already, so a
    check written against it would pass without the completer path existing at
    all and would assert nothing about A4.
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
    """A Memory request matching no enabled BAR is reported, not delivered.

    tlp_layer's default BAR map is one 4 KB window at address 0, so 0x8000_0000
    lands outside it.  Delivering it would hand the host an address it never
    claimed; dropping it silently would be A4 again.  The reason code
    distinguishes this from the unsupported-type case, which is why the two
    have separate encodings rather than one generic "dropped".
    """
    rc, completer = await init(dut)
    cq = CqWatch(dut)
    cq.start()

    await inject_rx(dut, memrd_tlp(tag=0x46, address=0x8000_0000))
    await cq.wait_drops(1)

    assert cq.drops == [CQ_DROP_NO_BAR], \
        f"expected CQ_DROP_NO_BAR ({CQ_DROP_NO_BAR}), saw {cq.drops}"
    assert cq.packets == []


@cocotb.test()
async def f1_no_inbound_request_is_silently_discarded(dut):
    """⭐ THE A4 CLOSURE ROW, in its general form.

    For a mixed batch of inbound requests -- deliverable and not -- every one
    must produce EXACTLY ONE of: a CQ packet, or a cq_dropped_o strobe.  Never
    neither, which was the defect, and never both, which would double-report.

    This is the row that would have to be deleted for A4 to come back, so it is
    written as a count identity over the whole batch rather than as a per-case
    assertion: a regression that reintroduces the discard for one request class
    fails here even if that class has no dedicated row of its own.
    """
    rc, completer = await init(dut)
    cq = CqWatch(dut)
    cq.start()

    batch = [
        memrd_tlp(tag=0x50, address=BAR0_ADDRESS),            # deliverable
        memwr_tlp(tag=0x51, address=BAR0_ADDRESS + 0x40),     # deliverable
        iord_tlp(tag=0x52),                                   # unsupported
        cfgrd0_tlp(tag=0x53),                                 # unsupported
        memrd_tlp(tag=0x54, address=0x8000_0000),             # no BAR
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


# ==========================================================================
# SS STAGE F-1 COMMIT 3 -- THE CC PATH
#
# pcie_cc_if now drives tlp_layer's completion_request_* group, so the host's
# CC descriptor becomes a real Cpl/CplD on the wire.  This is what flips the
# a4_inbound_memrd_returns_cpld row.
#
# Oracle: PG213 v1.3 Table 58 (p. 168-169) for the descriptor the bench BUILDS,
# and Base 2.1 §2.2.9 p. 97 for the Completion header the DUT must EMIT.  The
# two are independent documents and the test asserts the mapping between them.
# ==========================================================================

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

    Beat 0 carries descriptor Dwords 0..2 plus the first payload Dword, which
    is what PG213 Figure 32 specifies for a 128-bit interface.
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
    """⭐ The A4 read path, end to end: MemRd -> CQ -> CC -> CplD on the wire.

    The device reads, the host answers, and a real Completion goes back out.
    Every emitted header field is checked against Base 2.1 §2.2.9 p. 97 and
    against the CC descriptor the bench built from PG213 Table 58:

      Requester ID / Tag   echoed from the request (§2.2.9)
      Completer ID         OUR configured BDF, from completer_id_i -- NOT
                           anything the host put in the descriptor
      Byte Count           bytes remaining including this Completion
      Lower Address        low 7 bits of the first byte returned
      BCM                  0 (a PCI-X bridge field)

    The Completer ID assertion is the load-bearing one: it is what proves the
    Transaction Layer owns the Root Complex's identity rather than the host,
    which is why pcie_cc_if deliberately drops the descriptor's Completer Bus /
    Target Function / Completer ID Enable fields.
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
    """NEGATIVE PAIR for the row above: a bad status is refused, not emitted.

    PG213 Table 58 lists exactly three legal values on this interface -- SC,
    UR and CA.  CRS is deliberately NOT among them: a Root Complex may RECEIVE
    a CRS Completion (pcie_rc_if carries it faithfully, because enumeration has
    to see it) but must never ORIGINATE one.  A host that asks for CRS is
    refused with CC_ERR_BAD_STATUS and nothing goes on the wire.

    Without this row, the positive row above cannot distinguish "builds the
    Completion the descriptor asked for" from "builds a Completion regardless
    of what the descriptor said" (§22.81).
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


# ==========================================================================
# SS THE TWO MUTATION SURVIVORS, CLOSED
#
# The Stage F-1 census (evidence/stage-f-1/MUTATION_PREDICTIONS.md) predicted
# two survivors on the new arms and named the test each was owed.  These are
# those tests.  Both were written mutant-first: each was confirmed to FAIL
# against its mutant before being accepted, so it detects the defect rather
# than merely passing beside it.
#
#   M6  pcie_cq_if `offered_non_posted` -> 1'b1
#       Nothing in the suite dropped a POSTED request, so nothing
#       distinguished "posted requests get no Completion" from "everything
#       gets one".  A device would receive a spurious Completion for a write
#       it never expected one for.
#
#   M9  pcie_cc_if S_DESC preempt guard `&& dw_idx_r == 2'd0` removed
#       No test had a host CC packet in flight while a UR was pending, so
#       nothing exercised the boundary that stops a synthesised Completion
#       being interleaved into the middle of the host's descriptor.
# ==========================================================================

@cocotb.test()
async def f1_dropped_posted_write_gets_no_completion(dut):
    """M6's killer.  A dropped POSTED request is reported but never completed.

    Base 2.1 §2.1.2 p. 55: a Memory Write is Posted -- it has no Completion,
    ever.  So an undeliverable MemWr must raise cq_dropped_o and put NOTHING on
    the wire, while an undeliverable MemRd (non-posted) must additionally get a
    UR Completion.

    The pair is the point.  Asserting only "the write produces no Completion"
    would also pass against a design that had stopped completing everything;
    the read arm in the same test is what makes the absence meaningful (§22.81).
    """
    rc, completer = await init(dut)
    cq = CqWatch(dut)
    cq.start()

    # --- posted: a Memory Write that matches no BAR ---
    await inject_rx(dut, memwr_tlp(tag=0x70, address=0x8000_0000,
                                   payload=(0xBADD_0001,), first_be=0xF))
    await cq.wait_drops(1)
    await settle(dut, 200)

    assert cq.drops == [CQ_DROP_NO_BAR], \
        f"expected CQ_DROP_NO_BAR for the undeliverable write, saw {cq.drops}"
    assert cq.packets == [], "an out-of-BAR write must not be delivered"
    cpls = await cpls_on_wire(completer)
    assert cpls == [], (
        f"a POSTED request must never be completed (Base 2.1 §2.1.2 p. 55), "
        f"but {len(cpls)} Completion(s) went out: {cpls}")

    # --- non-posted control, same drop reason, opposite obligation ---
    await inject_rx(dut, memrd_tlp(tag=0x71, address=0x8000_0000))
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
    """M9's killer.  A synthesised UR never interleaves into a host descriptor.

    pcie_cc_if lets a pending auto-UR preempt the host's CC stream, but ONLY at
    dw_idx_r == 0 -- a packet boundary.  Without that guard the UR can be taken
    after one or two Dwords of the host's descriptor have been consumed, and
    collection then resumes at the wrong Dword position, so the host's
    Completion goes out with mangled fields.

    ! THE RACE IS TWO CYCLES WIDE and the bench cannot hit it by construction,
    so the ARRIVAL ORDER is swept.  Each iteration starts the I/O read (which
    becomes the pending UR after the parser has taken all three of its Dwords)
    and then starts the host's CC answer `offset` cycles later.  Sweeping
    offset walks the moment ur_valid_i rises across the whole host packet,
    including the two cycles when its descriptor is half-collected.

    ! ONE init, ONE clock.  An earlier version called init() per iteration,
    which starts a fresh cocotb Clock driver each time -- five drivers on one
    net.  It passed, which is worse than failing: the assertions were being
    evaluated against a clock nothing owned.

    Every iteration asserts the host's CplD is field-exact AND that the UR still
    appears, so a corruption at ANY offset fails the test; the sweep only
    changes how fast it is found.
    """
    rc, completer = await init(dut)
    dut.completer_id_i.value = COMPLETER
    cq = CqWatch(dut)
    cq.start()

    seen = 0
    for offset in range(16):
        tag = 0x80 + offset
        addr = BAR0_ADDRESS
        # length_dw > 1 requires BOTH byte enables non-zero (Base 2.1 §2.2.5
        # p. 67; tlp_validator enforces it) or the read is rejected as
        # malformed and no CQ packet is ever produced.
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


# ==========================================================================
# SS THE MULTI-RCB ORACLE (decision F1-RCB) AND THE ORDERING ROW
#
# tlp_completion_generator has clamped completions to the Read Completion
# Boundary since Commit 2a, and until now NOTHING crossed one: its six existing
# rows all fit inside a single RCB, so the whole multi-segment loop ran on
# stimulus that could not distinguish it from a single-segment implementation.
# That is §35.2's fixed-point blindness in a much bigger arm.  These rows are
# what measure it.
#
# Config, read from init(): RCB = 64 B (rcb_128b_i = 0 -- Base 2.1 §2.3.1.1
# p. 112 lets a Root Complex choose 64 or 128, and 64 is the conservative
# half), MPS = 128 B, BAR0 = one 4 KB window at address 0.
#
# The goldens below are HAND-DERIVED from Base 2.1 §2.3.1.1 p. 112 (segments
# must not cross a naturally-aligned RCB boundary) and PG213 Table 58 (Byte
# Count is the bytes REMAINING including this Completion; Lower Address is the
# low 7 bits of this Completion's own first byte).  They are not read back from
# the DUT.
# ==========================================================================

async def read_and_answer(dut, cq, completer, tag, address, total_bytes):
    """Inbound MemRd of `total_bytes`, answered by the host in ONE CC packet.

    The host hands over a single logical completion -- status, total Byte
    Count, starting Lower Address, whole payload -- and the Transaction Layer
    decides how many CplDs that becomes.  Returns the payload it sent, so the
    caller can check the split preserved it end to end.
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
    """128 B from an RCB-aligned start splits into TWO 64 B Completions.

    Base 2.1 §2.3.1.1 p. 112.  RCB = 64, so a 128 B read starting on a boundary
    is two full segments.  Byte Count counts DOWN (128 then 64) and Lower
    Address counts UP (0 then 64) -- PG213 Table 58.
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
    """⭐ 128 B starting 16 B INTO an RCB splits 48 / 64 / 16.

    THIS IS THE DISCRIMINATING ROW.  An implementation that splits every 64
    bytes from the START OF THE TRANSFER -- the obvious wrong rule -- produces
    64/64 here and passes cc_multi_rcb_split_aligned unharmed.  Only a segment
    that is clamped to the distance to the next NATURALLY ALIGNED RCB boundary
    yields 48 first (Base 2.1 §2.3.1.1 p. 112).

    It is also the only row whose middle segment is neither MPS nor a full RCB,
    and the only one whose final Lower Address wraps: 16+48+64 = 128, truncated
    to 7 bits = 0.
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
    """NEGATIVE PAIR: a read that fits inside one RCB is ONE Completion.

    Without this row the two split rows cannot distinguish "splits at the RCB
    boundary" from "always splits" (§22.81).  64 B from an aligned start
    exactly fills one RCB and must not be divided.
    """
    rc, completer = await init(dut)
    dut.completer_id_i.value = COMPLETER
    cq = CqWatch(dut)
    cq.start()

    tag, addr = 0x92, BAR0_ADDRESS
    payload = await read_and_answer(dut, cq, completer, tag, addr, 64)
    check_split(await cpls_on_wire(completer), [(16, 64, 0)], tag, payload)


@cocotb.test(expect_fail=True)
async def ordering_completion_behind_posted(dut):
    """⚠️ A Completion must not pass a queued Posted Request.  expect_fail.

    Base 2.1 §2.4.1 Table 2-33 p. 122-123, Row D (Read Completion) x Col 2
    (Posted Request) = "a) No".

    tlp_control arbitrates by strict alternation: prefer_completion_r starts at
    1 and is reloaded with !selected_completion on every granted header.  The
    violation needs FOUR things true in one cycle -- requester header valid,
    completion header valid, prefer_completion_r = 1, and !locked_r.

    ! GETTING THERE IS NOT OBVIOUS, and the first construction of this test
    FAILED TO PROVOKE IT.  Priming with a posted MemWr does not work: a write
    carries data, so tlp_control sets locked_r for its payload AND tlp_requester
    is itself busy streaming that payload, which means the NEXT write's header
    cannot be pending while a completion arrives.  The requester is serial, so
    two posted writes can never contend.

    The prime must therefore be NON-POSTED and data-less -- a Memory Read.  It
    is granted (setting prefer_completion_r <- 1), it occupies the generator
    while it is emitted, and it leaves the requester FREE to present the next
    header.  Then:

        RQ MemRd  R0   -> granted; prefer_completion_r <- 1; generator busy
        RQ MemWr  W1   -> header pending, blocked on the generator
        CC        C1   -> header pending too
        R0 drains      -> both valid, prefer = 1  ->  C1 GRANTED FIRST

    W1 was issued before C1, so a Completion has passed a posted request.

    The overlap window is a few cycles wide, so the CC arrival is SWEPT.  The
    row asserts the SPEC order on every iteration, so a violation at ANY offset
    fails it -- which is what this row is for.

    ! PRE-EXISTING, not introduced by Stage F-1.  tlp_control has always
    alternated; F-1 only makes it REACHABLE, because until the CC path existed
    only one of the two streams could ever present a header and the arbiter
    never had a contended cycle to get wrong.  Held red until F-2 makes
    tlp_control posted-aware.
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

        # Prime: a NON-POSTED read. Granted immediately, flips
        # prefer_completion_r to 1, and leaves the requester free.
        await send_rq(dut, [(rq_desc(RQ_MEM_READ, 1, address=0x2000),
                             0xF, True, tuser(0xF, 0x0))])
        # The posted write whose ordering is under test. Issued BEFORE the
        # completion, so the spec requires it on the wire first.
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
