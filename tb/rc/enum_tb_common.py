"""enum_tb_common -- goldens, models and checkers shared by enumeration benches

Author: Kourosh Ghahramani
Silicon Systems Research Lab, University of Washington

Purpose
    The one module from which the configuration-transaction and enumeration
    benches import their goldens, socket and completer models and checkers,
    so that they share one copy of each golden. Its users are the cocotb
    benches of pcie_cfg_txn, pcie_enum_bus, pcie_enum_top (the scan, BAR and
    bridge benches) and pcie_enum_dl_top; tb_rc.core and tb_rc_ep.core copy it
    beside the test modules in each fileset that needs it.

    Every descriptor and header builder is derived from the specification
    texts under References and from the field layouts of the RTL packages it
    names; nothing is read back from a DUT. Two self-tests run at import time
    in every bench: _selftest_type1_one_bit on the builders and
    _selftest_bridged_topology on the bridge model, so a broken golden fails
    before the first simulation step.

    Helpers whose ports or defaults differ per bench stay in the benches:
    settle(), init(), status(), start_scan(), wait_terminal(), the
    pcie_cfg_txn command-port drivers and the per-bench completers.

Structure
    Encodings                 RTL enum values, register numbers, byte enables
    RQ descriptor             rq_desc, tuser, their decoders and
                              assert_rq_descriptor
    RC descriptor             encode_rc_desc, rc_beats and their inverses
    On-wire TLP goldens       Configuration Request and Completion header
                              Dwords, and _selftest_type1_one_bit
    Socket model              Socket: pcie_rq_rc_top's user-side ports
    Integration-bench values  clock, timeout, IDs, enum_error_e, TlpRequest
    Flow control              set_credits, CreditDrip
    Monitor and wire check    Mon, assert_cfg_tlp_on_wire
    Golden device             the Type 0 device: IDs, header type, reg3
    Configuration space       BarSpec, ConfigDevice
    Empty-set guards          nonempty, expect_count, assert_sequence
    Bridged topology          BridgeConfigSpace, BridgedTopology,
                              BridgedCompleter, _selftest_bridged_topology

References
    PCIe Base Spec r2.1, §2.2.1
    PCIe Base Spec r2.1, §2.2.7
    PCIe Base Spec r2.1, §2.2.9
    PCIe Base Spec r2.1, §2.3.2
    PCIe Base Spec r2.1, §2.6.1.2
    PCIe Base Spec r2.1, §7.3.1
    PCIe Base Spec r2.1, §7.3.3
    PCIe Base Spec r2.1, §7.5.2
    PCIe Base Spec r2.1, §7.5.3
    PCIe Base Spec r2.1, §7.5.3.1
    PCIe Base Spec r2.1, §7.5.3.2
    PCIe Base Spec r2.1, §7.5.3.3
    PCI Local Bus Spec r3.0, §6.1
    PCI Local Bus Spec r3.0, §6.2.1
    PCI Local Bus Spec r3.0, §6.2.2
    PCI Local Bus Spec r3.0, §6.2.5.1
    PCI Local Bus Spec r3.0, §6.2.5.2
    PG213, Table 14
    PG213, Table 57
    PG213, Table 61
    PG213, Table 65
    PG213, Table 66
"""

# ---------------------------------------------------------------------------
# Encodings
# ---------------------------------------------------------------------------
# Python copies of the enum values and constants defined in pcie_rq_rc_pkg,
# tlp_pkg and pcie_enum_pkg, each block named after its RTL type, plus the
# Configuration Space register numbers and byte enables the benches use.
# The builders below and the tests read these names instead of literals.
# Where a value comes from a specification table, its block cites it.

# pcie_rq_rc_pkg::rq_req_type_e (PG213, Table 57)
RQ_CFG_READ0 = 0b1000
RQ_CFG_WRITE0 = 0b1010
# The Type 1 pair differs from the Type 0 pair in bit 0 only;
# _selftest_type1_one_bit checks this at import.
RQ_CFG_READ1 = 0b1001
RQ_CFG_WRITE1 = 0b1011

# pcie_rq_rc_pkg::rc_cpl_status_e. The RC descriptor's Completion Status
# [45:43] (PG213, Table 65) and the Completion header's field (PCIe Base Spec
# r2.1, §2.2.9) use the same encoding.
CPL_SC = 0b000
CPL_UR = 0b001
CPL_CRS = 0b010
CPL_CA = 0b100
# The four Reserved encodings, named so a test can drive them. A Completion
# with a Reserved status is handled as UR (PCIe Base Spec r2.1, §2.3.2).
CPL_RESERVED = (0b011, 0b101, 0b110, 0b111)

# pcie_rq_rc_pkg::rc_desc_error_e (PG213, Table 66)
EC_NORMAL = 0b0000
EC_POISONED = 0b0001
EC_BAD_STATUS = 0b0010          # terminated by UR / CA / CRS

# pcie_rq_rc_pkg::rc_error_e
RC_ERR_ORPHAN_DATA = 3

# tlp_pkg::tlp_error_e. tlp_request_tracker reports a completion whose tag
# matches no allocated tag once per packet, on rc_unexpected_completion_o.
# pcie_rc_if separately reports each payload Dword that has no result as
# RC_ERR_ORPHAN_DATA, so one such packet raises both;
# e5_late_completion_and_orphan_burst in test_pcie_enum_bar_tlp.py checks both.
TLP_ERR_UNEXPECTED_COMPLETION = 10

# pcie_enum_pkg::txn_outcome_e
TXN_OK = 0
TXN_UR = 1
TXN_CA = 2
TXN_CRS_EXHAUSTED = 3
TXN_TIMEOUT = 4

TXN_NAME = {
    TXN_OK: "TXN_OK",
    TXN_UR: "TXN_UR",
    TXN_CA: "TXN_CA",
    TXN_CRS_EXHAUSTED: "TXN_CRS_EXHAUSTED",
    TXN_TIMEOUT: "TXN_TIMEOUT",
}

# pcie_enum_pkg config register numbers: the Dword index into the Type 0
# header (PCIe Base Spec r2.1, §7.5.2, Figure 7-5)
CFG_REG_VENDOR_DEVICE = 0x00
CFG_REG_COMMAND_STATUS = 0x01
CFG_REG_REVISION_CLASS = 0x02
CFG_REG_CACHE_HEADER = 0x03
CFG_REG_BAR0 = 0x04
CFG_REG_BAR1 = 0x05
CFG_REG_BAR2 = 0x06
CFG_REG_BAR3 = 0x07
CFG_REG_BAR4 = 0x08
CFG_REG_BAR5 = 0x09
CFG_REG_BAR_FIRST = CFG_REG_BAR0
CFG_REG_BAR_LAST = CFG_REG_BAR5
BAR_SLOTS = 6
# The Expansion ROM Base Address register, offset 30h (PCI Local Bus Spec
# r3.0, §6.2.5.2), named so a test can assert that enumeration never
# accesses it.
CFG_REG_EXPANSION_ROM = 0x0C

# Byte enables. Last DW BE is 0000b for every Configuration Request (PCIe
# Base Spec r2.1, §2.2.7), so first_be is the only one a request chooses.
CFG_BE_DWORD = 0b1111
CFG_BE_LOWER_HALF = 0b0011
CFG_BE_BYTE2 = 0b0100
CFG_LAST_BE = 0b0000

# tlp_pkg::tlp_fmt_e and tlp_type_e, for the on-wire goldens
FMT_3DW_NO_DATA = 0b000
FMT_3DW_DATA = 0b010
TYPE_CFG0 = 0b00100
TYPE_CFG1 = 0b00101            # CfgRd1 / CfgWr1 (PCIe Base Spec r2.1, Table 2-3)
TYPE_CPL = 0b01010


# ---------------------------------------------------------------------------
# RQ descriptor -- what the DUT must emit
# ---------------------------------------------------------------------------
# The 128-bit Requester reQuest descriptor in its Configuration form (PG213,
# Table 61) and the first_be / last_be sideband in s_axis_rq_tuser (PG213,
# Table 14). rq_desc and tuser build the golden; decode_rq_desc and
# decode_tuser split an observed value into named fields for failure
# messages. assert_rq_descriptor is the check the standalone benches apply
# to a descriptor beat the socket captured.
def rq_desc(req_type, dword_count=1, address=0, completer_id=0, tc=0, attr=0,
            poisoned=0, tag=0, requester_id=0):
    """Build a 128-bit RQ descriptor (PG213, Table 61).

    pcie_rq_if ignores Tag [103:96] and Requester ID [95:80]: tags are
    allocated in the core and the Transaction Layer takes its Requester ID
    from requester_id_i. The golden still carries both fields, zero by
    default, so the whole-word compare in assert_rq_descriptor checks that
    the DUT drives them zero, not only that the core ignores them.
    """
    v = address & ((1 << 64) - 1)
    v |= (dword_count & 0x7FF) << 64
    v |= (req_type & 0xF) << 75
    v |= (poisoned & 0x1) << 79
    v |= (requester_id & 0xFFFF) << 80
    v |= (tag & 0xFF) << 96
    v |= (completer_id & 0xFFFF) << 104
    v |= (tc & 0x7) << 121
    v |= (attr & 0x7) << 124
    return v


def cfg_desc_address(reg_num, ext_reg=0):
    """Configuration form of the RQ descriptor address.

    {Reserved[63:12], Ext Reg Number[11:8], Register Number[7:2], Reserved[1:0]}
    (PG213, Table 61). Bits [1:0] are Reserved: the bytes within the Dword
    are selected by first_be, never by the address.
    """
    return ((ext_reg & 0xF) << 8) | ((reg_num & 0x3F) << 2)


def decode_rq_desc(v):
    """Inverse of rq_desc(), for asserting on what the DUT actually drove."""
    return {
        "address": v & ((1 << 64) - 1),
        "reg_num": (v >> 2) & 0x3F,
        "ext_reg": (v >> 8) & 0xF,
        "dword_count": (v >> 64) & 0x7FF,
        "req_type": (v >> 75) & 0xF,
        "poisoned": (v >> 79) & 1,
        "requester_id": (v >> 80) & 0xFFFF,
        "tag": (v >> 96) & 0xFF,
        "completer_id": (v >> 104) & 0xFFFF,
        "requester_id_en": (v >> 120) & 1,
        "tc": (v >> 121) & 0x7,
        "attr": (v >> 124) & 0x7,
        "force_ecrc": (v >> 127) & 1,
    }


def tuser(first_be, last_be=CFG_LAST_BE):
    """Low byte of s_axis_rq_tuser (PG213, Table 14).

    first_be is in bits [3:0] and last_be in bits [7:4].
    """
    return ((last_be & 0xF) << 4) | (first_be & 0xF)


def decode_tuser(v):
    """Split the low byte of s_axis_rq_tuser into first_be and last_be."""
    return {"first_be": v & 0xF, "last_be": (v >> 4) & 0xF}


def assert_rq_descriptor(observed_desc, observed_tuser, *, write, bdf, reg_num,
                         first_be, ext_reg=0, type1=False, what=""):
    """Assert one emitted RQ descriptor and its tuser against a fresh golden.

    The whole 128-bit word is compared, not a subset of fields, so a field
    the DUT sets and the golden leaves zero also fails. On a mismatch the
    message lists the differing fields. tuser must carry first_be and a
    Last DW BE of 0000b.

    type1 selects the Type 1 request types (1001b / 1011b) instead of Type 0
    (1000b / 1010b); the default is Type 0.
    """
    if type1:
        req_type = RQ_CFG_WRITE1 if write else RQ_CFG_READ1
    else:
        req_type = RQ_CFG_WRITE0 if write else RQ_CFG_READ0
    golden = rq_desc(
        req_type,
        dword_count=1,
        address=cfg_desc_address(reg_num, ext_reg),
        completer_id=bdf,
    )
    if observed_desc != golden:
        got, exp = decode_rq_desc(observed_desc), decode_rq_desc(golden)
        diff = {k: (hex(got[k]), hex(exp[k])) for k in exp if got[k] != exp[k]}
        raise AssertionError(
            f"{what}RQ descriptor mismatch\n"
            f"  observed 0x{observed_desc:032X}\n"
            f"  golden   0x{golden:032X}\n"
            f"  fields (got, expected): {diff}")
    exp_user = tuser(first_be)
    if (observed_tuser & 0xFF) != exp_user:
        raise AssertionError(
            f"{what}tuser mismatch: observed {decode_tuser(observed_tuser & 0xFF)}, "
            f"expected {decode_tuser(exp_user)} "
            f"(Last DW BE must be 0000b for every Configuration Request -- "
            f"Base 2.1 SS2.2.7 p.79)")


# ---------------------------------------------------------------------------
# RC descriptor -- what the socket delivers back
# ---------------------------------------------------------------------------
# The 96-bit Requester Completion descriptor (PG213, Table 65) and its beats
# on m_axis_rc. encode_rc_desc builds a descriptor whose defaults match what
# pcie_rc_if builds for a configuration read completion. rc_beats packs a
# descriptor and its payload into 128-bit beats in the layout pcie_rq_rc_pkg
# describes; decode_rc_desc, packet_dwords and split_packet take a packet
# apart again.
def encode_rc_desc(tag, status=CPL_SC, dword_count=None, request_completed=1,
                   byte_count=None, error_code=None, lower_address=0,
                   requester_id=0, completer_id=0, tc=0, attr=0, poisoned=0,
                   locked=0):
    """Build a 96-bit RC descriptor (PG213, Table 65).

    The defaults are what pcie_rc_if builds for a configuration read
    completion, so a bench that overrides nothing drives a realistic packet:

      * a Successful Completion carries one Dword and Byte Count
        4 x dword_count;
      * any other status carries no data, sets Request Completed and uses
        error code 0010b, since such a Completion has no data and is the
        final one for its Request (PCIe Base Spec r2.1, §2.3.2; PG213,
        Table 66); its Byte Count is 4 (PCIe Base Spec r2.1, §2.2.9).
    """
    if dword_count is None:
        dword_count = 1 if status == CPL_SC else 0
    if byte_count is None:
        byte_count = 4 * dword_count if status == CPL_SC else 4
    if error_code is None:
        error_code = EC_NORMAL if status == CPL_SC else EC_BAD_STATUS
    v = lower_address & 0xFFF
    v |= (error_code & 0xF) << 12
    v |= (byte_count & 0x1FFF) << 16
    v |= (locked & 1) << 29
    v |= (request_completed & 1) << 30
    v |= (dword_count & 0x7FF) << 32
    v |= (status & 0x7) << 43
    v |= (poisoned & 1) << 46
    v |= (requester_id & 0xFFFF) << 48
    v |= (tag & 0xFF) << 64
    v |= (completer_id & 0xFFFF) << 72
    v |= (tc & 0x7) << 89
    v |= (attr & 0x7) << 92
    return v


def decode_rc_desc(v):
    """Split a 96-bit RC descriptor into named fields (PG213, Table 65)."""
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


def rc_beats(desc, payload=()):
    """RC descriptor + payload -> [(tdata, tkeep, tlast), ...].

    Beat 0 carries the 3-Dword descriptor in Dwords 0..2 and the first payload
    Dword in Dword 3; later beats carry payload, offset by one Dword, as the
    RC descriptor note in pcie_rq_rc_pkg describes. A descriptor-only packet
    is a single beat with tkeep = 0b0111.
    """
    payload = list(payload)
    dwords = [desc & 0xFFFFFFFF, (desc >> 32) & 0xFFFFFFFF,
              (desc >> 64) & 0xFFFFFFFF] + payload
    beats = []
    for base in range(0, len(dwords), 4):
        chunk = dwords[base:base + 4]
        tdata = 0
        keep = 0
        for index, word in enumerate(chunk):
            tdata |= (word & 0xFFFFFFFF) << (32 * index)
            keep |= 1 << index
        beats.append((tdata, keep, 1 if base + 4 >= len(dwords) else 0))
    return beats


def packet_dwords(beats):
    """[(tdata, tkeep, tlast), ...] -> flat Dword list."""
    words = []
    for tdata, tkeep, _last in beats:
        for dword in range(4):
            if (tkeep >> dword) & 1:
                words.append((tdata >> (32 * dword)) & 0xFFFFFFFF)
    return words


def split_packet(beats):
    """Split RC beats into (96-bit descriptor, [payload Dwords])."""
    words = packet_dwords(beats)
    assert len(words) >= 3, f"RC packet shorter than a descriptor: {words}"
    return words[0] | (words[1] << 32) | (words[2] << 64), words[3:]


# ---------------------------------------------------------------------------
# On-wire TLP goldens
# ---------------------------------------------------------------------------
# Header Dwords of the Configuration Requests the Transaction Layer emits and
# of the Completions a bench injects, in the Dword form tlp_generator and
# tlp_parser use with PCIE_WIRE_ORDER = 0, the pcie_rq_rc_top default. DW0
# holds header byte N at bits [8N+7:8N]; DW1 and DW2 hold their fields most
# significant first. The integration benches compare against these, the
# pcie_enum_dl_top bench after converting its frames to this form, and
# test_pcie_enum_bus uses cfg_wire_dw2. _selftest_type1_one_bit runs at
# import and checks that the Type 0 and Type 1 goldens differ in one bit.
def cfg_wire_dw2(bus, dev, fn, reg_num, ext_reg=0):
    """The Configuration Request's third header Dword, as emitted.

    {Bus[31:24], Device[23:19], Function[18:16], Reserved[15:12],
     Ext Reg[11:8], Register[7:2], R[1:0]} (PCIe Base Spec r2.1, §2.2.7,
    Figure 2-18). pcie_rq_if takes the Bus, Device and Function from the RQ
    descriptor's Completer ID field; the descriptor's address supplies only
    the register numbers.
    """
    return (((bus & 0xFF) << 24) | ((dev & 0x1F) << 19) | ((fn & 0x7) << 16)
            | ((ext_reg & 0xF) << 8) | ((reg_num & 0x3F) << 2))


def cfg_wire_dw0(write, length_dw=1, tc=0, attr=0, type1=False):
    """Configuration Request DW0 as tlp_generator assembles it.

    Fmt is the only field a read and a write change. type1 selects Type 1
    (Type[4:0] = 00101b) instead of Type 0 (00100b).

    attr is Attr[2:0] = {IDO, RO, NS}: Attr[2] goes to dw0[10] and Attr[1:0]
    to dw0[21:20], the byte-1 and byte-2 positions of PCIe Base Spec r2.1,
    §2.2.1, so the two halves are not adjacent. Leave tc and attr at 0: a
    Configuration Request carries Length 1, TC 000b, Attr[1:0] 00b and AT
    00b, with Attr[2] reserved (PCIe Base Spec r2.1, §2.2.7), and any other
    value builds a TLP a Receiver may treat as Malformed.
    """
    fmt = FMT_3DW_DATA if write else FMT_3DW_NO_DATA
    enc = length_dw & 0x3FF
    v = (fmt << 5) | (TYPE_CFG1 if type1 else TYPE_CFG0)
    v |= ((attr >> 2) & 0x1) << 10
    v |= (tc & 0x7) << 12
    v |= (attr & 0x3) << 20
    v |= ((enc >> 8) & 0x3) << 16
    v |= (enc & 0xFF) << 24
    return v & 0xFFFFFFFF


def _selftest_type1_one_bit():
    """Import-time check that the Type 0 and Type 1 goldens differ in one bit.

    The request types differ in bit 0 (1000b / 1010b against 1001b / 1011b),
    and so do the wire Type fields (00100b against 00101b). The whole-golden
    checks show that the Type 1 encoding changes that bit and nothing else,
    in rq_desc and in cfg_wire_dw0.
    """
    assert RQ_CFG_READ1 ^ RQ_CFG_READ0 == 1, "req_type read pair not one bit apart"
    assert RQ_CFG_WRITE1 ^ RQ_CFG_WRITE0 == 1, "req_type write pair not one bit apart"
    assert TYPE_CFG1 ^ TYPE_CFG0 == 1, "tlp_type_e CFG pair not one bit apart"
    # Whole-golden distance: bit 75 (the low bit of req_type) in the
    # descriptor, bit 0 of DW0 on the wire.
    kw = dict(dword_count=1, address=cfg_desc_address(0x06), completer_id=0x0100)
    assert rq_desc(RQ_CFG_READ1, **kw) ^ rq_desc(RQ_CFG_READ0, **kw) == 1 << 75
    assert rq_desc(RQ_CFG_WRITE1, **kw) ^ rq_desc(RQ_CFG_WRITE0, **kw) == 1 << 75
    for write in (False, True):
        t0 = cfg_wire_dw0(write, type1=False)
        t1 = cfg_wire_dw0(write, type1=True)
        assert t1 ^ t0 == 1, f"wire DW0 goldens differ by {t1 ^ t0:#x}, not bit 0"
        assert t0 & 0x1F == TYPE_CFG0 and t1 & 0x1F == TYPE_CFG1


_selftest_type1_one_bit()


def cfg_wire_dw1(requester_id, tag, first_be, last_be=0):
    """{Requester ID[31:16], Tag[15:8], Last DW BE[7:4], 1st DW BE[3:0]}.

    A request's second header Dword, as tlp_generator assembles it. Last DW
    BE is 0000b for a Configuration Request (PCIe Base Spec r2.1, §2.2.7).
    """
    return (((requester_id & 0xFFFF) << 16) | ((tag & 0xFF) << 8)
            | ((last_be & 0xF) << 4) | (first_be & 0xF))


def dw0_length(dw0):
    """Length in Dwords from a DW0 in tlp_generator's layout.

    A Length field of 0 means 1024 Dwords (PCIe Base Spec r2.1, §2.2.1).
    """
    enc = ((dw0 >> 24) & 0xFF) | (((dw0 >> 16) & 0x3) << 8)
    return 1024 if enc == 0 else enc


def cpl_dw0(has_data, length_dw, tc=0, attr=0):
    """Completion DW0 in the layout tlp_parser reads in its RX_FIRST state.

    attr is Attr[2:0] = {IDO, RO, NS}, placed as in cfg_wire_dw0. A
    Completion repeats the Attribute values of the Request it answers (PCIe
    Base Spec r2.1, §2.2.9), so unlike a Configuration Request's they need
    not be zero.
    """
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
    """{Completer ID[31:16], Status[15:13], BCM[12], Byte Count[11:0]}."""
    return (((completer_id & 0xFFFF) << 16) | ((status & 0x7) << 13)
            | ((bcm & 1) << 12) | (byte_count & 0xFFF))


def cpl_dw2(requester_id, tag, lower_address=0):
    """{Requester ID[31:16], Tag[15:8], R[7], Lower Address[6:0]}."""
    return (((requester_id & 0xFFFF) << 16) | ((tag & 0xFF) << 8)
            | (lower_address & 0x7F))


# ---------------------------------------------------------------------------
# Socket model
# ---------------------------------------------------------------------------
# Socket plays pcie_rq_rc_top's user-side ports for test_pcie_enum_scan.py,
# test_pcie_enum_bus.py and test_pcie_enum_bar.py, whose DUT ends there: it
# accepts RQ packets, strobes a tag for each, and drives completions and
# completion-timeout strobes. In the real core the tag is allocated before
# tlp_requester builds the request TLP, so no completion or timeout can
# precede the tag strobe. The model checks this ordering instead of
# assuming it, and raises AssertionError if an invariant fails:
#   1. no completion is delivered for a tag that has not been strobed;
#   2. no timeout strobe fires for an allocated tag not yet strobed;
#   3. the tag strobe comes at least one cycle after the command is accepted.
# test_pcie_enum_txn.py defines its own Socket, which waits for the tag
# strobe as this one does but does not check invariant 3.
import cocotb                                          # noqa: E402
from cocotb.triggers import ReadOnly, RisingEdge       # noqa: E402


class SocketRequest:
    """One RQ packet the DUT drove, plus the tag the socket gave it."""

    def __init__(self, beats, tag, accept_cycle):
        """Keep the beats and decode the descriptor beat (beat 0)."""
        self.beats = beats
        self.tag = tag
        self.accept_cycle = accept_cycle
        self.desc = beats[0][0] & ((1 << 128) - 1)
        self.tkeep = beats[0][1]
        self.tuser = beats[0][3]
        self.payload = [b[0] & 0xFFFFFFFF for b in beats[1:]]
        self.write = len(beats) > 1

    def __repr__(self):
        """Render as CfgWr0 or CfgRd0 with tag, register number and descriptor."""
        kind = "CfgWr0" if self.write else "CfgRd0"
        return (f"{kind}(tag={self.tag:#04x}, reg={(self.desc >> 2) & 0x3F:#04x}, "
                f"desc=0x{self.desc:032X})")


class Socket:
    """pcie_rq_rc_top's user-side socket, played in Python; see the section header."""

    def __init__(self, dut, tag_delay=2, first_tag=0x5A):
        """tag_delay is the number of cycles from command accept to the tag
        strobe and must be at least 1 (invariant 3); tags are handed out
        from first_tag upward, modulo 256."""
        assert tag_delay >= 1, (
            "INVARIANT 3: tag_delay must be >= 1. The core cannot present the "
            "tag in the cycle the descriptor is accepted -- it allocates in "
            "REQ_TAG a cycle or more later (tlp_requester.sv:211, 215-218), "
            "which is why the socket pairs the tag with its own strobe.")
        self.dut = dut
        self.tag_delay = tag_delay
        self.requests = []
        self.tags = []
        self.strobed = {}          # tag -> cycle the strobe was driven
        self.cycle = 0
        self._next_tag = first_tag
        self._stall_left = 0

    def start(self):
        """Start the cycle counter and the RQ capture coroutine."""
        cocotb.start_soon(self._cycle_counter())
        cocotb.start_soon(self._rq())

    def stall_beats(self, cycles):
        """Hold s_axis_rq_tready low for `cycles` cycles, starting now."""
        self._stall_left = cycles

    async def _cycle_counter(self):
        """Count rising edges of clk_i in self.cycle."""
        while True:
            await RisingEdge(self.dut.clk_i)
            self.cycle += 1

    async def wait_for(self, count, cycles=6000):
        """Wait for `count` RQ packets; raise after `cycles` cycles."""
        for _ in range(cycles):
            await RisingEdge(self.dut.clk_i)
            if len(self.requests) >= count:
                return
        raise AssertionError(
            f"expected {count} RQ packets, saw {len(self.requests)}: {self.requests}")

    async def _rq(self):
        """Drive s_axis_rq_tready_i and capture each RQ packet.

        tready is high in reset and low while a stall_beats() count runs. The
        first beat of a packet arms its tag; the tlast beat completes a
        SocketRequest.
        """
        d = self.dut
        beats = []
        while True:
            await RisingEdge(d.clk_i)
            if int(d.rst_i.value):
                d.s_axis_rq_tready_i.value = 1
                beats = []
                continue
            ready = 0 if self._stall_left > 0 else 1
            if self._stall_left > 0:
                self._stall_left -= 1
            d.s_axis_rq_tready_i.value = ready
            await ReadOnly()
            if ready and int(d.s_axis_rq_tvalid_o.value):
                beats.append((int(d.s_axis_rq_tdata_o.value),
                              int(d.s_axis_rq_tkeep_o.value),
                              int(d.s_axis_rq_tlast_o.value),
                              int(d.s_axis_rq_tuser_o.value)))
                if len(beats) == 1:
                    self._accept_cycle = self.cycle
                    self._arm_tag()
                if beats[-1][2]:
                    self.requests.append(
                        SocketRequest(beats, self.tags[-1], self._accept_cycle))
                    beats = []

    def _arm_tag(self):
        """Hand out the next tag and schedule its strobe."""
        tag = self._next_tag
        self._next_tag = (self._next_tag + 1) & 0xFF
        self.tags.append(tag)
        cocotb.start_soon(self._strobe_tag(tag, self.cycle))

    async def _strobe_tag(self, tag, accept_cycle):
        """Drive `tag` on pcie_rq_tag_i with a one-cycle pcie_rq_tag_vld_i
        strobe, tag_delay cycles after accept, and record the cycle in
        self.strobed."""
        d = self.dut
        for _ in range(self.tag_delay):
            await RisingEdge(d.clk_i)
        # Invariant 3: the strobe may not share the accept cycle.
        assert self.cycle > accept_cycle, (
            f"INVARIANT 3 violated: tag {tag:#04x} strobed in the same cycle "
            f"the descriptor was accepted ({accept_cycle}). The real core "
            "cannot do this (pcie_rq_rc_top.sv:51-60).")
        d.pcie_rq_tag_i.value = tag
        d.pcie_rq_tag_vld_i.value = 1
        await RisingEdge(d.clk_i)
        d.pcie_rq_tag_vld_i.value = 0
        self.strobed[tag] = self.cycle

    async def _await_strobe(self, tag, cycles=400):
        """Block until this request's tag strobe has actually been driven.

        This enforces invariants 1 and 2: in the real core no completion or
        timeout for a tag can exist before its tag strobe.
        """
        for _ in range(cycles):
            if tag in self.strobed:
                return
            await RisingEdge(self.dut.clk_i)
        raise AssertionError(
            f"INVARIANT 1: tag {tag:#04x} was never strobed, so no completion "
            "for it can legally be delivered. The socket model is broken, not "
            "the DUT.")

    async def complete(self, req=None, tag=None, status=CPL_SC, data=None,
                       request_completed=1, dword_count=None, payload=None,
                       byte_count=None, error_code=None):
        """Deliver one completion on the RC stream.

        With req given, wait for its tag strobe first (invariant 1). A
        Successful Completion to a read carries one Dword, `data` or
        0xD0000000 | tag by default; any other completion carries none.
        """
        if req is not None:
            await self._await_strobe(req.tag)          # invariant 1
        if tag is None:
            tag = req.tag
        is_read = (req is not None) and (not req.write)
        has_data = is_read and status == CPL_SC
        if payload is None:
            payload = [0xD0000000 | tag if data is None else data] if has_data else []
        if dword_count is None:
            dword_count = len(payload)
        desc = encode_rc_desc(
            tag=tag, status=status, dword_count=dword_count,
            request_completed=request_completed, byte_count=byte_count,
            error_code=error_code)
        await self._drive_rc(rc_beats(desc, payload))

    async def _drive_rc(self, beats):
        """Drive RC beats on m_axis_rc_*_i, holding each until it is accepted."""
        d = self.dut
        for tdata, tkeep, tlast in beats:
            d.m_axis_rc_tdata_i.value = tdata
            d.m_axis_rc_tkeep_i.value = tkeep
            d.m_axis_rc_tlast_i.value = tlast
            d.m_axis_rc_tvalid_i.value = 1
            # pcie_cfg_txn ties m_axis_rc_tready_o high, but the socket still
            # holds each beat until it is high and raises after 4000 cycles.
            for _ in range(4000):
                await ReadOnly()
                fired = int(d.m_axis_rc_tready_o.value) == 1
                await RisingEdge(d.clk_i)
                if fired:
                    break
            else:
                raise AssertionError("m_axis_rc_tready_o never asserted")
        d.m_axis_rc_tvalid_i.value = 0
        d.m_axis_rc_tlast_i.value = 0

    async def fire_timeout(self, tag):
        """One-cycle strobe on cpl_timeout_valid_i naming `tag`.

        It stands for pcie_rq_rc_top's cpl_timeout_valid_o. Invariant 2:
        tlp_request_tracker cannot time out a tag it has not allocated, so a
        strobe for a tag this socket handed out waits for that tag's strobe
        first. A tag the socket never handed out fires at once; that is
        deliberate stimulus, not an ordering violation.
        """
        d = self.dut
        if tag in self.tags:
            await self._await_strobe(tag)
        d.cpl_timeout_tag_i.value = tag
        d.cpl_timeout_valid_i.value = 1
        await RisingEdge(d.clk_i)
        d.cpl_timeout_valid_i.value = 0


# ---------------------------------------------------------------------------
# Integration-bench values
# ---------------------------------------------------------------------------
# Constants and the request decoder shared by the _tlp benches, which put a
# real pcie_rq_rc_top behind the DUT, and by the pcie_enum_dl_top bench; the
# standalone benches import some of them too. Kept in the benches instead:
#   settle()      defaults differ (20, 30 or 40 cycles), and settle() always
#                 runs its full count, so its default sets simulation time;
#                 the waiters here return as soon as their condition holds.
#   init()        the benches drive different DUT port sets.
#   completers    ConfigCompleter, ConfigSpaceCompleter, BarSpaceCompleter
#                 and BridgedCompleter below share only the four-name
#                 interface .start / .seen / .wait_for / .complete.
#   send_cmd(), recv_rsp()  drive pcie_cfg_txn's command port, which only
#                 the transaction benches expose.

# Clock period, completion timeout, the RC's own Requester ID and the target.
CLK_NS = 4
CPL_TIMEOUT_CYCLES = 4096       # the benches' own value: each
                                # tb_pcie_enum_*_tlp.sv wrapper and
                                # tb_pcie_enum_dl_top.sv pass 4096 explicitly;
                                # the shipped default is tlp_pkg's
                                # CPL_TIMEOUT_DEFAULT_CYCLES, 10 ms at 8 ns
RID = 0x1234                    # the Root Complex's own requester_id_i
BDF = 0x0100                    # the target: bus 1, device 0, function 0
BUS, DEV, FN = 0x01, 0x00, 0x00

# pcie_enum_pkg::enum_error_e
ENUM_ERR_NONE = 0
ENUM_ERR_UR_POST_PROBE = 1
ENUM_ERR_CA = 2
ENUM_ERR_CRS_EXHAUSTED = 3
ENUM_ERR_TIMEOUT = 4
# The BAR-stage codes of pcie_enum_bar; enum_error_e is 4 bits wide to hold
# them.
ENUM_ERR_BAR_TYPE = 5
ENUM_ERR_BAR_SIZE = 6
ENUM_ERR_BAR_WINDOW = 7
ENUM_ERR_BAR_ADDR32 = 8
# A completion timeout on a request the credit gate was holding when it
# expired. It has its own code, so enum_error_code_o alone does not report a
# dead device; err_credit_blocked_o is still set with it.
ENUM_ERR_CREDIT_STARVED = 9

ERR_NAME = {
    ENUM_ERR_NONE: "ENUM_ERR_NONE",
    ENUM_ERR_UR_POST_PROBE: "ENUM_ERR_UR_POST_PROBE",
    ENUM_ERR_CA: "ENUM_ERR_CA",
    ENUM_ERR_CRS_EXHAUSTED: "ENUM_ERR_CRS_EXHAUSTED",
    ENUM_ERR_TIMEOUT: "ENUM_ERR_TIMEOUT",
    ENUM_ERR_BAR_TYPE: "ENUM_ERR_BAR_TYPE",
    ENUM_ERR_BAR_SIZE: "ENUM_ERR_BAR_SIZE",
    ENUM_ERR_BAR_WINDOW: "ENUM_ERR_BAR_WINDOW",
    ENUM_ERR_BAR_ADDR32: "ENUM_ERR_BAR_ADDR32",
    ENUM_ERR_CREDIT_STARVED: "ENUM_ERR_CREDIT_STARVED",
}


def err_name(value):
    """enum_error_e value as its name, for assertion messages."""
    return ERR_NAME.get(value, f"<unknown {value}>")


# The BAR allocator's window: the MEM_BAR_BASE and MEM_BAR_WINDOW parameter
# defaults of pcie_enum_bar. The addresses the BAR tests expect derive from
# MEM_BAR_BASE.
MEM_BAR_BASE = 0x0000_0000_8000_0000
MEM_BAR_WINDOW = 0x0000_0000_1000_0000

# The Command register value enumeration writes last, pcie_enum_pkg's
# CMD_ENABLE_VALUE: Memory Space Enable (bit 1) and Bus Master Enable (bit 2)
# (PCI Local Bus Spec r3.0, §6.2.2). I/O Space Enable stays 0 because
# pcie_enum_bar assigns no I/O BAR.
CMD_ENABLE_VALUE = 0x0000_0006


class TlpRequest:
    """One request TLP observed leaving the Transaction Layer.

    Decodes the three header Dwords, in the layout of the on-wire goldens,
    into named fields; dwords[3:] is the payload. Both the type and register
    fields and the routing Dword's Bus, Device and Function are decoded, so a
    test can check what a request does and where it is routed.
    """

    def __init__(self, dwords):
        """Decode header DW0..DW2 of `dwords`; the rest is payload."""
        dw0, dw1, dw2 = dwords[0], dwords[1], dwords[2]
        self.dwords = dwords
        self.dw0, self.dw1, self.dw2 = dw0, dw1, dw2
        self.fmt = (dw0 >> 5) & 0x7
        self.tlp_type = dw0 & 0x1F
        self.length_dw = dw0_length(dw0)
        self.requester_id = (dw1 >> 16) & 0xFFFF
        self.tag = (dw1 >> 8) & 0xFF
        self.last_be = (dw1 >> 4) & 0xF
        self.first_be = dw1 & 0xF
        self.reg_num = (dw2 >> 2) & 0x3F
        self.ext_reg = (dw2 >> 8) & 0xF
        self.bus = (dw2 >> 24) & 0xFF
        self.dev = (dw2 >> 19) & 0x1F
        self.fn = (dw2 >> 16) & 0x7
        self.payload = dwords[3:]
        self.is_read = (self.fmt & 0b010) == 0

    def __repr__(self):
        """Render with type, tag, BDF, register, first_be and payload."""
        kind = "Rd" if self.is_read else "Wr"
        # The type comes from Type[4:0], so a failure message never prints a
        # Type 1 request as Type 0.
        t = "1" if self.tlp_type == TYPE_CFG1 else "0"
        return (f"Cfg{kind}{t}(tag={self.tag:#04x}, "
                f"bdf={self.bus:02x}:{self.dev:02x}.{self.fn}, "
                f"reg={self.reg_num:#04x}, fbe={self.first_be:#06b}, "
                f"payload={[hex(w) for w in self.payload]})")


# ---------------------------------------------------------------------------
# Flow control
# ---------------------------------------------------------------------------
# Drivers for pcie_rq_rc_top's credit inputs fc_*_i. tlp_credit_manager
# loads each value as the credit limit of its pool: the cumulative
# CREDITS_ALLOCATED count a Receiver advertises in InitFC and UpdateFC
# DLLPs, modulo 2^8 for headers and 2^12 for data (PCIe Base Spec r2.1,
# §2.6.1.2). set_credits drives all six pool values at once; CreditDrip
# returns non-posted credit a little at a time.
def set_credits(dut, ph=0xFF, pd=0xFFF, nph=0xFF, npd=0xFFF, cplh=0xFF, cpld=0xFFF):
    """Drive the six fc_*_i pool values; the defaults are the field maxima."""
    dut.fc_ph_i.value = ph
    dut.fc_pd_i.value = pd
    dut.fc_nph_i.value = nph
    dut.fc_npd_i.value = npd
    dut.fc_cplh_i.value = cplh
    dut.fc_cpld_i.value = cpld


class CreditDrip:
    """A Receiver returning non-posted credit the way a real one does: cumulatively.

    Every `period` cycles it raises the NPH and NPD totals by `step`, modulo
    the field size, drives them on fc_nph_i and fc_npd_i, and pulses
    fc_update_valid_i. fc_*_i carries the raw CREDITS_ALLOCATED value, so an
    update advertises a running total. A drip that repeated a constant would
    stop the transmitter once that many credits were consumed, which looks
    the same as a DUT deadlock.
    """

    def __init__(self, dut, nph=1, npd=1, period=40, step=1):
        """nph and npd are the totals before the first update."""
        self.dut = dut
        self.nph = nph
        self.npd = npd
        self.period = period
        self.step = step
        self.updates = 0
        self._run_flag = True

    def start(self):
        """Start the drip coroutine."""
        cocotb.start_soon(self._run())

    def stop(self):
        """Stop sending updates; the coroutine keeps waiting but drives nothing."""
        self._run_flag = False

    async def _run(self):
        """Each `period` cycles, advertise the next totals with a one-cycle strobe."""
        d = self.dut
        while True:
            for _ in range(self.period):
                await RisingEdge(d.clk_i)
            if not self._run_flag:
                continue
            self.nph = (self.nph + self.step) & 0xFF
            self.npd = (self.npd + self.step) & 0xFFF
            d.fc_nph_i.value = self.nph
            d.fc_npd_i.value = self.npd
            d.fc_update_valid_i.value = 1
            await RisingEdge(d.clk_i)
            d.fc_update_valid_i.value = 0
            self.updates += 1


# ---------------------------------------------------------------------------
# Monitor and wire check
# ---------------------------------------------------------------------------
# Mon records, every cycle, the error and event strobes of pcie_rq_rc_top
# that the integration-bench wrappers expose, and clean() asserts that none
# fired beyond what a test allows. assert_cfg_tlp_on_wire checks one emitted
# Configuration Request against the on-wire goldens, Dword by Dword; the
# scan and transaction _tlp benches wrap it in a local assert_on_wire.
class Mon:
    """Error and event outputs of pcie_rq_rc_top, sampled every cycle.

    Sampled: the tag strobe, the RQ, RC, command and TX error outputs,
    unexpected completions, completion timeouts, late completions,
    credit_error_o, tx_fc_blocked_o and s_axis_rq_tvalid. Not sampled,
    among others: the gearbox, CQ, CC and RX error outputs.

    wait_timeouts and wait_lates return on the cycle the expected count is
    reached and raise only when their bound runs out, so a larger bound
    adds simulation time only to a failing test.
    """

    def __init__(self, dut):
        """Start with empty observation lists; start() begins sampling."""
        self.dut = dut
        self.tags_presented = []
        self.rq_errors = []
        self.rc_errors = []
        self.unexpected = []
        self.command_errors = []
        self.tx_errors = []
        self.timeouts = []
        self.lates = []
        self.credit_errors = 0
        self.blocked_seen = False
        self.rq_tvalid_seen = False

    def start(self):
        """Start the sampling coroutine."""
        cocotb.start_soon(self._run())

    async def _run(self):
        """Sample in the ReadOnly phase after each rising edge, outside reset."""
        d = self.dut
        while True:
            await RisingEdge(d.clk_i)
            await ReadOnly()
            if int(d.rst_i.value):
                continue
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
            if int(d.tx_error_valid_o.value):
                self.tx_errors.append(int(d.tx_error_code_o.value))
            if int(d.cpl_timeout_valid_o.value):
                self.timeouts.append(int(d.cpl_timeout_tag_o.value))
            if int(d.late_cpl_valid_o.value):
                self.lates.append(int(d.late_cpl_tag_o.value))
            if int(d.credit_error_o.value):
                self.credit_errors += 1
            if int(d.tx_fc_blocked_o.value):
                self.blocked_seen = True
            if int(d.s_axis_rq_tvalid.value):
                self.rq_tvalid_seen = True

    async def wait_timeouts(self, count, cycles=CPL_TIMEOUT_CYCLES + 900):
        """Wait for `count` completion timeouts; raise after `cycles` cycles."""
        for _ in range(cycles):
            await RisingEdge(self.dut.clk_i)
            if len(self.timeouts) >= count:
                return
        raise AssertionError(
            f"expected {count} cpl_timeout strobes, saw {len(self.timeouts)}")

    async def wait_lates(self, count, cycles=600):
        """Wait for `count` late-completion strobes; raise after `cycles` cycles."""
        for _ in range(cycles):
            await RisingEdge(self.dut.clk_i)
            if len(self.lates) >= count:
                return
        raise AssertionError(
            f"expected {count} late_cpl strobes, saw {len(self.lates)}")

    def clean(self, allow_timeouts=False, allow_orphans=False,
              allow_unexpected=False):
        """Assert that nothing fired that the test did not allow.

        credit_error_o must be silent in every case: tlp_credit_manager
        raises it only for a request that needs more data credit than the
        whole advertised pool, and a one-Dword configuration request never
        does.
        """
        assert self.rq_errors == [], f"RQ protocol errors: {self.rq_errors}"
        if not allow_unexpected:
            assert self.unexpected == [], \
                f"unexpected completions: {self.unexpected}"
        assert self.command_errors == [], f"TL command errors: {self.command_errors}"
        assert self.tx_errors == [], f"TX errors: {self.tx_errors}"
        assert self.credit_errors == 0, \
            (f"credit_error_o fired {self.credit_errors}x -- a one-Dword config "
             "request cannot exhaust an entire data advertisement")
        if not allow_orphans:
            assert self.rc_errors == [], f"RC protocol errors: {self.rc_errors}"
        if not allow_timeouts:
            assert self.timeouts == [], f"completion timeouts: {self.timeouts}"
            assert self.lates == [], f"late completions drained: {self.lates}"


def assert_cfg_tlp_on_wire(req, *, write, reg_num, first_be, tag, what="",
                           require_device0=False, type1=False, bus=None):
    """Assert one emitted Configuration TLP against the on-wire goldens.

    DW0, DW1 and DW2 are compared whole. Every Configuration Request has a
    Length of 1 Dword, Last DW BE 0000b and TC, Attr and AT zero (PCIe Base
    Spec r2.1, §2.2.7), and DW2 packs the BDF as Figure 2-18 shows. The
    whole-Dword compare of DW0 also checks Type[0], the one bit between a
    Type 0 and a Type 1 request.

    require_device0 adds the check that the request names Device 0,
    Function 0; on a Link only Device 0 is reachable (PCIe Base Spec r2.1,
    §7.3.1). It is off by default, so each bench opts in.

    type1 selects the Type 1 DW0 golden. bus overrides the bus field of DW2
    (default BUS); the golden Device and Function are 0 on every bus.
    """
    exp0 = cfg_wire_dw0(write=write, length_dw=1, type1=type1)
    exp1 = cfg_wire_dw1(RID, tag, first_be)
    exp2 = cfg_wire_dw2(BUS if bus is None else bus, DEV, FN, reg_num)
    assert req.dw0 == exp0, \
        f"{what}DW0 {req.dw0:#010x} != golden {exp0:#010x}"
    assert req.dw1 == exp1, \
        f"{what}DW1 {req.dw1:#010x} != golden {exp1:#010x} (rid/tag/BE)"
    assert req.dw2 == exp2, \
        f"{what}DW2 {req.dw2:#010x} != golden {exp2:#010x} (BDF routing Dword)"
    assert req.length_dw == 1, \
        f"{what}Length {req.length_dw} != 1 (Base 2.1 SS2.2.7 p.79)"
    assert req.last_be == 0, \
        f"{what}Last DW BE {req.last_be:#06b} != 0000b (Base 2.1 SS2.2.7 p.79)"
    if require_device0:
        assert req.dev == 0 and req.fn == 0, (
            f"{what}the request named device {req.dev} function {req.fn}. Only "
            "device 0 may be probed on this link (SS7.3.1 p.479)")


# ---------------------------------------------------------------------------
# Golden device
# ---------------------------------------------------------------------------
# The Type 0 device the enumeration benches model: the bus it sits on, its
# Vendor and Device IDs, the header type codes and Configuration register 3.
# The scan, BAR, bridge and pcie_enum_dl_top benches use this one description
# wherever they enumerate a plain Type 0 device. outcome_name renders a
# pcie_cfg_txn outcome for assertion messages.
SCAN_BUS = 0x01

# Not FFFFh, which is an invalid Vendor ID (PCI Local Bus Spec r3.0, §6.2.1);
# pcie_enum_scan takes absence from a UR completion, not from this value.
VENDOR = 0x144D
DEVICE = 0xA80A
REG0 = (DEVICE << 16) | VENDOR

HDR_TYPE0 = 0x00            # endpoint Function
HDR_TYPE0_MF = 0x80         # endpoint Function, multi-function (bit 7)
HDR_TYPE1 = 0x01            # PCI-to-PCI bridge (PCI Local Bus Spec r3.0, §6.2.1)


def reg3(header_type, bist=0x00, mlt=0x00, cls=0x10):
    """Configuration register 3 (byte offset 0Ch).

    {BIST[31:24], Header Type[23:16], Master Latency Timer[15:8],
     Cache Line Size[7:0]} (PCIe Base Spec r2.1, §7.5.2, Figure 7-5).
    """
    return (bist << 24) | ((header_type & 0xFF) << 16) | (mlt << 8) | cls


def outcome_name(value):
    """pcie_enum_pkg::txn_outcome_e as a name, for assertion messages."""
    return TXN_NAME.get(value, f"<unknown {value}>")


# ---------------------------------------------------------------------------
# Configuration space
# ---------------------------------------------------------------------------
# ConfigDevice is a Type 0 configuration space whose BARs behave as PCI Local
# Bus Spec r3.0, §6.2.5.1 describes: bits 3:0 of a memory BAR and bits 1:0
# of an I/O BAR are read-only, and the address bits below the BAR's size
# read back as 0, so writing all ones and reading back yields the size and
# keeps the type field. A model that stored writes verbatim would read back
# FFFFFFFFh, whose bit 0 marks an I/O BAR, and the DUT would appear broken
# when the bench is. The model counts the writes its mask altered
# (mask_hits, ro_low_hits), so a test can show the mask acted;
# assert_mask_exercised checks it. BarSpec describes one implemented BAR.

BAR_MEM32 = "mem32"
BAR_MEM64 = "mem64"
BAR_IO = "io"


class BarSpec:
    """One Base Address register as a device implements it.

    size is in bytes and must be a power of two (PCI Local Bus Spec r3.0,
    §6.2.5.1). A BAR_MEM64 spec occupies the candidate register it is placed
    at and the next one; ConfigDevice fills in the upper half, so a test
    names only the lower register.
    """

    def __init__(self, kind, size, prefetch=False):
        """kind is BAR_MEM32, BAR_MEM64 or BAR_IO; prefetch sets memory BAR bit 3."""
        assert kind in (BAR_MEM32, BAR_MEM64, BAR_IO), kind
        assert size > 0 and (size & (size - 1)) == 0, \
            f"BAR size {size:#x} is not a power of two -- [PCI3] p.226 :11226"
        self.kind = kind
        self.size = size
        self.prefetch = prefetch

    @property
    def type_field(self):
        """Bits [3:0], the read-only field (PCI Local Bus Spec r3.0, §6.2.5.1)."""
        if self.kind == BAR_IO:
            return 0b0001                     # bit 0 = 1, bit 1 reserved reads 0
        bits = 0b0000 if self.kind == BAR_MEM32 else 0b0100   # [2:1] = 00 or 10
        return bits | (0b1000 if self.prefetch else 0)

    @property
    def registers(self):
        """Number of Configuration registers the BAR occupies: 2 for BAR_MEM64."""
        return 2 if self.kind == BAR_MEM64 else 1


class ConfigDevice:
    """A Type 0 configuration space the enumeration benches can enumerate.

    bars is a mapping {candidate register number: BarSpec}. Every candidate
    register not named is unimplemented and reads 0 (PCI Local Bus Spec
    r3.0, §6.2.5.1); the upper half of a BAR_MEM64 is filled in
    automatically and may not be named separately.

    Besides the BARs, the model implements Vendor/Device ID, Command/Status
    and register 3. read() returns None for any other register that nothing
    has written, register 2 (Revision ID, class code) included, and the
    completers answer such a read with Unsupported Request. This is a bench
    convention: under PCI Local Bus Spec r3.0, §6.1 a read of an
    unimplemented register completes normally and returns 0. A write to any
    other register is stored and reads back.
    """

    def __init__(self, bars=None, header_type=HDR_TYPE0, vendor=VENDOR,
                 device=DEVICE, raw=None):
        # raw is {register: fixed readback value} and models a malformed
        # device: such a register answers the same value whatever is written,
        # which is the only way to present an encoding BarSpec cannot build,
        # such as a Reserved Type field or a readback whose implied size is
        # not a power of two. A raw register ignores writes, so the all-ones
        # sizing write cannot replace the value under test.
        self._raw = dict(raw or {})
        self.raw_reads = 0
        self.raw_writes_discarded = 0
        self.vendor = vendor
        self.device = device
        self.header_type = header_type
        self.mask_hits = 0        # a write had bits dropped by the size mask
        self.ro_low_hits = 0      # ...and specifically in the read-only bits 3:0
        self.writes = []          # (reg, value, first_be), in order

        # reg -> (read_only_bits, writable_mask)
        self._bar = {}
        # reg -> stored value, already masked
        self._stored = {}
        # ordinary registers, byte-writable
        self._plain = {
            CFG_REG_VENDOR_DEVICE: (device << 16) | vendor,
            CFG_REG_CACHE_HEADER: reg3(header_type),
            CFG_REG_COMMAND_STATUS: 0x0000_0000,
        }

        for reg in range(CFG_REG_BAR_FIRST, CFG_REG_BAR_LAST + 1):
            self._stored[reg] = 0

        for reg, spec in sorted((bars or {}).items()):
            assert CFG_REG_BAR_FIRST <= reg <= CFG_REG_BAR_LAST, \
                f"register {reg} is not a candidate BAR register"
            assert reg not in self._bar, f"register {reg} already claimed"
            if spec.kind == BAR_IO:
                # 32 bits wide always, bit 0 hardwired 1, bit 1 reserved reads 0.
                mask = (~(spec.size - 1)) & 0xFFFF_FFFC
                self._bar[reg] = (spec.type_field, mask)
            elif spec.kind == BAR_MEM32:
                mask = (~(spec.size - 1)) & 0xFFFF_FFFF
                self._bar[reg] = (spec.type_field, mask)
            else:
                assert reg < CFG_REG_BAR_LAST, (
                    f"a 64-bit BAR at register {reg} has no register to pair "
                    "with -- offset 28h is the Cardbus CIS Pointer")
                assert (reg + 1) not in self._bar, \
                    f"register {reg + 1} is the upper half of the pair at {reg}"
                mask64 = (~(spec.size - 1)) & 0xFFFF_FFFF_FFFF_FFFF
                self._bar[reg] = (spec.type_field, mask64 & 0xFFFF_FFFF)
                self._bar[reg + 1] = (0, (mask64 >> 32) & 0xFFFF_FFFF)

    # ---- the RC's view -----------------------------------------------------
    def read(self, reg):
        """Readback of register `reg`, or None where the completer answers UR."""
        if reg in self._raw:
            self.raw_reads += 1
            return self._raw[reg]         # malformed: fixed, write-immune
        if reg in self._bar:
            ro, mask = self._bar[reg]
            return (self._stored[reg] & mask) | ro
        if reg in self._stored:
            return 0                      # unimplemented: hardwired zero
        return self._plain.get(reg)       # None -> the completer answers UR

    def write(self, reg, value, first_be=CFG_BE_DWORD):
        """Apply a Configuration Write to `reg` under byte enables first_be."""
        self.writes.append((reg, value & 0xFFFF_FFFF, first_be))
        if reg in self._raw:
            self.raw_writes_discarded += 1
            return                        # a fixed-response register absorbs it
        byte_mask = 0
        for byte in range(4):
            if (first_be >> byte) & 1:
                byte_mask |= 0xFF << (8 * byte)

        if reg in self._bar:
            _ro, mask = self._bar[reg]
            effective = mask & byte_mask
            dropped = value & byte_mask & ~effective & 0xFFFF_FFFF
            if dropped:
                self.mask_hits += 1
                if dropped & 0xF:
                    self.ro_low_hits += 1
            self._stored[reg] = ((self._stored[reg] & ~effective)
                                 | (value & effective)) & 0xFFFF_FFFF
        elif reg in self._stored:
            pass                          # unimplemented: writes are discarded
        else:
            current = self._plain.get(reg, 0)
            self._plain[reg] = ((current & ~byte_mask)
                                | (value & byte_mask)) & 0xFFFF_FFFF

    # ---- what a test asserts against --------------------------------------
    @property
    def command(self):
        """The low half of register 1 -- the Command register."""
        return self._plain[CFG_REG_COMMAND_STATUS] & 0xFFFF

    def bar_written(self, reg):
        """The raw stored value of one BAR register, mask applied."""
        return self._stored[reg]

    def assert_mask_exercised(self, what=""):
        """Assert that the BAR write mask altered a write in a read-only low field.

        A completer that echoed BAR writes verbatim would return a wrong
        sizing readback, so the mask is shown to have acted, not assumed.
        ro_low_hits counts a subset of the writes mask_hits counts, so this
        one assertion also implies mask_hits > 0, and an added assertion on
        mask_hits could not change the verdict. mask_hits appears in the
        message as a diagnostic.
        """
        assert self.ro_low_hits > 0, (
            f"{what}no write was ever masked inside the read-only low field, so "
            "the BAR write mask never protected it ([PCI3] p.226 :11205 for a "
            "memory BAR's bits 3:0, p.225 :11187 for an I/O BAR's bits 1:0). "
            "This test would pass against a completer that echoed writes "
            "verbatim, which is the bug the mask exists to model. "
            f"(mask_hits={self.mask_hits}, ro_low_hits={self.ro_low_hits})")


# ---------------------------------------------------------------------------
# Empty-set guards
# ---------------------------------------------------------------------------
# An assertion over an empty collection passes without checking anything.
# nonempty fails instead when nothing was collected, as does expect_count
# for a non-zero count, and assert_sequence also rejects an empty golden.
# The BAR, bridge and pcie_enum_dl_top benches pass a collected list through
# one of these before asserting over it.
def nonempty(seq, what):
    """Return seq, having proved it has something in it."""
    items = list(seq)
    assert items, (
        f"{what}: the observation set is EMPTY, so every assertion over it "
        "would pass vacuously. Nothing was collected -- check the DUT emitted "
        "anything at all before trusting a green result.")
    return items


def expect_count(seq, count, what):
    """Return seq as a list, having checked it holds exactly `count` items.

    For a non-zero count the empty case fails first, through nonempty().
    """
    items = nonempty(seq, what) if count else list(seq)
    assert len(items) == count, (
        f"{what}: expected exactly {count} item(s), saw {len(items)}:\n  "
        + "\n  ".join(repr(i) for i in items[:24]))
    return items


def assert_sequence(observed, golden, what="", render=repr):
    """Whole-sequence compare with an empty-set guard and a first-diff report.

    Used for the transaction sequence of a whole enumeration run. An empty
    golden fails first, since it would make the compare assert nothing. The
    items are then compared pairwise, and the first difference is reported
    with the whole observed sequence. Last, the lengths must match, so an
    empty or short observed sequence still fails after the pairwise loop.
    """
    golden = list(golden)
    assert golden, (
        f"{what}: the GOLDEN sequence is empty, so this comparison asserts "
        "nothing. That is a bench bug, not a DUT result.")
    observed = list(observed)
    for index, (got, exp) in enumerate(zip(observed, golden)):
        if got != exp:
            raise AssertionError(
                f"{what}: transaction {index} differs\n"
                f"  observed {render(got)}\n"
                f"  golden   {render(exp)}\n"
                f"  (full observed sequence, {len(observed)} items:)\n    "
                + "\n    ".join(render(o) for o in observed))
    assert len(observed) == len(golden), (
        f"{what}: the first {min(len(observed), len(golden))} transactions "
        f"match but the lengths differ -- observed {len(observed)}, golden "
        f"{len(golden)}.\n  observed:\n    "
        + "\n    ".join(render(o) for o in observed)
        + "\n  golden:\n    " + "\n    ".join(render(g) for g in golden))


# ---------------------------------------------------------------------------
# Bridged topology
# ---------------------------------------------------------------------------
# One PCI-to-PCI bridge (Type 1 header) at 01:00.0 with one Endpoint behind
# it at 05:00.0, in three parts: BridgeConfigSpace, the bridge's own Type 1
# configuration space; BridgedTopology, a pure routing core with no cocotb in
# it; and BridgedCompleter, which serves the core's answers on the DUT's DLL
# streams. The core applies the routing rules of PCIe Base Spec r2.1, §7.3.3
# as written, including UR for a bus outside [Secondary, Subordinate]: both
# are 00h at reset, so a bus-number write wrongly sent as Type 1 is answered
# UR without a test written for that mistake. Completer IDs are captured as
# PCIe Base Spec r2.1, §2.2.9 requires, 0000h until the Function completes
# its first Type 0 Configuration Write; the ConfigSpaceCompleter and
# BarSpaceCompleter of the scan and BAR _tlp benches put BDF in every
# Completer ID instead. _selftest_bridged_topology runs at import.

# The value table. The four IDs differ from each other and from VENDOR and
# DEVICE, and none is FFFFh, so a read answered by the wrong Function cannot
# return the expected value. Bus numbers 1, 5 and 9 are not consecutive, so
# a bus number off by one matches none of them. SEC_BUS and SUB_BUS equal
# pcie_enum_pkg's SEC_BUS_NUMBER and SUB_BUS_NUMBER, which pcie_enum_bus
# writes. _selftest_bridged_topology checks the distinctness.
BRIDGE_BDF = 0x0100             # 01:00.0, the device the first scan probes
SEC_BUS = 0x05                  # Secondary: non-zero, != primary, != primary+1
SUB_BUS = 0x09                  # Subordinate: != Secondary
SEC_DEV_BDF = (SEC_BUS << 8)    # 05:00.0, Device 0 and Function 0 on Secondary
BRIDGE_VENDOR = 0x1AF4
BRIDGE_DEVICE = 0x1100
SEC_DEV_VENDOR = 0x15B3
SEC_DEV_DEVICE = 0x1017

# Register 6, byte offset 18h, the Type 1 bus-number Dword: {Secondary
# Latency Timer, Subordinate, Secondary, Primary} (PCIe Base Spec r2.1,
# §7.5.3, Figure 7-6).
CFG_REG_BUS_NUMBER = 0x06
# The value pcie_enum_bus writes. The latency byte is 00h because that
# register is read-only 00h (PCIe Base Spec r2.1, §7.5.3.3); the other three
# bytes differ from each other.
BUS_NUM_WDATA = 0x00090501      # {00, SUB_BUS, SEC_BUS, 0x01}

# Response latencies in cycles. A device answer takes BRIDGE_LATENCY +
# DEVICE_LATENCY (latency_for), so it arrives later than a bridge answer.
# Both are non-zero, so a request the DUT issues before an earlier
# completion arrives shows on the wire between the two; a zero latency
# would hide that ordering error.
BRIDGE_LATENCY = 5
DEVICE_LATENCY = 9


class BridgeConfigSpace:
    """A Type 1 configuration space (PCIe Base Spec r2.1, §7.5.3, Figure 7-6).

    It has two BARs, registers 4 and 5 (offsets 10h and 14h, PCIe Base Spec
    r2.1, §7.5.3.1); register 6, where a Type 0 header has BAR2, is the
    bus-number Dword. Both BARs are unimplemented and read 0: the bridge
    requests no memory range in this topology.

    Register 6:
      [31:24] Secondary Latency Timer, read-only 00h (PCIe Base Spec r2.1,
              §7.5.3.3). Writes to the byte are ignored and counted in
              latency_byte_writes_ignored, so a test can show the case ran.
      [23:0]  Subordinate, Secondary and Primary Bus Number, byte-writable.
              Primary is read-write but not used by PCI Express Functions
              (PCIe Base Spec r2.1, §7.5.3.2); BridgedTopology never reads it.
    """

    def __init__(self, vendor=BRIDGE_VENDOR, device=BRIDGE_DEVICE):
        """Reset state: every bus number 00h, Command 0000h."""
        self.vendor = vendor
        self.device = device
        self.bus_reg = 0x0000_0000          # reset: Pri = Sec = Sub = 00h
        self.writes = []                    # (reg, value, first_be), in order
        self.latency_byte_writes_ignored = 0
        self._bars = (4, 5)                 # a Type 1 header has only these two BARs
        self._plain = {
            CFG_REG_VENDOR_DEVICE: (device << 16) | vendor,
            CFG_REG_COMMAND_STATUS: 0x0000_0000,
            CFG_REG_CACHE_HEADER: reg3(HDR_TYPE1),
        }

    @property
    def primary(self):
        """Primary Bus Number, register 6 bits [7:0]."""
        return self.bus_reg & 0xFF

    @property
    def secondary(self):
        """Secondary Bus Number, register 6 bits [15:8]."""
        return (self.bus_reg >> 8) & 0xFF

    @property
    def subordinate(self):
        """Subordinate Bus Number, register 6 bits [23:16]."""
        return (self.bus_reg >> 16) & 0xFF

    def read(self, reg):
        """Readback of register `reg`, or None where the completer answers UR."""
        if reg == CFG_REG_BUS_NUMBER:
            # [31:24] reads 00h whatever was written. The mask is applied on
            # read as well as on write, so the byte reads 00h even if the
            # store held it.
            return self.bus_reg & 0x00FF_FFFF
        if reg in self._bars:
            return 0                        # unimplemented: hardwired zero
        return self._plain.get(reg)         # None -> the completer answers UR

    def write(self, reg, value, first_be=CFG_BE_DWORD):
        """Apply a Configuration Write to `reg` under byte enables first_be."""
        self.writes.append((reg, value & 0xFFFF_FFFF, first_be))
        byte_mask = 0
        for byte in range(4):
            if (first_be >> byte) & 1:
                byte_mask |= 0xFF << (8 * byte)
        if reg == CFG_REG_BUS_NUMBER:
            if byte_mask & 0xFF00_0000:
                self.latency_byte_writes_ignored += 1
            effective = byte_mask & 0x00FF_FFFF   # byte 3 is read-only
            self.bus_reg = ((self.bus_reg & ~effective)
                            | (value & effective)) & 0x00FF_FFFF
        elif reg in self._bars:
            pass                            # unimplemented: writes discarded
        elif reg in self._plain:
            self._plain[reg] = ((self._plain[reg] & ~byte_mask)
                                | (value & byte_mask)) & 0xFFFF_FFFF
        # Registers this model does not implement (a real Type 1 header has
        # registers at 1Ch to 3Ch) absorb writes. Their reads return None,
        # which the completer answers with UR, as for ConfigDevice.

    @property
    def command(self):
        """The Command register, the low half of register 1."""
        return self._plain[CFG_REG_COMMAND_STATUS] & 0xFFFF


class BridgedTopology:
    """The routing core: pure Python with no cocotb, so it is self-tested at
    import time, as _selftest_type1_one_bit tests the builders.

    handle(dwords) takes one request in wire-Dword form and returns
      (who, status, rdata, completer_id)
    where who is "bridge" or "device", status a CPL_* value, rdata the read
    data (None unless a successful read) and completer_id the value the
    Completion's DW1 carries: the answering Function's captured ID, 0000h
    until its first Type 0 Configuration Write.

    A Type 1 request is routed by these tests in order (PCIe Base Spec r2.1,
    §7.3.3):
      1. bus == Secondary            -> Type[0] changed from 1 to 0 and
                                        nothing else, then delivered to the
                                        device. _transform asserts that DW0
                                        changed in bit 0 only and that DW1,
                                        DW2 and the payload did not change.
      2. Secondary < bus <= Subord.  -> forwarded unmodified. With one device
                                        modelled, it reaches the Endpoint as
                                        Type 1 and is answered UR. The DUT's
                                        second-level scan probes only
                                        Secondary, so only the self-test
                                        reaches this case.
      3. otherwise                   -> Unsupported Request, from the bridge.
    A Type 0 request is for the bridge itself, the RC's Link partner. On
    either Link a Device number other than 0 is answered UR (PCIe Base Spec
    r2.1, §7.3.1), and so is a Function number other than 0, since only
    Function 0 is implemented (PCIe Base Spec r2.1, §7.3.3). The bridge never
    answers CRS for a request it forwards: bridge_crs_once applies to the
    bridge's own registers only, and device_crs_once to the device's.
    """

    def __init__(self, bridge=None, device=None,
                 bridge_crs_once=(), device_crs_once=()):
        """bridge and device default to a fresh BridgeConfigSpace and a
        ConfigDevice with the secondary IDs. *_crs_once name the registers
        that answer CRS once, the first time they are accessed."""
        self.bridge = bridge if bridge is not None else BridgeConfigSpace()
        self.device = device if device is not None else ConfigDevice(
            vendor=SEC_DEV_VENDOR, device=SEC_DEV_DEVICE)
        self.bridge_captured_id = 0x0000    # 0000h until the first CfgWr0
        self.device_captured_id = 0x0000
        self.bridge_crs_once = set(bridge_crs_once)
        self.device_crs_once = set(device_crs_once)
        # A counter per guard arm, so a test can show the arm ran.
        self.transforms = []                # (received dwords, forwarded dwords)
        self.route_ur_hits = 0              # routing case 3: bus out of range
        self.forward_unmodified_hits = 0    # routing case 2
        self.bridge_dev_ur_hits = 0         # Type 0 naming device or function != 0
        self.device_dev_ur_hits = 0         # same rule, secondary link
        self.device_type1_ur_hits = 0       # Type 1 to the Endpoint: UR
        self.bridge_crs_hits = 0
        self.device_crs_hits = 0

    # ---- Configuration Request routing --------------------------------------
    def handle(self, dwords):
        """Route one request given as wire Dwords; see the class docstring."""
        req = TlpRequest(list(dwords))
        if req.tlp_type == TYPE_CFG0:
            return self._bridge_local(req)
        if req.tlp_type == TYPE_CFG1:
            sec, sub = self.bridge.secondary, self.bridge.subordinate
            if req.bus == sec:
                forwarded = self._transform(list(dwords))
                return self._device_claim(TlpRequest(forwarded))
            if sec < req.bus <= sub:
                self.forward_unmodified_hits += 1
                return self._device_claim(req)      # arrives as raw Type 1
            self.route_ur_hits += 1                 # case 3
            return ("bridge", CPL_UR, None, self.bridge_captured_id)
        raise AssertionError(
            f"the bridged topology received a non-config TLP "
            f"(tlp_type {req.tlp_type:#07b}): {req!r}")

    def _transform(self, dwords):
        """Routing case 1: turn the Type 1 request into Type 0 by its Type
        field alone (PCIe Base Spec r2.1, §7.3.3). The result is asserted,
        not assumed."""
        forwarded = list(dwords)
        forwarded[0] = dwords[0] ^ 0b1              # Type[0]: 1 -> 0, Table 2-3
        assert forwarded[0] ^ dwords[0] == 1, "transform touched more than bit 0"
        assert forwarded[0] & 0x1F == TYPE_CFG0
        assert forwarded[1:] == list(dwords)[1:], (
            "the transform modified DW1/DW2/payload -- SS7.3.3 p.481 requires "
            "all fields except Type[4:0] unchanged")
        self.transforms.append((list(dwords), forwarded))
        return forwarded

    # ---- the bridge as a Completer in its own right ------------------------
    def _bridge_local(self, req):
        """Answer a Type 0 request addressed to the bridge itself."""
        if req.dev != 0 or req.fn != 0:
            self.bridge_dev_ur_hits += 1            # only Device 0 on a Link
            return ("bridge", CPL_UR, None, self.bridge_captured_id)
        if req.reg_num in self.bridge_crs_once:
            self.bridge_crs_once.discard(req.reg_num)
            self.bridge_crs_hits += 1               # the bridge's own access only
            return ("bridge", CPL_CRS, None, self.bridge_captured_id)
        if not req.is_read:
            self.bridge.write(req.reg_num,
                              req.payload[0] if req.payload else 0,
                              req.first_be)
            cid = self.bridge_captured_id           # this completion: old ID
            self.bridge_captured_id = (req.bus << 8) | (req.dev << 3) | req.fn
            return ("bridge", CPL_SC, None, cid)
        value = self.bridge.read(req.reg_num)
        if value is None:
            return ("bridge", CPL_UR, None, self.bridge_captured_id)
        return ("bridge", CPL_SC, value, self.bridge_captured_id)

    # ---- the device behind the bridge --------------------------------------
    def _device_claim(self, req):
        """Answer a request delivered to the Endpoint behind the bridge."""
        if req.tlp_type == TYPE_CFG1:
            self.device_type1_ur_hits += 1          # Type 1 to an Endpoint: UR
            return ("device", CPL_UR, None, self.device_captured_id)
        if req.dev != 0 or req.fn != 0:
            self.device_dev_ur_hits += 1
            return ("device", CPL_UR, None, self.device_captured_id)
        if req.reg_num in self.device_crs_once:
            self.device_crs_once.discard(req.reg_num)
            self.device_crs_hits += 1
            return ("device", CPL_CRS, None, self.device_captured_id)
        if not req.is_read:
            self.device.write(req.reg_num,
                              req.payload[0] if req.payload else 0,
                              req.first_be)
            cid = self.device_captured_id
            self.device_captured_id = (req.bus << 8) | (req.dev << 3) | req.fn
            return ("device", CPL_SC, None, cid)
        value = self.device.read(req.reg_num)
        if value is None:
            return ("device", CPL_UR, None, self.device_captured_id)
        return ("device", CPL_SC, value, self.device_captured_id)

    def latency_for(self, who):
        """Response latency in cycles: BRIDGE_LATENCY for the bridge's own
        answers, BRIDGE_LATENCY + DEVICE_LATENCY for the device's, whose
        requests and completions pass through the bridge."""
        return BRIDGE_LATENCY if who == "bridge" \
            else BRIDGE_LATENCY + DEVICE_LATENCY


class BridgedCompleter:
    """The BDF-routing completer: the four-name interface (.start / .seen /
    .wait_for / .complete) over a BridgedTopology. It captures each request
    TLP from m_dllp_axis_*, asks the core who answers and how, and injects
    the Completion on s_dllp_axis_*, as the other _tlp completers do. All
    routing policy lives in the pure core, which is tested without a
    simulator."""

    def __init__(self, dut, topo=None):
        """topo defaults to a fresh BridgedTopology."""
        self.dut = dut
        self.topo = topo if topo is not None else BridgedTopology()
        self.seen = []                      # every request TLP, in wire order
        self.answers = []                   # (req, who, status), in order
        self._partial = []
        self._answered = 0

    def start(self):
        """Start capturing request TLPs; serve() starts answering them."""
        cocotb.start_soon(self._watch_tx())

    async def wait_for(self, count, cycles=40000):
        """Wait until `count` request TLPs were seen; raise after `cycles` cycles."""
        for _ in range(cycles):
            await RisingEdge(self.dut.clk_i)
            if len(self.seen) >= count:
                return
        raise AssertionError(
            f"expected {count} request TLPs on the wire, saw {len(self.seen)} "
            f"({self.seen}) -- FC credits, or the sequence never issued?")

    async def complete(self, req, status=CPL_SC, data=None, completer_id=0):
        """Inject one Completion for `req`, with Byte Count 4.

        It carries `data` (CplD) only for a successful read given data, and
        no data (Cpl) otherwise.
        """
        has_data = req.is_read and status == CPL_SC and data is not None
        words = [
            cpl_dw0(has_data=has_data, length_dw=1 if has_data else 0),
            cpl_dw1(completer_id, status, byte_count=4),
            cpl_dw2(RID, req.tag, lower_address=0),
        ]
        if has_data:
            words.append(data)
        await self.inject(words)

    def serve(self):
        """Start answering captured requests in order."""
        cocotb.start_soon(self._serve())

    async def _watch_tx(self):
        """Capture request TLPs from m_dllp_axis_* on each tvalid and tready beat."""
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
        """Answer each captured request after the latency of whoever answers it."""
        while True:
            await RisingEdge(self.dut.clk_i)
            while self._answered < len(self.seen):
                req = self.seen[self._answered]
                self._answered += 1
                who, status, rdata, cid = self.topo.handle(req.dwords)
                # Requests are answered one at a time, which loses nothing:
                # pcie_cfg_txn has at most one request outstanding.
                for _ in range(self.topo.latency_for(who)):
                    await RisingEdge(self.dut.clk_i)
                self.answers.append((req, who, status))
                await self.complete(req, status=status, data=rdata,
                                    completer_id=cid)

    async def inject(self, words):
        """Drive `words` on s_dllp_axis_*, one Dword per accepted beat."""
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


def _selftest_bridged_topology():
    """The bridge model's guards, exercised on the pure core at import time.

    Each guard is seen to fire before any test depends on it:
      1. a Type 1 write to register 6 at reset   -> UR, register untouched;
      2. a Type 1 request reaching the device     -> UR;
      3. after bus assignment, bus == Secondary   -> the one-bit transform.
    It also checks the latency byte (ignored on write, 00h on read), the
    Device 0 rule on both Links, the Completer ID capture sequence and the
    distinctness of the value table.
    """
    rd = cfg_wire_dw0(False)
    rd1 = cfg_wire_dw0(False, type1=True)
    wr = cfg_wire_dw0(True)
    wr1 = cfg_wire_dw0(True, type1=True)
    dw1 = cfg_wire_dw1(RID, 0x21, CFG_BE_DWORD)

    # 1: at reset Secondary = Subordinate = 00h, so bus 1 is outside [0, 0]
    # and a Type 1 write to the bridge's bus-number register is answered UR.
    topo = BridgedTopology()
    who, status, data, cid = topo.handle(
        [wr1, dw1, cfg_wire_dw2(0x01, 0, 0, CFG_REG_BUS_NUMBER), BUS_NUM_WDATA])
    assert (who, status) == ("bridge", CPL_UR), (who, status)
    assert topo.route_ur_hits == 1, "the SS7.3.3 case-3 arm did not fire"
    assert topo.bridge.bus_reg == 0, "a UR'd write reached the register"
    assert cid == 0x0000, "completer ID nonzero before any CfgWr0 (P5.6)"

    # The correctly-typed write: claimed locally, register takes [23:0].
    who, status, data, cid = topo.handle(
        [wr, dw1, cfg_wire_dw2(0x01, 0, 0, CFG_REG_BUS_NUMBER), BUS_NUM_WDATA])
    assert (who, status) == ("bridge", CPL_SC)
    assert cid == 0x0000, "the capturing write's OWN completion must carry 0000h"
    assert topo.bridge.secondary == SEC_BUS and topo.bridge.subordinate == SUB_BUS
    assert topo.bridge_captured_id == BRIDGE_BDF, "P5.6 capture did not happen"

    # 3: the transform, now that bus 5 is Secondary.
    who, status, data, cid = topo.handle(
        [rd1, dw1, cfg_wire_dw2(SEC_BUS, 0, 0, CFG_REG_VENDOR_DEVICE)])
    assert (who, status) == ("device", CPL_SC)
    assert data == (SEC_DEV_DEVICE << 16) | SEC_DEV_VENDOR
    assert len(topo.transforms) == 1, "the transform arm did not run"
    received, forwarded = topo.transforms[0]
    assert received[0] ^ forwarded[0] == 1 and received[1:] == forwarded[1:]
    assert cid == 0x0000, \
        "probe-phase completer ID must be 0000h at the device too (Trap B)"

    # 2: bus 7 lies in (Secondary, Subordinate], so the request is forwarded
    # unmodified and the device answers the untransformed Type 1 with UR.
    # Because the device answers any Type 1 with UR, its SC in step 3 above
    # shows that the transform ran.
    who, status, data, cid = topo.handle(
        [rd1, dw1, cfg_wire_dw2(0x07, 0, 0, CFG_REG_VENDOR_DEVICE)])
    assert (who, status) == ("device", CPL_UR), (who, status)
    assert topo.forward_unmodified_hits == 1 and topo.device_type1_ur_hits == 1

    # Out-of-aperture stays UR post-assignment (bus 2 < Secondary).
    who, status, _, _ = topo.handle(
        [rd1, dw1, cfg_wire_dw2(0x02, 0, 0, CFG_REG_VENDOR_DEVICE)])
    assert (who, status) == ("bridge", CPL_UR) and topo.route_ur_hits == 2

    # The latency byte is ignored on write and reads 00h, even when the
    # writer drives it non-zero.
    topo.handle([wr, dw1, cfg_wire_dw2(0x01, 0, 0, CFG_REG_BUS_NUMBER),
                 0xAA00_0000 | BUS_NUM_WDATA])
    assert topo.bridge.latency_byte_writes_ignored >= 1
    _, _, readback, _ = topo.handle(
        [rd, dw1, cfg_wire_dw2(0x01, 0, 0, CFG_REG_BUS_NUMBER)])
    assert readback == BUS_NUM_WDATA, (
        f"18h readback {readback:#010x}: [31:24] must be 00h regardless of "
        "what was written (SS7.5.3.3 p.493)")

    # The Device 0 rule (PCIe Base Spec r2.1, §7.3.1), on both Links.
    _, status, _, _ = topo.handle(
        [rd, dw1, cfg_wire_dw2(0x01, 3, 0, CFG_REG_VENDOR_DEVICE)])
    assert status == CPL_UR and topo.bridge_dev_ur_hits == 1
    _, status, _, _ = topo.handle(
        [rd1, dw1, cfg_wire_dw2(SEC_BUS, 3, 0, CFG_REG_VENDOR_DEVICE)])
    assert status == CPL_UR and topo.device_dev_ur_hits == 1

    # The device captures its ID on its first Type 0 write, sent as Type 1
    # and transformed by the bridge.
    topo.handle([wr1, dw1, cfg_wire_dw2(SEC_BUS, 0, 0, CFG_REG_BAR0), 0xFFFFFFFF])
    assert topo.device_captured_id == SEC_DEV_BDF
    _, _, _, cid = topo.handle(
        [rd1, dw1, cfg_wire_dw2(SEC_BUS, 0, 0, CFG_REG_BAR0)])
    assert cid == SEC_DEV_BDF, "post-capture completions must carry the BDF"

    # The value table is pairwise distinct.
    ids = {VENDOR, DEVICE, BRIDGE_VENDOR, BRIDGE_DEVICE,
           SEC_DEV_VENDOR, SEC_DEV_DEVICE}
    assert len(ids) == 6 and 0xFFFF not in ids
    assert len({0x01, SEC_BUS, SUB_BUS}) == 3


_selftest_bridged_topology()
