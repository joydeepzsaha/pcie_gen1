"""Shared spec-golden helpers for the Commit 2b enumeration benches.

One importable module rather than a copy per test file: the descriptor builders
and decoders below are GOLDENS, and a golden that exists in three slightly
divergent copies is not a golden.  `tb_rc.core` gives this file its own `copyto`
entry in every fileset that needs it.

Commit 2b-3 widened the module's remit from goldens to the whole shared bench
surface -- the pcie_rq_rc_top monitor, the cumulative credit drip, the TLP
request decoder, the on-wire assertion, the enum_error_e codes and the golden
device the benches model.  Those were per-file copies until the BAR benches were
about to make a third and fourth.  Measured effect: symbols defined in more than
one bench file went from 29 to 9, and the nine that remain are per-bench by
design, not by neglect:

  settle, init                    -- see the exclusions note below
  CRS_RETRY_MAX                   -- each bench's own .sv shim override
  send_cmd, recv_rsp              -- pcie_cfg_txn's command port; no analogue
                                     in a sequencer bench
  assert_on_wire                  -- now a ~10-line BINDING onto the shared
                                     assert_cfg_tlp_on_wire, not a second copy
                                     of the goldens; each bench binds the
                                     optional properties it actually owns
  start_scan, status, wait_terminal -- the scan phase's status surface.  Left
                                     until Commit D on purpose: pcie_enum_top's
                                     surface grows BAR outputs there, so
                                     consolidating now would mean doing it
                                     twice against a shape that is about to
                                     change.

!! Two exclusions are deliberate and load-bearing; both are argued in full at
SS THE INTEGRATION-BENCH HELPERS below.  `settle()` stays local because its
default IS sim time, and the completers stay separate because what they share is
an interface contract, not an implementation.

Everything here is hand-derived from the specification:

  RQ descriptor, Configuration form ... PG213 v1.3 Table 61
                                        (pg213 markdown :3711,:3720,:3728,:3735)
  RQ Request Type encodings ........... PG213 v1.3 Table 57 (via :3725)
  RC descriptor ....................... PG213 v1.3 Table 65 (:4034); bit 30
                                        "Request Completed" at :4049;
                                        Completion Status [45:43] at :4052
  Configuration Request header ........ PCIe Base 2.1 SS2.2.7 p.79-80
  Completion header ................... PCIe Base 2.1 SS2.2.9 p.97-98
  Config Space Type 0 header offsets .. PCIe Base 2.1 Figure 7-5 p.491

Nothing here is read back from a DUT.  The 128-bit RQ descriptor values these
builders produce are pinned independently in docs/predictions/SPEC_PREDICTIONS_ENUM.md SS3.4,
which was committed before any of this RTL existed; `assert_rq_descriptor`
below is what ties the two together.
"""

# ---------------------------------------------------------------------------
# Encodings
# ---------------------------------------------------------------------------

# pcie_rq_rc_pkg::rq_req_type_e  (PG213 Table 57)
RQ_CFG_READ0 = 0b1000
RQ_CFG_WRITE0 = 0b1010
# Stage D-2: the Type 1 pair.  Exactly one bit (bit 0) away from the Type 0
# encodings -- _selftest_type1_one_bit() below pins that distance, because it
# is what makes a mistyped golden actively wrong rather than merely different
# (docs/predictions/SPEC_PREDICTIONS_STAGE_D.md SS8.1, Trap A).
RQ_CFG_READ1 = 0b1001
RQ_CFG_WRITE1 = 0b1011

# pcie_rq_rc_pkg::rc_cpl_status_e == PG213 [45:43] == PCIe CPL Completion Status
CPL_SC = 0b000
CPL_UR = 0b001
CPL_CRS = 0b010
CPL_CA = 0b100
# The four RESERVED encodings.  Base 2.1 SS2.3.2 p.122: "Completions with a
# Reserved Completion Status value are treated as if the Completion Status was
# Unsupported Request (UR)."  Named so tests can drive them deliberately.
CPL_RESERVED = (0b011, 0b101, 0b110, 0b111)

# pcie_rq_rc_pkg::rc_desc_error_e
EC_NORMAL = 0b0000
EC_POISONED = 0b0001
EC_BAD_STATUS = 0b0010          # terminated by UR / CA / CRS

# pcie_rq_rc_pkg::rc_error_e
RC_ERR_ORPHAN_DATA = 3

# tlp_pkg::tlp_error_e. A completion naming a tag the tracker never allocated is
# reported ONCE for the packet on rc_unexpected_completion_o
# (tlp_request_tracker.sv:316), independently of pcie_rc_if's per-Dword orphan
# reports. Two different surfaces describing two different facts about the same
# packet -- see e5 in test_pcie_enum_bar_tlp.py.
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

# pcie_enum_pkg config register numbers (Base 2.1 Figure 7-5 p.491)
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
# The Expansion ROM Base Address register, [PCI3] SS6.2.5.2 p.227 :11283/:11287.
# Present here ONLY so P-NO-ROM can assert that nothing ever touches it.
CFG_REG_EXPANSION_ROM = 0x0C

# Byte enables.  Base 2.1 SS2.2.7 p.79 pins Last DW BE to 0000b for every
# Configuration Request, so only first_be is ever a choice.
CFG_BE_DWORD = 0b1111
CFG_BE_LOWER_HALF = 0b0011
CFG_BE_BYTE2 = 0b0100
CFG_LAST_BE = 0b0000

# tlp_pkg::tlp_fmt_e / tlp_type_e (tlp_pkg.sv:8-27), for the integration bench
FMT_3DW_NO_DATA = 0b000
FMT_3DW_DATA = 0b010
TYPE_CFG0 = 0b00100
TYPE_CFG1 = 0b00101            # Stage D-2: Base 2.1 SS2.2.7 Table 2-3, one bit up
TYPE_CPL = 0b01010


# ---------------------------------------------------------------------------
# RQ descriptor -- what the DUT must emit
# ---------------------------------------------------------------------------
def rq_desc(req_type, dword_count=1, address=0, completer_id=0, tc=0, attr=0,
            poisoned=0, tag=0, requester_id=0):
    """PG213 Table 61 / Table 60 RQ descriptor, 128 bits.

    Tag [103:96] and Requester ID [95:80] are IGNORED by the core (tags are
    core-managed, the TL uses requester_id_i) but are settable so a test can
    prove the DUT leaves them zero rather than merely that the core ignores them.
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
    -- PG213 Table 61 :3715-3718.  Bits [1:0] are Reserved: the byte within the
    Dword is selected by first_be, never by the address.
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
    """s_axis_rq_tuser: [3:0] first_be, [7:4] last_be (PG213, pcie_rq_if.sv:147)."""
    return ((last_be & 0xF) << 4) | (first_be & 0xF)


def decode_tuser(v):
    return {"first_be": v & 0xF, "last_be": (v >> 4) & 0xF}


def assert_rq_descriptor(observed_desc, observed_tuser, *, write, bdf, reg_num,
                         first_be, ext_reg=0, type1=False, what=""):
    """Assert one emitted RQ descriptor against a freshly built golden.

    Compares the WHOLE 128-bit word, not a field subset: a field the DUT sets
    that the golden leaves zero is exactly the kind of thing a per-field check
    misses.  Reports the field-level diff on failure so the whole-word compare
    stays debuggable.

    type1 selects the Stage D-2 CFG1 golden (req_type 1001/1011 instead of
    1000/1010); default False keeps every pre-D-2 caller byte-identical.
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
def encode_rc_desc(tag, status=CPL_SC, dword_count=None, request_completed=1,
                   byte_count=None, error_code=None, lower_address=0,
                   requester_id=0, completer_id=0, tc=0, attr=0, poisoned=0,
                   locked=0):
    """PG213 Table 65, the 96-bit RC descriptor.

    Defaults follow what pcie_rc_if would actually build for a configuration
    completion, so a bench that does not override them is driving a realistic
    packet rather than an arbitrary one:

      * a Successful Completion to a config READ carries one Dword;
      * every non-SC status carries NO data and sets Request Completed --
        Base 2.1 SS2.3.2 p.122 ("No data is included with the Completion ...
        This Completion is the final Completion for the Request") and
        PG213 :4242 for the matching Error Code 0010.
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
    """PG213 Table 65."""
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

    Beat 0 is the 3-Dword descriptor in Dwords 0..2 with the FIRST payload Dword
    in Dword 3; later beats are payload, offset by one Dword
    (pcie_rq_rc_pkg.sv:109-115).  A descriptor-only packet is a single beat with
    tkeep = 0b0111.
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
    """(96-bit descriptor, [payload Dwords])."""
    words = packet_dwords(beats)
    assert len(words) >= 3, f"RC packet shorter than a descriptor: {words}"
    return words[0] | (words[1] << 32) | (words[2] << 64), words[3:]


# ---------------------------------------------------------------------------
# On-wire TLP goldens -- integration bench only
# ---------------------------------------------------------------------------
def cfg_wire_dw2(bus, dev, fn, reg_num, ext_reg=0):
    """The Configuration Request's third header Dword, as emitted.

    {Bus[31:24], Device[23:19], Function[18:16], Reserved[15:12],
     Ext Reg[11:8], Register[7:2], R[1:0]} -- PCIe Base 2.1 Figure 2-18 p.80,
    built by tlp_generator.sv, the dw2 assembly.  The BDF comes from the RQ descriptor's
    Completer ID field, NOT from the address.
    """
    return (((bus & 0xFF) << 24) | ((dev & 0x1F) << 19) | ((fn & 0x7) << 16)
            | ((ext_reg & 0xF) << 8) | ((reg_num & 0x3F) << 2))


def cfg_wire_dw0(write, length_dw=1, tc=0, attr=0, type1=False):
    # attr is PCIe Attr[2:0] = {IDO, RO, NS}; Base 2.1 SS2.2.1 p.57 puts
    # Attr[2] at dw0[10] and Attr[1:0] at dw0[21:20] -- the halves are not
    # adjacent.  KEEP attr=0 AND tc=0 HERE: Base 2.1 SS2.2.7 p.79 says a
    # Configuration Request must carry TC[2:0]=000b and Attr[1:0]=00b with
    # Attr[2] reserved, so a non-zero value would build an ILLEGAL TLP.
    # The enumeration targets are attr-blind by the spec, not by omission;
    # non-zero attr belongs on a completion (cpl_dw0) or a memory request.
    """Configuration Request DW0 as tlp_generator assembles it (:60-73).

    Base 2.1 SS2.2.7 p.79 fixes Length to 1 and TC/Attr/AT to zero for every
    Configuration Request; the fmt bit is the only thing read vs write changes.

    type1 selects the Stage D-2 Type 1 golden (dw0[4:0] = 00101 instead of
    00100); default False keeps every pre-D-2 caller byte-identical.
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
    """Stage D-2 builder self-assert (docs/predictions/SPEC_PREDICTIONS_STAGE_D.md SS8.1, Trap A).

    The Type 0 and Type 1 goldens must sit EXACTLY one bit apart, at both
    levels -- descriptor req_type 1000/1010 vs 1001/1011, wire dw0[4:0] 00100
    vs 00101.  That one-bit distance is what makes a mistyped golden actively
    wrong (it matches the OTHER type perfectly), so it is pinned here at import
    time in every bench that uses these builders, not assumed.
    """
    assert RQ_CFG_READ1 ^ RQ_CFG_READ0 == 1, "req_type read pair not one bit apart"
    assert RQ_CFG_WRITE1 ^ RQ_CFG_WRITE0 == 1, "req_type write pair not one bit apart"
    assert TYPE_CFG1 ^ TYPE_CFG0 == 1, "tlp_type_e CFG pair not one bit apart"
    # Whole-golden distance: exactly the req_type field's low bit ([75]) at the
    # descriptor level, exactly dw0 bit 0 on the wire -- nothing else moves.
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

    tlp_generator.sv, the dw1 assembly.  Base 2.1 SS2.2.7 p.79: Last DW BE must be 0000b.
    """
    return (((requester_id & 0xFFFF) << 16) | ((tag & 0xFF) << 8)
            | ((last_be & 0xF) << 4) | (first_be & 0xF))


def dw0_length(dw0):
    """Recover length_dw from a TX DW0 (inverse of tlp_generator.sv, the dw0 assembly)."""
    enc = ((dw0 >> 24) & 0xFF) | (((dw0 >> 16) & 0x3) << 8)
    return 1024 if enc == 0 else enc


def cpl_dw0(has_data, length_dw, tc=0, attr=0):
    # attr is PCIe Attr[2:0] = {IDO, RO, NS}: Attr[2] -> dw0[10] and
    # Attr[1:0] -> dw0[21:20] (Base 2.1 SS2.2.1 p.57).  Unlike a config
    # request, a completion may legitimately carry non-zero attributes,
    # echoed from the request it answers.
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
    """{Completer ID[31:16], Status[15:13], BCM[12], Byte Count[11:0]}."""
    return (((completer_id & 0xFFFF) << 16) | ((status & 0x7) << 13)
            | ((bcm & 1) << 12) | (byte_count & 0xFFF))


def cpl_dw2(requester_id, tag, lower_address=0):
    """{Requester ID[31:16], Tag[15:8], R[7], Lower Address[6:0]}."""
    return (((requester_id & 0xFFFF) << 16) | ((tag & 0xFF) << 8)
            | (lower_address & 0x7F))


# ===========================================================================
# SS THE SOCKET MODEL (Commit 2b-2)
#
# Plays pcie_rq_rc_top's user-facing socket for a standalone target.  This is
# bench code that behaves like RTL, which makes it exactly as capable of being
# wrong as RTL -- and its failure mode is worse, because a socket model that is
# too POLITE makes a broken DUT look correct.
#
# !! IT ASSERTS ITS OWN PHYSICAL ORDERING RATHER THAN BEING TRUSTED TO PRESERVE
# IT.  Commit 2b-1's bring-up lost two runs to this class of bug, both the model
# being too AGGRESSIVE rather than too polite: it delivered completions, and
# fired timeout strobes, before it had strobed the tag.  Neither ordering is
# physically possible -- tlp_request_tracker allocates the tag, which is what
# raises allocated_tag_valid_o, BEFORE the request TLP is generated and
# transmitted, so any response is at minimum a link round trip later.  The three
# invariants are now checked in the model:
#
#   1. a completion may not be delivered for a transaction whose tag has not
#      been strobed;
#   2. a timeout strobe may not fire for an allocated tag that has not been
#      strobed;
#   3. the tag strobe follows command accept by >= 1 cycle -- the surface
#      mutation SM-1 attacks.
#
# A violated invariant raises AssertionError, which fails the ONE test that
# tripped it.  That is the Python equivalent of the $warning-never-$error rule:
# it must not take down the shared multi-test process.
#
# NOTE ON DUPLICATION: test_pcie_enum_txn.py carries an earlier, module-local
# copy of this class without the invariants.  It is left alone deliberately --
# the 2b-2 brief forbids touching an existing testbench, and rewriting a green
# suite to share code is not worth perturbing a baseline for.  Migrate it the
# next time that file is opened for a real reason.
# ===========================================================================
import cocotb                                          # noqa: E402
from cocotb.triggers import ReadOnly, RisingEdge       # noqa: E402


class SocketRequest:
    """One RQ packet the DUT drove, plus the tag the socket gave it."""

    def __init__(self, beats, tag, accept_cycle):
        self.beats = beats
        self.tag = tag
        self.accept_cycle = accept_cycle
        self.desc = beats[0][0] & ((1 << 128) - 1)
        self.tkeep = beats[0][1]
        self.tuser = beats[0][3]
        self.payload = [b[0] & 0xFFFFFFFF for b in beats[1:]]
        self.write = len(beats) > 1

    def __repr__(self):
        kind = "CfgWr0" if self.write else "CfgRd0"
        return (f"{kind}(tag={self.tag:#04x}, reg={(self.desc >> 2) & 0x3F:#04x}, "
                f"desc=0x{self.desc:032X})")


class Socket:
    """pcie_rq_rc_top's socket, played in Python.  See the block comment above."""

    def __init__(self, dut, tag_delay=2, first_tag=0x5A):
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
        cocotb.start_soon(self._cycle_counter())
        cocotb.start_soon(self._rq())

    def stall_beats(self, cycles):
        """Hold s_axis_rq_tready low for `cycles` cycles, starting now."""
        self._stall_left = cycles

    async def _cycle_counter(self):
        while True:
            await RisingEdge(self.dut.clk_i)
            self.cycle += 1

    async def wait_for(self, count, cycles=6000):
        for _ in range(cycles):
            await RisingEdge(self.dut.clk_i)
            if len(self.requests) >= count:
                return
        raise AssertionError(
            f"expected {count} RQ packets, saw {len(self.requests)}: {self.requests}")

    async def _rq(self):
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
        tag = self._next_tag
        self._next_tag = (self._next_tag + 1) & 0xFF
        self.tags.append(tag)
        cocotb.start_soon(self._strobe_tag(tag, self.cycle))

    async def _strobe_tag(self, tag, accept_cycle):
        d = self.dut
        for _ in range(self.tag_delay):
            await RisingEdge(d.clk_i)
        # INVARIANT 3, checked rather than assumed.
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

        ORDERING CONSTRAINT, not a convenience -- see INVARIANT 1 above.
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
        """Deliver one completion on the RC stream."""
        if req is not None:
            await self._await_strobe(req.tag)          # INVARIANT 1
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
        d = self.dut
        for tdata, tkeep, tlast in beats:
            d.m_axis_rc_tdata_i.value = tdata
            d.m_axis_rc_tkeep_i.value = tkeep
            d.m_axis_rc_tlast_i.value = tlast
            d.m_axis_rc_tvalid_i.value = 1
            # The DUT ties tready high, but honour it anyway: a socket that
            # ignored tready could not detect a DUT that started lowering it.
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
        """One-cycle cpl_timeout_valid_o strobe naming `tag`.

        INVARIANT 2: the tracker cannot time out a tag it has not allocated, so
        a strobe for an allocated tag waits for that tag's strobe first.  A tag
        the socket never handed out fires immediately -- that is deliberate
        stimulus, not an ordering violation.
        """
        d = self.dut
        if tag in self.tags:
            await self._await_strobe(tag)
        d.cpl_timeout_tag_i.value = tag
        d.cpl_timeout_valid_i.value = 1
        await RisingEdge(d.clk_i)
        d.cpl_timeout_valid_i.value = 0


# ===========================================================================
# SS THE INTEGRATION-BENCH HELPERS (Commit 2b-3)
#
# Everything below is shared by the _tlp benches -- the ones that put a REAL
# pcie_rq_rc_top behind the DUT. They were duplicated per file until 2b-3.
# Consolidated here BEFORE a third copy was created by the BAR benches, which
# is the point at which a divergent copy stops being a nuisance and starts
# being a silent disagreement between two goldens.
#
# !! WHAT IS DELIBERATELY *NOT* HERE, AND WHY
#
#   settle()      -- three different defaults across four benches (20 / 30 /
#                    40) and, unlike every waiter below, it is NOT an
#                    early-exit loop: it always runs its full count, so the
#                    default IS sim time. Sharing it under one default would
#                    move verilate_enum_txn and verilate_enum_scan off their
#                    pinned sim end times. Four honest local copies beat one
#                    shared body wrapped per bench to restore a per-bench
#                    constant. See the Commit 2b-3 message.
#
#   init()        -- four variants with different signatures, because the
#                    benches drive different DUT port sets.
#
#   the completers -- ConfigCompleter (2b-1) and ConfigSpaceCompleter (2b-2)
#                    are genuinely different models. What they SHARE is the
#                    four-name interface .start / .seen / .wait_for /
#                    .complete, and that contract is specified in
#                    docs/spec-notes/EP_VERIFICATION_MODEL_SPEC.md rather than forced into a
#                    common base class. A third implementation (BAR write-mask
#                    semantics) joins them in Commit D.
#
#   send_cmd / recv_rsp -- pcie_cfg_txn's command port only; no analogue in a
#                    sequencer bench.
# ===========================================================================

# The RC's own identity and its target, identical in every integration bench.
CLK_NS = 4
CPL_TIMEOUT_CYCLES = 4096       # the integration benches' OWN value: each
                                # tb_pcie_enum_*_tlp.sv / _dl_top wrapper passes
                                # 4096 explicitly.  ⚠️ NOT the shipped default --
                                # that was 4096 before §63 #7e, 6250 until 7g-2,
                                # and is 10 ms = 1,250,000 cycles since (tlp_pkg)
RID = 0x1234                    # the Root Complex's own requester_id_i
BDF = 0x0100                    # the target: bus 1, device 0, function 0
BUS, DEV, FN = 0x01, 0x00, 0x00

# pcie_enum_pkg::enum_error_e. Duplicated in both scan benches before 2b-3;
# the BAR benches would have made four copies.
ENUM_ERR_NONE = 0
ENUM_ERR_UR_POST_PROBE = 1
ENUM_ERR_CA = 2
ENUM_ERR_CRS_EXHAUSTED = 3
ENUM_ERR_TIMEOUT = 4
# BAR phase (Commit 2b-3).  enum_error_e widened to [3:0] to hold these; codes
# 0..4 above keep their values byte-identically.
ENUM_ERR_BAR_TYPE = 5
ENUM_ERR_BAR_SIZE = 6
ENUM_ERR_BAR_WINDOW = 7
ENUM_ERR_BAR_ADDR32 = 8
# sec 63 #7f #19 (D-P3.5): a completion timeout on a request the credit gate
# was holding when it expired.  Reported as its own code so a reader of
# enum_error_code_o alone is not told "dead device"; err_credit_blocked_o is
# still set alongside it.
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
    return ERR_NAME.get(value, f"<unknown {value}>")


# The shipped allocator geometry -- pcie_enum_bar's parameter defaults.  The
# addresses every BAR test asserts derive from these, and they were pinned in
# docs/predictions/SPEC_PREDICTIONS_ENUM.md SSE.7.4 before the RTL existed.
MEM_BAR_BASE = 0x0000_0000_8000_0000
MEM_BAR_WINDOW = 0x0000_0000_1000_0000

# The Command register value written last.  Memory Space Enable | Bus Master
# Enable -- [PCI3] Table 6-1 p.218 :10764, :10767.  I/O Space Enable stays 0
# because no I/O BAR is ever assigned (SSE.6, SSE.7.2).
CMD_ENABLE_VALUE = 0x0000_0006


class TlpRequest:
    """One request TLP observed leaving the Transaction Layer.

    The field set is the UNION of what the two _tlp benches decoded before
    consolidation: 2b-1 needed tlp_type and ext_reg, 2b-2 needed bus/dev/fn for
    its device-0 assertion. Both are cheap, and the BAR benches need both --
    they assert on writes (tlp_type) and on the routing Dword (bus/dev/fn).
    """

    def __init__(self, dwords):
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
        kind = "Rd" if self.is_read else "Wr"
        # Stage D: name the type honestly -- a CfgRd1 rendered as "CfgRd0"
        # in a failure message is Trap A leaking into the diagnostics.
        t = "1" if self.tlp_type == TYPE_CFG1 else "0"
        return (f"Cfg{kind}{t}(tag={self.tag:#04x}, "
                f"bdf={self.bus:02x}:{self.dev:02x}.{self.fn}, "
                f"reg={self.reg_num:#04x}, fbe={self.first_be:#06b}, "
                f"payload={[hex(w) for w in self.payload]})")


# ---------------------------------------------------------------------------
# Flow control
# ---------------------------------------------------------------------------
def set_credits(dut, ph=0xFF, pd=0xFFF, nph=0xFF, npd=0xFFF, cplh=0xFF, cpld=0xFFF):
    dut.fc_ph_i.value = ph
    dut.fc_pd_i.value = pd
    dut.fc_nph_i.value = nph
    dut.fc_npd_i.value = npd
    dut.fc_cplh_i.value = cplh
    dut.fc_cpld_i.value = cpld


class CreditDrip:
    """A receiver returning credit the way a real one does: CUMULATIVELY.

    !! fc_*_i is the raw CREDITS_ALLOCATED off the wire (Base 2.1 SS2.6.1.2
    p.141) -- there is no arithmetic anywhere on the path.  An UpdateFC
    therefore advertises a RUNNING TOTAL that only ever grows, and a drip that
    re-pulses a constant is advertising "I have still only ever allocated N",
    which blocks the transmitter forever once N are consumed.

    That failure is indistinguishable from a DUT deadlock, which is exactly why
    the drip is itself mutation-tested: re-pulsing a constant must make the
    tests that depend on this coroutine FAIL, proving they test the DUT and not
    the coroutine.
    """

    def __init__(self, dut, nph=1, npd=1, period=40, step=1):
        self.dut = dut
        self.nph = nph
        self.npd = npd
        self.period = period
        self.step = step
        self.updates = 0
        self._run_flag = True

    def start(self):
        cocotb.start_soon(self._run())

    def stop(self):
        self._run_flag = False

    async def _run(self):
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


class Mon:
    """Every error and event surface of pcie_rq_rc_top, sampled each cycle.

    !! THE WAITER BOUNDS WERE UNIFIED *UPWARD* AT CONSOLIDATION.  The two _tlp
    benches carried different defaults (2b-1: CPL_TIMEOUT_CYCLES+600 and 400;
    2b-2: +900 and 600).  Both waiters are EARLY-EXIT loops -- they return on
    the cycle the expected count is reached and raise only on exhaustion, which
    is a failure, never a normal path.  So while the suite is green every call
    returns early and the bound is never reached, which makes raising it
    provably sim-time-neutral.  Lowering it would not be: a call that genuinely
    needed more than 400 cycles would turn a PASS into a FAIL.  The asymmetry is
    the whole argument for picking the larger value rather than either one.
    """

    def __init__(self, dut):
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
        cocotb.start_soon(self._run())

    async def _run(self):
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
        for _ in range(cycles):
            await RisingEdge(self.dut.clk_i)
            if len(self.timeouts) >= count:
                return
        raise AssertionError(
            f"expected {count} cpl_timeout strobes, saw {len(self.timeouts)}")

    async def wait_lates(self, count, cycles=600):
        for _ in range(cycles):
            await RisingEdge(self.dut.clk_i)
            if len(self.lates) >= count:
                return
        raise AssertionError(
            f"expected {count} late_cpl strobes, saw {len(self.lates)}")

    def clean(self, allow_timeouts=False, allow_orphans=False,
              allow_unexpected=False):
        """Nothing fired that the prediction did not name.

        credit_error_o is asserted silent unconditionally: a single-Dword config
        request can never trip it by construction, since that needs a request
        exceeding the peer's ENTIRE initial data advertisement.
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
    """Assert one emitted Configuration TLP against hand-derived goldens.

    Base 2.1 SS2.2.7 p.79 fixes Length to 1 Dword, Last DW BE to 0000b and
    TC/Attr/AT to zero for every Configuration Request; Figure 2-18 p.80 fixes
    the third header Dword's BDF packing.  docs/predictions/SPEC_PREDICTIONS_ENUM.md SS3.4,
    SSD.4 and SSE.8 pin the resulting values.

    require_device0 is opt-IN, not the default.  It is the SS7.3.1 p.479
    device-0-only property, which is the presence scan's subject; a bench that
    does not own that property should not silently start asserting it.

    type1 (Stage D) selects the CFG1 DW0 golden -- dw0[4:0] = 00101, one bit
    from Type 0, which is exactly why the DW0 compare here is the WHOLE Dword
    (Trap A, docs/predictions/SPEC_PREDICTIONS_STAGE_D.md SS8.1).  bus overrides the routing
    Dword's bus field (default: the direct-attach BUS); Device/Function stay 0
    on every bus level by construction (P5.3).
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
# The golden device the enumeration benches model, and its Type 0 header.
#
# One device description, not one per bench.  Commit D's acceptance test
# enumerates this same device end to end (docs/predictions/SPEC_PREDICTIONS_ENUM.md SSE.8), so
# consolidating here is the "before a third copy" case the 2b-3 brief names.
# ---------------------------------------------------------------------------
SCAN_BUS = 0x01

VENDOR = 0x144D             # a real-looking Vendor ID; NOT 0xFFFF -- absence is
                            # signalled by UR alone, never by a sentinel (SSD)
DEVICE = 0xA80A
REG0 = (DEVICE << 16) | VENDOR

HDR_TYPE0 = 0x00            # endpoint Function
HDR_TYPE0_MF = 0x80         # endpoint Function, multi-function (bit 7)
HDR_TYPE1 = 0x01            # PCI-to-PCI bridge -- [PCI3 SS6.2.1 p.216 :10685]


def reg3(header_type, bist=0x00, mlt=0x00, cls=0x10):
    """Configuration register 3 (byte offset 0Ch).

    {BIST[31:24], Header Type[23:16], Master Latency Timer[15:8],
     Cache Line Size[7:0]} -- [BASE Figure 7-5 p.491].
    """
    return (bist << 24) | ((header_type & 0xFF) << 16) | (mlt << 8) | cls


def outcome_name(value):
    """pcie_enum_pkg::txn_outcome_e as a name, for assertion messages."""
    return TXN_NAME.get(value, f"<unknown {value}>")


# ===========================================================================
# SS ⭐ THE CONFIGURATION SPACE, WITH REAL BASE ADDRESS REGISTER SEMANTICS
#
# This is bench code that behaves like hardware, so it is exactly as capable of
# being wrong as hardware -- and here its failure mode is specific and nasty.
#
# !! A COMPLETER THAT ECHOES BAR WRITES VERBATIM MAKES SIZING RETURN GARBAGE,
# !! AND THE DUT LOOKS BROKEN WHEN THE BENCH IS.
#
# The whole all-ones sizing algorithm rests on ONE sentence -- [PCI3] SS6.2.5.1
# p.226 :11205, "Bits 0-3 are read-only" -- plus the don't-care behaviour of
# :11222, "The device will return 0's in all don't-care address bits".  A model
# that stored FFFFFFFF and handed it back would report every BAR as 4 GB and
# would destroy the type/prefetch encoding the very next read depends on.
#
# So the model implements the masking, and COUNTS IT.  mask_hits and
# ro_low_hits exist so a test can assert the arm was actually exercised rather
# than merely present -- the 2b-2 silent-UR trap, one layer up.
# ===========================================================================

BAR_MEM32 = "mem32"
BAR_MEM64 = "mem64"
BAR_IO = "io"


class BarSpec:
    """One Base Address register as a device implements it.

    size is in bytes and must be a power of two.  A BAR_MEM64 spec occupies the
    candidate register it is placed at AND the next one -- that is what makes it
    a pair, and the model expands it so a test cannot forget.
    """

    def __init__(self, kind, size, prefetch=False):
        assert kind in (BAR_MEM32, BAR_MEM64, BAR_IO), kind
        assert size > 0 and (size & (size - 1)) == 0, \
            f"BAR size {size:#x} is not a power of two -- [PCI3] p.226 :11226"
        self.kind = kind
        self.size = size
        self.prefetch = prefetch

    @property
    def type_field(self):
        """Bits [3:0], the read-only field.  [PCI3] p.225 :11187, :11190, :11193."""
        if self.kind == BAR_IO:
            return 0b0001                     # bit 0 = 1, bit 1 reserved reads 0
        bits = 0b0000 if self.kind == BAR_MEM32 else 0b0100   # [2:1] = 00 or 10
        return bits | (0b1000 if self.prefetch else 0)

    @property
    def registers(self):
        return 2 if self.kind == BAR_MEM64 else 1


class ConfigDevice:
    """A Type 0 configuration space the enumeration benches can enumerate.

    bars is a mapping {candidate register number: BarSpec}.  Every candidate
    register not named is UNIMPLEMENTED and reads hardwired zero
    ([PCI3] p.226 :11224); the upper half of a BAR_MEM64 is filled in
    automatically and may not be named separately.

    Registers outside the model return None from read(), which the completers
    turn into an Unsupported Request -- Base 2.1 SS7.3.3 p.480.
    """

    def __init__(self, bars=None, header_type=HDR_TYPE0, vendor=VENDOR,
                 device=DEVICE, raw=None):
        # ⭐ raw is {register: fixed readback value} and it MODELS A MALFORMED
        # DEVICE. Such a register answers the same value no matter what is
        # written to it, which is the only way to present an encoding BarSpec
        # cannot legally build -- a Reserved Type field, or a size whose two's
        # complement is not a power of two.
        #
        # !! IT MUST IGNORE WRITES, AND THAT IS THE WHOLE POINT. A first attempt
        # injected these values into the ordinary register store instead; the
        # all-ones sizing write promptly overwrote them with FFFFFFFF, the
        # readback came back with bit 0 set, and the FSM correctly classified the
        # register as an I/O BAR and skipped it. Three fault tests then passed
        # their DUT and failed their own premise.
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
        """⭐ The write-mask arm actually ran.

        Brief SS3.1: a completer that echoes BAR writes verbatim makes sizing
        return garbage, so the mask must be proved live rather than assumed.

        !! ONE GATE, NOT TWO, AND THE MUTATION CAMPAIGN IS WHY. This first read
        "assert mask_hits > 0" and then "assert ro_low_hits > 0", with a comment
        claiming the second caught something the first did not. The comment was
        WRONG -- ro_low_hits counts a subset of what mask_hits counts, so the
        second strictly implies the first, and defeating the first alone could
        not change any verdict. It duly survived as a mutation.
        A redundant assertion is not a stronger check; it is an untested one.
        mask_hits stays as a diagnostic in the message.
        """
        assert self.ro_low_hits > 0, (
            f"{what}no write was ever masked inside the read-only low field, so "
            "the BAR write mask never protected it ([PCI3] p.226 :11205 for a "
            "memory BAR's bits 3:0, p.225 :11187 for an I/O BAR's bits 1:0). "
            "This test would pass against a completer that echoed writes "
            "verbatim, which is the bug the mask exists to model. "
            f"(mask_hits={self.mask_hits}, ro_low_hits={self.ro_low_hits})")


# ---------------------------------------------------------------------------
# SS ⭐ THE EMPTY-SET GUARD
#
# "A green diff, an empty finding list, and a passing assertion over an empty
# set are the same bug" -- docs/recon/RECON_commit2b3.md SS2, where it fired on the recon
# itself.  Brief SS3.2 trap 3 makes the guard mandatory for every on-wire
# assertion in Commits D and E, and for any helper that iterates a collected
# list.
#
# These are deliberately tiny and deliberately loud.  The alternative -- trusting
# each call site to remember -- is what produced the trap in the first place.
# ---------------------------------------------------------------------------
def nonempty(seq, what):
    """Return seq, having proved it has something in it."""
    items = list(seq)
    assert items, (
        f"{what}: the observation set is EMPTY, so every assertion over it "
        "would pass vacuously. Nothing was collected -- check the DUT emitted "
        "anything at all before trusting a green result.")
    return items


def expect_count(seq, count, what):
    """Return seq, having proved it is exactly `count` long and non-empty."""
    items = nonempty(seq, what) if count else list(seq)
    assert len(items) == count, (
        f"{what}: expected exactly {count} item(s), saw {len(items)}:\n  "
        + "\n  ".join(repr(i) for i in items[:24]))
    return items


def assert_sequence(observed, golden, what="", render=repr):
    """Whole-sequence compare with an empty-set guard and a first-diff report.

    Used for the transaction sequence of a whole enumeration run.  A per-item
    loop that happened to iterate zero times is exactly the vacuous pass this
    exists to prevent, so the length is checked FIRST and the emptiness of the
    golden is checked too -- a golden that is itself empty would make the
    comparison meaningless in the other direction.
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


# ===========================================================================
# SS ⭐ THE BRIDGED TOPOLOGY (Stage D increment 2)
#
# One virtual PCI bridge (Type 1 header) at 01:00.0 with one endpoint behind
# it at 05:00.0.  Three pieces, per docs/recon/RECON_stageD.md SS7: the Type 1 bridge
# config space, the routing/transform core, and a BDF-routing completer that
# dispatches on the wire instead of answering everything.
#
# !! THE MODEL IMPLEMENTS BASE 2.1 SS7.3.3 p.481 LITERALLY, NOT THE EXPECTED
# TRACE.  In particular the "bus outside [Secondary, Subordinate] -> UR" arm
# exists from the first line of this model, because at reset Secondary and
# Subordinate are 00h and that arm is what makes Trap C self-detecting: a
# wrongly-typed CfgWr1 for the bus-number write is answered UR automatically,
# with no test having to anticipate the mistake (docs/predictions/SPEC_PREDICTIONS_STAGE_D.md
# SS8.3).
#
# !! LATENCIES ARE NON-ZERO AND UNEQUAL (Trap D, SS8.4).  Stage D's headline
# claim is an ORDERING claim, and a zero-latency completer makes a wrong-order
# implementation unobservable.  BRIDGE_LATENCY != DEVICE_LATENCY, neither 0.
#
# !! COMPLETER ID IS CAPTURED, NOT CONFIGURED (P5.6, Base 2.1 SS2.2.9 p.99).
# Every completion carries 0000h until the addressed Function completes its
# first Type 0 Configuration Write; the capture happens AFTER that write's own
# completion is built.  The pre-D completers hardcode cpl_dw1(BDF, ...) --
# recorded bench infidelity that this model must not inherit.
# ===========================================================================

# The Stage D value table -- P5.2, five pairwise-distinct values, none equal
# to the 2b goldens (VENDOR/DEVICE above), none 0xFFFF, bus numbers 1/5/9
# non-consecutive so off-by-one is distinguishable from correct.
BRIDGE_BDF = 0x0100             # forced: the bus the existing scan probes
SEC_BUS = 0x05                  # Secondary: non-zero, != primary, != primary+1
SUB_BUS = 0x09                  # Subordinate: != Secondary
SEC_DEV_BDF = (SEC_BUS << 8)    # 05:00.0 -- bus differs, dev/fn 0 by P5.3
BRIDGE_VENDOR = 0x1AF4
BRIDGE_DEVICE = 0x1100
SEC_DEV_VENDOR = 0x15B3
SEC_DEV_DEVICE = 0x1017

# Register 6 == byte offset 18h, the Type 1 bus-number Dword ([BASE] SS7.5.3
# Figure 7-6 p.492): {Sec Latency Timer, Subordinate, Secondary, Primary}.
CFG_REG_BUS_NUMBER = 0x06
# The written value: latency byte 00h is the only defensible choice (P4.2 --
# the register is read-only 00h), and every other byte is distinct (P5.2).
BUS_NUM_WDATA = 0x00090501      # {00, SUB_BUS, SEC_BUS, 0x01}

# Trap D: response latencies in cycles, non-zero and unequal.  The device's
# total includes the bridge's forwarding hop, so the two completers are
# separated on the wire even when driven back to back.
BRIDGE_LATENCY = 5
DEVICE_LATENCY = 9


class BridgeConfigSpace:
    """A Type 1 configuration space -- [BASE] SS7.5.3 Figure 7-6 p.492.

    TWO BARs only, registers 4-5 (P4.7: SS7.5.3.1 names offsets 10h/14h and
    no others; register 6 IS the bus-number Dword, exactly where a Type 0
    header's BAR2 would sit).  Both are unimplemented-hardwired-zero here --
    the bridge requests no memory aperture in this topology, and Stage D
    never points a BAR stage at it (predictions SS10 item 7).

    Register 6 semantics:
      [31:24] Secondary Latency Timer -- READ-ONLY 00h (SS7.5.3.3 p.493:
              "must be read-only and hardwired to 00h").  Writes to the byte
              are ignored and COUNTED, so a test can prove the arm fired.
      [23:0]  Subordinate/Secondary/Primary -- byte-writable.  Primary is
              read-write but functionally inert (SS7.5.3.2 p.493); no routing
              decision below reads it.
    """

    def __init__(self, vendor=BRIDGE_VENDOR, device=BRIDGE_DEVICE):
        self.vendor = vendor
        self.device = device
        self.bus_reg = 0x0000_0000          # reset: Pri = Sec = Sub = 00h
        self.writes = []                    # (reg, value, first_be), in order
        self.latency_byte_writes_ignored = 0
        self._bars = (4, 5)                 # P4.7: the ONLY BAR registers
        self._plain = {
            CFG_REG_VENDOR_DEVICE: (device << 16) | vendor,
            CFG_REG_COMMAND_STATUS: 0x0000_0000,
            CFG_REG_CACHE_HEADER: reg3(HDR_TYPE1),
        }

    @property
    def primary(self):
        return self.bus_reg & 0xFF

    @property
    def secondary(self):
        return (self.bus_reg >> 8) & 0xFF

    @property
    def subordinate(self):
        return (self.bus_reg >> 16) & 0xFF

    def read(self, reg):
        if reg == CFG_REG_BUS_NUMBER:
            # [31:24] reads 00h REGARDLESS of what was written (P4.2).  The
            # mask is on the read path too, so even a model bug that stored
            # the byte could not present it.
            return self.bus_reg & 0x00FF_FFFF
        if reg in self._bars:
            return 0                        # unimplemented: hardwired zero
        return self._plain.get(reg)         # None -> the completer answers UR

    def write(self, reg, value, first_be=CFG_BE_DWORD):
        self.writes.append((reg, value & 0xFFFF_FFFF, first_be))
        byte_mask = 0
        for byte in range(4):
            if (first_be >> byte) & 1:
                byte_mask |= 0xFF << (8 * byte)
        if reg == CFG_REG_BUS_NUMBER:
            if byte_mask & 0xFF00_0000:
                self.latency_byte_writes_ignored += 1
            effective = byte_mask & 0x00FF_FFFF   # SS7.5.3.3: byte 3 read-only
            self.bus_reg = ((self.bus_reg & ~effective)
                            | (value & effective)) & 0x00FF_FFFF
        elif reg in self._bars:
            pass                            # unimplemented: writes discarded
        elif reg in self._plain:
            self._plain[reg] = ((self._plain[reg] & ~byte_mask)
                                | (value & byte_mask)) & 0xFFFF_FFFF
        # registers this model does not implement (1Ch..3Ch live in a real
        # Type 1 header) absorb writes; their READS return None -> UR, which
        # is the SS7.3.3 p.480 default this bench family always uses.

    @property
    def command(self):
        return self._plain[CFG_REG_COMMAND_STATUS] & 0xFFFF


class BridgedTopology:
    """The routing/transform core -- PURE, no cocotb, so it can be self-tested
    at import time exactly like _selftest_type1_one_bit.

    handle(dwords) takes one request in wire-Dword form and returns
      (who, status, rdata, completer_id)
    where who is "bridge" or "device", status a CPL_* value, rdata the read
    data (None unless SC read), and completer_id what the completion's DW1
    must carry (the P5.6 captured value, 0000h through the probe phase).

    Base 2.1 SS7.3.3 p.481, applied in sequence to a Type 1 request:
      1. bus == Secondary            -> transform Type[0] 1->0, NOTHING else,
                                        deliver to the device.  The model
                                        asserts received-vs-forwarded itself:
                                        DW0 differs in bit 0 only, DW1/DW2
                                        byte-identical.
      2. Secondary < bus <= Subord.  -> forward WITHOUT modification.
                                        Implemented, NOT claimed as covered
                                        (P3.2: unreachable at one level with
                                        Secondary == the only populated bus).
      3. else                        -> Unsupported Request.
    A Type 0 request is claimed locally by the bridge (it IS the link
    partner); device numbers 1-31 answer UR (SS7.3.1 p.479), the same rule on
    the secondary link (P5.3 -- point-to-point, Device 0 only).
    The bridge NEVER synthesises CRS for the device (P6.2): the CRS hooks are
    per-Function, and a forwarded request can only be CRS'd by the device.
    """

    def __init__(self, bridge=None, device=None,
                 bridge_crs_once=(), device_crs_once=()):
        self.bridge = bridge if bridge is not None else BridgeConfigSpace()
        self.device = device if device is not None else ConfigDevice(
            vendor=SEC_DEV_VENDOR, device=SEC_DEV_DEVICE)
        self.bridge_captured_id = 0x0000    # P5.6: 0000h until first CfgWr0
        self.device_captured_id = 0x0000
        self.bridge_crs_once = set(bridge_crs_once)
        self.device_crs_once = set(device_crs_once)
        # Guard counters -- a guard never seen firing is not known to work.
        self.transforms = []                # (received dwords, forwarded dwords)
        self.route_ur_hits = 0              # SS7.3.3 case 3 (Trap C's teeth)
        self.forward_unmodified_hits = 0    # case 2 -- implemented, uncovered
        self.bridge_dev_ur_hits = 0         # Type 0 naming device != 0
        self.device_dev_ur_hits = 0         # same rule, secondary link
        self.device_type1_ur_hits = 0       # P3.3: an endpoint URs any CFG1
        self.bridge_crs_hits = 0
        self.device_crs_hits = 0

    # ---- the SS7.3.3 dispatch ---------------------------------------------
    def handle(self, dwords):
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
            self.route_ur_hits += 1                 # case 3: reset state UR
            return ("bridge", CPL_UR, None, self.bridge_captured_id)
        raise AssertionError(
            f"the bridged topology received a non-config TLP "
            f"(tlp_type {req.tlp_type:#07b}): {req!r}")

    def _transform(self, dwords):
        """SS7.3.3 case 1: 'changing the value in the Type[4:0] field ... all
        other fields of the Request remain unchanged'.  Asserted, not trusted:
        this is bench code that behaves like hardware."""
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
        if req.dev != 0 or req.fn != 0:
            self.bridge_dev_ur_hits += 1            # SS7.3.1 p.479
            return ("bridge", CPL_UR, None, self.bridge_captured_id)
        if req.reg_num in self.bridge_crs_once:
            self.bridge_crs_once.discard(req.reg_num)
            self.bridge_crs_hits += 1               # its OWN access only (P6.2)
            return ("bridge", CPL_CRS, None, self.bridge_captured_id)
        if not req.is_read:
            self.bridge.write(req.reg_num,
                              req.payload[0] if req.payload else 0,
                              req.first_be)
            cid = self.bridge_captured_id           # THIS completion: pre-capture
            self.bridge_captured_id = (req.bus << 8) | (req.dev << 3) | req.fn
            return ("bridge", CPL_SC, None, cid)
        value = self.bridge.read(req.reg_num)
        if value is None:
            return ("bridge", CPL_UR, None, self.bridge_captured_id)
        return ("bridge", CPL_SC, value, self.bridge_captured_id)

    # ---- the device behind the bridge --------------------------------------
    def _device_claim(self, req):
        if req.tlp_type == TYPE_CFG1:
            self.device_type1_ur_hits += 1          # P3.3: SS7.3.3 p.480,
            return ("device", CPL_UR, None, self.device_captured_id)  # Endpoints
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
        """Trap D: non-zero, unequal.  The bridge's own answers take
        BRIDGE_LATENCY; anything the device answers crossed the bridge twice
        and takes BRIDGE_LATENCY + DEVICE_LATENCY."""
        return BRIDGE_LATENCY if who == "bridge" \
            else BRIDGE_LATENCY + DEVICE_LATENCY


class BridgedCompleter:
    """The BDF-routing socket: the four-name interface (.start / .seen /
    .wait_for / .complete) over a BridgedTopology, dispatching each request on
    the wire instead of answering everything.  Same DLL-stream mechanics as
    every _tlp completer; the routing decision and all policy live in the pure
    core so they stay self-testable without a simulator."""

    def __init__(self, dut, topo=None):
        self.dut = dut
        self.topo = topo if topo is not None else BridgedTopology()
        self.seen = []                      # every request TLP, in wire order
        self.answers = []                   # (req, who, status), in order
        self._partial = []
        self._answered = 0

    def start(self):
        cocotb.start_soon(self._watch_tx())

    async def wait_for(self, count, cycles=40000):
        for _ in range(cycles):
            await RisingEdge(self.dut.clk_i)
            if len(self.seen) >= count:
                return
        raise AssertionError(
            f"expected {count} request TLPs on the wire, saw {len(self.seen)} "
            f"({self.seen}) -- FC credits, or the sequence never issued?")

    async def complete(self, req, status=CPL_SC, data=None, completer_id=0):
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
                who, status, rdata, cid = self.topo.handle(req.dwords)
                # Trap D: the observable response window.  Serialized -- the
                # primitive is single-outstanding, so responses never overlap.
                for _ in range(self.topo.latency_for(who)):
                    await RisingEdge(self.dut.clk_i)
                self.answers.append((req, who, status))
                await self.complete(req, status=status, data=rdata,
                                    completer_id=cid)

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


def _selftest_bridged_topology():
    """⭐ The model's own guards, seen firing BEFORE any DUT exists.

    Brief: 'a guard never seen firing is not known to work.'  The three
    mandated drives, on the PURE core at import time in every bench:
      1. a wrongly-typed CfgWr1 to 18h at reset  -> UR, register untouched
         (Trap C: the reset-state route arm is self-detecting);
      2. a raw Type 1 reaching the device        -> UR (P3.3);
      3. post-assignment, bus == Secondary       -> the one-bit transform.
    Plus the P4.2 latency-byte ignore and the P5.6 capture sequence, because
    both are model arms a later test will lean on.
    """
    rd = cfg_wire_dw0(False)
    rd1 = cfg_wire_dw0(False, type1=True)
    wr = cfg_wire_dw0(True)
    wr1 = cfg_wire_dw0(True, type1=True)
    dw1 = cfg_wire_dw1(RID, 0x21, CFG_BE_DWORD)

    # 1 -- Trap C's teeth.  Secondary == Subordinate == 00h, bus 1 is outside
    # [0, 0] ... and outside every post-reset aperture too.
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

    # 3 -- the transform, now that bus 5 == Secondary.
    who, status, data, cid = topo.handle(
        [rd1, dw1, cfg_wire_dw2(SEC_BUS, 0, 0, CFG_REG_VENDOR_DEVICE)])
    assert (who, status) == ("device", CPL_SC)
    assert data == (SEC_DEV_DEVICE << 16) | SEC_DEV_VENDOR
    assert len(topo.transforms) == 1, "the transform arm did not run"
    received, forwarded = topo.transforms[0]
    assert received[0] ^ forwarded[0] == 1 and received[1:] == forwarded[1:]
    assert cid == 0x0000, \
        "probe-phase completer ID must be 0000h at the device too (Trap B)"

    # 2 -- a raw Type 1 reaching the device: the forward-unmodified arm
    # (implemented, NOT claimed covered -- P3.2) delivers it untransformed and
    # the device URs it, which is the free cross-check that transforms happen.
    who, status, data, cid = topo.handle(
        [rd1, dw1, cfg_wire_dw2(0x07, 0, 0, CFG_REG_VENDOR_DEVICE)])
    assert (who, status) == ("device", CPL_UR), (who, status)
    assert topo.forward_unmodified_hits == 1 and topo.device_type1_ur_hits == 1

    # Out-of-aperture stays UR post-assignment (bus 2 < Secondary).
    who, status, _, _ = topo.handle(
        [rd1, dw1, cfg_wire_dw2(0x02, 0, 0, CFG_REG_VENDOR_DEVICE)])
    assert (who, status) == ("bridge", CPL_UR) and topo.route_ur_hits == 2

    # P4.2 -- the latency byte is ignored on write and 00h on read, even when
    # the writer drives it non-zero.
    topo.handle([wr, dw1, cfg_wire_dw2(0x01, 0, 0, CFG_REG_BUS_NUMBER),
                 0xAA00_0000 | BUS_NUM_WDATA])
    assert topo.bridge.latency_byte_writes_ignored >= 1
    _, _, readback, _ = topo.handle(
        [rd, dw1, cfg_wire_dw2(0x01, 0, 0, CFG_REG_BUS_NUMBER)])
    assert readback == BUS_NUM_WDATA, (
        f"18h readback {readback:#010x}: [31:24] must be 00h regardless of "
        "what was written (SS7.5.3.3 p.493)")

    # SS7.3.1 device-number rule, both links.
    _, status, _, _ = topo.handle(
        [rd, dw1, cfg_wire_dw2(0x01, 3, 0, CFG_REG_VENDOR_DEVICE)])
    assert status == CPL_UR and topo.bridge_dev_ur_hits == 1
    _, status, _, _ = topo.handle(
        [rd1, dw1, cfg_wire_dw2(SEC_BUS, 3, 0, CFG_REG_VENDOR_DEVICE)])
    assert status == CPL_UR and topo.device_dev_ur_hits == 1

    # P5.6 completes: the device captures on its first CfgWr0 (post-transform).
    topo.handle([wr1, dw1, cfg_wire_dw2(SEC_BUS, 0, 0, CFG_REG_BAR0), 0xFFFFFFFF])
    assert topo.device_captured_id == SEC_DEV_BDF
    _, _, _, cid = topo.handle(
        [rd1, dw1, cfg_wire_dw2(SEC_BUS, 0, 0, CFG_REG_BAR0)])
    assert cid == SEC_DEV_BDF, "post-capture completions must carry the BDF"

    # The value table really is pairwise-distinct (Trap B's precondition).
    ids = {VENDOR, DEVICE, BRIDGE_VENDOR, BRIDGE_DEVICE,
           SEC_DEV_VENDOR, SEC_DEV_DEVICE}
    assert len(ids) == 6 and 0xFFFF not in ids
    assert len({0x01, SEC_BUS, SUB_BUS}) == 3


_selftest_bridged_topology()
