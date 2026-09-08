import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, Timer


async def reset(dut):
    dut.rst_i.value = 1
    for name in [
        "completion_request_valid", "request_requester_id", "request_tag",
        "request_tc", "request_attr", "completion_request_status",
        "completion_request_byte_count", "completion_request_lower_address",
        "completion_request_digest_valid",
        "completion_request_data", "completion_request_keep",
        "completion_request_data_valid", "completion_request_data_last",
        "requester_header_valid", "requester_has_data", "requester_data",
        "requester_keep", "requester_data_valid", "requester_data_last",
        "generator_header_ready", "generator_data_ready",
        "fair_requester_valid", "fair_completion_valid", "fair_generator_ready",
    ]:
        getattr(dut, name).value = 0
    dut.completer_id.value = 0xCAFE
    dut.max_payload_bytes.value = 128
    dut.rcb_128b.value = 1
    for _ in range(3):
        await RisingEdge(dut.clk_i)
    dut.rst_i.value = 0
    await RisingEdge(dut.clk_i)


async def submit_completion(dut, count, status=0, lower=3, attr=5):
    dut.request_requester_id.value = 0x1234
    dut.request_tag.value = 0x56
    dut.request_tc.value = 3
    dut.request_attr.value = attr
    dut.completion_request_status.value = status
    dut.completion_request_byte_count.value = count
    dut.completion_request_lower_address.value = lower
    dut.completion_request_valid.value = 1
    while not int(dut.completion_request_ready.value):
        await RisingEdge(dut.clk_i)
    await RisingEdge(dut.clk_i)
    dut.completion_request_valid.value = 0


@cocotb.test()
async def completion_priority_fields_and_packet_lock(dut):
    """A Completion must not pass a queued Posted Request; with RO Set it may.

    Base 2.1 SS2.4.1 Table 2-33 p.122-123, Row D (Completion) x Col 2 (Posted
    Request) = "a) No", spelled out at D2a p.124:

        "A Completion must not pass a Posted Request unless D2b applies.  If
        the Relaxed Ordering attribute bit is not set, then a Read Completion
        cannot pass a previously enqueued Memory Write or Message Request."

    and its exception, D2b p.124: "A Completion with RO Set is permitted to
    pass a Posted Request."

    !! REWRITTEN AT STAGE F-2, AND THE OLD VERSION IS WHY THE DEFECT SURVIVED.
    Until F-2 this test asserted the OPPOSITE.  Its own comment read
    "Completion has priority when both headers are pending" and it required
    generator_type == TLP_TYPE_CPL while a Memory Write header was pending.
    The wrapper builds that requester header as TLP_TYPE_MEM + TLP_FMT_3DW_DATA
    with attributes '0 (tb_tlp_completion_control.sv:68-69) -- a POSTED request
    with RO clear -- and submit_completion drives request_attr = 0b101, whose
    bit 1 (RO) is CLEAR and which tlp_completion_generator.sv:110 copies
    verbatim into the completion header.  So the old assertion was Table 2-33
    Row D / Col 2 a) written down as intended behaviour.

    THE ORDERING DEFECT WAS NOT MERELY UNTESTED -- IT WAS ASSERTED, in a unit
    test of the very module that had to change.  That is why it survived every
    rung until Stage F-1 made the two streams contend on a real path, and it is
    why the F-2 cold gate stopped here rather than on any of F-2's new work.

    Two arms, differing in EXACTLY ONE BIT of request_attr:

        arm A   attr = 0b101   RO CLEAR  ->  the Posted requester wins   (D2a)
        arm B   attr = 0b111   RO SET    ->  the Completion may pass     (D2b)

    Holding the other two Attr bits (IDO at [2], No Snoop at [0]) SET in both
    arms is deliberate: Relaxed Ordering is then the only difference between
    them, so passing both isolates RO as the cause rather than "some attribute
    changed".  !! RO is attributes[1] -- Attr is split across two non-adjacent
    header bytes and attributes[0] is No Snoop, not RO (tlp_generator.sv:66-78,
    tlp_parser.sv:125).  Reading bit 0 here would gate on No Snoop and look
    entirely plausible; M-2 already caught one such misplacement in this tree.

    Arm B also carries everything the original test proved and that F-2 must
    not lose -- the completion descriptor's fields, the payload packet-lock,
    and the requester being selected only after completion EOP.  They live in
    arm B because that is now the only arm in which the completion is granted
    first.
    """
    cocotb.start_soon(Clock(dut.clk_i, 10, units="ns").start())

    # ---- arm A: RO CLEAR -- the queued Posted request wins (D2a) ----------
    await reset(dut)
    dut.requester_has_data.value = 1        # -> TLP_FMT_3DW_DATA, i.e. a MemWr
    dut.requester_header_valid.value = 1
    await submit_completion(dut, 5, status=0, attr=0b101)   # RO clear

    dut.generator_header_ready.value = 1
    await Timer(1, units="ps")

    # premise: this must be a CONTENDED cycle, or "the requester wins" is not
    # an ordering result at all -- it is just the only header there was.
    assert int(dut.completion_header_valid.value) == 1, (
        "premise: the completion header must be pending, else arm A proves "
        "nothing about ordering")
    assert int(dut.requester_header_valid.value) == 1, "premise: MemWr pending"

    assert int(dut.generator_header_valid.value) == 1
    assert int(dut.generator_type.value) == 0, (
        f"Base 2.1 Table 2-33 Row D / Col 2 a): a Completion must not pass a "
        f"queued Posted Request.  A Memory Write header was pending with RO "
        f"clear, so the requester must be granted -- but generator_type is "
        f"{int(dut.generator_type.value)} (10 = TLP_TYPE_CPL, 0 = TLP_TYPE_MEM)")
    assert int(dut.requester_header_ready.value) == 1, (
        "the requester must actually be handshaked, not merely selected")
    assert int(dut.completion_header_ready.value) == 0, (
        "the completion must be HELD while the posted request drains")
    await RisingEdge(dut.clk_i)

    # ---- arm B: RO SET -- D2b permits the Completion to pass --------------
    await reset(dut)
    dut.requester_has_data.value = 1
    dut.requester_header_valid.value = 1
    await submit_completion(dut, 5, status=0, attr=0b111)   # RO set: one bit

    dut.generator_header_ready.value = 1
    await Timer(1, units="ps")
    assert int(dut.requester_header_valid.value) == 1, (
        "premise: the SAME posted contention as arm A, so the only difference "
        "between the two arms is the RO bit")
    assert int(dut.generator_header_valid.value) == 1
    assert int(dut.generator_type.value) == 10, (
        f"Base 2.1 Table 2-33 D2b: a Completion with RO Set is permitted to "
        f"pass a Posted Request, and this design honours that exception "
        f"(F-2 Decision 3).  generator_type is {int(dut.generator_type.value)}, "
        f"so the posted-aware term is OVER-BLOCKING: it is gating on the "
        f"pending posted header without excepting Relaxed Ordering")
    assert int(dut.generator_fmt.value) == 2
    assert int(dut.generator_requester_id.value) == 0x1234
    assert int(dut.generator_completer_id.value) == 0xCAFE
    assert int(dut.generator_tag.value) == 0x56
    assert int(dut.generator_byte_count.value) == 5
    assert int(dut.generator_lower_address.value) == 3
    await RisingEdge(dut.clk_i)
    dut.generator_header_ready.value = 0

    # While completion payload is locked, requester cannot interleave.
    dut.completion_request_data.value = 0x44332211
    dut.completion_request_keep.value = 0xF
    dut.completion_request_data_last.value = 0
    dut.completion_request_data_valid.value = 1
    dut.generator_data_ready.value = 0
    for _ in range(4):
        await RisingEdge(dut.clk_i)
        assert int(dut.requester_header_ready.value) == 0
        assert int(dut.generator_data.value) == 0x44332211
    dut.generator_data_ready.value = 1
    await RisingEdge(dut.clk_i)
    dut.completion_request_data.value = 0x55
    dut.completion_request_keep.value = 1
    dut.completion_request_data_last.value = 1
    await RisingEdge(dut.clk_i)
    dut.completion_request_data_valid.value = 0

    # The requester header is selected only after completion EOP.
    dut.generator_header_ready.value = 1
    await Timer(1, units="ps")
    assert int(dut.generator_type.value) == 0
    await RisingEdge(dut.clk_i)
    await no_data_error_completion_and_reset_lock(dut)


async def no_data_error_completion_and_reset_lock(dut):
    await reset(dut)
    await submit_completion(dut, 0, status=1)
    dut.generator_header_ready.value = 0
    for _ in range(3):
        await RisingEdge(dut.clk_i)
        assert int(dut.generator_header_valid.value) == 1
        assert int(dut.generator_fmt.value) == 0
        assert int(dut.generator_status.value) == 1
    dut.generator_header_ready.value = 1
    await RisingEdge(dut.clk_i)

    dut.requester_has_data.value = 1
    dut.requester_header_valid.value = 1
    await RisingEdge(dut.clk_i)
    dut.requester_header_valid.value = 0
    dut.rst_i.value = 1
    await RisingEdge(dut.clk_i)
    dut.rst_i.value = 0
    await RisingEdge(dut.clk_i)
    assert int(dut.generator_data_valid.value) == 0

    # A 200-byte completion is divided at the 128-byte RCB/MPS boundary.
    await reset(dut)
    await submit_completion(dut, 200, lower=0)
    dut.generator_header_ready.value = 1
    await Timer(1, units="ps")
    assert int(dut.generator_byte_count.value) == 200
    await RisingEdge(dut.clk_i)
    dut.generator_header_ready.value = 0
    dut.generator_data_ready.value = 1
    dut.completion_request_data_valid.value = 1
    dut.completion_request_keep.value = 0xF
    dut.completion_request_data_last.value = 0
    for beat in range(32):
        dut.completion_request_data.value = beat
        await RisingEdge(dut.clk_i)
    dut.completion_request_data_valid.value = 0
    for _ in range(10):
        await RisingEdge(dut.clk_i)
        if int(dut.generator_header_valid.value):
            break
    else:
        raise AssertionError("second split-completion header missing")
    assert int(dut.generator_byte_count.value) == 72
    dut.generator_header_ready.value = 1
    await RisingEdge(dut.clk_i)
    dut.generator_header_ready.value = 0
    dut.completion_request_data_valid.value = 1
    for beat in range(18):
        dut.completion_request_data.value = 0x100 + beat
        dut.completion_request_data_last.value = beat == 17
        await RisingEdge(dut.clk_i)
    dut.completion_request_data_valid.value = 0
    dut.completion_request_data_last.value = 0
    await RisingEdge(dut.clk_i)
    assert int(dut.completion_error_valid.value) == 0


@cocotb.test()
async def packet_boundary_round_robin_prevents_starvation(dut):
    cocotb.start_soon(Clock(dut.clk_i, 10, units="ns").start())
    await reset(dut)
    dut.requester_has_data.value = 0
    dut.fair_requester_valid.value = 1
    dut.fair_completion_valid.value = 1
    dut.fair_generator_ready.value = 1

    # Reset preference is completion, preserving prompt completion service.
    await Timer(1, units="ps")
    assert int(dut.fair_generator_valid.value) == 1
    assert int(dut.fair_generator_type.value) == 10
    await RisingEdge(dut.clk_i)

    # With both sources continuously pending, the next packet belongs to requester.
    await Timer(1, units="ps")
    assert int(dut.fair_generator_type.value) == 0
    await RisingEdge(dut.clk_i)

    # Completion is selected again on the following packet boundary.
    await Timer(1, units="ps")
    assert int(dut.fair_generator_valid.value) == 1
    assert int(dut.fair_generator_type.value) == 10
