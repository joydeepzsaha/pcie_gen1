"""T1b: pins the tracker's DEFAULT CPL_TIMEOUT_CYCLES.

⚠️ CORRECTED AT §63 #7e -- THIS DOCSTRING USED TO CLAIM SOMETHING FALSE. It
said the target "sets no parameter at all, so this exercises the value the RTL
ships with". The target sets no fusesoc `parameters:` entry, true -- but
tb_tlp_request_tracker.sv declares its OWN CPL_TIMEOUT_CYCLES default and passes
it to the DUT, so this row has always pinned a BENCH-LOCAL COPY of the number
and never the RTL default. A change to the RTL default alone would have left
this row green over a stale value. It was found only because §63 #7e changed the
RTL default and this row kept firing at k=4127.

The two are now equal at 6250 and the duplication is documented loudly at
tb_tlp_request_tracker.sv:5. What this row proves is that the SHIPPED-EQUIVALENT
value behaves correctly -- not, by itself, that the two numbers agree.  The rest of the mechanism is
covered at 64 cycles by test_tlp_cpl_timeout.py; the only thing proved here is
that the default really is 6250 -- nothing fires before it, and it fires inside
the one-scan-period window after it.

== §63 #7e, CONFORMANCE DEFECT #7: THE DEFAULT MOVED 4096 -> 6250 ==========

⚠️ THIS ROW MOVED BY CONSTRUCTION AND WAS PREDICTED TO (P1). Its whole purpose
is to pin the shipped default, so a change to that default MUST move it; a row
that pinned the old value and still passed would mean the edit had not landed.
The name changed too -- `t1b_default_timeout_is_4096` named the constant, so
leaving the name while changing the number would have been a lie in the one
place a reader looks first.

THE NEW VALUE IS NOT A PREFERENCE. Base 2.1 §7.8.16 Table 7-25 (pp.549-550): a
Function without Completion Timeout programmability "is required to implement a
timeout value in the range 50 µs to 50 ms". At the 8 ns clock this design runs
at, 50 µs = 6250 cycles, and the old 4096 was 32.8 µs -- BELOW the floor of the
required range, i.e. non-conformant, not merely inconvenient.

⚠️ MEASURED, NOT ARGUED: §63 #7e timed a real CfgRd0 -> CplD round trip through
two PHYs and the codec bridge at 5122 cycles = 41.0 µs. The old default expired
BEFORE that legitimate Completion returned. The link was inside spec and the
timeout was not.

⚠️ AND 6250 IS THE MINIMUM, WHICH IS TIGHT: it clears that measured round trip
by only 1128 cycles. tb_pcie_fullstack overrides to 65536 for that reason. A
bench needing headroom must ask for it rather than lean on the shipped floor.

⚠️⚠️ THE PARAMETER IS IN CYCLES, SO ITS CONFORMANCE MEANING IS CLOCK-DEPENDENT,
AND THIS BENCH DOES NOT RUN AT THE DESIGN'S CLOCK. CLK_NS below is 10 ns, so
6250 cycles is 62.5 µs HERE, while the §7.8.16 arithmetic that chose 6250 is
50 µs at the design's 8 ns. Both are inside the required 50 µs - 50 ms range, so
this row is not affected -- but the equivalence "6250 == 50 µs" holds ONLY at
125 MHz. A future reader re-deriving the constant from this file's own CLK_NS
would get the wrong answer, and a future clock change makes the default
non-conformant again without any parameter moving. Recorded here because this
row is where someone will look.
"""

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, Timer

TAG_COUNT = 32
DEFAULT_TIMEOUT = 6250  # §7.8.16 minimum: 50 us / 8 ns
CLK_NS = 10
RID = 0x1234


@cocotb.test()
async def t1b_default_timeout_is_6250_the_spec_minimum(dut):
    cocotb.start_soon(Clock(dut.clk_i, CLK_NS, units="ns").start())
    dut.rst_i.value = 1
    for name in ("allocate_valid", "completion_valid", "extended_tag_enable",
                 "allocate_requester_id", "allocate_byte_count", "allocate_address",
                 "allocate_context", "allocate_expects_data", "completion_requester_id",
                 "completion_tag", "completion_status", "completion_payload_bytes",
                 "completion_byte_count", "completion_lower_address"):
        getattr(dut, name).value = 0
    dut.result_ready.value = 1
    for _ in range(3):
        await RisingEdge(dut.clk_i)
    dut.rst_i.value = 0
    await RisingEdge(dut.clk_i)
    await Timer(1, units="ps")

    dut.allocate_requester_id.value = RID
    dut.allocate_byte_count.value = 4
    dut.allocate_expects_data.value = 1
    dut.allocate_valid.value = 1
    await Timer(1, units="ps")
    while not int(dut.allocate_ready.value):
        await RisingEdge(dut.clk_i)
        await Timer(1, units="ps")
    tag = int(dut.allocate_tag.value)
    await RisingEdge(dut.clk_i)
    await Timer(1, units="ps")
    dut.allocate_valid.value = 0
    assert tag == 0

    fired_at = None
    for k in range(1, DEFAULT_TIMEOUT + TAG_COUNT + 8):
        await RisingEdge(dut.clk_i)
        await Timer(1, units="ps")
        if int(dut.cpl_timeout_valid.value):
            assert fired_at is None, "cpl_timeout_valid must be a one-cycle strobe"
            fired_at = k
            assert int(dut.cpl_timeout_tag.value) == tag

    assert fired_at is not None, \
        f"no timeout within {DEFAULT_TIMEOUT + TAG_COUNT + 7} cycles -- default is too large"
    assert fired_at >= DEFAULT_TIMEOUT, (
        f"fired at k={fired_at}, EARLIER than the {DEFAULT_TIMEOUT}-cycle default -- "
        "the default is smaller than documented")
    assert fired_at <= DEFAULT_TIMEOUT + TAG_COUNT - 1, (
        f"fired at k={fired_at}, later than one scan period past {DEFAULT_TIMEOUT} -- "
        "the default is larger than documented")
