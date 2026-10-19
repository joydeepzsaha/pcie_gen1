"""lr1_host -- the host side of pcie_rc_top's requester socket, for LR1-b rows

Author: Kourosh Ghahramani
Silicon Systems Research Lab, University of Washington

Some LR1-b rows need TLPs the RC originates after enumeration (F1, C3, the
Reserved-bit Ack). The RC takes host requests on s_axis_rq_* in PG213's
Requester Request format, Dword-aligned, at 128 bits (PG213 Table 13; the
descriptor, Table 60; request types, Table 57):

  beat 0      the 16-byte descriptor: [1:0] Address Type 00b, [63:2]
              Address[63:2], [74:64] Dword Count, [78:75] Request Type,
              [103:96] Tag, [123:121] TC, [126:124] Attr; the rest 0
  beat 1..    the payload, four Dwords a beat, Dword 0 in [31:0], byte 0 of
              each Dword in its [7:0]
  tuser       [3:0] first_be, [7:4] last_be (Table 13)
  tlast       on the last beat; tkeep all ones (the RC does not read it)

The RC accepts host requests only after enumeration (rq_engine_owns_o low).

This is the RC's user-side socket, not link protocol; it imports only the
standard library and cocotb (BRIEF_LR1 D4).
"""
import cocotb
from cocotb.triggers import RisingEdge

REQ_MRD = 0b0000          # PG213 Table 57
REQ_MWR = 0b0001


def descriptor(req_type, addr, dwords, tag=0, tc=0, attr=0):
    """The 128-bit Requester Request descriptor (PG213 Table 60)."""
    return ((addr & ~0x3) & ((1 << 64) - 1)) | ((dwords & 0x7FF) << 64) | ((req_type & 0xF) << 75) | \
        ((tag & 0xFF) << 96) | ((tc & 0x7) << 121) | ((attr & 0x7) << 124)


def beats(req_type, addr, data=b'', dwords=None, tag=0):
    """The request as a list of 128-bit beats."""
    if dwords is None:
        dwords = max(1, len(data) // 4)
    out = [descriptor(req_type, addr, dwords, tag)]
    if req_type == REQ_MWR:
        assert len(data) == 4 * dwords
        for i in range(0, len(data), 16):
            out.append(int.from_bytes(bytes(data[i:i + 16]).ljust(16, b'\0'), 'little'))
    return out


class Host:
    """Drives s_axis_rq_*; one request at a time. `cycle_fn` reads the far
    end's clock count, so each request's acceptance is timed on the same
    scale as the far end's records."""

    def __init__(self, dut, cycle_fn):
        self.dut = dut
        self.now = cycle_fn
        self.log = []                      # (cycle the last beat was taken, name, addr, dwords, cycle the first was offered)

    def idle(self):
        d = self.dut
        d.s_axis_rq_tvalid.value = 0
        d.s_axis_rq_tlast.value = 0

    async def request(self, req_type, addr, data=b'', dwords=None, first_be=0xF, last_be=None):
        d = self.dut
        bl = beats(req_type, addr, data, dwords)
        n = dwords if dwords is not None else max(1, len(data) // 4)
        if last_be is None:
            last_be = 0 if n == 1 else 0xF
        t_first = self.now()
        d.s_axis_rq_tuser.value = ((last_be & 0xF) << 4) | (first_be & 0xF)
        d.s_axis_rq_tkeep.value = 0xF
        for i, b in enumerate(bl):
            d.s_axis_rq_tdata.value = b
            d.s_axis_rq_tlast.value = int(i == len(bl) - 1)
            d.s_axis_rq_tvalid.value = 1
            while True:
                await RisingEdge(d.clk_i)
                if int(d.s_axis_rq_tready.value):       # the value at this edge: the beat is taken
                    break
        self.idle()
        self.log.append((self.now(), 'MWr' if req_type == REQ_MWR else 'MRd', addr, n, t_first))
        return self.now()

    async def mwr(self, addr, data):
        return await self.request(REQ_MWR, addr, data)

    async def mrd(self, addr, dwords):
        return await self.request(REQ_MRD, addr, dwords=dwords)


def start(coro):
    return cocotb.start_soon(coro)
