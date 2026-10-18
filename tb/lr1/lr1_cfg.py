"""lr1_cfg -- the far end's Transaction Layer: one Function with a Type 0 space

Author: Kourosh Ghahramani
Silicon Systems Research Lab, University of Washington

Configuration space (Base 2.1 §7.5, Figure 7-5 p.491 "Type 0 Configuration
Space Header"):
  00h  Device ID / Vendor ID                       RO (bench values, below)
  04h  Status / Command                            Command RW: I/O, Memory, Bus
       Master, Parity Error Response, SERR#, Interrupt Disable (§7.5.1.1);
       Status.Capabilities List = 1 (p.487), the error bits RW1C (all 0 here)
  08h  Class Code / Revision ID                    RO
  0Ch  BIST / Header Type 00h / Latency Timer / Cache Line Size (RW)
  10h  BAR0-1: one 64-bit memory BAR, size bar_size, non-prefetchable (low
       nibble 4h) or prefetchable (Ch) -- the encoding is PCI Local Bus 3.0
       §6.2.5.1; the address bits above the size are RW, the rest RO
  18h-24h  BAR2-5 hardwired 0 (not implemented)
  2Ch  Subsystem ID / Subsystem Vendor ID          RO
  30h  Expansion ROM BAR 0 (not implemented)
  34h  Capabilities Pointer = 40h
  3Ch  Interrupt Pin 0 (no INTx), Interrupt Line RW
  40h  PCI Power Management Capability (§7.6 p.496; PCI PM 1.2, Version 011b),
       next 48h; PMCSR read-only (RULINGS_LR1 I: read-only and minimal)
  48h  PCI Express Capability (§7.8 pp.499-556), version 2, Device/Port Type
       0000b (PCI Express Endpoint); Device Capabilities: Max_Payload_Size
       Supported 128 B, Role-Based Error Reporting 1; Link Capabilities: 2.5
       GT/s, x1, ASPM L0s (01b, the only non-reserved choice short of L0s+L1,
       Table 7-15 p.518); Link Status: 2.5 GT/s, x1, Slot Clock 1; Link
       Control 2 Target Link Speed 0001b. Read-only.
  100h Extended Capability header 0 (no extended capabilities, §7.9.1)

Request handling (§7.3.1 p.479, §7.3.3 p.480, §2.2.9 pp.97-99):
  - Type 1 Configuration Requests: Unsupported Request.
  - Type 0 to Function 0, any Device Number (a non-ARI device answers them
    all, p.479): processed; to any other Function: Unsupported Request.
  - CfgWr0 captures the Bus and Device Numbers for the Completer ID (§2.2.6.2);
    before the first one the Completer ID is 0000h.
  - Completions: Byte Count 4, Lower Address 0 (p.97); UR read completions
    carry no data (Cpl).
  - Memory Read / Write inside BAR0 with Memory Space Enable: served from a
    small memory; otherwise UR (reads) or dropped (writes, posted).
  - CRS for `crs_cycles` after the first Configuration Request (knob V19;
    p.110: only after reset, never after a Successful Completion).
"""
import lr1_consts as C
import lr1_tlp as TLP

VENDOR_ID = 0x5AA5        # bench values; not an assigned ID
DEVICE_ID = 0x1A01
SUBSYS_VID = 0x5AA5
SUBSYS_ID = 0x0001
CLASS_CODE = 0xFF0000     # "device does not fit any defined class"


class ConfigSpace:
    def __init__(self, knobs):
        self.k = knobs
        size = knobs.bar_size
        assert size >= 128 and size & (size - 1) == 0
        low = 0xC if knobs.bar_prefetchable else 0x4
        self.ro = {}
        self.rw_mask = {}
        self.val = {}

        def reg(off, value, rw=0):
            self.val[off] = value
            self.rw_mask[off] = rw

        reg(0x00, (DEVICE_ID << 16) | VENDOR_ID)
        reg(0x04, (1 << 4) << 16, rw=0x0547)          # Status.CapList; Command RW bits 0,1,2,6,8,10
        reg(0x08, (CLASS_CODE << 8) | 0x00)
        reg(0x0C, 0x00000000, rw=0x000000FF)           # Header Type 00h; Cache Line Size RW
        reg(0x10, low, rw=(~(size - 1)) & 0xFFFFFFF0 & 0xFFFFFFFF)
        reg(0x14, 0x00000000, rw=0xFFFFFFFF)           # upper 32 bits of the 64-bit BAR
        for off in (0x18, 0x1C, 0x20, 0x24, 0x28, 0x30, 0x38):
            reg(off, 0)
        reg(0x2C, (SUBSYS_ID << 16) | SUBSYS_VID)
        reg(0x34, 0x40)
        reg(0x3C, 0x00000000, rw=0x000000FF)           # Interrupt Pin 0, Interrupt Line RW
        reg(0x40, (0x3 << 16) | (0x48 << 8) | 0x01)    # PMC version 011b, next 48h, ID 01h
        reg(0x44, 0x00000000)                          # PMCSR: D0
        reg(0x48, (0x0002 << 16) | (0x00 << 8) | 0x10)  # PCIe Cap: version 2, Endpoint; next 00h, ID 10h
        reg(0x4C, (1 << 15) | (0x7 << 9) | (0x7 << 6) | 0x0)  # Device Cap: RBER, L1/L0s accept = no limit, MPS 128
        reg(0x50, 0x00002810)                          # Device Control defaults (RO here); Device Status 0
        reg(0x54, (0x7 << 15) | (0x7 << 12) | (0x1 << 10) | (0x01 << 4) | 0x1)   # Link Cap
        reg(0x58, (0x1011 << 16))                      # Link Status: Slot Clock, x1, 2.5 GT/s
        reg(0x78, 0x00000001)                          # Link Control 2: Target Link Speed 2.5 GT/s
        self.bus = 0
        self.dev = 0

    def read(self, off):
        return self.val.get(off & 0xFFC, 0)

    def write(self, off, be, data4):
        off &= 0xFFC
        if off not in self.val:
            return
        cur = self.val[off]
        new = int.from_bytes(bytes(data4), 'little')
        mask = 0
        for i in range(4):
            if be & (1 << i):
                mask |= 0xFF << (8 * i)
        mask &= self.rw_mask[off]
        self.val[off] = (cur & ~mask) | (new & mask)

    def mem_enabled(self):
        return bool(self.val[0x04] & 0x2)

    def bar_base(self):
        return ((self.val[0x14] << 32) | self.val[0x10]) & ~0xF


class Completer:
    def __init__(self, knobs, log, cycle_fn):
        self.k = knobs
        self.log = log
        self.now = cycle_fn
        self.cfg = ConfigSpace(knobs)
        self.mem = {}
        self.first_cfg = None
        self.sc_sent = False
        self.requests = []               # (cycle, parsed TLP, completion status)
        self.held = []                   # (release cycle, completion) for knob cpl_delay

    def completer_id(self):
        return TLP.bdf(self.cfg.bus, self.cfg.dev, 0)

    def handle(self, t):
        """A TLP the Data Link Layer accepted. Returns the completions to send."""
        name = t.get('name')
        out = []
        status = None
        if name in ('CfgRd0', 'CfgWr0', 'CfgRd1', 'CfgWr1'):
            now = self.now()
            if self.first_cfg is None:
                self.first_cfg = now
            wr = name.startswith('CfgWr')
            if name.endswith('1') or t['fn'] != 0:
                status = C.CPL_UR
            elif self.k.crs_cycles and not self.sc_sent and now - self.first_cfg < self.k.crs_cycles:
                status = C.CPL_CRS
            else:
                status = C.CPL_SC
                if wr:
                    self.cfg.write(t['reg'], t['first_be'], t['payload'][:4])
                    self.cfg.bus, self.cfg.dev = t['bus'], t['dev']     # §2.2.6.2
                data = self.cfg.read(t['reg']).to_bytes(4, 'little') if not wr else None
                self.sc_sent = True
            if status == C.CPL_SC and not wr:
                out.append(TLP.cpl(self.completer_id(), C.CPL_SC, t['req_id'], t['tag'], 4, 0, data=data))
            else:
                out.append(TLP.cpl(self.completer_id(), status, t['req_id'], t['tag'], 4, 0))
        elif name in ('MRd', 'MWr'):
            base, size = self.cfg.bar_base(), self.k.bar_size
            hit = self.cfg.mem_enabled() and base <= t['addr'] < base + size
            if name == 'MWr':
                if hit:
                    for i, b in enumerate(t['payload']):
                        self.mem[t['addr'] - base + i] = b
                status = 'posted'
            elif hit:
                n = t['length'] or 1024
                data = bytes(self.mem.get(t['addr'] - base + i, 0) for i in range(4 * n))
                out.append(TLP.cpl(self.completer_id(), C.CPL_SC, t['req_id'], t['tag'], (4 * n) & 0xFFF,
                                   t['addr'] & 0x7F, data=data))
                status = C.CPL_SC
            else:
                out.append(TLP.cpl(self.completer_id(), C.CPL_UR, t['req_id'], t['tag'], 4, 0))
                status = C.CPL_UR
        elif name in ('Msg', 'MsgD'):
            status = 'message'
        else:
            nonposted = name in ('IORd', 'IOWr')
            if nonposted:
                out.append(TLP.cpl(self.completer_id(), C.CPL_UR, t['req_id'], t['tag'], 4, 0))
            status = C.CPL_UR
        self.requests.append((self.now(), t, status))
        if self.k.cpl_delay_cycles:
            rel = self.now() + self.k.cpl_delay_cycles
            self.held.extend((rel, c) for c in out)
            return []
        return out

    def due(self):
        now = self.now()
        out = [c for r, c in self.held if r <= now]
        self.held = [(r, c) for r, c in self.held if r > now]
        return out
