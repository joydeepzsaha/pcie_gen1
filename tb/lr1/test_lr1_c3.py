"""test_lr1_c3 -- pcie_rc_top against the LR1 far end: the C3 provocation at a 256-byte payload

Author: Kourosh Ghahramani
Silicon Systems Research Lab, University of Washington

Purpose
    LR1-b's C3 row (RULINGS_LR1A 4; BRIEF_LR1 D9) needs a 256-byte TLP from
    the RC (PHASE0_LR1 §6), so it runs in a build of tb_lr1_rc_top with
    MAX_PAYLOAD_BYTES = 256 (target verilate_lr1_c3), against a far end that
    supports Max_Payload_Size 256 B. The harness is test_lr1_inject's.

References
    PCIe Base Spec r2.1, §3.5.2.2, §4.2.2, §7.8.3
"""
import cocotb

from lr1_inject import InjKnobs
from test_lr1_inject import run_row, rc_tlp, covers


@cocotb.test()
async def prov_c3(dut):
    """D9, C3 (§63 #19; HANDSHAKE §7, the axis_user_demux ST_IDLE row): DLLPs
    reaching the RC's receive demux one beat per clock, from a backlog. The
    far end advertises one posted header credit and Max_Payload_Size
    Supported 256 B; the RC runs at Max_Payload_Size 256 (this target's
    build). After enumeration the host writes L (256 B) and then M (one
    Dword): M waits at the RC's credit gate
    (tx_fc_blocked_o). While L is on the wire the far end sends, back to back
    (§4.2.2 p.196), three minimal TLPs (one-Dword Memory Reads of the RC, 5
    beats each) -- the RC's Ack for the first waits behind L, so the
    second's Ack cannot be queued, the RC's receive path stalls and the third
    stops mid-frame at its demux -- then three UpdateFC-NP (unchanged) and
    D4, an UpdateFC-P raising its posted header limit by one, which frees M.
    It holds its Acks 75 clocks (Ack(L) 50-100 clocks, PHASE0) and sends no
    other UpdateFC-P for 4,000 clocks. §3.5.2.2 p.179: "Receivers
    must process all DLLPs received at the rate they are received."

    Timing: the host-to-wire latency is measured on a one-Dword write first,
    and the burst is armed so the far end's first TLP reaches the RC 4 clocks
    after L's STP (PHASE0_LR1 §6 and PRE_HW_CHECKS C3 give the mechanism). A
    first development attempt with two TLPs left M unblocked before L ended:
    DLLPs are not held up by a stalled TLP path unless a TLP is stuck in the
    demux mid-frame, which is the third TLP's job. A second, with L at 128 B
    in the 128-byte build, left the RC's Acks unblocked: L's beats had all
    entered the RC's transmit arbiter before the first far-end TLP's Ack was
    asked for, which is why PHASE0 chose 256 B.
    Non-vacuity at the pins: the burst went out; M was blocked at the credit
    gate after L started; the RC's Ack for the far end's first TLP leaves
    after L's END (it waited behind L, so the receive path stalled and the
    DLLPs queued); and D4 reached the RC's pins before M blocked, so M's
    block shows D4 was still queued. The far end sends no UpdateFC-P of its
    own before the hold ends, so only D4 can release M.

    Asserts (red until C3's fix, axis_user_demux's ST_IDLE taking the input
    handshake from the skid buffer that stores the beat): all four
    DLLPs are processed, so tx_fc_blocked_o falls within 2,000 clocks of D4
    reaching the RC and before the hold ends; M then leaves the RC, the far
    end stores its four bytes, and the RC Naks none of the far end's TLPs
    after the burst."""
    import lr1_symbols as SYM
    import lr1_tlp as TLP
    from lr1_host import Host
    row = 'prov_c3'
    res = {}

    async def go(fe, rise):
        host = Host(dut, lambda: fe.cycle)
        host.idle()
        base = int(dut.bar_addr_o.value) & ((1 << 64) - 1)
        # calibration: one Dword, its STP time against the host's last beat
        seq_x = fe.chk.rc_seq_next
        hx = cocotb.start_soon(host.mwr(base + 0x200, bytes(4)))
        await fe.run(4000, until=lambda: rc_tlp(fe, seq_x) is not None and hx.done(), every=1)
        lat = rc_tlp(fe, seq_x)[0] // 2 - host.log[-1][0]
        res['lat'] = lat
        await fe.run(2000)                      # its credit returns (UpdateFC-P)
        seq_l = fe.chk.rc_seq_next
        res['seq_l'] = seq_l
        hl = cocotb.start_soon(host.mwr(base, bytes(i & 0xFF for i in range(256))))
        await fe.run(4000, until=lambda: hl.done(), every=1)
        h = host.log[-1][0]
        hm = cocotb.start_soon(host.mwr(base + 0x100, bytes([0xD0, 0xD1, 0xD2, 0xD3])))
        # arm the burst: the first TLP to reach the RC 4 clocks after L's STP (h + lat)
        arm = h + lat + 4 - 1
        await fe.run(max(0, arm - fe.cycle), every=1)
        rid = TLP.bdf(fe.tl.cfg.bus, fe.tl.cfg.dev, 0)
        t1, t2, t3 = (TLP.mem_req(False, rid, tag, 0x10000000 + 4 * tag, 1) for tag in range(3))
        ah, ad = fe.dll.alloc['P']
        d4 = SYM.updatefc('P', (ah + 1) % 256, (ad + 16) % 4096)
        nph, npd = fe.dll.alloc['NP']
        fe.k.pkt_gap = 0
        fe.dll.hold_updatefc_p = fe.cycle + 4000
        res['hold'] = fe.dll.hold_updatefc_p
        fe.dll.script = [('TLP', t1), ('TLP', t2), ('TLP', t3)] + \
            [('DLLP', 'UpdateFC-NP', SYM.updatefc('NP', nph, npd))] * 3 + [('DLLP', 'UpdateFC-P', d4)]
        res['armed'] = fe.cycle
        await fe.run(200, until=lambda: not fe.dll.script, every=1)
        fe.k.pkt_gap = 8
        res['script'] = list(fe.dll.script_sent)
        await fe.run(3000)
        res['m_done'] = hm.done()
    k = InjKnobs(adv_credits={'P': (1, 0x80), 'NP': (1, 1), 'CPL': (0, 0)}, ack_delay_cycles=75,
                 mps_supported=256, max_payload=256, rc_max_payload=256)
    fe = await run_row(dut, k, row, after=go, tail=500)
    seq_l = res['seq_l']
    L = rc_tlp(fe, seq_l)
    M = rc_tlp(fe, (seq_l + 1) % 4096)
    d4_built = [c for c, n in res['script'] if n == 'UpdateFC-P']
    t_tlps = [c for c, n in res['script'] if n.startswith('TLP-')]
    assert L is not None and len(d4_built) == 1 and len(t_tlps) == 3, 'the burst did not go out: %s' % res
    t_d4 = d4_built[0] + 1 + 4                   # its last Symbol reaches the RC (8 Symbols, 4 clocks)
    blk = fe.edges.at['tx_fc_blocked_o']
    rose = [c for c, v in blk if v == 1 and c >= L[0] // 2]
    fell = [c for c, v in blk if v == 0 and rose and c > rose[0]]
    fe_seq_t1 = int(next(n for c, n in res['script'] if n.startswith('TLP-'))[4:], 16)
    ack_t1 = [t for t, kd, s in fe.rc_acknaks() if kd == 'Ack' and covers(s, fe_seq_t1) and t >= 2 * res['armed']]
    naks = [(t, s) for t, kd, s in fe.rc_acknaks(after=2 * res['armed']) if kd == 'Nak']
    m_bytes = [fe.tl.mem.get(0x100 + i) for i in range(4)]
    detail = ('host-to-STP %d clocks; L (RC seq %03x) %s; burst armed at %d, built %s; D4 at the RC ~%d; '
              'tx_fc_blocked_o edges %s; RC Ack covering the first far-end TLP at %s; hold to %d; '
              'M %s; far-end bytes at +100h %s; RC Naks after the burst %s' % (
                  res['lat'], seq_l, L, res['armed'], res['script'], t_d4, blk[-6:], ack_t1[:1],
                  res['hold'], M, m_bytes, naks))
    dut._log.info('MEASURE|%s|%s' % (row, detail))
    # the provocation happened (non-vacuity, §22.82)
    assert rose, 'M was never blocked at the credit gate after L started: ' + detail
    assert ack_t1 and ack_t1[0] > L[1], 'the RC\'s Ack for the first far-end TLP did not wait behind L: ' + detail
    assert t_d4 < rose[0], 'D4 reached the RC after M blocked, so it never waited in the receive path: ' + detail
    # the claim: D4 was processed, and it alone released M
    assert fell and fell[0] - t_d4 <= 2000, 'tx_fc_blocked_o stayed high after D4: ' + detail
    assert fell[0] < res['hold'], 'M was released only once the far end could send its own UpdateFC-P: ' + detail
    assert M is not None and M[0] // 2 > fell[0], 'M did not leave the RC after the credit gate opened: ' + detail
    assert m_bytes == [0xD0, 0xD1, 0xD2, 0xD3], 'the far end did not store M\'s Dword: ' + detail
    assert not naks, 'the RC Nak\'d a far-end TLP after the burst: ' + detail
