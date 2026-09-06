// ---------------------------------------------------------------------------
// pcie_rq_rc_top -- the Requester surface of the Root Complex, whole.
// Commit 2a-iii; closes Commit 2a.
//
//   host RQ AXIS -> pcie_rq_if -> tlp_layer -> TX DLLP stream
//   RX DLLP stream -> tlp_layer -> pcie_rc_if -> host RC AXIS
//
// This module is WIRING. It instantiates pcie_rq_if (2a-i), tlp_layer (the
// Transaction Layer, untouched) and pcie_rc_if (2a-ii), and presents the
// PG213-shaped user interface -- s_axis_rq_* slave, m_axis_rc_* master, plus
// tag, error and status. There is no behaviour here on purpose: every decision
// about descriptors, byte enables, tags and completion matching belongs to one
// of the two wrappers, and a reader chasing one should go there, not here. If
// this file ever grows an always_ff, something has been put in the wrong place.
//
// SPEC ANCHORS
//   PG213 v1.3 ......... the s_axis_rq_* / m_axis_rc_* user interface shape.
//                        The descriptor rules themselves live in pcie_rq_if
//                        (Table 60/61) and pcie_rc_if (Table 65); nothing here
//                        decodes a descriptor field.
//   PCIe Base 2.1 SS2.6  flow control. Not implemented here either -- this
//                        module only EXPOSES link_up_i / transmit_enable_i /
//                        fc_initialized_i / fc_update_valid_i, because
//                        tlp_layer is silent without all four. See the block
//                        immediately below, which is the whole reason those
//                        four ports are on this boundary at all.
//
// ===========================================================================
// !! FLOW CONTROL AND LINK STATE -- READ THIS BEFORE DEBUGGING SILENCE
// ===========================================================================
//
// tlp_layer emits ZERO TLPs, and reports NO error, until ALL of the following
// hold:
//
//     link_up_i         == 1
//     transmit_enable_i == 1
//     fc_initialized_i  == 1
//     at least one fc_update_valid_i pulse has loaded NON-ZERO credits
//
// (tlp_layer.sv:280 and :475-479, tlp_credit_manager.sv:53-54, 66-83.)
//
// The failure mode is SILENT. With any of them missing the RQ interface still
// accepts descriptors and still asserts s_axis_rq_tready, the command still
// reaches the Transaction Layer, and then nothing comes out: no TLP on
// m_dllp_axis_*, and no pulse on any error output. It looks exactly like a
// broken wrapper. This exact omission was regression RC1.
//
// !! A TAG IS STILL ALLOCATED, AND pcie_rq_tag_o STILL STROBES.
//
// (This block previously claimed "no tag on pcie_rq_tag_o". That was wrong;
// corrected after Commit 2b-1 measured it -- see tb/rc/test_pcie_enum_txn_tlp.py,
// test i8, which pins the behaviour.)
//
// Tag allocation sits UPSTREAM of the credit gate. tlp_requester enters REQ_TAG
// as soon as the command is accepted and raises tag_request_valid_o there
// (tlp_requester.sv:176 raises it, :247 enters the state), referencing neither
// fc_initialized_i nor any credit signal. The gate is further down, at the
// VC-buffer-to-transmit boundary:
// vc_packet_ready = credit_request_ready && transmit_enable_i &&
// link_up_i (tlp_layer.sv:280). So the tag is handed out, the TLP is assembled,
// and only then does it park in the VC buffer with nothing to spend.
//
// CONSEQUENCE FOR A CLIENT: a tag strobe is NOT evidence that a request reached
// the link. Correlate on the completion or the timeout strobe, never on the tag
// alone.
//
// !! AND THE COMPLETION TIMER IS ALREADY RUNNING.
//
// tlp_request_tracker measures per-tag age from ALLOCATION (that module's
// header, :39). Combined with the above, a request held off by credit for
// longer than CPL_TIMEOUT_CYCLES TIMES OUT WITHOUT EVER HAVING BEEN
// TRANSMITTED, and reports as an ordinary completion timeout -- no TLP on the
// wire, no error output, indistinguishable from a dead device. Commit 2b-1's
// test i9 predicts and confirms it.
//
// That bounds what any client above this module can promise: continuous credit
// starvation beyond CPL_TIMEOUT_CYCLES cannot be ridden out, no matter how the
// client is written. The limit is here, not in the client. Raising
// CPL_TIMEOUT_CYCLES toward the 10 ms the spec recommends is what moves it, and
// that is Stage-H work (see the KNOWN_GAPS note below).
//
// tx_fc_blocked_o is the signal that distinguishes "blocked on credit" from
// "blocked on something else"; watch it first.
//
// Configuration requests are NON-POSTED. A CfgRd0 consumes NPH=1 and NPD=0 --
// it carries no data, and tlp_vc_buffer.sv:91 charges data credits only when
// the packet has a payload; a CfgWr0 consumes NPH=1 and NPD=1
// (tlp_pkg.sv:121-133). Completions consume CPLH and CPLD. A credit pool that
// is initialised but saturated at zero for the class being used is the same
// silence.
//
// !! ZERO IS NOT EMPTY. An advertisement of 00h/000h made AT FC INITIALISATION
// means INFINITE credit for that type, not none (PCIe Base 2.1 SS2.6.1 p.138 and
// footnote 33 p.137; tlp_credit_manager.sv:106-120 latches it at init). Starving
// a pool therefore requires a small FINITE advertisement that is never
// replenished. Advertising zero to "starve" a class does the opposite and
// produces a test that passes while proving nothing.
//
// These four are deliberately EXPOSED rather than tied off internally: the
// integrator (or Commit 2b) owns link bring-up, and the Data Link Layer's
// InitFC exchange is what produces the real credit values.
//
// ===========================================================================
// !! HOW A CLIENT CORRELATES A COMPLETION WITH ITS REQUEST: BY TAG
// ===========================================================================
//
// Use pcie_rq_tag_o / pcie_rq_tag_vld_o out, and the RC descriptor's Tag field
// [71:64] back. That tag is the one the request tracker allocated and the one
// that physically went out in the emitted header's DW1 (the 54b8a72 fix), so
// the comparison is against the wire, not against a wrapper's idea of it.
//
// The tag is NOT available at the moment the command is accepted -- the
// requester leaves REQ_IDLE and allocates in REQ_TAG a cycle or more later
// (tlp_requester.sv:247, 251-252) -- which is why it comes with its own valid
// strobe rather than qualified by s_axis_rq_tready. Strobes arrive in issue
// order, one per emitted non-posted TLP.
//
// Posted writes (RQ_MEM_WRITE) allocate nothing and never strobe. There is no
// completion for them either, so there is nothing to correlate.
//
// !! command_context IS NOT AVAILABLE AS A CLIENT CHANNEL.
//
// The Transaction Layer's context echo (command_context_i -> result_context_o)
// is INTERNALLY CONSUMED by pcie_rc_if. pcie_rq_if loads it with
// {mem_read_r, addr_r[11:0]} -- 13 of the 16 bits -- and pcie_rc_if reads it
// back to reconstruct the RC descriptor's Lower Address field, which is not
// otherwise derivable because the CPL header carries only the low 7 bits
// (pcie_rc_if.sv:251, 252). It is not exposed on this module's ports and must
// not be treated as a spare correlation channel.
//
// Bits [15:13] of the context word are unused and would be free if a future
// user-context field is ever wanted. Wiring them out would mean new ports on
// both wrappers; nothing needs it today, because the tag round-trips for real.
//
// ===========================================================================
// SS WHAT IS TIED OFF, AND WHY IT IS SAFE
// ===========================================================================
//
// The Completer surface -- CQ (target_*) and CC (completion_request_*) -- is
// the ENDPOINT side and is out of scope for Commit 2a. Both are tied off here
// rather than raised to the top level:
//
//   target_request_ready_i / target_data_ready_i are tied 1, not 0. A received
//   request the Root Complex does not answer is DISCARDED, but the receive
//   path never stalls. Tying them 0 would back-pressure the RX stream and
//   wedge the whole receive side -- including completions -- the first time
//   any request arrived. Discarding is wrong in the long run; wedging is worse
//   and harder to diagnose.
//
//   completion_request_valid_i / _data_valid_i / _data_last_i are tied 0: this
//   module originates no completions.
//
// Raising CQ/CC properly is the Completer commit's work, and doing it here
// would mean inventing an interface for it that commit would then have to
// change.
// ---------------------------------------------------------------------------
// SS KNOWN_GAPS (consolidated for the whole of Commit 2a: 2a-0/i/ii/iii)
// ---------------------------------------------------------------------------
//
// From this level (2a-iii):
//
//  * RESOLVED (post-2a-iii): COMPLETION TIMEOUT now exists. It lives in
//    tlp_request_tracker.sv (see that module's header for the policy and the
//    PCIe Base 2.1 SS2.8 / SS7.8.16 citations) and surfaces here as
//    cpl_timeout_valid_o / cpl_timeout_tag_o and late_cpl_valid_o /
//    late_cpl_tag_o. A timed-out tag is QUARANTINED, not recycled: it stops
//    being allocatable, silently drains any late completion, and returns to
//    the pool on that late completion's last CPL or after a second timeout
//    interval. outstanding_o counts quarantined tags.
//    Residual gap: CPL_TIMEOUT_CYCLES defaults to 4096 cycles, which is a
//    simulation convenience roughly two orders of magnitude below the 10 ms
//    the spec recommends. A real value, and the Device Control 2 register that
//    would program it (SS7.8.16 bits 3:0 and bit 4), are Stage-H work.
//    Also not built: a PG213-style SYNTHESIZED ERROR COMPLETION on m_axis_rc
//    for a timed-out request. A client learns of the failure from the strobe,
//    not from a descriptor. Deliberate -- see the tracker header.
//
//  * CQ/CC tied off -- see above.
//
//  * No `tlp_layer` config-space client. bus_number_i / device_number_i /
//    function_number_i / memory_enable_i / extended_tag_enable_i /
//    max_payload_bytes_i / max_read_bytes_i / rcb_128b_i are passed straight
//    through to the integrator. Nothing here reads a config register to
//    populate them.
//
// From 2a-ii (pcie_rc_if.sv:128-154):
//
//  * RC descriptor Error Code 0011 (RC_DESC_ERR_BAD_LENGTH) is UNREACHABLE by
//    construction. The tracker suppresses the result for a completion with no
//    data when data was expected, or with a byte count overrun, and raises
//    unexpected_completion_o + TLP_ERR_COMPLETION_OVERFLOW instead
//    (tlp_request_tracker.sv:127-135). No result means no RC packet. The
//    condition surfaces on rc_unexpected_completion_o /
//    rc_completion_error_code_o. A client must not wait for 0011.
//  * Split memory reads: Lower Address [11:7] is the FIRST completion's.
//    Configuration completions never split (Dword Count is always 1), so
//    enumeration is unaffected; a memory-read DMA consumer would need it.
//  * m_axis_rc_tuser not driven (per-byte enables, is_sof/is_eof, discontinue).
//  * Locked Read Completions (descriptor [29]) tied 0 -- no origination path.
//  * Byte Count Modified is parsed by the TL but has no RC descriptor field.
//
// From 2a-i (pcie_rq_if.sv):
//
//  * Type 1 configuration requests (CFG_READ1 / CFG_WRITE1) rejected -- no
//    tlp_cmd_e exists. Commit 3.
//  * Non-contiguous byte enables rejected -- tlp_first_be/tlp_last_be build
//    contiguous range masks only (tlp_pkg.sv:165-193).
//  * Zero-length reads rejected for uniformity, though the TL would accept one
//    for TLP_CMD_MEM_READ (tlp_requester.sv:193).
//  * Atomics, locked reads, messages, ATS rejected -- no command path.
//  * Poison origination: command_* has no poison input; poisoned non-config
//    writes are forwarded UNPOISONED (flagged, not dropped).
//  * ECRC: command_ecrc_enable is tied 0 -- the TL computes ECRC itself
//    (tlp_ecrc.sv); RQ descriptor bit [127] Force ECRC is ignored.
//
// From 2a-0 (pcie_axis_dw_downsize.sv / pcie_axis_dw_upsize.sv):
//
//  * The gearboxes register tready, costing throughput on a stream that
//    back-pressures every cycle; they are byte-granular on both sides and
//    descriptor-blind by design.
//
// Guards use $warning, never $error: a procedural $error maps to $stop under
// the simulator, which would abort the shared multi-test process.
// ---------------------------------------------------------------------------
`timescale 1ns/1ps
module pcie_rq_rc_top
  import tlp_pkg::*;
  import pcie_rq_rc_pkg::*;
#(
    parameter int AXIS_DATA_WIDTH = 128,
    // PG213 tkeep is DWORD-granular on both RQ and RC: one bit per Dword.
    parameter int AXIS_KEEP_WIDTH = AXIS_DATA_WIDTH / 32,
    parameter int AXIS_USER_WIDTH = 60,
    parameter int TL_DATA_WIDTH   = 32,
    parameter int TL_KEEP_WIDTH   = TL_DATA_WIDTH / 8,
    parameter int TL_USER_WIDTH   = 3,
    // PG213 Table 10 sizes m_axis_cq_tuser at 88 bits on a 128/256-bit
    // interface. Only first_be[3:0] and last_be[7:4] are driven -- the same
    // descriptor-layer scope cut pcie_rq_if and pcie_rc_if already made. The
    // full width is declared so a consumer written against PG213 binds without
    // a width mismatch. CC tuser is 33 bits (Table 62) and is not driven at
    // all; it carries only parity and discontinue, neither of which this
    // design produces.
    parameter int CQ_USER_WIDTH   = 88,
    parameter int CC_USER_WIDTH   = 33,
    // CQ descriptor [120:115], PG213 Table 52: the aperture of the matching
    // BAR in address bits. 12 == 4 KB, which is what tlp_layer's DEFAULT
    // BAR_MASK (0xffff_ffff_ffff_f000) gives.
    //
    // !! This module does NOT pass BAR_COUNT / BAR_BASE / BAR_MASK /
    // BAR_ENABLE to tlp_layer, so the Root Complex's BAR map has only ever
    // been that one 4 KB window at address 0 -- a configuration nothing has
    // ever varied (sec 22.43). Stage F-1 makes the BAR decode load-bearing for
    // the first time, because the CQ descriptor now reports it. Making the
    // BARs programmable, and deriving this aperture from BAR_MASK instead of
    // asserting it here, is a registered item.
    parameter logic [5:0] CQ_BAR_APERTURE = 6'd12,
    parameter int CONTEXT_WIDTH   = 16,
    parameter int TAG_COUNT       = 32,
    // Completion Timeout; 0 disables. See tlp_request_tracker.sv header.
    parameter int unsigned CPL_TIMEOUT_CYCLES = 32'd4096,
    // Byte order of TLP headers on the DLL streams (tlp_layer.sv:13). The
    // default 1'b0 keeps host Dword order, which is what every Dword-speaking
    // RC bench drives; a top that stacks this module on the real Data Link
    // Layer must pass 1'b1 so headers cross the seam in PCIe wire order.
    parameter bit PCIE_WIRE_ORDER = 1'b0
) (
    input  logic                        clk_i,
    input  logic                        rst_i,

    // ---- link state and flow control -- SEE THE HEADER ---------------------
    // Nothing is transmitted until all four of these are satisfied, and the
    // failure is silent. tx_fc_blocked_o is the diagnostic.
    input  logic                        link_up_i,
    input  logic                        transmit_enable_i,
    input  logic                        fc_initialized_i,
    input  logic                        fc_update_valid_i,
    input  logic [7:0]                  fc_ph_i,
    input  logic [11:0]                 fc_pd_i,
    input  logic [7:0]                  fc_nph_i,
    input  logic [11:0]                 fc_npd_i,
    input  logic [7:0]                  fc_cplh_i,
    input  logic [11:0]                 fc_cpld_i,

    // ---- identity and negotiated limits ------------------------------------
    // requester_id_i is the ID that goes into every originated request header
    // and the one a completion must carry back to match.
    input  logic [15:0]                 requester_id_i,
    input  logic [15:0]                 completer_id_i,
    input  logic [7:0]                  bus_number_i,
    input  logic [4:0]                  device_number_i,
    input  logic [2:0]                  function_number_i,
    input  logic                        memory_enable_i,
    input  logic                        extended_tag_enable_i,
    input  logic [12:0]                 max_payload_bytes_i,
    input  logic [12:0]                 max_read_bytes_i,
    input  logic                        rcb_128b_i,

    // ---- PG213 Requester Request AXI4-Stream slave -------------------------
    // Beat 0 is the 16-byte RQ descriptor (PG213 Table 60/61); beats 1..n are
    // payload. tuser[3:0] = first_be, tuser[7:4] = last_be, read on beat 0.
    input  logic [AXIS_DATA_WIDTH-1:0]  s_axis_rq_tdata,
    input  logic [AXIS_KEEP_WIDTH-1:0]  s_axis_rq_tkeep,
    input  logic                        s_axis_rq_tvalid,
    input  logic                        s_axis_rq_tlast,
    input  logic [AXIS_USER_WIDTH-1:0]  s_axis_rq_tuser,
    output logic                        s_axis_rq_tready,

    // ---- core-managed tag presentation -------------------------------------
    // The tag the tracker allocated and put on the wire. Correlate completions
    // with this, not with the descriptor's Tag field (which is ignored) and
    // not with context (which is internally consumed).
    output logic [7:0]                  pcie_rq_tag_o,
    output logic                        pcie_rq_tag_vld_o,

    // ---- PG213 Requester Completion AXI4-Stream master ---------------------
    // Beat 0 carries the 3-Dword RC descriptor (PG213 Table 65) in Dwords 0..2
    // and the first payload Dword in Dword 3; later beats are payload.
    output logic [AXIS_DATA_WIDTH-1:0]  m_axis_rc_tdata,
    output logic [AXIS_KEEP_WIDTH-1:0]  m_axis_rc_tkeep,
    output logic                        m_axis_rc_tvalid,
    output logic                        m_axis_rc_tlast,
    input  logic                        m_axis_rc_tready,

    // ---- PG213 Completer Request AXI4-Stream master (Stage F-1) ------------
    // Inbound Memory/IO/Config requests from a device, presented to the host.
    // Beat 0 carries the 4-Dword CQ descriptor (PG213 Table 52, p. 146);
    // later beats are payload. tuser[3:0] = first_be, tuser[7:4] = last_be
    // (PG213 Table 10), valid on beat 0.
    //
    // !! DECLARED, NOT YET DRIVEN. This commit adds the boundary only; the
    // ports read constants until the pcie_cq_if commit fills them. A netlist
    // with these ports and no producer behind them is the intended
    // intermediate state, not an oversight.
    output logic [AXIS_DATA_WIDTH-1:0]  m_axis_cq_tdata,
    output logic [AXIS_KEEP_WIDTH-1:0]  m_axis_cq_tkeep,
    output logic                        m_axis_cq_tvalid,
    output logic                        m_axis_cq_tlast,
    output logic [CQ_USER_WIDTH-1:0]    m_axis_cq_tuser,
    input  logic                        m_axis_cq_tready,

    // ---- PG213 Completer Completion AXI4-Stream slave (Stage F-1) ----------
    // The host's response to a completer request. Beat 0 carries the 3-Dword
    // CC descriptor (PG213 Table 58, p. 168-169); later beats are payload.
    input  logic [AXIS_DATA_WIDTH-1:0]  s_axis_cc_tdata,
    input  logic [AXIS_KEEP_WIDTH-1:0]  s_axis_cc_tkeep,
    input  logic                        s_axis_cc_tvalid,
    input  logic                        s_axis_cc_tlast,
    input  logic [CC_USER_WIDTH-1:0]    s_axis_cc_tuser,
    output logic                        s_axis_cc_tready,

    // ---- Data Link Layer streams -------------------------------------------
    input  logic [TL_DATA_WIDTH-1:0]    s_dllp_axis_tdata,
    input  logic [TL_KEEP_WIDTH-1:0]    s_dllp_axis_tkeep,
    input  logic                        s_dllp_axis_tvalid,
    input  logic                        s_dllp_axis_tlast,
    input  logic [TL_USER_WIDTH-1:0]    s_dllp_axis_tuser,
    output logic                        s_dllp_axis_tready,

    output logic [TL_DATA_WIDTH-1:0]    m_dllp_axis_tdata,
    output logic [TL_KEEP_WIDTH-1:0]    m_dllp_axis_tkeep,
    output logic                        m_dllp_axis_tvalid,
    output logic                        m_dllp_axis_tlast,
    output logic [TL_USER_WIDTH-1:0]    m_dllp_axis_tuser,
    input  logic                        m_dllp_axis_tready,

    // ---- RQ error surface (pcie_rq_if) -------------------------------------
    // One-cycle pulse; the code is valid in the same cycle and holds until the
    // next rejection. A rejected descriptor emits NO TLP.
    output logic                        rq_protocol_error_o,
    output rq_error_e                   rq_error_code_o,
    output logic                        rq_gearbox_error_o,

    // ---- RC error surface (pcie_rc_if) -------------------------------------
    // rc_unexpected_completion_o: the completion matched no outstanding tag, or
    // overran its byte count. NO RC packet accompanies it.
    // ---- CQ/CC error surface (Stage F-1) -----------------------------------
    // cq_dropped_o is THE ANTI-A4 PORT: a one-cycle pulse for an inbound
    // request the completer did not deliver to the host and did not answer.
    // Nothing inbound is ever silently discarded again. Declared here, driven
    // by pcie_cq_if from the commit that adds it.
    output logic                        cq_dropped_o,
    output logic [3:0]                  cq_error_code_o,
    output logic                        cq_gearbox_error_o,
    output logic                        cc_protocol_error_o,
    output logic [3:0]                  cc_error_code_o,
    output logic                        cc_gearbox_error_o,

    output logic                        rc_unexpected_completion_o,
    output tlp_error_e                  rc_completion_error_code_o,
    output logic                        rc_protocol_error_o,
    output rc_error_e                   rc_error_code_o,
    output logic                        rc_gearbox_error_o,

    // ---- Transaction Layer error and status surface ------------------------
    output logic                        command_error_valid_o,
    output tlp_error_e                  command_error_code_o,
    output logic                        malformed_o,
    output logic                        rx_error_valid_o,
    output tlp_error_e                  rx_error_code_o,
    output logic                        rx_ecrc_error_o,
    output logic                        tx_error_valid_o,
    output tlp_error_e                  tx_error_code_o,
    // Asserted while the transmitter is held up for credit. The first thing to
    // look at when nothing is being emitted.
    output logic                        tx_fc_blocked_o,
    output logic                        credit_error_o,
    output logic                        vc_overflow_o,
    // ---- Completion Timeout surface (tlp_request_tracker) ------------------
    // One-cycle strobes with the tag valid in the same cycle, correlated
    // against the earlier pcie_rq_tag_o / pcie_rq_tag_vld_o allocation strobe.
    // cpl_timeout_*: a non-posted request was never answered; its tag is now
    // quarantined and the request has FAILED. late_cpl_*: a completion arrived
    // for an already-timed-out tag and was drained -- no RC packet accompanies
    // it. Both exist for the Commit 2b enumeration FSM, which probes absent
    // devices constantly and cannot free a tag it did not allocate.
    output logic                        cpl_timeout_valid_o,
    output logic [7:0]                  cpl_timeout_tag_o,
    output logic                        late_cpl_valid_o,
    output logic [7:0]                  late_cpl_tag_o,

    // Non-posted requests currently holding a tag, INCLUDING tags quarantined
    // by a completion timeout -- a quarantined tag is still unallocatable.
    // Returns to 0 when every outstanding request has been answered or has
    // timed out and been released.
    output logic [$clog2(TAG_COUNT+1)-1:0] outstanding_o
);

  // -------------------------------------------------------------------------
  // pcie_rq_if <-> tlp_layer command port
  // -------------------------------------------------------------------------
  logic                     command_valid;
  logic                     command_ready;
  tlp_cmd_e                 command;
  logic [63:0]              command_address;
  logic [12:0]              command_byte_count;
  logic [2:0]               command_tc;
  logic [2:0]               command_attr;
  logic [CONTEXT_WIDTH-1:0] command_context;
  logic                     command_prefix_valid;
  logic [31:0]              command_prefix;
  logic                     command_ecrc_enable;
  logic [TL_DATA_WIDTH-1:0] command_data;
  logic [TL_KEEP_WIDTH-1:0] command_keep;
  logic                     command_data_valid;
  logic                     command_data_last;
  logic                     command_data_ready;

  // The core-managed tag, straight from the tracker.
  logic [7:0]               allocated_tag;
  logic                     allocated_tag_valid;

  // -------------------------------------------------------------------------
  // tlp_layer <-> pcie_rc_if received-completion surface
  //
  // received_completion_header is struct-typed and stays internal: the RC
  // descriptor on m_axis_rc_* is the interface, not the TL's header shape.
  // -------------------------------------------------------------------------
  logic                     received_completion_valid;
  logic                     received_completion_ready;
  tlp_header_t              received_completion_header;
  logic [TL_DATA_WIDTH-1:0] received_completion_data;
  logic [TL_KEEP_WIDTH-1:0] received_completion_keep;
  logic                     received_completion_data_valid;
  logic                     received_completion_data_last;
  logic                     received_completion_data_ready;

  logic                     result_valid;
  logic                     result_ready;
  logic [CONTEXT_WIDTH-1:0] result_context;
  logic [2:0]               result_status;
  logic                     result_last;
  logic                     unexpected_completion;
  tlp_error_e               completion_error_code;

  // CC tie-off: struct-typed input needs a named zero. (The CQ tie-off is
  // gone -- pcie_cq_if drives that side from this commit.)
  tlp_header_t              completion_request_header_tie;
  assign completion_request_header_tie = '0;

  // tlp_layer's target_bar_o width, from ITS BAR_COUNT default of 2. This
  // module does not override BAR_COUNT -- see CQ_BAR_APERTURE above.
  localparam int TL_BAR_INDEX_WIDTH = 1;

  // -------------------------------------------------------------------------
  // Stage F-1 commit 1 -- BOUNDARY ONLY, NO BEHAVIOUR.
  //
  // The CQ/CC ports exist from here on so that the wrapper's interface stops
  // changing in the commits that add behaviour. They are driven to constants
  // and the tlp_layer tie-offs below are untouched, so this commit is
  // functionally identical to its parent by construction: no inbound request
  // is delivered, none is answered, and A4's three expect_fail rows stay red.
  //
  // The constants are chosen so an attached host sees a correctly IDLE
  // interface rather than an undriven one -- tvalid low, and tready low so a
  // host that starts sending CC descriptors is back-pressured rather than
  // having them silently accepted and dropped. Accepting and dropping is the
  // exact mistake A4 is, and it is not worth re-committing for one commit.
  // -------------------------------------------------------------------------
  assign s_axis_cc_tready    = 1'b0;
  assign cc_protocol_error_o = 1'b0;
  assign cc_error_code_o     = '0;
  assign cc_gearbox_error_o  = 1'b0;

  // -------------------------------------------------------------------------
  // Completer Request: TL target request -> PG213 CQ AXI-Stream. Stage F-1.
  //
  // This instantiation is what closes the CQ half of sec 41.1 A4. The
  // target_request_ready_i / target_data_ready_i literals that used to sit in
  // the tlp_layer instantiation below -- the two 1'b1s that consumed and
  // discarded every inbound request -- are now driven by this module, which
  // either emits a CQ packet or raises cq_dropped_o with a reason.
  // -------------------------------------------------------------------------
  logic                     target_request_valid;
  logic                     target_request_ready;
  tlp_header_t              target_request_header;
  logic                     target_memory;
  logic                     target_config;
  logic                     target_config_type_one;
  logic                     target_read;
  logic                     target_write;
  logic                     target_unsupported;
  logic                     target_bar_hit;
  logic                     target_bar_overlap;
  logic [TL_BAR_INDEX_WIDTH-1:0] target_bar;
  logic [TL_DATA_WIDTH-1:0] target_data;
  logic [TL_KEEP_WIDTH-1:0] target_keep;
  logic                     target_data_valid;
  logic                     target_data_last;
  logic                     target_data_ready;

  cq_error_e                cq_error_code;
  assign cq_error_code_o = 4'(cq_error_code);

  pcie_cq_if #(
      .AXIS_DATA_WIDTH(AXIS_DATA_WIDTH),
      .AXIS_KEEP_WIDTH(AXIS_KEEP_WIDTH),
      .CQ_USER_WIDTH  (CQ_USER_WIDTH),
      .TL_DATA_WIDTH  (TL_DATA_WIDTH),
      .TL_KEEP_WIDTH  (TL_KEEP_WIDTH),
      .BAR_INDEX_WIDTH(TL_BAR_INDEX_WIDTH),
      .CQ_BAR_APERTURE(CQ_BAR_APERTURE)
  ) u_cq_if (
      .clk_i(clk_i),
      .rst_i(rst_i),

      .target_request_valid_i  (target_request_valid),
      .target_request_ready_o  (target_request_ready),
      .target_request_header_i (target_request_header),
      .target_memory_i         (target_memory),
      .target_config_i         (target_config),
      .target_config_type_one_i(target_config_type_one),
      .target_read_i           (target_read),
      .target_write_i          (target_write),
      .target_unsupported_i    (target_unsupported),
      .target_bar_hit_i        (target_bar_hit),
      .target_bar_overlap_i    (target_bar_overlap),
      .target_bar_i            (target_bar),

      .target_data_i      (target_data),
      .target_keep_i      (target_keep),
      .target_data_valid_i(target_data_valid),
      .target_data_last_i (target_data_last),
      .target_data_ready_o(target_data_ready),

      .m_axis_cq_tdata (m_axis_cq_tdata),
      .m_axis_cq_tkeep (m_axis_cq_tkeep),
      .m_axis_cq_tvalid(m_axis_cq_tvalid),
      .m_axis_cq_tlast (m_axis_cq_tlast),
      .m_axis_cq_tuser (m_axis_cq_tuser),
      .m_axis_cq_tready(m_axis_cq_tready),

      .cq_dropped_o      (cq_dropped_o),
      .cq_error_code_o   (cq_error_code),
      .cq_gearbox_error_o(cq_gearbox_error_o)
  );

  // -------------------------------------------------------------------------
  // Requester Request: PG213 AXI-Stream -> TL command port. Commit 2a-i.
  // -------------------------------------------------------------------------
  pcie_rq_if #(
      .AXIS_DATA_WIDTH(AXIS_DATA_WIDTH),
      .AXIS_KEEP_WIDTH(AXIS_KEEP_WIDTH),
      .AXIS_USER_WIDTH(AXIS_USER_WIDTH),
      .TL_DATA_WIDTH  (TL_DATA_WIDTH),
      .TL_KEEP_WIDTH  (TL_KEEP_WIDTH),
      .CONTEXT_WIDTH  (CONTEXT_WIDTH)
  ) u_rq_if (
      .clk_i(clk_i),
      .rst_i(rst_i),

      .s_axis_rq_tdata (s_axis_rq_tdata),
      .s_axis_rq_tkeep (s_axis_rq_tkeep),
      .s_axis_rq_tvalid(s_axis_rq_tvalid),
      .s_axis_rq_tlast (s_axis_rq_tlast),
      .s_axis_rq_tuser (s_axis_rq_tuser),
      .s_axis_rq_tready(s_axis_rq_tready),

      .allocated_tag_i      (allocated_tag),
      .allocated_tag_valid_i(allocated_tag_valid),
      .pcie_rq_tag_o        (pcie_rq_tag_o),
      .pcie_rq_tag_vld_o    (pcie_rq_tag_vld_o),

      .command_valid_o       (command_valid),
      .command_ready_i       (command_ready),
      .command_o             (command),
      .command_address_o     (command_address),
      .command_byte_count_o  (command_byte_count),
      .command_tc_o          (command_tc),
      .command_attr_o        (command_attr),
      .command_context_o     (command_context),
      .command_prefix_valid_o(command_prefix_valid),
      .command_prefix_o      (command_prefix),
      .command_ecrc_enable_o (command_ecrc_enable),

      .command_data_o      (command_data),
      .command_keep_o      (command_keep),
      .command_data_valid_o(command_data_valid),
      .command_data_last_o (command_data_last),
      .command_data_ready_i(command_data_ready),

      .rq_protocol_error_o(rq_protocol_error_o),
      .rq_error_code_o    (rq_error_code_o),
      .rq_gearbox_error_o (rq_gearbox_error_o)
  );

  // -------------------------------------------------------------------------
  // The Transaction Layer. NOT modified by Commit 2a -- instantiated as it is.
  // -------------------------------------------------------------------------
  tlp_layer #(
      .DATA_WIDTH   (TL_DATA_WIDTH),
      .KEEP_WIDTH   (TL_KEEP_WIDTH),
      .USER_WIDTH   (TL_USER_WIDTH),
      .TAG_COUNT    (TAG_COUNT),
      .CONTEXT_WIDTH(CONTEXT_WIDTH),
      .CPL_TIMEOUT_CYCLES(CPL_TIMEOUT_CYCLES),
      .PCIE_WIRE_ORDER(PCIE_WIRE_ORDER)
  ) u_tlp_layer (
      .clk_i(clk_i),
      .rst_i(rst_i),

      .link_up_i        (link_up_i),
      .transmit_enable_i(transmit_enable_i),
      .requester_id_i   (requester_id_i),
      .completer_id_i   (completer_id_i),
      .bus_number_i     (bus_number_i),
      .device_number_i  (device_number_i),
      .function_number_i(function_number_i),
      .memory_enable_i  (memory_enable_i),
      .extended_tag_enable_i(extended_tag_enable_i),
      .max_payload_bytes_i  (max_payload_bytes_i),
      .max_read_bytes_i     (max_read_bytes_i),
      .rcb_128b_i           (rcb_128b_i),
      .fc_initialized_i (fc_initialized_i),
      .fc_update_valid_i(fc_update_valid_i),
      .fc_ph_i  (fc_ph_i),   .fc_pd_i  (fc_pd_i),
      .fc_nph_i (fc_nph_i),  .fc_npd_i (fc_npd_i),
      .fc_cplh_i(fc_cplh_i), .fc_cpld_i(fc_cpld_i),

      .s_dllp_axis_tdata (s_dllp_axis_tdata),
      .s_dllp_axis_tkeep (s_dllp_axis_tkeep),
      .s_dllp_axis_tvalid(s_dllp_axis_tvalid),
      .s_dllp_axis_tlast (s_dllp_axis_tlast),
      .s_dllp_axis_tuser (s_dllp_axis_tuser),
      .s_dllp_axis_tready(s_dllp_axis_tready),

      .m_dllp_axis_tdata (m_dllp_axis_tdata),
      .m_dllp_axis_tkeep (m_dllp_axis_tkeep),
      .m_dllp_axis_tvalid(m_dllp_axis_tvalid),
      .m_dllp_axis_tlast (m_dllp_axis_tlast),
      .m_dllp_axis_tuser (m_dllp_axis_tuser),
      .m_dllp_axis_tready(m_dllp_axis_tready),

      .command_valid_i       (command_valid),
      .command_ready_o       (command_ready),
      .command_i             (command),
      .command_address_i     (command_address),
      .command_byte_count_i  (command_byte_count),
      .command_tc_i          (command_tc),
      .command_attr_i        (command_attr),
      .command_context_i     (command_context),
      .command_prefix_valid_i(command_prefix_valid),
      .command_prefix_i      (command_prefix),
      .command_ecrc_enable_i (command_ecrc_enable),
      .command_data_i        (command_data),
      .command_keep_i        (command_keep),
      .command_data_valid_i  (command_data_valid),
      .command_data_last_i   (command_data_last),
      .command_data_ready_o  (command_data_ready),
      .command_error_valid_o (command_error_valid_o),
      .command_error_code_o  (command_error_code_o),
      .allocated_tag_o       (allocated_tag),
      .allocated_tag_valid_o (allocated_tag_valid),

      // ---- CQ (Completer Request): driven by u_cq_if -- sec 41.1 A4 CLOSED --
      // The two 1'b1 literals that used to sit on target_request_ready_i and
      // target_data_ready_i ARE what A4 was: they satisfied the parser's
      // handshake every cycle, so an inbound request was consumed and
      // discarded with all nineteen outputs unconnected and no strobe. Both
      // are now driven by pcie_cq_if, which either emits a CQ packet or raises
      // cq_dropped_o with a reason code.
      //
      // Four outputs stay unconnected, deliberately and not by omission:
      //   target_request_class_o  -- the CQ descriptor carries Request Type
      //                              (PG213 Table 57), which u_cq_if builds
      //                              from the memory/config/read/write
      //                              decodes; the TL's own class enum is a
      //                              credit concept, not a descriptor field.
      //   target_config_hit_o     -- folded into target_unsupported_o, which
      //   target_config_offset_o     is what u_cq_if acts on. A Config
      //                              completer that needs the register offset
      //                              is a later rung.
      //   target_offset_o         -- PG213's descriptor carries the FULL
      //                              address plus a BAR Aperture telling the
      //                              client which bits to ignore, not a
      //                              pre-subtracted offset.
      .target_request_valid_o  (target_request_valid),
      .target_request_ready_i  (target_request_ready),
      .target_request_header_o (target_request_header),
      .target_request_class_o  (),
      .target_memory_o         (target_memory),
      .target_config_o         (target_config),
      .target_config_hit_o     (),
      .target_config_type_one_o(target_config_type_one),
      .target_config_offset_o  (),
      .target_read_o           (target_read),
      .target_write_o          (target_write),
      .target_unsupported_o    (target_unsupported),
      .target_bar_hit_o        (target_bar_hit),
      .target_bar_overlap_o    (target_bar_overlap),
      .target_bar_o            (target_bar),
      .target_offset_o         (),
      .target_data_o           (target_data),
      .target_keep_o           (target_keep),
      .target_data_valid_o     (target_data_valid),
      .target_data_last_o      (target_data_last),
      .target_data_ready_i     (target_data_ready),

      // ---- CC (Completer Completion): out of scope, originates nothing -----
      .completion_request_valid_i        (1'b0),
      .completion_request_ready_o        (),
      .completion_request_header_i       (completion_request_header_tie),
      .completion_request_status_i       ('0),
      .completion_request_byte_count_i   ('0),
      .completion_request_lower_address_i('0),
      .completion_request_ecrc_enable_i  (1'b0),
      .completion_request_data_i         ('0),
      .completion_request_keep_i         ('0),
      .completion_request_data_valid_i   (1'b0),
      .completion_request_data_last_i    (1'b0),
      .completion_request_data_ready_o   (),

      .received_completion_valid_o     (received_completion_valid),
      .received_completion_ready_i     (received_completion_ready),
      .received_completion_header_o    (received_completion_header),
      .received_completion_data_o      (received_completion_data),
      .received_completion_keep_o      (received_completion_keep),
      .received_completion_data_valid_o(received_completion_data_valid),
      .received_completion_data_last_o (received_completion_data_last),
      .received_completion_data_ready_i(received_completion_data_ready),

      .result_valid_o  (result_valid),
      .result_ready_i  (result_ready),
      .result_context_o(result_context),
      .result_status_o (result_status),
      .result_last_o   (result_last),

      .malformed_o            (malformed_o),
      .rx_error_valid_o       (rx_error_valid_o),
      .rx_error_code_o        (rx_error_code_o),
      .rx_ecrc_error_o        (rx_ecrc_error_o),
      .tx_error_valid_o       (tx_error_valid_o),
      .tx_error_code_o        (tx_error_code_o),
      .tx_fc_blocked_o        (tx_fc_blocked_o),
      .credit_error_o         (credit_error_o),
      .vc_overflow_o          (vc_overflow_o),
      .unexpected_completion_o(unexpected_completion),
      .completion_error_code_o(completion_error_code),
      .cpl_timeout_valid_o    (cpl_timeout_valid_o),
      .cpl_timeout_tag_o      (cpl_timeout_tag_o),
      .late_cpl_valid_o       (late_cpl_valid_o),
      .late_cpl_tag_o         (late_cpl_tag_o),
      .outstanding_o          (outstanding_o)
  );

  // -------------------------------------------------------------------------
  // Requester Completion: TL received completion -> PG213 AXI-Stream.
  // Commit 2a-ii.
  // -------------------------------------------------------------------------
  pcie_rc_if #(
      .AXIS_DATA_WIDTH(AXIS_DATA_WIDTH),
      .AXIS_KEEP_WIDTH(AXIS_KEEP_WIDTH),
      .TL_DATA_WIDTH  (TL_DATA_WIDTH),
      .TL_KEEP_WIDTH  (TL_KEEP_WIDTH),
      .CONTEXT_WIDTH  (CONTEXT_WIDTH)
  ) u_rc_if (
      .clk_i(clk_i),
      .rst_i(rst_i),

      .received_completion_valid_i (received_completion_valid),
      .received_completion_ready_o (received_completion_ready),
      .received_completion_header_i(received_completion_header),

      .received_completion_data_i      (received_completion_data),
      .received_completion_keep_i      (received_completion_keep),
      .received_completion_data_valid_i(received_completion_data_valid),
      .received_completion_data_last_i (received_completion_data_last),
      .received_completion_data_ready_o(received_completion_data_ready),

      .result_valid_i         (result_valid),
      .result_ready_o         (result_ready),
      .result_context_i       (result_context),
      .result_status_i        (result_status),
      .result_last_i          (result_last),
      .unexpected_completion_i(unexpected_completion),
      .completion_error_code_i(completion_error_code),

      .m_axis_rc_tdata (m_axis_rc_tdata),
      .m_axis_rc_tkeep (m_axis_rc_tkeep),
      .m_axis_rc_tvalid(m_axis_rc_tvalid),
      .m_axis_rc_tlast (m_axis_rc_tlast),
      .m_axis_rc_tready(m_axis_rc_tready),

      .rc_unexpected_completion_o(rc_unexpected_completion_o),
      .rc_completion_error_code_o(rc_completion_error_code_o),
      .rc_protocol_error_o       (rc_protocol_error_o),
      .rc_error_code_o           (rc_error_code_o),
      .rc_gearbox_error_o        (rc_gearbox_error_o)
  );

endmodule
