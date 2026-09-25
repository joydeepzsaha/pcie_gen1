// ---------------------------------------------------------------------------
// pcie_cfg_txn -- ONE Configuration transaction, issue to outcome.
// Commit 2b-1; Stage D added the per-transaction Type 0/1 select (cmd_type1_i).
//
// SPEC ANCHORS
//   PCIe Base 2.1 SS2.3.2 p.122 .. Implementation Note naming the
//                                  device-existence semantics this module
//                                  classifies outcomes against, including CRS.
//   PCIe Base 2.1 Table 2-37 p.137 the MINIMUM initial Non-Posted Header credit
//                                  advertisement -- the reason a single
//                                  outstanding transaction is always issuable.
//   PG213 v1.3 Table 61 .......... the Configuration RQ descriptor this module
//                                  builds. Field placement is owned by
//                                  pcie_rq_rc_pkg; nothing is duplicated here.
//
//   cmd_* -> RQ descriptor -> pcie_rq_rc_top socket -> ... -> completion
//         -> classify -> rsp_*
//
// This module owns exactly one question: "I was asked for one config read or
// write; how did it end?" It builds the descriptor, drives it at the PG213
// socket, latches the tag the core actually put on the wire, matches the
// completion against that tag, runs the bounded CRS retry loop, and reports a
// single classified outcome. It sequences nothing and decides no policy.
//
// ===========================================================================
// SS WHY THIS IS A SEPARATE MODULE FROM THE ENUMERATION SEQUENCER
// ===========================================================================
//
// Because OUTCOME and POLICY are not the same thing, and the spec read that
// produced this design proved it.
//
// A UR completion means "no device at that BDF" during a Vendor-ID probe --
// PCIe Base 2.1 SS2.3.2 Implementation Note p.122 names the device-existence
// probe explicitly and makes an all-1s read (i.e. a UR) its answer. The SAME UR
// arriving on a later access to a device that has already answered is a fault.
// Identical wire event, opposite meaning, and the difference is context this
// module does not have and must not acquire.
//
// So: this module reports TXN_UR. The sequencer, which knows whether it is
// probing or configuring, decides what TXN_UR means. Folding the two together
// would put a "am I probing?" bit inside the transaction engine, which is the
// shape bugs live in.
//
// The split also keeps the standalone target honest forever: pcie_cfg_txn can
// be driven to every outcome by a bench with no enumeration in it at all.
//
// ===========================================================================
// SS SINGLE OUTSTANDING, BY CONSTRUCTION -- NOT AS A SIMPLIFICATION
// ===========================================================================
//
// One command is in flight at a time; cmd_ready_o is low from the moment a
// command is accepted until its response is consumed. This is not laziness
// about pipelining -- it is what the flow control permits.
//
// PCIe Base 2.1 Table 2-37 p.137 sets the MINIMUM initial Non-Posted Header
// advertisement at "1 unit - credit value of 01h". A compliant link partner may
// legally advertise exactly one NPH credit, and a configuration request
// consumes one NPH (tlp_pkg.sv:127-133 maps CFG0 to TLP_CREDIT_NON_POSTED;
// tlp_vc_buffer.sv:91 charges data credits only when the packet has data, so a
// CfgRd0 consumes NPH=1/NPD=0 and a CfgWr0 NPH=1/NPD=1).
//
// Tag availability is therefore NOT the binding constraint and must not be
// treated as one: with NPH=1 a second request cannot be transmitted no matter
// how many tags are free. Designing for one outstanding request makes the
// tag/completion correlation a single latched value instead of a scoreboard,
// and costs nothing that the credit floor was going to allow anyway.
//
// ===========================================================================
// SS THE TAG IS LATCHED FROM THE STROBE, NEVER FROM THE COMMAND HANDSHAKE
// ===========================================================================
//
// pcie_rq_tag_o is NOT valid when the descriptor is accepted. The Transaction
// Layer's requester leaves REQ_IDLE and only allocates in REQ_TAG a cycle or
// more later (tlp_requester.sv:247, 251-252), which is why the socket pairs the
// tag with its own one-cycle valid strobe instead of qualifying it with
// s_axis_rq_tready (pcie_rq_rc_top.sv:64-73).
//
// awaiting_tag_r is armed when the DESCRIPTOR beat is accepted -- that is when
// the command reaches the TL -- and the capture is written outside the state
// machine's case, because for a config WRITE the strobe can legitimately arrive
// while the payload beat is still being driven.
//
// A consequence worth stating: a socket that strobed the tag in the SAME cycle
// as the descriptor handshake would not be captured here, because
// awaiting_tag_r is still low in that cycle. That is correct -- the real socket
// cannot do it -- and it is the property mutation SM-1 exists to check.
//
// ===========================================================================
// SS WHAT THIS MODULE DOES NOT DO
// ===========================================================================
//
//  * NO LIVENESS TIMER. The only counter here is the CRS backoff, which PACES
//    retries; it never decides that something has taken too long. Completion
//    liveness belongs to tlp_request_tracker's completion timeout, which is the
//    single timer in the system for that job. s_axis_rq_tready may be low for
//    an arbitrary span (a real link partner advertises real credit and streams
//    UpdateFC continuously) and a completion may take arbitrarily long; neither
//    is an error here.
//
//  * NO tx_fc_blocked_o INPUT. It is a diagnostic, not a control signal. Making
//    it an input would be the same mistake as a watchdog.
//
//  * NO TAG MANAGEMENT. Tags are core-managed. On a completion timeout the tag
//    enters ZOMBIE quarantine upstream and is released by the tracker on a late
//    bit-30 completion or a second timeout interval. This module never
//    allocates, frees or reuses one.
//
//  * NO late_cpl / orphan-data PORTS. A drained late completion raises
//    late_cpl_valid_o upstream and, once per drained Dword, rc_protocol_error_o
//    with RC_ERR_ORPHAN_DATA (pcie_rc_if.sv:414-416). Neither is a fault and
//    neither reaches this module: the tracker drains the completion, so no RC
//    packet is produced. The requirement here is TRANSPARENCY -- an idle or
//    in-flight transaction must be unperturbed -- and transparency is proved by
//    the tests, not implemented by a counter.
//
// Guards use $warning, never $error: a procedural $error maps to $stop under
// the simulator, which would abort the shared multi-test process.
// ---------------------------------------------------------------------------
`timescale 1ns/1ps
module pcie_cfg_txn
  import pcie_rq_rc_pkg::*;
  import pcie_enum_pkg::*;
#(
    parameter int AXIS_DATA_WIDTH = 128,
    // PG213 tkeep is DWORD-granular on both RQ and RC: one bit per Dword.
    parameter int AXIS_KEEP_WIDTH = AXIS_DATA_WIDTH / 32,
    parameter int AXIS_USER_WIDTH = 60,

    // CRS retry policy. Both IMPLEMENTATION-DEFINED; see pcie_enum_pkg.
    parameter int unsigned CRS_RETRY_MAX      = CRS_RETRY_MAX_DEFAULT,
    parameter int unsigned CRS_BACKOFF_CYCLES = CRS_BACKOFF_CYCLES_DEFAULT,

    // NOT a timer. Present only so the elaboration-time P-CRS-BUDGET check
    // below can compare the retry budget against the completion timeout that
    // tlp_request_tracker is running. Set it to whatever the instantiating
    // design passes to pcie_rq_rc_top; 0 disables the check, matching the
    // tracker's "0 disables" convention.
    parameter int unsigned CPL_TIMEOUT_CYCLES = tlp_pkg::CPL_TIMEOUT_DEFAULT_CYCLES  // 63 #7g-2: 10 ms
) (
    input  logic                        clk_i,
    input  logic                        rst_i,

    // ---- command port ------------------------------------------------------
    // One transaction per handshake. dword_count is always 1 and Last DW BE is
    // always 0000b -- PCIe Base 2.1 SS2.2.7 p.79 requires both of every
    // Configuration Request, so neither is exposed as a choice.
    input  logic                        cmd_valid_i,
    output logic                        cmd_ready_o,
    input  logic                        cmd_write_i,
    // Stage D: Type 1 select, sampled with the command like every other cmd_*
    // field. This is a PER-TRANSACTION property, not phase knowledge -- the
    // primitive still does not know who is asking or why, exactly as it does
    // not know why cmd_reg_num_i is 0x06. Because it is latched once at accept
    // and the descriptor is a pure function of the latch, a CRS reissue
    // repeats the type by construction (docs/predictions/SPEC_PREDICTIONS_STAGE_D.md P6.3 --
    // the retry must not decay to Type 0).
    input  logic                        cmd_type1_i,
    // The target BDF. This becomes the descriptor's Completer ID, which is what
    // forms the routing Dword -- NOT the address (pcie_rq_if.sv:261-262,
    // tlp_generator.sv:81-82, PG213 Table 61 :3740).
    input  logic [15:0]                 cmd_bdf_i,
    input  logic [5:0]                  cmd_reg_num_i,
    input  logic [3:0]                  cmd_ext_reg_i,
    input  logic [3:0]                  cmd_first_be_i,
    input  logic [31:0]                 cmd_wdata_i,

    // ---- response port -----------------------------------------------------
    // rsp_valid_o is a HELD handshake, not a pulse: it stays asserted until
    // rsp_ready_i. A consumer that is busy cannot miss an outcome.
    output logic                        rsp_valid_o,
    input  logic                        rsp_ready_i,
    output txn_outcome_e                rsp_outcome_o,
    output logic [31:0]                 rsp_rdata_o,
    // The untranslated Completion Status off the wire, for the sequencer's
    // logs. Reserved encodings appear here verbatim even though they classify
    // as TXN_UR -- see the status decode below.
    output logic [2:0]                  rsp_status_raw_o,
    // CRS retries spent on the transaction being reported. Observability for
    // the sequencer's tests; 0 on a transaction that never saw CRS.
    output logic [$clog2(CRS_RETRY_MAX+1)-1:0] crs_retries_o,

    // ---- pcie_rq_rc_top socket: Requester Request --------------------------
    output logic [AXIS_DATA_WIDTH-1:0]  s_axis_rq_tdata_o,
    output logic [AXIS_KEEP_WIDTH-1:0]  s_axis_rq_tkeep_o,
    output logic                        s_axis_rq_tvalid_o,
    output logic                        s_axis_rq_tlast_o,
    output logic [AXIS_USER_WIDTH-1:0]  s_axis_rq_tuser_o,
    input  logic                        s_axis_rq_tready_i,

    // ---- pcie_rq_rc_top socket: core-managed tag ---------------------------
    input  logic [7:0]                  pcie_rq_tag_i,
    input  logic                        pcie_rq_tag_vld_i,

    // ---- pcie_rq_rc_top socket: Requester Completion -----------------------
    input  logic [AXIS_DATA_WIDTH-1:0]  m_axis_rc_tdata_i,
    input  logic [AXIS_KEEP_WIDTH-1:0]  m_axis_rc_tkeep_i,
    input  logic                        m_axis_rc_tvalid_i,
    input  logic                        m_axis_rc_tlast_i,
    output logic                        m_axis_rc_tready_o,

    // ---- pcie_rq_rc_top socket: completion timeout sideband ----------------
    input  logic                        cpl_timeout_valid_i,
    input  logic [7:0]                  cpl_timeout_tag_i
);

  localparam int CRS_CNT_W  = $clog2(CRS_RETRY_MAX + 1);
  localparam int BACKOFF_W  = $clog2(CRS_BACKOFF_CYCLES + 1);

  // -------------------------------------------------------------------------
  // Elaboration-time policy checks. $warning, never $error.
  // -------------------------------------------------------------------------
  initial begin
    if (CRS_RETRY_MAX < 1)
      $warning("pcie_cfg_txn: CRS_RETRY_MAX=%0d -- no CRS retry will ever be attempted, so a device that legally answers CRS is reported TXN_CRS_EXHAUSTED on its first completion.",
               CRS_RETRY_MAX);
    if (CRS_BACKOFF_CYCLES < 1)
      $warning("pcie_cfg_txn: CRS_BACKOFF_CYCLES=%0d -- retries are not paced and will be reissued back to back.",
               CRS_BACKOFF_CYCLES);
    // P-CRS-BUDGET, docs/predictions/SPEC_PREDICTIONS_ENUM.md SS5.2. If a full CRS retry storm can
    // outlast the completion timeout, a device that is merely slow to
    // initialise becomes indistinguishable from a dead one.
    if (CPL_TIMEOUT_CYCLES != 0 &&
        (CRS_RETRY_MAX * CRS_BACKOFF_CYCLES) >= CPL_TIMEOUT_CYCLES)
      $warning("pcie_cfg_txn: P-CRS-BUDGET violated -- CRS_RETRY_MAX*CRS_BACKOFF_CYCLES = %0d >= CPL_TIMEOUT_CYCLES = %0d. A slow-to-initialise device will time out mid-retry and be misreported as dead.",
               CRS_RETRY_MAX * CRS_BACKOFF_CYCLES, CPL_TIMEOUT_CYCLES);
  end

  // -------------------------------------------------------------------------
  // Latched command. Written once at cmd accept and NEVER touched again, which
  // is what makes a CRS reissue byte-identical to the original by construction
  // rather than by care: the descriptor below is a pure function of these.
  // -------------------------------------------------------------------------
  logic        write_r;
  logic        type1_r;
  logic [15:0] bdf_r;
  logic [5:0]  reg_num_r;
  logic [3:0]  ext_reg_r;
  logic [3:0]  first_be_r;
  logic [31:0] wdata_r;

  // -------------------------------------------------------------------------
  // Transaction state
  // -------------------------------------------------------------------------
  typedef enum logic [2:0] {
    S_IDLE,     // ready for a command
    S_DESC,     // driving the descriptor beat
    S_DATA,     // driving the payload beat (writes only)
    S_WAIT,     // awaiting the completion, or the timeout strobe
    S_BACKOFF,  // pacing a CRS retry
    S_RESP      // holding the outcome until it is consumed
  } txn_state_e;

  txn_state_e             state_r;
  logic [7:0]             tag_r;
  logic                   tag_valid_r;
  logic                   awaiting_tag_r;
  logic [CRS_CNT_W-1:0]   crs_count_r;
  logic [BACKOFF_W-1:0]   backoff_r;
  logic [31:0]            rdata_r;
  logic [2:0]             status_raw_r;
  txn_outcome_e           outcome_r;
  logic                   rc_in_packet_r;

  // -------------------------------------------------------------------------
  // RQ descriptor -- PG213 Table 61 (:3711,:3720,:3728,:3735), Configuration
  // form. Every field is derived from the latched command; the goldens for the
  // exact 128-bit values are in docs/predictions/SPEC_PREDICTIONS_ENUM.md SS3.4 and were written
  // before this module existed.
  //
  // Fields deliberately zero:
  //   tag [103:96]        IGNORED -- tags are core-managed
  //   requester_id [95:80] IGNORED -- the TL uses requester_id_i
  //   requester_id_en [120], force_ecrc [127], poisoned [79]
  //   tc [123:121], attr [126:124]  MUST be zero for a Configuration Request
  //                                 (Base 2.1 SS2.2.7 p.79)
  //   address [63:12]     Reserved in the Configuration form (Table 61 :3718)
  //   address [1:0]       Reserved (Table 61 :3715); the byte within the Dword
  //                       is selected by first_be, not by the address
  // -------------------------------------------------------------------------
  rq_descriptor_t desc;
  always_comb begin
    desc              = '0;
    desc.completer_id = bdf_r;                                 // the target BDF
    // Direction selects read/write; the latched Type 1 flag selects the pair.
    // These four encodings differ from their Type 0 partners in bit 0 only
    // (pcie_rq_rc_pkg.sv:63-79), mirroring Base 2.1 Table 2-3 p.58 on the wire.
    desc.req_type     = write_r ? (type1_r ? RQ_CFG_WRITE1 : RQ_CFG_WRITE0)
                                : (type1_r ? RQ_CFG_READ1  : RQ_CFG_READ0);
    desc.dword_count  = CFG_DWORD_COUNT;                       // always 1
    desc.address      = 64'd0;
    desc.address[11:8] = ext_reg_r;                            // Ext Reg Number
    desc.address[7:2]  = reg_num_r;                            // Register Number
  end

  wire driving_desc = (state_r == S_DESC);
  wire driving_data = (state_r == S_DATA);

  assign s_axis_rq_tdata_o  = driving_data ? {{(AXIS_DATA_WIDTH-32){1'b0}}, wdata_r}
                                           : AXIS_DATA_WIDTH'(desc);
  // Descriptor beat: all four Dwords. Payload beat: one Dword.
  assign s_axis_rq_tkeep_o  = driving_data ? AXIS_KEEP_WIDTH'(1) : '1;
  assign s_axis_rq_tvalid_o = driving_desc || driving_data;
  // A read is a single-beat packet; a write ends on its payload beat. Both
  // shapes are enforced upstream (pcie_rq_if.sv:307-309), so getting this wrong
  // is a rejection, not a silent malformation.
  assign s_axis_rq_tlast_o  = (driving_desc && !write_r) || driving_data;
  assign s_axis_rq_tuser_o  = {{(AXIS_USER_WIDTH-8){1'b0}}, CFG_LAST_BE, first_be_r};

  // -------------------------------------------------------------------------
  // Completion receive.
  //
  // tready is tied HIGH, permanently and on purpose. This is the only consumer
  // on the RC stream; back-pressuring it would stall the receive path for
  // completions this module is not even waiting for -- including the drained
  // remains of a late completion. The same reasoning ties CQ's ready high in
  // pcie_rq_rc_top.sv:100-105: discarding is survivable, wedging is not.
  //
  // Beat 0 carries the 3-Dword RC descriptor in Dwords 0..2 with the first
  // payload Dword in Dword 3 (pcie_rq_rc_pkg.sv:109-115). A configuration
  // completion is always a single beat -- Dword Count is 1 or 0 -- but later
  // beats are tracked and ignored anyway so that an oversized packet cannot
  // desynchronise the descriptor decode.
  // -------------------------------------------------------------------------
  assign m_axis_rc_tready_o = 1'b1;

  rc_descriptor_t rc_desc;
  assign rc_desc = rc_descriptor_t'(m_axis_rc_tdata_i[95:0]);
  wire [31:0] rc_payload_dw0 = m_axis_rc_tdata_i[127:96];

  wire rc_fire  = m_axis_rc_tvalid_i && m_axis_rc_tready_o;
  wire rc_beat0 = rc_fire && !rc_in_packet_r;

  // Match on the tag the core put on the wire, and complete on bit 30 -- the
  // last completion of the REQUEST, not the last beat of this completion
  // (PG213 :4049). For a configuration completion the two coincide, but taking
  // tlast would be right by accident and wrong the moment anything splits.
  wire rc_match = rc_beat0 && tag_valid_r && (rc_desc.tag == tag_r);
  wire rc_done  = rc_match && rc_desc.request_completed;

  // A completion timeout is only ours if it names the tag we are holding. The
  // tag_valid_r qualifier also covers the CRS window, where the previous
  // attempt's tag has already retired and a strobe cannot belong to us.
  wire timeout_match = cpl_timeout_valid_i && tag_valid_r &&
                       (cpl_timeout_tag_i == tag_r);

  // -------------------------------------------------------------------------
  // Completion Status decode.
  //
  // All EIGHT encodings are named. There is no `default`, deliberately: the
  // four legal values are only half the space, and Base 2.1 SS2.3.2 p.122 says
  // what the other half means -- "Completions with a Reserved Completion Status
  // value are treated as if the Completion Status was Unsupported Request
  // (UR)". A bare default over a 4-value enum would either invent a behaviour
  // the spec already specified, or leave the case incomplete.
  // -------------------------------------------------------------------------
  txn_outcome_e status_outcome;
  logic         status_is_crs;
  always_comb begin
    status_is_crs  = 1'b0;
    unique case (rc_desc.completion_status)
      3'b000: status_outcome = TXN_OK;                              // SC
      3'b001: status_outcome = TXN_UR;                              // UR
      3'b010: begin status_outcome = TXN_UR; status_is_crs = 1'b1; end // CRS
      3'b100: status_outcome = TXN_CA;                              // CA
      3'b011: status_outcome = TXN_UR;                              // Reserved
      3'b101: status_outcome = TXN_UR;                              // Reserved
      3'b110: status_outcome = TXN_UR;                              // Reserved
      3'b111: status_outcome = TXN_UR;                              // Reserved
    endcase
  end

  wire crs_budget_spent = (crs_count_r >= CRS_CNT_W'(CRS_RETRY_MAX));

  // -------------------------------------------------------------------------
  // Sequencing
  // -------------------------------------------------------------------------
  always_ff @(posedge clk_i) begin
    if (rst_i) begin
      state_r        <= S_IDLE;
      write_r        <= 1'b0;
      type1_r        <= 1'b0;
      bdf_r          <= '0;
      reg_num_r      <= '0;
      ext_reg_r      <= '0;
      first_be_r     <= '0;
      wdata_r        <= '0;
      tag_r          <= '0;
      tag_valid_r    <= 1'b0;
      awaiting_tag_r <= 1'b0;
      crs_count_r    <= '0;
      backoff_r      <= '0;
      rdata_r        <= '0;
      status_raw_r   <= '0;
      outcome_r      <= TXN_OK;
      rc_in_packet_r <= 1'b0;
    end else begin
      unique case (state_r)
        S_IDLE: begin
          if (cmd_valid_i) begin
            write_r      <= cmd_write_i;
            type1_r      <= cmd_type1_i;
            bdf_r        <= cmd_bdf_i;
            reg_num_r    <= cmd_reg_num_i;
            ext_reg_r    <= cmd_ext_reg_i;
            first_be_r   <= cmd_first_be_i;
            wdata_r      <= cmd_wdata_i;
            crs_count_r  <= '0;
            rdata_r      <= '0;
            status_raw_r <= '0;
            tag_valid_r  <= 1'b0;
            state_r      <= S_DESC;
          end
        end

        // The descriptor beat is where the command reaches the Transaction
        // Layer, so this is where the tag hunt is armed.
        S_DESC: begin
          if (s_axis_rq_tready_i) begin
            awaiting_tag_r <= 1'b1;
            state_r        <= write_r ? S_DATA : S_WAIT;
          end
        end

        S_DATA: begin
          if (s_axis_rq_tready_i) state_r <= S_WAIT;
        end

        S_WAIT: begin
          // Timeout first: if the request is dead, no completion is coming and
          // there is nothing to classify.
          if (timeout_match) begin
            outcome_r <= TXN_TIMEOUT;
            state_r   <= S_RESP;
          end else if (rc_done) begin
            status_raw_r <= rc_desc.completion_status;
            if (status_is_crs) begin
              if (crs_budget_spent) begin
                outcome_r <= TXN_CRS_EXHAUSTED;
                state_r   <= S_RESP;
              end else begin
                // A CRS TERMINATES the request (Base 2.1 SS2.3.2 IN p.113), so
                // the retry is a NEW request that will be given a NEW tag --
                // not a resumption. Drop the tag before reissuing.
                crs_count_r <= crs_count_r + CRS_CNT_W'(1);
                backoff_r   <= BACKOFF_W'(CRS_BACKOFF_CYCLES);
                tag_valid_r <= 1'b0;
                state_r     <= S_BACKOFF;
              end
            end else begin
              // Read data is meaningful only on a Successful Completion: every
              // other status arrives with no data at all (Base 2.1 SS2.3.2
              // p.122, "No data is included with the Completion").
              if ((status_outcome == TXN_OK) && !write_r) rdata_r <= rc_payload_dw0;
              outcome_r <= status_outcome;
              state_r   <= S_RESP;
            end
          end
        end

        S_BACKOFF: begin
          if (backoff_r == '0) state_r    <= S_DESC;
          else                 backoff_r  <= backoff_r - BACKOFF_W'(1);
        end

        S_RESP: begin
          if (rsp_ready_i) begin
            tag_valid_r <= 1'b0;
            state_r     <= S_IDLE;
          end
        end
      endcase

      // ---- tag capture, outside the case ---------------------------------
      // Written here rather than in a state arm because for a config WRITE the
      // strobe can arrive while the payload beat is still being driven. See the
      // header. In the descriptor-accept cycle awaiting_tag_r is still low, so
      // a same-cycle strobe is not captured -- which is correct, and is what
      // socket-model mutation SM-1 exercises.
      if (awaiting_tag_r && pcie_rq_tag_vld_i) begin
        tag_r          <= pcie_rq_tag_i;
        tag_valid_r    <= 1'b1;
        awaiting_tag_r <= 1'b0;
      end

      // ---- RC packet boundary tracking ------------------------------------
      if (rc_fire) rc_in_packet_r <= !m_axis_rc_tlast_i;
    end
  end

  assign cmd_ready_o      = (state_r == S_IDLE);
  assign rsp_valid_o      = (state_r == S_RESP);
  assign rsp_outcome_o    = outcome_r;
  assign rsp_rdata_o      = rdata_r;
  assign rsp_status_raw_o = status_raw_r;
  assign crs_retries_o    = crs_count_r;

endmodule
