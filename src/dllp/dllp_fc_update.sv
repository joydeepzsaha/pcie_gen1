// import pcie_datalink_pkg::*;
module dllp_fc_update
  import pcie_datalink_pkg::*;
#(
    // sec 63 #7g-2 D-7G.2: the link-clock PERIOD, the one source every
    // cycle-count timer derives from.  Was `CLK_RATE = 100` (MHz), which no
    // instantiator ever passed: every DLL in the gate elaborated 10 ns while
    // the design runs at 8 ns (pcie_docs FINDINGS_7G2_PHASE1.md sec 1.1).
    // Plumbed from pcie_datalink_layer; default 8 = the 125 MHz Gen1 PCLK.
    parameter int CLK_PERIOD_NS    = 8,
    // TLP data width
    parameter int DATA_WIDTH       = 32,
    // TLP strobe width
    parameter int STRB_WIDTH       = DATA_WIDTH / 8,
    parameter int KEEP_WIDTH       = STRB_WIDTH,
    parameter int USER_WIDTH       = 3,
    parameter int MAX_PAYLOAD_SIZE = 256
) (
    input  logic                   clk_i,                     // Clock signal
    input  logic                   rst_i,                     // Reset signal
    //link status
    input  pcie_dl_status_e        link_status_i,
    //flow control signals
    input  logic                   start_flow_control_i,
    output logic                   start_flow_control_ack_o,
    input  logic            [15:0] next_transmit_seq_i,
    input  logic                   tlp_nullified_i,
    // CREDITS_ALLOCATED from dllp2tlp -- the HdrFC/DataFC the UpdateFC carries
    // (Base 2.1 sec 2.6.1.2 p.141).  Was *_credits_consumed_i before sec 63 #7f
    // commit A; the value is the same register, now stepped at release.
    input  logic            [ 7:0] ph_credits_allocated_i,
    input  logic            [11:0] pd_credits_allocated_i,
    input  logic            [ 7:0] nph_credits_allocated_i,
    input  logic            [11:0] npd_credits_allocated_i,

    /*
     * DLLP UPDATE AXI output
     */
    output logic [(DATA_WIDTH)-1:0] m_axis_tdata,
    output logic [(KEEP_WIDTH)-1:0] m_axis_tkeep,
    output logic                    m_axis_tvalid,
    output logic                    m_axis_tlast,
    output logic [  USER_WIDTH-1:0] m_axis_tuser,
    input  logic                    m_axis_tready
);

  // localparam int PdMinCredits = MAX_PAYLOAD_SIZE >> 4;

  // localparam int PdMinCredits = ((8 << (5 + MAX_PAYLOAD_SIZE)) / 4 / 4);
  // localparam int HdrMinCredits = 8'h040;
  localparam int ClockPeriodNs = CLK_PERIOD_NS;
  // sec 63 #7g-2 (D-7G.3): Base 2.1 sec 2.6.1.2 p.143 -- "Update FCPs for each
  // enabled type of non-infinite FC credit must be scheduled for transmission
  // at least once every 30 us (-0%/+50%)".  30 us / period = 3,750 cycles at
  // 8 ns; the UpdateFC leaves FcWaitPeriod + 2 cycles after the last one of its
  // type (ST_IDLE sees the limit, ST_UPDATE_* is accepted the next cycle), i.e.
  // 30.016 us, inside [30, 45] us.  Was 2 ms (TwoMsTimeOut): 250,000 cycles,
  // 53x the nominal (pcie_docs FINDINGS_7G2_PHASE1.md row 1).
  localparam int FcWaitPeriod = 30_000 / ClockPeriodNs;
  localparam int TimerWidth = $clog2(FcWaitPeriod + 1);

  typedef enum logic [4:0] {
    ST_IDLE,
    ST_SEND_ACK,
    ST_SEND_ACK_CRC,
    ST_UPDATE_P,
    ST_UPDATE_CRC,
    ST_UPDATE_NP,
    ST_UPDATE_NP_CRC,
    ST_UPDATE_CPL,
    ST_UPDATE_CPL_CRC,
    ST_WAIT_LOW
  } fc_update_state_e;


  //axis registered output signals
  logic             [DATA_WIDTH-1:0] fc_axis_tdata;
  logic             [KEEP_WIDTH-1:0] fc_axis_tkeep;
  logic                              fc_axis_tvalid;
  logic                              fc_axis_tlast;
  logic             [USER_WIDTH-1:0] fc_axis_tuser;
  logic                              fc_axis_tready;
  // Internal state machine for link flow control
  (* syn_keep = "true", mark_debug = "true" *) fc_update_state_e                  curr_state;
  fc_update_state_e                  next_state;
  dllp_fc_t                          dll_packet_c;
  dllp_fc_t                          dll_packet_r;
  logic             [          15:0] dllp_lcrc_c;
  logic             [          15:0] dllp_lcrc_r;
  logic             [TimerWidth-1:0] timer_p_c, timer_p_r, timer_np_c, timer_np_r;
  logic             [          15:0] crc_out;
  logic             [          15:0] crc_reversed;
  logic                              start_ack_c;
  logic                              start_ack_r;
  logic             [          11:0] ack_nak_seq_c;
  logic             [          11:0] ack_nak_seq_r;
  logic                              ack_nak_is_nak_c;
  logic                              ack_nak_is_nak_r;
  logic             [DATA_WIDTH-1:0] ack_nak_payload;

  // ===========================================================================
  // RELEASE-TRIGGERED UpdateFC -- sec 63 #7f, #18 commit B.
  // Base 2.1 sec 2.6.1.2 p.142:
  //
  //   "For non-infinite NPH, NPD, PH, and CPLH types, an UpdateFC FCP must be
  //    scheduled for Transmission each time the following events occur:
  //     - all advertised FC units for a particular type are consumed by TLPs
  //       received
  //     - one or more units of that type are made available by TLPs processed"
  //
  // "Made available" is the CREDITS_ALLOCATED step commit A moved to the FIFO
  // release point in dllp2tlp.  An UpdateFC is OWED for a type whenever the
  // allocated pair for that type differs from the pair this module last put
  // on the wire for it -- *_pending below is exactly that comparison, so
  // releases that land while a DLLP is in flight coalesce into the next one
  // and nothing is owed at rest.  The first bullet is covered by the same
  // comparison: consumption at the peer cannot change what we advertise, and
  // our advertisement only ever changes on a release.
  //
  // !! THE LAST-ADVERTISED REGISTERS RESET TO THE InitFC CONSTANTS, NOT ZERO.
  // pcie_flow_ctrl_init advertises HdrMinCredits/PdMinCredits in InitFC1/2 and
  // in its post-init UpdateFC pair; dllp2tlp resets CREDITS_ALLOCATED to the
  // same constants.  Resetting *_last_r to them makes "nothing owed" true at
  // FC-init completion by construction, so no redundant pair is sent the
  // instant DL_ACTIVE is entered.  Three sites, two package constants: if one
  // moves, all three must.
  //
  // P and NP are scheduled INDEPENDENTLY: an NP release does not send a
  // redundant UpdateFC-P.  (The periodic path below was rewritten at sec 63
  // #7g-2; the paragraph that described it here is superseded by that block.)
  //
  // Measured BEFORE this commit (tb/fullstack rows W2/W3, tree aeeb739 + A):
  // the only UpdateFC-NP on the wire were pcie_flow_ctrl_init's post-init
  // pair, both HdrFC=16, before any TLP; the RC's CREDIT_LIMIT never moved,
  // its 17th non-posted request starved behind the credit gate, timed out
  // from allocation, and err_credit_blocked_o rose (F18, bar_count stalled at
  // 2).  Never B before A: with the accept-time step B would advertise buffer
  // space still occupied.
  // ===========================================================================
  logic             [           7:0] ph_last_c, ph_last_r, nph_last_c, nph_last_r;
  logic             [          11:0] pd_last_c, pd_last_r, npd_last_c, npd_last_r;
  logic                              p_pending, np_pending;

  assign p_pending  = (ph_credits_allocated_i  != ph_last_r)  ||
                      (pd_credits_allocated_i  != pd_last_r);
  assign np_pending = (nph_credits_allocated_i != nph_last_r) ||
                      (npd_credits_allocated_i != npd_last_r);

  // ===========================================================================
  // PERIODIC UpdateFC -- sec 63 #7g-2, Kourosh Q2 (2026-09-24): "one timer per
  // credit type, reset only by its own UpdateFC".  Base 2.1 sec 2.6.1.2 p.143
  // binds EACH type; Cpl is advertised infinite (F-2) so P and NP are the two.
  //
  // Measured BEFORE this commit (FINDINGS_7G2_PHASE1.md row 1b): ONE shared
  // timer, counting only in ST_IDLE, cleared by EVERY Ack request and by a
  // single-type release.  The RC's first periodic UpdateFC came exactly
  // +200,005 cycles after its last Ack, and it sent ZERO UpdateFCs during
  // enumeration -- so under traffic the timer was not a period at all, whatever
  // its value, and a stack whose releases were all one type never refreshed
  // the other.  Hence: two timers, and the Ack touches neither.
  //
  //   - each counts EVERY cycle of DL_Active, whatever the FSM is doing, and
  //     saturates at FcWaitPeriod; outside DL_Active it is held at 0, so
  //     pcie_flow_ctrl_init's post-init pair is the last refresh at entry;
  //   - each restarts ONLY on the handshake of its own type's UpdateFC beat
  //     (ST_UPDATE_P / ST_UPDATE_NP), periodic or release-triggered alike;
  //   - a type is OWED when its release is pending or its timer has expired.
  //
  // ST_IDLE priority stays Ack/Nak > owed UpdateFC, which is the spec's own
  // order (sec 3.5.2.1 Implementation Note pp.178-179: Nak 2, Ack 3, FC DLLPs
  // 4) and what verify_dllp_arbitration_priority asserts.  It cannot starve an
  // owed UpdateFC: dllp2tlp accepts no new packet while this module's Ack
  // handshake is up (dllp2tlp.sv ST_IDLE: skid_axis_tready gated on
  // !start_flow_control_ack_i), and a link TLP is >= 5 beats long, so after
  // every Ack ST_IDLE sees start_flow_control_i low for several cycles.
  // tb/dllp R-U3 measures this under a back-to-back Acked stream.
  // ===========================================================================
  logic dl_active, p_expired, np_expired, p_owed, np_owed;

  assign dl_active  = (link_status_i == DL_ACTIVE);
  assign p_expired  = (timer_p_r  >= FcWaitPeriod);
  assign np_expired = (timer_np_r >= FcWaitPeriod);
  assign p_owed     = p_pending  || p_expired;
  assign np_owed    = np_pending || np_expired;

  //crc byteswap
  always_comb begin : byteswap
    crc_reversed[7:0]  = ~dllp_lcrc_r[7:0];
    crc_reversed[15:8] = ~dllp_lcrc_r[15:8];
    // ⛔ DO NOT UNCOMMENT.  §63 #7i measured this, and the live line is CORRECT.
    // Conformance #5 ("the DLLP CRC is not bit-reversed at either end") was
    // REFUTED, not fixed: 70/70 captured DLLP frames are spec-correct against
    // Base 2.1 Table 3-2 p.167, checked by a Python model that also reproduces
    // 5,177/5,177 TLP LCRCs, so it is not a model that agrees with everything.
    // The complement-only form below ALREADY COMPOSES to the spec's per-byte
    // reversal once the byte order of the assembled field is accounted for.
    // Applying the table literally is a MEASURED REGRESSION on the LCRC side:
    // mutant MU2 does exactly that and kills three rows of
    // verilate_dll_comprehensive at the same sim times as forcing the compare
    // false.  §6 UNCOMMENT-ME trap: a commented line beside a suspected defect
    // is weak evidence the live line is wrong, NOT that the comment is the fix.
    // Evidence: pcie_docs evidence/fullstack/FINDINGS_7I_PHASE1.md; tracker §65.1
    // (struck at §63 #7g-1).
    // for (int i = 0; i < 8; i++) begin
    //   crc_reversed[i]   = dllp_lcrc_r[7-i];
    //   crc_reversed[i+8] = dllp_lcrc_r[15-i];
    // end
  end

  // Initialize to idle state
  always_ff @(posedge clk_i) begin : main_seq
    if (rst_i) begin
      curr_state <= ST_IDLE;
      dll_packet_r <= '0;
      timer_p_r <= '0;
      timer_np_r <= '0;
      dllp_lcrc_r <= '0;
      start_ack_r <= '0;
      ack_nak_seq_r <= '0;
      ack_nak_is_nak_r <= '0;
      ph_last_r <= HdrMinCredits;
      pd_last_r <= PdMinCredits;
      nph_last_r <= HdrMinCredits;
      npd_last_r <= PdMinCredits;
    end else begin
      curr_state <= next_state;
      dll_packet_r <= dll_packet_c;
      timer_p_r <= timer_p_c;
      timer_np_r <= timer_np_c;
      dllp_lcrc_r <= dllp_lcrc_c;
      start_ack_r <= start_ack_c;
      ack_nak_seq_r <= ack_nak_seq_c;
      ack_nak_is_nak_r <= ack_nak_is_nak_c;
      ph_last_r <= ph_last_c;
      pd_last_r <= pd_last_c;
      nph_last_r <= nph_last_c;
      npd_last_r <= npd_last_c;
    end
  end

  always_comb begin : ack_nak_payload_pack
    ack_nak_payload        = '0;
    ack_nak_payload[7:0]   = ack_nak_is_nak_r ? Nak : Ack;
    ack_nak_payload[15:8]  = 8'h00;
    ack_nak_payload[19:16] = ack_nak_seq_r[11:8];
    ack_nak_payload[23:20] = 4'h0;
    ack_nak_payload[31:24] = ack_nak_seq_r[7:0];
  end


  always_comb begin : combo_block
    next_state     = curr_state;
    dll_packet_c   = dll_packet_r;
    // both timers advance every DL_Active cycle, saturating; held at 0 outside it
    timer_p_c      = !dl_active ? '0 :
                     (p_expired  ? TimerWidth'(FcWaitPeriod) : timer_p_r  + 1'b1);
    timer_np_c     = !dl_active ? '0 :
                     (np_expired ? TimerWidth'(FcWaitPeriod) : timer_np_r + 1'b1);
    start_ack_c    = '0;
    ack_nak_seq_c = ack_nak_seq_r;
    ack_nak_is_nak_c = ack_nak_is_nak_r;
    ph_last_c      = ph_last_r;
    pd_last_c      = pd_last_r;
    nph_last_c     = nph_last_r;
    npd_last_c     = npd_last_r;
    //axis flow control defaults
    fc_axis_tdata  = '0;
    fc_axis_tkeep  = '0;
    fc_axis_tvalid = '0;
    fc_axis_tlast  = '0;
    fc_axis_tuser  = 4'h01;
    //crc signals
    dllp_lcrc_c    = dllp_lcrc_r;
    case (curr_state)
      ST_IDLE: begin
        if (start_flow_control_i) begin
          // sec 63 #7g-2: the Ack no longer clears any UpdateFC timer (Q2).
          next_state       = ST_SEND_ACK;
          ack_nak_seq_c    = next_transmit_seq_i[11:0];
          ack_nak_is_nak_c = tlp_nullified_i;
        end else if (dl_active && (p_owed || np_owed)) begin
          // p.142's release clause (credit made available and not yet
          // advertised) or p.143's periodic one (the type's timer expired).
          // Only the type(s) owed are sent; P first when both are.
          next_state = p_owed ? ST_UPDATE_P : ST_UPDATE_NP;
        end
      end
      ST_SEND_ACK: begin
        //build axis master output
        fc_axis_tdata  = ack_nak_payload;
        dllp_lcrc_c    = crc_out;
        fc_axis_tkeep  = '1;
        fc_axis_tvalid = '1;
        if (fc_axis_tready) begin
          next_state = ST_SEND_ACK_CRC;
        end
      end
      ST_SEND_ACK_CRC: begin
        //build axis master output
        fc_axis_tdata  = crc_reversed;
        fc_axis_tkeep  = 8'h3;
        fc_axis_tvalid = '1;
        fc_axis_tlast  = '1;
        if (fc_axis_tready) begin
          // ACK and NAK complete identically.  Periodic UpdateFC traffic is a
          // separate transaction and must not acknowledge this request.
          start_ack_c = '1;
          next_state  = ST_WAIT_LOW;
        end
      end
      ST_UPDATE_P: begin
        //build dllp fc update for crc
        //build axis master output
        fc_axis_tdata =
            send_fc_init(UpdateFC_P, '0, ph_credits_allocated_i, pd_credits_allocated_i);
        dllp_lcrc_c = crc_out;
        fc_axis_tkeep = '1;
        fc_axis_tvalid = '1;
        //done with dllp
        if (fc_axis_tready) begin
          // the pair now on the wire is the pair last advertised
          ph_last_c  = ph_credits_allocated_i;
          pd_last_c  = pd_credits_allocated_i;
          timer_p_c  = '0;   // P's own UpdateFC: the only thing that restarts it
          next_state = ST_UPDATE_CRC;
        end
      end
      ST_UPDATE_CRC: begin
        //build axis master output
        fc_axis_tdata  = crc_reversed;
        fc_axis_tkeep  = 8'h03;
        fc_axis_tvalid = '1;
        fc_axis_tlast  = '1;
        //done with dllp
        if (fc_axis_tready) begin
          // NP follows only if NP is owed in its own right.  In an idle link
          // the NP timer, restarted two cycles after P's, expires exactly
          // here, so the periodic pair stays a P-then-NP pair.
          next_state = (dl_active && np_owed) ? ST_UPDATE_NP : ST_IDLE;
        end
      end
      ST_UPDATE_NP: begin
        //build axis master output
        dllp_lcrc_c = crc_out;
        fc_axis_tkeep = '1;
        fc_axis_tvalid = '1;
        //build dllp fc update for crc
        fc_axis_tdata = send_fc_init(UpdateFC_NP, '0, nph_credits_allocated_i, npd_credits_allocated_i);
        //done with dllp
        if (fc_axis_tready) begin
          nph_last_c = nph_credits_allocated_i;
          npd_last_c = npd_credits_allocated_i;
          timer_np_c = '0;   // NP's own UpdateFC: the only thing that restarts it
          next_state = ST_UPDATE_NP_CRC;
        end
      end
      ST_UPDATE_NP_CRC: begin
        //build axis master output
        fc_axis_tdata  = crc_reversed;
        fc_axis_tkeep  = 8'h03;
        fc_axis_tvalid = '1;
        fc_axis_tlast  = '1;
        //done with dllp
        if (fc_axis_tready) begin
          next_state = ST_IDLE;
        end
      end
      //send np
      ST_UPDATE_CPL: begin
        //build dllp fc update for crc
        fc_axis_tdata =
            send_fc_init(UpdateFC_Cpl, '0, '0, '0);
        dllp_lcrc_c = crc_out;
        //build axis master output
        fc_axis_tkeep = '1;
        fc_axis_tvalid = '1;
        //done with dllp
        if (fc_axis_tready) begin
          next_state = ST_UPDATE_CPL_CRC;
        end
      end
      ST_UPDATE_CPL_CRC: begin
        //build axis master output
        fc_axis_tdata  = crc_reversed;
        fc_axis_tkeep  = 8'h03;
        fc_axis_tvalid = '1;
        fc_axis_tlast  = '1;
        //done with dllp
        if (fc_axis_tready) begin
          next_state = ST_IDLE;
        end
      end
      ST_WAIT_LOW: begin
        start_ack_c = '1;
        if (!start_flow_control_i) begin
          start_ack_c = '0;
          next_state = ST_IDLE;
        end
      end
      default: begin
        next_state  = ST_IDLE;
        start_ack_c = '0;
      end
    endcase
  end

  //axis skid buffer
  axis_register #(
      .DATA_WIDTH(DATA_WIDTH),
      .KEEP_ENABLE('1),
      .KEEP_WIDTH(KEEP_WIDTH),
      .LAST_ENABLE('1),
      .ID_ENABLE('0),
      .ID_WIDTH(1),
      .DEST_ENABLE('0),
      .DEST_WIDTH(1),
      .USER_ENABLE('1),
      .USER_WIDTH(USER_WIDTH),
      .REG_TYPE(SkidBuffer)
  ) axis_register_pipeline_inst (
      .clk(clk_i),
      .rst(rst_i),
      .s_axis_tdata(fc_axis_tdata),
      .s_axis_tkeep(fc_axis_tkeep),
      .s_axis_tvalid(fc_axis_tvalid),
      .s_axis_tready(fc_axis_tready),
      .s_axis_tlast(fc_axis_tlast),
      .s_axis_tuser(fc_axis_tuser),
      .s_axis_tid('0),
      .s_axis_tdest('0),
      .m_axis_tdata(m_axis_tdata),
      .m_axis_tkeep(m_axis_tkeep),
      .m_axis_tvalid(m_axis_tvalid),
      .m_axis_tready(m_axis_tready),
      .m_axis_tlast(m_axis_tlast),
      .m_axis_tuser(m_axis_tuser),
      .m_axis_tid(),
      .m_axis_tdest()
  );

  pcie_datalink_crc dllp_crc_inst (
      .crcIn ('1),
      .data  (fc_axis_tdata),
      .crcOut(crc_out)
  );

  assign start_flow_control_ack_o = start_ack_r;

endmodule
