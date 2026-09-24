// import pcie_datalink_pkg::*;
module dllp_fc_update
  import pcie_datalink_pkg::*;
#(
    parameter int CLK_RATE         = 100,
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
  localparam int ClockPeriodNs = ((10 ** 3) / CLK_RATE);
  localparam int TwoMsTimeOut = 2_000_000 / ClockPeriodNs;
  localparam int FcWaitPeriod = TwoMsTimeOut;
  localparam int TimerWidth = $clog2(FcWaitPeriod + 1);
  // localparam int TwoMsTimeOut = (CLK_RATE * (2 ** 5));  //32'h000B8D80;  //temp value

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
  logic             [TimerWidth-1:0] timer_c;
  logic             [TimerWidth-1:0] timer_r;
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
  // ST_IDLE priority is unchanged in kind: an Ack/Nak request still wins, the
  // periodic path is untouched (its 200,000-cycle period is #7g's), and the
  // release path sits between them.  P and NP are scheduled INDEPENDENTLY --
  // an NP release does not send a redundant UpdateFC-P -- while the periodic
  // path keeps its P-then-NP pair; rel_seq_r says which kind is in flight.
  // The periodic timer is cleared on every exit, as the Ack path already
  // clears it: the timer measures quiet since the last DLLP this module sent.
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
  logic                              rel_seq_c, rel_seq_r;
  logic                              p_pending, np_pending;

  assign p_pending  = (ph_credits_allocated_i  != ph_last_r)  ||
                      (pd_credits_allocated_i  != pd_last_r);
  assign np_pending = (nph_credits_allocated_i != nph_last_r) ||
                      (npd_credits_allocated_i != npd_last_r);

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
      timer_r <= '0;
      dllp_lcrc_r <= '0;
      start_ack_r <= '0;
      ack_nak_seq_r <= '0;
      ack_nak_is_nak_r <= '0;
      ph_last_r <= HdrMinCredits;
      pd_last_r <= PdMinCredits;
      nph_last_r <= HdrMinCredits;
      npd_last_r <= PdMinCredits;
      rel_seq_r <= '0;
    end else begin
      curr_state <= next_state;
      dll_packet_r <= dll_packet_c;
      timer_r <= timer_c;
      dllp_lcrc_r <= dllp_lcrc_c;
      start_ack_r <= start_ack_c;
      ack_nak_seq_r <= ack_nak_seq_c;
      ack_nak_is_nak_r <= ack_nak_is_nak_c;
      ph_last_r <= ph_last_c;
      pd_last_r <= pd_last_c;
      nph_last_r <= nph_last_c;
      npd_last_r <= npd_last_c;
      rel_seq_r <= rel_seq_c;
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
    timer_c        = timer_r;
    start_ack_c    = '0;
    ack_nak_seq_c = ack_nak_seq_r;
    ack_nak_is_nak_c = ack_nak_is_nak_r;
    ph_last_c      = ph_last_r;
    pd_last_c      = pd_last_r;
    nph_last_c     = nph_last_r;
    npd_last_c     = npd_last_r;
    rel_seq_c      = rel_seq_r;
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
        timer_c = (timer_r >= FcWaitPeriod) ? FcWaitPeriod : timer_r + 1;
        if (start_flow_control_i) begin
          next_state       = ST_SEND_ACK;
          timer_c          = '0;
          ack_nak_seq_c    = next_transmit_seq_i[11:0];
          ack_nak_is_nak_c = tlp_nullified_i;
        end else if ((link_status_i == DL_ACTIVE) && (p_pending || np_pending)) begin
          // sec 2.6.1.2 p.142, the release clause: credit was made available
          // by a TLP processed and has not yet been advertised.  Only the
          // type(s) owed are sent.
          rel_seq_c  = '1;
          next_state = p_pending ? ST_UPDATE_P : ST_UPDATE_NP;
        end else if ((timer_r >= FcWaitPeriod) && (link_status_i == DL_ACTIVE)) begin
          timer_c    = '0;
          rel_seq_c  = '0;
          next_state = ST_UPDATE_P;
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
          if (rel_seq_r && !np_pending) begin
            // release-triggered P only: nothing owed for NP, so no NP DLLP
            timer_c    = '0;
            rel_seq_c  = '0;
            next_state = ST_IDLE;
          end else begin
            next_state = ST_UPDATE_NP;
          end
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
          timer_c = '0;
          rel_seq_c = '0;
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
          timer_c = '0;
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
        timer_c     = '0;
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
