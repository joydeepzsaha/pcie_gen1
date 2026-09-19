//! @title dllp2tlp
//! @author Idris Somoye
//! Module handles transaction layer packets recieved from the physical layer.
//! Packets intended for the tlp layer are decoded and sent through the tlp
//! master axis bus.
module dllp2tlp
  import pcie_datalink_pkg::*;
#(
    // TLP data width
    parameter int DATA_WIDTH = 32,
    // TLP strobe width
    parameter int STRB_WIDTH = DATA_WIDTH / 8,
    parameter int KEEP_WIDTH = STRB_WIDTH,
    parameter int USER_WIDTH = 1,
    parameter int MAX_PAYLOAD_SIZE = 256,
    parameter int RX_FIFO_SIZE = 2
) (
    //clocks and resets
    input  logic                               clk_i,                     // Clock signal
    input  logic                               rst_i,                     // Reset signal
    //link status
    input  pcie_dl_status_e                    link_status_i,
    //TLP AXIS inputs
    input  logic            [  DATA_WIDTH-1:0] s_axis_tdata,
    input  logic            [  KEEP_WIDTH-1:0] s_axis_tkeep,
    input  logic                               s_axis_tvalid,
    input  logic                               s_axis_tlast,
    input  logic            [  USER_WIDTH-1:0] s_axis_tuser,
    output logic                               s_axis_tready,
    //flow control signals
    output logic                               start_flow_control_o,
    input  logic                               start_flow_control_ack_i,
    output logic            [            15:0] next_transmit_seq_o,
    output logic                               tlp_nullified_o,
    // CREDITS_ALLOCATED per non-infinite pool (Base 2.1 sec 2.6.1.2 p.141):
    // the count the InitFC/UpdateFC HdrFC/DataFC fields carry.  Stepped at
    // RELEASE, see the credits_allocated block below.
    output logic            [             7:0] ph_credits_allocated_o,
    output logic            [            11:0] pd_credits_allocated_o,
    output logic            [             7:0] nph_credits_allocated_o,
    output logic            [            11:0] npd_credits_allocated_o,
    //TLP dllp to tlp layer AXI Master
    output logic            [(DATA_WIDTH)-1:0] m_tlp_axis_tdata,
    output logic            [(KEEP_WIDTH)-1:0] m_tlp_axis_tkeep,
    output logic                               m_tlp_axis_tvalid,
    output logic                               m_tlp_axis_tlast,
    output logic            [(USER_WIDTH)-1:0] m_tlp_axis_tuser,
    input  logic                               m_tlp_axis_tready
);
  /* verilator lint_off WIDTHEXPAND */
  /* verilator lint_off WIDTHTRUNC */
  // localparam int PdMinCredits = (MAX_PAYLOAD_SIZE / 4);
  localparam int FcWaitPeriod = 8'hA0;
  localparam int TlpAxis = 0;
  localparam int UserIsTlp = 1;
  localparam int MaxTlpHdrSizeDW = 4;
  localparam int MaxTlpTotalSizeDW = MaxTlpHdrSizeDW + (MAX_PAYLOAD_SIZE >> 2) + 1;
  localparam int MinRxBufferSize = MaxTlpTotalSizeDW * (RX_FIFO_SIZE);
  localparam int RamDataWidth = DATA_WIDTH;
  localparam int RamAddrWidth = $clog2(MinRxBufferSize);

  //dllp to tlp fsm emum
  typedef enum logic [4:0] {
    ST_IDLE,
    ST_CHECK_TLP_TYPE,
    ST_TLP_STREAM,
    ST_TLP_LAST,
    ST_CHECK_CRC,
    ST_DRAIN_LCRC,
    ST_SEND_ACK,
    ST_SEND_ACK_CRC,
    ST_BUILD_FC_DLLP,
    ST_SEND_FC_DLLP,
    ST_SEND_FC_DLLP_CRC
  } dll_rx_st_e;


  dll_rx_st_e                            curr_state;
  dll_rx_st_e                            next_state;
  dllp_union_t                           dll_packet;
  //tlp nulled
  logic                                  fc_start_c;
  logic                                  fc_start_r;
  logic                                  tlp_nullified_c;
  logic                                  tlp_nullified_r;
  //transmit sequence logic
  logic                 [          11:0] next_transmit_seq_c;
  logic                 [          11:0] next_transmit_seq_r;
  logic                 [          11:0] next_expected_seq_num_c;
  logic                 [          11:0] next_expected_seq_num_r;
  // ACK/NAK response state.  The response sequence is the last TLP that was
  // successfully forwarded, not necessarily the sequence carried by the
  // packet currently being checked.
  logic                 [          11:0] response_seq_c;
  logic                 [          11:0] response_seq_r;
  logic                                  response_is_nak_c;
  logic                                  response_is_nak_r;
  // A NAK requests replay starting after the last good TLP.  Once scheduled,
  // later malformed/future TLPs must not schedule duplicates until the missing
  // expected TLP is accepted.
  logic                                  nak_scheduled_c;
  logic                                  nak_scheduled_r;
  logic                                  advance_expected_seq_c;
  logic                                  advance_expected_seq_r;
  logic                                  response_required_c;
  logic                                  response_required_r;
  logic                 [          11:0] ackd_transmit_seq_c;
  logic                 [          15:0] ackd_transmit_seq_r;
  //crc helper signals
  logic                 [          31:0] crc_from_tlp_c;
  logic                 [          31:0] crc_from_tlp_r;
  logic                 [          31:0] crc_calculated_c;
  logic                 [          31:0] crc_calculated_r;
  logic                 [          31:0] crc_output_16;
  logic                 [          31:0] crc_output_32;
  logic                 [          31:0] lcrc32d32;
  logic                 [          15:0] dllp_crc_out;
  logic                 [          15:0] dllp_lcrc32d32;
  logic                 [          31:0] dllp_lcrc_c;
  logic                 [          31:0] dllp_lcrc_r;
  logic                 [          31:0] word_count_c;
  logic                 [          31:0] word_count_r;
  logic                 [           1:0] crc_byte_select;
  //tlp type signals
  pcie_tlp_header_dw0_t                  tlp_dw0;
  logic                                  tlp_is_cplh_c;
  logic                                  tlp_is_cplh_r;
  logic                                  tlp_is_nph_c;
  logic                                  tlp_is_nph_r;
  logic                                  tlp_is_ph_c;
  logic                                  tlp_is_ph_r;
  logic                                  tlp_is_npd_c;
  logic                                  tlp_is_npd_r;
  logic                                  tlp_is_pd_c;
  logic                                  tlp_is_pd_r;
  logic                                  tlp_is_cpld_c;
  logic                                  tlp_is_cpld_r;
  //skid buffer axis signals
  logic                 [DATA_WIDTH-1:0] skid_axis_tdata;
  logic                 [KEEP_WIDTH-1:0] skid_axis_tkeep;
  logic                                  skid_axis_tvalid;
  logic                                  skid_axis_tlast;
  logic                 [USER_WIDTH-1:0] skid_axis_tuser;
  logic                                  skid_axis_tready;
  // A protected TLP is shifted by the two-byte sequence prefix.  Keep one
  // accepted input word and one assembled TLP word so alignment never relies
  // on an unaccepted AXI look-ahead beat.  This is intentionally bubble-safe.
  logic                 [DATA_WIDTH-1:0] previous_word_c;
  logic                 [DATA_WIDTH-1:0] previous_word_r;
  logic                 [DATA_WIDTH-1:0] pending_tlp_word_c;
  logic                 [DATA_WIDTH-1:0] pending_tlp_word_r;
  logic                                  pending_tlp_valid_c;
  logic                                  pending_tlp_valid_r;
  logic                 [DATA_WIDTH-1:0] crc_data;
  logic                 [DATA_WIDTH-1:0] aligned_tlp_word;
  //phy response signals
  // logic                 [DATA_WIDTH-1:0] phy_axis_tdata;
  // logic                 [KEEP_WIDTH-1:0] phy_axis_tkeep;
  // logic                                  phy_axis_tvalid;
  // logic                                  phy_axis_tlast;
  // logic                 [USER_WIDTH-1:0] phy_axis_tuser;
  // logic                                  phy_axis_tready;
  //tlp output axis signals
  logic                 [DATA_WIDTH-1:0] tlp_axis_tdata;
  logic                 [KEEP_WIDTH-1:0] tlp_axis_tkeep;
  logic                                  tlp_axis_tvalid;
  logic                                  tlp_axis_tlast;
  logic                 [USER_WIDTH-1:0] tlp_axis_tuser;
  logic                                  tlp_axis_tready;
  //credits tracking signals
  logic                 [          15:0] tlp_header_offset;
  // The six CREDITS_ALLOCATED registers are declared and owned by the
  // credits_allocated block below the FSM; they are stepped at RELEASE, not
  // here.  Before sec 63 #7f commit A they were *_credits_consumed_{c,r} and
  // stepped in ST_CHECK_CRC -- see that block for what moved and why.
  // ⚠️ DO NOT WIRE THE Cpl PAIR UP.  IT IS DEAD ON PURPOSE.
  //
  // Unlike ph/pd/nph/npd, cplh/cpld have NO output port (see the output
  // assigns -- there are four and no Cpl counterpart).  That asymmetry looks
  // like an oversight and is not.
  //
  // This design is a Root Complex that does not support peer-to-peer traffic
  // between all Root Ports, so PCIe Base 2.1 §2.6.1 p.137 REQUIRES it to
  // advertise INFINITE Completion credits -- "initial credit value of all 0s"
  // -- which pcie_flow_ctrl_init.sv:221,:315 does for InitFC1_Cpl/InitFC2_Cpl.
  // p.138 then says that once infinite has been advertised, no Flow Control
  // updates are required at all, and any UpdateFC that IS sent must carry zero
  // in the credit fields: "The Receiver may optionally check for non-zero
  // update values (in violation of this rule) ... the violation is a Flow
  // Control Protocol Error (FCPE)."
  //
  // So exporting these counters and feeding them into an UpdateFC_Cpl would
  // emit a non-zero update against an infinite advertisement -- an FCPE on the
  // link, caused by code that reads like a completed TODO.  The arithmetic
  // in the credits_allocated block is CORRECT (it is Table 2-36 fn 31's
  // Roundup(Length/4));
  // it is correct AND it must have no consumer.  The two facts are independent.
  //
  // Guarded by verilate_rc_dl_top's f2_initfc_cpl_advertises_infinite and
  // f2_no_updatefc_cpl_is_ever_emitted, each of which was shown to fail against
  // its own mutation (~/pcie_docs/evidence/stage-f-2/MUTATION_A.md).  Retiring
  // the counters outright is a cleanup-rung candidate, not a bug fix.

  function automatic logic keep_is_contiguous(
      input logic [KEEP_WIDTH-1:0] keep
  );
    logic [KEEP_WIDTH:0] extended_keep;
    begin
      extended_keep = {1'b0, keep};
      keep_is_contiguous = (keep != '0) &&
                           ((extended_keep & (extended_keep + 1'b1)) == '0);
    end
  endfunction

  // PCIe sequence arithmetic is modulo 4096.  A non-matching sequence whose
  // backward distance from NEXT_RCV_SEQ is at most half the sequence space is
  // a duplicate.  A larger distance identifies a future/out-of-sequence TLP.
  function automatic logic sequence_is_duplicate(
      input logic [11:0] received_sequence,
      input logic [11:0] expected_sequence
  );
    logic [11:0] backward_distance;
    begin
      backward_distance = expected_sequence - received_sequence;
      sequence_is_duplicate = (received_sequence != expected_sequence) &&
                              (backward_distance <= 12'h800);
    end
  endfunction

  //main sequential block
  always_ff @(posedge clk_i) begin : main_seq
    if (rst_i) begin
      curr_state              <= ST_IDLE;
      next_transmit_seq_r     <= '0;
      next_expected_seq_num_r <= '0;
      response_seq_r          <= 12'hfff;
      response_is_nak_r       <= '0;
      nak_scheduled_r         <= '0;
      advance_expected_seq_r  <= '0;
      response_required_r     <= '0;
      dllp_lcrc_r             <= '1;
      crc_calculated_r        <= '1;
      tlp_nullified_r         <= '0;
      fc_start_r              <= '0;
      word_count_r            <= '0;
      tlp_is_cplh_r           <= '0;
      tlp_is_nph_r            <= '0;
      tlp_is_ph_r             <= '0;
      tlp_is_cpld_r           <= '0;
      tlp_is_npd_r            <= '0;
      tlp_is_pd_r             <= '0;
      crc_from_tlp_r          <= '0;
      previous_word_r         <= '0;
      pending_tlp_word_r      <= '0;
      pending_tlp_valid_r     <= '0;
    end else begin
      curr_state              <= next_state;
      next_transmit_seq_r     <= next_transmit_seq_c;
      next_expected_seq_num_r <= next_expected_seq_num_c;
      response_seq_r          <= response_seq_c;
      response_is_nak_r       <= response_is_nak_c;
      nak_scheduled_r         <= nak_scheduled_c;
      advance_expected_seq_r  <= advance_expected_seq_c;
      response_required_r     <= response_required_c;
      dllp_lcrc_r             <= dllp_lcrc_c;
      crc_calculated_r        <= crc_calculated_c;
      tlp_nullified_r         <= tlp_nullified_c;
      fc_start_r              <= fc_start_c;
      word_count_r            <= word_count_c;
      tlp_is_cplh_r           <= tlp_is_cplh_c;
      tlp_is_nph_r            <= tlp_is_nph_c;
      tlp_is_ph_r             <= tlp_is_ph_c;
      tlp_is_cpld_r           <= tlp_is_cpld_c;
      tlp_is_npd_r            <= tlp_is_npd_c;
      tlp_is_pd_r             <= tlp_is_pd_c;
      crc_from_tlp_r          <= crc_from_tlp_c;
      previous_word_r         <= previous_word_c;
      pending_tlp_word_r      <= pending_tlp_word_c;
      pending_tlp_valid_r     <= pending_tlp_valid_c;
    end
  end


  always_comb begin : byteswap
    lcrc32d32 = {
      ~crc_calculated_r[0],
      ~crc_calculated_r[1],
      ~crc_calculated_r[2],
      ~crc_calculated_r[3],
      ~crc_calculated_r[4],
      ~crc_calculated_r[5],
      ~crc_calculated_r[6],
      ~crc_calculated_r[7],
      ~crc_calculated_r[8],
      ~crc_calculated_r[9],
      ~crc_calculated_r[10],
      ~crc_calculated_r[11],
      ~crc_calculated_r[12],
      ~crc_calculated_r[13],
      ~crc_calculated_r[14],
      ~crc_calculated_r[15],
      ~crc_calculated_r[16],
      ~crc_calculated_r[17],
      ~crc_calculated_r[18],
      ~crc_calculated_r[19],
      ~crc_calculated_r[20],
      ~crc_calculated_r[21],
      ~crc_calculated_r[22],
      ~crc_calculated_r[23],
      ~crc_calculated_r[24],
      ~crc_calculated_r[25],
      ~crc_calculated_r[26],
      ~crc_calculated_r[27],
      ~crc_calculated_r[28],
      ~crc_calculated_r[29],
      ~crc_calculated_r[30],
      ~crc_calculated_r[31]
    };
    // for (int i = 0; i < 8; i++) begin
    //   lcrc32d32[i]        = crc_calculated_r[7-i];
    //   lcrc32d32[i+8]      = crc_calculated_r[15-i];
    //   lcrc32d32[i+16]     = crc_calculated_r[23-i];
    //   lcrc32d32[i+24]     = crc_calculated_r[31-i];
    //   dllp_lcrc32d32[i]   = dllp_lcrc_r[7-i];
    //   dllp_lcrc32d32[i+8] = dllp_lcrc_r[15-i];
    // end
  end


  always_comb begin : main_combo
    next_state              = curr_state;
    dllp_lcrc_c             = dllp_lcrc_r;
    crc_calculated_c        = crc_calculated_r;
    crc_byte_select         = '0;
    crc_from_tlp_c          = crc_from_tlp_r;
    word_count_c            = word_count_r;
    tlp_is_cplh_c           = tlp_is_cplh_r;
    tlp_is_nph_c            = tlp_is_nph_r;
    tlp_is_ph_c             = tlp_is_ph_r;
    tlp_is_cpld_c           = tlp_is_cpld_r;
    tlp_is_npd_c            = tlp_is_npd_r;
    tlp_is_pd_c             = tlp_is_pd_r;
    skid_axis_tready        = '0;
    tlp_dw0                 = '0;
    dll_packet              = '0;
    tlp_header_offset       = '0;
    tlp_nullified_c         = tlp_nullified_r;
    fc_start_c              = '0;
    //tlp axis signals
    tlp_axis_tdata          = '0;
    tlp_axis_tkeep          = '0;
    tlp_axis_tvalid         = '0;
    tlp_axis_tlast          = '0;
    tlp_axis_tuser          = '0;
    next_transmit_seq_c     = next_transmit_seq_r;
    next_expected_seq_num_c = next_expected_seq_num_r;
    response_seq_c          = response_seq_r;
    response_is_nak_c       = response_is_nak_r;
    nak_scheduled_c         = nak_scheduled_r;
    advance_expected_seq_c  = advance_expected_seq_r;
    response_required_c     = response_required_r;
    previous_word_c         = previous_word_r;
    pending_tlp_word_c      = pending_tlp_word_r;
    pending_tlp_valid_c     = pending_tlp_valid_r;
    crc_data                = '0;
    aligned_tlp_word        = {skid_axis_tdata[15:0], previous_word_r[31:16]};
    case (curr_state)
      ST_IDLE: begin
        // Do not begin another packet until the previous response handshake
        // has returned to idle.  Otherwise a lingering ACK can acknowledge a
        // new request, or the new packet can overwrite the prior response.
        skid_axis_tready = (link_status_i == DL_ACTIVE) &&
                           !start_flow_control_ack_i;
        if (skid_axis_tready && skid_axis_tvalid) begin
          //store incoming sequence number
          next_transmit_seq_c = {skid_axis_tdata[3:0], skid_axis_tdata[15:8]};
          // Do not modify the latched ACK/NAK response while a packet is only
          // partially received.  dllp_fc_update may still be completing the
          // preceding response handshake and requires these fields to remain
          // stable until this packet is fully classified.
          // Clear packet-local error state. Reserved sequence bits mark this
          // frame bad but must not poison a later valid TLP.
          tlp_nullified_c = |skid_axis_tdata[7:4] ||
                            (skid_axis_tkeep != {KEEP_WIDTH{1'b1}});
          previous_word_c     = skid_axis_tdata;
          pending_tlp_word_c  = '0;
          pending_tlp_valid_c = '0;
          tlp_is_nph_c        = '0;
          tlp_is_pd_c         = '0;
          tlp_is_ph_c         = '0;
          tlp_is_npd_c        = '0;
          tlp_is_cplh_c       = '0;
          tlp_is_cpld_c       = '0;
          word_count_c        = '0;
          if (skid_axis_tlast) begin
            // A complete link TLP cannot contain sequence, header and LCRC in
            // one beat. Consume the truncated frame and request one replay.
            tlp_nullified_c = '1;
            if (!nak_scheduled_r) begin
              response_seq_c          = next_expected_seq_num_r - 12'h001;
              response_is_nak_c       = '1;
              advance_expected_seq_c  = '0;
              nak_scheduled_c         = '1;
              fc_start_c              = '1;
              next_state              = ST_SEND_ACK;
            end else begin
              next_state = ST_IDLE;
            end
          end else begin
            // The LCRC covers the two sequence bytes before it covers the
            // TLP.  Only accepted bytes are supplied to the CRC functions.
            crc_data             = {{(DATA_WIDTH-16){1'b0}}, skid_axis_tdata[15:0]};
            crc_byte_select      = 2'b11;
            crc_calculated_c     = crc_output_16;
            next_state           = ST_TLP_STREAM;
          end
        end
      end
      ST_TLP_STREAM: begin
        // The final protected beat contains only the upper half of the LCRC.
        // Capture it locally.  A non-final beat assembles one TLP dword from
        // two accepted protected-stream words and advances the LCRC exactly
        // once.  The pending dword is held until the following accepted beat
        // tells us whether it is the final TLP dword.
        if (skid_axis_tvalid && skid_axis_tlast) begin
          skid_axis_tready = '1;
        end else begin
          skid_axis_tready = !pending_tlp_valid_r || tlp_axis_tready;
          tlp_axis_tdata   = pending_tlp_word_r;
          tlp_axis_tkeep   = {KEEP_WIDTH{1'b1}};
          tlp_axis_tvalid  = skid_axis_tvalid && pending_tlp_valid_r;
        end

        if (skid_axis_tready && skid_axis_tvalid) begin
          if (skid_axis_tlast) begin
            crc_from_tlp_c = {skid_axis_tdata[15:0], previous_word_r[31:16]};
            if ((skid_axis_tkeep != {{(KEEP_WIDTH-2){1'b0}}, 2'b11}) ||
                !pending_tlp_valid_r) begin
              tlp_nullified_c = '1;
            end
            next_state = ST_CHECK_CRC;
          end else begin
            if (skid_axis_tkeep != {KEEP_WIDTH{1'b1}}) begin
              tlp_nullified_c = '1;
            end
            crc_data            = aligned_tlp_word;
            crc_calculated_c    = crc_output_32;
            previous_word_c     = skid_axis_tdata;
            pending_tlp_word_c  = aligned_tlp_word;
            pending_tlp_valid_c = '1;

            if (!pending_tlp_valid_r) begin
              tlp_dw0      = aligned_tlp_word;
              word_count_c = {tlp_dw0.byte2.Length1, tlp_dw0.byte3.Length0};
              // Eight of these labels embed `?` at the encoding's don't-care bits
              // -- Fmt[0] for the 3DW/4DW forms, the routing subfield for
              // messages.  A plain `case` compares 4-state-exact, so a `z` label
              // bit can never match received 0/1 data and those labels select
              // nothing.  The transmit sibling tlp2dllp matches the same labels
              // with `inside {...}`, which wildcard-matches; `casez` is the
              // receive-side equivalent.
              casez (tlp_dw0.byte0)
                MRd, MRdLk, IORd, CfgRd0, CfgRd1, TCfgRd:
                  tlp_is_nph_c = '1;
                MWr, MsgD:
                  tlp_is_pd_c = '1;
                Msg:
                  tlp_is_ph_c = '1;
                IOWr, CfgWr0, CfgWr1, TCfgWr, FetchAdd, Swap, CAS:
                  tlp_is_npd_c = '1;
                Cpl, CplLk:
                  tlp_is_cplh_c = '1;
                CplD, CplDLk:
                  tlp_is_cpld_c = '1;
                default: begin
                end
              endcase
            end
          end
        end
      end
      ST_CHECK_CRC: begin
        tlp_axis_tdata   = pending_tlp_word_r;
        tlp_axis_tkeep   = {KEEP_WIDTH{1'b1}};
        tlp_axis_tvalid  = pending_tlp_valid_r;
        tlp_axis_tlast   = '1;
        // Mark the final FIFO beat bad unless both framing/LCRC and sequence
        // checks pass.  FRAME_FIFO then atomically commits or drops the frame.
        if (tlp_nullified_r || (lcrc32d32 != crc_from_tlp_r) ||
            (next_expected_seq_num_r != next_transmit_seq_r)) begin
          tlp_axis_tuser = {USER_WIDTH{1'b1}};
        end

        if (!pending_tlp_valid_r) begin
          // A protected frame without a complete TLP dword cannot terminate a
          // FIFO frame.  It has not written any payload, so schedule NAK now.
          response_seq_c         = next_expected_seq_num_r - 12'h001;
          response_is_nak_c      = '1;
          advance_expected_seq_c = '0;
          tlp_nullified_c        = '1;
          crc_calculated_c       = '1;
          if (!nak_scheduled_r) begin
            nak_scheduled_c     = '1;
            response_required_c = '1;
            fc_start_c          = '1;
            next_state          = ST_SEND_ACK;
          end else begin
            response_required_c = '0;
            next_state          = ST_IDLE;
          end
        end else if (tlp_axis_tready) begin
          pending_tlp_valid_c  = '0;
          crc_calculated_c     = '1;
          response_seq_c       = next_expected_seq_num_r - 12'h001;
          response_is_nak_c    = '1;
          advance_expected_seq_c = '0;
          response_required_c  = '1;

          if (!tlp_nullified_r && (lcrc32d32 == crc_from_tlp_r) &&
              (next_expected_seq_num_r == next_transmit_seq_r)) begin
            response_seq_c         = next_transmit_seq_r;
            response_is_nak_c      = '0;
            nak_scheduled_c        = '0;
            advance_expected_seq_c = '1;
            tlp_nullified_c        = '0;
            // sec 63 #7f commit A: the credit step that used to sit here moved
            // to the credits_allocated block -- the frame is ACCEPTED here, its
            // buffer space is RELEASED when it leaves dllp2tlp_fifo_inst.
          end else if (!tlp_nullified_r && (lcrc32d32 == crc_from_tlp_r) &&
                       sequence_is_duplicate(next_transmit_seq_r,
                                             next_expected_seq_num_r)) begin
            response_seq_c         = next_expected_seq_num_r - 12'h001;
            response_is_nak_c      = '0;
            advance_expected_seq_c = '0;
            tlp_nullified_c        = '1;
          end else begin
            tlp_nullified_c = '1;
            if (!nak_scheduled_r) begin
              nak_scheduled_c = '1;
            end else begin
              response_required_c = '0;
            end
          end

          if (response_required_c) begin
            fc_start_c = '1;
            next_state = ST_SEND_ACK;
          end else begin
            fc_start_c = '0;
            next_state = ST_IDLE;
          end
        end
      end
      ST_SEND_ACK: begin
        fc_start_c = '1;
        if (start_flow_control_ack_i) begin
          fc_start_c = '0;
          if (advance_expected_seq_r) begin
            // Twelve-bit arithmetic provides the required 0xfff -> 0x000
            // rollover without widening into the reserved sequence bits.
            next_expected_seq_num_c = next_expected_seq_num_r + 12'h001;
          end
          advance_expected_seq_c = '0;
          response_required_c    = '0;
          tlp_is_nph_c     = '0;
          tlp_is_pd_c      = '0;
          tlp_is_ph_c      = '0;
          tlp_is_npd_c     = '0;
          tlp_is_cplh_c    = '0;
          tlp_is_cpld_c    = '0;
          crc_calculated_c = '1;
          next_state       = ST_IDLE;
        end
      end
      default: begin
      end
    endcase
  end

  // ===========================================================================
  // CREDITS_ALLOCATED -- the receive side's advertised count, stepped at RELEASE.
  // sec 63 #7f, #18 commit A.  Base 2.1 sec 2.6.1.2 p.141:
  //
  //   CREDITS_ALLOCATED: "Count of the total number of credits granted to the
  //   Transmitter since initialization, modulo 2^[Field Size]" ... "Initially
  //   set according to the buffer size and allocation policies of the
  //   Receiver" ... "This value is included in the InitFC and UpdateFC DLLPs"
  //   ... "Incremented as the Receiver Transaction Layer makes additional
  //   receive buffer space available by processing Received TLPs".
  //
  // Six registers, one per FC pool, reset to the InitFC advertisement
  // (HdrMinCredits / PdMinCredits for P and NP -- the same two constants
  // pcie_flow_ctrl_init puts in InitFC1/InitFC2 -- and 0 = infinite for Cpl,
  // F-2) and stepped when a TLP is HANDSHAKEN OUT of dllp2tlp_fifo_inst at
  // tlast, the point at which its buffer space is free.  Class and Length are
  // decoded from DW0 on the frame's first output beat with the SAME Fmt/Type
  // table ST_TLP_STREAM applies on the way in (Table 2-36); data credits are
  // Roundup(Length/4) with Length 0 = 1024 DW = 256 credits (Table 2-36 fn 31),
  // exactly the accept-side arithmetic this replaces.
  //
  // !! WHAT MOVED, AND WHY IT WAS RED.  Before this commit these six were named
  // *_credits_consumed_r and stepped in ST_CHECK_CRC on the LCRC pass -- before
  // the frame had even been committed to the receive FIFO.  A counter that
  // starts at the advertisement and counts received TLPs upward is
  // CREDITS_ALLOCATED wearing a consumed counter's name, and stepping it at
  // accept counts buffer space as available while the TLP still occupies it:
  // the Receiver Overflow hazard the same page names.  Measured RED in
  // tb/fullstack row W1 (fullstack_w1_ep_credits_allocated_advance_on_release):
  // every step landed one release ahead.  The FINAL value was always right,
  // which is why nothing before W1 saw it, and why the misnomer let Phase 2e's
  // probe label the PEER's limit as "the advertised register".
  //
  // !! INERT ON THE WIRE UNTIL COMMIT B (D-P3.3).  dllp_fc_update carries these
  // registers in its UpdateFC payload but fires from a 200,000-cycle timer no
  // test reaches (#7g); pcie_flow_ctrl_init's post-init UpdateFC pair carries
  // the constants, which equal these registers' reset value.  Commit B adds
  // the release-triggered schedule (sec 2.6.1.2 p.142) and is what makes the
  // peer see them.  Never B before A: B alone would broadcast a count that
  // steps before the buffer is free.
  //
  // FRAME_FIFO with DROP_BAD_FRAME drops a frame marked bad on its final beat
  // before it ever reaches the output, so a nullified, LCRC-failed or
  // out-of-sequence TLP frees nothing here -- and the accept path granted it
  // nothing either (it is not forwarded), so the two agree by construction.
  // DROP_WHEN_FULL=0: a TLP the far end held credit for is never lost, it
  // waits, and its credit is returned when it leaves.
  //
  // The tlp_is_*_r flags ST_TLP_STREAM still sets are the accept-side
  // classification; nothing reads them for credit any more.  Left in place --
  // removing FSM state is a different commit.
  // ===========================================================================
  logic                 [           7:0] ph_credits_allocated_r;
  logic                 [          11:0] pd_credits_allocated_r;
  logic                 [           7:0] nph_credits_allocated_r;
  logic                 [          11:0] npd_credits_allocated_r;
  logic                 [           7:0] cplh_credits_allocated_r;   // dead on purpose, see above
  logic                 [          11:0] cpld_credits_allocated_r;   // dead on purpose, see above

  logic                                  rel_first_r;   // the next output beat is a frame's DW0
  logic                                  rel_is_nph_r, rel_is_npd_r, rel_is_ph_r, rel_is_pd_r;
  logic                                  rel_is_cplh_r, rel_is_cpld_r;
  logic                 [           9:0] rel_length_r;
  pcie_tlp_header_dw0_t                  rel_dw0;
  logic                                  rel_hs, rel_hs_last;
  logic                                  dec_nph, dec_npd, dec_ph, dec_pd, dec_cplh, dec_cpld;
  logic                 [           9:0] dec_length;
  logic                                  cls_nph, cls_npd, cls_ph, cls_pd, cls_cplh, cls_cpld;
  logic                 [           9:0] cls_length;
  logic                 [          11:0] cls_data_credits;

  assign rel_hs      = m_tlp_axis_tvalid && m_tlp_axis_tready;
  assign rel_hs_last = rel_hs && m_tlp_axis_tlast;
  assign rel_dw0     = m_tlp_axis_tdata;
  assign dec_length  = {rel_dw0.byte2.Length1, rel_dw0.byte3.Length0};

  always_comb begin : release_classify
    dec_nph  = 1'b0;
    dec_npd  = 1'b0;
    dec_ph   = 1'b0;
    dec_pd   = 1'b0;
    dec_cplh = 1'b0;
    dec_cpld = 1'b0;
    // Same labels, same casez, as ST_TLP_STREAM's inbound classifier.
    casez (rel_dw0.byte0)
      MRd, MRdLk, IORd, CfgRd0, CfgRd1, TCfgRd:           dec_nph  = 1'b1;
      MWr, MsgD:                                          dec_pd   = 1'b1;
      Msg:                                                dec_ph   = 1'b1;
      IOWr, CfgWr0, CfgWr1, TCfgWr, FetchAdd, Swap, CAS:  dec_npd  = 1'b1;
      Cpl, CplLk:                                         dec_cplh = 1'b1;
      CplD, CplDLk:                                       dec_cpld = 1'b1;
      default: begin
      end
    endcase
  end

  // On the frame's last beat use the class latched from its first beat; a
  // frame whose first beat IS its last (no TLP is shorter than 3 DW, but the
  // FIFO does not know that) classifies live.
  assign cls_nph          = rel_first_r ? dec_nph  : rel_is_nph_r;
  assign cls_npd          = rel_first_r ? dec_npd  : rel_is_npd_r;
  assign cls_ph           = rel_first_r ? dec_ph   : rel_is_ph_r;
  assign cls_pd           = rel_first_r ? dec_pd   : rel_is_pd_r;
  assign cls_cplh         = rel_first_r ? dec_cplh : rel_is_cplh_r;
  assign cls_cpld         = rel_first_r ? dec_cpld : rel_is_cpld_r;
  assign cls_length       = rel_first_r ? dec_length : rel_length_r;
  assign cls_data_credits = (cls_length == '0) ? 12'd256
                                               : 12'((13'(cls_length) + 13'd3) >> 2);

  always_ff @(posedge clk_i) begin : credits_allocated_seq
    if (rst_i) begin
      rel_first_r              <= 1'b1;
      rel_is_nph_r             <= 1'b0;
      rel_is_npd_r             <= 1'b0;
      rel_is_ph_r              <= 1'b0;
      rel_is_pd_r              <= 1'b0;
      rel_is_cplh_r            <= 1'b0;
      rel_is_cpld_r            <= 1'b0;
      rel_length_r             <= '0;
      ph_credits_allocated_r   <= HdrMinCredits;
      pd_credits_allocated_r   <= PdMinCredits;
      nph_credits_allocated_r  <= HdrMinCredits;
      npd_credits_allocated_r  <= PdMinCredits;
      cplh_credits_allocated_r <= '0;
      cpld_credits_allocated_r <= '0;
    end else begin
      if (rel_hs && rel_first_r) begin
        rel_first_r   <= 1'b0;
        rel_is_nph_r  <= dec_nph;
        rel_is_npd_r  <= dec_npd;
        rel_is_ph_r   <= dec_ph;
        rel_is_pd_r   <= dec_pd;
        rel_is_cplh_r <= dec_cplh;
        rel_is_cpld_r <= dec_cpld;
        rel_length_r  <= dec_length;
      end
      if (rel_hs_last) begin
        rel_first_r <= 1'b1;
        // "made available by TLPs processed" -- one header credit per TLP,
        // plus Roundup(Length/4) data credits for the data-bearing classes.
        if (cls_nph) begin
          nph_credits_allocated_r  <= nph_credits_allocated_r + 8'h1;
        end else if (cls_npd) begin
          nph_credits_allocated_r  <= nph_credits_allocated_r + 8'h1;
          npd_credits_allocated_r  <= npd_credits_allocated_r + cls_data_credits;
        end else if (cls_ph) begin
          ph_credits_allocated_r   <= ph_credits_allocated_r + 8'h1;
        end else if (cls_pd) begin
          ph_credits_allocated_r   <= ph_credits_allocated_r + 8'h1;
          pd_credits_allocated_r   <= pd_credits_allocated_r + cls_data_credits;
        end else if (cls_cplh) begin
          cplh_credits_allocated_r <= cplh_credits_allocated_r + 8'h1;
        end else if (cls_cpld) begin
          cplh_credits_allocated_r <= cplh_credits_allocated_r + 8'h1;
          cpld_credits_allocated_r <= cpld_credits_allocated_r + cls_data_credits;
        end
      end
    end
  end

  //dllp2tlp fifo.. allows for processing tlp
  //and storing to confirm proper tlp seq num and crc..
  //before sending to the transaction layer
  axis_fifo #(
      .DEPTH               (RX_FIFO_SIZE * MAX_PAYLOAD_SIZE),
      .DATA_WIDTH          (DATA_WIDTH),
      .KEEP_ENABLE         (KEEP_WIDTH > 0),
      .KEEP_WIDTH          (KEEP_WIDTH),
      .LAST_ENABLE         (1),
      .ID_ENABLE           (0),
      .DEST_ENABLE         (0),
      .USER_ENABLE         ('1),
      .USER_WIDTH          (USER_WIDTH),
      // .PIPELINE_OUTPUT(2),
      .FRAME_FIFO          (1),
      .USER_BAD_FRAME_VALUE('1),
      .USER_BAD_FRAME_MASK ('1),
      // .PIPELINE_OUTPUT(),
      .DROP_BAD_FRAME      (1),
      .DROP_WHEN_FULL      (0)
  ) dllp2tlp_fifo_inst (
      .clk                (clk_i),
      .rst                (rst_i),
      // AXI input
      .s_axis_tdata       (tlp_axis_tdata),
      .s_axis_tkeep       (tlp_axis_tkeep),
      .s_axis_tvalid      (tlp_axis_tvalid),
      .s_axis_tready      (tlp_axis_tready),
      .s_axis_tlast       (tlp_axis_tlast),
      .s_axis_tuser       (tlp_axis_tuser),
      .s_axis_tid         (),
      .s_axis_tdest       (),
      // AXI output
      .m_axis_tdata       (m_tlp_axis_tdata),
      .m_axis_tkeep       (m_tlp_axis_tkeep),
      .m_axis_tvalid      (m_tlp_axis_tvalid),
      .m_axis_tready      (m_tlp_axis_tready),
      .m_axis_tlast       (m_tlp_axis_tlast),
      .m_axis_tuser       (m_tlp_axis_tuser),
      .m_axis_tid         (),
      .m_axis_tdest       (),
      .pause_ack          (),
      .pause_req          (),
      .status_depth       (),
      .status_depth_commit(),
      // Status
      .status_overflow    (),
      .status_bad_frame   (),
      .status_good_frame  ()
  );

  //axis input skid buffer
  axis_register #(
      .DATA_WIDTH (DATA_WIDTH),
      .KEEP_ENABLE('1),
      .KEEP_WIDTH (KEEP_WIDTH),
      .LAST_ENABLE('1),
      .ID_ENABLE  ('0),
      .ID_WIDTH   (1),
      .DEST_ENABLE('0),
      .DEST_WIDTH (1),
      .USER_ENABLE('1),
      .USER_WIDTH (USER_WIDTH),
      .REG_TYPE   (SkidBuffer)
  ) axis_register_pipeline_inst (
      .clk          (clk_i),
      .rst          (rst_i),
      .s_axis_tdata (s_axis_tdata),
      .s_axis_tkeep (s_axis_tkeep),
      .s_axis_tvalid(s_axis_tvalid),
      .s_axis_tready(s_axis_tready),
      .s_axis_tlast (s_axis_tlast),
      .s_axis_tuser (s_axis_tuser),
      .s_axis_tid   ('0),
      .s_axis_tdest ('0),
      .m_axis_tdata (skid_axis_tdata),
      .m_axis_tkeep (skid_axis_tkeep),
      .m_axis_tvalid(skid_axis_tvalid),
      .m_axis_tready(skid_axis_tready),
      .m_axis_tlast (skid_axis_tlast),
      .m_axis_tuser (skid_axis_tuser),
      .m_axis_tid   (),
      .m_axis_tdest ()
  );

  //tlp crc instance
  pcie_lcrc16 tlp_crc16_inst (
      .data  (crc_data),
      .crcIn (crc_calculated_r),
      .crcOut(crc_output_16)
  );

  pcie_lcrc32 pcie_lcrc32_inst (
      .crcIn (crc_calculated_r),
      .data  (crc_data),
      .crcOut(crc_output_32)
  );

  //output assignments
  // Preserve the existing port names for integration compatibility.  Their
  // values now have the protocol-correct meanings required by dllp_fc_update:
  // response sequence and response-is-NAK.
  assign next_transmit_seq_o    = {4'b0000, response_seq_r};
  assign tlp_nullified_o        = response_is_nak_r;
  assign ph_credits_allocated_o  = ph_credits_allocated_r;
  assign pd_credits_allocated_o  = pd_credits_allocated_r;
  assign nph_credits_allocated_o = nph_credits_allocated_r;
  assign npd_credits_allocated_o = npd_credits_allocated_r;
  assign start_flow_control_o   = fc_start_r;

  /* verilator lint_on WIDTHEXPAND */
  /* verilator lint_on WIDTHTRUNC */
endmodule
