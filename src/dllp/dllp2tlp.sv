// ---------------------------------------------------------------------------
//! @title dllp2tlp
//! @author Idris Somoye
// Modified by: Kourosh Ghahramani
// Silicon Systems Research Lab, University of Washington
//! Module handles transaction layer packets recieved from the physical layer.
//! Packets intended for the tlp layer are decoded and sent through the tlp
//! master axis bus.
//
// Purpose
//   The TLP receive path of the Data Link Layer. Each link TLP arrives as
//   the 2-byte sequence prefix, the TLP and the 4-byte LCRC. This module
//   checks framing, the LCRC and the sequence number against NEXT_RCV_SEQ,
//   writes the TLP without prefix and LCRC into a frame FIFO that keeps it
//   only if every check passes, and asks dllp_fc_update for the Ack or Nak
//   that PCIe Base Spec r2.1, §3.5.3.1 requires. pcie_lcrc16 steps the LCRC
//   over the two prefix bytes of the first beat, from the FFFF FFFFh seed;
//   pcie_lcrc32 steps it over each TLP Dword. The module also keeps
//   CREDITS_ALLOCATED, stepped as each TLP leaves the FIFO.
//
// Interfaces
//   Link         link_status_i: a frame is started only in DL_ACTIVE.
//   Input        s_axis_*: link TLP frames from axis_user_demux, through a
//                skid buffer. The first beat holds the prefix in bits 15:0;
//                the last holds LCRC bytes 2 and 3 (tkeep 0011b). tuser bit 2
//                (UserIsEdb) on the last beat marks a frame that ended in EDB.
//   Ack/Nak      start_flow_control_o, start_flow_control_ack_i: request and
//                acknowledge with dllp_fc_update. next_transmit_seq_o[11:0]:
//                the AckNak_Seq_Num; tlp_nullified_o: 1 for a Nak.
//   Credits      ph_, pd_, nph_, npd_credits_allocated_o: CREDITS_ALLOCATED
//                for P and NP, reset to HdrMinCredits and PdMinCredits.
//   TLP output   m_tlp_axis_*: TLPs that passed every check, from the FIFO.
//
// Clock and reset
//   clk_i only. rst_i is synchronous and active high; pcie_datalink_layer
//   also asserts it while the link is down.
//
// Limitations
//   DATA_WIDTH must be 32: the prefix shift and the LCRC capture use fixed
//   16-bit halves. USER_WIDTH must be at least 3 for tuser bit 2. Each
//   accepted TLP gets its own Ack request; there is no AckNak_LATENCY_TIMER.
//   There is no Receiver Error input. Completion credits are counted but not
//   output. Of the eleven states only ST_IDLE, ST_TLP_STREAM, ST_CHECK_CRC
//   and ST_SEND_ACK are entered.
//
// Structure
//   Functions; Registers; LCRC field and compare; Receive state machine;
//   CREDITS_ALLOCATED; Submodules and outputs.
//
// References
//   PCIe Base Spec r2.1, §2.2.1
//   PCIe Base Spec r2.1, §2.6.1
//   PCIe Base Spec r2.1, §2.6.1.2
//   PCIe Base Spec r2.1, §3.5.2.1
//   PCIe Base Spec r2.1, §3.5.3.1
// ---------------------------------------------------------------------------
module dllp2tlp
  import pcie_datalink_pkg::*;
#(
    parameter int DATA_WIDTH = 32,
    parameter int STRB_WIDTH = DATA_WIDTH / 8,
    parameter int KEEP_WIDTH = STRB_WIDTH,
    parameter int USER_WIDTH = 1,
    parameter int MAX_PAYLOAD_SIZE = 256,
    parameter int RX_FIFO_SIZE = 2
) (
    input  logic                               clk_i,
    input  logic                               rst_i,
    input  pcie_dl_status_e                    link_status_i,

    // ---- link TLP frames ---------------------------------------------------
    input  logic            [  DATA_WIDTH-1:0] s_axis_tdata,
    input  logic            [  KEEP_WIDTH-1:0] s_axis_tkeep,
    input  logic                               s_axis_tvalid,
    input  logic                               s_axis_tlast,
    input  logic            [  USER_WIDTH-1:0] s_axis_tuser,
    output logic                               s_axis_tready,

    // ---- Ack/Nak request to dllp_fc_update ---------------------------------
    output logic                               start_flow_control_o,
    input  logic                               start_flow_control_ack_i,
    output logic            [            15:0] next_transmit_seq_o,
    output logic                               tlp_nullified_o,

    // ---- CREDITS_ALLOCATED, carried by dllp_fc_update's UpdateFC -----------
    output logic            [             7:0] ph_credits_allocated_o,
    output logic            [            11:0] pd_credits_allocated_o,
    output logic            [             7:0] nph_credits_allocated_o,
    output logic            [            11:0] npd_credits_allocated_o,

    // ---- TLPs to the Transaction Layer -------------------------------------
    output logic            [(DATA_WIDTH)-1:0] m_tlp_axis_tdata,
    output logic            [(KEEP_WIDTH)-1:0] m_tlp_axis_tkeep,
    output logic                               m_tlp_axis_tvalid,
    output logic                               m_tlp_axis_tlast,
    output logic            [(USER_WIDTH)-1:0] m_tlp_axis_tuser,
    input  logic                               m_tlp_axis_tready
);
  /* verilator lint_off WIDTHEXPAND */
  /* verilator lint_off WIDTHTRUNC */
  localparam int FcWaitPeriod = 8'hA0;
  localparam int TlpAxis = 0;
  localparam int UserIsTlp = 1;
  // data_handler sets receive tuser bit 2 when the frame ended in EDB.
  localparam int UserIsEdb = 2;
  localparam int MaxTlpHdrSizeDW = 4;
  localparam int MaxTlpTotalSizeDW = MaxTlpHdrSizeDW + (MAX_PAYLOAD_SIZE >> 2) + 1;
  localparam int MinRxBufferSize = MaxTlpTotalSizeDW * (RX_FIFO_SIZE);
  localparam int RamDataWidth = DATA_WIDTH;
  localparam int RamAddrWidth = $clog2(MinRxBufferSize);

  // Only ST_IDLE, ST_TLP_STREAM, ST_CHECK_CRC and ST_SEND_ACK are entered; see
  // the receive state machine below.
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
  // Latched from tuser on the frame's last beat, because ST_CHECK_CRC, one
  // state later, runs after that beat has been consumed.
  logic                                  frame_is_edb_c;
  logic                                  frame_is_edb_r;
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
  //tlp output axis signals
  logic                 [DATA_WIDTH-1:0] tlp_axis_tdata;
  logic                 [KEEP_WIDTH-1:0] tlp_axis_tkeep;
  logic                                  tlp_axis_tvalid;
  logic                                  tlp_axis_tlast;
  logic                 [USER_WIDTH-1:0] tlp_axis_tuser;
  logic                                  tlp_axis_tready;
  //credits tracking signals
  logic                 [          15:0] tlp_header_offset;
  // The CREDITS_ALLOCATED registers are declared with their block, after the
  // receive state machine.

  // -------------------------------------------------------------------------
  // Functions
  // -------------------------------------------------------------------------
  // keep_is_contiguous is declared and never called. sequence_is_duplicate
  // classifies, for ST_CHECK_CRC, a TLP whose sequence number is not
  // NEXT_RCV_SEQ. In this file NEXT_RCV_SEQ is next_expected_seq_num_r, and
  // the received TLP's sequence number is next_transmit_seq_r, captured from
  // the first beat.

  // True when keep is non-zero and its ones are contiguous from bit 0.
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
  // a duplicate.  A larger distance identifies a future/out-of-sequence TLP
  // (PCIe Base Spec r2.1, §3.5.3.1).
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

  // -------------------------------------------------------------------------
  // Registers
  // -------------------------------------------------------------------------
  // The state register and the *_r registers of the receive state machine,
  // each loaded from its *_c value. Two reset values come from the
  // specification: crc_calculated_r starts at FFFF FFFFh, the LCRC seed (PCIe
  // Base Spec r2.1, §3.5.2.1), and response_seq_r at FFFh, which is
  // NEXT_RCV_SEQ - 1 while NEXT_RCV_SEQ is 000h, the value an Ack or Nak
  // carries (PCIe Base Spec r2.1, §3.5.3.1). Every exit from ST_CHECK_CRC and
  // ST_SEND_ACK sets crc_calculated_c back to all ones, so each frame starts
  // from the seed.
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
      frame_is_edb_r          <= '0;
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
      frame_is_edb_r          <= frame_is_edb_c;
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


  // -------------------------------------------------------------------------
  // LCRC field and compare
  // -------------------------------------------------------------------------
  // pcie_lcrc16 and pcie_lcrc32 keep the LCRC register unreflected, so the
  // field is formed here: the register complemented and bit-reversed across
  // all 32 bits. That gives the four LCRC bytes in the order they arrive, the
  // first in bits 7:0 as in crc_from_tlp_r, each with the bit mapping of PCIe
  // Base Spec r2.1, §3.5.2.1. A per-byte bit reversal alone would give the
  // same bytes in the opposite order. lcrc_matches is the ordinary check;
  // lcrc_matches_inverted is the check for a nullified TLP.
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
  end


  // A nullified TLP carries the LCRC without the final complement, the
  // logical NOT of the normal field (PCIe Base Spec r2.1, §3.5.2.1), so its
  // check is the same compare against the complemented received field, with
  // no second CRC computation.
  logic lcrc_matches;
  logic lcrc_matches_inverted;
  assign lcrc_matches          = (lcrc32d32 == crc_from_tlp_r);
  assign lcrc_matches_inverted = (lcrc32d32 == ~crc_from_tlp_r);

  // -------------------------------------------------------------------------
  // Receive state machine
  // -------------------------------------------------------------------------
  // Checks one link TLP and decides the response; aligned_tlp_word is the TLP
  // Dword made of the previous beat's upper half and this beat's lower half.
  //   ST_IDLE        takes the first beat: sequence number, prefix check,
  //                  LCRC over the prefix. Exit: ST_TLP_STREAM; a one-beat
  //                  frame goes to ST_SEND_ACK for a Nak, or stays.
  //   ST_TLP_STREAM  writes each Dword to the FIFO one beat late, steps the
  //                  LCRC, classifies DW0. Exit: the last beat, ST_CHECK_CRC.
  //   ST_CHECK_CRC   writes the last Dword with tlast, marked bad unless all
  //                  checks pass, and picks Ack, Nak or no response. Exit:
  //                  ST_SEND_ACK for a response, otherwise ST_IDLE.
  //   ST_SEND_ACK    holds the request; on the acknowledge advances
  //                  NEXT_RCV_SEQ after a good TLP. Exit: ST_IDLE.
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
    frame_is_edb_c          = frame_is_edb_r;
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
        // has returned to idle. The acknowledge stays high until dllp_fc_update
        // sees the request fall, so this wait gives dllp_fc_update's ST_IDLE at
        // least one cycle with no request, in which an owed UpdateFC can start.
        skid_axis_tready = (link_status_i == DL_ACTIVE) &&
                           !start_flow_control_ack_i;
        if (skid_axis_tready && skid_axis_tvalid) begin
          //store incoming sequence number
          next_transmit_seq_c = {skid_axis_tdata[3:0], skid_axis_tdata[15:8]};
          // The Ack/Nak response fields are kept until this frame is
          // classified.
          // Clear packet-local error state. Reserved sequence bits mark this
          // frame bad but must not poison a later valid TLP.
          tlp_nullified_c = |skid_axis_tdata[7:4] ||
                            (skid_axis_tkeep != {KEEP_WIDTH{1'b1}});
          // Packet-local, like tlp_nullified_c above it.
          frame_is_edb_c  = '0;
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
            // pcie_lcrc16 takes these two bytes as one 16-bit step.
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
            // tuser is sampled on the frame's last beat, the one whose end
            // Symbol data_handler classified.
            frame_is_edb_c = skid_axis_tuser[UserIsEdb];
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
        // A frame that ended in EDB is always discarded (PCIe Base Spec r2.1,
        // §3.5.3.1); only the response, below, depends on its LCRC.
        if (tlp_nullified_r || frame_is_edb_r || !lcrc_matches ||
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
        end else if (frame_is_edb_r && lcrc_matches_inverted && tlp_axis_tready) begin
          // EDB and the inverted LCRC: a nullified TLP, discarded with no Ack,
          // Nak or change to NAK_SCHEDULED (PCIe Base Spec r2.1, §3.5.3.1).
          // NEXT_RCV_SEQ stays: the next TLP reuses this sequence number
          // (PCIe Base Spec r2.1, §3.5.2.1).
          pending_tlp_valid_c = '0;
          crc_calculated_c    = '1;
          tlp_nullified_c     = '0;
          response_required_c = '0;
          fc_start_c          = '0;
          next_state          = ST_IDLE;
        end else if (tlp_axis_tready) begin
          // Every other frame, once its last Dword is accepted. An EDB frame
          // whose LCRC is not the logical NOT of the calculated value is
          // corrupt and takes the last branch below, as a bad LCRC does (PCIe
          // Base Spec r2.1, §3.5.3.1).
          pending_tlp_valid_c  = '0;
          crc_calculated_c     = '1;
          response_seq_c       = next_expected_seq_num_r - 12'h001;
          response_is_nak_c    = '1;
          advance_expected_seq_c = '0;
          response_required_c  = '1;

          if (!tlp_nullified_r && !frame_is_edb_r && lcrc_matches &&
              (next_expected_seq_num_r == next_transmit_seq_r)) begin
            response_seq_c         = next_transmit_seq_r;
            response_is_nak_c      = '0;
            nak_scheduled_c        = '0;
            advance_expected_seq_c = '1;
            tlp_nullified_c        = '0;
            // Accepted: an Ack for this TLP. Its credit is returned in the
            // CREDITS_ALLOCATED block when it leaves dllp2tlp_fifo_inst.
          end else if (!tlp_nullified_r && !frame_is_edb_r && lcrc_matches &&
                       sequence_is_duplicate(next_transmit_seq_r,
                                             next_expected_seq_num_r)) begin
            // A duplicate: discarded, and acknowledged with an Ack.
            response_seq_c         = next_expected_seq_num_r - 12'h001;
            response_is_nak_c      = '0;
            advance_expected_seq_c = '0;
            tlp_nullified_c        = '1;
          end else begin
            // Bad LCRC, bad framing or out of sequence: a Nak, unless one is
            // already scheduled, in which case no response at all.
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

  // -------------------------------------------------------------------------
  // CREDITS_ALLOCATED
  // -------------------------------------------------------------------------
  // One register per credit type, reset to the InitFC advertisement:
  // HdrMinCredits and PdMinCredits for P and NP, the values pcie_flow_ctrl_init
  // sends, and 0 (infinite) for Cpl. Credit is granted again as a TLP leaves
  // dllp2tlp_fifo_inst, on the tlast handshake of m_tlp_axis, when its buffer
  // space is free (PCIe Base Spec r2.1, §2.6.1.2). Class and Length come from
  // DW0 on the frame's first output beat, decoded as ST_TLP_STREAM does. The
  // FIFO drops a bad frame before its output, so a TLP that was not accepted
  // returns nothing. The tlp_is_*_r flags that ST_TLP_STREAM sets drive
  // nothing.
  logic                 [           7:0] ph_credits_allocated_r;
  logic                 [          11:0] pd_credits_allocated_r;
  logic                 [           7:0] nph_credits_allocated_r;
  logic                 [          11:0] npd_credits_allocated_r;
  // The Cpl pair is counted and has no output, and must not get one.
  // pcie_flow_ctrl_init advertises Completion credits as infinite, and after
  // that an UpdateFC must carry 0 in those credit fields (PCIe Base Spec r2.1,
  // §2.6.1). These counts in an UpdateFC-Cpl would break that rule.
  logic                 [           7:0] cplh_credits_allocated_r;
  logic                 [          11:0] cpld_credits_allocated_r;

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
  // frame whose first beat is its last (no TLP is shorter than 3 DW, but the
  // FIFO does not know that) classifies live.
  assign cls_nph          = rel_first_r ? dec_nph  : rel_is_nph_r;
  assign cls_npd          = rel_first_r ? dec_npd  : rel_is_npd_r;
  assign cls_ph           = rel_first_r ? dec_ph   : rel_is_ph_r;
  assign cls_pd           = rel_first_r ? dec_pd   : rel_is_pd_r;
  assign cls_cplh         = rel_first_r ? dec_cplh : rel_is_cplh_r;
  assign cls_cpld         = rel_first_r ? dec_cpld : rel_is_cpld_r;
  assign cls_length       = rel_first_r ? dec_length : rel_length_r;
  // Length 0 encodes 1024 DW (PCIe Base Spec r2.1, §2.2.1): 256 credits of
  // 4 DW each.
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
        // One header credit per TLP, plus Roundup(Length/4) data credits for
        // the classes that carry data (PCIe Base Spec r2.1, §2.6.1).
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

  // -------------------------------------------------------------------------
  // Submodules and outputs
  // -------------------------------------------------------------------------
  // dllp2tlp_fifo_inst holds each TLP until its last Dword is checked. As a
  // frame FIFO with DROP_BAD_FRAME it drops a frame whose last beat has tuser
  // all ones. With DROP_WHEN_FULL = 0 it holds the input off when full, so a
  // TLP the transmitter had credit for waits instead of being dropped.
  // axis_register_pipeline_inst is the input skid buffer. tlp_crc16_inst and
  // pcie_lcrc32_inst step crc_calculated_r over crc_data: 16 bits for the
  // prefix, 32 bits for each TLP Dword. The outputs to dllp_fc_update follow.
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
      .FRAME_FIFO          (1),
      .USER_BAD_FRAME_VALUE('1),
      .USER_BAD_FRAME_MASK ('1),
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

  // Input skid buffer
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

  // The 32-bit LCRC with a 16-bit data step, over crc_data[15:0]
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

  // Despite their names, next_transmit_seq_o carries the AckNak_Seq_Num of
  // the requested Ack or Nak and tlp_nullified_o selects a Nak, matching
  // dllp_fc_update's next_transmit_seq_i and tlp_nullified_i.
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
