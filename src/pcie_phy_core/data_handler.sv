// ---------------------------------------------------------------------------
// data_handler -- receive framing: Symbol words in, TLPs and DLLPs out
//
// Purpose
//   Takes the words gathered by pack_data and finds the framing Symbols: STP
//   or SDP opens a packet, END or EDB closes it (PCIe Base Spec r2.1,
//   §4.2.2). The framing Symbols are stripped, the bytes between them are
//   realigned to start at byte 0 of a beat, and each packet leaves on an
//   AXI stream with tkeep, tlast and a tuser that marks TLP or DLLP and a
//   frame ended by EDB. A DLLP, four bytes and a 16-bit CRC (§3.4.1), leaves
//   as a beat with tkeep 1111b and a tlast beat with tkeep 0011b.
//
// Interfaces
//   Input         data_i, data_k_i, data_valid_i: one gathered word from
//                 pack_data; only lane 0 is read.
//   Control       phy_link_up_i, phy_fifo_empty_i: ST_IDLE exits on link up
//                 with the FIFO not empty (phy_receive ties the latter to 0).
//                 curr_data_rate_i: 8b/10b framing below gen3.
//   Packets       m_dllp_axis_*, through a skid buffer; despite the name, TLPs
//                 leave on it too. tuser bit 0 marks a DLLP, bit 1 a TLP and
//                 bit 2 (UserIsEdb) a frame ended by EDB.
//   Unused        phy_fifo_rd_en_o (left open in phy_receive), sync_header_i,
//                 lane_reverse_i, pipe_width_i, num_active_lanes_i.
//
// Clock and reset
//   clk_i only (pipe_rx_usr_clk_i in phy_receive). rst_i is synchronous and
//   active high and resets curr_state and the skid buffer; the data
//   registers have no reset.
//
// Limitations
//   Lane 0 only. Nothing pushes back on pack_data: in ST_TX a word that
//   arrives while the skid buffer is not ready is never taken. The 8 GT/s
//   branch does not work: ST_TX_TLP has no exit, and sync_header_r, which
//   its STP test reads, never takes sync_header_i.
//
// References
//   PCIe Base Spec r2.1, §3.4.1
//   PCIe Base Spec r2.1, §3.5.3.1
//   PCIe Base Spec r2.1, §4.2.2
// ---------------------------------------------------------------------------
module data_handler
  import pcie_phy_pkg::*;
#(
    // Bits per output beat, and per lane of data_i.
    parameter int DATA_WIDTH    = 32,
    parameter int STRB_WIDTH    = DATA_WIDTH / 8,
    parameter int KEEP_WIDTH    = STRB_WIDTH,
    // At least 3, to carry tuser bit UserIsEdb: at the default, 1, the initial
    // block below calls $fatal. phy_receive passes its own, 5 by default.
    parameter int USER_WIDTH    = 1,
    parameter int MAX_NUM_LANES = 4
) (
    // ---- clock, reset and link state --------------------------------------
    input  logic clk_i,
    input  logic rst_i,
    input  logic phy_link_up_i,
    input  logic phy_fifo_empty_i,
    output logic phy_fifo_rd_en_o,
    // ---- input words from pack_data, and the data rate --------------------
    //! @virtualbus master_axis_bus @dir out

    input logic        lane_reverse_i,
    input rate_speed_e curr_data_rate_i,

    input logic [( MAX_NUM_LANES* DATA_WIDTH)-1:0] data_i,
    input logic [               MAX_NUM_LANES-1:0] data_valid_i,
    input logic [           (4*MAX_NUM_LANES)-1:0] data_k_i,
    input logic [           (2*MAX_NUM_LANES)-1:0] sync_header_i,

    // ---- packet output: TLPs and DLLPs -------------------------------------
    output logic [DATA_WIDTH-1:0] m_dllp_axis_tdata,
    output logic [KEEP_WIDTH-1:0] m_dllp_axis_tkeep,
    output logic                  m_dllp_axis_tvalid,
    output logic                  m_dllp_axis_tlast,
    output logic [USER_WIDTH-1:0] m_dllp_axis_tuser,
    input  logic                  m_dllp_axis_tready,
    // ---- unused -------------------------------------------------------------
    input  logic [           5:0] pipe_width_i,
    input  logic [           5:0] num_active_lanes_i
);


  // Only BytesPerTransfer is used.
  localparam int PipeWidthGen1 = 8;
  localparam int PipeWidthGen2 = 16;
  localparam int PipeWidthGen3 = 16;
  localparam int PipeWidthGen4 = 32;
  localparam int PipeWidthGen5 = 32;
  localparam int BytesPerTransfer = DATA_WIDTH / 8;
  localparam int MaxWordsPerTransaction = 512 / DATA_WIDTH;

  // -------------------------------------------------------------------------
  // Framing state machine
  // -------------------------------------------------------------------------
  // Below gen3 only ST_IDLE, ST_CHECK_FRAME and ST_TX are used.
  //
  //   State              Does                           Exit
  //   ST_IDLE            waits                          link up, FIFO not empty
  //   ST_CHECK_FRAME     looks for STP or SDP           start Symbol: ST_TX; at 8 GT/s
  //                                                     ST_TX_DLLP, ST_TX_TLP or ST_IDLE
  //   ST_TX              one beat per word; looks for   END or EDB: ST_CHECK_FRAME,
  //                      END, EDB and a start Symbol    unless a start follows
  //   ST_TX_DLLP         8 GT/s: two stored beats       ST_IDLE
  //   ST_TX_TLP          8 GT/s: no statements          none
  //
  // ST_CHECK_END and ST_CHECK_END_GEN3 are never entered.
  typedef enum logic [4:0] {
    ST_IDLE,
    ST_TX,
    ST_CHECK_FRAME,
    ST_CHECK_END,
    ST_CHECK_END_GEN3,
    ST_TX_TLP,
    ST_TX_DLLP
  } data_handler_state_e;


  data_handler_state_e                                    curr_state;
  data_handler_state_e                                    next_state;

  // Into the skid buffer. data_handler_axis_tuser is assigned below and read
  // nowhere: the skid buffer's tuser is handler_tuser.
  logic                [                  DATA_WIDTH-1:0] data_handler_axis_tdata;
  logic                [                  KEEP_WIDTH-1:0] data_handler_axis_tkeep;
  logic                                                   data_handler_axis_tvalid;
  logic                                                   data_handler_axis_tlast;
  logic                [                  USER_WIDTH-1:0] data_handler_axis_tuser;
  // tuser bit for a frame ended by EDB rather than END. The Data Link Layer
  // treats the two differently: a TLP ended by EDB whose LCRC is the inverse
  // of the calculated value is discarded without an error, while a bad LCRC
  // on a TLP ended by END is an error and schedules a Nak (PCIe Base Spec
  // r2.1, §3.5.3.1). dllp2tlp reads this bit.
  localparam int UserIsEdb = 2;
  logic                                                   frame_is_edb;
  logic                                                   data_handler_axis_tready;



  // The word being worked on. In ST_TX, data_r and data_k_r hold the previous
  // word: its last word_count_r bytes are carried into the next beat, and its
  // end Symbols are checked there. sync_header_c only ever copies
  // sync_header_r, so sync_header_r never takes sync_header_i.
  logic                [( MAX_NUM_LANES* DATA_WIDTH)-1:0] data_c;
  logic                [( MAX_NUM_LANES* DATA_WIDTH)-1:0] data_r;
  logic                [               MAX_NUM_LANES-1:0] data_valid_c;
  logic                [               MAX_NUM_LANES-1:0] data_valid_r;
  logic                [           (4*MAX_NUM_LANES)-1:0] data_k_c;
  logic                [           (4*MAX_NUM_LANES)-1:0] data_k_r;
  logic                [           (2*MAX_NUM_LANES)-1:0] sync_header_c;
  logic                [           (2*MAX_NUM_LANES)-1:0] sync_header_r;



  // Bytes of the packet that followed its start Symbol in the start word.
  // ST_TX carries that many bytes of each word into the next beat; ST_TX_DLLP
  // uses it as a beat count.
  logic                [                             5:0] word_count_c;
  logic                [                             5:0] word_count_r;

  // is_tlp_r records the packet type from its start Symbol; is_dllp_r is
  // never read, since handler_tuser marks every frame that is not a TLP as a
  // DLLP. data_start_r is high from the clock after a start Symbol is found
  // until ST_TX takes the next word, so ST_TX's end check on the previous word
  // skips the start word. skid_r is set only in ST_CHECK_END_GEN3, which is
  // never entered. ready_out and fifo_rd are never used.
  logic                                                   is_dllp_c;
  logic                                                   is_dllp_r;
  logic                                                   is_tlp_c;
  logic                                                   is_tlp_r;
  logic                                                   data_start_c;
  logic                                                   data_start_r;
  logic                                                   skid_c;
  logic                                                   skid_r;
  logic                                                   ready_out;


  logic                                                   fifo_rd;

  always_ff @(posedge clk_i) begin : main_seq_block
    if (rst_i) begin
      curr_state <= ST_IDLE;
    end else begin
      curr_state <= next_state;
    end
    data_valid_r  <= data_valid_c;
    sync_header_r <= sync_header_c;
    data_k_r      <= data_k_c;
    data_r        <= data_c;
    is_tlp_r      <= is_tlp_c;
    is_dllp_r     <= is_dllp_c;
    word_count_r  <= word_count_c;
    skid_r        <= skid_c;
    data_start_r  <= data_start_c;
  end

  always_comb begin : lane_data_sync

    data_handler_axis_tdata  = '0;
    data_handler_axis_tkeep  = '0;
    data_handler_axis_tvalid = '0;
    data_handler_axis_tlast  = '0;
    data_handler_axis_tuser  = '0;
    frame_is_edb             = '0;


    is_tlp_c                 = is_tlp_r;
    is_dllp_c                = is_dllp_r;
    data_start_c             = data_start_r;
    word_count_c             = word_count_r;
    next_state               = curr_state;
    data_valid_c             = '0;
    sync_header_c            = sync_header_r;
    data_k_c                 = data_k_r;
    data_c                   = data_r;
    phy_fifo_rd_en_o         = '0;
    skid_c                   = skid_r;
    case (curr_state)
      ST_IDLE: begin
        if (phy_link_up_i && !phy_fifo_empty_i) begin
          phy_fifo_rd_en_o = '1;
          next_state = ST_CHECK_FRAME;
          skid_c = '0;
        end
      end
      ST_CHECK_FRAME: begin
        phy_fifo_rd_en_o = '1;
        if (|data_valid_i) begin
          data_start_c = '0;
          if (curr_data_rate_i < gen3) begin
            data_c        = data_i;
            data_k_c      = data_k_i;
            sync_header_c = sync_header_r;
            word_count_c  = '0;
            is_dllp_c     = '0;
            is_tlp_c      = '0;
            // The last start Symbol in the word wins; word_count_c is the
            // number of bytes after it in this word.
            for (int byte_idx = 0; byte_idx < BytesPerTransfer; byte_idx++) begin
              if (data_k_i[byte_idx] && (data_i[8*byte_idx+:8] == SDP)) begin
                is_dllp_c    = '1;
                data_c       = data_i;
                next_state   = ST_TX;
                data_valid_c = data_valid_i;
                word_count_c = BytesPerTransfer - 1 - byte_idx;
                data_start_c = '1;
              end else if (data_k_i[byte_idx] && (data_i[8*byte_idx+:8] == STP)) begin
                is_tlp_c     = '1;
                data_c       = data_i;
                next_state   = ST_TX;
                data_valid_c = data_valid_i;
                word_count_c = BytesPerTransfer - 1 - byte_idx;
                data_start_c = '1;
              end
            end
          end else begin
            // 8 GT/s: SDP and STP are matched as bit patterns without K flags
            // (check_sdp, check_stp). sync_header_r never takes sync_header_i.
            word_count_c = '0;
            if (check_sdp(data_i)) begin
              data_c        = data_i;
              data_valid_c  = data_valid_i;
              data_k_c      = data_k_i;
              sync_header_c = sync_header_r;
              is_dllp_c     = '1;
              next_state    = ST_TX_DLLP;
            end else if (check_stp(data_i) && sync_header_r[1:0] == 2'b01) begin
              is_tlp_c   = '1;
              next_state = ST_TX_TLP;
            end else begin
              next_state = ST_IDLE;
            end
          end
        end
      end
      ST_TX: begin
        phy_fifo_rd_en_o = '1;
        if (data_handler_axis_tready && |data_valid_i) begin
          data_c = data_i;
          data_k_c = data_k_i;
          data_handler_axis_tuser = is_tlp_r ? '1 : '0;
          // A beat is the last word_count_r bytes of the previous word, moved
          // down to byte 0, followed by the low bytes of this word.
          data_handler_axis_tdata = data_r >> (8 * (BytesPerTransfer - 32'(word_count_r)));
          data_handler_axis_tkeep = '1;
          data_handler_axis_tvalid = '1;
          data_start_c = '0;
          for (int i = 3; i >= 0; i--) begin
            if (i >= word_count_r) begin
              data_handler_axis_tdata[i*8+:8] = data_i[(i-32'(word_count_r))*8+:8];
            end
          end

          for (int byte_idx = 0; byte_idx < BytesPerTransfer; byte_idx++) begin
            // END or EDB in this word, inside the part this beat takes.
            if (data_k_i[byte_idx] && ((data_i[8*byte_idx+:8] == ENDP) || (data_i[8*byte_idx+:8] == EDB))) begin
              if ((BytesPerTransfer - 32'(word_count_r)) > byte_idx) begin
                is_dllp_c               = '0;
                is_tlp_c                = '0;
                data_handler_axis_tlast = '1;
                // Which Symbol ended the frame, on the same beat as tlast.
                frame_is_edb            = (data_i[8*byte_idx+:8] == EDB);
                // The beat holds word_count_r carried bytes, then the byte_idx
                // bytes of this word below the end Symbol, so tkeep has
                // word_count_r + byte_idx ones. A DLLP's last beat must come
                // out as tkeep 0011b: dllp_handler takes its CRC beat only in
                // that form.
                data_handler_axis_tkeep = 4'((1 << (32'(word_count_r) + byte_idx)) - 1);
                next_state              = ST_CHECK_FRAME;
              end
            end
            // END or EDB among the bytes carried over from the previous word,
            // which the previous beat could not reach. data_start_r excludes
            // the start word, whose bytes below the start Symbol belong to the
            // previous frame. tkeep counts the carried bytes below the end
            // Symbol.
            if (data_k_r[byte_idx] && ((data_r[8*byte_idx+:8] == ENDP)||(data_r[8*byte_idx+:8] == EDB)) && (!data_start_r)) begin
              is_dllp_c = '0;
              is_tlp_c = '0;
              data_handler_axis_tlast = '1;
              // The same flag as in the check above.
              frame_is_edb = (data_r[8*byte_idx+:8] == EDB);
              data_handler_axis_tkeep = (4'hF >> 
              ((BytesPerTransfer - 32'(word_count_r)) + (BytesPerTransfer - byte_idx)));
              next_state = ST_CHECK_FRAME;
            end
          end
          for (int byte_idx = 0; byte_idx < BytesPerTransfer; byte_idx++) begin
            // A start Symbol in this word opens the next packet. These
            // assignments follow the end checks, so they override next_state
            // when one word holds both an end Symbol and the next start.
            if (data_k_i[byte_idx] && (data_i[8*byte_idx+:8] == SDP)) begin
              is_dllp_c    = '1;
              data_start_c = '1;
              next_state   = ST_TX;
              word_count_c = BytesPerTransfer - 1 - byte_idx;
            end
            if (data_k_i[byte_idx] && (data_i[8*byte_idx+:8] == STP)) begin
              is_tlp_c     = '1;
              data_start_c = '1;
              next_state   = ST_TX;
              word_count_c = BytesPerTransfer - 1 - byte_idx;
            end
          end

        end

      end
      // Never entered: no state assigns ST_CHECK_END.
      ST_CHECK_END: begin
        phy_fifo_rd_en_o = '1;
        data_handler_axis_tuser = is_tlp_r ? '1 : '0;
        if (data_handler_axis_tready && |data_valid_i) begin
          data_c     = data_i;
          next_state = ST_CHECK_FRAME;
          is_tlp_c   = '0;
          is_dllp_c  = '0;

          if (data_k_r[0] && data_r[7:0] == ENDP) begin
          end else if (data_k_r[1] && data_r[15:8] == ENDP) begin
          end else if (data_k_r[2] && data_r[23:16] == ENDP) begin
            is_dllp_c                = '1;
            data_c                   = '0;
            data_handler_axis_tdata  = {16'h0, data_r[15:0]};
            data_handler_axis_tkeep  = 4'b0011;
            data_handler_axis_tvalid = '1;
            data_handler_axis_tlast  = '1;
            next_state               = ST_CHECK_FRAME;
            data_valid_c             = data_valid_i;
          end else if (data_k_r[3] && data_r[31:24] == ENDP) begin
            is_dllp_c                = '1;
            data_c                   = '0;
            data_handler_axis_tdata  = {16'h0, data_r[23:8]};
            data_handler_axis_tkeep  = 4'b0011;
            data_handler_axis_tvalid = '1;
            data_handler_axis_tlast  = '1;
            next_state               = ST_CHECK_FRAME;
            data_valid_c             = data_valid_i;
          end
          for (int byte_idx = 0; byte_idx < BytesPerTransfer / 2; byte_idx++) begin
            if (data_k_i[byte_idx] && (data_i[8*byte_idx+:8] == SDP)) begin
              is_dllp_c    = '1;
              data_c       = data_i;
              next_state   = ST_TX;
              data_valid_c = data_valid_i;
              word_count_c = BytesPerTransfer - 1 - byte_idx;
            end else if (data_k_i[byte_idx] && (data_i[8*byte_idx+:8] == STP)) begin
              is_tlp_c     = '1;
              data_c       = data_i;
              next_state   = ST_TX;
              data_valid_c = data_valid_i;
              word_count_c = BytesPerTransfer - 1 - byte_idx;
            end
          end
        end
      end
      // Never entered: no state assigns ST_CHECK_END_GEN3.
      ST_CHECK_END_GEN3: begin
        if (!phy_fifo_empty_i) begin
          phy_fifo_rd_en_o = '1;
          skid_c           = '1;
          word_count_c     = '0;
          next_state       = ST_TX_DLLP;
        end
      end
      // 8 GT/s: no statements, so the state machine never leaves it.
      ST_TX_TLP: begin
      end
      // 8 GT/s: sends the stored word in 32-bit beats and returns to ST_IDLE
      // after the second.
      ST_TX_DLLP: begin
        if (data_handler_axis_tready) begin
          word_count_c             = word_count_r + 1'b1;
          data_c                   = data_r >> 32;
          data_valid_c             = data_valid_r >> 1;
          data_k_c                 = data_k_r >> 4;
          data_handler_axis_tdata  = data_r[31:0];
          data_handler_axis_tkeep  = '1;
          data_handler_axis_tvalid = '1;
          if (skid_r) begin
            data_handler_axis_tdata  = data_r[31:0];
            data_handler_axis_tkeep  = '1;
            data_handler_axis_tvalid = '1;
            data_c                   = data_i;
            data_valid_c             = data_valid_i;
            data_k_c                 = data_k_i;
            skid_c                   = '0;
          end
          if (word_count_r >= 6'd1) begin
            next_state               = ST_IDLE;
            data_c                   = data_r;
            data_valid_c             = data_valid_r;
            data_k_c                 = data_k_r;
            data_handler_axis_tvalid = '1;
          end
        end
      end
      default: begin
      end
    endcase
  end



  // -------------------------------------------------------------------------
  // Output tuser and skid buffer
  // -------------------------------------------------------------------------
  // handler_tuser, not data_handler_axis_tuser, is the skid buffer's tuser.
  // axis_user_demux routes on bit 0 (DLLP) and bit 1 (TLP); dllp2tlp reads
  // bit 2. Bit 0 is set for every frame that is not a TLP. The skid buffer's
  // s_axis_tready is data_handler_axis_tready, which ST_TX and ST_TX_DLLP
  // wait for.
  localparam int UserIsDllp = 0;
  localparam int UserIsTlp  = 1;

  logic [USER_WIDTH-1:0] handler_tuser;
  always_comb begin : output_tuser
    handler_tuser             = '0;
    handler_tuser[UserIsDllp] = !is_tlp_r;
    handler_tuser[UserIsTlp]  = is_tlp_r;
    // Set only on the tlast beat, so the flag travels with the frame it ends.
    handler_tuser[UserIsEdb]  = frame_is_edb;
  end

  // tuser must be wide enough for bit UserIsEdb; without this check a
  // narrower one would drop the flag without an error. frame_symbols has a
  // guard of the same kind.
  initial begin
    if (USER_WIDTH <= UserIsEdb) begin
      $fatal(1, "data_handler: USER_WIDTH=%0d cannot carry tuser bit %0d (the §63 #7i EDB flag). Both stacks pass 5.",
             USER_WIDTH, UserIsEdb);
    end
  end

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
  ) axis_register_inst (
      .clk(clk_i),
      .rst(rst_i),
      .s_axis_tdata(data_handler_axis_tdata),
      .s_axis_tkeep(data_handler_axis_tkeep),
      .s_axis_tvalid(data_handler_axis_tvalid),
      .s_axis_tready(data_handler_axis_tready),
      .s_axis_tlast(data_handler_axis_tlast),
      .s_axis_tuser(handler_tuser),
      .s_axis_tid('0),
      .s_axis_tdest('0),
      .m_axis_tdata(m_dllp_axis_tdata),
      .m_axis_tkeep(m_dllp_axis_tkeep),
      .m_axis_tvalid(m_dllp_axis_tvalid),
      .m_axis_tready(m_dllp_axis_tready),
      .m_axis_tlast(m_dllp_axis_tlast),
      .m_axis_tuser(m_dllp_axis_tuser),
      .m_axis_tid(),
      .m_axis_tdest()
  );

endmodule