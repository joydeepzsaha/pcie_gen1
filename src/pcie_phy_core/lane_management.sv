// ---------------------------------------------------------------------------
// lane_management -- puts framed packets and Ordered Sets onto the lanes
//
// Purpose
//   Takes two streams, framed TLPs and DLLPs from frame_symbols and Ordered
//   Sets from os_generator (Logical Idle included), each through a FIFO in
//   phy_transmit, and sends one at a time to the per-lane scramblers there:
//   pipe_width_o/8 bytes per lane per clock, each with a K flag. A packet is
//   never interrupted. In ST_IDLE a real Ordered Set wins over a waiting
//   packet; at the end of an Ordered Set the test reads a later beat, so a
//   TS1 or TS2 can lose to one (see Limitations). Logical Idle gives way to
//   a waiting packet in ST_IDLE and at the end of an Ordered Set, but not at
//   the end of a packet. The two TX states hand over to each other directly,
//   so a switch between streams costs no clock without data.
//
// Interfaces
//   Packets       s_dllp_axis_*: framed TLPs and DLLPs; tuser is the per-byte
//                 K mask. Lane l reads tdata from bit lane*32.
//   Ordered Sets  s_phy_axis_*: one Ordered Set per lane; tuser is the K mask,
//                 lane l at [USER_WIDTH*l +: USER_WIDTH].
//   Lanes         data_out_o, d_k_out_o: 32 bits and 4 K flags per lane, zero
//                 while data_valid_o is low. num_active_lanes_i: lanes from
//                 it up stay invalid.
//   Width         pipe_width_o: PIPE bits per lane per clock, 16.
//   Rate          curr_data_rate_i: selects the pipe width (see Limitations).
//   Unused        phy_link_up_i, lane_reverse_i. sync_header_o and
//                 start_block_o are never driven.
//
// Clock and reset
//   clk_i only; phy_transmit connects pipe_tx_usr_clk_i. rst_i is synchronous
//   and active high. A change of pipe_width_c also resets main_seq_block's
//   first branch, but not axis_register_inst.
//
// Limitations
//   Gen1 and Gen2 only. At gen3 and above pipe_width_c is 32, and the reset
//   that a width change triggers reloads pipe_width_r with PipeWidthGen1, so
//   the reset repeats on every clock: curr_state stays in ST_IDLE and no lane
//   is ever valid. Packet bytes are not striped across lanes: lane l reads
//   s_dllp_axis_tdata from bit lane*32, past the end of the DATA_WIDTH-bit
//   bus for every lane but lane 0 when DATA_WIDTH is 32. At the end of an
//   Ordered Set, phy_next_is_ordered_set reads the beat after the next one
//   while the Ordered Set FIFO has beats; for a TS1 or TS2 that is Symbols
//   4-7, with no K Symbol, so a waiting packet can go out before it.
//
// Structure
//   Ordered Set or Logical Idle   the K-flag test the arbitration uses
//   Registers                     main_seq_block
//   Pipe width                    data_rate_block
//   Gen3 sync header              sync_header_combo_block; no output
//   Bytes per packet              calc_bytes_per_packet; result unread
//   Lane state machine            lane_data_sync
//   Unread blocks                 set_sync_fifo_ready, flatten_decrambler
//   Ordered Set input register    axis_register_inst
//   Outputs                       ready and lane outputs
//
// References
//   PCIe Base Spec r2.1, §4.2.2
//   PCIe Base Spec r2.1, §4.2.4.1
//   PCIe Base Spec r2.1, §4.2.7.1
//   PCIe Base Spec r3.0, §4.2.2.1
// ---------------------------------------------------------------------------
module lane_management
  import pcie_phy_pkg::*;
#(
    parameter int DATA_WIDTH    = 32,
    parameter int STRB_WIDTH    = DATA_WIDTH / 8,
    parameter int KEEP_WIDTH    = STRB_WIDTH,
    parameter int USER_WIDTH    = 4,
    parameter int MAX_NUM_LANES = 16
) (
    // ---- clock and reset ----
    input  logic                  clk_i,
    input  logic                  rst_i,
    input  logic                  phy_link_up_i,
    // ---- framed packets from frame_symbols ----
    input  logic [DATA_WIDTH-1:0] s_dllp_axis_tdata,
    input  logic [KEEP_WIDTH-1:0] s_dllp_axis_tkeep,
    input  logic                  s_dllp_axis_tvalid,
    input  logic                  s_dllp_axis_tlast,
    input  logic [USER_WIDTH-1:0] s_dllp_axis_tuser,
    output logic                  s_dllp_axis_tready,

    // ---- Ordered Sets from os_generator ----
    input  logic [(DATA_WIDTH*MAX_NUM_LANES)-1:0] s_phy_axis_tdata,
    input  logic [(KEEP_WIDTH*MAX_NUM_LANES)-1:0] s_phy_axis_tkeep,
    input  logic                                  s_phy_axis_tvalid,
    input  logic                                  s_phy_axis_tlast,
    input  logic [(USER_WIDTH*MAX_NUM_LANES)-1:0] s_phy_axis_tuser,
    output logic                                  s_phy_axis_tready,

    // ---- lanes ----
    input  logic                                           lane_reverse_i,
    input  rate_speed_e                                    curr_data_rate_i,
    output logic        [( MAX_NUM_LANES* DATA_WIDTH)-1:0] data_out_o,
    output logic        [               MAX_NUM_LANES-1:0] data_valid_o,
    output logic        [           (4*MAX_NUM_LANES)-1:0] d_k_out_o,
    output logic        [           (2*MAX_NUM_LANES)-1:0] sync_header_o,
    output logic        [                             5:0] pipe_width_o,
    output logic        [               MAX_NUM_LANES-1:0] start_block_o,
    input  logic        [                             5:0] num_active_lanes_i
);



  // PIPE data width per lane, in bits, for each data rate.
  localparam int PipeWidthGen1 = 16;
  localparam int PipeWidthGen2 = 16;
  localparam int PipeWidthGen3 = 32;
  localparam int PipeWidthGen4 = 32;
  localparam int PipeWidthGen5 = 32;
  localparam int BytesPerTransfer = DATA_WIDTH / 8;
  localparam int MaxWordsPerTransaction = 512 / DATA_WIDTH;



  // States of lane_data_sync. Only ST_IDLE, ST_LANE_MNGT_TX_DATA and
  // ST_LANE_MNGT_TX_PHY are ever entered.
  typedef enum logic [4:0] {
    ST_IDLE,
    ST_LANE_MNGT_PHY,
    ST_LANE_MNGT_DATA,
    ST_LANE_MNGT_TX_PHY,
    ST_LANE_MNGT_TX_DATA,
    ST_LANE_MNGT_TX_GEN1,
    ST_LANE_MNGT_TX_GEN2,
    ST_LANE_MNGT_TX_GEN3,
    ST_LANE_MNGT_TX_GEN4,
    ST_LANE_MNGT_TX_GEN5
  } lane_mngt_state_e;


  lane_mngt_state_e                                    curr_state;
  lane_mngt_state_e                                    next_state;
  logic             [                             4:0] sync_width_c;
  logic             [                             4:0] sync_width_r;


  logic             [                             4:0] sync_count_c;
  logic             [                             4:0] sync_count_r;
  logic             [                             4:0] sync_width;
  logic             [                             4:0] sync_count;
  logic             [                             4:0] axis_sync_c;
  logic             [                             4:0] axis_sync_r;
  logic             [                             5:0] pipe_width_c;
  logic             [                             5:0] pipe_width_r;
  logic             [                             5:0] pkt_count_c;
  logic             [                             5:0] pkt_count_r;

  logic             [                             5:0] lanes_count_c;
  logic             [                             5:0] lanes_count_r;
  logic             [                             5:0] byte_count_c;
  logic             [                             5:0] byte_count_r;
  logic             [                             5:0] bytes_sent_c;
  logic             [                             5:0] bytes_sent_r;

  logic             [                             5:0] word_count_c;
  logic             [                             5:0] word_count_r;

  logic             [                             5:0] fifo_word_count_c;
  logic             [                             5:0] fifo_word_count_r;
  logic             [( MAX_NUM_LANES* DATA_WIDTH)-1:0] data_out_c;
  logic             [( MAX_NUM_LANES* DATA_WIDTH)-1:0] data_out_r;
  logic             [               MAX_NUM_LANES-1:0] data_valid_c;
  logic             [               MAX_NUM_LANES-1:0] data_valid_r;
  logic             [           (4*MAX_NUM_LANES)-1:0] d_k_out_c;
  logic             [           (4*MAX_NUM_LANES)-1:0] d_k_out_r;

  logic                                                is_ordered_set;
  logic                                                is_data;
  logic                                                ready_out;

  logic             [                             1:0] sync_header_c            [MAX_NUM_LANES];
  logic             [                             1:0] sync_header_r            [MAX_NUM_LANES];

  logic             [               MAX_NUM_LANES-1:0] block_start_c;
  logic             [               MAX_NUM_LANES-1:0] block_start_r;

  logic             [                           511:0] data_in_c;
  logic             [                           511:0] data_in_r;
  logic             [                    (512/8) -1:0] data_k_in_c;
  logic             [                    (512/8) -1:0] data_k_in_r;
  logic             [                             7:0] byte_start_index_c;
  logic             [                             7:0] byte_start_index_r;
  logic             [                             7:0] lane_start_index_c;
  logic             [                             7:0] lane_start_index_r;
  logic             [                             7:0] input_byte_start_index_c;
  logic             [                             7:0] input_byte_start_index_r;
  logic                                                is_phy_c;
  logic                                                is_phy_r;
  logic                                                is_dllp_c;
  logic                                                is_dllp_r;
  logic                                                replace_lane_c;
  logic                                                replace_lane_r;
  logic                                                complete_c;
  logic                                                complete_r;
  logic             [                            31:0] lane_data;
  logic             [                            31:0] data_out;
  logic             [                            31:0] bytes_per_packet;
  logic             [                             3:0] data_k_out;
  logic             [                             7:0] lane_idx;
  logic             [                             7:0] lane_shift_idx;
  logic             [                             7:0] pipewidth_shift_idx;
  logic             [                         (4)-1:0] temp_d_k;
  logic             [                             7:0] current_byte;

  logic                                                fifo_full;
  logic                                                fifo_empty;


  logic             [           (4*MAX_NUM_LANES)-1:0] d_k_out_temp;
  logic             [           (2*MAX_NUM_LANES)-1:0] sync_header_temp;


  logic                                                read_en_r;
  logic                                                read_en_c;
  logic             [( MAX_NUM_LANES* DATA_WIDTH)-1:0] temp_data_out;



  logic             [  (DATA_WIDTH*MAX_NUM_LANES)-1:0] fifo_phy_axis_tdata;
  logic             [  (KEEP_WIDTH*MAX_NUM_LANES)-1:0] fifo_phy_axis_tkeep;
  logic                                                fifo_phy_axis_tvalid;
  logic                                                fifo_phy_axis_tlast;
  logic             [  (USER_WIDTH*MAX_NUM_LANES)-1:0] fifo_phy_axis_tuser;
  logic                                                fifo_phy_axis_tready;

  localparam int PcieDataSize = $size(
      data_valid_r
  ) + $size(
      data_out_r
  ) + $size(
      block_start_r
  ) + $size(
      d_k_out_r
  ) + $size(
      sync_header_r
  );


  assign is_ordered_set = fifo_phy_axis_tvalid & fifo_phy_axis_tready;
  assign is_data        = s_dllp_axis_tvalid & s_dllp_axis_tready;

  // -------------------------------------------------------------------------
  // Ordered Set or Logical Idle
  // -------------------------------------------------------------------------
  // os_generator sends Logical Idle on the same stream as TS and SKP Ordered
  // Sets. Logical Idle is the data Symbol 00h with no K Symbol (PCIe Base Spec
  // r2.1, §4.2.2), and every Ordered Set starts with a COM, a K Symbol, so the
  // first beat of a real Ordered Set has a tuser bit set and a Logical Idle
  // beat has none; nor do the later beats of a TS1 or TS2. The link sends
  // Logical Idle only when it has no packet, so it gives way to a waiting
  // packet; a real Ordered Set does not, because a scheduled SKP Ordered Set
  // must go out at the next packet or Ordered Set boundary (PCIe Base Spec
  // r2.1, §4.2.7.1). ST_IDLE tests the head beat (fifo_phy_axis_*), the first
  // beat of a set. ST_LANE_MNGT_TX_PHY's exit tests s_phy_axis_*, the beat
  // after the next while the Ordered Set FIFO has beats (see Ordered Set
  // input register).
  logic phy_head_is_ordered_set;
  logic phy_next_is_ordered_set;
  assign phy_head_is_ordered_set = |fifo_phy_axis_tuser;
  assign phy_next_is_ordered_set = |s_phy_axis_tuser;


  // -------------------------------------------------------------------------
  // Registers
  // -------------------------------------------------------------------------
  // The registers in the first branch reset on rst_i and on a change of pipe
  // width (pipe_width_c != pipe_width_r); the rest load every clock. The reset
  // reloads pipe_width_r with PipeWidthGen1, whatever the new width.
  always_ff @(posedge clk_i) begin : main_seq_block
    if (rst_i || (pipe_width_c != pipe_width_r)) begin
      pipe_width_r      <= PipeWidthGen1;
      sync_count_r      <= '0;
      sync_width_r      <= '0;
      d_k_out_r         <= '{default: 'd0};
      axis_sync_r       <= '0;
      data_valid_r      <= '0;
      block_start_r     <= '0;
      read_en_r         <= '0;
      fifo_word_count_r <= '0;
      curr_state        <= ST_IDLE;
    end else begin
      block_start_r     <= block_start_c;
      sync_count_r      <= sync_count_c;
      sync_width_r      <= sync_width_c;
      d_k_out_r         <= d_k_out_c;
      axis_sync_r       <= axis_sync_c;
      data_valid_r      <= data_valid_c;
      fifo_word_count_r <= fifo_word_count_c;
      read_en_r         <= read_en_c;
      pipe_width_r      <= pipe_width_c;
      curr_state        <= next_state;
    end
    data_in_r                <= data_in_c;
    is_phy_r                 <= is_phy_c;
    is_dllp_r                <= is_dllp_c;
    pkt_count_r              <= pkt_count_c;
    word_count_r             <= word_count_c;
    lane_start_index_r       <= lane_start_index_c;
    byte_start_index_r       <= byte_start_index_c;
    replace_lane_r           <= replace_lane_c;
    complete_r               <= complete_c;
    sync_header_r            <= sync_header_c;
    data_out_r               <= data_out_c;
    data_k_in_r              <= data_k_in_c;
    byte_count_r             <= byte_count_c;
    lanes_count_r            <= lanes_count_c;
    bytes_sent_r             <= bytes_sent_c;
    input_byte_start_index_r <= input_byte_start_index_c;
  end


  // -------------------------------------------------------------------------
  // Pipe width
  // -------------------------------------------------------------------------
  // PipeWidthGen1 to PipeWidthGen5 by data rate. sync_width_c sets the Gen3
  // sync-header period, which only sync_header_combo_block reads.
  always_comb begin : data_rate_block
    pipe_width_c = pipe_width_r;
    sync_width_c = sync_width_r;
    case (curr_data_rate_i)
      gen1: begin
        pipe_width_c = PipeWidthGen1;
      end
      gen2: begin
        pipe_width_c = PipeWidthGen2;
      end
      gen3: begin
        pipe_width_c = PipeWidthGen3;
        sync_width_c = 5'd8;
      end
      gen4: begin
        pipe_width_c = PipeWidthGen4;
        sync_width_c = 5'd8;
      end
      gen5: begin
        pipe_width_c = PipeWidthGen5;
        sync_width_c = 5'd4;
      end
      default: begin
        pipe_width_c = PipeWidthGen1;
        sync_width_c = 5'd16;
      end
    endcase
  end

  // -------------------------------------------------------------------------
  // Gen3 sync header
  // -------------------------------------------------------------------------
  // At gen3 and above: block_start_c and a per-lane sync header, 10b for an
  // Ordered Set block and 01b for a data block. That is the reverse of PCIe
  // Base Spec r3.0, §4.2.2.1, where 10b marks a Data Block and 01b an Ordered
  // Set Block. Neither reaches an output: nothing assigns start_block_o or
  // sync_header_o.
  always_comb begin : sync_header_combo_block
    sync_count_c  = sync_count_r;
    sync_header_c = sync_header_r;
    block_start_c = block_start_r;
    if (curr_data_rate_i >= gen3) begin
      block_start_c = data_valid_c;
      //increment count only if valid transaction
      if (is_phy_r || is_dllp_r) begin
        sync_count_c = sync_count_r >= sync_width_r ? '0 : sync_count_r + 1'b1;
      end
    end else begin
      sync_count_c = '0;
    end
    //per lane sync header output
    for (int i = 0; i < MAX_NUM_LANES; i++) begin
      if (sync_count_r == '0 && (curr_data_rate_i >= gen3)) begin
        sync_header_c[i] = is_phy_r ? 2'b10 : 2'b01;
      end
    end
  end

  // -------------------------------------------------------------------------
  // Bytes per packet
  // -------------------------------------------------------------------------
  // bytes_per_packet from num_active_lanes_i, for power-of-two lane counts
  // only. Nothing reads it.
  always_comb begin : calc_bytes_per_packet
    bytes_per_packet = '0;
    for (int i = 0; i < 8; i++) begin
      if (num_active_lanes_i == (1 << i)) begin
        bytes_per_packet = (pipe_width_r) << i;
      end
    end
  end

  // -------------------------------------------------------------------------
  // Lane state machine
  // -------------------------------------------------------------------------
  // A TX state sends pipe_width_r/8 bytes per active lane per clock from byte
  // byte_count_r of the input beat, so a 4-byte beat lasts two clocks and
  // ready_out takes it on the second. Only the TX states set data_valid_c.
  //   state                 does            exit
  //   ST_IDLE               sends nothing   Ordered Set beat, unless Logical Idle with a
  //                                         packet waiting -> TX_PHY; else packet -> TX_DATA
  //   ST_LANE_MNGT_TX_DATA  sends a packet  last beat: Ordered Set beat waiting -> TX_PHY;
  //                                         else -> ST_IDLE
  //   ST_LANE_MNGT_TX_PHY   sends Ordered   last beat: packet waiting and s_phy_axis_* beat
  //                         Sets            Logical Idle -> TX_DATA; s_phy_axis_* beat
  //                                         waiting -> stay; else -> ST_IDLE
  always_comb begin : lane_data_sync
    d_k_out_c                = d_k_out_r;
    data_k_in_c              = data_k_in_r;
    data_out_c               = data_out_r;
    data_valid_c             = '0;
    data_in_c                = data_in_r;
    is_dllp_c                = is_dllp_r;
    is_phy_c                 = is_phy_r;
    lane_start_index_c       = lane_start_index_r;
    pkt_count_c              = pkt_count_r;
    word_count_c             = word_count_r;
    next_state               = curr_state;
    byte_start_index_c       = byte_start_index_r;
    replace_lane_c           = replace_lane_r;
    input_byte_start_index_c = input_byte_start_index_r;
    ready_out                = '0;
    complete_c               = '0;
    data_out                 = '0;
    lane_idx                 = '0;
    data_k_out               = '0;
    temp_d_k                 = '0;
    lane_data                = '0;
    pipewidth_shift_idx      = (pipe_width_r >> 3) - 1;
    lane_shift_idx           = (num_active_lanes_i >> 1);
    current_byte             = pipewidth_shift_idx - byte_start_index_r;
    byte_count_c             = byte_count_r;
    lanes_count_c            = lanes_count_r;
    bytes_sent_c             = bytes_sent_r;
    case (curr_state)
      ST_IDLE: begin
        // A real Ordered Set, or any Ordered Set beat with no packet waiting,
        // wins; Logical Idle gives way to a waiting packet.
        if (fifo_phy_axis_tvalid && (phy_head_is_ordered_set || !s_dllp_axis_tvalid)) begin
          pkt_count_c        = '0;
          word_count_c       = '0;
          lane_start_index_c = '0;
          byte_start_index_c = '0;
          is_dllp_c          = '0;
          is_phy_c           = '1;
          next_state         = ST_LANE_MNGT_TX_PHY;
          byte_count_c       = '0;
          lanes_count_c      = '0;
          replace_lane_c     = '0;
          bytes_sent_c       = '0;
          data_out_c         = '0;
          for (int i = 0; i < MAX_NUM_LANES; i++) begin
            data_in_c[8*i+:8]  = curr_data_rate_i >= gen3 ? 8'hf7 : '0;
            data_out_c[8*i+:8] = curr_data_rate_i >= gen3 ? 8'hf7 : '0;
          end
        end else if (s_dllp_axis_tvalid) begin
          pkt_count_c              = '0;
          word_count_c             = '0;
          lane_start_index_c       = '0;
          byte_start_index_c       = '0;
          next_state               = ST_LANE_MNGT_TX_DATA;
          is_dllp_c                = '1;
          is_phy_c                 = '0;
          replace_lane_c           = '0;
          byte_count_c             = '0;
          data_out_c               = '0;
          lanes_count_c            = '0;
          bytes_sent_c             = '0;
          input_byte_start_index_c = '0;
          for (int i = 0; i < MAX_NUM_LANES * BytesPerTransfer; i++) begin
            data_in_c[8*i+:8]  = curr_data_rate_i >= gen3 ? 8'hf7 : '0;
            data_out_c[8*i+:8] = curr_data_rate_i >= gen3 ? 8'hf7 : '0;
          end
        end
      end
      ST_LANE_MNGT_TX_DATA: begin
        if (s_dllp_axis_tvalid) begin
          byte_count_c = byte_count_r + ((pipe_width_r >> 3));
          for (logic [7:0] lane = 0; lane < MAX_NUM_LANES; lane = lane + 1) begin
            if (lane < num_active_lanes_i) begin
              data_valid_c[lane]      = '1;
              d_k_out_c[lane*4+:4]    = '0;
              data_out_c[lane*32+:32] = '0;
              for (int byte_ = 0; byte_ < DATA_WIDTH / 8; byte_++) begin
                if (byte_ < (pipe_width_r >> 3)) begin
                  data_out_c[(lane*32)+(byte_*8)+:8]   =
                  s_dllp_axis_tdata[(lane*32)+((byte_+byte_count_r)*8)+:8];
                  d_k_out_c[(lane*4)+(byte_*1)+:1] = s_dllp_axis_tuser[(lane*4)+((byte_+byte_count_r)*1)+:1];
                end
              end
            end
          end
          if ((byte_count_r + ((pipe_width_r >> 3))) >= (DATA_WIDTH / 8) - 1) begin
            byte_count_c = '0;
            ready_out = '1;
            if (s_dllp_axis_tlast) begin
              // At the end of a packet an Ordered Set beat is taken directly;
              // through ST_IDLE, the next clock would send nothing.
              if (fifo_phy_axis_tvalid) begin
                next_state         = ST_LANE_MNGT_TX_PHY;
                is_phy_c           = '1;
                is_dllp_c          = '0;
                lane_start_index_c = '0;
                byte_start_index_c = '0;
                replace_lane_c     = '0;
                lanes_count_c      = '0;
                bytes_sent_c       = '0;
                // data_out_c and data_in_c are not cleared: this clock still
                // sends the packet's last word.
              end else begin
                next_state = ST_IDLE;
              end
              complete_c   = '1;
              pkt_count_c  = '0;
              word_count_c = '0;
            end
          end
        end
      end
      ST_LANE_MNGT_TX_PHY: begin
        if (fifo_phy_axis_tvalid) begin
          byte_count_c = byte_count_r + (pipe_width_r >> 3);
          for (logic [7:0] lane = 0; lane < MAX_NUM_LANES; lane = lane + 1) begin
            if (lane < num_active_lanes_i) begin
              data_valid_c[lane]      = '1;
              d_k_out_c[lane*4+:4]    = '0;
              data_out_c[lane*32+:32] = '0;
              for (int byte_ = 0; byte_ < DATA_WIDTH / 8; byte_++) begin
                if (byte_ < (pipe_width_r >> 3)) begin
                  data_out_c[(lane*32)+(byte_*8)+:8]   =
                  fifo_phy_axis_tdata[(lane*32)+((byte_+byte_count_r)*8)+:8];
                  // Each lane takes its K flags from its own tuser slice,
                  // because Symbols 1 and 2 may be PAD on some lanes only. The
                  // source stride is USER_WIDTH (5 in phy_transmit), where
                  // os_generator packs lane l's mask; the destination stride is
                  // 4, the Symbols per lane of d_k_out_o. The two strides differ
                  // on purpose; with one lane both give the same index, so only
                  // a multi-lane bench can tell them apart.
                  d_k_out_c[(lane*4)+(byte_*1)+:1] =
                      fifo_phy_axis_tuser[(lane*USER_WIDTH) + byte_ + byte_count_r];
                end
              end
            end
          end
          if ((byte_count_r + (pipe_width_r >> 3)) >= (DATA_WIDTH / 8) - 1) begin
            byte_count_c = '0;
            ready_out = '1;
            if (fifo_phy_axis_tlast) begin
              // At the end of an Ordered Set a waiting packet is taken
              // directly when the beat at s_phy_axis_* is Logical Idle; through
              // ST_IDLE, the next clock would send nothing. test_7j2_idle checks
              // that valid never drops under continuous Logical Idle with a
              // packet offered. The set-up below is ST_IDLE's packet arm.
              if (s_dllp_axis_tvalid && !phy_next_is_ordered_set) begin
                next_state               = ST_LANE_MNGT_TX_DATA;
                is_dllp_c                = '1;
                is_phy_c                 = '0;
                lane_start_index_c       = '0;
                byte_start_index_c       = '0;
                input_byte_start_index_c = '0;
                replace_lane_c           = '0;
                lanes_count_c            = '0;
                bytes_sent_c             = '0;
                // Unlike ST_IDLE's arm, data_out_c and data_in_c are not cleared:
                // this clock still sends the Ordered Set's last word, which a
                // clear would zero. ST_LANE_MNGT_TX_DATA rewrites data_out_c for
                // every active lane on its first clock.
              end else if (s_phy_axis_tvalid) begin
                // An Ordered Set beat waits at s_phy_axis_*: stay, so
                // back-to-back Ordered Sets go out without a gap.
              end else begin
                next_state = ST_IDLE;
              end
              complete_c   = '1;
              pkt_count_c  = '0;
              word_count_c = '0;
            end
          end

        end
      end
      default: begin
      end
    endcase
  end

  // -------------------------------------------------------------------------
  // Unread blocks
  // -------------------------------------------------------------------------
  // set_sync_fifo_ready computes read_en_c and fifo_word_count_c, and
  // flatten_decrambler packs sync_header_r into sync_header_temp. Only
  // set_sync_fifo_ready reads read_en_r and fifo_word_count_r, and nothing
  // reads sync_header_temp.
  always_comb begin : set_sync_fifo_ready
    read_en_c         = read_en_r;
    fifo_word_count_c = fifo_word_count_r;
    if (complete_c) begin
      fifo_word_count_c = word_count_r;
      read_en_c = '1;
    end else if (read_en_r) begin
      fifo_word_count_c = fifo_word_count_r - 1'b1;
      if (fifo_word_count_r <= 32'b0) begin
        read_en_c = '0;
      end
    end
  end

  always_comb begin : flatten_decrambler
    for (int i = 0; i < MAX_NUM_LANES; i++) begin
      sync_header_temp[2*i+:2] = sync_header_r[i];
    end
  end

  // -------------------------------------------------------------------------
  // Ordered Set input register
  // -------------------------------------------------------------------------
  // A skid buffer between phy_transmit's Ordered Set FIFO (s_phy_axis_*) and
  // the state machine (fifo_phy_axis_*). It holds up to two beats: the head in
  // its output register and the next in its temp register. In
  // ST_LANE_MNGT_TX_PHY the head is taken every second clock, and on the clock
  // after each take the buffer refills its temp register from the input port.
  // So while the FIFO has beats, the input port shows the beat after the next
  // one when that state's exit tests it. For a following TS1 or TS2 that is
  // Symbols 4-7, which carry no K Symbol, so a waiting packet is taken ahead
  // of the Training Sequence, which PCIe Base Spec r2.1, §4.2.4.1 lets only
  // SKP Ordered Sets and EIEOSs interrupt.
  axis_register #(
      .DATA_WIDTH(DATA_WIDTH * MAX_NUM_LANES),
      .KEEP_ENABLE('1),
      .KEEP_WIDTH(KEEP_WIDTH * MAX_NUM_LANES),
      .LAST_ENABLE('1),
      .ID_ENABLE('0),
      .ID_WIDTH(1),
      .DEST_ENABLE('0),
      .DEST_WIDTH(1),
      .USER_ENABLE('1),
      .USER_WIDTH(USER_WIDTH * MAX_NUM_LANES),
      .REG_TYPE(SkidBuffer)
  ) axis_register_inst (
      .clk          (clk_i),
      .rst          (rst_i),
      .s_axis_tdata (s_phy_axis_tdata),
      .s_axis_tkeep (s_phy_axis_tkeep),
      .s_axis_tvalid(s_phy_axis_tvalid),
      .s_axis_tready(s_phy_axis_tready),
      .s_axis_tlast (s_phy_axis_tlast),
      .s_axis_tuser (s_phy_axis_tuser),
      .s_axis_tid   ('0),
      .s_axis_tdest ('0),
      .m_axis_tdata (fifo_phy_axis_tdata),
      .m_axis_tkeep (fifo_phy_axis_tkeep),
      .m_axis_tvalid(fifo_phy_axis_tvalid),
      .m_axis_tready(fifo_phy_axis_tready),
      .m_axis_tlast (fifo_phy_axis_tlast),
      .m_axis_tuser (fifo_phy_axis_tuser),
      .m_axis_tid   (),
      .m_axis_tdest ()
  );


  // -------------------------------------------------------------------------
  // Outputs
  // -------------------------------------------------------------------------
  // ready_out goes to the stream the state machine is on. data_valid_o is
  // data_valid_r, high only on clocks that carry Symbols: gen1_scramble
  // advances its LFSR per valid clock, so a clock without Symbols must not be
  // marked valid. data_out_o and d_k_out_o are zero on such clocks.
  assign s_dllp_axis_tready   = ready_out & is_dllp_r;
  assign fifo_phy_axis_tready = ready_out & is_phy_r;

  assign data_valid_o         = data_valid_r;
  assign data_out_o           = data_valid_r ? data_out_r : '0;
  assign d_k_out_o            = data_valid_r ? d_k_out_r : '0;
  assign pipe_width_o         = pipe_width_r;

endmodule
