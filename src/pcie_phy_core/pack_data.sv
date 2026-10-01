// ---------------------------------------------------------------------------
// pack_data -- gathers PIPE-width beats into DATA_WIDTH words
//
// Purpose
//   Sits between block_alignment and data_handler in phy_receive. Each valid
//   input clock carries pipe_width_i / 8 bytes of lane 0 (two at Gen1, where
//   lane_management sets the PIPE width to 16). They are appended, lowest
//   byte first, to a DATA_WIDTH / 8 byte word, and data_valid_o pulses for
//   one clock when the word is full. K flags travel with their bytes.
//
// Interfaces
//   Input         data_i, data_k_i, data_valid_i, sync_header_i: from
//                 block_alignment; only lane 0's low pipe_width_i / 8 bytes
//                 are read.
//   Output        data_o, data_k_o, data_valid_o, sync_header_o: the gathered
//                 word. data_o and data_k_o show the word as it fills; it is
//                 complete only while data_valid_o is high.
//   Control       phy_link_up_i: no byte is gathered while it is low.
//                 pipe_width_i, num_active_lanes_i: bytes per input beat.
//   Unused        lane_reverse_i, curr_data_rate_i; fifo_wr_o is constant 0.
//
// Clock and reset
//   clk_i only (pipe_rx_usr_clk_i in phy_receive). rst_i is synchronous and
//   active high; it empties the word and returns Q.state to ST_IDLE.
//
// Limitations
//   Lane 0 only: the fill count advances by num_active_lanes_i lanes' bytes
//   but only lane 0's bytes are copied, so with more than one active lane the
//   word carries stale bytes. No word completes while num_active_lanes_i is
//   0. Only ST_IDLE is ever used: nothing writes D.state.
// ---------------------------------------------------------------------------
module pack_data
  import pcie_phy_pkg::*;
#(
    // Bits per gathered word; phy_receive passes 32.
    parameter int DATA_WIDTH    = 32,
    parameter int MAX_NUM_LANES = 16
) (
    // ---- clock, reset and control ------------------------------------------
    input  logic                                           clk_i,
    input  logic                                           rst_i,
    input  logic                                           phy_link_up_i,
    input  logic                                           lane_reverse_i,
    input  rate_speed_e                                    curr_data_rate_i,
    // ---- input beat, from block_alignment ----------------------------------
    input  logic        [( MAX_NUM_LANES* DATA_WIDTH)-1:0] data_i,
    input  logic        [               MAX_NUM_LANES-1:0] data_valid_i,
    input  logic        [           (4*MAX_NUM_LANES)-1:0] data_k_i,
    input  logic        [           (2*MAX_NUM_LANES)-1:0] sync_header_i,
    // ---- gathered word, to data_handler ------------------------------------
    output logic        [( MAX_NUM_LANES* DATA_WIDTH)-1:0] data_o,
    output logic        [               MAX_NUM_LANES-1:0] data_valid_o,
    output logic        [           (4*MAX_NUM_LANES)-1:0] data_k_o,
    output logic        [           (2*MAX_NUM_LANES)-1:0] sync_header_o,
    input  logic        [                             5:0] pipe_width_i,
    output logic                                           fifo_wr_o,
    input  logic        [                             5:0] num_active_lanes_i
);



  // Only BytesPerTransfer is used.
  localparam int PipeWidthGen1 = 8;
  localparam int PipeWidthGen2 = 16;
  localparam int PipeWidthGen3 = 16;
  localparam int PipeWidthGen4 = 32;
  localparam int PipeWidthGen5 = 32;
  localparam int BytesPerTransfer = DATA_WIDTH / 8;
  localparam int MaxWordsPerTransaction = 512 / DATA_WIDTH;
  localparam int BytesPerTransaction = 512 / 8;

  // Only ST_IDLE is used: Q.state leaves reset as ST_IDLE and D.state is
  // never written.
  typedef enum logic [4:0] {
    ST_IDLE,
    ST_SEND_DATA,
    ST_GEN3_TLP,
    ST_GEN3_DLLP,
    ST_LAST_DATA
  } pack_st_e;


  // Declared and never used.
  logic        is_ordered_set;
  logic        is_data;
  logic        ready_out;


  // The fill count's advance per valid input clock: pipe_width_i / 8 bytes
  // for each active lane, though only lane 0's bytes are copied.
  logic [15:0] bytes_per_packet;


  // Assigned and never read.
  logic        end_packet;
  logic [31:0] byte_shift;


  // count is the fill level of the word in bytes. word_count and
  // tlp_byte_count are only ever reset or copied from Q, and fifo_wr is
  // cleared on every clock.
  typedef struct packed {
    pack_st_e                                state;
    logic [( MAX_NUM_LANES* DATA_WIDTH)-1:0] data;
    logic [MAX_NUM_LANES-1:0]                data_valid;
    logic [(4*MAX_NUM_LANES)-1:0]            data_k;
    logic [(2*MAX_NUM_LANES)-1:0]            sync_header;
    logic [5:0]                              word_count;
    logic [5:0]                              tlp_byte_count;
    logic                                    fifo_wr;
    logic [3:0]                              count;
  } pack_data_t;


  pack_data_t D;
  pack_data_t Q;




  always_ff @(posedge clk_i) begin : main_seq_block
    if (rst_i) begin
      Q <= '{state: ST_IDLE, default: 'd0};
    end else begin
      Q <= D;
    end
  end




  always_comb begin : block_alignment_combinational_logic
    D                = Q;
    end_packet       = '0;
    bytes_per_packet = num_active_lanes_i * ((pipe_width_i) >> 3);
    byte_shift       = (bytes_per_packet * Q.word_count);
    D.data_valid     = '0;
    D.fifo_wr        = '0;
    case (Q.state)
      ST_IDLE: begin
        if (phy_link_up_i && (|data_valid_i)) begin
          D.count = Q.count + bytes_per_packet;
          // This beat's bytes go in at byte offset Q.count; the word's earlier
          // bytes are kept by the D = Q default.
          for (int byte_idx = 0; byte_idx < BytesPerTransfer; byte_idx++) begin
            if (byte_idx < (pipe_width_i >> 3)) begin
              D.data[8*(byte_idx+Q.count)+:8] = data_i[8*byte_idx+:8];
              D.data_k[byte_idx+Q.count]    = data_k_i[byte_idx];
              D.sync_header         = sync_header_i;
            end
          end
          // The word is full: data_valid_o pulses with the registered word,
          // and the next beat starts a new word at byte 0.
          if ((Q.count + bytes_per_packet) >= BytesPerTransfer) begin
            D.count = '0;
            D.data_valid = '1;
          end
        end
      end
      default: begin
      end
    endcase
  end



  assign sync_header_o = Q.sync_header;
  assign data_valid_o  = Q.data_valid;
  assign data_k_o      = Q.data_k;
  assign data_o        = Q.data;
  assign fifo_wr_o     = Q.fifo_wr;
endmodule
