// ---------------------------------------------------------------------------
// block_alignment -- four-clock delay line for the descrambled lane stream
//
// Purpose
//   Sits between the per-lane descramblers and pack_data in phy_receive.
//   Despite its name it performs no block or Symbol alignment and no
//   lane-to-lane de-skew: every input beat leaves NumPipelines (4) clocks
//   later, unchanged, with its data, K flags and valid bit kept together.
//   A beat that enters while phy_link_up_i is low leaves with valid low.
//
// Interfaces
//   Input         data_i, data_k_i, data_valid_i, sync_header_i: per lane, one
//                 DATA_WIDTH word, four K flags, a valid bit and a sync header.
//   Output        data_o, data_k_o, data_valid_o, sync_header_o: the same
//                 beat four clocks later (sync_header_o: one clock later).
//   Control       phy_link_up_i: gates data_valid_i into the first stage.
//   Unused        lane_reverse_i, curr_data_rate_i; pipe_width_i and
//                 num_active_lanes_i feed only intermediates nothing reads.
//
// Clock and reset
//   clk_i only (pipe_rx_usr_clk_i in phy_receive). rst_i is synchronous and
//   active high and clears every stage.
//
// Limitations
//   No lane-to-lane de-skew. sync_header_o lags sync_header_i by one clock,
//   not four, so a sync header does not stay with its beat. Sync headers are
//   a Gen3 and above signal (PG239, Table 7).
//
// References
//   PCIe Base Spec r2.1, §4.2.4.10
//   PG239, Table 7: RX Data Signals for UltraScale+ Devices
// ---------------------------------------------------------------------------
module block_alignment
  import pcie_phy_pkg::*;
#(
    // Bits per lane per beat; phy_receive passes 32, the descrambler's width.
    parameter int DATA_WIDTH    = 32,
    parameter int MAX_NUM_LANES = 4
) (
    // ---- clock, reset and control ------------------------------------------
    input  logic                                           clk_i,
    input  logic                                           rst_i,
    input  logic                                           phy_link_up_i,
    input  logic                                           lane_reverse_i,
    input  rate_speed_e                                    curr_data_rate_i,
    // ---- input beat, from the descramblers ---------------------------------
    input  logic        [( MAX_NUM_LANES* DATA_WIDTH)-1:0] data_i,
    input  logic        [               MAX_NUM_LANES-1:0] data_valid_i,
    input  logic        [           (4*MAX_NUM_LANES)-1:0] data_k_i,
    input  logic        [           (2*MAX_NUM_LANES)-1:0] sync_header_i,
    // ---- output beat, to pack_data -----------------------------------------
    output logic        [( MAX_NUM_LANES* DATA_WIDTH)-1:0] data_o,
    output logic        [               MAX_NUM_LANES-1:0] data_valid_o,
    output logic        [           (4*MAX_NUM_LANES)-1:0] data_k_o,
    output logic        [           (2*MAX_NUM_LANES)-1:0] sync_header_o,
    // ---- read only into unused intermediates -------------------------------
    input  logic        [                             5:0] pipe_width_i,
    input  logic        [                             5:0] num_active_lanes_i
);



  // Only NumPipelines is used; the logic reads none of the others.
  localparam int PipeWidthGen1 = 8;
  localparam int PipeWidthGen2 = 16;
  localparam int PipeWidthGen3 = 16;
  localparam int NumPipelines = 4;
  localparam int PipeWidthGen4 = 32;
  localparam int PipeWidthGen5 = 32;
  localparam int BytesPerTransfer = DATA_WIDTH / 8;
  localparam int MaxWordsPerTransaction = 512 / DATA_WIDTH;
  localparam int MaxBytesPerTransfer = MAX_NUM_LANES * BytesPerTransfer;


  // data_out is never assigned. pipewidth_bytes feeds only
  // pipewidth_shift_idx, which nothing reads.
  logic [31:0] data_out;
  logic [ 7:0] pipewidth_bytes;


  // One entry per stage for data, valid, K flags and sync header. word_count,
  // is_ordered_set, is_data, ready_out and mask are only ever reset or copied
  // from Q, and nothing reads them.
  typedef struct {
    logic [NumPipelines-1:0][( MAX_NUM_LANES* DATA_WIDTH)-1:0] data;
    logic [NumPipelines-1:0][MAX_NUM_LANES-1:0]                data_valid;
    logic [NumPipelines-1:0][(4*MAX_NUM_LANES)-1:0]            data_k;
    logic [NumPipelines-1:0][(2*MAX_NUM_LANES)-1:0]            sync_header;
    logic [5:0]                                                word_count;
    logic                                                      is_ordered_set;
    logic                                                      is_data;
    logic                                                      ready_out;
    logic [15:0]                                               mask;

  } block_alignment_t;


  block_alignment_t D;
  block_alignment_t Q;

  // lane_number, byte_number and lane_idx are never assigned;
  // pipewidth_shift_idx and lanes_shift_idx are computed and never read.
  logic [7:0] lane_number;
  logic [7:0] byte_number;
  logic [7:0] pipewidth_shift_idx;
  logic [7:0] lanes_shift_idx;
  logic [7:0] lane_idx;


  always_ff @(posedge clk_i) begin : main_seq_block
    if (rst_i) begin
      Q <= '{default: 'd0};
    end else begin
      Q <= D;
    end
  end



  always_comb begin : block_alignment_combinational_logic
    pipewidth_bytes     = (pipe_width_i >> 3);
    pipewidth_shift_idx = (pipewidth_bytes) - 1;
    lanes_shift_idx     = 1 + (num_active_lanes_i >> 1);


    // The default keeps every field of D assigned on every pass; the fields
    // the loop does not write would otherwise infer latches.
    //
    // Every stage advances every clock, and data, K flags and valid advance
    // together, so an idle input clock enters as a bubble with valid low and
    // leaves four clocks later. Do not gate data and valid on different
    // conditions: the beat count would stay right and beats would pair with
    // the wrong valid bits.
    D = Q;
    for (int pipeline_idx = 0; pipeline_idx < NumPipelines; pipeline_idx++) begin
      if (pipeline_idx == 0) begin
        D.data[pipeline_idx]        = data_i;
        D.data_k[pipeline_idx]      = data_k_i;
        D.data_valid[pipeline_idx]  = {MAX_NUM_LANES{phy_link_up_i}} & data_valid_i;
        D.sync_header[pipeline_idx] = sync_header_i;
      end else begin
        D.data_valid[pipeline_idx]  = Q.data_valid[pipeline_idx-1];
        D.data[pipeline_idx]        = Q.data[pipeline_idx-1];
        D.data_k[pipeline_idx]      = Q.data_k[pipeline_idx-1];
        // Reads D, not Q: every stage takes stage 0's value in the same clock,
        // so sync_header_o lags sync_header_i by one clock while data_o lags
        // by four.
        D.sync_header[pipeline_idx] = D.sync_header[pipeline_idx-1];
      end
    end


  end


  assign sync_header_o = Q.sync_header[NumPipelines-1];
  assign data_valid_o  = Q.data_valid[NumPipelines-1];
  assign data_k_o      = Q.data_k[NumPipelines-1];
  assign data_o        = Q.data[NumPipelines-1];
endmodule
