// ---------------------------------------------------------------------------
// tlp_vc_buffer -- transmit TLP buffer in front of the credit gate
//
// Original author: Joydeep Saha
// Modified by: Kourosh Ghahramani
// Silicon Systems Research Lab, University of Washington
//
// Purpose
//   Stores up to PACKET_DEPTH whole TLPs from tlp_generator, each of up to
//   MAX_PACKET_WORDS beats, and offers the oldest to tlp_credit_manager with
//   its credit pool and data credits. Once the gate accepts it, the TLP is
//   sent to the Data Link Layer. A TLP is offered only after its last beat is
//   stored.
//
// Interfaces
//   Input   s_axis_*: TLP beats, tlast on the last.
//   Class   s_packet_class_i, s_packet_length_dw_i, s_packet_has_data_i: the
//           TLP's class and Length, sampled with its first beat.
//   Gate    packet_valid_o, packet_ready_i, packet_credit_class_o,
//           packet_data_credits_o: the oldest stored TLP, offered while no
//           TLP is being sent.
//   Output  m_axis_*: the TLP's beats, from the cycle after the gate takes it.
//   Status  overflow_o: a TLP longer than MAX_PACKET_WORDS was presented.
//
// Clock and reset
//   clk_i only. rst_i is synchronous and active high; it clears the pointers
//   and counts, not the stored beats.
//
// Limitations
//   One FIFO in transmit order and no VC field: a TLP held at the credit gate
//   holds every TLP behind it, including Posted Requests and Completions that
//   PCIe Base Spec r2.1, §2.4.1 and §2.6.1.1 require to be able to pass a
//   blocked Non-Posted Request.
//
// References
//   PCIe Base Spec r2.1, §2.4.1
//   PCIe Base Spec r2.1, §2.6.1
//   PCIe Base Spec r2.1, §2.6.1.1
// ---------------------------------------------------------------------------
`timescale 1ns/1ps
module tlp_vc_buffer
  import tlp_pkg::*;
#(
    parameter int DATA_WIDTH = 32,
    parameter int KEEP_WIDTH = DATA_WIDTH / 8,
    parameter int USER_WIDTH = 3,
    parameter int PACKET_DEPTH = 4,
    // The longest TLP tlp_generator sends: a TLP Prefix, a 4 DW header, 1024
    // payload DWs and the ECRC, 1030 beats.
    parameter int MAX_PACKET_WORDS = 1030
) (
    input  logic                  clk_i,
    input  logic                  rst_i,
    input  logic [DATA_WIDTH-1:0] s_axis_tdata,
    input  logic [KEEP_WIDTH-1:0] s_axis_tkeep,
    input  logic                  s_axis_tvalid,
    input  logic                  s_axis_tlast,
    input  logic [USER_WIDTH-1:0] s_axis_tuser,
    output logic                  s_axis_tready,
    input  tlp_class_e            s_packet_class_i,
    input  logic [10:0]           s_packet_length_dw_i,
    input  logic                  s_packet_has_data_i,

    output logic                  packet_valid_o,
    input  logic                  packet_ready_i,
    output tlp_credit_class_e     packet_credit_class_o,
    output logic [11:0]           packet_data_credits_o,

    output logic [DATA_WIDTH-1:0] m_axis_tdata,
    output logic [KEEP_WIDTH-1:0] m_axis_tkeep,
    output logic                  m_axis_tvalid,
    output logic                  m_axis_tlast,
    output logic [USER_WIDTH-1:0] m_axis_tuser,
    input  logic                  m_axis_tready,
    output logic                  overflow_o
);

  localparam int PACKET_INDEX_WIDTH = PACKET_DEPTH <= 1 ? 1 : $clog2(PACKET_DEPTH);
  localparam int WORD_INDEX_WIDTH = MAX_PACKET_WORDS <= 1 ? 1 : $clog2(MAX_PACKET_WORDS+1);
  localparam int MEM_DEPTH = PACKET_DEPTH * MAX_PACKET_WORDS;
  localparam int MEM_INDEX_WIDTH = MEM_DEPTH <= 1 ? 1 : $clog2(MEM_DEPTH);
  // One flat array: slot p holds its beats from entry p * MAX_PACKET_WORDS
  // on (wr_index, rd_index).
  logic [DATA_WIDTH-1:0] data_mem [0:MEM_DEPTH-1];
  logic [KEEP_WIDTH-1:0] keep_mem [0:MEM_DEPTH-1];
  logic [USER_WIDTH-1:0] user_mem [0:MEM_DEPTH-1];
  logic last_mem [0:MEM_DEPTH-1];
  logic [WORD_INDEX_WIDTH-1:0] word_count_mem [0:PACKET_DEPTH-1];
  tlp_credit_class_e class_mem [0:PACKET_DEPTH-1];
  logic [11:0] credit_mem [0:PACKET_DEPTH-1];
  logic [PACKET_INDEX_WIDTH-1:0] wr_packet_r, rd_packet_r;
  logic [WORD_INDEX_WIDTH-1:0] wr_word_r, rd_word_r;
  logic [MEM_INDEX_WIDTH-1:0] wr_index, rd_index;
  logic [$clog2(PACKET_DEPTH+1)-1:0] packet_count_r;
  logic transmitting_r;
  logic input_fire, output_fire, packet_completed, packet_released;
  integer reset_index;

  assign s_axis_tready = packet_count_r < PACKET_DEPTH && wr_word_r < MAX_PACKET_WORDS;
  assign input_fire = s_axis_tvalid && s_axis_tready;
  // packet_count_r counts complete TLPs only, so a TLP the gate accepts is
  // sent from memory without waiting for input.
  assign packet_valid_o = packet_count_r != 0 && !transmitting_r;
  assign packet_credit_class_o = tlp_credit_class_e'(class_mem[rd_packet_r]);
  assign packet_data_credits_o = credit_mem[rd_packet_r];
  assign packet_released = packet_valid_o && packet_ready_i;

  assign wr_index = MEM_INDEX_WIDTH'((wr_packet_r * MAX_PACKET_WORDS) + wr_word_r);
  assign rd_index = MEM_INDEX_WIDTH'((rd_packet_r * MAX_PACKET_WORDS) + rd_word_r);

  assign m_axis_tdata = data_mem[rd_index];
  assign m_axis_tkeep = keep_mem[rd_index];
  assign m_axis_tuser = user_mem[rd_index];
  assign m_axis_tlast = last_mem[rd_index];
  assign m_axis_tvalid = transmitting_r;
  assign output_fire = m_axis_tvalid && m_axis_tready;
  assign packet_completed = input_fire && s_axis_tlast;

  always_ff @(posedge clk_i) begin
    if (rst_i) begin
      wr_packet_r <= '0;
      rd_packet_r <= '0;
      wr_word_r <= '0;
      rd_word_r <= '0;
      packet_count_r <= '0;
      transmitting_r <= 1'b0;
      overflow_o <= 1'b0;
      for (reset_index = 0; reset_index < PACKET_DEPTH; reset_index = reset_index + 1)
        word_count_mem[reset_index] <= '0;
    end else begin
      // Packet-depth backpressure is normal ready/valid behavior. Overflow
      // only reports a source attempting to exceed the maximum packet size;
      // that beat is not taken, and s_axis_tready stays low until reset.
      overflow_o <= s_axis_tvalid && (wr_word_r >= MAX_PACKET_WORDS);
      if (input_fire) begin
        data_mem[wr_index] <= s_axis_tdata;
        keep_mem[wr_index] <= s_axis_tkeep;
        user_mem[wr_index] <= s_axis_tuser;
        last_mem[wr_index] <= s_axis_tlast;
        // tlp_layer registers the class and Length at the tlp_generator
        // header handshake, before the first beat.
        if (wr_word_r == 0) begin
          class_mem[wr_packet_r] <= tlp_credit_class(s_packet_class_i);
          credit_mem[wr_packet_r] <= s_packet_has_data_i ?
                                      tlp_data_credits(s_packet_length_dw_i) : 0;
        end
        if (s_axis_tlast) begin
          word_count_mem[wr_packet_r] <= wr_word_r + 1'b1;
          wr_word_r <= '0;
          wr_packet_r <= wr_packet_r == PACKET_DEPTH-1 ? '0 : wr_packet_r + 1'b1;
        end else begin
          wr_word_r <= wr_word_r + 1'b1;
        end
      end

      if (packet_released) begin
        transmitting_r <= 1'b1;
        rd_word_r <= '0;
      end
      if (output_fire) begin
        if (m_axis_tlast) begin
          transmitting_r <= 1'b0;
          rd_word_r <= '0;
          rd_packet_r <= rd_packet_r == PACKET_DEPTH-1 ? '0 : rd_packet_r + 1'b1;
        end else begin
          rd_word_r <= rd_word_r + 1'b1;
        end
      end

      case ({packet_completed, output_fire && m_axis_tlast})
        2'b10: packet_count_r <= packet_count_r + 1'b1;
        2'b01: packet_count_r <= packet_count_r - 1'b1;
        default: packet_count_r <= packet_count_r;
      endcase
    end
  end

endmodule
