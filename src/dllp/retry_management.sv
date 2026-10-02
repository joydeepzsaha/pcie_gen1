// ---------------------------------------------------------------------------
// retry_management -- Ack/Nak handling, REPLAY_TIMER and REPLAY_NUM per slot
//
//!module: retry_management
//! Author: Idris Somoye
// Modified by: Kourosh Ghahramani
// Silicon Systems Research Lab, University of Washington
//
// Purpose
//   Tracks the TLPs in the retry buffer by sequence number, one slot each,
//   frees the slots an Ack or Nak covers, and asks retry_transmit to replay a
//   slot on a Nak or on its REPLAY_TIMER expiry. On a REPLAY_NUM rollover it
//   requests a Link retrain and holds the replay until retraining completes.
//
// Interfaces
//   Allocation    tx_valid_i, tx_seq_num_i: tlp2dllp has framed a TLP with
//                 this sequence number; it takes slot retry_index_o.
//                 retry_available_o: a slot is free.
//   Sent          tlp_sent_i, tlp_sent_seq_i: a TLP's last beat has left the
//                 Data Link Layer; arms the matching slot's timer at 0.
//   Ack/Nak       ack_nack_i (1 = Ack), ack_nack_vld_i, ack_seq_num_i.
//   Replay        retry_valid_o: a request per slot, to retry_transmit;
//                 retry_ack_i: accepted; retry_complete_i: replayed.
//   Retrain       retry_err_o: the Link retrain request, a level.
//                 link_retraining_i: the LTSSM is in Recovery or Configuration.
//
// Clock and reset
//   clk_i only. rst_i is synchronous and active high.
//
// Limitations
//   REPLAY_TIMER and REPLAY_NUM are kept per slot. An Ack that frees older
//   slots restarts neither for the remaining slots, and an expiry replays
//   only its own slot; PCIe Base Spec r2.1, §3.5.2.1 keeps one timer and
//   replays every unacknowledged TLP. An out-of-window Ack or Nak is ignored
//   without a Data Link Layer Protocol Error.
//
// References
//   PCIe Base Spec r2.1, §3.5.2.1
//   PCIe Base Spec r2.1, §3.5.2.2
// ---------------------------------------------------------------------------
module retry_management
  import pcie_datalink_pkg::*;
#(
    parameter int DATA_WIDTH       = 32,
    parameter int STRB_WIDTH       = DATA_WIDTH / 8,
    parameter int KEEP_WIDTH       = STRB_WIDTH,
    parameter int USER_WIDTH       = 1,
    parameter int S_COUNT          = 1,
    parameter int MAX_PAYLOAD_SIZE = 256,
    parameter int RAM_DATA_WIDTH   = 32,
    parameter int RETRY_TLP_SIZE   = 3,               // number of retry slots
    // REPLAY_TIMER limit in clk_i cycles; pcie_datalink_layer passes its own.
    parameter int REPLAY_TIMER_CYCLES = pcie_datalink_pkg::replay_timer_cycles(128, 1, 8),
    // Replays before REPLAY_NUM rolls over; the next initiation requests a
    // Link retrain (PCIe Base Spec r2.1, §3.5.2.1).
    parameter int MAX_REPLAY_ATTEMPTS = 3,

    parameter int RAM_ADDR_WIDTH = $clog2(RAM_DATA_WIDTH)
) (
    input logic clk_i,
    input logic rst_i,
    // A TLP's last beat has left the Data Link Layer, with its sequence number.
    input  logic tlp_sent_i, input logic [11:0] tlp_sent_seq_i,
    input  logic [              11:0] tx_seq_num_i,
    input  logic                      tx_valid_i,
    // ---- slots, replay and retrain -----------------------------------------
    output logic                      retry_available_o,
    output logic [               7:0] retry_index_o,
    // link_retraining_i is synchronised to clk_i in pcie_phy_top or
    // pcie_endpoint_top.
    output logic                      retry_err_o, input logic link_retraining_i = 1'b0,
    output logic [RETRY_TLP_SIZE-1:0] retry_valid_o,
    input  logic [RETRY_TLP_SIZE-1:0] retry_ack_i,
    input  logic [RETRY_TLP_SIZE-1:0] retry_complete_i,
    // ---- received Ack or Nak -----------------------------------------------
    input  logic                      ack_nack_i,
    input  logic                      ack_nack_vld_i,
    input  logic [              11:0] ack_seq_num_i
);

  // Not used; axis_retry_fifo sizes the retry buffer.
  localparam int MaxTlpHdrSizeDW = 4;
  localparam int MaxBytesPerTLP = MAX_PAYLOAD_SIZE;
  localparam int MaxTlpTotalSizeDW = MaxTlpHdrSizeDW + MaxBytesPerTLP + 1;

  // Per-slot replay states; the state table is at gen_retry_counters.
  typedef enum logic [2:0] {
    ST_RETRY_IDLE,
    ST_CNT_RETRY,
    ST_REPLAY,
    ST_WAIT_REPLAY,
    ST_RETRY_ERR, ST_WAIT_RETRAIN
  } retry_st_e;

  // Set in ST_RETRY_ERR; nothing outside the slot state machines reads it.
  logic [RETRY_TLP_SIZE-1:0]       error_c;
  logic [RETRY_TLP_SIZE-1:0]       error_r;
  // Slot bookkeeping: retrys_r[i] marks slot i as holding an unacknowledged
  // TLP. An Ack is in the window when its sequence number is that of a held
  // TLP (ack_seq_is_outstanding), a Nak when a held TLP has its sequence
  // number or a later one (nack_seq_is_in_window).
  logic [               7:0]       next_retry_index_c;
  logic [               7:0]       next_retry_index_r;
  logic [RETRY_TLP_SIZE-1:0]       retry_valid_c;
  logic [RETRY_TLP_SIZE-1:0]       retry_valid_r;
  logic [RETRY_TLP_SIZE-1:0]       retrys_c;
  logic [RETRY_TLP_SIZE-1:0]       retrys_r;
  logic                            next_index_found;
  logic                            ack_seq_is_outstanding;
  logic                            nack_seq_is_in_window;
  //sequence number signals
  logic [RETRY_TLP_SIZE-1:0][11:0] ack_seq_mem_c;
  logic [RETRY_TLP_SIZE-1:0][11:0] ack_seq_mem_r;

  // True when sequence_number is at or before ack_number, modulo 4096.
  // Outstanding windows are smaller than half of the 12-bit sequence space,
  // so this modulo comparison remains unambiguous across 0xfff -> 0x000.
  function automatic logic seq_acked(
      input logic [11:0] sequence_number,
      input logic [11:0] ack_number
  );
    logic [11:0] distance;
    begin
      distance = ack_number - sequence_number;
      seq_acked = !distance[11];
    end
  endfunction

  // True when sequence_number is strictly after reference_number, modulo
  // 4096.
  function automatic logic seq_after(
      input logic [11:0] sequence_number,
      input logic [11:0] reference_number
  );
    logic [11:0] distance;
    begin
      distance = sequence_number - reference_number;
      seq_after = (distance != '0) && !distance[11];
    end
  endfunction

  always_ff @(posedge clk_i) begin : main_sequential_block
    if (rst_i) begin
      retrys_r           <= '0;
      error_r            <= '0;
      next_retry_index_r <= '0;
      retry_valid_r      <= '0;
      ack_seq_mem_r      <= '0;
    end else begin
      retrys_r           <= retrys_c;
      error_r            <= error_c;
      next_retry_index_r <= next_retry_index_c;
      retry_valid_r      <= retry_valid_c;
      ack_seq_mem_r      <= ack_seq_mem_c;
    end
  end

  // Frees the slots an in-window Ack or Nak covers, allocates the slot at
  // next_retry_index_r to a newly framed TLP, and picks the next free slot.
  always_comb begin : retry_tracking_combo
    retrys_c           = retrys_r;
    next_retry_index_c = next_retry_index_r;
    next_index_found   = '0;
    ack_seq_is_outstanding = '0;
    nack_seq_is_in_window = '0;
    for (int i = 0; i < RETRY_TLP_SIZE; i++) begin
      ack_seq_mem_c[i] = ack_seq_mem_r[i];
      if (retrys_r[i] && (ack_seq_mem_r[i] == ack_seq_num_i)) begin
        ack_seq_is_outstanding  = ack_nack_vld_i && ack_nack_i;
      end
      if (retrys_r[i] && ack_nack_vld_i && !ack_nack_i &&
          ((ack_seq_mem_r[i] == ack_seq_num_i) ||
           seq_after(ack_seq_mem_r[i], ack_seq_num_i))) begin
        nack_seq_is_in_window = '1;
      end
    end

    // Apply only in-window ACKs before allocating a TLP arriving on the same
    // cycle.  Erroneous future/old ACKs must not free retry entries.
    if (ack_seq_is_outstanding) begin
      for (int i = 0; i < RETRY_TLP_SIZE; i++) begin
        if (retrys_r[i] && seq_acked(ack_seq_mem_r[i], ack_seq_num_i)) begin
          retrys_c[i] = '0;
        end
      end
    end

    // NAK N acknowledges everything through N and requests replay strictly
    // after N.  N itself need not still occupy a retry-buffer entry.
    if (nack_seq_is_in_window) begin
      for (int i = 0; i < RETRY_TLP_SIZE; i++) begin
        if (retrys_r[i] && seq_acked(ack_seq_mem_r[i], ack_seq_num_i)) begin
          retrys_c[i] = '0;
        end
      end
    end

    if (tx_valid_i && !retrys_c[next_retry_index_r]) begin
      ack_seq_mem_c[next_retry_index_r] = tx_seq_num_i;
      retrys_c[next_retry_index_r]      = '1;
    end

    // Select a bounded free slot.  Searching from zero also guarantees a
    // deterministic wrap to slot zero after the last slot is consumed.
    // The search runs every cycle, so a slot that an Ack or Nak frees below
    // the current one becomes retry_index_o on the next edge, even while a
    // frame is being written into the current slot.
    for (int i = 0; i < RETRY_TLP_SIZE; i++) begin
      if (!retrys_c[i] && !next_index_found) begin
        next_retry_index_c = i;
        next_index_found   = '1;
      end
    end
  end


  // -------------------------------------------------------------------------
  // Retrain request
  // -------------------------------------------------------------------------
  // On a REPLAY_NUM rollover the Transmitter asks the Physical Layer to
  // retrain the Link and waits for retraining to complete before the replay;
  // Data Link Layer state, the retry buffer included, is kept (PCIe Base Spec
  // r2.1, §3.5.2.1). A slot that rolls over waits in ST_WAIT_RETRAIN with its
  // entry intact. retrain_req_r is a level, registered because it crosses
  // into the LTSSM's clock domain (pcie_phy_top and pcie_endpoint_top
  // synchronise it), and a level is not lost in the crossing. It drops once
  // link_retraining_i is seen: the LTSSM reads it in L0, and a request still
  // high on the return to L0 would start a second retrain. A retrain already
  // in progress at the rollover counts as seen, so the request does not rise.
  // Retraining is complete when link_retraining_i falls after it was seen.
  logic [RETRY_TLP_SIZE-1:0] wait_retrain;    // slot i is in ST_WAIT_RETRAIN
  logic                      retrain_seen_r;  // link_retraining_i seen while a slot waits
  logic                      retrain_done;    // ...and low again: retraining completed
  logic                      retrain_req_r;   // the request level (a CDC source)
  assign retrain_done = (|wait_retrain) && retrain_seen_r && !link_retraining_i;
  always_ff @(posedge clk_i) begin : retrain_handshake
    if (rst_i || !(|wait_retrain)) begin
      retrain_seen_r <= 1'b0;
    end else if (link_retraining_i) begin
      retrain_seen_r <= 1'b1;
    end
    if (rst_i) begin
      retrain_req_r <= 1'b0;
    end else begin
      retrain_req_r <= (|wait_retrain) && !retrain_seen_r && !link_retraining_i;
    end
  end

  // -------------------------------------------------------------------------
  // Per-slot replay state machine
  // -------------------------------------------------------------------------
  // One instance per slot; replay_cnt_r is its REPLAY_NUM and retry_timer_r
  // its REPLAY_TIMER in clk_i cycles. "Replay" below means ST_REPLAY, or
  // ST_WAIT_RETRAIN once replay_cnt_r has reached MAX_REPLAY_ATTEMPTS.
  //   State            Action            Exit
  //   ST_RETRY_IDLE    empty slot        filled: ST_CNT_RETRY; Nak: replay
  //   ST_CNT_RETRY     timer runs armed  Nak or timer expiry: replay
  //   ST_REPLAY        retry_valid_o     retry_ack_i: ST_WAIT_REPLAY
  //   ST_WAIT_REPLAY   frame replaying   retry_complete_i: ST_CNT_RETRY
  //   ST_WAIT_RETRAIN  retrain wait      retrain_done: ST_REPLAY
  //   ST_RETRY_ERR     error_c high      slot freed: ST_RETRY_IDLE
  // An Ack or Nak covering the slot returns it to ST_RETRY_IDLE from any state.
  // ST_RETRY_ERR is reached only from the default arm, on an illegal encoding.
  for (genvar i = 0; i < RETRY_TLP_SIZE; i++) begin : gen_retry_counters
    retry_st_e curr_state, next_state;
    localparam int REPLAY_COUNT_WIDTH =
        (MAX_REPLAY_ATTEMPTS < 2) ? 1 : $clog2(MAX_REPLAY_ATTEMPTS + 1);
    logic [REPLAY_COUNT_WIDTH-1:0] replay_cnt_c, replay_cnt_r;
    logic [31:0] retry_timer_c, retry_timer_r;
    // armed_r: this slot's TLP, or its latest retransmission, has left the
    // Data Link Layer. Each such last beat (sent_here) arms the slot and
    // restarts its timer at 0. It stands for the last Symbol of a transmission
    // or retransmission, where PCIe Base Spec r2.1, §3.5.2.1 starts the timer.
    // Unarmed, the timer holds; replay initiation, the error and a freed slot
    // disarm.
    logic armed_c, armed_r;
    logic sent_here;
    // Matched against the next-state slot (retrys_c, ack_seq_mem_c), so a last
    // beat in the cycle the slot is allocated still arms it.
    assign sent_here = tlp_sent_i && retrys_c[i] && (ack_seq_mem_c[i] == tlp_sent_seq_i);
    always @(posedge clk_i) begin : retry_buffer_seq
      if (rst_i) begin
        retry_timer_r <= '0;
        replay_cnt_r  <= '0;
        armed_r       <= 1'b0;
        curr_state    <= ST_RETRY_IDLE;
      end else begin
        retry_timer_r <= retry_timer_c;
        replay_cnt_r  <= replay_cnt_c;
        armed_r       <= armed_c;
        curr_state    <= next_state;
      end
    end
    always_comb begin : retry_timer
      replay_cnt_c     = replay_cnt_r;
      retry_timer_c    = retry_timer_r;
      armed_c          = armed_r;
      next_state       = curr_state;
      retry_valid_c[i] = retry_valid_r[i];
      error_c[i]       = error_r[i];
      case (curr_state)
        ST_RETRY_IDLE: begin
          // A filled slot moves to ST_CNT_RETRY unless an Ack or Nak decides it
          // in the same cycle.
          if (retrys_r[i]) begin
            retry_timer_c = '0;
            if (ack_seq_is_outstanding &&
                seq_acked(ack_seq_mem_r[i], ack_seq_num_i)) begin
              retry_valid_c[i] = '0;
            end else if (nack_seq_is_in_window &&
                         seq_after(ack_seq_mem_r[i], ack_seq_num_i)) begin
              armed_c = 1'b0;
              if (replay_cnt_r >= MAX_REPLAY_ATTEMPTS) begin
                replay_cnt_c = '0;               // REPLAY_NUM rolls over
                next_state   = ST_WAIT_RETRAIN;
              end else begin
                replay_cnt_c     = replay_cnt_r + 1'b1;
                retry_valid_c[i] = '1;
                next_state       = ST_REPLAY;
              end
            end else begin
              next_state = ST_CNT_RETRY;
            end
          end
        end
        ST_CNT_RETRY: begin
          if (!retrys_r[i]) begin  // freed by an Ack or Nak
            replay_cnt_c  = '0;
            retry_timer_c = '0;
            armed_c       = 1'b0;
            next_state    = ST_RETRY_IDLE;
          end else if (ack_seq_is_outstanding &&
                       seq_acked(ack_seq_mem_r[i], ack_seq_num_i)) begin
            replay_cnt_c     = '0;
            retry_timer_c    = '0;
            retry_valid_c[i] = '0;
            armed_c          = 1'b0;
            next_state       = ST_RETRY_IDLE;
          end else if (nack_seq_is_in_window &&
                       seq_after(ack_seq_mem_r[i], ack_seq_num_i)) begin
            // NAK wins if it arrives on the timeout boundary.
            retry_timer_c = '0;
            armed_c       = 1'b0;
            if (replay_cnt_r >= MAX_REPLAY_ATTEMPTS) begin
              replay_cnt_c = '0;                 // REPLAY_NUM rolls over
              next_state   = ST_WAIT_RETRAIN;
            end else begin
              replay_cnt_c     = replay_cnt_r + 1'b1;
              retry_valid_c[i] = '1;
              next_state       = ST_REPLAY;
            end
          end else if (armed_r && !link_retraining_i &&
                       (REPLAY_TIMER_CYCLES == 0 ||
                        retry_timer_r >= REPLAY_TIMER_CYCLES - 1)) begin
            // The REPLAY_TIMER holds while the LTSSM is in Recovery or
            // Configuration (PCIe Base Spec r2.1, §3.5.2.1), so it cannot
            // expire there either.
            retry_timer_c = '0;
            armed_c       = 1'b0;
            if (replay_cnt_r >= MAX_REPLAY_ATTEMPTS) begin
              // Rollover: REPLAY_NUM goes from 11b to 00b, and the replay waits
              // in ST_WAIT_RETRAIN for the Link retrain.
              replay_cnt_c = '0;
              next_state   = ST_WAIT_RETRAIN;
            end else begin
              replay_cnt_c     = replay_cnt_r + 1'b1;
              next_state       = ST_REPLAY;
              retry_valid_c[i] = '1;
            end
          end else if (armed_r && !link_retraining_i) begin  // holds while retraining
            retry_timer_c = retry_timer_r + 1'b1;
          end
        end
        ST_REPLAY: begin
          // An Ack that arrives before retry_transmit accepts the request
          // cancels the replay.
          if (!retrys_r[i] ||
              (ack_seq_is_outstanding &&
               seq_acked(ack_seq_mem_r[i], ack_seq_num_i))) begin
            replay_cnt_c     = '0;
            retry_timer_c    = '0;
            retry_valid_c[i] = '0;
            armed_c          = 1'b0;
            next_state       = ST_RETRY_IDLE;
          end
          else begin
            if (retry_ack_i[i]) begin
              retry_timer_c    = '0;
              retry_valid_c[i] = '0;
              next_state       = ST_WAIT_REPLAY;
            end
          end
        end
        ST_WAIT_REPLAY: begin
          if (!retrys_r[i] ||
              (ack_seq_is_outstanding &&
               seq_acked(ack_seq_mem_r[i], ack_seq_num_i))) begin
            replay_cnt_c  = '0;
            retry_timer_c = '0;
            armed_c       = 1'b0;
            next_state    = ST_RETRY_IDLE;
          end
          else begin
            // The restart is the retransmission's own last beat leaving the
            // Data Link Layer (sent_here, below), not retry_complete_i, which
            // retry_transmit raises upstream of both arbiters. If that beat
            // has already left, the timer runs, except while the Link retrains.
            if (armed_r && !link_retraining_i) retry_timer_c = retry_timer_r + 1'b1;
            if (retry_complete_i[i]) begin
              next_state    = ST_CNT_RETRY;
            end
          end
        end
        ST_WAIT_RETRAIN: begin
          // REPLAY_NUM has rolled over: retrain_req_r requests the retrain and
          // the replay waits for it to complete. The entry stays, so an Ack
          // that covers it frees it as in any other state.
          armed_c = 1'b0;
          if (!retrys_r[i] ||
              (ack_seq_is_outstanding &&
               seq_acked(ack_seq_mem_r[i], ack_seq_num_i))) begin
            replay_cnt_c     = '0;
            retry_timer_c    = '0;
            retry_valid_c[i] = '0;
            next_state       = ST_RETRY_IDLE;
          end else if (retrain_done) begin
            retry_timer_c    = '0;
            retry_valid_c[i] = '1;
            next_state       = ST_REPLAY;
          end
        end
        ST_RETRY_ERR: begin
          // Reached only from the default arm, on an illegal encoding; a
          // rollover goes to ST_WAIT_RETRAIN.
          error_c[i] = '1;
          armed_c    = 1'b0;
          if (!retrys_r[i]) begin
            error_c[i]       = '0;
            replay_cnt_c     = '0;
            retry_timer_c    = '0;
            retry_valid_c[i] = '0;
            next_state       = ST_RETRY_IDLE;
          end
        end
        default: begin
          retry_timer_c    = '0;
          retry_valid_c[i] = '0;
          armed_c          = 1'b0;
          next_state       = ST_RETRY_ERR;
        end
      endcase
      // REPLAY_NUM is one counter, and a rollover leaves it at 00b for every
      // TLP in the retry buffer (PCIe Base Spec r2.1, §3.5.2.1). While any
      // slot waits for the retrain, every slot's count is held at 00b, so a
      // second slot that was at 11b does not roll over at its next replay and
      // request another retrain. wait_retrain is decoded from registered
      // state, so the hold adds no combinational path between slots.
      if (|wait_retrain) replay_cnt_c = '0;
      // A free slot is never armed, so a re-allocated one starts unarmed.
      if (!retrys_c[i]) armed_c = 1'b0;
      // The start / restart event, last so it overrides the hold above -- but
      // never a replay initiation or the error decided this same cycle.
      if (sent_here && (next_state != ST_REPLAY) && (next_state != ST_RETRY_ERR) &&
          (next_state != ST_WAIT_RETRAIN)) begin
        armed_c       = 1'b1;
        retry_timer_c = '0;
      end
    end
    assign wait_retrain[i] = (curr_state == ST_WAIT_RETRAIN);
  end : gen_retry_counters


  assign retry_err_o       = retrain_req_r;  // the retrain request; error_r is not reported
  assign retry_available_o = !(&retrys_r);
  assign retry_index_o     = next_retry_index_r;
  assign retry_valid_o     = retry_valid_r;

endmodule
