//!module: retry_management
//! Author: Idris Somoye
//! Module implements a retry management controller. It uses a timer to track the time
//! between transmissions and ack/nack. Module resend TLPs stored in the retry FIFO and the
//! PCIe mandated retry increments.
module retry_management
  import pcie_datalink_pkg::*;
#(
    parameter int DATA_WIDTH       = 32,              //AXIS data width
    parameter int STRB_WIDTH       = DATA_WIDTH / 8,  // TLP strobe width
    parameter int KEEP_WIDTH       = STRB_WIDTH,
    parameter int USER_WIDTH       = 1,
    parameter int S_COUNT          = 1,
    parameter int MAX_PAYLOAD_SIZE = 256,
    parameter int RAM_DATA_WIDTH   = 32,              // width of the data
    parameter int RETRY_TLP_SIZE   = 3,               // Width of AXI stream interfaces in bits
    parameter int REPLAY_TIMER_CYCLES = pcie_datalink_pkg::replay_timer_cycles(128, 1, 8),  // 63 #7g-2 Q3; pcie_datalink_layer passes its own
    parameter int MAX_REPLAY_ATTEMPTS = 3,  // p.174: three replays proceed; the 4th initiation rolls REPLAY_NUM over and retrains (63 #7k)

    parameter int RAM_ADDR_WIDTH = $clog2(RAM_DATA_WIDTH)  // number of address bits
) (
    input logic clk_i,  // Clock signal
    input logic rst_i,  // Reset signal
    input  logic tlp_sent_i, input logic [11:0] tlp_sent_seq_i,  // 63 #7g-2 Q3: a TLP's last beat left the DLL, and its sequence number
    input  logic [              11:0] tx_seq_num_i,
    input  logic                      tx_valid_i,
    //retry signals
    output logic                      retry_available_o,
    output logic [               7:0] retry_index_o,
    output logic                      retry_err_o, input logic link_retraining_i = 1'b0,  // 63 #7k: LTSSM in Recovery/Configuration (pcie_phy_top syncs it)
    output logic [RETRY_TLP_SIZE-1:0] retry_valid_o,
    input  logic [RETRY_TLP_SIZE-1:0] retry_ack_i,
    input  logic [RETRY_TLP_SIZE-1:0] retry_complete_i,
    //dllp tlp sequence ack/nack
    input  logic                      ack_nack_i,
    input  logic                      ack_nack_vld_i,
    input  logic [              11:0] ack_seq_num_i
);

  //maxbytesper tlp
  localparam int MaxTlpHdrSizeDW = 4;
  localparam int MaxBytesPerTLP = MAX_PAYLOAD_SIZE;
  localparam int MaxTlpTotalSizeDW = MaxTlpHdrSizeDW + MaxBytesPerTLP + 1;

  //retry mechanism enum
  typedef enum logic [2:0] {
    ST_RETRY_IDLE,
    ST_CNT_RETRY,
    ST_REPLAY,
    ST_WAIT_REPLAY,
    ST_RETRY_ERR, ST_WAIT_RETRAIN  // 63 #7k: RETRY_ERR is no longer entered on rollover (D-7K.8: kept)
  } retry_st_e;

  //error tracking signals
  logic [RETRY_TLP_SIZE-1:0]       error_c;
  logic [RETRY_TLP_SIZE-1:0]       error_r;
  //retry signals
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

  //main  sequential block
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

  //retry tracking combo block
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
      for (int i = 0; i < RETRY_TLP_SIZE; i++) begin  //free retry
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
    for (int i = 0; i < RETRY_TLP_SIZE; i++) begin
      if (!retrys_c[i] && !next_index_found) begin
        next_retry_index_c = i;
        next_index_found   = '1;
      end
    end
  end


  // ===========================================================================
  // sec 63 #7k: REPLAY_NUM rollover -> retrain.  Base 2.1 sec 3.5.2.1 p.174:
  // "If REPLAY_NUM rolls over from 11b to 00b, the Transmitter signals the
  // Physical Layer to retrain the Link, and waits for the completion of
  // retraining before proceeding with the replay ... Data Link Layer state,
  // including the contents of the Retry Buffer, are not reset by this action".
  //
  // A slot whose REPLAY_NUM rolls over parks in ST_WAIT_RETRAIN with its entry
  // intact.  The request is a LEVEL, registered here because it crosses into
  // the LTSSM's clock (pcie_phy_top / pcie_endpoint_top synchronise it), and
  // held until the retrain is SEEN: link_retraining_i -- the LTSSM is in
  // Recovery or Configuration -- high while a slot waits.  A level cannot be
  // lost crossing clocks, and dropping it once seen means the LTSSM, which
  // takes it only in L0, is never sent round twice.  A retrain already under
  // way when the rollover happens (the peer started it) counts as seen: the
  // request never rises.  "Completion of retraining" is link_retraining_i
  // falling after it was seen; every waiting slot then proceeds with its
  // replay.  retry_err_o IS the request (D-7K.8: the port stays, its meaning
  // becomes "Recovery was requested"), and it rises on the same edge the old
  // error did.
  // ===========================================================================
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

  //retry generate loop
  for (genvar i = 0; i < RETRY_TLP_SIZE; i++) begin : gen_retry_counters
    retry_st_e curr_state, next_state;
    localparam int REPLAY_COUNT_WIDTH =
        (MAX_REPLAY_ATTEMPTS < 2) ? 1 : $clog2(MAX_REPLAY_ATTEMPTS + 1);
    logic [REPLAY_COUNT_WIDTH-1:0] replay_cnt_c, replay_cnt_r;
    logic [31:0] retry_timer_c, retry_timer_r;
    // =========================================================================
    // sec 63 #7g-2 step 2 (Kourosh Q3): the REPLAY_TIMER STARTS where the spec
    // starts it.  Base 2.1 sec 3.5.2.1 p.170: "Started at the last Symbol of any
    // TLP transmission or retransmission"; p.175: "Timing starts with ... the
    // last Symbol of a transmitted TLP".  This slot's timer used to start at
    // tx_valid_i, which tlp2dllp raises upstream of both arbiters: 4 cycles
    // before the TLP's last beat left the DLL in steady state and 60 for the
    // first TLP after FC init, which queues behind InitFC2 and the post-init
    // UpdateFC pair (FINDINGS_7G2_PHASE1.md sec 3 finding 2).
    //
    // armed_r: this slot's TLP -- or its latest retransmission -- has left the
    // DLL.  pcie_datalink_layer reports every TLP's last beat on m_phy_axis
    // with the sequence number from its first beat (tlp_sent_i/_seq_i); that
    // arms the matching slot and restarts its timer at 0, so a retransmission
    // restarts it exactly as p.170 says.  Unarmed, the timer holds.  Replay
    // initiation, the error, and freeing the slot all disarm it.
    //
    // Measured at 7g-2 Phase 1: the last beat follows the slot's allocation by
    // >= 4 cycles, so the slot always exists when its own last beat leaves.
    // Matching on the NEXT-state slot (retrys_c / ack_seq_mem_c) covers the
    // same-cycle case as well.
    // =========================================================================
    logic armed_c, armed_r;
    logic sent_here;
    assign sent_here = tlp_sent_i && retrys_c[i] && (ack_seq_mem_c[i] == tlp_sent_seq_i);
    //main sequential block
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
    //main retry combinational block
    always_comb begin : retry_timer
      replay_cnt_c     = replay_cnt_r;
      retry_timer_c    = retry_timer_r;
      armed_c          = armed_r;
      next_state       = curr_state;
      retry_valid_c[i] = retry_valid_r[i];
      error_c[i]       = error_r[i];
      case (curr_state)
        ST_RETRY_IDLE: begin
          //wait for tlp send at this retry index
          if (retrys_r[i]) begin
            retry_timer_c = '0;
            if (ack_seq_is_outstanding &&
                seq_acked(ack_seq_mem_r[i], ack_seq_num_i)) begin
              retry_valid_c[i] = '0;
            end else if (nack_seq_is_in_window &&
                         seq_after(ack_seq_mem_r[i], ack_seq_num_i)) begin
              armed_c = 1'b0;
              if (replay_cnt_r >= MAX_REPLAY_ATTEMPTS) begin
                replay_cnt_c = '0;               // 63 #7k: 11b -> 00b (p.174)
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
          if (!retrys_r[i]) begin  //check if tlp acked
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
              replay_cnt_c = '0;                 // 63 #7k: 11b -> 00b (p.174)
              next_state   = ST_WAIT_RETRAIN;
            end else begin
              replay_cnt_c     = replay_cnt_r + 1'b1;
              retry_valid_c[i] = '1;
              next_state       = ST_REPLAY;
            end
          end else if (armed_r && !link_retraining_i &&
                       (REPLAY_TIMER_CYCLES == 0 ||
                        retry_timer_r >= REPLAY_TIMER_CYCLES - 1)) begin
            // sec 63 #7k, p.170: REPLAY_TIMER "Not advanced during Link
            // retraining (holds its value when the LTSSM is in the Recovery or
            // Configuration state)" -- so it cannot expire there either.
            retry_timer_c = '0;
            armed_c       = 1'b0;
            if (replay_cnt_r >= MAX_REPLAY_ATTEMPTS) begin
              // The 4th initiation: p.174 rolls REPLAY_NUM 11b -> 00b and
              // retrains; the replay waits in ST_WAIT_RETRAIN (63 #7k).
              replay_cnt_c = '0;
              next_state   = ST_WAIT_RETRAIN;
            end else begin
              replay_cnt_c     = replay_cnt_r + 1'b1;
              next_state       = ST_REPLAY;
              retry_valid_c[i] = '1;
            end
          end else if (armed_r && !link_retraining_i) begin  // p.170 hold (63 #7k)
            retry_timer_c = retry_timer_r + 1'b1;
          end
        end
        ST_REPLAY: begin
          //check if late ack
          if (!retrys_r[i] ||
              (ack_seq_is_outstanding &&
               seq_acked(ack_seq_mem_r[i], ack_seq_num_i))) begin
            replay_cnt_c     = '0;
            retry_timer_c    = '0;
            retry_valid_c[i] = '0;
            armed_c          = 1'b0;
            next_state       = ST_RETRY_IDLE;
          end  //check that retry fifo has accepted resend request
          else begin
            if (retry_ack_i[i]) begin
              retry_timer_c    = '0;
              retry_valid_c[i] = '0;
              next_state       = ST_WAIT_REPLAY;
            end
          end
        end
        ST_WAIT_REPLAY: begin
          //wait for an ack..
          if (!retrys_r[i] ||
              (ack_seq_is_outstanding &&
               seq_acked(ack_seq_mem_r[i], ack_seq_num_i))) begin
            replay_cnt_c  = '0;
            retry_timer_c = '0;
            armed_c       = 1'b0;
            next_state    = ST_RETRY_IDLE;
          end  //wait for a resend complete from retry fifo
          else begin
            // sec 63 #7g-2 Q3: the restart is the retransmission's own last
            // beat (armed below), not retry_complete_i, which fires ~4 cycles
            // before that beat leaves the DLL.  If the beat has already left,
            // the timer is running and keeps running.
            if (armed_r && !link_retraining_i) retry_timer_c = retry_timer_r + 1'b1;  // p.170 hold (63 #7k)
            if (retry_complete_i[i]) begin
              next_state    = ST_CNT_RETRY;
            end
          end
        end
        ST_WAIT_RETRAIN: begin
          // sec 63 #7k: REPLAY_NUM rolled over; the retrain is requested
          // (retrain_req_r) and the replay waits for it to complete (p.174).
          // The entry stays: an Ack that covers it frees it as anywhere else.
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
          // sec 63 #7k: no longer entered on rollover (ST_WAIT_RETRAIN is);
          // reachable only from `default`, an illegal encoding.  Kept, D-7K.8.
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
      // sec 63 #7k (W2d): REPLAY_NUM is ONE counter for the Transmitter
      // (sec 3.5.2.1 p.170, "The following 2-bit counter is used: REPLAY_NUM")
      // and the rollover leaves it at 00b (p.174) -- for every TLP still in
      // the retry buffer, not only the slot whose timer expired first.  This
      // design keeps one count per slot, so while ANY slot waits for the
      // retrain every slot's count is held at 00b; otherwise a second slot at
      // 11b rolls over one timer after the retrain and asks for another.
      // wait_retrain is registered state, so this adds no path between slots.
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


  assign retry_err_o       = retrain_req_r;  // 63 #7k: the retrain request (was error_r != '0)
  assign retry_available_o = !(&retrys_r);
  assign retry_index_o     = next_retry_index_r;
  assign retry_valid_o     = retry_valid_r;

endmodule
