// ---------------------------------------------------------------------------
// retry_transmit -- retry buffer storage and replay output
//
//! module: retry_transmit
//! Author: Idris Somoye
//
// Purpose
//   Holds the retry buffer: RETRY_TLP_SIZE slots, one axis_retry_fifo each.
//   Every framed TLP from tlp2dllp is written into the slot retry_management
//   names, and stays there until it is overwritten. On a replay request from
//   retry_management, the requesting slot's frame is played out on m_axis.
//
// Interfaces
//   Replay        retry_valid_i: one request bit per slot, from
//                 retry_management. retry_ack_o: one cycle per accepted
//                 request, a cycle after the selection. retry_complete_o: one
//                 cycle as the replayed frame's last beat is accepted.
//   Slot select   retry_available_i, retry_index_i: the write enable and the
//                 slot to write, from retry_management.
//   Frame input   s_axis_*: tlp2dllp's output, observed. s_axis_tready is
//                 constant 1 and dllp_transmit leaves it unconnected.
//   Replay out    m_axis_*: one stored frame per accepted request, to
//                 dllp_transmit's arbiter.
//
// Clock and reset
//   clk_i only. rst_i is synchronous and active high. retry_index_r has no
//   reset; it is read only in ST_TLP_GET_COUNT, after ST_TLP_RX_IDLE sets it.
//
// Limitations
//   Requests are served lowest slot index first. That is the original
//   transmission order only when the slots were filled in index order;
//   PCIe Base Spec r2.1, §3.5.2.1 replays the oldest TLP first.
//   ST_TLP_GET_ADDR, ST_TLP_RX_SOP, ST_TLP_RX_STREAM and ST_TLP_RX_EOP are
//   declared and never entered. S_COUNT, RAM_DATA_WIDTH and RAM_ADDR_WIDTH
//   are not used.
//
// References
//   PCIe Base Spec r2.1, §3.5.2.1
// ---------------------------------------------------------------------------
module retry_transmit
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

    parameter int RAM_ADDR_WIDTH = $clog2(RAM_DATA_WIDTH)
) (
    input  logic                      clk_i,
    input  logic                      rst_i,
    input  logic [RETRY_TLP_SIZE-1:0] retry_valid_i,
    output logic [RETRY_TLP_SIZE-1:0] retry_ack_o,
    output logic [RETRY_TLP_SIZE-1:0] retry_complete_o,
    // ---- slot select, from retry_management --------------------------------
    input  logic                      retry_available_i,
    input  logic [               7:0] retry_index_i,
    // ---- frames to store ---------------------------------------------------
    input  logic [  (DATA_WIDTH)-1:0] s_axis_tdata,
    input  logic [  (KEEP_WIDTH)-1:0] s_axis_tkeep,
    input  logic                      s_axis_tvalid,
    input  logic                      s_axis_tlast,
    input  logic [    USER_WIDTH-1:0] s_axis_tuser,
    output logic                      s_axis_tready,
    // ---- replay output -----------------------------------------------------
    output logic [  (DATA_WIDTH)-1:0] m_axis_tdata,
    output logic [  (KEEP_WIDTH)-1:0] m_axis_tkeep,
    output logic                      m_axis_tvalid,
    output logic                      m_axis_tlast,
    output logic [    USER_WIDTH-1:0] m_axis_tuser,
    input  logic                      m_axis_tready
);


  // ST_TLP_RX_IDLE picks a request; ST_TLP_GET_COUNT plays the frame out.
  // The other four states are not used.
  typedef enum logic [2:0] {
    ST_TLP_RX_IDLE,
    ST_TLP_GET_COUNT,
    ST_TLP_GET_ADDR,
    ST_TLP_RX_SOP,
    ST_TLP_RX_STREAM,
    ST_TLP_RX_EOP
  } tlp_rx_st_e;

  //!state signals
  tlp_rx_st_e                      tlp_curr_state;
  tlp_rx_st_e                      tlp_next_state;
  //!internal signals
  logic       [RETRY_TLP_SIZE-1:0] retry_ack_c;
  logic       [RETRY_TLP_SIZE-1:0] retry_ack_r;
  logic       [RETRY_TLP_SIZE-1:0] retry_ready;
  logic       [RETRY_TLP_SIZE-1:0] mutex_flag;
  logic       [    DATA_WIDTH-1:0] retry_axis_tdata [RETRY_TLP_SIZE];
  logic       [    KEEP_WIDTH-1:0] retry_axis_tkeep [RETRY_TLP_SIZE];
  logic                            retry_axis_tvalid[RETRY_TLP_SIZE];
  logic                            retry_axis_tlast [RETRY_TLP_SIZE];
  logic       [    USER_WIDTH-1:0] retry_axis_tuser [RETRY_TLP_SIZE];
  logic                            retry_axis_tready[RETRY_TLP_SIZE];
  logic       [               7:0] retry_index_c;
  logic       [               7:0] retry_index_r;

  //! main sequential block
  always_ff @(posedge clk_i) begin : main_sequential_block
    if (rst_i) begin
      tlp_curr_state <= ST_TLP_RX_IDLE;
      retry_ack_r <= '0;
    end else begin
      tlp_curr_state <= tlp_next_state;
      retry_ack_r <= retry_ack_c;
    end
    retry_index_r <= retry_index_c;
  end

  //!retry generate loop
  // Every slot sees every beat; only the slot at retry_index_i, while a slot
  // is free, has its write enabled. The enable is s_axis_tvalid without a
  // ready: dllp_transmit does not pass its arbiter's ready here, so a beat
  // the arbiter holds is written again.
  for (genvar i = 0; i < RETRY_TLP_SIZE; i++) begin : gen_retry_axis_fifo
    axis_retry_fifo #(
        .DATA_WIDTH(DATA_WIDTH),
        .STRB_WIDTH(STRB_WIDTH),
        .KEEP_WIDTH(KEEP_WIDTH),
        .USER_WIDTH(USER_WIDTH),
        .MAX_PAYLOAD_SIZE(MAX_PAYLOAD_SIZE)
    ) axis_retry_fifo_inst (
        .clk_i(clk_i),
        .rst_i(rst_i),
        .s_axis_tdata(s_axis_tdata),
        .s_axis_tkeep(s_axis_tkeep),
        .s_axis_tvalid(s_axis_tvalid & retry_available_i & (retry_index_i == i)),
        .s_axis_tlast(s_axis_tlast),
        .s_axis_tuser(s_axis_tuser),
        .s_axis_tready(),
        .m_axis_tdata(retry_axis_tdata[i]),
        .m_axis_tkeep(retry_axis_tkeep[i]),
        .m_axis_tvalid(retry_axis_tvalid[i]),
        .m_axis_tlast(retry_axis_tlast[i]),
        .m_axis_tuser(retry_axis_tuser[i]),
        .m_axis_tready(retry_ready[i])
    );

  end


  //!transmit tlp from fifo through axi stream
  always_comb begin : transmit_tlp_combo
    tlp_next_state   = tlp_curr_state;
    retry_ready      = '0;
    m_axis_tdata     = '0;
    m_axis_tkeep     = '0;
    m_axis_tvalid    = '0;
    m_axis_tlast     = '0;
    m_axis_tuser     = '0;
    retry_complete_o = '0;
    // Acknowledgement is a pulse for each accepted replay request.
    retry_ack_c      = '0;
    retry_index_c    = retry_index_r;
    mutex_flag       = '0;
    case (tlp_curr_state)
      ST_TLP_RX_IDLE: begin
        if ((retry_valid_i != '0)) begin
          // The lowest requesting index wins: mutex_flag[i] marks a request
          // that has a lower-index competitor.
          for (int i = 0; i < RETRY_TLP_SIZE; i++) begin
            mutex_flag[i] = 1'b0;
            if (retry_valid_i[i]) begin
              for (int j = 0; j < RETRY_TLP_SIZE; j++) begin
                if (retry_valid_i[j] && j < i) begin
                  mutex_flag[i] = 1'b1;
                end
              end
              if (!mutex_flag[i]) begin
                retry_ack_c[i] = '1;
                retry_index_c  = i;
              end
            end
          end
          tlp_next_state = ST_TLP_GET_COUNT;
        end
      end
      // Plays the selected slot's frame. The FIFO rewinds after the last beat,
      // so the frame stays stored for a later replay.
      ST_TLP_GET_COUNT: begin
        retry_ready[retry_index_r] = m_axis_tready;
        m_axis_tdata               = retry_axis_tdata[retry_index_r];
        m_axis_tkeep               = retry_axis_tkeep[retry_index_r];
        m_axis_tvalid              = retry_axis_tvalid[retry_index_r];
        m_axis_tlast               = retry_axis_tlast[retry_index_r];
        m_axis_tuser               = retry_axis_tuser[retry_index_r];
        if (m_axis_tlast && m_axis_tvalid && m_axis_tready) begin
          retry_complete_o[retry_index_r] = '1;
          tlp_next_state                  = ST_TLP_RX_IDLE;
        end
      end
      default: begin
      end
    endcase
  end

  assign s_axis_tready = '1;
  assign retry_ack_o   = retry_ack_r;

endmodule
