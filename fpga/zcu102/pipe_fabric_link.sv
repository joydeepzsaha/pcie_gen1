// ---------------------------------------------------------------------------
// pipe_fabric_link -- the link between pcie_rc_top and pcie_endpoint_top
//                     when both are in one FPGA
//
// Author: Kourosh Ghahramani
// Silicon Systems Research Lab, University of Washington
//
// Purpose
//   Stands where the two transceivers and the channel would be. It has two
//   parts.
//     u_codec     pipe_codec_bridge (tb/fullstack), unchanged: the 8b/10b
//                 encoder and decoder between the Root Complex's 16 + 2 PIPE
//                 word and the Endpoint's two 10-bit Symbols, one register
//                 in each direction, with the bridge's packet error injector
//                 and DLLP blackouts.
//     sideband    for each end, the answers a PHY gives its MAC: PhyStatus
//                 with RxStatus 011b (Receiver detected) after a receiver
//                 detection request, and a receiver that leaves Electrical
//                 Idle after reset. The full-stack bench drives these from
//                 cocotb (receiver_detect and phy_presence in
//                 tb/fullstack/test_pcie_fullstack.py); here they are logic,
//                 with the bench's timing.
//
// Interfaces
//   A side        a_*: the Root Complex's PIPE interface. a_txdata_i,
//                 a_txdatak_i, a_txdata_valid_i from it; a_rxdata_o,
//                 a_rxdatak_o, a_rxvalid_o to it; a_txdetectrx_i and the
//                 status answers a_phystatus_o, a_rxstatus_o,
//                 a_rxelecidle_o, a_phystatus_rst_o.
//   B side        b_*: the Endpoint's Symbol interface, 20 bits per cycle,
//                 and the same command and status signals.
//   Injection     inj_*, starve_*: pipe_codec_bridge's ports of the same
//                 names. With inj_en_i, starve_en_i and starve_ep_en_i low
//                 the data passes unchanged.
//   Health        enc_illegal_k_o, dec_code_err_o, dec_disp_err_o: the
//                 bridge's sticky codec flags, cleared by rst_i.
//
// Clock and reset
//   One clock, clk_i, which must also clock both stacks: there is no elastic
//   buffer and no SKP handling. rst_i is active high and synchronous.
//
// Limitations
//   - One lane.
//   - Receiver detection always answers "Receiver detected", DETECT_LATENCY
//     cycles after the request, and a request made while an answer is
//     pending is not answered separately.
//   - a_rxelecidle_o and b_rxelecidle_o do not follow the other end's
//     TxElecIdle: each is high during reset and for IDLE_EXIT_CYCLES cycles
//     after it, then low until the next reset. That one falling edge is what
//     takes each LTSSM out of Detect.Quiet before its 12 ms timeout.
//   - a_phystatus_rst_o and b_phystatus_rst_o are constant 0.
//   - Polarity, PowerDown, TxElecIdle, TxCompliance and Rate requests are
//     not ports: nothing here acts on them.
//
// References
//   PG239, Table 9: Command Signals
//   PG239, Table 10: Status Signals
//   PCIe Base Spec r2.1, §4.2.6.1.1
// ---------------------------------------------------------------------------
`timescale 1ns/1ps

module pipe_fabric_link #(
    // Cycles from a rising TxDetectRx to the cycle before PhyStatus; the
    // bench's DETECT_LATENCY_CYCLES.
    parameter int unsigned DETECT_LATENCY   = 4,
    // Clock edges after reset at which each MAC still samples its receiver
    // in Electrical Idle; the bench's reset-to-presence interval.
    parameter int unsigned IDLE_EXIT_CYCLES = 6
) (
    input  logic        clk_i,
    input  logic        rst_i,

    // ---- A side: the Root Complex's PIPE interface ---------------------------
    input  logic [15:0] a_txdata_i,
    input  logic [1:0]  a_txdatak_i,
    input  logic        a_txdata_valid_i,
    output logic [15:0] a_rxdata_o,
    output logic [1:0]  a_rxdatak_o,
    output logic        a_rxvalid_o,
    input  logic        a_txdetectrx_i,
    output logic        a_phystatus_o,
    output logic [2:0]  a_rxstatus_o,
    output logic        a_rxelecidle_o,
    output logic        a_phystatus_rst_o,

    // ---- B side: the Endpoint's Symbol interface -----------------------------
    output logic [19:0] b_rx_symbol_o,
    output logic        b_rx_symbol_valid_o,
    input  logic [19:0] b_tx_symbol_i,
    input  logic        b_tx_symbol_valid_i,
    input  logic        b_txdetectrx_i,
    output logic        b_phystatus_o,
    output logic [2:0]  b_rxstatus_o,
    output logic        b_rxelecidle_o,
    output logic        b_phystatus_rst_o,

    // ---- pipe_codec_bridge's injection ports ---------------------------------
    input  logic        inj_en_i,
    input  logic [7:0]  inj_pkt_i,
    input  logic [1:0]  inj_mode_i,
    input  logic [7:0]  inj_off_i,
    input  logic [3:0]  inj_bit_i,
    output logic        inj_fired_o,
    output logic [7:0]  inj_end_byte_o,
    input  logic        starve_en_i,
    output logic [15:0] starve_cnt_o,
    input  logic        starve_ep_en_i,
    output logic [15:0] starve_ep_cnt_o,

    // ---- pipe_codec_bridge's sticky codec flags ------------------------------
    output logic        enc_illegal_k_o,
    output logic        dec_code_err_o,
    output logic        dec_disp_err_o
);

  localparam logic [2:0] RXSTATUS_RECEIVER_DETECTED = 3'b011;   // PG239, Table 10
  localparam int unsigned DET_W  = $clog2(DETECT_LATENCY + 1);
  localparam int unsigned IDLE_W = $clog2(IDLE_EXIT_CYCLES + 1);

  // ---------------------------------------------------------------------------
  // Data: the bench's bridge
  // ---------------------------------------------------------------------------
  pipe_codec_bridge #(
      .MAX_NUM_LANES  (1),
      .PIPE_DATA_WIDTH(16)
  ) u_codec (
      .clk_i(clk_i),
      .rst_i(rst_i),

      .a_txdata_i      (a_txdata_i),
      .a_txdatak_i     (a_txdatak_i),
      .a_txdata_valid_i(a_txdata_valid_i),
      .a_rxdata_o      (a_rxdata_o),
      .a_rxdatak_o     (a_rxdatak_o),
      .a_rxdata_valid_o(a_rxvalid_o),

      .b_rx_symbol_o      (b_rx_symbol_o),
      .b_rx_symbol_valid_o(b_rx_symbol_valid_o),
      .b_tx_symbol_i      (b_tx_symbol_i),
      .b_tx_symbol_valid_i(b_tx_symbol_valid_i),

      .inj_en_i      (inj_en_i),
      .inj_pkt_i     (inj_pkt_i),
      .inj_mode_i    (inj_mode_i),
      .inj_off_i     (inj_off_i),
      .inj_bit_i     (inj_bit_i),
      .inj_fired_o   (inj_fired_o),
      .inj_end_byte_o(inj_end_byte_o),

      .starve_en_i    (starve_en_i),
      .starve_cnt_o   (starve_cnt_o),
      .starve_ep_en_i (starve_ep_en_i),
      .starve_ep_cnt_o(starve_ep_cnt_o),

      .enc_illegal_k_o(enc_illegal_k_o),
      .dec_code_err_o (dec_code_err_o),
      .dec_disp_err_o (dec_disp_err_o)
  );

  // ---------------------------------------------------------------------------
  // Receiver detection
  // ---------------------------------------------------------------------------
  // Each MAC clears its detect latch on the rising edge of its own TxDetectRx
  // and sets it on a later PhyStatus with RxStatus 011b (pcie_phy_top for the
  // Root Complex, pcie_endpoint_top for the Endpoint). The answer is one
  // cycle of PhyStatus, sampled by the MAC DETECT_LATENCY + 1 clock edges
  // after the edge at which it drove the request. det_cnt counts the
  // latency; a request is taken only when no answer is pending or being
  // given. Index 0 is the A side, index 1 the B side.
  logic [1:0]       txdetectrx;
  logic [1:0]       txdetectrx_q;
  logic [1:0]       phystatus;
  logic [DET_W-1:0] det_cnt [2];

  assign txdetectrx = {b_txdetectrx_i, a_txdetectrx_i};

  always_ff @(posedge clk_i) begin
    if (rst_i) begin
      txdetectrx_q <= '0;
      phystatus    <= '0;
      for (int e = 0; e < 2; e++) det_cnt[e] <= '0;
    end else begin
      txdetectrx_q <= txdetectrx;
      for (int e = 0; e < 2; e++) begin
        phystatus[e] <= (det_cnt[e] == DET_W'(1));
        if (det_cnt[e] != '0) begin
          det_cnt[e] <= det_cnt[e] - 1'b1;
        end else if (txdetectrx[e] && !txdetectrx_q[e] && !phystatus[e]) begin
          det_cnt[e] <= DET_W'(DETECT_LATENCY);
        end
      end
    end
  end

  assign a_phystatus_o = phystatus[0];
  assign b_phystatus_o = phystatus[1];
  assign a_rxstatus_o  = phystatus[0] ? RXSTATUS_RECEIVER_DETECTED : 3'b000;
  assign b_rxstatus_o  = phystatus[1] ? RXSTATUS_RECEIVER_DETECTED : 3'b000;

  // ---------------------------------------------------------------------------
  // Electrical Idle and PHY reset status
  // ---------------------------------------------------------------------------
  // Both receivers report Electrical Idle during reset and at the first
  // IDLE_EXIT_CYCLES clock edges after it, then a live partner. Each LTSSM takes
  // the falling edge as the exit from Electrical Idle that ends Detect.Quiet
  // (PCIe Base Spec r2.1, §4.2.6.1.1); it must see the level high after its
  // own reset first, which is why the level is held.
  logic [IDLE_W-1:0] idle_cnt;
  logic              rxelecidle;

  always_ff @(posedge clk_i) begin
    if (rst_i) begin
      idle_cnt   <= '0;
      rxelecidle <= 1'b1;
    end else if (idle_cnt != IDLE_W'(IDLE_EXIT_CYCLES - 1)) begin
      idle_cnt   <= idle_cnt + 1'b1;
    end else begin
      rxelecidle <= 1'b0;
    end
  end

  assign a_rxelecidle_o    = rxelecidle;
  assign b_rxelecidle_o    = rxelecidle;
  assign a_phystatus_rst_o = 1'b0;
  assign b_phystatus_rst_o = 1'b0;

endmodule
