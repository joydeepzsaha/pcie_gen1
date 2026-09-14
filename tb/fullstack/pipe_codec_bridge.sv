// ===========================================================================
// pipe_codec_bridge -- the 8b/10b gap between the RC's PIPE seam and the EP's.
//
// BENCH INFRASTRUCTURE, NOT RTL. It lives in tb/ because nothing on either
// stack's boundary changes to accommodate it, and because a real link has a
// transceiver here rather than this.
//
// == WHAT THE TWO SEAMS ACTUALLY ARE (SS63 #7b Phase 0, measured) ============
//
// The two stacks terminate at different LAYERS but at the SAME RATE, and the
// second half of that sentence was not known when this bridge was scoped.
//
//   A side -- pcie_rc_top       phy_txdata[31:0] + phy_txdatak[3:0]   pre-codec
//   B side -- pcie_endpoint_top phy_tx_symbol_o[19:0]                 post-codec
//
// !! THE 32-BIT PORT CARRIES 16 LIVE BITS AT GEN1, AND THAT IS MEASURED.
// lane_management.sv:45 sets PipeWidthGen1 = 16 BITS, so pipe_width_r >> 3 is
// 2 bytes, and both states that emit live data (ST_LANE_MNGT_TX_DATA and
// ST_LANE_MNGT_TX_PHY) assign data_out_c[lane*32 +: 32] = '0 and then fill only
// byte_ < 2.  Measured at the artifact over a 37,721-cycle post-L0 window:
// phy_txdata[31:16] non-zero on ZERO cycles, phy_txdatak[3:2] on ZERO.
//
// So both seams carry TWO characters per valid beat.  THERE IS NO 2:1 GEARBOX
// AND NO 2x CLOCK -- the width conversion this bridge was scoped to contain
// does not exist.  It is a codec, and nothing else.
//
// !! The upper half is DRIVEN TO ZERO on the A-side output rather than left
// undriven, because the RC's phy_receive reads all 32 bits and an X there
// propagates into block_alignment.
//
// == DISPARITY IS THE ONLY WAY TO GET THIS WRONG ============================
//
// !! RUNNING DISPARITY ADVANCES ONLY ON A VALID BEAT.  The EP updates its own
// tx_running_disparity / rx_running_disparity only when the corresponding valid
// is high (pcie_endpoint_top.sv:488-503).  A bridge that advances on an invalid
// beat desynchronises from it, and EVERY subsequent symbol decodes as a
// disparity error -- a failure that looks like a codec bug and is a gating bug.
// Both disparity registers here are therefore gated on their own side's valid,
// and reset to 0, which is what the EP resets to.
//
// Symbol ordering needs no swap: symbol s occupies [lane*20 + s*10 +: 10] and
// carries byte [lane*32 + s*8 +: 8] with K flag [lane*4 + s], on both sides.
// Taken from the EP's own codec instantiation (pcie_endpoint_top.sv:468-485) so
// the two cannot disagree.
//
// == LATENCY ================================================================
//
// One clock each direction, deliberately.  The codec itself is combinational,
// so a zero-latency bridge is available and is the wrong choice twice over: it
// puts the RC's transmit cone and the EP's receive cone in one combinational
// path, and a zero-latency model is blind to ordering.  Row 1's existing
// loopback registers for the same reason.
//
// == WHAT THIS IS NOT =======================================================
//
// !! NO ELASTIC BUFFER AND NO SKP HANDLING.  Both ends run from one clock in
// this bench (D-7B.1), so there is no ppm offset to absorb and no reason for a
// SKP to be added or removed.  On real hardware the transceiver does this and
// the clocks are only plesiochronous; a bench that pretended otherwise would be
// modelling a mechanism it cannot exercise.  Stated here because the absence is
// a scope decision, not an oversight.
//
// !! NO SCRAMBLING.  Both ends already scramble and descramble internally
// (phy_transmit:153 / phy_receive:143, D-FS.3).  Adding it here would apply it
// twice.
// ===========================================================================

module pipe_codec_bridge #(
    parameter int MAX_NUM_LANES = 1
) (
    input  logic clk_i,
    input  logic rst_i,

    // ---- A side: the RC's 32+4 plaintext PIPE seam -------------------------
    input  logic [(MAX_NUM_LANES*32)-1:0] a_txdata_i,
    input  logic [(MAX_NUM_LANES*4)-1:0]  a_txdatak_i,
    input  logic [MAX_NUM_LANES-1:0]      a_txdata_valid_i,
    output logic [(MAX_NUM_LANES*32)-1:0] a_rxdata_o,
    output logic [(MAX_NUM_LANES*4)-1:0]  a_rxdatak_o,
    output logic [MAX_NUM_LANES-1:0]      a_rxdata_valid_o,

    // ---- B side: the EP's 20-bit encoded symbol seam -----------------------
    output logic [(MAX_NUM_LANES*20)-1:0] b_rx_symbol_o,
    output logic [MAX_NUM_LANES-1:0]      b_rx_symbol_valid_o,
    input  logic [(MAX_NUM_LANES*20)-1:0] b_tx_symbol_i,
    input  logic [MAX_NUM_LANES-1:0]      b_tx_symbol_valid_i,

    // ---- observability: the codec's own error pins, valid-gated ------------
    // Sticky, because a row that samples them on one cycle is a row with a
    // phase (SS22.89).  Cleared only by rst_i.
    output logic [MAX_NUM_LANES-1:0] enc_illegal_k_o,
    output logic [MAX_NUM_LANES-1:0] dec_code_err_o,
    output logic [MAX_NUM_LANES-1:0] dec_disp_err_o
);

  for (genvar lane = 0; lane < MAX_NUM_LANES; lane++) begin : gen_lane
    // Disparity chains: index 0 is the register, index 2 is the value after
    // both symbols.  Same shape as the EP's, so the two advance identically.
    logic [2:0] enc_disp;
    logic [2:0] dec_disp;
    logic       enc_disp_r;
    logic       dec_disp_r;

    logic [1:0] enc_illegal_k;
    logic [1:0] dec_code_err;
    logic [1:0] dec_disp_err;

    logic [19:0] enc_symbol_c;
    logic [15:0] dec_data_c;
    logic [ 1:0] dec_k_c;

    assign enc_disp[0] = enc_disp_r;
    assign dec_disp[0] = dec_disp_r;

    for (genvar symbol = 0; symbol < 2; symbol++) begin : gen_symbol
      encode_8b10b bridge_encoder_inst (
          .datain     ({a_txdatak_i[lane*4+symbol],
                        a_txdata_i[lane*32+symbol*8 +: 8]}),
          .dispin     (enc_disp[symbol]),
          .dataout    (enc_symbol_c[symbol*10 +: 10]),
          .dispout    (enc_disp[symbol+1]),
          .illegal_k_o(enc_illegal_k[symbol])
      );

      decode_8b10b bridge_decoder_inst (
          .datain  (b_tx_symbol_i[lane*20+symbol*10 +: 10]),
          .dispin  (dec_disp[symbol]),
          .dataout ({dec_k_c[symbol], dec_data_c[symbol*8 +: 8]}),
          .dispout (dec_disp[symbol+1]),
          .code_err(dec_code_err[symbol]),
          .disp_err(dec_disp_err[symbol])
      );
    end

    // ---- A -> B: encode, register, pass the valid alongside ---------------
    always_ff @(posedge clk_i) begin
      if (rst_i) begin
        enc_disp_r                 <= 1'b0;
        b_rx_symbol_o[lane*20+:20] <= '0;
        b_rx_symbol_valid_o[lane]  <= 1'b0;
        enc_illegal_k_o[lane]      <= 1'b0;
      end else begin
        b_rx_symbol_o[lane*20+:20] <= enc_symbol_c;
        b_rx_symbol_valid_o[lane]  <= a_txdata_valid_i[lane];
        // !! Gated on the valid: see the header.
        if (a_txdata_valid_i[lane]) begin
          enc_disp_r <= enc_disp[2];
          if (|enc_illegal_k) enc_illegal_k_o[lane] <= 1'b1;
        end
      end
    end

    // ---- B -> A: decode, register, zero the unused upper half -------------
    always_ff @(posedge clk_i) begin
      if (rst_i) begin
        dec_disp_r                 <= 1'b0;
        a_rxdata_o[lane*32+:32]    <= '0;
        a_rxdatak_o[lane*4+:4]     <= '0;
        a_rxdata_valid_o[lane]     <= 1'b0;
        dec_code_err_o[lane]       <= 1'b0;
        dec_disp_err_o[lane]       <= 1'b0;
      end else begin
        a_rxdata_o[lane*32+:32] <= {16'h0000, dec_data_c};
        a_rxdatak_o[lane*4+:4]  <= {2'b00, dec_k_c};
        a_rxdata_valid_o[lane]  <= b_tx_symbol_valid_i[lane];
        if (b_tx_symbol_valid_i[lane]) begin
          dec_disp_r <= dec_disp[2];
          if (|dec_code_err) dec_code_err_o[lane] <= 1'b1;
          if (|dec_disp_err) dec_disp_err_o[lane] <= 1'b1;
        end
      end
    end
  end

endmodule
