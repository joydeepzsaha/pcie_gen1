// ===========================================================================
// tb_pipe_codec_bridge -- the bridge ALONE, with its B side looped back.
//
// SS63 #7b Phase 2's probe rows. The brief requires the bridge to be proven
// before it touches a DUT, and this is the cheapest arrangement that can do it:
// tie b_tx_symbol_i to b_rx_symbol_o, so a character driven into the A side is
// encoded, decoded and returned to the A side, exercising BOTH codecs and BOTH
// running-disparity chains on one stimulus.
//
// !! THE LOOPBACK IS THE POINT, NOT A SHORTCUT, and it does not have the defect
// the row-1 PIPE loopback has. That one fails because phy_transmit SCRAMBLES
// and a self-loop puts one LFSR against its own output. There is no scrambler
// in this bridge (D-FS.3 -- both stacks scramble internally), so encode
// followed by decode is an identity on the character stream and a round trip is
// a real check rather than a self-defeating one.
//
// !! THE TWO DISPARITY CHAINS ARE SEPARATE REGISTERS AND THAT IS WHAT MAKES THE
// THIRD ROW MEANINGFUL. enc_disp_r and dec_disp_r are independent; they stay in
// lockstep only because both reset to 0 and both advance on the same beats with
// the same symbols. A row that watches the decoder's disp_err over a long run
// is therefore watching exactly the failure the EP would suffer if this bridge
// gated its disparity wrongly -- which is the one way to get the bridge wrong.
//
// Round-trip latency is TWO cycles: one for the encode register, one for the
// decode register.
// ===========================================================================

module tb_pipe_codec_bridge #(
    parameter int MAX_NUM_LANES = 1
) (
    input  logic clk_i,
    input  logic rst_i,

    input  logic [(MAX_NUM_LANES*32)-1:0] a_txdata_i,
    input  logic [(MAX_NUM_LANES*4)-1:0]  a_txdatak_i,
    input  logic [MAX_NUM_LANES-1:0]      a_txdata_valid_i,

    output logic [(MAX_NUM_LANES*32)-1:0] a_rxdata_o,
    output logic [(MAX_NUM_LANES*4)-1:0]  a_rxdatak_o,
    output logic [MAX_NUM_LANES-1:0]      a_rxdata_valid_o,

    // The encoded seam, exposed so a row can assert about the SYMBOLS rather
    // than only about the round trip -- otherwise an encoder and decoder that
    // were wrong in exactly opposite ways would round-trip perfectly.
    output logic [(MAX_NUM_LANES*20)-1:0] b_symbol_o,
    output logic [MAX_NUM_LANES-1:0]      b_symbol_valid_o,

    output logic [MAX_NUM_LANES-1:0] enc_illegal_k_o,
    output logic [MAX_NUM_LANES-1:0] dec_code_err_o,
    output logic [MAX_NUM_LANES-1:0] dec_disp_err_o
);

  logic [(MAX_NUM_LANES*20)-1:0] symbol;
  logic [MAX_NUM_LANES-1:0]      symbol_valid;

  assign b_symbol_o       = symbol;
  assign b_symbol_valid_o = symbol_valid;

  pipe_codec_bridge #(
      .MAX_NUM_LANES(MAX_NUM_LANES)
  ) u_bridge (
      .clk_i(clk_i),
      .rst_i(rst_i),

      .a_txdata_i      (a_txdata_i),
      .a_txdatak_i     (a_txdatak_i),
      .a_txdata_valid_i(a_txdata_valid_i),
      .a_rxdata_o      (a_rxdata_o),
      .a_rxdatak_o     (a_rxdatak_o),
      .a_rxdata_valid_o(a_rxdata_valid_o),

      // ---- the loopback ----------------------------------------------------
      .b_rx_symbol_o      (symbol),
      .b_rx_symbol_valid_o(symbol_valid),
      .b_tx_symbol_i      (symbol),
      .b_tx_symbol_valid_i(symbol_valid),

      .enc_illegal_k_o(enc_illegal_k_o),
      .dec_code_err_o (dec_code_err_o),
      .dec_disp_err_o (dec_disp_err_o)
  );

endmodule
