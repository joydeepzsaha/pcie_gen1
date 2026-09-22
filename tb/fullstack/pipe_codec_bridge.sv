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

  // =========================================================================
  // sec 63 #7i, D-7I.3 -- BENCH-ONLY single-bit error injection.
  //
  // The only way to exercise the far end's Nak/replay path against a REAL
  // partner rather than against a cocotb driver.  It is here, in the bridge,
  // because the bridge is where a transceiver would be: corruption between two
  // stacks, with neither stack's RTL changed.
  //
  // !! OFF UNLESS A PLUSARG TURNS IT ON, and off means bit-identical.  With no
  // +PR7I_INJ the mask is constant zero and a_txdata_eff === a_txdata_i, so
  // verilate_pipe_bridge and verilate_fullstack see the design they saw before.
  // A parameter would have done the same job and would have had to be threaded
  // through tb_pcie_fullstack's own parameter list to be settable per test;
  // the plusarg keeps every port list and parameter list unchanged.
  //
  // !! THE FLIP IS ON THE PRE-CODEC SIDE, DELIBERATELY.  Flipping a bit of the
  // encoded 10-bit symbol produces a code violation or a disparity error --
  // the decoder's own error pins would catch it and the LCRC would never be
  // consulted.  Flipping a pre-encode data bit produces a LEGAL symbol
  // carrying wrong data, which is exactly the fault the LCRC exists to catch.
  //
  //   +PR7I_INJ=1        arm
  //   +PR7I_PKT=<n>      flip inside the n'th STP-framed packet (1-based)
  //   +PR7I_OFF=<k>      k valid beats after that STP beat (k >= 1)
  //   +PR7I_BIT=<b>      which bit of a_txdata_i[15:0] to flip (0..15)
  //
  // STP is K27.7 = 8'hFB (pcie_phy_pkg).  Named here as a literal rather than
  // imported so the bridge keeps depending on nothing but the codecs.
  // =========================================================================
  localparam logic [7:0] SYM_STP = 8'hFB;

  int unsigned inj_en = 0, inj_pkt = 0, inj_off = 0, inj_bit = 0;
  initial begin
    if (!$value$plusargs("PR7I_INJ=%d", inj_en)) inj_en = 0;
    if (!$value$plusargs("PR7I_PKT=%d", inj_pkt)) inj_pkt = 1;
    if (!$value$plusargs("PR7I_OFF=%d", inj_off)) inj_off = 1;
    if (!$value$plusargs("PR7I_BIT=%d", inj_bit)) inj_bit = 0;
    if (inj_en != 0)
      $display("PR7I_INJ_ARM pkt=%0d off=%0d bit=%0d", inj_pkt, inj_off, inj_bit);
  end

  logic [(MAX_NUM_LANES*32)-1:0] a_txdata_eff;
  logic [(MAX_NUM_LANES*32)-1:0] inj_mask;

  // Lane 0 only: this bench is x1 (MAX_NUM_LANES=1) and a multi-lane injector
  // would have to decide which lane the packet's STP landed in -- a decision
  // with no test behind it.
  logic        a_stp_now;
  int unsigned stp_cnt, beat_cnt;
  logic        armed;

  assign a_stp_now = a_txdata_valid_i[0] &&
                     ((a_txdatak_i[0] && (a_txdata_i[7:0]  == SYM_STP)) ||
                      (a_txdatak_i[1] && (a_txdata_i[15:8] == SYM_STP)));

  always_ff @(posedge clk_i) begin
    if (rst_i) begin
      stp_cnt  <= 0;
      beat_cnt <= 0;
      armed    <= 1'b0;
    end else if (inj_en != 0) begin
      if (a_stp_now) begin
        stp_cnt  <= stp_cnt + 1;
        beat_cnt <= 0;
        armed    <= ((stp_cnt + 1) == inj_pkt);
      end else if (armed && a_txdata_valid_i[0]) begin
        beat_cnt <= beat_cnt + 1;
        if ((beat_cnt + 1) == inj_off) armed <= 1'b0;   // fire once, then disarm
      end
    end
  end

  // The mask is combinational and one beat wide.  It is zero whenever the
  // injector is disarmed, which includes every cycle of every run that did not
  // pass +PR7I_INJ.
  always_comb begin
    inj_mask = '0;
    if ((inj_en != 0) && armed && a_txdata_valid_i[0] &&
        ((beat_cnt + 1) == inj_off)) begin
      inj_mask[inj_bit[3:0]] = 1'b1;
    end
  end

  assign a_txdata_eff = a_txdata_i ^ inj_mask;

  always_ff @(posedge clk_i) begin
    if (!rst_i && (|inj_mask))
      $display("PR7I_INJ_FIRE t=%0t stp_cnt=%0d beat=%0d bit=%0d orig=0x%04h sent=0x%04h",
               $time, stp_cnt, beat_cnt + 1, inj_bit,
               a_txdata_i[15:0], a_txdata_eff[15:0]);
  end

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
                        a_txdata_eff[lane*32+symbol*8 +: 8]}),
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
