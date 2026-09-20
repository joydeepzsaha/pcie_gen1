module scrambler
  import pcie_phy_pkg::*;
(

    input  logic               clk_i,             //! 100MHz clock signal
    input  logic               rst_i,             //! Reset signal
    input  logic        [ 7:0] lane_number,
    input  logic        [ 1:0] sync_header_i,
    input  rate_speed_e        curr_data_rate_i,
    input  logic        [31:0] data_in_i,
    input  logic               block_start_i,
    input  logic               data_valid_i,
    output logic               data_valid_o,
    output logic        [31:0] data_out_o,
    input  logic        [ 3:0] data_k_in_i,
    input  logic        [ 5:0] pipe_width_i,
    output logic        [ 3:0] data_k_out_o,
    output logic        [ 1:0] sync_header_o,
    output logic               block_start_o
    // !Control
);


  logic [3:0] gen1_data_k;
  logic [31:0] gen1_data;
  logic gen1_valid;

  /*
   * Future Gen3 scrambling path.
   *
   * The integrated endpoint is currently Gen1-only, so this path is kept as
   * commented source and is not elaborated during endpoint synthesis or test.
   * Restore these declarations together with gen3_scramble_inst and the rate
   * selection mux below when a separately verified Gen3 PHY is enabled.
   *
  logic [3:0] gen3_data_k;
  logic [31:0] gen3_data;
  logic gen3_valid;

  gen3_scramble gen3_scramble_inst (
      .clk_i(clk_i),
      .rst_i(rst_i),
      .lane_number(lane_number),
      .sync_header_i(sync_header_i),
      .data_in_i(data_in_i),
      .data_valid_i(data_valid_i),
      .ltssm_polling_compliance_i('0),
      .data_valid_o(gen3_valid),
      .data_out_o(gen3_data),
      .data_k_in_i(data_k_in_i),
      .pipe_width_i(pipe_width_i),
      .data_k_out_o(gen3_data_k)
  );
  */

  gen1_scramble gen1_scramble_inst (
      .clk_i(clk_i),
      .rst_i(rst_i),
      .data_in_i(data_in_i),
      .data_valid_i(data_valid_i),
      .data_valid_o(gen1_valid),
      .data_out_o(gen1_data),
      .data_k_in_i(data_k_in_i),
      .pipe_width_i(pipe_width_i),
      .data_k_out_o(gen1_data_k)
  );

  // ⭐ §63 #7h (#21) B.  THE PUBLISHED VALID IS THE CHAIN'S OWN LAST-STAGE BIT.
  //
  // This used to be `data_valid_o <= data_valid_i`, a 1-stage register copy,
  // while the data needed three shift hops to cross gen1_scramble's 4-stage
  // chain.  The two disagreed at both ends of every packet:
  //
  //   at the START  the first 3 published beats carried whatever the chain was
  //                 holding, not the Symbols just presented -- the `fabricated`
  //                 beats test_scrambler_align.py's clause (d) records;
  //   at the END    valid fell while 3 real words were still in the chain, so
  //                 they were never announced.  Commit A makes the chain drain
  //                 them to data_out_o; without this commit nobody is told.
  //
  // `gen1_valid` is Q.data_valid[NumPipelines-1] -- the valid bit that rode the
  // chain alongside its own word.  Publishing it makes the announcement and the
  // word the same event by construction, which is what acceptance (i) and (ii)
  // assert: a lone packet publishes its END, and the STP->END span does not
  // depend on what follows the packet.
  //
  // ⚠️ THIS IS NOT THE :91 UNCOMMENT, AND THE DIFFERENCE IS COMMIT A.
  // Publishing gen1_valid while the chain stayed FROZEN is candidate A of the
  // O-ALIGN pair, and verilate_gen1_align carries two expect_fail rows
  // MEASURING that it republishes one stale word per idle clock and swallows a
  // real Symbol on resume.  Both are properties of a frozen chain: a valid bit
  // held high over a pipeline that is not moving.  Commit A makes the chain
  // move whenever it holds a word, so that held-high valid now accompanies real
  // words instead of stale ones.  The line at :91 stays commented -- it sits
  // inside the disabled Gen3 rate mux and would publish gen3_valid, a different
  // signal from a path this endpoint does not elaborate.
  //
  // ⚠️ sync_header_o and block_start_o KEEP the 1-stage copy.  Neither rides the
  // chain (gen1_scramble carries no sync_header or block_start field), so
  // depth-matching them here would be inventing an alignment the data path does
  // not have.  Gen1 does not use sync headers at all -- they are the Gen3
  // 128b/130b construct -- so this is left exactly as found rather than "made
  // consistent".
  always_ff @(posedge clk_i) begin
    if (rst_i) begin
      sync_header_o <= '0;
      // data_k_out_o  <= '0;
      block_start_o <= '0;
    end else begin
      sync_header_o <= sync_header_i;
      // data_k_out_o  <= data_k_in_i;
      block_start_o <= block_start_i;
    end
  end

  assign data_valid_o = gen1_valid;

  always_comb begin
    // The endpoint physical path is fixed to the functional Gen1 scrambler.
    data_k_out_o = gen1_data_k;
    data_out_o   = gen1_data;

    /* Future Gen3 rate-selection mux; intentionally disabled for Gen1.
    if (curr_data_rate_i < gen3) begin
      data_k_out_o = gen1_data_k;
      data_out_o = gen1_data;
      // data_valid_o = gen1_valid;
    end else begin
      data_k_out_o = gen3_data_k;
      data_out_o = gen3_data;
      // data_valid_o = gen3_valid;
    end
    */
  end





endmodule
