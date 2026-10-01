// ---------------------------------------------------------------------------
// scrambler -- per-lane wrapper around gen1_scramble
//
// Purpose
//   Scrambles or descrambles one lane. phy_transmit instantiates it per lane
//   as the scrambler and phy_receive as the descrambler, which is the same
//   operation because the XOR with the LFSR is its own inverse. The data and
//   K outputs come from gen1_scramble. The valid, sync header and block start
//   outputs are the inputs delayed by one register.
//
// Interfaces
//   Data in    data_in_i, data_k_in_i, data_valid_i, pipe_width_i: passed to
//              gen1_scramble.
//   Data out   data_out_o, data_k_out_o: gen1_scramble's outputs.
//              data_valid_o: data_valid_i delayed one clock.
//   Gen3       sync_header_i, block_start_i: delayed one clock onto
//              sync_header_o and block_start_o; nothing else reads them.
//   Unused     lane_number, curr_data_rate_i.
//
// Clock and reset
//   clk_i only. rst_i is synchronous and active high.
//
// Limitations
//   8b/10b-rate scrambling only: gen1_scramble is used whatever
//   curr_data_rate_i says, and gen3_scramble is not instantiated.
//
// References
//   PCIe Base Spec r2.1, §4.2.3
// ---------------------------------------------------------------------------
module scrambler
  import pcie_phy_pkg::*;
(

    input  logic               clk_i,             //! PIPE TX or RX user clock
    input  logic               rst_i,             //! Synchronous, active high
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
);


  logic [3:0] gen1_data_k;
  logic [31:0] gen1_data;
  logic gen1_valid;

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

  always_ff @(posedge clk_i) begin
    if (rst_i) begin
      sync_header_o <= '0;
      data_valid_o  <= '0;
      block_start_o <= '0;
    end else begin
      sync_header_o <= sync_header_i;
      // Do not drive data_valid_o from gen1_valid. gen1_scramble's pipeline
      // holds through a data_valid_i gap, so gen1_valid stays high and its
      // last word would be presented again on every idle clock.
      data_valid_o  <= data_valid_i;
      block_start_o <= block_start_i;
    end
  end

  always_comb begin
    data_k_out_o = gen1_data_k;
    data_out_o   = gen1_data;

  end





endmodule
