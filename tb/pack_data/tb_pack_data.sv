`timescale 1ns/1ps
// =============================================================================
//  tb_pack_data -- §63 #7g-1.  `pack_data`'s FIRST bench of its own.
//
//  The module has been in the RX path since before the gate existed and NO gate
//  target has ever exercised it as a unit: it is reached only through
//  `phy_receive`, where five other modules sit between the stimulus and its
//  ports.  §63 #7d and #7e both reasoned ABOUT it from source -- the claim in
//  test_pcie_fullstack.py that it "preserves SDP and END exactly" and "has no
//  tkeep/tlast port" is a READING, not a measurement -- and §22.90's second limb
//  says a property inferred from a module's source shape is not a property of
//  the artifact.  This bench turns those readings into rows.
// =============================================================================
module tb_pack_data #(
    parameter int DATA_WIDTH    = 32,
    parameter int MAX_NUM_LANES = 16
);
  import pcie_phy_pkg::*;

  logic clk_i = 0;
  logic rst_i;
  logic phy_link_up_i;
  logic lane_reverse_i;
  rate_speed_e curr_data_rate_i;
  logic [(MAX_NUM_LANES*DATA_WIDTH)-1:0] data_i;
  logic [MAX_NUM_LANES-1:0]              data_valid_i;
  logic [(4*MAX_NUM_LANES)-1:0]          data_k_i;
  logic [(2*MAX_NUM_LANES)-1:0]          sync_header_i;
  logic [(MAX_NUM_LANES*DATA_WIDTH)-1:0] data_o;
  logic [MAX_NUM_LANES-1:0]              data_valid_o;
  logic [(4*MAX_NUM_LANES)-1:0]          data_k_o;
  logic [(2*MAX_NUM_LANES)-1:0]          sync_header_o;
  logic [5:0]                            pipe_width_i;
  logic                                  fifo_wr_o;
  logic [5:0]                            num_active_lanes_i;

  pack_data #(
      .DATA_WIDTH(DATA_WIDTH),
      .MAX_NUM_LANES(MAX_NUM_LANES)
  ) dut (
      .clk_i(clk_i), .rst_i(rst_i), .phy_link_up_i(phy_link_up_i),
      .lane_reverse_i(lane_reverse_i), .curr_data_rate_i(curr_data_rate_i),
      .data_i(data_i), .data_valid_i(data_valid_i), .data_k_i(data_k_i),
      .sync_header_i(sync_header_i),
      .data_o(data_o), .data_valid_o(data_valid_o), .data_k_o(data_k_o),
      .sync_header_o(sync_header_o),
      .pipe_width_i(pipe_width_i), .fifo_wr_o(fifo_wr_o),
      .num_active_lanes_i(num_active_lanes_i)
  );
endmodule
