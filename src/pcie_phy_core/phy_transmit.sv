
module phy_transmit
  import pcie_phy_pkg::*;
#(
    parameter int CLK_RATE      = 100,             //!Clock speed in MHz, Defualt is 100
    parameter int MAX_NUM_LANES = 16,              //! Maximum number of lanes module can support
    // TLP data width
    parameter int DATA_WIDTH    = 32,              //! AXIS data width
    // TLP strobe width
    parameter int STRB_WIDTH    = DATA_WIDTH / 8,
    parameter int KEEP_WIDTH    = STRB_WIDTH,
    parameter int USER_WIDTH    = 5,
    // §63 #5 (GTH 8-1): the per-lane PIPE data width AT THE PORT -- 16 or 32.
    // NOT DATA_WIDTH, which is the DLL-facing Dword bus.  Inside this module the
    // symbol container stays 32 bits + 4 K flags per lane (lane_management and
    // the scrambler are built on it, and at Gen1 only pipe_width>>3 = 2 bytes of
    // it are live); the port takes its low PIPE_DATA_WIDTH bits at ONE site, the
    // TX conversion point below.  Default 32 keeps every standalone bench's port
    // exactly as it was; pcie_phy_top and pcie_endpoint_top pass 16.
    parameter int PIPE_DATA_WIDTH = 32
) (
    input logic clk_i,  //! 100MHz clock signal
    input logic pipe_rx_usr_clk_i,
    input logic pipe_tx_usr_clk_i,
    input logic rst_i,  //! Reset signal


    input  logic                                                 en_i,
    input  logic                                                 link_up_i,
    output logic              [(MAX_NUM_LANES*PIPE_DATA_WIDTH)-1:0] pipe_data_o,
    output logic              [               MAX_NUM_LANES-1:0] pipe_data_valid_o,
    output logic              [(MAX_NUM_LANES*PIPE_DATA_WIDTH/8)-1:0] pipe_data_k_o,
    output logic              [           (2*MAX_NUM_LANES)-1:0] pipe_sync_header_o,
    output logic              [               MAX_NUM_LANES-1:0] pipe_txstart_block_o,
    output logic              [                             5:0] pipe_width_o,
    input  logic              [                             5:0] num_active_lanes_i,
    input  logic                                                 send_ordered_set_i,
    // Per-lane ordered sets from the LTSSM (LTSSM-authoritative lane numbers,
    // Decision 1). Each lane carries its own fully-formed OS with lane_num
    // already inside; os_generator no longer invents it. At MAX_NUM_LANES=1 this
    // is one pcie_ordered_set_t, identical to the previous single-struct port.
    input  pcie_ordered_set_t [MAX_NUM_LANES-1:0]                ordered_set_i,
    input  rate_speed_e                                          curr_data_rate_i,
    output logic                                                 ordered_set_tranmitted_o,
    input  gen_os_struct_t                                       gen_os_ctrl_i,
    input  logic              [                  DATA_WIDTH-1:0] s_dllp_axis_tdata,
    input  logic              [                  KEEP_WIDTH-1:0] s_dllp_axis_tkeep,
    input  logic                                                 s_dllp_axis_tvalid,
    input  logic                                                 s_dllp_axis_tlast,
    input  logic              [                  USER_WIDTH-1:0] s_dllp_axis_tuser,
    output logic                                                 s_dllp_axis_tready
);
  parameter int DEPTH = 20;
  parameter int ID_ENABLE = 0;
  parameter int ID_WIDTH = 8;
  parameter int DEST_ENABLE = 0;
  parameter int DEST_WIDTH = 8;
  parameter int USER_ENABLE = 1;
  parameter int LAST_ENABLE = 1;
  parameter int KEEP_ENABLE = (DATA_WIDTH > 8);




  //   logic [DATA_WIDTH-1:0] dllp_axis_tdata;
  //   logic [KEEP_WIDTH-1:0] dllp_axis_tkeep;
  //   logic dllp_axis_tvalid;
  //   logic dllp_axis_tlast;
  //   logic [USER_WIDTH-1:0] dllp_axis_tuser;
  //   logic dllp_axis_tready;

  logic              [                  DATA_WIDTH-1:0] framed_axis_tdata;
  logic              [                  KEEP_WIDTH-1:0] framed_axis_tkeep;
  logic                                                 framed_axis_tvalid;
  logic                                                 framed_axis_tlast;
  logic              [                  USER_WIDTH-1:0] framed_axis_tuser;
  logic                                                 framed_axis_tready;



  logic              [                  DATA_WIDTH-1:0] fifo_framed_axis_tdata;
  logic              [                  KEEP_WIDTH-1:0] fifo_framed_axis_tkeep;
  logic                                                 fifo_framed_axis_tvalid;
  logic                                                 fifo_framed_axis_tlast;
  logic              [                  USER_WIDTH-1:0] fifo_framed_axis_tuser;
  logic                                                 fifo_framed_axis_tready;

  logic              [  (DATA_WIDTH*MAX_NUM_LANES)-1:0] phy_axis_tdata;
  logic              [  (KEEP_WIDTH*MAX_NUM_LANES)-1:0] phy_axis_tkeep;
  logic                                                 phy_axis_tvalid;
  logic                                                 phy_axis_tlast;
  logic              [  (USER_WIDTH*MAX_NUM_LANES)-1:0] phy_axis_tuser;
  logic                                                 phy_axis_tready;


  logic              [  (DATA_WIDTH*MAX_NUM_LANES)-1:0] fifo_phy_axis_tdata;
  logic              [  (KEEP_WIDTH*MAX_NUM_LANES)-1:0] fifo_phy_axis_tkeep;
  logic                                                 fifo_phy_axis_tvalid;
  logic                                                 fifo_phy_axis_tlast;
  logic              [  (USER_WIDTH*MAX_NUM_LANES)-1:0] fifo_phy_axis_tuser;
  logic                                                 fifo_phy_axis_tready;

  logic              [( MAX_NUM_LANES* DATA_WIDTH)-1:0] lm_data_out;
  logic              [               MAX_NUM_LANES-1:0] lm_data_valid;
  logic              [           (4*MAX_NUM_LANES)-1:0] lm_d_k_out;
  logic              [               MAX_NUM_LANES-1:0] lm_start_block;
  logic              [           (2*MAX_NUM_LANES)-1:0] lm_sync_header;
  logic              [                             5:0] lm_pipe_width;


  logic                                                 tx_fifo_empty;
  logic                                                 tx_fifo_full;
  logic                                                 tx_wr_en;
  logic                                                 tx_rd_en;

  logic                                                 rx_fifo_empty;
  logic                                                 rx_fifo_full;
  logic                                                 rx_wr_en;
  logic                                                 rx_rd_en;


  logic                                                 send_ordered_set;
  pcie_ordered_set_t                                    ordered_set;
  rate_speed_e                                          curr_data_rate;
  logic                                                 ordered_set_tranmitted;
  gen_os_struct_t                                       gen_os_ctrl;


  //   assign m_axis_tready   = phy_axis_tready;
  assign pipe_width_o = lm_pipe_width;


  localparam int LtssmDataInSize = 1 +
   + $size(ordered_set) + $size(gen_os_ctrl) + $size(pcie_ordered_set_t);


  frame_symbols #(
      .USER_WIDTH(USER_WIDTH),
      .DATA_WIDTH(DATA_WIDTH),
      .KEEP_WIDTH(KEEP_WIDTH)
  ) frame_symbols_inst (
      .clk_i           (clk_i),
      .rst_i           (rst_i),
      .curr_data_rate_i(curr_data_rate_i),
      .s_axis_tdata    (s_dllp_axis_tdata),
      .s_axis_tkeep    (s_dllp_axis_tkeep),
      .s_axis_tvalid   (s_dllp_axis_tvalid),
      .s_axis_tlast    (s_dllp_axis_tlast),
      .s_axis_tuser    (s_dllp_axis_tuser),
      .s_axis_tready   (s_dllp_axis_tready),
      .m_axis_tdata    (framed_axis_tdata),
      .m_axis_tkeep    (framed_axis_tkeep),
      .m_axis_tvalid   (framed_axis_tvalid),
      .m_axis_tlast    (framed_axis_tlast),
      .m_axis_tuser    (framed_axis_tuser),
      .m_axis_tready   (framed_axis_tready)
  );


  // The scrambler's per-lane 32/4 output container.  Module-level, not inside
  // the generate, so a bench can read the half the port drops (§63 #5 8-1 W4).
  logic [(MAX_NUM_LANES*32)-1:0] scr_data_out;
  logic [ (MAX_NUM_LANES*4)-1:0] scr_data_k_out;

  for (genvar lane = 0; lane < MAX_NUM_LANES; lane++) begin : gen_lane_scramble
    scrambler scrambler_inst (
        .clk_i           (pipe_tx_usr_clk_i),
        .rst_i           (rst_i),
        .lane_number     (lane),
        .curr_data_rate_i(curr_data_rate_i),
        .pipe_width_i    (lm_pipe_width),
        .data_in_i       (lm_data_out[lane*32+:32]),
        .data_k_in_i     (lm_d_k_out[lane*4+:4]),
        .data_valid_i    (lm_data_valid[lane]),
        .sync_header_i   (lm_sync_header[lane*2+:2]),
        .block_start_i   (lm_start_block[lane]),
        .data_valid_o    (pipe_data_valid_o[lane]),
        .data_out_o      (scr_data_out[lane*32+:32]),
        .data_k_out_o    (scr_data_k_out[lane*4+:4]),
        .block_start_o   (pipe_txstart_block_o[lane]),
        .sync_header_o   (pipe_sync_header_o[lane*2+:2])
    );
    // THE TX CONVERSION POINT (§63 #5 8-1, shape S).  The port carries the low
    // PIPE_DATA_WIDTH bits and PIPE_DATA_WIDTH/8 K flags of the lane's container.
    // At 16 the upper half is DROPPED here, and that is the interface as PG239
    // specifies it: Table 5 p.12, phy_txdata "Bits[31:16] are used for Gen3 only
    // and must be ignored in Gen1 and Gen2"; phy_txdatak[1:0] "for Gen1 and Gen2
    // only".  Nothing is lost: lane_management fills only bytes < pipe_width>>3
    // (2 at Gen1) and gen1_scramble passes bytes >= pipe_width>>3 through
    // unscrambled, so the dropped half is the zero lane_management wrote.  That
    // is a claim, so it is a gate row, not this comment: W4 asserts it on every
    // post-L0 cycle.  At 32 this is the whole container, i.e. the identity.
    assign pipe_data_o[lane*PIPE_DATA_WIDTH+:PIPE_DATA_WIDTH] =
        scr_data_out[lane*32+:PIPE_DATA_WIDTH];
    assign pipe_data_k_o[lane*(PIPE_DATA_WIDTH/8)+:(PIPE_DATA_WIDTH/8)] =
        scr_data_k_out[lane*4+:(PIPE_DATA_WIDTH/8)];
  end




  lane_management #(
      .DATA_WIDTH(DATA_WIDTH),
      .STRB_WIDTH(STRB_WIDTH),
      .KEEP_WIDTH(KEEP_WIDTH),
      .USER_WIDTH(USER_WIDTH),
      .MAX_NUM_LANES(MAX_NUM_LANES)
  ) lane_management_inst (
      .clk_i             (pipe_tx_usr_clk_i),
      .rst_i             (rst_i),
      .phy_link_up_i     (),
      .s_dllp_axis_tdata (fifo_framed_axis_tdata),
      .s_dllp_axis_tkeep (fifo_framed_axis_tkeep),
      .s_dllp_axis_tvalid(fifo_framed_axis_tvalid),
      .s_dllp_axis_tlast (fifo_framed_axis_tlast),
      .s_dllp_axis_tuser (fifo_framed_axis_tuser),
      .s_dllp_axis_tready(fifo_framed_axis_tready),
      .s_phy_axis_tdata  (fifo_phy_axis_tdata),
      .s_phy_axis_tkeep  (fifo_phy_axis_tkeep),
      .s_phy_axis_tvalid (fifo_phy_axis_tvalid),
      .s_phy_axis_tlast  (fifo_phy_axis_tlast),
      .s_phy_axis_tuser  (fifo_phy_axis_tuser),
      .s_phy_axis_tready (fifo_phy_axis_tready),
      .curr_data_rate_i  (curr_data_rate_i),
      .lane_reverse_i    ('0),
      .data_out_o        (lm_data_out),
      .data_valid_o      (lm_data_valid),
      .d_k_out_o         (lm_d_k_out),
      .sync_header_o     (lm_sync_header),
      .start_block_o     (lm_start_block),
      .pipe_width_o      (lm_pipe_width),
      .num_active_lanes_i(num_active_lanes_i)
  );


//   synchronous_fifo # (
//     .DEPTH(3),
//     .DATA_WIDTH(LtssmDataInSize)
//   )
//   ltssm_to_os_gen_async_fifo_inst (
//     .reset(rst_i),
//     .clk_in(pipe_rx_usr_clk_i),
//     .we('1),
//     .din({curr_data_rate_i, send_ordered_set_i, gen_os_ctrl_i, ordered_set_i}),
//     .busy(tx_fifo_full),
//     .clk_out(pipe_tx_usr_clk_i),
//     .re('1),
//     .dout({curr_data_rate, send_ordered_set, gen_os_ctrl, ordered_set}),
//     .ready(tx_fifo_empty)
//   );
//   async_fifo #(
//       .DSIZE(LtssmDataInSize),
//       .ASIZE(2)
//   ) ltssm_to_os_gen_async_fifo_inst (
//       .wclk(pipe_rx_usr_clk_i),
//       .wrst_n(!rst_i),
//       .winc(gen_os_ctrl_i.valid || send_ordered_set_i),
//       .wdata({curr_data_rate_i, send_ordered_set_i, gen_os_ctrl_i, ordered_set_i}),
//       .wfull(tx_fifo_full),
//       .awfull(),
//       .rclk(pipe_tx_usr_clk_i),
//       .rrst_n(!rst_i),
//       .rinc(!tx_fifo_empty),
//       .rdata({curr_data_rate, send_ordered_set, gen_os_ctrl, ordered_set}),
//       .rempty(tx_fifo_empty),
//       .arempty()
//   );


//     synchronous_fifo # (
//     .DEPTH(3),
//     .DATA_WIDTH(1)
//   )
//   os_gen_to_ltssm_async_fifo_inst (
//     .reset(rst_i),
//     .clk_in(pipe_tx_usr_clk_i),
//     .we('1),
//     .din({ordered_set_tranmitted}),
//     .busy(rx_fifo_full),
//     .clk_out(pipe_rx_usr_clk_i),
//     .re('1),
//     .dout({ordered_set_tranmitted_o}),
//     .ready(rx_fifo_empty)
//   );

//   async_fifo #(
//       .DSIZE(1),
//       .ASIZE(2)
//   ) os_gen_to_ltssm_async_fifo_inst (
//       .wclk(pipe_tx_usr_clk_i),
//       .wrst_n(!rst_i),
//       .winc(ordered_set_tranmitted),
//       .wdata({ordered_set_tranmitted}),
//       .wfull(rx_fifo_full),
//       .awfull(),
//       .rclk(pipe_rx_usr_clk_i),
//       .rrst_n(!rst_i),
//       .rinc(!rx_fifo_empty),
//       .rdata({ordered_set_tranmitted_o}),
//       .rempty(rx_fifo_empty),
//       .arempty()
//   );


  os_generator #(
      .CLK_RATE(CLK_RATE),
      .DATA_WIDTH(DATA_WIDTH),
      .KEEP_WIDTH(KEEP_WIDTH),
      .USER_WIDTH(USER_WIDTH),
      .MAX_NUM_LANES(MAX_NUM_LANES)
  ) os_generator_inst (
      .clk_i           (pipe_rx_usr_clk_i),
      .rst_i           (rst_i),
      .curr_data_rate_i(curr_data_rate_i),
      .link_up_i      (link_up_i),
      .send_ltssm_os_i (send_ordered_set_i),
      .preset_i        ('0),
      .gen_os_ctrl_i   (gen_os_ctrl_i),
      .os_sent_o       (ordered_set_tranmitted_o),
      .ordered_set_i   (ordered_set_i),
      .m_axis_tdata    (phy_axis_tdata),
      .m_axis_tkeep    (phy_axis_tkeep),
      .m_axis_tvalid   (phy_axis_tvalid),
      .m_axis_tlast    (phy_axis_tlast),
      .m_axis_tuser    (phy_axis_tuser),
      .m_axis_tready   (phy_axis_tready)
  );

    // Ordered-set FIFO carries the full per-lane bus (phy_axis_tdata =
    // DATA_WIDTH*MAX_NUM_LANES). Widen DATA/KEEP/USER to *MAX_NUM_LANES so
    // lanes 1..N-1 are not truncated to lane 0 (Phase 4b, Hop 9). DEPTH is in
    // bytes here (word-depth = DEPTH/KEEP_WIDTH inside axis_async_fifo), so it
    // scales by MAX_NUM_LANES too -- keeping word-depth invariant across widths
    // and avoiding $clog2(DEPTH/KEEP_WIDTH)=0. At x1 (*1) every value is
    // identical to the previous 20/32/KEEP/USER -- a provable no-op.
    axis_async_fifo #(
        .DEPTH      (DEPTH * MAX_NUM_LANES),
        .DATA_WIDTH (DATA_WIDTH * MAX_NUM_LANES),
        .KEEP_ENABLE(KEEP_ENABLE),
        .KEEP_WIDTH (KEEP_WIDTH * MAX_NUM_LANES),
        .LAST_ENABLE(LAST_ENABLE),
        .ID_ENABLE  (ID_ENABLE),
        .ID_WIDTH   (ID_WIDTH),
        .DEST_ENABLE(DEST_ENABLE),
        .DEST_WIDTH (DEST_WIDTH),
        .USER_ENABLE(USER_ENABLE),
        .USER_WIDTH (USER_WIDTH * MAX_NUM_LANES)
    ) ordered_set_axis_async_fifo_inst (
        .s_clk        (pipe_rx_usr_clk_i),
        .s_rst        (rst_i),
        .s_axis_tdata (phy_axis_tdata),
        .s_axis_tkeep (phy_axis_tkeep),
        .s_axis_tvalid(phy_axis_tvalid),
        .s_axis_tready(phy_axis_tready),
        .s_axis_tlast (phy_axis_tlast),
        .s_axis_tid   (),
        .s_axis_tdest (),
        .s_axis_tuser (phy_axis_tuser),



        .m_clk        (pipe_tx_usr_clk_i),
        .m_rst        (rst_i),
        .m_axis_tdata (fifo_phy_axis_tdata),
        .m_axis_tkeep (fifo_phy_axis_tkeep),
        .m_axis_tvalid(fifo_phy_axis_tvalid),
        .m_axis_tready(fifo_phy_axis_tready),
        .m_axis_tlast (fifo_phy_axis_tlast),
        .m_axis_tid   (),
        .m_axis_tdest (),
        .m_axis_tuser (fifo_phy_axis_tuser),

        .s_pause_req          ('0),
        .s_pause_ack          (),
        .m_pause_req          ('0),
        .m_pause_ack          (),
        .s_status_depth       (),
        .s_status_depth_commit(),
        .s_status_overflow    (),
        .s_status_bad_frame   (),
        .s_status_good_frame  (),
        .m_status_depth       (),
        .m_status_depth_commit(),
        .m_status_overflow    (),
        .m_status_bad_frame   (),
        .m_status_good_frame  ()
    );


  axis_async_fifo #(
      .DEPTH      (DEPTH),
      .DATA_WIDTH (DATA_WIDTH),
      .KEEP_ENABLE(KEEP_ENABLE),
      .KEEP_WIDTH (KEEP_WIDTH),
      .LAST_ENABLE(LAST_ENABLE),
      .ID_ENABLE  (ID_ENABLE),
      .ID_WIDTH   (ID_WIDTH),
      .DEST_ENABLE(DEST_ENABLE),
      .DEST_WIDTH (DEST_WIDTH),
      .USER_ENABLE(USER_ENABLE),
      .USER_WIDTH (USER_WIDTH)
  ) dllp_axis_async_fifo_inst (
      .s_clk        (clk_i),
      .s_rst        (rst_i),
      .s_axis_tdata (framed_axis_tdata),
      .s_axis_tkeep (framed_axis_tkeep),
      .s_axis_tvalid(framed_axis_tvalid),
      .s_axis_tready(framed_axis_tready),
      .s_axis_tlast (framed_axis_tlast),
      .s_axis_tid   (),
      .s_axis_tdest (),
      .s_axis_tuser (framed_axis_tuser),



      .m_clk        (pipe_tx_usr_clk_i),
      .m_rst        (rst_i),
      .m_axis_tdata (fifo_framed_axis_tdata),
      .m_axis_tkeep (fifo_framed_axis_tkeep),
      .m_axis_tvalid(fifo_framed_axis_tvalid),
      .m_axis_tready(fifo_framed_axis_tready),
      .m_axis_tlast (fifo_framed_axis_tlast),
      .m_axis_tid   (),
      .m_axis_tdest (),
      .m_axis_tuser (fifo_framed_axis_tuser),

      .s_pause_req          ('0),
      .s_pause_ack          (),
      .m_pause_req          ('0),
      .m_pause_ack          (),
      .s_status_depth       (),
      .s_status_depth_commit(),
      .s_status_overflow    (),
      .s_status_bad_frame   (),
      .s_status_good_frame  (),
      .m_status_depth       (),
      .m_status_depth_commit(),
      .m_status_overflow    (),
      .m_status_bad_frame   (),
      .m_status_good_frame  ()
  );

  //always #5  clk = ! clk ;

endmodule
