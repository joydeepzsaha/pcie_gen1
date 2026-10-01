// ---------------------------------------------------------------------------
// phy_transmit -- physical-layer transmit path, DLL stream to PIPE TX
//
// Purpose
//   Builds the per-lane PIPE transmit stream. frame_symbols adds the framing
//   Symbols to the DLL's TLP and DLLP stream. os_generator sends the Ordered
//   Sets the LTSSM requests, Logical Idle included, with their K flags, and
//   inserts SKP Ordered Sets. lane_management interleaves the two streams
//   onto the lanes, two Symbols per lane per clock, and one scrambler per lane
//   scrambles the result. 8b/10b encoding happens outside this module.
//
// Interfaces
//   DLL        s_dllp_axis_*: TLPs and DLLPs; tuser[0] marks a DLLP.
//   LTSSM      gen_os_ctrl_i, ordered_set_i, send_ordered_set_i: the Ordered
//              Set request; link_up_i enables SKP scheduling.
//              ordered_set_tranmitted_o: one pulse per Ordered Set, as
//              os_generator hands on its last beat. curr_data_rate_i: read
//              by frame_symbols and lane_management.
//   Lanes      num_active_lanes_i: the lanes lane_management drives.
//   PIPE TX    pipe_data_o, pipe_data_k_o: PIPE_DATA_WIDTH bits per lane;
//              pipe_data_valid_o, pipe_sync_header_o, pipe_txstart_block_o:
//              from the scramblers; pipe_width_o: lane_management's width.
//   Unused     en_i. CLK_RATE goes to os_generator, which ignores it.
//
// Clock and reset
//   clk_i: frame_symbols and the DLLP FIFO's write side. pipe_rx_usr_clk_i:
//   os_generator and the Ordered Set FIFO's write side. pipe_tx_usr_clk_i:
//   both FIFOs' read sides, lane_management and the scramblers. rst_i,
//   active high, resets every block in all three domains.
//
// Limitations
//   lane_management leaves its sync_header_o and start_block_o outputs
//   undriven, so after reset pipe_sync_header_o and pipe_txstart_block_o
//   register an undriven value. PG239 uses both at Gen3 and above only.
//
// References
//   PCIe Base Spec r2.1, §4.2.2
//   PCIe Base Spec r2.1, §4.2.3
//   PCIe Base Spec r2.1, §4.2.7.1
//   PG239, Table 5: TX Data Signals for Ultrascale+ Devices Interface Ports
// ---------------------------------------------------------------------------
module phy_transmit
  import pcie_phy_pkg::*;
#(
    parameter int CLK_RATE      = 100,             //! Unused: os_generator ignores it
    parameter int MAX_NUM_LANES = 16,              //! Maximum number of lanes module can support
    parameter int DATA_WIDTH    = 32,              //! AXIS data width
    parameter int STRB_WIDTH    = DATA_WIDTH / 8,
    parameter int KEEP_WIDTH    = STRB_WIDTH,
    parameter int USER_WIDTH    = 5,
    // The per-lane PIPE data width at the port, 16 or 32; DATA_WIDTH is the
    // DLL-side bus. Inside, each lane keeps a 32-bit word with 4 K flags, the
    // width lane_management and the scramblers use, and the port takes its low
    // PIPE_DATA_WIDTH bits at the TX conversion point below. pcie_phy_top and
    // pcie_endpoint_top pass 16.
    parameter int PIPE_DATA_WIDTH = 32
) (
    input logic clk_i,  //! DLL-side clock
    input logic pipe_rx_usr_clk_i,
    input logic pipe_tx_usr_clk_i,
    input logic rst_i,  //! Active high, all three clock domains


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
    // One complete Ordered Set per lane from the LTSSM, Lane Number included;
    // os_generator sends each as given.
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
  // Settings for the two axis_async_fifo instances. DEPTH counts bytes: with
  // KEEP_ENABLE set, axis_async_fifo holds DEPTH/KEEP_WIDTH beats, rounded up
  // to a power of two, and its address width is $clog2(DEPTH/KEEP_WIDTH).
  parameter int DEPTH = 20;
  parameter int ID_ENABLE = 0;
  parameter int ID_WIDTH = 8;
  parameter int DEST_ENABLE = 0;
  parameter int DEST_WIDTH = 8;
  parameter int USER_ENABLE = 1;
  parameter int LAST_ENABLE = 1;
  parameter int KEEP_ENABLE = (DATA_WIDTH > 8);

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


  // Nothing drives or reads the signals declared from here to gen_os_ctrl.
  // LtssmDataInSize, computed from the sizes of two of them, is unused too.
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


  assign pipe_width_o = lm_pipe_width;


  localparam int LtssmDataInSize = 1 +
   + $size(ordered_set) + $size(gen_os_ctrl) + $size(pcie_ordered_set_t);


  // DLL stream to frame_symbols, on clk_i.
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


  // Each lane's scrambler output, 32 data bits and 4 K flags. Declared outside
  // the generate loop so that a bench can read the upper half the port drops;
  // test_pcie_fullstack reads it.
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
    // The TX conversion point: the port takes the low PIPE_DATA_WIDTH bits and
    // PIPE_DATA_WIDTH/8 K flags of the lane's word. At 16 the upper half is
    // dropped, which matches PG239: at Gen1 and Gen2 the PHY ignores
    // phy_txdata bits 31:16, and phy_txdatak has two bits (PG239, Table 5: TX
    // Data Signals for Ultrascale+ Devices Interface Ports). The dropped half
    // carries no data: lane_management fills only the bytes below
    // pipe_width_o/8 and zeroes the rest, and gen1_scramble passes those upper
    // bytes through unscrambled. At 32 the port takes the whole word.
    assign pipe_data_o[lane*PIPE_DATA_WIDTH+:PIPE_DATA_WIDTH] =
        scr_data_out[lane*32+:PIPE_DATA_WIDTH];
    assign pipe_data_k_o[lane*(PIPE_DATA_WIDTH/8)+:(PIPE_DATA_WIDTH/8)] =
        scr_data_k_out[lane*4+:(PIPE_DATA_WIDTH/8)];
  end

  // Framed packets and Ordered Sets onto the lanes, on pipe_tx_usr_clk_i.
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


  // Ordered Sets as the LTSSM requests them, and SKP Ordered Sets, on
  // pipe_rx_usr_clk_i.
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

    // Ordered Set FIFO, pipe_rx_usr_clk_i to pipe_tx_usr_clk_i. It carries
    // every lane, so the data, keep and user widths scale with MAX_NUM_LANES.
    // DEPTH scales too, which keeps DEPTH/KEEP_WIDTH, the depth in beats, the
    // same at every lane count; an unscaled DEPTH would give a zero address
    // width from three lanes up.
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


  // DLLP and TLP FIFO, clk_i to pipe_tx_usr_clk_i. LAST_ENABLE carries tlast,
  // which lane_management needs to find the end of a packet.
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

endmodule
