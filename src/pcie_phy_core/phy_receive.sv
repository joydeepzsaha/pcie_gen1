// ---------------------------------------------------------------------------
// phy_receive -- logical Physical Layer receive path: PIPE Symbols in,
//                Ordered Set flags and a TLP / DLLP stream out
//
// Purpose
//   Per lane, a descrambler (scrambler) and an ordered_set_handler, which
//   reports TS1, TS2, idle data and polarity inversion to the LTSSM. Across
//   the lanes, the descrambled stream passes through block_alignment (a
//   four-clock delay), pack_data (PIPE beats gathered into 32-bit words) and
//   data_handler (framing Symbols found and stripped), and leaves through an
//   axis_async_fifo into the clk_i domain for the Data Link Layer.
//
// Interfaces
//   PIPE input    pipe_data_i, pipe_data_k_i, pipe_data_valid_i,
//                 pipe_sync_header_i, pipe_block_start_i: per lane,
//                 PIPE_DATA_WIDTH data bits and PIPE_DATA_WIDTH / 8 K flags.
//   Control       link_up_i: from the LTSSM; gates the packet path. en_i:
//                 unused. curr_data_rate_i, pipe_width_i, num_active_lanes_i:
//                 the data rate, the PIPE width in bits, the active lane count.
//   To the LTSSM  ordered_set_o, ts1_valid_o, ts2_valid_o, idle_valid_o,
//                 polarity_inverted_o: per lane, on pipe_rx_usr_clk_i.
//   Packets       m_dllp_axis_*: TLPs and DLLPs on clk_i; tuser bit 0 marks a
//                 DLLP, bit 1 a TLP, bit 2 a frame ended by EDB.
//
// Clock and reset
//   pipe_rx_usr_clk_i runs the receive path. clk_i runs the read side of the
//   output FIFO and read_ready_reg, which nothing reads. rst_i (active high)
//   goes to every submodule and to both sides of the FIFO.
//
// Limitations
//   No lane-to-lane de-skew, and the packet path reads lane 0 only. There is
//   no elastic buffer: SKP insertion and removal are left to the PIPE PHY
//   (PG239, Table 10, rxstatus 001b and 010b).
//
// References
//   PCIe Base Spec r2.1, §4.2.4.10
//   PCIe Base Spec r2.1, §4.2.7
//   PG239, Table 7: RX Data Signals for UltraScale+ Devices
//   PG239, Table 10: Status Signals
// ---------------------------------------------------------------------------
module phy_receive
  import pcie_phy_pkg::*;
#(
    parameter int CLK_RATE      = 100,             //! Clock rate in MHz; passed on, not used
    parameter int MAX_NUM_LANES = 16,              //! Maximum number of lanes module can support
    // Also the per-lane width after the descrambler, whose data port is 32
    // bits.
    parameter int DATA_WIDTH    = 32,              //! AXIS data width
    parameter int STRB_WIDTH    = DATA_WIDTH / 8,
    parameter int KEEP_WIDTH    = STRB_WIDTH,
    parameter int USER_WIDTH    = 5,
    // Per-lane PIPE data width at the ports, with PIPE_DATA_WIDTH / 8 K flags;
    // pcie_phy_top and pcie_endpoint_top pass 16. It is not DATA_WIDTH: from
    // the descrambler on, each lane is a 32-bit, 4-K-flag container, and the
    // ports are zero-extended into it in one place, gen_lane_descramble. At
    // the default, 32, that is an identity.
    parameter int PIPE_DATA_WIDTH = 32
) (
    input logic clk_i,  // read side of the output FIFO
    input logic rst_i,

    // ---- control and PIPE input --------------------------------------------
    input  logic                                                 en_i,
    input  logic                                                 link_up_i,
    input  logic                                                 pipe_rx_usr_clk_i,
    input  logic              [(MAX_NUM_LANES*PIPE_DATA_WIDTH)-1:0] pipe_data_i,
    input  logic              [               MAX_NUM_LANES-1:0] pipe_data_valid_i,
    input  logic              [(MAX_NUM_LANES*PIPE_DATA_WIDTH/8)-1:0] pipe_data_k_i,
    input  logic              [           (2*MAX_NUM_LANES)-1:0] pipe_sync_header_i,
    input  logic              [             (MAX_NUM_LANES)-1:0] pipe_block_start_i,
    input  logic              [                             5:0] pipe_width_i,
    input  logic              [                             5:0] num_active_lanes_i,
    // ---- to and from the LTSSM, per lane -----------------------------------
    output pcie_ordered_set_t [               MAX_NUM_LANES-1:0] ordered_set_o,
    output logic              [               MAX_NUM_LANES-1:0] ts1_valid_o,
    output logic              [               MAX_NUM_LANES-1:0] ts2_valid_o,
    output logic              [               MAX_NUM_LANES-1:0] idle_valid_o,
    output logic              [               MAX_NUM_LANES-1:0] polarity_inverted_o,
    input  rate_speed_e                                          curr_data_rate_i,
    // ---- packet output: TLPs and DLLPs, on clk_i ---------------------------
    output logic              [                  DATA_WIDTH-1:0] m_dllp_axis_tdata,
    output logic              [                  KEEP_WIDTH-1:0] m_dllp_axis_tkeep,
    output logic                                                 m_dllp_axis_tvalid,
    output logic                                                 m_dllp_axis_tlast,
    output logic              [                  USER_WIDTH-1:0] m_dllp_axis_tuser,
    input  logic                                                 m_dllp_axis_tready
);


  // Settings of the output axis_async_fifo.
  parameter int DEPTH = 20;
  parameter int ID_ENABLE = 0;
  parameter int ID_WIDTH = 8;
  parameter int DEST_ENABLE = 0;
  parameter int DEST_WIDTH = 8;
  parameter int USER_ENABLE = 1;
  parameter int LAST_ENABLE = 1;
  parameter int KEEP_ENABLE = (DATA_WIDTH > 8);

  logic [( MAX_NUM_LANES* DATA_WIDTH)-1:0] descrambler_data;
  logic [               MAX_NUM_LANES-1:0] descrambler_data_valid;
  logic [           (4*MAX_NUM_LANES)-1:0] descrambler_data_k;
  logic [           (2*MAX_NUM_LANES)-1:0] descrambler_sync_header;

  logic [( MAX_NUM_LANES* DATA_WIDTH)-1:0] block_alignment_data;
  logic [               MAX_NUM_LANES-1:0] block_alignment_data_valid;
  logic [           (4*MAX_NUM_LANES)-1:0] block_alignment_data_k;
  logic [           (2*MAX_NUM_LANES)-1:0] block_alignment_sync_header;

  logic [( MAX_NUM_LANES* DATA_WIDTH)-1:0] packer_data;
  logic [               MAX_NUM_LANES-1:0] packer_data_valid;
  logic [           (4*MAX_NUM_LANES)-1:0] packer_data_k;
  logic [           (2*MAX_NUM_LANES)-1:0] packer_sync_header;

  // The fifo_* signals and rd_en are never driven; wr_en is driven by
  // pack_data's fifo_wr_o, which is constant 0, and is never read.
  logic [( MAX_NUM_LANES* DATA_WIDTH)-1:0] fifo_data;
  logic [               MAX_NUM_LANES-1:0] fifo_data_valid;
  logic [           (4*MAX_NUM_LANES)-1:0] fifo_data_k;
  logic [           (2*MAX_NUM_LANES)-1:0] fifo_sync_header;


  logic                                    fifo_empty;
  logic                                    fifo_full;
  logic                                    wr_en;
  logic                                    rd_en;


  // Neither size is used.
  localparam int PcieDataSize = $size(
      descrambler_data
  ) + $size(
      descrambler_data_valid
  ) + $size(
      descrambler_data_k
  ) + $size(
      descrambler_sync_header
  );

  localparam int PcieLaneDataSize = 1 + DATA_WIDTH + 4 + 2 + 1;


  // data_handler's output, before the clock-crossing FIFO. Despite the names,
  // it carries DLLPs as well as TLPs.
  logic [DATA_WIDTH-1:0] tlp_axis_tdata;
  logic [KEEP_WIDTH-1:0] tlp_axis_tkeep;
  logic                  tlp_axis_tvalid;
  logic                  tlp_axis_tlast;
  logic [USER_WIDTH-1:0] tlp_axis_tuser;
  logic                  tlp_axis_tready;

  // The descrambler's per-lane 32-bit, 4-K-flag input. Declared outside the
  // generate loop so that test_pcie_fullstack can read it.
  logic [(MAX_NUM_LANES*32)-1:0] desc_data_in;
  logic [ (MAX_NUM_LANES*4)-1:0] desc_data_k_in;

  for (genvar lane = 0; lane < MAX_NUM_LANES; lane++) begin : gen_lane_descramble
    // Zero-extends the lane's PIPE_DATA_WIDTH data bits and K flags into the
    // 32 / 4 container. PIPE data bits 31:16 are used at Gen3 only and are
    // ignored at Gen1 and Gen2 (PG239, Table 7). A size cast, so the line is
    // valid at 32 (the identity) and at 16, with no zero-width select.
    assign desc_data_in[lane*32+:32] =
        32'(pipe_data_i[lane*PIPE_DATA_WIDTH+:PIPE_DATA_WIDTH]);
    assign desc_data_k_in[lane*4+:4] =
        4'(pipe_data_k_i[lane*(PIPE_DATA_WIDTH/8)+:(PIPE_DATA_WIDTH/8)]);

    // read_ready is never driven, and read_ready_reg is never read.
    logic read_ready;
    logic read_ready_reg;


    always_ff @(posedge clk_i) begin
      if (rst_i) begin
        read_ready_reg <= '0;
      end else begin
        read_ready_reg <= read_ready == '0;
      end
    end


    scrambler descrambler_inst (
        .clk_i           (pipe_rx_usr_clk_i),
        .rst_i           (rst_i),
        .lane_number     (lane),
        .curr_data_rate_i(curr_data_rate_i),
        .pipe_width_i    (pipe_width_i),
        .data_valid_i    (pipe_data_valid_i[lane]),
        .data_in_i       (desc_data_in[lane*32+:32]),
        .data_k_in_i     (desc_data_k_in[lane*4+:4]),
        .sync_header_i   (pipe_sync_header_i[2*lane+:2]),
        .block_start_i   (pipe_block_start_i[lane]),
        .data_valid_o    (descrambler_data_valid[lane]),
        .data_out_o      (descrambler_data[DATA_WIDTH*lane+:DATA_WIDTH]),
        .data_k_out_o    (descrambler_data_k[4*lane+:4]),
        .sync_header_o   (descrambler_sync_header[2*lane+:2]),
        .block_start_o   ()
    );


    ordered_set_handler #(
        .CLK_RATE  (CLK_RATE),
        .DATA_WIDTH(DATA_WIDTH),
        .KEEP_WIDTH(KEEP_WIDTH),
        .USER_WIDTH(USER_WIDTH)
    ) ordered_set_handler_inst (
        .clk_i           (pipe_rx_usr_clk_i),
        .rst_i           (rst_i),
        .curr_data_rate_i(curr_data_rate_i),
        .pipe_width_i    (pipe_width_i),
        .data_valid_i    (descrambler_data_valid[lane]),
        .data_in_i       (descrambler_data[DATA_WIDTH*lane+:DATA_WIDTH]),
        .data_k_in_i     (descrambler_data_k[4*lane+:4]),
        .sync_header_i   (descrambler_sync_header[2*lane+:2]),
        .ordered_set_o   (ordered_set_o[lane]),
        .idle_valid_o    (idle_valid_o[lane]),
        .ts1_valid_o     (ts1_valid_o[lane]),
        .ts2_valid_o     (ts2_valid_o[lane]),
        .eieos_valid_o   (),
        .polarity_inverted_o(polarity_inverted_o[lane])
    );
  end

  // The packet path: block_alignment, pack_data and data_handler, all on
  // pipe_rx_usr_clk_i, with lane reversal tied off.
  block_alignment #(
      .DATA_WIDTH(DATA_WIDTH),
      .MAX_NUM_LANES(MAX_NUM_LANES)
  ) block_alignment_inst (
      .clk_i             (pipe_rx_usr_clk_i),
      .rst_i             (rst_i),
      .phy_link_up_i     (link_up_i),
      .lane_reverse_i    ('0),
      .curr_data_rate_i  (curr_data_rate_i),
      .data_valid_i      (descrambler_data_valid),
      .data_i            (descrambler_data),
      .data_k_i          (descrambler_data_k),
      .sync_header_i     (descrambler_sync_header),
      .data_o            (block_alignment_data),
      .data_valid_o      (block_alignment_data_valid),
      .data_k_o          (block_alignment_data_k),
      .sync_header_o     (block_alignment_sync_header),
      .pipe_width_i      (pipe_width_i),
      .num_active_lanes_i(num_active_lanes_i)
  );

  pack_data #(
      .DATA_WIDTH(DATA_WIDTH),
      .MAX_NUM_LANES(MAX_NUM_LANES)
  ) pack_data_inst (
      .clk_i             (pipe_rx_usr_clk_i),
      .rst_i             (rst_i),
      .phy_link_up_i     (link_up_i),
      .lane_reverse_i    ('0),
      .curr_data_rate_i  (curr_data_rate_i),
      .data_i            (block_alignment_data),
      .data_valid_i      (block_alignment_data_valid),
      .data_k_i          (block_alignment_data_k),
      .sync_header_i     (block_alignment_sync_header),
      .data_o            (packer_data),
      .data_valid_o      (packer_data_valid),
      .data_k_o          (packer_data_k),
      .sync_header_o     (packer_sync_header),
      .pipe_width_i      (pipe_width_i),
      .fifo_wr_o         (wr_en),
      .num_active_lanes_i(num_active_lanes_i)
  );


  // No FIFO sits between pack_data and data_handler: phy_fifo_empty_i is tied
  // 0 and phy_fifo_rd_en_o is left open.
  data_handler #(
      .DATA_WIDTH(DATA_WIDTH),
      .STRB_WIDTH(STRB_WIDTH),
      .KEEP_WIDTH(KEEP_WIDTH),
      .USER_WIDTH(USER_WIDTH),
      .MAX_NUM_LANES(MAX_NUM_LANES)
  ) data_handler_inst (
      .clk_i             (pipe_rx_usr_clk_i),
      .rst_i             (rst_i),
      .phy_link_up_i     (link_up_i),
      .phy_fifo_empty_i  ('0),
      .phy_fifo_rd_en_o  (),
      .lane_reverse_i    ('0),
      .curr_data_rate_i  (curr_data_rate_i),
      .data_i            (packer_data),
      .data_valid_i      (packer_data_valid),
      .data_k_i          (packer_data_k),
      .sync_header_i     (packer_sync_header),
      .m_dllp_axis_tdata (tlp_axis_tdata),
      .m_dllp_axis_tkeep (tlp_axis_tkeep),
      .m_dllp_axis_tvalid(tlp_axis_tvalid),
      .m_dllp_axis_tlast (tlp_axis_tlast),
      .m_dllp_axis_tuser (tlp_axis_tuser),
      .m_dllp_axis_tready(tlp_axis_tready),
      .pipe_width_i      (pipe_width_i),
      .num_active_lanes_i(num_active_lanes_i)
  );


  // Carries the packet stream from pipe_rx_usr_clk_i to clk_i.
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
      .s_clk        (pipe_rx_usr_clk_i),
      .s_rst        (rst_i),
      .s_axis_tdata (tlp_axis_tdata),
      .s_axis_tkeep (tlp_axis_tkeep),
      .s_axis_tvalid(tlp_axis_tvalid),
      .s_axis_tready(tlp_axis_tready),
      .s_axis_tlast (tlp_axis_tlast),
      .s_axis_tid   (),
      .s_axis_tdest (),
      .s_axis_tuser (tlp_axis_tuser),



      .m_clk        (clk_i),
      .m_rst        (rst_i),
      .m_axis_tdata (m_dllp_axis_tdata),
      .m_axis_tkeep (m_dllp_axis_tkeep),
      .m_axis_tvalid(m_dllp_axis_tvalid),
      .m_axis_tready(m_dllp_axis_tready),
      .m_axis_tlast (m_dllp_axis_tlast),
      .m_axis_tid   (),
      .m_axis_tdest (),
      .m_axis_tuser (m_dllp_axis_tuser),

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
