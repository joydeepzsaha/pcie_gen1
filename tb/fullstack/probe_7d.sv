// =============================================================================
// §63 #7d Phase 1 -- measurement probe. BENCH-ONLY.
//
// !! NOTHING IN src/ CHANGES FOR THIS FILE. It attaches by `bind`, so the two
// phy_receive instances under measurement are byte-identical to the ones the
// gate builds. A probe that required an src/ edit would be measuring a tree
// that does not exist.
//
// It answers Phase 1.1/1.2/1.3 in ONE elaboration, because tb_pcie_fullstack
// carries BOTH phy_receive instances -- the RC's via pcie_rc_top -> pcie_phy_top
// (:262) and the EP's via pcie_endpoint_top (:533). `bind phy_receive` hits both
// and %m names which is which.
//
// !! IT COUNTS IN SYSTEMVERILOG, NOT IN COCOTB, ON PURPOSE. These are internal
// signals; reaching them from cocotb would need --public-flat-rw, which changes
// the build. A `final` block prints a summary that the run log carries.
//
// ⚠️ THE SHAPE OF THIS PROBE ALREADY ENCODES A PHASE-1 FINDING: pack_data has
// NO tkeep and NO tlast port, in either direction. The AXIS boundary signals are
// BORN in data_handler (phy_receive.sv:263-289). So "tkeep/tlast at pack_data's
// input" -- Phase 1.3 as written -- does not exist to be measured. The
// answerable form of the same question is whether the SDP/END framing K-codes
// are present and correctly placed in the data_k stream at pack_data's input
// (block_alignment's output) and at its output (data_handler's input). That is
// what this probe counts.
// =============================================================================
`timescale 1ns / 1ps

// -----------------------------------------------------------------------------
// Phase 1.1 -- parameters, resolved BY ELABORATION. The bind statement evaluates
// each expression in the scope of the bound-into instance, so these are the
// numbers that instance actually got, not numbers re-derived by hand from the
// instantiation chain. Hand-chaining them is exactly what this rung forbids.
// -----------------------------------------------------------------------------
module pr7d_params #(
    parameter int P_CLK_RATE      = 0,
    parameter int P_MAX_NUM_LANES = 0,
    parameter int P_DATA_WIDTH    = 0,
    parameter int P_STRB_WIDTH    = 0,
    parameter int P_KEEP_WIDTH    = 0,
    parameter int P_USER_WIDTH    = 0
) ();
  initial begin
    $display("PR7D_PARAM %m CLK_RATE=%0d MAX_NUM_LANES=%0d DATA_WIDTH=%0d STRB_WIDTH=%0d KEEP_WIDTH=%0d USER_WIDTH=%0d",
             P_CLK_RATE, P_MAX_NUM_LANES, P_DATA_WIDTH, P_STRB_WIDTH, P_KEEP_WIDTH, P_USER_WIDTH);
  end
endmodule

// -----------------------------------------------------------------------------
// Phase 1.2 -- the reset terms, separately, as waveforms. Bound into each top so
// the two terms of `rst_i || phy_phystatus_rst` can be told apart; phy_receive
// itself only ever sees the OR.
// -----------------------------------------------------------------------------
module pr7d_reset (
    input logic clk,
    input logic rst_top,
    input logic phystatus_rst
);
  longint unsigned n_clk = 0, n_rst = 0, n_phy = 0, n_either = 0;
  longint unsigned first_phy = 0, last_phy = 0, n_phy_rise = 0;
  logic phy_q = 1'b0;

  always @(posedge clk) begin
    n_clk <= n_clk + 1;
    if (rst_top) n_rst <= n_rst + 1;
    if (phystatus_rst) begin
      n_phy <= n_phy + 1;
      if (first_phy == 0) first_phy <= $time;
      last_phy <= $time;
    end
    if (rst_top || phystatus_rst) n_either <= n_either + 1;
    if (phystatus_rst && !phy_q) n_phy_rise <= n_phy_rise + 1;
    phy_q <= phystatus_rst;
  end

  final begin
    $display("PR7D_RESET %m clks=%0d rst_i_high=%0d phystatus_rst_high=%0d either_high=%0d phystatus_rises=%0d first_phy_ns=%0d last_phy_ns=%0d",
             n_clk, n_rst, n_phy, n_either, n_phy_rise, first_phy, last_phy);
  end
endmodule

// -----------------------------------------------------------------------------
// Phase 1.3 -- framing K-codes either side of pack_data, and the AXIS boundary
// both before and after the async FIFO.
//
// Gen1 8b/10b control codes (Base 2.1 §4.2.4.12, Table 4-3), matched only where
// the matching data_k bit is set:
//     SDP = K28.2 = 8'h5C   DLLP start      END = K29.7 = 8'hFD   frame end
//     STP = K27.7 = 8'hFB   TLP start       EDB = K30.7 = 8'hFE   nullified end
// -----------------------------------------------------------------------------
module pr7d_rx #(
    parameter int P_DATA_WIDTH    = 32,
    parameter int P_KEEP_WIDTH    = 4,
    parameter int P_MAX_NUM_LANES = 1
) (
    input logic                                     clk,
    input logic                                     rst,
    input logic                                     link_up,
    // phy_receive's OWN INPUT PORT -- the symbols as they arrive off the bridge,
    // before the descrambler. If a framing K-code is already missing HERE, every
    // module inside phy_receive is exonerated and the defect is upstream of the
    // whole receive path.
    input logic [P_MAX_NUM_LANES*P_DATA_WIDTH-1:0]  rx_data,
    input logic [             P_MAX_NUM_LANES-1:0]  rx_valid,
    input logic [           4*P_MAX_NUM_LANES-1:0]  rx_k,
    // pack_data's INPUT  == block_alignment's output
    input logic [P_MAX_NUM_LANES*P_DATA_WIDTH-1:0]  ba_data,
    input logic [             P_MAX_NUM_LANES-1:0]  ba_valid,
    input logic [           4*P_MAX_NUM_LANES-1:0]  ba_k,
    // pack_data's OUTPUT == data_handler's input
    input logic [P_MAX_NUM_LANES*P_DATA_WIDTH-1:0]  pk_data,
    input logic [             P_MAX_NUM_LANES-1:0]  pk_valid,
    input logic [           4*P_MAX_NUM_LANES-1:0]  pk_k,
    // data_handler's OUTPUT, pre-FIFO
    input logic [                P_KEEP_WIDTH-1:0]  dh_tkeep,
    input logic                                     dh_tvalid,
    input logic                                     dh_tlast,
    input logic                                     dh_tready,
    // phy_receive's OUTPUT, post-FIFO -- what §0 measured at the DLL
    input logic [                P_KEEP_WIDTH-1:0]  m_tkeep,
    input logic                                     m_tvalid,
    input logic                                     m_tlast,
    input logic                                     m_tready
);
  localparam logic [7:0] SDP = 8'h5C;
  localparam logic [7:0] END = 8'hFD;
  localparam logic [7:0] STP = 8'hFB;
  localparam logic [7:0] EDB = 8'hFE;

  longint unsigned rx_beats = 0, rx_sdp = 0, rx_end = 0, rx_stp = 0, rx_edb = 0;
  longint unsigned ba_beats = 0, ba_sdp = 0, ba_end = 0, ba_stp = 0, ba_edb = 0;
  longint unsigned pk_beats = 0, pk_sdp = 0, pk_end = 0, pk_stp = 0, pk_edb = 0;
  longint unsigned dh_beats = 0, dh_last = 0;
  longint unsigned m_beats = 0, m_last = 0;
  // tkeep histogram on the LAST beat, and over all beats. Indexed by tkeep value;
  // 16 entries covers KEEP_WIDTH<=4, which both sides are.
  longint unsigned dh_keep_on_last[16], m_keep_on_last[16], m_keep_all[16];
  logic seen_link_up = 1'b0;

  initial begin
    for (int i = 0; i < 16; i++) begin
      dh_keep_on_last[i] = 0;
      m_keep_on_last[i]  = 0;
      m_keep_all[i]      = 0;
    end
  end

  // Count K-code occurrences on a lane-parallel bus, only on valid lanes.
  function automatic void tally_k(input logic [P_MAX_NUM_LANES*P_DATA_WIDTH-1:0] d,
                                  input logic [4*P_MAX_NUM_LANES-1:0] k,
                                  input logic [P_MAX_NUM_LANES-1:0] v,
                                  ref longint unsigned c_sdp, ref longint unsigned c_end,
                                  ref longint unsigned c_stp, ref longint unsigned c_edb);
    logic [7:0] byte_v;
    for (int lane = 0; lane < P_MAX_NUM_LANES; lane++) begin
      if (!v[lane]) continue;
      for (int b = 0; b < P_DATA_WIDTH / 8; b++) begin
        if (!k[4*lane+b]) continue;
        byte_v = d[P_DATA_WIDTH*lane+8*b+:8];
        case (byte_v)
          SDP: c_sdp = c_sdp + 1;
          END: c_end = c_end + 1;
          STP: c_stp = c_stp + 1;
          EDB: c_edb = c_edb + 1;
          default: ;
        endcase
      end
    end
  endfunction

  always @(posedge clk) begin
    if (link_up) seen_link_up <= 1'b1;
    // Bound by a signal, not a cycle count (§22.89): everything below is counted
    // only from the first cycle link_up has ever been high.
    if (seen_link_up && !rst) begin
      if (|rx_valid) begin
        rx_beats <= rx_beats + 1;
        tally_k(rx_data, rx_k, rx_valid, rx_sdp, rx_end, rx_stp, rx_edb);
      end
      if (|ba_valid) begin
        ba_beats <= ba_beats + 1;
        tally_k(ba_data, ba_k, ba_valid, ba_sdp, ba_end, ba_stp, ba_edb);
      end
      if (|pk_valid) begin
        pk_beats <= pk_beats + 1;
        tally_k(pk_data, pk_k, pk_valid, pk_sdp, pk_end, pk_stp, pk_edb);
      end
      if (dh_tvalid && dh_tready) begin
        dh_beats <= dh_beats + 1;
        if (dh_tlast) begin
          dh_last <= dh_last + 1;
          dh_keep_on_last[dh_tkeep] <= dh_keep_on_last[dh_tkeep] + 1;
        end
      end
      if (m_tvalid && m_tready) begin
        m_beats <= m_beats + 1;
        m_keep_all[m_tkeep] <= m_keep_all[m_tkeep] + 1;
        if (m_tlast) begin
          m_last <= m_last + 1;
          m_keep_on_last[m_tkeep] <= m_keep_on_last[m_tkeep] + 1;
        end
      end
    end
  end

  final begin
    $display("PR7D_RX %m PHYRX_PORT   beats=%0d SDP=%0d END=%0d STP=%0d EDB=%0d",
             rx_beats, rx_sdp, rx_end, rx_stp, rx_edb);
    $display("PR7D_RX %m PACKDATA_IN  beats=%0d SDP=%0d END=%0d STP=%0d EDB=%0d",
             ba_beats, ba_sdp, ba_end, ba_stp, ba_edb);
    $display("PR7D_RX %m PACKDATA_OUT beats=%0d SDP=%0d END=%0d STP=%0d EDB=%0d",
             pk_beats, pk_sdp, pk_end, pk_stp, pk_edb);
    $display("PR7D_RX %m DH_PREFIFO   beats=%0d tlast=%0d", dh_beats, dh_last);
    for (int i = 0; i < 16; i++)
      if (dh_keep_on_last[i] != 0)
        $display("PR7D_RX %m DH_PREFIFO   tkeep_on_tlast[0x%0h]=%0d", i, dh_keep_on_last[i]);
    $display("PR7D_RX %m M_POSTFIFO   beats=%0d tlast=%0d", m_beats, m_last);
    for (int i = 0; i < 16; i++)
      if (m_keep_all[i] != 0)
        $display("PR7D_RX %m M_POSTFIFO   tkeep_all[0x%0h]=%0d", i, m_keep_all[i]);
    for (int i = 0; i < 16; i++)
      if (m_keep_on_last[i] != 0)
        $display("PR7D_RX %m M_POSTFIFO   tkeep_on_tlast[0x%0h]=%0d", i, m_keep_on_last[i]);
  end
endmodule

// -----------------------------------------------------------------------------
// The TRANSMIT side, so "the RC never receives END" can be told apart from "the
// EP never sends END" BY MEASUREMENT rather than by inferring one from the other
// across a bridge whose cleanliness is an inherited claim.
// -----------------------------------------------------------------------------
module pr7d_tx #(
    parameter int P_DATA_WIDTH    = 32,
    parameter int P_KEEP_WIDTH    = 4,
    parameter int P_MAX_NUM_LANES = 1
) (
    input logic                                    clk,
    input logic                                    rst,
    input logic                                    link_up,
    input logic [P_MAX_NUM_LANES*P_DATA_WIDTH-1:0] tx_data,
    input logic [             P_MAX_NUM_LANES-1:0] tx_valid,
    input logic [           4*P_MAX_NUM_LANES-1:0] tx_k,
    // phy_transmit's AXIS INPUT, from the Data Link Layer. END is generated from
    // tlast; if tlast never arrives here, no END can ever be framed, and the
    // defect is the DLL's, not the PHY's.
    input logic [                P_KEEP_WIDTH-1:0] s_tkeep,
    input logic                                    s_tvalid,
    input logic                                    s_tlast,
    input logic                                    s_tready
);
  localparam logic [7:0] SDP = 8'h5C;
  localparam logic [7:0] END = 8'hFD;
  localparam logic [7:0] STP = 8'hFB;
  localparam logic [7:0] EDB = 8'hFE;

  longint unsigned tx_beats = 0, tx_sdp = 0, tx_end = 0, tx_stp = 0, tx_edb = 0;
  longint unsigned s_beats = 0, s_last = 0;
  longint unsigned s_keep_on_last[16];
  logic seen_link_up = 1'b0;

  initial for (int i = 0; i < 16; i++) s_keep_on_last[i] = 0;

  always @(posedge clk) begin
    if (link_up) seen_link_up <= 1'b1;
    if (seen_link_up && !rst && s_tvalid && s_tready) begin
      s_beats <= s_beats + 1;
      if (s_tlast) begin
        s_last <= s_last + 1;
        s_keep_on_last[s_tkeep] <= s_keep_on_last[s_tkeep] + 1;
      end
    end
    if (seen_link_up && !rst && |tx_valid) begin
      tx_beats <= tx_beats + 1;
      for (int lane = 0; lane < P_MAX_NUM_LANES; lane++) begin
        if (tx_valid[lane]) begin
          for (int b = 0; b < P_DATA_WIDTH / 8; b++) begin
            if (tx_k[4*lane+b]) begin
              case (tx_data[P_DATA_WIDTH*lane+8*b+:8])
                SDP: tx_sdp <= tx_sdp + 1;
                END: tx_end <= tx_end + 1;
                STP: tx_stp <= tx_stp + 1;
                EDB: tx_edb <= tx_edb + 1;
                default: ;
              endcase
            end
          end
        end
      end
    end
  end

  final begin
    $display("PR7D_TX %m DLL_AXIS_IN  beats=%0d tlast=%0d", s_beats, s_last);
    for (int i = 0; i < 16; i++)
      if (s_keep_on_last[i] != 0)
        $display("PR7D_TX %m DLL_AXIS_IN  tkeep_on_tlast[0x%0h]=%0d", i, s_keep_on_last[i]);
    $display("PR7D_TX %m PHYTX_PORT   beats=%0d SDP=%0d END=%0d STP=%0d EDB=%0d",
             tx_beats, tx_sdp, tx_end, tx_stp, tx_edb);
  end
endmodule

// -----------------------------------------------------------------------------
// frame_symbols' OWN OUTPUT -- splits "ENDP is never generated" from "ENDP is
// generated and then lost downstream in lane_management/scrambler". Both stories
// look identical at phy_transmit's port, which is where the first measurement
// stopped.
//
// frame_symbols has no data_k output; K-ness is carried onward out of band, so
// this counts raw byte values. A data byte that happens to equal 8'hFD would be
// a false positive, which is why the RC is measured with the identical counter:
// the two sides are compared, not thresholded.
// -----------------------------------------------------------------------------
module pr7d_frame #(
    parameter int P_DATA_WIDTH = 32,
    parameter int P_KEEP_WIDTH = 4,
    parameter int P_USER_WIDTH = 0
) (
    input logic                    clk,
    input logic                    rst,
    input logic [P_DATA_WIDTH-1:0] m_tdata,
    input logic [P_KEEP_WIDTH-1:0] m_tkeep,
    input logic                    m_tvalid,
    input logic                    m_tlast,
    input logic                    m_tready,
    // tuser is frame_symbols' K-POSITION MASK, written as a 4-bit literal but
    // declared [USER_WIDTH-1:0]. Widened to 8 here so the probe never truncates
    // what it is measuring for truncation.
    input logic [             7:0] m_tuser
);
  localparam logic [7:0] SDP = 8'h5C;
  localparam logic [7:0] END = 8'hFD;

  longint unsigned f_beats = 0, f_last = 0, f_sdp = 0, f_end = 0;
  longint unsigned f_keep_on_last[16];
  longint unsigned f_user_on_last[256], f_user_all[256];

  initial begin
    for (int i = 0; i < 16; i++) f_keep_on_last[i] = 0;
    for (int i = 0; i < 256; i++) begin
      f_user_on_last[i] = 0;
      f_user_all[i] = 0;
    end
  end

  initial $display("PR7D_FRAME %m USER_WIDTH_RESOLVED=%0d", P_USER_WIDTH);

  always @(posedge clk) begin
    if (!rst && m_tvalid && m_tready) begin
      f_beats <= f_beats + 1;
      for (int b = 0; b < P_DATA_WIDTH / 8; b++) begin
        if (m_tdata[8*b+:8] == SDP) f_sdp <= f_sdp + 1;
        if (m_tdata[8*b+:8] == END) f_end <= f_end + 1;
      end
      f_user_all[m_tuser] <= f_user_all[m_tuser] + 1;
      if (m_tlast) begin
        f_last <= f_last + 1;
        f_keep_on_last[m_tkeep] <= f_keep_on_last[m_tkeep] + 1;
        f_user_on_last[m_tuser] <= f_user_on_last[m_tuser] + 1;
      end
    end
  end

  final begin
    $display("PR7D_FRAME %m FS_OUT beats=%0d tlast=%0d SDPbytes=%0d ENDbytes=%0d",
             f_beats, f_last, f_sdp, f_end);
    for (int i = 0; i < 16; i++)
      if (f_keep_on_last[i] != 0)
        $display("PR7D_FRAME %m FS_OUT tkeep_on_tlast[0x%0h]=%0d", i, f_keep_on_last[i]);
    for (int i = 0; i < 256; i++)
      if (f_user_all[i] != 0)
        $display("PR7D_FRAME %m FS_OUT tuser_all[0x%0h]=%0d", i, f_user_all[i]);
    for (int i = 0; i < 256; i++)
      if (f_user_on_last[i] != 0)
        $display("PR7D_FRAME %m FS_OUT tuser_on_tlast[0x%0h]=%0d", i, f_user_on_last[i]);
  end
endmodule

// =============================================================================
// bind statements -- file scope.
// =============================================================================

bind frame_symbols pr7d_frame #(
    .P_DATA_WIDTH(DATA_WIDTH),
    .P_KEEP_WIDTH(KEEP_WIDTH),
    .P_USER_WIDTH(USER_WIDTH)
) u_pr7d_frame (
    .clk     (clk_i),
    .rst     (rst_i),
    .m_tdata (m_axis_tdata),
    .m_tkeep (m_axis_tkeep),
    .m_tvalid(m_axis_tvalid),
    .m_tlast (m_axis_tlast),
    .m_tready(m_axis_tready),
    .m_tuser ({{(8 - USER_WIDTH) {1'b0}}, m_axis_tuser})
);

// Both phy_receive instances, RC and EP.
bind phy_receive pr7d_params #(
    .P_CLK_RATE     (CLK_RATE),
    .P_MAX_NUM_LANES(MAX_NUM_LANES),
    .P_DATA_WIDTH   (DATA_WIDTH),
    .P_STRB_WIDTH   (STRB_WIDTH),
    .P_KEEP_WIDTH   (KEEP_WIDTH),
    .P_USER_WIDTH   (USER_WIDTH)
) u_pr7d_params ();

bind phy_receive pr7d_rx #(
    .P_DATA_WIDTH   (DATA_WIDTH),
    .P_KEEP_WIDTH   (KEEP_WIDTH),
    .P_MAX_NUM_LANES(MAX_NUM_LANES)
) u_pr7d_rx (
    .clk      (pipe_rx_usr_clk_i),
    .rst      (rst_i),
    .link_up  (link_up_i),
    .rx_data  (pipe_data_i),
    .rx_valid (pipe_data_valid_i),
    .rx_k     (pipe_data_k_i),
    .ba_data  (block_alignment_data),
    .ba_valid (block_alignment_data_valid),
    .ba_k     (block_alignment_data_k),
    .pk_data  (packer_data),
    .pk_valid (packer_data_valid),
    .pk_k     (packer_data_k),
    .dh_tkeep (tlp_axis_tkeep),
    .dh_tvalid(tlp_axis_tvalid),
    .dh_tlast (tlp_axis_tlast),
    .dh_tready(tlp_axis_tready),
    .m_tkeep  (m_dllp_axis_tkeep),
    .m_tvalid (m_dllp_axis_tvalid),
    .m_tlast  (m_dllp_axis_tlast),
    .m_tready (m_dllp_axis_tready)
);

// Both phy_transmit instances, RC and EP.
bind phy_transmit pr7d_tx #(
    .P_DATA_WIDTH   (DATA_WIDTH),
    .P_KEEP_WIDTH   (KEEP_WIDTH),
    .P_MAX_NUM_LANES(MAX_NUM_LANES)
) u_pr7d_tx (
    .clk     (pipe_tx_usr_clk_i),
    .rst     (rst_i),
    .link_up (link_up_i),
    .tx_data (pipe_data_o),
    .tx_valid(pipe_data_valid_o),
    .tx_k    (pipe_data_k_o),
    .s_tkeep (s_dllp_axis_tkeep),
    .s_tvalid(s_dllp_axis_tvalid),
    .s_tlast (s_dllp_axis_tlast),
    .s_tready(s_dllp_axis_tready)
);

// The reset terms, one bind per top, because the signal has a different name on
// each side and phy_receive only ever sees the OR of the two.
bind pcie_phy_top pr7d_reset u_pr7d_reset (
    .clk          (pipe_rx_usr_clk_i),
    .rst_top      (rst_i),
    .phystatus_rst(phy_phystatus_rst)
);

bind pcie_endpoint_top pr7d_reset u_pr7d_reset (
    .clk          (pipe_rx_usr_clk_i),
    .rst_top      (rst_i),
    .phystatus_rst(phy_phystatus_rst_i)
);
