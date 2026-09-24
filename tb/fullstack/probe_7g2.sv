// =============================================================================
// §63 #7g-2 Phase 1 -- the timer probe. BENCH-ONLY, `bind`, NOT A GATE ROW.
//
// !! NOTHING IN src/ CHANGES FOR THIS FILE, and verilate_fullstack is untouched:
// own fileset, own target (§63 #7d's rule). A probe that perturbs the row it
// measures is not a measurement of that row.
//
// == WHAT IT ANSWERS (pcie_docs evidence/cleanup-7g/PREDICTIONS_7G2.md) =======
//   C-7G2-1/2  every DLLP dllp_fc_update emits, and every exit from its ST_IDLE
//              (periodic vs release vs Ack), both stacks
//   C-7G2-3    Ack latency at the Port: the TLP's END at phy_receive's PIPE
//              input -> its Ack's SDP at phy_transmit's PIPE output
//   C-7G2-4/5  each retry slot's lifetime, set -> cleared, and its offset from
//              the TLP's END on the wire
//   C-7G2-8    tag allocation -> TL->DLL -> DLL->PHY -> PIPE STP / END
//
// == RULES THIS FILE OBEYS ===================================================
// (1) §22.92: RAW EVENTS ONLY. Every line is one event at the instant it
//     happens, stamped with $time. No pairing, no arithmetic, no counters in
//     SV; all of that is offline in Python behind a known-answer self-test.
// (2) §22.92 naming clause: every field is printed under its RTL name.
// (3) ⚠️ ONE FILE PER INSTANCE, NOT STDOUT. §63 #7i measured a DUT line cut
//     mid-word by cocotb's logger on the shared stdout ("%Warnin" + cocotb
//     text), which is how .diag.narrow became a hash over a race. Each probe
//     instance opens its own file, named from %m, so no two writers share a
//     stream and no line can be spliced.
// (4) Sampled at the clock edge with the values the flops sample there, so
//     every event carries the edge at which it took effect (§22.89: the window
//     is defined by the edge, and the edge is stated).
// (5) No budget: the run is short enough that every event is kept. A bounded
//     log that does not say it is bounded reads as "this is everything".
//
// x1 bench: every symbol seam is lane 0, [31:0] / [3:0]. Stated, not assumed.
// =============================================================================
`timescale 1ns / 1ps

// -----------------------------------------------------------------------------
// dllp_fc_update: every DLLP it emits (first beat, type byte) and every exit
// from ST_IDLE with the timer value and the cause.
// -----------------------------------------------------------------------------
module pr7g2_ufc #(
    parameter int FCW = 0,
    parameter int TW  = 1,
    parameter int DW  = 32
) (
    input logic          clk,
    input logic          rst,
    input logic [4:0]    state,
    input logic [4:0]    next_state,
    input logic [TW-1:0] timer,
    input logic          rel_c,
    input logic          start_fc,
    input logic          dl_active,
    input logic [DW-1:0] tdata,
    input logic          tvalid,
    input logic          tready,
    input logic          tlast
);
  integer fd;
  logic   in_pkt = 1'b0;
  initial begin
    fd = $fopen($sformatf("pr7g2_ufc__%m.log"), "w");
    $fdisplay(fd, "PARAM scope=%m FcWaitPeriod=%0d TimerWidth=%0d", FCW, TW);
  end
  always @(posedge clk) begin
    if (!rst) begin
      if (tvalid && tready) begin
        if (!in_pkt)
          $fdisplay(fd, "UFC_TX t=%0t type=%02h w=%08h", $time, tdata[7:0], tdata[31:0]);
        in_pkt <= !tlast;
      end
      // ST_IDLE = 0. Exits: 1 = ST_SEND_ACK (Ack/Nak request), 3 = ST_UPDATE_P,
      // 5 = ST_UPDATE_NP. rel_c says release (1) or periodic (0) on 3/5.
      if (state == 5'd0 && next_state != 5'd0)
        $fdisplay(fd, "UFC_EXIT t=%0t next=%0d timer=%0d rel=%0d start_fc=%0d dl_active=%0d",
                  $time, next_state, timer, rel_c, start_fc, dl_active);
    end
  end
  final $fclose(fd);
endmodule

// -----------------------------------------------------------------------------
// dllp2tlp: the Ack/Nak request (fc_start_r rising) with the registered
// response it carries. The accept decision is the cycle before this edge.
// -----------------------------------------------------------------------------
module pr7g2_d2t (
    input logic        clk,
    input logic        rst,
    input logic        fc_start,
    input logic [11:0] resp_seq,
    input logic        resp_nak
);
  integer fd;
  logic   prev = 1'b0;
  initial fd = $fopen($sformatf("pr7g2_d2t__%m.log"), "w");
  always @(posedge clk) begin
    if (!rst) begin
      prev <= fc_start;
      if (fc_start && !prev)
        $fdisplay(fd, "ACKREQ t=%0t response_seq_r=%0d response_is_nak_r=%0d",
                  $time, resp_seq, resp_nak);
    end
  end
  final $fclose(fd);
endmodule

// -----------------------------------------------------------------------------
// retry_management: slot set / cleared (with the slot's sequence number), the
// tx_valid_i strobe, every Ack/Nak it is told about, replay and error rises.
// -----------------------------------------------------------------------------
module pr7g2_rm #(
    parameter int N = 3
) (
    input logic          clk,
    input logic          rst,
    input logic          tx_valid,
    input logic [11:0]   tx_seq,
    input logic          ack_nack,
    input logic          ack_vld,
    input logic [11:0]   ack_seq,
    input logic [N-1:0]  retrys,
    input logic [N*12-1:0] seqmem,
    input logic [N-1:0]  retry_valid,
    input logic          retry_err
);
  integer fd;
  logic [N-1:0] prev_retrys = '0, prev_rv = '0;
  logic         prev_err = 1'b0;
  initial fd = $fopen($sformatf("pr7g2_rm__%m.log"), "w");
  always @(posedge clk) begin
    if (!rst) begin
      prev_retrys <= retrys;
      prev_rv     <= retry_valid;
      prev_err    <= retry_err;
      if (tx_valid)
        $fdisplay(fd, "RM_TXV t=%0t tx_seq_num_i=%0d", $time, tx_seq);
      if (ack_vld)
        $fdisplay(fd, "RM_ACKIN t=%0t ack_nack_i=%0d ack_seq_num_i=%0d", $time, ack_nack, ack_seq);
      for (int i = 0; i < N; i++) begin
        if (retrys[i] && !prev_retrys[i])
          $fdisplay(fd, "RM_SET t=%0t slot=%0d seq=%0d", $time, i, seqmem[i*12+:12]);
        if (!retrys[i] && prev_retrys[i])
          $fdisplay(fd, "RM_CLR t=%0t slot=%0d", $time, i);
        if (retry_valid[i] && !prev_rv[i])
          $fdisplay(fd, "RM_REPLAY t=%0t slot=%0d", $time, i);
      end
      if (retry_err && !prev_err)
        $fdisplay(fd, "RM_ERR t=%0t", $time);
    end
  end
  final $fclose(fd);
endmodule

// -----------------------------------------------------------------------------
// tlp_request_tracker: allocation, completion, timeout strobes with tags.
// -----------------------------------------------------------------------------
module pr7g2_trk #(
    parameter int unsigned LIMIT = 0
) (
    input logic       clk,
    input logic       rst,
    input logic       alloc_valid,
    input logic       alloc_ready,
    input logic [7:0] alloc_tag,
    input logic       cpl_fire,
    input logic [7:0] cpl_tag,
    input logic       tmo_valid,
    input logic [7:0] tmo_tag
);
  integer fd;
  initial begin
    fd = $fopen($sformatf("pr7g2_trk__%m.log"), "w");
    $fdisplay(fd, "PARAM scope=%m TIMEOUT_LIMIT=%0d", LIMIT);
  end
  always @(posedge clk) begin
    if (!rst) begin
      if (alloc_valid && alloc_ready)
        $fdisplay(fd, "TRK_ALLOC t=%0t tag=%0d", $time, alloc_tag);
      if (cpl_fire)
        $fdisplay(fd, "TRK_CPL t=%0t tag=%0d", $time, cpl_tag);
      if (tmo_valid)
        $fdisplay(fd, "TRK_TMO t=%0t tag=%0d", $time, tmo_tag);
    end
  end
  final $fclose(fd);
endmodule

// -----------------------------------------------------------------------------
// pcie_datalink_layer: its four AXIS boundaries. First three beats of every
// TL->DLL packet (the tag is in DW1); first and last beat of every packet on
// the other three, with tuser (bit 0 DLLP, bit 1 TLP) as the classifier --
// never beat arithmetic (#7e).
// -----------------------------------------------------------------------------
module pr7g2_dll #(
    parameter int DW = 32,
    parameter int UW = 1
) (
    input logic          clk,
    input logic          rst,
    input logic [DW-1:0] tl_tdata,
    input logic          tl_tvalid,
    input logic          tl_tready,
    input logic          tl_tlast,
    input logic [DW-1:0] up_tdata,
    input logic          up_tvalid,
    input logic          up_tready,
    input logic          up_tlast,
    input logic [DW-1:0] tx_tdata,
    input logic [UW-1:0] tx_tuser,
    input logic          tx_tvalid,
    input logic          tx_tready,
    input logic          tx_tlast,
    input logic [DW-1:0] rx_tdata,
    input logic [UW-1:0] rx_tuser,
    input logic          rx_tvalid,
    input logic          rx_tready,
    input logic          rx_tlast
);
  integer fd;
  int     tl_idx = 0;
  logic   up_in = 1'b0, tx_in = 1'b0, rx_in = 1'b0;
  initial fd = $fopen($sformatf("pr7g2_dll__%m.log"), "w");
  always @(posedge clk) begin
    if (!rst) begin
      if (tl_tvalid && tl_tready) begin
        if (tl_idx < 3)
          $fdisplay(fd, "TL_IN t=%0t idx=%0d w=%08h last=%0d", $time, tl_idx, tl_tdata[31:0], tl_tlast);
        tl_idx <= tl_tlast ? 0 : tl_idx + 1;
      end
      if (up_tvalid && up_tready) begin
        if (!up_in) $fdisplay(fd, "UP_FIRST t=%0t w=%08h", $time, up_tdata[31:0]);
        if (up_tlast) $fdisplay(fd, "UP_LAST t=%0t", $time);
        up_in <= !up_tlast;
      end
      if (tx_tvalid && tx_tready) begin
        if (!tx_in) $fdisplay(fd, "TX_FIRST t=%0t user=%0h w=%08h", $time, tx_tuser, tx_tdata[31:0]);
        if (tx_tlast) $fdisplay(fd, "TX_LAST t=%0t user=%0h", $time, tx_tuser);
        tx_in <= !tx_tlast;
      end
      if (rx_tvalid && rx_tready) begin
        if (!rx_in) $fdisplay(fd, "RX_FIRST t=%0t user=%0h w=%08h", $time, rx_tuser, rx_tdata[31:0]);
        if (rx_tlast) $fdisplay(fd, "RX_LAST t=%0t user=%0h", $time, rx_tuser);
        rx_in <= !rx_tlast;
      end
    end
  end
  final $fclose(fd);
endmodule

// -----------------------------------------------------------------------------
// A PIPE symbol seam (phy_transmit output / phy_receive input): the framing K
// symbols only -- STP FB, SDP 5C, END FD, EDB FE -- gated on the seam's own
// valid (#7e: count symbols, not cycles), with the byte lane so two framing
// symbols in one cycle keep their order. K codes are not scrambled, so this
// identity survives the ciphertext side of the seam.
// -----------------------------------------------------------------------------
module pr7g2_sym (
    input logic        clk,
    input logic        rst,
    input logic [31:0] data,
    input logic [3:0]  k,
    input logic        valid
);
  integer fd;
  initial fd = $fopen($sformatf("pr7g2_sym__%m.log"), "w");
  always @(posedge clk) begin
    if (!rst && valid) begin
      for (int b = 0; b < 4; b++) begin
        if (k[b] && (data[b*8+:8] == 8'hFB || data[b*8+:8] == 8'h5C ||
                     data[b*8+:8] == 8'hFD || data[b*8+:8] == 8'hFE))
          $fdisplay(fd, "SYM t=%0t sym=%02h lane_byte=%0d", $time, data[b*8+:8], b);
      end
    end
  end
  final $fclose(fd);
endmodule

// =============================================================================
// The binds. Each fires once per instance of its target, RC and EP; %m in the
// file name says which.
// =============================================================================
bind dllp_fc_update pr7g2_ufc #(
    .FCW(FcWaitPeriod), .TW(TimerWidth), .DW(DATA_WIDTH)
) u_pr7g2_ufc (
    .clk(clk_i), .rst(rst_i), .state(curr_state), .next_state(next_state),
    .timer(timer_r), .rel_c(rel_seq_c), .start_fc(start_flow_control_i),
    .dl_active(link_status_i == DL_ACTIVE),
    .tdata(m_axis_tdata), .tvalid(m_axis_tvalid), .tready(m_axis_tready),
    .tlast(m_axis_tlast)
);

bind dllp2tlp pr7g2_d2t u_pr7g2_d2t (
    .clk(clk_i), .rst(rst_i), .fc_start(fc_start_r),
    .resp_seq(response_seq_r), .resp_nak(response_is_nak_r)
);

bind retry_management pr7g2_rm #(.N(RETRY_TLP_SIZE)) u_pr7g2_rm (
    .clk(clk_i), .rst(rst_i), .tx_valid(tx_valid_i), .tx_seq(tx_seq_num_i),
    .ack_nack(ack_nack_i), .ack_vld(ack_nack_vld_i), .ack_seq(ack_seq_num_i),
    .retrys(retrys_r), .seqmem(ack_seq_mem_r), .retry_valid(retry_valid_o),
    .retry_err(retry_err_o)
);

bind tlp_request_tracker pr7g2_trk #(.LIMIT(TIMEOUT_LIMIT)) u_pr7g2_trk (
    .clk(clk_i), .rst(rst_i), .alloc_valid(allocate_valid_i),
    .alloc_ready(allocate_ready_o), .alloc_tag(allocate_tag_o),
    .cpl_fire(completion_fire), .cpl_tag(completion_header_i.tag),
    .tmo_valid(cpl_timeout_valid_o), .tmo_tag(cpl_timeout_tag_o)
);

bind pcie_datalink_layer pr7g2_dll #(.DW(DATA_WIDTH), .UW(USER_WIDTH)) u_pr7g2_dll (
    .clk(clk_i), .rst(rst_i),
    .tl_tdata(s_tlp_axis_tdata), .tl_tvalid(s_tlp_axis_tvalid),
    .tl_tready(s_tlp_axis_tready), .tl_tlast(s_tlp_axis_tlast),
    .up_tdata(m_tlp_axis_tdata), .up_tvalid(m_tlp_axis_tvalid),
    .up_tready(m_tlp_axis_tready), .up_tlast(m_tlp_axis_tlast),
    .tx_tdata(m_phy_axis_tdata), .tx_tuser(m_phy_axis_tuser),
    .tx_tvalid(m_phy_axis_tvalid), .tx_tready(m_phy_axis_tready),
    .tx_tlast(m_phy_axis_tlast),
    .rx_tdata(s_phy_axis_tdata), .rx_tuser(s_phy_axis_tuser),
    .rx_tvalid(s_phy_axis_tvalid), .rx_tready(s_phy_axis_tready),
    .rx_tlast(s_phy_axis_tlast)
);

bind phy_transmit pr7g2_sym u_pr7g2_tx (
    .clk(pipe_tx_usr_clk_i), .rst(rst_i), .data(pipe_data_o[31:0]),
    .k(pipe_data_k_o[3:0]), .valid(pipe_data_valid_o[0])
);

bind phy_receive pr7g2_sym u_pr7g2_rx (
    .clk(pipe_rx_usr_clk_i), .rst(rst_i), .data(pipe_data_i[31:0]),
    .k(pipe_data_k_i[3:0]), .valid(pipe_data_valid_i[0])
);
