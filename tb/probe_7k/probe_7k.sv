// =============================================================================
// §63 #7k Phase 1 -- the starved-link probe. BENCH-ONLY, `bind`, NOT A GATE ROW,
// IN NO CORE.  Appended to verilate_fullstack's staged .vc by run_probe_7k.sh
// (the run_probe_7g3.sh method) and run against the W1 row alone.
//
// It is the SECOND instrument on W1's facts.  The row's own W1Capture samples
// from cocotb; this one samples from inside the RTL, so a sampling-phase or
// hierarchy error in either shows up as a disagreement (analyse_7k.py cross).
//
// Rules (§22.92, and probe_7g2.sv's list):
//  (1) RAW EVENTS ONLY: one line per change or event, stamped with $time; no
//      pairing, no counters, no classification in SV.
//  (2) every field under its RTL name;
//  (3) one file per instance, named from %m -- never stdout (§63 #7i's spliced
//      line);
//  (4) sampled at the clock edge the flops sample on, with the pre-edge values
//      (always @(posedge clk) reads what the flops read);
//  (5) no budget: every event is kept.
// x1 bench, RETRY_TLP_SIZE = 3 on both stacks (pcie_phy_top.sv:143,
// pcie_endpoint_top.sv:42) -- the three slot taps below assume it and the bind
// asserts it at elaboration through the port widths.
// =============================================================================
`timescale 1ns / 1ps

// ---- retry_management: every slot's state and REPLAY_NUM, the buffer, the
//      error output, the REPLAY_TIMER start event and every Ack/Nak in. -------
module pr7k_rm (
    input logic        clk,
    input logic        rst,
    input logic [2:0]  st0, input logic [2:0] st1, input logic [2:0] st2,
    input logic [1:0]  cnt0, input logic [1:0] cnt1, input logic [1:0] cnt2,
    input logic [2:0]  retrys_r,
    input logic [35:0] ack_seq_mem_r,
    input logic        retry_err_o,
    input logic        tlp_sent_i,
    input logic [11:0] tlp_sent_seq_i,
    input logic        ack_nack_vld_i,
    input logic        ack_nack_i,
    input logic [11:0] ack_seq_num_i
);
  integer fd;
  logic [2:0] p_st0, p_st1, p_st2;
  logic [1:0] p_c0, p_c1, p_c2;
  logic [2:0] p_rt;
  logic       p_err;
  logic       first = 1'b1;
  initial fd = $fopen($sformatf("pr7k_rm.%m.log"), "w");
  always @(posedge clk) begin
    if (!rst) begin
      if (first || st0 != p_st0 || cnt0 != p_c0)
        $fwrite(fd, "S %0t 0 %0d %0d\n", $time, st0, cnt0);
      if (first || st1 != p_st1 || cnt1 != p_c1)
        $fwrite(fd, "S %0t 1 %0d %0d\n", $time, st1, cnt1);
      if (first || st2 != p_st2 || cnt2 != p_c2)
        $fwrite(fd, "S %0t 2 %0d %0d\n", $time, st2, cnt2);
      if (first || retrys_r != p_rt)
        $fwrite(fd, "B %0t %0d %03h %03h %03h\n", $time, retrys_r,
                ack_seq_mem_r[11:0], ack_seq_mem_r[23:12], ack_seq_mem_r[35:24]);
      if (first || retry_err_o != p_err)
        $fwrite(fd, "E %0t %0d\n", $time, retry_err_o);
      if (tlp_sent_i)
        $fwrite(fd, "T %0t %03h\n", $time, tlp_sent_seq_i);
      if (ack_nack_vld_i)
        $fwrite(fd, "A %0t %0d %03h\n", $time, ack_nack_i, ack_seq_num_i);
      first <= 1'b0;
      p_st0 <= st0; p_st1 <= st1; p_st2 <= st2;
      p_c0 <= cnt0; p_c1 <= cnt1; p_c2 <= cnt2;
      p_rt <= retrys_r; p_err <= retry_err_o;
    end
  end
  final $fclose(fd);
endmodule

bind retry_management pr7k_rm u_pr7k_rm (
    .clk(clk_i), .rst(rst_i),
    .st0(3'(gen_retry_counters[0].curr_state)),
    .st1(3'(gen_retry_counters[1].curr_state)),
    .st2(3'(gen_retry_counters[2].curr_state)),
    .cnt0(2'(gen_retry_counters[0].replay_cnt_r)),
    .cnt1(2'(gen_retry_counters[1].replay_cnt_r)),
    .cnt2(2'(gen_retry_counters[2].replay_cnt_r)),
    .retrys_r(retrys_r), .ack_seq_mem_r(ack_seq_mem_r),
    .retry_err_o(retry_err_o),
    .tlp_sent_i(tlp_sent_i), .tlp_sent_seq_i(tlp_sent_seq_i),
    .ack_nack_vld_i(ack_nack_vld_i), .ack_nack_i(ack_nack_i),
    .ack_seq_num_i(ack_seq_num_i)
);

// ---- the LTSSM: state and link_up_o, every change ---------------------------
module pr7k_lt (
    input logic        clk,
    input logic [19:0] curr_state,
    input logic        link_up_o
);
  integer fd;
  logic [19:0] q;
  logic        u;
  logic        first = 1'b1;
  initial fd = $fopen($sformatf("pr7k_lt.%m.log"), "w");
  always @(posedge clk) begin
    if (first || curr_state !== q) $fwrite(fd, "L %0t %05h\n", $time, curr_state);
    if (first || link_up_o !== u)  $fwrite(fd, "U %0t %0d\n", $time, link_up_o);
    first <= 1'b0; q <= curr_state; u <= link_up_o;
  end
  final $fclose(fd);
endmodule

bind pcie_ltssm_downstream pr7k_lt u_pr7k_lt (
    .clk(clk_i), .curr_state(20'(curr_state)), .link_up_o(link_up_o)
);

// ---- pcie_datalink_layer: DLCMSM, FC-init state, link_up in, and the first
//      beat of every DLLP it hands its PHY (tuser bit 0) -- type byte raw. ----
module pr7k_dll #(
    parameter int UW = 1
) (
    input logic          clk,
    input logic          rst,
    input logic          phy_link_up_i,
    input logic [2:0]    dlcmsm,
    input logic [4:0]    fci,
    input logic [2:0]    hdl,
    input logic [31:0]   tdata,
    input logic [UW-1:0] tuser,
    input logic          tvalid,
    input logic          tready,
    input logic          tlast
);
  integer fd;
  logic [2:0] p_dl, p_hdl;
  logic [4:0] p_fci;
  logic       p_lu;
  logic       in_pkt = 1'b0;
  logic       first = 1'b1;
  initial fd = $fopen($sformatf("pr7k_dll.%m.log"), "w");
  always @(posedge clk) begin
    if (first || phy_link_up_i !== p_lu) $fwrite(fd, "U %0t %0d\n", $time, phy_link_up_i);
    if (first || dlcmsm !== p_dl)        $fwrite(fd, "D %0t %0d\n", $time, dlcmsm);
    if (first || fci !== p_fci)          $fwrite(fd, "F %0t %0d\n", $time, fci);
    if (first || hdl !== p_hdl)          $fwrite(fd, "H %0t %0d\n", $time, hdl);
    if (tvalid && tready) begin
      if (!in_pkt && tuser[0]) $fwrite(fd, "X %0t %02h\n", $time, tdata[7:0]);
      in_pkt <= !tlast;
    end
    first <= 1'b0; p_lu <= phy_link_up_i; p_dl <= dlcmsm; p_fci <= fci; p_hdl <= hdl;
  end
  final $fclose(fd);
endmodule

bind pcie_datalink_layer pr7k_dll #(.UW(USER_WIDTH)) u_pr7k_dll (
    .clk(clk_i), .rst(rst_i), .phy_link_up_i(phy_link_up_i),
    .dlcmsm(3'(pcie_datalink_init_inst.curr_state)),
    .fci(5'(pcie_flow_ctrl_init_inst.curr_state)),
    .hdl(3'(dllp_receive_inst.dllp_handler_inst.curr_state)),
    .tdata(m_phy_axis_tdata[31:0]), .tuser(m_phy_axis_tuser),
    .tvalid(m_phy_axis_tvalid), .tready(m_phy_axis_tready), .tlast(m_phy_axis_tlast)
);

// ---- phy_transmit's PIPE output: every K Symbol, lane 0 (x1), both bytes.
//      For F10 (Kourosh 2026-09-26: "DLL traffic between TS1s, a measurement
//      on the fixed tree"): COM BC opens every Ordered Set (TS1/TS2/SKP), SKP
//      1C, STP FB and SDP 5C open packets, END FD / EDB FE close them.  K codes
//      are not scrambled, so this is readable on the ciphertext side.  Raw:
//      the pairing against the LTSSM's Recovery interval is analyse_7k.py's.
module pr7k_wire (
    input logic        clk,
    input logic        rst,
    input logic [31:0] data,
    input logic [3:0]  k,
    input logic        valid
);
  integer fd;
  initial fd = $fopen($sformatf("pr7k_wire.%m.log"), "w");
  always @(posedge clk) begin
    if (!rst && valid) begin
      for (int b = 0; b < 2; b++) begin
        if (k[b]) $fwrite(fd, "K %0t %02h %0d\n", $time, data[b*8+:8], b);
      end
    end
  end
  final $fclose(fd);
endmodule

bind phy_transmit pr7k_wire u_pr7k_wire (
    .clk(pipe_tx_usr_clk_i), .rst(rst_i), .data(pipe_data_o[31:0]),
    .k(pipe_data_k_o[3:0]), .valid(pipe_data_valid_o[0])
);
