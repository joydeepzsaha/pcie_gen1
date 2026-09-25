// sec 63 #7g-3 Phase 1 -- NOT A GATE ROW, NOT IN ANY CORE.  Full-stack probe for
// C-7G3-1/2/3 (tlp2dllp tuser at in, skid, pipeline and out), C-7G3-8 (frame
// start alignment and the registered-word END arm at data_handler), and
// C-7G3-16 / G7-7 (data_valid_i at data_handler against the LTSSM state).
//
// Raw events only (sec 22.92): `<tag> <$time> <fields>`, one file per instance
// (%m), no pairing, no counters, no classification.  All pairing happens in
// analyse_7g3.py, which opens with a known-answer self-test.
// Signal names are the RTL's own (sec 22.92 naming clause).
// Width-clean on purpose: fullstack and rc_top are warnings-fatal.

// ---- tlp2dllp: the four tuser points the #7g-1 defect is about -------------
module pr7g3_t2d (
    input logic       clk,
    input logic [4:0] s_tuser,  input logic s_hs, input logic s_tlast,
    input logic [4:0] k_tuser,  input logic k_hs,
    input logic [4:0] p_tuser,  input logic p_hs,
    input logic [4:0] m_tuser,  input logic m_hs, input logic m_tlast,
    input logic [7:0] uw
);
  integer fd;
  initial begin
    fd = $fopen($sformatf("pr7g3_t2d.%m.log"), "w");
    $fwrite(fd, "W 0 USER_WIDTH=%0d\n", uw);
  end
  always @(posedge clk) begin
    if (s_hs) $fwrite(fd, "S %0t %02h %0d\n", $time, s_tuser, s_tlast);
    if (k_hs) $fwrite(fd, "K %0t %02h\n", $time, k_tuser);
    if (p_hs) $fwrite(fd, "P %0t %02h\n", $time, p_tuser);
    if (m_hs) $fwrite(fd, "M %0t %02h %0d\n", $time, m_tuser, m_tlast);
  end
  final $fclose(fd);
endmodule

bind tlp2dllp pr7g3_t2d u_pr7g3_t2d (
    .clk    (clk_i),
    .s_tuser(5'(s_axis_tuser)),        .s_hs(s_axis_tvalid && s_axis_tready), .s_tlast(s_axis_tlast),
    .k_tuser(5'(skid_axis_tuser)),     .k_hs(skid_axis_tvalid && skid_axis_tready),
    .p_tuser(5'(pipeline_axis_tuser)), .p_hs(pipeline_axis_tvalid && skid_axis_tready),
    .m_tuser(5'(m_axis_tuser)),        .m_hs(m_axis_tvalid && m_axis_tready), .m_tlast(m_axis_tlast),
    .uw     (8'(USER_WIDTH))
);

// ---- data_handler: valid edges, framing K at the input, the two END arms,
//      output beats, link_up edges.  Lane 0 only (both stacks are x1).
module pr7g3_dh (
    input logic        clk,
    input logic        rst,
    input logic        valid,       // |data_valid_i
    input logic        link_up,
    input logic [31:0] din,
    input logic [3:0]  kin,
    input logic [31:0] dr,
    input logic [3:0]  kr,
    input logic        start_r,
    input logic [4:0]  st,
    input logic        axis_rdy,
    input logic [5:0]  wc,
    input logic        o_hs,
    input logic [3:0]  o_tkeep,
    input logic        o_tlast,
    input logic [4:0]  o_tuser
);
  integer fd;
  logic v_q, u_q, first;
  initial begin
    fd = $fopen($sformatf("pr7g3_dh.%m.log"), "w");
    first = 1'b1;
  end
  always @(posedge clk) begin
    if (first) begin
      first <= 1'b0; v_q <= valid; u_q <= link_up;
      $fwrite(fd, "V %0t %0d\nU %0t %0d\n", $time, valid, $time, link_up);
    end else begin
      if (valid !== v_q)   begin v_q <= valid;   $fwrite(fd, "V %0t %0d\n", $time, valid); end
      if (link_up !== u_q) begin u_q <= link_up; $fwrite(fd, "U %0t %0d\n", $time, link_up); end
    end
    if (!rst && valid) begin
      for (int b = 0; b < 4; b++) begin
        if (kin[b] && (din[8*b+:8] == 8'h5C || din[8*b+:8] == 8'hFB ||
                       din[8*b+:8] == 8'hFD || din[8*b+:8] == 8'hFE))
          $fwrite(fd, "K %0t %0d %02h st=%0d wc=%0d\n", $time, b, din[8*b+:8], st, wc);
      end
    end
    // The registered-word END arm (registered as data_handler:256): its own
    // guard, read off the RTL -- ST_TX (=1), a beat taken, END/EDB in data_r
    // at byte b, not a fresh start.
    if (!rst && st == 5'd1 && axis_rdy && valid && !start_r) begin
      for (int b = 0; b < 4; b++) begin
        if (kr[b] && (dr[8*b+:8] == 8'hFD || dr[8*b+:8] == 8'hFE))
          $fwrite(fd, "R %0t %0d wc=%0d\n", $time, b, wc);
      end
    end
    if (o_hs) $fwrite(fd, "O %0t %h %0d %02h\n", $time, o_tkeep, o_tlast, o_tuser);
  end
  final $fclose(fd);
endmodule

bind data_handler pr7g3_dh u_pr7g3_dh (
    .clk     (clk_i),
    .rst     (rst_i),
    .valid   (|data_valid_i),
    .link_up (phy_link_up_i),
    .din     (32'(data_i)),
    .kin     (4'(data_k_i)),
    .dr      (32'(data_r)),
    .kr      (4'(data_k_r)),
    .start_r (data_start_r),
    .st      (5'(curr_state)),
    .axis_rdy(data_handler_axis_tready),
    .wc      (word_count_r),
    .o_hs    (m_dllp_axis_tvalid && m_dllp_axis_tready),
    .o_tkeep (4'(m_dllp_axis_tkeep)),
    .o_tlast (m_dllp_axis_tlast),
    .o_tuser (5'(m_dllp_axis_tuser))
);

// ---- the LTSSM: every state change, raw ------------------------------------
module pr7g3_lt (
    input logic        clk,
    input logic [19:0] st
);
  integer fd;
  logic [19:0] q;
  logic first;
  initial begin
    fd = $fopen($sformatf("pr7g3_lt.%m.log"), "w");
    first = 1'b1;
  end
  always @(posedge clk) begin
    if (first || st !== q) begin
      first <= 1'b0; q <= st;
      $fwrite(fd, "L %0t %05h\n", $time, st);
    end
  end
  final $fclose(fd);
endmodule

bind pcie_ltssm_downstream pr7g3_lt u_pr7g3_lt (
    .clk(clk_i),
    .st (20'(curr_state))
);
