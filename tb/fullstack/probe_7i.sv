// ===========================================================================
// probe_7i.sv -- sec 63 #7i Phase 1 instrumentation.  MEASUREMENT ONLY.
//
// !! THIS FILE EXISTS BECAUSE probe_7f.sv's EQUIVALENT WAS WRONG, and the way
// it was wrong is the rule this file is built to obey.
//
//   probe_7f.sv:674 printed   crcmatch = (crc_rx == crc_calc)
//
// -- raw register against raw register.  That is NOT the expression dllp2tlp
// evaluates.  The RTL compares  lcrc32d32 == crc_from_tlp_r,  where
// lcrc32d32 = {~crc_calculated_r[0], ... ~crc_calculated_r[31]}.  The probe
// re-implemented the predicate instead of sampling it, got 0 on 95/95 packets
// -- which is what a CORRECT dut produces -- and the conformance register
// carried "#20: the receive LCRC check never fires" for two rungs on that
// reading.
//
// It also sampled at  s_tvalid && s_tready && s_tlast,  one cycle BEFORE
// ST_CHECK_CRC, where crc_from_tlp_r has not yet loaded (crc_from_tlp_c is
// assigned on that very beat).  That is the "one-packet lag" #7f Phase 2c had
// to introduce to make its relation land.  SS22.89's phase problem, on a probe
// rather than on a waiter.
//
// So, three rules here, and they are the point of the file:
//   1. SAMPLE THE DUT'S OWN OPERANDS, BY THEIR RTL NAMES.  Never restate the
//      predicate.  Both operands are printed raw and the comparison is done
//      OFFLINE in Python (SS22.92).
//   2. SAMPLE WHERE THE DUT DECIDES.  Every verdict line is gated on
//      curr_state == ST_CHECK_CRC, which is where :489 and :519 are evaluated,
//      not on the last beat at the input port.
//   3. ANSWER "DELIVERED?" AT THE DELIVERY PORT.  m_tlp_axis is downstream of
//      the frame FIFO that DROP_BAD_FRAME acts on; tlp_axis is upstream of it.
//      A probe on the wrong side of that FIFO cannot see a drop at all.
// ===========================================================================

// -- (1) THE VERDICT, sampled in ST_CHECK_CRC ------------------------------
// Raw operands only.  ST_CHECK_CRC is state index 4 in dll_rx_st_e.
module pr7i_verdict (
    input logic        clk, rst,
    input logic [4:0]  state,
    input logic [31:0] lcrc32d32, crc_from_tlp_r, crc_calculated_r,
    input logic        tlp_nullified_r,
    input logic [11:0] next_expected_seq_num_r, next_transmit_seq_r,
    input logic        response_is_nak_r, nak_scheduled_r,
    input logic [11:0] response_seq_r,
    input logic        pending_tlp_valid_r, tlp_axis_tready
);
  localparam logic [4:0] ST_CHECK_CRC_IDX = 5'd4;
  longint unsigned cyc = 0, n = 0;
  always @(posedge clk) begin
    if (!rst) begin
      cyc <= cyc + 1;
      if (state == ST_CHECK_CRC_IDX) begin
        n <= n + 1;
        $display("PR7I_V scope=%m n=%0d cyc=%0d lcrc32d32=0x%08h crc_from_tlp_r=0x%08h crc_calculated_r=0x%08h nullified=%0b next_exp=%0d next_tx=%0d is_nak=%0b nak_sched=%0b resp_seq=%0d pend=%0b tready=%0b",
                 n + 1, cyc, lcrc32d32, crc_from_tlp_r, crc_calculated_r,
                 tlp_nullified_r, next_expected_seq_num_r, next_transmit_seq_r,
                 response_is_nak_r, nak_scheduled_r, response_seq_r,
                 pending_tlp_valid_r, tlp_axis_tready);
      end
    end
  end
  final $display("PR7I_VSUM scope=%m check_crc_visits=%0d", n);
endmodule

bind dllp2tlp pr7i_verdict u_pr7i_v (
    .clk(clk_i), .rst(rst_i),
    .state({{(5-$bits(curr_state)){1'b0}}, curr_state}),
    .lcrc32d32(lcrc32d32), .crc_from_tlp_r(crc_from_tlp_r),
    .crc_calculated_r(crc_calculated_r),
    .tlp_nullified_r(tlp_nullified_r),
    .next_expected_seq_num_r(next_expected_seq_num_r),
    .next_transmit_seq_r(next_transmit_seq_r),
    .response_is_nak_r(response_is_nak_r),
    .nak_scheduled_r(nak_scheduled_r),
    .response_seq_r(response_seq_r),
    .pending_tlp_valid_r(pending_tlp_valid_r),
    .tlp_axis_tready(tlp_axis_tready));

// -- (2) DELIVERY, at the port DOWNSTREAM of the drop FIFO ------------------
module pr7i_deliver (
    input logic        clk, rst,
    input logic [31:0] tdata,
    input logic        tvalid, tready, tlast,
    input logic [0:0]  tuser
);
  longint unsigned cyc = 0, beats = 0, frames = 0;
  always @(posedge clk) begin
    if (!rst) begin
      cyc <= cyc + 1;
      if (tvalid && tready) begin
        beats <= beats + 1;
        $display("PR7I_D scope=%m cyc=%0d beat=%0d tdata=0x%08h tlast=%0b tuser=0x%01h",
                 cyc, beats + 1, tdata, tlast, tuser);
        if (tlast) frames <= frames + 1;
      end
    end
  end
  final $display("PR7I_DSUM scope=%m delivered_beats=%0d delivered_frames=%0d", beats, frames);
endmodule

bind dllp2tlp pr7i_deliver u_pr7i_d (
    .clk(clk_i), .rst(rst_i),
    .tdata(m_tlp_axis_tdata), .tvalid(m_tlp_axis_tvalid),
    .tready(m_tlp_axis_tready), .tlast(m_tlp_axis_tlast),
    .tuser(m_tlp_axis_tuser));

// -- (3) NOT A PROBE: where 1-c's DLLP bytes came from ---------------------
// 1-c needed DLLP and TLP frames as BYTES, checked against a spec model.  No
// probe was written for it, because verilate_dll_comprehensive already logs
// every m_phy_axis frame in full ("frame N: length=L data=..."), which is
// 5,177 distinct TLP frames and 70 distinct DLLP frames from one run -- three
// orders of magnitude more than a new probe would have produced, and already
// in the gate's own harness.  The bytes are in
// pcie_docs/evidence/fullstack/wire_7i/ and the model is crc_oracle_7i.py.
//
// Recorded here because "there is no probe for 1-c" should read as a choice
// with a reason, not as an omission.
