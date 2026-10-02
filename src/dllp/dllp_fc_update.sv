// ---------------------------------------------------------------------------
// dllp_fc_update -- Ack/Nak and UpdateFC transmitter for the receive path
//
// Original author: Idris Somoye
// Modified by: Kourosh Ghahramani
// Silicon Systems Research Lab, University of Washington
//
// Purpose
//   Transmits the Ack and Nak DLLPs that dllp2tlp requests, and, in
//   DL_Active, the UpdateFC-P and UpdateFC-NP DLLPs for VC0. An UpdateFC of
//   a type is owed when dllp2tlp's credits for that type differ from the
//   values last sent, or when that type's timer reaches FcWaitPeriod (30 us).
//   A pending Ack or Nak request goes first.
//
// Interfaces
//   Link         link_status_i: UpdateFC DLLPs are timed and sent only in
//                DL_ACTIVE.
//   Ack/Nak      start_flow_control_i, start_flow_control_ack_o: the request
//                and its acknowledge, which rises after the CRC beat and holds
//                until the request falls. next_transmit_seq_i[11:0]: the
//                AckNak_Seq_Num; tlp_nullified_i: 1 for a Nak. Both are taken
//                in the cycle ST_IDLE accepts the request.
//   Credits      ph_, pd_, nph_, npd_credits_allocated_i: CREDITS_ALLOCATED
//                from dllp2tlp, the HdrFC and DataFC each UpdateFC carries.
//   DLLP output  m_axis_*, through a skid buffer. With DATA_WIDTH = 32 each
//                DLLP is one 4-byte beat followed by one CRC beat (tkeep
//                0011b, tlast).
//
// Clock and reset
//   clk_i only. rst_i is synchronous and active high; pcie_datalink_layer
//   also asserts it while the link is down. CLK_PERIOD_NS sets FcWaitPeriod.
//
// Limitations
//   VC0 only. No UpdateFC-Cpl is sent: ST_UPDATE_CPL and ST_UPDATE_CPL_CRC
//   are declared and never entered. MAX_PAYLOAD_SIZE is not used.
//
// References
//   PCIe Base Spec r2.1, §2.6.1
//   PCIe Base Spec r2.1, §2.6.1.2
//   PCIe Base Spec r2.1, §3.4.1
//   PCIe Base Spec r2.1, §3.5.2.1
// ---------------------------------------------------------------------------
module dllp_fc_update
  import pcie_datalink_pkg::*;
#(
    // Link clock period in ns; sets FcWaitPeriod, the UpdateFC timer limit.
    parameter int CLK_PERIOD_NS    = 8,
    parameter int DATA_WIDTH       = 32,
    parameter int STRB_WIDTH       = DATA_WIDTH / 8,
    parameter int KEEP_WIDTH       = STRB_WIDTH,
    parameter int USER_WIDTH       = 3,
    parameter int MAX_PAYLOAD_SIZE = 256
) (
    input  logic                   clk_i,
    input  logic                   rst_i,
    input  pcie_dl_status_e        link_status_i,

    // ---- Ack/Nak request from dllp2tlp -------------------------------------
    input  logic                   start_flow_control_i,
    output logic                   start_flow_control_ack_o,
    input  logic            [15:0] next_transmit_seq_i,
    input  logic                   tlp_nullified_i,

    // ---- CREDITS_ALLOCATED from dllp2tlp -----------------------------------
    input  logic            [ 7:0] ph_credits_allocated_i,
    input  logic            [11:0] pd_credits_allocated_i,
    input  logic            [ 7:0] nph_credits_allocated_i,
    input  logic            [11:0] npd_credits_allocated_i,

    // ---- DLLP output -------------------------------------------------------
    output logic [(DATA_WIDTH)-1:0] m_axis_tdata,
    output logic [(KEEP_WIDTH)-1:0] m_axis_tkeep,
    output logic                    m_axis_tvalid,
    output logic                    m_axis_tlast,
    output logic [  USER_WIDTH-1:0] m_axis_tuser,
    input  logic                    m_axis_tready
);

  localparam int ClockPeriodNs = CLK_PERIOD_NS;
  // In L0 and L0s, an UpdateFC for each non-infinite credit type is required
  // at least once every 30 us, tolerance -0%/+50% (PCIe Base Spec r2.1,
  // §2.6.1.2). Periodic beats of one type are at least FcWaitPeriod + 2 cycles
  // apart: the timer is 0 the cycle after the handshake, and the next beat is
  // accepted at the earliest one cycle after the timer saturates. At 8 ns that
  // is 30.016 us, inside the window. pcie_flow_ctrl_init declares its own
  // FcWaitPeriod, with another value and purpose.
  localparam int FcWaitPeriod = 30_000 / ClockPeriodNs;
  localparam int TimerWidth = $clog2(FcWaitPeriod + 1);

  // ST_IDLE picks the next DLLP. ST_SEND_ACK and ST_SEND_ACK_CRC send an Ack
  // or Nak, then ST_WAIT_LOW holds the acknowledge until the request falls.
  // ST_UPDATE_P and ST_UPDATE_NP, each with its CRC state, send one UpdateFC.
  typedef enum logic [4:0] {
    ST_IDLE,
    ST_SEND_ACK,
    ST_SEND_ACK_CRC,
    ST_UPDATE_P,
    ST_UPDATE_CRC,
    ST_UPDATE_NP,
    ST_UPDATE_NP_CRC,
    ST_UPDATE_CPL,
    ST_UPDATE_CPL_CRC,
    ST_WAIT_LOW
  } fc_update_state_e;


  logic             [DATA_WIDTH-1:0] fc_axis_tdata;
  logic             [KEEP_WIDTH-1:0] fc_axis_tkeep;
  logic                              fc_axis_tvalid;
  logic                              fc_axis_tlast;
  logic             [USER_WIDTH-1:0] fc_axis_tuser;
  logic                              fc_axis_tready;
  (* syn_keep = "true", mark_debug = "true" *) fc_update_state_e                  curr_state;
  fc_update_state_e                  next_state;
  dllp_fc_t                          dll_packet_c;
  dllp_fc_t                          dll_packet_r;
  logic             [          15:0] dllp_lcrc_c;
  logic             [          15:0] dllp_lcrc_r;
  logic             [TimerWidth-1:0] timer_p_c, timer_p_r, timer_np_c, timer_np_r;
  logic             [          15:0] crc_out;
  logic             [          15:0] crc_reversed;
  logic                              start_ack_c;
  logic                              start_ack_r;
  logic             [          11:0] ack_nak_seq_c;
  logic             [          11:0] ack_nak_seq_r;
  logic                              ack_nak_is_nak_c;
  logic                              ack_nak_is_nak_r;
  logic             [DATA_WIDTH-1:0] ack_nak_payload;

  // Release-triggered UpdateFC. An UpdateFC is owed for a type whenever
  // dllp2tlp's allocated pair for that type differs from the pair last sent
  // for it (*_pending). dllp2tlp steps that pair when a TLP leaves its receive
  // FIFO, not when the TLP is accepted, so an UpdateFC never advertises buffer
  // space a TLP still occupies. Every release is advertised, which covers the
  // scheduling rules of PCIe Base Spec r2.1, §2.6.1.2 for P and NP. Releases
  // while a DLLP is in flight merge into the next UpdateFC, and nothing is
  // pending at rest. P and NP are owed independently.
  logic             [           7:0] ph_last_c, ph_last_r, nph_last_c, nph_last_r;
  logic             [          11:0] pd_last_c, pd_last_r, npd_last_c, npd_last_r;
  logic                              p_pending, np_pending;

  assign p_pending  = (ph_credits_allocated_i  != ph_last_r)  ||
                      (pd_credits_allocated_i  != pd_last_r);
  assign np_pending = (nph_credits_allocated_i != nph_last_r) ||
                      (npd_credits_allocated_i != npd_last_r);

  // Periodic UpdateFC: one timer per type, restarted only by the handshake of
  // that type's own UpdateFC beat, periodic or release-triggered. Each counts
  // every DL_Active cycle, saturates at FcWaitPeriod and holds at 0 outside
  // DL_Active. A type is owed when it is pending or its timer has expired.
  logic dl_active, p_expired, np_expired, p_owed, np_owed;

  assign dl_active  = (link_status_i == DL_ACTIVE);
  assign p_expired  = (timer_p_r  >= FcWaitPeriod);
  assign np_expired = (timer_np_r >= FcWaitPeriod);
  assign p_owed     = p_pending  || p_expired;
  assign np_owed    = np_pending || np_expired;

  // The CRC is complemented, not bit-reversed: pcie_datalink_crc, a chain of
  // pcie_dllp_crc8 stages, already works in reflected bit order (polynomial
  // D008h, the bit reverse of 100Bh), so a per-byte bit reversal here would
  // reverse the bits a second time.
  always_comb begin : byteswap
    crc_reversed[7:0]  = ~dllp_lcrc_r[7:0];
    crc_reversed[15:8] = ~dllp_lcrc_r[15:8];
  end

  always_ff @(posedge clk_i) begin : main_seq
    if (rst_i) begin
      curr_state <= ST_IDLE;
      dll_packet_r <= '0;
      timer_p_r <= '0;
      timer_np_r <= '0;
      dllp_lcrc_r <= '0;
      start_ack_r <= '0;
      ack_nak_seq_r <= '0;
      ack_nak_is_nak_r <= '0;
      // The values pcie_flow_ctrl_init advertises in InitFC1 and InitFC2, and
      // dllp2tlp's CREDITS_ALLOCATED reset: nothing is owed when FC
      // initialization completes. The three sites must change together.
      ph_last_r <= HdrMinCredits;
      pd_last_r <= PdMinCredits;
      nph_last_r <= HdrMinCredits;
      npd_last_r <= PdMinCredits;
    end else begin
      curr_state <= next_state;
      dll_packet_r <= dll_packet_c;
      timer_p_r <= timer_p_c;
      timer_np_r <= timer_np_c;
      dllp_lcrc_r <= dllp_lcrc_c;
      start_ack_r <= start_ack_c;
      ack_nak_seq_r <= ack_nak_seq_c;
      ack_nak_is_nak_r <= ack_nak_is_nak_c;
      ph_last_r <= ph_last_c;
      pd_last_r <= pd_last_c;
      nph_last_r <= nph_last_c;
      npd_last_r <= npd_last_c;
    end
  end

  // Byte 0 is the DLLP Type, AckNak_Seq_Num fills byte 2 bits 3:0 and byte 3,
  // and the Reserved bits are 0 (PCIe Base Spec r2.1, §3.4.1).
  always_comb begin : ack_nak_payload_pack
    ack_nak_payload        = '0;
    ack_nak_payload[7:0]   = ack_nak_is_nak_r ? Nak : Ack;
    ack_nak_payload[15:8]  = 8'h00;
    ack_nak_payload[19:16] = ack_nak_seq_r[11:8];
    ack_nak_payload[23:20] = 4'h0;
    ack_nak_payload[31:24] = ack_nak_seq_r[7:0];
  end


  always_comb begin : combo_block
    next_state     = curr_state;
    dll_packet_c   = dll_packet_r;
    // both timers advance every DL_Active cycle, saturating; held at 0 outside it
    timer_p_c      = !dl_active ? '0 :
                     (p_expired  ? TimerWidth'(FcWaitPeriod) : timer_p_r  + 1'b1);
    timer_np_c     = !dl_active ? '0 :
                     (np_expired ? TimerWidth'(FcWaitPeriod) : timer_np_r + 1'b1);
    start_ack_c    = '0;
    ack_nak_seq_c = ack_nak_seq_r;
    ack_nak_is_nak_c = ack_nak_is_nak_r;
    ph_last_c      = ph_last_r;
    pd_last_c      = pd_last_r;
    nph_last_c     = nph_last_r;
    npd_last_c     = npd_last_r;
    //axis flow control defaults
    fc_axis_tdata  = '0;
    fc_axis_tkeep  = '0;
    fc_axis_tvalid = '0;
    fc_axis_tlast  = '0;
    fc_axis_tuser  = 4'h01;  // bit 0: a DLLP, which frame_symbols starts with SDP
    //crc signals
    dllp_lcrc_c    = dllp_lcrc_r;
    case (curr_state)
      ST_IDLE: begin
        // An Ack or Nak goes before an owed UpdateFC, the order the
        // Implementation Note in PCIe Base Spec r2.1, §3.5.2.1 recommends. At
        // most one Ack or Nak goes ahead of an owed UpdateFC: dllp2tlp takes no
        // new TLP while the acknowledge is high and registers its request, so
        // the request is low in the cycle ST_WAIT_LOW returns here.
        if (start_flow_control_i) begin
          // Neither UpdateFC timer restarts here: each times its own type.
          next_state       = ST_SEND_ACK;
          ack_nak_seq_c    = next_transmit_seq_i[11:0];
          ack_nak_is_nak_c = tlp_nullified_i;
        end else if (dl_active && (p_owed || np_owed)) begin
          // Only an owed type is sent, P first when both are.
          next_state = p_owed ? ST_UPDATE_P : ST_UPDATE_NP;
        end
      end
      ST_SEND_ACK: begin
        fc_axis_tdata  = ack_nak_payload;
        dllp_lcrc_c    = crc_out;
        fc_axis_tkeep  = '1;
        fc_axis_tvalid = '1;
        if (fc_axis_tready) begin
          next_state = ST_SEND_ACK_CRC;
        end
      end
      ST_SEND_ACK_CRC: begin
        fc_axis_tdata  = crc_reversed;
        fc_axis_tkeep  = 8'h3;
        fc_axis_tvalid = '1;
        fc_axis_tlast  = '1;
        if (fc_axis_tready) begin
          // ACK and NAK complete identically.  Periodic UpdateFC traffic is a
          // separate transaction and must not acknowledge this request.
          start_ack_c = '1;
          next_state  = ST_WAIT_LOW;
        end
      end
      ST_UPDATE_P: begin
        fc_axis_tdata =
            send_fc_init(UpdateFC_P, '0, ph_credits_allocated_i, pd_credits_allocated_i);
        dllp_lcrc_c = crc_out;
        fc_axis_tkeep = '1;
        fc_axis_tvalid = '1;
        if (fc_axis_tready) begin
          // the pair now on the wire is the pair last advertised
          ph_last_c  = ph_credits_allocated_i;
          pd_last_c  = pd_credits_allocated_i;
          timer_p_c  = '0;   // P's own UpdateFC: the only thing that restarts it
          next_state = ST_UPDATE_CRC;
        end
      end
      ST_UPDATE_CRC: begin
        fc_axis_tdata  = crc_reversed;
        fc_axis_tkeep  = 8'h03;
        fc_axis_tvalid = '1;
        fc_axis_tlast  = '1;
        if (fc_axis_tready) begin
          // NP follows only if NP is owed in its own right.  In an idle link
          // the NP timer, restarted two cycles after P's, expires exactly
          // here, so the periodic pair stays a P-then-NP pair.
          next_state = (dl_active && np_owed) ? ST_UPDATE_NP : ST_IDLE;
        end
      end
      ST_UPDATE_NP: begin
        dllp_lcrc_c = crc_out;
        fc_axis_tkeep = '1;
        fc_axis_tvalid = '1;
        fc_axis_tdata = send_fc_init(UpdateFC_NP, '0, nph_credits_allocated_i, npd_credits_allocated_i);
        if (fc_axis_tready) begin
          nph_last_c = nph_credits_allocated_i;
          npd_last_c = npd_credits_allocated_i;
          timer_np_c = '0;   // NP's own UpdateFC: the only thing that restarts it
          next_state = ST_UPDATE_NP_CRC;
        end
      end
      ST_UPDATE_NP_CRC: begin
        fc_axis_tdata  = crc_reversed;
        fc_axis_tkeep  = 8'h03;
        fc_axis_tvalid = '1;
        fc_axis_tlast  = '1;
        if (fc_axis_tready) begin
          next_state = ST_IDLE;
        end
      end
      // Never entered. Completion credits are advertised infinite, so no
      // UpdateFC-Cpl is required (PCIe Base Spec r2.1, §2.6.1).
      ST_UPDATE_CPL: begin
        fc_axis_tdata =
            send_fc_init(UpdateFC_Cpl, '0, '0, '0);
        dllp_lcrc_c = crc_out;
        fc_axis_tkeep = '1;
        fc_axis_tvalid = '1;
        if (fc_axis_tready) begin
          next_state = ST_UPDATE_CPL_CRC;
        end
      end
      ST_UPDATE_CPL_CRC: begin
        fc_axis_tdata  = crc_reversed;
        fc_axis_tkeep  = 8'h03;
        fc_axis_tvalid = '1;
        fc_axis_tlast  = '1;
        if (fc_axis_tready) begin
          next_state = ST_IDLE;
        end
      end
      // The acknowledge holds until dllp2tlp drops its request.
      ST_WAIT_LOW: begin
        start_ack_c = '1;
        if (!start_flow_control_i) begin
          start_ack_c = '0;
          next_state = ST_IDLE;
        end
      end
      default: begin
        next_state  = ST_IDLE;
        start_ack_c = '0;
      end
    endcase
  end

  // Output skid buffer
  axis_register #(
      .DATA_WIDTH(DATA_WIDTH),
      .KEEP_ENABLE('1),
      .KEEP_WIDTH(KEEP_WIDTH),
      .LAST_ENABLE('1),
      .ID_ENABLE('0),
      .ID_WIDTH(1),
      .DEST_ENABLE('0),
      .DEST_WIDTH(1),
      .USER_ENABLE('1),
      .USER_WIDTH(USER_WIDTH),
      .REG_TYPE(SkidBuffer)
  ) axis_register_pipeline_inst (
      .clk(clk_i),
      .rst(rst_i),
      .s_axis_tdata(fc_axis_tdata),
      .s_axis_tkeep(fc_axis_tkeep),
      .s_axis_tvalid(fc_axis_tvalid),
      .s_axis_tready(fc_axis_tready),
      .s_axis_tlast(fc_axis_tlast),
      .s_axis_tuser(fc_axis_tuser),
      .s_axis_tid('0),
      .s_axis_tdest('0),
      .m_axis_tdata(m_axis_tdata),
      .m_axis_tkeep(m_axis_tkeep),
      .m_axis_tvalid(m_axis_tvalid),
      .m_axis_tready(m_axis_tready),
      .m_axis_tlast(m_axis_tlast),
      .m_axis_tuser(m_axis_tuser),
      .m_axis_tid(),
      .m_axis_tdest()
  );

  // DLLP CRC of the 4-byte beat being offered, seeded with FFFFh; each DLLP
  // state registers it into dllp_lcrc_r for the CRC beat that follows.
  pcie_datalink_crc dllp_crc_inst (
      .crcIn ('1),
      .data  (fc_axis_tdata),
      .crcOut(crc_out)
  );

  assign start_flow_control_ack_o = start_ack_r;

endmodule
