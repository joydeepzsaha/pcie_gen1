// ---------------------------------------------------------------------------
// pcie_flow_ctrl_init -- Flow Control initialization transmitter for VC0
//
// Purpose
//   Transmits the InitFC1 and InitFC2 DLLP sets for VC0, then one UpdateFC-P
//   and UpdateFC-NP pair, and reports on fc2_values_sent_o that this side has
//   completed FC_INIT2. It starts FC_INIT1 on DL_Init without waiting for the
//   peer, and also answers a peer that transmits first.
//
// Interfaces
//   Control       start_flow_control_i: DL_Init, from pcie_datalink_init.
//                 init_ack_o: one cycle as ST_IDLE exits; pcie_datalink_init
//                 advances on it.
//   Received      fc1_values_stored_i, fc2_values_stored_i: the peer's InitFC1
//                 or InitFC2 values; update_fc_i: an UpdateFC DLLP;
//                 first_tlp_valid_i: a TLP.
//                 first_feature_exchange_dllp_received_i: in ST_IDLE, starts
//                 FC_INIT1 as fc1_values_stored_i does.
//                 idle_valid_i: counted into idle_count_r, which nothing reads.
//   DLLP output   m_axis_*, through a skid buffer. With DATA_WIDTH = 32 each
//                 DLLP is one 4-byte beat followed by one CRC beat (tkeep
//                 0011b, tlast).
//   Status        fc2_values_sent_o: high from FC_INIT2's exit until reset.
//
// Clock and reset
//   clk_i only. rst_i is synchronous and active high; pcie_datalink_layer also
//   resets this module on link down. CLK_PERIOD_NS sets the InitFC1 wait.
//
// Limitations
//   VC0 only. The advertised credits are fixed: HdrMinCredits and PdMinCredits
//   for P and NP, 0 (infinite) for Cpl. MAX_PAYLOAD_SIZE is not used.
//   ST_FC2_P and ST_FC2_P_CRC are declared and never entered; ST_FC2 sends
//   InitFC2-P.
//
// References
//   PCIe Base Spec r2.1, §3.2.1
//   PCIe Base Spec r2.1, §3.3.1
// ---------------------------------------------------------------------------
module pcie_flow_ctrl_init
  import pcie_datalink_pkg::*;
#(
    parameter int DATA_WIDTH = 32,
    parameter int STRB_WIDTH = DATA_WIDTH / 8,
    parameter int KEEP_WIDTH = STRB_WIDTH,
    parameter int USER_WIDTH = 3,
    parameter int MAX_PAYLOAD_SIZE = 256,
    // Link clock period in ns; sets the InitFC1 wait (FcInitWaitPeriod).
    parameter int CLK_PERIOD_NS = 8
) (
    input logic clk_i,
    input logic rst_i,
    input logic start_flow_control_i,
    input logic fc1_values_stored_i,
    input logic fc2_values_stored_i,
    input logic first_tlp_valid_i,
    input logic idle_valid_i,
    input logic update_fc_i,
    input logic first_feature_exchange_dllp_received_i,

    // ---- DLLP output -------------------------------------------------------
    output logic [(DATA_WIDTH)-1:0] m_axis_tdata,
    output logic [(KEEP_WIDTH)-1:0] m_axis_tkeep,
    output logic                    m_axis_tvalid,
    output logic                    m_axis_tlast,
    output logic [  USER_WIDTH-1:0] m_axis_tuser,
    input  logic                    m_axis_tready,
    output logic                    fc2_values_sent_o,
    output logic                    init_ack_o
);


  // Minimum wait, in cycles, before each InitFC DLLP. dllp_fc_update declares
  // its own FcWaitPeriod, with a different value and purpose.
  localparam int FcWaitPeriod = 8'h2;
  // The InitFC1 set must be sent at least once every 34 us (PCIe Base Spec
  // r2.1, §3.3.1); the wait before the first set targets 32 us. FcInitHopCycles
  // is the measured latency from this wait's compare to the first InitFC1-P
  // beat at the DLL output; re-measure it if a stage on that path changes.
  localparam int FcInitTargetNs  = 32_000;
  localparam int FcInitHopCycles = 7;
  localparam int FcInitWaitPeriod = (FcInitTargetNs / CLK_PERIOD_NS) - FcInitHopCycles;

  typedef enum logic [4:0] {
    ST_IDLE,
    ST_FC1_P,
    ST_FC1_CRC,
    ST_FC1_NP,
    ST_FC1_NP_CRC,
    ST_FC1_CPL,
    ST_FC1_CPL_CRC,
    CHECK_FC1,
    ST_FC2,
    ST_FC2_CRC,
    ST_FC2_P,
    ST_FC2_P_CRC,
    ST_FC2_NP,
    ST_FC2_NP_CRC,
    ST_FC2_CPL,
    ST_FC2_CPL_CRC,
    CHECK_FC2,
    ST_UPDATE_P,
    ST_UPDATE_CRC,
    ST_UPDATE_NP,
    ST_UPDATE_NP_CRC,
    ST_FC_COMPLETE
  } flow_control_state_e;


  logic                [DATA_WIDTH-1:0] fc_axis_tdata;
  logic                [KEEP_WIDTH-1:0] fc_axis_tkeep;
  logic                                 fc_axis_tvalid;
  logic                                 fc_axis_tlast;
  logic                [USER_WIDTH-1:0] fc_axis_tuser;
  logic                                 fc_axis_tready;

  (* syn_keep = "true", mark_debug = "true" *)  flow_control_state_e                  curr_state;
  flow_control_state_e                  next_state;
  dllp_fc_t                             dll_packet_c;
  dllp_fc_t                             dll_packet_r;
  logic                [          15:0] dllp_lcrc_c;
  logic                [          15:0] dllp_lcrc_r;
  logic                [          15:0] seq_count_c;
  logic                [          15:0] seq_count_r;
  logic                [          15:0] fc2_count_c;
  logic                [          15:0] fc2_count_r;
  logic                [          15:0] idle_count_c;
  logic                [          15:0] idle_count_r;
  logic                [          15:0] crc_out;
  logic                [          15:0] crc_reversed;
  logic                                 update_fc_c;
  logic                                 update_fc_r;


  // The CRC is complemented, not bit-reversed: pcie_dllp_crc8 already works
  // in reflected bit order (polynomial D008h, the bit reverse of 100Bh), so a
  // per-byte bit reversal here would reverse the bits a second time.
  always_comb begin : byteswap
    crc_reversed[7:0]  = ~dllp_lcrc_r[7:0];
    crc_reversed[15:8] = ~dllp_lcrc_r[15:8];
  end

  always_ff @(posedge clk_i) begin : main_seq
    if (rst_i) begin
      curr_state   <= ST_IDLE;
      dll_packet_r <= '0;
      idle_count_r <= '0;
      seq_count_r  <= '0;
      dllp_lcrc_r  <= '0;
      fc2_count_r  <= '0;
      update_fc_r  <= '0;
    end else begin
      curr_state   <= next_state;
      dll_packet_r <= dll_packet_c;
      idle_count_r <= idle_count_c;
      seq_count_r  <= seq_count_c;
      fc2_count_r  <= fc2_count_c;
      update_fc_r  <= update_fc_c;
      dllp_lcrc_r  <= dllp_lcrc_c;
    end
  end


  always_comb begin : combo_block
    next_state        = curr_state;
    dll_packet_c      = dll_packet_r;
    seq_count_c       = seq_count_r;
    fc_axis_tdata     = '0;
    fc_axis_tkeep     = '0;
    fc_axis_tvalid    = '0;
    fc_axis_tlast     = '0;
    idle_count_c      = idle_count_r;
    fc_axis_tuser     = 4'h01;
    update_fc_c       = update_fc_r;
    dllp_lcrc_c       = dllp_lcrc_r;
    fc2_count_c       = fc2_count_r;
    init_ack_o        = '0;
    fc2_values_sent_o = '0;


    // From FC_INIT2 on, a received UpdateFC is latched; CHECK_FC2's exit
    // condition reads it.
    if (curr_state >= ST_FC2) begin
      if (idle_valid_i) begin
        idle_count_c = idle_count_r + 1'b1;
      end
      if (update_fc_i) begin
        update_fc_c  = '1;
        idle_count_c = '0;
      end
    end
    case (curr_state)
      ST_IDLE: begin
        if (start_flow_control_i && (fc_axis_tready)) begin
          seq_count_c = seq_count_r >= FcInitWaitPeriod ? FcInitWaitPeriod : seq_count_r + 1'b1;
          // FC_INIT1 starts on DL_Init and transmits without waiting for the
          // peer (PCIe Base Spec r2.1, §3.3.1). The timer arm originates; the
          // other two arms answer a peer that transmitted first. Without the
          // timer arm, two instances of this module on one link both wait here.
          if (fc1_values_stored_i || first_feature_exchange_dllp_received_i
              || (seq_count_r >= FcInitWaitPeriod)) begin
            seq_count_c = '0;
            fc2_count_c = '0;
            init_ack_o  = '1;
            next_state  = ST_FC1_P;
          end
        end
      end
      ST_FC1_P: begin
        seq_count_c = seq_count_r >= FcWaitPeriod ? FcWaitPeriod : seq_count_r + 1'b1;
        if (fc_axis_tready && (seq_count_r >= FcWaitPeriod)) begin
          fc_axis_tdata  = send_fc_init(InitFC1_P, '0, HdrMinCredits, PdMinCredits);
          fc_axis_tkeep  = '1;
          fc_axis_tvalid = '1;
          fc_axis_tlast  = '0;
          seq_count_c    = '0;
          dllp_lcrc_c    = crc_out;
          next_state     = ST_FC1_CRC;
        end
      end
      ST_FC1_CRC: begin
        if (fc_axis_tready) begin
          seq_count_c    = '0;
          fc_axis_tdata  = crc_reversed;
          fc_axis_tkeep  = 8'h3;
          fc_axis_tvalid = '1;
          fc_axis_tlast  = '1;
          next_state     = ST_FC1_NP;
        end
      end
      ST_FC1_NP: begin
        seq_count_c = seq_count_r >= FcWaitPeriod ? FcWaitPeriod : seq_count_r + 1'b1;
        if (fc_axis_tready && (seq_count_r >= FcWaitPeriod)) begin
          seq_count_c    = '0;
          fc_axis_tdata  = send_fc_init(InitFC1_NP, '0, HdrMinCredits, PdMinCredits);
          dllp_lcrc_c    = crc_out;
          fc_axis_tkeep  = '1;
          fc_axis_tvalid = '1;
          fc_axis_tlast  = '0;
          next_state     = ST_FC1_NP_CRC;
        end
      end
      ST_FC1_NP_CRC: begin
        if (fc_axis_tready) begin
          fc_axis_tdata  = crc_reversed;
          fc_axis_tkeep  = 8'h3;
          fc_axis_tvalid = '1;
          fc_axis_tlast  = '1;
          seq_count_c    = '0;
          next_state     = ST_FC1_CPL;
        end
      end
      ST_FC1_CPL: begin
        seq_count_c = (seq_count_r >= FcWaitPeriod) ? FcWaitPeriod : seq_count_r + 1'b1;
        if (fc_axis_tready && (seq_count_r >= FcWaitPeriod)) begin

          fc_axis_tdata  = send_fc_init(InitFC1_Cpl, '0, '0, '0);
          dllp_lcrc_c    = crc_out;
          fc_axis_tkeep  = '1;
          fc_axis_tvalid = '1;
          fc_axis_tlast  = '0;
          seq_count_c    = '0;
          next_state     = ST_FC1_CPL_CRC;
        end
      end
      ST_FC1_CPL_CRC: begin
        if (fc_axis_tready) begin
          fc_axis_tdata  = crc_reversed;
          fc_axis_tkeep  = 8'h3;
          fc_axis_tvalid = '1;
          fc_axis_tlast  = '1;
          seq_count_c    = '0;
          next_state     = CHECK_FC1;
        end
      end
      CHECK_FC1: begin
        seq_count_c = (seq_count_r >= FcWaitPeriod) ? FcWaitPeriod : seq_count_r + 1'b1;
        if (fc_axis_tready && (seq_count_r >= FcWaitPeriod)) begin
          if (fc1_values_stored_i) begin
            seq_count_c = '0;
            fc2_count_c = fc2_count_r + 1;
            if (fc2_count_r >= 8'd5) begin
              fc2_count_c  = '0;
              idle_count_c = '0;
              next_state   = ST_FC2;
            end else begin
              next_state = ST_FC1_P;
            end
          end else begin
            // Until FI1 is set the InitFC1 set repeats, paced by FcWaitPeriod.
            // The repeat is required at least every 34 us for as long as
            // FC_INIT1 lasts (PCIe Base Spec r2.1, §3.3.1).
            seq_count_c = '0;
            next_state  = ST_FC1_P;
          end
        end
      end
      ST_FC2: begin
        seq_count_c = (seq_count_r >= FcWaitPeriod) ? FcWaitPeriod : seq_count_r + 1'b1;
        if (fc_axis_tready && (seq_count_r >= FcWaitPeriod)) begin

          if (seq_count_r >= FcWaitPeriod) begin
            fc_axis_tdata  = send_fc_init(InitFC2_P, '0, HdrMinCredits, PdMinCredits);
            fc_axis_tkeep  = '1;
            fc_axis_tvalid = '1;
            fc_axis_tlast  = '0;
            dllp_lcrc_c    = crc_out;
            seq_count_c    = '0;
            next_state     = ST_FC2_CRC;
          end
        end
      end
      ST_FC2_CRC: begin
        if (fc_axis_tready) begin
          fc_axis_tdata  = crc_reversed;
          fc_axis_tkeep  = 8'h3;
          fc_axis_tvalid = '1;
          fc_axis_tlast  = '1;
          seq_count_c    = '0;
          next_state     = ST_FC2_NP;
        end
      end
      ST_FC2_NP: begin
        seq_count_c = (seq_count_r >= FcWaitPeriod) ? FcWaitPeriod : seq_count_r + 1'b1;
        if (fc_axis_tready && (seq_count_r >= FcWaitPeriod)) begin

          fc_axis_tdata  = send_fc_init(InitFC2_NP, '0, HdrMinCredits, PdMinCredits);
          fc_axis_tkeep  = '1;
          fc_axis_tvalid = '1;
          fc_axis_tlast  = '0;
          dllp_lcrc_c    = crc_out;
          seq_count_c    = '0;
          next_state     = ST_FC2_NP_CRC;
        end
      end
      ST_FC2_NP_CRC: begin
        if (fc_axis_tready) begin
          fc_axis_tdata  = crc_reversed;
          fc_axis_tkeep  = 8'h3;
          fc_axis_tvalid = '1;
          fc_axis_tlast  = '1;
          seq_count_c    = '0;
          next_state     = ST_FC2_CPL;
        end
      end
      ST_FC2_CPL: begin
        seq_count_c = (seq_count_r >= FcWaitPeriod) ? FcWaitPeriod : seq_count_r + 1'b1;
        if (fc_axis_tready && (seq_count_r >= FcWaitPeriod)) begin

          fc_axis_tdata  = send_fc_init(InitFC2_Cpl, '0, '0, '0);
          dllp_lcrc_c    = crc_out;
          fc_axis_tkeep  = '1;
          fc_axis_tvalid = '1;
          fc_axis_tlast  = '0;
          seq_count_c    = '0;
          next_state     = ST_FC2_CPL_CRC;
        end
      end
      ST_FC2_CPL_CRC: begin
        if (fc_axis_tready) begin
          fc_axis_tdata  = crc_reversed;
          fc_axis_tkeep  = 8'h3;
          fc_axis_tvalid = '1;
          fc_axis_tlast  = '1;
          seq_count_c    = '0;
          next_state     = CHECK_FC2;
        end
      end
      // Entered only from ST_FC2_CPL_CRC, at the end of a chain from ST_FC2 in
      // which every state has one successor, so each arrival follows a full
      // InitFC2 set: the transmit half of FC_INIT2's exit condition holds here.
      CHECK_FC2: begin
        seq_count_c = (seq_count_r >= FcWaitPeriod) ? FcWaitPeriod : seq_count_r + 1'b1;
        if (fc_axis_tready) begin
          fc2_count_c = fc2_count_r + 1;
          // The receive half: an InitFC2 (fc2_values_stored_i), an UpdateFC
          // (update_fc_r) or a TLP (first_tlp_valid_i) has been received (PCIe
          // Base Spec r2.1, §3.3.1). The rule has no idle-time condition.
          if (fc2_values_stored_i || update_fc_r || first_tlp_valid_i) begin
            seq_count_c       = '0;
            fc_axis_tvalid    = '0;
            fc2_values_sent_o = '1;
            next_state        = ST_UPDATE_P;
          end else if (seq_count_r >= FcWaitPeriod) begin
            // Otherwise the InitFC2 set repeats, paced by FcWaitPeriod; it too
            // is required at least every 34 us (PCIe Base Spec r2.1, §3.3.1).
            seq_count_c = '0;

            next_state  = ST_FC2;
          end
        end
      end
      // One UpdateFC-P / UpdateFC-NP pair with the same credits as the InitFC
      // sets; dllp_fc_update sends every later UpdateFC.
      ST_UPDATE_P: begin
        fc_axis_tdata = send_fc_init(UpdateFC_P, '0, HdrMinCredits, PdMinCredits);
        dllp_lcrc_c = crc_out;
        fc_axis_tkeep = '1;
        fc_axis_tvalid = '1;
        // Held from FC_INIT2's exit until reset: FC initialization completes
        // once, and only Physical LinkUp = 0b ends DL_Active (PCIe Base Spec
        // r2.1, §3.2.1). The Endpoint stack uses pcie_datalink_layer's
        // fc_initialized_o unfiltered, so the level must be right at its source.
        fc2_values_sent_o = '1;
        if (fc_axis_tready) begin
          next_state = ST_UPDATE_CRC;
        end
      end
      ST_UPDATE_CRC: begin
        fc_axis_tdata  = crc_reversed;
        fc_axis_tkeep  = 8'h03;
        fc_axis_tvalid = '1;
        fc_axis_tlast  = '1;
        fc2_values_sent_o = '1;
        if (fc_axis_tready) begin
          next_state = ST_UPDATE_NP;
        end
      end
      ST_UPDATE_NP: begin
        dllp_lcrc_c = crc_out;
        fc_axis_tkeep = '1;
        fc_axis_tvalid = '1;
        fc_axis_tdata = send_fc_init(UpdateFC_NP, '0, HdrMinCredits, PdMinCredits);
        fc2_values_sent_o = '1;
        if (fc_axis_tready) begin
          next_state = ST_UPDATE_NP_CRC;
        end
      end
      ST_UPDATE_NP_CRC: begin
        fc_axis_tdata  = crc_reversed;
        fc_axis_tkeep  = 8'h03;
        fc_axis_tvalid = '1;
        fc_axis_tlast  = '1;
        fc2_values_sent_o = '1;
        if (fc_axis_tready) begin
          next_state = ST_FC_COMPLETE;
        end
      end
      ST_FC_COMPLETE: begin
        fc2_values_sent_o = '1;
      end
      default: begin

      end
    endcase
  end

  axis_register #(
      .DATA_WIDTH (DATA_WIDTH),
      .KEEP_ENABLE('1),
      .KEEP_WIDTH (KEEP_WIDTH),
      .LAST_ENABLE('1),
      .ID_ENABLE  ('0),
      .ID_WIDTH   (1),
      .DEST_ENABLE('0),
      .DEST_WIDTH (1),
      .USER_ENABLE('1),
      .USER_WIDTH (USER_WIDTH),
      .REG_TYPE   (SkidBuffer)
  ) axis_register_pipeline_inst (
      .clk          (clk_i),
      .rst          (rst_i),
      .s_axis_tdata (fc_axis_tdata),
      .s_axis_tkeep (fc_axis_tkeep),
      .s_axis_tvalid(fc_axis_tvalid),
      .s_axis_tready(fc_axis_tready),
      .s_axis_tlast (fc_axis_tlast),
      .s_axis_tuser (fc_axis_tuser),
      .s_axis_tid   ('0),
      .s_axis_tdest ('0),
      .m_axis_tdata (m_axis_tdata),
      .m_axis_tkeep (m_axis_tkeep),
      .m_axis_tvalid(m_axis_tvalid),
      .m_axis_tready(m_axis_tready),
      .m_axis_tlast (m_axis_tlast),
      .m_axis_tuser (m_axis_tuser),
      .m_axis_tid   (),
      .m_axis_tdest ()
  );

  pcie_datalink_crc dllp_crc_inst (
      .crcIn ('1),
      .data  (fc_axis_tdata),
      .crcOut(crc_out)
  );


endmodule
