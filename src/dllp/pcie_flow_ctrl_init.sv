// import pcie_datalink_pkg::*;
module pcie_flow_ctrl_init
  import pcie_datalink_pkg::*;
#(
    // TLP data width
    parameter int DATA_WIDTH = 32,
    // TLP strobe width
    parameter int STRB_WIDTH = DATA_WIDTH / 8,
    parameter int KEEP_WIDTH = STRB_WIDTH,
    parameter int USER_WIDTH = 3,
    parameter int MAX_PAYLOAD_SIZE = 256
) (
    input logic clk_i,                 // Clock signal
    input logic rst_i,                 // Reset signal
    input logic start_flow_control_i,
    input logic fc1_values_stored_i,
    input logic fc2_values_stored_i,
    input logic first_tlp_valid_i,
    input logic idle_valid_i,
    input logic update_fc_i,
    input logic first_feature_exchange_dllp_received_i,

    /*
     * DLLP UPDATE AXI output
     */
    output logic [(DATA_WIDTH)-1:0] m_axis_tdata,
    output logic [(KEEP_WIDTH)-1:0] m_axis_tkeep,
    output logic                    m_axis_tvalid,
    output logic                    m_axis_tlast,
    output logic [  USER_WIDTH-1:0] m_axis_tuser,
    input  logic                    m_axis_tready,
    output logic                    fc2_values_sent_o,
    output logic                    init_ack_o
);


  // localparam int PdMinCredits = MAX_PAYLOAD_SIZE / 4;  //((8 << (5 + MAX_PAYLOAD_SIZE)) / 4);
  //
  // ⚠️ NAME COLLISION, REGISTERED AND NOT FIXED HERE.  dllp_fc_update.sv ALSO
  // declares a localparam named FcWaitPeriod, and its value is TwoMsTimeOut --
  // five orders of magnitude from this one.  Two different FC DLLP transmitters,
  // one identifier.  Say which module you mean, every time.
  localparam int FcWaitPeriod = 8'h2;
  //
  // FC_INIT1 ORIGINATE INTERVAL -- Base 2.1 sec 3.3.1, p. 161:
  //   "The three InitFC1 DLLPs must be transmitted at least once every 34 us."
  // At the 8 ns link clock (125 MHz) that is 34us / 8ns = 4250 cycles.
  //
  // ⚠️ THIS VALUE IS DERIVED FROM THE SPEC, NOT FROM THE PREVIOUS ONE.  It was
  // 8'h0A * 11 = 110 cycles = 880 ns -- an arbitrary figure with no spec basis,
  // and DEAD: the ST_IDLE counter below saturated at it and no state ever read
  // it.  The counter, its saturation and its name were all written; only the
  // consumer was missing.  Restoring the consumer is this commit's whole
  // behaviour change, so the constant is restated at the value sec 3.3.1 gives.
  //
  // ⚠️ The 8 ns assumption is the ONLY thing tying this to a cycle count.  If
  // the link clock changes, this expires: rederive as 34us / clock_period.  The
  // spec bound is an upper limit, so a FASTER repeat stays conformant and a
  // slower one does not.
  localparam int FcInitWaitPeriod = 4250;

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


  //axis registered output signals
  logic                [DATA_WIDTH-1:0] fc_axis_tdata;
  logic                [KEEP_WIDTH-1:0] fc_axis_tkeep;
  logic                                 fc_axis_tvalid;
  logic                                 fc_axis_tlast;
  logic                [USER_WIDTH-1:0] fc_axis_tuser;
  logic                                 fc_axis_tready;

  // Internal state machine for link flow control
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


  always_comb begin : byteswap
    crc_reversed[7:0]  = ~dllp_lcrc_r[7:0];
    crc_reversed[15:8] = ~dllp_lcrc_r[15:8];
    // for (int i = 0; i < 8; i++) begin
    //   crc_reversed[i]   = dllp_lcrc_r[7-i];
    //   crc_reversed[i+8] = dllp_lcrc_r[15-i];
    // end
  end

  // Initialize to idle state
  always_ff @(posedge clk_i) begin : main_seq
    if (rst_i) begin
      curr_state   <= ST_IDLE;
      dll_packet_r <= '0;
      idle_count_r <= '0;
      seq_count_r  <= '0;
      //crc signals
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
      //crc signals
      dllp_lcrc_r  <= dllp_lcrc_c;
    end
  end


  always_comb begin : combo_block
    next_state        = curr_state;
    dll_packet_c      = dll_packet_r;
    seq_count_c       = seq_count_r;
    //axis flow control defaults
    fc_axis_tdata     = '0;
    fc_axis_tkeep     = '0;
    fc_axis_tvalid    = '0;
    fc_axis_tlast     = '0;
    idle_count_c      = idle_count_r;
    fc_axis_tuser     = 4'h01;
    update_fc_c       = update_fc_r;
    //crc signals
    dllp_lcrc_c       = dllp_lcrc_r;
    fc2_count_c       = fc2_count_r;
    //init handshake
    init_ack_o        = '0;
    fc2_values_sent_o = '0;


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
          // ORIGINATE (conformance defect #4).  Base 2.1 sec 3.3.1 p. 161 makes
          // entry to FC_INIT1 a LINK-STATE event -- "Entered when initialization
          // of a VC is required / Entrance to DL_Init state" -- and then requires
          // the DLL to TRANSMIT InitFC1 P/NP/Cpl.  Receiving governs only the
          // EXIT ("Set Flag FI1" / "Exit to FC_INIT2 if Flag FI1 has been set"),
          // never the entry.  Figure 3-3 p. 163 draws one side starting first.
          //
          // Before this commit the only way out of ST_IDLE was a RECEIVED DLLP,
          // so this DLL answered an InitFC1 and never originated one.  Two RTL
          // peers on one link therefore deadlocked, both parked here with
          // start_flow_control_i asserted and zero beats either way -- which is
          // what the RC<->EP bench measured over 20,000 cycles.  Every earlier
          // suite primed the link from Python, and a far end that always speaks
          // first makes responder-only indistinguishable from conformant.
          //
          // start_flow_control_i is the DL_Init signal: pcie_datalink_init drives
          // it from phy_link_up_i.  The timer arm is the originate path; the
          // fc1_values_stored_i arm below it is the pre-existing responder path
          // and is deliberately UNCHANGED -- an incoming InitFC1 is still
          // answered exactly as before, and on a Python-primed bench that arm
          // still wins the race by ~40x, so those benches see no change at all.
          if (fc1_values_stored_i || first_feature_exchange_dllp_received_i
              || (seq_count_r >= FcInitWaitPeriod)) begin
            seq_count_c = '0;
            fc2_count_c = '0;
            //build dllp packet
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
        // seq_count_c = seq_count_r >= FcWaitPeriod ? FcWaitPeriod : seq_count_r + 1'b1;
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
        //we never received an ack restart FC1P
        if (fc_axis_tready) begin
          fc_axis_tdata  = crc_reversed;
          fc_axis_tkeep  = 8'h3;
          fc_axis_tvalid = '1;
          fc_axis_tlast  = '1;
          seq_count_c    = '0;
          next_state     = ST_FC1_CPL;
        end
      end
      //send np
      ST_FC1_CPL: begin
        seq_count_c = (seq_count_r >= FcWaitPeriod) ? FcWaitPeriod : seq_count_r + 1'b1;
        if (fc_axis_tready && (seq_count_r >= FcWaitPeriod)) begin

          //wait for 10us
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
        //we never received an ack restart FC1P
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
            // KEEP ORIGINATING (conformance defect #4, second half).  Base 2.1
            // sec 3.3.1 p. 161 requires the InitFC1 triple "at least once every
            // 34 us" for as long as FC_INIT1 lasts -- the requirement is on the
            // REPEAT, not just the first transmission, so an FSM that sends one
            // triple and then waits is as non-conformant as one that never sends.
            //
            // Before this commit CHECK_FC1 had no else: with FI1 unset it simply
            // stalled here, silently, and nothing was retransmitted.
            //
            // ⚠️ This arm is the one CHECK_FC2 ALREADY HAS -- see its
            // "else if (seq_count_r >= FcWaitPeriod) next_state = ST_FC2".
            // FC_INIT2 has always repeated unconditionally and conformantly;
            // only FC_INIT1 was crippled.  This gives FC1 the shape FC2 has,
            // paced by the same FcWaitPeriod, which is far inside the 34 us
            // bound.  The fc1_values_stored_i arm above is untouched, so the
            // responder path is unchanged: this only covers its ABSENCE.
            seq_count_c = '0;
            next_state  = ST_FC1_P;
          end
        end
      end
      ST_FC2: begin
        seq_count_c = (seq_count_r >= FcWaitPeriod) ? FcWaitPeriod : seq_count_r + 1'b1;
        if (fc_axis_tready && (seq_count_r >= FcWaitPeriod)) begin

          //wait for 10us
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
        //we never received an ack restart FC1P
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

          //wait for 10us
          fc_axis_tdata  = send_fc_init(InitFC2_NP, '0, HdrMinCredits, PdMinCredits);
          // fc_axis_tdata = dll_packet_c;
          fc_axis_tkeep  = '1;
          fc_axis_tvalid = '1;
          fc_axis_tlast  = '0;
          dllp_lcrc_c    = crc_out;
          seq_count_c    = '0;
          next_state     = ST_FC2_NP_CRC;
        end
      end
      ST_FC2_NP_CRC: begin
        //we never received an ack restart FC1P
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
        //wait for 10us
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
        //we never received an ack restart FC1P
        if (fc_axis_tready) begin
          fc_axis_tdata  = crc_reversed;
          fc_axis_tkeep  = 8'h3;
          fc_axis_tvalid = '1;
          fc_axis_tlast  = '1;
          seq_count_c    = '0;
          next_state     = CHECK_FC2;
        end
      end
      CHECK_FC2: begin
        seq_count_c = (seq_count_r >= FcWaitPeriod) ? FcWaitPeriod : seq_count_r + 1'b1;
        if (fc_axis_tready) begin
          fc2_count_c = fc2_count_r + 1;
          if (fc2_values_stored_i && (update_fc_r == '1 || idle_count_r >= 16'h60)) begin
            seq_count_c       = '0;
            fc_axis_tvalid    = '0;
            fc2_values_sent_o = '1;
            // next_state        = ST_FC2;
            // if (first_tlp_valid_i) begin
            next_state        = ST_UPDATE_P;
            // end
            // next_state     = ST_FC_COMPLETE;
          end else if (seq_count_r >= FcWaitPeriod) begin
            seq_count_c = '0;

            next_state  = ST_FC2;
          end
        end
      end
      // ====================================================================
      // HOLD fc2_values_sent_o ACROSS THE FOUR ST_UPDATE_* STATES
      // (conformance defect #3, tracker SS36.2 -- CLOSED HERE)
      // ====================================================================
      // fc2_values_sent_o is a combinational output whose default is '0 (:165).
      // Before this commit it was driven '1 in only two places -- CHECK_FC2's
      // exit arm (:404) and ST_FC_COMPLETE (:464) -- so it fell back to '0
      // across the four states between them.  pcie_datalink_layer.sv:175 has
      //
      //     assign fc_initialized_o = fc2_values_sent && fc2_values_stored;
      //
      // so fc_initialized_o -- the Transaction Layer's "you may send" level --
      // went 1 -> 0 -> 1 while the link was in fact fully initialised, for the
      // width of the first UpdateFC pair.
      //
      // Base 2.1 SS3.2.1 pp. 158-159 and SS3.3.1 pp. 160-162 make completion a
      // one-way event, not a level recomputed each cycle:
      //
      //   p. 158, DL_Init:   "Exit to DL_Active if: Flow Control initialization
      //                       completes successfully, and the Physical Layer
      //                       continues to report Physical LinkUp = 1b"
      //   p. 161, FC_INIT2:  "Signal completion and exit if: Flag FI2 has been
      //                       set"
      //   p. 158, DL_Active: the ONLY exit is "Physical Layer reports Physical
      //                       LinkUp = 0b"
      //
      // The UpdateFC DLLPs these four states emit are ordinary DL_Active credit
      // traffic -- p. 158 lists "Generate and accept DLLPs" as something a
      // COMPLETED link does.  Emitting one while reporting flow control
      // uninitialised is self-contradictory.  Nothing short of link-down may
      // de-assert the level.
      //
      // WINDOW.  Four STATES, not four cycles: each is gated on fc_axis_tready
      // (:425, :436, :448, :459), so with PHY back-pressure the low window is
      // unbounded above.  Four is the floor, measured with tready held high.
      //
      // WHY HERE AND NOT A STICKY BIT.  A register inside this module, or the
      // fc_init_sticky_r filter in pcie_rc_dl_top.sv:254, hides the glitch
      // downstream of a source that still emits it -- and only on the vertical
      // that has the filter.  The RC stack filters; the Endpoint stack does not.
      // Holding the source correct fixes both, and makes the filter redundant
      // rather than wrong (its removal is a CL-2 candidate, not this rung).
      //
      // SCOPE.  The originate path added for conformance defect #4 (ST_IDLE's
      // timer arm and CHECK_FC1's retransmit arm, commit ddd8986) is UNTOUCHED
      // by this change -- no line of it moves.
      ST_UPDATE_P: begin
        //build dllp fc update for crc
        //build axis master output
        fc_axis_tdata = send_fc_init(UpdateFC_P, '0, HdrMinCredits, PdMinCredits);
        dllp_lcrc_c = crc_out;
        fc_axis_tkeep = '1;
        fc_axis_tvalid = '1;
        //FC init is complete from CHECK_FC2's exit onward (SS3.3.1 p. 161)
        fc2_values_sent_o = '1;
        //done with dllp
        if (fc_axis_tready) begin
          next_state = ST_UPDATE_CRC;
        end
      end
      ST_UPDATE_CRC: begin
        //build axis master output
        fc_axis_tdata  = crc_reversed;
        fc_axis_tkeep  = 8'h03;
        fc_axis_tvalid = '1;
        fc_axis_tlast  = '1;
        //FC init is complete from CHECK_FC2's exit onward (SS3.3.1 p. 161)
        fc2_values_sent_o = '1;
        //done with dllp
        if (fc_axis_tready) begin
          next_state = ST_UPDATE_NP;
        end
      end
      ST_UPDATE_NP: begin
        //build axis master output
        dllp_lcrc_c = crc_out;
        fc_axis_tkeep = '1;
        fc_axis_tvalid = '1;
        //build dllp fc update for crc
        fc_axis_tdata = send_fc_init(UpdateFC_NP, '0, HdrMinCredits, PdMinCredits);
        //FC init is complete from CHECK_FC2's exit onward (SS3.3.1 p. 161)
        fc2_values_sent_o = '1;
        //done with dllp
        if (fc_axis_tready) begin
          next_state = ST_UPDATE_NP_CRC;
        end
      end
      ST_UPDATE_NP_CRC: begin
        //build axis master output
        fc_axis_tdata  = crc_reversed;
        fc_axis_tkeep  = 8'h03;
        fc_axis_tvalid = '1;
        fc_axis_tlast  = '1;
        //FC init is complete from CHECK_FC2's exit onward (SS3.3.1 p. 161)
        fc2_values_sent_o = '1;
        //done with dllp
        if (fc_axis_tready) begin
          next_state = ST_FC_COMPLETE;
        end
      end
      ST_FC_COMPLETE: begin
        fc2_values_sent_o = '1;
        //hang around
      end
      default: begin

      end
    endcase
  end

  //axis skid buffer
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
