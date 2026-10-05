// ---------------------------------------------------------------------------
// os_generator -- sends the LTSSM's Ordered Sets and schedules SKP Ordered Sets
//
// Purpose
//   Sends on every lane the 16-Symbol Ordered Set the LTSSM supplies in
//   ordered_set_i, as four beats of four Symbols, with a per-lane K mask in
//   tuser. Symbol 0 (COM) is K; Symbols 1 and 2 of a TS1 or TS2 are K where
//   that lane's Link or Lane Number is PAD; a Logical Idle block has no K and
//   an EIOS block is all K. While the request stays the same the Ordered Set
//   repeats back to back. A SKP Ordered Set is inserted when skp_cnt reaches
//   SkpIntervalCounts, from ST_IDLE or at the end of an Ordered Set.
//
// Interfaces
//   Request    gen_os_ctrl_i: valid starts an Ordered Set; gen_ts1, gen_ts2,
//              gen_idle and gen_eios select the K mask. ordered_set_i: the
//              Symbols, one Ordered Set per lane. send_ltssm_os_i: high ends
//              the repetition.
//   Done       os_sent_o: one cycle, combinational, as the last beat of each
//              Ordered Set is accepted.
//   SKP timer  link_up_i: skp_cnt counts only while it is high.
//   Stream     m_axis_*: per-lane data and K mask, lane l at
//              [USER_WIDTH*l +: USER_WIDTH] of tuser, through a skid buffer.
//   Unused     curr_data_rate_i, preset_i, CLK_RATE.
//
// Clock and reset
//   clk_i only; phy_transmit connects pipe_rx_usr_clk_i. rst_i is synchronous
//   and active high.
//
// Limitations
//   The SKP Ordered Set is written to lane 0 only: ST_SKP assigns a 32-bit
//   literal to the multi-lane tdata, so lanes 1 and up carry 00h bytes, all
//   marked K.
//
// References
//   PCIe Base Spec r2.1, §4.2.2
//   PCIe Base Spec r2.1, §4.2.4.1
//   PCIe Base Spec r2.1, §4.2.4.2
//   PCIe Base Spec r2.1, §4.2.6.3.2
//   PCIe Base Spec r2.1, §4.2.7.1
// ---------------------------------------------------------------------------
module os_generator
  import pcie_phy_pkg::*;
#(
    parameter int CLK_RATE      = 100,             //! Unused
    parameter int MAX_NUM_LANES = 4,               //! Maximum number of lanes module can support
    parameter int DATA_WIDTH    = 32,              //! AXIS data width
    parameter int KEEP_WIDTH    = DATA_WIDTH / 8,
    parameter int USER_WIDTH    = 4
) (
    // ---- clock and reset ----
    input  logic                                               clk_i,
    input  logic                                               rst_i,
    // ---- Ordered Set request from the LTSSM ----
    input  gen_os_struct_t                                     gen_os_ctrl_i,
    input  rate_speed_e                                        curr_data_rate_i,
    input  logic                                               send_ltssm_os_i,
    output logic                                               os_sent_o,
    // One Ordered Set per lane; lane l's carries its own Lane Number.
    input  pcie_ordered_set_t [             MAX_NUM_LANES-1:0] ordered_set_i,
    input  presets_coeff_t    [             MAX_NUM_LANES-1:0] preset_i,
    input  logic                                               link_up_i,
    //! @virtualbus master_axis_bus @dir out
    output logic              [(DATA_WIDTH*MAX_NUM_LANES)-1:0] m_axis_tdata,
    output logic              [(KEEP_WIDTH*MAX_NUM_LANES)-1:0] m_axis_tkeep,
    output logic                                               m_axis_tvalid,
    output logic                                               m_axis_tlast,
    output logic              [(USER_WIDTH*MAX_NUM_LANES)-1:0] m_axis_tuser,
    input  logic                                               m_axis_tready
    //! @end

);

  // -------------------------------------------------------------------------
  // Ordered Set state machine
  // -------------------------------------------------------------------------
  //   state     action                            exit
  //   ST_IDLE   waits; skp_cnt is tested here     SKP due -> ST_SKP, even with a
  //                                               request; else gen_os_ctrl_i.valid
  //                                               -> ST_BUILD
  //   ST_BUILD  builds the per-lane K masks       -> ST_SEND
  //   ST_SEND   four beats of the Ordered Set     last beat: SKP due -> ST_SKP;
  //                                               same request -> repeat; else ST_IDLE
  //   ST_SKP    one SKP Ordered Set beat          beat accepted -> ST_IDLE
  typedef enum logic [7:0] {
    ST_IDLE,
    ST_BUILD,
    ST_SEND,
    ST_SKP
  } os_gen_state_e;

  // SKP scheduling threshold in clk_i cycles; the schedule is explained at its
  // test in ST_IDLE. ST_IDLE and ST_SEND's last beat both test it.
  localparam logic [31:0] SkpIntervalCounts = 32'h2A6;


  //! internal_axis_signals
  logic [(DATA_WIDTH*MAX_NUM_LANES)-1:0] ltssm_axis_tdata;
  logic [(KEEP_WIDTH*MAX_NUM_LANES)-1:0] ltssm_axis_tkeep;
  logic                                  ltssm_axis_tvalid;
  logic                                  ltssm_axis_tlast;
  logic [(USER_WIDTH*MAX_NUM_LANES)-1:0] ltssm_axis_tuser;
  logic                                  ltssm_axis_tready;


  typedef struct {
    os_gen_state_e                  state;
    logic [7:0]                     axis_pkt_cnt;
    logic [7:0]                     os_pkt_cnt;
    // One K mask per lane, one bit per Symbol. It is per lane because the Link
    // and Lane Numbers, Symbols 1 and 2 of a TS1 or TS2, are PAD (a K Symbol)
    // on some lanes and data on others (PCIe Base Spec r2.1, Table 4-2 and
    // §4.2.6.3.2).
    logic [MAX_NUM_LANES-1:0][(KEEP_WIDTH*8)-1:0] special_k;
    pcie_tsos_t [MAX_NUM_LANES-1:0] ordered_set;
    pcie_tsos_t                     temp_ordered_set;
    logic                           os_sent;
    logic [31:0]                    skp_cnt;
    gen_os_struct_t                 gen_os_ctrl;

  } os_gen_t;

  os_gen_t Q, D;



  //! main sequential block
  always_ff @(posedge clk_i) begin : main_seq
    if (rst_i) begin
      Q <= '{
          state: ST_IDLE,
          ordered_set: pcie_tsos_t'('0),
          temp_ordered_set: pcie_tsos_t'('0),
          gen_os_ctrl: gen_os_struct_t'(00),
          default: 'd0
      };
    end else begin
      Q <= D;
    end
  end

  // From D, so os_sent_o rises in the cycle the last beat is accepted.
  assign os_sent_o = D.os_sent;

  always_comb begin : send_ordered_set
    pcie_tsos_t temp_os;
    D                 = Q;
    ltssm_axis_tdata  = '0;
    ltssm_axis_tkeep  = '0;
    ltssm_axis_tvalid = '0;
    ltssm_axis_tlast  = '0;
    ltssm_axis_tuser  = '0;
    D.os_sent         = '0;
    temp_os           = Q.ordered_set;


    if (link_up_i) begin
      D.skp_cnt = Q.skp_cnt + 1;
    end


    case (Q.state)
      ST_IDLE: begin
        if (gen_os_ctrl_i.valid) begin
          D.skp_cnt = '0;
          for (int i = 0; i < MAX_NUM_LANES; i++) begin
            D.ordered_set[i] = ordered_set_i[i];
          end
          D.temp_ordered_set = ordered_set_i[0];
          D.axis_pkt_cnt     = '0;
          D.gen_os_ctrl      = gen_os_ctrl_i;
          D.state            = ST_BUILD;
        end
        // SKP schedule. skp_cnt restarts at 0 when a SKP is scheduled and then
        // counts every clock while link_up_i is high, so an unobstructed SKP
        // is scheduled every SkpIntervalCounts + 1 = 679 clocks. When
        // pipe_rx_usr_clk_i runs at the rate of pipe_tx_usr_clk_i, on which
        // lane_management sends two Symbols per lane per clock, that is 1358
        // Symbol Times: inside the 1180 to 1538 window (PCIe Base Spec r2.1,
        // §4.2.7.1), 178 above its floor and 180 below its ceiling. A request
        // accepted above also restarts skp_cnt. test_tx_skp measures the
        // interval.
        if (Q.skp_cnt >= SkpIntervalCounts) begin
          D.skp_cnt = '0;
          D.state   = ST_SKP;
        end
      end
      // One COM and three SKP Symbols, all K (PCIe Base Spec r2.1, §4.2.7.1).
      ST_SKP: begin
        if (ltssm_axis_tready) begin
          ltssm_axis_tdata = 32'h1c1c1cbc;
          ltssm_axis_tuser = '1;
          ltssm_axis_tkeep = '1;
          ltssm_axis_tvalid = '1;
          ltssm_axis_tlast = '1;
          D.state = ST_IDLE;
        end
      end
      ST_BUILD: begin
        // Four beats of KEEP_WIDTH = 4 Symbols: 16 Symbols per lane.
        D.os_pkt_cnt   = 32'd3;
        D.special_k    = '0;
        // An Ordered Set goes out on every lane at once (PCIe Base Spec r2.1,
        // §4.2.2), and Symbol 0, the COM, is K on every lane.
        for (int i = 0; i < MAX_NUM_LANES; i++) begin
          D.special_k[i][0] = '1;
        end
        D.axis_pkt_cnt = '0;
        if ((gen_os_ctrl_i.gen_ts1 || gen_os_ctrl_i.gen_ts2)) begin
          for (int i = 0; i < MAX_NUM_LANES; i++) begin
            // Symbol 1, the Link Number, and Symbol 2, the Lane Number, are K
            // exactly where this lane's own value is PAD (K23.7) (PCIe Base
            // Spec r2.1, Table 4-2 and Table 4-3).
            if (Q.ordered_set[i].link_num == PAD_) begin
              D.special_k[i][1] = '1;
            end

            if (Q.ordered_set[i].lane_num == PAD_) begin
              D.special_k[i][2] = '1;
            end

          end

        end

        // Logical Idle is data Symbol 00h only (PCIe Base Spec r2.1, §4.2.2).
        if (gen_os_ctrl_i.gen_idle) begin
          D.special_k = '0;
        end

        // An EIOS is COM and three IDL, all K (PCIe Base Spec r2.1, §4.2.4.2).
        if (gen_os_ctrl_i.gen_eios) begin
          D.special_k = '1;
        end
        D.state = ST_SEND;
      end
      ST_SEND: begin
        if (ltssm_axis_tready) begin
          D.axis_pkt_cnt = Q.axis_pkt_cnt + 1;
          for (int i = 0; i < MAX_NUM_LANES; i++) begin
            ltssm_axis_tdata[32*i+:32] = Q.ordered_set[i][32*Q.axis_pkt_cnt+:32];
          end
          // tuser is packed lane-major like tdata, lane l at [USER_WIDTH*l +:
          // USER_WIDTH]. A beat carries KEEP_WIDTH Symbols, so KEEP_WIDTH bits
          // are written per lane and the rest of each slice, when USER_WIDTH is
          // wider, stays 0.
          ltssm_axis_tuser = '0;
          for (int i = 0; i < MAX_NUM_LANES; i++) begin
            ltssm_axis_tuser[USER_WIDTH*i+:KEEP_WIDTH] =
                Q.special_k[i][KEEP_WIDTH*Q.axis_pkt_cnt+:KEEP_WIDTH];
          end
          ltssm_axis_tkeep  = '1;
          ltssm_axis_tvalid = '1;
          ltssm_axis_tlast  = '0;
          if (Q.axis_pkt_cnt >= Q.os_pkt_cnt) begin
            // A due SKP takes this boundary: a scheduled SKP Ordered Set that
            // finds an Ordered Set in progress goes out at the next Ordered Set
            // boundary (PCIe Base Spec r2.1, §4.2.7.1). The test sits in the
            // last-beat branch, which sets tlast and os_sent whatever it decides,
            // so the Ordered Set in progress always completes first. Without it, a
            // repeating Ordered Set keeps the FSM out of ST_IDLE and no SKP is
            // sent. test_tx_skp checks that no SKP lands inside a TS1.
            if (Q.skp_cnt >= SkpIntervalCounts) begin
              D.skp_cnt = '0;
              D.state   = ST_SKP;
            end
            // While the request is unchanged (same gen_os_ctrl_i, same lane 0
            // Ordered Set, send_ltssm_os_i low), the FSM stays in ST_SEND and
            // the Ordered Set repeats back to back.
            else if (Q.gen_os_ctrl == gen_os_ctrl_i && !send_ltssm_os_i
            && (ordered_set_i[0] == Q.temp_ordered_set)) begin

            end else begin
              D.state = ST_IDLE;
            end
            ltssm_axis_tlast = '1;
            D.os_sent        = '1;
            D.axis_pkt_cnt   = '0;
          end

        end
      end
      default: begin
      end
    endcase

  end


  // Output skid buffer.
  axis_register #(
      .DATA_WIDTH(DATA_WIDTH * MAX_NUM_LANES),
      .KEEP_ENABLE('1),
      .KEEP_WIDTH(KEEP_WIDTH * MAX_NUM_LANES),
      .LAST_ENABLE('1),
      .ID_ENABLE('0),
      .ID_WIDTH(1),
      .DEST_ENABLE('0),
      .DEST_WIDTH(1),
      .USER_ENABLE('1),
      .USER_WIDTH(USER_WIDTH * MAX_NUM_LANES),
      .REG_TYPE(SkidBuffer)
  ) axis_register_inst (
      .clk          (clk_i),
      .rst          (rst_i),
      .s_axis_tdata (ltssm_axis_tdata),
      .s_axis_tkeep (ltssm_axis_tkeep),
      .s_axis_tvalid(ltssm_axis_tvalid),
      .s_axis_tready(ltssm_axis_tready),
      .s_axis_tlast (ltssm_axis_tlast),
      .s_axis_tuser (ltssm_axis_tuser),
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




endmodule
