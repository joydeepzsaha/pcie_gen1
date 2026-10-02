// ---------------------------------------------------------------------------
//! @title dllp_handler
//! @author Idris Somoye
//! Checks the CRC of each received DLLP and decodes the DLLPs that pass.
//
// Purpose
//   Each DLLP frame from axis_user_demux is two beats: the four DLLP bytes
//   (tkeep all ones, no tlast), then the two CRC bytes (tkeep 0011b, tlast).
//   The first beat is stored with its CRC from pcie_datalink_crc; the second
//   must equal that CRC complemented. ST_PROCESS_DLLP then decodes the DLLP:
//   Ack and Nak report their AckNak_Seq_Num, InitFC1, InitFC2 and UpdateFC
//   store the peer's credits, and Feature_Exchange sets a flag. Nothing is
//   transmitted from here.
//
// Interfaces
//   Control   phy_link_up_i: DLLPs are accepted while it is high, whatever
//             the DL state, so InitFC DLLPs are taken during DL_Init.
//   Input     s_axis_*: DLLP frames, tuser bit 0 (UserIsDllp) set.
//   Ack/Nak   seq_num_o, seq_num_vld_o, seq_num_acknack_o: one cycle per
//             accepted Ack (seq_num_acknack_o = 1) or Nak (0).
//   FC init   fc1_values_stored_o, fc2_values_stored_o: InitFC1 (InitFC2)
//             for P, NP and Cpl have all been accepted; held until reset.
//   Credits   tx_fc_*_o: HdrFC and DataFC of the last InitFC1, InitFC2 or
//             UpdateFC of each type, the peer's limits for tlp2dllp.
//             update_fc_o: one cycle, after each accepted UpdateFC.
//   Feature   first_feature_exchange_dllp_received_o: held from the first
//             accepted Feature_Exchange DLLP until reset.
//
// Clock and reset
//   clk_i only. rst_i is synchronous and active high; pcie_datalink_layer
//   also asserts it while the link is down.
//
// Limitations
//   VC0 only: InitFC and UpdateFC labels match the whole type byte. An Ack,
//   Nak, InitFC or UpdateFC DLLP with a non-zero Reserved field is dropped,
//   although Receivers must ignore Reserved values (PCIe Base Spec r2.1,
//   §3.4.1 and §3.5.2.2). A DLLP that fails the CRC check is dropped with no
//   error output. PM and Vendor Specific DLLPs are dropped. Feature_Exchange
//   (0000 0010b) is a Reserved DLLP Type encoding in PCIe Base Spec r2.1,
//   §3.4.1. ST_DLL_RX_DATA and ST_TLP_EOP are never entered.
//
// References
//   PCIe Base Spec r2.1, §3.4.1
//   PCIe Base Spec r2.1, §3.5.2.2
// ---------------------------------------------------------------------------
module dllp_handler
  import pcie_datalink_pkg::*;
#(
    parameter int DATA_WIDTH = 32,
    parameter int STRB_WIDTH = DATA_WIDTH / 8,
    parameter int KEEP_WIDTH = STRB_WIDTH,
    parameter int USER_WIDTH = 4
) (
    input  logic                  clk_i,
    input  logic                  rst_i,
    input  logic                  phy_link_up_i,

    // ---- DLLP frames -------------------------------------------------------
    input  logic [DATA_WIDTH-1:0] s_axis_tdata,
    input  logic [KEEP_WIDTH-1:0] s_axis_tkeep,
    input  logic                  s_axis_tvalid,
    input  logic                  s_axis_tlast,
    input  logic [USER_WIDTH-1:0] s_axis_tuser,
    output logic                  s_axis_tready,

    // ---- received Ack and Nak ----------------------------------------------
    output logic [          11:0] seq_num_o,
    output logic                  seq_num_vld_o,
    output logic                  seq_num_acknack_o,

    // ---- received flow control ---------------------------------------------
    output logic                  fc1_values_stored_o,
    output logic                  fc2_values_stored_o,
    output logic [           7:0] tx_fc_ph_o,
    output logic [          11:0] tx_fc_pd_o,
    output logic [           7:0] tx_fc_nph_o,
    output logic [          11:0] tx_fc_npd_o,
    output logic [           7:0] tx_fc_cplh_o,
    output logic [          11:0] tx_fc_cpld_o,
    output logic                  update_fc_o,
    output logic                  first_feature_exchange_dllp_received_o
);

  localparam int UserIsDllp = 0;

  // ST_IDLE stores the first beat, ST_CHECK_CRC compares the CRC beat, and
  // ST_PROCESS_DLLP decodes for one cycle with ready low.
  typedef enum logic [2:0] {
    ST_IDLE,
    ST_CHECK_CRC,
    ST_PROCESS_DLLP,
    ST_DLL_RX_DATA,
    ST_TLP_EOP
  } dll_rx_st_e;

  (* syn_keep = "true", mark_debug = "true" *) dll_rx_st_e                   curr_state;
  dll_rx_st_e                   next_state;
  dllp_union_t                  dll_packet_c;
  dllp_union_t                  dll_packet_r;
  //crc helper signals
  logic        [          15:0] crc_in_c;
  logic        [          15:0] crc_in_r;
  logic        [          15:0] crc_out;
  //tlp nulled
  logic                         tlp_nullified_c;
  logic                         tlp_nullified_r;
  logic                         tx_tlp_ready_c;
  logic                         tx_tlp_ready_r;
  //transmit sequence logic
  logic        [          15:0] next_transmit_seq_c;
  logic        [          15:0] next_transmit_seq_r;
  logic        [          11:0] ackd_transmit_seq_c;
  logic        [          11:0] ackd_transmit_seq_r;
  //s axis skid buffer
  
  (* syn_keep = "true", mark_debug = "true" *) logic        [DATA_WIDTH-1:0] skid_s_axis_tdata;
  logic        [KEEP_WIDTH-1:0] skid_s_axis_tkeep;
  
  (* syn_keep = "true", mark_debug = "true" *)logic                         skid_s_axis_tvalid;
  logic                         skid_s_axis_tlast;
  
  logic        [USER_WIDTH-1:0] skid_s_axis_tuser;
  logic                         skid_s_axis_tready;
  //Flow control
  logic        [           7:0] tx_fc_ph_c;
  logic        [           7:0] tx_fc_ph_r;
  logic        [          11:0] tx_fc_pd_c;
  logic        [          11:0] tx_fc_pd_r;
  logic        [           7:0] tx_fc_nph_c;
  logic        [           7:0] tx_fc_nph_r;
  logic        [          11:0] tx_fc_npd_c;
  logic        [          11:0] tx_fc_npd_r;
  logic        [           7:0] tx_fc_cplh_c;
  logic        [           7:0] tx_fc_cplh_r;
  logic        [          11:0] tx_fc_cpld_c;
  logic        [          11:0] tx_fc_cpld_r;
  logic                         update_fc_c;
  logic                         update_fc_r;
  //fc1 vals
  logic                         fc1_np_stored_c;
  logic                         fc1_np_stored_r;
  logic                         fc1_p_stored_c;
  logic                         fc1_p_stored_r;
  logic                         fc1_c_stored_c;
  logic                         fc1_c_stored_r;
  //fc2 vals
  logic                         fc2_np_stored_c;
  logic                         fc2_np_stored_r;
  logic                         fc2_p_stored_c;
  logic                         fc2_p_stored_r;
  logic                         fc2_c_stored_c;
  logic                         fc2_c_stored_r;
  (* syn_keep = "true", mark_debug = "true" *) logic                [          15:0] crc_reversed;
  logic                         first_feature_exchange_dllp_received_r;
  logic                         first_feature_exchange_dllp_received_c;
  logic                         dllp_first_word_valid;
  logic                         dllp_crc_word_valid;
  logic                         ack_nack_fields_valid;
  logic                         fc_fields_valid;

  // debug
  (* syn_keep = "true", mark_debug = "true" *)logic [15:0] dbg_lower_skid_data_dllp;
  
  assign fc1_values_stored_o = fc1_np_stored_r & fc1_p_stored_r & fc1_c_stored_r;
  assign fc2_values_stored_o = fc2_np_stored_r & fc2_p_stored_r & fc2_c_stored_r;
  assign dbg_lower_skid_data_dllp = skid_s_axis_tdata[15:0];
  assign dllp_first_word_valid = (skid_s_axis_tkeep == {KEEP_WIDTH{1'b1}}) &&
                                 !skid_s_axis_tlast;
  assign dllp_crc_word_valid = skid_s_axis_tlast &&
                               (skid_s_axis_tkeep == {{(KEEP_WIDTH - 2){1'b0}}, 2'b11});
  assign ack_nack_fields_valid = (dll_packet_r.ack_nack.rsvd0 == '0) &&
                                 (dll_packet_r.ack_nack.rsvd1 == '0);
  assign fc_fields_valid = (dll_packet_r.flow_control.byte1.rsvd0 == '0) &&
                           (dll_packet_r.flow_control.byte2.rsvd1 == '0);


  // The CRC is complemented, not bit-reversed: pcie_datalink_crc, a chain of
  // pcie_dllp_crc8 stages, already works in reflected bit order (polynomial
  // D008h, the bit reverse of 100Bh), so the complement is the CRC field as
  // received, its first byte in bits 7:0. A per-byte bit reversal here would
  // reverse the bits a second time.
  always_comb begin : byteswap
    crc_reversed = ~crc_in_r;
  end

  always @(posedge clk_i) begin : main_seq
    if (rst_i) begin
      curr_state          <= ST_IDLE;
      next_transmit_seq_r <= '0;
      dll_packet_r        <= '0;
      //crc signals
      crc_in_r            <= '0;
      //flow control
      tx_fc_ph_r          <= '0;
      tx_fc_pd_r          <= '0;
      tx_fc_nph_r         <= '0;
      tx_fc_npd_r         <= '0;
      tx_fc_cplh_r        <= '0;
      tx_fc_cpld_r        <= '0;
      first_feature_exchange_dllp_received_r <= '0;
      //capture signals
      fc1_np_stored_r     <= '0;
      fc1_p_stored_r      <= '0;
      fc1_c_stored_r      <= '0;
      fc2_np_stored_r     <= '0;
      fc2_p_stored_r      <= '0;
      fc2_c_stored_r      <= '0;
      update_fc_r         <= '0;
    end else begin
      curr_state          <= next_state;
      next_transmit_seq_r <= next_transmit_seq_c;
      dll_packet_r        <= dll_packet_c;
      //crc signals
      crc_in_r            <= crc_in_c;
      //flow control
      tx_fc_ph_r          <= tx_fc_ph_c;
      tx_fc_pd_r          <= tx_fc_pd_c;
      tx_fc_nph_r         <= tx_fc_nph_c;
      tx_fc_npd_r         <= tx_fc_npd_c;
      tx_fc_cplh_r        <= tx_fc_cplh_c;
      tx_fc_cpld_r        <= tx_fc_cpld_c;
      first_feature_exchange_dllp_received_r <= first_feature_exchange_dllp_received_c;
      //capture signals
      fc1_np_stored_r     <= fc1_np_stored_c;
      fc1_p_stored_r      <= fc1_p_stored_c;
      fc1_c_stored_r      <= fc1_c_stored_c;
      fc2_np_stored_r     <= fc2_np_stored_c;
      fc2_p_stored_r      <= fc2_p_stored_c;
      fc2_c_stored_r      <= fc2_c_stored_c;
      update_fc_r         <= update_fc_c;
    end
  end


  always_comb begin : main_combo
    next_state          = curr_state;
    next_transmit_seq_c = next_transmit_seq_r;
    dll_packet_c        = dll_packet_r;
    skid_s_axis_tready  = '0;
    //crc signals
    crc_in_c            = crc_in_r;
    //ack_nack signals
    seq_num_o           = '0;
    seq_num_vld_o       = '0;
    seq_num_acknack_o   = '0;
    //flow control
    tx_fc_ph_c          = tx_fc_ph_r;
    tx_fc_pd_c          = tx_fc_pd_r;
    tx_fc_nph_c         = tx_fc_nph_r;
    tx_fc_npd_c         = tx_fc_npd_r;
    tx_fc_cplh_c        = tx_fc_cplh_r;
    tx_fc_cpld_c        = tx_fc_cpld_r;
    update_fc_c         = '0;
    first_feature_exchange_dllp_received_c = first_feature_exchange_dllp_received_r;

    //capture signals
    fc1_np_stored_c     = fc1_np_stored_r;
    fc1_p_stored_c      = fc1_p_stored_r;
    fc1_c_stored_c      = fc1_c_stored_r;
    fc2_np_stored_c     = fc2_np_stored_r;
    fc2_p_stored_c      = fc2_p_stored_r;
    fc2_c_stored_c      = fc2_c_stored_r;
    case (curr_state)
      ST_IDLE: begin
        if (phy_link_up_i) begin
          skid_s_axis_tready = '1;
          if (skid_s_axis_tvalid && skid_s_axis_tuser[UserIsDllp] &&
              dllp_first_word_valid) begin
            dll_packet_c = skid_s_axis_tdata;
            crc_in_c     = crc_out;
            next_state   = ST_CHECK_CRC;
          end
        end
      end
      ST_CHECK_CRC: begin
        skid_s_axis_tready = '1;
        if (skid_s_axis_tvalid && skid_s_axis_tuser[UserIsDllp]) begin
          if (dllp_crc_word_valid && (crc_reversed == skid_s_axis_tdata[15:0])) begin
            next_state = ST_PROCESS_DLLP;
          end
          else begin
            next_state = ST_IDLE;
          end
        end
      end
      ST_PROCESS_DLLP: begin
        // Ready stays low for this cycle while the stored DLLP is decoded.
        casez (dll_packet_r.generic.dllp_type)
          Ack: begin
            if (ack_nack_fields_valid) begin
              seq_num_o         = get_ack_nack_seq(dll_packet_r.ack_nack);
              seq_num_vld_o     = '1;
              seq_num_acknack_o = '1;
            end
          end
          Nak: begin
            if (ack_nack_fields_valid) begin
              seq_num_o     = get_ack_nack_seq(dll_packet_r.ack_nack);
              seq_num_vld_o = '1;
            end
          end
          // pcie_flow_ctrl_init starts FC_INIT1 on this flag as it does on a
          // received InitFC1 set.
          Feature_Exchange: begin
            first_feature_exchange_dllp_received_c = '1;
          end
          PM_Enter_L1: begin
            //not implemented
          end
          PM_Enter_L23: begin
            //not implemented
          end
          PM_Actv_St_Req_L1: begin
            //not implemented
          end
          PM_Request_Ack: begin
            //not implemented
          end
          Vendor_Specific: begin
            //not implemented
          end
          InitFC1_P: begin
            if (fc_fields_valid) begin
              get_fc_values(tx_fc_ph_c, tx_fc_pd_c, dll_packet_r.flow_control);
              fc1_p_stored_c = '1;
            end
          end
          InitFC1_NP: begin
            if (fc_fields_valid) begin
              get_fc_values(tx_fc_nph_c, tx_fc_npd_c, dll_packet_r.flow_control);
              fc1_np_stored_c = '1;
            end
          end
          InitFC1_Cpl: begin
            if (fc_fields_valid) begin
              get_fc_values(tx_fc_cplh_c, tx_fc_cpld_c, dll_packet_r.flow_control);
              fc1_c_stored_c = '1;
            end
          end
          InitFC2_P: begin
            if (fc_fields_valid) begin
              get_fc_values(tx_fc_ph_c, tx_fc_pd_c, dll_packet_r.flow_control);
              fc2_p_stored_c = '1;
            end
          end
          InitFC2_NP: begin
            if (fc_fields_valid) begin
              get_fc_values(tx_fc_nph_c, tx_fc_npd_c, dll_packet_r.flow_control);
              fc2_np_stored_c = '1;
            end
          end
          InitFC2_Cpl: begin
            if (fc_fields_valid) begin
              get_fc_values(tx_fc_cplh_c, tx_fc_cpld_c, dll_packet_r.flow_control);
              fc2_c_stored_c = '1;
            end
          end
          UpdateFC_P: begin
            if (fc_fields_valid) begin
              get_fc_values(tx_fc_ph_c, tx_fc_pd_c, dll_packet_r.flow_control);
              update_fc_c = '1;
            end
          end
          UpdateFC_NP: begin
            if (fc_fields_valid) begin
              get_fc_values(tx_fc_nph_c, tx_fc_npd_c, dll_packet_r.flow_control);
              update_fc_c = '1;
            end
          end
          UpdateFC_Cpl: begin
            if (fc_fields_valid) begin
              get_fc_values(tx_fc_cplh_c, tx_fc_cpld_c, dll_packet_r.flow_control);
              update_fc_c = '1;
            end
          end
          default: begin
          end
        endcase
        next_state = ST_IDLE;
      end
      default: begin
      end
    endcase
  end

  // CRC of the beat at the skid buffer output, seeded with FFFFh; ST_IDLE
  // registers it with the first beat of a DLLP.
  pcie_datalink_crc pcie_datalink_crc_inst (
      .crcIn (16'hFFFF),
      .data  (skid_s_axis_tdata),
      .crcOut(crc_out)
  );

  // Input skid buffer
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
  ) axis_register_inst (
      .clk(clk_i),
      .rst(rst_i),
      .s_axis_tdata(s_axis_tdata),
      .s_axis_tkeep(s_axis_tkeep),
      .s_axis_tvalid(s_axis_tvalid),
      .s_axis_tready(s_axis_tready),
      .s_axis_tlast(s_axis_tlast),
      .s_axis_tid('0),
      .s_axis_tdest('0),
      .s_axis_tuser(s_axis_tuser),
      .m_axis_tdata(skid_s_axis_tdata),
      .m_axis_tkeep(skid_s_axis_tkeep),
      .m_axis_tvalid(skid_s_axis_tvalid),
      .m_axis_tready(skid_s_axis_tready),
      .m_axis_tlast(skid_s_axis_tlast),
      .m_axis_tid(),
      .m_axis_tdest(),
      .m_axis_tuser(skid_s_axis_tuser)
  );

  assign tx_fc_ph_o   = tx_fc_ph_r;
  assign tx_fc_pd_o   = tx_fc_pd_r;
  assign tx_fc_nph_o  = tx_fc_nph_r;
  assign tx_fc_npd_o  = tx_fc_npd_r;
  assign tx_fc_cplh_o = tx_fc_cplh_r;
  assign tx_fc_cpld_o = tx_fc_cpld_r;
  assign update_fc_o  = update_fc_r;
  assign first_feature_exchange_dllp_received_o = first_feature_exchange_dllp_received_r;

endmodule
