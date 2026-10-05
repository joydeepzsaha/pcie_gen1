// ---------------------------------------------------------------------------
// pcie_phy_pkg -- Physical Layer Symbol codes, Ordered Set types and builders
//
// Purpose
//   Imported by the logical PHY (pcie_phy_top, phy_receive, phy_transmit
//   and their submodules, the scrambler among them), pcie_ltssm_downstream
//   and pcie_endpoint_top. An Ordered Set is held as 16 Symbols with
//   Symbol 0 in bits 7:0. The 8 GT/s (Gen3) definitions here have no
//   counterpart in PCIe Base Spec r2.1, which defines 2.5 and 5.0 GT/s
//   only.
//
// Contents
//   Symbol codes      the twelve K codes, Training Sequence values.
//   8 GT/s TS fields  Symbol 6-9 and equalisation layouts for the LTSSM.
//   Special Symbols   COM, STP, SDP, END, EDB, PAD, SKP, FTS, IDL, EIE; the
//                     8 GT/s tokens, STP layout, scrambler seeds, GEN3_SDS.
//   TS layout         Training Control, Data Rate Identifier, rate_speed_e,
//                     pcie_tsos_t, gen_os_struct_t, pcie_ordered_set_t.
//   Functions         Ordered Set builders (gen_ts_os, gen_zeros, gen_idle,
//                     gen_eios, gen_eieos) and 8 GT/s framing helpers.
//   Not used          phy_special_k_e, rx_tx_presets_e, gen_3_stp_byte3_t,
//                     data_t, GEN3_SDS, reset_lane_equal_ctrl_reg,
//                     gen_eq_tsos, get_tlp_len, gen_idle, gen_sds_os, gen_skp.
//
// References
//   PCIe Base Spec r2.1, §4.2.1.2
//   PCIe Base Spec r2.1, §4.2.4.1
//   PCIe Base Spec r2.1, §4.2.4.2
//   PCIe Base Spec r2.1, §4.2.4.4
//   PCIe Base Spec r2.1, §4.2.4.5
//   PCIe Base Spec r2.1, §4.2.7.1
// ---------------------------------------------------------------------------
package pcie_phy_pkg;



  // axis_register's REG_TYPE for a skid buffer; data_handler, frame_symbols,
  // lane_management and os_generator pass it.
  localparam int SkidBuffer = 2;

  /* verilator lint_off WIDTHTRUNC */
  // -------------------------------------------------------------------------
  // Symbol codes
  // -------------------------------------------------------------------------
  // Byte values of Symbols after 8b/10b decoding; the K flag travels
  // separately. phy_special_k_e names the twelve K codes (PCIe Base Spec
  // r2.1, §4.2.1.2) and is not used; the framing and link-management names
  // (COM, STP, ...) are in phy_layer_special_symbols_e further down.
  typedef enum logic [7:0] {
            K28_0 = 8'b000_11100,
            K28_1 = 8'b001_11100,
            K28_2 = 8'b010_11100,
            K28_3 = 8'b011_11100,
            K28_4 = 8'b100_11100,
            K28_5 = 8'b101_11100,
            K28_6 = 8'b110_11100,
            K28_7 = 8'b111_11100,
            K23_7 = 8'b111_10111,
            K27_7 = 8'b111_11011,
            K29_7 = 8'b111_11101,
            K30_7 = 8'b111_11110
          } phy_special_k_e;


  // TS1 and TS2 are the TS1 and TS2 Identifiers, D10.2 and D5.2; TS1_INV and
  // TS2_INV are the same Symbols received on a Lane with inverted polarity,
  // D21.5 and D26.5 (§4.2.4.1, §4.2.4.4). PAD_ has the value of PAD, K23.7.
  // SDS, SDS_BODY and IDLE are 8 GT/s values.
  typedef enum logic [7:0] {
            TS1      = 8'h4A,
            TS2      = 8'h45,
            TS1_INV  = 8'hB5,
            TS2_INV  = 8'hBA,
            SDS      = 8'hE1,
            PAD_     = 8'hf7,  // K23.7
            SDS_BODY = 8'h55,
            IDLE     = 8'h66
          } train_seq_e;

  // -------------------------------------------------------------------------
  // 8 GT/s Training Sequence and equalisation fields
  // -------------------------------------------------------------------------
  // At 2.5 and 5.0 GT/s, Symbols 6-15 of a TS1 or TS2 are all the TS
  // Identifier (§4.2.4.1). These types give Symbols 6-9 their 8 GT/s
  // equalisation fields. pcie_ltssm_downstream reads and writes the Symbol 6
  // fields only; no module names a Symbol 7-9 field. presets_coeff_t is the
  // type of the LTSSM's per-lane preset_coeff_o.

  // TS2 Symbol 6.
  typedef struct packed {
            logic       req_equal;
            logic       quience_guarantee;
            logic [5:0] rsvd;
          } ts2_symbol6_t;

  // TS1 Symbols 6 to 9.
  typedef struct packed {
            logic       use_preset;
            logic [3:0] trans_preset;
            logic       rst_eieos;
            logic [1:0] ec;
          } ts1_symbol6_t;

  typedef struct packed {
            logic [1:0] rsvd;
            logic [5:0] fs_pre_cursor;
          } ts1_symbol7_t;


  typedef struct packed {
            logic [1:0] rsvd;
            logic [5:0] lf_cursor_coef;
          } ts1_symbol8_t;

  typedef struct packed {
            logic       parity;
            logic       reject_coef;
            logic [5:0] post_cursor_coef;
          } ts1_symbol9_t;

  // Symbol 6 as TS1 fields, TS2 fields or a plain byte.
  typedef union packed {
            ts2_symbol6_t ts2;
            ts1_symbol6_t ts1;
            logic [7:0]   whole;
          } ts_symbol6_union_t;


  // One Lane's 8 GT/s equalisation presets.
  typedef struct packed {
            logic [15:15] RsvdP2;
            logic [14:12] upstream_rx_preset_hint;
            logic [11:8]  upstream_tx_preset;
            logic [7:7]   RsvdP;
            logic [6:4]   downstream_rx_preset_hint;
            logic [3:0]   downstream_tx_preset;
          } lane_equal_ctrl_reg_t;


  typedef struct packed {
            logic [7:0] pre_cursor;
            logic [7:0] cursor_coef;
            lane_equal_ctrl_reg_t lane_equal_reg;
          } presets_coeff_t;


  // One Symbol; pcie_tsos_t uses it for Symbols 10-15.
  typedef struct packed {logic [7:0] symbol;} ts_generic_symbol_t;


  // -------------------------------------------------------------------------
  // Special Symbols and 8 GT/s definitions
  // -------------------------------------------------------------------------
  // phy_layer_special_symbols_e holds the Special Symbols of Table 4-1
  // (§4.2.1.2) by function name, then TS1OS to SKP_END for the 8 GT/s
  // branches. EIEOS has no user, and EIOS appears only in an empty 5.0 GT/s
  // test in ordered_set_handler. Apart from phy_user_t, ubyte and data_t,
  // the rest of this block is 8 GT/s only: framing tokens, the STP token
  // layout, per-lane scrambler seeds and GEN3_SDS. rx_tx_presets_e, data_t
  // and GEN3_SDS are not used.
  typedef enum logic [7:0] {
            COM      = 8'hbc,  // K28.5
            STP      = 8'hfb,  // K27.7
            SDP      = 8'h5c,  // K28.2
            ENDP     = 8'hfd,  // K29.7
            EDB      = 8'hfe,  // K30.7
            PAD      = 8'hf7,  // K23.7
            SKP      = 8'h1c,  // K28.0
            FTS      = 8'h3c,  // K28.1
            IDL      = 8'h7c,  // K28.3
            EIE      = 8'hfc,  // K28.7
            RV2      = 8'h9c,  // K28.4
            RV3      = 8'hdc,  // K28.6
            TS1OS    = 8'h1E,
            TS2OS    = 8'h2D,
            EIOS     = 8'h66,
            EIEOS    = 8'h00,
            GEN3_SKP = 8'hAA,
            SKP_END  = 8'hE1


          } phy_layer_special_symbols_e;


  // 8 GT/s framing tokens, first Symbol in bits 7:0; check_sdp compares
  // against GEN3_SDP.
  typedef enum logic [31:0] {
            GEN3_IDL = '0,
            GEN3_SDP = {16'h0, 8'b10101100, 8'b11110000},
            GEN3_EDB = {8'b11000000, 8'b11000000, 8'b11000000, 8'b11000000},
            GEN3_EDS = {8'h0, 8'b10010000, 8'b10000000, 8'b00011111}
          } gen3_special_symbols_e;

  // Not used.
  typedef enum logic [63:0] {
            ReceiverpresetHintDSP    = 64'hAABBCCDD1122,
            ReceiverpresetHintUSP    = 64'h2211DDCCBBAA,
            TransmitterPresetHintUSP = 64'h11AA22BB33CC44DD

          } rx_tx_presets_e;

  // Three flags; pcie_ltssm_downstream sizes its default USER_WIDTH with
  // $bits of this type.
  typedef struct packed {
            logic use_link_in;
            logic is_config_tsos;
            logic is_polling_tsos;
          } phy_user_t;

  // The 8 GT/s STP token, one struct per byte; gen_3_stp_t holds byte 0 in
  // bits 7:0. Its byte_3 field is declared with gen_3_stp_byte0_t, so
  // gen_3_stp_byte3_t is not used.
  typedef struct packed {
            logic [3:0] tlp_len0;
            logic [3:0] rsvd;
          } gen_3_stp_byte0_t;

  typedef struct packed {
            logic fp;
            logic [6:0] tlp_len1;
          } gen_3_stp_byte1_t;

  typedef struct packed {
            logic [3:0] fcrc;
            logic [3:0] tlp_seq1;
          } gen_3_stp_byte2_t;

  typedef struct packed {logic [7:0] tlp_seq0;} gen_3_stp_byte3_t;

  typedef struct packed {
            gen_3_stp_byte0_t byte_3;
            gen_3_stp_byte2_t byte_2;
            gen_3_stp_byte1_t byte_1;
            gen_3_stp_byte0_t byte_0;
          } gen_3_stp_t;


  // Per-lane 8 GT/s scrambler seeds; gen3_scramble reads gen3_seed_values.
  typedef enum logic [31:0] {
            lane0_seed = 32'h1DBFBC,
            lane1_seed = 32'h0607BB,
            lane2_seed = 32'h1EC760,
            lane3_seed = 32'h18C0DB,
            lane4_seed = 32'h010F12,
            lane5_seed = 32'h19CFC9,
            lane6_seed = 32'h0277CE,
            lane7_seed = 32'h1BB807

          } gen3_seed_values_e;

  typedef logic [7:0] ubyte;


  // gen3_scramble indexes this array by Lane number, but the list starts
  // with lane7_seed, so element 0 holds Lane 7's seed. No core lists
  // gen3_scramble.sv.
  logic [23:0] gen3_seed_values[8] = {
          lane7_seed, lane6_seed, lane5_seed, lane4_seed, lane3_seed, lane2_seed, lane1_seed, lane0_seed
        };

  // Not used.
  typedef logic [7:0] data_t;

  // Not used.
  logic [127:0] GEN3_SDS = {
          8'h55,
          8'h47,
          8'h4E,
          8'hC7,
          8'hCC,
          8'hC6,
          8'hC9,
          8'h25,
          8'h6E,
          8'hEC,
          8'h88,
          8'h7F,
          8'h80,
          8'h8D,
          8'h8B,
          8'h8E
        };

  // -------------------------------------------------------------------------
  // Training Sequence layout, data rates and Ordered Set containers
  // -------------------------------------------------------------------------
  // A TS1 or TS2 is 16 Symbols (§4.2.4.1, Tables 4-2 and 4-3): COM, Link
  // Number, Lane Number, N_FTS, Data Rate Identifier, Training Control, then
  // the TS Identifier in Symbols 6-15. pcie_tsos_t gives that layout, and
  // pcie_ordered_set_t holds the same 16 Symbols as bytes. Both hold
  // Symbol 0 in bits 7:0.

  // Symbol 5, Training Control: bit 0 Hot Reset, bit 1 Disable Link, bit 2
  // Loopback, bit 3 Disable Scrambling. Bit 4, Compliance Receive in a TS1,
  // falls in rsvd.
  typedef struct packed {
            logic [7:4] rsvd;
            logic       scramble;
            logic       loopback;
            logic       dis_link;
            logic       hot_rst;
          } training_ctrl_t;

  // Symbol 4, Data Rate Identifier: bit 1 advertises 2.5 GT/s, bit 2 5.0
  // GT/s, bit 6 is Autonomous Change and bit 7 speed_change; bits 0 and 3-5
  // are reserved (§4.2.4.1). gen3_basic also sets bit 3, which r2.1
  // reserves.
  typedef enum logic [7:0] {
            gen1_basic = 8'b000_00010,
            gen2_basic = 8'b000_00110,
            gen3_basic = 8'b000_01110
          } rate_id_e;

  // The current data rate as a thermometer code, so rates compare by
  // magnitude (curr_data_rate_i < gen3). In rate_id_t, rate is bits 5:1:
  // gen1 sets bit 1 and gen2 bits 1 and 2, the Symbol 4 encoding.
  typedef enum logic [4:0] {
            gen1 = 5'b00001,
            gen2 = 5'b00011,
            gen3 = 5'b00111,
            gen4 = 5'b01111,
            gen5 = 5'b11111
          } rate_speed_e;

  typedef struct packed {
            logic        speed_change;
            logic        autonomous_change;
            rate_speed_e rate;
            logic        rsvd0;
          } rate_id_t;

  // Fields run from Symbol 15 down to Symbol 0, so com lands in bits 7:0.
  typedef struct packed {
            ts_generic_symbol_t [5:0]   ts_id;
            ts1_symbol9_t               ts_s9;
            ts1_symbol8_t               ts_s8;
            ts1_symbol7_t               ts_s7;
            ts_symbol6_union_t          ts_s6;
            training_ctrl_t             train_ctrl;
            rate_id_t                   rate_id;
            logic [7:0]                 n_fts;
            logic [7:0]                 lane_num;
            logic [7:0]                 link_num;
            phy_layer_special_symbols_e com;
          } pcie_tsos_t;


  // pcie_ltssm_downstream's request to os_generator, which acts on valid,
  // gen_ts1, gen_ts2, gen_idle and gen_eios.
  typedef struct packed {
            ts2_symbol6_t ts6_sym;
            rate_id_t     rate_id;
            logic [7:0]   link_number;
            logic         set_speed_change;
            logic         set_lane;
            logic         set_link;
            logic         gen_idle;
            logic         gen_skp;
            logic         gen_eios;
            logic         gen3_eieos;
            logic         gen2_eieos;
            logic         gen_ts2;
            logic         gen_ts1;
            logic         valid;
          } gen_os_struct_t;


  typedef struct packed {logic [15:0][7:0] symbols;} pcie_ordered_set_t;



  // -------------------------------------------------------------------------
  // Functions
  // -------------------------------------------------------------------------
  // Ordered Set builders that pcie_ltssm_downstream calls for the template it
  // passes to os_generator (gen_ts_os, gen_zeros, gen_eios, gen_eieos), 8 GT/s
  // framing helpers for data_handler and frame_symbols (check_sdp, check_stp,
  // gen_fcrc_parity, gen_stp_gen3), and functions nothing calls (gen_idle,
  // reset_lane_equal_ctrl_reg, gen_eq_tsos, get_tlp_len, gen_sds_os, gen_skp).

  // 8 GT/s: true when bits 15:0 equal GEN3_SDP's, F0h then ACh.
  function automatic logic [0:0] check_sdp(input logic [31:0] data_i);
    begin
      if ((data_i[15:0] == GEN3_SDP[15:0]))
      begin
        return '1;
      end
      else
      begin
        return '0;
      end
    end
  endfunction


  // 8 GT/s: true when bits 3:0 are 1111b, the fixed low nibble of an STP
  // token's first byte (see gen_stp_gen3).
  function automatic logic [0:0] check_stp(input logic [31:0] data_i);
    begin
      if ((data_i[3:0] == '1))
      begin
        return '1;
      end
      else
      begin
        return '0;
      end
    end
  endfunction


  // Not called. Clears an entry and sets both downstream preset fields to 4;
  // its five local variables are never used.
  function automatic void reset_lane_equal_ctrl_reg(output lane_equal_ctrl_reg_t reg_t);
    begin
      logic [14:12] upstream_rx_preset_hint;
      logic [ 11:8] upstream_tx_preset;
      logic [  7:7] RsvdP;
      logic [  6:4] downstream_rx_preset_hint;
      logic [  3:0] downstream_tx_preset;
      reg_t = '0;
      reg_t.downstream_tx_preset = 8'h4;
      reg_t.downstream_rx_preset_hint = 8'h4;
    end
  endfunction



  // 8 GT/s STP token: the 4-bit FCRC over the 11-bit TLP length, and a parity
  // bit over the length and the FCRC.
  function static void gen_fcrc_parity(output logic [3:0] fcrc_out, output logic parity_out,
                                         input logic [10:0] tlp_length);
    begin
      fcrc_out[0] = tlp_length[10] ^ tlp_length[7] ^ tlp_length[6] ^ tlp_length[4] ^ tlp_length[2]
              ^ tlp_length[1] ^ tlp_length[0];
      fcrc_out[1] = tlp_length[10] ^ tlp_length[9] ^ tlp_length[7] ^ tlp_length[5] ^ tlp_length[4]
              ^ tlp_length[3] ^ tlp_length[2];
      fcrc_out[2] = tlp_length[9] ^ tlp_length[8] ^ tlp_length[6] ^ tlp_length[4] ^ tlp_length[3]
              ^ tlp_length[2] ^ tlp_length[1];
      fcrc_out[3] = tlp_length[8] ^ tlp_length[7] ^ tlp_length[5] ^ tlp_length[3] ^ tlp_length[2]
              ^ tlp_length[1] ^ tlp_length[0];
      parity_out = tlp_length[10] ^ tlp_length[9] ^ tlp_length[8] ^ tlp_length[7] ^ tlp_length[6]
                 ^ tlp_length[5] ^ tlp_length[4] ^ tlp_length[3] ^ tlp_length[2] ^ tlp_length[1] ^ tlp_length[0]
                 ^ fcrc_out[3] ^ fcrc_out[2] ^ fcrc_out[1] ^  fcrc_out[0];
    end
  endfunction




  // A TS1 or TS2 (TSOS_ is TS1 or TS2) at 2.5 or 5.0 GT/s: COM, link_num,
  // lane_num, N_FTS FFh (255, the most a component may request, §4.2.4.5),
  // rate_id, train_ctrl, and TSOS_ in Symbols 6-15. Any other rate returns
  // all zeros. ts_s6 to ts_s9 are not used, although pcie_ltssm_downstream
  // passes Symbol 6 values in ts_s6.
  function static pcie_ordered_set_t gen_ts_os(
      input rate_speed_e rate_speed = gen1,
      input train_seq_e TSOS_ = TS1,
      input train_seq_e link_num = PAD_,
      input train_seq_e lane_num = PAD_, 
      input rate_id_t rate_id = rate_id_t'(gen1_basic),
      input training_ctrl_t train_ctrl = '0,
      input ubyte ts_s6 =  TS1,
      input ubyte ts_s7 =  TS1,
      input ubyte ts_s8 =  TS1,
      input ubyte ts_s9 =  TS1
      );

    pcie_tsos_t temp_os;
    // Not used: the loop below declares its own tsos_i.
    integer tsos_i;



    temp_os = '0;
    if (rate_speed == gen1 || rate_speed == gen2)
    begin
      temp_os.com        = COM;
      temp_os.link_num   = link_num;
      temp_os.lane_num   = lane_num;
      temp_os.rate_id    = rate_id;
      temp_os.train_ctrl = train_ctrl;
      temp_os.n_fts      = '1;
      temp_os.ts_s6      =  ts_symbol6_union_t'(TSOS_);
      temp_os.ts_s7      =  ts_symbol6_union_t'(TSOS_);

      temp_os.ts_s8      =  ts_symbol6_union_t'(TSOS_);
      temp_os.ts_s9      =  ts_symbol6_union_t'(TSOS_);
      for (int tsos_i = 0; tsos_i < 6; tsos_i++)
      begin
        temp_os.ts_id[tsos_i] = ubyte'(TSOS_);
      end
    end
    return pcie_ordered_set_t'(temp_os);
  endfunction


  // Not called. Passes its inputs to gen_ts_os, with an 8 GT/s default
  // rate_id.
  function static void gen_eq_tsos(
      output pcie_ordered_set_t tsos_out, input rate_speed_e rate_speed = gen1,
      input train_seq_e TSOS_ = TS1, input train_seq_e link_num = PAD_,
      input train_seq_e lane_num = PAD_, input rate_id_t rate_id = rate_id_t'(gen3_basic),
      input training_ctrl_t train_ctrl = '0, input ts_symbol6_union_t ts_s6 = ubyte'(TSOS_),
      input ts1_symbol6_t ts_s7 =  ubyte'(TSOS_), input ts1_symbol6_t ts_s8 =  ubyte'(TSOS_),
      input ts1_symbol6_t ts_s9 =  ubyte'(TSOS_));
    begin

      pcie_tsos_t temp_os;
      temp_os = gen_ts_os( rate_speed, TSOS_, link_num, lane_num, rate_id, train_ctrl, ts_s6, ts_s7,
                           ts_s8, ts_s9);
      tsos_out = temp_os;
    end
  endfunction



  // 8 GT/s: builds an STP token from the TLP length, FCRC and FP bit, with
  // the sequence number from dllp_frame_in[11:0].
  function automatic void gen_stp_gen3(output gen_3_stp_t stp_out, input logic fp_in,
                                         input logic [3:0] fcrc_in, input logic [10:0] tlp_length,
                                         input logic [31:0] dllp_frame_in);
    begin
      gen_3_stp_t temp_stp = '0;
      {temp_stp.byte_2, temp_stp.byte_3} = dllp_frame_in[15:0];
      temp_stp.byte_2.fcrc               = fcrc_in;
      temp_stp.byte_0.tlp_len0           = tlp_length[3:0];
      temp_stp.byte_0.rsvd               = '1;
      temp_stp.byte_1.tlp_len1           = tlp_length[10:4];
      temp_stp.byte_1.fp                 = fp_in;
      stp_out                            = temp_stp;
    end
  endfunction

  // Not called. Reads an STP token's TLP length into 8 bits, dropping its
  // top 3 bits.
  function automatic void get_tlp_len(output logic [7:0] length, input logic [31:0] data_i);
    begin
      gen_3_stp_t temp_stp = data_i;
      length = {temp_stp.byte_1.tlp_len1, temp_stp.byte_0.tlp_len0};
    end
  endfunction



  // An all-zero Ordered Set; the loop has an empty body.
  function static pcie_ordered_set_t gen_zeros();
    begin
      pcie_ordered_set_t temp_os;
      temp_os = '0;
      for (int i = 0; i < 16; i++)
      begin
      end
      return temp_os;
    end
  endfunction



  // Not called. Four EIOS, COM IDL IDL IDL each (§4.2.4.2): the same Symbols
  // as gen_eios below gen3.
  function static pcie_ordered_set_t gen_idle();
    begin
      pcie_ordered_set_t temp_os;
      temp_os = '0;
      for (int i = 0; i < 16; i++)
      begin
        if (i[1:0] == '0)
        begin
          temp_os[8*i+:8] = COM;
        end
        else
        begin
          temp_os[8*i+:8] = IDL;
        end
      end
      return temp_os;
    end
  endfunction


  // Not called. Named for the 8 GT/s SDS, but each temp_os[k] assignment
  // selects bit k of the packed set, not Symbol k, so no SDS is built.
  function automatic void gen_sds_os(output pcie_ordered_set_t sds_out);
    begin
      pcie_ordered_set_t temp_os;
      temp_os     = '0;
      temp_os[0]  = SDS;
      temp_os[1]  = SDS_BODY;
      temp_os[2]  = SDS_BODY;
      temp_os[3]  = SDS_BODY;
      temp_os[4]  = SDS_BODY;
      temp_os[5]  = SDS_BODY;
      temp_os[6]  = SDS_BODY;
      temp_os[7]  = SDS_BODY;
      temp_os[8]  = SDS_BODY;
      temp_os[9]  = SDS_BODY;
      temp_os[10] = SDS_BODY;
      temp_os[11] = SDS_BODY;
      temp_os[12] = SDS_BODY;
      temp_os[13] = SDS_BODY;
      temp_os[14] = SDS_BODY;
      temp_os[15] = SDS_BODY;
      sds_out     = temp_os;
    end
  endfunction

  // Not called. Below gen3, four SKP Ordered Sets, COM and three SKP each
  // (§4.2.7.1); from gen3, an AAh, E1h and FFh pattern. The result is in
  // skp_out: the function is declared to return pcie_ordered_set_t but
  // returns nothing.
  function automatic pcie_ordered_set_t gen_skp(output pcie_ordered_set_t skp_out,
        input rate_speed_e rate_speed = gen1);
    begin
      pcie_ordered_set_t temp_os;
      temp_os = '0;
      if (rate_speed < gen3)
      begin
        for (int i = 0; i < 16; i++)
        begin
          if (i[1:0] == '0)
          begin
            temp_os[8*i+:8] = COM;
          end
          else
          begin
            temp_os[8*i+:8] = SKP;
          end
        end
      end
      else
      begin
        for (int i = 0; i < 15; i++)
        begin
          if (i < 11)
          begin
            temp_os[8*i+:8] = GEN3_SKP;
          end
          else if (i == 12)
          begin
            temp_os[8*i+:8] = SKP_END;
          end
          else
          begin
            temp_os[8*i+:8] = 8'hff;
          end
        end
      end
      skp_out = temp_os;
    end
  endfunction

  // Below gen3, four EIOS, COM IDL IDL IDL each, which covers the one EIOS
  // required at 2.5 GT/s and the two at 5.0 GT/s (§4.2.4.2). From gen3,
  // sixteen IDLE (66h).
  function automatic void gen_eios(output pcie_ordered_set_t idle_out,
                                     input rate_speed_e rate_speed = gen1);
    begin
      pcie_ordered_set_t temp_os;
      temp_os = '0;
      if (rate_speed < gen3)
      begin
        for (int i = 0; i < 16; i++)
        begin
          if (i[1:0] == '0)
          begin
            temp_os[8*i+:8] = COM;
          end
          else
          begin
            temp_os[8*i+:8] = IDL;
          end
        end
      end
      else
      begin
        for (int i = 0; i < 16; i++)
        begin
          temp_os[8*i+:8] = IDLE;
        end
      end
      idle_out = temp_os;
    end
  endfunction


  // At gen2 and below: COM, fourteen EIE and D10.2, the EIEOS of Table 4-5,
  // which is sent only above 2.5 GT/s (§4.2.4.2). From gen3: 00h and FFh
  // alternating, 00h first.
  function automatic void gen_eieos(output pcie_ordered_set_t eieos_out,
                                      input rate_speed_e rate_speed = gen2);
    begin
      pcie_ordered_set_t temp_os;
      temp_os = '0;
      if (rate_speed <= gen2)
      begin
        temp_os.symbols[7:0] = COM;
        for (int i = 1; i < 15; i++)
        begin
          temp_os[8*i+:8] = EIE;
        end
        temp_os[8*15+:8] = TS1;
      end
      else
      begin
        for (logic [7:0] i = 0; i < 16; i++)
        begin
          // Even Symbols 00h, odd FFh.
          if (!i[0])
          begin
            temp_os[i*8+:8] = 8'h00;
          end
          else
          begin
            temp_os[i*8+:8] = 8'hFF;
          end
        end
      end
      eieos_out = temp_os;
    end
  endfunction


  /* verilator lint_on WIDTHTRUNC */
endpackage
