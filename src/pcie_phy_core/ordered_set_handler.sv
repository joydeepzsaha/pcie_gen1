// ---------------------------------------------------------------------------
// ordered_set_handler -- per-lane Ordered Set receiver: TS1, TS2, idle data
//
// Purpose
//   One instance per lane in phy_receive, after the descrambler. Below gen3
//   it collects the 16 Symbols that start with a COM, publishes them on
//   ordered_set_o and checks Symbols 6-15 for the TS1 or TS2 Identifier,
//   including their polarity-inverted forms. idle_valid_o reports Logical
//   Idle data or an IDL Symbol inside a set. pcie_ltssm_downstream consumes
//   the flags and the fields of the set.
//
// Interfaces
//   Input         data_in_i, data_k_in_i, data_valid_i: pipe_width_i / 8
//                 descrambled Symbols per clock, the first in bits 7:0.
//                 sync_header_i: read on the 8 GT/s path only.
//   Control       curr_data_rate_i: below gen3 selects the 8b/10b path.
//                 pipe_width_i: bits per clock; at most four Symbols are read.
//   Result        ordered_set_o: the last completed set, Symbol 0 in bits 7:0.
//                 ts1_valid_o, ts2_valid_o, polarity_inverted_o,
//                 eieos_valid_o: pulse two clocks after a set's last beat.
//                 idle_valid_o: pulses one clock after the beat that shows it.
//
// Clock and reset
//   clk_i only (pipe_rx_usr_clk_i in phy_receive). rst_i is synchronous and
//   active high; data_store_r has no reset.
//
// Limitations
//   The first IDL after a COM raises idle_valid_o; the rest of the EIOS is
//   not checked. eieos_valid_o is evaluated at 2.5 GT/s, where no EIEOS
//   exists, and checks three EIE Symbols, not fourteen. The 8 GT/s states
//   count beats and store no Symbols. CLK_RATE, KEEP_WIDTH and USER_WIDTH
//   are not used.
//
// References
//   PCIe Base Spec r2.1, §4.2.2
//   PCIe Base Spec r2.1, §4.2.4.1
//   PCIe Base Spec r2.1, §4.2.4.2
//   PCIe Base Spec r2.1, §4.2.4.4
// ---------------------------------------------------------------------------
module ordered_set_handler
  import pcie_phy_pkg::*;
#(
    parameter int CLK_RATE   = 100,             //! Clock rate in MHz; not used
    parameter int DATA_WIDTH = 32,              //! Width of data_swapped; data_in_i is 32 bits
    // KEEP_WIDTH and USER_WIDTH are not used.
    parameter int KEEP_WIDTH = DATA_WIDTH / 8,
    parameter int USER_WIDTH = 4
) (
    // ---- clock and reset ----------------------------------------------------
    input  logic                     clk_i,
    input  logic                     rst_i,
    // ---- descrambled Symbols and control ------------------------------------
    input  logic              [ 1:0] sync_header_i,
    input  rate_speed_e              curr_data_rate_i,
    input  logic              [31:0] data_in_i,
    input  logic                     data_valid_i,
    input  logic              [ 3:0] data_k_in_i,
    input  logic              [ 5:0] pipe_width_i,
    // ---- result, to the LTSSM -----------------------------------------------
    output pcie_ordered_set_t        ordered_set_o,
    output logic                     idle_valid_o,
    output logic                     ts1_valid_o,
    output logic                     ts2_valid_o,
    output logic                     eieos_valid_o,
    output logic                     polarity_inverted_o


);

  // Only MaxWordsPerOrderedSet is used: a set is held as 16 Symbols.
  localparam int MaxWordsPerOrderedSet = 16;
  localparam int MaxBytesPerOrderedSet = MaxWordsPerOrderedSet * 4;
  localparam int MaxBytesPerPacket = DATA_WIDTH / 8;

  // -------------------------------------------------------------------------
  // Ordered Set collection
  // -------------------------------------------------------------------------
  // axis_pkt_cnt_r counts the Symbols of the current set; byte_shift Symbols
  // arrive per valid clock.
  //
  //   State              Does                          Exit
  //   ST_IDLE            looks for COM; Logical Idle   COM: ST_RX_GEN1; 8 GT/s sync
  //                                                    header 10b: ST_RX_GEN3(_SKP)
  //   ST_RX_GEN1         stores Symbols to Symbol 15   full: ST_IDLE or
  //                                                    ST_RX_GEN1_OVRFL; IDL: ST_IDLE
  //   ST_RX_GEN1_OVRFL   starts the next set           next beat: ST_RX_GEN1; IDL: ST_IDLE
  //   ST_RX_GEN3         8 GT/s: counts beats          pkt_full: ST_IDLE
  //   ST_RX_GEN3_SKP     8 GT/s: counts beats          pkt_full or SKP_END: ST_IDLE
  //
  // ST_RX_IDLE_GEN1, ST_RX_GEN3_SKP_LAST, ST_RX_FULL_GEN1 and ST_SEND are
  // never entered.
  typedef enum logic [7:0] {
    ST_IDLE,
    ST_RX_GEN1,
    ST_RX_GEN1_OVRFL,
    ST_RX_IDLE_GEN1,
    ST_RX_GEN3,
    ST_RX_GEN3_SKP,
    ST_RX_GEN3_SKP_LAST,
    ST_RX_FULL_GEN1,
    ST_SEND
  } os_decode_state_e;


  os_decode_state_e curr_state;
  os_decode_state_e                   next_state;

  // Symbols of the current set stored so far.
  logic              [           7:0] axis_pkt_cnt_c;
  logic              [           7:0] axis_pkt_cnt_r;

  // The set being collected, Symbol 0 in bits 7:0.
  pcie_ordered_set_t                  ordered_set_c;
  pcie_ordered_set_t                  ordered_set_r;

  // The last completed set: ordered_set_o, and what the TS checks read.
  pcie_ordered_set_t                  ordered_set_out_c;
  pcie_ordered_set_t                  ordered_set_out_r;

  // The completed set including the beat consumed this clock; ST_RX_GEN1
  // captures it into ordered_set_out_c (see the note there).
  pcie_ordered_set_t                  ordered_set_final_c;

  // check_ordered_set_r is high in the clock after a set completes and
  // starts the checks. polarity_inverted_r is only reset and never read;
  // polarity_inverted_o is registered from polarity_inverted_c.
  logic                               check_ordered_set_c;
  logic                               check_ordered_set_r;
  logic                               idle_valid_c;
  logic                               ts1_valid;
  logic                               ts2_valid;
  logic                               eieos_valid;
  logic                               polarity_inverted_c;
  logic                               polarity_inverted_r;


  // The skp registers only ever hold their reset value; nothing reads them.
  logic              [           7:0] skp0_c;
  logic              [           7:0] skp0_r;
  logic              [           7:0] skp1_c;
  logic              [           7:0] skp1_r;
  logic              [           7:0] skp2_c;
  logic              [           7:0] skp2_r;
  logic              [           7:0] skp3_c;
  logic              [           7:0] skp3_r;

  // Never used.
  pcie_tsos_t                         training_set;

  // byte_shift is the number of Symbols per clock (pipe_width_i / 8).
  // packets_per_words is assigned and never read.
  logic              [           7:0] packets_per_words;
  logic              [           7:0] byte_shift;
  logic              [           7:0] byte_index;
  logic              [DATA_WIDTH-1:0] data_swapped;
  logic                               pkt_full;
  logic              [           7:0] word_index;

  // The previous beat while in ST_IDLE (for the Logical Idle check), or
  // byte 1 of an overflowing final beat (for ST_RX_GEN1_OVRFL).
  logic              [          31:0] data_store_c;
  logic              [          31:0] data_store_r;

  assign ordered_set_o = ordered_set_out_r;
  // The set is complete when this beat brings the Symbol count to 16.
  assign pkt_full = (axis_pkt_cnt_r + byte_shift)>= MaxWordsPerOrderedSet;


  //! main sequential block
  always_ff @(posedge clk_i) begin : main_seq
    if (rst_i) begin
      curr_state          <= ST_IDLE;
      check_ordered_set_r <= '0;
      idle_valid_o        <= '0;
      ts1_valid_o         <= '0;
      ts2_valid_o         <= '0;
      eieos_valid_o       <= '0;
      polarity_inverted_o <= '0;
      ordered_set_r       <= '0;
      idle_valid_o        <= '0;
      ts1_valid_o         <= '0;
      ts2_valid_o         <= '0;
      eieos_valid_o       <= '0;
      polarity_inverted_r <= '0;
      axis_pkt_cnt_r      <= '0;
      skp0_r              <= '0;
      skp1_r              <= '0;
      skp2_r              <= '0;
      skp3_r              <= '0;
      ordered_set_out_r   <= '0;
    end else begin
      curr_state          <= next_state;
      ordered_set_r       <= ordered_set_c;
      idle_valid_o        <= idle_valid_c;
      ts1_valid_o         <= ts1_valid;
      ts2_valid_o         <= ts2_valid;
      eieos_valid_o       <= eieos_valid;
      polarity_inverted_o <= polarity_inverted_c;
      axis_pkt_cnt_r      <= axis_pkt_cnt_c;
      skp0_r              <= skp0_c;
      skp1_r              <= skp1_c;
      skp2_r              <= skp2_c;
      skp3_r              <= skp3_c;
      ordered_set_out_r   <= ordered_set_out_c;
      check_ordered_set_r <= check_ordered_set_c;
    end
    //non-resetable
    data_store_r <= data_store_c;
  end


  always_comb begin : send_ordered_set
    // temp_os is assigned and never read.
    pcie_tsos_t temp_os;
    axis_pkt_cnt_c      = axis_pkt_cnt_r;
    next_state          = curr_state;
    ordered_set_c       = ordered_set_r;
    check_ordered_set_c = '0;
    idle_valid_c        = '0;
    temp_os             = ordered_set_r;
    skp0_c              = skp0_r;
    skp1_c              = skp1_r;
    skp2_c              = skp2_r;
    skp3_c              = skp3_r;
    data_swapped        = '0;
    byte_shift          = pipe_width_i >> 3;
    byte_index          = (byte_shift - 1'b1);
    word_index          = axis_pkt_cnt_r * byte_shift;
    packets_per_words   = MaxWordsPerOrderedSet - ((byte_shift) << 2);
    data_store_c        = data_store_r;
    ordered_set_out_c   = ordered_set_out_r;
    ordered_set_final_c = ordered_set_c;
    // data_swapped is the stored word at word_index with this beat's Symbols
    // written into it in reverse order. Only ST_RX_GEN3_SKP and the
    // unreachable ST_RX_IDLE_GEN1 read it, and nothing writes it back to
    // ordered_set_c, so the 8 GT/s states store no Symbols.
    if (data_valid_i) begin
      data_swapped = ordered_set_r[32*word_index[7:2]+:32];
      for (int i = 0; i < 4; i++) begin
        int offset;
        offset = word_index[1:0] + i;
        if (i < byte_shift) begin
          data_swapped[8*offset+:8] = data_in_i[8*(byte_index-i)+:8];
        end
      end
    end
    case (curr_state)
      ST_IDLE: begin
        if (data_valid_i) begin
          data_store_c = data_in_i;
          if (curr_data_rate_i < gen3) begin
            // Logical Idle: every Symbol of this beat is data 00h, and the same
            // bytes of the previous beat, data_store_r, are 00h (PCIe Base Spec
            // r2.1, §4.2.2).
            idle_valid_c = '1;
            for (int i = 0; i < 4; i++) begin
              if (i < byte_shift) begin
                if (data_k_in_i[i] != '0 || data_in_i[i*8+:8] != '0 ||
                (data_store_r[i*8+:8] != '0)) begin
                  idle_valid_c = '0;
                end
              end
            end
            // A COM starts a set: the COM and the Symbols after it in this beat
            // become Symbols 0, 1, ..., and axis_pkt_cnt_c counts them.
            for (int i = 0; i < 4; i++) begin
              if (i < byte_shift) begin
                if ((data_k_in_i[i]) && data_in_i[i*8+:8] == COM) begin
                  for (int j = 0; j < 4; j++) begin
                    if (i + j < byte_shift) begin
                      int sum_idx;
                      sum_idx = (i + j) % 4; // Bounded for synthesis
                      ordered_set_c[j*8+:8] = data_in_i[sum_idx*8+:8];
                    end
                  end
                  next_state = ST_RX_GEN1;
                  axis_pkt_cnt_c = byte_shift - i;
                end else if ((!data_k_in_i[i]) && data_in_i[i*8+:8] == '0) begin
                  // Empty: Logical Idle is detected by the loop above.
                end
              end
            end
          end else begin
            // 8 GT/s: sync header 10b starts ST_RX_GEN3, or ST_RX_GEN3_SKP when
            // Symbol 0 is GEN3_SKP. The SKP test reads ordered_set_c, the
            // stored set, not this beat.
            if ((sync_header_i == 2'b10)) begin
              if (ordered_set_c[7:0] == GEN3_SKP) begin
                next_state = ST_RX_GEN3_SKP;
                axis_pkt_cnt_c = 1'b1;
              end else begin
                axis_pkt_cnt_c = 1'b1;
                next_state = ST_RX_GEN3;
              end
            end
          end
        end
      end
      // Never entered: no state assigns ST_RX_IDLE_GEN1.
      ST_RX_IDLE_GEN1: begin
        if (data_valid_i) begin
          axis_pkt_cnt_c = axis_pkt_cnt_r + 1'b1;
          if (data_swapped[7:0] != '0) begin
            next_state = ST_IDLE;
          end
          if (pkt_full) begin
            next_state = ST_IDLE;
            if (data_swapped[7:0] == '0) begin
              next_state   = ST_IDLE;
              idle_valid_c = '1;
            end
          end
        end
      end
      ST_RX_GEN1: begin
        if (data_valid_i) begin
          axis_pkt_cnt_c = axis_pkt_cnt_r + byte_shift;

          // Capture the completed set, this beat included, before the loop
          // below writes this beat into ordered_set_c. Without this beat the
          // capture would hold the previous set's last Symbols (Symbol 15, or
          // 14 and 15 at a pipe width of 16), and the TS checks read Symbols
          // 6-15. The capture stays above the loop: on an IDL or a COM the
          // loop restarts collection, and a COM in the final beat rewrites
          // ordered_set_c with the next set, which a capture below the loop
          // would take instead.
          ordered_set_final_c = ordered_set_c;
          for (int i = 0; i < 4; i++) begin
            if (i < byte_shift) begin
              ordered_set_final_c[(axis_pkt_cnt_r+i)*8+:8] = data_in_i[8*i+:8];
            end
          end

          if (pkt_full) begin
            check_ordered_set_c = '1;
            ordered_set_out_c   = ordered_set_final_c;
            axis_pkt_cnt_c      = '0;
            // A K Symbol above byte 0 of the final beat starts the next set:
            // byte 1 is kept in data_store_c and becomes Symbol 0 in
            // ST_RX_GEN1_OVRFL. A COM or IDL among this beat's Symbols
            // overrides next_state in the loop below.
            if (data_k_in_i > 1) begin
              axis_pkt_cnt_c = 1'b1;
              data_store_c = data_in_i >> 8;
              next_state = ST_RX_GEN1_OVRFL;
            end else begin
              next_state = ST_IDLE;
            end
          end
          for (int i = 0; i < 4; i++) begin
            if (i < byte_shift) begin
              ordered_set_c[(axis_pkt_cnt_r+i)*8+:8] = data_in_i[8*i+:8];
              // An IDL ends the set and raises idle_valid; the rest of the EIOS
              // is not checked.
              if (data_k_in_i[i] && (data_in_i[8*i+:8] == IDL)) begin
                check_ordered_set_c = '0;
                axis_pkt_cnt_c      = '0;
                idle_valid_c        = '1;
                next_state          = ST_IDLE;
              end
              // A COM restarts collection from this Symbol.
              if ((data_k_in_i[i]) && data_in_i[i*8+:8] == COM) begin
                for (int j = 0; j < 4; j++) begin
                  if (i + j < byte_shift) begin
                    int sum_idx;
                    sum_idx = (i + j) % 4; // Bounded for synthesis
                    ordered_set_c[j*8+:8] = data_in_i[sum_idx*8+:8];
                  end
                end
                next_state = ST_RX_GEN1;
                axis_pkt_cnt_c = byte_shift - i;
              end
            end
          end
        end
      end
      // Symbol 0 is byte 1 of the previous beat; this beat's Symbols follow
      // from Symbol 1.
      ST_RX_GEN1_OVRFL: begin
        if (data_valid_i) begin
          next_state = ST_RX_GEN1;
          axis_pkt_cnt_c = axis_pkt_cnt_r + byte_shift;
          ordered_set_c[7:0] = data_store_r[7:0];
          for (int i = 0; i < 4; i++) begin
            if (i < byte_shift) begin
              if (data_k_in_i[i] && (data_in_i[8*i+:8] == IDL)) begin
                check_ordered_set_c = '0;
                idle_valid_c        = '1;
                next_state          = ST_IDLE;
              end else begin
                ordered_set_c[(axis_pkt_cnt_r+i)*8+:8] = data_in_i[8*i+:8];
              end
            end
          end
        end

      end
      // 8 GT/s: counts beats until pkt_full. Nothing writes ordered_set_c on
      // this path.
      ST_RX_GEN3: begin
        if (data_valid_i) begin
          axis_pkt_cnt_c = axis_pkt_cnt_r + 1'b1;
          if (pkt_full) begin
            check_ordered_set_c = '1;
            ordered_set_out_c   = ordered_set_c;
            axis_pkt_cnt_c      = '0;
            next_state          = ST_IDLE;
          end
        end
      end
      ST_RX_GEN3_SKP: begin
        // 8 GT/s SKP: ends on pkt_full or when data_swapped[7:0] is SKP_END.
        // Nothing is captured.
        if (data_valid_i) begin
          axis_pkt_cnt_c = axis_pkt_cnt_r + 1'b1;
          if (pkt_full) begin
            axis_pkt_cnt_c = '0;
            next_state     = ST_IDLE;
          end
          if ((data_swapped[7:0] == SKP_END)) begin
            axis_pkt_cnt_c = '0;
            next_state     = ST_IDLE;
          end
        end
      end
      default: begin
      end
    endcase
  end

  // -------------------------------------------------------------------------
  // Ordered Set checks
  // -------------------------------------------------------------------------
  // The TS checks run on the registered capture (check_ordered_set_r,
  // ordered_set_out_r), so the collector can return to ST_IDLE and take the
  // next set meanwhile. When a set completes, ts1_valid, ts2_valid and
  // eieos_valid start set and each failed test clears its flag. The flags are
  // registered again, so they pulse two clocks after the set's last beat.
  always_comb begin : check_ordered_set
    ts1_valid           = '0;
    ts2_valid           = '0;
    eieos_valid         = '0;
    polarity_inverted_c = '0;
    if (check_ordered_set_r) begin
      ts1_valid   = '1;
      ts2_valid   = '1;
      eieos_valid = '1;
      if (curr_data_rate_i < gen3) begin
        // A TS1 has the TS1 Identifier, D10.2 (4Ah), in all of Symbols 6-15,
        // and a TS2 has D5.2 (45h) (PCIe Base Spec r2.1, §4.2.4.1). On a Lane
        // with inverted polarity they arrive as D21.5 (B5h) and D26.5 (BAh)
        // (§4.2.4.4). All ten Symbols are compared, which relies on
        // ordered_set_out_r holding the set's final beat (see ST_RX_GEN1).
        begin
          logic all_ts1, all_ts1_inv, all_ts2, all_ts2_inv;
          all_ts1     = '1;
          all_ts1_inv = '1;
          all_ts2     = '1;
          all_ts2_inv = '1;
          for (int i = 6; i < 16; i++) begin
            if (ordered_set_out_r[8*i+:8] != TS1) all_ts1 = '0;
            if (ordered_set_out_r[8*i+:8] != TS1_INV) all_ts1_inv = '0;
            if (ordered_set_out_r[8*i+:8] != TS2) all_ts2 = '0;
            if (ordered_set_out_r[8*i+:8] != TS2_INV) all_ts2_inv = '0;
          end

          if (all_ts1) begin
            ts1_valid = '1;
          end else if (all_ts1_inv) begin
            ts1_valid           = '1;
            polarity_inverted_c = '1;
          end else begin
            ts1_valid = '0;
          end

          if (all_ts2) begin
            ts2_valid = '1;
          end else if (all_ts2_inv) begin
            ts2_valid           = '1;
            polarity_inverted_c = '1;
          end else begin
            ts2_valid = '0;
          end
        end
        if (curr_data_rate_i == gen1) begin
          // This IDL test and the EIOS test below have empty bodies and
          // change nothing.
          if (ordered_set_r[23:8] != {IDL, IDL}) begin
          end
        end
        if (curr_data_rate_i == gen2) begin
          if (ordered_set_r[15:0] != {EIOS, EIOS}) begin
          end
        end
        // EIEOS: Symbols 1-3 of ordered_set_r must be EIE. The test also runs
        // at 2.5 GT/s, where no EIEOS exists, and the EIEOS has fourteen EIE
        // Symbols, not three (§4.2.4.2).
        for (int i = 1; i < 4; i++) begin
          if (ordered_set_r[8*i+:8] != EIE) begin
            eieos_valid = '0;
          end
        end
      end else begin
        // 8 GT/s: Symbol 0 against TS1OS and TS2OS, and Symbols 0-3 against
        // FFh, 00h, FFh, 00h. ordered_set_r is never loaded on this path.
        if (ordered_set_r[7:0] != TS1OS) begin
          ts1_valid = '0;
        end
        if (ordered_set_r[7:0] != TS2OS) begin
          ts2_valid = '0;
        end
        if (ordered_set_r[31:0] != 32'h00FF00FF) begin
          eieos_valid = '0;
        end
      end
    end
  end
endmodule
