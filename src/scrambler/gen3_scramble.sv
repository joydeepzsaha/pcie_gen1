// ---------------------------------------------------------------------------
// gen3_scramble -- Gen3 (128b/130b) scrambler for one lane, not integrated
//
// Purpose
//   Not integrated: nothing in the design instantiates this module. scrambler
//   has no instance of it, and scrambler.core and synth/endpoint.tcl leave the
//   file out. It is a Gen3 (128b/130b) scrambler; PCIe Base Spec r2.1 defines
//   only the 8b/10b code, and this design uses gen1_scramble at every rate.
//   Each valid clock's word reaches data_out_o through one register. Inside
//   an Ordered Set block, a word that matches the EIEOS or the SKP check and
//   a TS1 or TS2 identifier pass through; every other byte below
//   pipe_width_i/8 is XORed with the bit-reversed LFSR value for its
//   position. An EIEOS word sets is_eieos_r, which reloads lfsr_r with
//   gen3_seed_values[lane_number[2:0]].
//
// Interfaces
//   Data in   data_in_i, data_valid_i, pipe_width_i: one word per valid clock.
//   Block     sync_header_i: 10b sets is_os_c and 01b clears it, the codes
//             lane_management drives for an Ordered Set and for data.
//   Lane      lane_number: indexes gen3_seed_values; the TS1/TS2 check is
//             lane 0 only.
//   Data out  data_out_o: registered; data_valid_o: data_valid_i delayed one
//             clock. data_k_out_o is tied to 0.
//   Unused    ltssm_polling_compliance_i, data_k_in_i.
//
// Clock and reset
//   clk_i only. rst_i is synchronous and active high; an EIEOS (is_eieos_r)
//   has the same effect on lfsr_r, data_valid_o and the block flags.
//   data_out_o_r and data_k_r have no reset.
//
// Limitations
//   Lane n loads gen3_seed_values[n], which holds Lane 7-n's seed:
//   pcie_phy_pkg fills the array from lane7_seed down.
//   Bit 0 of a scrambled byte meets LFSR bit 23, which is always 0, so it is
//   never inverted; PCIe Base Spec r3.0, §C.2 XORs it with LFSR bit 22.
//
// References
//   PCIe Base Spec r3.0, §4.2.2.4
//   PCIe Base Spec r3.0, §C.2
// ---------------------------------------------------------------------------
module gen3_scramble
  import pcie_phy_pkg::*;
(

    input  logic        clk_i,                       //! Clock
    input  logic        rst_i,                       //! Synchronous, active high
    input  logic [ 7:0] lane_number,
    input  logic [ 1:0] sync_header_i,
    input  logic [31:0] data_in_i,
    input  logic        data_valid_i,
    output logic        data_valid_o,
    output logic [31:0] data_out_o,
    input  logic        ltssm_polling_compliance_i,
    input  logic [ 3:0] data_k_in_i,
    input  logic [ 5:0] pipe_width_i,
    output logic [ 3:0] data_k_out_o
);

  logic [     23:0] lfsr_c;
  logic [     23:0] lfsr_r;
  logic [     23:0] lfsr_out             [5];
  logic [      3:0] scramble_reset;
  logic [      3:0] disable_lfsr_advance;
  logic [     31:0] data_out_o_c;
  logic [     31:0] data_out_o_r;
  logic [(8*4)-1:0] scrambled_data;

  logic [      3:0] data_k_swapped;
  logic [     31:0] data_in_swapped;
  logic [      7:0] byte_idx;
  logic [      3:0] data_k_c;
  logic [      3:0] data_k_r;
  logic             is_os_c;
  logic             is_os_r;
  logic             ts1_ts2_c;
  logic             ts1_ts2_r;
  logic [      3:0] is_eieos_c;
  logic [      3:0] is_eieos_r;
  logic [      3:0] is_skp_c;
  logic [      3:0] is_skp_r;
  logic [      3:0] ts_detected;
  logic [      3:0] eieos_detected;
  logic [      3:0] eieos_mask;
  logic [      3:0] skip_detected;
  logic [     15:0] eieos_compare;

  assign lfsr_out[0] = lfsr_r;
  assign data_k_out_o = '0;

  for (genvar i = 0; i < 4; i++) begin : gen_byte_scramble
    gen3_byte_scramble byte_scramble_inst (
        .disable_lfsr_advance(disable_lfsr_advance[0]),
        .lfsr_r(lfsr_out[i]),
        .lfsr_out(lfsr_out[i+1])
    );
  end


  always_ff @(posedge clk_i) begin : scramble_seq_block
    if (rst_i || is_eieos_r) begin
      lfsr_r       <= gen3_seed_values[lane_number[2:0]];
      data_valid_o <= '0;
      is_os_r      <= '0;
      ts1_ts2_r    <= '0;
      is_eieos_r   <= '0;
      is_skp_r     <= '0;
    end else begin
      lfsr_r       <= lfsr_c;
      data_valid_o <= data_valid_i;
      is_os_r      <= is_os_c;
      ts1_ts2_r    <= ts1_ts2_c;
      is_eieos_r   <= is_eieos_c;
      is_skp_r     <= is_skp_c;
    end
    data_k_r     <= data_k_c;
    data_out_o_r <= data_out_o_c;
  end

  always_comb begin : sync_header_decode
    is_os_c    = is_os_r;
    ts1_ts2_c  = ts1_ts2_r;
    is_eieos_c = is_eieos_r;
    is_skp_c   = is_skp_r;
    if (data_valid_i) begin
      ts1_ts2_c  = ts_detected != '0;
      is_eieos_c = eieos_detected != '0;
      is_skp_c   = skip_detected != '0;
      case (sync_header_i)
        2'b10: begin
          is_os_c = '1;
        end
        2'b01: begin
          is_os_c    = '0;
          ts1_ts2_c  = '0;
          is_eieos_c = '0;
          is_skp_c   = '0;
        end
        default: begin

        end
      endcase
    end
  end


  // PCIe Base Spec r3.0, §4.2.2.4 exempts from scrambling every Symbol of an
  // EIEOS, FTS, SDS, EIOS or SKP Ordered Set and Symbol 0 of a TS1 or TS2. A
  // SKP Ordered Set does not advance the LFSR, and an EIEOS reloads the seed.
  // Per byte below pipe_width_i/8. Inside an Ordered Set block (is_os_c), a
  // word whose low 16 bits are 00FFh (the EIEOS check), a TS1 or TS2
  // identifier (loop index 0, lane 0 only) or a word that skip_detected
  // matches passes through unscrambled; every other byte is XORed with the
  // bit-reversed LFSR value for its position. The loop index i maps to byte
  // (pipe_width_i/8 - 1 - i). Only disable_lfsr_advance[0] reaches the
  // gen3_byte_scramble instances.
  always_comb begin : scramble_comb_block
    scramble_reset       = '0;
    disable_lfsr_advance = '0;
    data_out_o_c         = data_out_o_r;
    lfsr_c               = lfsr_r;
    eieos_compare        = (data_in_i[15:0] == 16'h00FF);
    scrambled_data       = data_in_i;
    skip_detected        = (data_in_i[15:0] == {GEN3_SKP, 8'h1E});
    // Always false: scramble_reset is cleared above and set only further down.
    // The EIEOS seed reload happens through is_eieos_r in scramble_seq_block.
    if (scramble_reset != '0) begin
      lfsr_c = gen3_seed_values[lane_number[2:0]];
    end else if (data_valid_i) begin
      // The LFSR advances pipe_width_i/8 bytes per valid clock, unless the SKP
      // arm below sets disable_lfsr_advance[0].
      lfsr_c = lfsr_out[(pipe_width_i>>3)];
    end
    if (data_valid_i) begin
      data_out_o_c = data_in_i;
      for (int i = 0; i < 4; i++) begin
        scrambled_data[i] = data_in_i[i*8+:8];
        ts_detected[i]    = '0;
        eieos_detected[i] = '0;
        if (i < (pipe_width_i >> 3)) begin
          byte_idx      = ((pipe_width_i >> 3) - 1) - i;
          eieos_mask    = '1;
          eieos_mask[i] = '0;
          if (is_os_c) begin
            if ((eieos_compare && (sync_header_i == 2'b10))) begin
              scramble_reset[i] = '1;
              eieos_detected[i] = '1;
              data_out_o_c[byte_idx<<3+:8] = data_in_i[byte_idx<<3+:8];
            end else if (data_in_i[byte_idx<<3+:8] inside {TS1OS, TS2OS}
              && (lane_number == '0) && (i == '0)) begin
              ts_detected[i] = '1;
              data_out_o_c[byte_idx<<3+:8] = data_in_i[byte_idx<<3+:8];
            end else if (skip_detected) begin
              disable_lfsr_advance[i]      = '1;
              data_out_o_c[byte_idx<<3+:8] = data_in_i[byte_idx<<3+:8];
            end else begin
              data_out_o_c[byte_idx <<3 +: 8] = (data_in_i[byte_idx<<3+:8]
             ^ (24'({<<{lfsr_out[byte_idx]}})));
            end
          end else begin
            data_out_o_c[byte_idx <<3 +: 8] = (data_in_i[byte_idx<<3+:8]
           ^ (24'({<<{lfsr_out[byte_idx]}})));
          end
        end
      end
    end
  end
  assign data_out_o = data_out_o_r;
endmodule
