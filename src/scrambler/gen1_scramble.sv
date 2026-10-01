// ---------------------------------------------------------------------------
// gen1_scramble -- 8b/10b-rate scrambler for one lane
//
// Purpose
//   Scrambles the bytes of one lane with the 16-bit LFSR of PCIe Base Spec
//   r2.1, §4.2.3. A scrambled byte is XORed with the bit-reversed LFSR value
//   for its position, and byte_scramble advances the LFSR eight shifts per
//   Symbol. K Symbols pass unscrambled. A COM initializes the LFSR to FFFFh,
//   and a SKP does not advance it. A COM that does not open a SKP Ordered Set
//   starts a 16-Symbol window, the length of a TS1 or TS2 Ordered Set, in
//   which data Symbols pass unscrambled. The same module descrambles, because
//   the XOR is its own inverse: scrambler wraps it for phy_transmit and
//   phy_receive.
//
// Interfaces
//   Data in    data_in_i, data_k_in_i, data_valid_i: one word and its K flags.
//              A clock with data_valid_i low carries no Symbol; only the
//              stage-0 valid bit changes on it.
//   Width      pipe_width_i: bits per clock. Bytes at and above
//              pipe_width_i/8 pass through unscrambled.
//   Data out   data_out_o, data_k_out_o, data_valid_o: stage 3 of a pipeline
//              that advances only on valid clocks. data_valid_o holds its
//              value through a data_valid_i gap.
//
// Clock and reset
//   clk_i only. rst_i is synchronous and active high; it loads the LFSR with
//   FFFFh and clears every other field.
//
// Limitations
//   pipe_width_i is 16 in this design, because lane_management's pipe_width_o
//   never leaves PipeWidthGen1, so only the two-byte path is in use.
//
// References
//   PCIe Base Spec r2.1, §4.2.3
//   PCIe Base Spec r2.1, §C.1
// ---------------------------------------------------------------------------
module gen1_scramble
  import pcie_phy_pkg::*;
(

    input  logic        clk_i,         //! PIPE TX or RX user clock
    input  logic        rst_i,         //! Synchronous, active high
    input  logic [31:0] data_in_i,
    input  logic        data_valid_i,
    output logic        data_valid_o,
    output logic [31:0] data_out_o,
    input  logic [ 3:0] data_k_in_i,
    input  logic [ 5:0] pipe_width_i,
    output logic [ 3:0] data_k_out_o
);


  // Pipeline depth: stage 0 registers the input word, stage 3 drives the
  // outputs.
  localparam int NumPipelines = 4;
  logic [15:0] lfsr_out[5];
  logic [15:0] lfsr_swapped[4];
  logic [15:0] temp_lfsr_in[4];
  logic [15:0] temp_lfsr_out[4];

  // The registered state. data, data_k, data_valid and lfsr_out hold one
  // entry per pipeline stage. The 5-bit fields are indexed by byte position:
  //   lfsr_in             LFSR value for byte 0 of the next word
  //   scramble_reset      a COM at byte k sets bit k+1: the LFSR restarts at
  //                       FFFFh after the COM
  //   disable_scrambling  byte is inside an Ordered Set (no XOR)
  //   stop_scrambling     the Ordered Set has ended at this byte (XOR again)
  //   skp_os              byte belongs to a SKP Ordered Set
  //   byte_cnt            Symbols of the current Ordered Set counted so far
  typedef struct {
    logic [15:0]                        lfsr_in;
    logic [NumPipelines-1:0][4:0][15:0] lfsr_out;
    logic [4:0]                         scramble_reset;
    logic [4:0]                         disable_scrambling;
    logic [4:0]                         stop_scrambling;
    logic [4:0]                         skp_os;
    logic [NumPipelines-1:0][31:0]      data;
    logic [NumPipelines-1:0][4:0]       data_k;
    logic [NumPipelines-1:0]            data_valid;
    logic [31:0]                        byte_cnt;
  } gen1_scambler_t;


  gen1_scambler_t D, Q;


  assign lfsr_out[0] = Q.lfsr_in;

  // One byte_scramble per byte position: lfsr_out[i] is the LFSR value for
  // byte i of the word and lfsr_out[i+1] the value after it. A pending reset
  // forces lfsr_out[i+1] to FFFFh: scramble_reset[pipe_idx] for this position
  // (pipe_idx runs in reverse byte order), or any bit at or above
  // pipe_width_i/8, which a COM in the last byte sets so that the next word
  // starts from FFFFh.
  for (genvar i = 0; i < 4; i++) begin : gen_byte_scramble
    int   pipe_idx;
    logic reset_byte_scrambler;
    assign pipe_idx = ((pipe_width_i >> 3) - 1) - i;
    assign reset_byte_scrambler = Q.scramble_reset[pipe_idx] || (
    (Q.scramble_reset >> (pipe_width_i >> 3) ) != '0);

    assign temp_lfsr_in[i] = reset_byte_scrambler ? '1 : lfsr_out[i];

    // A SKP Ordered Set's COM never raises scramble_reset (the is_skp_os arm
    // below), so this term is what loads FFFFh after the Ordered Set: its COM
    // initializes the LFSR and its SKPs do not advance it (PCIe Base Spec
    // r2.1, §4.2.3). Holding the LFSR value here instead would skip that
    // initialization. test_scrambler_skpseed checks the data after a SKP
    // Ordered Set against a model of the specification.
    assign lfsr_out[i+1] = reset_byte_scrambler || Q.skp_os[i]? '1 : temp_lfsr_out[i];
    byte_scramble byte_scramble_inst (
        .disable_scrambling('0),
        .lfsr_q            (temp_lfsr_in[i]),
        .lfsr_out          (temp_lfsr_out[i])
    );
  end


  always_ff @(posedge clk_i) begin : scramble_seq_block
    if (rst_i) begin
      Q <= '{lfsr_in: '1, default: 'd0};
    end else begin
      Q <= D;
    end
  end


  always_comb begin : scramble_comb_block
    D                 = Q;
    // data_valid[0] is written on every clock, idle ones included, because it
    // records whether this clock carried a Symbol. Every other field changes
    // only on a valid clock.
    D.data_valid[0]   = data_valid_i;


    if (data_valid_i) begin
      // The scrambling rules are defined per Symbol, not per clock (PCIe Base
      // Spec r2.1, §4.2.3), so nothing below changes on a clock without one.
      // scramble_reset, stop_scrambling and skp_os are one-word pulses, set on
      // one valid clock and used on the next; clearing them here, inside the
      // guard, keeps a pulse alive across idle clocks.
      // test_scrambler_kgap inserts idle clocks around a COM and a SKP.
      D.scramble_reset  = '0;
      D.stop_scrambling = '0;
      D.skp_os          = '0;

      // The LFSR advances once per Symbol (PCIe Base Spec r2.1, §4.2.3). On a
      // clock without data_valid_i, D = Q holds lfsr_in, and since only a COM
      // re-initializes the LFSR, an advance there would desynchronize the
      // stream from its descrambler. The SKP arms below override this value.
      // test_scrambler_stall checks the output across data_valid_i gaps.
      D.lfsr_in = lfsr_out[(pipe_width_i>>3)];

      // Stage 0 takes the input word and a copy of the per-byte LFSR values;
      // stages 1 to 3 shift.
      for (int pipeline_idx = 0; pipeline_idx < NumPipelines; pipeline_idx++) begin
        if (pipeline_idx == 0) begin
          D.data[pipeline_idx] = data_in_i;
          D.data_k[pipeline_idx] = data_k_in_i;
          D.data_valid[pipeline_idx] = data_valid_i;
          for (int lfsr_idx = 0; lfsr_idx < 5; lfsr_idx++) begin
            D.lfsr_out[pipeline_idx][lfsr_idx] = lfsr_out[lfsr_idx];
          end
        end else begin
          D.data_valid[pipeline_idx] = Q.data_valid[pipeline_idx-1];
          D.lfsr_out[pipeline_idx]   = Q.lfsr_out[pipeline_idx-1];
          D.data[pipeline_idx]       = Q.data[pipeline_idx-1];
          D.data_k[pipeline_idx]     = Q.data_k[pipeline_idx-1];
        end
      end

      // An Ordered Set that ended last word re-enables scrambling; one still in
      // progress counts its Symbols and keeps every byte unscrambled.
      if (Q.stop_scrambling != '0) begin
        D.disable_scrambling = '0;
      end else if (Q.disable_scrambling != '0) begin
        D.byte_cnt = Q.byte_cnt + (pipe_width_i >> 3);
        D.disable_scrambling = '1;
      end

      // Per byte position below pipe_width_i/8: Ordered Set tracking on
      // stages 0 and 1, then the XOR from stage 2 into stage 3.
      for (int byte_idx = 0; byte_idx < 4; byte_idx++) begin
        int pipe_idx;
        pipe_idx = ((pipe_width_i >> 3) - 1) - byte_idx;
        lfsr_swapped[byte_idx] = '0;

        if (byte_idx < (pipe_width_i >> 3)) begin
          // End of a SKP Ordered Set: a byte flagged skp_os whose stage-0 byte
          // is not SKP clears disable_scrambling and sets stop_scrambling from
          // this position up. A SKP still in stage 1 flags its byte again below.
          if (Q.skp_os[byte_idx] != '0) begin
            if ((Q.data[0][byte_idx*8+:8] != SKP)) begin
              D.byte_cnt = '0;
              for (int idx = 0; idx < 4; idx++) begin
                if (idx >= byte_idx && idx < (pipe_width_i >> 3)) begin
                  D.disable_scrambling[idx] = '0;
                  D.stop_scrambling[idx] = '1;
                end
              end
            end
          end

          // End of any other Ordered Set: once byte_cnt passes 16 Symbols,
          // scrambling resumes from this position up, unless stage 1 holds a
          // COM, which opens the next Ordered Set.
          if ((Q.byte_cnt + (byte_idx + 1)) > 32'd16) begin
            logic flag;
            flag = '0;

            for (int idx = 0; idx < 4; idx++) begin
              if (idx < (pipe_width_i >> 3) && (Q.data_k[1][idx] && Q.data[1][idx*8+:8] == COM)) begin
                flag = '1;
              end
            end
            for (int idx = 0; idx < 4; idx++) begin
              if (idx >= byte_idx && (idx < (pipe_width_i >> 3)) && (flag == '0)) begin
                D.byte_cnt = '0;
                D.disable_scrambling[idx] = '0;
                D.stop_scrambling[idx] = '1;
              end
            end
          end

          // Stage 1: K Symbols. Only COM and SKP change the scrambling state.
          if (Q.data_k[1][byte_idx]) begin
            // A COM followed by SKP, in stage 1 or, for the last byte, in stage
            // 0, opens a SKP Ordered Set. Any other COM schedules the FFFFh
            // reload for the next byte, starts byte_cnt and disables scrambling
            // from the COM on.
            if (Q.data[1][byte_idx*8+:8] == COM) begin
              logic is_skp_os;
              is_skp_os = '0;
              if (byte_idx < (pipe_width_i >> 3) - 1) begin
                int next_idx;
                next_idx = (byte_idx == 3) ? 0 : byte_idx + 1;
                if (Q.data_k[1][next_idx] && Q.data[1][next_idx*8+:8] == SKP) is_skp_os = '1;
              end else begin
                if (Q.data_k[0][0] && Q.data[0][0+:8] == SKP) is_skp_os = '1;
              end
              
              if (!is_skp_os) begin
                D.scramble_reset[byte_idx+1] = '1;
                D.byte_cnt = (pipe_width_i >> 3) - (byte_idx);
                for (int d_idx = 0; d_idx < 4; d_idx++) begin
                  if (d_idx >= byte_idx) begin
                    D.disable_scrambling[d_idx] = '1;
                  end
                end
              end else begin
                D.skp_os[byte_idx] = '1;
                D.disable_scrambling[byte_idx] = '1;
                D.lfsr_in = lfsr_out[byte_idx];
              end
            end
            // A SKP is flagged and D.lfsr_in is taken at its byte position, not
            // at the end of the word: a SKP does not advance the LFSR (PCIe
            // Base Spec r2.1, §4.2.3).
            if (Q.data[1][byte_idx*8+:8] == SKP) begin
              D.skp_os[byte_idx]             = '1;
              D.disable_scrambling[byte_idx] = '1;
              D.lfsr_in                      = lfsr_out[byte_idx];
            end
          else if (Q.data[1][byte_idx*8+:8] == PAD_) begin
            // Nothing to do: PAD, like every K Symbol, is excluded from the XOR.
            end
          end

          // Stage 2 into stage 3: the XOR. The mask is the bit-reversed LFSR
          // value for this byte, so the data byte meets the reversed upper byte
          // of the LFSR (PCIe Base Spec r2.1, §C.1). While a SKP Ordered Set is
          // flagged, the stage-0 copy is used. A byte is XORed unless it is a K
          // Symbol or lies inside an Ordered Set without stop_scrambling.
          if (Q.skp_os == '0) begin
            lfsr_swapped[byte_idx] = ({<<{lfsr_out[byte_idx]}});
          end else begin
            lfsr_swapped[byte_idx] = ({<<{Q.lfsr_out[0][byte_idx]}});
          end

          D.data[NumPipelines-1][byte_idx*8+:8] = ((Q.disable_scrambling[byte_idx] == '0 || (
          Q.stop_scrambling[byte_idx]))) && !Q.data_k[NumPipelines-2][byte_idx]? 
      ( Q.data[NumPipelines-2][byte_idx*8+:8] ^ lfsr_swapped[byte_idx]): Q.data[NumPipelines-2][byte_idx*8+:8];
        end
      end
    end
  end

  assign data_out_o   = Q.data[NumPipelines-1];
  assign data_k_out_o = Q.data_k[NumPipelines-1];
  assign data_valid_o = Q.data_valid[NumPipelines-1];

endmodule
