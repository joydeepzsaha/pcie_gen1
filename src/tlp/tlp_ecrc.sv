// ---------------------------------------------------------------------------
// tlp_ecrc -- running ECRC (CRC-32) over a TLP, one 32-bit beat at a time
//
// Purpose
//   Accumulates the 32-bit ECRC of a TLP: a reflected CRC-32 (polynomial
//   EDB8_8320h, the bit reverse of 04C1_1DB7h) seeded with FFFF_FFFFh, over
//   the bytes whose keep_i bit is set, lane 0 first and bit 0 of each byte
//   first (tlp_crc32_dw in tlp_pkg). The result is complemented.
//   tlp_generator uses it to append the TLP Digest and tlp_parser to check one.
//
// Interfaces
//   Control  start_i: restart from the seed; the beat on data_i in the same
//            cycle is the first one included. finish_i: the beat in this
//            cycle is the last one.
//   Data     data_i, keep_i, data_valid_i: one beat, included only when
//            data_valid_i is set.
//   Result   ecrc_o: the complemented CRC, registered when finish_i and
//            data_valid_i are both set. ecrc_valid_o: set with it, cleared
//            by the next start_i.
//
// Clock and reset
//   clk_i only. rst_i is synchronous and active high.
//
// Limitations
//   The variant bits, Type[0] and EP, enter the CRC as they are rather than
//   as 1b (PCIe Base Spec r2.1, §2.7.1).
//
// References
//   PCIe Base Spec r2.1, §2.7.1
// ---------------------------------------------------------------------------
`timescale 1ns/1ps
module tlp_ecrc
  import tlp_pkg::*;
(
    input  logic        clk_i,
    input  logic        rst_i,
    input  logic        start_i,
    input  logic [31:0] data_i,
    input  logic [3:0]  keep_i,
    input  logic        data_valid_i,
    input  logic        finish_i,
    output logic [31:0] ecrc_o,
    output logic        ecrc_valid_o
);

  logic [31:0] crc_r;
  logic [31:0] base_crc;
  logic [31:0] next_crc;

  // start_i takes the seed in place of crc_r, so the beat that starts a TLP
  // is folded in the same cycle.
  always_comb begin
    base_crc = start_i ? 32'hffff_ffff : crc_r;
    next_crc = data_valid_i ? tlp_crc32_dw(base_crc, data_i, keep_i) : base_crc;
  end

  always_ff @(posedge clk_i) begin
    if (rst_i) begin
      crc_r        <= 32'hffff_ffff;
      ecrc_o       <= '0;
      ecrc_valid_o <= 1'b0;
    end else begin
      if (start_i)
        ecrc_valid_o <= 1'b0;
      if (start_i || data_valid_i)
        crc_r <= next_crc;
      if (finish_i && data_valid_i) begin
        ecrc_o       <= ~next_crc;
        ecrc_valid_o <= 1'b1;
      end
    end
  end

endmodule
