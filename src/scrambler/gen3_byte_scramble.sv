// ---------------------------------------------------------------------------
// gen3_byte_scramble -- Gen3 scrambler per-byte LFSR step, not integrated
//
// Purpose
//   Not integrated: nothing in the design instantiates this module. Its only
//   user is gen3_scramble, which is itself not instantiated, and
//   scrambler.core and synth/endpoint.tcl leave both files out. It belongs to
//   Gen3 (128b/130b) scrambling; PCIe Base Spec r2.1 defines only the 8b/10b
//   code. The module maps the 23-bit LFSR value in lfsr_r[22:0] to the value
//   for the next byte, combinationally. The equations are those of
//   calc_next_lfsr in PCIe Base Spec r3.0, §C.2: eight serial shifts of the
//   LFSR G(X) = X^23 + X^21 + X^16 + X^8 + X^5 + X^2 + 1 of §4.2.2.4.
//   gen3_scramble chains four instances, one per byte position.
//
// Interfaces
//   LFSR     lfsr_r: the value for this byte; lfsr_out: the value for the next
//            byte, with bit 23 at 0.
//   Control  disable_lfsr_advance: passes lfsr_r through unchanged, bit 23
//            included.
//
// Clock and reset
//   None: the module is combinational.
//
// References
//   PCIe Base Spec r3.0, §4.2.2.4
//   PCIe Base Spec r3.0, §C.2
// ---------------------------------------------------------------------------
module gen3_byte_scramble (
    input  logic        disable_lfsr_advance,
    input  logic [23:0] lfsr_r,
    output logic [23:0] lfsr_out
);


  always_comb begin : lfsr_out_computation
    lfsr_out = '0;
    if (disable_lfsr_advance) begin
      lfsr_out = lfsr_r;
    end else begin
      lfsr_out[0]  = lfsr_r[15] ^ lfsr_r[17] ^ lfsr_r[19] ^ lfsr_r[21] ^ lfsr_r[22];
      lfsr_out[1]  = lfsr_r[16] ^ lfsr_r[18] ^ lfsr_r[20] ^ lfsr_r[22];
      lfsr_out[2]  = lfsr_r[15] ^ lfsr_r[22];
      lfsr_out[3]  = lfsr_r[16];
      lfsr_out[4]  = lfsr_r[17];
      lfsr_out[5]  = lfsr_r[15] ^ lfsr_r[17] ^ lfsr_r[18] ^ lfsr_r[19] ^ lfsr_r[21] ^ lfsr_r[22];
      lfsr_out[6]  = lfsr_r[16] ^ lfsr_r[18] ^ lfsr_r[19] ^ lfsr_r[20] ^ lfsr_r[22];
      lfsr_out[7]  = lfsr_r[17] ^ lfsr_r[19] ^ lfsr_r[20] ^ lfsr_r[21];
      lfsr_out[8]  = lfsr_r[0] ^ lfsr_r[15] ^ lfsr_r[17] ^ lfsr_r[18] ^ lfsr_r[19] ^ lfsr_r[20];
      lfsr_out[9]  = lfsr_r[1] ^ lfsr_r[16] ^ lfsr_r[18] ^ lfsr_r[19] ^ lfsr_r[20] ^ lfsr_r[21];
      lfsr_out[10] = lfsr_r[2] ^ lfsr_r[17] ^ lfsr_r[19] ^ lfsr_r[20] ^ lfsr_r[21] ^ lfsr_r[22];
      lfsr_out[11] = lfsr_r[3] ^ lfsr_r[18] ^ lfsr_r[20] ^ lfsr_r[21] ^ lfsr_r[22];
      lfsr_out[12] = lfsr_r[4] ^ lfsr_r[19] ^ lfsr_r[21] ^ lfsr_r[22];
      lfsr_out[13] = lfsr_r[5] ^ lfsr_r[20] ^ lfsr_r[22];
      lfsr_out[14] = lfsr_r[6] ^ lfsr_r[21];
      lfsr_out[15] = lfsr_r[7] ^ lfsr_r[22];
      lfsr_out[16] = lfsr_r[8] ^ lfsr_r[15] ^ lfsr_r[17] ^ lfsr_r[19] ^ lfsr_r[21] ^ lfsr_r[22];
      lfsr_out[17] = lfsr_r[9] ^ lfsr_r[16] ^ lfsr_r[18] ^ lfsr_r[20] ^ lfsr_r[22];
      lfsr_out[18] = lfsr_r[10] ^ lfsr_r[17] ^ lfsr_r[19] ^ lfsr_r[21];
      lfsr_out[19] = lfsr_r[11] ^ lfsr_r[18] ^ lfsr_r[20] ^ lfsr_r[22];
      lfsr_out[20] = lfsr_r[12] ^ lfsr_r[19] ^ lfsr_r[21];
      lfsr_out[21] = lfsr_r[13] ^ lfsr_r[15] ^ lfsr_r[17] ^ lfsr_r[19] ^ lfsr_r[20] ^ lfsr_r[21];
      lfsr_out[22] = lfsr_r[14] ^ lfsr_r[16] ^ lfsr_r[18] ^ lfsr_r[20] ^ lfsr_r[21] ^ lfsr_r[22];
    end
  end



endmodule
