// ---------------------------------------------------------------------------
// pcie_dllp_crc8 -- DLLP CRC-16 step over one byte, in reflected bit order
//
// Purpose
//   One combinational step of the DLLP CRC over one data byte. The register
//   is kept in reflected bit order: it shifts toward bit 0 with polynomial
//   D008h, the bit reverse of 100Bh, and data[0] enters first, the order
//   PCIe Base Spec r2.1, §3.4.1 requires. Seeded with FFFFh and stepped over
//   a DLLP's four bytes, the complemented register holds the two bytes of
//   that section's CRC field with no bit reversal: the byte sent first in
//   bits 7:0, the second in bits 15:8. pcie_datalink_crc chains four steps
//   for one 32-bit word.
//
// Interfaces
//   CRC in   crcIn: the reflected CRC register before the step.
//   Data in  data: one DLLP byte.
//   CRC out  crcOut: the reflected CRC register after the step.
//
// Clock and reset
//   None; the module is combinational.
//
// References
//   PCIe Base Spec r2.1, §3.4.1
// ---------------------------------------------------------------------------
`timescale 1ns/1ps

module pcie_dllp_crc8 (
    input  [15:0] crcIn,
    input  [ 7:0] data,
    output [15:0] crcOut
);
  integer i;
  // crc[0] is crcIn with the byte XORed into bits 7:0; crc[n] follows n of
  // the eight shifts, so crc[8] is the register after the byte.
  reg [15:0] crc [8:0];
  always @(*) begin
    crc[0] = crcIn ^ data;
    for (i = 0; i < 8; i = i+1) begin
      if (crc[i] & 1) begin
        crc[i+1] = (crc[i] >> 1) ^ 16'hD008;
      end else begin
        crc[i+1] = crc[i] >> 1;

      end
    end
  end
  assign crcOut = crc[8];

endmodule
