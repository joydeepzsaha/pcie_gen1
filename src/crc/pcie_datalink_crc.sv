// vim: ts=4 sw=4 expandtab

// THIS IS GENERATED VERILOG CODE.
// https://bues.ch/h/crcgen
// 
// This code is Public Domain.
// Permission to use, copy, modify, and/or distribute this software for any
// purpose with or without fee is hereby granted.
// 
// THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
// WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
// MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
// SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES WHATSOEVER
// RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT,
// NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE
// USE OR PERFORMANCE OF THIS SOFTWARE.

`ifndef CRC_V_
`define CRC_V_

// CRC polynomial coefficients: x^16 + x^12 + x^11 + x^10 + x^8 + x^7 + x^5 + x^4 + x^2 + x + 1
//                              0x1DB7 (hex)
// CRC width:                   16 bits
// CRC shift direction:         left (big endian)
// Input word width:            32 bits

// ---------------------------------------------------------------------------
// pcie_datalink_crc -- DLLP CRC-16 over one 32-bit word
//
// Purpose
//   Advances the DLLP CRC over four bytes, data[7:0] first, by chaining four
//   pcie_dllp_crc8 byte steps. The register is in reflected bit order
//   (polynomial D008h, the bit reverse of 100Bh). Seeded with FFFFh over the
//   four bytes of a DLLP, crcOut complemented holds the two bytes of the
//   DLLP's CRC field with no bit reversal, the byte sent first in bits 7:0
//   (PCIe Base Spec r2.1, §3.4.1). The polynomial and shift direction in the
//   generator comment above do not describe this body.
//
// Interfaces
//   CRC in   crcIn: the reflected CRC register before the word. dllp_handler,
//            dllp_fc_update and pcie_flow_ctrl_init all tie it to ones, the
//            FFFFh seed.
//   Data in  data: four DLLP bytes, the first in data[7:0].
//   CRC out  crcOut: the reflected CRC register after the word, not
//            complemented.
//
// Clock and reset
//   None; the module is combinational.
//
// References
//   PCIe Base Spec r2.1, §3.4.1
// ---------------------------------------------------------------------------
`timescale 1ns/1ps

module pcie_datalink_crc (
    input  [15:0] crcIn,
    input  [31:0] data,
    output [15:0] crcOut
);

  logic [15:0] crc0;
  logic [15:0] crc1;
  logic [15:0] crc2;
  logic [15:0] crc3;
  reg [15:0] crc4;

  pcie_dllp_crc8 crc_inst_0 (
      .crcIn (crcIn),
      .data  (data[7:0]),
      .crcOut(crc0)
  );

  pcie_dllp_crc8 crc_inst_1 (
      .crcIn (crc0),
      .data  (data[15:8]),
      .crcOut(crc1)
  );

  pcie_dllp_crc8 crc_inst_2 (
      .crcIn (crc1),
      .data  (data[23:16]),
      .crcOut(crc2)
  );

  pcie_dllp_crc8 crc_inst_3 (
      .crcIn (crc2),
      .data  (data[31:24]),
      .crcOut(crc3)
  );

  // Not bit-reversed: in reflected order each byte of the register already
  // has the bit order of the CRC field (PCIe Base Spec r2.1, §3.4.1).
  assign crcOut = crc3;

endmodule

`endif  // CRC_V_
