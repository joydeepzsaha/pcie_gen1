// ---------------------------------------------------------------------------
// Crc16Gen -- DLLP CRC-16 step over 16 data bits, with CRC field mapping
//
// Purpose
//   One combinational step of the DLLP CRC (polynomial 100Bh, register
//   shifted toward bit 15) over two data bytes. The earlier byte is
//   Data[15:8], and each byte enters from bit 0 to bit 7, the order of PCIe
//   Base Spec r2.1, §3.4.1. Crc and CombCrc map a register value into the
//   16-bit CRC field: each byte bit-reversed, as in that section's bit
//   mapping, and complemented when Complement is 1. Seeded with FFFFh,
//   stepped over a DLLP's four bytes and with Complement at 1, CombCrc[15:8]
//   is the CRC byte sent first and CombCrc[7:0] the second.
//
// Interfaces
//   Data in   Data: two DLLP bytes, the earlier one in Data[15:8].
//             ShiftIn: the CRC register before the step, held by the caller.
//             Complement: 1 complements Crc and CombCrc.
//   CRC out   ShiftChain: the CRC register after the step.
//             Crc: the CRC field of ShiftIn.
//             CombCrc: the CRC field of ShiftChain.
//
// Clock and reset
//   None; the module is combinational.
//
// Limitations
//   No module instantiates Crc16Gen; crc.core and synth/endpoint.tcl only
//   compile it. The DLLP CRC in use is pcie_datalink_crc.
//
// References
//   PCIe Base Spec r2.1, §3.4.1
// ---------------------------------------------------------------------------
`timescale 1ns/1ps

module Crc16Gen (
    input logic [15:0] Data,
    input logic Complement,
    input logic [15:0] ShiftIn,
    output logic [15:0] ShiftChain,
    output logic [15:0] Crc,
    output logic [15:0] CombCrc
);

  // The CRC field of the caller's register, ShiftIn
  assign Crc             = {16{Complement}} ^ {ShiftIn[8],  ShiftIn[9],  ShiftIn[10], ShiftIn[11],
                                             ShiftIn[12], ShiftIn[13], ShiftIn[14], ShiftIn[15],
                                             ShiftIn[0],  ShiftIn[1],  ShiftIn[2],  ShiftIn[3],
                                             ShiftIn[4],  ShiftIn[5],  ShiftIn[6],  ShiftIn[7]};

  // The CRC field of the register after this step, ShiftChain
  assign CombCrc         = {16{Complement}} ^ {ShiftChain[8],  ShiftChain[9],  ShiftChain[10], ShiftChain[11],
                                             ShiftChain[12], ShiftChain[13], ShiftChain[14], ShiftChain[15],
                                             ShiftChain[0],  ShiftChain[1],  ShiftChain[2],  ShiftChain[3],
                                             ShiftChain[4],  ShiftChain[5],  ShiftChain[6],  ShiftChain[7]};

  // Bit 0 of each byte enters first, and the register shifts toward bit 15,
  // so each byte is bit-reversed within its lane: bit 0 lands on the lane's
  // high bit.
  logic [15:0] DtXorShift = { Data[8],  Data[9],  Data[10], Data[11], Data[12], Data[13], Data[14], Data[15],
                           Data[0],  Data[1],  Data[2],  Data[3],  Data[4],  Data[5],  Data[6],  Data[7]
                          } ^ ShiftIn;

  // Sixteen shifts with polynomial 100Bh, as one parity per register bit
  assign ShiftChain[00] = ^(DtXorShift & 16'hb111);
  assign ShiftChain[01] = ^(DtXorShift & 16'hd333);
  assign ShiftChain[02] = ^(DtXorShift & 16'ha666);
  assign ShiftChain[03] = ^(DtXorShift & 16'hfddd);
  assign ShiftChain[04] = ^(DtXorShift & 16'hfbba);
  assign ShiftChain[05] = ^(DtXorShift & 16'hf774);
  assign ShiftChain[06] = ^(DtXorShift & 16'heee8);
  assign ShiftChain[07] = ^(DtXorShift & 16'hddd0);
  assign ShiftChain[08] = ^(DtXorShift & 16'hbba0);
  assign ShiftChain[09] = ^(DtXorShift & 16'h7740);
  assign ShiftChain[10] = ^(DtXorShift & 16'hee80);
  assign ShiftChain[11] = ^(DtXorShift & 16'hdd00);
  assign ShiftChain[12] = ^(DtXorShift & 16'h0b11);
  assign ShiftChain[13] = ^(DtXorShift & 16'h1622);
  assign ShiftChain[14] = ^(DtXorShift & 16'h2c44);
  assign ShiftChain[15] = ^(DtXorShift & 16'h5888);

endmodule
