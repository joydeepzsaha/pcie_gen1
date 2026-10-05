////////////////////////////////////////////////////////////////////////
//
// Copyright (C) 2020 Akilesh Kannan <akileshkannan@gmail.com>
//
// File: lfsr.v
// Modified: 2020-07-15
// Description: Linear Feedback Shift Register (32-bit)
//              Used a pseudo-random number generator
//
// License: MIT
//
////////////////////////////////////////////////////////////////////////
// MIT License
//
// Copyright (c) 2019 Akilesh Kannan
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

// ---------------------------------------------------------------------------
// lfsr -- 32-bit LFSR pseudo-random generator, not used by the design
//
// Purpose
//   Shifts LFSRregister left by one bit on every rising clk edge, then loads
//   bit 0 with the XOR of bits 31, 29, 25 and 24 of the shifted value: both
//   assignments are blocking, so the second reads the result of the first.
//   The only instance is in synchronous_lifo, which nothing in the design
//   instantiates. phy_transmit.core and synth/endpoint.tcl list the file.
//
// Interfaces
//   Parameter  seed: the initial register value.
//   Output     LFSRregister: the register itself.
//
// Clock and reset
//   clk only. There is no reset: an initial block loads seed.
// ---------------------------------------------------------------------------
module lfsr #(
    parameter int seed = 32'b1
) (
    output reg [31:0] LFSRregister,
    input clk
);

  // initially register will contain seed value
  initial begin
    LFSRregister = seed;
  end

  // at edge of each clock pulse, shift and XOR required bits
  always @(posedge clk) begin
    LFSRregister = LFSRregister << 1;
    LFSRregister[0] = LFSRregister[31] ^ LFSRregister[29] ^ LFSRregister[25] ^ LFSRregister[24];
  end
endmodule
