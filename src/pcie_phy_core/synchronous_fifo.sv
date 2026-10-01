// MIT License
//
// Copyright (c) 2019 mcavoya
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
// synchronous_fifo -- one-byte clock-domain-crossing register with a
//                     busy/ready handshake
//
// Purpose
//   Carries one byte from clk_in to clk_out. A write stores din and raises
//   busy; busy, synchronised into clk_out, raises ready. A read drops ready;
//   ready, synchronised back into clk_in, drops busy, and the next byte may
//   be written. Despite its name it holds one byte, not a queue. No module in
//   the design instantiates it.
//
// Interfaces
//   Write side    clk_in, we, din, busy: we is taken whenever it is high, so
//                 a write while busy overwrites the held byte.
//   Read side     clk_out, re, dout, ready: dout is the clk_in register itself,
//                 read without a synchroniser.
//
// Clock and reset
//   clk_in and clk_out may be asynchronous. reset is active high and is sampled
//   separately by each clock, so it must last long enough for the slower one
//   to see it. The synchroniser registers rdy_q and bsy_q have no reset.
//
// Limitations
//   The stored byte is 8 bits whatever DATA_WIDTH is: din is truncated and
//   dout zero-extended. The edge detectors read bits 2:1 of rdy_q and bsy_q,
//   so DEPTH must be at least 3; a larger DEPTH adds unused bits, not stages.
// ---------------------------------------------------------------------------
`timescale 1ns / 1ps


module synchronous_fifo #(
    parameter int DEPTH = 3,
    parameter int DATA_WIDTH = 8
) (

    input reset,  // active high, sampled in each clock domain

    input clk_in,  // write clock
    input we,  // active-high write enable
    input [DATA_WIDTH-1:0] din,  // only bits 7:0 are stored
    output busy,  // active-high buffer full

    input clk_out,  // read clock
    input re,  // active-high read enable
    output [DATA_WIDTH-1:0] dout,  // the stored byte, zero-extended
    output ready  // active-high data ready flag
);

  // Written on clk_in and read as dout on clk_out without a synchroniser. A
  // writer that waits for busy to fall never changes it while ready is high.
  reg [7:0] data = 8'd0;
  always @(posedge clk_in) begin
    if (reset) data <= 8'd0;
    else if (we) data <= din;
  end
  assign dout = data;

  // Each flag crosses through a shift register clocked by the other side; an
  // edge seen at bits 2:1 of the synchronised copy is the event.
  reg rdy = 1'b0;
  reg bsy = 1'b0;
  reg [DEPTH-1:0] rdy_q = 3'd0;
  reg [DEPTH-1:0] bsy_q = 3'd0;

  // ready has fallen: the reader has taken the byte.
  always @(posedge clk_in) rdy_q <= {rdy_q[1:0], rdy};
  wire read_event = rdy_q[2:1] == 2'b10;

  always @(posedge clk_in) begin
    if (reset || read_event) bsy <= 1'b0;
    else if (we) bsy <= 1'b1;
  end

  // busy has risen: a new byte is in the register.
  always @(posedge clk_out) bsy_q <= {bsy_q[1:0], bsy};
  wire write_event = bsy_q[2:1] == 2'b01;

  always @(posedge clk_out) begin
    if (reset || re) rdy <= 1'b0;
    else if (write_event) rdy <= 1'b1;
  end

  assign ready = rdy;
  assign busy  = bsy;

endmodule
