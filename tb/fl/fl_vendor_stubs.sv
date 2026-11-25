// ---------------------------------------------------------------------------
// fl_vendor_stubs -- bench stand-ins for the vendor cells of pcie_fl_zcu102
//
// Author: Kourosh Ghahramani
// Silicon Systems Research Lab, University of Washington
//
// Purpose
//   Lets Verilator elaborate pcie_fl_zcu102 itself, the board top, with
//   nothing of its own logic replaced. Each module here has the ports of the
//   cell it stands for and the behaviour of AMD's simulation model of it:
//     IBUFDS, BUFG  the clock passes through
//     ila_fl        empty: an ILA drives nothing
//     vio_fl,       each probe_out holds its initial value until the bench
//     vio_rc        writes it, as the Hardware Manager's VIO console would
//   The probe widths and the initial values are those of
//   fpga/zcu102/ip_debug_fl.tcl; test_pcie_fl.py compares the two.
//
// Limitations
//   Bench only. A board build generates the real cores.
// ---------------------------------------------------------------------------
`timescale 1ns/1ps

module IBUFDS (
    input  wire I,
    input  wire IB,
    output wire O
);
  wire unused_ib = IB;
  assign O = I;
endmodule

module BUFG (
    input  wire I,
    output wire O
);
  assign O = I;
endmodule

module ila_fl (
    input wire clk,
    input wire [20:0] probe0,
    input wire [19:0] probe1,
    input wire [2:0] probe2,
    input wire [2:0] probe3,
    input wire [3:0] probe4,
    input wire [15:0] probe5,
    input wire [1:0] probe6,
    input wire probe7,
    input wire [15:0] probe8,
    input wire [1:0] probe9,
    input wire probe10,
    input wire [19:0] probe11,
    input wire probe12,
    input wire [19:0] probe13,
    input wire probe14,
    input wire [7:0] probe15,
    input wire [7:0] probe16,
    input wire [39:0] probe17,
    input wire probe18
);
endmodule

module vio_fl (
    input wire clk,
    input wire [14:0] probe_in0,
    input wire [15:0] probe_in1,
    input wire [15:0] probe_in2,
    input wire [20:0] probe_in3,
    input wire [19:0] probe_in4,
    input wire [5:0] probe_in5,
    input wire [15:0] probe_in6,
    input wire [15:0] probe_in7,
    input wire [15:0] probe_in8,
    input wire [15:0] probe_in9,
    input wire [15:0] probe_in10,
    input wire [15:0] probe_in11,
    input wire [15:0] probe_in12,
    input wire [15:0] probe_in13,
    input wire [15:0] probe_in14,
    input wire [15:0] probe_in15,
    input wire [15:0] probe_in16,
    input wire [15:0] probe_in17,
    input wire [15:0] probe_in18,
    input wire [15:0] probe_in19,
    input wire [15:0] probe_in20,
    input wire [15:0] probe_in21,
    input wire [15:0] probe_in22,
    input wire [15:0] probe_in23,
    input wire [15:0] probe_in24,
    input wire [31:0] probe_in25,
    input wire [31:0] probe_in26,
    input wire [31:0] probe_in27,
    input wire [31:0] probe_in28,
    input wire [31:0] probe_in29,
    input wire [31:0] probe_in30,
    input wire [31:0] probe_in31,
    input wire [31:0] probe_in32,
    input wire [7:0] probe_in33,
    output logic probe_out0 = 1'b0,
    output logic probe_out1 = 1'b0,
    output logic probe_out2 = 1'b1,
    output logic probe_out3 = 1'b0,
    output logic [7:0] probe_out4 = 8'h00,
    output logic [1:0] probe_out5 = 2'b00,
    output logic [7:0] probe_out6 = 8'h00,
    output logic [3:0] probe_out7 = 4'h0,
    output logic probe_out8 = 1'b0,
    output logic probe_out9 = 1'b0
);
endmodule

module vio_rc (
    input wire clk,
    input wire [3:0] probe_in0,
    input wire [15:0] probe_in1,
    input wire [9:0] probe_in2,
    input wire [3:0] probe_in3,
    input wire [3:0] probe_in4,
    input wire [15:0] probe_in5,
    input wire [15:0] probe_in6,
    input wire [15:0] probe_in7,
    input wire [7:0] probe_in8,
    input wire [3:0] probe_in9,
    input wire [23:0] probe_in10,
    input wire [255:0] probe_in11,
    input wire [127:0] probe_in12,
    input wire [255:0] probe_in13,
    input wire [127:0] probe_in14,
    input wire [6:0] probe_in15,
    input wire [15:0] probe_in16,
    input wire [15:0] probe_in17,
    input wire [15:0] probe_in18,
    input wire [7:0] probe_in19,
    input wire [3:0] probe_in20,
    input wire [23:0] probe_in21,
    input wire [255:0] probe_in22,
    input wire [127:0] probe_in23,
    input wire [255:0] probe_in24,
    input wire [127:0] probe_in25,
    input wire [9:0] probe_in26,
    input wire [3:0] probe_in27,
    input wire [18:0] probe_in28,
    input wire [35:0] probe_in29,
    input wire [21:0] probe_in30,
    output logic probe_out0 = 1'b1,
    output logic probe_out1 = 1'b1,
    output logic probe_out2 = 1'b0,
    output logic [7:0] probe_out3 = 8'h00,
    output logic probe_out4 = 1'b1,
    output logic probe_out5 = 1'b0
);
endmodule
