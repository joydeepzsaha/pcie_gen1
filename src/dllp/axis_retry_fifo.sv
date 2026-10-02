//==========================================================================
//
//  Morgan State University
//  Open Hardware Acceleration Lab (HAL-O)
//
//!  Project:   Open-Source PCIe Endpoint Controller.
//   File:      axis_retry_fifo.v
//! Author: Idris Somoye
//  Created:   <Date>
//
//! Description:
//! Module implements a retry management FIFO. Stores TLPs as axis frames.
//! Module resets read and write pointer after every frame allowing for retransmission
//! as long as data is not overwritten.
//
//
//  Project:
//    This file is part of the PCIe Gen1/Gen2 Endpoint Controller project.
//    Developed as an open-source, synthesizable Verilog RTL IP core, this
//    project provides FPGA designers and researchers with an educational
//    and extensible platform for high-speed interconnect design.
//
//  Institutional Acknowledgement:
//    - Project oversight and research guidance provided by the CEAMLS
//      (Center for Equitable AI & Machine Learning Systems) Director.
//
//  Notes:
//    - Compliant with PCIe Base Specification (Gen1: 2.5 GT/s,
//      Gen2: 5.0 GT/s).
//
//  License: MIT License
//
//==========================================================================

// ---------------------------------------------------------------------------
// axis_retry_fifo -- one retry buffer slot: stores one framed TLP for replay
//
// Purpose
//   Stores one AXI-Stream frame, a TLP with its sequence number and LCRC as
//   tlp2dllp emits it, and plays it back on request. The read pointer returns
//   to the first beat after every last beat, so the stored frame can be read
//   again until a new frame overwrites it. retry_transmit instantiates one
//   per retry slot.
//
// Interfaces
//   Write         s_axis_*: the frame to store. s_axis_tready is constant 1;
//                 retry_transmit enables s_axis_tvalid only for the slot that
//                 retry_management names in retry_index_o.
//   Read          m_axis_*: the stored frame, offered while a complete frame
//                 is held (frame_available).
//
// Clock and reset
//   clk_i only. rst_i is synchronous and active high and clears the memory;
//   it is dllp_transmit's reset, which pcie_datalink_layer also asserts while
//   the link is down.
//
// Limitations
//   MaxPktSize counts one beat per DW, so the depth assumes DATA_WIDTH = 32.
//   A frame longer than MaxPktSize beats is dropped and leaves the slot
//   empty. The first beat of a new frame discards the frame held.
//
// References
//   PCIe Base Spec r2.1, §3.5.2.1
// ---------------------------------------------------------------------------
module axis_retry_fifo
  import pcie_datalink_pkg::*;
#(
    parameter int DATA_WIDTH       = 32,
    parameter int STRB_WIDTH       = DATA_WIDTH / 8,
    parameter int KEEP_WIDTH       = STRB_WIDTH,
    parameter int USER_WIDTH       = 1,
    parameter int MAX_PAYLOAD_SIZE = 256
) (
    input logic clk_i,
    input logic rst_i,


    //! @virtualbus TLP_axis_inputs @dir in
    input  logic [DATA_WIDTH-1:0] s_axis_tdata,
    input  logic [KEEP_WIDTH-1:0] s_axis_tkeep,
    input  logic                  s_axis_tvalid,
    input  logic                  s_axis_tlast,
    input  logic [USER_WIDTH-1:0] s_axis_tuser,
    output logic                  s_axis_tready,
    //! @end
    //! @virtualbus TLP_axis_outputs @dir out
    output logic [DATA_WIDTH-1:0] m_axis_tdata,
    output logic [KEEP_WIDTH-1:0] m_axis_tkeep,
    output logic                  m_axis_tvalid,
    output logic                  m_axis_tlast,
    output logic [USER_WIDTH-1:0] m_axis_tuser,
    input  logic                  m_axis_tready
    //! @end
);

  localparam int MaxHdrSize = 4;
  // Payload and header DWs plus two beats: the 2-byte sequence number and the
  // 4-byte LCRC add six bytes to the TLP (tlp2dllp).
  localparam int MaxPktSize = ((MAX_PAYLOAD_SIZE + 3) >> 2) + MaxHdrSize + 2;
  localparam int PtrWidth = (MaxPktSize < 2) ? 1 : $clog2(MaxPktSize);

  // One stored beat. tvalid is stored with it and is 1 for every written beat.
  typedef struct packed {
    logic                  tvalid;
    logic [USER_WIDTH-1:0] tuser;
    logic                  tlast;
    logic [KEEP_WIDTH-1:0] tkeep;
    logic [DATA_WIDTH-1:0] tdata;
  } axis_tlp_pkt_t;


  typedef struct packed {
    logic [PtrWidth-1:0]            wr_ptr;
    logic [PtrWidth-1:0]            rd_ptr;
    axis_tlp_pkt_t [MaxPktSize-1:0] axis_mem;
    logic                           frame_available;
    logic                           dropping_frame;
  } retry_fifo_t;


  retry_fifo_t D;
  retry_fifo_t Q;
  always_ff @(posedge clk_i) begin : main_seq
    if (rst_i) begin
      Q <= '{default: 'd0};
    end else begin
      Q <= D;
    end
  end


  // The read side rewinds rd_ptr to 0 after each last beat, so every replay
  // starts at the frame's first beat. The write side never back-pressures:
  // slot choice belongs to retry_management, and an oversized frame is
  // dropped here (dropping_frame).
  always_comb begin : read_write_logic
    D             = Q;
    s_axis_tready = '1;
    m_axis_tdata  = '0;
    m_axis_tkeep  = '0;
    m_axis_tvalid = '0;
    m_axis_tlast  = '0;
    m_axis_tuser  = '0;

    if (Q.frame_available) begin
      m_axis_tvalid = Q.axis_mem[Q.rd_ptr].tvalid;
      m_axis_tdata  = Q.axis_mem[Q.rd_ptr].tdata;
      m_axis_tkeep  = Q.axis_mem[Q.rd_ptr].tkeep;
      m_axis_tlast  = Q.axis_mem[Q.rd_ptr].tlast;
      m_axis_tuser  = Q.axis_mem[Q.rd_ptr].tuser;
      if (m_axis_tready && m_axis_tvalid) begin
        if (m_axis_tlast) begin
          D.rd_ptr = '0;
        end else if (Q.rd_ptr == MaxPktSize - 1) begin
          // A stored frame can never legally continue past the memory boundary.
          D.rd_ptr          = '0;
          D.frame_available = '0;
        end else begin
          D.rd_ptr = Q.rd_ptr + 1'b1;
        end
      end
    end


    if (s_axis_tvalid && s_axis_tready) begin
      if (Q.dropping_frame) begin
        // Consume the remainder of an oversized/malformed frame without
        // allowing its pointer to wrap and overwrite the start of the buffer.
        if (s_axis_tlast) begin
          D.wr_ptr          = '0;
          D.dropping_frame  = '0;
          D.frame_available = '0;
        end
      end else begin
        if (Q.wr_ptr == '0) begin
          D.frame_available = '0;
        end
        D.axis_mem[Q.wr_ptr] = {
          s_axis_tvalid, s_axis_tuser, s_axis_tlast, s_axis_tkeep, s_axis_tdata
        };
        if (s_axis_tlast) begin
          D.wr_ptr          = '0;
          D.frame_available = '1;
        end else if (Q.wr_ptr == MaxPktSize - 1) begin
          D.wr_ptr          = '0;
          D.dropping_frame  = '1;
          D.frame_available = '0;
        end else begin
          D.wr_ptr = Q.wr_ptr + 1'b1;
        end
      end
    end
  end

endmodule
