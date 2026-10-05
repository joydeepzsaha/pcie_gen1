// ---------------------------------------------------------------------------
// pcie_axis_dw_downsize -- AXI4-Stream width converter, 128 to 32 bits (4:1)
//
// Author: Kourosh Ghahramani
// Silicon Systems Research Lab, University of Washington
//
// Purpose
//   Serializes each wide beat into narrow Dword beats, lowest group first:
//   one narrow beat for every 4-byte group up to the highest group with a set
//   tkeep bit, the last of them carrying its partial tkeep unchanged. The
//   module knows no PG213 descriptor layout. pcie_cc_if passes whole CC
//   packets through it and counts the descriptor Dwords itself; pcie_rq_if
//   decodes the RQ descriptor from the wide beat and passes payload beats only.
//
// Interfaces
//   Input       s_axis_*: 128-bit beats, one tkeep bit per byte.
//   Output      m_axis_*: 32-bit Dword beats, one tkeep bit per byte, as
//               tlp_requester's command_keep_i expects.
//   Error       gearbox_error_o: one-cycle pulse in the cycle after an illegal
//               wide beat is accepted.
//
// Clock and reset
//   clk_i only. rst_i is synchronous and active high, and discards a wide beat
//   that is still being serialized.
//
// Limitations
//   No tuser path. Only the 128-to-32 ratio is instantiated (pcie_rq_if,
//   pcie_cc_if) and tested (tb_pcie_axis_gearbox). A full wide beat takes five
//   cycles, four narrow beats and one reload, so a stream of full beats runs at
//   80% of the narrow side's rate.
//
// References
//   None: this module implements no PCIe or PG213 rule.
// ---------------------------------------------------------------------------
`timescale 1ns/1ps
module pcie_axis_dw_downsize #(
    parameter int DATA_WIDTH_WIDE   = 128,
    parameter int DATA_WIDTH_NARROW = 32,
    parameter int KEEP_WIDTH_WIDE   = DATA_WIDTH_WIDE / 8,
    parameter int KEEP_WIDTH_NARROW = DATA_WIDTH_NARROW / 8,
    parameter int RATIO             = DATA_WIDTH_WIDE / DATA_WIDTH_NARROW
) (
    input  logic                         clk_i,
    input  logic                         rst_i,

    // ---- wide input --------------------------------------------------------
    input  logic [DATA_WIDTH_WIDE-1:0]   s_axis_tdata,
    input  logic [KEEP_WIDTH_WIDE-1:0]   s_axis_tkeep,
    input  logic                         s_axis_tvalid,
    input  logic                         s_axis_tlast,
    output logic                         s_axis_tready,

    // ---- narrow output, Dword-serial ---------------------------------------
    output logic [DATA_WIDTH_NARROW-1:0] m_axis_tdata,
    output logic [KEEP_WIDTH_NARROW-1:0] m_axis_tkeep,
    output logic                         m_axis_tvalid,
    output logic                         m_axis_tlast,
    input  logic                         m_axis_tready,

    // Pulses for one cycle after the acceptance of a wide beat whose tkeep is
    // zero or not contiguous from bit 0. Informational: the beat is still
    // serialized.
    output logic                         gearbox_error_o
);

  localparam int PHASE_WIDTH = $clog2(RATIO);

  logic [DATA_WIDTH_WIDE-1:0] data_r;
  logic [KEEP_WIDTH_WIDE-1:0] keep_r;
  logic                       last_r;
  logic                       busy_r;
  logic [PHASE_WIDTH-1:0]     phase_r;   // group currently presented
  logic [PHASE_WIDTH:0]       beats_r;   // total groups this wide beat spans

  // Index of the highest non-empty KEEP_WIDTH_NARROW-byte group, plus one.
  // For the legal (contiguous-from-LSB) patterns this is ceil(popcount/4); for
  // an illegal pattern it still spans every group holding a valid byte, so no
  // byte is silently dropped.
  function automatic logic [PHASE_WIDTH:0] group_span(input logic [KEEP_WIDTH_WIDE-1:0] keep);
    group_span = '0;
    for (int g = 0; g < RATIO; g++)
      if (|keep[g*KEEP_WIDTH_NARROW +: KEEP_WIDTH_NARROW])
        group_span = (PHASE_WIDTH+1)'(g + 1);
  endfunction

  // Legal tkeep is contiguous from bit 0, i.e. (1<<n)-1 for n > 0.
  function automatic logic keep_illegal(input logic [KEEP_WIDTH_WIDE-1:0] keep);
    keep_illegal = (keep == '0) || ((keep & (keep + 1'b1)) != '0);
  endfunction

  wire load_beat = s_axis_tvalid && s_axis_tready;
  wire last_group = busy_r && (({1'b0, phase_r} + 1'b1) >= beats_r);
  wire drain_beat = m_axis_tvalid && m_axis_tready;

  // State only, never m_axis_tready, so no combinational path runs from the
  // narrow side's ready to the wide side's. In pcie_rq_if that ready comes
  // from tlp_requester's command_data_ready_o, itself combinational.
  assign s_axis_tready = !busy_r;

  assign m_axis_tvalid = busy_r;
  assign m_axis_tdata  = data_r[phase_r*DATA_WIDTH_NARROW +: DATA_WIDTH_NARROW];
  assign m_axis_tkeep  = keep_r[phase_r*KEEP_WIDTH_NARROW +: KEEP_WIDTH_NARROW];
  // tlast only on the final narrow beat derived from the wide beat that carried it.
  assign m_axis_tlast  = last_r && last_group;

  always_ff @(posedge clk_i) begin
    if (rst_i) begin
      // busy_r low marks the holding register empty, so after a mid-packet
      // reset no fragment of the old beat reaches the next packet.
      busy_r          <= 1'b0;
      phase_r         <= '0;
      beats_r         <= '0;
      data_r          <= '0;
      keep_r          <= '0;
      last_r          <= 1'b0;
      gearbox_error_o <= 1'b0;
    end else begin
      gearbox_error_o <= 1'b0;

      if (load_beat) begin
        data_r  <= s_axis_tdata;
        keep_r  <= s_axis_tkeep;
        last_r  <= s_axis_tlast;
        beats_r <= group_span(s_axis_tkeep);
        phase_r <= '0;
        busy_r  <= 1'b1;
        // An illegal beat is serialized unchanged, including any zero-keep
        // group below the highest valid one, and a zero tkeep gives one
        // zero-keep narrow beat. No byte is moved or dropped.
        if (keep_illegal(s_axis_tkeep)) begin
          gearbox_error_o <= 1'b1;
          $warning("pcie_axis_dw_downsize: illegal tkeep 0x%0h (must be contiguous from bit 0 and non-zero)",
                   s_axis_tkeep);
        end
      end else if (drain_beat) begin
        if (last_group) busy_r  <= 1'b0;   // releases s_axis_tready next cycle
        else            phase_r <= phase_r + 1'b1;
      end
    end
  end

endmodule
