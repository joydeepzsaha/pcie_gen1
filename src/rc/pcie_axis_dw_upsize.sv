// ---------------------------------------------------------------------------
// pcie_axis_dw_upsize -- AXI4-Stream width converter, 32 to 128 bits (4:1)
//
// Author: Kourosh Ghahramani
// Silicon Systems Research Lab, University of Washington
//
// Purpose
//   Packs up to four narrow Dword beats into one wide beat, lowest group
//   first. On tlast the partial wide beat is emitted at once with the tkeep
//   actually accumulated, without waiting for a fourth beat. The module knows
//   no PG213 descriptor layout: pcie_rc_if and pcie_cq_if push descriptor
//   Dwords and then payload Dwords, and the Dword-aligned layout of PG213 is
//   the plain concatenation of that stream.
//
// Interfaces
//   Input       s_axis_*: 32-bit Dword beats, one tkeep bit per byte.
//   Output      m_axis_*: 128-bit beats, one tkeep bit per byte; the callers
//               reduce tkeep to PG213's one bit per Dword.
//   Error       gearbox_error_o: one-cycle pulse in the cycle after an illegal
//               narrow beat is accepted.
//
// Clock and reset
//   clk_i only. rst_i is synchronous and active high, and discards a partly
//   assembled wide beat.
//
// Limitations
//   No tuser path. Only the 32-to-128 ratio is instantiated (pcie_rc_if,
//   pcie_cq_if) and tested (tb_pcie_axis_gearbox). s_axis_tready is low while
//   a wide beat waits in the output register, so a stream of full beats runs
//   at 80% of the narrow side's rate.
//
// References
//   None: this module implements no PCIe or PG213 rule.
// ---------------------------------------------------------------------------
`timescale 1ns/1ps
module pcie_axis_dw_upsize #(
    parameter int DATA_WIDTH_NARROW = 32,
    parameter int DATA_WIDTH_WIDE   = 128,
    parameter int KEEP_WIDTH_NARROW = DATA_WIDTH_NARROW / 8,
    parameter int KEEP_WIDTH_WIDE   = DATA_WIDTH_WIDE / 8,
    parameter int RATIO             = DATA_WIDTH_WIDE / DATA_WIDTH_NARROW
) (
    input  logic                         clk_i,
    input  logic                         rst_i,

    // ---- narrow input, Dword-serial ----------------------------------------
    input  logic [DATA_WIDTH_NARROW-1:0] s_axis_tdata,
    input  logic [KEEP_WIDTH_NARROW-1:0] s_axis_tkeep,
    input  logic                         s_axis_tvalid,
    input  logic                         s_axis_tlast,
    output logic                         s_axis_tready,

    // ---- wide output -------------------------------------------------------
    output logic [DATA_WIDTH_WIDE-1:0]   m_axis_tdata,
    output logic [KEEP_WIDTH_WIDE-1:0]   m_axis_tkeep,
    output logic                         m_axis_tvalid,
    output logic                         m_axis_tlast,
    input  logic                         m_axis_tready,

    // Pulses for one cycle after the acceptance of an illegal narrow beat (see
    // keep_illegal). Informational: the beat is still packed.
    output logic                         gearbox_error_o
);

  localparam int PHASE_WIDTH = $clog2(RATIO);

  // Accumulator: fills LSB group first.
  logic [DATA_WIDTH_WIDE-1:0] acc_data_r;
  logic [KEEP_WIDTH_WIDE-1:0] acc_keep_r;
  logic [PHASE_WIDTH-1:0]     phase_r;

  // Output holding register.
  logic [DATA_WIDTH_WIDE-1:0] out_data_r;
  logic [KEEP_WIDTH_WIDE-1:0] out_keep_r;
  logic                       out_last_r;
  logic                       out_valid_r;

  // A narrow beat is legal only if its tkeep is contiguous from bit 0 and
  // non-zero; anything short of full is legal only on the final beat, because
  // a partial Dword inside a packet would leave a hole in the packed word.
  function automatic logic keep_illegal(input logic [KEEP_WIDTH_NARROW-1:0] keep,
                                        input logic                        last);
    keep_illegal = (keep == '0) ||
                   ((keep & (keep + 1'b1)) != '0) ||
                   (!last && (keep != {KEEP_WIDTH_NARROW{1'b1}}));
  endfunction

  // State only, never m_axis_tready, so no combinational path runs from the
  // wide side's ready to the narrow side's. The cost is the one cycle per wide
  // beat in which the output register is full and no narrow beat is taken.
  assign s_axis_tready = !out_valid_r;

  assign m_axis_tvalid = out_valid_r;
  assign m_axis_tdata  = out_data_r;
  assign m_axis_tkeep  = out_keep_r;
  assign m_axis_tlast  = out_last_r;

  wire accept_beat = s_axis_tvalid && s_axis_tready;
  wire drain_beat  = m_axis_tvalid && m_axis_tready;
  // The accumulator completes on the fourth group, or early on tlast -- the
  // partial word goes out immediately, never waiting for a fourth beat.
  wire word_done   = accept_beat && (s_axis_tlast || (phase_r == PHASE_WIDTH'(RATIO-1)));

  // Accumulator contents with this beat merged in.
  logic [DATA_WIDTH_WIDE-1:0] merged_data;
  logic [KEEP_WIDTH_WIDE-1:0] merged_keep;
  always_comb begin
    merged_data = acc_data_r;
    merged_keep = acc_keep_r;
    merged_data[phase_r*DATA_WIDTH_NARROW +: DATA_WIDTH_NARROW] = s_axis_tdata;
    merged_keep[phase_r*KEEP_WIDTH_NARROW +: KEEP_WIDTH_NARROW] = s_axis_tkeep;
  end

  always_ff @(posedge clk_i) begin
    if (rst_i) begin
      // The accumulator and its keep both clear, so a half-assembled word from
      // before a mid-packet reset cannot leak into the next packet.
      acc_data_r      <= '0;
      acc_keep_r      <= '0;
      phase_r         <= '0;
      out_data_r      <= '0;
      out_keep_r      <= '0;
      out_last_r      <= 1'b0;
      out_valid_r     <= 1'b0;
      gearbox_error_o <= 1'b0;
    end else begin
      gearbox_error_o <= 1'b0;

      if (drain_beat) out_valid_r <= 1'b0;   // releases s_axis_tready next cycle

      if (accept_beat) begin
        // An illegal beat is still packed unchanged, so the fault stays
        // visible downstream instead of becoming a silent misalignment.
        if (keep_illegal(s_axis_tkeep, s_axis_tlast)) begin
          gearbox_error_o <= 1'b1;
          $warning("pcie_axis_dw_upsize: illegal tkeep 0x%0h (tlast=%0b); must be contiguous from bit 0, non-zero, and full unless final",
                   s_axis_tkeep, s_axis_tlast);
        end

        if (word_done) begin
          out_data_r  <= merged_data;
          out_keep_r  <= merged_keep;
          out_last_r  <= s_axis_tlast;
          out_valid_r <= 1'b1;
          // Clear the accumulator so the next word starts from group 0 and, if
          // it is short, carries no keep bits left over from this one.
          acc_data_r  <= '0;
          acc_keep_r  <= '0;
          phase_r     <= '0;
        end else begin
          acc_data_r <= merged_data;
          acc_keep_r <= merged_keep;
          phase_r    <= phase_r + 1'b1;
        end
      end
    end
  end

endmodule
