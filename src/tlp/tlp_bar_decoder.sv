// ---------------------------------------------------------------------------
// tlp_bar_decoder -- decodes a Memory Request address against BAR windows
//
// Purpose
//   Decides which of BAR_COUNT address windows a received Memory Request
//   falls in. A window matches when it is enabled, memory_enable_i is set,
//   and both the first and the last byte of the request lie inside it. A
//   request that matches more than one window is reported on overlap_o and
//   is not a hit. tlp_layer reports a Memory Request that is not a hit as
//   unsupported.
//
// Interfaces
//   Request  address_i, length_bytes_i: the request's start address and its
//            length in bytes. A length of 0 decodes the start address alone.
//   Control  memory_enable_i: when 0, no window matches.
//   Result   hit_o: exactly one window matches. overlap_o: more than one does.
//            bar_o, offset_o: the lowest-numbered matching window, and
//            address_i minus that window's base, also on an overlap; both
//            are 0 when no window matches.
//
// Clock and reset
//   None; the module is combinational.
//
// Limitations
//   The windows are parameters, not BAR registers. offset_o is the offset
//   into the window only when the window's BAR_BASE is aligned to its
//   BAR_MASK.
//
// References
//   PCIe Base Spec r2.1, §2.2.4.1
// ---------------------------------------------------------------------------
`timescale 1ns/1ps
module tlp_bar_decoder #(
    // Window i is bits [64i+63:64i] of BAR_BASE and BAR_MASK and bit i of
    // BAR_ENABLE. The default is one enabled 4 KB window at address 0.
    parameter int BAR_COUNT = 2,
    parameter logic [BAR_COUNT*64-1:0] BAR_BASE = '0,
    parameter logic [BAR_COUNT*64-1:0] BAR_MASK = {{(BAR_COUNT-1){64'd0}}, 64'hffff_ffff_ffff_f000},
    parameter logic [BAR_COUNT-1:0] BAR_ENABLE = {{(BAR_COUNT-1){1'b0}}, 1'b1}
) (
    input  logic [63:0] address_i,
    input  logic [12:0] length_bytes_i,
    input  logic        memory_enable_i,
    output logic        hit_o,
    output logic        overlap_o,
    output logic [((BAR_COUNT <= 1) ? 1 : $clog2(BAR_COUNT))-1:0] bar_o,
    output logic [63:0] offset_o
);

  integer index;
  logic [63:0] base;
  logic [63:0] mask;
  logic [64:0] end_address;
  logic start_match;
  logic end_match;
  integer match_count;

  always_comb begin
    hit_o    = 1'b0;
    overlap_o = 1'b0;
    bar_o    = '0;
    offset_o = '0;
    match_count = 0;
    end_address = {1'b0, address_i} +
                  (length_bytes_i == 0 ? 65'd0 : {52'd0, length_bytes_i} - 1'b1);
    for (index = 0; index < BAR_COUNT; index = index + 1) begin
      base = BAR_BASE[index*64 +: 64];
      mask = BAR_MASK[index*64 +: 64];
      // A 1 in BAR_MASK marks a compared address bit. With 1s in every bit
      // above the window, as in the default, all 64 address bits are decoded
      // and no address aliases into the window (PCIe Base Spec r2.1, §2.2.4.1).
      start_match = (address_i & mask) == (base & mask);
      // end_address[64] means the request runs past the top of the 64-bit
      // address space, which no window covers.
      end_match = !end_address[64] && ((end_address[63:0] & mask) == (base & mask));
      if (memory_enable_i && BAR_ENABLE[index] && start_match && end_match) begin
        match_count = match_count + 1;
        if (!hit_o) begin
          hit_o    = 1'b1;
          bar_o    = index[((BAR_COUNT <= 1) ? 1 : $clog2(BAR_COUNT))-1:0];
          offset_o = address_i - base;
        end
      end
    end
    // A request inside two windows has no single target, so it is not a hit.
    overlap_o = match_count > 1;
    if (overlap_o)
      hit_o = 1'b0;
  end

endmodule
