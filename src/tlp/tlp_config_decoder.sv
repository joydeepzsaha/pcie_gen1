// ---------------------------------------------------------------------------
// tlp_config_decoder -- decodes the target of a received Configuration Request
//
// Purpose
//   Decides whether a received header is a Configuration Request addressed to
//   this Function, says whether it is Type 1, and extracts the byte offset of
//   the register it accesses. tlp_layer reports a Configuration Request that
//   is not a hit as unsupported.
//
// Interfaces
//   Header    header_i: a parsed header. For a Configuration Request, address
//             bits [31:0] hold header DW2: Bus, Device and Function Numbers in
//             [31:16], Extended Register Number in [11:8], Register Number in
//             [7:2].
//   Identity  bus_number_i, device_number_i, function_number_i: this
//             Function's Bus, Device and Function Numbers.
//   Result    hit_o: a CfgRd0, CfgWr0, CfgRd1 or CfgWr1 whose Bus, Device and
//             Function Numbers all match. type_one_o: the header is Type 1.
//             register_offset_o: the byte offset of the addressed DW.
//
// Clock and reset
//   None; the module is combinational.
//
// Limitations
//   hit_o compares the Device Number of a Type 0 request, but a non-ARI
//   device must respond to every Type 0 Configuration Read whatever its
//   Device Number (PCIe Base Spec r2.1, §7.3.1). hit_o is also set for a
//   Type 1 request, which an Endpoint handles as an Unsupported Request
//   (§7.3.3); type_one_o lets the client do so.
//
// References
//   PCIe Base Spec r2.1, §2.2.7
//   PCIe Base Spec r2.1, §7.3.1
//   PCIe Base Spec r2.1, §7.3.2
//   PCIe Base Spec r2.1, §7.3.3
// ---------------------------------------------------------------------------
`timescale 1ns/1ps
module tlp_config_decoder
  import tlp_pkg::*;
(
    input  tlp_header_t header_i,
    input  logic [7:0]  bus_number_i,
    input  logic [4:0]  device_number_i,
    input  logic [2:0]  function_number_i,
    output logic        hit_o,
    output logic        type_one_o,
    output logic [11:0] register_offset_o
);

  always_comb begin
    type_one_o = header_i.tlp_type == TLP_TYPE_CFG1;
    // Extended Register Number and Register Number together, the Extended
    // Register Number as the more significant bits (PCIe Base Spec r2.1,
    // §7.3.2).
    register_offset_o = {header_i.address[11:2], 2'b00};
    hit_o = (header_i.tlp_type == TLP_TYPE_CFG0 ||
             header_i.tlp_type == TLP_TYPE_CFG1) &&
            header_i.address[31:24] == bus_number_i &&
            header_i.address[23:19] == device_number_i &&
            header_i.address[18:16] == function_number_i;
  end

endmodule
