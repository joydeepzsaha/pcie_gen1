// ---------------------------------------------------------------------------
// tlp_validator -- formation checks on a received TLP header
//
// Purpose
//   Checks a parsed header against the formation rules this design applies
//   and reports the first rule it breaks. tlp_parser discards a TLP whose
//   header fails and reports it as malformed; tlp_classifier reports such a
//   header as unsupported.
//
// Interfaces
//   Header  header_i: a parsed header. length_dw is the decoded DW count, 0
//           only for a Completion without data (tlp_parser).
//   Result  valid_o: every check passes. error_o: TLP_ERR_NONE when valid_o
//           is set, else the first failed check: TLP_ERR_BAD_FMT_TYPE,
//           TLP_ERR_BAD_ADDRESS_FORMAT, TLP_ERR_BAD_LENGTH or
//           TLP_ERR_BAD_BYTE_ENABLE.
//
// Clock and reset
//   None; the module is combinational.
//
// Limitations
//   Only Memory Read and Write, I/O, Configuration and Completion types pass.
//   MRdLk, Message and AtomicOp Requests fail as TLP_ERR_BAD_FMT_TYPE,
//   although they are defined types: under PCIe Base Spec r2.1, §2.3.1 a
//   Request type the device does not support is an Unsupported Request. A
//   Cpl or CplLk with a non-zero Length field fails, although that field is
//   Reserved there and §2.3 has a Receiver ignore Reserved fields. The Byte
//   Enable checks also apply to an MRd with TH set, whose Byte Enable fields
//   carry ST[7:0] instead (§2.2.5).
//
// References
//   PCIe Base Spec r2.1, §2.2.1
//   PCIe Base Spec r2.1, §2.2.4.1
//   PCIe Base Spec r2.1, §2.2.5
//   PCIe Base Spec r2.1, §2.2.7
//   PCIe Base Spec r2.1, §2.2.9
//   PCIe Base Spec r2.1, §2.3
//   PCIe Base Spec r2.1, §2.3.1
// ---------------------------------------------------------------------------
`timescale 1ns/1ps
module tlp_validator
  import tlp_pkg::*;
(
    input  tlp_header_t header_i,
    output logic        valid_o,
    output tlp_error_e  error_o
);

  logic completion;
  logic config_or_io;
  logic has_data;

  always_comb begin
    completion  = header_i.tlp_type == TLP_TYPE_CPL ||
                  header_i.tlp_type == TLP_TYPE_CPL_LOCK;
    config_or_io = header_i.tlp_type == TLP_TYPE_CFG0 ||
                   header_i.tlp_type == TLP_TYPE_CFG1 ||
                   header_i.tlp_type == TLP_TYPE_IO;
    has_data = tlp_has_data(header_i.fmt);
    valid_o = 1'b1;
    error_o = TLP_ERR_NONE;

    // Fmt 100b (a TLP Prefix where a header is due) or a reserved encoding.
    if (!(header_i.fmt == TLP_FMT_3DW_NO_DATA ||
          header_i.fmt == TLP_FMT_4DW_NO_DATA ||
          header_i.fmt == TLP_FMT_3DW_DATA ||
          header_i.fmt == TLP_FMT_4DW_DATA)) begin
      valid_o = 1'b0;
      error_o = TLP_ERR_BAD_FMT_TYPE;
    end else if (!(header_i.tlp_type == TLP_TYPE_MEM ||
                   header_i.tlp_type == TLP_TYPE_IO ||
                   header_i.tlp_type == TLP_TYPE_CFG0 ||
                   header_i.tlp_type == TLP_TYPE_CFG1 ||
                   completion)) begin
      valid_o = 1'b0;
      error_o = TLP_ERR_BAD_FMT_TYPE;
    // Configuration, I/O and Completion TLPs use a 3 DW header (PCIe Base
    // Spec r2.1, §2.2.7, §2.2.9).
    end else if ((config_or_io || completion) && tlp_is_4dw(header_i.fmt)) begin
      valid_o = 1'b0;
      error_o = TLP_ERR_BAD_FMT_TYPE;
    // Below 4 GB a Requester must use the 32-bit format. The Receiver's
    // behavior for a 64-bit one is not specified, and this design rejects it
    // (PCIe Base Spec r2.1, §2.2.4.1).
    end else if (header_i.tlp_type == TLP_TYPE_MEM &&
                 tlp_is_4dw(header_i.fmt) && header_i.address[63:32] == 0) begin
      valid_o = 1'b0;
      error_o = TLP_ERR_BAD_ADDRESS_FORMAT;
    // Configuration and I/O Requests have a Length of 1 DW (§2.2.7), a Cpl
    // or CplLk has length_dw 0, and every other TLP 1 to 1024 DW (§2.2.1).
    end else if ((config_or_io && header_i.length_dw != 1) ||
                 (!completion && header_i.length_dw == 0) ||
                 (completion && !has_data && header_i.length_dw != 0) ||
                 (has_data && header_i.length_dw == 0) ||
                 header_i.length_dw > 1024) begin
      valid_o = 1'b0;
      error_o = TLP_ERR_BAD_LENGTH;
    // A 1 DW Request has Last DW BE 0000b; a longer one has neither Byte
    // Enable field 0000b (PCIe Base Spec r2.1, §2.2.5).
    end else if (!completion && header_i.length_dw == 1 && header_i.last_be != 0) begin
      valid_o = 1'b0;
      error_o = TLP_ERR_BAD_BYTE_ENABLE;
    end else if (!completion && header_i.length_dw > 1 &&
                 (header_i.first_be == 0 || header_i.last_be == 0)) begin
      valid_o = 1'b0;
      error_o = TLP_ERR_BAD_BYTE_ENABLE;
    end
  end

endmodule
