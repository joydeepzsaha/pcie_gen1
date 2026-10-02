// ---------------------------------------------------------------------------
// tlp_classifier -- sorts a received header into Flow Control class and kind
//
// Purpose
//   Classifies a parsed header as a Posted Request, a Non-Posted Request or a
//   Completion, and as a memory, configuration, read or write request, for
//   tlp_layer's target and completion routing. A header that tlp_validator
//   rejects is unsupported, with every other flag cleared.
//
// Interfaces
//   Header  header_i: a parsed header.
//   Class   class_o: TLP_CLASS_POSTED for an MWr, TLP_CLASS_NON_POSTED for an
//           MRd, I/O or Configuration Request, TLP_CLASS_COMPLETION for a
//           Cpl, CplD, CplLk or CplDLk, else TLP_CLASS_UNSUPPORTED.
//   Kind    memory_request_o, config_request_o, completion_o,
//           read_request_o, write_request_o: set from the Type field and
//           from whether Fmt says the TLP carries data.
//   Status  unsupported_o: the header is not one of the types above, or
//           tlp_validator rejects it.
//
// Clock and reset
//   None; the module is combinational.
//
// References
//   PCIe Base Spec r2.1, §2.2.1
//   PCIe Base Spec r2.1, §2.6.1
// ---------------------------------------------------------------------------
`timescale 1ns/1ps
module tlp_classifier
  import tlp_pkg::*;
(
    input  tlp_header_t header_i,
    output tlp_class_e  class_o,
    output logic        memory_request_o,
    output logic        config_request_o,
    output logic        completion_o,
    output logic        read_request_o,
    output logic        write_request_o,
    output logic        unsupported_o
);

  logic header_valid;
  tlp_error_e header_error;

  always_comb begin
    class_o          = TLP_CLASS_UNSUPPORTED;
    memory_request_o = 1'b0;
    config_request_o = 1'b0;
    completion_o     = 1'b0;
    read_request_o   = 1'b0;
    write_request_o  = 1'b0;
    unsupported_o    = 1'b0;

    // Posted Requests are Memory Writes and Messages, and tlp_validator
    // rejects Messages, so Posted here is an MWr. Reads and I/O and
    // Configuration Writes are Non-Posted (PCIe Base Spec r2.1, §2.6.1).
    unique case (header_i.tlp_type)
      TLP_TYPE_MEM: begin
        memory_request_o = 1'b1;
        if (tlp_has_data(header_i.fmt)) begin
          class_o         = TLP_CLASS_POSTED;
          write_request_o = 1'b1;
        end else begin
          class_o        = TLP_CLASS_NON_POSTED;
          read_request_o = 1'b1;
        end
      end
      TLP_TYPE_IO, TLP_TYPE_CFG0, TLP_TYPE_CFG1: begin
        config_request_o = header_i.tlp_type == TLP_TYPE_CFG0 ||
                           header_i.tlp_type == TLP_TYPE_CFG1;
        class_o          = TLP_CLASS_NON_POSTED;
        read_request_o   = !tlp_has_data(header_i.fmt);
        write_request_o  =  tlp_has_data(header_i.fmt);
      end
      TLP_TYPE_CPL, TLP_TYPE_CPL_LOCK: begin
        class_o      = TLP_CLASS_COMPLETION;
        completion_o = 1'b1;
      end
      default: unsupported_o = 1'b1;
    endcase

    if (header_i.length_dw > 11'd1024) begin
      class_o       = TLP_CLASS_UNSUPPORTED;
      unsupported_o = 1'b1;
    end

    // A rejected header is unsupported with no other flag set. tlp_validator
    // rejects every type the default arm takes and every Length above 1024,
    // so this block decides those headers as well.
    if (!header_valid) begin
      class_o          = TLP_CLASS_UNSUPPORTED;
      memory_request_o = 1'b0;
      config_request_o = 1'b0;
      completion_o     = 1'b0;
      read_request_o   = 1'b0;
      write_request_o  = 1'b0;
      unsupported_o    = 1'b1;
    end
  end

  tlp_validator validator_inst (
      .header_i(header_i), .valid_o(header_valid), .error_o(header_error)
  );

endmodule
