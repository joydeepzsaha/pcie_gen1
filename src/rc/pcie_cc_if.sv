// ---------------------------------------------------------------------------
// pcie_cc_if -- PG213 Completer Completion stream to the TL completion port
//
// Author: Kourosh Ghahramani
// Silicon Systems Research Lab, University of Washington
//
// Purpose
//   Takes the host's completions in PG213's CC format at 128 bits (a 3-Dword
//   descriptor, then the payload), narrows each whole packet to 32 bits with
//   pcie_axis_dw_downsize, and presents it on tlp_layer's
//   completion_request_* port, which feeds tlp_completion_generator. It also
//   issues the Unsupported Request Completions that pcie_cq_if asks for,
//   because that port group has a single driver.
//
// Interfaces
//   CC stream     s_axis_cc_*: 128-bit beats, one tkeep bit per Dword.
//   Completion    completion_request_*: tlp_layer's completion port.
//   Auto-UR       ur_valid_i, ur_ready_o, ur_header_i, ur_byte_count_i: a
//                 dropped non-posted request from pcie_cq_if.
//   Errors        cc_protocol_error_o, cc_error_code_o, cc_gearbox_error_o.
//
// Clock and reset
//   clk_i only. rst_i is synchronous and active high.
//
// Limitations
//   Each packet's Byte Count is taken as the whole completion, and
//   tlp_completion_generator splits it at the Read Completion Boundary and
//   Max_Payload_Size, so a host must send a completion as one packet even
//   where PG213 has the user split it. The descriptor's Dword Count only
//   frames the packet. Poisoned, Locked Read, Address Type, Completer ID
//   Enable, Completer Bus, Target Function and s_axis_cc_tuser are not read.
//   A UR Completion for a Memory Read always carries Lower Address 0.
//
// References
//   PG213, Figure 32
//   PG213, Table 12
//   PG213, Table 58
//   PCIe Base Spec r2.1, §2.2.9
//   PCIe Base Spec r2.1, §2.3.1
//   PCIe Base Spec r2.1, §2.3.1.1
// ---------------------------------------------------------------------------
`timescale 1ns/1ps
module pcie_cc_if
  import tlp_pkg::*;
  import pcie_rq_rc_pkg::*;
#(
    parameter int AXIS_DATA_WIDTH = 128,
    parameter int AXIS_KEEP_WIDTH = AXIS_DATA_WIDTH / 32,
    parameter int CC_USER_WIDTH   = 33,
    parameter int TL_DATA_WIDTH   = 32,
    parameter int TL_KEEP_WIDTH   = TL_DATA_WIDTH / 8
) (
    input  logic                        clk_i,
    input  logic                        rst_i,

    // ---- PG213 Completer Completion AXI4-Stream slave ----------------------
    input  logic [AXIS_DATA_WIDTH-1:0]  s_axis_cc_tdata,
    input  logic [AXIS_KEEP_WIDTH-1:0]  s_axis_cc_tkeep,
    input  logic                        s_axis_cc_tvalid,
    input  logic                        s_axis_cc_tlast,
    // Not read: it carries discontinue and parity (PG213, Table 12).
    input  logic [CC_USER_WIDTH-1:0]    s_axis_cc_tuser,
    output logic                        s_axis_cc_tready,

    // ---- TL completion request, into tlp_completion_generator --------------
    output logic                        completion_request_valid_o,
    input  logic                        completion_request_ready_i,
    output tlp_header_t                 completion_request_header_o,
    output logic [2:0]                  completion_request_status_o,
    output logic [12:0]                 completion_request_byte_count_o,
    output logic [6:0]                  completion_request_lower_address_o,
    output logic                        completion_request_ecrc_enable_o,

    output logic [TL_DATA_WIDTH-1:0]    completion_request_data_o,
    output logic [TL_KEEP_WIDTH-1:0]    completion_request_keep_o,
    output logic                        completion_request_data_valid_o,
    output logic                        completion_request_data_last_o,
    input  logic                        completion_request_data_ready_i,

    // ---- auto-UR sideband from pcie_cq_if ----------------------------------
    // An inbound non-posted request the completer could not deliver. This
    // module synthesises the UR Completion for it, because it already drives
    // the completion_request_* group and that group must have one driver.
    input  logic                        ur_valid_i,
    output logic                        ur_ready_o,
    input  tlp_header_t                 ur_header_i,
    input  logic [12:0]                 ur_byte_count_i,

    // ---- error surface -----------------------------------------------------
    // One-cycle pulse; cc_error_code_o is valid in the same cycle and holds
    // until the next pulse. CC_ERR_NONE is never presented with the pulse.
    output logic                        cc_protocol_error_o,
    output cc_error_e                   cc_error_code_o,
    output logic                        cc_gearbox_error_o
);

  localparam int DESC_DWORDS = 3;   // PG213, Table 58; not referenced below

  // -------------------------------------------------------------------------
  // 128 -> 32 gearbox
  // -------------------------------------------------------------------------
  // The whole packet goes through it, descriptor included. In PG213's
  // Dword-aligned mode the 3-Dword descriptor shares 128-bit beat 0 with
  // payload Dword 0, so reading the descriptor from the wide beat and the
  // payload from the gearbox would need two readers of beat 0 with different
  // alignments. One narrow stream is counted instead: Dwords 0 to 2 are the
  // descriptor, 3 onward the payload.
  logic [TL_DATA_WIDTH-1:0] nb_tdata;
  logic [TL_KEEP_WIDTH-1:0] nb_tkeep;
  logic                     nb_tvalid, nb_tlast, nb_tready;

  // PG213's s_axis_cc_tkeep has one bit per Dword; the gearbox's has one per
  // byte. Each Dword bit is expanded to its four byte lanes, the mirror of
  // the reduction pcie_rc_if and pcie_cq_if do on the way out.
  logic [AXIS_DATA_WIDTH/8-1:0] cc_byte_keep;
  always_comb begin
    for (int d = 0; d < AXIS_KEEP_WIDTH; d++)
      cc_byte_keep[d*4 +: 4] = {4{s_axis_cc_tkeep[d]}};
  end

  pcie_axis_dw_downsize #(
      .DATA_WIDTH_WIDE  (AXIS_DATA_WIDTH),
      .DATA_WIDTH_NARROW(TL_DATA_WIDTH)
  ) u_cc_unpack (
      .clk_i(clk_i), .rst_i(rst_i),
      .s_axis_tdata (s_axis_cc_tdata),  .s_axis_tkeep (cc_byte_keep),
      .s_axis_tvalid(s_axis_cc_tvalid), .s_axis_tlast (s_axis_cc_tlast),
      .s_axis_tready(s_axis_cc_tready),
      .m_axis_tdata (nb_tdata), .m_axis_tkeep(nb_tkeep),
      .m_axis_tvalid(nb_tvalid), .m_axis_tlast(nb_tlast),
      .m_axis_tready(nb_tready),
      .gearbox_error_o(cc_gearbox_error_o)
  );

  // -------------------------------------------------------------------------
  // Descriptor assembly: Dwords 0..2 of the narrowed stream.
  // -------------------------------------------------------------------------
  logic [1:0]  dw_idx_r;      // 0..2 while collecting the descriptor
  logic [31:0] desc_dw0_r, desc_dw1_r;
  cc_descriptor_t desc_r;

  // Dword 2 is read straight from the stream, in the cycle it is accepted.
  wire [95:0] desc_bits = {nb_tdata, desc_dw1_r, desc_dw0_r};
  wire cc_desc_status_legal =
      (desc_bits[45:43] == TLP_CPL_SC) || (desc_bits[45:43] == TLP_CPL_UR) ||
      (desc_bits[45:43] == TLP_CPL_CA);

  typedef enum logic [2:0] {
    S_DESC,     // collecting the 3 descriptor Dwords
    S_PRESENT,  // offering the completion request to the TL
    S_PAYLOAD,  // streaming payload into the TL
    S_DROP,     // swallowing a rejected packet's remainder
    S_UR        // offering a synthesised UR Completion to the TL
  } cc_state_e;

  cc_state_e   state_r;
  logic [11:0] dw_rem_r;      // payload Dwords still owed by the host
  logic        has_data_r;

  // -------------------------------------------------------------------------
  // TL-facing outputs
  // -------------------------------------------------------------------------
  // The header and status come from desc_r, or in S_UR from ur_header_i and a
  // fixed UR status; the payload comes straight from the narrow stream. The
  // UR path and the host path share this port group and S_UR selects. They
  // never present together, because S_UR is entered only from S_DESC at a
  // packet boundary.
  wire in_ur = (state_r == S_UR);

  always_comb begin
    completion_request_header_o               = '0;
    completion_request_header_o.requester_id  = in_ur ? ur_header_i.requester_id
                                                      : desc_r.requester_id;
    completion_request_header_o.tag           = in_ur ? ur_header_i.tag
                                                      : desc_r.tag;
    completion_request_header_o.traffic_class = in_ur ? ur_header_i.traffic_class
                                                      : desc_r.tc;
    completion_request_header_o.attributes    = in_ur ? ur_header_i.attributes
                                                      : desc_r.attr;
  end

  assign completion_request_valid_o = (state_r == S_PRESENT) || in_ur;
  // A UR Completion carries no data (PCIe Base Spec r2.1, §2.3.1.1), and
  // tlp_completion_generator sends none whenever the status is not SC.
  // A UR Completion's Lower Address is always 0, which is right for an I/O or
  // Configuration request (PCIe Base Spec r2.1, §2.2.9). For a Memory Read it
  // must be the lower address bits of the first enabled byte (PCIe Base Spec
  // r2.1, §2.3.1.1); ur_header_i holds the address and first_be, but they are
  // not used. The Byte Count is ur_byte_count_i, as pcie_cq_if computes it.
  assign completion_request_status_o        = in_ur ? 3'(TLP_CPL_UR)
                                                    : desc_r.completion_status;
  assign completion_request_byte_count_o    = in_ur ? ur_byte_count_i
                                                    : desc_r.byte_count;
  assign completion_request_lower_address_o = in_ur ? 7'd0 : desc_r.lower_address;
  assign completion_request_ecrc_enable_o   = in_ur ? 1'b0 : desc_r.force_ecrc;
  assign ur_ready_o                         = in_ur && completion_request_ready_i;

  assign completion_request_data_o       = nb_tdata;
  assign completion_request_keep_o       = nb_tkeep;
  assign completion_request_data_valid_o = (state_r == S_PAYLOAD) && nb_tvalid;
  // Counter-derived, like pcie_rq_if's and pcie_cq_if's: the descriptor's own
  // Dword Count decides where the payload ends. The stream's own last is ORed
  // in so a short packet cannot wedge the TL mid-completion; the disagreement
  // is reported rather than absorbed.
  assign completion_request_data_last_o  = (dw_rem_r == 12'd1) || nb_tlast;

  // The narrowed stream is consumed while collecting the descriptor, while
  // draining a rejected packet, and -- gated on the TL -- during payload.
  always_comb begin
    unique case (state_r)
      // Ready drops on exactly the UR preempt's condition. In the cycle that
      // moves to S_UR the nb_beat arm below does not run, so a Dword accepted
      // then would be lost and the host's descriptor would shift by one Dword.
      // Gating on the whole condition rather than on ur_valid_i alone avoids a
      // deadlock: a UR that arrives mid-descriptor waits for dw_idx_r to return
      // to 0, which needs the stream to keep moving.
      S_DESC:    nb_tready = !(ur_valid_i && dw_idx_r == 2'd0);
      S_PAYLOAD: nb_tready = completion_request_data_ready_i;
      S_DROP:    nb_tready = 1'b1;
      // S_PRESENT holds the stream while the TL accepts the header; S_UR holds
      // it because the host's next packet must not be consumed while a
      // synthesised Completion is in flight.
      default:   nb_tready = 1'b0;
    endcase
  end

  wire nb_beat = nb_tvalid && nb_tready;

  always_ff @(posedge clk_i) begin
    if (rst_i) begin
      state_r             <= S_DESC;
      dw_idx_r            <= 2'd0;
      desc_dw0_r          <= '0;
      desc_dw1_r          <= '0;
      desc_r              <= '0;
      dw_rem_r            <= '0;
      has_data_r          <= 1'b0;
      cc_protocol_error_o <= 1'b0;
      cc_error_code_o     <= CC_ERR_NONE;
    end else begin
      cc_protocol_error_o <= 1'b0;

      unique case (state_r)
        // A pending auto-UR preempts only at dw_idx_r == 0, a packet
        // boundary, so a synthesised Completion never lands inside the
        // host's descriptor.
        S_DESC: if (ur_valid_i && dw_idx_r == 2'd0) begin
          state_r <= S_UR;
        end else if (nb_beat) begin
          unique case (dw_idx_r)
            2'd0: begin
              desc_dw0_r <= nb_tdata;
              dw_idx_r   <= 2'd1;
              if (nb_tlast) begin
                // A completion packet cannot be shorter than its descriptor.
                cc_protocol_error_o <= 1'b1;
                cc_error_code_o     <= CC_ERR_EARLY_LAST;
                $warning("pcie_cc_if: CC packet ended after 1 Dword -- the descriptor is 3");
                dw_idx_r <= 2'd0;
              end
            end
            2'd1: begin
              desc_dw1_r <= nb_tdata;
              dw_idx_r   <= 2'd2;
              if (nb_tlast) begin
                cc_protocol_error_o <= 1'b1;
                cc_error_code_o     <= CC_ERR_EARLY_LAST;
                $warning("pcie_cc_if: CC packet ended after 2 Dwords -- the descriptor is 3");
                dw_idx_r <= 2'd0;
              end
            end
            default: begin
              dw_idx_r <= 2'd0;
              desc_r   <= cc_descriptor_t'(desc_bits);
              // PG213 allows only SC, UR and CA here (Table 58). CRS answers
              // only a Configuration Request (PCIe Base Spec r2.1, §2.3.1),
              // and pcie_cq_if delivers no Configuration Request to the host.
              if (!cc_desc_status_legal) begin
                cc_protocol_error_o <= 1'b1;
                cc_error_code_o     <= CC_ERR_BAD_STATUS;
                $warning("pcie_cc_if: Completion Status %0d is not SC/UR/CA (PG213 Table 58) -- packet dropped",
                         desc_bits[45:43]);
                state_r <= nb_tlast ? S_DESC : S_DROP;
              end else if (desc_bits[45:43] != TLP_CPL_SC &&
                           desc_bits[42:32] != 11'd0) begin
                // PG213 requires Dword Count 0 on a UR or CA Completion
                // (Table 58). tlp_completion_generator sends such a Completion
                // with Length 0 and takes no payload, so a payload here would
                // never be consumed.
                cc_protocol_error_o <= 1'b1;
                cc_error_code_o     <= CC_ERR_DATA_ON_ERROR;
                $warning("pcie_cc_if: status %0d with Dword Count %0d -- a non-SC Completion carries no data",
                         desc_bits[45:43], desc_bits[42:32]);
                state_r <= nb_tlast ? S_DESC : S_DROP;
              end else begin
                // With Byte Count 0 and a non-zero Dword Count,
                // tlp_completion_generator takes no payload and S_PAYLOAD
                // waits until rst_i. A tlast that disagrees with the Dword
                // Count is not flagged here.
                has_data_r <= (desc_bits[42:32] != 11'd0);
                dw_rem_r   <= {1'b0, desc_bits[42:32]};
                state_r    <= S_PRESENT;
              end
            end
          endcase
        end

        // Offer the header, then the payload.
        S_PRESENT: if (completion_request_ready_i) begin
          state_r <= has_data_r ? S_PAYLOAD : S_DESC;
        end

        S_PAYLOAD: if (nb_beat) begin
          dw_rem_r <= dw_rem_r - 12'd1;
          if (nb_tlast && (dw_rem_r != 12'd1)) begin
            cc_protocol_error_o <= 1'b1;
            cc_error_code_o     <= CC_ERR_EARLY_LAST;
            $warning("pcie_cc_if: CC payload ended %0d Dwords before the descriptor's Dword Count",
                     dw_rem_r - 12'd1);
            state_r <= S_DESC;
          end else if (!nb_tlast && (dw_rem_r == 12'd1)) begin
            cc_protocol_error_o <= 1'b1;
            cc_error_code_o     <= CC_ERR_MISSING_LAST;
            $warning("pcie_cc_if: CC payload continued past the descriptor's Dword Count");
            state_r <= S_DROP;
          end else if (dw_rem_r == 12'd1) begin
            state_r <= S_DESC;
          end
        end

        // The synthesised UR Completion.
        S_UR: if (completion_request_ready_i) state_r <= S_DESC;

        // Swallow a rejected packet's remainder.
        S_DROP: if (nb_beat && nb_tlast) state_r <= S_DESC;

        default: state_r <= S_DESC;
      endcase
    end
  end

endmodule
