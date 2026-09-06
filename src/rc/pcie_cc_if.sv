// ---------------------------------------------------------------------------
// pcie_cc_if -- PG213 Completer Completion (CC) AXI4-Stream slave. Stage F-1.
//
// SPEC ANCHORS
//   PG213 v1.3 Table 58 (p. 168-169) . the 96-bit / 3-Dword CC descriptor this
//                                      module decodes.
//   PG213 v1.3 Figure 32 (p. 169) .... "The descriptor is always 12 bytes long
//                                      and is sent in the first 12 bytes of
//                                      the completion packet."
//   PCIe Base 2.1 SS2.2.9 p. 97 ...... Completion Rules -- the header fields.
//   PCIe Base 2.1 SS2.3.2 p. 120 ..... Completion Status encodings.
//   Field placement is owned by pcie_rq_rc_pkg (cc_descriptor_t).
//
// The mirror of pcie_rq_if: it takes the host's completion descriptor and
// payload off s_axis_cc_* at 128 bits, narrows through pcie_axis_dw_downsize,
// and presents them to tlp_layer's completion_request_* port group, which
// feeds tlp_completion_generator.
//
// ---------------------------------------------------------------------------
// SS WHAT THIS MODULE IS FOR: the other half of sec 41.1 A4
// ---------------------------------------------------------------------------
//
// pcie_cq_if closed the half where an inbound request vanished. This closes
// the half where it was never ANSWERED: pcie_rq_rc_top tied
// completion_request_valid_i to 1'b0 against an all-zero header, so
// tlp_completion_generator -- which has been instantiated and reachable the
// whole time -- was never once asked to emit anything. A device's inbound
// Memory Read got no Completion and had to discover that through its own
// Completion Timeout.
//
// ---------------------------------------------------------------------------
// SS WHAT THIS MODULE DELIBERATELY DOES NOT DO
// ---------------------------------------------------------------------------
//
// It does not split completions at the Read Completion Boundary, does not
// compute Byte Count, and does not decide Length. tlp_completion_generator
// already does all three: it clamps each segment to min(remaining, MPS,
// RCB boundary), advances Lower Address and Byte Count per segment, and
// re-enters its header state for the next one. Re-deriving any of that here
// would make this module a second owner of the segmentation rule.
//
// The host therefore hands over ONE logical completion -- status, total Byte
// Count, starting Lower Address, and the whole payload -- and the Transaction
// Layer decides how many CplDs that becomes. That is also why dword_count from
// the descriptor is used ONLY to frame this AXI-Stream packet and is never
// forwarded to the TL.
//
// ---------------------------------------------------------------------------
// SS WHY THE DESCRIPTOR IS TAKEN FROM THE NARROWED STREAM
// ---------------------------------------------------------------------------
//
// The CC descriptor is 3 Dwords, so on a 128-bit interface beat 0 carries the
// descriptor in Dwords 0..2 AND the first payload Dword in Dword 3. Reading
// the descriptor off the wide beat and the payload off a downsizer would mean
// two readers of beat 0 with different alignments -- the kind of split that
// produces an off-by-one-Dword payload nobody notices until a multi-Dword
// write. Instead the whole packet goes through one downsizer and this module
// counts Dwords: 0,1,2 are the descriptor, 3.. are payload. One reader, one
// alignment, and the 3-vs-4 Dword difference from the CQ side cannot leak in.
//
// ---------------------------------------------------------------------------
// SS OUT OF SCOPE (documented, not implemented -- KNOWN_GAPS)
// ---------------------------------------------------------------------------
//
//  * s_axis_cc_tuser. PG213 Table 62 carries parity and discontinue; this
//    design produces neither. Not read.
//  * Poisoned completions (descriptor bit 46). Read and $warning'd, not
//    forwarded -- tlp_header_t has a poisoned field but the generator does not
//    take one on its request port. Registered item.
//  * Completer ID Enable / Completer Bus / Target Function. See the note on
//    cc_descriptor_t: the TL owns our identity through completer_id_i and a
//    second owner is worse than a missing feature.
//
// Guards use $warning, never $error: a procedural $error maps to $stop under
// the simulator, which would abort the shared multi-test process.
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

    // ---- error surface -----------------------------------------------------
    // One-cycle pulse; cc_error_code_o is valid in the same cycle and holds
    // until the next pulse. CC_ERR_NONE is never presented with the pulse.
    output logic                        cc_protocol_error_o,
    output cc_error_e                   cc_error_code_o,
    output logic                        cc_gearbox_error_o
);

  localparam int DESC_DWORDS = 3;   // PG213 Figure 32 / Table 58

  // -------------------------------------------------------------------------
  // 128 -> 32 gearbox. The WHOLE packet goes through it -- see SS WHY THE
  // DESCRIPTOR IS TAKEN FROM THE NARROWED STREAM.
  // -------------------------------------------------------------------------
  logic [TL_DATA_WIDTH-1:0] nb_tdata;
  logic [TL_KEEP_WIDTH-1:0] nb_tkeep;
  logic                     nb_tvalid, nb_tlast, nb_tready;

  // PG213 s_axis_cc_tkeep is DWORD-granular; the gearbox is byte-granular.
  // Expand each Dword bit to its four byte lanes -- the mirror of the
  // reduction pcie_rc_if and pcie_cq_if do on the way out.
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

  wire [95:0] desc_bits = {nb_tdata, desc_dw1_r, desc_dw0_r};
  wire cc_desc_status_legal =
      (desc_bits[45:43] == TLP_CPL_SC) || (desc_bits[45:43] == TLP_CPL_UR) ||
      (desc_bits[45:43] == TLP_CPL_CA);

  typedef enum logic [1:0] {
    S_DESC,     // collecting the 3 descriptor Dwords
    S_PRESENT,  // offering the completion request to the TL
    S_PAYLOAD,  // streaming payload into the TL
    S_DROP      // swallowing a rejected packet's remainder
  } cc_state_e;

  cc_state_e   state_r;
  logic [11:0] dw_rem_r;      // payload Dwords still owed by the host
  logic        has_data_r;

  // -------------------------------------------------------------------------
  // TL-facing outputs, all from desc_r -- never from the live stream.
  // -------------------------------------------------------------------------
  always_comb begin
    completion_request_header_o               = '0;
    completion_request_header_o.requester_id  = desc_r.requester_id;
    completion_request_header_o.tag           = desc_r.tag;
    completion_request_header_o.traffic_class = desc_r.tc;
    completion_request_header_o.attributes    = desc_r.attr;
  end

  assign completion_request_valid_o         = (state_r == S_PRESENT);
  assign completion_request_status_o        = desc_r.completion_status;
  assign completion_request_byte_count_o    = desc_r.byte_count;
  assign completion_request_lower_address_o = desc_r.lower_address;
  assign completion_request_ecrc_enable_o   = desc_r.force_ecrc;

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
      S_DESC:    nb_tready = 1'b1;
      S_PAYLOAD: nb_tready = completion_request_data_ready_i;
      S_DROP:    nb_tready = 1'b1;
      default:   nb_tready = 1'b0;   // S_PRESENT: hold the stream while the
    endcase                          // TL accepts the header
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
        // ------------------------------------------------ 3 descriptor Dwords
        S_DESC: if (nb_beat) begin
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
              // PG213 Table 58: the only legal Completion Status values on
              // this interface are SC, UR and CA. CRS is NOT one of them -- a
              // Root Complex may RECEIVE a CRS completion but never originates
              // one, which is why pcie_rc_if carries CRS and this module
              // rejects it.
              if (!cc_desc_status_legal) begin
                cc_protocol_error_o <= 1'b1;
                cc_error_code_o     <= CC_ERR_BAD_STATUS;
                $warning("pcie_cc_if: Completion Status %0d is not SC/UR/CA (PG213 Table 58) -- packet dropped",
                         desc_bits[45:43]);
                state_r <= nb_tlast ? S_DESC : S_DROP;
              end else if (desc_bits[45:43] != TLP_CPL_SC &&
                           desc_bits[42:32] != 11'd0) begin
                // Base 2.1 SS2.2.9 p. 97 and PG213 Table 58: "The Dword count
                // must be set to 0 when sending a UR or CA Completion." A
                // non-SC completion carrying payload is a host bug; taking it
                // would emit a Completion whose Length contradicts its status.
                cc_protocol_error_o <= 1'b1;
                cc_error_code_o     <= CC_ERR_DATA_ON_ERROR;
                $warning("pcie_cc_if: status %0d with Dword Count %0d -- a non-SC Completion carries no data",
                         desc_bits[45:43], desc_bits[42:32]);
                state_r <= nb_tlast ? S_DESC : S_DROP;
              end else begin
                has_data_r <= (desc_bits[42:32] != 11'd0);
                dw_rem_r   <= {1'b0, desc_bits[42:32]};
                state_r    <= S_PRESENT;
              end
            end
          endcase
        end

        // ------------------------------------- offer the header, then payload
        S_PRESENT: if (completion_request_ready_i) begin
          state_r <= has_data_r ? S_PAYLOAD : S_DESC;
        end

        // ------------------------------------------------------------ payload
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

        // -------------------------------------- swallow a rejected remainder
        S_DROP: if (nb_beat && nb_tlast) state_r <= S_DESC;

        default: state_r <= S_DESC;
      endcase
    end
  end

endmodule
