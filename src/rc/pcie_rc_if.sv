// ---------------------------------------------------------------------------
// pcie_rc_if -- received completions to the PG213 Requester Completion stream
//
// Author: Kourosh Ghahramani
// Silicon Systems Research Lab, University of Washington
//
// Purpose
//   Turns each completion that tlp_layer delivers, together with the result
//   tlp_request_tracker produces for it, into one PG213 RC packet at 128
//   bits: the 3-Dword descriptor, then the payload, Dword-aligned, through
//   pcie_axis_dw_upsize. A completion without a result makes no packet and
//   its payload is drained; tlp_request_tracker's report of an unmatched or
//   rejected one is forwarded.
//
// Interfaces
//   Header        received_completion_valid_i, _ready_o, _header_i: the
//                 completion header from tlp_layer.
//   Payload       received_completion_data_i, _keep_i, _data_valid_i,
//                 _data_last_i, _data_ready_o: its payload.
//   Result        result_*: tlp_request_tracker's result for the completion.
//   Unexpected    unexpected_completion_i, completion_error_code_i: forwarded
//                 to rc_unexpected_completion_o, rc_completion_error_code_o.
//   RC stream     m_axis_rc_*: 128-bit beats, one tkeep bit per Dword.
//   Errors        rc_protocol_error_o, rc_error_code_o, rc_gearbox_error_o.
//
// Clock and reset
//   clk_i only. rst_i is synchronous and active high.
//
// Limitations
//   Only error codes 0000b to 0010b are driven; bit 29 (Locked Read) is 0.
//   Lower Address [11:7] is the command's for every completion, so it does
//   not follow a split read's later completions or tlp_requester's later
//   segments. No tuser: PG213 makes byte_en optional to use (Table 16).
//   Byte Count Modified has no descriptor field.
//
// References
//   PG213, Table 15
//   PG213, Table 16
//   PG213, Table 65
//   PCIe Base Spec r2.1, §2.2.9
//   PCIe Base Spec r2.1, §2.3.1.1
// ---------------------------------------------------------------------------
`timescale 1ns/1ps
module pcie_rc_if
  import tlp_pkg::*;
  import pcie_rq_rc_pkg::*;
#(
    parameter int AXIS_DATA_WIDTH = 128,
    // PG213's m_axis_rc_tkeep has one bit per Dword (Table 15); the gearbox's
    // byte keep is reduced to it below.
    parameter int AXIS_KEEP_WIDTH = AXIS_DATA_WIDTH / 32,
    parameter int TL_DATA_WIDTH   = 32,
    parameter int TL_KEEP_WIDTH   = TL_DATA_WIDTH / 8,
    parameter int CONTEXT_WIDTH   = 16
) (
    input  logic                        clk_i,
    input  logic                        rst_i,

    // ---- TL received-completion header -----------------------------------
    // Wire straight to tlp_layer's received_completion_valid_o /
    // received_completion_header_o / received_completion_ready_i.
    input  logic                        received_completion_valid_i,
    output logic                        received_completion_ready_o,
    input  tlp_header_t                 received_completion_header_i,

    // ---- TL received-completion payload ----------------------------------
    input  logic [TL_DATA_WIDTH-1:0]    received_completion_data_i,
    input  logic [TL_KEEP_WIDTH-1:0]    received_completion_keep_i,
    input  logic                        received_completion_data_valid_i,
    input  logic                        received_completion_data_last_i,
    output logic                        received_completion_data_ready_o,

    // ---- tlp_request_tracker result --------------------------------------
    // A registered valid with a ready: tlp_request_tracker holds a result
    // until result_ready_o takes it.
    input  logic                        result_valid_i,
    output logic                        result_ready_o,
    input  logic [CONTEXT_WIDTH-1:0]    result_context_i,
    input  logic [2:0]                  result_status_i,
    input  logic                        result_last_i,
    // One-cycle pulses from tlp_request_tracker. No result accompanies them,
    // so they make no RC packet.
    input  logic                        unexpected_completion_i,
    input  tlp_error_e                  completion_error_code_i,

    // ---- PG213 Requester Completion AXI4-Stream master -------------------
    output logic [AXIS_DATA_WIDTH-1:0]  m_axis_rc_tdata,
    output logic [AXIS_KEEP_WIDTH-1:0]  m_axis_rc_tkeep,
    output logic                        m_axis_rc_tvalid,
    output logic                        m_axis_rc_tlast,
    input  logic                        m_axis_rc_tready,

    // ---- error surface ---------------------------------------------------
    // A completion with no matching outstanding tag
    // (TLP_ERR_UNEXPECTED_COMPLETION), or one tlp_request_tracker rejects
    // (TLP_ERR_COMPLETION_OVERFLOW). No RC packet accompanies it.
    output logic                        rc_unexpected_completion_o,
    output tlp_error_e                  rc_completion_error_code_o,
    // One-cycle pulse about the payload stream; rc_error_code_o is valid in
    // the same cycle and holds until the next pulse.
    output logic                        rc_protocol_error_o,
    output rc_error_e                   rc_error_code_o,
    // Forwarded from the descriptor/payload gearbox: illegal tkeep.
    output logic                        rc_gearbox_error_o
);

  localparam int DESC_DWORDS    = 3;                        // PG213, Table 65
  localparam int AXIS_BYTE_KEEP = AXIS_DATA_WIDTH / 8;      // gearbox tkeep

  // -------------------------------------------------------------------------
  // Header capture
  // -------------------------------------------------------------------------
  // received_completion_header_i is qualified only in its handshake cycle,
  // while result_* for the same completion comes from a register in
  // tlp_request_tracker one cycle later. hdr_r takes the header on its own
  // handshake, so a descriptor built from hdr_r and result_* describes one
  // completion even if the next header is accepted as that result is taken.
  tlp_header_t hdr_r;
  wire hdr_beat = received_completion_valid_i && received_completion_ready_o;
  wire hdr_has_data = tlp_has_data(hdr_r.fmt);

  // -------------------------------------------------------------------------
  // Descriptor build
  // -------------------------------------------------------------------------
  // Every field reads hdr_r or the result, never received_completion_header_i.
  //
  // A Completion header carries Lower Address [6:0] only. [11:7] comes from
  // the request, through the context echo pcie_rq_if loads and
  // tlp_request_tracker keeps per tag, as PG213's block takes it from its
  // Split Completion Table (Table 65). Context bit 12 is set only for a
  // Memory Read; every other Completion's Lower Address is 0 (PCIe Base Spec
  // r2.1, §2.2.9).
  wire [4:0] lower_address_high = result_context_i[12] ? result_context_i[11:7] : 5'd0;

  // Status is tested first: a completion with a status other than SC ends
  // its request and carries no data (PCIe Base Spec r2.1, §2.3.1.1), so there
  // is no payload for the poisoned bit to qualify.
  rc_desc_error_e desc_error_code;
  always_comb begin
    if      (result_status_i != TLP_CPL_SC) desc_error_code = RC_DESC_ERR_BAD_STATUS;
    else if (hdr_r.poisoned)                desc_error_code = RC_DESC_ERR_POISONED;
    else                                    desc_error_code = RC_DESC_ERR_NORMAL;
  end

  rc_descriptor_t desc_next;
  always_comb begin
    desc_next                   = '0;   // every Reserved field reads 0
    desc_next.lower_address     = {lower_address_high, hdr_r.lower_address};
    desc_next.error_code        = desc_error_code;
    desc_next.byte_count        = hdr_r.byte_count;
    desc_next.locked_read       = 1'b0;                     // no Locked Read is issued
    // Bit 30 marks the last Completion of the request, not the last beat of
    // this one (PG213, Table 65). result_last_i is tlp_request_tracker's test
    // for exactly that, and pcie_cfg_txn ends a transaction on this bit.
    desc_next.request_completed = result_last_i;
    // Payload Dwords in this packet -- 0 for a Cpl with no data.
    desc_next.dword_count       = hdr_has_data ? hdr_r.length_dw : 11'd0;
    desc_next.completion_status = hdr_r.completion_status;
    desc_next.poisoned          = hdr_r.poisoned;
    desc_next.requester_id      = hdr_r.requester_id;
    desc_next.tag               = hdr_r.tag;
    desc_next.completer_id      = hdr_r.completer_id;
    desc_next.tc                = hdr_r.traffic_class;
    desc_next.attr              = hdr_r.attributes;
  end

  // -------------------------------------------------------------------------
  // FSM
  // -------------------------------------------------------------------------
  typedef enum logic [1:0] {
    S_IDLE,     // waiting for a result; draining any payload that has none
    S_DESC,     // pushing the 3 descriptor Dwords into the gearbox
    S_PAYLOAD   // forwarding completion payload into the gearbox
  } rc_state_e;

  rc_state_e      state_r;
  rc_descriptor_t desc_r;
  logic [1:0]     desc_idx_r;   // 0..2
  logic [11:0]    dw_rem_r;     // payload Dwords still owed to the gearbox
  logic           has_data_r;

  wire [95:0] desc_bits = desc_r;

  // The header and the result are taken only in S_IDLE. The descriptor is
  // copied into desc_r as the result is taken, and no header is accepted
  // after that cycle until the packet is done, so one header register plus
  // one descriptor register suffice. While this module is busy, tlp_layer
  // holds the next completion header at the parser, and a result already
  // made stays in tlp_request_tracker.
  assign received_completion_ready_o = (state_r == S_IDLE);
  assign result_ready_o              = (state_r == S_IDLE);

  // -------------------------------------------------------------------------
  // Descriptor/payload gearbox, 32 -> 128.
  // -------------------------------------------------------------------------
  logic [TL_DATA_WIDTH-1:0]   gb_tdata;
  logic [TL_KEEP_WIDTH-1:0]   gb_tkeep;
  logic                       gb_tvalid, gb_tlast, gb_tready;
  logic [AXIS_DATA_WIDTH-1:0] gb_m_tdata;
  logic [AXIS_BYTE_KEEP-1:0]  gb_m_tkeep;

  always_comb begin
    gb_tdata  = received_completion_data_i;
    gb_tkeep  = received_completion_keep_i;
    gb_tvalid = 1'b0;
    gb_tlast  = 1'b0;
    unique case (state_r)
      S_DESC: begin
        unique case (desc_idx_r)
          2'd0:    gb_tdata = desc_bits[31:0];
          2'd1:    gb_tdata = desc_bits[63:32];
          default: gb_tdata = desc_bits[95:64];
        endcase
        gb_tkeep  = {TL_KEEP_WIDTH{1'b1}};
        gb_tvalid = 1'b1;
        // A Cpl with no data is a descriptor-only packet: tlast lands on the
        // third descriptor Dword and the gearbox emits the partial beat at once.
        gb_tlast  = (desc_idx_r == 2'(DESC_DWORDS - 1)) && !has_data_r;
      end
      S_PAYLOAD: begin
        gb_tvalid = received_completion_data_valid_i;
        // Counter-derived, like pcie_rq_if's command_data_last_o: the header's
        // own Dword Count decides where the packet ends. The stream's tlast is
        // ORed in only so a short payload cannot wedge the gearbox mid-word;
        // the disagreement is reported below. Through tlp_layer the two always
        // agree, since tlp_parser derives the payload's last from Length.
        gb_tlast  = (dw_rem_r == 12'd1) || received_completion_data_last_i;
      end
      default: ;
    endcase
  end

  // Payload is forwarded only in S_PAYLOAD and drained in S_IDLE: a
  // completion without a result (unmatched, rejected, or late for a timed-out
  // tag) still has its payload replayed by tlp_parser, and with nothing to
  // take it the receive path would stall. The drain waits on !result_valid_i
  // because a good completion's first payload Dword and its result arrive in
  // the same cycle.
  assign received_completion_data_ready_o =
      (state_r == S_PAYLOAD) ? gb_tready :
      (state_r == S_IDLE)    ? !result_valid_i : 1'b0;

  wire gb_beat  = gb_tvalid && gb_tready;
  wire orphan_beat = received_completion_data_valid_i &&
                     received_completion_data_ready_o && (state_r == S_IDLE);

  pcie_axis_dw_upsize #(
      .DATA_WIDTH_NARROW(TL_DATA_WIDTH),
      .DATA_WIDTH_WIDE  (AXIS_DATA_WIDTH)
  ) u_rc_pack (
      .clk_i(clk_i), .rst_i(rst_i),
      .s_axis_tdata (gb_tdata),  .s_axis_tkeep (gb_tkeep),
      .s_axis_tvalid(gb_tvalid), .s_axis_tlast (gb_tlast),
      .s_axis_tready(gb_tready),
      .m_axis_tdata (gb_m_tdata), .m_axis_tkeep (gb_m_tkeep),
      .m_axis_tvalid(m_axis_rc_tvalid), .m_axis_tlast(m_axis_rc_tlast),
      .m_axis_tready(m_axis_rc_tready),
      .gearbox_error_o(rc_gearbox_error_o)
  );

  assign m_axis_rc_tdata = gb_m_tdata;

  // Byte-granular to Dword-granular. A completion payload is a whole number
  // of Dwords, with byte significance carried by Lower Address and Byte
  // Count, so each nibble is 0h or Fh and the reduction loses nothing.
  always_comb begin
    for (int d = 0; d < AXIS_KEEP_WIDTH; d++)
      m_axis_rc_tkeep[d] = |gb_m_tkeep[d*4 +: 4];
  end

  // -------------------------------------------------------------------------
  // Sequential
  // -------------------------------------------------------------------------
  always_ff @(posedge clk_i) begin
    if (rst_i) begin
      state_r                    <= S_IDLE;
      hdr_r                      <= '0;
      desc_r                     <= '0;
      desc_idx_r                 <= 2'd0;
      dw_rem_r                   <= '0;
      has_data_r                 <= 1'b0;
      rc_unexpected_completion_o <= 1'b0;
      rc_completion_error_code_o <= TLP_ERR_NONE;
      rc_protocol_error_o        <= 1'b0;
      rc_error_code_o            <= RC_ERR_NONE;
    end else begin
      rc_protocol_error_o        <= 1'b0;
      // Forwarded, not re-derived: a completion tlp_request_tracker rejects
      // produces no result, and so no RC packet either.
      rc_unexpected_completion_o <= unexpected_completion_i;
      rc_completion_error_code_o <= completion_error_code_i;

      // Written on the header's own handshake, one cycle before its result,
      // so the pair read in S_IDLE below is the same completion. A header
      // taken in the cycle the previous result is taken does not disturb that
      // result's descriptor, which reads hdr_r before this edge.
      if (hdr_beat) hdr_r <= received_completion_header_i;

      unique case (state_r)
        S_IDLE: begin
          if (orphan_beat) begin
            rc_protocol_error_o <= 1'b1;
            rc_error_code_o     <= RC_ERR_ORPHAN_DATA;
            $warning("pcie_rc_if: completion payload Dword 0x%08h with no result behind it -- drained",
                     received_completion_data_i);
          end
          if (result_valid_i && result_ready_o) begin
            // tlp_request_tracker copies the header's completion_status into
            // result_status_o, so the two disagree only if hdr_r and result_*
            // describe different completions. Two completions with the same
            // status look alike here, so this catches only some mis-pairings.
            if (hdr_r.completion_status != result_status_i)
              $warning("pcie_rc_if: header/result misalignment -- header status %0d, result status %0d, tag %0d",
                       hdr_r.completion_status, result_status_i, hdr_r.tag);
            desc_r     <= desc_next;
            has_data_r <= hdr_has_data;
            dw_rem_r   <= hdr_has_data ? {1'b0, hdr_r.length_dw} : 12'd0;
            desc_idx_r <= 2'd0;
            state_r    <= S_DESC;
          end
        end

        S_DESC: if (gb_beat) begin
          if (desc_idx_r == 2'(DESC_DWORDS - 1))
            state_r <= has_data_r ? S_PAYLOAD : S_IDLE;
          else
            desc_idx_r <= desc_idx_r + 2'd1;
        end

        S_PAYLOAD: if (gb_beat) begin
          dw_rem_r <= dw_rem_r - 12'd1;
          if (received_completion_data_last_i && (dw_rem_r != 12'd1)) begin
            // The payload stopped short of the header's Dword Count. The RC
            // descriptor already carries that count, so the packet goes out
            // truncated and flagged rather than being held open forever.
            rc_protocol_error_o <= 1'b1;
            rc_error_code_o     <= RC_ERR_EARLY_LAST;
            $warning("pcie_rc_if: completion payload ended %0d Dwords before the header's Dword Count %0d",
                     dw_rem_r - 12'd1, hdr_r.length_dw);
            state_r <= S_IDLE;
          end else if (!received_completion_data_last_i && (dw_rem_r == 12'd1)) begin
            // Surplus beats. The counter has already closed the RC packet; the
            // leftovers are swallowed by the S_IDLE drain, each reported as
            // RC_ERR_ORPHAN_DATA, rather than joining the next completion.
            rc_protocol_error_o <= 1'b1;
            rc_error_code_o     <= RC_ERR_MISSING_LAST;
            $warning("pcie_rc_if: completion payload continued past the header's Dword Count %0d",
                     hdr_r.length_dw);
            state_r <= S_IDLE;
          end else if (dw_rem_r == 12'd1) begin
            state_r <= S_IDLE;
          end
        end

        default: state_r <= S_IDLE;
      endcase
    end
  end

endmodule
