// ---------------------------------------------------------------------------
// pcie_cq_if -- inbound requests to the PG213 Completer Request stream
//
// Author: Kourosh Ghahramani
// Silicon Systems Research Lab, University of Washington
//
// Purpose
//   Delivers inbound requests that tlp_layer has parsed, classified and
//   BAR-decoded, each as one PG213 CQ packet at 128 bits: the 4-Dword
//   descriptor, then the payload, through pcie_axis_dw_upsize. Only a Memory
//   request that hits exactly one enabled BAR is delivered; any other is
//   dropped with a reason on cq_dropped_o, and a dropped non-posted request
//   goes to pcie_cc_if, which answers it with a UR Completion.
//
// Interfaces
//   Request       target_request_*, target_memory_i to target_bar_i: the
//                 request header and tlp_layer's decode of it.
//   Payload       target_data_i, _keep_i, _data_valid_i, _data_last_i,
//                 target_data_ready_o: the request payload.
//   CQ stream     m_axis_cq_*: 128-bit beats, one tkeep bit per Dword.
//   Auto-UR       ur_*: the dropped non-posted request, to pcie_cc_if.
//   Drops         cq_dropped_o, cq_error_code_o, cq_gearbox_error_o.
//
// Clock and reset
//   clk_i only. rst_i is synchronous and active high.
//
// Limitations
//   CQ_BAR_APERTURE is a parameter, not a decode. BAR ID is the decoded
//   index, where PG213 has a Root Port report 000b. m_axis_cq_tuser carries
//   first_be and last_be only, and can change while a one-beat packet waits
//   for m_axis_cq_tready. target_read_i is not read.
//
// References
//   PG213, Table 9
//   PG213, Table 10
//   PG213, Table 52
//   PCIe Base Spec r2.1, §2.1.1
//   PCIe Base Spec r2.1, §2.2.9
//   PCIe Base Spec r2.1, §2.3.1
//   PCIe Base Spec r2.1, §2.3.1.1
// ---------------------------------------------------------------------------
`timescale 1ns/1ps
module pcie_cq_if
  import tlp_pkg::*;
  import pcie_rq_rc_pkg::*;
#(
    parameter int AXIS_DATA_WIDTH = 128,
    // PG213's m_axis_cq_tkeep has one bit per Dword (Table 9); the gearbox's
    // byte keep is reduced to it below.
    parameter int AXIS_KEEP_WIDTH = AXIS_DATA_WIDTH / 32,
    parameter int CQ_USER_WIDTH   = 88,
    parameter int TL_DATA_WIDTH   = 32,
    parameter int TL_KEEP_WIDTH   = TL_DATA_WIDTH / 8,
    parameter int BAR_INDEX_WIDTH = 1,
    // Aperture of the matching BAR in address bits, for descriptor [120:115].
    // tlp_layer does not export the matched aperture, so this must match the
    // BAR_MASK it is built with; pcie_rq_rc_top derives both from
    // HOST_MEM_SIZE. The default, 12 (4 KB), matches tlp_layer's default.
    parameter logic [5:0] CQ_BAR_APERTURE = 6'd12
) (
    input  logic                        clk_i,
    input  logic                        rst_i,

    // ---- TL target request, from tlp_layer's completer surface -------------
    // The decode inputs are combinational from tlp_layer's parsed header and
    // are captured on the header handshake (see Captured request).
    input  logic                        target_request_valid_i,
    output logic                        target_request_ready_o,
    input  tlp_header_t                 target_request_header_i,
    input  logic                        target_memory_i,
    input  logic                        target_config_i,
    input  logic                        target_config_type_one_i,
    input  logic                        target_read_i,
    input  logic                        target_write_i,
    input  logic                        target_unsupported_i,
    input  logic                        target_bar_hit_i,
    input  logic                        target_bar_overlap_i,
    input  logic [BAR_INDEX_WIDTH-1:0]  target_bar_i,

    // ---- TL target payload -------------------------------------------------
    input  logic [TL_DATA_WIDTH-1:0]    target_data_i,
    input  logic [TL_KEEP_WIDTH-1:0]    target_keep_i,
    input  logic                        target_data_valid_i,
    input  logic                        target_data_last_i,
    output logic                        target_data_ready_o,

    // ---- PG213 Completer Request AXI4-Stream master ------------------------
    output logic [AXIS_DATA_WIDTH-1:0]  m_axis_cq_tdata,
    output logic [AXIS_KEEP_WIDTH-1:0]  m_axis_cq_tkeep,
    output logic                        m_axis_cq_tvalid,
    output logic                        m_axis_cq_tlast,
    output logic [CQ_USER_WIDTH-1:0]    m_axis_cq_tuser,
    input  logic                        m_axis_cq_tready,

    // ---- auto-UR sideband to pcie_cc_if ------------------------------------
    // A dropped non-posted request is owed a Completion with status UR (PCIe
    // Base Spec r2.1, §2.3.1). pcie_cc_if drives the completion_request_*
    // port group, so the request is handed to it here and it builds the
    // Completion. A dropped Memory Write is posted and gets no Completion;
    // cq_dropped_o is its whole report.
    output logic                        ur_valid_o,
    input  logic                        ur_ready_i,
    output tlp_header_t                 ur_header_o,
    output logic [12:0]                 ur_byte_count_o,

    // ---- drop report -------------------------------------------------------
    // One-cycle pulse; cq_error_code_o is valid in the same cycle and holds
    // until the next pulse. CQ_DROP_NONE is never presented with the pulse.
    output logic                        cq_dropped_o,
    output cq_error_e                   cq_error_code_o,
    // Forwarded from the descriptor/payload gearbox: illegal tkeep.
    output logic                        cq_gearbox_error_o
);

  localparam int DESC_DWORDS    = 4;                    // PG213, Table 52
  localparam int AXIS_BYTE_KEEP = AXIS_DATA_WIDTH / 8;

  // -------------------------------------------------------------------------
  // Captured request
  // -------------------------------------------------------------------------
  // The decode inputs follow whatever header tlp_parser presents, and once
  // this module leaves S_IDLE the parser may move on to other TLPs. The
  // request is therefore captured on its handshake into these registers, and
  // nothing after S_IDLE reads the live header or decode inputs.
  tlp_header_t              hdr_r;
  logic [BAR_INDEX_WIDTH-1:0] bar_r;
  cq_req_type_e             req_type_r;
  logic                     has_data_r;
  logic [11:0]              dw_rem_r;

  // -------------------------------------------------------------------------
  // Classification of the request being offered, from tlp_layer's decode.
  // Combinational -- read only in S_IDLE, on the acceptance edge.
  // -------------------------------------------------------------------------
  cq_req_type_e offered_type;
  always_comb begin
    if (target_memory_i)
      offered_type = target_write_i ? CQ_MEM_WRITE : CQ_MEM_READ;
    else if (target_config_i)
      offered_type = target_config_type_one_i ?
          (target_write_i ? CQ_CFG_WRITE1 : CQ_CFG_READ1) :
          (target_write_i ? CQ_CFG_WRITE0 : CQ_CFG_READ0);
    else
      offered_type = target_write_i ? CQ_IO_WRITE : CQ_IO_READ;
  end

  // Only a Memory request that hits exactly one enabled BAR is delivered;
  // every other request is dropped with a reason. Overlap is tested before
  // the hit because tlp_bar_decoder clears hit_o when two BARs match.
  logic offered_deliver;
  cq_error_e offered_drop_code;
  always_comb begin
    offered_deliver    = 1'b0;
    offered_drop_code  = CQ_DROP_NONE;
    if (!target_memory_i)              offered_drop_code = CQ_DROP_UNSUPPORTED;
    else if (target_bar_overlap_i)     offered_drop_code = CQ_DROP_BAR_OVERLAP;
    else if (!target_bar_hit_i)        offered_drop_code = CQ_DROP_NO_BAR;
    else if (target_unsupported_i)     offered_drop_code = CQ_DROP_UNSUPPORTED;
    else                               offered_deliver   = 1'b1;
  end

  wire offered_has_data = tlp_has_data(target_request_header_i.fmt);

  // Every request here but a Memory Write needs a Completion. I/O and
  // Configuration Writes carry data and are still non-posted, so having data
  // is not the test (PCIe Base Spec r2.1, §2.1.1, §2.2.9). Messages (posted)
  // and AtomicOps never get here: tlp_parser drops them as malformed.
  wire offered_non_posted = !(target_memory_i && target_write_i);

  // -------------------------------------------------------------------------
  // Descriptor build. Reads hdr_r / bar_r / req_type_r, never the live inputs.
  // -------------------------------------------------------------------------
  // Combinational from the captured request, which stays fixed while the
  // four descriptor Dwords are pushed, so no second register is needed;
  // pcie_rc_if has one only because its header and result arrive a cycle
  // apart. A registered copy written in the same block as hdr_r would capture
  // the previous request, since it would read hdr_r before the loading edge.
  cq_descriptor_t desc;
  always_comb begin
    desc                 = '0;        // every Reserved field reads 0
    desc.address_type    = hdr_r.address_type;
    desc.address         = hdr_r.address[63:2];
    desc.dword_count     = hdr_r.length_dw;
    desc.req_type        = req_type_r;
    desc.requester_id    = hdr_r.requester_id;
    desc.tag             = hdr_r.tag;
    desc.target_function = 8'd0;               // single-function Root Complex
    // The decoded BAR index, where PG213 has a Root Port report 000b
    // (Table 52); this field is the only use of target_bar_i. pcie_rq_rc_top
    // enables BAR 0 only, so there the two agree.
    desc.bar_id          = 3'(bar_r);
    desc.bar_aperture    = CQ_BAR_APERTURE;
    desc.tc              = hdr_r.traffic_class;
    desc.attr            = hdr_r.attributes;
  end

  wire [127:0] desc_bits = desc;

  // first_be at [3:0] and last_be at [7:4], valid in the first beat (PG213,
  // Table 10). byte_en, sop, discontinue, the TPH fields and parity read 0;
  // Table 10 makes byte_en and sop optional to use. Driven from hdr_r, which
  // changes when the next request is accepted. That can happen while this
  // packet's last beat still waits in the upsizer's output register, so a
  // one-beat packet held by m_axis_cq_tready can show the next request's
  // byte enables.
  always_comb begin
    m_axis_cq_tuser        = '0;
    m_axis_cq_tuser[3:0]   = hdr_r.first_be;
    m_axis_cq_tuser[7:4]   = hdr_r.last_be;
  end

  // -------------------------------------------------------------------------
  // FSM
  // -------------------------------------------------------------------------
  typedef enum logic [1:0] {
    S_IDLE,     // waiting for a request header
    S_DESC,     // pushing the 4 descriptor Dwords into the gearbox
    S_PAYLOAD,  // forwarding request payload into the gearbox
    S_DRAIN     // swallowing the payload of a dropped write
  } cq_state_e;

  cq_state_e  state_r;
  logic [2:0] desc_idx_r;   // 0..3

  // The pending auto-UR, one slot. While it is full no request is accepted
  // (target_request_ready_o), so a second drop cannot overwrite it.
  logic        ur_valid_r;
  tlp_header_t ur_hdr_r;
  logic [12:0] ur_byte_count_r;

  assign ur_valid_o      = ur_valid_r;
  assign ur_header_o     = ur_hdr_r;
  assign ur_byte_count_o = ur_byte_count_r;

  // Requests are taken only in S_IDLE with the UR slot empty, so hdr_r stays
  // fixed while its descriptor is pushed. Holding this low stalls tlp_parser
  // in RX_HEADER instead of losing the request. tlp_parser handles requests
  // and completions in one state machine, so a completion behind a stalled
  // request waits too.
  assign target_request_ready_o = (state_r == S_IDLE) && !ur_valid_r;

  // -------------------------------------------------------------------------
  // Descriptor/payload gearbox, 32 -> 128.
  // -------------------------------------------------------------------------
  logic [TL_DATA_WIDTH-1:0]   gb_tdata;
  logic [TL_KEEP_WIDTH-1:0]   gb_tkeep;
  logic                       gb_tvalid, gb_tlast, gb_tready;
  logic [AXIS_DATA_WIDTH-1:0] gb_m_tdata;
  logic [AXIS_BYTE_KEEP-1:0]  gb_m_tkeep;

  always_comb begin
    gb_tdata  = target_data_i;
    gb_tkeep  = target_keep_i;
    gb_tvalid = 1'b0;
    gb_tlast  = 1'b0;
    unique case (state_r)
      S_DESC: begin
        unique case (desc_idx_r)
          3'd0:    gb_tdata = desc_bits[31:0];
          3'd1:    gb_tdata = desc_bits[63:32];
          3'd2:    gb_tdata = desc_bits[95:64];
          default: gb_tdata = desc_bits[127:96];
        endcase
        gb_tkeep  = {TL_KEEP_WIDTH{1'b1}};
        gb_tvalid = 1'b1;
        // A read is a descriptor-only packet: tlast lands on the fourth
        // descriptor Dword and the gearbox emits the beat at once.
        gb_tlast  = (desc_idx_r == 3'(DESC_DWORDS - 1)) && !has_data_r;
      end
      S_PAYLOAD: begin
        gb_tvalid = target_data_valid_i;
        // Counter-derived, like pcie_rc_if's: the header's own Dword Count
        // decides where the packet ends. The stream's own last is ORed in so a
        // short payload cannot wedge the gearbox mid-word; the disagreement is
        // reported below. Through tlp_layer the two always agree, since
        // tlp_parser derives the payload's last from Length.
        gb_tlast  = (dw_rem_r == 12'd1) || target_data_last_i;
      end
      default: ;
    endcase
  end

  // Payload is taken in S_PAYLOAD (forwarded) and in S_DRAIN (swallowed). A
  // dropped write must still have its payload consumed, or tlp_parser stalls
  // in RX_REPLAY with nothing to accept it.
  assign target_data_ready_o = (state_r == S_PAYLOAD) ? gb_tready :
                               (state_r == S_DRAIN)   ? 1'b1 : 1'b0;

  wire gb_beat = gb_tvalid && gb_tready;

  pcie_axis_dw_upsize #(
      .DATA_WIDTH_NARROW(TL_DATA_WIDTH),
      .DATA_WIDTH_WIDE  (AXIS_DATA_WIDTH)
  ) u_cq_pack (
      .clk_i(clk_i), .rst_i(rst_i),
      .s_axis_tdata (gb_tdata),  .s_axis_tkeep (gb_tkeep),
      .s_axis_tvalid(gb_tvalid), .s_axis_tlast (gb_tlast),
      .s_axis_tready(gb_tready),
      .m_axis_tdata (gb_m_tdata), .m_axis_tkeep(gb_m_tkeep),
      .m_axis_tvalid(m_axis_cq_tvalid), .m_axis_tlast(m_axis_cq_tlast),
      .m_axis_tready(m_axis_cq_tready),
      .gearbox_error_o(cq_gearbox_error_o)
  );

  assign m_axis_cq_tdata = gb_m_tdata;

  // Byte-granular to Dword-granular, as PG213 defines m_axis_cq_tkeep. A
  // request payload is a whole number of Dwords, with byte significance
  // carried by first_be and last_be, so each nibble is 0h or Fh and the
  // reduction loses nothing.
  always_comb begin
    for (int d = 0; d < AXIS_KEEP_WIDTH; d++)
      m_axis_cq_tkeep[d] = |gb_m_tkeep[d*4 +: 4];
  end

  // -------------------------------------------------------------------------
  // Sequential
  // -------------------------------------------------------------------------
  always_ff @(posedge clk_i) begin
    if (rst_i) begin
      state_r         <= S_IDLE;
      hdr_r           <= '0;
      bar_r           <= '0;
      req_type_r      <= CQ_MEM_READ;
      desc_idx_r      <= 3'd0;
      dw_rem_r        <= '0;
      has_data_r      <= 1'b0;
      cq_dropped_o    <= 1'b0;
      cq_error_code_o <= CQ_DROP_NONE;
      ur_valid_r      <= 1'b0;
      ur_hdr_r        <= '0;
      ur_byte_count_r <= '0;
    end else begin
      cq_dropped_o <= 1'b0;
      if (ur_valid_r && ur_ready_i) ur_valid_r <= 1'b0;

      unique case (state_r)
        S_IDLE: if (target_request_valid_i && target_request_ready_o) begin
          hdr_r      <= target_request_header_i;
          bar_r      <= target_bar_i;
          req_type_r <= offered_type;
          has_data_r <= offered_has_data;
          dw_rem_r   <= offered_has_data ?
                        {1'b0, target_request_header_i.length_dw} : 12'd0;
          desc_idx_r <= 3'd0;
          if (offered_deliver) begin
            state_r <= S_DESC;
          end else begin
            // A request that is not delivered is always reported, with its
            // reason.
            cq_dropped_o    <= 1'b1;
            cq_error_code_o <= offered_drop_code;
            $warning("pcie_cq_if: inbound request dropped, reason %0d (type %0d, addr 0x%016h, tag %0d) -- no CQ packet emitted",
                     offered_drop_code, offered_type,
                     target_request_header_i.address, target_request_header_i.tag);
            // UR, not CA: an unsupported request that requires a Completion
            // gets status UR, and CA is wrong wherever a conventional PCI
            // target would not have claimed the request (PCIe Base Spec r2.1,
            // §2.3.1).
            if (offered_non_posted) begin
              ur_valid_r      <= 1'b1;
              ur_hdr_r        <= target_request_header_i;
              // The number of enabled bytes. For a Memory Read it equals the
              // Byte Count of PCIe Base Spec r2.1, §2.3.1.1 only for
              // contiguous byte enables with a non-zero first_be. An I/O or
              // Configuration Completion must carry 4 (PCIe Base Spec r2.1,
              // §2.2.9), which this gives only when first_be is 1111b.
              ur_byte_count_r <= rq_byte_count(target_request_header_i.length_dw,
                                               target_request_header_i.first_be,
                                               target_request_header_i.last_be);
            end
            state_r <= offered_has_data ? S_DRAIN : S_IDLE;
          end
        end

        S_DESC: if (gb_beat) begin
          if (desc_idx_r == 3'(DESC_DWORDS - 1))
            state_r <= has_data_r ? S_PAYLOAD : S_IDLE;
          else
            desc_idx_r <= desc_idx_r + 3'd1;
        end

        S_PAYLOAD: if (gb_beat) begin
          dw_rem_r <= dw_rem_r - 12'd1;
          if (target_data_last_i && (dw_rem_r != 12'd1)) begin
            // The payload stopped short of the header's Dword Count. The CQ
            // descriptor already carries that count, so the packet goes out
            // truncated and flagged rather than being held open forever.
            cq_dropped_o    <= 1'b1;
            cq_error_code_o <= CQ_DROP_EARLY_LAST;
            $warning("pcie_cq_if: request payload ended %0d Dwords before the header's Dword Count %0d",
                     dw_rem_r - 12'd1, hdr_r.length_dw);
            state_r <= S_IDLE;
          end else if (!target_data_last_i && (dw_rem_r == 12'd1)) begin
            cq_dropped_o    <= 1'b1;
            cq_error_code_o <= CQ_DROP_MISSING_LAST;
            $warning("pcie_cq_if: request payload continued past the header's Dword Count %0d",
                     hdr_r.length_dw);
            state_r <= S_IDLE;
          end else if (dw_rem_r == 12'd1) begin
            state_r <= S_IDLE;
          end
        end

        // Swallow a dropped write's payload.
        S_DRAIN: if (target_data_valid_i && target_data_ready_o &&
                     target_data_last_i)
          state_r <= S_IDLE;

        default: state_r <= S_IDLE;
      endcase
    end
  end

endmodule
