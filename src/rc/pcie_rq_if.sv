// ---------------------------------------------------------------------------
// pcie_rq_if -- PG213 Requester Request (RQ) stream to the TL command port
//
// Author: Kourosh Ghahramani
// Silicon Systems Research Lab, University of Washington
//
// Purpose
//   Accepts host requests in PG213's Dword-aligned RQ format at 128 bits: a
//   16-byte descriptor on beat 0, then the payload. The descriptor is checked
//   and becomes one command on tlp_layer's command_* port; the payload is
//   narrowed to 32 bits by pcie_axis_dw_downsize and masked with first_be and
//   last_be. A descriptor that fails a check is reported, its packet is
//   discarded and no command is issued. The transmit path below this module
//   has no tlp_validator instance (only tlp_parser and tlp_classifier have
//   one), and tlp_requester checks only a request's byte count, so a request
//   meets no other legality check on its way to the link.
//
// Interfaces
//   RQ stream     s_axis_rq_*: 128-bit beats. tuser[3:0] is first_be and
//                 tuser[7:4] last_be; tkeep and the rest of tuser are not read.
//   Tag           allocated_tag_i, allocated_tag_valid_i: tlp_layer's tag
//                 strobe. pcie_rq_tag_o, pcie_rq_tag_vld_o: the same strobe,
//                 one cycle later, for the host.
//   Command       command_*: tlp_layer's command port. command_context_o
//                 carries the Lower Address echo that pcie_rc_if reads back.
//   Errors        rq_protocol_error_o, rq_error_code_o: a rejected descriptor
//                 or a payload that ends early or late. rq_gearbox_error_o:
//                 forwarded from the gearbox.
//
// Clock and reset
//   clk_i only. rst_i is synchronous and active high.
//
// Limitations
//   AtomicOp, Locked Read and Message requests are rejected, and so are
//   non-contiguous byte enables and zero-length requests. Force ECRC, the
//   descriptor's Tag and Requester ID, and the Poisoned bit of any request
//   other than a Configuration Write are not forwarded. TC and Attr are
//   forwarded unchecked, although I/O and Configuration Requests must carry
//   TC 000b and Attr[1:0] 00b (PCIe Base Spec r2.1, §2.2.7). A Type 0
//   Configuration Request to a device other than 0 is forwarded with a
//   $warning, where a Root Port without ARI Forwarding must complete it with
//   UR. A Memory or I/O Write with Dword Count 2 and last_be 0000b is not
//   rejected for that, and holds the module in S_FLUSH until reset (bad_be).
//   So does a packet that runs past its Dword Count when tlp_requester takes
//   the last Dword in the cycle the surplus ends (S_DRAIN).
//
// Structure
//   Descriptor decode      request type, payload flag, command address
//   Legality checks        the reject conditions and their priority
//   Control state machine  states and registers
//   Payload gearbox        the 128-to-32 downsizer and its whole-Dword keeps
//   Byte-enable mask       narrow-side masking and the counted last
//   Command port           command_* from the registers loaded on beat 0
//   AXIS ready             s_axis_rq_tready per state
//   Sequential             tag strobe, descriptor accept, payload and abort
//
// References
//   PG213, Table 13
//   PG213, Table 14
//   PG213, Table 57
//   PG213, Table 60
//   PG213, Table 61
//   PCIe Base Spec r2.1, §2.2.5
//   PCIe Base Spec r2.1, §2.2.7
//   PCIe Base Spec r2.1, §2.2.9
//   PCIe Base Spec r2.1, §7.3.1
// ---------------------------------------------------------------------------
`timescale 1ns/1ps
module pcie_rq_if
  import tlp_pkg::*;
  import pcie_rq_rc_pkg::*;
#(
    parameter int AXIS_DATA_WIDTH = 128,
    // PG213's s_axis_rq_tkeep has one bit per Dword (Table 13).
    parameter int AXIS_KEEP_WIDTH = AXIS_DATA_WIDTH / 32,
    parameter int AXIS_USER_WIDTH = 60,
    parameter int TL_DATA_WIDTH   = 32,
    parameter int TL_KEEP_WIDTH   = TL_DATA_WIDTH / 8,
    parameter int CONTEXT_WIDTH   = 16
) (
    input  logic                        clk_i,
    input  logic                        rst_i,

    // ---- PG213 Requester Request AXI4-Stream slave -----------------------
    input  logic [AXIS_DATA_WIDTH-1:0]  s_axis_rq_tdata,
    // Not read: the Dword Count decides which Dwords of a beat are payload.
    input  logic [AXIS_KEEP_WIDTH-1:0]  s_axis_rq_tkeep,
    input  logic                        s_axis_rq_tvalid,
    input  logic                        s_axis_rq_tlast,
    // [3:0] first_be, [7:4] last_be, sampled on beat 0 (PG213, Table 14).
    input  logic [AXIS_USER_WIDTH-1:0]  s_axis_rq_tuser,
    output logic                        s_axis_rq_tready,

    // ---- core-managed tag presentation -----------------------------------
    // Wired to tlp_layer's allocated_tag_o / allocated_tag_valid_o: the tag
    // tlp_request_tracker hands out, which goes into the emitted header and
    // comes back in the completion. It exists only once tlp_requester reaches
    // REQ_TAG, at least one cycle after the command is accepted, so it travels
    // with its own valid strobe, as pcie_rq_tag in PG213 does (Table 13).
    //
    // One strobe per non-posted TLP, in issue order: a Memory Write never
    // enters REQ_TAG, and a segmented request strobes once per segment with
    // that segment's tag. The descriptor's Tag field is not read.
    input  logic [7:0]                  allocated_tag_i,
    input  logic                        allocated_tag_valid_i,
    output logic [7:0]                  pcie_rq_tag_o,
    output logic                        pcie_rq_tag_vld_o,

    // ---- Transaction Layer command port ----------------------------------
    output logic                        command_valid_o,
    input  logic                        command_ready_i,
    output tlp_cmd_e                    command_o,
    output logic [63:0]                 command_address_o,
    output logic [12:0]                 command_byte_count_o,
    output logic [2:0]                  command_tc_o,
    output logic [2:0]                  command_attr_o,
    output logic [CONTEXT_WIDTH-1:0]    command_context_o,
    output logic                        command_prefix_valid_o,
    output logic [31:0]                 command_prefix_o,
    output logic                        command_ecrc_enable_o,

    output logic [TL_DATA_WIDTH-1:0]    command_data_o,
    output logic [TL_KEEP_WIDTH-1:0]    command_keep_o,
    output logic                        command_data_valid_o,
    output logic                        command_data_last_o,
    input  logic                        command_data_ready_i,

    // ---- error surface ---------------------------------------------------
    // One-cycle pulse; rq_error_code_o is valid in the same cycle and holds
    // until the next pulse.
    output logic                        rq_protocol_error_o,
    output rq_error_e                   rq_error_code_o,
    // Forwarded from the payload gearbox: an illegal tkeep on its wide input.
    output logic                        rq_gearbox_error_o
);

  localparam int AXIS_DW = AXIS_DATA_WIDTH / 32;              // Dwords per beat
  localparam int AXIS_BYTE_KEEP = AXIS_DATA_WIDTH / 8;        // gearbox tkeep

  // -------------------------------------------------------------------------
  // Descriptor decode
  // -------------------------------------------------------------------------
  // Combinational, from the descriptor on s_axis_rq_tdata[127:0] and the byte
  // enables on s_axis_rq_tuser; used only in the cycle beat 0 is accepted.
  // Produces the TL command (desc_cmd) and its class (desc_is_config,
  // desc_is_io, desc_has_data), the byte offset (desc_off) and byte count
  // (desc_bc) from pcie_rq_rc_pkg's byte-enable arithmetic, and the command
  // address (desc_address).
  rq_descriptor_t desc;
  assign desc = rq_descriptor_t'(s_axis_rq_tdata[127:0]);

  wire [3:0]  desc_first_be = s_axis_rq_tuser[3:0];
  wire [3:0]  desc_last_be  = s_axis_rq_tuser[7:4];
  wire [1:0]  desc_off      = rq_be_offset(desc_first_be);
  wire [10:0] desc_n        = desc.dword_count;
  wire [12:0] desc_bc       = rq_byte_count(desc_n, desc_first_be, desc_last_be);

  rq_req_type_e desc_type;
  assign desc_type = rq_req_type_e'(desc.req_type);

  logic     type_ok;
  tlp_cmd_e desc_cmd;
  logic     desc_is_config, desc_is_io, desc_has_data;

  always_comb begin
    type_ok        = 1'b1;
    desc_cmd       = TLP_CMD_MEM_READ;
    desc_is_config = 1'b0;
    desc_is_io     = 1'b0;
    unique case (desc_type)
      RQ_MEM_READ:   desc_cmd = TLP_CMD_MEM_READ;
      RQ_MEM_WRITE:  desc_cmd = TLP_CMD_MEM_WRITE;
      RQ_IO_READ:  begin desc_cmd = TLP_CMD_IO_READ;    desc_is_io     = 1'b1; end
      RQ_IO_WRITE: begin desc_cmd = TLP_CMD_IO_WRITE;   desc_is_io     = 1'b1; end
      RQ_CFG_READ0:  begin desc_cmd = TLP_CMD_CFG_READ0;  desc_is_config = 1'b1; end
      RQ_CFG_WRITE0: begin desc_cmd = TLP_CMD_CFG_WRITE0; desc_is_config = 1'b1; end
      // desc_is_config is all that bad_cfg_n, bad_cfg_fit, bad_at and the
      // address assembly below need: they test the class, not the command,
      // so they apply to Type 0 and Type 1 alike.
      RQ_CFG_READ1:  begin desc_cmd = TLP_CMD_CFG_READ1;  desc_is_config = 1'b1; end
      RQ_CFG_WRITE1: begin desc_cmd = TLP_CMD_CFG_WRITE1; desc_is_config = 1'b1; end
      default:       type_ok = 1'b0;
    endcase
  end

  // Listed per command, like bad_poison below, not by class: a write command
  // missing from this list is treated as payload-less, and its packet is then
  // rejected with RQ_ERR_MISSING_LAST.
  assign desc_has_data = type_ok && (desc_cmd == TLP_CMD_MEM_WRITE ||
                                     desc_cmd == TLP_CMD_CFG_WRITE0 ||
                                     desc_cmd == TLP_CMD_CFG_WRITE1 ||
                                     desc_cmd == TLP_CMD_IO_WRITE);

  // A Configuration Request's address is its header's third Dword: Bus,
  // Device and Function in [31:16], Extended Register Number in [11:8] and
  // Register Number in [7:2] (PCIe Base Spec r2.1, §2.2.7). tlp_generator
  // sends address[31:2] with [1:0] as zero, so the byte offset placed in
  // [1:0] sets the byte enables without changing the Register Number.
  logic [63:0] desc_address;
  always_comb begin
    if (desc_is_config) begin
      desc_address          = 64'd0;
      desc_address[31:16]   = desc.completer_id;   // {Bus, Dev, Fn}
      desc_address[15:12]   = 4'h0;                // reserved in the config DW
      desc_address[11:8]    = desc.address[11:8];  // Ext Reg Number
      desc_address[7:2]     = desc.address[7:2];   // Register Number
      desc_address[1:0]     = desc_off;            // byte offset -> first_be
    end else begin
      desc_address          = {desc.address[63:2], desc_off};
    end
  end

  // -------------------------------------------------------------------------
  // Legality checks
  // -------------------------------------------------------------------------
  // Evaluated together on beat 0. The first failing check, in the order of
  // the desc_error chain below, names the error code. A rejected descriptor
  // issues no command, and the rest of its packet is drained.
  wire bad_type      = !type_ok;
  wire bad_n         = (desc_n == 11'd0) || (desc_n > 11'd1024);
  wire bad_cfg_n     = desc_is_config && (desc_n != 11'd1);
  // The fit rule byte_count <= 4 - offset, as tlp_requester's admission guard
  // applies it. PCIe constrains a Configuration Request's Length, not its
  // byte enables (PCIe Base Spec r2.1, §2.2.7), so a partial write such as
  // pcie_enum_bar's Command register write (first_be 0011b) must pass.
  wire bad_cfg_fit   = (desc_is_config || desc_is_io) &&
                       (desc_bc > (13'd4 - {11'd0, desc_off}));
  // A Memory request must not cross a 4-KB boundary (PCIe Base Spec r2.1,
  // §2.2.7).
  wire bad_4kb       = ({1'b0, desc_address[11:0]} + {1'b0, desc_bc}) > 14'd4096;
  // An I/O Request must carry AT 00b (PCIe Base Spec r2.1, §2.2.7), and
  // tlp_requester has no AT input, so a Memory request with another AT would
  // go out with AT 00b.
  wire bad_at        = !desc_is_config && (desc.address[1:0] != 2'b00);
  // Only the two Configuration Writes: PG213 supports the Poisoned bit on
  // every other request type (Tables 60 and 61). tlp_requester has no poison
  // input, so a poisoned I/O or Memory Write goes out unpoisoned.
  wire bad_poison    = (desc_cmd == TLP_CMD_CFG_WRITE0 ||
                        desc_cmd == TLP_CMD_CFG_WRITE1) && desc.poisoned;
  // An explicit guard only: once bad_n has passed, rq_byte_count is at most
  // 4096, so this never fires.
  wire bad_bc_fit    = desc_bc > 13'd4096;
  // Not caught by the round trip below, since tlp_first_be(0, 0) == 0 agrees
  // with a first_be of 0. tlp_requester accepts a byte count of 0 only for a
  // Memory Read; this module rejects every zero-length request.
  wire bad_zero_len  = (desc_n == 11'd1) && (desc_first_be == 4'h0);
  // The round trip: given (offset, byte_count), tlp_first_be and tlp_last_be
  // must rebuild exactly the descriptor's byte enables. They build contiguous
  // masks only, so this rejects non-contiguous byte enables, which PCIe
  // permits on 1-Dword requests and QW-aligned 2-Dword Memory requests (PCIe
  // Base Spec r2.1, §2.2.5). It does not compare Lengths: Dword Count 2 with
  // last_be 0000b passes, given a contiguous or zero first_be. tlp_requester
  // sends that as Length 1, or rejects it when the byte count is 0 and it is
  // not a Memory Read; a Memory or I/O Write of this shape then waits in
  // S_FLUSH, until reset, for a Dword tlp_requester never takes.
  wire bad_be        = (tlp_first_be(desc_off, desc_bc) != desc_first_be) ||
                       (tlp_last_be (desc_off, desc_bc) != desc_last_be);
  // A write whose packet ends on the descriptor beat carries no payload at all.
  wire bad_early_hdr = desc_has_data && s_axis_rq_tlast;
  // A read (or a rejected request) whose packet does not end on beat 0.
  wire bad_extra_hdr = !desc_has_data && !s_axis_rq_tlast;

  logic      desc_reject;
  rq_error_e desc_error;
  always_comb begin
    desc_reject = 1'b1;
    if      (bad_type)      desc_error = RQ_ERR_REQ_TYPE;
    else if (bad_n)         desc_error = RQ_ERR_DWORD_COUNT;
    else if (bad_cfg_n)     desc_error = RQ_ERR_CFG_DWORD_COUNT;
    else if (bad_zero_len)  desc_error = RQ_ERR_ZERO_LENGTH;
    else if (bad_be)        desc_error = RQ_ERR_BE_MISMATCH;
    else if (bad_cfg_fit)   desc_error = RQ_ERR_CFG_IO_FIT;
    else if (bad_at)        desc_error = RQ_ERR_ADDRESS_TYPE;
    else if (bad_4kb)       desc_error = RQ_ERR_4KB;
    else if (bad_poison)    desc_error = RQ_ERR_POISON_CFG_WR;
    else if (bad_bc_fit)    desc_error = RQ_ERR_BYTE_COUNT_FIT;
    else if (bad_early_hdr) desc_error = RQ_ERR_EARLY_LAST;
    else if (bad_extra_hdr) desc_error = RQ_ERR_MISSING_LAST;
    else begin
      desc_error  = RQ_ERR_NONE;
      desc_reject = 1'b0;
    end
  end

  // -------------------------------------------------------------------------
  // Control state machine
  // -------------------------------------------------------------------------
  //   S_DESC         takes and checks beat 0. A reject goes to S_DRAIN, or
  //                  stays if the packet ended; an admitted write goes to
  //                  S_PAYLOAD; an admitted read stays.
  //   S_PAYLOAD      feeds payload beats to the gearbox. S_FLUSH at the
  //                  counted last beat, S_ABORT_FLUSH on an early tlast,
  //                  S_DRAIN when tlast is missing at the count.
  //   S_FLUSH        waits for the TL to take the last Dword, then S_DESC.
  //   S_ABORT_FLUSH  discards the gearbox contents, then S_ABORT_TERM.
  //   S_ABORT_TERM   offers the zero-keep terminating beat; S_DESC once taken.
  //   S_DRAIN        swallows beats to tlast, then S_FLUSH if drain_owes_tl_r
  //                  is set and dw_sent_r is short of n_r, else S_DESC.
  typedef enum logic [2:0] {
    S_DESC,        // accepting beat 0
    S_PAYLOAD,     // forwarding payload beats into the gearbox
    S_FLUSH,       // AXIS done; draining the gearbox into the TL
    S_ABORT_FLUSH, // malformed mid-payload: discarding the gearbox contents
    S_ABORT_TERM,  // emitting the terminating zero-keep beat
    S_DRAIN        // swallowing the rest of a rejected or overlong AXIS packet
  } rq_state_e;

  rq_state_e state_r;

  tlp_cmd_e    cmd_r;
  logic [63:0] addr_r;
  logic [12:0] bc_r;
  logic [2:0]  tc_r, attr_r;
  logic        mem_read_r;   // context[12]: addr_r[11:0] is a real byte address
  logic [10:0] n_r;          // descriptor Dword Count
  logic [3:0]  first_be_r, last_be_r;
  logic [11:0] dw_rem_r;     // payload Dwords not yet handed to the gearbox
  logic [10:0] dw_sent_r;    // payload Dwords accepted by the TL
  logic        cmd_pending_r;
  // Set only on the RQ_ERR_MISSING_LAST path: the AXIS packet overran its own
  // Dword Count, so the surplus beats must be swallowed while the TL is still
  // being fed the Dwords it was promised. Without this the drain would gate
  // command_data_valid_o off and strand tlp_requester in REQ_DATA.
  logic        drain_owes_tl_r;

  // -------------------------------------------------------------------------
  // Payload gearbox, 128 to 32 bits
  // -------------------------------------------------------------------------
  // Fed whole-Dword byte keeps only (beat_keep), counted from the Dword Count
  // and contiguous from bit 0, so its input tkeep is always legal. The byte
  // enables are applied to its output instead: a byte-granular mask such as
  // first_be 0010b at its input would be a tkeep the gearbox flags as
  // illegal. In S_PAYLOAD at least one Dword remains, so beat_keep is never
  // zero and rq_gearbox_error_o cannot pulse.
  logic [AXIS_DATA_WIDTH-1:0] pay_s_tdata;
  logic [AXIS_BYTE_KEEP-1:0]  pay_s_tkeep;
  logic                       pay_s_tvalid, pay_s_tlast, pay_s_tready;
  logic [TL_DATA_WIDTH-1:0]   pay_m_tdata;
  logic [TL_KEEP_WIDTH-1:0]   pay_m_tkeep;
  logic                       pay_m_tvalid, pay_m_tlast, pay_m_tready;

  pcie_axis_dw_downsize #(
      .DATA_WIDTH_WIDE  (AXIS_DATA_WIDTH),
      .DATA_WIDTH_NARROW(TL_DATA_WIDTH)
  ) u_payload (
      .clk_i(clk_i), .rst_i(rst_i),
      .s_axis_tdata (pay_s_tdata),  .s_axis_tkeep (pay_s_tkeep),
      .s_axis_tvalid(pay_s_tvalid), .s_axis_tlast (pay_s_tlast),
      .s_axis_tready(pay_s_tready),
      .m_axis_tdata (pay_m_tdata),  .m_axis_tkeep (pay_m_tkeep),
      .m_axis_tvalid(pay_m_tvalid), .m_axis_tlast (pay_m_tlast),
      .m_axis_tready(pay_m_tready),
      .gearbox_error_o(rq_gearbox_error_o)
  );

  // Dwords this wide beat should carry, and the resulting all-ones byte keep.
  wire [11:0] beat_dw   = (dw_rem_r >= 12'(AXIS_DW)) ? 12'(AXIS_DW) : dw_rem_r;
  wire        beat_last = dw_rem_r <= 12'(AXIS_DW);

  logic [AXIS_BYTE_KEEP-1:0] beat_keep;
  always_comb begin
    beat_keep = '0;
    for (int d = 0; d < AXIS_DW; d++)
      if (12'(d) < beat_dw) beat_keep[d*4 +: 4] = 4'hF;
  end

  assign pay_s_tdata  = s_axis_rq_tdata;
  assign pay_s_tkeep  = beat_keep;
  assign pay_s_tvalid = (state_r == S_PAYLOAD) && s_axis_rq_tvalid;
  // tlast into the gearbox is this module's own count, or the host's when the
  // host ends early. In that case it marks the last Dword of the discarded
  // beat, which S_ABORT_FLUSH waits for so that the gearbox is left empty.
  assign pay_s_tlast  = beat_last || s_axis_rq_tlast;

  // -------------------------------------------------------------------------
  // Byte-enable mask and the counted last
  // -------------------------------------------------------------------------
  // command_byte_count_o and command_data_last_o both come from the
  // descriptor's Dword Count: the byte count through rq_byte_count, the last
  // flag from dw_sent_r reaching n_r - 1, never from s_axis_rq_tlast.
  // tlp_requester raises TLP_ERR_LOCAL_PAYLOAD when command_data_last_i
  // disagrees with the end of the whole request it derives from the byte
  // count; the two agree whenever the last Dword has a byte enabled (see
  // bad_be for the one admitted exception). S_ABORT_TERM's beat is the
  // deliberate disagreement.
  wire first_dw = dw_sent_r == 11'd0;
  wire last_dw  = dw_sent_r == (n_r - 11'd1);

  wire [3:0] keep_mask = (first_dw ? first_be_r : 4'hF) &
                         ((last_dw && (n_r > 11'd1)) ? last_be_r : 4'hF);

  wire payload_to_tl = ((state_r == S_PAYLOAD) || (state_r == S_FLUSH) ||
                        ((state_r == S_DRAIN) && drain_owes_tl_r)) && !cmd_pending_r;

  assign command_data_o       = (state_r == S_ABORT_TERM) ? '0 : pay_m_tdata;
  assign command_keep_o       = (state_r == S_ABORT_TERM) ? '0
                                                          : (pay_m_tkeep & keep_mask);
  assign command_data_valid_o = (state_r == S_ABORT_TERM) ? 1'b1
                                                          : (payload_to_tl && pay_m_tvalid);
  // Derived from n_r and dw_sent_r only, never from s_axis_rq_tlast.
  assign command_data_last_o  = (state_r == S_ABORT_TERM) ? 1'b1
                                                          : (payload_to_tl && last_dw);

  assign pay_m_tready = (state_r == S_ABORT_FLUSH) ? 1'b1
                                                   : (payload_to_tl && command_data_ready_i);

  // -------------------------------------------------------------------------
  // Command port
  // -------------------------------------------------------------------------
  // Driven from the registers loaded when beat 0 is accepted; command_valid_o
  // is cmd_pending_r. command_context_o is the context echo that
  // tlp_request_tracker returns with each result as result_context_o:
  // pcie_rc_if rebuilds the RC descriptor's Lower Address [11:7] from it,
  // since a Completion header carries only [6:0]. pcie_rq_rc_top does not
  // export the context, so a host correlates completions by tag.
  assign command_valid_o        = cmd_pending_r;
  assign command_o              = cmd_r;
  assign command_address_o      = addr_r;
  assign command_byte_count_o   = bc_r;
  assign command_tc_o           = tc_r;
  assign command_attr_o         = attr_r;
  // [11:0] is the request's address[11:0]. [12] marks it as a byte address,
  // true only for a Memory Read: every other Completion carries Lower Address
  // 0 (PCIe Base Spec r2.1, §2.2.9), and a Configuration Request's addr_r
  // holds a BDF and register number. A Memory Write has no completion.
  assign command_context_o      = {{(CONTEXT_WIDTH-13){1'b0}}, mem_read_r,
                                   addr_r[11:0]};
  assign command_prefix_valid_o = 1'b0;   // TLP prefixes out of scope
  assign command_prefix_o       = 32'd0;
  assign command_ecrc_enable_o  = 1'b0;   // no ECRC; Force ECRC is not read

  // -------------------------------------------------------------------------
  // AXIS ready
  // -------------------------------------------------------------------------
  // Beat 0 is taken only when no command is waiting for tlp_requester, so the
  // command registers are never overwritten while command_valid_o is high.
  // Payload beats move at the gearbox's pace, S_DRAIN takes every beat, and
  // the flush and abort states take none.
  always_comb begin
    unique case (state_r)
      S_DESC:    s_axis_rq_tready = !cmd_pending_r;
      S_PAYLOAD: s_axis_rq_tready = pay_s_tready;
      S_DRAIN:   s_axis_rq_tready = 1'b1;
      default:   s_axis_rq_tready = 1'b0;
    endcase
  end

  wire desc_beat = (state_r == S_DESC) && s_axis_rq_tvalid && s_axis_rq_tready;
  wire pay_beat  = (state_r == S_PAYLOAD) && s_axis_rq_tvalid && s_axis_rq_tready;
  wire tl_beat   = command_data_valid_o && command_data_ready_i;

  // -------------------------------------------------------------------------
  // Sequential
  // -------------------------------------------------------------------------
  // Registers the tag strobe, the command and the error pulse, and advances
  // the state machine. A payload that ends early is handled in one of two
  // ways. A write whose packet ends on beat 0 is rejected before any command
  // exists. A packet that ends mid-payload already has its command issued, so
  // the gearbox is flushed and S_ABORT_TERM sends one beat with
  // command_keep_o 0 and command_data_last_o 1; without it tlp_requester
  // would wait in REQ_DATA for payload that never comes. tlp_requester
  // reports TLP_ERR_LOCAL_PAYLOAD and returns to REQ_IDLE. It closes the TLP
  // it has started with fewer payload bytes than the Length in its header.
  always_ff @(posedge clk_i) begin
    if (rst_i) begin
      state_r             <= S_DESC;
      cmd_r               <= TLP_CMD_MEM_READ;
      addr_r              <= '0;
      bc_r                <= '0;
      tc_r                <= '0;
      attr_r              <= '0;
      mem_read_r          <= 1'b0;
      n_r                 <= 11'd1;
      first_be_r          <= 4'h0;
      last_be_r           <= 4'h0;
      dw_rem_r            <= '0;
      dw_sent_r           <= '0;
      cmd_pending_r       <= 1'b0;
      drain_owes_tl_r     <= 1'b0;
      pcie_rq_tag_o       <= 8'h00;
      pcie_rq_tag_vld_o   <= 1'b0;
      rq_protocol_error_o <= 1'b0;
      rq_error_code_o     <= RQ_ERR_NONE;
    end else begin
      rq_protocol_error_o <= 1'b0;
      pcie_rq_tag_vld_o   <= 1'b0;

      if (cmd_pending_r && command_ready_i) cmd_pending_r <= 1'b0;
      if (tl_beat && (state_r != S_ABORT_TERM)) dw_sent_r <= dw_sent_r + 11'd1;

      // Taken from tlp_layer's allocation strobe, not from the descriptor
      // accept, because the tag does not exist at accept time. Registered, so
      // no combinational path runs from inside tlp_layer to the host.
      if (allocated_tag_valid_i) begin
        pcie_rq_tag_o     <= allocated_tag_i;
        pcie_rq_tag_vld_o <= 1'b1;
      end

      unique case (state_r)
        S_DESC: if (desc_beat) begin
          if (desc_reject) begin
            rq_protocol_error_o <= 1'b1;
            rq_error_code_o     <= desc_error;
            $warning("pcie_rq_if: rejected RQ descriptor (code %0d, type %0d, dw_count %0d, first_be 0x%0h, last_be 0x%0h)",
                     desc_error, desc.req_type, desc_n, desc_first_be, desc_last_be);
            drain_owes_tl_r <= 1'b0;
            state_r         <= s_axis_rq_tlast ? S_DESC : S_DRAIN;
          end else begin
            // A Root Port without ARI Forwarding must complete a Type 0
            // Configuration Request to a device other than 0 with UR (PCIe
            // Base Spec r2.1, §7.3.1); this module forwards it unchanged and
            // only warns. Type 1 is exempt: it may name any device on the bus
            // behind a bridge.
            if ((desc_cmd == TLP_CMD_CFG_READ0 ||
                 desc_cmd == TLP_CMD_CFG_WRITE0) &&
                desc.completer_id[7:3] != 5'd0)
              $warning("pcie_rq_if: Type 0 config request to device %0d (BDF 0x%04h) admitted and forwarded unchanged -- Base 2.1 SS7.3.1 p.479 wants UR termination once a sweep-capable requester exists (deferred, Stage D brief SS8.1)",
                       desc.completer_id[7:3], desc.completer_id);
            cmd_r         <= desc_cmd;
            addr_r        <= desc_address;
            bc_r          <= desc_bc;
            tc_r          <= desc.tc;
            attr_r        <= desc.attr;
            mem_read_r    <= desc_cmd == TLP_CMD_MEM_READ;
            n_r           <= desc_n;
            first_be_r    <= desc_first_be;
            last_be_r     <= desc_last_be;
            dw_rem_r      <= {1'b0, desc_n};
            dw_sent_r     <= '0;
            cmd_pending_r <= 1'b1;
            state_r <= desc_has_data ? S_PAYLOAD : S_DESC;
          end
        end

        S_PAYLOAD: if (pay_beat) begin
          dw_rem_r <= dw_rem_r - beat_dw;
          if (s_axis_rq_tlast && !beat_last) begin
            // The host ended the packet before the Dword Count: abort.
            rq_protocol_error_o <= 1'b1;
            rq_error_code_o     <= RQ_ERR_EARLY_LAST;
            $warning("pcie_rq_if: s_axis_rq_tlast %0d Dwords before the descriptor's Dword Count %0d was met",
                     dw_rem_r - beat_dw, n_r);
            state_r <= S_ABORT_FLUSH;
          end else if (!s_axis_rq_tlast && beat_last) begin
            // The host kept going past the Dword Count. The TL is owed exactly
            // the Dwords counted so far, so this request completes normally
            // while S_DRAIN swallows the surplus beats.
            rq_protocol_error_o <= 1'b1;
            rq_error_code_o     <= RQ_ERR_MISSING_LAST;
            $warning("pcie_rq_if: beats continue past the descriptor's Dword Count %0d", n_r);
            drain_owes_tl_r <= 1'b1;
            state_r         <= S_DRAIN;
          end else if (beat_last) begin
            state_r <= S_FLUSH;
          end
        end

        // AXIS side done; wait for the TL to take the last Dword.
        S_FLUSH: if (tl_beat && last_dw) state_r <= S_DESC;

        // Discard whatever the gearbox still holds so no fragment of the
        // malformed packet can prepend itself to the next one.
        S_ABORT_FLUSH: if (pay_m_tvalid && pay_m_tlast) state_r <= S_ABORT_TERM;

        S_ABORT_TERM: if (command_data_ready_i) state_r <= S_DESC;

        // If the TL is still owed Dwords when the surplus ends, S_FLUSH pays
        // them out; S_DESC would leave tlp_requester waiting in REQ_DATA.
        // dw_sent_r is read before this cycle's increment, so a last Dword
        // taken in the tlast cycle also selects S_FLUSH, which then holds
        // until reset with nothing left to send.
        S_DRAIN: if (s_axis_rq_tvalid && s_axis_rq_tlast) begin
          drain_owes_tl_r <= 1'b0;
          state_r <= (drain_owes_tl_r && (dw_sent_r != n_r)) ? S_FLUSH : S_DESC;
        end

        default: state_r <= S_DESC;
      endcase
    end
  end

endmodule
