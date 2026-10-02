// ---------------------------------------------------------------------------
// pcie_rq_rc_top -- Root Complex Transaction Layer with PG213-style interfaces
//
// Author: Kourosh Ghahramani
// Silicon Systems Research Lab, University of Washington
//
// Purpose
//   Puts PG213-style AXI4-Stream host interfaces on tlp_layer. pcie_rq_if
//   turns Requester Request descriptors into tlp_layer commands, pcie_rc_if
//   turns received completions into Requester Completion packets, pcie_cq_if
//   presents inbound requests as Completer Request packets, and pcie_cc_if
//   turns the host's Completer Completion packets into Completions. This
//   module is wiring only, with no registers: the descriptor, byte-enable,
//   tag and completion-matching rules live in those modules and tlp_layer.
//
//     s_axis_rq_*   -> pcie_rq_if -> tlp_layer  -> m_dllp_axis_*
//     s_dllp_axis_* -> tlp_layer  -> pcie_rc_if -> m_axis_rc_*
//                                 -> pcie_cq_if -> m_axis_cq_*
//     s_axis_cc_*   -> pcie_cc_if -> tlp_layer  -> m_dllp_axis_*
//
// Interfaces
//   Link and FC   link_up_i, transmit_enable_i, fc_initialized_i,
//                 fc_update_valid_i, fc_*_i: tlp_layer transmits nothing
//                 until all of them allow it.
//   Identity      requester_id_i, completer_id_i, bus_number_i,
//                 device_number_i, function_number_i, memory_enable_i and the
//                 negotiated limits: passed to tlp_layer unchanged.
//   Requester     s_axis_rq_*, pcie_rq_tag_o, pcie_rq_tag_vld_o, m_axis_rc_*:
//                 the host's requests, their tags and their completions.
//   Completer     m_axis_cq_*, s_axis_cc_*: inbound requests to the host and
//                 the host's completions for them.
//   DLL streams   s_dllp_axis_*, m_dllp_axis_*: TL_DATA_WIDTH-bit streams.
//   Status        rq_*, rc_*, cq_*, cc_* error outputs; tlp_layer's error,
//                 credit and Completion Timeout outputs; outstanding_o.
//
// Clock and reset
//   clk_i only. rst_i is synchronous and active high. A low link_up_i also
//   resets tlp_layer, but not the four interface modules.
//
// Limitations
//   - A tag is allocated before the credit gate: a pcie_rq_tag_vld_o strobe
//     does not mean the request reached the link.
//   - When link_up_i falls, outstanding tags end with no cpl_timeout_*
//     strobe, and an interface module caught mid-payload waits until rst_i.
//   - The host-memory aperture is one power-of-two window set by parameters.
//   - CPL_TIMEOUT_CYCLES fixes the timeout; no Device Control 2 register.
//   - No RC descriptor carries error code 1001 (timeout) or 0011 (PG213,
//     Table 66); a split read's later completions carry the first
//     completion's Lower Address [11:7].
//   - pcie_rq_if rejects non-contiguous byte enables, zero-length reads,
//     Atomic Operations, locked reads, Messages and ATS requests, forwards a
//     poisoned non-configuration write unpoisoned, and ignores Force ECRC.
//   - No m_axis_rc_tuser port; m_axis_cq_tuser carries only first_be and
//     last_be; s_axis_cc_tuser is not read.
//
// Structure
//   Ports
//   Host aperture
//   Command port: pcie_rq_if to tlp_layer
//   Received completions: tlp_layer to pcie_rc_if
//   Completer Completion: s_axis_cc_* to tlp_layer
//   Completer Request: tlp_layer to m_axis_cq_*
//   Requester Request: s_axis_rq_* to tlp_layer
//   Transaction Layer
//   Requester Completion: tlp_layer to m_axis_rc_*
//
// References
//   PCIe Base Spec r2.1, §2.3.1
//   PCIe Base Spec r2.1, §2.6.1
//   PCIe Base Spec r2.1, §2.8
//   PCIe Base Spec r2.1, §7.5.3
//   PG213, Table 9
//   PG213, Table 10
//   PG213, Table 11
//   PG213, Table 14
//   PG213, Table 52
//   PG213, Table 57
//   PG213, Table 58
//   PG213, Table 60
//   PG213, Table 61
//   PG213, Table 65
//   PG213, Table 66
// ---------------------------------------------------------------------------
`timescale 1ns/1ps
module pcie_rq_rc_top
  import tlp_pkg::*;
  import pcie_rq_rc_pkg::*;
#(
    parameter int AXIS_DATA_WIDTH = 128,
    // PG213 tkeep is DWORD-granular on both RQ and RC: one bit per Dword.
    parameter int AXIS_KEEP_WIDTH = AXIS_DATA_WIDTH / 32,
    parameter int AXIS_USER_WIDTH = 60,
    parameter int TL_DATA_WIDTH   = 32,
    parameter int TL_KEEP_WIDTH   = TL_DATA_WIDTH / 8,
    parameter int TL_USER_WIDTH   = 3,
    // PG213 sizes m_axis_cq_tuser at 88 bits and s_axis_cc_tuser at 33
    // (PG213, Table 9 and Table 11), and these widths let a PG213 client bind
    // unchanged. pcie_cq_if drives only first_be [3:0] and last_be [7:4] and
    // ties the other bits to 0. s_axis_cc_tuser carries discontinue and
    // parity, which pcie_cc_if does not read.
    parameter int CQ_USER_WIDTH   = 88,
    parameter int CC_USER_WIDTH   = 33,
    // ---- host memory aperture ----------------------------------------------
    // The window of host memory an inbound Memory request may target. A Root
    // Port claims a Memory request as a PCI-to-PCI bridge would, by its
    // bridge configuration rather than by a BAR (PCIe Base Spec r2.1, §2.3.1).
    // The window is tlp_layer's BAR 0, decoded only while memory_enable_i is
    // high. pcie_cq_if drops a Memory request outside it with CQ_DROP_NO_BAR.
    //
    // HOST_MEM_SIZE must be a power of two and HOST_MEM_BASE aligned to it:
    // tlp_bar_decoder matches (address & mask) == (base & mask), which only
    // expresses a naturally aligned power-of-two window. A bridge describes
    // its window with Memory Base and Memory Limit registers instead (PCIe
    // Base Spec r2.1, §7.5.3), a range this mask form cannot always express.
    parameter logic [63:0] HOST_MEM_BASE = 64'h0000_0000_0000_0000,
    parameter logic [63:0] HOST_MEM_SIZE = 64'h0000_0001_0000_0000,  // 4 GB
    parameter int CONTEXT_WIDTH   = 16,
    parameter int TAG_COUNT       = 32,
    // Completion Timeout; 0 disables. See tlp_request_tracker.sv header.
    parameter int unsigned CPL_TIMEOUT_CYCLES = tlp_pkg::CPL_TIMEOUT_DEFAULT_CYCLES,  // 10 ms at 8 ns
    // Byte order of TLP headers on the DLL streams (tlp_layer's
    // PCIE_WIRE_ORDER). The default, 0, keeps the header Dwords after DW0 in
    // host Dword order for benches that drive this module directly;
    // pcie_rc_dl_top and pcie_rc_top, which stack it on pcie_datalink_layer,
    // pass 1 so that every header Dword has its first wire byte on byte lane
    // 0, as DW0 always does.
    parameter bit PCIE_WIRE_ORDER = 1'b0
) (
    input  logic                        clk_i,
    input  logic                        rst_i,

    // ---- link state and flow control ---------------------------------------
    // tlp_layer transmits nothing until link_up_i, transmit_enable_i and
    // fc_initialized_i are high and an fc_update_valid_i pulse has captured
    // the link partner's initial advertisement. With link_up_i high,
    // s_axis_rq_* still accepts requests, which wait with no error output;
    // tx_fc_blocked_o rises once a packet waits with transmit_enable_i high.
    // A packet with data waiting then also raises credit_error_o if
    // fc_initialized_i is high before that first pulse.
    //
    // While link_up_i is low, tlp_layer is held in reset, yet tlp_requester
    // and tlp_completion_generator still signal ready: a request or host
    // completion offered then is discarded, and one with data leaves
    // pcie_rq_if or pcie_cc_if waiting for data ready until rst_i.
    //
    // An advertisement of 00h or 000h at initialization means infinite credit
    // for that pool (PCIe Base Spec r2.1, §2.6.1), and tlp_credit_manager
    // latches it as such: only a non-zero advertisement that is used up and
    // never updated starves a pool. A Configuration Read costs one NPH and no NPD
    // credit, a Configuration Write one NPH and one NPD.
    input  logic                        link_up_i,
    input  logic                        transmit_enable_i,
    input  logic                        fc_initialized_i,
    input  logic                        fc_update_valid_i,
    input  logic [7:0]                  fc_ph_i,
    input  logic [11:0]                 fc_pd_i,
    input  logic [7:0]                  fc_nph_i,
    input  logic [11:0]                 fc_npd_i,
    input  logic [7:0]                  fc_cplh_i,
    input  logic [11:0]                 fc_cpld_i,

    // ---- identity and negotiated limits ------------------------------------
    // requester_id_i is the ID that goes into every originated request header
    // and the one a completion must carry back to match. memory_enable_i
    // gates the host-aperture decode in tlp_bar_decoder.
    input  logic [15:0]                 requester_id_i,
    input  logic [15:0]                 completer_id_i,
    input  logic [7:0]                  bus_number_i,
    input  logic [4:0]                  device_number_i,
    input  logic [2:0]                  function_number_i,
    input  logic                        memory_enable_i,
    input  logic                        extended_tag_enable_i,
    input  logic [12:0]                 max_payload_bytes_i,
    input  logic [12:0]                 max_read_bytes_i,
    input  logic                        rcb_128b_i,

    // ---- PG213 Requester Request AXI4-Stream slave -------------------------
    // Beat 0 is the 16-byte RQ descriptor (PG213, Table 60 and Table 61);
    // beats 1..n are payload. tuser[3:0] = first_be, tuser[7:4] = last_be
    // (PG213, Table 14), read on beat 0.
    input  logic [AXIS_DATA_WIDTH-1:0]  s_axis_rq_tdata,
    input  logic [AXIS_KEEP_WIDTH-1:0]  s_axis_rq_tkeep,
    input  logic                        s_axis_rq_tvalid,
    input  logic                        s_axis_rq_tlast,
    input  logic [AXIS_USER_WIDTH-1:0]  s_axis_rq_tuser,
    output logic                        s_axis_rq_tready,

    // ---- core-managed tag presentation -------------------------------------
    // The tag tlp_request_tracker allocated, which the emitted header carries.
    // Correlate completions with this, not with the descriptor's Tag field
    // (which is ignored) and not with context (which is internally consumed).
    // One strobe per non-posted TLP, in issue order, a cycle or more after the
    // request is accepted; posted writes allocate no tag. tlp_requester takes
    // the tag in REQ_TAG, before the credit gate, so a strobe does not mean
    // the request was transmitted.
    output logic [7:0]                  pcie_rq_tag_o,
    output logic                        pcie_rq_tag_vld_o,

    // ---- PG213 Requester Completion AXI4-Stream master ---------------------
    // Beat 0 carries the 3-Dword RC descriptor (PG213, Table 65) in Dwords
    // 0..2 and the first payload Dword in Dword 3; later beats are payload.
    output logic [AXIS_DATA_WIDTH-1:0]  m_axis_rc_tdata,
    output logic [AXIS_KEEP_WIDTH-1:0]  m_axis_rc_tkeep,
    output logic                        m_axis_rc_tvalid,
    output logic                        m_axis_rc_tlast,
    input  logic                        m_axis_rc_tready,

    // ---- PG213 Completer Request AXI4-Stream master ------------------------
    // Inbound Memory requests inside the host aperture, presented to the host.
    // Beat 0 carries the 4-Dword CQ descriptor (PG213, Table 52); later beats
    // are payload. tuser[3:0] = first_be, tuser[7:4] = last_be, valid on beat
    // 0 (PG213, Table 10).
    output logic [AXIS_DATA_WIDTH-1:0]  m_axis_cq_tdata,
    output logic [AXIS_KEEP_WIDTH-1:0]  m_axis_cq_tkeep,
    output logic                        m_axis_cq_tvalid,
    output logic                        m_axis_cq_tlast,
    output logic [CQ_USER_WIDTH-1:0]    m_axis_cq_tuser,
    input  logic                        m_axis_cq_tready,

    // ---- PG213 Completer Completion AXI4-Stream slave ----------------------
    // The host's response to a completer request. Beat 0 carries the 3-Dword
    // CC descriptor (PG213, Table 58) in Dwords 0..2 and the first payload
    // Dword in Dword 3; later beats are payload.
    input  logic [AXIS_DATA_WIDTH-1:0]  s_axis_cc_tdata,
    input  logic [AXIS_KEEP_WIDTH-1:0]  s_axis_cc_tkeep,
    input  logic                        s_axis_cc_tvalid,
    input  logic                        s_axis_cc_tlast,
    input  logic [CC_USER_WIDTH-1:0]    s_axis_cc_tuser,
    output logic                        s_axis_cc_tready,

    // ---- Data Link Layer streams -------------------------------------------
    input  logic [TL_DATA_WIDTH-1:0]    s_dllp_axis_tdata,
    input  logic [TL_KEEP_WIDTH-1:0]    s_dllp_axis_tkeep,
    input  logic                        s_dllp_axis_tvalid,
    input  logic                        s_dllp_axis_tlast,
    input  logic [TL_USER_WIDTH-1:0]    s_dllp_axis_tuser,
    output logic                        s_dllp_axis_tready,

    output logic [TL_DATA_WIDTH-1:0]    m_dllp_axis_tdata,
    output logic [TL_KEEP_WIDTH-1:0]    m_dllp_axis_tkeep,
    output logic                        m_dllp_axis_tvalid,
    output logic                        m_dllp_axis_tlast,
    output logic [TL_USER_WIDTH-1:0]    m_dllp_axis_tuser,
    input  logic                        m_dllp_axis_tready,

    // ---- RQ error surface (pcie_rq_if) -------------------------------------
    // One-cycle pulse; the code is valid in the same cycle and holds until the
    // next rejection. A rejected descriptor emits no TLP.
    output logic                        rq_protocol_error_o,
    output rq_error_e                   rq_error_code_o,
    output logic                        rq_gearbox_error_o,

    // ---- CQ / CC error surface ---------------------------------------------
    // cq_dropped_o: a one-cycle pulse, with its reason on cq_error_code_o, for
    // an inbound request pcie_cq_if did not deliver to the host. A dropped
    // non-posted request is still answered with an Unsupported Request
    // Completion through pcie_cc_if.
    output logic                        cq_dropped_o,
    output logic [3:0]                  cq_error_code_o,
    output logic                        cq_gearbox_error_o,
    output logic                        cc_protocol_error_o,
    output logic [3:0]                  cc_error_code_o,
    output logic                        cc_gearbox_error_o,

    // ---- RC error surface (pcie_rc_if) -------------------------------------
    // rc_unexpected_completion_o: the completion's Tag and Requester ID
    // matched no outstanding tag, or its payload, Byte Count or Lower Address
    // did not fit the request. No RC packet accompanies it.
    output logic                        rc_unexpected_completion_o,
    output tlp_error_e                  rc_completion_error_code_o,
    output logic                        rc_protocol_error_o,
    output rc_error_e                   rc_error_code_o,
    output logic                        rc_gearbox_error_o,

    // ---- Transaction Layer error and status surface ------------------------
    output logic                        command_error_valid_o,
    output tlp_error_e                  command_error_code_o,
    output logic                        malformed_o,
    output logic                        rx_error_valid_o,
    output tlp_error_e                  rx_error_code_o,
    output logic                        rx_ecrc_error_o,
    output logic                        tx_error_valid_o,
    output tlp_error_e                  tx_error_code_o,
    // Asserted while a packet waits at the credit gate, either for FC
    // initialization or for credit. tlp_credit_manager raises credit_error_o
    // only for a packet larger than the pool's whole advertised capacity.
    output logic                        tx_fc_blocked_o,
    output logic                        credit_error_o,
    output logic                        vc_overflow_o,
    // ---- Completion Timeout surface (tlp_request_tracker) ------------------
    // One-cycle strobes with the tag valid in the same cycle, to match against
    // the earlier pcie_rq_tag_o. cpl_timeout_*: a non-posted request was not
    // answered in time; it has failed and its tag is quarantined. late_cpl_*:
    // a completion arrived for a quarantined tag and was drained, with no RC
    // packet.
    //
    // The timer starts when the tag is allocated and restarts when the request
    // is handed to the Data Link Layer, so a transmitted request gets the full
    // CPL_TIMEOUT_CYCLES from transmission (PCIe Base Spec r2.1, §2.8). A
    // request still waiting for credit CPL_TIMEOUT_CYCLES after allocation
    // times out untransmitted, which that section does not require. Its
    // packet stays queued and can still be transmitted later.
    output logic                        cpl_timeout_valid_o,
    output logic [7:0]                  cpl_timeout_tag_o,
    output logic                        late_cpl_valid_o,
    output logic [7:0]                  late_cpl_tag_o,

    // Non-posted requests currently holding a tag, including tags quarantined
    // by a completion timeout: a quarantined tag cannot be allocated until its
    // last late completion arrives or a further CPL_TIMEOUT_CYCLES passes.
    // Returns to 0 when every request has been answered or released.
    output logic [$clog2(TAG_COUNT+1)-1:0] outstanding_o
);

  // -------------------------------------------------------------------------
  // Host aperture
  // -------------------------------------------------------------------------
  // tlp_bar_decoder takes the window as a mask; the CQ descriptor's BAR
  // Aperture field takes its size in address bits (PG213, Table 52). Both are
  // derived from HOST_MEM_SIZE, so the descriptor always describes the window
  // the decoder matches.
  localparam logic [63:0] HOST_MEM_MASK = ~(HOST_MEM_SIZE - 64'd1);
  localparam logic [5:0]  HOST_MEM_APERTURE = 6'($clog2(HOST_MEM_SIZE));

  // -------------------------------------------------------------------------
  // Command port: pcie_rq_if to tlp_layer
  // -------------------------------------------------------------------------
  // command_context carries {mem_read, address[11:0]} from pcie_rq_if,
  // through tlp_request_tracker, to pcie_rc_if, which rebuilds Lower Address
  // [11:7] from it: a completion header carries only bits [6:0]. It is not a
  // client channel, and bits [15:13] are unused.
  logic                     command_valid;
  logic                     command_ready;
  tlp_cmd_e                 command;
  logic [63:0]              command_address;
  logic [12:0]              command_byte_count;
  logic [2:0]               command_tc;
  logic [2:0]               command_attr;
  logic [CONTEXT_WIDTH-1:0] command_context;
  logic                     command_prefix_valid;
  logic [31:0]              command_prefix;
  logic                     command_ecrc_enable;
  logic [TL_DATA_WIDTH-1:0] command_data;
  logic [TL_KEEP_WIDTH-1:0] command_keep;
  logic                     command_data_valid;
  logic                     command_data_last;
  logic                     command_data_ready;

  // The allocated tag, from tlp_request_tracker through tlp_layer.
  logic [7:0]               allocated_tag;
  logic                     allocated_tag_valid;

  // -------------------------------------------------------------------------
  // Received completions: tlp_layer to pcie_rc_if
  // -------------------------------------------------------------------------
  // The parsed completion header and payload, and tlp_request_tracker's
  // result for the same completion (result_*). received_completion_header is
  // a tlp_header_t and stays internal: the host sees the RC descriptor.
  logic                     received_completion_valid;
  logic                     received_completion_ready;
  tlp_header_t              received_completion_header;
  logic [TL_DATA_WIDTH-1:0] received_completion_data;
  logic [TL_KEEP_WIDTH-1:0] received_completion_keep;
  logic                     received_completion_data_valid;
  logic                     received_completion_data_last;
  logic                     received_completion_data_ready;

  logic                     result_valid;
  logic                     result_ready;
  logic [CONTEXT_WIDTH-1:0] result_context;
  logic [2:0]               result_status;
  logic                     result_last;
  logic                     unexpected_completion;
  tlp_error_e               completion_error_code;


  // Width of tlp_layer's target_bar_o for BAR_COUNT = 2. Only BAR 0 is
  // enabled (BAR_ENABLE 2'b01), so the decoded index is always 0 and
  // tlp_bar_decoder's overlap output cannot assert.
  localparam int TL_BAR_INDEX_WIDTH = 1;

  // -------------------------------------------------------------------------
  // Completer Completion: s_axis_cc_* to tlp_layer
  // -------------------------------------------------------------------------
  // pcie_cc_if decodes the host's CC descriptor and payload onto tlp_layer's
  // completion_request_* port, which feeds tlp_completion_generator. It also
  // takes pcie_cq_if's auto-UR requests (ur_*) and sends each as an
  // Unsupported Request Completion.
  logic                     completion_request_valid;
  logic                     completion_request_ready;
  tlp_header_t              completion_request_header;
  logic [2:0]               completion_request_status;
  logic [12:0]              completion_request_byte_count;
  logic [6:0]               completion_request_lower_address;
  logic                     completion_request_ecrc_enable;
  logic [TL_DATA_WIDTH-1:0] completion_request_data;
  logic [TL_KEEP_WIDTH-1:0] completion_request_keep;
  logic                     completion_request_data_valid;
  logic                     completion_request_data_last;
  logic                     completion_request_data_ready;

  cc_error_e                cc_error_code;
  assign cc_error_code_o = 4'(cc_error_code);

  pcie_cc_if #(
      .AXIS_DATA_WIDTH(AXIS_DATA_WIDTH),
      .AXIS_KEEP_WIDTH(AXIS_KEEP_WIDTH),
      .CC_USER_WIDTH  (CC_USER_WIDTH),
      .TL_DATA_WIDTH  (TL_DATA_WIDTH),
      .TL_KEEP_WIDTH  (TL_KEEP_WIDTH)
  ) u_cc_if (
      .clk_i(clk_i),
      .rst_i(rst_i),

      .s_axis_cc_tdata (s_axis_cc_tdata),
      .s_axis_cc_tkeep (s_axis_cc_tkeep),
      .s_axis_cc_tvalid(s_axis_cc_tvalid),
      .s_axis_cc_tlast (s_axis_cc_tlast),
      .s_axis_cc_tuser (s_axis_cc_tuser),
      .s_axis_cc_tready(s_axis_cc_tready),

      .completion_request_valid_o        (completion_request_valid),
      .completion_request_ready_i        (completion_request_ready),
      .completion_request_header_o       (completion_request_header),
      .completion_request_status_o       (completion_request_status),
      .completion_request_byte_count_o   (completion_request_byte_count),
      .completion_request_lower_address_o(completion_request_lower_address),
      .completion_request_ecrc_enable_o  (completion_request_ecrc_enable),

      .completion_request_data_o      (completion_request_data),
      .completion_request_keep_o      (completion_request_keep),
      .completion_request_data_valid_o(completion_request_data_valid),
      .completion_request_data_last_o (completion_request_data_last),
      .completion_request_data_ready_i(completion_request_data_ready),

      .ur_valid_i     (ur_valid),
      .ur_ready_o     (ur_ready),
      .ur_header_i    (ur_header),
      .ur_byte_count_i(ur_byte_count),

      .cc_protocol_error_o(cc_protocol_error_o),
      .cc_error_code_o    (cc_error_code),
      .cc_gearbox_error_o (cc_gearbox_error_o)
  );

  // -------------------------------------------------------------------------
  // Completer Request: tlp_layer to m_axis_cq_*
  // -------------------------------------------------------------------------
  // pcie_cq_if takes tlp_layer's target_* request port and either emits a CQ
  // packet, for a Memory request inside the host aperture, or raises
  // cq_dropped_o with a reason. It holds target_request_ready and
  // target_data_ready low while busy, so a request waits rather than being
  // lost.
  logic                     target_request_valid;
  logic                     target_request_ready;
  tlp_header_t              target_request_header;
  logic                     target_memory;
  logic                     target_config;
  logic                     target_config_type_one;
  logic                     target_read;
  logic                     target_write;
  logic                     target_unsupported;
  logic                     target_bar_hit;
  logic                     target_bar_overlap;
  logic [TL_BAR_INDEX_WIDTH-1:0] target_bar;
  logic [TL_DATA_WIDTH-1:0] target_data;
  logic [TL_KEEP_WIDTH-1:0] target_keep;
  logic                     target_data_valid;
  logic                     target_data_last;
  logic                     target_data_ready;

  cq_error_e                cq_error_code;
  assign cq_error_code_o = 4'(cq_error_code);

  // The auto-UR sideband: pcie_cq_if knows which non-posted request it
  // dropped, and pcie_cc_if, which owns the completion_request_* port, sends
  // its Unsupported Request Completion (PCIe Base Spec r2.1, §2.3.1).
  logic        ur_valid;
  logic        ur_ready;
  tlp_header_t ur_header;
  logic [12:0] ur_byte_count;

  pcie_cq_if #(
      .AXIS_DATA_WIDTH(AXIS_DATA_WIDTH),
      .AXIS_KEEP_WIDTH(AXIS_KEEP_WIDTH),
      .CQ_USER_WIDTH  (CQ_USER_WIDTH),
      .TL_DATA_WIDTH  (TL_DATA_WIDTH),
      .TL_KEEP_WIDTH  (TL_KEEP_WIDTH),
      .BAR_INDEX_WIDTH(TL_BAR_INDEX_WIDTH),
      .CQ_BAR_APERTURE(HOST_MEM_APERTURE)
  ) u_cq_if (
      .clk_i(clk_i),
      .rst_i(rst_i),

      .target_request_valid_i  (target_request_valid),
      .target_request_ready_o  (target_request_ready),
      .target_request_header_i (target_request_header),
      .target_memory_i         (target_memory),
      .target_config_i         (target_config),
      .target_config_type_one_i(target_config_type_one),
      .target_read_i           (target_read),
      .target_write_i          (target_write),
      .target_unsupported_i    (target_unsupported),
      .target_bar_hit_i        (target_bar_hit),
      .target_bar_overlap_i    (target_bar_overlap),
      .target_bar_i            (target_bar),

      .target_data_i      (target_data),
      .target_keep_i      (target_keep),
      .target_data_valid_i(target_data_valid),
      .target_data_last_i (target_data_last),
      .target_data_ready_o(target_data_ready),

      .m_axis_cq_tdata (m_axis_cq_tdata),
      .m_axis_cq_tkeep (m_axis_cq_tkeep),
      .m_axis_cq_tvalid(m_axis_cq_tvalid),
      .m_axis_cq_tlast (m_axis_cq_tlast),
      .m_axis_cq_tuser (m_axis_cq_tuser),
      .m_axis_cq_tready(m_axis_cq_tready),

      .ur_valid_o     (ur_valid),
      .ur_ready_i     (ur_ready),
      .ur_header_o    (ur_header),
      .ur_byte_count_o(ur_byte_count),

      .cq_dropped_o      (cq_dropped_o),
      .cq_error_code_o   (cq_error_code),
      .cq_gearbox_error_o(cq_gearbox_error_o)
  );

  // -------------------------------------------------------------------------
  // Requester Request: s_axis_rq_* to tlp_layer
  // -------------------------------------------------------------------------
  // pcie_rq_if checks each RQ descriptor, drives tlp_layer's command port and
  // narrows the payload to TL_DATA_WIDTH. A rejected descriptor pulses
  // rq_protocol_error_o and emits no TLP. It also presents the allocated tag
  // on pcie_rq_tag_o.
  pcie_rq_if #(
      .AXIS_DATA_WIDTH(AXIS_DATA_WIDTH),
      .AXIS_KEEP_WIDTH(AXIS_KEEP_WIDTH),
      .AXIS_USER_WIDTH(AXIS_USER_WIDTH),
      .TL_DATA_WIDTH  (TL_DATA_WIDTH),
      .TL_KEEP_WIDTH  (TL_KEEP_WIDTH),
      .CONTEXT_WIDTH  (CONTEXT_WIDTH)
  ) u_rq_if (
      .clk_i(clk_i),
      .rst_i(rst_i),

      .s_axis_rq_tdata (s_axis_rq_tdata),
      .s_axis_rq_tkeep (s_axis_rq_tkeep),
      .s_axis_rq_tvalid(s_axis_rq_tvalid),
      .s_axis_rq_tlast (s_axis_rq_tlast),
      .s_axis_rq_tuser (s_axis_rq_tuser),
      .s_axis_rq_tready(s_axis_rq_tready),

      .allocated_tag_i      (allocated_tag),
      .allocated_tag_valid_i(allocated_tag_valid),
      .pcie_rq_tag_o        (pcie_rq_tag_o),
      .pcie_rq_tag_vld_o    (pcie_rq_tag_vld_o),

      .command_valid_o       (command_valid),
      .command_ready_i       (command_ready),
      .command_o             (command),
      .command_address_o     (command_address),
      .command_byte_count_o  (command_byte_count),
      .command_tc_o          (command_tc),
      .command_attr_o        (command_attr),
      .command_context_o     (command_context),
      .command_prefix_valid_o(command_prefix_valid),
      .command_prefix_o      (command_prefix),
      .command_ecrc_enable_o (command_ecrc_enable),

      .command_data_o      (command_data),
      .command_keep_o      (command_keep),
      .command_data_valid_o(command_data_valid),
      .command_data_last_o (command_data_last),
      .command_data_ready_i(command_data_ready),

      .rq_protocol_error_o(rq_protocol_error_o),
      .rq_error_code_o    (rq_error_code_o),
      .rq_gearbox_error_o (rq_gearbox_error_o)
  );

  // -------------------------------------------------------------------------
  // Transaction Layer
  // -------------------------------------------------------------------------
  // tlp_layer with the host aperture as its only enabled BAR. Its
  // completion_request_* port is driven by u_cc_if, its target_* port feeds
  // u_cq_if, and its received-completion and result ports feed u_rc_if.
  tlp_layer #(
      .DATA_WIDTH   (TL_DATA_WIDTH),
      .KEEP_WIDTH   (TL_KEEP_WIDTH),
      .USER_WIDTH   (TL_USER_WIDTH),
      .TAG_COUNT    (TAG_COUNT),
      .CONTEXT_WIDTH(CONTEXT_WIDTH),
      .CPL_TIMEOUT_CYCLES(CPL_TIMEOUT_CYCLES),
      .PCIE_WIRE_ORDER(PCIE_WIRE_ORDER),
      // ---- the host aperture as BAR 0 ------------------------------------
      // BAR_COUNT is tlp_layer's default of 2 with only index 0 enabled, so
      // one window is the whole map. BAR_BASE and BAR_MASK are
      // [BAR_COUNT*64-1:0] with index 0 in the low 64 bits.
      .BAR_COUNT (2),
      .BAR_BASE  ({64'd0, HOST_MEM_BASE}),
      .BAR_MASK  ({64'd0, HOST_MEM_MASK}),
      .BAR_ENABLE(2'b01)
  ) u_tlp_layer (
      .clk_i(clk_i),
      .rst_i(rst_i),

      .link_up_i        (link_up_i),
      .transmit_enable_i(transmit_enable_i),
      .requester_id_i   (requester_id_i),
      .completer_id_i   (completer_id_i),
      .bus_number_i     (bus_number_i),
      .device_number_i  (device_number_i),
      .function_number_i(function_number_i),
      .memory_enable_i  (memory_enable_i),
      .extended_tag_enable_i(extended_tag_enable_i),
      .max_payload_bytes_i  (max_payload_bytes_i),
      .max_read_bytes_i     (max_read_bytes_i),
      .rcb_128b_i           (rcb_128b_i),
      .fc_initialized_i (fc_initialized_i),
      .fc_update_valid_i(fc_update_valid_i),
      .fc_ph_i  (fc_ph_i),   .fc_pd_i  (fc_pd_i),
      .fc_nph_i (fc_nph_i),  .fc_npd_i (fc_npd_i),
      .fc_cplh_i(fc_cplh_i), .fc_cpld_i(fc_cpld_i),

      .s_dllp_axis_tdata (s_dllp_axis_tdata),
      .s_dllp_axis_tkeep (s_dllp_axis_tkeep),
      .s_dllp_axis_tvalid(s_dllp_axis_tvalid),
      .s_dllp_axis_tlast (s_dllp_axis_tlast),
      .s_dllp_axis_tuser (s_dllp_axis_tuser),
      .s_dllp_axis_tready(s_dllp_axis_tready),

      .m_dllp_axis_tdata (m_dllp_axis_tdata),
      .m_dllp_axis_tkeep (m_dllp_axis_tkeep),
      .m_dllp_axis_tvalid(m_dllp_axis_tvalid),
      .m_dllp_axis_tlast (m_dllp_axis_tlast),
      .m_dllp_axis_tuser (m_dllp_axis_tuser),
      .m_dllp_axis_tready(m_dllp_axis_tready),

      .command_valid_i       (command_valid),
      .command_ready_o       (command_ready),
      .command_i             (command),
      .command_address_i     (command_address),
      .command_byte_count_i  (command_byte_count),
      .command_tc_i          (command_tc),
      .command_attr_i        (command_attr),
      .command_context_i     (command_context),
      .command_prefix_valid_i(command_prefix_valid),
      .command_prefix_i      (command_prefix),
      .command_ecrc_enable_i (command_ecrc_enable),
      .command_data_i        (command_data),
      .command_keep_i        (command_keep),
      .command_data_valid_i  (command_data_valid),
      .command_data_last_i   (command_data_last),
      .command_data_ready_o  (command_data_ready),
      .command_error_valid_o (command_error_valid_o),
      .command_error_code_o  (command_error_code_o),
      .allocated_tag_o       (allocated_tag),
      .allocated_tag_valid_o (allocated_tag_valid),

      // ---- Completer Request: target_* to u_cq_if --------------------------
      // u_cq_if drives both ready inputs. Four outputs are left open:
      //   target_request_class_o  the CQ descriptor carries the Request Type
      //                           (PG213, Table 57), which u_cq_if builds from
      //                           the memory, config, read and write decodes;
      //                           tlp_layer's class (posted, non-posted,
      //                           completion) is not a descriptor field.
      //   target_config_hit_o,    u_cq_if delivers Memory requests only and
      //   target_config_offset_o  drops every Configuration request.
      //   target_offset_o         the CQ descriptor carries the full address
      //                           and a BAR Aperture, not an offset (PG213,
      //                           Table 52).
      .target_request_valid_o  (target_request_valid),
      .target_request_ready_i  (target_request_ready),
      .target_request_header_o (target_request_header),
      .target_request_class_o  (),
      .target_memory_o         (target_memory),
      .target_config_o         (target_config),
      .target_config_hit_o     (),
      .target_config_type_one_o(target_config_type_one),
      .target_config_offset_o  (),
      .target_read_o           (target_read),
      .target_write_o          (target_write),
      .target_unsupported_o    (target_unsupported),
      .target_bar_hit_o        (target_bar_hit),
      .target_bar_overlap_o    (target_bar_overlap),
      .target_bar_o            (target_bar),
      .target_offset_o         (),
      .target_data_o           (target_data),
      .target_keep_o           (target_keep),
      .target_data_valid_o     (target_data_valid),
      .target_data_last_o      (target_data_last),
      .target_data_ready_i     (target_data_ready),

      // ---- Completer Completion: u_cc_if to tlp_completion_generator -------
      .completion_request_valid_i        (completion_request_valid),
      .completion_request_ready_o        (completion_request_ready),
      .completion_request_header_i       (completion_request_header),
      .completion_request_status_i       (completion_request_status),
      .completion_request_byte_count_i   (completion_request_byte_count),
      .completion_request_lower_address_i(completion_request_lower_address),
      .completion_request_ecrc_enable_i  (completion_request_ecrc_enable),
      .completion_request_data_i         (completion_request_data),
      .completion_request_keep_i         (completion_request_keep),
      .completion_request_data_valid_i   (completion_request_data_valid),
      .completion_request_data_last_i    (completion_request_data_last),
      .completion_request_data_ready_o   (completion_request_data_ready),

      .received_completion_valid_o     (received_completion_valid),
      .received_completion_ready_i     (received_completion_ready),
      .received_completion_header_o    (received_completion_header),
      .received_completion_data_o      (received_completion_data),
      .received_completion_keep_o      (received_completion_keep),
      .received_completion_data_valid_o(received_completion_data_valid),
      .received_completion_data_last_o (received_completion_data_last),
      .received_completion_data_ready_i(received_completion_data_ready),

      .result_valid_o  (result_valid),
      .result_ready_i  (result_ready),
      .result_context_o(result_context),
      .result_status_o (result_status),
      .result_last_o   (result_last),

      .malformed_o            (malformed_o),
      .rx_error_valid_o       (rx_error_valid_o),
      .rx_error_code_o        (rx_error_code_o),
      .rx_ecrc_error_o        (rx_ecrc_error_o),
      .tx_error_valid_o       (tx_error_valid_o),
      .tx_error_code_o        (tx_error_code_o),
      .tx_fc_blocked_o        (tx_fc_blocked_o),
      .credit_error_o         (credit_error_o),
      .vc_overflow_o          (vc_overflow_o),
      .unexpected_completion_o(unexpected_completion),
      .completion_error_code_o(completion_error_code),
      .cpl_timeout_valid_o    (cpl_timeout_valid_o),
      .cpl_timeout_tag_o      (cpl_timeout_tag_o),
      .late_cpl_valid_o       (late_cpl_valid_o),
      .late_cpl_tag_o         (late_cpl_tag_o),
      .outstanding_o          (outstanding_o)
  );

  // -------------------------------------------------------------------------
  // Requester Completion: tlp_layer to m_axis_rc_*
  // -------------------------------------------------------------------------
  // pcie_rc_if pairs each received completion header with
  // tlp_request_tracker's result for it, builds the 3-Dword RC descriptor
  // (PG213, Table 65) and streams it ahead of the payload. A completion whose
  // Tag and Requester ID match no outstanding tag, or whose payload, Byte
  // Count or Lower Address does not fit the request, produces no RC packet
  // and pulses rc_unexpected_completion_o instead.
  pcie_rc_if #(
      .AXIS_DATA_WIDTH(AXIS_DATA_WIDTH),
      .AXIS_KEEP_WIDTH(AXIS_KEEP_WIDTH),
      .TL_DATA_WIDTH  (TL_DATA_WIDTH),
      .TL_KEEP_WIDTH  (TL_KEEP_WIDTH),
      .CONTEXT_WIDTH  (CONTEXT_WIDTH)
  ) u_rc_if (
      .clk_i(clk_i),
      .rst_i(rst_i),

      .received_completion_valid_i (received_completion_valid),
      .received_completion_ready_o (received_completion_ready),
      .received_completion_header_i(received_completion_header),

      .received_completion_data_i      (received_completion_data),
      .received_completion_keep_i      (received_completion_keep),
      .received_completion_data_valid_i(received_completion_data_valid),
      .received_completion_data_last_i (received_completion_data_last),
      .received_completion_data_ready_o(received_completion_data_ready),

      .result_valid_i         (result_valid),
      .result_ready_o         (result_ready),
      .result_context_i       (result_context),
      .result_status_i        (result_status),
      .result_last_i          (result_last),
      .unexpected_completion_i(unexpected_completion),
      .completion_error_code_i(completion_error_code),

      .m_axis_rc_tdata (m_axis_rc_tdata),
      .m_axis_rc_tkeep (m_axis_rc_tkeep),
      .m_axis_rc_tvalid(m_axis_rc_tvalid),
      .m_axis_rc_tlast (m_axis_rc_tlast),
      .m_axis_rc_tready(m_axis_rc_tready),

      .rc_unexpected_completion_o(rc_unexpected_completion_o),
      .rc_completion_error_code_o(rc_completion_error_code_o),
      .rc_protocol_error_o       (rc_protocol_error_o),
      .rc_error_code_o           (rc_error_code_o),
      .rc_gearbox_error_o        (rc_gearbox_error_o)
  );

endmodule
