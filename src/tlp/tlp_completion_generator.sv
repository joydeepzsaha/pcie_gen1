// ---------------------------------------------------------------------------
// tlp_completion_generator -- forms the Completions for one received request
//
// Purpose
//   Turns one completion request from the client (the original request's
//   header, a status, a byte count and a Lower Address) into one or more
//   Completion headers, and passes the payload beats through to tlp_control.
//   A Successful Completion with data is split so that no CplD crosses a
//   Read Completion Boundary (RCB) or carries more than max_payload_bytes_i.
//
// Interfaces
//   Config   completer_id_i: the Completer ID. max_payload_bytes_i: 0 means
//            128. rcb_128b_i: the RCB is 128 bytes, else 64.
//   Request  request_valid_i, request_ready_o, request_header_i,
//            request_status_i, request_byte_count_i, request_lower_address_i,
//            request_ecrc_enable_i: one handshake per request, in CPL_IDLE.
//   Data     request_data_*, request_keep_i: the payload of the whole
//            request, tlast on its last beat.
//   Packet   packet_header_*, packet_data_*, packet_keep_o: one header per
//            Completion and its payload beats, to tlp_control.
//   Error    error_valid_o, error_code_o: one cycle of TLP_ERR_LOCAL_PAYLOAD
//            when tlast and the last beat of the last CplD disagree.
//
// Clock and reset
//   clk_i only. rst_i is synchronous and active high.
//
// Limitations
//   A Successful Completion without data, as an I/O or Configuration Write
//   needs, is formed only with request_byte_count_i = 0, and its Byte Count
//   field then carries 0, which encodes 4096; PCIe Base Spec r2.1, §2.2.9
//   requires 4. A CplD with lower_address[1:0] = k > 0 ends after k bytes
//   more than its payload, and tlp_generator starts the payload at lane k.
//   From whole DWs with the first payload byte in lane k, as pcie_cc_if
//   passes them on, the CplD carries one DW more than its Length, each byte
//   k lanes late.
//
// References
//   PCIe Base Spec r2.1, §2.2.9
//   PCIe Base Spec r2.1, §2.3.1.1
// ---------------------------------------------------------------------------
`timescale 1ns/1ps
module tlp_completion_generator
  import tlp_pkg::*;
#(
    parameter int DATA_WIDTH = 32,
    parameter int KEEP_WIDTH = DATA_WIDTH / 8
) (
    input  logic                  clk_i,
    input  logic                  rst_i,
    input  logic [15:0]           completer_id_i,
    input  logic [12:0]           max_payload_bytes_i,
    input  logic                  rcb_128b_i,

    input  logic                  request_valid_i,
    output logic                  request_ready_o,
    input  tlp_header_t           request_header_i,
    input  logic [2:0]            request_status_i,
    input  logic [12:0]           request_byte_count_i,
    input  logic [6:0]            request_lower_address_i,
    input  logic                  request_ecrc_enable_i,

    input  logic [DATA_WIDTH-1:0] request_data_i,
    input  logic [KEEP_WIDTH-1:0] request_keep_i,
    input  logic                  request_data_valid_i,
    input  logic                  request_data_last_i,
    output logic                  request_data_ready_o,

    output tlp_header_t           packet_header_o,
    output logic                  packet_header_valid_o,
    input  logic                  packet_header_ready_i,
    output logic [DATA_WIDTH-1:0] packet_data_o,
    output logic [KEEP_WIDTH-1:0] packet_keep_o,
    output logic                  packet_data_valid_o,
    output logic                  packet_data_last_o,
    input  logic                  packet_data_ready_i,
    output logic                  error_valid_o,
    output tlp_error_e            error_code_o
);

  // CPL_IDLE    takes a completion request and forms the first header.
  // CPL_HEADER  offers the header; goes to CPL_DATA when it has a Length,
  //             else back to CPL_IDLE.
  // CPL_DATA    passes payload beats; at the end of a CplD forms the next
  //             header and returns to CPL_HEADER, or ends in CPL_IDLE.
  typedef enum logic [1:0] {CPL_IDLE, CPL_HEADER, CPL_DATA} cpl_state_e;
  cpl_state_e state_r;
  tlp_header_t header_r;
  logic [12:0] sent_bytes_r;
  logic [12:0] remaining_bytes_r;
  logic [12:0] segment_bytes_r;
  logic [6:0] lower_address_r;
  logic [12:0] accepted_bytes;
  logic [12:0] segment_wire_bytes;
  logic expected_last;
  integer lane;

  // The bytes the next CplD carries: what remains, cut at
  // max_payload_bytes_i and at the next RCB boundary above lower_address.
  // With max_payload_bytes_i of 128 or more the RCB cut binds, so every CplD
  // but the last ends on an RCB boundary (PCIe Base Spec r2.1, §2.3.1.1).
  function automatic logic [12:0] completion_segment(
      input logic [12:0] remaining,
      input logic [6:0] lower_address
  );
    logic [12:0] limit;
    logic [12:0] boundary;
    limit = max_payload_bytes_i == 0 ? 13'd128 : max_payload_bytes_i;
    boundary = rcb_128b_i ? 13'd128 - {6'd0,lower_address[6:0]} :
                            13'd64 - {7'd0,lower_address[5:0]};
    completion_segment = remaining;
    if (completion_segment > limit) completion_segment = limit;
    if (completion_segment > boundary) completion_segment = boundary;
  endfunction

  always_comb begin
    accepted_bytes = '0;
    for (lane = 0; lane < KEEP_WIDTH; lane = lane + 1)
      accepted_bytes = accepted_bytes + request_keep_i[lane];
    // Length covers segment_bytes_r + lower_address_r[1:0] bytes, counted from
    // the DW that holds lower_address, and expected_last waits for that many
    // kept bytes, so the bytes ahead of the first payload byte count as input.
    // tlp_generator offsets the payload by those bytes again (see Limitations).
    // segment_bytes_r stays a payload byte count for the split.
    segment_wire_bytes = segment_bytes_r + {11'd0, lower_address_r[1:0]};
    expected_last = sent_bytes_r + accepted_bytes >= segment_wire_bytes;
  end

  assign request_ready_o = state_r == CPL_IDLE;
  assign packet_header_o = header_r;
  assign packet_header_valid_o = state_r == CPL_HEADER;
  assign packet_data_o = request_data_i;
  assign packet_keep_o = request_keep_i;
  assign packet_data_valid_o = state_r == CPL_DATA && request_data_valid_i;
  assign packet_data_last_o = expected_last || request_data_last_i;
  assign request_data_ready_o = state_r == CPL_DATA && packet_data_ready_i;

  always_ff @(posedge clk_i) begin
    if (rst_i) begin
      state_r  <= CPL_IDLE;
      header_r <= '0;
      sent_bytes_r <= '0;
      remaining_bytes_r <= '0;
      segment_bytes_r <= '0;
      lower_address_r <= '0;
      error_valid_o <= 1'b0;
      error_code_o <= TLP_ERR_NONE;
    end else begin
      error_valid_o <= 1'b0;
      error_code_o <= TLP_ERR_NONE;
      unique case (state_r)
        CPL_IDLE: if (request_valid_i && request_ready_o) begin
          // Clears BCM, which a PCI Express Completer never sets (PCIe Base
          // Spec r2.1, §2.2.9).
          header_r <= '0;
          // A status other than SC carries no data and ends the Completions
          // for the request (PCIe Base Spec r2.1, §2.3.1.1).
          header_r.fmt <= request_byte_count_i == 0 || request_status_i != TLP_CPL_SC ?
                          TLP_FMT_3DW_NO_DATA : TLP_FMT_3DW_DATA;
          header_r.tlp_type <= TLP_TYPE_CPL;
          // Requester ID, Tag, TC and Attr are copied from the request
          // (PCIe Base Spec r2.1, §2.2.9).
          header_r.traffic_class <= request_header_i.traffic_class;
          header_r.attributes <= request_header_i.attributes;
          header_r.length_dw <= request_byte_count_i == 0 || request_status_i != TLP_CPL_SC ?
              11'd0 : 11'((completion_segment(request_byte_count_i,
              request_lower_address_i) +
              {11'd0, request_lower_address_i[1:0]} + 13'd3) >> 2);
          header_r.requester_id <= request_header_i.requester_id;
          header_r.completer_id <= completer_id_i;
          header_r.tag <= request_header_i.tag;
          header_r.completion_status <= request_status_i;
          header_r.byte_count <= request_byte_count_i;
          header_r.lower_address <= request_lower_address_i;
          header_r.digest_present <= request_ecrc_enable_i;
          sent_bytes_r <= '0;
          remaining_bytes_r <= request_byte_count_i;
          segment_bytes_r <= request_status_i == TLP_CPL_SC ?
              completion_segment(request_byte_count_i, request_lower_address_i) : 0;
          lower_address_r <= request_lower_address_i;
          state_r <= CPL_HEADER;
        end

        CPL_HEADER: if (packet_header_ready_i) begin
          if (header_r.length_dw == 0)
            state_r <= CPL_IDLE;
          else
            state_r <= CPL_DATA;
        end

        CPL_DATA: if (request_data_valid_i && request_data_ready_o) begin
          sent_bytes_r <= sent_bytes_r + accepted_bytes;
          // tlast is due on the last beat of the last CplD.
          if (request_data_last_i != (expected_last && remaining_bytes_r <= segment_bytes_r)) begin
            error_valid_o <= 1'b1;
            error_code_o <= TLP_ERR_LOCAL_PAYLOAD;
          end
          // An early tlast ends the request. packet_data_last_o closes the
          // CplD at that beat, short of the byte count its Length came from.
          if (request_data_last_i && !expected_last) begin
            state_r <= CPL_IDLE;
          end else if (expected_last) begin
            // The next CplD: Byte Count is what remains (PCIe Base Spec
            // r2.1, §2.3.1.1), and Lower Address moves past this one's data.
            if (remaining_bytes_r > segment_bytes_r) begin
              remaining_bytes_r <= remaining_bytes_r - segment_bytes_r;
              lower_address_r <= lower_address_r + segment_bytes_r[6:0];
              header_r.byte_count <= remaining_bytes_r - segment_bytes_r;
              header_r.lower_address <= lower_address_r + segment_bytes_r[6:0];
              segment_bytes_r <= completion_segment(remaining_bytes_r - segment_bytes_r,
                  lower_address_r + segment_bytes_r[6:0]);
              header_r.length_dw <= 11'((completion_segment(
                  remaining_bytes_r - segment_bytes_r,
                  lower_address_r + segment_bytes_r[6:0]) +
                  {11'd0,lower_address_r[1:0] + segment_bytes_r[1:0]} + 13'd3) >> 2);
              sent_bytes_r <= '0;
              state_r <= CPL_HEADER;
            end else state_r <= CPL_IDLE;
          end
        end

        default: state_r <= CPL_IDLE;
      endcase
    end
  end

endmodule
