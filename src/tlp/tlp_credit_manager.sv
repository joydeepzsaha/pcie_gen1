// ---------------------------------------------------------------------------
// tlp_credit_manager -- VC0 transmit Flow Control credit gate
//
// Original author: Joydeep Saha
// Modified by: Kourosh Ghahramani
// Silicon Systems Research Lab, University of Washington
//
// Purpose
//   Decides whether the TLP at the head of tlp_vc_buffer may be sent, from
//   the credits the Receiver has advertised. For each of the six credit types
//   it keeps CREDIT_LIMIT and CREDITS_CONSUMED as PCIe Base Spec r2.1,
//   §2.6.1.1 defines them, in 8-bit header and 12-bit data registers, the
//   [Field Size] of each type, so that their wraparound is the modulo
//   arithmetic the spec requires. A type advertised as 0 at FC
//   initialization is infinite and never blocks.
//
// Interfaces
//   Credits  fc_initialized_i: no request passes while it is low.
//            fc_update_valid_i, fc_ph_i to fc_cpld_i: the HdrFC and DataFC
//            values last received for each type in an InitFC or UpdateFC,
//            the Receiver's CREDITS_ALLOCATED (§2.6.1.2). The first strobe
//            after reset is the FC initialization, every later one an update.
//   Request  request_valid_i, request_ready_o, request_class_i,
//            request_data_credits_i: one TLP's pool and data credits. A grant
//            consumes one header credit and the data credits.
//   Status   blocked_o: request_valid_i without request_ready_o. error_o: a
//            request that no credit return can admit; tlp_layer reports it
//            as TLP_ERR_CREDIT_UNDERFLOW. *_available_o: CREDIT_LIMIT minus
//            CREDITS_CONSUMED for each type; tlp_layer leaves them open.
//
// Clock and reset
//   clk_i only. rst_i is synchronous and active high; tlp_layer also asserts
//   it while the link is down, so the next FC initialization starts afresh.
//
// Limitations
//   error_o can pulse falsely for a TLP with data waiting in the cycle of the
//   first fc_update_valid_i: *_capacity_r is still 0 then, and
//   pcie_datalink_layer can raise fc_initialized_o in that same cycle.
//
// References
//   PCIe Base Spec r2.1, §2.6.1
//   PCIe Base Spec r2.1, §2.6.1.1
//   PCIe Base Spec r2.1, §2.6.1.2
// ---------------------------------------------------------------------------
`timescale 1ns/1ps
module tlp_credit_manager
  import tlp_pkg::*;
(
    input  logic              clk_i,
    input  logic              rst_i,
    input  logic              fc_initialized_i,
    input  logic              fc_update_valid_i,
    input  logic [7:0]        fc_ph_i,
    input  logic [11:0]       fc_pd_i,
    input  logic [7:0]        fc_nph_i,
    input  logic [11:0]       fc_npd_i,
    input  logic [7:0]        fc_cplh_i,
    input  logic [11:0]       fc_cpld_i,

    input  logic              request_valid_i,
    output logic              request_ready_o,
    input  tlp_credit_class_e request_class_i,
    input  logic [11:0]       request_data_credits_i,

    output logic              blocked_o,
    output logic              error_o,
    output logic [7:0]        posted_header_available_o,
    output logic [11:0]       posted_data_available_o,
    output logic [7:0]        nonposted_header_available_o,
    output logic [11:0]       nonposted_data_available_o,
    output logic [7:0]        completion_header_available_o,
    output logic [11:0]       completion_data_available_o
);

  // CREDIT_LIMIT: loaded from the advertisement, never counted down.
  logic [7:0]  ph_limit_r, nph_limit_r, cplh_limit_r;
  logic [11:0] pd_limit_r, npd_limit_r, cpld_limit_r;

  // CREDITS_CONSUMED: cleared at FC initialization, counted up per grant.
  logic [7:0]  ph_consumed_r, nph_consumed_r, cplh_consumed_r;
  logic [11:0] pd_consumed_r, npd_consumed_r, cpld_consumed_r;

  // Set by the first fc_update_valid_i after reset, taken as FC
  // initialization. pcie_datalink_layer strobes when the peer's three InitFC2
  // are stored and for each received UpdateFC. Whichever is first carries the
  // initial advertisement: no TLP passes this gate before it, and the peer's
  // CREDITS_ALLOCATED grows only as it processes TLPs (§2.6.1.2).
  logic fc_init_seen_r;

  // An advertisement of 00h or 000h at FC initialization means infinite
  // credit, and the gate never blocks that type (PCIe Base Spec r2.1,
  // §2.6.1, §2.6.1.1). The flag is latched then and never re-evaluated, so a
  // finite type consumed down to 0 does not read as infinite. Header and data
  // are flagged separately, because one of them may be infinite without the
  // other (§2.6.1). An Endpoint, and a Root Complex that does not support
  // peer-to-peer traffic between all Root Ports, must advertise infinite CPLH
  // and CPLD (§2.6.1, Table 2-37).
  logic ph_infinite_r, nph_infinite_r, cplh_infinite_r;
  logic pd_infinite_r, npd_infinite_r, cpld_infinite_r;

  // The data advertisement at FC initialization: the Receiver's initial
  // allocation, which follows its buffer size (PCIe Base Spec r2.1,
  // §2.6.1.2). *_limit_r goes on to count cumulative credit and wraps, so it
  // is not a capacity.
  logic [11:0] pd_capacity_r, npd_capacity_r, cpld_capacity_r;

  // Available credit: (CREDIT_LIMIT - CREDITS_CONSUMED) modulo the register
  // width, so a wrapped limit needs no special case. Comparing it with the
  // request gives the same answer as the modular test of PCIe Base Spec
  // r2.1, §2.6.1.1 while the Receiver keeps no more than 2047 data and 127
  // header credits outstanding (§2.6.1); a TLP needs at most 256 data
  // credits.
  logic [7:0]  ph_available, nph_available, cplh_available;
  logic [11:0] pd_available, npd_available, cpld_available;

  assign ph_available   = ph_limit_r   - ph_consumed_r;
  assign pd_available   = pd_limit_r   - pd_consumed_r;
  assign nph_available  = nph_limit_r  - nph_consumed_r;
  assign npd_available  = npd_limit_r  - npd_consumed_r;
  assign cplh_available = cplh_limit_r - cplh_consumed_r;
  assign cpld_available = cpld_limit_r - cpld_consumed_r;

  logic selected_header_available;
  logic selected_data_available;
  logic selected_data_infinite;
  logic [11:0] selected_data_capacity;
  logic request_unsatisfiable;

  always_comb begin
    selected_header_available = 1'b0;
    selected_data_available = 1'b0;
    selected_data_infinite = 1'b0;
    selected_data_capacity = '0;
    case (request_class_i)
      TLP_CREDIT_POSTED: begin
        selected_header_available = ph_infinite_r || (ph_available != 0);
        selected_data_available =
            pd_infinite_r || (pd_available >= request_data_credits_i);
        selected_data_infinite = pd_infinite_r;
        selected_data_capacity = pd_capacity_r;
      end
      TLP_CREDIT_COMPLETION: begin
        selected_header_available = cplh_infinite_r || (cplh_available != 0);
        selected_data_available =
            cpld_infinite_r || (cpld_available >= request_data_credits_i);
        selected_data_infinite = cpld_infinite_r;
        selected_data_capacity = cpld_capacity_r;
      end
      default: begin
        selected_header_available = nph_infinite_r || (nph_available != 0);
        selected_data_available =
            npd_infinite_r || (npd_available >= request_data_credits_i);
        selected_data_infinite = npd_infinite_r;
        selected_data_capacity = npd_capacity_r;
      end
    endcase

    // The request needs more data credits than the Receiver's whole initial
    // allocation for its type, so no credit return can admit it. Ordinary
    // blocking, where only the current remainder falls short, does not count,
    // and an infinite type never does. error_o is a diagnostic of this
    // module, not one of the Flow Control Protocol Errors of §2.6.1; a
    // Transmitter short of credit only has to block the TLP (§2.6.1.1).
    request_unsatisfiable = fc_initialized_i && !selected_data_infinite &&
                            (request_data_credits_i > selected_data_capacity);
    request_ready_o = fc_initialized_i && selected_header_available &&
                      selected_data_available;
    blocked_o = request_valid_i && !request_ready_o;
  end

  assign posted_header_available_o = ph_available;
  assign posted_data_available_o = pd_available;
  assign nonposted_header_available_o = nph_available;
  assign nonposted_data_available_o = npd_available;
  assign completion_header_available_o = cplh_available;
  assign completion_data_available_o = cpld_available;

  always_ff @(posedge clk_i) begin
    if (rst_i) begin
      ph_limit_r <= '0;
      pd_limit_r <= '0;
      nph_limit_r <= '0;
      npd_limit_r <= '0;
      cplh_limit_r <= '0;
      cpld_limit_r <= '0;
      ph_consumed_r <= '0;
      pd_consumed_r <= '0;
      nph_consumed_r <= '0;
      npd_consumed_r <= '0;
      cplh_consumed_r <= '0;
      cpld_consumed_r <= '0;
      fc_init_seen_r <= 1'b0;
      ph_infinite_r <= 1'b0;
      pd_infinite_r <= 1'b0;
      nph_infinite_r <= 1'b0;
      npd_infinite_r <= 1'b0;
      cplh_infinite_r <= 1'b0;
      cpld_infinite_r <= 1'b0;
      pd_capacity_r <= '0;
      npd_capacity_r <= '0;
      cpld_capacity_r <= '0;
      error_o <= 1'b0;
    end else begin
      error_o <= 1'b0;
      if (fc_update_valid_i) begin
        fc_init_seen_r <= 1'b1;
        if (!fc_init_seen_r) begin
          // FC initialization: CREDIT_LIMIT takes the advertised values
          // (PCIe Base Spec r2.1, §2.6.1.1).
          ph_limit_r <= fc_ph_i;
          pd_limit_r <= fc_pd_i;
          nph_limit_r <= fc_nph_i;
          npd_limit_r <= fc_npd_i;
          cplh_limit_r <= fc_cplh_i;
          cpld_limit_r <= fc_cpld_i;
          ph_infinite_r <= (fc_ph_i == '0);
          pd_infinite_r <= (fc_pd_i == '0);
          nph_infinite_r <= (fc_nph_i == '0);
          npd_infinite_r <= (fc_npd_i == '0);
          cplh_infinite_r <= (fc_cplh_i == '0);
          cpld_infinite_r <= (fc_cpld_i == '0);
          pd_capacity_r <= fc_pd_i;
          npd_capacity_r <= fc_npd_i;
          cpld_capacity_r <= fc_cpld_i;
        end else begin
          // UpdateFC. The field of a type advertised infinite is ignored
          // (PCIe Base Spec r2.1, §2.6.1), so that type's *_limit_r stays 0.
          // Its gate tests *_infinite_r first and does not depend on *_limit_r.
          if (!ph_infinite_r)   ph_limit_r   <= fc_ph_i;
          if (!pd_infinite_r)   pd_limit_r   <= fc_pd_i;
          if (!nph_infinite_r)  nph_limit_r  <= fc_nph_i;
          if (!npd_infinite_r)  npd_limit_r  <= fc_npd_i;
          if (!cplh_infinite_r) cplh_limit_r <= fc_cplh_i;
          if (!cpld_infinite_r) cpld_limit_r <= fc_cpld_i;
        end
      end
      // CREDITS_CONSUMED is cleared at FC initialization and grows by each
      // granted TLP's credits (PCIe Base Spec r2.1, §2.6.1.1). The clear
      // cannot hide a grant: until the first strobe every limit is 0 and no
      // type is infinite, so request_ready_o is low.
      if (fc_update_valid_i && !fc_init_seen_r) begin
        ph_consumed_r <= '0;
        pd_consumed_r <= '0;
        nph_consumed_r <= '0;
        npd_consumed_r <= '0;
        cplh_consumed_r <= '0;
        cpld_consumed_r <= '0;
      end else if (request_valid_i && request_ready_o) begin
        case (request_class_i)
          TLP_CREDIT_POSTED: begin
            ph_consumed_r <= ph_consumed_r + 1'b1;
            pd_consumed_r <= pd_consumed_r + request_data_credits_i;
          end
          TLP_CREDIT_COMPLETION: begin
            cplh_consumed_r <= cplh_consumed_r + 1'b1;
            cpld_consumed_r <= cpld_consumed_r + request_data_credits_i;
          end
          default: begin
            nph_consumed_r <= nph_consumed_r + 1'b1;
            npd_consumed_r <= npd_consumed_r + request_data_credits_i;
          end
        endcase
      end
      // Registered: high in the cycle after each cycle that presents an
      // unsatisfiable request, with nothing to acknowledge.
      if (request_valid_i && request_unsatisfiable)
        error_o <= 1'b1;
    end
  end

endmodule
