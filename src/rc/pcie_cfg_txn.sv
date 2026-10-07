// ---------------------------------------------------------------------------
// pcie_cfg_txn -- one Configuration Request, from command to classified outcome
//
// Author: Kourosh Ghahramani
// Silicon Systems Research Lab, University of Washington
//
// Purpose
//   Builds the PG213 descriptor for one configuration read or write, drives
//   it into pcie_rq_rc_top's RQ socket, matches the completion by the tag the
//   core assigned, reissues after CRS, and reports one outcome
//   (txn_outcome_e). It does not know which enumeration stage it serves; the
//   stage decides what an outcome means.
//
// Interfaces
//   Command       cmd_*: one request per handshake, accepted in S_IDLE.
//   Response      rsp_*, crs_retries_o: held until rsp_ready_i.
//   RQ socket     s_axis_rq_*: the descriptor beat, then a write's payload.
//   Tag           pcie_rq_tag_*: allocated and freed by tlp_request_tracker.
//   RC socket     m_axis_rc_*: completions. A late one for a timed-out tag gets
//                 no result from tlp_request_tracker and never arrives.
//   Timeout       cpl_timeout_*: the tag tlp_request_tracker timed out.
//   Link          link_active_i: DL_Active, the reference of the hold.
//
// Clock and reset
//   clk_i only. rst_i is synchronous and active high.
//
// Holds
//   No request leaves until link_active_i has been high for CFG_HOLD_CYCLES:
//   neither a command nor a CRS reissue. The Root Complex cannot see the
//   device's Fundamental Reset, so the 100 ms of PCIe Base Spec r2.1, §6.6.1
//   is counted from an event known to follow it, DL_Active (named in PCIe
//   Base Spec r3.0, §6.7.3.3). A low link_active_i restarts the count, so the
//   hold applies after every link-up.
//
//   A Root Complex must allow 1.0 s after a Conventional Reset before it
//   judges a device that fails to return a Successful Completion broken
//   (PCIe Base Spec r2.1, §6.6.1). Until link_active_i has been high for
//   CRS_WINDOW_CYCLES, every CRS is reissued; CRS_RETRY_MAX counts only once
//   the window has closed. The decision is taken when the CRS arrives, so
//   the last reissue can leave up to one backoff after the window closes,
//   and its CRS is the one reported TXN_CRS_EXHAUSTED.
//
// Limitations
//   One request at a time. Written for AXIS_DATA_WIDTH = 128: one descriptor
//   beat, read data from bits 127:96.
//
// References
//   PG213, Table 61
//   PG213, Table 65
//   PCIe Base Spec r2.1, §2.2.1
//   PCIe Base Spec r2.1, §2.2.7
//   PCIe Base Spec r2.1, §2.3.1
//   PCIe Base Spec r2.1, §2.3.2
//   PCIe Base Spec r2.1, §2.6.1
//   PCIe Base Spec r2.1, §6.6.1
//   PCIe Base Spec r3.0, §6.7.3.3
// ---------------------------------------------------------------------------
`timescale 1ns/1ps
module pcie_cfg_txn
  import pcie_rq_rc_pkg::*;
  import pcie_enum_pkg::*;
#(
    parameter int AXIS_DATA_WIDTH = 128,
    // PG213 tkeep is DWORD-granular on both RQ and RC: one bit per Dword.
    parameter int AXIS_KEEP_WIDTH = AXIS_DATA_WIDTH / 32,
    parameter int AXIS_USER_WIDTH = 60,

    // CRS retry policy, implementation specific; see pcie_enum_pkg.
    parameter int unsigned CRS_RETRY_MAX      = CRS_RETRY_MAX_DEFAULT,
    parameter int unsigned CRS_BACKOFF_CYCLES = CRS_BACKOFF_CYCLES_DEFAULT,

    // Not a timer. Used only by the elaboration check below, which compares
    // CRS_RETRY_MAX * CRS_BACKOFF_CYCLES with the completion timeout of
    // tlp_request_tracker; pass the value given to pcie_rq_rc_top. 0 disables
    // the check, as 0 disables that timeout. The default is 10 ms of an 8 ns
    // clock.
    parameter int unsigned CPL_TIMEOUT_CYCLES = tlp_pkg::CPL_TIMEOUT_DEFAULT_CYCLES,

    // The hold and the CRS window, in clk_i cycles from the rise of
    // link_active_i (see Holds above). 0 disables each; with both 0 no
    // counter is built.
    parameter int unsigned CFG_HOLD_CYCLES   = 0,
    parameter int unsigned CRS_WINDOW_CYCLES = 0
) (
    input  logic                        clk_i,
    input  logic                        rst_i,

    // DL_Active. Read only when CFG_HOLD_CYCLES or CRS_WINDOW_CYCLES is not
    // 0; left unconnected it reads 0, the hold never ends and nothing is
    // issued.
    input  logic                        link_active_i = 1'b0,

    // ---- command port ------------------------------------------------------
    // One request per handshake. Dword Count is always 1 and Last DW BE always
    // 0000b, as every Configuration Request requires (PCIe Base Spec r2.1,
    // §2.2.7), so neither is a port.
    input  logic                        cmd_valid_i,
    output logic                        cmd_ready_o,
    input  logic                        cmd_write_i,
    // Type 1 select, latched with the rest of the command. The descriptor is
    // built from the latched copy, so a CRS reissue keeps the type.
    input  logic                        cmd_type1_i,
    // The target BDF. It becomes the descriptor's Completer ID, which
    // pcie_rq_if places in the Bus, Device and Function fields of the request;
    // the descriptor's address field carries only the Register and Extended
    // Register Numbers.
    input  logic [15:0]                 cmd_bdf_i,
    input  logic [5:0]                  cmd_reg_num_i,
    input  logic [3:0]                  cmd_ext_reg_i,
    input  logic [3:0]                  cmd_first_be_i,
    input  logic [31:0]                 cmd_wdata_i,

    // ---- response port -----------------------------------------------------
    // rsp_valid_o is held, not pulsed: it stays high until rsp_ready_i, so a
    // busy consumer cannot miss an outcome.
    output logic                        rsp_valid_o,
    input  logic                        rsp_ready_i,
    output txn_outcome_e                rsp_outcome_o,
    output logic [31:0]                 rsp_rdata_o,
    // The Completion Status as received, for logging. A Reserved encoding
    // appears here unchanged although it is reported as TXN_UR.
    output logic [2:0]                  rsp_status_raw_o,
    // CRS reissues spent on the current transaction, saturating at
    // CRS_RETRY_MAX; 0 if it saw no CRS.
    output logic [$clog2(CRS_RETRY_MAX+1)-1:0] crs_retries_o,

    // ---- pcie_rq_rc_top socket: Requester Request --------------------------
    output logic [AXIS_DATA_WIDTH-1:0]  s_axis_rq_tdata_o,
    output logic [AXIS_KEEP_WIDTH-1:0]  s_axis_rq_tkeep_o,
    output logic                        s_axis_rq_tvalid_o,
    output logic                        s_axis_rq_tlast_o,
    output logic [AXIS_USER_WIDTH-1:0]  s_axis_rq_tuser_o,
    input  logic                        s_axis_rq_tready_i,

    // ---- pcie_rq_rc_top socket: core-managed tag ---------------------------
    input  logic [7:0]                  pcie_rq_tag_i,
    input  logic                        pcie_rq_tag_vld_i,

    // ---- pcie_rq_rc_top socket: Requester Completion -----------------------
    input  logic [AXIS_DATA_WIDTH-1:0]  m_axis_rc_tdata_i,
    input  logic [AXIS_KEEP_WIDTH-1:0]  m_axis_rc_tkeep_i,
    input  logic                        m_axis_rc_tvalid_i,
    input  logic                        m_axis_rc_tlast_i,
    output logic                        m_axis_rc_tready_o,

    // ---- pcie_rq_rc_top socket: completion timeout sideband ----------------
    input  logic                        cpl_timeout_valid_i,
    input  logic [7:0]                  cpl_timeout_tag_i
);

  localparam int CRS_CNT_W  = $clog2(CRS_RETRY_MAX + 1);
  localparam int BACKOFF_W  = $clog2(CRS_BACKOFF_CYCLES + 1);

  // -------------------------------------------------------------------------
  // Elaboration checks
  // -------------------------------------------------------------------------
  // They warn and let elaboration continue. The first two flag a
  // CRS_RETRY_MAX or CRS_BACKOFF_CYCLES below 1, which also makes CRS_CNT_W
  // or BACKOFF_W zero.
  initial begin
    if (CRS_RETRY_MAX < 1)
      $warning("pcie_cfg_txn: CRS_RETRY_MAX=%0d -- no CRS retry will ever be attempted, so a device that legally answers CRS is reported TXN_CRS_EXHAUSTED on its first completion.",
               CRS_RETRY_MAX);
    if (CRS_BACKOFF_CYCLES < 1)
      $warning("pcie_cfg_txn: CRS_BACKOFF_CYCLES=%0d -- retries are not paced and will be reissued back to back.",
               CRS_BACKOFF_CYCLES);
    // Warns when CRS_RETRY_MAX * CRS_BACKOFF_CYCLES is not below
    // CPL_TIMEOUT_CYCLES. Nothing at run time compares the two: no tag is held
    // during a backoff, and each reissue gets a new tag that
    // tlp_request_tracker times on its own, so a device that keeps answering
    // CRS ends as TXN_CRS_EXHAUSTED, not TXN_TIMEOUT.
    if (CPL_TIMEOUT_CYCLES != 0 &&
        (CRS_RETRY_MAX * CRS_BACKOFF_CYCLES) >= CPL_TIMEOUT_CYCLES)
      $warning("pcie_cfg_txn: P-CRS-BUDGET violated -- CRS_RETRY_MAX*CRS_BACKOFF_CYCLES = %0d >= CPL_TIMEOUT_CYCLES = %0d. A slow-to-initialise device will time out mid-retry and be misreported as dead.",
               CRS_RETRY_MAX * CRS_BACKOFF_CYCLES, CPL_TIMEOUT_CYCLES);
    // A window that closes before the hold ends never covers a request.
    if (CRS_WINDOW_CYCLES != 0 && CRS_WINDOW_CYCLES <= CFG_HOLD_CYCLES)
      $warning("pcie_cfg_txn: CRS_WINDOW_CYCLES=%0d <= CFG_HOLD_CYCLES=%0d -- the CRS window closes before the first request can leave.",
               CRS_WINDOW_CYCLES, CFG_HOLD_CYCLES);
  end

  // -------------------------------------------------------------------------
  // Hold and window timer
  // -------------------------------------------------------------------------
  // since_r counts the cycles link_active_i has been high and saturates; a
  // low link_active_i clears it, so the count starts again at each link-up.
  // hold_done gates the command handshake and the end of a CRS backoff;
  // crs_window_open lets a CRS be reissued past CRS_RETRY_MAX.
  localparam int unsigned SINCE_MAX = (CFG_HOLD_CYCLES > CRS_WINDOW_CYCLES) ?
                                      CFG_HOLD_CYCLES : CRS_WINDOW_CYCLES;
  localparam int          SINCE_W   = (SINCE_MAX < 1) ? 1 : $clog2(SINCE_MAX + 1);

  logic hold_done;
  logic crs_window_open;

  generate
    if (SINCE_MAX != 0) begin : g_since
      logic [SINCE_W-1:0] since_r;
      always_ff @(posedge clk_i) begin
        if (rst_i || !link_active_i)            since_r <= '0;
        else if (since_r != SINCE_W'(SINCE_MAX)) since_r <= since_r + SINCE_W'(1);
      end
      assign hold_done       = (CFG_HOLD_CYCLES == 0) ||
                               (link_active_i && (since_r >= SINCE_W'(CFG_HOLD_CYCLES)));
      assign crs_window_open = (CRS_WINDOW_CYCLES != 0) && link_active_i &&
                               (since_r < SINCE_W'(CRS_WINDOW_CYCLES));
    end else begin : g_no_since
      assign hold_done       = 1'b1;
      assign crs_window_open = 1'b0;
    end
  endgenerate

  // -------------------------------------------------------------------------
  // Latched command
  // -------------------------------------------------------------------------
  // Written only when a command is accepted in S_IDLE. The descriptor below is
  // a function of these registers alone, so a CRS reissue repeats the
  // original request exactly.
  logic        write_r;
  logic        type1_r;
  logic [15:0] bdf_r;
  logic [5:0]  reg_num_r;
  logic [3:0]  ext_reg_r;
  logic [3:0]  first_be_r;
  logic [31:0] wdata_r;

  // -------------------------------------------------------------------------
  // Transaction state machine
  // -------------------------------------------------------------------------
  //   State      Does                               Exit
  //   S_IDLE     cmd_ready_o once the hold is       cmd_valid_i: S_DESC
  //              done; latches a command
  //   S_DESC     drives the descriptor beat; arms   s_axis_rq_tready_i: S_DATA
  //              the tag capture                    (write) or S_WAIT (read)
  //   S_DATA     drives the payload beat            s_axis_rq_tready_i: S_WAIT
  //   S_WAIT     waits for the completion or the    CRS in the window or in
  //              timeout of the held tag            budget: S_BACKOFF;
  //                                                 otherwise: S_RESP
  //   S_BACKOFF  CRS_BACKOFF_CYCLES + 1 cycles,     count at 0 and hold
  //              then waits for the hold            done: S_DESC
  //   S_RESP     rsp_valid_o; holds the outcome     rsp_ready_i: S_IDLE
  typedef enum logic [2:0] {
    S_IDLE,
    S_DESC,
    S_DATA,
    S_WAIT,
    S_BACKOFF,
    S_RESP
  } txn_state_e;

  txn_state_e             state_r;
  logic [7:0]             tag_r;
  logic                   tag_valid_r;
  logic                   awaiting_tag_r;
  logic [CRS_CNT_W-1:0]   crs_count_r;
  logic [BACKOFF_W-1:0]   backoff_r;
  logic [31:0]            rdata_r;
  logic [2:0]             status_raw_r;
  txn_outcome_e           outcome_r;
  logic                   rc_in_packet_r;

  // -------------------------------------------------------------------------
  // RQ descriptor
  // -------------------------------------------------------------------------
  // The Configuration form of the requester request descriptor (PG213, Table
  // 61), built from the latched command. Fields left zero:
  //   tag [103:96]           not used: tlp_request_tracker assigns the tag
  //   requester_id [95:80]   not used: the Transaction Layer takes
  //                          requester_id_i; requester_id_en [120],
  //                          force_ecrc [127] and poisoned [79] stay 0
  //   tc [123:121], attr [126:124]  must be 0 for a Configuration Request
  //                          (PCIe Base Spec r2.1, §2.2.7)
  //   address [63:12], [1:0] reserved in the Configuration form; first_be
  //                          selects the bytes within the Dword
  rq_descriptor_t desc;
  always_comb begin
    desc              = '0;
    desc.completer_id = bdf_r;                                 // the target BDF
    // Read or write by direction, Type 0 or Type 1 by the latched select. Each
    // Type 1 encoding differs from its Type 0 partner in bit 0 only, as the
    // TLP Type fields of CfgRd0 and CfgRd1 do (PCIe Base Spec r2.1, §2.2.1,
    // Table 2-3).
    desc.req_type     = write_r ? (type1_r ? RQ_CFG_WRITE1 : RQ_CFG_WRITE0)
                                : (type1_r ? RQ_CFG_READ1  : RQ_CFG_READ0);
    desc.dword_count  = CFG_DWORD_COUNT;                       // always 1
    desc.address      = 64'd0;
    desc.address[11:8] = ext_reg_r;                            // Ext Reg Number
    desc.address[7:2]  = reg_num_r;                            // Register Number
  end

  wire driving_desc = (state_r == S_DESC);
  wire driving_data = (state_r == S_DATA);

  assign s_axis_rq_tdata_o  = driving_data ? {{(AXIS_DATA_WIDTH-32){1'b0}}, wdata_r}
                                           : AXIS_DATA_WIDTH'(desc);
  // Descriptor beat: all four Dwords. Payload beat: one Dword.
  assign s_axis_rq_tkeep_o  = driving_data ? AXIS_KEEP_WIDTH'(1) : '1;
  assign s_axis_rq_tvalid_o = driving_desc || driving_data;
  // A read is a single-beat packet; a write ends on its payload beat.
  // pcie_rq_if checks tlast against the Dword Count and flags a mismatch as
  // RQ_ERR_EARLY_LAST or RQ_ERR_MISSING_LAST.
  assign s_axis_rq_tlast_o  = (driving_desc && !write_r) || driving_data;
  assign s_axis_rq_tuser_o  = {{(AXIS_USER_WIDTH-8){1'b0}}, CFG_LAST_BE, first_be_r};

  // -------------------------------------------------------------------------
  // Completion receive
  // -------------------------------------------------------------------------
  // m_axis_rc_tready_o is tied high: every completion beat is accepted and
  // only one that matches the held tag is used. Holding it low would keep
  // pcie_rc_if from taking the next completion header, which stalls
  // tlp_layer's receive parser and every TLP behind that completion.
  //
  // Beat 0 carries the 3-Dword RC descriptor in Dwords 0-2 and the first
  // payload Dword in Dword 3 (pcie_rq_rc_pkg, rc_descriptor_t). A
  // configuration completion is one beat (Dword Count 1 or 0); rc_in_packet_r
  // tracks later beats anyway, so an oversized packet cannot be decoded as a
  // new descriptor.
  assign m_axis_rc_tready_o = 1'b1;

  rc_descriptor_t rc_desc;
  assign rc_desc = rc_descriptor_t'(m_axis_rc_tdata_i[95:0]);
  wire [31:0] rc_payload_dw0 = m_axis_rc_tdata_i[127:96];

  wire rc_fire  = m_axis_rc_tvalid_i && m_axis_rc_tready_o;
  wire rc_beat0 = rc_fire && !rc_in_packet_r;

  // Match on the tag the core put on the wire, and finish on Request Completed
  // (descriptor bit 30, the last completion of the request; PG213, Table 65)
  // rather than on tlast, which ends one completion. For a configuration
  // request the two coincide.
  wire rc_match = rc_beat0 && tag_valid_r && (rc_desc.tag == tag_r);
  wire rc_done  = rc_match && rc_desc.request_completed;

  // A timeout counts only if it names the held tag. tag_valid_r is low from a
  // CRS completion until the reissue's tag strobe, so no timeout in that
  // window is taken for this request.
  wire timeout_match = cpl_timeout_valid_i && tag_valid_r &&
                       (cpl_timeout_tag_i == tag_r);

  // -------------------------------------------------------------------------
  // Completion Status decode
  // -------------------------------------------------------------------------
  // All eight encodings are named and there is no default arm: a Reserved
  // encoding is treated as Unsupported Request (PCIe Base Spec r2.1, §2.3.2).
  txn_outcome_e status_outcome;
  logic         status_is_crs;
  always_comb begin
    status_is_crs  = 1'b0;
    unique case (rc_desc.completion_status)
      3'b000: status_outcome = TXN_OK;                              // SC
      3'b001: status_outcome = TXN_UR;                              // UR
      3'b010: begin status_outcome = TXN_UR; status_is_crs = 1'b1; end // CRS
      3'b100: status_outcome = TXN_CA;                              // CA
      3'b011: status_outcome = TXN_UR;                              // Reserved
      3'b101: status_outcome = TXN_UR;                              // Reserved
      3'b110: status_outcome = TXN_UR;                              // Reserved
      3'b111: status_outcome = TXN_UR;                              // Reserved
    endcase
  end

  wire crs_budget_spent = (crs_count_r >= CRS_CNT_W'(CRS_RETRY_MAX));

  // The state machine; see the state table above the state declarations.
  always_ff @(posedge clk_i) begin
    if (rst_i) begin
      state_r        <= S_IDLE;
      write_r        <= 1'b0;
      type1_r        <= 1'b0;
      bdf_r          <= '0;
      reg_num_r      <= '0;
      ext_reg_r      <= '0;
      first_be_r     <= '0;
      wdata_r        <= '0;
      tag_r          <= '0;
      tag_valid_r    <= 1'b0;
      awaiting_tag_r <= 1'b0;
      crs_count_r    <= '0;
      backoff_r      <= '0;
      rdata_r        <= '0;
      status_raw_r   <= '0;
      outcome_r      <= TXN_OK;
      rc_in_packet_r <= 1'b0;
    end else begin
      unique case (state_r)
        S_IDLE: begin
          if (cmd_valid_i && hold_done) begin
            write_r      <= cmd_write_i;
            type1_r      <= cmd_type1_i;
            bdf_r        <= cmd_bdf_i;
            reg_num_r    <= cmd_reg_num_i;
            ext_reg_r    <= cmd_ext_reg_i;
            first_be_r   <= cmd_first_be_i;
            wdata_r      <= cmd_wdata_i;
            crs_count_r  <= '0;
            rdata_r      <= '0;
            status_raw_r <= '0;
            tag_valid_r  <= 1'b0;
            state_r      <= S_DESC;
          end
        end

        // The command enters the Transaction Layer with the descriptor beat,
        // so the tag capture is armed on its handshake.
        S_DESC: begin
          if (s_axis_rq_tready_i) begin
            awaiting_tag_r <= 1'b1;
            state_r        <= write_r ? S_DATA : S_WAIT;
          end
        end

        S_DATA: begin
          if (s_axis_rq_tready_i) state_r <= S_WAIT;
        end

        S_WAIT: begin
          // The timeout arm comes first. After a timeout the tag is
          // quarantined: tlp_request_tracker gives a late completion for it
          // no result, so no RC packet follows.
          if (timeout_match) begin
            outcome_r <= TXN_TIMEOUT;
            state_r   <= S_RESP;
          end else if (rc_done) begin
            status_raw_r <= rc_desc.completion_status;
            if (status_is_crs) begin
              if (crs_budget_spent && !crs_window_open) begin
                outcome_r <= TXN_CRS_EXHAUSTED;
                state_r   <= S_RESP;
              end else begin
                // A CRS completion terminates the request (PCIe Base Spec
                // r2.1, §2.3.1), so the reissue is a new request with a new
                // tag. The old tag is dropped before reissuing. Inside the
                // window the count saturates, so a CRS after it finds the
                // budget spent.
                if (!crs_budget_spent) crs_count_r <= crs_count_r + CRS_CNT_W'(1);
                backoff_r   <= BACKOFF_W'(CRS_BACKOFF_CYCLES);
                tag_valid_r <= 1'b0;
                state_r     <= S_BACKOFF;
              end
            end else begin
              // Read data is kept only on a Successful Completion: a read
              // completion with any other status carries no data (PCIe Base
              // Spec r2.1, §2.3.2).
              if ((status_outcome == TXN_OK) && !write_r) rdata_r <= rc_payload_dw0;
              outcome_r <= status_outcome;
              state_r   <= S_RESP;
            end
          end
        end

        // A reissue waits for the hold too: a link that went down during the
        // backoff restarts it.
        S_BACKOFF: begin
          if (backoff_r != '0) backoff_r <= backoff_r - BACKOFF_W'(1);
          else if (hold_done)  state_r   <= S_DESC;
        end

        S_RESP: begin
          if (rsp_ready_i) begin
            tag_valid_r <= 1'b0;
            state_r     <= S_IDLE;
          end
        end
      endcase

      // ---- tag capture, outside the case ---------------------------------
      // Not in a state arm: for a write the strobe can arrive while the
      // payload beat is still being driven, in S_DATA. awaiting_tag_r is set
      // by the descriptor handshake and is still low in that cycle, so a
      // strobe in the same cycle is not captured; pcie_rq_rc_top strobes a
      // tag only after the command has passed pcie_rq_if into the
      // Transaction Layer.
      if (awaiting_tag_r && pcie_rq_tag_vld_i) begin
        tag_r          <= pcie_rq_tag_i;
        tag_valid_r    <= 1'b1;
        awaiting_tag_r <= 1'b0;
      end

      // ---- RC packet boundary tracking ------------------------------------
      if (rc_fire) rc_in_packet_r <= !m_axis_rc_tlast_i;
    end
  end

  // One request in flight. A receiver may advertise a single NPH credit
  // (PCIe Base Spec r2.1, §2.6.1, Table 2-37), and every Configuration Request
  // consumes one (Table 2-36), so a second request might wait for credit
  // anyway. With one request the completion match is a single tag register.
  assign cmd_ready_o      = (state_r == S_IDLE) && hold_done;
  assign rsp_valid_o      = (state_r == S_RESP);
  assign rsp_outcome_o    = outcome_r;
  assign rsp_rdata_o      = rdata_r;
  assign rsp_status_raw_o = status_raw_r;
  assign crs_retries_o    = crs_count_r;

endmodule
