// ---------------------------------------------------------------------------
// tlp2dllp -- TLP framing: credit check, sequence number and LCRC
//
//!module: tlp2dllp
//! Author: Idris Somoye
//
// Purpose
//   Takes one TLP at a time from the Transaction Layer side, checks the
//   peer's flow control credits for its type, and emits it with the
//   sequence number in front and the LCRC behind. A TLP is taken only while
//   the retry buffer has a free slot, and each framed TLP is reported to
//   retry_management with its sequence number.
//
// Interfaces
//   TLP input     s_axis_*: one DW per beat, DW0 first; a TLP prefix may
//                 precede DW0.
//   Framed output m_axis_*: the 4 Reserved bits and 12-bit sequence number,
//                 the TLP, and the 32-bit LCRC. The TLP sits two bytes later
//                 than at the input: the first beat holds the sequence number
//                 and the first two TLP bytes, the last beat two LCRC bytes
//                 (tkeep 0011b, tlast). tuser is 2 on every beat: bit 1 marks
//                 a TLP, so USER_WIDTH must be at least 2.
//   Retry         retry_available_i: a retry slot is free. dllp_valid_o,
//                 seq_num_o: one cycle per framed TLP, with its sequence
//                 number, to retry_management.
//   Credits       tx_fc_*_i, update_fc_i: the peer's credit limits, taken
//                 while update_fc_i is high.
//
// Clock and reset
//   clk_i only. rst_i is synchronous and active high; it sets the sequence
//   number to 000h and every credit count and limit to 0.
//
// Limitations
//   DATA_WIDTH = 32 only: the framing moves the stream by half a beat.
//   A limit of 0 is infinite for CPLH and CPLD only; an advertised P or NP
//   limit of 0 blocks that type. An AtomicOp Request is charged one NPD
//   credit whatever its length. A TLP whose Fmt and Type match no credit
//   check stays in ST_IDLE or ST_PREFIX and blocks the input. No TLP is
//   nullified. The PD, NPD and CPLD checks subtract in 16 bits, not modulo
//   2^12: while a PD or CPLD limit has wrapped past 0 and its consumed count
//   has not, that check passes a TLP of any length. ST_CHECK_CREDITS,
//   retry_index_i, MAX_PAYLOAD_SIZE, S_COUNT and the RAM_* parameters are
//   not used.
//
// Structure
//   Credit-limit comparisons (fc8_is_forward, fc12_is_forward)
//   State registers
//   LCRC output mapping (byteswap)
//   Framing state machine (main_seq)
//   Credit limits (flow_contol)
//   Stream registers (three axis_register instances)
//   LCRC generators (pcie_lcrc16, pcie_lcrc32)
//
// References
//   PCIe Base Spec r2.1, §2.2.1
//   PCIe Base Spec r2.1, §2.6.1
//   PCIe Base Spec r2.1, §2.6.1.1
//   PCIe Base Spec r2.1, §3.5.1
//   PCIe Base Spec r2.1, §3.5.2.1
// ---------------------------------------------------------------------------
module tlp2dllp
  import pcie_datalink_pkg::*;
#(
    parameter int USER_WIDTH        = 1,
    parameter int S_COUNT           = 1,
    parameter int DATA_WIDTH        = 32,
    parameter int MAX_PAYLOAD_SIZE  = 0,
    parameter int MaxNumWordsPerHdr = (128 / DATA_WIDTH),
    parameter int KEEP_WIDTH        = ((DATA_WIDTH) / 8),
    parameter int RAM_DATA_WIDTH    = DATA_WIDTH,

    parameter int MaxBytesPerTLP = MAX_PAYLOAD_SIZE,
    parameter int MaxNumWordsPerTLP = (MaxBytesPerTLP / (DATA_WIDTH / 8)) + MaxNumWordsPerHdr + 2,
    parameter int RAM_ADDR_WIDTH = $clog2(MaxNumWordsPerTLP)
) (
    input  logic                  clk_i,
    input  logic                  rst_i,
    // ---- TLP input ---------------------------------------------------------
    input  logic [DATA_WIDTH-1:0] s_axis_tdata,
    input  logic [KEEP_WIDTH-1:0] s_axis_tkeep,
    input  logic                  s_axis_tvalid,
    input  logic                  s_axis_tlast,
    input  logic [USER_WIDTH-1:0] s_axis_tuser,
    output logic                  s_axis_tready,
    // ---- framed TLP output -------------------------------------------------
    output logic [DATA_WIDTH-1:0] m_axis_tdata,
    output logic [KEEP_WIDTH-1:0] m_axis_tkeep,
    output logic                  m_axis_tvalid,
    output logic                  m_axis_tlast,
    output logic [USER_WIDTH-1:0] m_axis_tuser,
    input  logic                  m_axis_tready,
    // ---- retry_management --------------------------------------------------
    output logic [          15:0] seq_num_o,
    output logic                  dllp_valid_o,
    input  logic                  retry_available_i,
    input  logic [           7:0] retry_index_i,
    // ---- peer credit limits ------------------------------------------------
    input  logic [           7:0] tx_fc_ph_i,
    input  logic [          11:0] tx_fc_pd_i,
    input  logic [           7:0] tx_fc_nph_i,
    input  logic [          11:0] tx_fc_npd_i,
    input  logic [           7:0] tx_fc_cplh_i,
    input  logic [          11:0] tx_fc_cpld_i,
    input  logic                  update_fc_i

);

  // Not used.
  localparam int FcPldSize = MAX_PAYLOAD_SIZE >> 4;
  localparam int FcHeaderFieldSize = 8;
  localparam int FCDataFieldSize = 12;
  localparam int FcHeaderFieldSizeDiv2 = FcHeaderFieldSize / 2;
  localparam int FCDataFieldSizeDiv2 = FCDataFieldSize / 2;


  //tlp to dllp fsm emum
  typedef enum logic [3:0] {
    ST_IDLE,
    ST_PREFIX,
    ST_CHECK_CREDITS_NPH,
    ST_CHECK_CREDITS_NPH_NPD,
    ST_CHECK_CREDITS_PH,
    ST_CHECK_CREDITS_PH_PD,
    ST_CHECK_CREDITS_CPLH,
    ST_CHECK_CREDITS_CPLH_CPLD,
    ST_TLP_STREAM,
    ST_TLP_CRC,
    ST_TLP_CRC_ALIGN,
    ST_TLP_CRC_TLAST_ALIGN,
    ST_TLP_LAST,
    ST_CHECK_CREDITS
  } dll_tx_st_e;

  //fsm holder signals
  dll_tx_st_e                            curr_state;
  dll_tx_st_e                            next_state;
  // next_transmit_seq_r is NEXT_TRANSMIT_SEQ. prefix_r: the TLP being
  // framed starts with a prefix, held in the flow register.
  logic                 [          11:0] next_transmit_seq_c;
  logic                 [          11:0] next_transmit_seq_r;
  logic                                  prefix_c;
  logic                                  prefix_r;
  //skid buffer axis stage1 signals
  logic                 [DATA_WIDTH-1:0] skid_axis_tdata;
  logic                 [KEEP_WIDTH-1:0] skid_axis_tkeep;
  logic                                  skid_axis_tvalid;
  logic                                  skid_axis_tlast;
  logic                 [USER_WIDTH-1:0] skid_axis_tuser;
  logic                                  skid_axis_tready;
  //flow buffer axis stage1 signals; only tdata and tvalid are read
  logic                 [DATA_WIDTH-1:0] pipeline_axis_tdata;
  logic                 [KEEP_WIDTH-1:0] pipeline_axis_tkeep;
  logic                                  pipeline_axis_tvalid;
  logic                                  pipeline_axis_tlast;
  logic                 [USER_WIDTH-1:0] pipeline_axis_tuser;
  logic                                  pipeline_axis_tready;
  //tlp output buffer axis signals
  logic                 [DATA_WIDTH-1:0] tlp_axis_tdata;
  logic                 [KEEP_WIDTH-1:0] tlp_axis_tkeep;
  logic                                  tlp_axis_tvalid;
  logic                                  tlp_axis_tlast;
  logic                 [USER_WIDTH-1:0] tlp_axis_tuser;
  logic                                  tlp_axis_tready;
  // crc_in_r is the running LCRC. crc_tlp_axis_tdata, crc_select, dllp_lcrc_c,
  // dllp_lcrc_r, dllp_crc_out and dllp_lcrc32d32 are not read.
  logic                 [DATA_WIDTH-1:0] crc_tlp_axis_tdata;
  logic                 [          31:0] crc_in_c;
  logic                 [          31:0] dllp_lcrc_c;
  logic                 [          31:0] crc_in_r;
  logic                 [          31:0] dllp_lcrc_r;
  logic                 [          31:0] crc_out_32;
  logic                 [          31:0] crc_out_16;
  logic                 [           1:0] crc_select;
  logic                 [          31:0] lcrc32d32;
  logic                 [          15:0] dllp_crc_out;
  logic                 [          15:0] dllp_lcrc32d32;
  // Not used: no TLP is nullified.
  logic                                  tlp_nullified_c;
  logic                                  tlp_nullified_r;
  // tlp_dw0 decodes DW0; initial_axis_tdata is the first output beat.
  // initial_crc_tdata feeds only crc_tlp_axis_tdata, which is not read.
  pcie_tlp_header_dw0_t                  tlp_dw0;
  logic                 [DATA_WIDTH-1:0] initial_axis_tdata;
  logic                 [DATA_WIDTH-1:0] initial_crc_tdata;
  logic                                  initial_axis_tvalid;
  // CREDITS_CONSUMED per credit type, modulo 2^8 or 2^12.
  logic                 [           7:0] ph_credits_consumed_c;
  logic                 [           7:0] ph_credits_consumed_r;
  logic                 [          11:0] pd_credits_consumed_c;
  logic                 [          11:0] pd_credits_consumed_r;
  logic                 [          11:0] npd_credits_consumed_c;
  logic                 [          11:0] npd_credits_consumed_r;
  logic                 [           7:0] nph_credits_consumed_c;
  logic                 [           7:0] nph_credits_consumed_r;
  logic                 [          11:0] cpld_credits_consumed_c;
  logic                 [          11:0] cpld_credits_consumed_r;
  logic                 [           7:0] cplh_credits_consumed_c;
  logic                 [           7:0] cplh_credits_consumed_r;
  // CREDIT_LIMIT per credit type. cplh_credit_limit is 12 bits wide but is
  // loaded from 8 bits, so bits [11:8] stay 0.
  logic                 [           7:0] ph_credit_limit_c;
  logic                 [           7:0] ph_credit_limit_r;
  logic                 [          11:0] pd_credit_limit_c;
  logic                 [          11:0] pd_credit_limit_r;
  logic                 [          11:0] cpld_credit_limit_c;
  logic                 [          11:0] cpld_credit_limit_r;
  logic                 [           7:0] nph_credit_limit_c;
  logic                 [           7:0] nph_credit_limit_r;
  logic                 [          11:0] npd_credit_limit_c;
  logic                 [          11:0] npd_credit_limit_r;
  logic                 [          11:0] cplh_credit_limit_c;
  logic                 [          11:0] cplh_credit_limit_r;

  // -------------------------------------------------------------------------
  // Credit-limit comparisons
  // -------------------------------------------------------------------------
  // fc8_is_forward and fc12_is_forward are true when new_value equals
  // old_value or is ahead of it by less than half the field, modulo 2^8 for
  // header credits and 2^12 for data credits. flow_contol uses them to
  // ignore an update that would move a credit limit backwards.
  function automatic logic fc8_is_forward(
      input logic [7:0] new_value,
      input logic [7:0] old_value
  );
    logic [7:0] distance;
    begin
      distance = new_value - old_value;
      fc8_is_forward = (distance == '0) || !distance[7];
    end
  endfunction

  function automatic logic fc12_is_forward(
      input logic [11:0] new_value,
      input logic [11:0] old_value
  );
    logic [11:0] distance;
    begin
      distance = new_value - old_value;
      fc12_is_forward = (distance == '0) || !distance[11];
    end
  endfunction




  // -------------------------------------------------------------------------
  // State registers
  // -------------------------------------------------------------------------
  // The state, NEXT_TRANSMIT_SEQ, the prefix flag, and CREDITS_CONSUMED and
  // CREDIT_LIMIT per credit type. crc_in_r is assigned again after the reset
  // branch, so its reset value never takes effect; the LCRC seed FFFF FFFFh
  // is loaded in ST_IDLE and ST_TLP_LAST instead.
  always @(posedge clk_i) begin
    if (rst_i) begin
      curr_state              <= ST_IDLE;
      next_transmit_seq_r     <= '0;
      prefix_r                <= 1'b0;
      crc_in_r                <= '1;
      ph_credits_consumed_r   <= '0;
      pd_credits_consumed_r   <= '0;
      nph_credits_consumed_r  <= '0;
      npd_credits_consumed_r  <= '0;
      cplh_credits_consumed_r <= '0;
      cpld_credits_consumed_r <= '0;
      ph_credit_limit_r       <= '0;
      pd_credit_limit_r       <= '0;
      nph_credit_limit_r      <= '0;
      npd_credit_limit_r      <= '0;
      cpld_credit_limit_r     <= '0;
      cplh_credit_limit_r     <= '0;
    end else begin
      curr_state              <= next_state;
      next_transmit_seq_r     <= next_transmit_seq_c;
      prefix_r                <= prefix_c;
      ph_credits_consumed_r   <= ph_credits_consumed_c;
      pd_credits_consumed_r   <= pd_credits_consumed_c;
      nph_credits_consumed_r  <= nph_credits_consumed_c;
      npd_credits_consumed_r  <= npd_credits_consumed_c;
      cplh_credits_consumed_r <= cplh_credits_consumed_c;
      cpld_credits_consumed_r <= cpld_credits_consumed_c;
      ph_credit_limit_r       <= ph_credit_limit_c;
      pd_credit_limit_r       <= pd_credit_limit_c;
      nph_credit_limit_r      <= nph_credit_limit_c;
      npd_credit_limit_r      <= npd_credit_limit_c;
      cpld_credit_limit_r     <= cpld_credit_limit_c;
      cplh_credit_limit_r     <= cplh_credit_limit_c;
    end
    crc_in_r <= crc_in_c;
  end

  // -------------------------------------------------------------------------
  // LCRC output mapping
  // -------------------------------------------------------------------------
  // lcrc32d32 is crc_in_r complemented, as the LCRC remainder is (PCIe Base
  // Spec r2.1, §3.5.2.1), with its 32 bits in reverse order: lcrc32d32[31]
  // is ~crc_in_r[0]. ST_TLP_CRC_ALIGN sends lcrc32d32[15:0] and
  // ST_TLP_CRC_TLAST_ALIGN sends lcrc32d32[31:16].
  always_comb begin : byteswap
    lcrc32d32 = {
      ~crc_in_r[0],
      ~crc_in_r[1],
      ~crc_in_r[2],
      ~crc_in_r[3],
      ~crc_in_r[4],
      ~crc_in_r[5],
      ~crc_in_r[6],
      ~crc_in_r[7],
      ~crc_in_r[8],
      ~crc_in_r[9],
      ~crc_in_r[10],
      ~crc_in_r[11],
      ~crc_in_r[12],
      ~crc_in_r[13],
      ~crc_in_r[14],
      ~crc_in_r[15],
      ~crc_in_r[16],
      ~crc_in_r[17],
      ~crc_in_r[18],
      ~crc_in_r[19],
      ~crc_in_r[20],
      ~crc_in_r[21],
      ~crc_in_r[22],
      ~crc_in_r[23],
      ~crc_in_r[24],
      ~crc_in_r[25],
      ~crc_in_r[26],
      ~crc_in_r[27],
      ~crc_in_r[28],
      ~crc_in_r[29],
      ~crc_in_r[30],
      ~crc_in_r[31]
    };
  end


  // -------------------------------------------------------------------------
  // Framing state machine
  // -------------------------------------------------------------------------
  // Each output beat takes the upper half of the previous input DW
  // (pipeline_axis_tdata) and the lower half of the current one
  // (skid_axis_tdata). In the first beat the Reserved bits and the sequence
  // number take the place of the previous half. crc_in_r accumulates the
  // LCRC over the beats as sent, from FFFF FFFFh (PCIe Base Spec r2.1,
  // §3.5.2.1).
  // A credit-check state sends the first beat once its credits allow.
  //   State                       Action                       Exit
  //   ST_IDLE                     seeds the LCRC, decodes DW0  ST_PREFIX or a check
  //   ST_PREFIX                   holds prefix, decodes DW0    a credit check
  //   ST_CHECK_CREDITS_NPH        checks NPH                   ST_TLP_STREAM
  //   ST_CHECK_CREDITS_NPH_NPD    checks NPH and NPD           ST_TLP_STREAM
  //   ST_CHECK_CREDITS_PH         checks PH                    ST_TLP_STREAM
  //   ST_CHECK_CREDITS_PH_PD      checks PH and PD             ST_TLP_STREAM
  //   ST_CHECK_CREDITS_CPLH       checks CPLH                  ST_TLP_STREAM
  //   ST_CHECK_CREDITS_CPLH_CPLD  checks CPLH and CPLD         ST_TLP_STREAM
  //   ST_TLP_STREAM               one beat per input beat      tlast: ST_TLP_CRC
  //   ST_TLP_CRC                  LCRC over last two bytes     ST_TLP_CRC_ALIGN
  //   ST_TLP_CRC_ALIGN            last two bytes, LCRC[15:0]   ST_TLP_CRC_TLAST_ALIGN
  //   ST_TLP_CRC_TLAST_ALIGN      LCRC[31:16], tlast           ST_TLP_LAST
  //   ST_TLP_LAST                 dllp_valid_o, sequence + 1   ST_IDLE
  //   ST_CHECK_CREDITS            never entered
  always_comb begin : main_seq
    next_state              = curr_state;
    tlp_axis_tdata          = '0;
    tlp_axis_tkeep          = '0;
    tlp_axis_tvalid         = '0;
    tlp_axis_tlast          = '0;
    skid_axis_tready        = '0;
    tlp_axis_tuser          = USER_WIDTH'(2);
    crc_select              = '1;
    crc_in_c                = crc_in_r;
    dllp_valid_o            = '0;
    tlp_dw0                 = '0;
    crc_tlp_axis_tdata      = '0;
    ph_credits_consumed_c   = ph_credits_consumed_r;
    pd_credits_consumed_c   = pd_credits_consumed_r;
    nph_credits_consumed_c  = nph_credits_consumed_r;
    npd_credits_consumed_c  = npd_credits_consumed_r;
    cplh_credits_consumed_c = cplh_credits_consumed_r;
    cpld_credits_consumed_c = cpld_credits_consumed_r;
    next_transmit_seq_c     = next_transmit_seq_r;
    prefix_c                = prefix_r;
    initial_axis_tdata      = {
      skid_axis_tdata[15:0], next_transmit_seq_r[7:0],
      4'h0, next_transmit_seq_r[11:8]
    };
    initial_crc_tdata       = {
      skid_axis_tdata[15:0], 4'h0,
      next_transmit_seq_r[11:8], next_transmit_seq_r[7:0]
    };
    initial_axis_tvalid     = skid_axis_tvalid;
    if (prefix_r) begin
      initial_axis_tdata = {
        pipeline_axis_tdata[15:0], next_transmit_seq_r[7:0],
        4'h0, next_transmit_seq_r[11:8]
      };
      initial_crc_tdata = {
        pipeline_axis_tdata[15:0], 4'h0,
        next_transmit_seq_r[11:8], next_transmit_seq_r[7:0]
      };
      initial_axis_tvalid = pipeline_axis_tvalid;
    end
    case (curr_state)
      // Seeds the LCRC and picks the credit check for DW0's Fmt and Type
      // (PCIe Base Spec r2.1, §2.6.1).
      ST_IDLE: begin
        // Do not remove a TLP from the input buffer unless retry storage has a
        // free slot.  Every transmitted TLP must be replayable until ACKed.
        if (skid_axis_tvalid && retry_available_i) begin
          crc_in_c = '1;
          if (skid_axis_tdata[7:5] == TLP_PREFIX) begin
            // Hold the prefix in the flow register. Credit classification must
            // use the following DW0, while the sequence number is still placed
            // before the prefix on the transmitted packet.
            skid_axis_tready = 1'b1;
            prefix_c = 1'b1;
            next_state = ST_PREFIX;
          end else if (tlp_axis_tready) begin
            tlp_dw0 = skid_axis_tdata;
            if (tlp_dw0.byte0 inside {MRd, MRdLk, IORd, CfgRd0, CfgRd1, TCfgRd}) begin
              next_state = ST_CHECK_CREDITS_NPH;
            end else if (tlp_dw0.byte0 inside {MWr, MsgD}) begin
              next_state = ST_CHECK_CREDITS_PH_PD;
            end else if (tlp_dw0.byte0 inside {Msg}) begin
              next_state = ST_CHECK_CREDITS_PH;
            end else if (tlp_dw0.byte0 inside {IOWr, CfgWr0, CfgWr1,TCfgWr,FetchAdd,
            Swap,CAS}) begin
              next_state = ST_CHECK_CREDITS_NPH_NPD;
            end else if (tlp_dw0.byte0 inside {Cpl, CplLk}) begin
              next_state = ST_CHECK_CREDITS_CPLH;
            end else if (tlp_dw0.byte0 inside {CplD, CplDLk}) begin
              next_state = ST_CHECK_CREDITS_CPLH_CPLD;
            end
          end
        end
      end
      ST_PREFIX: begin
        // The prefix is retained in pipeline_axis_tdata and DW0 remains at the
        // skid output until the appropriate credit check succeeds.
        if (skid_axis_tvalid) begin
          tlp_dw0 = skid_axis_tdata;
          if (tlp_dw0.byte0 inside {MRd, MRdLk, IORd, CfgRd0, CfgRd1, TCfgRd})
            next_state = ST_CHECK_CREDITS_NPH;
          else if (tlp_dw0.byte0 inside {MWr, MsgD})
            next_state = ST_CHECK_CREDITS_PH_PD;
          else if (tlp_dw0.byte0 inside {Msg})
            next_state = ST_CHECK_CREDITS_PH;
          else if (tlp_dw0.byte0 inside {IOWr, CfgWr0, CfgWr1,TCfgWr,FetchAdd,
          Swap,CAS})
            next_state = ST_CHECK_CREDITS_NPH_NPD;
          else if (tlp_dw0.byte0 inside {Cpl, CplLk})
            next_state = ST_CHECK_CREDITS_CPLH;
          else if (tlp_dw0.byte0 inside {CplD, CplDLk})
            next_state = ST_CHECK_CREDITS_CPLH_CPLD;
        end
      end
      ST_CHECK_CREDITS_NPH: begin
        static logic has_nph_credit;
        has_nph_credit = '0;
        tlp_axis_tdata = initial_axis_tdata;
        crc_tlp_axis_tdata = initial_crc_tdata;
        tlp_axis_tkeep = '1;
        if ((nph_credit_limit_r - nph_credits_consumed_r) >= 1'b1) begin
          has_nph_credit         = '1;
        end
        if (has_nph_credit) begin
          tlp_axis_tvalid = initial_axis_tvalid;
          skid_axis_tready = !prefix_r && tlp_axis_tready;
          if (initial_axis_tvalid && tlp_axis_tready) begin
            nph_credits_consumed_c = nph_credits_consumed_r + 1'b1;
            crc_in_c = crc_out_32;
            next_state = ST_TLP_STREAM;
          end
        end
      end
      ST_CHECK_CREDITS_NPH_NPD: begin
        static logic has_nph_credit;
        static logic has_npd_credit;
        static logic [15:0] data_credits_required;
        has_nph_credit = '0;
        has_npd_credit = '0;
        // I/O and Configuration Writes carry one DW, one NPD credit (PCIe Base
        // Spec r2.1, §2.6.1); AtomicOp Requests are charged the same here.
        data_credits_required = 1'b1;
        tlp_axis_tdata = initial_axis_tdata;
        crc_tlp_axis_tdata = initial_crc_tdata;
        tlp_axis_tkeep = '1;
        if ((nph_credit_limit_r - nph_credits_consumed_r) >= 1'b1) begin
          has_nph_credit = '1;
        end
        if ((npd_credit_limit_r - npd_credits_consumed_r) >= data_credits_required) begin
          has_npd_credit = '1;
        end
        if (has_nph_credit && has_npd_credit) begin
          tlp_axis_tvalid = initial_axis_tvalid;
          skid_axis_tready = !prefix_r && tlp_axis_tready;
          if (initial_axis_tvalid && tlp_axis_tready) begin
            nph_credits_consumed_c = nph_credits_consumed_r + 1'b1;
            npd_credits_consumed_c = npd_credits_consumed_r + data_credits_required;
            crc_in_c = crc_out_32;
            next_state = ST_TLP_STREAM;
          end
        end
      end
      ST_CHECK_CREDITS_PH: begin
        static logic has_ph_credit;
        has_ph_credit = '0;
        tlp_axis_tdata = initial_axis_tdata;
        crc_tlp_axis_tdata = initial_crc_tdata;
        tlp_axis_tkeep = '1;
        if ((ph_credit_limit_r - ph_credits_consumed_r) >= 1'b1) begin
          has_ph_credit         = '1;
        end
        if (has_ph_credit) begin
          tlp_axis_tvalid = initial_axis_tvalid;
          skid_axis_tready = !prefix_r && tlp_axis_tready;
          if (initial_axis_tvalid && tlp_axis_tready) begin
            ph_credits_consumed_c = ph_credits_consumed_r + 1'b1;
            crc_in_c = crc_out_32;
            next_state = ST_TLP_STREAM;
          end
        end
      end
      ST_CHECK_CREDITS_PH_PD: begin
        static logic has_ph_credit;
        static logic has_pd_credit;
        static logic [15:0] data_length;
        static logic [15:0] data_credits_required;
        tlp_dw0 = skid_axis_tdata;
        has_ph_credit = '0;
        has_pd_credit = '0;
        data_length = {tlp_dw0.byte2.Length1, tlp_dw0.byte3.Length0};
        // Length is in DW and a data credit is 4 DW, rounded up; Length 0 is
        // 1024 DW, 256 credits (PCIe Base Spec r2.1, §2.2.1, §2.6.1).
        data_credits_required = data_length == '0 ? 16'd256 : (data_length + 16'd3) >> 2;
        tlp_axis_tdata = initial_axis_tdata;
        crc_tlp_axis_tdata = initial_crc_tdata;
        tlp_axis_tkeep = '1;
        if ((ph_credit_limit_r - ph_credits_consumed_r) >= 1'b1) begin
          has_ph_credit = '1;
        end
        // Evaluated in 16 bits, the width of data_credits_required, not
        // modulo 2^12: while pd_credit_limit_r has wrapped past 0 and
        // pd_credits_consumed_r has not, the difference exceeds any
        // data_credits_required.
        if ((pd_credit_limit_r - pd_credits_consumed_r) >= data_credits_required) begin
          has_pd_credit = '1;
        end
        if (has_ph_credit && has_pd_credit) begin
          tlp_axis_tvalid = initial_axis_tvalid;
          skid_axis_tready = !prefix_r && tlp_axis_tready;
          if (initial_axis_tvalid && tlp_axis_tready) begin
            pd_credits_consumed_c = pd_credits_consumed_r + data_credits_required;
            ph_credits_consumed_c = ph_credits_consumed_r + 1'b1;
            crc_in_c = crc_out_32;
            next_state = ST_TLP_STREAM;
          end
        end
      end
      ST_CHECK_CREDITS_CPLH: begin
        static logic has_cplh_credit;
        has_cplh_credit = '0;
        tlp_axis_tdata = initial_axis_tdata;
        crc_tlp_axis_tdata = initial_crc_tdata;
        tlp_axis_tkeep = '1;
        // A CPLH limit of 0 means infinite credits, and the gate always
        // passes (PCIe Base Spec r2.1, §2.6.1.1).
        if ((cplh_credit_limit_r == '0) ||
            ((cplh_credit_limit_r[7:0] - cplh_credits_consumed_r) >= 1'b1)) begin
          has_cplh_credit = '1;
        end
        if (has_cplh_credit) begin
          tlp_axis_tvalid = initial_axis_tvalid;
          skid_axis_tready = !prefix_r && tlp_axis_tready;
          if (initial_axis_tvalid && tlp_axis_tready) begin
            cplh_credits_consumed_c = cplh_credits_consumed_r + 1'b1;
            crc_in_c = crc_out_32;
            next_state = ST_TLP_STREAM;
          end
        end
      end
      ST_CHECK_CREDITS_CPLH_CPLD: begin
        static logic has_cplh_credit;
        static logic has_cpld_credit;
        static logic [15:0] data_length;
        static logic [15:0] data_credits_required;
        tlp_dw0 = skid_axis_tdata;
        has_cplh_credit = '0;
        has_cpld_credit = '0;
        data_length = {tlp_dw0.byte2.Length1, tlp_dw0.byte3.Length0};
        data_credits_required = data_length == '0 ? 16'd256 : (data_length + 16'd3) >> 2;
        tlp_axis_tdata = initial_axis_tdata;
        crc_tlp_axis_tdata = initial_crc_tdata;
        tlp_axis_tkeep = '1;
        // CPLD credits are counted as PD credits are in ST_CHECK_CREDITS_PH_PD.
        // A CPLH or CPLD limit of 0 means infinite credits.
        if ((cplh_credit_limit_r == '0) ||
            ((cplh_credit_limit_r[7:0] - cplh_credits_consumed_r) >= 1'b1)) begin
          has_cplh_credit = '1;
        end
        if ((cpld_credit_limit_r == '0) ||
            ((cpld_credit_limit_r - cpld_credits_consumed_r) >= data_credits_required)) begin
          has_cpld_credit = '1;
        end
        if (has_cplh_credit && has_cpld_credit) begin
          tlp_axis_tvalid = initial_axis_tvalid;
          skid_axis_tready = !prefix_r && tlp_axis_tready;
          if (initial_axis_tvalid && tlp_axis_tready) begin
            cplh_credits_consumed_c = cplh_credits_consumed_r + 1'b1;
            cpld_credits_consumed_c = cpld_credits_consumed_r + data_credits_required;
            crc_in_c = crc_out_32;
            next_state = ST_TLP_STREAM;
          end
        end
      end
      // One beat per input beat: the upper half of the previous DW and the
      // lower half of the current one.
      ST_TLP_STREAM: begin
        skid_axis_tready = tlp_axis_tready;
        if (tlp_axis_tready && skid_axis_tvalid) begin
          crc_in_c           = crc_out_32;
          tlp_axis_tdata     = {skid_axis_tdata[15:0], pipeline_axis_tdata[31:16]};
          crc_tlp_axis_tdata = tlp_axis_tdata;
          tlp_axis_tkeep     = '1;
          tlp_axis_tvalid    = skid_axis_tvalid;
          if (skid_axis_tlast) begin
            next_state = ST_TLP_CRC;
          end
        end
      end
      // No beat is sent: the LCRC advances over the TLP's last two bytes,
      // pipeline_axis_tdata[31:16] on tlp_axis_tdata[15:0], with the 16-bit
      // data step of pcie_lcrc16.
      ST_TLP_CRC: begin
        skid_axis_tready = '0;
        if (tlp_axis_tready) begin
          crc_in_c           = crc_out_16;
          tlp_axis_tdata     = {skid_axis_tdata[15:0], pipeline_axis_tdata[31:16]};
          crc_tlp_axis_tdata = tlp_axis_tdata;
          next_state         = ST_TLP_CRC_ALIGN;
        end
      end
      // The TLP's last two bytes and lcrc32d32[15:0].
      ST_TLP_CRC_ALIGN: begin
        skid_axis_tready = '0;
        if (tlp_axis_tready) begin
          tlp_axis_tkeep = '1;
          tlp_axis_tvalid = '1;
          tlp_axis_tdata = {lcrc32d32[15:0], pipeline_axis_tdata[31:16]};
          crc_tlp_axis_tdata = tlp_axis_tdata;
          next_state = ST_TLP_CRC_TLAST_ALIGN;
        end
      end
      // lcrc32d32[31:16] in the low two bytes, tkeep 0011b, tlast.
      ST_TLP_CRC_TLAST_ALIGN: begin
        skid_axis_tready = '0;
        if (tlp_axis_tready) begin
          tlp_axis_tkeep = 4'b0011;
          tlp_axis_tvalid = '1;
          tlp_axis_tlast = '1;
          tlp_axis_tdata = {lcrc32d32[31:16]};
          crc_tlp_axis_tdata = tlp_axis_tdata;
          next_state = ST_TLP_LAST;
        end
      end
      ST_TLP_LAST: begin
        crc_in_c            = '1;
        // Commit the completed TLP to retry management before advancing the
        // 12-bit sequence number, which wraps modulo 4096 (PCIe Base Spec
        // r2.1, §3.5.2.1).
        dllp_valid_o        = '1;
        next_transmit_seq_c = next_transmit_seq_r + 1'b1;
        prefix_c            = 1'b0;
        next_state          = ST_IDLE;
      end
      default: begin
      end
    endcase
  end

  // -------------------------------------------------------------------------
  // Credit limits
  // -------------------------------------------------------------------------
  // CREDIT_LIMIT per credit type, loaded from tx_fc_*_i while update_fc_i is
  // high; pcie_datalink_layer raises it for each received UpdateFC and once
  // when the peer's InitFC2 values are stored. A value is taken only if it is
  // not behind the current limit, where PCIe Base Spec r2.1, §2.6.1.1 takes
  // any value that differs. The limits reset to 0, so P and NP TLPs wait for
  // the first update, and Cpl TLPs, for which 0 means infinite, do not.
  always_comb begin : flow_contol
    ph_credit_limit_c   = ph_credit_limit_r;
    pd_credit_limit_c   = pd_credit_limit_r;
    nph_credit_limit_c  = nph_credit_limit_r;
    npd_credit_limit_c  = npd_credit_limit_r;
    cpld_credit_limit_c = cpld_credit_limit_r;
    cplh_credit_limit_c = cplh_credit_limit_r;


    if (update_fc_i) begin
      if (fc8_is_forward(tx_fc_ph_i, ph_credit_limit_r))
        ph_credit_limit_c = tx_fc_ph_i;
      if (fc8_is_forward(tx_fc_nph_i, nph_credit_limit_r))
        nph_credit_limit_c = tx_fc_nph_i;
      if (fc12_is_forward(tx_fc_pd_i, pd_credit_limit_r))
        pd_credit_limit_c = tx_fc_pd_i;
      if (fc12_is_forward(tx_fc_npd_i, npd_credit_limit_r))
        npd_credit_limit_c = tx_fc_npd_i;
      if (fc12_is_forward(tx_fc_cpld_i, cpld_credit_limit_r))
        cpld_credit_limit_c = tx_fc_cpld_i;
      if (fc8_is_forward(tx_fc_cplh_i, cplh_credit_limit_r[7:0]))
        cplh_credit_limit_c = tx_fc_cplh_i;
    end
  end : flow_contol


  // -------------------------------------------------------------------------
  // Stream registers
  // -------------------------------------------------------------------------
  // axis_input_skid_inst buffers s_axis; its output, skid_axis_*, is the
  // current input beat. axis_input_flow_inst holds the beat taken before it
  // (pipeline_axis_*). Both advance on skid_axis_tready, and the flow
  // register's own s_axis_tready is not used. axis_output_register_inst
  // registers the framed beats onto m_axis.
  axis_register #(
      .DATA_WIDTH(DATA_WIDTH),
      .KEEP_ENABLE('1),
      .KEEP_WIDTH(KEEP_WIDTH),
      .LAST_ENABLE('1),
      .ID_ENABLE('0),
      .ID_WIDTH(1),
      .DEST_ENABLE('0),
      .DEST_WIDTH(1),
      .USER_ENABLE('1),
      .USER_WIDTH(USER_WIDTH),
      .REG_TYPE(SkidBuffer)
  ) axis_input_skid_inst (
      .clk(clk_i),
      .rst(rst_i),
      .s_axis_tdata(s_axis_tdata),
      .s_axis_tkeep(s_axis_tkeep),
      .s_axis_tvalid(s_axis_tvalid),
      .s_axis_tready(s_axis_tready),
      .s_axis_tlast(s_axis_tlast),
      .s_axis_tuser(s_axis_tuser),
      .s_axis_tid('0),
      .s_axis_tdest('0),
      .m_axis_tdata(skid_axis_tdata),
      .m_axis_tkeep(skid_axis_tkeep),
      .m_axis_tvalid(skid_axis_tvalid),
      .m_axis_tready(skid_axis_tready),
      .m_axis_tlast(skid_axis_tlast),
      .m_axis_tuser(skid_axis_tuser),
      .m_axis_tid(),
      .m_axis_tdest()
  );



  axis_register #(
      .DATA_WIDTH(DATA_WIDTH),
      .KEEP_ENABLE('1),
      .KEEP_WIDTH(KEEP_WIDTH),
      .LAST_ENABLE('1),
      .ID_ENABLE('0),
      .ID_WIDTH(1),
      .DEST_ENABLE('0),
      .DEST_WIDTH(1),
      .USER_ENABLE('1),
      .USER_WIDTH(USER_WIDTH),
      .REG_TYPE(SkidBuffer)
  ) axis_input_flow_inst (
      .clk(clk_i),
      .rst(rst_i),
      .s_axis_tdata(skid_axis_tdata),
      .s_axis_tkeep(skid_axis_tkeep),
      .s_axis_tvalid(skid_axis_tvalid),
      .s_axis_tready(),
      .s_axis_tlast(skid_axis_tlast),
      .s_axis_tuser(skid_axis_tuser),
      .s_axis_tid('0),
      .s_axis_tdest('0),
      .m_axis_tdata(pipeline_axis_tdata),
      .m_axis_tkeep(pipeline_axis_tkeep),
      .m_axis_tvalid(pipeline_axis_tvalid),
      .m_axis_tready(skid_axis_tready),
      .m_axis_tlast(pipeline_axis_tlast),
      .m_axis_tuser(pipeline_axis_tuser),
      .m_axis_tid(),
      .m_axis_tdest()
  );



  axis_register #(
      .DATA_WIDTH(DATA_WIDTH),
      .KEEP_ENABLE('1),
      .KEEP_WIDTH(KEEP_WIDTH),
      .LAST_ENABLE('1),
      .ID_ENABLE('0),
      .ID_WIDTH(1),
      .DEST_ENABLE('0),
      .DEST_WIDTH(1),
      .USER_ENABLE('1),
      .USER_WIDTH(USER_WIDTH),
      .REG_TYPE(SkidBuffer)
  ) axis_output_register_inst (
      .clk(clk_i),
      .rst(rst_i),
      .s_axis_tdata(tlp_axis_tdata),
      .s_axis_tkeep(tlp_axis_tkeep),
      .s_axis_tvalid(tlp_axis_tvalid),
      .s_axis_tready(tlp_axis_tready),
      .s_axis_tlast(tlp_axis_tlast),
      .s_axis_tid('0),
      .s_axis_tdest('0),
      .s_axis_tuser(tlp_axis_tuser),
      .m_axis_tdata(m_axis_tdata),
      .m_axis_tkeep(m_axis_tkeep),
      .m_axis_tvalid(m_axis_tvalid),
      .m_axis_tready(m_axis_tready),
      .m_axis_tlast(m_axis_tlast),
      .m_axis_tid(),
      .m_axis_tdest(),
      .m_axis_tuser(m_axis_tuser)
  );


  // -------------------------------------------------------------------------
  // LCRC generators
  // -------------------------------------------------------------------------
  // Both advance the 32-bit LCRC from crc_in_r over tlp_axis_tdata.
  // pcie_lcrc32 takes a 32-bit step (polynomial 04C1 1DB7h, PCIe Base Spec
  // r2.1, §3.5.2.1). pcie_lcrc16, despite its name, advances the same 32-bit
  // CRC by a 16-bit step over data[15:0]; ST_TLP_CRC uses it for the TLP's
  // last two bytes.
  pcie_lcrc16 tlp_crc16_inst (
      .data  (tlp_axis_tdata),
      .crcIn (crc_in_r),
      .crcOut(crc_out_16)
  );

  pcie_lcrc32 pcie_lcrc32_inst (
      .crcIn (crc_in_r),
      .data  (tlp_axis_tdata),
      .crcOut(crc_out_32)
  );

  assign seq_num_o = next_transmit_seq_r;


endmodule
