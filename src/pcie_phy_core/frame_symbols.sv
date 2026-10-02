//!module: frame_symbols
//! Author: Idris Somoye
//! Module accepts DLLPs or TLPs from the dllp layer and adds framing symbols prior
//! to lane management.
// ---------------------------------------------------------------------------
// frame_symbols -- adds the framing Symbols to the DLL's TLP and DLLP stream
//
// Purpose
//   At the 8b/10b data rates, puts STP before a TLP or SDP before a DLLP in
//   byte 0 of the first beat, moves the packet up one byte, and puts END after
//   its last byte. The output tuser marks which bytes are K Symbols. At gen3
//   and above, ST_IDLE takes a separate path that builds 128b/130b framing
//   tokens. phy_transmit instantiates this module on the DLL side, clk_i.
//
// Interfaces
//   DLL in      s_axis_*: one TLP or DLLP per frame, tlast on its last beat.
//               tuser[0] = 1 marks a DLLP; at Gen3, tuser[1] marks a TLP.
//   Framed out  m_axis_*: through axis_output_register_inst. tuser holds one K
//               flag per byte, so USER_WIDTH must be at least KEEP_WIDTH.
//   Rate        curr_data_rate_i: read in ST_IDLE only.
//
// Clock and reset
//   clk_i only. rst_i is synchronous and active high. It resets curr_state
//   and the three AXI-Stream buffers; tlp_length_r, is_tlp_r and is_dllp_r
//   have no reset.
//
// Limitations
//   At the 8b/10b rates a frame needs at least two input beats: ST_IDLE does
//   not test s_axis_tlast. The Gen3 path is incomplete: two of its states are
//   never entered, ST_FRAME_LAST_DLLP sends no tlast, the FIFO arm of the
//   input mux is never selected, and tlp_length_r is never cleared. At Gen3,
//   ST_IDLE also sends an all-zero beat on every clock that a non-DLLP input
//   beat is offered.
//
// References
//   PCIe Base Spec r2.1, §4.2.2
//   PCIe Base Spec r2.1, Table 4-1
// ---------------------------------------------------------------------------
module frame_symbols
  import pcie_phy_pkg::*;
#(
    parameter int USER_WIDTH       = 1,
    parameter int DATA_WIDTH       = 32,                  // Width of AXI stream interfaces in bits
    parameter int KEEP_WIDTH       = ((DATA_WIDTH) / 8),  // tkeep width: one bit per byte
    parameter int MAX_PAYLOAD_SIZE = 256,
    parameter int RX_FIFO_SIZE     = 2

) (
    input  logic                         clk_i,
    input  logic                         rst_i,
    input  rate_speed_e                  curr_data_rate_i,
    // ---- DLL stream in: TLPs and DLLPs ----
    input  logic        [DATA_WIDTH-1:0] s_axis_tdata,
    input  logic        [KEEP_WIDTH-1:0] s_axis_tkeep,
    input  logic                         s_axis_tvalid,
    input  logic                         s_axis_tlast,
    input  logic        [USER_WIDTH-1:0] s_axis_tuser,
    output logic                         s_axis_tready,
    // ---- framed stream out ----
    output logic        [DATA_WIDTH-1:0] m_axis_tdata,
    output logic        [KEEP_WIDTH-1:0] m_axis_tkeep,
    output logic                         m_axis_tvalid,
    output logic                         m_axis_tlast,
    output logic        [USER_WIDTH-1:0] m_axis_tuser,
    input  logic                         m_axis_tready

);

  // Elaboration check. tuser carries one K flag per byte of the data word, so
  // it needs KEEP_WIDTH bits; END in byte 3, for example, is marked 4'b1000. A
  // narrower USER_WIDTH truncates the mask, and a K Symbol in a high byte is
  // then sent as data. Lint does not report the truncation, because
  // lint/waiver.vlt waives truncating assignments in this file.
  if (USER_WIDTH < KEEP_WIDTH)
    $fatal(1,
           "frame_symbols: USER_WIDTH=%0d is narrower than KEEP_WIDTH=%0d. tuser carries a one-bit-per-byte K-position mask, so a narrower tuser silently drops the K flag of any Symbol in a high byte -- ENDP at byte 3 first. See §63 #7d.",
           USER_WIDTH, KEEP_WIDTH);


  // -------------------------------------------------------------------------
  // Framing state machine
  // -------------------------------------------------------------------------
  // At the 8b/10b rates an ST_FRAME_STREAM beat is the top byte of the
  // previous input beat, from axis_buffer_register_inst, then input bytes 0-2.
  // Every state waits for phy_axis_tready (ST_FRAME_GEN_3_TLP: fifo_ready).
  //   state                      action                          exit
  //   ST_IDLE                    STP/SDP + input bytes 0-2;      first beat accepted
  //                              Gen3: SDP token or FIFO write
  //   ST_FRAME_STREAM            carried byte + bytes 0-2        tlast: END fits here
  //                                                              -> ST_IDLE, else LAST
  //   ST_FRAME_LAST              carried byte, if any, + END     -> ST_IDLE
  //   ST_FRAME_GEN_3_DLLP        Gen3 DLLP, two-byte shift       tlast
  //   ST_FRAME_GEN_3_TLP         Gen3 TLP into the FIFO          tlast
  //   ST_FRAME_GEN_3_TLP_SDS     Gen3 STP token                  token sent
  //   ST_FRAME_GEN_3_STREAM      Gen3 TLP out of the FIFO        FIFO tlast
  //   ST_FRAME_LAST_DLLP         Gen3 tail, without tlast        -> ST_IDLE
  //   ST_FRAME_LAST_TLP          never entered
  //   ST_FRAME_LAST_DLLP_ALLIGN  never entered
  typedef enum logic [3:0] {
    ST_IDLE,
    ST_FRAME_STREAM,
    ST_FRAME_GEN_3_TLP,
    ST_FRAME_GEN_3_DLLP,
    ST_FRAME_GEN_3_TLP_SDS,
    ST_FRAME_GEN_3_STREAM,
    ST_FRAME_LAST,
    ST_FRAME_LAST_TLP,
    ST_FRAME_LAST_DLLP,
    ST_FRAME_LAST_DLLP_ALLIGN
  } frame_st_e;

  frame_st_e                  curr_state;
  frame_st_e                  next_state;
  // Input of axis_output_register_inst.
  logic      [DATA_WIDTH-1:0] phy_axis_tdata;
  logic      [KEEP_WIDTH-1:0] phy_axis_tkeep;
  logic                       phy_axis_tvalid;
  logic                       phy_axis_tlast;
  logic      [USER_WIDTH-1:0] phy_axis_tuser;
  logic                       phy_axis_tready;

  // Output of axis_buffer_register_inst: the bytes carried into the next beat.
  logic      [DATA_WIDTH-1:0] buffer_axis_tdata;
  logic      [KEEP_WIDTH-1:0] buffer_axis_tkeep;
  logic                       buffer_axis_tvalid;
  logic                       buffer_axis_tlast;
  logic      [USER_WIDTH-1:0] buffer_axis_tuser;
  logic                       buffer_axis_tready;

  // Output of dllp2tlp_fifo_inst (Gen3 TLPs).
  logic      [DATA_WIDTH-1:0] fifo_axis_tdata;
  logic      [KEEP_WIDTH-1:0] fifo_axis_tkeep;
  logic                       fifo_axis_tvalid;
  logic                       fifo_axis_tlast;
  logic      [USER_WIDTH-1:0] fifo_axis_tuser;
  logic                       fifo_axis_tready;

  // Input of axis_buffer_register_inst.
  logic      [DATA_WIDTH-1:0] mux_axis_tdata;
  logic      [KEEP_WIDTH-1:0] mux_axis_tkeep;
  logic                       mux_axis_tvalid;
  logic                       mux_axis_tlast;
  logic      [USER_WIDTH-1:0] mux_axis_tuser;
  logic                       mux_axis_tready;

  logic      [          15:0] tlp_length_c;
  logic      [          15:0] tlp_length_r;
  logic      [           3:0] fcrc;
  logic                       fp;
  logic                       fifo_ready;
  logic                       fifo_valid;
  logic                       is_tlp_c;
  logic                       is_tlp_r;
  logic                       is_dllp_c;
  logic                       is_dllp_r;
  logic                       mux_axis_buffer;

  always @(posedge clk_i) begin
    if (rst_i) begin
      curr_state <= ST_IDLE;
    end else begin
      curr_state <= next_state;
    end
    tlp_length_r <= tlp_length_c;
    is_tlp_r     <= is_tlp_c;
    is_dllp_r    <= is_dllp_c;
  end


  always_comb begin : main_seq
    next_state      = curr_state;
    phy_axis_tdata  = '0;
    phy_axis_tkeep  = '0;
    phy_axis_tvalid = '0;
    phy_axis_tlast  = '0;
    phy_axis_tuser  = '0;
    s_axis_tready   = '0;
    fifo_valid      = '0;
    mux_axis_buffer = '0;
    // Defaulted so that every path assigns it; set only in the Gen3 states, it
    // would infer a latch. Its only reader is dllp2tlp_fifo_inst, which is
    // written only on the Gen3 TLP path (fifo_valid), so the default has no
    // effect at the 8b/10b rates.
    fifo_axis_tready = '0;
    is_tlp_c        = is_tlp_r;
    is_dllp_c       = is_dllp_r;
    tlp_length_c    = tlp_length_r;

    // This test reads the '0 assigned above. ST_FRAME_GEN_3_TLP_SDS sets
    // mux_axis_buffer only later in this block, so the FIFO arm is never taken.
    if (!mux_axis_buffer) begin
      mux_axis_tdata  = s_axis_tdata;
      mux_axis_tkeep  = s_axis_tkeep;
      mux_axis_tvalid = s_axis_tvalid;
      mux_axis_tlast  = s_axis_tlast;
      mux_axis_tuser  = s_axis_tuser;
    end else begin
      mux_axis_tdata  = fifo_axis_tdata;
      mux_axis_tkeep  = fifo_axis_tkeep;
      mux_axis_tvalid = fifo_axis_tvalid;
      mux_axis_tlast  = fifo_axis_tlast;
      mux_axis_tuser  = fifo_axis_tuser;
    end
    case (curr_state)
      // The framing Symbol takes byte 0, so input bytes 0-2 move up one byte
      // and byte 3 is carried into the next beat; at Gen3 the two-byte token
      // moves the packet up two bytes. phy_axis_tuser 4'b0001 marks byte 0 as K.
      ST_IDLE: begin
        if (phy_axis_tready && s_axis_tvalid) begin
          phy_axis_tvalid = '1;
          phy_axis_tkeep  = '1;
          if (curr_data_rate_i < gen3) begin
            s_axis_tready = '1;
            phy_axis_tdata = {s_axis_tdata[23:0], s_axis_tuser[0] ? SDP : STP};
            phy_axis_tuser = 4'b0001;
            next_state = ST_FRAME_STREAM;
          end else begin
            if (s_axis_tuser[0]) begin
              is_dllp_c      = '1;
              is_tlp_c       = '0;
              s_axis_tready  = '1;
              phy_axis_tdata = {s_axis_tdata[15:0], GEN3_SDP[15:0]};
              phy_axis_tuser = 4'b0011;
              next_state     = ST_FRAME_GEN_3_DLLP;
            end else if (s_axis_tuser[1] && fifo_ready) begin
              is_dllp_c     = '0;
              is_tlp_c      = '1;
              fifo_valid    = '1;
              s_axis_tready = '1;
              next_state    = ST_FRAME_GEN_3_TLP;
            end
          end
        end
      end
      ST_FRAME_STREAM: begin
        s_axis_tready   = phy_axis_tready;
        if (s_axis_tready && s_axis_tvalid) begin
          s_axis_tready   = '1;
          phy_axis_tvalid = '1;
          phy_axis_tdata  = {s_axis_tdata[23:0], buffer_axis_tdata[31:24]};
          phy_axis_tkeep  = {s_axis_tkeep[2:0], buffer_axis_tkeep[3]};
          if (s_axis_tlast) begin
            next_state = ST_IDLE;
            // tlast is set per arm, not here: in the default arm the frame
            // still owes ST_FRAME_LAST's beat.
            case (s_axis_tkeep)
              // One byte left: END goes in byte 2.
              4'b0001: begin
                phy_axis_tdata[23:16] = ENDP;
                phy_axis_tkeep[2]     = '1;
                phy_axis_tuser        = 4'b0100;
                phy_axis_tlast        = '1;
              end
              // Two bytes left: END goes in byte 3.
              4'b0011: begin
                phy_axis_tdata[31:24] = ENDP;
                phy_axis_tuser        = 4'b1000;
                phy_axis_tlast        = '1;
                phy_axis_tkeep[3]     = '1;
              end
              // Three or four bytes left: END needs another beat.
              default: begin
                next_state = ST_FRAME_LAST;
              end
            endcase
          end
        end
      end
      ST_FRAME_GEN_3_DLLP: begin
        if (phy_axis_tready && s_axis_tvalid) begin
          s_axis_tready   = '1;
          phy_axis_tvalid = '1;
          phy_axis_tdata  = {s_axis_tdata[15:0], buffer_axis_tdata[31:16]};
          phy_axis_tkeep  = {s_axis_tkeep[1:0], buffer_axis_tkeep[3:2]};
          if (s_axis_tlast) begin
            phy_axis_tlast = '1;
            next_state = ST_IDLE;
            case (s_axis_tkeep)
              4'b0001, 4'b0011: begin
              end
              default: begin
                phy_axis_tlast = '0;
                next_state = ST_FRAME_LAST_DLLP;
              end
            endcase
          end
        end
      end
      ST_FRAME_GEN_3_TLP: begin
        s_axis_tready = fifo_ready;
        fifo_valid    = '1;
        if (fifo_ready && s_axis_tvalid) begin
          tlp_length_c = tlp_length_r + 1'b1;
          if (s_axis_tlast) begin
            next_state = ST_FRAME_GEN_3_TLP_SDS;
            if (s_axis_tkeep != 4'b0011) begin
              // Any tail other than two bytes returns to ST_IDLE instead.
              next_state = ST_IDLE;
            end
          end
        end
      end
      ST_FRAME_GEN_3_TLP_SDS: begin
        fifo_axis_tready = phy_axis_tready;
        mux_axis_buffer  = '1;
        if (phy_axis_tready && fifo_axis_tvalid) begin
          phy_axis_tvalid = '1;
          phy_axis_tuser  = '1;
          gen_fcrc_parity(fcrc, fp, tlp_length_r);
          gen_stp_gen3(phy_axis_tdata, fp, fcrc, tlp_length_r, {
                       fifo_axis_tdata[15:8], fifo_axis_tdata[3:0]});
          next_state = ST_FRAME_GEN_3_STREAM;
        end
      end
      ST_FRAME_GEN_3_STREAM: begin
        fifo_axis_tready = phy_axis_tready;
        if (phy_axis_tready && fifo_axis_tvalid) begin
          phy_axis_tvalid = '1;
          phy_axis_tdata  = {fifo_axis_tdata[15:0], buffer_axis_tdata[31:16]};
          phy_axis_tkeep  = {fifo_axis_tdata[1:0], buffer_axis_tkeep[3:2]};
          if (fifo_axis_tlast) begin
            next_state = ST_FRAME_LAST_DLLP;
            case (fifo_axis_tkeep)
              4'b0001, 4'b0011: begin
              end
              default: begin
                next_state = ST_FRAME_LAST_DLLP;
              end
            endcase
          end
        end
      end
      ST_FRAME_LAST: begin
        if (phy_axis_tready) begin
          next_state = ST_IDLE;
          phy_axis_tvalid = '1;
          // Every beat from this state is the frame's last, since next_state is
          // ST_IDLE unconditionally, so tlast is set before the case and covers
          // the default arm too. tlast is the frame boundary downstream:
          // phy_transmit's dllp_axis_async_fifo_inst carries it (LAST_ENABLE =
          // 1), and lane_management leaves ST_LANE_MNGT_TX_DATA on it. With
          // contiguous tkeep only the 0111b and 1111b tails reach this state.
          phy_axis_tlast  = '1;
          case (buffer_axis_tkeep)
            // Nothing carried: END alone, in byte 0.
            4'b0111: begin
              phy_axis_tuser      = 4'b0001;
              phy_axis_tdata[7:0] = ENDP;
              phy_axis_tkeep      = 4'b0001;
            end
            // The carried byte, then END in byte 1.
            4'b1111: begin
              phy_axis_tuser = 4'b0010;
              phy_axis_tdata = {ENDP, buffer_axis_tdata[31:24]};
              phy_axis_tkeep = 4'b0011;
            end
            default: begin
            end
          endcase
        end
      end
      ST_FRAME_LAST_DLLP: begin  // Gen3 only: from ST_FRAME_GEN_3_DLLP or ST_FRAME_GEN_3_STREAM
        if (phy_axis_tready) begin
          next_state = ST_IDLE;
          phy_axis_tvalid = '1;
          case (buffer_axis_tkeep)
            4'b0111: begin
              phy_axis_tdata[15:0] = buffer_axis_tdata[23:16];
              phy_axis_tkeep[1:0]  = buffer_axis_tkeep[3];
            end
            4'b1111: begin
              phy_axis_tdata[15:0] = buffer_axis_tdata[31:16];
              phy_axis_tkeep       = buffer_axis_tkeep[3:2];
            end
            default: begin
            end
          endcase
        end
      end
      ST_FRAME_LAST_DLLP_ALLIGN: begin  // never entered: no transition leads here
        if (phy_axis_tready) begin
          next_state      = ST_IDLE;
          phy_axis_tvalid = '1;
          phy_axis_tlast  = '1;
          case (buffer_axis_tkeep)
            4'b0111: begin
              phy_axis_tuser = 4'b0001;
              phy_axis_tdata = GEN3_EDS[31:24];
              phy_axis_tkeep = 4'b0001;
            end
            4'b1111: begin
              phy_axis_tuser    = 4'b0011;
              phy_axis_tdata    = GEN3_EDS[31:16];
              phy_axis_tkeep[0] = 4'b0011;
            end
            default: begin
            end
          endcase
        end
      end
      default: begin
      end
    endcase
  end

  // Keeps the latest input beat, whose top bytes the next output beat
  // carries. Its m_axis_tready is phy_axis_tready && mux_axis_tvalid, so it
  // moves on with the input stream; its s_axis_tready is not used.
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
  ) axis_buffer_register_inst (
      .clk(clk_i),
      .rst(rst_i),
      .s_axis_tdata(mux_axis_tdata),
      .s_axis_tkeep(mux_axis_tkeep),
      .s_axis_tvalid(mux_axis_tvalid),
      .s_axis_tready(),
      .s_axis_tlast(mux_axis_tlast),
      .s_axis_tid('0),
      .s_axis_tdest('0),
      .s_axis_tuser(mux_axis_tuser),
      .m_axis_tdata(buffer_axis_tdata),
      .m_axis_tkeep(buffer_axis_tkeep),
      .m_axis_tvalid(buffer_axis_tvalid),
      .m_axis_tready(phy_axis_tready && mux_axis_tvalid),
      .m_axis_tlast(buffer_axis_tlast),
      .m_axis_tid(),
      .m_axis_tdest(),
      .m_axis_tuser(buffer_axis_tuser)
  );


  // Output skid buffer; its s_axis_tready is phy_axis_tready.
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
      .s_axis_tdata(phy_axis_tdata),
      .s_axis_tkeep(phy_axis_tkeep),
      .s_axis_tvalid(phy_axis_tvalid),
      .s_axis_tready(phy_axis_tready),
      .s_axis_tlast(phy_axis_tlast),
      .s_axis_tid('0),
      .s_axis_tdest('0),
      .s_axis_tuser(phy_axis_tuser),
      .m_axis_tdata(m_axis_tdata),
      .m_axis_tkeep(m_axis_tkeep),
      .m_axis_tvalid(m_axis_tvalid),
      .m_axis_tready(m_axis_tready),
      .m_axis_tlast(m_axis_tlast),
      .m_axis_tid(),
      .m_axis_tdest(),
      .m_axis_tuser(m_axis_tuser)
  );


  // Gen3 only: holds a TLP while ST_FRAME_GEN_3_TLP adds one to tlp_length_r
  // per beat it accepts, for the STP token; nothing clears tlp_length_r.
  // Written only while fifo_valid is set: in ST_IDLE's Gen3 TLP arm and in
  // ST_FRAME_GEN_3_TLP.
  axis_fifo #(
      .DEPTH(RX_FIFO_SIZE * MAX_PAYLOAD_SIZE),
      .DATA_WIDTH(DATA_WIDTH),
      .KEEP_ENABLE(KEEP_WIDTH > 0),
      .KEEP_WIDTH(KEEP_WIDTH),
      .LAST_ENABLE(1),
      .ID_ENABLE(0),
      .DEST_ENABLE(0),
      .USER_ENABLE('1),
      .USER_WIDTH(USER_WIDTH),
      .FRAME_FIFO(1),
      .USER_BAD_FRAME_VALUE('1),
      .USER_BAD_FRAME_MASK('1),
      .DROP_BAD_FRAME(1),
      .DROP_WHEN_FULL(0)
  ) dllp2tlp_fifo_inst (
      .clk(clk_i),
      .rst(rst_i),
      // AXI input
      .s_axis_tdata(s_axis_tdata),
      .s_axis_tkeep(s_axis_tkeep),
      .s_axis_tvalid(s_axis_tvalid && fifo_valid),
      .s_axis_tready(fifo_ready),
      .s_axis_tlast(s_axis_tlast),
      .s_axis_tuser(s_axis_tuser),
      .s_axis_tid(),
      .s_axis_tdest(),
      // AXI output
      .m_axis_tdata(fifo_axis_tdata),
      .m_axis_tkeep(fifo_axis_tkeep),
      .m_axis_tvalid(fifo_axis_tvalid),
      .m_axis_tready(fifo_axis_tready),
      .m_axis_tlast(fifo_axis_tlast),
      .m_axis_tuser(fifo_axis_tuser),
      .m_axis_tid(),
      .m_axis_tdest(),
      .pause_ack(),
      .pause_req(),
      .status_depth(),
      .status_depth_commit(),
      // Status
      .status_overflow(),
      .status_bad_frame(),
      .status_good_frame()
  );

endmodule
