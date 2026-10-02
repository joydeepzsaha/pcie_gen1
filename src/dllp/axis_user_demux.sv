// ---------------------------------------------------------------------------
//! @title axis_user_demux
//! @author Idris Somoye
//! Splits the receive stream into a TLP output and a DLLP output by tuser.
//
// Purpose
//   Routes each frame from the Physical Layer receive stream to one of two
//   outputs, chosen by the tuser bits of its first beat: TLP frames to
//   m_tlp_axis_* (dllp2tlp) and DLLP frames to m_dllp_axis_* (dllp_handler).
//   The choice holds until the beat with tlast. Each output has its own skid
//   buffer. first_tlp_valid_o reports that a TLP frame has arrived.
//
// Interfaces
//   Input        s_axis_*: the receive stream from the Physical Layer.
//                s_axis_tuser bit 1 (UserIsTlp) marks a TLP frame and bit 0
//                (UserIsDllp) a DLLP frame; data_handler sets them.
//   TLP output   m_tlp_axis_*: TLP frames, unchanged.
//   DLLP output  m_dllp_axis_*: DLLP frames, unchanged.
//   Status       first_tlp_valid_o: high from the cycle after the first
//                entry to ST_TLP until reset; pcie_flow_ctrl_init reads it
//                as a received TLP.
//   Unused       link_status_i, and the parameters MAX_PAYLOAD_SIZE and
//                RX_FIFO_SIZE.
//
// Clock and reset
//   clk_i only. rst_i is synchronous and active high.
//
// Limitations
//   A first beat with neither tuser bit set is never accepted, and the input
//   stalls. ST_IDLE leaves on the first valid beat whether or not it is
//   accepted or carries tlast, so a one-beat frame accepted in ST_IDLE also
//   sends the next frame to the same output. In ST_IDLE the input handshake
//   uses the output's ready (m_tlp_axis_tready or m_dllp_axis_tready), but the
//   skid buffer takes the first beat on its own registered ready (tlp_ready or
//   dllp_ready). Under back-pressure the two can differ: the beat is then
//   passed on twice (skid buffer ready, output not) or lost (the reverse).
//   ST_STREAM is declared and never entered. USER_WIDTH must be at least 2
//   for tuser bit 1; its default is 1.
// ---------------------------------------------------------------------------
module axis_user_demux
  import pcie_datalink_pkg::*;
#(
    parameter int DATA_WIDTH = 32,
    parameter int STRB_WIDTH = DATA_WIDTH / 8,
    parameter int KEEP_WIDTH = STRB_WIDTH,
    parameter int USER_WIDTH = 1,
    parameter int MAX_PAYLOAD_SIZE = 256,
    parameter int RX_FIFO_SIZE = 2
) (
    input  logic                               clk_i,
    input  logic                               rst_i,
    input  pcie_dl_status_e                    link_status_i,
    output logic                               first_tlp_valid_o,

    // ---- receive stream ----------------------------------------------------
    input  logic            [  DATA_WIDTH-1:0] s_axis_tdata,
    input  logic            [  KEEP_WIDTH-1:0] s_axis_tkeep,
    input  logic                               s_axis_tvalid,
    input  logic                               s_axis_tlast,
    input  logic            [  USER_WIDTH-1:0] s_axis_tuser,
    output logic                               s_axis_tready,

    // ---- TLP output --------------------------------------------------------
    output logic            [(DATA_WIDTH)-1:0] m_tlp_axis_tdata,
    output logic            [(KEEP_WIDTH)-1:0] m_tlp_axis_tkeep,
    output logic                               m_tlp_axis_tvalid,
    output logic                               m_tlp_axis_tlast,
    output logic            [(USER_WIDTH)-1:0] m_tlp_axis_tuser,
    input  logic                               m_tlp_axis_tready,

    // ---- DLLP output -------------------------------------------------------
    output logic [(DATA_WIDTH)-1:0] m_dllp_axis_tdata,
    output logic [(KEEP_WIDTH)-1:0] m_dllp_axis_tkeep,
    output logic                    m_dllp_axis_tvalid,
    output logic                    m_dllp_axis_tlast,
    output logic [(USER_WIDTH)-1:0] m_dllp_axis_tuser,
    input  logic                    m_dllp_axis_tready
);

  localparam int UserIsTlp = 1;
  localparam int UserIsDllp = 0;

  // ST_IDLE picks the output from the first beat; ST_TLP and ST_DLLP pass the
  // rest of the frame through to tlast.
  typedef enum logic [2:0] {
    ST_IDLE,
    ST_STREAM,
    ST_DLLP,
    ST_TLP
  } demux_st_e;

  demux_st_e curr_state;
  demux_st_e next_state;


  logic dllp_valid;
  logic tlp_valid;
  logic first_tlp_valid_c;
  logic first_tlp_valid_r;


  logic tlp_ready;
  logic dllp_ready;

  always @(posedge clk_i) begin : main_seq
    if (rst_i) begin
      curr_state <= ST_IDLE;
      first_tlp_valid_r <= '0;
    end else begin
      curr_state <= next_state;
      first_tlp_valid_r <= first_tlp_valid_c;
    end
  end

  always_comb begin : main_combo
    next_state = curr_state;
    dllp_valid = '0;
    tlp_valid = '0;
    s_axis_tready = '0;
    first_tlp_valid_c = first_tlp_valid_r;
    case (curr_state)
      ST_IDLE: begin
        if (s_axis_tvalid) begin
          if (s_axis_tuser[UserIsTlp]) begin
            s_axis_tready = m_tlp_axis_tready;
            tlp_valid = s_axis_tvalid;
            next_state = ST_TLP;
          end else if (s_axis_tuser[UserIsDllp]) begin
            s_axis_tready = m_dllp_axis_tready;
            dllp_valid = s_axis_tvalid;
            next_state = ST_DLLP;
          end
        end
      end
      ST_TLP: begin
        s_axis_tready = tlp_ready;
        tlp_valid = s_axis_tvalid;
        first_tlp_valid_c = '1;
        if (s_axis_tvalid && tlp_ready && s_axis_tlast) begin
          next_state = ST_IDLE;
        end
      end
      ST_DLLP: begin
        s_axis_tready = dllp_ready;
        dllp_valid = s_axis_tvalid;
        if (s_axis_tvalid && dllp_ready && s_axis_tlast) begin
          next_state = ST_IDLE;
        end
      end
      default: begin
      end
    endcase
  end

  assign first_tlp_valid_o = first_tlp_valid_r;



  // DLLP output skid buffer
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
  ) dllp_axis_register_inst (
      .clk(clk_i),
      .rst(rst_i),
      .s_axis_tdata(s_axis_tdata),
      .s_axis_tkeep(s_axis_tkeep),
      .s_axis_tvalid(dllp_valid),
      .s_axis_tready(dllp_ready),
      .s_axis_tlast(s_axis_tlast),
      .s_axis_tid('0),
      .s_axis_tdest('0),
      .s_axis_tuser(s_axis_tuser),
      .m_axis_tdata(m_dllp_axis_tdata),
      .m_axis_tkeep(m_dllp_axis_tkeep),
      .m_axis_tvalid(m_dllp_axis_tvalid),
      .m_axis_tready(m_dllp_axis_tready),
      .m_axis_tlast(m_dllp_axis_tlast),
      .m_axis_tid(),
      .m_axis_tdest(),
      .m_axis_tuser(m_dllp_axis_tuser)
  );


  // TLP output skid buffer
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
  ) tlp_axis_register_inst (
      .clk(clk_i),
      .rst(rst_i),
      .s_axis_tdata(s_axis_tdata),
      .s_axis_tkeep(s_axis_tkeep),
      .s_axis_tvalid(tlp_valid),
      .s_axis_tready(tlp_ready),
      .s_axis_tlast(s_axis_tlast),
      .s_axis_tid('0),
      .s_axis_tdest('0),
      .s_axis_tuser(s_axis_tuser),
      .m_axis_tdata(m_tlp_axis_tdata),
      .m_axis_tkeep(m_tlp_axis_tkeep),
      .m_axis_tvalid(m_tlp_axis_tvalid),
      .m_axis_tready(m_tlp_axis_tready),
      .m_axis_tlast(m_tlp_axis_tlast),
      .m_axis_tid(),
      .m_axis_tdest(),
      .m_axis_tuser(m_tlp_axis_tuser)
  );

endmodule
