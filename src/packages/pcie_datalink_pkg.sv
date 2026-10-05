// ---------------------------------------------------------------------------
// pcie_datalink_pkg -- Data Link Layer encodings, DLLP layouts, REPLAY_TIMER
//
// Original author: Idris Somoye
// Modified by: Kourosh Ghahramani
// Silicon Systems Research Lab, University of Washington
//
// Purpose
//   Shared definitions for the Data Link Layer in src/dllp and for the
//   Transaction Layer, converter and configuration files that import it: DLLP
//   and TLP encodings, the packed layouts used to build and parse DLLPs and
//   TLP DW0, and the REPLAY_TIMER limit.
//
// Contents
//   Constants     SkidBuffer, the axis_register REG_TYPE of a skid buffer.
//                 HdrMinCredits and PdMinCredits, the header and data credits
//                 pcie_flow_ctrl_init advertises for P and NP. The other
//                 parameters are not used.
//   Link status   pcie_dl_status_e.
//   Encodings     dllp_type_e (DLLP Type), pcie_tlp_fmt_e (Fmt),
//                 pcie_tlp_type_e (Fmt and Type).
//   Layouts       pcie_tlp_header_dw0_t and the DLLP structs and unions.
//                 Byte k of a packet is bits [8k+7:8k], as on the 32-bit
//                 AXI-Stream beats; a DLLP's CRC is bits [47:32].
//   Functions     DLLP field packing and unpacking; the REPLAY_TIMER limit
//                 (replay_limit_symbol_times, replay_timer_cycles).
//
// References
//   PCIe Base Spec r2.1, §2.2.1
//   PCIe Base Spec r2.1, §2.6.1
//   PCIe Base Spec r2.1, §3.2
//   PCIe Base Spec r2.1, §3.4.1
//   PCIe Base Spec r2.1, §3.5.2.1
//   PCIe Base Spec r2.1, §7.8.4
// ---------------------------------------------------------------------------
package pcie_datalink_pkg;

  /* verilator lint_off WIDTHEXPAND */
  // HdrFc through FcClpData are not referenced by any module. The
  // REPLAY_TIMER limit comes from replay_timer_cycles below, and the
  // REPLAY_NUM limit from MAX_REPLAY_ATTEMPTS in pcie_datalink_layer.
  parameter byte HdrFc = 8'hFA  * 8;
  parameter byte DataFc = 8'hDA * 8;
  parameter byte FcPHdr = 8'h01 * 8;
  parameter byte FcNpHdr = 8'h01 * 8;
  parameter byte FcCplHdr = 8'h01 * 8;
  parameter int FcPData = 32'h040 * 8;
  parameter int FcNpData = 32'h010 * 8;
  parameter int DllpHdrByteSize = 32'h2;
  parameter int ReplayTimer = 32'd999;
  parameter int ReplayNum = 32'd2;
  parameter int LtssmDetect = 32'd1500;
  parameter int FcClpData = FcPData / FcPHdr;
  // axis_register's REG_TYPE for a skid buffer.
  parameter int SkidBuffer = 2;
  // Credits advertised for P and NP: 16 headers and 40h data credits, 1024
  // bytes at 4 DW per credit (PCIe Base Spec r2.1, §2.6.1). dllp2tlp and
  // dllp_fc_update load the same two constants at reset.
  parameter int HdrMinCredits = 8'h10;
  parameter int PdMinCredits = 8'h40;



  // AXI response codes; not used by any module.
  typedef enum logic [1:0] {
    OKAY = 2'b00,
    EXOKAY = 2'b01,
    SLVERR = 2'b10,
    DECERROR = 2'b11,
    RESP_X = 'X
  } resp_e;


  // DL_DOWN and DL_UP are the specification's status outputs; DL_ACTIVE
  // marks the DL_Active state, in which DL_Up is reported (PCIe Base Spec
  // r2.1, §3.2).
  typedef enum logic [1:0] {
    DL_DOWN,
    DL_UP,
    DL_ACTIVE
  } pcie_dl_status_e;


  // DLLP Type encodings (PCIe Base Spec r2.1, §3.4.1, Table 3-1). The flow
  // control types carry VC ID 0 in bits [2:0].
  typedef enum logic [7:0] {
    Ack               = 8'b00000000,
    // Reserved in Table 3-1. dllp_handler decodes it, and in ST_IDLE
    // pcie_flow_ctrl_init starts FC_INIT1 on it.
    Feature_Exchange  = 8'b00000010,
    Nak               = 8'b00010000,
    PM_Enter_L1       = 8'b00100000,
    PM_Enter_L23      = 8'b00100001,
    PM_Actv_St_Req_L1 = 8'b00100011,
    PM_Request_Ack    = 8'b00100100,
    Vendor_Specific   = 8'b00110000,
    InitFC1_P         = 8'b01000000,
    InitFC1_NP        = 8'b01010000,
    InitFC1_Cpl       = 8'b01100000,
    InitFC2_P         = 8'b11000000,
    InitFC2_NP        = 8'b11010000,
    InitFC2_Cpl       = 8'b11100000,
    UpdateFC_P        = 8'b10000000,
    UpdateFC_NP       = 8'b10010000,
    UpdateFC_Cpl      = 8'b10100000
  } dllp_type_e;

  // TLP Fmt field (PCIe Base Spec r2.1, §2.2.1, Table 2-2).
  typedef enum logic [2:0] {
    TLP_3DW_ND = 3'b000,
    TLP_4DW_ND = 3'b001,
    TLP_3DW_WD = 3'b010,
    TLP_4DW_WD = 3'b011,
    TLP_PREFIX = 3'b100
  } pcie_tlp_fmt_e;

  // Fmt and Type, byte 0 of TLP DW0 (PCIe Base Spec r2.1, §2.2.1,
  // Table 2-3). A ? bit is Fmt[0] (3 DW or 4 DW header) or a Type sub-field.
  typedef enum logic [7:0] {
    MRd      = 8'b00?0_0000,  //Memory Read Request
    MRdLk    = 8'b00?0_0001,  //Memory Read Request-Locked
    MWr      = 8'b01?0_0000,  //Memory Write Request
    IORd     = 8'b0000_0010,  //I/O Read Request
    IOWr     = 8'b0100_0010,  //I/O Write Request
    CfgRd0   = 8'b0000_0100,  //Configuration Read Type 0
    CfgWr0   = 8'b0100_0100,  //Configuration Write Type 0
    CfgRd1   = 8'b0000_0101,  //Configuration Read Type 1
    CfgWr1   = 8'b0100_0101,  //Configuration Write Type 1
    TCfgRd   = 8'b0001_1011,  //Deprecated TLP Type
    TCfgWr   = 8'b0101_1011,  //Deprecated TLP Type
    Msg      = 8'b0011_0???,  //Message Request
    MsgD     = 8'b0111_0???,  //Message Request with data payload
    Cpl      = 8'b0000_1010,  //Completion without Data
    CplD     = 8'b0100_1010,  //Completion with Data
    CplLk    = 8'b0000_1011,  //Completion for Locked Memory Read without Data
    CplDLk   = 8'b0100_1011,  //Completion for Locked Memory Read
    FetchAdd = 8'b01?0_1100,  //Fetch and Add AtomicOp Request
    Swap     = 8'b01?0_1101,  //Unconditional Swap AtomicOp Request
    CAS      = 8'b01?0_1110,  //Compare and Swap AtomicOp Request
    LPrx     = 8'b1000_????,  //Local TLP Prefix
    EPrfx    = 8'b1001_????   //End-End TLP Prefix
  } pcie_tlp_type_e;

  // Layouts. Byte k of a packet is bits [8k+7:8k]. half_byte_t, dllp_byte_t,
  // dllp_byte2_acknak_t and dllp_byte1_hdrfc_t are not used by any module.
  typedef struct packed {logic [3:0] half_byte;} half_byte_t;

  // TLP DW0, byte by byte (PCIe Base Spec r2.1, §2.2.1).
  typedef struct packed {
    logic [2:0] Fmt;
    logic [4:0] Type;
  } pcie_tlp_byte0_t;

  typedef struct packed {
    logic       RSVD2;
    logic [2:0] TC;
    logic       RSVD1;
    logic       Attr;
    logic       RSVD0;
    logic       TH;
  } pcie_tlp_byte1_t;


  typedef struct packed {
    logic       TD;
    logic       EP;
    logic [1:0] Attr;
    logic [1:0] AT;
    logic [1:0] Length1;
  } pcie_tlp_byte2_t;


  typedef struct packed {logic [7:0] Length0;} pcie_tlp_byte3_t;


  typedef struct packed {
    pcie_tlp_byte3_t byte3;
    pcie_tlp_byte2_t byte2;
    pcie_tlp_byte1_t byte1;
    pcie_tlp_byte0_t byte0;
  } pcie_tlp_header_dw0_t;

  // Not read by any module. Its fields do not follow the flow control type
  // byte, which has the type in bits [7:4] and the VC ID in bits [2:0].
  typedef struct packed {
    logic [2:0] vcd;
    logic reserved;
    logic [3:0] init_seq;
  } dllp_type_hdr_t;


  typedef struct packed {
    logic [3:0] half_byte1;
    logic [3:0] half_byte0;
  } dllp_byte_t;

  typedef union packed {
    dllp_type_e type_byte_;
    dllp_type_hdr_t type_vc;
  } dllp_type_union_t;

  typedef struct packed {
    logic [1:0] rsvd1;
    logic [7:0] HdrFC;
    logic [1:0] rsvd0;
  } dllp_hdr_t;

  typedef struct packed {
    logic [3:0] byte_1;
    logic [7:0] half_byte;
  } ack_nack_t;

  typedef union packed {
    ack_nack_t   acknack_seq_num;
    logic [11:0] data_fc;
  } dllp_seq_datafc_union_t;

  typedef union packed {
    dllp_hdr_t   hdr;
    logic [11:0] rsvd;
  } dllp_hdr_union_t;

  typedef struct packed {
    logic [3:0] reserved;
    logic [3:0] acknak1;
  } dllp_byte2_acknak_t;

  typedef struct packed {
    logic [1:0] reserved;
    logic [5:0] hdr2;
  } dllp_byte1_hdrfc_t;

  // Any DLLP. Only dllp_type is read (dllp_handler). The header and seq_datafc
  // views are not read, and their fields do not match the HdrFC, DataFC and
  // AckNak_Seq_Num positions that dllp_fc_t and dllp_ack_nack_t follow.
  typedef struct packed {
    logic [15:0] crc;
    dllp_seq_datafc_union_t seq_datafc;
    dllp_hdr_union_t header;
    dllp_type_union_t dllp_type;
  } dll_packet_t;

  // Ack or Nak DLLP: AckNak_Seq_Num is {ack_nack1, ack_nack0} (PCIe Base
  // Spec r2.1, §3.4.1).
  typedef struct packed {
    logic [15:0] crc;
    logic [7:0] ack_nack0;
    logic [3:0] rsvd1;
    logic [3:0] ack_nack1;
    logic [7:0] rsvd0;
    dllp_type_union_t ack_nack_;
  } dllp_ack_nack_t;

  typedef struct packed {
    logic [1:0] rsvd0;
    logic [5:0] hdrfc1;
  } dllp_fc_byte1_t;

  typedef struct packed {
    logic [1:0] hdrfc0;
    logic [1:0] rsvd1;
    logic [3:0] datafc1;
  } dllp_fc_byte2_t;

  // InitFC1, InitFC2 or UpdateFC DLLP: HdrFC is {byte1.hdrfc1, byte2.hdrfc0}
  // and DataFC {byte2.datafc1, datafc0} (PCIe Base Spec r2.1, §3.4.1).
  typedef struct packed {
    logic [15:0]      crc;
    logic [7:0]       datafc0;
    dllp_fc_byte2_t   byte2;
    dllp_fc_byte1_t   byte1;
    dllp_type_union_t fc_type_;
  } dllp_fc_t;


  // One DLLP, viewed by type.
  typedef union packed {
    dllp_fc_t       flow_control;
    dllp_ack_nack_t ack_nack;
    dll_packet_t    generic;
  } dllp_union_t;


  // AckNak_Seq_Num of an Ack or Nak DLLP.
  function static logic [11:0] get_ack_nack_seq(input dllp_ack_nack_t ack_nack_in);
    get_ack_nack_seq = {ack_nack_in.ack_nack1, ack_nack_in.ack_nack0};
  endfunction

  // HdrFC and DataFC of an InitFC or UpdateFC DLLP.
  function static void get_fc_values(output logic [7:0] hdr_fc_out, output logic [11:0] data_fc_out,
                                     input dllp_fc_t flow_control_in);
    hdr_fc_out  = {flow_control_in.byte1.hdrfc1, flow_control_in.byte2.hdrfc0};
    data_fc_out = {flow_control_in.byte2.datafc1, flow_control_in.datafc0};
  endfunction

  // An Ack or Nak DLLP with the given CRC field; not used by any module.
  function automatic dllp_ack_nack_t set_ack_nack(input dllp_type_e dllp_type,
                                       logic [11:0] seq_num, logic [15:0] crc_in = 16'h0);
    dllp_ack_nack_t temp_dllp = '0;
    temp_dllp.ack_nack_ = dllp_type;
    temp_dllp.ack_nack1 = seq_num[11:8];
    temp_dllp.ack_nack0 = seq_num[7:0];
    temp_dllp.crc = crc_in;
    return temp_dllp;
  endfunction

  // Bytes 0 to 3 of an Ack or Nak DLLP, without the CRC; not used by any
  // module.
  function automatic logic [31:0] build_ack_nack_payload(input dllp_type_e dllp_type,
                                                          input logic [11:0] seq_num);
    logic [31:0] payload;
    begin
      payload        = '0;
      payload[7:0]   = dllp_type;
      payload[15:8]  = 8'h00;
      payload[19:16] = seq_num[11:8];
      payload[23:20] = 4'h0;
      payload[31:24] = seq_num[7:0];
      return payload;
    end
  endfunction


  // An InitFC or UpdateFC DLLP with its CRC field 0. vcd is not used, so the
  // VC ID is bits [2:0] of dllp_type, which are 0 in every flow control value
  // of dllp_type_e.
  function automatic dllp_fc_t send_fc_init(input dllp_type_e dllp_type,
                                       input logic [2:0] vcd, input logic [7:0] hdrfc,
                                       input logic [11:0] datafc);
    begin
      dllp_fc_t dll_packet;
      dll_packet = '0;
      {dll_packet.fc_type_.type_byte_} = dllp_type;
      {dll_packet.byte1.hdrfc1, dll_packet.byte2.hdrfc0} = hdrfc;
      {dll_packet.byte2.datafc1, dll_packet.datafc0} = datafc;
      return dll_packet;
    end
  endfunction
  /* verilator lint_on WIDTHEXPAND */

  // -------------------------------------------------------------------------
  // REPLAY_TIMER limit
  // -------------------------------------------------------------------------
  // Table 3-4 gives the limit at 2.5 GT/s in Symbol Times, by
  // Max_Payload_Size and operating Link width, with a -0%/+100% tolerance
  // (PCIe Base Spec r2.1, §3.5.2.1). The Max_Payload_Size is the Device
  // Control field, reset value 000b or 128 bytes (PCIe Base Spec r2.1,
  // §7.8.4), not a buffer size. pcie_datalink_layer passes the result to
  // retry_management as REPLAY_TIMER_CYCLES.

  // Table 3-4 in Symbol Times; a size or width the table does not list
  // returns 0.
  function automatic int replay_limit_symbol_times(input int mps_bytes, input int link_width);
    int t;
    begin
      t = 0;
      case (mps_bytes)
        128:  case (link_width)
                1: t = 711;   2: t = 384;   4: t = 219;   8: t = 201;
                12: t = 174;  16: t = 144;  32: t = 99;   default: t = 0;
              endcase
        256:  case (link_width)
                1: t = 1248;  2: t = 651;   4: t = 354;   8: t = 321;
                12: t = 270;  16: t = 216;  32: t = 135;  default: t = 0;
              endcase
        512:  case (link_width)
                1: t = 1677;  2: t = 867;   4: t = 462;   8: t = 258;
                12: t = 327;  16: t = 258;  32: t = 156;  default: t = 0;
              endcase
        1024: case (link_width)
                1: t = 3213;  2: t = 1635;  4: t = 846;   8: t = 450;
                12: t = 582;  16: t = 450;  32: t = 252;  default: t = 0;
              endcase
        2048: case (link_width)
                1: t = 6285;  2: t = 3171;  4: t = 1614;  8: t = 834;
                12: t = 1095; 16: t = 834;  32: t = 444;  default: t = 0;
              endcase
        4096: case (link_width)
                1: t = 12429; 2: t = 6243;  4: t = 3150;  8: t = 1602;
                12: t = 2118; 16: t = 1602; 32: t = 828;  default: t = 0;
              endcase
        default: t = 0;
      endcase
      return t;
    end
  endfunction

  // The limit in link clock cycles at 1.75 times the table value, in the
  // upper half of the tolerance. A Symbol Time is 4 ns at 2.5 GT/s (PCIe Base
  // Spec r2.1, §3.5.2.1), so 1.75 x T Symbol Times is 7 x T ns. At x1, 128
  // bytes and 8 ns: 7 x 711 / 8 = 622 cycles, 1244 Symbol Times, within 711
  // to 1422.
  function automatic int replay_timer_cycles(input int mps_bytes, input int link_width,
                                             input int clk_period_ns);
    begin
      return (7 * replay_limit_symbol_times(mps_bytes, link_width)) / clk_period_ns;
    end
  endfunction

endpackage
