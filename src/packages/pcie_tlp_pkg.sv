// ---------------------------------------------------------------------------
// pcie_tlp_pkg -- TLP header structs and Completion builders
//
// Purpose
//   Packed views of TLP header fields and two functions that build a
//   Completion, for the Endpoint's configuration path: pcie_config_decode
//   collects each received header in a tlp_hdr_union_t for
//   pcie_config_handler, which answers a CfgRd0 with gen_cpld and a CfgWr0
//   with gen_cpl. axis_to_pcie_converter and pcie_to_axis_converter also use
//   tlp_hdr_union_t.
//   Two byte orders are in use. tlp_hdr_t holds the first byte of each
//   header DW in bits 31:24; pcie_config_decode reverses the bytes of every
//   received DW to fill it. cpl_tlp_hdr_t holds the first byte of each DW in
//   bits 7:0, as pcie_datalink_pkg's pcie_tlp_header_dw0_t does.
//
// Contents
//   Request header     tlp_hdr_byte_0_t to tlp_hdr_byte_3_t, common_tlp_hdr_t
//                      (DW0); read_req_dw_1_t (DW1); word_3_tlp_byte_0_t to
//                      word_3_tlp_byte_3_t, tlp_hdr_word_2_t (a Configuration
//                      Request's DW2); tlp_hdr_word_3_t; tlp_hdr_t and
//                      tlp_hdr_union_t, the whole header.
//   Completion         cpl_tlp_dw1_byte_*_t, cpl_tlp_dw1_t,
//                      cpl_tlp_dw2_byte_*_t, cpl_tlp_dw2_t, cpl_tlp_hdr_t: a
//                      3 DW Completion header and one data DW.
//   Builders           gen_cpld, gen_cpl.
//   Not used anywhere  word_2_tlp_hdr_t, tlp_hdr_word_1_t, cpl_tlp_dw3_t, and
//                      the Command and Status register types
//                      command_register_t, status_register_t, cfg_reg_ids_t.
//
// References
//   PCIe Base Spec r2.1, §2.2.1
//   PCIe Base Spec r2.1, §2.2.7
//   PCIe Base Spec r2.1, §2.2.9
//   PCIe Base Spec r2.1, §7.5.1.1
//   PCIe Base Spec r2.1, §7.5.1.2
//   PCI Local Bus Spec r3.0, §6.2.2
//   PCI Local Bus Spec r3.0, §6.2.3
// ---------------------------------------------------------------------------
package pcie_tlp_pkg;

  import pcie_datalink_pkg::*;

  // Header DW0, one struct per byte (PCIe Base Spec r2.1, §2.2.1). Attr is
  // Attr[2] in byte 1 and Attr[1:0] in byte 2. Length in byte 2 is
  // Length[9:8]; tlp_hdr_byte_3_t, below, holds Length[7:0].
  typedef struct packed {
    logic [7:5] Fmt;
    logic [4:0] Type;
  } tlp_hdr_byte_0_t;

  typedef struct packed {
    logic [7:7] R_2;
    logic [6:4] TC;
    logic [3:3] R_1;
    logic [2:2] Attr;
    logic [1:1] R_0;
    logic [0:0] TH;
  } tlp_hdr_byte_1_t;

  typedef struct packed {
    logic [7:7] TD;
    logic [6:6] EP;
    logic [5:4] Attr;
    logic [3:2] AT;
    logic [1:0] Length;
  } tlp_hdr_byte_2_t;

  // Completion header DW1 (bytes 4 to 7), one struct per byte: the Completer
  // ID, high byte first; Completion Status, BCM and Byte Count[11:8]; Byte
  // Count[7:0] (PCIe Base Spec r2.1, §2.2.9).
  typedef struct packed {logic [7:0] completer_id;} cpl_tlp_dw1_byte_0_t;
  typedef struct packed {logic [7:0] completer_id;} cpl_tlp_dw1_byte_1_t;
  typedef struct packed {
    logic [7:5] status;
    logic [4:4] b;
    logic [3:0] byte_count;
  } cpl_tlp_dw1_byte_2_t;
  typedef struct packed {logic [7:0] byte_count;} cpl_tlp_dw1_byte_3_t;


  // Completion header DW2 (bytes 8 to 11), one struct per byte: the Requester
  // ID, high byte first; the Tag; a Reserved bit and Lower Address.
  typedef struct packed {logic [7:0] requester_id;} cpl_tlp_dw2_byte_0_t;
  typedef struct packed {logic [7:0] requester_id;} cpl_tlp_dw2_byte_1_t;
  typedef struct packed {logic [7:0] tag;} cpl_tlp_dw2_byte_2_t;
  typedef struct packed {
    logic [7:7] reserved;
    logic [6:0] lower_address;
  } cpl_tlp_dw2_byte_3_t;



  typedef struct packed {logic [7:0] Length;} tlp_hdr_byte_3_t;

  // Header DW0 with byte 0 in bits 31:24.
  typedef struct packed {
    tlp_hdr_byte_0_t byte0;
    tlp_hdr_byte_1_t byte1;
    tlp_hdr_byte_2_t byte2;
    tlp_hdr_byte_3_t byte3;
  } common_tlp_hdr_t;


  // Not used.
  typedef struct packed {logic [31:0] word_1;} word_2_tlp_hdr_t;


  // Completion header DW1 with byte 4 in bits 7:0.
  typedef struct packed {
    cpl_tlp_dw1_byte_3_t byte3;
    cpl_tlp_dw1_byte_2_t byte2;
    cpl_tlp_dw1_byte_1_t byte1;
    cpl_tlp_dw1_byte_0_t byte0;
  } cpl_tlp_dw1_t;


  // Completion header DW2 with byte 8 in bits 7:0.
  typedef struct packed {
    cpl_tlp_dw2_byte_3_t byte3;
    cpl_tlp_dw2_byte_2_t byte2;
    cpl_tlp_dw2_byte_1_t byte1;
    cpl_tlp_dw2_byte_0_t byte0;
  } cpl_tlp_dw2_t;


  // Request header DW1 with byte 4 in bits 31:24: Requester ID, Tag, Last DW
  // BE and 1st DW BE (PCIe Base Spec r2.1, §2.2.7).
  typedef struct packed {
    logic [31:16] requester_id;
    logic [15:8]  tag;
    logic [7:4]   last_byte_enable;
    logic [3:0]   first_byte_enable;
  } read_req_dw_1_t;


  // Not used.
  typedef struct packed {logic [31:0] reserved;} cpl_tlp_dw3_t;


  // A Completion in the order pcie_config_handler sends it, from bits 31:0
  // up: the 3 DW header, then one data DW.
  typedef struct packed {
    logic [31:0]          data;
    cpl_tlp_dw2_t         dw_2;
    cpl_tlp_dw1_t         dw_1;
    pcie_tlp_header_dw0_t dw_0;
  } cpl_tlp_hdr_t;



  // DW2 of a Configuration Request (bytes 8 to 11), one struct per byte:
  // Bus Number; Device and Function Number; then the bytes that hold the
  // Extended Register and Register Numbers (PCIe Base Spec r2.1, §2.2.7).
  // Despite the word_3_ prefix these types make up tlp_hdr_word_2_t.
  typedef struct packed {logic [7:0] Bus_Number;} word_3_tlp_byte_0_t;

  typedef struct packed {
    logic [7:3] Device_Number;
    logic [2:0] Function_Number_With_ARI;
  } word_3_tlp_byte_1_t;


  typedef struct packed {logic [7:0] byte_2;} word_3_tlp_byte_2_t;

  typedef struct packed {logic [7:0] byte_3;} word_3_tlp_byte_3_t;

  // Not used: tlp_hdr_t takes read_req_dw_1_t for DW1.
  typedef struct packed {logic [31:0] word_1;} tlp_hdr_word_1_t;


  // Header DW2 with byte 8 in bits 31:24.
  typedef struct packed {
    word_3_tlp_byte_0_t byte_0;
    word_3_tlp_byte_1_t byte_1;
    word_3_tlp_byte_2_t byte_2;
    word_3_tlp_byte_3_t byte_3;
  } tlp_hdr_word_2_t;


  typedef struct packed {logic [31:0] word_3;} tlp_hdr_word_3_t;


  // A request header of up to 4 DW, DW0 in bits 31:0. Each DW holds its first
  // byte in bits 31:24.
  typedef struct packed {
    tlp_hdr_word_3_t word_3;
    tlp_hdr_word_2_t word_2;
    read_req_dw_1_t  word_1;
    common_tlp_hdr_t word_0;
  } tlp_hdr_t;

  // tlp_hdr_t over the flat 128-bit header bus.
  typedef union packed {
    tlp_hdr_t struct_;
    logic [127:0] whole_;
  } tlp_hdr_union_t;




  // The Command register (PCI Local Bus Spec r3.0, §6.2.2; PCIe Base Spec
  // r2.1, §7.5.1.1). The field named reserved holds bit 1, Memory Space
  // Enable, and bit 0, I/O Space Enable; unused holds reserved bits 15:11.
  typedef struct packed {
    logic [15:11] unused;
    logic [10:10] interrupt_disable;
    logic [9:9]   fast_b2b_transactions_enable;
    logic [8:8]   SERR_Enable;
    logic [7:7]   idsel_step_wait_cycle_control;
    logic [6:6]   parity_error_response;
    logic [5:5]   vga_palette_snoop;
    logic [4:4]   memory_write_invalidate;
    logic [3:3]   special_cycle_enable;
    logic [2:2]   bus_master_enable;
    logic [1:0]   reserved;
  } command_register_t;


  // The Status register (PCI Local Bus Spec r3.0, §6.2.3; PCIe Base Spec
  // r2.1, §7.5.1.2).
  typedef struct packed {
    logic [15:15] detected_parity_error;
    logic [14:14] signaled_system_error;
    logic [13:13] received_master_abort;
    logic [12:12] received_target_abort;
    logic [11:11] signaled_target_abort;
    logic [10:9]  devsel_timing;
    logic [8:8]   master_data_parity_error;
    logic [7:7]   fast_b2b_transactions_capable;
    logic [6:6]   unused;
    logic [5:5]   sixtysix_mhz_capable;
    logic [4:4]   capabilities_list;
    logic [3:3]   interrupt_status;
    logic [2:0]   reserved;
  } status_register_t;


  // The DW at configuration offset 04h: Status in bits 31:16, Command in
  // bits 15:0.
  typedef struct packed {
    status_register_t  status;
    command_register_t command;
  } cfg_reg_ids_t;



  // Builds pcie_config_handler's CplD for a CfgRd0 from the request header
  // and the register value data_in: Length 1, Byte Count 4, and Lower
  // Address, Completion Status, TC and Attr 0. The Completer ID is the Bus,
  // Device and Function Number the request addressed (its DW2); Requester ID
  // and Tag are copied from its DW1.
  function static cpl_tlp_hdr_t gen_cpld(input tlp_hdr_t tlp_hdr_in, logic [31:0] data_in);
    begin
      cpl_tlp_hdr_t temp_cpl;
      temp_cpl = '0;
      temp_cpl.data = data_in;
      temp_cpl.dw_0.byte0 = CplD;
      {temp_cpl.dw_0.byte2.Length1, temp_cpl.dw_0.byte3.Length0} = 10'h01;
      {temp_cpl.dw_1.byte2.byte_count, temp_cpl.dw_1.byte3.byte_count} = 12'h004;
      {temp_cpl.dw_1.byte0.completer_id, temp_cpl.dw_1.byte1.completer_id} =
      {tlp_hdr_in.word_2.byte_0,tlp_hdr_in.word_2.byte_1};
      temp_cpl.dw_2.byte3.lower_address = 6'h00;
      temp_cpl.dw_2.byte2.tag = tlp_hdr_in.word_1.tag;
      {temp_cpl.dw_2.byte0.requester_id,temp_cpl.dw_2.byte1.requester_id} =
      tlp_hdr_in.word_1.requester_id;
      return temp_cpl;
    end
  endfunction


  // Builds pcie_config_handler's Cpl for a CfgWr0, as gen_cpld does but with
  // Length 0 and Byte Count 000h, the encoding of 4096 bytes (PCIe Base Spec
  // r2.1, §2.2.9). data_in fills the data DW, which pcie_config_handler does
  // not send with a Cpl.
  function static cpl_tlp_hdr_t gen_cpl(input tlp_hdr_t tlp_hdr_in, logic [31:0] data_in);
    begin
      cpl_tlp_hdr_t temp_cpl;
      temp_cpl = '0;
      temp_cpl.data = data_in;
      temp_cpl.dw_0.byte0 = Cpl;
      {temp_cpl.dw_0.byte2.Length1, temp_cpl.dw_0.byte3.Length0} = 10'h00;
      {temp_cpl.dw_1.byte2.byte_count, temp_cpl.dw_1.byte3.byte_count} = 12'h000;
      {temp_cpl.dw_1.byte0.completer_id, temp_cpl.dw_1.byte1.completer_id} = 
      {tlp_hdr_in.word_2.byte_0,tlp_hdr_in.word_2.byte_1};
      temp_cpl.dw_2.byte3.lower_address = 6'h00;
      temp_cpl.dw_2.byte2.tag = tlp_hdr_in.word_1.tag;
      {temp_cpl.dw_2.byte0.requester_id,temp_cpl.dw_2.byte1.requester_id} =
      tlp_hdr_in.word_1.requester_id;
      return temp_cpl;
    end
  endfunction

endpackage
