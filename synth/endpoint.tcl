# Ordered RTL manifest for the integrated PCIe Gen1 endpoint.
#
# This file is sourced by s1_ooc.tcl for UNIT=endpoint.  The SystemVerilog
# hierarchy performs the instantiation; this manifest makes every dependency
# available to Vivado in package-before-module order.
#
# Endpoint architecture:
#   TLP -> DLL -> logical PHY -> Gen1 physical processing
# The active physical processing retains the Gen1 byte scrambler and 8b/10b
# encoder/decoder.  Gen3 scrambling is preserved in the repository for future
# use, but is intentionally commented out of this Gen1 endpoint manifest:
#   src/scrambler/gen3_byte_scramble.sv
#   src/scrambler/gen3_scramble.sv

set ENDPOINT_TOP pcie_endpoint_top
set ENDPOINT_GENERICS {
  INTEGRATED_GEN1_PHY=1
  MAX_NUM_LANES=1
}

set ENDPOINT_FILES {
  src/tlp/tlp_pkg.sv
  src/packages/pcie_datalink_pkg.sv
  src/packages/pcie_phy_pkg.sv
  src/packages/pcie_tlp_pkg.sv
  src/pcie_cfg/pcie_config_reg_pkg.sv

  src/verilog-axis/rtl/priority_encoder.v
  src/verilog-axis/rtl/arbiter.v
  src/verilog-axis/rtl/axis_register.v
  src/verilog-axis/rtl/axis_arb_mux.v
  src/verilog-axis/rtl/axis_fifo.v
  src/verilog-axis/rtl/axis_adapter.v
  src/verilog-axis/rtl/axis_async_fifo.v

  src/async_fifo/rtl/fifomem.v
  src/async_fifo/rtl/fifomem_dp.v
  src/async_fifo/rtl/rptr_empty.v
  src/async_fifo/rtl/sync_ptr.v
  src/async_fifo/rtl/sync_r2w.v
  src/async_fifo/rtl/sync_w2r.v
  src/async_fifo/rtl/wptr_full.v
  src/async_fifo/rtl/async_fifo.v
  src/async_fifo/rtl/async_bidir_fifo.v
  src/async_fifo/rtl/async_bidir_ramif_fifo.v

  src/bram/bram_dp.sv
  src/bram/bram_sp.sv

  src/crc/Crc16Gen.sv
  src/crc/pcie_crc8.v
  src/crc/pcie_dllp_crc8.v
  src/crc/pcie_datalink_crc.sv
  src/crc/pcie_lcrc16.sv
  src/crc/pcie_lcrc32.sv

  src/verilog-pcie/rtl/pcie_tlp_fifo_raw.v
  src/verilog-pcie/rtl/pcie_tlp_fifo.v
  src/converters/axis_to_pcie_converter.sv
  src/converters/pcie_to_axis_converter.sv

  src/pcie_cfg/pcie_config_decode.sv
  src/pcie_cfg/pcie_config_handler.sv
  src/pcie_cfg/pcie_config_mux.sv
  src/pcie_cfg/pcie_config_reg.sv
  src/pcie_cfg/pcie_cfg_wrapper.sv

  src/dllp/axis_retry_fifo.sv
  src/dllp/axis_user_demux.sv
  src/dllp/dllp_handler.sv
  src/dllp/dllp_fc_update.sv
  src/dllp/dllp2tlp.sv
  src/dllp/retry_transmit.sv
  src/dllp/retry_management.sv
  src/dllp/tlp2dllp.sv
  src/dllp/dllp_receive.sv
  src/dllp/dllp_transmit.sv
  src/dllp/pcie_datalink_init.sv
  src/dllp/pcie_flow_ctrl_init.sv
  src/dllp/pcie_datalink_layer.sv

  src/scrambler/byte_scramble.sv
  src/scrambler/gen1_scramble.sv
  src/scrambler/scrambler.sv
  src/scrambler/encode_8b10b.sv
  src/scrambler/decode_8b10b.sv

  src/pcie_phy_core/lfsr.v
  src/pcie_phy_core/synchronous_lifo.sv
  src/pcie_phy_core/synchronous_fifo.sv
  src/pcie_phy_core/frame_symbols.sv
  src/pcie_phy_core/lane_management.sv
  src/pcie_phy_core/os_generator.sv
  src/pcie_phy_core/ordered_set_handler.sv
  src/pcie_phy_core/block_alignment.sv
  src/pcie_phy_core/pack_data.sv
  src/pcie_phy_core/data_handler.sv
  src/pcie_phy_core/phy_receive.sv
  src/pcie_phy_core/phy_transmit.sv

  src/ltssm/pcie_ltssm_downstream.sv

  src/tlp/tlp_ecrc.sv
  src/tlp/tlp_validator.sv
  src/tlp/tlp_classifier.sv
  src/tlp/tlp_bar_decoder.sv
  src/tlp/tlp_config_decoder.sv
  src/tlp/tlp_parser.sv
  src/tlp/tlp_payload_formatter.sv
  src/tlp/tlp_request_tracker.sv
  src/tlp/tlp_requester.sv
  src/tlp/tlp_completion_generator.sv
  src/tlp/tlp_control.sv
  src/tlp/tlp_generator.sv
  src/tlp/tlp_credit_manager.sv
  src/tlp/tlp_vc_buffer.sv
  src/tlp/tlp_layer.sv

  src/pcie_endpoint/pcie_endpoint_top.sv
}
