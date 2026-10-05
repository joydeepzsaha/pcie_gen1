# PCIe Transaction Layer

## Purpose

This directory implements the PCIe Transaction Layer for VC0. It converts
application commands into Transaction Layer Packets, parses received TLPs, and
routes requests and completions between the Data Link Layer and its client.
The Endpoint (`pcie_endpoint_top`, `src/pcie_endpoint`) and the Root Complex
(`pcie_rq_rc_top`, `src/rc`) both instantiate it.

The integration module is [`tlp_layer.sv`](tlp_layer.sv), and
[`tlp_core.core`](tlp_core.core) (core `::tlp_core:1.0.0`) lists the RTL with
`tlp_pkg.sv` first. `tlp_layer` carries one DW per beat and supports only
`DATA_WIDTH` = 32.

## RTL components

| Module | Responsibility |
| --- | --- |
| `tlp_pkg.sv` | TLP formats, types, classes, commands, error codes, packed headers, and helper functions. |
| `tlp_validator.sv` | Formation checks on a received header: Fmt and Type, address format, Length and Byte Enables. |
| `tlp_classifier.sv` | Posted, non-posted, completion, memory, configuration, read, and write classification. |
| `tlp_bar_decoder.sv` | Address comparison, BAR selection, overlap reporting, and target offset calculation. |
| `tlp_config_decoder.sv` | Configuration request BDF and register-offset decoding. |
| `tlp_parser.sv` | AXI-Stream TLP disassembly, header extraction, payload forwarding, framing checks, and ECRC checking. |
| `tlp_payload_formatter.sv` | Places payload bytes in their address lanes: keeps the bytes whose `tkeep` bit is set, packs them, and starts them at the lane of the first byte's address. |
| `tlp_request_tracker.sv` | Tag allocation, outstanding-request context, completion accounting, tag retirement, and the Completion Timeout. |
| `tlp_requester.sv` | Application command conversion, request segmentation, tag requests, and request-header generation. |
| `tlp_completion_generator.sv` | Completion and Completion-with-Data header generation with the payload passed through; a CplD never crosses the Read Completion Boundary or carries more than `max_payload_bytes_i`. |
| `tlp_control.sv` | Arbitration between locally generated requests and completions while preserving packet boundaries. |
| `tlp_generator.sv` | TLP serialization, optional prefix insertion, payload emission, and optional ECRC insertion. |
| `tlp_ecrc.sv` | End-to-end CRC calculation. |
| `tlp_credit_manager.sv` | Independent posted, non-posted, and completion header/data credit accounting. |
| `tlp_vc_buffer.sv` | VC0 packet buffering, packet atomicity, credit metadata, and output backpressure. |
| `tlp_layer.sv` | Top-level connection of all Transaction Layer functions. |

## Testbench organization

The Transaction Layer tests are in [`../../tb/tlp`](../../tb/tlp), and
[`tb_tlp.core`](../../tb/tlp/tb_tlp.core) (core `fusesoc:pcie:tb_tlp:1.0.0`)
defines the Verilator targets that run them. Small SystemVerilog wrappers
(`tb_tlp_*.sv`) expose packed structures and module ports to cocotb; the
targets whose toplevel is `tlp_layer` drive its own ports. Python tests provide
stimulus, reference calculations, monitors, and assertions.

| Target | Toplevel | cocotb module | Coverage |
| --- | --- | --- | --- |
| `verilate_tlp_requester` | `tb_tlp_requester` | `test_tlp_requester` | Non-posted segmentation, tag timing and the 4-KiB boundary; posted write backpressure; zero-length read, rejected zero-length write, the 4-DW header above 4 GiB, an early or missing `command_data_last_i`, and reset with a read pending. |
| `verilate_tlp_request_tracker` | `tb_tlp_request_tracker` | `test_tlp_request_tracker` | Tag exhaustion at 32 tags, split completions and tag reuse; completions with a wrong requester ID, byte count or lower address; result backpressure; an error completion ending a request. |
| `verilate_tlp_cpl_timeout_off` | `tb_tlp_request_tracker` | `test_tlp_request_tracker` | The same tests with `CPL_TIMEOUT_CYCLES` = 0, the Completion Timeout disabled. |
| `verilate_tlp_parser` | `tb_tlp_parser` | `test_tlp_parser` | Posted, non-posted and completion parsing with input gaps and payload backpressure; partial keep, prefix without header, truncated header and short payload, then recovery; prefix, ECRC digest and its corruption, and reset during a packet. |
| `verilate_tlp_generator` | `tb_tlp_generator` | `test_tlp_generator` | Request headers with prefix, unaligned payload and digest under output stalls; requests and completions without data; output stability and reset. |
| `verilate_tlp_completion_gen` | `tb_tlp_completion_control` | `test_tlp_completion_control` | Completion fields; a completion without Relaxed Ordering held behind a waiting Memory Write and passing it with Relaxed Ordering set; packet locking; a no-data error completion; a 200-byte completion split at 128 bytes; round-robin at packet boundaries. |
| `verilate_tlp_comb` | `tb_tlp_comb` | `test_tlp_comb` | Package helpers, all traffic classes, BAR boundaries, BAR overlap, `memory_enable_i`, and configuration decode boundaries. |
| `verilate_tlp_payload_formatter` | `tb_tlp_payload_formatter` | `test_tlp_payload_formatter` | Start offsets 0 to 3, each with eleven payload lengths from 1 to 63 bytes, under output backpressure, and reset with a payload buffered. |
| `verilate_tlp_compile` | `tlp_layer` | `test_tlp_compile` | Layer reset, request routing, malformed timing, local command output, and exact credit-class blocking. |
| `verilate_tlp_end_to_end` | `tlp_layer` | `test_tlp_end_to_end` | Request families, 3-DW/4-DW formats, prefix/ECRC alignment, segmentation, request-to-completion tracking, malformed traffic, and recovery (see below). |
| `verilate_tlp_cfg0_spine` | `tlp_layer` | `test_tlp_cfg0_spine` | CfgRd0 and CfgWr0 sent, and their completions matched to the requests. |
| `verilate_tlp_cfg1_spine` | `tlp_layer` | `test_tlp_cfg1_spine` | CfgRd1 and CfgWr1 sent, and their completions matched to the requests. |
| `verilate_tlp_conf_cfg1` | `tlp_layer` | `test_tlp_conf_cfg1` | CfgRd1 and CfgWr1 headers, the (offset, byte count) pairs the requester admits, and an over-length CFG1 request rejected rather than split. |
| `verilate_tlp_conf_requester` | `tlp_layer` | `test_tlp_conf_requester` | MRd, MWr, IORd and IOWr headers, 3-DW and 4-DW, byte enables, and segmentation at the read request size, the payload size and the 4-KiB boundary. |
| `verilate_tlp_conf_tracker` | `tlp_layer` | `test_tlp_conf_tracker` | Out-of-order, multi-completion, unexpected, duplicate and malformed completions, tag exhaustion, and result backpressure. |
| `verilate_tlp_conf_parser` | `tb_tlp_conf_parser` | `test_tlp_conf_parser` | 3-DW and 4-DW memory requests, CplD, Cpl, configuration address, prefix, ECRC digest, poisoned, truncated, bad keep, and reset during a packet. |
| `verilate_tlp_conf_completion` | `tb_tlp_completion_control` | `test_tlp_conf_completion` | Cpl and CplD fields, status values, byte count and lower address, unaligned length, and completion priority in `tlp_control`. |
| `verilate_tlp_conf_generator` | `tb_tlp_generator` | `test_tlp_conf_generator` | CplD and Cpl as serialized by `tlp_generator`. |
| `verilate_tlp_conf_classifier` | `tb_tlp_comb` | `test_tlp_conf_classifier` | The class of memory, I/O, Type 0 and Type 1 configuration requests and of completions; locked, AtomicOp, undefined, over-length, 4-DW configuration, I/O and completion, and non-zero-length Cpl headers as unsupported. |
| `verilate_tlp_conf_cfgbe` | `tlp_layer` | `test_tlp_conf_cfgbe` | Byte-enable matrices for CfgWr0, CfgRd0, IOWr and IORd; an unaligned 4-byte configuration or I/O request rejected rather than split; memory byte enables unchanged; recovery after a rejection. |
| `verilate_tlp_conf_datalast` | `tb_tlp_requester` | `test_tlp_conf_datalast` | `command_data_last_i` on two- and three-segment writes, raised early and the recovery after it, an overrun at the byte count, and a partial keep on the last beat. |
| `verilate_tlp_conf_formatter` | `tb_tlp_payload_formatter` | `test_tlp_conf_formatter` | Offsets 0, 1 and 2, a carry into the second DW, and output backpressure. |
| `verilate_tlp_cpl_timeout` | `tb_tlp_request_tracker` | `test_tlp_cpl_timeout` | At `CPL_TIMEOUT_CYCLES` = 64: the expiry cycle, recovery after `TAG_COUNT` + 2 unanswered requests, the timer restart on a partial completion, a late completion and tag reuse, and a second expiry. |
| `verilate_tlp_cpl_timeout_default` | `tb_tlp_request_tracker` | `test_tlp_cpl_timeout_default` | The bench instance's 6,250-cycle `CPL_TIMEOUT_CYCLES` and the RTL default, through the second instance `dut_default_witness`. |
| `measure_7g2_cpl_sweep` | `tb_tlp_request_tracker` | `test_7g2_cpl_sweep` | Measures the cycle at which the bench instance's 6,250-cycle timeout fires in each of the 32 scan phases and writes it to a JSON file; each run ends long before `dut_default_witness` can fire. |
| `verilate_tlp_credit_manager` | `tb_tlp_credit_manager` | `test_tlp_credit_manager` | Exact credit consumption, short and repeated updates, independence of the six credit pools, cumulative counter wrap, header exhaustion, infinite pools, and `error_o`. |
| `verilate_tlp_credit_integration` | `tlp_layer` | `test_tlp_credit_integration` | `tlp_layer` transmits nothing past the cumulative credit limit, and still transmits on a single advertisement. |
| `verilate_tlp_vc_buffer` | `tb_tlp_vc_buffer` | `test_tlp_vc_buffer` | Packet atomicity, credit metadata, backpressure, slot isolation and boundaries, a write into one slot while the other drains, pointer wrap, and overflow. |
| `verilate_tlp_vc_buffer_wrap` | `tb_tlp_vc_buffer` | `test_tlp_vc_buffer_wrap` | At `PACKET_DEPTH` = 3 and `MAX_PACKET_WORDS` = 5: pointer wrap at a depth that is not a power of two, a maximum-length packet in every slot, metadata per slot, and zero data credits for a TLP without data. |

Each target builds and runs one cocotb module in its own simulation, and
FuseSoC 2.4.6 gives each flow target its own work directory under the build
root, so tags, credits, packet buffers, and parser state cannot leak between
suites.

## End-to-end Transaction Layer process

The `tlp_layer` end-to-end test (`test_tlp_end_to_end.py`) initializes:

- Link-up and transmit-enable state.
- Requester and completer IDs.
- Bus, device, and function numbers.
- Memory enable and extended tags.
- Maximum payload and maximum read request sizes.
- Posted, non-posted, and completion credits.
- `rcb_128b_i` and `m_dllp_axis_tready` at 1, and every other input whose name
  ends in `_i`, the target, completion, and result ready signals included, at
  0.

It then performs four groups of checks, each a cocotb test:

1. Send each of six request commands (memory, I/O and Type 0 configuration
   reads and writes) and decode its TLP on the receive side, with 3-DW and
   4-DW header forms; then receive a Type 1 configuration request.
2. Exercise prefix, ECRC, maximum-size, and segmented transfers.
3. Issue a non-posted request, return its completion, check the saved context,
   and confirm tag retirement.
4. Inject malformed timing, header, keep, and ECRC cases, then verify that a
   later valid packet is accepted.

## Running the tests

From the repository root, with the git submodules checked out
(`git submodule update --init`), FuseSoC 2.4.6 finds the cores with
`--cores-root .`. Run one suite by its target:

```bash
fusesoc --cores-root . run --target=verilate_tlp_parser fusesoc:pcie:tb_tlp:1.0.0
fusesoc --cores-root . run --target=verilate_tlp_requester fusesoc:pcie:tb_tlp:1.0.0
fusesoc --cores-root . run --target=verilate_tlp_end_to_end fusesoc:pcie:tb_tlp:1.0.0
```

The table above lists every target of `tb_tlp.core`.

## Pass criteria

A target passes when every cocotb test in its module passes. cocotb 1.9.2
writes each test's result to `results.xml`, the default of
`COCOTB_RESULTS_FILE`, in the directory the simulation runs in.

## Scope

The RTL implements the Transaction Layer around VC0 and the interfaces used by
this project. These benches simulate `tlp_layer` or its submodules alone, so
they do not test Data Link replay, LTSSM operation, or Physical Layer
encoding. Those are tested by the benches of their own layers (for example
`tb/dllp` and `tb/ltssm`), and the layers together by the benches in
`tb/endpoint`, `tb/rc_ep` and `tb/fullstack`.
