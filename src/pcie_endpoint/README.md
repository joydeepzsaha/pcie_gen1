# PCIe Endpoint Protocol Integration

## Purpose

This directory contains the PCIe Endpoint top module,
[`pcie_endpoint_top.sv`](pcie_endpoint_top.sv). Its FuseSoC description is
[`pcie_endpoint.core`](pcie_endpoint.core) (core
`fusesoc:pcie:endpoint_protocol:1.0.0`). The Endpoint places the Transaction
Layer above the Data Link Layer and exposes:

- An application command interface for generating requests.
- A target interface for requests received by the Endpoint.
- A completer interface for the Completions the application sends, and a
  completion and result interface for the Completions it receives.
- Toward the Physical Layer, either packet-oriented AXI-Stream ports or 10-bit
  Symbols with PIPE command and status signals (see below).
- The Bus, Device and Function Numbers, the peer's flow control credits, and
  error and status reports.

The parameter `INTEGRATED_GEN1_PHY` selects what lies below the Data Link
Layer:

- 0, the default: the Data Link Layer's PHY streams are the `s_phy_axis_*` and
  `m_phy_axis_*` ports, with `phy_link_up_i` and `idle_valid_i` standing in for
  the LTSSM. The Symbol PHY outputs are tied off.
- 1: `phy_receive` and `phy_transmit` (`src/pcie_phy_core`), the LTSSM
  `pcie_ltssm_downstream` (`src/ltssm`), and an `encode_8b10b` and
  `decode_8b10b` per Symbol (`src/scrambler`) form a Gen1 logical PHY. The
  boundary is then two 10-bit Symbols per lane per cycle
  (`phy_rx_symbol_i`, `phy_tx_symbol_o`) plus PIPE command and status signals,
  named after PG239, Table 9: Command Signals, Table 10: Status Signals and
  Table 11: TX Driver Signals for Gen1 and Gen2. The packet PHY ports are idle.

## Layer organization

```text
Endpoint application
  |
  | command_*, target_*, completion_request_*, received_completion_*, result_*
  v
tlp_layer (src/tlp)
  |
  | TLPs, one DW per beat
  v
pcie_datalink_layer (src/dllp), with pcie_cfg_wrapper (src/pcie_cfg)
  |
  | TLPs with sequence number and LCRC, and DLLPs
  v
INTEGRATED_GEN1_PHY = 0: s_phy_axis_* and m_phy_axis_* ports
INTEGRATED_GEN1_PHY = 1: phy_receive, phy_transmit, 8b/10b codec and
                         pcie_ltssm_downstream, to Symbol and PIPE ports
```

The Transaction Layer generates and parses TLPs, manages tags, classifies
requests, decodes BAR and configuration accesses, checks ECRC, buffers VC0
traffic, and consumes flow control credits. The Data Link Layer performs flow
control initialization, DLLP processing, sequence numbering, LCRC checking,
Ack and Nak handling, and replay. The configuration space sits in the Data
Link Layer's receive path: `pcie_cfg_wrapper` completes each CfgRd0 and CfgWr0
itself and supplies the Bus, Device and Function Numbers, which the Endpoint
uses as both its Requester ID and its Completer ID.

Received InitFC and UpdateFC values are exported by the Data Link Layer and
connected directly to the Transaction Layer credit manager. The Endpoint
reports flow control as initialized only after the local FC2 advertisement has
been sent and the remote FC2 values have been received.

`clk_i` clocks the Transaction Layer and the Data Link Layer. In the
integrated PHY, `pipe_rx_usr_clk_i` clocks the LTSSM, the PIPE side of
`phy_receive` and the ordered-set generator of `phy_transmit`, and
`pipe_tx_usr_clk_i` clocks the PIPE side of `phy_transmit`. The packet streams
cross between the domains through `axis_async_fifo` inside `phy_receive` and
`phy_transmit`; the link-up and idle indications and the retrain handshake
cross through two-flop synchronizers.

## Testbench files

The Endpoint testbench is located in [`../../tb/endpoint`](../../tb/endpoint):

| File | Purpose |
| --- | --- |
| `tb_pcie_endpoint_top.sv` | Instantiates the Endpoint with a single enabled BAR and exposes the internal TLP-to-DLL and DLL-to-TLP streams for verification. |
| `test_pcie_endpoint_top.py` | Cocotb stimulus, frame capture, packet construction, CRC checks, and the five Endpoint tests. |
| `tb_pcie_endpoint_top.core` | Core `fusesoc:pcie:tb_endpoint_protocol:1.0.0`, with the targets `sim` (VCS) and `verilate_endpoint_top` (Verilator). |

The SystemVerilog harness uses a 32-bit AXI-Stream interface, one enabled
4-KiB BAR at address zero, a shortened replay timer (`REPLAY_TIMER_CYCLES` =
64), `MAX_REPLAY_ATTEMPTS` = 2, and verification-only signals named
`mid_tx_axis_*` and `mid_rx_axis_*`. It leaves `INTEGRATED_GEN1_PHY` at 0 and
ties the Symbol PHY inputs to 0.

Two other benches instantiate `pcie_endpoint_top`: `tb/rc_ep` (target
`verilate_rc_ep` of `fusesoc:pcie:tb_rc_ep:1.0.0`) connects its packet PHY ports
to those of `pcie_enum_dl_top`, a Root Complex top, through a mux that can give
the Endpoint's receive stream to a bench injector instead, and `tb/fullstack`
(target `verilate_fullstack` of `fusesoc:pcie:tb_fullstack:1.0.0`) builds it
with `INTEGRATED_GEN1_PHY` = 1.

## Common test setup

Each cocotb test builds its own `EndpointTB` and performs the following
setup:

1. Start the 8 ns `clk_i` clock.
2. Assert reset and hold the physical link down.
3. Initialize all command, target, completion, and result interfaces.
4. Release reset and drive `phy_link_up_i`, `idle_valid_i` and
   `transmit_enable_i` high.
5. Send InitFC1 and InitFC2 DLLPs for the posted, non-posted, and completion
   credit types, InitFC2-P twice.
6. Wait for `fc_initialized_o`.
7. Check that the received header credits are visible on `fc_ph_o`, `fc_nph_o`
   and `fc_cplh_o`.

The physical-facing `tuser` classification used by the test is:

| Value | Packet class |
| ---: | --- |
| `1` | DLLP |
| `2` | TLP/link packet |

## Tests performed

### Application input to Data Link output

`application_input_reaches_data_link_output` submits a Memory Write through the
application command interface.

The test verifies:

- The command and payload are accepted through ready/valid handshakes.
- A complete TLP crosses the Transaction-to-Data-Link boundary.
- The Data Link output contains the same TLP with a two-byte sequence field.
- The output LCRC matches the CRC calculated by the testbench.
- The original payload is present in the generated TLP.
- `command_error_valid_o` is low once the link packet has arrived.

### Physical input to endpoint target

`physical_input_reaches_target_through_mid_layer` constructs a Memory Write,
adds sequence number zero and a valid LCRC, and injects it through the
physical-facing input.

The test verifies:

- The Data Link Layer accepts and strips the sequence number and LCRC.
- The TLP at `mid_rx_axis_*` exactly matches the injected raw TLP.
- The Transaction Layer identifies a Memory Write.
- `target_bar_hit_o` is set and the decoded offset is correct.
- The request header remains stable while the target interface is stalled.
- The complete payload reaches the target data interface without alteration.

### NAK replay

`data_link_nak_replays_transaction_layer_packet` generates a Memory Read,
captures its raw TLP and link packet, and sends a NAK whose sequence number is
the one before the packet's. It then sends an ACK for the packet.

The test verifies:

- The initial link packet contains the TLP generated by the Transaction Layer.
- The NAK causes the stored packet to be replayed.
- The replay is byte-for-byte identical to the original link packet.

### Corrupted LCRC rejection

`corrupted_link_input_is_rejected_with_nak` injects a Memory Write with one
LCRC bit inverted.

The test verifies:

- The Data Link Layer returns a NAK carrying sequence number `0xFFF`,
  NEXT_RCV_SEQ - 1 while no TLP has been accepted.
- The corrupted request is not presented to the Transaction Layer target in
  the 32 cycles after the NAK.

### Flow-control blocking and release

`flow_control_blocks_and_releases_mid_layer` sends a posted UpdateFC DLLP with
HdrFC and DataFC of 0, then submits a Memory Write.

The test verifies:

- `tx_fc_blocked_o` asserts while posted credits are unavailable.
- The packet has not crossed the TLP-to-DLL boundary when `tx_fc_blocked_o`
  asserts.
- After a later UpdateFC DLLP replenishes posted credits, the queued TLP is
  released and appears in the Data Link output.

## Running the test

From the repository root, with the git submodules checked out
(`git submodule update --init`), register the repository if it is not already
in the FuseSoC library:

```bash
fusesoc library add pcie-endpoint-controller ./
```

FuseSoC 2.4.6 writes this entry to `fusesoc.conf` in the current directory
unless `--config`, `--global` or the `FUSESOC_CONFIG` variable names another
file. Placing `--cores-root .` before the subcommand, as in
`fusesoc --cores-root . run ...`, works without it.

The Endpoint simulation targets are:

```bash
fusesoc run --target=verilate_endpoint_top fusesoc:pcie:tb_endpoint_protocol:1.0.0
fusesoc run --target=sim fusesoc:pcie:tb_endpoint_protocol:1.0.0
```

`verilate_endpoint_top` selects Verilator and `sim` selects VCS. Both need the
Python packages `cocotb`, `cocotbext-axi` and `cocotbext-pcie`, which
[`../../requirements.txt`](../../requirements.txt) lists.

The `synth` target of `pcie_endpoint.core` synthesizes `pcie_endpoint_top` in
Vivado for part `xczu7ev-ffvc1156-2-e` with `INTEGRATED_GEN1_PHY` = true and
`MAX_NUM_LANES` = 1. `synth/s1_ooc.tcl` with the unit `endpoint` runs an
out-of-context synthesis from the ordered file list in `synth/endpoint.tcl`,
with the same two parameter values.

## Pass criteria

The Endpoint test passes when all five cocotb tests pass, each with the checks
listed above and no timeout.
