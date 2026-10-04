# PCIe Data Link Layer and DLLP Logic

## Purpose

This directory implements the PCIe Data Link Layer for VC0. Although the
directory is named `dllp`, it handles both Data Link Layer Packets (DLLPs) and
the sequence numbers, LCRC and retry buffer that protect Transaction Layer
Packets (TLPs).

The top module is [`pcie_datalink_layer.sv`](pcie_datalink_layer.sv).
`pcie_endpoint_top` (`src/pcie_endpoint`), `pcie_rc_dl_top` (`src/rc`) and
`pcie_phy_top` (`src/pcie_phy_core`) instantiate it.
[`docs/dllp_handler.md`](docs/dllp_handler.md) describes the parameters and
ports of `dllp_handler`.

## Main responsibilities

The Data Link Layer:

- Tracks the Data Link Layer state from Physical LinkUp and holds its flow
  control, transmit and receive blocks in reset while the link is down.
- Transmits the InitFC1 and InitFC2 DLLPs of flow control initialization and
  stores the credits the peer advertises.
- Separates received TLPs from DLLPs by `tuser`.
- Checks the peer's credits for each outgoing TLP, then adds a 12-bit
  sequence number in front of it and a 32-bit LCRC behind it.
- Checks the sequence number and LCRC of each received TLP and requests the Ack
  or Nak the result calls for; a nullified TLP, or a bad one while a Nak is
  already scheduled, gets neither.
- Transmits Ack and Nak DLLPs, and UpdateFC-P and UpdateFC-NP DLLPs for the
  receive credits.
- Keeps each transmitted TLP in a retry buffer until an Ack or Nak covers it,
  and replays it after a Nak or a REPLAY_TIMER expiry.
- Requests a Link retrain when REPLAY_NUM rolls over, and holds the replay
  until retraining completes.
- Checks the 16-bit CRC of each received DLLP and decodes Ack, Nak, InitFC1,
  InitFC2, UpdateFC and Feature_Exchange DLLPs.
- Answers CfgRd0 and CfgWr0 itself: `dllp_receive` passes each good TLP
  through `pcie_cfg_wrapper` (`src/pcie_cfg`), which completes them and still
  forwards them on `m_tlp_axis_*`.
- Merges the DLLP and TLP streams toward the Physical Layer.

## RTL components

| Module | Core | Responsibility |
| --- | --- | --- |
| `pcie_datalink_layer.sv` | `dllp_core.core` | Top level: connects the blocks below and merges the TLPs to send and the streams toward the Physical Layer. |
| `pcie_datalink_init.sv` | `dllp_core.core` | Data Link Control and Management State Machine: `ST_DL_INACTIVE`, `ST_DL_INIT`, `ST_DL_INIT_FC1`, `ST_DL_INIT_FC2` and `ST_DL_ACTIVE`. |
| `pcie_flow_ctrl_init.sv` | `dllp_core.core` | Transmits the InitFC1 and InitFC2 DLLP sets for VC0, then one UpdateFC-P and UpdateFC-NP pair. |
| `dllp_receive.sv` | `dllp_receive.core` | Receive path: `axis_user_demux`, `dllp_handler`, `dllp2tlp`, `dllp_fc_update` and `pcie_cfg_wrapper`. |
| `axis_user_demux.sv` | `dllp_receive.core` | Routes each received frame by the `tuser` bits of its first beat: TLPs to `dllp2tlp`, DLLPs to `dllp_handler`. |
| `dllp_handler.sv` | `dllp_receive.core` | Checks the CRC of each received DLLP, reports Acks and Naks, and stores the peer's InitFC and UpdateFC credits. |
| `dllp2tlp.sv` | `dllp_receive.core` | Checks framing, LCRC and sequence number of each received TLP, keeps the TLP only if every check passes, requests the Ack or Nak, and counts the receive credits (CREDITS_ALLOCATED). |
| `dllp_fc_update.sv` | `dllp_receive.core` | Transmits the Ack and Nak DLLPs that `dllp2tlp` requests and, in DL_Active, UpdateFC-P and UpdateFC-NP DLLPs. |
| `dllp_transmit.sv` | `dllp_transmit.core` | Transmit path: `tlp2dllp`, `retry_management` and `retry_transmit`, with an arbiter that sends replays ahead of new TLPs. |
| `tlp2dllp.sv` | `dllp_transmit.core` | Checks the peer's credits for each TLP and frames it with its sequence number and LCRC. |
| `retry_management.sv` | `dllp_transmit.core` | Tracks the retry-buffer slots by sequence number, frees the slots an Ack or Nak covers, keeps REPLAY_TIMER and REPLAY_NUM per slot, and requests replays and the Link retrain. |
| `retry_transmit.sv` | `dllp_transmit.core` | Holds the retry buffer, `RETRY_TLP_SIZE` slots, and plays out a slot's frame on request. |
| `axis_retry_fifo.sv` | `dllp_transmit.core` | One retry-buffer slot: stores one framed TLP, which can be played back on request until a new frame overwrites it. |

The modules also use `pcie_datalink_pkg` (`src/packages`), the CRC modules
`pcie_datalink_crc`, `pcie_lcrc16` and `pcie_lcrc32` (`src/crc`), and
`axis_arb_mux`, `axis_register` and `axis_fifo` from the `src/verilog-axis`
git submodule (core `fusesoc:pcie:axis`).

## Interface conventions

`pcie_datalink_layer` carries one DW per beat: `dllp2tlp` and `tlp2dllp`
support only `DATA_WIDTH` = 32.

- `s_tlp_axis_*` accepts TLPs from the Transaction Layer. An arbiter merges
  them with the configuration Completions from `pcie_cfg_wrapper`, and
  `s_tlp_axis_*` wins.
- `m_tlp_axis_*` returns the received TLPs that passed every check.
- `s_phy_axis_*` accepts TLPs and DLLPs from the Physical Layer.
- `m_phy_axis_*` emits DLLPs and framed TLPs toward the Physical Layer. The
  PHY arbiter gives received-path DLLPs (Ack, Nak, UpdateFC) the highest
  priority, then `pcie_flow_ctrl_init`'s DLLPs, then TLPs.
- On both PHY streams, `tuser` bit 0 marks a DLLP and bit 1 a TLP. On a
  received TLP, `tuser` bit 2 on the last beat marks a frame that ended in EDB,
  so `USER_WIDTH` (default 3) must be at least 3.
- `fc_ph_o` to `fc_cpld_o` carry the peer's credits from the last InitFC1,
  InitFC2 or UpdateFC of each type, for the Transaction Layer's transmit
  decisions. `fc_update_valid_o` pulses for each received UpdateFC and for the
  first stored InitFC2 set.
- `fc_initialized_o` is high once `pcie_flow_ctrl_init` has left FC_INIT2 and
  the peer's InitFC2 values are stored.
- `link_retrain_req_o` asks the Physical Layer to retrain the Link;
  `link_retraining_i` reports that the LTSSM is in Recovery or Configuration
  and is 0 where there is no LTSSM.

## Receive alignment in `dllp2tlp`

`dllp2tlp` removes the two-byte sequence prefix and separates the LCRC using
only words that completed a ready/valid handshake. It keeps the last accepted
input word and one assembled TLP Dword, and holds that Dword until the next
accepted beat shows whether it is the last one. The last Dword enters the
frame FIFO with `tuser` all ones, which makes the FIFO drop the whole frame,
if the frame ended in EDB or failed a framing, LCRC or sequence check. Because
only accepted words are stored, idle cycles between the words of a frame do
not change the result. Do not add a look-ahead at an input beat that has not
been accepted: alignment and the LCRC check would then depend on whether the
source pauses inside a frame.

## Testbench organization

The Data Link Layer benches are in [`../../tb/dllp`](../../tb/dllp). Every
target's toplevel is an RTL module, `pcie_datalink_layer` or
`pcie_flow_ctrl_init`, with no SystemVerilog wrapper.

| Core | Target | Simulator | Toplevel | cocotb module |
| --- | --- | --- | --- | --- |
| `fusesoc:pcie:tb_dll_comprehensive:1.0.0` | `verilate_dll_comprehensive` | Verilator | `pcie_datalink_layer` | `test_dll_comprehensive` |
| `fusesoc:pcie:tb_dll_comprehensive:1.0.0` | `measure_7g2_dll` | Verilator | `pcie_datalink_layer` | `test_7g2_dll_timers` |
| `fusesoc:pcie:tb_dllp_core:1.0.0` | `default` | Icarus | `pcie_datalink_layer` | `test_pcie_dllp_core` |
| `fusesoc:pcie:tb_dllp_core:1.0.0` | `sim` | Verilator | `pcie_datalink_layer` | `test_pcie_dllp_core` |
| `fusesoc:pcie:tb_dllp_core:1.0.0` | `verilate_fcinit_spec` | Verilator | `pcie_flow_ctrl_init` | `test_flow_ctrl_init_spec` |

`tb_dllp.core` also has a `synth` target (Vivado, part `xc7a100tcsg324-1`) and
a `lint` target (Verilator, lint only).

### Comprehensive test

[`test_dll_comprehensive.py`](../../tb/dllp/test_dll_comprehensive.py) holds
sixteen tests. `run_test` runs twenty phases on one link:

1. Link-up and flow control initialization.
2. A TLP from `s_tlp_axis` to `m_phy_axis`.
3. Valid TLPs from `s_phy_axis` to `m_tlp_axis`; one of them arrives with three
   idle source cycles between accepted words.
4. Nak on a bad LCRC, a scheduled Nak that only the frame already in
   progress may precede at the PHY arbiter, and receive sequence-number
   errors.
5. Received Ack and Nak, and replay.
6. Malformed TLP and DLLP rejection.
7. UpdateFC handling and zero-credit transmit blocking.
8. Replay-timer retransmission.
9. Corrupt Ack and Nak CRCs, cumulative Ack, ordered multi-TLP replay, and the
   Ack and Nak sequence window boundaries.
10. Nak scheduling suppression and Ack latency.
11. Memory, Completion and Message TLPs with 3 DW and 4 DW headers, a
    maximum-payload TLP and a TLP with ECRC, received unchanged and
    transmitted with consecutive sequence numbers.
12. All six transmit credit counters: exhaustion and release, an UpdateFC with
    non-zero scale fields and a stale UpdateFC that must leave the limits
    unchanged, and the posted-header counter's 8-bit wrap.
13. Retry buffer full and slot reuse.
14. Receive and transmit sequence rollover from `0xFFF` to `0x000`.
15. `m_phy_axis` back-pressure, only when `PCIE_ENABLE_BACKPRESSURE` is set.
16. Link-down while a replay is pending.
17. Replay-timer exhaustion.
18. Repeated-Nak exhaustion.
19. Flow-control classification of received TLPs.
20. Consumed receive credit reaching the advertised UpdateFC.

The other fifteen tests check one rule each: InitFC1 origination and repeat
with a silent peer, the move to InitFC2 once the peer answers, a monotonic
`fc_initialized_o` with and without back-pressure, posted credit release and
its UpdateFC, a periodic UpdateFC per type under traffic, the REPLAY_TIMER
interval, three replays before a retrain request, the retrain handshake, the
REPLAY_TIMER hold while `link_retraining_i` is high, and the REPLAY_NUM restart
for every TLP in the retry buffer after a retrain.

`build_memory_write`, `build_memory_read` and `build_memory_write_64`, and the
FC, Ack and Nak DLLP builders, pack with cocotbext-pcie. `build_raw_tlp` and
`build_raw_dllp` build the other TLPs, among them three Memory Requests of
phase 11, and the raw DLLPs byte by byte. The test models the DLLP CRC itself
and computes the LCRC with zlib's CRC-32. A background monitor queues every
`m_phy_axis` frame. When the test drains that queue (`drain_queue`), it fails
on an Ack or Nak DLLP that no check consumed and on a six-byte frame whose
DLLP CRC is wrong.

### Other test modules

| File | Content |
| --- | --- |
| `test_flow_ctrl_init_spec.py` | Two tests of `pcie_flow_ctrl_init`'s exit from FC_INIT2, with `fc1_values_stored_i` and `fc2_values_stored_i` high and `update_fc_i` low: one with `idle_valid_i` high, one with it low. Each requires the state machine to reach `ST_UPDATE_P` or `ST_FC_COMPLETE` and to assert `fc2_values_sent_o` within 6000 cycles. |
| `test_7g2_dll_timers.py` | Records, edge by edge, the rise of `pcie_flow_ctrl_init`'s `start_flow_control_i`, the `m_phy_axis` frames and `retry_management` events into JSON files, in two cases: a peer that sends nothing during flow control initialization, and a TLP whose Ack never comes. Its assertions check only that each case took place. It imports `TB` and four helpers from `test_dll_comprehensive.py`. |
| `test_pcie_dllp_core.py` | `Fc1Test`, a pyuvm test that waits for `fc_initialized_o`, then runs the cocotbext-pcie Root Complex model's `enumerate()` and one 256-byte `config_read()`. |
| `pcie_base.py`, `pcie_sequences.py` | Subclasses of cocotbext-pcie's `Endpoint` and `Device` models, and the pyuvm sequence item and sequence, which `test_pcie_dllp_core.py` imports. |
| `test_pcie_datalink_layer.py` | Copied by `tb_dllp.core`'s `cocotb` fileset, but no target names it as its cocotb module and no file imports it. |

## Running the tests

From the repository root, with the git submodules checked out
(`git submodule update --init`), FuseSoC 2.4.6 finds the cores with
`--cores-root .`:

```bash
fusesoc --cores-root . run --target=verilate_dll_comprehensive fusesoc:pcie:tb_dll_comprehensive:1.0.0
fusesoc --cores-root . run --target=verilate_fcinit_spec fusesoc:pcie:tb_dllp_core:1.0.0
```

The `default` targets of `dllp_receive.core` and `dllp_transmit.core`, and the
`verilate` target of `dllp_transmit.core`, name the cocotb modules
`test_dllp_recieve` and `test_dllp_transmit`, which are not in the repository.
The `default` target of `dllp_core.core` names `test_pcie_dllp_core` but
copies no Python file; `tb/dllp/tb_dllp.core` lists that file in its `cocotb`
fileset.

## Pass criteria

A target passes when every cocotb test in its module passes. cocotb 1.9.2
writes each test's result to `results.xml`, the default of
`COCOTB_RESULTS_FILE`, in the directory the simulation runs in.

## Scope

These benches simulate `pcie_datalink_layer` or `pcie_flow_ctrl_init` alone,
with no Physical Layer and no LTSSM: `test_dll_comprehensive.py` drives
`phy_link_up_i` and `link_retraining_i` itself. The bench tops
`tb_pcie_endpoint_top` (`tb/endpoint`), `tb_pcie_rc_dl_top`,
`tb_pcie_enum_dl_top` and `tb_pcie_rc_top` (`tb/rc`), `tb_pcie_rc_ep_wrap`
(`tb/rc_ep`), `tb_pcie_fullstack` (`tb/fullstack`), and `tb_pcie_rc_gth` and
`tb_pcie_rc_gth_zcu102` (`tb/gth`) simulate `pcie_datalink_layer` inside larger
tops.
