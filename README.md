# pcie_gen1

A fully soft-logic PCIe Gen1 x1 **root complex and endpoint** in synthesizable SystemVerilog, with no
hard PCIe IP. The target board is the Xilinx ZCU102 — synthesis runs out-of-context on ZU7EV
(`xczu7ev-ffvc1156-2-e`), a same-family stand-in for the board's ZU9EG. The stack is part of the
AZilla / AraXL RISC-V vector system at SSRL, University of Washington.

The soft stack starts at the scrambler. Below it, the transceiver layer is the AMD PCIe PHY IP
(PG239) presenting a PIPE interface — planned, not yet integrated.

## What is in the tree

### Root complex

| layer | files |
|---|---|
| Downstream LTSSM | `pcie_ltssm_downstream.sv` |
| Scrambler, 8b/10b, framing | `scrambler.sv`, `gen1_scramble.sv`, `encode_8b10b.sv`, `decode_8b10b.sv`, `frame_symbols.sv` |
| Data Link Layer | `pcie_datalink_layer.sv`, `dllp_handler.sv`, `dllp_transmit.sv`, `dllp_receive.sv`, `retry_management.sv`, `axis_retry_fifo.sv`, `pcie_flow_ctrl_init.sv`, `dllp_fc_update.sv` |
| Transaction Layer | `tlp_layer.sv`, `tlp_parser.sv`, `tlp_generator.sv`, `tlp_requester.sv`, `tlp_completion_generator.sv`, `tlp_credit_manager.sv`, `tlp_request_tracker.sv`, `tlp_vc_buffer.sv` |
| Enumeration engine | `pcie_enum_top.sv`, `pcie_enum_scan.sv`, `pcie_enum_bus.sv`, `pcie_enum_bar.sv`, `pcie_cfg_txn.sv` |
| Configuration space | `pcie_config_reg.sv`, `pcie_config_handler.sv`, `pcie_config_decode.sv`, `pcie_config_mux.sv` |
| AXI-Stream host interface | `pcie_axis_dw_upsize.sv`, `pcie_axis_dw_downsize.sv` |
| Stacked tops | `pcie_rc_dl_top.sv` (TL over DLL), `pcie_enum_dl_top.sv` (enumeration engine over the same stack) |

### Endpoint

`pcie_endpoint_top.sv` integrates the endpoint-side Transaction and Data Link layers behind a packet
PHY interface, with an `INTEGRATED_GEN1_PHY` parameter that pulls the LTSSM, logical PHY, Gen1
scrambler and 8b/10b codec in below the Data Link Layer.

It is exercised by the `verilate_endpoint_top` target in `tb_pcie_endpoint_top.core`, and carries a
`synth` target in `pcie_endpoint.core`.

### Shared between both verticals

The LTSSM, ordered-set generator (`os_generator.sv`), lane management (`lane_management.sv`), block
alignment (`block_alignment.sv`), the TX and RX logical PHY (`phy_transmit.sv`, `phy_receive.sv`) and
the scrambler/codec are common to the root complex and the endpoint.

## Status

`main` is at PR #28. The regression gate is **99 Verilator/cocotb targets, 530 tests, all passing**,
cold-verified from a fresh clone and byte-compared between runs — a change is not considered done
until the gate artifact reproduces. Out-of-context synthesis closes at **125 MHz** on ZU7EV, the Gen1
PIPE clock rate. The fabric 8b/10b codec is proven exhaustively against the specification tables and
is kept as the reference model.

## Provenance

The PHY/LTSSM codebase was inherited from an earlier lab effort; it was audited line-by-line against
PCIe Base 2.1 and reworked during Aug–Sep 2026 (PRs #12–#28). Git history carries authorship.

## Running a target

Activate the `pcie` conda environment so FuseSoC, Verilator and cocotb are on `PATH`:

```bash
export PATH=/home/kourosh/miniconda3/envs/pcie/bin:$PATH
```

Then run one target against its core:

```bash
fusesoc --cores-root . run --target verilate_<target> <core>
# for example
fusesoc --cores-root . run --target verilate_tlp_parser fusesoc:pcie:tb_tlp:1.0.0
```

Target and core names live in the `.core` files under `tb/` and `src/`.

⚠️ **`fusesoc run` returns 0 on a cocotb FAIL, and returns 0 again when zero tests ran** — a stale
build directory yields no test table at all.
**Never judge a run by its exit code: read the `TESTS=` line.** A run that printed no `TESTS=` line
tested nothing; delete that target's build directory and re-run.

⚠️ Run targets **sequentially**. Concurrent runs share a build directory and race.

## Conventions

- Rows marked `expect_fail` record a deliberate divergence from the specification; a passing
  `expect_fail` row prints `STATUS=PASS`, so a red row there is news and a green one is not.
- RTL guards use `$warning`, never `$error` — a procedural `$error` maps to `$stop`, which would
  abort the shared multi-test process, and several tests deliberately trip these guards.
- One behaviour per commit.
- Pull requests are reviewed and land as merge commits; `main` is protected.
- `fusesoc.conf` is per-directory and gitignored, so it never arrives with a clone. Write it fresh
  pointing at the checkout you are in — a copied conf silently builds a *different* checkout.

## Roadmap

1. Completer path — the receive side of the root complex's transaction layer.
2. PHY IP evaluation (PG239) against the current PIPE boundary.
3. GTH transceiver bring-up.
4. Root-complex ↔ endpoint bench, both verticals in one netlist.
5. Full-stack place-and-route.
6. Hardware: loopback, then root complex ↔ endpoint on one chip, then GTH.
7. NVMe SSD enumeration and traffic.
8. x4, then Gen2 and Gen3.

## Specifications

- *PCI Express Base Specification, Revision 2.1* — the citation of record for every bench.
- *AMD PCI Express PHY LogiCORE IP Product Guide* (PG239).
- *UltraScale Architecture GTH Transceivers User Guide* (UG576).
- *ZCU102 Evaluation Board User Guide* (UG1182).
