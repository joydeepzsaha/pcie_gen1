# pcie_gen1

A PCI Express Gen1 (2.5 GT/s) Root Complex and Endpoint in synthesizable SystemVerilog. Both are
built from the same Transaction Layer, Data Link Layer, LTSSM and logical Physical Layer modules;
no module of either instantiates an AMD integrated block for PCI Express. On the ZCU102 evaluation
board the Root Complex reaches a GTH transceiver through the AMD PCI Express PHY IP (PG239), which
supplies 8b/10b encoding and decoding, the receive elastic buffer and the transceiver. The Endpoint
has no board top; its `synth` target and the out-of-context scripts in `synth/` build it with its
integrated Gen1 PHY.

Most benches are Verilator and cocotb targets run by FuseSoC; the `tb/gth/` benches run in the
Vivado simulator. The synthesis scripts and the board files are for Vivado.

## What is in the tree

### Root Complex

| Path | Contents |
|---|---|
| `src/rc/` | The Root Complex layer: the enumeration engine (`pcie_enum_top`, `pcie_enum_scan`, `pcie_enum_bus`, `pcie_enum_bar`, `pcie_cfg_txn`, `pcie_enum_pkg`), the PG213-style host interfaces (`pcie_rq_if`, `pcie_rc_if`, `pcie_cq_if`, `pcie_cc_if`, `pcie_rq_rc_pkg`) with their AXI4-Stream width converters (`pcie_axis_dw_upsize`, `pcie_axis_dw_downsize`), the tops below, and `rc_core.core`. |
| `src/rc/pcie_rq_rc_top.sv` | The Transaction Layer (`tlp_layer`) behind the four PG213-style interfaces. |
| `src/rc/pcie_rc_dl_top.sv` | `pcie_rq_rc_top` on the Data Link Layer (`pcie_datalink_layer`). |
| `src/rc/pcie_enum_dl_top.sv` | The enumeration engine on `pcie_rc_dl_top`. |
| `src/rc/pcie_rc_top.sv` | The enumeration engine, `pcie_rq_rc_top` and `pcie_phy_top` (Data Link Layer, LTSSM and logical PHY), ending on a PIPE interface before 8b/10b encoding. |
| `src/rc/pcie_rc_gth_top.sv` | `pcie_rc_top` on `pg239_gen1_x1`, an instance of the PG239 PHY IP: one lane at 2.5 GT/s, 16 data bits and 2 K flags on the PIPE. No `.core` file lists it; it needs the generated IP, the AMD primitives `IBUFDS_GTE4` and `BUFG_GT`, and the XPM macro `xpm_cdc_async_rst`. |
| `rc_top.core` | The core for `pcie_rc_top`, with a lint-only target (see Lint below). |

### Endpoint

| Path | Contents |
|---|---|
| `src/pcie_endpoint/` | `pcie_endpoint_top`: `tlp_layer` over `pcie_datalink_layer`. Its parameter `INTEGRATED_GEN1_PHY` selects what lies below the Data Link Layer: at 0, the default, packet streams that the protocol-level benches drive; at 1, a Gen1 logical PHY (`phy_receive`, `phy_transmit`, `pcie_ltssm_downstream`) with an 8b/10b encoder and decoder. `pcie_endpoint.core` has a Vivado `synth` target for the variant at 1. |
| `src/pcie_cfg/` | The configuration space: `pcie_cfg_wrapper` with `pcie_config_decode`, `pcie_config_handler` and `pcie_config_mux`, and the register block `pcie_config_reg.sv` and `pcie_config_reg_pkg.sv`, which carry the PeakRDL-regblock generator banner (see `update_rdl.sh` below). `dllp_receive` instantiates `pcie_cfg_wrapper`, so every `pcie_datalink_layer` holds one, the Root Complex's included. |

### Layers both use

| Path | Contents |
|---|---|
| `src/pcie_phy_core/` | The logical Physical Layer: `phy_transmit` (`frame_symbols`, `os_generator`, `lane_management`), `phy_receive` (`ordered_set_handler`, `data_handler`, `block_alignment`, `pack_data`), and `pcie_phy_top`, which puts `pcie_datalink_layer`, the LTSSM, `phy_receive` and `phy_transmit` behind one PIPE port for `pcie_rc_top`. No module instantiates `synchronous_fifo` or `synchronous_lifo`, and only `synchronous_lifo` instantiates `lfsr`. |
| `src/scrambler/` | The Gen1 scrambler (`scrambler`, `gen1_scramble`, `byte_scramble`) and the 8b/10b encoder and decoder (`encode_8b10b`, `decode_8b10b`), which `pcie_endpoint_top` instantiates and `pcie_rc_top` does not. No module instantiates `gen3_scramble`. |
| `src/ltssm/` | `pcie_ltssm_downstream`, the LTSSM that both `pcie_phy_top` and `pcie_endpoint_top` instantiate. |
| `src/dllp/` | The Data Link Layer for VC0, `pcie_datalink_layer`: `dllp_transmit` (`tlp2dllp`, `retry_management`, `retry_transmit`, `axis_retry_fifo`), `dllp_receive` (`axis_user_demux`, `dllp2tlp`, `dllp_handler`, `dllp_fc_update`), `pcie_datalink_init` and `pcie_flow_ctrl_init`. |
| `src/crc/` | The CRC generators: `pcie_lcrc16` and `pcie_lcrc32` for the LCRC, in `tlp2dllp` and `dllp2tlp`; `pcie_datalink_crc` and `pcie_dllp_crc8` for the DLLP CRC. No module instantiates `pcie_crc8` or `Crc16Gen`. |
| `src/tlp/` | The Transaction Layer for VC0, `tlp_layer`, with `tlp_requester`, `tlp_generator`, `tlp_parser`, `tlp_completion_generator`, `tlp_request_tracker`, `tlp_credit_manager`, `tlp_vc_buffer`, `tlp_control`, `tlp_classifier`, `tlp_validator`, `tlp_bar_decoder`, `tlp_config_decoder`, `tlp_payload_formatter` and `tlp_ecrc`, and the package `tlp_pkg`. |
| `src/packages/` | `pcie_phy_pkg`, `pcie_datalink_pkg` and `pcie_tlp_pkg`. |

### Other source

| Path | Contents |
|---|---|
| `src/converters/` | Converters between AXI4-Stream TLPs and the TLP interface of verilog-pcie. No module instantiates them. |
| `src/bram/` | The RAM modules `bram_dp` and `bram_sp`. No module instantiates them. |
| `src/async_fifo/`, `src/verilog-axis/`, `src/verilog-pcie/` | Git submodules. The design instantiates `async_fifo` (in `pcie_phy_top`) and the verilog-axis modules `axis_async_fifo`, `axis_fifo`, `axis_register` and `axis_arb_mux`; of verilog-pcie, only the converters instantiate `pcie_tlp_fifo`. |
| `update_rdl.sh` | Regenerates `pcie_config_reg.sv` and `pcie_config_reg_pkg.sv` in `src/pcie_cfg/` from `pcie_config.rdl` with PeakRDL-regblock, which `requirements.txt` pins. |
| `requirements.txt` | The Python packages, pinned. |

### Board, benches and scripts

| Path | Contents |
|---|---|
| `fpga/zcu102/` | The board top `pcie_rc_gth_zcu102` for the ZCU102's `xczu9eg-ffvb1156-2-e`: `pcie_rc_gth_top` with two ILA and two VIO debug cores, which drive and observe it on chip. Its constraints (`pcie_rc_gth_zcu102.xdc`, `pcie_rc_gth_zcu102_debug.xdc`), and Vivado 2023.2 Tcl procedures that create the PG239 IP (`ip_pg239.tcl`) and the debug cores (`ip_debug.tcl`). No generated IP is committed. |
| `tb/tlp/`, `tb/dllp/`, `tb/ltssm/`, `tb/ltssm_conformance/`, `tb/os_generator/`, `tb/pack_data/`, `tb/phy_receive/`, `tb/phy_transmit/`, `tb/phy_tx_golden/`, `tb/scrambler/` | Benches for the modules and layers of the shared directories, each directory with its `.core` file. |
| `tb/rc/` | Root Complex benches, from the width converters up to `pcie_rc_top`. |
| `tb/endpoint/` | The `pcie_endpoint_top` bench. |
| `tb/rc_ep/` | `pcie_enum_dl_top` and `pcie_endpoint_top` in one netlist, joined at the Data Link Layer's PHY-facing AXI4-Stream. |
| `tb/fullstack/` | `pcie_rc_top` and `pcie_endpoint_top` with its integrated Gen1 PHY, joined at the PIPE through the bench's 8b/10b bridge `pipe_codec_bridge`. |
| `tb/gth/` | Vivado simulator (xsim) benches for `pcie_rc_gth_top` and `pcie_rc_gth_zcu102`, with the serial pins looped back. No `.core` file lists them. |
| `tb/probe_7g3/` | Two scripts that build a target from its FuseSoC-staged files with Verilator directly; `run_probe_7g3.sh` also runs it. |
| `tb/base_uvm.py`, `tb/base_uvm.core` | pyuvm base classes that benches in `tb/dllp/` import. |
| `lint/` | `waiver.vlt`, the Verilator waiver file, which the core `fusesoc:pcie:lint` copies into the work root and the targets pass to Verilator; `check_waiver.py`, which reports three lexing faults in a `.vlt` file: a standalone `//` line, an apostrophe outside a comment, and an unterminated block comment; `lint.core`. |
| `synth/` | Vivado out-of-context flows, by default on `xczu7ev-ffvc1156-2-e`. `s1_ooc.tcl`, driven by `run_s1.sh`, synthesizes `pcie_enum_top`, `pcie_rq_rc_top` or the Endpoint (file manifest `endpoint.tcl`), with constraints from `pcie_datalink_layer_constraints.tcl` or `endpoint_constraints.tcl`. `par_ooc.tcl`, driven by `run_par.sh`, places and routes a checkpoint from `s1_ooc.tcl`. `stage_s_ooc.tcl` with `stage_s_ooc.xdc`, driven by `run_stage_s.sh`, synthesizes `pcie_rc_dl_top` or `pcie_enum_dl_top` from the file list that `stage_s_filelist.sh` asks FuseSoC for. The header of each `run_*.sh` driver, of `stage_s_filelist.sh` and of each `*_ooc.tcl` flow gives its usage. |

The directories `src/dllp/`, `src/tlp/` and `src/pcie_endpoint/` each have a README of their own.

## Running a target

`requirements.txt` pins the Python packages for Python 3.12, among them FuseSoC 2.4.6, edalize
0.6.8 and cocotb 1.9.2, and names the tools installed separately: Verilator 5.050 and Vivado 2023.2.
`synth/stage_s_filelist.sh` and `synth/run_stage_s.sh` expect FuseSoC in a conda environment named
`pcie`.

From the repository root:

```bash
git submodule update --init
pip install -r requirements.txt
fusesoc --cores-root . run --target verilate_tlp_parser fusesoc:pcie:tb_tlp:1.0.0
```

Many cores depend on cores in the three submodules, such as `fusesoc:pcie:axis` in
`src/verilog-axis/`, so the submodules must be checked out. Target and core names are in the
`.core` files under `src/` and `tb/`; most bench targets are named `verilate_*`.

Reading the result (FuseSoC 2.4.6, edalize 0.6.8, cocotb 1.9.2):

- The exit status of `fusesoc run` does not report test results. For a target with a
  `cocotb_module`, edalize builds cocotb's own Verilator main program, which returns 0 when the
  simulation ends whatever the tests did, and the sim flow fails only on a non-zero exit status.
- cocotb ends a run with a summary line, `TESTS=<n> PASS=<n> FAIL=<n> SKIP=<n>`, and writes
  `results.xml` in the target's work root, `build/<core>/<target>/`, where `<core>` is the core
  name with every leading `:` dropped and every other `:` replaced by `_` (for example
  `fusesoc_pcie_tb_tlp_1.0.0`, or `rc_top_core_1.0.0` for `::rc_top_core:1.0.0`). Read those.
- A run without that summary line has no result: cocotb prints it only at the end of its
  regression, and omits it when no test ran.
- A test declared with `expect_fail=True` counts as passed when it fails an `assert` (or a
  `pytest.raises` check), and as failed when it passes or ends on any other exception. The
  benches use it for behaviour the design is known to lack.
- FuseSoC keeps the work root of a `flow:` target between runs; `fusesoc run --clean` empties it
  first.
- FuseSoC also reads a `fusesoc.conf` in the current directory, and `.gitignore` keeps `*.conf` out
  of the repository. `fusesoc library add` records a local library by its absolute path, so a
  `fusesoc.conf` copied from another checkout points at that checkout's cores. A core found under
  `--cores-root` replaces a core of the same name from those libraries, with a warning.

### Lint

```bash
fusesoc --cores-root . run --build --target=lint ::rc_top_core
```

This target elaborates `pcie_rc_top` with Verilator in lint-only mode, with `lint/waiver.vlt`. In
that mode edalize 0.6.8's sim flow has nothing to run, so a run without `--build` fails after a
clean lint.

### Synthesis

The drivers in `synth/` stop unless Vivado is on `PATH`. `run_stage_s.sh` also stops if a conda
environment is active (`CONDA_PREFIX` set), and reads the file list that `stage_s_filelist.sh`
writes from a shell with FuseSoC. For the board, `fpga/zcu102/ip_pg239.tcl` and `ip_debug.tcl`
define Tcl procedures that create the PG239 IP and the debug cores in a Vivado 2023.2 project for
`xczu9eg-ffvb1156-2-e`. No script in the tree writes a bitstream.

## Specifications

- *PCI Express Base Specification, Revision 2.1*
- *PCI Local Bus Specification, Revision 3.0*
- *PCI Express PHY LogiCORE IP Product Guide* (PG239, v1.0)
- *UltraScale+ Devices Integrated Block for PCI Express Product Guide* (PG213, v1.3)
- *UltraScale Architecture GTH Transceivers User Guide* (UG576, v1.7.1)
- *ZCU102 Evaluation Board User Guide* (UG1182, v1.7)

## Provenance

This repository grew from an upstream project whose author is Idris Somoye:

- https://github.com/isomoye/pcie_datalink_layer, author Idris Somoye: the first commit of this
  repository's history is its first commit.
- https://github.com/isomoye-msu/pcie_datalink_layer, author Idris Somoye, with commits by Reece
  Winmond: this repository's history contains its history up to commit `4b4779c`.

Who wrote what, by area:

| Area | Directories | Authorship |
|---|---|---|
| Physical Layer | `src/pcie_phy_core/`, `src/scrambler/`, `src/ltssm/` | Original author: Idris Somoye |
| Data Link Layer | `src/dllp/`, `src/crc/` | Original author: Idris Somoye |
| Endpoint configuration space | `src/pcie_cfg/` | Original author: Idris Somoye |
| Transaction Layer | `src/tlp/` | Original author: Joydeep Saha |
| Endpoint top | `src/pcie_endpoint/` | Original author: Joydeep Saha |
| Root Complex layer and its tops | `src/rc/`, `rc_top.core` | Author: Kourosh Ghahramani |
| Board files | `fpga/` | Author: Kourosh Ghahramani |

Third-party code in these directories, with its authors, is listed in the THIRD_PARTY_NOTICES file
at the repository root.

Kourosh Ghahramani, Silicon Systems Research Lab, University of Washington, is the author of the
Root Complex layer, of the tops that assemble it with the shared layers, of the board files, and of
most of the verification: the benches that join the Root Complex and the Endpoint (`tb/rc_ep/`,
`tb/fullstack/`) and most of the other benches under `tb/`. The shares below include his changes to
the inherited layers.

Share of lines by author, from `git blame -w -M` at tag `pre-cleanup` (commit `a056c9c`), before the
comments of this tree were rewritten. They cover the 332 files of that commit that this tree still
has (the submodules excluded), each file weighted by its line count, rounded to whole percent:

| Area (line share by author) | Files | Lines | Kourosh Ghahramani | Idris Somoye | Joydeep Saha |
|---|---:|---:|---:|---:|---:|
| Whole tree | 332 | 115,816 | 70 % | 19 % | 11 % |
| Root Complex: `src/rc/`, `rc_top.core` | 20 | 9,290 | 100 % | 0 % | 0 % |
| Board: `fpga/` | 5 | 624 | 100 % | 0 % | 0 % |
| Physical Layer: `src/pcie_phy_core/`, `src/scrambler/`, `src/ltssm/` | 25 | 8,671 | 18 % | 82 % | 0 % |
| Data Link Layer: `src/dllp/`, `src/crc/` | 25 | 6,335 | 14 % | 71 % | 15 % |
| Transaction Layer: `src/tlp/` | 18 | 3,323 | 24 % | 0 % | 76 % |
| Endpoint configuration space: `src/pcie_cfg/` | 8 | 4,402 | 1 % | 98 % | 1 % |
| Endpoint top: `src/pcie_endpoint/` | 3 | 1,042 | 10 % | 0 % | 90 % |
| Benches: `tb/` | 199 | 77,884 | 86 % | 5 % | 9 % |

Reece Winmond is the author of the remaining 0.2 % of the whole tree, in `.gitignore` and six files
under `tb/`.

## Licence

The README of the upstream project `isomoye-msu/pcie_datalink_layer` ends with this License section:

> This project is licensed under the MIT License. See the LICENSE file for details.

That statement is the upstream project's. Neither this repository nor either upstream repository has
a LICENSE file at its root. Third-party code in this repository, with its licence and notice, is
listed in the THIRD_PARTY_NOTICES file at the repository root.
