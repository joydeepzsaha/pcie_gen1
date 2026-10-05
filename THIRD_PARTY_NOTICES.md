# Third-party notices

This file lists third-party sources of code in this repository. For each source it gives
where the code comes from, the licence that the source states, the files here that the source
covers, and the source's notice where the source has one. Each notice in a code block is the
source's text, unchanged.

## 1. 8b/10b encoder and decoder, Chuck Benz

- Source: Chuck Benz, `encode.v` (https://asics.chuckbenz.com/encode.v) and `decode.v`
  (https://asics.chuckbenz.com/decode.v).
- Licence: the source names none. Its notice, below, permits reuse and modification as long as the
  notice is preserved.
- Files: `src/scrambler/encode_8b10b.sv` and `src/scrambler/decode_8b10b.sv`, each of which begins
  with the notice.
- Notice, the first nine lines of both `encode.v` and `decode.v`:

```text
// Chuck Benz, Hollis, NH   Copyright (c)2002
//
// The information and description contained herein is the
// property of Chuck Benz.
//
// Permission is granted for any reuse of this information
// and description as long as this copyright notice is
// preserved.  Modifications may be made as long as this
// notice is preserved.
```

## 2. ccd_register, mcavoya

- Source: `mcavoya/ccd_register` (https://github.com/mcavoya/ccd_register): its `ccd_register.v`
  and `LICENSE`, both on the `master` branch.
- Licence: MIT License.
- Files: `src/pcie_phy_core/synchronous_fifo.sv`, which begins with the notice as `//` comment
  lines. No module in the repository instantiates `synchronous_fifo`.
- Notice, the source's `LICENSE`:

```text
MIT License

Copyright (c) 2019 mcavoya

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
```

## 3. getting-started-with-verilog, Akilesh Kannan

- Source: `aklsh/getting-started-with-verilog`
  (https://github.com/aklsh/getting-started-with-verilog): its `lfsr.v` and `LICENSE`, both on the
  `master` branch.
- Licence: MIT License. The header of the source's `lfsr.v` also says `License: MIT`.
- Files: `src/pcie_phy_core/lfsr.v`, which keeps the header of the source's `lfsr.v` and follows it
  with the notice as `//` comment lines. Only `synchronous_lifo` instantiates `lfsr`, and no module
  in the repository instantiates `synchronous_lifo`.
- Notice, the source's `LICENSE`:

```text
MIT License

Copyright (c) 2019 Akilesh Kannan

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
```

## 4. cocotbext-pcie, Alex Forencich

- Source: cocotbext-pcie (https://github.com/alexforencich/cocotbext-pcie):
  `tests/pcie/test_pcie.py` at commit `63a05dcede` and `tests/pcie_ptile/test_pcie_ptile.py` at
  commit `6815305df8`.
- Licence: MIT. Each source file gives the MIT License permission text in its module docstring,
  without naming the licence.
- Files: `tb/dllp/pcie_sequences.py`, from `test_pcie_ptile.py`, with that file's notice as its
  module docstring; `tb/dllp/test_pcie_dllp_core.py`, from both files, whose module docstring gives
  both copyright lines and the permission text once.
- Notice of `tests/pcie/test_pcie.py` at `63a05dcede`:

```text
Copyright (c) 2020 Alex Forencich

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
```

- Notice of `tests/pcie_ptile/test_pcie_ptile.py` at `6815305df8`:

```text
Copyright (c) 2022 Alex Forencich

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
```

## 5. verilog-axis test template, Alex Forencich

- Source: verilog-axis (https://github.com/alexforencich/verilog-axis),
  `tb/axis_broadcast/test_axis_broadcast.py` at commit `073d50d9dc`.
- Licence: MIT. The source file gives the MIT License permission text in its module docstring,
  without naming the licence.
- Files: `tb/ltssm/test_ltssm_configuration.py`, with the notice as its module docstring, and
  `tb/ltssm/pcie_ltssm/tb/test_ltssm_configuration.py`, a symbolic link to it.
- Notice:

```text
Copyright (c) 2021 Alex Forencich

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
```

## 6. HAL-O file header, Morgan State University

- Source: the file header of the Open Hardware Acceleration Lab (HAL-O), Morgan State University.
  The copy below is the header of `tb/dllp/test_pcie_datalink_layer.py` at the parent of commit
  `1055fa5` in this repository's history.
- Licence: as the header states it, `License: MIT License`.
- Files: `tb/dllp/test_pcie_datalink_layer.py` begins with this header. The same header, differing
  only in its `File` line, heads `tb/base_uvm.py`, `tb/dllp/pcie_base.py`,
  `tb/dllp/pcie_sequences.py`, `tb/dllp/test_pcie_dllp_core.py` and
  `tb/ltssm/test_ltssm_configuration.py`. `src/dllp/axis_retry_fifo.sv` carries a SystemVerilog
  version of it, with its own `File`, `Author` and `Created` lines.
- Notice:

```text
# ==========================================================================
#
#  Morgan State University
#  Open Hardware Acceleration Lab (HAL-O)
#
#!  Project:   Open-Source PCIe Endpoint Controller.
#   File:      test_pcie_datalink_layer.py
#   Author:    HAL-O
#   Created:   10/1/25
#
#!  Description:
#!   Module implements a retry management FIFO. Stores TLPs as axis frames.
#!   Module resets read and write pointer after every frame allowing for retransmission
#!   as long as data is not overwritten.
#
#
#   Project:
#     This file is part of the PCIe Gen1/Gen2 Endpoint Controller project.
#     Developed as an open-source, synthesizable Verilog RTL IP core, this
#     project provides FPGA designers and researchers with an educational
#     and extensible platform for high-speed interconnect design.
#
#   Institutional Acknowledgement:
#    - Project oversight and research guidance provided by the CEAMLS
#      (Center for Equitable AI & Machine Learning Systems) Director.
#
#   Notes:
#     - Compliant with PCIe Base Specification (Gen1: 2.5 GT/s,
#       Gen2: 5.0 GT/s).
# 
#   License: MIT License
# 
# ==========================================================================
```

## 7. pci_express_crc, labelled GPL

- Source: `crc32d16.v` of the GitHub repository `freecores/pci_express_crc`, on its `master`
  branch: https://raw.githubusercontent.com/freecores/pci_express_crc/master/crc32d16.v.
- Licence: the header of `crc32d16.v` says "This code adheres to the GNU public license" and names
  no version.
- Files: `src/crc/pcie_lcrc16.sv`, which carries no notice of `crc32d16.v`. Its 32 XOR equations
  are those of `crc32d16.v`, term for term, with `d` named `data` and `crc` named `crcIn`. The Data
  Link Layer uses it: `tlp2dllp` and `dllp2tlp` each instantiate it as `tlp_crc16_inst`.
- Notice, the first ten lines of `crc32d16.v`:

```text
// ===========================================================================
// File    : crc32d16N.v
// Author  : cwinward
// Date    : Sat Dec 8 14:00:37 MST 2007
// Project : TI PHY design
//
// Copyright (c) notice
// This code adheres to the GNU public license
//
// ===========================================================================
```

## 8. Files that no module instantiates: GPL-3.0 and CC BY-SA 4.0 sources

### 8.1 bram_sync_dp and bram_sync_sp, Wesley New

- Source: the GitHub gist 3952584 of `wnew`, `bram_sync_dp.v` and `bram_sync_sp.v`:
  https://gist.githubusercontent.com/wnew/3952584/raw/e2f7cce6cfc0e0c15541113cf6e07c8c50a5e415/bram_sync_dp.v
  and
  https://gist.githubusercontent.com/wnew/3952584/raw/82aa5e27cf949d0957f0b5e8ee2f8ac69407ca91/bram_sync_sp.v.
- Licence: the header of each file says `Licence: GNU General Public License ver 3`.
- Files: `src/bram/bram_dp.sv`, from `bram_sync_dp.v`, and `src/bram/bram_sp.sv`, from
  `bram_sync_sp.v`. Neither carries the notice. `src/bram/bram.core` lists both, and no module in
  the repository instantiates either.
- Notice of `bram_sync_dp.v`, its header:

```text
//============================================================================//
//                                                                            //
//      Syncronous dual-port BRAM                                             //
//                                                                            //
//      Module name: bram_sync_dp                                             //
//      Desc: parameterized, syncronous, inferable, true dual-port,           //
//            dual clock block ram                                            //
//      Date: Dec 2011                                                        //
//      Developer: Wesley New                                                 //
//      Licence: GNU General Public License ver 3                             //
//      Notes: Developed from a combiniation of bram implmentations           //
//             This is a read-before-write implementation of a BRAM           //
//                                                                            //
//============================================================================//
```

- Notice of `bram_sync_sp.v`, its header:

```text
//============================================================================//
//                                                                            //
//      Syncronous single-port BRAM                                           //
//                                                                            //
//      Module name: bram_sync_sp                                             //
//      Desc: parameterized, syncronous, single-port block ram                //
//      Date: Dec 2011                                                        //
//      Developer: Wesley New                                                 //
//      Licence: GNU General Public License ver 3                             //
//      Notes: Developed from a combiniation of bram implmentations           //
//                                                                            //
//============================================================================//
```

### 8.2 lifo, Konstantin Pavlov

- Source: `pConst/basic_verilog` (https://github.com/pConst/basic_verilog): its `lifo.sv`
  (https://raw.githubusercontent.com/pConst/basic_verilog/master/lifo.sv) and its README, both on
  the `master` branch.
- Licence: CC BY-SA 4.0, which the Licensing section of the project's README writes as
  `CC BY-SA 4_0`.
- Files: `src/pcie_phy_core/synchronous_lifo.sv`. After its `endmodule` it carries a
  commented-out, modified copy of `lifo.sv`, which begins with the four lines quoted below; it
  carries no licence statement. `src/pcie_phy_core/phy_transmit.core` lists it, and no module in
  the repository instantiates it.
- Notice of `lifo.sv`, its first four lines:

```text
//------------------------------------------------------------------------------
// lifo.sv
// Konstantin Pavlov, pavlovconst@gmail.com
//------------------------------------------------------------------------------
```

- The Licensing section of the project's README:

```text
Licensing
---------
The code is licensed under CC BY-SA 4_0   
That means, that you can remix, transform, and build upon the material for any purpose, even commercially.   
However, YOU MUST provide the name of the creator and distribute your contributions under the same license as the original.   
```

## 9. Fragments in files that carry no notice of their source

### 9.1 verilog-pcie port declarations, Alex Forencich

- Source: verilog-pcie (https://github.com/alexforencich/verilog-pcie), whose code the submodule
  `src/verilog-pcie` carries: the eight-port TLP interface group `rx_cpl_tlp_*`, for example in
  `rtl/pcie_us_if_rc.v`.
- Licence: the submodule's `COPYING`: the MIT License permission text, copyright holder Alex
  Forencich (2018).
- Files: `src/converters/axis_to_pcie_converter.sv` declares this group as `rx_tlp_*` with the
  same directions, and `src/pcie_cfg/pcie_config_handler.sv` declares it as `rx_tlp_*` with every
  direction reversed; both keep the widths and the port order. Neither file carries a verilog-pcie
  notice.

### 9.2 FuseSoC example comment

- Source: FuseSoC (https://github.com/olofk/fusesoc): the two comment lines of its example core
  file that explain the `default` target and the `&default` YAML anchor.
- Licence: BSD 2-Clause License, copyright holder FuseSoC contributors, as the `LICENSE` file of the
  FuseSoC 2.4.6 distribution states it.
- Files: `tb/base_uvm.core`, above its `default` target. It carries no FuseSoC notice.
- The two comment lines above the `default` target of `lint/lint.core` are byte-identical to
  these two lines of `tb/base_uvm.core`. `lint/lint.core` carries no FuseSoC notice.

## 10. Git submodules

Each submodule carries its own licence file. The URLs are those of `.gitmodules`; the commits are
those this tree pins.

| Path | URL | Pinned commit | Licence file | Instantiated here by |
|---|---|---|---|---|
| `src/async_fifo` | https://github.com/isomoye-msu/async_fifo.git | `2b1c1ea78e8231e26bba1ab9de1397850ee583b9` | `LICENSE`: The MIT License | `pcie_phy_top` (`async_fifo`) |
| `src/verilog-axis` | https://github.com/isomoye-msu/verilog-axis.git | `bd3fa65821b62342af61c2cfd87f9ce3426cf435` | `COPYING`: the MIT License permission text, copyright holder Alex Forencich (2014-2018) | the Data Link Layer, the logical PHY, the configuration space and the converters (`axis_async_fifo`, `axis_fifo`, `axis_register`, `axis_arb_mux`) |
| `src/verilog-pcie` | https://github.com/isomoye-msu/verilog-pcie.git | `e619d46975f48f6fa43c51a7157ce11f0de4407d` | `COPYING`: the MIT License permission text, copyright holder Alex Forencich (2018) | only the converters in `src/converters/` (`pcie_tlp_fifo`) |
