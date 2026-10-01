# --addr-width 12: a Function's configuration space is 4 KB (PCIe Base 2.1 sec 7.2
# p.472). Without it the regblock sizes its address to the RDL's contents -- 9 bits,
# 512 B -- and every offset aliased to (offset & 0x1FF) until sec 63 #7l.
# WARNING: src/pcie_cfg/pcie_config_reg.sv carries four hand edits made after
# generation (5bbe5ae, 1055fa5, c9f1912, 85ca822). Regenerating it reverts all
# four; diff against the tree before committing a regenerated file.
peakrdl regblock src/pcie_cfg/pcie_config.rdl -o src/pcie_cfg --cpuif axi4-lite-flat --addr-width 12
peakrdl c-header src/pcie_cfg/pcie_config.rdl -o src/pcie_cfg/pcie_cfg.h
peakrdl markdown src/pcie_cfg/pcie_config.rdl -o src/pcie_cfg/pcie_cfg.md
