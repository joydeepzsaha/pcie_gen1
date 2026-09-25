#!/usr/bin/env python3
"""Validate a .vlt before it is ever handed to Verilator.  §63 #7g-1.

Three faults cost this rung a gate abort and two failed verifications, and all
three are statically detectable:
  1. `*` immediately followed by `/` inside a block comment CLOSES it, so the
     rest of the file is lexed as directives.
  2. An apostrophe anywhere the lexer reaches -- pattern or broken-open comment
     -- reads as a based literal and gives `Unterminated string`.
  3. A standalone `//` line is a syntax error; only trailing `//` works.

Why a fault here is never cosmetic (corrected at §63 #7g-3): EVERY Verilator
build in this repository is warnings-fatal.  No core passes -Wno-fatal, and
fatal-on-warning is Verilator's default.  The #7g-1 record said "six cores",
because six were the ones carrying a visible warning when gate 2 died; the
other cores were no less fatal, only quieter.  So a waiver that fails to parse
or stops suppressing a site is a crashed build on every target that elaborates
the site, not a noisier log.  Check before every Verilator run.
"""
import sys, re
p = sys.argv[1]
src = open(p, encoding='utf-8').read()
bad = []
# walk the file tracking comment state, exactly as a lexer would
i, in_block, line = 0, False, 1
while i < len(src):
    c = src[i]
    if c == '\n':
        line += 1; i += 1; continue
    if not in_block and src.startswith('/*', i):
        in_block = True; i += 2; continue
    if in_block and src.startswith('*/', i):
        in_block = False; i += 2; continue
    if in_block:
        i += 1; continue
    # outside a comment
    if src.startswith('//', i):
        # trailing comment: fine. standalone: fatal.
        ls = src.rfind('\n', 0, i) + 1
        if not src[ls:i].strip():
            bad.append(f"{p}:{line}: standalone // comment (a .vlt syntax error)")
        i = src.find('\n', i)
        if i < 0: break
        continue
    if c == "'":
        bad.append(f"{p}:{line}: apostrophe outside a comment -- lexes as a based literal")
    i += 1
if in_block:
    bad.append(f"{p}: unterminated block comment (a '*/' inside comment text closed one early)")
for b in bad:
    print("WAIVER_INVALID:", b)
print(f"checked {p}: {len(bad)} fault(s)")
sys.exit(1 if bad else 0)
