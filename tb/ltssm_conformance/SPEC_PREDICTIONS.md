# LTSSM Ordered Set conformance: spec-derived predictions

**Purpose.** This file is the oracle of `test_ltssm_conformance.py` (target
`verilate_conformance` in `tb_ltssm_conformance.core`). The suite compares the
`ordered_set_o` that `pcie_ltssm_downstream` emits against the values below.
Every expected value is derived from the PCI Express Base Specification,
Revision 2.1, except the ones flagged as config-derived or as RTL encoding
choices. If the DUT disagrees, the DUT is wrong, unless the prediction is a
spec ambiguity flagged below. Do not edit a prediction to match the DUT: the
oracle would then no longer be independent of the RTL.

Section, table and appendix numbers refer to PCIe Base Spec r2.1.

Observation point: `ordered_set_o`, the 128-bit `pcie_ordered_set_t` the FSM
requests from `os_generator`, before the transmit datapath. The DUT is
configured as the Downstream Port / Root Complex (`IS_ROOT_PORT=1`), x1
(`MAX_NUM_LANES=1`), Gen1, `LINK_NUM=1`, `SIM_FAST_LINK=1`. In Configuration
the Downstream Lanes rules apply throughout (the Downstream Lanes
subsections of §4.2.6.3).

---

## Symbol to bit-offset map (codebase encoding, not a spec value)

`pcie_ordered_set_t` is `logic [15:0][7:0] symbols` (`src/packages/pcie_phy_pkg.sv`).
Spec Symbol N is `symbols[N]`, at bit offset 8*N, so Symbol 0 is bits 7:0.
`ltssm_tb_common.unpack_tsos` reads the same offsets (Link Number at bit 8,
Lane Number at bit 16, TS identifier at bits 48 and 80). The suite checks the
orientation itself: COM must be Symbol 0 and the TS identifier Symbols 6-15,
so a reversed layout fails the Symbol 0 check. The offsets are a codebase
fact; the values read at them are checked against the spec below.

| Spec symbol | Field                | bit offset  |
|-------------|----------------------|-------------|
| 0           | COM                  | 0           |
| 1           | Link Number          | 8           |
| 2           | Lane Number          | 16          |
| 3           | N_FTS                | 24          |
| 4           | Data Rate Identifier | 32          |
| 5           | Training Control     | 40          |
| 6-15        | TS1/TS2 Identifier   | 48 to 120   |

---

## A. Ordered Set field encodings (per Ordered Set type)

Byte values of the Special and data Symbols (§4.2.1.2, Table 4-1; byte values
from Appendix B, Tables B-1 and B-2). A Dx.y or Kx.y code is the byte
(y<<5)|x.

| Symbol   | 8b/10b | byte  | Spec ref                      |
|----------|--------|-------|-------------------------------|
| COM      | K28.5  | 0xBC  | Table 4-2/4-3 Symbol 0        |
| PAD      | K23.7  | 0xF7  | Table 4-2/4-3 Symbols 1 and 2 |
| IDL      | K28.3  | 0x7C  | Table 4-4 (EIOS)              |
| TS1 id   | D10.2  | 0x4A  | Table 4-2 Symbols 6-15        |
| TS2 id   | D5.2   | 0x45  | Table 4-3 Symbols 6-15        |

### TS1 Ordered Set (Table 4-2, §4.2.4.1)

- **Symbol 0 = 0xBC** (COM, K28.5). *Spec-confirmed.*
- **Symbol 1 = Link Number**: PAD (0xF7) in Polling; the selected non-PAD
  value in Configuration. The exact non-PAD value (`LINK_NUM=1`) is
  **config-derived**: the spec requires only that it is non-PAD and kept the
  same. *Spec-confirmed: PAD versus non-PAD; value flagged.*
- **Symbol 2 = Lane Number**: PAD (0xF7) until assigned, then a value from 0
  to n-1. §4.2.6.3.2.1 requires the assigned Lane numbers to run from 0 to
  n-1, so at x1 the only Lane number is **0**. *Spec-confirmed for x1: 0x00.*
- **Symbol 3 = N_FTS**: the number of Fast Training Sequences the Receiver
  needs, any value from 0 to 255 (D0.0 to D31.7). The spec does not fix a
  value, so it could not be confirmed (see the list at the end).
- **Symbol 4 = Data Rate Identifier**: bit 0 = 0 (reserved), **bit 1 = 1**
  (2.5 GT/s supported; every device starts training at 2.5 GT/s, §4.2.4.8),
  bits 5:3 = 0 (reserved), bit 7 (speed_change) = 0 outside Recovery. For a
  clean Gen1 link the byte is **0x02**. Bit 2 (5.0 GT/s supported) and bit 6
  (Autonomous Change) depend on what is advertised and on the state, so the
  fixed bits (0, 1, 3, 4, 5, 7) are spec-confirmed, and the full byte 0x02
  assumes a 2.5 GT/s-only advertisement (flagged).
- **Symbol 5 = Training Control**: a normal link-up asserts none of its bits:
  Hot Reset, Disable Link, Loopback, Disable Scrambling and Compliance Receive
  are 0, and the reserved bits 7:5 are 0, so the byte is **0x00**.
  *Spec-confirmed for the clean link-up path.*
- **Symbols 6-15 = 0x4A** (D10.2, TS1 identifier). *Spec-confirmed.*

### TS2 Ordered Set (Table 4-3, §4.2.4.1)

Symbols 0-5 as in TS1 (in Symbol 5 the reserved bits are 7:4 here, and the
byte is still 0x00 on a clean link-up), and **Symbols 6-15 = 0x45** (D5.2,
TS2 identifier). *Spec-confirmed.*

### Idle (Configuration.Idle, §4.2.6.3.6)

In Configuration.Idle the Transmitter sends Idle data Symbols on every
configured Lane. Idle data is the data byte 00h, scrambled and 8b/10b encoded
(§4.2.2), so at 2.5 GT/s it is scrambled zeros on the wire. At the struct
level, before scrambling, the FSM's idle is not a TS Ordered Set.
Prediction: in Configuration.Idle, `ordered_set_o` carries **no TS
identifier** (neither TS1 nor TS2), and the idle control
(`gen_os_ctrl_o.gen_idle`) is asserted. The all-zero struct encoding is an
**RTL representation choice**, flagged; the spec-level fact (not TS1 or TS2,
idle asserted) is what the suite asserts. The suite looks for the identifier
in Symbols 6 and 10, and checks `gen_idle` only where the simulator exposes
that struct field.

### EIOS (Table 4-4): not asserted

The EIOS is COM followed by three IDL at 2.5 GT/s, sent only before the
Transmitter enters Electrical Idle (§4.2.4.2). No transition on the x1 clean
link-up path to L0 sends an EIOS, so this Ordered Set type is documented here
but not exercised by the suite. Exercising it needs a path that enters
Electrical Idle, such as Disabled (§4.2.6.9), the move from L0 to L2
(§4.2.6.5) or Loopback.Exit (§4.2.6.10.3). It is outside the scope of
sections A to C.

---

## B. Per-state sequence counts and the 16-after-1 gating

`SIM_FAST_LINK=1` moves some magnitudes away from the spec: it lowers
Polling.Active's minimum TS1 count (`MinTS1sPolling`) from 1024 to 24 and
divides the 12 ms and 1 ms timeouts by 1000. The 16-after-1 thresholds of
Polling.Configuration, Configuration.Complete and Configuration.Idle stay at
16. The suite does not assert absolute counts. What it tests is the **gating
relationship**, which the spec states independently of the RTL: the exit
Ordered Sets are counted only **after** the first matching Ordered Set is
received.

- **Polling.Configuration to Configuration** (§4.2.6.2.3): the exit needs
  eight consecutive received TS2 with Link and Lane numbers PAD, **and 16 TS2
  transmitted after one TS2 was received**.
- **Configuration.Complete to Configuration.Idle** (§4.2.6.3.5.1): the exit
  needs eight consecutive received TS2 with matching non-PAD Link and Lane
  numbers (and identical Data Rate Identifiers), **and 16 TS2 sent after one
  TS2 was received**.
- **Configuration.Idle to L0** (§4.2.6.3.6): the exit needs Idle data
  received for eight consecutive Symbol Times on every configured Lane, **and
  16 Idle data Symbols transmitted after the first Idle data Symbol is
  received**.

**Prediction (spec):** after the state is entered and before the first
matching Ordered Set is received, the transmit count does **not advance**
toward its threshold. Once one matching Ordered Set is received, counting
begins. A raw count that runs from state entry is a **conformance defect**.

Test method: the suite enters Configuration.Complete (and then
Configuration.Idle), lets `os_tx_pulser` pulse `ordered_set_tranmitted_i` for
60 cycles with the matching receive strobe withheld, and asserts that the
internal `ordered_set_sent_cnt_r` and
`single_ts2_received`/`single_idle_received` are still 0. It then supplies the
receive strobe and asserts that the count starts to advance. The internal
signals are visible through `--public-flat-rw`; the *expected* behaviour is
spec-derived.

---

## C. Configuration.Lanenum.Wait: the 1 ms settle (§4.2.6.3.4.1)

The 1 ms is a **permitted** delay, not a mandatory minimum. §4.2.6.3.4.1
allows a delay of up to 1 ms before the move to Configuration.Lanenum.Accept,
so that receive errors or skew between Lanes do not affect the final
configured Link width. That sentence names the Upstream Lanes, although it
sits in the Downstream Lanes subsection. So:

- **Mandatory (spec-required) gate:** the next state is Lanenum.Accept once a
  Lane that detected a Receiver gets two consecutive TS1 whose **Lane Number
  differs from its value when the Lane entered Lanenum.Wait**, while not all
  Link numbers are PAD. The spec's other exit to Lanenum.Accept is two
  consecutive TS1 on all Lanes whose Link and Lane numbers match the ones
  being transmitted. *Prediction: the DUT does not exit on an unchanged Lane
  Number, and it exits once the Lane Number changes.* This is what the suite
  asserts. Its held stimulus (Link `LINK_NUM`, Lane PAD) matches neither exit,
  since the DUT transmits Lane 0; its changed stimulus (Lane 0) meets both, so
  the suite does not tell the two exits apart.
- **1 ms window:** an *upper-bound allowance* to absorb skew between Lanes,
  **not a required floor**. An implementation that evaluates the changed-Lane
  condition at once (no settle delay) is therefore **spec-compliant**. A
  missing settle window is a robustness gap against multi-Lane skew, not a
  conformance violation, and at x1 (one Lane) there is no skew between Lanes.

The spec does not mandate a settle window. The suite therefore asserts the
*changed-Lane* gate (spec-required) and separately *reports* whether a settle
floor exists (informational, skew robustness). It reports a floor when the
exit comes more than 1000 ns after it changes the received Lane Number; at
`SIM_FAST_LINK=1` the DUT's 1 ms timeout (`OneMsTimeOut`) is also 1000 ns.

---

## Values the spec could not confirm

1. **N_FTS (Symbol 3) exact value**: Table 4-2 (§4.2.4.1) defines it as the
   number of Fast Training Sequences the Receiver needs, any value from 0 to
   255. The spec gives no value. The suite records the values the DUT sends
   and reports them in its summary; it asserts nothing about Symbol 3. The
   number is **RTL-derived, not spec-confirmed.**
2. **Data Rate Identifier full byte (Symbol 4)**: bits 0, 1, 3, 4, 5 and 7 are
   fixed by the spec for a clean Gen1 link (giving 0x02); **bit 2 (5.0 GT/s
   advertised) and bit 6 (Autonomous Change, or de-emphasis) depend on what
   is advertised and on the state.** The suite asserts the fixed bits and
   reports every byte it observes; exactly 0x02 assumes a 2.5 GT/s-only
   advertisement, and a DUT that advertises more shows in the report.
3. **Idle struct encoding (Configuration.Idle)**: the spec requires Idle
   data; the all-zero `pcie_ordered_set_t` plus `gen_idle` representation is
   an **RTL encoding choice**, not a spec-mandated struct. The suite asserts
   the spec-level fact (not a TS Ordered Set; idle asserted), not the zero
   pattern itself.
4. **Link Number value (1)**: the spec requires only a non-PAD value kept the
   same; the value 1 is the `LINK_NUM` parameter, **config-derived.**
