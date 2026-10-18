"""lr1_consts -- every protocol constant, each with its table and page.

Author: Kourosh Ghahramani
Silicon Systems Research Lab, University of Washington

Spec: PCI Express Base Specification Revision 2.1 ("Base 2.1"). The copy on the
shelf is change-marked against 2.0, so table and figure numbers print twice
("Table 2-37--2-28"); constants are cited here by page and table title, and the
PDF page equals the printed page.

Independence (BRIEF_LR1 D4): this file, like every lr1_*.py, imports nothing
from the repository's RTL or from any other bench helper. It is written from
the spec text alone.
"""

# ---------------------------------------------------------------------------
# Symbols -- Base 2.1 p.194, Table 4-1 "Special Symbols". A K code is carried
# here as (byte, k=1); the byte is the 8b/10b data value Kxx.y = (y << 5) | xx.
# ---------------------------------------------------------------------------
COM = 0xBC   # K28.5 Comma
STP = 0xFB   # K27.7 Start TLP
SDP = 0x5C   # K28.2 Start DLLP
END = 0xFD   # K29.7 End
EDB = 0xFE   # K30.7 EnD Bad (nullified TLP)
PAD = 0xF7   # K23.7 Pad
SKP = 0x1C   # K28.0 Skip
FTS = 0x3C   # K28.1 Fast Training Sequence
IDL = 0x7C   # K28.3 Idle (EIOS)
EIE = 0xFC   # K28.7 Electrical Idle Exit (Reserved at 2.5 GT/s)
K_NAMES = {COM: 'COM', STP: 'STP', SDP: 'SDP', END: 'END', EDB: 'EDB',
           PAD: 'PAD', SKP: 'SKP', FTS: 'FTS', IDL: 'IDL', EIE: 'EIE'}

# ---------------------------------------------------------------------------
# Training sets -- Base 2.1 pp.201-203 Table 4-2 "TS1 Ordered Set" and
# pp.203-205 Table 4-3 "TS2 Ordered Set".
#   Symbol 0 COM; 1 Link Number (0-255 or PAD); 2 Lane Number (0-31 or PAD);
#   3 N_FTS; 4 Data Rate Identifier; 5 Training Control; 6-15 TS identifier.
# Inverted identifiers (polarity): §4.2.4.4 p.208.
# ---------------------------------------------------------------------------
TS1_ID = 0x4A        # D10.2
TS2_ID = 0x45        # D5.2
TS1_ID_INV = 0xB5    # D21.5, a TS1 identifier received with inverted polarity
TS2_ID_INV = 0xBA    # D26.5
TS_LEN = 16
# Symbol 4, Data Rate Identifier (Table 4-2): bit 0 Reserved, bit 1 2.5 GT/s,
# bit 2 5.0 GT/s, bits 5:3 Reserved, bit 6 Autonomous Change / Selectable
# De-emphasis, bit 7 speed_change.
RATE_2G5 = 0x02
RATE_BIT_AUTONOMOUS = 0x40
RATE_BIT_SPEED_CHANGE = 0x80
# Symbol 5, Training Control (Table 4-2): bit 0 Hot Reset, 1 Disable Link,
# 2 Loopback, 3 Disable Scrambling, 4 Compliance Receive (5.0 GT/s capable).
TC_HOT_RESET = 0x01
TC_DISABLE = 0x02
TC_LOOPBACK = 0x04
TC_NO_SCRAMBLE = 0x08
TC_COMPLIANCE_RX = 0x10

# EIOS -- Base 2.1 p.205, Table 4-4: COM IDL IDL IDL.
EIOS = ((COM, 1), (IDL, 1), (IDL, 1), (IDL, 1))

# SKP Ordered Set -- Base 2.1 §4.2.7 pp.261-262: COM followed by three SKP on
# transmit; a receiver must accept one to five SKP (§4.2.7.2 p.262).
SKP_TX_COUNT = 3
SKP_RX_MIN, SKP_RX_MAX = 1, 5
# Transmit scheduling, §4.2.7.1 p.261: "The SKP Ordered Set shall be scheduled
# for insertion at an interval between 1180 and 1538 Symbol Times", deferred
# behind a packet or Ordered Set already in progress and then sent back to
# back. A receiver tolerates N = 1538 + (Max_Payload_Size + 28) Symbols between
# SKP Ordered Sets (§4.2.7.2 p.262).
def skp_rx_max_separation(max_payload_bytes):
    return SKP_INTERVAL_MAX + max_payload_bytes + 28
SKP_INTERVAL_MIN = 1180
SKP_INTERVAL_MAX = 1538

# Training-sequence counts, §4.2.6.2.1 p.220 (Polling.Active: "at least 1024
# TS1"), §4.2.6.2.3 p.224 (Polling.Configuration: 8 TS2 received, 16 sent after
# receiving one), §4.2.6.3.5.2 p.236 (Configuration.Complete), §4.2.6.3.6 p.237
# (Configuration.Idle: 8 idle Symbol Times received, 16 sent).
POLLING_ACTIVE_MIN_TS1 = 1024
RX_CONSECUTIVE_TS = 8
TX_AFTER_RX_TS = 16
RX_CONSECUTIVE_IDLE = 8
TX_AFTER_RX_IDLE = 16

# ---------------------------------------------------------------------------
# Data Link Layer -- Base 2.1 p.165, Table 3-1 "DLLP Type Encodings".
# ---------------------------------------------------------------------------
DLLP_ACK = 0x00
DLLP_NAK = 0x10
DLLP_PM_ENTER_L1 = 0x20
DLLP_PM_ENTER_L23 = 0x21
DLLP_PM_AS_REQ_L1 = 0x23
DLLP_PM_REQ_ACK = 0x24
DLLP_VENDOR = 0x30
DLLP_INITFC1 = {'P': 0x40, 'NP': 0x50, 'CPL': 0x60}   # | VC[2:0]
DLLP_INITFC2 = {'P': 0xC0, 'NP': 0xD0, 'CPL': 0xE0}
DLLP_UPDATEFC = {'P': 0x80, 'NP': 0x90, 'CPL': 0xA0}
FC_TYPES = ('P', 'NP', 'CPL')     # the order InitFC must follow, §3.3.1 p.160
DLLP_LEN = 6                      # 4 bytes + 16-bit CRC, §3.4.1 p.164

# DLLP CRC -- Base 2.1 §3.4.1 pp.167-168, Table 3-2: polynomial 100Bh, seed
# FFFFh, data bit 0 of byte 0 first, result complemented and bit-swapped per
# byte into the 16-bit field; byte 4 = field[15:8].
DLLP_CRC_POLY = 0x100B
DLLP_CRC_SEED = 0xFFFF
# LCRC -- Base 2.1 §3.5.2.1 pp.171-173, Table 3-3 and the figure "Calculation of
# LCRC" p.173: polynomial 04C1 1DB7h, seed FFFF FFFFh, over the two sequence
# bytes {4'b0, Seq[11:8]}, Seq[7:0} and the whole TLP, bit 0 of each byte
# first; complemented; byte-wise bit swap; first LCRC byte = field[31:24].
LCRC_POLY = 0x04C11DB7
LCRC_SEED = 0xFFFFFFFF

# Sequence numbers -- §3.5.2.1 p.170 (NEXT_TRANSMIT_SEQ 000h, ACKD_SEQ FFFh),
# p.171 (stall at (NTS - ACKD) mod 4096 >= 2048), §3.5.3.1 p.183 (NEXT_RCV_SEQ).
SEQ_MOD = 4096

# Ack latency, x1 at 2.5 GT/s -- Base 2.1 p.187, Table 3-6, in Symbol Times,
# by Max_Payload_Size. REPLAY_TIMER limits -- pp.175-176, Table 3-4 (unadjusted,
# x1), tolerance -0 / +100 %.
ACK_LATENCY_X1 = {128: 237, 256: 416, 512: 559, 1024: 1071, 2048: 2095, 4096: 4143}
REPLAY_TIMER_X1 = {128: 711, 256: 1248, 512: 1677, 1024: 3213, 2048: 6285, 4096: 12429}

# Flow control -- §3.3.1 pp.160-162: InitFC1 and InitFC2 sets at least once
# every 34 us; §2.6.1.2 pp.142-143: UpdateFC for each non-infinite type at least
# every 30 us (-0 / +50 %) in L0.
FC_INIT_RESEND_US = 34
UPDATEFC_PERIOD_US = 30
# Endpoint minimum initial advertisements -- Base 2.1 p.137, Table 2-37
# "Minimum Initial Flow Control Advertisements": PH 1, PD = max payload / 16,
# NPH 1, NPD 1; CplH and CplD infinite (0) for an Endpoint (p.138).
MIN_PH, MIN_NPH, MIN_NPD = 0x01, 0x01, 0x01
def min_pd(max_payload_bytes):
    return max_payload_bytes // 16
# Field widths, p.138: header credits 8 bits, data credits 12 bits; 0 = infinite.
FC_HDR_BITS, FC_DATA_BITS = 8, 12

# ---------------------------------------------------------------------------
# Transaction Layer -- Base 2.1 pp.56-58 (Fields Present in All TLP Headers;
# Table 2-2 Fmt; Table 2-3 Fmt and Type), p.80 (Request Header Format for
# Configuration Transactions), p.98 (Completion Header Format; Table 2-28
# Completion Status).
# ---------------------------------------------------------------------------
FMT_3DW = 0b000
FMT_4DW = 0b001
FMT_3DW_D = 0b010
FMT_4DW_D = 0b011
TYPE_MEM = 0b00000
TYPE_IO = 0b00010
TYPE_CFG0 = 0b00100
TYPE_CFG1 = 0b00101
TYPE_CPL = 0b01010
TYPE_MSG_BASE = 0b10000       # | r[2:0]
CPL_SC, CPL_UR, CPL_CRS, CPL_CA = 0b000, 0b001, 0b010, 0b100
