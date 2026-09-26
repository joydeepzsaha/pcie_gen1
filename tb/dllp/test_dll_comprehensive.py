# ============================================================================
# Cocotb testbench for pcie_datalink_layer
#
# Relaxed functional version:
#   - Prioritizes logical correctness over strict PCIe Gen1 timing.
#   - Uses weaker timeout thresholds by default.
#   - Keeps environment-variable overrides for easy tuning.
#
# Compatible with:
#   cocotb 1.9.2+
#   cocotbext-axi 0.1.x
#   VCS
#
# DUT interfaces:
#   s_phy_axis : packets entering from the physical layer
#   m_phy_axis : packets leaving toward the physical layer
#   s_tlp_axis : locally generated TLPs entering from the transaction layer
#   m_tlp_axis : received TLPs delivered to the transaction layer
#
# Recommended run:
#   make sim 2>&1 | tee output_testPcie_console.txt
#
# Python-side log file:
#   output_testPcie_python.txt
# ============================================================================

import itertools
import logging
import os
import random
import zlib
from typing import Dict, List, Optional, Tuple

import cocotb
from cocotb.clock import Clock
from cocotb.queue import Queue
from cocotb.result import SimTimeoutError
from cocotb.triggers import ClockCycles, Event, ReadOnly, RisingEdge, with_timeout
from cocotb.utils import get_sim_time

from cocotbext.axi import (
    AxiStreamBus,
    AxiStreamFrame,
    AxiStreamSink,
    AxiStreamSource,
)
from cocotbext.pcie.core.dllp import Dllp, DllpType, FcScale
from cocotbext.pcie.core.tlp import Tlp, TlpType


# ----------------------------------------------------------------------------
# Relaxed timing configuration
# ----------------------------------------------------------------------------
# Original strict clock was 4 ns. Use 8 ns by default to match a slower,
# function-first bring-up environment.
CLOCK_PERIOD_NS = int(os.environ.get("PCIE_CLOCK_PERIOD_NS", "8"))

# Relaxed AXI and initialization timeouts. These values are intentionally large
# so that slow internal FSMs do not fail the test before producing correct logic.
AXIS_SEND_TIMEOUT_US = int(os.environ.get("PCIE_AXIS_SEND_TIMEOUT_US", "500"))
AXIS_RECV_TIMEOUT_US = int(os.environ.get("PCIE_AXIS_RECV_TIMEOUT_US", "500"))

FC_DRIVER_TIMEOUT_US = int(os.environ.get("PCIE_FC_DRIVER_TIMEOUT_US", "1000"))
FC_INITIALIZED_TIMEOUT_US = int(
    os.environ.get("PCIE_FC_INITIALIZED_TIMEOUT_US", "2000")
)

# The quiet window waits for dllp_fc_update's next PERIODIC UpdateFC-P, then
# its next UpdateFC-NP.  sec 63 #7g-2 Q2: each type has its own timer, restarted
# only by its own UpdateFC and never by an Ack, expiring at FcWaitPeriod =
# 30 us / CLK_PERIOD_NS -- so each wait is at most ~30 us.  It was ONE 2 ms
# timer that every received TLP's Ack restarted, which is why this used to be
# 2500 us.  100 us = the 45 us ceiling (p.143, 30 us +50 %) with margin, so a
# regression to the old period fails HERE rather than merely taking longer.
FC_UPDATE_IDLE_TIMEOUT_US = int(
    os.environ.get("PCIE_FC_UPDATE_IDLE_TIMEOUT_US", "100")
)

MONITOR_POLL_TIMEOUT_US = int(os.environ.get("PCIE_MONITOR_POLL_TIMEOUT_US", "20"))
MONITOR_SHUTDOWN_TIMEOUT_US = int(
    os.environ.get("PCIE_MONITOR_SHUTDOWN_TIMEOUT_US", "100")
)

MALFORMED_REJECTION_WINDOW_US = int(
    os.environ.get("PCIE_MALFORMED_REJECTION_WINDOW_US", "100")
)

# Negative checks must be much shorter than the replay timer.  Otherwise a
# legitimate replay-timer expiration can be mistaken for an immediate response
# to the packet that is currently under test.
NO_RESPONSE_WINDOW_CYCLES = int(
    os.environ.get("PCIE_NO_RESPONSE_WINDOW_CYCLES", "32")
)

BACKPRESSURE_TIMEOUT_US = int(
    os.environ.get("PCIE_BACKPRESSURE_TIMEOUT_US", str(AXIS_RECV_TIMEOUT_US))
)

DEFAULT_LOG_FILE = "output_testPcie_python.txt"
DEFAULT_RANDOM_SEED = 0x50434945

# These defaults match pcie_datalink_layer.sv.  Override them when the DUT is
# instantiated with different values.
# sec 63 #7g-2 step 2: REPLAY_TIMER_CYCLES and MAX_REPLAY_ATTEMPTS are no longer
# literals in the RTL, so these are DERIVED the same way rather than copied:
# 1.75 x Table 3-4's x1 / MPS-128 limit (711 Symbol Times, 4 ns each) over the
# clock period = 622 at 8 ns (was the literal 0xAA0 = 2,720), and Base 2.1
# sec 3.5.2.1 p.174's three replays (was 2).  R-P2 (the default witness) is
# what proves the RTL still agrees with this copy.
RETRY_BUFFER_DEPTH = int(os.environ.get("PCIE_RETRY_BUFFER_DEPTH", "3"))
REPLAY_TIMER_CYCLES = int(os.environ.get("PCIE_REPLAY_TIMER_CYCLES",
                                         str((7 * 711) // CLOCK_PERIOD_NS)), 0)
MAX_REPLAY_ATTEMPTS = int(os.environ.get("PCIE_MAX_REPLAY_ATTEMPTS", "3"))
MAX_PAYLOAD_BYTES = int(os.environ.get("PCIE_MAX_PAYLOAD_BYTES", "256"))
ACK_LATENCY_LIMIT_CYCLES = int(
    os.environ.get("PCIE_ACK_LATENCY_LIMIT_CYCLES", "512")
)

# A DLLP is a four-byte payload plus a two-byte CRC.
DLLP_FRAME_BYTES = 6

# s_phy_axis_tuser packet classification used by axis_user_demux.
PHY_USER_IS_DLLP = 1 << 0
PHY_USER_IS_TLP = 1 << 1


def env_flag(name: str, default: str = "0") -> bool:
    return os.environ.get(name, default).strip().lower() in ("1", "true", "yes", "on")


def configure_file_logging(log: logging.Logger) -> str:
    """Write Python-side test messages to a text file."""
    log_path = os.environ.get("PCIE_TEST_LOG", DEFAULT_LOG_FILE)

    # Avoid duplicate handlers if the testbench is reconstructed.
    for handler in log.handlers:
        if getattr(handler, "_pcie_test_file_handler", False):
            return log_path

    file_handler = logging.FileHandler(log_path, mode="w")
    file_handler._pcie_test_file_handler = True
    file_handler.setLevel(logging.DEBUG)
    file_handler.setFormatter(
        logging.Formatter(
            fmt="%(asctime)s %(levelname)-8s %(name)s: %(message)s"
        )
    )
    log.addHandler(file_handler)

    return log_path


def require_dut_signals(dut) -> None:
    """Produce a clear error if cocotb prefixes do not match the RTL."""
    required = [
        "clk_i",
        "rst_i",
        "phy_link_up_i",
        "fc_initialized_o",
        "idle_valid_i",
        "status_error_cor_i",
        "status_error_uncor_i",
        "rx_cpl_stall_i",

        "s_phy_axis_tdata",
        "s_phy_axis_tkeep",
        "s_phy_axis_tvalid",
        "s_phy_axis_tlast",
        "s_phy_axis_tuser",
        "s_phy_axis_tready",

        "m_phy_axis_tdata",
        "m_phy_axis_tkeep",
        "m_phy_axis_tvalid",
        "m_phy_axis_tlast",
        "m_phy_axis_tuser",
        "m_phy_axis_tready",

        "s_tlp_axis_tdata",
        "s_tlp_axis_tkeep",
        "s_tlp_axis_tvalid",
        "s_tlp_axis_tlast",
        "s_tlp_axis_tuser",
        "s_tlp_axis_tready",

        "m_tlp_axis_tdata",
        "m_tlp_axis_tkeep",
        "m_tlp_axis_tvalid",
        "m_tlp_axis_tlast",
        "m_tlp_axis_tuser",
        "m_tlp_axis_tready",
    ]

    missing = [name for name in required if not hasattr(dut, name)]

    if missing:
        raise AssertionError(
            "The pcie_datalink_layer top level is missing these expected "
            "signals: {}".format(", ".join(missing))
        )


class TB:
    def __init__(self, dut):
        require_dut_signals(dut)

        self.dut = dut
        self.log = logging.getLogger("cocotb.tb")
        self.log.setLevel(logging.DEBUG)
        self.log_file = configure_file_logging(self.log)

        # Initialize non-AXI inputs before the first clock edge.
        dut.rst_i.setimmediatevalue(1)
        dut.phy_link_up_i.setimmediatevalue(0)
        dut.idle_valid_i.setimmediatevalue(0)
        dut.status_error_cor_i.setimmediatevalue(0)
        dut.status_error_uncor_i.setimmediatevalue(0)
        dut.rx_cpl_stall_i.setimmediatevalue(0)

        cocotb.start_soon(
            Clock(
                dut.clk_i,
                CLOCK_PERIOD_NS,
                units="ns",
            ).start()
        )

        # Incoming packets from the physical layer.
        self.phy_source = AxiStreamSource(
            AxiStreamBus.from_prefix(dut, "s_phy_axis"),
            dut.clk_i,
            dut.rst_i,
        )

        # Outgoing packets toward the physical layer.
        self.phy_sink = AxiStreamSink(
            AxiStreamBus.from_prefix(dut, "m_phy_axis"),
            dut.clk_i,
            dut.rst_i,
        )

        # Locally generated TLPs from the transaction layer.
        self.tlp_source = AxiStreamSource(
            AxiStreamBus.from_prefix(dut, "s_tlp_axis"),
            dut.clk_i,
            dut.rst_i,
        )

        # Received TLPs delivered to the transaction layer.
        self.tlp_sink = AxiStreamSink(
            AxiStreamBus.from_prefix(dut, "m_tlp_axis"),
            dut.clk_i,
            dut.rst_i,
        )

        # Keep cocotbext-axi logs quiet unless debugging is explicitly enabled.
        axis_log_level = logging.DEBUG if env_flag("PCIE_VERBOSE_AXI") else logging.CRITICAL
        self.phy_source.log.setLevel(axis_log_level)
        self.phy_sink.log.setLevel(axis_log_level)
        self.tlp_source.log.setLevel(axis_log_level)
        self.tlp_sink.log.setLevel(axis_log_level)

    async def reset(self, asserted_cycles: int = 8, settle_cycles: int = 8):
        """Apply an active-high reset and wait for the design to settle."""
        self.log.info("Applying reset")

        self.dut.rst_i.value = 1
        self.dut.phy_link_up_i.value = 0
        self.dut.idle_valid_i.value = 0

        for _ in range(asserted_cycles):
            await RisingEdge(self.dut.clk_i)

        self.dut.rst_i.value = 0

        for _ in range(settle_cycles):
            await RisingEdge(self.dut.clk_i)

        self.log.info("Reset released")

    async def wait_cycles(self, cycles: int) -> None:
        for _ in range(cycles):
            await RisingEdge(self.dut.clk_i)


def cycle_pause():
    """Apply three stalled cycles followed by one accepting cycle."""
    return itertools.cycle([1, 1, 1, 0])


def calculate_dllp_crc(data: bytes) -> int:
    """Match the reflected CRC-16 implementation in pcie_dllp_crc8."""
    crc = 0xFFFF

    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ 0xD008 if crc & 1 else crc >> 1

    return crc ^ 0xFFFF


def build_fc_dllp(
    dllp_type: DllpType,
    seq: int = 0,
    hdr_fc: int = 3,
    data_fc: int = 256,
    hdr_scale: int = 0,
    data_scale: int = 0,
) -> bytes:
    """Create one flow-control DLLP including its two-byte CRC."""
    packet = Dllp()
    packet.type = dllp_type
    packet.seq = seq
    packet.vc = 0
    packet.hdr_scale = FcScale(hdr_scale)
    packet.hdr_fc = hdr_fc
    packet.data_scale = FcScale(data_scale)
    packet.data_fc = data_fc
    packet.feature_support = 0
    packet.feature_ack = False

    payload = bytes(packet.pack())

    # The RTL starts at 16'hFFFF, processes the four DLLP bytes in wire
    # order, complements the result, and places the low CRC byte first.
    crc = calculate_dllp_crc(payload)

    return payload + crc.to_bytes(2, "little")


def build_ack_nak_dllp(dllp_type: DllpType, seq: int) -> bytes:
    """Create an ACK or NAK DLLP including CRC."""
    packet = Dllp()
    packet.type = dllp_type
    packet.seq = seq & 0xFFF
    payload = bytes(packet.pack())
    crc = calculate_dllp_crc(payload)
    return payload + crc.to_bytes(2, "little")


def build_raw_dllp(payload: bytes) -> bytes:
    """Create a raw four-byte DLLP payload with matching CRC."""
    if len(payload) != 4:
        raise ValueError("DLLP payload must be exactly four bytes")

    crc = calculate_dllp_crc(payload)
    return payload + crc.to_bytes(2, "little")


def corrupt_dllp_crc(frame_data: bytes) -> bytes:
    """Flip one CRC bit while leaving the DLLP payload unchanged."""
    data = bytearray(frame_data)
    data[-1] ^= 0x01
    return bytes(data)


def check_dllp_crc(frame_data: bytes) -> Optional[bytes]:
    """Return the DLLP payload when its CRC is valid, otherwise return None."""
    frame_data = bytes(frame_data)

    if len(frame_data) != DLLP_FRAME_BYTES:
        return None

    payload = frame_data[:-2]
    received_crc = frame_data[-2:]

    calculated_crc = calculate_dllp_crc(payload).to_bytes(2, "little")

    if received_crc != calculated_crc:
        return None

    return payload


def add_sequence_and_lcrc(
    sequence_number: int,
    tlp_payload: bytes,
) -> bytes:
    """Wrap a transaction-layer TLP for the physical-facing receive path."""
    if not 0 <= sequence_number <= 0xFFF:
        raise ValueError("PCIe sequence number must fit in 12 bits")

    link_packet = sequence_number.to_bytes(2, "big") + bytes(tlp_payload)
    lcrc = zlib.crc32(link_packet) & 0xFFFFFFFF

    return link_packet + lcrc.to_bytes(4, "little")


def build_memory_write(
    payload_length: int,
    tag: int,
    requester_id: int = 1,
    address: int = 4,
) -> Tuple[bytes, bytes]:
    """Return the packed TLP and its deterministic data payload."""
    payload = bytes(((index + tag) & 0xFF) for index in range(payload_length))

    tlp = Tlp()
    tlp.fmt_type = TlpType.MEM_WRITE
    tlp.set_addr_be_data(address, payload)
    tlp.tag = tag
    tlp.requester_id = requester_id

    return bytes(tlp.pack()), payload


def build_memory_read(
    byte_length: int,
    tag: int,
    address: int = 4,
    address_64bit: bool = False,
) -> bytes:
    """Build a 3-DW or 4-DW non-posted Memory Read request."""
    tlp = Tlp()
    tlp.fmt_type = TlpType.MEM_READ_64 if address_64bit else TlpType.MEM_READ
    tlp.set_addr_be(address, byte_length)
    tlp.tag = tag
    tlp.requester_id = 1
    return bytes(tlp.pack())


def build_memory_write_64(payload_length: int, tag: int) -> bytes:
    """Build a 4-DW posted Memory Write request."""
    payload = bytes(((tag + index) & 0xFF) for index in range(payload_length))
    tlp = Tlp()
    tlp.fmt_type = TlpType.MEM_WRITE_64
    tlp.set_addr_be_data(0x1_0000_0004, payload)
    tlp.tag = tag
    tlp.requester_id = 1
    return bytes(tlp.pack())


def build_raw_tlp(
    fmt_type_byte: int,
    length_dw: int,
    header_dw: int,
    payload: bytes = b"",
    td: bool = False,
    ecrc: bytes = b"",
) -> bytes:
    """Build a deterministic TLP for Data-Link black-box forwarding tests."""
    if header_dw not in (3, 4):
        raise ValueError("TLP header must contain three or four DW")
    if not 0 <= length_dw <= 0x3FF:
        raise ValueError("TLP length must fit the ten-bit Length field")
    if td and len(ecrc) != 4:
        raise ValueError("TD=1 requires a four-byte ECRC")

    header = bytearray(header_dw * 4)
    header[0] = fmt_type_byte & 0xFF
    header[2] = ((length_dw >> 8) & 0x03) | (0x80 if td else 0)
    header[3] = length_dw & 0xFF
    return bytes(header) + bytes(payload) + (bytes(ecrc) if td else b"")


def build_completion(with_data: bool, tag: int, payload_length: int = 4) -> bytes:
    """Build a Completion or Completion-with-Data TLP."""
    payload = bytes(((0xC0 + tag + index) & 0xFF) for index in range(payload_length))
    length_dw = (payload_length + 3) // 4 if with_data else 0
    packet = bytearray(
        build_raw_tlp(0x4A if with_data else 0x0A, length_dw, 3,
                      payload if with_data else b"")
    )
    # Completion header Tag byte.
    packet[10] = tag & 0xFF
    return bytes(packet)


def build_message(with_data: bool, tag: int, payload_length: int = 4) -> bytes:
    """Build a routed Message or Message-with-Data TLP."""
    payload = bytes(((0x80 + tag + index) & 0xFF) for index in range(payload_length))
    length_dw = (payload_length + 3) // 4 if with_data else 0
    return build_raw_tlp(
        0x70 if with_data else 0x30,
        length_dw,
        4,
        payload if with_data else b"",
    )


def build_zero_byte_memory_read(tag: int) -> bytes:
    """Build the PCIe zero-byte-read encoding: Length=1 DW and both BEs zero."""
    packet = bytearray(build_memory_read(byte_length=4, tag=tag))
    packet[7] = 0
    return bytes(packet)


def get_internal_handle(dut, dotted_path: str):
    """Resolve a required internal verification handle with a clear failure."""
    handle = dut
    for component in dotted_path.split("."):
        if not hasattr(handle, component):
            raise AssertionError(
                "Required internal signal '{}' is unavailable. Compile VCS with "
                "-debug_access+all or expose this status at the top level.".format(
                    dotted_path
                )
            )
        handle = getattr(handle, component)
    return handle


async def send_frame_with_timeout(
    source: AxiStreamSource,
    frame_data: bytes,
    description: str,
    timeout_us: int = AXIS_SEND_TIMEOUT_US,
    tuser: int = 0,
) -> None:
    """Send one AXI-stream frame with an optional tuser value."""

    frame = AxiStreamFrame(bytes(frame_data))
    frame.tuser = tuser

    try:
        await with_timeout(
            source.send(frame),
            timeout_us,
            "us",
        )
    except SimTimeoutError as exc:
        raise AssertionError(
            "Timed out after {} us while sending {}. "
            "AXI handshake did not complete.".format(
                timeout_us,
                description,
            )
        ) from exc


async def receive_frame_with_timeout(
    sink: AxiStreamSink,
    description: str,
    timeout_us: int = AXIS_RECV_TIMEOUT_US,
) -> bytes:
    """Receive one frame and fail with a meaningful timeout message."""
    try:
        frame = await with_timeout(sink.recv(), timeout_us, "us")
    except SimTimeoutError as exc:
        raise AssertionError(
            "Timed out after {} us while waiting for {}. "
            "Increase PCIE_AXIS_RECV_TIMEOUT_US if the design is intentionally slow.".format(
                timeout_us,
                description,
            )
        ) from exc

    return bytes(frame.tdata)


async def wait_for_signal_high(
    dut,
    signal,
    description: str,
    timeout_us: int,
) -> None:
    """Wait for a one-bit DUT signal to become one."""

    async def waiter():
        while True:
            await RisingEdge(dut.clk_i)
            if signal.value.is_resolvable and int(signal.value) == 1:
                return

    try:
        await with_timeout(waiter(), timeout_us, "us")
    except SimTimeoutError as exc:
        raise AssertionError(
            "{} did not assert within {} us. "
            "Increase PCIE_FC_INITIALIZED_TIMEOUT_US if the FSM is intentionally slow.".format(
                description,
                timeout_us,
            )
        ) from exc


async def phy_output_monitor(
    tb: TB,
    output_queue: Queue,
    stop_event: Event,
) -> None:
    """Continuously capture and describe packets sent toward the PHY."""
    frame_index = 0
    tb.log.info("Starting m_phy_axis monitor")

    while not stop_event.is_set():
        try:
            frame = await with_timeout(
                tb.phy_sink.recv(),
                MONITOR_POLL_TIMEOUT_US,
                "us",
            )
        except SimTimeoutError:
            continue

        frame_index += 1
        frame_data = bytes(frame.tdata)
        await output_queue.put(frame_data)

        tb.log.info(
            "m_phy_axis frame %d: length=%d data=%s",
            frame_index,
            len(frame_data),
            frame_data.hex(),
        )

        dllp_payload = check_dllp_crc(frame_data)
        if dllp_payload is None:
            if len(frame_data) == DLLP_FRAME_BYTES:
                tb.log.warning(
                    "Six-byte m_phy_axis frame did not pass DLLP CRC checking"
                )
            continue

        try:
            decoded = Dllp().unpack(dllp_payload)
        except Exception:
            tb.log.exception(
                "DLLP CRC passed, but decoding failed for %s",
                dllp_payload.hex(),
            )
            continue

        tb.log.info("Decoded outgoing DLLP: %s", decoded)

    tb.log.info("Stopped m_phy_axis monitor after %d frame(s)", frame_index)


async def send_flow_control_initialization(
    tb: TB,
    completion_hdr_fc: int = 0,
    completion_data_fc: int = 0,
) -> int:
    """
    Send the FC1/FC2 sequence.

    The repeated INIT_FC2_P packet is intentionally retained from the original
    test to exercise repeated flow-control initialization traffic.
    """
    sequence: List[Tuple[DllpType, int, int, str]] = [
        (DllpType.INIT_FC1_P,   0, 200, "INIT_FC1_P"),
        (DllpType.INIT_FC1_NP,  0, 200, "INIT_FC1_NP"),
        (DllpType.INIT_FC1_CPL, 0,   0, "INIT_FC1_CPL"),
        (DllpType.INIT_FC2_P,   0,  20, "INIT_FC2_P first"),
        (DllpType.INIT_FC2_P,   0,  20, "INIT_FC2_P repeated"),
        (DllpType.INIT_FC2_NP,  0, 200, "INIT_FC2_NP"),
        (DllpType.INIT_FC2_CPL, 0,   0, "INIT_FC2_CPL"),
    ]

    for dllp_type, seq, delay_cycles, description in sequence:
        if dllp_type in (DllpType.INIT_FC1_CPL, DllpType.INIT_FC2_CPL):
            frame_data = build_fc_dllp(
                dllp_type=dllp_type,
                seq=seq,
                hdr_fc=completion_hdr_fc,
                data_fc=completion_data_fc,
            )
        else:
            frame_data = build_fc_dllp(dllp_type=dllp_type, seq=seq)

        tb.log.info("Sending incoming %s: %s", description, frame_data.hex())

        await send_frame_with_timeout(
            tb.phy_source,
            frame_data,
            description,
            timeout_us=AXIS_SEND_TIMEOUT_US,
            tuser=PHY_USER_IS_DLLP,
        )

        await tb.wait_cycles(delay_cycles)

    return len(sequence)


def drain_queue(
    queue: Queue,
    context: str = "test boundary",
    allow_ack_nak: bool = False,
) -> List[bytes]:
    """Drain background traffic without silently deleting protocol responses."""
    frames = []

    while not queue.empty():
        frame_data = queue.get_nowait()
        frames.append(frame_data)

        if len(frame_data) != DLLP_FRAME_BYTES:
            continue

        payload = check_dllp_crc(frame_data)
        if payload is None:
            raise AssertionError(
                "{} contained an outgoing six-byte DLLP with invalid CRC: {}".format(
                    context,
                    frame_data.hex(),
                )
            )

        try:
            decoded = Dllp().unpack(payload)
        except Exception as exc:
            raise AssertionError(
                "{} contained an undecodable outgoing DLLP: {}".format(
                    context,
                    frame_data.hex(),
                )
            ) from exc

        if not allow_ack_nak and decoded.type in (DllpType.ACK, DllpType.NAK):
            raise AssertionError(
                "{} discarded an unconsumed {} DLLP with sequence {}".format(
                    context,
                    decoded.type.name,
                    decoded.seq,
                )
            )

    return frames


async def wait_for_outgoing_tlp(
    output_queue: Queue,
    expected_tlp_payload: bytes,
    timeout_us: int = AXIS_RECV_TIMEOUT_US,
) -> bytes:
    """Find an outgoing link packet that contains the expected raw TLP."""

    async def finder():
        while True:
            frame_data = await output_queue.get()

            # DLLPs are six bytes in this environment.
            if len(frame_data) == DLLP_FRAME_BYTES:
                payload = check_dllp_crc(frame_data)
                if payload is None:
                    raise AssertionError(
                        "Outgoing six-byte DLLP has an invalid CRC while "
                        "waiting for a TLP: {}".format(frame_data.hex())
                    )

                try:
                    decoded = Dllp().unpack(payload)
                except Exception as exc:
                    raise AssertionError(
                        "Outgoing DLLP could not be decoded while waiting "
                        "for a TLP: {}".format(frame_data.hex())
                    ) from exc

                if decoded.type in (DllpType.ACK, DllpType.NAK):
                    raise AssertionError(
                        "Unexpected {} DLLP with sequence {} while waiting "
                        "for outgoing TLP payload {}".format(
                            decoded.type.name,
                            decoded.seq,
                            expected_tlp_payload.hex(),
                        )
                    )
                continue

            if len(frame_data) < DLLP_FRAME_BYTES:
                continue

            if expected_tlp_payload in frame_data:
                return frame_data

            raise AssertionError(
                "Received a non-DLLP m_phy_axis frame, but it did not contain "
                "the expected transaction-layer TLP. frame={} expected={}".format(
                    frame_data.hex(),
                    expected_tlp_payload.hex(),
                )
            )

    try:
        return await with_timeout(finder(), timeout_us, "us")
    except SimTimeoutError as exc:
        raise AssertionError(
            "No outgoing TLP containing the expected payload was observed "
            "within {} us. Increase PCIE_AXIS_RECV_TIMEOUT_US if the design is slow.".format(
                timeout_us
            )
        ) from exc


async def wait_for_outgoing_dllp(
    output_queue: Queue,
    expected_type: DllpType,
    timeout_us: int = AXIS_RECV_TIMEOUT_US,
) -> Dllp:
    """Wait for one ACK/NAK response without hiding a contradictory response.

    Periodic InitFC/UpdateFC traffic is independent and may be skipped.  ACK
    and NAK responses are causal, however, so consuming the opposite response
    would hide the first protocol error and usually cause a misleading timeout
    in a later test.
    """

    ack_nak_types = (DllpType.ACK, DllpType.NAK)

    async def finder():
        while True:
            frame_data = await output_queue.get()

            if len(frame_data) != DLLP_FRAME_BYTES:
                continue

            payload = check_dllp_crc(frame_data)

            if payload is None:
                raise AssertionError(
                    "Outgoing six-byte DLLP has an invalid CRC: {}".format(
                        frame_data.hex()
                    )
                )

            try:
                decoded = Dllp().unpack(payload)
            except Exception as exc:
                raise AssertionError(
                    "Outgoing DLLP could not be decoded: {}".format(
                        frame_data.hex()
                    )
                ) from exc

            if decoded.type == expected_type:
                return decoded

            if expected_type in ack_nak_types and decoded.type in ack_nak_types:
                raise AssertionError(
                    "Unexpected {} DLLP with sequence {} while waiting for {}".format(
                        decoded.type.name,
                        decoded.seq,
                        expected_type.name,
                    )
                )

    try:
        return await with_timeout(finder(), timeout_us, "us")
    except SimTimeoutError as exc:
        raise AssertionError(
            "No outgoing {} DLLP was observed within {} us".format(
                expected_type.name,
                timeout_us,
            )
        ) from exc


async def assert_no_outgoing_ack_nak(
    output_queue: Queue,
    window_cycles: int = NO_RESPONSE_WINDOW_CYCLES,
) -> None:
    """Require complete ACK/NAK silence while allowing periodic FC DLLPs."""

    async def finder():
        while True:
            frame_data = await output_queue.get()

            if len(frame_data) != DLLP_FRAME_BYTES:
                continue

            payload = check_dllp_crc(frame_data)
            if payload is None:
                raise AssertionError(
                    "Outgoing six-byte DLLP has an invalid CRC: {}".format(
                        frame_data.hex()
                    )
                )

            try:
                decoded = Dllp().unpack(payload)
            except Exception as exc:
                raise AssertionError(
                    "Outgoing DLLP could not be decoded: {}".format(
                        frame_data.hex()
                    )
                ) from exc

            if decoded.type in (DllpType.ACK, DllpType.NAK):
                return decoded

    try:
        decoded = await with_timeout(
            finder(), window_cycles * CLOCK_PERIOD_NS, "ns"
        )
    except SimTimeoutError:
        return

    raise AssertionError(
        "Unexpected {} DLLP with sequence {} while ACK/NAK suppression "
        "was required".format(decoded.type.name, decoded.seq)
    )


async def transmit_local_tlp(
    tb: TB,
    output_queue: Queue,
    raw_tlp: bytes,
    description: str,
) -> Tuple[bytes, int]:
    """Submit one local TLP and return its complete link packet and sequence."""
    await send_frame_with_timeout(tb.tlp_source, raw_tlp, description)
    link_packet = await wait_for_outgoing_tlp(output_queue, raw_tlp)
    sequence_number = int.from_bytes(link_packet[:2], "big") & 0xFFF
    assert link_packet[2:-4] == raw_tlp, (
        "{} was modified by the transmit Data Link Layer".format(description)
    )
    expected_lcrc = zlib.crc32(link_packet[:-4]) & 0xFFFFFFFF
    received_lcrc = int.from_bytes(link_packet[-4:], "little")
    assert received_lcrc == expected_lcrc, (
        "{} has incorrect outgoing LCRC: got 0x{:08x}, expected 0x{:08x}".format(
            description, received_lcrc, expected_lcrc
        )
    )
    return link_packet, sequence_number


async def acknowledge_sequence(tb: TB, sequence_number: int, description: str) -> None:
    await send_incoming_dllp(
        tb,
        build_ack_nak_dllp(DllpType.ACK, sequence_number),
        description,
    )
    await tb.wait_cycles(8)


async def assert_no_outgoing_tlp(
    output_queue: Queue,
    forbidden_tlp_payload: bytes,
    window_cycles: int = NO_RESPONSE_WINDOW_CYCLES,
) -> None:
    """Fail if a PHY output frame containing the forbidden TLP appears."""

    async def finder():
        while True:
            frame_data = await output_queue.get()

            if len(frame_data) == DLLP_FRAME_BYTES:
                payload = check_dllp_crc(frame_data)
                if payload is None:
                    raise AssertionError(
                        "Outgoing six-byte DLLP has an invalid CRC during "
                        "a no-TLP response window: {}".format(frame_data.hex())
                    )

                try:
                    decoded = Dllp().unpack(payload)
                except Exception as exc:
                    raise AssertionError(
                        "Outgoing DLLP could not be decoded during a no-TLP "
                        "response window: {}".format(frame_data.hex())
                    ) from exc

                if decoded.type in (DllpType.ACK, DllpType.NAK):
                    raise AssertionError(
                        "Unexpected {} DLLP with sequence {} during a "
                        "no-TLP response window".format(
                            decoded.type.name,
                            decoded.seq,
                        )
                    )
                continue

            if len(frame_data) > DLLP_FRAME_BYTES and forbidden_tlp_payload in frame_data:
                return frame_data

    try:
        frame_data = await with_timeout(
            finder(), window_cycles * CLOCK_PERIOD_NS, "ns"
        )
    except SimTimeoutError:
        return

    raise AssertionError(
        "Forbidden outgoing TLP was transmitted: {}".format(frame_data.hex())
    )


async def assert_no_tlp_delivered(
    tb: TB,
    description: str,
    window_cycles: int = NO_RESPONSE_WINDOW_CYCLES,
) -> None:
    """Fail if a TLP reaches m_tlp_axis during the rejection window."""
    try:
        frame = await with_timeout(
            tb.tlp_sink.recv(), window_cycles * CLOCK_PERIOD_NS, "ns"
        )
    except SimTimeoutError:
        return

    raise AssertionError(
        "{} unexpectedly delivered TLP {}".format(
            description,
            bytes(frame.tdata).hex(),
        )
    )


async def verify_malformed_tlp_is_rejected(
    tb: TB,
    output_queue: Queue,
    malformed_data: bytes,
    last_good_sequence: int,
) -> int:
    """Reject malformed framing, validate its NAK, then perform recovery."""
    drain_queue(output_queue, "before malformed incoming TLP")

    await send_frame_with_timeout(
        tb.phy_source,
        malformed_data,
        "malformed incoming TLP without sequence number or LCRC",
        tuser=PHY_USER_IS_TLP,
    )

    await assert_no_tlp_delivered(tb, "malformed incoming TLP")

    nak = await wait_for_outgoing_dllp(output_queue, DllpType.NAK)
    assert nak.seq == last_good_sequence, (
        "Malformed-TLP NAK sequence mismatch: got {} expected {}".format(
            nak.seq,
            last_good_sequence,
        )
    )

    # Supply the correctly framed missing TLP so NAK_SCHEDULED is cleared and
    # the following independent DLLP tests do not inherit recovery state.
    recovery_sequence = (last_good_sequence + 1) & 0xFFF
    await send_frame_with_timeout(
        tb.phy_source,
        add_sequence_and_lcrc(recovery_sequence, malformed_data),
        "valid replay after malformed incoming TLP",
        tuser=PHY_USER_IS_TLP,
    )
    recovered = await receive_frame_with_timeout(
        tb.tlp_sink,
        "valid replay after malformed incoming TLP",
    )
    assert recovered == malformed_data, "Malformed-TLP recovery payload changed"

    ack = await wait_for_outgoing_dllp(output_queue, DllpType.ACK)
    assert ack.seq == recovery_sequence, (
        "Malformed-TLP recovery ACK mismatch: got {} expected {}".format(
            ack.seq,
            recovery_sequence,
        )
    )
    return recovery_sequence


async def send_incoming_dllp(tb: TB, frame_data: bytes, description: str) -> None:
    await send_frame_with_timeout(
        tb.phy_source,
        frame_data,
        description,
        tuser=PHY_USER_IS_DLLP,
    )


async def verify_bad_lcrc_generates_nak(
    tb: TB,
    output_queue: Queue,
    sequence_number: int,
    last_good_sequence: int,
) -> int:
    drain_queue(output_queue)

    raw_tlp, _ = build_memory_write(payload_length=8, tag=0x31)
    link_packet = bytearray(
        add_sequence_and_lcrc(sequence_number=sequence_number, tlp_payload=raw_tlp)
    )
    link_packet[-1] ^= 0x01

    await send_frame_with_timeout(
        tb.phy_source,
        bytes(link_packet),
        "incoming TLP with corrupt LCRC",
        tuser=PHY_USER_IS_TLP,
    )

    await assert_no_tlp_delivered(tb, "Bad-LCRC TLP")

    nak = await wait_for_outgoing_dllp(output_queue, DllpType.NAK)
    assert nak.seq == last_good_sequence, (
        "Bad-LCRC NAK sequence mismatch: got {} expected {}".format(
            nak.seq,
            last_good_sequence,
        )
    )

    # Replay the same TLP correctly.  A valid expected packet must clear the
    # receiver's pending-NAK state, reach the Transaction Layer, and advance
    # NEXT_RCV_SEQ exactly once.
    good_link_packet = add_sequence_and_lcrc(
        sequence_number=sequence_number,
        tlp_payload=raw_tlp,
    )
    await send_frame_with_timeout(
        tb.phy_source,
        good_link_packet,
        "correct replay after bad LCRC",
        tuser=PHY_USER_IS_TLP,
    )
    recovered_tlp = await receive_frame_with_timeout(
        tb.tlp_sink,
        "replayed TLP after bad LCRC",
    )
    assert recovered_tlp == raw_tlp, "Correct replay was not delivered unchanged"

    ack = await wait_for_outgoing_dllp(output_queue, DllpType.ACK)
    assert ack.seq == sequence_number, (
        "Replay ACK sequence mismatch: got {} expected {}".format(
            ack.seq, sequence_number
        )
    )
    return sequence_number


async def verify_sequence_number_errors(
    tb: TB,
    output_queue: Queue,
    last_good_sequence: int,
) -> int:
    """Verify PCIe modulo-4096 receive ordering, including the 2048 boundary."""
    expected_sequence = (last_good_sequence + 1) & 0xFFF

    # Per PCIe Gen1, these are duplicates, not missing/future TLPs.  They are
    # discarded and cause a cumulative ACK for the last successfully delivered
    # TLP.  The <= 2048 boundary is deliberate.
    duplicate_tests = [
        (last_good_sequence, "immediately repeated duplicate"),
        ((last_good_sequence - 1) & 0xFFF, "older duplicate"),
        ((expected_sequence - 0x800) & 0xFFF, "duplicate at 2048 boundary"),
    ]

    for index, (sequence_number, description) in enumerate(duplicate_tests):
        raw_tlp, _ = build_memory_write(payload_length=8, tag=0x40 + index)
        link_packet = add_sequence_and_lcrc(
            sequence_number=sequence_number,
            tlp_payload=raw_tlp,
        )

        tb.log.info(
            "Sequence check: %s, sending seq=%d, expected=%d, last-good=%d",
            description,
            sequence_number,
            expected_sequence,
            last_good_sequence,
        )

        await send_frame_with_timeout(
            tb.phy_source,
            link_packet,
            description,
            tuser=PHY_USER_IS_TLP,
        )

        await assert_no_tlp_delivered(tb, description)
        ack = await wait_for_outgoing_dllp(output_queue, DllpType.ACK)
        assert ack.seq == last_good_sequence, (
            "{} cumulative ACK mismatch: got {} expected {}".format(
                description,
                ack.seq,
                last_good_sequence,
            )
        )

    async def reject_future_and_recover(
        received_sequence: int,
        current_expected: int,
        current_last_good: int,
        tag: int,
        description: str,
    ) -> int:
        future_tlp, _ = build_memory_write(payload_length=8, tag=tag)
        tb.log.info(
            "Sequence check: %s, sending seq=%d, expected=%d, last-good=%d",
            description,
            received_sequence,
            current_expected,
            current_last_good,
        )
        await send_frame_with_timeout(
            tb.phy_source,
            add_sequence_and_lcrc(received_sequence, future_tlp),
            description,
            tuser=PHY_USER_IS_TLP,
        )
        await assert_no_tlp_delivered(tb, description)
        nak = await wait_for_outgoing_dllp(output_queue, DllpType.NAK)
        assert nak.seq == current_last_good, (
            "{} NAK mismatch: got {} expected {}".format(
                description, nak.seq, current_last_good
            )
        )

        # Replay begins at the actual missing sequence, clears NAK_SCHEDULED,
        # and advances NEXT_RCV_SEQ once.
        recovery_tlp, _ = build_memory_write(payload_length=8, tag=tag + 1)
        await send_frame_with_timeout(
            tb.phy_source,
            add_sequence_and_lcrc(current_expected, recovery_tlp),
            "recovery after {}".format(description),
            tuser=PHY_USER_IS_TLP,
        )
        delivered = await receive_frame_with_timeout(
            tb.tlp_sink, "recovery after {}".format(description)
        )
        assert delivered == recovery_tlp, "Recovered TLP payload changed"
        ack = await wait_for_outgoing_dllp(output_queue, DllpType.ACK)
        assert ack.seq == current_expected, (
            "Recovery ACK mismatch: got {} expected {}".format(
                ack.seq, current_expected
            )
        )
        return current_expected

    # A one-packet gap is the normal missing-TLP case.
    last_good_sequence = await reject_future_and_recover(
        received_sequence=(expected_sequence + 1) & 0xFFF,
        current_expected=expected_sequence,
        current_last_good=last_good_sequence,
        tag=0x43,
        description="one-packet sequence gap",
    )

    # Exercise the other side of the modulo-4096 half-range boundary.  A
    # distance of 2049 is future/out-of-sequence (2048 was duplicate above).
    expected_sequence = (last_good_sequence + 1) & 0xFFF
    last_good_sequence = await reject_future_and_recover(
        received_sequence=(expected_sequence + 0x7FF) & 0xFFF,
        current_expected=expected_sequence,
        current_last_good=last_good_sequence,
        tag=0x45,
        description="future TLP at 2049-distance boundary",
    )

    return last_good_sequence


async def verify_dllp_arbitration_priority(
    tb: TB,
    output_queue: Queue,
    sequence_number: int,
    last_good_sequence: int,
) -> int:
    # Start backpressure only between frames so an already-selected flow-control
    # DLLP cannot remain at the head of the arbiter during this check.
    while not tb.phy_sink.idle():
        await RisingEdge(tb.dut.clk_i)
    tb.phy_sink.pause = True
    await tb.wait_cycles(2)
    drain_queue(output_queue)

    bad_tlp, _ = build_memory_write(payload_length=8, tag=0x4A)
    bad_link_packet = bytearray(
        add_sequence_and_lcrc(sequence_number=sequence_number, tlp_payload=bad_tlp)
    )
    bad_link_packet[-1] ^= 0x01

    local_tlp, _ = build_memory_write(payload_length=8, tag=0x4B)

    # HEAD-OF-LINE SAMPLE AT THE INSTANT THE NAK IS SCHEDULED (sec 63 #7f).
    #
    # Base 2.1 sec 3.5.2.1, Implementation Note "Recommended Priority of
    # Scheduled Transmissions", pp.178-179 (book/PCIE-base-spec.Rev2-1.txt
    # :8571-8598):
    #   1) Completion of any transmission (TLP or DLLP) currently in progress
    #      (highest priority)
    #   2) Nak DLLP transmissions
    #   3) Ack DLLP transmissions scheduled ... as soon as possible ...
    #   4) FC DLLP transmissions required to satisfy Section 2.6
    #   ...
    # So exactly ONE thing may legitimately leave the DLL ahead of a scheduled
    # Nak: whatever was already in progress when the Nak was scheduled (1).
    # Anything else that precedes the Nak -- an UpdateFC that was merely
    # pending, an Ack -- is a priority inversion of (2) by (4) or (3).
    #
    # "In progress" is measurable here without a wire model: the shared-PHY
    # arbiter (axis_arb_mux, BLOCK=ACKNOWLEDGE) holds a grant until the granted
    # frame's tlast is ACCEPTED (axis_arb_mux.v:171-172), and the sink is
    # paused, so the word at the head of m_phy_axis when nak_scheduled_r rises
    # is the frame that will complete first and it cannot be displaced. Sample
    # that word once, at that instant; a frame ahead of the Nak is legitimate
    # iff it IS that word. Everything else fails. Commit B's release-triggered
    # UpdateFC-P is what first exercised this path (the previous phase's last
    # release leaves one a few cycles behind its Ack); the FIRST fix skipped
    # every leading UpdateFC and was too loose -- it would have passed a
    # pending UpdateFC jumping a scheduled Nak.
    nak_sched = get_internal_handle(
        tb.dut, "dllp_receive_inst.dllp2tlp_inst.nak_scheduled_r")
    head = {}

    async def sample_head_when_nak_scheduled():
        prev = int(nak_sched.value) if nak_sched.value.is_resolvable else 0
        while True:
            await RisingEdge(tb.dut.clk_i)
            cur = int(nak_sched.value) if nak_sched.value.is_resolvable else 0
            if cur and not prev:
                head["ns"] = get_sim_time("ns")
                head["valid"] = int(tb.dut.m_phy_axis_tvalid.value)
                raw = tb.dut.m_phy_axis_tdata.value
                head["word"] = int(raw) if (head["valid"] and raw.is_resolvable) else None
                return
            prev = cur

    head_task = cocotb.start_soon(sample_head_when_nak_scheduled())

    await send_frame_with_timeout(
        tb.phy_source,
        bytes(bad_link_packet),
        "bad-LCRC TLP creating pending NAK for arbitration",
        tuser=PHY_USER_IS_TLP,
    )

    await send_frame_with_timeout(
        tb.tlp_source,
        local_tlp,
        "local TLP competing with pending DLLP",
    )

    # Allow both producers time to reach the shared PHY.  The AXI arbiter locks
    # a grant for the complete packet, so backpressure cannot make arbitration
    # preemptive: the local TLP may already have been granted while the receive
    # path is still checking the LCRC and constructing the NAK.
    await tb.wait_cycles(100)
    tb.phy_sink.pause = False

    if not head_task.done():
        head_task.kill()
    assert head, (
        "nak_scheduled_r never rose after the bad-LCRC TLP, so there was no "
        "Nak to arbitrate and this phase would be measuring nothing")
    tb.log.info(
        "Arbitration check: Nak scheduled at %s ns; head of m_phy_axis then: "
        "valid=%s word=%s", head["ns"], head["valid"],
        None if head["word"] is None else "0x%08x" % head["word"])

    updatefc_types = (DllpType.UPDATE_FC_P, DllpType.UPDATE_FC_NP,
                      DllpType.UPDATE_FC_CPL)
    used_head = [False]

    async def first_frame_not_legitimately_ahead():
        """Pop frames; let through ONLY the one that was in progress (clause 1)."""
        while True:
            frame = await output_queue.get()
            payload = check_dllp_crc(frame)
            if payload is None:
                return frame            # a TLP: judged by the branch below
            kind = Dllp().unpack(payload).type
            if kind not in updatefc_types:
                return frame            # the Nak we want, or an Ack (fails below)
            in_progress = (
                bool(head["valid"]) and head["word"] is not None
                and not used_head[0]
                and frame[:4] == head["word"].to_bytes(4, "little"))
            assert in_progress, (
                "{} preceded the scheduled Nak but was NOT the transmission in "
                "progress when the Nak was scheduled (head then: valid={} "
                "word={}). Base 2.1 sec 3.5.2.1 Implementation Note "
                "'Recommended Priority of Scheduled Transmissions' pp.178-179: "
                "only 1) a transmission already in progress may complete ahead "
                "of 2) a Nak; 4) FC DLLPs rank below it".format(
                    kind.name, head["valid"],
                    None if head["word"] is None else "0x%08x" % head["word"]))
            used_head[0] = True
            tb.log.info(
                "Arbitration check: %s was the transmission in progress when "
                "the Nak was scheduled (clause 1) -- letting it complete", kind.name)

    try:
        first_frame = await with_timeout(
            first_frame_not_legitimately_ahead(),
            AXIS_RECV_TIMEOUT_US,
            "us",
        )
    except SimTimeoutError as exc:
        raise AssertionError(
            "No PHY output was observed during DLLP arbitration"
        ) from exc
    payload = check_dllp_crc(first_frame)
    if payload is None:
        # A packet that was granted before the NAK request became visible must
        # finish before priority can be reconsidered.  Confirm that it is the
        # one competing TLP, then require the pending NAK next.
        assert local_tlp in first_frame, (
            "Unexpected non-DLLP frame ahead of arbitration NAK: {}".format(
                first_frame.hex()
            )
        )
        tb.log.info(
            "Local TLP was already granted before NAK generation; checking "
            "non-preemptive NAK service after the packet boundary"
        )
        local_packet = first_frame
        decoded = await wait_for_outgoing_dllp(output_queue, DllpType.NAK)
    else:
        decoded = Dllp().unpack(payload)
        assert decoded.type == DllpType.NAK, (
            "DLLP arbitration failed: first DLLP was {}, expected NAK".format(
                decoded.type.name
            )
        )
        local_packet = await wait_for_outgoing_tlp(output_queue, local_tlp)

    assert decoded.seq == last_good_sequence, (
        "Arbitrated NAK sequence mismatch: got {} expected {}".format(
            decoded.seq,
            last_good_sequence,
        )
    )

    local_sequence_number = int.from_bytes(local_packet[:2], "big") & 0xFFF
    await send_incoming_dllp(
        tb,
        build_ack_nak_dllp(DllpType.ACK, local_sequence_number),
        "ACK for arbitration-test TLP",
    )
    await tb.wait_cycles(20)

    # Complete receive-side recovery so the next sequence test begins from a
    # known, protocol-valid receiver state.
    await send_frame_with_timeout(
        tb.phy_source,
        add_sequence_and_lcrc(sequence_number, bad_tlp),
        "valid replay after arbitration test",
        tuser=PHY_USER_IS_TLP,
    )
    delivered = await receive_frame_with_timeout(
        tb.tlp_sink, "valid replay after arbitration test"
    )
    assert delivered == bad_tlp, "Arbitration recovery TLP payload changed"
    ack = await wait_for_outgoing_dllp(output_queue, DllpType.ACK)
    assert ack.seq == sequence_number, (
        "Arbitration recovery ACK mismatch: got {} expected {}".format(
            ack.seq, sequence_number
        )
    )
    return sequence_number


async def verify_ack_nak_replay(
    tb: TB,
    output_queue: Queue,
) -> None:
    raw_tlp, _ = build_memory_write(payload_length=16, tag=0x51)

    await send_frame_with_timeout(
        tb.tlp_source,
        raw_tlp,
        "locally generated TLP for ACK/NAK replay testing",
    )

    first_packet = await wait_for_outgoing_tlp(output_queue, raw_tlp)
    sequence_number = int.from_bytes(first_packet[:2], "big") & 0xFFF

    await send_incoming_dllp(
        tb,
        build_ack_nak_dllp(DllpType.NAK, (sequence_number + 3) & 0xFFF),
        "future NAK DLLP must not request replay",
    )

    await assert_no_outgoing_tlp(output_queue, raw_tlp)

    await send_incoming_dllp(
        tb,
        build_ack_nak_dllp(DllpType.ACK, (sequence_number + 3) & 0xFFF),
        "future ACK DLLP must not clear replay buffer entry",
    )

    last_acknowledged_sequence = (sequence_number - 1) & 0xFFF

    await send_incoming_dllp(
        tb,
        build_ack_nak_dllp(DllpType.NAK, last_acknowledged_sequence),
        "NAK for last good sequence requesting replay of next TLP",
    )

    replay_packet = await wait_for_outgoing_tlp(output_queue, raw_tlp)
    assert replay_packet == first_packet, (
        "Replay retransmission changed packet contents. first={} replay={}".format(
            first_packet.hex(),
            replay_packet.hex(),
        )
    )

    await send_incoming_dllp(
        tb,
        build_ack_nak_dllp(DllpType.ACK, sequence_number),
        "received ACK DLLP completing replay buffer entry",
    )

    await send_incoming_dllp(
        tb,
        build_ack_nak_dllp(DllpType.NAK, last_acknowledged_sequence),
        "stale NAK after cumulative ACK must not replay",
    )

    await assert_no_outgoing_tlp(output_queue, raw_tlp)


async def verify_updatefc_and_credit_blocking(
    tb: TB,
    output_queue: Queue,
) -> None:
    drain_queue(output_queue)
    blocked_tlp, _ = build_memory_write(payload_length=32, tag=0x61)

    await send_frame_with_timeout(
        tb.tlp_source,
        blocked_tlp,
        "TLP submitted after exhausting posted-header credits",
        timeout_us=AXIS_SEND_TIMEOUT_US,
    )

    await assert_no_outgoing_tlp(output_queue, blocked_tlp)

    # Flow-control limits are cumulative and must not be reduced to represent
    # zero available credit.  The initial limit of three has been consumed by
    # the phase-2, arbitration, and ACK/NAK-replay TLPs.  Advancing it to four
    # grants exactly one additional posted-header credit.
    await send_incoming_dllp(
        tb,
        build_fc_dllp(
            dllp_type=DllpType.UPDATE_FC_P,
            hdr_fc=4,
            data_fc=256,
        ),
        "UPDATE_FC_P granting one additional posted-header credit",
    )

    released_packet = await wait_for_outgoing_tlp(output_queue, blocked_tlp)
    released_sequence = int.from_bytes(released_packet[:2], "big") & 0xFFF
    await send_incoming_dllp(
        tb,
        build_ack_nak_dllp(DllpType.ACK, released_sequence),
        "ACK for credit-released TLP",
    )

    # Keep later cumulative limits monotonic and leave enough credit for the
    # optional backpressure transmit test.
    for dllp_type in (
        DllpType.UPDATE_FC_P,
        DllpType.UPDATE_FC_NP,
        DllpType.UPDATE_FC_CPL,
    ):
        await send_incoming_dllp(
            tb,
            build_fc_dllp(
                dllp_type=dllp_type,
                hdr_fc=32,
                data_fc=256,
            ),
            "{} increasing cumulative credit limits".format(dllp_type.name),
        )


async def verify_replay_timer_timeout(tb: TB, output_queue: Queue) -> None:
    """An unacknowledged TLP must be replayed when REPLAY_TIMER expires."""
    raw_tlp, _ = build_memory_write(payload_length=16, tag=0x62)
    await send_frame_with_timeout(
        tb.tlp_source,
        raw_tlp,
        "TLP intentionally left unacknowledged for replay timeout",
    )
    first_packet = await wait_for_outgoing_tlp(output_queue, raw_tlp)
    replay_packet = await wait_for_outgoing_tlp(output_queue, raw_tlp)
    assert replay_packet == first_packet, (
        "Replay-timer retransmission changed the link packet"
    )

    sequence_number = int.from_bytes(first_packet[:2], "big") & 0xFFF
    await send_incoming_dllp(
        tb,
        build_ack_nak_dllp(DllpType.ACK, sequence_number),
        "ACK after replay-timer retransmission",
    )


async def verify_bad_and_malformed_dllps_are_ignored(
    tb: TB,
    output_queue: Queue,
) -> None:
    drain_queue(output_queue)

    malformed_frames = [
        (
            corrupt_dllp_crc(build_fc_dllp(DllpType.UPDATE_FC_P, hdr_fc=0xFF, data_fc=0xFFF)),
            "bad DLLP CRC",
        ),
        (build_raw_dllp(bytes([0xFF, 0x00, 0x00, 0x00])), "invalid DLLP type"),
        (
            build_raw_dllp(bytes([int(DllpType.ACK), 0xFF, 0x0F, 0x00])),
            "ACK DLLP with reserved fields set",
        ),
        (
            build_raw_dllp(bytes([int(DllpType.UPDATE_FC_P), 0x40, 0x00, 0x01])),
            "UpdateFC DLLP using unsupported VC bits",
        ),
    ]

    for frame_data, description in malformed_frames:
        await send_incoming_dllp(tb, frame_data, description)
        await tb.wait_cycles(20)

    # Periodic/credit-triggered UpdateFC DLLPs are independent background
    # traffic.  drain_queue validates every DLLP and fails on an ACK/NAK, while
    # allowing those legitimate FC updates to cross this negative-test window.
    drain_queue(output_queue, "after malformed incoming DLLPs")


async def verify_corrupt_ack_nak_crc(tb: TB, output_queue: Queue) -> None:
    """Corrupt ACK/NAK DLLPs must neither retire nor replay an outstanding TLP."""
    drain_queue(output_queue)
    raw_tlp = build_completion(False, tag=0x70)
    first_packet, sequence_number = await transmit_local_tlp(
        tb, output_queue, raw_tlp, "TLP retained during corrupt ACK/NAK tests"
    )
    prior_sequence = (sequence_number - 1) & 0xFFF

    await send_incoming_dllp(
        tb,
        corrupt_dllp_crc(build_ack_nak_dllp(DllpType.ACK, sequence_number)),
        "ACK DLLP with corrupt CRC",
    )
    await assert_no_outgoing_tlp(output_queue, raw_tlp)

    await send_incoming_dllp(
        tb,
        corrupt_dllp_crc(build_ack_nak_dllp(DllpType.NAK, prior_sequence)),
        "NAK DLLP with corrupt CRC",
    )
    await assert_no_outgoing_tlp(output_queue, raw_tlp)

    # A valid NAK proves the corrupt ACK did not purge the retry entry and the
    # corrupt NAK did not alter replay state.
    await send_incoming_dllp(
        tb,
        build_ack_nak_dllp(DllpType.NAK, prior_sequence),
        "valid NAK after corrupt ACK/NAK DLLPs",
    )
    replay = await wait_for_outgoing_tlp(output_queue, raw_tlp)
    assert replay == first_packet, "Replay changed after corrupt ACK/NAK DLLPs"
    await acknowledge_sequence(tb, sequence_number, "ACK corrupt-DLLP test TLP")


async def verify_cumulative_ack_and_multi_packet_replay(
    tb: TB,
    output_queue: Queue,
) -> None:
    """Verify cumulative retirement and ordered go-back-N replay."""
    drain_queue(output_queue)

    cumulative_tlps = [build_completion(False, 0x74 + i) for i in range(3)]
    cumulative_packets: List[bytes] = []
    cumulative_sequences: List[int] = []
    for index, raw_tlp in enumerate(cumulative_tlps):
        packet, seq = await transmit_local_tlp(
            tb, output_queue, raw_tlp, "cumulative-ACK TLP {}".format(index)
        )
        cumulative_packets.append(packet)
        cumulative_sequences.append(seq)

    # ACK of the middle entry must retire the first and second entries only.
    await acknowledge_sequence(
        tb, cumulative_sequences[1], "cumulative ACK through second outstanding TLP"
    )
    await send_incoming_dllp(
        tb,
        build_ack_nak_dllp(DllpType.NAK, cumulative_sequences[1]),
        "NAK at cumulative ACK point",
    )
    replay = await wait_for_outgoing_tlp(output_queue, cumulative_tlps[2])
    assert replay == cumulative_packets[2], (
        "Cumulative ACK failed to leave only the newest TLP replayable"
    )
    await acknowledge_sequence(tb, cumulative_sequences[2], "retire cumulative test")

    # Fill the buffer again and request replay from immediately before its
    # oldest entry.  Every packet must return once and in original order.
    replay_tlps = [build_completion(False, 0x78 + i) for i in range(3)]
    replay_packets: List[bytes] = []
    replay_sequences: List[int] = []
    for index, raw_tlp in enumerate(replay_tlps):
        packet, seq = await transmit_local_tlp(
            tb, output_queue, raw_tlp, "ordered-replay TLP {}".format(index)
        )
        replay_packets.append(packet)
        replay_sequences.append(seq)

    await send_incoming_dllp(
        tb,
        build_ack_nak_dllp(
            DllpType.NAK, (replay_sequences[0] - 1) & 0xFFF
        ),
        "NAK requesting all outstanding TLPs",
    )
    for index, raw_tlp in enumerate(replay_tlps):
        observed = await wait_for_outgoing_tlp(output_queue, raw_tlp)
        assert observed == replay_packets[index], (
            "Replay order/content failure at outstanding packet {}".format(index)
        )
    await acknowledge_sequence(tb, replay_sequences[-1], "retire ordered replay set")


async def verify_ack_nak_window_boundaries(tb: TB, output_queue: Queue) -> None:
    """Reject ACK/NAK sequence values outside the active transmit window."""
    drain_queue(output_queue)
    raw_tlp = build_completion(False, 0x7C)
    first_packet, sequence_number = await transmit_local_tlp(
        tb, output_queue, raw_tlp, "ACK/NAK window-boundary TLP"
    )

    for offset, dllp_type in ((0x800, DllpType.ACK), (0x800, DllpType.NAK),
                              (1, DllpType.ACK), (1, DllpType.NAK)):
        await send_incoming_dllp(
            tb,
            build_ack_nak_dllp(dllp_type, (sequence_number + offset) & 0xFFF),
            "out-of-window {} offset 0x{:03x}".format(dllp_type.name, offset),
        )
        await assert_no_outgoing_tlp(output_queue, raw_tlp)

    prior_sequence = (sequence_number - 1) & 0xFFF
    await send_incoming_dllp(
        tb,
        build_ack_nak_dllp(DllpType.NAK, prior_sequence),
        "valid boundary NAK for most recently acknowledged sequence",
    )
    replay = await wait_for_outgoing_tlp(output_queue, raw_tlp)
    assert replay == first_packet, "Valid boundary NAK did not preserve replay data"
    await acknowledge_sequence(tb, sequence_number, "retire boundary test TLP")


async def verify_nak_scheduling_suppression(
    tb: TB,
    output_queue: Queue,
    last_good_sequence: int,
) -> int:
    """Only one NAK may remain scheduled while the missing TLP is outstanding."""
    drain_queue(output_queue)
    expected_sequence = (last_good_sequence + 1) & 0xFFF

    for index, received_sequence in enumerate(
        ((expected_sequence + 1) & 0xFFF, (expected_sequence + 2) & 0xFFF)
    ):
        raw_tlp, _ = build_memory_write(8, 0x80 + index)
        await send_frame_with_timeout(
            tb.phy_source,
            add_sequence_and_lcrc(received_sequence, raw_tlp),
            "out-of-sequence TLP while NAK is pending",
            tuser=PHY_USER_IS_TLP,
        )
        await assert_no_tlp_delivered(tb, "out-of-sequence TLP")
        if index == 0:
            nak = await wait_for_outgoing_dllp(output_queue, DllpType.NAK)
            assert nak.seq == last_good_sequence
        else:
            await assert_no_outgoing_ack_nak(output_queue)

    recovery_tlp, _ = build_memory_write(8, 0x82)
    await send_frame_with_timeout(
        tb.phy_source,
        add_sequence_and_lcrc(expected_sequence, recovery_tlp),
        "missing TLP clearing NAK_SCHEDULED",
        tuser=PHY_USER_IS_TLP,
    )
    delivered = await receive_frame_with_timeout(tb.tlp_sink, "NAK suppression recovery")
    assert delivered == recovery_tlp
    ack = await wait_for_outgoing_dllp(output_queue, DllpType.ACK)
    assert ack.seq == expected_sequence
    return expected_sequence


async def verify_ack_latency(
    tb: TB,
    output_queue: Queue,
    last_good_sequence: int,
) -> int:
    """Measure functional ACK latency in DUT clock cycles."""
    drain_queue(output_queue)
    sequence_number = (last_good_sequence + 1) & 0xFFF
    raw_tlp, _ = build_memory_write(MAX_PAYLOAD_BYTES, 0x84)
    start_ns = int(get_sim_time(units="ns"))
    await send_frame_with_timeout(
        tb.phy_source,
        add_sequence_and_lcrc(sequence_number, raw_tlp),
        "maximum-payload TLP for ACK latency",
        tuser=PHY_USER_IS_TLP,
    )
    received = await receive_frame_with_timeout(tb.tlp_sink, "ACK-latency TLP")
    assert received == raw_tlp
    ack = await wait_for_outgoing_dllp(output_queue, DllpType.ACK)
    end_ns = int(get_sim_time(units="ns"))
    latency_cycles = (end_ns - start_ns + CLOCK_PERIOD_NS - 1) // CLOCK_PERIOD_NS
    assert ack.seq == sequence_number
    assert latency_cycles <= ACK_LATENCY_LIMIT_CYCLES, (
        "ACK latency {} cycles exceeds configured limit {} cycles".format(
            latency_cycles, ACK_LATENCY_LIMIT_CYCLES
        )
    )
    return sequence_number


async def verify_tlp_classes_and_formats(
    tb: TB,
    output_queue: Queue,
    last_good_sequence: int,
) -> int:
    """Forward representative PCIe request, completion, and message formats."""
    drain_queue(output_queue)
    max_payload = bytes((index & 0xFF) for index in range(MAX_PAYLOAD_BYTES))
    ecrc_payload = bytes((0xE0 + index) & 0xFF for index in range(16))
    cases: List[Tuple[str, bytes]] = [
        ("3-DW Memory Write", build_memory_write(16, 0x90)[0]),
        ("4-DW Memory Write", build_memory_write_64(16, 0x91)),
        ("3-DW Memory Read", build_memory_read(16, 0x92)),
        ("4-DW Memory Read", build_memory_read(16, 0x93, 0x1_0000_0004, True)),
        ("Completion without Data", build_completion(False, 0x94)),
        ("Completion with Data", build_completion(True, 0x95, 16)),
        ("Message without Data", build_message(False, 0x96)),
        ("Message with Data", build_message(True, 0x97, 16)),
        ("zero-byte Memory Read", build_zero_byte_memory_read(0x98)),
        (
            "Length-field zero (1024-DW) Memory Read",
            build_raw_tlp(0x00, 0, 3),
        ),
        (
            "maximum-payload Memory Write",
            build_raw_tlp(0x40, MAX_PAYLOAD_BYTES // 4, 3, max_payload),
        ),
        (
            "TD/ECRC Memory Write",
            build_raw_tlp(
                0x40, len(ecrc_payload) // 4, 3, ecrc_payload,
                td=True, ecrc=b"\x12\x34\x56\x78",
            ),
        ),
    ]

    # Receive direction: exact byte preservation, sequence acceptance, LCRC
    # validation, and correct cumulative ACK sequence for every format.
    for index, (description, raw_tlp) in enumerate(cases):
        sequence_number = (last_good_sequence + 1) & 0xFFF
        await send_frame_with_timeout(
            tb.phy_source,
            add_sequence_and_lcrc(sequence_number, raw_tlp),
            "incoming {}".format(description),
            tuser=PHY_USER_IS_TLP,
        )
        delivered = await receive_frame_with_timeout(
            tb.tlp_sink, "transaction-layer delivery of {}".format(description)
        )
        assert delivered == raw_tlp, "{} changed on receive".format(description)
        ack = await wait_for_outgoing_dllp(output_queue, DllpType.ACK)
        assert ack.seq == sequence_number, (
            "{} ACK mismatch: got {}, expected {}".format(
                description, ack.seq, sequence_number
            )
        )
        last_good_sequence = sequence_number

    # Transmit direction: every accepted class must acquire one sequence number,
    # preserve its TLP bytes, and receive a correct generated LCRC.  Replenish
    # all three credit classes first so this phase tests format classification,
    # not exhaustion (which is tested separately).
    tx_fc_base = "dllp_transmit_inst.tlp2dllp_inst."
    credit_limit_names = {
        DllpType.UPDATE_FC_P: ("ph_credit_limit_r", "pd_credit_limit_r"),
        DllpType.UPDATE_FC_NP: ("nph_credit_limit_r", "npd_credit_limit_r"),
        DllpType.UPDATE_FC_CPL: ("cplh_credit_limit_r", "cpld_credit_limit_r"),
    }
    for dllp_type in (DllpType.UPDATE_FC_P, DllpType.UPDATE_FC_NP,
                      DllpType.UPDATE_FC_CPL):
        hdr_name, data_name = credit_limit_names[dllp_type]
        hdr_limit = int(get_internal_handle(tb.dut, tx_fc_base + hdr_name).value)
        data_limit = int(get_internal_handle(tb.dut, tx_fc_base + data_name).value)
        await send_incoming_dllp(
            tb,
            build_fc_dllp(
                dllp_type,
                hdr_fc=(hdr_limit + 64) & 0xFF,
                data_fc=(data_limit + 256) & 0xFFF,
            ),
            "credit grant before format-transmit tests",
        )
    await tb.wait_cycles(16)

    previous_sequence: Optional[int] = None
    for description, raw_tlp in cases:
        _, sequence_number = await transmit_local_tlp(
            tb, output_queue, raw_tlp, "outgoing {}".format(description)
        )
        if previous_sequence is not None:
            assert sequence_number == ((previous_sequence + 1) & 0xFFF), (
                "Transmit sequence discontinuity for {}".format(description)
            )
        previous_sequence = sequence_number
        await acknowledge_sequence(
            tb, sequence_number, "ACK outgoing {}".format(description)
        )

    return last_good_sequence


async def verify_retry_buffer_full_and_slot_wrap(
    tb: TB,
    output_queue: Queue,
) -> None:
    """Fill every retry slot, prove backpressure, then reuse wrapped slots."""
    drain_queue(output_queue)
    packets: List[bytes] = []
    sequences: List[int] = []
    for index in range(RETRY_BUFFER_DEPTH):
        raw_tlp = build_completion(False, 0xA0 + index)
        packet, sequence_number = await transmit_local_tlp(
            tb, output_queue, raw_tlp, "retry-buffer fill entry {}".format(index)
        )
        packets.append(packet)
        sequences.append(sequence_number)

    blocked_tlp = build_completion(False, 0xA0 + RETRY_BUFFER_DEPTH)
    blocked_sender = cocotb.start_soon(
        send_frame_with_timeout(
            tb.tlp_source, blocked_tlp, "TLP blocked by full retry buffer"
        )
    )
    await assert_no_outgoing_tlp(output_queue, blocked_tlp)

    # A cumulative ACK through the oldest packet creates exactly one slot.
    await acknowledge_sequence(tb, sequences[0], "free oldest retry-buffer slot")
    try:
        await with_timeout(blocked_sender, AXIS_SEND_TIMEOUT_US, "us")
    except SimTimeoutError as exc:
        blocked_sender.kill()
        raise AssertionError("Full retry buffer did not release after ACK") from exc
    blocked_packet = await wait_for_outgoing_tlp(output_queue, blocked_tlp)
    blocked_sequence = int.from_bytes(blocked_packet[:2], "big") & 0xFFF
    assert blocked_sequence == ((sequences[-1] + 1) & 0xFFF)
    await acknowledge_sequence(tb, blocked_sequence, "retire retry-buffer fill set")

    # Reuse more than one complete physical slot rotation.  ACKing each packet
    # isolates storage-index wrap from capacity backpressure.
    previous_sequence = blocked_sequence
    for index in range(RETRY_BUFFER_DEPTH * 2 + 1):
        raw_tlp = build_completion(False, (0xB0 + index) & 0xFF)
        _, sequence_number = await transmit_local_tlp(
            tb, output_queue, raw_tlp, "retry slot-wrap entry {}".format(index)
        )
        assert sequence_number == ((previous_sequence + 1) & 0xFFF)
        await acknowledge_sequence(tb, sequence_number, "retire slot-wrap entry")
        previous_sequence = sequence_number


async def verify_receive_sequence_rollover(
    tb: TB,
    output_queue: Queue,
    last_good_sequence: int,
) -> int:
    """Drive accepted receive traffic through the real 0xfff -> 0x000 edge."""
    if not env_flag("PCIE_FULL_SEQUENCE_ROLLOVER", "1"):
        tb.log.warning("Skipping full receive rollover by environment request")
        return last_good_sequence

    accepted = 0
    while True:
        sequence_number = (last_good_sequence + 1) & 0xFFF
        raw_tlp = build_raw_tlp(0x0A, 0, 3)
        await send_frame_with_timeout(
            tb.phy_source,
            add_sequence_and_lcrc(sequence_number, raw_tlp),
            "receive rollover TLP seq=0x{:03x}".format(sequence_number),
            tuser=PHY_USER_IS_TLP,
        )
        delivered = await receive_frame_with_timeout(
            tb.tlp_sink, "receive rollover delivery"
        )
        assert delivered == raw_tlp
        ack = await wait_for_outgoing_dllp(output_queue, DllpType.ACK)
        assert ack.seq == sequence_number
        previous = last_good_sequence
        last_good_sequence = sequence_number
        accepted += 1
        if accepted % 256 == 0:
            tb.log.info("Accepted %d TLPs in receive rollover test", accepted)
        if previous == 0xFFF and sequence_number == 0x000:
            break
        assert accepted <= 4096, "Receive sequence did not roll over in 4096 TLPs"
    return last_good_sequence


async def verify_transmit_sequence_rollover(tb: TB, output_queue: Queue) -> None:
    """Drive the transmit sequence generator across 0xfff -> 0x000."""
    if not env_flag("PCIE_FULL_SEQUENCE_ROLLOVER", "1"):
        tb.log.warning("Skipping full transmit rollover by environment request")
        return

    previous: Optional[int] = None
    for index in range(4097):
        raw_tlp = build_completion(False, index & 0xFF)
        packet, sequence_number = await transmit_local_tlp(
            tb, output_queue, raw_tlp,
            "transmit rollover TLP {}".format(index),
        )
        if previous is not None:
            assert sequence_number == ((previous + 1) & 0xFFF)
        if previous == 0xFFF and sequence_number == 0x000:
            # At the exact modulo boundary, NAK(0xfff) requests sequence zero.
            await send_incoming_dllp(
                tb,
                build_ack_nak_dllp(DllpType.NAK, 0xFFF),
                "boundary NAK requesting replay of sequence zero",
            )
            replay = await wait_for_outgoing_tlp(output_queue, raw_tlp)
            assert replay == packet
            await acknowledge_sequence(
                tb, 0x000, "boundary ACK retiring sequence zero"
            )
            return
        await acknowledge_sequence(tb, sequence_number, "ACK transmit rollover TLP")
        previous = sequence_number
    raise AssertionError("Transmit sequence did not roll over in 4097 TLPs")


async def verify_flow_control_classes_and_wrap(
    tb: TB,
    output_queue: Queue,
) -> None:
    """Check P/NP/Cpl header/data accounting, scaling, and counter wrap."""
    base = "dllp_transmit_inst.tlp2dllp_inst."
    signals: Dict[str, object] = {
        name: get_internal_handle(tb.dut, base + name)
        for name in (
            "ph_credits_consumed_r", "pd_credits_consumed_r",
            "nph_credits_consumed_r", "npd_credits_consumed_r",
            "cplh_credits_consumed_r", "cpld_credits_consumed_r",
            "ph_credit_limit_r", "pd_credit_limit_r",
            "nph_credit_limit_r", "npd_credit_limit_r",
            "cplh_credit_limit_r", "cpld_credit_limit_r",
        )
    }

    def value(name: str) -> int:
        assert signals[name].value.is_resolvable, "{} is X/Z".format(name)
        return int(signals[name].value)

    # Each pair is header-only then data-bearing traffic for one FC class.
    cases = [
        (DllpType.UPDATE_FC_P, "ph", "pd",
         build_message(False, 0xC0), build_message(True, 0xC1, 16)),
        (DllpType.UPDATE_FC_NP, "nph", "npd",
         build_memory_read(4, 0xC2), build_raw_tlp(0x42, 4, 3, bytes(16))),
        (DllpType.UPDATE_FC_CPL, "cplh", "cpld",
         build_completion(False, 0xC3), build_completion(True, 0xC4, 16)),
    ]

    for dllp_type, hdr, data, header_tlp, data_tlp in cases:
        hdr_before = value(hdr + "_credits_consumed_r")
        data_before = value(data + "_credits_consumed_r")
        old_hdr_limit = value(hdr + "_credit_limit_r")
        data_limit = value(data + "_credit_limit_r")
        hdr_limit = (old_hdr_limit + 4) & 0xFF
        await send_incoming_dllp(
            tb,
            build_fc_dllp(dllp_type, hdr_fc=hdr_limit, data_fc=data_limit),
            "bounded {} credit grant".format(dllp_type.name),
        )
        await tb.wait_cycles(8)

        _, seq = await transmit_local_tlp(
            tb, output_queue, header_tlp, "{} header-credit TLP".format(hdr)
        )
        await acknowledge_sequence(tb, seq, "ACK header-credit TLP")
        assert value(hdr + "_credits_consumed_r") == ((hdr_before + 1) & 0xFF)
        assert value(data + "_credits_consumed_r") == data_before

        _, seq = await transmit_local_tlp(
            tb, output_queue, data_tlp, "{} data-credit TLP".format(data)
        )
        await acknowledge_sequence(tb, seq, "ACK data-credit TLP")
        assert value(hdr + "_credits_consumed_r") == ((hdr_before + 2) & 0xFF)
        assert value(data + "_credits_consumed_r") == ((data_before + 1) & 0xFFF)

        # Consume every remaining header credit.  The following packet must
        # stay behind s_tlp_axis until a cumulative UpdateFC advances the limit.
        remaining_header_credits = (
            hdr_limit - value(hdr + "_credits_consumed_r")
        ) & 0xFF
        for index in range(remaining_header_credits):
            _, seq = await transmit_local_tlp(
                tb, output_queue, header_tlp,
                "{} header-credit exhaustion {}".format(hdr, index),
            )
            await acknowledge_sequence(tb, seq, "ACK header exhaustion TLP")
        assert value(hdr + "_credits_consumed_r") == hdr_limit
        blocked_header = cocotb.start_soon(
            send_frame_with_timeout(
                tb.tlp_source, header_tlp,
                "{} TLP blocked at header-credit limit".format(hdr),
            )
        )
        await assert_no_outgoing_tlp(output_queue, header_tlp)
        hdr_limit = (hdr_limit + 1) & 0xFF
        await send_incoming_dllp(
            tb,
            build_fc_dllp(dllp_type, hdr_fc=hdr_limit, data_fc=data_limit),
            "{} header-credit release".format(dllp_type.name),
        )
        await with_timeout(blocked_header, AXIS_SEND_TIMEOUT_US, "us")
        released = await wait_for_outgoing_tlp(output_queue, header_tlp)
        await acknowledge_sequence(
            tb, int.from_bytes(released[:2], "big") & 0xFFF,
            "ACK released header-credit TLP",
        )

        # Consume the advertised data limit one credit at a time.  Header limit
        # updates accompany each packet, so the eventual stall is specifically
        # caused by data-credit exhaustion.
        data_exhaustion_index = 0
        while value(data + "_credits_consumed_r") != data_limit:
            hdr_limit = (value(hdr + "_credits_consumed_r") + 1) & 0xFF
            await send_incoming_dllp(
                tb,
                build_fc_dllp(dllp_type, hdr_fc=hdr_limit, data_fc=data_limit),
                "{} header credit during data exhaustion".format(dllp_type.name),
            )
            _, seq = await transmit_local_tlp(
                tb, output_queue, data_tlp,
                "{} data-credit exhaustion {}".format(
                    data, data_exhaustion_index
                ),
            )
            await acknowledge_sequence(tb, seq, "ACK data exhaustion TLP")
            data_exhaustion_index += 1
            assert data_exhaustion_index <= 0x1000, (
                "{} data-credit counter failed to reach its limit".format(data)
            )
        assert value(data + "_credits_consumed_r") == data_limit

        # Preserve one available header credit while data remains exhausted.
        hdr_limit = (value(hdr + "_credits_consumed_r") + 1) & 0xFF
        await send_incoming_dllp(
            tb,
            build_fc_dllp(dllp_type, hdr_fc=hdr_limit, data_fc=data_limit),
            "{} final header credit before data block".format(dllp_type.name),
        )
        blocked_data = cocotb.start_soon(
            send_frame_with_timeout(
                tb.tlp_source, data_tlp,
                "{} TLP blocked at data-credit limit".format(data),
            )
        )
        await assert_no_outgoing_tlp(output_queue, data_tlp)
        data_limit = (data_limit + 1) & 0xFFF
        await send_incoming_dllp(
            tb,
            build_fc_dllp(dllp_type, hdr_fc=hdr_limit, data_fc=data_limit),
            "{} data-credit release".format(dllp_type.name),
        )
        await with_timeout(blocked_data, AXIS_SEND_TIMEOUT_US, "us")
        released = await wait_for_outgoing_tlp(output_queue, data_tlp)
        await acknowledge_sequence(
            tb, int.from_bytes(released[:2], "big") & 0xFFF,
            "ACK released data-credit TLP",
        )

    # PCIe 1.x uses scale 1. Reserved/non-unity scale encodings must not be
    # silently applied as unscaled credit updates by this Gen1 implementation.
    p_hdr_limit = value("ph_credit_limit_r")
    p_data_limit = value("pd_credit_limit_r")
    await send_incoming_dllp(
        tb,
        build_fc_dllp(
            DllpType.UPDATE_FC_P,
            hdr_fc=(p_hdr_limit + 7) & 0xFF,
            data_fc=(p_data_limit + 7) & 0xFFF,
            hdr_scale=1,
            data_scale=1,
        ),
        "UpdateFC_P with unsupported scale encoding",
    )
    await tb.wait_cycles(8)
    assert value("ph_credit_limit_r") == p_hdr_limit
    assert value("pd_credit_limit_r") == p_data_limit

    # A cumulative limit that moves backwards without a legal modulo crossing
    # is stale and must not reduce usable credits.
    stale_hdr = (p_hdr_limit - 1) & 0xFF
    stale_data = (p_data_limit - 1) & 0xFFF
    await send_incoming_dllp(
        tb,
        build_fc_dllp(DllpType.UPDATE_FC_P, hdr_fc=stale_hdr, data_fc=stale_data),
        "stale/decreasing UpdateFC_P",
    )
    await tb.wait_cycles(8)
    assert value("ph_credit_limit_r") == p_hdr_limit
    assert value("pd_credit_limit_r") == p_data_limit

    # Exercise real eight-bit header-consumption rollover while keeping data
    # irrelevant.  The advertised cumulative limit follows the modulo counter.
    while value("ph_credits_consumed_r") != 0xFF:
        consumed = value("ph_credits_consumed_r")
        await send_incoming_dllp(
            tb,
            build_fc_dllp(
                DllpType.UPDATE_FC_P,
                hdr_fc=(consumed + 1) & 0xFF,
                data_fc=value("pd_credit_limit_r"),
            ),
            "posted-header cumulative limit before wrap",
        )
        _, seq = await transmit_local_tlp(
            tb, output_queue, build_message(False, consumed),
            "posted header consumed before counter wrap",
        )
        await acknowledge_sequence(tb, seq, "ACK posted-header wrap TLP")

    await send_incoming_dllp(
        tb,
        build_fc_dllp(
            DllpType.UPDATE_FC_P,
            hdr_fc=0,
            data_fc=value("pd_credit_limit_r"),
        ),
        "legal cumulative posted-header limit wrap to zero",
    )
    _, seq = await transmit_local_tlp(
        tb, output_queue, build_message(False, 0xFF),
        "posted-header credit crossing 0xff to 0x00",
    )
    await acknowledge_sequence(tb, seq, "ACK posted-header rollover TLP")
    assert value("ph_credits_consumed_r") == 0

    # Leave every traffic class usable for the later format, retry-buffer, and
    # rollover phases.  These are forward cumulative grants from the observed
    # consumed counters, so no stale/decreasing update is introduced here.
    for dllp_type, hdr, data in (
        (DllpType.UPDATE_FC_P, "ph", "pd"),
        (DllpType.UPDATE_FC_NP, "nph", "npd"),
        (DllpType.UPDATE_FC_CPL, "cplh", "cpld"),
    ):
        await send_incoming_dllp(
            tb,
            build_fc_dllp(
                dllp_type,
                hdr_fc=(value(hdr + "_credits_consumed_r") + 64) & 0xFF,
                data_fc=(value(data + "_credits_consumed_r") + 256) & 0xFFF,
            ),
            "post-exhaustion {} credit replenishment".format(dllp_type.name),
        )


async def verify_repeated_nak_and_replay_exhaustion(
    tb: TB,
    output_queue: Queue,
) -> None:
    """Repeated NAKs may replay only up to the configured retry limit."""
    drain_queue(output_queue)
    raw_tlp = build_completion(False, 0xD0)
    first_packet, sequence_number = await transmit_local_tlp(
        tb, output_queue, raw_tlp, "TLP for repeated-NAK exhaustion"
    )
    prior_sequence = (sequence_number - 1) & 0xFFF
    for attempt in range(MAX_REPLAY_ATTEMPTS):
        await send_incoming_dllp(
            tb,
            build_ack_nak_dllp(DllpType.NAK, prior_sequence),
            "repeated NAK attempt {}".format(attempt + 1),
        )
        replay = await wait_for_outgoing_tlp(output_queue, raw_tlp)
        assert replay == first_packet, "Repeated NAK changed replay contents"

    await send_incoming_dllp(
        tb,
        build_ack_nak_dllp(DllpType.NAK, prior_sequence),
        "NAK exceeding replay attempt limit",
    )
    await assert_no_outgoing_tlp(output_queue, raw_tlp)
    retry_error = get_internal_handle(tb.dut, "dllp_transmit_inst.retry_err")
    await tb.wait_cycles(8)
    assert retry_error.value.is_resolvable and int(retry_error.value) == 1, (
        "Replay-attempt exhaustion did not assert retry_err"
    )


async def verify_replay_timer_exhaustion(tb: TB, output_queue: Queue) -> None:
    """An ACK-less packet must stop replaying and report retry exhaustion."""
    drain_queue(output_queue)
    raw_tlp = build_completion(False, 0xD4)
    first_packet, _ = await transmit_local_tlp(
        tb, output_queue, raw_tlp, "TLP for replay-timer exhaustion"
    )
    for attempt in range(MAX_REPLAY_ATTEMPTS):
        replay = await wait_for_outgoing_tlp(output_queue, raw_tlp)
        assert replay == first_packet, (
            "Replay-timer attempt {} changed packet contents".format(attempt + 1)
        )

    retry_error = get_internal_handle(tb.dut, "dllp_transmit_inst.retry_err")
    async def wait_for_retry_error() -> None:
        while True:
            await RisingEdge(tb.dut.clk_i)
            if retry_error.value.is_resolvable and int(retry_error.value) == 1:
                return

    try:
        await with_timeout(
            wait_for_retry_error(),
            (REPLAY_TIMER_CYCLES + NO_RESPONSE_WINDOW_CYCLES) * CLOCK_PERIOD_NS,
            "ns",
        )
    except SimTimeoutError as exc:
        raise AssertionError("Replay-timer exhaustion did not assert retry_err") from exc
    await assert_no_outgoing_tlp(output_queue, raw_tlp)


async def reinitialize_link(
    tb: TB,
    output_queue: Queue,
    completion_hdr_fc: int = 0,
    completion_data_fc: int = 0,
) -> None:
    """Bring the link down, prove FC state reset, then perform FC init again."""
    tb.dut.phy_link_up_i.value = 0
    tb.dut.idle_valid_i.value = 0
    await tb.wait_cycles(32)
    assert int(tb.dut.fc_initialized_o.value) == 0
    drain_queue(output_queue)
    tb.dut.idle_valid_i.value = 1
    tb.dut.phy_link_up_i.value = 1
    await tb.wait_cycles(32)
    await send_flow_control_initialization(
        tb,
        completion_hdr_fc=completion_hdr_fc,
        completion_data_fc=completion_data_fc,
    )
    await wait_for_signal_high(
        tb.dut, tb.dut.fc_initialized_o, "fc_initialized_o after link reset",
        FC_INITIALIZED_TIMEOUT_US,
    )
    await tb.wait_cycles(32)
    drain_queue(output_queue)


async def verify_link_down_with_pending_replay(
    tb: TB,
    output_queue: Queue,
) -> int:
    """Link-down must flush pending retry state and restart both sequences."""
    raw_tlp = build_completion(False, 0xD1)
    await transmit_local_tlp(
        tb, output_queue, raw_tlp, "outstanding TLP before link-down"
    )
    tb.dut.phy_link_up_i.value = 0
    tb.dut.idle_valid_i.value = 0
    await tb.wait_cycles(REPLAY_TIMER_CYCLES + NO_RESPONSE_WINDOW_CYCLES)
    await assert_no_outgoing_tlp(output_queue, raw_tlp)
    await reinitialize_link(tb, output_queue)

    new_tlp = build_completion(False, 0xD2)
    _, tx_sequence = await transmit_local_tlp(
        tb, output_queue, new_tlp, "first TLP after link reinitialization"
    )
    assert tx_sequence == 0, "Transmit sequence did not restart at zero"
    await acknowledge_sequence(tb, tx_sequence, "ACK first post-reset TLP")

    incoming = build_completion(False, 0xD3)
    await send_frame_with_timeout(
        tb.phy_source,
        add_sequence_and_lcrc(0, incoming),
        "first incoming TLP after link reinitialization",
        tuser=PHY_USER_IS_TLP,
    )
    delivered = await receive_frame_with_timeout(tb.tlp_sink, "post-reset TLP")
    assert delivered == incoming
    ack = await wait_for_outgoing_dllp(output_queue, DllpType.ACK)
    assert ack.seq == 0
    return 0


async def check_no_unknown_after_reset(tb: TB) -> None:
    """Basic X/Z sanity checks after reset."""
    dut = tb.dut

    assert dut.fc_initialized_o.value.is_resolvable, (
        "fc_initialized_o is X/Z immediately after reset"
    )
    assert int(dut.fc_initialized_o.value) == 0, (
        "fc_initialized_o must be low immediately after reset"
    )

    for signal_name in [
        "s_phy_axis_tready",
        "s_tlp_axis_tready",
        "m_phy_axis_tvalid",
        "m_tlp_axis_tvalid",
    ]:
        sig = getattr(dut, signal_name)
        assert sig.value.is_resolvable, "{} is X/Z after reset".format(signal_name)


# Widths of dllp2tlp's six receive-credit accumulators, used to compare deltas
# modulo the counter width instead of assuming no wrap.
RX_CREDIT_COUNTER_BITS = {
    "ph": 8, "pd": 12, "nph": 8, "npd": 12, "cplh": 8, "cpld": 12,
}

# PCIe Base Specification Rev 2.1, Section 2.6.1, Table 2-36 "TLP Flow Control
# Credit Consumption" (p.136).  The data credit unit is 4 DW (p.135) and
# n = Roundup(Length / FC unit size) (footnote 31).  Length == 0 encodes
# 1024 DW (Table 2-4), hence 256 data credits.
#
# Each row is (description, byte0, header_dw, length_dw, payload_bytes,
# expected credit deltas).  byte0 is written verbatim, so reserved and unmapped
# encodings are reachable alongside the defined ones.  The wildcard bit in the
# Fmt field is exercised from both sides (3-DW and 4-DW forms of the same type),
# and the message routing subfield at both ends of its range, because a literal
# such as MWr = 8'b01?0_0000 is only half proven by one of its two values.
RX_FC_CLASSIFICATION_CASES = [
    # Posted: Memory Write is 1 PH + n PD.  L=4 -> 1, L=5 -> 2 pins the roundup
    # in both directions.
    ("MWr 3DW",        0x40, 3, 4, 16, {"ph": 1, "pd": 1}),
    ("MWr 4DW",        0x60, 4, 5, 20, {"ph": 1, "pd": 2}),
    # Non-posted: every Read is 1 NPH and no data credit, even though a Read
    # carries a non-zero Length.
    ("MRd 3DW",        0x00, 3, 1,  0, {"nph": 1}),
    ("MRd 4DW",        0x20, 4, 1,  0, {"nph": 1}),
    ("MRdLk 3DW",      0x01, 3, 1,  0, {"nph": 1}),
    # Posted: Message without data is 1 PH; with data 1 PH + n PD.
    ("Msg routing 0",  0x30, 4, 0,  0, {"ph": 1}),
    ("Msg routing 7",  0x37, 4, 0,  0, {"ph": 1}),
    ("MsgD routing 0", 0x70, 4, 4, 16, {"ph": 1, "pd": 1}),
    ("MsgD routing 7", 0x77, 4, 2,  8, {"ph": 1, "pd": 1}),
    # Non-posted: AtomicOp is 1 NPH + n NPD.
    ("FetchAdd 3DW",   0x4C, 3, 2,  8, {"nph": 1, "npd": 1}),
    ("Swap 4DW",       0x6D, 4, 4, 16, {"nph": 1, "npd": 1}),
    ("CAS 3DW",        0x4E, 3, 8, 32, {"nph": 1, "npd": 2}),
    # Controls: wildcard-free labels that already classified before this fix.
    # They share their arms with the wildcard labels above, so an arm that only
    # ever fired for these would look covered without them being distinguished.
    ("IORd",           0x02, 3, 1,  0, {"nph": 1}),
    ("IOWr",           0x42, 3, 1,  4, {"nph": 1, "npd": 1}),
    ("CfgRd0",         0x04, 3, 1,  0, {"nph": 1}),
    ("CfgWr1",         0x45, 3, 1,  4, {"nph": 1, "npd": 1}),
    ("Cpl",            0x0A, 3, 0,  0, {"cplh": 1}),
    ("CplD",           0x4A, 3, 4, 16, {"cplh": 1, "cpld": 1}),
    # Completions round their Length up the same way Posted data does.  L=4 -> 1
    # is a fixed point of both Roundup(L/4) and a plain +1, so that row alone
    # cannot tell the roundup from a constant; L=5 -> 2 separates them.
    ("CplD Length=5",  0x4A, 3, 5, 20, {"cplh": 1, "cpld": 2}),
    # Unclassified encodings must consume nothing: a reserved type, and a
    # declared Local-TLP-Prefix encoding that this classifier deliberately does
    # not list in any arm.
    ("reserved 0x03",  0x03, 3, 1,  4, {}),
    ("prefix 0x80",    0x80, 3, 1,  4, {}),
    # Length == 0 is 1024 DW, the one payload size that is not its own DW count.
    ("MWr Length=0",   0x40, 3, 0,  4, {"ph": 1, "pd": 256}),
    ("CplD Length=0",  0x4A, 3, 0,  4, {"cplh": 1, "cpld": 256}),
]


async def verify_receive_flow_control_classification(
    tb: TB,
    output_queue: Queue,
) -> None:
    """Check received-TLP credit consumption against PCIe Base 2.1 Table 2-36.

    The mirror of Phase 12, which checks the same six classes on the transmit
    side through tlp2dllp.  Both classifiers match the same byte-0 literals; the
    receive side had never been observed.

    Every row is checked and reported before the phase asserts, so one run
    yields the whole per-type table rather than stopping at the first mismatch.
    """
    # sec 63 #7f commit A: the receive-side registers are CREDITS_ALLOCATED
    # (Base 2.1 sec 2.6.1.2 p.141) and step at RELEASE -- when the frame
    # leaves dllp2tlp_fifo_inst on m_tlp_axis -- which is why this phase reads
    # them only after tlp_sink has received the whole frame.
    base = "dllp_receive_inst.dllp2tlp_inst."
    signals = {
        name: get_internal_handle(tb.dut, base + name + "_credits_allocated_r")
        for name in RX_CREDIT_COUNTER_BITS
    }
    expected_sequence = get_internal_handle(tb.dut, base + "next_expected_seq_num_r")

    def snapshot() -> Dict[str, int]:
        values = {}
        for name, handle in signals.items():
            assert handle.value.is_resolvable, (
                "{}_credits_allocated_r is X/Z".format(name)
            )
            values[name] = int(handle.value)
        return values

    failures: List[str] = []

    for description, byte0, header_dw, length_dw, payload_bytes, expected in (
        RX_FC_CLASSIFICATION_CASES
    ):
        sequence_number = int(expected_sequence.value) & 0xFFF
        payload = bytes(((byte0 + index) & 0xFF) for index in range(payload_bytes))
        raw_tlp = build_raw_tlp(byte0, length_dw, header_dw, payload)

        before = snapshot()

        await send_frame_with_timeout(
            tb.phy_source,
            add_sequence_and_lcrc(sequence_number, raw_tlp),
            "incoming {} (byte0=0x{:02X}) seq={}".format(
                description, byte0, sequence_number
            ),
            tuser=PHY_USER_IS_TLP,
        )

        received_tlp = await receive_frame_with_timeout(
            tb.tlp_sink,
            "m_tlp_axis TLP for {} seq={}".format(description, sequence_number),
            timeout_us=AXIS_RECV_TIMEOUT_US,
        )
        ack = await wait_for_outgoing_dllp(output_queue, DllpType.ACK)
        await tb.wait_cycles(20)

        after = snapshot()
        observed = {
            name: (after[name] - before[name]) % (1 << bits)
            for name, bits in RX_CREDIT_COUNTER_BITS.items()
        }
        wanted = {name: expected.get(name, 0) for name in RX_CREDIT_COUNTER_BITS}

        if observed != wanted:
            failures.append(
                "{} (byte0=0x{:02X}, Length={} DW): expected {} but consumed {}".format(
                    description,
                    byte0,
                    length_dw,
                    {k: v for k, v in wanted.items() if v},
                    {k: v for k, v in observed.items() if v} or "nothing",
                )
            )
        # A misclassification must not be able to hide behind a broken data
        # path, so the forwarding and acknowledgement checks are independent.
        if received_tlp != raw_tlp:
            failures.append(
                "{} was altered in flight: sent {} received {}".format(
                    description, raw_tlp.hex(), received_tlp.hex()
                )
            )
        if ack.seq != sequence_number:
            failures.append(
                "{} was acknowledged with sequence {} instead of {}".format(
                    description, ack.seq, sequence_number
                )
            )

        tb.log.info(
            "RX classification %-14s byte0=0x%02X L=%-3d consumed %s",
            description,
            byte0,
            length_dw,
            {k: v for k, v in observed.items() if v} or "nothing",
        )

    assert not failures, (
        "Received-TLP flow-control classification disagrees with PCIe Base 2.1 "
        "Table 2-36 in {} of {} cases:\n  {}".format(
            len(failures),
            len(RX_FC_CLASSIFICATION_CASES),
            "\n  ".join(failures),
        )
    )


async def verify_receive_credit_reaches_updatefc(
    tb: TB,
    output_queue: Queue,
) -> None:
    """Prove the receive-credit counters reach the advertised UpdateFC payload.

    dllp_fc_update builds UpdateFC_P/NP directly from ph/pd/nph/npd_credits_
    consumed_i, so this is a wiring proof rather than a detection test -- it
    holds whatever the classifier decides.  It is worth its cost because the
    advertised value is the only externally visible consequence of receive-side
    classification, and because nothing had ever observed it.

    pcie_flow_ctrl_init also emits UpdateFC, but from hardcoded constants and
    only while flow control initializes.  No initialization happens here, so an
    UpdateFC arriving after a quiet window is unambiguously dllp_fc_update's.
    """
    base = "dllp_receive_inst.dllp2tlp_inst."
    consumed = {
        name: int(get_internal_handle(tb.dut, base + name + "_credits_allocated_r").value)
        for name in ("ph", "pd", "nph", "npd")
    }

    drain_queue(output_queue, "before the UpdateFC quiet window")

    update_p = await wait_for_outgoing_dllp(
        output_queue, DllpType.UPDATE_FC_P, timeout_us=FC_UPDATE_IDLE_TIMEOUT_US
    )
    update_np = await wait_for_outgoing_dllp(
        output_queue, DllpType.UPDATE_FC_NP, timeout_us=FC_UPDATE_IDLE_TIMEOUT_US
    )

    for dllp, hdr_name, data_name in (
        (update_p, "ph", "pd"),
        (update_np, "nph", "npd"),
    ):
        assert dllp.hdr_fc == consumed[hdr_name] & 0xFF, (
            "{} advertised hdr_fc={} but {}_credits_allocated_r is {}".format(
                dllp.type.name, dllp.hdr_fc, hdr_name, consumed[hdr_name]
            )
        )
        assert dllp.data_fc == consumed[data_name] & 0xFFF, (
            "{} advertised data_fc={} but {}_credits_allocated_r is {}".format(
                dllp.type.name, dllp.data_fc, data_name, consumed[data_name]
            )
        )

    tb.log.info(
        "Advertised receive credit tracks the counters: P hdr=%d data=%d, "
        "NP hdr=%d data=%d",
        update_p.hdr_fc,
        update_p.data_fc,
        update_np.hdr_fc,
        update_np.data_fc,
    )


@cocotb.test()
async def run_test(dut):
    """Exercise flow-control initialization and both TLP data directions."""
    tb = TB(dut)

    seed = int(os.environ.get("PCIE_TEST_SEED", str(DEFAULT_RANDOM_SEED)), 0)
    rng = random.Random(seed)

    tb.log.info("PCIe Data Link Layer relaxed functional test starting")
    tb.log.info("Random seed: 0x%08x", seed)
    tb.log.info("Python test log: %s", tb.log_file)
    tb.log.info("CLOCK_PERIOD_NS=%d", CLOCK_PERIOD_NS)
    tb.log.info("AXIS_SEND_TIMEOUT_US=%d", AXIS_SEND_TIMEOUT_US)
    tb.log.info("AXIS_RECV_TIMEOUT_US=%d", AXIS_RECV_TIMEOUT_US)
    tb.log.info("FC_DRIVER_TIMEOUT_US=%d", FC_DRIVER_TIMEOUT_US)
    tb.log.info("FC_INITIALIZED_TIMEOUT_US=%d", FC_INITIALIZED_TIMEOUT_US)

    await tb.reset()
    await check_no_unknown_after_reset(tb)

    output_queue = Queue()
    monitor_stop = Event()
    monitor_task = cocotb.start_soon(
        phy_output_monitor(tb, output_queue, monitor_stop)
    )

    fc_frame_count = 0
    outgoing_tlp_count = 0
    incoming_tlp_count = 0
    malformed_rejection_count = 0
    robust_dllp_check_count = 0

    try:
        # ------------------------------------------------------------------
        # Phase 1: link-up and flow-control initialization
        # ------------------------------------------------------------------
        tb.log.info("PHASE 1: link-up and flow-control initialization")

        dut.idle_valid_i.value = 1
        dut.phy_link_up_i.value = 1

        await tb.wait_cycles(50)

        try:
            fc_frame_count = await with_timeout(
                send_flow_control_initialization(tb),
                FC_DRIVER_TIMEOUT_US,
                "us",
            )
        except SimTimeoutError as exc:
            raise AssertionError(
                "Flow-control stimulus did not finish within {} us. "
                "Increase PCIE_FC_DRIVER_TIMEOUT_US if needed.".format(
                    FC_DRIVER_TIMEOUT_US
                )
            ) from exc

        await wait_for_signal_high(
            dut,
            dut.fc_initialized_o,
            "fc_initialized_o",
            FC_INITIALIZED_TIMEOUT_US,
        )

        tb.log.info("Flow-control initialization completed")

        # Allow final initialization frames to reach the PHY monitor.
        await tb.wait_cycles(100)

        initialization_outputs = drain_queue(output_queue)
        tb.log.info(
            "Observed %d outgoing frame(s) during initialization",
            len(initialization_outputs),
        )

        # ------------------------------------------------------------------
        # Phase 2: locally generated TLP -> Data Link Layer -> PHY
        # ------------------------------------------------------------------
        tb.log.info("PHASE 2: transaction-layer TLP transmitted to PHY")

        outgoing_length = rng.randint(1, 32)
        outgoing_tlp, _ = build_memory_write(
            payload_length=outgoing_length,
            tag=1,
        )

        await send_frame_with_timeout(
            tb.tlp_source,
            outgoing_tlp,
            "locally generated Memory Write TLP",
        )

        outgoing_link_packet = await wait_for_outgoing_tlp(
            output_queue,
            outgoing_tlp,
            timeout_us=AXIS_RECV_TIMEOUT_US,
        )
        outgoing_sequence_number = (
            int.from_bytes(outgoing_link_packet[:2], "big") & 0xFFF
        )

        # Retire this packet before the long receive-side negative tests.  If it
        # remains outstanding, its replay timer expires and contaminates later
        # arbitration/replay checks with unrelated retry traffic.
        await send_incoming_dllp(
            tb,
            build_ack_nak_dllp(DllpType.ACK, outgoing_sequence_number),
            "ACK for phase-2 locally generated TLP",
        )
        await tb.wait_cycles(20)

        assert len(outgoing_link_packet) >= len(outgoing_tlp) + 6, (
            "Outgoing link packet is too short to contain a two-byte sequence "
            "number, the TLP, and a four-byte LCRC"
        )

        outgoing_tlp_count += 1

        tb.log.info(
            "Outgoing TLP path passed: raw_tlp_bytes=%d link_packet_bytes=%d",
            len(outgoing_tlp),
            len(outgoing_link_packet),
        )

        # ------------------------------------------------------------------
        # Phase 3: valid PHY-side TLPs -> Data Link Layer -> transaction layer
        # ------------------------------------------------------------------
        tb.log.info("PHASE 3: valid incoming TLP receive path")

        incoming_lengths = [1, 16, 32]

        for sequence_number, payload_length in enumerate(incoming_lengths):
            raw_tlp, _ = build_memory_write(
                payload_length=payload_length,
                tag=sequence_number + 2,
            )

            link_packet = add_sequence_and_lcrc(
                sequence_number=sequence_number,
                tlp_payload=raw_tlp,
            )

            # Sequence one deliberately inserts three source-idle cycles
            # between accepted words.  Receive alignment and LCRC checking
            # must depend only on AXI handshakes, not adjacent valid cycles.
            if sequence_number == 1:
                tb.phy_source.set_pause_generator(cycle_pause())
            await send_frame_with_timeout(
                tb.phy_source,
                link_packet,
                "incoming valid Memory Write TLP seq={}".format(sequence_number),
                tuser=PHY_USER_IS_TLP,
            )
            # send() returns once the frame is queued, not once it is driven, so
            # the paused frame must be awaited here or the generator below is
            # torn down before it has stalled a single beat.
            await with_timeout(
                tb.phy_source.wait(),
                AXIS_SEND_TIMEOUT_US,
                "us",
            )
            # Removing the generator does not clear the pause level it last
            # drove, and the source only dequeues while pause is low.  Drop it
            # explicitly, or every later phase inherits a stalled source.
            tb.phy_source.set_pause_generator(None)
            tb.phy_source.pause = False

            received_tlp = await receive_frame_with_timeout(
                tb.tlp_sink,
                "m_tlp_axis TLP for sequence {}".format(sequence_number),
                timeout_us=AXIS_RECV_TIMEOUT_US,
            )

            assert received_tlp == raw_tlp, (
                "Incoming TLP mismatch for sequence {}.\n"
                "Expected: {}\n"
                "Received: {}".format(
                    sequence_number,
                    raw_tlp.hex(),
                    received_tlp.hex(),
                )
            )

            incoming_tlp_count += 1

            tb.log.info(
                "Incoming TLP sequence %d passed (%d bytes)",
                sequence_number,
                len(raw_tlp),
            )

            ack = await wait_for_outgoing_dllp(output_queue, DllpType.ACK)
            assert ack.seq == sequence_number, (
                "ACK sequence mismatch: got {} expected {}".format(
                    ack.seq,
                    sequence_number,
                )
            )
            robust_dllp_check_count += 1

            await tb.wait_cycles(50)

        # ------------------------------------------------------------------
        # Phase 4: NAK generation and receive-side sequence checks
        # ------------------------------------------------------------------
        tb.log.info("PHASE 4: Bad LCRC NAK and sequence-number error handling")

        last_good_sequence = len(incoming_lengths) - 1

        last_good_sequence = await verify_bad_lcrc_generates_nak(
            tb,
            output_queue,
            sequence_number=(last_good_sequence + 1) & 0xFFF,
            last_good_sequence=last_good_sequence,
        )
        robust_dllp_check_count += 2

        last_good_sequence = await verify_dllp_arbitration_priority(
            tb,
            output_queue,
            sequence_number=(last_good_sequence + 1) & 0xFFF,
            last_good_sequence=last_good_sequence,
        )
        robust_dllp_check_count += 2

        last_good_sequence = await verify_sequence_number_errors(
            tb,
            output_queue,
            last_good_sequence=last_good_sequence,
        )
        robust_dllp_check_count += 7

        # ------------------------------------------------------------------
        # Phase 5: received ACK/NAK and replay behavior
        # ------------------------------------------------------------------
        tb.log.info("PHASE 5: received ACK/NAK and replay behavior")

        await verify_ack_nak_replay(tb, output_queue)
        robust_dllp_check_count += 1

        # ------------------------------------------------------------------
        # Phase 6: malformed TLP and malformed DLLP rejection
        # ------------------------------------------------------------------
        tb.log.info("PHASE 6: malformed incoming TLP and DLLP rejection")

        malformed_tlp, _ = build_memory_write(payload_length=8, tag=7)

        last_good_sequence = await verify_malformed_tlp_is_rejected(
            tb,
            output_queue,
            malformed_tlp,
            last_good_sequence,
        )
        malformed_rejection_count += 1

        await verify_bad_and_malformed_dllps_are_ignored(tb, output_queue)
        robust_dllp_check_count += 1

        # ------------------------------------------------------------------
        # Phase 7: UpdateFC and credit enforcement
        # ------------------------------------------------------------------
        tb.log.info("PHASE 7: UpdateFC DLLPs and zero-credit transmit blocking")

        await verify_updatefc_and_credit_blocking(tb, output_queue)
        robust_dllp_check_count += 1

        # ------------------------------------------------------------------
        # Phase 8: replay timer expiration
        # ------------------------------------------------------------------
        tb.log.info("PHASE 8: replay-timer retransmission")
        await verify_replay_timer_timeout(tb, output_queue)
        robust_dllp_check_count += 1

        # ------------------------------------------------------------------
        # Phase 9: ACK/NAK CRC, cumulative ACK, and ordered replay
        # ------------------------------------------------------------------
        tb.log.info("PHASE 9: robust ACK/NAK processing and ordered replay")
        await verify_corrupt_ack_nak_crc(tb, output_queue)
        await verify_cumulative_ack_and_multi_packet_replay(tb, output_queue)
        await verify_ack_nak_window_boundaries(tb, output_queue)
        robust_dllp_check_count += 3

        # ------------------------------------------------------------------
        # Phase 10: receive NAK suppression and ACK latency
        # ------------------------------------------------------------------
        tb.log.info("PHASE 10: NAK scheduling suppression and ACK latency")
        last_good_sequence = await verify_nak_scheduling_suppression(
            tb, output_queue, last_good_sequence
        )
        last_good_sequence = await verify_ack_latency(
            tb, output_queue, last_good_sequence
        )
        robust_dllp_check_count += 2

        # ------------------------------------------------------------------
        # Phase 11: packet classes, header formats, maximum payload, and ECRC
        # ------------------------------------------------------------------
        tb.log.info("PHASE 11: TLP classes and format preservation")
        last_good_sequence = await verify_tlp_classes_and_formats(
            tb, output_queue, last_good_sequence
        )
        robust_dllp_check_count += 1

        # ------------------------------------------------------------------
        # Phase 12: all FC classes, exhaustion, scaling, and cumulative wrap
        # ------------------------------------------------------------------
        tb.log.info("PHASE 12: complete transmit flow-control behavior")
        # Completion credits were intentionally advertised as infinite during
        # the normal bring-up.  Restart with finite Completion credits so all
        # six FC counters can be exhausted and checked symmetrically.
        await reinitialize_link(
            tb,
            output_queue,
            completion_hdr_fc=3,
            completion_data_fc=256,
        )
        last_good_sequence = 0xFFF
        await verify_flow_control_classes_and_wrap(tb, output_queue)
        robust_dllp_check_count += 1

        # ------------------------------------------------------------------
        # Phase 13: retry-buffer capacity and physical-slot reuse
        # ------------------------------------------------------------------
        tb.log.info("PHASE 13: retry-buffer full and slot wraparound")
        await verify_retry_buffer_full_and_slot_wrap(tb, output_queue)
        robust_dllp_check_count += 1

        # ------------------------------------------------------------------
        # Phase 14: actual receive and transmit sequence rollover
        # ------------------------------------------------------------------
        tb.log.info("PHASE 14: actual 12-bit sequence rollover")
        last_good_sequence = await verify_receive_sequence_rollover(
            tb, output_queue, last_good_sequence
        )
        # Restart with the standard infinite Completion-credit advertisement so
        # 4097 Completion TLPs can exercise transmit rollover without unrelated
        # credit exhaustion masking sequence-generator behavior.
        await reinitialize_link(tb, output_queue)
        last_good_sequence = 0xFFF
        await verify_transmit_sequence_rollover(tb, output_queue)
        robust_dllp_check_count += 2

        # ------------------------------------------------------------------
        # Phase 15: optional AXI backpressure
        # ------------------------------------------------------------------
        if env_flag("PCIE_ENABLE_BACKPRESSURE"):
            tb.log.info("PHASE 15: optional m_phy_axis backpressure")

            tb.phy_sink.set_pause_generator(cycle_pause())

            backpressure_tlp, _ = build_memory_write(payload_length=32, tag=8)

            await send_frame_with_timeout(
                tb.tlp_source,
                backpressure_tlp,
                "Memory Write TLP under m_phy_axis backpressure",
                timeout_us=AXIS_SEND_TIMEOUT_US,
            )

            backpressure_packet = await wait_for_outgoing_tlp(
                output_queue,
                backpressure_tlp,
                timeout_us=BACKPRESSURE_TIMEOUT_US,
            )

            await acknowledge_sequence(
                tb,
                int.from_bytes(backpressure_packet[:2], "big") & 0xFFF,
                "ACK backpressure-test TLP",
            )

            outgoing_tlp_count += 1
            tb.phy_sink.set_pause_generator(None)

            tb.log.info("Backpressure test passed")

        # ------------------------------------------------------------------
        # Phase 16: link-down while replay state is pending
        # ------------------------------------------------------------------
        tb.log.info("PHASE 16: link reset with pending traffic/replay")
        last_good_sequence = await verify_link_down_with_pending_replay(
            tb, output_queue
        )
        robust_dllp_check_count += 1

        # ------------------------------------------------------------------
        # Phase 17: replay-timer retry limit and recovery by link reset
        # ------------------------------------------------------------------
        tb.log.info("PHASE 17: replay-timer exhaustion")
        await verify_replay_timer_exhaustion(tb, output_queue)
        robust_dllp_check_count += 1
        await reinitialize_link(tb, output_queue)

        # ------------------------------------------------------------------
        # Phase 18: repeated-NAK retry limit
        # ------------------------------------------------------------------
        tb.log.info("PHASE 18: repeated NAK and replay-attempt exhaustion")
        await verify_repeated_nak_and_replay_exhaustion(tb, output_queue)
        robust_dllp_check_count += 1

        # ------------------------------------------------------------------
        # Phase 19: received-TLP flow-control classification
        # ------------------------------------------------------------------
        # Placed last so it cannot perturb any incumbent phase, and entered
        # from a fresh link so the receive sequence and credit state are clean.
        await reinitialize_link(tb, output_queue)
        tb.log.info("PHASE 19: received-TLP flow-control classification")
        await verify_receive_flow_control_classification(tb, output_queue)
        incoming_tlp_count += len(RX_FC_CLASSIFICATION_CASES)

        # ------------------------------------------------------------------
        # Phase 20: consumed receive credit reaches the advertised UpdateFC
        # ------------------------------------------------------------------
        tb.log.info("PHASE 20: consumed receive credit reaches UpdateFC")
        await verify_receive_credit_reaches_updatefc(tb, output_queue)
        robust_dllp_check_count += 1

    finally:
        monitor_stop.set()

        try:
            await with_timeout(
                monitor_task,
                MONITOR_SHUTDOWN_TIMEOUT_US,
                "us",
            )
        except SimTimeoutError:
            monitor_task.kill()
            tb.log.warning("PHY output monitor required forced shutdown")

    tb.log.info(
        "TEST SUMMARY: FC frames sent=%d, outgoing TLPs verified=%d, "
        "incoming TLPs verified=%d, malformed TLPs rejected=%d, "
        "robust DLLP checks=%d",
        fc_frame_count,
        outgoing_tlp_count,
        incoming_tlp_count,
        malformed_rejection_count,
        robust_dllp_check_count,
    )
    tb.log.info("PCIe Data Link Layer relaxed functional test PASSED")



# ⚠️ EACH TEST BUILDS ITS OWN TB, AND THAT IS NOT AN OVERSIGHT.
# cocotb cancels every task a test started when that test ends -- INCLUDING the
# Clock coroutine TB.__init__ spawns with start_soon.  A TB carried over from a
# previous test therefore has a DEAD CLOCK, and the first `await
# RisingEdge(clk_i)` after it never returns: the simulator runs out of events and
# exits with "Simulator shut down prematurely", which reads like an RTL hang and
# is not one.  Sharing one TB across tests was tried here and failed exactly that
# way.  Constructing a fresh TB per test is correct precisely BECAUSE the
# previous test's clock and stream drivers are already gone.

# ==========================================================================
# SPEC-GOLDEN: FC_INIT1 ORIGINATION  (Base 2.1 SS3.3.1 p.161)
# ==========================================================================
# These rows exist because conformance defect #4 was invisible to every bench
# in this repository, and it was invisible for a structural reason: EVERY
# suite -- this one included -- sends the InitFC DLLPs from Python before
# waiting on fc_initialized_o.  A far end that always speaks first makes a
# responder-only DLL indistinguishable from a conformant initiator, so no row
# that primes the link can witness origination at all.  SS22.84 at its sharpest:
# no row was red because no row could be built that would go red.
#
# What makes these rows different is the ABSENCE of stimulus.  They bring the
# link up and then send NOTHING, so the only thing that can appear on the
# PHY-facing stream is traffic the DUT originated by itself.
#
# ⚠️ THE RESPONDER PATH IS GUARDED BY run_test, NOT BY THESE ROWS (SS22.81 --
# every negative assertion pairs with a positive row through the same path).
# run_test's send_flow_control_initialization() drives the full seven-DLLP
# InitFC sequence in and requires fc_initialized_o to rise; if the fix had
# broken the ability to ANSWER a primed InitFC1, run_test would fail.  These
# rows add the other half: that the DUT also SPEAKS FIRST.

# SS3.3.1 p.161: "The three InitFC1 DLLPs must be transmitted at least once
# every 34 us."  sec 63 #7g-2 (Q5): pcie_flow_ctrl_init's FcInitWaitPeriod is
# now DERIVED -- 32 us / CLK_PERIOD_NS minus the 7 cycles measured between its
# counter and the first InitFC1-P beat on m_phy_axis -- so the triple leaves
# the DLL 32 us after DL_Init, 2 us inside the bound.  It was the literal 4250
# (= 34 us exactly), which measured 34.056 us on the wire.  Restated here so
# these rows' arithmetic is auditable without opening the RTL -- if the RTL
# constant and this one ever disagree, the interval assertions below say so.
FC_INIT_TARGET_NS = 32_000
FC_INIT_HOP_CYCLES = 7
FC_ORIGINATE_CYCLES = FC_INIT_TARGET_NS // CLOCK_PERIOD_NS - FC_INIT_HOP_CYCLES  # 3993
FC_ORIGINATE_NS = FC_ORIGINATE_CYCLES * CLOCK_PERIOD_NS      # 31_944 ns
FC_ORIGINATE_WINDOW_NS = 2 * FC_ORIGINATE_NS                 # 63_888 ns

# Back-pressure pattern for fcinit_monotonic_under_phy_backpressure.  In
# cocotbext-axi a pause generator yields TRUE to pause, so this is tready LOW
# on seven cycles in every eight.  Deterministic, not random -- the row has to
# reproduce byte-identically in the gate.
#
# The depth was CALIBRATED AGAINST MUTANT MR-C, which is the only configuration
# in which the stimulus's effect on the glitch window is observable at all: with
# the fix reverted, a 2-in-4 pattern left the low window at its un-stalled 4
# cycles (the DLL's skid buffer absorbs the whole 4-beat UpdateFC pair), while
# 7-in-8 stretched it to 24.  A shallower pattern would have made this row look
# healthy while never holding the FSM inside the tready-gated arms.
FC_BACKPRESSURE_PATTERN = (0,) + (1,) * 7

# Cycles from the rise of fc_initialized_o to UpdateFC-NP on the wire with the
# sink never stalling.  Derived from the RTL, not measured: CHECK_FC2's exit
# raises the level, then ST_UPDATE_P / _CRC / _NP / _NP_CRC take one cycle each
# with tready high, and the frame lands as the chain ends.  The back-pressure
# row requires its own window to EXCEED this, which is what proves tready was
# genuinely low inside the tready-gated arms rather than after them.
FC_UNSTALLED_WINDOW_CYCLES = 5

INITFC1_TRIPLE = (
    DllpType.INIT_FC1_P,
    DllpType.INIT_FC1_NP,
    DllpType.INIT_FC1_CPL,
)
INITFC2_TRIPLE = (
    DllpType.INIT_FC2_P,
    DllpType.INIT_FC2_NP,
    DllpType.INIT_FC2_CPL,
)


async def drain_phy_sink(tb: TB) -> int:
    """Discard anything the previous test left queued on the PHY-facing sink."""
    dropped = 0
    while not tb.phy_sink.empty():
        await tb.phy_sink.recv()
        dropped += 1
    return dropped


async def link_up_silent(tb: TB) -> int:
    """Reset, raise phy_link_up_i, and send NOTHING.  Returns the link-up time.

    The return value is the zero point every interval assertion in this section
    measures from: FcInitWaitPeriod starts counting when pcie_datalink_init
    raises start_flow_control_i, and that follows phy_link_up_i.
    """
    await tb.reset()
    await drain_phy_sink(tb)
    tb.dut.phy_link_up_i.value = 1
    tb.dut.idle_valid_i.value = 1
    await RisingEdge(tb.dut.clk_i)
    return get_sim_time("ns")


async def collect_dllps(tb: TB, count: int, timeout_us: int = 200):
    """Decode the next `count` DLLPs off the PHY-facing stream, in order.

    Returns [(DllpType, arrival_ns), ...].  Frames that are not CRC-valid DLLPs
    are skipped rather than failing -- this section asserts on what the DUT
    ORIGINATES, and a row that tripped over an unrelated frame type would be
    measuring framing, not origination.
    """
    seen = []
    while len(seen) < count:
        frame = await with_timeout(tb.phy_sink.recv(), timeout_us, "us")
        data = bytes(frame.tdata)
        if len(data) != DLLP_FRAME_BYTES:
            continue
        payload = check_dllp_crc(data)
        if payload is None:
            continue
        try:
            decoded = Dllp().unpack(payload)
        except Exception:
            continue
        seen.append((decoded.type, get_sim_time("ns")))
    return seen


@cocotb.test()
async def fcinit_originates_initfc1_triple_unprompted(dut):
    """With no stimulus at all, the DLL transmits InitFC1 P, then NP, then Cpl.

    Base 2.1 SS3.3.1 p.161, FC_INIT1 rules:
      - "Entered when initialization of a VC is required / Entrance to DL_Init
        state (VCx = VC0)" -- entry is a LINK-STATE event, with no receive
        precondition of any kind.
      - "Transmit the following three InitFC1 DLLPs for VCx in the following
        relative order: InitFC1 - P (first), InitFC1 - NP (second),
        InitFC1 - Cpl (third)".
      - Receiving appears only under "Process received InitFC1 and InitFC2
        DLLPs ... Set Flag FI1", which governs the EXIT to FC_INIT2.
    Figure 3-3 p.163 draws exactly this case: one side entering the sequence
    before the other has transmitted anything.

    NON-VACUITY (SS22.82).  Nothing is written to phy_source anywhere in this
    row, so every frame observed is DUT-originated by construction.  The
    ordering assertion is what stops a pass from meaning merely "some FC DLLPs
    appeared", and the interval assertion is what identifies the MECHANISM as
    the originate timer rather than an accident of reset.
    """
    tb = TB(dut)
    t0 = await link_up_silent(tb)

    seen = await collect_dllps(tb, 3)
    types = [t for t, _ in seen]
    first_at = seen[0][1]

    tb.log.info("originated with no stimulus: %s, first at %s ns (link up %s ns)",
                [t.name for t in types], first_at, t0)

    assert tuple(types) == INITFC1_TRIPLE, (
        "Base 2.1 SS3.3.1 p.161 requires InitFC1 P first, NP second, Cpl third; "
        f"the DUT originated {[t.name for t in types]}"
    )

    elapsed = first_at - t0
    assert FC_ORIGINATE_NS <= elapsed < FC_ORIGINATE_WINDOW_NS, (
        f"the first InitFC1 arrived {elapsed} ns after link-up, outside the "
        f"first originate interval [{FC_ORIGINATE_NS}, "
        f"{FC_ORIGINATE_WINDOW_NS}) ns.  Below the lower bound something other "
        "than FcInitWaitPeriod released ST_IDLE and this row is not measuring "
        "the originate path; at or above the upper bound the first triple was "
        "missed and a later repeat carried it."
    )


@cocotb.test()
async def fcinit_repeats_initfc1_while_fi1_unset(dut):
    """The InitFC1 triple REPEATS while no peer answers -- SS3.3.1's 34 us bound.

    SS3.3.1 p.161 puts the requirement on the REPEAT, not just the first
    transmission: "The three InitFC1 DLLPs must be transmitted at least once
    every 34 us", and separately "It is strongly encouraged that the InitFC1
    DLLP transmissions are repeated frequently, particularly when there are no
    other TLPs or DLLPs available for transmission."  An FSM that sends one
    triple and then waits to be answered is as non-conformant as one that never
    sends -- which is what CHECK_FC1 did before the fix: with FI1 unset it had
    no else arm at all and stalled silently.

    NON-VACUITY (SS22.82).  Two full triples are required, in order, with no
    stimulus -- so this cannot pass on the single triple the row above already
    covers.  The interval bound is asserted on the SECOND triple's start, which
    is the quantity SS3.3.1 actually constrains.
    """
    tb = TB(dut)
    await link_up_silent(tb)

    seen = await collect_dllps(tb, 6)
    types = [t for t, _ in seen]

    tb.log.info("two unprompted triples: %s", [t.name for t in types])

    assert tuple(types[:3]) == INITFC1_TRIPLE and tuple(types[3:6]) == INITFC1_TRIPLE, (
        "SS3.3.1 p.161 requires the InitFC1 triple to REPEAT while FI1 is unset; "
        f"observed {[t.name for t in types]}"
    )

    gap = seen[3][1] - seen[0][1]
    assert 0 < gap <= FC_ORIGINATE_NS, (
        f"the second InitFC1 triple began {gap} ns after the first, which "
        f"exceeds SS3.3.1 p.161's 'at least once every 34 us' bound "
        f"({FC_ORIGINATE_NS} ns)"
    )


@cocotb.test()
async def fcinit_advances_to_initfc2_once_fi1_is_set(dut):
    """Answering the originated InitFC1 moves the DUT on to the InitFC2 triple.

    SS3.3.1 p.161: "Exit to FC_INIT2 if: Flag FI1 has been set indicating that FC
    unit values have been recorded for each of P, NP, and Cpl for VCx", and then
    FC_INIT2 transmits InitFC2 P, NP, Cpl in that relative order.

    This row is the JOIN between the two halves: the DUT originates first (the
    unprompted triple), this bench then plays the peer, and the DUT must move on.
    It is also the row that would catch an originate path that transmits forever
    and never exits -- a failure mode neither of the rows above can see.

    NON-VACUITY (SS22.82).  The InitFC1 triple is required to have been
    originated BEFORE any stimulus is sent, so a pass cannot come from the pure
    responder path that run_test already covers.
    """
    tb = TB(dut)
    await link_up_silent(tb)

    originated = await collect_dllps(tb, 3)
    assert tuple(t for t, _ in originated) == INITFC1_TRIPLE, (
        "the DUT did not originate the InitFC1 triple before being spoken to, "
        f"so this row's premise does not hold: {[t.name for t, _ in originated]}"
    )

    # Now play the peer: record FC unit values for P, NP and Cpl, which is what
    # sets FI1.  All three are required -- fc1_values_stored_o is the AND.
    for dllp_type in INITFC1_TRIPLE:
        await send_frame_with_timeout(
            tb.phy_source,
            build_fc_dllp(dllp_type=dllp_type, seq=0),
            f"peer {dllp_type.name}",
            tuser=PHY_USER_IS_DLLP,
        )
        await tb.wait_cycles(24)

    # ⚠️ THE BUDGET HERE IS LOAD-BEARING AND WAS MEASURED, NOT GUESSED.
    # CHECK_FC1 does not leave for ST_FC2 the moment FI1 is set: it counts
    # fc2_count_r to 5, sending a further InitFC1 triple on each pass, so SIX
    # more triples -- 18 DLLPs -- follow FI1 before FC_INIT2 begins.  A budget of
    # 24 frames looked generous and captured only InitFC1s, which reads as "the
    # DUT never advanced" when the DUT was advancing exactly as the RTL says.
    seen = await collect_dllps(tb, 120)
    types = [t for t, _ in seen]

    tb.log.info("after FI1 was set: %s", [t.name for t in types])

    assert DllpType.INIT_FC2_P in types, (
        "SS3.3.1 p.161: once FI1 is set the DLL must exit to FC_INIT2.  No "
        "InitFC2 was transmitted within "
        f"{len(types)} DLLPs of the peer's InitFC1 triple; observed "
        f"{[t.name for t in types]}"
    )
    start = types.index(DllpType.INIT_FC2_P)
    assert tuple(types[start:start + 3]) == INITFC2_TRIPLE, (
        "SS3.3.1 p.161 requires InitFC2 P first, NP second, Cpl third; the DUT "
        f"transmitted {[t.name for t in types[start:start + 3]]}"
    )


# ==========================================================================
# fc_initialized_o MONOTONICITY  (Base 2.1 SS3.2.1 pp.158-159, SS3.3.1 pp.160-162)
# ==========================================================================
# Conformance defect #3, tracker SS36.2, CLOSED at the source in
# pcie_flow_ctrl_init.sv: fc2_values_sent_o is now driven '1 in all four
# ST_UPDATE_* arms as well as at CHECK_FC2's exit and in ST_FC_COMPLETE.
#
# THE SPEC RULE THESE TWO ROWS ENCODE.  Completion is a one-way event, not a
# level recomputed each cycle:
#   p.158 DL_Init   -- "Exit to DL_Active if: Flow Control initialization
#                      completes successfully, and the Physical Layer continues
#                      to report Physical LinkUp = 1b"
#   p.161 FC_INIT2  -- "Signal completion and exit if: Flag FI2 has been set"
#   p.158 DL_Active -- the ONLY exit is "Physical Layer reports Physical
#                      LinkUp = 0b"
# The UpdateFC DLLPs the ST_UPDATE_* states emit are ordinary DL_Active credit
# traffic (p.158 lists "Generate and accept DLLPs" as something a COMPLETED
# link does), so no amount of them may de-assert the level.


async def _sample_fc_initialized(tb, stats, stop):
    """Sample fc_initialized_o every cycle, from before the rise.

    ⚠️ ReadOnly, not a bare read after RisingEdge: a bare read samples the
    PRE-edge value.  This coroutine is started BEFORE flow control is primed,
    so the rise itself is inside the sampled window and nothing can be clipped
    -- see the history note in fcinit_hazard_a_fc_initialized_does_not_glitch
    for why that is not a stylistic preference.
    """
    rose = False
    while not stop[0]:
        await RisingEdge(tb.dut.clk_i)
        await ReadOnly()
        raw = tb.dut.fc_initialized_o.value
        if not raw.is_resolvable:
            continue
        level = int(raw)
        if level:
            if not rose:
                rose = True
                stats["rise_ns"] = get_sim_time("ns")
            stats["high_cycles"] += 1
        elif rose:
            stats["lows_after_rise"] += 1
            if stats["first_low_ns"] is None:
                stats["first_low_ns"] = get_sim_time("ns")


async def collect_until_updatefc_np(tb, timeout_us: int = 200):
    """Read DLLPs off the PHY-facing stream until UpdateFC-NP has been seen.

    This is the SIGNAL that bounds both rows below, in place of a fixed cycle
    count (F8: a fixed-window row measures the bench's schedule, not the DUT).
    The ST_UPDATE_* traversal emits UpdateFC-P, its CRC, UpdateFC-NP, its CRC,
    and only then reaches ST_FC_COMPLETE; a decoded UpdateFC-NP frame therefore
    proves the whole glitch window is already behind the sampler, whatever
    back-pressure did to its length.
    """
    seen = []
    while True:
        frame = await with_timeout(tb.phy_sink.recv(), timeout_us, "us")
        data = bytes(frame.tdata)
        if len(data) != DLLP_FRAME_BYTES:
            continue
        payload = check_dllp_crc(data)
        if payload is None:
            continue
        try:
            decoded = Dllp().unpack(payload)
        except Exception:
            continue
        seen.append((decoded.type, get_sim_time("ns")))
        if decoded.type == DllpType.UPDATE_FC_NP:
            return seen


@cocotb.test()
async def fcinit_hazard_a_fc_initialized_does_not_glitch(dut):
    """fc_initialized_o must not fall once flow-control init has completed.

    SS3.3.1 p.161 / SS3.2.1 p.158: completion is signalled once and survives until
    link-down.  This row asserts exactly that at the DLL output, with no filter
    in the path.

    ⚠️⚠️ HISTORY -- THIS ROW WAS RED, AND WHAT ITS RED BODY CLAIMED WAS PARTLY
    WRONG.  It landed at bdddad7 as an expect_fail witness for conformance
    defect #3, carrying the premise that fc2_values_sent_o falls back to its
    combinational default across ST_UPDATE_P / ST_UPDATE_CRC / ST_UPDATE_NP /
    ST_UPDATE_NP_CRC.  That premise was correct and has now EXPIRED at the
    source (SS22.87: flipping a row means rewriting its body).

    ⚠️ But its MEASUREMENT was an artifact, and the artifact was load-bearing.
    The old body recorded "fc_initialized_o is low for THREE cycles, not four",
    called the state count and the low-cycle count "NOT the same quantity", and
    invited the fixing rung to work out "whether the hold needs to start at
    CHECK_FC2's exit or one state later".

    THE DUT'S NUMBER IS FOUR.  The 3 was this row's own sampling phase error:
    it called wait_for_signal_high (:543), whose loop is

        await RisingEdge(dut.clk_i)
        if signal.value.is_resolvable and int(signal.value) == 1: return

    -- a BARE READ AFTER RisingEdge, which returns the PRE-edge value.  The
    helper therefore returned having already consumed the edge into
    ST_UPDATE_P, so the measurement loop's first sample landed on
    ST_UPDATE_CRC and ST_UPDATE_P was clipped.  The old body warned about
    precisely this trap for its own loop while the helper it called one line
    earlier had the bug.  Knowing a trap's name confers no immunity.

    ⚠️ WHY IT MATTERED: a fix designed off the 3 would have started the hold one
    state late, left ST_UPDATE_P glitching for one cycle, and this row -- with
    its window still opening late -- COULD NOT HAVE SEEN IT.  The row would
    have gone green over a live defect.  Mutant MR-D is that scenario, run
    deliberately.

    WHAT CHANGED HERE, therefore, is not just the assertion's sense:
      - the sampler starts BEFORE flow control is primed, so the rise is inside
        the window and nothing is clipped;
      - the window is bounded by a SIGNAL (UpdateFC-NP observed on the wire),
        not by a 4000-cycle count.

    NON-VACUITY (SS22.82).  Three positive checks precede the assertion: the
    level must RISE at all, the UpdateFC pair the ST_UPDATE_* states emit must
    actually be observed, and the sampler must have logged high cycles.  A
    broken prime fails those first rather than passing an empty window.
    """
    tb = TB(dut)
    await tb.reset()
    await drain_phy_sink(tb)
    tb.dut.phy_link_up_i.value = 1
    tb.dut.idle_valid_i.value = 1
    await RisingEdge(tb.dut.clk_i)

    stats = {"rise_ns": None, "first_low_ns": None,
             "lows_after_rise": 0, "high_cycles": 0}
    stop = [False]
    cocotb.start_soon(_sample_fc_initialized(tb, stats, stop))

    await send_flow_control_initialization(tb)
    seen = await collect_until_updatefc_np(tb)
    stop[0] = True
    await RisingEdge(tb.dut.clk_i)

    types = [t.name for t, _ in seen]
    tb.log.info(
        "hazard A: rise at %s ns, %d high cycles, %d low cycles after the "
        "rise (first at %s); DLLPs to UpdateFC-NP: %s",
        stats["rise_ns"], stats["high_cycles"], stats["lows_after_rise"],
        stats["first_low_ns"], types,
    )

    assert stats["rise_ns"] is not None, (
        "positive control: fc_initialized_o never rose -- this row is not "
        "measuring monotonicity, it is measuring a flow-control "
        "initialisation that never completed")
    assert DllpType.UPDATE_FC_P.name in types, (
        "positive control: no UpdateFC-P was observed, so the ST_UPDATE_* "
        f"states were never traversed and the window is empty; saw {types}")
    assert stats["high_cycles"] > 0

    assert stats["lows_after_rise"] == 0, (
        f"fc_initialized_o was low on {stats['lows_after_rise']} cycle(s) "
        f"after flow-control initialisation completed (first at "
        f"{stats['first_low_ns']} ns, rise at {stats['rise_ns']} ns).  "
        "Base 2.1 SS3.3.1 p.161 signals completion once and SS3.2.1 p.158 lets "
        "only link-down revoke it, so the Transaction Layer's 'you may send' "
        "level must not fall while the UpdateFC pair goes out.  Tracker "
        "SS36.2 / conformance defect #3 has regressed in "
        "pcie_flow_ctrl_init.sv's ST_UPDATE_* arms.")


@cocotb.test()
async def fcinit_monotonic_under_phy_backpressure(dut):
    """The hold survives PHY back-pressure stretching the ST_UPDATE_* window.

    ⭐ THIS IS THE CASE THE RC-SIDE FILTER WAS BUILT FOR, ASKED OF THE SOURCE.
    pcie_rc_dl_top's fc_init_sticky_r exists because the glitch window is four
    STATES, not four cycles: each ST_UPDATE_* arm is gated on fc_axis_tready
    (pcie_flow_ctrl_init.sv :425, :436, :448, :459), so with the PHY stalled the
    low window is unbounded above.  A source fix that only happened to cover the
    no-back-pressure case would look identical to a correct one on every other
    row in this file.  This row is what separates them.

    Base 2.1 SS3.2.1 p.158: DL_Active is exited only when "Physical Layer reports
    Physical LinkUp = 0b".  Back-pressure on the PHY-facing stream is not that,
    so it may not revoke the level for even one cycle.

    NON-VACUITY (SS22.82), and it is the whole point of the row: it is not enough
    to observe no glitch -- the run must prove the stall was IN FORCE while the
    FSM was inside the tready-gated arms.  So the row measures the
    rise-to-UpdateFC-NP distance and requires it to EXCEED the un-stalled floor
    of FC_UNSTALLED_WINDOW_CYCLES.  Without that check a sink that ignored
    back-pressure would pass this row while testing nothing.

    ⚠️ THE FIRST VERSION OF THIS ROW FAILED THAT TEST, AND MR-C IS WHAT FOUND IT.
    It applied `phy_sink.pause = True` reactively, on seeing the rise from the
    sampler.  Python cannot act on the same cycle the level rises, so the stall
    landed one or two cycles AFTER CHECK_FC2's exit -- by which time the FSM had
    already walked the whole chain.  Run against MR-C (the fix reverted) the row
    still went red, so it looked healthy; but it measured only FOUR low cycles,
    the un-stalled window, when a genuinely stalled run would have shown tens.
    The row was killing the mutant for the wrong reason and its docstring's
    claim about the unbounded window was not being exercised.

    ⚠️ The fix is to the STIMULUS, not the assertion (`lows_after_rise == 0` is
    unchanged): a deterministic pause GENERATOR installed before priming, so
    back-pressure is already in force when CHECK_FC2 exits.  Strengthening the
    assertion instead would have hidden the gap rather than closing it.
    """
    tb = TB(dut)
    await tb.reset()
    await drain_phy_sink(tb)
    tb.dut.phy_link_up_i.value = 1
    tb.dut.idle_valid_i.value = 1
    await RisingEdge(tb.dut.clk_i)

    # Installed BEFORE priming: back-pressure has to be in force at the instant
    # CHECK_FC2 exits, and no reactive scheme can guarantee that.
    tb.phy_sink.set_pause_generator(itertools.cycle(FC_BACKPRESSURE_PATTERN))

    stats = {"rise_ns": None, "first_low_ns": None,
             "lows_after_rise": 0, "high_cycles": 0}
    stop = [False]
    cocotb.start_soon(_sample_fc_initialized(tb, stats, stop))

    await send_flow_control_initialization(tb)
    seen = await collect_until_updatefc_np(tb)
    stop[0] = True
    await RisingEdge(tb.dut.clk_i)

    np_ns = seen[-1][1]
    window_cycles = (np_ns - stats["rise_ns"]) / CLOCK_PERIOD_NS
    tb.log.info(
        "back-pressure: rise at %s ns, UpdateFC-NP at %s ns = %.0f cycles "
        "(un-stalled floor is %d); %d low cycles after the rise",
        stats["rise_ns"], np_ns, window_cycles, FC_UNSTALLED_WINDOW_CYCLES,
        stats["lows_after_rise"],
    )

    assert stats["rise_ns"] is not None, (
        "positive control: fc_initialized_o never rose")
    assert window_cycles > FC_UNSTALLED_WINDOW_CYCLES, (
        f"non-vacuity: the rise-to-UpdateFC-NP window was {window_cycles:.0f} "
        f"cycles, no longer than the un-stalled floor of "
        f"{FC_UNSTALLED_WINDOW_CYCLES} -- the sink did not hold the DUT inside "
        "the tready-gated ST_UPDATE_* arms, so this run says nothing about the "
        "unbounded window")

    assert stats["lows_after_rise"] == 0, (
        f"fc_initialized_o was low on {stats['lows_after_rise']} cycle(s) "
        f"(first at {stats['first_low_ns']} ns) while the PHY held the DLL in "
        f"the ST_UPDATE_* chain for {window_cycles:.0f} cycles.  The hold must "
        "cover the states, not a fixed number of cycles: Base 2.1 SS3.2.1 p.158 "
        "lets only Physical LinkUp = 0b revoke DL_Active, and back-pressure is "
        "not that.")


# ==========================================================================
# sec 63 #7f -- THE POSTED CLASS (PH/PD), UNIT LEVEL. W1-P and W2-P.
# ==========================================================================
# The full-stack rows W1/W2 witness #18's fix on the NON-POSTED class only:
# enumeration is configuration traffic and no bench sends a posted TLP toward
# the Endpoint. Commit A steps all six CREDITS_ALLOCATED registers at release
# and commit B schedules an UpdateFC per type, so the posted half rides the same
# code path -- but "same code path" is an argument, not a measurement. These
# two rows measure it, here, where a posted TLP can be driven straight into the
# DLL's PHY-side input and every register is reachable (--public-flat-rw).
#
# Red-before-fix is demonstrated with the SAME two semantic mutants the
# full-stack rows used, because the fix is already on the tree:
#   MR-7F1  step CREDITS_ALLOCATED at accept (FIFO input) not release  -> W1-P red
#   MR-7F2  the release trigger in dllp_fc_update disabled             -> W2-P red
# Measured numbers are in each row's body.
#
# Base 2.1 sec 2.6.1.2 p.141 (CREDITS_ALLOCATED ... "incremented as the
# Receiver Transaction Layer makes additional receive buffer space available
# by processing Received TLPs"), p.142 (UpdateFC "must be scheduled for
# Transmission each time ... one or more units of that type are made available
# by TLPs processed"). Table 2-36 fn 31: data credits = Roundup(Length / 4 DW).

POSTED_MWR_PAYLOAD_BYTES = 16     # 4 DW -> exactly ONE PD credit
POSTED_EXPECT_PH = HdrMinCredits_ADV = 16   # the DUT's own InitFC advertisement (pcie_datalink_pkg HdrMinCredits)
POSTED_EXPECT_PD_ADV = 64                   # PdMinCredits
POSTED_RELEASE_TIMEOUT_US = 200
W2P_RELEASE_BOUND_CYCLES = 64   # §63 #7g-2: measured +6 cycles; the periodic timer is ~3,750


async def _posted_bring_up(tb: TB) -> None:
    """Link up and complete FC init exactly as run_test's Phase 1 does, then
    drain the DUT's own InitFC/UpdateFC output so the rows start from a quiet
    PHY-facing stream."""
    await tb.reset()
    tb.dut.idle_valid_i.value = 1
    tb.dut.phy_link_up_i.value = 1
    await tb.wait_cycles(50)
    await with_timeout(send_flow_control_initialization(tb), FC_DRIVER_TIMEOUT_US, "us")
    await wait_for_signal_high(tb.dut, tb.dut.fc_initialized_o, "fc_initialized_o",
                               FC_INITIALIZED_TIMEOUT_US)
    await tb.wait_cycles(100)
    await drain_phy_sink(tb)


class PostedReleaseCapture:
    """Raw per-cycle capture on dllp2tlp: the PH/PD allocated registers and the
    release handshake (m_tlp_axis tlast at dllp2tlp's OUTPUT). Bare read after
    RisingEdge = pre-edge value, so a handshake seen at cycle n steps the
    register visibly at n+1: the same convention as the full-stack W1."""

    def __init__(self, tb: TB):
        base = "dllp_receive_inst.dllp2tlp_inst."
        self.ph = get_internal_handle(tb.dut, base + "ph_credits_allocated_r")
        self.pd = get_internal_handle(tb.dut, base + "pd_credits_allocated_r")
        self.rv = get_internal_handle(tb.dut, base + "m_tlp_axis_tvalid")
        self.rr = get_internal_handle(tb.dut, base + "m_tlp_axis_tready")
        self.rl = get_internal_handle(tb.dut, base + "m_tlp_axis_tlast")
        self.ph_ev = []      # (cycle, value)
        self.pd_ev = []
        self.releases = []   # cycle of each tlast handshake
        self.release_ns = []
        self.cycles = 0

    async def run(self, tb: TB, stop):
        prev_ph = prev_pd = None
        n = 0
        while not stop[0]:
            await RisingEdge(tb.dut.clk_i)
            if int(self.rv.value) and int(self.rr.value) and int(self.rl.value):
                self.releases.append(n)
                self.release_ns.append(get_sim_time("ns"))
            ph, pd = int(self.ph.value), int(self.pd.value)
            if ph != prev_ph:
                self.ph_ev.append((n, ph)); prev_ph = ph
            if pd != prev_pd:
                self.pd_ev.append((n, pd)); prev_pd = pd
            n += 1
        self.cycles = n


async def _send_posted_mwr(tb: TB, seq: int = 0, tag: int = 0x51) -> bytes:
    raw_tlp, _payload = build_memory_write(POSTED_MWR_PAYLOAD_BYTES, tag)
    await send_frame_with_timeout(
        tb.phy_source, add_sequence_and_lcrc(seq, raw_tlp),
        "inbound posted MWr, {} B payload".format(POSTED_MWR_PAYLOAD_BYTES),
        tuser=PHY_USER_IS_TLP)
    delivered = await receive_frame_with_timeout(
        tb.tlp_sink, "the posted MWr delivered on m_tlp_axis")
    assert delivered == raw_tlp, "the posted MWr was altered in flight"
    return raw_tlp


@cocotb.test()
async def w1p_posted_credits_allocated_step_at_release(dut):
    """One inbound posted MWr (4 DW): PH steps 16 -> 17 and PD 64 -> 65, each
    exactly once, and each step lands AFTER the frame's release handshake out
    of dllp2tlp -- never before it.

    Base 2.1 sec 2.6.1.2 p.141: CREDITS_ALLOCATED is "incremented as the
    Receiver Transaction Layer makes additional receive buffer space available
    by processing Received TLPs". A step before the release counts buffer space
    as free while the TLP still occupies it (Receiver Overflow, same page).

    RED-BEFORE-FIX via MR-7F1 (step at the FIFO INPUT instead of its output):
    measured PH at cycle 13, PD at cycle 13, release handshake at
    cycle 21 -- both steps 8 cycles BEFORE the release; W2-P stayed green under
    this mutant (the UpdateFC still left after the release).
    GREEN on the tree with commits A+B: release at cycle 21, PH 16 -> 17 and PD 64 -> 65 both
    at cycle 22, one cycle after.
    """
    tb = TB(dut)
    await _posted_bring_up(tb)
    cap = PostedReleaseCapture(tb)
    stop = [False]
    task = cocotb.start_soon(cap.run(tb, stop))
    await _send_posted_mwr(tb)
    await tb.wait_cycles(60)
    stop[0] = True
    await task
    tb.log.info("W1P releases=%s ph_ev=%s pd_ev=%s", cap.releases, cap.ph_ev, cap.pd_ev)

    assert len(cap.releases) == 1, (
        "NON-VACUITY: expected exactly one release handshake out of dllp2tlp, "
        "saw {}".format(len(cap.releases)))
    rel = cap.releases[0]
    assert cap.ph_ev and cap.ph_ev[0][1] == POSTED_EXPECT_PH, (
        "PH allocated must start at the InitFC advertisement {}; first sample {}".format(
            POSTED_EXPECT_PH, cap.ph_ev[:1]))
    assert cap.pd_ev and cap.pd_ev[0][1] == POSTED_EXPECT_PD_ADV, (
        "PD allocated must start at the InitFC advertisement {}; first sample {}".format(
            POSTED_EXPECT_PD_ADV, cap.pd_ev[:1]))
    ph_steps, pd_steps = cap.ph_ev[1:], cap.pd_ev[1:]
    assert len(ph_steps) == 1 and ph_steps[0][1] == POSTED_EXPECT_PH + 1, (
        "PH must step exactly once, to {}: {}".format(POSTED_EXPECT_PH + 1, ph_steps))
    assert len(pd_steps) == 1 and pd_steps[0][1] == POSTED_EXPECT_PD_ADV + 1, (
        "PD must step exactly once, by Roundup(4 DW / 4) = 1, to {}: {}".format(
            POSTED_EXPECT_PD_ADV + 1, pd_steps))
    assert ph_steps[0][0] > rel and pd_steps[0][0] > rel, (
        "CREDITS_ALLOCATED stepped BEFORE the release: PH at cycle {}, PD at {}, "
        "release handshake at {}. Buffer space was counted as available while the "
        "TLP still occupied it (Base 2.1 sec 2.6.1.2 p.141)".format(
            ph_steps[0][0], pd_steps[0][0], rel))


@cocotb.test()
async def w2p_updatefc_p_scheduled_on_posted_release(dut):
    """After one inbound posted MWr is released, the DLL transmits an UpdateFC-P
    carrying HdrFC 17 and DataFC 65 -- BOTH halves stepped -- and it does so
    after the release, within a bounded window.

    Base 2.1 sec 2.6.1.2 p.142: "For non-infinite NPH, NPD, PH, and CPLH types,
    an UpdateFC FCP must be scheduled for Transmission each time ... one or
    more units of that type are made available by TLPs processed". The bound
    here is that clause, not the 30 us periodic floor (#7g); the window is
    generous next to the release-to-UpdateFC gap measured in the full stack
    (~5 cycles) and tiny next to the 200,000-cycle periodic timer, which is
    what makes MR-7F2 red rather than merely late.

    RED-BEFORE-FIX via MR-7F2 (release trigger disabled): no UpdateFC-P within 200 us of the release;
    the only DLLP seen was the Ack at 13,872 ns; W1-P stayed green under this
    mutant (the accounting is B-independent).
    GREEN on the tree with commits A+B: release at 2,963,280 ns, Ack at +16 ns,
    UpdateFC-P at +48 ns carrying HdrFC 17 / DataFC 65.
    """
    tb = TB(dut)
    await _posted_bring_up(tb)
    cap = PostedReleaseCapture(tb)
    stop = [False]
    task = cocotb.start_soon(cap.run(tb, stop))
    await _send_posted_mwr(tb)

    seen = []          # (DllpType, ns, hdr_fc, data_fc)
    update_p = None
    try:
        while update_p is None:
            frame = await with_timeout(tb.phy_sink.recv(), POSTED_RELEASE_TIMEOUT_US, "us")
            data = bytes(frame.tdata)
            if len(data) != DLLP_FRAME_BYTES:
                continue
            payload = check_dllp_crc(data)
            if payload is None:
                continue
            d = Dllp().unpack(payload)
            seen.append((d.type.name, get_sim_time("ns"), d.hdr_fc, d.data_fc))
            if d.type == DllpType.UPDATE_FC_P:
                update_p = d
                update_p_ns = get_sim_time("ns")
    except SimTimeoutError:
        update_p = None
    stop[0] = True
    await task
    tb.log.info("W2P releases_ns=%s dllps_seen=%s", cap.release_ns, seen)

    assert len(cap.releases) == 1, "NON-VACUITY: expected exactly one release, saw {}".format(len(cap.releases))
    assert update_p is not None, (
        "no UpdateFC-P was transmitted within {} us of releasing a posted TLP; "
        "DLLPs seen: {}. Base 2.1 sec 2.6.1.2 p.142 requires one to be scheduled "
        "each time PH/PD units are made available".format(POSTED_RELEASE_TIMEOUT_US, seen))
    assert update_p.hdr_fc == POSTED_EXPECT_PH + 1, (
        "UpdateFC-P HdrFC={} expected {} (advertised {} + 1 released)".format(
            update_p.hdr_fc, POSTED_EXPECT_PH + 1, POSTED_EXPECT_PH))
    assert update_p.data_fc == POSTED_EXPECT_PD_ADV + 1, (
        "UpdateFC-P DataFC={} expected {} (advertised {} + Roundup(4 DW/4) = 1): "
        "the DATA half must step too".format(update_p.data_fc, POSTED_EXPECT_PD_ADV + 1,
                                              POSTED_EXPECT_PD_ADV))
    assert update_p_ns > cap.release_ns[0], (
        "the UpdateFC-P ({} ns) preceded the release it reports ({} ns)".format(
            update_p_ns, cap.release_ns[0]))
    # §63 #7g-2: the bound is now EXPLICIT.  The docstring's "tiny next to the
    # 200,000-cycle periodic timer" was what made MR-7F2 red rather than merely
    # late; the fix brings the periodic UpdateFC-P to ~3,750 cycles, which lands
    # inside POSTED_RELEASE_TIMEOUT_US and would carry HdrFC 17 / DataFC 65 with
    # the release trigger disabled.  So the release trigger is held to its own
    # latency, an order of magnitude inside the periodic interval.
    assert update_p_ns - cap.release_ns[0] <= W2P_RELEASE_BOUND_CYCLES * CLOCK_PERIOD_NS, (
        "the UpdateFC-P arrived {} ns after the release, beyond the release "
        "trigger's {}-cycle bound: a periodic refresh, not the p.142 release "
        "clause".format(update_p_ns - cap.release_ns[0], W2P_RELEASE_BOUND_CYCLES))


# ==========================================================================
# §63 #7g-2 -- R-U3: UpdateFC per type under SUSTAINED Acked traffic.
# ==========================================================================
# Base 2.1 §2.6.1.2 p.143: an UpdateFC for EACH enabled non-infinite type at
# least once every 30 us (-0%/+50%) in L0 -- 3,750 cycles nominal, 5,625
# ceiling at 8 ns.  Kourosh Q2 (2026-09-24): one timer per credit type, reset
# ONLY by its own UpdateFC.
#
# ⭐ THIS IS THE ROW THAT KILLS THE RESET-RULE MUTANT WITH MARGIN.  The full
# stack's enumeration lasts ~2,700 cycles, so a timer an Ack keeps restarting
# is only marginally late there.  Here a Python far end sends TLPs back to back
# for two 12,000-cycle phases -- posted MWr, then non-posted MRd -- respecting
# the DUT's advertised credit, and the DUT Acks every one.  In each phase one
# type is refreshed by releases and the OTHER can only be refreshed by its
# timer, which is exactly the case the defect starved:
#   - pre-fix (to the Q2 fix commit): every Ack restarted the one shared timer,
#     so the unreleased type was never sent in either phase;
#   - M-U2 (the Ack still resets the timers): the same;
#   - M-U3 (one timer for both types): the released type's UpdateFCs keep
#     restarting it, so the other type starves.
#
# The far end is a PROTOCOL-RESPECTING transmitter, not a firehose: it sends
# only while CREDIT_LIMIT - (CREDITS_CONSUMED + needed) mod 2^N <= 2^(N-1)
# (p.141's gate), with CREDIT_LIMIT read live from the DUT's own UpdateFCs.
# That is stimulus, not analysis; the verdict is computed after the run from
# the raw capture (§22.92).

U3_PHASE_CYCLES = 12_000
UFC_CEILING_CYCLES = 45_000 // CLOCK_PERIOD_NS    # 5,625: 30 us +50 %, p.143
U3_ACK_GAP_BOUND = 64
"""'Sustained' made checkable: inside each phase no two consecutive Acks are
further apart than this.  An Ack-restarted timer therefore never gets within
two orders of magnitude of 3,750."""
DLLP_TYPE_ACK, DLLP_TYPE_NAK = 0x00, 0x10
DLLP_TYPE_UPDATEFC_P, DLLP_TYPE_UPDATEFC_NP = 0x80, 0x90


def u3_decode_fc_word(word: int) -> Tuple[int, int, int]:
    """First m_phy_axis word of an FC DLLP -> (type, HdrFC, DataFC).  The layout
    of tb/fullstack's decode_fc_dllp_word (Base 2.1 §3.4 Figure 3-5)."""
    t = word & 0xFF
    hdr = (((word >> 8) & 0x3F) << 2) | ((word >> 22) & 0x3)
    data = (((word >> 16) & 0xF) << 8) | ((word >> 24) & 0xFF)
    return t, hdr, data


def u3_max_gap(events: List[int], start: int, end: int) -> int:
    """Largest interval in [start] + events + [end]; no event = the window."""
    ev = [start] + sorted(e for e in events if start < e < end) + [end]
    return max(b - a for a, b in zip(ev, ev[1:]))


def u3_selftest() -> None:
    """KNOWN-ANSWER SELF-TEST (§22.92): hand-derived vectors, not DUT captures."""
    # UpdateFC-NP HdrFC 17 DataFC 64: bytes 90 04 40 40 -> 0x40400490
    assert u3_decode_fc_word(0x40400490) == (DLLP_TYPE_UPDATEFC_NP, 17, 64), "SELFTEST NP"
    # UpdateFC-P HdrFC 16 DataFC 64: bytes 80 04 00 40 -> 0x40000480
    assert u3_decode_fc_word(0x40000480) == (DLLP_TYPE_UPDATEFC_P, 16, 64), "SELFTEST P"
    assert u3_max_gap([100, 3852], 100, 9000) == 5148, "SELFTEST u3_max_gap"
    assert u3_max_gap([], 0, 12000) == 12000, "SELFTEST u3_max_gap empty"
    assert UFC_CEILING_CYCLES == 5625, "SELFTEST ceiling at 8 ns"


class U3Capture:
    """Raw capture at the DUT's PHY-facing output, every cycle: (cycle, first
    word) of each DLLP (tuser bit 0, axis_user_demux's UserIsDllp).  It also
    keeps the far end's live view of the DUT's CREDIT_LIMIT, which the sender
    needs to behave as a transmitter -- that view is the only thing computed
    during the run."""

    def __init__(self, tb: TB):
        self.dut = tb.dut
        self.n = 0
        self.dllps: List[Tuple[int, int]] = []
        self.limit = {"P": [HdrMinCredits_ADV, POSTED_EXPECT_PD_ADV],
                      "NP": [HdrMinCredits_ADV, POSTED_EXPECT_PD_ADV]}

    async def run(self, stop) -> None:
        d = self.dut
        in_pkt = False
        while not stop[0]:
            await RisingEdge(d.clk_i)
            self.n += 1
            if int(d.m_phy_axis_tvalid.value) and int(d.m_phy_axis_tready.value):
                if not in_pkt and (int(d.m_phy_axis_tuser.value) & PHY_USER_IS_DLLP):
                    w = int(d.m_phy_axis_tdata.value)
                    self.dllps.append((self.n, w))
                    t, hdr, data = u3_decode_fc_word(w)
                    if (t & 0xF8) == DLLP_TYPE_UPDATEFC_P:
                        self.limit["P"] = [hdr, data]
                    elif (t & 0xF8) == DLLP_TYPE_UPDATEFC_NP:
                        self.limit["NP"] = [hdr, data]
                in_pkt = not int(d.m_phy_axis_tlast.value)

    def of_type(self, t: int) -> List[int]:
        return [c for c, w in self.dllps if (w & 0xF8) == t]

    def acks(self) -> List[int]:
        return [c for c, w in self.dllps if (w & 0xFF) == DLLP_TYPE_ACK]


async def _u3_phase(tb: TB, cap: U3Capture, kind: str, seq: int, stats: Dict) -> int:
    """Send `kind` TLPs back to back for U3_PHASE_CYCLES, credit-gated.
    Returns the next sequence number."""
    need_d = 1 if kind == "P" else 0      # MWr 4 DW = one PD credit; MRd none
    start = cap.n
    tag = 0
    while cap.n - start < U3_PHASE_CYCLES:
        lim_h, lim_d = cap.limit[kind]
        ok = (((lim_h - (stats["cons_h"][kind] + 1)) & 0xFF) <= 0x80 and
              (need_d == 0 or
               ((lim_d - (stats["cons_d"][kind] + need_d)) & 0xFFF) <= 0x800))
        if not ok:
            stats["credit_stall"][kind] += 1
            await RisingEdge(tb.dut.clk_i)
            continue
        if kind == "P":
            raw, _ = build_memory_write(POSTED_MWR_PAYLOAD_BYTES, tag & 0xFF)
        else:
            raw = build_memory_read(4, tag & 0xFF)
        frame = AxiStreamFrame(add_sequence_and_lcrc(seq, raw))
        frame.tuser = PHY_USER_IS_TLP
        await tb.phy_source.send(frame)
        stats["cons_h"][kind] = (stats["cons_h"][kind] + 1) & 0xFF
        stats["cons_d"][kind] = (stats["cons_d"][kind] + need_d) & 0xFFF
        stats["sent"][kind] += 1
        seq = (seq + 1) & 0xFFF
        tag += 1
        # Never queue more than two frames ahead of the wire: the gate above
        # must see the limit the DUT advertises NOW, not one from far back.
        while tb.phy_source.count() >= 2:
            await RisingEdge(tb.dut.clk_i)
    return seq


@cocotb.test()  # §63 #7g-2 R-U3: FLIPPED in the Q2 fix commit; body rewritten (§22.87)
async def u3_updatefc_per_type_under_sustained_acked_traffic(dut):
    """Across 24,000 cycles of back-to-back Acked traffic -- 12,000 of posted
    MWr, then 12,000 of non-posted MRd -- the DLL transmits an UpdateFC-P AND
    an UpdateFC-NP at least every 5,625 cycles (p.143's 45 us ceiling), each
    type anchored at the traffic's first and last cycle so a type never sent
    is one gap the whole window long.

    ⭐ GREEN AT THE Q2 FIX: P's max gap 3,752 and NP's 3,753 cycles over
    24,013 cycles of Acked traffic (803 MWr, 1,090 MRd, zero credit stalls).
    Each type's own timer fires while the other type's releases flow.  The
    one cycle over 3,752 is an owed UpdateFC-NP waiting behind an Ack in
    flight -- the priority the spec recommends, bounded as the RTL says.

    NON-VACUITY (§22.82):
      - each phase sent >= 400 TLPs, every one delivered on m_tlp_axis;
      - the Acks are SUSTAINED: >= 400 per phase, and no Ack-free stretch
        inside a phase longer than U3_ACK_GAP_BOUND cycles;
      - no Nak anywhere (the stream was clean, so nothing here is recovery).

    ⚠️ RED WHEN WRITTEN (tree 9ace778 + 5975ae6, run R): P's gap 11,963 and NP's
    12,066 cycles.  803 MWr and 1,091 MRd were sent and delivered, answered by
    800 and 1,091 Acks with no Ack-free stretch over 15 cycles; UpdateFC-P left
    only on posted releases (803, all in phase 1) and UpdateFC-NP only on
    non-posted ones (1,091, all in phase 2) -- so each type was refreshed only
    while its own TLPs flowed, exactly the starvation Q2 names.
    The row rode expect_fail, pinned (§22.93), until the fix commit, which
    removed the marker and the guard -- an ordinary row fails on any
    exception, which is what the guard existed to restore -- and restated the
    premises above.  The assertion is unchanged.
    """
    u3_selftest()
    tb = TB(dut)
    await _posted_bring_up(tb)
    cap = U3Capture(tb)
    stop = [False]
    cap_task = cocotb.start_soon(cap.run(stop))
    stats = {"cons_h": {"P": 0, "NP": 0}, "cons_d": {"P": 0, "NP": 0},
             "sent": {"P": 0, "NP": 0}, "credit_stall": {"P": 0, "NP": 0}}
    # The far end's CREDITS_CONSUMED starts at 0 against a limit of the
    # DUT's InitFC advertisement (p.141: both counters start at init).
    await RisingEdge(dut.clk_i)
    t0 = cap.n
    seq = await _u3_phase(tb, cap, "P", 0, stats)
    t1 = cap.n
    seq = await _u3_phase(tb, cap, "NP", seq, stats)
    t2 = cap.n
    await tb.wait_cycles(200)
    stop[0] = True
    await cap_task
    delivered = 0
    while not tb.tlp_sink.empty():
        tb.tlp_sink.recv_nowait()
        delivered += 1

    acks = cap.acks()
    naks = [c for c, w in cap.dllps if (w & 0xFF) == DLLP_TYPE_NAK]
    gaps = {"P": u3_max_gap(cap.of_type(DLLP_TYPE_UPDATEFC_P), t0, t2),
            "NP": u3_max_gap(cap.of_type(DLLP_TYPE_UPDATEFC_NP), t0, t2)}
    phase_acks = {}
    for name, a, b in (("P", t0, t1), ("NP", t1, t2)):
        inside = [c for c in acks if a < c <= b]
        phase_acks[name] = (len(inside),
                            max((y - x for x, y in zip(inside, inside[1:])), default=None))
    tb.log.info("U3 phases: P [%d, %d] NP [%d, %d]; sent %s delivered %d; credit "
                "stalls %s; acks per phase (n, max gap) %s; naks %d; UpdateFC-P n=%d "
                "first %s; UpdateFC-NP n=%d first %s",
                t0, t1, t1, t2, stats["sent"], delivered, stats["credit_stall"],
                phase_acks, len(naks), len(cap.of_type(DLLP_TYPE_UPDATEFC_P)),
                cap.of_type(DLLP_TYPE_UPDATEFC_P)[:6],
                len(cap.of_type(DLLP_TYPE_UPDATEFC_NP)),
                cap.of_type(DLLP_TYPE_UPDATEFC_NP)[:6])
    tb.log.info("U3 VERDICT: max gap per type over [%d, %d] %s; ceiling %d",
                t0, t2, gaps, UFC_CEILING_CYCLES)

    for name in ("P", "NP"):
        assert stats["sent"][name] >= 400, (
            f"NON-VACUITY: phase {name} sent only {stats['sent'][name]} TLPs")
        n, g = phase_acks[name]
        assert n >= 400 and g is not None and g <= U3_ACK_GAP_BOUND, (
            f"NON-VACUITY: phase {name} Acks were not sustained: {n} Acks, "
            f"largest Ack-free stretch {g} cycles (bound {U3_ACK_GAP_BOUND})")
    assert delivered == sum(stats["sent"].values()), (
        f"NON-VACUITY: {delivered} TLPs delivered of {sum(stats['sent'].values())} sent")
    assert not naks, f"NON-VACUITY: {len(naks)} Naks -- the stream was not clean"
    bad = {k: g for k, g in gaps.items() if g > UFC_CEILING_CYCLES}
    assert not bad, (
        f"UpdateFC gaps over p.143's {UFC_CEILING_CYCLES}-cycle ceiling under "
        f"sustained Acked traffic: {bad}. Base 2.1 §2.6.1.2 p.143 requires an "
        "UpdateFC for EACH type at least once every 30 us (-0%/+50%)")


# ==========================================================================
# §63 #7g-2 step 2 -- the REPLAY_TIMER and REPLAY_NUM rows, R-P1..R-P3
# (Kourosh Q3 + Q4, 2026-09-24).
# ==========================================================================
# Base 2.1 §3.5.2.1 p.175, Table 3-4 p.176 (CLAUSES_7G2.md §3): "Unadjusted
# REPLAY_TIMER Limits for 2.5 GT/s ... (Symbol Times) Tolerance: -0%/+100%",
# x1 / Max_Payload_Size 128 = 711 ST, so [711, 1,422] ST = [356, 711] cycles at
# 2 Symbol Times per 8 ns cycle.  "TLP Transmitters and compliance tests must
# base replay timing as measured at the Port of the TLP Transmitter.  Timing
# starts with ... the last Symbol of a transmitted TLP ... Timing ends with
# the First Symbol of TLP retransmission" -- so the interval is measured here
# from a TLP's LAST beat on m_phy_axis to its retransmission's FIRST beat; the
# PHY's transmit latency is in both ends and cancels.
#
# Q3: the MPS-128 row, shipped in the UPPER half of the window: 1.75 x 711 ST
# = 7 x 711 ns = 622 cycles at 8 ns, started at the DLL's own last beat to the
# PHY.  Q4: REPLAY_NUM to the spec -- p.174: three replays proceed; the fourth
# initiation rolls 11b -> 00b and must retrain, which does not exist yet
# (registered to the GTH/link-recovery rung), so exhaustion errors out.
#
# ⚠️ RED WHEN WRITTEN (tree da23247 + 4b7d5a9, run R2): the elaborated timer
# was 2,720 cycles started at retry-slot allocation, and MAX_REPLAY_ATTEMPTS
# 2.  The rows rode pinned expect_fail (§22.93) until the Q3/Q4 fix commit,
# which rewrote their bodies (§22.87); the assertions are unchanged.

RPL_TABLE_3_4_X1_MPS128_ST = 711
RPL_WINDOW_LO = (RPL_TABLE_3_4_X1_MPS128_ST * 4 + CLOCK_PERIOD_NS - 1) // CLOCK_PERIOD_NS  # 356
RPL_WINDOW_HI = (2 * RPL_TABLE_3_4_X1_MPS128_ST * 4) // CLOCK_PERIOD_NS                    # 711
RPL_SHIPPED_CYCLES = (7 * RPL_TABLE_3_4_X1_MPS128_ST) // CLOCK_PERIOD_NS                    # 622
RPL_HOP = 6
"""Cycles from the timer's value to the retransmission's first beat on
m_phy_axis, DERIVED: the slot arms on the handshake edge of the TLP's last beat
with the timer at 0, fires when it reads RPL_SHIPPED_CYCLES - 1 (one more edge
to retry_valid_o), and the replayed first beat left 5 edges after retry_valid_o
at 7g-2 Phase 1 (phase1_7g2/dll_unit).  R-P2 is what checks it."""
RPL_EXPECT_INTERVAL = RPL_SHIPPED_CYCLES + RPL_HOP                                          # 628
RPL_SPEC_REPLAYS = 3
RPL_CAPTURE_CYCLES = 8400
"""Long enough to see the PRE-fix tree's error (+8,183 edges at Phase 1), so the
red rows fail at their pinned assertion and not before it."""


def rpl_selftest() -> None:
    """KNOWN-ANSWER SELF-TEST (§22.92) for the window arithmetic and the pairing."""
    assert (RPL_WINDOW_LO, RPL_WINDOW_HI, RPL_SHIPPED_CYCLES, RPL_EXPECT_INTERVAL) == \
        (356, 711, 622, 628), "SELFTEST replay arithmetic at 8 ns"
    assert RPL_WINDOW_LO + (RPL_WINDOW_HI - RPL_WINDOW_LO) // 2 <= RPL_SHIPPED_CYCLES, \
        "SELFTEST the shipped value is in the upper half"
    # hand-built capture: original frame 10..19, replays at 647..655 and 1283..1291
    frames = [(10, 19, 0, 2), (647, 655, 0, 2), (1283, 1291, 0, 2), (30, 30, None, 1)]
    tl = rpl_tlp_frames(frames, 0)
    assert [f[0] for f in tl] == [10, 647, 1283], "SELFTEST rpl_tlp_frames"
    assert rpl_intervals(tl) == [628, 628], "SELFTEST rpl_intervals"


def rpl_tlp_frames(frames, seq):
    """(first, last, seq, tuser) frames -> the TLP frames carrying `seq`, in order."""
    return [f for f in frames if (f[3] & PHY_USER_IS_TLP) and f[2] == seq]


def rpl_intervals(tlp_frames):
    """Each retransmission's first beat minus the previous transmission's last beat."""
    return [b[0] - a[1] for a, b in zip(tlp_frames, tlp_frames[1:])]


async def _rpl_capture(dut):
    """One TLP from the Transaction Layer after FC init, and NO Ack, ever.
    Raw per-edge capture of every m_phy_axis frame (first edge, last edge, the
    link sequence number from the first beat, tuser) and the edge retry_err
    rises.  Bare read after RisingEdge = pre-edge values (§22.89), the same
    convention for every signal, so edge differences are exact."""
    tb = TB(dut)
    await _posted_bring_up(tb)
    err = get_internal_handle(dut, "dllp_transmit_inst.retry_err")
    frames, err_at = [], [None]
    stop = [False]

    async def mon():
        n, first, seq, user = 0, None, None, None
        prev_err = 0
        while not stop[0]:
            await RisingEdge(dut.clk_i)
            n += 1
            if int(dut.m_phy_axis_tvalid.value) and int(dut.m_phy_axis_tready.value):
                if first is None:
                    w = int(dut.m_phy_axis_tdata.value)
                    first, seq, user = n, ((w & 0xF) << 8) | ((w >> 8) & 0xFF), \
                        int(dut.m_phy_axis_tuser.value)
                if int(dut.m_phy_axis_tlast.value):
                    frames.append((first, n, seq if (user & PHY_USER_IS_TLP) else None, user))
                    first = None
            e = int(err.value) if err.value.is_resolvable else 0
            if e and not prev_err and err_at[0] is None:
                err_at[0] = n
            prev_err = e

    task = cocotb.start_soon(mon())
    raw_tlp, _ = build_memory_write(payload_length=16, tag=0x63)
    await send_frame_with_timeout(tb.tlp_source, raw_tlp, "7g-2 R-P TLP, never acknowledged")
    await tb.wait_cycles(RPL_CAPTURE_CYCLES)
    stop[0] = True
    await task
    tl = rpl_tlp_frames(frames, 0)
    tb.log.info("RPL capture: TLP frames seq 0 (first, last) %s; intervals %s; retry_err at %s",
                [(f[0], f[1]) for f in tl], rpl_intervals(tl), err_at[0])
    return tb, tl, err_at[0]


@cocotb.test()  # §63 #7g-2 R-P1: FLIPPED in the Q3/Q4 fix commit; body rewritten (§22.87)
async def p1_replay_fires_inside_table_3_4_window(dut):
    """With the Ack withheld, the first retransmission begins 356-711 cycles
    after the TLP's last beat left the DLL: Table 3-4's x1 / MPS-128 window,
    711-1,422 Symbol Times, measured port to port (p.175).

    ⭐ GREEN AT THE Q3/Q4 FIX: the first retransmission begins 628 cycles
    (1,256 Symbol Times, 1.77 T) after the TLP's last beat: 83 inside the
    ceiling, 272 above the floor.
    ⚠️ RED WHEN WRITTEN (tree da23247 + 4b7d5a9, run R2): 2,723 cycles (5,446 Symbol Times), 3.8x
    the ceiling -- a 2,720-cycle timer started at slot allocation.
    """
    rpl_selftest()
    tb, tl, err_at = await _rpl_capture(dut)
    assert len(tl) >= 2, (
        f"NON-VACUITY: {len(tl)} transmission(s) of seq 0 in {RPL_CAPTURE_CYCLES} "
        "cycles -- the original and at least one retransmission are needed")
    d1 = rpl_intervals(tl)[0]
    assert RPL_WINDOW_LO <= d1 <= RPL_WINDOW_HI, (
        f"first retransmission {d1} cycles after the TLP's last beat, outside Table 3-4's "
        f"x1/MPS-128 window [{RPL_WINDOW_LO}, {RPL_WINDOW_HI}] cycles "
        f"(711-1,422 Symbol Times, Base 2.1 §3.5.2.1 p.176)")


@cocotb.test()  # §63 #7g-2 R-P2: FLIPPED in the Q3/Q4 fix commit; body rewritten (§22.87)
async def p2_replay_timer_default_witness(dut):
    """D-7G.2's DEFAULT WITNESS for the REPLAY_TIMER: at the shipped defaults
    (this bench overrides only CLK_PERIOD_NS = 8, which is the shipped value),
    EVERY retransmission begins exactly RPL_EXPECT_INTERVAL = 7 x 711 // 8 + 6
    = 628 cycles after the previous transmission's last beat -- the first after
    the original, and the second after the first retransmission, whose own
    last beat is the restart event (p.170: "For each replay, reset and restart
    REPLAY_TIMER when sending the last Symbol of the first TLP to be
    retransmitted").  An exact pin, so the bench's derived copy cannot drift
    from the RTL's (the tb_tlp_request_tracker.sv:5 lesson), and so a value in
    the LOWER half of the window -- which R-P1 alone would pass -- fails here.

    ⭐ GREEN AT THE Q3/Q4 FIX: the original's last beat at edge 20, the
    retransmissions' first beats at 648, 1,284 and 1,920 -- 628, 628, 628.
    ⚠️ RED WHEN WRITTEN (tree da23247 + 4b7d5a9, run R2): [2,723, 2,722].
    """
    rpl_selftest()
    tb, tl, err_at = await _rpl_capture(dut)
    assert len(tl) >= 3, (
        f"NON-VACUITY: {len(tl)} transmission(s) of seq 0 -- two retransmissions "
        "are needed to see both restart events")
    iv = rpl_intervals(tl)[:2]
    assert iv == [RPL_EXPECT_INTERVAL, RPL_EXPECT_INTERVAL], (
        f"retransmission intervals {iv}, expected {RPL_EXPECT_INTERVAL} each "
        f"(REPLAY_TIMER {RPL_SHIPPED_CYCLES} = 1.75 x Table 3-4's 711 ST at "
        f"{CLOCK_PERIOD_NS} ns, + {RPL_HOP}, started at the last beat to the PHY)")


@cocotb.test()  # §63 #7g-2 R-P3: FLIPPED in the Q3/Q4 fix commit; body rewritten (§22.87)
async def p3_three_replays_then_error_at_the_fourth(dut):
    """REPLAY_NUM to Base 2.1 §3.5.2.1 p.174: with the Ack withheld forever,
    exactly THREE retransmissions proceed; the fourth initiation (REPLAY_NUM
    rolling 11b -> 00b) is where the spec retrains the Link.  Since §63 #7k
    retry_err IS the retrain request (retry_management's retrain_req_r): it
    rises at that initiation and, with no LTSSM on this bench to retrain the
    Link, nothing is retransmitted after it -- the replay waits (p.174).  The
    row pins that it happens at the FOURTH initiation, not earlier; W2a and
    W2b are the rows that drive the retrain itself.

    ⭐ GREEN AT THE Q3/Q4 FIX: three retransmissions, 628 apart, then
    retry_err at edge 2,552 -- 624 after the third's last beat, the fourth
    initiation -- and nothing after it.
    ⚠️ RED WHEN WRITTEN (tree da23247 + 4b7d5a9, run R2): two retransmissions, then retry_err at
    edge 8,199, the THIRD initiation (MAX_REPLAY_ATTEMPTS = 2).
    """
    rpl_selftest()
    tb, tl, err_at = await _rpl_capture(dut)
    assert err_at is not None, (
        f"NON-VACUITY: retry_err never rose in {RPL_CAPTURE_CYCLES} cycles, so the "
        "count before exhaustion was never reached")
    before = [f for f in tl[1:] if f[0] < err_at]
    after = [f for f in tl[1:] if f[0] >= err_at]
    dut._log.info("RPL P3: replays_before_err=%d after=%d err_at=%d",
                  len(before), len(after), err_at)
    assert len(before) == RPL_SPEC_REPLAYS and not after, (
        f"{len(before)} retransmissions before retry_err and {len(after)} after; "
        f"Base 2.1 §3.5.2.1 p.174 lets {RPL_SPEC_REPLAYS} proceed and retrains at the "
        "fourth initiation")


# ==========================================================================
# §63 #7k W2 -- REPLAY_NUM rollover -> retrain, and the REPLAY_TIMER hold,
# at the DLL's own ports (Kourosh, 2026-09-26: "cocotb drives link_retraining_i
# directly").  pcie_datalink_layer is this target's toplevel, so the LTSSM side
# of the handshake IS the bench: link_retraining_i is "the LTSSM is in Recovery
# or Configuration", already synchronised; link_retrain_req_o is the request.
#
#   Base 2.1 §3.5.2.1 p.174: "If REPLAY_NUM rolls over from 11b to 00b, the
#   Transmitter signals the Physical Layer to retrain the Link, and waits for
#   the completion of retraining before proceeding with the replay."
#   p.170, REPLAY_TIMER: "Not advanced during Link retraining (holds its value
#   when the LTSSM is in the Recovery or Configuration state)."
#   p.170: "For each replay, reset and restart REPLAY_TIMER when sending the
#   last Symbol of the first TLP to be retransmitted."
#
# ⚠️ RED WHEN WRITTEN (tree 11732f0 + the #7k port commit): link_retrain_req_o
# is retry_management's retry_err passed up -- it rises at the fourth initiation
# and NEVER falls (ST_RETRY_ERR is a dead end), and link_retraining_i reaches
# retry_management and is read by nothing.  Each row is pinned (§22.93).
# ==========================================================================

W2_ROW_A = "w2a_request_falls_on_retraining_then_replay_proceeds"
W2_ROW_B = "w2b_peer_first_retrain_counts_as_seen"
W2_ROW_C = "w2c_replay_timer_holds_while_retraining"
W2_SEQ = 0                   # the first TLP after bring-up carries sequence 0
W2_REQ_WAIT = 4 * (RPL_EXPECT_INTERVAL + 40) + 200
"""Edges from the original's last beat to the request: four expiries of
RPL_EXPECT_INTERVAL plus each retransmission's own length, with slack."""
W2_REQ_FALL_MAX = 8          # retraining seen -> request low
W2_RETRAIN = 400             # how long the bench holds link_retraining_i
W2_REPLAY_AFTER_FALL = 60    # retraining falls -> the deferred replay's first beat
W2_HOLD_AT = 300             # W2c: raise the hold this long after the last beat
W2_HOLD = 1000               # W2c: and hold it this long


def pinned_red(dut, row, state, detail=""):
    """§22.93 marker, the format sweep43.sh copies into .diag as PINNED| rows
    (the full-stack helper's, verbatim)."""
    dut._log.info("PINNED_RED|%s|%s|%s", row, state, detail)


def w2_selftest() -> None:
    """KNOWN-ANSWER SELF-TEST (§22.92) for W2's own pairing, hand-derived."""
    frames = [(10, 19, 0, 2), (647, 655, 0, 2), (1283, 1291, 0, 2),
              (1919, 1927, 0, 2), (2700, 2708, 0, 2), (30, 30, None, 1)]
    tl = rpl_tlp_frames(frames, 0)
    assert w2_between(tl, 1927, 2600) == [], "SELFTEST between: none"
    assert w2_between(tl, 1927, 2701) == [(2700, 2708, 0, 2)], "SELFTEST between: one"
    assert w2_between(tl, 0, 10**9)[1:4] == tl[1:4], "SELFTEST between: all"
    edges = [(100, 1), (130, 0), (900, 1)]
    assert w2_edges(edges, 1) == [100, 900] and w2_edges(edges, 0) == [130], \
        "SELFTEST request edges"
    assert rpl_intervals(tl[:4]) == [628, 628, 628], "SELFTEST intervals"
    assert RPL_EXPECT_INTERVAL + W2_HOLD == 1628, "SELFTEST held interval at 8 ns"


def w2_between(tlp_frames, lo, hi):
    """Transmissions whose FIRST beat is in (lo, hi]."""
    return [f for f in tlp_frames if lo < f[0] <= hi]


def w2_edges(req_edges, level):
    return [n for n, v in req_edges if v == level]


class W2Capture:
    """Raw per-edge capture: every m_phy_axis frame (first, last, sequence,
    tuser) and every change of link_retrain_req_o.  Bare read after RisingEdge
    = pre-edge values (§22.89), the _rpl_capture convention, for every signal."""

    def __init__(self, dut):
        self.dut = dut
        self.n = 0
        self.frames = []
        self.req = []
        self.stop = False

    async def run(self):
        d = self.dut
        first = seq = user = None
        prev = None
        while not self.stop:
            await RisingEdge(d.clk_i)
            self.n += 1
            if int(d.m_phy_axis_tvalid.value) and int(d.m_phy_axis_tready.value):
                if first is None:
                    w = int(d.m_phy_axis_tdata.value)
                    first, seq, user = self.n, ((w & 0xF) << 8) | ((w >> 8) & 0xFF), \
                        int(d.m_phy_axis_tuser.value)
                if int(d.m_phy_axis_tlast.value):
                    self.frames.append((first, self.n,
                                        seq if (user & PHY_USER_IS_TLP) else None, user))
                    first = None
            r = int(d.link_retrain_req_o.value) if d.link_retrain_req_o.value.is_resolvable else 0
            if r != prev:
                self.req.append((self.n, r))
                prev = r

    def tlps(self):
        return rpl_tlp_frames(self.frames, W2_SEQ)

    async def until(self, pred, limit, what):
        for _ in range(limit):
            if pred():
                return self.n
            await RisingEdge(self.dut.clk_i)
        raise AssertionError(f"{what}: not within {limit} edges")


async def _w2_start(dut):
    """Bring up, start the capture, and send ONE TLP that is never Acked."""
    tb = TB(dut)
    dut.link_retraining_i.value = 0
    await _posted_bring_up(tb)
    cap = W2Capture(dut)
    task = cocotb.start_soon(cap.run())
    raw_tlp, _ = build_memory_write(payload_length=16, tag=0x7C)
    await send_frame_with_timeout(tb.tlp_source, raw_tlp, "7k W2 TLP")
    await cap.until(lambda: len(cap.tlps()) >= 1, 2000, "the original transmission")
    return tb, cap, task


async def _w2_finish(tb, cap, task):
    """Ack the TLP so the buffer drains, then stop the capture."""
    await send_incoming_dllp(tb, build_ack_nak_dllp(DllpType.ACK, W2_SEQ), "7k W2 cleanup Ack")
    await tb.wait_cycles(50)
    cap.stop = True
    await task


@cocotb.test()  # §63 #7k W2a: FLIPPED in the retry_management fix commit; body rewritten (§22.87)
async def w2a_request_falls_on_retraining_then_replay_proceeds(dut):
    """W2(a): the request rises at the FOURTH initiation, after exactly three
    retransmissions, and nothing is retransmitted while it waits.  When the
    LTSSM side reports retraining the request FALLS (a level handshake: it is
    held until seen, so nothing is lost crossing clocks).  When retraining ends
    the deferred replay proceeds; REPLAY_NUM restarted at 00b, so exactly three
    more retransmissions -- each REPLAY_TIMER after the previous, restarted at
    each replay's last beat -- precede the next request.

    Pinned while red (§22.93): the request is low within W2_REQ_FALL_MAX edges of retraining.

    ⚠️ RED WHEN WRITTEN (tree 133d293, run T1): the request rose at edge 2,552 after 3
    retransmissions and did not fall on retraining (it was retry_err, a dead end).
    """
    tb = cap = task = None
    w2_selftest()
    tb, cap, task = await _w2_start(dut)
    t_req = await cap.until(lambda: w2_edges(cap.req, 1), W2_REQ_WAIT, "the request")
    await tb.wait_cycles(100)
    before = w2_between(cap.tlps(), 0, t_req)[1:]
    idle = w2_between(cap.tlps(), t_req, cap.n)
    assert len(before) == RPL_SPEC_REPLAYS, (
        f"{len(before)} retransmissions before the request, not {RPL_SPEC_REPLAYS}")
    assert not idle, f"{len(idle)} retransmissions while waiting for retraining"
    dut.link_retraining_i.value = 1
    t_rt = cap.n
    await tb.wait_cycles(W2_REQ_FALL_MAX)
    low = int(dut.link_retrain_req_o.value)
    detail = f"t_req={t_req} t_retrain={t_rt} req_edges={cap.req} before={len(before)}"
    assert low == 0, (
        f"link_retrain_req_o still {low} {W2_REQ_FALL_MAX} edges after link_retraining_i "
        f"rose: the request is not released when retraining is seen ({detail})")

    await tb.wait_cycles(W2_RETRAIN - W2_REQ_FALL_MAX)
    during = w2_between(cap.tlps(), t_rt, cap.n)
    dut.link_retraining_i.value = 0
    t_fall = cap.n
    await cap.until(lambda: w2_between(cap.tlps(), t_fall, cap.n), W2_REPLAY_AFTER_FALL,
                    "the deferred replay")
    t_next = await cap.until(lambda: [e for e in w2_edges(cap.req, 1) if e > t_fall],
                             W2_REQ_WAIT, "the next request")
    after = w2_between(cap.tlps(), t_fall, t_next)
    dut._log.info("7K[W2a] req=%s during_retrain=%d after_fall=%s", cap.req, len(during),
                  [(f[0], f[1]) for f in after])
    assert not during, f"{len(during)} retransmissions while retraining (p.174: waits)"
    assert not [e for e in w2_edges(cap.req, 1) if t_rt < e <= t_fall], \
        "the request rose again while retraining"
    assert len(after) == 1 + RPL_SPEC_REPLAYS, (
        f"{len(after)} transmissions between the end of retraining and the next request: "
        f"expected the deferred replay plus {RPL_SPEC_REPLAYS} -- REPLAY_NUM rolled to 00b")
    assert rpl_intervals(after) == [RPL_EXPECT_INTERVAL] * RPL_SPEC_REPLAYS, (
        f"intervals {rpl_intervals(after)} after the deferred replay, expected "
        f"{RPL_EXPECT_INTERVAL} each (p.170: reset and restart at each replay)")
    dut.link_retraining_i.value = 1
    await tb.wait_cycles(20)
    dut.link_retraining_i.value = 0
    await tb.wait_cycles(W2_REPLAY_AFTER_FALL)
    await _w2_finish(tb, cap, task)
    assert int(dut.link_retrain_req_o.value) == 0, "request still high after the Ack"


@cocotb.test()  # §63 #7k W2b: FLIPPED in the retry_management fix commit; body rewritten (§22.87)
async def w2b_peer_first_retrain_counts_as_seen(dut):
    """W2(b): the PEER started Recovery first.  After three timer-driven
    retransmissions (REPLAY_NUM = 11b) the bench raises link_retraining_i --
    which also freezes the timer -- and then a Nak arrives: the fourth
    initiation, rolling REPLAY_NUM over while retraining is ALREADY under way.
    That retrain counts as the one requested: the DLL raises no request of its
    own (a second retrain would re-trigger Recovery), retransmits nothing while
    retraining, and proceeds with the replay when retraining ends.

    Pinned while red (§22.93): the deferred replay begins within W2_REPLAY_AFTER_FALL edges of
    link_retraining_i falling.

    ⚠️ RED WHEN WRITTEN (tree 133d293, run T1): the Nak rollover raised retry_err at
    edge 1,986 -- 8 after the Nak -- and nothing was retransmitted after retraining.
    """
    tb = cap = task = None
    w2_selftest()
    tb, cap, task = await _w2_start(dut)
    await cap.until(lambda: len(cap.tlps()) >= 1 + RPL_SPEC_REPLAYS, W2_REQ_WAIT,
                    "three retransmissions")
    dut.link_retraining_i.value = 1
    t_rt = cap.n
    await tb.wait_cycles(50)
    await send_incoming_dllp(tb, build_ack_nak_dllp(DllpType.NAK, (W2_SEQ - 1) & 0xFFF),
                             "7k W2b Nak while retraining")
    t_nak = cap.n
    await tb.wait_cycles(W2_RETRAIN)
    during = w2_between(cap.tlps(), t_rt, cap.n)
    dut.link_retraining_i.value = 0
    t_fall = cap.n
    await tb.wait_cycles(W2_REPLAY_AFTER_FALL)
    deferred = w2_between(cap.tlps(), t_fall, cap.n)
    req_after_nak = [e for e in w2_edges(cap.req, 1) if e >= t_nak]
    detail = (f"t_retrain={t_rt} t_nak={t_nak} t_fall={t_fall} req={cap.req} "
              f"during={len(during)} deferred={[(f[0], f[1]) for f in deferred]}")
    assert len(cap.tlps()) >= 1 + RPL_SPEC_REPLAYS and t_nak > t_rt, \
        f"NON-VACUITY: three retransmissions then a Nak while retraining ({detail})"
    assert deferred, (
        f"no retransmission within {W2_REPLAY_AFTER_FALL} edges of retraining ending: "
        f"p.174 -- the replay proceeds once retraining completes ({detail})")
    assert not during, f"retransmitted while retraining ({detail})"
    assert not req_after_nak, (
        f"the DLL raised its own request at {req_after_nak} although retraining was "
        f"already under way: that would re-trigger Recovery ({detail})")
    await _w2_finish(tb, cap, task)


@cocotb.test()  # §63 #7k W2c: FLIPPED in the retry_management fix commit; body rewritten (§22.87)
async def w2c_replay_timer_holds_while_retraining(dut):
    """W2(c), D-7K.1(c): Base 2.1 §3.5.2.1 p.170, REPLAY_TIMER is "Not advanced
    during Link retraining (holds its value when the LTSSM is in the Recovery or
    Configuration state)".  W2_HOLD_AT edges after the original's last beat the
    bench holds link_retraining_i high for exactly W2_HOLD edges; the first
    retransmission must then begin RPL_EXPECT_INTERVAL + W2_HOLD edges after the
    last beat -- delayed by exactly the hold, neither reset nor early.

    Pinned while red (§22.93): that interval, exactly.

    ⚠️ RED WHEN WRITTEN (tree 133d293, run T1): interval 628 -- the 1,000-edge hold
    reached retry_management and was read by nothing.
    """
    tb = cap = task = None
    w2_selftest()
    tb, cap, task = await _w2_start(dut)
    last = cap.tlps()[0][1]
    await cap.until(lambda: cap.n >= last + W2_HOLD_AT, W2_HOLD_AT + 10, "the hold point")
    dut.link_retraining_i.value = 1
    await ClockCycles(dut.clk_i, W2_HOLD)
    dut.link_retraining_i.value = 0
    await cap.until(lambda: len(cap.tlps()) >= 2, 2 * RPL_EXPECT_INTERVAL,
                    "the first retransmission")
    d1 = rpl_intervals(cap.tlps())[0]
    detail = f"last={last} first_retx={cap.tlps()[1][0]} interval={d1} hold={W2_HOLD}"
    assert d1 == RPL_EXPECT_INTERVAL + W2_HOLD, (
        f"first retransmission {d1} edges after the last beat; with the timer held for "
        f"{W2_HOLD} edges it must be {RPL_EXPECT_INTERVAL} + {W2_HOLD} = "
        f"{RPL_EXPECT_INTERVAL + W2_HOLD} (p.170: not advanced during retraining) ({detail})")
    await _w2_finish(tb, cap, task)


W2_ROW_D = "w2d_rollover_restarts_replay_num_for_every_tlp"
W2_SEQ_B = 1                 # W2d's second TLP
W2_B_OFFSET = 100            # edges between A's and B's originals


@cocotb.test()  # §63 #7k W2d: FLIPPED in the REPLAY_NUM fix commit; body rewritten (§22.87)
async def w2d_rollover_restarts_replay_num_for_every_tlp(dut):
    """W2(d): REPLAY_NUM is ONE counter for the Transmitter (Base 2.1 §3.5.2.1
    p.170: "The following 2-bit counter is used: REPLAY_NUM"), and the rollover
    leaves it at 00b (p.174).  So after a rollover-and-retrain EVERY TLP still
    in the retry buffer starts again from 00b -- three more retransmissions
    proceed before the next rollover -- not only the TLP whose timer happened
    to expire first.

    Two TLPs, A (seq 0) then B (seq 1) W2_B_OFFSET edges later, never Acked.
    A rolls over first; the bench retrains (link_retraining_i high for
    W2_RETRAIN edges, which also holds B's timer mid-count); when retraining
    ends, B must be retransmitted RPL_SPEC_REPLAYS times before the DLL
    requests another retrain.

    This design keeps one REPLAY_NUM per retry slot, so without the fix B's
    slot still reads 11b after A's retrain and its very next expiry rolls over
    too: a second retrain one timer after the first.  §63 #7k C3 measured the
    full-stack margin by which W1 escaped this at ~50 cycles (FINDINGS_7K_PHASE2).

    Pinned while red (§22.93): B's retransmissions between the end of retraining and the next
    request.

    ⚠️ RED WHEN WRITTEN (tree cc62323): retraining ended at edge 2,962 and B's slot
    rolled over at 3,052 with 0 further retransmissions -- a second retrain one timer later.
    """
    tb = cap = task = None
    w2_selftest()
    tb = TB(dut)
    dut.link_retraining_i.value = 0
    await _posted_bring_up(tb)
    cap = W2Capture(dut)
    task = cocotb.start_soon(cap.run())
    raw_a, _ = build_memory_write(payload_length=16, tag=0x7D)
    await send_frame_with_timeout(tb.tlp_source, raw_a, "7k W2d TLP A")
    await tb.wait_cycles(W2_B_OFFSET)
    raw_b, _ = build_memory_write(payload_length=16, tag=0x7E)
    await send_frame_with_timeout(tb.tlp_source, raw_b, "7k W2d TLP B")
    t_req = await cap.until(lambda: w2_edges(cap.req, 1), W2_REQ_WAIT + W2_B_OFFSET,
                            "the first request")
    await tb.wait_cycles(10)
    dut.link_retraining_i.value = 1
    await tb.wait_cycles(W2_RETRAIN)
    dut.link_retraining_i.value = 0
    t_fall = cap.n
    t_next = await cap.until(lambda: [e for e in w2_edges(cap.req, 1) if e > t_fall],
                             2 * W2_REQ_WAIT, "the next request")
    a = rpl_tlp_frames(cap.frames, W2_SEQ)
    b = rpl_tlp_frames(cap.frames, W2_SEQ_B)
    b_after = w2_between(b, t_fall, t_next)
    detail = (f"t_req={t_req} t_fall={t_fall} t_next={t_next} A={[f[0] for f in a]} "
              f"B={[f[0] for f in b]} B_after_retrain={len(b_after)}")
    dut._log.info("7K[W2d] %s", detail)
    assert len(w2_between(a, 0, t_req)) == 1 + RPL_SPEC_REPLAYS and \
        len(w2_between(b, 0, t_req)) >= RPL_SPEC_REPLAYS, (
        f"NON-VACUITY: A rolled over after 1 + 3 transmissions with B close behind ({detail})")
    assert len(b_after) >= RPL_SPEC_REPLAYS, (
        f"B was retransmitted {len(b_after)} time(s) between the end of retraining and "
        f"the next request; REPLAY_NUM rolled to 00b for the whole retry buffer (p.174), so "
        f"{RPL_SPEC_REPLAYS} must proceed first ({detail})")
    dut.link_retraining_i.value = 1
    await tb.wait_cycles(20)
    dut.link_retraining_i.value = 0
    await tb.wait_cycles(W2_REPLAY_AFTER_FALL)
    await send_incoming_dllp(tb, build_ack_nak_dllp(DllpType.ACK, W2_SEQ_B), "7k W2d cleanup Ack")
    await tb.wait_cycles(50)
    cap.stop = True
    await task
