# tests/test_monitor.py

import time
import pytest
from pathlib import Path
from unittest.mock import MagicMock, patch
import sys

sys.path.insert(0, str(Path(__file__).parent.parent / "tools"))

from monitor import (
    FrameRecord,
    MonitorStats,
    SlotDecoder,
    BusMonitor,
    format_frame_line,
)
from bus_broker.transport.shm_writer import (
    SHMBusWriter,
    SHMBusReader,
    BusFrameSlot,
    make_slot,
)
from bus_broker.core.frames import CANFrame, Protocol
from bus_broker.core.encoder import CANEncoder
from bus_broker.core.signal import SignalConverter
from bus_broker.core.bus_controller import BusController
from bus_broker.protocols.can_protocol import CANProtocol


# ── Helpers ───────────────────────────────────────────────────────────────────

def make_real_slot(
    arb_id   : int   = 0x123,
    data     : bytes = bytes([0xDE, 0xAD, 0xBE, 0xEF]),
    protocol : Protocol = Protocol.CAN
) -> BusFrameSlot:
    dlc   = len(data)
    frame = CANFrame(
        arbitration_id = arb_id,
        dlc            = dlc,
        data           = data,
        protocol       = protocol
    )
    encoder   = CANEncoder()
    converter = SignalConverter()
    bits      = encoder.encode(frame)
    signal    = converter.to_differential(bits)
    return make_slot(signal, frame)


def make_frame_record(
    arb_id : int   = 0x123,
    data   : bytes = bytes([0xDE, 0xAD, 0xBE, 0xEF])
) -> FrameRecord:
    slot  = make_real_slot(arb_id=arb_id, data=data)
    frame = CANFrame(
        arbitration_id = arb_id,
        dlc            = len(data),
        data           = data
    )
    return FrameRecord(
        slot         = slot,
        frame        = frame,
        received_ns  = time.time_ns(),
        decode_error = ""
    )


# ── FrameRecord ───────────────────────────────────────────────────────────────

def test_frame_record_age_ms():
    record = make_frame_record()
    # Just created - should be very young
    assert record.age_ms < 100

def test_frame_record_timestamp_str():
    record = make_frame_record()
    ts     = record.timestamp_str
    # Should look like HH:MM:SS.mmm
    assert ":" in ts
    assert "." in ts

def test_frame_record_error():
    slot   = make_real_slot()
    record = FrameRecord(
        slot         = slot,
        frame        = None,
        received_ns  = time.time_ns(),
        decode_error = "CRC mismatch"
    )
    assert record.decode_error == "CRC mismatch"
    assert record.frame is None


# ── MonitorStats ──────────────────────────────────────────────────────────────

def test_stats_initial():
    s = MonitorStats()
    assert s.total_frames  == 0
    assert s.decode_errors == 0

def test_stats_record_frame():
    s      = MonitorStats()
    record = make_frame_record()
    s.record(record)
    assert s.total_frames == 1
    assert s.frames_by_id[0x123] == 1

def test_stats_record_error():
    s    = MonitorStats()
    slot = make_real_slot()
    rec  = FrameRecord(
        slot         = slot,
        frame        = None,
        received_ns  = time.time_ns(),
        decode_error = "bad frame"
    )
    s.record(rec)
    assert s.decode_errors == 1

def test_stats_bytes_tracked():
    s      = MonitorStats()
    record = make_frame_record(data=bytes([1, 2, 3, 4]))
    s.record(record)
    assert s.bytes_by_id[0x123] == 4

def test_stats_frames_per_second():
    s = MonitorStats()
    s.total_frames = 100
    # elapsed will be tiny but nonzero
    fps = s.frames_per_second
    assert fps > 0

def test_stats_print_summary_no_crash(capsys):
    s      = MonitorStats()
    record = make_frame_record()
    s.record(record)
    s.print_summary()
    out = capsys.readouterr().out
    assert "STATISTICS" in out
    assert "0x123" in out


# ── SlotDecoder ───────────────────────────────────────────────────────────────

def test_decode_valid_slot():
    decoder = SlotDecoder()
    slot    = make_real_slot(arb_id=0x123, data=bytes([0xDE, 0xAD, 0xBE, 0xEF]))
    frame, err = decoder.decode(slot)
    assert err   == ""
    assert frame is not None
    assert frame.arbitration_id == 0x123

def test_decode_preserves_data():
    decoder = SlotDecoder()
    data    = bytes([0x01, 0x02, 0x03, 0x04])
    slot    = make_real_slot(data=data)
    frame, err = decoder.decode(slot)
    assert err       == ""
    assert frame.data == data

def test_decode_unknown_protocol():
    decoder          = SlotDecoder()
    slot             = make_real_slot()
    slot.protocol    = 99
    frame, err = decoder.decode(slot)
    assert frame is None
    assert "Unknown protocol" in err

def test_decode_zero_data_frame():
    decoder = SlotDecoder()
    slot    = make_real_slot(data=b"")
    frame, err = decoder.decode(slot)
    assert err        == ""
    assert frame.dlc  == 0
    assert frame.data == b""

def test_decode_various_ids():
    decoder = SlotDecoder()
    for arb_id in [0x001, 0x100, 0x7FF]:
        slot       = make_real_slot(arb_id=arb_id)
        frame, err = decoder.decode(slot)
        assert err == "", f"Decode failed for ID 0x{arb_id:03X}: {err}"
        assert frame.arbitration_id == arb_id


# ── format_frame_line ─────────────────────────────────────────────────────────

def test_format_normal_frame():
    record = make_frame_record(arb_id=0x123)
    line   = format_frame_line(record, verbose=False)
    assert "0x123" in line
    assert "CAN"   in line

def test_format_shows_data():
    record = make_frame_record(data=bytes([0xDE, 0xAD]))
    line   = format_frame_line(record, verbose=False)
    assert "DE" in line
    assert "AD" in line

def test_format_error_frame():
    slot   = make_real_slot()
    record = FrameRecord(
        slot         = slot,
        frame        = None,
        received_ns  = time.time_ns(),
        decode_error = "CRC mismatch"
    )
    line = format_frame_line(record, verbose=False)
    assert "ERROR"       in line
    assert "CRC mismatch" in line

def test_format_verbose_shows_bits():
    record = make_frame_record(data=bytes([0b10101010]))
    line   = format_frame_line(record, verbose=True)
    assert "0b10101010" in line

def test_format_extended_frame():
    slot  = make_real_slot()
    frame = CANFrame(
        arbitration_id = 0x12345678,
        dlc            = 0,
        data           = b"",
        is_extended    = True
    )
    record = FrameRecord(
        slot        = slot,
        frame       = frame,
        received_ns = time.time_ns()
    )
    line = format_frame_line(record, verbose=False)
    assert "[EXT]"      in line
    assert "0x12345678" in line

def test_format_remote_frame():
    slot  = make_real_slot()
    frame = CANFrame(
        arbitration_id = 0x100,
        dlc            = 4,
        data           = b"",
        is_remote      = True
    )
    record = FrameRecord(
        slot        = slot,
        frame       = frame,
        received_ns = time.time_ns()
    )
    line = format_frame_line(record, verbose=False)
    assert "[RTR]" in line


# ── End to end: inject then monitor ──────────────────────────────────────────

@pytest.mark.asyncio
async def test_end_to_end_inject_and_read():
    """
    Full pipeline test:
    CANFrame → BusController → SHM → SlotDecoder → CANFrame
    """
    protocol = CANProtocol()

    with SHMBusWriter() as writer:
        reader  = SHMBusReader()
        decoder = SlotDecoder()

        controller = BusController(protocol, writer)

        frame = CANFrame(
            arbitration_id = 0x123,
            dlc            = 4,
            data           = bytes([0xDE, 0xAD, 0xBE, 0xEF])
        )

        await controller.submit(frame)
        await controller._tick()

        slot = reader.read()
        assert slot is not None

        decoded, err = decoder.decode(slot)
        assert err    == ""
        assert decoded.arbitration_id == frame.arbitration_id
        assert decoded.data           == frame.data
        assert decoded.dlc            == frame.dlc

        reader.close()


@pytest.mark.asyncio
async def test_end_to_end_multiple_frames():
    """Multiple frames survive the full pipeline in order."""
    protocol = CANProtocol()

    frames = [
        CANFrame(arbitration_id=0x100, dlc=1, data=bytes([i]))
        for i in range(5)
    ]

    with SHMBusWriter() as writer:
        reader     = SHMBusReader()
        decoder    = SlotDecoder()
        controller = BusController(protocol, writer)

        for frame in frames:
            await controller.submit(frame)
            await controller._tick()

        for i in range(5):
            slot = reader.read()
            assert slot is not None
            decoded, err = decoder.decode(slot)
            assert err                == ""
            assert decoded.data[0]    == i

        reader.close()
