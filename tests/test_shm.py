# tests/test_shm.py

import time
import ctypes
import pytest

from bus_broker.transport.shm_writer import (
    SHMBusWriter,
    SHMBusReader,
    BusFrameSlot,
    make_slot,
    _lib,
)
from bus_broker.core.frames import CANFrame, Protocol
from bus_broker.core.encoder import CANEncoder
from bus_broker.core.signal import SignalConverter


# ── Helpers ───────────────────────────────────────────────────────────────────

def make_test_slot(arb_id: int = 0x123) -> BusFrameSlot:
    frame = CANFrame(
        arbitration_id = arb_id,
        dlc            = 4,
        data           = bytes([0xDE, 0xAD, 0xBE, 0xEF])
    )
    encoder   = CANEncoder()
    converter = SignalConverter()
    bits      = encoder.encode(frame)
    signal    = converter.to_differential(bits)
    return make_slot(signal, frame)


def make_fresh_writer() -> SHMBusWriter:
    """Force a clean shm and return a writer mapped to it."""
    bus = _lib.shm_reset()
    if not bus:
        raise RuntimeError("shm_reset failed")
    w = SHMBusWriter.__new__(SHMBusWriter)
    w._bus = bus
    return w


def make_reader_at_zero() -> SHMBusReader:
    """Open existing shm and start reading from index 0."""
    r = SHMBusReader.__new__(SHMBusReader)
    r._bus = _lib.shm_open_existing()
    if not r._bus:
        raise RuntimeError("shm_open_existing failed")
    r._consumer_index = ctypes.c_uint64(0)
    return r


# ── Fixtures ──────────────────────────────────────────────────────────────────

@pytest.fixture
def writer():
    w = make_fresh_writer()
    yield w
    _lib.shm_close(w._bus)
    _lib.shm_unlink_bus()
    w._bus = None


@pytest.fixture
def reader(writer):
    # writer fixture has already reset shm - open and start at 0
    r = make_reader_at_zero()
    yield r
    r.close()


@pytest.fixture
def writer_reader():
    # Reset shm once, give both writer and reader access to it
    w = make_fresh_writer()
    r = make_reader_at_zero()
    yield w, r
    r.close()
    _lib.shm_close(w._bus)
    _lib.shm_unlink_bus()
    w._bus = None


# ── Helpers ───────────────────────────────────────────────────────────────────

def make_test_slot(arb_id: int = 0x123) -> BusFrameSlot:
    frame = CANFrame(
        arbitration_id = arb_id,
        dlc            = 4,
        data           = bytes([0xDE, 0xAD, 0xBE, 0xEF])
    )
    encoder   = CANEncoder()
    converter = SignalConverter()
    bits      = encoder.encode(frame)
    signal    = converter.to_differential(bits)
    return make_slot(signal, frame)


# ── Writer ────────────────────────────────────────────────────────────────────

def test_writer_creates_shm(writer):
    assert writer._bus is not None

def test_write_single_frame(writer):
    slot   = make_test_slot()
    result = writer.write(slot)
    assert result is True

def test_write_many_frames(writer):
    for i in range(100):
        slot = make_test_slot(arb_id=i % 0x7FF)
        assert writer.write(slot) is True


# ── Reader ────────────────────────────────────────────────────────────────────

def test_reader_opens_shm(writer):
    # Writer must exist first
    reader = SHMBusReader()
    assert reader._bus is not None
    reader.close()

def test_read_returns_none_when_empty(writer_reader):
    _, reader = writer_reader
    result = reader.read()
    assert result is None

def test_read_after_write(writer_reader):
    writer, reader = writer_reader
    slot = make_test_slot()
    writer.write(slot)
    received = reader.read()
    assert received is not None

def test_read_returns_none_after_consuming_all(writer_reader):
    writer, reader = writer_reader
    writer.write(make_test_slot())
    reader.read()                  # consume it
    assert reader.read() is None   # nothing left


# ── Data integrity ────────────────────────────────────────────────────────────

def test_arbitration_id_preserved(writer_reader):
    writer, reader = writer_reader
    slot = make_test_slot(arb_id=0x456)
    writer.write(slot)
    received = reader.read()
    assert received.arbitration_id == 0x456

def test_bit_count_preserved(writer_reader):
    writer, reader = writer_reader
    slot = make_test_slot()
    writer.write(slot)
    received = reader.read()
    assert received.bit_count == slot.bit_count

def test_canh_bytes_preserved(writer_reader):
    writer, reader = writer_reader
    slot = make_test_slot()
    writer.write(slot)
    received = reader.read()
    assert bytes(received.canh_bytes) == bytes(slot.canh_bytes)

def test_canl_bytes_preserved(writer_reader):
    writer, reader = writer_reader
    slot = make_test_slot()
    writer.write(slot)
    received = reader.read()
    assert bytes(received.canl_bytes) == bytes(slot.canl_bytes)

def test_protocol_preserved(writer_reader):
    writer, reader = writer_reader
    slot = make_test_slot()
    writer.write(slot)
    received = reader.read()
    assert received.protocol == int(Protocol.CAN)

def test_timestamp_preserved(writer_reader):
    writer, reader = writer_reader
    slot            = make_test_slot()
    slot.timestamp_ns = 123456789
    writer.write(slot)
    received = reader.read()
    assert received.timestamp_ns == 123456789


# ── Available count ───────────────────────────────────────────────────────────

def test_available_zero_when_empty(writer_reader):
    _, reader = writer_reader
    assert reader.available() == 0

def test_available_increments_with_writes(writer_reader):
    writer, reader = writer_reader
    for i in range(5):
        writer.write(make_test_slot())
    assert reader.available() == 5

def test_available_decrements_with_reads(writer_reader):
    writer, reader = writer_reader
    writer.write(make_test_slot())
    writer.write(make_test_slot())
    reader.read()
    assert reader.available() == 1


# ── Multiple readers (independent positions) ──────────────────────────────────

def test_two_readers_get_same_frame(writer):
    """Each ECU gets its own copy - readers are independent."""
    reader_a = make_reader_at_zero()
    reader_b = make_reader_at_zero()

    writer.write(make_test_slot(arb_id=0x111))

    recv_a = reader_a.read()
    recv_b = reader_b.read()

    assert recv_a is not None
    assert recv_b is not None
    assert recv_a.arbitration_id == 0x111
    assert recv_b.arbitration_id == 0x111

    reader_a.close()
    reader_b.close()


def test_two_readers_independent_positions(writer):
    """Reader A consuming a frame does not affect Reader B's position."""
    reader_a = make_reader_at_zero()
    reader_b = make_reader_at_zero()

    writer.write(make_test_slot())
    writer.write(make_test_slot())

    reader_a.read()
    reader_a.read()

    assert reader_b.available() == 2

    reader_a.close()
    reader_b.close()


# ── Context manager ───────────────────────────────────────────────────────────

def test_writer_context_manager():
    _lib.shm_reset()
    with SHMBusWriter() as w:
        assert w._bus is not None
    assert w._bus is None
    _lib.shm_unlink_bus()


def test_reader_context_manager():
    _lib.shm_reset()
    with SHMBusWriter() as w:
        r = make_reader_at_zero()
        with r:
            w.write(make_test_slot())
            assert r.read() is not None
    _lib.shm_unlink_bus()


# ── make_slot helper ──────────────────────────────────────────────────────────

def test_make_slot_sets_all_fields():
    frame = CANFrame(
        arbitration_id = 0x123,
        dlc            = 4,
        data           = bytes([1, 2, 3, 4]),
        is_extended    = False,
        is_remote      = False
    )
    encoder   = CANEncoder()
    converter = SignalConverter()
    bits      = encoder.encode(frame)
    signal    = converter.to_differential(bits)

    slot = make_slot(signal, frame, timestamp_ns=999)

    assert slot.arbitration_id == 0x123
    assert slot.protocol       == int(Protocol.CAN)
    assert slot.is_extended    == 0
    assert slot.is_remote      == 0
    assert slot.bit_count      == signal.bit_count
    assert slot.timestamp_ns   == 999
