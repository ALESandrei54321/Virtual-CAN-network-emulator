# tests/test_frames.py

import pytest
from bus_broker.core.frames import CANFrame, Protocol


# ── Happy path ────────────────────────────────────────────────────────────────

def test_standard_frame_ok():
    f = CANFrame(arbitration_id=0x123, dlc=4, data=bytes([0xDE, 0xAD, 0xBE, 0xEF]))
    assert f.arbitration_id == 0x123
    assert f.dlc == 4
    assert f.data_length == 4
    assert f.protocol == Protocol.CAN

def test_extended_frame_ok():
    f = CANFrame(
        arbitration_id=0x12345678,
        dlc=4,
        data=bytes(4),
        is_extended=True
    )
    assert f.arbitration_id == 0x12345678

def test_remote_frame_ok():
    f = CANFrame(arbitration_id=0x100, dlc=4, data=b"", is_remote=True)
    assert f.is_remote

def test_can_fd_frame_ok():
    # DLC=9 means 12 bytes in CAN FD
    f = CANFrame(
        arbitration_id=0x200,
        dlc=9,
        data=bytes(12),
        protocol=Protocol.CAN_FD
    )
    assert f.data_length == 12

def test_zero_data_frame_ok():
    f = CANFrame(arbitration_id=0x001, dlc=0, data=b"")
    assert f.data_length == 0


# ── ID validation ─────────────────────────────────────────────────────────────

def test_standard_id_max_ok():
    f = CANFrame(arbitration_id=0x7FF, dlc=0, data=b"")
    assert f.arbitration_id == 0x7FF

def test_standard_id_overflow():
    with pytest.raises(ValueError, match="out of range"):
        CANFrame(arbitration_id=0x800, dlc=0, data=b"")

def test_extended_id_max_ok():
    f = CANFrame(arbitration_id=0x1FFFFFFF, dlc=0, data=b"", is_extended=True)
    assert f.arbitration_id == 0x1FFFFFFF

def test_extended_id_overflow():
    with pytest.raises(ValueError, match="out of range"):
        CANFrame(arbitration_id=0x20000000, dlc=0, data=b"", is_extended=True)

def test_negative_id():
    with pytest.raises(ValueError, match="out of range"):
        CANFrame(arbitration_id=-1, dlc=0, data=b"")


# ── DLC validation ────────────────────────────────────────────────────────────

def test_dlc_too_large_can():
    with pytest.raises(ValueError, match="DLC"):
        CANFrame(arbitration_id=0x100, dlc=9, data=bytes(9))

def test_dlc_too_large_can_fd():
    with pytest.raises(ValueError, match="DLC"):
        CANFrame(arbitration_id=0x100, dlc=16, data=bytes(16), protocol=Protocol.CAN_FD)

def test_dlc_negative():
    with pytest.raises(ValueError, match="DLC"):
        CANFrame(arbitration_id=0x100, dlc=-1, data=b"")


# ── Data length validation ────────────────────────────────────────────────────

def test_data_too_short():
    with pytest.raises(ValueError, match="Data length"):
        CANFrame(arbitration_id=0x100, dlc=4, data=bytes(3))

def test_data_too_long():
    with pytest.raises(ValueError, match="Data length"):
        CANFrame(arbitration_id=0x100, dlc=4, data=bytes(5))


# ── Protocol rules ────────────────────────────────────────────────────────────

def test_remote_frame_with_data_rejected():
    with pytest.raises(ValueError, match="Remote frames cannot carry data"):
        CANFrame(arbitration_id=0x100, dlc=4, data=bytes(4), is_remote=True)

def test_can_fd_remote_frame_rejected():
    with pytest.raises(ValueError, match="CAN FD does not support remote frames"):
        CANFrame(
            arbitration_id=0x100,
            dlc=0,
            data=b"",
            protocol=Protocol.CAN_FD,
            is_remote=True
        )


# ── Repr ──────────────────────────────────────────────────────────────────────

def test_repr_standard():
    f = CANFrame(arbitration_id=0x123, dlc=4, data=bytes([0xDE, 0xAD, 0xBE, 0xEF]))
    r = repr(f)
    assert "0x123" in r
    assert "DEADBEEF" in r

def test_repr_extended():
    f = CANFrame(arbitration_id=0x12345678, dlc=0, data=b"", is_extended=True)
    assert "[EXT]" in repr(f)

def test_repr_remote():
    f = CANFrame(arbitration_id=0x100, dlc=0, data=b"", is_remote=True)
    assert "[RTR]" in repr(f)
