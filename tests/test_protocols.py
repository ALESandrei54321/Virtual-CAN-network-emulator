# tests/test_protocols.py

import pytest
from bus_broker.core.frames import CANFrame, Protocol
from bus_broker.core.signal import SignalConverter
from bus_broker.protocols.can_protocol import CANProtocol
from bus_broker.protocols.can_fd_protocol import CANFDProtocol
from bus_broker.protocols.registry import get_protocol, list_protocols


# ── Fixtures ──────────────────────────────────────────────────────────────────

@pytest.fixture
def can():
    return CANProtocol()

@pytest.fixture
def can_fd():
    return CANFDProtocol()

@pytest.fixture
def std_frame():
    return CANFrame(
        arbitration_id = 0x123,
        dlc            = 4,
        data           = bytes([0xDE, 0xAD, 0xBE, 0xEF])
    )

@pytest.fixture
def ext_frame():
    return CANFrame(
        arbitration_id = 0x12345678,
        dlc            = 4,
        data           = bytes([0x01, 0x02, 0x03, 0x04]),
        is_extended    = True
    )

@pytest.fixture
def fd_frame():
    return CANFrame(
        arbitration_id = 0x200,
        dlc            = 9,       # 12 bytes in CAN FD
        data           = bytes(12),
        protocol       = Protocol.CAN_FD
    )


# ── BaseProtocol properties ───────────────────────────────────────────────────

def test_can_name(can):
    assert can.name == "CAN"

def test_can_fd_name(can_fd):
    assert can_fd.name == "CAN_FD"

def test_can_default_bit_rate(can):
    assert can.default_bit_rate == 500_000

def test_can_fd_default_bit_rate(can_fd):
    assert can_fd.default_bit_rate == 500_000

def test_can_fd_data_bit_rate(can_fd):
    assert can_fd.data_bit_rate == 2_000_000

def test_can_max_frame_bits(can):
    assert can.max_frame_bits > 0

def test_can_fd_max_frame_bits_larger_than_can(can, can_fd):
    assert can_fd.max_frame_bits > can.max_frame_bits


# ── Encode ────────────────────────────────────────────────────────────────────

def test_can_encode_returns_signal(can, std_frame):
    sig = can.encode(std_frame)
    assert sig.bit_count > 0

def test_can_encode_standard_frame(can, std_frame):
    sig = can.encode(std_frame)
    assert sig.bit_count >= 47   # minimum frame length

def test_can_encode_extended_frame(can, ext_frame):
    sig = can.encode(ext_frame)
    assert sig.bit_count > 0

def test_can_fd_encode_returns_signal(can_fd, fd_frame):
    sig = can_fd.encode(fd_frame)
    assert sig.bit_count > 0

def test_encoded_bits_within_max(can, std_frame):
    sig = can.encode(std_frame)
    assert sig.bit_count <= can.max_frame_bits

def test_fd_encoded_bits_within_max(can_fd, fd_frame):
    sig = can_fd.encode(fd_frame)
    assert sig.bit_count <= can_fd.max_frame_bits


# ── Decode ────────────────────────────────────────────────────────────────────

def test_can_decode_standard_frame(can, std_frame):
    sig      = can.encode(std_frame)
    decoded  = can.decode(sig)
    assert decoded.arbitration_id == std_frame.arbitration_id
    assert decoded.dlc            == std_frame.dlc
    assert decoded.data           == std_frame.data
    assert decoded.is_extended    == std_frame.is_extended

def test_can_decode_extended_frame(can, ext_frame):
    sig     = can.encode(ext_frame)
    decoded = can.decode(sig)
    assert decoded.arbitration_id == ext_frame.arbitration_id
    assert decoded.is_extended    == True
    assert decoded.data           == ext_frame.data

def test_can_decode_zero_data(can):
    frame   = CANFrame(arbitration_id=0x100, dlc=0, data=b"")
    sig     = can.encode(frame)
    decoded = can.decode(sig)
    assert decoded.arbitration_id == 0x100
    assert decoded.dlc            == 0
    assert decoded.data           == b""

def test_can_decode_remote_frame(can):
    frame   = CANFrame(arbitration_id=0x100, dlc=4, data=b"", is_remote=True)
    sig     = can.encode(frame)
    decoded = can.decode(sig)
    assert decoded.is_remote      == True
    assert decoded.arbitration_id == 0x100

def test_can_round_trip_all_dlc(can):
    for dlc in range(9):
        frame   = CANFrame(
            arbitration_id = 0x100,
            dlc            = dlc,
            data           = bytes(range(dlc))
        )
        sig     = can.encode(frame)
        decoded = can.decode(sig)
        assert decoded.dlc  == dlc
        assert decoded.data == bytes(range(dlc))


# ── Arbitration priority ──────────────────────────────────────────────────────

def test_lower_id_higher_priority(can):
    frame_low  = CANFrame(arbitration_id=0x001, dlc=0, data=b"")
    frame_high = CANFrame(arbitration_id=0x7FF, dlc=0, data=b"")
    assert can.arbitration_priority(frame_low) < can.arbitration_priority(frame_high)

def test_same_id_same_priority(can):
    frame_a = CANFrame(arbitration_id=0x123, dlc=0, data=b"")
    frame_b = CANFrame(arbitration_id=0x123, dlc=0, data=b"")
    assert can.arbitration_priority(frame_a) == can.arbitration_priority(frame_b)


# ── Validation ────────────────────────────────────────────────────────────────

def test_can_validate_good_frame(can, std_frame):
    ok, msg = can.validate_frame(std_frame)
    assert ok  is True
    assert msg == ""

def test_can_validate_wrong_protocol(can, fd_frame):
    ok, msg = can.validate_frame(fd_frame)
    assert ok  is False
    assert msg != ""

def test_can_fd_validate_good_frame(can_fd, fd_frame):
    ok, msg = can_fd.validate_frame(fd_frame)
    assert ok  is True
    assert msg == ""

def test_can_fd_validate_wrong_protocol(can_fd, std_frame):
    ok, msg = can_fd.validate_frame(std_frame)
    assert ok  is False

def test_can_fd_validate_remote_frame_rejected(can_fd):
    # CANFrame won't let us create a CAN FD remote frame directly
    # so we test validate_frame with a manually patched frame
    frame = CANFrame(arbitration_id=0x100, dlc=0, data=b"", protocol=Protocol.CAN_FD)
    frame.is_remote = True   # bypass __post_init__ by setting directly
    ok, msg = can_fd.validate_frame(frame)
    assert ok  is False
    assert "remote" in msg.lower()


# ── Registry ──────────────────────────────────────────────────────────────────

def test_list_protocols_contains_can():
    assert "CAN" in list_protocols()

def test_list_protocols_contains_can_fd():
    assert "CAN_FD" in list_protocols()

def test_get_protocol_can():
    proto = get_protocol("CAN")
    assert proto.name == "CAN"

def test_get_protocol_can_fd():
    proto = get_protocol("CAN_FD")
    assert proto.name == "CAN_FD"

def test_get_protocol_with_kwargs():
    proto = get_protocol("CAN", bit_rate=250_000)
    assert proto.default_bit_rate == 250_000

def test_get_protocol_unknown_raises():
    with pytest.raises(ValueError, match="Unknown protocol"):
        get_protocol("FlexRay")

def test_repr_works(can):
    assert "CAN" in repr(can)
