# tests/test_signal.py

import pytest
from bus_broker.core.signal import DifferentialSignal, SignalConverter
from bus_broker.core.frames import CANFrame, Protocol
from bus_broker.core.encoder import CANEncoder


@pytest.fixture
def converter():
    return SignalConverter()


# ── DifferentialSignal construction ───────────────────────────────────────────

def test_signal_construction_ok(converter):
    sig = converter.to_differential([0, 1, 0, 1])
    assert sig.bit_count == 4
    assert len(sig.canh_bytes) == 1
    assert len(sig.canl_bytes) == 1

def test_signal_mismatched_arrays_rejected():
    with pytest.raises(ValueError, match="same length"):
        DifferentialSignal(
            canh_bytes = bytes(2),
            canl_bytes = bytes(1),
            bit_count  = 8
        )

def test_signal_wrong_byte_count_rejected():
    with pytest.raises(ValueError, match="does not match"):
        DifferentialSignal(
            canh_bytes = bytes(1),
            canl_bytes = bytes(1),
            bit_count  = 16      # needs 2 bytes
        )

def test_empty_bits_rejected(converter):
    with pytest.raises(ValueError, match="empty"):
        converter.to_differential([])


# ── Dominant bit encoding ─────────────────────────────────────────────────────

def test_dominant_bit_sets_canh_only(converter):
    """Dominant (0) → canh=1, canl=0"""
    sig = converter.to_differential([0])
    canh, canl = sig.get_bit(0)
    assert canh == 1
    assert canl == 0

def test_recessive_bit_sets_neither(converter):
    """Recessive (1) → canh=0, canl=0"""
    sig = converter.to_differential([1])
    canh, canl = sig.get_bit(0)
    assert canh == 0
    assert canl == 0

def test_alternating_bits(converter):
    bits = [0, 1, 0, 1, 0, 1, 0, 1]
    sig  = converter.to_differential(bits)
    for i, expected_bit in enumerate(bits):
        canh, canl = sig.get_bit(i)
        if expected_bit == 0:   # dominant
            assert canh == 1 and canl == 0, f"Bit {i} should be dominant"
        else:                    # recessive
            assert canh == 0 and canl == 0, f"Bit {i} should be recessive"


# ── Bit packing ───────────────────────────────────────────────────────────────

def test_eight_bits_pack_into_one_byte(converter):
    sig = converter.to_differential([0] * 8)
    assert len(sig.canh_bytes) == 1

def test_nine_bits_pack_into_two_bytes(converter):
    sig = converter.to_differential([0] * 9)
    assert len(sig.canh_bytes) == 2

def test_msb_first_packing(converter):
    """First bit should be in the MSB of the first byte"""
    # Single dominant bit → canh byte should be 0b10000000 = 0x80
    sig = converter.to_differential([0, 1, 1, 1, 1, 1, 1, 1])
    assert sig.canh_bytes[0] == 0x80

def test_all_dominant_bits(converter):
    """Eight dominant bits → canh byte = 0xFF"""
    sig = converter.to_differential([0] * 8)
    assert sig.canh_bytes[0] == 0xFF
    assert sig.canl_bytes[0] == 0x00

def test_all_recessive_bits(converter):
    """Eight recessive bits → both bytes = 0x00"""
    sig = converter.to_differential([1] * 8)
    assert sig.canh_bytes[0] == 0x00
    assert sig.canl_bytes[0] == 0x00


# ── Round trip ────────────────────────────────────────────────────────────────

def test_round_trip_simple(converter):
    bits = [0, 1, 0, 0, 1, 1, 0, 1]
    sig  = converter.to_differential(bits)
    back = converter.from_differential(sig)
    assert back == bits

def test_round_trip_all_dominant(converter):
    bits = [0] * 16
    sig  = converter.to_differential(bits)
    back = converter.from_differential(sig)
    assert back == bits

def test_round_trip_all_recessive(converter):
    bits = [1] * 16
    sig  = converter.to_differential(bits)
    back = converter.from_differential(sig)
    assert back == bits

def test_round_trip_full_can_frame(converter):
    """Encode a real CAN frame and verify round trip through differential"""
    encoder = CANEncoder()
    frame   = CANFrame(
        arbitration_id = 0x123,
        dlc            = 4,
        data           = bytes([0xDE, 0xAD, 0xBE, 0xEF])
    )
    bits = encoder.encode(frame)
    sig  = converter.to_differential(bits)
    back = converter.from_differential(sig)
    assert back == bits


# ── get_bit ───────────────────────────────────────────────────────────────────

def test_get_bit_out_of_range(converter):
    sig = converter.to_differential([0, 1])
    with pytest.raises(IndexError, match="out of range"):
        sig.get_bit(2)

def test_get_bit_last_valid_index(converter):
    sig = converter.to_differential([0, 1])
    canh, canl = sig.get_bit(1)
    assert canh == 0 and canl == 0   # recessive


# ── Invalid bus states ────────────────────────────────────────────────────────

def test_invalid_state_canh1_canl1_rejected(converter):
    """canh=1 canl=1 is physically impossible on CAN bus"""
    # Manually construct an invalid signal
    sig = DifferentialSignal(
        canh_bytes = bytes([0xFF]),
        canl_bytes = bytes([0xFF]),
        bit_count  = 8
    )
    with pytest.raises(ValueError, match="canh=1 canl=1"):
        converter.from_differential(sig)

def test_invalid_state_canh0_canl1_rejected(converter):
    """canh=0 canl=1 is physically impossible on CAN bus"""
    sig = DifferentialSignal(
        canh_bytes = bytes([0x00]),
        canl_bytes = bytes([0x80]),
        bit_count  = 8
    )
    with pytest.raises(ValueError, match="canh=0 canl=1"):
        converter.from_differential(sig)


# ── Bus wired-AND ─────────────────────────────────────────────────────────────

def test_bus_and_dominant_wins(converter):
    """
    When one node drives dominant and another recessive,
    the bus result is dominant (wired-AND).
    """
    # Node A: dominant (0)  → canh=1, canl=0
    # Node B: recessive (1) → canh=0, canl=0
    # Result: dominant      → canh=1, canl=0
    sig_a  = converter.to_differential([0])
    sig_b  = converter.to_differential([1])
    result = converter.bus_and(sig_a, sig_b)
    canh, canl = result.get_bit(0)
    assert canh == 1 and canl == 0   # dominant wins

def test_bus_and_both_recessive(converter):
    """Both nodes recessive → bus is recessive"""
    sig_a  = converter.to_differential([1])
    sig_b  = converter.to_differential([1])
    result = converter.bus_and(sig_a, sig_b)
    canh, canl = result.get_bit(0)
    assert canh == 0 and canl == 0   # recessive

def test_bus_and_both_dominant(converter):
    """Both nodes dominant → bus is dominant"""
    sig_a  = converter.to_differential([0])
    sig_b  = converter.to_differential([0])
    result = converter.bus_and(sig_a, sig_b)
    canh, canl = result.get_bit(0)
    assert canh == 1 and canl == 0   # dominant

def test_bus_and_length_mismatch_rejected(converter):
    sig_a = converter.to_differential([0, 1])
    sig_b = converter.to_differential([0])
    with pytest.raises(ValueError, match="different lengths"):
        converter.bus_and(sig_a, sig_b)


# ── Len and repr ──────────────────────────────────────────────────────────────

def test_len(converter):
    sig = converter.to_differential([0, 1, 0])
    assert len(sig) == 3

def test_repr(converter):
    sig = converter.to_differential([0, 1])
    r   = repr(sig)
    assert "bit_count=2" in r
    assert "canh=" in r
    assert "canl=" in r
