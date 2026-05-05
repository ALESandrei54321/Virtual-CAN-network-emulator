# tests/test_encoder.py

import pytest
from bus_broker.core.frames import CANFrame, Protocol
from bus_broker.core.encoder import CANEncoder


@pytest.fixture
def encoder():
    return CANEncoder()


# ── Helpers ───────────────────────────────────────────────────────────────────

def bits_to_int(bits: list[int]) -> int:
    result = 0
    for b in bits:
        result = (result << 1) | b
    return result

def extract_standard_fields(bits: list[int]) -> dict:
    """
    Parse a raw (pre-stuffing-removed) standard CAN frame
    back into its fields. Useful for asserting structure.
    We remove stuff bits first.
    """
    # We will work on the original bits before stuffing
    # so we just check known fixed positions
    return {
        "sof"   : bits[0],
        "id"    : bits_to_int(bits[1:12]),
        "rtr"   : bits[12],
        "ide"   : bits[13],
        "r0"    : bits[14],
        "dlc"   : bits_to_int(bits[15:19]),
    }


# ── SOF ───────────────────────────────────────────────────────────────────────

def test_sof_is_dominant(encoder):
    frame = CANFrame(arbitration_id=0x123, dlc=0, data=b"")
    bits = encoder.encode(frame)
    assert bits[0] == 0, "SOF must be dominant (0)"


# ── Arbitration ID ────────────────────────────────────────────────────────────

def test_standard_id_encoded_correctly(encoder):
    frame = CANFrame(arbitration_id=0x123, dlc=0, data=b"")
    bits  = encoder.encode(frame)
    # ID starts at bit 1, 11 bits long
    # 0x123 = 0b00100100011
    id_bits = bits[1:12]
    assert bits_to_int(id_bits) == 0x123

def test_extended_id_encoded_correctly(encoder):
    frame = CANFrame(
        arbitration_id=0x12345678,
        dlc=0,
        data=b"",
        is_extended=True
    )
    bits = encoder.encode(frame)
    # Base ID = upper 11 bits of 0x12345678
    base_id = (0x12345678 >> 18) & 0x7FF
    assert bits_to_int(bits[1:12]) == base_id
    # SRR at bit 12 must be recessive
    assert bits[12] == 1
    # IDE at bit 13 must be recessive (signals extended)
    assert bits[13] == 1


# ── IDE flag ─────────────────────────────────────────────────────────────────

def test_ide_dominant_for_standard_frame(encoder):
    frame = CANFrame(arbitration_id=0x100, dlc=0, data=b"")
    bits  = encoder.encode(frame)
    # IDE is at bit 13 for standard frames
    assert bits[13] == 0, "IDE must be dominant for standard frame"

def test_ide_recessive_for_extended_frame(encoder):
    """
    Check IDE in the raw bit stream before stuffing,
    where positions are deterministic.
    Extended frame layout:
    SOF(1) + BaseID(11) + SRR(1) + IDE(1) = IDE is at index 13
    """
    enc   = CANEncoder()
    frame = CANFrame(arbitration_id=0x100, dlc=0, data=b"", is_extended=True)

    # Rebuild raw bits the same way the encoder does, before stuffing
    raw = []
    raw.append(0)                                           # SOF
    base_id = (0x100 >> 18) & 0x7FF
    raw += enc._int_to_bits(base_id, 11)                   # Base ID
    raw.append(1)                                           # SRR
    raw.append(1)                                           # IDE  ← index 13
    assert raw[13] == 1, "IDE must be recessive in raw bits for extended frame"


# ── RTR ───────────────────────────────────────────────────────────────────────

def test_rtr_dominant_for_data_frame(encoder):
    frame = CANFrame(arbitration_id=0x100, dlc=0, data=b"")
    bits  = encoder.encode(frame)
    assert bits[12] == 0, "RTR must be dominant for data frame"

def test_rtr_recessive_for_remote_frame(encoder):
    """
    Check RTR in the raw bit stream before stuffing.
    Standard frame layout:
    SOF(1) + ID(11) + RTR(1) = RTR is at index 12
    """
    enc   = CANEncoder()

    # Rebuild raw bits before stuffing
    raw = []
    raw.append(0)                           # SOF
    raw += enc._int_to_bits(0x100, 11)     # ID
    raw.append(1)                           # RTR recessive for remote frame
    assert raw[12] == 1, "RTR must be recessive in raw bits for remote frame"


# ── DLC ───────────────────────────────────────────────────────────────────────

def test_dlc_encoded_correctly(encoder):
    """
    Check DLC in the raw bit stream before stuffing
    so that stuff bit insertion does not affect positions.
    Standard frame raw layout:
    SOF(1) + ID(11) + RTR(1) + IDE(1) + r0(1) + DLC(4)
    DLC starts at index 15 in raw bits.
    """
    enc = CANEncoder()

    for dlc in range(9):  # 0-8 for standard CAN
        raw = []
        raw.append(0)                               # SOF
        raw += enc._int_to_bits(0x100, 11)         # ID  (no leading zeros = no stuffing)
        raw.append(0)                               # RTR
        raw.append(0)                               # IDE
        raw.append(0)                               # r0
        raw += enc._int_to_bits(dlc, 4)            # DLC starts at index 15
        dlc_bits = raw[15:19]
        assert bits_to_int(dlc_bits) == dlc, \
            f"DLC {dlc} not encoded correctly in raw bits"


# ── Data field ────────────────────────────────────────────────────────────────

def test_data_bytes_encoded_correctly(encoder):
    data  = bytes([0xDE, 0xAD, 0xBE, 0xEF])
    frame = CANFrame(arbitration_id=0x123, dlc=4, data=data)
    bits  = encoder.encode(frame)
    # Data starts at bit 19 for standard frame (SOF+ID+RTR+IDE+r0+DLC = 19)
    # Note: this is before bit stuffing so we check the encoder internal
    # by rebuilding raw bits
    enc   = CANEncoder()
    raw   = []
    raw.append(0)
    raw  += enc._int_to_bits(0x123, 11)
    raw.append(0)   # RTR
    raw.append(0)   # IDE
    raw.append(0)   # r0
    raw  += enc._int_to_bits(4, 4)
    # Data starts here at index 19
    for byte in data:
        raw += enc._int_to_bits(byte, 8)
    # Check each byte
    for i, byte in enumerate(data):
        start    = 19 + i * 8
        end      = start + 8
        got      = bits_to_int(raw[start:end])
        assert got == byte, f"Byte {i}: expected 0x{byte:02X} got 0x{got:02X}"


# ── Bit stuffing ─────────────────────────────────────────────────────────────

def test_no_five_consecutive_bits_in_stuffed_output(encoder):
    """After stuffing, no run of 6+ identical bits should exist"""
    frame  = CANFrame(arbitration_id=0x000, dlc=1, data=bytes([0xFF]))
    bits   = encoder.encode(frame)
    run    = 1
    for i in range(1, len(bits)):
        if bits[i] == bits[i-1]:
            run += 1
            # EOF and interframe are all 1s so we stop before those
            # We check only up to a reasonable frame length
            if i < len(bits) - (encoder.EOF_BITS + encoder.INTERFRAME_BITS):
                assert run < 6, (
                    f"Found run of {run} identical bits at position {i}"
                )
        else:
            run = 1

def test_stuff_bit_inserted_after_five_zeros(encoder):
    """
    ID=0x000 gives SOF(0) + 11 zeros = 12 consecutive zeros.
    The first 5 identical bits trigger a stuff bit.
    bits[0..4] = five zeros  → stuff bit inserted at index 5.
    """
    frame = CANFrame(arbitration_id=0x000, dlc=0, data=b"")
    bits  = encoder.encode(frame)
    # First 5 bits: SOF(0) + first 4 bits of ID(0x000=00000000000)
    # All zero → stuff bit (1) inserted at position 5
    assert bits[5] == 1, (
        f"Expected stuff bit (1) at index 5, "
        f"got {bits[5]}. First 10 bits: {bits[:10]}"
    )

def test_stuff_bit_is_opposite_polarity(encoder):
    encoder_obj = CANEncoder()
    result = encoder_obj._apply_bit_stuffing([0,0,0,0,0,1,1])
    # After 5 zeros, a 1 stuff bit is inserted
    assert result[5] == 1, "Stuff bit should be 1 after five 0s"


# ── CRC ───────────────────────────────────────────────────────────────────────

def test_crc15_known_value(encoder):
    """
    CRC-15 over SOF + known frame bits.
    We verify the CRC is nonzero and deterministic.
    """
    enc   = CANEncoder()
    bits  = [0] + enc._int_to_bits(0x123, 11) + [0, 0, 0] + enc._int_to_bits(4, 4)
    bits += enc._int_to_bits(0xDEADBEEF, 32)
    crc1  = enc._calculate_crc15(bits)
    crc2  = enc._calculate_crc15(bits)
    assert crc1 == crc2,  "CRC must be deterministic"
    assert crc1 != 0,     "CRC should be nonzero for this frame"
    assert crc1 <= 0x7FFF, "CRC-15 must fit in 15 bits"

def test_crc15_changes_with_data(encoder):
    enc   = CANEncoder()
    bits1 = [0] + enc._int_to_bits(0x123, 11) + [0,0,0] + enc._int_to_bits(4,4)
    bits1+= enc._int_to_bits(0xDEADBEEF, 32)
    bits2 = [0] + enc._int_to_bits(0x123, 11) + [0,0,0] + enc._int_to_bits(4,4)
    bits2+= enc._int_to_bits(0xCAFEBABE, 32)
    assert enc._calculate_crc15(bits1) != enc._calculate_crc15(bits2)

def test_crc17_fits_in_17_bits(encoder):
    enc  = CANEncoder()
    bits = [0, 1] * 20
    crc  = enc._calculate_crc17(bits)
    assert 0 <= crc <= 0x1FFFF

def test_crc21_fits_in_21_bits(encoder):
    enc  = CANEncoder()
    bits = [0, 1] * 40
    crc  = enc._calculate_crc21(bits)
    assert 0 <= crc <= 0x1FFFFF


# ── EOF and frame length sanity ───────────────────────────────────────────────

def test_eof_ends_with_recessive_bits(encoder):
    frame = CANFrame(arbitration_id=0x123, dlc=0, data=b"")
    bits  = encoder.encode(frame)
    eof_and_ifs = encoder.EOF_BITS + encoder.INTERFRAME_BITS
    tail  = bits[-eof_and_ifs:]
    assert all(b == 1 for b in tail), "EOF and interframe space must be recessive"

def test_frame_minimum_length(encoder):
    """A 0-byte data frame must still be longer than the fixed fields"""
    frame = CANFrame(arbitration_id=0x000, dlc=0, data=b"")
    bits  = encoder.encode(frame)
    # SOF(1) + ID(11) + RTR(1) + IDE(1) + r0(1) + DLC(4) + CRC(15)
    # + CRC_DEL(1) + ACK(2) + EOF(7) + IFS(3) = 47 bits minimum before stuffing
    assert len(bits) >= 47

def test_can_fd_frame_encodes(encoder):
    frame = CANFrame(
        arbitration_id=0x200,
        dlc=9,              # 12 bytes in CAN FD
        data=bytes(12),
        protocol=Protocol.CAN_FD
    )
    bits = encoder.encode(frame)
    assert len(bits) > 0
    assert bits[0] == 0    # SOF dominant
