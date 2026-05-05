# bus_broker/protocols/can_protocol.py

from .base_protocol import BaseProtocol
from ..core.frames import CANFrame, Protocol, MAX_ID_STANDARD, MAX_ID_EXTENDED
from ..core.encoder import CANEncoder
from ..core.signal import DifferentialSignal, SignalConverter


class CANProtocol(BaseProtocol):
    """
    CAN 2.0A (standard 11-bit ID) and 2.0B (extended 29-bit ID).
    Bit rate configurable, default 500 kbit/s.
    """

    def __init__(self, bit_rate: int = 500_000):
        self._bit_rate = bit_rate
        self._encoder  = CANEncoder()
        self._conv     = SignalConverter()

    @property
    def name(self) -> str:
        return "CAN"

    @property
    def default_bit_rate(self) -> int:
        return self._bit_rate

    @property
    def max_frame_bits(self) -> int:
        # Standard CAN worst case with bit stuffing:
        # 1(SOF) + 11(ID) + 1(RTR) + 1(IDE) + 1(r0) + 4(DLC)
        # + 64(data) + 15(CRC) + 1(CRCDEL) + 2(ACK) + 7(EOF) + 3(IFS)
        # = 111 bits + ~20% stuffing overhead ≈ 135 bits
        return 150

    def encode(self, frame: CANFrame) -> DifferentialSignal:
        bits   = self._encoder.encode(frame)
        return self._conv.to_differential(bits)

    def decode(self, signal: DifferentialSignal) -> CANFrame:
        bits = self._conv.from_differential(signal)
        return self._decode_bits(bits)

    def arbitration_priority(self, frame: CANFrame) -> int:
        # Lower ID = higher priority on CAN bus
        return frame.arbitration_id

    def validate_frame(self, frame: CANFrame) -> tuple[bool, str]:
        if frame.protocol != Protocol.CAN:
            return False, f"Expected Protocol.CAN got {frame.protocol.name}"

        max_id = MAX_ID_EXTENDED if frame.is_extended else MAX_ID_STANDARD
        if frame.arbitration_id > max_id:
            return False, (
                f"ID 0x{frame.arbitration_id:X} exceeds max "
                f"0x{max_id:X} for "
                f"{'extended' if frame.is_extended else 'standard'} frame"
            )

        if frame.dlc > 8:
            return False, f"DLC {frame.dlc} exceeds CAN maximum of 8"

        return True, ""

    def _decode_bits(self, bits: list[int]) -> CANFrame:
        """
        Parse a raw bit stream back into a CANFrame.
        Removes stuff bits, then reads each field.
        """
        # Remove stuff bits first
        unstuffed = self._remove_bit_stuffing(bits)

        idx = 0

        # SOF
        # sof = unstuffed[idx]
        idx += 1

        # Peek at IDE to determine frame type
        # For standard: IDE is at bit 13 (after SOF+11ID+RTR)
        # For extended: IDE is at bit 13 (after SOF+11BaseID+SRR)
        ide = unstuffed[13]
        is_extended = (ide == 1)

        if is_extended:
            base_id  = self._bits_to_int(unstuffed[1:12])
            # skip SRR(1) IDE(1)
            ext_id   = self._bits_to_int(unstuffed[14:32])
            arb_id   = (base_id << 18) | ext_id
            rtr      = unstuffed[32]
            idx      = 35   # after SOF+11+SRR+IDE+18+RTR+r1+r0
            dlc      = self._bits_to_int(unstuffed[idx:idx+4])
            idx     += 4
        else:
            arb_id   = self._bits_to_int(unstuffed[1:12])
            rtr      = unstuffed[12]
            idx      = 15   # after SOF+11+RTR+IDE+r0
            dlc      = self._bits_to_int(unstuffed[idx:idx+4])
            idx     += 4

        # Data field
        data = bytearray()
        if not rtr:
            for _ in range(dlc):
                byte  = self._bits_to_int(unstuffed[idx:idx+8])
                data.append(byte)
                idx  += 8

        return CANFrame(
            arbitration_id = arb_id,
            dlc            = dlc,
            data           = bytes(data),
            protocol       = Protocol.CAN,
            is_extended    = is_extended,
            is_remote      = bool(rtr),
        )

    def _remove_bit_stuffing(self, bits: list[int]) -> list[int]:
        """Remove stuff bits from a received bit stream."""
        result      = []
        consecutive = 1
        last_bit    = bits[0]
        result.append(bits[0])

        i = 1
        while i < len(bits):
            bit = bits[i]

            if bit == last_bit:
                consecutive += 1
                if consecutive == 5:
                    # Next bit is a stuff bit - skip it
                    result.append(bit)
                    i += 1    # skip stuff bit
                    if i < len(bits):
                        consecutive = 1
                        last_bit    = bits[i] if i < len(bits) else bit
                        i          += 1
                    continue
            else:
                consecutive = 1

            result.append(bit)
            last_bit = bit
            i       += 1

        return result

    def _bits_to_int(self, bits: list[int]) -> int:
        result = 0
        for b in bits:
            result = (result << 1) | b
        return result
