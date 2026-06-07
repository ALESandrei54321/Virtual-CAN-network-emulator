# bus_broker/protocols/can_fd_protocol.py

from .base_protocol import BaseProtocol
from ..core.frames import CANFrame, Protocol, CAN_FD_DLC_MAP, MAX_ID_STANDARD, MAX_ID_EXTENDED
from ..core.encoder import CANEncoder
from ..core.signal import DifferentialSignal, SignalConverter


class CANFDProtocol(BaseProtocol):
    """
    CAN FD (Flexible Data-rate) per ISO 11898-1:2015.
    Supports up to 64 bytes of data and faster data phase bit rate.

    Key differences from classic CAN:
      - FDF bit (recessive) marks a frame as CAN FD
      - BRS bit enables bit rate switching (data phase at faster clock)
      - ESI bit indicates error state of the transmitter
      - DLC 9-15 map to 12, 16, 20, 24, 32, 48, 64 data bytes
      - CRC-17 for <=16 data bytes, CRC-21 for >16 data bytes
      - No remote frames

    Frame structure (standard 11-bit ID):
      SOF(1) | ID(11) | RTR(1) | IDE(1) | r0(1) | FDF(1) | res(1) |
      BRS(1) | ESI(1) | DLC(4) | Data(0-512) | CRC(17/21) |
      CRC_DEL(1) | ACK(2) | EOF(7) | IFS(3)
    """

    # Reverse DLC map: byte count → DLC code
    _BYTES_TO_DLC = {v: k for k, v in CAN_FD_DLC_MAP.items()}

    def __init__(
        self,
        nominal_bit_rate: int = 500_000,
        data_bit_rate:    int = 2_000_000
    ):
        self._nominal_rate = nominal_bit_rate
        self._data_rate    = data_bit_rate
        self._encoder      = CANEncoder()
        self._conv         = SignalConverter()

    @property
    def name(self) -> str:
        return "CAN_FD"

    @property
    def default_bit_rate(self) -> int:
        return self._nominal_rate

    @property
    def data_bit_rate(self) -> int:
        return self._data_rate

    @property
    def max_frame_bits(self) -> int:
        # CAN FD worst case: 64 data bytes + overhead + stuffing
        # 1+11+1+1+1+1+1+1+1+4+512+21+1+2+7+3 ≈ 569 + ~20% = ~700
        return 750

    def encode(self, frame: CANFrame) -> DifferentialSignal:
        bits   = self._encoder.encode(frame)
        return self._conv.to_differential(bits)

    def encode_with_brs(self, frame: CANFrame) -> tuple[DifferentialSignal, int]:
        """
        Encode a CAN FD frame and return (signal, brs_index).
        brs_index is the bit where the data-rate phase begins.
        """
        bits, brs_index = self._encoder.encode_with_metadata(frame)
        signal = self._conv.to_differential(bits)
        return signal, brs_index

    def decode(self, signal: DifferentialSignal) -> CANFrame:
        bits = self._conv.from_differential(signal)
        return self._decode_fd_bits(bits)

    def arbitration_priority(self, frame: CANFrame) -> int:
        return frame.arbitration_id

    def validate_frame(self, frame: CANFrame) -> tuple[bool, str]:
        if frame.protocol != Protocol.CAN_FD:
            return False, f"Expected Protocol.CAN_FD got {frame.protocol.name}"

        max_id = MAX_ID_EXTENDED if frame.is_extended else MAX_ID_STANDARD
        if frame.arbitration_id > max_id:
            return False, (
                f"ID 0x{frame.arbitration_id:X} exceeds max "
                f"0x{max_id:X}"
            )

        if frame.is_remote:
            return False, "CAN FD does not support remote frames"

        if frame.dlc > 15:
            return False, f"DLC {frame.dlc} exceeds CAN FD maximum of 15"

        return True, ""

    # ── CAN FD decode ─────────────────────────────────────────────────────────

    def _decode_fd_bits(self, bits: list[int]) -> CANFrame:
        """
        Parse a raw bit stream (after de-stuffing) into a CANFrame.

        CAN FD standard frame layout (pre-stuff):
          [0]     SOF
          [1:12]  ID (11 bits)
          [12]    RTR (always 0 for FD)
          [13]    IDE
          [14]    r0
          [15]    FDF  (1 = CAN FD)
          [16]    res
          [17]    BRS
          [18]    ESI
          [19:23] DLC (4 bits)
          [23:..] Data
          ...     CRC, CRC_DEL, ACK, EOF
        """
        unstuffed = self._remove_bit_stuffing(bits)

        idx = 0

        # SOF
        idx += 1

        # Check IDE for extended frame detection
        # Standard: IDE is at bit 13
        ide = unstuffed[13]
        is_extended = (ide == 1)

        if is_extended:
            base_id  = self._bits_to_int(unstuffed[1:12])
            # skip SRR(1) IDE(1)
            ext_id   = self._bits_to_int(unstuffed[14:32])
            arb_id   = (base_id << 18) | ext_id
            # RTR(1) at 32
            idx = 33
            # r0(1), FDF(1), res(1), BRS(1), ESI(1)
            # r0  = unstuffed[idx]
            idx += 1
            fdf = unstuffed[idx]; idx += 1
            # res = unstuffed[idx]
            idx += 1
            brs = unstuffed[idx]; idx += 1
            # esi = unstuffed[idx]
            idx += 1
        else:
            arb_id   = self._bits_to_int(unstuffed[1:12])
            # RTR = unstuffed[12], IDE = unstuffed[13]
            idx = 14
            # r0(1), FDF(1), res(1), BRS(1), ESI(1)
            # r0  = unstuffed[idx]
            idx += 1
            fdf = unstuffed[idx]; idx += 1
            # res = unstuffed[idx]
            idx += 1
            brs = unstuffed[idx]; idx += 1
            # esi = unstuffed[idx]
            idx += 1

        # DLC (4 bits)
        dlc = self._bits_to_int(unstuffed[idx:idx+4])
        idx += 4

        # Data field — use DLC map for FD
        if dlc <= 8:
            data_len = dlc
        else:
            data_len = CAN_FD_DLC_MAP.get(dlc, 0)

        data = bytearray()
        for _ in range(data_len):
            byte = self._bits_to_int(unstuffed[idx:idx+8])
            data.append(byte)
            idx += 8

        return CANFrame(
            arbitration_id = arb_id,
            dlc            = dlc,
            data           = bytes(data),
            protocol       = Protocol.CAN_FD,
            is_extended    = is_extended,
            is_remote      = False,
            brs            = bool(brs),
        )

    # ── Bit stuffing removal ──────────────────────────────────────────────────

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
