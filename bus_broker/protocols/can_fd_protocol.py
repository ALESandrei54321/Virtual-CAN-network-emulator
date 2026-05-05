# bus_broker/protocols/can_fd_protocol.py

from .base_protocol import BaseProtocol
from .can_protocol import CANProtocol
from ..core.frames import CANFrame, Protocol, MAX_ID_STANDARD, MAX_ID_EXTENDED
from ..core.encoder import CANEncoder
from ..core.signal import DifferentialSignal, SignalConverter


class CANFDProtocol(BaseProtocol):
    """
    CAN FD (Flexible Data-rate) per ISO 11898-1:2015.
    Supports up to 64 bytes of data and faster data phase bit rate.
    """

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
        # 1+11+1+1+1+1+1+1+4+512+21+1+2+7+3 ≈ 568 + ~20% = ~700
        return 750

    def encode(self, frame: CANFrame) -> DifferentialSignal:
        bits = self._encoder.encode(frame)
        return self._conv.to_differential(bits)

    def decode(self, signal: DifferentialSignal) -> CANFrame:
        # Reuse CAN protocol decoder - FD shares the same bit layout
        # for the fields we care about at this stage
        can = CANProtocol(self._nominal_rate)
        bits = self._conv.from_differential(signal)
        return can._decode_bits(bits)

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
