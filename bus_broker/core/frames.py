# bus_broker/core/frames.py

from dataclasses import dataclass, field
from enum import IntEnum

class Protocol(IntEnum):
    CAN    = 0
    CAN_FD = 1

# Valid DLC values for CAN FD (maps to actual byte counts)
CAN_FD_DLC_MAP = {
    0:0, 1:1, 2:2, 3:3, 4:4, 5:5, 6:6, 7:7, 8:8,
    9:12, 10:16, 11:20, 12:24, 13:32, 14:48, 15:64
}

# Max data bytes per protocol
MAX_DATA_BYTES = {
    Protocol.CAN:    8,
    Protocol.CAN_FD: 64,
}

# Max arbitration ID per frame type
MAX_ID_STANDARD = 0x7FF        # 11 bits
MAX_ID_EXTENDED = 0x1FFFFFFF   # 29 bits


@dataclass
class CANFrame:
    """
    Represents a single CAN or CAN FD frame.
    This is the structure that moves through the entire system.
    """

    arbitration_id : int
    dlc            : int
    data           : bytes
    protocol       : Protocol = Protocol.CAN
    is_extended    : bool     = False
    is_remote      : bool     = False

    # Set automatically by the bus controller, not by the user
    source_ecu_id  : str      = ""
    timestamp_ns   : int      = 0

    def __post_init__(self):
        self._validate()

    def _validate(self):
        # Protocol must be known
        if self.protocol not in Protocol.__members__.values():
            raise ValueError(f"Unknown protocol: {self.protocol}")

        # ID range check
        max_id = MAX_ID_EXTENDED if self.is_extended else MAX_ID_STANDARD
        if not (0 <= self.arbitration_id <= max_id):
            raise ValueError(
                f"Arbitration ID 0x{self.arbitration_id:X} out of range "
                f"for {'extended' if self.is_extended else 'standard'} frame "
                f"(max 0x{max_id:X})"
            )

        # DLC range check
        max_dlc = 15 if self.protocol == Protocol.CAN_FD else 8
        if not (0 <= self.dlc <= max_dlc):
            raise ValueError(
                f"DLC {self.dlc} out of range for {self.protocol.name} "
                f"(max {max_dlc})"
            )

        # Data length must match DLC
        # Remote frames carry no data regardless of DLC
        # (DLC on a remote frame = how many bytes the responder should send)
        if not self.is_remote:
            if self.protocol == Protocol.CAN_FD:
                expected_bytes = CAN_FD_DLC_MAP[self.dlc]
            else:
                expected_bytes = self.dlc

            if len(self.data) != expected_bytes:
                raise ValueError(
                    f"Data length {len(self.data)} does not match "
                    f"DLC {self.dlc} (expected {expected_bytes} bytes)"
                )

        # Remote frames cannot carry data
        if self.is_remote and len(self.data) > 0:
            raise ValueError("Remote frames cannot carry data")

        # CAN FD does not have remote frames
        if self.protocol == Protocol.CAN_FD and self.is_remote:
            raise ValueError("CAN FD does not support remote frames")

    @property
    def data_length(self) -> int:
        """Actual number of data bytes (not DLC code for CAN FD)"""
        if self.protocol == Protocol.CAN_FD:
            return CAN_FD_DLC_MAP[self.dlc]
        return self.dlc

    def __repr__(self) -> str:
        data_hex = self.data.hex().upper()
        id_str   = f"0x{self.arbitration_id:08X}" if self.is_extended \
                   else f"0x{self.arbitration_id:03X}"
        return (
            f"CANFrame("
            f"protocol={self.protocol.name}, "
            f"id={id_str}, "
            f"dlc={self.dlc}, "
            f"data={data_hex}"
            f"{'  [EXT]' if self.is_extended else ''}"
            f"{'  [RTR]' if self.is_remote  else ''}"
            f")"
        )
