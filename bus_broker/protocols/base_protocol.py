# bus_broker/protocols/base_protocol.py

from abc import ABC, abstractmethod
from ..core.frames import CANFrame
from ..core.signal import DifferentialSignal


class BaseProtocol(ABC):
    """
    Abstract base for all bus protocols.

    To add a new protocol (e.g. FlexRay):
      1. Create bus_broker/protocols/flexray_protocol.py
      2. Subclass BaseProtocol
      3. Implement all abstract methods
      4. Register it in PROTOCOL_REGISTRY at the bottom of this file

    That is all. No other files need to change.
    """

    @property
    @abstractmethod
    def name(self) -> str:
        """Human readable protocol name e.g. 'CAN', 'CAN_FD'"""
        pass

    @property
    @abstractmethod
    def default_bit_rate(self) -> int:
        """Nominal bit rate in bits per second"""
        pass

    @property
    @abstractmethod
    def max_frame_bits(self) -> int:
        """
        Maximum number of bits a single frame can produce
        after encoding and bit stuffing.
        Used to size buffers correctly.
        """
        pass

    @abstractmethod
    def encode(self, frame: CANFrame) -> DifferentialSignal:
        """
        Full pipeline: CANFrame → DifferentialSignal.
        Implementations call the encoder and signal converter internally.
        """
        pass

    @abstractmethod
    def decode(self, signal: DifferentialSignal) -> CANFrame:
        """
        Reverse pipeline: DifferentialSignal → CANFrame.
        Used by the bus monitor and ECU receiver.
        """
        pass

    @abstractmethod
    def arbitration_priority(self, frame: CANFrame) -> int:
        """
        Return an integer priority for bus arbitration.
        Lower value = higher priority.
        CAN uses arbitration ID directly.
        FlexRay would use slot ID.
        """
        pass

    @abstractmethod
    def validate_frame(self, frame: CANFrame) -> tuple[bool, str]:
        """
        Check if a frame is valid for this protocol.
        Returns (True, "") or (False, "reason").
        """
        pass

    def __repr__(self) -> str:
        return f"{self.__class__.__name__}(rate={self.default_bit_rate})"
