# bus_broker/core/signal.py

from dataclasses import dataclass


@dataclass
class DifferentialSignal:
    """
    Represents the physical state of the CAN bus differential pair.

    CAN bus uses two wires that are always complementary:

      Dominant  (logic 0): CANH = 3.5V, CANL = 1.5V  → difference =  2.0V
      Recessive (logic 1): CANH = 2.5V, CANL = 2.5V  → difference =  0.0V

    We represent this digitally as:
      Dominant  → canh=1, canl=0
      Recessive → canh=0, canl=0

    The bits are packed into bytes, MSB first.
    Index 0 of canh_bytes/canl_bytes corresponds to the first bit on the wire.

    bit_count tells us how many valid bits are in the packed arrays,
    since the last byte may not be fully used.
    """
    canh_bytes : bytes
    canl_bytes : bytes
    bit_count  : int

    def __post_init__(self):
        if len(self.canh_bytes) != len(self.canl_bytes):
            raise ValueError(
                f"CANH and CANL arrays must be the same length. "
                f"Got canh={len(self.canh_bytes)} canl={len(self.canl_bytes)}"
            )
        expected_bytes = (self.bit_count + 7) // 8
        if len(self.canh_bytes) != expected_bytes:
            raise ValueError(
                f"Array length {len(self.canh_bytes)} does not match "
                f"bit_count {self.bit_count} (expected {expected_bytes} bytes)"
            )

    def get_bit(self, index: int) -> tuple[int, int]:
        """
        Return (canh_bit, canl_bit) at a given bit index.
        Useful for replaying the signal one bit at a time.
        """
        if index >= self.bit_count:
            raise IndexError(
                f"Bit index {index} out of range (bit_count={self.bit_count})"
            )
        byte_idx = index // 8
        bit_idx  = 7 - (index % 8)   # MSB first
        canh_bit = (self.canh_bytes[byte_idx] >> bit_idx) & 1
        canl_bit = (self.canl_bytes[byte_idx] >> bit_idx) & 1
        return canh_bit, canl_bit

    def __len__(self) -> int:
        return self.bit_count

    def __repr__(self) -> str:
        return (
            f"DifferentialSignal("
            f"bit_count={self.bit_count}, "
            f"canh={self.canh_bytes.hex()}, "
            f"canl={self.canl_bytes.hex()}"
            f")"
        )


class SignalConverter:
    """
    Converts between bit lists and DifferentialSignal objects.

    This sits between the encoder and the shared memory transport:

      CANFrame
          ↓  CANEncoder
      list[int]  (bits)
          ↓  SignalConverter.to_differential
      DifferentialSignal
          ↓  SharedBusWriter
      /dev/shm ring buffer
    """

    def to_differential(self, bits: list[int]) -> DifferentialSignal:
        """
        Convert a bit list into a DifferentialSignal.

        For each bit:
          0 (dominant)  → canh=1, canl=0
          1 (recessive) → canh=0, canl=0
        """
        if not bits:
            raise ValueError("Cannot convert empty bit list to differential signal")

        bit_count     = len(bits)
        num_bytes     = (bit_count + 7) // 8
        canh_array    = bytearray(num_bytes)
        canl_array    = bytearray(num_bytes)

        for i, bit in enumerate(bits):
            byte_idx = i // 8
            bit_idx  = 7 - (i % 8)   # MSB first

            if bit == 0:   # dominant
                canh_array[byte_idx] |= (1 << bit_idx)
                # canl stays 0
            else:           # recessive
                pass        # both stay 0

        return DifferentialSignal(
            canh_bytes = bytes(canh_array),
            canl_bytes = bytes(canl_array),
            bit_count  = bit_count
        )

    def from_differential(self, signal: DifferentialSignal) -> list[int]:
        """
        Convert a DifferentialSignal back into a bit list.

        Rules:
          canh=1, canl=0 → dominant  → bit 0
          canh=0, canl=0 → recessive → bit 1
          canh=1, canl=1 → bus error → raises ValueError
          canh=0, canl=1 → bus error → raises ValueError
        """
        bits = []

        for i in range(signal.bit_count):
            canh_bit, canl_bit = signal.get_bit(i)

            if canh_bit == 1 and canl_bit == 0:
                bits.append(0)   # dominant
            elif canh_bit == 0 and canl_bit == 0:
                bits.append(1)   # recessive
            elif canh_bit == 1 and canl_bit == 1:
                raise ValueError(
                    f"Invalid bus state at bit {i}: "
                    f"canh=1 canl=1 is not a valid CAN state"
                )
            else:
                raise ValueError(
                    f"Invalid bus state at bit {i}: "
                    f"canh=0 canl=1 is not a valid CAN state"
                )

        return bits

    def bus_and(
        self,
        a: DifferentialSignal,
        b: DifferentialSignal
    ) -> DifferentialSignal:
        """
        Simulate the wired-AND nature of the CAN bus.

        When two nodes drive the bus simultaneously:
          - Any dominant bit (0) from either node makes the bus dominant
          - The bus is only recessive if ALL nodes are recessive

        Dominant  → canh=1, canl=0
        Recessive → canh=0, canl=0

        If either node is dominant, canh must be 1.
        That means we OR the canh arrays (any 1 wins).
        canl is always 0 in our encoding so OR works there too.
        """
        if a.bit_count != b.bit_count:
            raise ValueError(
                f"Cannot AND signals of different lengths: "
                f"{a.bit_count} vs {b.bit_count}"
            )

        canh = bytes(x | y for x, y in zip(a.canh_bytes, b.canh_bytes))
        canl = bytes(x | y for x, y in zip(a.canl_bytes, b.canl_bytes))

        return DifferentialSignal(
            canh_bytes = canh,
            canl_bytes = canl,
            bit_count  = a.bit_count
        )
