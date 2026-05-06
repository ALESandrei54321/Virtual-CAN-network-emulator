# bus_broker/machine/adc.py

"""
Fake machine.ADC class.
Returns configurable values for simulation.
Matches the MicroPython machine.ADC API.
"""


class ADC:
    """
    Simulated ADC channel.

    Usage (identical to real MicroPython):
        adc   = ADC(26)          # GP26 = ADC0
        value = adc.read_u16()   # 0-65535
    """

    # Class-level simulated values
    # ECU test code can set these to simulate sensor readings:
    # ADC._sim_values[26] = 32768
    _sim_values: dict[int, int] = {}

    # RP2040 ADC pin mapping
    _PIN_TO_CHANNEL = {
        26: 0,
        27: 1,
        28: 2,
        29: 3,
    }

    def __init__(self, pin: int):
        self._pin     = pin
        self._channel = self._PIN_TO_CHANNEL.get(pin, 0)
        print(f"[ADC] GP{pin} channel={self._channel}")

    def read_u16(self) -> int:
        """
        Read ADC value as 16-bit unsigned integer (0-65535).
        Returns simulated value if set, otherwise midpoint.
        """
        return self._sim_values.get(self._pin, 32768)

    def read_uv(self) -> int:
        """
        Read ADC value in microvolts.
        3.3V reference → max 3_300_000 uV
        """
        raw = self.read_u16()
        return int(raw / 65535 * 3_300_000)

    @classmethod
    def set_sim_value(cls, pin: int, value: int):
        """
        Set a simulated ADC reading for testing.
        value should be 0-65535.
        """
        cls._sim_values[pin] = max(0, min(65535, value))

    def __repr__(self) -> str:
        return f"ADC(GP{self._pin}, channel={self._channel})"