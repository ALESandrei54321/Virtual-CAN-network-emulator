# bus_broker/machine/pin.py

"""
Fake machine.Pin class.
Matches the MicroPython machine.Pin API.
Pin state changes are printed to stdout so ECU developers
can see what the firmware is doing to GPIO.
"""


class Pin:
    """
    Simulated GPIO pin.

    Usage (identical to real MicroPython):
        led = Pin(25, Pin.OUT)
        led.on()
        led.off()
        led.toggle()
        led.value(1)
        state = led.value()

        button = Pin(14, Pin.IN, Pin.PULL_UP)
        pressed = not button.value()
    """

    OUT      = 1
    IN       = 0
    PULL_UP  = 1
    PULL_DOWN= 2
    OPEN_DRAIN = 3

    # Class-level state shared across all Pin instances
    # so one ECU module can read what another set
    _states: dict[int, int] = {}

    def __init__(
        self,
        id    : int,
        mode  : int = IN,
        pull  : int | None = None,
        value : int = 0
    ):
        self._id   = id
        self._mode = mode
        self._pull = pull

        # Initialise state
        if id not in Pin._states:
            Pin._states[id] = value

        print(
            f"[Pin] GP{id:02d} "
            f"{'OUTPUT' if mode == self.OUT else 'INPUT '}"
            f"{' PULL_UP' if pull == self.PULL_UP else ''}"
            f"{' PULL_DOWN' if pull == self.PULL_DOWN else ''}"
        )

    def on(self):
        """Drive pin HIGH."""
        Pin._states[self._id] = 1
        print(f"[Pin] GP{self._id:02d} → HIGH")

    def off(self):
        """Drive pin LOW."""
        Pin._states[self._id] = 0
        print(f"[Pin] GP{self._id:02d} → LOW")

    def toggle(self):
        """Toggle pin state."""
        Pin._states[self._id] ^= 1
        state = Pin._states[self._id]
        print(f"[Pin] GP{self._id:02d} → {'HIGH' if state else 'LOW'}")

    def value(self, v: int | None = None) -> int | None:
        """
        Get or set pin value.
        value()  → returns current state
        value(1) → sets HIGH
        value(0) → sets LOW
        """
        if v is None:
            return Pin._states.get(self._id, 0)
        Pin._states[self._id] = int(bool(v))
        return None

    def __repr__(self) -> str:
        state = Pin._states.get(self._id, 0)
        return (
            f"Pin(GP{self._id}, "
            f"{'OUT' if self._mode == self.OUT else 'IN'}, "
            f"value={state})"
        )