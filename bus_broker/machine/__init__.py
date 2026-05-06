# bus_broker/machine/__init__.py

"""
Fake MicroPython machine module.

When an ECU file does:
    from machine import CAN, Pin, Timer

It gets our simulated versions that talk to the shared memory bus.
The ECU code is identical to what would run on a real RP2040.
"""

from .can   import CAN
from .pin   import Pin
from .timer import Timer
from .adc   import ADC

__all__ = ["CAN", "Pin", "Timer", "ADC"]