# bus_broker/machine/time_compat.py

"""
MicroPython time module compatibility.
Provides the MicroPython-specific time functions that do not
exist in standard Python.
"""

import time as _time


def sleep_ms(ms: int):
    """Sleep for ms milliseconds. Matches MicroPython time.sleep_ms()"""
    _time.sleep(ms / 1000.0)


def sleep_us(us: int):
    """Sleep for us microseconds. Matches MicroPython time.sleep_us()"""
    _time.sleep(us / 1_000_000.0)


def sleep(s: float):
    """Sleep for s seconds."""
    _time.sleep(s)


def ticks_ms() -> int:
    """
    Returns milliseconds since an arbitrary point.
    Matches MicroPython time.ticks_ms().
    """
    return int(_time.monotonic() * 1000)


def ticks_us() -> int:
    """
    Returns microseconds since an arbitrary point.
    Matches MicroPython time.ticks_us().
    """
    return int(_time.monotonic() * 1_000_000)


def ticks_diff(new: int, old: int) -> int:
    """
    Difference between two ticks values.
    Matches MicroPython time.ticks_diff().
    """
    return new - old


def time() -> float:
    """Seconds since epoch."""
    return _time.time()


def localtime():
    """Returns a time tuple. Matches MicroPython time.localtime()"""
    return _time.localtime()