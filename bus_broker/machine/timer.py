# bus_broker/machine/timer.py

"""
Fake machine.Timer class.
Uses Python threading to call callbacks at regular intervals.
Matches the MicroPython machine.Timer API.
"""

import threading
import time
from typing import Callable


class Timer:
    """
    Simulated hardware timer.

    Usage (identical to real MicroPython):
        t = Timer(-1)
        t.init(period=100, mode=Timer.PERIODIC, callback=my_func)
        # my_func will be called every 100ms
        t.deinit()
    """

    PERIODIC = 1
    ONE_SHOT = 0

    # Track all active timers so they can be stopped cleanly
    _active: list["Timer"] = []

    def __init__(self, id: int):
        self._id       = id
        self._thread   = None
        self._running  = False
        self._period   = 0.0
        self._mode     = self.PERIODIC
        self._callback = None

    def init(
        self,
        period   : int,
        mode     : int = PERIODIC,
        callback : Callable | None = None
    ):
        """
        Initialise and start the timer.
        period   : interval in milliseconds
        mode     : Timer.PERIODIC or Timer.ONE_SHOT
        callback : function to call, receives the timer as argument
        """
        # Stop any existing timer
        self.deinit()

        self._period   = period / 1000.0
        self._mode     = mode
        self._callback = callback
        self._running  = True

        self._thread = threading.Thread(
            target = self._run,
            daemon = True,     # dies when main process exits
            name   = f"Timer-{self._id}"
        )
        self._thread.start()
        Timer._active.append(self)

        print(
            f"[Timer] id={self._id} "
            f"period={period}ms "
            f"mode={'PERIODIC' if mode == self.PERIODIC else 'ONE_SHOT'}"
        )

    def _run(self):
        while self._running:
            time.sleep(self._period)
            if not self._running:
                break
            if self._callback:
                try:
                    self._callback(self)
                except Exception as e:
                    print(f"[Timer] Callback error: {e}")
            if self._mode == self.ONE_SHOT:
                self._running = False
                break

    def deinit(self):
        """Stop the timer."""
        self._running = False
        if self in Timer._active:
            Timer._active.remove(self)

    def __repr__(self) -> str:
        return (
            f"Timer(id={self._id}, "
            f"period={self._period*1000:.0f}ms, "
            f"running={self._running})"
        )

    @classmethod
    def stop_all(cls):
        """Stop all active timers. Called on ECU shutdown."""
        for t in list(cls._active):
            t.deinit()
        cls._active.clear()