# tools/run_ecu.py

"""
ECU Runner
==========
Launches an ECU firmware file with the fake machine module injected.
The ECU code runs exactly as it would on a real RP2040 with MicroPython.

Usage:
    python tools/run_ecu.py firmware/engine_ecu/main.py
    python tools/run_ecu.py firmware/engine_ecu/main.py --id engine_ecu
    python tools/run_ecu.py firmware/engine_ecu/main.py --verbose
"""

import sys
import signal
import argparse
import runpy
import threading
import time
from pathlib import Path

# Make sure bus_broker is importable
sys.path.insert(0, str(Path(__file__).parent.parent))


# ── Machine module injection ──────────────────────────────────────────────────

def inject_machine_module(ecu_id: str, verbose: bool):
    """
    Inject our fake machine module into sys.modules so that
    when the ECU code does 'from machine import CAN' it gets
    our simulated version.

    Also inject our time_compat as 'utime' which some MicroPython
    code uses as an alias for time.
    """
    import bus_broker.machine as machine_module
    import bus_broker.machine.time_compat as time_compat

    # Patch Pin print statements based on verbosity
    if not verbose:
        _silence_peripheral_prints(machine_module)

    # Inject as 'machine' - this is what ECU code imports
    sys.modules["machine"] = machine_module

    # Inject as 'utime' - MicroPython alias
    sys.modules["utime"]   = time_compat

    # Patch the standard 'time' module with MicroPython extras
    # while keeping standard time functions working
    _patch_time_module(time_compat)

    print(f"[Runner:{ecu_id}] Machine module injected.")


def _patch_time_module(time_compat):
    """
    Add MicroPython-specific functions to the standard time module.
    This lets ECU code use time.sleep_ms() without any import changes.
    """
    import time as stdlib_time
    stdlib_time.sleep_ms  = time_compat.sleep_ms
    stdlib_time.sleep_us  = time_compat.sleep_us
    stdlib_time.ticks_ms  = time_compat.ticks_ms
    stdlib_time.ticks_us  = time_compat.ticks_us
    stdlib_time.ticks_diff= time_compat.ticks_diff


def _silence_peripheral_prints(machine_module):
    """
    In non-verbose mode, suppress the [Pin], [Timer], [ADC]
    setup messages but keep CAN traffic messages.
    """
    import bus_broker.machine.pin   as pin_mod
    import bus_broker.machine.timer as timer_mod
    import bus_broker.machine.adc   as adc_mod

    _original_pin_init   = pin_mod.Pin.__init__
    _original_timer_init = timer_mod.Timer.init
    _original_adc_init   = adc_mod.ADC.__init__

    def quiet_pin_init(self, id, mode=pin_mod.Pin.IN, pull=None, value=0):
        self._id   = id
        self._mode = mode
        self._pull = pull
        if id not in pin_mod.Pin._states:
            pin_mod.Pin._states[id] = value

    def quiet_timer_init(self, period, mode=timer_mod.Timer.PERIODIC, callback=None):
        self.deinit()
        self._period   = period / 1000.0
        self._mode     = mode
        self._callback = callback
        self._running  = True
        self._thread   = threading.Thread(
            target = self._run,
            daemon = True,
            name   = f"Timer-{self._id}"
        )
        self._thread.start()
        timer_mod.Timer._active.append(self)

    def quiet_adc_init(self, pin):
        self._pin     = pin
        self._channel = adc_mod.ADC._PIN_TO_CHANNEL.get(pin, 0)

    pin_mod.Pin.__init__   = quiet_pin_init
    timer_mod.Timer.init   = quiet_timer_init
    adc_mod.ADC.__init__   = quiet_adc_init


# ── ECU loader ────────────────────────────────────────────────────────────────

class ECURunner:
    """
    Loads and runs an ECU firmware file in the current process.
    Handles graceful shutdown on Ctrl+C or SIGTERM.
    """

    def __init__(
        self,
        firmware_path : Path,
        ecu_id        : str,
        verbose       : bool = False,
    ):
        self.firmware_path = firmware_path
        self.ecu_id        = ecu_id
        self.verbose       = verbose
        self._running      = True

        signal.signal(signal.SIGINT,  self._on_signal)
        signal.signal(signal.SIGTERM, self._on_signal)

    def _on_signal(self, *_):
        print(f"\n[Runner:{self.ecu_id}] Shutting down...")
        self._running = False
        # Stop all timers
        from bus_broker.machine.timer import Timer
        Timer.stop_all()
        # Deinit all CAN instances
        self._deinit_can()
        sys.exit(0)

    def _deinit_can(self):
        """Find and deinit any CAN instances created by the ECU."""
        from bus_broker.machine.can import CAN
        # CAN instances are daemon threads so they die with the process
        # but we explicitly stop them for clean logging
        import gc
        for obj in gc.get_objects():
            if isinstance(obj, CAN):
                obj.deinit()

    def run(self):
        if not self.firmware_path.exists():
            print(
                f"[Runner:{self.ecu_id}] "
                f"Firmware file not found: {self.firmware_path}"
            )
            sys.exit(1)

        # Inject machine module before loading ECU code
        inject_machine_module(self.ecu_id, self.verbose)

        print(f"[Runner:{self.ecu_id}] Loading {self.firmware_path.name}")
        print(f"[Runner:{self.ecu_id}] Press Ctrl+C to stop.\n")

        try:
            import runpy
            runpy.run_path(
                str(self.firmware_path),
                run_name="__main__"
            )
        except KeyboardInterrupt:
            self._on_signal()
        except SystemExit:
            pass
        except Exception as e:
            print(f"\n[Runner:{self.ecu_id}] ECU crashed: {e}")
            import traceback
            traceback.print_exc()
            sys.exit(1)


# ── CLI ───────────────────────────────────────────────────────────────────────

def parse_args():
    parser = argparse.ArgumentParser(
        description="Run an ECU firmware file with the virtual CAN bus"
    )
    parser.add_argument(
        "firmware",
        type=Path,
        help="Path to ECU firmware file (e.g. firmware/engine_ecu/main.py)"
    )
    parser.add_argument(
        "--id",
        type=str,
        default=None,
        help="ECU identifier for log output (default: firmware filename)"
    )
    parser.add_argument(
        "--verbose", "-v",
        action="store_true",
        help="Show Pin/Timer/ADC setup messages"
    )
    return parser.parse_args()


def main():
    args   = parse_args()
    ecu_id = args.id or args.firmware.stem

    runner = ECURunner(
        firmware_path = args.firmware,
        ecu_id        = ecu_id,
        verbose       = args.verbose,
    )
    runner.run()


if __name__ == "__main__":
    main()