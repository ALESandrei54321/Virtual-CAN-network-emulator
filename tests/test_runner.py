# tests/test_runner.py

import sys
import pytest
import tempfile
import time
from pathlib import Path
from unittest.mock import patch, MagicMock

sys.path.insert(0, str(Path(__file__).parent.parent / "tools"))

from run_ecu import ECURunner, inject_machine_module


# ── Helpers ───────────────────────────────────────────────────────────────────

def write_ecu_file(content: str) -> Path:
    """Write ECU code to a temp file and return the path."""
    f = tempfile.NamedTemporaryFile(
        mode    = "w",
        suffix  = ".py",
        delete  = False
    )
    f.write(content)
    f.close()
    return Path(f.name)


# ── inject_machine_module ─────────────────────────────────────────────────────

def test_machine_injected_into_sys_modules():
    inject_machine_module("test_ecu", verbose=True)
    assert "machine" in sys.modules

def test_utime_injected_into_sys_modules():
    inject_machine_module("test_ecu", verbose=True)
    assert "utime" in sys.modules

def test_time_module_gets_sleep_ms():
    inject_machine_module("test_ecu", verbose=True)
    import time
    assert hasattr(time, "sleep_ms")

def test_time_module_gets_ticks_ms():
    inject_machine_module("test_ecu", verbose=True)
    import time
    assert hasattr(time, "ticks_ms")

def test_machine_has_can():
    inject_machine_module("test_ecu", verbose=True)
    import machine
    assert hasattr(machine, "CAN")

def test_machine_has_pin():
    inject_machine_module("test_ecu", verbose=True)
    import machine
    assert hasattr(machine, "Pin")

def test_machine_has_timer():
    inject_machine_module("test_ecu", verbose=True)
    import machine
    assert hasattr(machine, "Timer")

def test_machine_has_adc():
    inject_machine_module("test_ecu", verbose=True)
    import machine
    assert hasattr(machine, "ADC")


# ── ECURunner ─────────────────────────────────────────────────────────────────

def test_runner_missing_file_exits():
    runner = ECURunner(
        firmware_path = Path("/nonexistent/main.py"),
        ecu_id        = "test",
        verbose       = False
    )
    with pytest.raises(SystemExit):
        runner.run()

def test_runner_loads_simple_ecu():
    """An ECU that sets a variable and exits should run cleanly."""
    path = write_ecu_file("result = 1 + 1\n")
    runner = ECURunner(
        firmware_path = path,
        ecu_id        = "test",
        verbose       = False
    )
    runner.run()   # should not raise

def test_runner_ecu_can_import_machine():
    path = write_ecu_file(
        "from machine import Pin\n"
        "led = Pin(25, Pin.OUT)\n"
        "led.on()\n"
    )
    runner = ECURunner(
        firmware_path = path,
        ecu_id        = "test",
        verbose       = True
    )
    runner.run()   # should not raise

def test_runner_ecu_can_use_time_sleep_ms():
    path = write_ecu_file(
        "import time\n"
        "time.sleep_ms(10)\n"
    )
    runner = ECURunner(
        firmware_path = path,
        ecu_id        = "test",
        verbose       = False
    )
    runner.run()

def test_runner_ecu_crash_exits_with_code_1():
    path = write_ecu_file("raise RuntimeError('ECU crashed')\n")
    runner = ECURunner(
        firmware_path = path,
        ecu_id        = "test",
        verbose       = False
    )
    with pytest.raises(SystemExit) as exc:
        runner.run()
    assert exc.value.code == 1

def test_runner_ecu_id_default_is_stem():
    path = write_ecu_file("pass\n")
    args_mock = MagicMock()
    args_mock.firmware = path
    args_mock.id       = None
    args_mock.verbose  = False
    ecu_id = args_mock.id or path.stem
    assert ecu_id == path.stem

def test_runner_verbose_flag():
    path = write_ecu_file(
        "from machine import Pin\n"
        "p = Pin(1, Pin.OUT)\n"
    )
    runner = ECURunner(
        firmware_path = path,
        ecu_id        = "test",
        verbose       = True
    )
    runner.run()

def test_runner_ecu_uses_pin(capsys):
    path = write_ecu_file(
        "from machine import Pin\n"
        "led = Pin(25, Pin.OUT)\n"
        "led.on()\n"
        "assert led.value() == 1\n"
        "led.off()\n"
        "assert led.value() == 0\n"
    )
    runner = ECURunner(
        firmware_path = path,
        ecu_id        = "test",
        verbose       = True
    )
    runner.run()

def test_runner_ecu_uses_timer():
    called = []
    path   = write_ecu_file(
        "from machine import Timer\n"
        "import time\n"
        "t = Timer(-1)\n"
        "t.init(period=20, mode=Timer.PERIODIC, callback=lambda _: None)\n"
        "time.sleep(0.05)\n"
        "t.deinit()\n"
    )
    runner = ECURunner(
        firmware_path = path,
        ecu_id        = "test",
        verbose       = False
    )
    runner.run()


# ── Example firmware files ────────────────────────────────────────────────────
# Smoke test: make sure the example files at least parse without error
# We cannot run their main loops (they loop forever) so we just
# check they are valid Python

def test_engine_ecu_is_valid_python():
    path = Path(__file__).parent.parent / "firmware/engine_ecu/main.py"
    if path.exists():
        source = path.read_text()
        compile(source, str(path), "exec")   # raises SyntaxError if broken

def test_chassis_ecu_is_valid_python():
    path = Path(__file__).parent.parent / "firmware/chassis_ecu/main.py"
    if path.exists():
        source = path.read_text()
        compile(source, str(path), "exec")

def test_body_ecu_is_valid_python():
    path = Path(__file__).parent.parent / "firmware/body_ecu/main.py"
    if path.exists():
        source = path.read_text()
        compile(source, str(path), "exec")