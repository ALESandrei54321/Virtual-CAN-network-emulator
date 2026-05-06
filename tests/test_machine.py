# tests/test_machine.py

import pytest
import time
import threading
from unittest.mock import patch, MagicMock

from bus_broker.machine.pin   import Pin
from bus_broker.machine.timer import Timer
from bus_broker.machine.adc   import ADC
from bus_broker.machine.time_compat import (
    sleep_ms, sleep_us, ticks_ms, ticks_us, ticks_diff
)


# ── Pin ───────────────────────────────────────────────────────────────────────

@pytest.fixture(autouse=True)
def clear_pin_state():
    """Reset shared Pin state between tests."""
    Pin._states.clear()
    yield
    Pin._states.clear()


def test_pin_out_creation():
    p = Pin(25, Pin.OUT)
    assert p._mode == Pin.OUT

def test_pin_in_creation():
    p = Pin(14, Pin.IN)
    assert p._mode == Pin.IN

def test_pin_on():
    p = Pin(25, Pin.OUT)
    p.on()
    assert p.value() == 1

def test_pin_off():
    p = Pin(25, Pin.OUT)
    p.on()
    p.off()
    assert p.value() == 0

def test_pin_toggle():
    p = Pin(25, Pin.OUT)
    p.toggle()
    assert p.value() == 1
    p.toggle()
    assert p.value() == 0

def test_pin_value_set():
    p = Pin(25, Pin.OUT)
    p.value(1)
    assert p.value() == 1
    p.value(0)
    assert p.value() == 0

def test_pin_value_get_default():
    p = Pin(25, Pin.OUT)
    assert p.value() == 0

def test_pin_state_shared():
    """Two Pin objects on same id share state."""
    p1 = Pin(10, Pin.OUT)
    p2 = Pin(10, Pin.IN)
    p1.on()
    assert p2.value() == 1

def test_pin_repr():
    p = Pin(25, Pin.OUT)
    assert "GP25" in repr(p)

def test_pin_pull_up():
    p = Pin(14, Pin.IN, Pin.PULL_UP)
    assert p._pull == Pin.PULL_UP


# ── Timer ─────────────────────────────────────────────────────────────────────

@pytest.fixture(autouse=True)
def stop_timers():
    yield
    Timer.stop_all()


def test_timer_creation():
    t = Timer(-1)
    assert t._running is False

def test_timer_periodic_calls_callback():
    called = []
    t = Timer(-1)
    t.init(period=20, mode=Timer.PERIODIC, callback=lambda _: called.append(1))
    time.sleep(0.07)   # wait for ~3 callbacks
    t.deinit()
    assert len(called) >= 2

def test_timer_one_shot_calls_once():
    called = []
    t = Timer(-1)
    t.init(period=20, mode=Timer.ONE_SHOT, callback=lambda _: called.append(1))
    time.sleep(0.1)
    assert len(called) == 1

def test_timer_deinit_stops_callbacks():
    called = []
    t = Timer(-1)
    t.init(period=20, mode=Timer.PERIODIC, callback=lambda _: called.append(1))
    time.sleep(0.05)
    t.deinit()
    count_after_deinit = len(called)
    time.sleep(0.05)
    assert len(called) == count_after_deinit

def test_timer_repr():
    t = Timer(0)
    assert "Timer" in repr(t)

def test_timer_stop_all():
    t1 = Timer(0)
    t2 = Timer(1)
    t1.init(period=50, mode=Timer.PERIODIC, callback=lambda _: None)
    t2.init(period=50, mode=Timer.PERIODIC, callback=lambda _: None)
    Timer.stop_all()
    assert len(Timer._active) == 0


# ── ADC ───────────────────────────────────────────────────────────────────────

@pytest.fixture(autouse=True)
def clear_adc_state():
    ADC._sim_values.clear()
    yield
    ADC._sim_values.clear()


def test_adc_creation():
    a = ADC(26)
    assert a._pin == 26

def test_adc_default_value():
    a = ADC(26)
    assert a.read_u16() == 32768

def test_adc_sim_value():
    ADC.set_sim_value(26, 1000)
    a = ADC(26)
    assert a.read_u16() == 1000

def test_adc_sim_value_clamped():
    ADC.set_sim_value(26, 99999)
    a = ADC(26)
    assert a.read_u16() == 65535

def test_adc_read_uv_range():
    a = ADC(26)
    uv = a.read_uv()
    assert 0 <= uv <= 3_300_000

def test_adc_repr():
    a = ADC(26)
    assert "GP26" in repr(a)


# ── time_compat ───────────────────────────────────────────────────────────────

def test_sleep_ms():
    start = time.monotonic()
    sleep_ms(50)
    elapsed = time.monotonic() - start
    assert elapsed >= 0.045   # allow small tolerance

def test_sleep_us():
    start = time.monotonic()
    sleep_us(10000)   # 10ms
    elapsed = time.monotonic() - start
    assert elapsed >= 0.009

def test_ticks_ms_increases():
    t1 = ticks_ms()
    time.sleep(0.05)
    t2 = ticks_ms()
    assert t2 > t1

def test_ticks_us_increases():
    t1 = ticks_us()
    time.sleep(0.01)
    t2 = ticks_us()
    assert t2 > t1

def test_ticks_diff():
    t1 = ticks_ms()
    time.sleep(0.05)
    t2 = ticks_ms()
    diff = ticks_diff(t2, t1)
    assert diff >= 40


# ── CAN (without live shm) ────────────────────────────────────────────────────
# We test the CAN class logic without needing the broker running
# by mocking the shm layer.

def test_can_filter_mask16():
    from bus_broker.machine.can import CAN
    with patch("bus_broker.machine.can.SHMBusReader"), \
         patch("bus_broker.machine.can.SHMBusWriter"):
        can = CAN(0)
        can.setfilter(0, CAN.MASK16, 0, (0x100, 0x7FF))
        assert can._passes_filter(0x100) is True
        assert can._passes_filter(0x101) is False
        can.deinit()

def test_can_filter_list16():
    from bus_broker.machine.can import CAN
    with patch("bus_broker.machine.can.SHMBusReader"), \
         patch("bus_broker.machine.can.SHMBusWriter"):
        can = CAN(0)
        can.setfilter(0, CAN.LIST16, 0, (0x100, 0x200, 0x300))
        assert can._passes_filter(0x100) is True
        assert can._passes_filter(0x200) is True
        assert can._passes_filter(0x400) is False
        can.deinit()

def test_can_no_filter_accepts_all():
    from bus_broker.machine.can import CAN
    with patch("bus_broker.machine.can.SHMBusReader"), \
         patch("bus_broker.machine.can.SHMBusWriter"):
        can = CAN(0)
        assert can._passes_filter(0x000) is True
        assert can._passes_filter(0x7FF) is True
        can.deinit()

def test_can_clear_filter():
    from bus_broker.machine.can import CAN
    with patch("bus_broker.machine.can.SHMBusReader"), \
         patch("bus_broker.machine.can.SHMBusWriter"):
        can = CAN(0)
        can.setfilter(0, CAN.MASK16, 0, (0x100, 0x7FF))
        can.clearfilter()
        assert can._passes_filter(0x999) is True
        can.deinit()

def test_can_state():
    from bus_broker.machine.can import CAN
    with patch("bus_broker.machine.can.SHMBusReader"), \
         patch("bus_broker.machine.can.SHMBusWriter"):
        can = CAN(0)
        assert can.state() == CAN.ERROR_ACTIVE
        can.deinit()

def test_can_info():
    from bus_broker.machine.can import CAN
    with patch("bus_broker.machine.can.SHMBusReader"), \
         patch("bus_broker.machine.can.SHMBusWriter"):
        can = CAN(0)
        info = can.info()
        assert isinstance(info, list)
        assert len(info) == 6
        can.deinit()

def test_can_repr():
    from bus_broker.machine.can import CAN
    with patch("bus_broker.machine.can.SHMBusReader"), \
         patch("bus_broker.machine.can.SHMBusWriter"):
        can = CAN(0)
        assert "CAN" in repr(can)
        can.deinit()

def test_can_any_empty():
    from bus_broker.machine.can import CAN
    with patch("bus_broker.machine.can.SHMBusReader"), \
         patch("bus_broker.machine.can.SHMBusWriter"):
        can = CAN(0)
        assert can.any() is False
        can.deinit()

def test_can_recv_timeout():
    from bus_broker.machine.can import CAN
    with patch("bus_broker.machine.can.SHMBusReader"), \
         patch("bus_broker.machine.can.SHMBusWriter"):
        can = CAN(0)
        with pytest.raises(OSError, match="timeout"):
            can.recv(timeout=50)
        can.deinit()