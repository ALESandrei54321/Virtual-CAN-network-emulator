# tests/test_launch_network.py

import pytest
import time
import tempfile
import yaml
from pathlib import Path
from unittest.mock import patch, MagicMock
import sys

sys.path.insert(0, str(Path(__file__).parent.parent / "tools"))

from launch_network import NetworkConfig, ECUProcess, OutputMux, NetworkLauncher


# ── Helpers ───────────────────────────────────────────────────────────────────

def write_config(content: dict) -> Path:
    f = tempfile.NamedTemporaryFile(
        mode="w", suffix=".yaml", delete=False
    )
    yaml.dump(content, f)
    f.close()
    return Path(f.name)


def simple_config() -> Path:
    return write_config({
        "bus"  : {"protocol": "CAN", "bit_rate": 500000},
        "ecus" : [
            {
                "id"       : "test_ecu",
                "firmware" : "firmware/engine_ecu/main.py",
                "filters"  : [0x02F, 0x01A],
            }
        ]
    })


# ── NetworkConfig ─────────────────────────────────────────────────────────────

def test_config_loads():
    path = simple_config()
    cfg  = NetworkConfig(path)
    assert cfg.protocol == "CAN"
    assert cfg.bit_rate == 500_000

def test_config_loads_ecus():
    path = simple_config()
    cfg  = NetworkConfig(path)
    assert len(cfg.ecus) == 1
    assert cfg.ecus[0]["id"] == "test_ecu"

def test_config_ecu_firmware_is_path():
    path = simple_config()
    cfg  = NetworkConfig(path)
    assert isinstance(cfg.ecus[0]["firmware"], Path)

def test_config_ecu_filters():
    path = simple_config()
    cfg  = NetworkConfig(path)
    assert cfg.ecus[0]["filters"] == [0x02F, 0x01A]

def test_config_missing_file_raises():
    with pytest.raises(FileNotFoundError):
        NetworkConfig(Path("/nonexistent/network.yaml"))

def test_config_default_protocol():
    path = write_config({
        "ecus": [{"id": "e", "firmware": "fw.py", "filters": []}]
    })
    cfg = NetworkConfig(path)
    assert cfg.protocol == "CAN"

def test_config_default_bit_rate():
    path = write_config({
        "ecus": [{"id": "e", "firmware": "fw.py", "filters": []}]
    })
    cfg = NetworkConfig(path)
    assert cfg.bit_rate == 500_000

def test_config_no_ecus():
    path = write_config({"bus": {"protocol": "CAN"}})
    cfg  = NetworkConfig(path)
    assert cfg.ecus == []

def test_config_multiple_ecus():
    path = write_config({
        "ecus": [
            {"id": "ecu0", "firmware": "f0.py", "filters": []},
            {"id": "ecu1", "firmware": "f1.py", "filters": []},
            {"id": "ecu2", "firmware": "f2.py", "filters": []},
        ]
    })
    cfg = NetworkConfig(path)
    assert len(cfg.ecus) == 3

def test_real_network_yaml_loads():
    path = Path(__file__).parent.parent / "network.yaml"
    cfg  = NetworkConfig(path)
    assert len(cfg.ecus) == 4
    ids  = [e["id"] for e in cfg.ecus]
    assert "engine_ecu"  in ids
    assert "chassis_ecu" in ids
    assert "body_ecu"    in ids
    assert "gateway_ecu" in ids


# ── ECUProcess ────────────────────────────────────────────────────────────────

def test_ecu_process_initial_state():
    ecu = ECUProcess("test", Path("fw.py"))
    assert ecu.ecu_id   == "test"
    assert ecu.firmware == Path("fw.py")
    assert ecu.process  is None

def test_ecu_process_not_running_before_start():
    ecu = ECUProcess("test", Path("fw.py"))
    assert ecu.is_running() is False

def test_ecu_process_stop_when_not_started():
    ecu = ECUProcess("test", Path("fw.py"))
    ecu.stop()   # should not raise

def test_ecu_process_starts_real_firmware():
    """Start the engine ECU for a short time and check it runs."""
    fw  = Path(__file__).parent.parent / "firmware/engine_ecu/main.py"
    ecu = ECUProcess("engine_ecu", fw)
    ecu.start_inline()
    time.sleep(0.5)
    assert ecu.is_running()
    ecu.stop()
    assert not ecu.is_running()

def test_ecu_process_stop_terminates():
    fw  = Path(__file__).parent.parent / "firmware/engine_ecu/main.py"
    ecu = ECUProcess("engine_ecu", fw)
    ecu.start_inline()
    time.sleep(0.3)
    ecu.stop()
    time.sleep(0.2)
    assert not ecu.is_running()


# ── OutputMux ─────────────────────────────────────────────────────────────────

def test_output_mux_creation():
    ecu = ECUProcess("test", Path("fw.py"))
    mux = OutputMux([ecu])
    assert "test" in mux._colours

def test_output_mux_multiple_ecus():
    ecus = [
        ECUProcess("ecu0", Path("fw.py")),
        ECUProcess("ecu1", Path("fw.py")),
        ECUProcess("ecu2", Path("fw.py")),
    ]
    mux = OutputMux(ecus)
    assert len(mux._colours) == 3

def test_output_mux_poll_no_crash():
    """Poll with no running ECUs should not raise."""
    ecu = ECUProcess("test", Path("fw.py"))
    mux = OutputMux([ecu])
    mux.poll()   # ecu not started - should be fine


# ── NetworkLauncher ───────────────────────────────────────────────────────────

def test_launcher_creation():
    path     = simple_config()
    config   = NetworkConfig(path)
    launcher = NetworkLauncher(config)
    assert launcher._running is True

def test_launcher_print_header(capsys):
    path     = simple_config()
    config   = NetworkConfig(path)
    launcher = NetworkLauncher(config)
    launcher._print_header()
    out = capsys.readouterr().out
    assert "CAN"      in out
    assert "test_ecu" in out

def test_launcher_shutdown_no_crash():
    path     = simple_config()
    config   = NetworkConfig(path)
    launcher = NetworkLauncher(config)
    launcher._shutdown()   # nothing started - should not raise

def test_launcher_starts_and_stops_ecus():
    """
    Launch the real network for a short time
    then stop it cleanly.
    """
    path     = Path(__file__).parent.parent / "network.yaml"
    config   = NetworkConfig(path)
    launcher = NetworkLauncher(config, inline=True)

    # Replace the blocking loop with a short run
    import threading

    def stop_soon():
        time.sleep(1.5)
        launcher._running = False

    stopper = threading.Thread(target=stop_soon, daemon=True)
    stopper.start()

    launcher.launch()

    # All ECUs should be stopped after launch returns
    for ecu in launcher._ecus:
        assert not ecu.is_running()