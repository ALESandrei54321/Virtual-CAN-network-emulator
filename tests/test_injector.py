# tests/test_injector.py

import pytest
import asyncio
from pathlib import Path
import yaml
import tempfile
import os

import sys
sys.path.insert(0, str(Path(__file__).parent.parent / "tools"))

from injector import InjectorConfig, build_frame, dry_run
from bus_broker.core.frames import CANFrame, Protocol


# ── Helpers ───────────────────────────────────────────────────────────────────

def write_yaml(content: dict) -> Path:
    """Write a dict as YAML to a temp file, return the path."""
    f = tempfile.NamedTemporaryFile(
        mode="w", suffix=".yaml", delete=False
    )
    yaml.dump(content, f)
    f.close()
    return Path(f.name)


@pytest.fixture(autouse=True)
def cleanup_temps(tmp_path):
    yield
    # temp files cleaned up by OS


# ── InjectorConfig loading ────────────────────────────────────────────────────

def test_load_basic_yaml():
    path   = write_yaml({
        "bus"    : {"protocol": "CAN", "bit_rate": 500000},
        "frames" : [{"id": "0x123", "dlc": 4, "data": "DE AD BE EF"}]
    })
    config = InjectorConfig(path)
    assert config.protocol_name == "CAN"
    assert config.bit_rate      == 500000
    assert len(config.frames)   == 1

def test_missing_file_raises():
    with pytest.raises(FileNotFoundError):
        InjectorConfig(Path("/nonexistent/path.yaml"))

def test_empty_frames_raises():
    path = write_yaml({
        "bus"    : {"protocol": "CAN"},
        "frames" : []
    })
    with pytest.raises(ValueError, match="No frames"):
        InjectorConfig(path)

def test_unknown_protocol_raises():
    path = write_yaml({
        "bus"    : {"protocol": "FLEXRAY"},
        "frames" : [{"id": "0x1", "dlc": 0}]
    })
    with pytest.raises(ValueError, match="Unknown protocol"):
        InjectorConfig(path)

def test_default_protocol_is_can():
    path   = write_yaml({
        "frames": [{"id": "0x1", "dlc": 0}]
    })
    config = InjectorConfig(path)
    assert config.protocol_name == "CAN"

def test_default_bit_rate():
    path   = write_yaml({
        "frames": [{"id": "0x1", "dlc": 0}]
    })
    config = InjectorConfig(path)
    assert config.bit_rate == 500_000


# ── Frame parsing ─────────────────────────────────────────────────────────────

def test_parse_hex_id_string():
    path   = write_yaml({
        "frames": [{"id": "0x123", "dlc": 4, "data": "DEADBEEF"}]
    })
    config = InjectorConfig(path)
    assert config.frames[0]["arbitration_id"] == 0x123

def test_parse_integer_id():
    path   = write_yaml({
        "frames": [{"id": 0x123, "dlc": 4, "data": "DEADBEEF"}]
    })
    config = InjectorConfig(path)
    assert config.frames[0]["arbitration_id"] == 0x123

def test_parse_data_with_spaces():
    path   = write_yaml({
        "frames": [{"id": "0x1", "dlc": 4, "data": "DE AD BE EF"}]
    })
    config = InjectorConfig(path)
    assert config.frames[0]["data"] == bytes([0xDE, 0xAD, 0xBE, 0xEF])

def test_parse_data_without_spaces():
    path   = write_yaml({
        "frames": [{"id": "0x1", "dlc": 4, "data": "DEADBEEF"}]
    })
    config = InjectorConfig(path)
    assert config.frames[0]["data"] == bytes([0xDE, 0xAD, 0xBE, 0xEF])

def test_parse_empty_data():
    path   = write_yaml({
        "frames": [{"id": "0x1", "dlc": 0}]
    })
    config = InjectorConfig(path)
    assert config.frames[0]["data"] == b""

def test_parse_extended_flag():
    path   = write_yaml({
        "frames": [{"id": "0x12345678", "dlc": 0, "extended": True}]
    })
    config = InjectorConfig(path)
    assert config.frames[0]["is_extended"] is True

def test_parse_remote_flag():
    path   = write_yaml({
        "frames": [{"id": "0x123", "dlc": 4, "remote": True}]
    })
    config = InjectorConfig(path)
    assert config.frames[0]["is_remote"] is True

def test_parse_delay():
    path   = write_yaml({
        "frames": [{"id": "0x1", "dlc": 0, "delay_after_ms": 25.5}]
    })
    config = InjectorConfig(path)
    assert config.frames[0]["delay_after_ms"] == 25.5

def test_parse_comment():
    path   = write_yaml({
        "frames": [{"id": "0x1", "dlc": 0, "comment": "test frame"}]
    })
    config = InjectorConfig(path)
    assert config.frames[0]["comment"] == "test frame"

def test_missing_id_raises():
    path = write_yaml({
        "frames": [{"dlc": 4, "data": "DEADBEEF"}]
    })
    with pytest.raises(ValueError, match="Frame 0"):
        InjectorConfig(path)

def test_odd_hex_data_raises():
    path = write_yaml({
        "frames": [{"id": "0x1", "dlc": 1, "data": "ABC"}]
    })
    with pytest.raises(ValueError):
        InjectorConfig(path)

def test_invalid_hex_data_raises():
    path = write_yaml({
        "frames": [{"id": "0x1", "dlc": 1, "data": "ZZ"}]
    })
    with pytest.raises(ValueError):
        InjectorConfig(path)

def test_multiple_frames_parsed():
    path   = write_yaml({
        "frames": [
            {"id": "0x1", "dlc": 0},
            {"id": "0x2", "dlc": 0},
            {"id": "0x3", "dlc": 0},
        ]
    })
    config = InjectorConfig(path)
    assert len(config.frames) == 3


# ── build_frame ───────────────────────────────────────────────────────────────

def test_build_can_frame():
    entry = {
        "arbitration_id" : 0x123,
        "dlc"            : 4,
        "data"           : bytes([0xDE, 0xAD, 0xBE, 0xEF]),
        "is_extended"    : False,
        "is_remote"      : False,
        "comment"        : "",
        "delay_after_ms" : 0,
    }
    frame = build_frame(entry, "CAN")
    assert isinstance(frame, CANFrame)
    assert frame.arbitration_id == 0x123
    assert frame.protocol       == Protocol.CAN
    assert frame.data           == bytes([0xDE, 0xAD, 0xBE, 0xEF])

def test_build_can_fd_frame():
    entry = {
        "arbitration_id" : 0x200,
        "dlc"            : 9,
        "data"           : bytes(12),
        "is_extended"    : False,
        "is_remote"      : False,
        "comment"        : "",
        "delay_after_ms" : 0,
    }
    frame = build_frame(entry, "CAN_FD")
    assert frame.protocol == Protocol.CAN_FD

def test_build_extended_frame():
    entry = {
        "arbitration_id" : 0x12345678,
        "dlc"            : 0,
        "data"           : b"",
        "is_extended"    : True,
        "is_remote"      : False,
        "comment"        : "",
        "delay_after_ms" : 0,
    }
    frame = build_frame(entry, "CAN")
    assert frame.is_extended is True


# ── Example files ─────────────────────────────────────────────────────────────

def test_example_basic_loads():
    path   = Path(__file__).parent.parent / "tools/examples/basic.yaml"
    config = InjectorConfig(path)
    assert len(config.frames) > 0

def test_example_arbitration_loads():
    path   = Path(__file__).parent.parent / "tools/examples/arbitration.yaml"
    config = InjectorConfig(path)
    assert len(config.frames) == 3

def test_example_extended_loads():
    path   = Path(__file__).parent.parent / "tools/examples/extended.yaml"
    config = InjectorConfig(path)
    assert len(config.frames) > 0

def test_example_canfd_loads():
    path   = Path(__file__).parent.parent / "tools/examples/canfd.yaml"
    config = InjectorConfig(path)
    assert config.protocol_name == "CAN_FD"


# ── Dry run (smoke test - just check it does not crash) ───────────────────────

def test_dry_run_does_not_raise(capsys):
    path   = write_yaml({
        "bus"    : {"protocol": "CAN", "bit_rate": 500000},
        "frames" : [
            {"id": "0x123", "dlc": 4, "data": "DE AD BE EF", "comment": "test"}
        ]
    })
    config = InjectorConfig(path)
    dry_run(config, verbose=True)
    captured = capsys.readouterr()
    assert "0x123" in captured.out
