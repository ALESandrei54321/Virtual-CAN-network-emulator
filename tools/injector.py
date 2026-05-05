# tools/injector.py

"""
YAML Packet Injector
====================
Reads a YAML file describing CAN frames and injects them
into the virtual bus via the bus controller.

Usage:
    python tools/injector.py tools/examples/basic.yaml
    python tools/injector.py tools/examples/basic.yaml --verbose
    python tools/injector.py tools/examples/basic.yaml --repeat 3
    python tools/injector.py tools/examples/arbitration.yaml --dry-run
"""

import asyncio
import argparse
import sys
import time
from pathlib import Path

import yaml

# Make sure we can import bus_broker regardless of where we run from
sys.path.insert(0, str(Path(__file__).parent.parent))

from bus_broker.core.frames import CANFrame, Protocol
from bus_broker.core.bus_controller import BusController
from bus_broker.protocols.registry import get_protocol
from bus_broker.transport.shm_writer import SHMBusWriter


# ── YAML parsing ──────────────────────────────────────────────────────────────

class InjectorConfig:
    """Parsed and validated content of a YAML input file."""

    def __init__(self, path: Path):
        self.path = path
        self._raw = self._load(path)
        self.protocol_name = self._parse_protocol()
        self.bit_rate      = self._raw.get("bus", {}).get("bit_rate", 500_000)
        self.data_bit_rate = self._raw.get("bus", {}).get("data_bit_rate", 2_000_000)
        self.frames        = self._parse_frames()

    def _load(self, path: Path) -> dict:
        if not path.exists():
            raise FileNotFoundError(f"Input file not found: {path}")
        with open(path) as f:
            raw = yaml.safe_load(f)
        if not isinstance(raw, dict):
            raise ValueError("YAML file must be a mapping at the top level")
        return raw

    def _parse_protocol(self) -> str:
        bus      = self._raw.get("bus", {})
        protocol = bus.get("protocol", "CAN").upper()
        valid    = ["CAN", "CAN_FD"]
        if protocol not in valid:
            raise ValueError(
                f"Unknown protocol '{protocol}'. Valid: {valid}"
            )
        return protocol

    def _parse_frames(self) -> list[dict]:
        raw_frames = self._raw.get("frames", [])
        if not raw_frames:
            raise ValueError("No frames defined in YAML file")

        parsed = []
        for i, entry in enumerate(raw_frames):
            try:
                parsed.append(self._parse_one_frame(entry))
            except (KeyError, ValueError) as e:
                raise ValueError(f"Frame {i}: {e}") from e
        return parsed

    def _parse_one_frame(self, entry: dict) -> dict:
        # ID - required
        if "id" not in entry:
            raise KeyError("'id' is required")

        raw_id = entry["id"]
        if isinstance(raw_id, str):
            arb_id = int(raw_id, 16)
        else:
            arb_id = int(raw_id)

        # DLC - required unless it can be inferred from data
        data_str = entry.get("data", "")
        data     = self._parse_data(data_str)

        if "dlc" in entry:
            dlc = int(entry["dlc"])
        else:
            dlc = len(data)

        return {
            "arbitration_id" : arb_id,
            "dlc"            : dlc,
            "data"           : data,
            "is_extended"    : bool(entry.get("extended", False)),
            "is_remote"      : bool(entry.get("remote",   False)),
            "comment"        : entry.get("comment", ""),
            "delay_after_ms" : float(entry.get("delay_after_ms", 0)),
        }

    def _parse_data(self, data_str: str) -> bytes:
        """
        Parse hex data string. Accepts:
          "DE AD BE EF"
          "DEADBEEF"
          ""  (empty)
        """
        if not data_str:
            return b""
        cleaned = data_str.replace(" ", "").replace("\n", "")
        if len(cleaned) % 2 != 0:
            raise ValueError(
                f"Hex data '{data_str}' has odd number of characters"
            )
        try:
            return bytes.fromhex(cleaned)
        except ValueError as e:
            raise ValueError(f"Invalid hex data '{data_str}': {e}") from e


# ── Frame builder ─────────────────────────────────────────────────────────────

def build_frame(entry: dict, protocol_name: str) -> CANFrame:
    protocol = Protocol.CAN if protocol_name == "CAN" else Protocol.CAN_FD
    return CANFrame(
        arbitration_id = entry["arbitration_id"],
        dlc            = entry["dlc"],
        data           = entry["data"],
        protocol       = protocol,
        is_extended    = entry["is_extended"],
        is_remote      = entry["is_remote"],
    )


# ── Dry run (no shm needed) ───────────────────────────────────────────────────

def dry_run(config: InjectorConfig, verbose: bool):
    """
    Parse and display frames without touching shared memory.
    Useful for verifying a YAML file is correct.
    """
    print(f"\n{'─'*60}")
    print(f"  DRY RUN: {config.path.name}")
    print(f"  Protocol : {config.protocol_name}")
    print(f"  Bit rate : {config.bit_rate:,} bit/s")
    print(f"  Frames   : {len(config.frames)}")
    print(f"{'─'*60}\n")

    for i, entry in enumerate(config.frames):
        frame = build_frame(entry, config.protocol_name)
        _print_frame(i, entry, frame, verbose)

    print(f"\nDry run complete. {len(config.frames)} frames validated.\n")


# ── Live injection ────────────────────────────────────────────────────────────

async def inject(
    config  : InjectorConfig,
    repeat  : int,
    verbose : bool,
    burst   : bool,
):
    """
    Inject frames into the live bus controller.
    """
    kwargs = {"bit_rate": config.bit_rate}
    if config.protocol_name == "CAN_FD":
        kwargs["nominal_bit_rate"] = config.bit_rate
        kwargs["data_bit_rate"]    = config.data_bit_rate
        del kwargs["bit_rate"]

    protocol = get_protocol(config.protocol_name, **kwargs)

    print(f"\n{'─'*60}")
    print(f"  INJECTING: {config.path.name}")
    print(f"  Protocol : {config.protocol_name}")
    print(f"  Bit rate : {config.bit_rate:,} bit/s")
    print(f"  Frames   : {len(config.frames)} × {repeat} = "
          f"{len(config.frames) * repeat} total")
    print(f"  Burst    : {'yes (no delays)' if burst else 'no'}")
    print(f"{'─'*60}\n")

    with SHMBusWriter() as writer:
        sent_callback_frames = []

        def on_sent(frame: CANFrame):
            sent_callback_frames.append(frame)

        controller = BusController(protocol, writer, on_frame_sent=on_sent)

        total_sent = 0

        for run in range(repeat):
            if repeat > 1:
                print(f"--- Run {run + 1}/{repeat} ---")

            if burst:
                # Submit all frames at once then drain
                for entry in config.frames:
                    frame = build_frame(entry, config.protocol_name)
                    await controller.submit(frame)

                # Drain the queue
                while controller._queue:
                    await controller._tick()

                total_sent += len(config.frames)

            else:
                # Submit one at a time with delays
                for i, entry in enumerate(config.frames):
                    frame = build_frame(entry, config.protocol_name)

                    if verbose:
                        _print_frame(i, entry, frame, verbose=True)

                    await controller.submit(frame)
                    await controller._tick()
                    total_sent += 1

                    delay_ms = entry["delay_after_ms"]
                    if delay_ms > 0:
                        await asyncio.sleep(delay_ms / 1000)

        # Final stats
        print(f"\n{'─'*60}")
        print(f"  Done.")
        print(f"  Frames submitted : {total_sent}")
        print(f"  Frames sent      : {controller.stats.frames_sent}")
        print(f"  Frames dropped   : {controller.stats.frames_dropped}")
        print(f"  Arb events       : {controller.stats.arbitration_events}")
        if controller.stats.frames_sent > 0:
            print(
                f"  Avg latency      : "
                f"{controller.stats.avg_latency_ns/1000:.2f} μs"
            )
        print(f"{'─'*60}\n")


# ── Display helpers ───────────────────────────────────────────────────────────

def _print_frame(i: int, entry: dict, frame: CANFrame, verbose: bool):
    id_str   = (
        f"0x{frame.arbitration_id:08X}"
        if frame.is_extended
        else f"0x{frame.arbitration_id:03X}"
    )
    data_hex = frame.data.hex().upper() if frame.data else "(none)"
    comment  = f"  # {entry['comment']}" if entry.get("comment") else ""
    flags    = ""
    if frame.is_extended:
        flags += " [EXT]"
    if frame.is_remote:
        flags += " [RTR]"

    print(
        f"  [{i:>4}] "
        f"{frame.protocol.name:<6} "
        f"ID={id_str}  "
        f"DLC={frame.dlc}  "
        f"DATA={data_hex:<20}"
        f"{flags}"
        f"{comment}"
    )

    if verbose and entry.get("delay_after_ms", 0) > 0:
        print(f"         delay={entry['delay_after_ms']}ms")


# ── CLI ───────────────────────────────────────────────────────────────────────

def parse_args():
    parser = argparse.ArgumentParser(
        description="Inject CAN frames from a YAML file into the virtual bus"
    )
    parser.add_argument(
        "file",
        type=Path,
        help="Path to the YAML frame definition file"
    )
    parser.add_argument(
        "--verbose", "-v",
        action="store_true",
        help="Print each frame as it is injected"
    )
    parser.add_argument(
        "--dry-run", "-n",
        action="store_true",
        help="Parse and display frames without injecting (no shm needed)"
    )
    parser.add_argument(
        "--repeat", "-r",
        type=int,
        default=1,
        help="Repeat the sequence N times (default: 1)"
    )
    parser.add_argument(
        "--burst", "-b",
        action="store_true",
        help="Submit all frames at once to test arbitration"
    )
    return parser.parse_args()


def main():
    args   = parse_args()
    config = InjectorConfig(args.file)

    if args.dry_run:
        dry_run(config, verbose=args.verbose)
    else:
        asyncio.run(inject(
            config  = config,
            repeat  = args.repeat,
            verbose = args.verbose,
            burst   = args.burst,
        ))


if __name__ == "__main__":
    main()
