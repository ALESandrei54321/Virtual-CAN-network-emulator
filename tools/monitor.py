# tools/monitor.py

"""
Bus Monitor
===========
Reads frames from shared memory in real time and displays them.
Run this in a separate terminal while the injector is running.

Usage:
    python tools/monitor.py
    python tools/monitor.py --verbose
    python tools/monitor.py --filter 0x123 0x456
    python tools/monitor.py --count 10
    python tools/monitor.py --stats
"""

import sys
import time
import argparse
import signal
from pathlib import Path
from dataclasses import dataclass, field
from collections import defaultdict

sys.path.insert(0, str(Path(__file__).parent.parent))

from bus_broker.transport.shm_writer import SHMBusReader, BusFrameSlot
from bus_broker.protocols.registry import get_protocol
from bus_broker.protocols.base_protocol import BaseProtocol
from bus_broker.core.signal import SignalConverter, DifferentialSignal
from bus_broker.core.frames import CANFrame, Protocol
from bus_broker.transport.shm_writer import (
    SHMBusReader,
    BusFrameSlot,
    make_slot,
)


# ── Frame record ──────────────────────────────────────────────────────────────

@dataclass
class FrameRecord:
    """
    A decoded frame plus display metadata.
    """
    slot          : BusFrameSlot
    frame         : CANFrame | None   # None if decode failed
    received_ns   : int
    decode_error  : str = ""

    @property
    def age_ms(self) -> float:
        return (time.time_ns() - self.received_ns) / 1_000_000

    @property
    def timestamp_str(self) -> str:
        ts_ns  = self.slot.timestamp_ns
        ts_s   = ts_ns / 1_000_000_000
        h      = int(ts_s // 3600) % 24
        m      = int(ts_s // 60)   % 60
        s      = ts_s % 60
        return f"{h:02d}:{m:02d}:{s:06.3f}"


# ── Statistics tracker ────────────────────────────────────────────────────────

@dataclass
class MonitorStats:
    start_ns      : int = field(default_factory=time.time_ns)
    total_frames  : int = 0
    decode_errors : int = 0
    frames_by_id  : dict = field(default_factory=lambda: defaultdict(int))
    bytes_by_id   : dict = field(default_factory=lambda: defaultdict(int))

    def record(self, record: FrameRecord):
        self.total_frames += 1
        if record.decode_error:
            self.decode_errors += 1
        if record.frame:
            arb_id = record.frame.arbitration_id
            self.frames_by_id[arb_id] += 1
            self.bytes_by_id[arb_id]  += record.frame.data_length

    @property
    def elapsed_s(self) -> float:
        return (time.time_ns() - self.start_ns) / 1_000_000_000

    @property
    def frames_per_second(self) -> float:
        elapsed = self.elapsed_s
        if elapsed == 0:
            return 0.0
        return self.total_frames / elapsed

    def print_summary(self):
        print(f"\n{'═'*60}")
        print(f"  MONITOR STATISTICS")
        print(f"{'─'*60}")
        print(f"  Elapsed time   : {self.elapsed_s:.2f}s")
        print(f"  Total frames   : {self.total_frames}")
        print(f"  Decode errors  : {self.decode_errors}")
        print(f"  Frames/sec     : {self.frames_per_second:.1f}")
        print(f"{'─'*60}")

        if self.frames_by_id:
            print(f"  Frames by ID:")
            sorted_ids = sorted(self.frames_by_id.keys())
            for arb_id in sorted_ids:
                count     = self.frames_by_id[arb_id]
                total_b   = self.bytes_by_id[arb_id]
                print(
                    f"    0x{arb_id:03X}  "
                    f"{count:>6} frames  "
                    f"{total_b:>8} bytes"
                )
        print(f"{'═'*60}\n")


# ── Decoder ───────────────────────────────────────────────────────────────────

class SlotDecoder:
    """
    Converts a raw BusFrameSlot from shared memory back into a CANFrame.
    """

    def __init__(self):
        self._converter = SignalConverter()
        self._protocols = {
            0: get_protocol("CAN"),
            1: get_protocol("CAN_FD"),
        }

    def decode(self, slot: BusFrameSlot) -> tuple[CANFrame | None, str]:
        """
        Returns (frame, error_string).
        error_string is empty on success.
        """
        protocol = self._protocols.get(slot.protocol)
        if protocol is None:
            return None, f"Unknown protocol code {slot.protocol}"

        try:
            canh = bytes(slot.canh_bytes[:( (slot.bit_count + 7) // 8 )])
            canl = bytes(slot.canl_bytes[:( (slot.bit_count + 7) // 8 )])

            signal = DifferentialSignal(
                canh_bytes = canh,
                canl_bytes = canl,
                bit_count  = slot.bit_count
            )
            frame = protocol.decode(signal)
            return frame, ""

        except Exception as e:
            return None, str(e)


# ── Display ───────────────────────────────────────────────────────────────────

# ANSI colour codes
class Colour:
    RESET   = "\033[0m"
    GREEN   = "\033[32m"
    YELLOW  = "\033[33m"
    RED     = "\033[31m"
    CYAN    = "\033[36m"
    BOLD    = "\033[1m"
    DIM     = "\033[2m"

    @staticmethod
    def supported() -> bool:
        return sys.stdout.isatty()


def format_frame_line(record: FrameRecord, verbose: bool) -> str:
    use_colour = Colour.supported()

    if record.decode_error:
        line = (
            f"[{record.timestamp_str}] "
            f"{'ERROR':<6} "
            f"  {record.decode_error}"
        )
        if use_colour:
            line = Colour.RED + line + Colour.RESET
        return line

    frame   = record.frame
    id_str  = (
        f"0x{frame.arbitration_id:08X}"
        if frame.is_extended
        else f"0x{frame.arbitration_id:03X}"
    )
    data_hex = " ".join(f"{b:02X}" for b in frame.data) if frame.data else "(none)"
    flags    = ""
    if frame.is_extended:
        flags += " [EXT]"
    if frame.is_remote:
        flags += " [RTR]"
    if frame.is_fd:
        flags += " [FD]"
    if frame.brs:
        flags += " [BRS]"

    proto_str = frame.protocol.name

    line = (
        f"[{record.timestamp_str}] "
        f"{proto_str:<6} "
        f"ID={id_str}  "
        f"DLC={frame.dlc}  "
        f"DATA={data_hex}"
        f"{flags}"
    )

    if use_colour:
        id_part = Colour.CYAN + f"ID={id_str}" + Colour.RESET
        line    = (
            f"[{record.timestamp_str}] "
            f"{Colour.GREEN}{proto_str:<6}{Colour.RESET} "
            f"{id_part}  "
            f"DLC={frame.dlc}  "
            f"DATA={data_hex}"
            f"{Colour.YELLOW}{flags}{Colour.RESET}"
        )

    if verbose and frame.data:
        bits_preview = bin(frame.data[0])[2:].zfill(8)
        line += f"\n         first byte: 0b{bits_preview} = {frame.data[0]}"

    return line


# ── Monitor ───────────────────────────────────────────────────────────────────

class BusMonitor:
    """
    Main monitor loop.
    Polls shared memory and prints decoded frames.
    """

    POLL_INTERVAL_S = 0.0001   # 100 μs poll interval

    def __init__(
        self,
        id_filter   : list[int] | None = None,
        max_count   : int | None       = None,
        verbose     : bool             = False,
        show_stats  : bool             = False,
    ):
        self._filter     = set(id_filter) if id_filter else None
        self._max_count  = max_count
        self._verbose    = verbose
        self._show_stats = show_stats
        self._running    = True
        self._stats      = MonitorStats()
        self._decoder    = SlotDecoder()
        self._count      = 0

        # Handle Ctrl+C gracefully
        signal.signal(signal.SIGINT, self._on_sigint)

    def _on_sigint(self, *_):
        self._running = False

    def _wait_for_shm(self, timeout_s: float = 30.0) -> SHMBusReader | None:
        """
        Poll until shared memory appears or timeout is reached.
        This lets you start the monitor before the injector.
        """
        deadline = time.time() + timeout_s
        attempt  = 0

        while time.time() < deadline and self._running:
            try:
                reader = SHMBusReader()
                if attempt > 0:
                    print(f"\nConnected to bus.\n")
                return reader
            except RuntimeError:
                if attempt == 0:
                    print(
                        f"Shared memory not found. "
                        f"Waiting for broker to start "
                        f"(timeout {timeout_s:.0f}s)...",
                        end="",
                        flush=True
                    )
                else:
                    print(".", end="", flush=True)
                attempt += 1
                time.sleep(0.5)

        print(f"\nTimeout after {timeout_s}s. No broker appeared.")
        return None

    def run(self):
        print(f"\n{Colour.BOLD}Virtual CAN Bus Monitor{Colour.RESET}")
        print(f"{'─'*60}")
        if self._filter:
            ids = ", ".join(f"0x{i:03X}" for i in sorted(self._filter))
            print(f"Filter: {ids}")
        if self._max_count:
            print(f"Stopping after {self._max_count} frames")
        print(f"{'─'*60}")
        print(f"Waiting for frames... (Ctrl+C to stop)\n")

        reader = self._wait_for_shm()
        if reader is None:
            return

        with reader:
            while self._running:
                slot = reader.read()

                if slot is None:
                    time.sleep(self.POLL_INTERVAL_S)
                    continue

                self._process_slot(slot)

                if self._max_count and self._count >= self._max_count:
                    self._running = False

        if self._show_stats:
            self._stats.print_summary()

    def _process_slot(self, slot: BusFrameSlot):
        received_ns     = time.time_ns()
        frame, err      = self._decoder.decode(slot)

        record = FrameRecord(
            slot         = slot,
            frame        = frame,
            received_ns  = received_ns,
            decode_error = err,
        )

        # Apply ID filter
        if self._filter and frame:
            if frame.arbitration_id not in self._filter:
                return

        self._stats.record(record)
        self._count += 1

        line = format_frame_line(record, self._verbose)
        print(line)


# ── CLI ───────────────────────────────────────────────────────────────────────

def parse_args():
    parser = argparse.ArgumentParser(
        description="Monitor the virtual CAN bus in real time"
    )
    parser.add_argument(
        "--filter", "-f",
        type=lambda x: int(x, 16),
        nargs="+",
        metavar="ID",
        help="Only show frames with these IDs (hex, e.g. --filter 0x123 0x456)"
    )
    parser.add_argument(
        "--count", "-c",
        type=int,
        default=None,
        metavar="N",
        help="Stop after receiving N frames"
    )
    parser.add_argument(
        "--verbose", "-v",
        action="store_true",
        help="Show extra detail for each frame"
    )
    parser.add_argument(
        "--stats", "-s",
        action="store_true",
        help="Print statistics summary on exit"
    )
    return parser.parse_args()


def main():
    args    = parse_args()
    monitor = BusMonitor(
        id_filter  = args.filter,
        max_count  = args.count,
        verbose    = args.verbose,
        show_stats = args.stats,
    )
    monitor.run()


if __name__ == "__main__":
    main()
