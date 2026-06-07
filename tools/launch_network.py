# tools/launch_network.py

"""
Network Launch Script
=====================
Reads network.yaml and launches all ECU processes simultaneously.
Each ECU runs in its own process with the fake machine module injected.

By default, each ECU gets its own terminal window with colour-coded output.
Use --inline to multiplex all output into the current terminal instead.

Usage:
    python tools/launch_network.py
    python tools/launch_network.py --config network.yaml --monitor
    python tools/launch_network.py --inline
    python tools/launch_network.py --inject tools/examples/engine_inputs.yaml
"""

import sys
import os
import signal
import argparse
import subprocess
import time
import threading
from pathlib import Path
from collections import defaultdict

import yaml

sys.path.insert(0, str(Path(__file__).parent.parent))

from bus_broker.transport.shm_writer import SHMBusReader, BusFrameSlot


# ── Config ────────────────────────────────────────────────────────────────────

class NetworkConfig:
    def __init__(self, path: Path):
        if not path.exists():
            raise FileNotFoundError(f"Config not found: {path}")
        with open(path) as f:
            raw = yaml.safe_load(f)

        bus                  = raw.get("bus", {})
        self.protocol        = bus.get("protocol",  "CAN")
        # Support both old (bit_rate) and new (nominal_bit_rate) YAML keys
        self.nominal_bit_rate = bus.get("nominal_bit_rate", bus.get("bit_rate", 500_000))
        self.data_bit_rate    = bus.get("data_bit_rate", self.nominal_bit_rate)
        self.bit_rate         = self.nominal_bit_rate   # backward compat
        self.ecus             = [self._parse_ecu(e) for e in raw.get("ecus", [])]

    def _parse_ecu(self, entry: dict) -> dict:
        return {
            "id"       : entry["id"],
            "firmware" : Path(entry["firmware"]),
            "filters"  : entry.get("filters", []),
        }


# ── Per-ECU terminal colours ─────────────────────────────────────────────────

# ANSI colour definitions for each ECU
ECU_COLOUR_DEFS = [
    {"name": "green",   "ansi": "\033[32m", "ansi_bold": "\033[1;32m", "hex": "#4EC920"},
    {"name": "yellow",  "ansi": "\033[33m", "ansi_bold": "\033[1;33m", "hex": "#E5C07B"},
    {"name": "blue",    "ansi": "\033[34m", "ansi_bold": "\033[1;34m", "hex": "#61AFEF"},
    {"name": "magenta", "ansi": "\033[35m", "ansi_bold": "\033[1;35m", "hex": "#C678DD"},
    {"name": "cyan",    "ansi": "\033[36m", "ansi_bold": "\033[1;36m", "hex": "#56B6C2"},
]
RESET = "\033[0m"


# ── Process manager ───────────────────────────────────────────────────────────

class ECUProcess:
    """Manages a single ECU subprocess."""

    def __init__(self, ecu_id: str, firmware: Path, verbose: bool = False):
        self.ecu_id   = ecu_id
        self.firmware = firmware
        self.verbose  = verbose
        self.process  = None

    def _build_cmd(self) -> list[str]:
        runner = Path(__file__).parent / "run_ecu.py"
        cmd = [
            sys.executable,
            str(runner),
            str(self.firmware),
            "--id", self.ecu_id,
        ]
        if self.verbose:
            cmd.append("--verbose")
        return cmd

    def start_inline(self):
        """Start with piped stdout (for --inline mode)."""
        print(f"  [+] Starting {self.ecu_id} ({self.firmware.name})")
        self.process = subprocess.Popen(
            self._build_cmd(),
            stdout = subprocess.PIPE,
            stderr = subprocess.STDOUT,
            text   = True,
            bufsize= 1,
        )

    def start_in_terminal(self, colour_def: dict):
        """Start in a separate gnome-terminal window with colour title."""
        cmd = self._build_cmd()
        ecu_cmd_str = " ".join(cmd)

        # Colour the output: set terminal foreground colour via ANSI codes
        # We wrap the ECU command so its output is coloured
        ansi_bold = colour_def["ansi_bold"]
        ansi      = colour_def["ansi"]
        wrapper_script = (
            f'echo -e "{ansi_bold}╔══════════════════════════════════════╗{RESET}"; '
            f'echo -e "{ansi_bold}║  {self.ecu_id.upper():^36s}  ║{RESET}"; '
            f'echo -e "{ansi_bold}╚══════════════════════════════════════╝{RESET}"; '
            f'echo ""; '
            f'exec {ecu_cmd_str}'
        )

        title = f"🔧 {self.ecu_id.upper()}"

        print(f"  [+] Starting {self.ecu_id} ({self.firmware.name}) in terminal")
        self.process = subprocess.Popen([
            "gnome-terminal",
            "--title", title,
            "--geometry", "80x25",
            "--",
            "bash", "-c", wrapper_script,
        ])
        # gnome-terminal returns immediately (it forks), so self.process
        # is the gnome-terminal launcher, not the ECU process itself.

    def stop(self):
        if self.process and self.process.poll() is None:
            self.process.terminate()
            try:
                self.process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                self.process.kill()
            print(f"  [-] Stopped {self.ecu_id}")
        self.process = None

    def is_running(self) -> bool:
        return self.process is not None and self.process.poll() is None

    def read_output(self) -> str | None:
        """Non-blocking read of one line from the ECU process (inline mode only)."""
        if not self.is_running():
            return None
        try:
            self.process.stdout.fileno()
            import select
            ready, _, _ = select.select(
                [self.process.stdout], [], [], 0
            )
            if ready:
                line = self.process.stdout.readline()
                return line.rstrip() if line else None
        except Exception:
            pass
        return None


# ── Output multiplexer (inline mode) ─────────────────────────────────────────

class OutputMux:
    """
    Reads stdout from all ECU processes and prints it
    with colour-coded prefixes so you can tell which
    ECU is printing what.
    """

    def __init__(self, ecus: list[ECUProcess]):
        self._ecus    = ecus
        self._colours = {}
        for i, ecu in enumerate(ecus):
            self._colours[ecu.ecu_id] = ECU_COLOUR_DEFS[i % len(ECU_COLOUR_DEFS)]["ansi"]

    def poll(self):
        for ecu in self._ecus:
            line = ecu.read_output()
            if line:
                colour = self._colours.get(ecu.ecu_id, "")
                print(f"{colour}[{ecu.ecu_id}] {line}{RESET}")


# ── Bus statistics collector ──────────────────────────────────────────────────

class BusStatsCollector:
    """
    Background thread that reads every frame from the shared memory bus
    and accumulates per-ID statistics. Prints a summary on stop().
    """

    def __init__(self):
        self._running      = False
        self._thread       = None
        self._reader       = None
        self._start_ns     = 0
        self._stop_ns      = 0
        self._total_frames = 0
        self._frames_by_id : dict[int, int] = defaultdict(int)
        self._bytes_by_id  : dict[int, int] = defaultdict(int)
        self._first_frame_ns = 0
        self._last_frame_ns  = 0

    def start(self):
        """Open an SHMBusReader and begin collecting in the background."""
        try:
            self._reader = SHMBusReader()
        except RuntimeError:
            print("  [stats] Could not open bus — stats disabled.")
            return

        self._running  = True
        self._start_ns = time.time_ns()
        self._thread   = threading.Thread(
            target=self._poll_loop, daemon=True, name="BusStats"
        )
        self._thread.start()

    def stop(self):
        self._running = False
        self._stop_ns = time.time_ns()
        if self._thread:
            self._thread.join(timeout=2.0)
        if self._reader:
            self._reader.close()
            self._reader = None

    def _poll_loop(self):
        reader = self._reader
        while self._running:
            slot = reader.read()
            if slot is None:
                time.sleep(0.0001)   # 100 μs
                continue
            self._total_frames += 1
            arb_id = slot.arbitration_id
            self._frames_by_id[arb_id] += 1
            # Estimate data bytes from bit_count
            data_bytes = (slot.bit_count + 7) // 8
            self._bytes_by_id[arb_id] += data_bytes
            ts = slot.timestamp_ns
            if self._first_frame_ns == 0:
                self._first_frame_ns = ts
            self._last_frame_ns = ts

    def print_summary(self):
        """Print a formatted bus statistics table."""
        if self._total_frames == 0:
            print("\n  No frames were captured — nothing to report.")
            return

        elapsed_s = (self._stop_ns - self._start_ns) / 1_000_000_000
        if elapsed_s <= 0:
            elapsed_s = 0.001

        fps       = self._total_frames / elapsed_s
        unique    = len(self._frames_by_id)
        total_b   = sum(self._bytes_by_id.values())

        # Bus load estimate (at configured bit rate, rough)
        # Assume ~100 bits overhead per frame on average
        total_bits = sum(
            count * ((self._bytes_by_id[aid] // count) * 8 + 47)
            for aid, count in self._frames_by_id.items()
            if count > 0
        )
        # Default 500 kbit/s
        bus_load_pct = (total_bits / elapsed_s) / 500_000 * 100

        print(f"\n{'═'*70}")
        print(f"  📊  BUS STATISTICS")
        print(f"{'─'*70}")
        print(f"  Duration         : {elapsed_s:.2f} s")
        print(f"  Total frames     : {self._total_frames:,}")
        print(f"  Unique CAN IDs   : {unique}")
        print(f"  Total data       : {total_b:,} bytes")
        print(f"  Throughput       : {fps:,.1f} frames/sec")
        print(f"  Est. bus load    : {bus_load_pct:.1f}%")
        print(f"{'─'*70}")
        print(f"  {'CAN ID':<10} {'Frames':>10} {'Bytes':>10} {'Frames/s':>12} {'%Total':>8}")
        print(f"  {'─'*8:<10} {'─'*10:>10} {'─'*10:>10} {'─'*12:>12} {'─'*8:>8}")

        sorted_ids = sorted(self._frames_by_id.keys())
        for arb_id in sorted_ids:
            count    = self._frames_by_id[arb_id]
            byte_cnt = self._bytes_by_id[arb_id]
            id_fps   = count / elapsed_s
            pct      = (count / self._total_frames) * 100
            print(
                f"  0x{arb_id:03X}       "
                f"{count:>10,}  "
                f"{byte_cnt:>9,}  "
                f"{id_fps:>11,.1f}  "
                f"{pct:>7.1f}%"
            )

        print(f"{'═'*70}\n")


# ── Launcher ──────────────────────────────────────────────────────────────────

class NetworkLauncher:
    def __init__(
        self,
        config        : NetworkConfig,
        open_monitor  : bool  = False,
        inject_file   : Path | None = None,
        verbose_ecus  : bool  = False,
        inline        : bool  = False,
    ):
        self.config        = config
        self.open_monitor  = open_monitor
        self.inject_file   = inject_file
        self.verbose_ecus  = verbose_ecus
        self.inline        = inline
        self._ecus         : list[ECUProcess] = []
        self._ecu_pids     : list[int] = []       # PIDs of actual ECU processes
        self._monitor_proc = None
        self._inject_proc  = None
        self._stats        = BusStatsCollector()
        self._running      = True

        signal.signal(signal.SIGINT,  self._on_signal)
        signal.signal(signal.SIGTERM, self._on_signal)

    def _on_signal(self, *_):
        self._running = False

    def launch(self):
        self._print_header()

        if self.inline:
            self._launch_inline()
        else:
            self._launch_terminals()

        # Start monitor if requested
        if self.open_monitor:
            self._start_monitor()

        # Run injector if requested
        if self.inject_file:
            self._start_injector()

        # Start bus stats collector
        self._stats.start()

        print(f"  Press Ctrl+C to stop all ECUs.\n")
        print(f"{'─'*60}\n")

        if self.inline:
            self._run_inline_loop()
        else:
            self._run_terminal_loop()

        self._shutdown()

    def _launch_inline(self):
        """Launch ECUs with piped output (old behaviour)."""
        for ecu_cfg in self.config.ecus:
            ecu = ECUProcess(
                ecu_id   = ecu_cfg["id"],
                firmware = ecu_cfg["firmware"],
                verbose  = self.verbose_ecus,
            )
            ecu.start_inline()
            self._ecus.append(ecu)
            time.sleep(1.0)

        print(f"\n  {len(self._ecus)} ECU(s) started.")
        print(f"  Waiting for all ECUs to connect to bus...")
        time.sleep(2.0)
        print(f"  Network ready.\n")

    def _launch_terminals(self):
        """Launch each ECU in its own terminal window."""
        for i, ecu_cfg in enumerate(self.config.ecus):
            colour_def = ECU_COLOUR_DEFS[i % len(ECU_COLOUR_DEFS)]
            ecu = ECUProcess(
                ecu_id   = ecu_cfg["id"],
                firmware = ecu_cfg["firmware"],
                verbose  = self.verbose_ecus,
            )
            ecu.start_in_terminal(colour_def)
            self._ecus.append(ecu)
            time.sleep(1.0)

        print(f"\n  {len(self._ecus)} ECU(s) started in separate terminals.")
        print(f"  Waiting for all ECUs to connect to bus...")
        time.sleep(2.0)

        # Find the actual ECU python processes (children of gnome-terminal)
        self._discover_ecu_pids()
        print(f"  Network ready.\n")

    def _discover_ecu_pids(self):
        """Find PIDs of the actual run_ecu.py processes."""
        try:
            result = subprocess.run(
                ["pgrep", "-f", "run_ecu.py"],
                capture_output=True, text=True, timeout=5
            )
            if result.stdout.strip():
                self._ecu_pids = [
                    int(pid) for pid in result.stdout.strip().split("\n")
                    if pid.strip()
                ]
                print(f"  Found {len(self._ecu_pids)} ECU process(es).")
        except Exception:
            pass

    def _start_monitor(self):
        if self.inline:
            monitor = Path(__file__).parent / "monitor.py"
            self._monitor_proc = subprocess.Popen(
                [sys.executable, str(monitor), "--stats"],
                text=True,
            )
            print("  Bus monitor started.\n")
        else:
            # Open monitor in its own terminal too
            monitor = Path(__file__).parent / "monitor.py"
            monitor_cmd = f"{sys.executable} {monitor} --stats"
            self._monitor_proc = subprocess.Popen([
                "gnome-terminal",
                "--title", "📡 CAN Bus Monitor",
                "--geometry", "120x30",
                "--",
                "bash", "-c", monitor_cmd,
            ])
            print("  Bus monitor started in terminal.\n")
        time.sleep(0.5)

    def _start_injector(self):
        injector = Path(__file__).parent / "injector.py"
        print(f"  Waiting 3s for ECUs to fully initialise...")
        time.sleep(3.0)
        if self.inline:
            self._inject_proc = subprocess.Popen(
                [sys.executable, str(injector), str(self.inject_file), "--verbose"],
                text=True,
            )
        else:
            inject_cmd = f"{sys.executable} {injector} {self.inject_file} --verbose"
            self._inject_proc = subprocess.Popen([
                "gnome-terminal",
                "--title", "💉 Frame Injector",
                "--geometry", "100x20",
                "--",
                "bash", "-c", inject_cmd,
            ])
        print(f"  Injecting: {self.inject_file.name}\n")

    def _run_inline_loop(self):
        mux = OutputMux(self._ecus)
        while self._running:
            mux.poll()
            for ecu in self._ecus:
                if not ecu.is_running():
                    print(f"\n  WARNING: {ecu.ecu_id} exited unexpectedly.")
            if self._inject_proc:
                if self._inject_proc.poll() is not None:
                    self._inject_proc = None
            time.sleep(0.01)

    def _run_terminal_loop(self):
        """Wait for Ctrl+C. ECUs are in their own terminals."""
        while self._running:
            time.sleep(0.5)

    def _shutdown(self):
        print(f"\n{'─'*60}")
        print("  Shutting down network...")

        # Stop stats collector first (while bus is still alive)
        self._stats.stop()

        # In terminal mode, kill the actual ECU processes
        for pid in self._ecu_pids:
            try:
                os.kill(pid, signal.SIGTERM)
            except ProcessLookupError:
                pass

        # Also stop any inline ECU processes
        for ecu in self._ecus:
            ecu.stop()

        if self._monitor_proc:
            # Kill monitor processes
            try:
                result = subprocess.run(
                    ["pgrep", "-f", "monitor.py"],
                    capture_output=True, text=True, timeout=5
                )
                if result.stdout.strip():
                    for pid_str in result.stdout.strip().split("\n"):
                        try:
                            os.kill(int(pid_str), signal.SIGTERM)
                        except (ProcessLookupError, ValueError):
                            pass
            except Exception:
                pass
            self._monitor_proc.terminate()
            print("  Monitor stopped.")

        if self._inject_proc and self._inject_proc.poll() is None:
            self._inject_proc.terminate()

        print("  Network stopped.")

        # Print bus statistics summary
        self._stats.print_summary()

    def _print_header(self):
        print(f"\n{'═'*60}")
        print(f"  Virtual CAN Network")
        print(f"{'─'*60}")
        print(f"  Protocol  : {self.config.protocol}")
        if self.config.protocol == "CAN_FD":
            print(f"  Nominal   : {self.config.nominal_bit_rate:,} bit/s")
            print(f"  Data rate : {self.config.data_bit_rate:,} bit/s")
        else:
            print(f"  Bit rate  : {self.config.bit_rate:,} bit/s")
        print(f"  ECUs      : {len(self.config.ecus)}")
        mode = "inline (multiplexed)" if self.inline else "separate terminals"
        print(f"  Mode      : {mode}")
        print(f"{'─'*60}")
        for i, ecu in enumerate(self.config.ecus):
            flt = ", ".join(f"0x{f:03X}" for f in ecu["filters"])
            colour = ECU_COLOUR_DEFS[i % len(ECU_COLOUR_DEFS)]
            print(f"  {colour['ansi']}{ecu['id']:<20}{RESET} {ecu['firmware'].name}")
            if flt:
                print(f"  {colour['ansi']}  filters: {flt}{RESET}")
        print(f"{'═'*60}\n")


# ── CLI ───────────────────────────────────────────────────────────────────────

def parse_args():
    parser = argparse.ArgumentParser(
        description="Launch the virtual ECU network"
    )
    parser.add_argument(
        "--config", "-c",
        type    = Path,
        default = Path("network.yaml"),
        help    = "Path to network config YAML (default: network.yaml)"
    )
    parser.add_argument(
        "--monitor", "-m",
        action = "store_true",
        help   = "Also open the bus monitor"
    )
    parser.add_argument(
        "--inject", "-i",
        type    = Path,
        default = None,
        metavar = "FILE",
        help    = "Inject a YAML frame file after ECUs start"
    )
    parser.add_argument(
        "--verbose", "-v",
        action = "store_true",
        help   = "Show verbose ECU output (Pin/Timer setup messages)"
    )
    parser.add_argument(
        "--inline",
        action = "store_true",
        help   = "Multiplex all ECU output in the current terminal "
                 "(instead of opening separate windows)"
    )
    return parser.parse_args()


def main():
    args     = parse_args()
    config   = NetworkConfig(args.config)
    launcher = NetworkLauncher(
        config       = config,
        open_monitor = args.monitor,
        inject_file  = args.inject,
        verbose_ecus = args.verbose,
        inline       = args.inline,
    )
    launcher.launch()


if __name__ == "__main__":
    main()