#!/usr/bin/env python3
# tools/launch_dashboard.py

"""
CAN Network Dashboard
=====================
Launches the full CAN network as a tiled dashboard of terminal windows.
Each ECU gets its own gnome-terminal, positioned in a grid layout.
Optionally launches the CARLA client too.

Layout (2x3 grid):
    ┌──────────────────┬──────────────────┐
    │   Engine ECU     │   Chassis ECU    │
    ├──────────────────┼──────────────────┤
    │   Body ECU       │   Gateway ECU    │
    ├──────────────────┼──────────────────┤
    │   Bus Monitor    │   CARLA Client   │
    └──────────────────┴──────────────────┘

Usage:
    python tools/launch_dashboard.py
    python tools/launch_dashboard.py --with-carla
    python tools/launch_dashboard.py --with-carla --vehicle vehicle.audi.a2
"""

import sys
import os
import signal
import argparse
import subprocess
import time
import re
from pathlib import Path

import yaml

PROJ_ROOT = Path(__file__).parent.parent
sys.path.insert(0, str(PROJ_ROOT))

# ── Colours ───────────────────────────────────────────────────────────────────

COLOURS = {
    "engine_ecu":  {"ansi": "\033[1;32m", "label": "🔧 ENGINE ECU"},
    "chassis_ecu": {"ansi": "\033[1;33m", "label": "🔧 CHASSIS ECU"},
    "body_ecu":    {"ansi": "\033[1;34m", "label": "🔧 BODY ECU"},
    "gateway_ecu": {"ansi": "\033[1;35m", "label": "🌐 GATEWAY ECU"},
    "monitor":     {"ansi": "\033[1;36m", "label": "📡 BUS MONITOR"},
    "carla":       {"ansi": "\033[1;31m", "label": "🚗 CARLA CLIENT"},
}
RESET = "\033[0m"


# ── Screen geometry ───────────────────────────────────────────────────────────

def get_screen_size() -> tuple[int, int]:
    """Get screen dimensions from xdpyinfo or fallback."""
    try:
        out = subprocess.check_output(
            ["xdpyinfo"], stderr=subprocess.DEVNULL, text=True
        )
        m = re.search(r"dimensions:\s+(\d+)x(\d+)", out)
        if m:
            return int(m.group(1)), int(m.group(2))
    except Exception:
        pass
    return 1920, 1080


def compute_grid(screen_w, screen_h, rows=3, cols=2, margin=0, taskbar_h=40):
    """Compute window positions for a rows×cols grid."""
    usable_h = screen_h - taskbar_h
    cell_w = (screen_w - margin * (cols + 1)) // cols
    cell_h = (usable_h - margin * (rows + 1)) // rows

    positions = []
    for r in range(rows):
        for c in range(cols):
            x = margin + c * (cell_w + margin)
            y = margin + r * (cell_h + margin)
            positions.append((x, y, cell_w, cell_h))
    return positions


# ── Window management ─────────────────────────────────────────────────────────

def find_window_by_title(title: str, timeout=5.0) -> str | None:
    """Find a window ID by its title using wmctrl."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        try:
            out = subprocess.check_output(
                ["wmctrl", "-l"], text=True, stderr=subprocess.DEVNULL
            )
            for line in out.strip().split("\n"):
                if title in line:
                    return line.split()[0]
        except Exception:
            pass
        time.sleep(0.2)
    return None


def move_window(wid: str, x: int, y: int, w: int, h: int):
    """Move and resize a window using wmctrl."""
    try:
        # Remove maximised state first
        subprocess.run(
            ["wmctrl", "-i", "-r", wid, "-b", "remove,maximized_vert,maximized_horz"],
            check=False, capture_output=True
        )
        time.sleep(0.05)
        # Move and resize: gravity,x,y,w,h
        subprocess.run(
            ["wmctrl", "-i", "-r", wid, "-e", f"0,{x},{y},{w},{h}"],
            check=True, capture_output=True
        )
    except Exception as e:
        print(f"  [!] wmctrl error: {e}")


# ── Terminal launcher ─────────────────────────────────────────────────────────

class DashboardTerminal:
    """A single terminal window in the dashboard."""

    def __init__(self, name: str, title: str, command: str, cwd: str):
        self.name    = name
        self.title   = title
        self.command = command
        self.cwd     = cwd
        self.proc    = None
        self.wid     = None

    def launch(self):
        """Open a gnome-terminal with the given command."""
        # Use a unique title we can search for
        colour = COLOURS.get(self.name, {})
        label = colour.get("label", self.name)
        ansi  = colour.get("ansi", "")

        # Build a bash wrapper that:
        # 1. Prints a coloured banner
        # 2. Runs the actual command
        # 3. Waits for keypress if it exits (so terminal doesn't vanish)
        banner = (
            f'echo -e "{ansi}╔══════════════════════════════════════╗{RESET}"; '
            f'echo -e "{ansi}║  {label:^36s}  ║{RESET}"; '
            f'echo -e "{ansi}╚══════════════════════════════════════╝{RESET}"; '
            f'echo "";'
        )
        wrapper = f'{banner} cd {self.cwd} && {self.command}; echo ""; echo "[Process exited. Press Enter to close]"; read'

        self.proc = subprocess.Popen([
            "gnome-terminal",
            "--title", self.title,
            "--", "bash", "-c", wrapper,
        ])

    def position(self, x, y, w, h):
        """Find and position the terminal window."""
        self.wid = find_window_by_title(self.title, timeout=4.0)
        if self.wid:
            move_window(self.wid, x, y, w, h)
        else:
            print(f"  [!] Could not find window for {self.name}")


# ── Dashboard launcher ───────────────────────────────────────────────────────

class Dashboard:
    """Launches and manages the full CAN network dashboard."""

    def __init__(self, args):
        self.args      = args
        self.terminals : list[DashboardTerminal] = []
        self._running  = True

        signal.signal(signal.SIGINT,  self._on_signal)
        signal.signal(signal.SIGTERM, self._on_signal)

    def _on_signal(self, *_):
        self._running = False

    def run(self):
        self._print_banner()

        venv_python = str(PROJ_ROOT / ".venv" / "bin" / "python")
        cwd         = str(PROJ_ROOT)

        # ── Define terminals ──────────────────────────────────────────────

        # ECU runner command
        runner = str(PROJ_ROOT / "tools" / "run_ecu.py")

        config_path = PROJ_ROOT / self.args.config
        with open(config_path) as f:
            raw = yaml.safe_load(f)

        ecus = raw.get("ecus", [])

        for ecu in ecus:
            ecu_id   = ecu["id"]
            firmware = ecu["firmware"]
            title    = f"CAN_DASH_{ecu_id}"
            cmd      = f"{venv_python} {runner} {firmware} --id {ecu_id}"
            self.terminals.append(DashboardTerminal(ecu_id, title, cmd, cwd))

        # Monitor
        monitor_cmd = f"{venv_python} {PROJ_ROOT / 'tools' / 'monitor.py'} --stats"
        self.terminals.append(
            DashboardTerminal("monitor", "CAN_DASH_monitor", monitor_cmd, cwd)
        )

        # CARLA client (optional)
        if self.args.with_carla:
            carla_cmd = f"{venv_python} {PROJ_ROOT / 'tools' / 'carla_client.py'}"
            if self.args.vehicle:
                carla_cmd += f" --vehicle {self.args.vehicle}"
            if self.args.carla_host:
                carla_cmd += f" --host {self.args.carla_host}"
            self.terminals.append(
                DashboardTerminal("carla", "CAN_DASH_carla", carla_cmd, cwd)
            )

        # ── Launch terminals with stagger ─────────────────────────────────

        print(f"  Launching {len(self.terminals)} terminal(s)...\n")

        for i, term in enumerate(self.terminals):
            print(f"  [{i+1}/{len(self.terminals)}] {term.name}")
            term.launch()
            time.sleep(1.0)   # stagger for SHM init

        # ── Position windows ──────────────────────────────────────────────

        print(f"\n  Arranging windows...")
        time.sleep(1.0)   # let terminals finish opening

        screen_w, screen_h = get_screen_size()
        n = len(self.terminals)

        # Choose grid layout
        if n <= 2:
            rows, cols = 1, 2
        elif n <= 4:
            rows, cols = 2, 2
        elif n <= 6:
            rows, cols = 3, 2
        else:
            rows, cols = 3, 3

        positions = compute_grid(screen_w, screen_h, rows, cols, margin=4, taskbar_h=48)

        for i, term in enumerate(self.terminals):
            if i < len(positions):
                x, y, w, h = positions[i]
                term.position(x, y, w, h)

        # ── Dashboard ready ───────────────────────────────────────────────

        print(f"\n  ✅ Dashboard ready — {n} panels")
        print(f"  Press Ctrl+C here to stop everything.\n")
        print(f"{'─'*50}")

        # ── Wait for Ctrl+C ───────────────────────────────────────────────

        while self._running:
            time.sleep(0.5)

        self._shutdown()

    def _shutdown(self):
        print(f"\n{'─'*50}")
        print("  Shutting down dashboard...")

        # Kill all ECU python processes
        try:
            result = subprocess.run(
                ["pgrep", "-f", "run_ecu.py"],
                capture_output=True, text=True, timeout=5
            )
            if result.stdout.strip():
                for pid_str in result.stdout.strip().split("\n"):
                    try:
                        os.kill(int(pid_str.strip()), signal.SIGTERM)
                    except (ProcessLookupError, ValueError):
                        pass
        except Exception:
            pass

        # Kill monitor
        try:
            result = subprocess.run(
                ["pgrep", "-f", "monitor.py"],
                capture_output=True, text=True, timeout=5
            )
            if result.stdout.strip():
                for pid_str in result.stdout.strip().split("\n"):
                    try:
                        os.kill(int(pid_str.strip()), signal.SIGTERM)
                    except (ProcessLookupError, ValueError):
                        pass
        except Exception:
            pass

        # Kill CARLA client
        try:
            result = subprocess.run(
                ["pgrep", "-f", "carla_client.py"],
                capture_output=True, text=True, timeout=5
            )
            if result.stdout.strip():
                for pid_str in result.stdout.strip().split("\n"):
                    try:
                        os.kill(int(pid_str.strip()), signal.SIGTERM)
                    except (ProcessLookupError, ValueError):
                        pass
        except Exception:
            pass

        # Close terminal windows
        for term in self.terminals:
            if term.wid:
                try:
                    subprocess.run(
                        ["wmctrl", "-i", "-c", term.wid],
                        check=False, capture_output=True
                    )
                except Exception:
                    pass

        time.sleep(0.5)
        print("  Dashboard stopped.\n")

    def _print_banner(self):
        print()
        print("  ╔═══════════════════════════════════════════════╗")
        print("  ║        🚗  CAN Network Dashboard  🚗        ║")
        print("  ╠═══════════════════════════════════════════════╣")
        print("  ║  All ECUs + Monitor in tiled terminals       ║")
        print("  ║  Ctrl+C here stops everything                ║")
        print("  ╚═══════════════════════════════════════════════╝")
        print()


# ── CLI ───────────────────────────────────────────────────────────────────────

def parse_args():
    parser = argparse.ArgumentParser(
        description="Launch the CAN network as a tiled terminal dashboard"
    )
    parser.add_argument(
        "--config", "-c",
        type=str, default="network.yaml",
        help="Path to network config YAML (default: network.yaml)"
    )
    parser.add_argument(
        "--with-carla",
        action="store_true",
        help="Also launch the CARLA client"
    )
    parser.add_argument(
        "--vehicle",
        type=str, default="vehicle.tesla.model3",
        help="CARLA vehicle blueprint (default: vehicle.tesla.model3)"
    )
    parser.add_argument(
        "--carla-host",
        type=str, default=None,
        help="CARLA server host (default: 127.0.0.1)"
    )
    return parser.parse_args()


def main():
    args = parse_args()
    dashboard = Dashboard(args)
    dashboard.run()


if __name__ == "__main__":
    main()
