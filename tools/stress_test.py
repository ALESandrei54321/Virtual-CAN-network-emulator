#!/usr/bin/env python3
# tools/stress_test.py

"""
CAN Bus Stress Test
===================
Measures the maximum throughput and latency of the shared memory
CAN bus transport layer.

Modes:
  write-only  — N threads write as fast as possible
  read-only   — 1 writer + N readers measure read throughput
  full        — N writers + M readers simultaneously (default)

Usage:
    python tools/stress_test.py
    python tools/stress_test.py --duration 10
    python tools/stress_test.py --writers 4 --readers 4
    python tools/stress_test.py --mode write-only --writers 8
    python tools/stress_test.py --frame-size 64
"""

import sys
import os
import time
import threading
import argparse
import statistics
from pathlib import Path
from collections import defaultdict

sys.path.insert(0, str(Path(__file__).parent.parent))

from bus_broker.transport.shm_writer import (
    SHMBusWriter, SHMBusReader, BusFrameSlot, make_slot, MAX_SIGNAL_BYTES
)
from bus_broker.core.signal import SignalConverter, DifferentialSignal
from bus_broker.core.frames import CANFrame, Protocol, CAN_FD_DLC_MAP
from bus_broker.protocols.registry import get_protocol

# Reverse map: byte count → DLC code for CAN FD
_BYTES_TO_DLC = {v: k for k, v in CAN_FD_DLC_MAP.items()}


# ── Helpers ───────────────────────────────────────────────────────────────────

def make_test_frame(arb_id: int, data_bytes: int, fd: bool = False) -> tuple:
    """Create a pre-encoded slot for fast writing."""
    data = bytes([arb_id & 0xFF] * data_bytes)

    if fd:
        # Map data_bytes to proper CAN FD DLC code
        dlc = _BYTES_TO_DLC.get(data_bytes, data_bytes)
        frame = CANFrame(
            arbitration_id=arb_id,
            dlc=dlc,
            data=data,
            protocol=Protocol.CAN_FD,
            brs=True,
        )
        protocol = get_protocol("CAN_FD")
        signal, brs_index = protocol.encode_with_brs(frame)
        slot = make_slot(signal, frame, brs_index=brs_index)
    else:
        frame = CANFrame(
            arbitration_id=arb_id,
            dlc=data_bytes,
            data=data,
            protocol=Protocol.CAN,
        )
        protocol = get_protocol("CAN")
        signal = protocol.encode(frame)
        slot = make_slot(signal, frame)
    return slot


# ── Writer thread ─────────────────────────────────────────────────────────────

class WriterThread:
    def __init__(self, thread_id: int, arb_id_base: int, data_bytes: int, fd: bool = False):
        self.thread_id  = thread_id
        self.data_bytes = data_bytes
        self.arb_id     = arb_id_base + thread_id
        self.fd         = fd
        self.running    = False
        self.count      = 0
        self.errors     = 0
        self.latencies  = []  # ns per write
        self._thread    = None

    def start(self):
        self.running = True
        self._thread = threading.Thread(
            target=self._run, daemon=True,
            name=f"Writer-{self.thread_id}"
        )
        self._thread.start()

    def stop(self):
        self.running = False
        if self._thread:
            self._thread.join(timeout=3.0)

    def _run(self):
        writer = SHMBusWriter()
        slot = make_test_frame(self.arb_id, self.data_bytes, fd=self.fd)
        sample_every = 100  # sample latency every N writes

        try:
            while self.running:
                do_sample = (self.count % sample_every == 0)

                if do_sample:
                    t0 = time.perf_counter_ns()

                ok = writer.write(slot)

                if do_sample:
                    dt = time.perf_counter_ns() - t0
                    self.latencies.append(dt)

                if ok:
                    self.count += 1
                else:
                    self.errors += 1
        finally:
            writer.close()


# ── Reader thread ─────────────────────────────────────────────────────────────

class ReaderThread:
    def __init__(self, thread_id: int):
        self.thread_id = thread_id
        self.running   = False
        self.count     = 0
        self.latencies = []  # ns per read
        self.ids_seen  = defaultdict(int)
        self._thread   = None

    def start(self):
        self.running = True
        self._thread = threading.Thread(
            target=self._run, daemon=True,
            name=f"Reader-{self.thread_id}"
        )
        self._thread.start()

    def stop(self):
        self.running = False
        if self._thread:
            self._thread.join(timeout=3.0)

    def _run(self):
        reader = SHMBusReader()
        sample_every = 100

        try:
            while self.running:
                do_sample = (self.count % sample_every == 0)

                if do_sample:
                    t0 = time.perf_counter_ns()

                slot = reader.read()

                if slot is None:
                    # No data — tight spin but yield
                    time.sleep(0.000001)  # 1μs
                    continue

                if do_sample:
                    dt = time.perf_counter_ns() - t0
                    self.latencies.append(dt)

                self.count += 1
                self.ids_seen[slot.arbitration_id] += 1
        finally:
            reader.close()


# ── Stress Test ───────────────────────────────────────────────────────────────

class StressTest:
    def __init__(self, args):
        self.args    = args
        self.writers = []
        self.readers = []

    def run(self):
        self._print_header()

        mode       = self.args.mode
        n_writers  = self.args.writers
        n_readers  = self.args.readers
        duration   = self.args.duration
        data_bytes = self.args.frame_size

        if mode == "write-only":
            n_readers = 0
        elif mode == "read-only":
            n_writers = 1  # need at least 1 writer to produce data

        fd = self.args.fd
        proto_name = "CAN FD" if fd else "CAN"

        # Create threads
        for i in range(n_writers):
            self.writers.append(WriterThread(i, 0x100, data_bytes, fd=fd))
        for i in range(n_readers):
            self.readers.append(ReaderThread(i))

        # Start
        print(f"  Starting {n_writers} writer(s) + {n_readers} reader(s)...")
        print(f"  Protocol: {proto_name}")
        print(f"  Frame payload: {data_bytes} bytes")
        print(f"  Duration: {duration}s\n")

        start_time = time.time()

        for w in self.writers:
            w.start()
        time.sleep(0.05)  # let writers fill some data
        for r in self.readers:
            r.start()

        # Progress bar
        try:
            while time.time() - start_time < duration:
                elapsed = time.time() - start_time
                pct = elapsed / duration
                bar_len = 40
                filled = int(bar_len * pct)
                bar = "█" * filled + "░" * (bar_len - filled)

                total_w = sum(w.count for w in self.writers)
                total_r = sum(r.count for r in self.readers)
                w_rate = total_w / max(elapsed, 0.001)
                r_rate = total_r / max(elapsed, 0.001)

                sys.stdout.write(
                    f"\r  [{bar}] {pct*100:5.1f}%  "
                    f"W:{w_rate:,.0f}/s  R:{r_rate:,.0f}/s"
                )
                sys.stdout.flush()
                time.sleep(0.25)
        except KeyboardInterrupt:
            pass

        # Stop all threads
        for w in self.writers:
            w.stop()
        for r in self.readers:
            r.stop()

        end_time = time.time()
        actual_duration = end_time - start_time

        print(f"\r  {'─'*60}")
        self._print_results(actual_duration)

    def _print_results(self, duration):
        # ── Write stats ───────────────────────────────────────────────
        total_written = sum(w.count for w in self.writers)
        total_w_errors = sum(w.errors for w in self.writers)
        all_w_latencies = []
        for w in self.writers:
            all_w_latencies.extend(w.latencies)

        w_fps = total_written / duration if duration > 0 else 0

        print(f"\n{'═'*62}")
        print(f"  📊  STRESS TEST RESULTS")
        print(f"{'─'*62}")
        print(f"  Duration       : {duration:.2f}s")
        print(f"  Protocol       : {'CAN FD' if self.args.fd else 'CAN'}")
        print(f"  Writers        : {len(self.writers)}")
        print(f"  Readers        : {len(self.readers)}")
        print(f"  Frame payload  : {self.args.frame_size} bytes")

        print(f"\n{'─'*62}")
        print(f"  ✏️  WRITE PERFORMANCE")
        print(f"{'─'*62}")
        print(f"  Total written  : {total_written:>12,} frames")
        print(f"  Write errors   : {total_w_errors:>12,}")
        print(f"  Throughput     : {w_fps:>12,.1f} frames/sec")

        if all_w_latencies:
            self._print_latency_stats("  Write latency", all_w_latencies)

        # Per-writer breakdown
        if len(self.writers) > 1:
            print(f"\n  Per-writer breakdown:")
            for w in self.writers:
                wfps = w.count / duration
                print(f"    Writer {w.thread_id} (0x{w.arb_id:03X}): "
                      f"{w.count:>10,} frames  ({wfps:>10,.1f}/s)")

        # ── Read stats ────────────────────────────────────────────────
        if self.readers:
            total_read = sum(r.count for r in self.readers)
            r_fps = total_read / duration if duration > 0 else 0
            all_r_latencies = []
            for r in self.readers:
                all_r_latencies.extend(r.latencies)

            print(f"\n{'─'*62}")
            print(f"  📖  READ PERFORMANCE")
            print(f"{'─'*62}")
            print(f"  Total read     : {total_read:>12,} frames")
            print(f"  Throughput     : {r_fps:>12,.1f} frames/sec")

            if all_r_latencies:
                self._print_latency_stats("  Read latency", all_r_latencies)

            # Per-reader breakdown
            if len(self.readers) > 1:
                print(f"\n  Per-reader breakdown:")
                for r in self.readers:
                    rfps = r.count / duration
                    unique = len(r.ids_seen)
                    print(f"    Reader {r.thread_id}: "
                          f"{r.count:>10,} frames  ({rfps:>10,.1f}/s)  "
                          f"{unique} unique IDs")

        # ── Capacity estimate ─────────────────────────────────────────
        print(f"\n{'─'*62}")
        print(f"  🔧  BUS CAPACITY")
        print(f"{'─'*62}")

        # Estimate equivalent bit rate
        avg_frame_bits = (self.args.frame_size * 8) + 47  # data + overhead
        bit_rate = w_fps * avg_frame_bits
        nominal_rate = 500_000
        data_rate    = 2_000_000 if self.args.fd else nominal_rate
        can_load     = (bit_rate / data_rate) * 100

        print(f"  Peak write rate  : {w_fps:>12,.0f} frames/sec")
        print(f"  Equiv. bit rate  : {bit_rate/1_000_000:>12.2f} Mbit/s")
        rate_label = f"CAN FD {data_rate//1_000_000}Mbit/s" if self.args.fd else "CAN 500kbit/s"
        print(f"  vs {rate_label:14s}: {can_load:>10.0f}x capacity")
        print(f"  Ring buffer size : 4,096 slots")

        if total_w_errors > 0:
            loss_pct = total_w_errors / (total_written + total_w_errors) * 100
            print(f"  Frame loss       : {loss_pct:.2f}%")
        else:
            print(f"  Frame loss       : 0% (no drops)")

        print(f"{'═'*62}\n")

    def _print_latency_stats(self, label, latencies_ns):
        """Print percentile latency breakdown."""
        if not latencies_ns:
            return

        latencies_us = [ns / 1000 for ns in latencies_ns]
        p50  = statistics.median(latencies_us)
        p95  = sorted(latencies_us)[int(len(latencies_us) * 0.95)]
        p99  = sorted(latencies_us)[int(len(latencies_us) * 0.99)]
        pmax = max(latencies_us)
        pavg = statistics.mean(latencies_us)

        print(f"  {label}:")
        print(f"    avg={pavg:>8.1f}μs  "
              f"p50={p50:>8.1f}μs  "
              f"p95={p95:>8.1f}μs  "
              f"p99={p99:>8.1f}μs  "
              f"max={pmax:>8.1f}μs")

    def _print_header(self):
        print(f"\n{'═'*62}")
        print(f"  ⚡ CAN Bus Stress Test")
        print(f"{'═'*62}")


# ── CLI ───────────────────────────────────────────────────────────────────────

def parse_args():
    parser = argparse.ArgumentParser(
        description="Stress test the shared memory CAN bus"
    )
    parser.add_argument(
        "--mode", "-m",
        choices=["full", "write-only", "read-only"],
        default="full",
        help="Test mode (default: full)"
    )
    parser.add_argument(
        "--writers", "-w",
        type=int, default=4,
        help="Number of writer threads (default: 4)"
    )
    parser.add_argument(
        "--readers", "-r",
        type=int, default=4,
        help="Number of reader threads (default: 4)"
    )
    parser.add_argument(
        "--duration", "-d",
        type=int, default=5,
        help="Test duration in seconds (default: 5)"
    )
    parser.add_argument(
        "--frame-size", "-s",
        type=int, default=8,
        choices=[1, 2, 4, 8, 12, 16, 20, 24, 32, 48, 64],
        help="Frame data payload size in bytes (default: 8)"
    )
    parser.add_argument(
        "--fd",
        action="store_true",
        help="Use CAN FD frames with BRS (default: classic CAN)"
    )
    return parser.parse_args()


def main():
    args = parse_args()

    # Clean SHM before test
    shm_path = "/dev/shm/virtual_can_bus"
    if os.path.exists(shm_path):
        os.unlink(shm_path)

    test = StressTest(args)
    test.run()


if __name__ == "__main__":
    main()
