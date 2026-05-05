# bus_broker/core/bus_controller.py

import asyncio
import time
from dataclasses import dataclass, field
from typing import Callable

from .frames import CANFrame
from .signal import SignalConverter
from ..protocols.base_protocol import BaseProtocol
from ..transport.shm_writer import SHMBusWriter, make_slot


# ── Statistics ────────────────────────────────────────────────────────────────

@dataclass
class BusStats:
    """
    Counters updated by the bus controller.
    Useful for dissertation evaluation chapter.
    """
    frames_sent       : int   = 0
    frames_dropped    : int   = 0
    arbitration_events: int   = 0   # times more than one frame competed
    total_latency_ns  : int   = 0   # sum of all encode+write times
    min_latency_ns    : int   = 2**63
    max_latency_ns    : int   = 0

    @property
    def avg_latency_ns(self) -> float:
        if self.frames_sent == 0:
            return 0.0
        return self.total_latency_ns / self.frames_sent

    def record_latency(self, ns: int):
        self.total_latency_ns += ns
        if ns < self.min_latency_ns:
            self.min_latency_ns = ns
        if ns > self.max_latency_ns:
            self.max_latency_ns = ns

    def __repr__(self) -> str:
        return (
            f"BusStats("
            f"sent={self.frames_sent}, "
            f"dropped={self.frames_dropped}, "
            f"arb_events={self.arbitration_events}, "
            f"avg_latency={self.avg_latency_ns/1000:.1f}μs, "
            f"min={self.min_latency_ns/1000:.1f}μs, "
            f"max={self.max_latency_ns/1000:.1f}μs"
            f")"
        )


# ── Pending frame ─────────────────────────────────────────────────────────────

@dataclass(order=False)
class PendingFrame:
    """
    A frame waiting to be transmitted on the bus.
    Wraps the CANFrame with submission metadata.
    """
    frame        : CANFrame
    submitted_ns : int = field(default_factory=time.time_ns)
    retries      : int = 0

    def priority(self, protocol: BaseProtocol) -> int:
        return protocol.arbitration_priority(self.frame)


# ── Bus Controller ────────────────────────────────────────────────────────────

class BusController:
    """
    The main bus controller.

    Responsibilities:
      - Accept frames from any source (injector, CARLA bridge, ECU transmit)
      - Run CAN arbitration when multiple frames compete
      - Encode the winning frame to a DifferentialSignal
      - Write it to shared memory for all ECUs to read
      - Track statistics

    Usage:
        async with BusController(protocol, writer) as bus:
            await bus.submit(frame)
            await bus.run()
    """

    def __init__(
        self,
        protocol    : BaseProtocol,
        writer      : SHMBusWriter,
        on_frame_sent: Callable[[CANFrame], None] | None = None,
    ):
        self._protocol     = protocol
        self._writer       = writer
        self._converter    = SignalConverter()
        self._queue        : list[PendingFrame] = []
        self._lock         = asyncio.Lock()
        self._running      = False
        self._on_frame_sent = on_frame_sent
        self.stats         = BusStats()

    # ── Public API ────────────────────────────────────────────────────────────

    async def submit(self, frame: CANFrame):
        """
        Submit a frame for transmission.
        Can be called from any coroutine.
        Thread safe via asyncio lock.
        """
        ok, reason = self._protocol.validate_frame(frame)
        if not ok:
            raise ValueError(f"Frame rejected by protocol: {reason}")

        async with self._lock:
            self._queue.append(PendingFrame(frame=frame))

    async def run(self):
        """
        Main bus loop. Call this as an asyncio task.
        Runs until stop() is called.
        """
        self._running = True
        while self._running:
            await self._tick()
            # Yield control so other coroutines can submit frames
            await asyncio.sleep(0)

    def stop(self):
        self._running = False

    # ── Internal ──────────────────────────────────────────────────────────────

    async def _tick(self):
        """
        One bus tick:
          1. Grab all pending frames
          2. Run arbitration
          3. Transmit winner
          4. Requeue losers for retry
        """
        async with self._lock:
            if not self._queue:
                return
            pending = list(self._queue)
            self._queue.clear()

        if len(pending) > 1:
            self.stats.arbitration_events += 1

        # Sort by priority - lowest arbitration ID wins
        pending.sort(key=lambda p: p.priority(self._protocol))

        winner = pending[0]
        losers = pending[1:]

        # Transmit the winner
        transmitted = await self._transmit(winner)

        if transmitted:
            # Requeue losers for the next tick
            if losers:
                async with self._lock:
                    # Increment retry counter for losers
                    for loser in losers:
                        loser.retries += 1
                        self._queue.append(loser)
        else:
            # Write failed (shm full) - drop and count
            self.stats.frames_dropped += 1
            # Still requeue losers since the bus was not actually used
            async with self._lock:
                for loser in losers:
                    self._queue.append(loser)

    async def _transmit(self, pending: PendingFrame) -> bool:
        """
        Encode a frame and write it to shared memory.
        Returns True on success.
        """
        start_ns = time.time_ns()

        signal = self._protocol.encode(pending.frame)
        slot   = make_slot(
            signal,
            pending.frame,
            timestamp_ns=pending.submitted_ns
        )

        success = self._writer.write(slot)

        if success:
            elapsed = time.time_ns() - start_ns
            self.stats.frames_sent += 1
            self.stats.record_latency(elapsed)

            if self._on_frame_sent:
                self._on_frame_sent(pending.frame)

        return success

    # ── Context manager ───────────────────────────────────────────────────────

    async def __aenter__(self):
        return self

    async def __aexit__(self, *_):
        self.stop()
