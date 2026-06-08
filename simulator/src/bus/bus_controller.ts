// simulator/src/bus/bus_controller.ts

/**
 * BusController — the main-thread tick engine.
 *
 * This runs on the main thread (or a dedicated controller thread).
 * It drives the bus clock, merges wire outputs from all nodes,
 * and synchronizes worker threads via SharedArrayBuffer barriers.
 *
 * Lifecycle:
 *   1. Create bus + controller
 *   2. Workers connect (via SharedArrayBuffer passed through workerData)
 *   3. Call controller.run() — starts the tick loop
 *   4. Call controller.stop() — halts the loop
 *
 * Fast-forward mode:
 *   When enabled and the bus has been idle for >100 ticks,
 *   the controller skips ticks without waiting for workers.
 *   Workers detect the skip via the tick counter jump and
 *   fast-forward their internal state accordingly.
 */

import { PhysicalBus } from './physical_bus.js';
import { IBusProtocol, BusState } from './protocol.js';

// ── Stats ────────────────────────────────────────────────────────────────────

export interface BusStats {
  totalTicks: number;
  busyTicks: number;
  idleTicks: number;
  skippedTicks: number;
  elapsedMs: number;
  effectiveBitRate: number; // bits/sec (simulated)
  wallClockRatio: number; // simulated_time / wall_clock_time
}

// ── Bus Controller ───────────────────────────────────────────────────────────

export class BusController {
  readonly bus: PhysicalBus;

  private _running = false;
  private _tickCallback: ((tick: number) => void) | null = null;
  private _stats: BusStats = {
    totalTicks: 0,
    busyTicks: 0,
    idleTicks: 0,
    skippedTicks: 0,
    elapsedMs: 0,
    effectiveBitRate: 0,
    wallClockRatio: 0,
  };

  // Fast-forward settings
  private _fastForwardEnabled = true;
  private _fastForwardThreshold = 100; // idle ticks before fast-forward

  constructor(protocol: IBusProtocol, nodeCount: number) {
    this.bus = new PhysicalBus(protocol, nodeCount);
  }

  /** Get the SharedArrayBuffer to pass to workers */
  get buffer(): SharedArrayBuffer {
    return this.bus.buffer;
  }

  /** Current statistics */
  get stats(): Readonly<BusStats> {
    return { ...this._stats };
  }

  /** Enable/disable fast-forward mode */
  set fastForward(enabled: boolean) {
    this._fastForwardEnabled = enabled;
    this.bus.mem.fastForward = enabled;
  }

  /** Register a callback that fires on every tick (useful for monitoring) */
  onTick(callback: (tick: number) => void): void {
    this._tickCallback = callback;
  }

  /**
   * Run the tick loop for a fixed number of ticks.
   *
   * This is synchronous and blocks the calling thread.
   * For continuous operation, call run() with a large tick count
   * or use runAsync().
   */
  run(maxTicks: number): BusStats {
    this._running = true;
    const startTime = performance.now();
    let busyCount = 0;
    let idleCount = 0;
    let skipped = 0;

    for (let t = 0; t < maxTicks && this._running; t++) {
      // Fast-forward: skip idle ticks
      if (
        this._fastForwardEnabled &&
        this.bus.mem.idleTickCount > this._fastForwardThreshold
      ) {
        // Skip ahead without waiting for workers
        // Just advance the tick counter
        this.bus.mem.tick = this.bus.mem.tick + 1;
        idleCount++;
        skipped++;
        continue;
      }

      // Normal tick: merge + sync with workers
      this.bus.tick();
      this.bus.waitForWorkers((busyCount + idleCount + 1 - skipped) * this.bus.nodeCount);

      // Track stats
      if (this.bus.isIdle()) {
        idleCount++;
      } else {
        busyCount++;
      }

      // Callback
      if (this._tickCallback) {
        this._tickCallback(t);
      }
    }

    const elapsed = performance.now() - startTime;
    const totalTicks = busyCount + idleCount;
    const simulatedTimeNs = totalTicks * this.bus.currentBitTimeNs();
    const simulatedTimeMs = simulatedTimeNs / 1_000_000;

    this._stats = {
      totalTicks,
      busyTicks: busyCount,
      idleTicks: idleCount,
      skippedTicks: skipped,
      elapsedMs: elapsed,
      effectiveBitRate:
        elapsed > 0 ? (totalTicks / (elapsed / 1000)) : 0,
      wallClockRatio:
        elapsed > 0 ? simulatedTimeMs / elapsed : 0,
    };

    return this._stats;
  }

  /**
   * Run the tick loop asynchronously with periodic yielding.
   * This allows the event loop to process other tasks (e.g., monitoring).
   *
   * @param ticksPerBatch - How many ticks to run before yielding
   */
  async runAsync(
    maxTicks: number,
    ticksPerBatch: number = 10_000
  ): Promise<BusStats> {
    this._running = true;
    const startTime = performance.now();
    let busyCount = 0;
    let idleCount = 0;
    let skipped = 0;
    let ticksDone = 0;

    while (ticksDone < maxTicks && this._running) {
      const batchEnd = Math.min(ticksDone + ticksPerBatch, maxTicks);

      for (let t = ticksDone; t < batchEnd && this._running; t++) {
        if (
          this._fastForwardEnabled &&
          this.bus.mem.idleTickCount > this._fastForwardThreshold
        ) {
          this.bus.mem.tick = this.bus.mem.tick + 1;
          idleCount++;
          skipped++;
          ticksDone++;
          continue;
        }

        this.bus.tick();
        this.bus.waitForWorkers((busyCount + idleCount + 1 - skipped) * this.bus.nodeCount);

        if (this.bus.isIdle()) {
          idleCount++;
        } else {
          busyCount++;
        }

        if (this._tickCallback) {
          this._tickCallback(t);
        }
        ticksDone++;
      }

      // Yield to event loop
      const elapsed = performance.now() - startTime;
      const totalTicks = busyCount + idleCount;
      const simulatedTimeNs = totalTicks * this.bus.currentBitTimeNs();
      const simulatedTimeMs = simulatedTimeNs / 1_000_000;
      this._stats = {
        totalTicks,
        busyTicks: busyCount,
        idleTicks: idleCount,
        skippedTicks: skipped,
        elapsedMs: elapsed,
        effectiveBitRate:
          elapsed > 0 ? (totalTicks / (elapsed / 1000)) : 0,
        wallClockRatio:
          elapsed > 0 ? simulatedTimeMs / elapsed : 0,
      };

      await new Promise((resolve) => setImmediate(resolve));
    }

    const elapsed = performance.now() - startTime;
    const totalTicks = busyCount + idleCount;
    const simulatedTimeNs = totalTicks * this.bus.currentBitTimeNs();
    const simulatedTimeMs = simulatedTimeNs / 1_000_000;

    this._stats = {
      totalTicks,
      busyTicks: busyCount,
      idleTicks: idleCount,
      skippedTicks: skipped,
      elapsedMs: elapsed,
      effectiveBitRate:
        elapsed > 0 ? (totalTicks / (elapsed / 1000)) : 0,
      wallClockRatio:
        elapsed > 0 ? simulatedTimeMs / elapsed : 0,
    };

    return this._stats;
  }

  /** Stop the tick loop */
  stop(): void {
    this._running = false;
  }

  /** Check if the controller is currently running */
  get running(): boolean {
    return this._running;
  }

  /**
   * Run a single tick — useful for testing.
   * Does NOT wait for workers (caller must handle sync).
   */
  singleTick(): void {
    this.bus.tick();
  }
}
