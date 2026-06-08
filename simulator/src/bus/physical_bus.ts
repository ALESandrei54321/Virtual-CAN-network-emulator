// simulator/src/bus/physical_bus.ts

/**
 * PhysicalBus — protocol-agnostic shared bus.
 *
 * This is the core abstraction that workers and the bus controller
 * interact with. It wraps a SharedArrayBuffer and provides:
 *
 *   - For the bus controller: merge wires, advance ticks, fast-forward
 *   - For workers: read bus state, drive outputs, synchronize
 *
 * The bus does NOT know about CAN frames, encoders, or transceivers.
 * It only knows about wires, bits, and timing.
 */

import {
  BusMemoryView,
  createBusBuffer,
  CTRL_BARRIER,
  CTRL_READY,
} from './memory_layout.js';
import { IBusProtocol, BusState, BitRate } from './protocol.js';

// ── PhysicalBus ──────────────────────────────────────────────────────────────

export class PhysicalBus {
  readonly mem: BusMemoryView;
  readonly protocol: IBusProtocol;
  private readonly _nodeCount: number;

  /**
   * Create a new physical bus with a fresh SharedArrayBuffer.
   */
  constructor(protocol: IBusProtocol, nodeCount: number) {
    if (nodeCount > protocol.maxNodes) {
      throw new Error(
        `Too many nodes: ${nodeCount} > max ${protocol.maxNodes} for ${protocol.name}`
      );
    }

    this.protocol = protocol;
    this._nodeCount = nodeCount;

    // Create SHM with idle wire values
    const idleValues: number[] = [];
    for (let w = 0; w < protocol.wireCount; w++) {
      idleValues.push(protocol.idleValue(w));
    }
    const buffer = createBusBuffer(nodeCount, idleValues);
    this.mem = new BusMemoryView(buffer);
  }

  /** Get the underlying SharedArrayBuffer (to pass to workers) */
  get buffer(): SharedArrayBuffer {
    return this.mem.buffer;
  }

  get nodeCount(): number {
    return this._nodeCount;
  }

  // ── Bus Controller Operations ───────────────────────────────────────────

  /**
   * Merge all node wire outputs into the bus-visible merged wires.
   * Called by the bus controller BEFORE advancing the tick.
   */
  mergeWires(): void {
    const { protocol, mem } = this;
    const n = this._nodeCount;

    for (let w = 0; w < protocol.wireCount; w++) {
      const outputs = mem.getAllNodeWires(w, n);
      const merged = protocol.mergeWire(outputs);
      mem.setMergedWire(w, merged);
    }
  }

  /**
   * Check if the bus is currently idle (no node is driving).
   */
  isIdle(): boolean {
    const mergedWires: number[] = [];
    for (let w = 0; w < this.protocol.wireCount; w++) {
      mergedWires.push(this.mem.getMergedWire(w));
    }
    return this.protocol.isBusIdle(mergedWires);
  }

  /**
   * Advance the bus by one tick.
   *
   * 1. Merge all node outputs into bus-visible wires
   * 2. Update bus state (idle/busy)
   * 3. Track idle ticks for fast-forward
   * 4. Advance the barrier to wake workers
   */
  tick(): void {
    // 1. Merge
    this.mergeWires();

    // 2. Update state
    const idle = this.isIdle();
    this.mem.busState = idle ? BusState.IDLE : BusState.BUSY;

    // 3. Idle tracking for fast-forward
    if (idle) {
      this.mem.idleTickCount = this.mem.idleTickCount + 1;
    } else {
      this.mem.idleTickCount = 0;
    }

    // 4. Advance tick counter
    this.mem.tick = this.mem.tick + 1;

    // 5. Wake workers
    this.mem.advanceBarrier();
  }

  waitForWorkers(expectedReadyCount: number): void {
    let current = Atomics.load(this.mem.ctrl, CTRL_READY);
    let spins = 0;
    let stuckCycles = 0;
    while (current < expectedReadyCount) {
      if (spins < 2000) {
        spins++;
      } else {
        Atomics.wait(this.mem.ctrl, CTRL_READY, current, 1); // wait up to 1ms
        spins = 0;
      }
      current = Atomics.load(this.mem.ctrl, CTRL_READY);
      
      stuckCycles++;
      if (stuckCycles > 100000) {
        console.log(`[Bus] STUCK waiting for workers. Expected: ${expectedReadyCount}, Actual: ${current}`);
        stuckCycles = 0;
      }
    }
  }

  /**
   * Check if fast-forward should skip this tick.
   * Returns true if the bus has been idle for a long time and
   * fast-forward is enabled.
   */
  shouldFastForward(): boolean {
    return this.mem.fastForward && this.mem.idleTickCount > 100;
  }

  /** Switch to data bit rate (CAN FD BRS) */
  switchToDataRate(): void {
    this.mem.currentBitRate = BitRate.DATA;
  }

  /** Switch back to nominal bit rate */
  switchToNominalRate(): void {
    this.mem.currentBitRate = BitRate.NOMINAL;
  }

  /** Get the current bit time in nanoseconds */
  currentBitTimeNs(): number {
    return this.mem.currentBitRate === BitRate.DATA
      ? this.protocol.dataBitTimeNs
      : this.protocol.nominalBitTimeNs;
  }

  // ── Worker Operations ─────────────────────────────────────────────────

  /**
   * Read the bus-visible value of a specific wire.
   * Called by workers to sample the bus state.
   */
  readWire(wireIndex: number): number {
    return this.mem.getMergedWire(wireIndex);
  }

  /**
   * Drive a node's output on a specific wire.
   * Called by workers (transceiver chips) to transmit.
   */
  driveWire(nodeIndex: number, wireIndex: number, value: number): void {
    this.mem.setNodeWire(nodeIndex, wireIndex, value);
  }

  /**
   * Set a node's output to idle (recessive) on all wires.
   */
  releaseNode(nodeIndex: number): void {
    for (let w = 0; w < this.protocol.wireCount; w++) {
      this.mem.setNodeWire(nodeIndex, w, this.protocol.idleValue(w));
    }
  }
}

// ── Worker-side bus handle ────────────────────────────────────────────────────

/**
 * Lightweight handle for workers to interact with the bus.
 * Created from the SharedArrayBuffer received via workerData.
 */
export class BusWorkerHandle {
  readonly mem: BusMemoryView;

  constructor(
    buffer: SharedArrayBuffer,
    readonly nodeIndex: number,
    readonly protocol: IBusProtocol
  ) {
    this.mem = new BusMemoryView(buffer);
  }

  /** Read the merged bus value for a wire */
  readWire(wireIndex: number): number {
    return this.mem.getMergedWire(wireIndex);
  }

  /** Drive this node's output on a wire */
  driveWire(wireIndex: number, value: number): void {
    this.mem.setNodeWire(this.nodeIndex, wireIndex, value);
  }

  /** Release all wires to idle */
  release(): void {
    for (let w = 0; w < this.protocol.wireCount; w++) {
      this.mem.setNodeWire(
        this.nodeIndex,
        w,
        this.protocol.idleValue(w)
      );
    }
  }

  /** Wait for the next bus tick */
  waitForTick(lastSeen: number): number {
    return this.mem.waitForTick(lastSeen);
  }

  /** Signal that this worker has finished processing the current tick */
  signalReady(): void {
    this.mem.signalReady();
  }

  /** Get the current bus state */
  get busState(): number {
    return this.mem.busState;
  }

  /** Get the current bit rate mode */
  get currentBitRate(): number {
    return this.mem.currentBitRate;
  }

  /** Get the current tick count */
  get tick(): number {
    return this.mem.tick;
  }
}
