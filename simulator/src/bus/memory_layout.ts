// simulator/src/bus/memory_layout.ts

/**
 * SharedArrayBuffer memory layout for the physical bus.
 *
 * This defines the exact byte layout of the shared memory region
 * that all worker threads (ECU processes) and the bus controller
 * read/write through Atomics.
 *
 * Layout is protocol-agnostic — sized for the maximum wire count
 * and node count. Unused slots are zeroed.
 *
 * ┌─────────────────────────────────────────────────────┐
 * │  CONTROL REGION (Int32Array — Atomics-compatible)   │
 * │  [0] tick_counter       — bus clock                 │
 * │  [1] bus_state          — idle/busy/error           │
 * │  [2] current_bit_rate   — nominal/data              │
 * │  [3] node_count         — registered nodes          │
 * │  [4] barrier_gen        — tick generation counter    │
 * │  [5] ready_count        — workers done this tick     │
 * │  [6] fast_forward       — skip idle ticks flag       │
 * │  [7] idle_tick_count    — consecutive idle ticks     │
 * ├─────────────────────────────────────────────────────┤
 * │  WIRE REGION (Uint8Array overlay)                    │
 * │  Merged wires: MAX_WIRES bytes                       │
 * │  Per-node outputs: MAX_NODES × MAX_WIRES bytes      │
 * ├─────────────────────────────────────────────────────┤
 * │  NODE STATUS (Int32Array)                            │
 * │  Per-node status flags: MAX_NODES × 4 bytes         │
 * └─────────────────────────────────────────────────────┘
 */

// ── Constants ────────────────────────────────────────────────────────────────

/** Maximum number of wires per bus (FlexRay has 4, CAN has 2) */
export const MAX_WIRES = 4;

/** Maximum number of nodes on a single bus */
export const MAX_NODES = 16;

// ── Int32 Control Region (indices into Int32Array) ───────────────────────────

export const CTRL_TICK = 0;
export const CTRL_BUS_STATE = 1;
export const CTRL_BIT_RATE = 2;
export const CTRL_NODE_COUNT = 3;
export const CTRL_BARRIER = 4;
export const CTRL_READY = 5;
export const CTRL_FAST_FWD = 6;
export const CTRL_IDLE_TICKS = 7;

/** Number of Int32 slots in the control region */
const CTRL_INT32_COUNT = 8;

/** Byte offset where the control region ends / wire region begins */
const WIRE_REGION_BYTE_OFFSET = CTRL_INT32_COUNT * 4; // 32 bytes

// ── Wire Region (Uint8Array overlay) ─────────────────────────────────────────

/** Byte offset of merged wire values */
export const MERGED_WIRES_OFFSET = WIRE_REGION_BYTE_OFFSET;

/** Byte offset of per-node wire outputs */
export const NODE_WIRES_OFFSET = MERGED_WIRES_OFFSET + MAX_WIRES;

/** Total bytes for wire region */
const WIRE_REGION_SIZE = MAX_WIRES + MAX_NODES * MAX_WIRES; // 4 + 16*4 = 68

// ── Node Status Region (Int32Array) ──────────────────────────────────────────

/** Byte offset where node status begins */
const STATUS_REGION_BYTE_OFFSET = WIRE_REGION_BYTE_OFFSET + WIRE_REGION_SIZE;

// Align to 4 bytes
const STATUS_REGION_ALIGNED =
  Math.ceil(STATUS_REGION_BYTE_OFFSET / 4) * 4;

/** Int32 index for node status (relative to status region start) */
export const NODE_STATUS_INT32_OFFSET = STATUS_REGION_ALIGNED / 4;

// ── Total Buffer Size ────────────────────────────────────────────────────────

/** Total SharedArrayBuffer size in bytes */
export const BUS_SHM_SIZE =
  STATUS_REGION_ALIGNED + MAX_NODES * 4; // status: 1 int32 per node

// ── Accessor Helpers ─────────────────────────────────────────────────────────

/**
 * Helper class that provides typed views and accessors
 * over the SharedArrayBuffer.
 */
export class BusMemoryView {
  /** Int32Array view for Atomics operations (control + status regions) */
  readonly ctrl: Int32Array;

  /** Uint8Array view for wire values */
  readonly wires: Uint8Array;

  constructor(readonly buffer: SharedArrayBuffer) {
    if (buffer.byteLength < BUS_SHM_SIZE) {
      throw new Error(
        `Bus SHM buffer too small: ${buffer.byteLength} < ${BUS_SHM_SIZE}`
      );
    }
    this.ctrl = new Int32Array(buffer);
    this.wires = new Uint8Array(buffer);
  }

  // ── Control region ──────────────────────────────────────────────────────

  get tick(): number {
    return Atomics.load(this.ctrl, CTRL_TICK);
  }
  set tick(v: number) {
    Atomics.store(this.ctrl, CTRL_TICK, v);
  }

  get busState(): number {
    return Atomics.load(this.ctrl, CTRL_BUS_STATE);
  }
  set busState(v: number) {
    Atomics.store(this.ctrl, CTRL_BUS_STATE, v);
  }

  get currentBitRate(): number {
    return Atomics.load(this.ctrl, CTRL_BIT_RATE);
  }
  set currentBitRate(v: number) {
    Atomics.store(this.ctrl, CTRL_BIT_RATE, v);
  }

  get nodeCount(): number {
    return Atomics.load(this.ctrl, CTRL_NODE_COUNT);
  }
  set nodeCount(v: number) {
    Atomics.store(this.ctrl, CTRL_NODE_COUNT, v);
  }

  get barrier(): number {
    return Atomics.load(this.ctrl, CTRL_BARRIER);
  }

  get readyCount(): number {
    return Atomics.load(this.ctrl, CTRL_READY);
  }

  get fastForward(): boolean {
    return Atomics.load(this.ctrl, CTRL_FAST_FWD) !== 0;
  }
  set fastForward(v: boolean) {
    Atomics.store(this.ctrl, CTRL_FAST_FWD, v ? 1 : 0);
  }

  get idleTickCount(): number {
    return Atomics.load(this.ctrl, CTRL_IDLE_TICKS);
  }
  set idleTickCount(v: number) {
    Atomics.store(this.ctrl, CTRL_IDLE_TICKS, v);
  }

  // ── Barrier synchronization ─────────────────────────────────────────────

  /** Bus controller: advance the barrier and wake all workers */
  advanceBarrier(): void {
    Atomics.add(this.ctrl, CTRL_BARRIER, 1);
    Atomics.notify(this.ctrl, CTRL_BARRIER);
  }

  /** Worker: wait until the barrier advances past lastSeen */
  waitForTick(lastSeen: number): number {
    Atomics.wait(this.ctrl, CTRL_BARRIER, lastSeen);
    return Atomics.load(this.ctrl, CTRL_BARRIER);
  }

  /** Worker: signal that this worker is done for the current tick */
  signalReady(): void {
    Atomics.add(this.ctrl, CTRL_READY, 1);
    Atomics.notify(this.ctrl, CTRL_READY);
  }

  /** Bus controller: reset the ready counter for the next tick */
  resetReady(): void {
    Atomics.store(this.ctrl, CTRL_READY, 0);
  }

  // ── Wire accessors ──────────────────────────────────────────────────────

  /** Get the merged (bus-visible) value of a wire */
  getMergedWire(wireIndex: number): number {
    return this.wires[MERGED_WIRES_OFFSET + wireIndex];
  }

  /** Set the merged value of a wire (bus controller only) */
  setMergedWire(wireIndex: number, value: number): void {
    this.wires[MERGED_WIRES_OFFSET + wireIndex] = value;
  }

  /** Get a node's output for a specific wire */
  getNodeWire(nodeIndex: number, wireIndex: number): number {
    return this.wires[NODE_WIRES_OFFSET + nodeIndex * MAX_WIRES + wireIndex];
  }

  /** Set a node's output for a specific wire (worker only) */
  setNodeWire(nodeIndex: number, wireIndex: number, value: number): void {
    this.wires[NODE_WIRES_OFFSET + nodeIndex * MAX_WIRES + wireIndex] = value;
  }

  /** Get all node outputs for a specific wire (for merge computation) */
  getAllNodeWires(wireIndex: number, nodeCount: number): number[] {
    const outputs: number[] = [];
    for (let n = 0; n < nodeCount; n++) {
      outputs.push(this.getNodeWire(n, wireIndex));
    }
    return outputs;
  }

  // ── Node status ─────────────────────────────────────────────────────────

  getNodeStatus(nodeIndex: number): number {
    return Atomics.load(this.ctrl, NODE_STATUS_INT32_OFFSET + nodeIndex);
  }

  setNodeStatus(nodeIndex: number, status: number): void {
    Atomics.store(this.ctrl, NODE_STATUS_INT32_OFFSET + nodeIndex, status);
  }
}

/**
 * Create a fresh SharedArrayBuffer for a bus.
 * Initialize all wires to the idle (recessive) state.
 */
export function createBusBuffer(
  nodeCount: number,
  idleValues: number[] // one per wire
): SharedArrayBuffer {
  const buffer = new SharedArrayBuffer(BUS_SHM_SIZE);
  const view = new BusMemoryView(buffer);

  view.nodeCount = nodeCount;
  view.busState = 0; // idle
  view.currentBitRate = 0; // nominal

  // Initialize all wires to idle
  for (let w = 0; w < idleValues.length; w++) {
    view.setMergedWire(w, idleValues[w]);
    for (let n = 0; n < nodeCount; n++) {
      view.setNodeWire(n, w, idleValues[w]);
    }
  }

  return buffer;
}
