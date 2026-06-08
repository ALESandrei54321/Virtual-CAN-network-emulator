// simulator/src/bus/protocol.ts

/**
 * Protocol-agnostic bus abstraction.
 *
 * Any communication protocol (CAN FD, LIN, FlexRay, Automotive Ethernet)
 * can be plugged in by implementing the IBusProtocol interface.
 * The PhysicalBus and BusController work with any protocol — they only
 * care about wires, timing, and the merge rule.
 */

// ── Wire Merge Strategies ────────────────────────────────────────────────────

/**
 * Defines how multiple node outputs combine on a shared wire.
 *
 * WIRED_AND  — CAN, LIN: any node driving dominant (0) wins.
 *              In electrical terms: open-drain with pull-up.
 * WIRED_OR   — Some custom protocols: any node driving high wins.
 * TDMA       — FlexRay: only one node transmits at a time (slot-based).
 * EXCLUSIVE  — Point-to-point: only one node can drive at a time.
 */
export enum WireMergeStrategy {
  WIRED_AND = 'WIRED_AND',
  WIRED_OR = 'WIRED_OR',
  TDMA = 'TDMA',
  EXCLUSIVE = 'EXCLUSIVE',
}

// ── Bus State ────────────────────────────────────────────────────────────────

export enum BusState {
  IDLE = 0,
  BUSY = 1,
  ERROR = 2,
}

export enum BitRate {
  NOMINAL = 0,
  DATA = 1,
}

// ── Protocol Interface ───────────────────────────────────────────────────────

/**
 * Every bus protocol must implement this interface.
 * It tells the bus core how many wires it uses, how to merge them,
 * and what the timing looks like.
 */
export interface IBusProtocol {
  /** Human-readable protocol name, e.g. "CAN_FD", "LIN", "FlexRay" */
  readonly name: string;

  /** Number of physical wires on the bus (CAN=2, LIN=1, FlexRay=4) */
  readonly wireCount: number;

  /** Names for each wire, e.g. ["CAN_H", "CAN_L"] */
  readonly wireNames: readonly string[];

  /** How outputs from multiple nodes merge on each wire */
  readonly mergeStrategy: WireMergeStrategy;

  /** Nominal (arbitration) bit time in nanoseconds */
  readonly nominalBitTimeNs: number;

  /** Data phase bit time in nanoseconds (same as nominal if no BRS) */
  readonly dataBitTimeNs: number;

  /** Whether this protocol supports bit rate switching mid-frame */
  readonly supportsBRS: boolean;

  /** Maximum number of nodes supported on this bus */
  readonly maxNodes: number;

  /**
   * Merge outputs from multiple nodes into a single bus value for one wire.
   *
   * @param outputs - Array of values driven by each node (0=dominant, 1=recessive for CAN)
   * @returns The merged bus value
   */
  mergeWire(outputs: number[]): number;

  /**
   * Determine the idle (recessive) value for a wire.
   * CAN_H idle = 0 (not driven), CAN_L idle = 0 (not driven).
   * In terms of "dominant/recessive": recessive = 1 for both CAN_H and CAN_L.
   */
  idleValue(wireIndex: number): number;

  /**
   * Given the merged wire values, determine if the bus is idle.
   * For CAN: idle when CAN_H and CAN_L are both recessive (all nodes recessive).
   */
  isBusIdle(mergedWires: number[]): boolean;
}

// ── Node Output ──────────────────────────────────────────────────────────────

/**
 * Per-node wire output state.
 * Each transceiver drives its wire values here; the bus controller reads them.
 */
export interface INodeOutput {
  /** Node identifier (0-based index) */
  readonly nodeIndex: number;

  /** Current output value for each wire */
  wireValues: number[];

  /** Status flags */
  status: NodeStatus;
}

export enum NodeStatus {
  IDLE = 0,
  TRANSMITTING = 1,
  RECEIVING = 2,
  ARBITRATION_LOST = 3,
  ERROR = 4,
}
