// simulator/src/bus/protocols/can_fd_bus.ts

/**
 * CAN FD Bus Protocol Driver
 *
 * Implements the IBusProtocol interface for CAN FD (ISO 11898-1:2015).
 *
 * Physical layer:
 *   - 2 wires: CAN_H and CAN_L
 *   - Wired-AND: any node driving dominant (CAN_H=high, CAN_L=low) wins
 *   - Recessive state: CAN_H ≈ CAN_L ≈ 2.5V (both driven to 0 in our model)
 *   - Dominant state: CAN_H > CAN_L (CAN_H=1, CAN_L=1 in our model)
 *
 * Bit representation in our model:
 *   Recessive (1): CAN_H=0, CAN_L=0 (no node is driving)
 *   Dominant  (0): CAN_H=1, CAN_L=1 (at least one node driving dominant)
 *
 * The "dominant wins" rule is implemented via OR merge:
 *   If ANY node outputs 1 (dominant) on CAN_H, the merged CAN_H = 1.
 *   This models the wired-AND of the CAN bus physical layer.
 */

import { IBusProtocol, WireMergeStrategy } from '../protocol.js';

// ── Wire indices ─────────────────────────────────────────────────────────────

export const WIRE_CAN_H = 0;
export const WIRE_CAN_L = 1;

// ── Protocol Implementation ─────────────────────────────────────────────────

export class CANFDBusProtocol implements IBusProtocol {
  readonly name = 'CAN_FD';
  readonly wireCount = 2;
  readonly wireNames = ['CAN_H', 'CAN_L'] as const;
  readonly mergeStrategy = WireMergeStrategy.WIRED_AND;
  readonly supportsBRS = true;
  readonly maxNodes = 16;

  /** Nominal bit time in ns (arbitration phase) */
  readonly nominalBitTimeNs: number;

  /** Data bit time in ns (data phase, after BRS) */
  readonly dataBitTimeNs: number;

  constructor(
    nominalBitRate: number = 500_000,
    dataBitRate: number = 2_000_000
  ) {
    this.nominalBitTimeNs = Math.round(1e9 / nominalBitRate);
    this.dataBitTimeNs = Math.round(1e9 / dataBitRate);
  }

  /**
   * Merge outputs from multiple nodes for one wire.
   *
   * CAN uses wired-AND at the electrical level, which means:
   *   - If any node drives dominant (1 in our model), the bus is dominant
   *   - Only if ALL nodes are recessive (0), the bus is recessive
   *
   * This is effectively a logical OR of the output values.
   */
  mergeWire(outputs: number[]): number {
    for (const val of outputs) {
      if (val !== 0) return 1; // dominant wins
    }
    return 0; // all recessive
  }

  /**
   * Idle value for each wire.
   * In idle state, no node is driving — both wires are 0 (recessive).
   */
  idleValue(_wireIndex: number): number {
    return 0; // recessive = not driven
  }

  /**
   * The bus is idle when both CAN_H and CAN_L are in recessive state (0).
   */
  isBusIdle(mergedWires: number[]): boolean {
    return mergedWires[WIRE_CAN_H] === 0 && mergedWires[WIRE_CAN_L] === 0;
  }

  // ── CAN-specific helpers ────────────────────────────────────────────────

  /**
   * Check if a node lost arbitration.
   * A transmitting node loses arbitration when it drives recessive (0)
   * but reads back dominant (1) from the bus.
   */
  lostArbitration(nodeDrove: number, busValue: number): boolean {
    return nodeDrove === 0 && busValue === 1;
  }

  /**
   * Convert a logical CAN bit (0=dominant, 1=recessive) to wire values.
   *
   * Dominant: drive CAN_H high, CAN_L high (differential pair active)
   * Recessive: don't drive (CAN_H=0, CAN_L=0)
   */
  bitToWires(bit: number): [number, number] {
    if (bit === 0) {
      // Dominant: actively drive the differential pair
      return [1, 1]; // [CAN_H, CAN_L]
    }
    // Recessive: release the bus
    return [0, 0];
  }

  /**
   * Convert merged wire values back to a logical CAN bit.
   * If either CAN_H or CAN_L is driven (1), the bus is dominant (0).
   */
  wiresToBit(canh: number, canl: number): number {
    return (canh !== 0 || canl !== 0) ? 0 : 1; // dominant=0, recessive=1
  }
}
