// simulator/src/chips/can_encoder.ts

/**
 * CAN / CAN FD bit-level encoder.
 *
 * TypeScript port of bus_broker/core/encoder.py.
 * Generates the raw bit stream for a CAN frame including:
 *   - SOF, arbitration, control, data, CRC, ACK, EOF, IFS
 *   - Bit stuffing
 *   - BRS index tracking for rate switching
 */

// ── CAN FD DLC Map ──────────────────────────────────────────────────────────

/** Maps DLC code → actual data byte count for CAN FD */
export const CAN_FD_DLC_MAP: Record<number, number> = {
  0: 0, 1: 1, 2: 2, 3: 3, 4: 4, 5: 5, 6: 6, 7: 7, 8: 8,
  9: 12, 10: 16, 11: 20, 12: 24, 13: 32, 14: 48, 15: 64,
};

/** Reverse map: byte count → DLC code */
export const BYTES_TO_DLC: Record<number, number> = {};
for (const [dlc, bytes] of Object.entries(CAN_FD_DLC_MAP)) {
  BYTES_TO_DLC[bytes] = Number(dlc);
}

// ── Frame Structure ─────────────────────────────────────────────────────────

export interface CANFrameData {
  arbitrationId: number;
  dlc: number;
  data: Uint8Array;
  isExtended: boolean;
  isRemote: boolean;
  isFD: boolean;
  brs: boolean; // Bit Rate Switch
}

export interface EncodeResult {
  bits: number[];   // Complete stuffed bit stream
  brsIndex: number; // Bit position where data-rate phase starts (0 = no BRS)
}

// ── Encoder ─────────────────────────────────────────────────────────────────

const EOF_BITS = 7;
const INTERFRAME_BITS = 3;

export class CANEncoder {
  /**
   * Encode a frame into a bit stream.
   * Returns the stuffed bits and BRS index.
   */
  encode(frame: CANFrameData): EncodeResult {
    if (frame.isFD) {
      return this.encodeFD(frame);
    }
    return { bits: this.encodeStandard(frame), brsIndex: 0 };
  }

  // ── Standard CAN (2.0A / 2.0B) ──────────────────────────────────────

  private encodeStandard(frame: CANFrameData): number[] {
    const raw: number[] = [];

    // SOF
    raw.push(0);

    // Arbitration
    if (frame.isExtended) {
      const baseId = (frame.arbitrationId >> 18) & 0x7ff;
      raw.push(...this.intToBits(baseId, 11));
      raw.push(1); // SRR
      raw.push(1); // IDE
      const extId = frame.arbitrationId & 0x3ffff;
      raw.push(...this.intToBits(extId, 18));
      raw.push(frame.isRemote ? 1 : 0); // RTR
      raw.push(0); // r1
      raw.push(0); // r0
    } else {
      raw.push(...this.intToBits(frame.arbitrationId, 11));
      raw.push(frame.isRemote ? 1 : 0); // RTR
      raw.push(0); // IDE
      raw.push(0); // r0
    }

    // DLC
    raw.push(...this.intToBits(frame.dlc, 4));

    // Data
    if (!frame.isRemote) {
      for (let i = 0; i < frame.dlc; i++) {
        raw.push(...this.intToBits(frame.data[i] ?? 0, 8));
      }
    }

    // CRC-15
    const crc = this.calculateCRC15(raw);
    raw.push(...this.intToBits(crc, 15));

    // Bit stuffing
    const stuffed = this.applyBitStuffing(raw);

    // CRC delimiter
    stuffed.push(1);
    // ACK
    stuffed.push(0);
    stuffed.push(1);
    // EOF
    for (let i = 0; i < EOF_BITS; i++) stuffed.push(1);
    // IFS
    for (let i = 0; i < INTERFRAME_BITS; i++) stuffed.push(1);

    return stuffed;
  }

  // ── CAN FD ────────────────────────────────────────────────────────────

  private encodeFD(frame: CANFrameData): EncodeResult {
    const raw: number[] = [];

    // SOF
    raw.push(0);

    // Arbitration
    if (frame.isExtended) {
      const baseId = (frame.arbitrationId >> 18) & 0x7ff;
      raw.push(...this.intToBits(baseId, 11));
      raw.push(1); // SRR
      raw.push(1); // IDE
      const extId = frame.arbitrationId & 0x3ffff;
      raw.push(...this.intToBits(extId, 18));
      raw.push(0); // RTR — always dominant in FD
    } else {
      raw.push(...this.intToBits(frame.arbitrationId, 11));
      raw.push(0); // RTR — always dominant in FD
      raw.push(0); // IDE
    }

    // Control
    raw.push(0); // r0
    raw.push(1); // FDF — marks as CAN FD
    raw.push(0); // res

    // BRS bit
    const brsRawIndex = raw.length;
    raw.push(frame.brs ? 1 : 0);

    raw.push(0); // ESI
    raw.push(...this.intToBits(frame.dlc, 4));

    // Data
    const dataBytes = CAN_FD_DLC_MAP[frame.dlc] ?? frame.dlc;
    for (let i = 0; i < dataBytes; i++) {
      raw.push(...this.intToBits(frame.data[i] ?? 0, 8));
    }

    // CRC
    if (dataBytes <= 16) {
      const crc = this.calculateCRC17(raw);
      raw.push(...this.intToBits(crc, 17));
    } else {
      const crc = this.calculateCRC21(raw);
      raw.push(...this.intToBits(crc, 21));
    }

    // Bit stuffing
    const stuffed = this.applyBitStuffing(raw);

    // BRS index in stuffed stream
    let brsIndex = 0;
    if (frame.brs) {
      brsIndex = this.rawToStuffedIndex(raw, brsRawIndex + 1);
    }

    // CRC delimiter
    stuffed.push(1);
    // ACK
    stuffed.push(0);
    stuffed.push(1);
    // EOF
    for (let i = 0; i < EOF_BITS; i++) stuffed.push(1);
    // IFS
    for (let i = 0; i < INTERFRAME_BITS; i++) stuffed.push(1);

    return { bits: stuffed, brsIndex };
  }

  // ── Bit Stuffing ──────────────────────────────────────────────────────

  applyBitStuffing(raw: number[]): number[] {
    if (raw.length === 0) return [];
    const result: number[] = [raw[0]];
    let lastBit = raw[0];
    let consecutive = 1;

    for (let i = 1; i < raw.length; i++) {
      const bit = raw[i];
      result.push(bit);

      if (bit === lastBit) {
        consecutive++;
        if (consecutive === 5) {
          // Insert stuff bit (opposite polarity)
          const stuffBit = 1 - bit;
          result.push(stuffBit);
          lastBit = stuffBit;
          consecutive = 1;
        }
      } else {
        consecutive = 1;
        lastBit = bit;
      }
    }

    return result;
  }

  removeBitStuffing(bits: number[]): number[] {
    if (bits.length === 0) return [];
    const result: number[] = [bits[0]];
    let lastBit = bits[0];
    let consecutive = 1;

    let i = 1;
    while (i < bits.length) {
      const bit = bits[i];

      if (bit === lastBit) {
        consecutive++;
        result.push(bit);
        if (consecutive === 5) {
          i++; // skip stuff bit
          if (i < bits.length) {
            lastBit = bits[i]; // stuff bit value
            consecutive = 1;
          }
          i++;
          continue;
        }
      } else {
        consecutive = 1;
        result.push(bit);
        lastBit = bit;
      }
      i++;
    }

    return result;
  }

  // ── CRC Calculations ─────────────────────────────────────────────────

  private calculateCRC15(bits: number[]): number {
    let crc = 0;
    const poly = 0x4599;
    for (const bit of bits) {
      const msb = (crc >> 14) & 1;
      crc = ((crc << 1) & 0x7fff) | bit;
      if (msb) crc ^= poly;
    }
    return crc;
  }

  private calculateCRC17(bits: number[]): number {
    let crc = 0;
    const poly = 0x1685b;
    for (const bit of bits) {
      const msb = (crc >> 16) & 1;
      crc = ((crc << 1) & 0x1ffff) | bit;
      if (msb) crc ^= poly;
    }
    return crc;
  }

  private calculateCRC21(bits: number[]): number {
    let crc = 0;
    const poly = 0x302899;
    for (const bit of bits) {
      const msb = (crc >> 20) & 1;
      crc = ((crc << 1) & 0x1fffff) | bit;
      if (msb) crc ^= poly;
    }
    return crc;
  }

  // ── Helpers ───────────────────────────────────────────────────────────

  intToBits(value: number, length: number): number[] {
    const bits: number[] = [];
    for (let i = length - 1; i >= 0; i--) {
      bits.push((value >> i) & 1);
    }
    return bits;
  }

  bitsToInt(bits: number[]): number {
    let result = 0;
    for (const b of bits) {
      result = (result << 1) | b;
    }
    return result;
  }

  /**
   * Map a raw (pre-stuffing) bit index to its position in the
   * stuffed stream. Replays stuffing logic to count insertions.
   */
  private rawToStuffedIndex(raw: number[], rawIndex: number): number {
    let stuffedPos = 1; // first bit is always at position 0
    let lastBit = raw[0];
    let consecutive = 1;

    for (let i = 1; i < Math.min(rawIndex, raw.length); i++) {
      const bit = raw[i];
      if (bit === lastBit) {
        consecutive++;
        if (consecutive === 5) {
          stuffedPos += 2; // raw bit + stuff bit
          lastBit = 1 - bit;
          consecutive = 1;
          continue;
        }
      } else {
        consecutive = 1;
      }
      stuffedPos++;
      lastBit = bit;
    }

    return stuffedPos;
  }
}
