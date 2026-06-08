// simulator/src/chips/can_transceiver.ts

/**
 * CAN FD Transceiver Chip — MCP2518FD model.
 *
 * This is the custom Wokwi chip that bridges the RP2040 (via SPI)
 * to the physical CAN bus (via SharedArrayBuffer).
 *
 * Architecture:
 *   RP2040 ──SPI──► CANTransceiver ──CAN_H/CAN_L──► PhysicalBus
 *                        │
 *                        ├── TX engine (frame → bit stream → bus)
 *                        ├── RX engine (bus → bit sampling → frame)
 *                        ├── Arbitration state machine
 *                        └── SPI register file
 *
 * SPI Protocol:
 *   Write: [0x02, addr, lo, hi]   — write 16-bit value to register
 *   Read:  [0x03, addr] → [lo, hi] — read 16-bit value from register
 */

import { CANEncoder, CANFrameData, CAN_FD_DLC_MAP, BYTES_TO_DLC } from './can_encoder.js';
import { BusWorkerHandle } from '../bus/physical_bus.js';
import { CANFDBusProtocol, WIRE_CAN_H, WIRE_CAN_L } from '../bus/protocols/can_fd_bus.js';

// ── SPI Register Addresses ──────────────────────────────────────────────────

export const REG_TX_ID = 0x00;
export const REG_TX_DLC = 0x01;
export const REG_TX_DATA = 0x02;
export const REG_TX_CTRL = 0x03;
export const REG_RX_ID = 0x10;
export const REG_RX_DLC = 0x11;
export const REG_RX_DATA = 0x12;
export const REG_RX_STATUS = 0x13;
export const REG_STATUS = 0x20;
export const REG_FILTER_ID = 0x30;
export const REG_FILTER_MASK = 0x31;
export const REG_CONFIG = 0x40;
export const REG_INT_FLAGS = 0xff;

// ── TX/RX CTRL flags ────────────────────────────────────────────────────────

const TX_CTRL_SEND = 0x01;
const TX_DLC_FDF = 0x80;   // bit 7: FD Frame
const TX_DLC_BRS = 0x40;   // bit 6: Bit Rate Switch

// ── Interrupt flags ─────────────────────────────────────────────────────────

export const INT_TX_COMPLETE = 0x01;
export const INT_RX_AVAILABLE = 0x02;
export const INT_ARB_LOST = 0x04;
export const INT_BUS_ERROR = 0x08;

// ── Transceiver States ──────────────────────────────────────────────────────

export enum TXState {
  IDLE = 'idle',
  WAIT_BUS_FREE = 'wait_bus_free',
  ARBITRATING = 'arbitrating',
  TRANSMITTING = 'transmitting',
}

export enum RXState {
  IDLE = 'idle',
  RECEIVING = 'receiving',
}

// ── Received frame ──────────────────────────────────────────────────────────

export interface ReceivedFrame {
  arbitrationId: number;
  dlc: number;
  data: Uint8Array;
  isFD: boolean;
  brs: boolean;
}

// ── CAN Transceiver ─────────────────────────────────────────────────────────

export class CANTransceiver {
  // ── Internal state ──────────────────────────────────────────────────────

  private encoder = new CANEncoder();
  private canProtocol: CANFDBusProtocol;

  // SPI register file
  private txId = 0;
  private txDlc = 0;
  private txFdf = false;
  private txBrs = false;
  private txDataBuffer: number[] = [];
  private txDataIndex = 0;

  // TX engine
  private txState = TXState.IDLE;
  private txBitStream: number[] = [];
  private txBitIndex = 0;
  private txBrsIndex = 0; // bit where data rate starts
  private interframeCount = 0;

  // RX engine
  private rxState = RXState.IDLE;
  private rxBitBuffer: number[] = [];
  private rxConsecutiveSame = 0;
  private rxLastBit = -1;

  // RX FIFO
  private rxFifo: ReceivedFrame[] = [];
  private readonly rxFifoMaxSize = 16;

  // Acceptance filter
  private filterId = 0;
  private filterMask = 0; // 0 = accept all

  // Interrupt flags
  private intFlags = 0;

  // Stats
  private txFrameCount = 0;
  private rxFrameCount = 0;
  private arbLostCount = 0;

  // ── SPI interface callback ────────────────────────────────────────────

  /** Callback for when the transceiver wants to assert/deassert INT */
  onInterrupt: ((active: boolean) => void) | null = null;

  constructor(
    private bus: BusWorkerHandle,
    nominalBitRate: number = 500_000,
    dataBitRate: number = 2_000_000
  ) {
    this.canProtocol = new CANFDBusProtocol(nominalBitRate, dataBitRate);
  }

  // ── SPI Command Interface ─────────────────────────────────────────────

  /**
   * Handle an SPI write transaction.
   * Called when the RP2040 firmware writes to a register.
   *
   * @param addr Register address
   * @param value 16-bit value
   */
  spiWrite(addr: number, value: number): void {
    switch (addr) {
      case REG_TX_ID:
        this.txId = value;
        break;

      case REG_TX_DLC:
        this.txDlc = value & 0x0f;
        this.txFdf = !!(value & TX_DLC_FDF);
        this.txBrs = !!(value & TX_DLC_BRS);
        this.txDataBuffer = [];
        this.txDataIndex = 0;
        break;

      case REG_TX_DATA:
        this.txDataBuffer.push(value & 0xff);
        break;

      case REG_TX_CTRL:
        if (value & TX_CTRL_SEND) {
          this.startTransmission();
        }
        break;

      case REG_FILTER_ID:
        this.filterId = value;
        break;

      case REG_FILTER_MASK:
        this.filterMask = value;
        break;

      case REG_CONFIG:
        // Could reconfigure bit rate, but for now we use constructor values
        break;

      case REG_INT_FLAGS:
        // Write to clear specific flags
        this.intFlags &= ~value;
        this.updateInterruptPin();
        break;
    }
  }

  /**
   * Handle an SPI read transaction.
   * Called when the RP2040 firmware reads from a register.
   *
   * @param addr Register address
   * @returns 16-bit value
   */
  spiRead(addr: number): number {
    switch (addr) {
      case REG_RX_ID:
        if (this.rxFifo.length > 0) {
          return this.rxFifo[0].arbitrationId;
        }
        return 0;

      case REG_RX_DLC: {
        if (this.rxFifo.length > 0) {
          const f = this.rxFifo[0];
          let val = f.dlc;
          if (f.isFD) val |= TX_DLC_FDF;
          if (f.brs) val |= TX_DLC_BRS;
          return val;
        }
        return 0;
      }

      case REG_RX_DATA: {
        if (this.rxFifo.length > 0) {
          const f = this.rxFifo[0];
          const idx = this.txDataIndex; // reuse for RX read index
          this.txDataIndex++;
          if (idx < f.data.length) {
            // If we've read all data bytes, pop the frame
            if (this.txDataIndex >= f.data.length) {
              this.rxFifo.shift();
              this.txDataIndex = 0;
              if (this.rxFifo.length === 0) {
                this.intFlags &= ~INT_RX_AVAILABLE;
                this.updateInterruptPin();
              }
            }
            return f.data[idx];
          }
          return 0;
        }
        return 0;
      }

      case REG_RX_STATUS:
        return this.rxFifo.length;

      case REG_STATUS: {
        let status = 0;
        if (this.txState !== TXState.IDLE) status |= 0x01; // TX busy
        if (this.rxState !== RXState.IDLE) status |= 0x02; // RX active
        if (this.bus.busState === 1) status |= 0x04; // bus busy
        return status;
      }

      case REG_INT_FLAGS:
        return this.intFlags;

      default:
        return 0;
    }
  }

  // ── TX Engine ─────────────────────────────────────────────────────────

  private startTransmission(): void {
    if (this.txState !== TXState.IDLE) return;

    const dataLen = this.txFdf
      ? (CAN_FD_DLC_MAP[this.txDlc] ?? this.txDlc)
      : this.txDlc;

    const frame: CANFrameData = {
      arbitrationId: this.txId,
      dlc: this.txDlc,
      data: new Uint8Array(this.txDataBuffer.slice(0, dataLen)),
      isExtended: this.txId > 0x7ff,
      isRemote: false,
      isFD: this.txFdf,
      brs: this.txBrs,
    };

    const result = this.encoder.encode(frame);
    this.txBitStream = result.bits;
    this.txBrsIndex = result.brsIndex;
    this.txBitIndex = 0;
    this.txState = TXState.WAIT_BUS_FREE;
    this.interframeCount = 0;
  }

  // ── Bus Tick Processing ───────────────────────────────────────────────

  /**
   * Called once per bus tick.
   *
   * This is the main entry point for the transceiver's bit-level
   * state machine. It:
   *   1. Reads the current bus state (merged wires)
   *   2. Processes RX (samples the bus)
   *   3. Processes TX (drives the bus)
   *   4. Checks arbitration
   */
  onBusTick(): void {
    const busCANH = this.bus.readWire(WIRE_CAN_H);
    const busCAN_L = this.bus.readWire(WIRE_CAN_L);
    const busBit = this.canProtocol.wiresToBit(busCANH, busCAN_L);

    // Process RX: sample the bus
    this.processRX(busBit);

    // Process TX: drive the bus
    this.processTX(busBit);
  }

  private processTX(busBit: number): void {
    switch (this.txState) {
      case TXState.IDLE:
        // Nothing to do — make sure we're not driving
        this.bus.release();
        break;

      case TXState.WAIT_BUS_FREE:
        // Wait for the bus to be idle (recessive)
        if (busBit === 1) { // recessive
          this.interframeCount++;
          if (this.interframeCount >= 3) { // 3 recessive bits = bus free
            this.txState = TXState.ARBITRATING;
            this.txBitIndex = 0;
            this.driveTXBit();
          }
        } else {
          this.interframeCount = 0;
        }
        break;

      case TXState.ARBITRATING:
        // During arbitration, check if we lost
        if (this.txBitIndex > 0) {
          const prevBit = this.txBitStream[this.txBitIndex - 1];
          if (this.canProtocol.lostArbitration(prevBit === 1 ? 0 : 1, busBit === 0 ? 1 : 0)) {
            // We drove recessive but bus is dominant → lost arbitration
            this.txState = TXState.IDLE;
            this.bus.release();
            this.intFlags |= INT_ARB_LOST;
            this.arbLostCount++;
            this.updateInterruptPin();
            return;
          }
        }

        // Check if we've passed the arbitration phase
        // For standard CAN: arbitration is ID(11) + RTR + IDE = 13 bits
        // For extended: more bits. For simplicity, after SOF+ID we switch
        // to TRANSMITTING (arbitration is only meaningful for those bits)
        if (this.txBitIndex >= 13) {
          this.txState = TXState.TRANSMITTING;
        }

        this.driveTXBit();
        break;

      case TXState.TRANSMITTING:
        this.driveTXBit();
        break;
    }
  }

  private driveTXBit(): void {
    if (this.txBitIndex >= this.txBitStream.length) {
      // Frame complete
      this.txState = TXState.IDLE;
      this.bus.release();
      this.txFrameCount++;
      this.intFlags |= INT_TX_COMPLETE;
      this.updateInterruptPin();
      return;
    }

    const bit = this.txBitStream[this.txBitIndex];
    // Convert logical CAN bit to wire values
    // bit=0 (dominant) → drive CAN_H=1, CAN_L=1
    // bit=1 (recessive) → release CAN_H=0, CAN_L=0
    const [canh, canl] = this.canProtocol.bitToWires(bit);
    this.bus.driveWire(WIRE_CAN_H, canh);
    this.bus.driveWire(WIRE_CAN_L, canl);

    this.txBitIndex++;
  }

  private processRX(busBit: number): void {
    switch (this.rxState) {
      case RXState.IDLE:
        // Look for SOF (dominant bit after bus idle)
        if (busBit === 0 && this.txState === TXState.IDLE) {
          // Only receive if we're not transmitting
          this.rxState = RXState.RECEIVING;
          this.rxBitBuffer = [0]; // SOF
          this.rxConsecutiveSame = 1;
          this.rxLastBit = 0;
        }
        break;

      case RXState.RECEIVING:
        this.rxBitBuffer.push(busBit);

        // Track consecutive same bits for stuff-bit detection
        if (busBit === this.rxLastBit) {
          this.rxConsecutiveSame++;
        } else {
          this.rxConsecutiveSame = 1;
        }
        this.rxLastBit = busBit;

        // Detect end of frame: 7 consecutive recessive bits (EOF)
        if (busBit === 1 && this.rxConsecutiveSame >= 7) {
          this.completeRX();
        }

        // Safety: prevent infinite accumulation
        if (this.rxBitBuffer.length > 800) {
          this.rxState = RXState.IDLE;
          this.rxBitBuffer = [];
        }
        break;
    }
  }

  private completeRX(): void {
    this.rxState = RXState.IDLE;

    try {
      const frame = this.decodeRXBits(this.rxBitBuffer);
      if (!frame) return;

      // Apply acceptance filter
      if (this.filterMask !== 0) {
        if ((frame.arbitrationId & this.filterMask) !== (this.filterId & this.filterMask)) {
          return; // filtered out
        }
      }

      // Push to RX FIFO
      if (this.rxFifo.length < this.rxFifoMaxSize) {
        this.rxFifo.push(frame);
        this.rxFrameCount++;
        this.intFlags |= INT_RX_AVAILABLE;
        this.updateInterruptPin();
      }
    } catch {
      // Decode error — ignore corrupted frame
    }

    this.rxBitBuffer = [];
  }

  /**
   * Decode a received bit stream into a frame.
   * Removes bit stuffing and parses the CAN/CAN FD structure.
   */
  private decodeRXBits(bits: number[]): ReceivedFrame | null {
    // Remove EOF and IFS from end
    // Find where EOF starts (7 recessive bits)
    let eofStart = bits.length;
    let consecRecessive = 0;
    for (let i = bits.length - 1; i >= 0; i--) {
      if (bits[i] === 1) {
        consecRecessive++;
      } else {
        break;
      }
    }
    eofStart = bits.length - consecRecessive;

    // Everything before EOF + ACK + CRC_DEL (3 bits before EOF)
    const frameBits = bits.slice(0, Math.max(0, eofStart));

    // Remove stuff bits
    const unstuffed = this.encoder.removeBitStuffing(frameBits);

    if (unstuffed.length < 20) return null;

    let idx = 0;

    // SOF
    idx++; // skip SOF

    // Read 11-bit ID
    const id11 = this.encoder.bitsToInt(unstuffed.slice(idx, idx + 11));
    idx += 11;

    // RTR/SRR
    const rtrOrSrr = unstuffed[idx];
    idx++;

    // IDE
    const ide = unstuffed[idx];
    idx++;

    let arbitrationId: number;
    let isExtended = false;

    if (ide === 1) {
      // Extended frame
      isExtended = true;
      const extId = this.encoder.bitsToInt(unstuffed.slice(idx, idx + 18));
      idx += 18;
      arbitrationId = (id11 << 18) | extId;
      idx++; // RTR
      idx++; // r1
      idx++; // r0 or FDF check position
    } else {
      arbitrationId = id11;
      // r0 — could be FDF bit for CAN FD
      idx++; // r0
    }

    // Check for FDF bit
    let isFD = false;
    let brs = false;

    // In CAN FD, the bit after r0 is FDF (=1 for FD)
    if (idx < unstuffed.length && unstuffed[idx - 1] === 0) {
      // Look at FDF position
      if (idx < unstuffed.length) {
        const fdfBit = unstuffed[idx];
        if (fdfBit === 1) {
          isFD = true;
          idx++; // FDF
          idx++; // res
          brs = unstuffed[idx] === 1;
          idx++; // BRS
          idx++; // ESI
        }
      }
    }

    // DLC
    if (idx + 4 > unstuffed.length) return null;
    const dlc = this.encoder.bitsToInt(unstuffed.slice(idx, idx + 4));
    idx += 4;

    // Data bytes
    const dataLen = isFD
      ? (CAN_FD_DLC_MAP[dlc] ?? dlc)
      : Math.min(dlc, 8);
    const data = new Uint8Array(dataLen);

    for (let b = 0; b < dataLen; b++) {
      if (idx + 8 > unstuffed.length) break;
      data[b] = this.encoder.bitsToInt(unstuffed.slice(idx, idx + 8));
      idx += 8;
    }

    return {
      arbitrationId,
      dlc,
      data,
      isFD,
      brs,
    };
  }

  // ── Interrupt Management ──────────────────────────────────────────────

  private updateInterruptPin(): void {
    const active = this.intFlags !== 0;
    if (this.onInterrupt) {
      this.onInterrupt(active);
    }
  }

  // ── Getters ───────────────────────────────────────────────────────────

  get txCount(): number { return this.txFrameCount; }
  get rxCount(): number { return this.rxFrameCount; }
  get arbitrationLostCount(): number { return this.arbLostCount; }
  get pendingRX(): number { return this.rxFifo.length; }
  get isTXBusy(): boolean { return this.txState !== TXState.IDLE; }
  get isRXActive(): boolean { return this.rxState !== RXState.IDLE; }
  get currentTXState(): TXState { return this.txState; }
  get currentRXState(): RXState { return this.rxState; }
  get interruptFlags(): number { return this.intFlags; }
}
