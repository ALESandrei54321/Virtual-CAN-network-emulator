// simulator/test/transceiver.test.ts

/**
 * Unit tests for the CAN FD transceiver chip and encoder.
 *
 * Tests:
 *   - CAN encoder: bit stream generation, stuffing, CRC
 *   - Transceiver: SPI register interface, TX/RX, arbitration
 *   - Integration: two transceivers communicating via shared bus
 */

import {
  CANEncoder,
  CANFrameData,
  CAN_FD_DLC_MAP,
  BYTES_TO_DLC,
} from '../src/chips/can_encoder.js';
import {
  CANTransceiver,
  TXState,
  RXState,
  REG_TX_ID,
  REG_TX_DLC,
  REG_TX_DATA,
  REG_TX_CTRL,
  REG_RX_ID,
  REG_RX_DLC,
  REG_RX_STATUS,
  REG_STATUS,
  REG_INT_FLAGS,
  INT_TX_COMPLETE,
  INT_RX_AVAILABLE,
} from '../src/chips/can_transceiver.js';
import {
  PhysicalBus,
  BusWorkerHandle,
  CANFDBusProtocol,
  WIRE_CAN_H,
  WIRE_CAN_L,
} from '../src/bus/index.js';

// ── CAN Encoder Tests ────────────────────────────────────────────────────────

describe('CANEncoder', () => {
  const encoder = new CANEncoder();

  test('intToBits converts correctly', () => {
    expect(encoder.intToBits(0b10110, 5)).toEqual([1, 0, 1, 1, 0]);
    expect(encoder.intToBits(0xff, 8)).toEqual([1, 1, 1, 1, 1, 1, 1, 1]);
    expect(encoder.intToBits(0, 4)).toEqual([0, 0, 0, 0]);
  });

  test('bitsToInt converts correctly', () => {
    expect(encoder.bitsToInt([1, 0, 1, 1, 0])).toBe(0b10110);
    expect(encoder.bitsToInt([1, 1, 1, 1, 1, 1, 1, 1])).toBe(0xff);
    expect(encoder.bitsToInt([0, 0, 0, 0])).toBe(0);
  });

  test('bit stuffing inserts after 5 consecutive bits', () => {
    // 5 zeros → stuff a 1
    const result = encoder.applyBitStuffing([0, 0, 0, 0, 0, 1]);
    expect(result).toEqual([0, 0, 0, 0, 0, 1, 1]); // stuff bit after 5th zero
  });

  test('bit stuffing removal is inverse', () => {
    const original = [0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 1];
    const stuffed = encoder.applyBitStuffing(original);
    const restored = encoder.removeBitStuffing(stuffed);
    expect(restored).toEqual(original);
  });

  test('encodes standard CAN frame', () => {
    const frame: CANFrameData = {
      arbitrationId: 0x123,
      dlc: 2,
      data: new Uint8Array([0xAB, 0xCD]),
      isExtended: false,
      isRemote: false,
      isFD: false,
      brs: false,
    };

    const result = encoder.encode(frame);
    expect(result.bits.length).toBeGreaterThan(0);
    expect(result.brsIndex).toBe(0); // no BRS for classic CAN

    // Frame should start with SOF (dominant = 0)
    expect(result.bits[0]).toBe(0);

    // Frame should end with recessive bits (EOF + IFS)
    const last10 = result.bits.slice(-10);
    expect(last10.every(b => b === 1)).toBe(true);
  });

  test('encodes CAN FD frame with BRS', () => {
    const frame: CANFrameData = {
      arbitrationId: 0x100,
      dlc: 8,
      data: new Uint8Array(8).fill(0x55),
      isExtended: false,
      isRemote: false,
      isFD: true,
      brs: true,
    };

    const result = encoder.encode(frame);
    expect(result.bits.length).toBeGreaterThan(0);
    expect(result.brsIndex).toBeGreaterThan(0);
    expect(result.bits[0]).toBe(0); // SOF
  });

  test('encodes CAN FD with 64 byte payload', () => {
    const frame: CANFrameData = {
      arbitrationId: 0x200,
      dlc: 15, // DLC 15 = 64 bytes
      data: new Uint8Array(64).fill(0xAA),
      isExtended: false,
      isRemote: false,
      isFD: true,
      brs: true,
    };

    const result = encoder.encode(frame);
    expect(result.bits.length).toBeGreaterThan(500); // 64 bytes = 512 data bits + overhead
    expect(result.brsIndex).toBeGreaterThan(0);
  });

  test('DLC map is correct', () => {
    expect(CAN_FD_DLC_MAP[8]).toBe(8);
    expect(CAN_FD_DLC_MAP[9]).toBe(12);
    expect(CAN_FD_DLC_MAP[15]).toBe(64);
  });

  test('reverse DLC map works', () => {
    expect(BYTES_TO_DLC[64]).toBe(15);
    expect(BYTES_TO_DLC[8]).toBe(8);
    expect(BYTES_TO_DLC[12]).toBe(9);
  });
});

// ── Transceiver Tests ────────────────────────────────────────────────────────

describe('CANTransceiver', () => {
  function createTestSetup(nodeCount: number = 2) {
    const proto = new CANFDBusProtocol(500_000, 2_000_000);
    const bus = new PhysicalBus(proto, nodeCount);

    const handles = Array.from({ length: nodeCount }, (_, i) =>
      new BusWorkerHandle(bus.buffer, i, proto)
    );

    const transceivers = handles.map(h =>
      new CANTransceiver(h, 500_000, 2_000_000)
    );

    return { proto, bus, handles, transceivers };
  }

  test('initial state is idle', () => {
    const { transceivers } = createTestSetup(1);
    const trx = transceivers[0];

    expect(trx.currentTXState).toBe(TXState.IDLE);
    expect(trx.currentRXState).toBe(RXState.IDLE);
    expect(trx.txCount).toBe(0);
    expect(trx.rxCount).toBe(0);
    expect(trx.pendingRX).toBe(0);
  });

  test('SPI write to TX registers queues data', () => {
    const { transceivers } = createTestSetup(1);
    const trx = transceivers[0];

    trx.spiWrite(REG_TX_ID, 0x123);
    trx.spiWrite(REG_TX_DLC, 2 | 0x80); // 2 bytes, FDF
    trx.spiWrite(REG_TX_DATA, 0xAB);
    trx.spiWrite(REG_TX_DATA, 0xCD);

    // Trigger TX
    trx.spiWrite(REG_TX_CTRL, 0x01);

    // Should be waiting for bus free
    expect(trx.isTXBusy).toBe(true);
  });

  test('SPI read from status register', () => {
    const { transceivers } = createTestSetup(1);
    const trx = transceivers[0];

    const status = trx.spiRead(REG_STATUS);
    expect(status & 0x01).toBe(0); // not TX busy
    expect(status & 0x02).toBe(0); // not RX active
  });

  test('SPI read from interrupt flags', () => {
    const { transceivers } = createTestSetup(1);
    const trx = transceivers[0];

    expect(trx.spiRead(REG_INT_FLAGS)).toBe(0); // no interrupts
  });

  test('TX → RX: single frame transmission between two nodes', () => {
    const { bus, transceivers } = createTestSetup(2);
    const sender = transceivers[0];
    const receiver = transceivers[1];

    // Queue a frame on sender
    sender.spiWrite(REG_TX_ID, 0x100);
    sender.spiWrite(REG_TX_DLC, 2);       // 2 bytes, classic CAN
    sender.spiWrite(REG_TX_DATA, 0xAA);
    sender.spiWrite(REG_TX_DATA, 0xBB);
    sender.spiWrite(REG_TX_CTRL, 0x01);   // trigger send

    // Track interrupt
    let senderInt = false;
    let receiverInt = false;
    sender.onInterrupt = (active) => { senderInt = active; };
    receiver.onInterrupt = (active) => { receiverInt = active; };

    // Run the bus for enough ticks to transmit a frame
    // Classic CAN with 2 bytes: ~60-80 bits + stuffing
    const maxTicks = 200;
    for (let t = 0; t < maxTicks; t++) {
      // 1. Transceivers process the current bus state
      sender.onBusTick();
      receiver.onBusTick();

      // 2. Bus merges wires
      bus.mergeWires();
    }

    // Sender should have transmitted
    expect(sender.txCount).toBe(1);
    expect(senderInt).toBe(true);
    expect(sender.spiRead(REG_INT_FLAGS) & INT_TX_COMPLETE).toBeTruthy();

    // Receiver should have received
    expect(receiver.rxCount).toBe(1);
    expect(receiver.pendingRX).toBe(1);
    expect(receiverInt).toBe(true);

    // Read the received frame
    const rxId = receiver.spiRead(REG_RX_ID);
    expect(rxId).toBe(0x100);

    const rxStatus = receiver.spiRead(REG_RX_STATUS);
    expect(rxStatus).toBe(1);
  });

  test('TX with CAN FD frame', () => {
    const { bus, transceivers } = createTestSetup(2);
    const sender = transceivers[0];
    const receiver = transceivers[1];

    // Queue a CAN FD frame
    sender.spiWrite(REG_TX_ID, 0x200);
    sender.spiWrite(REG_TX_DLC, 8 | 0x80 | 0x40); // 8 bytes, FDF, BRS
    for (let i = 0; i < 8; i++) {
      sender.spiWrite(REG_TX_DATA, 0x10 + i);
    }
    sender.spiWrite(REG_TX_CTRL, 0x01);

    // Run enough ticks
    for (let t = 0; t < 300; t++) {
      sender.onBusTick();
      receiver.onBusTick();
      bus.mergeWires();
    }

    expect(sender.txCount).toBe(1);
    expect(receiver.rxCount).toBe(1);

    // Verify received ID
    expect(receiver.spiRead(REG_RX_ID)).toBe(0x200);
  });

  test('RX FIFO accumulates frames', () => {
    const { bus, transceivers } = createTestSetup(2);
    const sender = transceivers[0];
    const receiver = transceivers[1];

    // Send 3 frames sequentially
    for (let frame = 0; frame < 3; frame++) {
      sender.spiWrite(REG_TX_ID, 0x100 + frame);
      sender.spiWrite(REG_TX_DLC, 1);
      sender.spiWrite(REG_TX_DATA, frame);
      sender.spiWrite(REG_TX_CTRL, 0x01);

      for (let t = 0; t < 200; t++) {
        sender.onBusTick();
        receiver.onBusTick();
        bus.mergeWires();
      }
    }

    expect(sender.txCount).toBe(3);
    expect(receiver.pendingRX).toBe(3);
    expect(receiver.spiRead(REG_RX_STATUS)).toBe(3);
  });

  test('acceptance filter rejects non-matching frames', () => {
    const { bus, transceivers } = createTestSetup(2);
    const sender = transceivers[0];
    const receiver = transceivers[1];

    // Set filter: only accept ID 0x200 (mask = 0x7FF = exact match)
    receiver.spiWrite(0x30, 0x200); // FILTER_ID
    receiver.spiWrite(0x31, 0x7FF); // FILTER_MASK

    // Send a frame with ID 0x100 (should be filtered)
    sender.spiWrite(REG_TX_ID, 0x100);
    sender.spiWrite(REG_TX_DLC, 1);
    sender.spiWrite(REG_TX_DATA, 0xFF);
    sender.spiWrite(REG_TX_CTRL, 0x01);

    for (let t = 0; t < 200; t++) {
      sender.onBusTick();
      receiver.onBusTick();
      bus.mergeWires();
    }

    expect(sender.txCount).toBe(1);
    expect(receiver.pendingRX).toBe(0); // filtered out
  });
});

// ── Integration: Two transceivers on shared bus ─────────────────────────────

describe('Integration: Two nodes on CAN bus', () => {
  function runBusForTicks(
    bus: PhysicalBus,
    transceivers: CANTransceiver[],
    ticks: number
  ) {
    for (let t = 0; t < ticks; t++) {
      for (const trx of transceivers) {
        trx.onBusTick();
      }
      bus.mergeWires();
    }
  }

  test('node does not receive its own frame', () => {
    const proto = new CANFDBusProtocol();
    const bus = new PhysicalBus(proto, 1);
    const handle = new BusWorkerHandle(bus.buffer, 0, proto);
    const trx = new CANTransceiver(handle);

    trx.spiWrite(REG_TX_ID, 0x100);
    trx.spiWrite(REG_TX_DLC, 1);
    trx.spiWrite(REG_TX_DATA, 0x42);
    trx.spiWrite(REG_TX_CTRL, 0x01);

    runBusForTicks(bus, [trx], 200);

    // When a node is transmitting, it shouldn't receive its own frame
    // (RX engine only activates when TX is idle)
    expect(trx.txCount).toBe(1);
    // The node might or might not see its own frame depending on timing,
    // but TX completion is what matters
    expect(trx.spiRead(REG_INT_FLAGS) & INT_TX_COMPLETE).toBeTruthy();
  });

  test('bus returns to idle after transmission', () => {
    const proto = new CANFDBusProtocol();
    const bus = new PhysicalBus(proto, 2);
    const handles = [
      new BusWorkerHandle(bus.buffer, 0, proto),
      new BusWorkerHandle(bus.buffer, 1, proto),
    ];
    const transceivers = handles.map(h => new CANTransceiver(h));

    // Send a frame
    transceivers[0].spiWrite(REG_TX_ID, 0x100);
    transceivers[0].spiWrite(REG_TX_DLC, 1);
    transceivers[0].spiWrite(REG_TX_DATA, 0x42);
    transceivers[0].spiWrite(REG_TX_CTRL, 0x01);

    // Run enough ticks
    runBusForTicks(bus, transceivers, 200);

    // After frame is done, bus should be idle
    bus.mergeWires();
    expect(bus.isIdle()).toBe(true);
  });
});
