// simulator/test/bus.test.ts

/**
 * Unit tests for the modular bus core.
 *
 * Tests:
 *   - CAN FD protocol: wired-AND, idle detection, arbitration
 *   - Memory layout: SharedArrayBuffer read/write
 *   - PhysicalBus: wire merging, tick advancement
 *   - BusController: tick loop, fast-forward
 */

import {
  CANFDBusProtocol,
  WIRE_CAN_H,
  WIRE_CAN_L,
  BusMemoryView,
  createBusBuffer,
  BUS_SHM_SIZE,
  MAX_WIRES,
  PhysicalBus,
  BusController,
  BusState,
  BitRate,
} from '../src/bus/index.js';

// ── CAN FD Protocol Tests ────────────────────────────────────────────────────

describe('CANFDBusProtocol', () => {
  const proto = new CANFDBusProtocol(500_000, 2_000_000);

  test('has correct wire count and names', () => {
    expect(proto.wireCount).toBe(2);
    expect(proto.wireNames).toEqual(['CAN_H', 'CAN_L']);
    expect(proto.name).toBe('CAN_FD');
  });

  test('nominal and data bit times are correct', () => {
    expect(proto.nominalBitTimeNs).toBe(2000); // 1e9 / 500_000
    expect(proto.dataBitTimeNs).toBe(500);     // 1e9 / 2_000_000
  });

  test('supports BRS', () => {
    expect(proto.supportsBRS).toBe(true);
  });

  describe('mergeWire (wired-AND / dominant wins)', () => {
    test('all recessive → recessive (0)', () => {
      expect(proto.mergeWire([0, 0, 0, 0])).toBe(0);
    });

    test('one dominant → dominant (1)', () => {
      expect(proto.mergeWire([0, 1, 0, 0])).toBe(1);
    });

    test('all dominant → dominant (1)', () => {
      expect(proto.mergeWire([1, 1, 1, 1])).toBe(1);
    });

    test('single node recessive → recessive', () => {
      expect(proto.mergeWire([0])).toBe(0);
    });

    test('single node dominant → dominant', () => {
      expect(proto.mergeWire([1])).toBe(1);
    });

    test('empty → recessive', () => {
      expect(proto.mergeWire([])).toBe(0);
    });
  });

  describe('isBusIdle', () => {
    test('both wires recessive → idle', () => {
      expect(proto.isBusIdle([0, 0])).toBe(true);
    });

    test('CAN_H dominant → not idle', () => {
      expect(proto.isBusIdle([1, 0])).toBe(false);
    });

    test('CAN_L dominant → not idle', () => {
      expect(proto.isBusIdle([0, 1])).toBe(false);
    });

    test('both dominant → not idle', () => {
      expect(proto.isBusIdle([1, 1])).toBe(false);
    });
  });

  describe('bitToWires / wiresToBit', () => {
    test('dominant (0) → both wires driven', () => {
      expect(proto.bitToWires(0)).toEqual([1, 1]);
    });

    test('recessive (1) → both wires released', () => {
      expect(proto.bitToWires(1)).toEqual([0, 0]);
    });

    test('wires driven → dominant bit (0)', () => {
      expect(proto.wiresToBit(1, 1)).toBe(0);
    });

    test('wires released → recessive bit (1)', () => {
      expect(proto.wiresToBit(0, 0)).toBe(1);
    });

    test('partial drive → dominant bit', () => {
      expect(proto.wiresToBit(1, 0)).toBe(0);
      expect(proto.wiresToBit(0, 1)).toBe(0);
    });
  });

  describe('lostArbitration', () => {
    test('drove recessive, bus dominant → lost', () => {
      expect(proto.lostArbitration(0, 1)).toBe(true);
    });

    test('drove dominant, bus dominant → not lost', () => {
      expect(proto.lostArbitration(1, 1)).toBe(false);
    });

    test('drove recessive, bus recessive → not lost', () => {
      expect(proto.lostArbitration(0, 0)).toBe(false);
    });
  });
});

// ── Memory Layout Tests ──────────────────────────────────────────────────────

describe('BusMemoryView', () => {
  test('creates a buffer of correct size', () => {
    const buf = new SharedArrayBuffer(BUS_SHM_SIZE);
    const view = new BusMemoryView(buf);
    expect(view.buffer.byteLength).toBeGreaterThanOrEqual(BUS_SHM_SIZE);
  });

  test('rejects undersized buffers', () => {
    const buf = new SharedArrayBuffer(16);
    expect(() => new BusMemoryView(buf)).toThrow();
  });

  test('control region read/write', () => {
    const buf = new SharedArrayBuffer(BUS_SHM_SIZE);
    const view = new BusMemoryView(buf);

    view.tick = 42;
    expect(view.tick).toBe(42);

    view.busState = BusState.BUSY;
    expect(view.busState).toBe(BusState.BUSY);

    view.currentBitRate = BitRate.DATA;
    expect(view.currentBitRate).toBe(BitRate.DATA);

    view.nodeCount = 4;
    expect(view.nodeCount).toBe(4);

    view.fastForward = true;
    expect(view.fastForward).toBe(true);
    view.fastForward = false;
    expect(view.fastForward).toBe(false);

    view.idleTickCount = 999;
    expect(view.idleTickCount).toBe(999);
  });

  test('merged wire read/write', () => {
    const buf = new SharedArrayBuffer(BUS_SHM_SIZE);
    const view = new BusMemoryView(buf);

    view.setMergedWire(0, 1);
    view.setMergedWire(1, 0);
    expect(view.getMergedWire(0)).toBe(1);
    expect(view.getMergedWire(1)).toBe(0);
  });

  test('per-node wire read/write', () => {
    const buf = new SharedArrayBuffer(BUS_SHM_SIZE);
    const view = new BusMemoryView(buf);

    // Node 0, wire 0
    view.setNodeWire(0, 0, 1);
    expect(view.getNodeWire(0, 0)).toBe(1);

    // Node 3, wire 1
    view.setNodeWire(3, 1, 1);
    expect(view.getNodeWire(3, 1)).toBe(1);

    // Other nodes untouched
    expect(view.getNodeWire(1, 0)).toBe(0);
    expect(view.getNodeWire(2, 1)).toBe(0);
  });

  test('getAllNodeWires aggregates correctly', () => {
    const buf = new SharedArrayBuffer(BUS_SHM_SIZE);
    const view = new BusMemoryView(buf);
    view.nodeCount = 3;

    view.setNodeWire(0, 0, 0);
    view.setNodeWire(1, 0, 1);
    view.setNodeWire(2, 0, 0);

    expect(view.getAllNodeWires(0, 3)).toEqual([0, 1, 0]);
  });

  test('node status read/write', () => {
    const buf = new SharedArrayBuffer(BUS_SHM_SIZE);
    const view = new BusMemoryView(buf);

    view.setNodeStatus(0, 1);
    view.setNodeStatus(5, 3);
    expect(view.getNodeStatus(0)).toBe(1);
    expect(view.getNodeStatus(5)).toBe(3);
    expect(view.getNodeStatus(1)).toBe(0);
  });
});

describe('createBusBuffer', () => {
  test('initializes with correct idle values', () => {
    const buf = createBusBuffer(4, [0, 0]);
    const view = new BusMemoryView(buf);

    expect(view.nodeCount).toBe(4);
    expect(view.getMergedWire(0)).toBe(0);
    expect(view.getMergedWire(1)).toBe(0);

    for (let n = 0; n < 4; n++) {
      expect(view.getNodeWire(n, 0)).toBe(0);
      expect(view.getNodeWire(n, 1)).toBe(0);
    }
  });
});

// ── PhysicalBus Tests ────────────────────────────────────────────────────────

describe('PhysicalBus', () => {
  test('creates with correct node count', () => {
    const proto = new CANFDBusProtocol();
    const bus = new PhysicalBus(proto, 4);
    expect(bus.nodeCount).toBe(4);
  });

  test('rejects too many nodes', () => {
    const proto = new CANFDBusProtocol();
    expect(() => new PhysicalBus(proto, 20)).toThrow();
  });

  test('mergeWires computes wired-AND correctly', () => {
    const proto = new CANFDBusProtocol();
    const bus = new PhysicalBus(proto, 3);

    // All idle → bus idle
    bus.mergeWires();
    expect(bus.mem.getMergedWire(WIRE_CAN_H)).toBe(0);
    expect(bus.isIdle()).toBe(true);

    // Node 1 drives dominant
    bus.mem.setNodeWire(1, WIRE_CAN_H, 1);
    bus.mem.setNodeWire(1, WIRE_CAN_L, 1);
    bus.mergeWires();
    expect(bus.mem.getMergedWire(WIRE_CAN_H)).toBe(1);
    expect(bus.mem.getMergedWire(WIRE_CAN_L)).toBe(1);
    expect(bus.isIdle()).toBe(false);

    // Node 1 releases → back to idle
    bus.mem.setNodeWire(1, WIRE_CAN_H, 0);
    bus.mem.setNodeWire(1, WIRE_CAN_L, 0);
    bus.mergeWires();
    expect(bus.isIdle()).toBe(true);
  });

  test('tick advances the tick counter', () => {
    const proto = new CANFDBusProtocol();
    const bus = new PhysicalBus(proto, 2);

    const t0 = bus.mem.tick;
    bus.tick();
    expect(bus.mem.tick).toBe(t0 + 1);
    bus.tick();
    expect(bus.mem.tick).toBe(t0 + 2);
  });

  test('tick updates bus state', () => {
    const proto = new CANFDBusProtocol();
    const bus = new PhysicalBus(proto, 2);

    // Idle tick
    bus.tick();
    expect(bus.mem.busState).toBe(BusState.IDLE);

    // Drive node 0 dominant
    bus.mem.setNodeWire(0, WIRE_CAN_H, 1);
    bus.mem.setNodeWire(0, WIRE_CAN_L, 1);
    bus.tick();
    expect(bus.mem.busState).toBe(BusState.BUSY);
  });

  test('idle tick counter tracks consecutive idle ticks', () => {
    const proto = new CANFDBusProtocol();
    const bus = new PhysicalBus(proto, 2);

    // 5 idle ticks
    for (let i = 0; i < 5; i++) bus.tick();
    expect(bus.mem.idleTickCount).toBe(5);

    // 1 busy tick resets
    bus.mem.setNodeWire(0, WIRE_CAN_H, 1);
    bus.mem.setNodeWire(0, WIRE_CAN_L, 1);
    bus.tick();
    expect(bus.mem.idleTickCount).toBe(0);
  });

  test('releaseNode sets all wires to idle', () => {
    const proto = new CANFDBusProtocol();
    const bus = new PhysicalBus(proto, 2);

    bus.mem.setNodeWire(0, WIRE_CAN_H, 1);
    bus.mem.setNodeWire(0, WIRE_CAN_L, 1);
    bus.releaseNode(0);

    expect(bus.mem.getNodeWire(0, WIRE_CAN_H)).toBe(0);
    expect(bus.mem.getNodeWire(0, WIRE_CAN_L)).toBe(0);
  });

  test('bit rate switching', () => {
    const proto = new CANFDBusProtocol(500_000, 2_000_000);
    const bus = new PhysicalBus(proto, 2);

    expect(bus.currentBitTimeNs()).toBe(2000); // nominal
    bus.switchToDataRate();
    expect(bus.currentBitTimeNs()).toBe(500);  // data
    bus.switchToNominalRate();
    expect(bus.currentBitTimeNs()).toBe(2000);
  });
});

// ── BusController Tests ──────────────────────────────────────────────────────

describe('BusController', () => {
  test('creates with correct protocol', () => {
    const proto = new CANFDBusProtocol();
    const ctrl = new BusController(proto, 2);
    expect(ctrl.bus.nodeCount).toBe(2);
  });

  test('run returns stats', () => {
    const proto = new CANFDBusProtocol();
    const ctrl = new BusController(proto, 0); // 0 nodes = no workers to wait for
    ctrl.fastForward = false;

    const stats = ctrl.run(100);
    expect(stats.totalTicks).toBe(100);
    expect(stats.idleTicks).toBe(100);
    expect(stats.busyTicks).toBe(0);
    expect(stats.elapsedMs).toBeGreaterThan(0);
  });

  test('fast-forward skips idle ticks', () => {
    const proto = new CANFDBusProtocol();
    const ctrl = new BusController(proto, 0);
    ctrl.fastForward = true;

    // Run enough ticks to trigger fast-forward (threshold = 100 idle ticks)
    const stats = ctrl.run(500);
    expect(stats.skippedTicks).toBeGreaterThan(0);
    expect(stats.totalTicks).toBe(500);
  });

  test('stop() halts the tick loop', () => {
    const proto = new CANFDBusProtocol();
    const ctrl = new BusController(proto, 0);
    ctrl.fastForward = false;

    let tickCount = 0;
    ctrl.onTick(() => {
      tickCount++;
      if (tickCount >= 10) ctrl.stop();
    });

    ctrl.run(1_000_000);
    expect(tickCount).toBe(10);
  });

  test('singleTick advances by one', () => {
    const proto = new CANFDBusProtocol();
    const ctrl = new BusController(proto, 0);

    const t0 = ctrl.bus.mem.tick;
    ctrl.singleTick();
    expect(ctrl.bus.mem.tick).toBe(t0 + 1);
  });
});
