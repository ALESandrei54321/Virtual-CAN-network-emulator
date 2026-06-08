// simulator/src/index.ts

/**
 * Virtual CAN Network Simulator
 *
 * Modular vehicle network simulator with bit-level bus emulation.
 */

// Bus core
export {
  IBusProtocol,
  WireMergeStrategy,
  BusState,
  BitRate,
  NodeStatus,
} from './bus/protocol.js';

export {
  BusMemoryView,
  createBusBuffer,
  BUS_SHM_SIZE,
  MAX_WIRES,
  MAX_NODES,
} from './bus/memory_layout.js';

export { PhysicalBus, BusWorkerHandle } from './bus/physical_bus.js';
export { BusController } from './bus/bus_controller.js';
export type { BusStats } from './bus/bus_controller.js';

// Protocol drivers
export { CANFDBusProtocol, WIRE_CAN_H, WIRE_CAN_L } from './bus/protocols/can_fd_bus.js';

// Chips
export { CANEncoder } from './chips/can_encoder.js';
export type { CANFrameData, EncodeResult } from './chips/can_encoder.js';
export { CANTransceiver, TXState, RXState } from './chips/can_transceiver.js';
export type { ReceivedFrame } from './chips/can_transceiver.js';
