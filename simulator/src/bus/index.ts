// simulator/src/bus/index.ts

export {
  IBusProtocol,
  WireMergeStrategy,
  BusState,
  BitRate,
  INodeOutput,
  NodeStatus,
} from './protocol.js';

export {
  BusMemoryView,
  createBusBuffer,
  BUS_SHM_SIZE,
  MAX_WIRES,
  MAX_NODES,
  CTRL_TICK,
  CTRL_BUS_STATE,
  CTRL_BIT_RATE,
  CTRL_NODE_COUNT,
  CTRL_BARRIER,
  CTRL_READY,
  CTRL_FAST_FWD,
  CTRL_IDLE_TICKS,
  MERGED_WIRES_OFFSET,
  NODE_WIRES_OFFSET,
  NODE_STATUS_INT32_OFFSET,
} from './memory_layout.js';

export { PhysicalBus, BusWorkerHandle } from './physical_bus.js';
export { BusController, BusStats } from './bus_controller.js';
export { CANFDBusProtocol, WIRE_CAN_H, WIRE_CAN_L } from './protocols/can_fd_bus.js';
