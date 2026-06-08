// simulator/src/runner/ecu_worker.ts

/**
 * ECU Worker Thread
 *
 * Runs a single RP2040 instance simulating one ECU.
 *
 * Lifecycle:
 * 1. Read workerData (firmware path, bus SHM, node index)
 * 2. Setup RP2040 and LittleFS
 * 3. Mount CANTransceiver to SPI0
 * 4. Boot MicroPython
 * 5. Hook into the PC loop to advance the bus tick synchronously
 */

import { workerData, parentPort } from 'worker_threads';
import { readFileSync } from 'fs';
import {
  RP2040,
  USBCDC,
  ConsoleLogger,
  LogLevel,
  I2CMode,
} from 'rp2040js';
import { bootromB1 } from './bootrom.js';

import { BusWorkerHandle, CANFDBusProtocol } from '../bus/index.js';
import { WokwiMCP2518FD } from '../chips/mcp2518fd.js';

// ── Worker Data interface ───────────────────────────────────────────────────

export interface ECUWorkerData {
  nodeIndex: number;
  ecuName: string;
  firmwareImage: string; // Path to LittleFS image
  busBuffer: SharedArrayBuffer; // The shared bus
}

const data = workerData as ECUWorkerData;

// ── Bootrom fallback ────────────────────────────────────────────────────────

const BOOTROM_BYTES = new Uint8Array([
  // We need a minimal bootrom if rp2040js expects one and we don't have the real Pico bootrom
  // MicroPython UF2 usually handles its own setup, but rp2040js might want a bootrom file.
  // Actually, we'll load the micropython UF2 which includes everything.
]);

async function main() {
  console.log(`[${data.ecuName}] Starting ECU worker...`);

  // 1. Setup RP2040
  const mcu = new RP2040();
  mcu.loadBootrom(bootromB1);
  mcu.logger = new ConsoleLogger(LogLevel.Error);

  try {
    const uf2 = readFileSync('micropython.uf2');
    let offset = 0;
    let blocksLoaded = 0;
    while (offset < uf2.length) {
      const magicStart0 = uf2.readUInt32LE(offset);
      const targetAddr = uf2.readUInt32LE(offset + 12);
      const payloadSize = uf2.readUInt32LE(offset + 16);

      if (magicStart0 === 0x0A324655) {
        if (targetAddr >= 0x10000000 && targetAddr < 0x11000000) {
          const payload = uf2.subarray(offset + 32, offset + 32 + (payloadSize || 256));
          mcu.flash.set(payload, targetAddr - 0x10000000);
          blocksLoaded++;
        }
      }
      offset += 512;
    }
    if (parentPort) parentPort.postMessage({ type: 'log', ecu: data.ecuName, msg: `Loaded ${blocksLoaded} UF2 blocks.\n` });
  } catch (err) {
    if (parentPort) parentPort.postMessage({ type: 'log', ecu: data.ecuName, msg: `Failed to load micropython.uf2: ${err}\n` });
  }

  // 2. Load LittleFS image at offset 1MB (0x10100000)
  // MicroPython's default LittleFS partition is usually at the end of the flash,
  // but for testing, let's just use the RP2040 flash block.
  // Wait, MicroPython Pico port puts the filesystem at an offset.
  // Let's use wokwi's approach or assume 0x100A0000.
  // We'll write the littlefs image to flash memory.
  try {
    const fsImg = readFileSync(data.firmwareImage);
    // 0x100A0000 is often the LittleFS start in standard Pico build,
    // but the exact offset depends on the uf2. We will guess 0xA0000 (640KB)
    // Actually, RPI_PICO default FS offset is 0x100A0000 or 0x10100000.
    // Let's use 0x100A0000.
    mcu.flash.set(new Uint8Array(fsImg), 0xA0000);
  } catch (err) {
    console.warn(`[${data.ecuName}] No LittleFS image found or failed to load.`);
  }

  // 3. Setup USB CDC (Serial Console)
  const cdc = new USBCDC(mcu.usbCtrl);

  const origOnUSBEnabled = mcu.usbCtrl.onUSBEnabled;
  mcu.usbCtrl.onUSBEnabled = () => {
    if (parentPort) parentPort.postMessage({ type: 'log', ecu: data.ecuName, msg: `[USB DEBUG] onUSBEnabled called\n` });
    origOnUSBEnabled?.();
  };

  const origOnResetReceived = mcu.usbCtrl.onResetReceived;
  mcu.usbCtrl.onResetReceived = () => {
    if (parentPort) parentPort.postMessage({ type: 'log', ecu: data.ecuName, msg: `[USB DEBUG] onResetReceived called\n` });
    origOnResetReceived?.();
  };

  let serialLine = '';

  const processSerialByte = (byte: number) => {
    const char = String.fromCharCode(byte);
    if (char === '\n') {
      const line = serialLine.trim();
      serialLine = '';
      if (line.startsWith('$$CAN_')) {
        const parts = line.split(',');
        if (parts.length >= 3) {
          parentPort?.postMessage({
            type: 'can_packet',
            ecu: data.ecuName,
            dir: parts[0] === '$$CAN_TX' ? 'TX' : 'RX',
            id: parts[1],
            data: parts[2]
          });
        }
      } else if (line.startsWith('{"type":') || line.startsWith('{"status":')) {
        // Direct socket response from gateway_ecu
        parentPort?.postMessage({
          type: 'gateway_response',
          line: line
        });
      } else if (line.length > 0) {
        parentPort?.postMessage({ type: 'log', ecu: data.ecuName, msg: line + '\n' });
      }
    } else if (char !== '\r') {
      serialLine += char;
    }
  };

  cdc.onSerialData = (bytes) => {
    for (let i = 0; i < bytes.length; i++) {
      processSerialByte(bytes[i]);
    }
  };

  cdc.onDeviceConnected = () => {
    if (parentPort) parentPort.postMessage({ type: 'log', ecu: data.ecuName, msg: `[USB DEBUG] cdc.onDeviceConnected called\n` });
  };

  // Listen for message from parent process (CARLA telemetry)
  parentPort?.on('message', (msg) => {
    if (msg.type === 'carla_telemetry') {
      const line = msg.line;
      for (let i = 0; i < line.length; i++) {
        cdc.sendSerialByte(line.charCodeAt(i));
      }
    }
  });

  // 4. Start MCU directly using VTOR = 0x10000100 and PC = 0x10000000
  mcu.core.reset();
  mcu.core.VTOR = 0x10000100;
  mcu.core.PC = 0x10000000;

  // 5. Setup Bus & Wokwi Transceiver Chip
  const proto = new CANFDBusProtocol();
  const busHandle = new BusWorkerHandle(data.busBuffer, data.nodeIndex, proto);
  const transceiver = new WokwiMCP2518FD(mcu, busHandle, 0, 17, 20);

  // 6. Synchronous Execution Loop
  // The RP2040 runs at 125MHz. The CAN bus nominal bit rate is e.g. 500kbps (2000ns).
  // Data phase is 2Mbps (500ns).
  // 1 tick = 1 bit time.
  // If nominal is 2000ns, that's 250 RP2040 cycles (125Mhz = 8ns per cycle).
  // We'll execute MCU instructions, then wait for the bus tick.
  
  let currentBusTick = busHandle.tick;

  // Signal ready to parent port
  if (parentPort) {
    parentPort.postMessage({ type: 'ready', ecu: data.ecuName });
  }

  const runLoop = async () => {
    let instCount = 0;
    while (true) {
      const bitTimeNs = busHandle.currentBitRate === 1 ? proto.dataBitTimeNs : proto.nominalBitTimeNs;
      // 1. Run MCU for one bit time
      const cycleNanos = 1e9 / 125_000_000; // 8ns per cycle
      let elapsedNanos = 0;

      try {
        while (elapsedNanos < bitTimeNs) {
          if (mcu.core.waiting) {
            const nextAlarm = (mcu.clock as any).nanosToNextAlarm;
            let advanceNanos = 0;
            if (nextAlarm > 0) {
              advanceNanos = Math.min(nextAlarm, bitTimeNs - elapsedNanos);
            } else if (nextAlarm === 0) {
              advanceNanos = bitTimeNs - elapsedNanos;
            } else {
              advanceNanos = cycleNanos;
            }
            
            (mcu.clock as any).tick(advanceNanos);
            elapsedNanos += advanceNanos;
            
            if (mcu.core.interruptsUpdated && mcu.core.checkForInterrupts()) {
              mcu.core.waiting = false;
            }
          } else {
            instCount++;
            const cycles = mcu.core.executeInstruction();
            const nanos = cycles * cycleNanos;
            (mcu.clock as any).tick(nanos);
            elapsedNanos += nanos;
          }
        }
      } catch (e) {
        console.error(`[${data.ecuName}] CRASH:`, e);
        if (parentPort) parentPort.postMessage({ type: 'log', ecu: data.ecuName, msg: `CRASH: ${e}\n` });
        // wait a bit for message to flush
        setTimeout(() => process.exit(1), 100);
        return; // break out of while true
      }

      // 2. Transceiver processes bus state
      transceiver.onBusTick();

      // 3. Signal bus controller that we are done
      busHandle.signalReady();

      // 4. Wait for bus controller to merge wires and advance tick
      currentBusTick = busHandle.waitForTick(currentBusTick);

      if (currentBusTick % 50000 === 0) {
        if (parentPort) {
          parentPort.postMessage({
            type: 'log',
            ecu: data.ecuName,
            msg: `[DEBUG] Tick ${currentBusTick} | PC = 0x${mcu.core.PC.toString(16)} | SP = 0x${mcu.core.SP.toString(16)} | waiting = ${mcu.core.waiting}\n`
          });
        }
      }

      if (currentBusTick < 1000000) {
        if (currentBusTick % 1000 === 0) {
          await new Promise(resolve => setTimeout(resolve, 0));
        }
      } else {
        if (currentBusTick % 1000 === 0) {
          await new Promise(resolve => setImmediate(resolve));
        }
      }
    }
  };
  
  runLoop();
}

main();
