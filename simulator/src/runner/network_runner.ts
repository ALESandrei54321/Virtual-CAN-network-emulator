// simulator/src/runner/network_runner.ts

/**
 * Network Runner
 *
 * Spawns ECU workers, manages the shared bus, and drives the tick clock.
 */

import { Worker } from 'worker_threads';
import { resolve, join, dirname as pathDirname } from 'path';
import { fileURLToPath } from 'url';
import { execSync } from 'child_process';
import * as net from 'net';
import { createWriteStream } from 'fs';
import {
  CANFDBusProtocol,
  BusController,
} from '../bus/index.js';
import { ECUWorkerData } from './ecu_worker.js';

const __filename = fileURLToPath(import.meta.url);
const __dirname = pathDirname(__filename);

interface ECUNode {
  name: string;
  firmwareDir: string;
}

const ECUs: ECUNode[] = [
  { name: 'Gateway', firmwareDir: 'gateway_ecu' },
  { name: 'Engine', firmwareDir: 'engine_ecu' },
  { name: 'Chassis', firmwareDir: 'chassis_ecu' },
  { name: 'Body', firmwareDir: 'body_ecu' },
];

async function main() {
  console.log('--- Virtual CAN Network Simulator ---');

  const baseDir = resolve(__dirname, '../../../');
  const logStream = createWriteStream(join(baseDir, 'ecu_logs.log'), { flags: 'w' });
  const firmwareBase = join(baseDir, 'firmware');
  const workerPath = resolve(__dirname, 'ecu_worker.js'); // Assuming compiled to dist/runner/ecu_worker.js

  // 1. Build LittleFS images for each ECU
  console.log('[Runner] Building LittleFS firmware images...');
  const buildScript = join(baseDir, 'simulator', 'build_littlefs.py');
  
  for (const ecu of ECUs) {
    const srcDir = join(firmwareBase, ecu.firmwareDir);
    // Include lib files by copying them to a temp dir first, or using littlefs-python ability
    // Actually, simple hack: copy lib into ecu dir before building, or just tell script to include it?
    // We'll copy lib to ecu dir for now.
    execSync(`cp -r ${join(firmwareBase, 'lib')} ${srcDir}/`, { stdio: 'ignore' });
    
    const imgPath = join(firmwareBase, `${ecu.name.toLowerCase()}.img`);
    const pythonPath = join(baseDir, '.venv', 'bin', 'python');
    execSync(`${pythonPath} ${buildScript} ${srcDir} ${imgPath}`);
    console.log(`  -> ${imgPath}`);
  }

  // 2. Create the physical bus
  const proto = new CANFDBusProtocol(500_000, 2_000_000);
  const controller = new BusController(proto, ECUs.length);
  controller.fastForward = false; // Must be false so MCU steps execute

  // 3. Launch Workers
  console.log(`[Runner] Spawning ${ECUs.length} ECU workers...`);
  const workers: Worker[] = [];
  let readyCount = 0;

  const dashboardState: Record<string, any> = {};
  for (const ecu of ECUs) {
    dashboardState[ecu.name] = { totalTx: 0, totalRx: 0, txMap: new Map(), rxMap: new Map() };
  }

  const logQueue: string[] = [];

  // TCP Server for CARLA client
  const carlaSockets = new Set<net.Socket>();
  const tcpServer = net.createServer((socket) => {
    logQueue.push('[Runner] CARLA client connected.');
    if (logQueue.length > 10) logQueue.shift();
    carlaSockets.add(socket);

    socket.on('data', (data) => {
      // Forward telemetry data to Gateway worker
      const line = data.toString();
      const gatewayWorker = workers[0]; // Gateway is ECUs[0]
      if (gatewayWorker) {
        gatewayWorker.postMessage({
          type: 'carla_telemetry',
          line: line
        });
      }
    });

    socket.on('close', () => {
      logQueue.push('[Runner] CARLA client disconnected.');
      if (logQueue.length > 10) logQueue.shift();
      carlaSockets.delete(socket);
    });

    socket.on('error', (err) => {
      logQueue.push(`[Runner] CARLA socket error: ${err.message}`);
      if (logQueue.length > 10) logQueue.shift();
      carlaSockets.delete(socket);
    });
  });

  tcpServer.listen(5555, '127.0.0.1', () => {
    logQueue.push('[Runner] TCP Server listening on 127.0.0.1:5555 (CARLA proxy)');
    if (logQueue.length > 10) logQueue.shift();
  });

  await new Promise<void>((resolvePromise) => {
    ECUs.forEach((ecu, idx) => {
      const imgPath = join(firmwareBase, `${ecu.name.toLowerCase()}.img`);
      
      const workerData: ECUWorkerData = {
        nodeIndex: idx,
        ecuName: ecu.name,
        firmwareImage: imgPath,
        busBuffer: controller.buffer,
      };

      const worker = new Worker(workerPath, { workerData });
      
      worker.on('message', (msg) => {
        if (msg.type === 'log') {
          // Clean ANSI color codes from msg for internal log
          const cleanMsg = msg.msg.replace(/\x1B\[[0-9;]*[a-zA-Z]/g, '').trim();
          if (cleanMsg) {
            logStream.write(`[${new Date().toISOString()}] [${msg.ecu}] ${cleanMsg}\n`);
            logQueue.push(`[${msg.ecu}] ${cleanMsg}`);
            if (logQueue.length > 15) logQueue.shift();
          }
        } else if (msg.type === 'can_packet') {
          const state = dashboardState[msg.ecu];
          const map = msg.dir === 'TX' ? state.txMap : state.rxMap;
          if (msg.dir === 'TX') state.totalTx++; else state.totalRx++;
          
          if (!map.has(msg.id)) {
            map.set(msg.id, { count: 0, data: msg.data });
          }
          const p = map.get(msg.id);
          p.count++;
          p.data = msg.data;
        } else if (msg.type === 'gateway_response') {
          // Forward this back to the CARLA TCP client
          for (const socket of carlaSockets) {
            socket.write(msg.line + '\n');
          }
        } else if (msg.type === 'ready') {
          readyCount++;
          if (readyCount === ECUs.length) {
            resolvePromise();
          }
        }
      });

      worker.on('error', (err) => {
        console.error(`[Worker ${ecu.name}] Error:`, err);
      });

      worker.on('exit', (code) => {
        console.log(`[Worker ${ecu.name}] Exited with code ${code}`);
      });

      workers.push(worker);
    });
  });

  console.log('[Runner] All ECUs ready. Starting bus clock...');

  // 4. Run the bus tick loop asynchronously
  // Clear screen once
  process.stdout.write('\x1B[2J\x1B[H');
  
  setInterval(() => {
    // Move cursor to top left
    process.stdout.write('\x1B[H');
    
    const stats = controller.stats;
    let out = '';
    out += '================================================================================\n';
    out += '                           VIRTUAL CAN NETWORK DASHBOARD                        \n';
    out += '================================================================================\n';
    out += `Bus Stats | Ticks: ${stats.totalTicks.toLocaleString()} | Sim/Wall: ${stats.wallClockRatio.toFixed(2)}x | Effective Rate: ${stats.effectiveBitRate.toFixed(0)} bps\n`;
    out += '--------------------------------------------------------------------------------\n';
    
    for (const ecu of ECUs) {
      const state = dashboardState[ecu.name];
      out += `\x1B[1;36m[${ecu.name.toUpperCase()}]\x1B[0m Total TX: ${state.totalTx} | Total RX: ${state.totalRx}\n`;
      
      // Sort keys
      const txKeys = Array.from(state.txMap.keys()).sort();
      const rxKeys = Array.from(state.rxMap.keys()).sort();
      const maxRows = Math.max(txKeys.length, rxKeys.length);
      
      for (let i = 0; i < maxRows; i++) {
        let txStr = '                            ';
        let rxStr = '                            ';
        
        if (i < txKeys.length) {
          const k = txKeys[i];
          const p = state.txMap.get(k);
          txStr = `  \x1B[32mTX 0x${k}\x1B[0m: ${p.data.padEnd(8)} (cnt: ${p.count})`.padEnd(40);
        }
        if (i < rxKeys.length) {
          const k = rxKeys[i];
          const p = state.rxMap.get(k);
          rxStr = `  \x1B[33mRX 0x${k}\x1B[0m: ${p.data.padEnd(8)} (cnt: ${p.count})`;
        }
        out += `${txStr} ${rxStr}\n`;
      }
      out += '\n';
    }
    
    out += '----------------------------------- LOGS ---------------------------------------\n';
    for (const logLine of logQueue) {
      // Clear line to end first to handle shorter messages nicely
      out += `\x1B[K${logLine.slice(0, 78)}\n`;
    }
    for (let i = logQueue.length; i < 10; i++) {
      out += '\x1B[K\n';
    }
    out += '================================================================================\n';
    
    process.stdout.write(out);
  }, 100);

  await controller.runAsync(Infinity, 10000);
}

main().catch(err => {
  console.error('[Runner] Fatal Error:', err);
  process.exit(1);
});
