import { readFileSync } from 'fs';

const log = readFileSync('../ecu_logs.log', 'utf8');
const lines = log.split('\n');

const pcs = {};
let total = 0;

for (const line of lines) {
  if (line.includes('[Gateway] DEBUG: instCount=')) {
    const match = line.match(/PC=0x([0-9a-fA-F]+)/);
    if (match) {
      const pc = match[1];
      pcs[pc] = (pcs[pc] || 0) + 1;
      total++;
    }
  }
}

console.log(`Total Gateway debug points: ${total}`);
console.log('Unique PCs and their frequency:');
const sorted = Object.entries(pcs).sort((a, b) => b[1] - a[1]);
for (const [pc, count] of sorted.slice(0, 20)) {
  console.log(`  PC 0x${pc}: ${count} times (${((count/total)*100).toFixed(2)}%)`);
}
