import { bootromB1 } from './dist/runner/bootrom.js';

const startAddr = 0x2600;
const endAddr = 0x2660;

for (let addr = startAddr; addr < endAddr; addr += 2) {
  const wordIdx = Math.floor(addr / 4);
  const isHigh = (addr % 4) === 2;
  const word = bootromB1[wordIdx];
  const inst = isHigh ? (word >> 16) & 0xffff : word & 0xffff;
  console.log(`0x${addr.toString(16)}: 0x${inst.toString(16).padStart(4, '0')}`);
}
