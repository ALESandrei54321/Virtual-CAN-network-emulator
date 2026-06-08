import { readFileSync } from 'fs';

try {
  const log = readFileSync('../ecu_logs.log', 'utf8');
  const lines = log.split('\n');
  let count = 0;
  for (const line of lines) {
    if (line.trim() && !line.includes('DEBUG: instCount') && !line.includes('Loaded 1247 UF2 blocks')) {
      console.log(line);
      count++;
      if (count > 100) {
        console.log('... truncated ...');
        break;
      }
    }
  }
} catch (e) {
  console.error(e);
}
