// simulator/src/chips/mcp2518fd.ts

import { RP2040 } from 'rp2040js';
import { CANTransceiver } from './can_transceiver.js';
import { BusWorkerHandle } from '../bus/physical_bus.js';

export class WokwiMCP2518FD {
  private transceiver: CANTransceiver;
  private csActive = false;
  private spiCommand = 0;
  private spiAddr = 0;
  private byteCount = 0;
  private spiReadData = 0;
  private txWord = 0;

  constructor(
    private mcu: RP2040,
    private bus: BusWorkerHandle,
    private spiIndex: number = 0,
    private csPin: number = 17,
    private intPin: number = 20
  ) {
    this.transceiver = new CANTransceiver(bus);

    // Bind Interrupt Pin
    this.transceiver.onInterrupt = (active: boolean) => {
      // active = true means INT is asserted (low, since typical INT is active-low)
      this.mcu.gpio[this.intPin].setInputValue(active ? false : true);
    };

    // Bind CS Pin listener
    this.mcu.gpio[this.csPin].addListener((state) => {
      // CS is active low
      if (state === 0) {
        this.csActive = true;
        this.byteCount = 0;
      } else {
        this.csActive = false;
      }
    });

    // Bind SPI onTransmit hook
    const spi = this.mcu.spi[this.spiIndex];
    spi.onTransmit = (value: number) => {
      if (!this.csActive) {
        spi.completeTransmit(0);
        return;
      }

      let rxVal = 0;
      const count = this.byteCount;
      this.byteCount++;

      if (count === 0) {
        this.spiCommand = value;
      } else if (count === 1) {
        this.spiAddr = value;
      } else {
        if (this.spiCommand === 0x03) {
          // Read
          if (count === 2) {
            this.spiReadData = this.transceiver.spiRead(this.spiAddr);
            rxVal = this.spiReadData & 0xFF;
          } else if (count === 3) {
            rxVal = (this.spiReadData >> 8) & 0xFF;
          } else {
            const index = count - 2;
            if (index % 2 === 0) {
              this.spiAddr++;
              this.spiReadData = this.transceiver.spiRead(this.spiAddr);
              rxVal = this.spiReadData & 0xFF;
            } else {
              rxVal = (this.spiReadData >> 8) & 0xFF;
            }
          }
        } else if (this.spiCommand === 0x02) {
          // Write
          const index = count - 2;
          if (index % 2 === 0) {
            if (index > 0) {
              this.spiAddr++;
            }
            this.txWord = value;
          } else {
            this.txWord |= (value << 8);
            this.transceiver.spiWrite(this.spiAddr, this.txWord);
          }
        }
      }
      spi.completeTransmit(rxVal);
    };
  }

  onBusTick(): void {
    this.transceiver.onBusTick();
  }

  getTransceiver(): CANTransceiver {
    return this.transceiver;
  }
}
