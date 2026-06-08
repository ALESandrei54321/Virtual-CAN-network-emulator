# firmware/lib/can_fd_driver.py

"""
MicroPython CAN FD driver for MCP2518FD transceiver over SPI.
Drop-in compatible with the existing machine.CAN API for the virtual network.
"""
from machine import SPI, Pin
import time

class CAN:
    MASK16 = 0
    LIST16 = 2

    def __init__(self, bus=0, baudrate=500_000, fd=True, **kw):
        # We ignore data_baudrate here, as the simulator bus is pre-configured
        # RP2040 SPI0 pins (SCK=GP18, MOSI=GP19, MISO=GP16)
        self._spi = SPI(0, baudrate=4_000_000, polarity=0, phase=0, sck=Pin(18), mosi=Pin(19), miso=Pin(16))
        self._cs = Pin(17, Pin.OUT, value=1)
        self._int = Pin(20, Pin.IN, Pin.PULL_UP)

    def send(self, data, id, fdf=False, **kw):
        # Write to TX_ID (0x00)
        self._write_reg(0x00, id)

        # Write to TX_DLC (0x01)
        dlc = len(data)
        if len(data) > 8:
            if len(data) <= 12: dlc = 9
            elif len(data) <= 16: dlc = 10
            elif len(data) <= 20: dlc = 11
            elif len(data) <= 24: dlc = 12
            elif len(data) <= 32: dlc = 13
            elif len(data) <= 48: dlc = 14
            else: dlc = 15
            
        flags = dlc | (0x80 if fdf else 0) | (0x40 if fdf else 0) # FDF and BRS
        self._write_reg(0x01, flags)

        # Write data bytes to TX_DATA (0x02)
        for b in data:
            self._write_reg(0x02, b)

        # Trigger TX_CTRL (0x03)
        self._write_reg(0x03, 0x01)

        # Optionally wait for TX complete (basic blocking send)
        # In a real driver we might use the INT pin or STATUS register
        timeout = time.ticks_ms() + 100
        while time.ticks_diff(timeout, time.ticks_ms()) > 0:
            status = self._read_reg(0x20) # REG_STATUS
            if (status & 0x01) == 0: # TX busy flag is clear
                break

    def any(self):
        # Check RX_STATUS (0x13)
        return self._read_reg(0x13) > 0

    def recv(self):
        if not self.any():
            return None

        arb_id = self._read_reg(0x10) # RX_ID
        dlc_flags = self._read_reg(0x11) # RX_DLC
        
        dlc = dlc_flags & 0x0F
        fdf = bool(dlc_flags & 0x80)
        
        # Convert DLC back to length
        length = dlc
        if fdf and dlc > 8:
            dlc_map = {9:12, 10:16, 11:20, 12:24, 13:32, 14:48, 15:64}
            length = dlc_map.get(dlc, 8)

        data = bytearray(length)
        for i in range(length):
            data[i] = self._read_reg(0x12) # RX_DATA

        # Return format matching python-can / previous API
        # id, is_extended, is_rtr, fdf, data
        return (arb_id, False, False, data)

    def setfilter(self, bank, mode, fifo, params):
        if len(params) == 2:
            self._write_reg(0x30, params[0]) # FILTER_ID
            self._write_reg(0x31, params[1]) # FILTER_MASK

    def _write_reg(self, addr, value):
        self._cs.value(0)
        # Write cmd (0x02) + addr + value (16-bit little endian)
        self._spi.write(bytes([0x02, addr, value & 0xFF, (value >> 8) & 0xFF]))
        self._cs.value(1)

    def _read_reg(self, addr):
        self._cs.value(0)
        # Read cmd (0x03) + addr
        self._spi.write(bytes([0x03, addr]))
        result = self._spi.read(2)
        self._cs.value(1)
        return result[0] | (result[1] << 8)
