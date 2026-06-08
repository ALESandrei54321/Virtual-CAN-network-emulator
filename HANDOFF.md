# Handoff & Project Status Report

This document outlines the current architectural state, what has been implemented, and the specific next steps to complete the Virtual CAN Network simulator.

---

## 1. Project Overview & Current Architecture

The project emulates a **Virtual CAN Network** containing 4 distinct ECUs:
1. **Gateway ECU** (`gateway_ecu`) - Bridges host CARLA telemetry to the emulated CAN bus.
2. **Engine ECU** (`engine_ecu`) - Emulates engine dynamics, throttle/brake outputs, and transmission/gear selection.
3. **Chassis ECU** (`chassis_ecu`) - Emulates steering, braking (ABS), dynamics, and vehicle speed.
4. **Body ECU** (`body_ecu`) - Emulates lighting, wipers, and safety signals (collision, airbag).

### Physical Layer Emulation
*   **Wokwi RP2040 Microcontroller**: Each ECU runs as a Node.js `worker_thread` executing a virtual RP2040 chip via `@wokwi/rp2040` loaded with a custom MicroPython binary.
*   **Shared Bus Layer**: A physical bit-by-bit CAN bus model (`CANFDBusProtocol`) runs in shared memory using a synchronization barrier (`BusWorkerHandle` and `Atomics.wait/notify`).
*   **Custom Transceiver Chip (`WokwiMCP2518FD`)**: A custom-emulated transceiver chip interfaces with each RP2040 over SPI (modeled after the MCP2518FD) and propagates physical bit levels (CAN-H and CAN-L lines) onto the shared bus, allowing multi-node arbitration and collision detection.

### Storage & Mounting (Solved)
*   **Flash Partitioning**: Each RP2040 loads a custom LittleFS filesystem image from flash offset `0xA0000`.
*   **Image Version Alignment**: The Python image builder on the host (`simulator/build_littlefs.py`) is configured with `disk_version=0x00020000` to format images in **LittleFS v2.0**, matching MicroPython's target version. All 4 ECUs now successfully mount their filesystems automatically at boot.

---

## 2. Work Accomplished in the Latest Phase

1.  **Eliminated REPL Interception**: Removed raw REPL injection from `ecu_worker.ts` initialization. The microcontrollers now boot directly into their native loop and execute `/main.py` out of flash automatically.
2.  **Resolved MicroPython Limitations**:
    *   **Function Attributes**: Replaced assignments like `broadcast.tick = 0` (unsupported on MicroPython function objects) with a standard global variable `broadcast_tick`.
    *   **f-string Complex Expressions**: Standardized print formatting in `gateway_ecu/main.py` and `body_ecu/main.py` using standard `%` formatting to prevent syntax errors caused by nested quotes and conditional expressions inside f-string braces.
3.  **Clean Builds**: Recompiled TypeScript source files successfully (`tsc` exits with status `0`).

---

## 3. Current State of the Simulator Run

When you start the simulation via `node dist/runner/network_runner.js`:
*   The host automatically copies `/lib` imports to each ECU folder.
*   It formats and compiles a custom LittleFS image for each ECU using Python.
*   It launches 4 worker threads.
*   The worker threads execute the RP2040 cores synchronously.
*   The TCP server for CARLA telemetry binds to `127.0.0.1:5555` on the host.

---

## 4. Next Steps for the Incoming LLM

To finalize the simulation and tie it together:

1.  **Test Run Verification**:
    *   Start the simulation runner:
        ```bash
        node dist/runner/network_runner.js
        ```
    *   Observe the boot log. Ensure all 4 ECUs successfully print their startup messages (e.g. `[ENGINE ECU] Running.`, `[CHASSIS ECU] Running.`, etc.) without throwing syntax or attribute exceptions.

2.  **Telemetry Data Forwarding**:
    *   Verify the integration between the host TCP server and the Gateway ECU's serial buffer. Telemetry messages received on port `5555` should successfully forward to the Gateway, which parses it via `socket.py` mapping to stdout/stdin, transforming them into CAN frames on the bus.

3.  **Dashboard Display Polish**:
    *   polish the real-time telemetry metrics shown on the terminal dashboard (TX/RX counts, arbitration counts, packet frequency per ECU) to ensure it updates cleanly without excessive flashing or console flooding.
