# firmware/engine_ecu/main.py

"""
Engine ECU
==========
Handles powertrain control and reporting.
CAN IDs match the CARLA vehicle simulation definitions.

Receives (commands from chassis):
  0x02F  Throttle pedal command     (2 bytes, 0-1023)
  0x01A  Brake command              (2 bytes, 0-1023)
  0x06D  Gear shift command         (1 byte)
  0x1B8  Engine start button        (1 byte, 0-1)
  0x1C9  Parking brake command      (1 byte, 0-1)

Transmits (reports to chassis):
  0x039  Throttle position          (2 bytes, 0-1023)
  0x043  Engine RPM                 (4 bytes)
  0x024  Brake output               (2 bytes, 0-1023)
  0x077  Gear position              (1 byte)
  0x19A  Engine status              (1 byte, 0=off 1=on)
  0x1D3  Parking brake status       (1 byte)
  0x183  Engine coolant temp        (1 byte, 0-255)
  0x18D  Engine malfunction         (1 byte, bitmask)
  0x3D4  Fuel amount                (1 byte, 0-40 litres)
  0x3DE  Battery warning            (1 byte, 0-1)
"""

from machine import CAN, Pin, Timer
import time

# ── Constants ─────────────────────────────────────────────────────────────────

MAX_RPM       = 8000
IDLE_RPM      = 800
REDLINE_RPM   = 7500
BROADCAST_MS  = 10     # 10ms period matches CARLA definition

# ── Hardware ──────────────────────────────────────────────────────────────────

status_led = Pin(25, Pin.OUT)
fault_led  = Pin(15, Pin.OUT)

can = CAN(0, baudrate=500_000)

# Receive commands from chassis
can.setfilter(0, CAN.LIST16, 0, (0x02F, 0x01A, 0x06D, 0x1B8, 0x1C9))

# ── State ─────────────────────────────────────────────────────────────────────

throttle_cmd    = 0      # 0-1023 from chassis
brake_cmd       = 0      # 0-1023 from chassis
gear_cmd        = 0      # gear position
engine_running  = 0      # 0=off 1=on
parking_brake   = 0      # 0=off 1=on

engine_rpm      = 0
coolant_temp    = 80     # degrees C
fuel_level      = 40    # litres
battery_ok      = 1
malfunction     = 0


# ── Engine model ──────────────────────────────────────────────────────────────

def simulate_engine():
    global engine_rpm, coolant_temp, fuel_level, battery_ok

    if not engine_running:
        engine_rpm = max(0, engine_rpm - 100)
        return

    throttle_pct = throttle_cmd / 1023.0
    brake_pct    = brake_cmd    / 1023.0

    if gear_cmd == 0:
        target_rpm = IDLE_RPM + int(throttle_pct * 1500)
    else:
        target_rpm = IDLE_RPM + int(throttle_pct * (MAX_RPM - IDLE_RPM))

    target_rpm  = max(IDLE_RPM, target_rpm - int(brake_pct * 3000))
    rpm_delta   = target_rpm - engine_rpm
    engine_rpm += int(rpm_delta * 0.15)
    engine_rpm  = max(0, min(MAX_RPM, engine_rpm))

    # Temperature rises with load
    target_temp  = 80 + int((engine_rpm / MAX_RPM) * 40)
    coolant_temp += int((target_temp - coolant_temp) * 0.01)

    # Fuel consumption
    if engine_running:
        fuel_level = max(0, fuel_level - 0.00001 * (1 + throttle_pct))

    status_led.value(engine_running)
    fault_led.value(1 if malfunction else 0)


# ── CAN receive ───────────────────────────────────────────────────────────────

def process_frame(arb_id, data):
    global throttle_cmd, brake_cmd, gear_cmd
    global engine_running, parking_brake

    if arb_id == 0x02F:
        throttle_cmd = int.from_bytes(data[0:2], 'big')
        print(f"[ENGINE] Throttle cmd: {throttle_cmd}")

    elif arb_id == 0x01A:
        brake_cmd = int.from_bytes(data[0:2], 'big')
        print(f"[ENGINE] Brake cmd: {brake_cmd}")

    elif arb_id == 0x06D:
        gear_cmd = data[0]
        print(f"[ENGINE] Gear cmd: {gear_cmd}")

    elif arb_id == 0x1B8:
        engine_running = data[0]
        print(f"[ENGINE] Engine {'START' if engine_running else 'STOP'}")

    elif arb_id == 0x1C9:
        parking_brake = data[0]
        print(f"[ENGINE] Parking brake: {'ON' if parking_brake else 'OFF'}")


# ── CAN transmit ──────────────────────────────────────────────────────────────

def broadcast(timer):
    simulate_engine()

    # 0x039 Throttle position
    can.send(throttle_cmd.to_bytes(2, 'big'), 0x039)

    # 0x043 Engine RPM (4 bytes)
    can.send(engine_rpm.to_bytes(4, 'big'), 0x043)

    # 0x024 Brake output
    can.send(brake_cmd.to_bytes(2, 'big'), 0x024)

    # 0x077 Gear position
    can.send(bytes([gear_cmd]), 0x077)

    # 0x19A Engine status
    can.send(bytes([engine_running]), 0x19A)

    # 0x1D3 Parking brake status
    can.send(bytes([parking_brake]), 0x1D3)

    # 0x183 Coolant temperature
    can.send(bytes([min(255, coolant_temp)]), 0x183)

    # 0x18D Engine malfunction
    can.send(bytes([malfunction]), 0x18D)

    # 0x3D4 Fuel level (every 500ms - only send every 50th tick)
    if broadcast.tick % 50 == 0:
        can.send(bytes([int(fuel_level)]), 0x3D4)

    # 0x3DE Battery warning
    if broadcast.tick % 50 == 0:
        can.send(bytes([0 if battery_ok else 1]), 0x3DE)

    broadcast.tick += 1

    print(
        f"[ENGINE] RPM={engine_rpm:5d}  "
        f"Throttle={throttle_cmd:4d}  "
        f"Brake={brake_cmd:4d}  "
        f"Gear={gear_cmd}  "
        f"Temp={coolant_temp}C"
    )


broadcast.tick = 0

# ── Main ──────────────────────────────────────────────────────────────────────

print("[ENGINE ECU] Starting...")

timer = Timer(-1)
timer.init(period=BROADCAST_MS, mode=Timer.PERIODIC, callback=broadcast)

print("[ENGINE ECU] Running.")

while True:
    if can.any():
        arb_id, rtr, fdf, data = can.recv()
        process_frame(arb_id, bytes(data))
    time.sleep_ms(1)