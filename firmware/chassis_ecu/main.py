# firmware/chassis_ecu/main.py

"""
Chassis ECU
===========
Handles steering, ABS, traction control and vehicle dynamics.
CAN IDs match the CARLA vehicle simulation definitions.

Receives (sensor data and engine reports):
  0x043  Engine RPM              (from Engine ECU)
  0x039  Throttle position       (from Engine ECU)
  0x1D3  Parking brake status    (from Engine ECU)

Transmits (commands to engine + reports):
  0x02F  Throttle pedal command  (2 bytes, 0-1023)  → Engine ECU
  0x01A  Brake command           (2 bytes, 0-1023)  → Engine ECU
  0x06D  Gear shift command      (1 byte)           → Engine ECU
  0x058  Steering wheel position (2 bytes, -511 to 511)
  0x062  Power steering output   (2 bytes, 0-100)
  0x146  Brake oil indicator     (2 bytes)
  0x15A  ABS operation           (1 byte, 0-1)
  0x16F  Throttle adjustment     (2 bytes, kmh)
  0x198  Tire angle              (2 bytes)
  0x1C9  Parking brake command   (1 byte)           → Engine ECU
"""

from machine import CAN, Pin, Timer
import time

# ── Constants ─────────────────────────────────────────────────────────────────

BROADCAST_MS   = 10
TYRE_CIRC_M    = 1.96
ABS_THRESHOLD  = 0.75
TC_THRESHOLD   = 1.25

# ── Hardware ──────────────────────────────────────────────────────────────────

abs_led     = Pin(14, Pin.OUT)
tc_led      = Pin(13, Pin.OUT)
parking_led = Pin(12, Pin.OUT)

can = CAN(0, baudrate=500_000)

can.setfilter(0, CAN.LIST16, 0, (0x043, 0x039, 0x1D3))

# ── State ─────────────────────────────────────────────────────────────────────

engine_rpm      = 0
throttle_pos    = 0      # reported by engine ECU
parking_brake   = 0

# Driver inputs (in real system from steering wheel sensors)
# For simulation these are set by incoming CARLA data
throttle_input  = 0      # 0-1023
brake_input     = 0      # 0-1023
steer_input     = 0      # -511 to 511
gear_input      = 1      # 0=N 1-6 gear

# Computed
vehicle_speed   = 0      # km/h
abs_active      = 0
tire_angle      = 0
steering_output = 0


# ── Chassis model ─────────────────────────────────────────────────────────────

def compute_dynamics():
    global vehicle_speed, abs_active, tire_angle, steering_output

    # Simple vehicle speed from RPM (assume 4th gear ratio ~1.0)
    if engine_rpm > 0:
        wheel_rpm    = engine_rpm / 3.5
        speed_ms     = (wheel_rpm / 60.0) * TYRE_CIRC_M
        vehicle_speed = speed_ms * 3.6   # km/h

    # ABS - triggers if heavy braking at speed
    if brake_input > 800 and vehicle_speed > 10:
        abs_active = 1
        abs_led.on()
    else:
        abs_active = 0
        abs_led.off()

    # Tire angle proportional to steering input
    # steer_input -511 to 511 → tire angle 0-1 normalised
    tire_angle = int((steer_input + 511) / 1022 * 65535)

    # Power steering output 0-100
    steering_output = min(100, abs(steer_input) // 5)

    parking_led.value(parking_brake)


# ── CAN receive ───────────────────────────────────────────────────────────────

def process_frame(arb_id, data):
    global engine_rpm, throttle_pos, parking_brake

    if arb_id == 0x043:
        engine_rpm = int.from_bytes(data[0:4], 'big')

    elif arb_id == 0x039:
        throttle_pos = int.from_bytes(data[0:2], 'big')

    elif arb_id == 0x1D3:
        parking_brake = data[0]


# ── CAN transmit ──────────────────────────────────────────────────────────────

def broadcast(timer):
    compute_dynamics()

    # Commands to engine ECU
    can.send(throttle_input.to_bytes(2, 'big'), 0x02F)
    can.send(brake_input.to_bytes(2, 'big'),    0x01A)
    can.send(bytes([gear_input]),               0x06D)
    can.send(bytes([parking_brake]),            0x1C9)

    # Chassis reports
    # 0x058 Steering position (signed, encode as unsigned 2 bytes)
    steer_unsigned = steer_input + 511
    can.send(steer_unsigned.to_bytes(2, 'big'), 0x058)

    # 0x062 Power steering output
    can.send(steering_output.to_bytes(2, 'big'), 0x062)

    # 0x146 Brake oil (dummy - always OK)
    can.send(bytes([0x00, 0x00]), 0x146)

    # 0x15A ABS operation
    can.send(bytes([abs_active]), 0x15A)

    # 0x16F Throttle adjustment (vehicle speed in km/h)
    speed_encoded = min(1023, int(vehicle_speed))
    can.send(speed_encoded.to_bytes(2, 'big'), 0x16F)

    # 0x198 Tire angle
    can.send(tire_angle.to_bytes(2, 'big'), 0x198)

    print(
        f"[CHASSIS] Speed={vehicle_speed:.1f}km/h  "
        f"Steer={steer_input}  "
        f"ABS={abs_active}  "
        f"Gear={gear_input}"
    )


# ── Main ──────────────────────────────────────────────────────────────────────

print("[CHASSIS ECU] Starting...")

timer = Timer(-1)
timer.init(period=BROADCAST_MS, mode=Timer.PERIODIC, callback=broadcast)

print("[CHASSIS ECU] Running.")

while True:
    if can.any():
        arb_id, rtr, fdf, data = can.recv()
        process_frame(arb_id, bytes(data))
    time.sleep_ms(1)