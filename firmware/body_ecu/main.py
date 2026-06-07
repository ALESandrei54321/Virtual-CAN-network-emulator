# firmware/body_ecu/main.py

"""
Body ECU
========
Controls lighting, wipers, doors, windows, horn and safety systems.
CAN IDs match the CARLA vehicle simulation definitions.

Receives (commands from chassis):
  0x083  Turn signal switch        (1 byte: 1=left 2=right 4=hazard)
  0x098  Horn switch               (1 byte: 0-1)
  0x1A7  Light switch              (1 byte: 1=pos 2=head 4=highbeam 8=fog)
  0x1B1  Headlight flash switch    (1 byte: 0-8)
  0x25C  Front wiper switch        (1 byte: 1=int 2=low 4=high 10=wash)
  0x271  Rear wiper switch         (1 byte: 1=on 2=wash 3=on+wash)
  0x286  Door lock switch          (1 byte: 1=lock 2=unlock 3=all)
  0x29C  Right window switch       (1 byte: 1=up 2=down)
  0x2B1  Left window switch        (1 byte: 1=up 2=down)

Receives (reports from other ECUs):
  0x16F  Vehicle speed             (from Chassis ECU, km/h encoded)

Transmits (status reports):
  0x08D  Turn signal indicator     (1 byte)
  0x0A2  Horn operation            (1 byte)
  0x1BB  Light indicator           (1 byte)
  0x266  Front wiper status        (1 byte)
  0x27B  Rear wiper status         (1 byte)
  0x290  Door open/locked status   (2 bytes)
  0x2A6  Right window position     (2 bytes)
  0x2BB  Left window position      (2 bytes)
  0x0B4  Airbag activation         (1 byte)
  0x4B0  Collision alert           (1 byte)
  0x3E9  Closest radar point       (2 bytes)
  0x457  Seat belt sensor          (1 byte)
  0x461  Seat belt alarm           (1 byte)
"""

from machine import CAN, Pin, Timer
import time

# ── Constants ─────────────────────────────────────────────────────────────────

BROADCAST_MS      = 10
AUTOLOCK_SPEED_KH = 10    # km/h

# Light bitmasks (matching CARLA lightFront)
LIGHT_POSITION    = 0x01
LIGHT_HEAD        = 0x02
LIGHT_HIGHBEAM    = 0x04
LIGHT_FOG         = 0x08

# Turn signal values (matching CARLA lightTurn)
TURN_LEFT         = 1
TURN_RIGHT        = 2
TURN_HAZARD       = 4

# Wiper values
WIPER_INTERMIT    = 1
WIPER_LOW         = 2
WIPER_HIGH        = 4
WIPER_WASH        = 10

# ── Hardware ──────────────────────────────────────────────────────────────────

head_led   = Pin(10, Pin.OUT)
hazard_led = Pin(11, Pin.OUT)
lock_led   = Pin(12, Pin.OUT)
horn_led   = Pin(9,  Pin.OUT)

can = CAN(0, baudrate=500_000, fd=True)

can.setfilter(0, CAN.LIST16, 0, (
    0x083, 0x098, 0x1A7, 0x1B1,
    0x25C, 0x271, 0x286, 0x29C, 0x2B1,
    0x16F
))

# ── State ─────────────────────────────────────────────────────────────────────

vehicle_speed     = 0
turn_signal       = 0
horn_active       = 0
light_switch      = 0
light_flash       = 0
front_wiper       = 0
rear_wiper        = 0
door_lock_cmd     = 0
right_window_cmd  = 0
left_window_cmd   = 0

# Actual states
light_actual      = 0
doors_locked      = 0
right_window_pos  = 0    # 0=closed 255=open
left_window_pos   = 0
airbag_deployed   = 0
collision_alert   = 0
radar_distance    = 9999  # cm
seat_belt         = 1    # 1=buckled
hazard_tick       = 0


# ── Body logic ────────────────────────────────────────────────────────────────

def compute_lights() -> int:
    actual = light_switch

    # High beam needs headlights
    if actual & LIGHT_HIGHBEAM and not actual & LIGHT_HEAD:
        actual &= ~LIGHT_HIGHBEAM

    # Flash overrides if active
    if light_flash:
        actual |= LIGHT_HIGHBEAM

    return actual


def update_windows():
    global right_window_pos, left_window_pos

    if right_window_cmd == 1:
        right_window_pos = min(255, right_window_pos + 5)
    elif right_window_cmd == 2:
        right_window_pos = max(0,   right_window_pos - 5)

    if left_window_cmd == 1:
        left_window_pos  = min(255, left_window_pos + 5)
    elif left_window_cmd == 2:
        left_window_pos  = max(0,   left_window_pos - 5)


def check_autolock():
    global doors_locked

    if vehicle_speed >= AUTOLOCK_SPEED_KH and not doors_locked:
        doors_locked = 1
        lock_led.on()
        print(f"[BODY] Auto-lock at {vehicle_speed:.1f} km/h")

    elif vehicle_speed == 0 and doors_locked:
        doors_locked = 0
        lock_led.off()
        print("[BODY] Auto-unlock")


def check_seat_belt():
    """Warn if moving without seat belt."""
    if vehicle_speed > 5 and not seat_belt:
        print("[BODY] SEAT BELT WARNING")
        return 1
    return 0


# ── CAN receive ───────────────────────────────────────────────────────────────

def process_frame(arb_id, data):
    global turn_signal, horn_active, light_switch, light_flash
    global front_wiper, rear_wiper, door_lock_cmd
    global right_window_cmd, left_window_cmd, vehicle_speed

    if arb_id == 0x16F:
        new_val = int.from_bytes(data[0:2], 'big')
        if new_val != vehicle_speed:
            vehicle_speed = new_val
            check_autolock()

    elif arb_id == 0x083:
        new_val = data[0]
        if new_val != turn_signal:
            turn_signal = new_val
            ts = {1:"LEFT", 2:"RIGHT", 4:"HAZARD"}.get(turn_signal, "OFF")
            print(f"[BODY] Turn signal: {ts}")

    elif arb_id == 0x098:
        new_val = data[0]
        if new_val != horn_active:
            horn_active = new_val
            horn_led.value(horn_active)
            print(f"[BODY] Horn: {'ON' if horn_active else 'OFF'}")

    elif arb_id == 0x1A7:
        new_val = data[0]
        if new_val != light_switch:
            light_switch = new_val
            head_led.value(1 if light_switch & LIGHT_HEAD else 0)
            print(f"[BODY] Lights: 0x{light_switch:02X}")

    elif arb_id == 0x1B1:
        new_val = data[0]
        if new_val != light_flash:
            light_flash = new_val
            print(f"[BODY] Flash: {light_flash}")

    elif arb_id == 0x25C:
        new_val = data[0]
        if new_val != front_wiper:
            front_wiper = new_val
            wmode = {1:"INT", 2:"LOW", 4:"HIGH", 10:"WASH"}.get(front_wiper, "OFF")
            print(f"[BODY] Front wiper: {wmode}")

    elif arb_id == 0x271:
        new_val = data[0]
        if new_val != rear_wiper:
            rear_wiper = new_val
            print(f"[BODY] Rear wiper: {rear_wiper}")

    elif arb_id == 0x286:
        door_lock_cmd = data[0]
        if door_lock_cmd in (1, 3):
            doors_locked = 1
            lock_led.on()
            print("[BODY] Doors LOCKED")
        elif door_lock_cmd == 2:
            doors_locked = 0
            lock_led.off()
            print("[BODY] Doors UNLOCKED")

    elif arb_id == 0x29C:
        right_window_cmd = data[0]

    elif arb_id == 0x2B1:
        left_window_cmd = data[0]


# ── CAN transmit ──────────────────────────────────────────────────────────────

def broadcast(timer):
    global hazard_tick

    light_actual = compute_lights()
    update_windows()
    belt_alarm   = check_seat_belt()

    # Hazard flashing - toggle indicator every other tick
    turn_out = turn_signal
    if turn_signal == TURN_HAZARD:
        hazard_tick ^= 1
        turn_out = TURN_HAZARD if hazard_tick else 0

    hazard_led.value(1 if turn_signal == TURN_HAZARD and hazard_tick else 0)

    # 0x08D Turn signal indicator
    can.send(bytes([turn_out]), 0x08D, fdf=True)

    # 0x0A2 Horn operation
    can.send(bytes([horn_active]), 0x0A2, fdf=True)

    # 0x1BB Light indicator
    can.send(bytes([light_actual]), 0x1BB, fdf=True)

    # 0x266 Front wiper status
    can.send(bytes([front_wiper]), 0x266, fdf=True)

    # 0x27B Rear wiper status
    can.send(bytes([rear_wiper]), 0x27B, fdf=True)

    # 0x290 Door status (byte0=open bitmask byte1=locked)
    can.send(bytes([0x00, doors_locked]), 0x290, fdf=True)

    # 0x2A6 Right window position
    can.send(bytes([0x00, right_window_pos]), 0x2A6, fdf=True)

    # 0x2BB Left window position
    can.send(bytes([0x00, left_window_pos]), 0x2BB, fdf=True)

    # 0x0B4 Airbag
    can.send(bytes([airbag_deployed]), 0x0B4, fdf=True)

    # 0x4B0 Collision alert
    can.send(bytes([collision_alert]), 0x4B0, fdf=True)

    # 0x3E9 Radar distance
    can.send(radar_distance.to_bytes(2, 'big'), 0x3E9, fdf=True)

    # 0x457 Seat belt
    if broadcast.tick % 50 == 0:
        can.send(bytes([seat_belt]), 0x457, fdf=True)
        can.send(bytes([belt_alarm]), 0x461, fdf=True)

    broadcast.tick += 1

    if broadcast.tick % 10 == 0:
        print(
            f"[BODY] Lights=0x{light_actual:02X}  "
            f"Turn={turn_out}  "
            f"Wiper={front_wiper}  "
            f"Lock={doors_locked}"
        )


broadcast.tick = 0

# ── Main ──────────────────────────────────────────────────────────────────────

print("[BODY ECU] Starting...")

timer = Timer(-1)
timer.init(period=BROADCAST_MS, mode=Timer.PERIODIC, callback=broadcast)

print("[BODY ECU] Running.")

while True:
    if can.any():
        arb_id, rtr, fdf, data = can.recv()
        process_frame(arb_id, bytes(data))
    time.sleep_ms(1)