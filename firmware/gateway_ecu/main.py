# firmware/gateway_ecu/main.py

"""
Gateway ECU
===========
Bridges the CARLA simulator to the virtual CAN bus.
Runs a TCP server on localhost:5555. The CARLA client connects
and streams vehicle telemetry as JSON. The Gateway converts this
to CAN frames and writes them to the bus. It also reads bus
reports and sends them back to the client.

Transmits (commands derived from CARLA telemetry):
  0x02F  Throttle pedal command     (2 bytes, 0-1023)  → Engine
  0x01A  Brake command              (2 bytes, 0-1023)  → Engine
  0x06D  Gear shift command         (1 byte)           → Engine
  0x1B8  Engine start button        (1 byte, 0-1)      → Engine
  0x1C9  Parking brake command      (1 byte, 0-1)      → Engine
  0x058  Steering wheel position    (2 bytes)           → Chassis report
  0x083  Turn signal switch         (1 byte)            → Body
  0x1A7  Light switch               (1 byte)            → Body
  0x1B1  Headlight flash            (1 byte)            → Body
  0x7FF  CARLA heartbeat            (1 byte)            → Chassis (passive mode)

Receives (reports from other ECUs):
  0x043  Engine RPM                 (from Engine)
  0x19A  Engine status              (from Engine)
  0x077  Gear position              (from Engine)
  0x1D3  Parking brake status       (from Engine)
  0x16F  Vehicle speed              (from Chassis)
  0x08D  Turn signal indicator      (from Body)
  0x1BB  Light indicator            (from Body)
"""

from machine import CAN, Pin, Timer
import time
import socket
import threading
import json

# ── Constants ─────────────────────────────────────────────────────────────────

TCP_HOST         = "127.0.0.1"
TCP_PORT         = 5555
HEARTBEAT_MS     = 500     # Send CARLA-active heartbeat every 500ms
BROADCAST_MS     = 10      # Match other ECUs

# CAN IDs we write (commands to other ECUs)
ID_THROTTLE      = 0x02F
ID_BRAKE         = 0x01A
ID_GEAR          = 0x06D
ID_IGNITION      = 0x1B8
ID_PARK_BRAKE    = 0x1C9
ID_STEER         = 0x058
ID_TURN_SIGNAL   = 0x083
ID_LIGHT_FRONT   = 0x1A7
ID_LIGHT_FLASH   = 0x1B1
ID_HEARTBEAT     = 0x7FF

# ── Hardware ──────────────────────────────────────────────────────────────────

link_led = Pin(25, Pin.OUT)   # Indicates CARLA client connected

can = CAN(0, baudrate=500_000, fd=True)

# Listen for report frames from other ECUs
can.setfilter(0, CAN.LIST16, 0, (
    0x043, 0x19A, 0x077, 0x1D3,
    0x16F, 0x08D, 0x1BB
))

# ── State ─────────────────────────────────────────────────────────────────────

# Latest telemetry from CARLA client
carla_data = {
    "throttle":    0,
    "brake":       0,
    "steer":       0,
    "gear":        0,
    "ignition":    0,
    "hand_brake":  0,
    "light_turn":  0,
    "light_front": 0,
    "light_flash": 0,
}
data_lock = threading.Lock()

# Latest reports read from bus (sent back to client)
bus_reports = {
    "rpm":           0,
    "engine_status": 0,
    "gear_pos":      0,
    "park_brake":    0,
    "speed":         0,
    "turn_signal":   0,
    "lights":        0,
}
reports_lock = threading.Lock()

client_connected = False
client_socket    = None
client_lock      = threading.Lock()

# Previous values for change-only printing
_prev = {}


# ── TCP server ────────────────────────────────────────────────────────────────

def tcp_server():
    """Accept one CARLA client at a time and receive telemetry JSON."""
    global client_connected, client_socket

    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((TCP_HOST, TCP_PORT))
    server.listen(1)
    server.settimeout(1.0)

    print(f"[GATEWAY] TCP server listening on {TCP_HOST}:{TCP_PORT}")

    while True:
        try:
            conn, addr = server.accept()
            print(f"[GATEWAY] Client connected from {addr}")
            conn.settimeout(0.1)

            with client_lock:
                client_connected = True
                client_socket = conn

            link_led.on()
            handle_client(conn)

        except socket.timeout:
            continue
        except Exception as e:
            print(f"[GATEWAY] Server error: {e}")
            time.sleep(1)


def handle_client(conn):
    """Handle a connected CARLA client — read telemetry, send reports."""
    global client_connected, client_socket

    buf = ""
    while True:
        try:
            chunk = conn.recv(4096).decode("utf-8")
            if not chunk:
                break
            buf += chunk

            # Process complete JSON messages (newline-delimited)
            while "\n" in buf:
                line, buf = buf.split("\n", 1)
                line = line.strip()
                if not line:
                    continue
                try:
                    msg = json.loads(line)
                    if msg.get("type") == "telemetry":
                        with data_lock:
                            for k, v in msg.get("data", {}).items():
                                if k in carla_data:
                                    carla_data[k] = v

                    elif msg.get("type") == "request_reports":
                        with reports_lock:
                            resp = json.dumps({
                                "type": "reports",
                                "data": dict(bus_reports)
                            }) + "\n"
                        try:
                            conn.sendall(resp.encode("utf-8"))
                        except Exception:
                            break

                except json.JSONDecodeError:
                    pass

        except socket.timeout:
            continue
        except (ConnectionResetError, BrokenPipeError, OSError):
            break

    print("[GATEWAY] Client disconnected")
    with client_lock:
        client_connected = False
        client_socket = None
    link_led.off()
    conn.close()


# ── CAN receive ───────────────────────────────────────────────────────────────

def process_frame(arb_id, data):
    """Process report frames from other ECUs."""
    with reports_lock:
        if arb_id == 0x043:
            bus_reports["rpm"] = int.from_bytes(data[0:4], 'big')
        elif arb_id == 0x19A:
            bus_reports["engine_status"] = data[0]
        elif arb_id == 0x077:
            bus_reports["gear_pos"] = data[0]
        elif arb_id == 0x1D3:
            bus_reports["park_brake"] = data[0]
        elif arb_id == 0x16F:
            bus_reports["speed"] = int.from_bytes(data[0:2], 'big')
        elif arb_id == 0x08D:
            bus_reports["turn_signal"] = data[0]
        elif arb_id == 0x1BB:
            bus_reports["lights"] = data[0]


# ── CAN transmit ──────────────────────────────────────────────────────────────

def broadcast(timer):
    """Send CARLA telemetry as CAN commands every tick."""
    if not client_connected:
        # Only send heartbeat-stop when client disconnects
        return

    with data_lock:
        throttle    = carla_data["throttle"]
        brake       = carla_data["brake"]
        steer       = carla_data["steer"]
        gear        = carla_data["gear"]
        ignition    = carla_data["ignition"]
        hand_brake  = carla_data["hand_brake"]
        light_turn  = carla_data["light_turn"]
        light_front = carla_data["light_front"]
        light_flash = carla_data["light_flash"]

    # Clamp values
    throttle   = max(0, min(1023, int(throttle)))
    brake      = max(0, min(1023, int(brake)))
    steer_val  = max(-511, min(511, int(steer)))
    gear       = max(0, min(255, int(gear)))

    # Send command frames
    can.send(throttle.to_bytes(2, 'big'),  ID_THROTTLE, fdf=True)
    can.send(brake.to_bytes(2, 'big'),     ID_BRAKE, fdf=True)
    can.send(bytes([gear]),                ID_GEAR, fdf=True)
    can.send(bytes([1 if ignition else 0]),ID_IGNITION, fdf=True)
    can.send(bytes([1 if hand_brake else 0]), ID_PARK_BRAKE, fdf=True)

    # Steering as unsigned (add 511 offset, same as chassis ECU)
    steer_unsigned = steer_val + 511
    can.send(steer_unsigned.to_bytes(2, 'big'), ID_STEER, fdf=True)

    # Body commands
    can.send(bytes([int(light_turn) & 0xFF]),  ID_TURN_SIGNAL, fdf=True)
    can.send(bytes([int(light_front) & 0xFF]), ID_LIGHT_FRONT, fdf=True)
    can.send(bytes([int(light_flash) & 0xFF]), ID_LIGHT_FLASH, fdf=True)

    # CARLA heartbeat — tells Chassis ECU to go passive
    can.send(bytes([0x01]), ID_HEARTBEAT, fdf=True)

    # Print status changes only
    broadcast.tick += 1
    if broadcast.tick % 10 == 0:
        _print_status(throttle, brake, steer_val, gear, ignition)


broadcast.tick = 0


def _print_status(throttle, brake, steer, gear, ignition):
    """Print only when values change, to avoid flooding."""
    global _prev
    current = {
        "t": throttle, "b": brake, "s": steer,
        "g": gear, "i": ignition
    }
    if current != _prev:
        _prev = current
        print(
            f"[GATEWAY] Throttle={throttle:4d}  "
            f"Brake={brake:4d}  "
            f"Steer={steer:+4d}  "
            f"Gear={gear}  "
            f"Ign={'ON' if ignition else 'OFF'}"
        )


# ── Heartbeat for CARLA-disconnect ────────────────────────────────────────────

def heartbeat(timer):
    """Send heartbeat while connected, stop when disconnected."""
    if client_connected:
        can.send(bytes([0x01]), ID_HEARTBEAT, fdf=True)


# ── Main ──────────────────────────────────────────────────────────────────────

print("[GATEWAY ECU] Starting...")

# Start TCP server in background
tcp_thread = threading.Thread(target=tcp_server, daemon=True, name="GW-TCP")
tcp_thread.start()

# Start CAN broadcast timer
tx_timer = Timer(-1)
tx_timer.init(period=BROADCAST_MS, mode=Timer.PERIODIC, callback=broadcast)

# Start heartbeat timer (slower)
hb_timer = Timer(-2)
hb_timer.init(period=HEARTBEAT_MS, mode=Timer.PERIODIC, callback=heartbeat)

print(f"[GATEWAY ECU] Running. Waiting for CARLA client on {TCP_HOST}:{TCP_PORT}")

while True:
    if can.any():
        arb_id, rtr, fdf, data = can.recv()
        process_frame(arb_id, bytes(data))
    time.sleep_ms(1)
