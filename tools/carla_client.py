#!/usr/bin/env python3
# tools/carla_client.py

"""
CARLA Client
============
Connects to CARLA 0.9.16 server, spawns a vehicle, renders camera feed,
and sends vehicle telemetry to the Gateway ECU via TCP.

Usage:
    python tools/carla_client.py
    python tools/carla_client.py --host 127.0.0.1 --port 2000
    python tools/carla_client.py --vehicle vehicle.tesla.model3

Controls:
    W/UP        : throttle
    S/DOWN      : brake
    A/D or L/R  : steer
    ENTER       : toggle engine on/off
    Q           : toggle reverse
    SPACE       : hand-brake
    P           : toggle autopilot
    L           : cycle lights (position → low → fog → off)
    SHIFT+L     : toggle high beam
    E           : flash headlights
    Z           : toggle left blinker
    X           : toggle right blinker
    ESC         : quit
"""

import sys
import os
import argparse
import math
import time
import socket
import json
import threading

# ── Find CARLA module ─────────────────────────────────────────────────────────

import carla
import pygame
import numpy as np

from pygame.locals import (
    KMOD_CTRL, KMOD_SHIFT,
    K_UP, K_DOWN, K_LEFT, K_RIGHT,
    K_a, K_d, K_e, K_l, K_p, K_q, K_s, K_w, K_x, K_z,
    K_SPACE, K_RETURN, K_ESCAPE,
    K_F1,
)


# ── Gateway TCP Client ───────────────────────────────────────────────────────

class GatewayLink:
    """TCP connection to the Gateway ECU."""

    def __init__(self, host="127.0.0.1", port=5555):
        self.host      = host
        self.port      = port
        self.sock      = None
        self.connected = False
        self._buf      = ""
        self.reports   = {}

    def connect(self):
        """Connect to Gateway ECU TCP server."""
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.settimeout(2.0)
            self.sock.connect((self.host, self.port))
            self.sock.settimeout(0.05)
            self.connected = True
            print(f"[CARLA] Connected to Gateway ECU at {self.host}:{self.port}")
            return True
        except Exception as e:
            print(f"[CARLA] Cannot connect to Gateway: {e}")
            print(f"[CARLA] Make sure the CAN network is running first!")
            self.connected = False
            return False

    def send_telemetry(self, data: dict):
        """Send telemetry dict to Gateway."""
        if not self.connected:
            return
        msg = json.dumps({"type": "telemetry", "data": data}) + "\n"
        try:
            self.sock.sendall(msg.encode("utf-8"))
        except Exception:
            self.connected = False

    def request_reports(self) -> dict:
        """Request bus reports from Gateway."""
        if not self.connected:
            return {}
        msg = json.dumps({"type": "request_reports"}) + "\n"
        try:
            self.sock.sendall(msg.encode("utf-8"))
            # Read response
            chunk = self.sock.recv(4096).decode("utf-8")
            self._buf += chunk
            while "\n" in self._buf:
                line, self._buf = self._buf.split("\n", 1)
                line = line.strip()
                if line:
                    try:
                        resp = json.loads(line)
                        if resp.get("type") == "reports":
                            self.reports = resp.get("data", {})
                    except json.JSONDecodeError:
                        pass
        except socket.timeout:
            pass
        except Exception:
            self.connected = False
        return self.reports

    def close(self):
        if self.sock:
            self.sock.close()
            self.connected = False


# ── HUD ───────────────────────────────────────────────────────────────────────

class HUD:
    """On-screen display with vehicle info."""

    def __init__(self, width, height):
        self.dim = (width, height)
        font_name = 'mono'
        fonts = [x for x in pygame.font.get_fonts() if font_name in x]
        default_font = 'ubuntumono'
        mono = default_font if default_font in fonts else (fonts[0] if fonts else None)
        if mono:
            mono = pygame.font.match_font(mono)
        self._font = pygame.font.Font(mono, 16)
        self._font_big = pygame.font.Font(mono, 22)
        self._info = []
        self._notification = ""
        self._notification_end = 0

    def notification(self, text, seconds=3.0):
        self._notification = text
        self._notification_end = time.time() + seconds

    def tick(self, clock, control, vehicle, bus_reports, engine_on, autopilot):
        v = vehicle.get_velocity()
        speed_kmh = 3.6 * math.sqrt(v.x**2 + v.y**2 + v.z**2)

        bus_rpm   = bus_reports.get("rpm", 0)
        bus_speed = bus_reports.get("speed", 0)
        bus_gear  = bus_reports.get("gear_pos", 0)
        bus_eng   = bus_reports.get("engine_status", 0)

        self._info = [
            f"FPS:       {clock.get_fps():5.1f}",
            "",
            f"Speed:     {speed_kmh:5.1f} km/h   (bus: {bus_speed})",
            f"Engine:    {'ON' if engine_on else 'OFF'}  (bus: {'ON' if bus_eng else 'OFF'})",
            f"RPM:       {bus_rpm:5d}",
            f"Gear:      {control.gear}  (bus: {bus_gear})",
            f"Throttle:  {control.throttle:.2f}",
            f"Brake:     {control.brake:.2f}",
            f"Steer:     {control.steer:+.2f}",
            f"Hand brake:{' ON' if control.hand_brake else ' OFF'}",
            f"Reverse:   {'YES' if control.reverse else 'NO'}",
            f"Autopilot: {'ON' if autopilot else 'OFF'}",
            "",
            f"Gateway:   {'CONNECTED' if bus_reports else 'WAITING'}",
        ]

    def render(self, display):
        # Info panel background
        panel_w = 320
        panel = pygame.Surface((panel_w, self.dim[1]))
        panel.set_alpha(160)
        panel.fill((0, 0, 0))
        display.blit(panel, (0, 0))

        # Title
        title = self._font_big.render("CARLA → CAN Bus", True, (100, 220, 255))
        display.blit(title, (10, 8))

        # Info lines
        y = 40
        for line in self._info:
            surface = self._font.render(line, True, (220, 220, 220))
            display.blit(surface, (10, y))
            y += 20

        # Notification
        if self._notification and time.time() < self._notification_end:
            notif = self._font_big.render(self._notification, True, (255, 220, 80))
            rect = notif.get_rect(center=(self.dim[0] // 2, self.dim[1] - 40))
            display.blit(notif, rect)


# ── Camera ────────────────────────────────────────────────────────────────────

class CameraManager:
    """Manages a camera sensor and renders to pygame surface."""

    def __init__(self, vehicle, width, height):
        self.surface = None
        self._vehicle = vehicle
        bp_lib = vehicle.get_world().get_blueprint_library()
        bp = bp_lib.find("sensor.camera.rgb")
        bp.set_attribute("image_size_x", str(width))
        bp.set_attribute("image_size_y", str(height))
        bp.set_attribute("fov", "90")

        transform = carla.Transform(
            carla.Location(x=-5.5, z=2.5),
            carla.Rotation(pitch=-10)
        )
        self.sensor = vehicle.get_world().spawn_actor(
            bp, transform, attach_to=vehicle
        )
        self.sensor.listen(self._on_image)

    def _on_image(self, image):
        array = np.frombuffer(image.raw_data, dtype=np.uint8)
        array = array.reshape((image.height, image.width, 4))[:, :, :3]
        array = array[:, :, ::-1]  # BGR → RGB
        self.surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))

    def render(self, display):
        if self.surface:
            display.blit(self.surface, (0, 0))

    def destroy(self):
        if self.sensor and self.sensor.is_alive:
            self.sensor.destroy()


# ── Main Controller ───────────────────────────────────────────────────────────

class CarlaController:
    """Main game controller — handles input, telemetry, and rendering."""

    def __init__(self, args):
        self.args = args
        self.engine_on = True
        self.autopilot = False
        self._steer_cache = 0.0

        # Connect to CARLA
        print(f"[CARLA] Connecting to {args.host}:{args.port}...")
        self.client = carla.Client(args.host, args.port)
        self.client.set_timeout(10.0)
        self.world = self.client.get_world()

        # Spawn vehicle
        bp_lib = self.world.get_blueprint_library()
        bp = bp_lib.filter(args.vehicle)[0]
        if bp.has_attribute("color"):
            bp.set_attribute("color", "0,0,0")

        spawn_points = self.world.get_map().get_spawn_points()
        if not spawn_points:
            raise RuntimeError("No spawn points in map!")
        self.vehicle = self.world.try_spawn_actor(bp, spawn_points[0])
        if self.vehicle is None:
            # Try more spawn points
            for sp in spawn_points[1:5]:
                self.vehicle = self.world.try_spawn_actor(bp, sp)
                if self.vehicle:
                    break
        if self.vehicle is None:
            raise RuntimeError("Could not spawn vehicle!")

        print(f"[CARLA] Spawned {self.vehicle.type_id}")

        self._control = carla.VehicleControl()
        self._lights = carla.VehicleLightState.NONE

        # Pygame
        pygame.init()
        pygame.font.init()
        self.display = pygame.display.set_mode(
            (args.width, args.height),
            pygame.HWSURFACE | pygame.DOUBLEBUF
        )
        pygame.display.set_caption("CARLA → CAN Bus Client")

        self.clock = pygame.time.Clock()
        self.hud = HUD(args.width, args.height)
        self.camera = CameraManager(self.vehicle, args.width, args.height)

        # Gateway connection
        self.gateway = GatewayLink(args.gw_host, args.gw_port)

    def run(self):
        """Main loop."""
        # Connect to gateway (retry a few times)
        for attempt in range(10):
            if self.gateway.connect():
                break
            print(f"[CARLA] Retry {attempt+1}/10...")
            time.sleep(1.0)

        if not self.gateway.connected:
            print("[CARLA] WARNING: Running without Gateway connection")

        try:
            while True:
                self.clock.tick(30)

                # Process input
                if self._parse_events():
                    return

                # Apply control
                if not self.autopilot:
                    self._parse_keys(pygame.key.get_pressed(), self.clock.get_time())
                    self._control.reverse = self._control.gear < 0
                    # Update lights
                    self.vehicle.set_light_state(carla.VehicleLightState(self._lights))
                    self.vehicle.apply_control(self._control)

                # Send telemetry to Gateway
                self._send_telemetry()

                # Get reports from Gateway
                reports = self.gateway.request_reports()

                # Update HUD
                self.hud.tick(
                    self.clock, self._control, self.vehicle,
                    reports, self.engine_on, self.autopilot
                )

                # Render
                self.camera.render(self.display)
                self.hud.render(self.display)
                pygame.display.flip()

        finally:
            self._cleanup()

    def _send_telemetry(self):
        """Read vehicle state and send to Gateway."""
        ctrl   = self.vehicle.get_control()
        v      = self.vehicle.get_velocity()
        lights = int(self.vehicle.get_light_state())

        speed_kmh = 3.6 * math.sqrt(v.x**2 + v.y**2 + v.z**2)

        # Map throttle/brake to 0-1023 range
        throttle_val = int(ctrl.throttle * 1023)
        brake_val    = int(ctrl.brake * 1023)

        # Map steer -1..1 to -511..511
        steer_val = int(ctrl.steer * 511)

        # Gear
        gear_val = max(0, ctrl.gear) if not ctrl.reverse else 0

        # Lights
        light_turn = 0
        if lights & int(carla.VehicleLightState.LeftBlinker):
            light_turn = 1
        if lights & int(carla.VehicleLightState.RightBlinker):
            light_turn = 2 if light_turn == 0 else 4  # hazard

        light_front = 0
        if lights & int(carla.VehicleLightState.Position):
            light_front |= 0x01
        if lights & int(carla.VehicleLightState.LowBeam):
            light_front |= 0x02
        if lights & int(carla.VehicleLightState.HighBeam):
            light_front |= 0x04
        if lights & int(carla.VehicleLightState.Fog):
            light_front |= 0x08

        light_flash = 0x08 if (lights & int(carla.VehicleLightState.HighBeam)) else 0

        self.gateway.send_telemetry({
            "throttle":    throttle_val,
            "brake":       brake_val,
            "steer":       steer_val,
            "gear":        gear_val,
            "ignition":    1 if self.engine_on else 0,
            "hand_brake":  1 if ctrl.hand_brake else 0,
            "light_turn":  light_turn,
            "light_front": light_front,
            "light_flash": light_flash,
        })

    def _parse_events(self) -> bool:
        """Handle key-up events. Returns True to quit."""
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return True
            if event.type == pygame.KEYUP:
                if event.key == K_ESCAPE:
                    return True
                elif event.key == K_RETURN:
                    self.engine_on = not self.engine_on
                    state = "ON" if self.engine_on else "OFF"
                    self.hud.notification(f"Engine: {state}")
                elif event.key == K_p:
                    self.autopilot = not self.autopilot
                    self.vehicle.set_autopilot(self.autopilot)
                    self.hud.notification(
                        f"Autopilot {'ON' if self.autopilot else 'OFF'}"
                    )
                elif event.key == K_q:
                    self._control.gear = 1 if self._control.reverse else -1
                elif event.key == K_l and pygame.key.get_mods() & KMOD_SHIFT:
                    self._lights ^= int(carla.VehicleLightState.HighBeam)
                elif event.key == K_l:
                    self._cycle_lights()
                elif event.key == K_e:
                    self._lights ^= int(carla.VehicleLightState.HighBeam)
                    self.hud.notification("Flash!")
                elif event.key == K_z:
                    self._lights ^= int(carla.VehicleLightState.LeftBlinker)
                elif event.key == K_x:
                    self._lights ^= int(carla.VehicleLightState.RightBlinker)

        return False

    def _parse_keys(self, keys, millis):
        """Handle held-down keys for driving."""
        if not self.engine_on:
            self._control.throttle = 0.0
            self._control.brake = 0.0
            return

        # Throttle
        if keys[K_UP] or keys[K_w]:
            self._control.throttle = min(self._control.throttle + 0.1, 1.0)
        else:
            self._control.throttle = 0.0

        # Brake
        if keys[K_DOWN] or keys[K_s]:
            self._control.brake = min(self._control.brake + 0.2, 1.0)
        else:
            self._control.brake = 0.0

        # Steering
        steer_increment = 5e-4 * millis
        if keys[K_LEFT] or keys[K_a]:
            if self._steer_cache > 0:
                self._steer_cache = 0
            else:
                self._steer_cache -= steer_increment
        elif keys[K_RIGHT] or keys[K_d]:
            if self._steer_cache < 0:
                self._steer_cache = 0
            else:
                self._steer_cache += steer_increment
        else:
            self._steer_cache = 0.0

        self._steer_cache = max(-0.7, min(0.7, self._steer_cache))
        self._control.steer = round(self._steer_cache, 1)

        # Handbrake
        if keys[K_SPACE]:
            self._control.hand_brake = True
        else:
            self._control.hand_brake = False

    def _cycle_lights(self):
        """Cycle: off → position → low beam → fog → off."""
        pos = int(carla.VehicleLightState.Position)
        low = int(carla.VehicleLightState.LowBeam)
        fog = int(carla.VehicleLightState.Fog)

        if not (self._lights & pos):
            self._lights |= pos
            self.hud.notification("Position lights")
        elif not (self._lights & low):
            self._lights |= low
            self.hud.notification("Low beam")
        elif not (self._lights & fog):
            self._lights |= fog
            self.hud.notification("Fog lights")
        else:
            self._lights &= ~(pos | low | fog)
            self.hud.notification("Lights off")

    def _cleanup(self):
        """Clean up CARLA actors and pygame."""
        print("[CARLA] Cleaning up...")
        self.gateway.close()
        self.camera.destroy()
        if self.vehicle and self.vehicle.is_alive:
            self.vehicle.destroy()
        pygame.quit()
        print("[CARLA] Done.")


# ── CLI ───────────────────────────────────────────────────────────────────────

def parse_args():
    parser = argparse.ArgumentParser(
        description="CARLA client — sends vehicle telemetry to CAN bus via Gateway ECU"
    )
    parser.add_argument(
        "--host", default="192.168.1.140",
        help="CARLA server IP (default: 192.168.1.140)"
    )
    parser.add_argument(
        "-p", "--port", type=int, default=2000,
        help="CARLA server port (default: 2000)"
    )
    parser.add_argument(
        "--vehicle", default="vehicle.tesla.model3",
        help="Vehicle blueprint filter (default: vehicle.tesla.model3)"
    )
    parser.add_argument(
        "--res", default="1280x720",
        help="Window resolution WIDTHxHEIGHT (default: 1280x720)"
    )
    parser.add_argument(
        "--gw-host", default="127.0.0.1",
        help="Gateway ECU TCP host (default: 127.0.0.1)"
    )
    parser.add_argument(
        "--gw-port", type=int, default=5555,
        help="Gateway ECU TCP port (default: 5555)"
    )
    args = parser.parse_args()

    # Parse resolution
    w, h = args.res.split("x")
    args.width  = int(w)
    args.height = int(h)

    return args


def main():
    args = parse_args()
    try:
        controller = CarlaController(args)
        controller.run()
    except KeyboardInterrupt:
        print("\n[CARLA] Interrupted.")
    except Exception as e:
        print(f"\n[CARLA] Error: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()
