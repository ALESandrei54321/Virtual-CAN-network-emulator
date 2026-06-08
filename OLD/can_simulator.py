#!/usr/bin/env python

# Copyright (c) 2019 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

# Allows controlling a vehicle with a keyboard. For a simpler and more
# documented example, please take a look at tutorial.py.

"""
Welcome to CARLA manual control.

Use ARROWS or WASD keys for control.

    W            : throttle
    S            : brake
    A/D          : steer left/right

    ENTER        : toggle engine on/off
    J            : toggle CAN packet recording

    Q            : toggle reverse
    Space        : hand-brake
    P            : toggle autopilot
    M            : toggle manual transmission
    ,/.          : gear up/down
    CTRL + W     : toggle constant velocity mode at 60 km/h

    L            : toggle next light type
    SHIFT + L    : toggle high beam
    Z/X          : toggle right/left blinker
    I            : toggle interior light

    TAB          : change sensor position
    ` or N       : next sensor
    [1-9]        : change to sensor [1-9]
    G            : toggle radar visualization
    C            : change weather (Shift+C reverse)
    Backspace    : change vehicle

    O            : open/close all doors of vehicle
    T            : toggle vehicle's telemetry

    V            : Select next map layer (Shift+V reverse)
    B            : Load current selected map layer (Shift+B to unload)

    R            : toggle recording images to disk

    CTRL + R     : toggle recording of simulation (replacing any previous)
    CTRL + P     : start replaying last recorded simulation
    CTRL + +     : increments the start time of the replay by 1 second (+SHIFT = 10 seconds)
    CTRL + -     : decrements the start time of the replay by 1 second (+SHIFT = 10 seconds)

    F1           : toggle HUD
    H/?          : toggle help
    ESC          : quit
"""

from __future__ import print_function


# ==============================================================================
# -- find carla module ---------------------------------------------------------
# ==============================================================================


import glob
import os
import sys
from can_json_processor import CanJsonProcessor
from PyQt5.QtWidgets import QApplication

try:
    sys.path.append(glob.glob('carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass


# ==============================================================================
# -- imports -------------------------------------------------------------------
# ==============================================================================


import carla

from carla import ColorConverter as cc

import argparse
import collections
import datetime
import logging
import math
import random
import re
import weakref

try:
    import pygame
    from pygame.locals import KMOD_CTRL
    from pygame.locals import KMOD_SHIFT
    from pygame.locals import K_0
    from pygame.locals import K_9
    from pygame.locals import K_BACKQUOTE
    from pygame.locals import K_BACKSPACE
    from pygame.locals import K_COMMA
    from pygame.locals import K_DOWN
    from pygame.locals import K_ESCAPE
    from pygame.locals import K_F1
    from pygame.locals import K_LEFT
    from pygame.locals import K_PERIOD
    from pygame.locals import K_RIGHT
    from pygame.locals import K_SLASH
    from pygame.locals import K_SPACE
    from pygame.locals import K_TAB
    from pygame.locals import K_UP
    from pygame.locals import K_a
    from pygame.locals import K_b
    from pygame.locals import K_c
    from pygame.locals import K_d
    from pygame.locals import K_e
    from pygame.locals import K_f
    from pygame.locals import K_g
    from pygame.locals import K_h
    from pygame.locals import K_i
    from pygame.locals import K_j
    from pygame.locals import K_l
    from pygame.locals import K_m
    from pygame.locals import K_n
    from pygame.locals import K_o
    from pygame.locals import K_p
    from pygame.locals import K_q
    from pygame.locals import K_r
    from pygame.locals import K_s
    from pygame.locals import K_t
    from pygame.locals import K_v
    from pygame.locals import K_w
    from pygame.locals import K_x
    from pygame.locals import K_z
    from pygame.locals import K_MINUS
    from pygame.locals import K_EQUALS
    from pygame.locals import K_RETURN
except ImportError:
    raise RuntimeError('cannot import pygame, make sure pygame package is installed')

try:
    import numpy as np
except ImportError:
    raise RuntimeError('cannot import numpy, make sure numpy package is installed')


# ==============================================================================
# -- Global functions ----------------------------------------------------------
# ==============================================================================


def find_weather_presets():
    rgx = re.compile('.+?(?:(?<=[a-z])(?=[A-Z])|(?<=[A-Z])(?=[A-Z][a-z])|$)')
    name = lambda x: ' '.join(m.group(0) for m in rgx.finditer(x))
    presets = [x for x in dir(carla.WeatherParameters) if re.match('[A-Z].+', x)]
    return [(getattr(carla.WeatherParameters, x), name(x)) for x in presets]


def get_actor_display_name(actor, truncate=250):
    name = ' '.join(actor.type_id.replace('_', '.').title().split('.')[1:])
    return (name[:truncate - 1] + u'\u2026') if len(name) > truncate else name

def get_actor_blueprints(world, filter, generation):
    bps = world.get_blueprint_library().filter(filter)

    if generation.lower() == "all":
        return bps

    # If the filter returns only one bp, we assume that this one needed
    # and therefore, we ignore the generation
    if len(bps) == 1:
        return bps

    try:
        int_generation = int(generation)
        # Check if generation is in available generations
        if int_generation in [1, 2, 3]:
            bps = [x for x in bps if int(x.get_attribute('generation')) == int_generation]
            return bps
        else:
            print("   Warning! Actor Generation is not valid. No actor will be spawned.")
            return []
    except:
        print("   Warning! Actor Generation is not valid. No actor will be spawned.")
        return []


# ==============================================================================
# -- World ---------------------------------------------------------------------
# ==============================================================================


class World(object):
    def __init__(self, carla_world, hud, args):
        self.world = carla_world
        self.sync = args.sync
        self.actor_role_name = args.rolename
        import time as _time
        for _attempt in range(10):
            try:
                self.map = self.world.get_map()
                break
            except RuntimeError as error:
                if _attempt < 9:
                    print('Waiting for map data (attempt {}/10)...'.format(_attempt + 1))
                    _time.sleep(2)
                else:
                    print('RuntimeError: {}'.format(error))
                    print('  The server could not send the OpenDRIVE (.xodr) file:')
                    print('  Make sure it exists, has the same name of your town, and is correct.')
                    sys.exit(1)
        self.hud = hud
        self.player = None
        self.collision_sensor = None
        self.lane_invasion_sensor = None
        self.gnss_sensor = None
        self.imu_sensor = None
        self.radar_sensor = None
        self.camera_manager = None
        self._weather_presets = find_weather_presets()
        self._weather_index = 0
        self._actor_filter = args.filter
        self._actor_generation = args.generation
        self._gamma = args.gamma
        self.restart()
        self.world.on_tick(hud.on_world_tick)
        self.recording_enabled = False
        self.recording_start = 0
        self.constant_velocity_enabled = False
        self.show_vehicle_telemetry = False
        self.doors_are_open = False
        self.current_map_layer = 0
        self.map_layer_names = [
            carla.MapLayer.NONE,
            carla.MapLayer.Buildings,
            carla.MapLayer.Decals,
            carla.MapLayer.Foliage,
            carla.MapLayer.Ground,
            carla.MapLayer.ParkedVehicles,
            carla.MapLayer.Particles,
            carla.MapLayer.Props,
            carla.MapLayer.StreetLights,
            carla.MapLayer.Walls,
            carla.MapLayer.All
        ]

    def restart(self):
        self.player_max_speed = 1.589
        self.player_max_speed_fast = 3.713
        # Keep same camera config if the camera manager exists.
        cam_index = self.camera_manager.index if self.camera_manager is not None else 0
        cam_pos_index = self.camera_manager.transform_index if self.camera_manager is not None else 0
        # Get a random blueprint.
        # blueprint_list = get_actor_blueprints(self.world, self._actor_filter, self._actor_generation)
        # if not blueprint_list:
        #     raise ValueError("Couldn't find any blueprints with the specified filters")
        # blueprint = random.choice(blueprint_list)
        # blueprint.set_attribute('role_name', self.actor_role_name)
        ###
        blueprint = self.world.get_blueprint_library().filter(self._actor_filter)[0]
        if blueprint.has_attribute('terramechanics'):
            blueprint.set_attribute('terramechanics', 'true')
        if blueprint.has_attribute('color'):
            color = random.choice(blueprint.get_attribute('color').recommended_values)
            blueprint.set_attribute('color', color)
        if blueprint.has_attribute('driver_id'):
            driver_id = random.choice(blueprint.get_attribute('driver_id').recommended_values)
            blueprint.set_attribute('driver_id', driver_id)
        if blueprint.has_attribute('is_invincible'):
            blueprint.set_attribute('is_invincible', 'true')
        # set the max speed
        if blueprint.has_attribute('speed'):
            self.player_max_speed = float(blueprint.get_attribute('speed').recommended_values[1])
            self.player_max_speed_fast = float(blueprint.get_attribute('speed').recommended_values[2])

        # Spawn the player.
        if self.player is not None:
            spawn_point = self.player.get_transform()
            spawn_point.location.z += 2.0
            spawn_point.rotation.roll = 0.0
            spawn_point.rotation.pitch = 0.0
            self.destroy()
            self.player = self.world.try_spawn_actor(blueprint, spawn_point)
            self.show_vehicle_telemetry = False
            self.modify_vehicle_physics(self.player)
        while self.player is None:
            if not self.map.get_spawn_points():
                print('There are no spawn points available in your map/town.')
                print('Please add some Vehicle Spawn Point to your UE4 scene.')
                sys.exit(1)
            spawn_points = self.map.get_spawn_points()
            spawn_point = random.choice(spawn_points) if spawn_points else carla.Transform()
            self.player = self.world.try_spawn_actor(blueprint, spawn_point)
            self.show_vehicle_telemetry = False
            self.modify_vehicle_physics(self.player)
        # Set up the sensors.
        self.collision_sensor = CollisionSensor(self.player, self.hud)
        self.lane_invasion_sensor = LaneInvasionSensor(self.player, self.hud)
        self.gnss_sensor = GnssSensor(self.player)
        self.imu_sensor = IMUSensor(self.player)
        self.camera_manager = CameraManager(self.player, self.hud, self._gamma)
        self.camera_manager.transform_index = cam_pos_index
        self.camera_manager.set_sensor(cam_index, notify=False)
        actor_type = get_actor_display_name(self.player)
        self.hud.notification(actor_type)

        if self.sync:
            self.world.tick()
        else:
            self.world.wait_for_tick()

    def next_weather(self, reverse=False):
        self._weather_index += -1 if reverse else 1
        self._weather_index %= len(self._weather_presets)
        preset = self._weather_presets[self._weather_index]
        self.hud.notification('Weather: %s' % preset[1])
        self.player.get_world().set_weather(preset[0])

    def next_map_layer(self, reverse=False):
        self.current_map_layer += -1 if reverse else 1
        self.current_map_layer %= len(self.map_layer_names)
        selected = self.map_layer_names[self.current_map_layer]
        self.hud.notification('LayerMap selected: %s' % selected)

    def load_map_layer(self, unload=False):
        selected = self.map_layer_names[self.current_map_layer]
        if unload:
            self.hud.notification('Unloading map layer: %s' % selected)
            self.world.unload_map_layer(selected)
        else:
            self.hud.notification('Loading map layer: %s' % selected)
            self.world.load_map_layer(selected)

    def toggle_radar(self):
        if self.radar_sensor is None:
            self.radar_sensor = RadarSensor(self.player)
        elif self.radar_sensor.sensor is not None:
            self.radar_sensor.sensor.destroy()
            self.radar_sensor = None

    def modify_vehicle_physics(self, actor):
        #If actor is not a vehicle, we cannot use the physics control
        try:
            physics_control = actor.get_physics_control()
            physics_control.use_sweep_wheel_collision = True
            actor.apply_physics_control(physics_control)
        except Exception:
            pass

    def tick(self, clock):
        self.hud.tick(self, clock)

    def render(self, display):
        self.camera_manager.render(display)
        self.hud.render(display)

    def destroy_sensors(self):
        self.camera_manager.sensor.destroy()
        self.camera_manager.sensor = None
        self.camera_manager.index = None

    def destroy(self):
        if self.radar_sensor is not None:
            self.toggle_radar()
        sensors = [
            self.camera_manager.sensor,
            self.collision_sensor.sensor,
            self.lane_invasion_sensor.sensor,
            self.gnss_sensor.sensor,
            self.imu_sensor.sensor]
        for sensor in sensors:
            if sensor is not None:
                sensor.stop()
                sensor.destroy()
        if self.player is not None:
            self.player.destroy()







# ==============================================================================
# -- CAN Packet -------------------------------------------------------------
# ==============================================================================

class CANPacket:
    def __init__(self, can_id, src, dst, name, type, level, period, datasize, min, max, carlaVar, data):
        self.can_id = can_id
        self.src = src
        self.dst = dst
        self.name = name
        self.type = type
        self.level = level
        self.period = period
        self.datasize = datasize
        self.min = min
        self.max = max
        self.current_period = period
        self.carlaVar = carlaVar
        self.data = data

    def to_dict(self):
        return {
            "can_id": self.can_id,
            "src": self.src,
            "dst": self.dst,
            "name": self.name,
            "level": self.level,
            "type": self.type,
            "period": self.period,
            "datasize": self.datasize,
            "min": self.min,
            "max": self.max,
            "carlaVar": self.carlaVar,
            "data": self.data
        }
    def tick(self):
        if self.period != None:
            self.current_period -= 1

    def reset_period(self):
        self.current_period = self.period

    def to_can_binary(self):
        """
        Return a *simplified* binary representation of this CAN FD frame.
        NOTE:
        - No real CRC calculation is done.
        - No bit stuffing.
        - No extended ID.
        - BRS, ESI, etc. are hard-coded or simplified.
        """

        bits = ""

        # 1) SOF (Start of Frame): always 0 (dominant)
        bits += "0"

        # 2) Arbitration Field
        #    ID (11 bits)
        bits += format(int(self.can_id), '011b')  # 11-bit identifier

        #    r1 (Substitute Remote Request) = 0 for data frame
        bits += "0"

        # 3) Control Field

        #    IDE (Identifier Extension) = 0 for standard ID
        bits += "0"

        #    EDL (Extended Data Length) = 1 for CAN FD
        bits += "1"

        #    r0 (Remote Request Substitution) = 0 for data frame
        bits += "0"

        #    BRS bit (default to 1 => no bitrate switching)
        bits += "0"

        #    ESI bit (Error State Indicator), default to 1 (normal)
        bits += "1"

        # 3) DLC (4 bits)
        #    For CAN FD, DLC up to 15 is possible. 9..15 map to 12..64 bytes.
        #    Our datasize is small, so there us no need to map it.
        dlc_value = self.datasize
        if self.data == None:
            # If no data is provided, we set DLC to 0
            dlc_value = 0
        bits += format(dlc_value, '04b')

        # 4) Data field (up to 64 bytes).
        #    We'll just place exactly self.datasize bytes, ignoring the special DLC mapping.
        if self.data is not None:
            for i in range(self.datasize):
                # Extract each byte from self.data (assuming it's an int or array).
                # If self.data is an integer, shift/mask. If it's a bytes/bytearray, index it directly.
                byte = (self.data >> (8 * (self.datasize - 1 - i))) & 0xFF
                bits += format(byte, '08b')

        # 5) CRC field

        # --- CRC Calculation ---
        # CRC-17 polynomial for CAN FD: x^17 + x^16 + x^14 + x^13 + x^11 + x^6 + x^4 + x^3 + x^1 + 1 (0x3685B)
        CRC_POLY = 0x3685B
        CRC_LEN = 17

        # Take all bits from SOF to end of DATA field
        crc_input = bits
        crc_rg = 0

        for bit in crc_input:  # no 17 zeros appended
            nxtbit = int(bit)
            crcnxt = nxtbit ^ ((crc_rg >> (CRC_LEN - 1)) & 1)
            crc_rg = ((crc_rg << 1) & ((1 << CRC_LEN) - 1))  # shift and mask
            if crcnxt:
                crc_rg ^= CRC_POLY
                crc_rg &= (1 << CRC_LEN) - 1  # mask again after XOR


        crc_str = format(crc_rg, f'0{CRC_LEN}b')
        print(f"CRC Input: {crc_input}")
        print(f"CRC Calculation: {crc_str} ({crc_rg})")

        # Append CRC field (17 bits)
        bits += crc_str

        # 6) CRC Delimiter (1 bit, recessive = 1)
        bits += "1"

        # 7) ACK Slot (1 bit) + ACK Delimiter (1 bit)
        #    Typically the transmitter sends recessive bits here and a receiver that
        #    sees a valid message can drive the first one dominant. We'll just place "0" for slot and "1" for delimiter.
        bits += "01"

        # 8) EOF (7 bits, all recessive = 1)
        bits += "1111111"

        # 9) Add padding bits if needed (up to 81 bits total, the maximum for our simulation)
        if len(bits) < 81:
            bits += "1" * (81 - len(bits))

        return bits

    def __str__(self):
        return f"id={self.can_id}, name={self.name}, level={self.level}, period={self.period}, carlaVar={self.carlaVar}, data={self.data}"



# ==============================================================================
# -- CAN Simulator -------------------------------------------------------------
# ==============================================================================


# Class to simulate packages transmited during the simulation based on the json file
import json
from numpy import interp
import paho.mqtt.client as mqtt
import time

# Configurare client MQTT
mqtt_server = '192.168.100.120'  # Inlocuieste cu IP-ul laptopului
client_id = 'carla_client'
topic = 'canfd'


class CanSimulator:

    def __init__(self, world, json_file, controller, gui_processor=None):
        self.world = world
        self.json_file = json_file
        self.gui_processor = gui_processor

        self.packet_definitions = {}
        self.command_packets = []
        self.report_packets = {}
        self.filtered_ids = []

        self.gear_value = 0
        self.controller = controller
        self.current_lights = None
        self.hand_break_status = False

        self.recording = False
        self.recorded_packets = []  # list to store CAN packets
        self.mqtt_client = None
        self.start_mqtt()

        self.load_can_data()

    def start_mqtt(self):
        """Start the Paho MQTT client to send CAN packets"""

        if self.mqtt_client is not None:
            self.mqtt_client.disconnect()
            print("Disconnected from previous MQTT broker")

        self.mqtt_client = mqtt.Client()

        def on_connect(client, userdata, flags, rc):
            if rc == 0:
                print("Connected to MQTT broker")
            else:
                print(f"Failed to connect, return code {rc}")

        self.mqtt_client.on_connect = on_connect

        try:
            self.mqtt_client.connect(mqtt_server, 1883, 60)
            self.mqtt_client.loop_start()
        except Exception as e:
            print(f"Failed to connect to MQTT broker: {e}")

    def disconnect_mqtt(self):
        """Disconnect the MQTT client"""
        if self.mqtt_client is not None:
            self.mqtt_client.loop_stop()
            self.mqtt_client.disconnect()
            print("Disconnected from MQTT broker")
            self.mqtt_client = None


    def send_message(self, packet):
        """Send a message to the CAN bus"""
        # print(f"Sending message: {packet}")
        # Create a standard CAN message (not for carla, for extracting and sending to real ECU's)

        if self.gui_processor:
            if packet.level == "report":
                self.gui_processor.add_packet_to_table_report(packet.to_dict())
            else:
                self.gui_processor.add_packet_to_table(packet.to_dict())

        packet_no = len(self.recorded_packets)

        if packet_no in [100, 1000, 2000, 5000, 10000]:
            print(f"{packet_no} of packets so far.")


        if self.recording:
            #print(f"Sending message: id={packet.can_id}, data={packet.data}")
            can_frame = packet.to_can_binary()
            #print(f"CAN Frame: {can_frame}")
            self.recorded_packets.append(can_frame)

            if self.mqtt_client:
                try:
                    self.mqtt_client.publish(topic, payload=can_frame)
                except Exception as e:
                    print(f"Error publishing MQTT message: {e}")


    def load_can_data(self):
        """Load CAN packet definitions once and store in memory"""
        with open(self.json_file) as f:
            data = json.load(f)

        self.packet_definitions = data["can_id"]  # Store the full CAN packet definitions

        for can_id, details in self.packet_definitions.items():
            if details["level"] == "report":
                packet = CANPacket(
                    can_id=can_id,
                    src=details.get("source"),
                    dst=details.get("execution"),
                    name=details.get("name"),
                    level=details.get("level"),
                    type=details.get("type"),
                    period=details.get("period"),
                    datasize=details.get("datasize"),
                    min=details.get("min"),
                    max=details.get("max"),
                    carlaVar=details.get("carlaVar"),
                    data=self.get_carla_data(details.get("carlaVar"))
                )


                self.report_packets[can_id] = packet  # Store report packets separately

        # JUST FOR DEBUGGING COMMANDS, IGNORE REPORTS
        # self.packets = {}

    def get_carla_data(self, carlaVar):
        if carlaVar == "kmh":
            v = self.world.player.get_velocity()
            # return round(3.6 * math.sqrt(v.x**2 + v.y**2 + v.z**2), 2)
            return int(round(3.6 * math.sqrt(v.x**2 + v.y**2 + v.z**2), 2))
        elif carlaVar == "brake":
            value = round(self.world.player.get_control().brake, 2)
            return int(interp(value, [0, 1], [0, 1023]))
        elif carlaVar == "throttle":
            value = round(self.world.player.get_control().throttle, 2)
            return int(interp(value, [0, 1], [0, 1023]))
        elif carlaVar == "rpm":
            c = self.world.player.get_control()
            p = self.world.player.get_physics_control()
            engine_rpm = p.max_rpm * c.throttle
            # if c.gear > 0:
            #     engine_rpm *= c.gear
            return int(round(engine_rpm, 2))
        elif carlaVar == "steer":
            value = round(self.world.player.get_control().steer, 2)
            return int(interp(value, [-1, 1], [0, 100]))
        elif carlaVar == "tire_angle":
            ###
            return None
            # return self.world.player.get_wheel_steer_angle(carla.VehicleWheelLocation.FL_Wheel)
        elif carlaVar == "gear":
            if self.world.player.get_control().manual_gear_shift:
                return self.gear_value
            else:
                return self.world.player.get_control().gear
        elif carlaVar == "ignition":
            return self.controller.engine_on

        elif carlaVar == "engine":
            return None
        elif carlaVar == "lightTurn":
            if self.current_lights != None:
                lights = 0
                if self.current_lights & carla.VehicleLightState.LeftBlinker:
                    lights = 1
                if self.current_lights & carla.VehicleLightState.RightBlinker:
                    if lights == 0:
                        lights = 2
                    else:
                        lights = 4

                return lights
            else:
                return 0
        elif carlaVar == "lightFront":
            if self.current_lights != None:
                lights = ""
                lights += "1" if self.current_lights & carla.VehicleLightState.HighBeam else "0"
                lights += "1" if self.current_lights & carla.VehicleLightState.Position else "0"
                lights += "1" if self.current_lights & carla.VehicleLightState.LowBeam else "0"
                lights += "1" if self.current_lights & carla.VehicleLightState.Fog else "0"
                # print(lights)
                # if self.current_lights in [3, 7, 55, 23, 39, 35, 19, 51]:
                # if self.current_lights & carla.VehicleLightState.LowBeam & self.current_lights < 110:
                #     lights += "0"
                # # if self.current_lights in [131, 135, 183, 151, 167, 163, 147, 179]:
                # if self.current_lights & carla.VehicleLightState.LowBeam & self.current_lights > 110 and self.current_lights < 200:
                #     lights += "1"
                return int(lights, 2)
            else:
                return 0
        elif carlaVar == "passing":
            if self.current_lights != None:
                return 8 if self.current_lights & carla.VehicleLightState.HighBeam else 0
        elif carlaVar == "hand_brake":
            return self.hand_break_status
        elif carlaVar == "manual":
            return self.world.player.get_control().manual_gear_shift
        elif carlaVar == "lane_departure_sound":
            return None
        elif carlaVar == "collision":
            return len(self.world.collision_sensor.history)%90
        elif carlaVar == "radar":
            if self.world.radar_sensor is None:
                return None
            else :
                return int(self.world.radar_sensor.max_depth_cmd*100)
            return None
            radar_sensor = RadarSensor(self.world.player)
            detected_objects = radar_sensor._Radar_callback(weakref.ref(radar_sensor), radar_sensor.sensor)
            return detected_objects
        elif carlaVar == "lidar":
            return None
        else:
            return None

    def set_gear(self, gear):
        self.gear_value = gear

    def set_lights(self, lights):
        self.current_lights = lights

    def set_hand_brake(self, hand_brake):
        self.hand_break_status = hand_brake


    def toggle_recording(self):
        """Toggle recording on/off.
        When turning off, write recorded packets to a file.
        """
        self.recording = not self.recording
        if self.recording:
            self.world.hud.notification("Started recording CAN packets...")
            self.recorded_packets = []  # clear previous recordings
        else:

            # Filter out duplicate packets one after the other
            self.recorded_packets = [frame for i, frame in enumerate(self.recorded_packets) if i == 0 or frame != self.recorded_packets[i - 1]]


            filename = f"can_packets_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}.log"
            with open(filename, "w") as f:
                for frame in self.recorded_packets:
                    f.write(str(frame) + "\n")

            # Create special file for simulator
            packet_no = len(self.recorded_packets)

            with open(os.path.join("WOKWI", "chips", f"data_{packet_no}.h"), "w") as f:
                string = "#define MAX_MESSAGES " + str(len(self.recorded_packets) + 1) + "\n"
                string += "#define MAX_BITS 100\n"
                string += "static const char DATA[MAX_MESSAGES][MAX_BITS] = {\n"

                for frame in self.recorded_packets:
                    string += f"    \"{frame}\",\n"

                string += "};\n"
                f.write(string)

            self.world.hud.notification(f"Stopped recording. Recorded {len(self.recorded_packets)} packets to {filename}.")
            self.recorded_packets = []  # clear previous recordings

    def tick(self):
        """Process queued command packets and update report packets"""
        for can_id, packet in self.report_packets.items():
            packet.tick()
            if packet.period != None and packet.current_period <= 0:
                if (not self.filtered_ids) or (can_id in self.filtered_ids):
                    self.send_message(packet)

                packet.reset_period()
                packet.data = self.get_carla_data(packet.carlaVar)

        # Process command packets
        # If current_period is 0, send the packet and delete it
        for packet in self.command_packets:
            packet.tick()
            if packet.period != None and packet.current_period <= 0:
                if (not self.filtered_ids) or (packet.can_id in self.filtered_ids):
                    self.send_message(packet)
                self.command_packets.remove(packet)




    def add_command_packet(self, can_id):
        """Add a new command packet to the queue (allows multiple packets)"""

        if can_id in self.packet_definitions:
            details = self.packet_definitions[can_id]
            if details["level"] == "command":
                packet = CANPacket(
                    can_id=can_id,
                    src=details.get("source"),
                    dst=details.get("execution"),
                    name=details.get("name"),
                    level=details.get("level"),
                    type=details.get("type"),
                    period=details.get("period"),
                    datasize=details.get("datasize"),
                    min=details.get("min"),
                    max=details.get("max"),
                    carlaVar=details.get("carlaVar"),
                    data=self.get_carla_data(details.get("carlaVar"))
                )
                self.command_packets.append(packet)

            # else:
                # print(f"CAN ID {can_id} is not a command packet.")
        # else:
            # print(f"CAN ID {can_id} not found in CAN data.")





# ==============================================================================
# -- KeyboardControl -----------------------------------------------------------
# ==============================================================================


class KeyboardControl(object):
    """Class that handles keyboard input."""
    def __init__(self, world, start_in_autopilot):
        self._autopilot_enabled = start_in_autopilot
        self.engine_on = True  # Engine starts OFF
        self._ackermann_enabled = False
        self._ackermann_reverse = 1
        if isinstance(world.player, carla.Vehicle):
            self.window = CanJsonProcessor()
            self.can_simulator = CanSimulator(world, 'CAN_ID.json', self, gui_processor=self.window)
            self.window.show()
            self.guiEvent = False
            self._control = carla.VehicleControl()
            ###
            self._lights = carla.VehicleLightState.NONE
            world.player.set_autopilot(self._autopilot_enabled)
            world.player.set_light_state(self._lights)
        else:
            raise NotImplementedError("Actor type not supported")
        self._steer_cache = 0.0
        world.hud.notification("Press 'H' or '?' for help.", seconds=4.0)

    def parse_events(self, client, world, clock, sync_mode):
        if isinstance(self._control, carla.VehicleControl):
            current_lights = self._lights
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return True
            elif event.type == pygame.KEYUP:
                if self._is_quit_shortcut(event.key):
                    return True
                elif event.key == K_BACKSPACE:
                    if self._autopilot_enabled:
                        world.player.set_autopilot(False)
                        world.restart()
                        world.player.set_autopilot(True)
                    else:
                        world.restart()
                elif event.key == K_RETURN:
                    self.engine_on = not self.engine_on
                    state = "ON" if self.engine_on else "OFF"
                    world.hud.notification(f"Engine: {state}")
                    self.can_simulator.add_command_packet("440")
                elif event.key == K_F1:
                    world.hud.toggle_info()
                elif event.key == K_v and pygame.key.get_mods() & KMOD_SHIFT:
                    world.next_map_layer(reverse=True)
                elif event.key == K_v:
                    world.next_map_layer()
                elif event.key == K_b and pygame.key.get_mods() & KMOD_SHIFT:
                    world.load_map_layer(unload=True)
                elif event.key == K_b:
                    world.load_map_layer()
                elif event.key == K_h or (event.key == K_SLASH and pygame.key.get_mods() & KMOD_SHIFT):
                    world.hud.help.toggle()
                elif event.key == K_TAB:
                    world.camera_manager.toggle_camera()
                elif event.key == K_c and pygame.key.get_mods() & KMOD_SHIFT:
                    world.next_weather(reverse=True)
                elif event.key == K_c:
                    world.next_weather()
                elif event.key == K_g:
                    world.toggle_radar()
                elif event.key == K_BACKQUOTE:
                    world.camera_manager.next_sensor()
                elif event.key == K_n:
                    world.camera_manager.next_sensor()
                elif event.key == K_w and (pygame.key.get_mods() & KMOD_CTRL):
                    if world.constant_velocity_enabled:
                        world.player.disable_constant_velocity()
                        world.constant_velocity_enabled = False
                        world.hud.notification("Disabled Constant Velocity Mode")
                    else:
                        world.player.enable_constant_velocity(carla.Vector3D(17, 0, 0))
                        world.constant_velocity_enabled = True
                        world.hud.notification("Enabled Constant Velocity Mode at 60 km/h")
                elif event.key == K_o:
                    try:
                        if world.doors_are_open:
                            world.hud.notification("Closing Doors")
                            world.doors_are_open = False
                            world.player.close_door(carla.VehicleDoor.All)
                        else:
                            world.hud.notification("Opening doors")
                            world.doors_are_open = True
                            world.player.open_door(carla.VehicleDoor.All)
                    except Exception:
                        pass
                elif event.key == K_t:
                    if world.show_vehicle_telemetry:
                        world.player.show_debug_telemetry(False)
                        world.show_vehicle_telemetry = False
                        world.hud.notification("Disabled Vehicle Telemetry")
                    else:
                        try:
                            world.player.show_debug_telemetry(True)
                            world.show_vehicle_telemetry = True
                            world.hud.notification("Enabled Vehicle Telemetry")
                        except Exception:
                            pass
                elif event.key > K_0 and event.key <= K_9:
                    index_ctrl = 0
                    if pygame.key.get_mods() & KMOD_CTRL:
                        index_ctrl = 9
                    world.camera_manager.set_sensor(event.key - 1 - K_0 + index_ctrl)
                elif event.key == K_r and not (pygame.key.get_mods() & KMOD_CTRL):
                    world.camera_manager.toggle_recording()
                elif event.key == K_r and (pygame.key.get_mods() & KMOD_CTRL):
                    if (world.recording_enabled):
                        client.stop_recorder()
                        world.recording_enabled = False
                        world.hud.notification("Recorder is OFF")
                    else:
                        client.start_recorder("manual_recording.rec")
                        world.recording_enabled = True
                        world.hud.notification("Recorder is ON")
                elif event.key == K_p and (pygame.key.get_mods() & KMOD_CTRL):
                    # stop recorder
                    client.stop_recorder()
                    world.recording_enabled = False
                    # work around to fix camera at start of replaying
                    current_index = world.camera_manager.index
                    world.destroy_sensors()
                    # disable autopilot
                    self._autopilot_enabled = False
                    world.player.set_autopilot(self._autopilot_enabled)
                    world.hud.notification("Replaying file 'manual_recording.rec'")
                    # replayer
                    client.replay_file("manual_recording.rec", world.recording_start, 0, 0)
                    world.camera_manager.set_sensor(current_index)
                elif event.key == K_MINUS and (pygame.key.get_mods() & KMOD_CTRL):
                    if pygame.key.get_mods() & KMOD_SHIFT:
                        world.recording_start -= 10
                    else:
                        world.recording_start -= 1
                    world.hud.notification("Recording start time is %d" % (world.recording_start))
                elif event.key == K_EQUALS and (pygame.key.get_mods() & KMOD_CTRL):
                    if pygame.key.get_mods() & KMOD_SHIFT:
                        world.recording_start += 10
                    else:
                        world.recording_start += 1
                    world.hud.notification("Recording start time is %d" % (world.recording_start))
                if isinstance(self._control, carla.VehicleControl):
                    if event.key == K_q:
                        ###
                        self._control.gear = 1 if self._control.reverse else -1
                        self.can_simulator.add_command_packet("109")
                    elif event.key == K_m:
                        self._control.manual_gear_shift = not self._control.manual_gear_shift
                        self._control.gear = world.player.get_control().gear
                        self.can_simulator.set_gear(self._control.gear)
                        world.hud.notification('%s Transmission' %
                                               ('Manual' if self._control.manual_gear_shift else 'Automatic'))
                    elif self._control.manual_gear_shift and event.key == K_COMMA:
                        self._control.gear = max(-1, self._control.gear - 1)
                        self.can_simulator.set_gear(self._control.gear)
                        self.can_simulator.add_command_packet("109")
                    elif self._control.manual_gear_shift and event.key == K_PERIOD:
                        self._control.gear = self._control.gear + 1
                        self.can_simulator.set_gear(self._control.gear)
                        self.can_simulator.add_command_packet("109")
                    elif event.key == K_p and not pygame.key.get_mods() & KMOD_CTRL:
                        if not self._autopilot_enabled and not sync_mode:
                            print("WARNING: You are currently in asynchronous mode and could "
                                  "experience some issues with the traffic simulation")
                        self._autopilot_enabled = not self._autopilot_enabled
                        world.player.set_autopilot(self._autopilot_enabled)
                        world.hud.notification(
                            'Autopilot %s' % ('On' if self._autopilot_enabled else 'Off'))
                    elif event.key == K_l and pygame.key.get_mods() & KMOD_CTRL:
                        current_lights ^= carla.VehicleLightState.Special1
                    elif event.key == K_l and pygame.key.get_mods() & KMOD_SHIFT:
                        current_lights ^= carla.VehicleLightState.HighBeam
                        self.can_simulator.set_lights(current_lights)
                        self.can_simulator.add_command_packet("423")
                    elif event.key == K_e:
                        # Use 'E' to flash the lights
                        current_lights ^= carla.VehicleLightState.HighBeam
                        self.can_simulator.set_lights(current_lights)
                        self.can_simulator.add_command_packet("433")
                    elif event.key == K_l:
                        # Use 'L' key to switch between lights:
                        # closed -> position -> low beam -> fog
                        if not self._lights & carla.VehicleLightState.Position:
                            world.hud.notification("Position lights")
                            current_lights |= carla.VehicleLightState.Position
                            # print("1")
                        else:
                            world.hud.notification("Low beam lights")
                            current_lights |= carla.VehicleLightState.LowBeam
                            # print("2") # 3
                        if self._lights & carla.VehicleLightState.LowBeam:
                            world.hud.notification("Fog lights")
                            current_lights |= carla.VehicleLightState.Fog
                            # print("3") # 131
                        if self._lights & carla.VehicleLightState.Fog:
                            world.hud.notification("Lights off")
                            current_lights ^= carla.VehicleLightState.Position
                            current_lights ^= carla.VehicleLightState.LowBeam
                            current_lights ^= carla.VehicleLightState.Fog
                            # print("4")
                        self.can_simulator.set_lights(current_lights)
                        self.can_simulator.add_command_packet("423")
                    elif event.key == K_i:
                        current_lights ^= carla.VehicleLightState.Interior
                    elif event.key == K_z:
                        current_lights ^= carla.VehicleLightState.LeftBlinker
                        self.can_simulator.set_lights(current_lights)
                        self.can_simulator.add_command_packet("131")
                    elif event.key == K_x:
                        current_lights ^= carla.VehicleLightState.RightBlinker
                        self.can_simulator.set_lights(current_lights)
                        self.can_simulator.add_command_packet("131")
                    elif event.key == K_j:
                        self.can_simulator.toggle_recording()
        if self.window.input_packets is not None and self.window.input_packets:
            for packet in self.window.input_packets:
                self.guiEvent = True
                if packet["can_id"] == "440":
                    if packet["data"] is None:
                        self.engine_on = not self.engine_on
                        state = "ON" if self.engine_on else "OFF"
                        world.hud.notification(f"Engine: {state}")
                        print(self.engine_on)
                    else:
                        if packet["data"] == 1:
                            self.engine_on = True
                        elif packet["data"] == 0:
                            self.engine_on = False
                        state = "ON" if self.engine_on else "OFF"
                        world.hud.notification(f"Engine: {state}")
                elif packet["can_id"] == "433":
                    current_lights ^= carla.VehicleLightState.HighBeam
                    self.can_simulator.set_lights(current_lights)
                elif packet["can_id"] == "423":
                        if not self._lights & carla.VehicleLightState.Position:
                            world.hud.notification("Position lights")
                            current_lights |= carla.VehicleLightState.Position
                            # print("1")
                        else:
                            world.hud.notification("Low beam lights")
                            current_lights |= carla.VehicleLightState.LowBeam
                            # print("2") # 3
                            print(current_lights)
                        if self._lights & carla.VehicleLightState.LowBeam:
                            world.hud.notification("Fog lights")
                            current_lights |= carla.VehicleLightState.Fog
                            print(current_lights)
                        if self._lights & carla.VehicleLightState.Fog:
                            world.hud.notification("Lights off")
                            current_lights ^= carla.VehicleLightState.Position
                            current_lights ^= carla.VehicleLightState.LowBeam
                            current_lights ^= carla.VehicleLightState.Fog
                            # print("4")
                        self.can_simulator.set_lights(current_lights)
                elif packet["can_id"] == "131":
                    current_lights ^= carla.VehicleLightState.LeftBlinker
                    current_lights ^= carla.VehicleLightState.RightBlinker
                    self.can_simulator.set_lights(current_lights)
                elif packet["can_id"] == "109":
                    if self._control.manual_gear_shift and packet["data"] is not None:
                        self._control.gear = packet["data"]
                        self.can_simulator.set_gear(self._control.gear)
                elif packet["can_id"] == "457":
                    if packet["data"] is None:
                        self.can_simulator.set_hand_brake(False)
                    else:
                        if packet["data"] == 1:
                            self.can_simulator.set_hand_brake(True)
                        elif packet["data"] == 0:
                            self.can_simulator.set_hand_brake(False)
                elif packet["can_id"] == "26":
                    if packet["data"] is None:
                        self._control.brake = 0
                    else:
                        self._control.brake = packet["data"]
                elif packet["can_id"] == "47":
                    if packet["data"] is None:
                        self._control.throttle = 0
                    else:
                        self._control.throttle = packet["data"]
                elif packet["can_id"] == "88":
                    if packet["data"] is None:
                        self._steer_cache = 0
                    else:
                        self._steer_cache = packet["data"]
                        self._control.steer = round(self._steer_cache, 1)
                elif self.guiEvent:
                    self.guiEvent = False
            self.window.input_packets = []

        if not self._autopilot_enabled:
            if isinstance(self._control, carla.VehicleControl):
                self._parse_vehicle_keys(pygame.key.get_pressed(), clock.get_time())
                self._control.reverse = self._control.gear < 0
                # Set automatic control-related vehicle lights
                if self._control.brake:
                    current_lights |= carla.VehicleLightState.Brake
                else: # Remove the Brake flag
                    current_lights &= ~carla.VehicleLightState.Brake
                if self._control.reverse:
                    current_lights |= carla.VehicleLightState.Reverse
                else: # Remove the Reverse flag
                    current_lights &= ~carla.VehicleLightState.Reverse
                if current_lights != self._lights: # Change the light state only if necessary
                    self._lights = current_lights
                    world.player.set_light_state(carla.VehicleLightState(self._lights))
                # Apply control
                world.player.apply_control(self._control)

    def _parse_vehicle_keys(self, keys, milliseconds):

        if not self.engine_on:
            return  # Block all movement if the engine is off
        if keys[K_UP] or keys[K_w] or keys[K_DOWN] or keys[K_s] or keys[K_LEFT] or keys[K_a] or keys[K_RIGHT] or keys[K_d] or keys[K_SPACE]:
            self.guiEvent = False
        if self.guiEvent:
            return
        if keys[K_UP] or keys[K_w]:
            self.can_simulator.add_command_packet("47")
            ###
            self._control.throttle = min(self._control.throttle + 0.1, 1.00)
        else:
            if not self._ackermann_enabled:
                self._control.throttle = 0.0
        if keys[K_DOWN] or keys[K_s]:
            ###
            self._control.brake = min(self._control.brake + 0.2, 1)
            self.can_simulator.add_command_packet("26")
        else:
            if not self._ackermann_enabled:
                self._control.brake = 0

        steer_increment = 5e-4 * milliseconds
        if keys[K_LEFT] or keys[K_a]:
            if self._steer_cache > 0:
                self._steer_cache = 0
            else:
                self._steer_cache -= steer_increment
                self.can_simulator.add_command_packet("88")
        elif keys[K_RIGHT] or keys[K_d]:
            if self._steer_cache < 0:
                self._steer_cache = 0
            else:
                self._steer_cache += steer_increment
                self.can_simulator.add_command_packet("88")
        else:
            self._steer_cache = 0.0
        self._steer_cache = min(0.7, max(-0.7, self._steer_cache))

        self._control.steer = round(self._steer_cache, 1)
        # Send a CAN packet only if we started pressing the handbrake or stopped pressing it
        if keys[K_SPACE] and not self._control.hand_brake:
            self.can_simulator.set_hand_brake(True)
            self.can_simulator.add_command_packet("457")
        elif not keys[K_SPACE] and self._control.hand_brake:
            self.can_simulator.set_hand_brake(False)
            self.can_simulator.add_command_packet("457")
        self._control.hand_brake = keys[K_SPACE]

    def simulator_tick(self):
        self.can_simulator.tick()

    @staticmethod
    def _is_quit_shortcut(key):
        return (key == K_ESCAPE) or (key == K_q and pygame.key.get_mods() & KMOD_CTRL)


# ==============================================================================
# -- HUD -----------------------------------------------------------------------
# ==============================================================================


class HUD(object):
    controller = None
    def __init__(self, width, height):
        self.dim = (width, height)
        font = pygame.font.Font(pygame.font.get_default_font(), 20)
        font_name = 'courier' if os.name == 'nt' else 'mono'
        fonts = [x for x in pygame.font.get_fonts() if font_name in x]
        default_font = 'ubuntumono'
        mono = default_font if default_font in fonts else fonts[0]
        mono = pygame.font.match_font(mono)
        self._font_mono = pygame.font.Font(mono, 12 if os.name == 'nt' else 14)
        self._notifications = FadingText(font, (width, 40), (0, height - 40))
        self.help = HelpText(pygame.font.Font(mono, 16), width, height)
        self.server_fps = 0
        self.frame = 0
        self.simulation_time = 0
        self._show_info = True
        self._info_text = []
        self._server_clock = pygame.time.Clock()

    def set_controller(self, c):
        HUD.controller = c


    def on_world_tick(self, timestamp):
        self._server_clock.tick()
        self.server_fps = self._server_clock.get_fps()
        self.frame = timestamp.frame
        self.simulation_time = timestamp.elapsed_seconds

    def tick(self, world, clock):

        self._notifications.tick(world, clock)
        if not self._show_info:
            return
        t = world.player.get_transform()
        v = world.player.get_velocity()
        c = world.player.get_control()
        compass = world.imu_sensor.compass
        heading = 'N' if compass > 270.5 or compass < 89.5 else ''
        heading += 'S' if 90.5 < compass < 269.5 else ''
        heading += 'E' if 0.5 < compass < 179.5 else ''
        heading += 'W' if 180.5 < compass < 359.5 else ''
        colhist = world.collision_sensor.get_collision_history()
        collision = [colhist[x + self.frame - 200] for x in range(0, 200)]
        max_col = max(1.0, max(collision))
        collision = [x / max_col for x in collision]
        vehicles = world.world.get_actors().filter('vehicle.*')
        self._info_text = [
            'Server:  % 16.0f FPS' % self.server_fps,
            'Client:  % 16.0f FPS' % clock.get_fps(),
            '',
            'Vehicle: % 20s' % get_actor_display_name(world.player, truncate=20),
            'Map:     % 20s' % world.map.name.split('/')[-1],
            'Simulation time: % 12s' % datetime.timedelta(seconds=int(self.simulation_time)),
            '',
            'Speed:   % 15.0f km/h' % (3.6 * math.sqrt(v.x**2 + v.y**2 + v.z**2)),
            u'Compass:% 17.0f\N{DEGREE SIGN} % 2s' % (compass, heading),
            'Accelero: (%5.1f,%5.1f,%5.1f)' % (world.imu_sensor.accelerometer),
            'Gyroscop: (%5.1f,%5.1f,%5.1f)' % (world.imu_sensor.gyroscope),
            'Location:% 20s' % ('(% 5.1f, % 5.1f)' % (t.location.x, t.location.y)),
            'GNSS:% 24s' % ('(% 2.6f, % 3.6f)' % (world.gnss_sensor.lat, world.gnss_sensor.lon)),
            'Height:  % 18.0f m' % t.location.z,
            '']
        self._info_text += [
            ('Throttle:', c.throttle, 0.0, 1.0),
            ('Steer:', c.steer, -1.0, 1.0),
            ('Brake:', c.brake, 0.0, 1.0),
            ('Ignition:', HUD.controller.engine_on),
            ('Reverse:', c.reverse),
            ('Hand brake:', c.hand_brake),
            ('Manual:', c.manual_gear_shift),
            'Gear:        %s' % {-1: 'R', 0: 'N'}.get(c.gear, c.gear)]
        self._info_text += [
            '',
            'Collision:',
            collision,
            '',
            'Number of vehicles: % 8d' % len(vehicles)]
        if len(vehicles) > 1:
            self._info_text += ['Nearby vehicles:']
            distance = lambda l: math.sqrt((l.x - t.location.x)**2 + (l.y - t.location.y)**2 + (l.z - t.location.z)**2)
            vehicles = [(distance(x.get_location()), x) for x in vehicles if x.id != world.player.id]
            for d, vehicle in sorted(vehicles, key=lambda vehicles: vehicles[0]):
                if d > 200.0:
                    break
                vehicle_type = get_actor_display_name(vehicle, truncate=22)
                self._info_text.append('% 4dm %s' % (d, vehicle_type))
    def toggle_info(self):
        self._show_info = not self._show_info

    def notification(self, text, seconds=2.0):
        self._notifications.set_text(text, seconds=seconds)

    def error(self, text):
        self._notifications.set_text('Error: %s' % text, (255, 0, 0))

    def render(self, display):
        if self._show_info:
            info_surface = pygame.Surface((220, self.dim[1]))
            info_surface.set_alpha(100)
            display.blit(info_surface, (0, 0))
            v_offset = 4
            bar_h_offset = 100
            bar_width = 106
            for item in self._info_text:
                if v_offset + 18 > self.dim[1]:
                    break
                if isinstance(item, list):
                    if len(item) > 1:
                        points = [(x + 8, v_offset + 8 + (1.0 - y) * 30) for x, y in enumerate(item)]
                        pygame.draw.lines(display, (255, 136, 0), False, points, 2)
                    item = None
                    v_offset += 18
                elif isinstance(item, tuple):
                    if isinstance(item[1], bool):
                        rect = pygame.Rect((bar_h_offset, v_offset + 8), (6, 6))
                        pygame.draw.rect(display, (255, 255, 255), rect, 0 if item[1] else 1)
                    else:
                        rect_border = pygame.Rect((bar_h_offset, v_offset + 8), (bar_width, 6))
                        pygame.draw.rect(display, (255, 255, 255), rect_border, 1)
                        f = (item[1] - item[2]) / (item[3] - item[2])
                        if item[2] < 0.0:
                            rect = pygame.Rect((bar_h_offset + f * (bar_width - 6), v_offset + 8), (6, 6))
                        else:
                            rect = pygame.Rect((bar_h_offset, v_offset + 8), (f * bar_width, 6))
                        pygame.draw.rect(display, (255, 255, 255), rect)
                    item = item[0]
                if item:  # At this point has to be a str.
                    surface = self._font_mono.render(item, True, (255, 255, 255))
                    display.blit(surface, (8, v_offset))
                v_offset += 18
        self._notifications.render(display)
        self.help.render(display)


# ==============================================================================
# -- FadingText ----------------------------------------------------------------
# ==============================================================================


class FadingText(object):
    def __init__(self, font, dim, pos):
        self.font = font
        self.dim = dim
        self.pos = pos
        self.seconds_left = 0
        self.surface = pygame.Surface(self.dim)

    def set_text(self, text, color=(255, 255, 255), seconds=2.0):
        text_texture = self.font.render(text, True, color)
        self.surface = pygame.Surface(self.dim)
        self.seconds_left = seconds
        self.surface.fill((0, 0, 0, 0))
        self.surface.blit(text_texture, (10, 11))

    def tick(self, _, clock):
        delta_seconds = 1e-3 * clock.get_time()
        self.seconds_left = max(0.0, self.seconds_left - delta_seconds)
        self.surface.set_alpha(500.0 * self.seconds_left)

    def render(self, display):
        display.blit(self.surface, self.pos)


# ==============================================================================
# -- HelpText ------------------------------------------------------------------
# ==============================================================================


class HelpText(object):
    """Helper class to handle text output using pygame"""
    def __init__(self, font, width, height):
        lines = __doc__.split('\n')
        self.font = font
        self.line_space = 18
        self.dim = (780, len(lines) * self.line_space + 12)
        self.pos = (0.5 * width - 0.5 * self.dim[0], 0.5 * height - 0.5 * self.dim[1])
        self.seconds_left = 0
        self.surface = pygame.Surface(self.dim)
        self.surface.fill((0, 0, 0, 0))
        for n, line in enumerate(lines):
            text_texture = self.font.render(line, True, (255, 255, 255))
            self.surface.blit(text_texture, (22, n * self.line_space))
            self._render = False
        self.surface.set_alpha(220)

    def toggle(self):
        self._render = not self._render

    def render(self, display):
        if self._render:
            display.blit(self.surface, self.pos)


# ==============================================================================
# -- CollisionSensor -----------------------------------------------------------
# ==============================================================================


class CollisionSensor(object):
    def __init__(self, parent_actor, hud):
        self.sensor = None
        self.history = []
        self._parent = parent_actor
        self.hud = hud
        world = self._parent.get_world()
        bp = world.get_blueprint_library().find('sensor.other.collision')
        self.sensor = world.spawn_actor(bp, carla.Transform(), attach_to=self._parent)
        # We need to pass the lambda a weak reference to self to avoid circular
        # reference.
        weak_self = weakref.ref(self)
        self.sensor.listen(lambda event: CollisionSensor._on_collision(weak_self, event))

    def get_collision_history(self):
        history = collections.defaultdict(int)
        for frame, intensity in self.history:
            history[frame] += intensity
        return history

    @staticmethod
    def _on_collision(weak_self, event):
        self = weak_self()
        if not self:
            return
        actor_type = get_actor_display_name(event.other_actor)
        self.hud.controller.can_simulator.add_command_packet("1200")
        self.hud.notification('Collision with %r' % actor_type)
        impulse = event.normal_impulse
        intensity = math.sqrt(impulse.x**2 + impulse.y**2 + impulse.z**2)
        self.history.append((event.frame, intensity))
        if len(self.history) > 4000:
            self.history.pop(0)


# ==============================================================================
# -- LaneInvasionSensor --------------------------------------------------------
# ==============================================================================


class LaneInvasionSensor(object):
    def __init__(self, parent_actor, hud):
        self.sensor = None

        # If the spawn object is not a vehicle, we cannot use the Lane Invasion Sensor
        if parent_actor.type_id.startswith("vehicle."):
            self._parent = parent_actor
            self.hud = hud
            world = self._parent.get_world()
            bp = world.get_blueprint_library().find('sensor.other.lane_invasion')
            self.sensor = world.spawn_actor(bp, carla.Transform(), attach_to=self._parent)
            # We need to pass the lambda a weak reference to self to avoid circular
            # reference.
            weak_self = weakref.ref(self)
            self.sensor.listen(lambda event: LaneInvasionSensor._on_invasion(weak_self, event))


    @staticmethod
    def _on_invasion(weak_self, event):
        self = weak_self()
        if not self:
            return
        lane_types = set(x.type for x in event.crossed_lane_markings)
        text = ['%r' % str(x).split()[-1] for x in lane_types]
        self.hud.controller.can_simulator.add_command_packet("1313")
        # print(event.crossed_lane_markings)
        self.hud.notification('Crossed line %s' % ' and '.join(text))




# ==============================================================================
# -- GnssSensor ----------------------------------------------------------------
# ==============================================================================


class GnssSensor(object):
    def __init__(self, parent_actor):
        self.sensor = None
        self._parent = parent_actor
        self.lat = 0.0
        self.lon = 0.0
        world = self._parent.get_world()
        bp = world.get_blueprint_library().find('sensor.other.gnss')
        self.sensor = world.spawn_actor(bp, carla.Transform(carla.Location(x=1.0, z=2.8)), attach_to=self._parent)
        # We need to pass the lambda a weak reference to self to avoid circular
        # reference.
        weak_self = weakref.ref(self)
        self.sensor.listen(lambda event: GnssSensor._on_gnss_event(weak_self, event))

    @staticmethod
    def _on_gnss_event(weak_self, event):
        self = weak_self()
        if not self:
            return
        self.lat = event.latitude
        self.lon = event.longitude


# ==============================================================================
# -- IMUSensor -----------------------------------------------------------------
# ==============================================================================


class IMUSensor(object):
    def __init__(self, parent_actor):
        self.sensor = None
        self._parent = parent_actor
        self.accelerometer = (0.0, 0.0, 0.0)
        self.gyroscope = (0.0, 0.0, 0.0)
        self.compass = 0.0
        world = self._parent.get_world()
        bp = world.get_blueprint_library().find('sensor.other.imu')
        self.sensor = world.spawn_actor(
            bp, carla.Transform(), attach_to=self._parent)
        # We need to pass the lambda a weak reference to self to avoid circular
        # reference.
        weak_self = weakref.ref(self)
        self.sensor.listen(
            lambda sensor_data: IMUSensor._IMU_callback(weak_self, sensor_data))

    @staticmethod
    def _IMU_callback(weak_self, sensor_data):
        self = weak_self()
        if not self:
            return
        limits = (-99.9, 99.9)
        self.accelerometer = (
            max(limits[0], min(limits[1], sensor_data.accelerometer.x)),
            max(limits[0], min(limits[1], sensor_data.accelerometer.y)),
            max(limits[0], min(limits[1], sensor_data.accelerometer.z)))
        self.gyroscope = (
            max(limits[0], min(limits[1], math.degrees(sensor_data.gyroscope.x))),
            max(limits[0], min(limits[1], math.degrees(sensor_data.gyroscope.y))),
            max(limits[0], min(limits[1], math.degrees(sensor_data.gyroscope.z))))
        self.compass = math.degrees(sensor_data.compass)


# ==============================================================================
# -- RadarSensor ---------------------------------------------------------------
# ==============================================================================


class RadarSensor(object):
    def __init__(self, parent_actor):
        self.sensor = None
        self._parent = parent_actor
        self.velocity_range = 7.5 # m/s
        self.max_depth_cmd = 0.0
        world = self._parent.get_world()
        self.debug = world.debug
        bp = world.get_blueprint_library().find('sensor.other.radar')
        bp.set_attribute('horizontal_fov', str(35))
        bp.set_attribute('vertical_fov', str(20))
        self.sensor = world.spawn_actor(
            bp,
            carla.Transform(
                carla.Location(x=2.8, z=1.0),
                carla.Rotation(pitch=5)),
            attach_to=self._parent)
        # We need a weak reference to self to avoid circular reference.
        weak_self = weakref.ref(self)
        self.sensor.listen(
            lambda radar_data: RadarSensor._Radar_callback(weak_self, radar_data))

    @staticmethod
    def _Radar_callback(weak_self, radar_data):
        self = weak_self()
        if not self:
            return
        # To get a numpy [[vel, altitude, azimuth, depth],...[,,,]]:
        # points = np.frombuffer(radar_data.raw_data, dtype=np.dtype('f4'))
        # points = np.reshape(points, (len(radar_data), 4))
        current_rot = radar_data.transform.rotation

        # max_depth = max(detect.depth for detect in radar_data)

        FRONT_AZIMUTH_LIMIT = 1  # ±10 grade

        filtered = [d for d in radar_data if abs(math.degrees(d.azimuth)) < FRONT_AZIMUTH_LIMIT]
        if not filtered:
            return
        max_depth = max(d.depth for d in filtered)

        print("Max depth:", int(max_depth*1000), "mm")
        self.max_depth_cmd = max_depth
        for detect in radar_data:
            azi = math.degrees(detect.azimuth)
            alt = math.degrees(detect.altitude)
            # The 0.25 adjusts a bit the distance so the dots can
            # be properly seen
            fw_vec = carla.Vector3D(x=detect.depth - 0.25)
            carla.Transform(
                carla.Location(),
                carla.Rotation(
                    pitch=current_rot.pitch + alt,
                    yaw=current_rot.yaw + azi,
                    roll=current_rot.roll)).transform(fw_vec)

            def clamp(min_v, max_v, value):
                return max(min_v, min(value, max_v))

            norm_velocity = detect.velocity / self.velocity_range # range [-1, 1]
            r = int(clamp(0.0, 1.0, 1.0 - norm_velocity) * 255.0)
            g = int(clamp(0.0, 1.0, 1.0 - abs(norm_velocity)) * 255.0)
            b = int(abs(clamp(- 1.0, 0.0, - 1.0 - norm_velocity)) * 255.0)
            self.debug.draw_point(
                radar_data.transform.location + fw_vec,
                size=0.075,
                life_time=0.06,
                persistent_lines=False,
                color=carla.Color(r, g, b))

# ==============================================================================
# -- CameraManager -------------------------------------------------------------
# ==============================================================================

###

class CameraManager(object):
    def __init__(self, parent_actor, hud, gamma_correction):
        self.sensor = None
        self.surface = None
        self._parent = parent_actor
        self.hud = hud
        self.recording = False
        bound_y = 0.5 + self._parent.bounding_box.extent.y
        Attachment = carla.AttachmentType
        self._camera_transforms = [
            (carla.Transform(carla.Location(x=-5.5, z=2.5), carla.Rotation(pitch=8.0)), Attachment.SpringArm),
            (carla.Transform(carla.Location(x=1.6, z=1.7)), Attachment.Rigid),
            (carla.Transform(carla.Location(x=5.5, y=1.5, z=1.5)), Attachment.SpringArm),
            (carla.Transform(carla.Location(x=-8.0, z=6.0), carla.Rotation(pitch=6.0)), Attachment.SpringArm),
            (carla.Transform(carla.Location(x=-1, y=-bound_y, z=0.5)), Attachment.Rigid)]
        self.transform_index = 1
        self.sensors = [
            ['sensor.camera.rgb', cc.Raw, 'Camera RGB', {}],
            ['sensor.camera.depth', cc.Raw, 'Camera Depth (Raw)', {}],
            ['sensor.camera.depth', cc.Depth, 'Camera Depth (Gray Scale)', {}],
            ['sensor.camera.depth', cc.LogarithmicDepth, 'Camera Depth (Logarithmic Gray Scale)', {}],
            ['sensor.camera.semantic_segmentation', cc.Raw, 'Camera Semantic Segmentation (Raw)', {}],
            ['sensor.camera.semantic_segmentation', cc.CityScapesPalette,
                'Camera Semantic Segmentation (CityScapes Palette)', {}],
            ['sensor.lidar.ray_cast', None, 'Lidar (Ray-Cast)', {'range': '50'}],
            ['sensor.camera.dvs', cc.Raw, 'Dynamic Vision Sensor', {}],
            ['sensor.camera.rgb', cc.Raw, 'Camera RGB Distorted',
                {'lens_circle_multiplier': '3.0',
                'lens_circle_falloff': '3.0',
                'chromatic_aberration_intensity': '0.5',
                'chromatic_aberration_offset': '0'}]]
        world = self._parent.get_world()
        bp_library = world.get_blueprint_library()
        for item in self.sensors:
            bp = bp_library.find(item[0])
            if item[0].startswith('sensor.camera'):
                bp.set_attribute('image_size_x', str(hud.dim[0]))
                bp.set_attribute('image_size_y', str(hud.dim[1]))
                if bp.has_attribute('gamma'):
                    bp.set_attribute('gamma', str(gamma_correction))
                for attr_name, attr_value in item[3].items():
                    bp.set_attribute(attr_name, attr_value)
            elif item[0].startswith('sensor.lidar'):
                self.lidar_range = 50

                for attr_name, attr_value in item[3].items():
                    bp.set_attribute(attr_name, attr_value)
                    if attr_name == 'range':
                        self.lidar_range = float(attr_value)

            item.append(bp)
        self.index = None

    def toggle_camera(self):
        self.transform_index = (self.transform_index + 1) % len(self._camera_transforms)
        self.set_sensor(self.index, notify=False, force_respawn=True)

    def set_sensor(self, index, notify=True, force_respawn=False):
        index = index % len(self.sensors)
        needs_respawn = True if self.index is None else \
            (force_respawn or (self.sensors[index][2] != self.sensors[self.index][2]))
        if needs_respawn:
            if self.sensor is not None:
                self.sensor.destroy()
                self.surface = None
            self.sensor = self._parent.get_world().spawn_actor(
                self.sensors[index][-1],
                self._camera_transforms[self.transform_index][0],
                attach_to=self._parent,
                attachment_type=self._camera_transforms[self.transform_index][1])
            # We need to pass the lambda a weak reference to self to avoid
            # circular reference.
            weak_self = weakref.ref(self)
            self.sensor.listen(lambda image: CameraManager._parse_image(weak_self, image))
        if notify:
            self.hud.notification(self.sensors[index][2])
        self.index = index

    def next_sensor(self):
        self.set_sensor(self.index + 1)

    def toggle_recording(self):
        self.recording = not self.recording
        self.hud.notification('Recording %s' % ('On' if self.recording else 'Off'))

    def render(self, display):
        if self.surface is not None:
            display.blit(self.surface, (0, 0))

    @staticmethod
    def _parse_image(weak_self, image):
        self = weak_self()
        if not self:
            return
        if self.sensors[self.index][0].startswith('sensor.lidar'):
            points = np.frombuffer(image.raw_data, dtype=np.dtype('f4'))
            points = np.reshape(points, (int(points.shape[0] / 4), 4))
            lidar_data = np.array(points[:, :2])
            lidar_data *= min(self.hud.dim) / (2.0 * self.lidar_range)
            lidar_data += (0.5 * self.hud.dim[0], 0.5 * self.hud.dim[1])
            lidar_data = np.fabs(lidar_data)  # pylint: disable=E1111
            lidar_data = lidar_data.astype(np.int32)
            lidar_data = np.reshape(lidar_data, (-1, 2))
            lidar_img_size = (self.hud.dim[0], self.hud.dim[1], 3)
            lidar_img = np.zeros((lidar_img_size), dtype=np.uint8)
            lidar_img[tuple(lidar_data.T)] = (255, 255, 255)
            self.surface = pygame.surfarray.make_surface(lidar_img)
        elif self.sensors[self.index][0].startswith('sensor.camera.dvs'):
            # Example of converting the raw_data from a carla.DVSEventArray
            # sensor into a NumPy array and using it as an image
            dvs_events = np.frombuffer(image.raw_data, dtype=np.dtype([
                ('x', np.uint16), ('y', np.uint16), ('t', np.int64), ('pol', np.bool)]))
            dvs_img = np.zeros((image.height, image.width, 3), dtype=np.uint8)
            # Blue is positive, red is negative
            dvs_img[dvs_events[:]['y'], dvs_events[:]['x'], dvs_events[:]['pol'] * 2] = 255
            self.surface = pygame.surfarray.make_surface(dvs_img.swapaxes(0, 1))
        elif self.sensors[self.index][0].startswith('sensor.camera.optical_flow'):
            image = image.get_color_coded_flow()
            array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
            array = np.reshape(array, (image.height, image.width, 4))
            array = array[:, :, :3]
            array = array[:, :, ::-1]
            self.surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
        else:
            image.convert(self.sensors[self.index][1])
            array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
            array = np.reshape(array, (image.height, image.width, 4))
            array = array[:, :, :3]
            array = array[:, :, ::-1]
            self.surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
        if self.recording:
            image.save_to_disk('_out/%08d' % image.frame)


# ==============================================================================
# -- game_loop() ---------------------------------------------------------------
# ==============================================================================


def game_loop(args):
    pygame.init()
    pygame.font.init()
    world = None
    original_settings = None

    try:
        client = carla.Client(args.host, args.port)
        client.set_timeout(2000.0)

        sim_world = client.get_world()
        if args.sync:
            original_settings = sim_world.get_settings()
            settings = sim_world.get_settings()
            if not settings.synchronous_mode:
                settings.synchronous_mode = True
                settings.fixed_delta_seconds = 0.05
            sim_world.apply_settings(settings)

            traffic_manager = client.get_trafficmanager()
            traffic_manager.set_synchronous_mode(True)

        if args.autopilot and not sim_world.get_settings().synchronous_mode:
            print("WARNING: You are currently in asynchronous mode and could "
                  "experience some issues with the traffic simulation")

        display = pygame.display.set_mode(
            (args.width, args.height),
            pygame.HWSURFACE | pygame.DOUBLEBUF)
        display.fill((0,0,0))
        pygame.display.flip()

        hud = HUD(args.width, args.height)
        world = World(sim_world, hud, args)
        controller = KeyboardControl(world, args.autopilot)
        hud.set_controller(controller)

        if args.sync:
            sim_world.tick()
        else:
            sim_world.wait_for_tick()

        clock = pygame.time.Clock()
        while True:
            if args.sync:
                sim_world.tick()
            clock.tick_busy_loop(60)
            if controller.parse_events(client, world, clock, args.sync):
                return
            world.tick(clock)
            controller.simulator_tick()
            world.render(display)
            pygame.display.flip()

    finally:

        if original_settings:
            sim_world.apply_settings(original_settings)

        if (world and world.recording_enabled):
            client.stop_recorder()

        if world is not None:
            world.destroy()

        pygame.quit()


# ==============================================================================
# -- main() --------------------------------------------------------------------
# ==============================================================================


def main():
    argparser = argparse.ArgumentParser(
        description='CARLA Manual Control Client')
    argparser.add_argument(
        '-v', '--verbose',
        action='store_true',
        dest='debug',
        help='print debug information')
    argparser.add_argument(
        '--host',
        metavar='H',
        default='127.0.0.1',
        help='IP of the host server (default: 127.0.0.1)')
    argparser.add_argument(
        '-p', '--port',
        metavar='P',
        default=2000,
        type=int,
        help='TCP port to listen to (default: 2000)')
    argparser.add_argument(
        '-a', '--autopilot',
        action='store_true',
        help='enable autopilot')
    argparser.add_argument(
        '--res',
        metavar='WIDTHxHEIGHT',
        default='1280x720',
        help='window resolution (default: 1280x720)')
    argparser.add_argument(
        '--filter',
        metavar='PATTERN',
        default='vehicle.*',
        help='actor filter (default: "vehicle.*")')
    argparser.add_argument(
        '--generation',
        metavar='G',
        default='2',
        help='restrict to certain actor generation (values: "1","2","All" - default: "2")')
    argparser.add_argument(
        '--rolename',
        metavar='NAME',
        default='hero',
        help='actor role name (default: "hero")')
    argparser.add_argument(
        '--gamma',
        default=2.2,
        type=float,
        help='Gamma correction of the camera (default: 2.2)')
    argparser.add_argument(
        '--sync',
        action='store_true',
        help='Activate synchronous mode execution')
    args = argparser.parse_args()

    args.width, args.height = [int(x) for x in args.res.split('x')]

    log_level = logging.DEBUG if args.debug else logging.INFO
    logging.basicConfig(format='%(levelname)s: %(message)s', level=log_level)

    logging.info('listening to server %s:%s', args.host, args.port)

    print(__doc__)

    try:

        game_loop(args)

    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')



if __name__ == '__main__':

    app = QApplication(sys.argv)


    main()