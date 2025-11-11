"""
This file stores the configuration information for the simulator.
"""

# This file is part of SimMeR, an educational mechatronics robotics simulator.
# Initial development funded by the University of Toronto MIE Department.
# Copyright (C) 2023  Ian G. Bennett
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU Affero General Public License as published
# by the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Affero General Public License for more details.
#
# You should have received a copy of the GNU Affero General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.

import pygame.math as pm
from devices.motors import MotorSimple
from devices.ultrasonic import Ultrasonic
from devices.gyroscope import Gyroscope
from devices.compass import Compass
from devices.infrared import Infrared
from devices.drive import Drive
import math

# Control Flags and Setup
rand_error = False  # Use either true random error generator (True) or repeatable error generation (False)
rand_bias = False  # Use a randomized, normally distributed set of bias values for drives (placeholder, not implemented)
bias_strength = [
    0.05,
    1,
]  # How intense the random drive bias is, if enabled (placeholder, not implemented)

# Network configuration for sockets
host = "127.0.0.1"
port_rx = 61200
port_tx = 61201
timeout = 300
str_encoding = "ascii"
frame_start = "["
frame_end = "]"

# General communication settings
round_digits = 3

# Block information
block_position = [-12, -12]  # Block starting location
block_rotation = 0  # Block rotation (deg)
block_size = 3  # Block side length in inches

# Robot information
robot_start_position = [6, 42]  # Robot starting location (in)
robot_start_rotation = 180  # Robot starting rotation (deg)
robot_width = 6  # Robot width in inches
robot_height = 6  # Robot height in inches
# robot_outline = [  # Robot outline, relative to center position
#     pm.Vector2(-robot_width / 2, -robot_width / 2),
#     pm.Vector2(-robot_width / 2, robot_width / 2),
#     pm.Vector2(robot_width / 2, robot_width / 2),
#     pm.Vector2(robot_width / 2, -robot_width / 2),
# ]

# Maze definition information
wall_segment_length = 12  # Length of maze wall segments (inches)
floor_segment_length = 3  # Size of floor pattern squares (inches)
walls = [
    [3, 3, 1, 1, 0, 2, 0, 2],
    [3, 3, 0, 1, 1, 1, 1, 1],
    [1, 0, 2, 0, 0, 1, 0, 1],
    [1, 1, 1, 1, 1, 1, 0, 2],
]  # Matrix to define the maze walls
floor_seed = 5489  # Randomization seed for generating correctfloor pattern
maze_dim_x = len(walls[0]) * wall_segment_length
maze_dim_y = len(walls) * wall_segment_length


# Graphics information
frame_rate = 60  # Target frame rate (Hz)
ppi = 12  # Number of on-screen pixels per inch on display
border_pixels = (
    floor_segment_length * ppi
)  # Size of the border surrounding the maze area

background_color = (43, 122, 120)

wall_thickness = 0.25  # Thickness to draw wall segments, in inches
wall_color = (255, 0, 0)  # Tuple with wall color in (R,G,B) format

robot_thickness = 0.25  # Thickness to draw robot perimeter, in inches
robot_color = (0, 0, 255)  # Tuple with robot perimeter color in (R,G,B) format

block_thickness = 0.25  # Thickness to draw robot perimeter, in inches
block_color = (127, 127, 0)  # Tuple with robot perimeter color in (R,G,B) format

radius = 3.75

robot_outline = [
    pm.Vector2(
        (radius - 0.01) * math.cos(2 * math.pi * i / 16),
        (radius - 0.01) * math.sin(2 * math.pi * i / 16)
    )
    for i in range(16)
]

# Additional motor (just used to mark the front of the robot)
c0_info = {
    "id": "c0",
    "position": [0, 2.5],
    "rotation": 90,
    "visible": True}

# robot_outline = [
#     pm.Vector2(-3.75, -3.75),
#     pm.Vector2(-3.75, 3.75),
#     pm.Vector2(3.75, 3.75),
#     pm.Vector2(3.75, -3.75),
# ]

# Motors
m0_info = {"id": "m0", "position": [3.75 * math.sqrt(3)/2, 1.5], "rotation": 30, "visible": True}
m1_info = {"id": "m1", "position": [-3.75 * math.sqrt(3)/2, 1.5], "rotation": 150, "visible": True}
m2_info = {"id": "m2", "position": [0, -3], "rotation": 270, "visible": True}

motors = {"m0": MotorSimple(m0_info),
          "m1": MotorSimple(m1_info),
          "m2": MotorSimple(m2_info)}

w0_info = {
    "id": "w1",
    "position": [0, 0],
    "rotation": 0,
    "visible": False,
    "velocity": [0, 3],
    "ang_velocity": 0.01,
    "motors": [motors["m0"], motors["m1"], motors["m2"]],
    "motor_direction": [math.sqrt(3) / 2, -math.sqrt(3) / 2, 0],
    "bias": {"x": 0, "y": 0, "rotation": -1.0},
    "error": {"x": 0.01, "y": 0.01, "rotation": 0.005}
}

r0_info = {
    "id": "w1",
    "position": [0, 0],
    "rotation": 0,
    "visible": False,
    "velocity": [0, 0],
    "ang_velocity": 60,
    "motors": [motors["m0"], motors["m1"], motors["m2"]],
    "motor_direction": [1, 1, 1],
    "bias": {"x": 0, "y": 0, "rotation": 0.0},
    "error": {"x": 0.05, "y": 0.05, "rotation": 0.02}
}

drives = {"w0": Drive(w0_info),
          "r0": Drive(r0_info)
         }

sensors = {}
for num_sensors in range(6):
    radians = math.pi/2 - 2 * math.pi * (num_sensors / 6)
    sensor_dict = {"id": f"s{num_sensors}",
            "position": [radius * math.cos(radians), radius * math.sin(radians)],
            "height": 1,
            "rotation": 180 / math.pi * (radians - math.pi/2),
            "error": 0.05,
            "outline": [
                pm.Vector2(-0.5, -0.5),
                pm.Vector2(-0.5, 0.5),
                pm.Vector2(0.5, 0.5),
                pm.Vector2(0.5, -0.5),
            ],
            "visible": True,
            "visible_measurement": True,
    }
    sensors[f"s{num_sensors}"] = Ultrasonic(sensor_dict)

sensors["c0"] = MotorSimple(c0_info)

### TESTING AND DEBUG SETTINGS ###
simulate_list = [f"s{u}" for u in range(6)]
