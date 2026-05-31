#!/usr/bin/env python3

import random
import math
import argparse
import struct
import sys
import os

import numpy as np

# Argument Parser for Command Line Execution
parser = argparse.ArgumentParser(description='Generate a Gazebo world with dense obstacles.')
parser.add_argument('--count', type=int, default=200, help='Target number of obstacles (default: 250)')
parser.add_argument('--min_dist', type=float, default=1.5, help='Minimum distance between obstacle centers (default: 1.4m)')
parser.add_argument('--seed', type=int, default=42, help='Random seed for deterministic generation (default: 42)')
parser.add_argument('--output', type=str, default=None, help='Output file path (default: vehicle_simulator/world/dense_obstacles.world)')
parser.add_argument('--map-output', type=str, default=None,
                    help='Output path for 2D occupancy grid map (.pgm). '
                         'A companion .yaml is written alongside it. '
                         'Default: <world_dir>/dense_obstacles_map.pgm')
parser.add_argument('--map-resolution', type=float, default=0.05,
                    help='Map resolution in meters/pixel (default: 0.05)')

args = parser.parse_args()

# Deterministic Seed
random.seed(args.seed)

# Parameters
TARGET_COUNT = args.count
MIN_DIST = args.min_dist
CORNER_SAFE_RAD = 5.5

obstacles = []
placed = []
obstacle_meta = []  # geometry metadata for map generation
corners = [(-18, -18), (18, -18), (-18, 18), (18, 18)]

def in_corner(x, y, radius=5.5):
    for cx, cy in corners:
        if math.sqrt((x - cx)**2 + (y - cy)**2) < radius:
            return True
    return False

def too_close(x, y, min_dist=MIN_DIST):
    for px, py in placed:
        # Simple Euclidean distance check
        if math.sqrt((x - px)**2 + (y - py)**2) < min_dist:
            return True
    return False

# Generation Loop
attempts = 0
max_attempts = 200000

print(f"Generating World: Target={TARGET_COUNT}, MinDist={MIN_DIST}, Seed={args.seed}")

while len(obstacles) < TARGET_COUNT and attempts < max_attempts:
    attempts += 1
    # Generate random position in [-16.5, 16.5]
    x = random.uniform(-16.5, 16.5)
    y = random.uniform(-16.5, 16.5)
    
    # Validation
    if in_corner(x, y, CORNER_SAFE_RAD):
        continue
    if too_close(x, y, MIN_DIST):
        continue

    # Create Obstacle
    is_cyl = random.random() < 0.6
    idx = len(obstacles)
    
    # Introduce some small variation in size
    if is_cyl:
        r = round(random.uniform(0.15, 0.35), 2)
        h = round(random.uniform(0.8, 1.5), 1)
        # XML formatting
        obs = (
            f'    <model name="obs_{idx:03d}">\n'
            f'      <static>1</static>\n'
            f'      <link name="link">\n'
            f'        <collision name="collision"><geometry><cylinder><radius>{r}</radius><length>{h}</length></cylinder></geometry></collision>\n'
            f'        <visual name="visual"><geometry><cylinder><radius>{r}</radius><length>{h}</length></cylinder></geometry>\n'
            f'          <material><script><uri>file://media/materials/scripts/gazebo.material</uri><name>Gazebo/DarkGrey</name></script></material></visual>\n'
            f'      </link>\n'
            f'      <pose>{x:.2f} {y:.2f} {h/2:.2f} 0 0 0</pose>\n'
            f'    </model>\n'
        )
    else:
        # Keep boxes smaller to help passage
        sx = round(random.uniform(0.3, 0.6), 2) 
        sy = round(random.uniform(0.3, 0.6), 2)
        h = round(random.uniform(0.8, 1.5), 1)
        yaw = round(random.uniform(-3.14, 3.14), 2)
        obs = (
            f'    <model name="obs_{idx:03d}">\n'
            f'      <static>1</static>\n'
            f'      <link name="link">\n'
            f'        <collision name="collision"><geometry><box><size>{sx} {sy} {h}</size></box></geometry></collision>\n'
            f'        <visual name="visual"><geometry><box><size>{sx} {sy} {h}</size></box></geometry>\n'
            f'          <material><script><uri>file://media/materials/scripts/gazebo.material</uri><name>Gazebo/Wood</name></script></material></visual>\n'
            f'      </link>\n'
            f'      <pose>{x:.2f} {y:.2f} {h/2:.2f} 0 0 {yaw}</pose>\n'
            f'    </model>\n'
        )
        
    obstacles.append(obs)
    placed.append((x, y))
    if is_cyl:
        obstacle_meta.append({"type": "cylinder", "x": x, "y": y, "r": r})
    else:
        obstacle_meta.append({"type": "box", "x": x, "y": y, "sx": sx, "sy": sy, "yaw": yaw})

# World Template
header = """<sdf version='1.6'>
  <world name='default'>
    <light name='sun' type='directional'>
      <cast_shadows>1</cast_shadows>
      <pose frame=''>0 0 10 0 -0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.5 0.1 -0.9</direction>
    </light>
    <gravity>0 0 -9.8</gravity>
    <magnetic_field>6e-06 2.3e-05 -4.2e-05</magnetic_field>
    <atmosphere type='adiabatic'/>
    <physics name='default_physics' default='0' type='ode'>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>
    <scene>
      <ambient>0.4 0.4 0.4 1</ambient>
      <background>0.7 0.7 0.7 1</background>
      <shadows>1</shadows>
    </scene>
    <wind/>
    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <latitude_deg>0</latitude_deg>
      <longitude_deg>0</longitude_deg>
      <elevation>0</elevation>
      <heading_deg>0</heading_deg>
    </spherical_coordinates>

    <!-- Ground Plane -->
    <model name='ground_plane'>
      <static>1</static>
      <link name='link'>
        <collision name='collision'>
          <geometry><plane><normal>0 0 1</normal><size>80 80</size></plane></geometry>
          <surface><friction><ode><mu>0.5</mu><mu2>0.5</mu2></ode><torsional><ode/></torsional></friction><contact><ode/></contact><bounce/></surface>
          <max_contacts>10</max_contacts>
        </collision>
        <visual name='visual'>
          <cast_shadows>0</cast_shadows>
          <geometry><plane><normal>0 0 1</normal><size>80 80</size></plane></geometry>
          <material><script><uri>file://media/materials/scripts/gazebo.material</uri><name>Gazebo/Grey</name></script></material>
        </visual>
      </link>
      <pose>0 0 0 0 0 0</pose>
    </model>

    <!-- Boundary Walls (40m x 40m) -->
    <model name='wall_north'>
      <static>1</static>
      <link name='link'>
        <collision name='collision'><geometry><box><size>40.4 0.2 1.5</size></box></geometry></collision>
        <visual name='visual'><geometry><box><size>40.4 0.2 1.5</size></box></geometry>
          <material><script><uri>file://media/materials/scripts/gazebo.material</uri><name>Gazebo/Grey</name></script></material></visual>
      </link>
      <pose>0 20.1 0.75 0 0 0</pose>
    </model>
    <model name='wall_south'>
      <static>1</static>
      <link name='link'>
        <collision name='collision'><geometry><box><size>40.4 0.2 1.5</size></box></geometry></collision>
        <visual name='visual'><geometry><box><size>40.4 0.2 1.5</size></box></geometry>
          <material><script><uri>file://media/materials/scripts/gazebo.material</uri><name>Gazebo/Grey</name></script></material></visual>
      </link>
      <pose>0 -20.1 0.75 0 0 0</pose>
    </model>
    <model name='wall_east'>
      <static>1</static>
      <link name='link'>
        <collision name='collision'><geometry><box><size>0.2 40.4 1.5</size></box></geometry></collision>
        <visual name='visual'><geometry><box><size>0.2 40.4 1.5</size></box></geometry>
          <material><script><uri>file://media/materials/scripts/gazebo.material</uri><name>Gazebo/Grey</name></script></material></visual>
      </link>
      <pose>20.1 0 0.75 0 0 0</pose>
    </model>
    <model name='wall_west'>
      <static>1</static>
      <link name='link'>
        <collision name='collision'><geometry><box><size>0.2 40.4 1.5</size></box></geometry></collision>
        <visual name='visual'><geometry><box><size>0.2 40.4 1.5</size></box></geometry>
          <material><script><uri>file://media/materials/scripts/gazebo.material</uri><name>Gazebo/Grey</name></script></material></visual>
      </link>
      <pose>-20.1 0 0.75 0 0 0</pose>
    </model>

"""

footer = """
    <!-- Gazebo GUI -->
    <gui fullscreen='0'>
      <camera name='user_camera'>
        <pose frame=''>0 -45 35 0 0.7 1.57</pose>
        <view_controller>orbit</view_controller>
        <projection_type>perspective</projection_type>
      </camera>
    </gui>

  </world>
</sdf>
"""

# Determine Output Path
if args.output:
    outpath = args.output
else:
    # Try to find default path relative to this script
    script_dir = os.path.dirname(os.path.abspath(__file__))
    # Assuming standard ROS package layout: src/vehicle_simulator/scripts/gen_obstacles.py
    # World folder: src/vehicle_simulator/world/
    outpath = os.path.join(script_dir, '../world/dense_obstacles.world')
    outpath = os.path.normpath(outpath)

# Verify Directory
out_dir = os.path.dirname(outpath)
if not os.path.exists(out_dir):
    print(f"Error: Output directory {out_dir} does not exist. Please specify a valid output path.")
    sys.exit(1)

# Write File
with open(outpath, 'w') as f:
    f.write(header)
    f.write(f'    <!-- Generated by gen_obstacles.py -->\n')
    f.write(f'    <!-- Parameters: Count={len(obstacles)}, MinDist={MIN_DIST}, Seed={args.seed} -->\n\n')
    for obs in obstacles:
        f.write(obs)
    f.write(footer)

print(f'Done. Generated {len(obstacles)} obstacles to {outpath}')

# --- 2D Occupancy Grid Map Generation ---

MAP_SIZE = 40.0          # meters (from -20 to +20)
MAP_ORIGIN = -20.0       # origin x and y
WALL_HALF_THICKNESS = 0.1
WALL_POS = 20.1          # wall center positions (±20.1)
WALL_HALF_LEN = 20.2     # half-length of each wall segment

map_output = args.map_output
if map_output is None:
    map_output = os.path.join(out_dir, 'dense_obstacles_map.pgm')

res = args.map_resolution
grid_size = int(round(MAP_SIZE / res))
grid = np.full((grid_size, grid_size), 254, dtype=np.uint8)  # 254 = free


def world_to_pixel(wx, wy):
    px = int(round((wx - MAP_ORIGIN) / res))
    py = int(round((wy - MAP_ORIGIN) / res))
    return px, py


def fill_circle(grid, cx, cy, radius):
    px_c, py_c = world_to_pixel(cx, cy)
    r_px = int(math.ceil(radius / res)) + 1
    for dy in range(-r_px, r_px + 1):
        for dx in range(-r_px, r_px + 1):
            px, py = px_c + dx, py_c + dy
            if 0 <= px < grid_size and 0 <= py < grid_size:
                wx = MAP_ORIGIN + px * res
                wy = MAP_ORIGIN + py * res
                if (wx - cx)**2 + (wy - cy)**2 <= radius**2:
                    grid[grid_size - 1 - py, px] = 0


def fill_box(grid, cx, cy, sx, sy, yaw):
    cos_a, sin_a = math.cos(-yaw), math.sin(-yaw)
    half_diag = math.sqrt(sx**2 + sy**2) / 2.0
    px_c, py_c = world_to_pixel(cx, cy)
    r_px = int(math.ceil(half_diag / res)) + 1
    hx, hy = sx / 2.0, sy / 2.0
    for dy in range(-r_px, r_px + 1):
        for dx in range(-r_px, r_px + 1):
            px, py = px_c + dx, py_c + dy
            if 0 <= px < grid_size and 0 <= py < grid_size:
                wx = MAP_ORIGIN + px * res - cx
                wy = MAP_ORIGIN + py * res - cy
                lx = cos_a * wx - sin_a * wy
                ly = sin_a * wx + cos_a * wy
                if abs(lx) <= hx and abs(ly) <= hy:
                    grid[grid_size - 1 - py, px] = 0


# Draw walls
for wy in range(grid_size):
    for wx in range(grid_size):
        world_x = MAP_ORIGIN + wx * res
        world_y = MAP_ORIGIN + wy * res
        in_wall = False
        if abs(world_y - WALL_POS) <= WALL_HALF_THICKNESS and abs(world_x) <= WALL_HALF_LEN:
            in_wall = True
        if abs(world_y + WALL_POS) <= WALL_HALF_THICKNESS and abs(world_x) <= WALL_HALF_LEN:
            in_wall = True
        if abs(world_x - WALL_POS) <= WALL_HALF_THICKNESS and abs(world_y) <= WALL_HALF_LEN:
            in_wall = True
        if abs(world_x + WALL_POS) <= WALL_HALF_THICKNESS and abs(world_y) <= WALL_HALF_LEN:
            in_wall = True
        if in_wall:
            grid[grid_size - 1 - wy, wx] = 0

# Draw obstacles
for meta in obstacle_meta:
    if meta["type"] == "cylinder":
        fill_circle(grid, meta["x"], meta["y"], meta["r"])
    else:
        fill_box(grid, meta["x"], meta["y"], meta["sx"], meta["sy"], meta["yaw"])

# Write PGM (P5 binary)
with open(map_output, 'wb') as f:
    header_str = f"P5\n{grid_size} {grid_size}\n255\n"
    f.write(header_str.encode('ascii'))
    f.write(grid.tobytes())

# Write companion YAML for map_server
yaml_path = map_output.replace('.pgm', '.yaml')
with open(yaml_path, 'w') as f:
    f.write(f"image: {os.path.basename(map_output)}\n")
    f.write(f"resolution: {res}\n")
    f.write(f"origin: [{MAP_ORIGIN}, {MAP_ORIGIN}, 0.0]\n")
    f.write("negate: 0\n")
    f.write("occupied_thresh: 0.65\n")
    f.write("free_thresh: 0.196\n")

print(f"Map generated: {map_output} ({grid_size}x{grid_size} @ {res}m/px)")
print(f"Map YAML:      {yaml_path}")
