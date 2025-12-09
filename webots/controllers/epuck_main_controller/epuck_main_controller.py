from controller import Robot
from odom import Odometry
from slam import OccupancyGrid, MapLocalizer
import math
import json

robot = Robot()
timestep = int(robot.getBasicTimeStep())

# Encoders
left_ps = robot.getDevice('left wheel sensor')
right_ps = robot.getDevice('right wheel sensor')
left_ps.enable(timestep)
right_ps.enable(timestep)

# LIDAR (LDS-01)
lidar = robot.getDevice('LDS-01')  # use the exact device name from the TurtleBot proto if different
lidar.enable(timestep)
try:
    lidar.enablePointCloud()
except Exception:
    pass

# helper to sample lidar ranges at desired sensor_angles (angles are relative to robot heading)
def lidar_sample_ranges(lidar, sensor_angles, max_range):
    scan = lidar.getRangeImage()  # array len 360 of range values, starting at 180 and going clockwise
    ranges = scan[180:] + scan[:180]  # reorder to start from in front
    ranges = [max_range if not (0 < r <= max_range) else r for r in ranges]
    return ranges

# ODOM
odom = Odometry(wheel_radius=0.033, axle_length=0.178)

# LiDAR beam angles (0..359 degrees -> radians)
sensor_angles = [math.radians(angle) for angle in range(360)]

# use lidar max range for localisation
max_range = lidar.getMaxRange()

# ================= FIXED MAP SETUP =================

map_width_m = 2.0
map_height_m = 2.0
resolution = 0.02

grid = OccupancyGrid(map_width_m, map_height_m, resolution)

# If you have a saved map from a previous mapping run, load it here:
# with open("saved_map.json", "r") as f:
#     grid.log_odds = json.load(f)

# Localizer on this (fixed) map
localizer = MapLocalizer(grid, sensor_angles, max_range)

# MOTORS
left_motor = robot.getDevice("left wheel motor")
left_motor.setPosition(float('inf'))
left_motor.setVelocity(0.0)

right_motor = robot.getDevice("right wheel motor")
right_motor.setPosition(float('inf'))
right_motor.setVelocity(0.0)

# gyro (optional, unused right now)
gyro = robot.getDevice("gyro")
gyro.enable(timestep)

keyboard = robot.getKeyboard()
keyboard.enable(timestep)

robot.step(timestep)  # initial step to get sensor readings
odom.reset(left_ps.getValue(), right_ps.getValue())

while robot.step(timestep) != -1:
    # ----- ODOM -----
    pose_odom = odom.update(left_ps.getValue(), right_ps.getValue())
    print("ODOM Pose:", pose_odom)

    # ----- LIDAR RANGES -----
    ranges = lidar_sample_ranges(lidar, sensor_angles, max_range)

    # ----- LOCALISATION -----
    if robot.getTime() % 3.2 < timestep / 1000.0:  # update every 3.2 seconds
        pose_est = localizer.update(pose_odom, ranges)
        print("EST Pose:", pose_est)

    # Keyboard control
    left_speed = 0.0
    right_speed = 0.0
    
    key = keyboard.getKey()
    if key == ord('W'):  # Forward
        left_speed = 2.0
        right_speed = 2.0
    elif key == ord('S'):  # Backward
        left_speed = -2.0
        right_speed = -2.0
    elif key == ord('A'):  # Turn left
        left_speed = -1.0
        right_speed = 1.0
    elif key == ord('D'):  # Turn right
        left_speed = 1.0
        right_speed = -1.0
    elif key == ord('Q'):  # Forward left
        left_speed = 1.0
        right_speed = 2.0
    elif key == ord('E'):  # Forward right
        left_speed = 2.0
        right_speed = 1.0
    
    left_motor.setVelocity(left_speed)
    right_motor.setVelocity(right_speed)
