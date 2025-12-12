from controller import Robot
from odom import Odometry
from rbpf_slam import RBPF_SLAM   
from print_map import print_map
from go_to_goal import PathPlanner
from goals import Goals
from obstacle_avoidance import obstacle_avoidance_override
import math

robot = Robot()
timestep = int(robot.getBasicTimeStep())

# ----------------- ENCODERS -----------------
left_ps = robot.getDevice('left wheel sensor')
right_ps = robot.getDevice('right wheel sensor')
left_ps.enable(timestep)
right_ps.enable(timestep)

# ----------------- LIDAR (LDS-01) -----------------
lidar = robot.getDevice('LDS-01')  # make sure name matches PROTO
lidar.enable(timestep)
try:
    lidar.enablePointCloud()
except Exception:
    pass

def lidar_sample_ranges(lidar, sensor_angles, max_range):
    """Get a 360° range array, reordered so index 0 is 'front' of robot."""
    scan = lidar.getRangeImage()        # len 360, starting at 180 and going clockwise
    ranges = scan[180:] + scan[:180]    # reorder so front is index 0
    # clamp invalid readings to max_range
    ranges = [max_range if not (0 < r <= max_range) else r for r in ranges]
    return ranges

# ----------------- ODOM -----------------
odom = Odometry(wheel_radius=0.033, axle_length=0.178)

# LiDAR beam angles (0..359 degrees -> radians)
sensor_angles = [math.radians(-angle) for angle in range(360)] #double check the added negative to the angle is correct here

# max lidar range
max_range = lidar.getMaxRange()

# ----------------- RBPF SLAM SETUP -----------------
map_width_m = 4.0
map_height_m = 4.0
resolution = 0.02

rbpf_slam = RBPF_SLAM(
    map_width_m,
    map_height_m,
    resolution,
    sensor_angles,
    max_range,
    num_particles=40  # reduce if sim is slow (20, etc.)
)

# ----------------- MOTORS -----------------
left_motor = robot.getDevice("left wheel motor")
left_motor.setPosition(float('inf'))
left_motor.setVelocity(0.0)

right_motor = robot.getDevice("right wheel motor")
right_motor.setPosition(float('inf'))
right_motor.setVelocity(0.0)

# ----------------- DISPLAY (MAP VIEW) -----------------
map_display = robot.getDevice("map_display")

# ----------------- GYRO (optional) -----------------
gyro = robot.getDevice("gyro")
gyro.enable(timestep)

# ----------------- KEYBOARD -----------------
keyboard = robot.getKeyboard()
keyboard.enable(timestep)

robot.step(timestep)  # initial step to get sensor readings
odom.reset(left_ps.getValue(), right_ps.getValue())
print(odom.x, odom.y, odom.theta)
path = None
goals_manager = Goals()
last_print_time = -1  # for throttling printing

# goals = [(-0.75, -1.25), (-1.0, -0.75), (-1.0, 0.0), (0.25, 1.25)]

while robot.step(timestep) != -1:
    
    pose_odom = odom.update(left_ps.getValue(), right_ps.getValue())
    ranges = lidar_sample_ranges(lidar, sensor_angles, max_range)
    pose_est = rbpf_slam.update(pose_odom, ranges)
    goals_manager.discover_goals(pose_est)

    # scans the surroundings for 3 seconds (rays initially have unknown areas between)
    if robot.getTime() < 5.0: 
        left_motor.setVelocity(-2.5)
        right_motor.setVelocity(2.5)
        continue

    #if goal reached, set goal to none
    if path.goal_dist < 0.08:
        path.goal = None
        if path.goal == goals_manager.current_goal:
            goals_manager.goal_reached()

    #if goal none, find new one
    if path.goal is None:
        task_goal = goals_manager.choose_goal(pose_est, rbpf_slam)
        #if task_goal, go there, else sets to none and explores
        path = PathPlanner(pose=pose_est, slam_map=rbpf_slam, goal=(task_goal["pos"] if task_goal else None))

    # ----- PRINT POSE + MAP ONCE PER SECOND -----
    t_int = int(robot.getTime())
    if t_int != last_print_time:
        last_print_time = t_int
        print_map(pose_est, rbpf_slam, map_display) 

    if path.goal:
        left_speed, right_speed = path.go_to_point(pose_est, max_speed=5.0, ka=4.0)
        #highest priority: obstacle avoidance
        left_speed, right_speed = obstacle_avoidance_override(ranges, left_speed, right_speed)
    else:
        print("EXPLORATION AND GOALS COMPLETE")
        left_speed, right_speed = 0.0, 0.0
        
    left_motor.setVelocity(left_speed)
    right_motor.setVelocity(right_speed)
