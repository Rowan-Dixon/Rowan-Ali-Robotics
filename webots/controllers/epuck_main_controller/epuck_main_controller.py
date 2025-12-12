from controller import Robot
from odom import Odometry
from rbpf_slam import RBPF_SLAM
from goals_map import CURRENT_LOAD, MAX_CARRY
from nav_controller import compute_wheel_speeds
from print_map import print_map
import math

# ----------------- WEBOTS SETUP -----------------
robot = Robot()
timestep = int(robot.getBasicTimeStep())

# ----------------- ENCODERS -----------------
left_ps = robot.getDevice('left wheel sensor')
right_ps = robot.getDevice('right wheel sensor')
left_ps.enable(timestep)
right_ps.enable(timestep)

# ----------------- LIDAR (LDS-01) -----------------
lidar = robot.getDevice('LDS-01')
lidar.enable(timestep)
try:
    lidar.enablePointCloud()
except Exception:
    pass

def lidar_sample_ranges(lidar, sensor_angles, max_range):
    """Get a 360° range array, reordered so index 0 is 'front' of robot."""
    scan = lidar.getRangeImage()
    ranges = scan[180:] + scan[:180]
    ranges = [max_range if not (0 < r <= max_range) else r for r in ranges]
    return ranges

# ----------------- ODOM -----------------
odom = Odometry(wheel_radius=0.033, axle_length=0.178)

sensor_angles = [math.radians(-angle) for angle in range(360)]
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
    num_particles=40
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

robot.step(timestep)
odom.reset(left_ps.getValue(), right_ps.getValue())

last_print_time = -1

# ----------------- MAIN LOOP (AUTONOMOUS) -----------------
while robot.step(timestep) != -1:
    current_time = robot.getTime()

    # ----- ODOM -----
    pose_odom = odom.update(left_ps.getValue(), right_ps.getValue())

    # ----- LIDAR RANGES -----
    ranges = lidar_sample_ranges(lidar, sensor_angles, max_range)

    # ----- INITIAL SPIN PHASE FOR MAPPING -----
    if current_time < 5.0:
        left_motor.setVelocity(-2.5)
        right_motor.setVelocity(2.5)

        # still update SLAM so the map builds while spinning
        pose_est = rbpf_slam.update(pose_odom, ranges)

        # optionally draw map during spin
        t_int = int(current_time)
        if t_int != last_print_time:
            last_print_time = t_int
            print_map(pose_est, rbpf_slam, map_display)

        continue

    # ----- RBPF SLAM (mapping + localisation) -----
    pose_est = rbpf_slam.update(pose_odom, ranges)

    # ----- HIGH-LEVEL NAVIGATION (tasks + exploration + control) -----
    left_speed, right_speed = compute_wheel_speeds(pose_est, rbpf_slam, ranges)

    # clamp motor speeds
    max_motor_speed = 6.28
    left_speed = max(-max_motor_speed, min(max_motor_speed, left_speed))
    right_speed = max(-max_motor_speed, min(max_motor_speed, right_speed))

    # ----- SEND COMMANDS TO MOTORS -----
    left_motor.setVelocity(left_speed)
    right_motor.setVelocity(right_speed)

    # ----- PRINT POSE + CARRY INFO & DRAW MAP (THROTTLED) -----
    t_int = int(current_time)
    if t_int != last_print_time:
        last_print_time = t_int
        print(
            f"EST Pose: x={pose_est[0]:.3f}, y={pose_est[1]:.3f}, th={pose_est[2]:.3f}, "
            f"LOAD={CURRENT_LOAD}/{MAX_CARRY}"
        )
        print_map(pose_est, rbpf_slam, map_display)
