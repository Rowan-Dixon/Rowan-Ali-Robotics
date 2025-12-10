from controller import Robot
from odom import Odometry
from slam import Slam
from go_to_goal import go_to_point, get_next_point
import math

robot = Robot()
timestep = int(robot.getBasicTimeStep())

left_ps = robot.getDevice('left wheel sensor')
right_ps = robot.getDevice('right wheel sensor')
left_ps.enable(timestep)
right_ps.enable(timestep)

# Replace proximity sensor setup with LIDAR (LDS-01) setup:
lidar = robot.getDevice('LDS-01')  # use the exact device name from the TurtleBot proto if different
lidar.enable(timestep)
# optional: if point cloud is needed
try:
    lidar.enablePointCloud()
except Exception:
    pass

# helper to sample lidar ranges at desired sensor_angles (angles are relative to robot heading)
def lidar_sample_ranges(lidar, sensor_angles, max_range):
    ranges = []
    scan = lidar.getRangeImage() # array len 360 of range values, starting at 180 and going clockwise
    ranges = scan[180:] + scan[:180]  # reorder to start from in front
    ranges = [max_range if not (0 < r <= max_range) else r for r in ranges]
    return ranges

# odom = Odometry(wheel_radius=0.0205, axle_length=0.058)
odom = Odometry(wheel_radius=0.033, axle_length=0.178)
sensors_angles = [math.radians(angle) for angle in range(360)] 
# use lidar max range for SLAM
max_range = lidar.getMaxRange()
slam = Slam(2.0, 2.0, 0.02, sensors_angles, max_range)

left_ps = robot.getDevice('left wheel sensor')
right_ps = robot.getDevice('right wheel sensor')
left_ps.enable(timestep)
right_ps.enable(timestep)

left_motor = robot.getDevice("left wheel motor")
left_motor.setPosition(float('inf'))
left_motor.setVelocity(0.0)

right_motor = robot.getDevice("right wheel motor")
right_motor.setPosition(float('inf'))
right_motor.setVelocity(0.0)

gyro = robot.getDevice("gyro")
gyro.enable(timestep)   

keyboard = robot.getKeyboard()
keyboard.enable(timestep)


robot.step(timestep)  # initial step to get sensor readings
odom.reset(left_ps.getValue(), right_ps.getValue())


points = [(0.5,0.5), (1.0,1.0), (1.5,0.5), (2.0,0.0), (2.0,-0.5)] #[(0.0, 0.0), (-0.5, 1.0)]  
next_point = None
left_speed, right_speed, dist = 0.0, 0.0, math.inf
prev_dist_to_goal = math.inf

while robot.step(timestep) != -1:
    
    pose_odom = odom.update(left_ps.getValue(), right_ps.getValue())
    ranges = lidar_sample_ranges(lidar, sensors_angles, max_range)
    pose_slam = slam.update(pose_odom, ranges) #we might want to run localisation only once every x timesteps for performance and accuracy
    if not next_point:
        # next_point = get_next_point(pose_slam, final_goal)
        next_point = points.pop(0)

    if points:
        left_speed, right_speed, dist = go_to_point(pose_slam, next_point, ka=10.0, kd=10.0)
        if dist < 0.2 and points:
            # next_point = get_next_point(pose_slam, final_goal)
            next_point = points.pop(0)

    if not points:
        left_speed, right_speed, dist = go_to_point(pose_slam, next_point, ka=10.0, kd=10.0)
        if dist < 0.05:
            left_speed = 0.0
            right_speed = 0.0
            print("final pose: ", pose_slam)

    left_motor.setVelocity(left_speed)
    right_motor.setVelocity(right_speed)


    # # Keyboard control
    # left_speed = 0
    # right_speed = 0
    
    # key = keyboard.getKey()
    # if key == ord('W'):  # Forward
    #     left_speed = 2
    #     right_speed = 2
    # elif key == ord('S'):  # Backward
    #     left_speed = -2
    #     right_speed = -2
    # elif key == ord('A'):  # Turn left
    #     left_speed = -1
    #     right_speed = 1
    # elif key == ord('D'):  # Turn right
    #     left_speed = 1
    #     right_speed = -1
    # elif key == ord('Q'):  # Forward left
    #     left_speed = 1
    #     right_speed = 2
    # elif key == ord('E'):  # Forward right
    #     left_speed = 2
    #     right_speed = 1

    # slam.debug_draw()  # optional: visualize SLAM map and pose
