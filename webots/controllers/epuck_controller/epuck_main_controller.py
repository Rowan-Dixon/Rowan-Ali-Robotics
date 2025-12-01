from controller import Robot
from odom import Odometry
from slam import Slam

robot = Robot()
timestep = int(robot.getBasicTimeStep())

left_ps = robot.getDevice('left wheel sensor')
right_ps = robot.getDevice('right wheel sensor')
left_ps.enable(timestep)
right_ps.enable(timestep)

ps_names = [f'ps{i}' for i in range(8)]
proximity_sensors = []
for name in ps_names:
    s = robot.getDevice(name)
    s.enable(timestep)
    proximity_sensors.append(s)

def raw_ro_distance(raw, max_range = 0.06):
    raw = max(0.0, min(raw, 4000.0))
    norm =1.0 - (raw / 4000.0)
    return norm * max_range

odom = Odometry(wheel_radius=0.0205, axle_length=0.058)

sendors_angles = [...]  # fill in with actual sensor angles
max_range = 0.06
slam = Slam(2.0, 2.0, 0.02, sendors_angles, max_range)

left_motor = robot.getDevice('left wheel sensor')
right_motor = robot.getDevice('right wheel sensor')
left_motor.enable(timestep)
right_motor.enable(timestep)

robot.step(timestep)  # initial step to get sensor readings
odom.rest(left_motor.getValue(), right_motor.getValue())

while robot.step(timestep) != -1:
    # ODOM

    x, y, theta = odom.update(left_motor.getValue(), right_motor.getValue())
    
    # RANGES
    ranges = [raw_to_distance(s.getValue(), max_range) for s in proximity_sensors]

    # SLAM
    pose_slam = slam.update(pose_odom, ranges)

    # Using pose_slam + slam.get_proibability_map() for planning etc.
    print("SLAM Pose:", pose_slam)
