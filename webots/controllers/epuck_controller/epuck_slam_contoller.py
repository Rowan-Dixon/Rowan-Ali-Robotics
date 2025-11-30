from epuck_controller import Robot
import math

from slam import Slam


def raw_to_distance(raw, max_range=0.06):
    # Rough conversion from IR 'raw' -> distance (m)
    raw = max(0.0, min(raw, 4000.0))
    norm = 1.0 - (raw / 4000.0)
    return norm * max_range


if __name__ == "__main__":
    robot = Robot()
    timestep = int(robot.getBasicTimeStep())
    max_speed = 6.28

    # Motors & encoders
    left_motor = robot.getDevice("left wheel motor")
    right_motor = robot.getDevice("right wheel motor")
    left_motor.setPosition(float('inf'))
    right_motor.setPosition(float('inf'))
    left_motor.setVelocity(0.0)
    right_motor.setVelocity(0.0)

    left_ps = robot.getDevice("left wheel sensor")
    right_ps = robot.getDevice("right wheel sensor")
    left_ps.enable(timestep)
    right_ps.enable(timestep)

    wheel_radius = 0.0205
    axle_length = 0.052    #(you can change to 0.058 if that matches your model)

    # Proximity sensors
    ps_names = [f"ps{i}" for i in range(8)]
    proximity_sensors = []
    for name in ps_names:
        s = robot.getDevice(name)
        s.enable(timestep)
        proximity_sensors.append(s)

    sensor_angles = [
        0.30,   # ps0
        0.85,   # ps1
        2.20,   # ps2
       -2.20,   # ps3
       -0.85,   # ps4
       -0.30,   # ps5
        0.0,    # ps6
        0.0     # ps7
    ]
    max_range = 0.06 

    slam = Slam(
        map_width_m=2.0,
        map_height_m=2.0,
        resolution=0.02,
        sensor_angles=sensor_angles,
        max_range=max_range
    )

    # Odometry state
    last_ps_values = [0.0, 0.0]
    robot_pose = [0.0, 0.0, 0.0]

    robot.step(timestep)
    last_ps_values[0] = left_ps.getValue()
    last_ps_values[1] = right_ps.getValue()

    forward_speed = 0.5 * max_speed
    turn_speed = 0.3 * max_speed

    while robot.step(timestep) != -1:
        # --- ODOM ---
        ps_vals = [left_ps.getValue(), right_ps.getValue()]
        dl_rad = ps_vals[0] - last_ps_values[0]
        dr_rad = ps_vals[1] - last_ps_values[1]
        last_ps_values = ps_vals.copy()

        dl = wheel_radius * dl_rad
        dr = wheel_radius * dr_rad
        ds = (dl + dr) / 2.0
        dtheta = (dr - dl) / axle_length

        theta_mid = robot_pose[2] + dtheta / 2.0
        robot_pose[0] += ds * math.cos(theta_mid)
        robot_pose[1] += ds * math.sin(theta_mid)
        robot_pose[2] = math.atan2(
            math.sin(robot_pose[2] + dtheta),
            math.cos(robot_pose[2] + dtheta)
        )

        odom_pose = tuple(robot_pose)

        # --- SENSORS -> ranges ---
        ranges = []
        for s in proximity_sensors:
            raw = s.getValue()
            r = raw_to_distance(raw, max_range)
            ranges.append(r)

        slam_pose = slam.update(odom_pose, ranges)

        front_close = (ranges[0] < 0.03) or (ranges[6] < 0.03) or (ranges[7] < 0.03)

        if front_close:
            left_speed = -turn_speed
            right_speed = turn_speed
        else:
            left_speed = forward_speed
            right_speed = forward_speed

        left_motor.setVelocity(left_speed)
        right_motor.setVelocity(right_speed)

        x, y, theta = slam_pose
        print(f"SLAM pose: x={x:.3f} y={y:.3f} theta={math.degrees(theta):.1f}°")
        # probs = slam.get_map()  # <- ready for planner later
