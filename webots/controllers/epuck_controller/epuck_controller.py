from controller import Robot
import math

if __name__ == '__main__':
    # create the Robot instance.
    robot = Robot()
    
    # get the time step of the current world.
    timestep = 64
    max_speed = 6.28
    
    # Get keyboard
    keyboard = robot.getKeyboard()
    keyboard.enable(timestep)
    
    left_motor = robot.getDevice("left wheel motor")
    left_motor.setPosition(float('inf'))
    left_motor.setVelocity(0.0)
    
    right_motor = robot.getDevice("right wheel motor")
    right_motor.setPosition(float('inf'))
    right_motor.setVelocity(0.0)
    
    left_ps = robot.getDevice("left wheel sensor")
    left_ps.enable(timestep)
    
    right_ps = robot.getDevice("right wheel sensor")
    right_ps.enable(timestep)
    
    range_finder = robot.getDevice("range finder")
    range_finder.enable(timestep)
    
    # Movement parameters
    forward_speed = 2.0
    turn_speed = 1.5
    target_distance = 0.25  # meters
    target_angle = math.pi / 2  # 90 degrees in radians
    
    # robot constants
    wheel_radius = 0.0205  # meters
    axel_length = 0.058   # meters
    encoder_resolution = 6.28  # radians per encoder unit (one full rotation)
    
    # robot values
    last_ps_values = [0, 0]
    robot_pose = [0, 0, 0]  # [x, y, theta]
    
    # Initialize - read first values
    robot.step(timestep)
    last_ps_values[0] = left_ps.getValue()
    last_ps_values[1] = right_ps.getValue()
    
    # Main loop:
    while robot.step(timestep) != -1:
        ps_values = [left_ps.getValue(), right_ps.getValue()]
        
        # Calculate change in encoder values
        dl_ticks = ps_values[0] - last_ps_values[0]
        dr_ticks = ps_values[1] - last_ps_values[1]
        
        # Convert encoder ticks to radians, then to linear distance
        dl_rad = dl_ticks  # encoder values are already in radians
        dr_rad = dr_ticks
        
        dl = wheel_radius * dl_rad  # left wheel distance in meters
        dr = wheel_radius * dr_rad  # right wheel distance in meters
        
        # Calculate robot displacement
        ds = (dl + dr) / 2.0  # center displacement
        dtheta = (dr - dl) / axel_length  # change in orientation
        
        # Update robot pose using differential drive kinematics
        robot_pose[0] += ds * math.cos(robot_pose[2] + dtheta / 2.0)
        robot_pose[1] += ds * math.sin(robot_pose[2] + dtheta / 2.0)
        robot_pose[2] += dtheta
        
        # Normalize theta to [-pi, pi]
        robot_pose[2] = math.atan2(math.sin(robot_pose[2]), math.cos(robot_pose[2]))
        
        # Update last values
        last_ps_values = ps_values.copy()
        
        # Print odometry info
        print(f"Encoder values: L={ps_values[0]:.3f}, R={ps_values[1]:.3f}")
        print(f"dl={dl:.6f}m, dr={dr:.6f}m, ds={ds:.6f}m, dtheta={dtheta:.6f}rad")
        print(f"Robot pose: x={robot_pose[0]:.6f}m, y={robot_pose[1]:.6f}m, theta={robot_pose[2]:.6f}rad ({math.degrees(robot_pose[2]):.2f}°)")
        print("=" * 60)
        
        # Keyboard control
        left_speed = 0
        right_speed = 0
        
        key = keyboard.getKey()
        if key == ord('W'):  # Forward
            left_speed = 2
            right_speed = 2
        elif key == ord('S'):  # Backward
            left_speed = -2
            right_speed = -2
        elif key == ord('A'):  # Turn left
            left_speed = -1
            right_speed = 1
        elif key == ord('D'):  # Turn right
            left_speed = 1
            right_speed = -1
        elif key == ord('Q'):  # Forward left
            left_speed = 1
            right_speed = 2
        elif key == ord('E'):  # Forward right
            left_speed = 2
            right_speed = 1
        
        left_motor.setVelocity(left_speed)
        right_motor.setVelocity(right_speed)