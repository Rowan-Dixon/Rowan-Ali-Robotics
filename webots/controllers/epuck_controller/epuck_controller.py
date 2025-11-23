"""epuck_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
import math

if __name__ == '__main__':
    # create the Robot instance.
    robot = Robot()
    
    # get the time step of the current world.
    timestep = 64 #int(robot.getBasicTimeStep())
    max_speed = 6.28
    
    left_motor = robot.getDevice("left wheel motor")
    left_motor.setPosition(float('inf'))
    left_motor.setVelocity(0.0)
    
    right_motor = robot.getDevice("right wheel motor")
    right_motor.setPosition(float('inf'))
    right_motor.setVelocity(0.0)
    
    left_ps  = robot.getDevice("left wheel sensor")
    left_ps.enable(timestep)
    
    right_ps = robot.getDevice("right wheel sensor")
    right_ps.enable(timestep)
    
    
    #for testing
    left_speed = 2
    right_speed = 2
    
    #robot constants
    wheel_radius = 0.0205
    # wheel_circum = 2 * 3.14 * wheel_radius
    axel_length = 0.052
    # encoder_unit  =wheel_circum/6.28
    
    #robot values
    # ps_values = [left_ps.getValue(),right_ps.getValue()]
    # last_ps_values = [left_ps.getValue(),right_ps.getValue()]
    ps_values = [0, 0]
    last_ps_values = [0, 0]
   
    # dist_values=[0,0]
    robot_pose = [0,0,0]
    
    # Main loop:
    # - perform simulation steps until Webots is stopping the controller
    while robot.step(timestep) != -1:
        ps_values[0] = left_ps.getValue()
        ps_values[1] = right_ps.getValue()
        print(ps_values)
                
        # print("===================================")
        # print("position sensor values: {} {}".format(ps_values[0], ps_values[1]))
        
        dl = ps_values[0] - last_ps_values[0]
        dr = ps_values[1] - last_ps_values[1]
        last_ps_values = ps_values.copy()
        print(last_ps_values)
        
        ds = wheel_radius * (dl + dr) / 2
        dtheta = wheel_radius * (dr-dl) / axel_length
        
        robot_pose[0] += ds * math.cos(robot_pose[2] + dtheta / 2.0)
        robot_pose[1] += ds * math.sin(robot_pose[2] + dtheta / 2.0)
        robot_pose[2] += dtheta
        
        print("\ndl, dr:", dl, dr)
        print("robot_pose: {}".format(robot_pose))
    
        left_motor.setVelocity(left_speed)
        right_motor.setVelocity(right_speed)
        
        # if robot_pose[0] < 0.025:
            # left_motor.setVelocity(2)
            # right_motor.setVelocity(2)
        # elif robot_pose[2] < math.pi:
            # left_motor.setVelocity(0)
            # right_motor.setVelocity(2) 
        # elif robot_pose[1] < 0.025:
            # left_motor.setVelocity(2)
            # right_motor.setVelocity(2)  
    
    # Enter here exit cleanup code.
    