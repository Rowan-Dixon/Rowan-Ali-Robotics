from controller import Robot
from odom import Odometry
from rbpf_slam import RBPF_SLAM   
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

last_print_time = -1  # for throttling printing

while robot.step(timestep) != -1:
    # ----- ODOM -----
    pose_odom = odom.update(left_ps.getValue(), right_ps.getValue())

    # ----- LIDAR RANGES -----
    ranges = lidar_sample_ranges(lidar, sensor_angles, max_range)

    # ----- RBPF SLAM (mapping + localisation) -----
    pose_est = rbpf_slam.update(pose_odom, ranges)

        # ----- PRINT POSE + MAP ONCE PER SECOND -----
    t_int = int(robot.getTime())
    if t_int != last_print_time:
        last_print_time = t_int

        # Print pose to console (optional)
        print(f"EST Pose: x={pose_est[0]:.3f}, y={pose_est[1]:.3f}, th={pose_est[2]:.3f}")

        # Get probability map from SLAM
        prob_map = rbpf_slam.get_probability_map()
        height_cells = len(prob_map)
        width_cells = len(prob_map[0]) if height_cells > 0 else 0

        # Clear the display (white background)
        disp_w = map_display.getWidth()
        disp_h = map_display.getHeight()
        cell_scale_x = disp_w / width_cells
        cell_scale_y = disp_h / height_cells
        cell_scale = min(cell_scale_x, cell_scale_y) #scales with display size
        map_display.setColor(0xFFFFFF)  # white
        map_display.fillRectangle(0, 0, disp_w, disp_h)

        # Draw each cell
        for j in range(height_cells):
            for i in range(width_cells):
                p = prob_map[j][i]

                # Color by occupancy probability
                if p < 0.3:
                    color = 0xFFFFFF  # free -> white
                elif p > 0.7:
                    color = 0x000000  # occupied -> black
                else:
                    color = 0x808080  # unknown-ish -> grey

                map_display.setColor(color)

                # Convert map cell (i, j) to display coordinates
                x_screen = i * cell_scale
                # invert y so top of map is top of display
                y_screen = (height_cells - 1 - j) * cell_scale

                map_display.fillRectangle(x_screen, y_screen,
                                          cell_scale, cell_scale)
        # could we add the actual robot overlayed ontop of the map here?


        # If you still want ASCII map in console, uncomment:
        # rbpf_slam.debug_print_map()


    # ----- KEYBOARD CONTROL -----
    left_speed = 0.0
    right_speed = 0.0
    
    key = keyboard.getKey()
    if key == ord('W'):  # Forward
        left_speed = 5.0
        right_speed = 5.0
    elif key == ord('S'):  # Backward
        left_speed = -5.0
        right_speed = -5.0
    elif key == ord('A'):  # Turn left
        left_speed = -2.5
        right_speed = 2.5
    elif key == ord('D'):  # Turn right
        left_speed = 2.5
        right_speed = -2.5
    elif key == ord('Q'):  # Forward left
        left_speed = 2.5
        right_speed = 5.0
    elif key == ord('E'):  # Forward right
        left_speed = 5.0
        right_speed = 2.5
    
    left_motor.setVelocity(left_speed)
    right_motor.setVelocity(right_speed)
