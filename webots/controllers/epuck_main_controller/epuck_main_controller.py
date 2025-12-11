from controller import Robot
from odom import Odometry
from rbpf_slam import RBPF_SLAM
from collections import deque
from go_to_goal import PathPlanner
import math

# ----------------- TASK GOALS -----------------
GOALS = [
     # --- PICKUP GOALS ---
    {"name": "pickup_1", "pos": ( 0.30,  0.10), "kind": "pickup",  "pair": 1,
     "known": False, "done": False, "reachable": None},

    {"name": "pickup_2", "pos": ( 0.50, -0.20), "kind": "pickup",  "pair": 2,
     "known": False, "done": False, "reachable": None},

    {"name": "pickup_3", "pos": (-0.40,  0.25), "kind": "pickup",  "pair": 3,
     "known": False, "done": False, "reachable": None},

    {"name": "pickup_4", "pos": (-0.10, -0.30), "kind": "pickup",  "pair": 4,
     "known": False, "done": False, "reachable": None},

    {"name": "pickup_5", "pos": ( 0.00,  0.40), "kind": "pickup",  "pair": 5,
     "known": False, "done": False, "reachable": None},

    # --- DROPOFF GOALS ---
    {"name": "dropoff_1", "pos": (-0.20,  0.15), "kind": "dropoff", "pair": 1,
     "known": False, "done": False, "reachable": None},

    {"name": "dropoff_2", "pos": ( 0.45,  0.05), "kind": "dropoff", "pair": 2,
     "known": False, "done": False, "reachable": None},

    {"name": "dropoff_3", "pos": ( 0.15, -0.35), "kind": "dropoff", "pair": 3,
     "known": False, "done": False, "reachable": None},

    {"name": "dropoff_4", "pos": (-0.50, -0.10), "kind": "dropoff", "pair": 4,
     "known": False, "done": False, "reachable": None},

    {"name": "dropoff_5", "pos": ( 0.00, -0.45), "kind": "dropoff", "pair": 5,
     "known": False, "done": False, "reachable": None},
]

KNOWN_GOALS = []

DETECT_RADIUS = 0.10
REACHED_RADIUS = 0.05

MAX_CARRY = 3
CURRENT_LOAD = 0

CARRIED_ITEMS = []

desired_target_pos = None
current_planner = None
current_target_pos = None

explore_target_cached = None
explore_target_age = 0

# ----------------- GOAL / MAP HELPERS -----------------


def goal_is_in_wall(grid, goal_pos, occ_threshold=0.6):
    "Returns True if the goal cell is considered occupied or outside the map."
    mx_my = grid.world_to_map(goal_pos[0], goal_pos[1])
    if mx_my is None:
        # outside map bounds -> treat as invalid
        return True

    mx, my = mx_my
    prob_map = grid.get_probability_map()
    p = prob_map[my][mx]
    return p > occ_threshold


def is_reachable(grid, start_pos, goal_pos, occ_threshold=0.6):
    """
    Returns True if there is a free/unknown path from start_pos to goal_pos
    on the current occupancy grid (using 4-connected BFS).
    """
    start = grid.world_to_map(start_pos[0], start_pos[1])
    goal = grid.world_to_map(goal_pos[0], goal_pos[1])

    if start is None or goal is None:
        return False

    sx, sy = start
    gx, gy = goal

    prob = grid.get_probability_map()
    H = grid.height_cells
    W = grid.width_cells

    visited = set()
    q = deque()
    q.append((sx, sy))
    visited.add((sx, sy))

    while q:
        x, y = q.popleft()
        if (x, y) == (gx, gy):
            return True

        # 4-connected neighbours
        for nx, ny in ((x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1)):
            if 0 <= nx < W and 0 <= ny < H:
                if (nx, ny) in visited:
                    continue
                # traversable if NOT highly occupied
                if prob[ny][nx] < occ_threshold:
                    visited.add((nx, ny))
                    q.append((nx, ny))

    return False


def update_goals(pose, slam, goals, known_list):
    """
    Update goal discovery and completion based on current SLAM pose.

    - Discover goals within DETECT_RADIUS (reject if in wall, mark reachable).
    - When within REACHED_RADIUS of a known & reachable goal:
        * PICKUP: only if we have capacity; record its pair in CARRIED_ITEMS.
        * DROPOFF: only if we are carrying a matching pair; remove it.
    """
    global CURRENT_LOAD, CARRIED_ITEMS

    x, y, theta = pose
    grid = slam.grid

    for g in goals:
        if g["done"]:
            continue

        gx, gy = g["pos"]
        d = math.hypot(gx - x, gy - y)

        # ---------- 1) DISCOVERY ----------
        if (not g["known"]) and d < DETECT_RADIUS:
            if goal_is_in_wall(grid, g["pos"]):
                print(f"[GOAL INVALID] {g['name']} is in an obstacle or outside map, ignoring.")
                g["done"] = True
                g["reachable"] = False
                continue

            g["known"] = True
            known_list.append(g)
            print(f"[GOAL DISCOVERED] {g['name']} ({g['kind']}) at {g['pos']} (d = {d:.2f} m)")

            reachable = is_reachable(grid, (x, y), g["pos"])
            g["reachable"] = reachable
            if reachable:
                print(f"[GOAL STATUS] {g['name']} is currently REACHABLE.")
            else:
                print(f"[GOAL STATUS] {g['name']} is currently BLOCKED / UNREACHABLE.")

        # ---------- 2) REACHING GOAL ----------
        if g["known"] and (g.get("reachable") is not False) and d < REACHED_RADIUS:
            pair_id = g.get("pair")

            if g["kind"] == "pickup":
                # PICKUP with capacity + record pair
                if CURRENT_LOAD < MAX_CARRY:
                    CURRENT_LOAD += 1
                    g["done"] = True
                    if pair_id is not None:
                        CARRIED_ITEMS.append(pair_id)
                    print(
                        f"[PICKUP] Reached {g['name']} (pair {pair_id}) at d={d:.3f} m. "
                        f"Picked up item. LOAD = {CURRENT_LOAD}/{MAX_CARRY}, "
                        f"CARRIED_ITEMS = {CARRIED_ITEMS}"
                    )
                else:
                    print(
                        f"[PICKUP BLOCKED] {g['name']} reached at d={d:.3f} m, "
                        f"but capacity is FULL ({CURRENT_LOAD}/{MAX_CARRY})."
                    )
                    # do NOT mark as done -> can come back later

            elif g["kind"] == "dropoff":
                # DROPOFF only if we have a matching pair
                if CURRENT_LOAD > 0 and pair_id is not None and pair_id in CARRIED_ITEMS:
                    CURRENT_LOAD -= 1
                    CARRIED_ITEMS.remove(pair_id)
                    g["done"] = True
                    print(
                        f"[DROPOFF] Reached {g['name']} (pair {pair_id}) at d={d:.3f} m. "
                        f"Dropped 1 item. LOAD = {CURRENT_LOAD}/{MAX_CARRY}, "
                        f"CARRIED_ITEMS = {CARRIED_ITEMS}"
                    )
                elif CURRENT_LOAD == 0:
                    print(
                        f"[DROPOFF EMPTY] Reached {g['name']} at d={d:.3f} m, "
                        "but robot is not carrying anything."
                    )
                else:
                    print(
                        f"[DROPOFF WRONG PAIR] Reached {g['name']} (pair {pair_id}) at d={d:.3f} m, "
                        f"but not carrying matching item. CARRIED_ITEMS = {CARRIED_ITEMS}"
                    )

# ----------------- GOAL SELECTION & MOTION CONTROL -----------------


def select_next_goal(pose, goals):
    """
    Choose the next goal based on:
    - If we carry nothing  -> go to nearest unfinished pickup.
    - If we carry something -> go to nearest unfinished dropoff whose pair is in CARRIED_ITEMS.
      If none, fallback to any unfinished dropoff.
    - If no goals left -> return None.
    """
    global CURRENT_LOAD, CARRIED_ITEMS

    x, y, _ = pose

    unfinished = [g for g in goals if not g["done"]]
    if not unfinished:
        return None

    if CURRENT_LOAD == 0:
        candidates = [g for g in unfinished if g["kind"] == "pickup"]
        if not candidates:
            candidates = unfinished
    else:
        # carrying something: prefer matching dropoffs
        drop_candidates = [
            g for g in unfinished
            if g["kind"] == "dropoff" and g.get("pair") in CARRIED_ITEMS
        ]
        if not drop_candidates:
            # fallback: any unfinished dropoff
            drop_candidates = [g for g in unfinished if g["kind"] == "dropoff"]
        candidates = drop_candidates or unfinished

    reachable_candidates = [g for g in candidates if g.get("reachable") is not False]
    if reachable_candidates:
        candidates = reachable_candidates

    def dist_to_goal(g):
        gx, gy = g["pos"]
        return math.hypot(gx - x, gy - y)

    best = min(candidates, key=dist_to_goal)
    return best

def map_cell_to_world(grid, mx, my):
    """
    Convert map indices (mx, my) to world coordinates (x, y) at cell centre.
    """
    res = grid.resolution
    width_m = grid.width_m
    height_m = grid.height_m

    x = (mx + 0.5) * res - width_m / 2.0
    y = (my + 0.5) * res - height_m / 2.0
    return x, y

def cell_is_unknown(prob_map, mx, my, p_min=0.3, p_max=0.5):
    p = prob_map[my][mx]
    return p_min < p < p_max

def has_unknown_region_around(prob_map, mx, my, p_min=0.3, p_max=0.5, min_unknown_neighbors=3):
    """
    Returns True if there are at least `min_unknown_neighbors` unknown-ish cells
    in the 3x3 neighbourhood around (mx, my). This filters out tiny 1x1 noise blobs.
    """
    H = len(prob_map)
    W = len(prob_map[0]) if H > 0 else 0

    count = 0
    for dy in (-1, 0, 1):
        for dx in (-1, 0, 1):
            nx = mx + dx
            ny = my + dy
            if 0 <= nx < W and 0 <= ny < H:
                if cell_is_unknown(prob_map, nx, ny, p_min, p_max):
                    count += 1

    return count >= min_unknown_neighbors

def find_exploration_target(pose, grid, p_min=0.3, p_max=0.5, min_unknown_neighbors=3):
    """
    Scan the occupancy grid and find the nearest cell to the robot
    whose probability is in (p_min, p_max) and which is part of a
    non-trivial unknown region (not just 1x1 noise).

    Returns (x, y) in world coordinates, or None if no such cell.
    """
    x_r, y_r, _ = pose
    prob_map = grid.get_probability_map()
    H = grid.height_cells
    W = grid.width_cells

    best_dist = None
    best_world = None

    for my in range(H):
        for mx in range(W):
            if not cell_is_unknown(prob_map, mx, my, p_min, p_max):
                continue

            # Ignore tiny single-cell unknowns
            if not has_unknown_region_around(prob_map, mx, my, p_min, p_max, min_unknown_neighbors):
                continue

            wx, wy = map_cell_to_world(grid, mx, my)
            d = math.hypot(wx - x_r, wy - y_r)

            if (best_dist is None) or (d < best_dist):
                best_dist = d
                best_world = (wx, wy)

    return best_world 

def obstacle_avoidance_override(ranges, left_speed, right_speed,
                                front_thresh=0.25, side_thresh=0.20,
                                turn_speed=3.0):
    """
    Very simple reactive obstacle avoidance using LiDAR ranges.
    - If something is too close in front -> turn on the spot.
    - If something is close on one side -> bias turn away from it.
    Otherwise, keep the PathPlanner speeds.
    """

    n = len(ranges)
    if n == 0:
        return left_speed, right_speed

    front_indices = list(range(350, 360)) + list(range(0, 10))
    left_indices = list(range(300, 350))
    right_indices = list(range(10, 60))

    def min_in_sector(idxs):
        vals = [ranges[i] for i in idxs if 0 < ranges[i] < float('inf')]
        return min(vals) if vals else float('inf')

    d_front = min_in_sector(front_indices)
    d_left = min_in_sector(left_indices)
    d_right = min_in_sector(right_indices)

    if d_front < front_thresh:
        if d_left > d_right:
            return -turn_speed, turn_speed
        else:
            return turn_speed, -turn_speed

    if d_left < side_thresh and d_right >= side_thresh:
        return left_speed + 0.0, right_speed + 1.0
    if d_right < side_thresh and d_left >= side_thresh:
        return left_speed + 1.0, right_speed + 0.0

    return left_speed, right_speed

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

# LiDAR beam angles (0..359 degrees -> radians)
sensor_angles = [math.radians(angle) for angle in range(360)]

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

robot.step(timestep)  # initial step to get sensor readings
odom.reset(left_ps.getValue(), right_ps.getValue())

last_print_time = -1  # for throttling printing

# ----------------- MAIN LOOP (AUTONOMOUS) -----------------
while robot.step(timestep) != -1:
    # ----- ODOM -----
    pose_odom = odom.update(left_ps.getValue(), right_ps.getValue())

    # ----- LIDAR RANGES -----
    ranges = lidar_sample_ranges(lidar, sensor_angles, max_range)

    # ----- RBPF SLAM (mapping + localisation) -----
    pose_est = rbpf_slam.update(pose_odom, ranges)

        # ----- TASK GOALS (discovery + completion + capacity + pairing) -----
    update_goals(pose_est, rbpf_slam, GOALS, KNOWN_GOALS)

    # ----- HIGH-LEVEL TARGET SELECTION (TASKS + EXPLORATION) -----
    # Decide where we *want* to go: a pickup/dropoff if available,
    # otherwise an exploration frontier cell in the unknown map.

    task_goal = select_next_goal(pose_est, GOALS)

    if task_goal is not None:
        desired_target_pos = task_goal["pos"]
        explore_target_cached = None
        explore_target_age = 0
    else:
        explore_target_age += 1

        if explore_target_cached is None or explore_target_age > 50:
            explore_target_cached = find_exploration_target(pose_est, rbpf_slam.grid)
            explore_target_age = 0
            if explore_target_cached is not None:
                print(f"[TARGET] New exploration target at {explore_target_cached}")
            else:
                print("[TARGET] No exploration frontier found (map mostly known).")

        desired_target_pos = explore_target_cached

    # ----- PATH PLANNER -> WHEEL SPEEDS -----
    if desired_target_pos is not None:
        if current_planner is None or current_target_pos is None or \
           math.hypot(desired_target_pos[0] - current_target_pos[0],
                      desired_target_pos[1] - current_target_pos[1]) > 0.05:
            current_planner = PathPlanner(pose_est, desired_target_pos)
            current_target_pos = desired_target_pos
            print(f"[PLANNER] New target set at {current_target_pos}")

        left_speed, right_speed, dist = current_planner.go_to_point(pose_est)
    else:
        current_planner = None
        current_target_pos = None
        left_speed = 1.0
        right_speed = 1.0


    # ----- LOCAL OBSTACLE AVOIDANCE OVERRIDE -----
    left_speed, right_speed = obstacle_avoidance_override(
        ranges, left_speed, right_speed
    )

    # (optional) clamp to something reasonable for your robot
    max_motor_speed = 6.28  # e.g. TurtleBot max
    left_speed = max(-max_motor_speed, min(max_motor_speed, left_speed))
    right_speed = max(-max_motor_speed, min(max_motor_speed, right_speed))

    # ----- SEND COMMANDS TO MOTORS -----
    left_motor.setVelocity(left_speed)
    right_motor.setVelocity(right_speed)


    # ----- PRINT POSE + CARRY INFO & DRAW MAP (THROTTLED) -----
    t_int = int(robot.getTime())
    if t_int != last_print_time:
        last_print_time = t_int
        print(
            f"EST Pose: x={pose_est[0]:.3f}, y={pose_est[1]:.3f}, th={pose_est[2]:.3f}, "
            f"LOAD={CURRENT_LOAD}/{MAX_CARRY}"
        )

        # ----- DRAW MAP ON DISPLAY ----- 
        prob_map = rbpf_slam.get_probability_map()
        height_cells = len(prob_map)
        width_cells = len(prob_map[0]) if height_cells > 0 else 0

        if height_cells > 0 and width_cells > 0:
            disp_w = map_display.getWidth()
            disp_h = map_display.getHeight()
            cell_scale_x = disp_w / width_cells
            cell_scale_y = disp_h / height_cells
            cell_scale = min(cell_scale_x, cell_scale_y)

            map_display.setColor(0xFFFFFF)
            map_display.fillRectangle(0, 0, disp_w, disp_h)

            for j in range(height_cells):
                for i in range(width_cells):
                #   ^^^ careful to keep this indented inside the throttled block
                    p = prob_map[j][i]

                    if p < 0.3:
                        color = 0xFFFFFF  # free -> white
                    elif p > 0.7:
                        color = 0x000000  # occupied -> black
                    else:
                        color = 0x808080  # unknown-ish -> grey

                    map_display.setColor(color)

                    x_screen = int(i * cell_scale)
                    y_screen = int((height_cells - 1 - j) * cell_scale)

                    map_display.fillRectangle(x_screen, y_screen,
                                              int(cell_scale) + 1, int(cell_scale) + 1)
