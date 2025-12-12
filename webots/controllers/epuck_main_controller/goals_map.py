import math
from collections import deque

# ----------------- TASK GOALS -----------------
GOALS = [
    # --- PICKUP GOALS ---
    {"name": "pickup_1", "pos": ( 1.25,  1.0), "kind": "pickup",  "pair": 1,
     "known": False, "done": False, "reachable": None},

    {"name": "pickup_2", "pos": ( 0.50, -0.75), "kind": "pickup",  "pair": 2,
     "known": False, "done": False, "reachable": None},

    {"name": "pickup_3", "pos": (-0.50,  0.25), "kind": "pickup",  "pair": 3,
     "known": False, "done": False, "reachable": None},

    {"name": "pickup_4", "pos": (-0.0, -1.25), "kind": "pickup",  "pair": 4,
     "known": False, "done": False, "reachable": None},

    {"name": "pickup_5", "pos": ( 0.00,  1.4), "kind": "pickup",  "pair": 5,
     "known": False, "done": False, "reachable": None},

    # --- DROPOFF GOALS ---
    {"name": "dropoff_1", "pos": (-1.20,  0.15), "kind": "dropoff", "pair": 1,
     "known": False, "done": False, "reachable": None},

    {"name": "dropoff_2", "pos": ( 1.4,  1.0), "kind": "dropoff", "pair": 2,
     "known": False, "done": False, "reachable": None},

    {"name": "dropoff_3", "pos": ( 1.15, -0.5), "kind": "dropoff", "pair": 3,
     "known": False, "done": False, "reachable": None},

    {"name": "dropoff_4", "pos": (-0.50, -0.90), "kind": "dropoff", "pair": 4,
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


def update_goals(pose, slam, goals=GOALS, known_list=KNOWN_GOALS):
    """
    Update goals using the current SLAM pose: discover reachable goals within **DETECT_RADIUS**, and when within **REACHED_RADIUS**, 
    complete pickups (if capacity allows) or dropoffs (if carrying the matching item).
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


def select_next_goal(pose, goals=GOALS):
    """
    Select the next goal by going to the nearest valid target:
    - If carrying nothing, go to the nearest unfinished pickup.
    - If carrying something, go to the nearest dropoff whose paired item is being carried (or any unfinished dropoff if none match).
    Return **None** if no goals remain.
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


# ----------------- EXPLORATION HELPERS -----------------

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
    Find the nearest occupancy-grid cell whose probability lies in *(p_min, p_max)* and belongs to a meaningful unknown region (not just a 1×1 patch).
    Return its world-coordinate *(x, y)*, or **None** if none exists.
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