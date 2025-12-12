import math
from go_to_goal import PathPlanner

# ----------------- INTERNAL STATE FOR NAVIGATION -----------------
current_planner = None
current_target_pos = None

explore_target_cached = None
explore_target_age = 0


def obstacle_avoidance_override(ranges, left_speed, right_speed,
                                front_thresh=0.25, side_thresh=0.20,
                                turn_speed=3.0):
    """
    Simple reactive LiDAR-based avoidance: turn in place if an obstacle is directly ahead, turn away if one is close on a side, otherwise keep the PathPlanner speeds.
    """
    n = len(ranges)
    if n == 0:
        return left_speed, right_speed

    # assuming ranges[0] is front and indices increase clockwise
    front_indices = list(range(350, 360)) + list(range(0, 10))
    left_indices = list(range(300, 350))
    right_indices = list(range(10, 60))

    def min_in_sector(idxs):
        vals = [ranges[i] for i in idxs if 0 < ranges[i] < float('inf')]
        return min(vals) if vals else float('inf')

    d_front = min_in_sector(front_indices)
    d_left = min_in_sector(left_indices)
    d_right = min_in_sector(right_indices)

    # Hard front avoidance
    if d_front < front_thresh:
        if d_left > d_right:
            return -turn_speed, turn_speed
        else:
            return turn_speed, -turn_speed

    # Side nudges
    if d_left < side_thresh and d_right >= side_thresh:
        return left_speed + 0.0, right_speed + 1.0
    if d_right < side_thresh and d_left >= side_thresh:
        return left_speed + 1.0, right_speed + 0.0

    return left_speed, right_speed


def compute_wheel_speeds(pose_est, rbpf_slam, ranges):
    """
    High-level navigation: update goals, choose a task goal or exploration target, plan a path, then apply obstacle avoidance. Returns *(left_speed, right_speed)*.
    """
    global current_planner, current_target_pos
    global explore_target_cached, explore_target_age

    # ----- Update goals / capacity state -----
    update_goals(pose_est, rbpf_slam)

    # ----- TASK GOAL SELECTION -----
    task_goal = select_next_goal(pose_est)

    if task_goal is not None:
        desired_target_pos = task_goal["pos"]
        explore_target_cached = None
        explore_target_age = 0
    else:
        # no task goal -> exploration
        explore_target_age += 1

        if explore_target_cached is None or explore_target_age > 50:
            explore_target_cached = find_exploration_target(pose_est, rbpf_slam.grid)
            explore_target_age = 0
            if explore_target_cached is not None:
                print(f"[TARGET] New exploration target at {explore_target_cached}")
            else:
                print("[TARGET] No exploration frontier found (map mostly known).")

        desired_target_pos = explore_target_cached

    # ----- PATH PLANNING -----
    if desired_target_pos is not None:
        if (
            current_planner is None
            or current_target_pos is None
            or math.hypot(
                desired_target_pos[0] - current_target_pos[0],
                desired_target_pos[1] - current_target_pos[1]
            ) > 0.05
        ):
            current_planner = PathPlanner(pose_est, rbpf_slam, goal=desired_target_pos)
            current_target_pos = desired_target_pos
            print(f"[PLANNER] New target set at {current_target_pos}")

        if not current_planner.points:
            left_speed, right_speed = 0.5, -0.5
        else:
            left_speed, right_speed = current_planner.go_to_point(pose_est)

    else:
        current_planner = None
        current_target_pos = None
        left_speed = 1.0
        right_speed = 1.0

    left_speed, right_speed = obstacle_avoidance_override(
        ranges, left_speed, right_speed
    )

    return left_speed, right_speed
