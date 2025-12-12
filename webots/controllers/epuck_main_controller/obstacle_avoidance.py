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