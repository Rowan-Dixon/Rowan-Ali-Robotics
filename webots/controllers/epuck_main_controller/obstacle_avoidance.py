import math


def obstacle_avoidance_override(ranges, pose, goal, left_speed, right_speed, max_speed):
    #turns on spot in direction of goal if obstacle too close
    closese_point = min(ranges)
    index = ranges.index(closese_point)
    if closese_point < 0.15 and (index >= 270 or index <= 90): 
        # print("OBSTACLE AVOIDANCE OVERRIDE: obstacle ", closese_point, "m away at ", index, "degrees")
        # dx, dy = goal[0] - pose[0], goal[1] - pose[1]
        # angle_to_goal = math.atan2(dy, dx)
        # angle_diff = angle_to_goal - pose[2]
        # angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))
        if index <= 90 > 0:
            return -max_speed/2, max_speed/2 
        else: 
            return max_speed/2, -max_speed/2 
    else:
        return left_speed, right_speed

# def obstacle_avoidance_override(ranges, left_speed, right_speed,
#                                 front_thresh=0.25, side_thresh=0.20,
#                                 turn_speed=3.0):
#     """
#     Simple reactive LiDAR-based avoidance: turn in place if an obstacle is directly ahead, turn away if one is close on a side, otherwise keep the PathPlanner speeds.
#     """
#     n = len(ranges)
#     if n == 0:
#         return left_speed, right_speed

#     # assuming ranges[0] is front and indices increase clockwise
#     front_indices = list(range(350, 360)) + list(range(0, 10))
#     left_indices = list(range(300, 350))
#     right_indices = list(range(10, 60))

#     def min_in_sector(idxs):
#         vals = [ranges[i] for i in idxs if 0 < ranges[i] < float('inf')]
#         return min(vals) if vals else float('inf')

#     d_front = min_in_sector(front_indices)
#     d_left = min_in_sector(left_indices)
#     d_right = min_in_sector(right_indices)

#     # Hard front avoidance
#     if d_front < front_thresh:
#         if d_left > d_right:
#             return -turn_speed, turn_speed
#         else:
#             return turn_speed, -turn_speed

#     # Side nudges
#     if d_left < side_thresh and d_right >= side_thresh:
#         return left_speed + 0.0, right_speed + 1.0
#     if d_right < side_thresh and d_left >= side_thresh:
#         return left_speed + 1.0, right_speed + 0.0

#     return left_speed, right_speed