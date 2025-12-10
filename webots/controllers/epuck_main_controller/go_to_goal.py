import math


def go_to_point(pose, goal, max_speed=3.0, ka=2.0, kd=1.0):
    """
    ka: angular gain (slows more when turning, less smooth curves)
    kd: distance gain (forward motion)
    """
    x, y, theta = pose
    gx, gy = goal

    # if abs(theta - math.atan2(gy - y, gx - x)) > math.pi / 2:
    #     return 

    # print("theta: ", theta, "goal angle: ", math.atan2(gy - y, gx - x), "difference: ", math.degrees(theta - math.atan2(gy - y, gx - x)))

    # --- Compute angle and distance ---
    dx = gx - x
    dy = gy - y
    distance = math.sqrt(dx*dx + dy*dy)

    # desired heading
    desired_theta = math.atan2(dy, dx)

    if theta - desired_theta > 1:
        return max_speed, -max_speed, distance
    if theta - desired_theta < -1:
        return -max_speed, max_speed, distance

    # smallest angle difference [-pi, pi]
    angle_error = math.atan2(math.sin(desired_theta - theta),
                             math.cos(desired_theta - theta))

    # --- Controller ---
    # rotational speed
    w = ka * angle_error

    # forward speed gated by angle error
    v = kd * distance * max(0.0, 1.0 - abs(angle_error))
    v = max(v, 2.0)

    # clamp
    v = max(min(v, max_speed), -max_speed)
    w = max(min(w, max_speed), -max_speed)

    # convert (v, w) to differential wheel speeds
    L = 0.178   # your axle length from odom
    left = v - (L/2.0) * w
    right = v + (L/2.0) * w
    return left, right, distance


def get_next_point(pose, goal):
    return (0.5,0.5)