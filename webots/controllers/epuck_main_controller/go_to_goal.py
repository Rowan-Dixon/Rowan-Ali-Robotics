import math

'''
To use: 
    * Instantiate once - path = PathPlanner(pose_slam, (2.0, -0.5))
    * call go to point each timestep - left_speed, right_speed, dist = path.go_to_point(pose_slam, ka=10.0, kd=10.0)
'''

class PathPlanner:
    def __init__(self, pose, goal):
        self.goal = goal
        self.points = self.return_path(pose, goal)
        self.next_point = self.points.pop(0)

    def return_path(self, pose, goal):
        return [goal]

    def go_to_point(self, pose, max_speed=3.0, ka=2.0, kd=1.0):
        """
        ka: angular gain (slows more when turning, less smooth curves)
        kd: distance gain (forward motion)
        """
        x, y, theta = pose
        gx, gy = self.next_point

        # --- Compute angle and distance ---
        dx = gx - x
        dy = gy - y
        distance = math.sqrt(dx*dx + dy*dy)

        # desired heading
        desired_theta = math.atan2(dy, dx)

        # if angle is too big, it turns too widely. so here it will turn on the spot instead
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
        L = 0.178   # axle length from odom
        left = v - (L/2.0) * w
        right = v + (L/2.0) * w

        if self.points:
            if distance < 0.2 and self.points:
                self.next_point = self.points.pop(0)

        if not self.points:
            if distance < 0.05:
                left = 0.0
                right = 0.0

        return left, right, distance
