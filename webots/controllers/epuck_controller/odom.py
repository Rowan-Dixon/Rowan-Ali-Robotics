import math

class Odometry:
    def __init__(self, wheel_radius: float, axle_length: float):
        "Initializes the odometry with the given wheel radius and axle length."
        self.wheel_radius = wheel_radius
        self.axle_length = axle_length
        self.last_left_wheel_pos = None # type: float | None
        self.last_right_wheel_pos = None # type: float | None
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

    def rest(self, left_enc: float, right_enc: float, x:float =0.0, y: float =0.0, theta: float = 0.0) -> None:
        "Resets the odometry to the given position and wheel encoder readings."
        self.last_left_wheel_pos = left_enc
        self.last_right_wheel_pos = right_enc
        self.x = x
        self.y = y
        self.theta = theta

    def update(self, left_enc: float, right_enc: float):
        "Updates the robots position based on wheel encoder readings."
        "retutns the new positsion (x, y, theta)"
        if self.last_left_wheel_pos is None or self.last_right_wheel_pos is None:
            self.reset(left_enc, right_enc, self.c, self.y, self.theta)
            return self.x, self.y, self.theta
        dl_rad = left_enc - self.last_left_wheel_pos
        dr_rad = right_enc - self.last_right_wheel_pos

        self.last_left_wheel_pos = left_enc
        self.last_right_wheel_pos = right_enc

        dl = self.wheel_radius * dl_rad
        dr = self.wheel_radius * dr_rad

        ds = (dl + dr) / 2.0
        dtheta = (dr - dl) / self.axle_length

        theta_mid = self.theta + dtheta / 2.0
        self.x += ds * math.cos(theta_mid)
        self.y += ds * math.sin(theta_mid)

        self.theta = math.atan2(
            math.sin(self.theta + dtheta),
            math.cos(self.theta + dtheta)
        )

        return self.x, self.y, self.theta
