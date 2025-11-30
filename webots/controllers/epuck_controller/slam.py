import math

class OccupancyGrid:
    def __init__(self, width_m, height_m, resolution):
        self.width_m = width_m
        self.height_m = height_m
        self.resolution = resolution

        self.width_cells = int(width_m / resolution)
        self.height_cells = int(height_m / resolution)

        self.log_odds = [[0.0 for _ in range(self.width_cells)]
                         for _ in range(self.height_cells)]

        self.l_occupied = 0.9
        self.l_free = -0.7
        self.l_min = -4.0
        self.l_max = 4.0

    def world_to_map(self, x, y):
        mx = int((x + self.width_m / 2.0) / self.resolution)
        my = int((y + self.height_m / 2.0) / self.resolution)
        if 0 <= mx < self.width_cells and 0 <= my < self.height_cells:
            return mx, my
        return None

    def _update_cell_log_odds(self, mx, my, delta_l):
        if 0 <= mx < self.width_cells and 0 <= my < self.height_cells:
            self.log_odds[my][mx] += delta_l
            if self.log_odds[my][mx] > self.l_max:
                self.log_odds[my][mx] = self.l_max
            elif self.log_odds[my][mx] < self.l_min:
                self.log_odds[my][mx] = self.l_min

    def _bresenham_line(self, x0, y0, x1, y1):
        points = []
        dx = abs(x1 - x0)
        sx = 1 if x0 < x1 else -1
        dy = -abs(y1 - y0)
        sy = 1 if y0 < y1 else -1
        err = dx + dy
        x, y = x0, y0

        while True:
            points.append((x, y))
            if x == x1 and y == y1:
                break
            e2 = 2 * err
            if e2 >= dy:
                err += dy
                x += sx
            if e2 <= dx:
                err += dx
                y += sy
        return points

    def update_with_ray(self, pose, r, a, max_range):
        x, y, theta = pose

        if r <= 0.0 or r > max_range:
            return

        ray_theta = theta + a

        end_x = x + r * math.cos(ray_theta)
        end_y = y + r * math.sin(ray_theta)

        start = self.world_to_map(x, y)
        end = self.world_to_map(end_x, end_y)
        if start is None or end is None:
            return

        sx, sy = start
        ex, ey = end

        line_cells = self._bresenham_line(sx, sy, ex, ey)

        for (cx, cy) in line_cells[:-1]:
            self._update_cell_log_odds(cx, cy, self.l_free)

        end_cell = line_cells[-1]
        self._update_cell_log_odds(end_cell[0], end_cell[1], self.l_occupied)

    def get_probability_map(self):
        probs = [[0.0 for _ in range(self.width_cells)]
                 for _ in range(self.height_cells)]
        for j in range(self.height_cells):
            for i in range(self.width_cells):
                l = self.log_odds[j][i]
                p = 1.0 - 1.0 / (1.0 + math.exp(l))
                probs[j][i] = p
        return probs

    def get_log_odds_map(self):
        return self.log_odds
        

class Slam:
    """
    Simple occupancy-grid SLAM scaffold.

    For now:
      - Pose in = pose out (no correction, we just trust odometry).
      - Map = updated using range measurements.
    Later:
      - We can add a filter inside this class to correct the pose.
    """
    def __init__(self, map_width_m, map_height_m, resolution,
                 sensor_angles, max_range):
        """
        sensor_angles: list of angles (rad) for each range measurement, 
                       relative to robot forward (0 rad).
        max_range: maximum valid range of the sensors (meters).
        """
        self.grid = OccupancyGrid(map_width_m, map_height_m, resolution)
        self.sensor_angles = sensor_angles
        self.max_range = max_range
        self._pose = (0.0, 0.0, 0.0)

    def update(self, odom_pose, ranges):
        self._pose = odom_pose

        for r, a in zip(ranges, self.sensor_angles):
            self.grid.update_with_ray(self._pose, r, a, self.max_range)

        return self._pose

    def get_pose(self):
        return self._pose

    def get_map(self):
        return self.grid.get_probability_map()

    def get_log_odds_map(self):
        return self.grid.get_log_odds_map()
