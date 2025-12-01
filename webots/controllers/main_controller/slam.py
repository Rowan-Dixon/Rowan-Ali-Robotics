import math


class OccupancyGrid:
    def __init__(self, width_m: float, height_m: float, resolution: float):
        "Initializes the occupancy grid with the given dimensions and resolution."
        self.width_m = width_m
        self.height_m = height_m
        self.resolution = resolution

        self.width_cells = int(width_m / resolution)
        self.height_cells = int(height_m / resolution)

        self.log_odds = [
            [0.0 for _ in range(self.width_cells)]
            for _ in range(self.height_cells)
        ]

        self.l_occupied = 0.9
        self.l_free = -0.7

        self.l_min = -4.0
        self.l_max = 4.0

    def word_to_map(self, x: float, y: float):
        "Converts world coordinates (in meters) to map cell indices."
        mx = int((x + (self.width_cells * self.resolution) / 2) / self.resolution)
        my = int((y + (self.height_cells * self.resolution) / 2) / self.resolution)
        if 0 <= mx < self.width_cells and 0 <= my < self.height_cells:
            return mx, my
        return None

    # alias with the correct spelling, in case other code calls world_to_map
    def world_to_map(self, x: float, y: float):
        return self.word_to_map(x, y)

    def update_cell_log_odds(self, mx: int, my: int, delta_l: float) -> None:
        "Updates the log-odds value of a cell."
        if 0 <= mx < self.width_cells and 0 <= my < self.height_cells:
            self.log_odds[my][mx] += delta_l
            if self.log_odds[my][mx] > self.l_max:
                self.log_odds[my][mx] = self.l_max
            elif self.log_odds[my][mx] < self.l_min:
                self.log_odds[my][mx] = self.l_min

    def _bresenham_line(self, x0: int, y0: int, x1: int, y1: int):
        "Generates the cells along a line using Bresenham's algorithm."
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

    def update_with_lidar(self, pose, r: float, a: float, max_range: float) -> None:
        "Updates the occupancy grid based on a LIDAR measurement."
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
            self.update_cell_log_odds(cx, cy, self.l_free)

        end_cell = line_cells[-1]
        self.update_cell_log_odds(end_cell[0], end_cell[1], self.l_occupied)

    def get_probability_map(self):
        "Returns the occupancy grid as a probability map."
        probs = [
            [0.0 for _ in range(self.width_cells)]
            for _ in range(self.height_cells)
        ]
        for j in range(self.height_cells):
            row_lo = self.log_odds[j]
            row_p = probs[j]
            for i in range(self.width_cells):
                l = row_lo[i]
                row_p[i] = 1.0 - 1.0 / (1.0 + math.exp(l))
        return probs

    def get_log_odds_map(self):
        "Returns the occupancy grid as a log-odds map."
        return self.log_odds

    def ascii_debug_print(self, threshold=0.5, step=2) -> None:
        "Prints an ASCII representation of the occupancy grid for debugging."
        prob_map = self.get_probability_map()
        for j in range(0, self.height_cells, step):
            line = ""
            for i in range(0, self.width_cells, step):
                p = prob_map[j][i]
                if p < 0.3:
                    line += "."   # mostly free
                elif p > threshold:
                    line += "#"   # likely occupied
                else:
                    line += " "   # unknown-ish
            print(line)
        print("-" * 40)


class Slam:
    def __init__(
        self,
        map_width_m: float,
        map_height_m: float,
        resolution: float,
        sensor_angles,
        max_range: float,
    ):
        "Initializes the SLAM module with an occupancy grid map."
        self.grid = OccupancyGrid(map_width_m, map_height_m, resolution)
        self.sensor_angles = list(sensor_angles)
        self.max_range = max_range

        self._pose = (0.0, 0.0, 0.0)

    def update(self, odom_pose, ranges):
        "Updates the SLAM map with new odometry and LIDAR data."
        # For now just trust odometry for the pose
        self._pose = odom_pose

        for r, a in zip(ranges, self.sensor_angles):
            self.grid.update_with_lidar(self._pose, r, a, self.max_range)
        return self._pose

    def get_pose(self):
        return self._pose

    def get_probability_map(self):
        return self.grid.get_probability_map()

    def get_log_odds_map(self):
        return self.grid.get_log_odds_map()

    def debgug_print_map(self):
        # keep your original name but call ascii_debug_print inside
        self.grid.ascii_debug_print()
