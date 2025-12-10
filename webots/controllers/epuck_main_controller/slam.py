# slam.py
import math
import os
import random


class OccupancyGrid:
    def __init__(self, width_m: float, height_m: float, resolution: float):
        """Initializes the occupancy grid with the given dimensions and resolution."""
        self.width_m = width_m
        self.height_m = height_m
        self.resolution = resolution

        self.width_cells = int(width_m / resolution)
        self.height_cells = int(height_m / resolution)

        # log-odds initialised to 0 (unknown)
        self.log_odds = [
            [0.0 for _ in range(self.width_cells)]
            for _ in range(self.height_cells)
        ]

        self.l_occupied = 0.9
        self.l_free = -0.7
        self.l_min = -4.0
        self.l_max = 4.0

        self._prob_map_cache = None
        self._prob_map_dirty = True

    # ------------- coords -------------

    def world_to_map(self, x: float, y: float):
        """Converts world coordinates (m) to map cell indices (mx, my)."""
        mx = int((x + self.width_m / 2.0) / self.resolution)
        my = int((y + self.height_m / 2.0) / self.resolution)
        if 0 <= mx < self.width_cells and 0 <= my < self.height_cells:
            return mx, my
        return None

    # ------------- core ops -------------

    def update_cell_log_odds(self, mx: int, my: int, delta_l: float) -> None:
        "Updates the log-odds value of a cell."
        if 0 <= mx < self.width_cells and 0 <= my < self.height_cells:
            # update log-odds with clamping
            cell_lo = self.log_odds[my][mx] + delta_l

            if cell_lo > self.l_max:
                cell_lo = self.l_max
            elif cell_lo < self.l_min:
                cell_lo = self.l_min

            self.log_odds[my][mx] = cell_lo

            # mark probability cache as dirty
            self._prob_map_dirty = True


    def _bresenham_line(self, x0: int, y0: int, x1: int, y1: int):
        """Generates the cells along a line using Bresenham's algorithm."""
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
        """
        Updates the occupancy grid based on a LIDAR-style measurement.
        pose = (x, y, theta), r = range, a = sensor angle relative to robot.
        """
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

        # All cells before the hit are free
        for (cx, cy) in line_cells[:-1]:
            self.update_cell_log_odds(cx, cy, self.l_free)

        # Final cell is occupied
        end_cell = line_cells[-1]
        self.update_cell_log_odds(end_cell[0], end_cell[1], self.l_occupied)

    # ------------- outputs -------------

    def get_probability_map(self):
        "Returns the occupancy grid as a probability map (list[list[float]])."
        if self._prob_map_cache is not None and not self._prob_map_dirty:
            return self._prob_map_cache

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

        self._prob_map_cache = probs
        self._prob_map_dirty = False
        return probs


    def get_log_odds_map(self):
        """Returns the occupancy grid as a log-odds map."""
        return self.log_odds

    def ascii_debug_print(self, threshold=0.5, step=2) -> None:
        """Prints an ASCII representation of the occupancy grid for debugging."""
        os.system('cls' if os.name == 'nt' else 'clear')
        prob_map = self.get_probability_map()
        for j in range(0, self.height_cells, step):
            line = ""
            for i in range(0, self.width_cells, step):
                p = prob_map[j][i]
                if p < 0.3:
                    line += "."   # likely free
                elif p > threshold:
                    line += "#"   # likely occupied
                else:
                    line += " "   # unknown-ish
            print(line)
        print("-" * 40)

    def expected_range(self, x: float, y: float, ray_theta:float, max_range: float, step: float = None, occ_threshold: float = 0.5):

        " Casts a ray from (x, y) in direction ray_theta and returns expected range until hitting an occupied cell or leaving map bounds."

        if step is None:
            step = self.resolution * 2

        dist = 0.0
        prob_map = self.get_probability_map()

        while dist < max_range: 
            px = x + dist * math.cos(ray_theta)
            py = y + dist * math.sin(ray_theta)
            cell = self.world_to_map(px, py)

            if cell is None:
                return dist
            
            mx, my = cell 
            p_occ = prob_map[my][mx]
            if p_occ >= occ_threshold:
                return dist
            
            dist += step
        
        return max_range


class MapLocalizer:
    def __init__(self, grid: OccupancyGrid, sensor_angles, max_range: float, num_particles: int = 80):
        "Localisation on a fixed known map using a simple particle filter."
        
        self.grid = grid
        self.sensor_angles = list(sensor_angles)
        self.max_range = max_range
        self.num_particles = num_particles

        self.particles = []
        self._pose = (0.0, 0.0, 0.0)

        self._last_odom = None

        self._init_particles_uniform()

    def _init_particles_uniform(self):
        self.particles = []
        for _ in range(self.num_particles):
            x = random.uniform(-0.2, 0.2)
            y = random.uniform(-0.2, 0.2)
            theta = random.uniform(-math.pi, math.pi)
            w = 1.0 / self.num_particles
            self.particles.append([x, y, theta, w])

    def _motion_update(self, odom_pose):
        if self._last_odom is None:
            self._last_odom = odom_pose
            return

        x_old, y_old, th_old = self._last_odom
        x_new, y_new, th_new = odom_pose

        dx = x_new - x_old
        dy = y_new - y_old
        dtheta = math.atan2(math.sin(th_new - th_old), math.cos(th_new - th_old))

        self._last_odom = odom_pose

        for p in self.particles:
            px, py, pth, pw = p

            ndx = dx + random.gauss(0, 0.01)
            ndy = dy + random.gauss(0, 0.01)
            ndtheta = dtheta + random.gauss(0, 0.01)

            p[0] = px + ndx
            p[1] = py + ndy
            p[2] = math.atan2(math.sin(pth + ndtheta), math.cos(pth + ndtheta))

    def _measurement_likelihood(self, px, py, pth, ranges, beam_step = 30):
        "Compare LiDAR readings for this particle against map."
        "Returns a likelihood (higher = better)."
        "beam_step: use every Nth beam to save time."
    
        score = 0.0
        count = 0

        for i in range(0, len(self.sensor_angles), beam_step):
            r_meas = ranges[i]
            angle = self.sensor_angles[i]
            ray_theta = pth + angle

            r_exp = self.grid.expected_range(px, py, ray_theta, self.max_range)

            diff = abs(r_meas - r_exp)
            score += -diff
            count += 1

        if count == 0:
            return 1e-6
        
        avg_score = score / count

        return math.exp(avg_score)
    
    def _measurement_update(self, ranges):
        "Update particle weights based on LIDAR."

        total_w = 0.0
        for p in self.particles:
            px, py, pth, w = p
            likelihood = self._measurement_likelihood(px, py, pth, ranges)
            p[3] = w * likelihood
            total_w += p[3]
        
        if total_w == 0.0:
            self._init_particles_uniform()
            return
        
        for p in self.particles:
            p[3] /= total_w

    def _resample_particles(self):
        "Low variance resampling."
        new_particles = []
        weights = [p[3] for p in self.particles]
        if not weights:
            return
        
        N = self.num_particles
        r = random.random() / N
        c = weights[0]
        i = 0

        for m in range(N):
            U = r + m / N
            while U > c and i < N - 1:
                i += 1
                c += weights[i]
            px, py, pth, w = self.particles[i]
            new_particles.append([px, py, pth, 1.0 / N])

        self.particles = new_particles
    
    def estimate_pose(self):
        "Return weighted mean pose of particles."
        if not self.particles:
            return (0.0, 0.0, 0.0)
        
        x_sum = 0.0
        y_sum = 0.0
        cos_sum = 0.0
        sin_sum = 0.0

        for px, py, pth, w in self.particles:
            x_sum += px * w
            y_sum += py * w
            cos_sum += math.cos(pth) * w
            sin_sum += math.sin(pth) * w

        theta = math.atan2(sin_sum, cos_sum)
        return (x_sum, y_sum, theta)
    
    def update(self, odom_pose, ranges):
        "Update localisation estimate using odometry + LIDAR ranges. Returns estimated pose (x, y, theta)."
        self._motion_update(odom_pose)
        self._measurement_update(ranges)
        self._resample_particles()
        self._pose = self.estimate_pose()
        return self._pose
    
    def get_pose(self):
        return self._pose


class Slam:
    def __init__(self,
                 map_width_m: float,
                 map_height_m: float,
                 resolution: float,
                 sensor_angles,
                 max_range: float):
        """Initializes the SLAM module with an occupancy grid map."""
        self.grid = OccupancyGrid(map_width_m, map_height_m, resolution)
        self.sensor_angles = list(sensor_angles)
        self.max_range = max_range

        # current estimated pose
        self._pose = (0.0, 0.0, 0.0)

    def update(self, odom_pose, ranges):
        """Updates the SLAM map with new odometry and range data."""
        # for now we just trust odometry for pose
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

    def debug_print_map(self):
        self.grid.ascii_debug_print()
