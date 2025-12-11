import math
import random
from slam import OccupancyGrid

class RBPF_SLAM:
    """
    GMapping-style Rao-Blackwellised Particle Filter SLAM:"
    - particle filter over robot pose
    - single global occupancy grid map
    """""
    def __init__ (self, map_width_m: float, map_height_m: float, resolution: float, sensor_angles, max_range: float, num_particles: int = 40):
        self.grid = OccupancyGrid(map_width_m, map_height_m, resolution)
        self.sensor_angles = list(sensor_angles)
        self.max_range = max_range
        self.num_particles = num_particles

        #particles: list of (x, y, theta, weight)
        self.particles = []
        self._pose = (0.0, 0.0, 0.0)
        self._last_odom = None

        self._init_particles_uniform()

    def _init_particles_uniform(self):
        "Initialise particles around the origin with random headings."
        self.particles = []
        for _ in range(self.num_particles):
            x = 0.0
            y = 0.0
            theta = 0.0 # random.uniform(-math.pi, math.pi)
            w = 1.0 / self.num_particles
            self.particles.append([x, y, theta, w])
    
    def _motion_update(self, odom_pose):
        if self._last_odom is None:
            self._last_odom = odom_pose
            return
        
        x_old, y_old, theta_old = self._last_odom
        x_new, y_new, theta_new = odom_pose

        dx = x_new - x_old
        dy = y_new - y_old
        dtheta = math.atan2(math.sin(theta_new - theta_old), math.cos(theta_new - theta_old))

        self._last_odom = odom_pose

        for p in self.particles:
            px, py, pth, pw = p

            # Add small Gaussian noise to motion
            ndx = dx + random.gauss(0, 0.03)
            ndy = dy + random.gauss(0, 0.03)
            ndtheta = dtheta + random.gauss(0, 0.001)

            p[0] = px + ndx
            p[1] = py + ndy
            p[2] = math.atan2(math.sin(pth + ndtheta), math.cos(pth + ndtheta))
    
    def _measurement_likelihood(self, px, py, pth, ranges, beam_step=10):
        #Increase sigma_r -> SLAM trusts LiDAR more and odom less -> more map drift, less jitter/noise
        sigma_r = 0.05  # 5 cm range noise
        score = 0.0
        count = 0

        for i in range(0, len(self.sensor_angles), beam_step):
            r_mes = ranges[i]
            if not (0.0 < r_mes < self.max_range):
                continue

            angle = self.sensor_angles[i]
            ray_theta = pth + angle

            r_exp = self.grid.expected_range(px, py, ray_theta, self.max_range)

            diff = r_mes - r_exp
            score += -(diff * diff) / (2.0 * sigma_r * sigma_r) #gaussian range model
            count += 1

        if count == 0:
            return 1e-9
        
        avg_score = score / count
        return math.exp(avg_score)
    
    def _measurement_update(self, ranges):
        total_weight = 0.0

        for p in self.particles:
            px, py, pth, pw = p
            likelihood = self._measurement_likelihood(px, py, pth, ranges)
            p[3] = pw * likelihood
            total_weight += p[3]

        if total_weight == 0.0:
            self._init_particles_uniform()
            return
        
        for p in self.particles:
            p[3] /= total_weight

    def _resample_particles(self):
        "Low-variance resampling."
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

    def _estimate_pose(self):
        "Weighted average of particle poses."
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
    
    def _map_update(self, pose, ranges, beam_step = 5):
        "Update occupancy grid using estimated pose and LiDAR"

        x, y, theta = pose
        for i in range(0, len(self.sensor_angles), beam_step):
            r = ranges[i]
            a = self.sensor_angles[i]
            self.grid.update_with_lidar((x, y, theta), r, a, self.max_range)

    def update(self, odom_pose, ranges):
        "Main RBPF SLAM step. Returns estimated pose (x, y, theta)."
        self._motion_update(odom_pose)
        self._measurement_update(ranges)
        self._resample_particles()
        self._pose = self._estimate_pose()
        self._map_update(self._pose, ranges)
        return self._pose
    
    def get_pose(self):
        return self._pose
    
    def get_probability_map(self):
        return self.grid.get_probability_map()
    
    def get_log_odds_map(self):
        return self.grid.get_log_odds_map()
    
    def debug_print_map(self):
        self.grid.ascii_debug_print()
