import math
import heapq
from map_debug import plot_map
from obstacle_avoidance import obstacle_avoidance_override

'''
To use: 
    * Instantiate once - path = PathPlanner(pose_slam, (2.0, -0.5))
    * call go to point each timestep - left_speed, right_speed, dist = path.go_to_point(pose_slam, ka=10.0, kd=10.0)
To change the goal: re-instantiate with new goal
'''

class PathPlanner:
    def __init__(self, pose, slam_map=None, goal=[]):
        '''
        initialise empty by not setting slam map
        otherwise, it will plan path to goal
        goal will be nearest unexplored cell if not given
        '''
        self.points, self.goal = self.return_path(pose, slam_map, goal)
        if self.points and self.goal is not None:
            dx = self.goal[0] - pose[0] 
            dy = self.goal[1] - pose[1]
            self.goal_dist = math.hypot(dx, dy)
            self.next_point = self.points.pop(0) if self.points else None
        else: 
            self.goal_dist = 0.0
            self.points = []
            self.next_point = None

    def return_path(self, pose, slam_map, goal=None):
        print("Planning path to", goal)

        prob_map = slam_map.get_probability_map()
        grid = slam_map.grid

        # add padding to obstacles to account for robot size
        inflated = self.inflate_obstacles(prob_map,
                                    occ_threshold=0.3,
                                    robot_radius_m=0.04,   
                                    resolution=grid.resolution)
        
        #if no goal, go to nearest known free space, if no free space, return empty path
        # unexplored area must border explored area for a path to be possible
        if not goal:
            goal = get_region_to_explore(self, inflated, prob_map, grid)

        #translate world to map (origin in centre instead of bottom left)
        start_cell = grid.world_to_map(pose[0], pose[1])
        goal_cell = grid.world_to_map(goal[0], goal[1])
        if start_cell is None or goal_cell is None:
            print("Start/goal out of map!")
            return [], None

        path_cells = self.astar(inflated, start_cell, goal_cell)
        if not path_cells:
            print("NO PATH FOUND")
            return [], None

        #translate back
        waypoints = []
        res = grid.resolution
        for cx, cy in path_cells:
            wx = cx * res - grid.width_m/2.0
            wy = cy * res - grid.height_m/2.0
            waypoints.append((wx, wy))

        plot_map(inflated, grid, path=waypoints, point=goal)
        return waypoints, goal

    def go_to_point(self, pose, ranges, lookahead=0.22, max_speed=5.0, ka=3.0):

        if self.goal is None or len(self.points) == 0:
            return 0.0, 0.0

        x, y, theta = pose
        self.goal_dist = math.hypot(self.goal[0] - x, self.goal[1] - y)
        closest_i = 0
        closest_dist = float('inf')

        # find closes point on path and dist
        for i, (px, py) in enumerate(self.points):
            d = (x - px)**2 + (y - py)**2
            if d < closest_dist:
                closest_dist = d
                closest_i = i
        closest_dist = math.sqrt(closest_dist)

        # lookahead is carrot on stick algorithm
        target = None
        for i in range(closest_i, len(self.points)):
            px, py = self.points[i]
            if math.hypot(px - x, py - y) >= lookahead:
                target = (px, py)
                break
        if target is None:
            target = self.points[-1]
        tx, ty = target

        # (to look ahead point)
        desired_theta = math.atan2(ty - y, tx - x)
        heading_error = math.atan2(
            math.sin(desired_theta - theta),
            math.cos(desired_theta - theta)
        )

        drift = closest_dist
        drift_factor = max(0.3, 1.0 - drift * 1.8)

        # slow down for sharp turns
        turn_factor = max(0.4, 1.0 - abs(heading_error) * 0.7)

        v = max_speed * drift_factor * turn_factor
        w = ka * heading_error
        L = 0.178
        left = v - (L/2) * w
        right = v + (L/2) * w
        wheel_max = 6.67 #will error above this number
        scale = max(1.0, abs(left) / wheel_max, abs(right) / wheel_max)
        left /= scale
        right /= scale

        left, right = obstacle_avoidance_override(ranges, pose, target, left, right, max_speed)
        return left, right
    
    def inflate_obstacles(self, prob_map, occ_threshold, robot_radius_m, resolution):
        H = len(prob_map)
        W = len(prob_map[0])

        inflation_cells = int(robot_radius_m / resolution)
        inflated = [[0 for _ in range(W)] for _ in range(H)]

        for y in range(H):
            for x in range(W):
                if prob_map[y][x] > occ_threshold:
                    for dy in range(-inflation_cells, inflation_cells+1):
                        for dx in range(-inflation_cells, inflation_cells+1):
                            nx = x + dx
                            ny = y + dy
                            if 0 <= nx < W and 0 <= ny < H: #if inside map, treat as occupied
                                inflated[ny][nx] = 1
        return inflated


    def astar(self, grid, start, goal):
        H = len(grid)
        W = len(grid[0])
        sx, sy = start
        gx, gy = goal

        pq = []
        heapq.heappush(pq, (0, (sx, sy)))
        came_from = {}
        g_score = { (sx, sy): 0 }

        neighbors = [ (1,0), (-1,0), (0,1), (0,-1),
                    (1,1), (1,-1), (-1,1), (-1,-1) ]

        while pq:
            _, current = heapq.heappop(pq)
            if current == (gx, gy):
                path = [] #reconstruct path
                node = current
                while node in came_from:
                    path.append(node)
                    node = came_from[node]
                path.append((sx, sy))
                return path[::-1]

            cx, cy = current
            for dx, dy in neighbors:
                nx = cx + dx
                ny = cy + dy
                if not (0 <= nx < W and 0 <= ny < H):
                    continue
                if grid[ny][nx] == 1:  # blocked
                    continue

                new_cost = g_score[current] + math.hypot(dx, dy)
                if (nx, ny) not in g_score or new_cost < g_score[(nx, ny)]:
                    g_score[(nx, ny)] = new_cost
                    priority = new_cost + math.hypot(gx - nx, gy - ny)
                    heapq.heappush(pq, (priority, (nx, ny)))
                    came_from[(nx, ny)] = current
        return None
    
    def get_region_to_explore(self, inflated, prob_map, grid):
        '''
        I sent you what inflated map and prob map looks like
        inflated adds a cushion to all obstacles so path wont brush past and crash into the wall
        inflated also treats unknown areas as occuipied 

        needs to return a goal which is an explored area but borders an unexplored area
        so that the lidar will scan that unexplored area
        '''
        return [0.0, 0.0]  # Placeholder implementation