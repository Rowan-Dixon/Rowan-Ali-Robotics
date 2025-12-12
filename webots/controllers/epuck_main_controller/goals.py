# goals_manager.py
import math
from go_to_goal import PathPlanner

#written by ali, pushed by rowan with some edits

# ----------------- DEFAULT GOAL DEFINITIONS (INITIALLY UNKNOWN) -----------------
PICKUP_GOALS = [
    {"name": "pickup_1", "pos": (1.25,  1.00), "kind": "pickup", "pair": 1,},
    {"name": "pickup_2", "pos": (0.60, -0.25), "kind": "pickup", "pair": 2,},
    {"name": "pickup_3", "pos": (-1.00, 0.00), "kind": "pickup", "pair": 3,},
    {"name": "pickup_4", "pos": (-0.50, -1.25), "kind": "pickup", "pair": 4,},
    {"name": "pickup_5", "pos": (0.00,  1.40), "kind": "pickup", "pair": 5,},
]

DROPOFF_GOALS = [
    {"name": "dropoff_1", "pos": (-1.20,  0.15), "kind": "dropoff", "pair": 1,},
    {"name": "dropoff_2", "pos": (1.40,  0.50), "kind": "dropoff", "pair": 2,},
    {"name": "dropoff_3", "pos": (1.15, -0.50), "kind": "dropoff", "pair": 3,},
    {"name": "dropoff_4", "pos": (-0.50, -0.90), "kind": "dropoff", "pair": 4,},
    {"name": "dropoff_5", "pos": (1.00, -0.00), "kind": "dropoff", "pair": 5,},
]

class Goals:
    """
    Handles unknown goals, discovers them when nearby, selects which one to pursue, ensures a valid path exists, and manages storage for carrying up to three items.
    """

    def __init__(self):
        self.unknown_pick_ups = PICKUP_GOALS.copy()
        self.unknown_drop_offs = DROPOFF_GOALS.copy()

        self.goals = []

        self.current_goal = None

        self.storage = []
        self.storage_capacity = 3

        self.detect_radius = 0.12
        self.reached_radius = 0.08

    def discover_goals(self, pose, slam_map):
        x, y, _ = pose
        grid = slam_map.grid
        prob_map = grid.get_probability_map()

        new_pickups = []
        for g in self.unknown_pick_ups:
            wx, wy = g["pos"]
            mx, my = grid.world_to_map(wx, wy)
            if prob_map[my][mx] < 0.3:  # free space
                print(f"[GOALS] DISCOVERED pickup {g['name']} at {g['pos']}")
                self.goals.append(g)
            else:
                new_pickups.append(g)

        self.unknown_pick_ups = new_pickups


        new_dropoffs = []
        for g in self.unknown_drop_offs:
            gx, gy = g["pos"]
            if math.hypot(gx - x, gy - y) < self.detect_radius:
                print(f"[GOALS] DISCOVERED dropoff {g['name']} at {g['pos']}")
                self.goals.append(g)
            else:
                new_dropoffs.append(g)
        self.unknown_drop_offs = new_dropoffs

    # SORTING (closest first)
    def sort_goals(self, pose, goals_list):
        x, y, _ = pose
        return sorted(goals_list, key=lambda g: math.hypot(g["pos"][0] - x, g["pos"][1] - y))

    # INTERNALLY BUILD CANDIDATE LIST
    def _get_candidates(self):
        pickups = [g for g in self.goals if g["kind"] == "pickup"]
        dropoffs = [g for g in self.goals if g["kind"] == "dropoff"]
        dropoffs_matching = [g for g in dropoffs if g["pair"] in self.storage]

        if len(self.storage) >= self.storage_capacity:
            return dropoffs_matching or dropoffs
        else:
            return pickups + dropoffs_matching

        return dropoffs

    # SELECT NEXT GOAL using valid path
    def choose_goal(self, pose, slam_map):
        if self.current_goal is not None:
            return self.current_goal

        candidates = self._get_candidates()
        if not candidates:
            return None

        sorted_candidates = self.sort_goals(pose, candidates)

        for g in sorted_candidates:
            planner = PathPlanner(pose, slam_map, goal=g["pos"])

            if planner.points:
                print(f"[GOALS] SELECTED {g['kind']} {g['name']} at {g['pos']}")
                self.current_goal = g
                self.goals.remove(g)
                return g

            else:
                print(f"[GOALS] NO PATH to {g['name']} – skipping")

        self.current_goal = None
        return None

    # CHECK IF REACHED GOAL + UPDATE STORAGE
    def goal_reached(self):
        
        pair = self.current_goal["pair"]
        kind = self.current_goal["kind"]

        print(f"[GOALS] REACHED {kind} {self.current_goal['name']}")

        if kind == "pickup":
            if len(self.storage) < self.storage_capacity:
                self.storage.append(pair)
                print(f"[GOALS] PICKED UP pair {pair} → storage = {self.storage}")
            else:
                print("[GOALS] STORAGE FULL!")

        elif kind == "dropoff":
            if pair in self.storage:
                self.storage.remove(pair)
                print(f"[GOALS] DROPPED OFF pair {pair} → storage = {self.storage}")
            else:
                print("[GOALS] WRONG DROP-OFF (pair not carried)")

        self.current_goal = None
        return True


# Create ONE global instance
goals_manager = Goals()