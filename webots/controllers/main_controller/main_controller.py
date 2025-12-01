# test_slam.py
import math
from slam import Slam


def main():
    # 2m x 2m map with 2cm cells
    map_width_m = 2.0
    map_height_m = 2.0
    resolution = 0.02

    # Use 5 beams: forward, ±45°, ±90°
    sensor_angles = [
        0.0,
        math.radians(45),
        math.radians(-45),
        math.radians(90),
        math.radians(-90),
    ]
    max_range = 1.0  # 1 meter

    slam = Slam(map_width_m, map_height_m, resolution, sensor_angles, max_range)

    # --- First fake pose + ranges ---
    pose1 = (0.0, 0.0, 0.0)  # at origin, facing +x

    # Imagine a wall ~0.8m in front, 0.6m at ±45°, nothing at ±90°
    ranges1 = [
        0.8,   # forward
        0.6,   # +45°
        0.6,   # -45°
        0.0,   # +90° (ignored)
        0.0,   # -90° (ignored)
    ]

    print("Updating SLAM with first pose and ranges...")
    slam.update(pose1, ranges1)
    slam.debgug_print_map()

    # --- Second fake pose + ranges ---
    pose2 = (0.3, 0.0, math.radians(45))  # moved forward + rotated 45°
    ranges2 = [
        0.7,   # forward in new heading
        0.0,
        0.0,
        0.7,   # maybe wall on +90° side now
        0.0,
    ]

    print("Updating SLAM with second pose and ranges...")
    slam.update(pose2, ranges2)
    slam.debgug_print_map()

    print("Done. '.' = free-ish, '#' = occupied-ish, ' ' = unknown.")


if __name__ == "__main__":
    main()
