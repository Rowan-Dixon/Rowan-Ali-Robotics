from matplotlib import pyplot as plt
import numpy as np

def plot_map(prob_map, grid, path=None, point=None):
    """
    prob_map : 2D list/array of probabilities
    grid     : slam_map.grid (must have .resolution and map size)
    path     : list of world-coordinate waypoints / single goal point (optional)
    """

    arr = np.array(prob_map)
    H, W = arr.shape
    res = grid.resolution  # meters per cell

    # Put robot at (0,0) → map centre in world coords
    half_w = (W * res) / 2.0
    half_h = (H * res) / 2.0
    extent = [-half_w, half_w,   # x from -half_w to +half_w
              -half_h, half_h]   # y from -half_h to +half_h

    fig, ax = plt.subplots(figsize=(6, 6))
    ax.imshow(arr, cmap='Greys', origin='lower', extent=extent)

    # Optional: draw path which is already in world coords (meters)
    if path:
        xs = [p[0] for p in path]
        ys = [p[1] for p in path]
        ax.plot(xs, ys, 'r-', linewidth=2)
        ax.scatter(xs[0], ys[0], c='green', s=40)
        ax.scatter(xs[-1], ys[-1], c='blue', s=40)
    if point:
        ax.scatter(point[0], point[1], c='red', s=40)

    ax.set_xlabel("x (meters)")
    ax.set_ylabel("y (meters)")
    ax.set_title("SLAM Map (World Coordinates)")

    # Optional: ticks every 0.25 m (your Unity square size)
    tick_step = 0.25
    ax.set_xticks(np.arange(-half_w, half_w + tick_step, tick_step))
    ax.set_yticks(np.arange(-half_h, half_h + tick_step, tick_step))

    ax.grid(True, linestyle=':', linewidth=0.5)
    plt.tight_layout()
    plt.show()
