import os
from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
from matplotlib.patches import Patch
from matplotlib.lines import Line2D

# ============================================================
# Configuration / Constants
# ============================================================

EXPORT_DIR = Path("export")

MAP_FILE = EXPORT_DIR / "world_map_live.csv"
FRONTIER_FILE = EXPORT_DIR / "frontiers_live.csv"
POSE_FILE = EXPORT_DIR / "pose_live.csv"

GRID_SIZE = 300  # 300 x 300 cells

# Cell states
UNKNOWN = 0
FREE = 1
OCCUPIED = 2

# Plot / update parameters
PADDING = 0.10       # 10% margin around the map bounds
ARROW_LEN = 0.25     # robot pose arrow length in meters
UPDATE_DT = 0.10     # seconds (0.10s -> 10 Hz)

# Colormap (UNKNOWN, FREE, OCCUPIED)
cmap = ListedColormap(["#808080", "#FF0000", "#FF0000"])


# ============================================================
# Helper functions
# ============================================================

def update_world_bounds(df_map: pd.DataFrame, padding: float = PADDING):
    """
    Compute world bounds from map points + padding.
    Returns: (x_min, x_max, y_min, y_max)
    """
    x_min = float(df_map["x"].min())
    x_max = float(df_map["x"].max())
    y_min = float(df_map["y"].min())
    y_max = float(df_map["y"].max())

    x_range = max(x_max - x_min, 1e-6)  # guard against 0 range
    y_range = max(y_max - y_min, 1e-6)

    x_min -= x_range * padding
    x_max += x_range * padding
    y_min -= y_range * padding
    y_max += y_range * padding

    return x_min, x_max, y_min, y_max


def points_to_grid_indices(
    x: np.ndarray,
    y: np.ndarray,
    bounds: tuple[float, float, float, float],
    cells_per_meter: float,
    grid_size: int = GRID_SIZE,):
    """
    Vectorized mapping: world points (x, y) -> grid indices (rows, cols).
    Points outside the grid are filtered to avoid IndexError.
    """
    x_min, x_max, y_min, y_max = bounds

    cols = np.floor((x - x_min) * cells_per_meter).astype(int)
    rows = np.floor((y - y_min) * cells_per_meter).astype(int)

    valid = (rows >= 0) & (rows < grid_size) & (cols >= 0) & (cols < grid_size)
    return rows[valid], cols[valid]


def read_csv_if_updated(path: Path, last_mtime: float):
    """
    Read a CSV only if it changed (mtime).
    Returns: (df or None, new_mtime, updated_flag)
    """
    try:
        mtime = path.stat().st_mtime
    except FileNotFoundError:
        return None, last_mtime, False

    if mtime == last_mtime:
        return None, last_mtime, False

    try:
        df = pd.read_csv(path)
        return df, mtime, True
    except Exception as exc:
        print(f"Error reading {path}: {exc}")
        return None, last_mtime, False


def build_legend_handles():
    """Create legend handles so the legend stays consistent."""
    return [
        Patch(facecolor="#808080", label="Unknown"),
        #Patch(facecolor="#FFFFFF", edgecolor="black", label="Free"),
        Patch(facecolor="#FF0000", label="Occupied"),
        Line2D([0], [0], color="green", lw=1, label="Frontiers"),
        Line2D([0], [0], color="blue", lw=1, label="Robot path"),
        Line2D([0], [0], color="purple", lw=2, label="Robot pose"),
    ]


# ============================================================
# Main
# ============================================================

def main() -> None:
    plt.ion()  # interactive mode
    fig, ax = plt.subplots(figsize=(8, 8))

    # mtimes for live reload
    last_map_mtime = 0.0
    last_frontier_mtime = 0.0
    last_pose_mtime = 0.0

    df_map = None
    df_front = None
    df_pose = None

    # Start with a fully UNKNOWN grid
    grid = np.full((GRID_SIZE, GRID_SIZE), UNKNOWN, dtype=np.uint8)

    # Default bounds
    bounds = (-2.0, 2.0, -2.0, 2.0)
    cells_per_meter = GRID_SIZE / (bounds[1] - bounds[0])

    legend_handles = build_legend_handles()

    try:
        while plt.fignum_exists(fig.number):
            updated = False

            # ----------------------------
            # Map CSV
            # ----------------------------
            new_map, last_map_mtime, map_updated = read_csv_if_updated(MAP_FILE, last_map_mtime)
            if map_updated and new_map is not None:
                df_map = new_map

                # Recompute bounds from map data
                bounds = update_world_bounds(df_map)

                x_min, x_max, y_min, y_max = bounds
                world_w = x_max - x_min
                world_h = y_max - y_min

                # Choose cells_per_meter so everything fits into GRID_SIZE
                cells_per_meter = min(GRID_SIZE / world_w, GRID_SIZE / world_h)
                updated = True

            # ----------------------------
            # Frontier CSV
            # ----------------------------
            new_front, last_frontier_mtime, front_updated = read_csv_if_updated(FRONTIER_FILE, last_frontier_mtime)
            if front_updated and new_front is not None:
                df_front = new_front
                updated = True

            # ----------------------------
            # Pose CSV
            # ----------------------------
            new_pose, last_pose_mtime, pose_updated = read_csv_if_updated(POSE_FILE, last_pose_mtime)
            if pose_updated and new_pose is not None:
                df_pose = new_pose
                updated = True

            # ----------------------------
            # Redraw if anything changed
            # ----------------------------
            if updated:
                # Reset the grid
                grid[:] = UNKNOWN

                # ----- Write map data into the grid -----
                if df_map is not None and {"x", "y", "is_wall"}.issubset(df_map.columns):
                    walls = df_map[df_map["is_wall"] == 1]
                    free = df_map[df_map["is_wall"] == 0]

                    if len(walls) > 0:
                        wall_r, wall_c = points_to_grid_indices(
                            walls["x"].to_numpy(),
                            walls["y"].to_numpy(),
                            bounds,
                            cells_per_meter,
                        )
                        grid[wall_r, wall_c] = OCCUPIED

                    if len(free) > 0:
                        free_r, free_c = points_to_grid_indices(
                            free["x"].to_numpy(),
                            free["y"].to_numpy(),
                            bounds,
                            cells_per_meter,
                        )
                        
                # ----- Plot -----
                ax.clear()
                x_min, x_max, y_min, y_max = bounds

                ax.imshow(
                    grid,
                    cmap=cmap,
                    origin="lower",
                    extent=[x_min, x_max, y_min, y_max],
                    interpolation="nearest",
                )

                # ----- Draw frontiers -----
                if df_front is not None and len(df_front) > 0 and {"ax", "ay", "bx", "by", "mx", "my"}.issubset(df_front.columns):
                    for _, rec in df_front.iterrows():
                        ax.plot([rec.ax, rec.bx], [rec.ay, rec.by], color="green", linewidth=1)
                        ax.scatter(rec.mx, rec.my, s=10, color="green", marker="x")

                # ----- Draw robot path + pose -----
                if df_pose is not None and len(df_pose) > 0 and {"x", "y", "theta"}.issubset(df_pose.columns):
                    ax.plot(
                        df_pose["x"],
                        df_pose["y"],
                        linestyle="-",
                        linewidth=0.7,
                        color="blue",
                    )

                    x_last = float(df_pose["x"].iloc[-1])
                    y_last = float(df_pose["y"].iloc[-1])
                    theta = float(df_pose["theta"].iloc[-1])

                    dx = np.cos(theta) * ARROW_LEN
                    dy = np.sin(theta) * ARROW_LEN

                    ax.quiver(
                        x_last,
                        y_last,
                        dx,
                        dy,
                        angles="xy",
                        scale_units="xy",
                        scale=1.0,
                        width=0.008,
                        color="purple",
                    )

                # ----- Legend -----
                ax.legend(handles=legend_handles, loc="upper right", framealpha=0.9)

                # ----- Formatting -----
                ax.set_xlabel("x [m]")
                ax.set_ylabel("y [m]")
                ax.set_title("Map + Frontiers + Pose")
                ax.set_aspect("equal", adjustable="box")

                plt.tight_layout()
                plt.draw()

            plt.pause(UPDATE_DT)

    finally:
        plt.ioff()
        plt.close("all")


if __name__ == "__main__":
    main()
