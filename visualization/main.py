import os

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
from pathlib import Path

# --- paths to csv data --- 
MAP_FILE = "export/world_map_live.csv"
FRONTIER_FILE = "export/frontiers_live.csv"
POSE_FILE = "export/pose_live.csv"

# --- grid configuration ---
# number of cells per axis
GRID_SIZE = 400   # 400 x 400 cells 

UNKNOWN = 0     # grey
FREE = 1        # white
OCCUPIED = 2    # red

# --- define colors in colormap ---
cmap = ListedColormap(["#808080", "#FFFFFF", "#FF0000"])


# --- update world bounds and cell scaling ---
def update_world_parameters(df_map):
    # 10% margin around the data
    padding = 0.1

    xmin = df_map["x"].min()
    xmax = df_map["x"].max()
    ymin = df_map["y"].min()
    ymax = df_map["y"].max()

    dx = xmax - xmin
    dy = ymax - ymin

    xmin -= dx * padding
    xmax += dx * padding
    ymin -= dy * padding
    ymax += dy * padding

    return xmin, xmax, ymin, ymax

# --- world coordinates to grid ---

def world_to_grid(x, y, bounds, cells_per_meter):
    # convert world coordinates (meters) to grid indices (row, col)

    xmin, xmax, ymin, ymax = bounds

    col = int((x - xmin) * cells_per_meter)
    row = int((y - ymin) * cells_per_meter)
    return row, col



plt.ion()                   # interactive mode
fig, ax = plt.subplots(figsize=(8, 8))

last_map_mtime = 0
last_frontier_mtime = 0
last_pose_mtime = 0

df_pose = None

# --- grid completly unknown (initialized as grey) ---
grid = np.full((GRID_SIZE, GRID_SIZE), UNKNOWN, dtype=np.uint8)
# default bounds
bounds = (-2.0, 2.0, -2.0, 2.0) 
cells_per_meter = GRID_SIZE / (bounds[1] - bounds[0])

try:
    while plt.fignum_exists(fig.number):
        updated = False

        # --- Check world map file ---
        try:
            map_mtime = os.path.getmtime(MAP_FILE)
            if map_mtime != last_map_mtime:
                last_map_mtime = map_mtime
                df_map = pd.read_csv(MAP_FILE)
                # clalculate new world extent
                bounds = update_world_parameters(df_map)
                world_width = bounds[1] - bounds[0]
                world_height = bounds[3] - bounds[2]

                scale_x = GRID_SIZE / world_width
                scale_y = GRID_SIZE / world_height

                cells_per_meter = min(scale_x, scale_y)

                updated = True
        except Exception as e:
            print("Error reading map CSV:", e)

        # --- check frontier file ---
        try:
            frontier_mtime = os.path.getmtime(FRONTIER_FILE)
            if frontier_mtime != last_frontier_mtime:
                last_frontier_mtime = frontier_mtime
                df_front = pd.read_csv(FRONTIER_FILE)
                updated = True
        except Exception as e:
            print("Error reading frontier CSV:", e)

        # --- check pose file
        try:
            pose_mtime = os.path.getmtime(POSE_FILE)
            if pose_mtime != last_pose_mtime:
                last_pose_mtime = pose_mtime
                df_pose = pd.read_csv(POSE_FILE)
                updated = True
        except Exception as e:
            print("Error reading pose CSV:", e)

        # --- If either file updated -> redraw ---
        if updated:
            # set everything to UNKNOWN
            grid[:] = UNKNOWN
            # ----- plot map -----
            if 'df_map' in locals():

                # --- filter walls and free spaces 
                walls = df_map[df_map["is_wall"] == 1]
                free  = df_map[df_map["is_wall"] == 0]

                # --- add walls to grid ---
                for _, row in walls.iterrows():
                    r, c = world_to_grid(row["x"], row["y"], bounds, cells_per_meter)
                    grid[r, c] = OCCUPIED
                
                # --- add free cells to grid ---
                for _, row in free.iterrows():
                    r, c = world_to_grid(row["x"], row["y"], bounds, cells_per_meter)
                    grid[r, c] = FREE

                # --- update plot ---
            
            # --- plot occupancy grid ---
            ax.clear()

            xmin, xmax, ymin, ymax = bounds

            im = ax.imshow(
                grid,
                cmap=cmap,
                origin="lower",
                extent=[xmin, xmax, ymin, ymax],
                interpolation="nearest"
            )
                
            # ----- Plot frontiers -----

            if 'df_front' in locals() and len(df_front) > 0:
                for _, row in df_front.iterrows():
                    # segment
                    ax.plot([row.ax, row.bx], 
                            [row.ay, row.by],
                            color="green", 
                            linewidth=1)
                    # midpoint
                    ax.scatter(row.mx, 
                            row.my, 
                            s=10, 
                            color="green", 
                            marker="x")
                    
            # --- Plot robot path & pose ---
            if df_pose is not None and len(df_pose) > 0:
                # path: plot all old poses as line
                if {"x", "y"}.issubset(df_pose.columns):
                    ax.plot(
                        df_pose["x"],
                        df_pose["y"],
                        linestyle="-",
                        linewidth=0.7,
                        color="blue",
                        label="Robot path"
                    )

                    # last pose
                    x = df_pose["x"].iloc[-1]
                    y = df_pose["y"].iloc[-1]

                    # --- orientation theta ---
                    theta_col = None
                    for cand in ["theta", "yaw", "heading"]:
                        if cand in df_pose.columns:
                            theta_col = cand
                            break

                 
                    theta = df_pose[theta_col].iloc[-1]
                    # arrow
                    arrow_len = 0.25
                    dx = np.cos(theta) * arrow_len
                    dy = np.sin(theta) * arrow_len
                    ax.quiver(
                        x,
                        y,
                        dx,
                        dy,
                        angles = "xy",
                        scale_units = "xy",
                        scale = 1.0,
                        width = 0.01,
                        color = "purple",
                        label = "Robot pose"
                    )
        




            # --- apply plot formatting ---
            
            ax.set_xlabel("x [m]")
            ax.set_ylabel("y [m]")
            ax.set_title("Map + Frontiers + Pose")
            ax.set_aspect("equal", adjustable="box")
            
            plt.tight_layout()
            plt.draw()

        plt.pause(0.1)   # update at 10 Hz

finally:
    plt.ioff()
    plt.close('all')
