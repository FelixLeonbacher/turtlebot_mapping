import time
import os

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap, BoundaryNorm
from pathlib import Path

# --- paths to csv data --- 
MAP_FILE = Path("export") / "world_map_live.csv"
FRONTIER_FILE = Path("export") / "frontiers_live.csv"

# --- grid configuration ---
GRID_W = 800
GRID_H = 800

UNKNOWN = 0     # grey
FREE = 1        # white
OCCUPIED = 2    # red

WORLD_X_MIN = -2.0
WORLD_X_MAX = 2.0
WORLD_Y_MIN = -2.0
WORLD_Y_MAX = 2.0
CELLS_PER_METER = GRID_W / (WORLD_X_MAX - WORLD_X_MIN)



# --- define colours in colormap ---
cmap = ListedColormap(["#808080", "#FFFFFF", "#FF0000"])
norm = BoundaryNorm([-0.5, 0.5, 1.5, 2.5], cmap.N)


def update_world_parameters(df_map):
    global WORLD_X_MIN, WORLD_X_MAX, WORLD_Y_MIN, WORLD_Y_MAX, CELLS_PER_METER
    padding = 0.1

    xmin = df_map["x"].min()
    xmax = df_map["x"].max()
    ymin = df_map["y"].min()
    ymax = df_map["y"].max()

    dx = xmax - xmin
    dy = ymax - ymin

    WORLD_X_MIN = xmin - dx * padding
    WORLD_X_MAX = xmax + dx * padding
    WORLD_Y_MIN = ymin - dy * padding
    WORLD_Y_MAX = ymax + dy * padding

    world_width = WORLD_X_MAX - WORLD_X_MIN
    world_height = WORLD_Y_MAX - WORLD_Y_MIN

    scale_x = GRID_W / world_width
    scale_y = GRID_H / world_height
    CELLS_PER_METER = min(scale_x, scale_y)


# --- world coordinates to grid ---

def world_to_grid(x, y):
    col = int((x - WORLD_X_MIN) * CELLS_PER_METER)
    row = int((y - WORLD_Y_MIN) * CELLS_PER_METER)
    return row, col


plt.ion()                   # interactive mode
fig, ax = plt.subplots(figsize=(6, 6))

last_map_mtime = 0
last_frontier_mtime = 0

# --- initial Grid complettly grey ---
grid = np.full((GRID_H, GRID_W), UNKNOWN, dtype=np.uint8)


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
                update_world_parameters(df_map)
                updated = True
        except Exception as e:
            print("Error reading map CSV:", e)

        # --- Check frontier file ---
        try:
            frontier_mtime = os.path.getmtime(FRONTIER_FILE)
            if frontier_mtime != last_frontier_mtime:
                last_frontier_mtime = frontier_mtime
                df_front = pd.read_csv(FRONTIER_FILE)
                updated = True
        except Exception as e:
            print("Error reading frontier CSV:", e)

        # --- If either file updated -> redraw ---
        if updated:
            # set everything to UNKNOWN
            grid[:] = UNKNOWN
            # ----- Plot map -----
            if 'df_map' in locals():

                # --- filter walls and free spaces 
                walls = df_map[df_map["is_wall"] == 1]
                free  = df_map[df_map["is_wall"] == 0]

                # --- add walls to grid ---
                for _, row in walls.iterrows():
                    r, c = world_to_grid(row["x"], row["y"])
                    grid[r, c] = OCCUPIED
                
                # --- add free cells to grid ---
                for _, row in free.iterrows():
                    r, c = world_to_grid(row["x"], row["y"])
                    grid[r, c] = FREE

                # --- update plot ---
            
            # --- plot grid ---
            ax.clear

            im = ax.imshow(
                grid,
                cmap=cmap,
                norm=norm,
                origin="lower",
                extent=[WORLD_X_MIN, WORLD_X_MAX, WORLD_Y_MIN, WORLD_Y_MAX],
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
                    
            # --- set formatation ---
            
            ax.set_xlabel("x [m]")
            ax.set_ylabel("y [m]")
            ax.set_title("Map + Frontiers")
            ax.set_aspect("equal", adjustable="box")
            
            plt.tight_layout()
            plt.draw()

        plt.pause(0.1)   # update at 10 Hz

finally:
    plt.ioff()
    plt.close('all')
