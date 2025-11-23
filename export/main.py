import pandas as pd
import matplotlib.pyplot as plt
import time
import os

MAP_FILE = r"C:\Users\padav\Desktop\turtlebot_mapping\export\world_map_live.csv"
FRONTIER_FILE = r"C:\Users\padav\Desktop\turtlebot_mapping\export\frontiers_live.csv"

plt.ion()                   # interactive mode
fig, ax = plt.subplots(figsize=(6, 6))

last_map_mtime = 0
last_frontier_mtime = 0


while True:
    updated = False

    # --- Check world map file ---
    try:
        map_mtime = os.path.getmtime(MAP_FILE)
        if map_mtime != last_map_mtime:
            last_map_mtime = map_mtime
            df_map = pd.read_csv(MAP_FILE)
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
        ax.clear()

        # ----- Plot map -----
        if 'df_map' in locals():
            walls = df_map[df_map["is_wall"] == 1]
            free  = df_map[df_map["is_wall"] == 0]

            ax.scatter(walls["x"], walls["y"], s=20, color="red", label="Wall")
            ax.scatter(free["x"],  free["y"],  s=20, color="blue", label="Free")

        # ----- Plot frontiers -----
        if 'df_front' in locals() and len(df_front) > 0:
            for _, row in df_front.iterrows():
                # segment
                ax.plot([row.ax, row.bx], [row.ay, row.by],
                        color="green", linewidth=2)
                # midpoint
                ax.scatter(row.mx, row.my, s=40, color="green", marker="x")

        # ----- Formatting -----
        ax.set_xlabel("x [m]")
        ax.set_ylabel("y [m]")
        ax.set_title("Live Map + Frontiers")
        ax.grid(True)
        ax.legend()
        ax.axis("equal")

        plt.draw()

    plt.pause(0.1)   # update at 10 Hz
