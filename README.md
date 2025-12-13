# TurtleBot Mapping & Control Framework

A **multi-threaded C++ robot control and mapping framework** for a TurtleBot platform.  
The system performs **LiDAR-based mapping with frontier detection**, **goal-driven motion control**, **TCP-based sensor/command communication**, and **live visualization via CSV export**.

---

## Overview

This project implements a complete perception–control pipeline:

- TCP-based network communication (LiDAR, Odometry, Control)
- Multi-threading with shared memory (Windows)
- 2D LiDAR mapping with frontier detection
- Goal-based linear motion control
- CSV-based live data export
- Python-based real-time visualization

The architecture is modular and thread-based, allowing perception, mapping, and control to run independently but synchronized.

---

## System Architecture

The application consists of the following main threads:

| Thread | Responsibility |
|------|---------------|
| **Network Thread** | Checks TCP connectivity to LiDAR and Odometry sources |
| **Sensor Thread** | Receives LiDAR scans and odometry data |
| **Mapping Thread** | Builds a global map and detects frontiers |
| **Goal Thread** | Reads user-defined goals from stdin |
| **Controller Thread** | Drives the robot to the goal using a linear controller |

All threads communicate through **shared memory and semaphores**.

---

## Thread Flow

1. **Network thread** verifies LiDAR & ODOM connections  
2. **Sensor thread** reads LiDAR scans and odometry and writes them into shared memory  
3. **Mapping thread**
   - Converts scans into world coordinates
   - Detects frontiers using jump detection
   - Maintains a global map
   - Exports CSV files  
4. **Goal thread** accepts user goals `(x y theta)` via terminal input  
5. **Controller thread** computes `(v, ω)` commands and sends them to the robot  

---

## Mapping & Frontier Detection

- LiDAR scans are converted into **world coordinates**
- Points are **rounded to a configurable grid resolution**
- **Frontiers** are detected when:
  - Range jumps exceed a threshold
  - Validity of consecutive LiDAR beams changes
- Small frontiers are filtered out
- Frontiers are removed automatically once their endpoints are observed as walls

Core functions:
- `scan_to_data()`
- `data_to_world()`
- `polish_frontiers()`

---

## Control

The controller is a **linear pose controller**:

v = k_roh · ρ  
ω = k_alpha · α + k_beta · β  

Features:
- Velocity and angular velocity saturation
- Goal reached detection via distance threshold
- Immediate interruption when a new goal is issued

---

## Configuration

The system is configured via a JSON file loaded at startup.

---

## CSV Export & Visualization

The mapping thread continuously exports:

| File | Content |
|----|--------|
| `world_map_live.csv` | Occupied / free map points |
| `frontiers_live.csv` | Frontier line segments |
| `pose_live.csv` | Robot pose history |

---

## Build & Run

mkdir build  
cd build  
cmake ..  
cmake --build .  

---

## Authors

- **Mapping & Architecture**: Felix Leonbacher  
- **Controller**: Rainhard Wipp  
- **Network & Parsing**: Philipp Riegler, Victor Swekis  
- **System Integration**: Merle Rehpeen  

---

## License

Educational / research use only.  
No warranty. Use at your own risk.
