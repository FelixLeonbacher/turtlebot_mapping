#include "threads.hpp"

#include <iostream>
#include <iomanip>
#include <vector>
#include <stdexcept>
#include <thread>
#include <chrono>
#include <cmath>

#include "sharedMemory.hpp"
#include "core/geometry.hpp"            // MapSet, Pose2D, LidarScan, Frontier
#include "mapping/mapping.hpp"          // mapping::data_to_world()
#include "export/export.hpp"            // export_world_map_csv()

#include "config.hpp"
extern Config g_config;


void mapping_thread()
{
    std::cout << std::fixed << std::setprecision(3);

    // Mapping paramaters from configuration
    const float jump_thresh = g_config.mapping.jump_threshold;     // frontier detection threshold
    const float map_res     = CORE_ROUND_RES;  // grid resolution from geometry.hpp
    const int max_scans   = -1;  // -1:: process scans indefinitely



    // World representation and frontier set (in world coordinates)
    mapping::MapSet world_map(0, mapping::Point2DHash(map_res), mapping::Point2DEq(map_res));
    std::vector<core::Frontier> all_frontiers;

    std::cout << "[Mapping] Thread started. \n";

    int scan_idx = 0;

    // Main mapping loop: wait for new scans, update world map and export CSVs
    while(max_scans < 0 || scan_idx < max_scans){

        // check stop flag
        if (is_stop_requested()) {
            std::cout << "[Mapping] Stop requested, exiting.\n";
            break;
        }

        // Wait until sensor thread signals a new LiDAR scan
        g_scan_sem.acquire();

        // check stop flag
        if (is_stop_requested()) {
            std::cout << "[Mapping] Stop requested, exiting.\n";
            break;
        }

        core::LidarScan scan;
        bool goal_reached_flag = false;

        {
            // protect with mutex
            std::lock_guard<std::mutex> lock(g_shm_mutex);

            // Pose at which the LiDAR scan was taken
            scan.pose.x = g_shm->current_pose.x;
            scan.pose.y = g_shm->current_pose.y;
            scan.pose.theta = g_shm->current_pose.theta;

            // LiDAR scan meta information
            scan.angle_min = g_shm->lidar_angle_min;
            scan.angle_inc = g_shm->lidar_angle_inc;
            scan.range_min = g_shm->lidar_range_min;
            scan.range_max = g_shm->lidar_range_max;

            // Copy ranges from shared memory to local vector
            int count = g_shm->lidar_count;
            scan.ranges.resize(static_cast<std::size_t>(count));
            for (int i = 0; i < count; ++i) {
                scan.ranges[i] = g_shm->lidar_scan[i];
            }

            // Mark scan as consumed
            g_shm->scan_valid = 0;

            goal_reached_flag = (g_shm->goal_reached != 0);


        }

        ++scan_idx;
        
        // Convert LiDAR scan to internal scan data + detect frontiers
        core::ScanData scan_data;
        std::vector<core::Frontier> scan_frontiers;

        mapping::scan_to_data(scan, scan_data, scan_frontiers, jump_thresh);


        // Polish detected frontiers (remove too small ones)
        mapping::polish_frontiers(scan_frontiers, g_config.mapping.min_frontier_width);

        // Integrate new scan into global world map representation
        if (goal_reached_flag) {
            mapping::data_to_world(scan_data, scan_frontiers, world_map, all_frontiers, map_res);
        }
        // Export world map, frontiers and current pose to CSV
        export_utils::export_world_map_csv(world_map, g_config.export_cfg.export_path_world);
        export_utils::export_frontiers_csv(all_frontiers, g_config.export_cfg.export_path_frontiers);
        export_utils::export_pose_csv(scan.pose, g_config.export_cfg.export_path_pose);
    }
}