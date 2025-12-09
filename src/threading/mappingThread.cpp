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

    // ============================
    // GLOBAL PARAMETERS
    // ============================
    const float jump_thresh = g_config.mapping.jump_threshold;      // frontier detection threshold
    const float map_res     = CORE_ROUND_RES;  // grid resolution from geometry.hpp
    const int max_scans   = -1;  // read xx scans, or set to -1 for infinite



    // ============================
    // CREATE WORLD & FRONTIER SET & CURRENT POSE
    // ============================
    mapping::MapSet world_map(0, mapping::Point2DHash(map_res), mapping::Point2DEq(map_res));
    std::vector<core::Frontier> all_frontiers;

    std::cout << "[Mapping] Thread started. \n";

    int scan_idx = 0;



    // ============================
    // MAIN LOOP
    // ============================

    while(max_scans < 0 || scan_idx < max_scans){

        // ============================
        // Load data from shared memory
        // ============================

        // wait for new scan
        g_scan_sem.acquire();

        core::LidarScan scan;
        {
            // protect with mutex
            std::lock_guard<std::mutex> lock(g_shm_mutex);

            // Pose
            scan.pose.x = g_shm->current_pose.x;
            scan.pose.y = g_shm->current_pose.y;
            scan.pose.theta = g_shm->current_pose.theta;

            // Meta-Infos
            scan.angle_min = g_shm->lidar_angle_min;
            scan.angle_inc = g_shm->lidar_angle_inc;
            scan.range_min = g_shm->lidar_range_min;
            scan.range_max = g_shm->lidar_range_max;

            // LiDAR-Werte
            int count = g_shm->lidar_count;
            scan.ranges.resize(static_cast<std::size_t>(count));
            for (int i = 0; i < count; ++i) {
                scan.ranges[i] = g_shm->lidar_scan[i];
            }
            // scan als verbraucht markieren
            g_shm->scan_valid = 0;

        }

        ++scan_idx;
        //std::cout << "\n===============================================\n";
        //std::cout << "[Mapping] Processing LiDAR scan " << scan_idx
        //          << " with " << scan.ranges.size() << " ranges\n";

        
        // ============================
        // scan_to_data: Scan -> ScanData + Frontiers
        // ============================

        core::ScanData scan_data;
        std::vector<core::Frontier> scan_frontiers;

        mapping::scan_to_data(scan, scan_data, scan_frontiers, jump_thresh);

        //std::cout << "[Mapping] Extracted " << scan_frontiers.size() << "frontiers from scan\n";

        //std::cout << "[Mapping] Current pose: x=" << scan.pose.x << " y=" << scan.pose.y << scan.pose.theta << "\n";

        // ============================
        // data_to_world: Scan-Daten in Weltkarte
        // ============================


        mapping::data_to_world(scan_data, scan_frontiers, world_map, all_frontiers, map_res);

        //std::cout << "[Mapping] World has now " << world_map.size() << " cells\n";


        // ============================
        // Export as csv
        // ============================

        export_utils::export_world_map_csv(world_map, g_config.export_cfg.export_path_world);
        export_utils::export_frontiers_csv(all_frontiers, g_config.export_cfg.export_path_frontiers);
        export_utils::export_pose_csv(scan.pose, g_config.export_cfg.export_path_pose);

        //std::cout << "[Mapping] Exported updated world + frontiers\n";

    }


}