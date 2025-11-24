#include <iostream>
#include <iomanip>
#include <vector>
#include <stdexcept>

#include "connection/connection.hpp"    // TCP reader (readTaggedMessage)
#include "core/geometry.hpp"            // MapSet, Pose2D, LidarScan, Frontier
#include "core/parser.hpp"              // parseLidarScanFromMsg(), scan_to_data()
#include "mapping/mapping.hpp"          // mapping::data_to_world()
#include "export/export.hpp"            // export_world_map_csv()

using namespace core;

int main()
{
    std::cout << std::fixed << std::setprecision(3);

    // ============================
    // GLOBAL PARAMETERS
    // ============================
    const float jump_thresh = 0.1f;      // frontier detection threshold
    const float map_res     = CORE_ROUND_RES;  // grid resolution from geometry.hpp

    const std::string ip = "192.168.100.54";
    const int port_lidar  = 9997;
    const int port_odom   = 9998;
    const int max_scans   = 100;  // read 50 scans, or set to -1 for infinite

    // ============================
    // INITIALIZE NETWORK
    // ============================
    try {
        connection::init();
        std::cout << "[INFO] Winsock initialized\n";
    }
    catch (const std::exception& e) {
        std::cerr << "[ERROR] Winsock init failed: " << e.what() << "\n";
        return 1;
    }

    // ============================
    // CREATE WORLD & FRONTIER SET & CURRENT POSE
    // ============================
    MapSet world_map(0, Point2DHash(map_res), Point2DEq(map_res));
    std::vector<Frontier> all_frontiers;
    core::Pose2D current_pose{0.0f, 0.0f, 0.0f};

    // ============================
    // MAIN STREAM LOOP
    // ============================
    for (int i = 0; i < max_scans || max_scans < 0; i++) {

        std::cout << "\n===============================================\n";
        std::cout << "Reading LiDAR scan " << (i+1) << "...\n";

        // 1) Receive tagged message
        std::string msg_lidar;
        std::string msg_odom;
        try {
            msg_lidar = connection::readTaggedMessage(ip, port_lidar);
            msg_odom  = connection::readTaggedMessage(ip, port_odom);
        }
        catch (const std::exception& e) {
            std::cerr << "[ERROR] Failed reading from LiDAR: " << e.what() << "\n";
            break;
        }

        // 2) Parse JSON → LidarScan
        LidarScan scan;
        try {
            scan = parseLidarScanFromMsg(msg_lidar);
        }
        catch (const std::exception& e) {
            std::cerr << "[ERROR] Failed to parse LiDAR scan: " << e.what() << "\n";
            continue;    // skip faulty scan
        }


        std::cout << "[INFO] Scan has " << scan.ranges.size() << " ranges\n";


        
        // 3) Convert scan → ScanData + frontiers
        ScanData scan_data;
        std::vector<Frontier> scan_frontiers;

        // update current pose from odom
        if (!parseOdomToPose2D(msg_odom, current_pose)) {
            std::cerr << "[ERROR] Failed to parse ODOM pose\n";
            continue;    // skip faulty odom
        }

        scan.pose = current_pose;

        scan_to_data(scan, scan_data, scan_frontiers, jump_thresh);

        std::cout << "[INFO] Extracted " << scan_frontiers.size() 
                  << " frontiers from scan\n";

        

        std::cout << "[INFO] Current pose: x=" << current_pose.x
                  << " y=" << current_pose.y
                  << " theta=" << current_pose.theta << "\n";


        // 4) Insert scan data into world map
        mapping::data_to_world(scan_data, scan_frontiers, world_map, all_frontiers, map_res);

        std::cout << "[INFO] World has now " << world_map.size() << " cells\n";

        // 5) Export after each scan (optional)
        export_utils::export_world_map_csv(world_map, "../export/world_map_live.csv");
        export_utils::export_frontiers_csv(all_frontiers, "../export/frontiers_live.csv");

        std::cout << "[INFO] Exported updated world + frontiers\n";
    }

    // ============================
    // SHUTDOWN
    // ============================
    connection::shutdown();
    std::cout << "[INFO] Winsock cleaned up, program done.\n";

    return 0;
}
