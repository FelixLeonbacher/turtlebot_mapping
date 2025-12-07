#include "threads.hpp"

#include <iostream>
#include <string>
#include <exception>

#include "sharedMemory.hpp"
#include "connection/connection.hpp"    // TCP reader (readTaggedMessage)
#include "core/geometry.hpp"            // MapSet, Pose2D, LidarScan, Frontier
#include "core/parser.hpp"              // parseLidarScanFromMsg(), scan_to_data()


void sensor_thread()
{
    const std::string ip        = "192.168.100.54";
    const int         port_scan = 9997;  // LiDAR
    const int         port_odom = 9998;  // Odom

    const int lidar_max = 2048;  // maximale Speicherkapazität für LiDar Scan



    std::cout << "[Sensor] Thread started. Listening for LiDAR + ODOM...\n" << std::endl;


     // ============================
    // MAIN STREAM LOOP
    // ============================
    while(true) {

        // read odom and lidar message from the network
        std::string msg_lidar;
        std::string msg_odom;

        try {
            msg_lidar = connection::readTaggedMessage(ip, port_scan);
            msg_odom = connection::readTaggedMessage(ip, port_odom);
        }

        catch (const std::exception& e) {
            std::cerr << "[Sensor][ERROR] Failed to read from network: " << e.what() << "\n" << std::endl;
            break;
        }


        // 2) Parse JSON → LidarScan
        core::LidarScan scan;
        try {
            scan = parseLidarScanFromMsg(msg_lidar);
        }
        catch (const std::exception& e) {
            std::cerr << "[Sensor][Error] Failed to parse Lidar scan: " << e.what() << "\n";
            continue;
        }

        std::cout << "[INFO] Scan has " << scan.ranges.size() << " ranges\n";


        // Parse ODOM message into current pose
        core::Pose2D pose{0.0f, 0.0f, 0.0f};
        if (!parseOdomToPose2D(msg_odom, pose)) {
            std::cerr << "[Sensor][ERROR] Failed to parse ODOM pose\n";
            continue;    // skip faulty odom
        }

        
        // Anzahl Werte im Scan
        std::size_t count_scan = scan.ranges.size();
        std::size_t max_size = static_cast<std::size_t>(lidar_max);


        // Falls mehr Werte als im max_size. Werte abschneiden
        if (count_scan> max_size) {
            std::cerr << "[Sensor][Warn] LiDAR scan has " << count_scan << " ranges, but buffer only has " << max_size << ". Truncating.\n";
            count_scan = max_size;
        }


        // Write pose + LiDAR ranges into shared memory
        if (g_shm != nullptr) {
            // semaphore einbetten TODO 


            //Copy current pose
            g_shm->current_pose.x = pose.x;
            g_shm->current_pose.y = pose.y;
            g_shm->current_pose.theta = pose.theta;

            //Copy lidar scan  
            for (std::size_t i = 0; i < count_scan; ++i) {
                g_shm->lidar_scan[i] = scan.ranges[i];
            }

            g_shm->lidar_count = static_cast<int>(count_scan);

            //Set flags to indicate thet new data is available
            g_shm->pose_valid = 1;
            g_shm->scan_valid = 1;
        }

    }
}