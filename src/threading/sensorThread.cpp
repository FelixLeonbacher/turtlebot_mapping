/**
 * @author Merle Rehpeen
 * @version 1.0
 */

#include "threads.hpp"

#include <iostream>
#include <string>
#include <exception>

#include "sharedMemory.hpp"
#include "connection/connection.hpp"    // TCP reader (readTaggedMessage)
#include "core/geometry.hpp"            // MapSet, Pose2D, LidarScan, Frontier
#include "core/parser.hpp"              // parseLidarScanFromMsg(), scan_to_data()
#include "config.hpp"

extern Config g_config;


void sensor_thread()
{
    const std::string ip        = g_config.connection.ip;
    const int         port_scan = g_config.connection.port_scan;  // LiDAR
    const int         port_odom = g_config.connection.port_odom;  // Odom



    std::cout << "[Sensor] Thread started. Listening for LiDAR + ODOM...\n" << std::endl;


    // Main loop: read LiDAR + ODOM from TCP and write into shared memory
    while(true) {

        // check stop flag
        if (is_stop_requested()) {
            std::cout << "[Sensor] Stop requested, exiting.\n";
            break;
        }

        std::string msg_lidar;
        std::string msg_odom;

        // read odom and lidar message from the network
        try {
            msg_lidar = connection::readTaggedMessage(ip, port_scan);
            msg_odom = connection::readTaggedMessage(ip, port_odom);
        }

        catch (const std::exception& e) {
            std::cerr << "[Sensor][ERROR] Failed to read from network: " << e.what() << "\n" << std::endl;
            request_global_stop();
            break;
        }


        // parse LiDAR scan
        core::LidarScan scan;
        try {
            scan = parseLidarScanFromMsg(msg_lidar);
        }
        catch (const std::exception& e) {
            std::cerr << "[Sensor][Error] Failed to parse Lidar scan: " << e.what() << "\n";
            continue;
        }

        // Parse ODOM message into current pose
        core::Pose2D pose{0.0f, 0.0f, 0.0f};
        if (!parseOdomToPose2D(msg_odom, pose)) {
            std::cerr << "[Sensor][ERROR] Failed to parse ODOM pose\n";
            continue;    // skip faulty odom
        }

        
        // Limit number of ranges to our fixed shared memory buffer
        std::size_t count_scan = scan.ranges.size();
        std::size_t max_size = static_cast<std::size_t>(lidar_max);

        if (count_scan> max_size) {
            std::cerr << "[Sensor][Warn] LiDAR scan has " << count_scan << " ranges, but buffer only has " << max_size << ". Truncating.\n";
            count_scan = max_size;
        }


        // Write pose + LiDAR ranges into shared memory
        if (g_shm != nullptr) {
            
            std::lock_guard<std::mutex> lock(g_shm_mutex);  

            //Copy current pose
            g_shm->current_pose.x = pose.x;
            g_shm->current_pose.y = pose.y;
            g_shm->current_pose.theta = pose.theta;

            //Copy LiDAR scan  
            for (std::size_t i = 0; i < count_scan; ++i) {
                g_shm->lidar_scan[i] = scan.ranges[i];
            }

            g_shm->lidar_count = static_cast<int>(count_scan);

            g_shm->lidar_angle_min = scan.angle_min;
            g_shm->lidar_angle_inc = scan.angle_inc;
            g_shm->lidar_range_min = scan.range_min;
            g_shm->lidar_range_max = scan.range_max;


            //Set flags to indicate thet new data is available
            g_shm->pose_valid = 1;
            g_shm->scan_valid = 1;
        }

        // Notify mapping thread that a new scan is available
        g_scan_sem.release();

    }
}