#include <iostream>
#include <string>
#include <exception>

#include "connection/connection.hpp"
#include "threads.hpp"
#include "sharedMemory.hpp"

#include "config.hpp"
extern Config g_config;


void network_shutdown() {
    connection::shutdown();
    std::cout << "[Network] shutdown done\n";
}

void network_thread() {


    const std::string ip        = g_config.connection.ip;
    const int port_lidar        = g_config.connection.port_scan;
    const int port_odom         = g_config.connection.port_odom;

    std::cout << "[MAIN] Starting network checker thread...\n" << std::endl;

    try {
        // Initialize networking
        connection::init();
        std::cout << "[Network][INFO] Winsock initialized.\n";
        
        // Read LiDAR message 
        std::cout << "[INFO] Waiting for LiDAR message on " 
                  << ip << ":" << port_lidar << "...\n";

        std::string lidarMsg = connection::readTaggedMessage(ip, port_lidar);

        // Read Odom message 
        std::cout << "[INFO] Waiting for ODOM message on " 
                  << ip << ":" << port_odom << "...\n";

        std::string odomMsg = connection::readTaggedMessage(ip, port_odom);

        // set Flag in Shared Memory 
        if (g_shm != nullptr) {
            std::lock_guard<std::mutex> lock(g_shm_mutex);
            g_shm->network_ok = 1;
        }

        std::cout << "[Network][INFO] Connection is working.\n";


    }
    
    catch (const std::exception& e) {
        std::cerr << "\n[Network][ERROR] " << e.what() << "\n";

        // set Flag in Shared Memory 
        if (g_shm != nullptr) {
            std::lock_guard<std::mutex> lock(g_shm_mutex);
            g_shm->network_ok = -1;
        }

        request_global_stop();

    }    
}
