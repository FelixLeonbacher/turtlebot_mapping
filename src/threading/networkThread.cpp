#include <iostream>
#include <string>
#include <exception>

#include "connection/connection.hpp"
#include "threads.hpp"
#include "sharedMemory.hpp"


void network_shutdown() {
    connection::shutdown();
    std::cout << "[Network] shutdown done\n";
}

void network_thread() {


    const std::string ip        = "192.168.100.54";
    const int port_lidar        = 9997;
    const int port_odom         = 9998;


    try {
        // Initialize networking
        connection::init();
        std::cout << "[Network][INFO] Winsock initialized.\n";
        
        // --- Read LiDAR message ---
        std::cout << "[INFO] Waiting for LiDAR message on " 
                  << ip << ":" << port_lidar << "...\n";

        std::string lidarMsg = connection::readTaggedMessage(ip, port_lidar);

        std::cout << "\n===== LIDAR MESSAGE =====\n";
        std::cout << lidarMsg << "\n";
        std::cout << "==========================\n\n";

        // --- Read Odom message ---
        std::cout << "[INFO] Waiting for ODOM message on " 
                  << ip << ":" << port_odom << "...\n";

        std::string odomMsg = connection::readTaggedMessage(ip, port_odom);

        std::cout << "\n===== ODOM MESSAGE =====\n";
        std::cout << odomMsg << "\n";
        std::cout << "=========================\n\n";
        

        // --- set Flag in Shared Memory ---
        if (g_shm != nullptr) {
            g_shm->network_ok = 1;
        }

        std::cout << "[Network][INFO] Connection is working.\n";


    }
    
    catch (const std::exception& e) {
        std::cerr << "\n[Network][ERROR] " << e.what() << "\n";

        // --- set Flag in Shared Memory ---
        if (g_shm != nullptr) {
            g_shm->network_ok = -1;
        }
    }    
}
