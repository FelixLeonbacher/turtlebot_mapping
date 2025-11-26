#include <iostream>
#include <string>
#include "connection/connection.hpp"

int main()
{
    const std::string ip = "192.168.100.54";
    const int port_lidar = 9997;
    const int port_odom  = 9998;

    try {
        // Initialize networking
        connection::init();
        std::cout << "[INFO] Winsock initialized.\n";

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

        // Cleanup networking
        connection::shutdown();
        std::cout << "[INFO] Winsock cleaned up.\n";
    }
    catch (const std::exception& e) {
        std::cerr << "\n[ERROR] " << e.what() << "\n";
        return 1;
    }

    return 0;
}
