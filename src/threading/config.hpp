#pragma once

#include <string>

// Configuration for mapping parameters
struct MappingConfig {
    // Threshold for detecting jumps / frontiers in LiDAR Data
    double jump_threshold = 0.1;
    // Resolution of the world map grid [m / cell]
    double map_resolution = 0.05;
};

/// \brief Configuration fpr TCP/IP connection parameters
struct ConnectionConfig {
    std::string ip;         //< IP adress of the robot
    int port_scan = 0;      //< TCP port for LiDAR scans
    int port_odom = 0;      //< TCP port for odometry data
    int port_cmd = 0;       //< TCP port for velocity commands
};

/// \brief Configuration for CSV export paths
struct ExportConfig {
    std::string export_path_world;          //< Path for grid export
    std::string export_path_frontiers;      //< Path for frontier export
    std::string export_path_pose;           //< Path for robot pose export
};

/// \brief Configuration for the entire application
struct Config {
    MappingConfig mapping;
    ConnectionConfig connection;
    ExportConfig export_cfg;
};

/// \brief Load configuration from a JSON file.
///
/// \param filename Path to the JSON config file.
/// \param cfg      Output parameter that will be filled with the parsed config.
/// \return true on success, false on error (messages printed to std::cerr).
bool loadConfig(const std::string& filename, Config& cfg);