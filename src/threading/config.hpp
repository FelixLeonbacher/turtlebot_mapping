#pragma once

#include <string>

struct AppConfig {
    double jump_threshold = 0.1;
    double map_resolution = 0.05;

    // connection
    std::string ip;
    int port_scan = 0;
    int port_odom = 0;
    int port_cmd = 0;

    // export 
    std::string export_path_world;
    std::string export_path_frontiers;
};

bool load_config(const std::string& filename, AppConfig& cfg);