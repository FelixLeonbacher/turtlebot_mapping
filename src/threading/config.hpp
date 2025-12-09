#pragma once

#include <string>

struct MappingConfig {
    double jump_threshold = 0.1;
    double map_resolution = 0.05;
};

struct ConnectionConfig {
    // connection
    std::string ip;
    int port_scan = 0;
    int port_odom = 0;
    int port_cmd = 0;
};

struct ExportConfig {
    // export 
    std::string export_path_world;
    std::string export_path_frontiers;
    std::string export_path_pose;
};

struct Config {
    MappingConfig mapping;
    ConnectionConfig connection;
    ExportConfig export_cfg;
};

bool loadConfig(const std::string& filename, Config& cfg);