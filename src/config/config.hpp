// soon to be implemented

#include <string>


struct MappingConfig {
    double jump_threshold;
    double map_resolution;
};

struct ConnectionConfig {
    std::string ip;
    int port_scan;
    int port_odom;
    int port_cmd;
};

struct ExportConfig {
    std::string export_path_world;
    std::string export_path_frontiers;
};

struct Config {
    MappingConfig    mapping;
    ConnectionConfig connection;
    ExportConfig     export_cfg;  // can't name it "export" in C++ easily
};


