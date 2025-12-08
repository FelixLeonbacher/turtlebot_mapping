// config.cpp
#include "config.hpp"

#include <iostream>
#include <fstream>
#include <string>
#include <iterator>    // für std::istreambuf_iterator
#include "picojson.h"

bool load_config(const std::string& filename, AppConfig& cfg)
{
    // --- Read file into string ---
    std::ifstream ifs(filename);
    if (!ifs.is_open()) {
        std::cerr << "Could not open config file: " << filename << "\n";
        return false;
    }

    std::string json_text(
        (std::istreambuf_iterator<char>(ifs)),
        std::istreambuf_iterator<char>()
    );

    // --- Parse JSON with picojson ---
    picojson::value v;
    std::string err = picojson::parse(v, json_text);
    if (!err.empty()) {
        std::cerr << "JSON parse error: " << err << "\n";
        return false;
    }

    if (!v.is<picojson::object>()) {
        std::cerr << "Top-level JSON is not an object.\n";
        return false;
    }

    picojson::object& root = v.get<picojson::object>();

    // ============================
    // mapping
    // ============================
    if (root["mapping"].is<picojson::object>()) {
        picojson::object& mapping = root["mapping"].get<picojson::object>();

        if (mapping["jump_threshold"].is<double>()) {
            cfg.jump_threshold = mapping["jump_threshold"].get<double>();
        } else {
            std::cerr << "Warning: mapping.jump_threshold missing or not number\n";
        }

        if (mapping["map_resolution"].is<double>()) {
            cfg.map_resolution = mapping["map_resolution"].get<double>();
        } else {
            std::cerr << "Warning: mapping.map_resolution missing or not number\n";
        }
    } else {
        std::cerr << "Warning: 'mapping' object missing in config\n";
    }

    // ============================
    // connection
    // ============================
    if (root["connection"].is<picojson::object>()) {
        picojson::object& conn = root["connection"].get<picojson::object>();

        if (conn["ip"].is<std::string>()) {
            cfg.ip = conn["ip"].get<std::string>();
        } else {
            std::cerr << "Warning: connection.ip missing or not string\n";
        }

        if (conn["port_scan"].is<double>()) {
            cfg.port_scan = static_cast<int>(conn["port_scan"].get<double>());
        } else {
            std::cerr << "Warning: connection.port_scan missing or not number\n";
        }

        if (conn["port_odom"].is<double>()) {
            cfg.port_odom = static_cast<int>(conn["port_odom"].get<double>());
        } else {
            std::cerr << "Warning: connection.port_odom missing or not number\n";
        }

        if (conn["port_cmd"].is<double>()) {
            cfg.port_cmd = static_cast<int>(conn["port_cmd"].get<double>());
        } else {
            std::cerr << "Warning: connection.port_cmd missing or not number\n";
        }
    } else {
        std::cerr << "Warning: 'connection' object missing in config\n";
    }

    // ============================
    // export
    // ============================
    if (root["export"].is<picojson::object>()) {
        picojson::object& ex = root["export"].get<picojson::object>();

        if (ex["export_path_world"].is<std::string>()) {
            cfg.export_path_world = ex["export_path_world"].get<std::string>();
        } else {
            std::cerr << "Warning: export.export_path_world missing or not string\n";
        }

        if (ex["export_path_frontiers"].is<std::string>()) {
            cfg.export_path_frontiers = ex["export_path_frontiers"].get<std::string>();
        } else {
            std::cerr << "Warning: export.export_path_frontiers missing or not string\n";
        }
    } else {
        std::cerr << "Warning: 'export' object missing in config\n";
    }

    return true;
}