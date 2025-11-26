#include <iostream>
#include <fstream>
#include <string>
#include "picojson.h"

int main()
{
    const std::string filename = "../src/config.json";

    // --- Read file into string ---
    std::ifstream ifs(filename);
    if (!ifs.is_open()) {
        std::cerr << "Could not open config file: " << filename << "\n";
        return 1;
    }

    std::string json_text((std::istreambuf_iterator<char>(ifs)),
                           std::istreambuf_iterator<char>());

    // --- Parse JSON with picojson ---
    picojson::value v;
    std::string err = picojson::parse(v, json_text);
    if (!err.empty()) {
        std::cerr << "JSON parse error: " << err << "\n";
        return 1;
    }

    if (!v.is<picojson::object>()) {
        std::cerr << "Top-level JSON is not an object.\n";
        return 1;
    }

    picojson::object& root = v.get<picojson::object>();

    // ============================
    // mapping: { jump_threshold, map_resolution }
    // ============================
    double jump_threshold = 0.0;
    double map_resolution = 0.0;

    if (root["mapping"].is<picojson::object>()) {
        picojson::object& mapping = root["mapping"].get<picojson::object>();

        if (mapping["jump_threshold"].is<double>()) {
            jump_threshold = mapping["jump_threshold"].get<double>();
        } else {
            std::cerr << "Warning: mapping.jump_threshold missing or not number\n";
        }

        if (mapping["map_resolution"].is<double>()) {
            map_resolution = mapping["map_resolution"].get<double>();
        } else {
            std::cerr << "Warning: mapping.map_resolution missing or not number\n";
        }
    } else {
        std::cerr << "Warning: 'mapping' object missing in config\n";
    }

    // ============================
    // connection: { ip, port_scan, port_odom, port_cmd }
    // ============================
    std::string ip;
    int port_scan = 0;
    int port_odom = 0;
    int port_cmd  = 0;

    if (root["connection"].is<picojson::object>()) {
        picojson::object& conn = root["connection"].get<picojson::object>();

        if (conn["ip"].is<std::string>()) {
            ip = conn["ip"].get<std::string>();
        } else {
            std::cerr << "Warning: connection.ip missing or not string\n";
        }

        if (conn["port_scan"].is<double>()) {
            port_scan = static_cast<int>(conn["port_scan"].get<double>());
        } else {
            std::cerr << "Warning: connection.port_scan missing or not number\n";
        }

        if (conn["port_odom"].is<double>()) {
            port_odom = static_cast<int>(conn["port_odom"].get<double>());
        } else {
            std::cerr << "Warning: connection.port_odom missing or not number\n";
        }

        if (conn["port_cmd"].is<double>()) {
            port_cmd = static_cast<int>(conn["port_cmd"].get<double>());
        } else {
            std::cerr << "Warning: connection.port_cmd missing or not number\n";
        }
    } else {
        std::cerr << "Warning: 'connection' object missing in config\n";
    }

    // ============================
    // export: { export_path_world, export_path_frontiers }
    // ============================
    std::string export_path_world;
    std::string export_path_frontiers;

    if (root["export"].is<picojson::object>()) {
        picojson::object& ex = root["export"].get<picojson::object>();

        if (ex["export_path_world"].is<std::string>()) {
            export_path_world = ex["export_path_world"].get<std::string>();
        } else {
            std::cerr << "Warning: export.export_path_world missing or not string\n";
        }

        if (ex["export_path_frontiers"].is<std::string>()) {
            export_path_frontiers = ex["export_path_frontiers"].get<std::string>();
        } else {
            std::cerr << "Warning: export.export_path_frontiers missing or not string\n";
        }
    } else {
        std::cerr << "Warning: 'export' object missing in config\n";
    }

    // ============================
    // Print out everything
    // ============================
    std::cout << "===== Loaded config.json =====\n\n";

    std::cout << "[mapping]\n";
    std::cout << "  jump_threshold  = " << jump_threshold  << "\n";
    std::cout << "  map_resolution  = " << map_resolution  << "\n\n";

    std::cout << "[connection]\n";
    std::cout << "  ip         = " << ip        << "\n";
    std::cout << "  port_scan  = " << port_scan << "\n";
    std::cout << "  port_odom  = " << port_odom << "\n";
    std::cout << "  port_cmd   = " << port_cmd  << "\n\n";

    std::cout << "[export]\n";
    std::cout << "  export_path_world     = " << export_path_world     << "\n";
    std::cout << "  export_path_frontiers = " << export_path_frontiers << "\n";

    return 0;
}
