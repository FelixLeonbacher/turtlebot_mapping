#include <iostream>
#include <iomanip>
#include <vector>

#include "core/geometry.hpp"
#include "mapping/mapping.hpp" 
#include "export/export.hpp"


#include "core/parser.hpp"

using namespace core;

int main() {
    std::cout << std::fixed << std::setprecision(3);

    // --- Global parameters ---
    const float jump_thresh = 1.0f;      // frontier-Erkennungsschwelle
    const float map_res     = CORE_ROUND_RES;  // Rundung in m (z.B. 1.0f oder 0.05f) derzeit aus geometry.hpp

    // --- 1) Globale Welt & Frontier-Liste ---
    mapping::MapSet world_map(0, mapping::Point2DHash(map_res), mapping::Point2DEq(map_res));
    std::vector<Frontier> all_frontiers;

    // =======================================================
    // === 2) Erster Scan ====================================
    // =======================================================
    Pose2D pose1{0.0f, 0.0f, 0.0f};
    std::vector<float> ranges1 = {1.0f, 1.41f, 1.0f, 1.41f, 3.0f, 1.41f, 1.0f, 1.41f};
    float angle_min1 = 0.0f;         // 0°
    float angle_inc1 = 0.785398f;    // 45°
    float range_min1 = 0.10f, range_max1 = 30.0f;

    LidarScan scan1{ranges1, angle_min1, angle_inc1, range_min1, range_max1, pose1};
    ScanData data1;
    std::vector<Frontier> frontiers1;

    mapping::scan_to_data(scan1, data1, frontiers1, jump_thresh);

    std::cout << "\n=== First scan: frontiers ===\n";
    for (const auto& f : frontiers1) {
        std::cout << "  A(" << f.a.x << ", " << f.a.y << ")"
                  << "  B(" << f.b.x << ", " << f.b.y << ")"
                  << "  M(" << f.m.x << ", " << f.m.y << ")"
                  << "  width=" << f.width << "\n";
    }

    std::cout << "\nFirst scan: points (scan_ordered)\n";
    for (const auto& p : data1.scan_ordered) {
        std::cout << "  (" << p.x << ", " << p.y << ")  wall=" << (p.is_wall ? 1 : 0) << "\n";
    }

    std::cout << "\nWorld points with walls BEFORE inserting 1st scan:\n";
    for (const auto& p : world_map) {
        std::cout << "  (" << p.x << ", " << p.y << ")  wall=" << (p.is_wall ? 1 : 0) << "\n";
    }

    // --- Scan-1-Daten in Welt integrieren ---
    mapping::data_to_world(data1, frontiers1, world_map, all_frontiers, map_res);

    std::cout << "\nWorld points with walls AFTER inserting 1st scan:\n";
    for (const auto& p : world_map) {
        std::cout << "  (" << p.x << ", " << p.y << ")  wall=" << (p.is_wall ? 1 : 0) << "\n";
    }

    std::cout << "\nFrontiers after 1st scan (global):\n";
    for (const auto& f : all_frontiers) {
        std::cout << "  A(" << f.a.x << ", " << f.a.y << ")"
                  << "  B(" << f.b.x << ", " << f.b.y << ")"
                  << "  M(" << f.m.x << ", " << f.m.y << ")"
                  << "  width=" << f.width << "\n";
    }

    export_utils::export_world_map_csv(world_map, "../export/world_map_after_scan1.csv");
    export_utils::export_frontiers_csv(all_frontiers, "../export/frontiers_after_scan1.csv");


    // =======================================================
    // === 3) Zweiter Scan ===================================
    // =======================================================
    Pose2D pose2{-2.0f, 0.0f, 0.0f};
    std::vector<float> ranges2 = {3.0f, 1.41f, 2.0f, 1.41f, 1.0f, 1.41f, 3.0f, 1.41f};
    float angle_min2 = 0.0f;         // 0°
    float angle_inc2 = 0.785398f;    // 45°
    float range_min2 = 0.10f, range_max2 = 30.0f;

    LidarScan scan2{ranges2, angle_min2, angle_inc2, range_min2, range_max2, pose2};
    ScanData data2;
    std::vector<Frontier> frontiers2;

    mapping::scan_to_data(scan2, data2, frontiers2, jump_thresh);

    std::cout << "\n=== Second scan: frontiers ===\n";
    for (const auto& f : frontiers2) {
        std::cout << "  A(" << f.a.x << ", " << f.a.y << ")"
                  << "  B(" << f.b.x << ", " << f.b.y << ")"
                  << "  M(" << f.m.x << ", " << f.m.y << ")"
                  << "  width=" << f.width << "\n";
    }

    std::cout << "\nSecond scan: points (scan_ordered)\n";
    for (const auto& p : data2.scan_ordered) {
        std::cout << "  (" << p.x << ", " << p.y << ")  wall=" << (p.is_wall ? 1 : 0) << "\n";
    }

    std::cout << "\nWorld points with walls BEFORE inserting 2nd scan:\n";
    for (const auto& p : world_map) {
        std::cout << "  (" << p.x << ", " << p.y << ")  wall=" << (p.is_wall ? 1 : 0) << "\n";
    }

    // --- Scan-2-Daten in Welt integrieren ---
    mapping::data_to_world(data2, frontiers2, world_map, all_frontiers, map_res);

    

    export_utils::export_world_map_csv(world_map, "../export/world_map_after_scan2.csv");
    export_utils::export_frontiers_csv(all_frontiers, "../export/frontiers_after_scan2.csv");


    std::cout << "\nWorld points with walls AFTER inserting 2nd scan:\n";
    for (const auto& p : world_map) {
        std::cout << "  (" << p.x << ", " << p.y << ")  wall=" << (p.is_wall ? 1 : 0) << "\n";
    }

    std::cout << "\nFinal frontiers (global):\n";
    for (const auto& f : all_frontiers) {
        std::cout << "  A(" << f.a.x << ", " << f.a.y << ")"
                  << "  B(" << f.b.x << ", " << f.b.y << ")"
                  << "  M(" << f.m.x << ", " << f.m.y << ")"
                  << "  width=" << f.width << "\n";
    }

    std::cout << "\n========= FINAL WORLD =========\n";
    for (const auto& p : world_map) {
        std::cout << "  (" << p.x << ", " << p.y << ")  wall=" << (p.is_wall ? 1 : 0) << "\n";
    }

    return 0;
}
