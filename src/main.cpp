#include <iostream>
#include <iomanip>
#include <vector>
#include "core/geometry.hpp"

using namespace core;

int main() {
    std::cout << std::fixed << std::setprecision(3);

    // --- Global parameters ---
    const float jump_thresh = 1.0f;
    const float inset_toward_robot = 0.10f;
    const float map_res = CORE_ROUND_RES;

    // --- 1) Global world containers ---
    MapSet world_map(0, Point2DHash(map_res), Point2DEq(map_res));
    std::vector<Frontier> all_frontiers;

    // =======================================================
    // === 2) First scan (creates frontiers) ================
    // =======================================================
    Pose2D pose1{0.0f, 0.0f, 0.0f};
    std::vector<float> ranges1 = {2.0f, 2.82f, 2.0f, 2.82f, 5.0f, 2.82f, 2.0f, 2.82f};
    float angle_min1 = 0.0f;              // 0°
    float angle_inc1 = 0.785398f;         // 45°
    float range_min1 = 0.10f, range_max1 = 30.0f;

    LidarScan scan1{ranges1, angle_min1, angle_inc1, range_min1, range_max1, pose1};
    ScanData data1;
    std::vector<Frontier> frontiers1;

    //scan_to_data(scan1, data1, frontiers1, jump_thresh, inset_toward_robot);

    // Insert rounded points from first scan
    for (const auto& p : data1.scan_ordered)
        if (p.x != 0.0f || p.y != 0.0f)
            insert_rounded_unique(p, world_map, map_res);

    all_frontiers.insert(all_frontiers.end(), frontiers1.begin(), frontiers1.end());

    // =======================================================
    // === 3) Second scan (new data, new possible frontiers) =
    // =======================================================
    Pose2D pose2{1.0f, 0.0f, 0.0f};
    std::vector<float> ranges2 = {1.0f, 1.41f, 1.0f, 1.41f, 3.0f, 3*1.41, 5.0f, 2.82f };//{1.0f, 1.41f, 2.0f, 2.82f, 6.0f, 2.82f, 2.0f, 1.41f};
    float angle_min2 = 0.0f;            // 0°
    float angle_inc2 = 0.785398f;         // 45°
    float range_min2 = 0.10f, range_max2 = 30.0f;

    LidarScan scan2{ranges2, angle_min2, angle_inc2, range_min2, range_max2, pose2};
    ScanData data2;
    std::vector<Frontier> frontiers2;

    scan_to_data(scan2, data2, frontiers2, jump_thresh, inset_toward_robot);

    // Insert rounded points from second scan
    for (const auto& p : data2.scan_ordered)
        if (p.x != 0.0f || p.y != 0.0f)
            insert_rounded_unique(p, world_map, map_res);

    all_frontiers.insert(all_frontiers.end(), frontiers2.begin(), frontiers2.end());



    // =======================================================
    // === 5) Print summary ==================================
    // =======================================================
    std::cout << "\n========= SCAN SUMMARY =========\n";
    std::cout << "World map (unique rounded points): " << world_map.size() << "\n";
    std::cout << "Frontiers total: " << all_frontiers.size() << "\n";

    std::cout << "\nFrontiers:\n";
    int idx = 0;
    for (const auto& f : all_frontiers) {
        std::cout << "  #" << idx++
                  << "  A(" << f.a.x << ", " << f.a.y << ")"
                  << "  B(" << f.b.x << ", " << f.b.y << ")"
                  << "  M(" << f.m.x << ", " << f.m.y << ")"
                  << "  width=" << f.width << "\n";
    }

    std::cout << "\nWorld points with walls:\n";
    for (const auto& p : world_map) {
        std::cout << "  (" << p.x << ", " << p.y << ")  wall=" << (p.is_wall ? 1 : 0) << "\n";
    }

    return 0;
}
