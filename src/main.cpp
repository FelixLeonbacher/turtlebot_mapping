#include <iostream>
#include <iomanip>
#include <vector>
#include "core/geometry.hpp"

using namespace core;

int main() {
    std::cout << std::fixed << std::setprecision(3);

    // --- Global parameters ---
    const float jump_thresh = 1.0f; // frontier erkennungsschwelle
    const float map_res = CORE_ROUND_RES;  // rundungswert für die Punkte 

    // --- 1) Global world containers ---
    MapSet world_map(0, Point2DHash(map_res), Point2DEq(map_res));
    std::vector<Frontier> all_frontiers;

    // =======================================================
    // === 2) First scan (creates frontiers) ================
    // =======================================================
    Pose2D pose1{0.0f, 0.0f, 0.0f};
    std::vector<float> ranges1 = {1, 1.41, 1, 1.41, 3, 1.41, 1, 1.41};//{2.0f, 2.82f, 2.0f, 2.82f, 5.0f, 2.82f, 2.0f, 2.82f};
    float angle_min1 = 0.0f;              // 0°
    float angle_inc1 = 0.785398f;         // 45°
    float range_min1 = 0.10f, range_max1 = 30.0f;

    LidarScan scan1{ranges1, angle_min1, angle_inc1, range_min1, range_max1, pose1};
    ScanData data1;
    std::vector<Frontier> frontiers1;

    scan_to_data(scan1, data1, frontiers1, jump_thresh);

    // Insert rounded points from first scan

    std::cout << "\n First scan:\n";
    for (const auto& f : frontiers1) {
        std::cout << "  A(" << f.a.x << ", " << f.a.y << ")"
                  << "  B(" << f.b.x << ", " << f.b.y << ")"
                  << "  M(" << f.m.x << ", " << f.m.y << ")"
                  << "  width=" << f.width << "\n";
    }

    // data print
    for (const auto& p : data1.scan_ordered)
    {
        std::cout << "  (" << p.x << ", " << p.y << ")  wall=" << (p.is_wall ? 1 : 0) << "\n";
    }

    std::cout << "\nWorld points with walls before insert 1 scan:\n";
    for (const auto& p : world_map) {
        std::cout << "  (" << p.x << ", " << p.y << ")  wall=" << (p.is_wall ? 1 : 0) << "\n";
    }

    for (const auto& p : data1.scan_ordered)
    {
        if (p.x == 0.0f && p.y == 0.0f)
            continue;

        // 1) Punkt in Weltkarte einfügen (wenn neu)
        world_map.insert(p);

        // 2) Prüfen, ob er zu einem Frontier-Endpunkt gehört
        for (auto it = all_frontiers.begin(); it != all_frontiers.end(); /* kein ++ hier */)
        {
            bool is_endpoint = 
                (p.x == it->a.x && p.y == it->a.y) ||
                (p.x == it->b.x && p.y == it->b.y);

            if (is_endpoint)
            {
                // ---- (a) Den Punkt in world_map auf is_wall = true setzen ----
                // zuerst alten rausnehmen
                auto existing = world_map.find(p);
                if (existing != world_map.end())
                    world_map.erase(existing);

                // dann als Wand neu einfügen
                Point2D wall_point = p;
                wall_point.is_wall = true;
                world_map.insert(wall_point);

                // ---- (b) Frontier löschen ----
                it = all_frontiers.erase(it);
            }
            else
            {
                ++it;
            }
        }
    }


    for (auto it = frontiers1.begin(); it != frontiers1.end(); )
    {
        bool remove_frontier = false;

        for (const auto& wp : world_map)
        {
            if (wp.is_wall)
            {
                // Vergleiche positionsgerastert (nicht nur float-gleich!)
                auto same_cell = [&](const core::Point2D& p1, const core::Point2D& p2) {
                    long long ix1 = llround(p1.x / map_res);
                    long long iy1 = llround(p1.y / map_res);
                    long long ix2 = llround(p2.x / map_res);
                    long long iy2 = llround(p2.y / map_res);
                    return ix1 == ix2 && iy1 == iy2;
                };

                if (same_cell(it->a, wp) || same_cell(it->b, wp))
                {
                    remove_frontier = true;
                    break;
                }
            }
        }

        if (remove_frontier)
            it = frontiers1.erase(it);  // Frontier löschen
        else
            ++it;
    }



    all_frontiers.insert(all_frontiers.end(), frontiers1.begin(), frontiers1.end());

    std::cout << "\nWorld points with walls after insert 1 scan:\n";
    for (const auto& p : world_map) {
        std::cout << "  (" << p.x << ", " << p.y << ")  wall=" << (p.is_wall ? 1 : 0) << "\n";
    }

    // =======================================================
    // === 3) Second scan (new data, new possible frontiers) =
    // =======================================================
    Pose2D pose2{-2.0f, 0.0f, 0.0f};
    std::vector<float> ranges2 = {3.0f, 1.41f, 2.0f, 1.41f, 1.0f, 1.41f, 3.0f, 1.41f};
    float angle_min2 = 0.0f;            // 0°
    float angle_inc2 = 0.785398f;         // 45°
    float range_min2 = 0.10f, range_max2 = 30.0f;

    LidarScan scan2{ranges2, angle_min2, angle_inc2, range_min2, range_max2, pose2};
    ScanData data2;
    std::vector<Frontier> frontiers2;

    scan_to_data(scan2, data2, frontiers2, jump_thresh);


    std::cout << "\n Second scan:\n";
    for (const auto& f : frontiers2) {
        std::cout << "  A(" << f.a.x << ", " << f.a.y << ")"
                  << "  B(" << f.b.x << ", " << f.b.y << ")"
                  << "  M(" << f.m.x << ", " << f.m.y << ")"
                  << "  width=" << f.width << "\n";
    }

    // data print
    for (const auto& p : data2.scan_ordered)
    {
        std::cout << "  (" << p.x << ", " << p.y << ")  wall=" << (p.is_wall ? 1 : 0) << "\n";
    }

    std::cout << "\nWorld points 2 scan with walls:\n";
    for (const auto& p : world_map) {
        std::cout << "  (" << p.x << ", " << p.y << ")  wall=" << (p.is_wall ? 1 : 0) << "\n";
    }


    for (const auto& p : data2.scan_ordered)
    {
        if (p.x == 0.0f && p.y == 0.0f)
            continue;

        // 1) Punkt in Weltkarte einfügen
        world_map.insert(p);

        // 2) Prüfen, ob er zu einem Frontier-Endpunkt gehört
        for (auto it = all_frontiers.begin(); it != all_frontiers.end(); /* kein ++ hier */)
        {
            bool is_endpoint = 
                (p.x == it->a.x && p.y == it->a.y) ||
                (p.x == it->b.x && p.y == it->b.y);

            if (is_endpoint)
            {
                // ---- (a) Den Punkt in world_map auf is_wall = true setzen ----
                // zuerst alten rausnehmen
                auto existing = world_map.find(p);
                if (existing != world_map.end())
                    world_map.erase(existing);

                // dann als Wand neu einfügen
                Point2D wall_point = p;
                wall_point.is_wall = true;
                world_map.insert(wall_point);
                // ---- (b) Frontier löschen ----
                it = all_frontiers.erase(it);
            }
            else
            {
                ++it;
            }
        }
    }


    for (auto it = frontiers2.begin(); it != frontiers2.end(); )
    {
        bool remove_frontier = false;

        for (const auto& wp : world_map)
        {
            if (wp.is_wall)
            {
                // Vergleiche positionsgerastert (nicht nur float-gleich!)
                auto same_cell = [&](const core::Point2D& p1, const core::Point2D& p2) {
                    long long ix1 = llround(p1.x / map_res);
                    long long iy1 = llround(p1.y / map_res);
                    long long ix2 = llround(p2.x / map_res);
                    long long iy2 = llround(p2.y / map_res);
                    return ix1 == ix2 && iy1 == iy2;
                };

                if (same_cell(it->a, wp) || same_cell(it->b, wp))
                {
                    remove_frontier = true;
                    break;
                }
            }
        }

        if (remove_frontier)
            it = frontiers2.erase(it);  // Frontier löschen
        else
            ++it;
    }


    all_frontiers.insert(all_frontiers.end(), frontiers2.begin(), frontiers2.end());

    std::cout << "\nFinal frontiers scans:\n";
    for (const auto& f : all_frontiers) {
        std::cout << "  A(" << f.a.x << ", " << f.a.y << ")"
                  << "  B(" << f.b.x << ", " << f.b.y << ")"
                  << "  M(" << f.m.x << ", " << f.m.y << ")"
                  << "  width=" << f.width << "\n";
    }

    std::cout << "\nFinal World points with walls:\n";
    for (const auto& p : world_map) {
        std::cout << "  (" << p.x << ", " << p.y << ")  wall=" << (p.is_wall ? 1 : 0) << "\n";
    }


    return 0;
}
