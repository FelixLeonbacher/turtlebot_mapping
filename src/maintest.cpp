#include <iostream>
#include <iomanip>
#include <vector>
#include <cmath>
#include "core/geometry.hpp"

using namespace core;

// ------------ helpers (scan/frontier workflow) ------------
static inline bool same_cell(const core::Point2D& a, const core::Point2D& b, float res) {
    long long ax = llround(a.x / res), ay = llround(a.y / res);
    long long bx = llround(b.x / res), by = llround(b.y / res);
    return ax == bx && ay == by;
}

static inline bool endpoint_is_wall_in_scan(const core::Point2D& endpoint,
                                            const core::ScanData& data,
                                            float res) {
    for (size_t i = 0; i < data.scan_ordered.size(); ++i) {
        if (!data.valid[i]) continue;
        const auto& p = data.scan_ordered[i];
        if (same_cell(p, endpoint, res) && p.is_wall) return true;
    }
    return false;
}

static inline void add_scan_to_world(const core::ScanData& data,
                                           core::MapSet& world_map,
                                           float res) {
    for (size_t i = 0; i < data.scan_ordered.size(); ++i) {
        const auto& p = data.scan_ordered[i];
        if (data.valid[i] && p.is_wall) {
            insert_rounded_unique(p, world_map, res);
        }
    }
}

static inline bool frontier_same_cells(const core::Frontier& f1,
                                       const core::Frontier& f2,
                                       float res) {
    bool a_eq = same_cell(f1.a, f2.a, res) && same_cell(f1.b, f2.b, res);
    bool b_eq = same_cell(f1.a, f2.b, res) && same_cell(f1.b, f2.a, res);
    return a_eq || b_eq;
}

static inline void add_new_frontiers(std::vector<core::Frontier>& global_frontiers,
                                     const std::vector<core::Frontier>& new_frontiers,
                                     float res) {
    for (const auto& nf : new_frontiers) {
        bool dup = false;
        for (const auto& gf : global_frontiers) {
            if (frontier_same_cells(nf, gf, res)) { dup = true; break; }
        }
        if (!dup) global_frontiers.push_back(nf);
    }
}

static inline void promote_frontiers_with_scan(std::vector<core::Frontier>& frontier_list,
                                               const core::ScanData& data,
                                               core::MapSet& world_map,
                                               float res,
                                               int min_neighbors = 2) {
    // Promote if BOTH endpoints are now observed as wall in this scan
    std::vector<core::Frontier> keep;
    keep.reserve(frontier_list.size());

    for (const auto& f : frontier_list) {
        bool a_now_wall = endpoint_is_wall_in_scan(f.a, data, res);
        bool b_now_wall = endpoint_is_wall_in_scan(f.b, data, res);
        if (a_now_wall && b_now_wall) {
            insert_rounded_unique(f.a, world_map, res);
            insert_rounded_unique(f.b, world_map, res);
        } else {
            keep.push_back(f);
        }
    }
    frontier_list.swap(keep);

    // Optional: consolidate walls via neighborhood after promotion (cheap)
    auto consolidated = compute_walls_from_set(world_map, res, min_neighbors);
    world_map.clear();
    for (const auto& p : consolidated) {
        if (p.is_wall) insert_rounded_unique(p, world_map, res);
    }
}

// ------------ pretty printers ------------
static inline void print_scan(const char* tag,
                              const core::ScanData& data,
                              const std::vector<core::Frontier>& fronts) {
    std::cout << "\n=== " << tag << " ===\n";
    std::cout << "points: " << data.scan_ordered.size()
              << "   frontiers: " << fronts.size() << "\n";

    for (size_t i = 0; i < data.scan_ordered.size(); ++i) {
        const auto& p = data.scan_ordered[i];
        std::cout << "  [" << i << "] (" << p.x << ", " << p.y
                  << ") valid=" << (int)data.valid[i]
                  << " wall=" << (p.is_wall ? 1 : 0) << "\n";
    }
    for (size_t i = 0; i < fronts.size(); ++i) {
        const auto& f = fronts[i];
        std::cout << "  Frontier #" << i
                  << "  A(" << f.a.x << ", " << f.a.y << ")"
                  << "  B(" << f.b.x << ", " << f.b.y << ")"
                  << "  M(" << f.m.x << ", " << f.m.y << ")"
                  << "  width=" << f.width << "\n";
    }
}

static inline void print_world(const char* tag, const core::MapSet& world_map) {
    std::cout << "\n" << tag << "  (size=" << world_map.size() << ")\n";
    for (const auto& p : world_map) {
        std::cout << "  (" << std::setw(6) << p.x
                  << ", " << std::setw(6) << p.y
                  << ")  wall=" << (p.is_wall ? 1 : 0)
                  << "\n";
    }
}

static inline void print_frontiers(const char* tag, const std::vector<core::Frontier>& Fs) {
    std::cout << "\n" << tag << "  count=" << Fs.size() << "\n";
    for (size_t i = 0; i < Fs.size(); ++i) {
        const auto& f = Fs[i];
        std::cout << "  #" << i
                  << "  A(" << f.a.x << ", " << f.a.y << ")"
                  << "  B(" << f.b.x << ", " << f.b.y << ")"
                  << "  M(" << f.m.x << ", " << f.m.y << ")"
                  << "  width=" << f.width << "\n";
    }
}

int main() {
    std::cout << std::fixed << std::setprecision(3);

    // ------------ global params ------------
    const float jump_thresh = 1.5f;
    const float inset_toward_robot = 0.10f;
    const float map_res = CORE_ROUND_RES; // e.g., 1.0f for your 1 m layout

    // ------------ global state ------------
    MapSet world_map(0, Point2DHash(map_res), Point2DEq(map_res)); // stores only walls (rounded)
    std::vector<Frontier> all_frontiers; // global frontier list (deduped)

    // ------------ Scan 1 ------------
    Pose2D pose1{0.0f, 0.0f, 0.0f};
    std::vector<float> ranges1 = {1.0f, 1.41f, 1.0f, 1.41f, 3.0f, 1.41f, 1.0f, 1.41f};
    float angle_min1 = 0.0f;           // 0°
    float angle_inc1 = 0.785398f;      // 45°
    float range_min = 0.10f, range_max = 30.0f;

    LidarScan scan1{ranges1, angle_min1, angle_inc1, range_min, range_max, pose1};
    ScanData data1;
    std::vector<Frontier> fronts1;

    // compute rounded scan + frontiers
    scan_to_data(scan1, data1, fronts1, jump_thresh, inset_toward_robot);
    print_scan("Scan 1 (raw)", data1, fronts1);

    // promote old frontiers (none yet) using current scan
    promote_frontiers_with_scan(all_frontiers, data1, world_map, map_res);

    // add new frontiers from this scan (deduped)
    add_new_frontiers(all_frontiers, fronts1, map_res);

    // add walls from this scan to world
    add_scan_to_world(data1, world_map, map_res);

    print_world("World after Scan 1 (walls only)", world_map);
    print_frontiers("Frontiers after Scan 1 (global)", all_frontiers);

    // ------------ Scan 2 ------------
    Pose2D pose2{-2.0f, 0.0f, 0.0f};
    std::vector<float> ranges2 = {3.0f, 1.41f, 2.0f, 1.41f, 1.0f, 1.41f, 3.0f, 1.41f};
    float angle_min2 = 0.0f;           // 0°
    float angle_inc2 = 0.785398f;      // 45°

    LidarScan scan2{ranges2, angle_min2, angle_inc2, range_min, range_max, pose2};
    ScanData data2;
    std::vector<Frontier> fronts2;

    scan_to_data(scan2, data2, fronts2, jump_thresh, inset_toward_robot);
    print_scan("Scan 2 (raw)", data2, fronts2);

    // promote existing global frontiers using this scan
    promote_frontiers_with_scan(all_frontiers, data2, world_map, map_res);

    // add new frontiers from this scan
    add_new_frontiers(all_frontiers, fronts2, map_res);

    // add walls from this scan
    add_scan_to_world(data2, world_map, map_res);

    print_world("World after Scan 2 (walls only)", world_map);
    print_frontiers("Frontiers after Scan 2 (global)", all_frontiers);

    // ------------ final summary ------------
    std::cout << "\n========= FINAL SUMMARY =========\n";
    std::cout << "Walls in world_map: " << world_map.size() << "\n";
    std::cout << "Frontiers total: " << all_frontiers.size() << "\n";

    return 0;
}
