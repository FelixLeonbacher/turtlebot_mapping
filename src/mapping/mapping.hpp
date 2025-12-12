// src/mapping/mapping.hpp
#pragma once

#include "../core/geometry.hpp"
#include <vector>
#include <unordered_set>

namespace mapping {

    struct Point2DHash {
        explicit Point2DHash(float res = CORE_ROUND_RES) : inv_res(1.0f / res) {}
        size_t operator()(const core::Point2D& p) const noexcept {
            const long long ix = llround(p.x * inv_res);
            const long long iy = llround(p.y * inv_res);
            return std::hash<long long>()((ix << 32) ^ (unsigned long long)iy);
        }
        double inv_res;
    };

    struct Point2DEq {
        explicit Point2DEq(float res = CORE_ROUND_RES) : inv_res(1.0f / res) {}
        bool operator()(const core::Point2D& a, const core::Point2D& b) const noexcept {
            const long long ax = llround(a.x * inv_res);
            const long long ay = llround(a.y * inv_res);
            const long long bx = llround(b.x * inv_res);
            const long long by = llround(b.y * inv_res);
            return ax == bx && ay == by;
        }
        double inv_res;
    };

    // Alias für die Weltkarte
    using MapSet = std::unordered_set<core::Point2D,
                                      Point2DHash,
                                      Point2DEq>;

    // Deklaration: Scan-Daten → Weltkarte
    void data_to_world(const core::ScanData& scan_data,
                       std::vector<core::Frontier>& scan_frontiers,
                       MapSet& world_map,
                       std::vector<core::Frontier>& all_frontiers,
                       float map_res);

    // Deklaration: LidarScan → ScanData + Frontiers
    void scan_to_data(const core::LidarScan& scan,
                      core::ScanData& out_scan,
                      std::vector<core::Frontier>& out_frontiers,
                      float jump_thresh = 1.0f);


    void polish_frontiers(std::vector<core::Frontier>& frontiers,
                        float min_width);

} // namespace mapping
