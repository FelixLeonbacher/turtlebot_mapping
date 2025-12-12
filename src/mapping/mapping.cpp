/**
 * @file mapping.cpp
 * @brief How the map is created and updated
 * @author Felix Leonbacher
 * @version 1.0
 */

#include "../core/geometry.hpp"
#include "mapping.hpp"
#include <cmath>
#include <vector>
#include <unordered_set>

namespace mapping {

    // Hilfsfunktion: Punkt in Rasterzelle vergleichen
    static inline bool same_cell(const core::Point2D& p1, const core::Point2D& p2, float res) {
        long long ix1 = llround(p1.x / res);
        long long iy1 = llround(p1.y / res);
        long long ix2 = llround(p2.x / res);
        long long iy2 = llround(p2.y / res);
        return ix1 == ix2 && iy1 == iy2;
    }

    // ==========================================================
    //                 MAP OPERATIONS 
    // ==========================================================

    inline void insert_rounded_unique(const core::Point2D& p_world,
                                      std::unordered_set<core::Point2D,
                                                         mapping::Point2DHash,
                                                         mapping::Point2DEq>& map_set,
                                      float resolution)
    {
        map_set.insert(core::round_point(p_world, resolution));
    }

    /**
     * data_to_world:
     *  - fügt die Scan-Punkte in die Weltkarte ein
     *  - setzt Frontier-Endpunkte → Wandpunkte
     *  - löscht Frontiers, deren Endpunkte schon Wand sind
     *  - fügt neue Frontiers zur globalen frontier-Liste hinzu
     */
    void data_to_world(const core::ScanData& scan_data,
                       std::vector<core::Frontier>& scan_frontiers,
                       MapSet& world_map,
                       std::vector<core::Frontier>& all_frontiers,
                       float map_res)
    {
        // === 1) Alle Punkte des Scans in Weltkarte übernehmen ===
        for (const auto& p : scan_data.scan_ordered)
        {
            if (p.x == 0.0f && p.y == 0.0f)
                continue;

            insert_rounded_unique(p, world_map, map_res);

            // Prüfe, ob der Punkt ein Endpunkt eines bestehenden globalen Frontiers ist
            for (auto it = all_frontiers.begin(); it != all_frontiers.end(); /* kein ++ */)
            {
                bool is_endpoint =
                    same_cell(p, it->a, map_res) ||
                    same_cell(p, it->b, map_res);

                if (is_endpoint)
                {
                    // (a) Punkt in der Weltkarte zu einer Wand machen
                    core::Point2D wall_point = p;
                    wall_point.is_wall = true;

                    auto existing = world_map.find(p);
                    if (existing != world_map.end())
                        world_map.erase(existing);

                    insert_rounded_unique(wall_point, world_map, map_res);

                    // (b) Frontier löschen
                    it = all_frontiers.erase(it);
                }
                else
                {
                    ++it;
                }
            }
        }

        // === 2) Neu erzeugte Frontiers aus diesem Scan säubern ===
        for (auto it = scan_frontiers.begin(); it != scan_frontiers.end(); /* kein ++ */)
        {
            bool remove_frontier = false;

            for (const auto& wp : world_map)
            {
                if (!wp.is_wall) continue;

                if (same_cell(it->a, wp, map_res) || same_cell(it->b, wp, map_res))
                {
                    remove_frontier = true;
                    break;
                }
            }

            if (remove_frontier)
                it = scan_frontiers.erase(it);
            else
                ++it;
        }

        // === 3) Übriggebliebene Frontiers zum globalen Frontier-Set hinzufügen ===
        all_frontiers.insert(all_frontiers.end(),
                             scan_frontiers.begin(),
                             scan_frontiers.end());
    }

    // ==========================================================
    //                 SCAN → DATA (ROUNDED)
    // ==========================================================

    void scan_to_data(const core::LidarScan& scan,
                      core::ScanData& out_scan,
                      std::vector<core::Frontier>& out_frontiers,
                      float jump_thresh)
    {
        const size_t n = scan.ranges.size();
        out_scan.scan_ordered.assign(n, core::Point2D{0.0f, 0.0f, false});  // alle standardmäßig kein Wandpunkt
        out_scan.valid.assign(n, 0);
        out_frontiers.clear();
        out_frontiers.reserve(n / 8);

        // keep previous beam’s info to decide frontier with the current beam
        bool  prev_valid = false;
        float prev_r     = 0.0f;
        float prev_a     = 0.0f;
        core::Point2D prev_world{};  // rounded

        for (size_t i = 0; i < n; ++i) {
            const float r = scan.ranges[i];
            const float a = scan.angle_min + static_cast<float>(i) * scan.angle_inc;

            const bool valid = (r > scan.range_min && r < scan.range_max);
            out_scan.valid[i] = valid;

            core::Point2D world_i{};
            if (valid) {
                world_i = to_world_rounded(scan.pose, r, a, CORE_ROUND_RES, true);
                out_scan.scan_ordered[i] = world_i;
            }

            if (i > 0) {
                // jump if both valid and |Δr| large OR validity changed
                const bool jump = (prev_valid && valid && std::fabs(r - prev_r) > jump_thresh)
                                  || (prev_valid != valid);

                if (jump) {
                    core::Point2D A, B;

                    if (prev_valid && valid) {
                        // both valid → endpoints are prev and current
                        A = prev_world;
                        B = world_i;
                    } else if (!prev_valid && valid) {
                        // prev invalid, current valid → use current range at prev angle
                        core::Point2D at_prev = to_world_rounded(scan.pose, r, prev_a, CORE_ROUND_RES, true);
                        A = at_prev;
                        B = world_i;
                    } else if (prev_valid && !valid) {
                        // prev valid, current invalid → use prev range at current angle
                        core::Point2D at_curr = to_world_rounded(scan.pose, prev_r, a, CORE_ROUND_RES, true);
                        A = prev_world;
                        B = at_curr;
                    } else {
                        // both invalid → no frontier
                        prev_valid = valid;
                        prev_r     = r;
                        prev_a     = a;
                        prev_world = world_i;
                        continue;
                    }

                    // midpoint with small inset toward robot, then round
                    core::Point2D M{
                        0.5f * (A.x + B.x),
                        0.5f * (A.y + B.y),
                        false
                    };

                    M = core::round_point(M, CORE_ROUND_RES);

                    // width
                    const float dx = A.x - B.x;
                    const float dy = A.y - B.y;
                    const float width = std::sqrt(dx * dx + dy * dy);

                    out_frontiers.push_back(core::Frontier{A, B, M, width});

                    // markieren: Frontier → diese beiden Strahlen sind KEINE Wandpunkte
                    out_scan.scan_ordered[i].is_wall     = false;
                    out_scan.scan_ordered[i - 1].is_wall = false;
                }
            }

            // carry state to next iteration
            prev_valid = valid;
            prev_r     = r;
            prev_a     = a;
            prev_world = world_i; // safe even if invalid; only used when prev_valid==true
        }
    }


    // bugfix: es gibt noch nen Problem mit der Frontier - Löschung (deshalb dieser temp. Fix)
    void polish_frontiers(std::vector<core::Frontier>& frontiers,
                        float min_width)
    {
        for (auto it = frontiers.begin(); it != frontiers.end(); /* kein ++ */)
        {
            if (it->width < min_width)
            {
                it = frontiers.erase(it);
            }
            else
            {
                ++it;
            }
        }
    }

} // namespace mapping
