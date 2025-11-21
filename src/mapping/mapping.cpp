// mapping.cpp
#include "../core/geometry.hpp"
#include "mapping.hpp"
#include <cmath>
#include <vector>

namespace mapping {

using core::Point2D;
using core::Frontier;
using core::ScanData;
using core::MapSet;
using core::insert_rounded_unique;

static inline bool same_cell(const Point2D& p1, const Point2D& p2, float res) {
    long long ix1 = llround(p1.x / res);
    long long iy1 = llround(p1.y / res);
    long long ix2 = llround(p2.x / res);
    long long iy2 = llround(p2.y / res);
    return ix1 == ix2 && iy1 == iy2;
}

/**
 * data_to_world:
 *  - fügt die Scan-Punkte in die Weltkarte ein
 *  - setzt Frontier-Endpunkte → Wandpunkte
 *  - löscht Frontiers, deren Endpunkte schon Wand sind
 *  - fügt neue Frontiers zur globalen frontier-Liste hinzu
 */
void data_to_world(const ScanData& scan_data,
                   std::vector<Frontier>& scan_frontiers,
                   MapSet& world_map,
                   std::vector<Frontier>& all_frontiers,
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
                Point2D wall_point = p;
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

} // namespace mapping
