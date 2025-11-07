#include <iostream>
#include <vector>
#include <unordered_set>
#include "core/geometry.hpp"

int main() {
    // --- 1. Weltkarte vorbereiten ---
    float resolution = 0.05f; // 5 cm Raster
    std::unordered_set<core::Point2D, core::Point2DHash, core::Point2DEq> worldMap(
        0, core::Point2DHash(resolution), core::Point2DEq(resolution));

    // --- 2. Beispielhafte Pose + Scan ---
    core::Pose2D pose{2.0f, 1.0f, 0.3f};
    std::vector<float> ranges = {1.0f, 1.5f, 2.0f, 7.0f, 17.0f, 17.2f};
    float angle_min = -1.57f;
    float angle_inc = 0.3f;
    float range_min = 0.1f, range_max = 30.0f;

    // --- 3. Scan verarbeiten ---
    std::vector<core::Point2D> worldPoints;
    core::laserScanToWorldPoints(ranges, angle_min, angle_inc,
                                 range_min, range_max, pose,
                                 worldPoints, 5.0f); // 5 m Frontier-Sprung

    // --- 4. Punkte runden + in Weltkarte einfügen ---
    for (const auto& p : worldPoints) {
        core::Point2D rounded = core::roundPoint(p, resolution);
        worldMap.insert(rounded);
    }

    // --- 5. Ausgabe ---
    std::cout << "Weltkarte enthält " << worldMap.size() << " Punkte:\n";
    for (const auto& p : worldMap) {
        std::cout << "(" << p.x << ", " << p.y 
                  << ") isWall=" << p.isWall << "\n";
    }
}
