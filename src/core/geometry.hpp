#pragma once
#include <cmath>
#include <vector>
#include <unordered_set>
#include <limits>
#include <cstdint>


namespace core {

// -------------------- Types --------------------
struct Point2D {
    float x{0.0f};
    float y{0.0f};
    bool  isWall{false};   // ob als Wand klassifiziert
};

struct Pose2D {
    float x{0.0f};
    float y{0.0f};
    float theta{0.0f};     // rad
};

// -------------------- Helpers --------------------
inline Point2D polarToCartesian(float range, float angle) {
    return { range * std::cos(angle), range * std::sin(angle) };
}

inline void cartesianToPolar(const Point2D& p, float& range, float& angle) {
    range = std::sqrt(p.x * p.x + p.y * p.y);
    angle = std::atan2(p.y, p.x);
}

inline Point2D pointInWorldFrame(const Pose2D& lidarPose, const Point2D& pLidar) {
    float c = std::cos(lidarPose.theta);
    float s = std::sin(lidarPose.theta);
    return {
        lidarPose.x + c * pLidar.x - s * pLidar.y,
        lidarPose.y + s * pLidar.x + c * pLidar.y,
        pLidar.isWall
    };
}

inline Point2D pointInLidarFrame(const Pose2D& lidarPose, const Point2D& pWorld) {
    float c = std::cos(lidarPose.theta);
    float s = std::sin(lidarPose.theta);
    float dx = pWorld.x - lidarPose.x;
    float dy = pWorld.y - lidarPose.y;
    return { c * dx + s * dy, -s * dx + c * dy, pWorld.isWall };
}

// Runden auf ein Raster (z.B. 5 cm)
inline Point2D roundPoint(const Point2D& p, float resolution = 0.05f) {
    Point2D r;
    r.x = std::round(p.x / resolution) * resolution;
    r.y = std::round(p.y / resolution) * resolution;
    r.isWall = p.isWall;
    return r;
}

// Hash/Equal passend zur Rasterung (vermeidet Float-Ärger)
struct Point2DHash {
    explicit Point2DHash(float res = 0.05f) : inv_res(std::round(1.0f / res)) {}
    size_t operator()(const Point2D& p) const noexcept {
        // diskretisiere auf Raster in Integer
        int ix = static_cast<int>(std::llround(p.x * inv_res));
        int iy = static_cast<int>(std::llround(p.y * inv_res));
        // einfache 2D-Hash-Kombination
        return std::hash<long long>()((static_cast<long long>(ix) << 32) ^ (static_cast<unsigned int>(iy)));
    }
    double inv_res;
};
struct Point2DEq {
    explicit Point2DEq(float res = 0.05f) : inv_res(std::round(1.0f / res)) {}
    bool operator()(const Point2D& a, const Point2D& b) const noexcept {
        int ax = static_cast<int>(std::llround(a.x * inv_res));
        int ay = static_cast<int>(std::llround(a.y * inv_res));
        int bx = static_cast<int>(std::llround(b.x * inv_res));
        int by = static_cast<int>(std::llround(b.y * inv_res));
        return ax == bx && ay == by;
    }
    double inv_res;
};

// -------------------- Main: Scan -> World Points --------------------
// Frontier-Logik: wir nutzen die originalen ranges; nur wenn beide gültig und
// |r[i]-r[i-1]| > jumpThreshold, setzen wir beide auf isWall=false.
inline void laserScanToWorldPoints(const std::vector<float>& ranges,
                                   float angle_min, float angle_increment,
                                   float range_min, float range_max,
                                   const Pose2D& lidarPose,
                                   std::vector<Point2D>& outPoints,
                                   float frontierJumpThreshold = 5.0f) // ggf. 1.0f indoor
{
    const size_t N = ranges.size();
    outPoints.reserve(outPoints.size() + N);

    // 1) Valide Beams markieren
    std::vector<uint8_t> valid(N, 0);
    for (size_t i = 0; i < N; ++i) {
        float r = ranges[i];
        valid[i] = (r > range_min && r < range_max) ? 1u : 0u;
    }

    // 2) Frontier-Detektion auf Basis der Ranges
    std::vector<uint8_t> isWall(N, 0);
    for (size_t i = 0; i < N; ++i) isWall[i] = valid[i]; // initial: gültige sind Wand
    for (size_t i = 1; i < N; ++i) {
        if (valid[i] && valid[i-1]) {
            if (std::fabs(ranges[i] - ranges[i-1]) > frontierJumpThreshold) {
                isWall[i]   = 0;
                isWall[i-1] = 0;
            }
        }
    }

    // 3) Punkte erzeugen, in Welt transformieren
    for (size_t i = 0; i < N; ++i) {
        if (!valid[i]) continue;
        float angle = angle_min + static_cast<float>(i) * angle_increment;
        Point2D local = polarToCartesian(ranges[i], angle);
        local.isWall = (isWall[i] != 0);
        Point2D world = pointInWorldFrame(lidarPose, local);
        outPoints.push_back(world);
    }
}

// -------------------- unique-insert (gerundet) --------------------
inline void insertRoundedUnique(const Point2D& pWorld,
                                std::unordered_set<Point2D, Point2DHash, Point2DEq>& mapSet,
                                float resolution = 0.05f)
{
    // Achtung: mapSet muss mit denselben Hash/Eq-Objekten (gleiche Auflösung) erstellt werden.
    Point2D rounded = roundPoint(pWorld, resolution);
    mapSet.insert(rounded); // kein Duplikat, wenn Zelle schon vorhanden
}

} // namespace core
