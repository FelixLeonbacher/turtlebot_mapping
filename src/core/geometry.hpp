#pragma once
#include <cmath>
#include <vector>
#include <unordered_set>
#include <cstdint>

namespace core {

// ==========================================================
//                      CONFIGURATION
// ==========================================================
#ifndef CORE_ROUND_RES
#define CORE_ROUND_RES 0.05f  // default rounding resolution (5 cm)
#endif


// ==========================================================
//                      DATA STRUCTURES
// ==========================================================
struct Point2D {
    float x{0.0f};
    float y{0.0f};
    bool is_wall{false};
};

struct Pose2D {
    float x{0.0f};
    float y{0.0f};
    float theta{0.0f};  // [rad]
};

struct Frontier {
    core::Point2D a;  // left endpoint (world)
    core::Point2D b;  // right endpoint (world)
    core::Point2D m;  // midpoint (world)
    float width;      // |a - b|
};

struct LidarScan {
    const std::vector<float>& ranges;  // distances per beam
    float angle_min;                   // [rad]
    float angle_inc;                   // [rad/beam]
    float range_min;                   // [m]
    float range_max;                   // [m]
    core::Pose2D pose;                 // LiDAR pose in world
};

struct ScanData {
    std::vector<core::Point2D> scan_ordered;  // ordered, rounded world points
    std::vector<uint8_t> valid;               // validity mask per beam
};


// ==========================================================
//                      BASIC HELPERS
// ==========================================================
inline Point2D polar_to_cartesian(float r, float a) {
    return {r * std::cos(a), r * std::sin(a), false};
}

inline Point2D point_in_world_frame(const Pose2D& t, const Point2D& p_local) {
    float c = std::cos(t.theta), s = std::sin(t.theta);
    return {t.x + c * p_local.x - s * p_local.y,
            t.y + s * p_local.x + c * p_local.y,
            p_local.is_wall};
}

// round metric points to grid (e.g., 5 cm)
inline Point2D round_point(const Point2D& p, float res) {
    Point2D r;
    r.x = std::round(p.x / res) * res;
    r.y = std::round(p.y / res) * res;
    r.is_wall = p.is_wall;
    return r;
}

inline Point2D to_world_rounded(const Pose2D& pose, float r, float a, float res = CORE_ROUND_RES, bool isWall = false) {
    const float cR = std::cos(pose.theta);
    const float sR = std::sin(pose.theta);
    const float xl = r * std::cos(a);
    const float yl = r * std::sin(a);
    Point2D pw{
        pose.x + cR * xl - sR * yl,
        pose.y + sR * xl + cR * yl,
        isWall
    };
    return round_point(pw, res);
}

// ==========================================================
//                  HASH / EQUAL FOR POINTS
// ==========================================================
struct Point2DHash {
    explicit Point2DHash(float res = CORE_ROUND_RES) : inv_res(1.0f / res) {}
    size_t operator()(const Point2D& p) const noexcept {
        const long long ix = llround(p.x * inv_res);
        const long long iy = llround(p.y * inv_res);
        return std::hash<long long>()((ix << 32) ^ (unsigned long long)iy);
    }
    double inv_res;
};

struct Point2DEq {
    explicit Point2DEq(float res = CORE_ROUND_RES) : inv_res(1.0f / res) {}
    bool operator()(const Point2D& a, const Point2D& b) const noexcept {
        const long long ax = llround(a.x * inv_res);
        const long long ay = llround(a.y * inv_res);
        const long long bx = llround(b.x * inv_res);
        const long long by = llround(b.y * inv_res);
        return ax == bx && ay == by;
    }
    double inv_res;
};


// ==========================================================
//                 SCAN → DATA (ROUNDED)
// ==========================================================

inline void scan_to_data(const LidarScan& scan,
                         ScanData& out_scan,
                         std::vector<core::Frontier>& out_frontiers,
                         float jump_thresh = 1.0f)
{
    const size_t n = scan.ranges.size();
    out_scan.scan_ordered.assign(n, core::Point2D{0.0f, 0.0f, true});  // <-- alle standardmäßig Wand
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
                    // update prev and continue
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
                out_scan.scan_ordered[i].is_wall     = false;                      // 2 Strahl kein Wandpunkt
                out_scan.scan_ordered[i - 1].is_wall = false;                      // 1 Strahl kein Wandpunkt

            }
        }

        // carry state to next iteration
        prev_valid = valid;
        prev_r     = r;
        prev_a     = a;
        prev_world = world_i; // safe even if invalid; only used when prev_valid==true
    }
}



// ==========================================================
//                  SCAN → WORLD POINTS (RAW)
// ==========================================================
inline void laser_scan_to_world_points(const std::vector<float>& ranges,
                                       float angle_min, float angle_inc,
                                       float range_min, float range_max,
                                       const Pose2D& lidar_pose,
                                       std::vector<Point2D>& out_points)
{
    out_points.reserve(out_points.size() + ranges.size());
    for (size_t i = 0; i < ranges.size(); ++i) {
        float r = ranges[i];
        if (!(r > range_min && r < range_max)) continue;
        float a = angle_min + static_cast<float>(i) * angle_inc;
        Point2D local = polar_to_cartesian(r, a);
        Point2D world = point_in_world_frame(lidar_pose, local);
        out_points.push_back(world);
    }
}



// ==========================================================
//                 MAP OPERATIONS (WALLS / UNIQUE)
// ==========================================================

// Map type: unordered set of rounded points
using MapSet = std::unordered_set<Point2D, Point2DHash, Point2DEq>;

inline void insert_rounded_unique(const Point2D& p_world,
                                  std::unordered_set<Point2D, Point2DHash, Point2DEq>& map_set,
                                  float resolution)
{
    map_set.insert(round_point(p_world, resolution));
}

inline std::vector<Point2D> compute_walls_from_set(const MapSet& map_set,
                                                   float res,
                                                   int min_neighbors = 2)
{
    std::vector<Point2D> out;
    out.reserve(map_set.size());

    auto cell = [res](float x, float y) {
        return std::pair<long long, long long>{
            llround(x / res), llround(y / res)
        };
    };

    for (const auto& p : map_set) {
        auto [ix, iy] = cell(p.x, p.y);
        int neighbors = 0;

        for (int dy = -1; dy <= 1; ++dy) {
            for (int dx = -1; dx <= 1; ++dx) {
                if (dx == 0 && dy == 0) continue;

                Point2D q;
                q.x = (ix + dx) * res;
                q.y = (iy + dy) * res;

                if (map_set.count(q)) {
                    if (++neighbors >= min_neighbors) break;
                }
            }
            if (neighbors >= min_neighbors) break;
        }

        Point2D r = p;
        r.is_wall = (neighbors >= min_neighbors);
        out.push_back(r);
    }
    return out;
}

} // namespace core
