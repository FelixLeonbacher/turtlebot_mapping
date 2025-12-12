/**
 * @file geometry.hpp
 * @brief basic geometry types and helpers
 * @author Felix Leonbacher, Victor Swekis
 * @version 1.0
 */

#pragma once
#include <cmath>
#include <vector>
#include <cstdint>

namespace core {

// ==========================================================
//                      CONFIGURATION
// ==========================================================
#ifndef CORE_ROUND_RES
#define CORE_ROUND_RES 0.01f  // default rounding resolution in meters
#endif


// ==========================================================
//                      DATA STRUCTURES
// ==========================================================

// Victor 
struct Quaternion {
    double w, x, y, z;
};

struct EulerAngles {
    double roll, pitch, yaw;
};


// Felix & Rainhard
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
    std::vector<float> ranges;  // distances per beam
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

// Felix
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




} // namespace core
