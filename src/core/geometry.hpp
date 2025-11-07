#pragma once
#include <cmath>
#include <vector>

namespace core {

// Basic 2D point
struct Point2D {
    float x{0.0f};
    float y{0.0f};
    bool isWall{false};  // true, wenn der Punkt eine Wand darstellt
};

// Robot or LiDAR pose in 2D (position + orientation)
struct Pose2D {
    float x{0.0f};
    float y{0.0f};
    float theta{0.0f};   // orientation in radians
};

// --- Basic helpers ---

// Convert polar coordinates (r, theta) → Cartesian (x, y)
inline Point2D polarToCartesian(float range, float angle) {
    return { range * std::cos(angle), range * std::sin(angle) };
}

// Convert Cartesian (x, y) → polar (r, theta)
inline void cartesianToPolar(const Point2D& p, float& range, float& angle) {
    range = std::sqrt(p.x * p.x + p.y * p.y);
    angle = std::atan2(p.y, p.x);
}

// Apply a pose to a point: from LiDAR frame → world frame
inline Point2D pointInWorldFrame(const Pose2D& lidarPose, const Point2D& pointInLidar) {
    float c = std::cos(lidarPose.theta);
    float s = std::sin(lidarPose.theta);
    return {
        lidarPose.x + c * pointInLidar.x - s * pointInLidar.y,
        lidarPose.y + s * pointInLidar.x + c * pointInLidar.y
    };
}

// Transform from world frame → LiDAR frame (inverse)
inline Point2D pointInLidarFrame(const Pose2D& lidarPose, const Point2D& pointInWorld) {
    float c = std::cos(lidarPose.theta);
    float s = std::sin(lidarPose.theta);
    float dx = pointInWorld.x - lidarPose.x;
    float dy = pointInWorld.y - lidarPose.y;
    return {
        c * dx + s * dy,
       -s * dx + c * dy
    };
}

inline void laserScanToWorldPoints(const std::vector<float>& ranges,
                                   float angle_min, float angle_increment,
                                   float range_min, float range_max,
                                   const Pose2D& lidarPose,
                                   std::vector<Point2D>& outPoints)
{
    outPoints.reserve(outPoints.size() + ranges.size());
    std::vector<Point2D> tempPoints;
    tempPoints.reserve(ranges.size());

    // --- 1. Schritt: lokale Punkte + erste isWall-Bewertung ---
    for (size_t i = 0; i < ranges.size(); ++i) {
        float r = ranges[i];
        if (!(r > range_min && r < range_max)) {
            tempPoints.push_back({0.0f, 0.0f, false}); // vllt auch als frontier kenzeichen?
            continue;
        }

        float angle = angle_min + static_cast<float>(i) * angle_increment;
        Point2D local = polarToCartesian(r, angle);
        local.isWall = true; // erstmal alle gültigen Punkte = Wand
        tempPoints.push_back(local);
    }

    // --- 2. Schritt: Sprungprüfung zwischen aufeinanderfolgenden Punkten ---
    const float jumpThreshold = 5.0f;  // z.B. 5 Meter Sprung = Frontier

    for (size_t i = 1; i < tempPoints.size(); ++i) {
        float prevRange = std::sqrt(tempPoints[i-1].x * tempPoints[i-1].x +
                                    tempPoints[i-1].y * tempPoints[i-1].y);
        float currRange = std::sqrt(tempPoints[i].x * tempPoints[i].x +
                                    tempPoints[i].y * tempPoints[i].y);

        if (std::fabs(currRange - prevRange) > jumpThreshold) {
            // Großer Sprung → beide Punkte keine Wand
            tempPoints[i-1].isWall = false;
            tempPoints[i].isWall   = false;
        }
    }

    // --- 3. Schritt: Transformation in Weltkoordinaten ---
    for (const auto& local : tempPoints) {
        if (!(local.x == 0.0f && local.y == 0.0f)) {
            Point2D world = pointInWorldFrame(lidarPose, local);
            outPoints.push_back(world);
        }
    }
}
