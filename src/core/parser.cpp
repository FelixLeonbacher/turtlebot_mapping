/**
 * @file parser.cpp
 * @brief message parsers for LidarScan and Odometry
 * @author Victor Swekis, Philipp Riegler
 * @version 1.0
 */

#include <string>
#include <vector>
#include <stdexcept>
#include <iostream>
#include "picojson.h"
#include "geometry.hpp"
#include <regex>         // Für std::regex zum Extrahieren der Zahlen
#include <cmath>

// build errors without this
#define M_PI 3.14159265358979323846



// Philipp 
core::LidarScan parseLidarScanFromMsg(const std::string& dump)
{
    // ---START--- und ___END___ rausfiltern
    const std::string startTag = "---START---";
    const std::string endTag   = "___END___";

    const size_t startPos = dump.find(startTag);
    const size_t endPos   = dump.find(endTag);

    if (startPos == std::string::npos || endPos == std::string::npos || endPos <= startPos) {
        throw std::runtime_error("parseLidarScanFromMsg: START/END not found");
    }

    const size_t jsonStart = startPos + startTag.size();
    const size_t jsonLen   = endPos - jsonStart;
    const std::string jsonText = dump.substr(jsonStart, jsonLen);

    // ---- JSON mit picojson parsen ----
    picojson::value v;
    std::string err = picojson::parse(v, jsonText);

    if (!err.empty()) {
        throw std::runtime_error("JSON parse error: " + err);
    }

    if (!v.is<picojson::object>()) {
        throw std::runtime_error("JSON is not an object");
    }

    picojson::object& j = v.get<picojson::object>();

    core::LidarScan scan{};

    // floats lesen
    scan.angle_min = static_cast<float>(j["angle_min"].get<double>());
    scan.angle_inc = static_cast<float>(j["angle_increment"].get<double>());
    scan.range_min = static_cast<float>(j["range_min"].get<double>());
    scan.range_max = static_cast<float>(j["range_max"].get<double>());

    // ranges-Array
    picojson::array& arr = j["ranges"].get<picojson::array>();
    scan.ranges.reserve(arr.size());

    for (auto& e : arr) {
        scan.ranges.push_back(static_cast<float>(e.get<double>()));
    }

    // Pose später
    scan.pose = core::Pose2D{0.0f, 0.0f, 0.0f};

    return scan;
}




// Victor

// Quaternion -> Euler (roll, pitch, yaw)
core::EulerAngles ToEulerAngles(core::Quaternion q) {
    core::EulerAngles angles{};

    double sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z);
    double cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y);
    angles.roll = std::atan2(sinr_cosp, cosr_cosp);

    double sinp = std::sqrt(1.0 + 2.0 * (q.w * q.y - q.x * q.z));
    double cosp = std::sqrt(1.0 - 2.0 * (q.w * q.y - q.x * q.z));
    angles.pitch = 2.0 * std::atan2(sinp, cosp) - M_PI / 2.0;

    double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    angles.yaw = std::atan2(siny_cosp, cosy_cosp);

    return angles;
}

// Extract "orientation": {"x": ..., "y": ..., "z": ..., "w": ...}
bool extractOrientation(const std::string& input,
                        double& x, double& y, double& z, double& w)
{
    std::regex re(
        "\"orientation\"\\s*:\\s*\\{\\s*"
        "\"x\"\\s*:\\s*([-0-9\\.eE]+)\\s*,\\s*"
        "\"y\"\\s*:\\s*([-0-9\\.eE]+)\\s*,\\s*"
        "\"z\"\\s*:\\s*([-0-9\\.eE]+)\\s*,\\s*"
        "\"w\"\\s*:\\s*([-0-9\\.eE]+)"
    );

    std::smatch match;

    if (std::regex_search(input, match, re) && match.size() == 5) {
        x = std::stod(match[1]);
        y = std::stod(match[2]);
        z = std::stod(match[3]);
        w = std::stod(match[4]);
        return true;
    }
    return false;
}

// Extract "position": {"x": ..., "y": ..., "z": ...}
bool extractPositionXY(const std::string& input,
                       double& x, double& y)
{
    std::regex re(
        "\"position\"\\s*:\\s*\\{\\s*"
        "\"x\"\\s*:\\s*([-0-9\\.eE]+)\\s*,\\s*"
        "\"y\"\\s*:\\s*([-0-9\\.eE]+)"
    );

    std::smatch match;

    if (std::regex_search(input, match, re) && match.size() == 3) {
        x = std::stod(match[1]);
        y = std::stod(match[2]);
        return true;
    }
    return false;
}

bool parseOdomToPose2D(const std::string& msg, core::Pose2D& pose)
{
    // 1) Extract quaternion
    double qx, qy, qz, qw;
    if (!extractOrientation(msg, qx, qy, qz, qw)) {
        return false; // orientation block not found
    }

    core::Quaternion q{qw, qx, qy, qz};
    core::EulerAngles e = ToEulerAngles(q);

    // 2) Extract position x,y
    double px, py;
    if (!extractPositionXY(msg, px, py)) {
        return false; // position block not found
    }

    // 3) Fill Pose2D (convert to float)
    pose.x     = static_cast<float>(px);
    pose.y     = static_cast<float>(py);
    pose.theta = static_cast<float>(e.yaw);  // yaw in radians

    return true;
}


