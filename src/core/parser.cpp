#include <string>
#include <vector>
#include <stdexcept>
#include <iostream>
#include "picojson.h"
#include "geometry.hpp"



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
