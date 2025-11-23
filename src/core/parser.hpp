#pragma once

#include <string>
#include "geometry.hpp"

// Parst einen Dump im Format
//   ---START---{ ... JSON ... }___END___
// in einen core::LidarScan.
core::LidarScan parseLidarScanFromMsg(const std::string& dump);
