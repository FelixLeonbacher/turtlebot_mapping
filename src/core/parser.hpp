#pragma once

#include <string>
#include "geometry.hpp"
#include "parser.cpp"

// Parst einen Dump im Format
//   ---START---{ ... JSON ... }___END___
// in einen core::LidarScan.
core::LidarScan parseLidarScanFromDump(const std::string& dump);
