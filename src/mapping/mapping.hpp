#pragma once
#include "../core/geometry.hpp"
#include "mapping.cpp"
#include <vector>

namespace mapping {

void data_to_world(const core::ScanData& scan_data,
                   std::vector<core::Frontier>& scan_frontiers,
                   core::MapSet& world_map,
                   std::vector<core::Frontier>& all_frontiers,
                   float map_res);

}
