#pragma once
#include <string>
#include <vector>


#include "../core/geometry.hpp"
#include "../mapping/mapping.hpp"

#define EXPORT_PATH "../export/"

#define EXPORT_PATH_WORLD EXPORT_PATH "world/"
#define EXPORT_PATH_FRONTIERS EXPORT_PATH "frontiers/"

namespace export_utils {

void export_world_map_csv(const mapping::MapSet& world_map,
                          const std::string& filename);

void export_frontiers_csv(const std::vector<core::Frontier>& frontiers,
                          const std::string& filename);

void export_pose_csv(const core::Pose2D& pose,
                        const std::string& filename);
                        
} // namespace export_utils
