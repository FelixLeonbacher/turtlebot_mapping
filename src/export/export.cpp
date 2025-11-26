#include "export.hpp"
#include <fstream>
#include <iostream>
#include "../mapping/mapping.hpp"

namespace export_utils {

    // world_map: unordered_set<Point2D, ...>
    void export_world_map_csv(const mapping::MapSet& world_map,
                            const std::string& filename)
    {
        std::ofstream ofs(filename);
        if (!ofs) {
            std::cerr << "Cannot open file " << filename << " for writing\n";
            return;
        }

        // Header
        ofs << "x,y,is_wall\n";

        // Daten
        for (const auto& p : world_map) {
            ofs << p.x << "," << p.y << "," << (p.is_wall ? 1 : 0) << "\n";
        }
    }

    // frontiers: vector<Frontier>
    void export_frontiers_csv(const std::vector<core::Frontier>& frontiers,
                            const std::string& filename)
    {
        std::ofstream ofs(filename);
        if (!ofs) {
            std::cerr << "Cannot open file " << filename << " for writing\n";
            return;
        }

        // Header
        ofs << "ax,ay,bx,by,mx,my,width\n";

        for (const auto& f : frontiers) {
            ofs << f.a.x << "," << f.a.y << ","
                << f.b.x << "," << f.b.y << ","
                << f.m.x << "," << f.m.y << ","
                << f.width << "\n";
        }
    }



    void export_pose_csv(const core::Pose2D& pose,
                        const std::string& filename)
    {
        std::ofstream ofs(filename, std::ios::app);
        if (!ofs) {
            std::cerr << "Cannot open file " << filename << " for writing\n";
            return;
        }

        // Check if file is empty
        ofs.seekp(0, std::ios::end);
        if (ofs.tellp() == 0) {
            ofs << "x,y,theta\n";
        }

        // Append pose
        ofs << pose.x << "," << pose.y << "," << pose.theta << "\n";
    }

} // namespace export_utils
