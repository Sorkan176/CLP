#include <fstream>
#include "visualize_3d.hpp"

void save_solution_csv(
        const std::vector<Placement>& placements,
        const std::string& filename)
{
    std::ofstream file("../visualization/" + filename);

    file << "id,x,y,z,w,h,d\n";

    for (const auto& p : placements)
    {
        file << p.item.id << ","
             << p.x << ","
             << p.y << ","
             << p.z << ","
             << p.w << ","
             << p.l << ","
             << p.d << "\n";
    }
}