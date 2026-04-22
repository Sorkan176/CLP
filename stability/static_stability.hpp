#pragma once

#include <vector>
#include "../structure/data_structure.hpp"

double support_area(
        const Placement& pl,
        const std::vector<Placement>& placed_list
);
