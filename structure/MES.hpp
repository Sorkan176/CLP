#pragma once

#include <vector>
#include <tuple>
#include "data_structure.hpp"


std::pair<std::tuple<int,int,int>, std::tuple<int,int,int>>
coord_placement_min_max(const Placement& pl);

std::pair<std::tuple<int,int,int>, std::tuple<int,int,int>>
coord_space_min_max(const Space& s);

Space make_space(
        const std::tuple<int,int,int>& min_coord,
        const std::tuple<int,int,int>& max_coord
);

std::vector<Space> split_space_mes(
        const Space& space,
        const Placement& placement
);

std::vector<Space> update_free_spaces_mes(
        const std::vector<Space>& free_spaces,
        const Placement& placement
);

bool is_boxes_intersect(
        const std::pair<std::tuple<int,int,int>, std::tuple<int,int,int>>& a,
        const std::pair<std::tuple<int,int,int>, std::tuple<int,int,int>>& b
);

std::vector<Space> remove_contained_spaces(
        const std::vector<Space>& spaces
);

bool fits_in_space(
        const Space& space,
        const Orientation& dims
);

int calculate_total_weight(
        const std::vector<Placement>& placed_items
);

bool placement_overlaps_existing(
        const Placement& pl,
        const std::vector<Placement>& placed_list
);

Space choose_anchor_space(
        std::vector<Space> spaces
);
