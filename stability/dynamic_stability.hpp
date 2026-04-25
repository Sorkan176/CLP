#pragma once

#include <vector>
#include <string>
#include "../structure/data_structure.hpp"

constexpr double MAX_ACCELERATION = 1.5;
constexpr double MU = 0.35;
constexpr double g = 9.81;

bool is_dynamically_stable(
        const std::vector<Placement>& placed,
        int container_wd,
        int container_lth,
        double max_accel = MAX_ACCELERATION,
        double mu = MU
);

std::vector<std::string> check_constrained_sides(
        const Placement& box,
        const std::vector<Placement>& placements,
        int container_wd,
        int container_lth
);

double calculate_area(
        const Placement& pl,
        const Placement& box
);

bool is_tilt_stable(
        const Placement& pl,
        const std::vector<Placement>& placed,
        int container_wd,
        int container_lth,
        double max_accel = MAX_ACCELERATION,
        double tipping_angle_rad = 0.158
);