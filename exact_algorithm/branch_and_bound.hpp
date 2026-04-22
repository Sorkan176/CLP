#pragma once

#include <vector>
#include "../structure/data_structure.hpp"

double fractional_knapsack_bound(
        const std::vector<Item>& unused_items,
        int available_volume
);

std::pair<double, std::vector<Placement>> greedy_initial_solution(
        const Space& container_space,
        const std::vector<Item>& items
);

bool check_destination_priority(
        const Placement& current,
        const std::vector<Placement>& placed
);

void branch_and_bound(
        const std::vector<Space>& free_spaces,
        const std::vector<Placement>& placed,
        const std::vector<Item>& unused_items,
        double current_value,
        SolverState& state,
        const Space& container
);

SolverState exact_algorithm(
        const std::vector<std::vector<int>>& data
);