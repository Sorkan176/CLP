#pragma once

#include <map>
#include "../structure/data_structure.hpp"

std::vector<std::pair<Placement,double>> generate_feasible_placements(
        const Space& space,
        const std::vector<Item>& items,
        const std::vector<Placement>& placed,
        double min_support = 0.70);

double placement_score(
        const Placement& pl,
        double support,
        double alpha = 1.0,
        double beta = 0.1,
        double gamma = 0.05);

std::vector<Placement> build_rcl(
        const std::vector<std::pair<Placement,double>>& candidates,
        double alpha);

Placement select_block_randomly(const std::vector<Placement>& rcl);

std::map<int, std::vector<Item>> group_items_by_destination(
        const std::vector<Item>& items);

Placement choose_best_candidate(
        const std::vector<std::pair<Placement,double>>& candidates);

std::vector<Placement> check_dynamic_stability(
        const std::vector<Placement>& placements,
        const Space& container,
        double max_accel = 1.5,
        double mu = 0.35);

std::vector<Placement> construct_packing(
        const std::vector<Item>& items,
        const Space& container);

std::vector<Placement> improvement_phase(
        const std::vector<Placement>& placements,
        const std::vector<Placement>& unstable,
        const Space& container,
        double remove_ratio = 0.18);

SolverState grasp(const std::vector<std::vector<int>>& data);

