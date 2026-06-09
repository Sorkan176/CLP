#include <algorithm>
#include <set>
#include <iostream>
#include <chrono>
#include "branch_and_bound.hpp"
#include "../structure/MES.hpp"
#include "../stability/static_stability.hpp"
#include "../stability/dynamic_stability.hpp"
#include "../heuristic_algorithm/grasp.hpp"

double fractional_knapsack_bound(
        const std::vector<Item>& unused_items,
        int available_volume)
{
    std::vector<Item> items = unused_items;

    std::sort(items.begin(), items.end(),
              [](const Item& a, const Item& b)
              {
                  double va = (a.value > 0) ? a.value : a.volume;
                  double vb = (b.value > 0) ? b.value : b.volume;
                  return va > vb;
              });

    int rem = available_volume;
    double bound_value = 0.0;

    for (const auto& it : items)
    {
        double v = (it.value > 0) ? it.value : it.volume;
        int vol = it.volume;

        if (vol <= rem)
        {
            bound_value += v;
            rem -= vol;
        }
        else
        {
            if (vol > 0 && rem > 0)
            {
                bound_value += v * ((double)rem / vol);
                rem = 0;
            }
            break;
        }
    }

    return bound_value;
}

bool check_destination_priority(
        const std::vector<Item>& items,
        const Placement& current,
        const std::vector<Placement>& placed)
{
    for (const auto& pl : placed)
    {
        if (current.z == pl.z + pl.d &&
            items[current.item_index].destination < items[pl.item_index].destination)
        {
            auto [cur_min, cur_max] = coord_placement_min_max(current);
            auto [pl_min, pl_max] = coord_placement_min_max(pl);

            int ix_min = std::max(std::get<0>(cur_min), std::get<0>(pl_min));
            int iy_min = std::max(std::get<1>(cur_min), std::get<1>(pl_min));

            int ix_max = std::min(std::get<0>(cur_min), std::get<0>(pl_min));
            int iy_max = std::min(std::get<1>(cur_min), std::get<1>(pl_min));

            bool no_overlap =
                    (ix_min >= ix_max) ||
                    (iy_min >= iy_max);

            return !no_overlap;
        }
    }

    return false;
}

void branch_and_bound(
        const std::vector<Item>& items,
        const std::vector<Space>& free_spaces,
        const std::vector<Placement>& placed,
        const std::vector<Item>& unused_items,
        double current_value,
        SolverState& state,
        const Space& container)
{
    state.nodes++;

    if (state.best_placements.size() == (placed.size() + unused_items.size())) {
        return;
    }

    int avail_vol = 0;
    for (auto& s : free_spaces)
        avail_vol += s.volume();

    double bound = current_value +
                   fractional_knapsack_bound(unused_items, avail_vol);

    if (bound <= state.best_value + 1e-9)
        return;

    if (free_spaces.empty() || unused_items.empty())
    {
        if (is_dynamically_stable(placed, container.w, container.l))
        {
            if (current_value > state.best_value)
            {
                state.best_value = current_value;
                state.best_placements = placed;
            }
        }
        return;
    }

    Space anchor = choose_anchor_space(free_spaces);

    struct Candidate
    {
        Item item;
        Orientation dims;
        double value;
    };

    std::vector<Candidate> candidates;

    for (const auto& it : unused_items)
    {
        std::set<Orientation> unique;

        auto orientations = it.turning ? ORIENTATIONS
                                       : std::vector<Orientation>(ORIENTATIONS.begin(),
                                                                  ORIENTATIONS.begin()+2);

        for (auto& orient : orientations)
        {
            int dims_arr[3] = {it.w, it.l, it.d};

            Orientation dims{
                    dims_arr[orient.x],
                    dims_arr[orient.y],
                    dims_arr[orient.z]
            };

            if (unique.count(dims))
                continue;

            unique.insert(dims);

            if (fits_in_space(anchor, dims.x, dims.y, dims.z))
            {
                double v = (it.value > 0) ? it.value : it.volume;
                candidates.push_back({it,dims,v});
            }
        }
    }

    /*std::sort(candidates.begin(), candidates.end(),
              [](const Candidate& a, const Candidate& b)
              {
                  return a.item.destination < b.item.destination;
              });*/

    std::sort(candidates.begin(), candidates.end(),
                      [](auto&a, auto&b){
                          return a.item.volume > b.item.volume;
                      });

    for (auto& c : candidates)
    {
        Placement pl{
                c.item.id,
                anchor.x, anchor.y, anchor.z,
                c.dims.x, c.dims.y, c.dims.z
        };

//        if (check_destination_priority(pl, placed))
//            continue;

        if (calculate_total_weight(items,placed) + c.item.wt > container.payload)
            continue;

        if (support_area(pl, placed) / (double)(pl.w*pl.l) < 0.70)
            continue;

        if (placement_overlaps_existing(pl, placed))
            continue;

        auto new_free = update_free_spaces_mes(free_spaces, pl);

        auto new_placed = placed;
        new_placed.push_back(pl);

        std::vector<Item> new_unused;
        for (auto& u : unused_items)
            if (u.id != c.item.id)
                new_unused.push_back(u);

        double new_value = current_value + c.value;

        if (new_value > state.best_value)
        {
            state.best_value = new_value;
            state.best_placements = new_placed;
        }

        branch_and_bound(
                items,
                new_free,
                new_placed,
                new_unused,
                new_value,
                state,
                container
        );
    }

    std::vector<Space> new_free2;

    for (auto& s : free_spaces)
        if (!(s == anchor))
            new_free2.push_back(s);

    branch_and_bound(
            items,
            new_free2,
            placed,
            unused_items,
            current_value,
            state,
            container
    );
}

SolverState exact_algorithm(
        const std::vector<std::vector<int>>& data)
{
    auto container_data = data[0];

    Space container{
            0,0,0,
            container_data[0],
            container_data[1],
            container_data[2],
            container_data[3]
    };

    std::vector<Item> items;

    for (size_t i=1;i<data.size();i++)
    {
        items.emplace_back(data[i]);
    }

    for (auto& it : items)
        if (it.value <= 0)
            it.value = it.volume;

    // auto greedy = greedy_initial_solution(container, items);
    SolverState state = grasp(data);

//    SolverState state;
//    state.best_value = greedy.first;
//    state.best_placements = greedy.second;

    std::cout << "Greedy: ";
    std::cout << "Best value " << std::to_string(state.best_value) << "   ";
    std::cout << "Placement " << state.best_placements.size()<< "\n";

    auto start = std::chrono::high_resolution_clock::now();

    branch_and_bound(
            items,
            {container},
            {},
            items,
            0.0,
            state,
            container
    );

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end - start;

    int total_items = items.size();
    int placed_items = state.best_placements.size();

    long long total_items_volume = 0;
    for (const auto& item : items)
        total_items_volume += item.volume;

    long long placed_items_volume = 0;
    for (const auto& pl : state.best_placements)
        placed_items_volume += pl.w * pl.l * pl.d;

    long long container_volume = container.volume();

    double item_ratio = double(placed_items) / double(total_items);
    double volume_ratio = double(placed_items_volume) / double(container_volume);
    double placed_volume_percent =
            100.0 * static_cast<double>(placed_items_volume) /
            static_cast<double>(total_items_volume);

    std::cout << "B&B\n";
    std::cout << "Total items:          " << total_items << "\n";
    std::cout << "Placed items:         " << placed_items
              << " (" << item_ratio * 100.0 << "%)\n";

    std::cout << "\nContainer volume:     " << container_volume << "\n";
    std::cout << "Items volume (total): " << total_items_volume << "\n";
    std::cout << "Packed volume:        " << placed_items_volume
              << " (" << volume_ratio * 100.0 << "% of container)"
              << " (" << placed_volume_percent << "% of items)\n";

    std::cout << "\nExecution time:       " << elapsed.count() << " seconds\n";

    return state;
}