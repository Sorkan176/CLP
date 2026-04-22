#include <random>
#include <algorithm>
#include <iostream>
#include <chrono>
#include "grasp.hpp"
#include "../structure/MES.hpp"
#include "../stability/static_stability.hpp"
#include "../stability/dynamic_stability.hpp"

std::vector<std::pair<Placement,double>> generate_feasible_placements(
        const Space& space,
        const std::vector<Item>& items,
        const std::vector<Placement>& placed,
        double min_support)
{
    std::vector<std::pair<Placement,double>> candidates;

    for (const auto& it : items)
    {
        int dims_arr[3] = {it.w, it.l, it.d};

        for (const auto& orient : ORIENTATIONS)
        {
            Orientation dims{
                    dims_arr[orient.x],
                    dims_arr[orient.y],
                    dims_arr[orient.z]
            };

            if (!fits_in_space(space, dims))
                continue;

            Placement pl{
                    it,
                    space.x,
                    space.y,
                    space.z,
                    dims.x,
                    dims.y,
                    dims.z
            };

            if (placement_overlaps_existing(pl, placed))
                continue;

            double base_area = dims.x * dims.y;
            double sup = support_area(pl, placed);

            if (sup < min_support * base_area)
                continue;

            candidates.emplace_back(pl, sup);
        }
    }

    return candidates;
}

double placement_score(
        const Placement& pl,
        double support,
        double alpha,
        double beta,
        double gamma)
{
    double volume = pl.w * pl.l * pl.d;
    double base = pl.w * pl.l;

    double stability_score = support / base;

    return alpha * volume +
           beta * support +
           gamma * stability_score * volume;
}

std::vector<Placement> build_rcl(
        const std::vector<std::pair<Placement,double>>& candidates,
        double alpha)
{
    if (candidates.empty())
        return {};

    std::vector<std::pair<Placement,double>> scored;

    for (auto& c : candidates)
    {
        double sc = placement_score(c.first, c.second);
        scored.emplace_back(c.first, sc);
    }

    double f_max = scored.front().second;
    double f_min = scored.front().second;

    for (auto& s : scored)
    {
        f_max = std::max(f_max, s.second);
        f_min = std::min(f_min, s.second);
    }

    double threshold = f_max - alpha * (f_max - f_min);

    std::vector<Placement> rcl;

    for (auto& s : scored)
        if (s.second >= threshold)
            rcl.push_back(s.first);

    return rcl;
}

Placement select_block_randomly(const std::vector<Placement>& rcl)
{
    static std::mt19937 rng(std::random_device{}());

    std::uniform_int_distribution<int> dist(0, rcl.size()-1);

    return rcl[dist(rng)];
}

std::map<int, std::vector<Item>> group_items_by_destination(
        const std::vector<Item>& items)
{
    std::map<int,std::vector<Item>> groups;

    for (const auto& it : items)
        groups[it.destination].push_back(it);

    return groups;
}

Placement choose_best_candidate(
        const std::vector<std::pair<Placement,double>>& candidates)
{
    Placement best{};
    double best_score = -std::numeric_limits<double>::infinity();

    for (const auto& [pl, sup] : candidates)
    {
        double volume = pl.w * pl.l * pl.d;
        double base = pl.w * pl.l;

        double support_ratio = (base > 0) ? sup / base : 0;

        double score =
                volume +
                0.5 * sup +
                10.0 * support_ratio;

        if (score > best_score)
        {
            best_score = score;
            best = pl;
        }
    }

    return best;
}

std::vector<Placement> check_dynamic_stability(
        const std::vector<Placement>& placements,
        const Space& container,
        double max_accel,
        double mu)
{
    const double g = 9.81;

    if (max_accel >= mu * g)
    {
        std::cerr << "Any kind of loading will be unstable due to low mu coefficient or high acceleration\n";
        std::exit(1);
    }

    std::vector<Placement> sorted_boxes = placements;

    std::sort(sorted_boxes.begin(), sorted_boxes.end(),
              [](const Placement& a, const Placement& b)
              {
                  return (a.z + a.d) > (b.z + b.d);
              });

    std::vector<Placement> unstable;

    for (const auto& pl : sorted_boxes)
    {
        double h = pl.z + pl.d / 2.0;
        double b = pl.l / 2.0;

        if (max_accel * h > g * b)
        {
            unstable.push_back(pl);
            continue;
        }

        if (!is_tilt_stable(pl, sorted_boxes, container.w, container.l))
        {
            unstable.push_back(pl);
        }
    }

    return unstable;
}

std::vector<Placement> construct_packing(
        const std::vector<Item>& items,
        const Space& container)
{
    std::vector<Space> free_spaces{container};
    std::vector<Placement> placed;

    auto groups = group_items_by_destination(items);

    std::mt19937 rng(std::random_device{}());

    for (auto& [dest, group] : groups)
    {
        auto remaining = group;

        while (!free_spaces.empty() && !remaining.empty())
        {
            Space space = choose_anchor_space(free_spaces);

            auto candidates =
                    generate_feasible_placements(space, remaining, placed);

            if (candidates.empty())
            {
                free_spaces.erase(
                        std::remove(free_spaces.begin(),
                                    free_spaces.end(),
                                    space),
                        free_spaces.end());
                continue;
            }

            std::uniform_real_distribution<double> dist(0.1,0.45);
            double alpha = dist(rng);

            auto rcl = build_rcl(candidates, alpha);

            Placement chosen = select_block_randomly(rcl);

            placed.push_back(chosen);

            remaining.erase(
                    std::remove_if(
                            remaining.begin(),
                            remaining.end(),
                            [&](const Item& i){ return i.id == chosen.item.id; }),
                    remaining.end());

            free_spaces = update_free_spaces_mes(free_spaces, chosen);
        }
    }

    return placed;
}

std::vector<Placement> improvement_phase(
        const std::vector<Placement>& placements,
        const std::vector<Placement>& unstable,
        const Space& container,
        double remove_ratio)
{
    if (placements.empty())
        return placements;

    std::vector<std::pair<double,Placement>> scored;

    for (const auto& pl : placements)
    {
        double sup = support_area(pl, placements);
        double base = pl.w * pl.l;
        double ratio = (base > 0) ? sup / base : 0;

        scored.emplace_back(ratio, pl);
    }

    std::sort(scored.begin(), scored.end(),
              [](auto& a, auto& b){ return a.first < b.first; });

    int k = std::max(1, (int)(placements.size() * remove_ratio));

    std::vector<Placement> to_remove;

    for (int i = 0; i < k && i < scored.size(); i++)
        to_remove.push_back(scored[i].second);

    for (const auto& u : unstable)
    {
        auto it = std::find_if(
                to_remove.begin(), to_remove.end(),
                [&](const Placement& p){ return p.item.id == u.item.id; });

        if (it == to_remove.end())
            to_remove.push_back(u);
    }

    std::vector<Placement> remaining;

    for (const auto& pl : placements)
    {
        bool removed = false;

        for (const auto& r : to_remove)
            if (pl.item.id == r.item.id)
                removed = true;

        if (!removed)
            remaining.push_back(pl);
    }

    std::vector<Space> free_spaces{container};

    for (const auto& pl : remaining)
        free_spaces = update_free_spaces_mes(free_spaces, pl);

    std::vector<Item> removed_items;

    for (const auto& pl : to_remove)
        removed_items.push_back(pl.item);

    std::vector<Placement> placed = remaining;

    auto groups = group_items_by_destination(removed_items);

    for (auto& [dest, group] : groups)
    {
        auto items = group;

        while (!free_spaces.empty() && !items.empty())
        {
            Space space = choose_anchor_space(free_spaces);

            auto candidates =
                    generate_feasible_placements(space, items, placed);

            if (candidates.empty())
            {
                free_spaces.erase(
                        std::remove(free_spaces.begin(),
                                    free_spaces.end(),
                                    space),
                        free_spaces.end());
                continue;
            }

            Placement best = choose_best_candidate(candidates);

            placed.push_back(best);

            items.erase(
                    std::remove_if(items.begin(), items.end(),
                                   [&](const Item& i)
                                   { return i.id == best.item.id; }),
                    items.end());

            free_spaces = update_free_spaces_mes(free_spaces, best);
        }
    }

    auto new_unstable = check_dynamic_stability(placed, container);

    std::vector<Placement> final;

    for (const auto& pl : placed)
    {
        bool bad = false;

        for (const auto& u : new_unstable)
            if (pl.item.id == u.item.id)
                bad = true;

        if (!bad)
            final.push_back(pl);
    }

    return final;
}

SolverState hybrid_container_loading(
        const std::vector<Item>& items,
        const Space& container,
        int iterations = 150)
{
    SolverState state;

    std::mt19937 rng(std::random_device{}());

    for (int it = 0; it < iterations; it++)
    {
        std::vector<Item> shuffled = items;

        std::shuffle(shuffled.begin(), shuffled.end(), rng);

        auto placements =
                construct_packing(shuffled, container);

        auto unstable =
                check_dynamic_stability(placements, container);

        placements =
                improvement_phase(
                        placements,
                        unstable,
                        container
                );

        double value = 0;

        for (const auto& pl : placements)
            value += pl.item.volume();

        if (value > state.best_value)
        {
            state.best_value = value;
            state.best_placements = placements;
        }
    }

    return state;
}

SolverState grasp(const std::vector<std::vector<int>>& data)
{
    Space container{
            0,0,0,
            data[0][0],
            data[0][1],
            data[0][2]
    };

    std::vector<Item> items;

    for (size_t i=1;i<data.size();i++)
        items.emplace_back(data[i]);

    auto start = std::chrono::high_resolution_clock::now();

    auto state = hybrid_container_loading(items, container, 3000);

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end - start;

//    std::cout << "Items count " << data.size()-1 << "\n";
//    std::cout << "Best value " << std::to_string(state.best_value) << "\n";
//    std::cout << "Placement " << state.best_placements.size()<< "\n";

    int total_items = items.size();
    int placed_items = state.best_placements.size();

    long long total_items_volume = 0;
    for (const auto& item : items)
        total_items_volume += item.volume();

    long long placed_items_volume = 0;
    for (const auto& pl : state.best_placements)
        placed_items_volume += pl.w * pl.l * pl.d;

    long long container_volume = container.volume();

    double item_ratio = double(placed_items) / double(total_items);
    double volume_ratio = double(placed_items_volume) / double(container_volume);
    double placed_volume_percent =
            100.0 * static_cast<double>(placed_items_volume) /
            static_cast<double>(total_items_volume);


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