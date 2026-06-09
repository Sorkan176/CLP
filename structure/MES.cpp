#include <utility>
#include "MES.hpp"
#include <algorithm>


std::pair<std::tuple<int,int,int>, std::tuple<int,int,int>>
coord_placement_min_max(const Placement& pl)
{
    return {
            {pl.x, pl.y, pl.z},
            {pl.x + pl.w, pl.y + pl.l, pl.z + pl.d}
    };
}

std::pair<std::tuple<int,int,int>, std::tuple<int,int,int>>
coord_space_min_max(const Space& s)
{
    return {
            {s.x, s.y, s.z},
            {s.x + s.w, s.y + s.l, s.z + s.d}
    };
}

Space make_space(
        const std::tuple<int,int,int>& min_coord,
        const std::tuple<int,int,int>& max_coord)
{
    auto [x1,y1,z1] = min_coord;
    auto [x2,y2,z2] = max_coord;

    return Space{
            x1,
            y1,
            z1,
            x2 - x1,
            y2 - y1,
            z2 - z1
    };
}

std::vector<Space> split_space_mes(
        const Space& space,
        const Placement& placement)
{
    std::vector<Space> new_spaces;

    auto [smin, smax] = coord_space_min_max(space);
    auto [pmin, pmax] = coord_placement_min_max(placement);

    auto [sx1,sy1,sz1] = smin;
    auto [sx2,sy2,sz2] = smax;

    auto [px1,py1,pz1] = pmin;
    auto [px2,py2,pz2] = pmax;

    // LEFT
    if (px1 > sx1)
        new_spaces.push_back(make_space({sx1,sy1,sz1},{px1,sy2,sz2}));

    // RIGHT
    if (px2 < sx2)
        new_spaces.push_back(make_space({px2,sy1,sz1},{sx2,sy2,sz2}));

    // BACK
    if (py1 > sy1)
        new_spaces.push_back(make_space({sx1,sy1,sz1},{sx2,py1,sz2}));

    // FRONT
    if (py2 < sy2)
        new_spaces.push_back(make_space({sx1,py2,sz1},{sx2,sy2,sz2}));

    // BOTTOM
    if (pz1 > sz1)
        new_spaces.push_back(make_space({sx1,sy1,sz1},{sx2,sy2,pz1}));

    // TOP
    if (pz2 < sz2)
        new_spaces.push_back(make_space({sx1,sy1,pz2},{sx2,sy2,sz2}));

    std::vector<Space> filtered;

    for (auto& s : new_spaces)
        if (s.w > 0 && s.l > 0 && s.d > 0)
            filtered.push_back(s);

    return filtered;
}

bool is_boxes_intersect(
        const std::pair<std::tuple<int,int,int>, std::tuple<int,int,int>>& a,
        const std::pair<std::tuple<int,int,int>, std::tuple<int,int,int>>& b)
{
    auto [a_min,a_max] = a;
    auto [b_min,b_max] = b;

    auto [ax1,ay1,az1] = a_min;
    auto [ax2,ay2,az2] = a_max;

    auto [bx1,by1,bz1] = b_min;
    auto [bx2,by2,bz2] = b_max;

    int ix_min = std::max(ax1,bx1);
    int iy_min = std::max(ay1,by1);
    int iz_min = std::max(az1,bz1);

    int ix_max = std::min(ax2,bx2);
    int iy_max = std::min(ay2,by2);
    int iz_max = std::min(az2,bz2);

    return !(ix_min >= ix_max ||
             iy_min >= iy_max ||
             iz_min >= iz_max);
}

std::vector<Space> remove_contained_spaces(const std::vector<Space>& spaces)
{
    std::vector<Space> result;

    for (size_t i=0;i<spaces.size();i++)
    {
        bool contained = false;

        for (size_t j=0;j<spaces.size();j++)
        {
            if (i!=j && spaces[j].contains(spaces[i]))
            {
                contained = true;
                break;
            }
        }

        if (!contained)
            result.push_back(spaces[i]);
    }

    return result;
}


std::vector<Space> update_free_spaces_mes(
        const std::vector<Space>& free_spaces,
        const Placement& pl)
{
    std::vector<Space> new_free;
    new_free.reserve(free_spaces.size() * 2);

    const int px1 = pl.x;
    const int py1 = pl.y;
    const int pz1 = pl.z;
    const int px2 = pl.x + pl.w;
    const int py2 = pl.y + pl.l;
    const int pz2 = pl.z + pl.d;

    for (const auto& s : free_spaces)
    {
        const int sx1 = s.x;
        const int sy1 = s.y;
        const int sz1 = s.z;
        const int sx2 = s.x + s.w;
        const int sy2 = s.y + s.l;
        const int sz2 = s.z + s.d;

        // no overlap
        if (px2 <= sx1 || px1 >= sx2 ||
            py2 <= sy1 || py1 >= sy2 ||
            pz2 <= sz1 || pz1 >= sz2)
        {
            new_free.push_back(s);
            continue;
        }

        auto split = split_space_mes(s, pl);

        // avoid insert
        for (auto& sp : split)
            new_free.push_back(sp);
    }

    return remove_contained_spaces(new_free);
}

bool fits_in_space(
        const Space& space,
        int dx,
        int dy,
        int dz)
{
    return dx <= space.w &&
           dy <= space.l &&
           dz <= space.d;
}

int calculate_total_weight(const std::vector<Item>& items, const std::vector<Placement>& placed_items)
{
    int total = 0;

    for (const auto& pl : placed_items)
        total += items[pl.item_index].wt;

    return total;
}

bool placement_overlaps_existing(
        const Placement& pl,
        const std::vector<Placement>& placed_list)
{
    const int ax1 = pl.x;
    const int ay1 = pl.y;
    const int az1 = pl.z;
    const int ax2 = pl.x + pl.w;
    const int ay2 = pl.y + pl.l;
    const int az2 = pl.z + pl.d;

    for (const auto& p : placed_list)
    {
        const int bx1 = p.x;
        const int by1 = p.y;
        const int bz1 = p.z;
        const int bx2 = p.x + p.w;
        const int by2 = p.y + p.l;
        const int bz2 = p.z + p.d;

        if (!(ax2 <= bx1 || ax1 >= bx2 ||
              ay2 <= by1 || ay1 >= by2 ||
              az2 <= bz1 || az1 >= bz2))
        {
            return true;
        }
    }

    return false;
}

Space choose_anchor_space(const std::vector<Space>& spaces)
{
    auto it = std::min_element(
            spaces.begin(), spaces.end(),
            [](const Space& a, const Space& b)
            {
                if (a.z != b.z) return a.z < b.z;
                if (a.y != b.y) return a.y < b.y;
                return a.x < b.x;
            });

    return *it;
}
