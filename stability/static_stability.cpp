#include "static_stability.hpp"

#include <algorithm>


struct SupportRect
{
    int x1;
    int y1;
    int x2;
    int y2;
};

double support_area(
        const Placement& pl,
        const std::vector<Placement>& placed_list)
{
    std::vector<SupportRect> support_pieces;

    if (pl.z == 0)
    {
        // box stands on the container floor
        support_pieces.push_back({
        pl.x,
        pl.y,
        pl.x + pl.w,
        pl.y + pl.l
        });
    }
    else
    {
        for (const auto& p : placed_list)
        {
            // check if top face of p matches bottom face of pl
            if (p.z + p.d == pl.z)
            {
                int ix1 = std::max(pl.x, p.x);
                int iy1 = std::max(pl.y, p.y);
                int ix2 = std::min(pl.x + pl.w, p.x + p.w);
                int iy2 = std::min(pl.y + pl.l, p.y + p.l);

                if (ix1 < ix2 && iy1 < iy2)
                {
                    support_pieces.push_back({ix1, iy1, ix2, iy2});
                }
            }
        }
    }

    double total_support = 0.0;

    for (const auto& r : support_pieces)
    {
        total_support += (r.x2 - r.x1) * (r.y2 - r.y1);
    }

    return total_support;
}