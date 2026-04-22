#include "dynamic_stability.hpp"

#include <cmath>
#include <algorithm>


struct Rect
{
    double x1,y1,x2,y2;
};

static std::vector<Rect> get_support_pieces(
        const Placement& pl,
        const std::vector<Placement>& placed_list)
{
    std::vector<Rect> pieces;

    if (pl.z == 0)
    {
        pieces.push_back({(double)pl.x,(double)pl.y,
                          (double)(pl.x+pl.w),(double)(pl.y+pl.l)});
        return pieces;
    }

    for (const auto& p : placed_list)
    {
        if (p.z + p.d == pl.z)
        {
            double x1 = std::max(pl.x, p.x);
            double y1 = std::max(pl.y, p.y);
            double x2 = std::min(pl.x + pl.w, p.x + p.w);
            double y2 = std::min(pl.y + pl.l, p.y + p.l);

            if (x1 < x2 && y1 < y2)
                pieces.push_back({x1,y1,x2,y2});
        }
    }

    return pieces;
}

static bool support_bounds(
        const Placement& pl,
        const std::vector<Placement>& placed,
        double& xmin,
        double& xmax,
        double& ymin,
        double& ymax)
{
    auto pieces = get_support_pieces(pl, placed);

    if (pieces.empty())
        return false;

    xmin = pieces[0].x1;
    ymin = pieces[0].y1;
    xmax = pieces[0].x2;
    ymax = pieces[0].y2;

    for (auto& p : pieces)
    {
        xmin = std::min(xmin,p.x1);
        ymin = std::min(ymin,p.y1);
        xmax = std::max(xmax,p.x2);
        ymax = std::max(ymax,p.y2);
    }

    return true;
}

bool is_dynamically_stable(
        const std::vector<Placement>& placed,
        int container_wd,
        int container_lth,
        double max_accel,
        double mu)
{
    const double g = 9.81;

    // sliding
    if (max_accel >= mu * g)
        return false;

    for (const auto& pl : placed)
    {
        double h = pl.z + pl.d / 2.0;
        double b = pl.l / 2.0;

        if (max_accel * h > g * b)
            return false;

        if (!is_tilt_stable(pl, placed, container_wd, container_lth))
            return false;
    }

    return true;
}


double calculate_area(const Placement& pl, const Placement& box)
{
    double x1 = std::max(pl.x, box.x);
    double y1 = std::max(pl.y, box.y);
    double x2 = std::min(pl.x + pl.w, box.x + box.w);
    double y2 = std::min(pl.y + pl.l, box.y + box.l);

    if (x1 < x2 && y1 < y2)
        return (x2 - x1) * (y2 - y1);

    return 0.0;
}


std::vector<std::string> check_constrained_sides(
        const Placement& box,
        const std::vector<Placement>& placements,
        int container_wd,
        int container_lth)
{
    std::vector<std::string> unconstrained;

    double support_area = 0.49;

    double box_xz_area = box.w * box.d;
    double box_yz_area = box.l * box.d;

    double side_xz1=0, side_yz1=0, side_xz2=0, side_yz2=0;

    if (box.y == 0) side_xz1 = 1.0;
    if (box.x == 0) side_yz1 = 1.0;
    if (box.y + box.l == container_lth) side_xz2 = 1.0;
    if (box.x + box.w == container_wd) side_yz2 = 1.0;

    for (const auto& pl : placements)
    {
        if (!side_xz1 &&
            pl.y + pl.l == box.y &&
            pl.x < box.x + box.w && pl.x + pl.w > box.x &&
            pl.z < box.z + box.d && pl.z + pl.d > box.z)
        {
            side_xz1 += calculate_area(pl,box);
        }

        if (!side_yz1 &&
            pl.x + pl.w == box.x &&
            pl.y < box.y + box.l && pl.y + pl.l > box.y &&
            pl.z < box.z + box.d && pl.z + pl.d > box.z)
        {
            side_yz1 += calculate_area(pl,box);
        }

        if (!side_xz2 &&
            pl.y == box.y + box.l &&
            pl.x < box.x + box.w && pl.x + pl.w > box.x &&
            pl.z < box.z + box.d && pl.z + pl.d > box.z)
        {
            side_xz2 += calculate_area(pl,box);
        }

        if (!side_yz2 &&
            pl.x == box.x + box.w &&
            pl.y < box.y + box.l && pl.y + pl.l > box.y &&
            pl.z < box.z + box.d && pl.z + pl.d > box.z)
        {
            side_yz2 += calculate_area(pl,box);
        }
    }

    if (!side_xz1 && side_xz1/box_xz_area < support_area)
        unconstrained.push_back("XZ1");

    if (!side_yz1 && side_yz1/box_yz_area < support_area)
        unconstrained.push_back("YZ1");

    if (!side_xz2 && side_xz2/box_xz_area < support_area)
        unconstrained.push_back("XZ2");

    if (!side_yz2 && side_yz2/box_yz_area < support_area)
        unconstrained.push_back("YZ2");

    return unconstrained;
}


bool is_tilt_stable(
        const Placement& pl,
        const std::vector<Placement>& placed,
        int container_wd,
        int container_lth,
        double tipping_angle_deg)
{
    auto unconstrained =
            check_constrained_sides(pl, placed, container_wd, container_lth);

    if (unconstrained.empty())
        return true;

    double xmin,xmax,ymin,ymax;

    if (!support_bounds(pl, placed, xmin,xmax,ymin,ymax))
        return false;

    double cx = pl.x + pl.w/2.0;
    double cy = pl.y + pl.l/2.0;

    double h = pl.d/2.0;

    double tipping_angle =
            tipping_angle_deg * M_PI / 180.0;

    for (const auto& side : unconstrained)
    {
        double w = 0;

        if (side=="YZ1") w = cx-xmin;
        else if (side=="YZ2") w = xmax-cx;
        else if (side=="XZ1") w = cy-ymin;
        else if (side=="XZ2") w = ymax-cy;

        if (w <= 0)
            return false;

        double critical_angle = std::atan(w/h);

        if (critical_angle < tipping_angle)
            return false;
    }

    return true;
}
