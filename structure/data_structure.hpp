#pragma once

#include <vector>
#include <tuple>

struct Item {
    int id;
    int w;
    int l;
    int d;
    int wt;
    int destination;
    bool turning = true;
    double value = 0.0;

    Item() = default;

    Item(const std::vector<int>& row)
            : id(row[0]),
              w(row[1]),
              l(row[2]),
              d(row[3]),
              wt(row[4]),
              destination(row[5]) {}

    int volume() const {
        return w * l * d;
    }

    std::tuple<int, int, int> oriented_dims(const std::tuple<int,int,int>& orientation) const {
        return orientation;
    }
};


struct Space {
    int x;
    int y;
    int z;
    int w;
    int l;
    int d;
    int payload;

    int volume() const {
        return w * l * d;
    }

    bool contains(const Space& other) const {
        return (other.x >= x && other.y >= y && other.z >= z &&
                other.x + other.w <= x + w &&
                other.y + other.l <= y + l &&
                other.z + other.d <= z + d);
    }

    bool operator==(const Space& other) const
    {
        return x == other.x &&
               y == other.y &&
               z == other.z &&
               w == other.w &&
               l == other.l &&
               d == other.d;
    }
};


struct Placement {
    Item item;
    int x;
    int y;
    int z;
    int w;
    int l;
    int d;
};


struct Orientation
{
    int x;
    int y;
    int z;

    bool operator==(const Orientation& other) const
    {
        return x == other.x &&
               y == other.y &&
               z == other.z;
    }

    bool operator<(const Orientation& other) const
    {
        if (x != other.x) return x < other.x;
        if (y != other.y) return y < other.y;
        return z < other.z;
    }
};

static const std::vector<Orientation> ORIENTATIONS = {
        {0,1,2},
        {1,0,2},
        {0,2,1},
        {1,2,0},
        {2,0,1},
        {2,1,0}
};


struct SolverState {
    double best_value = 0.0;
    std::vector<Placement> best_placements;
    long long nodes = 0;
};