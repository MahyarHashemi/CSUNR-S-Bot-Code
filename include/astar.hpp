#pragma once
#include "main.h"

using Grid = std::vector<std::vector<int>>;
using Coord = std::pair<int,int>;

struct Node {
    int r, c;
    double g, h;

    double f() const { return g + h; }

    bool operator>(const Node& other) const {
        return f() > other.f();
    }
};

double heuristic(int r1, int c1, int r2, int c2);

Grid inflateObstacles(const Grid& grid, int radius);

std::vector<Coord> aStarPlanner(
    const Grid& originalGrid,
    int startR, int startC,
    int goalR,  int goalC,
    int inflationRadius = 0);