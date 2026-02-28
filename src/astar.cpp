#include "main.h"
#include "EZ-Template/drive/drive.hpp"
#include <iostream>
#include <vector>

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

double heuristic(int r1, int c1, int r2, int c2){
    return std::sqrt((r1 - r2)*(r1 - r2) + (c1 - c2)*(c1 - c2));
}

Grid inflateObstacles(const Grid& grid, int radius){
    Grid inflated = grid;

    int rows = grid.size();
    int cols = grid[0].size();

    for (int r = 0; r < rows; ++r){
        for (int c = 0; c < cols; ++c){
            if (grid[r][c] == 1){
                for (int dr = -radius; dr <= radius; ++dr){
                    for (int dc = -radius; dc <= radius; ++dc){
                        int nr = r + dr;
                        int nc = c + dc;

                        if ((nr >= 0 && nr < rows) && (nc >= 0 && nc < cols)){
                            inflated[nr][nc] = 1;
                        }
                    }
                }
            }
        }
    }
    return inflated;
}

std::vector<Coord> aStarPlanner(
    const Grid& originalGrid,
    int startR, int startC,
    int goalR,  int goalC,
    int inflationRadius = 0){

    Grid grid = inflateObstacles(originalGrid, inflationRadius);

    int rows = grid.size();
    int cols = grid[0].size();

    std::priority_queue<Node, std::vector<Node>, std::greater<Node>> open;

    std::vector<std::vector<double>> gCost(rows, std::vector<double>(cols, 1e9));
    std::vector<std::vector<bool>> closed(rows, std::vector<bool>(cols, false));
    std::vector<std::vector<Coord>> parent(rows, std::vector<Coord>(cols, {-1,-1}));

    gCost[startR][startC] = 0;

    open.push({startR, startC, 0, heuristic(startR,startC,goalR,goalC)});

    // 8-connected motion
    std::vector<Coord> directions = {
        {1,0},{-1,0},{0,1},{0,-1},
        {1,1},{1,-1},{-1,1},{-1,-1}
    };

    while (!open.empty()) {
        Node current = open.top();
        open.pop();

        int r = current.r;
        int c = current.c;

        if (closed[r][c]) continue;
        closed[r][c] = true;

        if (r == goalR && c == goalC) {
            std::vector<Coord> path;
            Coord cur = {goalR,goalC};

            while (!(cur.first == startR &&
                     cur.second == startC))
            {
                path.push_back(cur);
                cur = parent[cur.first][cur.second];
            }

            path.push_back({startR,startC});
            std::reverse(path.begin(), path.end());
            return path;
        }

        for (auto [dr,dc] : directions)
        {
            int nr = r + dr;
            int nc = c + dc;

            if (nr < 0 || nr >= rows ||
                nc < 0 || nc >= cols)
                continue;

            if (grid[nr][nc] == 1)  // obstacle
                continue;

            double moveCost =
                (dr == 0 || dc == 0) ? 1.0 : 1.414;  // diagonal cost

            double terrainCost =
                std::max(0, originalGrid[nr][nc]);   // cost map support

            double newG = gCost[r][c] + moveCost + terrainCost;

            if (newG < gCost[nr][nc])
            {
                gCost[nr][nc] = newG;
                parent[nr][nc] = {r,c};

                double h = heuristic(nr,nc,goalR,goalC);
                open.push({nr,nc,newG,h});
            }
        }
    }

    return {};  // no path
}
