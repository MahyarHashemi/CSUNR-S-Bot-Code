#include "main.h"
#include "EZ-Template/drive/drive.hpp"
#include <vector>
#include <iostream>

std::vector<double> mulMat(double mat1[3][1], double mat2[3][3])
{
    std::vector<double> rot_error(3, 0.0);
    double rslt[3][1];

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 1; j++) {
            rslt[i][j] = 0;

            for (int k = 0; k < 3; k++) {
                rslt[i][j] += mat1[i][0] * mat2[k][j];
            }
            rot_error[i] = rslt[i][j];
        }
    }
    return rot_error;
}

std::vector<std::vector<int>> create_map(int rows, int cols){
    std::vector<std::vector<int>> grid(rows, std::vector<int>(cols, 0));
    return grid;
}

//Convert from 16th of an inch to the appropriate cell
std::pair<int, int> convert_to_cell(double x1, double y1){
    int cell_x1 = abs(x1) * 16;
    int cell_y1 = abs(y1) * 16;
    
    if (x1 >= 0){
        cell_x1 += 1152;
    }

    if (y1 >= 0){
        cell_y1 += 1152;
    }

    round(cell_x1);
    round(cell_y1);

    return {x1, y1};
}

void addBlockObstacles(std::vector<std::vector<int>>& grid, double x1, double y1, double x2, double y2) {
    auto [x1_cell, y1_cell] = convert_to_cell(x1, y1);
    auto [x2_cell, y2_cell] = convert_to_cell(x2, y2);
    for (int i = x1_cell; i <= x2_cell; ++i) {
        for (int j = y1_cell; j <= y2_cell; ++j) {
            grid[i][j] = 1; // Mark as an obstacle
        }
    }
}

void addObstacles(std::vector<std::vector<int>>& grid, int x1, int y1) {
    auto [x1_cell, y1_cell] = convert_to_cell(x1, y1);
    grid[x1_cell][y1_cell] = 1; // Mark a cell as an obstacle
}

