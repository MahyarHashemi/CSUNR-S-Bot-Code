#pragma once
#include <iostream>
#include <vector>

std::vector<double> mulMat(double mat1[3][1], double mat2[3][3]);

std::vector<std::vector<int>> create_map(int rows, int cols);

//Convert from 16th of an inch to the appropriate cell
std::pair<int, int> convert_to_cell(double x1, double y1);

void addBlockObstacles(std::vector<std::vector<int>>& grid, double x1, double y1, double x2, double y2);

void addObstacles(std::vector<std::vector<int>>& grid, int x1, int y1);