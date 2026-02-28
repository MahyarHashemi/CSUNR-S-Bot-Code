#include "main.h"
#include "EZ-Template/drive/drive.hpp"
#include <vector>
#include <iostream>

double x_des, 
        y_des, 
        theta_des, 
        x_act, y_act, 
        theta_act, 
        b = 6.7, 
        z = 0.5, 
        vd, 
        wd;

double error[3][1] = {{x_des - x_act},
                   {y_des - y_act},
                   {theta_des - theta_act}};
double rotation[3][3] = {{cos(theta_act), sin(theta_act), 0},
                        {-sin(theta_act), cos(theta_act), 0},
                        {0, 0, 1}};

//Create map in cell value of 1/16th of in
//These values are not inches
std::vector<std::vector<int>> grid = create_map(2304, 2304);


