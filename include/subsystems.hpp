#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"

extern Drive chassis;

// Your motors, sensors, etc. should go here.  Below are examples

inline ez::Piston rake('A');
inline ez::Piston angle('B');
inline ez::Piston outtake('C');

inline pros::MotorGroup top_intake({-12, 17});
inline pros::MotorGroup bottom_intake({-16, 11});

// inline pros::Motor intake(1);
// inline pros::adi::DigitalIn limit_switch('A');