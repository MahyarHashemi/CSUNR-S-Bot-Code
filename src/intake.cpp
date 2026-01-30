#include "main.h"
#include "subsystems.hpp"

void intake_speed(int speed){
    top_intake.move(speed);
    bottom_intake.move(speed);
}

void intake_zero(){
    top_intake.tare_position();
    bottom_intake.tare_position();
}

void intake_set_pos(double pos, int speed){
    top_intake.move_absolute(pos, speed);
    bottom_intake.move_absolute(pos, speed);
}

void intake_control(){
    if (master.get_digital(DIGITAL_R1)){
        top_intake.move(127);
        bottom_intake.move(127);
    }
    else if (master.get_digital(DIGITAL_R2)){
        top_intake.move(-127);
        bottom_intake.move(-127);
    }
    // else if (master.get_digital(DIGITAL_X)){
    //     top_intake.move(63);
    //     bottom_intake.move(63);
    // }
    else{
        top_intake.move(0);
        bottom_intake.move(0);
    }
}