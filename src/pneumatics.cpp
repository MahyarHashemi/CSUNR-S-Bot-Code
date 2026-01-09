#include "main.h"
#include "subsystems.hpp"
//#include "piston.hpp"

extern bool rake_state = false;
extern bool outtake_state = false;
extern bool angle_state = false;

void rake_down(){
    rake.set(true);
}

void rake_up(){
    rake.set(false);
}

void outtake_down(){
    outtake.set(false);
}

void outtake_up(){
    outtake.set(true);
}

void angle_down(){
    angle.set(false);
}

void angle_up(){
    angle.set(true);
}

void pneumatic_control(){
    if (rake_state){
        rake_down();
    }
    else{
        rake_up();
    }

    if (master.get_digital_new_press(DIGITAL_DOWN)){
        rake_state = !rake_state;
    }


    // if (master.get_digital(DIGITAL_L1)){
    //     outtake_state = false;
    // }
    // else{
    //     outtake_state = true;
    // }

    // if (master.get_digital(DIGITAL_L2)){
    //     angle_state = false;
    // }
    // else{
    //     angle_state = true;
    // }


    if (angle_state){
        angle_down();
    }
    else{
        angle_up();
    }

    if (master.get_digital_new_press(DIGITAL_L2)){
        angle_state = !angle_state;
    }


    if (outtake_state){
        outtake_down();
    }
    else{
        outtake_up();
    }

    if (master.get_digital_new_press(DIGITAL_L1)){
        outtake_state = !outtake_state;
    }
}