#pragma once

extern bool rake_state;
extern bool outtake_state;
extern bool angle_state;

void rake_down();
void rake_up();
void outtake_down();
void outtake_up();
void angle_down();
void angle_up();

void pneumatic_control();