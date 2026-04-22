#include "main.h"
#include "EZ-Template/intake.hpp"
#include "EZ-Template/pneumatics.hpp"

/////
// For installation, upgrading, documentations, and tutorials, check out our website!
// https://ez-robotics.github.io/EZ-Template/
/////

// These are out of 127
const int DRIVE_SPEED = 110;
const int TURN_SPEED = 90;
const int SWING_SPEED = 110;

//Mahyar Variables (Mainly for PID as of rn)
double c_mass = 7.39;
//Drive Constants
double kp_drive = 20.0; double kd_drive = 2 * sqrt(c_mass * kp_drive);
//Heading Constants
double kp_heading = 11.0; double kd_heading = 2 * sqrt(c_mass * kp_heading);
//Turn Constants
double kp_turn = 3.0; double kd_turn = 2 * sqrt(c_mass * kp_turn);
//Swing Constants
double kp_swing = 6.5; double kd_swing = 2 * sqrt(c_mass * kp_swing);
//Odom Constants
double kp_odom = 6.5; double kd_odom = 2 * sqrt(c_mass * kp_odom);
//Boomerang Constants
double kp_boom = 5.8; double kd_boom = 2 * sqrt(c_mass * kp_boom);

///
// Constants
///
void default_constants() {
  // P, I, D, and Start I
  chassis.pid_drive_constants_set(20.0, 0.0, 100.0);         // Fwd/rev constants, used for odom and non odom motions
  chassis.pid_heading_constants_set(11.0, 0.0, 20.0);        // Holds the robot straight while going forward without odom
  chassis.pid_turn_constants_set(3.0, 0.05, 20.0, 15.0);     // Turn in place constants
  chassis.pid_swing_constants_set(6.0, 0.0, 65.0);           // Swing constants
  chassis.pid_odom_angular_constants_set(6.5, 0.0, 52.5);    // Angular control for odom motions
  chassis.pid_odom_boomerang_constants_set(5.8, 0.0, 32.5);  // Angular control for boomerang motions

  // Exit conditions
  chassis.pid_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_swing_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 500_ms);
  chassis.pid_odom_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 750_ms);
  chassis.pid_odom_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 750_ms);
  chassis.pid_turn_chain_constant_set(3_deg);
  chassis.pid_swing_chain_constant_set(5_deg);
  chassis.pid_drive_chain_constant_set(3_in);

  // Slew constants
  chassis.slew_turn_constants_set(3_deg, 70);
  chassis.slew_drive_constants_set(3_in, 70);
  chassis.slew_swing_constants_set(3_in, 80);

  // The amount that turns are prioritized over driving in odom motions
  // - if you have tracking wheels, you can run this higher.  1.0 is the max
  chassis.odom_turn_bias_set(0.9);

  chassis.odom_look_ahead_set(7_in);           // This is how far ahead in the path the robot looks at
  chassis.odom_boomerang_distance_set(16_in);  // This sets the maximum distance away from target that the carrot point can be
  chassis.odom_boomerang_dlead_set(0.625);     // This handles how aggressive the end of boomerang motions are

  chassis.pid_angle_behavior_set(ez::shortest);  // Changes the default behavior for turning, this defaults it to the shortest path there
}

///
// Drive Example
///
void drive_example() {
  // The first parameter is target inches
  // The second parameter is max speed the robot will drive at
  // The third parameter is a boolean (true or false) for enabling/disabling a slew at the start of drive motions
  // for slew, only enable it when the drive distance is greater than the slew distance + a few inches

  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_drive_set(-12_in, DRIVE_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-12_in, DRIVE_SPEED);
  chassis.pid_wait();
}

///
// Turn Example
///
void turn_example() {
  // The first parameter is the target in degrees
  // The second parameter is max speed the robot will drive at

  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();
}

///
// Combining Turn + Drive
///
void drive_and_turn() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}

///
// Wait Until and Changing Max Speed
///
void wait_until_change_speed() {
  // pid_wait_until will wait until the robot gets to a desired position

  // When the robot gets to 6 inches slowly, the robot will travel the remaining distance at full speed
  chassis.pid_drive_set(24_in, 30, true);
  chassis.pid_wait_until(6_in);
  chassis.pid_speed_max_set(DRIVE_SPEED);  // After driving 6 inches at 30 speed, the robot will go the remaining distance at DRIVE_SPEED
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  // When the robot gets to -6 inches slowly, the robot will travel the remaining distance at full speed
  chassis.pid_drive_set(-24_in, 30, true);
  chassis.pid_wait_until(-6_in);
  chassis.pid_speed_max_set(DRIVE_SPEED);  // After driving 6 inches at 30 speed, the robot will go the remaining distance at DRIVE_SPEED
  chassis.pid_wait();
}

///
// Swing Example
///
void swing_example() {
  // The first parameter is ez::LEFT_SWING or ez::RIGHT_SWING
  // The second parameter is the target in degrees
  // The third parameter is the speed of the moving side of the drive
  // The fourth parameter is the speed of the still side of the drive, this allows for wider arcs

  chassis.pid_swing_set(ez::LEFT_SWING, 45_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::RIGHT_SWING, 0_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::RIGHT_SWING, 45_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::LEFT_SWING, 0_deg, SWING_SPEED, 45);
  chassis.pid_wait();
}

///
// Motion Chaining
///
void motion_chaining() {
  // Motion chaining is where motions all try to blend together instead of individual movements.
  // This works by exiting while the robot is still moving a little bit.
  // To use this, replace pid_wait with pid_wait_quick_chain.
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  // Your final motion should still be a normal pid_wait
  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}

///
// Auto that tests everything
///
void combining_movements() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::RIGHT_SWING, -45_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}

///
// Interference example
///
void tug(int attempts) {
  for (int i = 0; i < attempts - 1; i++) {
    // Attempt to drive backward
    printf("i - %i", i);
    chassis.pid_drive_set(-12_in, 127);
    chassis.pid_wait();

    // If failsafed...
    if (chassis.interfered) {
      chassis.drive_sensor_reset();
      chassis.pid_drive_set(-2_in, 20);
      pros::delay(1000);
    }
    // If the robot successfully drove back, return
    else {
      return;
    }
  }
}

// If there is no interference, the robot will drive forward and turn 90 degrees.
// If interfered, the robot will drive forward and then attempt to drive backward.
void interfered_example() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  if (chassis.interfered) {
    tug(3);
    return;
  }

  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();
}

///
// Odom Drive PID
///
void odom_drive_example() {
  // This works the same as pid_drive_set, but it uses odom instead!
  // You can replace pid_drive_set with pid_odom_set and your robot will
  // have better error correction.

  chassis.pid_odom_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_odom_set(-12_in, DRIVE_SPEED);
  chassis.pid_wait();

  chassis.pid_odom_set(-12_in, DRIVE_SPEED);
  chassis.pid_wait();
}

///
// Odom Pure Pursuit
///
void odom_pure_pursuit_example() {
  // Drive to 0, 30 and pass through 6, 10 and 0, 20 on the way, with slew
  chassis.pid_odom_set({{{6_in, 10_in}, fwd, DRIVE_SPEED},
                        {{0_in, 20_in}, fwd, DRIVE_SPEED},
                        {{0_in, 30_in}, fwd, DRIVE_SPEED}},
                       true);
  chassis.pid_wait();

  // Drive to 0, 0 backwards
  chassis.pid_odom_set({{0_in, 0_in}, rev, DRIVE_SPEED},
                       true);
  chassis.pid_wait();
}

///
// Odom Pure Pursuit Wait Until
///
void odom_pure_pursuit_wait_until_example() {
  chassis.pid_odom_set({{{0_in, 24_in}, fwd, DRIVE_SPEED},
                        {{12_in, 24_in}, fwd, DRIVE_SPEED},
                        {{24_in, 24_in}, fwd, DRIVE_SPEED}},
                       true);
  chassis.pid_wait_until_index(1);  // Waits until the robot passes 12, 24
  // Intake.move(127);  // Set your intake to start moving once it passes through the second point in the index
  chassis.pid_wait();
  // Intake.move(0);  // Turn the intake off
}

///
// Odom Boomerang
///
void odom_boomerang_example() {
  chassis.pid_odom_set({{0_in, 24_in, 45_deg}, fwd, DRIVE_SPEED},
                       true);
  chassis.pid_wait();

  chassis.pid_odom_set({{0_in, 0_in, 0_deg}, rev, DRIVE_SPEED},
                       true);
  chassis.pid_wait();
}

///
// Odom Boomerang Injected Pure Pursuit
///
void odom_boomerang_injected_pure_pursuit_example() {
  chassis.pid_odom_set({{{0_in, 24_in, 45_deg}, fwd, DRIVE_SPEED},
                        {{12_in, 24_in}, fwd, DRIVE_SPEED},
                        {{24_in, 24_in}, fwd, DRIVE_SPEED}},
                       true);
  chassis.pid_wait();

  chassis.pid_odom_set({{0_in, 0_in, 0_deg}, rev, DRIVE_SPEED},
                       true);
  chassis.pid_wait();
}

///
// Calculate the offsets of your tracking wheels
///
void measure_offsets() {
  // Number of times to test
  int iterations = 10;

  // Our final offsets
  double l_offset = 0.0, r_offset = 0.0, b_offset = 0.0, f_offset = 0.0;

  // Reset all trackers if they exist
  if (chassis.odom_tracker_left != nullptr) chassis.odom_tracker_left->reset();
  if (chassis.odom_tracker_right != nullptr) chassis.odom_tracker_right->reset();
  if (chassis.odom_tracker_back != nullptr) chassis.odom_tracker_back->reset();
  if (chassis.odom_tracker_front != nullptr) chassis.odom_tracker_front->reset();
  
  for (int i = 0; i < iterations; i++) {
    // Reset pid targets and get ready for running an auton
    chassis.pid_targets_reset();
    chassis.drive_imu_reset();
    chassis.drive_sensor_reset();
    chassis.drive_brake_set(MOTOR_BRAKE_HOLD);
    chassis.odom_xyt_set(0_in, 0_in, 0_deg);
    double imu_start = chassis.odom_theta_get();
    double target = i % 2 == 0 ? 90 : 270;  // Switch the turn target every run from 270 to 90

    // Turn to target at half power
    chassis.pid_turn_set(target, 63, ez::raw);
    chassis.pid_wait();
    pros::delay(250);

    // Calculate delta in angle
    double t_delta = util::to_rad(fabs(util::wrap_angle(chassis.odom_theta_get() - imu_start)));

    // Calculate delta in sensor values that exist
    double l_delta = chassis.odom_tracker_left != nullptr ? chassis.odom_tracker_left->get() : 0.0;
    double r_delta = chassis.odom_tracker_right != nullptr ? chassis.odom_tracker_right->get() : 0.0;
    double b_delta = chassis.odom_tracker_back != nullptr ? chassis.odom_tracker_back->get() : 0.0;
    double f_delta = chassis.odom_tracker_front != nullptr ? chassis.odom_tracker_front->get() : 0.0;

    // Calculate the radius that the robot traveled
    l_offset += l_delta / t_delta;
    r_offset += r_delta / t_delta;
    b_offset += b_delta / t_delta;
    f_offset += f_delta / t_delta;
  }

  // Average all offsets
  l_offset /= iterations;
  r_offset /= iterations;
  b_offset /= iterations;
  f_offset /= iterations;

  // Set new offsets to trackers that exist
  if (chassis.odom_tracker_left != nullptr) chassis.odom_tracker_left->distance_to_center_set(l_offset);
  if (chassis.odom_tracker_right != nullptr) chassis.odom_tracker_right->distance_to_center_set(r_offset);
  if (chassis.odom_tracker_back != nullptr) chassis.odom_tracker_back->distance_to_center_set(b_offset);
  if (chassis.odom_tracker_front != nullptr) chassis.odom_tracker_front->distance_to_center_set(f_offset);
}

// . . .
// Make your own autonomous functions here!

void match_auton(){
  // y = -50, x = 18, theta = 90
  chassis.drive_imu_reset();
  chassis.pid_targets_reset();
  chassis.drive_sensor_reset();
  chassis.odom_reset();
  chassis.odom_xyt_set(-16_in, -49_in, 270_deg);
  // chassis.odom_x_flip();

  // chassis.pid_drive_set(26_in, DRIVE_SPEED);
  // chassis.pid_wait_quick();

  // chassis.pid_turn_set(180_deg, TURN_SPEED);
  // chassis.pid_wait();

  chassis.pid_odom_ptp_set({{-50_in, -49_in, 270_deg}, fwd, 65}, true);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, 60);
  chassis.pid_wait();

  chassis.pid_drive_set(5_in, DRIVE_SPEED);
  chassis.pid_wait();

  rake_down();
  outtake_down();
  intake_speed(127);
  // outtake_speed(127);

  chassis.pid_drive_set(-24_in, 60);
  chassis.pid_wait();

  pros::delay(100);

  chassis.pid_drive_set(-2_in, DRIVE_SPEED, false);
  chassis.pid_wait();

  pros::delay(1000);

  chassis.pid_drive_set(10_in, DRIVE_SPEED);
  chassis.pid_wait();

  rake_up();
  angle_up();
  // intake_speed(0);
  // outtake_speed(0);

  // chassis.pid_turn_set(1.75_deg, 70);
  // chassis.pid_wait();

  chassis.pid_drive_set(17.5_in, DRIVE_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-1.5_in, 60);
  chassis.pid_wait();

  outtake_up();
  pros::delay(500);
  // intake_speed(127);
  // outtake_speed(127);
  pros::delay(1500);
  
  chassis.pid_drive_set(-10_in, DRIVE_SPEED);
  chassis.pid_wait();

  intake_speed(0);
  // outtake_speed(0);

  chassis.pid_turn_set(90_deg, 80);
  chassis.pid_wait();

  intake_speed(-127);
  // outtake_speed(-127);
  pros::delay(100);
  intake_speed(127);
  // outtake_speed(127);
  pros::delay(1000);

  chassis.pid_turn_set(0_deg, 60);
  chassis.pid_wait();

  rake_down();
  outtake_down();
  intake_speed(127);
  // outtake_speed(127);

  chassis.pid_drive_set(7.5_in, 80);
  chassis.pid_wait();

  chassis.pid_drive_set(-24_in, 70);
  chassis.pid_wait();

  chassis.pid_drive_set(-2_in, DRIVE_SPEED);
  chassis.pid_wait();

  // chassis.pid_drive_set(-2_in, DRIVE_SPEED, false);
  // chassis.pid_wait();
  // chassis.pid_drive_set(3_in, DRIVE_SPEED, false);
  // chassis.pid_wait();

  //was 3000
  pros::delay(1500);

  chassis.pid_drive_set(10_in, 75);
  chassis.pid_wait();

  rake_up();
  angle_up();
  // intake_speed(0);
  // outtake_speed(0);

  // chassis.pid_turn_set(4.5_deg, 60);
  // chassis.pid_wait();

  intake_speed(-127);
  // outtake_speed(-127);
  pros::delay(100);
  intake_speed(0);
  // outtake_speed(0);

  chassis.pid_drive_set(22_in, DRIVE_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-1.5_in, 60);
  chassis.pid_wait();

  outtake_up();
  intake_speed(127);
  // outtake_speed(127);
  pros::delay(500);
  // intake_speed(127);
  // outtake_speed(127);
  pros::delay(1500);

  chassis.pid_drive_set(-10_in, 80);
  chassis.pid_wait();

  intake_speed(0);
  // outtake_speed(0);
}

void mid_match_auton(){
  // y = -50, x = 18, theta = 90

  //Reset of all sensors and initialization of starting position/heading
  chassis.drive_imu_reset();
  chassis.pid_targets_reset();
  chassis.drive_sensor_reset();
  chassis.odom_reset();
  chassis.odom_xyt_set(-16_in, -49_in, 270_deg);
  pros::delay(10);

  // chassis.pid_odom_ptp_set({{-50_in, -50_in, 271_deg}, fwd, 75}, true);
  // chassis.pid_wait();

  //Drive forward to match loader
  chassis.pid_drive_set(34_in, 75, true);
  chassis.pid_wait();

  //Turn to face rake to match loader
  chassis.pid_turn_set(0_deg, 60);
  chassis.pid_wait();

  //Drop rake mech, bring down block cover, and start running intake
  rake_down();
  outtake_down();
  intake_speed(127);
  // outtake_speed(127);

  //Drive backward into match loader
  //Speed was 60
  chassis.pid_drive_set(-24_in, 60);
  chassis.pid_wait();

  pros::delay(100);

  //Drive further into match loader to fully seat rake in
  chassis.pid_drive_set(-2_in, DRIVE_SPEED, false);
  chassis.pid_wait();

  pros::delay(100);

  intake_speed(-127);
  pros::delay(250);
  intake_speed(127);

  //Perform shimmy to dislodge any jammed blocks in match loader
  chassis.pid_drive_set(2_in, DRIVE_SPEED, false);
  chassis.pid_wait();
  chassis.pid_drive_set(-3_in, DRIVE_SPEED, false);
  chassis.pid_wait();
  chassis.pid_drive_set(1_in, DRIVE_SPEED, false);
  chassis.pid_wait();
  chassis.pid_drive_set(-3_in, DRIVE_SPEED, false);
  chassis.pid_wait();

  pros::delay(800);

  //Drive out of match loader
  chassis.pid_drive_set(10_in, DRIVE_SPEED);
  chassis.pid_wait();

  //Bring up rake mech
  rake_up();
  
  //Turn to face mid goals
  //Was 70 for speed
  chassis.pid_turn_set(43_deg, 60);
  chassis.pid_wait();
  intake_speed(0);

  //Drive towards upper mid goal
  //Was 90 for speed
  chassis.pid_drive_set(50_in, 70);
  pros::delay(50);
  intake_speed(-127);
  pros::delay(250);
  intake_speed(0);
  chassis.pid_wait();

  //Drive back to allow for some extra room for error
  //Was -1.5 for drive distance
  chassis.pid_drive_set(-0.75_in, 75);
  chassis.pid_wait();

  //Lift up ball cover and wait 2 seconds
  //Was 1 second for time
  outtake_up();
  pros::delay(50);
  intake_speed(127);
  pros::delay(2500);

  //Drive back in front of match loader
  chassis.pid_drive_set(-44.125_in, 90);
  chassis.pid_wait();

  //Turn back to face match loader
  chassis.pid_turn_set(0_deg, 80);
  chassis.pid_wait();

  rake_down();
  intake_speed(127);
  outtake_down();

  //Was 100 for speed
  chassis.pid_drive_set(-12_in, 50);
  chassis.pid_wait();
  
  pros::delay(2000);

  chassis.pid_drive_set(3_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_drive_set(-4_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  pros::delay(750);
  intake_speed(-127);
  pros::delay(350);
  intake_speed(127);

  chassis.pid_drive_set(1_in, DRIVE_SPEED, false);
  chassis.pid_wait();
  chassis.pid_drive_set(-3_in, DRIVE_SPEED, false);
  chassis.pid_wait();

  pros::delay(1000);

  chassis.pid_drive_set(10_in, 90);
  chassis.pid_wait();

  rake_up();
  angle_up();
  pros::delay(100);

  chassis.pid_drive_set(20_in, 80);
  pros::delay(100);
  intake_speed(-127);
  pros::delay(250);
  intake_speed(0);
  chassis.pid_wait();

  chassis.pid_drive_set(-1.5_in, DRIVE_SPEED);
  chassis.pid_wait();

  outtake_up();
  intake_speed(127);
  pros::delay(3000);

  chassis.pid_drive_set(-10_in, DRIVE_SPEED);
  chassis.pid_wait();

  descore_up();

  chassis.pid_odom_ptp_set({{-36_in, -24_in, 0_deg}, fwd, DRIVE_SPEED}, true);
  chassis.pid_wait();

  descore_down();
  pros::delay(25);

  chassis.pid_drive_set(10_in, DRIVE_SPEED);
  chassis.pid_wait();
}

void skills_auton(){
  // y = -50, x = 18, theta = 90
  chassis.drive_imu_reset();
  chassis.pid_targets_reset();
  chassis.drive_sensor_reset();
  chassis.odom_reset();
  chassis.odom_xyt_set(-16_in, -49_in, 270_deg);

  chassis.pid_drive_set(34_in, 75, true);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, 60);
  chassis.pid_wait();

  chassis.pid_drive_set(5_in, DRIVE_SPEED);
  chassis.pid_wait();

  rake_down();
  outtake_down();
  intake_speed(127);

  chassis.pid_drive_set(-24_in, 60);
  chassis.pid_wait();

  pros::delay(100);

  chassis.pid_drive_set(-2_in, 70, false);
  chassis.pid_wait();

  pros::delay(500);

  chassis.pid_drive_set(2_in, DRIVE_SPEED, false);
  chassis.pid_wait();
  chassis.pid_drive_set(-3_in, DRIVE_SPEED, false);
  chassis.pid_wait();

  pros::delay(1500);

  chassis.pid_drive_set(10_in, DRIVE_SPEED);
  chassis.pid_wait();

  rake_up();
  angle_up();

  chassis.pid_odom_set({{{-59.725_in, -34.27_in, 0_deg}, fwd, DRIVE_SPEED},
                        {{-52.273_in, 47.929_in, 90_deg}, fwd, DRIVE_SPEED},
                        {{-47_in, 29_in, 180_deg}, fwd, 75}}, true);

  chassis.pid_drive_set(-1.5_in, 60);
  chassis.pid_wait();

  intake_speed(-127);
  pros::delay(250);

  outtake_up();
  intake_speed(127);
  pros::delay(500);
  // intake_speed(127);
  // outtake_speed(127);
  pros::delay(1500);
  
  chassis.pid_drive_set(-10_in, DRIVE_SPEED);
  chassis.pid_wait();

  intake_speed(0);
  // outtake_speed(0);

  chassis.pid_turn_set(0_deg, 60);
  chassis.pid_wait();

  rake_down();
  outtake_down();
  intake_speed(127);

  chassis.pid_drive_set(7.5_in, 80);
  chassis.pid_wait();

  chassis.pid_drive_set(-24_in, 70);
  chassis.pid_wait();

  chassis.pid_drive_set(-2_in, DRIVE_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(2_in, DRIVE_SPEED, false);
  chassis.pid_wait();
  chassis.pid_drive_set(-3_in, DRIVE_SPEED, false);
  chassis.pid_wait();

  //was 3000
  pros::delay(3000);

  chassis.pid_drive_set(10_in, 75);
  chassis.pid_wait();

  rake_up();
  angle_up();

  intake_speed(-127);
  // outtake_speed(-127);
  pros::delay(100);
  intake_speed(0);
  // outtake_speed(0);

  chassis.pid_drive_set(22_in, DRIVE_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-1.5_in, 60);
  chassis.pid_wait();

  outtake_up();
  intake_speed(127);
  // outtake_speed(127);
  pros::delay(500);
  // intake_speed(127);
  // outtake_speed(127);
  pros::delay(1500);

  chassis.pid_drive_set(-10_in, 80);
  chassis.pid_wait();

  intake_speed(0);
  // outtake_speed(0);
}



void long_mid_match_auton(){
  // y = -50, x = 18, theta = 90

  //Reset of all sensors and initialization of starting position/heading
  chassis.drive_imu_reset();
  chassis.pid_targets_reset();
  chassis.drive_sensor_reset();
  chassis.odom_reset();
  chassis.odom_xyt_set(-16_in, -49_in, 270_deg);
  pros::delay(10);

  //Drive forward to match loader
  chassis.pid_drive_set(33_in, 75, true);
  chassis.pid_wait();

  //Turn to face rake to match loader
  chassis.pid_turn_set(0_deg, 60);
  chassis.pid_wait();

  //Drop rake mech, bring down block cover, and start running intake
  rake_down();
  outtake_down();
  intake_speed(127);
  // outtake_speed(127);

  //Drive backward into match loader
  //Speed was 60
  chassis.pid_drive_set(-24_in, 60);
  chassis.pid_wait();

  pros::delay(100);

  //Drive further into match loader to fully seat rake in
  chassis.pid_drive_set(-2_in, DRIVE_SPEED, false);
  chassis.pid_wait();

  pros::delay(100);

  intake_speed(-127);
  pros::delay(250);
  intake_speed(127);

  //Perform shimmy to dislodge any jammed blocks in match loader
  chassis.pid_drive_set(2_in, DRIVE_SPEED, false);
  chassis.pid_wait();
  chassis.pid_drive_set(-3_in, DRIVE_SPEED, false);
  chassis.pid_wait();
  chassis.pid_drive_set(1_in, DRIVE_SPEED, false);
  chassis.pid_wait();
  chassis.pid_drive_set(-3_in, DRIVE_SPEED, false);
  chassis.pid_wait();

  pros::delay(800);

  //Drive out of match loader
  chassis.pid_drive_set(10_in, DRIVE_SPEED);
  chassis.pid_wait();

  //Bring up rake mech
  rake_up();
  angle_up();
  pros::delay(100);

  chassis.pid_drive_set(23_in, 80);
  pros::delay(100);
  intake_speed(-127);
  pros::delay(250);
  intake_speed(0);
  chassis.pid_wait();

  chassis.pid_drive_set(-1.5_in, DRIVE_SPEED);
  chassis.pid_wait();

  outtake_up();
  intake_speed(127);
  pros::delay(3000);

  chassis.pid_drive_set(-12_in, DRIVE_SPEED);
  rake_down();
  intake_speed(127);
  outtake_down();
  angle_down();
  chassis.pid_wait();

  //Turn back to face match loader
  chassis.pid_turn_set(0_deg, 80);
  chassis.pid_wait();

  //Was 100 for speed
  chassis.pid_drive_set(-17.5_in, 75);
  chassis.pid_wait();
  
  pros::delay(2000);

  chassis.pid_drive_set(3_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_drive_set(-4_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  pros::delay(750);
  intake_speed(-127);
  pros::delay(350);
  intake_speed(127);

  chassis.pid_drive_set(1_in, DRIVE_SPEED, false);
  chassis.pid_wait();
  chassis.pid_drive_set(-3_in, DRIVE_SPEED, false);
  chassis.pid_wait();

  pros::delay(1000);

  chassis.pid_drive_set(5_in, 80);
  chassis.pid_wait();

  rake_up();
  pros::delay(250);

  //Turn to face mid goals
  //Was 70 for speed
  chassis.pid_turn_set(40_deg, 60);
  chassis.pid_wait();


  intake_speed(0);

  //Drive towards upper mid goal
  //Was 90 for speed
  chassis.pid_drive_set(55_in, 70);
  pros::delay(50);
  intake_speed(-127);
  pros::delay(250);
  intake_speed(0);
  chassis.pid_wait();

  //Drive back to allow for some extra room for error
  //Was -1.5 for drive distance, then -0.75
  chassis.pid_drive_set(-1.25_in, 75);
  chassis.pid_wait();

  //Lift up ball cover and wait 2 seconds
  //Was 1 second for time
  outtake_up();
  pros::delay(50);
  intake_speed(127);
  pros::delay(2500);
}


void mid_long_match_auton(){
  // y = -50, x = 18, theta = 90

  //Reset of all sensors and initialization of starting position/heading
  chassis.drive_imu_reset();
  chassis.pid_targets_reset();
  chassis.drive_sensor_reset();
  chassis.odom_reset();
  chassis.odom_xyt_set(-16_in, -49_in, 270_deg);
  pros::delay(10);

  //Drive forward to match loader
  chassis.pid_drive_set(33_in, 75, true);
  chassis.pid_wait();

  //Turn to face rake to match loader
  chassis.pid_turn_set(0_deg, 60);
  chassis.pid_wait();

  //Drop rake mech, bring down block cover, and start running intake
  rake_down();
  outtake_down();
  intake_speed(127);

  //Drive backward into match loader
  //Speed was 60
  chassis.pid_drive_set(-24_in, 60);
  chassis.pid_wait();

  pros::delay(100);

  //Drive further into match loader to fully seat rake in
  chassis.pid_drive_set(-2_in, DRIVE_SPEED, false);
  chassis.pid_wait();

  pros::delay(100);

  intake_speed(-127);
  pros::delay(350);
  intake_speed(127);

  //Perform shimmy to dislodge any jammed blocks in match loader
  chassis.pid_drive_set(2_in, DRIVE_SPEED, false);
  chassis.pid_wait();
  chassis.pid_drive_set(-3_in, DRIVE_SPEED, false);
  chassis.pid_wait();
  chassis.pid_drive_set(1_in, DRIVE_SPEED, false);
  chassis.pid_wait();
  chassis.pid_drive_set(-3_in, DRIVE_SPEED, false);
  chassis.pid_wait();

  pros::delay(800);

  //Drive out of match loader
  chassis.pid_drive_set(5_in, 80);
  chassis.pid_wait();

  //Bring up rake mech
  rake_up();
  pros::delay(100);

  //Turn to face mid goals
  //Was 70 for speed
  //Was 40_deg
  chassis.pid_turn_set(41.5_deg, 60);
  chassis.pid_wait();


  intake_speed(0);

  //Drive towards upper mid goal
  //Was 90 for speed
  //Was 55 in
  chassis.pid_drive_set(55_in, 70);
  pros::delay(50);
  intake_speed(-127);
  pros::delay(250);
  intake_speed(0);
  chassis.pid_wait();

  //Drive back to allow for some extra room for error
  //Was -1.5 for drive distance, then -0.75
  chassis.pid_drive_set(-1.25_in, 75);
  chassis.pid_wait();

  //Lift up ball cover and wait 2 seconds
  //Was 1 second for time
  outtake_up();
  pros::delay(50);
  intake_speed(127);
  pros::delay(1500);

  //Was 53.75_in
  chassis.pid_drive_set(-52.75_in, 70);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, 60);
  outtake_down();
  chassis.pid_wait();

  rake_down();

  chassis.pid_drive_set(5_in, DRIVE_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-17.5_in, 75);
  chassis.pid_wait();
  
  pros::delay(2000);

  chassis.pid_drive_set(3_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_drive_set(-4_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  pros::delay(750);
  intake_speed(-127);
  pros::delay(350);
  intake_speed(127);

  chassis.pid_drive_set(1_in, DRIVE_SPEED, false);
  chassis.pid_wait();
  chassis.pid_drive_set(-3_in, DRIVE_SPEED, false);
  chassis.pid_wait();

  pros::delay(1000);

  //Was 23_in
  chassis.pid_drive_set(30_in, 80);
  pros::delay(100);
  angle_up();
  intake_speed(-127);
  pros::delay(250);
  intake_speed(0);
  chassis.pid_wait();

  chassis.pid_drive_set(-1.5_in, DRIVE_SPEED);
  chassis.pid_wait();

  outtake_up();
  intake_speed(127);
  pros::delay(3000);
}

void long_descore_quick_match(){
  // y = -50, x = 18, theta = 90

  //Reset of all sensors and initialization of starting position/heading
  chassis.drive_imu_reset();
  chassis.pid_targets_reset();
  chassis.drive_sensor_reset();
  chassis.odom_reset();
  chassis.odom_xyt_set(-16_in, -49_in, 270_deg);
  pros::delay(10);

  // chassis.pid_odom_ptp_set({{-50_in, -50_in, 271_deg}, fwd, 75}, true);
  // chassis.pid_wait();

  //Drive forward to match loader
  chassis.pid_drive_set(33_in, 75, true);
  chassis.pid_wait();

  //Turn to face rake to match loader
  chassis.pid_turn_set(0_deg, 60);
  chassis.pid_wait();

  //Drive away from match loader for windup
  // chassis.pid_drive_set(5_in, DRIVE_SPEED);
  // chassis.pid_wait();

  //Drop rake mech, bring down block cover, and start running intake
  rake_down();
  outtake_down();
  intake_speed(127);
  // outtake_speed(127);

  //Drive backward into match loader
  //Speed was 60
  chassis.pid_drive_set(-24_in, 60);
  chassis.pid_wait();

  pros::delay(100);

  //Drive further into match loader to fully seat rake in
  chassis.pid_drive_set(-2_in, DRIVE_SPEED, false);
  chassis.pid_wait();

  pros::delay(100);

  intake_speed(-127);
  pros::delay(250);
  intake_speed(127);

  //Perform shimmy to dislodge any jammed blocks in match loader
  chassis.pid_drive_set(2_in, DRIVE_SPEED, false);
  chassis.pid_wait();
  chassis.pid_drive_set(-3_in, DRIVE_SPEED, false);
  chassis.pid_wait();
  chassis.pid_drive_set(1_in, DRIVE_SPEED, false);
  chassis.pid_wait();
  chassis.pid_drive_set(-3_in, DRIVE_SPEED, false);
  chassis.pid_wait();

  pros::delay(800);

  //Drive out of match loader
  chassis.pid_drive_set(10_in, DRIVE_SPEED);
  chassis.pid_wait();

  //Bring up rake mech
  rake_up();
  angle_up();
  pros::delay(100);

  chassis.pid_drive_set(23_in, 80);
  pros::delay(100);
  intake_speed(-127);
  pros::delay(250);
  intake_speed(0);
  chassis.pid_wait();

  chassis.pid_drive_set(-1.5_in, DRIVE_SPEED);
  chassis.pid_wait();

  outtake_up();
  intake_speed(127);
  pros::delay(3000);

  chassis.pid_drive_set(-12_in, DRIVE_SPEED);
  // rake_down();
  // intake_speed(127);
  // angle_down();
  outtake_down();
  descore_up();
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, 50);
  chassis.pid_wait();

  chassis.pid_drive_set(13.5_in, DRIVE_SPEED, false);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::RIGHT_SWING, 0_deg, 65, 10);
  chassis.pid_wait();

  chassis.pid_drive_set(2_in, DRIVE_SPEED, false);
  chassis.pid_wait();

  descore_down();
  pros::delay(250);

  chassis.pid_drive_set(20_in, 65);
  chassis.pid_wait();
}

void long_descore_slow_match(){
  // y = -50, x = 18, theta = 90

  //Reset of all sensors and initialization of starting position/heading
  chassis.drive_imu_reset();
  chassis.pid_targets_reset();
  chassis.drive_sensor_reset();
  chassis.odom_reset();
  chassis.odom_xyt_set(-16_in, -49_in, 270_deg);
  pros::delay(10);

  // chassis.pid_odom_ptp_set({{-50_in, -50_in, 271_deg}, fwd, 75}, true);
  // chassis.pid_wait();

  //Drive forward to match loader
  chassis.pid_drive_set(33_in, 75, true);
  chassis.pid_wait();

  //Turn to face rake to match loader
  chassis.pid_turn_set(0_deg, 60);
  chassis.pid_wait();

  //Drive away from match loader for windup
  // chassis.pid_drive_set(5_in, DRIVE_SPEED);
  // chassis.pid_wait();

  //Drop rake mech, bring down block cover, and start running intake
  rake_down();
  outtake_down();
  intake_speed(127);
  // outtake_speed(127);

  //Drive backward into match loader
  //Speed was 60
  chassis.pid_drive_set(-24_in, 60);
  chassis.pid_wait();

  pros::delay(100);

  //Drive further into match loader to fully seat rake in
  chassis.pid_drive_set(-2_in, DRIVE_SPEED, false);
  chassis.pid_wait();

  pros::delay(100);

  intake_speed(-127);
  pros::delay(250);
  intake_speed(127);

  //Perform shimmy to dislodge any jammed blocks in match loader
  chassis.pid_drive_set(2_in, DRIVE_SPEED, false);
  chassis.pid_wait();
  chassis.pid_drive_set(-3_in, DRIVE_SPEED, false);
  chassis.pid_wait();
  chassis.pid_drive_set(1_in, DRIVE_SPEED, false);
  chassis.pid_wait();
  chassis.pid_drive_set(-3_in, DRIVE_SPEED, false);
  chassis.pid_wait();

  pros::delay(800);

  //Drive out of match loader
  chassis.pid_drive_set(10_in, DRIVE_SPEED);
  chassis.pid_wait();

  //Bring up rake mech
  rake_up();
  angle_up();
  pros::delay(100);

  chassis.pid_drive_set(23_in, 80);
  pros::delay(100);
  intake_speed(-127);
  pros::delay(250);
  intake_speed(0);
  chassis.pid_wait();

  chassis.pid_drive_set(-1.5_in, DRIVE_SPEED);
  chassis.pid_wait();

  outtake_up();
  intake_speed(127);
  pros::delay(3000);

  chassis.pid_drive_set(-12_in, DRIVE_SPEED);
  rake_down();
  intake_speed(127);
  angle_down();
  outtake_down();
  // descore_up();
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, 80);
  chassis.pid_wait();

  chassis.pid_drive_set(-17.5_in, DRIVE_SPEED);
  chassis.pid_wait();

  pros::delay(2000);

  chassis.pid_drive_set(3_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_drive_set(-4_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  pros::delay(750);
  intake_speed(-127);
  pros::delay(350);
  intake_speed(127);

  chassis.pid_drive_set(1_in, DRIVE_SPEED, false);
  chassis.pid_wait();
  chassis.pid_drive_set(-3_in, DRIVE_SPEED, false);
  chassis.pid_wait();

  pros::delay(1000);

  chassis.pid_drive_set(5_in, 80);
  chassis.pid_wait();

  rake_up();
  pros::delay(250);

  //Drive out of match loader
  chassis.pid_drive_set(10_in, DRIVE_SPEED);
  chassis.pid_wait();

  //Bring up rake mech
  rake_up();
  angle_up();
  pros::delay(100);

  chassis.pid_drive_set(23_in, 80);
  pros::delay(100);
  intake_speed(-127);
  pros::delay(250);
  intake_speed(0);
  chassis.pid_wait();

  chassis.pid_drive_set(-1.5_in, DRIVE_SPEED);
  chassis.pid_wait();

  outtake_up();
  intake_speed(127);
  pros::delay(3000);

  chassis.pid_drive_set(-12_in, DRIVE_SPEED);
  // rake_down();
  // intake_speed(127);
  // angle_down();
  outtake_down();
  descore_up();
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, 50);
  chassis.pid_wait();

  chassis.pid_drive_set(13.5_in, DRIVE_SPEED);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::RIGHT_SWING, 0_deg, 65, 10);
  chassis.pid_wait();

  chassis.pid_drive_set(2_in, DRIVE_SPEED, false);
  chassis.pid_wait();

  descore_down();
  pros::delay(250);

  chassis.pid_drive_set(20_in, 65);
  chassis.pid_wait();

}

void random_auton(){
  std::random_device rd;  // a seed source for the random number engine
  std::mt19937 gen(rd()); // mersenne_twister_engine seeded with rd()
  std::uniform_int_distribution<> distrib(1, 4);

  int auto_iter = distrib(gen);

  if (auto_iter == 1){
    long_mid_match_auton();
  }
  else if (auto_iter == 2){
    mid_long_match_auton();
  }
  else if (auto_iter == 3){
    long_descore_quick_match();
  }
  else if (auto_iter == 4){
    long_descore_slow_match();
  }
}

void laz_long_mid_match_auton(){
  // y = -50, x = 18, theta = 90

  //Reset of all sensors and initialization of starting position/heading
  chassis.drive_imu_reset();
  chassis.pid_targets_reset();
  chassis.drive_sensor_reset();
  chassis.odom_reset();
  chassis.odom_xyt_set(16_in, -49_in, 90_deg);
  pros::delay(10);

  //Drive forward to match loader
  chassis.pid_drive_set(33_in, 75, true);
  chassis.pid_wait();

  //Turn to face rake to match loader
  chassis.pid_turn_set(0_deg, 60);
  chassis.pid_wait();

  //Drop rake mech, bring down block cover, and start running intake
  rake_down();
  outtake_down();
  intake_speed(127);
  // outtake_speed(127);

  //Drive backward into match loader
  //Speed was 60
  chassis.pid_drive_set(-24_in, 60);
  chassis.pid_wait();

  pros::delay(100);

  //Drive further into match loader to fully seat rake in
  chassis.pid_drive_set(-2_in, DRIVE_SPEED, false);
  chassis.pid_wait();

  pros::delay(100);

  intake_speed(-127);
  pros::delay(250);
  intake_speed(127);

  //Perform shimmy to dislodge any jammed blocks in match loader
  chassis.pid_drive_set(2_in, DRIVE_SPEED, false);
  chassis.pid_wait();
  chassis.pid_drive_set(-3_in, DRIVE_SPEED, false);
  chassis.pid_wait();
  chassis.pid_drive_set(1_in, DRIVE_SPEED, false);
  chassis.pid_wait();
  chassis.pid_drive_set(-3_in, DRIVE_SPEED, false);
  chassis.pid_wait();

  pros::delay(800);

  //Drive out of match loader
  chassis.pid_drive_set(10_in, DRIVE_SPEED);
  chassis.pid_wait();

  //Bring up rake mech
  rake_up();
  angle_up();
  pros::delay(100);

  chassis.pid_drive_set(23_in, 80);
  pros::delay(100);
  intake_speed(-127);
  pros::delay(250);
  intake_speed(0);
  chassis.pid_wait();

  chassis.pid_drive_set(-1.5_in, DRIVE_SPEED);
  chassis.pid_wait();

  outtake_up();
  intake_speed(127);
  pros::delay(3000);

  chassis.pid_drive_set(-12_in, DRIVE_SPEED);
  rake_down();
  intake_speed(127);
  outtake_down();
  angle_down();
  chassis.pid_wait();

  //Turn back to face match loader
  chassis.pid_turn_set(0_deg, 80);
  chassis.pid_wait();

  //Was 100 for speed
  chassis.pid_drive_set(-17.5_in, 75);
  chassis.pid_wait();
  
  pros::delay(2000);

  chassis.pid_drive_set(3_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_drive_set(-4_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  pros::delay(750);
  intake_speed(-127);
  pros::delay(350);
  intake_speed(127);

  chassis.pid_drive_set(1_in, DRIVE_SPEED, false);
  chassis.pid_wait();
  chassis.pid_drive_set(-3_in, DRIVE_SPEED, false);
  chassis.pid_wait();

  pros::delay(1000);

  chassis.pid_drive_set(5_in, 80);
  chassis.pid_wait();

  rake_up();
  pros::delay(250);

  //Turn to face mid goals
  //Was 70 for speed
  chassis.pid_turn_set(130_deg, 60);
  chassis.pid_wait();


  intake_speed(0);

  //Drive towards upper mid goal
  //Was 90 for speed
  chassis.pid_drive_set(-55_in, 70);
  pros::delay(50);
  intake_speed(-127);
  pros::delay(250);
  intake_speed(127);
  pros::delay(100);
  intake_speed(0);
  chassis.pid_wait();

  //Drive back to allow for some extra room for error
  //Was -1.5 for drive distance, then -0.75
  chassis.pid_drive_set(1.25_in, 75);
  chassis.pid_wait();

  //Lift up ball cover and wait 2 seconds
  //Was 1 second for time
  outtake_up();
  pros::delay(50);
  intake_speed(-127);
  pros::delay(2500);
}


void laz_mid_long_match_auton(){
  // y = -50, x = 18, theta = 90

  //Reset of all sensors and initialization of starting position/heading
  chassis.drive_imu_reset();
  chassis.pid_targets_reset();
  chassis.drive_sensor_reset();
  chassis.odom_reset();
  chassis.odom_xyt_set(16_in, -49_in, 90_deg);
  pros::delay(10);

  //Drive forward to match loader
  chassis.pid_drive_set(33_in, 75, true);
  chassis.pid_wait();

  //Turn to face rake to match loader
  chassis.pid_turn_set(0_deg, 60);
  chassis.pid_wait();

  //Drop rake mech, bring down block cover, and start running intake
  rake_down();
  outtake_down();
  intake_speed(127);

  //Drive backward into match loader
  //Speed was 60
  chassis.pid_drive_set(-24_in, 60);
  chassis.pid_wait();

  pros::delay(100);

  //Drive further into match loader to fully seat rake in
  chassis.pid_drive_set(-2_in, DRIVE_SPEED, false);
  chassis.pid_wait();

  pros::delay(100);

  intake_speed(-127);
  pros::delay(350);
  intake_speed(127);

  //Perform shimmy to dislodge any jammed blocks in match loader
  chassis.pid_drive_set(2_in, DRIVE_SPEED, false);
  chassis.pid_wait();
  chassis.pid_drive_set(-3_in, DRIVE_SPEED, false);
  chassis.pid_wait();
  chassis.pid_drive_set(1_in, DRIVE_SPEED, false);
  chassis.pid_wait();
  chassis.pid_drive_set(-3_in, DRIVE_SPEED, false);
  chassis.pid_wait();

  pros::delay(800);

  //Drive out of match loader
  chassis.pid_drive_set(5_in, 80);
  chassis.pid_wait();

  //Bring up rake mech
  rake_up();
  pros::delay(100);

  //Turn to face mid goals
  //Was 70 for speed
  //Was 40_deg
  chassis.pid_turn_set(131.5_deg, 60);
  chassis.pid_wait();


  intake_speed(0);

  //Drive towards upper mid goal
  //Was 90 for speed
  //Was 55 in
  chassis.pid_drive_set(-55_in, 70);
  pros::delay(50);
  intake_speed(-127);
  pros::delay(250);
  intake_speed(127);
  pros::delay(100);
  intake_speed(0);
  chassis.pid_wait();

  //Drive back to allow for some extra room for error
  //Was -1.5 for drive distance, then -0.75
  chassis.pid_drive_set(1.25_in, 75);
  chassis.pid_wait();

  //Lift up ball cover and wait 2 seconds
  //Was 1 second for time
  // outtake_up();
  pros::delay(50);
  intake_speed(-127);
  pros::delay(1500);

  //Was 53.75_in
  chassis.pid_drive_set(52.75_in, 70);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, 60);
  outtake_down();
  chassis.pid_wait();

  rake_down();

  chassis.pid_drive_set(5_in, DRIVE_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-17.5_in, 75);
  chassis.pid_wait();
  
  pros::delay(2000);

  chassis.pid_drive_set(3_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_drive_set(-4_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  pros::delay(750);
  intake_speed(-127);
  pros::delay(350);
  intake_speed(127);

  chassis.pid_drive_set(1_in, DRIVE_SPEED, false);
  chassis.pid_wait();
  chassis.pid_drive_set(-3_in, DRIVE_SPEED, false);
  chassis.pid_wait();

  pros::delay(1000);

  //Was 23_in
  chassis.pid_drive_set(30_in, 80);
  pros::delay(100);
  angle_up();
  intake_speed(-127);
  pros::delay(250);
  intake_speed(0);
  chassis.pid_wait();

  chassis.pid_drive_set(-1.5_in, DRIVE_SPEED);
  chassis.pid_wait();

  outtake_up();
  intake_speed(127);
  pros::delay(3000);
}

void laz_long_descore_quick_match(){
  // y = -50, x = 18, theta = 90

  //Reset of all sensors and initialization of starting position/heading
  chassis.drive_imu_reset();
  chassis.pid_targets_reset();
  chassis.drive_sensor_reset();
  chassis.odom_reset();
  chassis.odom_xyt_set(16_in, -49_in, 90_deg);
  pros::delay(10);

  // chassis.pid_odom_ptp_set({{-50_in, -50_in, 271_deg}, fwd, 75}, true);
  // chassis.pid_wait();

  //Drive forward to match loader
  chassis.pid_drive_set(33_in, 75, true);
  chassis.pid_wait();

  //Turn to face rake to match loader
  chassis.pid_turn_set(0_deg, 60);
  chassis.pid_wait();

  //Drive away from match loader for windup
  // chassis.pid_drive_set(5_in, DRIVE_SPEED);
  // chassis.pid_wait();

  //Drop rake mech, bring down block cover, and start running intake
  rake_down();
  outtake_down();
  intake_speed(127);
  // outtake_speed(127);

  //Drive backward into match loader
  //Speed was 60
  chassis.pid_drive_set(-24_in, 60);
  chassis.pid_wait();

  pros::delay(100);

  //Drive further into match loader to fully seat rake in
  chassis.pid_drive_set(-2_in, DRIVE_SPEED, false);
  chassis.pid_wait();

  pros::delay(100);

  intake_speed(-127);
  pros::delay(250);
  intake_speed(127);

  //Perform shimmy to dislodge any jammed blocks in match loader
  chassis.pid_drive_set(2_in, DRIVE_SPEED, false);
  chassis.pid_wait();
  chassis.pid_drive_set(-3_in, DRIVE_SPEED, false);
  chassis.pid_wait();
  chassis.pid_drive_set(1_in, DRIVE_SPEED, false);
  chassis.pid_wait();
  chassis.pid_drive_set(-3_in, DRIVE_SPEED, false);
  chassis.pid_wait();

  pros::delay(800);

  //Drive out of match loader
  chassis.pid_drive_set(10_in, DRIVE_SPEED);
  chassis.pid_wait();

  //Bring up rake mech
  rake_up();
  angle_up();
  pros::delay(100);

  chassis.pid_drive_set(23_in, 80);
  pros::delay(100);
  intake_speed(-127);
  pros::delay(250);
  intake_speed(0);
  chassis.pid_wait();

  chassis.pid_drive_set(-1.5_in, DRIVE_SPEED);
  chassis.pid_wait();

  outtake_up();
  intake_speed(127);
  pros::delay(3000);

  chassis.pid_drive_set(-12_in, DRIVE_SPEED);
  // rake_down();
  // intake_speed(127);
  // angle_down();
  outtake_down();
  descore_up();
  chassis.pid_wait();

  chassis.pid_turn_set(-45_deg, 50);
  chassis.pid_wait();

  chassis.pid_drive_set(13.5_in, DRIVE_SPEED, false);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::LEFT_SWING, 0_deg, 65, 10);
  chassis.pid_wait();

  chassis.pid_drive_set(2_in, DRIVE_SPEED, false);
  chassis.pid_wait();

  descore_down();
  pros::delay(250);

  chassis.pid_drive_set(20_in, 65);
  chassis.pid_wait();
}

void laz_long_descore_slow_match(){
  // y = -50, x = 18, theta = 90

  //Reset of all sensors and initialization of starting position/heading
  chassis.drive_imu_reset();
  chassis.pid_targets_reset();
  chassis.drive_sensor_reset();
  chassis.odom_reset();
  chassis.odom_xyt_set(16_in, -49_in, 90_deg);
  pros::delay(10);

  // chassis.pid_odom_ptp_set({{-50_in, -50_in, 271_deg}, fwd, 75}, true);
  // chassis.pid_wait();

  //Drive forward to match loader
  chassis.pid_drive_set(33_in, 75, true);
  chassis.pid_wait();

  //Turn to face rake to match loader
  chassis.pid_turn_set(0_deg, 60);
  chassis.pid_wait();

  //Drive away from match loader for windup
  // chassis.pid_drive_set(5_in, DRIVE_SPEED);
  // chassis.pid_wait();

  //Drop rake mech, bring down block cover, and start running intake
  rake_down();
  outtake_down();
  intake_speed(127);
  // outtake_speed(127);

  //Drive backward into match loader
  //Speed was 60
  chassis.pid_drive_set(-24_in, 60);
  chassis.pid_wait();

  pros::delay(100);

  //Drive further into match loader to fully seat rake in
  chassis.pid_drive_set(-2_in, DRIVE_SPEED, false);
  chassis.pid_wait();

  pros::delay(100);

  intake_speed(-127);
  pros::delay(250);
  intake_speed(127);

  //Perform shimmy to dislodge any jammed blocks in match loader
  chassis.pid_drive_set(2_in, DRIVE_SPEED, false);
  chassis.pid_wait();
  chassis.pid_drive_set(-3_in, DRIVE_SPEED, false);
  chassis.pid_wait();
  chassis.pid_drive_set(1_in, DRIVE_SPEED, false);
  chassis.pid_wait();
  chassis.pid_drive_set(-3_in, DRIVE_SPEED, false);
  chassis.pid_wait();

  pros::delay(800);

  //Drive out of match loader
  chassis.pid_drive_set(10_in, DRIVE_SPEED);
  chassis.pid_wait();

  //Bring up rake mech
  rake_up();
  angle_up();
  pros::delay(100);

  chassis.pid_drive_set(23_in, 80);
  pros::delay(100);
  intake_speed(-127);
  pros::delay(250);
  intake_speed(0);
  chassis.pid_wait();

  chassis.pid_drive_set(-1.5_in, DRIVE_SPEED);
  chassis.pid_wait();

  outtake_up();
  intake_speed(127);
  pros::delay(3000);

  chassis.pid_drive_set(-12_in, DRIVE_SPEED);
  rake_down();
  intake_speed(127);
  angle_down();
  outtake_down();
  // descore_up();
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, 80);
  chassis.pid_wait();

  chassis.pid_drive_set(-17.5_in, DRIVE_SPEED);
  chassis.pid_wait();

  pros::delay(2000);

  chassis.pid_drive_set(3_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_drive_set(-4_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  pros::delay(750);
  intake_speed(-127);
  pros::delay(350);
  intake_speed(127);

  chassis.pid_drive_set(1_in, DRIVE_SPEED, false);
  chassis.pid_wait();
  chassis.pid_drive_set(-3_in, DRIVE_SPEED, false);
  chassis.pid_wait();

  pros::delay(1000);

  chassis.pid_drive_set(5_in, 80);
  chassis.pid_wait();

  rake_up();
  pros::delay(250);

  //Drive out of match loader
  chassis.pid_drive_set(10_in, DRIVE_SPEED);
  chassis.pid_wait();

  //Bring up rake mech
  rake_up();
  angle_up();
  pros::delay(100);

  chassis.pid_drive_set(23_in, 80);
  pros::delay(100);
  intake_speed(-127);
  pros::delay(250);
  intake_speed(0);
  chassis.pid_wait();

  chassis.pid_drive_set(-1.5_in, DRIVE_SPEED);
  chassis.pid_wait();

  outtake_up();
  intake_speed(127);
  pros::delay(3000);

  chassis.pid_drive_set(-12_in, DRIVE_SPEED);
  // rake_down();
  // intake_speed(127);
  // angle_down();
  outtake_down();
  descore_up();
  chassis.pid_wait();

  chassis.pid_turn_set(-45_deg, 50);
  chassis.pid_wait();

  chassis.pid_drive_set(13.5_in, DRIVE_SPEED);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::LEFT_SWING, 0_deg, 65, 10);
  chassis.pid_wait();

  chassis.pid_drive_set(2_in, DRIVE_SPEED, false);
  chassis.pid_wait();

  descore_down();
  pros::delay(250);

  chassis.pid_drive_set(20_in, 65);
  chassis.pid_wait();

}

// . . .