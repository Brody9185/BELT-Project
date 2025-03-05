#pragma once
#include "EZ-Template/drive/drive.hpp"
#include "EZ-Template/util.hpp"
#include "main.h"
#include "lemlib/chassis/chassis.hpp"
#include "pros/rtos.hpp"
#include <new>
using namespace okapi;

/////
// For installation, upgrading, documentations, and tutorials, check out our website!
// https://ez-robotics.github.io/EZ-Template/
/////

// These are out of 127
const int DRIVE_SPEED = 110;
const int TURN_SPEED = 90;
const int SWING_SPEED = 110;

///
// Constants
///
inline void default_constants() {
  // P, I, D, and Start I
  EZchassis.pid_drive_constants_forward_set(27.0,0.0,155);      // Fwd/rev constants, used for odom and non odom motions
  EZchassis.pid_drive_constants_backward_set(20.0, 0.0, 120);         // Fwd/rev constants, used for odom and non odom motions
  EZchassis.pid_heading_constants_set(15.0, 0.0, 0);        // Holds the robot straight while going forward without odom
  EZchassis.pid_turn_constants_set(4.9, 0, 45.0);     // Turn in place constants
  EZchassis.pid_swing_constants_set(7.0, 0.0, 30);           // Swing constants
  EZchassis.pid_odom_angular_constants_set(6.8, 0.0, 70);    // Angular control for odom motions
  EZchassis.pid_odom_boomerang_constants_set(5.8, 0.0, 32.5);  // Angular control for boomerang motions

  //LemLib PID Constants
  ez::PID linPID{0,0,10,0,"LEM Linear"}; //Set the LEM PID Values for Linear Movment
  ez::PID angPID{0,0,3,0,"LEM Angular"}; //Set the LEM PID Values for Angular Movment

  // Exit conditions
  EZchassis.pid_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  EZchassis.pid_swing_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  EZchassis.pid_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 500_ms);
  EZchassis.pid_odom_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 750_ms);
  EZchassis.pid_odom_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 750_ms);
  EZchassis.pid_turn_chain_constant_set(3_deg);
  EZchassis.pid_swing_chain_constant_set(5_deg);
  EZchassis.pid_drive_chain_constant_set(3_in);

  // Slew constants
  EZchassis.slew_turn_constants_set(3_deg, 70);
  EZchassis.slew_drive_constants_set(3_in, 70);
  EZchassis.slew_swing_constants_set(3_in, 80);

  // The amount that turns are prioritized over driving in odom motions
  // - if you have tracking wheels, you can run this higher.  1.0 is the max
  EZchassis.odom_turn_bias_set(0.9);

  EZchassis.odom_look_ahead_set(7_in);           // This is how far ahead in the path the robot looks at
  EZchassis.odom_boomerang_distance_set(16_in);  // This sets the maximum distance away from target that the carrot point can be
  EZchassis.odom_boomerang_dlead_set(0.625);     // This handles how aggressive the end of boomerang motions are

  EZchassis.pid_angle_behavior_set(ez::shortest);  // Changes the default behavior for turning, this defaults it to the shortest path there
}

///
//Right Side Tournament
///
inline void Test(){
EZchassis.pid_odom_set({{0,48},fwd,110});
EZchassis.pid_wait();
}

inline void Red_Solo_WP() {
  ColorSort = 2;
  EZchassis.odom_xyt_set(-12,-57,140);
  ladyBrownPID.target_set(AlianceStake);
  pros::delay(500);
  ladyBrownPID.target_set(1200);
  EZchassis.pid_odom_set(-4,DRIVE_SPEED);
  EZchassis.pid_wait();
  EZchassis.pid_odom_set({{0,-40},fwd,35});
  EZchassis.pid_wait();
  setIntake(127);
  pros::delay(100);
  intakeHM.move(127);
  pros::delay(200);
  EZchassis.pid_odom_set({{-36,-48},rev,70});
  EZchassis.pid_wait();
  EZchassis.pid_odom_set({{-24,-24},rev,70});
  EZchassis.pid_wait();
  clampP.set(true);
  pros::delay(200);
  setIntake(-127);
  EZchassis.pid_odom_set({{-46,-24},fwd,DRIVE_SPEED});
  EZchassis.pid_wait();
  setIntake(-127);
  EZchassis.pid_odom_set({{-48,-6},fwd,70});
  EZchassis.pid_wait();
  setIntake(-127);
  EZchassis.pid_odom_set({{-54,-13},rev,70});
  EZchassis.pid_wait();
  EZchassis.pid_odom_set({{-56,-6},fwd,70});
  EZchassis.pid_wait();
  pros::delay(500);
  EZchassis.pid_odom_set({{-52,-13},rev,70});
  EZchassis.pid_wait();
  setIntake(0);
  EZchassis.pid_odom_set({{-12,-8},fwd,90});
  setIntake(-127);
  EZchassis.pid_wait();
  EZchassis.pid_odom_set({{-8,-4},fwd,70});
  EZchassis.pid_wait();
  pros::delay(500);
  setIntake(-127);
  EZchassis.pid_odom_set({{-9,-8},rev,70});
  EZchassis.pid_wait();
  setIntake(-127);
  EZchassis.pid_odom_set({{8,-4},fwd,70});
  EZchassis.pid_wait();
}

inline void Blue_Solo_WP() {
  EZchassis.odom_xyt_set(-12,57,40);
  ladyBrownPID.target_set(AlianceStake);
  pros::delay(500);
  ladyBrownPID.target_set(1200);
  EZchassis.pid_odom_set(-4,DRIVE_SPEED);
  EZchassis.pid_wait();
  EZchassis.pid_odom_set({{0,46},fwd,40});
  EZchassis.pid_wait();
  setIntake(127);
  pros::delay(100);
  intakeHM.move(127);
  pros::delay(200);
  EZchassis.pid_odom_set({{-36,48},rev,70});
  EZchassis.pid_wait();
  EZchassis.pid_odom_set({{-24,25},rev,70});
  EZchassis.pid_wait();
  clampP.set(true);
  pros::delay(200);
  setIntake(-127);
  EZchassis.pid_odom_set({{-46,24},fwd,DRIVE_SPEED});
  pros::delay(350);
  setIntake(127);
  pros::delay(150);
  setIntake(-127);
  EZchassis.pid_wait();
  setIntake(-127);
  EZchassis.pid_odom_set({{-46,7},fwd,70});
  EZchassis.pid_wait();
  pros::delay(350);
  setIntake(127);
  pros::delay(150);
  setIntake(-127);
  EZchassis.pid_odom_set({{-54,13},rev,70});
  EZchassis.pid_wait();
  setIntake(127);
  pros::delay(250);
  setIntake(-127);
  EZchassis.pid_odom_set({{-54,6},fwd,70});
  EZchassis.pid_wait();
  pros::delay(500);
  EZchassis.pid_odom_set({{-52,13},rev,70});
  EZchassis.pid_wait();
  EZchassis.pid_odom_set({{0,24},fwd,90});
  EZchassis.pid_wait();
}

inline void Red_Tournament() {
  EZchassis.odom_xyt_set(-12,-57,140);
  EZchassis.pid_odom_set({{-36,-48},rev,70});
  EZchassis.pid_wait();
  EZchassis.pid_odom_set({{-24,-25},rev,70});
  EZchassis.pid_wait();
  clampP.set(true);
  pros::delay(200);
  setIntake(-127);
  EZchassis.pid_odom_set({{-46,-24},fwd,DRIVE_SPEED});
  pros::delay(350);
  setIntake(127);
  pros::delay(150);
  setIntake(-127);
  EZchassis.pid_wait();
  setIntake(-127);
  EZchassis.pid_odom_set({{-46,-7},fwd,70});
  EZchassis.pid_wait();
  pros::delay(350);
  setIntake(127);
  pros::delay(150);
  setIntake(-127);
  EZchassis.pid_odom_set({{-54,-13},rev,70});
  EZchassis.pid_wait();
  setIntake(127);
  pros::delay(250);
  setIntake(-127);
  EZchassis.pid_odom_set({{-54,-6},fwd,70});
  EZchassis.pid_wait();
  pros::delay(500);
  EZchassis.pid_odom_set({{-52,-13},rev,70});
  EZchassis.pid_wait();
  EZchassis.pid_odom_set({{0,-24},fwd,90});
  EZchassis.pid_wait();
}

inline void Blue_Tournament() {
  EZchassis.odom_xyt_set(24,60,180);
  EZchassis.pid_odom_set({{0,48},fwd,60});
  EZchassis.pid_wait();
  setIntake(127);
  pros::delay(75);
  setIntake(0);
  intakeHM.move(127);
  pros::delay(200);
  EZchassis.pid_odom_set({{18,42},rev,70});
  EZchassis.pid_wait();
  EZchassis.pid_odom_set({{26,22},rev,70});
  EZchassis.pid_wait();
  clampP.set(true);
  pros::delay(200);
  setIntake(-127);
  EZchassis.pid_odom_set({{48,24},fwd,DRIVE_SPEED});
  EZchassis.pid_wait();
  setIntake(-127);
  EZchassis.pid_odom_set({{44,7},fwd,DRIVE_SPEED});
  EZchassis.pid_wait();
  setIntake(-127);
  EZchassis.pid_odom_set({{44,15},fwd,DRIVE_SPEED});
  EZchassis.pid_wait();
  setIntake(-127);
  EZchassis.pid_odom_set({{52,7},fwd,DRIVE_SPEED});
  EZchassis.pid_wait();
}

inline void Red_3_Ring() {
  EZchassis.odom_xyt_set(24,-60,0);
  EZchassis.pid_odom_set({{30,-42},rev,DRIVE_SPEED});
  EZchassis.pid_wait();
  EZchassis.pid_odom_set({{26,-22},rev,70});
  EZchassis.pid_wait();
  clampP.set(true);
  pros::delay(200);
  EZchassis.pid_odom_set({{0,-24},rev,DRIVE_SPEED});
  EZchassis.pid_wait();
  EZchassis.pid_odom_set({{48,-24},fwd,DRIVE_SPEED});
  pros::delay(750);
  setIntake(-127);
  EZchassis.pid_wait();
  setIntake(-127);
}

inline void Blue_3_Ring() {
  EZchassis.odom_xyt_set(24,60,0);
  EZchassis.pid_odom_set({{30,42},rev,DRIVE_SPEED});
  EZchassis.pid_wait();
  EZchassis.pid_odom_set({{26,22},rev,70});
  EZchassis.pid_wait();
  clampP.set(true);
  pros::delay(200);
  EZchassis.pid_odom_set({{0,24},rev,DRIVE_SPEED});
  EZchassis.pid_wait();
  EZchassis.pid_odom_set({{48,24},fwd,DRIVE_SPEED});
  pros::delay(750);
  setIntake(-127);
  EZchassis.pid_wait();
  setIntake(-127);
}

inline void Skills() {
  EZchassis.odom_xyt_set(12,-57,180);
  EZchassis.pid_turn_relative_set(55,TURN_SPEED);
  EZchassis.pid_wait();
  ladyBrownPID.target_set(AlianceStake);
  pros::delay(500);
  ladyBrownPID.target_set(1200);
  EZchassis.pid_odom_set({{0,-48},rev,90});
  EZchassis.pid_wait();
  EZchassis.pid_odom_set({{24,-48},rev,90});
  EZchassis.pid_wait();
  clampP.set(true);
  pros::delay(200);
  setIntake(-127);
  EZchassis.pid_odom_set({{24,-24},fwd,90});
  EZchassis.pid_wait();
  setIntake(-127);
  EZchassis.pid_odom_set({{48,-24},fwd,90});
  EZchassis.pid_wait();
  EZchassis.pid_odom_set({{48,-48},fwd,90});
  EZchassis.pid_wait();
  EZchassis.pid_odom_set({{48,-52},fwd,90});
  EZchassis.pid_wait();
  EZchassis.pid_odom_set({{56,-56},rev,90, ez::right_turn});
  EZchassis.pid_wait();
  clampP.set(false);
  setIntake(127);
  pros::delay(100);
  setIntake(0);
  EZchassis.pid_odom_set({{0,-48},fwd,90});
  EZchassis.pid_wait();
  EZchassis.pid_odom_set({{-24,-48},rev,90});
  EZchassis.pid_wait();
  clampP.set(true);
  pros::delay(200);
  setIntake(-127);
  EZchassis.pid_odom_set({{-24,-24},fwd,90});
  EZchassis.pid_wait();
  setIntake(-127);
  EZchassis.pid_odom_set({{-48,-24},fwd,90});
  EZchassis.pid_wait();
  setIntake(-127);
  EZchassis.pid_odom_set({{-48,-48},fwd,90});
  EZchassis.pid_wait();
  setIntake(-127);
  EZchassis.pid_odom_set({{-48,-52},fwd,90});
  EZchassis.pid_wait();
  setIntake(-127);
  EZchassis.pid_odom_set({{-56,-56},rev,90,ez::left_turn});
  EZchassis.pid_wait();
  clampP.set(false);
  setIntake(127);
  pros::delay(100);
  setIntake(-127);
  EZchassis.pid_odom_set({{-59,0},fwd,90});
  EZchassis.pid_wait();
  setIntake(0);
  intakeHM.move(127);
  EZchassis.pid_odom_set({{-48,24},fwd,90});
  EZchassis.pid_wait();
  EZchassis.pid_odom_set({{-24,60},rev,90});
  EZchassis.pid_wait();
  clampP.set(true);
  pros::delay(200);
  EZchassis.pid_odom_set({{-56,56},rev,90});
  EZchassis.pid_wait();
  clampP.set(false);
  EZchassis.pid_odom_set({{-24,48},fwd,90});
  EZchassis.pid_wait();
  EZchassis.pid_odom_set({{0,48},rev,90});
  EZchassis.pid_wait();
  clampP.set(true);
  pros::delay(200);
  EZchassis.pid_turn_set(0, TURN_SPEED);
  EZchassis.pid_wait();
  EZchassis.pid_drive_set(6,90);
  EZchassis.pid_wait();
  ladyBrownPID.target_set(Loading);
  pros::delay(100);
  ladyBrownPID.target_set(10);
  pros::delay(100);
  ladyBrownPID.target_set(Loading);
  ladyBrownWait();
  setIntake(-127);
  pros::delay(200);
  setIntake(0);
  ladyBrownPID.target_set(AlianceStake);
  ladyBrownWait();
  ladyBrownPID.target_set(Resting);
  EZchassis.pid_odom_set({{0,48},rev,90});
  EZchassis.pid_wait();
  setIntake(-127);
  EZchassis.pid_odom_set({{24,24},fwd,90});
  EZchassis.pid_wait();
  EZchassis.pid_odom_set({{48,24},fwd,90});
  EZchassis.pid_wait();
  EZchassis.pid_odom_set({{59,48},fwd,90});
  EZchassis.pid_wait();
  EZchassis.pid_odom_set({{56,56},rev,90});
  EZchassis.pid_wait();
  clampP.set(false);
  setIntake(127);
  pros::delay(100);
  setIntake(-127);
  ladyBrownPID.target_set(Loading);
  EZchassis.pid_odom_set({{60,0},fwd,90});
  EZchassis.pid_wait();
  EZchassis.pid_odom_set({{24,24},rev,90});
  EZchassis.pid_wait();
  EZchassis.pid_odom_set({{60,0,90},fwd,90});
  EZchassis.pid_wait();
  ladyBrownPID.target_set(WallStake);
  ladyBrownWait();
}





///
// Drive Example
///
inline void drive_example() {
  // The first parameter is target inches
  // The second parameter is max speed the robot will drive at
  // The third parameter is a boolean (true or false) for enabling/disabling a slew at the start of drive motions
  // for slew, only enable it when the drive distance is greater than the slew distance + a few inches

  EZchassis.pid_odom_set(48_in, DRIVE_SPEED);
  EZchassis.pid_wait();
}

///
// Turn Example
///
inline void turn_example() {
  // The first parameter is the target in degrees
  // The second parameter is max speed the robot will drive at

  EZchassis.pid_turn_set(90_deg, TURN_SPEED);
  EZchassis.pid_wait();

  EZchassis.pid_turn_set(45_deg, TURN_SPEED);
  EZchassis.pid_wait();

  EZchassis.pid_turn_set(0_deg, TURN_SPEED);
  EZchassis.pid_wait();
}

///
// Combining Turn + Drive
///
inline void drive_and_turn() {
  EZchassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  EZchassis.pid_wait();

  EZchassis.pid_turn_set(45_deg, TURN_SPEED);
  EZchassis.pid_wait();

  EZchassis.pid_turn_set(-45_deg, TURN_SPEED);
  EZchassis.pid_wait();

  EZchassis.pid_turn_set(0_deg, TURN_SPEED);
  EZchassis.pid_wait();

  EZchassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  EZchassis.pid_wait();
}

///
// Wait Until and Changing Max Speed
///
inline void wait_until_change_speed() {
  // pid_wait_until will wait until the robot gets to a desired position

  // When the robot gets to 6 inches slowly, the robot will travel the remaining distance at full speed
  EZchassis.pid_drive_set(24_in, 30, true);
  EZchassis.pid_wait_until(6_in);
  EZchassis.pid_speed_max_set(DRIVE_SPEED);  // After driving 6 inches at 30 speed, the robot will go the remaining distance at DRIVE_SPEED
  EZchassis.pid_wait();

  EZchassis.pid_turn_set(45_deg, TURN_SPEED);
  EZchassis.pid_wait();

  EZchassis.pid_turn_set(-45_deg, TURN_SPEED);
  EZchassis.pid_wait();

  EZchassis.pid_turn_set(0_deg, TURN_SPEED);
  EZchassis.pid_wait();

  // When the robot gets to -6 inches slowly, the robot will travel the remaining distance at full speed
  EZchassis.pid_drive_set(-24_in, 30, true);
  EZchassis.pid_wait_until(-6_in);
  EZchassis.pid_speed_max_set(DRIVE_SPEED);  // After driving 6 inches at 30 speed, the robot will go the remaining distance at DRIVE_SPEED
  EZchassis.pid_wait();
}

///
// Swing Example
///
inline void swing_example() {
  // The first parameter is ez::LEFT_SWING or ez::RIGHT_SWING
  // The second parameter is the target in degrees
  // The third parameter is the speed of the moving side of the drive
  // The fourth parameter is the speed of the still side of the drive, this allows for wider arcs

  EZchassis.pid_swing_set(ez::LEFT_SWING, 45_deg, SWING_SPEED, 45);
  EZchassis.pid_wait();

  EZchassis.pid_swing_set(ez::RIGHT_SWING, 0_deg, SWING_SPEED, 45);
  EZchassis.pid_wait();

  EZchassis.pid_swing_set(ez::RIGHT_SWING, 45_deg, SWING_SPEED, 45);
  EZchassis.pid_wait();

  EZchassis.pid_swing_set(ez::LEFT_SWING, 0_deg, SWING_SPEED, 45);
  EZchassis.pid_wait();
}

///
// Motion Chaining
///
inline void motion_chaining() {
  // Motion chaining is where motions all try to blend together instead of individual movements.
  // This works by exiting while the robot is still moving a little bit.
  // To use this, replace pid_wait with pid_wait_quick_chain.
  EZchassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  EZchassis.pid_wait();

  EZchassis.pid_turn_set(45_deg, TURN_SPEED);
  EZchassis.pid_wait_quick_chain();

  EZchassis.pid_turn_set(-45_deg, TURN_SPEED);
  EZchassis.pid_wait_quick_chain();

  EZchassis.pid_turn_set(0_deg, TURN_SPEED);
  EZchassis.pid_wait();

  // Your final motion should still be a normal pid_wait
  EZchassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  EZchassis.pid_wait();
}

///
// Auto that tests everything
///
inline void combining_movements() {
  EZchassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  EZchassis.pid_wait();

  EZchassis.pid_turn_set(45_deg, TURN_SPEED);
  EZchassis.pid_wait();

  EZchassis.pid_swing_set(ez::RIGHT_SWING, -45_deg, SWING_SPEED, 45);
  EZchassis.pid_wait();

  EZchassis.pid_turn_set(0_deg, TURN_SPEED);
  EZchassis.pid_wait();

  EZchassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  EZchassis.pid_wait();
}

///
// Interference example
///
inline void tug(int attempts) {
  for (int i = 0; i < attempts - 1; i++) {
    // Attempt to drive backward
    printf("i - %i", i);
    EZchassis.pid_drive_set(-12_in, 127);
    EZchassis.pid_wait();

    // If failsafed...
    if (EZchassis.interfered) {
      EZchassis.drive_sensor_reset();
      EZchassis.pid_drive_set(-2_in, 20);
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
inline void interfered_example() {
  EZchassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  EZchassis.pid_wait();

  if (EZchassis.interfered) {
    tug(3);
    return;
  }

  EZchassis.pid_turn_set(90_deg, TURN_SPEED);
  EZchassis.pid_wait();
}

///
// Odom Drive PID
///
inline void odom_drive_example() {
  // This works the same as pid_drive_set, but it uses odom instead!
  // You can replace pid_drive_set with pid_odom_set and your robot will
  // have better error correction.

  EZchassis.pid_odom_set(24_in, DRIVE_SPEED, true);
  EZchassis.pid_wait();

  EZchassis.pid_odom_set(-12_in, DRIVE_SPEED);
  EZchassis.pid_wait();

  EZchassis.pid_odom_set(-12_in, DRIVE_SPEED);
  EZchassis.pid_wait();
}

///
// Odom Pure Pursuit
///
inline void odom_pure_pursuit_example() {
  // Drive to 0, 30 and pass through 6, 10 and 0, 20 on the way, with slew
  EZchassis.pid_odom_set({{{0, 24}, fwd, DRIVE_SPEED},
                        {{24, 24}, fwd,DRIVE_SPEED},
                        {{0, 24}, rev, DRIVE_SPEED}});
  EZchassis.pid_wait();

  // Drive to 0, 0 backwards
  //EZchassis.pid_odom_set({{0_in, 0_in}, rev, DRIVE_SPEED});
  //EZchassis.pid_wait();
}

///
// Odom Pure Pursuit Wait Until
///
inline void odom_pure_pursuit_wait_until_example() {
  EZchassis.pid_odom_set({{{0_in, 24_in}, fwd, DRIVE_SPEED},
                        {{12_in, 24_in}, fwd, DRIVE_SPEED},
                        {{24_in, 24_in}, fwd, DRIVE_SPEED}},
                       true);
  EZchassis.pid_wait_until_index(1);  // Waits until the robot passes 12, 24
  // Intake.move(127);  // Set your intake to start moving once it passes through the second point in the index
  EZchassis.pid_wait();
  // Intake.move(0);  // Turn the intake off
}

///
// Odom Boomerang
///
inline void odom_boomerang_example() {
  EZchassis.pid_odom_set({{0_in, 24_in, 45_deg}, fwd, DRIVE_SPEED},
                       true);
  EZchassis.pid_wait();

  EZchassis.pid_odom_set({{0_in, 0_in, 0_deg}, rev, DRIVE_SPEED},
                       true);
  EZchassis.pid_wait();
}

///
// Odom Boomerang Injected Pure Pursuit
///
inline void odom_boomerang_injected_pure_pursuit_example() {
  EZchassis.pid_odom_set({{{0_in, 24_in, 45_deg}, fwd, DRIVE_SPEED},
                        {{12_in, 24_in}, fwd, DRIVE_SPEED},
                        {{24_in, 24_in}, fwd, DRIVE_SPEED}},
                       true);
  EZchassis.pid_wait();

  EZchassis.pid_odom_set({{0_in, 0_in, 0_deg}, rev, DRIVE_SPEED},
                       true);
  EZchassis.pid_wait();
}

///
// Calculate the offsets of your tracking wheels
///
inline void measure_offsets() {
  // Number of times to test
  int iterations = 10;

  // Our final offsets
  double l_offset = 0.0, r_offset = 0.0, b_offset = 0.0, f_offset = 0.0;

  // Reset all trackers if they exist
  if (EZchassis.odom_tracker_left != nullptr) EZchassis.odom_tracker_left->reset();
  if (EZchassis.odom_tracker_right != nullptr) EZchassis.odom_tracker_right->reset();
  if (EZchassis.odom_tracker_back != nullptr) EZchassis.odom_tracker_back->reset();
  if (EZchassis.odom_tracker_front != nullptr) EZchassis.odom_tracker_front->reset();
  
  for (int i = 0; i < iterations; i++) {
    // Reset pid targets and get ready for running an auton
    EZchassis.pid_targets_reset();
    EZchassis.drive_imu_reset();
    EZchassis.drive_sensor_reset();
    EZchassis.drive_brake_set(pros::E_MOTOR_BRAKE_HOLD);
    EZchassis.odom_xyt_set(0_in, 0_in, 0_deg);
    double imu_start = EZchassis.odom_theta_get();
    double target = i % 2 == 0 ? 90 : 270;  // Switch the turn target every run from 270 to 90

    // Turn to target at half power
    EZchassis.pid_turn_set(target, 63, ez::raw);
    EZchassis.pid_wait();
    pros::delay(250);

    // Calculate delta in angle
    double t_delta = util::to_rad(fabs(util::wrap_angle(EZchassis.odom_theta_get() - imu_start)));

    // Calculate delta in sensor values that exist
    double l_delta = EZchassis.odom_tracker_left != nullptr ? EZchassis.odom_tracker_left->get() : 0.0;
    double r_delta = EZchassis.odom_tracker_right != nullptr ? EZchassis.odom_tracker_right->get() : 0.0;
    double b_delta = EZchassis.odom_tracker_back != nullptr ? EZchassis.odom_tracker_back->get() : 0.0;
    double f_delta = EZchassis.odom_tracker_front != nullptr ? EZchassis.odom_tracker_front->get() : 0.0;

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
  if (EZchassis.odom_tracker_left != nullptr) EZchassis.odom_tracker_left->distance_to_center_set(l_offset);
  if (EZchassis.odom_tracker_right != nullptr) EZchassis.odom_tracker_right->distance_to_center_set(r_offset);
  if (EZchassis.odom_tracker_back != nullptr) EZchassis.odom_tracker_back->distance_to_center_set(b_offset);
  if (EZchassis.odom_tracker_front != nullptr) EZchassis.odom_tracker_front->distance_to_center_set(f_offset);
}

// . . .
// Make your own autonomous functions here!
// . . .