#include "main.h"
#include "lemlib/chassis/chassis.hpp"
#include "pros/rtos.hpp"
#include <new>
using namespace okapi;
//#include "EZ-Template/PID.hpp"

// The Purpose for this File is to Store Values for EZ and LemLib PID so that Autonomous Routes can be Created using Ez and Lemlib Drive Commands.

//GPS Code
//setStrip1(0,0); // Set the new 0,0 from the bottom left corner. Use this when placing the bot against the 1st strip.
//setStrip2(0,0); // Set the new 0,0 from the bottom left corner. Use this when placing the bot against the 2nd strip.
//setStrip3(0,0); // Set the new 0,0 from the bottom left corner. Use this when placing the bot against the 3rd strip.
//setStrip4(0,0); // Set the new 0,0 from the bottom left corner. Use this when placing the bot against the 4th strip.
//Lemchassis.setPose(GPSX,GPSY,GPSH); // Set the Lemlib starting position based on the GPS(note: you can also use this during the code to recalibrate your position).

// Defines PID Constains for both EZ and LemLib
void defaultConstants() {
  // EZ Forward PID Constants
  EZchassis.pid_drive_constants_forward_set(16.5, 0, 3); // Sets forward
  EZchassis.pid_drive_constants_backward_set(16.5, 0, 3); // Sets backward
  EZchassis.pid_heading_constants_set(8, 0, 23); // Sets Heading
  EZchassis.pid_turn_constants_set(3.6, 0, 20); // Sets Turn
  EZchassis.pid_swing_constants_forward_set(5, 0, 30); // Sets forward Swing
  EZchassis.pid_swing_constants_backward_set(5, 0, 30); // Sets backward Swing
  EZchassis.slew_drive_constants_set(20_in, 65); //Controls how soon, and how vilently the robot will decelerate.
  EZchassis.pid_odom_turn_exit_condition_set(80_ms, 3_deg, 250_ms, 7_deg, 500_ms, 750_ms);
  EZchassis.pid_odom_drive_exit_condition_set(80_ms, 1_in, 250_ms, 3_in, 500_ms, 750_ms);

  /*// EZ Forbackward PID Constants
  EZBackwardchassis.pid_drive_constants_forward_set(3.3, 0, 0); // Sets forward
  EZBackwardchassis.pid_drive_constants_backward_set(3.4, 0, 0); // Sets backward
  EZBackwardchassis.pid_heading_constants_set(5, 0, 18); // Sets Heading
  EZBackwardchassis.pid_turn_constants_set(3.3, 0, 20); // Sets Turn
  EZBackwardchassis.pid_swing_constants_forward_set(5, 0, 30); // Sets forward Swing
  EZBackwardchassis.pid_swing_constants_backward_set(5, 0, 30); // Sets backward Swing
  */

  //LemLib PID Constants
  ez::PID linPID{0,0,10,0,"LEM Linear"}; //Set the LEM PID Values for Linear Movment
  ez::PID angPID{0,0,3,0,"LEM Angular"}; //Set the LEM PID Values for Angular Movment
}

// Test Example Autonomous
void testAuton(){
  EZchassis.drive_brake_set(MOTOR_BRAKE_HOLD); // Sets Drive Break to Brake Hole from EZ Template.
  EZchassis.pid_odom_set(48,110, true);
  EZchassis.pid_wait();
}

// Angular PID Example Autonomous
void left_side_blue() {
  Color = false;
  EZchassis.drive_brake_set(MOTOR_BRAKE_HOLD); // Sets Drive Break to Brake Hole from EZ Template.
  ladyBrownM.move(-127);
  pros::delay(1000);
  ladyBrownM.move(0);
  EZchassis.pid_odom_set(-12,115);
  EZchassis.pid_wait();
  ladyBrownM.move(127);
  EZchassis.pid_turn_relative_set(58,100);
  EZchassis.pid_wait();
  EZchassis.pid_odom_set(-31,85);
  EZchassis.pid_wait();
  ladyBrownM.move(0);
  pros::delay(500);
  clampP.set(true);
  pros::delay(500);
  EZchassis.pid_turn_relative_set(85,100);
  EZchassis.pid_wait();
  setIntake(-127);
  EZchassis.pid_odom_set(18,80);
  EZchassis.pid_wait();
  pros::delay(750);
  setIntake(0);
  EZchassis.pid_turn_relative_set(83,100);
  EZchassis.pid_wait();
  setIntake(-127);
  EZchassis.pid_odom_set(14,80);
  EZchassis.pid_wait();
  pros::delay(800);
  EZchassis.pid_turn_relative_set(110,100);
  EZchassis.pid_wait();
  EZchassis.pid_odom_set(30,127);
}

// Right Side Example Autonomous
void right_side_blue(){
  Color = false;
  EZchassis.drive_brake_set(MOTOR_BRAKE_HOLD); // Sets Drive Break to Brake Hole from EZ Template.
  ladyBrownM.move(-127);
  pros::delay(1000);
  ladyBrownM.move(0);
  EZchassis.pid_odom_set(-12,115);
  EZchassis.pid_wait();
  ladyBrownM.move(127);
  EZchassis.pid_turn_relative_set(-58,100);
  EZchassis.pid_wait();
  EZchassis.pid_odom_set(-31,85);
  EZchassis.pid_wait();
  ladyBrownM.move(0);
  pros::delay(500);
  clampP.set(true);
  pros::delay(500);
  EZchassis.pid_turn_relative_set(-85,100);
  EZchassis.pid_wait();
  setIntake(-127);
  EZchassis.pid_odom_set(18,80);
  EZchassis.pid_wait();
  pros::delay(750);
  setIntake(0);
  EZchassis.pid_turn_relative_set(-83,100);
  EZchassis.pid_wait();
  setIntake(-127);
  EZchassis.pid_odom_set(14,80);
  EZchassis.pid_wait();
  pros::delay(800);
  EZchassis.pid_turn_relative_set(-110,100);
  EZchassis.pid_wait();
  EZchassis.pid_odom_set(30,127);
}

// Left Side Example Autonomous
void left_side_red() {
  Color = true;
  EZchassis.drive_brake_set(MOTOR_BRAKE_HOLD); // Sets Drive Break to Brake Hole from EZ Template.
  ladyBrownM.move(-127);
  pros::delay(1000);
  ladyBrownM.move(0);
  EZchassis.pid_odom_set(-12,115);
  EZchassis.pid_wait();
  ladyBrownM.move(127);
  EZchassis.pid_turn_relative_set(58,100);
  EZchassis.pid_wait();
  EZchassis.pid_odom_set(-31,85);
  EZchassis.pid_wait();
  ladyBrownM.move(0);
  pros::delay(500);
  clampP.set(true);
  pros::delay(500);
  EZchassis.pid_turn_relative_set(85,100);
  EZchassis.pid_wait();
  setIntake(-127);
  EZchassis.pid_odom_set(18,80);
  EZchassis.pid_wait();
  pros::delay(750);
  setIntake(0);
  EZchassis.pid_turn_relative_set(83,100);
  EZchassis.pid_wait();
  setIntake(-127);
  EZchassis.pid_odom_set(14,80);
  EZchassis.pid_wait();
  pros::delay(800);
  EZchassis.pid_turn_relative_set(110,100);
  EZchassis.pid_wait();
  EZchassis.pid_odom_set(30,127);
}

// Right Side Example Autonomous
void right_side_red(){
  Color = true;
  EZchassis.drive_brake_set(MOTOR_BRAKE_HOLD); // Sets Drive Break to Brake Hole from EZ Template.
  ladyBrownM.move(-127);
  pros::delay(1000);
  ladyBrownM.move(0);
  EZchassis.pid_odom_set(-12,115);
  EZchassis.pid_wait();
  ladyBrownM.move(127);
  EZchassis.pid_turn_relative_set(-58,100);
  EZchassis.pid_wait();
  EZchassis.pid_odom_set(-31,85);
  EZchassis.pid_wait();
  ladyBrownM.move(0);
  pros::delay(500);
  clampP.set(true);
  pros::delay(500);
  EZchassis.pid_turn_relative_set(-85,100);
  EZchassis.pid_wait();
  setIntake(-127);
  EZchassis.pid_odom_set(18,80);
  EZchassis.pid_wait();
  pros::delay(750);
  setIntake(0);
  EZchassis.pid_turn_relative_set(-83,100);
  EZchassis.pid_wait();
  setIntake(-127);
  EZchassis.pid_odom_set(14,80);
  EZchassis.pid_wait();
  pros::delay(800);
  EZchassis.pid_turn_relative_set(-110,100);
  EZchassis.pid_wait();
  EZchassis.pid_odom_set(30,127);
}
// Skill Autonomous Example Autonomous
void skills_auto(){
  EZchassis.drive_brake_set(MOTOR_BRAKE_HOLD); // Sets Drive Break to Brake Hole from EZ Template.
  
}

// Do Nothing Example Autonomous
void do_nothing() {

}

// Position Example Autonomous
void pos_example(){
  updatePID(); // Sets EZ PID Values to be Eqaul to LemLib PID Values - ALWAYS CALL FIRST.
  EZchassis.pid_drive_toggle(false); // Disable PID Drive Toggle from EZ Template.
  LEMchassis.setPose(0,0,0);
  LEMchassis.moveToPose(10,10,90,4000);
}

// Point Example Autonomous
void point_example(){
  updatePID(); // Sets EZ PID Values to be Eqaul to LemLib PID Values - ALWAYS CALL FIRST.
  EZchassis.pid_drive_toggle(false); // Disable PID Drive Toggle from EZ Template.
  LEMchassis.setPose(0,0,0); // Sets Position of Where the Robot Starts to (0,0).
  LEMchassis.moveToPoint(10,10, 4000); // Sets the New Position for the Robot to Move to - Robot Continues to Face this Direction.
}