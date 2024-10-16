#include "main.h"
#include "lemlib/chassis/chassis.hpp"
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
  EZchassis.pid_drive_constants_forward_set(3.3, 0, 0); // Sets forward
  EZchassis.pid_drive_constants_backward_set(0, 0, 0); // Sets backward
  EZchassis.pid_heading_constants_set(5, 0, 18); // Sets Heading
  EZchassis.pid_turn_constants_set(3.3, 0, 20); // Sets Turn
  EZchassis.pid_swing_constants_forward_set(5, 0, 30); // Sets forward Swing
  EZchassis.pid_swing_constants_backward_set(5, 0, 30); // Sets backward Swing

  // EZ Forward PID Constants
  EZBackwardchassis.pid_drive_constants_forward_set(0, 0, 0); // Sets forward
  EZBackwardchassis.pid_drive_constants_backward_set(3.4, 0, 0); // Sets backward
  EZBackwardchassis.pid_heading_constants_set(5, 0, 18); // Sets Heading
  EZBackwardchassis.pid_turn_constants_set(3, 0, 20); // Sets Turn
  EZBackwardchassis.pid_swing_constants_forward_set(5, 0, 30); // Sets forward Swing
  EZBackwardchassis.pid_swing_constants_backward_set(5, 0, 30); // Sets backward Swing

  //EZchassis.pid_turn_exit_condition_set(0, 0, 0, 0, 0, 0);
  //EZchassis.pid_swing_exit_condition_set(0, 0, 0, 0, 0, 0);
  //EZchassis.pid_drive_exit_condition_set(0, 0, 0, 0, 0, 0);
  //EZchassis.slew_drive_constants_set(35_in, 65);
  //LemLib PID Constants
  ez::PID linPID{0,0,10,0,"LEM Linear"}; //Set the LEM PID Values for Linear Movment
  ez::PID angPID{0,0,3,0,"LEM Angular"}; //Set the LEM PID Values for Angular Movment
}

// Test Example Autonomous
void testAuton(){
  EZchassis.drive_brake_set(MOTOR_BRAKE_HOLD); // Sets Drive Break to Brake Hole from EZ Template.
  EZBackwardchassis.drive_brake_set(MOTOR_BRAKE_HOLD);
  EZchassis.pid_drive_toggle(true); // Enables PID Forward Drive Toggle from EZ Template.
  EZBackwardchassis.pid_drive_toggle(false); // Disables PID Backward Drive Toggle from EZ Template.
  EZchassis.pid_drive_set(48, 110, true); // Robot Turns 45 Degrees to the Left.
  EZchassis.pid_wait();
  EZchassis.pid_drive_toggle(false); // Disables PID Forward Drive Toggle from EZ Template.
  EZBackwardchassis.pid_drive_toggle(true); // Enables PID Backward Drive Toggle from EZ Template.
  EZBackwardchassis.pid_drive_set(-48,110,true);
  EZBackwardchassis.pid_wait();
}

// Angular PID Example Autonomous
void ang_PID() {
  EZchassis.drive_brake_set(MOTOR_BRAKE_HOLD); // Sets Drive Break to Brake Hole from EZ Template.
  EZBackwardchassis.drive_brake_set(MOTOR_BRAKE_HOLD);
  EZchassis.pid_drive_toggle(true); // Enables PID Forward Drive Toggle from EZ Template.
  EZBackwardchassis.pid_drive_toggle(false); // Disables PID Backward Drive Toggle from EZ Template.
  EZchassis.pid_turn_set(90,110);
  EZchassis.pid_wait();
  pros::delay(2500);
  EZchassis.pid_turn_set(180,110);
  EZchassis.pid_wait();
  pros::delay(2500);
  EZchassis.pid_turn_set(270,110);
  EZchassis.pid_wait();
  pros::delay(2500);
  EZchassis.pid_turn_set(360,110);
  EZchassis.pid_wait();
}

// Linear PID Example Autonomous
void lin_PID(){
  updatePID(); // Sets EZ PID Values to be Eqaul to LemLib PID Values - ALWAYS CALL FIRST.
  EZchassis.pid_drive_toggle(false); //disable PID from EZ Template 
  LEMchassis.setPose(0,0,0);//set position to 0,0 and heading 0 
  LEMchassis.moveToPoint(0,48,10000); //go forward 48 inches
}

// Left Side Example Autonomous
void left_side() {
  updatePID(); // Sets EZ PID Values to be Eqaul to LemLib PID Values - ALWAYS CALL FIRST.
  EZchassis.pid_drive_toggle(false); // Disable PID Drive Toggle from EZ Template.

}

// Right Side Example Autonomous
void right_side(){
  updatePID(); // Sets EZ PID Values to be Eqaul to LemLib PID Values - ALWAYS CALL FIRST.
  EZchassis.pid_drive_toggle(false); // Disable PID Drive Toggle from EZ Template.
}

// Skill Autonomous Example Autonomous
void skills_auto(){
  updatePID(); // Sets EZ PID Values to be Eqaul to LemLib PID Values - ALWAYS CALL FIRST.
  EZchassis.pid_drive_toggle(false); // Disable PID Drive Toggle from EZ Template.
}

// Do Nothing Example Autonomous
void do_nothing() {
  updatePID(); // Sets EZ PID Values to be Eqaul to LemLib PID Values - ALWAYS CALL FIRST.
  EZchassis.pid_drive_toggle(false); // Disable PID Drive Toggle from EZ Template.
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