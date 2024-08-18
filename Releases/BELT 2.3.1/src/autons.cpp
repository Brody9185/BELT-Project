#include "main.h"
#include "lemlib/chassis/chassis.hpp"

// The Purpose for this File is to Store Values for EZ and LemLib PID so that Autonomous Routes can be Created using Ez and Lemlib Drive Commands.

// Locks the Code into a While Loop until the Robot has Settled - A universal version 'chassis.pid_wait()'
void allPIDWait() {
LEMchassis.waitUntilDone();
EZchassis.pid_wait();
}

// Defines PID Constains for both EZ and LemLib
void defaultConstants() {
  // EZ PID Constants
  EZchassis.pid_drive_constants_forward_set(10, 0, 100); // Sets forward
  EZchassis.pid_drive_constants_backward_set(10, 0, 100); // Sets backward
  EZchassis.pid_heading_constants_set(3, 0, 20); // Sets Heading
  EZchassis.pid_turn_constants_set(3, 0, 20); // Sets Turn
  EZchassis.pid_swing_constants_forward_set(5, 0, 30); // Sets forward Swing
  EZchassis.pid_swing_constants_backward_set(5, 0, 30); // Sets backward Swing
  
  //LemLib PID Constants
  ez::PID linPID{0,0,10,0,"LEM Linear"}; //Set the LEM PID Values for Linear Movment
  ez::PID angPID{0,0,3,0,"LEM Angular"}; //Set the LEM PID Values for Angular Movment
}

// Test Example Autonomous
void testAuton(){
  updatePID(); // Sets LEM PID Values to be eqaul to EZ PID Tuner Values - ALWAYS CALL FIRST.
  EZchassis.drive_brake_set(MOTOR_BRAKE_HOLD); // Sets Drive Break to Brake Hole from EZ Template.
  EZchassis.pid_drive_toggle(false); // Disables PID Drive Toggle from EZ Template.

  LEMchassis.turnToHeading(100,10); // Robot Turns to Face 100 Degrees.
  allPIDWait();

  LEMchassis.moveToPoint(20, 15, 3000, {true, 60}, true); // Sets the New Position for the Robot to Move to - Robot Moves at 50% Speed(Max = 120) - Robot Continues to Face this Direction.
  allPIDWait();

  LEMchassis.turnToHeading(0,10); // Robot Turns to Face 0 Degrees.
  allPIDWait();

  EZchassis.pid_drive_toggle(true); // Remember to Activate EZ PID Drive Toggle when using EZ Commands.
  EZchassis.pid_turn_set(-45, 110, true); // Robot Turns 45 Degrees to the Left.
  allPIDWait();

  EZchassis.pid_drive_set(25, 110, true); // Robot Drives 25in Forward.
  allPIDWait();

  EZchassis.pid_drive_toggle(false); // Disable EZ PID Drive Toggle Once Done with EZ Drive Commands.
}

// Angular PID Example Autonomous
void ang_PID() {
  updatePID(); // Sets EZ PID Values to be Eqaul to LemLib PID Values - ALWAYS CALL FIRST.
  EZchassis.pid_drive_toggle(false); //disable PID from EZ Template 
  LEMchassis.setPose(0,0,0); //set position to 0,0 and heading 0 
  LEMchassis.turnToHeading(90, 100000); //set turn to 90 with high timeout
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