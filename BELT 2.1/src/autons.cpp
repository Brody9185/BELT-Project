#include "main.h"
#include "lemlib/asset.hpp"
#include "pros/misc.h"
#include "pros/rtos.hpp"
#include "lemlib/api.hpp"
#include "lemlib/chassis/chassis.hpp"

void allPIDWait() {
LEMchassis.waitUntilDone();
EZchassis.pid_wait();
}

void defaultConstants() {
  // EZ PID Constants
  EZchassis.pid_drive_constants_forward_set(10, 0, 100); // Sets forward
  EZchassis.pid_drive_constants_forward_set(10, 0, 100); // Sets backward
  EZchassis.pid_heading_constants_set(3, 0, 20);
  EZchassis.pid_turn_constants_set(3, 0, 20);
  EZchassis.pid_swing_constants_set(5, 0, 30); // Sets forward and backward
  
  //LemLib PID Constants

}

void testAuton(){

    EZchassis.drive_brake_set(MOTOR_BRAKE_HOLD); 
    EZchassis.pid_drive_toggle(false);

    LEMchassis.turnToHeading(100,10);
    allPIDWait();

    LEMchassis.moveToPoint(20, 15, 3000, {true, 60}, true);
    allPIDWait();

    LEMchassis.turnToHeading(0,10);
    allPIDWait();

    EZchassis.pid_drive_toggle(true);
    EZchassis.pid_turn_set(-45, 110, true);
    allPIDWait();

    EZchassis.pid_drive_set(25, 110, true);
    allPIDWait();

    EZchassis.pid_drive_toggle(false);

}

void ang_PID() {
  //disable PID from EZ Template 
  EZchassis.pid_drive_toggle(false);
//set position to 0,0 and heading 0 
  LEMchassis.setPose(0,0,0);
//set turn to 90 with bonkerz timeout
  LEMchassis.turnToHeading(90, 100000);
}

void lat_PID(){
  //disable PID from EZ Template 
  EZchassis.pid_drive_toggle(false);
  //set position to 0,0 and heading 0 
  LEMchassis.setPose(0,0,0);
  //go forward 48 inches
  LEMchassis.moveToPoint(0,48,10000);

}

void left_side() {
  EZchassis.pid_drive_toggle(false);

}

void right_side(){
  EZchassis.pid_drive_toggle(false);
}

void skills_auto(){
  EZchassis.pid_drive_toggle(false);
}

void do_nothing() {
  EZchassis.pid_drive_toggle(false);
}

void pos_example(){
  EZchassis.pid_drive_toggle(false);
  LEMchassis.setPose(0,0,0);
  LEMchassis.moveToPose(10,10,90,4000);
}

void point_example(){
  EZchassis.pid_drive_toggle(false);
  LEMchassis.setPose(0,0,0);
  LEMchassis.moveToPoint(10,10, 4000);
}