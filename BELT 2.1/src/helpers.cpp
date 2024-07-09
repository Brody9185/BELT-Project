#include "main.h"
#include "EZ-Template/util.hpp"
#include "lemlib/api.hpp"
#include "EZ-Template/PID.hpp"
#include "helpers.hpp"

//Piston


//Intake
void setIntake(int intakePower){
    intakeM.move(intakePower);
}

//Wheel
void set_wheel(int input){
}

ez::PID wheelPID{1,1,1,1,"wheel"};

void wheel_wait(){
    while(wheelPID.exit_condition({wheelM},true)== ez::RUNNING){
        pros::delay(ez::util::DELAY_TIME);
    }
}
void wheel_task(){
    pros::delay(2000); 
    while (true) {
    set_wheel(wheelPID.compute(wheelM.get_actual_velocity()));
    
        pros::delay(ez::util::DELAY_TIME);
    }
}

void set_arm(int input){
}

ez::PID armPID{1,1,1,1,"arm"};

void arm_wait(){
    while(armPID.exit_condition({armM},true)== ez::RUNNING){
        pros::delay(ez::util::DELAY_TIME);
    }
}
void arm_task(){
    pros::delay(2000); 
    while (true) {
    set_arm(armPID.compute(armM.get_position()));
    
        pros::delay(ez::util::DELAY_TIME);
    }
}

//Initialize PID
void init() {
    //arm Init
    armM.tare_position();
    armPID.exit_condition_set(80,
    50,
    300,
    150,
    500,
    500);
    //wheel Init
    wheelM.tare_position();
    wheelPID.exit_condition_set(80,
    50,
    300,
    150,
    500,
    500);
}


//LEM PID Tuner Functions

//Declaring LEM PID
ez::PID linPID{0,0,10,0,"LEM Linear"};
ez::PID angPID{0,0,3,0,"LEM Angular"};


    //linear motion controller
    lemlib::ControllerSettings linearController(linPID.constants.kp, // proportional gain (kP)
                                            linPID.constants.ki, // integral gain (kI)
                                            linPID.constants.kd, // derivative gain (kD)
                                            3, // anti windup
                                            1, // small error range, in inches
                                            100, // small error range timeout, in milliseconds
                                            3, // large error range, in inches
                                            500, // large error range timeout, in milliseconds
                                            20 // maximum acceleration (slew)
    );// angular motion controller
  lemlib::ControllerSettings angularController(angPID.constants.kp, // proportional gain (kP)
                                            angPID.constants.ki, // integral gain (kI)
                                            angPID.constants.kd, // derivative gain (kD)
                                            3, // anti windup
                                            1, // small error range, in degrees
                                            100, // small error range timeout, in milliseconds
                                            3, // large error range, in degrees
                                            500, // large error range timeout, in milliseconds
                                            0 // maximum acceleration (slew)
);


//Create The Chassis using these numbers
// create the chassis
lemlib::Chassis LEMchassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);