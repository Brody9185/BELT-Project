#include "main.h"
#include "EZ-Template/util.hpp"

// The Purpose for this File is to Define Functions and Commands for any and every possible Subsystem which a Team may use on their Robot

// Example Intake Helper Function Code
void setIntake(int intakePower){
    intakeM.move(intakePower);
}

// Example Set Wheel Helper Function Code(Flywheel Intake PID)
void set_wheel(int input){
}

// Example Wheel(Flywheel Intake) PID
ez::PID wheelPID{1,1,1,1,"wheel"};

// Example Wheel(Flywheel Intake PID) Wait Helper Function Code
void wheel_wait(){
    while(wheelPID.exit_condition({wheelM},true)== ez::RUNNING){
        pros::delay(ez::util::DELAY_TIME);
    }
}

// Example Wheel(Flywheel Intake PID) Task Helper Function Code
void wheel_task(){
    pros::delay(2000); 
    while (true) {
    set_wheel(wheelPID.compute(wheelM.get_actual_velocity()));
    
        pros::delay(ez::util::DELAY_TIME);
    }
}

// Example Set Arm(Lift) Helper Function Code
void set_arm(int input){
}

// Example Arm(Lift) PID
ez::PID armPID{1,1,1,1,"arm"};

// Example Arm(Lift) Wait Helper Function Code
void arm_wait(){
    while(armPID.exit_condition({armM},true)== ez::RUNNING){
        pros::delay(ez::util::DELAY_TIME);
    }
}

// Example Arm(Lift) Task Helper Function Code
void arm_task(){
    pros::delay(2000); 
    while (true) {
    set_arm(armPID.compute(armM.get_position()));
    
        pros::delay(ez::util::DELAY_TIME);
    }
}

// Initialize PID, Used in 'main.cpp'.
void init() {
    //Arm Init
    armM.tare_position();
    armPID.exit_condition_set(80,
    50,
    300,
    150,
    500,
    500);
    //Wheel Init
    wheelM.tare_position();
    wheelPID.exit_condition_set(80,
    50,
    300,
    150,
    500,
    500);
}

//Declare EZ PID for LEM PID
ez::PID linPID{0,0,10,0,"LEM Linear"}; //Set the LEM PID Values for Linear Movment
ez::PID angPID{0,0,3,0,"LEM Angular"}; //Set the LEM PID Values for Angular Movment

//Update LEM PID Values to the EZ Values
void updatePID() {
    LEMchassis.lateralSettings.kP = linPID.constants.kp;
    LEMchassis.lateralSettings.kI = linPID.constants.ki;
    LEMchassis.lateralSettings.kD = linPID.constants.kd;
    LEMchassis.angularSettings.kP = angPID.constants.kp;
    LEMchassis.angularSettings.kP = angPID.constants.ki;
    LEMchassis.angularSettings.kP = angPID.constants.kd;
}