#include "main.h"
#include "EZ-Template/util.hpp"
#include "lemlib/api.hpp"

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