#include "main.h"
#include "EZ-Template/PID.hpp"

// The Purpose for this File is to Define Functions and Commands for any and every possible Subsystem which a Team may use on their Robot

//Declare EZ PID for LEM PID
inline ez::PID linPID{0,0,10,0,"LEM Linear"}; //Set the LEM PID Values for Linear Movment

inline ez::PID angPID{0,0,3,0,"LEM Angular"}; //Set the LEM PID Values for Angular Movment

// Example Wheel(Flywheel Intake) PID
inline ez::PID wheelPID{1,1,1,1,"wheel"};

// Example Arm(Lift) PID
inline ez::PID armPID{1,1,1,1,"arm"};

//Motors
inline pros::Motor intakeFM (15, pros::v5::MotorGears::blue, pros::v5::MotorUnits::rotations);
inline pros::Motor intakeHM (16, pros::v5::MotorGears::blue, pros::v5::MotorUnits::rotations);
inline pros::Motor armM (0);
inline pros::Motor wheelM (0, pros::v5::MotorGears::green, pros::v5::MotorUnits::rotations);

// Example Intake Helper Function Code
inline void setIntake(int intakePower){
    intakeFM.move(intakePower);
    intakeHM.move(-intakePower);
}

inline void setArm(int armPower){
    armM.move(armPower);
}

// Example Set Wheel Helper Function Code(Flywheel Intake PID)
inline void set_wheel(int input){
}

// Example Wheel(Flywheel Intake PID) Wait Helper Function Code
inline void wheel_wait(){
    while(wheelPID.exit_condition({wheelM},true)== ez::RUNNING){
        pros::delay(ez::util::DELAY_TIME);
    }
}

// Example Wheel(Flywheel Intake PID) Task Helper Function Code
inline void wheel_task(){
    pros::delay(2000); 
    while (true) {
    set_wheel(wheelPID.compute(wheelM.get_actual_velocity()));
    
        pros::delay(ez::util::DELAY_TIME);
    }
}

// Example Set Arm(Lift) Helper Function Code
inline void set_arm(int input){
}

// Example Arm(Lift) Wait Helper Function Code
inline void arm_wait(){
    while(armPID.exit_condition({armM},true)== ez::RUNNING){
        pros::delay(ez::util::DELAY_TIME);
    }
}

// Example Arm(Lift) Task Helper Function Code
inline void arm_task(){
    pros::delay(2000); 
    while (true) {
    set_arm(armPID.compute(armM.get_position()));
    
        pros::delay(ez::util::DELAY_TIME);
    }
}

// Initialize PID, Used in 'main.cpp'.
inline void init() {
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

//Update LEM PID Values to the EZ Values
extern void updatePID();
//Color Correction
inline bool Color;
inline void colorCheck(bool Active){
if(Active){
    if(Color){
        if(colorCheckO.get_hue() > 350 && colorCheckO.get_hue() < 10){
        pros::delay(200);
        colorCorrectP.set(true);
        pros::delay(300);
        colorCorrectP.set(false);
        }
    } else if(!Color){
        if(colorCheckO.get_hue() > 40 && colorCheckO.get_hue() < 180){
        pros::delay(200);
        colorCorrectP.set(true);
        pros::delay(300);
        colorCorrectP.set(false);
        }
    }
}
}