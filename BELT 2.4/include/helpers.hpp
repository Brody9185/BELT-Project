#include "EZ-Template/util.hpp"
#include "main.h"
#include "EZ-Template/PID.hpp"
#include "pros/motors.h"
#include "pros/rotation.h"
#include "pros/rtos.hpp"
#include <ranges>

// The Purpose for this File is to Define Functions and Commands for any and every possible Subsystem which a Team may use on their Robot

//Declare EZ PID for LEM PID
inline ez::PID linPID{0,0,10,0,"LEM Linear"}; //Set the LEM PID Values for Linear Movment

inline ez::PID angPID{0,0,3,0,"LEM Angular"}; //Set the LEM PID Values for Angular Movment

//Motors
inline pros::Motor intakeFM (15, pros::v5::MotorGears::blue, pros::v5::MotorUnits::rotations);
inline pros::Motor intakeHM (16, pros::v5::MotorGears::blue, pros::v5::MotorUnits::rotations);

// Example Intake Helper Function Code
inline void setIntake(int intakePower){
    intakeFM.move(intakePower);
    intakeHM.move(-intakePower);
}

/*inline void setLadyBrown(int LBPower){
    ladyBrownM.move(LBPower);
} */

//Update LEM PID Values to the EZ Values
extern void updatePID();
//Color Correction
inline bool Color;
inline bool Active = true;
inline void colorCheck(bool Active){
if(Active){
    if(Color){
        if(colorCheckO.get_hue() > 13 && colorCheckO.get_hue() < 16){
        pros::delay(200);
        colorCorrectP.set(true);
        pros::delay(300);
        colorCorrectP.set(false);
        }
    } else if(!Color){
        if(colorCheckO.get_hue() > 215 && colorCheckO.get_hue() < 220){
        pros::delay(200);
        colorCorrectP.set(true);
        pros::delay(300);
        colorCorrectP.set(false);
        }
    }
}
}

//Lady Brown
inline void setLadyBrown(int ladyBorwnPower){
    ladyBrownM.move(ladyBorwnPower);
}
inline ez::PID ladyBrownPID(0.4, 0, 5,0,"LadyBrown");

inline void ladyBrownWait(){
    while(ladyBrownPID.exit_condition(ladyBrownM,true) == ez::RUNNING){
        pros::delay(ez::util::DELAY_TIME);
    }
}
inline void ladyBrownTask(){
    pros::delay(2000);
    while(true){
        setLadyBrown(ladyBrownPID.compute(ladyBrownM.get_position()));

        pros::delay(ez::util::DELAY_TIME);
    }
}


// Initialize PID, Used in 'main.cpp'.
inline void init() {
    //LadyBrown
    ladyBrownM.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    ladyBrownR.reset_position();
    ladyBrownM.tare_position();
    ladyBrownPID.exit_condition_set(500,500,800,800,500,500);
}
