#include "EZ-Template/util.hpp"
#include "main.h"
#include "EZ-Template/PID.hpp"
#include "pros/distance.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/rotation.h"
#include "pros/rotation.hpp"
#include "pros/rtos.hpp"
#include <cmath>
#include <ranges>

// The Purpose for this File is to Define Functions and Commands for any and every possible Subsystem which a Team may use on their Robot

//Declare EZ PID for LEM PID
inline ez::PID linPID{0,0,10,0,"LEM Linear"}; //Set the LEM PID Values for Linear Movment

inline ez::PID angPID{0,0,3,0,"LEM Angular"}; //Set the LEM PID Values for Angular Movment

//Motors
inline pros::Motor intakeHM (16, pros::v5::MotorGears::blue, pros::v5::MotorUnits::rotations);
inline pros::Motor intakeFM (15, pros::v5::MotorGears::blue, pros::v5::MotorUnits::rotations);

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
    //gps
}
/*
//Mechanum Active Break
inline pros::Rotation HorTracking(22);
inline pros::Rotation VerTracking(23);
inline pros::Motor FL(24);
inline pros::Motor FR(24);
inline pros::Motor BL(24);
inline pros::Motor BR(24);

inline int stickThreshold = 5;
inline bool idle = true;
inline int Error = 100;
inline int actveBreakPower = 60;

//This can be used in opcontrol, or at the top of initialize depending on if you want to change these numbers during the code
inline void setMechActiveBreakConditions(int activeBreakPower,int Error, int stickThreshold){
activeBreakPower = activeBreakPower;
Error = Error;
}
inline void moveForward(){
FL.move((actveBreakPower/100) * 127);
FR.move((actveBreakPower/100) * 127);
BL.move((actveBreakPower/100) * 127);
BR.move((actveBreakPower/100) * 127);
}
inline void moveBackward(){
FL.move(-(actveBreakPower/100) * 127);
FR.move(-(actveBreakPower/100) * 127);
BL.move(-(actveBreakPower/100) * 127);
BR.move(-(actveBreakPower/100) * 127);
}
inline void moveRight(){
FL.move((actveBreakPower/100) * 127);
FR.move(-(actveBreakPower/100) * 127);
BL.move(-(actveBreakPower/100) * 127);
BR.move((actveBreakPower/100) * 127);
}
inline void moveLeft(){
FL.move(-(actveBreakPower/100) * 127);
FR.move((actveBreakPower/100) * 127);
BL.move((actveBreakPower/100) * 127);
BR.move(-(actveBreakPower/100) * 127);
}
inline void moveDownRight(){
FR.move(-(actveBreakPower/100) * 127);
BL.move(-(actveBreakPower/100) * 127);
}
inline void moveUpLeft(){
FR.move((actveBreakPower/100) * 127);
BL.move((actveBreakPower/100) * 127);
}
inline void moveUpRight(){
FL.move((actveBreakPower/100) * 127);
BR.move((actveBreakPower/100) * 127);
}
inline void moveDownLeft(){
FL.move(-(actveBreakPower/100) * 127);
BR.move(-(actveBreakPower/100) * 127);
}
inline void doNotMove(){
FL.move(0);
FR.move(0);
BL.move(0);
BR.move(0);
}

//This goes inside of a pros task in intitialize
inline void MechActiveBreakTask(int StickThreshold){
    if(controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) <= StickThreshold && controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X) <= StickThreshold && controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y) <= StickThreshold && controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X) <= StickThreshold){
    bool idle = true;
    } else if(controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) > StickThreshold && controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X) > StickThreshold && controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y) > StickThreshold && controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X) > StickThreshold){
    bool idle = false;
    }
}

//This goes in driver Control
inline void MechActiveBreak(){
    if(!idle){
    HorTracking.reset_position();
    } else if(idle){
    if(HorTracking.get_position() < -Error){
        moveRight();
    } else if(HorTracking.get_position() > Error){
        moveLeft();
    } else if(VerTracking.get_position() < -Error){
        moveForward();
    } else if(VerTracking.get_position() > Error){
        moveBackward();
    } else if(HorTracking.get_position() < -Error && VerTracking.get_position() > Error){
        moveDownRight();
    } else if(HorTracking.get_position() > Error && VerTracking.get_position() > Error){
        moveDownLeft();
    } else if(HorTracking.get_position() < -Error && VerTracking.get_position() < -Error){
        moveUpRight();
    } else if(HorTracking.get_position() > Error && VerTracking.get_position() < -Error){
        moveUpLeft();
    } else if(HorTracking.get_position() < Error && HorTracking.get_position() > -Error && VerTracking.get_position() > -Error && VerTracking.get_position() < Error){
        doNotMove();
    }
    }
}*/

inline double gpsxpos;
inline double gpsypos;
inline double gpstpos;

//GPS code
inline void gpsTask(){
    gpsxpos = (odomGPS.get_position_x() * 39.26);
    gpsypos = (odomGPS.get_position_y() * 39.26);
    gpstpos = odomGPS.get_heading();
    pros::delay(ez::util::DELAY_TIME);
}

//Timer
inline int timer_val;

inline void timerTask() {
    pros::delay(2000);
    while(true){
        timer_val = timer_val + 20;
        pros::delay(20);
    }

}

/*inline void setTimer(int Timer){
    while()
} */