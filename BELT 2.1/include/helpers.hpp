#include "EZ-Template/PID.hpp"
#include "main.h"

//Functions

void setIntake(int intakePower);

void arm_wait();

void arm_task();

void wheel_task();

void wheel_wait();

void init();

void setPID();

//PID
extern lemlib::ControllerSettings linearController;

extern lemlib::ControllerSettings angularController;

extern ez::PID armPID;

extern ez::PID wheelPID;

extern ez::PID linPID;

extern ez::PID angPID;

extern lemlib::Chassis LEMchassis;