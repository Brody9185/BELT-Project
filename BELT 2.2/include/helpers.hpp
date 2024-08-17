#include "main.h"
#include "EZ-Template/PID.hpp"

void setIntake(int intakePower);

void arm_wait();

void arm_task();

void wheel_task();

void wheel_wait();

void init();

extern void updatePID();

extern ez::PID armPID;

extern ez::PID wheelPID;