#include "EZ-Template/PID.hpp"

void setIntake(int intakePower);

void pistonToggle();

void arm_wait();

void arm_task();

void wheel_task();

void wheel_wait();

void init();

extern ez::PID armPID;

extern ez::PID wheelPID;