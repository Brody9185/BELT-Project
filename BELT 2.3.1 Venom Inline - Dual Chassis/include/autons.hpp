#include "main.h"

//functions

// Locks the Code into a While Loop until the Robot has Settled - A universal version 'chassis.pid_wait()'
inline void allPIDWait() {
LEMchassis.waitUntilDone();
EZchassis.pid_wait();
}
extern void defaultConstants();

//Autons
void testAuton();
void ang_PID();
void lin_PID();
void left_side();
void right_side();
void skills_auto();
void do_nothing();
void pos_example();
void point_example();

//PID
extern ez::PID linPID;

extern ez::PID angPID; 