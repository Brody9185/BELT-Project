#include "main.h"
#include "EZ-Template/util.hpp"

//Update LEM PID Values to the EZ Values
void updatePID() {
    LEMchassis.lateralSettings.kP = linPID.constants.kp;
    LEMchassis.lateralSettings.kI = linPID.constants.ki;
    LEMchassis.lateralSettings.kD = linPID.constants.kd;
    LEMchassis.angularSettings.kP = angPID.constants.kp;
    LEMchassis.angularSettings.kI = angPID.constants.ki;
    LEMchassis.angularSettings.kD = angPID.constants.kd;
}