#include "main.h"
#include "pros/motors.hpp"
#include "pros/optical.hpp"
#include "EZ-Template/piston.hpp"
#include "pros/rotation.hpp"

// The Purpose for this File is to Define Motors, Pistons, and Sensors that are used on the Robot to be used in Functions and Commands in 'helpers.cpp'. //

//Pistons
inline ez::Piston clampP('G',false);
inline ez::Piston colorCorrectP('H',false);

inline pros::Optical colorCheckO(7);

inline pros::Motor ladyBrownM(2, pros::MotorGears::blue, pros::v5::MotorUnits::counts);

inline pros::Rotation ladyBrownR(12);

//Auton Selector
inline pros::adi::DigitalIn increase('A'); //limit swith for auton selector
inline pros::adi::DigitalIn decrease('B'); //limit swith for auton selector

inline pros::adi::DigitalIn ladyBrownL('C');

inline int countController = 0;