#include "main.h"

// The Purpose for this File is to Define Motors, Pistons, and Sensors that are used on the Robot to be used in Functions and Commands in 'helpers.cpp'. //

//Pistons
inline ez::Piston clampP('A',false);
inline ez::Piston intakeP('B',false);

//Auton Selector
inline pros::adi::DigitalIn increase('G'); //limit swith for auton selector
inline pros::adi::DigitalIn decrease('H'); //limit swith for auton selector