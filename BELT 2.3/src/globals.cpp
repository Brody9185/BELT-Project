#include "main.h"

// The Purpose for this File is to Define Motors, Pistons, and Sensors that are used on the Robot to be used in Functions and Commands in 'helpers.cpp'. //

//Motors
pros::Motor intakeM (10, pros::v5::MotorGears::green, pros::v5::MotorUnits::rotations);
pros::Motor armM (11, pros::v5::MotorGears::green, pros::v5::MotorUnits::rotations);
pros::Motor wheelM (11, pros::v5::MotorGears::green, pros::v5::MotorUnits::rotations);

//Pistons
ez::Piston Piston1('A',false);

//Auton Selector
pros::adi::DigitalIn increase('G'); //limit swith for auton selector
pros::adi::DigitalIn decrease('H'); //limit swith for auton selector