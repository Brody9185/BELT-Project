#include "main.h"

//Motors
extern pros::Motor intakeM;
extern pros::Motor armM;
extern pros:: Motor wheelM;

//Pistons
extern ez::Piston Piston1;

//Auton Selector
extern pros::adi::DigitalIn increase; //limit swith for auton selector
extern pros::adi::DigitalIn decrease; //limit swith for auton selector