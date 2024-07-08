#include "lemlib/chassis/trackingWheel.hpp"
#include "main.h"
#include "lemlib/asset.hpp"
#include "pros/misc.h"
#include "pros/rotation.hpp"
#include "pros/rtos.hpp"
#include "lemlib/api.hpp"
#include "lemlib/chassis/chassis.hpp"

//Motors
pros::Motor intakeM (10, pros::v5::MotorGears::green, pros::v5::MotorUnits::rotations);
pros::Motor armM (11, pros::v5::MotorGears::green, pros::v5::MotorUnits::rotations);
pros::Motor wheelM (11, pros::v5::MotorGears::green, pros::v5::MotorUnits::rotations);

//Pistons
ez::Piston Piston1('A',false);

//Auton Selector
pros::adi::DigitalIn increase('G'); //limit swith for auton selector
pros::adi::DigitalIn decrease('H'); //limit swith for auton selector