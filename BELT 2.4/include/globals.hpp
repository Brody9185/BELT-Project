#include "main.h"
#include "pros/adi.hpp"
#include "pros/device.hpp"
#include "pros/gps.hpp"
#include "pros/motors.hpp"
#include "pros/optical.hpp"
#include "EZ-Template/piston.hpp"
#include "pros/rotation.hpp"

// The Purpose for this File is to Define Motors, Pistons, and Sensors that are used on the Robot to be used in Functions and Commands in 'helpers.cpp'. //

inline double Resting = 0;
inline double Loading = -230;
inline double WallStake = -750;
inline double AlianceStake = -1150;

//Pistons
inline ez::Piston clampP('A',false);
inline ez::Piston RightDoink('B');
inline ez::Piston LeftDoink('C');

inline ez::Piston colorCorrectP('G',false);

inline pros::Optical colorCheckO(7);

inline pros::Optical colorCheckO2(8);

inline pros::Gps odomGPS(3,-6,5.5, 0);

inline pros::Motor ladyBrownM(-2, pros::MotorGears::blue, pros::v5::MotorUnits::counts);

inline pros::Rotation ladyBrownR(12);

inline bool WrongColor;

inline int ColorSort;

inline int lineCount;

//Auton Selector
//inline pros::adi::DigitalIn increase('A'); //limit swith for auton selector
//inline pros::adi::DigitalIn decrease('B'); //limit swith for auton selector
inline pros::adi::DigitalIn lbCheck('H');

inline int countController = 0;
