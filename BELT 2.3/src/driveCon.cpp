#include "lemlib/chassis/trackingWheel.hpp"
#include "main.h"
#include "pros/misc.h"
#include "pros/rotation.hpp"
#include "lemlib/chassis/chassis.hpp"

//driveCon

//Inputs
#define LEFTMOTOR1 20//(Make sure this is not included in the motor groups below if not used)
#define LEFTMOTOR2 9//(Make sure this is not included in the motor groups below if not used)
#define LEFTMOTOR3 0//Set 0 because it is unused.(Make sure this is not included in the motor groups below if not used)
#define LEFTMOTOR4 0//Set 0 because it is unused.(Make sure this is not included in the motor groups below if not used)
#define DRIVETRAIN_RPM 450//Sets the drivetrain RPM.
#define IMU_PORT1 18//Sets the 1st IMU PORT.
#define IMU_PORT2 19//Sets the 2nd IMU PORT.(Set 0 if unused)
#define EZ_WHEEL_DIAMETER 3.25// This is for EZ Template Constructors.
#define LEM_WHEEL_DIAMETER lemlib::Omniwheel::NEW_325//This is for LEMLIB Template Custructors.
#define TRACK_WIDTH 10.25//Sets the Track Width.
#define HORIZONTAL_DRIFT 2//Sets Horizontal Drift.

// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);// Creates the controller.

// motor groups
std::vector<int8_t> lemLeftMotors = {20, 9}; //LEFT lemlib type motor array.
std::vector<int8_t> lemRightMotors = {-11, -1}; //RIGHT lemlib type motor array.
std::vector<int> ezLeftMotors = {lemLeftMotors[0], lemLeftMotors[1]}; //LEFT ez type motor array.
std::vector<int> ezRightMotors = {lemRightMotors[0], lemRightMotors[1]}; //RIGHT ez type motor array.
pros::MotorGroup leftMotors(lemLeftMotors, pros::MotorGearset::blue); // left motor group.
pros::MotorGroup rightMotors(lemRightMotors, pros::MotorGearset::blue); // right motor group.

// Inertial Sensor on port 18 & 19
pros::Imu imu(IMU_PORT1);//Creates the 1st IMU.
pros::Imu imu2(IMU_PORT2);//Creates the 2nd IMU.



ez::Drive EZchassis(
    
    ezLeftMotors,// Left Chassis Ports.(negative port will reverse it!)
    ezRightMotors,// Right Chassis Ports.(negative port will reverse it!)

    IMU_PORT1,// IMU Port.
    EZ_WHEEL_DIAMETER,// Wheel Diameter.
    DRIVETRAIN_RPM);// Drive RPM.

lemlib::Drivetrain drivetrain(&leftMotors,// left motor group.
                              &rightMotors,// right motor group.
                              TRACK_WIDTH,// track width.
                              LEM_WHEEL_DIAMETER,// wheel diameter.
                              DRIVETRAIN_RPM,// drivetrain.
                              HORIZONTAL_DRIFT// horizontal drift.
);

//LemLib

lemlib::ExpoDriveCurve throttleCurve(0, // joystick deadband out of 127.
                                     10, // minimum output where drivetrain will move out of 127.
                                     3.0 // expo curve gain.
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steerCurve(0, // joystick deadband out of 127.
                                  10, // minimum output where drivetrain will move out of 127.
                                  4.5 // expo curve gain.
);


//LEMLIB Auton Constructors

// tracking wheels

// horizontal tracking wheel encoder. Rotation sensor, port 15, not reversed
pros::Rotation horizontalEnc(-15);
lemlib::TrackingWheel horizontal(&horizontalEnc, 2.75, -6);
/*// horizontal tracking wheel encoder. Rotation sensor, port 15, not reversed //This is an optional setup for a second vert tacking wheel.
pros::Rotation horizontalEnc(-15);
lemlib::TrackingWheel horizontal(&horizontalEnc, 2.75, -6); */
// vertical tracking wheel encoder. Rotation sensor, port 11, reversed (negative signs don't work due to a pros bug)
pros::Rotation verticalEnc(-16);
lemlib::TrackingWheel vertical(&verticalEnc, 2.75, -.125);
/*// vertical tracking wheel encoder. Rotation sensor, port 11, reversed //This is an optional setup for a second vert tacking wheel.
pros::Rotation verticalEnc(-16);
lemlib::TrackingWheel vertical(&verticalEnc, 2.75, -.125); */

  //linear motion controller
  lemlib::ControllerSettings linearController(linPID.constants.kp, // proportional gain (kP)
                                            linPID.constants.ki, // integral gain (kI)
                                            linPID.constants.kd, // derivative gain (kD)
                                            3, // anti windup
                                            1, // small error range, in inches
                                            100, // small error range timeout, in milliseconds
                                            3, // large error range, in inches
                                            500, // large error range timeout, in milliseconds
                                            20 // maximum acceleration (slew)
); 
  // angular motion controller
  lemlib::ControllerSettings angularController(angPID.constants.kp, // proportional gain (kP)
                                            angPID.constants.ki, // integral gain (kI)
                                            angPID.constants.kd, // derivative gain (kD)
                                            3, // anti windup
                                            1, // small error range, in degrees
                                            100, // small error range timeout, in milliseconds
                                            3, // large error range, in degrees
                                            500, // large error range timeout, in milliseconds
                                            0 // maximum acceleration (slew)
);

// sensors for odometry
lemlib::OdomSensors sensors(&vertical, // vertical tracking wheel
                            nullptr, // vertical tracking wheel 2, set to nullptr as we don't have a second one
                            &horizontal, // horizontal tracking wheel
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu, // inertial sensor
                            &imu2 // 2nd inertial sensor
);

// create the chassis
lemlib::Chassis LEMchassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);