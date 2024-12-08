#include "EZ-Template/tracking_wheel.hpp"
#include "main.h"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/misc.h"
#include "pros/rotation.hpp"
#include "lemlib/chassis/chassis.hpp"

//driveCon

//Inputs
#define LEFTMOTOR1 8//(Make sure this is not included in the motor groups below if not used)
#define LEFTMOTOR2 -17//(Make sure this is not included in the motor groups below if not used)
#define LEFTMOTOR3 19//(Make sure this is not included in the motor groups below if not used)
#define LEFTMOTOR4 0//Set 0 because it is unused.(Make sure this is not included in the motor groups below if not used)
#define RIGHTMOTOR1 -20//(Make sure this is not included in the motor groups below if not used)
#define RIGHTMOTOR2 -4//(Make sure this is not included in the motor groups below if not used)
#define RIGHTMOTOR3 -6//(Make sure this is not included in the motor groups below if not used)
#define RIGHTMOTOR4 0//Set 0 because it is unused.(Make sure this is not included in the motor groups below if not used)
#define DRIVETRAIN_RPM 600//Sets the drivetrain RPM.
#define IMU_PORT1 18//Sets the 1st IMU PORT.
#define IMU_PORT2 0//Sets the 2nd IMU PORT.(Set 0 if unused)
#define Hor1Tracking_PORT 0
#define Hor1Offset 0
#define Hor2Tracking_PORT 0
#define Hor2Offset 0
#define HorTrackingWheelDiameter lemlib::Omniwheel::NEW_275_HALF
#define Ver1Tracking_PORT 20
#define Ver1Offset 0
#define Ver2Tracking_PORT -21
#define Ver2Offset 0
#define VerTrackingWheelDiameter lemlib::Omniwheel::NEW_275
#define WHEEL_DIAMETER lemlib::Omniwheel::NEW_275//This is the wheel diameter used by both EZ and LEM.
#define TRACK_WIDTH 11.75//Sets the Track Width.
#define HORIZONTAL_DRIFT 2//Sets Horizontal Drift.

// controller
inline pros::Controller controller(pros::E_CONTROLLER_MASTER);// Creates the controller.

// motor groups
inline std::vector<int8_t> lemLeftMotors = {LEFTMOTOR1, -LEFTMOTOR2,LEFTMOTOR3}; //LEFT lemlib type motor array.
inline std::vector<int8_t> lemRightMotors = {RIGHTMOTOR1, RIGHTMOTOR2,RIGHTMOTOR3}; //RIGHT lemlib type motor array.
inline std::vector<int> ezForwardLeftMotors = {lemLeftMotors[0], lemLeftMotors[1], lemLeftMotors[2]}; //LEFT ez type motor array.
inline std::vector<int> ezForwardRightMotors = {lemRightMotors[0], lemRightMotors[1], lemRightMotors[2]}; //RIGHT ez type motor array.
//inline std::vector<int> ezBackwardLeftMotors = {lemLeftMotors[2], lemLeftMotors[1], lemLeftMotors[0]}; //LEFT ez type motor array.
//inline std::vector<int> ezBackwardRightMotors = {lemRightMotors[2], lemRightMotors[1], lemRightMotors[0]}; //RIGHT ez type motor array.
inline pros::MotorGroup leftMotors(lemLeftMotors, pros::MotorGearset::blue); // left motor group.
inline pros::MotorGroup rightMotors(lemRightMotors, pros::MotorGearset::blue); // right motor group.

// Inertial Sensor
inline pros::Imu imu(IMU_PORT1);//Creates the 1st IMU.
inline pros::Imu imu2(IMU_PORT2);//Creates the 2nd IMU.

//ez::tracking_wheel Left(20,2,10,1);

inline ez::Drive EZchassis(
    
    ezForwardLeftMotors,// Left Chassis Ports.(negative port will reverse it!)
    ezForwardRightMotors,// Right Chassis Ports.(negative port will reverse it!)

    IMU_PORT1,// IMU Port.
    WHEEL_DIAMETER,// Wheel Diameter.
    600// Drive RPM.
    //Ver2Tracking_PORT,
    //Ver1Tracking_PORT
    );

inline lemlib::Drivetrain drivetrain(&leftMotors,// left motor group.
                            &rightMotors,// right motor group.
                            TRACK_WIDTH,// track width.
                            WHEEL_DIAMETER,// wheel diameter.
                            DRIVETRAIN_RPM,// drivetrain.
                            HORIZONTAL_DRIFT// horizontal drift.
);

/*inline ez::Drive EZBackwardchassis(
    
    ezBackwardLeftMotors,// Left Chassis Ports.(negative port will reverse it!)
    ezBackwardRightMotors,// Right Chassis Ports.(negative port will reverse it!)

    IMU_PORT1,// IMU Port.
    WHEEL_DIAMETER,// Wheel Diameter.
    DRIVETRAIN_RPM// Drive RPM.
    ); */

//LemLib

inline lemlib::ExpoDriveCurve throttleCurve(0, // joystick deadband out of 127.
                                     10, // minimum output where drivetrain will move out of 127.
                                     3.0 // expo curve gain.
);

// input curve for steer input during driver control
inline lemlib::ExpoDriveCurve steerCurve(0, // joystick deadband out of 127.
                                  10, // minimum output where drivetrain will move out of 127.
                                  4.5 // expo curve gain.
);


//LEMLIB Auton Constructors

// tracking wheels

// horizontal tracking wheel encoder. Rotation sensor, port 15, not reversed
inline pros::Rotation horizontalEnc1(-Hor1Tracking_PORT);
inline lemlib::TrackingWheel horizontal1(&horizontalEnc1, HorTrackingWheelDiameter, Hor1Offset);
/*// horizontal tracking wheel encoder. Rotation sensor, port 15, not reversed //This is an optional setup for a second vert tacking wheel.
inline pros::Rotation horizontalEnc2(-Hor2Tracking_PORT);
inline lemlib::TrackingWheel horizontal2(&horizontalEnc2, HorTrackingWheelDiameter, Hor2Offset); */
// vertical tracking wheel encoder. Rotation sensor, port 11, reversed (negative signs don't work due to a pros bug)
inline pros::Rotation verticalEnc1(-Ver1Tracking_PORT);
inline lemlib::TrackingWheel vertical1(&verticalEnc1, VerTrackingWheelDiameter, Ver1Offset);
// vertical tracking wheel encoder. Rotation sensor, port 11, reversed //This is an optional setup for a second vert tacking wheel.
inline pros::Rotation verticalEnc2(-Ver2Tracking_PORT);
inline lemlib::TrackingWheel vertical2(&verticalEnc2, VerTrackingWheelDiameter, Ver2Offset);

//linear motion controller
inline lemlib::ControllerSettings linearController(linPID.constants.kp, // proportional gain (kP)
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
inline lemlib::ControllerSettings angularController(angPID.constants.kp, // proportional gain (kP)
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
inline lemlib::OdomSensors sensors(&vertical1, // vertical tracking wheel
                            &vertical2, // vertical tracking wheel 2, set to nullptr as we don't have a second one
                            &horizontal1, // horizontal tracking wheel
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu, // inertial sensor
                            &imu2 // 2nd inertial sensor
);

// create the chassis
inline lemlib::Chassis LEMchassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);