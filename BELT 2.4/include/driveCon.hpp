#include "EZ-Template/tracking_wheel.hpp"
#include "EZ-Template/util.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/misc.h"
#include "pros/rotation.hpp"
#include "lemlib/chassis/chassis.hpp"
#include <cmath>
#include "main.h"

//driveCon

//Inputs
#define LEFTMOTOR1 18//(Make sure this is not included in the motor groups below if not used)
#define LEFTMOTOR2 -17//(Make sure this is not included in the motor groups below if not used)
#define LEFTMOTOR3 -16//(Make sure this is not included in the motor groups below if not used)
#define LEFTMOTOR4 0//Set 0 because it is unused.(Make sure this is not included in the motor groups below if not used)
#define RIGHTMOTOR1 -14//(Make sure this is not included in the motor groups below if not used)
#define RIGHTMOTOR2 13//(Make sure this is not included in the motor groups below if not used)
#define RIGHTMOTOR3 -15//(Make sure this is not included in the motor groups below if not used)
#define RIGHTMOTOR4 0//Set 0 because it is unused.(Make sure this is not included in the motor groups below if not used)
#define DRIVETRAIN_RPM 600//Sets the drivetrain RPM.
#define IMU_PORT1 11//Sets the 1st IMU PORT.
#define IMU_PORT2 0//Sets the 2nd IMU PORT.(Set 0 if unused)
#define Hor1Tracking_PORT -3
#define Hor1Offset 3.81
#define Hor2Tracking_PORT 0
#define Hor2Offset 0
#define HorTrackingWheelDiameter lemlib::Omniwheel::NEW_2
#define Ver1Tracking_PORT 20
#define Ver1Offset 1.5
#define Ver2Tracking_PORT 0
#define Ver2Offset 0
#define VerTrackingWheelDiameter 2.07
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
//inline std::vector<int> ez2ndLeftMotors = {lemLeftMotors[2], lemLeftMotors[1], lemLeftMotors[0]}; //LEFT ez type motor array.
//inline std::vector<int> ez2ndRightMotors = {lemRightMotors[2], lemRightMotors[1], lemRightMotors[0]}; //RIGHT ez type motor array.
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
    DRIVETRAIN_RPM// Drive RPM.
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

/*inline ez::Drive EZ2ndchassis(
    ez2ndLeftMotors,// Left Chassis Ports.(negative port will reverse it!)
    ezRightMotors,// Right Chassis Ports.(negative port will reverse it!)

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
inline pros::Rotation horizontalEnc1(Hor1Tracking_PORT);
inline lemlib::TrackingWheel horizontal1(&horizontalEnc1, HorTrackingWheelDiameter, Hor1Offset);
/*// horizontal tracking wheel encoder. Rotation sensor, port 15, not reversed //This is an optional setup for a second vert tacking wheel.
inline pros::Rotation horizontalEnc2(-Hor2Tracking_PORT);
inline lemlib::TrackingWheel horizontal2(&horizontalEnc2, HorTrackingWheelDiameter, Hor2Offset); */
// vertical tracking wheel encoder. Rotation sensor, port 11, reversed (negative signs don't work due to a pros bug)
inline pros::Rotation verticalEnc1(Ver1Tracking_PORT);
inline lemlib::TrackingWheel vertical1(&verticalEnc1, VerTrackingWheelDiameter, Ver1Offset);
// vertical tracking wheel encoder. Rotation sensor, port 11, reversed //This is an optional setup for a second vert tacking wheel.
/*inline pros::Rotation verticalEnc2(-Ver2Tracking_PORT);
inline lemlib::TrackingWheel vertical2(&verticalEnc2, VerTrackingWheelDiameter, Ver2Offset); */

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
                            nullptr, // vertical tracking wheel 2, set to nullptr as we don't have a second one
                            &horizontal1, // horizontal tracking wheel
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu, // inertial sensor
                            &imu2 // 2nd inertial sensor
);

// create the chassis
inline lemlib::Chassis LEMchassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);

//Distance Sensor Update Code

inline double RawAngle; // Angle Relative to field wall with - values for left
inline double Angle; // Angle Relative to field wall with 180 + for values to the left
inline double Theta; // Angle converted to whole field angle, to allow for angle reset in odom
inline double Offset; // Distance between the 2 distance sensors
inline double RightSensorCenterDistance; // Radius of cirlce distance from robot center point
inline double LeftSensorCenterDistance; // Radius of circle distance from robot center point
inline double RightVal; // Distance Sensor value of the right sensor
inline double LeftVal; // Distance Sensor value of the left sensor
inline double Distance; // Distance of robot center from field wall on either x, or y axis
inline int ConfidenceAvg; // Average confidence values from both sensors(used to ensure robot is close enough, with a good reading to correct the numbers)
inline int ConfidenceMin = 30; // Sets the minimum confidence value you want to accept
inline pros::v5::Distance SensorLeft(3); // Left Distance Sensor
inline pros::v5::Distance SensorRight(9); // Right Distance Sensor
inline double NorthWall = 72; // y value of the North field wall
inline double EastWall = 72; // x value of the East field wall
inline double SouthWall = -72; // y value of the South field wall
inline double WestWall = -72; // x value of the West field wall

inline double SkillsOffset =  (-72 + ((((SensorLeft.get_distance() + SensorRight.get_distance()) / 2) / 25.4) + 7));

inline void PosTask(){
    RightVal = SensorRight.get_distance() / 25.4; // Coverts Sensor value to inches
    LeftVal = SensorLeft.get_distance() / 25.4; // Converts Sensor value to inches
    ConfidenceAvg = (SensorRight.get_confidence() + SensorLeft.get_confidence()) / 2; // Finds both sensors confidence values, and averages them
}

inline void UpdatePos(char NESW) {
    if(ConfidenceAvg > ConfidenceMin ) { // Checks that we are above the minimum cofidence level
    RawAngle = atan((RightVal - LeftVal) / Offset); // Finds the RawAngle based on the inverse tangent of the right sensor - the left sensor. Before / by the distance between.
    if(RawAngle < 0) { // if value is -, then add 360 to convert angle to full circle / 180+ form
        Angle = RawAngle + 360;
    } else if(RawAngle >= 0) { // if value is +, then set Angle to RawAngle
        Angle = RawAngle;
    }
    if(NESW == 'N'){ // if North field wall specifed use north numbers for x vs y, and theta calculations
    Theta = Angle; // sets the Theta to Angle based on whole field orientation
    Distance = ((fabs(RightSensorCenterDistance * sin(fabs(RawAngle))) + fabs(LeftSensorCenterDistance * sin(fabs(RawAngle)))) / 2) + ((cos(fabs(RawAngle)) * RightVal + cos(fabs(RawAngle)) * LeftVal) / 2); // uses absolute val of RawAngle in order to file the average distance from sensor to wall, and sensor to robot center
    EZchassis.odom_y_set(NorthWall - Distance);
    EZchassis.odom_theta_set(Theta);
    } else if(NESW == 'E'){ // if East field wall specifed use north numbers for x vs y, and theta calculations
    Theta = Angle + 90; // sets the Theta to 90* more then Angle to account for 90* to the right
    Distance = ((fabs(RightSensorCenterDistance * cos(fabs(RawAngle))) + fabs(LeftSensorCenterDistance * cos(fabs(RawAngle)))) / 2) + ((cos(fabs(RawAngle)) * RightVal + cos(fabs(RawAngle)) * LeftVal) / 2);
    EZchassis.odom_x_set(EastWall - Distance);
    EZchassis.odom_theta_set(Theta);
    } else if(NESW == 'S') { // if South field wall specifed use north numbers for x vs y, and theta calculations
    Theta = Angle + 180; // sets the Theta to 180* more then Angle to account for 180* to the right
    Distance = ((fabs(RightSensorCenterDistance * sin(fabs(RawAngle))) + fabs(LeftSensorCenterDistance * sin(fabs(RawAngle)))) / 2) + ((cos(fabs(RawAngle)) * RightVal + cos(fabs(RawAngle)) * LeftVal) / 2);
    EZchassis.odom_y_set(SouthWall - Distance);
    EZchassis.odom_theta_set(Theta);
    } else if(NESW == 'W') { // if West field wall specifed use north numbers for x vs y, and theta calculations
    Theta = Angle + 270; // sets the Theta to 270* more then Angle to account for 270* to the right
    Distance = ((fabs(RightSensorCenterDistance * cos(fabs(RawAngle))) + fabs(LeftSensorCenterDistance * cos(fabs(RawAngle)))) / 2) + ((cos(fabs(RawAngle)) * RightVal + cos(fabs(RawAngle)) * LeftVal) / 2);
    EZchassis.odom_x_set(WestWall - Distance);
    EZchassis.odom_theta_set(Theta);
    }
}
}

inline void Ensure_POS_Reading() {
    while(ConfidenceAvg < ConfidenceMin) {
        EZchassis.pid_odom_set(1,127);
        EZchassis.pid_wait();
        pros::delay(ez::util::DELAY_TIME);
    }
}

inline void POSWait() {
    while(ConfidenceAvg < ConfidenceMin) {
        pros::delay(ez::util::DELAY_TIME);
    }
}