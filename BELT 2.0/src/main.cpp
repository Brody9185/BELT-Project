#include "main.h"
#include "lemlib/asset.hpp"
#include "pros/misc.h"
#include "pros/rtos.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/odom.hpp"

void initialize() {
    
    pros::lcd::initialize(); // initialize brain screen
    EZchassis.initialize(); // EZ calibrate sensors (MUST BE FIRST)
    LEMchassis.calibrate(false); // LEMLIB calibrate sensors (MUST BE SECOND)
    defaultConstants(); // EZ set constants
    EZchassis.opcontrol_curve_default_set(3, 3.5); // EZ set drive curve
    init();
    
    // the default rate is 50. however, if you need to change the rate, you
    // can do the following.
    // lemlib::bufferedStdout().setRate(...);
    // If you use bluetooth or a wired connection, you will want to have a rate of 10ms

    // for more information on how the formatting for the loggers
    // works, refer to the fmtlib docs

        ez::as::auton_selector.autons_add({
    Auton("Testing EZ+Lem", testAuton),
    Auton("Angular PID", ang_PID),
    Auton("Lateral PID", lat_PID),
    Auton("Left Side", left_side),
    Auton("Right Side", right_side),
    Auton("Skills", skills_auto),
    Auton("Do Nothing", do_nothing),
    Auton("Turn and move to Position", pos_example),
    Auton("Turn and move to point", point_example),
});

    //PID Tasks
    pros::Task wheel_Task(wheel_task);
    pros::Task arm_Task(arm_task);

    // thread to for brain screen and position logging
    /*pros::Task screenTask([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(6, "X: %f", LEMchassis.getPose().x); // x
            pros::lcd::print(7, "Y: %f", LEMchassis.getPose().y); // y
            pros::lcd::print(8, "Theta: %f", LEMchassis.getPose().theta); // heading
            // log position telemetry
            lemlib::telemetrySink()->info("Chassis pose: {}", LEMchassis.getPose());
            // delay to save resources
            pros::delay(50);
        }
    }); */
}


void disabled() {

}

void competition_initialize() {

}

// get a path used for pure pursuit
// this needs to be put outside a function

ASSET(example_txt); // '.' replaced with "_" to make c++ happy

void autonomous() {
ez::as::auton_selector.selected_auton_call();
}

void opcontrol() {
    // loop to continuously update motors
    while (true) {
    //PID Tuner
    if (!pros::competition::is_connected()) {
      //  When enabled:
      //  * use A and Y to increment / decrement the constants
      //  * use the arrow keys to navigate the constants
    if (master.get_digital_new_press(DIGITAL_X))
        EZchassis.pid_tuner_toggle();

      // Trigger the selected autonomous routine
    if (master.get_digital(DIGITAL_B) && master.get_digital(DIGITAL_DOWN)) {
        autonomous();
        EZchassis.drive_brake_set(pros::E_MOTOR_BRAKE_COAST);
    }

      EZchassis.pid_tuner_iterate();  // Allow PID Tuner to iterate
    }

    //Button Inputs
    if(master)

        EZchassis.drive_brake_set(pros::E_MOTOR_BRAKE_COAST);
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        
        EZchassis.opcontrol_arcade_standard(ez::SPLIT); 
        
        
        pros::delay(10);    
    }
}
