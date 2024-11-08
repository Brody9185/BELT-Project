#include "main.h"
#include "EZ-Template/sdcard.hpp"
#include "lemlib/asset.hpp"
#include "pros/misc.h"
#include "pros/rtos.hpp"
#include "lemlib/chassis/chassis.hpp"

void initialize() {
    //EZ
    EZchassis.initialize();
    ez::as::initialize();
    EZchassis.opcontrol_drive_activebrake_set(0.1); //Sets the EZ active brake.
    EZchassis.opcontrol_curve_buttons_toggle(false); //Toggles weather you can live update joystick curve with buttons.
    EZchassis.opcontrol_curve_default_set(3, 3.5); // EZ set drive curve
    EZchassis.initialize(); // EZ calibrate sensors (MUST BE FIRST)
    //LEM
    LEMchassis.calibrate(false); // LEMLIB calibrate sensors (MUST BE SECOND)
    EZchassis.opcontrol_curve_default_set(3, 3.5); // EZ set drive curve
    pros::lcd::initialize(); // initialize brain screen
    defaultConstants(); // set constants
    init();

    
    //Custom PID Tuning
    EZchassis.pid_tuner_pids.push_back({"Arm PID Constants", &armPID.constants});
    EZchassis.pid_tuner_pids.push_back({"Wheel PID Constants", &wheelPID.constants});
    EZchassis.pid_tuner_pids.push_back({"Lin PID Constants", &linPID.constants}); // Creat Ez PID for these
    EZchassis.pid_tuner_pids.push_back({"Ang PID Constants", &angPID.constants}); // Creat Ez PID for these

    // the default rate is 50. however, if you need to change the rate, you
    // can do the following.
    // lemlib::bufferedStdout().setRate(...);
    // If you use bluetooth or a wired connection, you will want to have a rate of 10ms

    // for more information on how the formatting for the loggers
    // works, refer to the fmtlib docs

    ez::as::auton_selector.autons_add({
    Auton("Left Side Blue", left_side_blue),
    Auton("Right Side Blue", right_side_blue),
    Auton("Left Side Red", left_side_red),
    Auton("Right Side Red", right_side_red),
    Auton("Skills", skills_auto),
    Auton("Do Nothing", do_nothing),
});

    //PID Tasks

    // thread to for brain screen and position logging
    /*pros::Task screenTask([&]() {
        while (true) {
            // print robot location to the brain screen
            //pros::lcd::print(5, "X: %f", LEMchassis.getPose().x); // x
            //pros::lcd::print(6, "Y: %f", LEMchassis.getPose().y); // y
            //pros::lcd::print(7, "Theta: %f", LEMchassis.getPose(false).theta); // heading
            // log position telemetry
            lemlib::telemetrySink()->info("Chassis pose: {}", LEMchassis.getPose());
            // delay to save resources
            pros::delay(50);
        }
    }); */
    master.rumble(".");
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
EZchassis.drive_sensor_reset();
//EZBackwardchassis.drive_sensor_reset();
}

void opcontrol() {
    // loop to continuously update motors
    while (true) {
    //PID Tuner
    if (!pros::competition::is_connected()) {
      //  When enabled:
      //  * use A and Y to increment / decrement the constants
      //  * use the arrow keys to navigate the constants
    if (master.get_digital_new_press(DIGITAL_X)){
        EZchassis.pid_tuner_toggle(); 
    }
      // Trigger the selected autonomous routine
    if (master.get_digital(DIGITAL_B) && master.get_digital(DIGITAL_DOWN)) {
        autonomous();
        EZchassis.drive_brake_set(pros::E_MOTOR_BRAKE_COAST);
    }

      EZchassis.pid_tuner_iterate();  // Allow PID Tuner to iterate
    }

    //Button Inputs
    EZchassis.opcontrol_arcade_standard(ez::SPLIT);
    clampP.button_toggle(master.get_digital_new_press(DIGITAL_L1));
    colorCorrectP.button_toggle(master.get_digital_new_press(DIGITAL_L2));
    setIntake((master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)-master.get_digital(pros::E_CONTROLLER_DIGITAL_R2))*127);

        EZchassis.drive_brake_set(pros::E_MOTOR_BRAKE_COAST);
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X); 

        pros::delay(10);    
    }
}
