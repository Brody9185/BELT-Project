#include "main.h"
#include "EZ-Template/sdcard.hpp"
#include "lemlib/asset.hpp"
#include "pros/misc.h"
#include "pros/rtos.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/odom.hpp"

void initialize() {
    
    pros::lcd::initialize(); // initialize brain screen
    ez::as::initialize();
    EZchassis.initialize(); // EZ calibrate sensors (MUST BE FIRST)
    LEMchassis.calibrate(false); // LEMLIB calibrate sensors (MUST BE SECOND)
    defaultConstants(); // EZ set constants
    EZchassis.opcontrol_curve_default_set(3, 3.5); // EZ set drive curve
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
    Auton("Testing EZ+Lem", testAuton),
    Auton("Angular PID", ang_PID),
    Auton("Lateral PID", lin_PID),
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
    pros::Task screenTask([&]() {
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
    }); 
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
        pros::lcd::print(5, "X: %f", LEMchassis.getPose().x); // x
        pros::lcd::print(6, "Y: %f", LEMchassis.getPose().y); // y
        pros::lcd::print(7, "Theta: %f", LEMchassis.getPose(false).theta); // heading

      // Trigger the selected autonomous routine
    if (master.get_digital(DIGITAL_B) && master.get_digital(DIGITAL_DOWN)) {
        autonomous();
        EZchassis.drive_brake_set(pros::E_MOTOR_BRAKE_COAST);
    }

      EZchassis.pid_tuner_iterate();  // Allow PID Tuner to iterate
    }

    //Button Inputs
    if(master.get_digital(pros::E_CONTROLLER_DIGITAL_A)){

    }
    else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_B)){//right back scuff

    }
    else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_X)){

    }
    /*else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_Y)){//right fwd scuff

    } */
    else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_UP)){

    }
    else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)){//left back scuff

    }
    else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT)){

    }
    else if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)){//left fwd scuff
    
    }
    else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){

    }    
    else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){

    }
    /*else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){

    }    
    else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){

    } */

    Piston1.button_toggle(master.get_digital(DIGITAL_Y));
    setIntake((master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)-master.get_digital(pros::E_CONTROLLER_DIGITAL_L2))*127);

        EZchassis.drive_brake_set(pros::E_MOTOR_BRAKE_COAST);
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        
        EZchassis.opcontrol_arcade_standard(ez::SPLIT); 
        
        pros::delay(10);    
    }
}
