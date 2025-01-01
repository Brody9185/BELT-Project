#include "main.h"
#include "EZ-Template/sdcard.hpp"
#include "autons.hpp"
#include "lemlib/asset.hpp"
#include "pros/misc.h"
#include "pros/rtos.hpp"
#include "lemlib/chassis/chassis.hpp"

pros::Task LadyBrownTask(ladyBrownTask);
pros::Task GPSTask(gpsTask);

void initialize() {
	init();
    //EZ
    ez::as::initialize();
    EZchassis.opcontrol_drive_activebrake_set(0.1); //Sets the EZ active brake.
    EZchassis.opcontrol_curve_buttons_toggle(false); //Toggles weather you can live update joystick curve with buttons.
    EZchassis.opcontrol_curve_default_set(0, 0); // EZ set drive curve
    EZchassis.initialize(); // EZ calibrate sensors (MUST BE FIRST)
    //LEM
    LEMchassis.calibrate(false); // LEMLIB calibrate sensors (MUST BE SECOND)
    EZchassis.opcontrol_curve_default_set(3, 3.5); // EZ set drive curve
    pros::lcd::initialize(); // initialize brain screen
    default_constants(); // set constants

    
    //Custom PID Tuning
    EZchassis.pid_tuner_pids.push_back({"LB PID Constants", &ladyBrownPID.constants}); // Creat Ez PID for these
    EZchassis.pid_tuner_pids.push_back({"Lin PID Constants", &linPID.constants}); // Creat Ez PID for these
    EZchassis.pid_tuner_pids.push_back({"Ang PID Constants", &angPID.constants}); // Creat Ez PID for these

    // the default rate is 50. however, if you need to change the rate, you
    // can do the following.
    // lemlib::bufferedStdout().setRate(...);
    // If you use bluetooth or a wired connection, you will want to have a rate of 10ms

    // for more information on how the formatting for the loggers
    // works, refer to the fmtlib docs

    ez::as::auton_selector.autons_add({
        {"Red Solo WP", Red_Solo_WP},
        {"Blue Solo WP", Blue_Solo_WP},
        {"Right Side Red Tournament", right_side_red_tournament},
        {"Left Side Red Tournament", left_side_red_tournament},
        {"Skills", Skills},
        /*{"Drive\n\nDrive forward and come back", drive_example},
        {"Turn\n\nTurn 3 times.", turn_example},
        {"Drive and Turn\n\nDrive forward, turn, come back", drive_and_turn},
        {"Drive and Turn\n\nSlow down during drive", wait_until_change_speed},
        {"Swing Turn\n\nSwing in an 'S' curve", swing_example},
        {"Motion Chaining\n\nDrive forward, turn, and come back, but blend everything together :D", motion_chaining},
        {"Combine all 3 movements", combining_movements},
        {"Interference\n\nAfter driving forward, robot performs differently if interfered or not", interfered_example},
        {"Simple Odom\n\nThis is the same as the drive example, but it uses odom instead!", odom_drive_example},
        {"Pure Pursuit\n\nGo to (0, 30) and pass through (6, 10) on the way.  Come back to (0, 0)", odom_pure_pursuit_example},
        {"Pure Pursuit Wait Until\n\nGo to (24, 24) but start running an intake once the robot passes (12, 24)", odom_pure_pursuit_wait_until_example},
        {"Boomerang\n\nGo to (0, 24, 45) then come back to (0, 0, 0)", odom_boomerang_example},
        {"Boomerang Pure Pursuit\n\nGo to (0, 24, 45) on the way to (24, 24) then come back to (0, 0, 0)", odom_boomerang_injected_pure_pursuit_example},
        {"Measure Offsets\n\nThis will turn the robot a bunch of times and calculate your offsets for your tracking wheels.", odom_boomerang_injected_pure_pursuit_example},
        */
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
void ez_screen_task() {
    while (true) {
      // Only run this when not connected to a competition switch
      if (!pros::competition::is_connected()) {
        // Blank page for odom debugging
        if (EZchassis.odom_enabled() && !EZchassis.pid_tuner_enabled()) {
          // If we're on the first blank page...
          if (ez::as::page_blank_is_on(0)) {
            // Display X, Y, and Theta
            ez::screen_print("x: " + util::to_string_with_precision(EZchassis.odom_x_get()) +
                                 "\ny: " + util::to_string_with_precision(EZchassis.odom_y_get()) +
                                 "\na: " + util::to_string_with_precision(EZchassis.odom_theta_get()) +
                                 "\ngpsx: " + util::to_string_with_precision(gpsxpos) +
                                 "\ngpsy: " + util::to_string_with_precision(gpsypos),
                             1);  // Don't override the top Page line
  
            // Display all trackers that are being used
            //screen_print_tracker(EZchassis.odom_tracker_left, "l", 4);
            //screen_print_tracker(EZchassis.odom_tracker_right, "r", 5);
            //screen_print_tracker(EZchassis.odom_tracker_back, "b", 6);
            //screen_print_tracker(EZchassis.odom_tracker_front, "f", 7);
          }
        }
      }
  
      // Remove all blank pages when connected to a comp switch
      else {
        if (ez::as::page_blank_amount() > 0)
          ez::as::page_blank_remove_all();
      }
  
      pros::delay(ez::util::DELAY_TIME);
    }
  }
  pros::Task ezScreenTask(ez_screen_task);


void disabled() {

}

void competition_initialize() {

}

// get a path used for pure pursuit
// this needs to be put outside a function

ASSET(example_txt); // '.' replaced with "_" to make c++ happy

void autonomous() {
    EZchassis.pid_targets_reset();                // Resets PID targets to 0
    EZchassis.drive_imu_reset();                  // Reset gyro position to 0
    EZchassis.drive_sensor_reset();               // Reset drive sensors to 0
    EZchassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Set motors to hold.  This helps autonomous consistency
    ez::as::auton_selector.selected_auton_call();
    EZchassis.drive_sensor_reset();
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
    setIntake((master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)-master.get_digital(pros::E_CONTROLLER_DIGITAL_R2))*127);
	if(master.get_digital_new_press(DIGITAL_Y)){
		ladyBrownPID.target_set(-300);
	} else if(master.get_digital_new_press(DIGITAL_DOWN)){
		ladyBrownPID.target_set(-2100);
	} else if(master.get_digital_new_press(DIGITAL_RIGHT)){
		ladyBrownPID.target_set(1000);
	} else if(lbCheck.get_new_press()){
    ladyBrownM.set_zero_position(0);
    ladyBrownPID.target_set(0);
  }


    EZchassis.drive_brake_set(pros::E_MOTOR_BRAKE_COAST);
    int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X); 

        pros::delay(10);    
    }
}
