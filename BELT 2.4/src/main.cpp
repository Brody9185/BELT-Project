#include "main.h"
#include "EZ-Template/sdcard.hpp"
#include "EZ-Template/util.hpp"
#include "autons.hpp"
#include "lemlib/asset.hpp"
#include "pros/misc.h"
#include "pros/rtos.hpp"
#include "lemlib/chassis/chassis.hpp"

pros::Task LadyBrownTask(ladyBrownTask);
pros::Task GPSTask(gpsTask);
pros::Task DISTANCETask(PosTask);
pros::Task colorSortTask([]() {
  pros::delay(2000);
  colorCheckO.set_led_pwm(100);
  colorCheckO2.set_led_pwm(100);
  while (true) {
    // Red Sorting
    if( ColorSort == 1){
      if(colorCheckO.get_hue() < 40 && colorCheckO.get_hue() > 12 || colorCheckO2.get_hue() < 40 && colorCheckO2.get_hue() > 12){
        WrongColor = true;
        lineCount = 1;
      }
    }
    //Blue Sorting
    if(ColorSort == 2){
      if(colorCheckO.get_hue() < 220 && colorCheckO.get_hue() > 180 || colorCheckO2.get_hue() < 220 && colorCheckO2.get_hue() > 180){
        WrongColor = true;
        lineCount = 2;
      }
    }
    //Sort
    if(WrongColor){
      pros::delay(175);
      setIntake(127);
      pros::delay(250);
      setIntake(-127);
      WrongColor = false;
    }
    pros::delay(ez::util::DELAY_TIME);
    }
  }
);

//ez::tracking_wheel horiz_tracker(8, 2.75, 4.0);  // This tracking wheel is perpendicular to the drive wheels
ez::tracking_wheel EZvert_tracker(Ver1Tracking_PORT, VerTrackingWheelDiameter, 0); // This tracking wheel is parallel to the drive wheels
ez::tracking_wheel EZhor_tracker(Hor1Tracking_PORT,HorTrackingWheelDiameter,0); // This tracking wheel is perpendicular to the drive wheels

void initialize() {
	init();
    //EZ
    ez::as::initialize();
    EZchassis.opcontrol_drive_activebrake_set(0.1); //Sets the EZ active brake.
    EZchassis.opcontrol_curve_buttons_toggle(false); //Toggles weather you can live update joystick curve with buttons.
    EZchassis.opcontrol_curve_default_set(0, 0); // EZ set drive curve
    EZchassis.initialize(); // EZ calibrate sensors (MUST BE FIRST)
    EZchassis.odom_tracker_right_set(&EZvert_tracker);
    EZchassis.odom_tracker_front_set(&EZhor_tracker);
    //LEM
    LEMchassis.calibrate(false); // LEMLIB calibrate sensors (MUST BE SECOND)
    EZchassis.opcontrol_curve_default_set(3, 3.5); // EZ set drive curve
    pros::lcd::initialize(); // initialize brain screen
    default_constants(); // set constants
    ProfInit();

    
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
      //{"Boomerang\n\nGo to (0, 24, 45) then come back to (0, 0, 0)", Test},
        /*{"Skills", Skills},
        */{"Red Solo WP", Red_Solo_WP},
        /*{"Blue Solo WP", Blue_Solo_WP},
        {"Red Tournament", Red_Tournament},
        {"Blue Tournament", Blue_Tournament},
        {"Red 3 Ring", Red_3_Ring},
        {"Red 3 Ring", Blue_3_Ring},
        *{"Drive\n\nDrive forward and come back", drive_example},
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
        */{"Boomerang\n\nGo to (0, 24, 45) then come back to (0, 0, 0)", odom_boomerang_example},
        /*{"Boomerang Pure Pursuit\n\nGo to (0, 24, 45) on the way to (24, 24) then come back to (0, 0, 0)", odom_boomerang_injected_pure_pursuit_example},
        */
        {"Measure Offsets\n\nThis will turn the robot a bunch of times and calculate your offsets for your tracking wheels.", measure_offsets},
        
    });

    //PID Tasks

    // thread to for brain screen and position logging
    pros::Task screenTask([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(5, "X: %f", LEMchassis.getPose().x); // x
            pros::lcd::print(6, "Y: %f", LEMchassis.getPose().y); // y
            pros::lcd::print(7, "Theta: %f", LEMchassis.getPose(false).theta); // heading
            DataSet1.DisplayTask();
            // log position telemetry
            lemlib::telemetrySink()->info("Chassis pose: {}", LEMchassis.getPose());
            // delay to save resources
            pros::delay(50);
        }
    });
    master.rumble(".");
}

/**
 * Simplifies printing tracker values to the brain screen
 */
 void screen_print_tracker(ez::tracking_wheel *tracker, std::string name, int line) {
  std::string tracker_value = "", tracker_width = "";
  // Check if the tracker exists
  if (tracker != nullptr) {
    tracker_value = name + " tracker: " + util::to_string_with_precision(tracker->get());             // Make text for the tracker value
    tracker_width = "  width: " + util::to_string_with_precision(tracker->distance_to_center_get());  // Make text for the distance to center
  }
  ez::screen_print(tracker_value + tracker_width, line);  // Print final tracker text
}

/**
 * Ez screen task
 * Adding new pages here will let you view them during user control or autonomous
 * and will help you debug problems you're having
 */
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
                              "\na: " + util::to_string_with_precision(EZchassis.odom_theta_get()),
                           1);  // Don't override the top Page line
          // print robot location to the brain screen
          pros::lcd::print(5, "X: %f", LEMchassis.getPose().x); // x
          pros::lcd::print(6, "Y: %f", LEMchassis.getPose().y); // y
          pros::lcd::print(7, "Theta: %f", LEMchassis.getPose(false).theta); // heading
          // log position telemetry
          lemlib::telemetrySink()->info("Chassis pose: {}", LEMchassis.getPose());

          // Display all trackers that are being used
          screen_print_tracker(EZchassis.odom_tracker_left, "l", 4);
          screen_print_tracker(EZchassis.odom_tracker_right, "r", 5);
          screen_print_tracker(EZchassis.odom_tracker_back, "b", 6);
          screen_print_tracker(EZchassis.odom_tracker_front, "f", 7);
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
  ColorSort = 1;
    EZchassis.pid_targets_reset();                // Resets PID targets to 0
    EZchassis.drive_imu_reset();                  // Reset gyro position to 0
    EZchassis.drive_sensor_reset();               // Reset drive sensors to 0
    EZchassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Set motors to hold.  This helps autonomous consistency
    ez::as::auton_selector.selected_auton_call();
    EZchassis.drive_sensor_reset();
}

void opcontrol() {
  ColorSort = 2;
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
    //Intake
    if(!WrongColor){
      setIntake((master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)-master.get_digital(pros::E_CONTROLLER_DIGITAL_R2))*127);
  }

    EZchassis.opcontrol_arcade_standard(ez::SPLIT);
    clampP.button_toggle(master.get_digital_new_press(DIGITAL_L1));
    RightDoink.button_toggle(master.get_digital_new_press(DIGITAL_A));
    LeftDoink.button_toggle(master.get_digital_new_press(DIGITAL_LEFT));
    //setIntake((master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)-master.get_digital(pros::E_CONTROLLER_DIGITAL_R2))*127);
	if(master.get_digital_new_press(DIGITAL_Y)){
		ladyBrownPID.target_set(Loading);
	} else if(master.get_digital_new_press(DIGITAL_DOWN)){
		ladyBrownPID.target_set(WallStake);
	} else if(master.get_digital_new_press(DIGITAL_RIGHT)){
    if(ladyBrownPID.target_get() == Resting){
    ladyBrownPID.target_set(100);
    } else if(ladyBrownPID.target_get() == Loading){
      ladyBrownPID.target_set(255);
    } else if(ladyBrownPID.target_get() == -WallStake){
      ladyBrownPID.target_set(755);
    } else if(ladyBrownPID.target_get() == WallStake){
      ladyBrownPID.target_set(1220);
    }
  } else if(master.get_digital_new_press(DIGITAL_L2)){
    ladyBrownM.set_zero_position(0);
    ladyBrownPID.target_set(Resting);
  }else if (master.get_digital_new_press(DIGITAL_B)) {
    ladyBrownPID.target_set(-1200);
  }
    EZchassis.drive_brake_set(pros::E_MOTOR_BRAKE_COAST);
    int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X); 

        pros::delay(10);    
    }
}
