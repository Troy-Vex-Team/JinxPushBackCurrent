#include "main.h"
#include "EZ-Template/util.hpp"
#include "autons.hpp"
#include "pros/misc.h"
using namespace pros;
using namespace ez;

/////
// For installation, upgrading, documentations, and tutorials, check out our website!
// https://ez-robotics.github.io/EZ-Template/
/////


const int IN_SPEED = 127;
// Bump drive-related speeds to make autonomous motions faster (max is 127)
const int DRIVE_SPEED = 30;
const int TURN_SPEED = 30;
const int SWING_SPEED = 100;
const int timing = 1000;


pros::MotorGroup leftDrive({-12, -5, -11});
pros::MotorGroup rightDrive({9, 10, 15});

pros::MotorGroup intake({-1}, pros::MotorGearset::blue);
pros::MotorGroup indexer({-2}, pros::MotorGearset::green);
pros::adi::Pneumatics dpactivation('d', false);
pros::adi::Pneumatics descorer('c', false);
pros::Optical optical(20);
pros::adi::Pneumatics matchL('a', false);
pros::adi::Pneumatics dp('h', false);



// Chassis constructor

ez::Drive chassis(
    // These are your drive motors, the first motor is used for sensing!
    {-12, -5, -11},     // Left Chassis Ports (negative port will reverse it!)
    {9, 10, 15},  // Right Chassis Ports (negative port will reverse it!)

    19,      // IMU Port
    3.25,  // Wheel Diameter (Remember, 4" wheels without screw holes are actually 4.125!)
    600);   // Wheel RPM = cartridge * (motor gear / wheel gear)

  

/*
ez::Drive chassis(
    // These are your drive motors, the first motor is used for sensing!
    {-20, -19, -18},     // Left Chassis Ports (negative port will reverse it!)
    {13, 12, 11},  // Right Chassis Ports (negative port will reverse it!)

    5,      // IMU Port
    3.25,  // Wheel Diameter (Remember, 4" wheels without screw holes are actually 4.125!)
    450); 
  
 
pros::MotorGroup intake({10,1}, pros::MotorGearset::blue);
pros::adi::Pneumatics lift('a', false);
pros::adi::Pneumatics descorer('h', false);
pros::Optical optical(1);
pros::adi::Pneumatics matchL('f', false);

*/

// Uncomment the trackers you're using here!
// - `8` and `9` are smart ports (making these negative will reverse the sensor)
//  - you should get positive values on the encoders going FORWARD and RIGHT
// - `2.75` is the wheel diameter
// - `4.0` is the distance from the center of the wheel to the center of the robot
// ez::tracking_wheel horiz_tracker(8, 2.75, 4.0);  // This tracking wheel is perpendicular to the drive wheels
ez::tracking_wheel vert_tracker(17, 2.00, 5.5);   // This tracking wheel is parallel to the drive wheels

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */





  

void initialize() {
  // Print our branding over your terminal :D
  ez::ez_template_print();

  pros::delay(500);  // Stop the user from doing anything while legacy ports configure

  // Look at your horizontal tracking wheel and decide if it's in front of the midline of your robot or behind it
  //  - change `back` to `front` if the tracking wheel is in front of the midline
  //  - ignore this if you aren't using a horizontal tracker
  // chassis.odom_tracker_back_set(&horiz_tracker);
  // Look at your vertical tracking wheel and decide if it's to the left or right of the center of the robot
  //  - change `left` to `right` if the tracking wheel is to the right of the centerline
  //  - ignore this if you aren't using a vertical tracker
  chassis.odom_tracker_right_set(&vert_tracker);

  // Configure your chassis controls
  chassis.opcontrol_curve_buttons_toggle(false);   // Enables modifying the controller curve with buttons on the joysticks
  chassis.opcontrol_drive_activebrake_set(0.0);   // Sets the active brake kP. We recommend ~2.  0 will disable.
  chassis.opcontrol_curve_default_set(0.0, 0.0); 

   // Defaults for curve. If using tank, only the first parameter is used. (Comment this line out if you have an SD card!)

  // Set the drive to your own constants from autons.cpp!
  default_constants();

  // These are already defaulted to these buttons, but you can change the left/right curve buttons here!
  // chassis.opcontrol_curve_buttons_left_set(pros::E_CONTROLLER_DIGITAL_LEFT, pros::E_CONTROLLER_DIGITAL_RIGHT);  // If using tank, only the left side is used.
  // chassis.opcontrol_curve_buttons_right_set(pros::E_CONTROLLER_DIGITAL_Y, pros::E_CONTROLLER_DIGITAL_A);

  /* Autonomous Selector using LLEMU
  ez::as::auton_selector.autons_add({
    {"red left corner", red_left_corner},
      {"Drive\n\nDrive forward and come back", drive_example},
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
      {"Measure Offsets\n\nThis will turn the robot a bunch of times and calculate your offsets for your tracking wheels.", measure_offsets},
  });

  */

  // Initialize chassis and auton selector
  chassis.initialize();
  ez::as::initialize();
  master.rumble(chassis.drive_imu_calibrated() ? "." : "---");
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
  // . . .
}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {
  // . . .
}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */

 void checkOptical(){ // Built for debugging purposes and realtime
  while (true){
    ez::screen_print("Brightness: " + std::to_string(optical.get_brightness()), 0);
    ez::screen_print("Hue: " + std::to_string(optical.get_hue()), 1);
    ez::screen_print("R: " + std::to_string(optical.get_rgb().red), 2);
    ez::screen_print("G: " + std::to_string(optical.get_rgb().green), 3);
    ez::screen_print("B: " + std::to_string(optical.get_rgb().blue), 4);
  }
 }


 void indexer_move(){
  indexer.move(IN_SPEED);
 }
  void indexer_stop(){
    indexer.move(0);
    indexer.brake();
  }

  void indexer_reverse(){
    indexer.move(-IN_SPEED);
  }

 void intake_move(){
  intake.move(IN_SPEED);
}

void intake_stop(){
  intake.move(0);
  intake.brake();
}

void outtake(){
  intake.move(-IN_SPEED);
}

void intake_3(){
  intake_move();
  pros::delay(timing);
  chassis.pid_drive_set(38_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  intake_stop();
}

void firstGoal(){
  chassis.pid_turn_set(-90_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(15_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  matchL.set_value(true);
  outtake();
  pros::delay(timing);
  intake_stop();
  matchL.set_value(false);
}

void second_Middle(){
  chassis.pid_drive_set(-15_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  chassis.pid_turn_set(315_deg, TURN_SPEED);
  intake_move();
  chassis.pid_drive_set(48_in, DRIVE_SPEED, true);
  pros::delay(timing);
  intake_stop();
  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_drive_set(10_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  outtake();
  pros::delay(timing);
  intake_stop();
  
}

void intake_3Right(){
  chassis.pid_drive_set(20_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  chassis.pid_turn_set(-180_deg, TURN_SPEED);
  chassis.pid_wait();
  intake_move();
  chassis.pid_drive_set(35_in, DRIVE_SPEED, true);
  chassis.pid_wait();

}

void longGoal() {
  chassis.pid_drive_set(-10_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  chassis.pid_turn_set(180_deg, TURN_SPEED);
  chassis.pid_wait();
  outtake();
  chassis.pid_drive_set(20_in, DRIVE_SPEED, true);
}

void in5(){
  chassis.pid_drive_set(5_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}

 void pushback_auton_full(){
  // Drive to first group of blocks
  intake_3();
  
  // Turn towards the first middle goal and outtake 3

  firstGoal();
  
  
  // Move to second middle goal and outtake 1
  second_Middle();

  // Take the 3 group on the right side of the field
  //intake_3Right();

  //longGoal();

}


//intake 3 blocks on left and outtake in left goal
 void left_simple()
{
  intake_move();
  // intake the blocks
  chassis.pid_drive_set(27_in, 40, false);
  chassis.pid_wait();
  chassis.pid_drive_set(-15_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  chassis.pid_turn_set(-75_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(25_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  chassis.pid_turn_set(30_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(12_in, DRIVE_SPEED, true);

  //outtake to goal
  outtake();
  pros::delay(1000);
  chassis.pid_drive_set(-12_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  chassis.pid_turn_set(-160_deg, TURN_SPEED);
}


void smoveML(){
  chassis.pid_drive_set(48_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();
  
}

void smL(){
  intake_move();
  matchL.set_value(true);
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  pros::delay(timing);
  intake_stop();
  chassis.pid_drive_set(40_in, DRIVE_SPEED, true);
  intake_move();
  pros::delay(timing);
  intake_stop();
}


void sCrossLG(){
  chassis.pid_turn_set(-90_deg, TURN_SPEED, true);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(12_in, DRIVE_SPEED, true);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(90_deg, TURN_SPEED, true);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(50_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  chassis.pid_turn_set(90_deg, TURN_SPEED, true);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(12_in, DRIVE_SPEED, true);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(-90_deg, TURN_SPEED, true);
  chassis.pid_wait_quick_chain();
}

void skills_120(){
  smoveML();

  smL();

  chassis.pid_turn_set(180_deg, TURN_SPEED, true);
  
  sCrossLG();

  smL();

  chassis.pid_turn_set(180_deg, TURN_SPEED, true);


  
}

void pid_code(){
  chassis.pid_turn_set(90_deg, DRIVE_SPEED, true);
  chassis.pid_wait();
  double lateral = chassis.odom_theta_get();
  screen_print(std::to_string(lateral));
}

void colorSort(){
  while (true){
    if (optical.get_hue() >= 100){
      //blue
      indexer_reverse();
      intake_move();
      pros::delay(50);
      indexer_stop();
      intake_stop();

    }
    else{
      //red
      
    }
    
  }
 }

void cs_test(){
  intake_move();
  colorSort();
}

/*
void skills_100(){
  chassis.pid_drive_set(42_in, DRIVE_SPEED, true);
  chassis.pid_turn_set(90_deg, TURN_SPEED, true);
  chassis.pid_drive_set(10_in, DRIVE_SPEED, true);
  intake_move();
  pros::delay(timing);
  intake_stop();
  chassis.pid_drive_set(-10_in, DRIVE_SPEED, true);
  chassis.pid_turn_set(180_deg, TURN_SPEED, true);
  chassis.pid_drive_set(38_in, DRIVE_SPEED, true);
  lift.set_value(true);
  outtake();
  pros::delay(timing);
  lift.set_value(false);
  chassis.pid_turn_set(-90_deg, TURN_SPEED, true);
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_turn_set(90_deg, TURN_SPEED, true);
  chassis.pid_drive_set(80_in, DRIVE_SPEED, true);
  chassis.pid_turn_set(90_deg, TURN_SPEED, true);
  chassis.pid_drive_set(12_in, DRIVE_SPEED, true);
  chassis.pid_turn_set(-90_deg, TURN_SPEED, true);
  chassis.pid_drive_set(10_in, DRIVE_SPEED, true);
  intake_move();
  pros::delay(timing);
  intake_stop();
  chassis.pid_drive_set(-10_in, DRIVE_SPEED, true);
  chassis.pid_turn_set(180_deg, TURN_SPEED, true);
  chassis.pid_drive_set(34_in, DRIVE_SPEED, true);
  lift.set_value(true);
  outtake();
  pros::delay(timing);
  lift.set_value(false);
  chassis.pid_turn_set(90_deg, TURN_SPEED, true);
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  intake_move();
  pros::delay(timing);
  intake_stop();
  chassis.pid_turn_set(-90_deg, TURN_SPEED, true);
  chassis.pid_drive_set(42_in, DRIVE_SPEED, true);
  intake_move();
  pros::delay(timing);
  intake_stop();
  chassis.pid_turn_set(45_deg, TURN_SPEED, true);
  chassis.pid_drive_set(30_in, DRIVE_SPEED, true);
  outtake();
  prodelay(timing);
  intake_stop();
  chassis.pid_turn_set(-60_deg, TURN_SPEED, true);
  chassis.pid_drive_set(33_in, DRIVE_SPEED, true);
  intake_move();
  delay(timing);
  intake_stop();
  chassis.pid_turn_set(160_deg, TURN_SPEED, true);
  chassis.pid_drive_set(20_in, DRIVE_SPEED, true);
  lift.set_value(true);
  outtake();
  delay(timing);
  lift.set_value(false);
  chassis.pid_turn_set(-70_deg, TURN_SPEED, true);
  chassis.pid_drive_set(35_in, DRIVE_SPEED, true);
  intake_move();
  delay(timing);
  intake_stop();
  chassis.pid_drive_set(48_in, DRIVE_SPEED, true);
  chassis.pid_turn_set(-20_deg, TURN_SPEED, true);
  chassis.pid_drive_set(10_in, DRIVE_SPEED, true);
  chassis.pid_turn_set(90_deg, TURN_SPEED, true);
  chassis.pid_drive_set(10_in, DRIVE_SPEED, true);
  intake_move();
  delay(timing);
  intake_stop();
  chassis.pid_turn_set(180_deg, TURN_SPEED, true);
  chassis.pid_drive_set(42_in, DRIVE_SPEED, true);
  lift.set_value(true);
  outtake();
  delay(timing);
  lift.set_value(false);
  chassis.pid_turn_set(-90_deg, TURN_SPEED, true);
  chassis.pid_drive_set(12_in, DRIVE_SPEED, true);
  chassis.pid_turn_set(90_deg, TURN_SPEED, true);
  chassis.pid_drive_set(75_in, DRIVE_SPEED, true);
  chassis.pid_turn_set(90_deg, TURN_SPEED, true);
  chassis.pid_drive_set(10_in, DRIVE_SPEED, true);
  chassis.pid_turn_set(-90_deg, TURN_SPEED, true);
  chassis.pid_drive_set(20_in, DRIVE_SPEED, true);
  intake_move();
  delay(timing);
  intake_stop();
  chassis.pid_drive_set(-10_in, DRIVE_SPEED, true);
  chassis.pid_turn_set(180_deg, TURN_SPEED, true);
  chassis.pid_drive_set(40_in, DRIVE_SPEED, true);
  lift.set_value(true);
  outtake();
  delay(timing);
  lift.set_value(false);
  chassis.pid_drive_set(-10_in, DRIVE_SPEED, true);
  chassis.pid_turn_set(160_deg, TURN_SPEED, true);
  chassis.pid_drive_set(33_in, DRIVE_SPEED, true);
  chassis.pid_turn_set(-30_deg, TURN_SPEED, true);
  chassis.pid_drive_set(30_in, DRIVE_SPEED, true);


}
}
*/


void autonomous() {
  chassis.pid_targets_reset();                // Resets PID targets to 0
  chassis.drive_imu_reset();                  // Reset gyro position to 0
  chassis.drive_sensor_reset();               // Reset drive sensors to 0
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);    // Set the current position, you can start at a specific position with this
  chassis.drive_brake_set(E_MOTOR_BRAKE_COAST);  // Set motors to hold.  This helps autonomous consistency

  /*
  Odometry and Pure Pursuit are not magic

  It is possible to get perfectly consistent results without tracking wheels,
  but it is also possible to have extremely inconsistent results without tracking wheels.
  When you don't use tracking wheels, you need to:
   - avoid wheel slip
   - avoid wheelies
   - avoid throwing momentum around (super harsh turns, like in the example below)
  You can do cool curved motions, but you have to give your robot the best chance
  to be consistent
  */
  
  // choose 1, comment the rest
  pushback_auton_full();
  //pid_code();
  //cs_test();


  //pushback_auton_full();
  //ez::as::auton_selector.selected_auton_call();  // Calls selected auton from autonomous selector
}

/*
void PID_tester(){
 float kP = 5.0;   // How strong robot reacts to the size of the error
 float kI = 0.0;   // Fixes small errors as the robot runs
 float kD = 0.0;   // In a way, braking when the correction is too big


 float error = 0.0;
 float integral = 0.0;
 float derivative = 0.0;
 float prevError = 0.0;


 float target = 100.0;
 Imu imu_sensor(67); // whatever the port is
 imu_sensor.reset();


 chassis.drive_sensor_reset();



 while (fabs(error)>1.0){
   //float current = (leftDrive.get_position() + rightDrive.get_position())/2.0;
   error = target - current;
   integral += error;
   derivative = error - prevError;


   if (fabs(error)>100){ // done if the inegral accumul
     integral = 0;
   }




   float output = (kP * error) + (kI * integral) + (kD * derivative);


   if (output > 127){
     output = 127;
   }
   else if (output < -127){
     output = -127;
   }





  
   printf("IMU: %.2f degTarget: %.2f | Current: %.2f | Error: %.2f | Output: %.2f\n", imu_sensor.get_rotation(),target, current, error, output);


   prevError = error;
   delay(40);
 }



 delay(500);
 //printf("Final Position: %.2f\n |IMU: %.2f\n", (leftDrive.get_position() + rightDrive.get_position())/2.0, imu_sensor.get_rotation());
}


void sFirstML(){
  chassis.pid_drive_set(48_in, DRIVE_SPEED, true);
  chassis.pid_turn_set(90_deg, TURN_SPEED);
  intake_move();
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  delay(timing);
  intake_stop();
  chassis.pid_drive_set(-4_in, DRIVE_SPEED, true);
  chassis.pid_turn_set(180_deg, TURN_SPEED);
  chassis.pid_drive_set(40_in, DRIVE_SPEED, true);
  lift.set_value(true);
  outtake();
  delay(timing);
  lift.set_value(false);
  chassis.pid_turn_set(-90_deg, TURN_SPEED, true);
  chassis.pid_drive_set(12_in, DRIVE_SPEED, true);
  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_drive_set(72_in, DRIVE_SPEED, true);
  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_drive_set(12_in, DRIVE_SPEED, true);
  chassis.pid_turn_set(-90_deg, TURN_SPEED);
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  intake_move();
  delay(timing);
  chassis.pid_drive_set(-4_in, DRIVE_SPEED, true);
  intake_stop();
  chassis.pid_turn_set(180_deg, TURN_SPEED);
  chassis.pid_drive_set(40_in, DRIVE_SPEED, true);
  lift.set_value(true);
  outtake();
  delay(timing);
  lift.set_value(false);
  chassis.pid_drive_set(-4_in, DRIVE_SPEED, true);
  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  intake_move();
  delay(timing);
  intake_stop();
  chassis.pid_drive_set(48_in, DRIVE_SPEED, true);
  intake_move();
  delay(timing);
  intake_stop();
  chassis.pid_turn_set(-135_deg, TURN_SPEED, true);
  chassis.pid_drive_set(20_in, DRIVE_SPEED, true);
  outtake();
  delay(timing);
  chassis.pid_turn_set(90_deg, TURN_SPEED, true);
  chassis.pid_drive_set(15_in, DRIVE_SPEED, true);
  chassis.pid_turn_set(-45_deg, TURN_SPEED, true);
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  intake_move();
  delay(timing);
  intake_stop();
  chassis.pid_turn_set(-90_deg, DRIVE_SPEED, true);
  chassis.pid_drive_set(48_in, DRIVE_SPEED, true);
  intake_move();
  delay(timing);
  intake_stop();
  chassis.pid_drive_set(72_in, DRIVE_SPEED, true);
  chassis.pid_turn_set(-90_deg, TURN_SPEED, true);
  chassis.pid_drive_set(72_in, DRIVE_SPEED, true);
  chassis.pid_turn_set(90_deg,TURN_SPEED, true);
  chassis.pid_drive_set(30_in, DRIVE_SPEED, true);
  intake_move();
  delay(timing);
  intake_stop();
  chassis.pid_drive_set(-4_in, DRIVE_SPEED, true);
  chassis.pid_turn_set(180_deg, TURN_SPEED);
  chassis.pid_drive_set(40_in, DRIVE_SPEED, true);
  lift.set_value(true);
  outtake();
  delay(timing);
  lift.set_value(false);
  chassis.pid_drive_set(-4_in, DRIVE_SPEED, true);
  chassis.pid_turn_set(-90_deg, TURN_SPEED);
  chassis.pid_drive_set(12_in, DRIVE_SPEED, true);
  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_drive_set(72_in, DRIVE_SPEED, true);
  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_drive_set(12_in, DRIVE_SPEED, true);
  chassis.pid_turn_set(-90_deg, TURN_SPEED);
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  intake_move();
  delay(timing);
  intake_stop();
  chassis.pid_drive_set(-4_in, DRIVE_SPEED, true);
  chassis.pid_turn_set(180_deg, TURN_SPEED);
  chassis.pid_drive_set(40_in, DRIVE_SPEED, true);
  lift.set_value(true);
  outtake();
  delay(timing);
  lift.set_value(false);
  chassis.pid_drive_set(-4_in, DRIVE_SPEED, true);
  chassis.pid_turn_set(135_deg, TURN_SPEED);
  chassis.pid_drive_set(66_in, DRIVE_SPEED, true);
  chassis.pid_turn_set(-45_deg, TURN_SPEED);
} 

void cross(){
  chassis.pid_turn_set(-90_deg, SWING_SPEED, true);
  chassis.pid_wait();
  chassis.pid_drive_set(12_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  chassis.pid_turn_set(90_deg, SWING_SPEED, true);
  chassis.pid_wait();
}

void skills(){
  sFirstML();
}
*/


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
      if (chassis.odom_enabled() && !chassis.pid_tuner_enabled()) {
        // If we're on the first blank page...
        if (ez::as::page_blank_is_on(0)) {
          // Display X, Y, and Theta
          ez::screen_print("x: " + util::to_string_with_precision(chassis.odom_x_get()) +
                               "\ny: " + util::to_string_with_precision(chassis.odom_y_get()) +
                               "\na: " + util::to_string_with_precision(chassis.odom_theta_get()),
                           1);  // Don't override the top Page line

          // Display all trackers that are being used
          screen_print_tracker(chassis.odom_tracker_left, "l", 4);
          screen_print_tracker(chassis.odom_tracker_right, "r", 5);
          screen_print_tracker(chassis.odom_tracker_back, "b", 6);
          screen_print_tracker(chassis.odom_tracker_front, "f", 7);
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

/**
 * Gives you some extras to run in your opcontrol:
 * - run your autonomous routine in opcontrol by pressing DOWN and B
 *   - to prevent this from accidentally happening at a competition, this
 *     is only enabled when you're not connected to competition control.
 * - gives you a GUI to change your PID values live by pressing X
 */
void ez_template_extras() {
  // Only run this when not connected to a competition switch
  if (!pros::competition::is_connected()) {
    // PID Tuner
    // - after you find values that you're happy with, you'll have to set them in auton.cpp

    // Enable / Disable PID Tuner
    //  When enabled:
    //  * use A and Y to increment / decrement the constants
    //  * use the arrow keys to navigate the constants
    if (master.get_digital_new_press(DIGITAL_A))
      chassis.pid_tuner_toggle();

    // Trigger the selected autonomous routine
    if (master.get_digital(DIGITAL_B) && master.get_digital(DIGITAL_DOWN)) {
      pros::motor_brake_mode_e_t preference = chassis.drive_brake_get();
      autonomous();
      chassis.drive_brake_set(preference);
    }

    // Allow PID Tuner to iterate
    chassis.pid_tuner_iterate();
  }

  // Disable PID Tuner when connected to a comp switch
  else {
    if (chassis.pid_tuner_enabled())
      chassis.pid_tuner_disable();
  }
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */




void opcontrol() {
  // This is preference to what you like to drive on

  dpactivation.set_value(false);
  dp.set_value(false);
  descorer.set_value(false);
  matchL.set_value(true);
  bool dpactivationFlag = false;
  bool descoreFlag = false;
  bool matchFlag = true;
  bool dpFlag = false;

  bool last_dpactivation_state = false; 
  bool last_dp_state = false;
  bool last_descore_state = false; 
  bool last_matchL_state = false;
  // This is preference to what you like to drive on
  chassis.drive_brake_set(E_MOTOR_BRAKE_COAST);

  while (true) {
    // Gives you some extras to make EZ-Template ezier
    ez_template_extras();

    //chassis.opcontrol_arcade_flipped(ez::SPLIT);
    //chassis.opcontrol_tank();  // Tank control
    chassis.opcontrol_arcade_standard(ez::SPLIT);   // Standard split arcade
    //chassis.opcontrol_arcade_standard(ez::SINGLE);  // Standard single arcade
    //chassis.opcontrol_arcade_standard(ez::SPLIT);    // Flipped split arcade
    // chassis.opcontrol_arcade_flipped(ez::SINGLE);   // Flipped single arcade

    /*
    const double SPEED_MULTIPLIER = 4.0;

    // Joystick inputs
    int power = master.get_analog(ANALOG_LEFT_Y);   // forward/back
    int turn  = master.get_analog(ANALOG_RIGHT_X);  // turning

    // Basic split arcade mixing
    double left  = (power + turn) * SPEED_MULTIPLIER;
    double right = (power - turn) * SPEED_MULTIPLIER;

    // Clamp to valid motor range [-127, 127]
    if (left > 127) left = 127;
    if (left < -127) left = -127;
    if (right > 127) right = 127;
    if (right < -127) right = -127;

    // Send to motors
    leftDrive.move(static_cast<int>(left));
    rightDrive.move(static_cast<int>(right));
    
    
        // === SUPER AGGRESSIVE SPLIT ARCADE FOR OPCONTROL ===

    // You can tune these 3 numbers:
    const double DRIVE_MULT = 1.8;   // how fast you hit max speed forward/back
    const double TURN_MULT  = 2.0;   // how fast you turn
    const double GLOBAL_CAP = 127.0; // absolute max motor power

    // Joystick inputs
    int rawPower = master.get_analog(ANALOG_LEFT_Y);   // forward/back
    int rawTurn  = master.get_analog(ANALOG_RIGHT_X);  // turning

    // Optional: small deadzone so it doesn't drift
    if (std::abs(rawPower) < 5) rawPower = 0;
    if (std::abs(rawTurn)  < 5) rawTurn  = 0;

    // Scale them up – this makes you reach full speed faster
    double power = DRIVE_MULT * rawPower;
    double turn  = TURN_MULT  * rawTurn;

    // Mixed split-arcade
    double left  = power + turn;
    double right = power - turn;

    // Normalize if either side exceeds the cap
    double maxMag = std::max(std::abs(left), std::abs(right));
    if (maxMag > GLOBAL_CAP) {
      left  = left  * (GLOBAL_CAP / maxMag);
      right = right * (GLOBAL_CAP / maxMag);
    }
    
    // Send to motors
    leftDrive.move(static_cast<int>(left));
    rightDrive.move(static_cast<int>(right));
    */
    // . . .
    // Put more user control code here!
    // . . .
    // Intake control with controller buttons
    if (master.get_digital(DIGITAL_R1)) {
      intake_move(); // Intake in
    } else if (master.get_digital(DIGITAL_R2)) {
      outtake(); // Intake out
    } else {
      intake_stop(); // Stop intake
    }

    if (master.get_digital(DIGITAL_A)) {
      indexer_move(); // Intake in
    } else if (master.get_digital(DIGITAL_B)) {
      indexer_reverse(); // Intake out
    } else {
      indexer_stop(); // Stop intake
    }

    bool current_dpactivation_state = master.get_digital(pros::E_CONTROLLER_DIGITAL_Y);
    
    if (current_dpactivation_state && !last_dpactivation_state) {
      dpactivationFlag = !dpactivationFlag;
      dpactivation.set_value(dpactivationFlag);
    
    }
    
    last_dpactivation_state = current_dpactivation_state;

    bool current_descore_state = master.get_digital(pros::E_CONTROLLER_DIGITAL_L2);
    
    if (current_descore_state && !last_descore_state) {
      descoreFlag = !descoreFlag;
      descorer.set_value(descoreFlag);
    
    }
    last_descore_state = current_descore_state;

    
    bool current_matchLoader_state = master.get_digital(pros::E_CONTROLLER_DIGITAL_L1);

    if (current_matchLoader_state && !last_matchL_state){
    matchFlag = !matchFlag;
    matchL.set_value(matchFlag);
    }
    last_matchL_state = current_matchLoader_state;

    bool current_dp_state = master.get_digital(pros::E_CONTROLLER_DIGITAL_X);

    if (current_dp_state && !last_dp_state){
    dpFlag = !dpFlag;
    matchL.set_value(dpFlag);
    }
    last_dp_state = current_dp_state;


    pros::delay(ez::util::DELAY_TIME);  // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
  }
}
