#include "main.h"
#include <cstdlib>
#include "EZ-Template/auton.hpp"
#include "autons.hpp"
#include "pros/apix.h"
#include "pros/misc.h"
#include "pros/motor_group.hpp"
#include "pros/motors.h"
#include "subsystems.hpp"

// Chassis constructor
ez::Drive chassis(
    // These are your drive motors, the first motor is used for sensing!
    {-1, -2, -3},     // Left Chassis Ports (negative port will reverse it!)
    {4, 5, 6},       // Right Chassis Ports (negative port will reverse it!)

    19,                                   // IMU Port
    3.25,                           // Wheel Diameter (Remember, 4" wheels without screw holes are actually 4.125!)
    450);                                    // Wheel RPM

//Initialize important variables
static int autonNum = 0;
static bool clampPiston {false};
static bool doinkPiston {false};
static bool team {true};//----------------------> true = red    false = blue
static bool armActive{false};


void initialize() {
  autonNum = 0;
  pros::delay(500);  // Stop the user from doing anything while legacy ports configure
  Arm.set_voltage_limit(5500);//---------------------> Set 5.5 watt motor limit
  ArmSensor.reset();
  // Configure your chassis controls
  chassis.opcontrol_curve_buttons_toggle(true);  // Enables modifying the controller curve with buttons on the joysticks
  chassis.opcontrol_drive_activebrake_set(0);    // Sets the active brake kP. We recommend ~2.  0 will disable.
  chassis.opcontrol_curve_default_set(0, 0);     // Defaults for curve. If using tank, only the first parameter is used.

  // Set the drive to your own constants from autons.cpp!
  default_constants();

  // These are already defaulted to these buttons, but you can change the left/right curve buttons here!
  // chassis.opcontrol_curve_buttons_left_set(pros::E_CONTROLLER_DIGITAL_LEFT, pros::E_CONTROLLER_DIGITAL_RIGHT);// If using tank, only the left side is used.
  // chassis.opcontrol_curve_buttons_right_set(pros::E_CONTROLLER_DIGITAL_Y, pros::E_CONTROLLER_DIGITAL_A);

  // Autonomous Selector using LLEMU
  ez::as::auton_selector.autons_add({
      Auton("Example Drive\n\nDrive forward and come back.", RedLeftAWP),
      Auton("Example Turn\n\nTurn 3 times.", turn_example),
      Auton("Drive and Turn\n\nDrive forward, turn, come back. ", drive_and_turn),
      Auton("Drive and Turn\n\nSlow down during drive.", wait_until_change_speed),
      Auton("Swing Example\n\nSwing in an 'S' curve", swing_example),
      Auton("Motion Chaining\n\nDrive forward, turn, and come back, but blend everything together :D", motion_chaining),
      Auton("Combine all 3 movements", combining_movements),
      Auton("Interference\n\nAfter driving forward, robot performs differently if interfered or not.", interfered_example),
  });

  // Initialize chassis and auton selector
  chassis.initialize();
  ez::as::initialize();
  master.rumble(".");
}

//Color sorting function for the intake
void colorSort(bool teamCol){
  if(teamCol == true){//------------------------> If Red, spit out blue
    optical.set_led_pwm(100);
    if((optical.get_hue() >= 200) && (optical.get_hue() <= 240)) {
        if(IntakeSensor.get_angle() == 1000){
          Intake.move_velocity(0);
          pros::delay(330);
        }
    }
    Intake.move_velocity(600);
  }else if(teamCol == false){//-----------------> If Blue, spit out red
    optical.set_led_pwm(100); {
    if(optical.get_hue() >= 345 && optical.get_hue() <= 20) {
        if(IntakeSensor.get_angle() == 1000){
          Intake.move_velocity(0);
          pros::delay(330);
        }
    }
    Intake.move_velocity(600);
    }
  }
}

void disabled() {
  // . . .
}

void competition_initialize() {
  // . . .
}

void autonomous() {
  chassis.pid_targets_reset();                            // Resets PID targets to 0
  chassis.drive_imu_reset();                              // Reset gyro position to 0
  chassis.drive_sensor_reset();                           // Reset drive sensors to 0
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Set motors to hold.  This helps autonomous consistency
  // if(autonNum == 0){
  //   BlueRightAWP();
  // }else if (autonNum == 1) {
  //   RedLeftAWP();
  // }else if (autonNum == 2) {
  //   test();
  // }else if(autonNum == 3){
  //   BlueRight6();
  // }
  ez::as::auton_selector.selected_auton_call();           // Calls selected auton from autonomous selector
}

void opcontrol() {
  // This is preference to what you like to drive on
  pros::motor_brake_mode_e_t driver_preference_brake = MOTOR_BRAKE_COAST;

  chassis.drive_brake_set(driver_preference_brake);
  chassis.pid_tuner_disable();

  pros::Task controller_screen([&]() {//------> Prints important shit to the controller screen
      while(true){
        double avrMotorTemp = (FrontL.get_temperature() + MidL.get_temperature() + BackL.get_temperature() + FrontR.get_temperature() 
        + MidR.get_temperature() + BackR.get_temperature()) / 6;
        std::string teamStr = "";
        std::string autoStr = "";
        if(autonNum == 0)
          autoStr = "BSWP ";
        else if(autonNum == 1)
          autoStr = "RSWP ";
        else if (autonNum == 2) 
          autoStr = "Test "; 
        else if (autonNum == 3)
          autoStr = "BRush";
        if(team)
          teamStr = "R";
        else
          teamStr = "B";
        std::string controllerString = teamStr + " A: " + autoStr + "T: " + std::to_string(avrMotorTemp);
        master.print(0, 1, "%s", controllerString);
        pros::delay(100);
      }
    });

  while (true) {
    // PID Tuner
    // After you find values that you're happy with, you'll have to set them in auton.cpp
    // if (!pros::competition::is_connected()) {
    //   // Enable / Disable PID Tuner
    //   //  When enabled:
    //   //  * use A and Y to increment / decrement the constants
    //   //  * use the arrow keys to navigate the constants
    //   if (master.get_digital_new_press(DIGITAL_DOWN))
    //     chassis.pid_tuner_toggle();

    //   // Trigger the selected autonomous routine
    //   if (master.get_digital(DIGITAL_X) && master.get_digital(DIGITAL_DOWN)) {
    //     autonomous();
    //     chassis.drive_brake_set(driver_preference_brake);
    //   }

    //   chassis.pid_tuner_iterate();  // Allow PID Tuner to iterate
    // }
  
    chassis.opcontrol_arcade_standard(ez::SPLIT);   // Standard split arcade

    //--------------------------------------------------------------Setup-------------------------------------------------------------------
  // Left button will cycle back though the autons and right button cycles forward
  if(autonNum == 4  || autonNum == -1){
    autonNum = 0;
  }if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)){
    autonNum = autonNum + 1;
  }if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)){
    autonNum = autonNum - 1;
  }
  // Pressing the up button will change the bots color sorting to the opposite color
  if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
    team = !team;
  }
//--------------------------------------------------------------Pistons-----------------------------------------------------------------
  //Pressing B will acuate the mobile goal clamp (is toggle)
  if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)){
    if (!clampPiston) {
        clamp_piston.set_value(true);
        clampPiston = !clampPiston;
    }
    else {
        clamp_piston.set_value(false);
        clampPiston = !clampPiston;
    }}
  //Pressing A will acuate THE DOINKER (is toggle)
  if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)){
    if (!doinkPiston) {
        doinker_piston.set_value(true);
        doinkPiston = !doinkPiston;
    }
    else {
        doinker_piston.set_value(false);
        doinkPiston = !doinkPiston;
    }}
//----------------------------------------------------Auto, Intake, & Arm code----------------------------------------------------------
  //Pressing both X and Down on the controller will run the selected auton (for testing/development)
  if(master.get_digital(pros::E_CONTROLLER_DIGITAL_X) 
  && master.get_digital(pros::E_CONTROLLER_DIGITAL_UP)){
    autonomous();
  }
  //Pressing L1 will intake with colorsort and L2 will outake
  if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1) == true) {
    colorSort(team);
  } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2) == true) {
    Intake.move_velocity(-600);
  } else {
    Intake.move_velocity(0);
  }
  //Pressing R1 will move the Lady Brown mech up and R2 will move it down
  if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R1) == true){
    Arm.move_velocity(200);
  }else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R2) == true){
    Arm.move_velocity(-200);
  }else{
    Arm.move_velocity(0);
  }

  if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)){
    if (armActive) {
      float target = 108;
      Arm.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
      while (ArmSensor.get_angle() < target - 10) {
        Arm.move_velocity(target - (target * (ArmSensor.get_angle() / target)) + 100);
      }
      Arm.move_velocity(0);
      armActive = !armActive;
    }
    else {
      armActive = !armActive;
    }}

    pros::delay(ez::util::DELAY_TIME);  // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
  }
}