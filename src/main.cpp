#include "main.h"
#include <sys/stat.h>
#include <cmath>
#include <cstdlib>
#include <future>
#include <string>
#include "EZ-Template/PID.hpp"
#include "EZ-Template/auton.hpp"
#include "EZ-Template/util.hpp"
#include "autons.hpp"
#include "pros/apix.h"
#include "pros/misc.h"
#include "pros/motor_group.hpp"
#include "pros/motors.h"
#include "pros/rtos.hpp"
#include "subsystems.hpp"

                                  
//Initialize important variables
static int autonNum = 0;//----------------------> auton selector integer
static bool clampPiston {false};//--------------> toggle for mogo clamp
static bool doinkPiston {false};//--------------> toggle for doinker
static bool team {true};//----------------------> true = red    false = blue
static bool armMacro {false};
static double DunkPos = 0;

//Color sorting function for the intake
void colorSort(bool teamCol){
  if(teamCol == true){//------------------------> If Red, spit out blue
    optical.set_led_pwm(100);//----------> Lights up LED
    if ((optical.get_hue() >= 200) && (optical.get_hue() <= 240)) {
        if (IntakeSensor.get_angle() == 1000) {// After Intake spins near the top,
          Intake.move_velocity(0);   // pause intake to let the ring fly over
          pros::delay(330);
        }
    }
    Intake.move_velocity(600);//------> Spins intake
  }else if(teamCol == false){//-----------------> If Blue, spit out red
    optical.set_led_pwm(100);//----------> Lights up LED
    if (optical.get_hue() >= 345 && optical.get_hue() <= 20) {
        if (IntakeSensor.get_angle() == 1000) {// After Intake spins near the top,
          Intake.move_velocity(0);   // pause intake to let the ring fly over
          pros::delay(330);
        }
    }
    Intake.move_velocity(600);//------> Spins intake
  }
}

// Chassis constructor
ez::Drive chassis(
    {-10, -9, -8},     // Left Chassis Ports (negative port will reverse it!)
    {7, 6, 5},       // Right Chassis Ports (negative port will reverse it!)

    1,                                    // IMU Port
    3.25,                           // Wheel Diameter (Remember, 4" wheels without screw holes are actually 4.125!)
    450                                      // Wheel RPM
);  

ez::PID armPID{2, 0.003, 20, 15, "ArmMacro"};

void initialize() {
  autonNum = 0;
  pros::delay(500);//-----------------------> Stop the user from doing anything while legacy ports configure
  Arm.set_voltage_limit(5500);//---------------> Set 5.5 watt motor limit for the half watt arm motors
  IntakeFlex.set_voltage_limit(5500);
  ArmSensor.reset();
  Arm.tare_position();
  armPID.exit_condition_set(100, 3, 500, 7, 500, 500);

  // Configure your chassis controls
  chassis.opcontrol_curve_buttons_toggle(true);  // Enables modifying the controller curve with buttons on the joysticks
  chassis.opcontrol_drive_activebrake_set(0);        // Sets the active brake kP. 0 will disable.
  chassis.opcontrol_curve_default_set(0, 0);// Defaults for curve. If using tank, only the first parameter is used.

  // Set the drive to your own constants from autons.cpp!
  default_constants();

  // Initialize chassis and auton selector
  chassis.initialize();
  master.rumble(".");
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
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Set motors to hold. This helps autonomous consistency
  ArmSensor.reset();
  ArmSensor.set_position(0);

  //Auton selector, Runs selected auton based off auton variable
  if/*---*/(autonNum == 0){
    sixRingBlue();
  }else if (autonNum == 1) {
    sixRingRed();
  }else if (autonNum == 2) {
    BlueLeftRush();
  }else if (autonNum == 3){
    RedRightRush();
  }else if (autonNum == 4){
    BlueRightAWP();
  }else if (autonNum == 5) {
    RedLeftAWP();
  }else if (autonNum == 6) {
    prog();
  }else if (autonNum == 7) {
    test();
  }

  
  
}

void opcontrol() {
  // Resets the arm sensor for the macro
  ArmSensor.reset();
  ArmSensor.set_position(0);

  // This is preference to what you like to drive on
  pros::motor_brake_mode_e_t driver_preference_brake = MOTOR_BRAKE_COAST;
  chassis.drive_brake_set(driver_preference_brake);
  Arm.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

  // Enable/Disable for EZ Template built in PID tuner
  chassis.pid_tuner_disable();

  pros::Task controller_screen([&]() {//------> Prints important shit to the controller screen
    while(true){
      double avrMotorTemp = (FrontL.get_temperature() + MidL.get_temperature() + BackL.get_temperature() + FrontR.get_temperature() 
      + MidR.get_temperature() + BackR.get_temperature()) / 6;
      //                       // gets average motor temp for the drivetrain
      
      std::string teamStr = "";// Initializes the strings for the info
      std::string autoStr = "";

      if(autonNum == 0)        // Prints the auton name depending on autoNum
        autoStr = "6B";
      else if(autonNum == 1)
        autoStr = "6R";
      else if (autonNum == 2) 
        autoStr = "RushB"; 
      else if (autonNum == 3)
        autoStr = "RushR";
      else if (autonNum == 4)
        autoStr = "BSWP";
      else if (autonNum == 5)
        autoStr = "RSWP";
      else if (autonNum == 6)
        autoStr = "PROG";
      else if (autonNum == 7)
        autoStr = "Test";

      if(team)                 // Prints what color alliance you are
        teamStr = "R";
      else
        teamStr = "B";

      // Combines all the strings and prints it as one to the controller screen
      std::string controllerString = teamStr + " A: " + autoStr + "T: " + std::to_string(avrMotorTemp);
      master.print(0, 1, "%s", controllerString);

      // delay so the brain doesn't get fried
      pros::delay(150);
    }
  });

  // pros::Task ArmMacro([&]() {//---------------> Macro for arm
  //   while(true){
  //   // Resets the position of the arm sensor if it turns negative
  //   if (ArmSensor.get_position() < 0)                                        
  //     ArmSensor.set_position(0);
  //   if (armMacro == true) {  // button Y activates the macro
  //     int target = 1500;
  //     int timeout = 0;
  //     Arm.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);  // sets braking to hold for better consistency
  //     if (ArmSensor.get_position() <= target) {
  //       Arm.move_velocity(200);
  //     }
  //   }}});

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
    //   chassis.pid_tuner_iterate();  // Allow PID Tuner to iterate
    // }
  
//--------------------------------------------------------------Setup-------------------------------------------------------------------
  
  // Standard split arcade(left stick = forward-back | right stick = turning)
  chassis.opcontrol_arcade_standard(ez::SPLIT);   

  // Left button cycles back though the autons and right button cycles forward
  if (autonNum == 8  || autonNum == -1) {
    autonNum = 0;
  } if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
    autonNum++;
  } if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) {
    autonNum--;
  }

  // Pressing the up button will change the bots color sorting to the opposite color
  if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP) && master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) {
    team = !team;
  }

//--------------------------------------------------------------Pistons-----------------------------------------------------------------
  
  // Pressing B will acuate the mobile goal clamp (is toggle)
  if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
    if (!clampPiston) {
        clamp_piston.set_value(true);
        clampPiston = !clampPiston;
    }else {
        clamp_piston.set_value(false);
        clampPiston = !clampPiston;
    }}

  // Pressing A will acuate THE DOINKER (is toggle)
  if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT) 
  || master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
    if (!doinkPiston) {
        doinker_piston.set_value(true);
        doinkPiston = !doinkPiston;
    } else {
        doinker_piston.set_value(false);
        doinkPiston = !doinkPiston;
    }}

    // Lady Brown make sure to define DunkPos somewhere silly Needs to be zero- bot is fucked if the dunk mech starts at a weird spot 
    if(master.get_digital_new_press(DIGITAL_DOWN))
    {
      if(DunkPos == 0){ // lady brown is down and needs a revive to score that ring dub
        Arm.move_velocity(0); // probs not needed but works?
        ArmSensor.reset_position(); // its down so reset it pookie
        Arm.move_velocity(200); // flip
        DunkPos = 1;
      }
      
      else if(DunkPos == 2) //go from ring grab to being high af
      { 
        Arm.move_velocity(0); // probs not needed but works?
        Arm.move_velocity(200); // flip
        DunkPos = 3;
      }
      else if(DunkPos == 4) //return to home position
      { 
        Arm.move_velocity(0); // probs not needed but works?
        Arm.move_velocity(200); // flip
        DunkPos = 5;
      }
      else if(DunkPos == 6) //return to home position
      { 
        Arm.move_velocity(0);
        Arm.move_velocity(-200); // flip
        pros::delay(500);
        Arm.set_brake_mode(pros::E_MOTOR_BRAKE_COAST); 
        Arm.move_velocity(0);
        DunkPos = 0;
      }
    }

    int grabRingPos = 15;
    int almostScoreRingPos = 90;
    int scoreRingPos = 120;
    int returnHomePos = 10; // falls back to zero as motor braking is turned off- zero is recalibrated as well - bot bouncing may hurt this dunno it funny
    if(DunkPos == 1 && ArmSensor.get_position() >= 100 * grabRingPos){
      Arm.move_velocity(0);
      DunkPos = 2;
      Arm.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD); 
      Arm.move_velocity(0);
    }
    else if(DunkPos == 3 && ArmSensor.get_position() >= 100 * almostScoreRingPos){
      Arm.move_velocity(0);
      DunkPos = 4;
      Arm.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD); 
      Arm.move_velocity(0);
    }
    else if(DunkPos == 5 && ArmSensor.get_position() >= 100 * scoreRingPos){
      Arm.move_velocity(0);
      DunkPos = 6;
      Arm.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD); 
      Arm.move_velocity(0);
    }

//---------------------------------------------------Auton, Intake, & Arm code----------------------------------------------------------
  
  // Pressing both X and Down on the controller will run the selected auton (for testing/development)
  if (master.get_digital(pros::E_CONTROLLER_DIGITAL_X) 
  && master.get_digital(pros::E_CONTROLLER_DIGITAL_UP)) {
    autonomous();
  }

  // Pressing L1 will intake with colorsort and L2 will outake
  if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1) == true
  || master.get_digital(pros::E_CONTROLLER_DIGITAL_L2) == true) {
    Intake.move_velocity(-600);
    IntakeFlex.move_velocity(-200);
  } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1) == true
  || master.get_digital(pros::E_CONTROLLER_DIGITAL_R1) == true){
    Intake.move_velocity(600);
    IntakeFlex.move_velocity(200);
  } else {
    Intake.move_velocity(0);
    IntakeFlex.move_velocity(0);
  }
  // // Pressing Y will move the macro to the scoring
  // if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)){
  //   armMacro = !armMacro;
  // }

  pros::delay(ez::util::DELAY_TIME);  // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
  
  }
}