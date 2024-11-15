#include "main.h"
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
static int autonNum = 0;
static bool clampPiston {false};
static bool doinkPiston {false};
static bool team {true};//----------------------> true = red    false = blue
static int armPos = 0;

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

// Chassis constructor
ez::Drive chassis(
    // These are your drive motors, the first motor is used for sensing!
    {-1, -2, -3},     // Left Chassis Ports (negative port will reverse it!)
    {4, 5, 6},       // Right Chassis Ports (negative port will reverse it!)

    19,                                   // IMU Port
    3.25,                           // Wheel Diameter (Remember, 4" wheels without screw holes are actually 4.125!)
    450                                      // Wheel RPM
);  

void initialize() {
  autonNum = 0;
  pros::delay(500);                        // Stop the user from doing anything while legacy ports configure
  Arm.set_voltage_limit(5500);//-------------------> Set 5.5 watt motor limit
  // ArmLeft.tare_position();
  // armPID.exit_condition_set(80, 50, 100, 150, 400, 100);
  ArmSensor.reset();
  // Configure your chassis controls
  chassis.opcontrol_curve_buttons_toggle(true);  // Enables modifying the controller curve with buttons on the joysticks
  chassis.opcontrol_drive_activebrake_set(0);        // Sets the active brake kP. We recommend ~2.  0 will disable.
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
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Set motors to hold.  This helps autonomous consistency
  if(autonNum == 0){
    BlueRightAWP();
  }else if (autonNum == 1) {
    RedLeftAWP();
  }else if (autonNum == 2) {
    BlueLeftRush();
  }else if(autonNum == 3){
    RedRightRush();
  }else if(autonNum == 4){
    prog();
  }
  // ez::as::auton_selector.selected_auton_call();        // Calls selected auton from autonomous selector
}

void opcontrol() {
  ArmSensor.reset();
  ArmSensor.set_position(0);
  // This is preference to what you like to drive on
  pros::motor_brake_mode_e_t driver_preference_brake = MOTOR_BRAKE_COAST;
  chassis.drive_brake_set(driver_preference_brake);
  Arm.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

  chassis.pid_tuner_disable();
  // armStates armState = armStates::down;

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
          autoStr = "BRush"; 
        else if (autonNum == 3)
          autoStr = "RRush";
        else if (autonNum == 4)
          autoStr = "Prog ";
        if(team)
          teamStr = "R";
        else
          teamStr = "B";
        std::string controllerString = teamStr + " A: " + autoStr + "T: " + std::to_string(avrMotorTemp);
        // master.print(0, 1, "%s", controllerString);
        master.print(0, 1, "%d", ArmSensor.get_position());
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
    //   chassis.pid_tuner_iterate();  // Allow PID Tuner to iterate
    // }
  
//--------------------------------------------------------------Setup-------------------------------------------------------------------
  
  // Standard split arcade(left = forward-back right = turning)
  chassis.opcontrol_arcade_standard(ez::SPLIT);   
  // Left button will cycle back though the autons and right button cycles forward
  if(autonNum == 5  || autonNum == -1){
    autonNum = 0;
  }if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)){
    autonNum++;
  }if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)){
    autonNum--;
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
    }else {
        clamp_piston.set_value(false);
        clampPiston = !clampPiston;
    }}

  //Pressing A will acuate THE DOINKER (is toggle)
  if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)){
    if (!doinkPiston) {
        doinker_piston.set_value(true);
        doinkPiston = !doinkPiston;
    }else {
        doinker_piston.set_value(false);
        doinkPiston = !doinkPiston;
    }}

//---------------------------------------------------Auton, Intake, & Arm code----------------------------------------------------------
  
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
    Arm.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
    Arm.move_velocity(200);
  }else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R2) == true){
    Arm.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
    Arm.move_velocity(-200);
  }else{
    Arm.move_velocity(0);
  }

  if(ArmSensor.get_position() <= 0)
    ArmSensor.set_position(0);
  if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)){
      int target = 17500;
      Arm.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
      if(ArmSensor.get_position() <= target){
        while(ArmSensor.get_position() <= target - 300) {
          Arm.move_velocity(100 + (11 * (ArmSensor.get_position()/10000)));
        }
      }else if(ArmSensor.get_position() >= target){
        while(ArmSensor.get_position() >= target) {
          Arm.move_velocity(-100 + (11 * (ArmSensor.get_position()/10000)));
        }
      Arm.move_velocity(0);}
      }

// // Lady Brown make sure to define armPos somewhere silly Needs to be zero- bot is fucked if the dunk mech starts at a weird spot 
//    if(master.get_digital_new_press(DIGITAL_Y))
//    {
//      if(armPos == 0){ // lady brown is down and needs a revive to score that ring dub
//        Arm.move_velocity(0); // probs not needed but works?
//        ArmSensor.reset_position(); // its down so reset it pookie
//        Arm.move(-100); // flip
//        armPos = 1;
//      }else if(armPos == 2) //go from ring grab to being high af
//       { 
//       Arm.move_velocity(0); // probs not needed but works?
//         ArmSensor.reset_position(); // its down so reset it pookie
//         Arm.move(-100); // flip
//         armPos = 3;
//       }else if(armPos == 4) //return to home position
//       { 
//       Arm.move_velocity(0); // probs not needed but works?
//         ArmSensor.reset_position(); // its down so reset it pookie
//         Arm.move(100); // flip
//         armPos = 5;
//       }
      

//     }
//     int grabRingPos = 35;
//     int scoreRingPos = 150;
//     int returnHomePos = 6; // falls back to zero as motor braking is turned off- zero is recalibrated as well - bot bouncing may hurt this dunno it funny
//     if(armPos == 1 && ArmSensor.get_position() >= 1000*grabRingPos){
//       Arm.move(0);
//       armPos = 2;
//       Arm.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD); 
//       Arm.move_velocity(0);
//     }else if(armPos == 3 && ArmSensor.get_position() >= 1000*scoreRingPos){
//       Arm.move(0);
//       armPos = 4;
//       Arm.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD); 
//       Arm.move_velocity(0);
//     }else if(armPos == 5 && ArmSensor.get_position() <= 1000*returnHomePos){
//       Arm.move(0);
//       armPos = 0;
//       Arm.set_brake_mode(pros::E_MOTOR_BRAKE_COAST); 
//       Arm.move_velocity(0); //resets pos to zero cuz it home now :)
//     }
  
  pros::delay(ez::util::DELAY_TIME);  // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
  
  }
}