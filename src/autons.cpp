#include "autons.hpp"
#include <sys/_intsup.h>
#include <algorithm>
#include <string>
#include "EZ-Template/slew.hpp"
#include "main.h"
#include "pros/rtos.hpp"
#include "subsystems.hpp"

/////
// For installation, upgrading, documentations, and tutorials, check out our website!
// https://ez-robotics.github.io/EZ-Template/
/////

// These are out of 127
const int DRIVE_SPEED = 110;
const int TURN_SPEED = 90;
const int SWING_SPEED = 90;

///
// Constants
///
void default_constants() {
  chassis.pid_heading_constants_set(11, 0, 20);
  chassis.pid_drive_constants_set(20, 0, 100);
  chassis.pid_turn_constants_set(3, 0.05, 20, 15);
  chassis.pid_swing_constants_set(6, 0, 65);

  chassis.pid_turn_exit_condition_set(80_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_swing_exit_condition_set(80_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_drive_exit_condition_set(80_ms, 1_in, 250_ms, 3_in, 500_ms, 500_ms);

  chassis.pid_turn_chain_constant_set(3_deg);
  chassis.pid_swing_chain_constant_set(5_deg);
  chassis.pid_drive_chain_constant_set(3_in);

  chassis.slew_drive_constants_set(7_in, 80);
}

///
// Drive Example
///
void drive_example() {
  // The first parameter is target inches
  // The second parameter is max speed the robot will drive at
  // The third parameter is a boolean (true or false) for enabling/disabling a slew at the start of drive motions
  // for slew, only enable it when the drive distance is greater than the slew distance + a few inches

  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_drive_set(-12_in, DRIVE_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-12_in, DRIVE_SPEED);
  chassis.pid_wait();
}

///
// Turn Example
///
void turn_example() {
  // The first parameter is the target in degrees
  // The second parameter is max speed the robot will drive at

  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();
}

///
// Combining Turn + Drive
///
void drive_and_turn() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}

///
// Wait Until and Changing Max Speed
///
void wait_until_change_speed() {
  // pid_wait_until will wait until the robot gets to a desired position

  // When the robot gets to 6 inches slowly, the robot will travel the remaining distance at full speed
  chassis.pid_drive_set(24_in, 30, true);
  chassis.pid_wait_until(6_in);
  chassis.pid_speed_max_set(DRIVE_SPEED);  // After driving 6 inches at 30 speed, the robot will go the remaining distance at DRIVE_SPEED
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  // When the robot gets to -6 inches slowly, the robot will travel the remaining distance at full speed
  chassis.pid_drive_set(-24_in, 30, true);
  chassis.pid_wait_until(-6_in);
  chassis.pid_speed_max_set(DRIVE_SPEED);  // After driving 6 inches at 30 speed, the robot will go the remaining distance at DRIVE_SPEED
  chassis.pid_wait();
}

///
// Swing Example
///
void swing_example() {
  // The first parameter is ez::LEFT_SWING or ez::RIGHT_SWING
  // The second parameter is the target in degrees
  // The third parameter is the speed of the moving side of the drive
  // The fourth parameter is the speed of the still side of the drive, this allows for wider arcs

  chassis.pid_swing_set(ez::LEFT_SWING, 45_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::RIGHT_SWING, 0_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::RIGHT_SWING, 45_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::LEFT_SWING, 0_deg, SWING_SPEED, 45);
  chassis.pid_wait();
}

///
// Motion Chaining
///
void motion_chaining() {
  // Motion chaining is where motions all try to blend together instead of individual movements.
  // This works by exiting while the robot is still moving a little bit.
  // To use this, replace pid_wait with pid_wait_quick_chain.
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  // Your final motion should still be a normal pid_wait
  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}

///
// Auto that tests everything
///
void combining_movements() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::RIGHT_SWING, -45_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}

///
// Interference example
///
void tug(int attempts) {
  for (int i = 0; i < attempts - 1; i++) {
    // Attempt to drive backward
    printf("i - %i", i);
    chassis.pid_drive_set(-12_in, 127);
    chassis.pid_wait();

    // If failsafed...
    if (chassis.interfered) {
      chassis.drive_sensor_reset();
      chassis.pid_drive_set(-2_in, 20);
      pros::delay(1000);
    }
    // If the robot successfully drove back, return
    else {
      return;
    }
  }
}

// If there is no interference, the robot will drive forward and turn 90 degrees.
// If interfered, the robot will drive forward and then attempt to drive backward.
void interfered_example() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  if (chassis.interfered) {
    tug(3);
    return;
  }

  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();
}

// . . .
// Make your own autonomous functions here!
// . . .
void BlueRightAWP(){
  //score on alliance stake
  chassis.pid_drive_set(-17_in, 50, true);
  chassis.pid_wait();
  chassis.pid_turn_set(-90, 75);
  chassis.pid_wait();
  chassis.pid_drive_set(-1.75_in, 40, false);
  chassis.pid_wait();
  Intake.move_velocity(600);
  pros::delay(600);
  Intake.move_velocity(-600);
  pros::delay(200);
  Intake.move_velocity(0);

  //move forward, turn, and go to the mogo
  chassis.pid_drive_set(2_in, DRIVE_SPEED, false);
  chassis.pid_wait();
  chassis.pid_turn_set(130, 80);
  chassis.pid_wait();
  
  //clamp mogo 
  chassis.pid_drive_set(-36_in, 80, true);
  chassis.pid_wait_until(-35.5_in);
  clamp_piston.set_value(true);
  chassis.pid_wait_quick_chain();
  
  //turn to 2 stack
  chassis.pid_turn_set(0, 80);
  chassis.pid_wait();

  //go to 2 stack and intake
  Intake.move_velocity(600);
  chassis.pid_drive_set(18.5_in, 90, true);
  chassis.pid_wait();
  chassis.pid_drive_set(-4.5_in, 25, false);
  chassis.pid_wait();
  pros::delay(500);

  //move to 4 pile and intake 2 alliance rings  
  chassis.pid_turn_set(275, 80);
  chassis.pid_wait();
  chassis.pid_drive_set(9_in, 90, true);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-5_in, 25, false);
  chassis.pid_wait();
  pros::delay(800);
  Intake.move_velocity(0);
  chassis.pid_drive_set(-18_in, 100, true);
  chassis.pid_wait();

  //turn to ladder and touch
  Arm.move_velocity(200);
  chassis.pid_turn_set(225, 80);
  chassis.pid_wait();
  clamp_piston.set_value(false);
  pros::delay(400);
  Arm.move_velocity(0);
  chassis.pid_drive_set(28_in, 90, true);
  chassis.pid_wait();

}

void RedLeftAWP(){
  //score on alliance stake
  chassis.pid_drive_set(-17.5_in, 50, true);
  chassis.pid_wait();
  chassis.pid_turn_set(90, 75);
  chassis.pid_wait();
  chassis.pid_drive_set(-1.8_in, 40, false);
  chassis.pid_wait();
  Intake.move_velocity(600);
  pros::delay(600);
  Intake.move_velocity(-600);
  pros::delay(200);
  Intake.move_velocity(0);

  //move forward, turn, and go to the mogo
  chassis.pid_drive_set(2_in, DRIVE_SPEED, false);
  chassis.pid_wait();
  chassis.pid_turn_set(-130, 80);
  chassis.pid_wait();
  
  //clamp mogo 
  chassis.pid_drive_set(-39_in, 90, true);
  chassis.pid_wait_until(-38.5_in);
  clamp_piston.set_value(true);
  chassis.pid_wait();
  
  //turn to 2 stack
  chassis.pid_turn_set(0, 80);
  chassis.pid_wait();

  // go to 2 stack and intake
  Intake.move_velocity(600);
  chassis.pid_drive_set(17_in, 80, true);
  chassis.pid_wait();
  chassis.pid_drive_set(-4.5_in, 25, false);
  chassis.pid_wait();
  pros::delay(500);

  //move to 4 pile and intake 2 alliance rings  
  chassis.pid_turn_set(-275, 80);
  chassis.pid_wait();
  // chassis.pid_drive_set(9.5_in, 90, true);
  // chassis.pid_wait_quick_chain();
  // chassis.pid_drive_set(-5.5_in, 25, false);
  // chassis.pid_wait();
  // pros::delay(800);
  // Intake.move_velocity(0);
  // chassis.pid_drive_set(-18_in, 100, true);
  // chassis.pid_wait();

  // //turn to ladder and touch
  // Arm.move_velocity(200);
  // chassis.pid_turn_set(-225, 80);
  // chassis.pid_wait();
  // clamp_piston.set_value(false);
  // pros::delay(400);
  // Arm.move_velocity(0);
  // chassis.pid_drive_set(28_in, 90, true);
  // chassis.pid_wait();
}

void BlueLeftRush(){
  chassis.pid_drive_set(-37_in, 60, true);
  chassis.pid_wait_until(-36_in);
  clamp_piston.set_value(true);
  chassis.pid_turn_set(90, 70);
  chassis.pid_wait();
  Intake.move_velocity(600);
  chassis.pid_drive_set(18_in, 45, true);
  chassis.pid_wait();
  pros::delay(1600);
  Intake.move_velocity(0);
  chassis.pid_turn_set(-90, 70);
  chassis.pid_wait();
  clamp_piston.set_value(false);
  chassis.pid_drive_set(18_in, 45, true);
  chassis.pid_wait();
  chassis.pid_turn_set(-130, 70);
  chassis.pid_wait();
  Arm.move_velocity(200);
  pros::delay(800);
  Arm.move_velocity(0);
  //move to mogo and put doinker down

  //move backwards with the mogo

  //doinker up and turn around

  //move to mogo and clamp

  //turn to 2 stack and intake

  //turn to 4 ring corner

  //move to corner and doinker down

  //clear corner

  //move back, turn to alliance rings, and intake 

}

void RedRightRush(){
  chassis.pid_drive_set(-37_in, 60, true);
  chassis.pid_wait_until(-36_in);
  clamp_piston.set_value(true);
  chassis.pid_turn_set(-90, 70);
  chassis.pid_wait();
  Intake.move_velocity(600);
  chassis.pid_drive_set(18_in, 45, true);
  chassis.pid_wait();
  pros::delay(1600);
  Intake.move_velocity(0);
  chassis.pid_turn_set(90, 70);
  chassis.pid_wait();
  clamp_piston.set_value(false);
  chassis.pid_drive_set(18_in, 45, true);
  chassis.pid_wait();
  chassis.pid_turn_set(130, 70);
  chassis.pid_wait();
  Arm.move_velocity(200);
  pros::delay(800);
  Arm.move_velocity(0);


  //move to mogo and put doinker down

  //move backwards with the mogo

  //doinker up and turn around

  //move to mogo and clamp

  //turn to 2 stack and intake

  //turn to 4 ring corner

  //move to corner and doinker down

  //clear corner

  //move back, turn to alliance rings, and intake 
  
}

void prog(){
  //score alliance preload on alliance stake and move forward
  Intake.move_velocity(600);
  pros::delay(600);
  Intake.move_velocity(-600);
  pros::delay(200);
  Intake.move_velocity(0);
  chassis.pid_drive_set(2_in, DRIVE_SPEED, false);
  chassis.pid_wait();

  //turn towards left side mogo
  chassis.pid_turn_set(125, TURN_SPEED);
  chassis.pid_wait();

  // //go to mogo and clamp
  // chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  // chassis.pid_wait_until(-22_in);
  // clamp_piston.set_value(true);

  // //turn to ladder close ring and start up intake
  // chassis.pid_turn_set(0, TURN_SPEED);
  // chassis.pid_wait();
  // Intake.move_velocity(600);

  // //go to ladder close ring and intake
  // chassis.pid_drive_set(10_in, DRIVE_SPEED, true);
  // chassis.pid_wait();

  // //turn to single ring, go to ring, and intake
  // chassis.pid_turn_set(90, TURN_SPEED);
  // chassis.pid_wait();
  // chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  // chassis.pid_wait();

  // //turn to the neutral side ring of the three rings
  // chassis.pid_turn_set(90, TURN_SPEED);
  // chassis.pid_wait();

  // //go in the L pattern and score all the rings
  // chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  // chassis.pid_wait();
  // chassis.pid_turn_set(90, TURN_SPEED);
  // chassis.pid_wait();
  // chassis.pid_drive_set(12_in, DRIVE_SPEED, true);
  // chassis.pid_wait();
  // chassis.pid_turn_set(90, TURN_SPEED);
  // chassis.pid_wait();
  // chassis.pid_drive_set(12_in, DRIVE_SPEED, true);
  // chassis.pid_wait();
  

  // //turn mogo to the corner and score it
  // chassis.pid_turn_set(90, TURN_SPEED);
  // chassis.pid_wait();
  // chassis.pid_drive_set(-12_in, DRIVE_SPEED, true);
  // chassis.pid_wait();
  // clamp_piston.set_value(false);
  // Intake.move_velocity(0);

  // //move forward and go to second mogo
  // chassis.pid_drive_set(-12_in, DRIVE_SPEED, true);
  // chassis.pid_wait();
  // chassis.pid_turn_set(90, TURN_SPEED);
  // chassis.pid_wait();
  // chassis.pid_drive_set(-64_in, DRIVE_SPEED, true);
  // chassis.pid_wait_until(-60_in);

  // //clamp mogo
  // clamp_piston.set_value(true);

  // //turn to ladder close ring
  // chassis.pid_turn_set(0, TURN_SPEED);
  // chassis.pid_wait();
  // Intake.move_velocity(600);

  // //go to ladder close ring and intake
  // chassis.pid_drive_set(10_in, DRIVE_SPEED, true);
  // chassis.pid_wait();

  // //turn to single ring, go to ring, and intake
  // chassis.pid_turn_set(90, TURN_SPEED);
  // chassis.pid_wait();
  // chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  // chassis.pid_wait();

  // //turn to the neutral side ring of the three rings
  // chassis.pid_turn_set(90, TURN_SPEED);
  // chassis.pid_wait();

  // //go in the L pattern and score all the rings
  // chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  // chassis.pid_wait();
  // chassis.pid_turn_set(90, TURN_SPEED);
  // chassis.pid_wait();
  // chassis.pid_drive_set(12_in, DRIVE_SPEED, true);
  // chassis.pid_wait();
  // chassis.pid_turn_set(90, TURN_SPEED);
  // chassis.pid_wait();
  // chassis.pid_drive_set(12_in, DRIVE_SPEED, true);
  // chassis.pid_wait();

  // //turn mogo to the corner and score it
  // chassis.pid_turn_set(90, TURN_SPEED);
  // chassis.pid_wait();
  // chassis.pid_drive_set(-12_in, DRIVE_SPEED, true);
  // chassis.pid_wait();
  // clamp_piston.set_value(false);

  // //move forward and go to second mogo
  // chassis.pid_drive_set(-12_in, DRIVE_SPEED, true);
  // chassis.pid_wait();
}

void test(){

}