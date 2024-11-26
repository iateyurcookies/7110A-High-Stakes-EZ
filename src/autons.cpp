#include "autons.hpp"
#include <sys/_intsup.h>
#include <algorithm>
#include <string>
#include "EZ-Template/slew.hpp"
#include "EZ-Template/util.hpp"
#include "liblvgl/hal/lv_hal_indev.h"
#include "main.h"
#include "okapi/api/units/QLength.hpp"
#include "pros/rtos.hpp"
#include "subsystems.hpp"


// These are out of 127
const int DRIVE_SPEED = 110;
const int TURN_SPEED = 90;
const int SWING_SPEED = 90;

// Constants
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

// Drive Example
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

// Turn Example
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

// Combining Turn + Drive
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

// Wait Until and Changing Max Speed
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

// Swing Example
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

// Motion Chaining
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

// Auto that tests everything
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

// Interference example
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

void BlueRightAWP(){
  Intake.move_relative(180, 600);
  
  //score alliance
  Arm.move_velocity(200);
  pros::delay(1200);
  Arm.move_velocity(0);

  //move back and turn to mogo
  chassis.pid_drive_set(-12_in, 65, true);
  chassis.pid_wait();
  chassis.pid_turn_set(-80, 75);
  chassis.pid_wait();

  //move to mogo and clamp
  chassis.pid_drive_set(-30_in, 45, true);
  chassis.pid_wait_until(-28);
  clamp_piston.set_value(true);

  chassis.pid_turn_set(-180, 75);
  chassis.pid_wait();
  Intake.move_velocity(600);
  chassis.pid_drive_set(16_in, 65, true);
  chassis.pid_wait();
  chassis.pid_drive_set(-4_in, 65, false);
  chassis.pid_wait();
  pros::delay(600);

  chassis.pid_swing_set(ez::LEFT_SWING, -303, 75);
  chassis.pid_wait();
  chassis.pid_drive_set(16_in, 65, true);
  chassis.pid_wait();
}

void RedLeftAWP(){
  Intake.move_relative(180, 600);
  
  //score alliance
  Arm.move_velocity(200);
  pros::delay(1200);
  Arm.move_velocity(0);

  //move back and turn to mogo
  chassis.pid_drive_set(-12_in, 65, true);
  chassis.pid_wait();
  chassis.pid_turn_set(80, 75);
  chassis.pid_wait();

  //move to mogo and clamp
  chassis.pid_drive_set(-30_in, 45, true);
  chassis.pid_wait_until(-28);
  clamp_piston.set_value(true);

  chassis.pid_turn_set(180, 75);
  chassis.pid_wait();
  Intake.move_velocity(600);
  chassis.pid_drive_set(16_in, 65, true);
  chassis.pid_wait();
  chassis.pid_drive_set(-4_in, 65, false);
  chassis.pid_wait();
  pros::delay(600);

  chassis.pid_swing_set(ez::RIGHT_SWING, 303, 75);
  chassis.pid_wait();
  chassis.pid_drive_set(16_in, 65, true);
  chassis.pid_wait();

}

void BlueLeftRush(){
  Intake.move_relative(180, 600);
  //move back
  chassis.pid_drive_set(-34_in, 100, true);
  chassis.pid_wait();

  //turn to mogo, go back, and clamp
  chassis.pid_turn_set(30, 90);
  chassis.pid_wait();
  chassis.pid_drive_set(-12_in, 45, true);
  chassis.pid_wait_until(-10_in);
  chassis.pid_speed_max_set(35);
  clamp_piston.set_value(true);

  //turn to 2 stack and intake
  chassis.pid_swing_set(ez::LEFT_SWING, 0, SWING_SPEED);
  Intake.move_velocity(600);
  chassis.pid_drive_set(10_in, 65, true);
  chassis.pid_wait();
  pros::delay(200);

  //turn to 4 ring corner
  chassis.pid_drive_set(34_in, 85, true);
  chassis.pid_wait_until(12_in);
  clamp_piston.set_value(false);
  chassis.pid_wait();
  chassis.pid_turn_set(45, 45);
  chassis.pid_wait();
  // doinker_piston.set_value(true);
  Intake.move_velocity(0);

  // // clear corner
  // chassis.pid_drive_set(12, 65, false);
  // chassis.pid_wait();
  // chassis.pid_turn_set(180, 120);
  // chassis.pid_wait();
  // doinker_piston.set_value(false);

  // //move back, turn to alliance rings, and intake

}

void RedRightRush(){
  Intake.move_relative(180, 600);
  //move back
  chassis.pid_drive_set(-34_in, 100, true);
  chassis.pid_wait();

  //turn to mogo, go back, and clamp
  chassis.pid_turn_set(-30, 90);
  chassis.pid_wait();
  chassis.pid_drive_set(-12_in, 45, true);
  chassis.pid_wait_until(-10_in);
  chassis.pid_speed_max_set(35);
  clamp_piston.set_value(true);

  //turn to 2 stack and intake
  chassis.pid_swing_set(ez::LEFT_SWING, 0, SWING_SPEED);
  Intake.move_velocity(600);
  chassis.pid_drive_set(10_in, 65, true);
  chassis.pid_wait();
  pros::delay(200);

  //turn to 4 ring corner
  chassis.pid_drive_set(34_in, 85, true);
  chassis.pid_wait_until(12_in);
  clamp_piston.set_value(false);
  chassis.pid_wait();
  chassis.pid_turn_set(-45, 45);
  chassis.pid_wait();
  // doinker_piston.set_value(true);
  Intake.move_velocity(0);

  // // clear corner
  // chassis.pid_drive_set(12, 65, false);
  // chassis.pid_wait();
  // chassis.pid_turn_set(180, 120);
  // chassis.pid_wait();
  // doinker_piston.set_value(false);

  // //move back, turn to alliance rings, and intake
}

void sixRingBlue(){
  Intake.move_relative(180, 600);
  //move back to mogo and clamp
  chassis.pid_drive_set(-16_in, 85, false);
  chassis.pid_wait();
  chassis.pid_swing_set(ez::LEFT_SWING, -35, 80, 15);
  chassis.pid_wait();
  chassis.pid_drive_set(-4_in, 65, false);
  chassis.pid_wait_until(-2_in);
  clamp_piston.set_value(true);

  //turn to 4 stack and start up intake
  Intake.move_velocity(600);
  chassis.pid_turn_set(-140, 50);
  chassis.pid_wait();

  //move forward and disrupt
  chassis.pid_drive_set(22_in, 45, false);
  chassis.pid_wait_until(8_in);
  chassis.pid_speed_max_set(65);
  chassis.pid_wait();
  pros::delay(500);

  //swing to be parallel to the 4 stack rings and intake
  chassis.pid_swing_set(ez::LEFT_SWING, -90, 80);
  chassis.pid_wait();

  //turn to 2 stack
  chassis.pid_turn_set(-10, 65);
  chassis.pid_wait();

  //move to 2 stack and intake
  chassis.pid_drive_set(12_in, 65, false);
  chassis.pid_wait();
  chassis.pid_drive_set(-4_in, 35, true);
  chassis.pid_wait();

  //turn to ladder
  chassis.pid_turn_set(0, 65);
  chassis.pid_wait();
  chassis.pid_swing_set(ez::LEFT_SWING, 135, 90, 15);
  chassis.pid_wait_until(90);

  Intake.move_velocity(-600);
  Arm.move_velocity(200);
  chassis.pid_drive_set(35_in, 35, false);
  chassis.pid_wait();

}

void sixRingRed(){
  Intake.move_relative(180, 600);
  //move back to mogo and clamp
  chassis.pid_drive_set(-16_in, 85, false);
  chassis.pid_wait();
  chassis.pid_swing_set(ez::RIGHT_SWING, 35, 80, 15);
  chassis.pid_wait();
  chassis.pid_drive_set(-4_in, 65, false);
  chassis.pid_wait_until(-2_in);
  clamp_piston.set_value(true);

  //turn to 4 stack and start up intake
  Intake.move_velocity(600);
  chassis.pid_turn_set(140, 50);
  chassis.pid_wait();

  //move forward and disrupt
  chassis.pid_drive_set(22_in, 45, false);
  chassis.pid_wait_until(8_in);
  chassis.pid_speed_max_set(65);
  chassis.pid_wait();
  pros::delay(500);

  //swing to be parallel to the 4 stack rings and intake
  chassis.pid_swing_set(ez::RIGHT_SWING, 90, 80);
  chassis.pid_wait();

  //turn to 2 stack
  chassis.pid_turn_set(10, 65);
  chassis.pid_wait();

  //move to 2 stack and intake
  chassis.pid_drive_set(12_in, 65, false);
  chassis.pid_wait();
  chassis.pid_drive_set(-4_in, 35, true);
  chassis.pid_wait();

  //turn to ladder
  chassis.pid_turn_set(0, 65);
  chassis.pid_wait();
  chassis.pid_swing_set(ez::RIGHT_SWING, -135, 90, 15);
  chassis.pid_wait_until(-90);

  Intake.move_velocity(-600);
  Arm.move_velocity(200);
  chassis.pid_drive_set(35_in, 35, false);
  chassis.pid_wait();
}

void prog(){
  Intake.move_relative(180, 600);
  //score alliance
  Arm.move_velocity(200);
  pros::delay(1200);
  Arm.move_velocity(-200);
  
  //go to mogo and clamp
  chassis.pid_swing_set(ez::RIGHT_SWING,50, 65, 5);
  chassis.pid_wait();
  Arm.move_velocity(0);
  chassis.pid_drive_set(-12_in, 45, false);
  chassis.pid_wait();
  clamp_piston.set_value(true);
  
  //turn to ladder close ring and start up intake
  chassis.pid_turn_set(-100, 65);
  chassis.pid_wait();

  // //go to ladder close ring and intake
  Intake.move_velocity(600);
  chassis.pid_drive_set(20_in, 85, true);
  chassis.pid_wait();
  pros::delay(800);

  //turn to single ring, go to ring, and intake
  chassis.pid_turn_set(-190, 65);
  chassis.pid_wait();
  chassis.pid_drive_set(18_in, 85, true);
  chassis.pid_wait();
  pros::delay(400);

  //turn to the neutral side ring of the three rings
  chassis.pid_swing_set(ez::LEFT_SWING, 80, SWING_SPEED, 10);
  chassis.pid_wait();

  // //go in the L pattern and score all the rings
  // chassis.pid_drive_set(14_in, 45, true);
  // chassis.pid_wait();
  // pros::delay(200);
  // chassis.pid_drive_set(14_in, 45, true);
  // chassis.pid_wait();
  // pros::delay(200);
  
  // //turn mogo to the corner and score it
  // chassis.pid_turn_set(255, 45);
  // chassis.pid_wait();
  // chassis.pid_drive_set(-13_in, 70, true);
  // chassis.pid_wait();
  // clamp_piston.set_value(false);
  // Intake.move_velocity(-600);
  
  // //move forward and turn to mogo
  // chassis.pid_drive_set(17_in, 70, true);
  // chassis.pid_wait();
  // chassis.pid_turn_set(115, 50);
  // chassis.pid_wait();
  // Intake.move_velocity(0);

  // //move forward and go to second mogo
  // chassis.pid_drive_set(-64_in, 75, true);
  // chassis.pid_wait_until(-62_in);
  // clamp_piston.set_value(true);

  // //score L rings on mogo
  // chassis.pid_turn_set(115, 50);
  // chassis.pid_wait();
  // Intake.move_velocity(600);
  // chassis.pid_drive_set(12_in, 55, true);
  // chassis.pid_wait();

  //turn to corner and score

}

void test(){
  chassis.pid_drive_set(8_in, 45, false);
  chassis.pid_wait();
}