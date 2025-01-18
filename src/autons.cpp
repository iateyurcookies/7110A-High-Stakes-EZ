#include "autons.hpp"
#include <sys/_intsup.h>
#include <algorithm>
#include <string>
#include "EZ-Template/slew.hpp"
#include "EZ-Template/util.hpp"
#include "liblvgl/hal/lv_hal_indev.h"
#include "main.h"
#include "okapi/api/units/QLength.hpp"
#include "pros/device.hpp"
#include "pros/motors.h"
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

void sixRingBlue(){
  // Releases intake
  IntakeFlex.move_relative(180, 600);
  
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
  IntakeFlex.move_velocity(200);
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
  IntakeFlex.move_velocity(-200);
  Arm.move_velocity(200);
  chassis.pid_drive_set(35_in, 35, false);
  chassis.pid_wait();

}

void sixRingRed(){
  // Releases intake
  IntakeFlex.move_relative(180, 600);
  
  //move back to mogo and clamp
  chassis.pid_drive_set(-16_in, 85, false);
  chassis.pid_wait();
  chassis.pid_swing_set(ez::RIGHT_SWING, 35, 80, 15);
  chassis.pid_wait();
  chassis.pid_drive_set(-4_in, 65, false);
  chassis.pid_wait_until(-2_in);
  clamp_piston.set_value(true);

  // //turn to 4 stack and start up intake
  // Intake.move_velocity(600);
  // IntakeFlex.move_velocity(200);
  // chassis.pid_turn_set(140, 50);
  // chassis.pid_wait();

  // //move forward and disrupt
  // chassis.pid_drive_set(22_in, 45, false);
  // chassis.pid_wait_until(8_in);
  // chassis.pid_speed_max_set(65);
  // chassis.pid_wait();
  // pros::delay(500);

  // //swing to be parallel to the 4 stack rings and intake
  // chassis.pid_swing_set(ez::RIGHT_SWING, 90, 80);
  // chassis.pid_wait();

  // //turn to 2 stack
  // chassis.pid_turn_set(10, 65);
  // chassis.pid_wait();

  // //move to 2 stack and intake
  // chassis.pid_drive_set(12_in, 65, false);
  // chassis.pid_wait();
  // chassis.pid_drive_set(-4_in, 35, true);
  // chassis.pid_wait();

  // //turn to ladder
  // chassis.pid_turn_set(0, 65);
  // chassis.pid_wait();
  // chassis.pid_swing_set(ez::RIGHT_SWING, -135, 90, 15);
  // chassis.pid_wait_until(-90);

  // Intake.move_velocity(-600);
  // IntakeFlex.move_velocity(-200);
  // Arm.move_velocity(200);
  // chassis.pid_drive_set(35_in, 35, false);
  // chassis.pid_wait();
}

void BlueLeftRush(){
  // Releases intake
  IntakeFlex.move_relative(180, 600);

  //move forward
  chassis.pid_drive_set(34_in, 110, true);
  chassis.pid_wait();

  //put doinker down and grab mogo
  chassis.pid_turn_set(40, 85);
  chassis.pid_wait_quick();
  chassis.pid_drive_set(7_in, 60, false);
  chassis.pid_wait_until(6.2_in);
  doinker_piston.set_value(true);
  pros::delay(100);
  chassis.pid_wait();
  
  //move back with mogo
  chassis.pid_swing_set(ez::LEFT_SWING, 0, 95, 15);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-18_in, 65, true);
  
  //put doinker up and turn around
  chassis.pid_wait_until(-16_in);
  doinker_piston.set_value(false);
  chassis.pid_wait();
  chassis.pid_turn_set(180, 65);
  chassis.pid_wait();

  //clamp mogo and score preload
  chassis.pid_swing_set(ez::LEFT_SWING, 155, 85, 5);
  chassis.pid_wait();
  chassis.pid_drive_set(-8_in, 45, false);
  chassis.pid_wait_until(-6_in);
  clamp_piston.set_value(true);
  chassis.pid_wait();
  IntakeFlex.move_velocity(200);
  Intake.move_velocity(600);

  //put mogo semi in corner
  chassis.pid_drive_set(20_in, 50, false);
  chassis.pid_wait_until(14_in);
  Intake.move_velocity(0);
  IntakeFlex.move_velocity(-200);
  chassis.pid_turn_set(60, 65);
  chassis.pid_wait();
  clamp_piston.set_value(false);

  //turn to other mogo
  chassis.pid_drive_set(2_in, 70, true);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(230, 65);
  chassis.pid_wait();

  //move back and clamp mogo
  chassis.pid_drive_set(-30_in, 80, true);
  chassis.pid_wait_until(-20_in);
  chassis.pid_speed_max_set(50);
  chassis.pid_wait_until(-28_in);
  clamp_piston.set_value(true);
  chassis.pid_wait();

  //turn to 2 stack and intake
  chassis.pid_turn_set(270, 65);
  chassis.pid_wait();
  Intake.move_velocity(600);
  IntakeFlex.move_velocity(200);
  chassis.pid_drive_set(20_in, 70, false);
  chassis.pid_wait_until(16_in);
  chassis.pid_speed_max_set(50);
  chassis.pid_wait();
  pros::delay(600);

  //go to positive corner
  chassis.pid_turn_set(0, 75);
  chassis.pid_wait_quick_chain();
  Intake.move_velocity(0);
  IntakeFlex.move_velocity(0);
  chassis.pid_drive_set(-22_in, 80, true);
  chassis.pid_wait();
  
}

void RedRightRush(){
  // Releases intake
  IntakeFlex.move_relative(180, 600);

  //move forward
  chassis.pid_drive_set(34_in, 110, true);
  chassis.pid_wait();

  //put doinker down and grab mogo
  chassis.pid_turn_set(-20, 85);
  chassis.pid_wait_quick();
  chassis.pid_drive_set(5_in, 60, true);
  chassis.pid_wait_until(4.5_in);
  doinker_piston.set_value(true);
  pros::delay(100);
  chassis.pid_wait();
  
  //move back with mogo
  chassis.pid_drive_set(-8_in, 75, true);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(0, 55);
  chassis.pid_wait();
  chassis.pid_drive_set(-16_in, 65, true);
  
  //put doinker up and turn around
  chassis.pid_wait_until(-14_in);
  doinker_piston.set_value(false);
  chassis.pid_wait();
  chassis.pid_turn_set(180, 65);
  chassis.pid_wait();

  //clamp mogo and score preload
  chassis.pid_drive_set(-14_in, 45, false);
  chassis.pid_wait_until(-12_in);
  clamp_piston.set_value(true);
  chassis.pid_wait();
  IntakeFlex.move_velocity(200);
  Intake.move_velocity(600);

  //put mogo semi in corner
  chassis.pid_drive_set(20_in, 75, false);
  chassis.pid_wait_until(14_in);
  Intake.move_velocity(0);
  IntakeFlex.move_velocity(-200);
  chassis.pid_turn_set(-60, 75);
  chassis.pid_wait();
  clamp_piston.set_value(false);

  //turn to other mogo
  chassis.pid_drive_set(2_in, 70, true);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(130, 65);
  chassis.pid_wait();

  //move back and clamp mogo
  chassis.pid_drive_set(-32_in, 80, true);
  chassis.pid_wait_until(-20_in);
  chassis.pid_speed_max_set(45);
  chassis.pid_wait_until(-30_in);
  clamp_piston.set_value(true);
  chassis.pid_wait();

  //turn to 2 stack and intake
  chassis.pid_turn_set(90, 65);
  chassis.pid_wait();
  Intake.move_velocity(600);
  IntakeFlex.move_velocity(200);
  chassis.pid_drive_set(22_in, 70, false);
  chassis.pid_wait_until(16_in);
  chassis.pid_speed_max_set(50);
  chassis.pid_wait();
  pros::delay(600);

  //go to positive corner
  chassis.pid_turn_set(0, 75);
  chassis.pid_wait_quick_chain();
  Intake.move_velocity(0);
  IntakeFlex.move_velocity(0);
  chassis.pid_drive_set(-22_in, 80, true);
  chassis.pid_wait();
}

void BlueRightAWP(){
  // Releases intake
  IntakeFlex.move_relative(180, 600);
  Arm.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
  
  //score alliance
  Arm.move_velocity(200);
  pros::delay(1200);
  Arm.move_velocity(-200);

  //move back and turn to alliance 2 stack
  chassis.pid_drive_set(-12_in, 65, true);
  chassis.pid_wait();
  chassis.pid_turn_set(45, 75);
  chassis.pid_wait();
  Arm.move_velocity(0);

  //move to 2 stack and grab the top ring with doinker
  chassis.pid_drive_set(12_in, 65, true);
  chassis.pid_wait();
  doinker_piston.set_value(true);
  pros::delay(250);
  chassis.pid_turn_set(90, 75);
  chassis.pid_wait_until(75);
  doinker_piston.set_value(false);
  chassis.pid_wait();

  //intake top ring
  IntakeFlex.move_velocity(200);
  chassis.pid_turn_set(50, 75);
  chassis.pid_wait();
  chassis.pid_drive_set(16_in, 75, true);
  chassis.pid_wait();
  pros::delay(400);
  chassis.pid_drive_set(-8_in, 65, false);
  chassis.pid_wait();

  //move to mogo and clamp
  chassis.pid_turn_set(-50, 75);
  chassis.pid_wait();
  chassis.pid_drive_set(-24_in, 45, true);
  chassis.pid_wait_until(-22);
  IntakeFlex.move_velocity(0);
  clamp_piston.set_value(true);

  //turn towards two stack
  chassis.pid_turn_set(-171, 75);
  chassis.pid_wait();

  //move and intake ring
  Intake.move_velocity(400);
  IntakeFlex.move_velocity(200);
  chassis.pid_drive_set(22_in, 65, true);
  chassis.pid_wait_until(18_in);
  chassis.pid_speed_max_set(45);
  chassis.pid_wait_quick_chain();
  IntakeFlex.move_velocity(0);
  chassis.pid_drive_set(-6_in, 65, false);
  chassis.pid_wait();
  pros::delay(600);

  //touch ladder
  Arm.move_velocity(200);
  chassis.pid_swing_set(ez::LEFT_SWING, -303, 75);
  chassis.pid_wait();
  chassis.pid_drive_set(16_in, 65, true);
  chassis.pid_wait();
  Arm.move_velocity(0);
  Arm.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
}

void RedLeftAWP(){
  // Releases intake
  IntakeFlex.move_relative(180, 600);
  Arm.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
  
  //score alliance
  Arm.move_velocity(200);
  pros::delay(1200);
  Arm.move_velocity(-200);

  //move back and turn to alliance 2 stack
  chassis.pid_drive_set(-12_in, 65, true);
  chassis.pid_wait();
  chassis.pid_turn_set(-24, 75);
  chassis.pid_wait();
  Arm.move_velocity(0);

  //move to 2 stack and grab the top ring with doinker
  chassis.pid_drive_set(14_in, 55, true);
  chassis.pid_wait();
  doinker_piston.set_value(true);
  chassis.pid_turn_set(-70, 45);
  chassis.pid_wait_until(-55);
  doinker_piston.set_value(false);
  chassis.pid_wait();

  //intake top ring
  chassis.pid_turn_set(-95, 75);
  chassis.pid_wait_quick();
  IntakeFlex.move_velocity(200);
  chassis.pid_drive_set(20_in, 45, true);
  chassis.pid_wait();
  pros::delay(400);
  chassis.pid_drive_set(-8_in, 45, false);
  chassis.pid_wait_quick();

  //move to mogo and clamp
  chassis.pid_turn_set(50, 75);
  chassis.pid_wait_quick();
  chassis.pid_drive_set(-24_in, 85, true);
  chassis.pid_wait_until(-16_in);
  chassis.pid_speed_max_set(45);
  chassis.pid_wait_until(-22_in);
  clamp_piston.set_value(true);
  chassis.pid_wait();
  IntakeFlex.move_velocity(0);

  //turn towards two stack
  chassis.pid_turn_set(-187, 75);
  chassis.pid_wait();

  //move and intake ring
  Intake.move_velocity(400);
  IntakeFlex.move_velocity(200);
  chassis.pid_drive_set(20_in, 65, true);
  chassis.pid_wait_until(16_in);
  chassis.pid_speed_max_set(45);
  chassis.pid_wait();
  chassis.pid_drive_set(-6_in, 65, false);
  chassis.pid_wait();
  pros::delay(600);

  //touch ladder
  chassis.pid_drive_set(-16_in, 65, true);
  chassis.pid_wait();
  Arm.move_velocity(200);
  chassis.pid_turn_set(-54, 75);
  chassis.pid_wait();
  chassis.pid_drive_set(2_in, 65, false);
  chassis.pid_wait();
  Arm.move_velocity(0);
  Arm.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
}

void prog(){
  // Releases intake
  IntakeFlex.move_relative(180, 600);
  //Sets arm brake mode to hold for consistency
  Arm.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);

  //score alliance
  Intake.move_velocity(600);
  pros::delay(800);
  Intake.move_velocity(0);
  //                                3 POINTS

  //============================================================STAGE 1=================================================================
  
  //turn to mogo
  chassis.pid_drive_set(10, 45, true);
  chassis.pid_wait();
  chassis.pid_turn_set(90, 75);
  chassis.pid_wait();

  //move back and clamp mogo
  chassis.pid_drive_set(-24_in, 45, true);
  chassis.pid_wait_until(-22);
  clamp_piston.set_value(true);
  chassis.pid_wait();

  //turn to ladder side ring
  chassis.pid_turn_set(0, 75);
  chassis.pid_wait();
  
  //go forward and intake ring
  Intake.move_velocity(600);
  IntakeFlex.move_velocity(200);
  chassis.pid_drive_set(24_in, 65, true);
  chassis.pid_wait_until(18_in);
  chassis.pid_speed_max_set(35);
  chassis.pid_wait();

  //turn towards the ring past the double lines
  chassis.pid_turn_set(-25, 75);
  chassis.pid_wait();

  //go forward and intake ring
  chassis.pid_drive_set(52_in, 75, true);  
  chassis.pid_wait();
  pros::delay(200);

  //turn towards the neutral stake ring and raise arm
  chassis.pid_turn_set(-180, 75);
  chassis.pid_wait();
  chassis.pid_drive_set(3_in, 35, false);
  chassis.pid_wait();
  
  //go forward and intake ring into the arm and score
  Arm.move_relative(355, 200);
  chassis.pid_swing_set(ez::LEFT_SWING, -90, SWING_SPEED, 20);
  chassis.pid_wait();
  Intake.move_velocity(600);
  chassis.pid_drive_set(3_in, 25, false);
  chassis.pid_wait();
  pros::delay(575);
  Intake.move_velocity(-100);
  pros::delay(50);
  Intake.move_velocity(0);
  Arm.move_velocity(200);
  pros::delay(1000);
  Arm.move_velocity(0);

  //move back and turn towards the L pattern rings
  chassis.pid_drive_set(-11_in, 65, true);
  chassis.pid_wait();
  chassis.pid_turn_set(-180, 75);
  chassis.pid_wait();

  //drive forward and intake 3 rings
  Intake.move_velocity(600);
  chassis.pid_drive_set(58_in, 65, true);
  chassis.pid_wait_until(24_in);
  chassis.pid_speed_max_set(30);
  chassis.pid_wait();

  //turn towards last corner ring
  chassis.pid_swing_set(ez::RIGHT_SWING, -90, SWING_SPEED);
  chassis.pid_wait();

  //move forward and intake 5th ring
  chassis.pid_drive_set(12_in, 65, true);
  chassis.pid_wait();

  //swing into corner and unclamp mogo
  chassis.pid_swing_set(ez::RIGHT_SWING,70, SWING_SPEED);
  chassis.pid_wait();
  Arm.move_velocity(-200);
  Intake.move_velocity(-50);
  chassis.pid_drive_set(-10_in, 60, true);
  chassis.pid_wait_until(-8_in);
  Intake.move_velocity(0);
  clamp_piston.set_value(false);
  chassis.pid_wait();

  //                                16 POINTS

  //============================================================STAGE 2=================================================================
  
  //grab other mogo
  chassis.pid_drive_set(36_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  chassis.pid_turn_set(-90, 75);
  chassis.pid_wait();
  Intake.move_velocity(-600);
  IntakeFlex.move_velocity(-200);
  chassis.pid_drive_set(-24_in, 75, true);
  chassis.pid_wait_quick();
  chassis.pid_drive_set(-16_in, 35, false);
  chassis.pid_wait_until(-14_in);
  clamp_piston.set_value(true);
  Arm.move_velocity(0);
  chassis.pid_wait();

  //start up intake and turn to ladder side ring
  chassis.pid_turn_set(0, 75);
  chassis.pid_wait();
  
  //go forward and intake ring
  Intake.move_velocity(600);
  IntakeFlex.move_velocity(200);
  chassis.pid_drive_set(24_in, 65, true);
  chassis.pid_wait_until(18_in);
  chassis.pid_speed_max_set(35);
  chassis.pid_wait();

  //turn towards the ring past the double lines
  chassis.pid_turn_set(25, 75);
  chassis.pid_wait();

  //go forward and intake ring
  chassis.pid_drive_set(52_in, 75, true);  
  chassis.pid_wait();
  pros::delay(200);

  //turn towards the neutral stake ring and raise arm
  chassis.pid_turn_set(180, 75);
  chassis.pid_wait();
  chassis.pid_drive_set(2_in, 35, false);
  chassis.pid_wait();
  
  //go forward and intake ring into the arm and score
  Arm.move_relative(355, 200);
  chassis.pid_swing_set(ez::RIGHT_SWING, 90, SWING_SPEED, 20);
  chassis.pid_wait();
  Intake.move_velocity(600);
  chassis.pid_drive_set(4_in, 25, false);
  chassis.pid_wait();
  pros::delay(650);
  Intake.move_velocity(-100);
  pros::delay(50);
  Intake.move_velocity(0);
  Arm.move_velocity(200);
  pros::delay(950);
  Arm.move_velocity(0);

  //move back and turn towards the L pattern rings
  chassis.pid_drive_set(-12_in, 65, true);
  chassis.pid_wait();
  chassis.pid_turn_set(180, 75);
  chassis.pid_wait();

  //drive forward and intake 3 rings
  Intake.move_velocity(600);
  chassis.pid_drive_set(58_in, 65, true);
  chassis.pid_wait_until(24_in);
  chassis.pid_speed_max_set(30);
  chassis.pid_wait();

  //turn towards last corner ring
  chassis.pid_swing_set(ez::LEFT_SWING, 90, SWING_SPEED);
  chassis.pid_wait();

  //move forward and intake 5th ring
  chassis.pid_drive_set(12_in, 65, true);
  chassis.pid_wait();

  //swing into corner and unclamp mogo
  chassis.pid_swing_set(ez::LEFT_SWING,-70, SWING_SPEED);
  chassis.pid_wait();
  Arm.move_velocity(-200);
  Intake.move_velocity(-50);
  chassis.pid_drive_set(-10_in, 60, true);
  chassis.pid_wait_until(-8_in);
  Intake.move_velocity(0);
  clamp_piston.set_value(false);
  chassis.pid_wait();

  //                                16 POINTS

  //============================================================STAGE 3=================================================================

  //turn toward other side
  chassis.pid_turn_set(0, 75);
  chassis.pid_wait();

  //drive to center mogo
  chassis.pid_swing_set(ez::RIGHT_SWING,-130, 120, 90);
  chassis.pid_wait();
  chassis.pid_turn_set(135, 75);
  chassis.pid_wait();

  // //turn towards empty mogo
  // chassis.pid_turn_set(90, 75);
  // chassis.pid_wait();

  // //move to mogo and clamp
  // chassis.pid_drive_set(16_in, 65, true);
  // chassis.pid_wait_until(14_in);
  // clamp_piston.set_value(true);


}

void test(){
  chassis.pid_drive_set(8_in, 45, false);
  chassis.pid_wait();
}