#include "autons.hpp"
#include <sys/_intsup.h>
#include "EZ-Template/util.hpp"
#include "okapi/api/units/QLength.hpp"
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

void sixRingBlue(){
  // Releases intake
  IntakeFlex.move_relative(180, 600);
  
  //move back to mogo and clamp
  chassis.pid_drive_set(-16_in, 85, false);
  chassis.pid_wait();
  chassis.pid_swing_set(ez::LEFT_SWING, -35, 80, 15);
  chassis.pid_wait();
  chassis.pid_drive_set(-4_in, 65, false);
  chassis.pid_wait();
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

  //swing to be parallel to the 4 stack rings and intake
  chassis.pid_swing_set(ez::LEFT_SWING, -90, 80);
  chassis.pid_wait();

  //turn to 2 stack
  chassis.pid_turn_set(-10, 65);
  chassis.pid_wait();

  //move to 2 stack and intake
  chassis.pid_drive_set(14_in, 65, false);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-6_in, 35, true);
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
  Arm.move_velocity(0);

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
  chassis.pid_wait();
  clamp_piston.set_value(true);

  //turn to 4 stack and start up intake
  Intake.move_velocity(600);
  IntakeFlex.move_velocity(200);
  chassis.pid_turn_set(140, 50);
  chassis.pid_wait();

  //move forward and disrupt
  chassis.pid_drive_set(22_in, 45, false);
  chassis.pid_wait_until(8_in);
  chassis.pid_speed_max_set(65);
  chassis.pid_wait();

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
  IntakeFlex.move_velocity(-200);
  Arm.move_velocity(200);
  chassis.pid_drive_set(35_in, 35, false);
  chassis.pid_wait();
  Arm.move_velocity(0);
}

void BlueLeftRush(){
  // Releases intake
  IntakeFlex.move_relative(180, 600);

  //move forward
  doinker_piston.set_value(true);
  IntakeFlex.move_velocity(200);
  chassis.pid_drive_set(34_in, 127, true);
  chassis.pid_wait_quick_chain();
  doinker_clamp.set_value(false);
  
  //put doinker down and grab mogo
  chassis.pid_swing_set(ez::LEFT_SWING, 0, 95, 15);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-24_in, 100, true);
  
  //put doinker up and turn around
  chassis.pid_wait_until(-16_in);
  doinker_clamp.set_value(true);
  chassis.pid_wait();
  IntakeFlex.move_velocity(0);
  chassis.pid_turn_set(165, 65);
  chassis.pid_wait();
  doinker_piston.set_value(false);

  //clamp mogo and score preload
  chassis.pid_drive_set(-18_in, 35, false);
  chassis.pid_wait_until(-16_in);
  clamp_piston.set_value(true);
  chassis.pid_wait();
  Intake.move_velocity(600);

  //put mogo semi in corner
  chassis.pid_turn_set(0, 65);
  chassis.pid_wait(); 
  Intake.move_velocity(0);
  chassis.pid_drive_set(-14_in, 100, true);
  chassis.pid_wait_quick_chain();
  clamp_piston.set_value(false);
  chassis.pid_drive_set(3_in, 100, true);
  chassis.pid_wait();

  //turn to other mogo
  chassis.pid_turn_set(-104, 65);
  chassis.pid_wait();

  //move back and clamp mogo
  chassis.pid_drive_set(-32_in, 80, true);
  chassis.pid_wait_until(-22_in);
  chassis.pid_speed_max_set(30);
  chassis.pid_wait_until(-30_in);
  clamp_piston.set_value(true);
  chassis.pid_wait();

  //turn to 2 stack and intake
  chassis.pid_turn_set(120, 65);
  chassis.pid_wait();
  IntakeFlex.move_velocity(200);
  intake_piston.set_value(true);
  chassis.pid_drive_set(26_in, 70, false);
  chassis.pid_wait_quick();
  intake_piston.set_value(false);
  chassis.pid_drive_set(-14_in, 50, false);
  chassis.pid_wait_quick_chain();
  Intake.move_velocity(600);

  //go to positive corner
  chassis.pid_swing_set(ez::LEFT_SWING, 22, 95, 5);
  chassis.pid_wait();
  chassis.pid_drive_set(-22_in, 95, true);
  chassis.pid_wait();
}

void RedRightRush(){
  // Releases intake
  IntakeFlex.move_relative(180, 600);

  //move forward
  doinker_piston.set_value(true);
  IntakeFlex.move_velocity(200);
  chassis.pid_drive_set(34_in, 127, true);
  chassis.pid_wait_quick_chain();
  doinker_clamp.set_value(false);
  

  //put doinker down and grab mogo
  chassis.pid_swing_set(ez::RIGHT_SWING, 0, 95, 15);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-56_in, 127, true);
  chassis.pid_wait();

}

void BlueLeftRushTug(){
  // Releases intake
  IntakeFlex.move_relative(180, 600);

  //move forward
  doinker_piston.set_value(true);
  IntakeFlex.move_velocity(200);
  chassis.pid_drive_set(34_in, 127, true);
  chassis.pid_wait_quick_chain();
  doinker_clamp.set_value(false);
  
  //put doinker down and grab mogo
  chassis.pid_swing_set(ez::LEFT_SWING, 0, 95, 15);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-56_in, 127, true);
  chassis.pid_wait();

}

void RedRightRushTug(){
  // Releases intake
  IntakeFlex.move_relative(180, 600);

  //move forward
  doinker_piston.set_value(true);
  IntakeFlex.move_velocity(200);
  chassis.pid_drive_set(34_in, 127, true);
  chassis.pid_wait_quick_chain();
  doinker_clamp.set_value(false);
  

  //put doinker down and grab mogo
  chassis.pid_swing_set(ez::RIGHT_SWING, 0, 95, 15);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-24_in, 95, true);
  
  //put doinker up and turn around
  chassis.pid_wait_until(-16_in);
  doinker_clamp.set_value(true);
  chassis.pid_wait();
  IntakeFlex.move_velocity(0);
  chassis.pid_turn_set(172, 65);
  chassis.pid_wait();
  doinker_piston.set_value(false);
  
  //clamp mogo and score preload
  chassis.pid_drive_set(-16_in, 35, false);
  chassis.pid_wait_until(-14_in);
  clamp_piston.set_value(true);
  chassis.pid_wait();
  Intake.move_velocity(600);

  
  //put mogo semi in corner
  chassis.pid_turn_set(-15, 65);
  chassis.pid_wait(); 
  Intake.move_velocity(0);
  chassis.pid_drive_set(-30_in, 127, true);
  chassis.pid_wait_quick_chain();
  clamp_piston.set_value(false);
  chassis.pid_drive_set(6_in, 120, false);
  chassis.pid_wait_quick();

  //turn to other mogo
  chassis.pid_turn_set(122, 65);
  chassis.pid_wait();

  //move back and clamp mogo
  chassis.pid_drive_set(-28_in, 95, true);
  chassis.pid_wait_until(-14_in);
  chassis.pid_speed_max_set(30);
  chassis.pid_wait_until(-26_in);
  clamp_piston.set_value(true);
  chassis.pid_wait();

  //turn to 2 stack and intake
  chassis.pid_turn_set(205, 75);
  chassis.pid_wait();
  IntakeFlex.move_velocity(200);
  intake_piston.set_value(true);
  chassis.pid_drive_set(26_in, 75, false);
  chassis.pid_wait_quick();
  intake_piston.set_value(false);
  chassis.pid_drive_set(-6_in, 60, false);
  chassis.pid_wait();
  Intake.move_velocity(600);

  //go to positive corner
  chassis.pid_swing_set(ez::RIGHT_SWING, 338, 95, 45);
  chassis.pid_wait();
  chassis.pid_drive_set(-18_in, 95, true);
  chassis.pid_wait();

}

void QualBlueLeftRush(){
  // Releases intake
  IntakeFlex.move_relative(180, 600);

  //move forward
  doinker_piston.set_value(true);
  IntakeFlex.move_velocity(200);
  chassis.pid_drive_set(34_in, 127, true);
  chassis.pid_wait_quick_chain();
  doinker_clamp.set_value(false);
  
  //put doinker down and grab mogo
  chassis.pid_swing_set(ez::LEFT_SWING, 0, 95, 15);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-24_in, 100, true);
  
  //put doinker up and turn around
  chassis.pid_wait_until(-16_in);
  doinker_clamp.set_value(true);
  chassis.pid_wait();
  IntakeFlex.move_velocity(0);
  chassis.pid_turn_set(165, 65);
  chassis.pid_wait();
  doinker_piston.set_value(false);

  //clamp mogo and score preload
  chassis.pid_drive_set(-16_in, 35, false);
  chassis.pid_wait_until(-14_in);
  clamp_piston.set_value(true);
  chassis.pid_wait();
  Intake.move_velocity(600);

  //put mogo semi in corner
  chassis.pid_turn_set(0, 65);
  chassis.pid_wait(); 
  Intake.move_velocity(0);
  chassis.pid_drive_set(-14_in, 100, true);
  chassis.pid_wait_quick_chain();
  clamp_piston.set_value(false);
  chassis.pid_drive_set(3_in, 100, true);
  chassis.pid_wait();

  //turn to other mogo
  chassis.pid_turn_set(-104, 65);
  chassis.pid_wait();

  //move back and clamp mogo
  chassis.pid_drive_set(-32_in, 80, true);
  chassis.pid_wait_until(-22_in);
  chassis.pid_speed_max_set(30);
  chassis.pid_wait_until(-30_in);
  clamp_piston.set_value(true);
  chassis.pid_wait();

  //turn to 2 stack and intake
  chassis.pid_turn_set(120, 65);
  chassis.pid_wait();
  IntakeFlex.move_velocity(0);
  intake_piston.set_value(true);
  chassis.pid_drive_set(26_in, 70, false);
  chassis.pid_wait_quick();
  intake_piston.set_value(false);
  chassis.pid_drive_set(-14_in, 50, false);
  chassis.pid_wait_quick_chain();
  Intake.move_velocity(600);
  
  //touch ladder
  Arm.move_velocity(200);
  chassis.pid_swing_set(ez::LEFT_SWING, 29, 50);
  chassis.pid_wait_quick_chain();
  Arm.move_velocity(0);
  chassis.drive_brake_set(pros::E_MOTOR_BRAKE_COAST);
  Arm.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  chassis.pid_drive_set(6_in, 70, false);
  chassis.pid_wait();
}

void QualRedRightRush(){
  // Releases intake
  IntakeFlex.move_relative(180, 600);

  //move forward
  doinker_piston.set_value(true);
  IntakeFlex.move_velocity(200);
  chassis.pid_drive_set(34_in, 127, true);
  chassis.pid_wait_quick_chain();
  doinker_clamp.set_value(false);
  

  //put doinker down and grab mogo
  chassis.pid_swing_set(ez::RIGHT_SWING, 0, 95, 15);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-24_in, 95, true);
  
  //put doinker up and turn around
  chassis.pid_wait_until(-16_in);
  doinker_clamp.set_value(true);
  chassis.pid_wait();
  IntakeFlex.move_velocity(0);
  chassis.pid_turn_set(172, 65);
  chassis.pid_wait();
  doinker_piston.set_value(false);
  
  //clamp mogo and score preload
  chassis.pid_drive_set(-16_in, 35, false);
  chassis.pid_wait_until(-14_in);
  clamp_piston.set_value(true);
  chassis.pid_wait();
  Intake.move_velocity(600);

  //put mogo semi in corner
  chassis.pid_turn_set(-15, 65);
  chassis.pid_wait(); 
  Intake.move_velocity(0);
  chassis.pid_drive_set(-30_in, 127, true);
  chassis.pid_wait_quick_chain();
  clamp_piston.set_value(false);
  chassis.pid_drive_set(6_in, 120, false);
  chassis.pid_wait_quick();

  //turn to other mogo
  chassis.pid_turn_set(122, 65);
  chassis.pid_wait();

  //move back and clamp mogo
  chassis.pid_drive_set(-28_in, 95, true);
  chassis.pid_wait_until(-14_in);
  chassis.pid_speed_max_set(30);
  chassis.pid_wait_until(-26_in);
  clamp_piston.set_value(true);
  chassis.pid_wait();

  //turn to 2 stack and intake
  chassis.pid_turn_set(205, 75);
  chassis.pid_wait();
  IntakeFlex.move_velocity(0);
  intake_piston.set_value(true);
  chassis.pid_drive_set(26_in, 75, false);
  chassis.pid_wait_quick();
  intake_piston.set_value(false);
  chassis.pid_drive_set(-6_in, 60, false);
  chassis.pid_wait();
  Intake.move_velocity(600);

  //touch ladder
  chassis.pid_swing_set(ez::RIGHT_SWING, 300, 90);
  chassis.pid_wait_quick_chain();
  Arm.move_velocity(200);
  chassis.pid_drive_set(6_in, 75, true);
  chassis.pid_wait();
  Arm.move_velocity(0);

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
  chassis.pid_drive_set(-12_in, 95, true);
  chassis.pid_wait_quick();
  chassis.pid_turn_set( -73, 90);
  chassis.pid_wait();
  Arm.move_velocity(0);

  chassis.pid_drive_set(-36_in, 95, true);
  chassis.pid_wait_until(-16_in);
  chassis.pid_speed_max_set(45);
  chassis.pid_wait_until(-34_in);
  clamp_piston.set_value(true);
  chassis.pid_wait_quick();

  chassis.pid_turn_set( -206, 90);
  chassis.pid_wait_quick();

  IntakeFlex.move_velocity(200);
  Intake.move_velocity(600);
  chassis.pid_drive_set(16_in, 95, true);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set( -162, 55);
  chassis.pid_wait_quick();
  chassis.pid_drive_set(10_in, 45, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(-8_in, 95, true);
  chassis.pid_wait_quick();
  chassis.pid_turn_set( -87, 90);
  chassis.pid_wait_quick();
  chassis.pid_drive_set(16_in, 95, true);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-2_in, 45, true);
  chassis.pid_wait_quick();
  Intake.move_velocity(25);

  chassis.pid_swing_set(ez::LEFT_SWING, -303, 90);
  chassis.pid_wait_quick();
  chassis.pid_drive_set(14_in, 45, true);
  chassis.pid_wait_quick();

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
  chassis.pid_drive_set(-12_in, 95, true);
  chassis.pid_wait_quick();
  chassis.pid_turn_set( 73, 90);
  chassis.pid_wait();
  Arm.move_velocity(0);

  chassis.pid_drive_set(-36_in, 95, true);
  chassis.pid_wait_until(-16_in);
  chassis.pid_speed_max_set(45);
  chassis.pid_wait_until(-34_in);
  clamp_piston.set_value(true);
  chassis.pid_wait_quick();

  chassis.pid_turn_set( 206, 90);
  chassis.pid_wait_quick();

  IntakeFlex.move_velocity(200);
  Intake.move_velocity(600);
  chassis.pid_drive_set(16_in, 95, true);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set( 162, 55);
  chassis.pid_wait_quick();
  chassis.pid_drive_set(10_in, 45, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(-8_in, 95, true);
  chassis.pid_wait_quick();
  chassis.pid_turn_set( 87, 90);
  chassis.pid_wait_quick();
  chassis.pid_drive_set(16_in, 95, true);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-2_in, 45, true);
  chassis.pid_wait_quick();
  Intake.move_velocity(25);

  chassis.pid_swing_set(ez::RIGHT_SWING, 303, 90);
  chassis.pid_wait_quick();
  chassis.pid_drive_set(14_in, 45, true);
  chassis.pid_wait_quick();
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
  chassis.pid_drive_set(12, 75, true);
  chassis.pid_wait_quick();
  chassis.pid_turn_set(90, 75);
  chassis.pid_wait_quick();

  //move back and clamp mogo
  chassis.pid_drive_set(-24_in, 50, true);
  chassis.pid_wait_until(-22);
  clamp_piston.set_value(true);
  chassis.pid_wait_quick();

  //turn to ladder side ring
  chassis.pid_turn_set(0, 90);
  chassis.pid_wait();
  
  //go forward and intake ring
  Intake.move_velocity(600);
  IntakeFlex.move_velocity(200);
  chassis.pid_drive_set(22_in, 75, true);
  chassis.pid_wait_until(16_in);
  chassis.pid_speed_max_set(35);
  chassis.pid_wait();

  //turn towards the ring past the double lines
  chassis.pid_turn_set(-25, 75);
  chassis.pid_wait();

  //go forward and intake ring
  chassis.pid_drive_set(52_in, 127, true);  
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-2_in, 90, false);  
  chassis.pid_wait_quick();

  //turn towards the neutral stake ring intake
  chassis.pid_turn_set(-163, 60);
  chassis.pid_wait_quick();
  chassis.pid_drive_set(32_in, 90, true);
  chassis.pid_wait_until(26_in);
  chassis.pid_speed_max_set(35);
  chassis.pid_wait_quick_chain();
  Arm.move_relative(300, 200);
  Intake.move_velocity(0);
  chassis.pid_drive_set(-7_in, 75, true);
  chassis.pid_wait();
  Intake.move_velocity(600);
  
  //go forward and intake ring into the arm and score
  chassis.pid_turn_set(-90, 75);
  chassis.pid_wait_quick();
  chassis.pid_drive_set(4_in, 50, true);
  chassis.pid_wait_quick();
  pros::delay(200);
  Intake.move_velocity(-100);
  pros::delay(30);
  Intake.move_velocity(0);
  Arm.move_velocity(200);
  pros::delay(1200);
  Arm.move_velocity(0);

  //move back and turn towards the L pattern rings
  chassis.pid_drive_set(-12.5_in, 65, true);
  chassis.pid_wait_quick();
  chassis.pid_turn_set(-180, 75);
  chassis.pid_wait();

  //drive forward and intake 3 rings
  Intake.move_velocity(600);
  chassis.pid_drive_set(58_in, 95, true);
  chassis.pid_wait_until(20_in);
  chassis.pid_speed_max_set(45);
  chassis.pid_wait_quick();

  //turn towards last corner ring
  chassis.pid_swing_set(ez::RIGHT_SWING, -90, 90);
  chassis.pid_wait();

  //move forward and intake 5th ring
  chassis.pid_drive_set(12_in, 65, true);
  chassis.pid_wait_quick_chain();

  //swing into corner and unclamp mogo
  chassis.pid_swing_set(ez::RIGHT_SWING,70, 85);
  chassis.pid_wait_quick();
  Arm.move_velocity(-200);
  Intake.move_velocity(-200);
  chassis.pid_drive_set(-10_in, 60, true);
  chassis.pid_wait_until(-8_in);
  Intake.move_velocity(0);
  clamp_piston.set_value(false);
  chassis.pid_wait();

  //                                16 POINTS

  //============================================================STAGE 2=================================================================
  
  //grab other mogo
  chassis.pid_drive_set(36_in, 127, true);
  chassis.pid_wait();
  chassis.pid_turn_set(-90, 75);
  chassis.pid_wait();
  Intake.move_velocity(-600);
  IntakeFlex.move_velocity(-200);
  chassis.pid_drive_set(-26_in, 75, true);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-16_in, 35, false);
  chassis.pid_wait_until(-12_in);
  clamp_piston.set_value(true);
  Arm.move_velocity(0);
  chassis.pid_wait();

  //turn to ladder side ring
  chassis.pid_turn_set(0, 90);
  chassis.pid_wait();
  
  //go forward and intake ring
  Intake.move_velocity(600);
  IntakeFlex.move_velocity(200);
  chassis.pid_drive_set(24_in, 85, true);
  chassis.pid_wait_until(16_in);
  chassis.pid_speed_max_set(35);
  chassis.pid_wait();

  //turn towards the ring past the double lines
  chassis.pid_turn_set(25, 90);
  chassis.pid_wait();

  //go forward and intake ring
  chassis.pid_drive_set(52_in, 85, true);  
  chassis.pid_wait_quick();
  chassis.pid_drive_set(-2_in, 90, false);  
  chassis.pid_wait_quick();

  //turn towards the neutral stake ring intake
  chassis.pid_turn_set(163, 60);
  chassis.pid_wait_quick();
  chassis.pid_drive_set(28_in, 75, true);
  chassis.pid_wait_quick();
  Arm.move_relative(325, 200);
  Intake.move_velocity(0);
  chassis.pid_drive_set(-6_in, 75, true);
  chassis.pid_wait();
  Intake.move_velocity(600);
  
  //go forward and intake ring into the arm and score
  chassis.pid_turn_set(90, 75);
  chassis.pid_wait_quick();
  chassis.pid_drive_set(4_in, 50, true);
  chassis.pid_wait_quick();
  pros::delay(200);
  Intake.move_velocity(-100);
  pros::delay(30);
  Intake.move_velocity(0);
  Arm.move_velocity(200);
  pros::delay(1200);
  Arm.move_velocity(0);

  //move back and turn towards the L pattern rings
  chassis.pid_drive_set(-12_in, 65, true);
  chassis.pid_wait_quick();
  chassis.pid_turn_set(180, 90);
  chassis.pid_wait();

  //drive forward and intake 3 rings
  Intake.move_velocity(600);
  chassis.pid_drive_set(58_in, 95, true);
  chassis.pid_wait_until(20_in);
  chassis.pid_speed_max_set(50);
  chassis.pid_wait_quick();

  //turn towards last corner ring
  chassis.pid_swing_set(ez::LEFT_SWING, 90, 90);
  chassis.pid_wait();

  //move forward and intake 5th ring
  chassis.pid_drive_set(14_in, 65, true);
  chassis.pid_wait_quick();

  //swing into corner and unclamp mogo
  chassis.pid_drive_set(-6_in, 120, true);
  chassis.pid_wait_quick();
  chassis.pid_turn_set(-42, 65);
  chassis.pid_wait();
  Arm.move_velocity(0); 
  Intake.move_velocity(-600);
  clamp_piston.set_value(false);
  chassis.pid_drive_set(-14_in, 120, true);
  chassis.pid_wait_quick_chain();
  Intake.move_velocity(0);
  
  //                                16 POINTS

  //============================================================STAGE 3=================================================================

  // turn toward center ring
  Intake.move_velocity(150);
  IntakeFlex.move_velocity(200);
  chassis.pid_drive_set(4_in, 65, false);
  chassis.pid_wait_quick();
  chassis.pid_turn_set(-45, 75);
  chassis.pid_wait_quick();
  
  //drive to center ring and intake
  Arm.move_velocity(-200); 
  chassis.pid_drive_set(62_in, 120, true);
  chassis.pid_wait_quick_chain();
  
  // Drive to second red ring and intake
  Arm.move_velocity(0);
  chassis.pid_turn_set(-45, 75);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(40_in, 120, true);
  chassis.pid_wait_until(24_in);
  Intake.move_velocity(0);
  chassis.pid_wait();
  pros::delay(150);

  // Turn towards closest blue ring mogo
  chassis.pid_turn_set(180, 75);
  chassis.pid_wait_quick();
  Arm.move_velocity(0);

  // Clamp mogo and turn towards corner
  chassis.pid_drive_set(-36_in, 45, true);
  chassis.pid_wait_until(-32_in); 
  clamp_piston.set_value(true);
  chassis.pid_wait_quick();
  chassis.pid_turn_set(110, 75);
  chassis.pid_wait();

  // Score mogo, move forward, and turn towards second blue ring mogo
  chassis.pid_drive_set(-30_in, 110, true);
  chassis.pid_wait_quick_chain();
  clamp_piston.set_value(false);
  chassis.pid_drive_set(8_in, 75, true);
  chassis.pid_wait_quick();
  chassis.pid_turn_set(-90, 75);
  chassis.pid_wait();

  // Clamp mogo and turn towards corner
  chassis.pid_drive_set(-60_in, 120, true);
  chassis.pid_wait_quick();
  chassis.pid_drive_set(-12_in, 45, true);
  chassis.pid_wait_until(-10_in); 
  clamp_piston.set_value(true);
  chassis.pid_wait_quick();
  chassis.pid_turn_set(-110, 75);
  chassis.pid_wait();

  // Score mogo, move forward, and turn towards empty mogo
  chassis.pid_drive_set(-28_in, 110, true);
  chassis.pid_wait_quick_chain();
  clamp_piston.set_value(false);
  chassis.pid_drive_set(8_in, 75, true);
  chassis.pid_wait_quick();
  chassis.pid_drive_set(12_in, 110, true);
  chassis.pid_wait_quick();

  // // Clamp empty mogo

  // // Start up intake and turn towards corner of L rings

  // // Intake corner ring and raise arm to hang

  // // Go to ladder and hang
  

}

void test(){
  chassis.pid_drive_set(8_in, 45, false);
  chassis.pid_wait();
}