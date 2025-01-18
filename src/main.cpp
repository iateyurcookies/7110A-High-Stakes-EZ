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

// Initialize important variables
static int autonNum = 0; //----------------------> auton selector integer
static int DunkPos = 0; //-----------------------> arm macro integer
static bool isMoving = false; //-----------------> is arm active
static bool clampPiston{false}; //---------------> toggle for mogo clamp
static bool doinkPiston{false}; //---------------> toggle for doinker
static bool team{true}; //-----------------------> true = red    false = blue
static int grabRingPos = 16; //------------------> position values for the macro
static int almostScoreRingPos = 90;
static int scoreRingPos = 140;
static int returnHomePos = 2;

// // Color sorting function for the intake
// void colorSort(bool teamCol) {
//   if (teamCol == true) {       //------------------------> If Red, spit out blue
//     optical.set_led_pwm(100);  //----------> Lights up LED
//     if ((optical.get_hue() >= 200) && (optical.get_hue() <= 240)) {
//       if (IntakeSensor.get_angle() == 1000) {  // After Intake spins near the top,
//         Intake.move_velocity(0);               // pause intake to let the ring fly over
//         pros::delay(330);
//       }
//     }
//     Intake.move_velocity(600);    //------> Spins intake
//   } else if (teamCol == false) {  //-----------------> If Blue, spit out red
//     optical.set_led_pwm(100);     //----------> Lights up LED
//     if (optical.get_hue() >= 345 && optical.get_hue() <= 20) {
//       if (IntakeSensor.get_angle() == 1000) {  // After Intake spins near the top,
//         Intake.move_velocity(0);               // pause intake to let the ring fly over
//         pros::delay(330);
//       }
//     }
//     Intake.move_velocity(600);  //------> Spins intake
//   }
// }

// Chassis constructor
ez::Drive chassis(
    {-10, -9, -8}, // Left Chassis Ports (negative port will reverse it!)
    {7, 6, 5},   // Right Chassis Ports (negative port will reverse it!)

    1, //-------------------------------> IMU Port
    3.25, //----------------------> Wheel Diameter (Remember, 4" wheels without screw holes are actually 4.125!)
    450 //---------------------------------> Wheel RPM
);

void initialize() {
  autonNum = 0;
  DunkPos = 0;
  pros::delay(500); //-----------------------> Stop the user from doing anything while legacy ports configure
  Arm.set_voltage_limit(5500); //-------------------> Set 5.5 watt motor limit for the half watt arm motor
  IntakeFlex.set_voltage_limit(5500);
  ArmSensor.reset();
  Arm.tare_position();

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

  // Auton selector, Runs selected auton based off auton variable
  if /*---*/ (autonNum == 0) {
    sixRingBlue();
  } else if (autonNum == 1) {
    sixRingRed();
  } else if (autonNum == 2) {
    BlueLeftRush();
  } else if (autonNum == 3) {
    RedRightRush();
  } else if (autonNum == 4) {
    BlueRightAWP();
  } else if (autonNum == 5) {
    RedLeftAWP();
  } else if (autonNum == 6) {
    prog();
  } else if (autonNum == 7) {
    test();
  }
}

void opcontrol() {
  if(autonNum == 4 || autonNum == 5){
    DunkPos = 4;
  }
  // This is preference to what you like to drive on
  pros::motor_brake_mode_e_t driver_preference_brake = MOTOR_BRAKE_COAST;
  chassis.drive_brake_set(driver_preference_brake);
  Arm.set_brake_mode(MOTOR_BRAKE_COAST);

  // Enable/Disable for EZ Template built in PID tuner
  chassis.pid_tuner_disable();

  pros::Task colorSort([&]() { //---> Color sorting for the intake
    if (team == true) { //---------------------------> If Red, spit out blue
      optical.set_led_pwm(100);  //-----------> Lights up LED
      if ((optical.get_hue() >= 200) && (optical.get_hue() <= 240)) {
        if (IntakeSensor.get_angle() == 1000) { //---> After Intake spins near the top,
          Intake.move_velocity(0); //------> pause intake to let the ring fly over
          pros::delay(330);
        }
      }
    } else if (team == false) { //-------------------> If Blue, spit out red
      optical.set_led_pwm(100); //------------> Lights up LED
      if (optical.get_hue() >= 345 && optical.get_hue() <= 20) {
        if (IntakeSensor.get_angle() == 1000) { //---> After Intake spins near the top,
          Intake.move_velocity(0); //------> pause intake to let the ring fly over
          pros::delay(330);
        }
      }
    }
  });

  pros::Task controller_screen([&]() { //-------> Prints important shit to the controller screen
    while (true) {
      //                        // gets average motor temp for the drivetrain
      float avrMotorTemp = (FrontL.get_temperature() + MidL.get_temperature() + BackL.get_temperature() 
                           + FrontR.get_temperature() + MidR.get_temperature() + BackR.get_temperature()) / 6;

      std::string teamStr = ""; // Initializes the strings for the info
      std::string autoStr = "";

      if (autonNum == 0) //---> Prints the auton name depending on autoNum
        autoStr = "4B-";
      else if (autonNum == 1)
        autoStr = "4R-";
      else if (autonNum == 2)
        autoStr = "RB+";
      else if (autonNum == 3)
        autoStr = "RR+";
      else if (autonNum == 4)
        autoStr = "BWP-";
      else if (autonNum == 5)
        autoStr = "RWP-";
      else if (autonNum == 6)
        autoStr = "PROG";
      else if (autonNum == 7)
        autoStr = "Test";

      if (team) //------------> Prints what color alliance you are
        teamStr = "R";
      else
        teamStr = "B";

      // Combines all the strings and prints it as one to the controller screen
      std::string controllerString = teamStr + "A: " + autoStr + " T:" + std::to_string(avrMotorTemp);
      master.print(0, 1, "%s", controllerString);

      // delay so the brain doesn't get fried
      pros::delay(150);
    }
  });

  while (true) {  // Driver control while loop
//--------------------------------------------------------------Setup-------------------------------------------------------------------

    // Standard split arcade(left stick = forward-back | right stick = turning)
    chassis.opcontrol_arcade_standard(ez::SPLIT);

    // Left button cycles back though the autons and right button cycles forward
    if (autonNum == 8) {
      autonNum = 0;
    }if(master.get_digital_new_press(DIGITAL_UP)) {
      autonNum++;
    }if(master.get_digital_new_press(DIGITAL_LEFT)) {
      autonNum--;
    }

    // Pressing the Up and Left buttons will change the bots color sorting to the opposite color for color-sorting
    if (master.get_digital_new_press(DIGITAL_UP) 
     && master.get_digital_new_press(DIGITAL_X)) {
      team = !team;
    }

//--------------------------------------------------------------Pistons-----------------------------------------------------------------

    // Pressing B will acuate the mobile goal clamp (is toggle)
    if (master.get_digital_new_press(DIGITAL_B)) {
      if (!clampPiston) {
        clamp_piston.set_value(true);
        clampPiston = !clampPiston;
      } else {
        clamp_piston.set_value(false);
        clampPiston = !clampPiston;
      }
    }

    // Pressing Right or Y will acuate THE DOINKER (is toggle)
    if (master.get_digital_new_press(DIGITAL_Y)) {
      if (!doinkPiston) {
        doinker_piston.set_value(true);
        doinkPiston = !doinkPiston;
      } else {
        doinker_piston.set_value(false);
        doinkPiston = !doinkPiston;
      }
    }

//-------------------------------------Lady Brown macro code (make sure it starts at the hardstop)--------------------------------------

    if(master.get_digital_new_press(DIGITAL_RIGHT)){
      DunkPos = 5;
      Arm.move_velocity(-200);
    }
    if (master.get_digital_new_press(DIGITAL_DOWN) 
     && master.get_digital(DIGITAL_RIGHT) != true
     && isMoving == false) {
      if (DunkPos == 0) { //------------------> If it's at pos 0, go to pos 1 (ring grabbing)
        ArmSensor.reset_position();
        isMoving = true;
        Arm.move_velocity(200);
        DunkPos = 1;
      } else if (DunkPos == 2) { //-----------> Goes from ring grabbing (pos 1) to scoring (pos 3)
        isMoving = true;
        Arm.move_velocity(200);
        DunkPos = 3;
      } 
      else if (DunkPos == 4) { //-----------> Goes from scoring (pos 3) to home (pos 0)
        isMoving = true;
        Arm.move_velocity(-200);
        DunkPos = 5;
      } 
    }

    //----------------------------------------> Goes from home pos to ring grabbing
    if (DunkPos == 1 && ArmSensor.get_position() >= 100 * grabRingPos) {
      DunkPos = 2;
      isMoving = false;
      Arm.set_brake_mode(MOTOR_BRAKE_HOLD);
      Arm.move_velocity(0);
    } //--------------------------------------> Goes from ring grabbing to lining up
    // else if (DunkPos == 3 && ArmSensor.get_position() >= 100 * almostScoreRingPos) {
    //   Arm.move_velocity(0);
    //   DunkPos = 4;
    //   isMoving = false;
    //   Arm.set_brake_mode(MOTOR_BRAKE_HOLD);
    //   Arm.move_velocity(0);
    //} //--------------------------------------> Goes from lining up to scored
    else if (DunkPos == 3 && ArmSensor.get_position() >= 100 * scoreRingPos) {
      Arm.move_velocity(0);
      DunkPos = 4;
      isMoving = false;
      Arm.set_brake_mode(MOTOR_BRAKE_HOLD);
      Arm.move_velocity(0);
    } //--------------------------------------> Goes from scored to home
    else if (DunkPos == 5 && ArmSensor.get_position() <= 100 * returnHomePos) {
      Arm.move_velocity(0);
      DunkPos = 0;
      isMoving = false;
      Arm.set_brake_mode(MOTOR_BRAKE_HOLD);
      Arm.move_velocity(0);
    } 

//------------------------------------------------------Auton and Intake code-----------------------------------------------------------

    // Pressing both X and Up on the controller will run the selected auton (for testing/development)
    if (master.get_digital(DIGITAL_X) 
     && master.get_digital(DIGITAL_A)) {
      autonomous();
    }

    // Pressing L1/L2 will outake and R1/R2 will intake
    if/*----*/(master.get_digital(DIGITAL_L1) == true 
    || master.get_digital(DIGITAL_L2)         == true) {
      Intake.move_velocity(-600);
      IntakeFlex.move_velocity(-200);
    } else if (master.get_digital(DIGITAL_R1) == true 
    || master.get_digital(DIGITAL_R2)         == true) {
      Intake.move_velocity(600);
      IntakeFlex.move_velocity(200);
    } else {
      Intake.move_velocity(0);
      IntakeFlex.move_velocity(0);
    }

    pros::delay(ez::util::DELAY_TIME);  // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
  }
}