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
#include "pros/screen.hpp"
#include "subsystems.hpp"

// Initialize important variables
static bool clampPiston{false}; //---------------> toggle for mogo clamp
static bool doinkPiston{false}; //---------------> toggle for doinker
static bool intakePiston{false};//---------------> toggle for intake piston
static bool doinkClamp{true};   //---------------> toggle for doinker clamp

static bool team{true};         //---------------> true = red    false = blue

static int DunkPos = 0;         //---------------> arm macro integer
static bool isMoving = false;   //---------------> is arm active
static float grabRingPos = 14.5;    //---------------> position values for the macro
static int almostScoreRingPos = 90;
static int scoreRingPos = 140;
static int returnHomePos = 6;

// Load image from sd
rd::Image logo("/usd/robotics/logo.bin", "Logo");
rd::Image dhyan("/usd/robotics/dhyan.bin", "Dhyan");

//Initialize console
rd::Console console;

//Initialize auton selector
rd::Selector selector({
    {"Blue4-", sixRingBlue},
    {"Red4-", sixRingRed},
    {"BRush+", BlueLeftRush},
    {"RRush+", RedRightRush},
    {"BSWP-", BlueRightAWP},
    {"RSWP-", RedLeftAWP},
    {"Prog", prog},
    {"Testing", test},
});

// Chassis constructor
ez::Drive chassis(
    {-10, -9, -8},  // Left Chassis Ports (negative port will reverse it!)
    {7, 6, 5},     // Right Chassis Ports (negative port will reverse it!)

    1, //-------------------------------> IMU Port
    3.25, //----------------------> Wheel Diameter (Remember, 4" wheels without screw holes are actually 4.125!)
    450 //---------------------------------> Wheel RPM
);

void initialize() {
  pros::delay(500); //-----------------------> Stop the user from doing anything while legacy ports configure
  DunkPos = 0; //------------------------------------------> Arm stuff
  ArmSensor.reset();
  ArmSensor.set_position(0);
  intake_piston.set_value(false); //-----------------------> Sets intake piston to false so it starts down

  // Configure your chassis controls
  chassis.opcontrol_curve_buttons_toggle(true);  // Enables modifying the controller curve with buttons on the joysticks
  chassis.opcontrol_drive_activebrake_set(0);        // Sets the active brake kP. 0 will disable.
  chassis.opcontrol_curve_default_set(0, 0);// Defaults for curve. If using tank, only the first parameter is used.

  default_constants();

  chassis.initialize(); //---------------------------------> Initializes chassis and auton selector
  master.rumble(".");
  pros::screen::erase();
  pros::delay(500);
  selector.focus();
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

  selector.run_auton();
  console.focus();

  pros::Task console_display([&]() { //-------------------------> printing important shit to the brain
    while (true) {
      console.printf("BATTERY: %.0f % \n\n", pros::battery::get_capacity());
      //drive temps
      console.println("MOTOR_TEMPS:"); //-----------------------------------> Prints the motor temperatures for each motor
      console.printf("L1 %.0fC  ", FrontL.get_temperature()); 
      console.printf("R1 %.0fC \n", FrontR.get_temperature());
      console.printf("L2 %.0fC  ", MidL.get_temperature());
      console.printf("R2 %.0fC \n", MidR.get_temperature());
      console.printf("L3 %.0fC  ", BackL.get_temperature());
      console.printf("R3 %.0fC \n\n", BackR.get_temperature());
      //intake & arm
      console.printf("INTAKE %.0fC \n", Intake.get_temperature());
      console.printf("ARM %.0fC", Arm.get_temperature());
      pros::delay(2000);
      console.clear(); //------------------------------------------------------> Refreshes screen after delay to save resources
    }
  });
}

void opcontrol() {
  //Team branding
  logo.focus();

  // Makes it so the SWP autons don't mess up macro
  if (selector.get_auton()->name == "BSWP-" 
  ||  selector.get_auton()->name == "RSWP-"){
    DunkPos = 5;
    Arm.move_velocity(-200);
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

  pros::Task controller_screen([&]() {  //-------> Prints important shit to the controller screen
    while (true) { 
      // Initializes the strings for the info
      std::string teamStr = ""; 
      std::string autoStr = "";

      // Gets the auton name
      if(selector.get_auton().has_value() == false){
        autoStr = "NOTHING  ";
      }else{
        autoStr = selector.get_auton()->name;
      }

      //Prints what color alliance you are
      if (team)
        teamStr = "R";
      else
        teamStr = "B";

      // Combines all the strings and prints it as one to the controller screen
      std::string controllerString = "T: " + teamStr + " " + autoStr + "      ";
      master.print(0, 1, "%s", controllerString);

      // delay so the brain doesn't get fried
      pros::delay(100);
    }
  });

  while (true) {// Driver control while loop

//--------------------------------------------------------------Setup-------------------------------------------------------------------

    // Standard split arcade(left stick = forward-back | right stick = turning)
    chassis.opcontrol_arcade_standard(ez::SPLIT);

    // Left button cycles back though the autons and right button cycles forward
    if (master.get_digital_new_press(DIGITAL_UP)) {
    selector.prev_auton(true); 
    } if (master.get_digital_new_press(DIGITAL_LEFT)) {
    selector.next_auton(true); 
    }
    
    // Pressing all R and L triggers on the controller will run the selected auton (for testing/development)
    if (master.get_digital_new_press(DIGITAL_L1) == true && master.get_digital_new_press(DIGITAL_R1) == true) {
      autonomous();
    }

    // Pressing the Up and Left buttons will change the bots color sorting to the opposite color for color-sorting
    if (master.get_digital_new_press(DIGITAL_UP) && master.get_digital_new_press(DIGITAL_LEFT)) {
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

    // Pressing Y will acuate THE DOINKER (is toggle)
    if (master.get_digital_new_press(DIGITAL_Y)) {
      if (!doinkPiston) {
        doinker_piston.set_value(true);
        doinkPiston = !doinkPiston;
      } else {
        doinker_piston.set_value(false);
        doinkPiston = !doinkPiston;
      }
    }

    // Pressing X will acuate THE DOINKER CLAMP (is toggle)
    if (master.get_digital_new_press(DIGITAL_X)) {
      if (!doinkClamp) {
        doinker_clamp.set_value(true);
        doinkClamp = !doinkClamp;
      } else {
        doinker_clamp.set_value(false);
        doinkClamp = !doinkClamp;
      }
    }

    // Pressing A will acuate the piston intake (is toggle)
    if (master.get_digital_new_press(DIGITAL_A)) {
      if (!intakePiston) {
        intake_piston.set_value(true);
        intakePiston = !intakePiston;
      } else {
        intake_piston.set_value(false);
        intakePiston = !intakePiston;
      }
    }
    
//-------------------------------------Lady Brown macro code (make sure it starts at the hardstop)--------------------------------------

    if(master.get_digital_new_press(DIGITAL_RIGHT))
    { //--------------------------------------> Returns to home regardless of position
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
    if ((DunkPos == 1 && (ArmSensor.get_position() >= 100 * grabRingPos))
    ||   DunkPos == 7 && (ArmSensor.get_position() <= 100 * grabRingPos)) {
      DunkPos = 2;
      isMoving = false;
      Arm.set_brake_mode(MOTOR_BRAKE_HOLD);
      Arm.move_velocity(0);
    }  //--------------------------------------> Goes from lining up to scored
    else if (DunkPos == 3 && (ArmSensor.get_position() >= 100 * scoreRingPos)) {
      Arm.move_velocity(0);
      DunkPos = 4;
      isMoving = false;
      Arm.set_brake_mode(MOTOR_BRAKE_HOLD);
      Arm.move_velocity(0);
    }  //--------------------------------------> Goes from scored to home
    else if (DunkPos == 5 && ArmSensor.get_position() <= 100 * returnHomePos) {
      Arm.move_velocity(0);
      DunkPos = 0;
      isMoving = false;
      Arm.set_brake_mode(MOTOR_BRAKE_HOLD);
      Arm.move_velocity(0);
    }

//---------------------------------------------------------Intake code------------------------------------------------------------------

    // Pressing L1/L2 will outake and R1/R2 will intake
    if/*----*/(master.get_digital(DIGITAL_L1) == true || master.get_digital(DIGITAL_L2) == true) {
      Intake.move_velocity(-600);
      IntakeFlex.move_velocity(-200);
    } else if (master.get_digital(DIGITAL_R1) == true || master.get_digital(DIGITAL_R2) == true) {
      Intake.move_velocity(600);
      IntakeFlex.move_velocity(200);
    } else {
      Intake.move_velocity(0);
      IntakeFlex.move_velocity(0);
    }

    pros::delay(ez::util::DELAY_TIME);  // This is used for timer calculations
  }
}