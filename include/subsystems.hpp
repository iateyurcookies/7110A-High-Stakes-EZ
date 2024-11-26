#pragma once

#include "api.h"
#include "pros/rotation.hpp"

//Drive Motors, all 600 rpm blue carts
inline::pros::Motor FrontL(1, pros::MotorGearset::blue);
inline::pros::Motor MidL(2, pros::MotorGearset::blue);
inline::pros::Motor BackL(3, pros::MotorGearset::blue);
inline::pros::Motor FrontR(-4, pros::MotorGearset::blue);
inline::pros::Motor MidR(-5, pros::MotorGearset::blue);
inline::pros::Motor BackR(-6, pros::MotorGearset::blue);

//Arm Motors, 5.5 watt half motors
inline::pros::Motor ArmRight(17, pros::MotorGearset::green);
inline::pros::Motor ArmLeft(-8, pros::MotorGearset::green);

// Motor group for Arm
inline::pros::MotorGroup Arm({17, -8},// Arm motors on ports 7 & 8
pros::MotorGearset::green);          // both motors are green(5.5W is volted down)

// Intake Motor, 600 rpm blue cart
inline::pros::Motor Intake(20, pros::MotorGearset::blue);

// Clamp & Doinker Piston, Clamp : port H | Arm : port G
inline::pros::adi::DigitalOut clamp_piston(8);
inline::pros::adi::DigitalOut doinker_piston(7);

// Optical sensor on port 10
inline::pros::Optical optical(10);

// Rotation sensors for arm and intake
inline::pros::Rotation ArmSensor(-9);
inline::pros::Rotation IntakeSensor(99);

// inline inline::pros::Motor intake(1);
// inline pros::adi::DigitalIn limit_switch('A');