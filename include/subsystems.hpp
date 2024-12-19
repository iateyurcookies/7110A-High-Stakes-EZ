#pragma once

#include "api.h"
#include "pros/rotation.hpp"

//Drive Motors, all 600 rpm blue carts
inline::pros::Motor FrontL(10, pros::MotorGearset::blue);
inline::pros::Motor MidL(9, pros::MotorGearset::blue);
inline::pros::Motor BackL(8, pros::MotorGearset::blue);
inline::pros::Motor FrontR(-7, pros::MotorGearset::blue);
inline::pros::Motor MidR(-6, pros::MotorGearset::blue);
inline::pros::Motor BackR(-5, pros::MotorGearset::blue);

//Arm Motors, 5.5 watt half motors
inline::pros::Motor Arm(-2, pros::MotorGearset::green);

// Intake Motor, 600 rpm blue cart
inline::pros::Motor Intake(4, pros::MotorGearset::blue);
inline::pros::Motor IntakeFlex(3, pros::MotorGearset::green);

// Clamp & Doinker Piston, Clamp : port A | Doinker : port B
inline::pros::adi::DigitalOut clamp_piston(1);
inline::pros::adi::DigitalOut doinker_piston(2);

// Optical sensor on port 10
inline::pros::Optical optical(20);

// Rotation sensors for arm and intake
inline::pros::Rotation ArmSensor(19);
inline::pros::Rotation IntakeSensor(99);

// inline inline::pros::Motor intake(1);
// inline pros::adi::DigitalIn limit_switch('A');