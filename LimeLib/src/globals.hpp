#pragma once

#ifndef GLOBALS_HPP
#define GLOBALS_HPP

#include <string>
#include <stdlib.h>
#include <iostream>
#include <sstream>
#include <string>
#include <cmath>
#include <deque>
#include <atomic>
#include <memory>
#include "main.h"

using namespace limelib;

/*----------------------Defines----------------------*/

// Constants
constexpr double PI = 3.141592;

/*---Controller---*/
inline pros::Controller master(pros::E_CONTROLLER_MASTER);
inline pros::Controller slave(pros::E_CONTROLLER_PARTNER);

/*------------------Global Variables------------------*/

namespace buttons
{
    constexpr auto R1 = pros::E_CONTROLLER_DIGITAL_R1;
    constexpr auto R2 = pros::E_CONTROLLER_DIGITAL_R2;
    constexpr auto L1 = pros::E_CONTROLLER_DIGITAL_L1;
    constexpr auto L2 = pros::E_CONTROLLER_DIGITAL_L2;
    constexpr auto A = pros::E_CONTROLLER_DIGITAL_A;
    constexpr auto B = pros::E_CONTROLLER_DIGITAL_B;
    constexpr auto Y = pros::E_CONTROLLER_DIGITAL_Y;
    constexpr auto X = pros::E_CONTROLLER_DIGITAL_X;
    constexpr auto UP = pros::E_CONTROLLER_DIGITAL_UP;
    constexpr auto DOWN = pros::E_CONTROLLER_DIGITAL_DOWN;
    constexpr auto LEFT = pros::E_CONTROLLER_DIGITAL_LEFT;
    constexpr auto RIGHT = pros::E_CONTROLLER_DIGITAL_RIGHT;
}

const pros::motor_brake_mode_e_t brake = pros::E_MOTOR_BRAKE_BRAKE;
const pros::motor_brake_mode_e_t coast = pros::E_MOTOR_BRAKE_COAST;
const pros::motor_brake_mode_e_t hold = pros::E_MOTOR_BRAKE_HOLD;

/*-------------Define all configurations-------------*/
inline pros::MotorGroup leftDT({-1, -2, -3}); // Change these ports to match your left drivetrain motors
inline pros::MotorGroup rightDT({8, 9, 10});  // Change these ports to match your right drivetrain motors
inline pros::Rotation vertical(15);
inline pros::Imu inertial(13);

namespace localization
{
    // Distance Sensors - Change to match your distance sensor ports
    inline pros::Distance rightDS(20);
    inline pros::Distance leftDS(5);
    inline pros::Distance frontDS(21);
    inline pros::Distance backDS(4);
}
inline pros::Rotation vertical(14);

inline std::vector<MCLDistance> mclSensors = {
    // Distance Sensors - adjust poses to match your robot's configuration
    // Pose2D(x, y, theta in degrees)
    // x = forward/backward offset from robot center (positive is forward)
    // y = left/right offset from robot center (positive is right)
    // theta = orientation of the sensor relative to the robot (clockwise in degrees)
    {localization::rightDS, Pose2D(4.25, -2.25, 90)},
    {localization::leftDS, Pose2D(-4.25, -2.25, 270)},
    {localization::frontDS, Pose2D(4.5, 3, 0)},
    {localization::backDS, Pose2D(-3.5, -5.5, 180)}};

// Tracking Wheels - adjust diameter and offset to match your robot's configuration
inline TrackingWheel verticalTW(&vertical, 2.75, -0.25);

inline std::vector<std::shared_ptr<Object2D>> obstacles = {
    // No obstacles for Push Back
    };

inline Field2D field(144.0f, 144.0f, obstacles); // Field dimensions for VEX field (inches)

inline MCL locator( // MCL localization
    &verticalTW, // Vertical Tracking wheel
    nullptr, // No horizontal tracking wheel
    inertial, // IMU sensor
    mclSensors, // Distance sensors
    field, // Field2D object
    200, // Number of particles
    0.1, // Rotation noise
    0.1, // Translation noise
    false, // Debug disabled
    5 // Frequency of updates (lower is more frequent)
);
// inline Odometry locator(&verticalTW, nullptr, inertial); // Alternative: Odometry localization

inline PID linearPID(3, 0.0, 2);
inline PID angularPID(0.9, 0.0, 1);
inline Chassis chassis(locator, leftDT, rightDT, linearPID, angularPID);
#endif