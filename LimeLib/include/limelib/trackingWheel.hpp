#pragma once

#ifndef TRACKINGWHEEL_HPP
#define TRACKINGWHEEL_HPP

#include "limelib/types.hpp"
#include "pros/rotation.hpp"
#include "pros/motor_group.hpp"

namespace limelib
{
    class TrackingWheel
    {
    public:
        // Allow nullptr construction with default values
        TrackingWheel(pros::Rotation *sensor, real_t diameter = 2, real_t offset = 0);
        TrackingWheel(pros::MotorGroup *left, pros::MotorGroup *right, real_t diameter = 3.25, real_t gearRatio = 1);
        
        // Return 0 if sensor is nullptr
        real_t getDistanceTravelled();
        void calibrate();
        real_t getOffset() const;
        real_t getDiameter() const;

    private:
        pros::Rotation *sensor;
        real_t diameter;
        real_t offset;
        real_t lastPos;
        real_t gearRatio;
        bool isDifferential;
        pros::MotorGroup *left;
        pros::MotorGroup *right;
    };
}

#endif