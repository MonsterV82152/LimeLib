#pragma once

#ifndef MCLDISTANCE_HPP
#define MCLDISTANCE_HPP

#include "limelib/types.hpp"
#include "limelib/geometry/pose2d.hpp"
#include "pros/distance.hpp"

namespace limelib
{
    struct MCLDistance
    {
        pros::Distance &sensor;
        Pose2D pose;
        real_t reading = 0;
        MCLDistance(pros::Distance &sensor, Pose2D pose) : sensor(sensor), pose(pose) {}
    };
    struct ValidSensor
    {
        real_t reading; // in millimeters
        real_t pose_x;
        real_t pose_y;
        real_t pose_theta_rad; // Pre-converted to radians

        ValidSensor(real_t r, real_t x, real_t y, real_t theta)
            : reading(r), pose_x(x), pose_y(y), pose_theta_rad(theta) {}
    };
}

#endif