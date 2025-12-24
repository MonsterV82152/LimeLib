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
        pros::Distance& sensor;
        Pose2D pose;
        real_t reading = 0;
        MCLDistance(pros::Distance& sensor, Pose2D pose) : sensor(sensor), pose(pose) {}
    };
}

#endif