#pragma once

#ifndef LIMELIB_POSE2D_HPP
#define LIMELIB_POSE2D_HPP
#include "limelib/types.hpp"

namespace limelib
{
    /**
     * Struct representing a 2D pose (position and orientation)
     */
    struct Pose2D
    {
        real_t x;
        real_t y;
        real_t theta;
        /**
         * Constructor for Pose2D
         * @param x X coordinate of the pose
         * @param y Y coordinate of the pose
         * @param theta Orientation angle in radians
         */
        Pose2D(real_t x = 0, real_t y = 0, real_t theta = 0) : x(x), y(y), theta(theta) {}
        /**
         * Convert theta between degrees and radians
         * @return Pose2D with converted theta
         */
        Pose2D toDegrees() const
        {
            return Pose2D(x, y, theta * 180 / M_PI);
        }
        /**
         * Convert theta between degrees and radians
         * @return Pose2D with converted theta
         */
        Pose2D toRadians() const
        {
            return Pose2D(x, y, theta * M_PI / 180);
        }
    };
}

#endif // LIMELIB_POSE2D_HPP