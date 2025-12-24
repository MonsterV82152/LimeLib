#pragma once

#ifndef LIMELIB_POINT2D_HPP
#define LIMELIB_POINT2D_HPP

#include "limelib/types.hpp"

namespace limelib
{
    /**
     * Struct representing a 2D point
     */
    struct Point2D
    {
        real_t x;
        real_t y;
        /**
         * Constructor for Point2D
         * @param x X coordinate of the point
         * @param y Y coordinate of the point
         */
        Point2D(real_t x = 0, real_t y = 0) : x(x), y(y) {}
    };
}

#endif // LIMELIB_POSE2D_HPP