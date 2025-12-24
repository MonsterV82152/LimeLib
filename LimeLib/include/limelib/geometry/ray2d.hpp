#pragma once

#ifndef LIMELIB_RAY2D_HPP
#define LIMELIB_RAY2D_HPP

#include "limelib/geometry/point2d.hpp"
namespace limelib
{
    /**
     * Struct representing a 2D ray
     */
    struct Ray2D
    {
        Point2D start;
        real_t radians;
        /**
         * Constructor for Ray2D
         * @param start Starting point of the ray
         * @param radians Angle of the ray in radians
         */
        Ray2D(Point2D start = Point2D(), real_t radians = 0) : start(start), radians(radians) {}
    };

}

#endif