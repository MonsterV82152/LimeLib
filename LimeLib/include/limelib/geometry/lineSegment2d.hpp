#pragma once

#ifndef LIMELIB_LINESEGMENT2D_HPP
#define LIMELIB_LINESEGMENT2D_HPP

#include "limelib/geometry/pose2d.hpp"
#include "limelib/geometry/point2d.hpp"
#include "limelib/types.hpp"

namespace limelib
{
    /**
     * Struct representing a 2D line segment
     */
    struct LineSegment2D
    {
        /**
         * Start and end points of the line segment
         */
        Point2D start, end;
        /**
         * Constructor for LineSegment2D
         * @param start Starting point of the line segment
         * @param end Ending point of the line segment
         */
        LineSegment2D(Point2D start, Point2D end) : start(start), end(end) {}
        /**
         * Constructor for LineSegment2D using coordinates
         * @param startX X coordinate of the starting point
         * @param startY Y coordinate of the starting point
         * @param endX X coordinate of the ending point
         * @param endY Y coordinate of the ending point
         */
        LineSegment2D(real_t startX, real_t startY, real_t endX, real_t endY)
            : start(startX, startY), end(endX, endY) {}
    };

}

#endif