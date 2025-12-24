#pragma once

#ifndef LIMELIB_OBJECT2D_HPP
#define LIMELIB_OBJECT2D_HPP

#include "limelib/geometry/pose2d.hpp"
#include "limelib/geometry/point2d.hpp"
#include "limelib/geometry/lineSegment2d.hpp"
#include "limelib/types.hpp"
#include <fstream>

namespace limelib
{
    /**
     * Rotate a point around an origin by a given angle
     * @param point The point to rotate
     * @param origin The origin to rotate around
     * @param angle The angle in radians to rotate
     * @return The rotated point
     */
    Point2D rotatePoint(Point2D point, Point2D origin, real_t angle);
    class Object2D
    {
    public:
        Object2D() = default;
        Object2D(const Object2D &) = delete; // Prevent copying
        Object2D &operator=(const Object2D &) = delete;
        Object2D(Object2D &&) = default; // Allow moving
        Object2D &operator=(Object2D &&) = default;
        virtual ~Object2D() = default;

        // bool isContacting(Pose2D pose) const;
        // bool isContacting(real_t x, real_t y) const;
        virtual bool isContacting(Point2D point) const = 0;
        virtual const std::vector<LineSegment2D> &getEdges() const = 0;
        // virtual const std::vector<Point2D> &getCorners() const;
    };
    /**
     * Class representing a 2D rectangle object
     */
    class Rectangle2D : public Object2D
    {
    public:
        /**
         * Constructor for Rectangle2D
         * @param x X coordinate of the rectangle center (bottom-left corner)
         * @param y Y coordinate of the rectangle center (bottom-left corner)
         * @param width Width of the rectangle
         * @param height Height of the rectangle
         * @param theta Rotation angle of the rectangle in radians
         */
        Rectangle2D(real_t x, real_t y, real_t width, real_t height, real_t theta = 0);
        /**
         * Constructor for Rectangle2D using Pose2D
         * @param pose Pose2D representing the rectangle's position and orientation
         * @param width Width of the rectangle
         * @param height Height of the rectangle
         */
        Rectangle2D(Pose2D, real_t width, real_t height);
        /**
         * Destructor for Rectangle2D
         * @note Copying is disabled to prevent slicing issues
         */
        Rectangle2D(const Rectangle2D &) = delete;
        /**
         * Assignment operator for Rectangle2D
         * @note Copying is disabled to prevent slicing issues
         */
        Rectangle2D &operator=(const Rectangle2D &) = delete;
        Rectangle2D(Rectangle2D &&) = default;
        Rectangle2D &operator=(Rectangle2D &&) = default;
        /**
         * Check if a point is contacting the rectangle
         * @param point Point2D to check for contact
         */
        bool isContacting(Point2D point) const override;
        /**
         * Get the edges of the rectangle
         * @return Vector of LineSegment2D representing the rectangle's edges
         */
        const std::vector<LineSegment2D> &getEdges() const override;
        // const std::vector<Point2D> &getCorners() const override { return corners; }

    private:
        real_t width;
        real_t height;
        std::vector<LineSegment2D> edges;
        std::vector<Point2D> corners;
        Pose2D pose;
    };
    /**
     * Class representing a 2D circle object
     */
    class Circle2D : public Object2D
    {
    public:
        /**
         * Constructor for Circle2D
         * @param x X coordinate of the circle center
         * @param y Y coordinate of the circle center
         * @param radius Radius of the circle
         */
        Circle2D(real_t x, real_t y, real_t radius)
            : pose(x, y), radius(radius) {};
        Circle2D(const Circle2D &) = delete;
        Circle2D &operator=(const Circle2D &) = delete;
        Circle2D(Circle2D &&) = default;
        Circle2D &operator=(Circle2D &&) = default;
        /**
         * Check if a point is contacting the circle
         * @param pose Point2D to check for contact
         */
        bool isContacting(Point2D pose) const override;
        /**
         * Get the edges of the circle (approximated as a polygon)
         * @return Vector of LineSegment2D representing the circle's edges
         */
        const std::vector<LineSegment2D> &getEdges() const override;
        // const std::vector<Point2D> &getCorners() const override { return corners; }

    private:
        real_t radius;
        mutable std::vector<LineSegment2D> edges;
        mutable std::vector<Point2D> corners;
        Point2D pose;
        void initializeEdges() const;
    };

    class Polygon2D : public Object2D
    {
    public:
        /**
         * Constructor for Polygon2D
         * @param corners Vector of Point2D representing the polygon's corners
         */
        Polygon2D(const std::vector<Point2D> &corners);
        Polygon2D(const Polygon2D &) = delete;
        Polygon2D &operator=(const Polygon2D &) = delete;
        Polygon2D(Polygon2D &&) = default;
        Polygon2D &operator=(Polygon2D &&) = default;
        /**
         * Check if a point is contacting the polygon
         * @param point Point2D to check for contact
         */
        bool isContacting(Point2D point) const override;
        /**
         * Get the edges of the polygon
         * @return Vector of LineSegment2D representing the polygon's edges
         */
        const std::vector<LineSegment2D> &getEdges() const override;
        // const std::vector<Point2D> &getCorners() const override { return corners; }

    private:
        std::vector<Point2D> corners;
        std::vector<LineSegment2D> edges;
        void constructEdges();
        bool pointInPolygon(const Point2D &point) const;
    };

    /**
     * Class representing a 2D line obstacle (a single edge)
     */
    class Line2D : public Object2D
    {
    public:
        /**
         * Constructor for Line2D
         * @param start Starting point of the line
         * @param end Ending point of the line
         */
        Line2D(Point2D start, Point2D end);
        /**
         * Constructor for Line2D using coordinates
         * @param startX X coordinate of the starting point
         * @param startY Y coordinate of the starting point
         * @param endX X coordinate of the ending point
         * @param endY Y coordinate of the ending point
         */
        Line2D(real_t startX, real_t startY, real_t endX, real_t endY);
        Line2D(const Line2D &) = delete;
        Line2D &operator=(const Line2D &) = delete;
        Line2D(Line2D &&) = default;
        Line2D &operator=(Line2D &&) = default;
        /**
         * Check if a point is contacting the line (within tolerance)
         * @param point Point2D to check for contact
         */
        bool isContacting(Point2D point) const override;
        /**
         * Get the edge (the line itself)
         * @return Vector containing the single LineSegment2D
         */
        const std::vector<LineSegment2D> &getEdges() const override;

    private:
        std::vector<LineSegment2D> edges;
    };

    /**
     * Equality operator for shared pointers to Object2D
     * @param a First shared pointer to Object2D
     * @param b Second shared pointer to Object2D
     */
    bool operator==(const std::shared_ptr<Object2D> &a, const std::shared_ptr<Object2D> &b);
    /**
     * Class representing a group of 2D objects
     */
    class ObjectGroup2D final : public Object2D
    {
    public:
        ObjectGroup2D() = default;
        /**
         * Constructor for ObjectGroup2D
         * @param objects Vector of shared pointers to Object2D
         */
        explicit ObjectGroup2D(std::vector<std::shared_ptr<Object2D>> objects);
        ~ObjectGroup2D() override = default;

        ObjectGroup2D(const ObjectGroup2D &) = delete;
        ObjectGroup2D &operator=(const ObjectGroup2D &) = delete;
        ObjectGroup2D(ObjectGroup2D &&) = default;
        ObjectGroup2D &operator=(ObjectGroup2D &&) = default;
        /**
         * Add an object to the group
         * @param object Shared pointer to the Object2D to add
         */
        void addObject(std::shared_ptr<Object2D> object);
        /**
         * Remove an object from the group
         * @param object Shared pointer to the Object2D to remove
         */
        void removeObject(const std::shared_ptr<Object2D> &object);
        /**
         * Check if a point is contacting any object in the group
         * @param point Point2D to check for contact
         */
        bool isContacting(Point2D point) const override;
        /**
         * Get the edges of all objects in the group
         * @return Vector of LineSegment2D representing all edges
         */
        const std::vector<LineSegment2D> &getEdges() const override;

    private:
        std::vector<std::shared_ptr<Object2D>> objects;
        mutable std::vector<LineSegment2D> edges;
        void constructEdges() const;
    };
}

#endif