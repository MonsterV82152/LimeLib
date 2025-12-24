#pragma once

#ifndef LIMELIB_FIELD2D_HPP
#define LIMELIB_FIELD2D_HPP
#include "limelib/geometry/object2d.hpp"
#include <memory>

namespace limelib
{
    /**
     * Class representing a 2D field with objects and edges
     */
    class Field2D
    {
    public:
        /**
         * Constructor for Field2D
         * @param width Width of the field
         * @param height Height of the field
         * @param objects Initial objects to add to the field
         */
        Field2D(real_t width, real_t height, const std::vector<std::shared_ptr<Object2D>> &objects = {});
        /**
         * Add an object to the field
         * @param object Shared pointer to the Object2D to add
         */
        void addObject(std::shared_ptr<Object2D> object);
        /**
         * Remove an object from the field
         * @param object Shared pointer to the Object2D to remove
         */
        void removeObject(const std::shared_ptr<Object2D> &object);
        /**
         * Get the width of the field
         * @return Width of the field
         */
        real_t getWidth() const;
        /**
         * Get the height of the field
         * @return Height of the field
         */
        real_t getHeight() const;
        /**
         * Get the objects in the field
         * @return Vector of shared pointers to Object2D
         */
        const std::vector<std::shared_ptr<Object2D>> &getObjects() const;
        /**
         * Get the edges of all objects in the field
         * @return Vector of LineSegment2D representing all edges
         */
        const std::vector<LineSegment2D> &getEdges() const;

    private:
        real_t width;
        real_t height;
        void calculateEdges();
        std::vector<std::shared_ptr<Object2D>> objects;
        mutable std::vector<LineSegment2D> edges;
    };

}
#endif