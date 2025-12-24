#include "limelib/geometry/field2d.hpp"

limelib::Field2D::Field2D(real_t width, real_t height, const std::vector<std::shared_ptr<Object2D>> &objects)
    : width(width), height(height), objects(objects)
{
    calculateEdges();
}

void limelib::Field2D::addObject(std::shared_ptr<Object2D> object)
{
    if (object)
    {
        objects.push_back(std::move(object));
        calculateEdges();
    }
}

void limelib::Field2D::removeObject(const std::shared_ptr<Object2D> &object)
{
    auto it = std::find(objects.begin(), objects.end(), object);
    if (it != objects.end())
    {
        objects.erase(it);
        calculateEdges();
    }
}

limelib::real_t limelib::Field2D::getWidth() const
{
    return width;
}

limelib::real_t limelib::Field2D::getHeight() const
{
    return height;
}

const std::vector<std::shared_ptr<limelib::Object2D>> &limelib::Field2D::getObjects() const
{
    return objects;
}

const std::vector<limelib::LineSegment2D> &limelib::Field2D::getEdges() const
{
    return edges;
}

void limelib::Field2D::calculateEdges()
{
    edges.clear();
    for (const auto &object : objects)
    {
        if (object)
        {
            const auto &objectEdges = object->getEdges();
            edges.insert(edges.end(), objectEdges.begin(), objectEdges.end());
        }
    }

    // Add field boundary edges
    edges.emplace_back(-width / 2, -height / 2, width / 2, -height / 2); // Bottom edge
    edges.emplace_back(width / 2, -height / 2, width / 2, height / 2);   // Right edge
    edges.emplace_back(width / 2, height / 2, -width / 2, height / 2);   // Top edge
    edges.emplace_back(-width / 2, height / 2, -width / 2, -height / 2); // Left edge
}
