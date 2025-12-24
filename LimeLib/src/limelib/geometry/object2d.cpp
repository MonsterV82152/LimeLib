#include "limelib/geometry/object2d.hpp"

limelib::Point2D limelib::rotatePoint(Point2D point, Point2D origin, real_t angle)
{
    real_t s = sin(angle);
    real_t c = cos(angle);

    // Translate point back to origin
    point.x -= origin.x;
    point.y -= origin.y;

    // Rotate point
    real_t xnew = point.x * c - point.y * s;
    real_t ynew = point.x * s + point.y * c;

    // Translate point back
    point.x = xnew + origin.x;
    point.y = ynew + origin.y;

    return point;
}

// bool limelib::Object2D::isContacting(Pose2D pose) const
// {
//     return isContacting(Point2D(pose.x, pose.y));
// }

// bool limelib::Object2D::isContacting(real_t x, real_t y) const
// {
//     return isContacting(Point2D(x, y));
// }

// bool limelib::Object2D::isContacting(Point2D point) const
// {
//     // Default implementation - should be overridden by derived classes
//     return false;
// }

// const std::vector<limelib::LineSegment2D> &limelib::Object2D::getEdges() const
// {
//     // Default implementation - should be overridden by derived classes
//     static const std::vector<LineSegment2D> empty_edges;
//     std::cout << "Object2D getEdges called - returning empty edges" << std::endl;
//     return empty_edges;
// }

// Rectangle2D Implementation
limelib::Rectangle2D::Rectangle2D(Pose2D pose, real_t width, real_t height)
{
    Rectangle2D(pose.x, pose.y, width, height, pose.theta);
}

limelib::Rectangle2D::Rectangle2D(real_t x, real_t y, real_t width, real_t height, real_t theta)
    : width(width), height(height), pose(x, y, theta)
{
    std::vector<Point2D> corners = {
        Point2D(x, y),
        Point2D(cos(theta) * width + x, cos(theta + M_PI / 2) * width + y),
        Point2D(cos(theta) * width + x + cos(theta + M_PI) * height, sin(theta) * width + y), // Adjusted
        Point2D(cos(theta) * width / 2 + x, sin(theta) * width / 2 + y)};
    edges = {
        LineSegment2D(corners[0], corners[1]),
        LineSegment2D(corners[1], corners[2]),
        LineSegment2D(corners[2], corners[3]),
        LineSegment2D(corners[3], corners[0])};
}

const std::vector<limelib::LineSegment2D> &limelib::Rectangle2D::getEdges() const
{
    return edges;
}

bool limelib::Rectangle2D::isContacting(Point2D point) const
{
    // Translate point to rectangle's rotation origin
    point.x -= pose.x;
    point.y -= pose.y;

    // Rotate point by negative angle to "undo" the rectangle's rotation
    real_t s = sin(-pose.theta);
    real_t c = cos(-pose.theta);

    real_t rotated_x = point.x * c - point.y * s;
    real_t rotated_y = point.x * s + point.y * c;

    // Now check if the rotated point is within the axis-aligned rectangle
    return (rotated_x >= 0 && rotated_x <= width &&
            rotated_y >= 0 && rotated_y <= height);
}

// Circle2D Implementation
bool limelib::Circle2D::isContacting(Point2D point) const
{
    real_t dx = point.x - pose.x;
    real_t dy = point.y - pose.y;
    real_t distance_squared = dx * dx + dy * dy;
    return distance_squared <= radius * radius;
}

const std::vector<limelib::LineSegment2D> &limelib::Circle2D::getEdges() const
{
    // For a circle, we approximate with line segments
    // Use 32 segments for a smooth approximation
    if (edges.empty())
    {
        initializeEdges();
    }
    return edges;
}

void limelib::Circle2D::initializeEdges() const
{
    const int num_segments = 32;
    corners.reserve(num_segments);
    edges.reserve(num_segments);

    for (int i = 0; i < num_segments; ++i)
    {
        real_t angle = (2.0 * M_PI * i) / num_segments;
        Point2D corner(pose.x + radius * cos(angle), pose.y + radius * sin(angle));
        corners.push_back(corner);
    }

    for (int i = 0; i < num_segments; ++i)
    {
        int next_i = (i + 1) % num_segments;

        edges.push_back(LineSegment2D(corners[i], corners[next_i]));
    }
}

// Polygon2D Implementation
limelib::Polygon2D::Polygon2D(const std::vector<Point2D> &corners)
    : corners(corners)
{
    constructEdges();
}

void limelib::Polygon2D::constructEdges()
{
    edges.clear();
    if (corners.size() < 3)
        return; // Need at least 3 points for a polygon

    edges.reserve(corners.size());
    for (size_t i = 0; i < corners.size(); ++i)
    {
        size_t next_idx = (i + 1) % corners.size();
        edges.emplace_back(corners[i], corners[next_idx]);
    }
}

bool limelib::Polygon2D::isContacting(Point2D point) const
{
    return pointInPolygon(point);
}

bool limelib::Polygon2D::pointInPolygon(const Point2D &point) const
{
    if (corners.size() < 3)
        return false;

    // Ray casting algorithm
    bool inside = false;
    size_t j = corners.size() - 1;

    for (size_t i = 0; i < corners.size(); ++i)
    {
        const real_t xi = corners[i].x;
        const real_t yi = corners[i].y;
        const real_t xj = corners[j].x;
        const real_t yj = corners[j].y;

        // Check if ray from point crosses edge
        if (((yi > point.y) != (yj > point.y)) &&
            (point.x < (xj - xi) * (point.y - yi) / (yj - yi) + xi))
        {
            inside = !inside;
        }
        j = i;
    }

    return inside;
}

const std::vector<limelib::LineSegment2D> &limelib::Polygon2D::getEdges() const
{
    return edges;
}

// Line2D Implementation
limelib::Line2D::Line2D(Point2D start, Point2D end)
{
    edges.emplace_back(start, end);
}

limelib::Line2D::Line2D(real_t startX, real_t startY, real_t endX, real_t endY)
{
    edges.emplace_back(startX, startY, endX, endY);
}

bool limelib::Line2D::isContacting(Point2D point) const
{
    // Check if point is near the line segment (within small tolerance)
    if (edges.empty())
        return false;

    const auto &edge = edges[0];
    const real_t tolerance = 0.5f; // Half inch tolerance

    // Calculate distance from point to line segment
    real_t dx = edge.end.x - edge.start.x;
    real_t dy = edge.end.y - edge.start.y;
    real_t lengthSq = dx * dx + dy * dy;

    if (lengthSq < 1e-9f)
    {
        // Line is a point
        real_t distSq = (point.x - edge.start.x) * (point.x - edge.start.x) +
                        (point.y - edge.start.y) * (point.y - edge.start.y);
        return distSq <= tolerance * tolerance;
    }

    // Project point onto line
    real_t t = ((point.x - edge.start.x) * dx + (point.y - edge.start.y) * dy) / lengthSq;
    t = std::clamp(t, 0.0f, 1.0f);

    real_t nearestX = edge.start.x + t * dx;
    real_t nearestY = edge.start.y + t * dy;

    real_t distSq = (point.x - nearestX) * (point.x - nearestX) +
                    (point.y - nearestY) * (point.y - nearestY);

    return distSq <= tolerance * tolerance;
}

const std::vector<limelib::LineSegment2D> &limelib::Line2D::getEdges() const
{
    return edges;
}

// ObjectGroup2D Implementation
limelib::ObjectGroup2D::ObjectGroup2D(std::vector<std::shared_ptr<Object2D>> objects)
    : objects(std::move(objects))
{
    constructEdges();
}

void limelib::ObjectGroup2D::addObject(std::shared_ptr<Object2D> object)
{
    if (object)
    {
        objects.push_back(std::move(object));
        constructEdges();
    }
}

void limelib::ObjectGroup2D::removeObject(const std::shared_ptr<Object2D> &object)
{
    auto it = std::find(objects.begin(), objects.end(), object);
    if (it != objects.end())
    {
        objects.erase(it);
        constructEdges();
    }
}

void limelib::ObjectGroup2D::constructEdges() const
{
    edges.clear();

    for (const auto &obj : objects)
    {
        if (obj)
        {
            const auto &obj_edges = obj->getEdges();
            edges.insert(edges.end(), obj_edges.begin(), obj_edges.end());
        }
    }
}

bool limelib::ObjectGroup2D::isContacting(Point2D point) const
{
    for (const auto &obj : objects)
    {
        if (obj && obj->isContacting(point))
        {
            return true;
        }
    }
    return false;
}

const std::vector<limelib::LineSegment2D> &limelib::ObjectGroup2D::getEdges() const
{
    return edges;
}