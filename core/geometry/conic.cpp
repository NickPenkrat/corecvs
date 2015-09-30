#include "conic.h"

namespace corecvs {

Sphere3d::Sphere3d()
{
}

Circle3d Sphere3d::intersectWith(const Sphere3d &other)
{
    return Circle3d();
}

Circle3d Sphere3d::intersectWith(const Plane3d &plane)
{
    return Circle3d();
}

bool Circle2d::intersectWith(const Circle2d &other, Vector2dd &point1, Vector2dd &point2)
{
    double d = (other.c - c).l2Metric();
    double r1 = other.r;

    double x = (d * d - r1 * r1 + r * r) / (2 * d);

    return false;
}

} // namespace corecvs
