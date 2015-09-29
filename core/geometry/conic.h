#ifndef CONIC_H
#define CONIC_H

#include "line.h"

namespace corecvs {

class Circle2d {
public:
    Vector2dd c;
    double r;

    bool intersectWith(const Circle2d &other, Vector2dd &point1, Vector2dd &point2);
};


class Circle3d
{
public:
    Vector3dd normal;
    Vector3dd center;

    double radius;

    Plane3d getPlane() {
        return Plane3d::FormNormalAndPoint(normal, center);
    }
};

class Sphere3d
{
public:
    Vector3dd c;
    double r;


    Sphere3d();

    Circle3d intersectWith(const Sphere3d &other);
    Circle3d intersectWith(const Plane3d &plane);

};

} // namespace corecvs

#endif // CONIC_H
