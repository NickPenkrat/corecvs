#ifndef CONVEXQUICKHULL_H
#define CONVEXQUICKHULL_H


#include <algorithm>
#include <array>
#include "queue"
#include "core/math/vector/vector3d.h"
#include "core/geometry/polygons.h"

using namespace corecvs;

class ConvexQuickHull
{
public:
    typedef vector<Vector3dd> vertices;

    struct HullFace {
        HullFace(const Vector3dd &p1, const Vector3dd &p2, const Vector3dd &p3) :
            plane(p1,p2,p3)
        {}

        Triangle3dd plane;
        vertices points;
    };

    typedef vector<HullFace> HullFaces;

    static HullFaces quickHull(const vertices& listVertices, double epsilon);

protected:

    static double pointFaceDist(const Triangle3dd &face, const Vector3dd &point);

    static bool faceIsVisible(const Vector3dd &eyePoint, const HullFace &face, double eps);

    static void addPointsToFaces(HullFace* faces, unsigned long faces_count, const vertices &listVertices, double eps);

};


#endif // CONVEXQUICKHULL_H
