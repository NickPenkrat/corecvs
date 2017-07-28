#ifndef CONVEXPOLYHEDRON_H
#define CONVEXPOLYHEDRON_H

#include <vector>
#include "line.h"
#include "axisAlignedBox.h"

namespace corecvs {

using std::vector;

template<typename HalfspaceType, typename VectorType>
class ConvexPolyhedronGeneric
{
public:
    vector<HalfspaceType> faces;

    ConvexPolyhedronGeneric() {}

    bool isInside(const VectorType &p)
    {
        for (HalfspaceType &type: faces )
        {
            if (type.pointWeight(p) < 0)
                return false;
        }
        return true;
    }

    unsigned size() const
    {
		return (unsigned)faces.size();
    }

    Vector3dd getPoint(int i) const
    {
        return faces[i].projectZeroTo();
    }

    Vector3dd getNormal(int i) const
    {
        return faces[i].normal();
    }


};


class ConvexPolyhedron : public ConvexPolyhedronGeneric<Plane3d, Vector3dd>
{
public:
    ConvexPolyhedron();
    ConvexPolyhedron(const AxisAlignedBox3d &box);

    bool intersectWith(const Ray3d &ray, double &t1, double &t2)
    {
        return ray.clip<ConvexPolyhedron> (*this, t1, t2);
    }
};

class ConvexPolygon : public ConvexPolyhedronGeneric< Line2d, Vector2dd>
{
public:

};


} // namespace corecvs


#endif // CONVEXPOLYHEDRON_H
