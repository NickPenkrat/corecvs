/**
 * \file polygons.cpp
 * \brief Add Comment Here
 *
 * \ingroup cppcorefiles
 * \date Nov 14, 2010
 * \author alexander
 */

#include "polygons.h"
#include "mathUtils.h"
#include "global.h"

namespace corecvs {

/**
 *
 * Check if point is inside the polygon
 **/
int Polygon::isInside(const Vector2dd &a) const
{
    double oldsign = 0;
    int len = (int)size();
    for (int i = 0; i < len; i++)
    {
        const Vector2dd &curr = operator [](i);
        const Vector2dd &next = operator []((i + 1) % len);
        const Vector2dd normal = (next - curr).rightNormal();
        Vector2dd diff = a - curr;
        double sign = diff & normal;
        if (oldsign > 0  && sign < 0)
            return false;
        if (oldsign < 0  && sign > 0)
            return false;
        oldsign = sign;
    }
    return true;
}

bool Polygon::isConvex(bool *direction) const
{
    double oldsign = 0;
    int len = (int)size();
    for (int i = 0; i < len; i++)
    {
        const Vector2dd &curr = operator [](i);
        const Vector2dd &next = operator []((i + 1) % len);
        const Vector2dd &nnext = operator []((i + 2) % len);

        double sign = (next - curr).rightNormal() & (nnext - next);

        if (oldsign > 0  && sign < 0)
            return false;
        if (oldsign < 0  && sign > 0)
            return false;
        oldsign = sign;
    }
    if (direction != NULL) {
        *direction =  (oldsign > 0);
    }
    return true;
}

Polygon Polygon::RegularPolygon(int sides, const Vector2dd &center, double radius) {
    Polygon toReturn;
    toReturn.reserve(sides);

    double step = degToRad(360.0 / sides);

    for (int i = 0; i < sides; i++)
    {
        toReturn.push_back(center + Vector2dd::FromPolar(step * i, radius));
    }
    return toReturn;
}

Polygon Polygon::Reverse(const Polygon &p)
{
    Polygon toReturn;
    toReturn.reserve(p.size());

    for (auto ri = p.rbegin(); ri != p.rend(); ri++)
    {
        toReturn.push_back(*ri);
    }
    return toReturn;
}

double Polygon::signedArea()
{
    Vector2dd c = center();
    double sum = 0.0;
    int len = (int)size();

    for (int i = 0; i < len; i++)
    {
        const Vector2dd &curr = operator [](i);
        const Vector2dd &next = operator []((i + 1) % len);

        Vector2dd v1 = curr - c;
        Vector2dd v2 = next - c;

        double part = v1.triangleOrientedAreaTo(v2);
        sum += part;
    }
    return sum;
}

void PolygonCombiner::prepare()
{
    c1.clear();
    c2.clear();

    /*for (Vector2dd p : pol1) {
        inside1.push_back((pol2.isInside(p));
    }

    for (Vector2dd p : pol2) {
        inside2.push_back((pol1.isInside(p));
    }*/



}

Vector2dd PointPath::center()
{
    Vector2dd mean(0.0);
    for (Vector2dd point : *this)
    {
        mean += point;
    }
    if (!this->empty())
    {
        mean /= this->size();
    }
    return mean;
}

Polygon ConvexHull::GiftWrap(const std::vector<Vector2dd> &list)
{
    Polygon toReturn;
    if (list.empty())
    {
        return toReturn;
    }

    if (list.size() == 1)
    {
        toReturn.push_back(list[0]);
        return toReturn;
    }

    /* Find one point in hull */
    int minYId = 0;
    double minY = list[0].y();
    for (size_t i = 1; i < list.size(); i++)
    {
        if (list[i].y() < minY)
        {
            minY = list[i].y();
            minYId = i;
        }
    }
    toReturn.push_back(list[minYId]);
    Vector2dd direction = Vector2dd::OrtX();
    Vector2dd current = list[minYId];
    /* Wrap */
    do {
        Vector2dd next;
        Vector2dd nextDir;
        double vmin = std::numeric_limits<double>::max();

        for (const Vector2dd &point : list)
        {
            if (point == current)
                continue;

            Vector2dd dir1 = (point - current).normalised();
            double v = direction.azimuthTo(dir1);
            if (v < vmin) {
                vmin = v;
                next = point;
                nextDir = dir1;
            }
        }

        /* That is exact same point. double equality is safe */
        if (next == toReturn.front())
        {
            break;
        }

        toReturn.push_back(next);
        current = next;
        direction = nextDir;

    } while (true);


    return toReturn;
}


#if 0
bool Polygon::clipRay(const Ray2d &ray, double &t1, double &t2)
{
    t1 = -numeric_limits<double>::max();
    t2 =  numeric_limits<double>::max();


    const Vector2dd &a = ray.a;
    const Vector2dd &p = ray.p;

    for (unsigned i = 0; i < size();  i++) {
        int j = (i + 1) % size();
        Vector2dd &r1 = operator [](i);
        Vector2dd &r2 = operator [](j);

        Vector2dd n = (r2 - r1).rightNormal();
        Vector2dd diff = r1 - p;

        double numen = diff & n;
        double denumen = a & n;
        double t = numen / denumen;

        if ((denumen > 0) && (t > t1)) {
            t1 = t;
        }
        if ((denumen < 0) && (t < t2)) {
            t2 = t;
        }

        cout << "Intersection " << t << " at " << ray.getPoint(t) << " is " << (numen > 0 ? "enter" : "exit") << std::endl;

    }
    return t2 > t1;
}
#endif


} //namespace corecvs

