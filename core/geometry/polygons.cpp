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

#include "rgb24Buffer.h"
#include "abstractPainter.h"

namespace corecvs {

/**
 *
 * Check if point is inside the polygon
 **/
int Polygon::isInsideConvex(const Vector2dd &a) const
{
    double oldsign = 0;
    int len = (int)size();
    for (int i = 0; i < len; i++)
    {
        const Vector2dd &curr = getPoint(i);
        const Vector2dd &next = getNextPoint(i);
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

/**
 *  Using idea from  http://geomalgorithms.com/a03-_inclusion.html
 **/
int Polygon::windingNumber(const Vector2dd &point) const
{
    int    windingNumber = 0;

    for (size_t i = 0; i < size(); i++) {
        Vector2dd cur  = getPoint(i);
        Vector2dd next = getNextPoint(i);
        Vector2dd dir  = next - cur;

        bool startsNotAbove = (cur.y() <= point.y());
        bool startsAbove    = !startsNotAbove;

        bool endsAbove      = (next.y() > point.y());
        bool endsNotAbove   = !endsAbove;


        if (startsNotAbove && endsAbove) { // Crossing up
            if ( dir.parallelogramOrientedAreaTo(point - next) > 0)
                 windingNumber++;
        }

        if (startsAbove && endsNotAbove)  { // Crossing down
            if ( dir.parallelogramOrientedAreaTo(point - next) < 0)
                 windingNumber--;
        }
    }
    return windingNumber;
}

int  Polygon::isInside(const Vector2dd &point) const
{
    return windingNumber(point) != 0;
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

Polygon Polygon::RegularPolygon(int sides, const Vector2dd &center, double radius, double startAngleRad) {
    Polygon toReturn;
    toReturn.reserve(sides);

    double step = degToRad(360.0 / sides);

    for (int i = 0; i < sides; i++)
    {
        toReturn.push_back(center + Vector2dd::FromPolar(step * i + startAngleRad, radius));
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
    intersectionNumber = 0;
    intersections.clear();
    c[0].clear();
    c[1].clear();

    for (int p = 0; p < 2; p++)
    {
        if (pol[p].signedArea() > 0) pol[p] = Polygon::Reverse(pol[p]);
    }

    for (int p = 0; p < 2; p++)
    {
        for (size_t i = 0; i < pol[p].size(); i++) {
            Vector2dd point = pol[p][i];
            VertexData vd = {i, point, (pol[1-p].isInside(point) ? INSIDE : OUTSIDE), 0};
            c[p].push_back(vd);
        }
    }

    for (size_t i = 0; i < pol[0].size(); i++)
    {
        for (size_t j = 0; j < pol[1].size(); j++)
        {
            Ray2d r1 = pol[0].getRay(i);
            Ray2d r2 = pol[1].getRay(j);
            /* Could make a fast check here before any divisions */

            double t1 = 0;
            double t2 = 0;
            Vector2dd x = Ray2d::intersection(r1, r2, t1, t2);

            if (t1 == std::numeric_limits<double>::infinity())
            {
                /**
                 * Sides are parallel. We assume nothing is to be done so far.
                 **/
                continue;
            }


            if ((t1 < 0 || t1 > 1.0) ||
                (t2 < 0 || t2 > 1.0))
            {
                continue;
            }

            VertexData vd1(i, x, COMMON, t1, intersectionNumber);
            c[0].push_back(vd1);

            VertexData vd2(j, x, COMMON, t2,  intersectionNumber);
            c[1].push_back(vd2);

            intersectionNumber++;
        }
    }

    auto comparator = [](VertexData &vd1, VertexData &vd2)
    {
            if (vd1.orgId == vd2.orgId)
                return vd1.t < vd2.t;
            return vd1.orgId < vd2.orgId;
    };

    std::sort(c[0].begin(), c[0].end(), comparator); // TODO: First array can be created presorted
    std::sort(c[1].begin(), c[1].end(), comparator);

    intersections.resize(intersectionNumber);
    for (size_t i = 0; i < c[0].size(); i++)
    {
        if (c[0][i].flag == COMMON)
            intersections[c[0][i].other].first = i;
    }
    for (size_t i = 0; i < c[1].size(); i++)
    {
        if (c[1][i].flag == COMMON)
            intersections[c[1][i].other].second = i;
    }
    for (size_t i = 0; i < intersections.size(); i++)
    {
        c[0][intersections[i].first ].other = intersections[i].second;
        c[1][intersections[i].second].other = intersections[i].first;
    }
}

bool PolygonCombiner::validateState() const
{
    bool ok = true;
    for (int p = 0; p < 2; p++)
    {
        size_t start = -1;
        size_t oldId = -1;

        int skipped = 0;

        VertexData vc = c[p][0];

        for (size_t i = 0; i < c[p].size(); i++)
        {
            vc = c[p][i];
            if (vc.flag == INSIDE || vc.flag == OUTSIDE)
            {
                start = i;
                break;
            }
        }

        if (start == -1)
        {
            cout << "No inside or outside nodes... " << endl;
            ok = false;
        }

        oldId = start;

        for (size_t i = 0; i <= c[p].size(); i++)
        {
            int num = (i + start) % c[p].size();

           const VertexData &vn = c[p][num];
           if (vn.flag == COMMON)
           {
                skipped++;
                continue;
           }

           if (vn.flag == INSIDE  && vc.flag == OUTSIDE)
           {
               if (skipped % 2 != 1) {
                   printf("We have a problematic span [%c%d - out %c%d - in]: crossings %d \n", p == 0 ? 'A' : 'B', (int)oldId , p == 0 ? 'A' : 'B', num, skipped);
                   ok = false;
               }
           }

           if (vn.flag == OUTSIDE && vc.flag == INSIDE )
           {
               if (skipped % 2 != 1) {
                   printf("We have a problematic span [%c%d - in %c%d - out]: crossings %d \n", p == 0 ? 'A' : 'B', (int)oldId , p == 0 ? 'A' : 'B', num, skipped);
                   ok = false;
               }
           }

           if (vn.flag == INSIDE  && vc.flag == INSIDE)
           {
               if (skipped % 2 == 1) {
                   printf("We have a problematic span [%c%d - in %c%d - in] : crossings %d\n", p == 0 ? 'A' : 'B', (int)oldId , p == 0 ? 'A' : 'B', num, skipped);
                   ok = false;
               }
           }

           if (vn.flag == OUTSIDE && vc.flag == OUTSIDE )
           {
               if (skipped % 2 == 1) {
                   printf("We have a problematic span [%c%d - out %c%d - out] : crossings %d\n", p == 0 ? 'A' : 'B', (int)oldId , p == 0 ? 'A' : 'B', num, skipped);
                   ok = false;
               }
           }

           oldId = num;
           skipped = 0;
           vc = vn;
        }
    }
    return ok;
}

void PolygonCombiner::drawDebug(RGB24Buffer *buffer) const
{
    AbstractPainter<RGB24Buffer> painter(buffer);

    painter.drawPolygon(pol[0], RGBColor::Yellow());
    painter.drawPolygon(pol[1], RGBColor::Cyan  ());

    for (int p = 0; p < 2; p++)
    {
        for (size_t i = 0; i < c[p].size(); i++)
        {
            const VertexData &v = c[p][i];
            Vector2dd pos = v.pos;

            if (v.flag == INSIDE)
                buffer->drawCrosshare3(pos.x(), pos.y(), RGBColor::Red());
            if (v.flag == OUTSIDE)
                buffer->drawCrosshare3(pos.x(), pos.y(), RGBColor::Green());
            if (v.flag == COMMON)
                buffer->drawCrosshare3(pos.x(), pos.y(), RGBColor::Blue());

            painter.drawFormat(pos.x(), pos.y() + p * 10, RGBColor::White(), 1, "%c%d (%0.2lf) [%d]", p == 0 ? 'A' : 'B', i, v.t, v.other);
            printf("(%lf %lf) %c%d (%0.2lf) [%d]\n", pos.x(), pos.y() , p == 0 ? 'A' : 'B', i, v.t, v.other);
        }
    }


}

Polygon PolygonCombiner::intersection() const
{
    Polygon result;
    if (intersectionNumber == 0) /* There are no contur intersection */
    {
        /* First poligon is inside the second one */
        if (!c[0].empty() && c[0].front().flag == INSIDE)
            return Polygon(pol[0]);

        /* second poligon is inside the first one */
        if (!c[1].empty() && c[1].front().flag == INSIDE)
            return Polygon(pol[1]);

        return result;
    }

    const std::pair<int, int> &fst = intersections[0];

    int currentId = fst.first;
    int currentChain = 0;

    printf("Exit condition A%d or B%d\n", fst.first, fst.second);

    int limit = 0;
    while (limit ++ < 100) {
        VertexData v = c[currentChain][currentId];
        result.push_back(v.pos);
        printf("Adding vertex c: %c%d point: %d (%lf %lf)\n", currentChain == 0 ? 'A' : 'B',
               currentId,
               v.orgId,
               c[currentChain][v.orgId].pos.x(),
               c[currentChain][v.orgId].pos.y());

        if (v.flag == INSIDE)
        {
            /* Moving inside the outer polygon */
            currentId = (currentId + 1) % c[currentChain].size();
            continue;
        }
        if (v.flag == OUTSIDE)
        {
            cout << "Internal Error" << endl;
            break;
        }

        if (v.flag == COMMON)
        {
            int otherChain = 1 - currentChain;
            int nextCurrent = (currentId  + 1) % c[currentChain].size();
            int nextOther   = (v.other    + 1) % c[otherChain  ].size();

            const VertexData &candidate1 = c[currentChain][nextCurrent];
            const VertexData &candidate2 = c[otherChain  ][nextOther];

            printf("Branching (%c%d) (%c%d)\n", currentChain == 0 ? 'A' : 'B' , nextCurrent , otherChain == 0 ? 'A' : 'B', nextOther);

            if ((currentId == 0) && (nextCurrent == fst.first))
                break;
            if ((currentId == 1) && (nextCurrent == fst.second))
                break;

            if (candidate1.flag != OUTSIDE) {
                cout << "choice1" << endl;
                currentId = nextCurrent;
            } else {
                cout << "choice2" << endl;
                currentChain = otherChain;
                currentId    = nextOther;
            }
        }
    }
    return result;
}

Polygon PolygonCombiner::combination() const
{
    SYNC_PRINT(("PolygonCombiner::combination(): Not yet implemented\n"));
    return Polygon();
}

Polygon PolygonCombiner::difference() const
{
    SYNC_PRINT(("PolygonCombiner::difference(): Not yet implemented\n"));
    return Polygon();
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

