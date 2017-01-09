#ifndef POLYGONS_H_
#define POLYGONS_H_

/**
 * \file polygons.h
 * \brief Add Comment Here
 *
 * \ingroup cppcorefiles
 * \date Nov 14, 2010
 * \author alexander
 */


#include <vector>
#include <algorithm>
#include <ostream>

#include "vector3d.h"
#include "generated/axisAlignedBoxParameters.h"
#include "line.h"
#include "rectangle.h"

namespace corecvs {

using std::vector;

class PlaneFrame {
public:
    Vector3dd p1;
    Vector3dd e1;
    Vector3dd e2;

    PlaneFrame(Vector3dd p1, Vector3dd e1, Vector3dd e2) :
        p1(p1), e1(e1), e2(e2)
    {}

    Vector3dd getNormal() const
    {
        return e1 ^ e2;
    }

    bool intersectWithP(Ray3d &ray, double &resT, double &u, double &v)
    {
        double EPSILON = 0.00001;
        Vector3dd p =  ray.a ^ e2;

        /* This is the volume of the parallepiped built on two edges and a ray origin */
        double vol = e1 & p;
        if(vol > -EPSILON && vol < EPSILON)
            return false;

        double inv_vol = 1.0 / vol;

        Vector3dd T = ray.p - p1;

        u = (T & p) * inv_vol;
        if(u < 0.0 || u > 1.0) {
            return false;
        }

        Vector3dd Q = T ^ e1;

        v = (ray.a & Q) * inv_vol;

        if(v < 0.f || u + v  > 1.f) return false;

        double t = (e2 & Q) * inv_vol;

        resT = t;
        return true;
    }

};


template<typename PointType>
class GenericTriangle
{
public:
    static const int SIZE = 3;
    PointType p[SIZE];

    PointType &p1() {return p[0];}
    PointType &p2() {return p[1];}
    PointType &p3() {return p[2];}

    const PointType &p1() const {return p[0];}
    const PointType &p2() const {return p[1];}
    const PointType &p3() const {return p[2];}

    GenericTriangle() {}

    GenericTriangle(const PointType _p1, const PointType _p2, const PointType _p3)
    {
        p1() = _p1;
        p2() = _p2;
        p3() = _p3;
    }

    Plane3d getPlane() const
    {
        return Plane3d::FromPoints(p1(), p2(), p3());
    }

    Vector3dd getNormal() const
    {
        return Plane3d::NormalFromPoints(p1(), p2(), p3());
    }

    PlaneFrame toPlaneFrame() const
    {
        return PlaneFrame(p1(), p2() - p1(), p3() - p1());
    }


    bool intersectWithP(Ray3d &ray, double &resT)
    {
        //double EPSILON = 0.00001;

        PlaneFrame frame = toPlaneFrame();
        double u, v;
        return frame.intersectWithP(ray, resT, u, v);
    }

    bool intersectWith(Ray3d &ray, Vector3dd &point)
    {
        double t;
        if (intersectWithP(ray, t))
        {
            point = ray.getPoint(t);
            return true;
        }
        return false;
    }

    /** NOTE: This could swap the normal **/
    void sortByY() {
        if (p1().y() > p2().y()) std::swap(p1(), p2());
        if (p2().y() > p3().y()) std::swap(p2(), p3());
        if (p1().y() > p2().y()) std::swap(p1(), p2());
    }


    void transform(const Matrix33 &transform)
    {
        for (int i = 0; i < SIZE; i++)
        {
            p[i] = transform * p[i];
        }
    }

};

typedef GenericTriangle<Vector3d<int32_t> > Triangle32;
typedef GenericTriangle<Vector2d<double> > Triangle2dd;

typedef GenericTriangle<Vector3d<double> > Triangle3dd;


class PointPath : public vector<Vector2dd>
{
public:
    PointPath(){}

    PointPath(int len) : vector<Vector2dd>(len)
    {}

    friend std::ostream & operator <<(std::ostream &out, const PointPath &pointPath)
    {
        out << "[";
        for (size_t i = 0; i < pointPath.size(); i++)
           out << (i == 0 ? "" : ", ") << pointPath.at(i) << std::endl;
        out << "]";
        return out;
    }

    /* This function checks if the poligon is inside the buffer. It assumes that the poligon coorinate can be rounded to upper value  */
    bool isInsideBuffer(const Vector2d<int> &bufferSize)
    {
        for (Vector2dd point : *this)
        {
            if (point.x() < 0 || point.y() < 0)
                return false;
            if (point.x() + 1 > bufferSize.x() || point.y() + 1 > bufferSize.y())
                return false;

        }
        return true;
    }

    Vector2dd center();
};

/**
 *  Polygon
 **/
class Polygon : public PointPath
{
public:
    Polygon(){}

    Polygon(const Vector2dd *points, int len) : PointPath(len)
    {
        for (unsigned i = 0; i < size(); i++) {
           this->operator[](i) = points[i];
        }
    }

    /**
     * This thing checks if the point is inside of the convex poligon
     *
     * \attention convex only
     **/
    int  isInsideConvex(const Vector2dd &point) const;


    /**
     *  Winding number is the number of loops polygon makes around the point
     **/
    int windingNumber( const Vector2dd &point ) const;
    int  isInside(const Vector2dd &point) const;



    bool isConvex(bool *direction = NULL) const;

    Vector2dd &getPoint (int i) {
        return operator [](i);
    }

    const Vector2dd &getPoint (int i) const {
        return operator [](i);
    }

    Vector2dd &getNextPoint(int idx)
    {
       return operator []((idx + 1) % size());
    }

    const Vector2dd &getNextPoint(int idx) const
    {
       return operator []((idx + 1) % size());
    }

    /** **/
    Ray2d getRay(int i)  const {
        return Ray2d::FromPoints(getPoint(i), getNextPoint(i));
    }


    /** This method uses the index by module of size() **/
    Vector2dd &getPointM(int idx)
    {
       return operator [](idx % size());
    }


    Vector2dd getNormal(int i) const
    {
        Vector2dd r1 = getPoint(i);
        Vector2dd r2 = getNextPoint(i);

        return (r2 - r1).rightNormal();
    }

    static Polygon FromRectagle(const Rectangled &rect) {
        Polygon toReturn;
        toReturn.reserve(4);
        toReturn.push_back(rect.ulCorner());
        toReturn.push_back(rect.urCorner());
        toReturn.push_back(rect.lrCorner());
        toReturn.push_back(rect.llCorner());
        return toReturn;
    }

    static Polygon RegularPolygon(int sides, const Vector2dd &center, double radius, double startAngleRad = 0.0);

    static Polygon Reverse(const Polygon &p);


    Polygon transformed(const Matrix33 &transform) const {
        Polygon toReturn;
        toReturn.reserve(size());
        for (Vector2dd p: *this ) {
            toReturn.push_back(transform * p);
        }
        return toReturn;
    }

    void transform(const Matrix33 &transform) {
        for (Vector2dd &p: *this ) {
            p = transform * p;
        }
    }

    /* non const versions */
    double &x(int idx) {
        return operator [](idx).x();
    }

    double &y(int idx) {
        return operator [](idx).y();
    }

    /* const versions*/
    const double &x(int idx) const {
        return operator [](idx).x();
    }

    const double &y(int idx) const {
        return operator [](idx).y();
    }

    Ray2d edgeAsRay(int idx) {
        Vector2dd &start = getPoint(idx);
        Vector2dd &end   = getNextPoint(idx);
        return Ray2d::FromPoints(start, end);
    }

    /* Valid for any type of simple poligon*/
    double signedArea();

    double area() {
        return fabs(signedArea());
    }


    //bool clipRay(const Ray2d &ray, double &t1, double &t2);
};





class RGB24Buffer;

/**
 *  This class implements Weiler–Atherton algotithm
 *
 **/
class PolygonCombiner
{
public:
    Polygon pol[2]; /* We actually don't need to copy poligon, but for sake of simplicity we reverse them to positive orientation*/

    enum VertexType {
        INSIDE,
        COMMON,
        OUTSIDE
    };

    static std::string getName(const VertexType type)
    {
        switch (type) {
        case INSIDE:
            return "inside";
            break;
        case OUTSIDE:
            return "outside";
            break;
        case COMMON:
            return "common";
            break;
        default:
            break;
        }
        return "";
    }

    struct VertexData {
        size_t orgId;
        Vector2dd pos;     /* We don't need this, just a cache*/
        VertexType flag;
        double t;
        size_t other;

        VertexData(size_t orgId, Vector2dd pos, VertexType inside, double t, size_t other = 0) :
           orgId(orgId),
           pos(pos),
           flag(inside),
           t(t),
           other(other)
        {}
    };

    typedef std::vector<VertexData> ContainerType; /* This type should better be list */

    ContainerType c[2];

    int intersectionNumber;
    std::vector<std::pair<int, int>> intersections;

    /* Method that initialise internal data structures of the PoligonCombiner*/
    void prepare(void);

    /* */
    bool validateState(void);
    void drawDebug(RGB24Buffer *buffer);

    Polygon intersection();
    Polygon combination();
    Polygon difference();

    PolygonCombiner(){}
    PolygonCombiner(Polygon &p1, Polygon &p2)
    {
         pol[0] = p1;
         pol[1] = p2;
    }

    friend std::ostream & operator <<(std::ostream &out, const PolygonCombiner &combiner)
    {
        out << "A: [" << std::endl;
        for (size_t i = 0; i < combiner.c[0].size(); i++)
        {
            const VertexData &vd = combiner.c[0][i];
            out << " " << i << ":  (" << vd.pos << " " << getName(vd.flag) << " " << vd.t << " " << vd.other << ") " << std::endl;
        }
        out << "]" << std::endl;
        out << "B: [" << std::endl;
        for (size_t i = 0; i < combiner.c[1].size(); i++)
        {
            const VertexData &vd = combiner.c[1][i];
            out << " " << i << ":  (" << vd.pos << " " << getName(vd.flag) << " " << vd.t << " " << vd.other << ") " << std::endl;
        }
        out << "]" << std::endl;

        return out;
    }


};


class ConvexHull
{
public:
    /**
     * Most trivial and slow algorighm
     *
     * This methods need a lot of additional testing
     ***/
    static Polygon GiftWrap(const std::vector<Vector2dd> &list);
};

} //namespace corecvs
#endif /* POLYGONS_H_ */

