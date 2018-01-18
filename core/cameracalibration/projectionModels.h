#ifndef PROJECTIONMODELS_H
#define PROJECTIONMODELS_H

#include "core/math/vector/vector2d.h"
#include "core/math/vector/vector3d.h"


namespace corecvs {

class CameraProjection {
public:
    enum ProjectionType {
        PINHOLE = 0,
        EQUIDISTANT,
        CATADIOPTRIC,
        STEREOGRAPHIC,
        EQUISOLID,
        ORTHOGRAPIC /*Other impelementation*/

    };

    ProjectionType projection = PINHOLE;

    CameraProjection(ProjectionType projection) :
        projection(projection)
    {}

    /**
     * Returns 2d image point of the 3d target object
     *
     * If point is not visible result is undefined
     **/
    virtual Vector2dd   project(const Vector3dd &p) const = 0;

    /**
     * Returns 3d vector that 2d image point of the 3d target object
     *
     * If point is not visible result is undefined
     **/
    virtual Vector3dd   reverse(const Vector2dd &p) const = 0;


    virtual bool      isVisible(const Vector3dd &p) const = 0;

    /**
     * Returns target image size
     **/
    virtual Vector2dd size() const = 0;

    virtual Vector2dd distortedSize() const = 0;

    virtual Vector2dd principal() const = 0;
    /**
     *  Optional inteface part that automates projection usage as function
     **/
    template<class ElementType>
    static ElementType rayToAngle(const Vector3d<ElementType> &ray)
    {
        return atan2(ray.xy().l2Metric(), ray.z());
    }

    virtual CameraProjection *clone() const = 0;

    virtual ~CameraProjection();

    /* Now some helper methods */
    double  w() const  { return size().x(); }
    double  h() const  { return size().y(); }

    Vector2dd reprojectionError(const Vector3dd &p, const Vector2dd &pp) const
    {
        return project(p) - pp;
    }
    Vector3dd crossProductError(const Vector3dd &p, const Vector2dd &pp)
    {
        return p.normalised() ^ reverse(pp).normalised();
    }

    Vector3dd rayDiffError(const Vector3dd &p, const Vector2dd &pp)
    {
        return reverse(pp).normalised() - p.normalised();
    }

    /* Common method */
    bool isPinhole() const
    {
        return projection == PINHOLE;
    }


};



class StereographicProjection : public CameraProjection {
public:
    Vector2dd principal;        /**< Principal point of optical axis on image plane (in pixel). Usually center of imager */
    double    focal;            /**< Focal length */

    StereographicProjection(Vector2dd principal, double focal) :
        CameraProjection(STEREOGRAPHIC),
        principal(principal),
        focal(focal)
    {}

public:
    virtual Vector2dd project(const Vector3dd &p) const override
    {

        double tau = rayToAngle<double>(p);
        Vector2dd dir = p.xy().normalised();
        return dir * 2 * focal * tan(tau / 2);
    }

    virtual Vector3dd reverse(const Vector2dd &p) const override
    {
        Vector2dd shift = p - principal;
        double r = shift.l2Metric();
        shift /= r;
        double tau = 2 * atan2(r / 2, focal);
        return Vector3dd(shift * tau, 1.0).normalised();
    }

    /* TODO: Function not actually implemented */
    virtual bool isVisible(const Vector3dd &/*p*/) const override
    {
        return false;
    }

    virtual ~StereographicProjection();
};


/**
 * Equisolid projection
 **/

class EquisolidAngleProjection : public CameraProjection {
public:
    Vector2dd principal;        /**< Principal point of optical axis on image plane (in pixel). Usually center of imager */
    double    focal;            /**< Focal length */

    EquisolidAngleProjection(Vector2dd principal, double focal) :
        CameraProjection(EQUISOLID),
        principal(principal),
        focal(focal)
    {}

    // CameraProjection interface
public:
    virtual Vector2dd project(const Vector3dd &p) const override
    {
        double tau = rayToAngle(p);
        Vector2dd dir = p.xy().normalised();
        return dir * 2 * focal * sin(tau / 2);
    }

    virtual Vector3dd reverse(const Vector2dd &p) const override
    {
        Vector2dd shift = p - principal;
        double r = shift.l2Metric();
        shift /= r;
        double tau = 2 * asin(r / 2.0 / focal);
        return Vector3dd(shift * tau, 1.0).normalised();
    }

    /* TODO: Function not actually implemented */
    virtual bool isVisible(const Vector3dd &/*p*/) const override
    {
        return false;
    }
};

class OrthographicProjection : public CameraProjection {
public:

    Vector2dd principal;        /**< Principal point of optical axis on image plane (in pixel). Usually center of imager */
    double    focal;            /**< Focal length */

    OrthographicProjection(Vector2dd principal, double focal) :
        CameraProjection(ORTHOGRAPIC),
        principal(principal),
        focal(focal)
    {}

    // CameraProjection interface
public:
    virtual Vector2dd project(const Vector3dd &p) const override
    {
        double tau = rayToAngle(p);
        Vector2dd dir = p.xy().normalised();
        return dir * focal * sin(tau);
    }

    virtual Vector3dd reverse(const Vector2dd &p) const override
    {
        Vector2dd shift = p - principal;
        double r = shift.l2Metric();
        shift /= r;
        double tau = asin(r / focal);
        return Vector3dd(shift * tau, 1.0).normalised();
    }

    /* TODO: Function not actually implemented */
    virtual bool isVisible(const Vector3dd &/*p*/) const override
    {
        return false;
    }
};




} // namespace corecvs


#endif // PROJECTIONMODELS_H

