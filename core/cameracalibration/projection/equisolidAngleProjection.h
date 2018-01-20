#ifndef EQUISOLIDANGLEPROJECTION_H
#define EQUISOLIDANGLEPROJECTION_H

#include "core/math/vector/vector2d.h"
#include "core/math/vector/vector3d.h"
#include "core/function/function.h"

#include "core/cameracalibration/projection/projectionModels.h"
#include "core/xml/generated/projectionBaseParameters.h"

namespace corecvs {
/**
 * Equisolid projection
 **/

class EquisolidAngleProjection :  public ProjectionBaseParameters, public CameraProjection {
public:
    //Vector2dd principal;        /**< Principal point of optical axis on image plane (in pixel). Usually center of imager */
    //double    focal;            /**< Focal length */

    EquisolidAngleProjection() :
        CameraProjection(ProjectionType::EQUISOLID)
    {}

    EquisolidAngleProjection(const Vector2dd &principal, double focal, const Vector2dd &size) :
        ProjectionBaseParameters(principal.x(), principal.y(), focal, size.x(), size.y(), size.x(), size.y()),
        CameraProjection(ProjectionType::EQUISOLID)
    {}

    // CameraProjection interface
public:
    virtual Vector2dd project(const Vector3dd &p) const override
    {
        double tau = rayToAngle(p);
        Vector2dd dir = p.xy().normalised();
        return dir * 2 * focal() * sin(tau / 2);
    }

    virtual Vector3dd reverse(const Vector2dd &p) const override
    {
        Vector2dd shift = p - principal();
        double r = shift.l2Metric();
        shift /= r;
        double tau = 2 * asin(r / 2.0 / focal());
        return Vector3dd(shift * tau, 1.0).normalised();
    }

    /* TODO: Function not actually implemented */
    virtual bool isVisible(const Vector3dd &/*p*/) const override
    {
        return false;
    }

    virtual Vector2dd size() const override
    {
        return  Vector2dd(sizeX(), sizeY());
    }

    virtual Vector2dd distortedSize() const override
    {
        return  Vector2dd(distortedSizeX(), distortedSizeY());
    }

    /*Vector2dd focal() const
    {
        return  Vector2dd(focalX(), focalY());
    }*/

    virtual Vector2dd principal() const override
    {
        return  Vector2dd(principalX(), principalY());
    }


    /* Misc */
    virtual EquisolidAngleProjection *clone() const
    {
        EquisolidAngleProjection *p = new EquisolidAngleProjection();
        *p = *this;
        return p;
    }

    virtual DynamicObjectWrapper getDynamicWrapper() override
    {
        return DynamicObjectWrapper(&reflection, static_cast<ProjectionBaseParameters *>(this));
    }

    virtual ~EquisolidAngleProjection() {}
};

} // namespace corecvs
#endif // EQUISOLIDANGLEPROJECTION_H
