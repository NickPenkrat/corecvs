#ifndef RAYTRACERENDERER_H
#define RAYTRACERENDERER_H

#include "calibrationCamera.h"

namespace corecvs {

typedef Vector3dd TraceColor;

class Raytraceable;
class RaytraceRenderer;

class RayIntersection {
public:
    Raytraceable *object = NULL;
    Ray3d ray;
    Ray3d reflection;
    Ray3d refraction;
    Vector3dd normal;
    double t;

    double weight;
    int depth;

    /* This needs to be separated */
    TraceColor ownColor;

    void computeBaseRays();
    Vector3dd getPoint();

};

class RaytraceablePointLight {
public:
    TraceColor color;
    Vector3dd position;

    RaytraceablePointLight(
        TraceColor _color,
        Vector3dd _position
    ) : color(_color),
        position(_position)
    {}
};

class RaytraceableMaterial {
public:
    double reflCoef = 0.0;
    double refrCoef = 0.0;

    TraceColor ambient  = TraceColor::Zero();
    TraceColor diffuse  = TraceColor::Zero();
    TraceColor specular = TraceColor::Zero();
    double specPower = 4.0;

    virtual void getColor(RayIntersection &ray, RaytraceRenderer &renderer);

};

class RaytraceableChessMaterial : public RaytraceableMaterial {
public:
    virtual void getColor(RayIntersection &ray, RaytraceRenderer &renderer);
};


class Raytraceable {
public:
    std::string name;

    TraceColor color;
    RaytraceableMaterial *material = NULL;

    virtual bool intersect(RayIntersection &intersection) = 0;
    virtual Vector3dd normal(const Vector3dd &vector) = 0;
    virtual bool inside (Vector3dd &point) = 0;

    virtual ~Raytraceable();
};

class RaytraceableSphere : public Raytraceable{
public:
    static const double EPSILON;

    bool flag;
    Sphere3d mSphere;

    RaytraceableSphere(const Sphere3d &sphere) :
        flag(false),
        mSphere(sphere)
    {
    }

    virtual bool intersect(RayIntersection &intersection) override;
    virtual Vector3dd normal(const Vector3dd &vector)  override;
    virtual bool inside (Vector3dd &point)  override;
};


class RaytraceablePlane : public Raytraceable {
public:
    static const double EPSILON;

    bool flag;
    Plane3d mPlane;

    RaytraceablePlane(const Plane3d &plane) :
        flag(false),
        mPlane(plane)
    {
    }

    virtual bool intersect(RayIntersection &intersection) override;
    virtual Vector3dd normal(const Vector3dd &vector)  override;
    virtual bool inside (Vector3dd &point)  override;
};


class RaytraceableUnion : public Raytraceable {
public:
    vector<Raytraceable *> elements;

    virtual bool intersect(RayIntersection &intersection) override;
    virtual Vector3dd normal(const Vector3dd &vector)  override;
    virtual bool inside (Vector3dd &point)  override;
};


class RaytraceRenderer
{
public:
    PinholeCameraIntrinsics intrisics;
    Raytraceable *object;
    vector<RaytraceablePointLight *> lights;
    TraceColor ambient;

    /**/
    AbstractBuffer<TraceColor> *energy = NULL;
    AbstractBuffer<int> *markup = NULL;

    int currentY, currentX;

    RaytraceRenderer();

    void trace(RayIntersection &intersection);
    void trace(RGB24Buffer *buffer);


};

} // namespace corecvs

#endif // RAYTRACERENDERER_H
