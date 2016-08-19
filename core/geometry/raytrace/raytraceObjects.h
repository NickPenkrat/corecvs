#ifndef RAYTRACEOBJECTS_H
#define RAYTRACEOBJECTS_H

#include "bspTree.h"
#include "mesh3DDecorated.h"
#include "raytrace/raytraceRenderer.h"

class RaytraceableTransform : public Raytraceable
{
public:
    Matrix44 mMatrix;
    Matrix44 mMatrixInv;

    Raytraceable *mObject;

    RaytraceableTransform(Raytraceable *object, const Matrix44 &matrix) :
        mMatrix(matrix),
        mObject(object)
    {
        mMatrixInv = mMatrix.inverted();
    }

    virtual bool intersect(RayIntersection &intersection) override;
    virtual void normal(RayIntersection &intersection)   override;
    virtual bool inside (Vector3dd &point)  override;

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
    virtual void normal(RayIntersection &intersection)   override;
    virtual bool inside (Vector3dd &point)  override;



    // Raytraceable interface
public:
    virtual bool toMesh(Mesh3D &target) override;
};

class RaytraceableCylinder: public Raytraceable {
public:
    static const double EPSILON;

    bool flag;

    /* We need matrix here */
    Vector3dd p;   /**< center of one of the faces */
    Matrix33 rotation; /**<   Stores cylinder orientation. World to "cylinder zero" tarnsformation  */



    double r;      /**< cylinder radius */
    double h;      /**< cylinder height */

    RaytraceableCylinder() :
        flag(false),
        rotation(Matrix33::RotationX(degToRad(90)))
    {}

    virtual bool intersect(RayIntersection &intersection) override;
    virtual void normal(RayIntersection &intersection)   override;
    virtual bool inside (Vector3dd &point)  override;

    // Raytraceable interface
public:
    virtual bool toMesh(Mesh3D &) override;
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
    virtual void normal(RayIntersection &intersection) override;
    virtual bool inside (Vector3dd &point)  override;
};


class RaytraceableTriangle : public Raytraceable {
public:
    static const double EPSILON;

    Triangle3dd mTriangle;

    RaytraceableTriangle(const Triangle3dd &triangle) :
        mTriangle(triangle)
    {
    }

    virtual bool intersect(RayIntersection &intersection) override;
    virtual void normal(RayIntersection &intersection) override;
    virtual bool inside (Vector3dd &point)  override;
};


class RaytraceableMesh : public Raytraceable {
public:
    static const double EPSILON;

    Mesh3DDecorated *mMesh;

    RaytraceableMesh(Mesh3DDecorated *mesh);

    virtual bool intersect(RayIntersection &intersection) override;
    virtual void normal(RayIntersection &intersection) override;
    virtual bool inside (Vector3dd &point)  override;
};




class RaytraceableOptiMesh : public RaytraceableMesh {
public:
    static const double EPSILON;



    BSPTreeNode *opt = NULL;

    RaytraceableOptiMesh(Mesh3DDecorated *mesh) :
        RaytraceableMesh(mesh)
    {}

    void optimize();
    void dumpToMesh(Mesh3D &mesh, bool plane, bool volume);

    virtual bool intersect(RayIntersection &intersection) override;
    virtual void normal(RayIntersection &intersection) override;

    virtual ~RaytraceableOptiMesh()
    {
        delete_safe(opt);
    }
};


class RaytraceableUnion : public Raytraceable {
public:
    vector<Raytraceable *> elements;

    virtual bool intersect(RayIntersection &intersection) override;
    virtual void normal(RayIntersection &intersection) override;
    virtual bool inside (Vector3dd &point)  override;
};


#endif // RAYTRACEOBJECTS_H
