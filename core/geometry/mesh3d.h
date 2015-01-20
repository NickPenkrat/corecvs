#pragma once

/**
 * \file mesh3d.h
 *
 * \date Dec 13, 2012
 **/

#include "generated/axisAlignedBoxParameters.h"
#include "vector3d.h"
#include "matrix33.h"
#include "matrix44.h"
#include "cameraParameters.h"

namespace corecvs
{

class AxisAlignedBox3d
{
    Vector3dd mLow;
    Vector3dd mHigh;
public:

    AxisAlignedBox3d()
    {
        _initByLowHigh(Vector3dd(0.0), Vector3dd(0.0));
    }

    AxisAlignedBox3d(const Vector3dd &low, const Vector3dd &high)
    {
        _initByLowHigh(low, high);
    }

    AxisAlignedBox3d(const AxisAlignedBoxParameters &params)
    {
        Vector3dd measure(params.width(), params.height(), params.depth());
        Vector3dd center (params.x()    , params.y()     , params.z    ());

        _initByCenter(center, measure);
    }

    void _initByLowHigh(const Vector3dd &low, const Vector3dd &high)
    {
        /* TODO: Move this to vector operations and use cycle */
        mLow .x() = std::min(low.x(), high.x());
        mHigh.x() = std::max(low.x(), high.x());

        mLow .y() = std::min(low.y(), high.y());
        mHigh.y() = std::max(low.y(), high.y());

        mLow .z() = std::min(low.z(), high.z());
        mHigh.z() = std::max(low.z(), high.z());
    }

    void _initByCenter(const Vector3dd &center, const Vector3dd &measure)
    {
        _initByLowHigh((center - measure / 2.0), (center + measure / 2.0));
    }

    static AxisAlignedBox3d ByCenter(const Vector3dd &center, const Vector3dd &measure)
    {
        AxisAlignedBox3d box;
        box._initByCenter(center, measure);
        return box;
    }

    bool contains (const Vector3dd &vector) const
    {
        return vector.isInCube(mLow, mHigh);
    }

    const Vector3dd &low()  const {
        return mLow;
    }

    const Vector3dd &high() const {
        return mHigh;
    }

    double width() const {
        return mHigh.x() - mLow.x();
    }

    double height() const {
        return mHigh.y() - mLow.y();
    }

    double depth() const {
        return mHigh.z() - mLow.z();
    }

    Vector3dd getCenter()
    {
        return ((mLow + mHigh) / 2.0);
    }

    Vector3dd measure()
    {
        return mHigh - mLow;
    }

    void getPoints(Vector3dd points[8])
    {
        points[0] = Vector3dd(mLow .x() , mLow .y(), mLow.z());
        points[1] = Vector3dd(mHigh.x() , mLow .y(), mLow.z());
        points[2] = Vector3dd(mHigh.x() , mHigh.y(), mLow.z());
        points[3] = Vector3dd(mLow .x() , mHigh.y(), mLow.z());
        points[4] = Vector3dd(mLow .x() , mLow .y(), mHigh.z());
        points[5] = Vector3dd(mHigh.x() , mLow .y(), mHigh.z());
        points[6] = Vector3dd(mHigh.x() , mHigh.y(), mHigh.z());
        points[7] = Vector3dd(mLow .x() , mHigh.y(), mHigh.z());
    }

    AxisAlignedBox3d outset(double value) const
    {
        return AxisAlignedBox3d(mLow - Vector3dd(value), mHigh + Vector3dd(value));
    }

};



class Mesh3D {
public:

    Mesh3D() :
        hasCentral(false)
    {}


    Vector3dd  centralPoint;
    bool hasCentral;

    vector<Vector3dd>  vertexes;
    vector<Vector3d32> faces;
    vector<Vector2dd>  textureCoords;

    void setCentral(Vector3dd _central)
    {
        centralPoint = _central;
        hasCentral = true;
    }


    void addAOB(Vector3dd corner1, Vector3dd corner2);
    void addAOB(const AxisAlignedBoxParameters &box);
    void addAOB(const AxisAlignedBox3d &box);

    void addSphere(Vector3dd center, double radius, int step);

    void addCamera(const CameraIntrinsics &cam, double len);

#if 0
    void addTruncatedCone(double r1, double r2, double length, int steps = 16);
#endif

    void dumpPLY(ostream &out);
    void transform (const Matrix44 &matrix);
    Mesh3D transformed(const Matrix44 &matrix);

    void add(const Mesh3D &other);

};



} /* namespace corecvs */
/* EOF */
