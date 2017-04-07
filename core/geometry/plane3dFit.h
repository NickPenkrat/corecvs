#ifndef PLANE3DFIT_H
#define PLANE3DFIT_H

#include <vector>
#include "vector3d.h"
#include "line.h"
#include "ellipticalApproximation.h"

namespace corecvs {

class Plane3dFitParameters {
public:
    double tolerance = 1.0;
    int sampleNumber = 3;

};

class Plane3dFit
{
public:
    class PlaneReconstructionModel {
    public:
        EllipticalApproximation3d approx;
        Plane3d plane;

        PlaneReconstructionModel()
        {
            plane = Plane3d::FromNormal(Vector3dd::OrtX());
        }

        PlaneReconstructionModel (vector<Vector3dd *> points)
        {
            for (auto v: points) {
                approx.addPoint(*v);
            }
            approx.getEllipseParameters();
            plane = Plane3d::FromNormalAndPoint(approx.mAxes[2], approx.getMean());
            plane.normalise();
        }

        bool fits(const Vector3dd &data, double fitTreshold)
        {
            return (plane.distanceTo(data) < fitTreshold);
        }
    };


public:
    Plane3dFit();

    Plane3dFitParameters mParameters;
    Plane3d result;

    std::vector<Vector3dd> points;

    void addPoint    (const Vector3dd &point);
    void setPointList(const vector<Vector3dd> &pointList);

    void setParameters(const Plane3dFitParameters& parameters)
    {
        mParameters = parameters;
    }

    void operator ()();

};

} // namespace corecvs

#endif // PLANE3DFIT_H
