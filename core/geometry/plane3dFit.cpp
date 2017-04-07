#include "plane3dFit.h"
#include "ransac.h"

namespace corecvs {

Plane3dFit::Plane3dFit()
{

}

void Plane3dFit::addPoint(const Vector3dd &point)
{
    points.push_back(point);
}

void Plane3dFit::setPointList(const vector<Vector3dd> &pointList)
{
    points.clear();
    points = pointList;
}

void Plane3dFit::operator ()()
{
    Ransac<Vector3dd, PlaneReconstructionModel> estimator(mParameters.sampleNumber);
    vector<Vector3dd *> ptrArr;
    ptrArr.reserve(points.size());
    for (Vector3dd &v : points)
    {
        ptrArr.push_back(&v);
    }

    estimator.data = &ptrArr;
    PlaneReconstructionModel model = estimator.getModelRansac();

}

} // namespace corecvs
