#ifndef FIXTURE_CAMERA_H
#define FIXTURE_CAMERA_H

#include <stdint.h>
#include <unordered_map>

#include "atomicOps.h"
#include "calibrationLocation.h"  // LocationData
#include "lensDistortionModelParameters.h"
#include "line.h"
#include "convexPolyhedron.h"
#include "pointObservation.h"
#include "calibrationCamera.h"

#include "cameraPrototype.h"

namespace corecvs {

class CameraFixture;



class FixtureCamera : public FixtureScenePart, public CameraModel
{
public:
    CameraFixture   *cameraFixture = NULL;
    /**
     *   We are now in transition. Camera prototype should prevail and the inheritance of CameraModel
     *   should be removed. So far we have both.
     **/
    CameraPrototype *cameraPrototype = NULL;

    /* This variable is not controlled and maintained */
    int             sequenceNumber;

    FixtureCamera(FixtureScene * owner = NULL) :
        FixtureScenePart(owner),
        cameraFixture(NULL)
    {}

    FixtureCamera(
            const PinholeCameraIntrinsics &_intrinsics,
            const CameraLocationData &_extrinsics = CameraLocationData(),
            const LensDistortionModelParameters &_distortion = LensDistortionModelParameters(),
            FixtureScene * owner = NULL)
        : FixtureScenePart(owner)
        , CameraModel(_intrinsics, _extrinsics, _distortion)
        , cameraFixture(NULL)
    {}

    template<class VisitorType>
    void accept(VisitorType &visitor)
    {
        CameraModel::accept(visitor);
        IdType id = getObjectId();
        visitor.visit(id, IdType(0) , "id");
        setObjectId(id);
    }

    /** This is an experimental block of functions  it may change. Please use with caution **/

    /** WHY SO SLOW? **/
    bool projectPointFromWorld(const Vector3dd &point, Vector2dd *projectionPtr = NULL);

    Affine3DQ getWorldLocation();
    void setWorldLocation(const Affine3DQ &location);

    CameraModel getWorldCameraModel();

};





} // namespace corecvs

#endif // #ifndef FIXTURE_CAMERA_H
