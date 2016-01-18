#pragma once

#include <unordered_map>

#include "calibrationLocation.h"  // LocationData
#include "lensDistortionModelParameters.h"
#include "line.h"
#include "convexPolyhedron.h"
#include "pointObservation.h"
#include "calibrationCamera.h"

namespace corecvs {

class FixtureScene;
class CameraFixture;

/**
 * This class is so far just a common base for all objects in scene heap.
 * Should bring this to other file
 **/
class FixtureScenePart {
public:
    /* No particular reason for this, except to encourage leak checks */
    static int OBJECT_COUNT;

    FixtureScene *ownerScene;

    /* We could have copy constructors and stuff... but so far this is enough */
    FixtureScenePart(FixtureScene * owner = NULL) :
        ownerScene(owner)
    {
        atomic_inc_and_fetch(&OBJECT_COUNT);
    }


    virtual ~FixtureScenePart() {
        atomic_dec_and_fetch(&OBJECT_COUNT);
    }
};


typedef std::unordered_map<std::string, void *> MetaContainer;

class FixtureCamera : public FixtureScenePart, public CameraModel
{
public:
    CameraFixture   *cameraFixture;

    FixtureCamera(FixtureScene * owner = NULL) :
        FixtureScenePart(owner)
    {}

    FixtureCamera(
            const PinholeCameraIntrinsics &_intrinsics,
            const CameraLocationData &_extrinsics = CameraLocationData(),
            const LensDistortionModelParameters &_distortion = LensDistortionModelParameters(),
            FixtureScene * owner = NULL) :
        FixtureScenePart(owner),
        CameraModel(_intrinsics, _extrinsics, _distortion)
    {}



};


} // namespace corecvs
