#include "fixtureCamera.h"
#include "cameraFixture.h"

namespace corecvs {

atomic_int FixtureScenePart::OBJECT_COUNT(0);

bool FixtureCamera::projectPointFromWorld(const Vector3dd &point, Vector2dd *projectionPtr)
{
    FixtureCamera worldCam = *this;
    if (cameraFixture != NULL) {
        worldCam = cameraFixture->getWorldCamera(this);
    }

    Vector2dd projection = worldCam.project(point);
    if (!worldCam.isVisible(projection) || !worldCam.isInFront(point))
    {
        return false;
    }

    if (projectionPtr != NULL) {
        *projectionPtr = projection;
    }
    return true;
}

Affine3DQ FixtureCamera::getWorldLocation()
{
   Affine3DQ local = extrinsics.toAffine3D();
   if (cameraFixture == NULL) {
       return local;
   }
   return (cameraFixture->location * local);
}

void FixtureCamera::setWorldLocation(const Affine3DQ &location)
{
    if (cameraFixture == NULL) {
        setLocation(location);
    }
    setLocation(cameraFixture->location.inverted() * location);
}

CameraModel FixtureCamera::getWorldCameraModel()
{
    if (cameraFixture != NULL) {
        return cameraFixture->getWorldCamera(this);
    }

    return *this;
}

} // namespace corecvs
