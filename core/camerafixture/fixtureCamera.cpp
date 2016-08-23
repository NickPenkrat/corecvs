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


} // namespace corecvs
