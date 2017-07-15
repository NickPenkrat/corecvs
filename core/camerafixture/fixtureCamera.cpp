#include "fixtureCamera.h"
#include "cameraFixture.h"
#include "fixtureScene.h"

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

void FixtureCamera::setWorldLocation(const Affine3DQ &location, bool moveFixture)
{
    if (cameraFixture == NULL) {
        setLocation(location);
    }

    if (!moveFixture) {
        setLocation(cameraFixture->location.inverted() * location);
    } else {
       /* W  =    F * C */
       /* W1 = ( W1 *  C ^ -1 )  C */
       Affine3DQ F1 = location * getAffine().inverted();
       cameraFixture->location = F1;
    }
}

CameraModel FixtureCamera::getWorldCameraModel()
{
    if (cameraFixture != NULL) {
        return cameraFixture->getWorldCamera(this);
    }

    return *this;
}

void FixtureCamera::addImageToCamera(ImageRelatedData *data)
{
    if (data == NULL)
        return;

    this->mImages.push_back(data);
    data->camera  = this;
    //data->mStationImages = static_cast<CameraFixtureUI *>(this->cameraFixture);
}

void FixtureCamera::setImageCount(size_t count)
{
    if (ownerScene == NULL)
    {
        SYNC_PRINT(("FixtureCamera(%p)::setImageCount(): ownerScene is NULL ", static_cast<void*>(this)));
        return;
    }

    while (mImages.size() > count) {
        ImageRelatedData *image = mImages.back();
        mImages.pop_back();
        ownerScene->deleteImage(image);
    }

    while (mImages.size() < count) {
        ImageRelatedData *data = ownerScene->createImageData();
        addImageToCamera(data);
    }
}

} // namespace corecvs
