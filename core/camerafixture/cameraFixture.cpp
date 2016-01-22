#include "cameraFixture.h"
#include "fixtureScene.h"

namespace corecvs {

void CameraFixture::setCameraCount(int count) {
    while  (cameras.size() > (size_t)count) {
        FixtureCamera *model = cameras.back();
        cameras.pop_back(); /* delete camera will generally do it, but only in owner scene.*/
        model->ownerScene->deleteCamera(model);
    }

    while  (cameras.size() < (size_t)count) {
        FixtureCamera *model  = ownerScene->createCamera();
        ownerScene->addCameraToFixture(model, this);
    }
}



} // namespace corecvs

