#include "reconstructionFixtureScene.h"

using namespace corecvs;

ReconstructionFixtureScene::ReconstructionFixtureScene()
{
}

void ReconstructionFixtureScene::deleteCamera(FixtureCamera *camera)
{
    FixtureScene::deleteCamera(camera);
    deleteFixtureCameraUMWPP(images,    camera);
    deleteFixtureCameraUMWPP(keyPoints, camera);
    deleteFixtureCameraUMWPP(matches,   camera);
    deleteFixtureCameraUMWPP(trackMap,  camera);
}

void ReconstructionFixtureScene::deleteCameraFixture(CameraFixture *fixture, bool recursive)
{
    FixtureScene::deleteCameraFixture(fixture, recursive);
    deleteCameraFixtureUMWPP(images,    fixture);
    deleteCameraFixtureUMWPP(keyPoints, fixture);
    deleteCameraFixtureUMWPP(matches,   fixture);
    deleteCameraFixtureUMWPP(trackMap,  fixture);
    initializationData.erase(fixture);
    initializationData.erase(std::remove(initializationData.begin(), initializationData.end(), fixture), initializationData.end());
}

void ReconstructionFixtureScene::deleteFixturePair(CameraFixture *fixture, FixtureCamera *camera)
{
    FixtureScene::deleteFixturePair(fixture, camera);
    deletePairUMWPP(images,    fixture, camera);
    deletePairUMWPP(keyPoints, fixture, camera);
    deletePairUMWPP(matches,   fixture, camera);
    deletePairUMWPP(trackMap,  fixture, camera);
}
