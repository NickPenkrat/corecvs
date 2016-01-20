#include "reconstructionFixtureScene.h"

#include <set>

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
    vectorErase(placedFixtures, fixture);
    vectorErase(placingQueue, fixture);
}

void ReconstructionFixtureScene::deleteFixturePair(CameraFixture *fixture, FixtureCamera *camera)
{
    FixtureScene::deleteFixturePair(fixture, camera);
    deletePairUMWPP(images,    fixture, camera);
    deletePairUMWPP(keyPoints, fixture, camera);
    deletePairUMWPP(matches,   fixture, camera);
    deletePairUMWPP(trackMap,  fixture, camera);
}

void ReconstructionFixtureScene::detectAllFeatures(const FeatureDetectionParams &params)
{
}

int ReconstructionFixtureScene::getDistinctCameraCount() const
{
    std::set<FixtureCamera*> cameras;
    for (auto& f: placedFixtures)
    {
        for (auto& c: f->cameras)
        {
            cameras.insert(c);
        }
    }
    return cameras.size();
}
