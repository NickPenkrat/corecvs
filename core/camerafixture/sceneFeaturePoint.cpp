#include "sceneFeaturePoint.h"
#include "fixtureScene.h"

namespace corecvs {

std::string SceneObservation::getPointName()
{
    if (featurePoint == NULL) return std::string("");
    return featurePoint->name;
}

FixtureCamera *SceneObservation::getCameraById(FixtureCamera::IdType id)
{
    return featurePoint->ownerScene->getCameraById(id);
}

} //namespace corecvs
