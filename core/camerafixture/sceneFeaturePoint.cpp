#include "sceneFeaturePoint.h"
#include "fixtureScene.h"

namespace corecvs {
#ifdef WIN32
WPP::UTYPE const WPP::UWILDCARD = nullptr;
WPP::VTYPE const WPP::VWILDCARD = nullptr;
#endif

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
