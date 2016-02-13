#include "sceneFeaturePoint.h"
#include "fixtureScene.h"
#include "multicameraTriangulator.h"

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

bool SceneFeaturePoint::hasObservation(FixtureCamera *cam)
{
    auto it = observations.find(cam);
    return (it != observations.end());
}

SceneObservation *SceneFeaturePoint::getObservation(FixtureCamera *cam)
{
    auto it = observations.find(cam);
    if (it == observations.end()) {
        return NULL;
    }

    return &((*it).second);
}

Vector3dd SceneFeaturePoint::triangulate(bool use__)
{
    MulticameraTriangulator mct;
    if (use__)
    {
        for (auto& obs: observations__)
            mct.addCamera(obs.first.u->getMMatrix(obs.first.v), obs.second.observation);
    }
    else
    {
        for (auto& obs: observations)
            mct.addCamera(obs.second.cameraFixture->getMMatrix(obs.second.camera), obs.second.observation);
    }
    return mct.triangulateLM(mct.triangulate());
}

} //namespace corecvs
