#include "sceneFeaturePoint.h"
#include "fixtureScene.h"
#include "multicameraTriangulator.h"


#ifdef WITH_BOOST
#include <boost/math/special_functions/gamma.hpp>
#endif

namespace corecvs {

#if !defined(WIN32) || (_MSC_VER >= 1900) // Sometime in future (when we switch to VS2015 due to https://msdn.microsoft.com/library/hh567368.apx ) we will get constexpr on windows
#else
WPP::UTYPE const WPP::UWILDCARD = nullptr;
WPP::VTYPE const WPP::VWILDCARD = nullptr;
#endif

std::string SceneObservation::getPointName()
{
    return featurePoint ? featurePoint->name : "";
}

FixtureCamera *SceneObservation::getCameraById(FixtureCamera::IdType id)
{
    CORE_ASSERT_TRUE_S(featurePoint);
    CORE_ASSERT_TRUE_S(featurePoint->ownerScene);

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

void SceneFeaturePoint::removeObservation(SceneObservation *in)
{
    auto it = observations.begin();

    for (; it != observations.end(); it++)
    {
        //FixtureCamera *cam = it->first;
        const SceneObservation &observ = it->second;

        if (in == &observ) {
            break;
        }
    }

    if (it == observations.end()) {
        return;
    }

    observations.erase(it);
}

Vector3dd SceneFeaturePoint::triangulate(bool use__, uint32_t mask)
{
    MulticameraTriangulator mct;
    int id = 0;
    if (use__)
    {
        for (auto& obs: observations__)
            if (mask & (1 << id))
                mct.addCamera(obs.first.u->getMMatrix(obs.first.v), obs.second.observation);
    }
    else
    {
        for (auto& obs: observations)
            if (mask & (1 << id))
                mct.addCamera(obs.second.cameraFixture->getMMatrix(obs.second.camera), obs.second.observation);
    }
    auto res = mct.triangulateLM(mct.triangulate());
    accuracy = mct.getCovarianceInvEstimation(res);
    return res;
}

double SceneFeaturePoint::queryPValue(const corecvs::Vector3dd &query) const
{
/*
 * Enjoy the first place of adding boost into corecvs!
 * If you are clever enough (and not too lazy) to compute
 * incomplete gamma function inverse manually - consider at least
 * writing something usefull in #else part.
 *
 * You can start from
 *     A. R. Didonato, A. H. Morris, Computation of the Incomplete Gamma Function Ratios and their Inverse 1986
 *
 * Here we are using covariance estimate to get p-value for multivariate normal distribution.
 * NOTE: however it does not seem to work on non-synthetic data either 'cause crude hessian estimation or
 * because of huge non-normality of the estimate
 */
#ifdef WITH_BOOST
    auto q = query - reprojectedPosition;
    return boost::math::gamma_p(3.0 / 2.0, std::max(0.0, q & (accuracy * q)));
#else
    CORE_UNUSED(query);
    return -1.0;
#endif
}

} //namespace corecvs
