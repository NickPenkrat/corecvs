#include "sceneFeaturePoint.h"
#include "fixtureScene.h"
#include "multicameraTriangulator.h"
#include "propertyListVisitor.h"


#ifdef WITH_BOOST
#include <boost/math/special_functions/gamma.hpp>
#endif

namespace corecvs {

std::string SceneObservation::getPointName()
{
    return featurePoint ? featurePoint->name : "";
}

int SceneObservation::ensureDistorted(bool distorted)
{
    if (!(distorted ^ onDistorted))                         // if we have what is requested, use it
        return false;

    //cout << "SceneObservation::ensureDistorted: convert to " << (distorted ? "dist" : "undist") << " coords" << endl;
    //auto obs = observation;

    observation = getDistorted(distorted);
    onDistorted = distorted;                        // this must be after the function above call!

    //cout << "SceneObservation::ensureDistorted(" << (distorted ? "dist" : "undist") << ") " << obs << " => " << observation << endl;
    return true;
}

Vector2dd SceneObservation::getDistorted(bool distorted) const
{
    if (distorted) {
        return  onDistorted ? observation : camera->distortion.mapForward(observation);  // undist => dist
    }
    else {
        return !onDistorted ? observation : camera->distortion.mapBackward(observation); // dist => undist
    }
}

FixtureCamera *SceneObservation::getCameraById(FixtureCamera::IdType id)
{
    CORE_ASSERT_TRUE_S(featurePoint);
    CORE_ASSERT_TRUE_S(featurePoint->ownerScene);

    return featurePoint->ownerScene->getCameraById(id);
}

SceneObservation *SceneFeaturePoint::getObservation(FixtureCamera *cam)
{
    auto it = observations.find(cam);
    if (it == observations.end()) {
        return nullptr;
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

int SceneFeaturePoint::ensureDistortedObservations(bool distorted)
{
    int toReturn = 0;
    for (auto& obs : observations)      // we need to calc dist/undist coords for all observations if need
    {
        toReturn += obs.second.ensureDistorted(distorted);
    }
    return toReturn;
}

Vector3dd SceneFeaturePoint::triangulate(bool use__, std::vector<int> *mask, uint* numProjections)
{
    MulticameraTriangulator mct;
    int id = 0, ptr = 0;

    if (use__)
    {
        for (auto& obs: observations__)
        {
            if (!mask || (ptr < mask->size() && (*mask)[ptr] == id))
            {
                mct.addCamera(obs.first.u->getMMatrix(obs.first.v), obs.second.observation);
                if (mask && ptr + 1 < mask->size())
                    CORE_ASSERT_TRUE_S((*mask)[ptr] < (*mask)[ptr + 1]);
                ++ptr;
            }
            if (mask && ptr == mask->size())
                break;
            ++id;
        }
    }
    else
    {
        for (auto& obs : observations)
        {
            //cout << "SceneFeaturePoint::triangulate(" << name << ") distorted:" << obs.second.onDistorted << " " << obs.second.observation << endl;

            CORE_ASSERT_TRUE_S(obs.second.cameraFixture != NULL);
            if (!mask || (ptr < mask->size() && (*mask)[ptr] == id))
            {
                mct.addCamera(obs.second.cameraFixture->getMMatrix(obs.second.camera), obs.second.observation);
                if (mask && ptr + 1 < mask->size())
                    CORE_ASSERT_TRUE_S((*mask)[ptr] < (*mask)[ptr + 1]);
                ptr++;
            }
            if (mask && ptr == mask->size())
                break;
            id++;
        }
    }
    bool ok = false;
    Vector3dd initial = mct.triangulate(&ok);
#ifdef DEEP_TRACE_702
    if (!ok) {
        SYNC_PRINT(("SceneFeaturePoint::triangulate(%s): initial guess unable to obtain\n", name.c_str()));
    }
#endif

	//if (ok == false)
	//	return Vector3dd(0.0);

    ok = true;
    Vector3dd res = mct.triangulateLM(initial, &ok);

#ifdef DEEP_TRACE_702
    if (!ok) {
        SYNC_PRINT(("SceneFeaturePoint::triangulate(%s): LM guess unable to obtain\n", name.c_str()));
    }
#endif

#ifdef DEEP_TRACE_702
    {
        std::ostringstream ss;
        ss << "dump" << name << ".txt";
        PropertyListWriterVisitor writer(ss.str());
        mct.accept<PropertyListWriterVisitor>(writer);
    }
#endif

    // TODO: in fail case covariance couldn't be estimated, thus we keep "accuracy" field as it is.
    if (ok)
        accuracy = mct.getCovarianceInvEstimation(res); 

	if (numProjections != nullptr)
		*numProjections = mct.P.size();

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

Vector3dd SceneFeaturePoint::getDrawPosition(bool preferReprojected, bool forceKnown) const
{
    if (preferReprojected)
    {
        if (hasKnownReprojectedPosition || forceKnown)
            return reprojectedPosition;

        if (hasKnownPosition || forceKnown)
            return position;
    }
    else
    {
        if (hasKnownPosition || forceKnown)
            return position;

        if(hasKnownReprojectedPosition || forceKnown)
            return reprojectedPosition;
    }
    return position;
}

} //namespace corecvs
