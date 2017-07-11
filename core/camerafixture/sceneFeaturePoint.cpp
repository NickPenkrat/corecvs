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

bool SceneFeaturePoint::checkTriangulationAngle(const corecvs::Vector3dd& pointPosition, bool use__)
{
    //double angle = 1.0;
    bool res = false;
    if (use__)
    {
        for (auto& obs0 : observations__)
        {
            Vector3dd c0_location = obs0.first.v->getWorldLocation().shift - pointPosition;
            for (auto& obs1 : observations__)
            {
                if (obs0.first == obs1.first)
                    continue;

                Vector3dd c1_location = obs1.first.v->getWorldLocation().shift - pointPosition;
                double cos_ = (c0_location & c1_location) / (c0_location.l2Metric() * c1_location.l2Metric());
                //angle = std::min( angle, cos_ );
                if (cos_ < 0.9993) // > ~2.0 degree
                {
                    res = true;
                    break;
                }
            }

            if (res)
                break;
        }
    }
    else
    {
        for (auto& obs0 : observations)
        {
            Vector3dd c0_location = obs0.first->getWorldLocation().shift - pointPosition;
            for (auto& obs1 : observations)
            {
                if (obs0.first == obs1.first)
                    continue;

                Vector3dd c1_location = obs1.first->getWorldLocation().shift - pointPosition;
                double cos_ = (c0_location & c1_location) / (c0_location.l2Metric() * c1_location.l2Metric());
                //angle = std::min( angle, cos_ );
                if (cos_ < 0.9993) // > ~2.0 degree
                {
                    res = true;
                    break;
                }
            }

            if (res)
                break;
        }
    }

    //angle = radToDeg( std::acos( angle ) );
    return res;
}

Vector3dd SceneFeaturePoint::triangulate(bool use__, std::vector<int> *mask, bool* succeeded, bool checkMinimalAngle)
{
    MulticameraTriangulator mct;
    int id = 0;
    size_t ptr = 0;

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
        for (auto& pos : observations)
        {
            const SceneObservation &obs = pos.second;

            //cout << "SceneFeaturePoint::triangulate(" << name << ") distorted:" << obs.onDistorted << " " << obs.observation << endl;

            CORE_ASSERT_TRUE_S(obs.cameraFixture != NULL);
            if (!mask || (ptr < mask->size() && (*mask)[ptr] == id))
            {
                mct.addCamera(obs.cameraFixture->getMMatrix(obs.camera), obs.observation);
                if (mask && ptr + 1 < mask->size())
                    CORE_ASSERT_TRUE_S((*mask)[ptr] < (*mask)[ptr + 1]);
                ptr++;
            }
            if (mask && ptr == mask->size())
                break;
            ++id;
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

    if (checkMinimalAngle && ok)
    {
        ok = checkTriangulationAngle(res, use__);
        //if (!ok)
        //{
        //    res = Vector3dd(0.0);
        //}
    }

    if (succeeded != nullptr)
        *succeeded = ok;

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

SceneFeaturePoint *FixtureSceneGeometry::getPointById(FixtureScenePart::IdType id)
{
    return ownerScene->getPointById(id);
}

std::vector< double > SceneFeaturePoint::estimateReconstructedReprojectionErrorL2()
{
    std::vector< double > out;
    if ( !hasKnownReprojectedPosition )
        return out;

    for ( auto& obs : observations )
    { 
        Vector2dd xy;
        if ( !obs.first->projectPointFromWorld( reprojectedPosition, &xy ) )
            continue;

        xy -= obs.second.observation;
        out.push_back( xy.l2Metric() );
    }
        
    return out;
}

std::vector< double > SceneFeaturePoint::estimateReprojectionErrorL2()
{
    std::vector< double > out;
    if (!hasKnownPosition)
        return out;

    for ( auto& obs : observations )
    {
        Vector2dd xy;
        if ( !obs.first->projectPointFromWorld( position, &xy ) )
            continue;

        xy -= obs.second.observation;
        out.push_back( xy.l2Metric() );
    }

    return out;
}

PointPath SceneFeaturePoint::getEpipath(FixtureCamera *camera1, FixtureCamera *camera2, int segments)
{
    PointPath result;

    CameraModel secondCamera = camera2->getWorldCameraModel();
    ConvexPolyhedron secondViewport = secondCamera.getCameraViewport();

    SceneObservation *obs1 = getObservation(camera1);

    if (obs1 == NULL) {
        return result;
    }

    Vector2dd m = obs1->getDistorted(false); /*We work with geometry, so we take projective undistorted objservation */

    CameraModel model = camera1->getWorldCameraModel();
    Ray3d ray = model.rayFromPixel(m);

    //cout << "Ray" << ray << endl;

    /* We go with ray analysis instead of essential matrix beacause it possibly gives
     * more semanticly valuable info
	 */
    double t1 = 0;
    double t2 = 0;
    bool hasIntersection = secondViewport.intersectWith(ray, t1, t2);
    if (hasIntersection)
	{
        if (t1 < 0.0) t1 = 0.0;

        //cout << "t1 : " << t1 << " t2 : " << t2 << endl;

        FixedVector<double, 4> out1 = (secondCamera.getCameraMatrix() * ray.getProjectivePoint(t1));
        FixedVector<double, 4> out2 = (secondCamera.getCameraMatrix() * ray.getProjectivePoint(t2));

        //cout << "out1: " << out1 << endl;
        //cout << "out2: " << out2 << endl;

        Vector2dd pos1 = Vector2dd( out1[0], out1[1]) / out1[2];
        Vector2dd pos2 = Vector2dd( out2[0], out2[1]) / out2[2];

        //cout << "pos1:" << pos1 << endl;
        //cout << "pos2:" << pos2 << endl;

        /* We should apply distorion here */

        for (int segm = 0; segm <= segments; segm++)
        {
            Vector2dd ssP = lerp(pos1, pos2, segm    , 0, segments);
            Vector2dd ssI = secondCamera.distortion.mapForward(ssP);

            //cout << "Undistortion: " << ssP << " - " << ssI << endl;
            //cout << "Undistortion: " << seP << " - " << seI << endl;

            result.push_back(ssI);
        }

    }

    return result;
}

/*
SceneFeaturePoint *FixtureSceneGeometry::getPointById(FixtureScenePart::IdType id)
{
    CORE_ASSERT_TRUE_S(featurePoint);
    CORE_ASSERT_TRUE_S(featurePoint->ownerScene);

    return featurePoint->ownerScene->getCameraById(id);
}
*/

} //namespace corecvs
