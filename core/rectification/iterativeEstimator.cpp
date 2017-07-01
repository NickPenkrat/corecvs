/**
 * \file iterativeEstimator.cpp
 * \brief Add Comment Here
 *
 * \date Oct 26, 2011
 * \author alexander
 */

#include "global.h"
#include "log.h"

#include "iterativeEstimator.h"
namespace corecvs {


EssentialMatrix IterativeEstimator::getEssential (const vector<Correspondence *> &samples)
{
    EssentialMatrix model;
    int iteration;
    vector<Correspondence *> workingSamples = samples;

    EssentialEstimator::CostFunction7to1 initialCost(&samples);

    L_INFO_P("Starting iterative rectification...");

    double sigma = params.initialSigma();
    for (iteration = 0; iteration < params.iterationsNumber(); iteration++)
    {
        EssentialEstimator estimator;
        if (workingSamples.size() < 9)
        {
            cout << "Too few inputs to continue:" << workingSamples.size() << endl;
            break;
        }
        model = estimator.getEssential(workingSamples, method);
        EssentialEstimator::CostFunction7to1 cost(&workingSamples);

        int passed = 0;
        int rejected = 0;

        vector<Correspondence *> passedSamples;
        for (unsigned i = 0; i < workingSamples.size(); i++)
        {
            Correspondence *corr = workingSamples[i];
            double value = model.epipolarDistance(*corr);
            if (value > sigma)
            {
                rejected++;
                corr->value = iteration;
            } else {
                passed++;
                passedSamples.push_back(corr);
            }
        }

        double errorPerPoint     = sqrt(cost.getCost(model) / workingSamples.size());
        double errorPerPointInit = sqrt(initialCost.getCost(model) / samples.size());

#ifdef TRACE
        cout << "Iteration: " << iteration << endl;
        cout << "Error:" << cost.getCost(model) << endl;
        cout << "Sq per point:" << (cost.getCost(model) / workingSamples.size()) << endl;
        cout << "Error per point:" << errorPerPoint << endl;

        cout << " Total:" << workingSamples.size() << endl;
        cout << "   Passed  :" << passed   << endl;
        cout << "   Rejected:" << rejected << endl;
#endif
        //printf("Iteration %4d: %6d %6d %6d %lf %lf \n", iteration, passed + rejected, passed, rejected, errorPerPoint, errorPerPointInit);
        // Make real callback
        L_INFO_P("Iteration %4d: %6d %6d %6d %lf %lf"
                 , iteration, passed + rejected, passed, rejected, errorPerPoint, errorPerPointInit);

        workingSamples = passedSamples;
        sigma *= params.sigmaFactor();
    }

    for (unsigned i = 0; i < samples.size(); i++)
    {
        samples[i]->flags |= Correspondence::FLAG_FAILER;
    }

    for (unsigned i = 0; i < workingSamples.size(); i++)
    {
        workingSamples[i]->flags &= ~Correspondence::FLAG_FAILER;
        workingSamples[i]->flags |=  Correspondence::FLAG_PASSER;
        workingSamples[i]->value = iteration;
    }

    L_INFO_P("Iterative rectification finished");

    return model;
}


IterativeEstimator::~IterativeEstimator()
{
    // TODO Auto-generated destructor stub
}

EssentialDecomposition IterativeEstimatorScene::getEssentialIterative(FixtureScene *scene, FixtureCamera *camera1, FixtureCamera *camera2)
{
    vector<Correspondence > data;
    vector<Correspondence *> dataPtr;

    //CameraModel cam1world = camera1->getWorldCameraModel();
    //CameraModel cam2world = camera2->getWorldCameraModel();


    for(SceneFeaturePoint *point: scene->featurePoints())
    {
        SceneObservation *obs1 = point->getObservation(camera1);
        SceneObservation *obs2 = point->getObservation(camera2);

        if (obs1 != NULL && obs2 != NULL)
        {
            Correspondence c;
            Vector2dd startPixel = obs1->getDistorted(false);
            Vector2dd endPixel   = obs2->getDistorted(false);

            c.start = camera1->intrinsics.reverse(startPixel).xy();
            c.end   = camera2->intrinsics.reverse(endPixel  ).xy();

            data.push_back(c);
            dataPtr.push_back(&data.back());
        }
    }

    double thresholdScaler = camera1->intrinsics.fx();

    IterativeEstimator estimator;
    estimator.params = params;
    estimator.method = EssentialEstimator::METHOD_DEFAULT;
    EssentialMatrix model = estimator.getEssential(dataPtr);

    int count = 0;
    for(SceneFeaturePoint *point: scene->featurePoints())
    {
        SceneObservation *obs1 = point->getObservation(camera1);
        SceneObservation *obs2 = point->getObservation(camera2);

        if (obs1 != NULL && obs2 != NULL)
        {
//            Correspondence::CorrespondenceFlags flag = (Correspondence::CorrespondenceFlags)data[count].flags;

            double cost = model.epipolarDistance(data[count]);
            Vector2dd accuracy(cost * thresholdScaler / params.initialSigma(), cost * thresholdScaler);

            obs1->accuracy = accuracy;
            obs2->accuracy = accuracy;
            count++;
        }
    }


    EssentialDecomposition variants[4];
    EssentialDecomposition dec = model.decompose(&dataPtr, variants);
    return dec;
}


} //namespace corecvs

