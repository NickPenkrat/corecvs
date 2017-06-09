/**
 * \file ransacEstimator.cpp
 * \brief Add Comment Here
 *
 * \date Nov 14, 2011
 * \author alexander
 */

#include <vector>

#include "global.h"

#include "ransacEstimator.h"
#include "essentialMatrix.h"
#include "essentialEstimator.h"
#include "ransac.h"
#include "correspondenceList.h"
namespace corecvs {




class Model8Point {
public:
    EssentialMatrix model;

    Model8Point()
        : model(Matrix33(1.0))
    {

    }

    Model8Point(const vector<Correspondence *> &samples)
        : model(Matrix33(1.0))
    {
        EssentialEstimator estimator;
        model = estimator.getEssential(samples, EssentialEstimator::METHOD_SVD_LSE);
    }

    double getCost(const Correspondence &data)
    {
        return (model.epipolarDistance(data));
    }

    bool fits(const Correspondence &data, double fitTreshold)
    {
        return (model.epipolarDistance(data) < fitTreshold);
    }
};

class Model5Point {
public:
    EssentialMatrix model;

    Model5Point(const EssentialMatrix &m) :
        model(m)
    {
    }

    Model5Point()
    {
        model = Matrix33(1.0);
    }

    static vector<Model5Point> getModels(const vector<Correspondence *> &samples)
    {
        EssentialEstimator estimator;
        vector<EssentialMatrix> ms = estimator.getEssential5point(samples);
        vector<Model5Point> result;
        result.reserve(ms.size());
        for ( EssentialMatrix &m: ms)
        {
            result.push_back(m);
        }
        return result;

    }

    double getCost(const Correspondence &data)
    {
        return (model.epipolarDistance(data));
    }

    bool fits(const Correspondence &data, double fitTreshold)
    {
        return (model.epipolarDistance(data) < fitTreshold);
    }
};


class ModelFundamental8Point : public Model8Point
{
public:
    ModelFundamental8Point() : Model8Point() {}
    ModelFundamental8Point(const vector<Correspondence *> &samples) :
                Model8Point(samples)
    {
        model.assertRank2();
    }
};

class ModelEssential8Point : public Model8Point
{
public:
    ModelEssential8Point() : Model8Point() {}
    ModelEssential8Point(const vector<Correspondence *> &samples) :
                Model8Point(samples)
    {
        model.projectToEssential();
    }
};


Matrix33 RansacEstimator::getFundamentalRansac1(CorrespondenceList *list)
{
    vector<Correspondence *> data;
    data.reserve(list->size());
    for(unsigned i = 0; i < list->size(); i++)
        data.push_back(&(list->at(i)));

    return getFundamentalRansac(&data);
}

Matrix33 RansacEstimator::getFundamentalRansac(vector<Correspondence *> *data)
{
    Ransac<Correspondence, ModelFundamental8Point>
        ransac(trySize, ransacParams);

    ransac.data = data;

    ModelFundamental8Point result = ransac.getModelRansac();
    for (unsigned i = 0; i < data->size(); i++)
    {
        bool fits = result.fits(*(data->operator[](i)), ransacParams.inlierThreshold());
        data->operator[](i)->flags |= (fits ? Correspondence::FLAG_PASSER : Correspondence::FLAG_FAILER);
    }
    for (unsigned i = 0; i < ransac.bestSamples.size(); i++)
    {
        ransac.bestSamples[i]->flags  |= Correspondence::FLAG_IS_BASED_ON;
    }

    return result.model;
}



Matrix33 RansacEstimator::getEssentialRansac1(CorrespondenceList *list)
{

    vector<Correspondence *> data;
    data.reserve(list->size());
    for(unsigned i = 0; i < list->size(); i++)
        data.push_back(&(list->at(i)));

    return getEssentialRansac(&data);
}

Matrix33 RansacEstimator::getEssentialRansac(vector<Correspondence *> *data)
{
    Ransac<Correspondence, ModelEssential8Point>
        ransac(trySize, ransacParams);

    ransac.data = data;

    ModelEssential8Point result = ransac.getModelRansac();
    for (unsigned i = 0; i < data->size(); i++)
    {
        bool fits = result.fits(*(data->operator[](i)), ransacParams.inlierThreshold());
        data->operator[](i)->flags |= (fits ? Correspondence::FLAG_PASSER : Correspondence::FLAG_FAILER);
    }

    for (unsigned i = 0; i < ransac.bestSamples.size(); i++)
    {
        ransac.bestSamples[i]->flags  |= Correspondence::FLAG_IS_BASED_ON;
    }

    return result.model;
}

EssentialDecomposition RansacEstimatorScene::getEssentialRansac(FixtureScene *scene, FixtureCamera *camera1, FixtureCamera *camera2)
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

    EssentialMatrix model;

#if 0
    Ransac<Correspondence, Model5Point>  ransac(5, params);
    ransac.trace = trace;
    ransac.data = &dataPtr;
    Model5Point result = ransac.getModelRansacMultimodel();    
    model = result.model;
#endif
    Ransac<Correspondence, Model8Point>  ransac(8, params);
    ransac.trace = trace;
    ransac.data = &dataPtr;
    Model8Point result = ransac.getModelRansac();

    model = result.model;

    cout << "Model as a result" << model << endl;

    int count = 0;
    for(SceneFeaturePoint *point: scene->featurePoints())
    {
        SceneObservation *obs1 = point->getObservation(camera1);
        SceneObservation *obs2 = point->getObservation(camera2);

        if (obs1 != NULL && obs2 != NULL)
        {
//            Correspondence::CorrespondenceFlags flag = (Correspondence::CorrespondenceFlags)data[count].flags;

            double cost = result.getCost(data[count]);
            obs1->accuracy = Vector2dd(cost / params.inlierThreshold());
#if 0
            if (flag & Correspondence::FLAG_PASSER) {
                obs1->accuracy = Vector2dd(0.5);
                obs2->accuracy = Vector2dd(0.5);
            }

            if (flag & Correspondence::FLAG_FAILER) {
                obs1->accuracy = Vector2dd(1.0);
                obs2->accuracy = Vector2dd(1.0);
            }

            if (flag & Correspondence::FLAG_IS_BASED_ON) {
                obs1->accuracy = Vector2dd(0.0);
                obs2->accuracy = Vector2dd(0.0);
            }
#endif
            count++;
        }
    }


    EssentialDecomposition variants[4];
    EssentialDecomposition dec = model.decompose(&dataPtr, variants);

    return dec;
}


} //namespace corecvs

