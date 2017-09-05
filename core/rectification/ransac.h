#ifndef RANSAC_H_
#define RANSAC_H_
/**
 * \file ransac.h
 * \brief a header that holds generic RANSAC implementation
 *
 * \date Jul 2, 2011
 * \author alexander
 */

#include <random>
#include <vector>
#include <algorithm>

#include "global.h"
#include "ransacParameters.h"

namespace corecvs {

using std::vector;
using std::find;

/**
 *  This template class is used to implement classic RANSAC algorithm is a generic form
 *
 *
 *
 **/
#if 0
class RansacParameters {
public:
    int iterationsNumber;
    double inliersPercent;
    double inlierThreshold;
};
#endif

template<typename SampleType, typename ModelType>
class Ransac : public RansacParameters {
public:
    vector<SampleType *> *data;
    int dataLen;

    vector<SampleType *> samples;
    int sampleNumber;

    int iteration;
    vector<SampleType *> bestSamples;
    ModelType bestModel;
    int bestInliers;

    bool trace = false;

    /* */
    std::mt19937 rng;

    Ransac(int _sampleNumber, const RansacParameters &params = RansacParameters())
        : RansacParameters(params)
        , sampleNumber(_sampleNumber)
    {
        samples.reserve(sampleNumber);
        rng.seed();
    }

    virtual void randomSelect()
    {        
        std::uniform_int_distribution<int> uniform(0, (int)data->size() - 1);
        samples.clear();
        for (int i = 0; i < sampleNumber; i++)
        {
            unsigned index;
            SampleType *element = NULL;
            do {
                index = uniform(rng);
                element = data->at(index);

                if (find(samples.begin(), samples.end(), element) == samples.end())
                    break;
            } while (true);

            samples.push_back(element);
        }
    }

    ModelType getModelRansac()
    {
        bestInliers = 0;
        iteration = 0;

        while (true)
        {
            randomSelect();
            ModelType model = ModelType(samples);

            int inliers = 0;
            for (size_t i = 0; i < data->size(); i++)
            {
                if (model.fits(*(data->at(i)), inlierThreshold()))
                    inliers++;
            }

            if (trace) SYNC_PRINT(("iteration %d : %d inliers (max so far %d) out of %d (%lf%%)\n",
                                   iteration, inliers, bestInliers, (int)data->size(), (double)100.0 * bestInliers / data->size() ));

            if (inliers > bestInliers)
            {
                bestSamples = samples;
                bestInliers = inliers;
                bestModel = model;
            }

            if (bestInliers >  data->size() * inliersPercent() / 100.0 ||
                iteration >= iterationsNumber() )
            {
                if (trace) {
                    std::cout << "Fininshing:" << std::endl;
                    std::cout << "BestInliers:" << bestInliers << std::endl;
                }
                return bestModel;
            }
            iteration++;
        }
    }

    ModelType getModelRansacMultimodel()
    {
        bestInliers = 0;
        iteration = 0;

        while (true)
        {
            randomSelect();
            vector<ModelType> models = ModelType::getModels(samples);

            for (ModelType &model : models)
            {
                int inliers = 0;

                for (size_t i = 0; i < data->size(); i++)
                {
                    if (model.fits(*(data->at(i)), inlierThreshold()))
                        inliers++;
                }

                if (trace) SYNC_PRINT(("iteration %d : %d inliers (max so far %d) out of %d (%lf%%)\n",
                                       iteration, inliers, bestInliers, data->size(), (double)100.0 * bestInliers / data->size() ));

                if (inliers > bestInliers)
                {
                    bestSamples = samples;
                    bestInliers = inliers;
                    bestModel = model;
                }

                if (bestInliers >  data->size() * inliersPercent() / 100.0 ||
                    iteration >= iterationsNumber() )
                {
                    if (trace) {
                        std::cout << "Fininshing:" << std::endl;
                        std::cout << "BestInliers:" << bestInliers << std::endl;
                    }
                    return bestModel;
                }
            }
            iteration++;
        }
    }

    ~Ransac()
    {}
};

} //namespace corecvs

#endif  //RANSAC_H_
