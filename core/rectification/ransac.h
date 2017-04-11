#ifndef RANSAC_H_
#define RANSAC_H_
/**
 * \file ransac.h
 * \brief a header that holds generic RANSAC implementation
 *
 * \date Jul 2, 2011
 * \author alexander
 */

#include <vector>
#include <algorithm>

#include "global.h"
namespace corecvs {

using std::vector;
using std::find;

/**
 *  This template class is used to implement classic RANSAC algorithm is a generic form
 *
 *
 *
 **/
class RansacParameters {
public:
    int iterationsNumber;
    double inliersPercent;
    double inlierThreshold;
};

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

    Ransac(int _sampleNumber ) :
        sampleNumber(_sampleNumber)
    {
        samples.reserve(sampleNumber);
    }

    virtual void randomSelect ()
    {
        samples.clear();
        for (int i = 0; i < sampleNumber; i++)
        {
            unsigned index;
            SampleType *element = NULL;
            do {
                index = (rand() % data->size());
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
            for (int i = 0; i < data->size(); i++)
            {
                if (model.fits(*(data->at(i)), inlierThreshold))
                    inliers++;
            }

            if (trace) SYNC_PRINT(("iteration %d : %d inliers \n", iteration, inliers));

            if (inliers > bestInliers)
            {
                bestSamples = samples;
                bestInliers = inliers;
                bestModel = model;
            }

            if (bestInliers >  data->size() * inliersPercent ||
                iteration >= iterationsNumber )
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

    ~Ransac()
    {}
};



} //namespace corecvs
#endif  //RANSAC_H_



