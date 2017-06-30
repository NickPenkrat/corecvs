#ifndef ITERATIVEESTIMATOR_H_
#define ITERATIVEESTIMATOR_H_
/**
 * \file iterativeEstimator.h
 * \brief Add Comment Here
 *
 * \date Oct 26, 2011
 * \author alexander
 */

#include "essentialEstimator.h"
#include "iterativeEstimateParameters.h"

namespace corecvs {

class IterativeEstimator
{
public:

    IterativeEstimateParameters params;
    EssentialEstimator::OptimisationMethod method;

    IterativeEstimator(
            int _maxIterations,
            double _initialSigma,
            double _sigmaFactor,
            EssentialEstimator::OptimisationMethod _method = EssentialEstimator::METHOD_DEFAULT
    ) :
        method(_method)
    {
        params.setIterationsNumber(_maxIterations);
        params.setInitialSigma(_initialSigma);
        params.setSigmaFactor(_sigmaFactor);

    }

    EssentialMatrix getEssential   (const vector<Correspondence *> &samples);

    virtual ~IterativeEstimator();
};


} //namespace corecvs
#endif /* ITERATIVEESTIMATOR_H_ */

