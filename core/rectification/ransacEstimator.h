/**
 * \file ransacEstimator.h
 * \brief a header for ransacEstimator.c
 *
 * \date Nov 14, 2011
 * \author alexander
 */
#ifndef RANSACESTIMATOR_H_
#define RANSACESTIMATOR_H_

#include <vector>

#include "matrix33.h"
#include "correspondenceList.h"
#include "ransac.h"
#include "fixtureScene.h"

namespace corecvs {

class RansacEstimator
{
public:
    unsigned trySize;
    RansacParameters ransacParams;


    RansacEstimator(
            unsigned  _trySize,
            unsigned _maxIterations,
            double _treshold ) :
        trySize(_trySize)
    {
        ransacParams.setIterationsNumber(_maxIterations);
        ransacParams.setInlierThreshold(_treshold);
    }

    Matrix33 getFundamentalRansac1(CorrespondenceList *list);
    Matrix33 getEssentialRansac1  (CorrespondenceList *list);

    Matrix33 getFundamentalRansac(vector<Correspondence *> *data);
    Matrix33 getEssentialRansac  (vector<Correspondence *> *data);
};

class RansacEstimatorScene
{
public:
    RansacParameters params;
    bool trace = true;

    EssentialDecomposition getEssentialRansac(FixtureScene *scene, FixtureCamera *cam1, FixtureCamera *cam2);
    //EssentialDecomposition getEssentialRansac(FixtureScene *scene, FixtureCamera *cam1, FixtureCamera *cam2);

};



} //namespace corecvs
#endif  //RANSACESTIMATOR_H_



