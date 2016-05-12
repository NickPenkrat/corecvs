#ifndef ITERATIVE_RECONSTRUCTION_SETTINGS
#define ITERATIVE_RECONSTRUCTION_SETTINGS

#include "reconstructionFunctor.h"
#include "reconstructionFixtureScene.h"

namespace corecvs
{
/*
 * Here we store all params for
 * all stages of iterative reconstruction
 */

// This block covers reconstruction initialization
struct IterativeReconstructionInitializationParams
{
    // b2b threshold for feature matches in 5PT->EM problem
    double b2bRansacP5RPThreshold = 0.8;
    // 2d pairwise correspondence inlier threshold
    double inlierP5RPThreshold = 5.0;
    // Maximal number of ransac iterations for essential matrix estimation
    int maxEssentialRansacIterations = 600000;
    // b2b threshold for feature matches in 6PT->pose problem for generalized cameras
    double b2bRansacP6RPThreshold = 0.8;
    // Turns on 5PT->EM-filtering prior 6PT->pose
    bool runEssentialFiltering = true;
    // target error probability for essential matrix estimation
    double essentialTargetGamma = 0.01;

    // Maximal number of ransac iterations for pose estimation using pairwise correspondences
    int maxP6RPIterations = 400000;
    // Inlier threshold for pose estimation using pairwise correspondences
    double inlierP6RPThreshold = 1.0;
    // Target failure probability for pose estimation
    double gammaP6RP = 0.001;
};

// This block covers point track construction and feature point detection
struct IterativeReconstructionFeatureSelectionParams
{
    // Threshold for static point initialization
    double inlierThreshold = 5.0;
    // Threshold for reprojection error of track inlier
    double trackInlierThreshold = 3;
    // Threshold for accepting reconstructed point
    double distanceLimit =1000.0;
    // Feature detection params
    FeatureDetectionParams featureDetectionParams;
    // Track pruning threshold
    double trackPruningThreshold = 2.8;
};

// This block covers iterative appending
struct IterativeReconstructionAppendParams
{
    // Maximal post-append iterations
    int maxPostAppend = 2;
    // Inlier threshold for 3P->pose
    double inlierP3PThreshold = 2.0;
    // Maximal ransac iterations for 3P-> pose
    int maxP3PIterations = 100000;
    // Target error probability for 3P pose estimation
    double gammaP3P = 0.001;
    // Inlier threshold for 6P->pose/3P->orientation
    double inlierP6PThreshold = 1.0;
    // Maximal ransac iterations for 6P->pose/3P->orientation
    int maxP6PIterations = 400000;
    // Target error probability for 6P->pose/3P->orientation
    double gammaP6P = 0.001;
    // This defines how many multicameras are subject for P3P evaluation at each iteration
    size_t speculativity = 1000;
    // Minimal inlier count for hypotheis acceptance
    size_t minimalInlierCount = 32;
    // Maximal failure probability for hypothesis acceptance
    double maximalFailureProbability = 0.15;
};

// And, finally, non-linear optimization params
struct IterativeReconstructionNonlinearOptimizationParams
{
    // By default only orientations and reconstructed points are subject for optimization
    ReconstructionFunctorOptimizationType optimizationParams =
        ReconstructionFunctorOptimizationType::NON_DEGENERATE_ORIENTATIONS | ReconstructionFunctorOptimizationType::DEGENERATE_ORIENTATIONS | ReconstructionFunctorOptimizationType::FOCALS | ReconstructionFunctorOptimizationType::PRINCIPALS |
        ReconstructionFunctorOptimizationType::POINTS;
    // Default error type is ray difference, but 'cause of
    // cool closed-form solvers it does not make sense at all
    ReconstructionFunctorOptimizationErrorType errorType = ReconstructionFunctorOptimizationErrorType::RAY_DIFF;
    // Post-append non-linear optimization iterations
    int postAppendNonlinearIterations = 200;
    // Final non-linear iterations
    int finalNonLinearIterations = 2000;
};

}


#endif
