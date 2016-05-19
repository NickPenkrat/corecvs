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

    template<class VisitorType>
        void accept(VisitorType &visitor)
        {
            visitor.visit(b2bRansacP5RPThreshold       ,0.8    ,"b2bRansacP5RPThreshold"         );
            visitor.visit(inlierP5RPThreshold          ,5.0    ,"inlierP5RPThreshold"            );
            visitor.visit(maxEssentialRansacIterations ,600000 ,"maxEssentialRansacIterations"   );
            visitor.visit(b2bRansacP6RPThreshold       ,0.8    ,"b2bRansacP6RPThreshold"         );
            visitor.visit(runEssentialFiltering        ,true   ,"runEssentialFiltering"          );
            visitor.visit(essentialTargetGamma         ,0.01   ,"essentialTargetGamma"           );
            visitor.visit(maxP6RPIterations            ,400000 ,"maxP6RPIterations"              );
            visitor.visit(inlierP6RPThreshold            ,1.0    ,"inlierP6RPThreshold"            );
            visitor.visit(gammaP6RP                    ,0.001  ,"gammaP6RP"                      );
        }
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
    // Scaler for pruning
    double rmsePruningScaler = 3.0;
    // Scaler for pruning
    double maxPruningScaler = 5.0;

    template<class VisitorType>
        void accept(VisitorType &visitor)
        {
            visitor.visit(inlierThreshold      ,5.0   ,"inlierThreshold"          );
            visitor.visit(trackInlierThreshold ,3.0   ,"trackInlierThreshold"          );
            visitor.visit(distanceLimit        ,1000.0   ,"distanceLimit"          );
            visitor.visit(rmsePruningScaler    ,3.0   ,"rmsePruningScaler"          );
            visitor.visit(maxPruningScaler     ,5.0   ,"maxPruningScaler"          );

            visitor.visit(featureDetectionParams        ,FeatureDetectionParams()   ,"featureDetectionParams"          );
        }
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

    template<class VisitorType>
        void accept(VisitorType &visitor)
        {
            visitor.visit(maxPostAppend , 2,"maxPostAppend");
            visitor.visit(inlierP3PThreshold , 2.0,"inlierP3PThreshold");
            visitor.visit(maxP3PIterations , 100000,"maxP3PIterations");
            visitor.visit(gammaP3P , 0.001,"gammaP3P");
            visitor.visit(inlierP6PThreshold , 1.0,"inlierP6PThreshold");
            visitor.visit(maxP6PIterations , 400000,"maxP6PIterations");
            visitor.visit(gammaP6P , 0.001,"gammaP6P");
            visitor.visit(speculativity , (size_t)1000,"speculativity");
            visitor.visit(minimalInlierCount , (size_t)32,"minimalInlierCount");
            visitor.visit(maximalFailureProbability , 0.15,"maximalFailureProbability");
        }
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
    // Alternating optimization on success steps
    int alternatingIterations = 20;
    // Excessive/non-excessive quaternion parametrization
    bool excessiveQuaternionParametrization = true;

    template<class VisitorType>
        void accept(VisitorType &visitor)
        {
            visitor.visit(postAppendNonlinearIterations , 200, "postAppendNonlinearIterations");
            visitor.visit(finalNonLinearIterations , 2000,     "finalNonLinearIterations");
            visitor.visit(alternatingIterations , 20,"alternatingIterations");
            visitor.visit(excessiveQuaternionParametrization , true,"excessiveQuaternionParametrization");
            visitor.visit(optimizationParams, ReconstructionFunctorOptimizationType::NON_DEGENERATE_ORIENTATIONS | ReconstructionFunctorOptimizationType::DEGENERATE_ORIENTATIONS | ReconstructionFunctorOptimizationType::FOCALS | ReconstructionFunctorOptimizationType::PRINCIPALS | ReconstructionFunctorOptimizationType::POINTS, "optimizationParams");

        }

};

}


#endif