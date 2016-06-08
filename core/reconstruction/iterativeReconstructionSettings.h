#ifndef ITERATIVE_RECONSTRUCTION_SETTINGS
#define ITERATIVE_RECONSTRUCTION_SETTINGS

#include "reconstructionFunctor.h"
#include "reconstructionFixtureScene.h"

#include "generated/iterativeReconstructionInitializationParams.h"
#include "generated/iterativeReconstructionFeatureSelectionParams.h"
#include "generated/iterativeReconstructionAppendParams.h"

namespace corecvs
{
/*
 * Here we store all params for
 * all stages of iterative reconstruction
 */

struct IterativeReconstructionNonlinearOptimizationParams
{
    // By default only orientations and reconstructed points are subject for optimization
    ReconstructionFunctorOptimizationType optimizationParams =
        ReconstructionFunctorOptimizationType::NON_DEGENERATE_ORIENTATIONS | ReconstructionFunctorOptimizationType::DEGENERATE_ORIENTATIONS | ReconstructionFunctorOptimizationType::FOCALS | ReconstructionFunctorOptimizationType::PRINCIPALS |
        ReconstructionFunctorOptimizationType::POINTS;
    // Default error type is ray difference, but 'cause of
    // cool closed-form solvers it does not make sense at all
    ReconstructionFunctorOptimizationErrorType::ReconstructionFunctorOptimizationErrorType errorType = ReconstructionFunctorOptimizationErrorType::RAY_DIFF;
    // Post-append non-linear optimization iterations
    int postAppendNonlinearIterations = 4000;
    // Final non-linear iterations
    int finalNonLinearIterations = 40000;
    // Alternating optimization on success steps
    int alternatingIterations = 20;
    // Excessive/non-excessive quaternion parametrization
    bool excessiveQuaternionParametrization = true;

    template<class VisitorType>
    void accept(VisitorType &visitor)
    {
        visitor.visit(postAppendNonlinearIterations , 200, "postAppendNonlinearIterations");
        visitor.visit(errorType, ReconstructionFunctorOptimizationType::ReconstructionFunctorOptimizationErrorType::RAY_DIFF, "errorType");
        visitor.visit(finalNonLinearIterations , 2000,     "finalNonLinearIterations");
        visitor.visit(alternatingIterations , 20,"alternatingIterations");
        visitor.visit(excessiveQuaternionParametrization , true,"excessiveQuaternionParametrization");
        visitor.visit(optimizationParams, ReconstructionFunctorOptimizationType::NON_DEGENERATE_ORIENTATIONS | ReconstructionFunctorOptimizationType::DEGENERATE_ORIENTATIONS | ReconstructionFunctorOptimizationType::FOCALS | ReconstructionFunctorOptimizationType::PRINCIPALS | ReconstructionFunctorOptimizationType::POINTS, "optimizationParams");

    }
};

}


#endif
