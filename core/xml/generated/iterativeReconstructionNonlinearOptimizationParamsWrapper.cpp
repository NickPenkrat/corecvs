/**
 * \file iterativeReconstructionNonlinearOptimizationParamsWrapper.cpp
 * \attention This file is automatically generated and should not be in general modified manually
 *
 * \date MMM DD, 20YY
 * \author autoGenerator
 */

#include <vector>
#include <stddef.h>
#include "iterativeReconstructionNonlinearOptimizationParamsWrapper.h"

/**
 *  Looks extremely unsafe because it depends on the order of static initialization.
 *  Should check standard if this is ok
 *
 *  Also it's not clear why removing "= Reflection()" breaks the code;
 **/

namespace corecvs {
template<>
Reflection BaseReflection<IterativeReconstructionNonlinearOptimizationParamsWrapper>::reflection = Reflection();
template<>
int BaseReflection<IterativeReconstructionNonlinearOptimizationParamsWrapper>::dummy = IterativeReconstructionNonlinearOptimizationParamsWrapper::staticInit();
} // namespace corecvs 

SUPPRESS_OFFSET_WARNING_BEGIN

int IterativeReconstructionNonlinearOptimizationParamsWrapper::staticInit()
{

    ReflectionNaming &nameing = naming();
    nameing = ReflectionNaming(
        "Iterative Reconstruction Nonlinear Optimization Params Wrapper",
        "Iterative Reconstruction Nonlinear Optimization Params Wrapper",
        ""
    );
     

    CompositeField* field0 = new CompositeField
        (
          IterativeReconstructionNonlinearOptimizationParamsWrapper::OPTIMIZATIONPARAMS_ID,
          offsetof(IterativeReconstructionNonlinearOptimizationParamsWrapper, mOptimizationParams),
          "optimizationParams",
          "ReconstructionFunctorOptimizationParams",
          "optimizationParams",
          "optimizationParams",
           NULL
        );
    field0->precision=-1;
    fields().push_back(field0);
    /*  */ 
    EnumField* field1 = new EnumField
        (
          IterativeReconstructionNonlinearOptimizationParamsWrapper::ERRORTYPE_ID,
          offsetof(IterativeReconstructionNonlinearOptimizationParamsWrapper, mErrorType),
          3,
          "errorType",
          "errorType",
          "Functor optimizattion error type",
          new EnumReflection(4
          , new EnumOption(0,"REPROJECTION")
          , new EnumOption(1,"ANGULAR")
          , new EnumOption(2,"CROSS_PRODUCT")
          , new EnumOption(3,"RAY_DIFF")
          )
        );
    field1->widgetHint=BaseField::COMBO_BOX;
    field1->precision=-1;
    fields().push_back(field1);
    /*  */ 
    IntField* field2 = new IntField
        (
          IterativeReconstructionNonlinearOptimizationParamsWrapper::POSTAPPENDNONLINEARITERATIONS_ID,
          offsetof(IterativeReconstructionNonlinearOptimizationParamsWrapper, mPostAppendNonlinearIterations),
          80,
          "postAppendNonlinearIterations",
          "postAppendNonlinearIterations",
          "Post-append non-linear optimization iterations",
          true,
         1,
         100000
        );
    field2->precision=-1;
    fields().push_back(field2);
    /*  */ 
    IntField* field3 = new IntField
        (
          IterativeReconstructionNonlinearOptimizationParamsWrapper::FINALNONLINEARITERATIONS_ID,
          offsetof(IterativeReconstructionNonlinearOptimizationParamsWrapper, mFinalNonLinearIterations),
          400,
          "finalNonLinearIterations",
          "finalNonLinearIterations",
          "Final non-linear iterations",
          true,
         1,
         100000
        );
    field3->precision=-1;
    fields().push_back(field3);
    /*  */ 
    IntField* field4 = new IntField
        (
          IterativeReconstructionNonlinearOptimizationParamsWrapper::ALTERNATINGITERATIONS_ID,
          offsetof(IterativeReconstructionNonlinearOptimizationParamsWrapper, mAlternatingIterations),
          0,
          "alternatingIterations",
          "alternatingIterations",
          "Alternating optimization on success steps",
          true,
         0,
         1000
        );
    field4->precision=-1;
    fields().push_back(field4);
    /*  */ 
    BoolField* field5 = new BoolField
        (
          IterativeReconstructionNonlinearOptimizationParamsWrapper::EXCESSIVEQUATERNIONPARAMETRIZATION_ID,
          offsetof(IterativeReconstructionNonlinearOptimizationParamsWrapper, mExcessiveQuaternionParametrization),
          false,
          "excessiveQuaternionParametrization",
          "excessiveQuaternionParametrization",
          "Excessive/non-excessive quaternion parametrization"
        );
    field5->widgetHint=BaseField::CHECK_BOX;
    field5->precision=-1;
    fields().push_back(field5);
    /*  */ 
    BoolField* field6 = new BoolField
        (
          IterativeReconstructionNonlinearOptimizationParamsWrapper::EXPLICITINVERSE_ID,
          offsetof(IterativeReconstructionNonlinearOptimizationParamsWrapper, mExplicitInverse),
          false,
          "explicitInverse",
          "explicitInverse",
          "Explisit inverse in Schur complement solving"
        );
    field6->widgetHint=BaseField::CHECK_BOX;
    field6->precision=-1;
    fields().push_back(field6);
    /*  */ 
   return 0;
}

SUPPRESS_OFFSET_WARNING_END


