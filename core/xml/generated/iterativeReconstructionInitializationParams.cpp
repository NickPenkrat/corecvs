/**
 * \file iterativeReconstructionInitializationParams.cpp
 * \attention This file is automatically generated and should not be in general modified manually
 *
 * \date MMM DD, 20YY
 * \author autoGenerator
 */

#include <vector>
#include <stddef.h>
#include "iterativeReconstructionInitializationParams.h"

/**
 *  Looks extremely unsafe because it depends on the order of static initialization.
 *  Should check standard if this is ok
 *
 *  Also it's not clear why removing "= Reflection()" breaks the code;
 **/

namespace corecvs {
template<>
Reflection BaseReflection<IterativeReconstructionInitializationParams>::reflection = Reflection();
template<>
int BaseReflection<IterativeReconstructionInitializationParams>::dummy = IterativeReconstructionInitializationParams::staticInit();
} // namespace corecvs 

SUPPRESS_OFFSET_WARNING_BEGIN

int IterativeReconstructionInitializationParams::staticInit()
{

    ReflectionNaming &nameing = naming();
    nameing = ReflectionNaming(
        "Iterative Reconstruction Initialization Params",
        "Iterative Reconstruction Initialization Params",
        ""
    );
     

    DoubleField* field0 = new DoubleField
        (
          IterativeReconstructionInitializationParams::B2BRANSACP5RPTHRESHOLD_ID,
          offsetof(IterativeReconstructionInitializationParams, mB2bRansacP5RPThreshold),
          0.8,
          "b2bRansacP5RPThreshold",
          "b2bRansacP5RPThreshold",
          "Best-2nd best essential estimator threshold",
          true,
         0,
         1
        );
    field0->widgetHint=BaseField::SPIN_BOX;
    field0->precision=2;
    fields().push_back(field0);
    /*  */ 
    DoubleField* field1 = new DoubleField
        (
          IterativeReconstructionInitializationParams::INLIERP5RPTHRESHOLD_ID,
          offsetof(IterativeReconstructionInitializationParams, mInlierP5RPThreshold),
          5,
          "inlierP5RPThreshold",
          "inlierP5RPThreshold",
          "Inlier threshold",
          true,
         0,
         50000
        );
    field1->widgetHint=BaseField::SPIN_BOX;
    field1->precision=2;
    fields().push_back(field1);
    /*  */ 
    IntField* field2 = new IntField
        (
          IterativeReconstructionInitializationParams::MAXESSENTIALRANSACITERATIONS_ID,
          offsetof(IterativeReconstructionInitializationParams, mMaxEssentialRansacIterations),
          600000,
          "maxEssentialRansacIterations",
          "maxEssentialRansacIterations",
          "Maximal essential estimator rounds",
          true,
         0,
         5000000
        );
    field2->precision=-1;
    fields().push_back(field2);
    /*  */ 
    DoubleField* field3 = new DoubleField
        (
          IterativeReconstructionInitializationParams::B2BRANSACP6RPTHRESHOLD_ID,
          offsetof(IterativeReconstructionInitializationParams, mB2bRansacP6RPThreshold),
          0.8,
          "b2bRansacP6RPThreshold",
          "b2bRansacP6RPThreshold",
          "Best-2nd best relative pose estimator threshold",
          true,
         0,
         1
        );
    field3->widgetHint=BaseField::SPIN_BOX;
    field3->precision=2;
    fields().push_back(field3);
    /*  */ 
    BoolField* field4 = new BoolField
        (
          IterativeReconstructionInitializationParams::RUNESSENTIALFILTERING_ID,
          offsetof(IterativeReconstructionInitializationParams, mRunEssentialFiltering),
          true,
          "runEssentialFiltering",
          "runEssentialFiltering",
          "Run essential filtering prior relative pose estimation"
        );
    field4->widgetHint=BaseField::CHECK_BOX;
    field4->precision=-1;
    fields().push_back(field4);
    /*  */ 
    DoubleField* field5 = new DoubleField
        (
          IterativeReconstructionInitializationParams::ESSENTIALTARGETGAMMA_ID,
          offsetof(IterativeReconstructionInitializationParams, mEssentialTargetGamma),
          0.01,
          "essentialTargetGamma",
          "essentialTargetGamma",
          "essentialTargetGamma",
          true,
         0,
         1
        );
    field5->widgetHint=BaseField::SPIN_BOX;
    field5->precision=6;
    fields().push_back(field5);
    /*  */ 
    IntField* field6 = new IntField
        (
          IterativeReconstructionInitializationParams::MAXP6RPITERATIONS_ID,
          offsetof(IterativeReconstructionInitializationParams, mMaxP6RPIterations),
          400000,
          "maxP6RPIterations",
          "maxP6RPIterations",
          "Maximal number of ransac iterations for pose estimation using pairwise correspondences",
          true,
         0,
         100000000
        );
    field6->precision=-1;
    fields().push_back(field6);
    /*  */ 
    DoubleField* field7 = new DoubleField
        (
          IterativeReconstructionInitializationParams::INLIERP6RPTHRESHOLD_ID,
          offsetof(IterativeReconstructionInitializationParams, mInlierP6RPThreshold),
          1,
          "inlierP6RPThreshold",
          "inlierP6RPThreshold",
          "Inlier threshold for pose estimation using pairwise correspondences",
          true,
         0,
         1000
        );
    field7->widgetHint=BaseField::SPIN_BOX;
    field7->precision=6;
    fields().push_back(field7);
    /*  */ 
    DoubleField* field8 = new DoubleField
        (
          IterativeReconstructionInitializationParams::GAMMAP6RP_ID,
          offsetof(IterativeReconstructionInitializationParams, mGammaP6RP),
          0.01,
          "gammaP6RP",
          "gammaP6RP",
          "Target failure probability for pose estimation",
          true,
         0,
         1
        );
    field8->widgetHint=BaseField::SPIN_BOX;
    field8->precision=6;
    fields().push_back(field8);
    /*  */ 
   return 0;
}

SUPPRESS_OFFSET_WARNING_END


