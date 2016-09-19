/**
 * \file rectifyParameters.cpp
 * \attention This file is automatically generated and should not be in general modified manually
 *
 * \date MMM DD, 20YY
 * \author autoGenerator
 */

#include <vector>
#include <stddef.h>
#include "rectifyParameters.h"

/**
 *  Looks extremely unsafe because it depends on the order of static initialization.
 *  Should check standard if this is ok
 *
 *  Also it's not clear why removing "= Reflection()" breaks the code;
 **/

namespace corecvs {
template<>
Reflection BaseReflection<RectifyParameters>::reflection = Reflection();
template<>
int BaseReflection<RectifyParameters>::dummy = RectifyParameters::staticInit();
} // namespace corecvs 

SUPPRESS_OFFSET_WARNING_BEGIN

int RectifyParameters::staticInit()
{

    ReflectionNaming &nameing = naming();
    nameing = ReflectionNaming(
        "Rectify Parameters",
        "osd parameters",
        ""
    );

     getReflection()->objectSize = sizeof(RectifyParameters);
     

    EnumField* field0 = new EnumField
        (
          RectifyParameters::MATCHINGMETHOD_ID,
          offsetof(RectifyParameters, mMatchingMethod),
          1,
          "matchingMethod",
          "matchingMethod",
          "matchingMethod",
          new EnumReflection(2
          , new EnumOption(0,"SurfCV")
          , new EnumOption(1,"viTech")
          )
        );
    field0->widgetHint=BaseField::COMBO_BOX;
    fields().push_back(field0);
    /*  */ 
    DoubleField* field1 = new DoubleField
        (
          RectifyParameters::HESSIANTHRESHOLD_ID,
          offsetof(RectifyParameters, mHessianThreshold),
          400,
          "hessianThreshold",
          "hessianThreshold",
          "hessianThreshold"
        );
    field1->widgetHint=BaseField::SPIN_BOX;
    field1->precision=2;
    fields().push_back(field1);
    /*  */ 
    IntField* field2 = new IntField
        (
          RectifyParameters::OCTAVES_ID,
          offsetof(RectifyParameters, mOctaves),
          3,
          "octaves",
          "octaves",
          "octaves"
        );
    fields().push_back(field2);
    /*  */ 
    IntField* field3 = new IntField
        (
          RectifyParameters::OCTAVELAYERS_ID,
          offsetof(RectifyParameters, mOctaveLayers),
          4,
          "octaveLayers",
          "octaveLayers",
          "octaveLayers"
        );
    fields().push_back(field3);
    /*  */ 
    BoolField* field4 = new BoolField
        (
          RectifyParameters::EXTENDED_ID,
          offsetof(RectifyParameters, mExtended),
          false,
          "extended",
          "extended",
          "extended"
        );
    field4->widgetHint=BaseField::CHECK_BOX;
    fields().push_back(field4);
    /*  */ 
    DoubleField* field5 = new DoubleField
        (
          RectifyParameters::FILTERMINIMUMLENGTH_ID,
          offsetof(RectifyParameters, mFilterMinimumLength),
          3,
          "filterMinimumLength",
          "filterMinimumLength",
          "filterMinimumLength",
          true,
         0,
         99
        );
    field5->widgetHint=BaseField::SPIN_BOX;
    field5->precision=2;
    fields().push_back(field5);
    /*  */ 
    BoolField* field6 = new BoolField
        (
          RectifyParameters::USEKLT_ID,
          offsetof(RectifyParameters, mUseKLT),
          false,
          "useKLT",
          "useKLT",
          "useKLT"
        );
    field6->widgetHint=BaseField::CHECK_BOX;
    fields().push_back(field6);
    /*  */ 
    BoolField* field7 = new BoolField
        (
          RectifyParameters::COMPUTEESSENTIAL_ID,
          offsetof(RectifyParameters, mComputeEssential),
          true,
          "computeEssential",
          "computeEssential",
          "computeEssential"
        );
    field7->widgetHint=BaseField::CHECK_BOX;
    fields().push_back(field7);
    /*  */ 
    DoubleField* field8 = new DoubleField
        (
          RectifyParameters::PRIORFOCAL_ID,
          offsetof(RectifyParameters, mPriorFocal),
          820.427,
          "priorFocal",
          "priorFocal",
          "priorFocal",
          true,
         0,
         99999
        );
    field8->widgetHint=BaseField::SPIN_BOX;
    field8->precision=2;
    fields().push_back(field8);
    /*  */ 
    DoubleField* field9 = new DoubleField
        (
          RectifyParameters::PRIORFOCAL2_ID,
          offsetof(RectifyParameters, mPriorFocal2),
          820.427,
          "priorFocal2",
          "priorFocal2",
          "priorFocal2",
          true,
         0,
         99999
        );
    field9->widgetHint=BaseField::SPIN_BOX;
    field9->precision=2;
    fields().push_back(field9);
    /*  */ 
    DoubleField* field10 = new DoubleField
        (
          RectifyParameters::BASELINELENGTH_ID,
          offsetof(RectifyParameters, mBaselineLength),
          60,
          "baselineLength",
          "baselineLength",
          "baselineLength",
          true,
         -1000,
         1000
        );
    field10->widgetHint=BaseField::SPIN_BOX;
    field10->precision=2;
    fields().push_back(field10);
    /*  */ 
    DoubleField* field11 = new DoubleField
        (
          RectifyParameters::FOVANGLE_ID,
          offsetof(RectifyParameters, mFovAngle),
          60,
          "fovAngle",
          "fovAngle",
          "fovAngle",
          true,
         0,
         20
        );
    field11->widgetHint=BaseField::SPIN_BOX;
    field11->precision=2;
    fields().push_back(field11);
    /*  */ 
    EnumField* field12 = new EnumField
        (
          RectifyParameters::ESTIMATIONMETHOD_ID,
          offsetof(RectifyParameters, mEstimationMethod),
          1,
          "estimationMethod",
          "estimationMethod",
          "estimationMethod",
          new EnumReflection(3
          , new EnumOption(0,"RANSAC")
          , new EnumOption(1,"Iterative")
          , new EnumOption(2,"Manual")
          )
        );
    field12->widgetHint=BaseField::COMBO_BOX;
    fields().push_back(field12);
    /*  */ 
    BoolField* field13 = new BoolField
        (
          RectifyParameters::NORMALISE_ID,
          offsetof(RectifyParameters, mNormalise),
          true,
          "normalise",
          "normalise",
          "normalise"
        );
    field13->widgetHint=BaseField::CHECK_BOX;
    fields().push_back(field13);
    /*  */ 
    IntField* field14 = new IntField
        (
          RectifyParameters::RANSACITERATIONS_ID,
          offsetof(RectifyParameters, mRansacIterations),
          4000,
          "ransacIterations",
          "ransacIterations",
          "ransacIterations",
          true,
         1,
         9999
        );
    fields().push_back(field14);
    /*  */ 
    IntField* field15 = new IntField
        (
          RectifyParameters::RANSACTESTSIZE_ID,
          offsetof(RectifyParameters, mRansacTestSize),
          15,
          "ransacTestSize",
          "ransacTestSize",
          "ransacTestSize",
          true,
         1,
         9999
        );
    fields().push_back(field15);
    /*  */ 
    DoubleField* field16 = new DoubleField
        (
          RectifyParameters::RANSACTHRESHOLD_ID,
          offsetof(RectifyParameters, mRansacThreshold),
          1,
          "ransacThreshold",
          "ransacThreshold",
          "ransacThreshold",
          true,
         1,
         20
        );
    field16->widgetHint=BaseField::SPIN_BOX;
    field16->precision=2;
    fields().push_back(field16);
    /*  */ 
    DoubleField* field17 = new DoubleField
        (
          RectifyParameters::BASELINEX_ID,
          offsetof(RectifyParameters, mBaselineX),
          1,
          "baselineX",
          "baselineX",
          "baselineX",
          true,
         -20,
         20
        );
    field17->widgetHint=BaseField::SPIN_BOX;
    field17->precision=2;
    fields().push_back(field17);
    /*  */ 
    DoubleField* field18 = new DoubleField
        (
          RectifyParameters::BASELINEY_ID,
          offsetof(RectifyParameters, mBaselineY),
          0,
          "baselineY",
          "baselineY",
          "baselineY",
          true,
         -20,
         20
        );
    field18->widgetHint=BaseField::SPIN_BOX;
    field18->precision=2;
    fields().push_back(field18);
    /*  */ 
    DoubleField* field19 = new DoubleField
        (
          RectifyParameters::BASELINEZ_ID,
          offsetof(RectifyParameters, mBaselineZ),
          0,
          "baselineZ",
          "baselineZ",
          "baselineZ",
          true,
         -20,
         20
        );
    field19->widgetHint=BaseField::SPIN_BOX;
    field19->precision=2;
    fields().push_back(field19);
    /*  */ 
    EnumField* field20 = new EnumField
        (
          RectifyParameters::ITERATIVEMETHOD_ID,
          offsetof(RectifyParameters, mIterativeMethod),
          2,
          "iterativeMethod",
          "iterativeMethod",
          "iterativeMethod",
          new EnumReflection(5
          , new EnumOption(0,"SVD")
          , new EnumOption(1,"Gradient Descent")
          , new EnumOption(2,"Marquardt Levenberg")
          , new EnumOption(3,"Classic Kalman")
          , new EnumOption(4,"Kalman")
          )
        );
    field20->widgetHint=BaseField::COMBO_BOX;
    fields().push_back(field20);
    /*  */ 
    IntField* field21 = new IntField
        (
          RectifyParameters::ITERATIVEITERATIONS_ID,
          offsetof(RectifyParameters, mIterativeIterations),
          30,
          "iterativeIterations",
          "iterativeIterations",
          "iterativeIterations",
          true,
         1,
         9999
        );
    fields().push_back(field21);
    /*  */ 
    DoubleField* field22 = new DoubleField
        (
          RectifyParameters::ITERATIVEINITIALSIGMA_ID,
          offsetof(RectifyParameters, mIterativeInitialSigma),
          30,
          "iterativeInitialSigma",
          "iterativeInitialSigma",
          "iterativeInitialSigma",
          true,
         0,
         20
        );
    field22->widgetHint=BaseField::SPIN_BOX;
    field22->precision=2;
    fields().push_back(field22);
    /*  */ 
    DoubleField* field23 = new DoubleField
        (
          RectifyParameters::ITERATIVEFACTORSIGMA_ID,
          offsetof(RectifyParameters, mIterativeFactorSigma),
          0.95,
          "iterativeFactorSigma",
          "iterativeFactorSigma",
          "iterativeFactorSigma",
          true,
         0,
         20
        );
    field23->widgetHint=BaseField::SPIN_BOX;
    field23->precision=2;
    fields().push_back(field23);
    /*  */ 
    DoubleField* field24 = new DoubleField
        (
          RectifyParameters::MANUALX_ID,
          offsetof(RectifyParameters, mManualX),
          1,
          "manualX",
          "manualX",
          "manualX",
          true,
         -20,
         20
        );
    field24->widgetHint=BaseField::SPIN_BOX;
    field24->precision=2;
    fields().push_back(field24);
    /*  */ 
    DoubleField* field25 = new DoubleField
        (
          RectifyParameters::MANUALY_ID,
          offsetof(RectifyParameters, mManualY),
          0,
          "manualY",
          "manualY",
          "manualY",
          true,
         -20,
         20
        );
    field25->widgetHint=BaseField::SPIN_BOX;
    field25->precision=2;
    fields().push_back(field25);
    /*  */ 
    DoubleField* field26 = new DoubleField
        (
          RectifyParameters::MANUALZ_ID,
          offsetof(RectifyParameters, mManualZ),
          0,
          "manualZ",
          "manualZ",
          "manualZ",
          true,
         -20,
         20
        );
    field26->widgetHint=BaseField::SPIN_BOX;
    field26->precision=2;
    fields().push_back(field26);
    /*  */ 
    DoubleField* field27 = new DoubleField
        (
          RectifyParameters::MANUALPITCH_ID,
          offsetof(RectifyParameters, mManualPitch),
          0,
          "manualPitch",
          "manualPitch",
          "manualPitch",
          true,
         -20,
         20
        );
    field27->widgetHint=BaseField::SPIN_BOX;
    field27->precision=2;
    fields().push_back(field27);
    /*  */ 
    DoubleField* field28 = new DoubleField
        (
          RectifyParameters::MANUALYAW_ID,
          offsetof(RectifyParameters, mManualYaw),
          0,
          "manualYaw",
          "manualYaw",
          "manualYaw",
          true,
         -20,
         20
        );
    field28->widgetHint=BaseField::SPIN_BOX;
    field28->precision=2;
    fields().push_back(field28);
    /*  */ 
    DoubleField* field29 = new DoubleField
        (
          RectifyParameters::MANUALROLL_ID,
          offsetof(RectifyParameters, mManualRoll),
          0,
          "manualRoll",
          "manualRoll",
          "manualRoll",
          true,
         -20,
         20
        );
    field29->widgetHint=BaseField::SPIN_BOX;
    field29->precision=2;
    fields().push_back(field29);
    /*  */ 
    DoubleField* field30 = new DoubleField
        (
          RectifyParameters::ZDIRX_ID,
          offsetof(RectifyParameters, mZdirX),
          1,
          "zdirX",
          "zdirX",
          "zdirX",
          true,
         -20,
         20
        );
    field30->widgetHint=BaseField::SPIN_BOX;
    field30->precision=2;
    fields().push_back(field30);
    /*  */ 
    DoubleField* field31 = new DoubleField
        (
          RectifyParameters::ZDIRY_ID,
          offsetof(RectifyParameters, mZdirY),
          0,
          "zdirY",
          "zdirY",
          "zdirY",
          true,
         -20,
         20
        );
    field31->widgetHint=BaseField::SPIN_BOX;
    field31->precision=2;
    fields().push_back(field31);
    /*  */ 
    DoubleField* field32 = new DoubleField
        (
          RectifyParameters::ZDIRZ_ID,
          offsetof(RectifyParameters, mZdirZ),
          0,
          "zdirZ",
          "zdirZ",
          "zdirZ",
          true,
         -20,
         20
        );
    field32->widgetHint=BaseField::SPIN_BOX;
    field32->precision=2;
    fields().push_back(field32);
    /*  */ 
    BoolField* field33 = new BoolField
        (
          RectifyParameters::AUTOZ_ID,
          offsetof(RectifyParameters, mAutoZ),
          true,
          "autoZ",
          "autoZ",
          "autoZ"
        );
    field33->widgetHint=BaseField::CHECK_BOX;
    fields().push_back(field33);
    /*  */ 
    BoolField* field34 = new BoolField
        (
          RectifyParameters::AUTOSHIFT_ID,
          offsetof(RectifyParameters, mAutoShift),
          true,
          "autoShift",
          "autoShift",
          "autoShift"
        );
    field34->widgetHint=BaseField::CHECK_BOX;
    fields().push_back(field34);
    /*  */ 
    IntField* field35 = new IntField
        (
          RectifyParameters::PRESHIFT_ID,
          offsetof(RectifyParameters, mPreShift),
          0,
          "preShift",
          "preShift",
          "preShift",
          true,
         -9999,
         9999
        );
    fields().push_back(field35);
    /*  */ 
    IntField* field36 = new IntField
        (
          RectifyParameters::GUESSSHIFTTHRESHOLD_ID,
          offsetof(RectifyParameters, mGuessShiftThreshold),
          0,
          "guessShiftThreshold",
          "guessShiftThreshold",
          "guessShiftThreshold",
          true,
         0,
         99999
        );
    fields().push_back(field36);
    /*  */ 
    ReflectionDirectory &directory = *ReflectionDirectoryHolder::getReflectionDirectory();
    directory[std::string("Rectify Parameters")]= &reflection;
   return 0;
}

SUPPRESS_OFFSET_WARNING_END


