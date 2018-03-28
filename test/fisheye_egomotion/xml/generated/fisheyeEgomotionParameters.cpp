/**
 * \file fisheyeEgomotionParameters.cpp
 * \attention This file is automatically generated and should not be in general modified manually
 *
 * \date MMM DD, 20YY
 * \author autoGenerator
 */

#include <vector>
#include <stddef.h>
#include "fisheyeEgomotionParameters.h"

/**
 *  Looks extremely unsafe because it depends on the order of static initialization.
 *  Should check standard if this is ok
 *
 *  Also it's not clear why removing "= Reflection()" breaks the code;
 **/

namespace corecvs {
template<>
Reflection BaseReflection<FisheyeEgomotionParameters>::reflection = Reflection();
template<>
int BaseReflection<FisheyeEgomotionParameters>::dummy = FisheyeEgomotionParameters::staticInit();
} // namespace corecvs 

SUPPRESS_OFFSET_WARNING_BEGIN


using namespace corecvs;

int FisheyeEgomotionParameters::staticInit()
{

    ReflectionNaming &nameing = naming();
    nameing = ReflectionNaming(
        "Fisheye Egomotion Parameters",
        "Fisheye Egomotion Parameters",
        ""
    );

     getReflection()->objectSize = sizeof(FisheyeEgomotionParameters);
     

    BoolField* field0 = new BoolField
        (
          FisheyeEgomotionParameters::UNWARP_ID,
          offsetof(FisheyeEgomotionParameters, mUnwarp),
          false,
          "unwarp",
          "Produce unwrapped image",
          "Produce unwrapped image"
        );
    field0->widgetHint=BaseField::CHECK_BOX;
    fields().push_back(field0);
    /*  */ 
    BoolField* field1 = new BoolField
        (
          FisheyeEgomotionParameters::JOINT_ID,
          offsetof(FisheyeEgomotionParameters, mJoint),
          false,
          "joint",
          "Make joint optimsation at the end",
          "Make joint optimsation at the end"
        );
    field1->widgetHint=BaseField::CHECK_BOX;
    fields().push_back(field1);
    /*  */ 
    BoolField* field2 = new BoolField
        (
          FisheyeEgomotionParameters::SCENE_ID,
          offsetof(FisheyeEgomotionParameters, mScene),
          false,
          "scene",
          "Output info in CVS .json scene format",
          "Output info in CVS .json scene format"
        );
    field2->widgetHint=BaseField::CHECK_BOX;
    fields().push_back(field2);
    /*  */ 
    BoolField* field3 = new BoolField
        (
          FisheyeEgomotionParameters::CAPS_ID,
          offsetof(FisheyeEgomotionParameters, mCaps),
          false,
          "caps",
          "Print capabilities and exit",
          "Print capabilities and exit"
        );
    field3->widgetHint=BaseField::CHECK_BOX;
    fields().push_back(field3);
    /*  */ 
    BoolField* field4 = new BoolField
        (
          FisheyeEgomotionParameters::SUBPIXEL_ID,
          offsetof(FisheyeEgomotionParameters, mSubpixel),
          false,
          "subpixel",
          "Force subpixel flow precision. If applicable",
          "Force subpixel flow precision. If applicable"
        );
    field4->widgetHint=BaseField::CHECK_BOX;
    fields().push_back(field4);
    /*  */ 
    StringField* field5 = new StringField
        (
          FisheyeEgomotionParameters::INTRINSICS_ID,
          offsetof(FisheyeEgomotionParameters, mIntrinsics),
          "",
          "intrinsics",
          "intrinsics",
          "intrinsics"
        );
    fields().push_back(field5);
    /*  */ 
    StringField* field6 = new StringField
        (
          FisheyeEgomotionParameters::PROVIDER_ID,
          offsetof(FisheyeEgomotionParameters, mProvider),
          "CVS",
          "provider",
          "which flow provider to use. For options see --caps",
          "which flow provider to use. For options see --caps"
        );
    fields().push_back(field6);
    /*  */ 
    IntField* field7 = new IntField
        (
          FisheyeEgomotionParameters::FRAMES_ID,
          offsetof(FisheyeEgomotionParameters, mFrames),
          10,
          "frames",
          "How many frames to process",
          "How many frames to process",
          true,
         -50000,
         50000,
         1
        );
    fields().push_back(field7);
    /*  */ 
    IntField* field8 = new IntField
        (
          FisheyeEgomotionParameters::SKIPF_ID,
          offsetof(FisheyeEgomotionParameters, mSkipf),
          0,
          "skipf",
          "skipf",
          "skipf",
          true,
         -50000,
         50000,
         1
        );
    fields().push_back(field8);
    /*  */ 
    DoubleField* field9 = new DoubleField
        (
          FisheyeEgomotionParameters::RTHRESHOLD_ID,
          offsetof(FisheyeEgomotionParameters, mRthreshold),
          0.009,
          "rthreshold",
          "Threshold for RANSAC during model estimation",
          "Threshold for RANSAC during model estimation",
          true,
         0,
         500,
         1
        );
    field9->widgetHint=BaseField::SPIN_BOX;
    field9->precision=6;
    fields().push_back(field9);
    /*  */ 
    DoubleField* field10 = new DoubleField
        (
          FisheyeEgomotionParameters::DTHRESHOLD_ID,
          offsetof(FisheyeEgomotionParameters, mDthreshold),
          0.009,
          "dthreshold",
          "Threshold for actual detection",
          "Threshold for actual detection",
          true,
         0,
         500,
         1
        );
    field10->widgetHint=BaseField::SPIN_BOX;
    field10->precision=6;
    fields().push_back(field10);
    /*  */ 
    DoubleField* field11 = new DoubleField
        (
          FisheyeEgomotionParameters::RSTHRESHOLD_ID,
          offsetof(FisheyeEgomotionParameters, mRsthreshold),
          3,
          "rsthreshold",
          "Threshold for Static flow",
          "Threshold for Static flow",
          true,
         0,
         500,
         1
        );
    field11->widgetHint=BaseField::SPIN_BOX;
    field11->suffixHint="px";
    field11->precision=3;
    fields().push_back(field11);
    /*  */ 
    DoubleField* field12 = new DoubleField
        (
          FisheyeEgomotionParameters::RSTHRESPREC_ID,
          offsetof(FisheyeEgomotionParameters, mRsthresprec),
          70,
          "rsthresprec",
          "precent to fit Static Thresold ",
          "precent to fit Static Thresold ",
          true,
         0,
         500,
         1
        );
    field12->widgetHint=BaseField::SPIN_BOX;
    field12->suffixHint="%";
    field12->precision=3;
    fields().push_back(field12);
    /*  */ 
    DoubleField* field13 = new DoubleField
        (
          FisheyeEgomotionParameters::DSTHRESHOLD_ID,
          offsetof(FisheyeEgomotionParameters, mDsthreshold),
          3,
          "dsthreshold",
          "Threshold for Static actual detection",
          "Threshold for Static actual detection",
          true,
         0,
         500,
         1
        );
    field13->widgetHint=BaseField::SPIN_BOX;
    field13->suffixHint="px";
    field13->precision=3;
    fields().push_back(field13);
    /*  */ 
    BoolField* field14 = new BoolField
        (
          FisheyeEgomotionParameters::TRACEPARAMS_ID,
          offsetof(FisheyeEgomotionParameters, mTraceParams),
          false,
          "traceParams",
          "Trace parameters of the provider",
          "Trace parameters of the provider"
        );
    field14->widgetHint=BaseField::CHECK_BOX;
    fields().push_back(field14);
    /*  */ 
    ReflectionDirectory &directory = *ReflectionDirectoryHolder::getReflectionDirectory();
    directory[std::string("Fisheye Egomotion Parameters")]= &reflection;
   return 0;
}
int FisheyeEgomotionParameters::relinkCompositeFields()
{
   return 0;
}

SUPPRESS_OFFSET_WARNING_END


