/**
 * \file stereoAlignParameters.cpp
 * \attention This file is automatically generated and should not be in general modified manually
 *
 * \date MMM DD, 20YY
 * \author autoGenerator
 */

#include <vector>
#include <stddef.h>
#include "stereoAlignParameters.h"

/**
 *  Looks extremely unsafe because it depends on the order of static initialization.
 *  Should check standard if this is ok
 *
 *  Also it's not clear why removing "= Reflection()" breaks the code;
 **/

namespace corecvs {
template<>
Reflection BaseReflection<StereoAlignParameters>::reflection = Reflection();
template<>
int BaseReflection<StereoAlignParameters>::dummy = StereoAlignParameters::staticInit();
} // namespace corecvs 

SUPPRESS_OFFSET_WARNING_BEGIN


using namespace corecvs;

int StereoAlignParameters::staticInit()
{

    ReflectionNaming &nameing = naming();
    nameing = ReflectionNaming(
        "Stereo Align Parameters",
        "Stereo Align Parameters",
        ""
    );

     getReflection()->objectSize = sizeof(StereoAlignParameters);
     

    BoolField* field0 = new BoolField
        (
          StereoAlignParameters::PRODUCE_CAMERAS_ID,
          offsetof(StereoAlignParameters, mProduceCameras),
          true,
          "Produce Cameras",
          "Produce Cameras",
          "Produce Cameras"
        );
    field0->widgetHint=BaseField::CHECK_BOX;
    fields().push_back(field0);
    /*  */ 
    DoubleField* field1 = new DoubleField
        (
          StereoAlignParameters::ZDIRX_ID,
          offsetof(StereoAlignParameters, mZdirX),
          0,
          "zdirX",
          "zdirX",
          "zdirX",
          true,
         -20,
         20
        );
    field1->widgetHint=BaseField::SPIN_BOX;
    field1->precision=2;
    fields().push_back(field1);
    /*  */ 
    DoubleField* field2 = new DoubleField
        (
          StereoAlignParameters::ZDIRY_ID,
          offsetof(StereoAlignParameters, mZdirY),
          1,
          "zdirY",
          "zdirY",
          "zdirY",
          true,
         -20,
         20
        );
    field2->widgetHint=BaseField::SPIN_BOX;
    field2->precision=2;
    fields().push_back(field2);
    /*  */ 
    DoubleField* field3 = new DoubleField
        (
          StereoAlignParameters::ZDIRZ_ID,
          offsetof(StereoAlignParameters, mZdirZ),
          0,
          "zdirZ",
          "zdirZ",
          "zdirZ",
          true,
         -20,
         20
        );
    field3->widgetHint=BaseField::SPIN_BOX;
    field3->precision=2;
    fields().push_back(field3);
    /*  */ 
    BoolField* field4 = new BoolField
        (
          StereoAlignParameters::AUTOZ_ID,
          offsetof(StereoAlignParameters, mAutoZ),
          true,
          "autoZ",
          "autoZ",
          "autoZ"
        );
    field4->widgetHint=BaseField::CHECK_BOX;
    fields().push_back(field4);
    /*  */ 
    BoolField* field5 = new BoolField
        (
          StereoAlignParameters::AUTOSHIFT_ID,
          offsetof(StereoAlignParameters, mAutoShift),
          true,
          "autoShift",
          "autoShift",
          "autoShift"
        );
    field5->widgetHint=BaseField::CHECK_BOX;
    fields().push_back(field5);
    /*  */ 
    IntField* field6 = new IntField
        (
          StereoAlignParameters::PRESHIFT_ID,
          offsetof(StereoAlignParameters, mPreShift),
          0,
          "preShift",
          "preShift",
          "preShift",
          true,
         -9999,
         9999
        );
    fields().push_back(field6);
    /*  */ 
    IntField* field7 = new IntField
        (
          StereoAlignParameters::GUESSSHIFTTHRESHOLD_ID,
          offsetof(StereoAlignParameters, mGuessShiftThreshold),
          0,
          "guessShiftThreshold",
          "guessShiftThreshold",
          "guessShiftThreshold",
          true,
         0,
         99999
        );
    fields().push_back(field7);
    /*  */ 
    ReflectionDirectory &directory = *ReflectionDirectoryHolder::getReflectionDirectory();
    directory[std::string("Stereo Align Parameters")]= &reflection;
   return 0;
}
int StereoAlignParameters::relinkCompositeFields()
{
   return 0;
}

SUPPRESS_OFFSET_WARNING_END


