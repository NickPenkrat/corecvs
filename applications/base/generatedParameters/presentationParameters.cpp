/**
 * \file presentationParameters.cpp
 * \attention This file is automatically generated and should not be in general modified manually
 *
 * \date MMM DD, 20YY
 * \author autoGenerator
 */

#include <vector>
#include <stddef.h>
#include "presentationParameters.h"

/**
 *  Looks extremely unsafe because it depends on the order of static initialization.
 *  Should check standard if this is ok
 *
 *  Also it's not clear why removing "= Reflection()" breaks the code;
 **/

namespace corecvs {
template<>
Reflection BaseReflection<PresentationParameters>::reflection = Reflection();
template<>
int BaseReflection<PresentationParameters>::dummy = PresentationParameters::staticInit();
} // namespace corecvs 

SUPPRESS_OFFSET_WARNING_BEGIN

int PresentationParameters::staticInit()
{

    ReflectionNaming &nameing = naming();
    nameing = ReflectionNaming(
        "Presentation Parameters",
        "Presentation parameters",
        ""
    );
     

    fields().push_back(
        new EnumField
        (
          PresentationParameters::OUTPUT_ID,
          offsetof(PresentationParameters, mOutput),
          0,
          "Output",
          "Output",
          "Output",
           NULL
        )
    );
    fields().push_back(
        new EnumField
        (
          PresentationParameters::STEREO_ID,
          offsetof(PresentationParameters, mStereo),
          0,
          "Stereo",
          "Stereo",
          "Stereo",
           NULL
        )
    );
    fields().push_back(
        new EnumField
        (
          PresentationParameters::FLOW_ID,
          offsetof(PresentationParameters, mFlow),
          0,
          "Flow",
          "Flow",
          "Flow",
           NULL
        )
    );
    fields().push_back(
        new BoolField
        (
          PresentationParameters::SHOWCLUSTERS_ID,
          offsetof(PresentationParameters, mShowClusters),
          true,
          "showClusters",
          "showClusters",
          "showClusters"
        )
    );
    fields().push_back(
        new BoolField
        (
          PresentationParameters::SHOWHISTOGRAM_ID,
          offsetof(PresentationParameters, mShowHistogram),
          true,
          "showHistogram",
          "showHistogram",
          "showHistogram"
        )
    );
    fields().push_back(
        new BoolField
        (
          PresentationParameters::AUTO_UPDATE_HISTOGRAM_ID,
          offsetof(PresentationParameters, mAutoUpdateHistogram),
          false,
          "Auto Update Histogram",
          "Auto Update Histogram",
          "Auto Update Histogram"
        )
    );
    fields().push_back(
        new BoolField
        (
          PresentationParameters::SHOWAREAOFINTEREST_ID,
          offsetof(PresentationParameters, mShowAreaOfInterest),
          true,
          "showAreaOfInterest",
          "showAreaOfInterest",
          "showAreaOfInterest"
        )
    );
    fields().push_back(
        new BoolField
        (
          PresentationParameters::PRODUCE3D_ID,
          offsetof(PresentationParameters, mProduce3D),
          false,
          "produce3D",
          "produce3D",
          "produce3D"
        )
    );
    fields().push_back(
        new BoolField
        (
          PresentationParameters::PRODUCE6D_ID,
          offsetof(PresentationParameters, mProduce6D),
          false,
          "produce6D",
          "produce6D",
          "produce6D"
        )
    );
    fields().push_back(
        new BoolField
        (
          PresentationParameters::DUMP3D_ID,
          offsetof(PresentationParameters, mDump3D),
          false,
          "dump3D",
          "dump3D",
          "dump3D"
        )
    );
   return 0;
}

SUPPRESS_OFFSET_WARNING_END


