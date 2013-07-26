/**
 * \file baseParameters.cpp
 * \attention This file is automatically generated and should not be in general modified manually
 *
 * \date MMM DD, 20YY
 * \author autoGenerator
 */

#include <vector>
#include <stddef.h>
#include "baseParameters.h"

/**
 *  Looks extremely unsafe because it depends on the order of static initialization.
 *  Should check standard if this is ok
 *
 *  Also it's not clear why removing "= Reflection()" breaks the code;
 **/

namespace core3vi {
template<>
Reflection BaseReflection<BaseParameters>::reflection = Reflection();
template<>
int BaseReflection<BaseParameters>::dummy = BaseParameters::staticInit();
} // namespace core3vi 

SUPPRESS_OFFSET_WARNING_BEGIN

int BaseParameters::staticInit()
{

    ReflectionNaming &nameing = naming();
    nameing = ReflectionNaming(
        "Base Parameters",
        "Base parameters",
        ""
    );
     

    fields().push_back(
        new EnumField
        (
          BaseParameters::ROTATION_ID,
          offsetof(BaseParameters, mRotation),
          0,
          "rotation",
          "rotation",
          "rotation",
           NULL
        )
    );
    fields().push_back(
        new BoolField
        (
          BaseParameters::MIRROR_ID,
          offsetof(BaseParameters, mMirror),
          false,
          "mirror",
          "mirror",
          "mirror"
        )
    );
    fields().push_back(
        new BoolField
        (
          BaseParameters::SWAPCAMERAS_ID,
          offsetof(BaseParameters, mSwapCameras),
          false,
          "swapCameras",
          "swapCameras",
          "swapCameras"
        )
    );
    fields().push_back(
        new BoolField
        (
          BaseParameters::FILTERLOCK_ID,
          offsetof(BaseParameters, mFilterLock),
          false,
          "filterLock",
          "filterLock",
          "filterLock"
        )
    );
    fields().push_back(
        new BoolField
        (
          BaseParameters::ENABLEFILTERGRAPH_ID,
          offsetof(BaseParameters, mEnableFilterGraph),
          false,
          "enableFilterGraph",
          "enableFilterGraph",
          "enableFilterGraph"
        )
    );
    fields().push_back(
        new DoubleField
        (
          BaseParameters::DOWNSAMPLE_ID,
          offsetof(BaseParameters, mDownsample),
          1.5,
          "downsample",
          "downsample",
          "downsample"
        )
    );
    fields().push_back(
        new IntField
        (
          BaseParameters::H_ID,
          offsetof(BaseParameters, mH),
          640,
          "h",
          "h",
          "h"
        )
    );
    fields().push_back(
        new IntField
        (
          BaseParameters::W_ID,
          offsetof(BaseParameters, mW),
          480,
          "w",
          "w",
          "w"
        )
    );
    fields().push_back(
        new BoolField
        (
          BaseParameters::AUTOH_ID,
          offsetof(BaseParameters, mAutoH),
          true,
          "autoH",
          "autoH",
          "autoH"
        )
    );
    fields().push_back(
        new BoolField
        (
          BaseParameters::AUTOW_ID,
          offsetof(BaseParameters, mAutoW),
          true,
          "autoW",
          "autoW",
          "autoW"
        )
    );
    fields().push_back(
        new IntField
        (
          BaseParameters::X_ID,
          offsetof(BaseParameters, mX),
          0,
          "x",
          "x",
          "x"
        )
    );
    fields().push_back(
        new IntField
        (
          BaseParameters::Y_ID,
          offsetof(BaseParameters, mY),
          0,
          "y",
          "y",
          "y"
        )
    );
    fields().push_back(
        new EnumField
        (
          BaseParameters::INTERPOLATIONTYPE_ID,
          offsetof(BaseParameters, mInterpolationType),
          0,
          "InterpolationType",
          "InterpolationType",
          "InterpolationType",
           NULL
        )
    );
   return 0;
}

SUPPRESS_OFFSET_WARNING_END

