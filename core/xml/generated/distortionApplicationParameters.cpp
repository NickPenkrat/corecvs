/**
 * \file distortionApplicationParameters.cpp
 * \attention This file is automatically generated and should not be in general modified manually
 *
 * \date MMM DD, 20YY
 * \author autoGenerator
 */

#include <vector>
#include <stddef.h>
#include "distortionApplicationParameters.h"

/**
 *  Looks extremely unsafe because it depends on the order of static initialization.
 *  Should check standard if this is ok
 *
 *  Also it's not clear why removing "= Reflection()" breaks the code;
 **/

namespace corecvs {
template<>
Reflection BaseReflection<DistortionApplicationParameters>::reflection = Reflection();
template<>
int BaseReflection<DistortionApplicationParameters>::dummy = DistortionApplicationParameters::staticInit();
} // namespace corecvs 

SUPPRESS_OFFSET_WARNING_BEGIN

int DistortionApplicationParameters::staticInit()
{

    ReflectionNaming &nameing = naming();
    nameing = ReflectionNaming(
        "Distortion Application Parameters",
        "Distortion Application Parameters",
        ""
    );
     

    fields().push_back(
        new DoubleField
        (
          DistortionApplicationParameters::FORCE_SCALE_ID,
          offsetof(DistortionApplicationParameters, mForceScale),
          0,
          "Force Scale",
          "Force Scale",
          "Force Scale"
        )
    );
    fields().push_back(
        new EnumField
        (
          DistortionApplicationParameters::RESIZE_POLICY_ID,
          offsetof(DistortionApplicationParameters, mResizePolicy),
          0,
          "Resize policy",
          "Resize policy",
          "Resize policy",
          new EnumReflection(4
          , new EnumOption(0,"No Change")
          , new EnumOption(1,"Force Size")
          , new EnumOption(2,"To Fit Result")
          , new EnumOption(3,"To No Gaps")
          )
        )
    );
    fields().push_back(
        new IntField
        (
          DistortionApplicationParameters::NEW_H_ID,
          offsetof(DistortionApplicationParameters, mNewH),
          0,
          "New H",
          "New H",
          "New H"
        )
    );
    fields().push_back(
        new IntField
        (
          DistortionApplicationParameters::NEW_W_ID,
          offsetof(DistortionApplicationParameters, mNewW),
          0,
          "New W",
          "New W",
          "New W"
        )
    );
   return 0;
}

SUPPRESS_OFFSET_WARNING_END


