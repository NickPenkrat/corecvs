/**
 * \file featureDetectionParams.cpp
 * \attention This file is automatically generated and should not be in general modified manually
 *
 * \date MMM DD, 20YY
 * \author autoGenerator
 */

#include <vector>
#include <stddef.h>
#include "featureDetectionParams.h"

/**
 *  Looks extremely unsafe because it depends on the order of static initialization.
 *  Should check standard if this is ok
 *
 *  Also it's not clear why removing "= Reflection()" breaks the code;
 **/

namespace corecvs {
template<>
Reflection BaseReflection<FeatureDetectionParams>::reflection = Reflection();
template<>
int BaseReflection<FeatureDetectionParams>::dummy = FeatureDetectionParams::staticInit();
} // namespace corecvs 

SUPPRESS_OFFSET_WARNING_BEGIN

int FeatureDetectionParams::staticInit()
{

    ReflectionNaming &nameing = naming();
    nameing = ReflectionNaming(
        "Feature Detection Params",
        "Feature Detection Params",
        ""
    );
     

    fields().push_back(
        new StringField
        (
          FeatureDetectionParams::DETECTOR_ID,
          offsetof(FeatureDetectionParams, mDetector),
          "ORB",
          "detector",
          "detector",
          "detector"
        )
    );
    fields().push_back(
        new StringField
        (
          FeatureDetectionParams::DESCRIPTOR_ID,
          offsetof(FeatureDetectionParams, mDescriptor),
          "ORB",
          "descriptor",
          "descriptor",
          "descriptor"
        )
    );
    fields().push_back(
        new StringField
        (
          FeatureDetectionParams::MATCHER_ID,
          offsetof(FeatureDetectionParams, mMatcher),
          "BF",
          "matcher",
          "matcher",
          "matcher"
        )
    );
    fields().push_back(
        new DoubleField
        (
          FeatureDetectionParams::B2BTHRESHOLD_ID,
          offsetof(FeatureDetectionParams, mB2bThreshold),
          0.9,
          "b2bThreshold",
          "b2bThreshold",
          "b2bThreshold"
        )
    );
    fields().push_back(
        new BoolField
        (
          FeatureDetectionParams::MATCHF2F_ID,
          offsetof(FeatureDetectionParams, mMatchF2F),
          false,
          "matchF2F",
          "matchF2F",
          "matchF2F"
        )
    );
   return 0;
}

SUPPRESS_OFFSET_WARNING_END


