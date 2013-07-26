/**
 * \file headSearchParameters.cpp
 * \attention This file is automatically generated and should not be in general modified manually
 *
 * \date MMM DD, 20YY
 * \author autoGenerator
 */

#include <vector>
#include <stddef.h>
#include "headSearchParameters.h"

/**
 *  Looks extremely unsafe because it depends on the order of static initialization.
 *  Should check standard if this is ok
 *
 *  Also it's not clear why removing "= Reflection()" breaks the code;
 **/

namespace core3vi {
template<>
Reflection BaseReflection<HeadSearchParameters>::reflection = Reflection();
template<>
int BaseReflection<HeadSearchParameters>::dummy = HeadSearchParameters::staticInit();
} // namespace core3vi 

SUPPRESS_OFFSET_WARNING_BEGIN

int HeadSearchParameters::staticInit()
{

    ReflectionNaming &nameing = naming();
    nameing = ReflectionNaming(
        "Head Search Parameters",
        "Head Search Parameters",
        ""
    );
     

    fields().push_back(
        new DoubleField
        (
          HeadSearchParameters::THRESHOLD_DISTANCE_ID,
          offsetof(HeadSearchParameters, mThresholdDistance),
          100,
          "Threshold Distance",
          "Threshold Distance",
          "Threshold Distance in mm"
        )
    );
    fields().push_back(
        new DoubleField
        (
          HeadSearchParameters::CLUSTER_DEPTH_ID,
          offsetof(HeadSearchParameters, mClusterDepth),
          100,
          "Cluster depth",
          "Cluster depth",
          "Cluster depth in mm"
        )
    );
    fields().push_back(
        new IntField
        (
          HeadSearchParameters::CLUSTER_MIN_SIZE_ID,
          offsetof(HeadSearchParameters, mClusterMinSize),
          100,
          "Cluster min size",
          "Cluster min size",
          "Cluster min size"
        )
    );
    fields().push_back(
        new DoubleField
        (
          HeadSearchParameters::HEAD_AREA_RADIUS_ID,
          offsetof(HeadSearchParameters, mHeadAreaRadius),
          100,
          "Head area radius",
          "Head area radius",
          "Head area in mm"
        )
    );
    fields().push_back(
        new IntField
        (
          HeadSearchParameters::HEAD_NUMBER_ID,
          offsetof(HeadSearchParameters, mHeadNumber),
          1,
          "Head Number",
          "Head Number",
          "Expected number of heads"
        )
    );
   return 0;
}

SUPPRESS_OFFSET_WARNING_END

