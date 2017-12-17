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

namespace corecvs {
template<>
Reflection BaseReflection<HeadSearchParameters>::reflection = Reflection();
template<>
int BaseReflection<HeadSearchParameters>::dummy = HeadSearchParameters::staticInit();
} // namespace corecvs 

SUPPRESS_OFFSET_WARNING_BEGIN


using namespace corecvs;

int HeadSearchParameters::staticInit()
{

    ReflectionNaming &nameing = naming();
    nameing = ReflectionNaming(
        "Head Search Parameters",
        "Head Search Parameters",
        ""
    );

     getReflection()->objectSize = sizeof(HeadSearchParameters);
     

    DoubleField* field0 = new DoubleField
        (
          HeadSearchParameters::THRESHOLD_DISTANCE_ID,
          offsetof(HeadSearchParameters, mThresholdDistance),
          100,
          "Threshold Distance",
          "Threshold Distance",
          "Threshold Distance in mm",
          true,
         0,
         99999
        );
    field0->widgetHint=BaseField::SPIN_BOX;
    field0->precision=2;
    fields().push_back(field0);
    /*  */ 
    DoubleField* field1 = new DoubleField
        (
          HeadSearchParameters::CLUSTER_DEPTH_ID,
          offsetof(HeadSearchParameters, mClusterDepth),
          100,
          "Cluster depth",
          "Cluster depth",
          "Cluster depth in mm",
          true,
         0,
         99999
        );
    field1->widgetHint=BaseField::SPIN_BOX;
    field1->precision=2;
    fields().push_back(field1);
    /*  */ 
    IntField* field2 = new IntField
        (
          HeadSearchParameters::CLUSTER_MIN_SIZE_ID,
          offsetof(HeadSearchParameters, mClusterMinSize),
          100,
          "Cluster min size",
          "Cluster min size",
          "Cluster min size",
          true,
         0,
         99999
        );
    fields().push_back(field2);
    /*  */ 
    DoubleField* field3 = new DoubleField
        (
          HeadSearchParameters::HEAD_AREA_RADIUS_ID,
          offsetof(HeadSearchParameters, mHeadAreaRadius),
          100,
          "Head area radius",
          "Head area radius",
          "Head area in mm",
          true,
         0,
         99999
        );
    field3->widgetHint=BaseField::SPIN_BOX;
    field3->precision=2;
    fields().push_back(field3);
    /*  */ 
    IntField* field4 = new IntField
        (
          HeadSearchParameters::HEAD_NUMBER_ID,
          offsetof(HeadSearchParameters, mHeadNumber),
          1,
          "Head Number",
          "Head Number",
          "Expected number of heads",
          true,
         0,
         100
        );
    fields().push_back(field4);
    /*  */ 
    ReflectionDirectory &directory = *ReflectionDirectoryHolder::getReflectionDirectory();
    directory[std::string("Head Search Parameters")]= &reflection;
   return 0;
}
int HeadSearchParameters::relinkCompositeFields()
{
   return 0;
}

SUPPRESS_OFFSET_WARNING_END


