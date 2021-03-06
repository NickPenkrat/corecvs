/**
 * \file meshFlowDrawParameters.cpp
 * \attention This file is automatically generated and should not be in general modified manually
 *
 * \date MMM DD, 20YY
 * \author autoGenerator
 */

#include <vector>
#include <stddef.h>
#include "meshFlowDrawParameters.h"

/**
 *  Looks extremely unsafe because it depends on the order of static initialization.
 *  Should check standard if this is ok
 *
 *  Also it's not clear why removing "= Reflection()" breaks the code;
 **/

namespace corecvs {
template<>
Reflection BaseReflection<MeshFlowDrawParameters>::reflection = Reflection();
template<>
int BaseReflection<MeshFlowDrawParameters>::dummy = MeshFlowDrawParameters::staticInit();
} // namespace corecvs 

SUPPRESS_OFFSET_WARNING_BEGIN


using namespace corecvs;

int MeshFlowDrawParameters::staticInit()
{

    ReflectionNaming &nameing = naming();
    nameing = ReflectionNaming(
        "Mesh Flow Draw Parameters",
        "Mesh Flow Draw Parameters",
        ""
    );

     getReflection()->objectSize = sizeof(MeshFlowDrawParameters);
     

    IntField* field0 = new IntField
        (
          MeshFlowDrawParameters::GRID_ROWS_ID,
          offsetof(MeshFlowDrawParameters, mGridRows),
          16,
          "Grid Rows",
          "Grid Rows",
          "Grid Rows",
          true,
         0,
         1000,
         1
        );
    fields().push_back(field0);
    /*  */ 
    IntField* field1 = new IntField
        (
          MeshFlowDrawParameters::GRID_COLUMNS_ID,
          offsetof(MeshFlowDrawParameters, mGridColumns),
          16,
          "Grid Columns",
          "Grid Columns",
          "Grid Columns",
          true,
         0,
         1000,
         1
        );
    fields().push_back(field1);
    /*  */ 
    IntField* field2 = new IntField
        (
          MeshFlowDrawParameters::RANSAC_GRID_ROWS_ID,
          offsetof(MeshFlowDrawParameters, mRansacGridRows),
          4,
          "Ransac Grid Rows",
          "Ransac Grid Rows",
          "Ransac Grid Rows",
          true,
         0,
         1000,
         1
        );
    fields().push_back(field2);
    /*  */ 
    IntField* field3 = new IntField
        (
          MeshFlowDrawParameters::RANSAC_GRID_COLUMNS_ID,
          offsetof(MeshFlowDrawParameters, mRansacGridColumns),
          4,
          "Ransac Grid Columns",
          "Ransac Grid Columns",
          "Ransac Grid Columns",
          true,
         0,
         1000,
         1
        );
    fields().push_back(field3);
    /*  */ 
    IntField* field4 = new IntField
        (
          MeshFlowDrawParameters::MEDIAN_FILTER_SIZE_H_ID,
          offsetof(MeshFlowDrawParameters, mMedianFilterSizeH),
          5,
          "Median Filter Size H",
          "Median Filter Size H",
          "Median Filter Size H",
          true,
         0,
         1000,
         1
        );
    fields().push_back(field4);
    /*  */ 
    IntField* field5 = new IntField
        (
          MeshFlowDrawParameters::MEDIAN_FILTER_SIZE_W_ID,
          offsetof(MeshFlowDrawParameters, mMedianFilterSizeW),
          5,
          "Median Filter Size W",
          "Median Filter Size W",
          "Median Filter Size W",
          true,
         0,
         1000,
         1
        );
    fields().push_back(field5);
    /*  */ 
    IntField* field6 = new IntField
        (
          MeshFlowDrawParameters::MAX_FEATURE_NUMBER_ID,
          offsetof(MeshFlowDrawParameters, mMaxFeatureNumber),
          50,
          "Max Feature Number",
          "Max Feature Number",
          "Max Feature Number",
          true,
         0,
         10000,
         1
        );
    fields().push_back(field6);
    /*  */ 
    DoubleField* field7 = new DoubleField
        (
          MeshFlowDrawParameters::FEATURE_TRESHOLD_ID,
          offsetof(MeshFlowDrawParameters, mFeatureTreshold),
          10,
          "Feature Treshold",
          "Feature Treshold",
          "Feature Treshold",
          true,
         0,
         10000,
         0.1
        );
    field7->widgetHint=BaseField::SPIN_BOX;
    field7->precision=2;
    fields().push_back(field7);
    /*  */ 
    ReflectionDirectory &directory = *ReflectionDirectoryHolder::getReflectionDirectory();
    directory[std::string("Mesh Flow Draw Parameters")]= &reflection;
   return 0;
}
int MeshFlowDrawParameters::relinkCompositeFields()
{
   return 0;
}

SUPPRESS_OFFSET_WARNING_END


