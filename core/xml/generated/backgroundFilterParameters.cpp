/**
 * \file backgroundFilterParameters.cpp
 * \attention This file is automatically generated and should not be in general modified manually
 *
 * \date MMM DD, 20YY
 * \author autoGenerator
 */

#include <vector>
#include <stddef.h>
#include "backgroundFilterParameters.h"

/**
 *  Looks extremely unsafe because it depends on the order of static initialization.
 *  Should check standard if this is ok
 *
 *  Also it's not clear why removing "= Reflection()" breaks the code;
 **/

namespace corecvs {
template<>
Reflection BaseReflection<BackgroundFilterParameters>::reflection = Reflection();
template<>
int BaseReflection<BackgroundFilterParameters>::dummy = BackgroundFilterParameters::staticInit();
} // namespace corecvs 

SUPPRESS_OFFSET_WARNING_BEGIN


using namespace corecvs;

int BackgroundFilterParameters::staticInit()
{

    ReflectionNaming &nameing = naming();
    nameing = ReflectionNaming(
        "Background Filter Parameters",
        "Background Filter Parameters",
        ""
    );

     getReflection()->objectSize = sizeof(BackgroundFilterParameters);
     

    IntField* field0 = new IntField
        (
          BackgroundFilterParameters::THRESHOLD_ID,
          offsetof(BackgroundFilterParameters, mThreshold),
          100,
          "Threshold",
          "Threshold",
          "Threshold",
          true,
         0,
         10000,
         1
        );
    fields().push_back(field0);
    /*  */ 
    ReflectionDirectory &directory = *ReflectionDirectoryHolder::getReflectionDirectory();
    directory[std::string("Background Filter Parameters")]= &reflection;
   return 0;
}
int BackgroundFilterParameters::relinkCompositeFields()
{
   return 0;
}

SUPPRESS_OFFSET_WARNING_END


