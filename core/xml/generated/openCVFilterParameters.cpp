/**
 * \file openCVFilterParameters.cpp
 * \attention This file is automatically generated and should not be in general modified manually
 *
 * \date MMM DD, 20YY
 * \author autoGenerator
 */

#include <vector>
#include <stddef.h>
#include "openCVFilterParameters.h"

/**
 *  Looks extremely unsafe because it depends on the order of static initialization.
 *  Should check standard if this is ok
 *
 *  Also it's not clear why removing "= Reflection()" breaks the code;
 **/

namespace corecvs {
template<>
Reflection BaseReflection<OpenCVFilterParameters>::reflection = Reflection();
template<>
int BaseReflection<OpenCVFilterParameters>::dummy = OpenCVFilterParameters::staticInit();
} // namespace corecvs 

SUPPRESS_OFFSET_WARNING_BEGIN


using namespace corecvs;

int OpenCVFilterParameters::staticInit()
{

    ReflectionNaming &nameing = naming();
    nameing = ReflectionNaming(
        "OpenCV Filter Parameters",
        "OpenCV Filter Parameters",
        ""
    );

     getReflection()->objectSize = sizeof(OpenCVFilterParameters);
     

    EnumField* field0 = new EnumField
        (
          OpenCVFilterParameters::OPENCVFILTER_ID,
          offsetof(OpenCVFilterParameters, mOpenCVFilter),
          0,
          "OpenCVFilter",
          "OpenCVFilter",
          "OpenCVFilter",
          new EnumReflection(1
          , new EnumOption(0,"Canny")
          )
        );
    field0->widgetHint=BaseField::COMBO_BOX;
    fields().push_back(field0);
    /*  */ 
    IntField* field1 = new IntField
        (
          OpenCVFilterParameters::PARAM1_ID,
          offsetof(OpenCVFilterParameters, mParam1),
          50,
          "Param1",
          "Param1",
          "Param1",
          true,
         0,
         10000
        );
    fields().push_back(field1);
    /*  */ 
    IntField* field2 = new IntField
        (
          OpenCVFilterParameters::PARAM2_ID,
          offsetof(OpenCVFilterParameters, mParam2),
          100,
          "Param2",
          "Param2",
          "Param2",
          true,
         0,
         10000
        );
    fields().push_back(field2);
    /*  */ 
    ReflectionDirectory &directory = *ReflectionDirectoryHolder::getReflectionDirectory();
    directory[std::string("OpenCV Filter Parameters")]= &reflection;
   return 0;
}
int OpenCVFilterParameters::relinkCompositeFields()
{
   return 0;
}

SUPPRESS_OFFSET_WARNING_END


